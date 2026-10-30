/**
 * @file    motor_speed_control.c
 * @brief   电机速度闭环控制器 功能实现
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#include <stddef.h>
#include "at32f45x.h"
#include "motor_control_config.h"
#include "motor_current_control.h"
#include "motor_direction.h"
#include "motor_parameter.h"
#include "motor_ramp.h"
#include "motor_speed_control.h"
#include "motor_speed_feedback.h"
#include "motor_speed_pi.h"

static motor_speed_control_config_t motor_speed_control_config;
static motor_speed_pi_t motor_speed_controller;
static motor_ramp_t motor_speed_ramp;
static volatile motor_speed_control_status_t motor_speed_control_status;
static int32_t motor_speed_control_pi_target_ma;
static int8_t motor_speed_run_direction;
static uint32_t motor_speed_reverse_zero_ms;
static bool motor_speed_startup_slew_active;
static uint32_t motor_speed_startup_stable_ms;
static uint32_t motor_speed_slew_remainder;
static bool motor_speed_control_config_set(  const motor_speed_control_config_t *config);

/**
 * @brief 锁存速度控制故障并安全停止底层电流环。
 * @param fault 非NONE的速度控制故障原因。
 * @return 无。
 * @details 先记录故障状态，再调用电流控制停止接口关闭PWM；故障由统一motor_control轮询上报。
 */
static void motor_speed_control_fault_set(
  motor_speed_control_fault_t fault)
{
  motor_speed_control_status.fault = fault;
  motor_speed_control_status.state = MOTOR_SPEED_CONTROL_FAULT;
  motor_current_control_stop();
}

/**
 * @brief 计算32位有符号数绝对值。
 * @param value 输入值；本模块调用值均受配置限幅，不会传入INT32_MIN。
 * @return 输入值的非负绝对值。
 */
static int32_t motor_speed_control_abs(int32_t value)
{
  return (value < 0) ? -value : value;
}

/**
 * @brief 判断有符号milli-rpm指令是否在配置范围内。
 * @param speed_millirpm 待检查机械转速，单位0.001 rpm。
 * @return 在正负最高转速范围内返回true，否则返回false。
 */
static bool motor_speed_control_command_valid(int32_t speed_millirpm)
{
  int64_t maximum_millirpm =
    (int64_t)motor_speed_control_config.maximum_speed_rpm * 1000L;
  return ((int64_t)speed_millirpm != 0) &&
         ((int64_t)speed_millirpm <= maximum_millirpm) &&
         ((int64_t)speed_millirpm >= -maximum_millirpm);
}

void motor_speed_control_init(void)
{
  motor_parameter_t parameter;

  (void)motor_parameter_active_read(&parameter);
  motor_speed_control_config.proportional_gain_q20 = parameter.speed_kp_q20;
  motor_speed_control_config.integral_gain_q20 = parameter.speed_ki_q20;
  motor_speed_control_config.current_limit_ma =
    (int32_t)parameter.speed_current_limit_ma;
  motor_speed_control_config.maximum_speed_rpm =
    (int32_t)parameter.speed_maximum_rpm;
  motor_speed_control_config.acceleration_rpm_per_s =
    parameter.speed_acceleration_rpm_per_s;
  motor_speed_control_config.deceleration_rpm_per_s = parameter.speed_deceleration_rpm_per_s;
  (void)motor_speed_pi_init(
    &motor_speed_controller,
    motor_speed_control_config.proportional_gain_q20,
    motor_speed_control_config.integral_gain_q20,
    motor_speed_control_config.current_limit_ma);
  (void)motor_ramp_init(&motor_speed_ramp, 0, 0,
    motor_speed_control_config.acceleration_rpm_per_s * 1000U);
  motor_speed_control_status.state = MOTOR_SPEED_CONTROL_STOPPED;
  motor_speed_control_status.fault = MOTOR_SPEED_CONTROL_FAULT_NONE;
  motor_speed_control_status.target_speed_millirpm = 0;
  motor_speed_control_status.ramped_speed_millirpm = 0;
  motor_speed_control_status.feedback_speed_millirpm = 0;
  motor_speed_control_status.quadrature_current_command_ma = 0;
  motor_speed_control_status.update_count = 0U;
  motor_speed_control_pi_target_ma = 0;
}

bool motor_speed_control_parameter_reload(void)
{
  motor_parameter_t parameter;
  motor_speed_control_config_t config = motor_speed_control_config;

  if (!motor_parameter_active_read(&parameter)) return false;
  config.proportional_gain_q20 = parameter.speed_kp_q20;
  config.integral_gain_q20 = parameter.speed_ki_q20;
  config.maximum_speed_rpm = (int32_t)parameter.speed_maximum_rpm;
  config.acceleration_rpm_per_s = parameter.speed_acceleration_rpm_per_s;
  config.deceleration_rpm_per_s = parameter.speed_deceleration_rpm_per_s;
  config.current_limit_ma = (int32_t)parameter.speed_current_limit_ma;
  return motor_speed_control_config_set(&config);
}

static bool motor_speed_control_config_set(
  const motor_speed_control_config_t *config)
{
  if ((config == NULL) ||
      (motor_speed_control_status.state != MOTOR_SPEED_CONTROL_STOPPED) ||
      (config->proportional_gain_q20 < 0) ||
      (config->integral_gain_q20 < 0) ||
      (config->current_limit_ma <= 0) ||
      (config->current_limit_ma > MOTOR_CURRENT_COMMAND_MAX_MA) ||
      (config->maximum_speed_rpm <= 0) ||
      (config->acceleration_rpm_per_s == 0U) ||
      (config->deceleration_rpm_per_s == 0U) ||
      (config->deceleration_rpm_per_s > (UINT32_MAX / 1000U)) ||
      (config->acceleration_rpm_per_s > (UINT32_MAX / 1000U)))
  {
    return false;
  }

  motor_speed_control_config = *config;
  (void)motor_speed_pi_init(
    &motor_speed_controller,
    config->proportional_gain_q20,
    config->integral_gain_q20,
    config->current_limit_ma);
  (void)motor_ramp_init(&motor_speed_ramp, 0, 0,
                        config->acceleration_rpm_per_s * 1000U);
  return true;
}

static bool motor_speed_control_start_locked(int32_t target_speed_millirpm,
                               int32_t initial_quadrature_current_ma)
{
  motor_current_control_status_t current_control;
  motor_speed_feedback_t speed_feedback;
  int32_t logical_initial_current_ma;

  if ((motor_speed_control_status.state != MOTOR_SPEED_CONTROL_STOPPED) ||
      (!motor_speed_control_command_valid(target_speed_millirpm)) ||
      (!motor_current_control_status_read(&current_control)) ||
      (current_control.state != MOTOR_CURRENT_CONTROL_RUNNING) ||
      (!motor_speed_feedback_read(&speed_feedback)) ||
      (initial_quadrature_current_ma >
       motor_speed_control_config.current_limit_ma) ||
      (initial_quadrature_current_ma <
       -motor_speed_control_config.current_limit_ma))
  {
    return false;
  }
  /* 静止或反馈尚未建立时按0 rpm启动；只拒绝明确反向旋转。 */
  if (!speed_feedback.valid)
  {
    speed_feedback.filtered_speed_millirpm = 0;
    speed_feedback.raw_speed_millirpm = 0;
  }

  if (!motor_direction_transform_s32(
        initial_quadrature_current_ma,
        motor_parameter_direction_inverted_get(),
        &logical_initial_current_ma))
  {
    return false;
  }
  (void)motor_speed_pi_output_seed(&motor_speed_controller,
                                   logical_initial_current_ma);
  (void)motor_ramp_init(
    &motor_speed_ramp,
    speed_feedback.filtered_speed_millirpm,
    target_speed_millirpm,
    motor_speed_control_config.acceleration_rpm_per_s * 1000U);
  motor_speed_startup_slew_active = true;
  motor_speed_startup_stable_ms = 0U;
  motor_speed_slew_remainder = 0U;
  motor_speed_run_direction = speed_feedback.filtered_speed_millirpm != 0 ?
    (speed_feedback.filtered_speed_millirpm < 0 ? -1 : 1) :
    (target_speed_millirpm < 0 ? -1 : 1);
  motor_speed_reverse_zero_ms = 0U;
  motor_speed_control_status.fault = MOTOR_SPEED_CONTROL_FAULT_NONE;
  motor_speed_control_status.target_speed_millirpm = target_speed_millirpm;
  motor_speed_control_status.ramped_speed_millirpm =
    speed_feedback.filtered_speed_millirpm;
  motor_speed_control_status.feedback_speed_millirpm =
    speed_feedback.filtered_speed_millirpm;
  motor_speed_control_status.quadrature_current_command_ma =
    logical_initial_current_ma;
  motor_speed_control_status.update_count = 0U;
  motor_speed_control_pi_target_ma = logical_initial_current_ma;
  motor_speed_control_status.state = MOTOR_SPEED_CONTROL_RUNNING;
  return true;
}

bool motor_speed_control_start(int32_t target_speed_millirpm,
                               int32_t initial_quadrature_current_ma)
{
  uint32_t primask = __get_PRIMASK();
  bool ok;
  __disable_irq();
  ok = motor_speed_control_start_locked(target_speed_millirpm, initial_quadrature_current_ma);
  __set_PRIMASK(primask);
  return ok;
}

static bool motor_speed_control_target_set_locked(int32_t target_speed_millirpm)
{
  if ((motor_speed_control_status.state != MOTOR_SPEED_CONTROL_RUNNING) ||
      (!motor_speed_control_command_valid(target_speed_millirpm)))
  {
    return false;
  }

  if (!motor_ramp_target_set(
        &motor_speed_ramp,
        target_speed_millirpm,
        motor_speed_control_config.acceleration_rpm_per_s * 1000U))
  {
    return false;
  }
  if (((int64_t)motor_speed_control_status.target_speed_millirpm *
       target_speed_millirpm) < 0) motor_speed_reverse_zero_ms = 0U;
  motor_speed_control_status.target_speed_millirpm = target_speed_millirpm;
  return true;
}

bool motor_speed_control_target_set(int32_t target_speed_millirpm)
{
  uint32_t primask = __get_PRIMASK();
  bool ok;
  __disable_irq();
  ok = motor_speed_control_target_set_locked(target_speed_millirpm);
  __set_PRIMASK(primask);
  return ok;
}

void motor_speed_control_stop(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  motor_speed_control_status.state = MOTOR_SPEED_CONTROL_STOPPED;
  motor_speed_pi_reset(&motor_speed_controller);
  (void)motor_ramp_init(&motor_speed_ramp, 0, 0,
    motor_speed_control_config.acceleration_rpm_per_s * 1000U);
  motor_speed_control_status.state = MOTOR_SPEED_CONTROL_STOPPED;
  motor_speed_startup_slew_active = false;
  motor_speed_startup_stable_ms = 0U;
  motor_speed_slew_remainder = 0U;
  motor_speed_run_direction = 0;
  motor_speed_reverse_zero_ms = 0U;
  motor_speed_control_status.fault = MOTOR_SPEED_CONTROL_FAULT_NONE;
  motor_speed_control_status.target_speed_millirpm = 0;
  motor_speed_control_status.ramped_speed_millirpm = 0;
  motor_speed_control_status.feedback_speed_millirpm = 0;
  motor_speed_control_status.quadrature_current_command_ma = 0;
  motor_speed_control_pi_target_ma = 0;
  __set_PRIMASK(primask);
}

void motor_speed_control_process_1khz(void)
{
  motor_speed_feedback_t speed_feedback;
  motor_current_control_status_t current_status;
  motor_current_control_command_t current_command;
  int32_t ramped_speed_millirpm;
  int32_t quadrature_current_ma;
  int32_t physical_quadrature_current_ma;
  int32_t ramp_target;
  uint32_t ramp_rate;
  int32_t previous_integral_ma;
  int32_t previous_integral_remainder;
  int32_t previous_iq;
  int32_t slew_step;
  int32_t speed_tolerance;

  if (motor_speed_control_status.state != MOTOR_SPEED_CONTROL_RUNNING)
  {
    return;
  }

  /* 反馈无效(静止/低于测速门限)按0 rpm参与PI，Iq由PI积分建立。 */
  if (!motor_speed_feedback_read(&speed_feedback)) return;
  if (!speed_feedback.valid)
  {
    speed_feedback.filtered_speed_millirpm = 0;
    speed_feedback.raw_speed_millirpm = 0;
  }
  if ((!motor_current_control_status_read(&current_status)) ||
      (current_status.state != MOTOR_CURRENT_CONTROL_RUNNING))
  {
    motor_speed_control_fault_set(
      MOTOR_SPEED_CONTROL_FAULT_CURRENT_CONTROL);
    return;
  }

  if (motor_speed_control_abs(speed_feedback.filtered_speed_millirpm) >
      (int32_t)(MOTOR_SPEED_OVERSPEED_RPM * 1000U))
  {
    motor_speed_control_fault_set(MOTOR_SPEED_CONTROL_FAULT_OVERSPEED);
    return;
  }


  ramp_target = motor_speed_control_status.target_speed_millirpm;
  if (((int64_t)ramp_target * motor_speed_run_direction) < 0)
  {
    /* Decelerate the command to zero, then confirm measured standstill. */
    if ((motor_speed_ramp.current == 0) &&
        ((speed_feedback.valid &&
          motor_speed_control_abs(speed_feedback.filtered_speed_millirpm) <= 30000) ||
         (!speed_feedback.valid && motor_speed_feedback_standstill_confirmed())))
      motor_speed_reverse_zero_ms++;
    else motor_speed_reverse_zero_ms = 0U;
    if (motor_speed_reverse_zero_ms >= 100U)
    {
      motor_speed_run_direction = ramp_target < 0 ? -1 : 1;
      motor_speed_reverse_zero_ms = 0U;
      motor_speed_pi_reset(&motor_speed_controller);
      motor_speed_startup_slew_active = true;
      motor_speed_startup_stable_ms = 0U;
      motor_speed_slew_remainder = 0U;
    }
    else ramp_target = 0;
  }
  else motor_speed_reverse_zero_ms = 0U;
  ramp_rate = motor_speed_control_abs(ramp_target) <
              motor_speed_control_abs(motor_speed_ramp.current) ?
    motor_speed_control_config.deceleration_rpm_per_s :
    motor_speed_control_config.acceleration_rpm_per_s;
  /* Do not run the speed reference ahead while torque is constrained.
   * Deceleration/reversal always retains its requested ramp. */
  if ((((int64_t)ramp_target * motor_speed_ramp.current) >= 0) &&
      (motor_speed_control_abs(ramp_target) > motor_speed_control_abs(motor_speed_ramp.current)) &&
      ((motor_speed_control_config.current_limit_ma == 0) ||
       (motor_speed_control_abs(motor_speed_control_status.quadrature_current_command_ma) >=
        motor_speed_control_config.current_limit_ma) ||
       (motor_speed_startup_slew_active && motor_speed_control_pi_target_ma !=
        motor_speed_control_status.quadrature_current_command_ma)) &&
      ((motor_speed_control_config.current_limit_ma == 0) ||
       (motor_speed_control_abs(motor_speed_ramp.current -
         speed_feedback.filtered_speed_millirpm) >= 30000)))
    ramp_target = motor_speed_ramp.current;
  (void)motor_ramp_target_set(&motor_speed_ramp, ramp_target, ramp_rate * 1000U);
  ramped_speed_millirpm = motor_ramp_process(
    &motor_speed_ramp, MOTOR_SPEED_LOOP_FREQUENCY_HZ);
  previous_integral_ma = motor_speed_controller.integral_ma;
  previous_integral_remainder = motor_speed_controller.integral_remainder_q20;
  motor_speed_control_pi_target_ma = motor_speed_pi_process(
    &motor_speed_controller, ramped_speed_millirpm,
    speed_feedback.filtered_speed_millirpm);

  quadrature_current_ma = motor_speed_control_pi_target_ma;
  if (motor_speed_startup_slew_active)
  {
    previous_iq = motor_speed_control_status.quadrature_current_command_ma;
    motor_speed_slew_remainder += motor_parameter_debug_get(
      MOTOR_PARAMETER_FIELD_SPEED_STARTUP_IQ_SLEW_MA_S);
    slew_step = (int32_t)(motor_speed_slew_remainder / MOTOR_SPEED_LOOP_FREQUENCY_HZ);
    motor_speed_slew_remainder %= MOTOR_SPEED_LOOP_FREQUENCY_HZ;
    /* Torque reduction is immediate. A sign change first unloads to zero. */
    if (((int64_t)previous_iq * quadrature_current_ma) < 0)
      quadrature_current_ma = 0;
    else if (quadrature_current_ma > motor_speed_control_abs(previous_iq) + slew_step)
      quadrature_current_ma = motor_speed_control_abs(previous_iq) + slew_step;
    else if (quadrature_current_ma < -motor_speed_control_abs(previous_iq) - slew_step)
      quadrature_current_ma = -motor_speed_control_abs(previous_iq) - slew_step;
    if (quadrature_current_ma != motor_speed_control_pi_target_ma)
    {
      /* Freeze integration while the requested torque cannot yet be applied. */
      motor_speed_controller.integral_ma = previous_integral_ma;
      motor_speed_controller.integral_remainder_q20 = previous_integral_remainder;
      motor_speed_controller.output_ma = quadrature_current_ma;
    }
    speed_tolerance = motor_speed_control_abs(
      motor_speed_control_status.target_speed_millirpm) / 10;
    if (speed_tolerance < 30000) speed_tolerance = 30000;
    if (speed_feedback.valid &&
        (((int64_t)speed_feedback.filtered_speed_millirpm *
          motor_speed_control_status.target_speed_millirpm) > 0) &&
        (ramped_speed_millirpm == motor_speed_control_status.target_speed_millirpm) &&
        (motor_speed_control_abs(speed_feedback.filtered_speed_millirpm -
          ramped_speed_millirpm) <= speed_tolerance) &&
        (quadrature_current_ma == motor_speed_control_pi_target_ma))
      motor_speed_startup_stable_ms++;
    else motor_speed_startup_stable_ms = 0U;
    if (motor_speed_startup_stable_ms >= 100U)
      motor_speed_startup_slew_active = false;
  }
  /* Zero speed under load is allowed: keep producing the limited Iq command. */
  current_command.direct_reference_ma = 0;
  if (!motor_direction_transform_s32(
        quadrature_current_ma,
        motor_parameter_direction_inverted_get(),
        &physical_quadrature_current_ma))
  {
    motor_speed_control_fault_set(MOTOR_SPEED_CONTROL_FAULT_COMMAND);
    return;
  }
  current_command.quadrature_reference_ma = physical_quadrature_current_ma;
  if (!motor_current_control_command_set(&current_command))
  {
    motor_speed_control_fault_set(
      MOTOR_SPEED_CONTROL_FAULT_CURRENT_CONTROL);
    return;
  }

  motor_speed_control_status.ramped_speed_millirpm =
    ramped_speed_millirpm;
  motor_speed_control_status.feedback_speed_millirpm =
    speed_feedback.filtered_speed_millirpm;
  motor_speed_control_status.quadrature_current_command_ma =
    quadrature_current_ma;
  motor_speed_control_status.update_count++;
}

bool motor_speed_control_status_read(motor_speed_control_status_t *status)
{
  uint32_t primask;

  if (status == NULL)
  {
    return false;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  *status = motor_speed_control_status;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return true;
}

int32_t motor_speed_control_maximum_rpm_get(void)
{
  return motor_speed_control_config.maximum_speed_rpm;
}

bool motor_speed_control_current_limit_set(int32_t limit_ma)
{
  uint32_t primask;
  motor_current_control_status_t current;

  if (limit_ma < 0) return false;
  if (limit_ma > MOTOR_SPEED_CONTROL_CURRENT_LIMIT_MA)
    limit_ma = MOTOR_SPEED_CONTROL_CURRENT_LIMIT_MA;
  /* 速度PI在SysTick中运行：限幅与积分钳位需原子更新。 */
  primask = __get_PRIMASK();
  __disable_irq();
  if ((motor_speed_control_status.state == MOTOR_SPEED_CONTROL_RUNNING) &&
      (limit_ma > motor_speed_control_config.current_limit_ma))
  {
    motor_speed_startup_slew_active = true;
    motor_speed_startup_stable_ms = 0U;
    motor_speed_slew_remainder = 0U;
  }
  motor_speed_control_config.current_limit_ma = limit_ma;
  motor_speed_controller.output_limit_ma = limit_ma;
  if (motor_speed_controller.integral_ma > limit_ma)
    motor_speed_controller.integral_ma = limit_ma;
  else if (motor_speed_controller.integral_ma < -limit_ma)
    motor_speed_controller.integral_ma = -limit_ma;
  if (motor_speed_controller.output_ma > limit_ma)
    motor_speed_controller.output_ma = limit_ma;
  else if (motor_speed_controller.output_ma < -limit_ma)
    motor_speed_controller.output_ma = -limit_ma;
  if (limit_ma == 0) motor_speed_controller.integral_remainder_q20 = 0;
  if (motor_speed_control_status.state == MOTOR_SPEED_CONTROL_RUNNING)
  {
    if (motor_speed_control_status.quadrature_current_command_ma > limit_ma)
      motor_speed_control_status.quadrature_current_command_ma = limit_ma;
    else if (motor_speed_control_status.quadrature_current_command_ma < -limit_ma)
      motor_speed_control_status.quadrature_current_command_ma = -limit_ma;
    motor_speed_control_pi_target_ma =
      motor_speed_control_status.quadrature_current_command_ma;
    if (motor_current_control_status_read(&current) &&
        (current.state == MOTOR_CURRENT_CONTROL_RUNNING))
    {
      if (current.command.quadrature_reference_ma > limit_ma)
        current.command.quadrature_reference_ma = limit_ma;
      else if (current.command.quadrature_reference_ma < -limit_ma)
        current.command.quadrature_reference_ma = -limit_ma;
      (void)motor_current_control_command_set(&current.command);
    }
  }
  __set_PRIMASK(primask);
  return true;
}
