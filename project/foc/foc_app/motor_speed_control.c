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
static uint32_t motor_speed_control_last_feedback_sample_count;
static int32_t motor_speed_control_last_raw_speed_millirpm;
static bool motor_speed_control_feedback_decelerating;

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

static int32_t motor_speed_control_current_slew(int32_t target_ma,
                                                int32_t current_ma,
                                                bool fast_release_allowed)
{
  uint32_t slew_ma_per_s = MOTOR_SPEED_CONTROL_CURRENT_SLEW_MA_PER_S;
  int32_t step_ma;
  int64_t difference = (int64_t)target_ma - current_ma;

  /* Startup handover seeds the PI with the torque already being applied.
   * On a lightly loaded motor that torque can accelerate the rotor faster
   * than the speed ramp.  Permit torque magnitude reduction (or reversal)
   * to happen faster, while retaining the normal gentle torque build-up. */
  if ((fast_release_allowed &&
       (motor_speed_control_abs(target_ma) <
        motor_speed_control_abs(current_ma))) ||
      (((int64_t)target_ma * current_ma) <= 0))
  {
    slew_ma_per_s = MOTOR_SPEED_CONTROL_CURRENT_RELEASE_MA_PER_S;
  }
  step_ma = (int32_t)(slew_ma_per_s / MOTOR_SPEED_LOOP_FREQUENCY_HZ);
  if (step_ma < 1) step_ma = 1;
  if (difference > step_ma) return current_ma + step_ma;
  if (difference < -step_ma) return current_ma - step_ma;
  return target_ma;
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
    MOTOR_SPEED_CONTROL_CURRENT_LIMIT_MA;
  motor_speed_control_config.maximum_speed_rpm =
    MOTOR_SPEED_CONTROL_MAXIMUM_SPEED_RPM;
  motor_speed_control_config.acceleration_rpm_per_s =
    MOTOR_SPEED_CONTROL_ACCELERATION_RPM_PER_S;
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
  motor_speed_control_status.stall_time_ms = 0U;
  motor_speed_control_status.update_count = 0U;
  motor_speed_control_last_feedback_sample_count = 0U;
  motor_speed_control_last_raw_speed_millirpm = 0;
  motor_speed_control_feedback_decelerating = false;
}

bool motor_speed_control_parameter_reload(void)
{
  motor_parameter_t parameter;
  motor_speed_control_config_t config = motor_speed_control_config;

  if (!motor_parameter_active_read(&parameter)) return false;
  config.proportional_gain_q20 = parameter.speed_kp_q20;
  config.integral_gain_q20 = parameter.speed_ki_q20;
  return motor_speed_control_config_set(&config);
}

bool motor_speed_control_config_set(
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

bool motor_speed_control_start(int32_t target_speed_millirpm,
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
      (!speed_feedback.valid) ||
      (((int64_t)target_speed_millirpm *
        speed_feedback.filtered_speed_millirpm) <= 0) ||
      (initial_quadrature_current_ma >
       motor_speed_control_config.current_limit_ma) ||
      (initial_quadrature_current_ma <
       -motor_speed_control_config.current_limit_ma))
  {
    return false;
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
  motor_speed_control_status.state = MOTOR_SPEED_CONTROL_RUNNING;
  motor_speed_control_status.fault = MOTOR_SPEED_CONTROL_FAULT_NONE;
  motor_speed_control_status.target_speed_millirpm = target_speed_millirpm;
  motor_speed_control_status.ramped_speed_millirpm =
    speed_feedback.filtered_speed_millirpm;
  motor_speed_control_status.feedback_speed_millirpm =
    speed_feedback.filtered_speed_millirpm;
  motor_speed_control_status.quadrature_current_command_ma =
    logical_initial_current_ma;
  motor_speed_control_status.stall_time_ms = 0U;
  motor_speed_control_status.update_count = 0U;
  motor_speed_control_last_feedback_sample_count = speed_feedback.sample_count;
  motor_speed_control_last_raw_speed_millirpm =
    speed_feedback.raw_speed_millirpm;
  motor_speed_control_feedback_decelerating = false;
  return true;
}

bool motor_speed_control_target_set(int32_t target_speed_millirpm)
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
  motor_speed_control_status.target_speed_millirpm = target_speed_millirpm;
  return true;
}

void motor_speed_control_stop(void)
{
  motor_speed_pi_reset(&motor_speed_controller);
  (void)motor_ramp_init(&motor_speed_ramp, 0, 0,
    motor_speed_control_config.acceleration_rpm_per_s * 1000U);
  motor_speed_control_status.state = MOTOR_SPEED_CONTROL_STOPPED;
  motor_speed_control_status.fault = MOTOR_SPEED_CONTROL_FAULT_NONE;
  motor_speed_control_status.target_speed_millirpm = 0;
  motor_speed_control_status.ramped_speed_millirpm = 0;
  motor_speed_control_status.feedback_speed_millirpm = 0;
  motor_speed_control_status.quadrature_current_command_ma = 0;
  motor_speed_control_status.stall_time_ms = 0U;
  motor_speed_control_last_feedback_sample_count = 0U;
  motor_speed_control_last_raw_speed_millirpm = 0;
  motor_speed_control_feedback_decelerating = false;
}

void motor_speed_control_process_1khz(void)
{
  motor_speed_feedback_t speed_feedback;
  motor_current_control_status_t current_status;
  motor_current_control_command_t current_command;
  int32_t ramped_speed_millirpm;
  int32_t quadrature_current_ma;
  int32_t physical_quadrature_current_ma;
  int32_t stall_current_threshold_ma;
  bool fast_release_allowed;

  if (motor_speed_control_status.state != MOTOR_SPEED_CONTROL_RUNNING)
  {
    return;
  }

  if ((!motor_speed_feedback_read(&speed_feedback)) ||
      (!speed_feedback.valid))
  {
    motor_speed_control_fault_set(MOTOR_SPEED_CONTROL_FAULT_FEEDBACK);
    return;
  }
  if ((!motor_current_control_status_read(&current_status)) ||
      (current_status.state != MOTOR_CURRENT_CONTROL_RUNNING))
  {
    motor_speed_control_fault_set(
      MOTOR_SPEED_CONTROL_FAULT_CURRENT_CONTROL);
    return;
  }

  /* 目标跨零时由斜坡完成减速、过零和反向加速；斜坡尚未到达最终
   * 目标期间，反馈方向与最终目标暂时相反属于正常换向过程。 */
  if ((motor_speed_ramp.current ==
       motor_speed_control_status.target_speed_millirpm) &&
      (((motor_speed_control_status.target_speed_millirpm > 0) &&
        (speed_feedback.direction < 0)) ||
       ((motor_speed_control_status.target_speed_millirpm < 0) &&
        (speed_feedback.direction > 0))))
  {
    motor_speed_control_fault_set(
      MOTOR_SPEED_CONTROL_FAULT_REVERSE_DIRECTION);
    return;
  }
  if (motor_speed_control_abs(speed_feedback.filtered_speed_millirpm) >
      (int32_t)(MOTOR_SPEED_OVERSPEED_RPM * 1000U))
  {
    motor_speed_control_fault_set(MOTOR_SPEED_CONTROL_FAULT_OVERSPEED);
    return;
  }

  stall_current_threshold_ma = (int32_t)(
    ((int64_t)motor_speed_control_config.current_limit_ma *
     MOTOR_SPEED_STALL_CURRENT_PERCENT) / 100U);
  if ((motor_speed_control_abs(motor_speed_ramp.current) >=
       (int32_t)(MOTOR_SPEED_STALL_MIN_TARGET_RPM * 1000U)) &&
      (motor_speed_control_abs(speed_feedback.filtered_speed_millirpm) <=
       (int32_t)(MOTOR_SPEED_STALL_MAX_FEEDBACK_RPM * 1000U)) &&
      (motor_speed_control_abs(
         current_status.command.quadrature_reference_ma) >=
       stall_current_threshold_ma))
  {
    if (motor_speed_control_status.stall_time_ms <
        MOTOR_SPEED_STALL_TIMEOUT_MS)
    {
      motor_speed_control_status.stall_time_ms++;
    }
    if (motor_speed_control_status.stall_time_ms >=
        MOTOR_SPEED_STALL_TIMEOUT_MS)
    {
      motor_speed_control_fault_set(MOTOR_SPEED_CONTROL_FAULT_STALL);
      return;
    }
  }
  else
  {
    motor_speed_control_status.stall_time_ms = 0U;
  }

  ramped_speed_millirpm = motor_ramp_process(
    &motor_speed_ramp, MOTOR_SPEED_LOOP_FREQUENCY_HZ);
  quadrature_current_ma = motor_speed_pi_process(
    &motor_speed_controller, ramped_speed_millirpm,
    speed_feedback.filtered_speed_millirpm);

  if (speed_feedback.sample_count !=
      motor_speed_control_last_feedback_sample_count)
  {
    motor_speed_control_feedback_decelerating =
      (motor_speed_control_abs(speed_feedback.raw_speed_millirpm) +
       (int32_t)(MOTOR_SPEED_CONTROL_RELEASE_DECEL_MARGIN_RPM * 1000U) <
       motor_speed_control_abs(motor_speed_control_last_raw_speed_millirpm));
    motor_speed_control_last_feedback_sample_count =
      speed_feedback.sample_count;
    motor_speed_control_last_raw_speed_millirpm =
      speed_feedback.raw_speed_millirpm;
  }

  /* Hall speed changes in sector-sized steps.  During acceleration, a new
   * edge can briefly put feedback just ahead of the 1 rpm/ms reference and
   * make the PI request less torque.  Do not apply the aggressive release
   * rate for that quantisation error; reserve it for real overspeed. */
  fast_release_allowed =
    (!motor_speed_control_feedback_decelerating) &&
    (motor_speed_control_abs(speed_feedback.filtered_speed_millirpm) >
     (motor_speed_control_abs(ramped_speed_millirpm) +
      (int32_t)(MOTOR_SPEED_CONTROL_RELEASE_OVERSPEED_MARGIN_RPM * 1000U)));
  quadrature_current_ma = motor_speed_control_current_slew(
    quadrature_current_ma,
    motor_speed_control_status.quadrature_current_command_ma,
    fast_release_allowed);
  motor_speed_pi_output_track(&motor_speed_controller,
                              quadrature_current_ma);
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
