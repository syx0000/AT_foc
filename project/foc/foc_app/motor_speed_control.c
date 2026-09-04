#include <stddef.h>
#include "at32f45x.h"
#include "motor_control_config.h"
#include "motor_current_control.h"
#include "motor_ramp.h"
#include "motor_speed_control.h"
#include "motor_speed_feedback.h"
#include "motor_speed_pi.h"

static motor_speed_control_config_t motor_speed_control_config;
static motor_speed_pi_t motor_speed_controller;
static motor_ramp_t motor_speed_ramp;
static volatile motor_speed_control_status_t motor_speed_control_status;

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
  return ((int64_t)speed_millirpm > 0) &&
         ((int64_t)speed_millirpm <= maximum_millirpm);
}

void motor_speed_control_init(void)
{
  motor_speed_control_config.proportional_gain_q20 = MOTOR_SPEED_PI_KP_Q20;
  motor_speed_control_config.integral_gain_q20 = MOTOR_SPEED_PI_KI_Q20;
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

  if ((motor_speed_control_status.state != MOTOR_SPEED_CONTROL_STOPPED) ||
      (!motor_speed_control_command_valid(target_speed_millirpm)) ||
      (!motor_current_control_status_read(&current_control)) ||
      (current_control.state != MOTOR_CURRENT_CONTROL_RUNNING) ||
      (!motor_speed_feedback_read(&speed_feedback)) ||
      (!speed_feedback.valid) ||
      (initial_quadrature_current_ma >
       motor_speed_control_config.current_limit_ma) ||
      (initial_quadrature_current_ma <
       -motor_speed_control_config.current_limit_ma))
  {
    return false;
  }

  (void)motor_speed_pi_output_seed(&motor_speed_controller,
                                   initial_quadrature_current_ma);
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
    initial_quadrature_current_ma;
  motor_speed_control_status.stall_time_ms = 0U;
  motor_speed_control_status.update_count = 0U;
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
}

void motor_speed_control_process_1khz(void)
{
  motor_speed_feedback_t speed_feedback;
  motor_current_control_status_t current_status;
  motor_current_control_command_t current_command;
  int32_t ramped_speed_millirpm;
  int32_t quadrature_current_ma;
  int32_t stall_current_threshold_ma;

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

  if (speed_feedback.direction <= 0)
  {
    motor_speed_control_fault_set(
      MOTOR_SPEED_CONTROL_FAULT_REVERSE_DIRECTION);
    return;
  }
  if (speed_feedback.filtered_speed_millirpm >
      (int32_t)(MOTOR_SPEED_OVERSPEED_RPM * 1000U))
  {
    motor_speed_control_fault_set(MOTOR_SPEED_CONTROL_FAULT_OVERSPEED);
    return;
  }

  stall_current_threshold_ma = (int32_t)(
    ((int64_t)motor_speed_control_config.current_limit_ma *
     MOTOR_SPEED_STALL_CURRENT_PERCENT) / 100U);
  if ((motor_speed_ramp.current >=
       (int32_t)(MOTOR_SPEED_STALL_MIN_TARGET_RPM * 1000U)) &&
      (speed_feedback.filtered_speed_millirpm <=
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
  current_command.direct_reference_ma = 0;
  current_command.quadrature_reference_ma = quadrature_current_ma;
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
