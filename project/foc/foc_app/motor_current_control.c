/**
 * @file    motor_current_control.c
 * @brief   FOC电流闭环控制器 功能实现
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#include <stddef.h>
#include "at32f45x.h"
#include "motor_board_config.h"
#include "motor_control_config.h"
#include "motor_current_control.h"
#include "motor_current_pi.h"
#include "motor_current_sample.h"
#include "motor_foc_math.h"
#include "motor_rotor_feedback.h"
#include "motor_parameter.h"
#include "motor_pwm_port.h"
#include "motor_slow_sensor.h"
#include "motor_voltage_limit.h"

static volatile motor_current_control_status_t motor_current_control_status;
static volatile motor_current_control_command_t motor_current_control_command;
static motor_current_control_config_t motor_current_control_config;
static motor_current_pi_axis_t motor_current_control_d_pi, motor_current_control_q_pi;
static uint32_t motor_current_control_bus_mv;
static int32_t motor_current_control_voltage_limit_mv;
static uint32_t motor_current_control_sample_settle_cycles;
static uint32_t motor_current_control_hall_invalid_cycles;
static int32_t motor_current_control_start_direct_mv;
static int32_t motor_current_control_start_quadrature_mv;

static void motor_current_control_bus_voltage_update(void)
{
  motor_slow_sensor_state_t sensor;
  uint32_t bus_mv;
  int32_t voltage_limit_mv;

  if ((!motor_slow_sensor_state_read(&sensor)) ||
      (sensor.bus_voltage_0p1v < MOTOR_BUS_UNDERVOLTAGE_0P1V) ||
      (sensor.bus_voltage_0p1v > MOTOR_BUS_OVERVOLTAGE_0P1V))
  {
    return;
  }
  bus_mv = (uint32_t)sensor.bus_voltage_0p1v * 100U;
  voltage_limit_mv = (int32_t)(
    (bus_mv * motor_current_control_config.voltage_limit_percent) / 100U);
  if (voltage_limit_mv > motor_current_control_config.hard_voltage_limit_mv)
  {
    voltage_limit_mv = motor_current_control_config.hard_voltage_limit_mv;
  }
  if ((bus_mv == motor_current_control_bus_mv) &&
      (voltage_limit_mv == motor_current_control_voltage_limit_mv))
  {
    return;
  }
  motor_current_control_bus_mv = bus_mv;
  motor_current_control_voltage_limit_mv = voltage_limit_mv;
  (void)motor_current_pi_axis_output_limit_set(
    &motor_current_control_d_pi, voltage_limit_mv);
  (void)motor_current_pi_axis_output_limit_set(
    &motor_current_control_q_pi, voltage_limit_mv);
}

static int32_t motor_current_control_abs(int32_t value)
{
  return (value < 0) ? -value : value;
}

static bool motor_current_control_command_valid(
  const motor_current_control_command_t *command)
{
  return (command != NULL) &&
    (motor_current_control_abs(command->direct_reference_ma) <=
     motor_current_control_config.command_limit_ma) &&
    (motor_current_control_abs(command->quadrature_reference_ma) <=
     motor_current_control_config.command_limit_ma);
}

static bool motor_current_control_config_valid(
  const motor_current_control_config_t *config)
{
  return (config != NULL) && (config->direct_kp_q15 >= 0) &&
    (config->quadrature_kp_q15 >= 0) && (config->integral_gain_q15 >= 0) &&
    (config->voltage_limit_percent > 0U) &&
    (config->voltage_limit_percent <= 57U) &&
    (config->hard_voltage_limit_mv > 0) &&
    (config->command_limit_ma > 0) &&
    (config->abort_current_ma >= config->command_limit_ma) &&
    (config->abort_current_ma <= MOTOR_SOFTWARE_OVERCURRENT_MA);
}

static motor_alpha_beta_q15_t motor_current_control_voltage_apply(
  int32_t direct_mv, int32_t quadrature_mv, uint16_t angle_u16)
{
  motor_alpha_beta_q15_t voltage = {0, 0};
  int16_t direct_q15;
  int16_t quadrature_q15;

  /* prepare() 尚未成功时 bus_mv 为零，直接返回零向量并跳过PWM写入，
   * 让上一次 compare 保持不变，避免除零UB和无根据的输出。 */
  if (motor_current_control_bus_mv == 0U) return voltage;
  direct_q15 = (int16_t)(((int64_t)direct_mv * 32768) / motor_current_control_bus_mv);
  quadrature_q15 = (int16_t)(((int64_t)quadrature_mv * 32768) /
                                     motor_current_control_bus_mv);
  voltage = motor_foc_inverse_park_q15(
    direct_q15, quadrature_q15, angle_u16);
  motor_svpwm_duty_q15_t duty = motor_foc_svpwm_q15(voltage);
  motor_pwm_compare_t compare;
  compare.phase_a = (uint16_t)(((uint32_t)duty.phase_a *
                                MOTOR_PWM_PERIOD_COUNTS) / 32767U);
  compare.phase_b = (uint16_t)(((uint32_t)duty.phase_b *
                                MOTOR_PWM_PERIOD_COUNTS) / 32767U);
  compare.phase_c = (uint16_t)(((uint32_t)duty.phase_c *
                                 MOTOR_PWM_PERIOD_COUNTS) / 32767U);
  motor_pwm_port_compare_set(&compare);
  return voltage;
}

static bool motor_current_control_prepare(
  const motor_current_control_command_t *command,
  int32_t direct_seed_mv, int32_t quadrature_seed_mv)
{
  motor_slow_sensor_state_t sensor;
  motor_rotor_feedback_t rotor_feedback;
  (void)motor_slow_sensor_process();
  if ((!motor_current_control_command_valid(command)) ||
      (!motor_slow_sensor_state_read(&sensor)) ||
      (!motor_rotor_feedback_read(&rotor_feedback)) ||
      (!rotor_feedback.valid) ||
      (sensor.bus_voltage_0p1v < MOTOR_BUS_UNDERVOLTAGE_0P1V) ||
      (sensor.bus_voltage_0p1v > MOTOR_BUS_OVERVOLTAGE_0P1V))
  {
    return false;
  }
  motor_current_control_bus_mv = (uint32_t)sensor.bus_voltage_0p1v * 100U;
  motor_current_control_voltage_limit_mv = (int32_t)(
    (motor_current_control_bus_mv * motor_current_control_config.voltage_limit_percent) / 100U);
  if (motor_current_control_voltage_limit_mv > motor_current_control_config.hard_voltage_limit_mv)
  {
    motor_current_control_voltage_limit_mv = motor_current_control_config.hard_voltage_limit_mv;
  }
  motor_current_pi_axis_init(&motor_current_control_d_pi, motor_current_control_config.direct_kp_q15,
                             motor_current_control_config.integral_gain_q15,
                             motor_current_control_voltage_limit_mv);
  motor_current_pi_axis_init(&motor_current_control_q_pi, motor_current_control_config.quadrature_kp_q15,
                             motor_current_control_config.integral_gain_q15,
                             motor_current_control_voltage_limit_mv);
  (void)motor_current_pi_axis_output_seed(&motor_current_control_d_pi, direct_seed_mv);
  (void)motor_current_pi_axis_output_seed(&motor_current_control_q_pi, quadrature_seed_mv);
  motor_current_control_command = *command;
  /* The startup state machine reads the published command before the first
     settled ADC frame.  Publish the new seed command here so a restart cannot
     ramp from the previous run's quadrature-current reference. */
  motor_current_control_status.command = *command;
  motor_current_control_status.fault = MOTOR_CURRENT_CONTROL_FAULT_NONE;
  motor_current_control_status.sample_count = 0U;
  motor_current_control_sample_settle_cycles = 0U;
  motor_current_control_hall_invalid_cycles = 0U;
  return true;
}

void motor_current_control_init(void)
{
  motor_parameter_t parameter;

  (void)motor_parameter_active_read(&parameter);
  motor_current_control_config.direct_kp_q15 = parameter.current_d_kp_q15;
  motor_current_control_config.quadrature_kp_q15 = parameter.current_q_kp_q15;
  motor_current_control_config.integral_gain_q15 = parameter.current_ki_q15;
  motor_current_control_config.voltage_limit_percent =
    MOTOR_CURRENT_CONTROL_VOLTAGE_LIMIT_PERCENT;
  motor_current_control_config.hard_voltage_limit_mv =
    MOTOR_CURRENT_CONTROL_HARD_VOLTAGE_LIMIT_MV;
  motor_current_control_config.command_limit_ma = MOTOR_CURRENT_COMMAND_MAX_MA;
  motor_current_control_config.abort_current_ma = MOTOR_CURRENT_CONTROL_ABORT_CURRENT_MA;
  motor_current_control_status.state = MOTOR_CURRENT_CONTROL_STOPPED;
  motor_current_control_status.fault = MOTOR_CURRENT_CONTROL_FAULT_NONE;
  motor_current_control_status.sample_count = 0U;
  motor_current_control_hall_invalid_cycles = 0U;
}

bool motor_current_control_parameter_reload(void)
{
  motor_parameter_t parameter;
  motor_current_control_config_t config = motor_current_control_config;

  if (!motor_parameter_active_read(&parameter)) return false;
  config.direct_kp_q15 = parameter.current_d_kp_q15;
  config.quadrature_kp_q15 = parameter.current_q_kp_q15;
  config.integral_gain_q15 = parameter.current_ki_q15;
  /* 参数/辨识切换结束后必须恢复正式运行限制，不能沿用低功率测试配置。 */
  config.voltage_limit_percent = MOTOR_CURRENT_CONTROL_VOLTAGE_LIMIT_PERCENT;
  config.hard_voltage_limit_mv = MOTOR_CURRENT_CONTROL_HARD_VOLTAGE_LIMIT_MV;
  config.command_limit_ma = MOTOR_CURRENT_COMMAND_MAX_MA;
  config.abort_current_ma = MOTOR_CURRENT_CONTROL_ABORT_CURRENT_MA;
  return motor_current_control_config_set(&config);
}

bool motor_current_control_config_set(const motor_current_control_config_t *config)
{
  if ((!motor_current_control_config_valid(config)) ||
      (motor_current_control_status.state != MOTOR_CURRENT_CONTROL_STOPPED))
  {
    return false;
  }
  motor_current_control_config = *config;
  return true;
}

bool motor_current_control_start(const motor_current_control_command_t *command)
{
  return motor_current_control_start_seeded(command, 0, 0, 0U);
}

bool motor_current_control_start_seeded(
  const motor_current_control_command_t *command,
  int32_t direct_voltage_seed_mv,
  int32_t quadrature_voltage_seed_mv,
  uint32_t sample_settle_cycles)
{
  motor_rotor_feedback_t rotor_feedback;
  if (motor_pwm_port_output_is_enabled() ||
      (!motor_current_control_prepare(command, direct_voltage_seed_mv,
                                      quadrature_voltage_seed_mv)) ||
      (!motor_rotor_feedback_read(&rotor_feedback)))
  {
    return false;
  }
  motor_current_control_voltage_apply(
    direct_voltage_seed_mv, quadrature_voltage_seed_mv,
    rotor_feedback.electrical_angle_u16);
  motor_current_control_start_direct_mv = direct_voltage_seed_mv;
  motor_current_control_start_quadrature_mv = quadrature_voltage_seed_mv;
  motor_current_control_sample_settle_cycles = sample_settle_cycles;
  if (!motor_pwm_port_output_enable())
  {
    motor_current_control_sample_settle_cycles = 0U;
    motor_current_control_status.state = MOTOR_CURRENT_CONTROL_FAULT;
    motor_current_control_status.fault = MOTOR_CURRENT_CONTROL_FAULT_PWM;
    return false;
  }
  /* Publish RUNNING only after the initial vector, settle guard and MOE are
     all live; otherwise the ADC ISR can execute the controller in the
     prepare-to-enable window and immediately fault on MOE=0. */
  motor_current_control_status.state = MOTOR_CURRENT_CONTROL_RUNNING;
  return true;
}

bool motor_current_control_handover(
  const motor_current_control_command_t *command,
  int32_t direct_voltage_seed_mv, int32_t quadrature_voltage_seed_mv)
{
  if ((!motor_pwm_port_output_is_enabled()) ||
      (!motor_current_control_prepare(command, direct_voltage_seed_mv,
                                      quadrature_voltage_seed_mv)))
  {
    return false;
  }
  motor_current_control_status.state = MOTOR_CURRENT_CONTROL_RUNNING;
  return true;
}

bool motor_current_control_command_set(
  const motor_current_control_command_t *command)
{
  uint32_t primask;
  if ((!motor_current_control_command_valid(command)) ||
      (motor_current_control_status.state != MOTOR_CURRENT_CONTROL_RUNNING))
  {
    return false;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  motor_current_control_command = *command;
  motor_current_control_status.command = *command;
  if (primask == 0U) __enable_irq();
  return true;
}

void motor_current_control_stop(void)
{
  uint32_t primask;
  motor_current_control_command_t zero_command = {0, 0};
  motor_current_control_sample_settle_cycles = 0U;
  motor_current_control_hall_invalid_cycles = 0U;
  motor_pwm_port_output_disable();
  primask = __get_PRIMASK();
  __disable_irq();
  motor_current_control_command = zero_command;
  motor_current_control_status.command = zero_command;
  motor_current_control_status.state = MOTOR_CURRENT_CONTROL_STOPPED;
  if (primask == 0U) __enable_irq();
}

void motor_current_control_fast_process(void)
{
  motor_current_sample_state_t current;
  motor_rotor_feedback_t rotor_feedback;
  motor_alpha_beta_q15_t alpha_beta;
  motor_direct_quadrature_q15_t dq;
  int32_t direct_ma, quadrature_ma, direct_mv, quadrature_mv;
  bool limited;

  if (motor_current_control_status.state != MOTOR_CURRENT_CONTROL_RUNNING) return;
  if (motor_current_control_sample_settle_cycles != 0U)
  {
    if ((!motor_rotor_feedback_read(&rotor_feedback)) ||
        (!rotor_feedback.valid))
    {
      motor_current_control_status.fault = MOTOR_CURRENT_CONTROL_FAULT_HALL;
      motor_current_control_status.state = MOTOR_CURRENT_CONTROL_FAULT;
      motor_pwm_port_emergency_stop();
      return;
    }
    motor_current_control_sample_settle_cycles--;
    motor_current_control_voltage_apply(
      motor_current_control_start_direct_mv,
      motor_current_control_start_quadrature_mv,
      rotor_feedback.electrical_angle_u16);
    return;
  }
  if (!motor_current_sample_state_read(&current))
    motor_current_control_status.fault = MOTOR_CURRENT_CONTROL_FAULT_SAMPLE;
  else if (!motor_rotor_feedback_read(&rotor_feedback))
  {
    /* Hall快照正被边沿中断更新时跳过本周期，下一控制周期重新读取。 */
    return;
  }
  else if (!rotor_feedback.valid)
  {
    if (motor_current_control_hall_invalid_cycles <
        MOTOR_CURRENT_CONTROL_HALL_INVALID_CYCLES)
    {
      motor_current_control_hall_invalid_cycles++;
    }
    if (motor_current_control_hall_invalid_cycles <
        MOTOR_CURRENT_CONTROL_HALL_INVALID_CYCLES)
    {
      return;
    }
    motor_current_control_status.fault = MOTOR_CURRENT_CONTROL_FAULT_HALL;
  }
  else
  {
    motor_current_control_hall_invalid_cycles = 0U;
    if (current.overcurrent_fault)
      motor_current_control_status.fault = MOTOR_CURRENT_CONTROL_FAULT_OVERCURRENT;
    else if (MOTOR_PWM_TIMER->brk_bit.oen == 0U)
      motor_current_control_status.fault = MOTOR_CURRENT_CONTROL_FAULT_PWM;
    else
    {
      motor_current_control_bus_voltage_update();
      alpha_beta = motor_foc_clarke_q15(current.phase_a_q15, current.phase_b_q15);
      dq = motor_foc_park_q15(alpha_beta,
                              rotor_feedback.electrical_angle_u16);
      direct_ma = (int32_t)(((int64_t)dq.direct_q15 * MOTOR_CURRENT_Q15_BASE_MA) / 32768);
      quadrature_ma = (int32_t)(((int64_t)dq.quadrature_q15 * MOTOR_CURRENT_Q15_BASE_MA) / 32768);
      direct_mv = motor_current_pi_axis_process(&motor_current_control_d_pi,
        motor_current_control_command.direct_reference_ma, direct_ma);
      quadrature_mv = motor_current_pi_axis_process(&motor_current_control_q_pi,
        motor_current_control_command.quadrature_reference_ma, quadrature_ma);
      motor_current_control_status.direct_p_mv = (int32_t)(((int64_t)(motor_current_control_command.direct_reference_ma - direct_ma) * motor_current_control_d_pi.proportional_gain_q15) >> 15);
      motor_current_control_status.direct_i_mv = motor_current_control_d_pi.integral_mv;
      motor_current_control_status.direct_request_mv = direct_mv;
      motor_current_control_status.quadrature_p_mv = (int32_t)(((int64_t)(motor_current_control_command.quadrature_reference_ma - quadrature_ma) * motor_current_control_q_pi.proportional_gain_q15) >> 15);
      motor_current_control_status.quadrature_i_mv = motor_current_control_q_pi.integral_mv;
      motor_current_control_status.quadrature_request_mv = quadrature_mv;
      limited = motor_voltage_limit_apply(&direct_mv, &quadrature_mv,
                                           motor_current_control_voltage_limit_mv);
      if (limited)
      {
        motor_current_pi_axis_output_track(&motor_current_control_d_pi, direct_mv);
        motor_current_pi_axis_output_track(&motor_current_control_q_pi, quadrature_mv);
      }
      (void)motor_current_control_voltage_apply(
        direct_mv, quadrature_mv, rotor_feedback.electrical_angle_u16);
      motor_current_control_status.command = motor_current_control_command;
      motor_current_control_status.direct_feedback_ma = direct_ma;
      motor_current_control_status.quadrature_feedback_ma = quadrature_ma;
      motor_current_control_status.direct_voltage_mv = direct_mv;
      motor_current_control_status.quadrature_voltage_mv = quadrature_mv;
      motor_current_control_status.electrical_angle_u16 =
        rotor_feedback.electrical_angle_u16;
      motor_current_control_status.electrical_frequency_millihz =
        rotor_feedback.electrical_frequency_millihz;
      motor_current_control_status.voltage_limited = limited;
      motor_current_control_status.sample_count++;
      return;
    }
  }
  motor_current_control_status.state = MOTOR_CURRENT_CONTROL_FAULT;
  motor_pwm_port_emergency_stop();
}

bool motor_current_control_status_read(motor_current_control_status_t *status)
{
  uint32_t primask;
  if (status == NULL) return false;
  primask = __get_PRIMASK();
  __disable_irq();
  *status = motor_current_control_status;
  if (primask == 0U) __enable_irq();
  return true;
}
