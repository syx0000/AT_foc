#include <stddef.h>
#include "at32f45x.h"
#include "motor_board_config.h"
#include "motor_control_config.h"
#include "motor_current_control.h"
#include "motor_current_pi.h"
#include "motor_current_sample.h"
#include "motor_foc_math.h"
#include "motor_hall_angle_estimator.h"
#include "motor_pwm_port.h"
#include "motor_slow_sensor.h"
#include "motor_voltage_limit.h"

static volatile motor_current_control_status_t motor_current_control_status;
static volatile motor_current_control_command_t motor_current_control_command;
static motor_current_control_config_t motor_current_control_config;
static motor_current_pi_axis_t motor_current_control_d_pi, motor_current_control_q_pi;
static uint32_t motor_current_control_bus_mv;
static int32_t motor_current_control_voltage_limit_mv;

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
    (config->voltage_limit_percent <= 50U) &&
    (config->absolute_voltage_limit_mv > 0) &&
    (config->command_limit_ma > 0) &&
    (config->abort_current_ma >= config->command_limit_ma) &&
    (config->abort_current_ma <= MOTOR_SOFTWARE_OVERCURRENT_MA);
}

static void motor_current_control_voltage_apply(int32_t direct_mv,
                                                int32_t quadrature_mv,
                                                uint16_t angle_u16)
{
  int16_t direct_q15 = (int16_t)(((int64_t)direct_mv * 32768) / motor_current_control_bus_mv);
  int16_t quadrature_q15 = (int16_t)(((int64_t)quadrature_mv * 32768) /
                                     motor_current_control_bus_mv);
  motor_alpha_beta_q15_t voltage = motor_foc_inverse_park_q15(
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
}

static bool motor_current_control_prepare(
  const motor_current_control_command_t *command,
  int32_t direct_seed_mv, int32_t quadrature_seed_mv)
{
  motor_slow_sensor_state_t sensor;
  motor_hall_angle_estimator_t hall;
  (void)motor_slow_sensor_process();
  if ((!motor_current_control_command_valid(command)) ||
      (!motor_slow_sensor_state_read(&sensor)) ||
      (!motor_hall_angle_estimator_read(&hall)) || (!hall.valid) ||
      (sensor.bus_voltage_0p1v < MOTOR_BUS_UNDERVOLTAGE_0P1V) ||
      (sensor.bus_voltage_0p1v > MOTOR_BUS_OVERVOLTAGE_0P1V))
  {
    return false;
  }
  motor_current_control_bus_mv = (uint32_t)sensor.bus_voltage_0p1v * 100U;
  motor_current_control_voltage_limit_mv = (int32_t)(
    (motor_current_control_bus_mv * motor_current_control_config.voltage_limit_percent) / 100U);
  if (motor_current_control_voltage_limit_mv > motor_current_control_config.absolute_voltage_limit_mv)
  {
    motor_current_control_voltage_limit_mv = motor_current_control_config.absolute_voltage_limit_mv;
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
  motor_current_control_status.state = MOTOR_CURRENT_CONTROL_RUNNING;
  motor_current_control_status.fault = MOTOR_CURRENT_CONTROL_FAULT_NONE;
  motor_current_control_status.sample_count = 0U;
  return true;
}

void motor_current_control_init(void)
{
  motor_current_control_config.direct_kp_q15 = MOTOR_CURRENT_PI_D_KP_Q15;
  motor_current_control_config.quadrature_kp_q15 = MOTOR_CURRENT_PI_Q_KP_Q15;
  motor_current_control_config.integral_gain_q15 = MOTOR_CURRENT_PI_KI_Q15;
  motor_current_control_config.voltage_limit_percent =
    MOTOR_CURRENT_CONTROL_VOLTAGE_LIMIT_PERCENT;
  motor_current_control_config.absolute_voltage_limit_mv =
    MOTOR_CURRENT_CONTROL_ABSOLUTE_VOLTAGE_LIMIT_MV;
  motor_current_control_config.command_limit_ma = MOTOR_CURRENT_COMMAND_MAX_MA;
  motor_current_control_config.abort_current_ma = MOTOR_CURRENT_CONTROL_ABORT_CURRENT_MA;
  motor_current_control_status.state = MOTOR_CURRENT_CONTROL_STOPPED;
  motor_current_control_status.fault = MOTOR_CURRENT_CONTROL_FAULT_NONE;
  motor_current_control_status.sample_count = 0U;
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
  motor_hall_angle_estimator_t hall;
  if (motor_pwm_port_output_is_enabled() ||
      (!motor_current_control_prepare(command, 0, 0)) ||
      (!motor_hall_angle_estimator_read(&hall)))
  {
    return false;
  }
  motor_current_control_voltage_apply(0, 0, hall.electrical_angle_u16);
  if (!motor_pwm_port_output_enable())
  {
    motor_current_control_status.state = MOTOR_CURRENT_CONTROL_FAULT;
    motor_current_control_status.fault = MOTOR_CURRENT_CONTROL_FAULT_PWM;
    return false;
  }
  return true;
}

bool motor_current_control_handover(
  const motor_current_control_command_t *command,
  int32_t direct_voltage_seed_mv, int32_t quadrature_voltage_seed_mv)
{
  return motor_pwm_port_output_is_enabled() &&
    motor_current_control_prepare(command, direct_voltage_seed_mv,
                                  quadrature_voltage_seed_mv);
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
  if (primask == 0U) __enable_irq();
  return true;
}

void motor_current_control_stop(void)
{
  motor_pwm_port_output_disable();
  motor_current_control_status.state = MOTOR_CURRENT_CONTROL_STOPPED;
}

void motor_current_control_fast_process(void)
{
  motor_current_sample_state_t current;
  motor_hall_angle_estimator_t hall;
  motor_alpha_beta_q15_t alpha_beta;
  motor_direct_quadrature_q15_t dq;
  int32_t direct_ma, quadrature_ma, direct_mv, quadrature_mv;
  bool limited;

  if (motor_current_control_status.state != MOTOR_CURRENT_CONTROL_RUNNING) return;
  if (!motor_current_sample_state_read(&current))
    motor_current_control_status.fault = MOTOR_CURRENT_CONTROL_FAULT_SAMPLE;
  else if ((!motor_hall_angle_estimator_read(&hall)) || (!hall.valid))
    motor_current_control_status.fault = MOTOR_CURRENT_CONTROL_FAULT_HALL;
  else if (current.overcurrent_fault ||
           (motor_current_control_abs(current.phase_a_ma) >= motor_current_control_config.abort_current_ma) ||
           (motor_current_control_abs(current.phase_b_ma) >= motor_current_control_config.abort_current_ma) ||
           (motor_current_control_abs(current.phase_c_ma) >= motor_current_control_config.abort_current_ma))
    motor_current_control_status.fault = MOTOR_CURRENT_CONTROL_FAULT_OVERCURRENT;
  else if (MOTOR_PWM_TIMER->brk_bit.oen == 0U)
    motor_current_control_status.fault = MOTOR_CURRENT_CONTROL_FAULT_PWM;
  else
  {
    alpha_beta = motor_foc_clarke_q15(current.phase_a_q15, current.phase_b_q15);
    dq = motor_foc_park_q15(alpha_beta, hall.electrical_angle_u16);
    direct_ma = (int32_t)(((int64_t)dq.direct_q15 * MOTOR_CURRENT_Q15_BASE_MA) / 32768);
    quadrature_ma = (int32_t)(((int64_t)dq.quadrature_q15 * MOTOR_CURRENT_Q15_BASE_MA) / 32768);
    direct_mv = motor_current_pi_axis_process(&motor_current_control_d_pi,
      motor_current_control_command.direct_reference_ma, direct_ma);
    quadrature_mv = motor_current_pi_axis_process(&motor_current_control_q_pi,
      motor_current_control_command.quadrature_reference_ma, quadrature_ma);
    limited = motor_voltage_limit_apply(&direct_mv, &quadrature_mv,
                                         motor_current_control_voltage_limit_mv);
    if (limited)
    {
      motor_current_pi_axis_output_track(&motor_current_control_d_pi, direct_mv);
      motor_current_pi_axis_output_track(&motor_current_control_q_pi, quadrature_mv);
    }
    motor_current_control_voltage_apply(direct_mv, quadrature_mv,
                                        hall.electrical_angle_u16);
    motor_current_control_status.command = motor_current_control_command;
    motor_current_control_status.direct_feedback_ma = direct_ma;
    motor_current_control_status.quadrature_feedback_ma = quadrature_ma;
    motor_current_control_status.direct_voltage_mv = direct_mv;
    motor_current_control_status.quadrature_voltage_mv = quadrature_mv;
    motor_current_control_status.electrical_angle_u16 = hall.electrical_angle_u16;
    motor_current_control_status.electrical_frequency_millihz = hall.electrical_frequency_millihz;
    motor_current_control_status.voltage_limited = limited;
    motor_current_control_status.sample_count++;
    return;
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
