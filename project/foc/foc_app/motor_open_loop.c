#include <stddef.h>
#include "at32f45x.h"
#include "motor_board_config.h"
#include "motor_control_config.h"
#include "motor_current_sample.h"
#include "motor_foc_math.h"
#include "motor_open_loop.h"
#include "motor_pwm_port.h"
#include "motor_ramp.h"
#include "motor_slow_sensor.h"

static volatile motor_open_loop_status_t motor_open_loop_status;
static motor_open_loop_config_t motor_open_loop_config;
static motor_open_loop_command_t motor_open_loop_command;
static uint32_t motor_open_loop_phase_accumulator;
static uint32_t motor_open_loop_alignment_ticks;
static motor_ramp_t motor_open_loop_frequency_ramp;
static uint32_t motor_open_loop_bus_voltage_mv;

static int32_t motor_open_loop_abs(int32_t value)
{
  return (value < 0) ? -value : value;
}

static bool motor_open_loop_config_valid(const motor_open_loop_config_t *config)
{
  return (config != NULL) && (config->maximum_voltage_mv > 0) &&
    (config->maximum_frequency_millihz > 0) &&
    (config->start_frequency_millihz >= 0) &&
    (config->start_frequency_millihz <= config->maximum_frequency_millihz) &&
    (config->alignment_voltage_mv >= 0) &&
    (config->alignment_voltage_mv <= config->maximum_voltage_mv);
}

static bool motor_open_loop_command_valid(const motor_open_loop_command_t *command)
{
  int32_t direct_abs;
  int32_t quadrature_abs;
  int32_t maximum;
  int32_t minimum;

  if ((command == NULL) || (command->acceleration_millihz_per_s == 0U) ||
      (motor_open_loop_abs(command->target_frequency_millihz) >
       motor_open_loop_config.maximum_frequency_millihz))
  {
    return false;
  }
  direct_abs = motor_open_loop_abs(command->direct_voltage_mv);
  quadrature_abs = motor_open_loop_abs(command->quadrature_voltage_mv);
  maximum = (direct_abs > quadrature_abs) ? direct_abs : quadrature_abs;
  minimum = (direct_abs > quadrature_abs) ? quadrature_abs : direct_abs;
  return (maximum + minimum / 2) <= motor_open_loop_config.maximum_voltage_mv;
}

static uint16_t motor_open_loop_duty_to_compare(uint16_t duty_q15)
{
  uint32_t compare = ((uint32_t)duty_q15 * MOTOR_PWM_PERIOD_COUNTS + 16383U) /
                     32767U;
  if (compare > MOTOR_PWM_COMPARE_MAX)
  {
    compare = MOTOR_PWM_COMPARE_MAX;
  }
  return (uint16_t)compare;
}

static void motor_open_loop_voltage_apply(int32_t direct_voltage_mv,
                                          int32_t quadrature_voltage_mv,
                                          uint16_t electrical_angle_u16)
{
  int32_t direct_q15 = (int32_t)(((int64_t)direct_voltage_mv * 32768) /
                                 motor_open_loop_bus_voltage_mv);
  int32_t quadrature_q15 = (int32_t)(((int64_t)quadrature_voltage_mv * 32768) /
                                     motor_open_loop_bus_voltage_mv);
  motor_alpha_beta_q15_t voltage = motor_foc_inverse_park_q15(
    (int16_t)direct_q15, (int16_t)quadrature_q15, electrical_angle_u16);
  motor_svpwm_duty_q15_t duty = motor_foc_svpwm_q15(voltage);
  motor_pwm_compare_t compare;

  compare.phase_a = motor_open_loop_duty_to_compare(duty.phase_a);
  compare.phase_b = motor_open_loop_duty_to_compare(duty.phase_b);
  compare.phase_c = motor_open_loop_duty_to_compare(duty.phase_c);
  motor_pwm_port_compare_set(&compare);
  motor_open_loop_status.electrical_angle_u16 = electrical_angle_u16;
  motor_open_loop_status.applied_direct_voltage_mv = direct_voltage_mv;
  motor_open_loop_status.applied_quadrature_voltage_mv = quadrature_voltage_mv;
  motor_open_loop_status.duty_a_q15 = duty.phase_a;
  motor_open_loop_status.duty_b_q15 = duty.phase_b;
  motor_open_loop_status.duty_c_q15 = duty.phase_c;
  motor_open_loop_status.voltage_limited = false;
}

void motor_open_loop_init(void)
{
  motor_open_loop_config.maximum_voltage_mv = MOTOR_OPEN_LOOP_MAXIMUM_VOLTAGE_MV;
  motor_open_loop_config.maximum_frequency_millihz =
    MOTOR_OPEN_LOOP_MAXIMUM_FREQUENCY_MILLIHZ;
  motor_open_loop_config.start_frequency_millihz =
    MOTOR_OPEN_LOOP_START_FREQUENCY_MILLIHZ;
  motor_open_loop_config.alignment_voltage_mv = MOTOR_OPEN_LOOP_ALIGNMENT_VOLTAGE_MV;
  motor_open_loop_config.alignment_angle_u16 = MOTOR_OPEN_LOOP_ALIGNMENT_ANGLE_U16;
  motor_open_loop_config.alignment_time_ms = MOTOR_OPEN_LOOP_ALIGNMENT_TIME_MS;
  motor_open_loop_status.state = MOTOR_OPEN_LOOP_STOPPED;
  motor_open_loop_status.electrical_angle_u16 = 0U;
  motor_open_loop_status.target_frequency_millihz = 0;
  motor_open_loop_status.actual_frequency_millihz = 0;
  motor_open_loop_status.applied_direct_voltage_mv = 0;
  motor_open_loop_status.applied_quadrature_voltage_mv = 0;
  motor_open_loop_status.duty_a_q15 = 0U;
  motor_open_loop_status.duty_b_q15 = 0U;
  motor_open_loop_status.duty_c_q15 = 0U;
  motor_open_loop_status.voltage_limited = false;
  motor_open_loop_phase_accumulator = 0U;
  motor_open_loop_alignment_ticks = 0U;
  (void)motor_ramp_init(&motor_open_loop_frequency_ramp, 0, 0, 1U);
}

bool motor_open_loop_config_set(const motor_open_loop_config_t *config)
{
  if ((!motor_open_loop_config_valid(config)) ||
      (motor_open_loop_status.state != MOTOR_OPEN_LOOP_STOPPED))
  {
    return false;
  }
  motor_open_loop_config = *config;
  return true;
}

bool motor_open_loop_command_set(const motor_open_loop_command_t *command)
{
  uint32_t primask;
  if ((!motor_open_loop_command_valid(command)) ||
      ((motor_open_loop_status.state != MOTOR_OPEN_LOOP_ALIGNING) &&
       (motor_open_loop_status.state != MOTOR_OPEN_LOOP_RUNNING)))
  {
    return false;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  motor_open_loop_command = *command;
  (void)motor_ramp_target_set(&motor_open_loop_frequency_ramp,
                              command->target_frequency_millihz,
                              command->acceleration_millihz_per_s);
  motor_open_loop_status.target_frequency_millihz =
    command->target_frequency_millihz;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return true;
}

bool motor_open_loop_start(const motor_open_loop_command_t *command)
{
  motor_current_sample_state_t current;
  motor_slow_sensor_state_t sensor;

  (void)motor_slow_sensor_process();
  if ((!motor_open_loop_command_valid(command)) ||
      (!motor_current_sample_state_read(&current)) || current.overcurrent_fault ||
      (!motor_slow_sensor_state_read(&sensor)) ||
      (sensor.bus_voltage_0p1v < MOTOR_BUS_UNDERVOLTAGE_0P1V) ||
      (sensor.bus_voltage_0p1v > MOTOR_BUS_OVERVOLTAGE_0P1V) ||
      motor_pwm_port_output_is_enabled())
  {
    return false;
  }
  motor_open_loop_command = *command;
  motor_open_loop_bus_voltage_mv = (uint32_t)sensor.bus_voltage_0p1v * 100U;
  motor_open_loop_phase_accumulator =
    (uint32_t)motor_open_loop_config.alignment_angle_u16 << 16;
  motor_open_loop_alignment_ticks = 0U;
  (void)motor_ramp_init(&motor_open_loop_frequency_ramp, 0,
                        command->target_frequency_millihz,
                        command->acceleration_millihz_per_s);
  motor_open_loop_status.target_frequency_millihz = command->target_frequency_millihz;
  motor_open_loop_status.actual_frequency_millihz = 0;
  motor_open_loop_voltage_apply(motor_open_loop_config.alignment_voltage_mv, 0,
                                motor_open_loop_config.alignment_angle_u16);
  if (!motor_pwm_port_output_enable())
  {
    motor_open_loop_status.state = MOTOR_OPEN_LOOP_FAULT;
    motor_pwm_port_emergency_stop();
    return false;
  }
  motor_open_loop_status.state = MOTOR_OPEN_LOOP_ALIGNING;
  return true;
}

void motor_open_loop_stop(void)
{
  motor_pwm_port_output_disable();
  motor_open_loop_status.state = MOTOR_OPEN_LOOP_STOPPED;
  motor_open_loop_status.target_frequency_millihz = 0;
  motor_open_loop_status.actual_frequency_millihz = 0;
}

bool motor_open_loop_control_release(void)
{
  if ((motor_open_loop_status.state != MOTOR_OPEN_LOOP_RUNNING) ||
      (!motor_pwm_port_output_is_enabled()))
  {
    return false;
  }
  motor_open_loop_status.state = MOTOR_OPEN_LOOP_STOPPED;
  motor_open_loop_status.target_frequency_millihz = 0;
  motor_open_loop_status.actual_frequency_millihz = 0;
  return true;
}

void motor_open_loop_fast_process(void)
{
  int64_t phase_step;

  if ((motor_open_loop_status.state != MOTOR_OPEN_LOOP_ALIGNING) &&
      (motor_open_loop_status.state != MOTOR_OPEN_LOOP_RUNNING))
  {
    return;
  }
  if (!motor_pwm_port_output_is_enabled())
  {
    motor_open_loop_status.state = MOTOR_OPEN_LOOP_FAULT;
    return;
  }
  if (motor_open_loop_status.state == MOTOR_OPEN_LOOP_ALIGNING)
  {
    motor_open_loop_alignment_ticks++;
    if (motor_open_loop_alignment_ticks <
        motor_open_loop_config.alignment_time_ms * (MOTOR_PWM_FREQUENCY_HZ / 1000U))
    {
      return;
    }
    motor_open_loop_status.state = MOTOR_OPEN_LOOP_RUNNING;
    motor_open_loop_status.actual_frequency_millihz =
      (motor_open_loop_command.target_frequency_millihz < 0) ?
      -motor_open_loop_config.start_frequency_millihz :
      motor_open_loop_config.start_frequency_millihz;
    motor_open_loop_frequency_ramp.current =
      motor_open_loop_status.actual_frequency_millihz;
  }
  motor_open_loop_status.actual_frequency_millihz = motor_ramp_process(
    &motor_open_loop_frequency_ramp, MOTOR_PWM_FREQUENCY_HZ);
  phase_step = ((int64_t)motor_open_loop_status.actual_frequency_millihz << 32) /
               ((int64_t)MOTOR_PWM_FREQUENCY_HZ * 1000);
  motor_open_loop_phase_accumulator += (uint32_t)phase_step;
  motor_open_loop_voltage_apply(motor_open_loop_command.direct_voltage_mv,
                                motor_open_loop_command.quadrature_voltage_mv,
                                (uint16_t)(motor_open_loop_phase_accumulator >> 16));
}

bool motor_open_loop_status_read(motor_open_loop_status_t *status)
{
  uint32_t primask;
  if (status == NULL)
  {
    return false;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  *status = motor_open_loop_status;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return true;
}
