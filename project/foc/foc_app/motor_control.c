#include <stddef.h>
#include "at32f45x.h"
#include "motor_control.h"
#include "motor_current_control.h"
#include "motor_current_sample.h"
#include "motor_foc_math.h"
#include "motor_hall_angle_estimator.h"
#include "motor_pwm_port.h"
#include "motor_slow_sensor.h"
static volatile motor_control_status_t motor_control_status;
void motor_control_init(void)
{
  motor_control_status.state = MOTOR_CONTROL_STATE_STARTUP;
  motor_control_status.fault_code = 0U;
}
bool motor_control_ready_set(void)
{
  if ((motor_control_status.state != MOTOR_CONTROL_STATE_STARTUP) ||
      motor_pwm_port_output_is_enabled()) return false;
  motor_control_status.state = MOTOR_CONTROL_STATE_READY;
  return true;
}
bool motor_control_open_loop_start(const motor_open_loop_command_t *command)
{
  if ((motor_control_status.state != MOTOR_CONTROL_STATE_READY) ||
      (!motor_open_loop_start(command))) return false;
  motor_control_status.state = MOTOR_CONTROL_STATE_OPEN_LOOP;
  return true;
}
bool motor_control_open_loop_command_set(const motor_open_loop_command_t *command)
{
  return (motor_control_status.state == MOTOR_CONTROL_STATE_OPEN_LOOP) &&
         motor_open_loop_command_set(command);
}
bool motor_control_current_control_start(
  const motor_current_control_command_t *command)
{
  motor_open_loop_status_t open_loop;
  motor_hall_angle_estimator_t hall;
  motor_slow_sensor_state_t sensor;
  motor_alpha_beta_q15_t alpha_beta;
  motor_direct_quadrature_q15_t voltage_dq;
  uint32_t bus_mv;
  int32_t direct_seed_mv;
  int32_t quadrature_seed_mv;

  (void)motor_slow_sensor_process();
  if ((motor_control_status.state != MOTOR_CONTROL_STATE_OPEN_LOOP) ||
      (!motor_open_loop_status_read(&open_loop)) ||
      (open_loop.state != MOTOR_OPEN_LOOP_RUNNING) ||
      (!motor_hall_angle_estimator_read(&hall)) || (!hall.valid) ||
      (!motor_slow_sensor_state_read(&sensor))) return false;
  bus_mv = (uint32_t)sensor.bus_voltage_0p1v * 100U;
  alpha_beta = motor_foc_inverse_park_q15(
    (int16_t)(((int64_t)open_loop.applied_direct_voltage_mv * 32768) / bus_mv),
    (int16_t)(((int64_t)open_loop.applied_quadrature_voltage_mv * 32768) / bus_mv),
    open_loop.electrical_angle_u16);
  voltage_dq = motor_foc_park_q15(alpha_beta, hall.electrical_angle_u16);
  direct_seed_mv = (int32_t)(((int64_t)voltage_dq.direct_q15 * bus_mv) / 32768);
  quadrature_seed_mv = (int32_t)(
    ((int64_t)voltage_dq.quadrature_q15 * bus_mv) / 32768);
  if (!motor_open_loop_control_release()) return false;
  if (!motor_current_control_handover(command, direct_seed_mv,
                                       quadrature_seed_mv))
  {
    motor_control_fault_set(7U);
    return false;
  }
  motor_control_status.state = MOTOR_CONTROL_STATE_CURRENT_CONTROL;
  return true;
}
bool motor_control_current_control_command_set(
  const motor_current_control_command_t *command)
{
  return (motor_control_status.state == MOTOR_CONTROL_STATE_CURRENT_CONTROL) &&
         motor_current_control_command_set(command);
}
void motor_control_stop(void)
{
  motor_control_state_t previous_state = motor_control_status.state;
  motor_current_control_stop();
  motor_open_loop_stop();
  if ((previous_state == MOTOR_CONTROL_STATE_OPEN_LOOP) ||
      (previous_state == MOTOR_CONTROL_STATE_CURRENT_CONTROL))
    motor_control_status.state = MOTOR_CONTROL_STATE_READY;
}
void motor_control_fault_set(uint32_t fault_code)
{
  motor_pwm_port_emergency_stop();
  motor_control_status.fault_code = (fault_code == 0U) ? 1U : fault_code;
  motor_control_status.state = MOTOR_CONTROL_STATE_FAULT;
}
bool motor_control_fault_clear(void)
{
  if ((motor_control_status.state != MOTOR_CONTROL_STATE_FAULT) ||
      motor_pwm_port_output_is_enabled()) return false;
  motor_current_control_stop();
  motor_open_loop_stop();
  if ((!motor_current_sample_fault_clear()) ||
      (!motor_pwm_port_fault_clear())) return false;
  motor_control_status.fault_code = 0U;
  motor_control_status.state = MOTOR_CONTROL_STATE_READY;
  return true;
}
void motor_control_poll(void)
{
  motor_open_loop_status_t open_loop;
  motor_current_control_status_t current_control;
  (void)motor_open_loop_status_read(&open_loop);
  (void)motor_current_control_status_read(&current_control);
  if ((open_loop.state == MOTOR_OPEN_LOOP_FAULT) ||
      (current_control.state == MOTOR_CURRENT_CONTROL_FAULT))
    motor_control_fault_set(2U);
}
bool motor_control_status_read(motor_control_status_t *status)
{
  uint32_t primask;
  if (status == NULL) return false;
  primask = __get_PRIMASK();
  __disable_irq();
  *status = motor_control_status;
  if (primask == 0U) __enable_irq();
  return true;
}
