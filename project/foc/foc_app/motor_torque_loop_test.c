#include <stddef.h>
#include "motor_board_config.h"
#include "motor_control_config.h"
#include "motor_current_control.h"
#include "motor_current_sample.h"
#include "motor_foc_math.h"
#include "motor_hall_angle_estimator.h"
#include "motor_open_loop.h"
#include "motor_pwm_port.h"
#include "motor_ramp.h"
#include "motor_slow_sensor.h"
#include "motor_torque_loop_test.h"
#include "wk_system.h"

#define MOTOR_TORQUE_TEST_OPEN_LOOP_VOLTAGE_MV 1920L
#define MOTOR_TORQUE_TEST_OPEN_LOOP_FREQUENCY_MILLIHZ 10000L
#define MOTOR_TORQUE_TEST_OPEN_LOOP_ACCELERATION_MILLIHZ_PER_S 2667U

static volatile bool motor_torque_loop_test_active, motor_torque_loop_test_done, motor_torque_loop_test_fault;
static volatile uint32_t motor_torque_loop_test_count, motor_torque_loop_test_statistics_count;
static volatile int64_t motor_torque_loop_test_id_sum, motor_torque_loop_test_iq_sum, motor_torque_loop_test_vd_sum, motor_torque_loop_test_vq_sum;
static volatile int32_t motor_torque_loop_test_id_peak, motor_torque_loop_test_iq_peak;
static volatile motor_torque_loop_test_status_t motor_torque_loop_test_status;
static motor_ramp_t motor_torque_loop_test_id_ramp, motor_torque_loop_test_iq_ramp;

static int32_t motor_torque_loop_test_abs(int32_t value)
{
  return (value < 0) ? -value : value;
}

static uint32_t motor_torque_loop_test_ramp_rate_get(int32_t initial,
                                                     int32_t target)
{
  uint32_t difference = (uint32_t)motor_torque_loop_test_abs(target - initial);
  uint32_t rate = (uint32_t)(((uint64_t)difference * MOTOR_PWM_FREQUENCY_HZ) /
                             MOTOR_TORQUE_TEST_REFERENCE_RAMP_SAMPLES);
  return (rate == 0U) ? 1U : rate;
}

void motor_torque_loop_test_fast_process(void)
{
  motor_current_control_status_t control;
  motor_current_control_command_t command;
  int32_t absolute_value;

  if (!motor_torque_loop_test_active)
  {
    return;
  }
  if ((!motor_current_control_status_read(&control)) ||
      (control.state != MOTOR_CURRENT_CONTROL_RUNNING))
  {
    motor_torque_loop_test_status = (control.fault == MOTOR_CURRENT_CONTROL_FAULT_HALL) ?
      MOTOR_TORQUE_TEST_STATUS_HALL_LOST :
      ((control.fault == MOTOR_CURRENT_CONTROL_FAULT_OVERCURRENT) ?
       MOTOR_TORQUE_TEST_STATUS_OVERCURRENT :
       MOTOR_TORQUE_TEST_STATUS_PWM_FAULT);
    motor_torque_loop_test_fault = true;
    motor_torque_loop_test_active = false;
    return;
  }

  command.direct_reference_ma = motor_ramp_process(&motor_torque_loop_test_id_ramp,
                                                    MOTOR_PWM_FREQUENCY_HZ);
  command.quadrature_reference_ma = motor_ramp_process(&motor_torque_loop_test_iq_ramp,
                                                        MOTOR_PWM_FREQUENCY_HZ);
  if (!motor_current_control_command_set(&command))
  {
    motor_torque_loop_test_status = MOTOR_TORQUE_TEST_STATUS_PWM_FAULT;
    motor_torque_loop_test_fault = true;
    motor_torque_loop_test_active = false;
    motor_current_control_stop();
    return;
  }

  if (motor_torque_loop_test_count >= MOTOR_TORQUE_TEST_REFERENCE_RAMP_SAMPLES)
  {
    motor_torque_loop_test_id_sum += control.direct_feedback_ma;
    motor_torque_loop_test_iq_sum += control.quadrature_feedback_ma;
    motor_torque_loop_test_vd_sum += control.direct_voltage_mv;
    motor_torque_loop_test_vq_sum += control.quadrature_voltage_mv;
    motor_torque_loop_test_statistics_count++;
    absolute_value = motor_torque_loop_test_abs(control.direct_feedback_ma);
    if (absolute_value > motor_torque_loop_test_id_peak) motor_torque_loop_test_id_peak = absolute_value;
    absolute_value = motor_torque_loop_test_abs(control.quadrature_feedback_ma);
    if (absolute_value > motor_torque_loop_test_iq_peak) motor_torque_loop_test_iq_peak = absolute_value;
  }

  motor_torque_loop_test_count++;
  if (motor_torque_loop_test_count >= MOTOR_TORQUE_TEST_DURATION_SAMPLES)
  {
    motor_torque_loop_test_active = false;
    motor_torque_loop_test_done = true;
    motor_current_control_stop();
  }
}

bool motor_torque_loop_test_run(motor_torque_loop_test_result_t *result)
{
  motor_open_loop_command_t open_command;
  motor_open_loop_status_t open_status;
  motor_hall_angle_estimator_t hall;
  motor_current_sample_state_t current;
  motor_slow_sensor_state_t sensor;
  motor_current_control_command_t current_command;
  motor_current_control_config_t current_control_config;
  motor_current_control_status_t control;
  motor_alpha_beta_q15_t alpha_beta;
  motor_direct_quadrature_q15_t dq;
  uint32_t bootstrap_ms;
  uint32_t bus_mv;
  int32_t seed_direct_mv, seed_quadrature_mv;

  if (result == NULL) return false;
  result->status = MOTOR_TORQUE_TEST_STATUS_OK;
  result->sample_count = 0U;
  if (motor_pwm_port_output_is_enabled())
  {
    result->status = MOTOR_TORQUE_TEST_STATUS_PWM_BUSY;
    return false;
  }

  current_control_config.direct_kp_q15 = MOTOR_CURRENT_PI_D_KP_Q15;
  current_control_config.quadrature_kp_q15 = MOTOR_CURRENT_PI_Q_KP_Q15;
  current_control_config.integral_gain_q15 = MOTOR_CURRENT_PI_KI_Q15;
  current_control_config.voltage_limit_percent =
    MOTOR_TORQUE_TEST_BUS_VOLTAGE_LIMIT_PERCENT;
  current_control_config.absolute_voltage_limit_mv =
    MOTOR_TORQUE_TEST_ABSOLUTE_VOLTAGE_LIMIT_MV;
  current_control_config.command_limit_ma =
    MOTOR_TORQUE_TEST_ABORT_CURRENT_MA;
  current_control_config.abort_current_ma =
    MOTOR_TORQUE_TEST_ABORT_CURRENT_MA;
  if (!motor_current_control_config_set(&current_control_config))
  {
    result->status = MOTOR_TORQUE_TEST_STATUS_PWM_BUSY;
    return false;
  }

  open_command.direct_voltage_mv = 0;
  open_command.quadrature_voltage_mv = MOTOR_TORQUE_TEST_OPEN_LOOP_VOLTAGE_MV;
  open_command.target_frequency_millihz =
    MOTOR_TORQUE_TEST_OPEN_LOOP_FREQUENCY_MILLIHZ;
  open_command.acceleration_millihz_per_s =
    MOTOR_TORQUE_TEST_OPEN_LOOP_ACCELERATION_MILLIHZ_PER_S;
  if (!motor_open_loop_start(&open_command))
  {
    result->status = MOTOR_TORQUE_TEST_STATUS_OPEN_LOOP_START_FAILED;
    return false;
  }

  for (bootstrap_ms = 0U;
       bootstrap_ms < MOTOR_TORQUE_TEST_BOOTSTRAP_TIMEOUT_MS; bootstrap_ms++)
  {
    wk_delay_ms(1U);
    (void)motor_hall_angle_estimator_read(&hall);
    (void)motor_open_loop_status_read(&open_status);
    if ((open_status.state == MOTOR_OPEN_LOOP_FAULT) ||
        (!motor_pwm_port_output_is_enabled()))
    {
      motor_open_loop_stop();
      result->status = MOTOR_TORQUE_TEST_STATUS_BOOTSTRAP_FAILED;
      return false;
    }
    if (hall.valid && (hall.electrical_frequency_millihz >=
                       MOTOR_TORQUE_TEST_HANDOVER_FREQUENCY_MILLIHZ)) break;
  }
  if (bootstrap_ms >= MOTOR_TORQUE_TEST_BOOTSTRAP_TIMEOUT_MS)
  {
    motor_open_loop_stop();
    result->status = MOTOR_TORQUE_TEST_STATUS_BOOTSTRAP_TIMEOUT;
    return false;
  }

  (void)motor_slow_sensor_process();
  if ((!motor_slow_sensor_state_read(&sensor)) ||
      (!motor_current_sample_state_read(&current)) || (!hall.valid))
  {
    motor_open_loop_stop();
    result->status = MOTOR_TORQUE_TEST_STATUS_SENSOR_INVALID;
    return false;
  }
  bus_mv = (uint32_t)sensor.bus_voltage_0p1v * 100U;
  alpha_beta = motor_foc_clarke_q15(current.phase_a_q15, current.phase_b_q15);
  dq = motor_foc_park_q15(alpha_beta, hall.electrical_angle_u16);
  current_command.direct_reference_ma = (int32_t)(
    ((int64_t)dq.direct_q15 * MOTOR_CURRENT_Q15_BASE_MA) / 32768);
  current_command.quadrature_reference_ma = (int32_t)(
    ((int64_t)dq.quadrature_q15 * MOTOR_CURRENT_Q15_BASE_MA) / 32768);

  alpha_beta = motor_foc_inverse_park_q15(
    (int16_t)(((int64_t)open_status.applied_direct_voltage_mv * 32768) / bus_mv),
    (int16_t)(((int64_t)open_status.applied_quadrature_voltage_mv * 32768) / bus_mv),
    open_status.electrical_angle_u16);
  dq = motor_foc_park_q15(alpha_beta, hall.electrical_angle_u16);
  seed_direct_mv = (int32_t)(((int64_t)dq.direct_q15 * bus_mv) / 32768);
  seed_quadrature_mv = (int32_t)(((int64_t)dq.quadrature_q15 * bus_mv) / 32768);

  (void)motor_ramp_init(&motor_torque_loop_test_id_ramp, current_command.direct_reference_ma, 0,
    motor_torque_loop_test_ramp_rate_get(current_command.direct_reference_ma, 0));
  (void)motor_ramp_init(&motor_torque_loop_test_iq_ramp, current_command.quadrature_reference_ma,
    MOTOR_TORQUE_TEST_QUADRATURE_REFERENCE_MA,
    motor_torque_loop_test_ramp_rate_get(current_command.quadrature_reference_ma,
                                         MOTOR_TORQUE_TEST_QUADRATURE_REFERENCE_MA));
  if ((!motor_open_loop_control_release()) ||
      (!motor_current_control_handover(&current_command, seed_direct_mv,
                                       seed_quadrature_mv)))
  {
    motor_pwm_port_emergency_stop();
    result->status = MOTOR_TORQUE_TEST_STATUS_PWM_FAULT;
    return false;
  }

  motor_torque_loop_test_count = 0U;
  motor_torque_loop_test_statistics_count = 0U;
  motor_torque_loop_test_id_sum = 0;
  motor_torque_loop_test_iq_sum = 0;
  motor_torque_loop_test_vd_sum = 0;
  motor_torque_loop_test_vq_sum = 0;
  motor_torque_loop_test_id_peak = 0;
  motor_torque_loop_test_iq_peak = 0;
  motor_torque_loop_test_status = MOTOR_TORQUE_TEST_STATUS_OK;
  motor_torque_loop_test_done = false;
  motor_torque_loop_test_fault = false;
  motor_torque_loop_test_active = true;
  while (motor_torque_loop_test_active) wk_delay_ms(1U);

  (void)motor_current_control_status_read(&control);
  (void)motor_current_sample_state_read(&current);
  (void)motor_hall_angle_estimator_read(&hall);
  result->status = motor_torque_loop_test_fault ? motor_torque_loop_test_status : MOTOR_TORQUE_TEST_STATUS_OK;
  result->sample_count = motor_torque_loop_test_fault ? motor_torque_loop_test_count : motor_torque_loop_test_statistics_count;
  result->final_frequency_millihz = hall.electrical_frequency_millihz;
  result->final_hall_state = hall.hall_state;
  result->final_phase_a_ma = current.phase_a_ma;
  result->final_phase_b_ma = current.phase_b_ma;
  result->final_phase_c_ma = current.phase_c_ma;
  if (motor_torque_loop_test_fault || (!motor_torque_loop_test_done) || (motor_torque_loop_test_statistics_count == 0U)) return false;

  result->direct_average_ma = (int32_t)(motor_torque_loop_test_id_sum / motor_torque_loop_test_statistics_count);
  result->quadrature_average_ma = (int32_t)(motor_torque_loop_test_iq_sum / motor_torque_loop_test_statistics_count);
  result->direct_peak_ma = motor_torque_loop_test_id_peak;
  result->quadrature_peak_ma = motor_torque_loop_test_iq_peak;
  result->direct_voltage_average_mv = (int32_t)(motor_torque_loop_test_vd_sum / motor_torque_loop_test_statistics_count);
  result->quadrature_voltage_average_mv = (int32_t)(motor_torque_loop_test_vq_sum / motor_torque_loop_test_statistics_count);
  result->final_direct_voltage_mv = control.direct_voltage_mv;
  result->final_quadrature_voltage_mv = control.quadrature_voltage_mv;
  return true;
}
