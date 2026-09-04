#include <stddef.h>
#include "at32f45x_conf.h"
#include "motor_board_config.h"
#include "motor_control_config.h"
#include "motor_current_loop_test.h"
#include "motor_current_pi.h"
#include "motor_current_sample.h"
#include "motor_foc_math.h"
#include "motor_pwm_port.h"
#include "motor_slow_sensor.h"
#include "wk_system.h"

static volatile bool motor_current_loop_test_active, motor_current_loop_test_done, motor_current_loop_test_fault;
static volatile uint32_t motor_current_loop_test_count, motor_current_loop_test_bus_mv;
static volatile int64_t motor_current_loop_test_id_sum, motor_current_loop_test_iq_sum;
static volatile int32_t motor_current_loop_test_id_peak, motor_current_loop_test_iq_peak, motor_current_loop_test_vd, motor_current_loop_test_vq;
static motor_current_pi_axis_t motor_current_loop_test_d_pi, motor_current_loop_test_q_pi;

static int32_t motor_current_loop_test_abs(int32_t value) { return (value < 0) ? -value : value; }
static void motor_current_loop_test_voltage_apply(int32_t vd_mv, int32_t vq_mv)
{
  int32_t vd_q15 = (int32_t)(((int64_t)vd_mv * 32768) / motor_current_loop_test_bus_mv);
  int32_t vq_q15 = (int32_t)(((int64_t)vq_mv * 32768) / motor_current_loop_test_bus_mv);
  motor_alpha_beta_q15_t voltage;
  motor_svpwm_duty_q15_t duty;
  motor_pwm_compare_t compare;
  if (vd_q15 > 32767) vd_q15 = 32767; if (vd_q15 < -32768) vd_q15 = -32768;
  if (vq_q15 > 32767) vq_q15 = 32767; if (vq_q15 < -32768) vq_q15 = -32768;
  voltage = motor_foc_inverse_park_q15((int16_t)vd_q15, (int16_t)vq_q15, 0U);
  duty = motor_foc_svpwm_q15(voltage);
  compare.phase_a = (uint16_t)(((uint32_t)duty.phase_a * MOTOR_PWM_PERIOD_COUNTS) / 32767U);
  compare.phase_b = (uint16_t)(((uint32_t)duty.phase_b * MOTOR_PWM_PERIOD_COUNTS) / 32767U);
  compare.phase_c = (uint16_t)(((uint32_t)duty.phase_c * MOTOR_PWM_PERIOD_COUNTS) / 32767U);
  motor_pwm_port_compare_set(&compare);
}

void motor_current_loop_test_fast_process(void)
{
  motor_current_sample_state_t current;
  motor_alpha_beta_q15_t ab;
  int32_t id_ma, iq_ma;
  if (!motor_current_loop_test_active) return;
  if ((!motor_current_sample_state_read(&current)) || current.overcurrent_fault ||
      (motor_current_loop_test_abs(current.phase_a_ma) >= MOTOR_CURRENT_LOOP_TEST_ABORT_CURRENT_MA) ||
      (motor_current_loop_test_abs(current.phase_b_ma) >= MOTOR_CURRENT_LOOP_TEST_ABORT_CURRENT_MA) ||
      (motor_current_loop_test_abs(current.phase_c_ma) >= MOTOR_CURRENT_LOOP_TEST_ABORT_CURRENT_MA) ||
      (MOTOR_PWM_TIMER->brk_bit.oen == 0U)) {
    motor_current_loop_test_fault = true; motor_current_loop_test_active = false; motor_pwm_port_emergency_stop(); return;
  }
  ab = motor_foc_clarke_q15(current.phase_a_q15, current.phase_b_q15);
  id_ma = (int32_t)(((int64_t)ab.alpha_q15 * MOTOR_CURRENT_Q15_BASE_MA) / 32768);
  iq_ma = (int32_t)(((int64_t)ab.beta_q15 * MOTOR_CURRENT_Q15_BASE_MA) / 32768);
  motor_current_loop_test_vd = motor_current_pi_axis_process(&motor_current_loop_test_d_pi,
    MOTOR_CURRENT_LOOP_TEST_DIRECT_REFERENCE_MA, id_ma);
  motor_current_loop_test_vq = motor_current_pi_axis_process(&motor_current_loop_test_q_pi, 0, iq_ma);
  motor_current_loop_test_voltage_apply(motor_current_loop_test_vd, motor_current_loop_test_vq);
  motor_current_loop_test_id_sum += id_ma; motor_current_loop_test_iq_sum += iq_ma;
  if (motor_current_loop_test_abs(id_ma) > motor_current_loop_test_id_peak) motor_current_loop_test_id_peak = motor_current_loop_test_abs(id_ma);
  if (motor_current_loop_test_abs(iq_ma) > motor_current_loop_test_iq_peak) motor_current_loop_test_iq_peak = motor_current_loop_test_abs(iq_ma);
  motor_current_loop_test_count++;
  if (motor_current_loop_test_count >= MOTOR_CURRENT_LOOP_TEST_DURATION_SAMPLES) {
    motor_current_loop_test_active = false; motor_current_loop_test_done = true; motor_pwm_port_output_disable();
  }
}

bool motor_current_loop_test_run_with_gains(
  motor_current_loop_test_result_t *result,
  int32_t direct_kp_q15,
  int32_t quadrature_kp_q15,
  int32_t integral_gain_q15)
{
  motor_slow_sensor_state_t sensor;
  if ((result == NULL) || (direct_kp_q15 <= 0) ||
      (quadrature_kp_q15 <= 0) || (integral_gain_q15 <= 0)) return false;
  (void)motor_slow_sensor_process();
  if ((!motor_slow_sensor_state_read(&sensor)) || motor_pwm_port_output_is_enabled()) return false;
  motor_current_loop_test_bus_mv = (uint32_t)sensor.bus_voltage_0p1v * 100U;
  motor_current_pi_axis_init(&motor_current_loop_test_d_pi, direct_kp_q15,
    integral_gain_q15, MOTOR_CURRENT_PI_OUTPUT_LIMIT_MV);
  motor_current_pi_axis_init(&motor_current_loop_test_q_pi, quadrature_kp_q15,
    integral_gain_q15, MOTOR_CURRENT_PI_OUTPUT_LIMIT_MV);
  motor_current_loop_test_count = 0U; motor_current_loop_test_id_sum = 0; motor_current_loop_test_iq_sum = 0;
  motor_current_loop_test_id_peak = 0; motor_current_loop_test_iq_peak = 0; motor_current_loop_test_vd = 0; motor_current_loop_test_vq = 0;
  motor_current_loop_test_done = false; motor_current_loop_test_fault = false;
  motor_current_loop_test_voltage_apply(0, 0);
  if (!motor_pwm_port_output_enable()) return false;
  motor_current_loop_test_active = true;
  while (motor_current_loop_test_active) wk_delay_ms(1U);
  motor_pwm_port_output_disable();
  if (motor_current_loop_test_fault || (!motor_current_loop_test_done) || (motor_current_loop_test_count == 0U)) return false;
  result->direct_average_ma = (int32_t)(motor_current_loop_test_id_sum / motor_current_loop_test_count);
  result->quadrature_average_ma = (int32_t)(motor_current_loop_test_iq_sum / motor_current_loop_test_count);
  result->direct_peak_ma = motor_current_loop_test_id_peak; result->quadrature_peak_ma = motor_current_loop_test_iq_peak;
  result->direct_voltage_mv = motor_current_loop_test_vd; result->quadrature_voltage_mv = motor_current_loop_test_vq;
  result->sample_count = motor_current_loop_test_count;
  return true;
}

bool motor_current_loop_test_run(motor_current_loop_test_result_t *result)
{
  return motor_current_loop_test_run_with_gains(
    result, MOTOR_CURRENT_PI_D_KP_Q15, MOTOR_CURRENT_PI_Q_KP_Q15,
    MOTOR_CURRENT_PI_KI_Q15);
}
