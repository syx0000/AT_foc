#include <stddef.h>
#include "at32f45x_conf.h"
#include "motor_board_config.h"
#include "motor_control_config.h"
#include "motor_current_sample.h"
#include "motor_foc_math.h"
#include "motor_inductance_identification.h"
#include "motor_pwm_port.h"
#include "motor_slow_sensor.h"
#include "wk_system.h"

typedef enum { IDENT_IDLE, IDENT_D, IDENT_Q, IDENT_DONE, IDENT_FAULT } ident_state_t;
static volatile ident_state_t ident_state;
static volatile uint32_t ident_count;
static volatile int64_t ident_sum_sin;
static volatile int64_t ident_sum_cos;
static volatile uint32_t ident_phase;
static volatile uint32_t ident_phase_step;
static volatile uint16_t ident_voltage_q15;
static volatile uint32_t ident_current_amplitude_q15;

static uint32_t ident_sqrt(uint32_t value)
{
  uint32_t result = 0U, bit = 1UL << 30;
  while (bit > value) bit >>= 2;
  while (bit != 0U) {
    if (value >= result + bit) { value -= result + bit; result = (result >> 1) + bit; }
    else result >>= 1;
    bit >>= 2;
  }
  return result;
}

static void ident_voltage_apply(int16_t vd, int16_t vq)
{
  motor_alpha_beta_q15_t voltage = motor_foc_inverse_park_q15(vd, vq, 0U);
  motor_svpwm_duty_q15_t duty = motor_foc_svpwm_q15(voltage);
  motor_pwm_compare_t compare;
  compare.phase_a = (uint16_t)(((uint32_t)duty.phase_a * MOTOR_PWM_PERIOD_COUNTS) / 32767U);
  compare.phase_b = (uint16_t)(((uint32_t)duty.phase_b * MOTOR_PWM_PERIOD_COUNTS) / 32767U);
  compare.phase_c = (uint16_t)(((uint32_t)duty.phase_c * MOTOR_PWM_PERIOD_COUNTS) / 32767U);
  motor_pwm_port_compare_set(&compare);
}

void motor_inductance_identification_fast_process(void)
{
  motor_current_sample_state_t current;
  motor_alpha_beta_q15_t ab;
  motor_sin_cos_q15_t trig;
  int16_t injection, measured;
  uint32_t amplitude;
  int32_t sin_component, cos_component;

  if ((ident_state != IDENT_D) && (ident_state != IDENT_Q)) return;
  if ((!motor_current_sample_state_read(&current)) ||
      (current.overcurrent_fault) ||
      ((uint32_t)((current.phase_a_ma < 0) ? -current.phase_a_ma : current.phase_a_ma) >= MOTOR_INDUCTANCE_IDENT_ABORT_CURRENT_MA) ||
      ((uint32_t)((current.phase_b_ma < 0) ? -current.phase_b_ma : current.phase_b_ma) >= MOTOR_INDUCTANCE_IDENT_ABORT_CURRENT_MA) ||
      ((uint32_t)((current.phase_c_ma < 0) ? -current.phase_c_ma : current.phase_c_ma) >= MOTOR_INDUCTANCE_IDENT_ABORT_CURRENT_MA) ||
      (MOTOR_PWM_TIMER->brk_bit.oen == 0U)) {
    ident_state = IDENT_FAULT;
    motor_pwm_port_emergency_stop();
    return;
  }

  ident_phase += ident_phase_step;
  trig = motor_foc_sin_cos_q15((uint16_t)(ident_phase >> 16));
  injection = (int16_t)(((int32_t)ident_voltage_q15 * trig.sin_q15) >> 15);
  ident_voltage_apply((ident_state == IDENT_D) ? injection : 0,
                      (ident_state == IDENT_Q) ? injection : 0);
  ab = motor_foc_clarke_q15(current.phase_a_q15, current.phase_b_q15);
  measured = (ident_state == IDENT_D) ? ab.alpha_q15 : ab.beta_q15;

  ident_count++;
  if (ident_count > MOTOR_INDUCTANCE_IDENT_SETTLE_SAMPLES) {
    ident_sum_sin += (int32_t)measured * trig.sin_q15;
    ident_sum_cos += (int32_t)measured * trig.cos_q15;
  }
  if (ident_count >= MOTOR_INDUCTANCE_IDENT_SETTLE_SAMPLES + MOTOR_INDUCTANCE_IDENT_MEASURE_SAMPLES) {
    sin_component = (int32_t)((ident_sum_sin * 2) /
      ((int64_t)MOTOR_INDUCTANCE_IDENT_MEASURE_SAMPLES * 32767));
    cos_component = (int32_t)((ident_sum_cos * 2) /
      ((int64_t)MOTOR_INDUCTANCE_IDENT_MEASURE_SAMPLES * 32767));
    amplitude = ident_sqrt((uint32_t)(sin_component * sin_component + cos_component * cos_component));
    ident_current_amplitude_q15 = amplitude;
    ident_state = IDENT_DONE;
    motor_pwm_port_output_disable();
  }
}

static bool ident_axis_run(ident_state_t axis, uint32_t *current_ma)
{
  ident_state = axis;
  ident_count = 0U; ident_sum_sin = 0; ident_sum_cos = 0; ident_phase = 0U;
  ident_voltage_apply(0, 0);
  if (!motor_pwm_port_output_enable()) { ident_state = IDENT_FAULT; return false; }
  while ((ident_state == axis)) wk_delay_ms(1U);
  if (ident_state != IDENT_DONE) return false;
  *current_ma = (ident_current_amplitude_q15 * (uint32_t)MOTOR_CURRENT_Q15_BASE_MA) / 32768U;
  return (*current_ma != 0U);
}

static uint32_t ident_inductance_get(uint16_t voltage_mv, uint32_t current_ma,
                                     uint32_t resistance_mohm)
{
  uint32_t impedance_mohm = ((uint32_t)voltage_mv * 1000U) / current_ma;
  uint32_t reactance_squared;
  uint32_t reactance_mohm;
  if (impedance_mohm <= resistance_mohm) return 0U;
  reactance_squared = impedance_mohm * impedance_mohm - resistance_mohm * resistance_mohm;
  reactance_mohm = ident_sqrt(reactance_squared);
  return (reactance_mohm * 1000U) / (6283U * MOTOR_INDUCTANCE_IDENT_FREQUENCY_HZ / 1000U);
}

bool motor_inductance_identification_run(uint32_t phase_resistance_mohm,
  motor_inductance_identification_result_t *result)
{
  motor_slow_sensor_state_t sensor;
  uint32_t bus_mv;
  if ((result == NULL) || (phase_resistance_mohm == 0U)) return false;
  (void)motor_slow_sensor_process();
  if ((!motor_slow_sensor_state_read(&sensor)) || motor_pwm_port_output_is_enabled()) return false;
  bus_mv = (uint32_t)sensor.bus_voltage_0p1v * 100U;
  ident_voltage_q15 = (uint16_t)(((uint64_t)MOTOR_INDUCTANCE_IDENT_VOLTAGE_MV * 32768U) / bus_mv);
  ident_phase_step = (uint32_t)(((uint64_t)MOTOR_INDUCTANCE_IDENT_FREQUENCY_HZ << 32) / MOTOR_PWM_FREQUENCY_HZ);
  result->injection_voltage_mv = (uint16_t)(((uint64_t)ident_voltage_q15 * bus_mv) / 32768U);
  if (!ident_axis_run(IDENT_D, &result->direct_current_amplitude_ma)) return false;
  wk_delay_ms(100U);
  if (!ident_axis_run(IDENT_Q, &result->quadrature_current_amplitude_ma)) return false;
  result->direct_inductance_uh = ident_inductance_get(result->injection_voltage_mv,
    result->direct_current_amplitude_ma, phase_resistance_mohm);
  result->quadrature_inductance_uh = ident_inductance_get(result->injection_voltage_mv,
    result->quadrature_current_amplitude_ma, phase_resistance_mohm);
  ident_state = IDENT_IDLE;
  return (result->direct_inductance_uh != 0U) && (result->quadrature_inductance_uh != 0U);
}
