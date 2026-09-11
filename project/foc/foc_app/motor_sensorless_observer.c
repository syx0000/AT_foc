/**
 * @file    motor_sensorless_observer.c
 * @brief   Shadow back-EMF/PLL observer for diagnostic use only.
 *
 * The implementation intentionally uses integer arithmetic so it can run in
 * the ADC current-loop ISR on the same targets as the existing fixed-point
 * FOC. The voltage input is the post-limit alpha/beta vector actually sent
 * to SVPWM. No observer state is used by FOC, speed control, or protection.
 */

#include <stddef.h>
#include "at32f45x.h"
#include "motor_control_config.h"
#include "motor_parameter.h"
#include "motor_sensorless_observer.h"

#define MOTOR_SENSORLESS_OBSERVER_TWO_PI_X1000 6283LL
#define MOTOR_SENSORLESS_OBSERVER_CONFIDENCE_MAX 1000U

static volatile motor_sensorless_observer_state_t
  motor_sensorless_observer_state;
static uint32_t motor_sensorless_observer_resistance_mohm;
static uint32_t motor_sensorless_observer_inductance_uh;
static uint32_t motor_sensorless_observer_flux_uwb;
static int32_t motor_sensorless_observer_previous_current_alpha_ma;
static int32_t motor_sensorless_observer_previous_current_beta_ma;
static int32_t motor_sensorless_observer_current_derivative_alpha_ma;
static int32_t motor_sensorless_observer_current_derivative_beta_ma;
static uint16_t motor_sensorless_observer_previous_measure_angle_u16;
static bool motor_sensorless_observer_sample_initialized;
static bool motor_sensorless_observer_angle_initialized;
static bool motor_sensorless_observer_previous_measure_valid;
static uint32_t motor_sensorless_observer_valid_samples;

static uint32_t motor_sensorless_observer_abs32(int32_t value)
{
  return (value < 0) ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
}

static uint16_t motor_sensorless_observer_atan2_u16(int32_t y, int32_t x)
{
  uint32_t abs_y = motor_sensorless_observer_abs32(y);
  uint32_t abs_x = motor_sensorless_observer_abs32(x);
  uint32_t ratio;
  int32_t angle;

  if ((abs_x == 0U) && (abs_y == 0U)) return 0U;
  /* Bounded linearized atan approximation over each 90 degree quadrant. */
  if (abs_x >= abs_y)
  {
    ratio = (uint32_t)(((uint64_t)(abs_x - abs_y) * 8192U) /
                       (abs_x + abs_y));
    angle = 8192 - (int32_t)ratio;
  }
  else
  {
    ratio = (uint32_t)(((uint64_t)(abs_y - abs_x) * 8192U) /
                       (abs_x + abs_y));
    angle = 8192 + (int32_t)ratio;
  }
  if (x < 0) angle = 32768 - angle;
  if (y < 0) angle = -angle;
  return (uint16_t)angle;
}

static int32_t motor_sensorless_observer_phase_error(uint16_t target,
                                                      uint16_t actual)
{
  return (int32_t)(int16_t)(target - actual);
}

static int32_t motor_sensorless_observer_frequency_step(
  int32_t frequency_millihz)
{
  int64_t numerator = (int64_t)frequency_millihz *
    MOTOR_SENSORLESS_OBSERVER_SAMPLE_PERIOD_US * 65536LL;
  return (int32_t)(numerator / 1000000000LL);
}

static int32_t motor_sensorless_observer_frequency_from_angle_delta(
  int32_t angle_delta_u16)
{
  int64_t numerator = (int64_t)angle_delta_u16 * 1000000000LL;
  return (int32_t)(numerator /
    (65536LL * MOTOR_SENSORLESS_OBSERVER_SAMPLE_PERIOD_US));
}

static int32_t motor_sensorless_observer_frequency_clamp(int32_t frequency)
{
  if (frequency > MOTOR_SENSORLESS_OBSERVER_MAX_FREQUENCY_MILLIHZ)
    return MOTOR_SENSORLESS_OBSERVER_MAX_FREQUENCY_MILLIHZ;
  if (frequency < -MOTOR_SENSORLESS_OBSERVER_MAX_FREQUENCY_MILLIHZ)
    return -MOTOR_SENSORLESS_OBSERVER_MAX_FREQUENCY_MILLIHZ;
  return frequency;
}

static uint16_t motor_sensorless_observer_confidence(uint32_t magnitude_mv,
                                                      int32_t frequency_millihz,
                                                      int32_t phase_error_u16)
{
  uint32_t amplitude_score;
  uint32_t phase_score;
  uint32_t speed_score;
  uint32_t expected_mv;
  uint32_t ratio_score;
  uint32_t frequency_abs = motor_sensorless_observer_abs32(
    frequency_millihz);
  uint32_t phase_abs = motor_sensorless_observer_abs32(phase_error_u16);
  uint32_t confidence;

  amplitude_score = (magnitude_mv * MOTOR_SENSORLESS_OBSERVER_CONFIDENCE_MAX) /
                    MOTOR_SENSORLESS_OBSERVER_MIN_BEMF_MV;
  if (amplitude_score > MOTOR_SENSORLESS_OBSERVER_CONFIDENCE_MAX)
    amplitude_score = MOTOR_SENSORLESS_OBSERVER_CONFIDENCE_MAX;
  phase_score = MOTOR_SENSORLESS_OBSERVER_CONFIDENCE_MAX;
  if (phase_abs >= 16384U)
    phase_score = 0U;
  else
    phase_score -= (phase_abs * MOTOR_SENSORLESS_OBSERVER_CONFIDENCE_MAX) /
                   16384U;
  speed_score = (frequency_abs * MOTOR_SENSORLESS_OBSERVER_CONFIDENCE_MAX) /
                MOTOR_SENSORLESS_OBSERVER_MIN_FREQUENCY_MILLIHZ;
  if (speed_score > MOTOR_SENSORLESS_OBSERVER_CONFIDENCE_MAX)
    speed_score = MOTOR_SENSORLESS_OBSERVER_CONFIDENCE_MAX;

  /* E[mV] ~= flux[uWb] * f[mHz] * 2*pi * 1e-6. Use a soft 1:4..4:1
     amplitude window to reject switching ripple without hard limiting FOC. */
  expected_mv = (uint32_t)(((uint64_t)motor_sensorless_observer_flux_uwb *
                            frequency_abs *
                            MOTOR_SENSORLESS_OBSERVER_TWO_PI_X1000) /
                           1000000000ULL);
  ratio_score = MOTOR_SENSORLESS_OBSERVER_CONFIDENCE_MAX;
  if (expected_mv >= MOTOR_SENSORLESS_OBSERVER_MIN_BEMF_MV)
  {
    if ((magnitude_mv * 4U) < expected_mv)
      ratio_score = (magnitude_mv * 4U *
                     MOTOR_SENSORLESS_OBSERVER_CONFIDENCE_MAX) / expected_mv;
    else if (magnitude_mv > (expected_mv * 4U))
      ratio_score = (expected_mv *
                     MOTOR_SENSORLESS_OBSERVER_CONFIDENCE_MAX) / magnitude_mv;
  }
  confidence = amplitude_score;
  if (phase_score < confidence) confidence = phase_score;
  if (speed_score < confidence) confidence = speed_score;
  if (ratio_score < confidence) confidence = ratio_score;
  return (uint16_t)confidence;
}

static void motor_sensorless_observer_dynamic_reset(void)
{
  motor_sensorless_observer_previous_current_alpha_ma = 0;
  motor_sensorless_observer_previous_current_beta_ma = 0;
  motor_sensorless_observer_current_derivative_alpha_ma = 0;
  motor_sensorless_observer_current_derivative_beta_ma = 0;
  motor_sensorless_observer_previous_measure_angle_u16 = 0U;
  motor_sensorless_observer_sample_initialized = false;
  motor_sensorless_observer_angle_initialized = false;
  motor_sensorless_observer_previous_measure_valid = false;
  motor_sensorless_observer_valid_samples = 0U;
  motor_sensorless_observer_state.electrical_angle_u16 = 0U;
  motor_sensorless_observer_state.electrical_frequency_millihz = 0L;
  motor_sensorless_observer_state.bemf_alpha_mv = 0L;
  motor_sensorless_observer_state.bemf_beta_mv = 0L;
  motor_sensorless_observer_state.phase_error_u16 = 0;
  motor_sensorless_observer_state.confidence = 0U;
  motor_sensorless_observer_state.sample_count = 0U;
  motor_sensorless_observer_state.valid = false;
}

void motor_sensorless_observer_init(void)
{
  motor_sensorless_observer_resistance_mohm = MOTOR_PHASE_RESISTANCE_MOHM;
  motor_sensorless_observer_inductance_uh =
    (MOTOR_DIRECT_INDUCTANCE_UH + MOTOR_QUADRATURE_INDUCTANCE_UH) / 2U;
  motor_sensorless_observer_flux_uwb = MOTOR_FLUX_LINKAGE_UWB;
  motor_sensorless_observer_dynamic_reset();
  (void)motor_sensorless_observer_parameter_reload();
}

bool motor_sensorless_observer_parameter_reload(void)
{
  motor_parameter_t parameter;
  uint32_t primask;

  if (!motor_parameter_active_read(&parameter)) return false;
  if ((parameter.phase_resistance_mohm == 0U) ||
      (parameter.direct_inductance_uh == 0U) ||
      (parameter.quadrature_inductance_uh == 0U) ||
      (parameter.flux_linkage_uwb == 0U))
    return false;
  primask = __get_PRIMASK();
  __disable_irq();
  motor_sensorless_observer_resistance_mohm =
    parameter.phase_resistance_mohm;
  motor_sensorless_observer_inductance_uh =
    (parameter.direct_inductance_uh + parameter.quadrature_inductance_uh) / 2U;
  motor_sensorless_observer_flux_uwb = parameter.flux_linkage_uwb;
  motor_sensorless_observer_dynamic_reset();
  if (primask == 0U) __enable_irq();
  return true;
}

void motor_sensorless_observer_reset(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  motor_sensorless_observer_dynamic_reset();
  __set_PRIMASK(primask);
}

void motor_sensorless_observer_update(int32_t voltage_alpha_mv,
                                      int32_t voltage_beta_mv,
                                      int32_t current_alpha_ma,
                                      int32_t current_beta_ma)
{
  int32_t delta_alpha_ma;
  int32_t delta_beta_ma;
  int32_t resistive_alpha_mv;
  int32_t resistive_beta_mv;
  int32_t inductive_alpha_mv;
  int32_t inductive_beta_mv;
  int32_t raw_bemf_alpha_mv;
  int32_t raw_bemf_beta_mv;
  int32_t phase_measure_error;
  int32_t measured_frequency_millihz;
  int32_t phase_error_u16 = 0;
  int32_t angle_step_u16;
  int32_t max_bemf_mv;
  int32_t min_bemf_mv;
  uint32_t magnitude_mv;
  uint16_t measure_angle_u16 = 0U;
  bool measure_valid;

  if (!motor_sensorless_observer_sample_initialized)
  {
    motor_sensorless_observer_previous_current_alpha_ma = current_alpha_ma;
    motor_sensorless_observer_previous_current_beta_ma = current_beta_ma;
    motor_sensorless_observer_sample_initialized = true;
    return;
  }
  delta_alpha_ma = current_alpha_ma -
    motor_sensorless_observer_previous_current_alpha_ma;
  delta_beta_ma = current_beta_ma -
    motor_sensorless_observer_previous_current_beta_ma;
  motor_sensorless_observer_previous_current_alpha_ma = current_alpha_ma;
  motor_sensorless_observer_previous_current_beta_ma = current_beta_ma;
  motor_sensorless_observer_current_derivative_alpha_ma +=
    (delta_alpha_ma - motor_sensorless_observer_current_derivative_alpha_ma) >>
    MOTOR_SENSORLESS_OBSERVER_CURRENT_FILTER_SHIFT;
  motor_sensorless_observer_current_derivative_beta_ma +=
    (delta_beta_ma - motor_sensorless_observer_current_derivative_beta_ma) >>
    MOTOR_SENSORLESS_OBSERVER_CURRENT_FILTER_SHIFT;

  resistive_alpha_mv = (int32_t)(((int64_t)
    motor_sensorless_observer_resistance_mohm * current_alpha_ma) / 1000LL);
  resistive_beta_mv = (int32_t)(((int64_t)
    motor_sensorless_observer_resistance_mohm * current_beta_ma) / 1000LL);
  inductive_alpha_mv = (int32_t)(((int64_t)
    motor_sensorless_observer_inductance_uh *
    motor_sensorless_observer_current_derivative_alpha_ma) /
    MOTOR_SENSORLESS_OBSERVER_SAMPLE_PERIOD_US);
  inductive_beta_mv = (int32_t)(((int64_t)
    motor_sensorless_observer_inductance_uh *
    motor_sensorless_observer_current_derivative_beta_ma) /
    MOTOR_SENSORLESS_OBSERVER_SAMPLE_PERIOD_US);
  raw_bemf_alpha_mv = voltage_alpha_mv - resistive_alpha_mv -
                      inductive_alpha_mv;
  raw_bemf_beta_mv = voltage_beta_mv - resistive_beta_mv -
                     inductive_beta_mv;
  motor_sensorless_observer_state.bemf_alpha_mv +=
    (raw_bemf_alpha_mv - motor_sensorless_observer_state.bemf_alpha_mv) >>
    MOTOR_SENSORLESS_OBSERVER_BEMF_FILTER_SHIFT;
  motor_sensorless_observer_state.bemf_beta_mv +=
    (raw_bemf_beta_mv - motor_sensorless_observer_state.bemf_beta_mv) >>
    MOTOR_SENSORLESS_OBSERVER_BEMF_FILTER_SHIFT;

  {
    int32_t abs_alpha = (int32_t)motor_sensorless_observer_abs32(
      motor_sensorless_observer_state.bemf_alpha_mv);
    int32_t abs_beta = (int32_t)motor_sensorless_observer_abs32(
      motor_sensorless_observer_state.bemf_beta_mv);
    max_bemf_mv = (abs_alpha > abs_beta) ? abs_alpha : abs_beta;
    min_bemf_mv = (abs_alpha < abs_beta) ? abs_alpha : abs_beta;
  }
  magnitude_mv = (uint32_t)(max_bemf_mv + ((min_bemf_mv * 3) >> 3));
  measure_valid = magnitude_mv >= MOTOR_SENSORLESS_OBSERVER_MIN_BEMF_MV;
  if (measure_valid)
  {
    /* e = d(psi)/dt, so rotor flux is 90 electrical degrees behind e. */
    measure_angle_u16 = (uint16_t)(
      motor_sensorless_observer_atan2_u16(
        motor_sensorless_observer_state.bemf_beta_mv,
        motor_sensorless_observer_state.bemf_alpha_mv) - 16384U);
    if (!motor_sensorless_observer_angle_initialized)
    {
      motor_sensorless_observer_state.electrical_angle_u16 =
        measure_angle_u16;
      motor_sensorless_observer_angle_initialized = true;
      motor_sensorless_observer_previous_measure_valid = false;
      motor_sensorless_observer_state.electrical_frequency_millihz = 0L;
    }
    else
    {
      phase_measure_error = motor_sensorless_observer_phase_error(
        measure_angle_u16,
        motor_sensorless_observer_previous_measure_angle_u16);
      if (motor_sensorless_observer_previous_measure_valid)
      {
        measured_frequency_millihz =
          motor_sensorless_observer_frequency_from_angle_delta(
            phase_measure_error);
        measured_frequency_millihz =
          motor_sensorless_observer_frequency_clamp(measured_frequency_millihz);
        motor_sensorless_observer_state.electrical_frequency_millihz +=
          (measured_frequency_millihz -
           motor_sensorless_observer_state.electrical_frequency_millihz) >>
          MOTOR_SENSORLESS_OBSERVER_PLL_FILTER_SHIFT;
      }
      phase_error_u16 = motor_sensorless_observer_phase_error(
        measure_angle_u16,
        motor_sensorless_observer_state.electrical_angle_u16);
      angle_step_u16 = motor_sensorless_observer_frequency_step(
        motor_sensorless_observer_state.electrical_frequency_millihz);
      motor_sensorless_observer_state.electrical_angle_u16 = (uint16_t)(
        motor_sensorless_observer_state.electrical_angle_u16 + angle_step_u16 +
        (phase_error_u16 >> MOTOR_SENSORLESS_OBSERVER_PLL_PHASE_SHIFT));
    }
    motor_sensorless_observer_previous_measure_angle_u16 = measure_angle_u16;
    motor_sensorless_observer_previous_measure_valid = true;
    if (motor_sensorless_observer_valid_samples <
        MOTOR_SENSORLESS_OBSERVER_VALID_SAMPLES)
      motor_sensorless_observer_valid_samples++;
  }
  else
  {
    motor_sensorless_observer_previous_measure_valid = false;
    motor_sensorless_observer_state.electrical_frequency_millihz -=
      motor_sensorless_observer_state.electrical_frequency_millihz >> 5;
    if (motor_sensorless_observer_valid_samples != 0U)
      motor_sensorless_observer_valid_samples--;
    angle_step_u16 = motor_sensorless_observer_frequency_step(
      motor_sensorless_observer_state.electrical_frequency_millihz);
    motor_sensorless_observer_state.electrical_angle_u16 = (uint16_t)(
      motor_sensorless_observer_state.electrical_angle_u16 + angle_step_u16);
  }
  if (measure_valid)
    phase_error_u16 = motor_sensorless_observer_phase_error(
      measure_angle_u16, motor_sensorless_observer_state.electrical_angle_u16);
  motor_sensorless_observer_state.phase_error_u16 = (int16_t)phase_error_u16;
  motor_sensorless_observer_state.confidence =
    motor_sensorless_observer_confidence(
      magnitude_mv,
      motor_sensorless_observer_state.electrical_frequency_millihz,
      phase_error_u16);
  motor_sensorless_observer_state.valid = measure_valid &&
    (motor_sensorless_observer_valid_samples >=
     MOTOR_SENSORLESS_OBSERVER_VALID_SAMPLES) &&
    (motor_sensorless_observer_abs32(
       motor_sensorless_observer_state.electrical_frequency_millihz) >=
     MOTOR_SENSORLESS_OBSERVER_MIN_FREQUENCY_MILLIHZ) &&
    (motor_sensorless_observer_state.confidence >= 200U);
  motor_sensorless_observer_state.sample_count++;
}

bool motor_sensorless_observer_read(motor_sensorless_observer_state_t *state)
{
  uint32_t primask;

  if (state == NULL) return false;
  primask = __get_PRIMASK();
  __disable_irq();
  *state = motor_sensorless_observer_state;
  __set_PRIMASK(primask);
  return true;
}
