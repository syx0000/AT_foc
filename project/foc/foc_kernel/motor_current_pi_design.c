#include <limits.h>
#include <stddef.h>
#include "motor_current_pi_design.h"

#define MOTOR_TWO_PI_SCALED 6283185ULL
#define MOTOR_PI_SCALE      1000000ULL

bool motor_current_pi_design(uint32_t phase_resistance_mohm,
                             uint32_t direct_inductance_uh,
                             uint32_t quadrature_inductance_uh,
                             uint32_t bandwidth_hz,
                             uint32_t sample_frequency_hz,
                             motor_current_pi_design_result_t *result)
{
  uint64_t d_kp;
  uint64_t q_kp;
  uint64_t ki;

  if ((result == NULL) || (phase_resistance_mohm == 0U) ||
      (direct_inductance_uh == 0U) || (quadrature_inductance_uh == 0U) ||
      (bandwidth_hz == 0U) || (sample_frequency_hz <= bandwidth_hz))
    return false;
  d_kp = ((uint64_t)direct_inductance_uh * MOTOR_TWO_PI_SCALED *
          bandwidth_hz * 32768U + 500000000000ULL) / 1000000000000ULL;
  q_kp = ((uint64_t)quadrature_inductance_uh * MOTOR_TWO_PI_SCALED *
          bandwidth_hz * 32768U + 500000000000ULL) / 1000000000000ULL;
  ki = ((uint64_t)phase_resistance_mohm * MOTOR_TWO_PI_SCALED *
        bandwidth_hz * 32768U +
        ((MOTOR_PI_SCALE * 1000U * sample_frequency_hz) / 2U)) /
       (MOTOR_PI_SCALE * 1000U * sample_frequency_hz);
  if ((d_kp > INT32_MAX) || (q_kp > INT32_MAX) || (ki > INT32_MAX))
    return false;
  result->direct_kp_q15 = (int32_t)d_kp;
  result->quadrature_kp_q15 = (int32_t)q_kp;
  result->integral_gain_q15 = (int32_t)ki;
  return true;
}
