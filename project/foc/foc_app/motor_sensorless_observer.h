/**
 * @file    motor_sensorless_observer.h
 * @brief   Shadow back-EMF/PLL observer for diagnostic use only.
 */

#ifndef MOTOR_SENSORLESS_OBSERVER_H
#define MOTOR_SENSORLESS_OBSERVER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint16_t electrical_angle_u16;
  int32_t electrical_frequency_millihz;
  int32_t bemf_alpha_mv;
  int32_t bemf_beta_mv;
  int16_t phase_error_u16;
  uint16_t confidence;                 /**< 0..1000 plausibility score. */
  uint32_t sample_count;
  bool valid;
} motor_sensorless_observer_state_t;

/** @brief Initialize the shadow observer from the active motor parameters. */
void motor_sensorless_observer_init(void);

/** @brief Reload Rs/Ld/Lq/flux after a runtime parameter change. */
bool motor_sensorless_observer_parameter_reload(void);

/** @brief Clear dynamic observer state without changing its parameters. */
void motor_sensorless_observer_reset(void);

/**
 * @brief Process one 10 kHz sample.
 * @param voltage_alpha_mv Actual limited alpha voltage applied by SVPWM.
 * @param voltage_beta_mv  Actual limited beta voltage applied by SVPWM.
 * @param current_alpha_ma Sampled alpha current.
 * @param current_beta_ma  Sampled beta current.
 * @details This function only updates diagnostic state.  It never changes
 *          the FOC angle, speed feedback, command, or protection state.
 */
void motor_sensorless_observer_update(int32_t voltage_alpha_mv,
                                      int32_t voltage_beta_mv,
                                      int32_t current_alpha_ma,
                                      int32_t current_beta_ma);

/** @brief Read an atomic observer snapshot. */
bool motor_sensorless_observer_read(motor_sensorless_observer_state_t *state);

#endif /* MOTOR_SENSORLESS_OBSERVER_H */
