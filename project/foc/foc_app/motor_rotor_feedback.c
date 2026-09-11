/**
 * @file    motor_rotor_feedback.c
 * @brief   通用转子反馈接口 功能实现
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#include <stddef.h>
#include <motor_control_config.h>
#include "at32f45x.h"
#include "motor_hall_angle_estimator.h"
#include "motor_hall_angle_math.h"
#include "motor_hall_decoder.h"
#include "motor_parameter.h"
#include "motor_rotor_feedback.h"
#include "motor_timebase.h"

static volatile motor_rotor_feedback_t motor_rotor_feedback_override;
static volatile bool motor_rotor_feedback_override_enabled;

void motor_rotor_feedback_init(void)
{
  motor_rotor_feedback_override_enabled = false;
}

void motor_rotor_feedback_override_set(uint16_t electrical_angle_u16,
                                       int32_t electrical_frequency_millihz,
                                       int8_t direction)
{
  uint32_t primask;
  uint32_t magnitude = (electrical_frequency_millihz < 0) ?
    (uint32_t)(-electrical_frequency_millihz) :
    (uint32_t)electrical_frequency_millihz;
  uint8_t pole_pairs = motor_parameter_pole_pairs_get();
  uint32_t mechanical_speed_millirpm = (pole_pairs == 0U) ? 0U :
    (uint32_t)(((uint64_t)magnitude * 60U) / pole_pairs);
  primask = __get_PRIMASK();
  __disable_irq();
  motor_rotor_feedback_override.source =
    MOTOR_ROTOR_FEEDBACK_SOURCE_CURRENT_STARTUP;
  motor_rotor_feedback_override.electrical_angle_u16 = electrical_angle_u16;
  motor_rotor_feedback_override.mechanical_angle_u16 = 0U;
  motor_rotor_feedback_override.mechanical_speed_millirpm =
    (direction < 0) ? -(int32_t)mechanical_speed_millirpm :
                      (int32_t)mechanical_speed_millirpm;
  motor_rotor_feedback_override.electrical_frequency_millihz = magnitude;
  motor_rotor_feedback_override.timestamp_cycles = motor_timebase_cycles_get();
  motor_rotor_feedback_override.update_count++;
  motor_rotor_feedback_override.source_state = 0U;
  motor_rotor_feedback_override.direction = direction;
  motor_rotor_feedback_override.mechanical_angle_valid = false;
  motor_rotor_feedback_override.valid = true;
  motor_rotor_feedback_override_enabled = true;
  __set_PRIMASK(primask);
}

void motor_rotor_feedback_override_clear(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  motor_rotor_feedback_override_enabled = false;
  __set_PRIMASK(primask);
}

bool motor_rotor_feedback_override_active(void)
{
  return motor_rotor_feedback_override_enabled;
}

bool motor_rotor_feedback_read(motor_rotor_feedback_t *feedback)
{
  motor_hall_angle_estimator_t estimator;
  motor_hall_sample_t hall;
  motor_parameter_t parameter;
  uint16_t sector_span_u16;
  uint16_t standstill_angle_u16;
  uint8_t pole_pairs;
  uint32_t mechanical_speed_millirpm;

  uint32_t primask;

  if (feedback == NULL) return false;
  primask = __get_PRIMASK();
  __disable_irq();
  if (motor_rotor_feedback_override_enabled)
  {
    *feedback = motor_rotor_feedback_override;
    __set_PRIMASK(primask);
    return true;
  }
  __set_PRIMASK(primask);
  if (
      (!motor_hall_angle_estimator_read(&estimator)) ||
      (!motor_hall_decoder_sample_read(&hall)))
  {
    return false;
  }

  pole_pairs = motor_parameter_pole_pairs_get();
  if (pole_pairs == 0U)
  {
    return false;
  }
  /* A missing Hall edge means zero speed at a commanded-current stall, not
   * necessarily a lost position sensor.  The interpolating estimator times
   * out because it cannot extrapolate indefinitely, while the discrete Hall
   * state still identifies a bounded 60-degree electrical sector.  Fall back
   * to that sector's calibrated centre so the current loop can keep producing
   * limited holding torque.  Keep the old Hall timestamp and zero direction;
   * the speed-feedback layer will therefore still reject this as a valid
   * moving-speed sample, preserving speed-loop stall/timeout protection. */
  if ((!estimator.valid) && hall.valid &&
      motor_parameter_active_read(&parameter) &&
      motor_hall_sector_span_get(parameter.hall_positive_next,
                                 parameter.hall_entry_angle_u16,
                                 hall.state, &sector_span_u16))
  {
    standstill_angle_u16 = (uint16_t)(
      parameter.hall_entry_angle_u16[hall.state] + sector_span_u16 / 2U +
      parameter.hall_rotor_offset_u16);
    feedback->source = MOTOR_ROTOR_FEEDBACK_SOURCE_HALL;
    feedback->electrical_angle_u16 = standstill_angle_u16;
    feedback->mechanical_angle_u16 = 0U;
    feedback->mechanical_speed_millirpm = 0;
    feedback->electrical_frequency_millihz = 0U;
    feedback->timestamp_cycles = motor_timebase_cycles_get();
    feedback->update_count = hall.frequency_update_count;
    feedback->source_state = hall.state;
    feedback->direction = 0;
    feedback->mechanical_angle_valid = false;
    feedback->valid = true;
    return true;
  }
  mechanical_speed_millirpm = (uint32_t)(
    ((uint64_t)estimator.electrical_frequency_millihz * 60U) / pole_pairs);
  feedback->source = MOTOR_ROTOR_FEEDBACK_SOURCE_HALL;
  feedback->electrical_angle_u16 = estimator.electrical_angle_u16;
  feedback->mechanical_angle_u16 = 0U;
  feedback->mechanical_speed_millirpm = (estimator.direction < 0) ?
    -(int32_t)mechanical_speed_millirpm : (int32_t)mechanical_speed_millirpm;
  feedback->electrical_frequency_millihz =
    estimator.electrical_frequency_millihz;
  feedback->timestamp_cycles = hall.timestamp_cycles;
  feedback->update_count = hall.frequency_update_count;
  feedback->source_state = hall.state;
  feedback->direction = estimator.direction;
  feedback->mechanical_angle_valid = false;
  feedback->valid = estimator.valid && hall.valid &&
                    (estimator.direction != 0) &&
                    (estimator.electrical_frequency_millihz != 0U);
  return true;
}
