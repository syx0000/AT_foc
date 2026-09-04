#include <limits.h>
#include <stddef.h>
#include "motor_hall_angle_math.h"

bool motor_hall_angle_boundary_get(const uint8_t positive_next[8],
                                   const uint16_t entry_angle_u16[8],
                                   uint8_t current_state,
                                   int8_t physical_direction,
                                   uint16_t rotor_offset_u16,
                                   uint16_t *boundary_angle_u16)
{
  uint8_t boundary_state;

  if ((positive_next == NULL) || (entry_angle_u16 == NULL) ||
      (boundary_angle_u16 == NULL) || (current_state < 1U) ||
      (current_state > 6U) ||
      ((physical_direction != 1) && (physical_direction != -1)))
  {
    return false;
  }
  boundary_state = (physical_direction > 0) ? current_state :
    positive_next[current_state];
  if ((boundary_state < 1U) || (boundary_state > 6U)) return false;
  *boundary_angle_u16 = (uint16_t)(
    entry_angle_u16[boundary_state] + rotor_offset_u16);
  return true;
}

bool motor_hall_phase_step_get(uint32_t electrical_frequency_millihz,
                               uint32_t update_frequency_hz,
                               int8_t physical_direction,
                               int32_t *phase_step)
{
  uint64_t magnitude;

  if ((electrical_frequency_millihz == 0U) ||
      (update_frequency_hz == 0U) || (phase_step == NULL) ||
      ((physical_direction != 1) && (physical_direction != -1)))
  {
    return false;
  }
  magnitude = ((uint64_t)electrical_frequency_millihz << 32) /
              ((uint64_t)update_frequency_hz * 1000U);
  if (magnitude > INT32_MAX) return false;
  *phase_step = (physical_direction > 0) ?
    (int32_t)magnitude : -(int32_t)magnitude;
  return true;
}
