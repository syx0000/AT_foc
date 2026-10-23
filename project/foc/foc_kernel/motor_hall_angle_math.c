/**
 * @file    motor_hall_angle_math.c
 * @brief   Hall角度定点算法 功能实现
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

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

bool motor_hall_sector_span_get(const uint8_t positive_next[8],
                                const uint16_t entry_angle_u16[8],
                                uint8_t state,
                                uint16_t *span_u16)
{
  uint8_t next_state;

  if ((positive_next == NULL) || (entry_angle_u16 == NULL) ||
      (span_u16 == NULL) || (state < 1U) || (state > 6U))
    return false;
  next_state = positive_next[state];
  if ((next_state < 1U) || (next_state > 6U)) return false;
  *span_u16 = (uint16_t)(entry_angle_u16[next_state] -
                         entry_angle_u16[state]);
  return (*span_u16 != 0U);
}

bool motor_hall_dynamic_compensation_get(
  uint32_t electrical_frequency_millihz,
  int8_t physical_direction,
  uint32_t delay_us,
  uint16_t maximum_compensation_u16,
  int32_t *compensation_u16)
{
  uint64_t magnitude;

  if ((compensation_u16 == NULL) ||
      ((physical_direction != 1) && (physical_direction != -1)) ||
      (maximum_compensation_u16 == 0U) ||
      (maximum_compensation_u16 >= 32768U))
  {
    return false;
  }
  if ((delay_us == 0U) || (electrical_frequency_millihz == 0U))
  {
    *compensation_u16 = 0;
    return true;
  }

  /* f[mHz] * delay[us] * 65536 / 1e9 = U16一周制补偿角。 */
  magnitude = ((uint64_t)electrical_frequency_millihz * delay_us * 65536U +
               500000000ULL) / 1000000000ULL;
  if (magnitude > maximum_compensation_u16)
    magnitude = maximum_compensation_u16;
  *compensation_u16 = (physical_direction > 0) ?
    (int32_t)magnitude : -(int32_t)magnitude;
  return true;
}
