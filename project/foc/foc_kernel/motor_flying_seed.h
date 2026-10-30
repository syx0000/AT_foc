#ifndef MOTOR_FLYING_SEED_H
#define MOTOR_FLYING_SEED_H
#include <stdbool.h>
#include <stdint.h>
/* Back-EMF feedforward estimate, not a terminal-voltage measurement. */
static bool motor_flying_seed_mv(uint32_t flux_uwb, uint32_t frequency_millihz,
                                 int8_t direction, int32_t limit_mv,
                                 int32_t *seed_mv)
{
  uint64_t product;
  uint64_t magnitude;
  if (!seed_mv || !flux_uwb || limit_mv <= 0 ||
      (direction != 1 && direction != -1)) return false;
  product = (uint64_t)flux_uwb * frequency_millihz;
  if (product > UINT64_MAX / 6283U) return false;
  magnitude = product * 6283U / 1000000000ULL;
  if (magnitude > (uint32_t)limit_mv) magnitude = (uint32_t)limit_mv;
  *seed_mv = direction < 0 ? -(int32_t)magnitude : (int32_t)magnitude;
  return true;
}
#endif
