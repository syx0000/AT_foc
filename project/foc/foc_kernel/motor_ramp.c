#include <stddef.h>
#include "motor_ramp.h"

bool motor_ramp_init(motor_ramp_t *ramp, int32_t initial, int32_t target,
                     uint32_t rate_per_second)
{
  if ((ramp == NULL) || (rate_per_second == 0U))
  {
    return false;
  }
  ramp->current = initial;
  ramp->target = target;
  ramp->rate_per_second = rate_per_second;
  ramp->remainder = 0U;
  return true;
}

bool motor_ramp_target_set(motor_ramp_t *ramp, int32_t target,
                           uint32_t rate_per_second)
{
  if ((ramp == NULL) || (rate_per_second == 0U))
  {
    return false;
  }
  ramp->target = target;
  ramp->rate_per_second = rate_per_second;
  return true;
}

int32_t motor_ramp_process(motor_ramp_t *ramp,
                           uint32_t update_frequency_hz)
{
  int64_t difference;
  uint32_t step;

  if ((ramp == NULL) || (update_frequency_hz == 0U))
  {
    return 0;
  }
  difference = (int64_t)ramp->target - ramp->current;
  ramp->remainder += ramp->rate_per_second;
  step = ramp->remainder / update_frequency_hz;
  ramp->remainder %= update_frequency_hz;
  if ((step == 0U) || (difference == 0))
  {
    return ramp->current;
  }
  if ((difference <= (int64_t)step) && (difference >= -(int64_t)step))
  {
    ramp->current = ramp->target;
  }
  else if (difference > 0)
  {
    ramp->current += (int32_t)step;
  }
  else
  {
    ramp->current -= (int32_t)step;
  }
  return ramp->current;
}
