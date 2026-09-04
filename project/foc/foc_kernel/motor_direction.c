#include <limits.h>
#include <stddef.h>
#include "motor_direction.h"

int8_t motor_direction_sign_get(bool direction_inverted)
{
  return direction_inverted ? -1 : 1;
}

bool motor_direction_transform_s32(int32_t logical_value,
                                   bool direction_inverted,
                                   int32_t *physical_value)
{
  if ((physical_value == NULL) ||
      (direction_inverted && (logical_value == INT32_MIN)))
  {
    return false;
  }
  *physical_value = direction_inverted ? -logical_value : logical_value;
  return true;
}
