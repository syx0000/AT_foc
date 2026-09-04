/**
 * @file    motor_direction.c
 * @brief   电机逻辑方向变换 功能实现
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

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
