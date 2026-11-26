/**
 * @file    motor_voltage_limit.c
 * @brief   dq电压矢量限幅算法 功能实现
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#include <stddef.h>
#include "motor_voltage_limit.h"

/* Restoring integer square root. Round upward to keep the output inside the circle. */
static uint64_t motor_voltage_magnitude_ceil(uint64_t square)
{
  uint64_t root = 0;
  uint64_t bit = UINT64_C(1) << 62;

  while (bit > square) bit >>= 2;
  while (bit != 0)
  {
    if (square >= root + bit)
    {
      square -= root + bit;
      root = (root >> 1) + bit;
    }
    else
    {
      root >>= 1;
    }
    bit >>= 2;
  }
  return root + (square != 0);
}

bool motor_voltage_limit_apply(int32_t *direct_mv,
                               int32_t *quadrature_mv,
                               int32_t limit_mv)
{
  if (direct_mv == NULL || quadrature_mv == NULL || limit_mv <= 0)
  {
    return false;
  }

  const int64_t direct = *direct_mv;
  const int64_t quadrature = *quadrature_mv;
  const uint64_t square = (uint64_t)(direct * direct) +
                          (uint64_t)(quadrature * quadrature);
  if (square <= (uint64_t)limit_mv * (uint64_t)limit_mv)
  {
    return false;
  }

  const int64_t magnitude = (int64_t)motor_voltage_magnitude_ceil(square);
  *direct_mv = (int32_t)(direct * limit_mv / magnitude);
  *quadrature_mv = (int32_t)(quadrature * limit_mv / magnitude);
  return true;
}
