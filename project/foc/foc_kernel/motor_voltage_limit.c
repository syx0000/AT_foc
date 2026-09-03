#include <stddef.h>
#include "motor_voltage_limit.h"

static int32_t motor_voltage_limit_abs(int32_t value)
{
  return (value < 0) ? -value : value;
}

bool motor_voltage_limit_apply(int32_t *direct_mv,
                               int32_t *quadrature_mv,
                               int32_t limit_mv)
{
  int32_t direct_abs;
  int32_t quadrature_abs;
  int32_t maximum;
  int32_t minimum;
  int32_t magnitude_upper;

  if ((direct_mv == NULL) || (quadrature_mv == NULL) || (limit_mv <= 0))
  {
    return false;
  }
  direct_abs = motor_voltage_limit_abs(*direct_mv);
  quadrature_abs = motor_voltage_limit_abs(*quadrature_mv);
  maximum = (direct_abs > quadrature_abs) ? direct_abs : quadrature_abs;
  minimum = (direct_abs > quadrature_abs) ? quadrature_abs : direct_abs;
  magnitude_upper = maximum + minimum / 2;
  if (magnitude_upper <= limit_mv)
  {
    return false;
  }
  *direct_mv = (int32_t)(((int64_t)*direct_mv * limit_mv) / magnitude_upper);
  *quadrature_mv = (int32_t)(((int64_t)*quadrature_mv * limit_mv) /
                             magnitude_upper);
  return true;
}
