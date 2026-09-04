#include <assert.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include "motor_direction.h"
#include "motor_hall_angle_math.h"

int main(void)
{
  static const uint8_t positive_next[8] =
    {0U, 5U, 3U, 1U, 6U, 4U, 2U, 0U};
  static const uint16_t entry_angle[8] =
    {0U, 1000U, 2000U, 3000U, 4000U, 5000U, 6000U, 0U};
  int32_t transformed;
  int32_t positive_step;
  int32_t negative_step;
  uint16_t angle;

  assert(motor_direction_sign_get(false) == 1);
  assert(motor_direction_sign_get(true) == -1);
  assert(motor_direction_transform_s32(1200, false, &transformed));
  assert(transformed == 1200);
  assert(motor_direction_transform_s32(1200, true, &transformed));
  assert(transformed == -1200);
  assert(motor_direction_transform_s32(-1200, true, &transformed));
  assert(transformed == 1200);
  assert(!motor_direction_transform_s32(INT32_MIN, true, &transformed));

  assert(motor_hall_angle_boundary_get(
    positive_next, entry_angle, 5U, 1, 100U, &angle));
  assert(angle == 5100U);
  assert(motor_hall_angle_boundary_get(
    positive_next, entry_angle, 1U, -1, 100U, &angle));
  assert(angle == 5100U);
  assert(motor_hall_angle_boundary_get(
    positive_next, entry_angle, 1U, 1, 65000U, &angle));
  assert(angle == 464U);

  assert(motor_hall_phase_step_get(10000U, 10000U, 1,
                                   &positive_step));
  assert(motor_hall_phase_step_get(10000U, 10000U, -1,
                                   &negative_step));
  assert(positive_step == -negative_step);
  assert(positive_step > 0);
  assert(!motor_hall_phase_step_get(0U, 10000U, 1, &positive_step));
  printf("motor_direction_angle_test: PASS\n");
  return 0;
}
