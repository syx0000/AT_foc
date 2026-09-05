/**
 * @file    motor_direction_angle_test.c
 * @brief   电机方向与角度变换 单元测试
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

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
  static const uint16_t equal_sector_entry_angle[8] =
    {0U, 0U, 43691U, 54613U, 21845U, 10923U, 32768U, 0U};
  int32_t transformed;
  int32_t positive_step;
  int32_t negative_step;
  int32_t dynamic_compensation;
  int32_t sector_step;
  uint32_t sector_intervals[8] =
    {0U, 480000U, 480000U, 480000U, 480000U, 480000U, 480000U, 0U};
  uint16_t sector_span;
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
  assert(motor_hall_dynamic_compensation_get(
    200000U, 1, 100U, 5461U, &dynamic_compensation));
  assert(dynamic_compensation == 1311);
  assert(motor_hall_dynamic_compensation_get(
    200000U, -1, 100U, 5461U, &dynamic_compensation));
  assert(dynamic_compensation == -1311);
  assert(motor_hall_dynamic_compensation_get(
    300000U, 1, 1000U, 5461U, &dynamic_compensation));
  assert(dynamic_compensation == 5461);
  assert(motor_hall_dynamic_compensation_get(
    200000U, 1, 0U, 5461U, &dynamic_compensation));
  assert(dynamic_compensation == 0);
  assert(motor_hall_sector_span_get(
    positive_next, entry_angle, 1U, &sector_span));
  assert(sector_span == (uint16_t)(entry_angle[5] - entry_angle[1]));
  assert(motor_hall_sector_phase_step_get(
    10923U, 480000U, 288000000U, 10000U, 1, &sector_step));
  assert(sector_step == 42950984);
  assert(motor_hall_sector_phase_step_get(
    10923U, 480000U, 288000000U, 10000U, -1, &sector_step));
  assert(sector_step == -42950984);
  assert(motor_hall_sector_table_phase_step_get(
    positive_next, equal_sector_entry_angle, sector_intervals, 1U, 288000000U,
    10000U, 1, 480000U, 10U, &sector_step));
  assert(sector_step > 0);
  sector_intervals[3] = 0U;
  assert(!motor_hall_sector_table_phase_step_get(
    positive_next, equal_sector_entry_angle, sector_intervals, 1U, 288000000U,
    10000U, 1, 480000U, 10U, &sector_step));
  sector_intervals[3] = 900000U;
  assert(!motor_hall_sector_table_phase_step_get(
    positive_next, equal_sector_entry_angle, sector_intervals, 1U, 288000000U,
    10000U, 1, 480000U, 10U, &sector_step));
  printf("motor_direction_angle_test: PASS\n");
  return 0;
}
