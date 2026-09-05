/**
 * @file    motor_hall_angle_estimator.c
 * @brief   Hall转子电角度估算器 功能实现
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#include <stddef.h>
#include <string.h>
#include "at32f45x.h"
#include "motor_board_config.h"
#include "motor_control_config.h"
#include "motor_hall_angle_estimator.h"
#include "motor_hall_angle_math.h"
#include "motor_hall_decoder.h"
#include "motor_parameter.h"
#include "motor_timebase.h"

static volatile motor_hall_angle_estimator_t motor_hall_angle_estimator;
static uint32_t motor_hall_phase_accumulator;
static int32_t motor_hall_phase_step;
static int32_t motor_hall_dynamic_compensation_u16;
static uint32_t motor_hall_last_edge_cycles;
static uint32_t motor_hall_sector_interval_cycles[2][8];
static uint8_t motor_hall_sector_stable_count[2][8];
static int32_t motor_hall_sector_phase_step[2][8];
static int32_t motor_hall_edge_correction_q16;
static uint8_t motor_hall_edge_correction_cycles;

#define MOTOR_HALL_SECTOR_TOTAL_TOLERANCE_PERCENT       10U
#define MOTOR_HALL_SECTOR_STABILITY_TOLERANCE_PERCENT   15U
#define MOTOR_HALL_SECTOR_STABILITY_REQUIRED_COUNT       3U
#define MOTOR_HALL_EDGE_BLEND_CYCLES                     32U
#define MOTOR_HALL_EDGE_MAXIMUM_CORRECTION_U16         2731U
#define MOTOR_HALL_SECTOR_STEP_MAXIMUM_CHANGE_PERCENT     5U

static uint8_t motor_hall_direction_index_get(int8_t direction)
{
  return (direction > 0) ? 0U : 1U;
}

static void motor_hall_sector_interval_update(uint8_t direction_index,
                                              uint8_t state,
                                              uint32_t measured_cycles,
                                              uint32_t expected_cycles)
{
  uint32_t previous;

  if ((direction_index > 1U) || (state < 1U) || (state > 6U) ||
      (measured_cycles == 0U) || (expected_cycles == 0U)) return;
  /* 只学习与当前六边沿平均测速相符的扇区周期。 */
  if (((uint64_t)measured_cycles * 2U < expected_cycles) ||
      ((uint64_t)measured_cycles > (uint64_t)expected_cycles * 2U))
  {
    motor_hall_sector_stable_count[direction_index][state] = 0U;
    return;
  }
  previous = motor_hall_sector_interval_cycles[direction_index][state];
  /* 启动阶段遗留的低速周期偏离当前速度时，直接重新捕获。 */
  if ((previous == 0U) ||
      ((uint64_t)previous * 2U < expected_cycles) ||
      ((uint64_t)previous > (uint64_t)expected_cycles * 2U))
  {
    motor_hall_sector_interval_cycles[direction_index][state] = measured_cycles;
    motor_hall_sector_stable_count[direction_index][state] = 0U;
    return;
  }
  if (((uint64_t)measured_cycles * 100U >=
       (uint64_t)previous *
         (100U - MOTOR_HALL_SECTOR_STABILITY_TOLERANCE_PERCENT)) &&
      ((uint64_t)measured_cycles * 100U <=
       (uint64_t)previous *
         (100U + MOTOR_HALL_SECTOR_STABILITY_TOLERANCE_PERCENT)))
  {
    if (motor_hall_sector_stable_count[direction_index][state] < 255U)
      motor_hall_sector_stable_count[direction_index][state]++;
  }
  else
  {
    motor_hall_sector_stable_count[direction_index][state] = 0U;
  }
  /* 合法周期用1/8 IIR跟踪转速变化并抑制边沿周期抖动。 */
  motor_hall_sector_interval_cycles[direction_index][state] =
    (uint32_t)(((uint64_t)previous * 7U + measured_cycles + 4U) / 8U);
}

static int32_t motor_hall_sector_phase_step_limit(uint8_t direction_index,
                                                   uint8_t state,
                                                   int32_t candidate_step)
{
  int32_t previous_step;
  uint32_t previous_magnitude;
  uint32_t candidate_magnitude;
  uint32_t minimum_magnitude;
  uint32_t maximum_magnitude;

  if ((direction_index > 1U) || (state < 1U) || (state > 6U) ||
      (candidate_step == 0)) return candidate_step;
  previous_step = motor_hall_sector_phase_step[direction_index][state];
  if (((candidate_step > 0) != (previous_step > 0)) || (previous_step == 0))
  {
    motor_hall_sector_phase_step[direction_index][state] = candidate_step;
    return candidate_step;
  }
  previous_magnitude = (previous_step > 0) ?
    (uint32_t)previous_step : (uint32_t)(-previous_step);
  candidate_magnitude = (candidate_step > 0) ?
    (uint32_t)candidate_step : (uint32_t)(-candidate_step);
  minimum_magnitude = (uint32_t)(
    ((uint64_t)previous_magnitude *
     (100U - MOTOR_HALL_SECTOR_STEP_MAXIMUM_CHANGE_PERCENT)) / 100U);
  maximum_magnitude = (uint32_t)(
    ((uint64_t)previous_magnitude *
     (100U + MOTOR_HALL_SECTOR_STEP_MAXIMUM_CHANGE_PERCENT)) / 100U);
  if (candidate_magnitude < minimum_magnitude)
    candidate_magnitude = minimum_magnitude;
  else if (candidate_magnitude > maximum_magnitude)
    candidate_magnitude = maximum_magnitude;
  candidate_step = (candidate_step > 0) ?
    (int32_t)candidate_magnitude : -(int32_t)candidate_magnitude;
  motor_hall_sector_phase_step[direction_index][state] = candidate_step;
  return candidate_step;
}

static bool motor_hall_sector_table_stable(uint8_t direction_index)
{
  uint8_t state;

  if (direction_index > 1U) return false;
  for (state = 1U; state <= 6U; state++)
  {
    if (motor_hall_sector_stable_count[direction_index][state] <
        MOTOR_HALL_SECTOR_STABILITY_REQUIRED_COUNT)
      return false;
  }
  return true;
}

void motor_hall_angle_estimator_init(void)
{
  motor_hall_angle_estimator.electrical_angle_u16 = 0U;
  motor_hall_angle_estimator.dynamic_compensation_u16 = 0;
  motor_hall_angle_estimator.phase_step = 0;
  motor_hall_angle_estimator.edge_error_u16 = 0;
  motor_hall_angle_estimator.edge_correction_remaining_u16 = 0;
  motor_hall_angle_estimator.sector_interval_cycles = 0U;
  motor_hall_angle_estimator.electrical_frequency_millihz = 0U;
  motor_hall_angle_estimator.hall_state = 0U;
  motor_hall_angle_estimator.direction = 0;
  motor_hall_angle_estimator.valid = false;
  motor_hall_angle_estimator.sector_interpolation_active = false;
  motor_hall_phase_accumulator = 0U;
  motor_hall_phase_step = 0U;
  motor_hall_dynamic_compensation_u16 = 0;
  (void)memset(motor_hall_sector_interval_cycles, 0,
               sizeof(motor_hall_sector_interval_cycles));
  (void)memset(motor_hall_sector_stable_count, 0,
               sizeof(motor_hall_sector_stable_count));
  (void)memset(motor_hall_sector_phase_step, 0,
               sizeof(motor_hall_sector_phase_step));
  motor_hall_edge_correction_q16 = 0;
  motor_hall_edge_correction_cycles = 0U;
  motor_hall_last_edge_cycles = motor_timebase_cycles_get();
}

void motor_hall_angle_estimator_edge_process(void)
{
  motor_hall_sample_t hall_sample;
  motor_parameter_t parameter;
  uint16_t boundary_angle_u16;
  uint32_t sector_interval_cycles;
  uint32_t expected_sector_interval_cycles;
  uint8_t direction_index;
  int32_t base_phase_step;
  int32_t sector_phase_step;
  int32_t edge_error_u16;
  int64_t correction_total_q16;
  uint16_t predicted_angle_u16;
  bool estimator_was_valid;
  int8_t previous_direction;

  /* Both snapshots use sequence counters.  An edge interrupt can collide
   * with a lower-priority update and make a read return false temporarily.
   * Keep the last valid estimate and retry at the next edge; a snapshot
   * collision is not evidence that the physical Hall signal disappeared. */
  if (!motor_hall_decoder_sample_read(&hall_sample))
    return;
  if (!motor_parameter_active_read(&parameter))
    return;

  if ((!hall_sample.valid) ||
      (hall_sample.direction == 0) ||
      (hall_sample.electrical_frequency_millihz == 0U))
  {
    motor_hall_angle_estimator.valid = false;
    motor_hall_angle_estimator.direction = 0;
    motor_hall_angle_estimator.dynamic_compensation_u16 = 0;
    motor_hall_angle_estimator.phase_step = 0;
    motor_hall_angle_estimator.edge_error_u16 = 0;
    motor_hall_angle_estimator.edge_correction_remaining_u16 = 0;
    motor_hall_angle_estimator.sector_interval_cycles = 0U;
    motor_hall_angle_estimator.sector_interpolation_active = false;
    motor_hall_dynamic_compensation_u16 = 0;
    motor_hall_edge_correction_q16 = 0;
    motor_hall_edge_correction_cycles = 0U;
    return;
  }

  if (hall_sample.timestamp_cycles == motor_hall_last_edge_cycles) return;
  estimator_was_valid = motor_hall_angle_estimator.valid;
  previous_direction = motor_hall_angle_estimator.direction;
  direction_index = motor_hall_direction_index_get(hall_sample.direction);
  expected_sector_interval_cycles = (uint32_t)(
    ((uint64_t)system_core_clock * 1000U) /
    ((uint64_t)hall_sample.electrical_frequency_millihz * 6U));
  motor_hall_sector_interval_update(direction_index,
                                    hall_sample.previous_state,
                                    hall_sample.interval_cycles,
                                    expected_sector_interval_cycles);
  sector_interval_cycles =
    motor_hall_sector_interval_cycles[direction_index][hall_sample.state];

  if ((!motor_hall_angle_boundary_get(
        parameter.hall_positive_next, parameter.hall_entry_angle_u16,
        hall_sample.state, hall_sample.direction,
        parameter.hall_rotor_offset_u16, &boundary_angle_u16)) ||
      (!motor_hall_phase_step_get(
        hall_sample.electrical_frequency_millihz,
        MOTOR_PWM_FREQUENCY_HZ, hall_sample.direction,
        &base_phase_step)) ||
      (!motor_hall_dynamic_compensation_get(
        hall_sample.electrical_frequency_millihz, hall_sample.direction,
        parameter.hall_delay_compensation_us,
        MOTOR_HALL_DYNAMIC_COMPENSATION_MAXIMUM_U16,
        &motor_hall_dynamic_compensation_u16)))
  {
    motor_hall_angle_estimator.valid = false;
    motor_hall_angle_estimator.direction = 0;
    motor_hall_angle_estimator.dynamic_compensation_u16 = 0;
    motor_hall_angle_estimator.phase_step = 0;
    motor_hall_angle_estimator.edge_error_u16 = 0;
    motor_hall_angle_estimator.edge_correction_remaining_u16 = 0;
    motor_hall_angle_estimator.sector_interval_cycles = 0U;
    motor_hall_angle_estimator.sector_interpolation_active = false;
    motor_hall_dynamic_compensation_u16 = 0;
    motor_hall_edge_correction_q16 = 0;
    motor_hall_edge_correction_cycles = 0U;
    return;
  }
  motor_hall_phase_step = base_phase_step;
  motor_hall_angle_estimator.sector_interpolation_active = false;
  if (motor_hall_sector_table_stable(direction_index) &&
      motor_hall_sector_table_phase_step_get(
        parameter.hall_positive_next, parameter.hall_entry_angle_u16,
        motor_hall_sector_interval_cycles[direction_index],
        hall_sample.state, system_core_clock, MOTOR_PWM_FREQUENCY_HZ,
        hall_sample.direction, expected_sector_interval_cycles,
        MOTOR_HALL_SECTOR_TOTAL_TOLERANCE_PERCENT, &sector_phase_step))
  {
    motor_hall_phase_step = motor_hall_sector_phase_step_limit(
      direction_index, hall_sample.state, sector_phase_step);
    motor_hall_angle_estimator.sector_interpolation_active = true;
  }
  motor_hall_angle_estimator.phase_step = motor_hall_phase_step;
  motor_hall_angle_estimator.sector_interval_cycles = sector_interval_cycles;
  predicted_angle_u16 = (uint16_t)(motor_hall_phase_accumulator >> 16);
  if ((!estimator_was_valid) || (previous_direction != hall_sample.direction))
  {
    motor_hall_phase_accumulator = (uint32_t)boundary_angle_u16 << 16;
    motor_hall_edge_correction_q16 = 0;
    motor_hall_edge_correction_cycles = 0U;
    predicted_angle_u16 = boundary_angle_u16;
  }
  else
  {
    edge_error_u16 = (int16_t)(boundary_angle_u16 - predicted_angle_u16);
    correction_total_q16 = (int64_t)motor_hall_edge_correction_q16 +
                           (int64_t)edge_error_u16 * 65536L;
    if (correction_total_q16 >
        (int64_t)MOTOR_HALL_EDGE_MAXIMUM_CORRECTION_U16 * 65536L)
      correction_total_q16 =
        (int64_t)MOTOR_HALL_EDGE_MAXIMUM_CORRECTION_U16 * 65536L;
    else if (correction_total_q16 <
             -(int64_t)MOTOR_HALL_EDGE_MAXIMUM_CORRECTION_U16 * 65536L)
      correction_total_q16 =
        -(int64_t)MOTOR_HALL_EDGE_MAXIMUM_CORRECTION_U16 * 65536L;
    motor_hall_edge_correction_q16 = (int32_t)correction_total_q16;
    motor_hall_edge_correction_cycles = MOTOR_HALL_EDGE_BLEND_CYCLES;
    motor_hall_angle_estimator.edge_error_u16 = edge_error_u16;
    motor_hall_angle_estimator.edge_correction_remaining_u16 =
      motor_hall_edge_correction_q16 / 65536L;
  }
  motor_hall_last_edge_cycles = hall_sample.timestamp_cycles;
  motor_hall_angle_estimator.electrical_angle_u16 = (uint16_t)(
    (int32_t)predicted_angle_u16 + motor_hall_dynamic_compensation_u16);
  motor_hall_angle_estimator.dynamic_compensation_u16 =
    motor_hall_dynamic_compensation_u16;
  motor_hall_angle_estimator.electrical_frequency_millihz =
    hall_sample.electrical_frequency_millihz;
  motor_hall_angle_estimator.hall_state = hall_sample.state;
  motor_hall_angle_estimator.direction = hall_sample.direction;
  motor_hall_angle_estimator.valid = true;
}

void motor_hall_angle_estimator_fast_process(void)
{
  int32_t correction_step_q16;
  uint32_t timeout_cycles =
    (system_core_clock / 1000U) * MOTOR_HALL_SIGNAL_TIMEOUT_MS;

  if (!motor_hall_angle_estimator.valid)
  {
    return;
  }

  if (motor_timebase_cycles_elapsed(motor_hall_last_edge_cycles) > timeout_cycles)
  {
    motor_hall_angle_estimator.valid = false;
    motor_hall_angle_estimator.direction = 0;
    motor_hall_angle_estimator.dynamic_compensation_u16 = 0;
    motor_hall_angle_estimator.phase_step = 0;
    motor_hall_angle_estimator.edge_error_u16 = 0;
    motor_hall_angle_estimator.edge_correction_remaining_u16 = 0;
    motor_hall_angle_estimator.sector_interval_cycles = 0U;
    motor_hall_angle_estimator.sector_interpolation_active = false;
    motor_hall_dynamic_compensation_u16 = 0;
    motor_hall_edge_correction_q16 = 0;
    motor_hall_edge_correction_cycles = 0U;
    return;
  }

  motor_hall_phase_accumulator += (uint32_t)motor_hall_phase_step;
  if (motor_hall_edge_correction_cycles != 0U)
  {
    correction_step_q16 = motor_hall_edge_correction_q16 /
                          motor_hall_edge_correction_cycles;
    motor_hall_phase_accumulator += (uint32_t)correction_step_q16;
    motor_hall_edge_correction_q16 -= correction_step_q16;
    motor_hall_edge_correction_cycles--;
    motor_hall_angle_estimator.edge_correction_remaining_u16 =
      motor_hall_edge_correction_q16 / 65536L;
  }
  motor_hall_angle_estimator.electrical_angle_u16 =
    (uint16_t)((int32_t)(motor_hall_phase_accumulator >> 16) +
               motor_hall_dynamic_compensation_u16);
}

bool motor_hall_angle_estimator_read(motor_hall_angle_estimator_t *estimator)
{
  uint32_t primask;

  if (estimator == NULL)
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *estimator = motor_hall_angle_estimator;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return true;
}
