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
/* 最近一次边沿对齐的边界角，用于超前冻结判据。 */
static uint16_t motor_hall_edge_boundary_u16;
/* 各方向、各状态的扇区持续周期，仅用于自适应无边沿超时。 */
static uint32_t motor_hall_sector_interval_cycles[2][8];
static int32_t motor_hall_correction_remaining;
static uint16_t motor_hall_forward_table[8],motor_hall_reverse_table[8];
static uint8_t motor_hall_cached_next[8];
static uint16_t motor_hall_cached_entry[8],motor_hall_cached_offset;
static bool motor_hall_tables_valid;

/**
 * @brief 将估算器置为无效并清除运动相关输出。
 * @param 无。
 * @return 无。
 */
static void motor_hall_angle_estimator_invalidate(void)
{
  motor_hall_angle_estimator.valid = false;
  motor_hall_angle_estimator.direction = 0;
  motor_hall_angle_estimator.dynamic_compensation_u16 = 0;
  motor_hall_angle_estimator.phase_step = 0;
  motor_hall_angle_estimator.edge_error_u16 = 0;
  motor_hall_angle_estimator.sector_interval_cycles = 0U;
  motor_hall_dynamic_compensation_u16 = 0;
  motor_hall_correction_remaining = 0;
}

/**
 * @brief 用实测扇区周期更新该扇区的持续时间估计。
 * @param direction_index 0为正向，1为反向。
 * @param state 刚结束的Hall状态1..6。
 * @param measured_cycles 该扇区实测持续CPU周期。
 * @param expected_cycles 按当前平均电频率推算的单扇区周期。
 * @return 无。
 * @details 只学习与平均测速相差不超过2倍的周期；偏离过大的旧值直接替换，
 *          其余用1/8 IIR跟踪。
 */
static void motor_hall_sector_interval_update(uint8_t direction_index,
                                              uint8_t state,
                                              uint32_t measured_cycles,
                                              uint32_t expected_cycles)
{
  uint32_t previous;

  if ((direction_index > 1U) || (state < 1U) || (state > 6U) ||
      (measured_cycles == 0U) || (expected_cycles == 0U)) return;
  if (((uint64_t)measured_cycles * 2U < expected_cycles) ||
      ((uint64_t)measured_cycles > (uint64_t)expected_cycles * 2U)) return;
  previous = motor_hall_sector_interval_cycles[direction_index][state];
  if ((previous == 0U) ||
      ((uint64_t)previous * 2U < expected_cycles) ||
      ((uint64_t)previous > (uint64_t)expected_cycles * 2U))
  {
    motor_hall_sector_interval_cycles[direction_index][state] = measured_cycles;
    return;
  }
  motor_hall_sector_interval_cycles[direction_index][state] =
    (uint32_t)(((uint64_t)previous * 7U + measured_cycles + 4U) / 8U);
}

void motor_hall_angle_estimator_init(void)
{
  motor_hall_tables_valid = false;
  motor_hall_angle_estimator_invalidate();
  motor_hall_angle_estimator.electrical_angle_u16 = 0U;
  motor_hall_angle_estimator.electrical_frequency_millihz = 0U;
  motor_hall_angle_estimator.hall_state = 0U;
  motor_hall_phase_accumulator = 0U;
  motor_hall_phase_step = 0;
  motor_hall_edge_boundary_u16 = 0U;
  (void)memset(motor_hall_sector_interval_cycles, 0,
               sizeof(motor_hall_sector_interval_cycles));
  motor_hall_last_edge_cycles = motor_timebase_cycles_get();
}

void motor_hall_angle_estimator_motion_reset(void)
{
  motor_hall_angle_estimator_invalidate();
  motor_hall_angle_estimator.electrical_frequency_millihz = 0U;
  motor_hall_phase_step = 0;
  motor_hall_last_edge_cycles = motor_timebase_cycles_get();
}

void motor_hall_angle_estimator_edge_process(void)
{
  motor_hall_sample_t hall_sample;
  motor_parameter_t parameter;
  uint16_t boundary_angle_u16;
  uint16_t predicted_angle_u16;
  uint32_t expected_sector_interval_cycles;
  uint8_t direction_index;
  int32_t base_phase_step;
  bool tables_changed;
  uint32_t publish_primask;

  /* Both snapshots use sequence counters.  An edge interrupt can collide
   * with a lower-priority update and make a read return false temporarily.
   * Keep the last valid estimate and retry at the next edge; a snapshot
   * collision is not evidence that the physical Hall signal disappeared. */
  if (!motor_hall_decoder_sample_read(&hall_sample))
    return;
  if (!motor_parameter_active_read(&parameter))
    return;

  if ((!hall_sample.valid) || hall_sample.health_fault ||
      (hall_sample.direction == 0) ||
      (hall_sample.electrical_frequency_millihz == 0U))
  {
    motor_hall_angle_estimator_invalidate();
    return;
  }

  if (hall_sample.timestamp_cycles == motor_hall_last_edge_cycles) return;
  tables_changed = !motor_hall_tables_valid ||
    memcmp(motor_hall_cached_next,parameter.hall_positive_next,sizeof(motor_hall_cached_next)) ||
    memcmp(motor_hall_cached_entry,parameter.hall_entry_angle_u16,sizeof(motor_hall_cached_entry)) ||
    motor_hall_cached_offset!=parameter.hall_rotor_offset_u16;
  if (tables_changed)
  {
    motor_hall_tables_valid = motor_hall_direction_tables_build(parameter.hall_positive_next,
      parameter.hall_entry_angle_u16,parameter.hall_rotor_offset_u16,
      motor_hall_forward_table,motor_hall_reverse_table);
    if (!motor_hall_tables_valid) { motor_hall_angle_estimator_invalidate(); return; }
    memcpy(motor_hall_cached_next,parameter.hall_positive_next,sizeof(motor_hall_cached_next));
    memcpy(motor_hall_cached_entry,parameter.hall_entry_angle_u16,sizeof(motor_hall_cached_entry));
    motor_hall_cached_offset=parameter.hall_rotor_offset_u16;
  }
  direction_index = (hall_sample.direction > 0) ? 0U : 1U;
  expected_sector_interval_cycles = (uint32_t)(
    ((uint64_t)system_core_clock * 1000U) /
    ((uint64_t)hall_sample.electrical_frequency_millihz * 6U));
  motor_hall_sector_interval_update(direction_index,
                                    hall_sample.previous_state,
                                    hall_sample.interval_cycles,
                                    expected_sector_interval_cycles);

  boundary_angle_u16 = hall_sample.direction>0 ?
    motor_hall_forward_table[hall_sample.state] : motor_hall_reverse_table[hall_sample.state];
  if ((!motor_hall_phase_step_get(
        hall_sample.electrical_frequency_millihz,
        MOTOR_PWM_FREQUENCY_HZ, hall_sample.direction,
        &base_phase_step)) ||
      (!motor_hall_dynamic_compensation_get(
        hall_sample.electrical_frequency_millihz, hall_sample.direction,
        parameter.hall_delay_compensation_us,
        MOTOR_HALL_DYNAMIC_COMPENSATION_MAXIMUM_U16,
        &motor_hall_dynamic_compensation_u16)))
  {
    motor_hall_angle_estimator_invalidate();
    return;
  }

  /* Publish phase/correction/step together against the higher-priority ADC ISR. */
  publish_primask=__get_PRIMASK();
  __disable_irq();
  predicted_angle_u16 = (uint16_t)(motor_hall_phase_accumulator >> 16);
  motor_hall_angle_estimator.edge_error_u16 =
    (int16_t)(boundary_angle_u16 - predicted_angle_u16);
  if (!tables_changed && motor_hall_angle_estimator.valid &&
      motor_hall_angle_estimator.direction==hall_sample.direction &&
      hall_sample.electrical_frequency_millihz>=10000U &&
      motor_hall_angle_estimator.edge_error_u16>=-2731 &&
      motor_hall_angle_estimator.edge_error_u16<=2731)
    motor_hall_correction_remaining=motor_hall_angle_estimator.edge_error_u16;
  else
  {
    motor_hall_phase_accumulator = (uint32_t)boundary_angle_u16 << 16;
    motor_hall_correction_remaining=0;
  }
  motor_hall_phase_step = base_phase_step;
  motor_hall_edge_boundary_u16 = boundary_angle_u16;

  motor_hall_angle_estimator.sector_interval_cycles =
    motor_hall_sector_interval_cycles[direction_index][hall_sample.state];
  motor_hall_angle_estimator.phase_step = motor_hall_phase_step;
  motor_hall_last_edge_cycles = hall_sample.timestamp_cycles;
  motor_hall_angle_estimator.electrical_angle_u16 = (uint16_t)(
    (int32_t)(motor_hall_phase_accumulator >> 16) + motor_hall_dynamic_compensation_u16);
  motor_hall_angle_estimator.dynamic_compensation_u16 =
    motor_hall_dynamic_compensation_u16;
  motor_hall_angle_estimator.electrical_frequency_millihz = (uint32_t)(
    ((uint64_t)((motor_hall_phase_step < 0) ?
      -(int64_t)motor_hall_phase_step : motor_hall_phase_step) *
      MOTOR_PWM_FREQUENCY_HZ * 1000U) >> 32);
  motor_hall_angle_estimator.hall_state = hall_sample.state;
  motor_hall_angle_estimator.direction = hall_sample.direction;
  motor_hall_angle_estimator.valid = true;
  __set_PRIMASK(publish_primask);
}

void motor_hall_angle_estimator_fast_process(void)
{
  uint32_t timeout_cycles =
    (system_core_clock / 1000U) * MOTOR_HALL_SIGNAL_TIMEOUT_MS;
  uint32_t adaptive_timeout_cycles;
  int16_t lead_u16;
  int32_t correction;
  int32_t correction_limit;

  if (!motor_hall_angle_estimator.valid)
  {
    return;
  }

  if (motor_hall_angle_estimator.sector_interval_cycles != 0U)
  {
    adaptive_timeout_cycles =
      motor_hall_angle_estimator.sector_interval_cycles *
      MOTOR_HALL_TIMEOUT_SECTORS;
    if (adaptive_timeout_cycles <
        ((system_core_clock / 1000U) * MOTOR_HALL_MINIMUM_TIMEOUT_MS))
      adaptive_timeout_cycles =
        (system_core_clock / 1000U) * MOTOR_HALL_MINIMUM_TIMEOUT_MS;
    if (adaptive_timeout_cycles < timeout_cycles)
      timeout_cycles = adaptive_timeout_cycles;
  }

  if (motor_timebase_cycles_elapsed(motor_hall_last_edge_cycles) > timeout_cycles)
  {
    motor_hall_angle_estimator_invalidate();
    return;
  }

  /* 估计角相对边界角超前达到上限后停止累加，等待下一边沿
   * （对应mclib max_angle_err）。 */
  correction=motor_hall_correction_step(motor_hall_correction_remaining);
  /* A negative correction must not reverse the moving angle trajectory. */
  correction_limit=(int32_t)(((motor_hall_phase_step<0 ?
    -(int64_t)motor_hall_phase_step : motor_hall_phase_step)) / 131072LL);
  if (correction_limit<1) correction_limit=1;
  if (correction>correction_limit) correction=correction_limit;
  if (correction<-correction_limit) correction=-correction_limit;
  motor_hall_correction_remaining-=correction;
  motor_hall_phase_accumulator+=(uint32_t)((int64_t)correction*65536LL);
  lead_u16 = (int16_t)((uint16_t)(motor_hall_phase_accumulator >> 16) -
                       motor_hall_edge_boundary_u16);
  if (((motor_hall_phase_step > 0) &&
       (lead_u16 < (int16_t)MOTOR_HALL_MAX_ANGLE_LEAD_U16)) ||
      ((motor_hall_phase_step < 0) &&
       (lead_u16 > -(int16_t)MOTOR_HALL_MAX_ANGLE_LEAD_U16)))
    motor_hall_phase_accumulator += (uint32_t)motor_hall_phase_step;
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
