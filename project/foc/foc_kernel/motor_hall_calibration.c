#include <stddef.h>
#include <string.h>
#include "motor_hall_calibration.h"

/**
 * @brief 锁存首个Hall标定错误并进入FAULT。
 * @param calibration 待更新上下文，不允许为空。
 * @param error 非OK错误码。
 * @return 原样返回error，方便调用方直接返回。
 */
static motor_hall_calibration_error_t motor_hall_calibration_fault_set(
  motor_hall_calibration_t *calibration,
  motor_hall_calibration_error_t error)
{
  calibration->error = error;
  calibration->state = MOTOR_HALL_CALIBRATION_FAULT;
  return error;
}

/**
 * @brief 判断两个Hall状态是否只变化一个输入位。
 * @param previous_state 前一组合状态。
 * @param current_state 当前组合状态。
 * @return 异或结果为1、2或4时返回true。
 */
static bool motor_hall_calibration_states_adjacent(uint8_t previous_state,
                                                   uint8_t current_state)
{
  uint8_t changed_bits = (uint8_t)(previous_state ^ current_state);
  return (changed_bits == 1U) || (changed_bits == 2U) ||
         (changed_bits == 4U);
}

/**
 * @brief 将一个U16周期角加入参考角附近的有符号展开累计器。
 * @param accumulator 待更新角度累计器，不允许为空。
 * @param angle_u16 新角度样本。
 * @return 无。
 */
static void motor_hall_calibration_angle_add(
  motor_hall_calibration_angle_accumulator_t *accumulator,
  uint16_t angle_u16)
{
  int16_t delta;

  if (accumulator->count == 0U)
  {
    accumulator->reference_angle_u16 = angle_u16;
    accumulator->minimum_delta = 0;
    accumulator->maximum_delta = 0;
    accumulator->delta_sum = 0;
    accumulator->count = 1U;
    return;
  }
  delta = (int16_t)(angle_u16 - accumulator->reference_angle_u16);
  accumulator->delta_sum += delta;
  if (delta < accumulator->minimum_delta)
    accumulator->minimum_delta = delta;
  if (delta > accumulator->maximum_delta)
    accumulator->maximum_delta = delta;
  accumulator->count++;
}

/**
 * @brief 计算周期角累计器的展开平均值。
 * @param accumulator 至少含一个样本的累计器，不允许为空。
 * @return U16一周制环形平均角。
 */
static uint16_t motor_hall_calibration_angle_mean(
  const motor_hall_calibration_angle_accumulator_t *accumulator)
{
  int64_t mean_delta = accumulator->delta_sum /
                       (int64_t)accumulator->count;
  return (uint16_t)(accumulator->reference_angle_u16 + mean_delta);
}

/**
 * @brief 计算样本相对其平均角的最大绝对展开偏差。
 * @param accumulator 至少含一个样本的累计器，不允许为空。
 * @return 最大偏差，单位U16角度计数。
 */
static uint16_t motor_hall_calibration_angle_deviation(
  const motor_hall_calibration_angle_accumulator_t *accumulator)
{
  int32_t mean_delta = (int32_t)(accumulator->delta_sum /
                                 (int64_t)accumulator->count);
  int32_t low = mean_delta - accumulator->minimum_delta;
  int32_t high = accumulator->maximum_delta - mean_delta;
  return (uint16_t)((low > high) ? low : high);
}

/**
 * @brief 校验正向跳转表并计算正向边界角与扇区宽度。
 * @param calibration 已完成正向采集的上下文。
 * @param entry_angle_u16 输出8元素边界角表。
 * @param sector_width_u16 输出8元素正向扇区宽度表。
 * @param maximum_deviation 输出所有状态最大样本偏差。
 * @return 全部状态形成单一六步闭环且角度满足配置时返回OK。
 */
static motor_hall_calibration_error_t motor_hall_calibration_forward_check(
  const motor_hall_calibration_t *calibration,
  uint16_t entry_angle_u16[8], uint16_t sector_width_u16[8],
  uint16_t *maximum_deviation)
{
  bool visited[8] = {false, false, false, false,
                     false, false, false, false};
  uint8_t predecessor_count[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
  uint8_t state;
  uint8_t next;
  uint8_t index;
  uint16_t deviation;
  uint16_t width;

  *maximum_deviation = 0U;
  for (state = 1U; state <= 6U; state++)
  {
    next = calibration->positive_next[state];
    if ((next < 1U) || (next > 6U) ||
        (!motor_hall_calibration_states_adjacent(state, next)) ||
        (calibration->forward_angle[state].count <
         calibration->config.minimum_samples_per_state))
    {
      return MOTOR_HALL_CALIBRATION_INCOMPLETE_SEQUENCE;
    }
    predecessor_count[next]++;
    entry_angle_u16[state] = motor_hall_calibration_angle_mean(
      &calibration->forward_angle[state]);
    deviation = motor_hall_calibration_angle_deviation(
      &calibration->forward_angle[state]);
    if (deviation > *maximum_deviation) *maximum_deviation = deviation;
  }
  for (state = 1U; state <= 6U; state++)
  {
    if (predecessor_count[state] != 1U)
      return MOTOR_HALL_CALIBRATION_INCOMPLETE_SEQUENCE;
  }
  state = 1U;
  for (index = 0U; index < 6U; index++)
  {
    if (visited[state]) return MOTOR_HALL_CALIBRATION_INCOMPLETE_SEQUENCE;
    visited[state] = true;
    state = calibration->positive_next[state];
  }
  if (state != 1U) return MOTOR_HALL_CALIBRATION_INCOMPLETE_SEQUENCE;
  if (*maximum_deviation > calibration->config.maximum_edge_deviation_u16)
    return MOTOR_HALL_CALIBRATION_EDGE_DEVIATION_EXCEEDED;

  for (state = 1U; state <= 6U; state++)
  {
    next = calibration->positive_next[state];
    width = (uint16_t)(entry_angle_u16[next] - entry_angle_u16[state]);
    sector_width_u16[state] = width;
    if ((width < calibration->config.minimum_sector_width_u16) ||
        (width > calibration->config.maximum_sector_width_u16))
    {
      return MOTOR_HALL_CALIBRATION_SECTOR_ANGLE_INVALID;
    }
  }
  return MOTOR_HALL_CALIBRATION_OK;
}

motor_hall_calibration_error_t motor_hall_calibration_init(
  motor_hall_calibration_t *calibration,
  const motor_hall_calibration_config_t *config)
{
  if ((calibration == NULL) || (config == NULL) ||
      (config->minimum_samples_per_state == 0U) ||
      (config->minimum_sector_width_u16 == 0U) ||
      (config->minimum_sector_width_u16 >=
       config->maximum_sector_width_u16) ||
      (config->maximum_sector_width_u16 >= 32768U) ||
      (config->maximum_edge_deviation_u16 >= 32768U) ||
      (config->maximum_forward_reverse_error_u16 >= 32768U))
  {
    return MOTOR_HALL_CALIBRATION_INVALID_ARGUMENT;
  }
  (void)memset(calibration, 0, sizeof(*calibration));
  calibration->config = *config;
  calibration->state = MOTOR_HALL_CALIBRATION_IDLE;
  calibration->error = MOTOR_HALL_CALIBRATION_OK;
  return MOTOR_HALL_CALIBRATION_OK;
}

motor_hall_calibration_error_t motor_hall_calibration_scan_begin(
  motor_hall_calibration_t *calibration, int8_t direction)
{
  if (calibration == NULL) return MOTOR_HALL_CALIBRATION_INVALID_ARGUMENT;
  if ((direction == 1) &&
      (calibration->state == MOTOR_HALL_CALIBRATION_IDLE))
  {
    calibration->state = MOTOR_HALL_CALIBRATION_FORWARD_SCAN;
  }
  else if ((direction == -1) &&
           (calibration->state == MOTOR_HALL_CALIBRATION_FORWARD_READY))
  {
    calibration->state = MOTOR_HALL_CALIBRATION_REVERSE_SCAN;
  }
  else
  {
    return MOTOR_HALL_CALIBRATION_SEQUENCE_ERROR;
  }
  calibration->timestamp_valid = false;
  return MOTOR_HALL_CALIBRATION_OK;
}

motor_hall_calibration_error_t motor_hall_calibration_edge_add(
  motor_hall_calibration_t *calibration, uint8_t previous_state,
  uint8_t current_state, uint16_t open_loop_angle_u16,
  uint32_t timestamp_cycles)
{
  uint32_t interval_cycles;
  uint8_t boundary_state;

  if (calibration == NULL) return MOTOR_HALL_CALIBRATION_INVALID_ARGUMENT;
  if ((calibration->state != MOTOR_HALL_CALIBRATION_FORWARD_SCAN) &&
      (calibration->state != MOTOR_HALL_CALIBRATION_REVERSE_SCAN))
    return MOTOR_HALL_CALIBRATION_SEQUENCE_ERROR;
  if ((previous_state < 1U) || (previous_state > 6U) ||
      (current_state < 1U) || (current_state > 6U))
    return motor_hall_calibration_fault_set(
      calibration, MOTOR_HALL_CALIBRATION_INVALID_STATE);
  if (previous_state == current_state)
    return motor_hall_calibration_fault_set(
      calibration, MOTOR_HALL_CALIBRATION_DUPLICATE_STATE);
  if (!motor_hall_calibration_states_adjacent(previous_state, current_state))
    return motor_hall_calibration_fault_set(
      calibration, MOTOR_HALL_CALIBRATION_NON_ADJACENT_STATE);

  if (calibration->timestamp_valid)
  {
    interval_cycles = timestamp_cycles - calibration->last_timestamp_cycles;
    if ((interval_cycles == 0U) ||
        ((calibration->config.maximum_edge_interval_cycles != 0U) &&
         (interval_cycles >
          calibration->config.maximum_edge_interval_cycles)))
    {
      return motor_hall_calibration_fault_set(
        calibration, MOTOR_HALL_CALIBRATION_TIMESTAMP_INVALID);
    }
  }
  calibration->last_timestamp_cycles = timestamp_cycles;
  calibration->timestamp_valid = true;

  if (calibration->state == MOTOR_HALL_CALIBRATION_FORWARD_SCAN)
  {
    if ((calibration->positive_next[previous_state] != 0U) &&
        (calibration->positive_next[previous_state] != current_state))
    {
      return motor_hall_calibration_fault_set(
        calibration, MOTOR_HALL_CALIBRATION_MULTIPLE_SUCCESSOR);
    }
    calibration->positive_next[previous_state] = current_state;
    boundary_state = current_state;
    motor_hall_calibration_angle_add(
      &calibration->forward_angle[boundary_state], open_loop_angle_u16);
  }
  else
  {
    if (calibration->positive_next[current_state] != previous_state)
    {
      return motor_hall_calibration_fault_set(
        calibration, MOTOR_HALL_CALIBRATION_REVERSE_MISMATCH);
    }
    boundary_state = previous_state;
    motor_hall_calibration_angle_add(
      &calibration->reverse_angle[boundary_state], open_loop_angle_u16);
  }
  return MOTOR_HALL_CALIBRATION_OK;
}

motor_hall_calibration_error_t motor_hall_calibration_scan_end(
  motor_hall_calibration_t *calibration)
{
  uint16_t entry_angle_u16[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
  uint16_t sector_width_u16[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
  uint16_t maximum_deviation;
  uint16_t reverse_deviation;
  uint16_t reverse_angle;
  uint16_t difference;
  uint8_t state;
  motor_hall_calibration_error_t error;

  if (calibration == NULL) return MOTOR_HALL_CALIBRATION_INVALID_ARGUMENT;
  if (calibration->state == MOTOR_HALL_CALIBRATION_FORWARD_SCAN)
  {
    error = motor_hall_calibration_forward_check(
      calibration, entry_angle_u16, sector_width_u16, &maximum_deviation);
    if (error != MOTOR_HALL_CALIBRATION_OK)
      return motor_hall_calibration_fault_set(calibration, error);
    calibration->state = MOTOR_HALL_CALIBRATION_FORWARD_READY;
    return MOTOR_HALL_CALIBRATION_OK;
  }
  if (calibration->state != MOTOR_HALL_CALIBRATION_REVERSE_SCAN)
    return MOTOR_HALL_CALIBRATION_SEQUENCE_ERROR;

  error = motor_hall_calibration_forward_check(
    calibration, entry_angle_u16, sector_width_u16, &maximum_deviation);
  if (error != MOTOR_HALL_CALIBRATION_OK)
    return motor_hall_calibration_fault_set(calibration, error);
  for (state = 1U; state <= 6U; state++)
  {
    if (calibration->reverse_angle[state].count <
        calibration->config.minimum_samples_per_state)
    {
      return motor_hall_calibration_fault_set(
        calibration, MOTOR_HALL_CALIBRATION_INCOMPLETE_SEQUENCE);
    }
    reverse_deviation = motor_hall_calibration_angle_deviation(
      &calibration->reverse_angle[state]);
    if (reverse_deviation >
        calibration->config.maximum_edge_deviation_u16)
    {
      return motor_hall_calibration_fault_set(
        calibration, MOTOR_HALL_CALIBRATION_EDGE_DEVIATION_EXCEEDED);
    }
    reverse_angle = motor_hall_calibration_angle_mean(
      &calibration->reverse_angle[state]);
    difference = (uint16_t)((int16_t)(reverse_angle -
                                      entry_angle_u16[state]));
    if ((int16_t)difference < 0)
      difference = (uint16_t)(-(int16_t)difference);
    if (difference > calibration->config.maximum_forward_reverse_error_u16)
    {
      return motor_hall_calibration_fault_set(
        calibration, MOTOR_HALL_CALIBRATION_REVERSE_MISMATCH);
    }
  }
  calibration->state = MOTOR_HALL_CALIBRATION_COMPLETE;
  return MOTOR_HALL_CALIBRATION_OK;
}

bool motor_hall_calibration_result_get(
  const motor_hall_calibration_t *calibration,
  motor_hall_calibration_result_t *result)
{
  uint16_t reverse_angle;
  uint16_t difference;
  uint16_t deviation;
  uint8_t state;
  motor_hall_calibration_error_t error;

  if (result == NULL) return false;
  (void)memset(result, 0, sizeof(*result));
  if ((calibration == NULL) ||
      (calibration->state != MOTOR_HALL_CALIBRATION_COMPLETE) ||
      (calibration->error != MOTOR_HALL_CALIBRATION_OK))
  {
    return false;
  }
  error = motor_hall_calibration_forward_check(
    calibration, result->entry_angle_u16, result->sector_width_u16,
    &result->maximum_edge_deviation_u16);
  if (error != MOTOR_HALL_CALIBRATION_OK) return false;
  (void)memcpy(result->positive_next, calibration->positive_next,
               sizeof(result->positive_next));
  for (state = 1U; state <= 6U; state++)
  {
    result->forward_samples[state] =
      calibration->forward_angle[state].count;
    result->reverse_samples[state] =
      calibration->reverse_angle[state].count;
    deviation = motor_hall_calibration_angle_deviation(
      &calibration->reverse_angle[state]);
    if (deviation > result->maximum_edge_deviation_u16)
      result->maximum_edge_deviation_u16 = deviation;
    reverse_angle = motor_hall_calibration_angle_mean(
      &calibration->reverse_angle[state]);
    difference = (uint16_t)((int16_t)(reverse_angle -
      result->entry_angle_u16[state]));
    if ((int16_t)difference < 0)
      difference = (uint16_t)(-(int16_t)difference);
    if (difference > result->maximum_forward_reverse_error_u16)
      result->maximum_forward_reverse_error_u16 = difference;
  }
  result->reverse_verified = true;
  result->valid = true;
  return true;
}
