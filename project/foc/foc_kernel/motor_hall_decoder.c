#include <stddef.h>
#include <string.h>
#include "motor_hall_decoder.h"

static volatile motor_hall_sample_t motor_hall_decoder_sample;
static volatile uint32_t motor_hall_decoder_sequence;
static uint8_t motor_hall_decoder_positive_next[MOTOR_HALL_DECODER_STATE_COUNT];
static uint32_t motor_hall_decoder_interval_cycles[MOTOR_HALL_DECODER_SPEED_WINDOW_SIZE];
static uint64_t motor_hall_decoder_interval_sum_cycles;
static uint32_t motor_hall_decoder_last_edge_cycles;
static uint32_t motor_hall_decoder_core_clock_hz;
static uint8_t motor_hall_decoder_interval_index;
static uint8_t motor_hall_decoder_interval_count;
static int8_t motor_hall_decoder_last_direction;
static bool motor_hall_decoder_initialized;

/**
 * @brief 校验正向Hall表是否构成状态1..6的唯一闭环。
 * @param positive_next 8元素状态后继表，不允许为空。
 * @return 六个有效状态恰好组成一个闭环时返回true，否则返回false。
 */
static bool motor_hall_decoder_sequence_validate(const uint8_t *positive_next)
{
  bool visited[8] = {false, false, false, false,
                     false, false, false, false};
  uint8_t predecessor_count[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
  uint8_t state;
  uint8_t index;
  uint8_t changed_bits;

  if ((positive_next == NULL) || (positive_next[0] != 0U) ||
      (positive_next[7] != 0U))
  {
    return false;
  }
  for (state = 1U; state <= 6U; state++)
  {
    if ((positive_next[state] < 1U) || (positive_next[state] > 6U) ||
        (positive_next[state] == state))
    {
      return false;
    }
    changed_bits = (uint8_t)(state ^ positive_next[state]);
    if ((changed_bits != 1U) && (changed_bits != 2U) &&
        (changed_bits != 4U))
    {
      return false;
    }
    predecessor_count[positive_next[state]]++;
  }
  for (state = 1U; state <= 6U; state++)
  {
    if (predecessor_count[state] != 1U) return false;
  }
  state = 1U;
  for (index = 0U; index < 6U; index++)
  {
    if (visited[state]) return false;
    visited[state] = true;
    state = positive_next[state];
  }
  return (state == 1U);
}

/**
 * @brief 清空边沿测速滑动窗口。
 * @param timestamp_cycles 下一段合法边沿间隔的起点。
 * @return 无。
 */
static void motor_hall_decoder_speed_reset(uint32_t timestamp_cycles)
{
  (void)memset(motor_hall_decoder_interval_cycles, 0,
               sizeof(motor_hall_decoder_interval_cycles));
  motor_hall_decoder_interval_sum_cycles = 0U;
  motor_hall_decoder_last_edge_cycles = timestamp_cycles;
  motor_hall_decoder_interval_index = 0U;
  motor_hall_decoder_interval_count = 0U;
}

/**
 * @brief 将一个同方向合法边沿间隔加入6段滑动测速窗口。
 * @param interval_cycles 相邻合法边沿的无符号周期差，必须大于0。
 * @return 无。
 */
static void motor_hall_decoder_speed_push(uint32_t interval_cycles)
{
  if (interval_cycles == 0U) return;
  if (motor_hall_decoder_interval_count >=
      MOTOR_HALL_DECODER_SPEED_WINDOW_SIZE)
  {
    motor_hall_decoder_interval_sum_cycles -=
      motor_hall_decoder_interval_cycles[motor_hall_decoder_interval_index];
  }
  else
  {
    motor_hall_decoder_interval_count++;
  }
  motor_hall_decoder_interval_cycles[motor_hall_decoder_interval_index] =
    interval_cycles;
  motor_hall_decoder_interval_sum_cycles += interval_cycles;
  motor_hall_decoder_interval_index++;
  if (motor_hall_decoder_interval_index >=
      MOTOR_HALL_DECODER_SPEED_WINDOW_SIZE)
  {
    motor_hall_decoder_interval_index = 0U;
  }
  motor_hall_decoder_sample.electrical_frequency_millihz =
    (uint32_t)(((uint64_t)motor_hall_decoder_core_clock_hz * 1000U *
                motor_hall_decoder_interval_count) /
               (motor_hall_decoder_interval_sum_cycles * 6U));
  motor_hall_decoder_sample.frequency_update_count++;
}

bool motor_hall_decoder_init(const uint8_t *positive_next,
                             uint32_t core_clock_hz,
                             uint8_t initial_state,
                             uint32_t initial_timestamp_cycles)
{
  motor_hall_decoder_initialized = false;
  motor_hall_decoder_sequence = 0U;
  (void)memset((void *)&motor_hall_decoder_sample, 0,
               sizeof(motor_hall_decoder_sample));
  if ((!motor_hall_decoder_sequence_validate(positive_next)) ||
      (core_clock_hz == 0U) || (initial_state > 7U))
  {
    return false;
  }
  (void)memcpy(motor_hall_decoder_positive_next, positive_next,
               sizeof(motor_hall_decoder_positive_next));
  motor_hall_decoder_core_clock_hz = core_clock_hz;
  motor_hall_decoder_sample.state = initial_state;
  motor_hall_decoder_sample.valid =
    ((initial_state != 0U) && (initial_state != 7U));
  motor_hall_decoder_sample.timestamp_cycles = initial_timestamp_cycles;
  motor_hall_decoder_last_direction = 0;
  motor_hall_decoder_speed_reset(initial_timestamp_cycles);
  motor_hall_decoder_initialized = true;
  return true;
}

bool motor_hall_decoder_edge_process(uint8_t state, uint32_t edge_count,
                                     uint32_t timestamp_cycles)
{
  uint8_t previous_state;
  bool state_valid;
  bool previous_valid;
  int8_t direction;
  uint32_t interval_cycles;

  if ((!motor_hall_decoder_initialized) || (state > 7U)) return false;
  motor_hall_decoder_sequence++;
  previous_state = motor_hall_decoder_sample.state;
  state_valid = ((state != 0U) && (state != 7U));
  previous_valid = ((previous_state != 0U) && (previous_state != 7U));
  motor_hall_decoder_sample.state = state;
  motor_hall_decoder_sample.valid = state_valid;
  motor_hall_decoder_sample.edge_count = edge_count;

  if (state == previous_state)
  {
    motor_hall_decoder_sample.duplicate_count++;
    motor_hall_decoder_sequence++;
    return false;
  }
  if ((!state_valid) || (!previous_valid))
  {
    motor_hall_decoder_sample.invalid_transition_count++;
    motor_hall_decoder_sample.direction = 0;
    motor_hall_decoder_sample.electrical_frequency_millihz = 0U;
    motor_hall_decoder_last_direction = 0;
    motor_hall_decoder_speed_reset(timestamp_cycles);
    motor_hall_decoder_sequence++;
    return false;
  }
  if (motor_hall_decoder_positive_next[previous_state] == state)
  {
    motor_hall_decoder_sample.positive_count++;
    direction = 1;
  }
  else if (motor_hall_decoder_positive_next[state] == previous_state)
  {
    motor_hall_decoder_sample.negative_count++;
    direction = -1;
  }
  else
  {
    motor_hall_decoder_sample.invalid_transition_count++;
    motor_hall_decoder_sample.direction = 0;
    motor_hall_decoder_sample.electrical_frequency_millihz = 0U;
    motor_hall_decoder_last_direction = 0;
    motor_hall_decoder_speed_reset(timestamp_cycles);
    motor_hall_decoder_sequence++;
    return false;
  }

  motor_hall_decoder_sample.direction = direction;
  motor_hall_decoder_sample.timestamp_cycles = timestamp_cycles;
  if (direction != motor_hall_decoder_last_direction)
  {
    motor_hall_decoder_last_direction = direction;
    motor_hall_decoder_sample.electrical_frequency_millihz = 0U;
    motor_hall_decoder_speed_reset(timestamp_cycles);
  }
  else
  {
    interval_cycles = timestamp_cycles - motor_hall_decoder_last_edge_cycles;
    motor_hall_decoder_last_edge_cycles = timestamp_cycles;
    motor_hall_decoder_speed_push(interval_cycles);
  }
  motor_hall_decoder_sequence++;
  return true;
}

bool motor_hall_decoder_sample_read(motor_hall_sample_t *sample)
{
  uint32_t sequence_before;
  uint32_t sequence_after;

  if ((sample == NULL) || (!motor_hall_decoder_initialized)) return false;
  do
  {
    sequence_before = motor_hall_decoder_sequence;
    if ((sequence_before & 1U) != 0U) continue;
    *sample = motor_hall_decoder_sample;
    sequence_after = motor_hall_decoder_sequence;
  } while ((sequence_before != sequence_after) ||
           ((sequence_after & 1U) != 0U));
  return true;
}
