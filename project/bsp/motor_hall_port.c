#include <stddef.h>
#include "at32f45x_conf.h"
#include "motor_hall_port.h"
#include "motor_control_config.h"
#include "motor_timebase.h"
#include "at32f45x_wk_config.h"

static volatile motor_hall_sample_t motor_hall_sample;
#define MOTOR_HALL_SPEED_WINDOW_SIZE 6U
static uint32_t motor_hall_interval_cycles[MOTOR_HALL_SPEED_WINDOW_SIZE];
static uint64_t motor_hall_interval_sum_cycles;
static uint32_t motor_hall_last_edge_cycles;
static uint8_t motor_hall_interval_index;
static uint8_t motor_hall_interval_count;
static int8_t motor_hall_last_direction;

/**
 * @brief 清空Hall边沿周期测速滑动窗口。
 * @param timestamp_cycles 当前合法边沿的DWT周期时间戳，作为下一段间隔起点。
 * @return 无。
 * @details 在初始化、方向改变或非法跳变后调用；清零历史间隔，防止跨方向或异常数据参与测速。
 */
static void motor_hall_speed_window_reset(uint32_t timestamp_cycles)
{
  uint8_t index;

  for (index = 0U; index < MOTOR_HALL_SPEED_WINDOW_SIZE; index++)
  {
    motor_hall_interval_cycles[index] = 0U;
  }
  motor_hall_interval_sum_cycles = 0U;
  motor_hall_last_edge_cycles = timestamp_cycles;
  motor_hall_interval_index = 0U;
  motor_hall_interval_count = 0U;
}

/**
 * @brief 写入一个合法Hall边沿间隔并更新电频率。
 * @param interval_cycles 相邻两个同方向合法Hall边沿之间的DWT周期数，必须大于0。
 * @return 无。
 * @details 保存最近最多6段间隔，以平均扇区周期乘6估算完整电周期；窗口未填满时也会逐边沿输出测速结果。
 */
static void motor_hall_speed_interval_push(uint32_t interval_cycles)
{
  if (interval_cycles == 0U)
  {
    return;
  }

  if (motor_hall_interval_count >= MOTOR_HALL_SPEED_WINDOW_SIZE)
  {
    motor_hall_interval_sum_cycles -=
      motor_hall_interval_cycles[motor_hall_interval_index];
  }
  else
  {
    motor_hall_interval_count++;
  }

  motor_hall_interval_cycles[motor_hall_interval_index] = interval_cycles;
  motor_hall_interval_sum_cycles += interval_cycles;
  motor_hall_interval_index++;
  if (motor_hall_interval_index >= MOTOR_HALL_SPEED_WINDOW_SIZE)
  {
    motor_hall_interval_index = 0U;
  }

  motor_hall_sample.electrical_frequency_millihz =
    (uint32_t)(((uint64_t)system_core_clock * 1000U *
                motor_hall_interval_count) /
               (motor_hall_interval_sum_cycles * 6U));
  motor_hall_sample.frequency_update_count++;
}

/* 正向六步定义：1→5→4→6→2→3→1；机械正方向后续由整机定义确认。 */
static const uint8_t motor_hall_positive_next[8] =
{
  0U,
  MOTOR_HALL_POSITIVE_NEXT_STATE_1,
  MOTOR_HALL_POSITIVE_NEXT_STATE_2,
  MOTOR_HALL_POSITIVE_NEXT_STATE_3,
  MOTOR_HALL_POSITIVE_NEXT_STATE_4,
  MOTOR_HALL_POSITIVE_NEXT_STATE_5,
  MOTOR_HALL_POSITIVE_NEXT_STATE_6,
  0U
};

/**
 * @brief 读取三路Hall GPIO并组合为3位状态。
 * @param 无。
 * @return bit0=HA、bit1=HB、bit2=HC，范围0..7。
 */
static uint8_t motor_hall_port_state_get(void)
{
  uint8_t state = 0U;

  if (gpio_input_data_bit_read(HA_GPIO_PORT, HA_PIN) != RESET)
  {
    state |= 0x01U;
  }
  if (gpio_input_data_bit_read(HB_GPIO_PORT, HB_PIN) != RESET)
  {
    state |= 0x02U;
  }
  if (gpio_input_data_bit_read(HC_GPIO_PORT, HC_PIN) != RESET)
  {
    state |= 0x04U;
  }
  return state;
}

void motor_hall_port_init(void)
{
  uint8_t state = motor_hall_port_state_get();

  motor_hall_sample.state = state;
  motor_hall_sample.valid = ((state != 0U) && (state != 7U));
  motor_hall_sample.edge_count = 0U;
  motor_hall_sample.timestamp_cycles = motor_timebase_cycles_get();
  motor_hall_sample.positive_count = 0U;
  motor_hall_sample.negative_count = 0U;
  motor_hall_sample.invalid_transition_count = 0U;
  motor_hall_sample.duplicate_count = 0U;
  motor_hall_sample.electrical_frequency_millihz = 0U;
  motor_hall_sample.frequency_update_count = 0U;
  motor_hall_sample.direction = 0;
  motor_hall_speed_window_reset(motor_hall_sample.timestamp_cycles);
  motor_hall_last_direction = 0;
}

void motor_hall_port_edge_capture(void)
{
  uint8_t state = motor_hall_port_state_get();
  uint8_t previous_state = motor_hall_sample.state;
  bool state_valid = ((state != 0U) && (state != 7U));
  bool previous_valid = ((previous_state != 0U) && (previous_state != 7U));
  uint32_t timestamp = motor_timebase_cycles_get();
  uint32_t interval_cycles;
  int8_t direction;

  motor_hall_sample.state = state;
  motor_hall_sample.valid = state_valid;
  motor_hall_sample.edge_count++;

  if (state == previous_state)
  {
    motor_hall_sample.duplicate_count++;
    return;
  }

  if ((!state_valid) || (!previous_valid))
  {
    motor_hall_sample.invalid_transition_count++;
    motor_hall_sample.direction = 0;
    motor_hall_sample.electrical_frequency_millihz = 0U;
    motor_hall_speed_window_reset(timestamp);
    motor_hall_last_direction = 0;
    return;
  }

  if (motor_hall_positive_next[previous_state] == state)
  {
    motor_hall_sample.positive_count++;
    direction = 1;
  }
  else if (motor_hall_positive_next[state] == previous_state)
  {
    motor_hall_sample.negative_count++;
    direction = -1;
  }
  else
  {
    motor_hall_sample.invalid_transition_count++;
    motor_hall_sample.direction = 0;
    motor_hall_sample.electrical_frequency_millihz = 0U;
    motor_hall_speed_window_reset(timestamp);
    motor_hall_last_direction = 0;
    return;
  }

  motor_hall_sample.direction = direction;
  motor_hall_sample.timestamp_cycles = timestamp;

  if (direction != motor_hall_last_direction)
  {
    motor_hall_last_direction = direction;
    motor_hall_sample.electrical_frequency_millihz = 0U;
    motor_hall_speed_window_reset(timestamp);
    return;
  }

  interval_cycles = timestamp - motor_hall_last_edge_cycles;
  motor_hall_last_edge_cycles = timestamp;
  motor_hall_speed_interval_push(interval_cycles);
}

bool motor_hall_port_sample_read(motor_hall_sample_t *sample)
{
  uint32_t primask;

  if (sample == NULL)
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *sample = motor_hall_sample;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return true;
}
