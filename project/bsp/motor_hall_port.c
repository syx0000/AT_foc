#include <stddef.h>
#include "at32f45x_conf.h"
#include "motor_hall_port.h"
#include "motor_timebase.h"
#include "at32f45x_wk_config.h"

static volatile motor_hall_sample_t motor_hall_sample;
static uint32_t motor_hall_cycle_start_cycles;
static uint8_t motor_hall_cycle_transition_count;
static int8_t motor_hall_last_direction;

/* 正向六步定义：1→5→4→6→2→3→1；机械正方向后续由整机定义确认。 */
static const uint8_t motor_hall_positive_next[8] =
{
  0U, 5U, 3U, 1U, 6U, 4U, 2U, 0U
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
  motor_hall_sample.direction = 0;
  motor_hall_cycle_start_cycles = motor_hall_sample.timestamp_cycles;
  motor_hall_cycle_transition_count = 0U;
  motor_hall_last_direction = 0;
}

void motor_hall_port_edge_capture(void)
{
  uint8_t state = motor_hall_port_state_get();
  uint8_t previous_state = motor_hall_sample.state;
  bool state_valid = ((state != 0U) && (state != 7U));
  bool previous_valid = ((previous_state != 0U) && (previous_state != 7U));
  uint32_t timestamp = motor_timebase_cycles_get();
  uint32_t cycle_cycles;
  int8_t direction;

  motor_hall_sample.state = state;
  motor_hall_sample.valid = state_valid;
  motor_hall_sample.timestamp_cycles = timestamp;
  motor_hall_sample.edge_count++;

  if (state == previous_state)
  {
    motor_hall_sample.duplicate_count++;
    return;
  }

  if ((!state_valid) || (!previous_valid))
  {
    motor_hall_sample.invalid_transition_count++;
    motor_hall_cycle_transition_count = 0U;
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
    motor_hall_cycle_transition_count = 0U;
    motor_hall_last_direction = 0;
    return;
  }

  motor_hall_sample.direction = direction;

  if (direction != motor_hall_last_direction)
  {
    motor_hall_last_direction = direction;
    motor_hall_cycle_start_cycles = timestamp;
    motor_hall_cycle_transition_count = 0U;
    return;
  }

  motor_hall_cycle_transition_count++;
  if (motor_hall_cycle_transition_count >= 6U)
  {
    cycle_cycles = timestamp - motor_hall_cycle_start_cycles;
    if (cycle_cycles != 0U)
    {
      motor_hall_sample.electrical_frequency_millihz =
        (uint32_t)(((uint64_t)system_core_clock * 1000U) / cycle_cycles);
    }
    motor_hall_cycle_start_cycles = timestamp;
    motor_hall_cycle_transition_count = 0U;
  }
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
