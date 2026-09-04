#include <stddef.h>
#include "at32f45x_conf.h"
#include "motor_hall_port.h"
#include "motor_timebase.h"
#include "at32f45x_wk_config.h"

static volatile motor_hall_port_sample_t motor_hall_port_sample;

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

  motor_hall_port_sample.state = state;
  motor_hall_port_sample.edge_count = 0U;
  motor_hall_port_sample.timestamp_cycles = motor_timebase_cycles_get();
}

void motor_hall_port_edge_capture(void)
{
  motor_hall_port_sample.state = motor_hall_port_state_get();
  motor_hall_port_sample.edge_count++;
  motor_hall_port_sample.timestamp_cycles = motor_timebase_cycles_get();
}

bool motor_hall_port_sample_read(motor_hall_port_sample_t *sample)
{
  uint32_t primask;

  if (sample == NULL)
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *sample = motor_hall_port_sample;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return true;
}
