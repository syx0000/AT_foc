/**
 * @file    motor_hall_port.c
 * @brief   电机Hall硬件端口 功能实现
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#include <stddef.h>
#include "at32f45x_conf.h"
#include "motor_board_config.h"
#include "motor_hall_port.h"
#include "motor_timebase.h"
#include "at32f45x_wk_config.h"

static volatile motor_hall_port_sample_t motor_hall_port_sample;
static volatile uint64_t motor_hall_timer_epoch;
static volatile uint64_t motor_hall_capture_ticks;

/**
 * @brief 读取三路Hall GPIO并组合为3位状态。
 * @param 无。
 * @return bit0=HA、bit1=HB、bit2=HC，范围0..7。
 */
uint8_t motor_hall_port_raw_state_get(void)
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

/**
 * @brief 将Hall输入切换为TMR4 XOR捕获并使能滤波后的边沿中断。
 * @param 无。
 * @return 无。
 * @details 三路Hall引脚配置为TMR4复用输入；
 *          CH1/CH2/CH3异或后经数字滤波送入C1，C1INC作为触发输入，每个
 *          滤波后的Hall边沿产生一次TRIGGER中断。计数器以1 MHz自由运行，
 *          C1硬件锁存边沿时标，溢出中断扩展计数；DWT只保留事件诊断时间。
 */
static void motor_hall_port_timer_xor_init(void)
{
  gpio_init_type gpio_init_struct;
  tmr_input_config_type tmr_input_struct;


  crm_periph_clock_enable(MOTOR_HALL_CAPTURE_CRM_CLK, TRUE);

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_pins = HA_PIN;
  gpio_init(HA_GPIO_PORT, &gpio_init_struct);
  gpio_init_struct.gpio_pins = HB_PIN;
  gpio_init(HB_GPIO_PORT, &gpio_init_struct);
  gpio_init_struct.gpio_pins = HC_PIN;
  gpio_init(HC_GPIO_PORT, &gpio_init_struct);
  gpio_pin_mux_config(HA_GPIO_PORT, MOTOR_HALL_A_PIN_SOURCE,
                      MOTOR_HALL_GPIO_MUX);
  gpio_pin_mux_config(HB_GPIO_PORT, MOTOR_HALL_B_PIN_SOURCE,
                      MOTOR_HALL_GPIO_MUX);
  gpio_pin_mux_config(HC_GPIO_PORT, MOTOR_HALL_C_PIN_SOURCE,
                      MOTOR_HALL_GPIO_MUX);

  tmr_reset(MOTOR_HALL_CAPTURE_TIMER);
  tmr_base_init(MOTOR_HALL_CAPTURE_TIMER, 0xFFFFU,
                (system_core_clock / 1000000U) - 1U);
  tmr_cnt_dir_set(MOTOR_HALL_CAPTURE_TIMER, TMR_COUNT_UP);
  tmr_clock_source_div_set(MOTOR_HALL_CAPTURE_TIMER,
                           MOTOR_HALL_CAPTURE_FILTER_CLK_DIV);

  tmr_input_default_para_init(&tmr_input_struct);
  tmr_input_struct.input_channel_select = TMR_SELECT_CHANNEL_1;
  tmr_input_struct.input_mapped_select = TMR_CC_CHANNEL_MAPPED_STI;
  tmr_input_struct.input_polarity_select = TMR_INPUT_RISING_EDGE;
  tmr_input_struct.input_filter_value = MOTOR_HALL_CAPTURE_FILTER;
  tmr_input_channel_init(MOTOR_HALL_CAPTURE_TIMER, &tmr_input_struct,
                         TMR_CHANNEL_INPUT_DIV_1);
  tmr_channel1_input_select(MOTOR_HALL_CAPTURE_TIMER,
                            TMR_CHANEL1_2_3_CONNECTED_C1IRAW_XOR);
  tmr_trigger_input_select(MOTOR_HALL_CAPTURE_TIMER, TMR_SUB_INPUT_SEL_C1INC);
  tmr_sub_mode_select(MOTOR_HALL_CAPTURE_TIMER, TMR_SUB_MODE_DIABLE);

  tmr_flag_clear(MOTOR_HALL_CAPTURE_TIMER,
                 TMR_TRIGGER_FLAG | TMR_C1_FLAG | TMR_OVF_FLAG);
  tmr_interrupt_enable(MOTOR_HALL_CAPTURE_TIMER, TMR_TRIGGER_INT | TMR_OVF_INT, TRUE);
  nvic_irq_enable(MOTOR_HALL_CAPTURE_IRQn, 2, 0);
  tmr_counter_enable(MOTOR_HALL_CAPTURE_TIMER, TRUE);
}

void motor_hall_port_init(void)
{
  uint8_t state;

  motor_hall_port_timer_xor_init();
  motor_hall_timer_epoch = 0U;
  motor_hall_capture_ticks = 0U;
  state = motor_hall_port_raw_state_get();
  motor_hall_port_sample.state = state;
  motor_hall_port_sample.edge_count = 0U;
  motor_hall_port_sample.timestamp_cycles = motor_timebase_cycles_get();
  motor_hall_port_sample.capture_us = 0U;
}

void motor_hall_port_overflow_capture(void)
{
  uint32_t primask=__get_PRIMASK();
  __disable_irq();
  motor_hall_timer_epoch += 65536ULL;
  tmr_flag_clear(MOTOR_HALL_CAPTURE_TIMER,TMR_OVF_FLAG);
  __set_PRIMASK(primask);
}

static uint64_t motor_hall_timer_now(void)
{
  uint64_t epoch = motor_hall_timer_epoch;
  uint32_t count = tmr_counter_value_get(MOTOR_HALL_CAPTURE_TIMER);
  if (tmr_flag_get(MOTOR_HALL_CAPTURE_TIMER, TMR_OVF_FLAG) != RESET)
  {
    epoch += 65536ULL;
    count = tmr_counter_value_get(MOTOR_HALL_CAPTURE_TIMER);
  }
  return epoch + count;
}

uint32_t motor_hall_port_quiet_us(void)
{
  uint32_t primask = __get_PRIMASK();
  uint64_t elapsed;
  __disable_irq();
  elapsed = motor_hall_timer_now() - motor_hall_capture_ticks;
  __set_PRIMASK(primask);
  return elapsed > UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed;
}

void motor_hall_port_edge_capture(void)
{
  uint32_t captured = tmr_channel_value_get(MOTOR_HALL_CAPTURE_TIMER, TMR_SELECT_CHANNEL_1);
  uint64_t now = motor_hall_timer_now();
  uint64_t epoch = now & ~65535ULL;
  uint32_t primask;
  if (captured > (uint32_t)(now & 65535U)) epoch -= 65536ULL;
  primask=__get_PRIMASK();
  __disable_irq();
  motor_hall_capture_ticks = epoch + captured;
  motor_hall_port_sample.capture_us = (uint32_t)motor_hall_capture_ticks;
  motor_hall_port_sample.state = motor_hall_port_raw_state_get();
  motor_hall_port_sample.edge_count++;
  motor_hall_port_sample.timestamp_cycles = motor_timebase_cycles_get();
  __set_PRIMASK(primask);
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
