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

#if (MOTOR_HALL_CAPTURE_MODE == MOTOR_HALL_CAPTURE_MODE_TIMER_XOR)
/**
 * @brief 将Hall输入切换为TMR4 XOR捕获并使能滤波后的边沿中断。
 * @param 无。
 * @return 无。
 * @details 关闭WorkBench生成的EXINT 6/7/8中断，三路引脚改为TMR4复用输入；
 *          CH1/CH2/CH3异或后经数字滤波送入C1，C1INC作为触发输入，每个
 *          滤波后的Hall边沿产生一次TRIGGER中断。计数器仅作为滤波时基，
 *          边沿间隔仍使用DWT时间戳。
 */
static void motor_hall_port_timer_xor_init(void)
{
  gpio_init_type gpio_init_struct;
  tmr_input_config_type tmr_input_struct;

  exint_interrupt_enable(EXINT_LINE_6 | EXINT_LINE_7 | EXINT_LINE_8, FALSE);
  exint_flag_clear(EXINT_LINE_6 | EXINT_LINE_7 | EXINT_LINE_8);

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
  tmr_base_init(MOTOR_HALL_CAPTURE_TIMER, 0xFFFFU, 0U);
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
  tmr_sub_mode_select(MOTOR_HALL_CAPTURE_TIMER, TMR_SUB_RESET_MODE);

  tmr_flag_clear(MOTOR_HALL_CAPTURE_TIMER,
                 TMR_TRIGGER_FLAG | TMR_C1_FLAG | TMR_OVF_FLAG);
  tmr_interrupt_enable(MOTOR_HALL_CAPTURE_TIMER, TMR_TRIGGER_INT, TRUE);
  nvic_irq_enable(MOTOR_HALL_CAPTURE_IRQn, 2, 0);
  tmr_counter_enable(MOTOR_HALL_CAPTURE_TIMER, TRUE);
}
#endif

void motor_hall_port_init(void)
{
  uint8_t state;

#if (MOTOR_HALL_CAPTURE_MODE == MOTOR_HALL_CAPTURE_MODE_TIMER_XOR)
  motor_hall_port_timer_xor_init();
#endif
  state = motor_hall_port_state_get();
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
