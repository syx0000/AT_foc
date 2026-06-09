/* add user code begin Header */
/**
  **************************************************************************
  * @file     wk_tmr.c
  * @brief    work bench config program
  **************************************************************************
  * Copyright (c) 2025, Artery Technology, All rights reserved.
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
/* add user code end Header */

/* Includes ------------------------------------------------------------------*/
#include "wk_tmr.h"

/* add user code begin 0 */

/* ============================================================
 * ADC ISR performance counters (DWT cycles, set by foc_api.c)
 * Migrated from cubemx_yxsui/Core/Src/tim.c
 * ============================================================ */
volatile uint32_t g_adc_isr_t_read    = 0;
volatile uint32_t g_adc_isr_t_enc     = 0;
volatile uint32_t g_adc_isr_t_pos     = 0;
volatile uint32_t g_adc_isr_t_vel     = 0;
volatile uint32_t g_adc_isr_t_cur     = 0;
volatile uint32_t g_adc_isr_t_read_max = 0;
volatile uint32_t g_adc_isr_t_enc_max  = 0;
volatile uint32_t g_adc_isr_t_pos_max  = 0;
volatile uint32_t g_adc_isr_t_vel_max  = 0;
volatile uint32_t g_adc_isr_t_cur_max  = 0;

/* ============================================================
 * DWT cycle counter (for performance measurement)
 * Migrated from cubemx_yxsui/Core/Src/tim.c
 * ============================================================ */

/**
 * @brief  Initialize DWT cycle counter
 * @note   CPU frequency is lazy-loaded on first DWT_GetMicros/CyclesToUs call,
 *         so DWT_Init() can be called BEFORE wk_system_clock_config()
 */
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief  Get current DWT cycle count
 * @retval 32-bit cycle count (overflows every ~22s @192MHz)
 */
uint32_t DWT_GetCycles(void)
{
    return DWT->CYCCNT;
}

/* CPU frequency for cycles->us conversion (lazy-loaded on first use) */
static uint32_t s_cpu_freq_mhz = 0;

static void update_cpu_freq(void)
{
    crm_clocks_freq_type clocks;
    crm_clocks_freq_get(&clocks);
    s_cpu_freq_mhz = clocks.sclk_freq / 1000000U;
    if (s_cpu_freq_mhz == 0) s_cpu_freq_mhz = 192;  /* fallback */
}

uint32_t DWT_GetMicros(void)
{
    if (s_cpu_freq_mhz == 0) update_cpu_freq();
    return DWT->CYCCNT / s_cpu_freq_mhz;
}

uint32_t DWT_CyclesToUs(uint32_t cycles)
{
    if (s_cpu_freq_mhz == 0) update_cpu_freq();
    return cycles / s_cpu_freq_mhz;
}

/* add user code end 0 */

/**
  * @brief  init tmr1 function.
  * @param  none
  * @retval none
  */
void wk_tmr1_init(void)
{
  /* add user code begin tmr1_init 0 */

  /* add user code end tmr1_init 0 */

  gpio_init_type gpio_init_struct;
  tmr_output_config_type tmr_output_struct;
  tmr_brkdt_config_type tmr_brkdt_struct;

  gpio_default_para_init(&gpio_init_struct);

  /* add user code begin tmr1_init 1 */

  /* add user code end tmr1_init 1 */

  /* configure the CH1C pin */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE7, GPIO_MUX_1);
  gpio_init_struct.gpio_pins = GPIO_PINS_7;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(GPIOA, &gpio_init_struct);

  /* configure the CH2C pin */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE0, GPIO_MUX_1);
  gpio_init_struct.gpio_pins = GPIO_PINS_0;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(GPIOB, &gpio_init_struct);

  /* configure the CH3C pin */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE1, GPIO_MUX_1);
  gpio_init_struct.gpio_pins = GPIO_PINS_1;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(GPIOB, &gpio_init_struct);

  /* configure the BRK pin */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE12, GPIO_MUX_1);
  gpio_init_struct.gpio_pins = GPIO_PINS_12;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(GPIOB, &gpio_init_struct);

  /* configure the CH1 pin */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE8, GPIO_MUX_1);
  gpio_init_struct.gpio_pins = GPIO_PINS_8;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(GPIOA, &gpio_init_struct);

  /* configure the CH2 pin */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_1);
  gpio_init_struct.gpio_pins = GPIO_PINS_9;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(GPIOA, &gpio_init_struct);

  /* configure the CH3 pin */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE10, GPIO_MUX_1);
  gpio_init_struct.gpio_pins = GPIO_PINS_10;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(GPIOA, &gpio_init_struct);

  /* configure counter settings */
  tmr_cnt_dir_set(TMR1, TMR_COUNT_TWO_WAY_3);
  tmr_clock_source_div_set(TMR1, TMR_CLOCK_DIV1);
  tmr_repetition_counter_set(TMR1, 1);
  tmr_period_buffer_enable(TMR1, FALSE);
  tmr_base_init(TMR1, 9599, 0);

  /* configure primary mode settings */
  tmr_sub_sync_mode_set(TMR1, FALSE);
  tmr_primary_mode_select(TMR1, TMR_PRIMARY_SEL_OVERFLOW);

  /* configure channel 1 output settings */
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
  tmr_output_struct.oc_output_state = TRUE;
  tmr_output_struct.occ_output_state = TRUE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.oc_idle_state = FALSE;
  tmr_output_struct.occ_idle_state = FALSE;
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &tmr_output_struct);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, 0);
  tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_1, FALSE);

  tmr_output_channel_immediately_set(TMR1, TMR_SELECT_CHANNEL_1, TRUE);

  /* configure channel 2 output settings */
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
  tmr_output_struct.oc_output_state = TRUE;
  tmr_output_struct.occ_output_state = TRUE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.oc_idle_state = FALSE;
  tmr_output_struct.occ_idle_state = FALSE;
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_2, &tmr_output_struct);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, 0);
  tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_2, FALSE);

  tmr_output_channel_immediately_set(TMR1, TMR_SELECT_CHANNEL_2, TRUE);

  /* configure channel 3 output settings */
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
  tmr_output_struct.oc_output_state = TRUE;
  tmr_output_struct.occ_output_state = TRUE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.oc_idle_state = FALSE;
  tmr_output_struct.occ_idle_state = FALSE;
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_3, &tmr_output_struct);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, 0);
  tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_3, FALSE);

  tmr_output_channel_immediately_set(TMR1, TMR_SELECT_CHANNEL_3, TRUE);

  /* configure channel 4 output settings */
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_OFF;
  tmr_output_struct.oc_output_state = TRUE;
  tmr_output_struct.occ_output_state = FALSE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.oc_idle_state = FALSE;
  tmr_output_struct.occ_idle_state = FALSE;
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_4, &tmr_output_struct);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, 0);
  tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_4, FALSE);

  /* configure break and dead-time settings */
  tmr_brkdt_struct.brk_enable = FALSE;
  tmr_brkdt_struct.auto_output_enable = FALSE;
  tmr_brkdt_struct.brk_polarity = TMR_BRK_INPUT_ACTIVE_LOW;
  tmr_brkdt_struct.fcsoen_state = FALSE;
  tmr_brkdt_struct.fcsodis_state = FALSE;
  tmr_brkdt_struct.wp_level = TMR_WP_OFF;
  tmr_brkdt_struct.deadtime = 80;
  tmr_brkdt_config(TMR1, &tmr_brkdt_struct);

  tmr_brk_filter_value_set(TMR1, 0);

  tmr_output_enable(TMR1, TRUE);

  tmr_counter_enable(TMR1, TRUE);

  /* enable c4ien interrupt */
  tmr_interrupt_enable(TMR1, TMR_C4_INT, TRUE);

  /* add user code begin tmr1_init 2 */

  /* add user code end tmr1_init 2 */
}

/**
  * @brief  init tmr6 function.
  * @param  none
  * @retval none
  */
void wk_tmr6_init(void)
{
  /* add user code begin tmr6_init 0 */

  /* add user code end tmr6_init 0 */

  /* add user code begin tmr6_init 1 */

  /* add user code end tmr6_init 1 */

  /* configure counter settings */
  tmr_cnt_dir_set(TMR6, TMR_COUNT_UP);
  tmr_period_buffer_enable(TMR6, FALSE);
  tmr_base_init(TMR6, 999, 191);

  /* configure primary mode settings */
  tmr_primary_mode_select(TMR6, TMR_PRIMARY_SEL_OVERFLOW);

  tmr_counter_enable(TMR6, TRUE);

  /* add user code begin tmr6_init 2 */

  /* add user code end tmr6_init 2 */
}

/* add user code begin 1 */

/* ============================================================
 * TMR1 PWM control (FOC motor drive)
 * ============================================================ */

/**
 * @brief  Start TMR1 PWM output (enable counter + output + all 3 channels)
 * @note   Call after wk_tmr1_init(), enables complementary PWM with dead-time
 */
void TIM1_PWM_Start(void)
{
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, 0);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, 0);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, 0);
    tmr_output_enable(TMR1, TRUE);
    tmr_counter_enable(TMR1, TRUE);
}

/**
 * @brief  Stop TMR1 PWM output (disable output, keep counter for ADC trigger)
 * @note   Safe shutdown: all outputs go to inactive state
 */
void TIM1_PWM_Stop(void)
{
    tmr_output_enable(TMR1, FALSE);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, 0);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, 0);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, 0);
}

/**
 * @brief  Set three-phase PWM duty cycle
 * @param  ccr1: Phase A compare value (0 ~ 9599)
 * @param  ccr2: Phase B compare value (0 ~ 9599)
 * @param  ccr3: Phase C compare value (0 ~ 9599)
 */
void TIM1_SetDuty(uint32_t ccr1, uint32_t ccr2, uint32_t ccr3)
{
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, ccr1);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, ccr2);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, ccr3);
}

/**
 * @brief  PWM compare register write (called by SVPWM, hot path)
 * @note   Migrated from cubemx_yxsui/foc/foc_fast/foc_bsp.c
 *         Direct register write for ISR speed (avoids function call overhead).
 */
void pwm_ccr_set(uint32_t ccr1, uint32_t ccr2, uint32_t ccr3)
{
    TMR1->c1dt = ccr1;
    TMR1->c2dt = ccr2;
    TMR1->c3dt = ccr3;
}

/* add user code end 1 */
