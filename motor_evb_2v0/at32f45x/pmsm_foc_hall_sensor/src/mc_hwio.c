/**
  **************************************************************************
  * @file     mc_hwio.c
  * @brief    Hardware peripheral configuration
  **************************************************************************
  *                       Copyright notice & Disclaimer
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

#include "mc_lib.h"

/** @addtogroup motor_evb_2v0
  * @{
  */

/** @addtogroup pmsm_foc_hall_sensor pmsm_foc_hall_sensor
  * @{
  */

/** @defgroup mc_hwio
  * @brief Hardware peripheral configuration
  * @{
  */

__IO uint16_t adc_in_tab[ADC_IDX_MAX] = {0};

/**
  * @brief  initialization of nested vectored interrupt controller
  * @param  none
  * @retval none
  */
void nvic_config(void)
{
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  /* pwm isr configuration */
  nvic_irq_enable(ADVTMR_PWM_BRK_IRQn, 0, 0);
  nvic_irq_enable(ADVTMR_PWM_CYCLE_IRQn, 2, 0);

  /* adc isr configuration */
  nvic_irq_enable(ADC_SHUNT_SAMP_READY_IRQn, 1, 0);

  /* speed control loop isr configuration */
  nvic_irq_enable(SPEED_LOOP_TIMER_IRQn, 4, 0);

#if defined HALL_SENSORS
  nvic_irq_enable(HALL_CAPTURE_IRQn, 3, 0);
#endif

#if defined MT_METHOD
  /* nvic_irq_enable(ENCODER_CAPTURE_IRQn, 6, 0); -- no encoder on target board */
#endif

#if defined ABZ || defined MAGNET_ENCODER_W_ABZ
  /* encoder index configuration */
  /* nvic_irq_enable(EXINT_ENCODER_IDX_IRQn, 0, 0); -- no encoder on target board */
#endif

#if defined MAGNET_ENCODER_WO_ABZ
  /* nvic_irq_enable(SYNC_TIMER_CH_IRQn, 0, 0); -- no magnetic encoder sync */
#endif

  /* systick interrupt nvic init */
  nvic_irq_enable(SysTick_IRQn, 5, 0);

  /* button interrupt nvic init */
  /* nvic_irq_enable(BUTTON_EXINT_IRQn, 7, 0); -- no button on target board */
  /* usart1 interrupt nvic init */
  nvic_irq_enable(COMM_UART_IRQn, 8, 0);
}

/**
  * @brief  initialization of a timer for PWM
  * @param  none
  * @retval none
  */
void tmr_pwm_init(void)
{
  gpio_init_type gpio_init_struct = {0};
  tmr_output_config_type tmr_output_struct;
  tmr_brkdt_config_type tmr_brkdt_config_struct = {0};

  /* enable pwm timer/high side gpio/low side gpio clock */
  crm_periph_clock_enable(PWM_ADVANCE_TIMER_CRM_CLK, TRUE);
  crm_periph_clock_enable(PWM_PHASE_A_HI_GPIO_CRM_CLK, TRUE);
  crm_periph_clock_enable(PWM_PHASE_B_HI_GPIO_CRM_CLK, TRUE);
  crm_periph_clock_enable(PWM_PHASE_C_HI_GPIO_CRM_CLK, TRUE);
  crm_periph_clock_enable(PWM_PHASE_A_LOW_GPIO_CRM_CLK, TRUE);
  crm_periph_clock_enable(PWM_PHASE_B_LOW_GPIO_CRM_CLK, TRUE);
  crm_periph_clock_enable(PWM_PHASE_C_LOW_GPIO_CRM_CLK, TRUE);
  crm_periph_clock_enable(PWM_BRK_GPIO_CRM_CLK, TRUE);
  crm_periph_clock_enable(TMR_ADC_TRIG_GPIO_CRM_CLK, TRUE);

  /* advance timer output pin Configuration */
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;

  /* High-side, Phase A,B,C Config */
  gpio_init_struct.gpio_pins = PWM_PHASE_A_HI_GPIO_PIN;
  gpio_init(PWM_PHASE_A_HI_PORT, &gpio_init_struct);
  gpio_pin_mux_config(PWM_PHASE_A_HI_PORT, PWM_PHASE_A_HI_PIN_SOURCE, PWM_PHASE_A_HI_IOMUX);

  gpio_init_struct.gpio_pins = PWM_PHASE_B_HI_GPIO_PIN;
  gpio_init(PWM_PHASE_B_HI_PORT, &gpio_init_struct);
  gpio_pin_mux_config(PWM_PHASE_B_HI_PORT, PWM_PHASE_B_HI_PIN_SOURCE, PWM_PHASE_B_HI_IOMUX);

  gpio_init_struct.gpio_pins = PWM_PHASE_C_HI_GPIO_PIN;
  gpio_init(PWM_PHASE_C_HI_PORT, &gpio_init_struct);
  gpio_pin_mux_config(PWM_PHASE_C_HI_PORT, PWM_PHASE_C_HI_PIN_SOURCE, PWM_PHASE_C_HI_IOMUX);

  /* Low-side, Phase A,B,C Config */
#if defined GATE_DRIVER_LOW_SIDE_INVERT
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
#else
  gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;
#endif

  gpio_init_struct.gpio_pins = PWM_PHASE_A_LOW_GPIO_PIN;
  gpio_init(PWM_PHASE_A_LOW_PORT, &gpio_init_struct);
  gpio_pin_mux_config(PWM_PHASE_A_LOW_PORT, PWM_PHASE_A_LOW_PIN_SOURCE, PWM_PHASE_A_LOW_IOMUX);

  gpio_init_struct.gpio_pins = PWM_PHASE_B_LOW_GPIO_PIN;
  gpio_init(PWM_PHASE_B_LOW_PORT, &gpio_init_struct);
  gpio_pin_mux_config(PWM_PHASE_B_LOW_PORT, PWM_PHASE_B_LOW_PIN_SOURCE, PWM_PHASE_B_LOW_IOMUX);

  gpio_init_struct.gpio_pins = PWM_PHASE_C_LOW_GPIO_PIN;
  gpio_init(PWM_PHASE_C_LOW_PORT, &gpio_init_struct);
  gpio_pin_mux_config(PWM_PHASE_C_LOW_PORT, PWM_PHASE_C_LOW_PIN_SOURCE, PWM_PHASE_C_LOW_IOMUX);

  tmr_brkdt_config_struct.brk_enable = FALSE;
  tmr_brkdt_config(PWM_ADVANCE_TIMER, &tmr_brkdt_config_struct);
  /* clear interupt flag */
  tmr_flag_clear(PWM_ADVANCE_TIMER, TMR_BRK_FLAG);

  /* disable brkin ISR */
  tmr_interrupt_enable(PWM_ADVANCE_TIMER, TMR_BRK_INT, FALSE);

  /* Break in input config */
  gpio_init_struct.gpio_pins = PWM_BRK_GPIO_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(PWM_BRK_PORT, &gpio_init_struct);
  gpio_pin_mux_config(PWM_BRK_PORT, PWM_BRK_GPIO_PIN_SOURCE, PWM_BRK_IOMUX);

  /* ADC trigger IO config */
  gpio_init_struct.gpio_pins = TMR_ADC_TRIG_GPIO_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(TMR_ADC_TRIG_PORT, &gpio_init_struct);
  gpio_pin_mux_config(TMR_ADC_TRIG_PORT, TMR_ADC_TRIG_GPIO_PIN_SOURCE, TMR_ADC_TRIG_IOMUX);

  /* enable pwm compare value update buffer */
  tmr_output_channel_buffer_enable(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_1, TRUE);
  tmr_output_channel_buffer_enable(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_2, TRUE);
  tmr_output_channel_buffer_enable(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_3, TRUE);

  /* pwm time base configuration*/
#if defined THREE_SHUNT || defined TWO_SHUNT
  tmr_repetition_counter_set(PWM_ADVANCE_TIMER, 1);               /* the pwm cycle isr in underflow (high-side pwm on) */
#elif defined ONE_SHUNT
  tmr_repetition_counter_set(PWM_ADVANCE_TIMER, 0);
#endif
  tmr_base_init(PWM_ADVANCE_TIMER, PWM_PERIOD, 0);
  tmr_cnt_dir_set(PWM_ADVANCE_TIMER, TMR_COUNT_TWO_WAY_1);        /* output compare interrupt flags are set only count-down */
  /* set dead time clock */
  tmr_clock_source_div_set(PWM_ADVANCE_TIMER, DEADTIME_CLK_SFT_BITS);

  /* channel 1,2,3,1C,2C,3C configuration in output mode */
  tmr_channel_value_set(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_1, 0);
  tmr_channel_value_set(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_2, 0);
  tmr_channel_value_set(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_3, 0);

  tmr_output_default_para_init(&tmr_output_struct);
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_output_struct.oc_output_state = TRUE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.oc_idle_state = FALSE;
  tmr_output_struct.occ_output_state = TRUE;
#if defined GATE_DRIVER_LOW_SIDE_INVERT
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_LOW;
  tmr_output_struct.occ_idle_state = TRUE;
#else
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.occ_idle_state = FALSE;
#endif

  /* channel 1, 2, 3 */
  tmr_output_channel_config(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_1, &tmr_output_struct);
  tmr_output_channel_config(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_2, &tmr_output_struct);
  tmr_output_channel_config(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_3, &tmr_output_struct);

  /* TMR CH for ADC trigger */
#if defined THREE_SHUNT || defined TWO_SHUNT
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_output_struct.occ_output_state = FALSE;
  tmr_output_channel_config(ADC_TIMER, ADC_TIMER_SELECT_CHANNEL, &tmr_output_struct);
  tmr_channel_value_set(ADC_TIMER, ADC_TIMER_SELECT_CHANNEL, ADC_TRIG_POS);
#elif defined  ONE_SHUNT
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_output_struct.occ_output_state = FALSE;
  tmr_output_channel_config(ADC_TIMER, ADC_TIMER_SELECT_CHANNEL, &tmr_output_struct);
  tmr_channel_value_set(ADC_TIMER, ADC_TIMER_SELECT_CHANNEL, ADC_TRIG_POS);
  tmr_channel_dma_select(ADC_TIMER, TMR_DMA_REQUEST_BY_CHANNEL);
  tmr_dma_request_enable(ADC_TIMER, TMR_CH_ADC_DMA_REQUEST, TRUE);

  /* dma cofiguration for one shunt */
  dma_init_type dma_init_struct;
  crm_periph_clock_enable(TMR_ADC_DMA_CRM_CLK, TRUE);

  dma_reset(TMR_ADC_DMA_CH);
  dma_default_para_init(&dma_init_struct);
  dma_init_struct.buffer_size = 1;
  dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
  dma_init_struct.memory_base_addr = (uint32_t) &(pwm_duty.adc_trig.second_pos);
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_base_addr = (uint32_t) &(TMR_ADC_DMA_PERIPHERAL_ADDR);
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_HIGH;
  dma_init_struct.loop_mode_enable = TRUE;
  dma_init(TMR_ADC_DMA_CH, &dma_init_struct);

  /* config flexible dma for adc trigger */
  dma_flexible_config(TMR_ADC_DMA, TMR_ADC_DMA_FLEX_CH, TMR_ADC_DMA_FLEX);

  dma_channel_enable(TMR_ADC_DMA_CH, TRUE);
#endif

  /* automatic output enable, break, dead time and lock configuration */
  tmr_brkdt_default_para_init(&tmr_brkdt_config_struct);
  tmr_brkdt_config_struct.brk_enable = TRUE;
  tmr_brkdt_config_struct.auto_output_enable = FALSE;
  tmr_brkdt_config_struct.deadtime = DEADTIME;
  tmr_brkdt_config_struct.fcsodis_state = TRUE;
  tmr_brkdt_config_struct.fcsoen_state = TRUE;
  tmr_brkdt_config_struct.brk_polarity = TMR_BRK_INPUT_ACTIVE_LOW;
  tmr_brkdt_config_struct.wp_level = TMR_WP_OFF;
  tmr_brkdt_config(PWM_ADVANCE_TIMER, &tmr_brkdt_config_struct);

  //tmr_primary_mode_select(PWM_ADVANCE_TIMER, TMR_PRIMARY_SEL_OVERFLOW);

  tmr_flag_clear(PWM_ADVANCE_TIMER, TMR_OVF_FLAG | TMR_BRK_FLAG | TMR_C4_FLAG);
  tmr_interrupt_enable(PWM_ADVANCE_TIMER, TMR_OVF_INT, TRUE);
  tmr_interrupt_enable(PWM_ADVANCE_TIMER, TMR_BRK_INT, TRUE);

  /* disable single pulse mode */
  tmr_one_cycle_mode_enable(PWM_ADVANCE_TIMER, FALSE);
}

/**
  * @brief  initialization of adc ordinary sampling for bus volt, MOS temp. and etc
  * @param  none
  * @retval none
  */
void adc_ordinary_config(void)
{
  gpio_init_type gpio_init_struct = {0};
  adc_common_config_type adc_common_struct;
  adc_base_config_type adc_base_struct;
  dma_init_type dma_init_struct;

  /* adc clock configuration */
  crm_periph_clock_enable(ADC_CONVERTER_CRM_CLK, TRUE);

  adc_common_default_para_init(&adc_common_struct);
  /* config combine mode */
  adc_common_struct.combine_mode = ADC_INDEPENDENT_MODE;
  /* config division,adcclk is division by hclk */
  adc_common_struct.div = ADC_CONVERTER_CRM_CLK_DIV;        /* HCLK Max. CLK = 288M Hz, ADC_CLK = 288/4 = 72M Hz */
  /* config common dma mode,it's not useful in independent mode */
  adc_common_struct.common_dma_mode = ADC_COMMON_DMAMODE_DISABLE;
  /* config common dma request repeat */
  adc_common_struct.common_dma_request_repeat_state = FALSE;
  /* config adjacent adc sampling interval,it's useful for ordinary shifting mode */
  adc_common_struct.sampling_interval = ADC_SAMPLING_INTERVAL_5CYCLES;
  /* config inner temperature sensor and vintrv */
  adc_common_struct.tempervintrv_state = FALSE;
  /* config voltage battery */
  adc_common_struct.vbat_state = FALSE;
  adc_common_config(&adc_common_struct);

  /* dma clock configuration */
  crm_periph_clock_enable(ADC_ORDINARY_DMA_CRM_CLK, TRUE);

  /* gpio clock configuration */
  crm_periph_clock_enable(VOLT_BUS_ADC_GPIO_CRM_CLK, TRUE);
  crm_periph_clock_enable(MOS_TEMP_ADC_GPIO_CRM_CLK, TRUE);
  crm_periph_clock_enable(POTENTIO_ADC_GPIO_CRM_CLK, TRUE);
  crm_periph_clock_enable(IBUS_AVG_ADC_GPIO_CRM_CLK, TRUE);
#if defined VOLT_SENSE || defined WIND_SENSE
  crm_periph_clock_enable(BEMF_A_ADC_GPIO_CRM_CLK, TRUE);
  crm_periph_clock_enable(BEMF_B_ADC_GPIO_CRM_CLK, TRUE);
  crm_periph_clock_enable(BEMF_C_ADC_GPIO_CRM_CLK, TRUE);
#endif

  /* gpio configuration */
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;

  gpio_init_struct.gpio_pins = VOLT_BUS_ADC_GPIO_PIN;
  gpio_init(VOLT_BUS_ADC_PORT, &gpio_init_struct);

  gpio_init_struct.gpio_pins = MOS_TEMP_ADC_GPIO_PIN;
  gpio_init(MOS_TEMP_ADC_PORT, &gpio_init_struct);

  gpio_init_struct.gpio_pins = POTENTIO_ADC_GPIO_PIN;
  gpio_init(POTENTIO_ADC_PORT, &gpio_init_struct);

  gpio_init_struct.gpio_pins = IBUS_AVG_ADC_GPIO_PIN;
  gpio_init(IBUS_AVG_ADC_PORT, &gpio_init_struct);

#if defined VOLT_SENSE || defined WIND_SENSE
  gpio_init_struct.gpio_pins = BEMF_A_ADC_GPIO_PIN;
  gpio_init(BEMF_A_ADC_PORT, &gpio_init_struct);

  gpio_init_struct.gpio_pins = BEMF_B_ADC_GPIO_PIN;
  gpio_init(BEMF_B_ADC_PORT, &gpio_init_struct);

  gpio_init_struct.gpio_pins = BEMF_C_ADC_GPIO_PIN;
  gpio_init(BEMF_C_ADC_PORT, &gpio_init_struct);
#endif

  adc_enable(ADC_CONVERTER, FALSE);

  dma_channel_enable(ADC_ORDINARY_DMA_CHANNEL, FALSE);

  /* dma configuration */
  dma_reset(ADC_ORDINARY_DMA_CHANNEL);
  dma_default_para_init(&dma_init_struct);
  dma_init_struct.buffer_size = ADC_ORDINARY_CH_LEN;
  dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  dma_init_struct.memory_base_addr = (uint32_t)adc_in_tab;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_base_addr = (uint32_t)&(ADC_CONVERTER->odt);
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
  dma_init_struct.loop_mode_enable = TRUE;
  dma_init(ADC_ORDINARY_DMA_CHANNEL, &dma_init_struct);

  /* config flexible dma for adc ordinary */
  dma_flexible_config(ADC_ORDINARY_DMA, ADC_ORDINARY_DMA_FLEX_CH, ADC_ORDINARY_DMA_FLEX);

  /* enable dma channel of adc */
  dma_channel_enable(ADC_ORDINARY_DMA_CHANNEL, TRUE);

  adc_base_default_para_init(&adc_base_struct);

  adc_base_struct.sequence_mode = TRUE;
  adc_base_struct.repeat_mode = FALSE;
  adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
  adc_base_struct.ordinary_channel_length = ADC_ORDINARY_CH_LEN;
  adc_base_config(ADC_CONVERTER, &adc_base_struct);
  adc_resolution_set(ADC_CONVERTER, ADC_RESOLUTION_12B);

  /* ordinary channel configuration */
#if defined VOLT_SENSE || defined WIND_SENSE
  adc_ordinary_channel_set(ADC_CONVERTER, BEMF_A_ADC_CH, ADC_BEMF_A_IDX+1, ADC_SAMPLETIME_6_5);
  adc_ordinary_channel_set(ADC_CONVERTER, BEMF_B_ADC_CH, ADC_BEMF_B_IDX+1, ADC_SAMPLETIME_6_5);
  adc_ordinary_channel_set(ADC_CONVERTER, BEMF_C_ADC_CH, ADC_BEMF_C_IDX+1, ADC_SAMPLETIME_6_5);
#endif
  adc_ordinary_channel_set(ADC_CONVERTER, VOLT_BUS_ADC_CH, ADC_BUS_VOLT_IDX+1, ADC_SAMPLETIME_6_5);
  adc_ordinary_channel_set(ADC_CONVERTER, MOS_TEMP_ADC_CH, ADC_MOS_TEMP_IDX+1, ADC_SAMPLETIME_6_5);
  adc_ordinary_channel_set(ADC_CONVERTER, POTENTIO_ADC_CH, ADC_POTENTIO_IDX+1, ADC_SAMPLETIME_6_5);
  adc_ordinary_channel_set(ADC_CONVERTER, IBUS_AVG_ADC_CH, ADC_IBUS_AVE_IDX+1, ADC_SAMPLETIME_6_5);

  adc_ordinary_conversion_trigger_set(ADC_CONVERTER, ADC_ORDINARY_TRIG_TMR1CH1, ADC_ORDINARY_TRIG_EDGE_NONE);
  adc_dma_mode_enable(ADC_CONVERTER, TRUE);

  /* config dma request repeat,it's not useful when common dma mode is use */
  adc_dma_request_repeat_enable(ADC_CONVERTER, TRUE);

  /* ADC enable and calibration */
  if(ADC_CONVERTER->ctrl2_bit.adcen != TRUE)
  {
    adc_enable(ADC_CONVERTER, TRUE);
    while(adc_flag_get(ADC_CONVERTER, ADC_RDY_FLAG) == RESET);
    adc_calibration_init(ADC_CONVERTER);
    while(adc_calibration_init_status_get(ADC_CONVERTER));
    adc_calibration_start(ADC_CONVERTER);
    while(adc_calibration_status_get(ADC_CONVERTER));
  }
}

/**
  * @brief  initialization of adc preempt sampling for phase/bus current sensing
  * @param  none
  * @retval none
  */
void adc_preempt_config(void)
{
  gpio_init_type gpio_init_struct = {0};
  adc_common_config_type adc_common_struct;
  adc_base_config_type adc_base_struct;

  adc_enable(ADC_CONVERTER, FALSE);

  /* adc clock configuration */
  crm_periph_clock_enable(ADC_CONVERTER_CRM_CLK, TRUE);
#if defined TWO_ADC_CONVERTERS
  crm_periph_clock_enable(ADC_SIMULTANE_CONVERTER_CRM_CLK, TRUE);
#endif

  adc_common_default_para_init(&adc_common_struct);
  /* config combine mode */
  adc_common_struct.combine_mode = ADC_INDEPENDENT_MODE;
  /* config division,adcclk is division by hclk */
  adc_common_struct.div = ADC_CONVERTER_CRM_CLK_DIV;        /* HCLK Max. CLK = 288M Hz, ADC_CLK = 288/4 = 72M Hz */
  /* config common dma mode,it's not useful in independent mode */
  adc_common_struct.common_dma_mode = ADC_COMMON_DMAMODE_DISABLE;
  /* config common dma request repeat */
  adc_common_struct.common_dma_request_repeat_state = FALSE;
  /* config adjacent adc sampling interval,it's useful for ordinary shifting mode */
  adc_common_struct.sampling_interval = ADC_SAMPLING_INTERVAL_5CYCLES;
  /* config inner temperature sensor and vintrv */
  adc_common_struct.tempervintrv_state = FALSE;
  /* config voltage battery */
  adc_common_struct.vbat_state = FALSE;
  adc_common_config(&adc_common_struct);

  /* gpio clock configuration */
#if defined THREE_SHUNT
  crm_periph_clock_enable(CURR_PHASE_A_ADC_GPIO_CRM_CLK, TRUE);
  crm_periph_clock_enable(CURR_PHASE_B_ADC_GPIO_CRM_CLK, TRUE);
  crm_periph_clock_enable(CURR_PHASE_C_ADC_GPIO_CRM_CLK, TRUE);
#elif defined TWO_SHUNT
#if defined U_V_SHUNT
  crm_periph_clock_enable(CURR_PHASE_A_ADC_GPIO_CRM_CLK, TRUE);
  crm_periph_clock_enable(CURR_PHASE_B_ADC_GPIO_CRM_CLK, TRUE);
#elif defined V_W_SHUNT
  crm_periph_clock_enable(CURR_PHASE_B_ADC_GPIO_CRM_CLK, TRUE);
  crm_periph_clock_enable(CURR_PHASE_C_ADC_GPIO_CRM_CLK, TRUE);
#elif defined U_W_SHUNT
  crm_periph_clock_enable(CURR_PHASE_A_ADC_GPIO_CRM_CLK, TRUE);
  crm_periph_clock_enable(CURR_PHASE_C_ADC_GPIO_CRM_CLK, TRUE);
#endif
#elif defined ONE_SHUNT
  crm_periph_clock_enable(CURR_BUS_ADC_GPIO_CRM_CLK, TRUE);
#endif

  /* gpio configuration */
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;

#if defined THREE_SHUNT
  gpio_init_struct.gpio_pins = CURR_PHASE_A_ADC_GPIO_PIN;
  gpio_init(CURR_PHASE_A_ADC_PORT, &gpio_init_struct);
  gpio_init_struct.gpio_pins = CURR_PHASE_B_ADC_GPIO_PIN;
  gpio_init(CURR_PHASE_B_ADC_PORT, &gpio_init_struct);
  gpio_init_struct.gpio_pins = CURR_PHASE_C_ADC_GPIO_PIN;
  gpio_init(CURR_PHASE_C_ADC_PORT, &gpio_init_struct);
#elif defined TWO_SHUNT
#if defined U_V_SHUNT
  gpio_init_struct.gpio_pins = CURR_PHASE_A_ADC_GPIO_PIN;
  gpio_init(CURR_PHASE_A_ADC_PORT, &gpio_init_struct);
  gpio_init_struct.gpio_pins = CURR_PHASE_B_ADC_GPIO_PIN;
  gpio_init(CURR_PHASE_B_ADC_PORT, &gpio_init_struct);
#elif defined V_W_SHUNT
  gpio_init_struct.gpio_pins = CURR_PHASE_B_ADC_GPIO_PIN;
  gpio_init(CURR_PHASE_B_ADC_PORT, &gpio_init_struct);
  gpio_init_struct.gpio_pins = CURR_PHASE_C_ADC_GPIO_PIN;
  gpio_init(CURR_PHASE_C_ADC_PORT, &gpio_init_struct);
#elif defined U_W_SHUNT
  gpio_init_struct.gpio_pins = CURR_PHASE_A_ADC_GPIO_PIN;
  gpio_init(CURR_PHASE_A_ADC_PORT, &gpio_init_struct);
  gpio_init_struct.gpio_pins = CURR_PHASE_C_ADC_GPIO_PIN;
  gpio_init(CURR_PHASE_C_ADC_PORT, &gpio_init_struct);
#endif
#elif defined ONE_SHUNT
  gpio_init_struct.gpio_pins = CURR_BUS_ADC_GPIO_PIN;
  gpio_init(CURR_BUS_ADC_PORT, &gpio_init_struct);
#endif

  adc_base_default_para_init(&adc_base_struct);

  adc_base_struct.sequence_mode = TRUE;
  adc_base_struct.repeat_mode = FALSE;
  adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
  adc_base_struct.ordinary_channel_length = ADC_ORDINARY_CH_LEN;
  adc_base_config(ADC_CONVERTER, &adc_base_struct);
  adc_resolution_set(ADC_CONVERTER, ADC_RESOLUTION_12B);

  /* preempt channel configuration */
#if defined TWO_ADC_CONVERTERS
  adc_preempt_channel_length_set(ADC_CONVERTER, 1);

#if defined TWO_SHUNT
  adc_preempt_channel_length_set(ADC_SIMULTANE_CONVERTER, 1);

#if defined U_V_SHUNT
  adc_preempt_channel_set(ADC_CONVERTER, CURR_PHASE_A_ADC_CH, 1, ADC_PREEMPT_SAMPLETIME);
  adc_preempt_channel_set(ADC_SIMULTANE_CONVERTER, CURR_PHASE_B_ADC_CH, 1, ADC_PREEMPT_SAMPLETIME);
#elif defined V_W_SHUNT
  adc_preempt_channel_set(ADC_CONVERTER, CURR_PHASE_B_ADC_CH, 1, ADC_PREEMPT_SAMPLETIME);
  adc_preempt_channel_set(ADC_SIMULTANE_CONVERTER, CURR_PHASE_C_ADC_CH, 1, ADC_PREEMPT_SAMPLETIME);
#elif defined U_W_SHUNT
  adc_preempt_channel_set(ADC_CONVERTER, CURR_PHASE_A_ADC_CH, 1, ADC_PREEMPT_SAMPLETIME);
  adc_preempt_channel_set(ADC_SIMULTANE_CONVERTER, CURR_PHASE_C_ADC_CH, 1, ADC_PREEMPT_SAMPLETIME);
#endif

#else
  adc_preempt_channel_set(ADC_CONVERTER, CURR_PHASE_A_ADC_CH, 1, ADC_PREEMPT_SAMPLETIME);
  adc_preempt_channel_length_set(ADC_SIMULTANE_CONVERTER, 2);
  adc_preempt_channel_set(ADC_SIMULTANE_CONVERTER, CURR_PHASE_B_ADC_CH, 1, ADC_PREEMPT_SAMPLETIME);
  adc_preempt_channel_set(ADC_SIMULTANE_CONVERTER, CURR_PHASE_C_ADC_CH, 2, ADC_PREEMPT_SAMPLETIME);
#endif

#else
#if defined THREE_SHUNT
  adc_preempt_channel_length_set(ADC_CONVERTER, 3);
  adc_preempt_channel_set(ADC_CONVERTER, CURR_PHASE_A_ADC_CH, 1, ADC_PREEMPT_SAMPLETIME);
  adc_preempt_channel_set(ADC_CONVERTER, CURR_PHASE_B_ADC_CH, 2, ADC_PREEMPT_SAMPLETIME);
  adc_preempt_channel_set(ADC_CONVERTER, CURR_PHASE_C_ADC_CH, 3, ADC_PREEMPT_SAMPLETIME);
#elif defined TWO_SHUNT
  adc_preempt_channel_length_set(ADC_CONVERTER, 2);
#if defined U_V_SHUNT
  adc_preempt_channel_set(ADC_CONVERTER, CURR_PHASE_A_ADC_CH, 1, ADC_PREEMPT_SAMPLETIME);
  adc_preempt_channel_set(ADC_CONVERTER, CURR_PHASE_B_ADC_CH, 2, ADC_PREEMPT_SAMPLETIME);
#elif defined V_W_SHUNT
  adc_preempt_channel_set(ADC_CONVERTER, CURR_PHASE_B_ADC_CH, 1, ADC_PREEMPT_SAMPLETIME);
  adc_preempt_channel_set(ADC_CONVERTER, CURR_PHASE_C_ADC_CH, 2, ADC_PREEMPT_SAMPLETIME);
#elif defined U_W_SHUNT
  adc_preempt_channel_set(ADC_CONVERTER, CURR_PHASE_A_ADC_CH, 1, ADC_PREEMPT_SAMPLETIME);
  adc_preempt_channel_set(ADC_CONVERTER, CURR_PHASE_C_ADC_CH, 2, ADC_PREEMPT_SAMPLETIME);
#endif

#elif defined ONE_SHUNT
  adc_preempt_channel_length_set(ADC_CONVERTER, 2);
  adc_preempt_channel_set(ADC_CONVERTER, CURR_BUS_ADC_CH, 1, ADC_PREEMPT_SAMPLETIME);
  adc_preempt_channel_set(ADC_CONVERTER, CURR_BUS_ADC_CH, 2, ADC_PREEMPT_SAMPLETIME);
  adc_preempt_part_mode_enable(ADC_CONVERTER, TRUE);
#endif

#endif

  /* adc prempt trigger source */
  adc_preempt_conversion_trigger_set(ADC_CONVERTER, TMR_ADC_TRIG_SOURCE, TMR_ADC_TRIG_SIGNAL);

#ifndef MOS_RDS_SHUNT
  /* adc voltage monitoring for over current dectector  */
  adc_voltage_monitor_enable(ADC_CONVERTER, ADC_VMONITOR_ALL_PREEMPT);
  adc_voltage_monitor_threshold_value_set(ADC_CONVERTER, OVERCURRENT_THRESHOLD_d, UNDERCURRENT_THRESHOLD_d);
  adc_interrupt_enable(ADC_CONVERTER, ADC_VMOR_INT, TRUE);
#endif

  /* ADC enable and calibration */
  if(ADC_CONVERTER->ctrl2_bit.adcen != TRUE)
  {
    adc_enable(ADC_CONVERTER, TRUE);
    while(adc_flag_get(ADC_CONVERTER, ADC_RDY_FLAG) == RESET);
    adc_calibration_init(ADC_CONVERTER);
    while(adc_calibration_init_status_get(ADC_CONVERTER));
    adc_calibration_start(ADC_CONVERTER);
    while(adc_calibration_status_get(ADC_CONVERTER));
  }

#if defined TWO_ADC_CONVERTERS
  /* select indepenent mode */
  adc_base_struct.sequence_mode = TRUE;
  adc_base_struct.repeat_mode = FALSE;
  adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
  adc_base_struct.ordinary_channel_length = 0;
  adc_base_config(ADC_SIMULTANE_CONVERTER, &adc_base_struct);

  /* adc prempt trigger source */
  adc_preempt_conversion_trigger_set(ADC_SIMULTANE_CONVERTER, TMR_ADC_TRIG_SOURCE, TMR_ADC_TRIG_SIGNAL);

#ifndef MOS_RDS_SHUNT
  adc_voltage_monitor_enable(ADC_SIMULTANE_CONVERTER, ADC_VMONITOR_ALL_PREEMPT);
  adc_voltage_monitor_threshold_value_set(ADC_SIMULTANE_CONVERTER, OVERCURRENT_THRESHOLD_d, UNDERCURRENT_THRESHOLD_d);
  adc_interrupt_enable(ADC_SIMULTANE_CONVERTER, ADC_VMOR_INT, TRUE);
#endif

  if(ADC_SIMULTANE_CONVERTER->ctrl2_bit.adcen != TRUE)
  {
    adc_enable(ADC_SIMULTANE_CONVERTER, TRUE);
    adc_calibration_init(ADC_SIMULTANE_CONVERTER);
    while(adc_calibration_init_status_get(ADC_SIMULTANE_CONVERTER));
    adc_calibration_start(ADC_SIMULTANE_CONVERTER);
    while(adc_calibration_status_get(ADC_SIMULTANE_CONVERTER));
  }
#endif
}

/**
  * @brief  initialization of a timer for incremental encoder
  * @param  none
  * @retval none
  */
void encoder_timer_init(void)
{
  gpio_init_type gpio_init_struct = {0};

  crm_periph_clock_enable(ENCODER_MODE_CRM_CLK, TRUE);
  crm_periph_clock_enable(ENCODER_A_GPIO_CRM_CLK, TRUE);
  crm_periph_clock_enable(ENCODER_B_GPIO_CRM_CLK, TRUE);

  /* remap encoder timer input pins */
  gpio_pin_mux_config(ENCODER_A_PORT, ENCODER_A_GPIO_PIN_SOURCE, ENCODER_A_IOMUX);
  gpio_pin_mux_config(ENCODER_B_PORT, ENCODER_B_GPIO_PIN_SOURCE, ENCODER_B_IOMUX);

  /* timer encoder pin Configuration */
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_pins = ENCODER_A_GPIO_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(ENCODER_A_PORT, &gpio_init_struct);

  gpio_init_struct.gpio_pins = ENCODER_B_GPIO_PIN;
  gpio_init(ENCODER_B_PORT, &gpio_init_struct);

#if defined ABZ || defined MAGNET_ENCODER_W_ABZ
  exint_init_type exint_init_struct;
  /* Index pin configuration */
  crm_periph_clock_enable(ENCODER_Z_EXINT_CRM_CLK, TRUE);
  crm_periph_clock_enable(ENCODER_Z_GPIO_CRM_CLK, TRUE);
  scfg_exint_line_config(ENCODER_Z_PORT_SOURCE, ENCODER_Z_GPIO_PIN_SOURCE);

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_pins = ENCODER_Z_GPIO_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(ENCODER_Z_PORT, &gpio_init_struct);

  exint_default_para_init(&exint_init_struct);
  exint_init_struct.line_enable = TRUE;
  exint_init_struct.line_mode = EXINT_LINE_INTERRUPT;
  exint_init_struct.line_select = ENCODER_Z_EXINT_LINE;
  exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
  exint_init(&exint_init_struct);
#endif
  /* timer encoder interface configuration */

  tmr_base_init(ENCODER_MODE_TIMER, (ENC_CPR_NBR-1), 0);
  tmr_cnt_dir_set(ENCODER_MODE_TIMER, TMR_COUNT_UP);

  /* config encoder mode */
#if defined REVERSE_ENCODER_COUNT
  tmr_encoder_mode_config(ENCODER_MODE_TIMER, TMR_ENCODER_MODE_C, TMR_INPUT_RISING_EDGE, TMR_INPUT_FALLING_EDGE);
#else
  tmr_encoder_mode_config(ENCODER_MODE_TIMER, TMR_ENCODER_MODE_C, TMR_INPUT_RISING_EDGE, TMR_INPUT_RISING_EDGE);
#endif

  /* enable tmr */
  tmr_counter_enable(ENCODER_MODE_TIMER, TRUE);
}

/**
  * @brief  initialization of a timer for capturing incremental encoder
  * @param  none
  * @retval none
  */
#ifdef AT_MOTOR_EVB_V2
void encoder_capture_timer_init(void)
{
  gpio_init_type gpio_init_struct = {0};
  tmr_input_config_type tmr_input_config_struct;

  crm_periph_clock_enable(ENCODER_CAPTURE_CRM_CLK, TRUE);
  crm_periph_clock_enable(ENC_A_GPIO_CRM_CLK, TRUE);
  crm_periph_clock_enable(ENC_B_GPIO_CRM_CLK, TRUE);

  /* timer encoder capture pin Configuration */
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_pins = ENC_A_GPIO_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(ENC_A_PORT, &gpio_init_struct);

  gpio_init_struct.gpio_pins = ENC_B_GPIO_PIN;
  gpio_init(ENC_B_PORT, &gpio_init_struct);

  /* remap encoder capture timer input pins */
  gpio_pin_mux_config(ENC_A_PORT, ENC_A_GPIO_PIN_SOURCE, ENC_A_IOMUX);
  gpio_pin_mux_config(ENC_B_PORT, ENC_B_GPIO_PIN_SOURCE, ENC_B_IOMUX);

  /* timer encoder capture interface configuration */
  tmr_base_init(ENCODER_CAPTURE_TIMER, MAX_CAP_COUNT, (ENC_TIM_CAP_CLK_DIV - 1));
  tmr_cnt_dir_set(ENCODER_CAPTURE_TIMER, TMR_COUNT_UP);

  /* config ti1 trc as input source */
  tmr_clock_source_div_set(ENCODER_CAPTURE_TIMER, ENCODER_CAPTURE_FILTER_CLK_DIV);
  tmr_input_default_para_init(&tmr_input_config_struct);
  tmr_input_config_struct.input_channel_select = TMR_SELECT_CHANNEL_1;
  tmr_input_config_struct.input_mapped_select = TMR_CC_CHANNEL_MAPPED_STI;
  tmr_input_config_struct.input_polarity_select = TMR_INPUT_RISING_EDGE;
  tmr_input_config_struct.input_filter_value = TMR_ENCODER_IN_FILTER;
  tmr_input_channel_init(ENCODER_CAPTURE_TIMER, &tmr_input_config_struct, TMR_CHANNEL_INPUT_DIV_1);

  /* xor funtion enable */
  tmr_channel1_input_select(ENCODER_CAPTURE_TIMER, TMR_CHANEL1_2_3_CONNECTED_C1IRAW_XOR);

  /* select the tmr input trigger: C1INC */
  tmr_trigger_input_select(ENCODER_CAPTURE_TIMER, TMR_SUB_INPUT_SEL_C1INC);

  /* select the slave mode: reset mode */
  tmr_sub_mode_select(ENCODER_CAPTURE_TIMER, TMR_SUB_RESET_MODE);

  /* clear interrupt flag */
  tmr_flag_clear(ENCODER_CAPTURE_TIMER, TMR_OVF_FLAG | TMR_TRIGGER_FLAG | TMR_C1_FLAG);

  /* trigger and overflow interrupt enable */
  tmr_interrupt_enable(ENCODER_CAPTURE_TIMER, TMR_TRIGGER_INT | TMR_OVF_INT, TRUE);

  /* enable encoder capture timer */
  tmr_counter_enable(ENCODER_CAPTURE_TIMER, TRUE);
}
#endif

/**
  * @brief  initialization of a timer for reading magnetic encoder
  * @param  none
  * @retval none
  */
#if defined MAGNET_ENCODER_WO_ABZ
void magnetic_encoder_timer_init(void)
{
  tmr_output_config_type tmr_output_struct;

  crm_periph_clock_enable(SYNC_TIMER_CRM_CLK, TRUE);
  tmr_base_init(SYNC_TIMER, DOUBLE_PWM_PERIOD, 0);
  tmr_cnt_dir_set(SYNC_TIMER, TMR_COUNT_UP);
  tmr_clock_source_div_set(SYNC_TIMER, DEADTIME_CLK_SFT_BITS);

  tmr_output_default_para_init(&tmr_output_struct);
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
  tmr_output_struct.oc_output_state = FALSE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.oc_idle_state = FALSE;
  tmr_output_channel_config(SYNC_TIMER, SYNC_TIMER_SELECT_CHANNEL, &tmr_output_struct);
  tmr_channel_value_set(SYNC_TIMER, SYNC_TIMER_SELECT_CHANNEL, (DOUBLE_PWM_PERIOD - MAG_ENCODER_SAMPLE_POS));

  /* disable single pulse mode */
  tmr_one_cycle_mode_enable(SYNC_TIMER, FALSE);

  /* primary mode selection: SYNC_TIMER */
  tmr_sub_sync_mode_set(SYNC_TIMER, TRUE);
  tmr_primary_mode_select(SYNC_TIMER, TMR_PRIMARY_SEL_OVERFLOW);

  /* subordinate mode selection: PWM_ADVANCE_TIMER */
  tmr_sub_mode_select(PWM_ADVANCE_TIMER, TMR_SUB_RESET_MODE);
  tmr_trigger_input_select(PWM_ADVANCE_TIMER, SYNC_TIMER_INT_SOURCE);

  tmr_flag_clear(SYNC_TIMER, TMR_OVF_FLAG | SYNC_TIMER_CH_INT);
  tmr_interrupt_enable(SYNC_TIMER, SYNC_TIMER_CH_INT, TRUE);
}
#endif

/**
  * @brief  initialization of a timer for capturing hall signals
  * @param  none
  * @retval none
  */
void hall_timer_init(void)
{
  gpio_init_type gpio_init_struct = {0};
  tmr_input_config_type tmr_input_config_struct;

  crm_periph_clock_enable(HALL_CAPTURE_CRM_CLK, TRUE);
  crm_periph_clock_enable(HALL_A_GPIO_CRM_CLK, TRUE);
  crm_periph_clock_enable(HALL_B_GPIO_CRM_CLK, TRUE);
  crm_periph_clock_enable(HALL_C_GPIO_CRM_CLK, TRUE);

  /* timer hall sensor pin Configuration */
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_pins = HALL_A_GPIO_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(HALL_A_PORT, &gpio_init_struct);

  gpio_init_struct.gpio_pins = HALL_B_GPIO_PIN;
  gpio_init(HALL_B_PORT, &gpio_init_struct);

  gpio_init_struct.gpio_pins = HALL_C_GPIO_PIN;
  gpio_init(HALL_C_PORT, &gpio_init_struct);

  /* remap hall timer input pins */
  gpio_pin_mux_config(HALL_A_PORT, HALL_A_GPIO_PIN_SOURCE, HALL_A_IOMUX);
  gpio_pin_mux_config(HALL_B_PORT, HALL_B_GPIO_PIN_SOURCE, HALL_B_IOMUX);
  gpio_pin_mux_config(HALL_C_PORT, HALL_C_GPIO_PIN_SOURCE, HALL_C_IOMUX);

  /* timer hall interface configuration */
  tmr_base_init(HALL_CAPTURE_TIMER, MAX_CAP_COUNT, (TIM_CAP_CLK_DIV - 1));
  tmr_cnt_dir_set(HALL_CAPTURE_TIMER, TMR_COUNT_UP);

  /* config ti1 trc as input source */
  tmr_clock_source_div_set(HALL_CAPTURE_TIMER, HALL_CAPTURE_FILTER_CLK_DIV);
  tmr_input_default_para_init(&tmr_input_config_struct);
  tmr_input_config_struct.input_channel_select = TMR_SELECT_CHANNEL_1;
  tmr_input_config_struct.input_mapped_select = TMR_CC_CHANNEL_MAPPED_STI;
  tmr_input_config_struct.input_polarity_select = TMR_INPUT_RISING_EDGE;
  tmr_input_config_struct.input_filter_value = TMR_HALL_IN_FILTER;
  tmr_input_channel_init(HALL_CAPTURE_TIMER, &tmr_input_config_struct, TMR_CHANNEL_INPUT_DIV_1);

  /* xor funtion enable */
  tmr_channel1_input_select(HALL_CAPTURE_TIMER, TMR_CHANEL1_2_3_CONNECTED_C1IRAW_XOR);

  /* select the tmr input trigger: C1INC */
  tmr_trigger_input_select(HALL_CAPTURE_TIMER, TMR_SUB_INPUT_SEL_C1INC);

  /* select the slave mode: reset mode */
  tmr_sub_mode_select(HALL_CAPTURE_TIMER, TMR_SUB_RESET_MODE);

  /* clear interrupt flag of hall timer */
  tmr_flag_clear(HALL_CAPTURE_TIMER, TMR_TRIGGER_FLAG | TMR_OVF_FLAG | TMR_C4_FLAG);

  /* enable overflow flag of hall timer */
  tmr_interrupt_enable(HALL_CAPTURE_TIMER, TMR_TRIGGER_INT | TMR_C4_INT, TRUE);

  /* enable hall timer */
  tmr_counter_enable(HALL_CAPTURE_TIMER, TRUE);
}

/**
  * @brief  initialization of adc and timer for init. angle detector
  * @param  none
  * @retval none
  */
void foc_angle_init_config(void)
{
  adc_enable(ADC_CONVERTER, FALSE);
  dma_channel_enable(ADC_ORDINARY_DMA_CHANNEL, FALSE);

#if defined TWO_SHUNT || defined ONE_SHUNT
  gpio_init_type gpio_init_struct = {0};

  /* gpio clock configuration */
  crm_periph_clock_enable(CURR_BUS_ADC_GPIO_CRM_CLK, TRUE);

  /* gpio configuration */
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;

  gpio_init_struct.gpio_pins = CURR_BUS_ADC_GPIO_PIN;
  gpio_init(CURR_BUS_ADC_PORT, &gpio_init_struct);
#endif

#if defined THREE_SHUNT
  /* preempt channel configuration */
  adc_preempt_channel_length_set(ADC_CONVERTER, 3);
  adc_preempt_channel_set(ADC_CONVERTER, CURR_PHASE_A_ADC_CH, 1, ADC_PREEMPT_SAMPLETIME);
  adc_preempt_channel_set(ADC_CONVERTER, CURR_PHASE_B_ADC_CH, 2, ADC_PREEMPT_SAMPLETIME);
  adc_preempt_channel_set(ADC_CONVERTER, CURR_PHASE_C_ADC_CH, 3, ADC_PREEMPT_SAMPLETIME);
#elif defined TWO_SHUNT || defined ONE_SHUNT
  /* preempt channel configuration */
  adc_preempt_channel_length_set(ADC_CONVERTER, 1);
  adc_preempt_channel_set(ADC_CONVERTER, CURR_BUS_ADC_CH, 1, ADC_PREEMPT_SAMPLETIME);
#endif

  /* adc prempt trigger source */
  adc_preempt_conversion_trigger_set(ADC_CONVERTER, TMR_ADC_TRIG_SOURCE, TMR_ADC_TRIG_SIGNAL);

  /* ADC enable and calibration */
  if(ADC_CONVERTER->ctrl2_bit.adcen != TRUE)
  {
    adc_enable(ADC_CONVERTER, TRUE);
    while(adc_flag_get(ADC_CONVERTER, ADC_RDY_FLAG) == RESET);
    adc_calibration_init(ADC_CONVERTER);
    while(adc_calibration_init_status_get(ADC_CONVERTER));
    adc_calibration_start(ADC_CONVERTER);
    while(adc_calibration_status_get(ADC_CONVERTER));
  }

  tmr_output_config_type tmr_output_struct;

  /* disable pwm timer */
  tmr_counter_enable(PWM_ADVANCE_TIMER, FALSE);
  /* set single pulse mode */
  tmr_one_cycle_mode_enable(PWM_ADVANCE_TIMER, TRUE);
  /* set tmr peried */
  tmr_base_init(PWM_ADVANCE_TIMER, (ANGLE_INIT_PERIOD-1), (ANGLE_INIT_DIV-1));
  tmr_repetition_counter_set(PWM_ADVANCE_TIMER, 0);
  tmr_cnt_dir_set(PWM_ADVANCE_TIMER, TMR_COUNT_UP);
  tmr_clock_source_div_set(PWM_ADVANCE_TIMER, DEADTIME_CLK_SFT_BITS);
  /* disable tmr buffer */
  tmr_channel_buffer_enable(PWM_ADVANCE_TIMER, FALSE);

  tmr_output_channel_buffer_enable(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_1, FALSE);
  tmr_output_channel_buffer_enable(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_2, FALSE);
  tmr_output_channel_buffer_enable(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_3, FALSE);
  tmr_output_channel_buffer_enable(angle_detector.ADC_TMRx, angle_detector.ADC_TMRx_CHx, FALSE);

  /* set pwm compare value */
  tmr_channel_value_set(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_1, angle_detector.detect_duty);
  tmr_channel_value_set(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_2, angle_detector.detect_duty);
  tmr_channel_value_set(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_3, angle_detector.detect_duty);
  tmr_channel_value_set(angle_detector.ADC_TMRx, angle_detector.ADC_TMRx_CHx, *angle_detector.adc_trig);

  /* output compare toggle mode configuration: channel1 */
  tmr_output_default_para_init(&tmr_output_struct);
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
  tmr_output_struct.oc_idle_state = FALSE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.oc_output_state = TRUE;
  tmr_output_struct.occ_output_state = TRUE;
#if defined GATE_DRIVER_LOW_SIDE_INVERT
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_LOW;
  tmr_output_struct.occ_idle_state = TRUE;
#else
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.occ_idle_state = FALSE;
#endif

  tmr_output_channel_config(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_1, &tmr_output_struct);
  tmr_output_channel_config(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_2, &tmr_output_struct);
  tmr_output_channel_config(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_3, &tmr_output_struct);
  tmr_output_struct.occ_output_state = FALSE;
  tmr_output_channel_config(angle_detector.ADC_TMRx, angle_detector.ADC_TMRx_CHx, &tmr_output_struct);

  PWM_ADVANCE_TIMER->brk_bit.dtc = 0;  /* set deadtime = 0 */

  /* clear tmr counter */
  tmr_counter_value_set(PWM_ADVANCE_TIMER, 0);
  tmr_counter_enable(PWM_ADVANCE_TIMER, TRUE);
  angle_detector.step_count = 1;
}

/**
  * @brief  initialization of a timer for a speed controller
  * @param  none
  * @retval none
  */
void speed_timer_init(void)
{
  /* enable speed timer clock */
  crm_periph_clock_enable(SPEED_LOOP_TIMER_CRM_CLK, TRUE);

  /* time base configuration */
  tmr_base_init(SPEED_LOOP_TIMER, (SPD_LOOP_TMR_PR-1), SPD_LOOP_TMR_DIV);
  tmr_cnt_dir_set(SPEED_LOOP_TIMER, TMR_COUNT_UP);

  /* overflow interrupt enable */
  tmr_interrupt_enable(SPEED_LOOP_TIMER, TMR_OVF_INT, TRUE);

  /* enable tmr */
  tmr_counter_enable(SPEED_LOOP_TIMER, TRUE);
}

/**
  * @brief  initialization of a timer for braking
  * @param  none
  * @retval none
  */
void brake_pwm_init(void)
{
  gpio_init_type gpio_init_struct = {0};
  tmr_output_config_type tmr_oc_init_structure;

  crm_periph_clock_enable(BRAKE_PWM_TIMER_CRM_CLK, TRUE);
  crm_periph_clock_enable(BRAKE_PWM_GPIO_CRM_CLK, TRUE);

  /* timer hall sensor pin Configuration */
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_pins = BRAKE_PWM_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(BRAKE_PWM_PORT, &gpio_init_struct);

  /* remap brake pwm timer pins */
  gpio_pin_mux_config(BRAKE_PWM_PORT, BRAKE_PWM_PIN_SOURCE, BRAKE_PWM_IOMUX);

  /* tmr base configuration */
  tmr_base_init(BRAKE_PWM_TIMER, (BRAKE_PWM_PERIOD-1), (BRAKE_PWM_DIV-1));
  tmr_cnt_dir_set(BRAKE_PWM_TIMER, TMR_COUNT_UP);
  tmr_clock_source_div_set(BRAKE_PWM_TIMER, TMR_CLOCK_DIV1);

  tmr_output_channel_buffer_enable(BRAKE_PWM_TIMER, BRAKE_PWM_TIMER_CH, FALSE);

  /* output compare toggle mode configuration: channel1 */
  tmr_channel_value_set(BRAKE_PWM_TIMER, BRAKE_PWM_TIMER_CH, BRAKE_PWM_VALUE);
  tmr_output_default_para_init(&tmr_oc_init_structure);
  tmr_oc_init_structure.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_oc_init_structure.oc_idle_state = FALSE;
  tmr_oc_init_structure.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_oc_init_structure.oc_output_state = TRUE;
  tmr_output_channel_config(BRAKE_PWM_TIMER, BRAKE_PWM_TIMER_CH, &tmr_oc_init_structure);

  /* disable brake tmr channel */
  tmr_channel_enable(BRAKE_PWM_TIMER, BRAKE_PWM_TIMER_CH, FALSE);

  /* disable one cycle mode */
  tmr_one_cycle_mode_enable(BRAKE_PWM_TIMER, FALSE);

  /* enable brake timer */
  tmr_counter_enable(BRAKE_PWM_TIMER, TRUE);
}

/* support printf function, usemicrolib is unnecessary */
#if (__ARMCC_VERSION > 6000000)
__asm (".global __use_no_semihosting\n\t");
void _sys_exit(int x)
{
  x = x;
}
/* __use_no_semihosting was requested, but _ttywrch was */
void _ttywrch(int ch)
{
  ch = ch;
}
FILE __stdout;
#else
#ifdef __CC_ARM
#pragma import(__use_no_semihosting)
struct __FILE
{
  int handle;
};
FILE __stdout;
void _sys_exit(int x)
{
  x = x;
}
/* __use_no_semihosting was requested, but _ttywrch was */
void _ttywrch(int ch)
{
  ch = ch;
}
#endif
#endif

#if defined (__GNUC__) && !defined (__clang__)
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

/**
  * @brief  retargets the c library printf function to the usart.
  * @param  none
  * @retval none
  */
PUTCHAR_PROTOTYPE
{
  while(usart_flag_get(COMM_UART, USART_TDBE_FLAG) == RESET);
  usart_data_transmit(COMM_UART, ch);
  return ch;
}

/**
  * @brief  initialize uart
  * @param  usart_config: usart configuration related variables
  * @retval none
  */
void uart_init(usart_config_type *usart_config)
{
  gpio_init_type gpio_init_struct;

  /* enable the uart and gpio clock */
  crm_periph_clock_enable(usart_config->usart_clock, TRUE);
  crm_periph_clock_enable(usart_config->usart_tx_gpio_clock, TRUE);
  crm_periph_clock_enable(usart_config->usart_rx_gpio_clock, TRUE);

  gpio_default_para_init(&gpio_init_struct);

  /* configure the uart tx pin */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = usart_config->usart_tx_gpio_pin;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(usart_config->usart_tx_gpio_port, &gpio_init_struct);

  /* configure the uart rx pin */
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_OPEN_DRAIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = usart_config->usart_rx_gpio_pin;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init(usart_config->usart_rx_gpio_port, &gpio_init_struct);

  /* remap usart1 tx and rx pins */
  gpio_pin_mux_config(usart_config->usart_tx_gpio_port, usart_config->gpio_tx_pin_source, usart_config->gpio_tx_pin_mux);
  gpio_pin_mux_config(usart_config->usart_rx_gpio_port, usart_config->gpio_rx_pin_source, usart_config->gpio_rx_pin_mux);

  /* configure uart param */
  usart_init(usart_config->usart_x, usart_config->baudrate, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_transmitter_enable(usart_config->usart_x, TRUE);
  usart_receiver_enable(usart_config->usart_x, TRUE);
  usart_dma_transmitter_enable(usart_config->usart_x, TRUE);
  usart_dma_receiver_enable(usart_config->usart_x, TRUE);
  usart_interrupt_enable(usart_config->usart_x,USART_IDLE_INT, TRUE);
  usart_enable(usart_config->usart_x, TRUE);
}

/**
  * @brief  configure button exint
  * @param  none
  * @retval none
  */
void button_exint_init(void)
{
  /* target board has no start/stop or hall-learn buttons */
}

/**
  * @brief  configure led gpio
  * @param  none
  * @retval none
  */
void led_init(void)
{
  gpio_init_type gpio_init_struct;

  /* target board has only LED_ERR (PC14) and LED_RUN (PC13) */
  crm_periph_clock_enable(ERROR_LED_GPIO_CRM_CLK, TRUE);
  crm_periph_clock_enable(STATUS1_LED_GPIO_CRM_CLK, TRUE);

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;

  gpio_init_struct.gpio_pins = ERROR_LED_GPIO_PIN;
  gpio_init(ERROR_LED_PORT, &gpio_init_struct);

  gpio_init_struct.gpio_pins = STATUS1_LED_GPIO_PIN;
  gpio_init(STATUS1_LED_PORT, &gpio_init_struct);
}

/**
  * @brief  turns selected led on.
  * @param  led_ports: to select the led gpio peripheral.
  *         this parameter can be one of the following values:
  *         STATUS1_LED_PORT, STATUS2_LED_PORT, STATUS3_LED_PORT, ERROR_LED_PORT.
  * @param  led_pins: led gpio pin number
  *         this parameter can be any combination of the following:
  *         STATUS1_LED_GPIO_PIN, STATUS2_LED_GPIO_PIN, STATUS3_LED_GPIO_PIN, ERROR_LED_GPIO_PIN.
  * @retval none
  */
void led_on(gpio_type *led_gpio_port, uint16_t led_gpio_pin)
{
  led_gpio_port->clr = led_gpio_pin;
}

/**
  * @brief  turns selected led off.
  * @param  led_ports: to select the led gpio peripheral.
  *         this parameter can be one of the following values:
  *         STATUS1_LED_PORT, STATUS2_LED_PORT, STATUS3_LED_PORT, ERROR_LED_PORT.
  * @param  led_pins: led gpio pin number
  *         this parameter can be any combination of the following:
  *         STATUS1_LED_GPIO_PIN, STATUS2_LED_GPIO_PIN, STATUS3_LED_GPIO_PIN, ERROR_LED_GPIO_PIN.
  * @retval none
  */
void led_off(gpio_type *led_gpio_port, uint16_t led_gpio_pin)
{
  led_gpio_port->scr = led_gpio_pin;
}

/**
  * @brief  turns selected led tooggle.
  * @param  led_ports: to select the led gpio peripheral.
  *         this parameter can be one of the following values:
  *         STATUS1_LED_PORT, STATUS2_LED_PORT, STATUS3_LED_PORT, ERROR_LED_PORT.
  * @param  led_pins: led gpio pin number
  *         this parameter can be any combination of the following:
  *         STATUS1_LED_GPIO_PIN, STATUS2_LED_GPIO_PIN, STATUS3_LED_GPIO_PIN, ERROR_LED_GPIO_PIN.
  * @retval none
  */
void led_toggle(gpio_type *led_gpio_port, uint16_t led_gpio_pin)
{
  led_gpio_port->odt ^= led_gpio_pin;
}

/**
  * @brief  initilize the leds.
  * @param  none
  * @retval none
  */
void led_config(void)
{
  led_init();
  led_off(ERROR_LED_PORT, ERROR_LED_GPIO_PIN);
  led_off(STATUS1_LED_PORT, STATUS1_LED_GPIO_PIN);
}

/**
  * @brief  blink the leds.
  * @param  none
  * @retval none
  */
void led_blink(void)
{
  uint8_t count;
  for(count=0; count<3; count++)
  {
    led_on(STATUS1_LED_PORT, STATUS1_LED_GPIO_PIN);
    mc_delay_ms(500);
    led_off(STATUS1_LED_PORT, STATUS1_LED_GPIO_PIN);
    mc_delay_ms(500);
  }
}

/**
  * @brief  configure mode switch
  * @param  none
  * @retval none
  */
void mode_switch_init(void)
{
  /* target board has no MODE1/MODE2/REVERSE/BRAKE/LOCK/PARKING switches */
}

/**
  * @brief  configure gpio pins
  * @param  none
  * @retval none
  */
void gpio_pins_init(void)
{
  gpio_init_type gpio_init_struct;

  gpio_default_para_init(&gpio_init_struct);

  /* EN_GATE = PC15 for DRV8353, init low (gate driver disabled) */
  crm_periph_clock_enable(EN_GATE_GPIO_CRM_CLK, TRUE);
  gpio_bits_reset(EN_GATE_PORT, EN_GATE_GPIO_PIN);
  gpio_init_struct.gpio_pins = EN_GATE_GPIO_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(EN_GATE_PORT, &gpio_init_struct);
}

/**
  * @brief  initialization of adc and timer for motor parameter ID
  * @param  none
  * @retval none
  */
void motor_parameter_ID_config(void)
{
  tmr_output_config_type tmr_output_struct;

  /* set single pulse mode */
  tmr_one_cycle_mode_enable(PWM_ADVANCE_TIMER, FALSE);
  /* set tmr peried */
  tmr_repetition_counter_set(PWM_ADVANCE_TIMER, 1);               /* the pwm cycle isr in underflow (high-side pwm on) */
  tmr_base_init(PWM_ADVANCE_TIMER, PWM_PERIOD_ID, 0);
  tmr_cnt_dir_set(PWM_ADVANCE_TIMER, TMR_COUNT_TWO_WAY_1);
  /* set dead time clock */
  tmr_clock_source_div_set(PWM_ADVANCE_TIMER, DEADTIME_CLK_SFT_BITS);
  /* disable tmr buffer */
  tmr_channel_buffer_enable(PWM_ADVANCE_TIMER, FALSE);

  tmr_output_channel_buffer_enable(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_1, FALSE);
  tmr_output_channel_buffer_enable(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_2, FALSE);
  tmr_output_channel_buffer_enable(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_3, FALSE);
  tmr_output_channel_buffer_enable(ADC_TIMER, ADC_TIMER_SELECT_CHANNEL, FALSE);

  /* set pwm compare value */
  tmr_channel_value_set(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_1, 0);
  tmr_channel_value_set(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_2, 0);
  tmr_channel_value_set(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_3, 0);
  tmr_channel_value_set(ADC_TIMER, ADC_TIMER_SELECT_CHANNEL, ADC_TRIG_POS_ID);

  /* output compare toggle mode configuration: channel1 */
  tmr_output_default_para_init(&tmr_output_struct);
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_output_struct.oc_output_state = TRUE;
  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.oc_idle_state = FALSE;
  tmr_output_struct.occ_output_state = TRUE;
#if defined GATE_DRIVER_LOW_SIDE_INVERT
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_LOW;
  tmr_output_struct.occ_idle_state = TRUE;
#else
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct.occ_idle_state = FALSE;
#endif

  tmr_output_channel_config(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_1, &tmr_output_struct);
  tmr_output_channel_config(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_2, &tmr_output_struct);
  tmr_output_channel_config(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_3, &tmr_output_struct);
#ifdef ONE_SHUNT
  tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
  dma_channel_enable(TMR_ADC_DMA_CH, FALSE);
  adc_preempt_channel_length_set(ADC_CONVERTER, 1);
  adc_preempt_channel_set(ADC_CONVERTER, CURR_BUS_ADC_CH, 1, ADC_PREEMPT_SAMPLETIME);
#endif
  tmr_output_struct.occ_output_state = FALSE;
  tmr_output_channel_config(ADC_TIMER, ADC_TIMER_SELECT_CHANNEL, &tmr_output_struct);

  /* clear tmr counter */
  tmr_counter_value_set(PWM_ADVANCE_TIMER, 0);

  tmr_channel_enable(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_3, FALSE);
  tmr_channel_enable(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_3C, FALSE);

  /* enable pwm timer */
  tmr_counter_enable(PWM_ADVANCE_TIMER, TRUE);
  /* pwm timer output enable */
  tmr_output_enable(PWM_ADVANCE_TIMER, TRUE);
}

/**
  * @brief  obtain vref calibration ratio
  * @param  none
  * @retval none
  */
void get_int_vref_cal_ratio(void)
{
  adc_common_config_type adc_common_struct;
  int32_t vref_adc;

  /* adc clock configuration */
  crm_periph_clock_enable(ADC_CONVERTER_CRM_CLK, TRUE);

  adc_enable(ADC_CONVERTER, FALSE);

  adc_common_default_para_init(&adc_common_struct);
  /* config combine mode */
  adc_common_struct.combine_mode = ADC_INDEPENDENT_MODE;
  /* config division,adcclk is division by hclk */
  adc_common_struct.div = ADC_CONVERTER_CRM_CLK_DIV;        /* HCLK Max. CLK = 288M Hz, ADC_CLK = 288/4 = 72M Hz */
  /* config inner temperature sensor and vintrv */
  adc_common_struct.tempervintrv_state = TRUE;
  adc_common_config(&adc_common_struct);

  adc_preempt_channel_length_set(ADC_CONVERTER, 1);

  adc_preempt_channel_set(ADC_CONVERTER, ADC_CHANNEL_17, 1, ADC_SAMPLETIME_640_5);      //get internal vref voltage

  /* change adc prempt trigger source */
  adc_preempt_conversion_trigger_set(ADC_CONVERTER, TMR_ADC_TRIG_SOURCE, TMR_ADC_TRIG_NO_SIGNAL);
  /* ADC enable and calibration */
  if(ADC_CONVERTER->ctrl2_bit.adcen != TRUE)
  {
    adc_enable(ADC_CONVERTER, TRUE);
    while(adc_flag_get(ADC_CONVERTER, ADC_RDY_FLAG) == RESET);
    adc_calibration_init(ADC_CONVERTER);
    while(adc_calibration_init_status_get(ADC_CONVERTER));
    adc_calibration_start(ADC_CONVERTER);
    while(adc_calibration_status_get(ADC_CONVERTER));
  }

  adc_flag_clear(ADC_CONVERTER, ADC_PCCE_FLAG);

  adc_preempt_software_trigger_enable(ADC_CONVERTER, TRUE);

  while(adc_flag_get(ADC_CONVERTER, ADC_PCCE_FLAG) == RESET);

  vref_adc = adc_preempt_conversion_data_get(ADC_CONVERTER, ADC_PREEMPT_CHANNEL_1);

  adc_flag_clear(ADC_CONVERTER, ADC_PCCE_FLAG);

  adc_preempt_software_trigger_enable(ADC_CONVERTER, TRUE);

  while(adc_flag_get(ADC_CONVERTER, ADC_PCCE_FLAG) == RESET);

  adc_flag_clear(ADC_CONVERTER, ADC_PCCE_FLAG);

  vref_adc += adc_preempt_conversion_data_get(ADC_CONVERTER, ADC_PREEMPT_CHANNEL_1);

  adc_preempt_config();

  vref_adc >>= 1;

  vref_cal_ratio = (int16_t) ((IDEAL_1V2_ADC_VALUE<<14) / vref_adc);
}