/* add user code begin Header */
/**
  **************************************************************************
  * @file     wk_adc.c
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
#include "wk_adc.h"

/* add user code begin 0 */

/* ============================================================
 * ADC Business Layer Implementation
 * Migrated from cubemx_yxsui/Core/Src/adc.c
 * ============================================================ */

/* FOC电流采样数据（10kHz注入组） */
volatile FOC_CurrentSample_t g_foc_current = {0, 0, 0};

/* ADC偏置零点 */
volatile int32_t g_adc_offset_a = 0;
volatile int32_t g_adc_offset_b = 0;

/* 规则通道数据（1kHz普通组 + DMA） */
volatile uint16_t g_temp_motor_raw = 0;
volatile uint16_t g_temp_mos_raw = 0;
volatile uint16_t g_so_c_raw = 0;
volatile uint16_t g_so_3_raw = 0;
volatile uint32_t g_reg_callback_count = 0;

/* Bus voltage raw value (used by foc/can_wly modules)
 * Migrated from cubemx_yxsui/Core/Src/adc.c */
volatile uint32_t g_vdc_raw = 0;

/* FOC open-loop test enable flag (set 1 to bypass closed-loop in PCCE ISR)
 * Migrated from cubemx_yxsui/Core/Src/adc.c:30 */
volatile uint8_t g_foc_openloop_enable = 0;

/* adc_ordinary_buffer在main.c定义，这里extern引用 */
extern uint16_t adc_ordinary_buffer[4];

/**
 * @brief  ADC注入组转换完成处理（PCCE ISR中调用）
 * @note   读取ADC1 CH0(PA0) + ADC2 CH1(PA1) 注入数据，减零点偏置
 *         使用库函数 adc_preempt_conversion_data_get() 读取pdt寄存器
 */
void adc_foc_on_injected_done(void)
{
    /* AT32注入组数据读取：使用库函数读取preempt channel 1 */
    int32_t raw_a = (int32_t)adc_preempt_conversion_data_get(ADC1, ADC_PREEMPT_CHANNEL_1);
    int32_t raw_b = (int32_t)adc_preempt_conversion_data_get(ADC2, ADC_PREEMPT_CHANNEL_1);

    g_foc_current.i_a_raw = raw_a - g_adc_offset_a;
    g_foc_current.i_b_raw = raw_b - g_adc_offset_b;
    g_foc_current.sample_count++;
}

/**
 * @brief  ADC普通组DMA传输完成处理（DMA FDT或OCCE ISR中调用）
 * @note   从adc_ordinary_buffer[4]读取：
 *         [0]=ADC1_CH4(PA4/TEMP_MOTOR), [1]=ADC1_CH5(PA5/TEMP_MOS)
 *         [2]=ADC2_CH2(PA2/SO_C),       [3]=ADC2_CH3(PA3/VDC)
 */
void adc_foc_on_regular_done(void)
{
    g_temp_motor_raw = adc_ordinary_buffer[0];
    g_temp_mos_raw   = adc_ordinary_buffer[1];
    g_so_c_raw       = adc_ordinary_buffer[2];
    g_vdc_raw        = adc_ordinary_buffer[3];

    /* Update g_udc_volt (unit: V) for SVPWM
     * Formula: V = raw * 3.3 * divider_ratio / 4095
     * divider_ratio = 21 (hardware voltage divider)
     * Simplified: raw * 33 * 21 / 4095 / 10 */
    extern volatile uint16_t g_udc_volt;
    uint16_t udc_v = (uint16_t)(g_vdc_raw * 33U * 21U / 4095U / 10U);
    if (udc_v < 10) udc_v = 10;  /* prevent div-by-zero in SVPWM */
    g_udc_volt = udc_v;

    g_reg_callback_count++;
}

/**
 * @brief  校准电流零点偏置（电机静止，IGBT关闭）
 * @param  n_samples 采样次数，建议1024
 */
void adc_calibrate_offsets(uint16_t n_samples)
{
    int64_t sum_a = 0, sum_b = 0;
    uint32_t start = g_foc_current.sample_count;

    /* 等待采样启动 */
    while ((g_foc_current.sample_count - start) < 1);

    /* 暂时清零偏置 */
    int32_t saved_a = g_adc_offset_a;
    int32_t saved_b = g_adc_offset_b;
    g_adc_offset_a = 0;
    g_adc_offset_b = 0;

    /* 累加n_samples个原始值 */
    start = g_foc_current.sample_count;
    uint32_t count = 0;
    while (count < n_samples) {
        uint32_t now = g_foc_current.sample_count;
        if (now != start) {
            sum_a += g_foc_current.i_a_raw;
            sum_b += g_foc_current.i_b_raw;
            start = now;
            count++;
        }
    }

    /* 计算平均偏置 */
    g_adc_offset_a = (int32_t)(sum_a / n_samples);
    g_adc_offset_b = (int32_t)(sum_b / n_samples);
    (void)saved_a; (void)saved_b;
}

/* add user code end 0 */

/**
  * @brief  init adc-common function.
  * @param  none
  * @retval none
  */
void wk_adc_common_init(void)
{
  /* add user code begin adc_common_init 0 */

  /* add user code end adc_common_init 0 */

  adc_common_config_type adc_common_struct;

  /* add user code begin adc_common_init 1 */

  /* add user code end adc_common_init 1 */

  adc_reset();

  /* adc_common_settings------------------------------------------------------------ */
  adc_common_default_para_init(&adc_common_struct);
  adc_common_struct.combine_mode = ADC_ORDINARY_SMLT_PREEMPT_SMLT_ONESLAVE_MODE;
  adc_common_struct.div = ADC_HCLK_DIV_3;
  adc_common_struct.common_dma_mode = ADC_COMMON_DMAMODE_1;
  adc_common_struct.common_dma_request_repeat_state = TRUE;
  adc_common_struct.sampling_interval = ADC_SAMPLING_INTERVAL_5CYCLES;
  adc_common_struct.tempervintrv_state = FALSE;
  adc_common_struct.vbat_state = FALSE;
  adc_common_config(&adc_common_struct);
  
  /* add user code begin adc_common_init 2 */

  /* add user code end adc_common_init 2 */
}

/**
  * @brief  init adc1 function.
  * @param  none
  * @retval none
  */
void wk_adc1_init(void)
{
  /* add user code begin adc1_init 0 */

  /* add user code end adc1_init 0 */

  gpio_init_type gpio_init_struct;
  adc_base_config_type adc_base_struct;

  gpio_default_para_init(&gpio_init_struct);

  /* add user code begin adc1_init 1 */

  /* add user code end adc1_init 1 */

  /*gpio--------------------------------------------------------------------*/ 
  /* configure the IN0 pin */
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_pins = SOA_PIN;
  gpio_init(SOA_GPIO_PORT, &gpio_init_struct);

  /* configure the IN4 pin */
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_pins = TEMP_MOTOR_PIN;
  gpio_init(TEMP_MOTOR_GPIO_PORT, &gpio_init_struct);

  /* configure the IN5 pin */
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_pins = TEMP_MOS_PIN;
  gpio_init(TEMP_MOS_GPIO_PORT, &gpio_init_struct);

  /* adc_settings------------------------------------------------------------------- */
  adc_base_default_para_init(&adc_base_struct);
  adc_base_struct.sequence_mode = TRUE;
  adc_base_struct.repeat_mode = FALSE;
  adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
  adc_base_struct.ordinary_channel_length = 2;
  adc_base_config(ADC1, &adc_base_struct);

  adc_resolution_set(ADC1, ADC_RESOLUTION_12B);

  /* adc_ordinary_conversionmode---------------------------------------------------- */
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_4, 1, ADC_SAMPLETIME_2_5);
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_5, 2, ADC_SAMPLETIME_2_5);

  /* When "ADC_ORDINARY_TRIG_EDGE_NONE" is selected, the external trigger source is invalid, and user can only use software trigger. \
  The software trigger function is adc_ordinary_software_trigger_enable(ADCx, TRUE); */
  adc_ordinary_conversion_trigger_set(ADC1, ADC_ORDINARY_TRIG_TMR6TRGOUT, ADC_ORDINARY_TRIG_EDGE_RISING);

  /* adc_preempt_conversionmode----------------------------------------------------- */
  adc_preempt_channel_length_set(ADC1, 1);

  adc_preempt_channel_set(ADC1, ADC_CHANNEL_0, 1, ADC_SAMPLETIME_2_5);
  adc_preempt_offset_value_set(ADC1, ADC_PREEMPT_CHANNEL_1, 0x0);

  /* When "ADC_PREEMPT_TRIG_EDGE_NONE" is selected, the external trigger source is invalid, and user can only use software trigger. \
  The software trigger function is adc_preempt_software_trigger_enable(ADCx, TRUE); */
  adc_preempt_conversion_trigger_set(ADC1, ADC_PREEMPT_TRIG_TMR1TRGOUT, ADC_PREEMPT_TRIG_EDGE_RISING);

  /* enable ordinary channels conversion end interrupt */
  adc_interrupt_enable(ADC1, ADC_OCCE_INT, TRUE);

  /* enable preempted channels conversion end interrupt */
  adc_interrupt_enable(ADC1, ADC_PCCE_INT, TRUE);

  /* enable trigger convert fail interrupt */
  adc_interrupt_enable(ADC1, ADC_TCF_INT, TRUE);

  /* add user code begin adc1_init 2 */
  
  /* add user code end adc1_init 2 */

  adc_enable(ADC1, TRUE);
  while(adc_flag_get(ADC1, ADC_RDY_FLAG) == RESET);

  /* adc calibration---------------------------------------------------------------- */
  adc_calibration_init(ADC1);
  while(adc_calibration_init_status_get(ADC1));
  adc_calibration_start(ADC1);
  while(adc_calibration_status_get(ADC1));

  /* add user code begin adc1_init 3 */
  
  /* add user code end adc1_init 3 */
}

/**
  * @brief  init adc2 function.
  * @param  none
  * @retval none
  */
void wk_adc2_init(void)
{
  /* add user code begin adc2_init 0 */

  /* add user code end adc2_init 0 */

  gpio_init_type gpio_init_struct;
  adc_base_config_type adc_base_struct;

  gpio_default_para_init(&gpio_init_struct);

  /* add user code begin adc2_init 1 */

  /* add user code end adc2_init 1 */

  /*gpio--------------------------------------------------------------------*/ 
  /* configure the IN1 pin */
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_pins = SOB_PIN;
  gpio_init(SOB_GPIO_PORT, &gpio_init_struct);

  /* configure the IN2 pin */
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_pins = SOC_PIN;
  gpio_init(SOC_GPIO_PORT, &gpio_init_struct);

  /* configure the IN3 pin */
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_pins = SO3_PIN;
  gpio_init(SO3_GPIO_PORT, &gpio_init_struct);

  /* adc_settings------------------------------------------------------------------- */
  adc_base_default_para_init(&adc_base_struct);
  adc_base_struct.sequence_mode = TRUE;
  adc_base_struct.repeat_mode = FALSE;
  adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
  adc_base_struct.ordinary_channel_length = 2;
  adc_base_config(ADC2, &adc_base_struct);

  adc_resolution_set(ADC2, ADC_RESOLUTION_12B);

  /* adc_ordinary_conversionmode---------------------------------------------------- */
  adc_ordinary_channel_set(ADC2, ADC_CHANNEL_2, 1, ADC_SAMPLETIME_2_5);
  adc_ordinary_channel_set(ADC2, ADC_CHANNEL_3, 2, ADC_SAMPLETIME_2_5);

  /* When "ADC_ORDINARY_TRIG_EDGE_NONE" is selected, the external trigger source is invalid, and user can only use software trigger. \
  The software trigger function is adc_ordinary_software_trigger_enable(ADCx, TRUE); */
  adc_ordinary_conversion_trigger_set(ADC2, ADC_ORDINARY_TRIG_TMR6TRGOUT, ADC_ORDINARY_TRIG_EDGE_NONE);

  /* adc_preempt_conversionmode----------------------------------------------------- */
  adc_preempt_channel_length_set(ADC2, 1);

  adc_preempt_channel_set(ADC2, ADC_CHANNEL_1, 1, ADC_SAMPLETIME_2_5);
  adc_preempt_offset_value_set(ADC2, ADC_PREEMPT_CHANNEL_1, 0x0);

  /* When "ADC_PREEMPT_TRIG_EDGE_NONE" is selected, the external trigger source is invalid, and user can only use software trigger. \
  The software trigger function is adc_preempt_software_trigger_enable(ADCx, TRUE); */
  adc_preempt_conversion_trigger_set(ADC2, ADC_PREEMPT_TRIG_TMR1TRGOUT, ADC_PREEMPT_TRIG_EDGE_NONE);

  /* add user code begin adc2_init 2 */

  /* add user code end adc2_init 2 */

  adc_enable(ADC2, TRUE);
  while(adc_flag_get(ADC2, ADC_RDY_FLAG) == RESET);

  /* adc calibration---------------------------------------------------------------- */
  adc_calibration_init(ADC2);
  while(adc_calibration_init_status_get(ADC2));
  adc_calibration_start(ADC2);
  while(adc_calibration_status_get(ADC2));

  /* add user code begin adc2_init 3 */

  /* add user code end adc2_init 3 */
}

/* add user code begin 1 */
/**
  * @brief  adc ordinary conversion recovery for dual ADC mode.
  * @param  none
  * @retval none
  */
void adc_ordinary_convert_recovery(void)
{
    uint32_t recovery_index = 0;

    /* disable adc */
    adc_enable(ADC1, FALSE);
    adc_enable(ADC2, FALSE);

    /* record adc mode configuration */
    recovery_index = adc_combine_mode_get();

    /* clear adc mode configuration */
    adc_combine_mode_set(ADC_INDEPENDENT_MODE);

    /* reinitialize dma */
    dma_channel_enable(DMA1_CHANNEL3, FALSE);
    dma_flag_clear(DMA1_FDT3_FLAG);
    dma_data_number_set(DMA1_CHANNEL3, 4);
    dma_channel_enable(DMA1_CHANNEL3, TRUE);

    /* recovery adc mode configuration */
    adc_combine_mode_set((adc_combine_mode_type)recovery_index);

    /* enable adc to detection trigger */
    adc_enable(ADC1, TRUE);
    adc_enable(ADC2, TRUE);
}
/* add user code end 1 */

/* add user code begin 1 */

/* add user code end 1 */
