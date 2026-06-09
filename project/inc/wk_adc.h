/* add user code begin Header */
/**
  **************************************************************************
  * @file     wk_adc.h
  * @brief    header file of work bench config
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

/* define to prevent recursive inclusion -----------------------------------*/
#ifndef __WK_ADC_H
#define __WK_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes -----------------------------------------------------------------------*/
#include "at32f45x_wk_config.h"

/* private includes -------------------------------------------------------------*/
/* add user code begin private includes */

/* add user code end private includes */

/* exported types -------------------------------------------------------------*/
/* add user code begin exported types */

/* ============================================================
 * ADC Business Layer (migrated from cubemx_yxsui/Core/Inc/adc.h)
 * ============================================================ */

/* FOC电流采样数据（由TMR1 TRGO触发，10kHz采样率） */
typedef struct {
    int32_t  i_a_raw;       /* CUR_A 原始值（有符号，已减零点偏置） */
    int32_t  i_b_raw;       /* CUR_B 原始值 */
    uint32_t sample_count;  /* 采样计数 */
} FOC_CurrentSample_t;

/* add user code end exported types */

/* exported constants --------------------------------------------------------*/
/* add user code begin exported constants */

/* add user code end exported constants */

/* exported macro ------------------------------------------------------------*/
/* add user code begin exported macro */

/* add user code end exported macro */

/* exported functions ------------------------------------------------------- */

  /* init adc-common function. */
  void wk_adc_common_init(void);

  /* init adc1 function. */
  void wk_adc1_init(void);

  /* init adc2 function. */
  void wk_adc2_init(void);

/* add user code begin exported functions */

/* ============================================================
 * ADC Business Layer - Exported Variables & Functions
 * ============================================================ */

/* FOC电流采样全局数据（10kHz注入组） */
extern volatile FOC_CurrentSample_t g_foc_current;

/* ADC偏置零点（启动时校准） */
extern volatile int32_t g_adc_offset_a;
extern volatile int32_t g_adc_offset_b;

/* 规则通道实时数据（1kHz普通组 + DMA，温度/电流） */
extern volatile uint16_t g_temp_motor_raw;  /* PA4 ADC1 CH4 */
extern volatile uint16_t g_temp_mos_raw;    /* PA5 ADC1 CH5 */
extern volatile uint16_t g_so_c_raw;        /* PA2 ADC2 CH2 */
extern volatile uint16_t g_so_3_raw;        /* PA3 ADC2 CH3 */
extern volatile uint32_t g_reg_callback_count;
extern volatile uint32_t g_vdc_raw;         /* Bus voltage raw value */

/**
 * @brief  ADC注入组转换完成处理（在PCCE ISR中调用）
 * @note   读取ADC1/ADC2注入组数据，减零点偏置，更新g_foc_current
 */
void adc_foc_on_injected_done(void);

/**
 * @brief  ADC普通组DMA传输完成处理（在DMA FDT或OCCE ISR中调用）
 * @note   从adc_ordinary_buffer[4]读取4路数据，更新g_temp_*, g_so_*
 */
void adc_foc_on_regular_done(void);

/**
 * @brief  校准电流零点偏置（电机静止，IGBT关闭状态下调用）
 * @param  n_samples 采样次数，建议1024
 * @note   阻塞等待采样完成，计算平均值后设置g_adc_offset_a/b
 */
void adc_calibrate_offsets(uint16_t n_samples);

void adc_ordinary_convert_recovery(void);
/* add user code end exported functions */

#ifdef __cplusplus
}
#endif

#endif
