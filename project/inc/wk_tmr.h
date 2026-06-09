/* add user code begin Header */
/**
  **************************************************************************
  * @file     wk_tmr.h
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
#ifndef __WK_TMR_H
#define __WK_TMR_H

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

/* add user code end exported types */

/* exported constants --------------------------------------------------------*/
/* add user code begin exported constants */

/* add user code end exported constants */

/* exported macro ------------------------------------------------------------*/
/* add user code begin exported macro */

/* add user code end exported macro */

/* exported functions ------------------------------------------------------- */

  /* init tmr1 function. */
  void wk_tmr1_init(void);

  /* init tmr6 function. */
  void wk_tmr6_init(void);

/* add user code begin exported functions */

/* ============================================================
 * DWT cycle counter (for performance measurement)
 * Migrated from cubemx_yxsui/Core/Src/tim.c
 * ============================================================ */
void DWT_Init(void);
uint32_t DWT_GetCycles(void);
uint32_t DWT_GetMicros(void);
uint32_t DWT_CyclesToUs(uint32_t cycles);

/* ============================================================
 * TMR1 PWM control (FOC motor drive)
 * Migrated from cubemx_yxsui/Core/Src/tim.c + iflytek foc_bsp.h
 * ============================================================ */
void TIM1_PWM_Start(void);
void TIM1_PWM_Stop(void);
void TIM1_SetDuty(uint32_t ccr1, uint32_t ccr2, uint32_t ccr3);

/* PWM compare register write (hot path, called by SVPWM in 10kHz ISR) */
void pwm_ccr_set(uint32_t ccr1, uint32_t ccr2, uint32_t ccr3);

/* ADC ISR performance counters (DWT cycles, set by foc_api.c) */
extern volatile uint32_t g_adc_isr_t_read,    g_adc_isr_t_enc,    g_adc_isr_t_pos,    g_adc_isr_t_vel,    g_adc_isr_t_cur;
extern volatile uint32_t g_adc_isr_t_read_max, g_adc_isr_t_enc_max, g_adc_isr_t_pos_max, g_adc_isr_t_vel_max, g_adc_isr_t_cur_max;

/* add user code end exported functions */

#ifdef __cplusplus
}
#endif

#endif
