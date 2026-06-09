/* add user code begin Header */
/**
  **************************************************************************
  * @file     wk_usart.h
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
#ifndef __WK_USART_H
#define __WK_USART_H

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

  /* init usart1 function. */
  void wk_usart1_init(void);

  /* init usart3 function. */
  void wk_usart3_init(void);

/* add user code begin exported functions */

/* ============================================================
 * isr_print - non-blocking ISR-safe debug output
 * Migrated from cubemx_yxsui/Core/Src/usart.c
 * ============================================================ */
void isr_print_init(void);
void isr_print(const char *str);
void isr_print_poll(void);  /* Call from main loop to drain ring buffer */

/* ============================================================
 * USART1 Debug serial receive (DMA + IDLE interrupt)
 * ============================================================ */
extern uint8_t dbgRecvBuf[1024];
extern volatile uint16_t usart_rx_len;

void USART1_DebugRx_Start(void);
void USART1_IDLE_Handler(void);  /* Call from USART1 IRQ IDLE handler */

/* ============================================================
 * DPT Dual Magnetic Encoder (USART3 @ 2.5Mbps RS485 hardware mode)
 * Migrated from cubemx_yxsui/Core/Src/encoder.c
 * ============================================================ */

/* DPT command codes (datasheet chapter 7) */
#define DPT_CMD_READ_DUAL_ANGLE          0x33u  /* 7B response: A0..A2 B0..B2 CRC */
#define DPT_CMD_READ_DUAL_WITH_STATUS    0x43u  /* 8B response: A0..A2 B0..B2 S CRC */
#define DPT_CMD_READ_TEMPERATURE         0x74u  /* 3B response: T0 T1 CRC */

typedef enum {
    DPT_OK             = 0,
    DPT_ERR_TIMEOUT    = 1,
    DPT_ERR_CRC        = 2,
    DPT_ERR_BUSY       = 3,
    DPT_ERR_HAL        = 4,
    DPT_ERR_PARAM      = 5,
} DPT_Status;

typedef struct {
    uint32_t inner_raw;        /* 24-bit inner angle [0, 1<<24) */
    uint32_t outer_raw;        /* 24-bit outer angle [0, 1<<24) */
    float    inner_deg;        /* degrees (0..360) */
    float    outer_deg;
    uint8_t  status;           /* valid only for *_WithStatus reads */
    uint8_t  has_status;
} DPT_Angles;

/* CRC-8 over raw bytes, poly x^8+x^7+x^4+x^2+x+1 (datasheet ch.12) */
uint8_t DPT_CalcCRC8(const uint8_t *buffer, uint8_t length);

/* Blocking read of both 24-bit angles + CRC (command 0x33, 7B response) */
DPT_Status DPT_ReadDualAngle(DPT_Angles *out, uint32_t timeout_ms);

/* Blocking read of both angles + status + CRC (command 0x43, 8B response) */
DPT_Status DPT_ReadDualAngleWithStatus(DPT_Angles *out, uint32_t timeout_ms);

/* Blocking read of internal temperature (command 0x74, 3B response, 0.1degC) */
DPT_Status DPT_ReadTemperature(int16_t *temp_tenths, uint32_t timeout_ms);

/* ============================================================
 * DPT Async API (DMA1_CH1 TX / DMA1_CH2 RX + USART3 IDLE interrupt)
 * - Init reconfigures DMA1_CH1/CH2 to BYTE width (WorkBench default is HALFWORD;
 *   that is broken for 8-bit UART transfers and never worked).
 * - AsyncRequest is non-blocking: kicks one 0x33 transaction (1B TX, 7B RX).
 *   Skipped if previous transaction still in flight (counted as busy).
 * - GetLatestAngles snapshots last successfully decoded frame (lock-free
 *   sequence-number pattern; safe from main loop and same-priority ISRs).
 * - Round-trip @ 2.5 Mbps: ~50 µs; safe to kick once per 100 µs PCCE ISR.
 * ============================================================ */
void       DPT_Async_Init(void);
uint8_t    DPT_AsyncRequest(void);              /* returns 1=kicked, 0=busy */
void       DPT_GetLatestAngles(DPT_Angles *out);
void       DPT_GetLatestAngles_ISR(DPT_Angles *out);
void       DPT_USART3_IDLE_Handler(void);       /* called from USART3_IRQHandler */
void       DPT_GetAsyncStats(uint32_t *ok, uint32_t *crc_err,
                             uint32_t *len_err, uint32_t *busy_skip);

/* add user code end exported functions */

#ifdef __cplusplus
}
#endif

#endif
