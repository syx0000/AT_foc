/**
  **************************************************************************
  * @file     mc_comm_uart.h
  * @brief    Declaration of peripherals configuration related to communication interface.
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

#ifndef __MC_COMM_UART_H
#define __MC_COMM_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mc_lib.h"

void dma_uart_configuration(void);
void uart_rx_init(void);
#if defined USE_UART_LOG
flag_status uart_log_rx_take(uint8_t *data, uint8_t *length);
#endif

/* UART external buffer variables define  */
extern uint8_t usart_rx_buffer[];

#ifdef __cplusplus
}
#endif

#endif

