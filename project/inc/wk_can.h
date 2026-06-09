/* add user code begin Header */
/**
  **************************************************************************
  * @file     wk_can.h
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
#ifndef __WK_CAN_H
#define __WK_CAN_H

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

  /* init can1 function. */
  void wk_can1_init(void);

/* add user code begin exported functions */

/* HAL compat status */
#ifndef HAL_OK
#define HAL_OK      0
#define HAL_ERROR   1
#endif
#ifndef HAL_STATUS_TYPEDEF_DEFINED
#define HAL_STATUS_TYPEDEF_DEFINED
typedef uint32_t HAL_StatusTypeDef;
#endif

/* CAN1 business layer (port of cubemx_yxsui/Core/Src/fdcan.c) */
HAL_StatusTypeDef fdcan_send(uint32_t std_id, const uint8_t *data, uint32_t len);
void fdcan_rx_user(uint32_t id, const uint8_t *data, uint32_t len);  /* weak, override in can_wly */
void wk_can1_rx_dispatch(void);  /* call from CAN1_RX_IRQHandler */
uint32_t fdcan_get_tx_ok_count(void);
uint32_t fdcan_get_tx_fail_count(void);

extern volatile uint8_t g_cantest_stub;   /* 1 = printf instead of TX (for cantest cmd) */
extern volatile uint8_t g_can_rx_debug;   /* 1 = printf RX frames */

/* add user code end exported functions */

#ifdef __cplusplus
}
#endif

#endif
