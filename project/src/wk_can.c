/* add user code begin Header */
/**
  **************************************************************************
  * @file     wk_can.c
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
#include "wk_can.h"

/* add user code begin 0 */

#include <string.h>
#include <stdio.h>

/* ============================================================
 * CAN business layer - AT32 port of cubemx_yxsui/Core/Src/fdcan.c
 *
 * Reference API (preserved for can_wly.c / can_debug.c compatibility):
 *   HAL_StatusTypeDef fdcan_send(uint32_t std_id, const uint8_t *data, uint32_t len);
 *   __weak void fdcan_rx_user(uint32_t id, const uint8_t *data, uint32_t len);
 *
 * AT32 implementation:
 *   - TX: can_txbuf_write(CAN_TXBUF_STB) + can_txbuf_transmit
 *   - RX: can_rxbuf_read in CAN1_RX_IRQHandler (calls fdcan_rx_user)
 * ============================================================ */

/* HAL compat status codes (already defined in flash_port.h, redefine if needed) */
#ifndef HAL_OK
#define HAL_OK      0
#define HAL_ERROR   1
typedef uint32_t HAL_StatusTypeDef;
#endif

/* CAN test stub flag (for cantest command - 1=skip TX, just printf the frame) */
volatile uint8_t g_cantest_stub = 0;

/* CAN RX debug flag (defined in can_wly.c) */
extern volatile uint8_t g_can_rx_debug;

/* TX statistics */
static volatile uint32_t g_can_tx_ok_count = 0;
static volatile uint32_t g_can_tx_fail_count = 0;

/**
 * @brief  Map byte length (0..64) to AT32 CAN_DLC_BYTES_x enum
 * @note   Matches FDCAN data length encoding (0x00~0x0F).
 */
static uint8_t len_to_dlc(uint32_t len)
{
    static const uint8_t pad_size[] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
    for (uint32_t idx = 0; idx < sizeof(pad_size); idx++) {
        if (pad_size[idx] >= len) return (uint8_t)idx;
    }
    return 0x0F;  /* CAN_DLC_BYTES_64 */
}

static uint8_t dlc_to_pad(uint8_t dlc)
{
    static const uint8_t pad_size[] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
    if (dlc < 16) return pad_size[dlc];
    return 0;
}

/**
 * @brief  Send a CAN-FD frame on CAN1 (compatible with reference fdcan_send)
 * @param  std_id: 11-bit standard identifier
 * @param  data:   payload buffer
 * @param  len:    byte length (0~64, will be padded to nearest valid DLC)
 * @retval HAL_OK on success
 *
 * Migrated from cubemx_yxsui/Core/Src/fdcan.c:fdcan_send()
 * Always uses CAN-FD + BRS (8Mbit/s data phase) for short frame time.
 */
HAL_StatusTypeDef fdcan_send(uint32_t std_id, const uint8_t *data, uint32_t len)
{
    if (g_cantest_stub) {
        printf("  [TX] ID=0x%03X len=%lu D=", (unsigned int)std_id, (unsigned long)len);
        for (uint32_t i = 0; i < len; i++) printf("%02X ", data[i]);
        printf("\r\n");
        return HAL_OK;
    }
    if (len > 64) return HAL_ERROR;

    uint8_t dlc = len_to_dlc(len);
    uint8_t pad_len = dlc_to_pad(dlc);

    can_txbuf_type tx = {0};
    tx.id             = std_id & 0x7FF;
    tx.id_type        = CAN_ID_STANDARD;
    tx.frame_type     = CAN_FRAME_DATA;
    tx.data_length    = (can_data_length_type)dlc;
#ifdef SUPPORT_CAN_FD
    tx.fd_format      = CAN_FORMAT_FD;
    tx.fd_rate_switch = CAN_BRS_ON;          /* Bit Rate Switch - data phase 8Mbps */
#endif
    tx.tx_timestamp   = FALSE;
    tx.handle         = 0;

    if (data && len) memcpy(tx.data, data, len);
    /* AT32 BSP zero-inits the rest via {0} above */
    (void)pad_len;

    if (can_txbuf_write(CAN1, CAN_TXBUF_STB, &tx) != SUCCESS) {
        g_can_tx_fail_count++;
        return HAL_ERROR;
    }
    if (can_txbuf_transmit(CAN1, CAN_TRANSMIT_STB_ONE) != SUCCESS) {
        g_can_tx_fail_count++;
        return HAL_ERROR;
    }
    g_can_tx_ok_count++;
    return HAL_OK;
}

uint32_t fdcan_get_tx_ok_count(void)   { return g_can_tx_ok_count; }
uint32_t fdcan_get_tx_fail_count(void) { return g_can_tx_fail_count; }

/**
 * @brief  Weak default RX handler (override in can_wly.c / can_debug.c)
 * @param  id:   CAN identifier
 * @param  data: payload buffer (up to 64 bytes)
 * @param  len:  byte length
 */
__attribute__((weak))
void fdcan_rx_user(uint32_t id, const uint8_t *data, uint32_t len)
{
    if (g_can_rx_debug) {
        printf("  [RX] ID=0x%03X len=%lu D=", (unsigned int)id, (unsigned long)len);
        for (uint32_t i = 0; i < len && i < 16; i++) printf("%02X ", data[i]);
        printf("\r\n");
    }
}

/**
 * @brief  CAN1 RX dispatch (call from CAN1_RX_IRQHandler)
 * Reads all pending frames from CAN1 RXBUF and forwards to fdcan_rx_user.
 */
void wk_can1_rx_dispatch(void)
{
    can_rxbuf_type rx;

    while (can_rxbuf_read(CAN1, &rx) != ERROR) {
        uint8_t pad = dlc_to_pad((uint8_t)rx.data_length);
        fdcan_rx_user(rx.id, rx.data, pad);
    }
}

/* add user code end 0 */

/**
  * @brief  init can1 function.
  * @param  none
  * @retval none
  */
void wk_can1_init(void)
{
  /* add user code begin can1_init 0 */

  /* add user code end can1_init 0 */

  gpio_init_type gpio_init_struct;
  can_bittime_type can_bittime_struct;
  /* add user code begin can1_init 1 */

  /* add user code end can1_init 1 */

  /*gpio-----------------------------------------------------------------------------*/ 
  gpio_default_para_init(&gpio_init_struct);

  /* configure the CAN1 TX pin */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE12, GPIO_MUX_9);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_12;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);

  /* configure the CAN1 RX pin */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE11, GPIO_MUX_9);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_11;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);

  crm_can_clock_select(CRM_CAN1, CRM_CAN_CLOCK_SOURCE_PCLK);

  can_software_reset(CAN1, TRUE);

  /*can_bit_time_setting-------------------------------------------------------------*/
  can_bittime_default_para_init(&can_bittime_struct);

  /*set boudrate = pclk/(bittime_div *(bts1_size + bts2_size))-----------------------*/
  can_bittime_struct.bittime_div = 1;
  can_bittime_struct.ac_bts1_size = 144;
  can_bittime_struct.ac_bts2_size = 48;
  can_bittime_struct.ac_rsaw_size = 48;
  can_bittime_struct.fd_bts1_size  = 36;
  can_bittime_struct.fd_bts2_size = 12;
  can_bittime_struct.fd_rsaw_size = 12;
  can_bittime_struct.fd_ssp_offset = 37;
  can_bittime_set(CAN1, &can_bittime_struct);

  /* enable the ISO 11898-1:2015 protocol mode of CAN-FD */
  can_fd_iso_mode_enable(CAN1, TRUE);

  can_software_reset(CAN1, FALSE);

  /*can_base_config------------------------------------------------------------------*/
  can_retransmission_limit_set(CAN1, CAN_RE_TRANS_TIMES_UNLIMIT);
  can_rearbitration_limit_set(CAN1, CAN_RE_ARBI_TIMES_UNLIMIT);
  can_mode_set(CAN1, CAN_MODE_COMMUNICATE);
  can_stb_transmit_mode_set(CAN1, CAN_STB_TRANSMIT_BY_FIFO);
  can_rxbuf_warning_set(CAN1, 6);
  can_rxbuf_overflow_mode_set(CAN1, CAN_RXBUF_OVERFLOW_BE_OVWR);
  can_error_warning_set(CAN1, 15);
  can_restricted_operation_enable(CAN1, FALSE);
  can_receive_all_enable(CAN1, FALSE);

  /* enable receiver interrupt */
  can_interrupt_enable(CAN1, CAN_RIE_INT, TRUE);

  /* add user code begin can1_init 2 */

  /* add user code end can1_init 2 */
}

/* add user code begin 1 */

/* add user code end 1 */
