/* add user code begin Header */
/**
  **************************************************************************
  * @file     wk_usart.c
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
#include "wk_usart.h"

/* add user code begin 0 */
#include <string.h>
#include "ota_app.h"

/* ============================================================
 * isr_print - ring buffer based non-blocking print
 * Migrated from cubemx_yxsui/Core/Src/usart.c
 * AT32 implementation: ring buffer + polling drain (HAL DMA not available)
 * ============================================================ */

#define ISR_PRINT_BUF_SIZE 1024  /* power of 2 for fast modulo */

static volatile uint8_t  s_ring[ISR_PRINT_BUF_SIZE];
static volatile uint16_t s_head = 0;
static volatile uint16_t s_tail = 0;

void isr_print_init(void)
{
    s_head = 0;
    s_tail = 0;
}

void isr_print(const char *str)
{
    if (!str) return;
    uint16_t head = s_head;
    uint16_t tail = s_tail;
    while (*str) {
        uint16_t next = (head + 1) & (ISR_PRINT_BUF_SIZE - 1);
        if (next == tail) break;  /* buffer full */
        s_ring[head] = (uint8_t)(*str++);
        head = next;
    }
    s_head = head;
}

void isr_print_poll(void)
{
    while (s_tail != s_head) {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET);
        usart_data_transmit(USART1, s_ring[s_tail]);
        s_tail = (s_tail + 1) & (ISR_PRINT_BUF_SIZE - 1);
    }
}

/* ============================================================
 * USART1 Debug receive (DMA + IDLE interrupt)
 * Migrated from cubemx_yxsui/Core/Src/usart.c
 * ============================================================ */

#define DBG_RX_BUF_SIZE  512

static uint8_t usart1_rx_dma_buf[DBG_RX_BUF_SIZE];

uint8_t dbgRecvBuf[1024];
volatile uint16_t usart_rx_len = 0;

void USART1_DebugRx_Start(void)
{
    dma_channel_enable(DMA1_CHANNEL4, FALSE);
    DMA1_CHANNEL4->dtcnt = DBG_RX_BUF_SIZE;
    DMA1_CHANNEL4->maddr = (uint32_t)usart1_rx_dma_buf;
    DMA1_CHANNEL4->paddr = (uint32_t)&USART1->dt;
    dma_channel_enable(DMA1_CHANNEL4, TRUE);
}

/* Diagnostic counters (visible from main loop) */
volatile uint32_t g_usart1_idle_cnt = 0;
volatile uint32_t g_usart1_rx_bytes_total = 0;

void USART1_IDLE_Handler(void)
{
    if (usart_interrupt_flag_get(USART1, USART_IDLEF_FLAG) == RESET) return;

    g_usart1_idle_cnt++;

    /* Clear IDLE flag (read SR then DR) */
    usart_flag_clear(USART1, USART_IDLEF_FLAG);

    /* Stop DMA, calculate received length */
    dma_channel_enable(DMA1_CHANNEL4, FALSE);
    uint16_t remain = DMA1_CHANNEL4->dtcnt;
    uint16_t rx_size = DBG_RX_BUF_SIZE - remain;

    g_usart1_rx_bytes_total += rx_size;

    if (rx_size > 0) {
        if (g_ota_rx_mode) {
            /* OTA mode: feed raw bytes to OTA ring buffer */
            ota_rx_feed(usart1_rx_dma_buf, rx_size);
        } else if (usart_rx_len == 0) {
            /* Normal text command mode */
            if (rx_size > sizeof(dbgRecvBuf) - 1) rx_size = sizeof(dbgRecvBuf) - 1;
            memcpy(dbgRecvBuf, usart1_rx_dma_buf, rx_size);
            dbgRecvBuf[rx_size] = '\0';
            usart_rx_len = rx_size;
        }
    }

    /* Restart DMA for next reception */
    USART1_DebugRx_Start();
}

/* add user code end 0 */

/**
  * @brief  init usart1 function
  * @param  none
  * @retval none
  */
void wk_usart1_init(void)
{
  /* add user code begin usart1_init 0 */

  /* add user code end usart1_init 0 */

  gpio_init_type gpio_init_struct;
  gpio_default_para_init(&gpio_init_struct);

  /* add user code begin usart1_init 1 */

  /* add user code end usart1_init 1 */

  /* configure the TX pin */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE15, GPIO_MUX_7);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_15;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);

  /* configure the RX pin */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE3, GPIO_MUX_7);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_3;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

  /* configure param */
  usart_init(USART1, 921600, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_transmitter_enable(USART1, TRUE);
  usart_receiver_enable(USART1, TRUE);
  usart_parity_selection_config(USART1, USART_PARITY_NONE);

  usart_dma_transmitter_enable(USART1, TRUE);

  usart_dma_receiver_enable(USART1, TRUE);

  usart_hardware_flow_control_set(USART1, USART_HARDWARE_FLOW_NONE);

  /* enable idle interrupt */
  usart_interrupt_enable(USART1, USART_IDLE_INT, TRUE);

  /* enable error interrupt */
  usart_interrupt_enable(USART1, USART_ERR_INT, TRUE);

  /* add user code begin usart1_init 2 */

  /* add user code end usart1_init 2 */

  usart_enable(USART1, TRUE);

  /* add user code begin usart1_init 3 */

  /* add user code end usart1_init 3 */
}

/**
  * @brief  init usart3 function
  * @param  none
  * @retval none
  */
void wk_usart3_init(void)
{
  /* add user code begin usart3_init 0 */

  /* add user code end usart3_init 0 */

  gpio_init_type gpio_init_struct;
  gpio_default_para_init(&gpio_init_struct);

  /* add user code begin usart3_init 1 */

  /* add user code end usart3_init 1 */

  /* configure the TX pin */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE10, GPIO_MUX_7);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_10;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

  /* configure the RX pin */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE6, GPIO_MUX_8);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_6;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);

  /* configure the DE pin */
  gpio_pin_mux_config(EN_485_GPIO_PORT, GPIO_PINS_SOURCE14, GPIO_MUX_7);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = EN_485_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(EN_485_GPIO_PORT, &gpio_init_struct);

  /* configure param */
  usart_init(USART3, 2500000, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_transmitter_enable(USART3, TRUE);
  usart_receiver_enable(USART3, TRUE);
  usart_parity_selection_config(USART3, USART_PARITY_NONE);

  usart_de_polarity_set(USART3, USART_DE_POLARITY_HIGH);
  usart_rs485_delay_time_config(USART3, 0, 0);
  usart_rs485_mode_enable(USART3, TRUE);

  usart_dma_transmitter_enable(USART3, TRUE);

  usart_dma_receiver_enable(USART3, TRUE);

  usart_hardware_flow_control_set(USART3, USART_HARDWARE_FLOW_NONE);

  /* enable idle interrupt */
  usart_interrupt_enable(USART3, USART_IDLE_INT, TRUE);

  /* enable error interrupt */
  usart_interrupt_enable(USART3, USART_ERR_INT, TRUE);

  /* add user code begin usart3_init 2 */

  /* add user code end usart3_init 2 */

  usart_enable(USART3, TRUE);

  /* add user code begin usart3_init 3 */

  /* add user code end usart3_init 3 */
}

/* add user code begin 1 */

/* ============================================================
 * DPT Dual Magnetic Encoder Driver (blocking version)
 * USART3 @ 2.5Mbps with hardware RS485 DE control (PB14)
 * Migrated from cubemx_yxsui/Core/Src/encoder.c
 * ============================================================ */

/* CRC-8 lookup table (poly x^8+x^7+x^4+x^2+x+1) from DPT datasheet ch.12 */
static const uint8_t dpt_crc8_table[256] = {
    0x00, 0x97, 0xB9, 0x2E, 0xE5, 0x72, 0x5C, 0xCB, 0x5D, 0xCA, 0xE4, 0x73, 0xB8, 0x2F, 0x01, 0x96,
    0xBA, 0x2D, 0x03, 0x94, 0x5F, 0xC8, 0xE6, 0x71, 0xE7, 0x70, 0x5E, 0xC9, 0x02, 0x95, 0xBB, 0x2C,
    0xE3, 0x74, 0x5A, 0xCD, 0x06, 0x91, 0xBF, 0x28, 0xBE, 0x29, 0x07, 0x90, 0x5B, 0xCC, 0xE2, 0x75,
    0x59, 0xCE, 0xE0, 0x77, 0xBC, 0x2B, 0x05, 0x92, 0x04, 0x93, 0xBD, 0x2A, 0xE1, 0x76, 0x58, 0xCF,
    0x51, 0xC6, 0xE8, 0x7F, 0xB4, 0x23, 0x0D, 0x9A, 0x0C, 0x9B, 0xB5, 0x22, 0xE9, 0x7E, 0x50, 0xC7,
    0xEB, 0x7C, 0x52, 0xC5, 0x0E, 0x99, 0xB7, 0x20, 0xB6, 0x21, 0x0F, 0x98, 0x53, 0xC4, 0xEA, 0x7D,
    0xB2, 0x25, 0x0B, 0x9C, 0x57, 0xC0, 0xEE, 0x79, 0xEF, 0x78, 0x56, 0xC1, 0x0A, 0x9D, 0xB3, 0x24,
    0x08, 0x9F, 0xB1, 0x26, 0xED, 0x7A, 0x54, 0xC3, 0x55, 0xC2, 0xEC, 0x7B, 0xB0, 0x27, 0x09, 0x9E,
    0xA2, 0x35, 0x1B, 0x8C, 0x47, 0xD0, 0xFE, 0x69, 0xFF, 0x68, 0x46, 0xD1, 0x1A, 0x8D, 0xA3, 0x34,
    0x18, 0x8F, 0xA1, 0x36, 0xFD, 0x6A, 0x44, 0xD3, 0x45, 0xD2, 0xFC, 0x6B, 0xA0, 0x37, 0x19, 0x8E,
    0x41, 0xD6, 0xF8, 0x6F, 0xA4, 0x33, 0x1D, 0x8A, 0x1C, 0x8B, 0xA5, 0x32, 0xF9, 0x6E, 0x40, 0xD7,
    0xFB, 0x6C, 0x42, 0xD5, 0x1E, 0x89, 0xA7, 0x30, 0xA6, 0x31, 0x1F, 0x88, 0x43, 0xD4, 0xFA, 0x6D,
    0xF3, 0x64, 0x4A, 0xDD, 0x16, 0x81, 0xAF, 0x38, 0xAE, 0x39, 0x17, 0x80, 0x4B, 0xDC, 0xF2, 0x65,
    0x49, 0xDE, 0xF0, 0x67, 0xAC, 0x3B, 0x15, 0x82, 0x14, 0x83, 0xAD, 0x3A, 0xF1, 0x66, 0x48, 0xDF,
    0x10, 0x87, 0xA9, 0x3E, 0xF5, 0x62, 0x4C, 0xDB, 0x4D, 0xDA, 0xF4, 0x63, 0xA8, 0x3F, 0x11, 0x86,
    0xAA, 0x3D, 0x13, 0x84, 0x4F, 0xD8, 0xF6, 0x61, 0xF7, 0x60, 0x4E, 0xD9, 0x12, 0x85, 0xAB, 0x3C
};

uint8_t DPT_CalcCRC8(const uint8_t *buffer, uint8_t length) {
    uint8_t temp = *buffer++;
    while (--length) {
        temp = *buffer++ ^ dpt_crc8_table[temp];
    }
    return dpt_crc8_table[temp];
}

/* External systick_ms for timeout (defined in at32f45x_int.c) */
extern volatile uint32_t systick_ms;

/* Blocking transaction: send 1B cmd, receive rx_len bytes
 * Hardware RS485 DE auto-controls direction; no software switch needed.
 */
static DPT_Status dpt_transact(uint8_t cmd, uint8_t *rx_buf, uint8_t rx_len, uint32_t timeout_ms)
{
    /* Flush any stale RX data */
    while (usart_flag_get(USART3, USART_RDBF_FLAG) != RESET) {
        (void)usart_data_receive(USART3);
    }

    /* Send command (1 byte) - hardware DE auto-asserts during TX */
    while (usart_flag_get(USART3, USART_TDBE_FLAG) == RESET);
    usart_data_transmit(USART3, cmd);

    /* Wait for TX complete (DE will auto-deassert) */
    uint32_t t0 = systick_ms;
    while (usart_flag_get(USART3, USART_TDC_FLAG) == RESET) {
        if (systick_ms - t0 > timeout_ms) return DPT_ERR_TIMEOUT;
    }

    /* Receive rx_len bytes */
    for (uint8_t i = 0; i < rx_len; i++) {
        while (usart_flag_get(USART3, USART_RDBF_FLAG) == RESET) {
            if (systick_ms - t0 > timeout_ms) return DPT_ERR_TIMEOUT;
        }
        rx_buf[i] = (uint8_t)usart_data_receive(USART3);
    }

    /* Verify CRC (full frame XOR through table should yield 0) */
    if (DPT_CalcCRC8(rx_buf, rx_len) != 0) return DPT_ERR_CRC;
    return DPT_OK;
}

DPT_Status DPT_ReadDualAngle(DPT_Angles *out, uint32_t timeout_ms)
{
    if (!out) return DPT_ERR_PARAM;
    uint8_t buf[7];
    DPT_Status st = dpt_transact(DPT_CMD_READ_DUAL_ANGLE, buf, 7, timeout_ms);
    if (st != DPT_OK) return st;

    out->inner_raw = ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | buf[0];
    out->outer_raw = ((uint32_t)buf[5] << 16) | ((uint32_t)buf[4] << 8) | buf[3];
    out->inner_deg = (float)out->inner_raw / (float)(1u << 24) * 360.0f;
    out->outer_deg = (float)out->outer_raw / (float)(1u << 24) * 360.0f;
    out->has_status = 0;
    return DPT_OK;
}

DPT_Status DPT_ReadDualAngleWithStatus(DPT_Angles *out, uint32_t timeout_ms)
{
    if (!out) return DPT_ERR_PARAM;
    uint8_t buf[8];
    DPT_Status st = dpt_transact(DPT_CMD_READ_DUAL_WITH_STATUS, buf, 8, timeout_ms);
    if (st != DPT_OK) return st;

    out->inner_raw = ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | buf[0];
    out->outer_raw = ((uint32_t)buf[5] << 16) | ((uint32_t)buf[4] << 8) | buf[3];
    out->inner_deg = (float)out->inner_raw / (float)(1u << 24) * 360.0f;
    out->outer_deg = (float)out->outer_raw / (float)(1u << 24) * 360.0f;
    out->status = buf[6];
    out->has_status = 1;
    return DPT_OK;
}

DPT_Status DPT_ReadTemperature(int16_t *temp_tenths, uint32_t timeout_ms)
{
    if (!temp_tenths) return DPT_ERR_PARAM;
    uint8_t buf[3];
    DPT_Status st = dpt_transact(DPT_CMD_READ_TEMPERATURE, buf, 3, timeout_ms);
    if (st != DPT_OK) return st;

    *temp_tenths = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    return DPT_OK;
}

/* ============================================================
 * DPT Async driver: DMA1_CH1 (TX) + DMA1_CH2 (RX) + USART3 IDLE IRQ
 * Reference: cubemx_yxsui async DMA pattern, AT32 idiom.
 * ============================================================ */

#define DPT_RX_FRAME_LEN     7u    /* 0x33 response: A0..A2 B0..B2 CRC */
#define DPT_RX_BUF_SIZE      8u    /* tolerate +1 stray byte before IDLE */

typedef enum {
    DPT_ST_IDLE = 0,   /* idle, ready to start a new transaction */
    DPT_ST_BUSY = 1    /* TX armed, waiting for RX + IDLE */
} DPT_AsyncState;

static volatile uint8_t  s_dpt_state = DPT_ST_IDLE;
static volatile uint8_t  s_dpt_ready = 0;     /* 1 after DPT_Async_Init */
static uint8_t           s_dpt_tx[1] = { DPT_CMD_READ_DUAL_ANGLE };
static uint8_t           s_dpt_rx[DPT_RX_BUF_SIZE];
static DPT_Angles        s_dpt_latest;       /* updated by IDLE IRQ */
static volatile uint32_t s_dpt_seq = 0;       /* monotonic; readers detect torn copy */
static volatile uint32_t s_dpt_ok = 0;
static volatile uint32_t s_dpt_crc_err = 0;
static volatile uint32_t s_dpt_len_err = 0;
static volatile uint32_t s_dpt_busy = 0;

void DPT_Async_Init(void)
{
    dma_init_type t;

    /* === DMA1_CH1: USART3 TX, BYTE width (overrides WB default HALFWORD) === */
    dma_channel_enable(DMA1_CHANNEL1, FALSE);
    dma_reset(DMA1_CHANNEL1);
    dma_default_para_init(&t);
    t.buffer_size           = 1;
    t.direction             = DMA_DIR_MEMORY_TO_PERIPHERAL;
    t.peripheral_base_addr  = (uint32_t)&USART3->dt;
    t.memory_base_addr      = (uint32_t)s_dpt_tx;
    t.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    t.memory_data_width     = DMA_MEMORY_DATA_WIDTH_BYTE;
    t.peripheral_inc_enable = FALSE;
    t.memory_inc_enable     = TRUE;
    t.priority              = DMA_PRIORITY_MEDIUM;
    t.loop_mode_enable      = FALSE;
    dma_init(DMA1_CHANNEL1, &t);
    dmamux_enable(DMA1, TRUE);
    dmamux_init(DMA1MUX_CHANNEL1, DMAMUX_DMAREQ_ID_USART3_TX);

    /* === DMA1_CH2: USART3 RX, BYTE width === */
    dma_channel_enable(DMA1_CHANNEL2, FALSE);
    dma_reset(DMA1_CHANNEL2);
    dma_default_para_init(&t);
    t.buffer_size           = DPT_RX_BUF_SIZE;
    t.direction             = DMA_DIR_PERIPHERAL_TO_MEMORY;
    t.peripheral_base_addr  = (uint32_t)&USART3->dt;
    t.memory_base_addr      = (uint32_t)s_dpt_rx;
    t.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    t.memory_data_width     = DMA_MEMORY_DATA_WIDTH_BYTE;
    t.peripheral_inc_enable = FALSE;
    t.memory_inc_enable     = TRUE;
    t.priority              = DMA_PRIORITY_MEDIUM;
    t.loop_mode_enable      = FALSE;
    dma_init(DMA1_CHANNEL2, &t);
    dmamux_init(DMA1MUX_CHANNEL2, DMAMUX_DMAREQ_ID_USART3_RX);

    /* clear any latched IDLE flag from prior traffic */
    usart_flag_clear(USART3, USART_IDLEF_FLAG);

    s_dpt_state    = DPT_ST_IDLE;
    s_dpt_seq      = 0;
    s_dpt_ok       = 0;
    s_dpt_crc_err  = 0;
    s_dpt_len_err  = 0;
    s_dpt_busy     = 0;
    s_dpt_ready    = 1;
}

uint8_t DPT_AsyncRequest(void)
{
    if (!s_dpt_ready) return 0;          /* DPT_Async_Init not yet called */
    /* Caller may run from PCCE ISR or main loop; both are fine because
     * USART3 IDLE IRQ is the only writer of s_dpt_state and runs at the
     * same NVIC priority — same-priority ISRs cannot preempt each other.
     * Main-loop callers tolerate a brief race because the worst case is
     * an extra "busy" skip on a frame we'd retry next tick anyway. */
    if (s_dpt_state != DPT_ST_IDLE) {
        s_dpt_busy++;
        return 0;
    }
    s_dpt_state = DPT_ST_BUSY;

    /* Drain any stale RX byte (would otherwise land in DMA buffer) */
    while (usart_flag_get(USART3, USART_RDBF_FLAG) != RESET) {
        (void)usart_data_receive(USART3);
    }

    /* Arm RX BEFORE TX (so first response byte never gets lost) */
    dma_channel_enable(DMA1_CHANNEL2, FALSE);
    DMA1_CHANNEL2->dtcnt = DPT_RX_BUF_SIZE;
    DMA1_CHANNEL2->maddr = (uint32_t)s_dpt_rx;
    dma_channel_enable(DMA1_CHANNEL2, TRUE);

    /* Arm TX */
    dma_channel_enable(DMA1_CHANNEL1, FALSE);
    s_dpt_tx[0] = DPT_CMD_READ_DUAL_ANGLE;
    DMA1_CHANNEL1->dtcnt = 1;
    DMA1_CHANNEL1->maddr = (uint32_t)s_dpt_tx;
    usart_flag_clear(USART3, USART_TDC_FLAG);
    dma_channel_enable(DMA1_CHANNEL1, TRUE);
    return 1;
}

void DPT_USART3_IDLE_Handler(void)
{
    if (usart_interrupt_flag_get(USART3, USART_IDLEF_FLAG) == RESET) return;
    usart_flag_clear(USART3, USART_IDLEF_FLAG);

    if (s_dpt_state != DPT_ST_BUSY) return;  /* spurious IDLE while idle */

    dma_channel_enable(DMA1_CHANNEL2, FALSE);
    uint16_t remain = (uint16_t)DMA1_CHANNEL2->dtcnt;
    uint16_t got    = (uint16_t)DPT_RX_BUF_SIZE - remain;

    if (got != DPT_RX_FRAME_LEN) {
        s_dpt_len_err++;
        s_dpt_state = DPT_ST_IDLE;
        return;
    }

    if (DPT_CalcCRC8(s_dpt_rx, DPT_RX_FRAME_LEN) != 0) {
        s_dpt_crc_err++;
        s_dpt_state = DPT_ST_IDLE;
        return;
    }

    uint32_t inner = ((uint32_t)s_dpt_rx[2] << 16) | ((uint32_t)s_dpt_rx[1] << 8) | s_dpt_rx[0];
    uint32_t outer = ((uint32_t)s_dpt_rx[5] << 16) | ((uint32_t)s_dpt_rx[4] << 8) | s_dpt_rx[3];

    s_dpt_latest.inner_raw = inner;
    s_dpt_latest.outer_raw = outer;
    s_dpt_latest.inner_deg = (float)inner * (360.0f / (float)(1u << 24));
    s_dpt_latest.outer_deg = (float)outer * (360.0f / (float)(1u << 24));
    s_dpt_latest.has_status = 0;
    s_dpt_seq++;
    s_dpt_ok++;

    /* Record encoder-done timestamp for logid 140 timing analysis */
    extern volatile uint32_t g_tim1_enc_done_cycles;
    g_tim1_enc_done_cycles = DWT->CYCCNT;

    s_dpt_state = DPT_ST_IDLE;
}

void DPT_GetLatestAngles(DPT_Angles *out)
{
    if (!out) return;
    /* Sequence-number snapshot: re-read until producer didn't bump seq mid-copy. */
    uint32_t seq1, seq2;
    do {
        seq1 = s_dpt_seq;
        *out = s_dpt_latest;
        seq2 = s_dpt_seq;
    } while (seq1 != seq2);
}

void DPT_GetLatestAngles_ISR(DPT_Angles *out)
{
    /* From PCCE ISR (same priority as USART3 IRQ → no preemption):
     * a single read is atomic w.r.t. the producer. Use seq pattern anyway
     * for parity with the non-ISR version; cost is two volatile reads. */
    if (!out) return;
    uint32_t seq1, seq2;
    do {
        seq1 = s_dpt_seq;
        *out = s_dpt_latest;
        seq2 = s_dpt_seq;
    } while (seq1 != seq2);
}

void DPT_GetAsyncStats(uint32_t *ok, uint32_t *crc_err,
                       uint32_t *len_err, uint32_t *busy_skip)
{
    if (ok)        *ok        = s_dpt_ok;
    if (crc_err)   *crc_err   = s_dpt_crc_err;
    if (len_err)   *len_err   = s_dpt_len_err;
    if (busy_skip) *busy_skip = s_dpt_busy;
}

/* add user code end 1 */
