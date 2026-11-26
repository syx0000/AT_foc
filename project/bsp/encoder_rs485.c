/**
 ******************************************************************************
 * @file     encoder_rs485.c
 * @brief    锐鹰 SRMA46 RS485 绝对值编码器驱动
 *
 * 协议要点（规格书 V1.1 第 7 章）：
 *  - 2.5 Mbps，8N1，半双工；DE 由 USART3 硬件自动控制
 *  - 请求帧：1 字节 CF
 *  - ID0(0x02) 回应 6 字节：CF + SF + ABS0 + ABS1 + ABS2 + CRC
 *  - CRC：除 CRC 外所有字节（含 CF）异或
 *  - ABS = ABS0 | (ABS1<<8) | (ABS2<<16)，取低 17 位
 *
 * 若收发器 RE 常开，接收端会先收到本机发出的 CF 回显，解析时按
 * "CF + 有效 CRC" 在缓冲区中搜索帧头，兼容有/无回显两种接法。
 ******************************************************************************
 */

#include "encoder_rs485.h"
#include "at32f45x.h"
#include "motor_timebase.h"
#include "motor_board_config.h"

static volatile bool s_paused = false;
static volatile bool s_initialized = false;

/* ID0 回应字节数：CF(1) + SF(1) + ABS0~2(3) + CRC(1) */
#define ENC_RX_LEN_ABS        6u

/* 缓冲区容纳：可能的回显 1 字节 + 完整回应 + 余量 */
#define ENC_DMA_RX_BUF_SIZE   16u

/* Allow turnaround and a late reply without blocking the fast loop. */
#define ENC_TIMEOUT_US        125u
#define ENC_UART_ERRORS       (USART_PERR_FLAG | USART_FERR_FLAG | USART_NERR_FLAG | USART_ROERR_FLAG)

typedef enum {
    ENC_STATE_IDLE = 0,
    ENC_STATE_WAIT_RESP,
} enc_state_t;

static volatile enc_state_t  s_state        = ENC_STATE_IDLE;
static uint32_t              s_request_cycles;
static uint32_t              s_timeout_cycles;
static volatile enc_result_t s_last_result  = ENC_OK;

static volatile uint8_t s_rx_buf[ENC_DMA_RX_BUF_SIZE] __attribute__((aligned(4)));
static uint8_t s_tx_byte __attribute__((aligned(4)));

static volatile enc_data_t  s_data;
static volatile enc_data_t  s_control_data;
static uint32_t s_sample_window_cycles;
static volatile enc_stats_t s_stats;

static uint8_t enc_crc8(const volatile uint8_t *buf, uint8_t len)
{
    uint8_t crc = 0u;
    for (uint8_t i = 0u; i < len; i++) {
        crc ^= buf[i];
    }
    return crc;
}

/* WorkBench 生成的 USART3 DMA 为半字宽度，这里强制改为字节宽度。 */
static void enc_dma_byte_width_config(dma_channel_type *ch)
{
    ch->ctrl_bit.chen   = 0u;
    ch->ctrl_bit.mwidth = 0u;   /* 8 bit */
    ch->ctrl_bit.pwidth = 0u;   /* 8 bit */
}

static void enc_dma_rx_reload(void)
{
    dma_channel_enable(DMA1_CHANNEL2, FALSE);
    dma_flag_clear(DMA1_FDT2_FLAG);
    DMA1_CHANNEL2->maddr = (uint32_t)s_rx_buf;
    DMA1_CHANNEL2->dtcnt = ENC_DMA_RX_BUF_SIZE;
    dma_channel_enable(DMA1_CHANNEL2, TRUE);
}

static void enc_send_cmd(uint8_t cf)
{
    s_tx_byte = cf;
    dma_channel_enable(DMA1_CHANNEL1, FALSE);
    dma_flag_clear(DMA1_FDT1_FLAG);
    DMA1_CHANNEL1->maddr = (uint32_t)&s_tx_byte;
    DMA1_CHANNEL1->dtcnt = 1u;
    dma_channel_enable(DMA1_CHANNEL1, TRUE);
}

void encoder_rs485_init(void)
{
    /* 停掉 main.c 中按 WorkBench 默认缓冲区启动的两个通道，
     * 防止上电时把默认缓冲区内容发到总线上。 */
    dma_channel_enable(DMA1_CHANNEL1, FALSE);
    dma_channel_enable(DMA1_CHANNEL2, FALSE);
    enc_dma_byte_width_config(DMA1_CHANNEL1);
    enc_dma_byte_width_config(DMA1_CHANNEL2);
    DMA1_CHANNEL1->paddr = (uint32_t)&USART3->dt;
    DMA1_CHANNEL2->paddr = (uint32_t)&USART3->dt;

    /* 丢弃上电期间可能收到的残留字节 */
    (void)usart_data_receive(USART3);
    usart_flag_clear(USART3, USART_NERR_FLAG | USART_ROERR_FLAG |
                             USART_FERR_FLAG | USART_IDLEF_FLAG);

    usart_interrupt_enable(USART3, USART_IDLE_INT, TRUE);
    usart_interrupt_enable(USART3, USART_ERR_INT, TRUE);
    s_state = ENC_STATE_IDLE;
    s_last_result = ENC_OK;
    s_data.valid = false;
    s_data.update_count = 0u;
    s_timeout_cycles = (system_core_clock / 1000000u) * ENC_TIMEOUT_US;
    s_sample_window_cycles = system_core_clock / MOTOR_PWM_FREQUENCY_HZ;
    s_initialized = true;
}

static bool enc_receive(uint32_t sts, bool expired)
{
    const uint16_t rx_len = ENC_DMA_RX_BUF_SIZE - (uint16_t)DMA1_CHANNEL2->dtcnt;
    uint16_t start = 0u;
    bool found = false;

    const bool uart_error = (sts & ENC_UART_ERRORS) != 0u;
    const bool idle = (sts & USART_IDLEF_FLAG) != 0u;

    if (!expired && !uart_error && (!idle || rx_len < ENC_RX_LEN_ABS)) {
        return false;
    }

    /* 从后往前找“CF + 有效 CRC”的 6 字节帧，跳过可能的回显 */
    if (idle && rx_len >= ENC_RX_LEN_ABS && !uart_error) {
        start = (uint16_t)(rx_len - ENC_RX_LEN_ABS + 1u);
    }
    while (start != 0u) {
        --start;
        const volatile uint8_t *f = &s_rx_buf[start];
        if ((f[0] == ENC_CF_READ_ABS) &&
            (enc_crc8(f, ENC_RX_LEN_ABS - 1u) == f[ENC_RX_LEN_ABS - 1u])) {
            found = true;
            break;
        }
    }

    /* Six bytes can also be an echo followed by a partial reply. */
    if (!expired && !found && rx_len == ENC_RX_LEN_ABS &&
        (sts & ENC_UART_ERRORS) == 0u) {
        return false;
    }

    dma_channel_enable(DMA1_CHANNEL2, FALSE);
    s_stats.last_rx_len = (uint8_t)rx_len;
    for (uint8_t i = 0u; i < sizeof(s_stats.last_raw); i++) {
        s_stats.last_raw[i] = (i < rx_len) ? s_rx_buf[i] : 0u;
    }

    if (!uart_error && (!idle || rx_len < ENC_RX_LEN_ABS)) {
        s_last_result = ENC_ERR_TIMEOUT;
        s_data.valid = false;
        s_stats.timeout_count++;
        return true;
    }

    if (!found) {
        s_last_result = ENC_ERR_CRC;
        s_data.valid  = false;
        s_stats.crc_error_count++;
        return true;
    }

    {
        const volatile uint8_t *f = &s_rx_buf[start];
        uint8_t sf = f[1];

        s_data.status = sf;
        if (sf & (ENC_SF_CNT_ERR | ENC_SF_MULTI_ERR)) {
            s_last_result = ENC_ERR_STATUS;
            s_data.valid  = false;
            s_stats.status_error_count++;
            return true;
        }

        {
            uint32_t raw_pos = (uint32_t)f[2]
                             | ((uint32_t)f[3] << 8u)
                             | ((uint32_t)(f[4] & 0x01u) << 16u);
#if (MOTOR_ENCODER_DIRECTION_INVERTED != 0U)
            /* 取反到与物理电角度同向的计数方向 */
            raw_pos = (ENC_SINGLE_MAX - raw_pos) & (ENC_SINGLE_MAX - 1u);
#endif
            s_data.abs_pos = raw_pos;
        }
        s_data.timestamp_cycles = s_request_cycles;
        s_data.valid   = true;
        s_data.update_count++;
        s_last_result  = ENC_OK;
        s_stats.ok_count++;
    }
    return true;
}

void encoder_rs485_trigger(void)
{
    const uint32_t primask = __get_PRIMASK();

    /* Keep DMA arming and transaction publication together. */
    __disable_irq();
    if (s_initialized && !s_paused && s_state == ENC_STATE_IDLE) {
        usart_flag_clear(USART3, ENC_UART_ERRORS | USART_IDLEF_FLAG);
        enc_dma_rx_reload();
        s_request_cycles = motor_timebase_cycles_get();
        enc_send_cmd(ENC_CF_READ_ABS);
        s_stats.request_count++;
        s_state = ENC_STATE_WAIT_RESP;
    }
    __set_PRIMASK(primask);
}

/* USART and ADC use the same preemption priority. Only CH4 can re-arm
 * DMA, and it skips until the entire receive result has been published. */
static void enc_service(bool check_timeout)
{
    const uint32_t primask = __get_PRIMASK();
    uint32_t sts;
    bool waiting;
    bool expired;

    __disable_irq();
    sts = USART3->sts;
    waiting = s_initialized && !s_paused && s_state == ENC_STATE_WAIT_RESP;
    expired = waiting && check_timeout &&
        motor_timebase_cycles_elapsed(s_request_cycles) >= s_timeout_cycles;

    /* IDLE clearing reads DT. Give RX DMA ownership of a pending byte. */
    if (waiting && !expired &&
        (sts & (USART_RDBF_FLAG | ENC_UART_ERRORS)) == USART_RDBF_FLAG) {
        __set_PRIMASK(primask);
        return;
    }
    if ((sts & (ENC_UART_ERRORS | USART_IDLEF_FLAG)) != 0u) {
        usart_flag_clear(USART3, ENC_UART_ERRORS | USART_IDLEF_FLAG);
    }
    __set_PRIMASK(primask);

    if (waiting && enc_receive(sts, expired)) {
        s_state = ENC_STATE_IDLE;
    }
}

void encoder_rs485_usart_irq(void)
{
    enc_service(false);
}

void encoder_rs485_collect(void)
{
    /* ADC can win same-priority arbitration with an IDLE IRQ already pending. */
    if (!s_initialized || s_paused) {
        s_control_data.valid = false;
        return;
    }
    enc_service(true);
    const bool fresh = s_data.valid &&
        s_data.update_count != s_control_data.update_count &&
        motor_timebase_cycles_elapsed(s_data.timestamp_cycles) < s_sample_window_cycles;
    s_control_data = s_data;
    s_control_data.valid = fresh;
}

static bool enc_snapshot_read(const volatile enc_data_t *sample, enc_data_t *out)
{
    uint32_t primask;

    if (out == 0) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *out = *sample;
    __set_PRIMASK(primask);
    return true;
}

bool encoder_rs485_read(enc_data_t *out)
{
    return enc_snapshot_read(&s_data, out);
}

bool encoder_rs485_read_control(enc_data_t *out)
{
    return enc_snapshot_read(&s_control_data, out);
}

const enc_data_t *encoder_rs485_get_data(void)
{
    return (const enc_data_t *)&s_data;
}

enc_result_t encoder_rs485_get_result(void)
{
    return s_last_result;
}

bool encoder_rs485_stats_read(enc_stats_t *out)
{
    uint32_t primask;

    if (out == 0) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *out = *(const enc_stats_t *)&s_stats;
    __set_PRIMASK(primask);
    return true;
}

static void enc_wait_us(uint32_t us)
{
    uint32_t t0 = motor_timebase_cycles_get();
    while (motor_timebase_cycles_to_us(motor_timebase_cycles_elapsed(t0)) < us) {
    }
}

bool encoder_rs485_poll_test(enc_poll_test_t *out)
{
    uint32_t t0;
    uint32_t elapsed_us;
    uint32_t sts;
    uint16_t high_count = 0u;

    if (out == 0) {
        return false;
    }

    /* 先抓取正常运行时的寄存器现场 */
    out->usart_sts   = USART3->sts;
    out->usart_ctrl1 = USART3->ctrl1;
    out->usart_ctrl3 = USART3->ctrl3;
    out->dma_tx_ctrl = DMA1_CHANNEL1->ctrl;
    out->dma_tx_cnt  = DMA1_CHANNEL1->dtcnt;
    out->dma_rx_ctrl = DMA1_CHANNEL2->ctrl;
    out->dma_rx_cnt  = DMA1_CHANNEL2->dtcnt;
    out->rx_len      = 0u;
    out->tx_done     = false;
    out->err_sts     = 0u;

    /* 暂停快环请求，等待可能在途的一帧结束 */
    s_paused = true;
    enc_wait_us(200u);

    NVIC_DisableIRQ(USART3_IRQn);
    dma_channel_enable(DMA1_CHANNEL1, FALSE);
    dma_channel_enable(DMA1_CHANNEL2, FALSE);
    usart_dma_transmitter_enable(USART3, FALSE);
    usart_dma_receiver_enable(USART3, FALSE);

    /* PA6(RO) 空闲电平：正常应为高；常低说明 RE 未使能或总线异常 */
    for (uint16_t i = 0u; i < 100u; i++) {
        if (gpio_input_data_bit_read(GPIOA, GPIO_PINS_6) != RESET) {
            high_count++;
        }
    }
    out->rx_idle_high_pct = (uint8_t)high_count;

    /* 清残留 */
    while (usart_flag_get(USART3, USART_RDBF_FLAG) != RESET) {
        (void)usart_data_receive(USART3);
    }
    usart_flag_clear(USART3, USART_NERR_FLAG | USART_ROERR_FLAG |
                             USART_FERR_FLAG | USART_IDLEF_FLAG);

    t0 = motor_timebase_cycles_get();
    while (usart_flag_get(USART3, USART_TDBE_FLAG) == RESET) {
        if (motor_timebase_cycles_to_us(motor_timebase_cycles_elapsed(t0)) > 100u) {
            break;
        }
    }
    usart_data_transmit(USART3, ENC_CF_READ_ABS);

    t0 = motor_timebase_cycles_get();
    do {
        sts = USART3->sts;
        out->err_sts |= sts & (USART_NERR_FLAG | USART_ROERR_FLAG | USART_FERR_FLAG);
        if (sts & USART_TDC_FLAG) {
            out->tx_done = true;
        }
        if (sts & USART_RDBF_FLAG) {
            uint8_t b = (uint8_t)usart_data_receive(USART3);
            if (out->rx_len < sizeof(out->rx)) {
                out->rx[out->rx_len] = b;
            }
            out->rx_len++;
        }
        elapsed_us = motor_timebase_cycles_to_us(motor_timebase_cycles_elapsed(t0));
    } while (elapsed_us < 200u);

    /* 第二阶段：绕过 USART 接收器，直接高速采样 PA6 电平。
     * 用来区分"线上没有回应"和"线上有回应但 USART RX 没收到（引脚复用错）"。
     * 2.5 Mbps 一位 0.4 us，紧循环采样间隔约几十 ns，足以看到每一位。 */
    {
        const uint32_t cyc_per_100ns = system_core_clock / 10000000u;
        const uint32_t window_cycles = system_core_clock / 1000000u * 80u; /* 80 us */
        uint32_t start, now, dt;
        uint32_t level, prev_level = 1u;

        out->wire_gpioa_cfgr = GPIOA->cfgr;
        out->wire_gpioa_muxl = GPIOA->muxl;
        out->wire_gpioa_pull = GPIOA->pull;
        out->wire_low_samples = 0u;
        out->wire_fall_edges  = 0u;
        out->wire_first_low_100ns = 0u;
        out->wire_last_low_100ns  = 0u;
        out->wire_tx_end_100ns    = 0u;
        out->wire_usart_rx_count  = 0u;

        enc_wait_us(100u);
        while (usart_flag_get(USART3, USART_TDBE_FLAG) == RESET) {
        }
        usart_flag_clear(USART3, USART_TDC_FLAG);
        start = DWT->CYCCNT;
        usart_data_transmit(USART3, ENC_CF_READ_ABS);
        do {
            now   = DWT->CYCCNT;
            level = (GPIOA->idt >> 6) & 1u;
            dt    = now - start;
            if ((out->wire_tx_end_100ns == 0u) &&
                ((USART3->sts & USART_TDC_FLAG) != 0u)) {
                out->wire_tx_end_100ns = dt / cyc_per_100ns;
            }
            /* 只统计发送完成（DE 释放）之后的低电平，发送期间 RO 为高阻 */
            if ((out->wire_tx_end_100ns != 0u) && (level == 0u)) {
                out->wire_low_samples++;
                if (out->wire_first_low_100ns == 0u) {
                    out->wire_first_low_100ns = dt / cyc_per_100ns;
                }
                out->wire_last_low_100ns = dt / cyc_per_100ns;
                if (prev_level != 0u) {
                    out->wire_fall_edges++;
                }
            }
            prev_level = level;
        } while (dt < window_cycles);

        /* 这一轮 USART 可能收到了字节，读掉 */
        while (usart_flag_get(USART3, USART_RDBF_FLAG) != RESET) {
            (void)usart_data_receive(USART3);
            out->wire_usart_rx_count++;
        }
    }

    /* 恢复 DMA 模式 */
    usart_flag_clear(USART3, USART_NERR_FLAG | USART_ROERR_FLAG |
                             USART_FERR_FLAG | USART_IDLEF_FLAG);
    usart_dma_transmitter_enable(USART3, TRUE);
    usart_dma_receiver_enable(USART3, TRUE);
    s_state = ENC_STATE_IDLE;
    NVIC_ClearPendingIRQ(USART3_IRQn);
    NVIC_EnableIRQ(USART3_IRQn);
    s_paused = false;
    return true;
}

uint16_t encoder_abs_to_electrical_angle_q16(uint32_t abs_pos, uint8_t pole_pairs)
{
    return (uint16_t)((abs_pos * pole_pairs) >> (ENC_SINGLE_BITS - 16u));
}

uint16_t encoder_abs_to_angle_q16(uint32_t abs_pos)
{
    /* 131072 步/圈 → 65536，右移 1 位 */
    return (uint16_t)(abs_pos >> 1u);
}
