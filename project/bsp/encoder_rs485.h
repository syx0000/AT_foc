/**
 ******************************************************************************
 * @file     encoder_rs485.h
 * @brief    锐鹰 SRMA46 系列 RS485 绝对值编码器驱动
 *           协议：私有二进制，2.5Mbps，8N1，半双工
 *           USART3: TX=PB10, RX=PA6, DE=PB14（硬件自动）
 ******************************************************************************
 */
#ifndef ENCODER_RS485_H
#define ENCODER_RS485_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/* 协议常量                                                              */
/* ------------------------------------------------------------------ */
/* Control Field 命令字节 */
#define ENC_CF_READ_ABS       0x02u  /* 读单圈位置: 回 CF+SF+ABS0~2+CRC 共6字节 */
#define ENC_CF_READ_ABM       0x8Au  /* 读多圈位置: CF+SF+ABM0~2+CRC, 回5字节 */
#define ENC_CF_READ_ID        0x92u  /* 读编码器ID: CF+SF+ENID+CRC,   回3字节 */
#define ENC_CF_READ_ALL       0x1Au  /* 读全部信息: CF+SF+8字节数据+CRC, 回10字节 */
#define ENC_CF_FAULT_RESET    0xBAu  /* 故障复位(需连发10次,间隔≥62.5us) */
#define ENC_CF_ABS_ZERO       0xC2u  /* 单圈清零(需连发10次) */
#define ENC_CF_ABM_ZERO       0x62u  /* 多圈清零(需连发10次) */

/* 分辨率 */
#define ENC_SINGLE_BITS       17u
#define ENC_MULTI_BITS        16u
#define ENC_SINGLE_MAX        (1UL << ENC_SINGLE_BITS)   /* 131072 */
#define ENC_MULTI_MAX         (1UL << ENC_MULTI_BITS)    /* 65536  */

/* Status Field 状态位 */
#define ENC_SF_CNT_ERR        (1u << 4)  /* 单圈解算故障 */
#define ENC_SF_MULTI_ERR      (1u << 5)  /* 多圈数据错误 */

/* ALMC 故障位 */
#define ENC_ALMC_OVERSPEED    (1u << 0)
#define ENC_ALMC_CNT_ERR      (1u << 2)
#define ENC_ALMC_MULTI_ERR    (1u << 5)
#define ENC_ALMC_BAT_ERR      (1u << 6)
#define ENC_ALMC_BAT_ALARM    (1u << 7)

/* ------------------------------------------------------------------ */
/* 数据结构                                                              */
/* ------------------------------------------------------------------ */
typedef enum {
    ENC_OK = 0,
    ENC_ERR_TIMEOUT,     /* 等待 DMA/IDLE 超时 */
    ENC_ERR_CRC,         /* CRC 校验失败 */
    ENC_ERR_STATUS,      /* SF 中有故障标志 */
    ENC_ERR_BUSY,        /* 上次传输未完成 */
} enc_result_t;

typedef struct {
    uint32_t abs_pos;    /* 单圈绝对位置，0 ~ 131071 (17 bit) */
    uint16_t multi_turn; /* 多圈圈数，0 ~ 65535 (16 bit) */
    uint8_t  status;     /* Status Field 原始值 */
    uint8_t  almc;       /* ALMC 故障字节（仅 READ_ALL 时有效） */
    bool     valid;      /* 数据有效标志 */
    uint32_t update_count; /* 成功解析的帧计数，用于判断是否有新数据 */
    uint32_t timestamp_cycles; /* 请求发送时刻的 CPU 周期计数 */
} enc_data_t;

typedef struct {
    uint32_t request_count;      /* 已发出请求次数 */
    uint32_t ok_count;           /* 正确帧次数 */
    uint32_t crc_error_count;    /* 收够字节但找不到有效帧 */
    uint32_t timeout_count;      /* 超时未收齐 */
    uint32_t status_error_count; /* SF 报故障 */
    uint8_t  last_rx_len;        /* 最近一次接收字节数 */
    uint8_t  last_raw[8];        /* 最近一次原始接收字节 */
} enc_stats_t;

/** @brief 原子读取编码器数据快照 */
bool encoder_rs485_read(enc_data_t *out);

/** @brief 原子读取通信统计 */
bool encoder_rs485_stats_read(enc_stats_t *out);

/* ------------------------------------------------------------------ */
/* 对外接口                                                              */
/* ------------------------------------------------------------------ */

/**
 * @brief  编码器驱动初始化（在 wk_usart3_init() 之后调用）
 */
void encoder_rs485_init(void);

/** @brief Start the DMA transaction from the PWM compare interrupt. */
void encoder_rs485_trigger(void);

/** @brief Collect the response from the ADC fast loop without waiting. */
void encoder_rs485_collect(void);

/**
 * @brief  获取最新有效数据（无论上次请求是否已完成）
 */
const enc_data_t *encoder_rs485_get_data(void);

/**
 * @brief  查询上次请求结果
 */
enc_result_t encoder_rs485_get_result(void);

/**
 * @brief  将 17bit 单圈位置转换为电角度 [0, 2π) 对应的 Q16 定点数
 *         方便直接送入 FOC 角度输入（电机极对数 = 1 时使用，
 *         多极对数时在外部乘极对数再取模）
 * @param  abs_pos  单圈位置
 * @retval 0 ~ 65535，对应 0 ~ 360°
 */
uint16_t encoder_abs_to_angle_q16(uint32_t abs_pos);

/* 轮询自检：绕过 DMA 发送 1 字节 0x02，200us 内轮询接收 */
typedef struct {
    uint8_t  rx[16];
    uint8_t  rx_len;           /* 实际收到字节数（可大于 16） */
    bool     tx_done;          /* TDC 是否置位 */
    uint8_t  rx_idle_high_pct; /* PA6 空闲时为高的比例，0~100 */
    uint32_t err_sts;          /* 期间出现的 NERR/ROERR/FERR */
    uint32_t usart_sts, usart_ctrl1, usart_ctrl3;
    uint32_t dma_tx_ctrl, dma_tx_cnt, dma_rx_ctrl, dma_rx_cnt;
    /* 线级采样（PA6 GPIO 直读，绕过 USART） */
    uint32_t wire_tx_end_100ns;    /* 发送完成时刻，单位 0.1 us */
    uint32_t wire_low_samples;     /* 发送完成后 PA6 为低的采样数 */
    uint32_t wire_fall_edges;      /* 发送完成后 PA6 下降沿个数 */
    uint32_t wire_first_low_100ns; /* 第一个低电平时刻 */
    uint32_t wire_last_low_100ns;  /* 最后一个低电平时刻 */
    uint32_t wire_usart_rx_count;  /* 同一轮里 USART 收到的字节数 */
    uint32_t wire_gpioa_cfgr, wire_gpioa_muxl, wire_gpioa_pull;
} enc_poll_test_t;

bool encoder_rs485_poll_test(enc_poll_test_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_RS485_H */
