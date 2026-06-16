/**
 * @file    ota_app.h
 * @brief   OTA firmware upgrade state machine (App side)
 *
 * 兼容cubemx参考工程的Stage 2 OTA协议:
 *   - App接收固件写入Staging区 (0x08040000)
 *   - 重启后Bootloader验证+复制Staging→App
 *   - 支持CAN-FD和USART1双通道
 *
 * AT32F456CEU7 512KB Flash布局 (ota_config.h):
 *   0x08000000  Bootloader        14KB
 *   0x08003800  App Header         2KB
 *   0x08004000  App Code         238KB
 *   0x0803F800  Staging Header     2KB
 *   0x08040000  Staging Area     254KB
 *   0x0807F800  User Data          2KB
 */
#ifndef __OTA_APP_H__
#define __OTA_APP_H__

#include <stdint.h>

/* 使用bootloader共享的ota_config.h地址定义 */
#include "ota_config.h"

/* OTA chunk size (与上位机protocol.py一致) */
#define OTA_CHUNK_SIZE          256

/* Ring buffer for OTA RX data (2KB, holds ~8 chunks) */
#define OTA_RX_RING_SIZE        2048

/**
 * OTA state machine states.
 */
typedef enum {
    OTA_IDLE = 0,       // No session active
    OTA_ERASING,        // Erasing staging sectors (blocking)
    OTA_RECEIVING,      // Receiving data chunks
    OTA_DONE,           // Success, waiting for otaswap
    OTA_ERROR           // Failure (call ota_abort to reset)
} ota_state_t;

/**
 * Initialize OTA module (call once at startup).
 */
void ota_init(void);

/**
 * Begin OTA session: erase staging area, prepare to receive.
 * Blocks while erasing staging sectors (~127 sectors * ~5ms = ~635ms).
 *
 * @param size      Expected firmware size in bytes (must fit in 254KB)
 * @param crc32     Expected CRC32 (IEEE 802.3) of the entire firmware
 * @param version   Version number (stored in header)
 * @return          0 on success, -1 on error
 */
int ota_begin(uint32_t size, uint32_t crc32, uint32_t version);

/**
 * Feed raw bytes into OTA ring buffer.
 * ISR-safe (called from USART IDLE or CAN RX ISR context).
 */
void ota_rx_feed(const uint8_t *data, uint16_t len);

/**
 * Process accumulated RX data: parse OTA_DATA frames, write to Flash.
 * Call from main loop (non-blocking, processes one frame per call).
 *
 * @return  Bytes written to Flash this call (0 if no complete frame)
 */
uint32_t ota_process(void);

/**
 * Finalize OTA: flush write buffer, verify CRC32, write staging header.
 * @return  0 on success, -1 on CRC mismatch or Flash error
 */
int ota_end(void);

/**
 * Abort OTA session: clear staging header, reset state.
 */
void ota_abort(void);

/**
 * Get current OTA state.
 */
ota_state_t ota_get_state(void);

/**
 * Get progress: bytes written / total expected.
 */
void ota_get_progress(uint32_t *written, uint32_t *total);

/**
 * Clear boot_count in App Header to confirm stable boot.
 * Call once after app is running normally (e.g., 3s after startup).
 * Costs one 2KB sector erase+write.
 */
void ota_mark_self_stable(void);

/* Global flag: 1=USART1 data goes to ota_rx_feed, 0=normal text commands */
extern volatile uint8_t g_ota_rx_mode;

#endif /* __OTA_APP_H__ */
