/**
 * @file    ota_config.h
 * @brief   OTA配置 — Bootloader和App共享
 *          定义内存布局、app_header_t结构、标志位
 *
 * 兼容 cubemx_yxsui Stage 2 OTA 设计:
 *   - app_header_t 结构完全一致
 *   - 相同的 magic / flags 语义
 *   - boot_count 回滚保护
 *
 * AT32F456CEU7 512KB Flash 布局:
 *   0x08000000  Bootloader          14KB
 *   0x08003800  App Header           2KB (app_header_t, 128B有效)
 *   0x08004000  App 执行区          238KB
 *   0x0803F800  Staging Header       2KB
 *   0x08040000  Staging 暂存区      254KB
 *   0x0807F800  User Data (FOC)      2KB (不动)
 */

#ifndef __OTA_CONFIG_H__
#define __OTA_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================================================
 * Flash 分区地址
 * ============================================================ */
#define BOOT_SIZE               0x3800U             /* 14KB Bootloader */
#define APP_HEADER_ADDR         0x08003800U         /* App Header sector (2KB) */
#define APP_START_ADDR          0x08004000U         /* App execute base */
#define APP_MAX_SIZE            0x3B800U            /* 238KB max app size */
#define STAGING_HEADER_ADDR     0x0803F800U         /* Staging Header sector (2KB) */
#define STAGING_START_ADDR      0x08040000U         /* Staging area base */
#define STAGING_MAX_SIZE        0x3F800U            /* 254KB max staging size */
#define USER_DATA_ADDR          0x0807F800U         /* FOC parameter sector (2KB) */
#define FLASH_SECTOR_SIZE       0x800U              /* AT32F456 sector = 2KB */
#define FLASH_PAGE_SIZE_OTA     FLASH_SECTOR_SIZE   /* Legacy alias */

/* ============================================================
 * App Header 结构 (128 字节, 兼容 cubemx H743)
 * ============================================================ */
#define APP_HEADER_MAGIC        0x41434F46U         /* 'FOCA' */
#define APP_FLAG_VALID          0x01U               /* bit0: 固件有效 */
#define APP_FLAG_PENDING        0x02U               /* bit1: 待应用(staging→app) */
#define APP_FLAG_TESTED         0x04U               /* bit2: 已标记稳定 */
#define MAX_BOOT_COUNT          3U                  /* 连续崩溃N次后不再启动 */

typedef struct {
    uint32_t magic;             /* APP_HEADER_MAGIC */
    uint32_t version;           /* 固件版本号 */
    uint32_t app_size;          /* 应用字节数(不含header) */
    uint32_t app_crc32;         /* IEEE 802.3 CRC32 */
    uint32_t boot_count;        /* 启动尝试次数 */
    uint32_t flags;             /* APP_FLAG_xxx */
    uint32_t build_time;        /* Unix时间戳 */
    uint32_t reserved[25];      /* 填充至128字节 */
} app_header_t;

/* 编译断言: header必须128字节 */
_Static_assert(sizeof(app_header_t) == 128, "app_header_t must be 128 bytes");

/* ============================================================
 * IAP Legacy 兼容
 * ============================================================ */
#define IAP_UPGRADE_FLAG_ADDR   APP_HEADER_ADDR     /* 复用同一sector */
#define IAP_UPGRADE_FLAG        0x43575342U         /* 'BWSC' 旧IAP标志 */

#ifdef __cplusplus
}
#endif

#endif /* __OTA_CONFIG_H__ */
