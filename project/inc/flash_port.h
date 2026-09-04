/**
 * @file    flash_port.h
 * @brief   AT32F456 internal Flash read/write API
 *
 * Migrated from cubemx_yxsui/Core/Inc/flash_port.h
 * Adapted to AT32F456 BSP (reference: iflytek_joint_module/project/src/fmc_operation.c)
 *
 * AT32F456CEU7 Flash: 512KB total (0x08000000 - 0x08080000)
 * Sector size: 2KB per sector (verified from iflytek FLASH_PAGE_SIZE_OTA = 0x800)
 * Reserve last two sectors (0x0807F000 - 0x0807FFFF, 4KB) for A/B parameter storage.
 */
#ifndef __FLASH_PORT_H__
#define __FLASH_PORT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "at32f45x.h"

/* AT32F456 Flash sector size (verified: iflytek FLASH_PAGE_SIZE_OTA = 0x800) */
#define FLASH_SECTOR_SIZE       0x800U       /* 2KB */

/* Parameter storage location: last two 2KB sectors of 512KB Flash */
#define FLASH_USER_START_ADDR   0x0807F000U
#define FLASH_USER_END_ADDR     0x08080000U

/* Write granularity: AT32 supports word (4B) program */
#define FLASH_WRITE_GRANULARITY 4U

/* Status return codes (compatible with reference HAL_StatusTypeDef) */
#ifndef HAL_OK
#define HAL_OK      0
#define HAL_ERROR   1
#define HAL_BUSY    2
#define HAL_TIMEOUT 3
#endif

/* Flash status type alias for HAL compatibility */
#ifndef HAL_STATUS_TYPEDEF_DEFINED
#define HAL_STATUS_TYPEDEF_DEFINED
typedef uint32_t HAL_StatusTypeDef;
#endif

/**
 * @brief  Erase the user data sector (2KB at FLASH_USER_START_ADDR)
 * @retval HAL_OK on success, HAL_ERROR on failure
 */
/**
 * @brief 擦除参数区内指定的一个2KB扇区。
 * @param addr 扇区首地址，必须2KB对齐并位于参数区内。
 * @return 成功返回HAL_OK，地址非法或硬件操作失败返回HAL_ERROR。
 */
HAL_StatusTypeDef flash_port_sector_erase(uint32_t addr);

/**
 * @brief  Program data to flash (must be erased first)
 * @param  addr: target address (must be 4-byte aligned, within user region)
 * @param  data: source buffer
 * @param  len:  byte length
 * @retval HAL_OK on success
 */
HAL_StatusTypeDef flash_port_write(uint32_t addr, const void *data, uint32_t len);

/**
 * @brief  Read data from flash (memcpy from absolute address)
 * @param  addr: source flash address
 * @param  buf:  destination buffer
 * @param  len:  byte length
 */
void flash_port_read(uint32_t addr, void *buf, uint32_t len);

/**
 * @brief  Compute IEEE 802.3 CRC32 over a buffer
 * @param  data: input buffer
 * @param  len:  byte length
 * @retval CRC32 result
 */
uint32_t flash_port_crc32(const void *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif
