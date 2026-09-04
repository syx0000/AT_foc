/**
 * @file    flash_port.c
 * @brief   AT32F456 internal Flash read/write implementation
 *
 * Migrated from cubemx_yxsui/Core/Src/flash_port.c
 * Adapted for AT32 BSP, verified against iflytek_joint_module/project/src/fmc_operation.c
 *
 * Key implementation details (from iflytek same-platform reference):
 * - Sector size: 2KB (FLASH_PAGE_SIZE_OTA = 0x800)
 * - flash_word_program for 4-byte writes
 * - __set_PRIMASK(1)/(0) disables interrupts during write to prevent corruption
 * - Clear FLASH_ODF_FLAG | PRGMERR_FLAG | EPPERR_FLAG before each operation
 */
#include "flash_port.h"
#include "at32f45x_flash.h"
#include <string.h>

HAL_StatusTypeDef flash_port_sector_erase(uint32_t addr)
{
    uint32_t primask;
    flash_status_type st;

    if ((addr < FLASH_USER_START_ADDR) || (addr >= FLASH_USER_END_ADDR) ||
        ((addr & (FLASH_SECTOR_SIZE - 1U)) != 0U)) return HAL_ERROR;
    flash_unlock();
    /* Disable interrupts to prevent flash operation interruption */
    primask = __get_PRIMASK();
    __disable_irq();
    flash_flag_clear(FLASH_ODF_FLAG | FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);

    st = flash_sector_erase(addr);

    flash_flag_clear(FLASH_ODF_FLAG | FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
    __set_PRIMASK(primask);
    flash_lock();
    return (st == FLASH_OPERATE_DONE) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef flash_port_write(uint32_t addr, const void *data, uint32_t len)
{
    uint32_t primask;
    /* Address must be 4-byte aligned */
    if (addr & (FLASH_WRITE_GRANULARITY - 1)) return HAL_ERROR;
    /* Must fall within Flash memory range (512KB) */
    if ((data == NULL) || (len == 0U)) return HAL_ERROR;
    if (addr < FLASH_USER_START_ADDR || addr >= FLASH_USER_END_ADDR) return HAL_ERROR;
    if ((len > FLASH_USER_END_ADDR - addr)) return HAL_ERROR;

    flash_unlock();
    /* Disable interrupts during write (per iflytek reference) */
    primask = __get_PRIMASK();
    __disable_irq();
    flash_flag_clear(FLASH_ODF_FLAG | FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);

    HAL_StatusTypeDef st = HAL_OK;
    uint32_t cur_addr = addr;
    uint32_t remaining = len;
    const uint8_t *src = (const uint8_t *)data;

    /* Write 4 bytes at a time */
    while (remaining >= 4) {
        uint32_t word;
        memcpy(&word, src, 4);
        if (flash_word_program(cur_addr, word) != FLASH_OPERATE_DONE) {
            st = HAL_ERROR;
            break;
        }
        cur_addr += 4;
        src += 4;
        remaining -= 4;
    }

    /* Tail: pad with 0xFF if not 4-byte aligned */
    if (st == HAL_OK && remaining > 0) {
        uint8_t tail[4] = {0xFF, 0xFF, 0xFF, 0xFF};
        memcpy(tail, src, remaining);
        uint32_t word;
        memcpy(&word, tail, 4);
        if (flash_word_program(cur_addr, word) != FLASH_OPERATE_DONE) {
            st = HAL_ERROR;
        }
    }

    __set_PRIMASK(primask);
    flash_lock();
    return st;
}

void flash_port_read(uint32_t addr, void *buf, uint32_t len)
{
    memcpy(buf, (const void *)(uintptr_t)addr, len);
}

uint32_t flash_port_crc32(const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;
    while (len--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++) {
            crc = (crc >> 1) ^ (0xEDB88320u & -(int32_t)(crc & 1));
        }
    }
    return ~crc;
}
