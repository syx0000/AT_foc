/**
 * @file    main.c
 * @brief   AT32F456 Bootloader — OTA Stage 2 + IAP兜底
 *
 * 启动流程:
 *   1. 检查 Staging Header: PENDING? → 验证CRC → 复制到App区 → 写App Header
 *   2. 检查 App Header: magic+VALID+CRC OK? → boot_count<3? → 跳转
 *   3. 以上都失败 → 初始化USART1 → 进入IAP串口升级模式
 *
 * 兼容 cubemx_yxsui Stage 2 OTA 协议 (app_header_t)
 */

#include "at32f45x.h"
#include "ota_config.h"
#include "flash.h"
#include "iap.h"
#include "boot_usart.h"
#include <stdio.h>
#include <string.h>

/* ============================================================
 * CRC32 (IEEE 802.3, 与 flash_port.c / boot_crc32.c 相同算法)
 * ============================================================ */
static uint32_t boot_crc32(const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;
    while (len--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320u;
            else         crc >>= 1;
        }
    }
    return ~crc;
}

/* ============================================================
 * 系统时钟配置 (与FOC工程 wk_system_clock_config 一致)
 * HSE 8MHz → PLL → 192MHz
 * ============================================================ */
static void system_clock_config(void)
{
    crm_reset();
    flash_psr_set(FLASH_WAIT_CYCLE_5);

    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
    pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);

    crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
    while (crm_hext_stable_wait() == ERROR);

    crm_pll_config(CRM_PLL_SOURCE_HEXT, 144, 1, CRM_PLL_FP_6);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
    while (crm_flag_get(CRM_PLL_STABLE_FLAG) != SET);

    crm_ahb_div_set(CRM_AHB_DIV_1);
    crm_apb1_div_set(CRM_APB1_DIV_1);
    crm_apb2_div_set(CRM_APB2_DIV_1);

    crm_auto_step_mode_enable(TRUE);
    crm_sysclk_switch(CRM_SCLK_PLL);
    while (crm_sysclk_switch_status_get() != CRM_SCLK_PLL);
    crm_auto_step_mode_enable(FALSE);

    system_core_clock_update();
}

/* ============================================================
 * App Header 校验
 * ============================================================ */
static int header_valid(const app_header_t *h, uint32_t app_base)
{
    if (h->magic != APP_HEADER_MAGIC) return 0;
    if (!(h->flags & APP_FLAG_VALID)) return 0;
    if (h->app_size == 0 || h->app_size > APP_MAX_SIZE) return 0;

    /* CRC32 校验整个App区域 */
    uint32_t crc = boot_crc32((const void *)app_base, h->app_size);
    return (crc == h->app_crc32);
}

/* ============================================================
 * Flash 扇区擦除 (连续多扇区)
 * ============================================================ */
static int erase_region(uint32_t addr, uint32_t size)
{
    flash_unlock();
    __set_PRIMASK(1);

    uint32_t end = addr + size;
    while (addr < end) {
        flash_flag_clear(FLASH_ODF_FLAG | FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
        if (flash_sector_erase(addr) != FLASH_OPERATE_DONE) {
            __set_PRIMASK(0);
            flash_lock();
            return -1;
        }
        addr += FLASH_SECTOR_SIZE;
    }

    __set_PRIMASK(0);
    flash_lock();
    return 0;
}

/* ============================================================
 * Flash 块写入 (从RAM复制到Flash)
 * ============================================================ */
static int program_block(uint32_t dest, const void *src, uint32_t len)
{
    flash_unlock();
    __set_PRIMASK(1);

    const uint8_t *p = (const uint8_t *)src;
    uint32_t remaining = len;
    uint32_t cur = dest;

    while (remaining >= 4) {
        uint32_t word;
        memcpy(&word, p, 4);
        flash_flag_clear(FLASH_ODF_FLAG | FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
        if (flash_word_program(cur, word) != FLASH_OPERATE_DONE) {
            __set_PRIMASK(0);
            flash_lock();
            return -1;
        }
        cur += 4;
        p += 4;
        remaining -= 4;
    }
    if (remaining > 0) {
        uint8_t tail[4] = {0xFF, 0xFF, 0xFF, 0xFF};
        memcpy(tail, p, remaining);
        uint32_t word;
        memcpy(&word, tail, 4);
        flash_flag_clear(FLASH_ODF_FLAG | FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
        if (flash_word_program(cur, word) != FLASH_OPERATE_DONE) {
            __set_PRIMASK(0);
            flash_lock();
            return -1;
        }
    }

    __set_PRIMASK(0);
    flash_lock();
    return 0;
}

/* ============================================================
 * 尝试应用 Staging 区的待更新固件
 * ============================================================ */
static int try_apply_pending(void)
{
    /* 读取 Staging Header (在擦除前复制到RAM) */
    app_header_t snap;
    memcpy(&snap, (const void *)STAGING_HEADER_ADDR, sizeof(snap));

    /* 检查 PENDING 标志 */
    if (snap.magic != APP_HEADER_MAGIC) return 0;
    if (!(snap.flags & APP_FLAG_PENDING)) return 0;
    if (snap.app_size == 0 || snap.app_size > STAGING_MAX_SIZE) return 0;

    printf("BOOT: staging detected, size=%u, verifying CRC...\r\n",
           (unsigned)snap.app_size);

    /* 验证 Staging 区 CRC32 */
    uint32_t crc = boot_crc32((const void *)STAGING_START_ADDR, snap.app_size);
    if (crc != snap.app_crc32) {
        printf("BOOT: staging CRC mismatch (got 0x%08lX, expect 0x%08lX), discard\r\n",
               (unsigned long)crc, (unsigned long)snap.app_crc32);
        /* 清除 Staging Header 防止重复尝试 */
        erase_region(STAGING_HEADER_ADDR, FLASH_SECTOR_SIZE);
        return 0;
    }

    printf("BOOT: CRC OK, applying update...\r\n");

    /* 擦除 App 区 (App Header + App 代码) */
    uint32_t app_erase_size = FLASH_SECTOR_SIZE + snap.app_size;
    /* 向上对齐到扇区边界 */
    app_erase_size = (app_erase_size + FLASH_SECTOR_SIZE - 1) & ~(FLASH_SECTOR_SIZE - 1);
    if (erase_region(APP_HEADER_ADDR, app_erase_size) != 0) {
        printf("BOOT: app erase FAILED\r\n");
        return -1;
    }

    /* 复制 Staging → App */
    if (program_block(APP_START_ADDR, (const void *)STAGING_START_ADDR, snap.app_size) != 0) {
        printf("BOOT: app program FAILED\r\n");
        return -1;
    }

    /* 验证复制后的 CRC */
    uint32_t verify_crc = boot_crc32((const void *)APP_START_ADDR, snap.app_size);
    if (verify_crc != snap.app_crc32) {
        printf("BOOT: post-copy CRC FAILED\r\n");
        return -1;
    }

    /* 写入新的 App Header */
    app_header_t fresh;
    memset(&fresh, 0xFF, sizeof(fresh));
    fresh.magic      = APP_HEADER_MAGIC;
    fresh.version    = snap.version;
    fresh.app_size   = snap.app_size;
    fresh.app_crc32  = snap.app_crc32;
    fresh.boot_count = 0;
    fresh.flags      = APP_FLAG_VALID;
    fresh.build_time = snap.build_time;

    if (program_block(APP_HEADER_ADDR, &fresh, sizeof(fresh)) != 0) {
        printf("BOOT: header write FAILED\r\n");
        return -1;
    }

    /* 清除 Staging Header (防止重复应用) */
    erase_region(STAGING_HEADER_ADDR, FLASH_SECTOR_SIZE);

    printf("BOOT: update applied OK, ver=%u\r\n", (unsigned)fresh.version);
    return 1;
}

/* ============================================================
 * 跳转到 App
 * ============================================================ */
__attribute__((noreturn))
static void jump_to_app(uint32_t app_base)
{
    /* 禁用USART (如果初始化过) */
    crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, FALSE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, FALSE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, FALSE);

    /* 关闭 SysTick */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    /* 禁用所有中断 */
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    /* 设置 VTOR */
    SCB->VTOR = app_base;
    __DSB();
    __ISB();

    /* 跳转 */
    uint32_t sp = *(volatile uint32_t *)app_base;
    uint32_t pc = *(volatile uint32_t *)(app_base + 4);

    __set_MSP(sp);
    ((void (*)(void))pc)();

    while (1); /* never reached */
}

/* ============================================================
 * 主函数
 * ============================================================ */
int main(void)
{
    system_clock_config();
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

    /* 初始化USART用于调试输出 (OTA流程可能需要打印) */
    boot_usart_init();
    printf("\r\nBOOT: AT32F456 OTA Bootloader v2.0\r\n");

    /* ============================================================
     * 阶段1: 检查 Staging 区是否有待应用的 OTA 固件
     * ============================================================ */
    int apply_result = try_apply_pending();
    if (apply_result < 0) {
        printf("BOOT: apply failed, trying existing app...\r\n");
    }

    /* ============================================================
     * 阶段2: 检查 App Header 有效性
     * ============================================================ */
    const app_header_t *ah = (const app_header_t *)APP_HEADER_ADDR;

    if (header_valid(ah, APP_START_ADDR)) {
        /* App有效，检查 boot_count */
        if (ah->boot_count >= MAX_BOOT_COUNT) {
            printf("BOOT: boot_count=%u >= %u, app unstable! Entering IAP...\r\n",
                   (unsigned)ah->boot_count, MAX_BOOT_COUNT);
            /* 不跳转，进入IAP */
        } else {
            /* boot_count++, 写回 header */
            printf("BOOT: app valid, ver=%u, count=%u->%u, jumping...\r\n",
                   (unsigned)ah->version, (unsigned)ah->boot_count,
                   (unsigned)(ah->boot_count + 1));

            app_header_t updated;
            memcpy(&updated, ah, sizeof(updated));
            updated.boot_count++;

            erase_region(APP_HEADER_ADDR, FLASH_SECTOR_SIZE);
            program_block(APP_HEADER_ADDR, &updated, sizeof(updated));

            jump_to_app(APP_START_ADDR);
            /* never returns */
        }
    } else {
        /* ============================================================
         * 兼容无header的旧固件 (开发模式: 调试器直接烧写App区)
         * 检查向量表: 复位地址在0x08xxxxxx范围内
         * ============================================================ */
        uint32_t reset_vector = *(volatile uint32_t *)(APP_START_ADDR + 4);
        uint32_t stack_ptr    = *(volatile uint32_t *)APP_START_ADDR;

        if ((reset_vector & 0xFF000000) == 0x08000000 &&
            stack_ptr >= 0x20000000 && stack_ptr <= 0x20020000) {
            printf("BOOT: no valid header, but vector table OK (dev mode), jumping...\r\n");
            jump_to_app(APP_START_ADDR);
        }

        printf("BOOT: no valid app found\r\n");
    }

    /* ============================================================
     * 阶段3: IAP 串口升级模式
     * ============================================================ */
    printf("BOOT: entering IAP mode (USART1 @ 921600)...\r\n");
    iap_init();
    while (1) {
        iap_upgrade_app_handle();
    }
}
