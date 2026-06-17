/**
 * @file    main.c
 * @brief   AT32F456 Bootloader — OTA Stage 2 only
 *
 * 启动流程:
 *   1. 检查 Staging Header: PENDING? → 验证CRC → 复制到App区 → 写App Header
 *   2. 检查 App Header: magic+VALID+CRC OK? → boot_count<3? → 跳转
 *   3. 兼容dev mode: 无header但向量表合法 → 跳转
 *   4. 以上都失败 → 死循环等待烧录器重新烧写
 *
 * 兼容 cubemx_yxsui Stage 2 OTA 协议 (app_header_t)
 */

#include "at32f45x.h"
#include "ota_config.h"
#include "flash.h"
#include "boot_usart.h"
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

    boot_puts("BOOT: staging sz="); boot_put_u32(snap.app_size); boot_puts("\r\n");

    /* 验证 Staging 区 CRC32 */
    uint32_t crc = boot_crc32((const void *)STAGING_START_ADDR, snap.app_size);
    if (crc != snap.app_crc32) {
        /* 清除 Staging Header 防止重复尝试 */
        erase_region(STAGING_HEADER_ADDR, FLASH_SECTOR_SIZE);
        return 0;
    }

    /* 擦除 App 区 (App Header + App 代码) */
    uint32_t app_erase_size = FLASH_SECTOR_SIZE + snap.app_size;
    /* 向上对齐到扇区边界 */
    app_erase_size = (app_erase_size + FLASH_SECTOR_SIZE - 1) & ~(FLASH_SECTOR_SIZE - 1);
    if (erase_region(APP_HEADER_ADDR, app_erase_size) != 0) {
        return -1;
    }

    /* 复制 Staging → App */
    if (program_block(APP_START_ADDR, (const void *)STAGING_START_ADDR, snap.app_size) != 0) {
        return -1;
    }

    /* 验证复制后的 CRC */
    uint32_t verify_crc = boot_crc32((const void *)APP_START_ADDR, snap.app_size);
    if (verify_crc != snap.app_crc32) {
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
        return -1;
    }

    /* 清除 Staging Header (防止重复应用) */
    erase_region(STAGING_HEADER_ADDR, FLASH_SECTOR_SIZE);

    boot_puts("BOOT: updated v"); boot_put_u32(fresh.version); boot_puts("\r\n");
    return 1;
}

/* 函数指针类型定义 */
typedef void (*app_func_t)(void);

/* ============================================================
 * 跳转到 App (内联汇编，最可靠)
 * ============================================================ */
__attribute__((noreturn))
static void jump_to_app(uint32_t app_base)
{
    uint32_t sp = *(volatile uint32_t *)app_base;
    uint32_t pc = *(volatile uint32_t *)(app_base + 4);

    /* 栈指针有效性检查 */
    if ((sp - 0x20000000) > (144 * 1024)) {
        boot_puts("BOOT: bad SP\r\n");
        while (1);
    }

    /* 等待UART发送完成 */
    while (usart_flag_get(BOOT_USART, USART_TDC_FLAG) == RESET);

    /* 关闭用到的外设 */
    crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, FALSE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, FALSE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, FALSE);

    /* 禁用USART1中断 */
    nvic_irq_disable(USART1_IRQn);
    NVIC_ClearPendingIRQ(USART1_IRQn);

    /* 内联汇编跳转 (ARM Thumb-2) */
    __asm volatile (
        "MSR MSP, %0    \n"  /* 设置主栈指针 */
        "BX  %1         \n"  /* 跳转到Reset_Handler */
        : : "r" (sp), "r" (pc)
    );

    while (1);
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
    boot_puts("\r\nBOOT v2.0\r\n");

    /* ============================================================
     * 阶段1: 检查 Staging 区是否有待应用的 OTA 固件
     * ============================================================ */
    int apply_result = try_apply_pending();
    if (apply_result < 0) {
        /* apply failed, fall through to existing app check */
    }

    /* ============================================================
     * 阶段2: 检查 App Header 有效性
     * ============================================================ */
    const app_header_t *ah = (const app_header_t *)APP_HEADER_ADDR;

    if (header_valid(ah, APP_START_ADDR)) {
        /* App有效，检查 boot_count */
        if (ah->boot_count >= MAX_BOOT_COUNT) {
            boot_puts("BOOT: unstable!\r\n");
            /* 不跳转 */
        } else {
            /* boot_count++, 写回 header */
            boot_puts("BOOT: jump v"); boot_put_u32(ah->version); boot_puts("\r\n");

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
            boot_puts("BOOT: dev mode\r\n");
            jump_to_app(APP_START_ADDR);
        }
    }

    /* ============================================================
     * 阶段3: 无有效App，死循环等待烧录器重新烧写
     * ============================================================ */
    boot_puts("BOOT: no app!\r\n");
    while (1) {
        /* 等待调试器烧写或硬件复位 */
    }
}
