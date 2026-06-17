/**
 * @file    ota_app.c
 * @brief   OTA firmware upgrade state machine — AT32F456 adaptation
 *
 * Based on cubemx_yxsui/Core/Src/ota_app.c, adapted:
 *   - Flash write granularity: 4 bytes (vs STM32H7 32 bytes)
 *   - Sector size: 2KB (vs STM32H7 128KB)
 *   - AT32 flash API: flash_sector_erase / flash_word_program
 */
#include "ota_app.h"
#include "flash_port.h"
#include <string.h>
#include <stdio.h>

/* OTA RX mode flag (shared with wk_usart.c / can_debug.c) */
volatile uint8_t g_ota_rx_mode = 0;

/* OTA session state */
static struct {
    ota_state_t state;
    uint32_t expected_size;
    uint32_t expected_crc32;
    uint32_t version;
    uint32_t bytes_written;
    uint16_t next_seq;
    uint8_t  write_buf[4];      /* 4-byte accumulator (AT32 write granularity) */
    uint8_t  write_buf_len;
} g_ota;

/* Ring buffer for RX data */
static struct {
    uint8_t  buf[OTA_RX_RING_SIZE];
    volatile uint16_t head;     /* Write index (ISR) */
    uint16_t tail;              /* Read index (main loop) */
} g_ota_rx;

/* --- CRC16-MODBUS (frame validation) --- */
static uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc >>= 1;
        }
    }
    return crc;
}

/* --- CRC32 IEEE 802.3 (whole-image validation) --- */
static uint32_t crc32_calc(const void *data, uint32_t len)
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

/* --- AT32 Flash helpers --- */
static int flash_erase_region(uint32_t addr, uint32_t size)
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

static int flash_write_words(uint32_t dest, const void *src, uint32_t len)
{
    flash_unlock();
    __set_PRIMASK(1);

    const uint8_t *p = (const uint8_t *)src;
    uint32_t cur = dest;
    uint32_t remaining = len;

    while (remaining >= 4) {
        uint32_t word;
        memcpy(&word, p, 4);
        flash_flag_clear(FLASH_ODF_FLAG | FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
        if (flash_word_program(cur, word) != FLASH_OPERATE_DONE) {
            __set_PRIMASK(0);
            flash_lock();
            return -1;
        }
        cur += 4; p += 4; remaining -= 4;
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
 * Public API
 * ============================================================ */

void ota_init(void)
{
    memset(&g_ota, 0, sizeof(g_ota));
    g_ota.state = OTA_IDLE;
    g_ota_rx.head = 0;
    g_ota_rx.tail = 0;
}

int ota_begin(uint32_t size, uint32_t crc32, uint32_t version)
{
    if (g_ota.state != OTA_IDLE) {
        printf("OTA_ERR already_active\r\n");
        return -1;
    }
    if (size == 0 || size > STAGING_MAX_SIZE) {
        printf("OTA_ERR size_invalid(%u, max=%u)\r\n",
               (unsigned)size, (unsigned)STAGING_MAX_SIZE);
        return -1;
    }

    g_ota.state = OTA_ERASING;
    g_ota.expected_size = size;
    g_ota.expected_crc32 = crc32;
    g_ota.version = version;
    g_ota.bytes_written = 0;
    g_ota.next_seq = 0;
    g_ota.write_buf_len = 0;

    /* Reset RX ring */
    g_ota_rx.head = 0;
    g_ota_rx.tail = 0;

    /* Erase Staging Header + Staging Area */
    uint32_t erase_size = FLASH_SECTOR_SIZE + size;
    /* Round up to sector boundary */
    erase_size = (erase_size + FLASH_SECTOR_SIZE - 1) & ~(FLASH_SECTOR_SIZE - 1);

    printf("OTA: erasing staging %uKB...\r\n", (unsigned)(erase_size / 1024));

    /* Erase staging header sector first */
    if (flash_erase_region(STAGING_HEADER_ADDR, FLASH_SECTOR_SIZE) != 0) {
        printf("OTA_ERR erase_header_failed\r\n");
        g_ota.state = OTA_ERROR;
        return -1;
    }
    /* Erase staging code area */
    uint32_t code_erase = (size + FLASH_SECTOR_SIZE - 1) & ~(FLASH_SECTOR_SIZE - 1);
    if (flash_erase_region(STAGING_START_ADDR, code_erase) != 0) {
        printf("OTA_ERR erase_staging_failed\r\n");
        g_ota.state = OTA_ERROR;
        return -1;
    }

    g_ota.state = OTA_RECEIVING;
    g_ota_rx_mode = 1;
    printf("OTA_READY chunk=%d\r\n", OTA_CHUNK_SIZE);
    return 0;
}

void ota_rx_feed(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (g_ota_rx.head + 1) % OTA_RX_RING_SIZE;
        if (next == g_ota_rx.tail) {
            /* Overflow: discard oldest */
            g_ota_rx.tail = (g_ota_rx.tail + 1) % OTA_RX_RING_SIZE;
        }
        g_ota_rx.buf[g_ota_rx.head] = data[i];
        g_ota_rx.head = next;
    }
}

/* Helper: available bytes in ring */
static uint16_t ring_avail(void)
{
    return (g_ota_rx.head >= g_ota_rx.tail)
           ? (g_ota_rx.head - g_ota_rx.tail)
           : (OTA_RX_RING_SIZE - g_ota_rx.tail + g_ota_rx.head);
}

/* Helper: peek N bytes without consuming */
static uint16_t ring_peek(uint8_t *out, uint16_t len)
{
    if (ring_avail() < len) return 0;
    uint16_t idx = g_ota_rx.tail;
    for (uint16_t i = 0; i < len; i++) {
        out[i] = g_ota_rx.buf[idx];
        idx = (idx + 1) % OTA_RX_RING_SIZE;
    }
    return len;
}

/* Helper: consume N bytes */
static void ring_consume(uint16_t len)
{
    g_ota_rx.tail = (g_ota_rx.tail + len) % OTA_RX_RING_SIZE;
}

uint32_t ota_process(void)
{
    if (g_ota.state != OTA_RECEIVING) return 0;

    /* OTA_DATA frame: 'O' 'D' seq[2] len[2] payload[len] crc16[2] */
    uint8_t header[6];
    if (ring_peek(header, 6) < 6) return 0;

    if (header[0] != 'O' || header[1] != 'D') {
        ring_consume(1);  /* Sync: discard 1 byte */
        return 0;
    }

    uint16_t seq = header[2] | (header[3] << 8);
    uint16_t len = header[4] | (header[5] << 8);

    if (len > OTA_CHUNK_SIZE) {
        printf("OTA_NAK %u bad_len=%u\r\n", seq, len);
        ring_consume(6);
        return 0;
    }

    uint16_t frame_len = 6 + len + 2;
    uint8_t frame_buf[6 + OTA_CHUNK_SIZE + 2];

    if (ring_peek(frame_buf, frame_len) < frame_len) return 0;

    /* Verify CRC16 */
    uint16_t expected_crc = crc16_modbus(frame_buf, 6 + len);
    uint16_t actual_crc = frame_buf[6 + len] | (frame_buf[6 + len + 1] << 8);

    if (actual_crc != expected_crc) {
        printf("OTA_NAK %u crc16 exp=%04X got=%04X len=%u hdr=%02X%02X%02X%02X%02X%02X\r\n",
               seq, expected_crc, actual_crc, len,
               frame_buf[0], frame_buf[1], frame_buf[2],
               frame_buf[3], frame_buf[4], frame_buf[5]);
        ring_consume(frame_len);
        return 0;
    }

    /* Check sequence */
    if (seq != g_ota.next_seq) {
        printf("OTA_NAK %u seq_err(exp=%u)\r\n", seq, g_ota.next_seq);
        ring_consume(frame_len);
        return 0;
    }

    /* Write payload to Flash (4-byte aligned) */
    const uint8_t *payload = &frame_buf[6];
    uint32_t written_this_call = 0;

    for (uint16_t i = 0; i < len; i++) {
        g_ota.write_buf[g_ota.write_buf_len++] = payload[i];

        if (g_ota.write_buf_len == 4) {
            uint32_t addr = STAGING_START_ADDR + g_ota.bytes_written;
            if (flash_write_words(addr, g_ota.write_buf, 4) != 0) {
                printf("OTA_NAK %u flash_err\r\n", seq);
                g_ota.state = OTA_ERROR;
                ring_consume(frame_len);
                return 0;
            }
            g_ota.bytes_written += 4;
            written_this_call += 4;
            g_ota.write_buf_len = 0;
        }
    }

    printf("OTA_ACK %u\r\n", seq);
    g_ota.next_seq = (seq + 1) & 0xFFFF;
    ring_consume(frame_len);

    /* Auto-exit RX mode when all data received */
    uint32_t total_received = g_ota.bytes_written + g_ota.write_buf_len;
    if (total_received >= g_ota.expected_size) {
        g_ota_rx_mode = 0;
        printf("OTA: all data received (%u bytes)\r\n", (unsigned)total_received);
    }

    return written_this_call;
}

int ota_end(void)
{
    if (g_ota.state != OTA_RECEIVING) {
        printf("OTA_FAIL not_receiving\r\n");
        return -1;
    }

    g_ota_rx_mode = 0;

    /* Flush partial write buffer (pad with 0xFF) */
    if (g_ota.write_buf_len > 0) {
        memset(&g_ota.write_buf[g_ota.write_buf_len], 0xFF, 4 - g_ota.write_buf_len);
        uint32_t addr = STAGING_START_ADDR + g_ota.bytes_written;
        if (flash_write_words(addr, g_ota.write_buf, 4) != 0) {
            printf("OTA_FAIL flush_err\r\n");
            g_ota.state = OTA_ERROR;
            return -1;
        }
        g_ota.bytes_written += g_ota.write_buf_len;
        g_ota.write_buf_len = 0;
    }

    /* Verify whole-image CRC32 */
    printf("OTA: verifying CRC32 (%u bytes)...\r\n", (unsigned)g_ota.expected_size);
    uint32_t actual_crc = crc32_calc((const void *)STAGING_START_ADDR, g_ota.expected_size);

    if (actual_crc != g_ota.expected_crc32) {
        printf("OTA_FAIL crc32 exp=0x%08X got=0x%08X\r\n",
               (unsigned)g_ota.expected_crc32, (unsigned)actual_crc);
        g_ota.state = OTA_ERROR;
        return -1;
    }

    /* Write Staging Header with PENDING flag */
    app_header_t hdr;
    memset(&hdr, 0xFF, sizeof(hdr));
    hdr.magic      = APP_HEADER_MAGIC;
    hdr.version    = g_ota.version;
    hdr.app_size   = g_ota.expected_size;
    hdr.app_crc32  = g_ota.expected_crc32;
    hdr.boot_count = 0;
    hdr.flags      = APP_FLAG_VALID | APP_FLAG_PENDING;
    hdr.build_time = 0;

    if (flash_write_words(STAGING_HEADER_ADDR, &hdr, sizeof(hdr)) != 0) {
        printf("OTA_FAIL hdr_write_err\r\n");
        g_ota.state = OTA_ERROR;
        return -1;
    }

    g_ota.state = OTA_DONE;
    printf("OTA_DONE size=%u crc=0x%08X ver=%u\r\n",
           (unsigned)g_ota.bytes_written, (unsigned)actual_crc, (unsigned)g_ota.version);
    printf("OTA: use 'otaswap' to reboot and apply\r\n");
    return 0;
}

void ota_abort(void)
{
    g_ota_rx_mode = 0;

    /* Erase staging header to invalidate */
    flash_erase_region(STAGING_HEADER_ADDR, FLASH_SECTOR_SIZE);

    g_ota.state = OTA_IDLE;
    g_ota.bytes_written = 0;
    g_ota.next_seq = 0;
    g_ota.write_buf_len = 0;
    printf("OTA aborted\r\n");
}

ota_state_t ota_get_state(void)
{
    return g_ota.state;
}

void ota_get_progress(uint32_t *written, uint32_t *total)
{
    *written = g_ota.bytes_written;
    *total = g_ota.expected_size;
}

void ota_mark_self_stable(void)
{
    const app_header_t *cur = (const app_header_t *)APP_HEADER_ADDR;
    if (cur->magic != APP_HEADER_MAGIC) return;
    if (cur->boot_count == 0) return;

    /* Rewrite header with boot_count=0 */
    app_header_t fresh;
    memcpy(&fresh, cur, sizeof(fresh));
    fresh.boot_count = 0;
    fresh.flags |= APP_FLAG_TESTED;

    if (flash_erase_region(APP_HEADER_ADDR, FLASH_SECTOR_SIZE) != 0) {
        printf("BOOT: mark_stable erase_err\r\n");
        return;
    }
    if (flash_write_words(APP_HEADER_ADDR, &fresh, sizeof(fresh)) != 0) {
        printf("BOOT: mark_stable write_err\r\n");
        return;
    }
    printf("BOOT: marked stable (ver=%u)\r\n", (unsigned)fresh.version);
}
