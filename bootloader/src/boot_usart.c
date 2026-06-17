/**
  * @file     boot_usart.c
  * @brief    Bootloader USART1 (PA15-TX, PB3-RX, 921600) + 轻量打印
  */
#include "boot_usart.h"

void boot_usart_init(void)
{
    gpio_init_type gpio_init_struct;

    crm_periph_clock_enable(BOOT_USART_TX_GPIO_CLK, TRUE);
    crm_periph_clock_enable(BOOT_USART_RX_GPIO_CLK, TRUE);
    crm_periph_clock_enable(BOOT_USART_CLK, TRUE);

    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = BOOT_USART_TX_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(BOOT_USART_TX_GPIO, &gpio_init_struct);
    gpio_pin_mux_config(BOOT_USART_TX_GPIO, BOOT_USART_TX_PIN_SRC, BOOT_USART_TX_MUX);

    gpio_init_struct.gpio_pins = BOOT_USART_RX_PIN;
    gpio_init(BOOT_USART_RX_GPIO, &gpio_init_struct);
    gpio_pin_mux_config(BOOT_USART_RX_GPIO, BOOT_USART_RX_PIN_SRC, BOOT_USART_RX_MUX);

    usart_init(BOOT_USART, BOOT_USART_BAUD, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(BOOT_USART, TRUE);
    usart_receiver_enable(BOOT_USART, TRUE);
    usart_enable(BOOT_USART, TRUE);
}

static void uart_putc(char c)
{
    while (usart_flag_get(BOOT_USART, USART_TDBE_FLAG) == RESET);
    usart_data_transmit(BOOT_USART, (uint8_t)c);
}

void boot_puts(const char *s)
{
    while (*s) uart_putc(*s++);
}

void boot_put_u32(uint32_t v)
{
    char buf[10];
    int i = 0;
    if (v == 0) { uart_putc('0'); return; }
    while (v) { buf[i++] = '0' + (v % 10); v /= 10; }
    while (i--) uart_putc(buf[i]);
}

void boot_put_hex(uint32_t v)
{
    for (int i = 28; i >= 0; i -= 4) {
        uint8_t n = (v >> i) & 0xF;
        uart_putc(n < 10 ? '0' + n : 'A' + n - 10);
    }
}
