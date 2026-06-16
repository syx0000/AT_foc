/**
  * @file     boot_usart.c
  * @brief    Bootloader USART1 initialization (PA15-TX, PB3-RX, 921600)
  */
#include <stdio.h>
#include "boot_usart.h"

void boot_usart_init(void)
{
    gpio_init_type gpio_init_struct;

    /* Enable clocks */
    crm_periph_clock_enable(BOOT_USART_TX_GPIO_CLK, TRUE);
    crm_periph_clock_enable(BOOT_USART_RX_GPIO_CLK, TRUE);
    crm_periph_clock_enable(BOOT_USART_CLK, TRUE);

    /* TX pin: PA15, MUX7 */
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = BOOT_USART_TX_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(BOOT_USART_TX_GPIO, &gpio_init_struct);
    gpio_pin_mux_config(BOOT_USART_TX_GPIO, BOOT_USART_TX_PIN_SRC, BOOT_USART_TX_MUX);

    /* RX pin: PB3, MUX7 */
    gpio_init_struct.gpio_pins = BOOT_USART_RX_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(BOOT_USART_RX_GPIO, &gpio_init_struct);
    gpio_pin_mux_config(BOOT_USART_RX_GPIO, BOOT_USART_RX_PIN_SRC, BOOT_USART_RX_MUX);

    /* USART config: 921600 8N1 */
    usart_init(BOOT_USART, BOOT_USART_BAUD, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(BOOT_USART, TRUE);
    usart_receiver_enable(BOOT_USART, TRUE);

    usart_enable(BOOT_USART, TRUE);
}

/* printf重定向 */
int fputc(int ch, FILE *f)
{
    (void)f;
    while(usart_flag_get(BOOT_USART, USART_TDBE_FLAG) == RESET);
    usart_data_transmit(BOOT_USART, (uint8_t)ch);
    return ch;
}
