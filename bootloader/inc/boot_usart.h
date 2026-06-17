/**
  * @file     boot_usart.h
  * @brief    Bootloader USART configuration (替代 wk_usart.h)
  */
#ifndef __BOOT_USART_H
#define __BOOT_USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "at32f45x.h"

/* Bootloader使用USART1 (与App调试串口一致) */
#define BOOT_USART              USART1
#define BOOT_USART_BAUD         921600
#define BOOT_USART_CLK          CRM_USART1_PERIPH_CLOCK

/* TX: PA15 (MUX7), RX: PB3 (MUX7) */
#define BOOT_USART_TX_GPIO      GPIOA
#define BOOT_USART_TX_PIN       GPIO_PINS_15
#define BOOT_USART_TX_PIN_SRC   GPIO_PINS_SOURCE15
#define BOOT_USART_TX_MUX       GPIO_MUX_7
#define BOOT_USART_TX_GPIO_CLK  CRM_GPIOA_PERIPH_CLOCK
#define BOOT_USART_RX_GPIO      GPIOB
#define BOOT_USART_RX_PIN       GPIO_PINS_3
#define BOOT_USART_RX_PIN_SRC   GPIO_PINS_SOURCE3
#define BOOT_USART_RX_MUX       GPIO_MUX_7
#define BOOT_USART_RX_GPIO_CLK  CRM_GPIOB_PERIPH_CLOCK
#define BOOT_USART_IRQn         USART1_IRQn

void boot_usart_init(void);
void boot_puts(const char *s);
void boot_put_u32(uint32_t v);
void boot_put_hex(uint32_t v);

#ifdef __cplusplus
}
#endif

#endif
