/**
  **************************************************************************
  * @file     AS5047P.h
  * @brief    Definition and declaration of magnetic encoder(AS5047P) peripheral configuration
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

#ifndef __AS5047P_H
#define __AS5047P_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mc_lib.h"

#if defined AT_MOTOR_EVB_V1 || defined AT_MOTOR_EVB_V2
/* spi interface */
#define MAG_ENCODER_SPI_INTERFACE             SPI1
#define MAG_ENCODER_SPI_CRM_CLK               CRM_SPI1_PERIPH_CLOCK
/* CS pin */
#define MAG_ENCODER_SPI_CS_CRM_CLK            CRM_GPIOB_PERIPH_CLOCK
#define MAG_ENCODER_SPI_CS_GPIO_PORT          GPIOB
#define MAG_ENCODER_SPI_CS_GPIO_PIN           GPIO_PINS_3
#define MAG_ENCODER_SPI_CS_GPIO_PIN_SOURCE    GPIO_PINS_SOURCE3
#define MAG_ENCODER_SPI_CS_IOMUX              GPIO_MUX_3
/* CLK pin */
#define MAG_ENCODER_SPI_CLK_CRM_CLK           CRM_GPIOA_PERIPH_CLOCK
#define MAG_ENCODER_SPI_CLK_GPIO_PORT         GPIOA
#define MAG_ENCODER_SPI_CLK_GPIO_PIN          GPIO_PINS_5
#define MAG_ENCODER_SPI_CLK_GPIO_PIN_SOURCE   GPIO_PINS_SOURCE5
#if defined AT32F421xx || defined AT32F425xx || defined AT32F422xx || defined AT32F426xx
#define MAG_ENCODER_SPI_CLK_IOMUX             GPIO_MUX_0
#elif defined AT32F423xx || defined AT32F435xx || defined AT32F437xx || defined AT32M412xx || defined AT32M416xx || defined AT32F455xx || defined AT32F456xx || defined AT32F457xx || defined AT32F402xx || defined AT32F405xx
#define MAG_ENCODER_SPI_CLK_IOMUX             GPIO_MUX_5
#endif
/* MISO pin */
#define MAG_ENCODER_SPI_MISO_CRM_CLK          CRM_GPIOA_PERIPH_CLOCK
#define MAG_ENCODER_SPI_MISO_GPIO_PORT        GPIOA
#define MAG_ENCODER_SPI_MISO_GPIO_PIN         GPIO_PINS_6
#define MAG_ENCODER_SPI_MISO_GPIO_PIN_SOURCE  GPIO_PINS_SOURCE6
#if defined AT32F421xx || defined AT32F425xx || defined AT32F422xx || defined AT32F426xx
#define MAG_ENCODER_SPI_MISO_IOMUX            GPIO_MUX_0
#elif defined AT32F423xx || defined AT32F435xx || defined AT32F437xx || defined AT32M412xx || defined AT32M416xx || defined AT32F455xx || defined AT32F456xx || defined AT32F457xx || defined AT32F402xx || defined AT32F405xx
#define MAG_ENCODER_SPI_MISO_IOMUX            GPIO_MUX_5
#endif
/* MOSI pin */
#define MAG_ENCODER_SPI_MOSI_CRM_CLK          CRM_GPIOA_PERIPH_CLOCK
#define MAG_ENCODER_SPI_MOSI_GPIO_PORT        GPIOA
#define MAG_ENCODER_SPI_MOSI_GPIO_PIN         GPIO_PINS_7
#define MAG_ENCODER_SPI_MOSI_GPIO_PIN_SOURCE  GPIO_PINS_SOURCE7
#if defined AT32F421xx || defined AT32F425xx || defined AT32F422xx || defined AT32F426xx
#define MAG_ENCODER_SPI_MOSI_IOMUX            GPIO_MUX_0
#elif defined AT32F423xx || defined AT32F435xx || defined AT32F437xx || defined AT32M412xx || defined AT32M416xx || defined AT32F455xx || defined AT32F456xx || defined AT32F457xx || defined AT32F402xx || defined AT32F405xx
#define MAG_ENCODER_SPI_MOSI_IOMUX            GPIO_MUX_5
#endif

#elif defined M412_LV_V1_0

/* spi interface */
#define MAG_ENCODER_SPI_INTERFACE             SPI2
#define MAG_ENCODER_SPI_CRM_CLK               CRM_SPI2_PERIPH_CLOCK
/* CS pin */
#define MAG_ENCODER_SPI_CS_CRM_CLK            CRM_GPIOA_PERIPH_CLOCK
#define MAG_ENCODER_SPI_CS_GPIO_PORT          GPIOA
#define MAG_ENCODER_SPI_CS_GPIO_PIN           GPIO_PINS_15
#define MAG_ENCODER_SPI_CS_GPIO_PIN_SOURCE    GPIO_PINS_SOURCE15
#define MAG_ENCODER_SPI_CS_IOMUX              GPIO_MUX_0
/* CLK pin */
#define MAG_ENCODER_SPI_CLK_CRM_CLK           CRM_GPIOB_PERIPH_CLOCK
#define MAG_ENCODER_SPI_CLK_GPIO_PORT         GPIOB
#define MAG_ENCODER_SPI_CLK_GPIO_PIN          GPIO_PINS_13
#define MAG_ENCODER_SPI_CLK_GPIO_PIN_SOURCE   GPIO_PINS_SOURCE13
#define MAG_ENCODER_SPI_CLK_IOMUX             GPIO_MUX_5
/* MISO pin */
#define MAG_ENCODER_SPI_MISO_CRM_CLK          CRM_GPIOB_PERIPH_CLOCK
#define MAG_ENCODER_SPI_MISO_GPIO_PORT        GPIOB
#define MAG_ENCODER_SPI_MISO_GPIO_PIN         GPIO_PINS_14
#define MAG_ENCODER_SPI_MISO_GPIO_PIN_SOURCE  GPIO_PINS_SOURCE14
#define MAG_ENCODER_SPI_MISO_IOMUX            GPIO_MUX_5
/* MOSI pin */
#define MAG_ENCODER_SPI_MOSI_CRM_CLK          CRM_GPIOB_PERIPH_CLOCK
#define MAG_ENCODER_SPI_MOSI_GPIO_PORT        GPIOB
#define MAG_ENCODER_SPI_MOSI_GPIO_PIN         GPIO_PINS_15
#define MAG_ENCODER_SPI_MOSI_GPIO_PIN_SOURCE  GPIO_PINS_SOURCE15
#define MAG_ENCODER_SPI_MOSI_IOMUX            GPIO_MUX_5

#endif




#define MAG_ENCODER_SPI_CS_ENABLE             gpio_bits_write(MAG_ENCODER_SPI_CS_GPIO_PORT, MAG_ENCODER_SPI_CS_GPIO_PIN, FALSE)
#define MAG_ENCODER_SPI_CS_DISABLE            gpio_bits_write(MAG_ENCODER_SPI_CS_GPIO_PORT, MAG_ENCODER_SPI_CS_GPIO_PIN, TRUE)

/* Magnetic encoder command */
#define READ_ANGLE_VALUE                      (0x3FFE)

void mag_encoder_spi_init(void);
uint16_t get_even_parity_data(uint16_t data);
uint16_t SPI_ReadWriteByte(uint16_t TxData);
uint16_t ReadValue(uint16_t RegValue);
uint16_t ReadAngle(void);

#ifdef __cplusplus
}
#endif

#endif
