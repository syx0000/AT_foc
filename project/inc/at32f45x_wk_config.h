/* add user code begin Header */
/**
  **************************************************************************
  * @file     at32f45x_wk_config.h
  * @brief    header file of work bench config
  **************************************************************************
  * Copyright (c) 2025, Artery Technology, All rights reserved.
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
/* add user code end Header */

/* define to prevent recursive inclusion -----------------------------------*/
#ifndef __AT32F45x_WK_CONFIG_H
#define __AT32F45x_WK_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes -----------------------------------------------------------------------*/
#include "stdio.h"
#include "at32f45x.h"
/* private includes -------------------------------------------------------------*/
/* add user code begin private includes */

/* add user code end private includes */

/* exported types -------------------------------------------------------------*/
/* add user code begin exported types */

/* add user code end exported types */

/* exported constants --------------------------------------------------------*/
/* add user code begin exported constants */

/* add user code end exported constants */

/* exported macro ------------------------------------------------------------*/
/* add user code begin exported macro */

/* add user code end exported macro */

/* add user code begin dma define */
/* user can only modify the dma define value */
#define DMA1_CHANNEL1_BUFFER_SIZE   0
#define DMA1_CHANNEL1_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL1_PERIPHERAL_BASE_ADDR  0

#define DMA1_CHANNEL2_BUFFER_SIZE   0
#define DMA1_CHANNEL2_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL2_PERIPHERAL_BASE_ADDR   0

/* external variables for DMA buffers */
extern uint16_t adc_ordinary_buffer[4];

#define DMA1_CHANNEL3_BUFFER_SIZE   4
#define DMA1_CHANNEL3_MEMORY_BASE_ADDR   (uint32_t)adc_ordinary_buffer
//#define DMA1_CHANNEL3_PERIPHERAL_BASE_ADDR   0

#define DMA1_CHANNEL4_BUFFER_SIZE   0
#define DMA1_CHANNEL4_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL4_PERIPHERAL_BASE_ADDR   0

#define DMA1_CHANNEL5_BUFFER_SIZE   0
#define DMA1_CHANNEL5_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL5_PERIPHERAL_BASE_ADDR   0

//#define DMA1_CHANNEL6_BUFFER_SIZE   0
//#define DMA1_CHANNEL6_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL6_PERIPHERAL_BASE_ADDR   0

//#define DMA1_CHANNEL7_BUFFER_SIZE   0
//#define DMA1_CHANNEL7_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL7_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL1_BUFFER_SIZE   0
//#define DMA2_CHANNEL1_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL1_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL2_BUFFER_SIZE   0
//#define DMA2_CHANNEL2_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL2_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL3_BUFFER_SIZE   0
//#define DMA2_CHANNEL3_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL3_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL4_BUFFER_SIZE   0
//#define DMA2_CHANNEL4_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL4_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL5_BUFFER_SIZE   0
//#define DMA2_CHANNEL5_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL5_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL6_BUFFER_SIZE   0
//#define DMA2_CHANNEL6_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL6_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL7_BUFFER_SIZE   0
//#define DMA2_CHANNEL7_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL7_PERIPHERAL_BASE_ADDR   0
/* add user code end dma define */

/* Private defines -------------------------------------------------------------*/
#define LED_RUN_PIN    GPIO_PINS_13
#define LED_RUN_GPIO_PORT    GPIOC
#define LED_ERR_PIN    GPIO_PINS_14
#define LED_ERR_GPIO_PORT    GPIOC
#define EN_GATE_PIN    GPIO_PINS_15
#define EN_GATE_GPIO_PORT    GPIOC
#define SOA_PIN    GPIO_PINS_0
#define SOA_GPIO_PORT    GPIOA
#define SOB_PIN    GPIO_PINS_1
#define SOB_GPIO_PORT    GPIOA
#define SOC_PIN    GPIO_PINS_2
#define SOC_GPIO_PORT    GPIOA
#define SO3_PIN    GPIO_PINS_3
#define SO3_GPIO_PORT    GPIOA
#define TEMP_MOTOR_PIN    GPIO_PINS_4
#define TEMP_MOTOR_GPIO_PORT    GPIOA
#define TEMP_MOS_PIN    GPIO_PINS_5
#define TEMP_MOS_GPIO_PORT    GPIOA
#define TP_TEST_PIN    GPIO_PINS_11
#define TP_TEST_GPIO_PORT    GPIOB
#define CAN_RES_PIN    GPIO_PINS_3
#define CAN_RES_GPIO_PORT    GPIOH
#define EN_485_PIN    GPIO_PINS_14
#define EN_485_GPIO_PORT    GPIOB
#define CAN_STB_PIN    GPIO_PINS_2
#define CAN_STB_GPIO_PORT    GPIOH

/* exported functions ------------------------------------------------------- */
  /* system clock config. */
  void wk_system_clock_config(void);

  /* config periph clock. */
  void wk_periph_clock_config(void);

  /* nvic config. */
  void wk_nvic_config(void);

/* add user code begin exported functions */

/* add user code end exported functions */

#ifdef __cplusplus
}
#endif

#endif
