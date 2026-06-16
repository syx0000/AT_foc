/**
  **************************************************************************
  * @file     at32f45x_conf.h
  * @brief    at32f45x config header file (Bootloader)
  **************************************************************************
  */

#ifndef __AT32F45x_CONF_H
#define __AT32F45x_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief clock source values
  */
#define HEXT_VALUE        ((uint32_t)8000000)   /* 8MHz external crystal */
#define HICK_VALUE        ((uint32_t)8000000)   /* 8MHz internal RC */
#define HEXT_STARTUP_TIMEOUT  ((uint16_t)0x3000)

/**
  * @brief module enable defines
  */
#define CRM_MODULE_ENABLED
#define FLASH_MODULE_ENABLED
#define GPIO_MODULE_ENABLED
#define USART_MODULE_ENABLED
#define MISC_MODULE_ENABLED
#define PWC_MODULE_ENABLED

/**
  * @brief at32f45x standard peripheral library includes
  */
#include "at32f45x_crm.h"
#include "at32f45x_flash.h"
#include "at32f45x_gpio.h"
#include "at32f45x_usart.h"
#include "at32f45x_misc.h"
#include "at32f45x_pwc.h"

#ifdef __cplusplus
}
#endif

#endif
