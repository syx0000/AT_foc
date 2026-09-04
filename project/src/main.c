/* add user code begin Header */
/**
  **************************************************************************
  * @file     main.c
  * @brief    main program
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

/* Includes ------------------------------------------------------------------*/
#include "at32f45x_wk_config.h"
#include "wk_adc.h"
#include "wk_can.h"
#include "wk_exint.h"
#include "wk_tmr.h"
#include "wk_usart.h"
#include "wk_dma.h"
#include "wk_gpio.h"
#include "wk_system.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#include <stdio.h>
#include "product_version.h"
#include "motor_log.h"
#include "interrupt_monitor.h"
#include "motor_pwm_port.h"
#include "motor_board_config.h"
#include "motor_timebase.h"
#include "motor_adc_port.h"
#include "motor_current_calibration.h"
#include "motor_current_sample.h"
#include "motor_slow_sensor.h"
#include "motor_open_loop.h"
#include "motor_hall_port.h"
#include "motor_hall_decoder.h"
#include "motor_hall_angle_observer.h"
#include "motor_hall_angle_estimator.h"
#include "motor_speed_feedback.h"
#include "motor_speed_control.h"
#include "motor_current_transform.h"
#include "motor_current_control.h"
#include "motor_uart_port.h"
#include "motor_cli.h"
#include "motor_control.h"
#include "motor_parameter.h"

/* add user code end private includes */

/* private typedef -----------------------------------------------------------*/
/* add user code begin private typedef */

/* add user code end private typedef */

/* private define ------------------------------------------------------------*/
/* add user code begin private define */

/* add user code end private define */

/* private macro -------------------------------------------------------------*/
/* add user code begin private macro */

/* add user code end private macro */

/* private variables ---------------------------------------------------------*/
/* add user code begin private variables */
/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */
static bool motor_driver_prepare_for_calibration(void);

/* add user code end function prototypes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/**
 * @brief 唤醒门极驱动器并准备电流零偏校准条件。
 * @param 无。
 * @return true表示nFAULT已恢复、BIF已清除且PWM仍关闭；false表示驱动器
 *         未就绪，函数已执行紧急关断。
 * @details 先强制关闭PWM输出，再拉高EN_GATE并等待DRV8353及电流放大器稳定，
 *          随后检查nFAULT并清除TMR1刹车标志。该函数不会开启MOE或输出PWM。
 */
static bool motor_driver_prepare_for_calibration(void)
{
  bool fault_ready;

  motor_pwm_port_output_disable();
  motor_pwm_port_gate_driver_set(true);
  wk_delay_ms(MOTOR_GATE_DRIVER_WAKE_DELAY_MS);
  fault_ready = motor_pwm_port_fault_clear();

  if ((!fault_ready) ||
      (MOTOR_PWM_TIMER->brk_bit.oen != 0U))
  {
    motor_pwm_port_emergency_stop();
    return false;
  }

  return true;
}

/* add user code end 0 */

/**
  * @brief main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  /* add user code begin 1 */
  motor_parameter_t motor_parameter;
  motor_hall_port_sample_t hall_port_sample;

  /* add user code end 1 */

  /* system clock config. */
  wk_system_clock_config();

  /* config periph clock. */
  wk_periph_clock_config();

  /**
   * users need add interrupt handler code into the below function in the at32f45x_int.c file.
   *  --void SystTick_IRQHandler(void)
   */
  systick_interrupt_config(1000);

  /* nvic config. */
  wk_nvic_config();

  /* timebase config for
     void wk_delay_us(uint32_t delay);
     void wk_delay_ms(uint32_t delay); */
  wk_timebase_init();
  motor_timebase_init();
  motor_parameter_init();
  motor_adc_port_init();
  motor_current_sample_init();
  motor_slow_sensor_init();
  motor_open_loop_init();
  motor_hall_angle_observer_init();
  motor_hall_angle_estimator_init();
  motor_speed_feedback_init();
  motor_speed_control_init();
  motor_current_transform_init();
  motor_current_control_init();
  motor_control_init();

  /* init gpio function. */
  wk_gpio_config();

  /* init dma1 channel3 */
  wk_dma1_channel3_init();
  /* config dma channel transfer parameter */
  /* user need to modify define values DMAx_CHANNELy_XXX_BASE_ADDR 
     and DMAx_CHANNELy_BUFFER_SIZE in at32xxx_wk_config.h */
  wk_dma_channel_config(DMA1_CHANNEL3, 
                        (uint32_t)&ADCCOM->codt, 
                        DMA1_CHANNEL3_MEMORY_BASE_ADDR, 
                        DMA1_CHANNEL3_BUFFER_SIZE);
  dma_channel_enable(DMA1_CHANNEL3, TRUE);

  /* init tmr1 function. */
  wk_tmr1_init();
  motor_pwm_port_init();

  /* init adc-common function. */
  wk_adc_common_init();

  /* init adc1 function. */
  wk_adc1_init();

  /* init adc2 function. */
  wk_adc2_init();

  /* init tmr6 function. */
  wk_tmr6_init();

  /* usart1 already supports printf. */
  /* init usart1 function. */
  wk_usart1_init();

  /* init usart3 function. */
  wk_usart3_init();

  /* init can1 function. */
  wk_can1_init();

  /* init dma1 channel1 */
  wk_dma1_channel1_init();
  /* config dma channel transfer parameter */
  /* user need to modify define values DMAx_CHANNELy_XXX_BASE_ADDR 
     and DMAx_CHANNELy_BUFFER_SIZE in at32xxx_wk_config.h */
  wk_dma_channel_config(DMA1_CHANNEL1, 
                        (uint32_t)&USART3->dt, 
                        DMA1_CHANNEL1_MEMORY_BASE_ADDR, 
                        DMA1_CHANNEL1_BUFFER_SIZE);
  dma_channel_enable(DMA1_CHANNEL1, TRUE);

  /* init dma1 channel2 */
  wk_dma1_channel2_init();
  /* config dma channel transfer parameter */
  /* user need to modify define values DMAx_CHANNELy_XXX_BASE_ADDR 
     and DMAx_CHANNELy_BUFFER_SIZE in at32xxx_wk_config.h */
  wk_dma_channel_config(DMA1_CHANNEL2, 
                        (uint32_t)&USART3->dt, 
                        DMA1_CHANNEL2_MEMORY_BASE_ADDR, 
                        DMA1_CHANNEL2_BUFFER_SIZE);
  dma_channel_enable(DMA1_CHANNEL2, TRUE);

  /* init dma1 channel4 */
  wk_dma1_channel4_init();
  /* config dma channel transfer parameter */
  /* user need to modify define values DMAx_CHANNELy_XXX_BASE_ADDR 
     and DMAx_CHANNELy_BUFFER_SIZE in at32xxx_wk_config.h */
  wk_dma_channel_config(DMA1_CHANNEL4, 
                        (uint32_t)&USART1->dt, 
                        DMA1_CHANNEL4_MEMORY_BASE_ADDR, 
                        DMA1_CHANNEL4_BUFFER_SIZE);
  dma_channel_enable(DMA1_CHANNEL4, TRUE);

  /* init dma1 channel5 */
  wk_dma1_channel5_init();
  /* config dma channel transfer parameter */
  /* user need to modify define values DMAx_CHANNELy_XXX_BASE_ADDR 
     and DMAx_CHANNELy_BUFFER_SIZE in at32xxx_wk_config.h */
  wk_dma_channel_config(DMA1_CHANNEL5, 
                        (uint32_t)&USART1->dt, 
                        DMA1_CHANNEL5_MEMORY_BASE_ADDR, 
                        DMA1_CHANNEL5_BUFFER_SIZE);
  dma_channel_enable(DMA1_CHANNEL5, TRUE);

  /* init exint function. */
  wk_exint_config();
  motor_hall_port_init();
  (void)motor_parameter_active_read(&motor_parameter);
  (void)motor_hall_port_sample_read(&hall_port_sample);
  if (!motor_hall_decoder_init(motor_parameter.hall_positive_next,
                               system_core_clock,
                               hall_port_sample.state,
                               hall_port_sample.timestamp_cycles))
  {
    LOGE("Hall decoder init failed\r\n");
  }

  /* add user code begin 2 */
  motor_uart_port_init();
  LOGI("AT_foc_hall\r\n");
  LOGI("FW: %s\r\n", FIRMWARE_VERSION_STRING);
  LOGI("HW: %s\r\n", HARDWARE_VERSION_STRING);
  LOGI("APP: OK\r\n");
  if (motor_driver_prepare_for_calibration())
  {
    motor_current_calibration_result_t calibration_result;

    LOGI("DRV calibration ready: nFAULT=1 BIF=0 MOE=0\r\n");
    LOGI("ADC calibration: started samples=%lu\r\n",
         (unsigned long)MOTOR_CURRENT_CALIBRATION_DEFAULT_SAMPLES);
    if (motor_current_calibration_run(
          MOTOR_CURRENT_CALIBRATION_DEFAULT_SAMPLES,
          500U,
          &calibration_result))
    {
      if (motor_current_sample_offsets_set(
            calibration_result.phase_a_offset_raw,
            calibration_result.phase_b_offset_raw))
      {
        LOGI("ADC calibration: offset_a=%u offset_b=%u samples=%lu valid=1\r\n",
             (unsigned int)calibration_result.phase_a_offset_raw,
             (unsigned int)calibration_result.phase_b_offset_raw,
             (unsigned long)calibration_result.sample_count);

        wk_delay_ms(2U);
        if (motor_control_ready_set())
        {
          LOGI("Motor control: READY, waiting for command\r\n");
        }
        else
        {
          LOGE("Motor control: failed to enter READY\r\n");
          motor_control_fault_set(3U);
        }

      }
      else
      {
        LOGE("ADC calibration: offset_a=%u offset_b=%u valid=0\r\n",
             (unsigned int)calibration_result.phase_a_offset_raw,
             (unsigned int)calibration_result.phase_b_offset_raw);
        motor_control_fault_set(4U);
      }
    }
    else
    {
      LOGE("ADC calibration: timeout or invalid request\r\n");
      motor_control_fault_set(5U);
    }
  }
  else
  {
    motor_control_fault_set(6U);
    LOGE("DRV calibration blocked: nFAULT=%u BIF=%u MOE=%u\r\n",
         (unsigned int)gpio_input_data_bit_read(MOTOR_PWM_BREAK_PORT,
                                                 MOTOR_PWM_BREAK_PIN),
         (unsigned int)tmr_flag_get(MOTOR_PWM_TIMER, TMR_BRK_FLAG),
         (unsigned int)MOTOR_PWM_TIMER->brk_bit.oen);
  }

  /* add user code end 2 */

  while(1)
  {
    /* add user code begin 3 */
    (void)motor_slow_sensor_process();
    motor_cli_poll();
    motor_control_poll();
    interrupt_monitor_poll();

    /* add user code end 3 */
  }
}

  /* add user code begin 4 */

  /* add user code end 4 */
