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
#include "motor_control_config.h"
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
#include "motor_hall_angle_observer.h"
#include "motor_hall_angle_estimator.h"
#include "motor_current_transform.h"
#include "motor_resistance_identification.h"
#include "motor_inductance_identification.h"
#include "motor_current_loop_test.h"
#include "motor_current_control.h"
#include "motor_torque_loop_test.h"

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
  motor_adc_port_init();
  motor_current_sample_init();
  motor_slow_sensor_init();
  motor_open_loop_init();
  motor_hall_angle_observer_init();
  motor_hall_angle_estimator_init();
  motor_current_transform_init();
  motor_current_control_init();

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

  /* add user code begin 2 */
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

        /* 等待10 kHz正式电流处理产生有效样本，再允许开启开环PWM。 */
        wk_delay_ms(2U);
        {
          motor_resistance_identification_result_t resistance_result;

          LOGI("Rs identification: started target=3000 mA max_voltage=1000 mV\r\n");
          if (motor_resistance_identification_run(&resistance_result))
          {
            LOGI("Rs identification: PASS ud=%u mV ia=%ld mA rs=%lu mOhm samples=%lu\r\n",
                 (unsigned int)resistance_result.applied_voltage_mv,
                 (long)resistance_result.phase_a_average_ma,
                 (unsigned long)resistance_result.resistance_average_mohm,
                 (unsigned long)resistance_result.sample_count);
            {
              motor_inductance_identification_result_t inductance_result;
              LOGI("L identification: started frequency=600 Hz voltage=500 mV\r\n");
              if (motor_inductance_identification_run(
                    resistance_result.resistance_average_mohm,
                    &inductance_result))
              {
                LOGI("L identification: PASS Ld=%lu uH Lq=%lu uH Id_amp=%lu mA Iq_amp=%lu mA\r\n",
                     (unsigned long)inductance_result.direct_inductance_uh,
                     (unsigned long)inductance_result.quadrature_inductance_uh,
                     (unsigned long)inductance_result.direct_current_amplitude_ma,
                     (unsigned long)inductance_result.quadrature_current_amplitude_ma);
                {
                  motor_current_loop_test_result_t current_loop_result;
                  LOGI("Current loop test: started Id_ref=2000 mA Iq_ref=0 mA\r\n");
                  if (motor_current_loop_test_run(&current_loop_result))
                  {
                    LOGI("Current loop test: PASS Id_avg=%ld Iq_avg=%ld mA peak=%ld/%ld mA Vd/Vq=%ld/%ld mV samples=%lu\r\n",
                         (long)current_loop_result.direct_average_ma,
                         (long)current_loop_result.quadrature_average_ma,
                         (long)current_loop_result.direct_peak_ma,
                         (long)current_loop_result.quadrature_peak_ma,
                         (long)current_loop_result.direct_voltage_mv,
                         (long)current_loop_result.quadrature_voltage_mv,
                         (unsigned long)current_loop_result.sample_count);
                    {
                      motor_torque_loop_test_result_t torque_result;
                      LOGI("Torque loop test: started Id_ref=0 mA Iq_ref=%ld mA duration=%lu ms\r\n",
                           (long)MOTOR_TORQUE_TEST_QUADRATURE_REFERENCE_MA,
                           (unsigned long)(MOTOR_TORQUE_TEST_DURATION_SAMPLES /
                                           (MOTOR_PWM_FREQUENCY_HZ / 1000U)));
                      if (motor_torque_loop_test_run(&torque_result))
                      {
                        LOGI("Torque loop test: PASS Id/Iq_avg=%ld/%ld mA peak=%ld/%ld mA Vd/Vq_avg=%ld/%ld mV final=%ld/%ld mV freq=%lu.%03lu Hz samples=%lu\r\n",
                             (long)torque_result.direct_average_ma,
                             (long)torque_result.quadrature_average_ma,
                             (long)torque_result.direct_peak_ma,
                             (long)torque_result.quadrature_peak_ma,
                             (long)torque_result.direct_voltage_average_mv,
                             (long)torque_result.quadrature_voltage_average_mv,
                             (long)torque_result.final_direct_voltage_mv,
                             (long)torque_result.final_quadrature_voltage_mv,
                             (unsigned long)(torque_result.final_frequency_millihz / 1000U),
                             (unsigned long)(torque_result.final_frequency_millihz % 1000U),
                             (unsigned long)torque_result.sample_count);
                      }
                      else
                      {
                        LOGE("Torque loop test: FAIL status=%u samples=%lu hall=%u freq=%lu.%03lu Hz current=%ld/%ld/%ld mA\r\n",
                             (unsigned int)torque_result.status,
                             (unsigned long)torque_result.sample_count,
                             (unsigned int)torque_result.final_hall_state,
                             (unsigned long)(torque_result.final_frequency_millihz / 1000U),
                             (unsigned long)(torque_result.final_frequency_millihz % 1000U),
                             (long)torque_result.final_phase_a_ma,
                             (long)torque_result.final_phase_b_ma,
                             (long)torque_result.final_phase_c_ma);
                        motor_pwm_port_emergency_stop();
                      }
                    }
                  }
                  else
                  {
                    LOGE("Current loop test: FAIL\r\n");
                    motor_pwm_port_emergency_stop();
                  }
                }
              }
              else
              {
                LOGE("L identification: FAIL\r\n");
                motor_pwm_port_emergency_stop();
              }
            }
          }
          else
          {
            LOGE("Rs identification: FAIL status=%u ud=%u mV ia=%ld mA\r\n",
                 (unsigned int)resistance_result.status,
                 (unsigned int)resistance_result.applied_voltage_mv,
                 (long)resistance_result.phase_a_average_ma);
          }
        }
        /* 电阻辨识完成后保持PWM关闭，不自动进入开环旋转。 */
      }
      else
      {
        LOGE("ADC calibration: offset_a=%u offset_b=%u valid=0\r\n",
             (unsigned int)calibration_result.phase_a_offset_raw,
             (unsigned int)calibration_result.phase_b_offset_raw);
        motor_pwm_port_emergency_stop();
      }
    }
    else
    {
      LOGE("ADC calibration: timeout or invalid request\r\n");
      motor_pwm_port_emergency_stop();
    }
  }
  else
  {
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
    interrupt_monitor_poll();

    /* add user code end 3 */
  }
}

  /* add user code begin 4 */

  /* add user code end 4 */
