/* add user code begin Header */
/**
  **************************************************************************
  * @file     at32f45x_int.c
  * @brief    main interrupt service routines.
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

/* includes ------------------------------------------------------------------*/
#include "at32f45x_int.h"
/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#include "interrupt_monitor.h"
#include "motor_performance_monitor.h"
#include "motor_adc_port.h"
#include "motor_current_calibration.h"
#include "motor_current_sample.h"
#include "motor_pwm_port.h"
#include "motor_open_loop.h"
#include "motor_hall_port.h"
#include "motor_hall_angle_observer.h"
#include "motor_hall_angle_estimator.h"
#include "motor_current_transform.h"
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

/* add user code end function prototypes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/* add user code end 0 */

/* external variables ---------------------------------------------------------*/
/* add user code begin external variables */

/* add user code end external variables */

/**
  * @brief  this function handles nmi exception.
  * @param  none
  * @retval none
  */
void NMI_Handler(void)
{
  /* add user code begin NonMaskableInt_IRQ 0 */

  /* add user code end NonMaskableInt_IRQ 0 */

  /* add user code begin NonMaskableInt_IRQ 1 */

  /* add user code end NonMaskableInt_IRQ 1 */
}

/**
  * @brief  this function handles hard fault exception.
  * @param  none
  * @retval none
  */
void HardFault_Handler(void)
{
  /* add user code begin HardFault_IRQ 0 */

  /* add user code end HardFault_IRQ 0 */
  /* go to infinite loop when hard fault exception occurs */
  while (1)
  {
    /* add user code begin W1_HardFault_IRQ 0 */

    /* add user code end W1_HardFault_IRQ 0 */
  }
}


/**
  * @brief  this function handles memory manage exception.
  * @param  none
  * @retval none
  */
void MemManage_Handler(void)
{
  /* add user code begin MemoryManagement_IRQ 0 */

  /* add user code end MemoryManagement_IRQ 0 */
  /* go to infinite loop when memory manage exception occurs */
  while (1)
  {
    /* add user code begin W1_MemoryManagement_IRQ 0 */

    /* add user code end W1_MemoryManagement_IRQ 0 */
  }
}

/**
  * @brief  this function handles bus fault exception.
  * @param  none
  * @retval none
  */
void BusFault_Handler(void)
{
  /* add user code begin BusFault_IRQ 0 */

  /* add user code end BusFault_IRQ 0 */
  /* go to infinite loop when bus fault exception occurs */
  while (1)
  {
    /* add user code begin W1_BusFault_IRQ 0 */

    /* add user code end W1_BusFault_IRQ 0 */
  }
}

/**
  * @brief  this function handles usage fault exception.
  * @param  none
  * @retval none
  */
void UsageFault_Handler(void)
{
  /* add user code begin UsageFault_IRQ 0 */

  /* add user code end UsageFault_IRQ 0 */
  /* go to infinite loop when usage fault exception occurs */
  while (1)
  {
    /* add user code begin W1_UsageFault_IRQ 0 */

    /* add user code end W1_UsageFault_IRQ 0 */
  }
}

/**
  * @brief  this function handles svcall exception.
  * @param  none
  * @retval none
  */
void SVC_Handler(void)
{
  /* add user code begin SVCall_IRQ 0 */

  /* add user code end SVCall_IRQ 0 */
  /* add user code begin SVCall_IRQ 1 */

  /* add user code end SVCall_IRQ 1 */
}

/**
  * @brief  this function handles debug monitor exception.
  * @param  none
  * @retval none
  */
void DebugMon_Handler(void)
{
  /* add user code begin DebugMonitor_IRQ 0 */

  /* add user code end DebugMonitor_IRQ 0 */
  /* add user code begin DebugMonitor_IRQ 1 */

  /* add user code end DebugMonitor_IRQ 1 */
}

/**
  * @brief  this function handles pendsv_handler exception.
  * @param  none
  * @retval none
  */
void PendSV_Handler(void)
{
  /* add user code begin PendSV_IRQ 0 */

  /* add user code end PendSV_IRQ 0 */
  /* add user code begin PendSV_IRQ 1 */

  /* add user code end PendSV_IRQ 1 */
}


/**
  * @brief  this function handles systick handler.
  * @param  none
  * @retval none
  */
void SysTick_Handler(void)
{
  /* add user code begin SysTick_IRQ 0 */
  interrupt_monitor_counters.systick++;

  /* add user code end SysTick_IRQ 0 */

  /* add user code begin SysTick_IRQ 1 */

  /* add user code end SysTick_IRQ 1 */
}

/**
  * @brief  this function handles DMA1 Channel 3 handler.
  * @param  none
  * @retval none
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* add user code begin DMA1_Channel3_IRQ 0 */

  /* add user code end DMA1_Channel3_IRQ 0 */

  if(dma_interrupt_flag_get(DMA1_FDT3_FLAG) != RESET)
  {   
    /* add user code begin DMA1_FDT3_FLAG */
    /* handle full data transfer and clear flag */
    dma_flag_clear(DMA1_FDT3_FLAG);
    interrupt_monitor_counters.adc_slow_dma_complete++;
    motor_adc_port_slow_sample_capture();
    /* add user code end DMA1_FDT3_FLAG */ 
  }

  /* add user code begin DMA1_Channel3_IRQ 1 */

  /* add user code end DMA1_Channel3_IRQ 1 */
}

/**
  * @brief  this function handles ADC1 2 handler.
  * @param  none
  * @retval none
  */
void ADC1_2_IRQHandler(void)
{
  /* add user code begin ADC1_2_IRQ 0 */
  uint32_t start_cycles = motor_performance_monitor_begin();
  uint32_t adc_fast_handled = 0U;
  motor_adc_fast_sample_t adc_fast_sample;

  /* add user code end ADC1_2_IRQ 0 */

  if(adc_interrupt_flag_get(ADC1, ADC_PCCE_FLAG) != RESET)
  {
    /* add user code begin ADC1_ADC_PCCE_FLAG */
    /* clear flag */
    adc_flag_clear(ADC1, ADC_PCCE_FLAG);
    interrupt_monitor_counters.adc_fast_complete++;
    motor_adc_port_fast_sample_capture(&adc_fast_sample);
    motor_current_calibration_sample_process(&adc_fast_sample);
    if (motor_current_sample_process(&adc_fast_sample))
    {
      motor_pwm_port_emergency_stop();
    }
    motor_open_loop_fast_process();
    motor_hall_angle_estimator_fast_process();
    motor_current_transform_fast_process();
    motor_inductance_identification_fast_process();
    motor_current_loop_test_fast_process();
    motor_current_control_fast_process();
    motor_torque_loop_test_fast_process();
    adc_fast_handled = 1U;
    /* add user code end ADC1_ADC_PCCE_FLAG */ 
  }

  if(adc_interrupt_flag_get(ADC1, ADC_TCF_FLAG) != RESET)
  {
    /* add user code begin ADC1_ADC_TCF_FLAG */
    /* clear flag */
    adc_flag_clear(ADC1, ADC_TCF_FLAG);
    interrupt_monitor_counters.adc_trigger_fail++;
    /* add user code end ADC1_ADC_TCF_FLAG */ 
  }

  /* add user code begin ADC1_2_IRQ 1 */
  if (adc_fast_handled != 0U)
  {
    (void)motor_performance_monitor_end(&adc_fast_performance_counter,
                                        start_cycles);
  }

  /* add user code end ADC1_2_IRQ 1 */
}

/**
  * @brief  this function handles CAN1 RX handler.
  * @param  none
  * @retval none
  */
void CAN1_RX_IRQHandler(void)
{
  can_rxbuf_type can_rxbuf_struct;

  /* add user code begin CAN1_RX_IRQ 0 */

  /* add user code end CAN1_RX_IRQ 0 */

  if(can_interrupt_flag_get(CAN1, CAN_RIF_FLAG) != RESET)
  {
    /* add user code begin CAN1_CAN_RIF_FLAG */
    /* clear flag and receive buffer release */
    can_flag_clear(CAN1, CAN_RIF_FLAG);
    can_rxbuf_read(CAN1, &can_rxbuf_struct);
    interrupt_monitor_counters.can1_rx++;
    /* add user code end CAN1_CAN_RIF_FLAG */
  }

  /* add user code begin CAN1_RX_IRQ 1 */

  /* add user code end CAN1_RX_IRQ 1 */
}

/**
  * @brief  this function handles CAN1 ERR handler.
  * @param  none
  * @retval none
  */
void CAN1_ERR_IRQHandler(void)
{
  /* add user code begin CAN1_ERR_IRQ 0 */
  interrupt_monitor_counters.can1_error++;

  /* add user code end CAN1_ERR_IRQ 0 */

  /* add user code begin CAN1_ERR_IRQ 1 */

  /* add user code end CAN1_ERR_IRQ 1 */
}

/**
  * @brief  this function handles EXINT Line [9:5] handler.
  * @param  none
  * @retval none
  */
void EXINT9_5_IRQHandler(void)
{
  /* add user code begin EXINT9_5_IRQ 0 */
  uint32_t hall_edge_handled = 0U;

  /* add user code end EXINT9_5_IRQ 0 */

  if(exint_interrupt_flag_get(EXINT_LINE_5) != RESET)
  {
    /* add user code begin EXINT_LINE_5 */
    /* clear flag */
    exint_flag_clear(EXINT_LINE_5);
    interrupt_monitor_counters.hall_a_edge++;
    hall_edge_handled = 1U;
    /* add user code end EXINT_LINE_5 */ 
  }

  if(exint_interrupt_flag_get(EXINT_LINE_6) != RESET)
  {
    /* add user code begin EXINT_LINE_6 */
    /* clear flag */
    exint_flag_clear(EXINT_LINE_6);
    interrupt_monitor_counters.hall_b_edge++;
    hall_edge_handled = 1U;
    /* add user code end EXINT_LINE_6 */ 
  }

  if(exint_interrupt_flag_get(EXINT_LINE_7) != RESET)
  {
    /* add user code begin EXINT_LINE_7 */
    /* clear flag */
    exint_flag_clear(EXINT_LINE_7);
    interrupt_monitor_counters.hall_c_edge++;
    hall_edge_handled = 1U;
    /* add user code end EXINT_LINE_7 */ 
  }

  /* add user code begin EXINT9_5_IRQ 1 */
  if (hall_edge_handled != 0U)
  {
    motor_hall_port_edge_capture();
    motor_hall_angle_observer_edge_process();
    motor_hall_angle_estimator_edge_process();
  }

  /* add user code end EXINT9_5_IRQ 1 */
}

/**
  * @brief  this function handles TMR1 Channel handler.
  * @param  none
  * @retval none
  */
void TMR1_CH_IRQHandler(void)
{
  /* add user code begin TMR1_CH_IRQ 0 */

  /* add user code end TMR1_CH_IRQ 0 */

  /* channel4 interrupt management */
  if(tmr_interrupt_flag_get(TMR1, TMR_C4_FLAG) != RESET)
  {
    /* add user code begin TMR1_TMR_C4_FLAG */
    /* clear flag */
    tmr_flag_clear(TMR1, TMR_C4_FLAG);
    interrupt_monitor_counters.tmr1_channel4++;
    /* add user code end TMR1_TMR_C4_FLAG */
  }

  /* add user code begin TMR1_CH_IRQ 1 */

  /* add user code end TMR1_CH_IRQ 1 */
}

/**
  * @brief  this function handles USART1 handler.
  * @param  none
  * @retval none
  */
void USART1_IRQHandler(void)
{
  /* add user code begin USART1_IRQ 0 */

  /* add user code end USART1_IRQ 0 */

  if((usart_interrupt_flag_get(USART1, USART_NERR_FLAG) != RESET) || \
    (usart_interrupt_flag_get(USART1, USART_ROERR_FLAG) != RESET) || \
    (usart_interrupt_flag_get(USART1, USART_FERR_FLAG) != RESET))
  {
    /* add user code begin USART1_USART_NERR_FLAG, USART_ROERR_FLAG or USART_FERR_FLAG */
    /* clear flag */
    usart_flag_clear(USART1, USART_NERR_FLAG | USART_ROERR_FLAG | USART_FERR_FLAG);
    /* add user code end  USART1_USART_NERR_FLAG, USART_ROERR_FLAG or USART_FERR_FLAG */ 
  }

  if(usart_interrupt_flag_get(USART1, USART_IDLEF_FLAG) != RESET)
  {
    /* add user code begin USART1_USART_IDLEF_FLAG */
    /* clear flag */
    usart_flag_clear(USART1, USART_IDLEF_FLAG);
    interrupt_monitor_counters.usart1_idle++;
    /* add user code end USART1_USART_IDLEF_FLAG */ 
  }

  /* add user code begin USART1_IRQ 1 */

  /* add user code end USART1_IRQ 1 */
}

/**
  * @brief  this function handles USART3 handler.
  * @param  none
  * @retval none
  */
void USART3_IRQHandler(void)
{
  /* add user code begin USART3_IRQ 0 */

  /* add user code end USART3_IRQ 0 */

  if((usart_interrupt_flag_get(USART3, USART_NERR_FLAG) != RESET) || \
    (usart_interrupt_flag_get(USART3, USART_ROERR_FLAG) != RESET) || \
    (usart_interrupt_flag_get(USART3, USART_FERR_FLAG) != RESET))
  {
    /* add user code begin USART3_USART_NERR_FLAG, USART_ROERR_FLAG or USART_FERR_FLAG */
    /* clear flag */
    usart_flag_clear(USART3, USART_NERR_FLAG | USART_ROERR_FLAG | USART_FERR_FLAG);
    /* add user code end  USART3_USART_NERR_FLAG, USART_ROERR_FLAG or USART_FERR_FLAG */ 
  }

  if(usart_interrupt_flag_get(USART3, USART_IDLEF_FLAG) != RESET)
  {
    /* add user code begin USART3_USART_IDLEF_FLAG */
    /* clear flag */
    usart_flag_clear(USART3, USART_IDLEF_FLAG);
    interrupt_monitor_counters.usart3_idle++;
    /* add user code end USART3_USART_IDLEF_FLAG */ 
  }

  /* add user code begin USART3_IRQ 1 */

  /* add user code end USART3_IRQ 1 */
}

/* add user code begin 1 */

/* add user code end 1 */
