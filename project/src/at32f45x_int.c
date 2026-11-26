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
#include "motor_adc_port.h"
#include "motor_current_calibration.h"
#include "motor_current_sample.h"
#include "motor_pwm_port.h"
#include "motor_board_config.h"
#include "motor_open_loop.h"
#if (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_HALL)
#include "motor_hall_port.h"
#include "motor_hall_decoder.h"
#include "motor_hall_angle_estimator.h"
#elif (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_ENCODER)
#include "encoder_rs485.h"
#endif
#include "motor_commissioning.h"
#include "motor_rotor_feedback.h"
#include "motor_speed_feedback.h"
#include "motor_speed_control.h"
#include "motor_can.h"
#include "motor_current_transform.h"
#include "motor_inductance_identification.h"
#include "motor_current_control.h"
#include "motor_trace.h"
#include "motor_current_startup.h"
#include "motor_uart_port.h"
#include "motor_control.h"

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

#if (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_HALL)
/**
 * @brief 捕获一次Hall边沿并分发给下游模块。
 * @param sample 输出本次捕获后的端口快照，不允许为空。
 * @return 无。
 * @details 标定独立接收原始边沿并自行校验，不依赖待重新学习的旧顺序表。
 *          运行角度估算仍仅接收解码器确认的合法跳变。
 */
static void hall_edge_dispatch(motor_hall_port_sample_t *sample)
{
  motor_hall_port_edge_capture();
  if (motor_hall_port_sample_read(sample))
  {
    motor_commissioning_hall_edge_process();
    if (motor_hall_decoder_edge_process_captured(sample->state, sample->edge_count,
                                        sample->timestamp_cycles, sample->capture_us))
      motor_hall_angle_estimator_edge_process();
  }
}
#endif

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
  __disable_irq();
  motor_pwm_port_emergency_stop();
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
  __disable_irq();
  motor_pwm_port_emergency_stop();
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
  __disable_irq();
  motor_pwm_port_emergency_stop();
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
  __disable_irq();
  motor_pwm_port_emergency_stop();
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
  motor_current_startup_process_1khz();
#if (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_ENCODER)
  encoder_rs485_request_abs();
#endif
  motor_rotor_feedback_process_1khz();
  motor_speed_feedback_process_1khz();
  motor_speed_control_process_1khz();
  motor_can_tick();

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
  motor_adc_fast_sample_t adc_fast_sample;
  static bool monitor_overcurrent_pending;

  if (adc_flag_get(ADC1, ADC_VMOR_FLAG) != RESET)
  {
    adc_flag_clear(ADC1, ADC_VMOR_FLAG);
    monitor_overcurrent_pending = true;
  }
  if (adc_flag_get(ADC2, ADC_VMOR_FLAG) != RESET)
  {
    adc_flag_clear(ADC2, ADC_VMOR_FLAG);
    monitor_overcurrent_pending = true;
  }

  /* add user code end ADC1_2_IRQ 0 */

  if(adc_interrupt_flag_get(ADC1, ADC_PCCE_FLAG) != RESET)
  {
    /* add user code begin ADC1_ADC_PCCE_FLAG */
    /* clear flag */
    adc_flag_clear(ADC1, ADC_PCCE_FLAG);
    motor_adc_port_fast_sample_capture(&adc_fast_sample);
    motor_current_calibration_sample_process(&adc_fast_sample);
    if (motor_current_sample_process(&adc_fast_sample, monitor_overcurrent_pending))
    {
      motor_control_fault_notify_from_isr(MOTOR_FAULT_OVERCURRENT);
    }
    monitor_overcurrent_pending = false;
    motor_open_loop_fast_process();
#if (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_HALL)
    motor_hall_angle_estimator_fast_process();
#endif
    motor_current_startup_fast_process();
    motor_current_transform_fast_process();
    motor_inductance_identification_fast_process();
    motor_current_control_fast_process();
    motor_trace_fast_sample();
    /* add user code end ADC1_ADC_PCCE_FLAG */ 
  }

  if(adc_interrupt_flag_get(ADC1, ADC_TCF_FLAG) != RESET)
  {
    /* add user code begin ADC1_ADC_TCF_FLAG */
    /* clear flag */
    adc_flag_clear(ADC1, ADC_TCF_FLAG);
    /* add user code end ADC1_ADC_TCF_FLAG */ 
  }

  /* add user code begin ADC1_2_IRQ 1 */

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

  /* add user code end CAN1_ERR_IRQ 0 */

  /* add user code begin CAN1_ERR_IRQ 1 */

  /* add user code end CAN1_ERR_IRQ 1 */
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
    motor_uart_port_idle_capture();
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
#if (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_ENCODER)
    encoder_rs485_on_idle();
#endif
    /* add user code end USART3_USART_IDLEF_FLAG */
  }

  /* add user code begin USART3_IRQ 1 */

  /* add user code end USART3_IRQ 1 */
}

/* add user code begin 1 */

/**
 * @brief 处理TMR1硬件Break中断。
 * @param 无。
 * @return 无。
 * @details Break硬件首先自动清除MOE；本函数继续关闭门极驱动并投递故障，
 *          不在中断中清除BIF或打印日志，留待故障恢复流程确认nFAULT后处理。
 */
void TMR1_BRK_TMR9_IRQHandler(void)
{
  if (tmr_interrupt_flag_get(TMR1, TMR_BRK_FLAG) != RESET)
  {
    /* BIF在nFAULT仍为低时不能安全清除；先屏蔽该中断，防止重复进入。 */
    tmr_interrupt_enable(TMR1, TMR_BRK_INT, FALSE);
    motor_control_fault_notify_from_isr(MOTOR_FAULT_HARDWARE_BREAK);
  }
}

/**
 * @brief 处理TMR4 Hall XOR捕获中断。
 * @param 无。
 * @return 无。
 * @details 三路Hall异或并经数字滤波后，双边沿捕获置位C1标志。按前后两次
 *          端口状态并分发Hall边沿。
 */
void TMR4_GLOBAL_IRQHandler(void)
{
#if (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_HALL)
  motor_hall_port_sample_t hall_port_sample;
  if (tmr_interrupt_flag_get(MOTOR_HALL_CAPTURE_TIMER, TMR_OVF_FLAG) != RESET)
  {
    motor_hall_port_overflow_capture();
  }

  if (tmr_interrupt_flag_get(MOTOR_HALL_CAPTURE_TIMER, TMR_C1_FLAG) !=
      RESET)
  {
    tmr_flag_clear(MOTOR_HALL_CAPTURE_TIMER, TMR_C1_FLAG);
    hall_edge_dispatch(&hall_port_sample);
  }
#endif
}

/* add user code end 1 */
