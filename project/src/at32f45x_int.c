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
#include "at32f45x_wk_config.h"
#include "wk_adc.h"
#include "wk_usart.h"
#include "wk_can.h"
#include "foc_api.h"
#include "foc_controller.h"
#include "foc_current_loop.h"
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
uint32_t tmr1_ch4_int_count = 0;
volatile uint32_t systick_ms = 0;
uint32_t adc_occe_count = 0;
uint32_t adc_pcce_count = 0;
uint32_t dma_fdt3_count = 0;
uint32_t adc_occo_count = 0;
uint32_t adc_tcf_count = 0;
/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */

/* add user code end function prototypes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/* add user code end 0 */

/* external variables ---------------------------------------------------------*/
/* add user code begin external variables */
extern uint16_t adc_ordinary_buffer[4];
extern ControllerStruct controller_eyou;
extern volatile uint8_t g_foc_openloop_enable;
extern uint8_t open_loop_mode;
extern int16_t v_d_test, v_q_test;
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
  systick_ms++;
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
		dma_fdt3_count++;

		gpio_bits_reset(TP_TEST_GPIO_PORT, TP_TEST_PIN);
		gpio_bits_set(TP_TEST_GPIO_PORT, TP_TEST_PIN);

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
  extern volatile uint32_t g_adc_isr_in_cycles, g_adc_isr_out_cycles;
  extern volatile uint32_t g_adc_isr_cycles, g_adc_isr_cycles_max;
  uint32_t _isr_entry = DWT->CYCCNT;
  g_adc_isr_in_cycles = _isr_entry;
  /* add user code end ADC1_2_IRQ 0 */

  if(adc_interrupt_flag_get(ADC1, ADC_OCCE_FLAG) != RESET)
  {
    /* add user code begin ADC1_ADC_OCCE_FLAG */
    /* clear flag */
    adc_flag_clear(ADC1, ADC_OCCE_FLAG);
		adc_occe_count++;

    /* Read ordinary group (temperature/VDC, 1kHz) */
    adc_foc_on_regular_done();

//		/* recovery ADC and DMA for next conversion in dual mode */
//		adc_ordinary_convert_recovery();

//		/* toggle TP_TEST pin */
//		if(TP_TEST_GPIO_PORT->odt & TP_TEST_PIN) {
//			gpio_bits_reset(TP_TEST_GPIO_PORT, TP_TEST_PIN);
//		} else {
//			gpio_bits_set(TP_TEST_GPIO_PORT, TP_TEST_PIN);
//		}

//		gpio_bits_reset(TP_TEST_GPIO_PORT, TP_TEST_PIN);
//		gpio_bits_set(TP_TEST_GPIO_PORT, TP_TEST_PIN);

    /* add user code end ADC1_ADC_OCCE_FLAG */ 
  }

  if(adc_interrupt_flag_get(ADC1, ADC_PCCE_FLAG) != RESET)
  {
    /* add user code begin ADC1_ADC_PCCE_FLAG */
    /* clear flag */
    adc_flag_clear(ADC1, ADC_PCCE_FLAG);
		adc_pcce_count++;

    /* Read injected group (FOC current sampling, 10kHz) */
    adc_foc_on_injected_done();

    /* DPT async now triggered by CC4 ISR (ahead of ADC, see TMR1_CH_IRQHandler).
     * Encoder data is ready by the time ADC ISR runs (Enc_done < T0). */

    /* ============================================================
     * FOC scheduling (10kHz, migrated from cubemx_yxsui adc.c:551-596)
     * Guard: skip FOC code until Init_foc() is complete (g_foc_init_done=1)
     * ============================================================ */
    if (g_foc_init_done) {
      /* Get raw ADC values for FOC (before offset subtraction) */
      uint16_t raw_a = adc_preempt_conversion_data_get(ADC1, ADC_PREEMPT_CHANNEL_1);
      uint16_t raw_b = adc_preempt_conversion_data_get(ADC2, ADC_PREEMPT_CHANNEL_1);

      /* Pass raw values to FOC controller */
      controller_eyou.Ia_raw = raw_a;
      controller_eyou.Ib_raw = raw_b;

      if (controller_eyou.foc_run >= 1) {
        /* Closed-loop: full FOC scheduling (current/speed/position loops) */
        MC_Loop_Schedule(&controller_eyou);
      } else if (g_foc_openloop_enable) {
        /* Open-loop test: current sampling + open-loop SVPWM */
        phase_current_sample(&controller_eyou);
        FocOpenTest(&controller_eyou, open_loop_mode, v_d_test, v_q_test, raw_a, raw_b);
      }
    }

    /* add user code end ADC1_ADC_PCCE_FLAG */ 
  }

  if(adc_interrupt_flag_get(ADC1, ADC_TCF_FLAG) != RESET)
  {
    /* add user code begin ADC1_ADC_TCF_FLAG */
    /* clear flag */
    adc_flag_clear(ADC1, ADC_TCF_FLAG);
    adc_tcf_count++;

    /* recovery ADC when trigger convert fail */
    adc_ordinary_convert_recovery();
    /* add user code end ADC1_ADC_TCF_FLAG */ 
  }

  /* add user code begin ADC1_2_IRQ 1 */
  {
    uint32_t _isr_exit = DWT->CYCCNT;
    uint32_t dt = _isr_exit - _isr_entry;
    g_adc_isr_cycles = dt;
    if (dt > g_adc_isr_cycles_max) g_adc_isr_cycles_max = dt;
    g_adc_isr_out_cycles = _isr_exit;
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
    /* clear flag and dispatch all pending frames to fdcan_rx_user */
    can_flag_clear(CAN1, CAN_RIF_FLAG);
    wk_can1_rx_dispatch();
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
  extern volatile uint32_t g_tim1_cc4_cycles, g_tim1_cc4_exit_cycles;
  uint32_t _cc4_entry = DWT->CYCCNT;
  uint8_t _cc4_is_upcount = !(TMR1->ctrl1 & (1u << 4));
  if (_cc4_is_upcount) g_tim1_cc4_cycles = _cc4_entry;
  /* add user code end TMR1_CH_IRQ 0 */

  /* channel4 interrupt management */
  if(tmr_interrupt_flag_get(TMR1, TMR_C4_FLAG) != RESET)
  {
    /* add user code begin TMR1_TMR_C4_FLAG */
    /* clear flag */
    tmr_flag_clear(TMR1, TMR_C4_FLAG);
    tmr1_ch4_int_count++;

    /* Only trigger DPT on downcount (first match in period).
     * ctrl1 bit4 (DIR): 0=counting up, 1=counting down */
    /* Trigger DPT encoder async read (ahead of next ADC ISR) */
    if (_cc4_is_upcount) DPT_AsyncRequest();

    // /* isr_print test: print every 10000 ticks (~1Hz @10kHz) */
    // if ((tmr1_ch4_int_count % 10000) == 0) {
    //   isr_print("[ISR] TMR1 CH4 tick\r\n");
    // }

//		gpio_bits_reset(TP_TEST_GPIO_PORT, TP_TEST_PIN);
//		gpio_bits_set(TP_TEST_GPIO_PORT, TP_TEST_PIN);

    /* add user code end TMR1_TMR_C4_FLAG */
  }

  /* add user code begin TMR1_CH_IRQ 1 */
  if (_cc4_is_upcount) g_tim1_cc4_exit_cycles = DWT->CYCCNT;
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
    USART1_IDLE_Handler();
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
    /* DPT async driver handles flag clear + frame parse + state advance */
    DPT_USART3_IDLE_Handler();
    /* add user code end USART3_USART_IDLEF_FLAG */ 
  }

  /* add user code begin USART3_IRQ 1 */

  /* add user code end USART3_IRQ 1 */
}

/* add user code begin 1 */

/* add user code end 1 */
