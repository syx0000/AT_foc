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
#include "wk_tmr.h"
#include "wk_usart.h"
#include "wk_dma.h"
#include "wk_gpio.h"
#include "wk_system.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#include "foc_api.h"
#include "foc_bsp.h"
#include "foc_controller.h"
#include "foc_data.h"
#include "encoder_calc.h"
#include "ifly_fault.h"
#include "ifly_test.h"
#include "can_wly.h"
#include "can_debug.h"
#include "flash_port.h"
#include "ota_app.h"
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
extern uint32_t tmr1_ch4_int_count;
extern volatile uint32_t systick_ms;
extern uint32_t adc_occe_count;
extern uint32_t adc_pcce_count;
extern uint32_t dma_fdt3_count;
extern uint32_t adc_occo_count;
extern uint32_t adc_tcf_count;
extern uint32_t recovery_call_count;
uint16_t pwm_duty = 0;
uint8_t pwm_dir = 1;
uint16_t adc_ordinary_buffer[4]; // HALFWORD: 2x ADC1 + 2x ADC2

/* ADC business layer data (from wk_adc.c) */
extern volatile FOC_CurrentSample_t g_foc_current;
extern volatile uint16_t g_temp_motor_raw;
extern volatile uint16_t g_temp_mos_raw;
extern volatile uint16_t g_so_c_raw;

/* FOC layer (from foc_api.c) */
extern ControllerStruct controller_eyou;
extern uint8_t NPP;
extern volatile uint8_t g_foc_openloop_enable;
extern volatile uint32_t g_vdc_raw;

/* Open-loop test parameters (used by ADC ISR FocOpenTest call)
 * Migrated from cubemx_yxsui/Core/Src/main.c:64-66 */
uint8_t open_loop_mode = 0;  /* 0=auto rotate, 1=encoder follow */
int16_t v_d_test = 0;        /* d-axis voltage (Q10 format) */
int16_t v_q_test = 512;      /* q-axis voltage (Q10 ~0.5V) */

/* CAN protocol calibration request flag (set by CAN 0x2F01 cmd) */
volatile uint8_t g_can_cali_request = 0;
/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */

/* add user code end function prototypes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/* add user code end 0 */

/**
  * @brief main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  /* add user code begin 1 */
  /* VTOR重定位: App从0x08004000开始，Bootloader在0x08000000~0x08003FFF */
  SCB->VTOR = 0x08004000;

  /* DWT cycle counter (CPU freq lazy-loaded on first use, no clock dependency) */
  DWT_Init();
  isr_print_init();
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

  /* init gpio function. */
  wk_gpio_config();

  /* init tmr6 function. */
  wk_tmr6_init();

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

  /* init adc-common function. */
  wk_adc_common_init();

  /* init adc1 function. */
  wk_adc1_init();

  /* init adc2 function. */
  wk_adc2_init();

  /* init usart1 function. */
  wk_usart1_init();

  /* init usart3 function. */
  wk_usart3_init();

  /* init can1 function. */
  wk_can1_init();

  /* init tmr1 function. */
  wk_tmr1_init();

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

  /* add user code begin 2 */
	printf("\r\n\r\n FOC start\r\n");

	/* DWT verification: 1000 NOPs (each NOP = 1 cycle) + 100us busy wait */
	{
		uint32_t t0 = DWT_GetCycles();
		for (int i = 0; i < 100; i++) {
			__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
			__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		}
		uint32_t cycles = DWT_GetCycles() - t0;
		printf("DWT: 1000-NOP cycles=%u\r\n", cycles);

		uint32_t t1 = DWT_GetCycles();
		while ((DWT_GetCycles() - t1) < 100 * 192);  /* 100us @192MHz */
		uint32_t actual_us = DWT_CyclesToUs(DWT_GetCycles() - t1);
		printf("DWT: 100us busy-wait, actual=%u us\r\n", actual_us);
	}

	/* ADC zero offset calibration (motor must be stopped, IGBT off) */
	printf("ADC: calibrating offsets (1024 samples)...\r\n");
	adc_calibrate_offsets(1024);
	printf("ADC: offset_a=%d offset_b=%d\r\n", g_adc_offset_a, g_adc_offset_b);

	/* DPT encoder async driver: reconfigures DMA1_CH1/CH2 to BYTE width
	 * (WB defaults are HALFWORD which is broken for 8-bit UART). After this,
	 * PCCE ISR kicks a new transaction every 100µs and IDLE IRQ updates
	 * s_dpt_latest, so DPT_GetLatestAngles*() returns real data instead of 0. */
	DPT_Async_Init();
	printf("DPT async driver initialized (DMA1_CH1 TX, DMA1_CH2 RX, USART3 IDLE)\r\n");

	/* ============================================================
	 * FOC controller initialization (reference: cubemx_yxsui main.c:173-223)
	 * ============================================================ */
	printf("Initializing FOC controller...\r\n");

	/* 1. Set motor parameters (must be before Init_foc) */
	set_ver_par(90);  /* id=90: motor_h7_0426 compatible, NPP=8 */

	/* 2. FOC core init (filters/ramps/flags/FlashData) */
	Init_foc(&controller_eyou);

	/* 3. Sync ADC zero offset to FOC controller */
	controller_eyou.FlashData.Ia_offset = (uint16_t)g_adc_offset_a;
	controller_eyou.FlashData.Ib_offset = (uint16_t)g_adc_offset_b;

	/* 4. Output encoder zero init */
	Encoder_out_data_Reset(controller_eyou.FlashData.MaxPositionLimit,
	                       controller_eyou.FlashData.MinPositionLimit);

	/* 5. Reset controller data (clear integrators) */
	ResetControlData(&controller_eyou);

	/* 6. Set initial state - SAFE: foc_run=0 (no PWM driving) */
	controller_eyou.foc_run = 0;
	controller_eyou.controller_mode = PROFILE_TORQUE_MODE;
	controller_eyou.I_q_ref = 0;

	/* 7. Open-loop test auto-config from compile-time macro (per cubemx_yxsui/main.c:236) */
	g_foc_openloop_enable = USEFOC_OPEN_TEST;
	if (g_foc_openloop_enable == 1) {
		controller_eyou.foc_run = 0;  // Force foc_run=0 in openloop mode
	}

	/* 8. Enable PWM AFTER Init_foc (PCCE ISR calls FOC code, data must be initialized first) */
	TIM1_PWM_Start();
	/* CH1/2/3 outputs were already enabled by wk_tmr1_init via tmr_output_channel_config().
	 * No need to OR cctrl by hand — doing so before Init_foc finished was the source of the
	 * apparent freeze: PCCE ISR (running since wk_adc_init) entered FOC code with stale state. */

	printf("FOC init done: NPP=%d foc_run=%d mode=%d openloop=%d\r\n",
	       NPP, controller_eyou.foc_run, controller_eyou.controller_mode, g_foc_openloop_enable);

	/* Start USART1 DMA debug receive */
	USART1_DebugRx_Start();

	/* CAN protocol stack init */
	can_wly_init();
	can_debug_init();

	/* OTA subsystem init */
	ota_init();

	/* Clear boot_count in App Header to confirm stable boot.
	 * Bootloader increments boot_count before jumping; if we reach here
	 * the app is running fine so we mark it as stable. */
	ota_mark_self_stable();

  /* add user code end 2 */

  while(1)
  {
    /* add user code begin 3 */
    /* Drain isr_print ring buffer to USART */
    isr_print_poll();

    /* Serial debug command parser */
    dbg_cmd_set();

    /* OTA background processing (parse frames, write Flash) */
    ota_process();

    /* Bandwidth test done flag poll + result print */
    Test_log_print();

    /* CAN RX debug frame async print (canrxdbg cmd) */
    can_wly_dbg_poll();
    can_debug_poll();

    /* CAN 0x2F01 zero-point calibration request (set by CAN ISR, executed here) */
    if (g_can_cali_request) {
        g_can_cali_request = 0;
        uint8_t old_run = controller_eyou.foc_run;
        controller_eyou.foc_run = 0;
        controller_eyou.ServoErrFlag.All_Flag = 0;
        set_phase_voltage(&controller_eyou, 0, 0, 0);  /* 清零输出电压 */
        wk_delay_ms(10);
        TMR1->cctrl |= 0x0555u;
        TMR1->brk   |= (1u << 15);  /* MOE enable */
        printf("CAN Cali start\r\n");
        ElecAngleEstimate(&controller_eyou);
        if (Flash_EraseSector() != HAL_OK) {
            printf("CAN Cali: Flash erase FAIL\r\n");
        } else {
            WriteDataToFlash();
            printf("CAN Cali done\r\n");
        }
        controller_eyou.foc_run = old_run;
    }

    /* Periodic debug log print (period set by 'logfreq' cmd) */
    {
        static uint32_t log_tick = 0;
        if (systick_ms - log_tick >= logPriodMs) {
            log_tick = systick_ms;
            dbg_log_print();
        }
    }

    /* 1ms fault protection tick */
    {
        static uint32_t fault_tick = 0;
        if (systick_ms - fault_tick >= 1) {
            fault_tick = systick_ms;
            adc_convert();
            dcVoltageProFunc();
            boradTempProFunc();
            motorTempProFunc();
            overloadProFunc();
            if (!g_foc_openloop_enable) {
                CheckAndHandleAllFaultBits();
            }
            fault_brake_tick_1ms();
            drv_reset_tick_1ms();
            target_reach_check();
        }
    }

    /* CAN 1ms tick (active report mode) */
    {
        static uint32_t can_tick = 0;
        if (systick_ms - can_tick >= 1) {
            can_tick = systick_ms;
            can_wly_tick_1ms();
        }
    }

    /* add user code end 3 */
  }
}

  /* add user code begin 4 */

  /* add user code end 4 */
