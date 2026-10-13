/**
  **************************************************************************
  * @file     main.c
  * @brief    main program
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

#include "mc_lib.h"
#include "mc_ident_diag.h"

/** @addtogroup motor_evb_2v0
  * @{
  */

/** @addtogroup pmsm_foc_hall_sensor pmsm_foc_hall_sensor
  * @{
  */

/** @defgroup main
  * @brief main program
  * @{
  */

crm_clocks_freq_type crm_clocks_freq_struct = {0};
void check_configured_core_clock(void);
static flag_status esc_state_requires_gate_driver(esc_state_type state);
static void gate_driver_fault_latch(void);
/**
  * @brief  main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  system_clock_config();

  /* enable clock for remap or gpio exint */
  remap_exint_clock_enable_config();

  systick_init();

  mc_delay_init();

  gpio_pins_init();

  button_exint_init();

  led_config();

  adc_ordinary_config();

  adc_preempt_config();

  get_int_vref_cal_ratio();

  tmr_pwm_init();

  speed_timer_init();

  /* Hall inputs must be readable before param_init(). */
#if defined HALL_EXINT_EDGE_CAPTURE
  /* EXINT stays masked until the Hall lookup tables have been initialized. */
  hall_gpio_init();
#else
  /* Preserve the original TMR4 CH1/CH2/CH3 XOR initialization sequence. */
  hall_timer_init();
#endif

#ifdef BRAKING_RESISTOR
  /* pwm init for brake resistor */
  brake_pwm_init();
#endif

  mode_switch_init();

  /* get system clock */
  crm_clocks_freq_get(&crm_clocks_freq_struct);

  /* uart initialization */
#if defined USE_MOTOR_MONITOR || defined USE_UART_LOG
  uart_init(&ui_usart);
  dma_uart_configuration();
  uart_rx_init();
#endif
#if defined USE_UART_LOG
  printf("\r\nFOC UART log ready, baud=921600\r\n");
#endif

  /*delay for hardware stable */
  mc_delay_ms(500);

  firmware_id = get_fw_id();
  param_initial_rdy = mc_param_init(firmware_id);

  /*
   * Build and bind the Hall state/angle tables before either Hall or PWM
   * interrupts can consume them.
   */
  param_init();
  nvic_config();
#if defined HALL_EXINT_EDGE_CAPTURE
  hall_timer_init();
#endif

  /* enable tmr channel mode buffer */
  enable_pwm_timer_channel_buffer(&pwm_duty);

  if (gate_driver_enable() != RESET)
  {
    /* TMR1_CH4 must be active while I_offset_init() waits for ADC triggers. */
    enable_pwm_timer(&pwm_duty);
    curr_offset_rdy = I_offset_init(&current);
    gate_driver_disable();
  }
  else
  {
    curr_offset_rdy = RESET;
    gate_driver_fault_latch();
  }

  led_blink();

  while(1)
  {
#if defined USE_UART_LOG && defined MOTOR_PARAM_IDENTIFY
    motor_ident_diag_task();
#endif

    if(hall_learn.check_flag == SET)
    {
      write_flash_cmd();
      hall_learn.check_flag = RESET;
    }

    if (ctrl_mode_cmd != ctrl_mode_old)
    {
      param_clear();
      start_stop_btn_flag = RESET;
      if (ctrl_mode_cmd == POSITION_CTRL)
      {
        angle.cmd_final = (int32_t)(pos.val * PULSE_TO_ANGLE);
        pos.cmd_new = pos.val;
        pos.cmd_final = pos.cmd_new;
        pos.command = pos.cmd_final;
      }
      ctrl_mode = ctrl_mode_cmd;
      ctrl_mode_old = ctrl_mode;
    }

    if (esc_state != esc_state_old)
    {
      esc_state_type next_state = esc_state;
      flag_status next_requires_driver =
        esc_state_requires_gate_driver(next_state);

      /*
       * A stop request may race with a pending transition into RUNNING.
       * Do not wake the driver for a command that has already been stopped.
       */
      if (((next_state == ESC_STATE_RUNNING) ||
           (next_state == ESC_STATE_I_TUNE)) &&
          (start_stop_btn_flag == RESET))
      {
        gate_driver_disable();
        esc_state = ESC_STATE_FREE_RUN;
        continue;
      }

      /*
       * A boot-time gate fault skips current-offset calibration.  After the
       * fault is acknowledged, rebuild the offset before leaving IDLE.
       */
      if ((next_state == ESC_STATE_IDLE) && (curr_offset_rdy == RESET))
      {
        if (gate_driver_enable() == RESET)
        {
          gate_driver_fault_latch();
          continue;
        }

        enable_pwm_timer(&pwm_duty);
        curr_offset_rdy = I_offset_init(&current);
        gate_driver_disable();

        if (curr_offset_rdy == RESET)
        {
          gate_driver_fault_latch();
          continue;
        }
      }

      if ((next_requires_driver != RESET) &&
          (esc_state_requires_gate_driver(esc_state_old) == RESET))
      {
        if (gate_driver_enable() == RESET)
        {
          gate_driver_fault_latch();
          continue;
        }
      }
      else if (next_requires_driver == RESET)
      {
        if (next_state == ESC_STATE_ERROR)
        {
          gate_driver_emergency_shutdown();
        }
        else
        {
          gate_driver_disable();
        }
      }

      /* A Break IRQ may replace the requested state during preparation. */
      if (esc_state != next_state)
      {
        continue;
      }

      ESC_State_Init(next_state);
      if (esc_state == next_state)
      {
        esc_state_old = next_state;
      }
    }
  }
}

/**
  * @brief  Check the core frequency matching the required frequency
  * @param  none
  * @retval none
  */
void check_configured_core_clock(void)
{
  if (system_core_clock != SYSTEM_CORE_CLOCK)
  {
    printf("Core freq.: %d, does not match the required freq. : %d", system_core_clock, SYSTEM_CORE_CLOCK);
    while(1);
  }
}

/**
  * @brief  report whether an ESC state needs the external gate driver
  * @param  state: requested ESC state
  * @retval SET for an active drive state, otherwise RESET
  */
static flag_status esc_state_requires_gate_driver(esc_state_type state)
{
  switch (state)
  {
  case ESC_STATE_RUNNING:
  case ESC_STATE_I_TUNE:
  case ESC_STATE_AUTO_LEARN:
#ifdef MOTOR_PARAM_IDENTIFY
  case ESC_STATE_WINDING_PARAM_ID:
#endif
    return SET;

  default:
    return RESET;
  }
}

/**
  * @brief  latch a gate-driver fault and discard the previous run request
  * @param  none
  * @retval none
  */
static void gate_driver_fault_latch(void)
{
  ui_wave_param.user_define_b = OVERCURRENT_DIAG_GATE_DRIVER;
  gate_driver_emergency_shutdown();
  foc_rdy = RESET;
  start_stop_btn_flag = RESET;
  error_code |= error_code_mask & MC_OVER_CURRENT_ERROR;
  esc_state = ESC_STATE_ERROR;
}

/**
  * @}
  */
