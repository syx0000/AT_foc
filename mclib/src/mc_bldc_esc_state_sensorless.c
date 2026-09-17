/**
  **************************************************************************
  * @file     mc_bldc_esc_state_sensorless.c
  * @brief    State Machine Switching of Motor Vector Control in sensor-less Motor
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

/** @addtogroup Motor_Control_Library
  * @{
  */

/** @defgroup mc_bldc_esc_state_sensorless
  * @brief State Machine Switching of Motor Vector Control in Hall Sensor-Based Motor.
  * @{
  */
/**
  * @brief  Initialize ESC state handler.(Typically called when ESC state changes)
  * @param  esc_state_handler: ESC state handler structure
  * @retval none
  */
void ESC_State_Init(esc_state_type esc_state_handler)
{
  switch(esc_state_handler)
  {
  case ESC_STATE_IDLE:
    disable_mosfet(PWM_ADVANCE_TIMER);
    bldc_rdy = RESET;
    closeloop_rdy = RESET;
#if defined AT32M412xx
    led_off(LED_R_PORT, LED_R_GPIO_PIN);
#endif
    break;

  case ESC_STATE_SAFETY_READY:
    disable_mosfet(PWM_ADVANCE_TIMER);
    param_clear();
    sys_counter = 0;
    speed_LPF.output_temp = 0;
    bldc_rdy = RESET;
    closeloop_rdy = RESET;
    const_current_ctrl = RESET;
    change_phase_flag = RESET;
#if defined (BLDC_SENSORLESS_ADC) && defined (EMF_PULL_UP)
    /* disable channel in interrupt*/
    tmr_interrupt_enable(PWM_ADVANCE_TIMER, TMR_C1_INT, FALSE);
#elif defined (INTERNAL_COMP)
    cmp_blanking_config(BEMF_COMP, CMP_BLANKING_SOURCE);
#endif
    break;

  case ESC_STATE_ANGLE_INIT:
    sys_counter = 0;
    closeloop_rdy = RESET;
    start_up_config();
    break;

  case ESC_STATE_STARTING:
    sys_counter = 0;
    /* Set next hall state for detecting */
    set_next_detect_hall_state();
    /* Set start-up voltage command */
    set_starting_volt();

    bldc_detectEMF_param_init(&adc_sample);
    bldc_sensorless_detectEMF_config(&adc_sample);
    bldc_rdy = SET;
    closeloop_rdy = SET;

#if defined (BLDC_SENSORLESS_ADC) && defined (EMF_PULL_UP)
    /* clear interupt flag */
    tmr_flag_clear(PWM_ADVANCE_TIMER, TMR_C1_FLAG | TMR_C2_FLAG | TMR_C3_FLAG | TMR_C4_FLAG);

    /* enable channel in interrupt*/
    tmr_interrupt_enable(PWM_ADVANCE_TIMER, TMR_C1_INT, TRUE);
#endif

    /* Start bldc setting */
    start_bldc();
    break;

  case ESC_STATE_RUNNING:
    bldc_rdy = SET;

    if(ctrl_mode != OPEN_LOOP_CTRL)
    {
      closeloop_rdy = SET;
    }
    else
    {
      closeloop_rdy = RESET;
    }

    const_current_ctrl = RESET;
    sys_counter = 0;
    break;

  case ESC_STATE_FREE_RUN:
    disable_mosfet(PWM_ADVANCE_TIMER);
#if defined (INTERNAL_COMP)
    cmp_blanking_config(BEMF_COMP, CMP_BLANKING_NONE);
    adc_preempt_channel_set(ADC_INSTANT_CONVERTER, BEMF_A_ADC_CH, 1, ADC_SAMPLETIME_7_5);
    adc_preempt_channel_set(ADC_INSTANT_CONVERTER, BEMF_B_ADC_CH, 2, ADC_SAMPLETIME_7_5);
    adc_preempt_channel_set(ADC_INSTANT_CONVERTER, BEMF_C_ADC_CH, 3, ADC_SAMPLETIME_7_5);
    tmr_counter_enable(COMP_OUT_CAPTURE_TIMER, FALSE);
    dma_channel_enable(DMA_CHANNEL_BLANK_TRIGGER, FALSE);
#endif
    bldc_rdy = RESET;
    closeloop_rdy = RESET;
    current.Ibus.Iref = 0;
    current.Ibus.Ireal_pu = 0;
    speed_ramp.cmd_final = 0;
    speed_ramp.command = 0;
    break;

  case ESC_STATE_BRAKING:
    disable_mosfet(PWM_ADVANCE_TIMER);
    bldc_rdy = RESET;
    closeloop_rdy = RESET;
    start_stop_btn_flag = RESET;
    param_clear();
    break;

  case ESC_STATE_ERROR:
    disable_mosfet(PWM_ADVANCE_TIMER);
    bldc_rdy = RESET;
    closeloop_rdy = RESET;
    start_stop_btn_flag = RESET;
    param_clear();
#if defined AT32M412xx
    led_on(LED_R_PORT, LED_R_GPIO_PIN);
#endif
    break;

  case ESC_STATE_I_TUNE:
    /* set flag status */
    bldc_rdy = SET;
    closeloop_rdy = SET;
    current_loop_ctrl = SET;
    const_current_ctrl = RESET;
    change_phase_flag = SET;

    bldc_detectEMF_param_init(&adc_sample);
    bldc_sensorless_detectEMF_config(&adc_sample);

    motor_cw_init();
    /* set pwm output mode in shadow buffer of timer cctrl reg. */
    hall.state = HALL_LEARN_4_STATE;
    bldc_output_config(hall.state);
    /* update pwm output mode from shadow buffer of timer cctrl reg. */
    tmr_event_sw_trigger(PWM_ADVANCE_TIMER, TMR_HALL_SWTRIG);
    /* reset counter value */
    tmr_counter_value_set(PWM_ADVANCE_TIMER, 0);
    tmr_counter_value_set(ADC_TIMER, 0);
    /* enable adc timer */
    tmr_counter_enable(ADC_TIMER, TRUE);
    /* enable pwm timer */
    tmr_counter_enable(PWM_ADVANCE_TIMER, TRUE);
    /* enable pwm timer output */
    tmr_output_enable(PWM_ADVANCE_TIMER, TRUE);
    break;

  case ESC_STATE_AUTO_LEARN:
    break;
#ifdef MOTOR_PARAM_IDENTIFY

  case ESC_STATE_WINDING_PARAM_ID:
    motor_parameter_ID_config();
    motor_param_ident.timeout_count = 0;
    break;
#endif

  case ESC_STATE_NONE:
    break;
  }
}

/**
  * @brief  ESC State Machine Task Function
  * @param  esc_state_handler: ESC state handler structure
  * @retval none
  */
void ESC_State_Task(esc_state_type esc_state_handler)
{
  int32_t spd_err;

  switch(esc_state_old)
  {
  case ESC_STATE_IDLE:
    if((sp_value <= 0) && (param_initial_rdy != RESET) && (curr_offset_rdy != RESET))
    {
      esc_state = ESC_STATE_SAFETY_READY;
    }
    else
    {
      esc_state = ESC_STATE_IDLE;
    }

    break;

  case ESC_STATE_SAFETY_READY:
    if(ctrl_mode == ID_MANUAL_TUNE || ctrl_mode == IQ_MANUAL_TUNE)
    {
      if(start_stop_btn_flag == SET)
      {
        esc_state = ESC_STATE_I_TUNE;
      }
    }
    else if(ctrl_mode == SPEED_CTRL)
    {
      if((speed_ramp.cmd_final != 0) && (start_stop_btn_flag == SET))
      {
        if(speed_ramp.cmd_final > 0)        /* CW*/
        {
          motor_cw_init();
        }
        else if(speed_ramp.cmd_final < 0)   /* CCW*/
        {
          motor_ccw_init();
        }

        esc_state = ESC_STATE_ANGLE_INIT;
      }
      else
      {
        start_stop_btn_flag = RESET;
        esc_state = ESC_STATE_SAFETY_READY;
      }
    }
    else if(ctrl_mode == TORQUE_CTRL)
    {
      if((current.Ibus.Iref != 0) && (start_stop_btn_flag == SET))
      {
        if(current.Ibus.Iref > 0)        /* CW*/
        {
          motor_cw_init();
        }
        else if(current.Ibus.Iref < 0)   /* CCW*/
        {
          motor_ccw_init();
        }

        esc_state = ESC_STATE_ANGLE_INIT;
      }
      else
      {
        start_stop_btn_flag = RESET;
        esc_state = ESC_STATE_SAFETY_READY;
      }
    }
    else if(ctrl_mode == OPEN_LOOP_CTRL)
    {
      if(start_stop_btn_flag == SET)
      {
        openloop.period_ref = openloop.olc_init_period;
        openloop.volt_ref = calcValueByVref(openloop.olc_init_volt);
        esc_state = ESC_STATE_STARTING;
      }
      else
      {
        esc_state = ESC_STATE_SAFETY_READY;
      }
    }

    if(I_auto_tune.state_flag == PROCESSING)
    {
      current_auto_tuning(&I_auto_tune);
      set_current_pid_param(&I_auto_tune, &pid_is);
      I_auto_tune.state_flag = SUCCEED;
    }

    break;

  case ESC_STATE_ANGLE_INIT:
    /* Determine the initial position at the beginning(BLDC sensor-less) */
    angle_init_func();

    /* Stop motor */
    if(start_stop_btn_flag == RESET)
    {
      esc_state = ESC_STATE_FREE_RUN;
    }

    break;

  case ESC_STATE_STARTING:
    if(ctrl_mode == OPEN_LOOP_CTRL)
    {
      esc_state = ESC_STATE_RUNNING;
    }
    else
    {
      if (sense_hall_steps < SENSE_HALL_TIMES)
      {
        sys_counter++;

        if (sys_counter >= REBOOT_PERIOD_MS)
        {
          esc_state = ESC_STATE_SAFETY_READY;
        }
      }
      else
      {
        esc_state = ESC_STATE_RUNNING;
        start_state = START_STATE_STABLE_RUN;
        sys_counter = 0;
        /* Set parameter for switching to closed-loop control */
        rdy_to_close_loop_param();
      }
    }

    /* Stop motor */
    if(start_stop_btn_flag == RESET)
    {
      esc_state = ESC_STATE_FREE_RUN;
    }

    break;

  case ESC_STATE_RUNNING:
    if(ctrl_mode == SPEED_CTRL)
    {
      if (start_stop_btn_flag == RESET)
      {
        esc_state = ESC_STATE_FREE_RUN;
      }
      else
      {
        if(abs(speed_ramp.cmd_final) < MIN_SPEED_RPM)
        {
          if(rotor_speed.dir == CW)
          {
            speed_ramp.cmd_final = MIN_SPEED_RPM;
          }
          else
          {
            speed_ramp.cmd_final = -MIN_SPEED_RPM;
          }
        }
      }

      command_ramp(&speed_ramp);
      spd_err = speed_ramp.command - rotor_speed.filtered;
      current.Ibus.Iref = pid_controller(&pid_spd, spd_err);

#if defined LOW_SPEED_VOLT_CTRL && !defined WITHOUT_CURRENT_CTRL
      volt_cmd = pid_controller(&pid_spd_volt, spd_err);

      /* low - speed control */
      if(current_loop_ctrl != SET && abs(speed_ramp.command) >= HYSTERESIS_HIGH_SPEED)    /* switch to I-control*/
      {
        current_loop_ctrl = SET;
      }
      else if(current_loop_ctrl != RESET && abs(speed_ramp.command) <= HYSTERESIS_LOW_SPEED)  /* switch to V-control*/
      {
        current_loop_ctrl = RESET;
      }

#elif defined WITHOUT_CURRENT_CTRL
      volt_cmd = pid_controller(&pid_spd_volt, spd_err);
      current_loop_ctrl = RESET;
#else
      current_loop_ctrl = SET;
#endif
    }
    else if(ctrl_mode == TORQUE_CTRL)
    {
      current_loop_ctrl = SET;

      if (start_stop_btn_flag == RESET)
      {
        esc_state = ESC_STATE_FREE_RUN;
      }
      else
      {
        esc_state = ESC_STATE_RUNNING;
      }
    }
    else if(ctrl_mode == OPEN_LOOP_CTRL)
    {
      if (start_stop_btn_flag == RESET)
      {
        esc_state = ESC_STATE_FREE_RUN;
      }
      else
      {
        closeloop_rdy = RESET;

        /* 5 ms inc openloop volt & spd */
        open_loop_cmd_ramp(&openloop);
      }
    }

    break;

  case ESC_STATE_FREE_RUN:
    if (rotor_speed.filtered == 0)
    {
      esc_state = ESC_STATE_SAFETY_READY;
    }
    else
    {
      esc_state = ESC_STATE_FREE_RUN;
    }

    break;

  case ESC_STATE_BRAKING:
    break;

  case ESC_STATE_ERROR:
    if(error_code == MC_NO_ERROR)
    {
      esc_state = ESC_STATE_IDLE;
    }

    break;

  case ESC_STATE_I_TUNE:
    if (start_stop_btn_flag == SET)
    {
      I_tune_manual();
    }
    else
    {
      current.Ibus.Iref = 0;
      esc_state = ESC_STATE_FREE_RUN;
    }

    break;

  case ESC_STATE_AUTO_LEARN:
    break;
#ifdef MOTOR_PARAM_IDENTIFY
  case ESC_STATE_WINDING_PARAM_ID:
    motor_parameter_id_process();
    break;
#endif

  case ESC_STATE_NONE:
    break;
  }
}
