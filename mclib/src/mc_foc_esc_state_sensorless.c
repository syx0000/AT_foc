/**
  **************************************************************************
  * @file     mc_foc_esc_state_sensorless.c
  * @brief    State Machine Switching of Motor Vector Control in Sensorless Motor
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

/** @defgroup mc_foc_esc_state_sensorless
  * @brief State Machine Switching of Motor Vector Control in Sensorless Motor.
  * @{
  */

trig_components_type trig_components_1;

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
    foc_rdy = RESET;
    pwm_switch_off();
    param_clear();
    led_off(ERROR_LED_PORT, ERROR_LED_GPIO_PIN);
    break;

  case ESC_STATE_SAFETY_READY:
    foc_rdy = RESET;
    pwm_switch_off();
    param_clear();
    break;

  case ESC_STATE_ANGLE_INIT:
#if defined INIT_ANGLE_STARTUP
    foc_angle_init_config();
#endif
    break;

  case ESC_STATE_STARTING:
#if (defined INIT_ANGLE_STARTUP && !defined WIND_SENSE)
    adc_ordinary_config();
    adc_preempt_config();
    tmr_pwm_init();
    mc_delay_us(100); /* wait for response of timer configuration*/
    startup.elec_angle = angle_detector.init_elec_angle;
    startup.ol_angle = angle_detector.init_elec_angle;
    state_observer.elec_angle = angle_detector.init_elec_angle;
    charge_boot_cap(&pwm_duty);
    pwm_switch_on();
    foc_rdy = SET;
#elif defined OPENLOOP_STARTUP || defined ALIGN_AND_GO_STARTUP
#ifndef WIND_SENSE
    charge_boot_cap(&pwm_duty);
#endif
    pwm_switch_on();
    foc_rdy = SET;
#endif
    break;

  case ESC_STATE_RUNNING:
    if (ctrl_mode == OPEN_LOOP_CTRL)
    {
      charge_boot_cap(&pwm_duty);
      pwm_switch_on();
      foc_rdy = SET;
    }
    break;

  case ESC_STATE_FREE_RUN:
    foc_rdy = RESET;
    pwm_switch_off();
    param_clear();
    break;

  case ESC_STATE_BRAKING:
    foc_rdy = RESET;
    pwm_switch_off();
    param_clear();
    start_stop_btn_flag = RESET;
    break;

  case ESC_STATE_ERROR:
    foc_rdy = RESET;
    pwm_switch_off();
    param_clear();
    start_stop_btn_flag = RESET;
    led_on(ERROR_LED_PORT, ERROR_LED_GPIO_PIN);
    break;

  case ESC_STATE_I_TUNE:
    param_clear();
    pwm_switch_on();
    foc_rdy = SET;
    break;

  case ESC_STATE_AUTO_LEARN:
    break;
#ifdef MOTOR_PARAM_IDENTIFY
  case ESC_STATE_WINDING_PARAM_ID:
    motor_parameter_ID_config();
    motor_param_ident.timeout_count = 0;
    break;
#endif

#ifdef WIND_SENSE
  case ESC_STATE_SPIN_CHECK:
    charge_boot_cap(&pwm_duty);
    wind_detect_time = 0;
    break;

  case ESC_STATE_HEADWIND_BRAKE:
    tmr_channel_value_set(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_1, 0);
    tmr_channel_value_set(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_2, 0);
    tmr_channel_value_set(PWM_ADVANCE_TIMER, TMR_SELECT_CHANNEL_3, 0);
    pwm_switch_on();
    brake_time = 0;
    break;

  case ESC_STATE_LOCK_MOTOR:
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
  switch(esc_state_handler)
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
    if(start_stop_btn_flag != RESET)
    {
      if(ctrl_mode == ID_MANUAL_TUNE || ctrl_mode == IQ_MANUAL_TUNE)
      {
        esc_state = ESC_STATE_I_TUNE;
      }
      else if(ctrl_mode == OPEN_LOOP_CTRL)
      {
        esc_state = ESC_STATE_RUNNING;
      }
      else
      {
#ifdef WIND_SENSE
        esc_state = ESC_STATE_SPIN_CHECK;
#else
#if  defined OPENLOOP_STARTUP || defined ALIGN_AND_GO_STARTUP
        esc_state = ESC_STATE_STARTING;
#elif defined INIT_ANGLE_STARTUP
        esc_state = ESC_STATE_ANGLE_INIT;
#endif
#endif
      }
    }

    if(I_auto_tune.state_flag == PROCESSING)
    {
      current_auto_tuning(&I_auto_tune);
      set_current_pid_param(&I_auto_tune, &pid_id);
      set_current_pid_param(&I_auto_tune, &pid_iq);
      pid_iq.kp_gain = (int16_t)(pid_id.kp_gain/LD_LQ_RATIO);
      I_auto_tune.state_flag = SUCCEED;
    }
    break;

  case ESC_STATE_ANGLE_INIT:
#if defined INIT_ANGLE_STARTUP
    if(angle_detector.step_count < STEP_8_MAX)
    {
      foc_sensorless_angle_init(&angle_detector, &current);
      angle_detector.step_count++;
      esc_state = ESC_STATE_ANGLE_INIT;
    }
    else
    {
      esc_state = ESC_STATE_STARTING;
    }

#endif
    break;

  case ESC_STATE_STARTING:
    if (startup.closeloop_rdy != RESET)
    {
      speed_ramp.command = rotor_speed_val;
      esc_state = ESC_STATE_RUNNING;
    }
    break;

  case ESC_STATE_RUNNING:
    if (ctrl_mode == SPEED_CTRL)
    {
      command_ramp(&speed_ramp);
    }

    if (start_stop_btn_flag == RESET)
    {
      esc_state = ESC_STATE_FREE_RUN;
    }

    if (startup_counter < DETECT_DELAY_TIME)
    {
      startup_counter++;
    }
    else
    {
      if (ctrl_mode == TORQUE_CTRL)
      {
        if ((current.Iqdref.q > 0 && rotor_speed_val_filt < -DETECT_REVERSE_SPEED) || (current.Iqdref.q < 0 && rotor_speed_val_filt > DETECT_REVERSE_SPEED))
        {
          error_code |= error_code_mask & MC_STARTUP_ERROR;
        }
        else if (current.Iqdref.q == 0)
        {
          start_stop_btn_flag = RESET;
          esc_state = ESC_STATE_FREE_RUN;
        }
        if (abs(rotor_speed_val_filt) > DETECT_MAX_SPEED)
        {
          error_code |= error_code_mask & MC_STARTUP_ERROR;
        }
      }
      else if (ctrl_mode == SPEED_CTRL)
      {
        if ((speed_ramp.cmd_final >= MIN_SPEED_RPM && rotor_speed_val_filt < -DETECT_REVERSE_SPEED) || (speed_ramp.cmd_final <= -MIN_SPEED_RPM && rotor_speed_val_filt > DETECT_REVERSE_SPEED))
        {
          error_code |= error_code_mask & MC_STARTUP_ERROR;
        }
        else if (speed_ramp.cmd_final == 0)
        {
          start_stop_btn_flag = RESET;
          esc_state = ESC_STATE_FREE_RUN;
        }
        if (abs(rotor_speed_val_filt) > DETECT_MAX_SPEED)
        {
          error_code |= error_code_mask & MC_STARTUP_ERROR;
        }
      }
    }

    break;

  case ESC_STATE_FREE_RUN:
#ifdef WIND_SENSE
    esc_state = ESC_STATE_SAFETY_READY;
#else
    if (rotor_speed_val == 0 || stop_counter > FREE_STOP_TIME)
    {
      stop_counter = 0;
      esc_state = ESC_STATE_SAFETY_READY;
    }
    else
    {
      stop_counter++;
      esc_state = ESC_STATE_FREE_RUN;
    }
#endif
    break;

  case ESC_STATE_BRAKING:

    break;

  case ESC_STATE_ERROR:
    if (error_code == MC_NO_ERROR)
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
      current.Iqdref.d = 0;
      current.Iqdref.q = 0;
      esc_state = ESC_STATE_FREE_RUN;
    }

    if(ctrl_mode != ID_MANUAL_TUNE && ctrl_mode != IQ_MANUAL_TUNE)
    {
      esc_state = ESC_STATE_SAFETY_READY;
    }

    break;

  case ESC_STATE_AUTO_LEARN:
    break;
#ifdef MOTOR_PARAM_IDENTIFY
  case ESC_STATE_WINDING_PARAM_ID:
    motor_parameter_id_process();
    break;
#endif

#ifdef WIND_SENSE
  case ESC_STATE_SPIN_CHECK:
    if (wind_detect_time >= WIND_DETECT_TIME)
    {
      if (state_observer.motor_speed.filtered >= TAILWIND_SPEED)
      {
        /* trigonometric functions transformation */
        trig_components_1 = trig_functions(state_observer.elec_angle);

        /* Park transformation */
        foc_park_trans(&current, &trig_components_1);
        park_trans(&(motor_voltage.Vpu), &(volt_cmd.Vqd), &trig_components_1);
        pid_id.integral = volt_cmd.Vqd.d << pid_id.ki_shift;
        pid_iq.integral = volt_cmd.Vqd.q << pid_iq.ki_shift;
        speed_ramp.command = rotor_speed_val;
        startup.closeloop_rdy = SET;
        startup.closeloop_rdy_old = SET;
        foc_rdy = SET;
        pwm_switch_on();

        esc_state = ESC_STATE_RUNNING;
      }
      else if (state_observer.motor_speed.filtered > -TAILWIND_SPEED && state_observer.motor_speed.filtered < TAILWIND_SPEED)
      {
        esc_state = ESC_STATE_STARTING;
      }
      else
      {
        esc_state = ESC_STATE_HEADWIND_BRAKE;
      }
    }
    else
      wind_detect_time++;

    break;

  case ESC_STATE_HEADWIND_BRAKE:
    if (brake_time > HEADWIND_BRAKE_TIME)
    {
      pwm_switch_off();
      esc_state = ESC_STATE_STARTING;
    }
    else
      brake_time++;
    break;

  case ESC_STATE_LOCK_MOTOR:
    break;
#endif
  case ESC_STATE_NONE:
    break;
  }
}
