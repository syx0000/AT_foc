/**
  **************************************************************************
  * @file     mc_bldc_esc_state_hall.c
  * @brief    State Machine Switching of Motor Vector Control in Hall Sensor-Based Motor
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

/** @defgroup mc_bldc_esc_state_hall
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
    break;

  case ESC_STATE_SAFETY_READY:
    disable_mosfet(PWM_ADVANCE_TIMER);
    param_clear();
    sys_counter = 0;
    bldc_rdy = RESET;
    break;

  case ESC_STATE_ANGLE_INIT:
    break;

  case ESC_STATE_STARTING:
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

    start_bldc();
    sys_counter = 0;
    break;

  case ESC_STATE_FREE_RUN:
    disable_mosfet(PWM_ADVANCE_TIMER);
    bldc_rdy = RESET;
    current.Ibus.Iref = 0;
    current.Ibus.Ireal_pu = 0;
    speed_ramp.cmd_final = 0;
    speed_ramp.command = 0;
    break;

  case ESC_STATE_BRAKING:
    break;

  case ESC_STATE_ERROR:
    disable_mosfet(PWM_ADVANCE_TIMER);
    bldc_rdy = RESET;
    param_clear();
    break;

  case ESC_STATE_I_TUNE:
    bldc_rdy = SET;
    closeloop_rdy = SET;
    current_loop_ctrl = SET;
    motor_cw_init();
    /* set pwm output mode in shadow buffer of timer cctrl reg. */
    hall.state = HALL_LEARN_4_STATE;
    bldc_output_config(hall.state);
    /* update pwm output mode from shadow buffer of timer cctrl reg. */
    tmr_event_sw_trigger(PWM_ADVANCE_TIMER, TMR_HALL_SWTRIG);
    /* enable pwm timer output */
    tmr_output_enable(PWM_ADVANCE_TIMER, TRUE);
    break;

  case ESC_STATE_AUTO_LEARN:
    hall_learn_register_setting();
    closeloop_rdy = RESET;
    sys_counter = 0;
    hall_learn.dir ^= 1;
    hall_learn.process_state = PROCESS_0_LOCK;
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
        esc_state = ESC_STATE_RUNNING;
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
        esc_state = ESC_STATE_RUNNING;
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
        esc_state = ESC_STATE_RUNNING;
      }
      else
      {
        esc_state = ESC_STATE_SAFETY_READY;
      }
    }

    if(hall_learn.start_flag != RESET)
    {
      esc_state = ESC_STATE_AUTO_LEARN;
    }

    if(I_auto_tune.state_flag == PROCESSING)
    {
      current_auto_tuning(&I_auto_tune);
      set_current_pid_param(&I_auto_tune, &pid_is);
      I_auto_tune.state_flag = SUCCEED;
    }

    break;

  case ESC_STATE_ANGLE_INIT:
    break;

  case ESC_STATE_STARTING:
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
        if (rotor_speed.filtered == 0)
        {
          if(speed_ramp.cmd_final > 0)        /* CW*/
          {
            motor_cw_init();
          }
          else if(speed_ramp.cmd_final < 0)   /* CCW*/
          {
            motor_ccw_init();
          }
          else  //speed_ramp.cmd_final = 0
          {
            esc_state = ESC_STATE_FREE_RUN;
          }
        }

        command_ramp(&speed_ramp);
      }

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
        if (rotor_speed.filtered == 0)
        {
          if(current.Ibus.Iref > 0)        /* CW*/
          {
            motor_cw_init();
          }
          else if(current.Ibus.Iref < 0)   /* CCW*/
          {
            motor_ccw_init();
          }
        }
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
    if (hall_learn.start_flag == RESET)
    {
      esc_state = ESC_STATE_FREE_RUN;
    }
    else
    {
      hall_learn_process();
      sys_counter++;
    }

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