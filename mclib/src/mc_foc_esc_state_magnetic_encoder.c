/**
  **************************************************************************
  * @file     mc_foc_esc_state_magnetic_encoder.c
  * @brief    State Machine Switching of Motor Vector Control in Magnetic Encoder Sensor-Based
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

/** @defgroup mc_foc_esc_state_magnetic_encoder
  * @brief State Machine Switching of Motor Vector Control in Magnetic Encoder Sensor-Based Motor.
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
    break;

  case ESC_STATE_STARTING:
    break;

  case ESC_STATE_ENC_ALIGN:
    if (ctrl_mode != OPEN_LOOP_CTRL)
    {
      if (encoder.align != SUCCEED)
      {
        encoder.align = PROCESSING;
        write_flash_flag = SET;
      }
      pwm_switch_on();
      foc_rdy = SET;
    }
    break;

  case ESC_STATE_RUNNING:
    charge_boot_cap(&pwm_duty);
    pwm_switch_on();
    foc_rdy = SET;
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
#ifdef ANGLE_CALIBRATION
    if (encoder.calibrate_flag == SET)
    {
      encoder.calibrate_state = PROCESS_6_ERROR;
      encoder.calibrate_count = 0;
      encoder.calibrate = FAILED;
      encoder.calibrate_flag = RESET;
      write_flash_flag = RESET;
    }
#endif
    led_on(ERROR_LED_PORT, ERROR_LED_GPIO_PIN);
    break;

  case ESC_STATE_I_TUNE:
    param_clear();
    pwm_switch_on();
    foc_rdy = SET;
    break;

  case ESC_STATE_AUTO_LEARN:
#ifdef ANGLE_CALIBRATION
    encoder.calibrate_count = 0;
    encoder.calibrate_state = PROCESS_1_ALIGN;
    pwm_switch_on();
    foc_rdy = SET;
    encoder.align = UNDO;
    encoder.calibrate = PROCESSING;
#endif
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
        esc_state = ESC_STATE_ENC_ALIGN;
      }
    }
#ifdef ANGLE_CALIBRATION
    if(encoder.calibrate_flag != RESET)
    {
      esc_state = ESC_STATE_AUTO_LEARN;
    }
#endif
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
    break;

  case ESC_STATE_STARTING:
    break;

  case ESC_STATE_ENC_ALIGN:
    if (ctrl_mode != OPEN_LOOP_CTRL)
    {
      if (encoder.align == SUCCEED)
      {
        esc_state = ESC_STATE_RUNNING;
      }
    }
    else
    {
      esc_state = ESC_STATE_RUNNING;
    }

    break;

  case ESC_STATE_RUNNING:
    if (ctrl_mode == SPEED_CTRL || ctrl_mode == POSITION_CTRL)
    {
      command_ramp(&speed_ramp);
    }
#if defined TORQUE_CTRL_WITH_SPEED_LIMIT
    else if(ctrl_mode == TORQUE_CTRL)
    {
      curr_cmd_handler_speed_limit();
    }
#endif

    if (ctrl_mode != OPEN_LOOP_CTRL && encoder.align != SUCCEED)
    {
      esc_state = ESC_STATE_ENC_ALIGN;
    }

    if (start_stop_btn_flag == RESET)
    {
      esc_state = ESC_STATE_FREE_RUN;
    }

    break;

  case ESC_STATE_FREE_RUN:
    if (rotor_speed_val == 0)
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
#ifdef ANGLE_CALIBRATION
    angle_calibrate_process();
#endif
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
