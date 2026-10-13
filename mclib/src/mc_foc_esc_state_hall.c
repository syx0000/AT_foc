/**
  **************************************************************************
  * @file     mc_foc_esc_state_hall.c
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
#include "mc_ident_diag.h"

/** @addtogroup Motor_Control_Library
  * @{
  */

/** @defgroup mc_foc_esc_state_hall
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
    foc_rdy = RESET;
    pwm_switch_off();
    param_clear();
    led_off(ERROR_LED_PORT, ERROR_LED_GPIO_PIN);
    break;

  case ESC_STATE_SAFETY_READY:
    foc_rdy = RESET;
    pwm_switch_off();
    param_clear();
#ifdef LOW_SPEED_VOLT_CTRL
    set_normal_pwm_mode();
#endif
    break;

  case ESC_STATE_ANGLE_INIT:
    break;

  case ESC_STATE_STARTING:
    break;

  case ESC_STATE_RUNNING:
    ui_wave_param.user_define_b = OVERCURRENT_DIAG_NONE;
    if ((ctrl_mode == POSITION_CTRL) || (ctrl_mode == SPEED_CTRL))
    {
      if (((rotor_speed_val == 0) && (speed_ramp.cmd_final >= 0)) || (rotor_speed_val > 0))
      {
        hall_cw_ctrl_para();
      }
      else
      {
        hall_ccw_ctrl_para();
      }
#ifdef LOW_SPEED_VOLT_CTRL
      /*
       * STARTUP_VOLTAGE is otherwise used only by sensorless startup.
       * Seed the Hall voltage PI at standstill to overcome static friction.
       */
      if (abs(rotor_speed_val) < SLICK_SPEED_RPM)
      {
        if (rotor_speed_hall.dir == CW)
        {
          pid_spd_volt.integral =
            ((int32_t)STARTUP_VOLTAGE_CMD << pid_spd_volt.ki_shift);
        }
        else
        {
          pid_spd_volt.integral =
            -((int32_t)STARTUP_VOLTAGE_CMD << pid_spd_volt.ki_shift);
        }
      }
#endif
    }
    else if (ctrl_mode == TORQUE_CTRL)
    {
      if (((rotor_speed_val == 0) && (current.Iqdref.q >= 0)) || (rotor_speed_val > 0))
      {
        hall_cw_ctrl_para();
      }
      else
      {
        hall_ccw_ctrl_para();
      }
    }
    else
    {
      hall_cw_ctrl_para();
    }
#if !defined HALL_EXINT_EDGE_CAPTURE
    /*
     * TMR XOR capture keeps running while the motor is stopped.  Reset its
     * phase here so the first Hall edge cannot turn a stale counter remainder
     * into a false high-speed sample.  Saturating the first interval also
     * makes startup independent of the rotor's initial position in a sector.
     */
    tmr_interrupt_enable(HALL_CAPTURE_TIMER,
                         TMR_TRIGGER_INT | TMR_C4_INT, FALSE);
    rotor_speed_hall.filtered = 0;
    hall.theta_inc = 0;
    hall.offset = MAX_CAP_COUNT;
    hall.double_interval = MAX_CAP_COUNT;
    reset_ma_buffer(hall_interval_moving_average);
    tmr_counter_value_set(HALL_CAPTURE_TIMER, 0);
    tmr_channel_value_set(HALL_CAPTURE_TIMER, TMR_SELECT_CHANNEL_4,
                          MAX_CAP_COUNT);
    tmr_flag_clear(HALL_CAPTURE_TIMER,
                   TMR_TRIGGER_FLAG | TMR_C1_FLAG |
                   TMR_C4_FLAG | TMR_OVF_FLAG);
    error_code |= error_code_mask &
                  hall_at_zero_speed(&hall, &rotor_angle_hall,
                                     hall_next_state_table);
    tmr_interrupt_enable(HALL_CAPTURE_TIMER,
                         TMR_TRIGGER_INT | TMR_C4_INT, TRUE);
#endif
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
#ifdef MOTOR_PARAM_IDENTIFY
    if ((esc_state_old == ESC_STATE_WINDING_PARAM_ID) &&
        (motor_param_ident.state_flag == PROCESSING))
    {
      motor_parameter_id_abort();
    }
#endif
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
    sys_counter = 0;
    ctrl_mode_cmd = OPEN_LOOP_CTRL;
    ctrl_mode = ctrl_mode_cmd;
    ctrl_mode_old = ctrl_mode;
    hall_learn.dir ^= 1;
    hall_learn.process_state = PROCESS_0_LOCK;
    break;
#ifdef MOTOR_PARAM_IDENTIFY
  case ESC_STATE_WINDING_PARAM_ID:
    motor_param_ident.id_flag = RESET;
    motor_param_ident.step_flag = 0;
    motor_param_ident.count = 0;
    motor_param_ident.duty = 0;
#if defined USE_MOTOR_MONITOR
    ui_wave_param.user_define_a = 0;
    ui_wave_param.user_define_b = 0;
#endif
    motor_parameter_ID_config();
    motor_param_ident.timeout_count = 0;
#if defined USE_UART_LOG
    motor_ident_diag_reset();
    motor_ident_diag_capture(IDENT_DIAG_START);
#endif
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
      else
      {
        esc_state = ESC_STATE_RUNNING;
      }
    }

    if(hall_learn.start_flag != RESET)
    {
      esc_state = ESC_STATE_AUTO_LEARN;
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
    break;

  case ESC_STATE_STARTING:
    break;

  case ESC_STATE_RUNNING:
    if (ctrl_mode == SPEED_CTRL || ctrl_mode == POSITION_CTRL)
    {
      command_ramp(&speed_ramp);
    }

    if (rotor_speed_val == 0)
    {
      if ((ctrl_mode == POSITION_CTRL) || (ctrl_mode == SPEED_CTRL) || (ctrl_mode == TORQUE_CTRL))
      {
        if (current.Iqdref.q >= 0)
        {
          hall_cw_ctrl_para();
        }
        else
        {
          hall_ccw_ctrl_para();
        }
        if ((pid_spd.integral * rotor_speed_hall.dir) < 0)
          pid_spd.integral = 0;
      }
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
    if (hall_learn.start_flag == RESET)
    {
#if defined HALL_EXINT_EDGE_CAPTURE
      hall_timer_init();
#endif
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
