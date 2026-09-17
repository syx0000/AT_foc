/**
  **************************************************************************
  * @file     mc_foc_esc_state_hall_ebike.c
  * @brief    State Machine Switching of Motor Vector Control in Hall Sensor-Based E-bikes
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

/** @defgroup mc_foc_esc_state_hall_ebike
  * @brief State Machine Switching of Motor Vector Control in Hall Sensor-Based E-bikes.
  * @{
  */

int16_t braking_cmd;

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

  case ESC_STATE_RUNNING:
    vdc_ratio = calcVdcRatio(vref_cal_ratio);
    hall_cw_ctrl_para();
    if (rotor_speed_val_filt == 0)
    {
      charge_boot_cap(&pwm_duty);
    }
    pwm_switch_on();
    foc_rdy = SET;
    break;

  case ESC_STATE_FREE_RUN:
    foc_rdy = RESET;
    pwm_switch_off();
    param_clear();
    break;

  case ESC_STATE_BRAKING:
    if (rotor_speed_val > STABLE_SPEED_RPM)
    {
#if defined IPM_MTPA_MTPV_CTRL
      braking_cmd = BRAKING_TORQUE_PU;
#else
      braking_cmd = BRAKING_CURRENT_PU;
#endif
      if (ctrl_mode == TORQUE_CTRL)
      {
        pid_spd_trq.lower_limit_output = braking_cmd;
        pid_spd_trq.lower_limit_integral = (braking_cmd << pid_spd_trq.ki_shift);
        speed_ramp.cmd_final = 0;
        speed_ramp.command = 0;
      }
      else if (ctrl_mode == SPEED_CTRL)
      {
        pid_spd.lower_limit_output = braking_cmd;
        pid_spd.lower_limit_integral = (braking_cmd << pid_spd_trq.ki_shift);
        speed_ramp.cmd_final = 0;
      }
    }
    break;

  case ESC_STATE_ERROR:
    foc_rdy = RESET;
    pwm_switch_off();
    param_clear();
    start_stop_btn_flag = RESET;
    led_on(ERROR_LED_PORT, ERROR_LED_GPIO_PIN);
    break;

  case ESC_STATE_I_TUNE:
    /* get dc bus voltage value */
    I_auto_tune.Vdc_rated_adc = (uint32_t)adc_in_tab[ADC_BUS_VOLT_IDX]*vref_cal_ratio>>14;
    vdc_ratio = calcVdcRatio(vref_cal_ratio);
    hall_learn.learn_volt = *(intCoeffs32_p+MC_PROTOCOL_REG_HALL_LEARN_VOLT);
    hall_learn.learn_volt = adjustValueByVdc(hall_learn.learn_volt,vdc_ratio);
    param_clear();
    pwm_switch_on();
    foc_rdy = SET;
    break;

  case ESC_STATE_REVERSE_RUN:
    vdc_ratio = calcVdcRatio(vref_cal_ratio);
    hall_ccw_ctrl_para();
#if defined IPM_MTPA_MTPV_CTRL
    Iq_ref_cmd = REVERSE_TORQUE_PU;
#else
    Iq_ref_cmd = REVERSE_CURRENT_PU;
#endif
    if (ctrl_mode == TORQUE_CTRL)
    {
      pid_spd_trq.lower_limit_output = Iq_ref_cmd;
      pid_spd_trq.lower_limit_integral = (Iq_ref_cmd << pid_spd_trq.ki_shift);
      speed_ramp.cmd_final = REVERSE_MAX_SPEED_RPM;
      speed_ramp.command = REVERSE_MAX_SPEED_RPM;
    }
    else if (ctrl_mode == SPEED_CTRL)
    {
      pid_spd.lower_limit_output = Iq_ref_cmd;
      pid_spd.lower_limit_integral = (Iq_ref_cmd << pid_spd.ki_shift);
      speed_ramp.cmd_final = REVERSE_MAX_SPEED_RPM;
    }
    pwm_switch_on();
    foc_rdy = SET;
    break;

  case ESC_STATE_ANTI_THEFT:
    vdc_ratio = calcVdcRatio(vref_cal_ratio);
    hall_cw_ctrl_para();
    charge_boot_cap(&pwm_duty);
    pwm_switch_on();
    foc_rdy = SET;
    lock_motor_current = ANTI_THEFT_INIT_CURRENT_PU;
    lock_motor_inc_current = ANTI_THEFT_INC_CURRENT_PU;
    lock_motor_time = 1;
    /* get hall state */
    error_code |= error_code_mask & read_hall_state(&hall);
    rotor_angle_hall.elec_angle_val = hall_startup_theta_table[hall.state];
    hall.theta_inc = 0;
    hall.pre_state = hall.state;    //update hall sensor state
    lock_motor_hall_state = hall.state;
    break;

  case ESC_STATE_PARKING_LOCK:
    vdc_ratio = calcVdcRatio(vref_cal_ratio);
    hall_cw_ctrl_para();
    charge_boot_cap(&pwm_duty);
    pwm_switch_on();
    foc_rdy = SET;
    lock_motor_current = PARKING_LOCK_INIT_CURRENT_PU;
    lock_motor_inc_current = PARKING_LOCK_INC_CURRENT_PU;
    /* get hall state */
    error_code |= error_code_mask & read_hall_state(&hall);
    rotor_angle_hall.elec_angle_val = hall_startup_theta_table[hall.state];
    hall.theta_inc = 0;
    hall.pre_state = hall.state;    //update hall sensor state
    lock_motor_hall_state = hall.state;
    lock_motor_time = 1;
    break;

  case ESC_STATE_AUTO_LEARN:
    I_auto_tune.Vdc_rated_adc = (uint32_t)adc_in_tab[ADC_BUS_VOLT_IDX]*vref_cal_ratio>>14;
    vdc_ratio = calcVdcRatio(vref_cal_ratio);
    hall_learn.learn_volt = *(intCoeffs32_p+MC_PROTOCOL_REG_HALL_LEARN_VOLT);
    hall_learn.learn_volt = adjustValueByVdc(hall_learn.learn_volt,vdc_ratio);
    sys_counter = 0;
    ctrl_mode_cmd = OPEN_LOOP_CTRL;
    ctrl_mode = ctrl_mode_cmd;
    ctrl_mode_old = ctrl_mode;
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
  switch(esc_state_handler)
  {
  case ESC_STATE_IDLE:
    if (anti_theft_flag != RESET && ctrl_mode == TORQUE_CTRL)
    {
      esc_state = ESC_STATE_ANTI_THEFT;
    }
    else if((sp_value <= 0) && (param_initial_rdy != RESET) && (curr_offset_rdy != RESET) && (reverse_flag != SET))
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
      else if(ctrl_mode == OPEN_LOOP_CTRL || ctrl_mode == VF_CTRL)
      {
        esc_state = ESC_STATE_RUNNING;
      }
    }
    if(ctrl_mode == TORQUE_CTRL)
    {
      if (anti_theft_flag != RESET)
      {
        esc_state = ESC_STATE_ANTI_THEFT;
      }
      else if (parking_lock_flag != RESET)
      {
        esc_state = ESC_STATE_PARKING_LOCK;
      }
      else if (brake_flag == RESET)
      {
        if (reverse_flag == SET)
        {
          esc_state = ESC_STATE_REVERSE_RUN;
        }
        else if (Iq_ref_cmd > 0)
        {
          esc_state = ESC_STATE_RUNNING;
        }
      }
    }
    else if (ctrl_mode == SPEED_CTRL)
    {
      if (brake_flag == RESET)
      {
        if (reverse_flag == SET)
        {
          esc_state = ESC_STATE_REVERSE_RUN;
        }
        else if (speed_ramp.cmd_final > 0)
        {
          esc_state = ESC_STATE_RUNNING;
        }
      }
    }

    if(hall_learn.start_flag != RESET)
    {
      esc_state = ESC_STATE_AUTO_LEARN;
    }

    if(I_auto_tune.state_flag == PROCESSING)
    {
      I_auto_tune.Vdc_rated_adc = (uint32_t)adc_in_tab[ADC_BUS_VOLT_IDX]*vref_cal_ratio>>14;
      vdc_ratio = calcVdcRatio(vref_cal_ratio);
      hall_learn.learn_volt = *(intCoeffs32_p+MC_PROTOCOL_REG_HALL_LEARN_VOLT);
      hall_learn.learn_volt = adjustValueByVdc(hall_learn.learn_volt,vdc_ratio);
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

    if(ctrl_mode == TORQUE_CTRL)
    {
      if (brake_flag == SET)
      {
        Iq_ref_cmd = 0;
        esc_state = ESC_STATE_BRAKING;
      }
      else if (Iq_ref_cmd > 0)
      {
        curr_cmd_handler_ebike(&pid_spd_trq, &speed_ramp, Iq_ref_cmd);
      }
      else if (Iq_ref_cmd == 0)
      {
        esc_state = ESC_STATE_BRAKING;
      }
    }
    else if(ctrl_mode == SPEED_CTRL)
    {
      if (brake_flag == SET)
      {
        speed_ramp.cmd_final = 0;
        esc_state = ESC_STATE_BRAKING;
      }
      else if (speed_ramp.cmd_final == 0)
      {
        esc_state = ESC_STATE_BRAKING;
      }
      command_ramp(&speed_ramp);
    }

    if (start_stop_btn_flag == RESET && (ctrl_mode == OPEN_LOOP_CTRL || ctrl_mode == VF_CTRL))
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

    if(ctrl_mode == TORQUE_CTRL)
    {
      if (Iq_ref_cmd > 0 && brake_flag == RESET)
      {
        esc_state = ESC_STATE_RUNNING;
      }
    }
    else if(ctrl_mode == SPEED_CTRL)
    {
      if (speed_ramp.cmd_final > 0 && brake_flag == RESET)
      {
        esc_state = ESC_STATE_RUNNING;
      }
      command_ramp(&speed_ramp);
    }

    break;

  case ESC_STATE_BRAKING:
    if (ctrl_mode == TORQUE_CTRL)
    {
      if (Iq_ref_cmd > 0 && brake_flag == RESET)
      {
        esc_state = ESC_STATE_RUNNING;
      }
      else if (Iq_ref_cmd <= 0 && rotor_speed_val < STABLE_SPEED_RPM)
      {
        esc_state = ESC_STATE_FREE_RUN;
      }
    }
    else if (ctrl_mode == SPEED_CTRL)
    {
      if (speed_ramp.cmd_final > 0 && brake_flag == RESET)
      {
        esc_state = ESC_STATE_RUNNING;
      }
      else if (speed_ramp.cmd_final == 0 && rotor_speed_val < STABLE_SPEED_RPM)
      {
        esc_state = ESC_STATE_FREE_RUN;
      }
      command_ramp(&speed_ramp);
    }
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

  case ESC_STATE_REVERSE_RUN:
    if (brake_flag == SET)
    {
      esc_state = ESC_STATE_FREE_RUN;
    }
    else if (reverse_flag == SET)
    {
      if (ctrl_mode == TORQUE_CTRL)
      {
#if defined IPM_MTPA_MTPV_CTRL
        Iq_ref_cmd = REVERSE_TORQUE_PU;
#else
        Iq_ref_cmd = REVERSE_CURRENT_PU;
#endif
        speed_ramp.cmd_final = REVERSE_MAX_SPEED_RPM;
      }
      else if(ctrl_mode == SPEED_CTRL)
      {
        speed_ramp.cmd_final = REVERSE_MAX_SPEED_RPM;
      }
    }
    else
    {
      esc_state = ESC_STATE_FREE_RUN;
    }
    if(ctrl_mode == SPEED_CTRL)
    {
      command_ramp(&speed_ramp);
    }
    break;

  case ESC_STATE_ANTI_THEFT:
    if (anti_theft_flag != RESET)
    {
      if (lock_motor_time > ANTI_THEFT_LOCK_TIME)
      {
        lock_motor_current -= ANTI_THEFT_DEC_CURRENT_PU;

        if (lock_motor_current < ANTI_THEFT_INIT_CURRENT_PU)
          lock_motor_current = ANTI_THEFT_INIT_CURRENT_PU;
      }
      else if (hall.state != lock_motor_hall_state)
      {
        if (lock_motor_current < MAX_LOCK_CURRENT_PU)
          lock_motor_current += lock_motor_inc_current;
      }
      current.Iqdref.d = lock_motor_current;
      current.Iqdref.q = 0;
      lock_motor_time++;
      if (lock_motor_time >= 0xFFFFFFFE)
        lock_motor_time = 0xFFFFFFFE;
    }
    else
    {
      current.Iqdref.d = 0;
      current.Iqdref.q = 0;
      lock_motor_time = 0;
      lock_motor_current = 0;
      esc_state = ESC_STATE_IDLE;
    }
    break;

  case ESC_STATE_PARKING_LOCK:
    if (parking_lock_flag != RESET)
    {
      if (lock_motor_time > PARKING_LOCK_TIME)
      {
        lock_motor_current -= PARKING_LOCK_DEC_CURRENT_PU;

        if (lock_motor_current < PARKING_LOCK_INIT_CURRENT_PU)
          lock_motor_current = PARKING_LOCK_INIT_CURRENT_PU;
      }
      else if (hall.state != lock_motor_hall_state)
      {
        if (lock_motor_current < MAX_LOCK_CURRENT_PU)
          lock_motor_current += lock_motor_inc_current;
      }

      current.Iqdref.d = lock_motor_current;
      current.Iqdref.q = 0;
      lock_motor_time++;
      if (lock_motor_time >= 0xFFFFFFFE)
        lock_motor_time = 0xFFFFFFFE;
    }
    else
    {
      current.Iqdref.d = 0;
      current.Iqdref.q = 0;
      lock_motor_time = 0;
      lock_motor_current = 0;
      pwm_switch_off();
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
