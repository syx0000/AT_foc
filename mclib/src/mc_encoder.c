/**
  **************************************************************************
  * @file     mc_encoder.c
  * @brief    Encoder-related functions.
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

/** @defgroup mc_encoder
  * @brief Encoder-related functions.
  * @{
  */

/**
  * @brief  reset encoder count function
  * @param  enc_handler: encoder count
  * @param  spd_handler: direction of motor rotating
  * @retval none
  */
void encoder_count_reset(encoder_type *enc_handler, speed_type *spd_handler)
{
  if (spd_handler->dir >= 0) /* CW */
  {
    tmr_counter_value_set(enc_handler->TMRx, 0);
  }
  else  /* CCW */
  {
    tmr_counter_value_set(enc_handler->TMRx, enc_handler->index_count);
  }

  if (enc_handler->align == PROCESSING)
  {
    enc_handler->idx_reset_flag = TRUE;
  }
}

/**
  * @brief  encoder alignment with index function
  * @param  volt_handler: voltage related variables
  * @param  enc_handler: encoder related variables
  * @retval the error code of encoder alignment with index (MC_NO_ERROR or MC_ENCODER_ERROR)
  */
err_code_type encoder_alignment_index(voltage_type *volt_handler, encoder_type *enc_handler)
{
  static int16_t angle_handler;
  static uint32_t count;
  static uint8_t  elec_count;
  trig_components_type trig_components;
  err_code_type error_code_handler = MC_NO_ERROR;

  enc_handler->count = tmr_counter_value_get(enc_handler->TMRx);

  if (enc_handler->idx_reset_flag != TRUE)
  {
    angle_handler += 1;

    if (angle_handler < 0)
    {
      angle_handler += 0x8000;
      elec_count++;
    }

    volt_handler->Vqd.d = 0;
    volt_handler->Vqd.q = enc_handler->volt.q;

    if (elec_count > enc_handler->pole_pairs)
    {
      volt_handler->Vqd.q = 0;
      enc_handler->align = FAILED;
      error_code_handler = MC_ENCODER_ERROR;
      elec_count = 0;
    }
  }
  else
  {
    angle_handler = 0;
    volt_handler->Vqd.d = enc_handler->volt.d;
    volt_handler->Vqd.q = 0;

    if (enc_handler->count >= enc_handler->half_cpr_nbr)
    {
      enc_handler->offset = enc_handler->cpr_nbr - enc_handler->count;
    }
    else
    {
      enc_handler->offset = -enc_handler->count;
    }

    count++;

    if (count >= enc_handler->align_time)
    {
      count = 0;
      volt_handler->Vqd.d = 0;
      enc_handler->idx_reset_flag = FALSE;
      enc_handler->align = SUCCEED;
      ctrl_mode = ctrl_mode_cmd;
    }

    elec_count = 0;
  }

  /* trigonometric functions transformation */
  trig_components = trig_functions(angle_handler);

  /* Inverse Park transformation */
  foc_inver_park_trans(volt_handler, &trig_components);

  return (error_code_handler);
}

/**
  * @brief  encoder alignment without index function
  * @param  volt_handler: voltage related variables
  * @param  enc_handler: encoder related variables
  * @retval none
  */
void encoder_alignment(voltage_type *volt_handler, encoder_type *enc_handler)
{
  static uint32_t count;
  trig_components_type trig_components;

  if (count < (enc_handler->align_time >> 1))
  {
    volt_handler->Vqd.d = 0;
    volt_handler->Vqd.q = enc_handler->volt.d;
    count++;
  }
  else if (count >= (enc_handler->align_time >> 1) && count <= enc_handler->align_time)
  {
    volt_handler->Vqd.d = enc_handler->volt.d;
    volt_handler->Vqd.q = 0;
    count++;
  }
  else
  {
#if defined MAGNET_ENCODER_WO_ABZ
    enc_handler->offset = -enc_handler->count;
    //enc_handler->pre_count = 0;
#elif defined AB
    tmr_counter_value_set(enc_handler->TMRx, 0);
    enc_handler->offset = 0;
#endif
    volt_handler->Vqd.d = 0;
    count = 0;
    enc_handler->align = SUCCEED;
    ctrl_mode = ctrl_mode_cmd;
  }

  /* trigonometric functions transformation */
  trig_components = trig_functions(0);

  /* Inverse Park transformation */
  foc_inver_park_trans(volt_handler, &trig_components);
}

/**
  * @brief  read rotor angle of encoder
  * @param  enc_handler: encoder related variables
  * @param  rotor_angle_handler: rotor angle related variables
  * @retval rotor angle
  */
int16_t enc_rotor_angle_get(encoder_type *enc_handler, rotor_angle_type *rotor_angle_handler)
{
  int32_t enc_cnt_temp;

#if !defined MAGNET_ENCODER_WO_ABZ
  enc_handler->count = tmr_counter_value_get(enc_handler->TMRx);
#endif

  enc_cnt_temp = enc_handler->count + enc_handler->offset;

  if (enc_cnt_temp < 0)
  {
    enc_cnt_temp = enc_cnt_temp + enc_handler->cpr_nbr;
  }
  else if (enc_cnt_temp >= enc_handler->cpr_nbr)
  {
    enc_cnt_temp = enc_cnt_temp - enc_handler->cpr_nbr;
  }

  enc_handler->val = enc_cnt_temp;

  rotor_angle_handler->elec_angle_val = ((enc_handler->val * enc_handler->mech_to_elect_angle) >> enc_handler->mech_to_elect_angle_shift) & Q15_MAX;

  return (rotor_angle_handler->elec_angle_val);
}

/**
  * @brief  read rotor speed of encoder using M mehtod
  * @param  enc_handler: encoder related variables
  * @param  spd_handler: rotor speed related variables
  * @retval rotor speed
  */
int32_t enc_rotor_speed_get(encoder_type *enc_handler, speed_type *spd_handler)
{
  int16_t angle_delta_temp;

#if !defined MAGNET_ENCODER_WO_ABZ
  enc_handler->count = tmr_counter_value_get(enc_handler->TMRx);
#endif

  angle_delta_temp = enc_handler->count - enc_handler->pre_count;
  enc_handler->pre_count = enc_handler->count;

  if (angle_delta_temp < -enc_handler->half_cpr_nbr)
  {
    angle_delta_temp = angle_delta_temp + enc_handler->cpr_nbr;
  }
  else if (angle_delta_temp > enc_handler->half_cpr_nbr)
  {
    angle_delta_temp = angle_delta_temp - enc_handler->cpr_nbr;
  }

  enc_handler->delta = angle_delta_temp;

  if (enc_handler->delta > 0)
  {
    spd_handler->dir = CW;
  }
  else if (enc_handler->delta < 0)
  {
    spd_handler->dir = CCW;
  }

  return ( (int32_t)(enc_handler->delta * enc_handler->cpr_to_rpm));
}

/**
  * @brief  read rotor speed of encoder using M/T mehtod
  * @param  enc_handler: encoder related variables
  * @param  spd_handler: rotor speed related variables
  * @retval rotor speed
  */
int32_t enc_speed_get_MTmethod(encoder_type *enc_handler, speed_type *spd_handler)
{
  int16_t angle_delta_temp;
  int32_t total_pulse_clks;
  uint16_t encoder_pulse_interval;
  static uint16_t old_encoder_pulse_interval;

  enc_handler->count = tmr_counter_value_get(enc_handler->TMRx);

  angle_delta_temp = enc_handler->count - enc_handler->pre_count;
  enc_handler->pre_count = enc_handler->count;

  if (angle_delta_temp < -enc_handler->half_cpr_nbr)
  {
    angle_delta_temp = angle_delta_temp + enc_handler->cpr_nbr;
  }
  else if (angle_delta_temp > enc_handler->half_cpr_nbr)
  {
    angle_delta_temp = angle_delta_temp - enc_handler->cpr_nbr;
  }

  enc_handler->delta = angle_delta_temp;

  if (enc_handler->delta > 0)
  {
    spd_handler->dir = CW;
  }
  else if (enc_handler->delta < 0)
  {
    spd_handler->dir = CCW;
  }

  if (enc_handler->delta == 0)
  {
    enc_handler->spd_period_count++;
  }
  else
  {
    encoder_pulse_interval = tmr_counter_value_get(enc_handler->TMRx_Cap);
    total_pulse_clks = enc_handler->const_timer_clks + (enc_handler->const_timer_clks * enc_handler->spd_period_count) + old_encoder_pulse_interval - encoder_pulse_interval;
    old_encoder_pulse_interval = encoder_pulse_interval;
    enc_handler->spd_period_count = 0;
  }

  if ((enc_handler->delta != 0) && (total_pulse_clks > 10))
  {
    enc_handler->spd_period_count = 0;
    return ( ((int32_t)enc_handler->est_speed_const * (int32_t)enc_handler->delta) / (int32_t)total_pulse_clks );
  }
  else
  {
    return (spd_handler->val_temp);
  }
}

/**
  * @brief  check encoder error
  * @param  enc_handler: encoder related variables
  * @param  cmd_ramp_handler: speed command related variables
  * @param  curr_handler: current related variables
  * @retval error code (MC_NO_ERROR or MC_ENCODER_ERROR)
  */
err_code_type enc_error_check(encoder_type *enc_handler, ramp_cmd_type *cmd_ramp_handler, current_type *curr_handler)
{
  static uint16_t stop_counter;

  if (ctrl_mode == POSITION_CTRL || ctrl_mode == SPEED_CTRL)
  {
    if (enc_handler->delta == 0 && cmd_ramp_handler->cmd_final != 0 && (abs(curr_handler->Iqdref.q) >= (curr_handler->nominal >> 2)))
    {
      stop_counter++;
    }
    else
    {
      stop_counter = 0;
    }
  }
  else if (ctrl_mode == TORQUE_CTRL)
  {
    if ( (abs(curr_handler->Iqdref.q) >= (curr_handler->nominal >> 2)) && enc_handler->delta == 0)
    {
      stop_counter++;
    }
    else
    {
      stop_counter = 0;
    }
  }

  if (stop_counter > enc_handler->stall_time)
  {
    stop_counter = 0;
    return ( MC_ENCODER_ERROR );
  }
  else
  {
    return ( MC_NO_ERROR );
  }
}

/**
  * @brief  magnetic encoder angle calibration process
  * @param  none
  * @retval none
  */
#ifdef ANGLE_CALIBRATION
void angle_calibrate_process(void)
{
  if (error_code != MC_NO_ERROR)
  {
    encoder.calibrate_state = PROCESS_6_ERROR;
  }

  switch(encoder.calibrate_state)
  {
  case PROCESS_1_ALIGN:
    if (encoder.align != SUCCEED)
    {
      encoder.align = PROCESSING;
    }
    else
    {
      openloop.theta = 0;
      openloop.volt.q = 0;
      openloop.volt.d = encoder.calibrate_volt;
      openloop.inc = ENC_OLC_ANGLE_INC;
      encoder.calibrate_count = 0;
      encoder.calibrate_state = PROCESS_2_CW_CLIBRATION;
    }

    break;

  case PROCESS_2_CW_CLIBRATION:
    if(++encoder.calibrate_count > ENC_OLC_TIME)
    {
      openloop.theta = 0;
      openloop.volt.q = 0;
      openloop.volt.d = encoder.calibrate_volt;
      openloop.inc = 0;
      encoder.calibrate_count = 0;
      encoder.calibrate_state = PROCESS_3_LOCK;
    }

    break;

  case PROCESS_3_LOCK:
    if(++encoder.calibrate_count > 1000)
    {
      openloop.theta = 0;
      openloop.volt.q = 0;
      openloop.volt.d = encoder.calibrate_volt;
      openloop.inc = -ENC_OLC_ANGLE_INC;
      encoder.calibrate_count = 0;
      encoder.calibrate_state = PROCESS_4_CCW_CALIBRATION;
    }

    break;

  case PROCESS_4_CCW_CALIBRATION:
    if(++encoder.calibrate_count > ENC_OLC_TIME)
    {
      encoder.calibrate_state = PROCESS_5_FINISH;
      encoder.calibrate = SUCCEED;
      encoder.calibrate_flag = RESET;
      write_flash_flag = SET;
      esc_state = ESC_STATE_FREE_RUN;
    }

    break;

  case PROCESS_6_ERROR:
    encoder.calibrate_count = 0;
    encoder.calibrate = FAILED;
    encoder.calibrate_flag = RESET;
    write_flash_flag = RESET;
    esc_state = ESC_STATE_ERROR;

  default:
    break;
  }
}

/**
  * @brief  record magnetic encoder angle error
  * @param  none
  * @retval none
  */
void record_angle_deviation(void)
{
  trig_components_type trig_components;
  int16_t angle_diff, index, resi;

  foc_open_loop_ctrl(&volt_cmd, &openloop);
  elec_angle_val = openloop.theta;

  angle_diff = openloop.theta - rotor_angle_encoder.elec_angle_val;

  if(angle_diff > 16384)
  {
    angle_diff = angle_diff - 32767;
  }
  else if(angle_diff < -16384)
  {
    angle_diff = angle_diff + 32767;
  }

  index = encoder.count / encoder_table_div;
  resi = encoder.count - index * encoder_table_div;

  if (resi <= 2)
  {
    if(encoder.calibrate_state == PROCESS_2_CW_CLIBRATION)
    {
      if (encoder_error_CW[index] != 0x7FFF)
      {
        encoder_error_CW[index] += angle_diff;
        encoder_error_CW[index] >>= 1;
      }
      else
      {
        encoder_error_CW[index] = angle_diff;
      }
    }
    else if(encoder.calibrate_state == PROCESS_4_CCW_CALIBRATION)
    {
      if (encoder_error_CCW[index] != 0x7FFF)
      {
        encoder_error_CCW[index] += angle_diff;
        encoder_error_CCW[index] >>= 1;
      }
      else
      {
        encoder_error_CCW[index] = angle_diff;
      }

      encoder_error_aver[index] = (encoder_error_CW[index] + encoder_error_CCW[index]) >> 1;
      encoder_error[index] = moving_average_shift(encoder_comp_ma_fliter, encoder_error_aver[index]);
    }
  }

  /* trigonometric functions transformation */
  trig_components = trig_functions(elec_angle_val);
  /* Clarke transformation */
  foc_clarke_trans(&current.Iabc, &current.Ialphabeta);
  /* Park transformation */
  foc_park_trans(&current, &trig_components);
  /* foc circle limitation */
  foc_circle_limitation(&volt_cmd);
  /* Inverse Park transformation */
  foc_inver_park_trans(&volt_cmd, &trig_components);
}
#endif
