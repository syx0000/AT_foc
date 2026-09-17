/**
  **************************************************************************
  * @file     mc_foc_sensorless.c
  * @brief    foc sensorless related functions.
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

/** @defgroup mc_foc_sensorless
  * @brief foc sensorless related functions.
  * @{
  */

/**
  * @brief  clear sensorless related variables
  * @param  state_obs_handler: sensorless related variables
  * @retval none
  */
void observer_pll_clear(state_observer_type *state_obs_handler)
{
  state_obs_handler->wIalpha_est = ( int32_t )0;
  state_obs_handler->wIbeta_est = ( int32_t )0;
  state_obs_handler->hIalpha_est = ( int16_t )0;
  state_obs_handler->hIbeta_est = ( int16_t )0;
  state_obs_handler->wBemf_alpha_est = ( int32_t )0;
  state_obs_handler->wBemf_beta_est = ( int32_t )0;
  state_obs_handler->hBemf_alpha_est = ( int16_t )0;
  state_obs_handler->hBemf_beta_est = ( int16_t )0;
  state_obs_handler->elec_angle = ( int16_t )0;
  state_obs_handler->motor_speed.val = ( int32_t )0;
  state_obs_handler->motor_speed.filtered = ( int32_t )0;

  pid_set_integral(&state_obs_handler->pid_pll, ( int32_t )0);
}

/**
  * @brief  read EMF voltage offset value
  * @param  emf_abc_offset_handler: adc value of emf voltage offset
  * @param  motor_volt_handler: motor voltage related variables
  * @retval none
  */
void voltage_offset_init(abc_type* emf_abc_offset_handler, motor_volt_type* motor_volt_handler)
{
  static uint8_t icount = 0;
  static uint16_t offset_a = 0;
  static uint16_t offset_b = 0;
  static uint16_t offset_c = 0;

  emf_abc_offset_handler->a = 0;
  emf_abc_offset_handler->b = 0;
  emf_abc_offset_handler->c = 0;

  while (icount < 16)
  {
    if (dma_flag_get(ADC_ORDINARY_DMA_FT_STS_FLAG) != RESET)
    {
      dma_flag_clear(ADC_ORDINARY_DMA_FT_STS_FLAG);

      offset_a += *(motor_volt_handler->Va);
      offset_b += *(motor_volt_handler->Vb);
      offset_c += *(motor_volt_handler->Vc);

      icount ++;
    }
  }
  emf_abc_offset_handler->a = offset_a>>4;
  emf_abc_offset_handler->b = offset_b>>4;
  emf_abc_offset_handler->c = offset_c>>4;
}

/**
  * @brief  calculate motor voltage values
  * @param  motor_volt_handler: motor voltage related variables
  * @retval none
  */
void motor_volt_calc(motor_volt_type *motor_volt_handler)
{
  int32_t MulTemp;
  int16_t Vbus_handler;

  Vbus_handler = *(motor_volt_handler->Vbus) << 3;

  MulTemp = (int32_t) motor_volt_handler->Valphabeta->alpha * Vbus_handler;
  motor_volt_handler->Vpu.alpha = MulTemp >> 15;

  MulTemp = (int32_t) motor_volt_handler->Valphabeta->beta * Vbus_handler;
  motor_volt_handler->Vpu.beta = MulTemp >> 15;
}

/**
  * @brief  read motor EMF voltage values for volt sense
  * @param  motor_volt_handler: motor voltage related variables
  * @param  motor_emf_handler: motor EMF voltage related variables
  * @retval none
  */
void motor_volt_sense(motor_volt_type *motor_volt_handler, motor_emf_type *motor_emf_handler)
{
  abc_type *emf_abc_volt_handler = &(motor_emf_handler->emf_abc_voltage);
  abc_type *emf_abc_offset_handler = &(motor_emf_handler->emf_abc_offset);
  alphabeta_type *emf_alphabeta_volt_handler = &(motor_emf_handler->emf_alphabeta_voltage);

  emf_abc_volt_handler->a = (*(motor_volt_handler->Va) - emf_abc_offset_handler->a) << 4;
  emf_abc_volt_handler->b = (*(motor_volt_handler->Vb) - emf_abc_offset_handler->b) << 4;
  emf_abc_volt_handler->c = (*(motor_volt_handler->Vc) - emf_abc_offset_handler->c) << 4;

  foc_clarke_transform(emf_abc_volt_handler, emf_alphabeta_volt_handler);

  emf_alphabeta_volt_handler->alpha = (motor_volt_handler->emf_factor * emf_alphabeta_volt_handler->alpha) >> 15;
  emf_alphabeta_volt_handler->beta = (motor_volt_handler->emf_factor * emf_alphabeta_volt_handler->beta) >> 15;
}

/**
  * @brief  read motor EMF voltage values for wind sense
  * @param  motor_volt_handler: motor voltage related variables
  * @param  motor_emf_handler: motor EMF voltage related variables
  * @retval none
  */
void motor_volt_sense2(motor_volt_type *motor_volt_handler, motor_emf_type *motor_emf_handler)
{
  abc_type *emf_abc_volt_handler = &(motor_emf_handler->emf_abc_voltage);
  //abc_type *emf_abc_offset_handler = &(motor_emf_handler->emf_abc_offset);
  alphabeta_type *emf_alphabeta_volt_handler = &(motor_emf_handler->emf_alphabeta_voltage);

  emf_abc_volt_handler->a = (*(motor_volt_handler->Va)) << 3;
  emf_abc_volt_handler->b = (*(motor_volt_handler->Vb)) << 3;
  emf_abc_volt_handler->c = (*(motor_volt_handler->Vc)) << 3;

  foc_clarke_transform(emf_abc_volt_handler, emf_alphabeta_volt_handler);

  emf_alphabeta_volt_handler->alpha <<= 1;
  emf_alphabeta_volt_handler->beta <<= 1;

  emf_alphabeta_volt_handler->alpha = (motor_volt_handler->emf_factor * emf_alphabeta_volt_handler->alpha) >> 15;
  emf_alphabeta_volt_handler->beta = (motor_volt_handler->emf_factor * emf_alphabeta_volt_handler->beta) >> 15;
}

/**
  * @brief  Back-EMF voltage sense function
  * @param  none
  * @retval none
  */
void volt_sense(void)
{
  if(curr_offset_rdy & foc_rdy) //pwm on
  {
    motor_volt_sense(&motor_voltage, &motor_emf);
    motor_voltage.Vpu = motor_emf.emf_alphabeta_voltage;
    state_observer.elec_angle = rotor_angle_sensorless(&state_observer, &motor_voltage);
    /* low pass filter for speed by observer */
    state_observer.motor_speed.filtered = lowpass_filtering(&obs_speed_LPF, state_observer.motor_speed.val);
  }
  else //pwm off
  {
    state_observer.motor_speed.val = 0;
    state_observer.motor_speed.filtered = 0;
  }
}

/**
  * @brief  wind sense function for headwind and tailwind startup
  * @param  none
  * @retval none
  */
void wind_sense(void)
{
  if(curr_offset_rdy & foc_rdy) //pwm on
  {
    motor_volt_calc(&motor_voltage);
    state_observer.elec_angle = rotor_angle_sensorless(&state_observer, &motor_voltage);
    /* low pass filter for speed by observer */
    state_observer.motor_speed.filtered = lowpass_filtering(&obs_speed_LPF, state_observer.motor_speed.val);
    motor_emf.time_count = 0;
  }
  else//pwm off
  {
    motor_volt_sense2(&motor_voltage, &motor_emf);

    if (motor_emf.time_count > BEMF_DETECT_COUNT)
    {
      motor_emf.max_bemf_val_filt = ma_filter(motor_emf.max_bemf_val, motor_emf.max_bemf_val_filt, 2);
      motor_emf.max_bemf_val = 0;
      motor_emf.time_count = 0;
    }
    motor_emf.time_count++;

    if (motor_emf.emf_abc_voltage.a > motor_emf.max_bemf_val)
    {
      motor_emf.max_bemf_val = motor_emf.emf_abc_voltage.a;
    }
    if (motor_emf.emf_abc_voltage.b > motor_emf.max_bemf_val)
    {
      motor_emf.max_bemf_val = motor_emf.emf_abc_voltage.b;
    }
    if (motor_emf.emf_abc_voltage.c > motor_emf.max_bemf_val)
    {
      motor_emf.max_bemf_val = motor_emf.emf_abc_voltage.c;
    }

    if (motor_emf.max_bemf_val_filt > AVG_MIN_PEAK_BEMF)
    {
      motor_voltage.Vpu = motor_emf.emf_alphabeta_voltage;
      state_observer.elec_angle = rotor_angle_sensorless(&state_observer, &motor_voltage);
      /* low pass filter for speed by observer */
      state_observer.motor_speed.filtered = lowpass_filtering(&obs_speed_LPF, state_observer.motor_speed.val);
    }
    else
    {
      observer_pll_clear(&state_observer);
    }
  }
}

/**
  * @brief  Open loop startup function
  * @param  startup_handler: pointer to the parameters of the structure sensorless_startup_type
  * @retval flag_status: return SET (successful) or RESET (failed)
  */
flag_status startup_openloop(sensorless_startup_type *startup_handler)
{
  static int16_t delay_count;
  static int16_t local_speed, angle_inc = 1;
  flag_status local_flag = RESET;

  if (startup_handler->startup_mode == CONST_CURR_START)
  {
    if (startup_handler->dir == CW)
    {
      startup_handler->startup_command->d = startup_handler->start_current;
    }
    else
    {
      startup_handler->startup_command->d = -startup_handler->start_current;
    }
    startup_handler->startup_command->q = 0;
  }
  else if (startup_handler->startup_mode == CONST_VOLT_START)
  {
    if (startup_handler->dir == CW)
    {
      startup_handler->startup_command->q = startup_handler->start_volt;
    }
    else
    {
      startup_handler->startup_command->q = -startup_handler->start_volt;
    }
    startup_handler->startup_command->d = 0;
  }

  if (delay_count++ > startup_handler->ol_delay_count)
  {
    if (local_speed < startup_handler->max_speed)
    {
      local_speed += startup_handler->elec_spd_to_rpm;
      angle_inc++;
    }
    else
    {
      local_flag = SET;
      local_speed = 0;
      angle_inc = 1;
    }

    delay_count = 0;
  }

  if (startup_handler->dir == CW)
  {
    startup_handler->ol_angle += angle_inc;
  }
  else
  {
    startup_handler->ol_angle -= angle_inc;
  }

  if (startup_handler->ol_angle < 0)
  {
    startup_handler->elec_angle = startup_handler->ol_angle + 0x8000;
  }
  else
  {
    startup_handler->elec_angle = startup_handler->ol_angle;
  }

  if (local_flag != RESET)
  {
    startup_handler->ol_angle = 0;
  }

  return(local_flag);
}

/**
  * @brief  Alignment and open loop startup function
  * @param  startup_handler: pointer to the parameters of the structure sensorless_startup_type
  * @retval flag_status: return SET (successful) or RESET (failed)
  */
flag_status startup_align_openloop(sensorless_startup_type *startup_handler)
{
  static uint32_t align_count;
  static int16_t delay_count;
  static int16_t local_speed, angle_inc = 1;
  flag_status local_flag = RESET;
  int16_t startup_cmd;

  if (startup_handler->startup_mode == CONST_CURR_START)
  {
    startup_cmd = startup_handler->start_current;
  }
  else if (startup_handler->startup_mode == CONST_VOLT_START)
  {
    startup_cmd = startup_handler->start_volt;
  }

  if (align_count < startup_handler->align_count)
  {
    startup_handler->elec_angle = 0;
    startup_handler->startup_command->d = 0;
    startup_handler->startup_command->q = startup_cmd;
    align_count++;
  }
  else if (align_count < (startup_handler->align_count<<1))
  {
    startup_handler->elec_angle = 0;
    startup_handler->startup_command->d = startup_cmd;
    startup_handler->startup_command->q = 0;
    align_count++;
  }
  else
  {
    if (startup_handler->startup_mode == CONST_CURR_START)
    {
      if (startup_handler->dir == CW)
      {
        startup_handler->startup_command->d = startup_cmd;
      }
      else
      {
        startup_handler->startup_command->d = -startup_cmd;
      }
      startup_handler->startup_command->q = 0;
    }
    else if (startup_handler->startup_mode == CONST_VOLT_START)
    {
      if (startup_handler->dir == CW)
      {
        startup_handler->startup_command->q = startup_cmd;
      }
      else
      {
        startup_handler->startup_command->q = -startup_cmd;
      }
      startup_handler->startup_command->d = 0;
    }

    if (delay_count++ > startup_handler->ol_delay_count)
    {
      if (local_speed < startup_handler->max_speed)
      {
        local_speed += startup_handler->elec_spd_to_rpm;
        angle_inc++;
      }
      else
      {
        local_flag = SET;
        align_count = 0;
        local_speed = 0;
        angle_inc = 1;
      }
      delay_count = 0;
    }

    if (startup_handler->dir == CW)
    {
      startup_handler->ol_angle += angle_inc;
    }
    else
    {
      startup_handler->ol_angle -= angle_inc;
    }

    if (startup_handler->ol_angle < 0)
    {
      startup_handler->elec_angle = startup_handler->ol_angle + 0x8000;
    }
    else
    {
      startup_handler->elec_angle = startup_handler->ol_angle;
    }

    if (local_flag != RESET)
    {
      startup_handler->ol_angle = 0;
    }
  }

  return(local_flag);
}
