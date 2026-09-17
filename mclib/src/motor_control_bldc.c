/**
  **************************************************************************
  * @file     motor_control_bldc.c
  * @brief    motor control related function for BLDC
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

/** @defgroup motor_control_bldc
  * @brief motor control related function for BLDC.
  * @{
  */

int16_t ramp_cnt = 0;
int32_t wave_wide;
int16_t pos_flag_old, switch_count = 0;
int32_t speed_command_calc = 0;
int32_t speed_cmd_filter;
flag_status drag_brk_flag = RESET;

/**
  * @brief  get firmware id
  * @param  none
  * @retval firmware_id_type fw_id
  */
firmware_id_type get_fw_id(void)
{
  firmware_id_type fw_id;
  fw_id = (firmware_id_type)(intCoeffs32[MC_PROTOCOL_REG_FIRMWARE_ID]);
  return (fw_id);
}
/**
  * @brief  enable clock for remap or gpio exint
  * @param  none
  * @retval none
  */
void remap_exint_clock_enable_config(void)
{
#if defined AT32F403Axx || defined AT32F413xx || defined AT32F415xx
  crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
#else
  crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
#endif
}
/**
  * @brief  clear the control parameters
  * @param  none
  * @retval none
  */
void param_clear(void)
{
  pid_is.integral = 0;
  pid_spd.integral = 0;
#ifdef LOW_SPEED_VOLT_CTRL
  pid_spd_volt.integral = 0;
#endif
  speed_ramp.cmd_final = 0;
  speed_ramp.command = 0;
  current.Ibus.Iref = 0;
  reset_ma_buffer(interval_moving_average_fliter);
  speed_LPF.output_temp = 0;
  volt_cmd = 0;
  hall.pre_state = 0;
  start_state = START_STATE_IDLE;

  /* open loop parameter initial*/
  openloop.olc_count = 0;

  openloop.olc_init_period = (int16_t)(10.0 * PWM_FREQ / openloop.olc_init_spd / POLE_PAIRS);
  openloop.olc_final_period = (int16_t)(10.0 * PWM_FREQ / openloop.olc_final_spd / POLE_PAIRS);

  if(openloop.olc_times != 0)
  {
    openloop.olc_period_dec = (int16_t)((openloop.olc_init_period - openloop.olc_final_period) / openloop.olc_times);
  }
  else
  {
    openloop.olc_period_dec = 0;
  }

  openloop.volt_ref = calcValueByVref(openloop.olc_init_volt);
  openloop.period_ref = openloop.olc_init_period;

  if (ctrl_source == CTRL_SOURCE_SOFTWARE)
  {
    sp_value = 0;
  }
}

/**
  * @brief  initialization of motor control parameters
  * @param  none
  * @retval none
  */
void param_init(void)
{
  intCoeffs32_p = (int32_t*)&intCoeffs32;
  hall.offset = 0xFFFF;
  interval_moving_average_fliter = moving_average(SPEED_FILTER_TIMES);
  lowpass_filter_init(&speed_LPF);
#ifdef CURRENT_LP_FILTER
  lowpass_filter_init(&current_LPF);
#endif
  hall_to_tmr_register_setting();
  ui_wave_param.sample_cycle = (uint8_t)(((DATA_BUFFER_FRAME_SIZE * 10.0 / UI_UART_BAUDRATE) + 0.002) * PWM_FREQ / (DATA_BUFFER_SIZE / 4)) + 1;
  ui_wave_param.sampling_rate = (uint16_t)(PWM_FREQ / ui_wave_param.sample_cycle);
}

/**
  * @brief  PWM control table of TMRx_CM1,TMRx_CM2 register in different Hall state
  * @param  none
  * @retval none
  */
void hall_to_tmr_register_setting(void)
{
  uint16_t table[7] = {0};

  for(uint8_t i = 0; i < 7 ; i++)
  {
    table[i] = hall_learn_state_table[i % 6];
  }

  for(int16_t i = 1; i < 6; i++)
  {
    next_hall_state[0][table[i]] = table[i + 1];
    next_hall_state[1][table[i]] = table[i - 1];
  }

  next_hall_state[0][table[0]] = table[1];
  next_hall_state[1][table[0]] = table[5];

  tmr_pwm_channel_mode[table[pwm_pattern[hall_learn.dir][1]]][0] = BH_CL_PWM_MODE_CM1;
  tmr_pwm_channel_mode[table[pwm_pattern[hall_learn.dir][1]]][1] = BH_CL_PWM_MODE_CM2;
  tmr_pwm_output_mode[table[pwm_pattern[hall_learn.dir][1]]] = BH_CL_PWM_OUT_CCTRL;

  tmr_pwm_channel_mode[table[pwm_pattern[hall_learn.dir][2]]][0] = BH_AL_PWM_MODE_CM1;
  tmr_pwm_channel_mode[table[pwm_pattern[hall_learn.dir][2]]][1] = BH_AL_PWM_MODE_CM2;
  tmr_pwm_output_mode[table[pwm_pattern[hall_learn.dir][2]]] = BH_AL_PWM_OUT_CCTRL;

  tmr_pwm_channel_mode[table[pwm_pattern[hall_learn.dir][3]]][0] = CH_AL_PWM_MODE_CM1;
  tmr_pwm_channel_mode[table[pwm_pattern[hall_learn.dir][3]]][1] = CH_AL_PWM_MODE_CM2;
  tmr_pwm_output_mode[table[pwm_pattern[hall_learn.dir][3]]] = CH_AL_PWM_OUT_CCTRL;

  tmr_pwm_channel_mode[table[pwm_pattern[hall_learn.dir][4]]][0] = CH_BL_PWM_MODE_CM1;
  tmr_pwm_channel_mode[table[pwm_pattern[hall_learn.dir][4]]][1] = CH_BL_PWM_MODE_CM2;
  tmr_pwm_output_mode[table[pwm_pattern[hall_learn.dir][4]]] = CH_BL_PWM_OUT_CCTRL;

  tmr_pwm_channel_mode[table[pwm_pattern[hall_learn.dir][5]]][0] = AH_BL_PWM_MODE_CM1;
  tmr_pwm_channel_mode[table[pwm_pattern[hall_learn.dir][5]]][1] = AH_BL_PWM_MODE_CM2;
  tmr_pwm_output_mode[table[pwm_pattern[hall_learn.dir][5]]] = AH_BL_PWM_OUT_CCTRL;

  tmr_pwm_channel_mode[table[pwm_pattern[hall_learn.dir][6]]][0] = AH_CL_PWM_MODE_CM1;
  tmr_pwm_channel_mode[table[pwm_pattern[hall_learn.dir][6]]][1] = AH_CL_PWM_MODE_CM2;
  tmr_pwm_output_mode[table[pwm_pattern[hall_learn.dir][6]]] = AH_CL_PWM_OUT_CCTRL;
}

/**
  * @brief  initial PWM control table of TMRx_CM1,TMRx_CM2 register in hall learning mode
  * @param  none
  * @retval none
  */
void hall_learn_register_setting(void)
{
  for(int16_t i = 0; i < 7; i++)
  {
    tmr_pwm_channel_mode[i][0] = init_tmr_pwm_channel_mode[i][0];
    tmr_pwm_channel_mode[i][1] = init_tmr_pwm_channel_mode[i][1];
    tmr_pwm_output_mode[i] = init_tmr_pwm_output_mode[i];
    next_hall_state[0][i] = init_next_hall_state[0][i];
    next_hall_state[1][i] = init_next_hall_state[1][i];
  }
}

/**
* @brief  disable_mosfet
* @param  tmr_x: select the tmr peripheral.
  *       The timer is used for output PWM.
* @retval none
*/
void disable_mosfet(tmr_type *tmr_x)
{
  /* disable pwm timer output */
  tmr_output_enable(tmr_x, FALSE);

  /* clear pwm timer PWM control mode */
  tmr_x->cm1 &= (~TMR_PWM_MODE_CM1_MASK);
  tmr_x->cm2 &= (~TMR_PWM_MODE_CM2_MASK);

  /* update pwm output mode from shadow buffer of timer cctrl reg. */
  tmr_event_sw_trigger(tmr_x, TMR_HALL_SWTRIG);

  /* set channel dt */
  tmr_x->c1dt = 0;
  tmr_x->c2dt = 0;
  tmr_x->c3dt = 0;
  tmr_x->c4dt = 0;
}

/**
  * @brief  current offset initialize function
  * @param  none
  * @retval none
  */
void I_offset_init(void)
{
  current_offset_tmr_setting(PWM_ADVANCE_TIMER);
  curr_offset_rdy = current_offset_init(&current, ONESHUNT);
  adc_flag_clear(ADC_INSTANT_CONVERTER, ADC_PCCE_FLAG);
  /* Enable current sensing interrupt */
  adc_interrupt_enable(ADC_INSTANT_CONVERTER, ADC_PCCE_INT, TRUE);
}

/**
  * @brief  configure current offset timer
  * @param  tmr_x: select the tmr peripheral.
  * @retval none
  */
void current_offset_tmr_setting(tmr_type *tmr_x)
{
  gpio_init_type gpio_init_struct = {0};

  /* pwm timer output disable */
  tmr_output_enable(PWM_ADVANCE_TIMER, FALSE);
  /* disable pwm timer */
  tmr_counter_enable(PWM_ADVANCE_TIMER, FALSE);

  /* timer pwm output pin Configuration */
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;

  /* High-side, Phase A,B,C Config */
  gpio_init_struct.gpio_pins = PWM_PHASE_A_HI_GPIO_PIN;
  gpio_init(PWM_PHASE_A_HI_PORT, &gpio_init_struct);
#if !defined AT32F403Axx && !defined AT32F407xx && !defined AT32F413xx && !defined AT32F415xx
  gpio_pin_mux_config(PWM_PHASE_A_HI_PORT, PWM_PHASE_A_HI_PIN_SOURCE, PWM_PHASE_A_HI_IOMUX);
#endif

  gpio_init_struct.gpio_pins = PWM_PHASE_B_HI_GPIO_PIN;
  gpio_init(PWM_PHASE_B_HI_PORT, &gpio_init_struct);
#if !defined AT32F403Axx && !defined AT32F407xx && !defined AT32F413xx && !defined AT32F415xx
  gpio_pin_mux_config(PWM_PHASE_B_HI_PORT, PWM_PHASE_B_HI_PIN_SOURCE, PWM_PHASE_B_HI_IOMUX);
#endif

  gpio_init_struct.gpio_pins = PWM_PHASE_C_HI_GPIO_PIN;
  gpio_init(PWM_PHASE_C_HI_PORT, &gpio_init_struct);
#if !defined AT32F403Axx && !defined AT32F407xx && !defined AT32F413xx && !defined AT32F415xx
  gpio_pin_mux_config(PWM_PHASE_C_HI_PORT, PWM_PHASE_C_HI_PIN_SOURCE, PWM_PHASE_C_HI_IOMUX);
#endif

  /* Low-side, Phase A,B,C Config */
  gpio_init_struct.gpio_pins = PWM_PHASE_A_LOW_GPIO_PIN;
  gpio_init(PWM_PHASE_A_LOW_PORT, &gpio_init_struct);
#if !defined AT32F403Axx && !defined AT32F407xx && !defined AT32F413xx && !defined AT32F415xx
  gpio_pin_mux_config(PWM_PHASE_A_LOW_PORT, PWM_PHASE_A_LOW_PIN_SOURCE, PWM_PHASE_A_LOW_IOMUX);
#endif

  gpio_init_struct.gpio_pins = PWM_PHASE_B_LOW_GPIO_PIN;
  gpio_init(PWM_PHASE_B_LOW_PORT, &gpio_init_struct);
#if !defined AT32F403Axx && !defined AT32F407xx && !defined AT32F413xx && !defined AT32F415xx
  gpio_pin_mux_config(PWM_PHASE_B_LOW_PORT, PWM_PHASE_B_LOW_PIN_SOURCE, PWM_PHASE_B_LOW_IOMUX);
#endif

  gpio_init_struct.gpio_pins = PWM_PHASE_C_LOW_GPIO_PIN;
  gpio_init(PWM_PHASE_C_LOW_PORT, &gpio_init_struct);
#if !defined AT32F403Axx && !defined AT32F407xx && !defined AT32F413xx && !defined AT32F415xx
  gpio_pin_mux_config(PWM_PHASE_C_LOW_PORT, PWM_PHASE_C_LOW_PIN_SOURCE, PWM_PHASE_C_LOW_IOMUX);
#endif
  /* close AH/BH/CH */
  gpio_bits_write(PWM_PHASE_A_HI_PORT, PWM_PHASE_A_HI_GPIO_PIN, FALSE);
  gpio_bits_write(PWM_PHASE_B_HI_PORT, PWM_PHASE_B_HI_GPIO_PIN, FALSE);
  gpio_bits_write(PWM_PHASE_C_HI_PORT, PWM_PHASE_C_HI_GPIO_PIN, FALSE);
  /* open AL/BL/CL */
#if defined GATE_DRIVER_LOW_SIDE_INVERT
  gpio_bits_write(PWM_PHASE_A_LOW_PORT, PWM_PHASE_A_LOW_GPIO_PIN, FALSE);
  gpio_bits_write(PWM_PHASE_B_LOW_PORT, PWM_PHASE_B_LOW_GPIO_PIN, FALSE);
  gpio_bits_write(PWM_PHASE_C_LOW_PORT, PWM_PHASE_C_LOW_GPIO_PIN, FALSE);
#else
  gpio_bits_write(PWM_PHASE_A_LOW_PORT, PWM_PHASE_A_LOW_GPIO_PIN, TRUE);
  gpio_bits_write(PWM_PHASE_B_LOW_PORT, PWM_PHASE_B_LOW_GPIO_PIN, TRUE);
  gpio_bits_write(PWM_PHASE_C_LOW_PORT, PWM_PHASE_C_LOW_GPIO_PIN, TRUE);
#endif
  tmr_channel_value_set(tmr_x, TMR_SELECT_CHANNEL_4, PWM_PERIOD / 4);
  /* pwm timer output enable */
  tmr_output_enable(tmr_x, TRUE);
  /* enable pwm timer */
  tmr_counter_enable(tmr_x, TRUE);
}

/**
  * @brief  current mamual tuning function
  * @param  none
  * @retval none
  */
void I_tune_manual(void)
{
  /* Ibus pulse */
  I_tune_count++;

  if ((I_tune_count % current_tune_total_period) == 0)
  {
    I_tune_count = 0;
  }

  if (I_tune_count < current_tune_step_period)
  {
    current.Ibus.Iref = current_tune_target_current;

    if(current.Ibus.Iref > 0)
    {
      current.volt_sign = 1;
      pid_is.lower_limit_output = 0;
      pid_is.upper_limit_output = INT16_MAX;
    }
    else if(current.Ibus.Iref < 0)
    {
      current.volt_sign = -1;
      pid_is.lower_limit_output = -INT16_MAX;
      pid_is.upper_limit_output = 0;
    }
  }
  else
  {
    current.Ibus.Iref = 0;
    pid_is.integral = 0;
    volt_cmd = 0;
  }
}

/**
  * @brief  Function for setting forward rotation parameters
  * @param  none
  * @retval none
  */
void motor_cw_init(void)
{
  readCurrentPidFromFlash(&pid_is);
  readSpeedVoltPidFromFlash(&pid_spd_volt);

  pid_is.kp_gain = adjustValueByVdc(pid_is.kp_gain, vdc_ratio);
  pid_is.ki_gain = adjustValueByVdc(pid_is.ki_gain, vdc_ratio);

  pid_spd_volt.kp_gain = adjustValueByVdc(pid_spd_volt.kp_gain, vdc_ratio);
  pid_spd_volt.ki_gain = adjustValueByVdc(pid_spd_volt.ki_gain, vdc_ratio);

  hall_cw_ctrl_para(&pid_spd_volt, INT16_MAX);
  hall_cw_ctrl_para(&pid_spd, MAX_CURRENT_PU);
  hall_cw_ctrl_para(&pid_is, INT16_MAX);
  rotor_speed.dir = CW;
  current.volt_sign = 1;
  current.volt_sign_coming = 1;
}
/**
  * @brief  Function for setting reverse rotation parameters
  * @param  none
  * @retval none
  */
void motor_ccw_init(void)
{
  readCurrentPidFromFlash(&pid_is);
  readSpeedVoltPidFromFlash(&pid_spd_volt);

  pid_is.kp_gain = adjustValueByVdc(pid_is.kp_gain, vdc_ratio);
  pid_is.ki_gain = adjustValueByVdc(pid_is.ki_gain, vdc_ratio);

  pid_spd_volt.kp_gain = adjustValueByVdc(pid_spd_volt.kp_gain, vdc_ratio);
  pid_spd_volt.ki_gain = adjustValueByVdc(pid_spd_volt.ki_gain, vdc_ratio);

  hall_ccw_ctrl_para(&pid_spd_volt, -INT16_MAX);
  hall_ccw_ctrl_para(&pid_spd, MIN_CURRENT_PU);
  hall_ccw_ctrl_para(&pid_is, -INT16_MAX);
  rotor_speed.dir = CCW;
  current.volt_sign = -1;
  current.volt_sign_coming = -1;
}
/**
 * @brief  Calculates voltage compensation ratio for parameter scaling
 * @param  vref_ratio ADC reference voltage ratio (Q14 fixed-point format)
 *         @note 8192 in Q14 = 1.0 (full scale reference)
 *         @note Typical range: 4096-12288 (0.5x-1.5x nominal voltage)
 *
 * @retval uint16_t Voltage compensation gain ratio (normalized to ideal voltage)
 */
uint16_t calcVdcRatio(int16_t vref_ratio)
{
  int16_t ideal_vbus_value;
  uint16_t volt_control_gain_ratio, dc_volt_adc_value;

  ideal_vbus_value = *(intCoeffs32_p + MC_PROTOCOL_REG_I_TUNE_VDC_RATE);
  dc_volt_adc_value = (uint32_t)adc_in_tab[ADC_BUS_VOLT_IDX] * vref_ratio >> 14;

  if (dc_volt_adc_value < GAIN_RATIO_MIN_VOLT)
  {
    dc_volt_adc_value = GAIN_RATIO_MIN_VOLT;
  }

  volt_control_gain_ratio = (uint16_t)(((float)ideal_vbus_value * VBUS_GAIN_RATIO) / dc_volt_adc_value);

  return(volt_control_gain_ratio);
}
/**
 * @brief  Adjusts control parameters based on DC bus voltage ratio
 * @param  value       Original parameter value (tuned at nominal voltage)
 * @param  Vdc_ratio   Voltage compensation ratio from calcVdcRatio()
 * @retval int16_t     Voltage-compensated parameter value
 */
int16_t adjustValueByVdc(int16_t value, int16_t Vdc_ratio)
{
  int16_t value_ajusted;

  value_ajusted = (int16_t)(((uint32_t)value * Vdc_ratio) >> VBUS_GAIN_LOG);

  return(value_ajusted);
}

/**
  * @brief  hall control parameters cofiguration(dir:CW)
  * @param  none
  * @retval none
  */
void hall_cw_ctrl_para(pid_ctrl_type *pid_handler, int16_t max_value)
{
  pid_handler->upper_limit_integral = max_value << pid_handler->ki_shift;
  pid_handler->lower_limit_integral = 0;

  pid_handler->upper_limit_output = max_value;
  pid_handler->lower_limit_output = 0;
}

/**
  * @brief  hall control parameters cofiguration(dir:CCW)
  * @param  none
  * @retval none
  */
void hall_ccw_ctrl_para(pid_ctrl_type *pid_handler, int16_t min_value)
{
  pid_handler->upper_limit_integral = 0;
  pid_handler->lower_limit_integral = min_value << pid_handler->ki_shift;

  pid_handler->upper_limit_output = 0;
  pid_handler->lower_limit_output = min_value;
}

/**
  * @brief  update pwm duty in overflow
  * @param  none
  * @retval none
  */
void pwm_duty_update(void)
{
  PWM_ADVANCE_TIMER->c1dt = pwm_comp_value;
  PWM_ADVANCE_TIMER->c2dt = pwm_comp_value;
  PWM_ADVANCE_TIMER->c3dt = pwm_comp_value;
}

/**
  * @brief  Set ADC sample point
  * @param  adc_sample : ADC sampling related structure variables
  * @retval none
  */
void set_adc_sample_point(adc_sample_type *adc_sample)
{
#if defined HALL_SENSORS
  /* current sample point */
  adc_sample->ADC_TMRx->c4dt = adc_sample->current_sampling_point;
#else
  adc_sample->adc_sample_page ^= 0x01;
  adc_sample->adc_sample_point[adc_sample->adc_sample_page][0] = adc_sample->current_sampling_point;
  adc_sample->adc_sample_point[adc_sample->adc_sample_page][1] = adc_sample->emf_sampling_point;

  blank_trigger.sample_point[1] = pwm_comp_value;
#endif

#if defined EMF_CONTINOUS_SAMPLE
  /* current sample point */
  adc_sample->ADC_TMRx->c4dt = adc_sample->current_sampling_point;
  /* emf sample point */
  READ_EMF_TIMER->c3dt = pwm_comp_value + GATE_DELAY_COUNT + EMF_SIG_FALLING_COUNT;
  READ_EMF_TIMER->c4dt = adc_sample->current_sampling_point + GATE_DELAY_COUNT;

  lowspd_sample_end = (int16_t)EMF_LOW_SPD_CONT_SAMPLE_END;
  highspd_sample_end = (int16_t)(pwm_comp_value - EMF_SAMPLE_INTERVAL - SENSE_GPIN_DELAY);
#endif
}

/**
  * @brief  Calculate motor speed function
  * @param  rotor_speed : Speed-related structure variables
  * @retval none
  */
void calc_motor_speed(speed_type *rotor_speed)
{
  rotor_speed->speed_count++;

  if(rotor_speed->speed_count > rotor_speed->val_temp)
  {
    if(rotor_speed->speed_count < MAX_SPD_CNT)
    {
      spd_total_cval = rotor_speed->speed_count * PWM_PERIOD;
      calc_spd_rdy = SET;
    }
    else
    {
      rotor_speed->speed_count = MAX_SPD_CNT;
      rotor_speed->val_temp = MAX_SPD_CNT;
      rotor_speed->filtered = 0;
      i32_speed_filterd = 0;
      speed_LPF.output_temp = 0;
      reset_ma_buffer(interval_moving_average_fliter);
    }
  }
}


/**
  * @brief  Open-loop control command ramp
  * @param  openloop_handler : Open-loop related structure variables
  * @retval none
  */
void open_loop_cmd_ramp(olc_type *openloop_handler)
{
  int16_t period_err;

  if(ramp_cnt % 5 == 0)
  {
    period_err = openloop_handler->period_ref - openloop_handler->olc_final_period;

    if (period_err > 0)
    {
      if (period_err < openloop_handler->olc_period_dec)
      {
        openloop_handler->period_ref = openloop_handler->olc_final_period;
      }
      else
      {
        openloop_handler->period_ref -= openloop_handler->olc_period_dec;
      }

      openloop_handler->volt_ref += openloop_handler->olc_volt_inc;
    }

    ramp_cnt = 0;
  }

  ramp_cnt++;
}

/**
  * @brief  enable pwm timer counter
  * @param  none
  * @retval none
  */
void enable_pwm_timer(void)
{
  tmr_counter_enable(PWM_ADVANCE_TIMER, TRUE);
}

/**
  * @brief  disable pwm timer counter
  * @param  none
  * @retval none
  */
void disable_pwm_timer(void)
{
  tmr_counter_enable(PWM_ADVANCE_TIMER, FALSE);
}
/**
  * @brief  Motor parameter auto-identification process handler
  * @details This function manages the complete motor parameter identification sequence:
  *          - Resistance (Rs) and inductance (Ls) measurement
  *          - State machine handling (processing/finished/failed)
  *          - Post-identification system reinitialization
  * @param  none
  * @retval none
  */
void motor_parameter_id_process(void)
{
  if (motor_param_ident.state_flag == PROCESSING)
  {
    param_id_process(&motor_param_ident);
  }
  else if (motor_param_ident.state_flag == FINISHED)
  {
    if (motor_param_ident.Ls.f <= 0 || motor_param_ident.Rs.f <= 0)
    {
      motor_param_ident.state_flag = FAILED;
      error_code |= error_code_mask & MC_PARAM_IDENT_ERROR;
      motor_param_ident.Rs.f = motor_param_ident.Rs_Old.f;
      motor_param_ident.Ls.f = motor_param_ident.Ls_Old.f;
    }
    else
    {
      motor_param_ident.state_flag = SUCCEED;
      motor_param_ident.Rs_Old.f = motor_param_ident.Rs.f;
      motor_param_ident.Ls_Old.f = motor_param_ident.Ls.f;
    }

    tmr_output_enable(PWM_ADVANCE_TIMER, FALSE);
    disable_pwm_timer();
    tmr_pwm_init();
    adc_preempt_config();
    PWM_ADVANCE_TIMER->cval = 0;
    ADC_TIMER->cval = 0;
    enable_pwm_timer();
    esc_state = ESC_STATE_FREE_RUN;
  }
  else if (motor_param_ident.state_flag == FAILED)
  {
    tmr_output_enable(PWM_ADVANCE_TIMER, FALSE);
    disable_pwm_timer();
    tmr_pwm_init();
    adc_preempt_config();
    PWM_ADVANCE_TIMER->cval = 0;
    ADC_TIMER->cval = 0;
    enable_pwm_timer();
    error_code |= error_code_mask & MC_PARAM_IDENT_ERROR;
  }
}

/**
  * @brief  Current PI controller parameter auto-tuning
  * @param  i_tune_handler: i_auto_tune_type pointer
  * @retval none
  */
void current_auto_tuning(i_auto_tune_type *i_tune_handler)
{
  uint16_t mul_temp;
  float cal_temp;

  i_tune_handler->Vdc_rated = i_tune_handler->Vdc_rated_adc / 4095.0f * i_tune_handler->voltage_base;

  cal_temp = 2 * 2 * PI * 0.35f / (3.0f * (*i_tune_handler->Ls) / (*i_tune_handler->Rs));

  if (cal_temp > i_tune_handler->bandwidth)
  {
    i_tune_handler->bandwidth = (uint16_t)cal_temp;
  }

  if (i_tune_handler->bandwidth > i_tune_handler->BW_limit)
  {
    i_tune_handler->bandwidth = i_tune_handler->BW_limit;
  }

  i_tune_handler->kp_coff = (float)i_tune_handler->bandwidth * (*i_tune_handler->Ls) * i_tune_handler->current_base / i_tune_handler->Vdc_rated;

  mul_temp = (uint16_t)(13 - round(log2(i_tune_handler->kp_coff)));

  if (mul_temp < 15)
  {
    i_tune_handler->kp_shift = mul_temp;
  }
  else
  {
    i_tune_handler->kp_shift = 15;
  }

  i_tune_handler->kp = (int16_t)(i_tune_handler->kp_coff * (1 << i_tune_handler->kp_shift));

  i_tune_handler->ki_coff = (float)i_tune_handler->bandwidth * (*i_tune_handler->Rs) * i_tune_handler->current_base / i_tune_handler->Vdc_rated / i_tune_handler->sample_freq;

  mul_temp = (uint16_t)(13 - round(log2(i_tune_handler->ki_coff)));

  if (mul_temp < 15)
  {
    i_tune_handler->ki_shift = mul_temp;
  }
  else
  {
    i_tune_handler->ki_shift = 15;
  }

  i_tune_handler->ki = (int16_t)(i_tune_handler->ki_coff * (1 << i_tune_handler->ki_shift));
}

/**
  * @brief  Controls LED blinking behavior for normal motor operation state
  * @param  led_blink_count : Pointer to the LED blink counter variable.
  *         The counter is incremented each call and reset when reaching LED_BLINK_PERIOD.
  * @param  led_gpio_port : to select the led gpio peripheral.
  * @param  led_gpio_pin: led gpio pin number
  * @retval none
  */
void normal_state_led_blink(uint16_t *led_blink_count, gpio_type *led_gpio_port, uint16_t led_gpio_pin)
{
  if (esc_state_old != ESC_STATE_ERROR)
  {
    if ((esc_state_old == ESC_STATE_IDLE) || (esc_state_old == ESC_STATE_SAFETY_READY))
    {
      led_on(led_gpio_port, led_gpio_pin);
    }
    else
    {
      if (*led_blink_count >= LED_BLINK_PERIOD)
      {
        *led_blink_count = 0;
        led_toggle(led_gpio_port, led_gpio_pin);
      }
      else
      {
        (*led_blink_count)++;
      }
    }
  }
  else
  {
    led_off(led_gpio_port, led_gpio_pin);
    led_blink_count = 0;
  }
}

/**
 * @brief  Adjusts control parameters based on Vref voltage ratio
 * @param  input_value     Original parameter value (tuned at nominal voltage)
 * @retval int16_t     Voltage-compensated parameter value
 */
int16_t calcValueByVref(int16_t input_value)
{
  int16_t output_value;
  output_value = (int16_t)((input_value * adc_sample.vref_cal_ratio) >> 14);
  return(output_value);
}

/**
 * @brief  Converts external circuit commands (from ADC potentiometer) into digital commands
 *         for either torque or speed control modes.
 * @param  none
 * @retval none
 */

void external_input_handler(void)
{
  int16_t sp_value = 0;
  flag_status mode_switch1 = RESET;

  sp_value = adc_in_tab[ADC_POTENTIO_IDX] - SP_OFFSET;
#if !defined (M412_LV_V1_0)
  mode_switch1 = gpio_input_data_bit_read(MODE1_BUTTON_PORT, MODE1_BUTTON_PIN);
#endif
  if (ctrl_mode == TORQUE_CTRL)
  {
    if (sp_value >= SP_RUN_POINT)
    {
      if (mode_switch1 == SET)    /* CCW*/
      {
        current.Ibus.Iref = -(sp_value * SP_TO_I_CMD) >> 15;
      }
      else                        /* CW*/
      {
        current.Ibus.Iref = (sp_value * SP_TO_I_CMD) >> 15;
      }
#if defined (M412_LV_V1_0)
      start_stop_btn_flag = SET;
#endif
    }
    else
    {
      current.Ibus.Iref = 0;
#if defined SENSORLESS || defined (M412_LV_V1_0)
      start_stop_btn_flag = RESET;
#endif
    }
  }
  else if (ctrl_mode == SPEED_CTRL)
  {
    if (mode_switch1 == SET)  /* CCW*/
    {
      if (sp_value >= SP_RUN_POINT)
      {
        speed_ramp.cmd_final = -MIN_SPEED_RPM - (((sp_value << 1) * SP_TO_SPD_CMD) >> 15);
#if defined (M412_LV_V1_0)
        start_stop_btn_flag = SET;
#endif
      }
      else if(sp_value < SP_RUN_POINT && sp_value > SP_STOP_POINT)
      {
        speed_ramp.cmd_final = -MIN_SPEED_RPM;
      }
      else
      {
        speed_ramp.cmd_final = 0;
#if defined SENSORLESS || defined (M412_LV_V1_0)
        start_stop_btn_flag = RESET;
#endif
      }
    }
    else                     /* CW*/
    {
      if (sp_value >= SP_RUN_POINT)
      {
        speed_ramp.cmd_final = MIN_SPEED_RPM + (((sp_value << 1) * SP_TO_SPD_CMD) >> 15);
#if defined (M412_LV_V1_0)
        start_stop_btn_flag = SET;
#endif
      }
      else if(sp_value < SP_RUN_POINT && sp_value > SP_STOP_POINT)
      {
        speed_ramp.cmd_final = MIN_SPEED_RPM;
      }
      else
      {
        speed_ramp.cmd_final = 0;
#if defined SENSORLESS || defined (M412_LV_V1_0)
        start_stop_btn_flag = RESET;
#endif
      }
    }
  }
}

#ifdef PWM_INPUT
/**
  * @brief  Speed curve calculation
  * @param  icvalue : the input channel data value of remote control
  * @retval none
  */
void spd_curve_calculation(int16_t icvalue)
{
  int16_t diff = 0;

  diff = abs(pre_ic1value - icvalue);

  if(icvalue > (pwm_input.zero_bw_cval + DEADZONE_PW_CVAL))
  {
    /* ---MOTOR CW--- */
    wave_wide = icvalue - pwm_input.zero_bw_cval;
    speed_cmd_filter = moving_average_update(spd_cmd_average_filter, (int16_t)speed_command_calc);
    speed_ramp.cmd_final = speed_cmd_filter * (pwm_input.dir);
  }
  else if(icvalue < (pwm_input.zero_bw_cval - DEADZONE_PW_CVAL))
  {
    /* ---MOTOR CCW--- */
    wave_wide = pwm_input.zero_bw_cval - icvalue;

    speed_cmd_filter = moving_average_update(spd_cmd_average_filter, (int16_t) speed_command_calc);
    speed_ramp.cmd_final = speed_cmd_filter * (pwm_input.dir);
  }
  else
  {
    /* ---MOTOR STOP(deadtime zone)--- */
    speed_cmd_filter = 0;
  }
}

/**
 * @brief  Convert PWM pulse width to speed command
 * @param  pulse_width  Input PWM width (us)
 * @param  base_speed   Basic speed
 * @param  slope        Speed sensitivity (RPM/us)
 * @retval Computed speed command
 * @note   Formula: speed = base + (pulse × slope) >> PWM_GAIN_SHIFT
 */
int32_t getSpeedFromPulseWidth(int32_t pulse_width, int16_t base_speed, int16_t slope)
{
  int32_t spd_command;
  spd_command = base_speed + ((pulse_width * slope) >> PWM_IN_GAIN_LOG);

  return(spd_command);
}
#endif

