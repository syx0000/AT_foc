/**
  **************************************************************************
  * @file     motor_control_foc.c
  * @brief    motor control related funciton for Field Oriented Control(FOC)
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

/** @defgroup motor_control_foc
  * @brief motor control related funciton for Field Oriented Control(FOC)
  * @{
  */
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
  abc_type null_abc = {(int16_t)0, (int16_t)0, (int16_t)0};
  qd_type null_qd = {(int16_t)0, (int16_t)0};
  alphabeta_type null_alphabeta =  {(int16_t)0, (int16_t)0};

  speed_ramp.cmd_final = 0;
  speed_ramp.command = 0;

  current.Iabc = null_abc;
  current.Iabc_shunt = null_abc;
  current.Ialphabeta = null_alphabeta;
  current.Iqd = null_qd;
  current.Iqd_LPF = null_qd;
  current.Iqdref = null_qd;
  current.Tref = 0;

  volt_cmd.Vqd = null_qd;
  volt_cmd.Valphabeta = null_alphabeta;

  openloop.volt = null_qd;
  openloop.inc = 0;
  openloop.theta = 0;

  pid_id.integral = 0;
  pid_iq.integral = 0;
  pid_spd.integral = 0;
  pid_pos.integral = 0;

#ifdef LOW_SPEED_VOLT_CTRL
  pid_spd_volt.integral = 0;
#endif

  pwm_duty.OF.a = pwm_duty.half_duty;
  pwm_duty.OF.b = pwm_duty.half_duty;
  pwm_duty.OF.c = pwm_duty.half_duty;
  pwm_duty.UF.a = pwm_duty.half_duty;
  pwm_duty.UF.b = pwm_duty.half_duty;
  pwm_duty.UF.c = pwm_duty.half_duty;

  pwm_duty.ADVTMRx->c1dt = pwm_duty.half_duty;
  pwm_duty.ADVTMRx->c2dt = pwm_duty.half_duty;
  pwm_duty.ADVTMRx->c3dt = pwm_duty.half_duty;

#ifdef SENSORLESS
#if (!defined WIND_SENSE)
  observer_pll_clear(&state_observer);
#endif
  startup.closeloop_rdy = RESET;
  startup.closeloop_rdy_old = RESET;
  obs_speed_LPF.output_temp = 0;
  obs_speed_LPF.residual = 0;
  startup_counter = 0;
  stop_counter = 0;
#endif

#ifdef CURRENT_DECOUPLE_CTRL
  current_decoupling.Vff = null_qd;
  current_decoupling.Iq_LPF.output_temp = 0;
  current_decoupling.Iq_LPF.residual = 0;
  current_decoupling.Id_LPF.output_temp = 0;
  current_decoupling.Id_LPF.residual = 0;
#endif

#ifdef IPM_MTPA_MTPV_TABLE
  mtpa.Iq_LPF.output_temp = 0;
  mtpa.Iq_LPF.residual = 0;
  mtpa.Id_LPF.output_temp = 0;
  mtpa.Id_LPF.residual = 0;
  mtpa.mtpa_table.I_filt_val = null_qd;
#endif

#ifdef FIELD_WEAKENING
  fw_clear(&field_weakening, &volt_cmd);
#endif

#if defined E_BIKE_SCOOTER
  Iq_ref_cmd = 0;
  pid_spd_trq = pid_spd;
  pid_spd_trq.integral = 0;
  pid_spd_trq.upper_limit_integral = 0;
  pid_spd_trq.lower_limit_integral = 0;
  pid_spd_trq.upper_limit_output = 0;
  pid_spd_trq.lower_limit_output = 0;
  current.Idc.val = 0;
  current.Idc.filtered = 0;
  lock_motor_time = 0;
#if defined IPM_MTPA_MTPV_CTRL
  pid_spd.lower_limit_output = (int16_t)BRAKING_TORQUE_PU;
  pid_spd.lower_limit_integral = (int32_t)(BRAKING_TORQUE_PU << pid_spd_trq.ki_shift);
#else
  pid_spd.lower_limit_output = (int16_t)BRAKING_CURRENT_PU;
  pid_spd.lower_limit_integral = (int32_t)(BRAKING_CURRENT_PU << pid_spd_trq.ki_shift);
#endif
#endif
#if defined TORQUE_CTRL_WITH_SPEED_LIMIT
  Iq_ref_cmd = 0;
  pid_spd_trq = pid_spd;
  pid_spd_trq.integral = 0;
  pid_spd_trq.upper_limit_integral = 0;
  pid_spd_trq.lower_limit_integral = 0;
  pid_spd_trq.upper_limit_output = 0;
  pid_spd_trq.lower_limit_output = 0;
#endif
#ifdef BRAKING_RESISTOR
  /* disable brake tmr channel */
  tmr_channel_enable(BRAKE_PWM_TIMER, BRAKE_PWM_TIMER_CH, FALSE);
#endif
#if CURRENT_LP_FILTER
  q_current_LPF.output_temp = 0;
  q_current_LPF.residual = 0;
  d_current_LPF.output_temp = 0;
  d_current_LPF.residual = 0;
#endif
#if AC_CURRENT_LP_FILTER
  ia_current_LPF.output_temp = 0;
  ia_current_LPF.residual = 0;
  ib_current_LPF.output_temp = 0;
  ib_current_LPF.residual = 0;
#endif
#if RDS_AUTO_CALIBRATION
  Rds_Cali.Iq_LPF.output_temp = 0;
  Rds_Cali.Id_LPF.output_temp = 0;
  Rds_Cali.Id_LPF.residual = 0;
  Rds_Cali.Iq_LPF.residual = 0;
#endif
  if (ctrl_source == CTRL_SOURCE_SOFTWARE)
    sp_value = 0;
}

/**
  * @brief  initialization of motor control parameters
  * @param  none
  * @retval none
  */
void param_init(void)
{
  intCoeffs32_p = (int32_t*)&intCoeffs32;
#if defined INCREM_ENCODER || defined MAGNET_ENCODER_W_ABZ || defined MAGNET_ENCODER_WO_ABZ
  encoder.mech_to_elect_angle_shift = (uint8_t)(16-ceil(log2(encoder.pole_pairs)));
  encoder.mech_to_elect_angle = (uint32_t)(ENC_MECH_TO_ELECT_ANGLE * pow(2,encoder.mech_to_elect_angle_shift));
#endif
#ifdef ANGLE_CALIBRATION
  int16_t iii;
  encoder_comp_ma_fliter = moving_average_shift_init(16);
  encoder_table_shift = (int8_t)(ceil(log2(ENC_CPR_NBR)) - log2(INT16_SIZE_ARRARY));
  if (encoder_table_shift < 0)
    encoder_table_shift = 0;
  encoder_table_div = (int16_t)pow(2, encoder_table_shift);
  table_interp_coeff = (int32_t)pow(2, 20-encoder_table_shift);
  for (iii=0; iii<1024; iii++)
  {
    encoder_error_CW[iii] = 0x7FFF;
    encoder_error_CCW[iii] = 0x7FFF;
  }
#endif
#if defined HALL_SENSORS
  foc_hall_table_mapping();
  hall_interval_moving_average = moving_average(6);
  hall_next_state_table = hall_cw_next_state_table;
  hall_theta_table = hall_cw_theta_table;

  error_code |= error_code_mask & hall_at_zero_speed(&hall, &rotor_angle_hall, hall_next_state_table);
  hall.slick_speed = SLICK_SPEED_RPM_SHIFT;
  rotor_angle_hall_old.elec_angle_val = rotor_angle_hall.elec_angle_val;
  rotor_angle_hall.elec_angle_pre_val = rotor_angle_hall.elec_angle_val;
#endif
#if defined SENSORLESS
  lowpass_filter_init(&obs_speed_LPF);
  state_observer.c1 = (int16_t)(0x7FFF*EXP_TAYLOR(-(*state_observer.Rs)/((*state_observer.Ls)*PWM_FREQ)));
  state_observer.c3 = (int32_t)(0x7FFF*((ZS_BASE/(*state_observer.Rs))*(1.0 - EXP_TAYLOR(-(*state_observer.Rs)/((*state_observer.Ls)*PWM_FREQ)))));
#endif
#if defined CURRENT_DECOUPLE_CTRL
  current_decoupling.const_1d = (int32_t)(32767.0f*POLE_PAIRS*2*PI*motor_param_ident.Ls.f*ADC_DIGITAL_SCALE_12BITS/60/ZS_BASE);
  current_decoupling.const_1q = (int32_t)(current_decoupling.const_1d/LD_LQ_RATIO);
  lowpass_filter_init(&current_decoupling.Iq_LPF);
  lowpass_filter_init(&current_decoupling.Id_LPF);
#endif
#if defined IPM_MTPA_MTPV_TABLE
  lowpass_filter_init(&mtpa.Iq_LPF);
  lowpass_filter_init(&mtpa.Id_LPF);
#endif
#if defined IPM_MTPA_MTPV_CTRL
  pid_spd.upper_limit_output = (int16_t)INT16_MAX;
  pid_spd.lower_limit_output = (int16_t)-INT16_MAX;
  pid_spd.upper_limit_integral = (int32_t)(INT16_MAX << intCoeffs32[MC_PROTOCOL_REG_SPEED_KI_DIV]);
  pid_spd.lower_limit_integral = (int32_t)(-(INT16_MAX << intCoeffs32[MC_PROTOCOL_REG_SPEED_KI_DIV]));
#endif
#if defined TORQUE_CTRL_WITH_SPEED_LIMIT || defined E_BIKE_SCOOTER
  pid_spd_trq = pid_spd;
#endif
#if CURRENT_LP_FILTER
  lowpass_filter_init(&q_current_LPF);
  lowpass_filter_init(&d_current_LPF);
#endif
#if AC_CURRENT_LP_FILTER
  lowpass_filter_init(&ia_current_LPF);
  lowpass_filter_init(&ib_current_LPF);
#endif
#if DC_CURRENT_LIMIT | RDS_AUTO_CALIBRATION
  Idc_ma_fliter = moving_average_shift_init(DC_CURR_MA_NBR);
  Iq_ref_cmd_limit = current.max_current;
#if RDS_AUTO_CALIBRATION
  lowpass_filter_init(&Rds_Cali.Iq_LPF);
  lowpass_filter_init(&Rds_Cali.Id_LPF);
#endif
#endif

#if defined TWO_ADC_CONVERTERS && defined THREE_SHUNT
  current_adc_ch.ph_a = CURR_PHASE_A_ADC_CH;
  current_adc_ch.ph_b = CURR_PHASE_B_ADC_CH;
  current_adc_ch.ph_c = CURR_PHASE_C_ADC_CH;
  pwm_duty.threshold = HIGH_PWM_DUTY_COUNT;
  adc_ch_sel_sector = 6;
#endif

  speed_ma_fliter = moving_average_shift_init(8);
  foc_rdy = RESET;
  pwm_switch_off();
  param_clear();
  ui_wave_param.sample_cycle = (uint8_t)(((DATA_BUFFER_FRAME_SIZE*10.0/UI_UART_BAUDRATE)+0.002)*PWM_FREQ/(DATA_BUFFER_SIZE/4)+1);
  ui_wave_param.sampling_rate = (uint16_t)(PWM_FREQ/ui_wave_param.sample_cycle);
}

/**
  * @brief  gate drive capacitor precharge function
  * @param  pwm_duty_handler: three-phase pwm duty cycle and timer used
  * @retval none
  */
void charge_boot_cap(pwm_duty_type *pwm_duty_handler)
{
  tmr_output_enable(pwm_duty_handler->ADVTMRx, TRUE);
  tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_1, 0);
  tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_2, 0);
  tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_3, 0);
#if (defined AT32M412xx || defined AT32M416xx)
  tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_1A, 0);
  tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_2A, 0);
  tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_3A, 0);
#endif
  mc_delay_ms(1);
  tmr_output_enable(pwm_duty_handler->ADVTMRx, FALSE);
}

/**
  * @brief  enable pwm timer channel mode buffer
  * @param  pwm_duty_handler: three-phase pwm duty cycle and timer used
  * @retval none
  */
void enable_pwm_timer_channel_buffer(pwm_duty_type *pwm_duty_handler)
{
  tmr_channel_buffer_enable(pwm_duty_handler->ADVTMRx, TRUE);
}

/**
  * @brief  enable pwm timer counter
  * @param  pwm_duty_handler: three-phase pwm duty cycle and timer used
  * @retval none
  */
void enable_pwm_timer(pwm_duty_type *pwm_duty_handler)
{
#if defined MAGNET_ENCODER_WO_ABZ
  tmr_counter_enable(pwm_duty_handler->SYNC_TMRx, TRUE);
#endif
  tmr_output_enable(pwm_duty_handler->ADVTMRx, TRUE);
  tmr_counter_enable(pwm_duty_handler->ADVTMRx, TRUE);
}

/**
  * @brief  disable pwm timer counter
  * @param  pwm_duty_handler: three-phase pwm duty cycle and timer used
  * @retval none
  */
void disable_pwm_timer(pwm_duty_type *pwm_duty_handler)
{
#if defined MAGNET_ENCODER_WO_ABZ
  tmr_counter_enable(pwm_duty_handler->SYNC_TMRx, FALSE);
#endif
  tmr_counter_enable(pwm_duty_handler->ADVTMRx, FALSE);
}

#if defined AT32F423xx || defined AT32F425xx || defined AT32F402xx || defined AT32F405xx
/**
  * @brief  disable pwm output
  * @param  none
  * @retval none
  */
void pwm_switch_off(void)
{
  tmr_output_enable(pwm_duty.ADVTMRx, FALSE);
  tmr_output_enable(pwm_duty.adc_TMRx, FALSE);
  tmr_channel_enable(pwm_duty.adc_TMRx, pwm_duty.adc_tmr_trig_ch, FALSE);
}

/**
  * @brief  enable pwm output
  * @param  none
  * @retval none
  */
void pwm_switch_on(void)
{
  tmr_output_enable(pwm_duty.ADVTMRx, FALSE);
  tmr_counter_enable(pwm_duty.ADVTMRx, FALSE);
  tmr_channel_enable(pwm_duty.adc_TMRx, pwm_duty.adc_tmr_trig_ch, FALSE);
  tmr_output_enable(pwm_duty.adc_TMRx, FALSE);
  tmr_counter_enable(pwm_duty.adc_TMRx, FALSE);
#ifdef ONE_SHUNT
  dma_channel_enable(TMR_ADC_DMA_CH, FALSE);
  /* reset adc preempt conversion order */
  ADC_CONVERTER->psq_bit.pclen = 0;
  ADC_CONVERTER->psq_bit.pclen = 1;
  dma_channel_enable(TMR_ADC_DMA_CH, TRUE);
  pwm_duty.adc_trig.first_pos = (DOUBLE_PWM_PERIOD - PWM_PERIOD/2);
  pwm_duty.adc_trig.second_pos[0] = (DOUBLE_PWM_PERIOD -PWM_PERIOD/4);
  pwm_duty.adc_trig.second_temp = (DOUBLE_PWM_PERIOD - PWM_PERIOD/4);

  tmr_counter_value_set(pwm_duty.adc_TMRx, PWM_PERIOD);
  tmr_counter_value_set(pwm_duty.ADVTMRx, PWM_PERIOD);

  tmr_channel_value_set(pwm_duty.adc_TMRx, pwm_duty.adc_tmr_trig_ch, pwm_duty.adc_trig.first_pos);
#endif
  tmr_counter_enable(pwm_duty.adc_TMRx, TRUE);
  tmr_counter_enable(pwm_duty.ADVTMRx, TRUE);
  tmr_output_enable(pwm_duty.ADVTMRx, TRUE);
  tmr_channel_enable(pwm_duty.adc_TMRx, pwm_duty.adc_tmr_trig_ch, TRUE);
  tmr_output_enable(pwm_duty.adc_TMRx, TRUE);
}

#else

/**
  * @brief  disable pwm output
  * @param  none
  * @retval none
  */
void pwm_switch_off(void)
{
  tmr_output_enable(pwm_duty.ADVTMRx, FALSE);

#if (defined OP_INP_MODE_SWITCH) && (defined AT32M412xx || defined AT32M416xx)
  tmr_output_enable(SW_OP_INP_MODE_TIMER, FALSE);
  tmr_counter_enable(SW_OP_INP_MODE_TIMER, FALSE);

  int32_t gpio_cfgr_temp;
  int16_t shift_bit;

  dma_channel_enable(TMR_SW_OP_PORTA_DMA_CH, FALSE);
  shift_bit = LOG2(OP1_INP_PIN)*2;
  gpio_cfgr_temp = OP1_INP_PORT->cfgr & ~(0x03 << shift_bit);
  OP1_INP_PORT->cfgr = gpio_cfgr_temp | (GPIO_MODE_ANALOG << shift_bit);

  dma_channel_enable(TMR_SW_OP_PORTB_DMA_CH, FALSE);
  shift_bit = LOG2(OP4_INP_PIN)*2;
  gpio_cfgr_temp = OP4_INP_PORT->cfgr & ~(0x03 << shift_bit);
  OP4_INP_PORT->cfgr = gpio_cfgr_temp | (GPIO_MODE_ANALOG << shift_bit);
#endif
}

/**
  * @brief  enable pwm output
  * @param  none
  * @retval none
  */
void pwm_switch_on(void)
{
#if (defined OP_INP_MODE_SWITCH) && (defined AT32M412xx || defined AT32M416xx)
  int32_t gpio_cfgr_temp;
  int16_t shift_bit;
#endif

#ifdef MOS_RDS_SHUNT
  adc_preempt_conversion_trigger_set(ADC_CONVERTER, TMR_ADC_TRIG_SOURCE, TMR_ADC_TRIG_NO_SIGNAL);
#endif
  tmr_output_enable(pwm_duty.ADVTMRx, FALSE);
  tmr_counter_enable(pwm_duty.ADVTMRx, FALSE);
#if defined MAGNET_ENCODER_WO_ABZ
  tmr_counter_enable(pwm_duty.SYNC_TMRx, FALSE);
#endif
#ifdef ONE_SHUNT
  /* reset adc trigger dma counter */
  dma_channel_enable(TMR_ADC_DMA_CH, FALSE);
  TMR_ADC_DMA_CH->dtcnt = 0;
  TMR_ADC_DMA_CH->dtcnt = 2;
  /* reset adc preempt conversion order */
  ADC_CONVERTER->psq_bit.pclen = 0;
  ADC_CONVERTER->psq_bit.pclen = 1;
  dma_channel_enable(TMR_ADC_DMA_CH, TRUE);
  pwm_duty.adc_trig.first_pos = (PWM_PERIOD/2);
  pwm_duty.adc_trig.second_pos[0] = (PWM_PERIOD/4);
  pwm_duty.adc_trig.second_temp = (PWM_PERIOD/4);
  pwm_duty.adc_trig.second_pos[1] = PWM_PERIOD;
  tmr_counter_value_set(pwm_duty.ADVTMRx, PWM_PERIOD);
  tmr_channel_value_set(pwm_duty.adc_TMRx, pwm_duty.adc_tmr_trig_ch, pwm_duty.adc_trig.first_pos);
#ifdef AT32L021xx
  dma_channel_enable(TMR_BURST_DMA_CH, FALSE);
  TMR_BURST_DMA_CH->dtcnt = 6;
  dma_channel_enable(TMR_BURST_DMA_CH, TRUE);
#endif
#endif
#if (defined OP_INP_MODE_SWITCH) && (defined AT32M412xx || defined AT32M416xx)
  tmr_counter_enable(SW_OP_INP_MODE_TIMER, FALSE);
  tmr_flag_clear(SW_OP_INP_MODE_TIMER, TMR_OVF_FLAG | TMR_C1_FLAG);

  dma_channel_enable(TMR_SW_OP_PORTA_DMA_CH, FALSE);
  TMR_SW_OP_PORTA_DMA_CH->maddr = (uint32_t) gpioa_mode_cfg;	   // set transfer address
  TMR_SW_OP_PORTA_DMA_CH->dtcnt = 2;

  shift_bit = LOG2(OP1_INP_PIN)*2;
  gpio_cfgr_temp = OP1_INP_PORT->cfgr & ~(0x03 << shift_bit);
  gpioa_mode_cfg[0] = gpio_cfgr_temp | (GPIO_MODE_ANALOG << shift_bit);
  gpioa_mode_cfg[1] = gpio_cfgr_temp | (GPIO_MODE_OUTPUT << shift_bit);
  OP1_INP_PORT->cfgr = gpioa_mode_cfg[1];
  dma_channel_enable(TMR_SW_OP_PORTA_DMA_CH, TRUE);

  dma_channel_enable(TMR_SW_OP_PORTB_DMA_CH, FALSE);
  TMR_SW_OP_PORTB_DMA_CH->maddr = (uint32_t) gpiob_mode_cfg;	   // set transfer address
  TMR_SW_OP_PORTB_DMA_CH->dtcnt = 2;

  shift_bit = LOG2(OP4_INP_PIN)*2;
  gpio_cfgr_temp = OP4_INP_PORT->cfgr & ~(0x03 << shift_bit);
  gpiob_mode_cfg[0] = gpio_cfgr_temp | (GPIO_MODE_ANALOG << shift_bit);
  gpiob_mode_cfg[1] = gpio_cfgr_temp | (GPIO_MODE_OUTPUT << shift_bit);
  OP4_INP_PORT->cfgr = gpiob_mode_cfg[1];
  dma_channel_enable(TMR_SW_OP_PORTB_DMA_CH, TRUE);

  dma_channel_enable(TMR_SW_OP_POS_DMA_CH, FALSE);
  TMR_SW_OP_POS_DMA_CH->maddr = (uint32_t) sw_op_inp_mode_pos;	   // set transfer address
  TMR_SW_OP_POS_DMA_CH->dtcnt = 2;

  tmr_channel_value_set(SW_OP_INP_MODE_TIMER, SW_OP_INP_MODE_SELECT_CHANNEL, SW_OP_INP_MODE_POS2);
  dma_channel_enable(TMR_SW_OP_POS_DMA_CH, TRUE);

  tmr_counter_value_set(PWM_ADVANCE_TIMER, 0);
  tmr_counter_value_set(SW_OP_INP_MODE_TIMER, DOUBLE_PWM_PERIOD);
  tmr_counter_enable(SW_OP_INP_MODE_TIMER, TRUE);
#endif
  tmr_counter_enable(pwm_duty.ADVTMRx, TRUE);
  tmr_output_enable(pwm_duty.ADVTMRx, TRUE);
#ifdef MOS_RDS_SHUNT
  adc_preempt_conversion_trigger_set(ADC_CONVERTER, TMR_ADC_TRIG_SOURCE, TMR_ADC_TRIG_SIGNAL);
#endif
#if defined MAGNET_ENCODER_WO_ABZ
  tmr_counter_enable(pwm_duty.SYNC_TMRx, TRUE);
#endif
}

#endif


/**
  * @brief  current offset initialize function
  * @param  curr_handler: three-phase current values and their offset
  * @retval none
  */
flag_status I_offset_init(current_type *curr_handler)
{
  flag_status offset_rdy = RESET;

#if defined TWO_ADC_CONVERTERS

#if defined THREE_SHUNT
  offset_rdy = current_offset_init_2adc(curr_handler, THREESHUNT);
#elif defined TWO_SHUNT
  offset_rdy = current_offset_init_2adc(curr_handler, TWOSHUNT);
#endif

#else

#if defined THREE_SHUNT
  offset_rdy = current_offset_init(curr_handler, THREESHUNT);
#elif defined TWO_SHUNT
  offset_rdy = current_offset_init(curr_handler, TWOSHUNT);
#elif defined ONE_SHUNT
  offset_rdy = current_offset_init(curr_handler, ONESHUNT);
#endif

#endif
  adc_flag_clear(curr_handler->ADCx, ADC_PCCE_FLAG);
  adc_interrupt_enable(curr_handler->ADCx, ADC_PCCE_INT, TRUE); /* Enable current sensing interrupt */

  return (offset_rdy);
}

/**
  * @brief  current mamual tuning function
  * @param  none
  * @retval none
  */
void I_tune_manual(void)
{
  /* Id-axis or Iq-axis pulse */
  I_tune_count++;

  if (I_tune_count % current_tune_total_period == 0)
  {
    I_tune_count = 0;
  }

  if (I_tune_count < current_tune_step_period)
  {
    if (ctrl_mode == ID_MANUAL_TUNE)
    {
      current.Iqdref.d = current_tune_target_current;
    }
    else
    {
      current.Iqdref.q = current_tune_target_current;
    }
  }
  else
  {
    if (ctrl_mode == ID_MANUAL_TUNE)
    {
      current.Iqdref.d = 0;
    }
    else
    {
      current.Iqdref.q = 0;
    }
  }
}

/**
  * @brief  hall control parameters cofiguration(dir:CW)
  * @param  none
  * @retval none
  */
void hall_cw_ctrl_para(void)
{
#ifdef E_BIKE_SCOOTER
  I_auto_tune.Vdc_rated_adc = *(intCoeffs32_p+MC_PROTOCOL_REG_I_TUNE_VDC_RATE);
  adjustIdqPIDGain(&pid_id, &pid_iq, &volt_cmd);
#endif
  hall_next_state_table = hall_cw_next_state_table;
  hall_theta_table = hall_cw_theta_table;
  rotor_speed_hall.dir = CW;
  pid_iq.upper_limit_integral = volt_cmd.Vq_max << pid_iq.ki_shift;
  pid_iq.lower_limit_integral = 0;
  pid_iq.upper_limit_output = volt_cmd.Vq_max;
  pid_iq.lower_limit_output = 0;

#ifdef LOW_SPEED_VOLT_CTRL
  pid_spd_volt.upper_limit_integral = volt_cmd.Vq_max << pid_spd_volt.ki_shift;
  pid_spd_volt.lower_limit_integral = 0;
  pid_spd_volt.upper_limit_output = volt_cmd.Vq_max;
  pid_spd_volt.lower_limit_output = 0;
#endif

}

/**
  * @brief  hall control parameters cofiguration(dir:CCW)
  * @param  none
  * @retval none
  */
void hall_ccw_ctrl_para(void)
{
#ifdef E_BIKE_SCOOTER
  I_auto_tune.Vdc_rated_adc = *(intCoeffs32_p+MC_PROTOCOL_REG_I_TUNE_VDC_RATE);
  adjustIdqPIDGain(&pid_id, &pid_iq, &volt_cmd);
#endif
  hall_next_state_table = hall_ccw_next_state_table;
  hall_theta_table = hall_ccw_theta_table;
  rotor_speed_hall.dir = CCW;
  pid_iq.upper_limit_integral = 0;
  pid_iq.lower_limit_integral = -volt_cmd.Vq_max << pid_iq.ki_shift;
  pid_iq.upper_limit_output = 0;
  pid_iq.lower_limit_output = -volt_cmd.Vq_max;

#ifdef LOW_SPEED_VOLT_CTRL
  pid_spd_volt.upper_limit_integral = 0;
  pid_spd_volt.lower_limit_integral = -volt_cmd.Vq_max << pid_spd_volt.ki_shift;
  pid_spd_volt.upper_limit_output = 0;
  pid_spd_volt.lower_limit_output = -volt_cmd.Vq_max;
#endif
}

/**
  * @brief  control parameters cofiguration in torque control(E_BIKE_SCOOTER Mode)
  * @param  none
  * @retval none
  */
void curr_cmd_handler_ebike(pid_ctrl_type *pid_handler, ramp_cmd_type *cmd_ramp_handler, int16_t Iqref_handler)
{
  if (Iqref_handler >= 0)
  {
    pid_handler->upper_limit_output = Iqref_handler;
    pid_handler->lower_limit_output = -Iqref_handler;
    pid_handler->upper_limit_integral = (Iqref_handler << pid_handler->ki_shift);
    pid_handler->lower_limit_integral = -(Iqref_handler << pid_handler->ki_shift);
    cmd_ramp_handler->cmd_final = MAX_SPEED_RPM;
    cmd_ramp_handler->command = MAX_SPEED_RPM;
  }
}

/**
  * @brief  dc current limitation function(E_BIKE_SCOOTER Mode)
  * @param  none
  * @retval none
  */
#if DC_CURRENT_LIMIT | RDS_AUTO_CALIBRATION
void dc_current_limit(void)
{
  current.Idc.val = dc_current_read(&current, &adc_in_tab[ADC_IBUS_AVE_IDX]);
  current.Idc.filtered = moving_average_shift(Idc_ma_fliter, current.Idc.val);
#if DC_CURRENT_LIMIT
  if (ctrl_mode == TORQUE_CTRL)
  {
    if (Iq_ref_cmd > 0)
    {
      if (current.Idc.filtered >= DC_MAX_CURRENT_PU)
      {
        Iq_ref_cmd_limit -= REDUCE_IQ;

        if (Iq_ref_cmd_limit < 0)
          Iq_ref_cmd_limit = 0;
      }
      else
      {
        Iq_ref_cmd_limit += RECOVER_IQ;

        if (Iq_ref_cmd_limit >= Iq_ref_cmd)
          Iq_ref_cmd_limit = Iq_ref_cmd;
      }
      if (Iq_ref_cmd_limit < Iq_ref_cmd)
      {
        pid_spd_trq.upper_limit_output = Iq_ref_cmd_limit;
        pid_spd_trq.lower_limit_output = -Iq_ref_cmd_limit;
        pid_spd_trq.upper_limit_integral = (Iq_ref_cmd_limit << pid_spd.ki_shift);
        pid_spd_trq.lower_limit_integral = -(Iq_ref_cmd_limit << pid_spd.ki_shift);
      }
      else
      {
        pid_spd_trq.upper_limit_output = Iq_ref_cmd;
        pid_spd_trq.lower_limit_output = -Iq_ref_cmd;
        pid_spd_trq.upper_limit_integral = (Iq_ref_cmd << pid_spd.ki_shift);
        pid_spd_trq.lower_limit_integral = -(Iq_ref_cmd << pid_spd.ki_shift);
      }
    }
  }
  else if (ctrl_mode == SPEED_CTRL)
  {
    if (current.Iqdref.q > 0)
    {
      if (current.Idc.filtered >= DC_MAX_CURRENT_PU)
      {
        Iq_ref_cmd_limit -= REDUCE_IQ;

        if (Iq_ref_cmd_limit < 0)
          Iq_ref_cmd_limit = 0;
      }
      else
      {
        Iq_ref_cmd_limit += RECOVER_IQ;

        if (Iq_ref_cmd_limit >= current.max_current)
          Iq_ref_cmd_limit = current.max_current;
      }
      pid_spd.upper_limit_output = Iq_ref_cmd_limit;
      pid_spd.upper_limit_integral = (Iq_ref_cmd_limit << pid_spd.ki_shift);
    }
  }
#endif
}
#endif
/**
  * @brief  control parameters cofiguration in torque control(TORQUE_CTRL_WITH_SPEED_LIMIT)
  * @param  none
  * @retval none
  */
void curr_cmd_handler_speed_limit(void)
{
  if (Iq_ref_cmd >= 0)
  {
    pid_spd_trq.upper_limit_output = Iq_ref_cmd;
    pid_spd_trq.lower_limit_output = -Iq_ref_cmd;
    pid_spd_trq.upper_limit_integral = (Iq_ref_cmd << pid_spd_trq.ki_shift);
    pid_spd_trq.lower_limit_integral = -(Iq_ref_cmd << pid_spd_trq.ki_shift);
    speed_ramp.cmd_final = MAX_SPEED_RPM;
    speed_ramp.command = MAX_SPEED_RPM;
  }
  else
  {
    pid_spd_trq.upper_limit_output = -Iq_ref_cmd;
    pid_spd_trq.lower_limit_output = Iq_ref_cmd;
    pid_spd_trq.upper_limit_integral = -(Iq_ref_cmd << pid_spd_trq.ki_shift);
    pid_spd_trq.lower_limit_integral = (Iq_ref_cmd << pid_spd_trq.ki_shift);
    speed_ramp.cmd_final = -MAX_SPEED_RPM;
    speed_ramp.command = -MAX_SPEED_RPM;
  }
}

/**
  * @brief  Step position command is converted into S-curve command
  * @param  pos_handler: point to the parameters of the structure position_type
  * @param  cmd_ramp_handler: point to the parameters of the structure ramp_cmd_type
  * @param  pid_handler: point to the parameters of the structure pid_ctrl_type
  * @retval none
  */
void position_cmd_ramp(position_type *pos_handler, ramp_cmd_type *cmd_ramp_handler, pid_ctrl_type *pid_handler)
{
  int32_t brake_distance;
  int32_t Err;
  int32_t ramp_cmd_inc, ramp_cmd_dec;

  pos_handler->error = pos_handler->cmd_final - pos_handler->command;
  Err = pos_handler->cmd_final - pos_handler->val;

  if (pos_handler->cmd_final > pos_handler->command)  /* CW */
  {
    //brake_distance = (int32_t) ((pos_handler->rpm_to_cpr / 2) * (pos_handler->spd_cmd + 0.5*cmd_ramp_handler->dec_slope * pos_handler->spd_slope_ratio) * (pos_handler->spd_cmd + 0.5*cmd_ramp_handler->dec_slope * pos_handler->spd_slope_ratio) / (cmd_ramp_handler->dec_slope * pos_handler->spd_slope_ratio)); /*** S = V0^2 / 2a ***/
    brake_distance = (int32_t) ((pos_handler->rpm_to_cpr / 2) * (pos_handler->spd_cmd) * (pos_handler->spd_cmd) / (cmd_ramp_handler->dec_slope * pos_handler->spd_slope_ratio)); /*** S = V0^2 / 2a ***/

    if ((pos_handler->error <= brake_distance) && (pos_handler->brake_flag == RESET)) /* braking */
    {
      pos_handler->brake_flag = SET;
      pos_handler->spd_slope = -(cmd_ramp_handler->dec_slope * pos_handler->spd_slope_ratio); /* deceleration */
    }
    else if ((pos_handler->error > brake_distance) && (pos_handler->brake_flag == RESET)) /* acceleration */
    {
      pos_handler->spd_slope = cmd_ramp_handler->acc_slope * pos_handler->spd_slope_ratio; /* acceleration */
    }

    pos_handler->spd_cmd = pos_handler->spd_cmd + pos_handler->spd_slope;

    if (pos_handler->spd_cmd > pid_handler->upper_limit_output) /* Max speed limitation */
    {
      pos_handler->spd_cmd = pid_handler->upper_limit_output;
    }

    if (pos_handler->spd_cmd < cmd_ramp_handler->dec_slope * pos_handler->spd_slope_ratio)    /* avoid spd_cmd < 0 */
    {
      if ((pos_handler->cmd_final - pos_handler->val) > pos_handler->small_pos_cmd_gap)
      {
        pos_handler->spd_cmd = cmd_ramp_handler->dec_slope * pos_handler->spd_slope_ratio;
      }
      else
      {
        pos_handler->spd_cmd = pos_handler->min_pos_ctrl_spd;
      }
    }

    ramp_cmd_inc = (int32_t)(pos_handler->spd_cmd * pos_handler->rpm_to_cpr);
    pos_handler->command = pos_handler->command + ramp_cmd_inc; /* position ramp command */

    if((pos_handler->cmd_final - pos_handler->command) <= ramp_cmd_inc) /* positiom ramp command is close to position command */
    {
      pos_handler->command = pos_handler->cmd_final;
      pos_handler->brake_flag = RESET;
      pos_handler->spd_cmd = 0;
    }
  }
  else if (pos_handler->cmd_final < pos_handler->command) /* CCW */
  {
    //brake_distance = (int32_t) ((pos_handler->rpm_to_cpr / 2) * (pos_handler->spd_cmd - 0.5*cmd_ramp_handler->acc_slope * pos_handler->spd_slope_ratio) * (pos_handler->spd_cmd - 0.5*cmd_ramp_handler->acc_slope * pos_handler->spd_slope_ratio) / (cmd_ramp_handler->acc_slope * pos_handler->spd_slope_ratio)); /*** S = V0^2 / 2a ***/
    brake_distance = (int32_t) ((pos_handler->rpm_to_cpr / 2) * (pos_handler->spd_cmd) * (pos_handler->spd_cmd) / (cmd_ramp_handler->acc_slope * pos_handler->spd_slope_ratio)); /*** S = V0^2 / 2a ***/
    if ((-pos_handler->error <= brake_distance) && (pos_handler->brake_flag == RESET)) /* braking */
    {
      pos_handler->brake_flag = SET;
      pos_handler->spd_slope = cmd_ramp_handler->acc_slope * pos_handler->spd_slope_ratio; /* acceleration */
    }
    else if ((-pos_handler->error > brake_distance) && (pos_handler->brake_flag == RESET)) /* deceleration */
    {
      pos_handler->spd_slope = -(cmd_ramp_handler->dec_slope * pos_handler->spd_slope_ratio); /* deceleration */
    }

    pos_handler->spd_cmd = pos_handler->spd_cmd + pos_handler->spd_slope;

    if (pos_handler->spd_cmd < pid_handler->lower_limit_output) /* Min speed limitation */
    {
      pos_handler->spd_cmd = pid_handler->lower_limit_output;
    }

    if (pos_handler->spd_cmd > -cmd_ramp_handler->acc_slope * pos_handler->spd_slope_ratio)    /* avoid spd_cmd > 0 */
    {
      if ((pos_handler->cmd_final - pos_handler->val) < -(pos_handler->small_pos_cmd_gap))
      {
        pos_handler->spd_cmd = -cmd_ramp_handler->acc_slope * pos_handler->spd_slope_ratio;
      }
      else
      {
        pos_handler->spd_cmd = -(pos_handler->min_pos_ctrl_spd);
      }
    }

    ramp_cmd_dec = (int32_t)(pos_handler->spd_cmd * pos_handler->rpm_to_cpr);
    pos_handler->command = pos_handler->command + ramp_cmd_dec; /* position ramp command */

    if(pos_handler->cmd_final - pos_handler->command >= ramp_cmd_dec) /* positiom ramp command is close to position command */
    {
      pos_handler->command = pos_handler->cmd_final;
      pos_handler->brake_flag = RESET;
      pos_handler->spd_cmd = 0;
    }
  }

  if (pid_handler->integral > 0 && Err < 0) /* acculated error > 0 but actual error < 0 (CCW overshoot), clear the acculated error */
  {
    pid_handler->integral = 0;
  }
  else if (pid_handler->integral < 0 && Err > 0) /* acculated error < 0 but actual error > 0 (CW overshoot), clear the acculated error */
  {
    pid_handler->integral = 0;
  }

  if (abs(Err) < pos_handler->cmd_to_val_gap)    /* check if the position stable or not */
  {
    pos_handler->stable = TRUE;            /* position is stable, user could change position command */    /* position is close target value*/
    pid_handler->ki_gain = pid_handler->ki_gain_2nd;              /* enhance the ki_gain to reach the target quickly*/
    pos_handler->brake_flag = RESET;
  }
  else
  {
    pos_handler->stable = FALSE;
    pid_handler->ki_gain = pid_handler->ki_gain_1st;
  }

  if (Err == 0)
  {
    pos_handler->spd_cmd = 0;
    pos_handler->stable = TRUE;            /* position is stable, user could change position command */
  }
}

/**
  * @brief  position controller and command configuration
  * @param  none
  * @retval none
  */
void position_control_handler(void)
{
  int32_t pos_err;
#if defined HALL_SENSORS && defined LOW_SPEED_VOLT_CTRL
  static flag_status lock_flag = RESET;
#endif

  pos.cmd_new = (int32_t)(angle.cmd_final * ANGLE_TO_PULSE);   /* if angle.cmd_final = 36010, which mean 36010/100 = 360.1 degree */

  if (pos.cmd_new != pos.cmd_final)
  {
    if (pos.stable == TRUE)
    {
      pos.cmd_final = pos.cmd_new;
#if defined HALL_SENSORS && defined LOW_SPEED_VOLT_CTRL
      set_normal_pwm_mode();
      lock_flag = RESET;
#endif
      if (pos.cmd_final >= pos.val)
      {
        pid_pos.lower_limit_output = -pos.min_pos_ctrl_spd;
        pid_pos.lower_limit_integral = -pos.min_pos_ctrl_spd << (pid_pos.ki_shift);
        pid_pos.upper_limit_output = intCoeffs32[MC_PROTOCOL_REG_MAX_APP_SPEED];
        pid_pos.upper_limit_integral = intCoeffs32[MC_PROTOCOL_REG_MAX_APP_SPEED] << (pid_pos.ki_shift);
      }
      else
      {
        pid_pos.upper_limit_output = pos.min_pos_ctrl_spd;
        pid_pos.upper_limit_integral = pos.min_pos_ctrl_spd << (pid_pos.ki_shift);
        pid_pos.lower_limit_output = -intCoeffs32[MC_PROTOCOL_REG_MAX_APP_SPEED];
        pid_pos.lower_limit_integral = -(intCoeffs32[MC_PROTOCOL_REG_MAX_APP_SPEED] << (pid_pos.ki_shift));
      }
    }
    else
    {
      pos.cmd_new = pos.cmd_final;
      angle.cmd_final = (int32_t)(pos.cmd_new * PULSE_TO_ANGLE);
    }
  }
  position_cmd_ramp(&pos, &speed_ramp, &pid_pos);

  pos_err = pos.command - pos.val;

#if defined HALL_SENSORS && defined LOW_SPEED_VOLT_CTRL
  if (pos.stable == TRUE)
  {
    if (abs(pos_err) < ROTOR_LOCK_GAP)
    {
      lock_rotor();
      lock_flag = SET;
    }
  }
  else
  {
    if (lock_flag == SET)
    {
      set_normal_pwm_mode();
      lock_flag = RESET;
    }
    speed_ramp.cmd_final = pid_controller(&pid_pos, pos_err);
  }
#else
  speed_ramp.cmd_final = pid_controller(&pid_pos, pos_err);
#endif
  angle.command = (int32_t)(pos.command * PULSE_TO_ANGLE);
}

/**
  * @brief  fine-tune speed pi parameters at low speed control
  * @param  none
  * @retval none
  */
void speed_pid_param(void)
{
  if (speed_ramp.command >= 0)
  {
    if (speed_ramp.command == 0)
    {
      pid_spd.kp_gain = 400;
      pid_spd.ki_gain = 1500;
    }
    else if (speed_ramp.command > 0 && speed_ramp.command < 20)
    {
      pid_spd.kp_gain = 400;
      pid_spd.ki_gain = 1500;
    }
    else if (speed_ramp.command >= 20 && speed_ramp.command < 100)
    {
      pid_spd.kp_gain = 2000;
      pid_spd.ki_gain = 1000;
    }
    else if (speed_ramp.command >= 100)
    {
      pid_spd.kp_gain = 2000;
      pid_spd.ki_gain = 50;
    }
  }
  else
  {
    if (speed_ramp.command > -20)
    {
      pid_spd.kp_gain = 400;
      pid_spd.ki_gain = 1500;
    }
    else if (speed_ramp.command <= -20 && speed_ramp.command > -100)
    {
      pid_spd.kp_gain = 2000;
      pid_spd.ki_gain = 1000;
    }
    else if (speed_ramp.command <= -100)
    {
      pid_spd.kp_gain = 2000;
      pid_spd.ki_gain = 50;
    }
  }
}

/**
  * @brief  enable adc trigger for monitoring signals
  * @param  none
  * @retval none
  */
void monitoring_signal_adc_trigger(void)
{
  adc_ordinary_software_trigger_enable(ADC_CONVERTER, TRUE);
}

/**
  * @brief  space vector modulation function(1/2/3-shunt)
  * @param  volt_handler: voltage related variables
  * @param  pwm_duty_handler: three-phase pwm duty cycle and timer used
  * @retval none
  */
void svpwm_func(voltage_type *volt_handler, pwm_duty_type *pwm_duty_handler)
{
#if defined THREE_SHUNT
  svpwm_3shunt(volt_handler, pwm_duty_handler);
#elif defined TWO_SHUNT
  svpwm_2shunt(volt_handler, pwm_duty_handler);
#elif defined ONE_SHUNT
  svpwm_1shunt(volt_handler, pwm_duty_handler);
#if defined AT32F423xx || defined AT32F425xx || defined AT32F402xx || defined AT32F405xx
  pwm_duty_handler->adc_trig.first_pos = DOUBLE_PWM_PERIOD - pwm_duty_handler->adc_trig.first_pos;
  pwm_duty_handler->adc_trig.second_temp = DOUBLE_PWM_PERIOD - pwm_duty_handler->adc_trig.second_temp;
#endif
#endif
}

/**
  * @brief  update pwm duty in overflow
  * @param  pwm_duty_handler: three-phase pwm duty cycle and timer used
  * @retval none
  */
void pwm_duty_update(pwm_duty_type *pwm_duty_handler)
{
#ifdef ONE_SHUNT
  tmr_channel_value_set(pwm_duty_handler->adc_TMRx, pwm_duty_handler->adc_tmr_trig_ch, pwm_duty_handler->adc_trig.first_pos); /* trigger adc current sensing */
  pwm_duty_handler->adc_trig.second_pos[0] = pwm_duty_handler->adc_trig.second_temp;
#endif
  tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_1, pwm_duty_handler->UF.a);
#ifdef HALL_SENSORS
  if (hall_learn.dir == 1)
  {
    tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_2, pwm_duty_handler->UF.c);
    tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_3, pwm_duty_handler->UF.b);
  }
  else
  {
    tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_2, pwm_duty_handler->UF.b);
    tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_3, pwm_duty_handler->UF.c);
  }
#else
  tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_2, pwm_duty_handler->UF.b);
  tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_3, pwm_duty_handler->UF.c);
#endif
}

/**
  * @brief  update pwm duty in underflow
  * @param  pwm_duty_handler: three-phase pwm duty cycle and timer used
  * @retval none
  */
void pwm_duty_extra_update(pwm_duty_type *pwm_duty_handler)
{
  tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_1, pwm_duty_handler->OF.a);
#ifdef HALL_SENSORS
  if (hall_learn.dir == 1)
  {
    tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_2, pwm_duty_handler->OF.c);
    tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_3, pwm_duty_handler->OF.b);
  }
  else
  {
    tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_2, pwm_duty_handler->OF.b);
    tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_3, pwm_duty_handler->OF.c);
  }
#else
  tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_2, pwm_duty_handler->OF.b);
  tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_3, pwm_duty_handler->OF.c);
#endif
}

#if (defined AT32M412xx || defined AT32M416xx)
/**
  * @brief  update asymmetric pwm duty
  * @param  pwm_duty_handler: three-phase pwm duty cycle and timer used
  * @retval none
  */
void asym_pwm_duty_update(pwm_duty_type *pwm_duty_handler)
{
  tmr_channel_value_set(pwm_duty_handler->adc_TMRx, pwm_duty_handler->adc_tmr_trig_ch, pwm_duty_handler->adc_trig.first_pos); /* trigger adc current sensing */
  pwm_duty_handler->adc_trig.second_pos[0] = pwm_duty_handler->adc_trig.second_temp;

  tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_1, pwm_duty_handler->OF.a);
  tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_1A, pwm_duty_handler->UF.a);
#ifdef HALL_SENSORS
  if (hall_learn.dir == 1)
  {
    tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_2, pwm_duty_handler->OF.c);
    tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_3, pwm_duty_handler->OF.b);
    tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_2A, pwm_duty_handler->UF.c);
    tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_3A, pwm_duty_handler->UF.b);
  }
  else
  {
    tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_2, pwm_duty_handler->OF.b);
    tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_3, pwm_duty_handler->OF.c);
    tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_2A, pwm_duty_handler->UF.b);
    tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_3A, pwm_duty_handler->UF.c);
  }
#else
  tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_2, pwm_duty_handler->OF.b);
  tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_3, pwm_duty_handler->OF.c);
  tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_2A, pwm_duty_handler->UF.b);
  tmr_channel_value_set(pwm_duty_handler->ADVTMRx, TMR_SELECT_CHANNEL_3A, pwm_duty_handler->UF.c);
#endif
}
#endif

/**
  * @brief  update pwm duty in overflow and underflow
  * @param  pwm_duty_handler: three-phase pwm duty cycle and timer used
  * @retval none
  */
void pwm_duty_buffer(pwm_duty_type *pwm_duty_handler)
{
  tmr_channel_value_set(pwm_duty_handler->adc_TMRx, pwm_duty_handler->adc_tmr_trig_ch, pwm_duty_handler->adc_trig.first_pos); /* trigger adc current sensing */
  pwm_duty_handler->adc_trig.second_pos[0] = pwm_duty_handler->adc_trig.second_temp;
#ifdef HALL_SENSORS
  pwm_duty_handler->duty_buffer[0] = pwm_duty_handler->OF.a;
  pwm_duty_handler->duty_buffer[3] = pwm_duty_handler->UF.a;
  if (hall_learn.dir == 1)
  {
    pwm_duty_handler->duty_buffer[1] = pwm_duty_handler->OF.c;
    pwm_duty_handler->duty_buffer[2] = pwm_duty_handler->OF.b;
    pwm_duty_handler->duty_buffer[4] = pwm_duty_handler->UF.c;
    pwm_duty_handler->duty_buffer[5] = pwm_duty_handler->UF.b;
  }
  else
  {
    pwm_duty_handler->duty_buffer[1] = pwm_duty_handler->OF.b;
    pwm_duty_handler->duty_buffer[2] = pwm_duty_handler->OF.c;
    pwm_duty_handler->duty_buffer[4] = pwm_duty_handler->UF.b;
    pwm_duty_handler->duty_buffer[5] = pwm_duty_handler->UF.c;
  }
#else
  pwm_duty_handler->duty_buffer[0] = pwm_duty_handler->OF.a;
  pwm_duty_handler->duty_buffer[1] = pwm_duty_handler->OF.b;
  pwm_duty_handler->duty_buffer[2] = pwm_duty_handler->OF.c;
  pwm_duty_handler->duty_buffer[3] = pwm_duty_handler->UF.a;
  pwm_duty_handler->duty_buffer[4] = pwm_duty_handler->UF.b;
  pwm_duty_handler->duty_buffer[5] = pwm_duty_handler->UF.c;
#endif
}

#if defined HALL_SENSORS && defined LOW_SPEED_VOLT_CTRL
/**
  * @brief  set pwm output mode to normal pwm control mode
  * @param  none
  * @retval none
  */
void set_normal_pwm_mode(void)
{
  /* clear pwm timer PWM control mode */
  PWM_ADVANCE_TIMER->cm1 &= (~TMR_PWM_MODE_CM1_MASK);
  PWM_ADVANCE_TIMER->cm2 &= (~TMR_PWM_MODE_CM2_MASK);
  /* set pwm timer PWM control mode */
  PWM_ADVANCE_TIMER->cm1 |= TMR_PWM_1PWMA_2PWMA;
  PWM_ADVANCE_TIMER->cm2 |= TMR_PWM_3PWMA;

  /* change tmr channel mode */
  PWM_ADVANCE_TIMER->swevt |= TMR_HALL_SWTRIG;
}

/**
  * @brief  set pwm output mode to turn on the low side power switches of three phase
  * @param  none
  * @retval none
  */
void lock_rotor(void)
{
  /* clear pwm timer PWM control mode */
  PWM_ADVANCE_TIMER->cm1 &= (~TMR_PWM_MODE_CM1_MASK);
  PWM_ADVANCE_TIMER->cm2 &= (~TMR_PWM_MODE_CM2_MASK);
  /* set pwm timer PWM control mode */
  PWM_ADVANCE_TIMER->cm1 |= TMR_PWM_1LOW_2LOW;
  PWM_ADVANCE_TIMER->cm2 |= TMR_PWM_3LOW;

  /* change tmr channel mode */
  PWM_ADVANCE_TIMER->swevt |= TMR_HALL_SWTRIG;

  hall.theta_inc = 0;
  rotor_speed_hall.filtered = 0;
  reset_ma_buffer(hall_interval_moving_average);
  hall.offset = 0;
  speed_ramp.cmd_final = 0;
  speed_ramp.command = 0;
  pid_spd.integral = 0;
  pid_spd_volt.integral = 0;
  pid_iq.integral = 0;
  pid_id.integral = 0;
  error_code |= error_code_mask & hall_at_zero_speed(&hall, &rotor_angle_hall, hall_next_state_table);
}
#endif

/**
  * @brief  motor parameter identification process
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
    pwm_switch_off();
    disable_pwm_timer(&pwm_duty);
    tmr_pwm_init();
    adc_preempt_config();
    enable_pwm_timer(&pwm_duty);
    esc_state = ESC_STATE_FREE_RUN;
  }
  else if (motor_param_ident.state_flag == FAILED)
  {
    pwm_switch_off();
    disable_pwm_timer(&pwm_duty);
    tmr_pwm_init();
    adc_preempt_config();
    enable_pwm_timer(&pwm_duty);
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

  i_tune_handler->Vdc_rated = i_tune_handler->Vdc_rated_adc/4095.0f*i_tune_handler->voltage_base;

  cal_temp = 2*2*PI*0.35f/(3.0f*(*i_tune_handler->Ls)/(*i_tune_handler->Rs));

  if (cal_temp > i_tune_handler->bandwidth)
  {
    i_tune_handler->bandwidth = (uint16_t)cal_temp;
  }
  if (i_tune_handler->bandwidth > i_tune_handler->BW_limit)
  {
    i_tune_handler->bandwidth = i_tune_handler->BW_limit;
  }

  i_tune_handler->kp_coff = (float)i_tune_handler->bandwidth * (*i_tune_handler->Ls) * SQRT_3 * i_tune_handler->current_base / i_tune_handler->Vdc_rated;

  mul_temp = (uint16_t)(13 - round(log2(i_tune_handler->kp_coff)));

  if (mul_temp < 15)
    i_tune_handler->kp_shift = mul_temp;
  else
    i_tune_handler->kp_shift = 15;

  i_tune_handler->kp = (int16_t)(i_tune_handler->kp_coff * (1 << i_tune_handler->kp_shift));

  i_tune_handler->ki_coff = (float)i_tune_handler->bandwidth * (*i_tune_handler->Rs) * SQRT_3 * i_tune_handler->current_base / i_tune_handler->Vdc_rated / i_tune_handler->sample_freq;

  mul_temp = (uint16_t)(13 - round(log2(i_tune_handler->ki_coff)));

  if (mul_temp < 15)
    i_tune_handler->ki_shift = mul_temp;
  else
    i_tune_handler->ki_shift = 15;

  i_tune_handler->ki = (int16_t)(i_tune_handler->ki_coff * (1 << i_tune_handler->ki_shift));
}

#ifdef IPM_MTPA_MTPV_CTRL
/**
  * @brief  MTPA/MTPV control for IPMSM motor
  * @param  none
  * @retval none
  */
void ipmsm_mtpa_control(void)
{
  int32_t spd_temp;
  int32_t X, Y;

  X = current.Tref * X_SPAN;

  rotor_speed_abs_val = abs(rotor_speed_val);

  spd_temp = VDC_RATED_ADC*rotor_speed_abs_val/adc_in_tab[ADC_BUS_VOLT_IDX] - MTPA_MTPV_TABLE_MIN_SPEED;

  if (spd_temp > 0)
  {
    Y = spd_temp * Y_SPAN;
    if (Y >= Y_MAX_LIMIT)
    {
      Y = Y_MAX_LIMIT;
    }
  }
  else
  {
    Y=0;
  }

  if (X >= 0)
  {
    current.Iqdref.q = arm_bilinear_interp_q15(&iq_lookup_table, X, Y);
    current.Iqdref.d = arm_bilinear_interp_q15(&id_lookup_table, X, Y);
  }
  else
  {
    current.Iqdref.q = -arm_bilinear_interp_q15(&iq_lookup_table, -X, Y);
    current.Iqdref.d = arm_bilinear_interp_q15(&id_lookup_table, -X, Y);
  }
}
#endif

#ifdef E_BIKE_SCOOTER
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
  ideal_vbus_value= *(intCoeffs32_p+MC_PROTOCOL_REG_I_TUNE_VDC_RATE);
  dc_volt_adc_value = (uint32_t)adc_in_tab[ADC_BUS_VOLT_IDX]*vref_ratio>>14;

  if (dc_volt_adc_value < GAIN_RATIO_MIN_VOLT)
    dc_volt_adc_value = GAIN_RATIO_MIN_VOLT;

  volt_control_gain_ratio = (uint16_t)(((float)ideal_vbus_value*VBUS_GAIN_RATIO)/dc_volt_adc_value);

  return(volt_control_gain_ratio);
}

/**
 * @brief  Adjusts control parameters based on DC bus voltage ratio
 * @param  value       Original parameter value (tuned at nominal voltage)
 * @param  Vdc_ratio   Voltage compensation ratio from calcVdcRatio()
 * @retval int16_t     Voltage-compensated parameter value
 */
int16_t adjustValueByVdc(int16_t value,int16_t Vdc_ratio)
{
  int16_t value_ajusted;

  value_ajusted = (int16_t)(((uint32_t)value* Vdc_ratio)>>VBUS_GAIN_LOG);

  return(value_ajusted);
}

/**
  * @brief  Brake input handler function(E_BIKE_SCOOTER Mode)
  * @param  none
  * @retval none
  */
void brake_input_handler(void)
{
  static uint8_t count1;

  if (gpio_input_data_bit_read(BRAKE_SW_PORT, BRAKE_SW_PIN) == RESET)
  {
    if (++count1>20)
      count1=20;
  }
  else
    count1 = 0;

  if (count1 >=20)
    brake_flag = SET;
  else
    brake_flag = RESET;
}

/**
  * @brief  Reverse input handler function(E_BIKE_SCOOTER Mode)
  * @param  none
  * @retval none
  */
void reverse_input_handler(void)
{
  static uint8_t count1;

  if (gpio_input_data_bit_read(REVERSE_SW_PORT, REVERSE_SW_PIN) == RESET)
  {
    if (++count1>20)
      count1=20;
  }
  else
    count1 = 0;

  if (count1 >=20)
    reverse_flag = SET;
  else
    reverse_flag = RESET;
}

/**
  * @brief  Anti-theft mode input handler function(E_BIKE_SCOOTER Mode)
  * @param  none
  * @retval none
  */
void anti_theft_input_handler(void)
{
  static uint8_t count1;

  if (gpio_input_data_bit_read(LOCK_MOTOR_SW_PORT, LOCK_MOTOR_SW_PIN) == RESET)
  {
    if (++count1>20)
      count1=20;
  }
  else
    count1 = 0;

  if (count1 >=20)
    anti_theft_flag = SET;
  else
    anti_theft_flag = RESET;
}

/**
  * @brief  Parking mode input handler function(E_BIKE_SCOOTER Mode)
  * @param  none
  * @retval none
  */
void parking_lock_input_handler(void)
{
  static uint8_t count1;

  if (gpio_input_data_bit_read(PARKING_LOCK_SW_PORT, PARKING_LOCK_SW_PIN) == RESET)
  {
    if (++count1>20)
      count1=20;
  }
  else
    count1 = 0;

  if (count1 >=20)
    parking_lock_flag = SET;
  else
    parking_lock_flag = RESET;
}

/**
  * @brief  External input handler function(E_BIKE_SCOOTER Mode)
  * @param  none
  * @retval none
  */
void external_input_handler(void)
{
  sp_value = adc_in_tab[ADC_POTENTIO_IDX] - SP_OFFSET;

  if (ctrl_mode == TORQUE_CTRL)
  {
    if (sp_value >= SP_RUN_POINT && brake_flag == RESET)
    {
      Iq_ref_cmd = (sp_value * SP_TO_I_CMD) >> 15;
#if defined (M412_LV_V1_0)
      start_stop_btn_flag = SET;
#endif
    }
    else
    {
      Iq_ref_cmd = 0;
#if defined (M412_LV_V1_0)
      start_stop_btn_flag = RESET;
#endif
    }
  }
  else if (ctrl_mode == SPEED_CTRL)
  {
    if (sp_value >= SP_RUN_POINT && brake_flag == RESET)
    {
      speed_ramp.cmd_final = ((sp_value << 1) * SP_TO_SPD_CMD) >> 15;
#if defined (M412_LV_V1_0)
      start_stop_btn_flag = SET;
#endif
    }
    else
    {
      speed_ramp.cmd_final = 0;
#if defined (M412_LV_V1_0)
      start_stop_btn_flag = RESET;
#endif
    }
  }
}
#else
/**
  * @brief  External input handler function
  * @param  none
  * @retval none
  */
void external_input_handler(void)
{
  sp_value = adc_in_tab[ADC_POTENTIO_IDX] - SP_OFFSET;

  if (ctrl_mode == TORQUE_CTRL)
  {
    if (sp_value >= SP_RUN_POINT)
    {
#if defined TORQUE_CTRL_WITH_SPEED_LIMIT
      Iq_ref_cmd = (sp_value * SP_TO_I_CMD) >> 15;;
      curr_cmd_handler_speed_limit();
#else
      current.Iqdref.q = (sp_value * SP_TO_I_CMD) >> 15;
#endif
#if defined (M412_LV_V1_0)
      start_stop_btn_flag = SET;
#endif
    }
    else
    {
      current.Iqdref.q = 0;
#if defined (M412_LV_V1_0)
      start_stop_btn_flag = RESET;
#endif
    }
  }
  else if (ctrl_mode == SPEED_CTRL)
  {
    if (sp_value >= SP_RUN_POINT)
    {
      speed_ramp.cmd_final = ((sp_value << 1) * SP_TO_SPD_CMD) >> 15;
#if defined (M412_LV_V1_0)
      start_stop_btn_flag = SET;
#endif
    }
#ifdef SENSORLESS
    else if (sp_value < SP_RUN_POINT && sp_value > SP_STOP_POINT)
    {
      speed_ramp.cmd_final = MIN_SPEED_RPM;
    }
#endif
    else
    {
      speed_ramp.cmd_final = 0;
#if defined (M412_LV_V1_0)
      start_stop_btn_flag = RESET;
#endif
    }
  }
}
#endif

/**
  * @brief  Controls LED blinking behavior for normal motor operation state
  * @param  led_blink_count : Pointer to the LED blink counter variable.
  *         The counter is incremented each call and reset when reaching LED_BLINK_PERIOD.
  * @param  led_gpio_port : to select the led gpio peripheral.
  * @param  led_gpio_pin: led gpio pin number
  * @retval none
  */
void normal_state_led_blink(uint16_t *led_blink_count,gpio_type *led_gpio_port,uint16_t led_gpio_pin)
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
        (*led_blink_count)++;
    }
  }
  else
  {
    led_off(led_gpio_port, led_gpio_pin);
    led_blink_count = 0;
  }
}
