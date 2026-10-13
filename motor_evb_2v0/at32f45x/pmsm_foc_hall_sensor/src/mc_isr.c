/**
  **************************************************************************
  * @file     mc_isr.c
  * @brief    Relevant motor control interrupt functions
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

trig_components_type trig_components;
int16_t Iq_cmd;
int8_t Iq_cmd_count;

#ifdef LOW_SPEED_VOLT_CTRL
/**
  * @brief  Prepare an inactive PID so its next output continues from the
  *         currently active controller output.
  */
static void pid_prepare_output(pid_ctrl_type *pid_handler, int32_t var_err, int16_t target_output)
{
  int32_t tracked_output;
  int32_t proportion_output;
  int64_t integral;

  tracked_output = target_output;
  if (tracked_output > pid_handler->upper_limit_output)
  {
    tracked_output = pid_handler->upper_limit_output;
  }
  else if (tracked_output < pid_handler->lower_limit_output)
  {
    tracked_output = pid_handler->lower_limit_output;
  }

  proportion_output = (int32_t)(((int64_t)pid_handler->kp_gain * var_err) >> pid_handler->kp_shift);
  integral = ((int64_t)(tracked_output - proportion_output) * ((int64_t)1 << pid_handler->ki_shift))
             - ((int64_t)pid_handler->ki_gain * var_err);

  if (integral > pid_handler->upper_limit_integral)
  {
    integral = pid_handler->upper_limit_integral;
  }
  else if (integral < pid_handler->lower_limit_integral)
  {
    integral = pid_handler->lower_limit_integral;
  }

  pid_handler->integral = (int32_t)integral;
  pid_handler->error = var_err;
  pid_handler->pre_error = var_err;
  pid_handler->out_start = tracked_output;
}


#endif
/** @addtogroup motor_evb_2v0
  * @{
  */

/** @addtogroup pmsm_foc_hall_sensor pmsm_foc_hall_sensor
  * @{
  */

/** @defgroup mc_isr
  * @brief Relevant motor control interrupt functions
  * @{
  */

/**
  * @brief  PWM timer interrupt handler for update event
  * @param  none
  * @retval none
  */
void ADVTMR_PWM_CYCLE_IRQ(void)
{
#if defined USE_MOTOR_MONITOR
  static int8_t ui_count = 0;
#endif
  int16_t position_delta;

  if (tmr_flag_get(PWM_ADVANCE_TIMER, TMR_OVF_FLAG) != RESET)
  {
    tmr_flag_clear(PWM_ADVANCE_TIMER, TMR_OVF_FLAG);

#ifdef ONE_SHUNT
    if ((PWM_ADVANCE_TIMER->ctrl1_bit.cnt_dir & TMR_COUNT_DOWN) == RESET)  /* underflow interrupt */
    {
#endif
      monitoring_signal_adc_trigger(); /* adc orydinary software trigger */

#ifdef MOTOR_PARAM_IDENTIFY
      if (motor_param_ident.state_flag == PROCESSING)
      {
        if (motor_param_ident.id_flag == SET)
        {
          param_identify(&motor_param_ident);
        }
#ifdef ONE_SHUNT
        if (esc_state == ESC_STATE_WINDING_PARAM_ID)
        {
          tmr_channel_value_set(ADC_TIMER, ADC_TIMER_SELECT_CHANNEL, motor_param_ident.duty); /* trigger adc current sensing */
        }
#endif
      }
#endif

      error_code |= error_code_mask & read_hall_state(&hall);
      rotor_angle_hall.elec_angle_val = hall_rotor_angle_get(&hall, &rotor_angle_hall, hall_theta_table);
      position_delta = rotor_angle_hall.elec_angle_val - rotor_angle_hall.elec_angle_pre_val;
      rotor_angle_hall.elec_angle_pre_val = rotor_angle_hall.elec_angle_val;
      if (position_delta >= HALF_TURN_DEGREE)
      {
        rotor_angle_hall.mech_anlge_val--;
      }
      else if (position_delta <= -HALF_TURN_DEGREE)
      {
        rotor_angle_hall.mech_anlge_val++;
      }

      if (curr_offset_rdy & foc_rdy)
      {
#ifdef ONE_SHUNT
        /* pwm output */
        pwm_duty_update(&pwm_duty);
#endif

#if defined TWO_ADC_CONVERTERS && defined THREE_SHUNT
        /* adc channels selection */
        select_two_adc_channels();
#endif
        if(ctrl_mode == OPEN_LOOP_CTRL)
        {
          foc_open_loop_ctrl(&volt_cmd, &openloop);

          elec_angle_val = openloop.theta;

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

          /* svpwm function */
          svpwm_func(&volt_cmd, &pwm_duty);
        }
        else
        {
          elec_angle_val = rotor_angle_hall.elec_angle_val;

          /* trigonometric functions transformation */
          trig_components = trig_functions(elec_angle_val);

          /* Clarke transformation */
          foc_clarke_trans(&current.Iabc, &current.Ialphabeta);

          /* Park transformation */
          foc_park_trans(&current, &trig_components);

#if CURRENT_LP_FILTER
          /* low pass filter for q/d-axis current */
          current.Iqd_LPF.d = lowpass_filtering(&d_current_LPF, current.Iqd.d);
          current.Iqd_LPF.q = lowpass_filtering(&q_current_LPF, current.Iqd.q);
          Ival.d = current.Iqd_LPF.d;
          Ival.q = current.Iqd_LPF.q;
#else
          Ival.d = current.Iqd.d;
          Ival.q = current.Iqd.q;
#endif

          Iref.d = current.Iqdref.d;
          Iref.q = current.Iqdref.q;

          /* Torque control */
#ifdef LOW_SPEED_VOLT_CTRL
          if (ctrl_mode == TORQUE_CTRL || ctrl_mode == ID_MANUAL_TUNE || ctrl_mode == IQ_MANUAL_TUNE)
#else
          if (ctrl_mode == POSITION_CTRL || ctrl_mode == SPEED_CTRL || ctrl_mode == TORQUE_CTRL || ctrl_mode == ID_MANUAL_TUNE || ctrl_mode == IQ_MANUAL_TUNE)
#endif
          {
#ifdef CURRENT_DECOUPLE_CTRL
            voltage_comp();
#else
            volt_cmd.Vqd.d = pid_controller(&pid_id, (Iref.d - Ival.d));

            /* foc Vq limitation */
            foc_vq_limitation(&volt_cmd, &pid_iq);

            volt_cmd.Vqd.q = pid_controller(&pid_iq, (Iref.q - Ival.q));
#endif
#if defined FIELD_WEAKENING
            field_weakening.fw_err_filt = ma_filter(pid_iq.upper_limit_output - abs(pid_iq.out_start), field_weakening.fw_err_filt, FW_MA_NBR_LOG);
#endif
          }
          else if (ctrl_mode == VF_CTRL)
          {
            /* foc circle limitation */
            foc_circle_limitation(&volt_cmd);
          }

#ifdef LOW_SPEED_VOLT_CTRL
          else if ((ctrl_mode == SPEED_CTRL) || (ctrl_mode == POSITION_CTRL))
          {
            if (low_spd_ctrl_mode == WITHOUT_CURR_CTRL)
            {
              pid_prepare_output(&pid_id, (Iref.d - Ival.d), volt_cmd.Vqd.d);
              pid_prepare_output(&pid_iq, (Iref.q - Ival.q), volt_cmd.Vqd.q);
            }
            else
            {
#ifdef CURRENT_DECOUPLE_CTRL
              (void)voltage_comp();
#else
              volt_cmd.Vqd.d = pid_controller(&pid_id, (Iref.d - Ival.d));

              /* foc Vq limitation */
              foc_vq_limitation(&volt_cmd, &pid_iq);

              volt_cmd.Vqd.q = pid_controller(&pid_iq, (Iref.q - Ival.q));
#endif
#if defined FIELD_WEAKENING
              field_weakening.fw_err_filt = ma_filter(pid_iq.upper_limit_output - abs(pid_iq.out_start), field_weakening.fw_err_filt, FW_MA_NBR_LOG);
#endif
            }
          }
#endif
          /* Inverse Park transformation */
          foc_inver_park_trans(&volt_cmd, &trig_components);

          /* svpwm function */
          svpwm_func(&volt_cmd, &pwm_duty);
        }
#ifndef ONE_SHUNT
        /* pwm output */
        pwm_duty_update(&pwm_duty);
#endif
      }
#ifdef BRAKING_RESISTOR
      if (adc_in_tab[ADC_BUS_VOLT_IDX] > HYSTERESIS_HIGH_VOLTAGE_d)
      {
        tmr_channel_enable(BRAKE_PWM_TIMER, BRAKE_PWM_TIMER_CH, TRUE);
      }
      else if (adc_in_tab[ADC_BUS_VOLT_IDX] < HYSTERESIS_LOW_VOLTAGE_d)
      {
        tmr_channel_enable(BRAKE_PWM_TIMER, BRAKE_PWM_TIMER_CH, FALSE);
      }
#endif
      /* UI sample */
#if defined USE_MOTOR_MONITOR
      if(++ui_count >= ui_wave_param.sample_cycle)
      {
#ifdef MOTOR_PARAM_IDENTIFY
        if (motor_param_ident.state_flag == PROCESSING)
        {
          /* Parameter-ID diagnostics: PWM count and library step (0...9). */
          ui_wave_param.user_define_a = (int16_t)motor_param_ident.duty;
          ui_wave_param.user_define_b = (int16_t)motor_param_ident.step_flag;
        }
        else if (motor_param_ident.state_flag != FAILED)
#endif
        {
          ui_wave_param.user_define_a = (int16_t)hall.state;
          ui_wave_param.user_define_b = (int16_t)elec_angle_val;
        }
        ui_save_monitor_data();
        ui_count = 0;
      }
#endif

#ifdef ONE_SHUNT
    }
    else    /* overflow interrupt */
    {
      if (curr_offset_rdy & foc_rdy)
      {
        /* pwm output */
        pwm_duty_extra_update(&pwm_duty);
      }
    }
#endif
  }
}

/**
  * @brief  timer interrupt handler for break event
  * @param  none
  * @retval none
  */
void ADVTMR_PWM_BRK_IRQ(void)
{
  if (tmr_flag_get(PWM_ADVANCE_TIMER, TMR_BRK_FLAG) != RESET)
  {
    ui_wave_param.user_define_b = OVERCURRENT_DIAG_PWM_BRK;
    /* nFAULT is active: immediately disable the complete power stage. */
    gate_driver_emergency_shutdown();
    tmr_flag_clear(PWM_ADVANCE_TIMER, TMR_BRK_FLAG);

    /* Latch the fault and discard the command that was running. */
    foc_rdy = RESET;
    start_stop_btn_flag = RESET;
    error_code |= error_code_mask & MC_OVER_CURRENT_ERROR;
    esc_state = ESC_STATE_ERROR;
  }
}

/**
  * @brief  adc interrupt handler for end of preempted channel conversion
  * @param  none
  * @retval none
  */
void ADC_SHUNT_SAMP_READY_IRQ(void)
{
#ifndef MOS_RDS_SHUNT
  static uint8_t vmor_confirm_count = 0U;
  static flag_status vmor_pending = RESET;

  if(adc_interrupt_flag_get(ADC_CONVERTER, ADC_VMOR_FLAG) != RESET)
  {
    adc_flag_clear(ADC_CONVERTER, ADC_VMOR_FLAG);
    vmor_pending = SET;
  }
#if defined TWO_ADC_CONVERTERS
  if(adc_interrupt_flag_get(ADC_SIMULTANE_CONVERTER, ADC_VMOR_FLAG) != RESET)
  {
    adc_flag_clear(ADC_SIMULTANE_CONVERTER, ADC_VMOR_FLAG);
    vmor_pending = SET;
  }
#endif
#endif
  if (adc_flag_get(ADC_CONVERTER, ADC_PCCE_FLAG) != RESET)
  {
    adc_flag_clear(ADC_CONVERTER, ADC_PCCE_FLAG);

#ifndef MOS_RDS_SHUNT
    if (vmor_pending != RESET)
    {
      if (vmor_confirm_count < ADC_VMOR_CONFIRM_COUNT)
      {
        vmor_confirm_count++;
      }
      if (vmor_confirm_count >= ADC_VMOR_CONFIRM_COUNT)
      {
        ui_wave_param.user_define_b = OVERCURRENT_DIAG_ADC;
        error_code |= error_code_mask & MC_OVER_CURRENT_ERROR;
      }
      vmor_pending = RESET;
    }
    else
    {
      vmor_confirm_count = 0U;
    }
#endif

    if (esc_state != ESC_STATE_ANGLE_INIT)
    {
#if defined TWO_ADC_CONVERTERS
#if defined THREE_SHUNT
      current_read_foc_3shunt_2adc(&current);
#elif defined TWO_SHUNT
      current_read_foc_2shunt_2adc(&current);
#endif

#else
#if defined THREE_SHUNT
      current_read_foc_3shunt(&current, &pwm_duty);
#elif defined TWO_SHUNT
      current_read_foc_2shunt(&current);
#elif defined ONE_SHUNT
#ifdef MOTOR_PARAM_IDENTIFY
      if (esc_state == ESC_STATE_WINDING_PARAM_ID)
      {
        current_read_1shunt_ID(&current);
      }
      else
#endif
        current_read_foc_1shunt(&current, volt_cmd.sector_old);
#endif
#endif

#if defined INVERT_PHASE_CURRENT_POLARITY
      current.Iabc.a = -current.Iabc.a;
      current.Iabc.b = -current.Iabc.b;
      current.Iabc.c = -current.Iabc.c;
#endif

#if AC_CURRENT_LP_FILTER
      current.Iabc.a = lowpass_filtering(&ia_current_LPF, current.Iabc.a);
      current.Iabc.b = lowpass_filtering(&ib_current_LPF, current.Iabc.b);
      current.Iabc.c = -(current.Iabc.a + current.Iabc.b);
#endif
    }
  }
}

/**
  * @brief  shared EXINT handler for Hall edges on PB5/PB6/PB7
  * @param  none
  * @retval none
  */
#if defined HALL_EXINT_EDGE_CAPTURE
void HALL_EXINT_IRQ(void)
{
  uint8_t hall_state;
  uint32_t hall_interval;

  if (exint_interrupt_flag_get(HALL_EXINT_LINES) != RESET)
  {
    /*
     * Clear the grouped pending bits once, then process one coherent GPIOB
     * snapshot even if more than one Hall line is pending.
     */
    exint_flag_clear(HALL_EXINT_LINES);
    hall_state = HALL_GPIO_STATE_GET();
    hall_interval = tmr_counter_value_get(HALL_CAPTURE_TIMER);

    /*
     * Reading CVAL before OVF closes the wrap race: if overflow happens
     * before or during this check, the real interval is at least one full
     * timer period and must be saturated instead of treated as a fast edge.
     */
    if(tmr_flag_get(HALL_CAPTURE_TIMER, TMR_OVF_FLAG) != RESET)
    {
      hall_interval = MAX_CAP_COUNT;
    }

    hall_edge_isr_handler(hall_state, hall_interval);
  }
}
#endif

/**
  * @brief  hall timer interrupt handler for the no-edge timeout
  * @param  none
  * @retval none
  */
void HALL_CAPTURE_IRQ(void)
{
  hall_isr_handler();
}

/**
  * @brief  timer interrupt handler for speed control loop
  * @param  none
  * @retval none
  */
void SPEED_LOOP_TIMER_IRQ(void)
{
  int32_t delta;
#ifdef LOW_SPEED_VOLT_CTRL
  int32_t switch_speed;
  int32_t switch_command;
  int32_t low_speed_release_current;
  int32_t low_speed_vq_target;
  int32_t low_speed_vq_delta;
  int64_t low_speed_current_sq;
  static int32_t low_speed_vq_applied = 0;
  static flag_status low_speed_current_limit_active = RESET;
#endif

  if (tmr_flag_get(SPEED_LOOP_TIMER, TMR_OVF_FLAG) != RESET)
  {
    /* clear flags of overflow events */
    tmr_flag_clear(SPEED_LOOP_TIMER, TMR_OVF_FLAG);

    delta = ((rotor_angle_hall.mech_anlge_val - rotor_angle_hall_old.mech_anlge_val) << 15) + (rotor_angle_hall.elec_angle_val - rotor_angle_hall_old.elec_angle_val);
    rotor_angle_hall_old = rotor_angle_hall;
    pos.val += delta;
    angle.val = (int32_t)(pos.val * PULSE_TO_ANGLE);

#ifdef LOW_SPEED_VOLT_CTRL
    if ((esc_state != ESC_STATE_RUNNING) ||
        ((ctrl_mode != SPEED_CTRL) && (ctrl_mode != POSITION_CTRL)))
    {
      low_spd_ctrl_mode = WITHOUT_CURR_CTRL;
      low_spd_switch_count = 0U;
      low_speed_vq_applied = 0;
      low_speed_current_limit_active = RESET;
    }
#endif

    if (esc_state == ESC_STATE_RUNNING)
    {
      if (ctrl_mode == SPEED_CTRL || ctrl_mode == POSITION_CTRL)
      {
#ifdef LOW_SPEED_VOLT_CTRL
        /* position control loop */
        if (ctrl_mode == POSITION_CTRL)
        {
          if (pos.count < POSITION_LOOP_COUNT)
            pos.count++;
          else
          {
            pos.count = 0;
            position_control_handler();
          }
        }
        else
          pos.count = 0;

        /*
         * Enter current control quickly to limit acceleration overshoot.
         * The slower filtered speed is retained for the return to voltage
         * control, where chatter immunity matters more than latency.
         */
        switch_speed = (low_spd_ctrl_mode == WITHOUT_CURR_CTRL) ?
                       rotor_speed_val : rotor_speed_val_filt;
        if (switch_speed < 0)
        {
          switch_speed = -switch_speed;
        }
        switch_command = speed_ramp.command;
        if (switch_command < 0)
        {
          switch_command = -switch_command;
        }

        if (low_spd_ctrl_mode == WITHOUT_CURR_CTRL)
        {
          if ((switch_speed >= HYSTERESIS_HIGH_SPEED) &&
              (switch_command >= HYSTERESIS_HIGH_SPEED))
          {
            if (low_spd_switch_count < LOW_SPEED_TO_CURRENT_COUNT)
            {
              low_spd_switch_count++;
            }
            if (low_spd_switch_count >= LOW_SPEED_TO_CURRENT_COUNT)
            {
              low_spd_ctrl_mode = WITH_CURR_CTRL;
              low_spd_switch_count = 0U;
            }
          }
          else
          {
            low_spd_switch_count = 0U;
          }
        }
        else
        {
          if (switch_speed <= HYSTERESIS_LOW_SPEED)
          {
            if (low_spd_switch_count < LOW_SPEED_TO_VOLTAGE_COUNT)
            {
              low_spd_switch_count++;
            }
            if (low_spd_switch_count >= LOW_SPEED_TO_VOLTAGE_COUNT)
            {
              low_spd_ctrl_mode = WITHOUT_CURR_CTRL;
              low_spd_switch_count = 0U;
            }
          }
          else
          {
            low_spd_switch_count = 0U;
          }
        }

        spd_err = speed_ramp.command - rotor_speed_val;
        if (low_spd_ctrl_mode == WITHOUT_CURR_CTRL)
        {
          low_speed_vq_target = pid_controller(&pid_spd_volt, spd_err);

          /*
           * Low-speed voltage control has no inner current loop.  Remove Vq
           * when the filtered dq-current magnitude reaches the configured
           * startup current, then release at 75% to avoid chatter.
           */
          low_speed_current_sq =
            (int64_t)Ival.d * Ival.d + (int64_t)Ival.q * Ival.q;
          low_speed_release_current = ((int32_t)STARTUP_CURRENT_PU * 3) / 4;
          if (low_speed_current_sq >=
              ((int64_t)STARTUP_CURRENT_PU * STARTUP_CURRENT_PU))
          {
            low_speed_current_limit_active = SET;
          }
          else if (low_speed_current_sq <=
                   ((int64_t)low_speed_release_current *
                    low_speed_release_current))
          {
            low_speed_current_limit_active = RESET;
          }

          if (low_speed_current_limit_active != RESET)
          {
            low_speed_vq_applied = 0;
          }
          else
          {
            low_speed_vq_delta = low_speed_vq_target - low_speed_vq_applied;
            if (low_speed_vq_delta > LOW_SPEED_VQ_RAMP_STEP)
            {
              low_speed_vq_applied += LOW_SPEED_VQ_RAMP_STEP;
            }
            else if (low_speed_vq_delta < -LOW_SPEED_VQ_RAMP_STEP)
            {
              low_speed_vq_applied -= LOW_SPEED_VQ_RAMP_STEP;
            }
            else
            {
              low_speed_vq_applied = low_speed_vq_target;
            }
          }

          volt_cmd.Vqd.q = (int16_t)low_speed_vq_applied;
          pid_prepare_output(&pid_spd_volt, spd_err, volt_cmd.Vqd.q);

          /*
           * Track the torque-producing current that is actually moving the
           * motor.  At the 400-rpm handoff the speed error is nearly zero, so
           * a proportional-only seed would also be zero and lose all torque.
           */
          current.Iqdref.q = Ival.q;
          if (current.Iqdref.q > STARTUP_CURRENT_PU)
          {
            current.Iqdref.q = STARTUP_CURRENT_PU;
          }
          else if (current.Iqdref.q < -STARTUP_CURRENT_PU)
          {
            current.Iqdref.q = -STARTUP_CURRENT_PU;
          }
          pid_prepare_output(&pid_spd, spd_err, current.Iqdref.q);
          volt_cmd.Vqd.d = (switch_speed > SLICK_SPEED_RPM) ? VD_VOLT_CMD_LOW_SPD : 0;
        }
        else
        {
          low_speed_vq_applied = volt_cmd.Vqd.q;
          low_speed_current_limit_active = RESET;
          current.Iqdref.q = pid_controller(&pid_spd, spd_err);

          /*
           * Keep the voltage speed PI aligned with the current-loop voltage
           * for a smooth transition back into low-speed voltage control.
           */
          pid_prepare_output(&pid_spd_volt, spd_err, volt_cmd.Vqd.q);
        }
#else
        spd_err = speed_ramp.command - rotor_speed_val;
        current.Iqdref.q = pid_controller(&pid_spd, spd_err);
#endif

#ifdef FIELD_WEAKENING
        current.Iqdref.d = pid_controller(&(field_weakening.pid_fw), field_weakening.fw_err_filt);
#endif

#ifdef CURRENT_DECOUPLE_CTRL
        /* current decoupling control */
        curr_decoupling_ctrl(&current_decoupling, current.Iqd);
#endif
      }

#ifdef CURRENT_DECOUPLE_CTRL
      else if(ctrl_mode == TORQUE_CTRL)
      {
        /* current decoupling control */
        curr_decoupling_ctrl(&current_decoupling, current.Iqd);
      }
#endif
    }
  }
}

/**
  * @brief  this function handles systick handler.
  * @param  none
  * @retval none
  */
void SysTick_Handler(void)
{
  static flag_status monitoring_adc_ready = RESET;
  static uint16_t undervoltage_count = 0U;

  rotor_speed_val = rotor_speed_hall.filtered >> rotor_speed_hall.shift;
  rotor_speed_val_filt = moving_average_shift(speed_ma_fliter, rotor_speed_val);

  if(ctrl_source == CTRL_SOURCE_EXTERNAL)
  {
    external_input_handler();
  }

  ESC_State_Task(esc_state_old);

  /*
   * SysTick starts before PWM begins triggering the ordinary ADC.  Do not
   * treat the zero-initialized DMA buffer as a real 0 V / hot NTC sample.
   * A full DMA transfer proves that every monitoring channel is valid.
   */
  if ((monitoring_adc_ready == RESET) &&
      (dma_flag_get(ADC_ORDINARY_DMA_FT_STS_FLAG) != RESET))
  {
    dma_flag_clear(ADC_ORDINARY_DMA_FT_STS_FLAG);
    monitoring_adc_ready = SET;
  }

  if (monitoring_adc_ready != RESET)
  {
    /* NTC (10k@25C, B=3950K) lookup, returns Celsius x 100 */
    iMosTemperature = mos_temp_ntc_x100_from_adc(adc_in_tab[ADC_MOS_TEMP_IDX]);
    ui_wave_param.iMosTemperature_meas = (int16_t)(iMosTemperature);
    ui_wave_param.iBusVoltage_meas = (int16_t)(adc_in_tab[ADC_BUS_VOLT_IDX]*vref_cal_ratio>>14);
  }
  ui_wave_param.speed_meas_filter_pu = (int16_t)((rotor_speed_val_filt * RPM_TO_SPEED_PU)>>15);
  ui_wave_param.speed_reference_pu = (int16_t)((speed_ramp.command * RPM_TO_SPEED_PU)>>15);
  ui_wave_param.position_meas_pu = (int16_t)((int32_t)(angle.val * DEGREE_TO_POS_PU)>>15);
  ui_wave_param.position_reference_pu = (int16_t)((int32_t)(angle.command * DEGREE_TO_POS_PU)>>15);

  if (monitoring_adc_ready != RESET)
  {
    /* Over/under voltage protection */
    if (ui_wave_param.iBusVoltage_meas < UNDERVOLTAGE_THRESHOLD_d)
    {
      if (undervoltage_count < UNDER_VOLT_DEBOUNCE_MS)
      {
        undervoltage_count++;
      }
      if (undervoltage_count >= UNDER_VOLT_DEBOUNCE_MS)
      {
        error_code |= error_code_mask & MC_UNDER_VOLT_ERROR;
      }
    }
    else
    {
      undervoltage_count = 0U;
      if (ui_wave_param.iBusVoltage_meas > OVERVOLTAGE_THRESHOLD_d)
      {
        error_code |= error_code_mask & MC_OVER_VOLT_ERROR;
      }
    }

    /* MOS Temperature protection (NTC path, compare in Celsius x 100) */
    if (iMosTemperature > (OVER_TEMP_THRESHOLD * 100))
    {
      error_code |= error_code_mask & MC_OVER_TEMP_ERROR;
    }
  }

  /* Enter error state handler */
  if (error_code != MC_NO_ERROR)
  {
    esc_state = ESC_STATE_ERROR;
  }

  /* step Iq current test */
#if 0

  if(ctrl_mode == TORQUE_CTRL)
  {
    if (Iq_cmd_count >= 0)
    {
      current.Iqdref.q = Iq_cmd;
    }
    else
    {
      current.Iqdref.q = -Iq_cmd;
    }

    Iq_cmd_count++;
  }

#endif
}

/**
  * @brief  button interrupt handler
  * @param  none
  * @retval none
  */
void BUTTON_EXINT_IRQ(void)
{
  if (exint_flag_get(BUTTON_EXINT_LINE) != RESET)
  {
    /* delay 30ms for debouncing */
    mc_delay_ms(30);

    /* clear interrupt pending bit */
    exint_flag_clear(BUTTON_EXINT_LINE);

    if(gpio_input_data_bit_read(USER_BUTTON_PORT, USER_BUTTON_PIN) == SET)
    {
      if(start_stop_btn_flag != RESET)
      {
        start_stop_btn_flag = RESET;
      }
      else
      {
        start_stop_btn_flag = SET;
      }
    }
  }
  if (exint_flag_get(HALL_LEARN_BUTTON_EXINT_LINE) != RESET)
  {
    /* delay 30ms for debouncing */
    mc_delay_ms(30);

    /* clear interrupt pending bit */
    exint_flag_clear(HALL_LEARN_BUTTON_EXINT_LINE);

    if(gpio_input_data_bit_read(HALL_LEARN_BUTTON_PORT, HALL_LEARN_BUTTON_PIN) == SET)
    {
      if(hall_learn.start_flag != RESET)
      {
        hall_learn.start_flag = RESET;
      }
      else
      {
        hall_learn.start_flag = SET;
      }
    }
  }
}

