/**
  **************************************************************************
  * @file     motor_control_foc.h
  * @brief    Declaration of motor control related funciton for Field Oriented Control(FOC)
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

#ifndef __MOTOR_CONTROL_FOC_H
#define __MOTOR_CONTROL_FOC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mc_lib.h"
firmware_id_type get_fw_id(void);
void remap_exint_clock_enable_config(void);
void param_clear(void);
void param_init(void);
void charge_boot_cap(pwm_duty_type *pwm_duty_handler);
void enable_pwm_timer_channel_buffer(pwm_duty_type *pwm_duty_handler);
void enable_pwm_timer(pwm_duty_type *pwm_duty_handler);
void disable_pwm_timer(pwm_duty_type *pwm_duty_handler);
void pwm_switch_off(void);
void pwm_switch_on(void);
flag_status I_offset_init(current_type *curr_handler);
void I_tune_manual(void);
void hall_cw_ctrl_para(void);
void hall_ccw_ctrl_para(void);
void curr_cmd_handler_ebike(pid_ctrl_type *pid_handler, ramp_cmd_type *cmd_ramp_handler, int16_t Iqref_handler);
void dc_current_limit(void);
void curr_cmd_handler_speed_limit(void);
void position_cmd_ramp(position_type *pos_handler, ramp_cmd_type *cmd_ramp_handler, pid_ctrl_type *pid_handler);
void position_control_handler(void);
void speed_pid_param(void);
void monitoring_signal_adc_trigger(void);
void svpwm_func(voltage_type *volt_handler, pwm_duty_type *pwm_duty_handler);
void pwm_duty_update(pwm_duty_type *pwm_duty_handler);
void pwm_duty_extra_update(pwm_duty_type *pwm_duty_handler);
void asym_pwm_duty_update(pwm_duty_type *pwm_duty_handler);
void pwm_duty_buffer(pwm_duty_type *pwm_duty_handler);
void set_normal_pwm_mode(void);
void lock_rotor(void);
void motor_parameter_id_process(void);
void current_auto_tuning(i_auto_tune_type *i_tune_handler);
void ipmsm_mtpa_control(void);
uint16_t calcVdcRatio(int16_t vref_ratio);
int16_t adjustValueByVdc(int16_t value, int16_t Vdc_ratio);
void brake_input_handler(void);
void reverse_input_handler(void);
void external_input_handler(void);
void anti_theft_input_handler(void);
void parking_lock_input_handler(void);
void normal_state_led_blink(uint16_t *led_blink_count,gpio_type *led_gpio_port,uint16_t led_gpio_pin);

#ifdef __cplusplus
}
#endif

#endif
