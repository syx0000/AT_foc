/**
  **************************************************************************
  * @file     motor_control_bldc.h
  * @brief    Declaration of motor control related funciton for BLDC
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

#ifndef __MOTOR_CONTROL_BLDC_H
#define __MOTOR_CONTROL_BLDC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mc_lib.h"

firmware_id_type get_fw_id(void);
void remap_exint_clock_enable_config(void);
void param_clear(void);
void param_init(void);
void hall_to_tmr_register_setting(void);
void hall_learn_register_setting(void);
void disable_mosfet(tmr_type *tmr_x);
void I_offset_init(void);
void current_offset_tmr_setting(tmr_type *tmr_x);
void I_tune_manual(void);
void start_up_param_init(void);
void motor_cw_init(void);
void motor_ccw_init(void);
void hall_cw_ctrl_para(pid_ctrl_type *pid_handler,int16_t max_value);
void hall_ccw_ctrl_para(pid_ctrl_type *pid_handler,int16_t min_value);
uint16_t calcVdcRatio(int16_t vref_ratio);
int16_t adjustValueByVdc(int16_t value,int16_t Vdc_ratio);
void pwm_duty_update(void);
void set_adc_sample_point(adc_sample_type *adc_sample);
void calc_motor_speed(speed_type *rotor_speed);
void open_loop_cmd_ramp(olc_type *openloop_handler);
void enable_pwm_timer(void);
void disable_pwm_timer(void);
void motor_parameter_id_process(void);
void current_auto_tuning(i_auto_tune_type *i_tune_handler);
void normal_state_led_blink(uint16_t *led_blink_count,gpio_type *led_gpio_port,uint16_t led_gpio_pin);
int16_t calcValueByVref(int16_t input_value);
void external_input_handler(void);
#ifdef PWM_INPUT
void calibration_process(void);
void spd_curve_calculation(int16_t icvalue);
int32_t getSpeedFromPulseWidth(int32_t pulse_width,int16_t base_speed,int16_t slope);
#endif

#ifdef __cplusplus
}
#endif

#endif

