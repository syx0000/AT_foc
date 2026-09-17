/**
  **************************************************************************
  * @file     mc_pid_controller.c
  * @brief    pid control related functions.
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

/** @defgroup mc_pid_controller
  * @brief pid control related functions.
  * @{
  */

/**
  * @brief  set Kp value
  * @param  pid_handler: pid controller related variables
  * @param  kp_gain: proportional gain
  * @retval none
  */
void pid_set_kp(pid_ctrl_type *pid_handler, int16_t kp_gain)
{
  pid_handler->kp_gain = kp_gain;
}

/**
  * @brief  set Ki value
  * @param  pid_handler: pid controller related variables
  * @param  ki_gain: integral gain
  * @retval none
  */
void pid_set_ki(pid_ctrl_type *pid_handler, int16_t ki_gain)
{
  pid_handler->ki_gain = ki_gain;
}

/**
  * @brief  set Kd value
  * @param  pid_handler: pid controller related variables
  * @param  kd_gain: derivative gain
  * @retval none
  */
void pid_set_kd(pid_ctrl_type *pid_handler, int16_t kd_gain)
{
  pid_handler->kd_gain = kd_gain;
}

/**
  * @brief  set integral value
  * @param  pid_handler: pid controller related variables
  * @param  integral_value: integral value
  * @retval none
  */
void pid_set_integral(pid_ctrl_type *pid_handler, int32_t integral_value)
{
  pid_handler->integral = integral_value;
}

/**
  * @brief  get Kp value
  * @param  pid_handler: pid controller related variables
  * @retval proportional gain
  */
int16_t pid_get_kp(pid_ctrl_type *pid_handler)
{
  return ( pid_handler->kp_gain );
}

/**
  * @brief  get Ki value
  * @param  pid_handler: pid controller related variables
  * @retval integral gain
  */
int16_t pid_get_ki(pid_ctrl_type *pid_handler)
{
  return ( pid_handler->ki_gain );
}

/**
  * @brief  get Kd value
  * @param  pid_handler: pid controller related variables
  * @retval derivative gain
  */
int16_t pid_get_kd(pid_ctrl_type *pid_handler)
{
  return ( pid_handler->kd_gain );
}

/**
  * @brief  set auto-tune current pid parameters
  * @param  i_tune_handler: current pid auto-tune related variables
  * @param  pid_handler: pid controller related variables
  * @retval none
  */
void set_current_pid_param(i_auto_tune_type *i_tune_handler, pid_ctrl_type *pid_handler)
{
  pid_handler->kp_gain = i_tune_handler->kp;
  pid_handler->ki_gain = i_tune_handler->ki;
  pid_handler->kp_shift = i_tune_handler->kp_shift;
  pid_handler->ki_shift = i_tune_handler->ki_shift;

  pid_handler->upper_limit_integral = pid_handler->upper_limit_output << pid_handler->ki_shift;
  pid_handler->lower_limit_integral = pid_handler->lower_limit_output << pid_handler->ki_shift;
}

/**
  * @brief  convert step command to slope command
  * @param  cmd_ramp_handler: command and slope related variables
  * @retval none
  */
void command_ramp(ramp_cmd_type *cmd_ramp_handler)
{
  int32_t cmd_ramp_err;

  cmd_ramp_err = cmd_ramp_handler->cmd_final - cmd_ramp_handler->command;

  if (cmd_ramp_err >= cmd_ramp_handler->acc_slope)
  {
    cmd_ramp_handler->command += cmd_ramp_handler->acc_slope;

    if (cmd_ramp_handler->command >= cmd_ramp_handler->cmd_final)
    {
      cmd_ramp_handler->command = cmd_ramp_handler->cmd_final;
    }
  }
  else if (cmd_ramp_err < -cmd_ramp_handler->dec_slope)
  {
    cmd_ramp_handler->command -= cmd_ramp_handler->dec_slope;

    if (cmd_ramp_handler->command <= cmd_ramp_handler->cmd_final)
    {
      cmd_ramp_handler->command = cmd_ramp_handler->cmd_final;
    }
  }
  else
  {
    cmd_ramp_handler->command = cmd_ramp_handler->cmd_final;
  }
}

/**
  * @brief  pid controller with static clamp
  * @param  pid_handler: pid controller related variables
  * @param  var_err: error
  * @retval none
  */
int16_t pid_controller_static_clamp(pid_ctrl_type *pid_handler, int32_t var_err)
{
  int32_t output_temp, proportion_output, derivative_output;

  pid_handler->error = var_err;

  pid_handler->integral = pid_handler->integral + pid_handler->ki_gain * var_err;     /*Q12 * Q15 = Q27*/

  if (pid_handler->integral > pid_handler->upper_limit_integral)
  {
    pid_handler->integral = pid_handler->upper_limit_integral;
  }
  else if (pid_handler->integral < pid_handler->lower_limit_integral)
  {
    pid_handler->integral = pid_handler->lower_limit_integral;
  }

  proportion_output = pid_handler->kp_gain * var_err;
  derivative_output = pid_handler->kd_gain * (var_err - pid_handler->pre_error);
  output_temp = (proportion_output >> pid_handler->kp_shift) + (pid_handler->integral >> pid_handler->ki_shift) + (derivative_output >> pid_handler ->kd_shift);

  pid_handler->out_start = output_temp;

  if (output_temp > pid_handler->upper_limit_output)
  {
    output_temp = pid_handler->upper_limit_output;
  }
  else if (output_temp < pid_handler->lower_limit_output)
  {
    output_temp = pid_handler->lower_limit_output;
  }

  pid_handler->pre_error = var_err;

  return ( output_temp );
}

/**
  * @brief  pid controller with dynamic clamp
  * @param  pid_handler: pid controller related variables
  * @param  var_err: error
  * @retval none
  */
int16_t pid_controller_dyna_clamp(pid_ctrl_type *pid_handler, int32_t var_err)
{
  int32_t  output_temp, pre_output_temp;
  int32_t proportion_output, derivative_output;

  pid_handler->error = var_err;

  proportion_output = pid_handler->kp_gain * var_err;
  derivative_output = pid_handler->kd_gain * (var_err - pid_handler->pre_error);
  pid_handler->pre_error = var_err;

  pre_output_temp = (proportion_output >> pid_handler->kp_shift) + (pid_handler->integral >> pid_handler->ki_shift) + (derivative_output >> pid_handler ->kd_shift);

  output_temp = pre_output_temp;

  pid_handler->out_start = output_temp;

  if (output_temp > pid_handler->upper_limit_output)
  {
    output_temp = pid_handler->upper_limit_output;
  }
  else if (output_temp < pid_handler->lower_limit_output)
  {
    output_temp = pid_handler->lower_limit_output;
  }

  /* dynamic Integrator clamp */
  pid_handler->integral += pid_handler->ki_gain * var_err + ((output_temp - pre_output_temp) << pid_handler->ki_shift);     /*Q12 * Q15 = Q27*/

  return ( output_temp );
}

#if defined SIX_STEP_CONTROL
/**
  * @brief  Reads current loop PID parameters from flash memory
  * @param  pid_handler Pointer to PID controller structure
  * @retval none
  */
void readCurrentPidFromFlash(pid_ctrl_type *pid_handler)
{
  I_auto_tune.Vdc_rated = *(intCoeffs32 + MC_PROTOCOL_REG_I_TUNE_VDC_RATE);

  pid_handler->kp_gain = *(intCoeffs32 + MC_PROTOCOL_REG_TORQUE_KP);
  pid_handler->ki_gain = *(intCoeffs32 + MC_PROTOCOL_REG_TORQUE_KI);
  pid_handler->kp_shift = *(intCoeffs32 + MC_PROTOCOL_REG_TORQUE_KP_DIV);
  pid_handler->ki_shift = *(intCoeffs32 + MC_PROTOCOL_REG_TORQUE_KI_DIV);

  pid_handler->upper_limit_integral = (INT16_MAX  << pid_handler->ki_shift);
  pid_handler->lower_limit_integral = 0;
  pid_handler->upper_limit_output = INT16_MAX;
  pid_handler->lower_limit_output = 0;
}
/**
  * @brief  Reads speed-voltage loop PID parameters from flash memory
  * @param  pid_handler Pointer to PID controller structure
  * @retval none
  */
void readSpeedVoltPidFromFlash(pid_ctrl_type *pid_handler)
{
  I_auto_tune.Vdc_rated = *(intCoeffs32 + MC_PROTOCOL_REG_I_TUNE_VDC_RATE);

  pid_handler->kp_gain = *(intCoeffs32 + MC_PROTOCOL_REG_SPEED_VOLT_KP);
  pid_handler->ki_gain = *(intCoeffs32 + MC_PROTOCOL_REG_SPEED_VOLT_KI);
  pid_handler->kp_shift = *(intCoeffs32 + MC_PROTOCOL_REG_SPEED_VOLT_KP_DIV);
  pid_handler->ki_shift = *(intCoeffs32 + MC_PROTOCOL_REG_SPEED_VOLT_KI_DIV);

  pid_handler->upper_limit_integral = (INT16_MAX  << pid_handler->ki_shift);
  pid_handler->lower_limit_integral = 0;
  pid_handler->upper_limit_output = INT16_MAX;
  pid_handler->lower_limit_output = 0;
}
#endif

#if defined E_BIKE_SCOOTER
/**
  * @brief  Reads D-axis  current loop PID parameters from flash memory
  * @param  pid_handler Pointer to PID controller structure
  * @retval none
  */
void readIdPIDFromFlash(pid_ctrl_type *pid_handler, voltage_type *volt_handler)
{
  pid_handler->kp_gain = (int16_t)*(intCoeffs32_p+MC_PROTOCOL_REG_FLUX_KP);
  pid_handler->ki_gain = (int16_t)*(intCoeffs32_p+MC_PROTOCOL_REG_FLUX_KI);
  pid_handler->kp_shift = (int16_t)*(intCoeffs32_p+MC_PROTOCOL_REG_FLUX_KP_DIV);
  pid_handler->ki_shift = (int16_t)*(intCoeffs32_p+MC_PROTOCOL_REG_FLUX_KI_DIV);

  pid_handler->upper_limit_integral = (int32_t)(volt_handler->Vd_max << pid_handler->ki_shift);
  pid_handler->lower_limit_integral = (int32_t)(-(volt_handler->Vd_max << pid_handler->ki_shift));
}
/**
  * @brief  Reads Q-axis current loop PID parameters from flash memory
  * @param  pid_handler Pointer to PID controller structure
  * @retval none
  */
void readIqPIDFromFlash(pid_ctrl_type *pid_handler)
{
  pid_handler->kp_gain = (int16_t)*(intCoeffs32_p+MC_PROTOCOL_REG_TORQUE_KP);
  pid_handler->ki_gain = (int16_t)*(intCoeffs32_p+MC_PROTOCOL_REG_TORQUE_KI);
  pid_handler->kp_shift = (int16_t)*(intCoeffs32_p+MC_PROTOCOL_REG_TORQUE_KP_DIV);
  pid_handler->ki_shift = (int16_t)*(intCoeffs32_p+MC_PROTOCOL_REG_TORQUE_KI_DIV);
}

void adjustIdqPIDGain(pid_ctrl_type *pid_Id_handler, pid_ctrl_type *pid_Iq_handler, voltage_type *volt_handler)
{
  readIdPIDFromFlash(pid_Id_handler, volt_handler);
  readIqPIDFromFlash(pid_Iq_handler);
  pid_Id_handler->kp_gain = adjustValueByVdc(pid_Id_handler->kp_gain,vdc_ratio);
  pid_Id_handler->ki_gain = adjustValueByVdc(pid_Id_handler->ki_gain,vdc_ratio);
  pid_Iq_handler->kp_gain = adjustValueByVdc(pid_Iq_handler->kp_gain,vdc_ratio);
  pid_Iq_handler->ki_gain = adjustValueByVdc(pid_Iq_handler->ki_gain,vdc_ratio);
}
#endif
