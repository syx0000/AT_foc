/**
  **************************************************************************
  * @file     mc_curr_decoupling.c
  * @brief    current decoupling control related functions.
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

/** @defgroup mc_curr_decoupling
  * @brief current decoupling control related functions
  * @{
  */

/**
  * @brief  current decoupling control function
  * @param  decouple_handler: current decoupling related variables
  * @param  Iqdref: Id and Iq reference
  * @retval none
  */
void curr_decoupling_ctrl(curr_decoupling_type *decouple_handler, qd_type Iqdref)
{
  int64_t cal_temp1, cal_temp2, cal_temp3;
  qd_type I_filt_val;

  I_filt_val.d = lowpass_filtering(&decouple_handler->Id_LPF, Iqdref.d);
  I_filt_val.q = lowpass_filtering(&decouple_handler->Iq_LPF, Iqdref.q);

  cal_temp1 = ((int64_t)rotor_speed_val * I_filt_val.q);
  cal_temp1 = -(((int64_t)cal_temp1 * decouple_handler->const_1q / (*decouple_handler->Vbus))>>15);

  if (cal_temp1 > VD_MAX)
  {
    decouple_handler->Vff.d = VD_MAX;
  }
  else if (cal_temp1 < -VD_MAX)
  {
    decouple_handler->Vff.d = -VD_MAX;
  }
  else
  {
    decouple_handler->Vff.d = cal_temp1;
  }

  cal_temp1 = ((int64_t)rotor_speed_val * I_filt_val.d);
  cal_temp2 = (((int64_t)cal_temp1 * decouple_handler->const_1d / (*decouple_handler->Vbus))>>15);
  cal_temp3 = (rotor_speed_val * decouple_handler->const_2 / (*decouple_handler->Vbus));
  cal_temp1 = cal_temp2 + cal_temp3;

  if (cal_temp1 > VQ_MAX)
  {
    decouple_handler->Vff.q = VQ_MAX;
  }
  else if (cal_temp1 < -VQ_MAX)
  {
    decouple_handler->Vff.q = -VQ_MAX;
  }
  else
  {
    decouple_handler->Vff.q = cal_temp1;
  }
}

/**
  * @brief  Vector voltage VQ maximum output limitation for current decoupling function
  * @param  volt_handler: voltage vector d-axis and q-axis
  * @param  pid_handler: pid controller output limitation
  * @param  decouple_handler: current decoupling related variables
  * @retval none
  */
void foc_vq_limitation_decoupling(voltage_type *volt_handler, pid_ctrl_type *pid_handler, curr_decoupling_type *decouple_handler)
{
  int32_t cal_temp;
  int16_t cal_temp1,cal_temp2;

  cal_temp = (volt_handler->Vqd_squr_sum_max - (int32_t)volt_handler->Vqd.d * volt_handler->Vqd.d) << 1;
  sqrt_fixed(cal_temp, &cal_temp1);

  if (decouple_handler->Vff.q >= 0)
  {
    cal_temp2 = (int32_t)cal_temp1 - decouple_handler->Vff.q;
  }
  else
  {
    cal_temp2 = (int32_t)cal_temp1 + decouple_handler->Vff.q;
  }
  if (cal_temp2 < 0)
  {
    cal_temp2 = 0;
  }
  pid_handler->upper_limit_output = cal_temp2;
  pid_handler->lower_limit_output = -cal_temp2;
  pid_handler->upper_limit_integral = cal_temp2 << pid_handler->ki_shift;
  pid_handler->lower_limit_integral = -(pid_handler->upper_limit_integral);
}

/**
  * @brief  q-axis and d-axis voltage compensation by current decoupling
  * @param  none
  * @retval Vtemp.q: q-axis current PI controller output
  */
int16_t voltage_comp(void)
{
  qd_type Vtemp;

  Vtemp.d = pid_controller(&pid_id, (Iref.d - Ival.d));
  volt_cmd.Vqd.d = Vtemp.d + current_decoupling.Vff.d;

  /* foc Vq limitation */
  foc_vq_limitation_decoupling(&volt_cmd, &pid_iq, &current_decoupling);

  Vtemp.q = pid_controller(&pid_iq, (Iref.q - Ival.q));
  volt_cmd.Vqd.q = Vtemp.q + current_decoupling.Vff.q;

  return ( Vtemp.q );
}
