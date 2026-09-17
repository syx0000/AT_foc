/**
  **************************************************************************
  * @file     mc_foc.c
  * @brief    Field Oriented Control-related functions.
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

/** @defgroup mc_foc
  * @brief Field Oriented Control-related functions
  * @{
  */

/**
  * @brief  Trigonometric function sine/cosine function calculation.
  * @param  angle_handler: Rotor angle, the range [0 32767] is expressed as [0 2*pi]
  * @retval sine/cosine functions calculate values
  */
trig_components_type trig_functions (int16_t angle_handler)
{
  trig_components_type theta_handler;

  theta_handler.sin_value = arm_sin_q15(angle_handler);
  theta_handler.cos_value = arm_cos_q15(angle_handler);

  return (theta_handler);
}

/**
  * @brief  clarke transformation
  * @param  abc_handler: vector a,b and c
  * @param  alphabeta_handler: vector alpha and beta
  * @retval none
  */
void foc_clarke_transform(abc_type *abc_handler, alphabeta_type *alphabeta_handler)
{
  int32_t MulTemp;

  MulTemp = INV_3 * (((int32_t)abc_handler->a << 1) - abc_handler->b - abc_handler->c);
  alphabeta_handler->alpha = (int16_t) (MulTemp >> 15);

  MulTemp = (INV_SQRT_3 * ((int32_t)abc_handler->b - abc_handler->c));
  alphabeta_handler->beta = (int16_t) (MulTemp >> 15);
}

/**
  * @brief  clarke transformation
  * @param  abc_handler: vector a,b and c
  * @param  alphabeta_handler: vector alpha and beta
  * @retval none
  */
void foc_clarke_trans(abc_type *abc_handler, alphabeta_type *alphabeta_handler)
{
  int32_t MulTemp;

  alphabeta_handler->alpha = abc_handler->a;
  MulTemp = (INV_SQRT_3 * (abc_handler->b - abc_handler->c));
  alphabeta_handler->beta = MulTemp >> 15;
}

/**
  * @brief  park transformation
  * @param  curr_handler: current vector alpha and beta
  * @param  theta_handler: Sine/cosine values
  * @retval none
  */
void foc_park_trans(current_type *curr_handler, trig_components_type *theta_handler)
{
  int32_t MulTemp;

  MulTemp = curr_handler->Ialphabeta.alpha * theta_handler->cos_value + curr_handler->Ialphabeta.beta * theta_handler->sin_value;
  curr_handler->Iqd.d = MulTemp >> 15;

  MulTemp = curr_handler->Ialphabeta.beta * theta_handler->cos_value - curr_handler->Ialphabeta.alpha * theta_handler->sin_value;
  curr_handler->Iqd.q = MulTemp >> 15;
}

/**
  * @brief  park transformation
  * @param  curr_handler: current vector alpha and beta
  * @param  theta_handler: Sine/cosine values
  * @retval none
  */
void park_trans(alphabeta_type *alphabeta_handler, qd_type *qd_handler, trig_components_type *theta_handler)
{
  int32_t MulTemp;

  MulTemp = alphabeta_handler->alpha * theta_handler->cos_value + alphabeta_handler->beta * theta_handler->sin_value;
  qd_handler->d = MulTemp >> 15;

  MulTemp = alphabeta_handler->beta * theta_handler->cos_value - alphabeta_handler->alpha * theta_handler->sin_value;
  qd_handler->q = MulTemp >> 15;
}

/**
  * @brief  inverse park transformation
  * @param  volt_handler: voltage vector d-axis and q-axis
  * @param  theta_handler: Sine/cosine values
  * @retval none
  */
void foc_inver_park_trans(voltage_type *volt_handler, trig_components_type *theta_handler)
{
  int32_t MulTemp;

  MulTemp = (volt_handler->Vqd.d * theta_handler->cos_value - volt_handler->Vqd.q * theta_handler->sin_value);
  volt_handler-> Valphabeta.alpha = MulTemp >> 15;
  MulTemp = (volt_handler->Vqd.d * theta_handler->sin_value + volt_handler->Vqd.q * theta_handler->cos_value);
  volt_handler->Valphabeta.beta = MulTemp >> 15;
}

/**
  * @brief  open loop control function
  * @param  volt_handler: voltage vector d-axis and q-axis
  * @param  openloop_handler: open loop voltage d-axis, q-axis, and angle increment
  * @retval none
  */
void foc_open_loop_ctrl(voltage_type *volt_handler, open_loop_type *openloop_handler)
{
  volt_handler->Vqd.q = openloop_handler->volt.q;
  volt_handler->Vqd.d = openloop_handler->volt.d;
  openloop_handler->theta += openloop_handler->inc;

  if (openloop_handler->theta < 0)
  {
    openloop_handler->theta += 0x8000;
  }
}

/**
  * @brief  Vector voltage VQ maximum output limitation
  * @param  volt_handler: voltage vector d-axis and q-axis
  * @param  pid_handler: pid controller output limitation
  * @retval none
  */
void foc_vq_limitation(voltage_type *volt_handler, pid_ctrl_type *pid_handler)
{
  int32_t cal_temp;
  int16_t  cal_temp1;

  cal_temp = (volt_handler->Vqd_squr_sum_max - (int32_t)volt_handler->Vqd.d * volt_handler->Vqd.d) << 1;
  sqrt_fixed(cal_temp, &cal_temp1);

  pid_handler->upper_limit_output = cal_temp1;
  pid_handler->lower_limit_output = -cal_temp1;
  pid_handler->upper_limit_integral = cal_temp1 << pid_handler->ki_shift;
  pid_handler->lower_limit_integral = -(pid_handler->upper_limit_integral);
}

/**
  * @brief  Synthetic vector voltage maximum output limitation
  * @param  volt_handler: voltage vector d-axis, q-axis and Maximum Sum of Squares
  * @retval none
  */
void foc_circle_limitation(voltage_type *volt_handler)
{
  int32_t cal_temp;
  int16_t  cal_temp1;

  volt_handler->Vqd_squr_sum = (int32_t)volt_handler->Vqd.q * volt_handler->Vqd.q + (int32_t)volt_handler->Vqd.d * volt_handler->Vqd.d;

  if (volt_handler->Vqd_squr_sum > volt_handler->Vqd_squr_sum_max)
  {
    cal_temp = (volt_handler->Vqd_squr_sum_max - (int32_t)volt_handler->Vqd.d * volt_handler->Vqd.d) << 1;
    sqrt_fixed(cal_temp, &cal_temp1);

    if (volt_handler->Vqd.q < 0)
    {
      volt_handler->Vqd.q = -cal_temp1;
    }
    else
    {
      volt_handler->Vqd.q = cal_temp1;
    }
  }
}

/**
  * @brief  Vector current IQ maximum output limitation
  * @param  curr_handler: current vector d-axis and q-axis
  * @param  pid_handler: pid controller output limitation
  * @retval none
  */
void foc_iq_limitation(current_type *curr_handler, pid_ctrl_type *pid_handler)
{
  int32_t cal_temp;
  int16_t  cal_temp1;

  cal_temp = (curr_handler->Iqd_squr_sum_max - (int32_t)curr_handler->Iqdref.d * curr_handler->Iqdref.d) << 1;
  sqrt_fixed(cal_temp, &cal_temp1);

  pid_handler->upper_limit_output = cal_temp1;
  pid_handler->lower_limit_output = -cal_temp1;
  pid_handler->upper_limit_integral = cal_temp1 << pid_handler->ki_shift;
  pid_handler->lower_limit_integral = -(pid_handler->upper_limit_integral);
}
