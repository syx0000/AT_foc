/**
  **************************************************************************
  * @file     mc_curr_fdbk.c
  * @brief    current offset and sensing related functions.
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

/** @defgroup mc_curr_fdbk
  * @brief current offset and sensing related functions.
  * @{
  */

/**
  * @brief  read current offset values
  * @param  curr_handler: voltage vector d-axis and q-axis
  * @param  shunt_nbr: shunt number (ONESHUNT, TWOSHUNT or THREESHUNT)
  * @retval the state of initialize offset current (SET or RESET)
  */
flag_status current_offset_init(current_type* curr_handler, shunt_nbr_type shunt_nbr)
{
  static uint8_t icount = 0;
  static uint16_t temp1 = 0, temp2 = 0, temp3 = 0;

  while (icount < 17)
  {
    while (adc_flag_get(curr_handler->ADCx, ADC_PCCE_FLAG))
    {
      adc_flag_clear(curr_handler->ADCx, ADC_PCCE_FLAG);

      if (icount > 0)
      {
        if (shunt_nbr > 0)
        {
          temp1 += adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_1);
        }

        if (shunt_nbr > 1)
        {
          temp2 += adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_2);
        }

        if (shunt_nbr > 2)
        {
          temp3 += adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_3);
        }
      }

      icount++;
    }
  }

  if (shunt_nbr == 1)
  {
    curr_handler->offset.bus = temp1 >> 4;
  }
  else if(shunt_nbr == 2)
  {
#if defined U_V_SHUNT
    curr_handler->offset.a = temp1 >> 4;
    curr_handler->offset.b = temp2 >> 4;
#elif defined V_W_SHUNT
    curr_handler->offset.b = temp1 >> 4;
    curr_handler->offset.c = temp2 >> 4;
#elif defined U_W_SHUNT
    curr_handler->offset.a = temp1 >> 4;
    curr_handler->offset.c = temp2 >> 4;
#endif
  }
  else if (shunt_nbr == 3)
  {
    curr_handler->offset.a = temp1 >> 4;
    curr_handler->offset.b = temp2 >> 4;
    curr_handler->offset.c = temp3 >> 4;
  }

  return (SET);
}

/**
  * @brief  read three-phase current values
  * @param  curr_handler: three-phase current values and their offset
  * @param  pwm_duty_handler: three-phase pwm duty
  * @retval none
  */
void current_read_foc_3shunt(current_type *curr_handler, pwm_duty_type *pwm_duty_handler)
{
  int32_t mul_temp;
#ifdef HALL_SENSORS
  int16_t swap_temp;
#endif

  curr_handler->Iabc_shunt.a = -((adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_1) - curr_handler->offset.a) << 3);
  curr_handler->Iabc_shunt.b = -((adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_2) - curr_handler->offset.b) << 3);
  curr_handler->Iabc_shunt.c = -((adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_3) - curr_handler->offset.c) << 3);

#ifdef HALL_SENSORS

  if (hall_learn.dir == 1)
  {
    swap_temp = curr_handler->Iabc_shunt.c;
    curr_handler->Iabc_shunt.c = curr_handler->Iabc_shunt.b;
    curr_handler->Iabc_shunt.b = swap_temp;
  }

#endif

  mul_temp = ((int32_t)curr_handler->span * curr_handler->Iabc_shunt.a);
  curr_handler->Iabc_shunt.a = (mul_temp >> (15 - curr_handler->span_shift));

  mul_temp = ((int32_t)curr_handler->span * curr_handler->Iabc_shunt.b);
  curr_handler->Iabc_shunt.b = (mul_temp >> (15 - curr_handler->span_shift));

  mul_temp = ((int32_t)curr_handler->span * curr_handler->Iabc_shunt.c);
  curr_handler->Iabc_shunt.c = (mul_temp >> (15 - curr_handler->span_shift));

  if (pwm_duty_handler->UF.a > pwm_duty_handler->threshold)
  {
    curr_handler->Iabc.b = curr_handler->Iabc_shunt.b;
    curr_handler->Iabc.c = curr_handler->Iabc_shunt.c;
    curr_handler->Iabc.a = -(curr_handler->Iabc.b + curr_handler->Iabc.c);
  }
  else if (pwm_duty_handler->UF.b > pwm_duty_handler->threshold)
  {
    curr_handler->Iabc.a = curr_handler->Iabc_shunt.a;
    curr_handler->Iabc.c = curr_handler->Iabc_shunt.c;
    curr_handler->Iabc.b = -(curr_handler->Iabc.a + curr_handler->Iabc.c);
  }
  else if (pwm_duty_handler->UF.c > pwm_duty_handler->threshold)
  {
    curr_handler->Iabc.a = curr_handler->Iabc_shunt.a;
    curr_handler->Iabc.b = curr_handler->Iabc_shunt.b;
    curr_handler->Iabc.c = -(curr_handler->Iabc.a + curr_handler->Iabc.b);
  }
  else
  {
    curr_handler->Iabc.a = curr_handler->Iabc_shunt.a;
    curr_handler->Iabc.b = curr_handler->Iabc_shunt.b;
    curr_handler->Iabc.c = -(curr_handler->Iabc.a + curr_handler->Iabc.b);
  }
}

/**
  * @brief  read three-phase current values
  * @param  curr_handler: two-phase current values and their offset
  * @retval none
  */
void current_read_foc_2shunt(current_type *curr_handler)
{
  int32_t mul_temp;
#ifdef HALL_SENSORS
  int16_t swap_temp;
#endif

#if defined U_V_SHUNT
  curr_handler->Iabc.a = -((adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_1) - curr_handler->offset.a) << 3);
  curr_handler->Iabc.b = -((adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_2) - curr_handler->offset.b) << 3);
  curr_handler->Iabc.c = -(curr_handler->Iabc.a + curr_handler->Iabc.b);
#elif defined V_W_SHUNT
  curr_handler->Iabc.b = -((adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_1) - curr_handler->offset.b) << 3);
  curr_handler->Iabc.c = -((adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_2) - curr_handler->offset.c) << 3);
  curr_handler->Iabc.a = -(curr_handler->Iabc.b + curr_handler->Iabc.c);
#elif defined U_W_SHUNT
  curr_handler->Iabc.a = -((adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_1) - curr_handler->offset.a) << 3);
  curr_handler->Iabc.c = -((adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_2) - curr_handler->offset.c) << 3);
  curr_handler->Iabc.b = -(curr_handler->Iabc.a + curr_handler->Iabc.c);
#endif

#ifdef HALL_SENSORS

  if (hall_learn.dir == 1)
  {
    swap_temp = curr_handler->Iabc.c;
    curr_handler->Iabc.c = curr_handler->Iabc.b;
    curr_handler->Iabc.b = swap_temp;
  }

#endif

  mul_temp = ((int32_t)curr_handler->span * curr_handler->Iabc.a);
  curr_handler->Iabc.a = (mul_temp >> (15 - curr_handler->span_shift));

  mul_temp = ((int32_t)curr_handler->span * curr_handler->Iabc.b);
  curr_handler->Iabc.b = (mul_temp >> (15 - curr_handler->span_shift));

  mul_temp = ((int32_t)curr_handler->span * curr_handler->Iabc.c);
  curr_handler->Iabc.c = (mul_temp >> (15 - curr_handler->span_shift));
}

/**
  * @brief  read bus current value
  * @param  curr_handler: bus current value and its offset
  * @retval none
  */
#if defined SIX_STEP_CONTROL
void current_read_bldc(current_type *curr_handler)
{
  int32_t  mul_result;

  if (ADC_TIMER->c4dt > (I_SAMP_MIN_CNT + I_SAMP_DLY_CNT))
  {
    curr_handler->Ibus.Icalc = ((adc_preempt_conversion_data_get(curr_handler->ADCx, CURR_BUS_ADC_PREEMPT_CH) - curr_handler->offset.bus) << 3);
  }
  else
  {
    mul_result = curr_handler->Ibus.Icalc * curr_handler->Ibus.decay_const;
    curr_handler->Ibus.Icalc = mul_result >> 15;
  }

  mul_result = ((int32_t)curr_handler->span * curr_handler->Ibus.Icalc);
  curr_handler->Ibus.Ireal_pu = (mul_result >> (15 - curr_handler->span_shift)) * curr_handler->volt_sign;
}
#endif

/**
  * @brief  read dc current offset value
  * @param  adc_value: adc value of dc current offset
  * @retval average of dc current offset values
  */
int16_t Idc_offset_init(__IO uint16_t *adc_value)
{
  static uint8_t icount = 0;
  static uint16_t sum = 0;

  while (icount < 16)
  {
    sum += *adc_value;
    mc_delay_ms(1);
    icount++;
  }

  return (sum >> 4);
}

/**
  * @brief  read dc current value
  * @param  curr_handler: dc current value and its offset
  * @param  adc_value: adc value of dc current
  * @retval none
  */
int16_t dc_current_read(current_type *curr_handler, __IO uint16_t *adc_value)
{
  int16_t temp_i_dc;
  int32_t  mul_result;

  temp_i_dc = (*adc_value - curr_handler->Idc.offset) << 3;

  mul_result = ((int32_t)curr_handler->dc_span * temp_i_dc);

  return (mul_result >> (15 - curr_handler->span_shift));
}

/**
  * @brief  read 1-shunt current values for motor parameter identification
  * @param  curr_handler: bus current value and its offset
  * @retval none
  */
void current_read_1shunt_ID(current_type *curr_handler)
{
  int32_t mul_temp;

  curr_handler->ID_1shunt = (adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_1) - curr_handler->offset.bus) << 3;
  mul_temp = ((int32_t)curr_handler->span * curr_handler->ID_1shunt);
  curr_handler->ID_1shunt = (mul_temp >> (15 - curr_handler->span_shift));
}

#if defined FOC_CONTROL
/**
  * @brief  read current offset values for two adc converters
  * @param  curr_handler: voltage vector d-axis and q-axis
  * @param  shunt_nbr: shunt number (ONESHUNT, TWOSHUNT or THREESHUNT)
  * @retval the state of initialize offset current (SET or RESET)
  */
flag_status current_offset_init_2adc(current_type* curr_handler, shunt_nbr_type shunt_nbr)
{
  static uint8_t icount = 0;
  static uint16_t temp1 = 0, temp2 = 0, temp3 = 0;

  while (icount < 17)
  {
    while (adc_flag_get(curr_handler->ADCx, ADC_PCCE_FLAG))
    {
      adc_flag_clear(curr_handler->ADCx, ADC_PCCE_FLAG);

      if (icount > 0)
      {
        if (shunt_nbr > 0)
        {
          temp1 += adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_1);
        }

        if (shunt_nbr > 1)
        {
          temp2 += adc_preempt_conversion_data_get(curr_handler->ADC2x, ADC_PREEMPT_CHANNEL_1);
        }

        if (shunt_nbr > 2)
        {
          temp3 += adc_preempt_conversion_data_get(curr_handler->ADC2x, ADC_PREEMPT_CHANNEL_2);
        }
      }

      icount++;
    }
  }

  if(shunt_nbr == 2)
  {
#if defined U_V_SHUNT
    curr_handler->offset.a = temp1 >> 4;
    curr_handler->offset.b = temp2 >> 4;
#elif defined V_W_SHUNT
    curr_handler->offset.b = temp1 >> 4;
    curr_handler->offset.c = temp2 >> 4;
#elif defined U_W_SHUNT
    curr_handler->offset.a = temp1 >> 4;
    curr_handler->offset.c = temp2 >> 4;
#endif
  }
  else if (shunt_nbr == 3)
  {
    curr_handler->offset.a = temp1 >> 4;
    curr_handler->offset.b = temp2 >> 4;
    curr_handler->offset.c = temp3 >> 4;

    adc_preempt_channel_length_set(curr_handler->ADC2x, 1);
    adc_preempt_channel_set(curr_handler->ADC2x, CURR_PHASE_B_ADC_CH, 1, curr_handler->adc_sampletime);
  }

  return (SET);
}

/**
  * @brief  read two phase current values by two adc converters for three phase current sensing
  * @param  curr_handler: two phase current values and their offset
  * @retval none
  */
void current_read_foc_2shunt_2adc(current_type *curr_handler)
{
  int32_t mul_temp;
#ifdef HALL_SENSORS
  int16_t swap_temp;
#endif

#if defined U_V_SHUNT
  curr_handler->Iabc.a = -((adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_1) - curr_handler->offset.a) << 3);
  curr_handler->Iabc.b = -((adc_preempt_conversion_data_get(curr_handler->ADC2x, ADC_PREEMPT_CHANNEL_1) - curr_handler->offset.b) << 3);
  curr_handler->Iabc.c = -(curr_handler->Iabc.a + curr_handler->Iabc.b);
#elif defined V_W_SHUNT
  curr_handler->Iabc.b = -((adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_1) - curr_handler->offset.b) << 3);
  curr_handler->Iabc.c = -((adc_preempt_conversion_data_get(curr_handler->ADC2x, ADC_PREEMPT_CHANNEL_1) - curr_handler->offset.c) << 3);
  curr_handler->Iabc.a = -(curr_handler->Iabc.b + curr_handler->Iabc.c);
#elif defined U_W_SHUNT
  curr_handler->Iabc.a = -((adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_1) - curr_handler->offset.a) << 3);
  curr_handler->Iabc.c = -((adc_preempt_conversion_data_get(curr_handler->ADC2x, ADC_PREEMPT_CHANNEL_1) - curr_handler->offset.c) << 3);
  curr_handler->Iabc.b = -(curr_handler->Iabc.a + curr_handler->Iabc.c);
#endif

#ifdef HALL_SENSORS

  if (hall_learn.dir == 1)
  {
    swap_temp = curr_handler->Iabc.c;
    curr_handler->Iabc.c = curr_handler->Iabc.b;
    curr_handler->Iabc.b = swap_temp;
  }

#endif

  mul_temp = ((int32_t)curr_handler->span * curr_handler->Iabc.a);
  curr_handler->Iabc.a = (mul_temp >> (15 - curr_handler->span_shift));

  mul_temp = ((int32_t)curr_handler->span * curr_handler->Iabc.b);
  curr_handler->Iabc.b = (mul_temp >> (15 - curr_handler->span_shift));

  mul_temp = ((int32_t)curr_handler->span * curr_handler->Iabc.c);
  curr_handler->Iabc.c = (mul_temp >> (15 - curr_handler->span_shift));
}

/**
  * @brief  3-shunt current sampling function (for motor initial angle detection)
  * @param  angle_detect_handler: pointer to the parameters of the structure foc_angle_init_type
  * @param  curr_handler: pointer to the parameters of the structure current_type
  * @retval none
  */
void current_angle_init_3shunt(foc_angle_init_type *angle_detect_handler, current_type *curr_handler)
{
  int32_t mul_temp;

  curr_handler->Iabc_shunt.a = -((adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_1) - curr_handler->offset.a) << 3);
  curr_handler->Iabc_shunt.b = -((adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_2) - curr_handler->offset.b) << 3);
  curr_handler->Iabc_shunt.c = -((adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_3) - curr_handler->offset.c) << 3);

  mul_temp = ((int32_t)curr_handler->span * curr_handler->Iabc_shunt.a);
  curr_handler->Iabc.a = (mul_temp >> (15 - curr_handler->span_shift));

  mul_temp = ((int32_t)curr_handler->span * curr_handler->Iabc_shunt.b);
  curr_handler->Iabc.b = (mul_temp >> (15 - curr_handler->span_shift));

  mul_temp = ((int32_t)curr_handler->span * curr_handler->Iabc_shunt.c);
  curr_handler->Iabc.c = (mul_temp >> (15 - curr_handler->span_shift));

  switch (angle_detect_handler->step_count)
  {
  case STEP_1_AH_PWM_BCL:
    break;

  case STEP_2_BCH_PWM_AL:
    angle_detect_handler->Iap = -(curr_handler->Iabc.b * 2);
    break;

  case STEP_3_BH_PWM_ACL:
    angle_detect_handler->Ian = -curr_handler->Iabc.a;
    break;

  case STEP_4_ACH_PWM_BL:
    angle_detect_handler->Ibp = -(curr_handler->Iabc.a * 2);
    break;

  case STEP_5_CH_PWM_ABL:
    angle_detect_handler->Ibn = -curr_handler->Iabc.b;
    break;

  case STEP_6_ABH_PWM_CL:
    angle_detect_handler->Icp = -(curr_handler->Iabc.a * 2);
    break;

  case STEP_7_ANGLE_EST:
    angle_detect_handler->Icn = -curr_handler->Iabc.c;
    break;

  default:
    break;
  }
}

/**
  * @brief  Bus current sampling function (for motor initial angle detection)
  * @param  angle_detect_handler: pointer to the parameters of the structure foc_angle_init_type
  * @param  curr_handler: pointer to the parameters of the structure current_type
  * @retval none
  */
void current_angle_init_2_1shunt(foc_angle_init_type *angle_detect_handler, current_type *curr_handler)
{
  switch (angle_detect_handler->step_count)
  {
  case STEP_1_AH_PWM_BCL:
    break;

  case STEP_2_BCH_PWM_AL:
    angle_detect_handler->Iap = adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_1);
    break;

  case STEP_3_BH_PWM_ACL:
    angle_detect_handler->Ian = adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_1);
    break;

  case STEP_4_ACH_PWM_BL:
    angle_detect_handler->Ibp = adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_1);
    break;

  case STEP_5_CH_PWM_ABL:
    angle_detect_handler->Ibn = adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_1);
    break;

  case STEP_6_ABH_PWM_CL:
    angle_detect_handler->Icp = adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_1);
    break;

  case STEP_7_ANGLE_EST:
    angle_detect_handler->Icn = adc_preempt_conversion_data_get(curr_handler->ADCx, ADC_PREEMPT_CHANNEL_1);
    break;

  default:
    break;
  }
}
#endif
