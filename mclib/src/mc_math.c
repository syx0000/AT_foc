/**
  **************************************************************************
  * @file     mc_math.c
  * @brief    Filter related functions.
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

/** @defgroup mc_math
  * @brief Filter related functions.
  * @{
  */

/**
  * @brief  moving average filter initialize function
  * @param  order: order of moving average
  * @retval moving average filter related variables
  */
moving_average_type* moving_average(uint16_t order)
{
  moving_average_type* filter = malloc(sizeof(moving_average_type));
  int32_t* buffer = calloc(order, sizeof(int32_t));

  filter->order = order;
  filter->index = 0;
  filter->sum = 0;
  filter->buffer = buffer;

  return filter;
}

/**
  * @brief  moving average filter function
  * @param  filter: moving average filter related variables
  * @param  data: raw data
  * @retval filtered data
  */
int32_t moving_average_update(moving_average_type* filter, int32_t data)
{
  int32_t avg_value;

  if (filter->index >= (filter->order)) /* reset buffer index, ring buffer mechanism.*/
  {
    filter->index = 0;
    filter->full_flag = 1;
  }

  if (filter->full_flag > 0)
  {
    filter->sum = filter->sum - filter->buffer[filter->index] + data;
#ifdef AT32L021xx
    hwdiv_dividend_set((uint32_t)filter->sum);
    hwdiv_divisor_set((uint32_t)filter->order);
    avg_value = (int32_t)hwdiv_quotient_get();
#else
    avg_value = filter->sum / filter->order;
#endif
  }
  else
  {
    filter->sum = filter->sum + data;
#ifdef AT32L021xx
    hwdiv_dividend_set((uint32_t)filter->sum);
    hwdiv_divisor_set((uint32_t)(filter->index + 1));
    avg_value = (int32_t)hwdiv_quotient_get();
#else
    avg_value = filter->sum / (filter->index + 1);
#endif
  }

  filter->buffer[filter->index++] = data;

  return (avg_value);
}
/**
  * @brief  reset moving average filter function
  * @param  filter: moving average filter related variables
  * @retval none
  */
void reset_ma_buffer(moving_average_type* filter)
{
  filter->full_flag = 0;
  filter->index = 0;
  filter->sum = 0;
}

/**
  * @brief  Fast moving average filter function
  * @param  input_handler: Raw input data to be filtered
  * @param  average_handler: Previous filtered data value
  * @param  PowOf2: Power of two value that determines the filter window size (2^PowOf2)
  *                 Higher values provide stronger filtering but slower response
  *                 Typical range: 2-10 (window size: 4-1024 samples)
  * @retval Next filtered data value
  */
int32_t ma_filter(int32_t input_handler, int32_t average_handler, uint16_t PowOf2)
{
  int64_t cal_temp;

  cal_temp = (int64_t)average_handler << PowOf2;
  cal_temp = cal_temp - average_handler + input_handler;

  average_handler = (int32_t) (cal_temp >> PowOf2);

  return ( average_handler );
}

/**
  * @brief  lowpass filter initialize function
  * @param  lowpass_handler: lowpass filter related variables
  * @retval none
  */
void lowpass_filter_init(lowpass_filter_type *lowpass_handler)
{
  double cal_temp;

  cal_temp = (double) lowpass_handler->bandwidth / lowpass_handler->sample_freq;

  lowpass_handler->coef1 = (int16_t)(0x7FFF / (1 + cal_temp));
  lowpass_handler->coef2 = (int16_t)(0x7FFF * cal_temp / (1 + cal_temp));
  lowpass_handler->residual = 0;
}

/**
  * @brief  lowpass filter function
  * @param  lowpass_handler: lowpass filter related variables
  * @param  input_handler: raw data
  * @retval filtered data
  */
int32_t lowpass_filtering(lowpass_filter_type *lowpass_handler, int32_t input_handler)
{
  int64_t cal_temp;
  cal_temp = (int64_t)lowpass_handler->coef1 * lowpass_handler->output_temp + (int64_t)lowpass_handler->coef2 * input_handler + lowpass_handler->residual;
  lowpass_handler->output_temp = cal_temp >> 15;
  lowpass_handler->residual = cal_temp - (lowpass_handler->output_temp << 15);

  return ( lowpass_handler->output_temp);
}

/**
  * @brief  two to the power of n moving average filter initialize function
  * @param  order: order of moving average for two to the power(2,4,8,16....etc)
  * @retval moving average filter related variables
  */
moving_average_type* moving_average_shift_init(uint16_t order)
{
  moving_average_type* filter = malloc(sizeof(moving_average_type));
  int16_t shift = (int16_t)log2(order);
  int16_t new_order = (int16_t)pow(2, shift);
  int32_t* buffer = calloc(new_order, sizeof(int32_t));

  filter->order = new_order;
  filter->shift = shift;
  filter->index = 0;
  filter->sum = 0;
  filter->buffer = buffer;

  return (filter);
}

/**
  * @brief  two to the power of n moving average filter function
  * @param  filter: moving average filter related variables
  * @param  data: raw data
  * @retval filtered data
  */
int32_t moving_average_shift(moving_average_type* filter, int32_t data)
{
  int32_t avg_value = 0;

  filter->sum -= filter->buffer[filter->index];
  filter->buffer[filter->index] = data;
  filter->sum += data;
  filter->index++;

  if(filter->index >= filter->order)
  {
    filter->index = 0;
  }

  avg_value = filter->sum >> filter->shift;
  return (avg_value);
}
