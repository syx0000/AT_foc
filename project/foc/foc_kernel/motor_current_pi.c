/**
 * @file    motor_current_pi.c
 * @brief   电流PI定点算法 功能实现
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#include <stddef.h>
#include "motor_current_pi.h"

void motor_current_pi_axis_init(motor_current_pi_axis_t *controller,
                                int32_t proportional_gain_q15,
                                int32_t integral_gain_q15)
{
  if (controller == NULL) return;
  controller->proportional_gain_q15 = proportional_gain_q15;
  controller->integral_gain_q15 = integral_gain_q15;
  controller->integral_q15 = 0;
  controller->integral_mv = 0;
  controller->output_mv = 0;
}

static void motor_current_pi_integral_set(motor_current_pi_axis_t *controller,
                                          int64_t integral_q15)
{
  /* Keep the Q15 accumulator convertible to an int32_t millivolt value. */
  const int64_t minimum = (int64_t)INT32_MIN * 32768;
  const int64_t maximum = (int64_t)INT32_MAX * 32768;
  if (integral_q15 > maximum) integral_q15 = maximum;
  if (integral_q15 < minimum) integral_q15 = minimum;
  controller->integral_q15 = integral_q15;
  controller->integral_mv = (int32_t)(integral_q15 / 32768);
}

bool motor_current_pi_axis_output_seed(motor_current_pi_axis_t *controller,
                                       int32_t output_mv)
{
  if (controller == NULL)
  {
    return false;
  }
  motor_current_pi_integral_set(controller, (int64_t)output_mv * 32768);
  controller->output_mv = controller->integral_mv;
  return true;
}

void motor_current_pi_axis_back_calculate(motor_current_pi_axis_t *controller,
                                         int64_t residual_mv,
                                         uint16_t gain_q15, uint16_t deadband_mv)
{
  if (controller == NULL ||
      (residual_mv >= -(int64_t)deadband_mv && residual_mv <= deadband_mv))
  {
    return;
  }
  motor_current_pi_integral_set(controller,
    controller->integral_q15 + residual_mv * gain_q15);
}

int32_t motor_current_pi_axis_process(motor_current_pi_axis_t *controller,
                                      int32_t reference_ma,
                                      int32_t feedback_ma)
{
  if (controller == NULL) return 0;

  const int32_t error = reference_ma - feedback_ma;
  motor_current_pi_integral_set(controller, controller->integral_q15 +
    (int64_t)error * controller->integral_gain_q15);
  const int64_t request = ((int64_t)error * controller->proportional_gain_q15 +
    controller->integral_q15) / 32768;
  if (request > INT32_MAX) controller->output_mv = INT32_MAX;
  else if (request < INT32_MIN) controller->output_mv = INT32_MIN;
  else controller->output_mv = (int32_t)request;
  return controller->output_mv;
}
