#include <stddef.h>
#include "motor_current_pi.h"

/**
 * @brief 将有符号数限制在对称电压范围内。
 * @param value 待限制值。
 * @param limit 正的绝对限幅。
 * @return 限幅后的数值。
 */
static int32_t motor_current_pi_limit(int32_t value, int32_t limit)
{
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

void motor_current_pi_axis_init(motor_current_pi_axis_t *controller,
                                int32_t proportional_gain_q15,
                                int32_t integral_gain_q15,
                                int32_t output_limit_mv)
{
  if ((controller == NULL) || (output_limit_mv <= 0)) return;
  controller->proportional_gain_q15 = proportional_gain_q15;
  controller->integral_gain_q15 = integral_gain_q15;
  controller->output_limit_mv = output_limit_mv;
  controller->integral_mv = 0;
  controller->output_mv = 0;
}

void motor_current_pi_axis_reset(motor_current_pi_axis_t *controller)
{
  if (controller == NULL) return;
  controller->integral_mv = 0;
  controller->output_mv = 0;
}

bool motor_current_pi_axis_output_seed(motor_current_pi_axis_t *controller,
                                       int32_t output_mv)
{
  if ((controller == NULL) || (controller->output_limit_mv <= 0))
  {
    return false;
  }
  controller->integral_mv = motor_current_pi_limit(
    output_mv, controller->output_limit_mv);
  controller->output_mv = controller->integral_mv;
  return true;
}

void motor_current_pi_axis_output_track(motor_current_pi_axis_t *controller,
                                        int32_t applied_output_mv)
{
  if ((controller == NULL) || (controller->output_limit_mv <= 0))
  {
    return;
  }
  applied_output_mv = motor_current_pi_limit(
    applied_output_mv, controller->output_limit_mv);
  controller->integral_mv = motor_current_pi_limit(
    controller->integral_mv + applied_output_mv - controller->output_mv,
    controller->output_limit_mv);
  controller->output_mv = applied_output_mv;
}

int32_t motor_current_pi_axis_process(motor_current_pi_axis_t *controller,
                                      int32_t reference_ma,
                                      int32_t feedback_ma)
{
  int32_t error_ma;
  int32_t proportional_mv;
  int32_t integral_step_mv;
  int32_t integral_candidate_mv;
  int32_t output_candidate_mv;

  if ((controller == NULL) || (controller->output_limit_mv <= 0)) return 0;
  error_ma = reference_ma - feedback_ma;
  proportional_mv = (int32_t)(((int64_t)error_ma *
                               controller->proportional_gain_q15) >> 15);
  integral_step_mv = (int32_t)(((int64_t)error_ma *
                                controller->integral_gain_q15) >> 15);
  integral_candidate_mv = motor_current_pi_limit(
    controller->integral_mv + integral_step_mv, controller->output_limit_mv);
  output_candidate_mv = proportional_mv + integral_candidate_mv;

  if (((output_candidate_mv <= controller->output_limit_mv) &&
       (output_candidate_mv >= -controller->output_limit_mv)) ||
      ((output_candidate_mv > controller->output_limit_mv) && (error_ma < 0)) ||
      ((output_candidate_mv < -controller->output_limit_mv) && (error_ma > 0)))
  {
    controller->integral_mv = integral_candidate_mv;
  }

  controller->output_mv = motor_current_pi_limit(
    proportional_mv + controller->integral_mv, controller->output_limit_mv);
  return controller->output_mv;
}
