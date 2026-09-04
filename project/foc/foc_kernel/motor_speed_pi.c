#include <stddef.h>
#include "motor_speed_pi.h"

/**
 * @brief 将64位有符号数限制在对称电流范围内。
 * @param value 待限幅数值。
 * @param limit 正的绝对电流限幅，单位mA。
 * @return 限幅后的32位电流值，单位mA。
 */
static int32_t motor_speed_pi_limit(int64_t value, int32_t limit)
{
  if (value > limit)
  {
    return limit;
  }
  if (value < -(int64_t)limit)
  {
    return -limit;
  }
  return (int32_t)value;
}

bool motor_speed_pi_init(motor_speed_pi_t *controller,
                         int32_t proportional_gain_q20,
                         int32_t integral_gain_q20,
                         int32_t output_limit_ma)
{
  if ((controller == NULL) || (proportional_gain_q20 < 0) ||
      (integral_gain_q20 < 0) || (output_limit_ma <= 0))
  {
    return false;
  }

  controller->proportional_gain_q20 = proportional_gain_q20;
  controller->integral_gain_q20 = integral_gain_q20;
  controller->output_limit_ma = output_limit_ma;
  controller->integral_ma = 0;
  controller->output_ma = 0;
  return true;
}

void motor_speed_pi_reset(motor_speed_pi_t *controller)
{
  if (controller == NULL)
  {
    return;
  }
  controller->integral_ma = 0;
  controller->output_ma = 0;
}

bool motor_speed_pi_output_seed(motor_speed_pi_t *controller,
                                int32_t output_ma)
{
  if ((controller == NULL) || (controller->output_limit_ma <= 0))
  {
    return false;
  }

  controller->integral_ma = motor_speed_pi_limit(
    output_ma, controller->output_limit_ma);
  controller->output_ma = controller->integral_ma;
  return true;
}

void motor_speed_pi_output_track(motor_speed_pi_t *controller,
                                 int32_t applied_output_ma)
{
  int32_t limited_output_ma;

  if ((controller == NULL) || (controller->output_limit_ma <= 0))
  {
    return;
  }

  limited_output_ma = motor_speed_pi_limit(
    applied_output_ma, controller->output_limit_ma);
  controller->integral_ma = motor_speed_pi_limit(
    (int64_t)controller->integral_ma + limited_output_ma -
    controller->output_ma, controller->output_limit_ma);
  controller->output_ma = limited_output_ma;
}

int32_t motor_speed_pi_process(motor_speed_pi_t *controller,
                               int32_t reference_millirpm,
                               int32_t feedback_millirpm)
{
  int64_t error_millirpm;
  int64_t proportional_ma;
  int64_t integral_step_ma;
  int32_t integral_candidate_ma;
  int64_t output_candidate_ma;

  if ((controller == NULL) || (controller->output_limit_ma <= 0))
  {
    return 0;
  }

  error_millirpm =
    (int64_t)reference_millirpm - feedback_millirpm;
  proportional_ma =
    (error_millirpm * controller->proportional_gain_q20) >> 20;
  integral_step_ma =
    (error_millirpm * controller->integral_gain_q20) >> 20;
  integral_candidate_ma = motor_speed_pi_limit(
    (int64_t)controller->integral_ma + integral_step_ma,
    controller->output_limit_ma);
  output_candidate_ma = proportional_ma + integral_candidate_ma;

  if (((output_candidate_ma <= controller->output_limit_ma) &&
       (output_candidate_ma >= -controller->output_limit_ma)) ||
      ((output_candidate_ma > controller->output_limit_ma) &&
       (error_millirpm < 0)) ||
      ((output_candidate_ma < -controller->output_limit_ma) &&
       (error_millirpm > 0)))
  {
    controller->integral_ma = integral_candidate_ma;
  }

  controller->output_ma = motor_speed_pi_limit(
    proportional_ma + controller->integral_ma,
    controller->output_limit_ma);
  return controller->output_ma;
}
