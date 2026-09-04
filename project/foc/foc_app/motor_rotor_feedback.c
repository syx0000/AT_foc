/**
 * @file    motor_rotor_feedback.c
 * @brief   通用转子反馈接口 功能实现
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#include <stddef.h>
#include "motor_hall_angle_estimator.h"
#include "motor_hall_decoder.h"
#include "motor_parameter.h"
#include "motor_rotor_feedback.h"

void motor_rotor_feedback_init(void)
{
  /* 当前仅提供Hall适配器，无独立运行状态需要初始化。 */
}

bool motor_rotor_feedback_read(motor_rotor_feedback_t *feedback)
{
  motor_hall_angle_estimator_t estimator;
  motor_hall_sample_t hall;
  uint8_t pole_pairs;
  uint32_t mechanical_speed_millirpm;

  if ((feedback == NULL) ||
      (!motor_hall_angle_estimator_read(&estimator)) ||
      (!motor_hall_decoder_sample_read(&hall)))
  {
    return false;
  }

  pole_pairs = motor_parameter_pole_pairs_get();
  if (pole_pairs == 0U)
  {
    return false;
  }
  mechanical_speed_millirpm = (uint32_t)(
    ((uint64_t)estimator.electrical_frequency_millihz * 60U) / pole_pairs);
  feedback->source = MOTOR_ROTOR_FEEDBACK_SOURCE_HALL;
  feedback->electrical_angle_u16 = estimator.electrical_angle_u16;
  feedback->mechanical_angle_u16 = 0U;
  feedback->mechanical_speed_millirpm = (estimator.direction < 0) ?
    -(int32_t)mechanical_speed_millirpm : (int32_t)mechanical_speed_millirpm;
  feedback->electrical_frequency_millihz =
    estimator.electrical_frequency_millihz;
  feedback->timestamp_cycles = hall.timestamp_cycles;
  feedback->update_count = hall.frequency_update_count;
  feedback->source_state = hall.state;
  feedback->direction = estimator.direction;
  feedback->mechanical_angle_valid = false;
  feedback->valid = estimator.valid && hall.valid &&
                    (estimator.direction != 0) &&
                    (estimator.electrical_frequency_millihz != 0U);
  return true;
}
