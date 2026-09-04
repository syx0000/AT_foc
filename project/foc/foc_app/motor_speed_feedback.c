/**
 * @file    motor_speed_feedback.c
 * @brief   电机机械速度反馈 功能实现
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#include <stddef.h>
#include "at32f45x.h"
#include "motor_control_config.h"
#include "motor_direction.h"
#include "motor_parameter.h"
#include "motor_rotor_feedback.h"
#include "motor_speed_feedback.h"
#include "motor_timebase.h"

static volatile motor_speed_feedback_t motor_speed_feedback;
static uint32_t motor_speed_feedback_last_update_count;

void motor_speed_feedback_init(void)
{
  motor_speed_feedback.raw_speed_millirpm = 0;
  motor_speed_feedback.filtered_speed_millirpm = 0;
  motor_speed_feedback.electrical_frequency_millihz = 0U;
  motor_speed_feedback.sample_count = 0U;
  motor_speed_feedback.direction = 0;
  motor_speed_feedback.valid = false;
  motor_speed_feedback_last_update_count = 0U;
}

void motor_speed_feedback_process_1khz(void)
{
  motor_rotor_feedback_t rotor_feedback;
  int32_t raw_speed_millirpm;
  int64_t filter_error;
  bool direction_inverted;
  int8_t logical_direction;

  /* 反馈源正在原子更新时保留上一结果，下一次1 kHz周期重新读取。 */
  if (!motor_rotor_feedback_read(&rotor_feedback)) return;

  if ((!rotor_feedback.valid) ||
      (motor_timebase_cycles_to_us(motor_timebase_cycles_elapsed(
        rotor_feedback.timestamp_cycles)) >
       (MOTOR_SPEED_FEEDBACK_TIMEOUT_MS * 1000U)))
  {
    motor_speed_feedback.raw_speed_millirpm = 0;
    motor_speed_feedback.filtered_speed_millirpm = 0;
    motor_speed_feedback.electrical_frequency_millihz = 0U;
    motor_speed_feedback.direction = 0;
    motor_speed_feedback.valid = false;
    return;
  }

  direction_inverted = motor_parameter_direction_inverted_get();
  raw_speed_millirpm = rotor_feedback.mechanical_speed_millirpm;
  (void)motor_direction_transform_s32(raw_speed_millirpm,
                                      direction_inverted,
                                      &raw_speed_millirpm);
  logical_direction = (int8_t)(
    rotor_feedback.direction * motor_direction_sign_get(direction_inverted));

  if ((!motor_speed_feedback.valid) ||
      (motor_speed_feedback.direction != logical_direction))
  {
    motor_speed_feedback.filtered_speed_millirpm = raw_speed_millirpm;
  }
  else if (rotor_feedback.update_count !=
           motor_speed_feedback_last_update_count)
  {
    filter_error = (int64_t)raw_speed_millirpm -
                   motor_speed_feedback.filtered_speed_millirpm;
    motor_speed_feedback.filtered_speed_millirpm +=
      (int32_t)(filter_error / (1L << MOTOR_SPEED_FEEDBACK_FILTER_SHIFT));
  }

  motor_speed_feedback.raw_speed_millirpm = raw_speed_millirpm;
  motor_speed_feedback.electrical_frequency_millihz =
    rotor_feedback.electrical_frequency_millihz;
  motor_speed_feedback.direction = logical_direction;
  if (rotor_feedback.update_count !=
      motor_speed_feedback_last_update_count)
  {
    motor_speed_feedback.sample_count++;
    motor_speed_feedback_last_update_count =
      rotor_feedback.update_count;
  }
  motor_speed_feedback.valid = true;
}

bool motor_speed_feedback_read(motor_speed_feedback_t *feedback)
{
  uint32_t primask;

  if (feedback == NULL)
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *feedback = motor_speed_feedback;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return true;
}
