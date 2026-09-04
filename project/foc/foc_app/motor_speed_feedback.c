#include <stddef.h>
#include "at32f45x.h"
#include "motor_control_config.h"
#include "motor_direction.h"
#include "motor_hall_decoder.h"
#include "motor_parameter.h"
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
  motor_hall_sample_t hall_sample;
  uint32_t timeout_cycles;
  uint32_t speed_millirpm;
  int32_t raw_speed_millirpm;
  int64_t filter_error;
  bool direction_inverted;
  uint8_t pole_pairs;
  int8_t logical_direction;

  timeout_cycles =
    (system_core_clock / 1000U) * MOTOR_SPEED_FEEDBACK_TIMEOUT_MS;
  if ((!motor_hall_decoder_sample_read(&hall_sample)) ||
      (!hall_sample.valid) ||
      (hall_sample.direction == 0) ||
      (hall_sample.electrical_frequency_millihz == 0U) ||
      (motor_timebase_cycles_elapsed(hall_sample.timestamp_cycles) >
       timeout_cycles))
  {
    motor_speed_feedback.raw_speed_millirpm = 0;
    motor_speed_feedback.filtered_speed_millirpm = 0;
    motor_speed_feedback.electrical_frequency_millihz = 0U;
    motor_speed_feedback.direction = 0;
    motor_speed_feedback.valid = false;
    return;
  }

  direction_inverted = motor_parameter_direction_inverted_get();
  pole_pairs = motor_parameter_pole_pairs_get();
  speed_millirpm =
    (uint32_t)(((uint64_t)hall_sample.electrical_frequency_millihz * 60U) /
               pole_pairs);
  raw_speed_millirpm = (hall_sample.direction > 0) ?
    (int32_t)speed_millirpm : -(int32_t)speed_millirpm;
  (void)motor_direction_transform_s32(raw_speed_millirpm,
                                      direction_inverted,
                                      &raw_speed_millirpm);
  logical_direction = (int8_t)(
    hall_sample.direction * motor_direction_sign_get(direction_inverted));

  if ((!motor_speed_feedback.valid) ||
      (motor_speed_feedback.direction != logical_direction))
  {
    motor_speed_feedback.filtered_speed_millirpm = raw_speed_millirpm;
  }
  else if (hall_sample.frequency_update_count !=
           motor_speed_feedback_last_update_count)
  {
    filter_error = (int64_t)raw_speed_millirpm -
                   motor_speed_feedback.filtered_speed_millirpm;
    motor_speed_feedback.filtered_speed_millirpm +=
      (int32_t)(filter_error / (1L << MOTOR_SPEED_FEEDBACK_FILTER_SHIFT));
  }

  motor_speed_feedback.raw_speed_millirpm = raw_speed_millirpm;
  motor_speed_feedback.electrical_frequency_millihz =
    hall_sample.electrical_frequency_millihz;
  motor_speed_feedback.direction = logical_direction;
  if (hall_sample.frequency_update_count !=
      motor_speed_feedback_last_update_count)
  {
    motor_speed_feedback.sample_count++;
    motor_speed_feedback_last_update_count =
      hall_sample.frequency_update_count;
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
