#include "at32f45x.h"
#include "motor_current_calibration.h"
#include "motor_timebase.h"

typedef enum
{
  MOTOR_CURRENT_CALIBRATION_IDLE = 0,
  MOTOR_CURRENT_CALIBRATION_RUNNING,
  MOTOR_CURRENT_CALIBRATION_COMPLETE
} motor_current_calibration_state_t;

static volatile motor_current_calibration_state_t motor_current_calibration_state =
  MOTOR_CURRENT_CALIBRATION_IDLE;
static volatile uint32_t motor_current_calibration_target_count;
static volatile uint32_t motor_current_calibration_sample_count;
static volatile uint32_t motor_current_calibration_phase_a_sum;
static volatile uint32_t motor_current_calibration_phase_b_sum;
static volatile motor_current_calibration_result_t motor_current_calibration_result;

bool motor_current_calibration_run(
  uint32_t sample_count,
  uint32_t timeout_ms,
  motor_current_calibration_result_t *result)
{
  uint32_t primask;
  uint32_t start_cycles;
  uint32_t timeout_cycles;

  if ((sample_count == 0U) ||
      (sample_count > MOTOR_CURRENT_CALIBRATION_MAX_SAMPLES) ||
      (timeout_ms == 0U) ||
      (timeout_ms > MOTOR_CURRENT_CALIBRATION_MAX_TIMEOUT_MS) ||
      (result == 0))
  {
    return false;
  }

  timeout_cycles = (system_core_clock / 1000U) * timeout_ms;

  primask = __get_PRIMASK();
  __disable_irq();
  if (motor_current_calibration_state == MOTOR_CURRENT_CALIBRATION_RUNNING)
  {
    __set_PRIMASK(primask);
    return false;
  }

  motor_current_calibration_target_count = sample_count;
  motor_current_calibration_sample_count = 0U;
  motor_current_calibration_phase_a_sum = 0U;
  motor_current_calibration_phase_b_sum = 0U;
  motor_current_calibration_result.sample_count = 0U;
  motor_current_calibration_state = MOTOR_CURRENT_CALIBRATION_RUNNING;
  __set_PRIMASK(primask);

  start_cycles = motor_timebase_cycles_get();
  while (motor_current_calibration_state == MOTOR_CURRENT_CALIBRATION_RUNNING)
  {
    if (motor_timebase_cycles_elapsed(start_cycles) >= timeout_cycles)
    {
      primask = __get_PRIMASK();
      __disable_irq();
      motor_current_calibration_state = MOTOR_CURRENT_CALIBRATION_IDLE;
      __set_PRIMASK(primask);
      return false;
    }
  }

  primask = __get_PRIMASK();
  __disable_irq();
  result->phase_a_offset_raw = motor_current_calibration_result.phase_a_offset_raw;
  result->phase_b_offset_raw = motor_current_calibration_result.phase_b_offset_raw;
  result->sample_count = motor_current_calibration_result.sample_count;
  motor_current_calibration_state = MOTOR_CURRENT_CALIBRATION_IDLE;
  __set_PRIMASK(primask);

  return true;
}

void motor_current_calibration_sample_process(
  const motor_adc_fast_sample_t *sample)
{
  if ((sample == 0) ||
      (motor_current_calibration_state != MOTOR_CURRENT_CALIBRATION_RUNNING))
  {
    return;
  }

  motor_current_calibration_phase_a_sum += sample->phase_a_raw;
  motor_current_calibration_phase_b_sum += sample->phase_b_raw;
  motor_current_calibration_sample_count++;

  if (motor_current_calibration_sample_count >= motor_current_calibration_target_count)
  {
    motor_current_calibration_result.phase_a_offset_raw = (uint16_t)
      (motor_current_calibration_phase_a_sum / motor_current_calibration_target_count);
    motor_current_calibration_result.phase_b_offset_raw = (uint16_t)
      (motor_current_calibration_phase_b_sum / motor_current_calibration_target_count);
    motor_current_calibration_result.sample_count = motor_current_calibration_sample_count;
    motor_current_calibration_state = MOTOR_CURRENT_CALIBRATION_COMPLETE;
  }
}
