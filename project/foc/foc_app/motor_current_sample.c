#include "at32f45x.h"
#include "motor_current_sample.h"
#include "motor_control_config.h"

static volatile bool motor_current_offsets_ready;
static volatile uint16_t motor_current_phase_a_offset_raw;
static volatile uint16_t motor_current_phase_b_offset_raw;
static volatile motor_current_sample_state_t motor_current_state;

/**
 * @brief 计算32位有符号电流的绝对值。
 * @param value 输入电流，单位mA。
 * @return 输入值的无符号绝对值。
 */
static uint32_t motor_current_sample_abs(int32_t value)
{
  return (value < 0) ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
}

void motor_current_sample_init(void)
{
  motor_current_offsets_ready = false;
  motor_current_phase_a_offset_raw = 0U;
  motor_current_phase_b_offset_raw = 0U;
  motor_current_state.phase_a_ma = 0L;
  motor_current_state.phase_b_ma = 0L;
  motor_current_state.phase_c_ma = 0L;
  motor_current_state.phase_a_q15 = 0;
  motor_current_state.phase_b_q15 = 0;
  motor_current_state.phase_c_q15 = 0;
  motor_current_state.sample_count = 0U;
  motor_current_state.overcurrent_count = 0U;
  motor_current_state.overcurrent_fault = false;
}

bool motor_current_sample_offsets_valid(
  uint16_t phase_a_offset_raw,
  uint16_t phase_b_offset_raw)
{
  uint32_t offset_min = MOTOR_CURRENT_OFFSET_NOMINAL_COUNTS -
                        MOTOR_CURRENT_OFFSET_TOLERANCE_COUNTS;
  uint32_t offset_max = MOTOR_CURRENT_OFFSET_NOMINAL_COUNTS +
                        MOTOR_CURRENT_OFFSET_TOLERANCE_COUNTS;
  uint32_t offset_difference;

  if (((uint32_t)phase_a_offset_raw < offset_min) ||
      ((uint32_t)phase_a_offset_raw > offset_max) ||
      ((uint32_t)phase_b_offset_raw < offset_min) ||
      ((uint32_t)phase_b_offset_raw > offset_max))
  {
    return false;
  }

  if (phase_a_offset_raw >= phase_b_offset_raw)
  {
    offset_difference = phase_a_offset_raw - phase_b_offset_raw;
  }
  else
  {
    offset_difference = phase_b_offset_raw - phase_a_offset_raw;
  }

  return offset_difference <= MOTOR_CURRENT_OFFSET_DIFFERENCE_MAX;
}

bool motor_current_sample_offsets_set(
  uint16_t phase_a_offset_raw,
  uint16_t phase_b_offset_raw)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  if (!motor_current_sample_offsets_valid(phase_a_offset_raw,
                                           phase_b_offset_raw))
  {
    motor_current_offsets_ready = false;
    __set_PRIMASK(primask);
    return false;
  }

  motor_current_phase_a_offset_raw = phase_a_offset_raw;
  motor_current_phase_b_offset_raw = phase_b_offset_raw;
  motor_current_state.sample_count = 0U;
  motor_current_state.overcurrent_count = 0U;
  motor_current_state.overcurrent_fault = false;
  motor_current_offsets_ready = true;
  __set_PRIMASK(primask);

  return true;
}

int32_t motor_current_sample_raw_to_ma(
  uint16_t adc_raw,
  uint16_t offset_raw)
{
  int32_t adc_delta = (int32_t)adc_raw - (int32_t)offset_raw;
  int64_t current_ma;
  int64_t denominator =
    (int64_t)MOTOR_ADC_FULL_SCALE_COUNTS *
    (int64_t)MOTOR_CURRENT_SHUNT_UOHM *
    (int64_t)MOTOR_CURRENT_CSA_GAIN;

  current_ma = (int64_t)adc_delta *
               (int64_t)MOTOR_ADC_REFERENCE_UV * 1000LL /
               denominator;

#if MOTOR_CURRENT_ADC_POLARITY_POSITIVE == 0U
  current_ma = -current_ma;
#endif

  return (int32_t)current_ma;
}

int16_t motor_current_sample_ma_to_q15(int32_t current_ma)
{
  int64_t current_q15;

  current_q15 = ((int64_t)current_ma * 32768LL) /
                MOTOR_CURRENT_Q15_BASE_MA;

  if (current_q15 > 32767LL)
  {
    current_q15 = 32767LL;
  }
  else if (current_q15 < -32768LL)
  {
    current_q15 = -32768LL;
  }

  return (int16_t)current_q15;
}

bool motor_current_sample_process(const motor_adc_fast_sample_t *sample)
{
  uint32_t max_absolute_current;

  if ((sample == 0) || (!motor_current_offsets_ready))
  {
    return false;
  }

  motor_current_state.phase_a_ma = motor_current_sample_raw_to_ma(
    sample->phase_a_raw, motor_current_phase_a_offset_raw);
  motor_current_state.phase_b_ma = motor_current_sample_raw_to_ma(
    sample->phase_b_raw, motor_current_phase_b_offset_raw);
  motor_current_state.phase_c_ma =
    -(motor_current_state.phase_a_ma + motor_current_state.phase_b_ma);
  motor_current_state.phase_a_q15 =
    motor_current_sample_ma_to_q15(motor_current_state.phase_a_ma);
  motor_current_state.phase_b_q15 =
    motor_current_sample_ma_to_q15(motor_current_state.phase_b_ma);
  motor_current_state.phase_c_q15 =
    motor_current_sample_ma_to_q15(motor_current_state.phase_c_ma);
  motor_current_state.sample_count++;

  max_absolute_current = motor_current_sample_abs(
    motor_current_state.phase_a_ma);
  if (motor_current_sample_abs(motor_current_state.phase_b_ma) >
      max_absolute_current)
  {
    max_absolute_current = motor_current_sample_abs(
      motor_current_state.phase_b_ma);
  }
  if (motor_current_sample_abs(motor_current_state.phase_c_ma) >
      max_absolute_current)
  {
    max_absolute_current = motor_current_sample_abs(
      motor_current_state.phase_c_ma);
  }

  if (max_absolute_current >= (uint32_t)MOTOR_SOFTWARE_OVERCURRENT_MA)
  {
    if (motor_current_state.overcurrent_count <
        MOTOR_SOFTWARE_OVERCURRENT_SAMPLES)
    {
      motor_current_state.overcurrent_count++;
    }
  }
  else
  {
    motor_current_state.overcurrent_count = 0U;
  }

  if ((!motor_current_state.overcurrent_fault) &&
      (motor_current_state.overcurrent_count >=
       MOTOR_SOFTWARE_OVERCURRENT_SAMPLES))
  {
    motor_current_state.overcurrent_fault = true;
    return true;
  }

  return false;
}

bool motor_current_sample_state_read(motor_current_sample_state_t *state)
{
  uint32_t primask;

  if (state == 0)
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  state->phase_a_ma = motor_current_state.phase_a_ma;
  state->phase_b_ma = motor_current_state.phase_b_ma;
  state->phase_c_ma = motor_current_state.phase_c_ma;
  state->phase_a_q15 = motor_current_state.phase_a_q15;
  state->phase_b_q15 = motor_current_state.phase_b_q15;
  state->phase_c_q15 = motor_current_state.phase_c_q15;
  state->sample_count = motor_current_state.sample_count;
  state->overcurrent_count = motor_current_state.overcurrent_count;
  state->overcurrent_fault = motor_current_state.overcurrent_fault;
  __set_PRIMASK(primask);

  return motor_current_offsets_ready && (state->sample_count != 0U);
}
