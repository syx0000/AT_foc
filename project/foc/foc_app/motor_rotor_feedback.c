/**
 * @file    motor_rotor_feedback.c
 * @brief   通用转子反馈接口 功能实现
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#include <stddef.h>
#include <motor_control_config.h>
#include "at32f45x.h"
#include "motor_board_config.h"
#if (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_HALL)
#include "motor_hall_angle_estimator.h"
#include "motor_hall_angle_math.h"
#include "motor_hall_decoder.h"
#include "motor_hall_port.h"
#elif (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_ENCODER)
#include "encoder_rs485.h"
#endif
#include "motor_parameter.h"
#include "motor_rotor_feedback.h"
#include "motor_timebase.h"

static volatile motor_rotor_feedback_t motor_rotor_feedback_override;
static volatile bool motor_rotor_feedback_override_enabled;

#if (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_ENCODER)
/* Speed-loop feedback published once per control window. */
static volatile struct {
  uint32_t last_cycles;
  int32_t speed_millirpm;
  uint32_t elec_freq_millihz;
  int8_t direction;
  uint32_t update_count;
  bool valid;
} enc_speed;

static volatile struct {
  int64_t velocity_sum;
  uint32_t samples;
  uint32_t last_cycles;
} enc_speed_window;
static uint16_t encoder_mechanical_zero_u16;
static struct {
  uint32_t previous_position;
  int64_t electrical_counts_per_second;
  int32_t mechanical_counts_per_second;
  bool velocity_valid;
  uint16_t sample_advance_u16;
  uint16_t voltage_advance_u16;
  bool previous_valid;
} enc_fast_phase;
#endif

/* 连续单圈机械角累加器：SysTick 1 kHz更新，read()原子读取快照。
 * Hall分辨率决定机械角只精确到60°电角度/极对数，本累加只做长期跟踪，不作精测。 */
static volatile struct
{
  int32_t residue;              /**< 电角度→机械角整除余数，保留亚计数精度。 */
  uint16_t last_electrical_u16; /**< 上一次采样到的转子电角度。 */
  uint16_t mechanical_angle_u16;/**< 累计出的单圈机械角，U16一周制。 */
  bool tracking;                /**< 已建立追踪起点则为true。 */
} motor_rotor_feedback_tracker;

static void motor_rotor_feedback_tracker_reset(void)
{
  motor_rotor_feedback_tracker.residue = 0;
  motor_rotor_feedback_tracker.last_electrical_u16 = 0U;
  motor_rotor_feedback_tracker.mechanical_angle_u16 = 0U;
  motor_rotor_feedback_tracker.tracking = false;
}

void motor_rotor_feedback_init(void)
{
  motor_rotor_feedback_override_enabled = false;
  motor_rotor_feedback_tracker_reset();
#if (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_ENCODER)
  encoder_mechanical_zero_u16 = 0U;
  enc_fast_phase.previous_valid = false;
  enc_fast_phase.electrical_counts_per_second = 0;
  enc_fast_phase.velocity_valid = false;
  enc_fast_phase.sample_advance_u16 = 0U;
  enc_fast_phase.voltage_advance_u16 = 0U;
  enc_speed_window.velocity_sum = 0;
  enc_speed_window.samples = 0U;
  enc_speed_window.last_cycles = 0U;
  enc_speed.last_cycles     = 0U;
  enc_speed.speed_millirpm  = 0;
  enc_speed.elec_freq_millihz = 0U;
  enc_speed.direction       = 0;
  enc_speed.update_count    = 0U;
  enc_speed.valid           = false;
#endif
}

void motor_rotor_feedback_override_set(uint16_t electrical_angle_u16,
                                       int32_t electrical_frequency_millihz,
                                       int8_t direction)
{
  uint32_t primask;
  uint32_t magnitude = (electrical_frequency_millihz < 0) ?
    (uint32_t)(-electrical_frequency_millihz) :
    (uint32_t)electrical_frequency_millihz;
  uint8_t pole_pairs = motor_parameter_pole_pairs_get();
  uint32_t mechanical_speed_millirpm = (pole_pairs == 0U) ? 0U :
    (uint32_t)(((uint64_t)magnitude * 60U) / pole_pairs);
  primask = __get_PRIMASK();
  __disable_irq();
  motor_rotor_feedback_override.source =
    MOTOR_ROTOR_FEEDBACK_SOURCE_CURRENT_STARTUP;
  motor_rotor_feedback_override.electrical_angle_u16 = electrical_angle_u16;
  motor_rotor_feedback_override.mechanical_angle_u16 = 0U;
  motor_rotor_feedback_override.mechanical_speed_millirpm =
    (direction < 0) ? -(int32_t)mechanical_speed_millirpm :
                      (int32_t)mechanical_speed_millirpm;
  motor_rotor_feedback_override.electrical_frequency_millihz = magnitude;
  motor_rotor_feedback_override.timestamp_cycles = motor_timebase_cycles_get();
  motor_rotor_feedback_override.update_count++;
  motor_rotor_feedback_override.source_state = 0U;
  motor_rotor_feedback_override.direction = direction;
  motor_rotor_feedback_override.mechanical_angle_valid = false;
  motor_rotor_feedback_override.valid = true;
  motor_rotor_feedback_override.speed_valid = electrical_frequency_millihz != 0U;
  motor_rotor_feedback_override_enabled = true;
  __set_PRIMASK(primask);
}

void motor_rotor_feedback_override_clear(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  motor_rotor_feedback_override_enabled = false;
  __set_PRIMASK(primask);
}

bool motor_rotor_feedback_encoder_zero_here(void)
{
#if (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_ENCODER)
  enc_data_t enc;
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  (void)encoder_rs485_read(&enc);
  const bool valid = enc.valid &&
    motor_timebase_cycles_elapsed(enc.timestamp_cycles) <=
      2U * system_core_clock / MOTOR_PWM_FREQUENCY_HZ;

  if (valid)
  {
    encoder_mechanical_zero_u16 = encoder_abs_to_angle_q16(enc.abs_pos);
  }
  __set_PRIMASK(primask);
  return valid;
#else
  return false;
#endif
}

#if (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_ENCODER)
static int32_t encoder_position_delta(uint32_t position, uint32_t previous)
{
  int32_t delta = (int32_t)position - (int32_t)previous;
  if (delta > (int32_t)(ENC_SINGLE_MAX / 2U))
    delta -= (int32_t)ENC_SINGLE_MAX;
  else if (delta < -(int32_t)(ENC_SINGLE_MAX / 2U))
    delta += (int32_t)ENC_SINGLE_MAX;
  return delta;
}
#endif

void motor_rotor_feedback_process_fast(void)
{
#if (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_ENCODER)
  enc_data_t enc;

  (void)encoder_rs485_read_control(&enc);
  enc_fast_phase.electrical_counts_per_second = 0;
  enc_fast_phase.velocity_valid = false;
  enc_fast_phase.sample_advance_u16 = 0U;
  enc_fast_phase.voltage_advance_u16 = 0U;
  if (enc.valid && enc_fast_phase.previous_valid)
  {
    /* Consecutive ADC cycles are one PWM period apart. At 17 bits,
       two mechanical counts correspond to one U16 angle unit. */
    const int32_t delta = encoder_position_delta(enc.abs_pos, enc_fast_phase.previous_position);
    const int32_t counts_per_second = delta * (int32_t)MOTOR_PWM_FREQUENCY_HZ;
    enc_fast_phase.mechanical_counts_per_second = counts_per_second;
    enc_fast_phase.velocity_valid = true;
    enc_speed_window.velocity_sum += counts_per_second;
    enc_speed_window.last_cycles = enc.timestamp_cycles;
    enc_speed_window.samples++;
    const int64_t electrical_counts_per_second =
      (int64_t)counts_per_second * motor_parameter_pole_pairs_get();
    enc_fast_phase.electrical_counts_per_second = electrical_counts_per_second;
    /* Delays are in quarter-microseconds to preserve the PWM midpoint. */
    const uint32_t sample_delay = 4U * MOTOR_ENCODER_SAMPLE_LEAD_US;
    const uint32_t voltage_delay = sample_delay + 6000000U / MOTOR_PWM_FREQUENCY_HZ;

    enc_fast_phase.sample_advance_u16 =
      (uint16_t)(electrical_counts_per_second * sample_delay / 8000000LL);
    enc_fast_phase.voltage_advance_u16 =
      (uint16_t)(electrical_counts_per_second * voltage_delay / 8000000LL);
  }
  enc_fast_phase.previous_position = enc.abs_pos;
  enc_fast_phase.previous_valid = enc.valid;
#endif
}

bool motor_rotor_feedback_encoder_fast_speed_read(int32_t *speed_millirpm)
{
#if (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_ENCODER)
  if (speed_millirpm != NULL && enc_fast_phase.velocity_valid)
  {
    *speed_millirpm = (int32_t)((int64_t)enc_fast_phase.mechanical_counts_per_second *
                               60000 / ENC_SINGLE_MAX);
    return true;
  }
#else
  (void)speed_millirpm;
#endif
  return false;
}

int64_t motor_rotor_feedback_foc_omega_mrad_s(void)
{
#if (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_ENCODER)
  return enc_fast_phase.electrical_counts_per_second * 6283 / ENC_SINGLE_MAX;
#else
  return 0;
#endif
}

uint16_t motor_rotor_feedback_foc_angle(const motor_rotor_feedback_t *feedback,
                                        bool voltage_apply)
{
#if (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_ENCODER)
  if (feedback->source == MOTOR_ROTOR_FEEDBACK_SOURCE_ABSOLUTE_ENCODER)
  {
    const uint16_t advance = voltage_apply ? enc_fast_phase.voltage_advance_u16 :
                                             enc_fast_phase.sample_advance_u16;
    return (uint16_t)(feedback->electrical_angle_u16 + advance);
  }
#else
  (void)voltage_apply;
#endif
  return feedback->electrical_angle_u16;
}

bool motor_rotor_feedback_read(motor_rotor_feedback_t *feedback)
{
  uint32_t primask;

  if (feedback == NULL) return false;
  primask = __get_PRIMASK();
  __disable_irq();
  if (motor_rotor_feedback_override_enabled)
  {
    *feedback = motor_rotor_feedback_override;
    __set_PRIMASK(primask);
    return true;
  }
  __set_PRIMASK(primask);

#if (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_ENCODER)
  {
    enc_data_t enc;
    motor_parameter_t parameter;

    if (!motor_parameter_active_read(&parameter) || parameter.pole_pairs == 0U)
      return false;
    const uint8_t pole_pairs = parameter.pole_pairs;
    (void)encoder_rs485_read_control(&enc);
    if (!enc.valid)
    {
      feedback->valid      = false;
      feedback->speed_valid = false;
      feedback->mechanical_angle_valid = false;
      return true;
    }

    const uint16_t mech_u16 = encoder_abs_to_angle_q16(enc.abs_pos);
    const uint16_t elec_u16 = (uint16_t)(
      encoder_abs_to_electrical_angle_q16(enc.abs_pos, pole_pairs) -
      parameter.encoder_electrical_zero_u16);

    primask = __get_PRIMASK();
    __disable_irq();
    uint32_t speed_mhz   = enc_speed.elec_freq_millihz;
    int8_t   direction   = enc_speed.direction;
    bool     spd_valid   = enc_speed.valid;
    int32_t  speed_millirpm = enc_speed.speed_millirpm;
    uint32_t spd_count   = enc_speed.update_count;
    uint32_t spd_cycles  = enc_speed.last_cycles;
    feedback->mechanical_angle_u16    = (uint16_t)(mech_u16 - encoder_mechanical_zero_u16);
    feedback->mechanical_angle_valid  = true;
    __set_PRIMASK(primask);

    feedback->source                    = MOTOR_ROTOR_FEEDBACK_SOURCE_ABSOLUTE_ENCODER;
    feedback->electrical_angle_u16      = elec_u16;
    feedback->mechanical_speed_millirpm = speed_millirpm;
    feedback->electrical_frequency_millihz = speed_mhz;
    /* 编码器每帧都给出绝对角度：角度有效即 valid；静止时速度为 0 也是有效测量。 */
    feedback->timestamp_cycles          = spd_cycles;
    feedback->update_count              = spd_count;
    feedback->source_state              = 0U;
    feedback->direction                 = direction;
    feedback->valid                     = true;
    feedback->speed_valid               = spd_valid;
    return true;
  }

#else /* MOTOR_POS_SENSOR_HALL */
  {
    motor_hall_angle_estimator_t estimator;
    motor_hall_sample_t hall;
    motor_parameter_t parameter;
    uint16_t sector_span_u16;
    uint16_t standstill_angle_u16;
    uint8_t pole_pairs;

    if (
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
    if (motor_hall_port_raw_state_get() != hall.state || !hall.valid || hall.health_fault)
    {
      feedback->valid = false;
      feedback->speed_valid = false;
      feedback->mechanical_angle_valid = false;
      return true;
    }
    if ((!estimator.valid) && hall.valid &&
        motor_parameter_active_read(&parameter) &&
        motor_hall_sector_span_get(parameter.hall_positive_next,
                                   parameter.hall_entry_angle_u16,
                                   hall.state, &sector_span_u16))
    {
      standstill_angle_u16 = (uint16_t)(
        parameter.hall_entry_angle_u16[hall.state] + sector_span_u16 / 2U +
        parameter.hall_rotor_offset_u16);
      feedback->source = MOTOR_ROTOR_FEEDBACK_SOURCE_HALL;
      feedback->electrical_angle_u16 = standstill_angle_u16;
      primask = __get_PRIMASK();
      __disable_irq();
      feedback->mechanical_angle_u16 =
        motor_rotor_feedback_tracker.mechanical_angle_u16;
      feedback->mechanical_angle_valid = motor_rotor_feedback_tracker.tracking;
      __set_PRIMASK(primask);
      feedback->mechanical_speed_millirpm = 0;
      feedback->electrical_frequency_millihz = 0U;
      feedback->timestamp_cycles = motor_timebase_cycles_get();
      feedback->update_count = hall.frequency_update_count;
      feedback->source_state = hall.state;
      feedback->direction = 0;
      feedback->valid = true;
      feedback->speed_valid = false;
      feedback->mechanical_angle_valid = false;
      return true;
    }
    mechanical_speed_millirpm = (uint32_t)(
      ((uint64_t)estimator.electrical_frequency_millihz * 60U) / pole_pairs);
    feedback->source = MOTOR_ROTOR_FEEDBACK_SOURCE_HALL;
    feedback->electrical_angle_u16 = estimator.electrical_angle_u16;
    primask = __get_PRIMASK();
    __disable_irq();
    feedback->mechanical_angle_u16 =
      motor_rotor_feedback_tracker.mechanical_angle_u16;
    feedback->mechanical_angle_valid = motor_rotor_feedback_tracker.tracking;
    __set_PRIMASK(primask);
    feedback->mechanical_speed_millirpm = (estimator.direction < 0) ?
      -(int32_t)mechanical_speed_millirpm : (int32_t)mechanical_speed_millirpm;
    feedback->electrical_frequency_millihz =
      estimator.electrical_frequency_millihz;
    feedback->timestamp_cycles = hall.timestamp_cycles;
    feedback->update_count = hall.frequency_update_count;
    feedback->source_state = hall.state;
    feedback->direction = estimator.direction;
    feedback->valid = estimator.valid && hall.valid &&
                      (estimator.direction != 0) &&
                      (estimator.electrical_frequency_millihz != 0U);
    feedback->speed_valid = feedback->valid;
    return true;
  }
#endif
}

void motor_rotor_feedback_process_speed(void)
{
  uint32_t primask;

#if (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_ENCODER)
  /* Drain the fast-loop window atomically. Leave the conversions preemptible. */
  primask = __get_PRIMASK();
  __disable_irq();
  const int64_t velocity_sum = enc_speed_window.velocity_sum;
  const uint32_t samples = enc_speed_window.samples;
  const uint32_t last_cycles = enc_speed_window.last_cycles;
  enc_speed_window.velocity_sum = 0;
  enc_speed_window.samples = 0U;
  __set_PRIMASK(primask);

  const uint8_t pole_pairs = motor_parameter_pole_pairs_get();
  if (samples == 0U || pole_pairs == 0U || motor_rotor_feedback_override_enabled)
  {
    enc_speed.valid = false;
    return;
  }

  const int64_t denominator = (int64_t)ENC_SINGLE_MAX * samples;
  const uint64_t magnitude = (uint64_t)(velocity_sum < 0 ? -velocity_sum : velocity_sum);
  const uint64_t frequency = magnitude * 1000U * pole_pairs / (uint64_t)denominator;
  const int64_t speed = velocity_sum * 60000 / denominator;

  primask = __get_PRIMASK();
  __disable_irq();
  enc_speed.elec_freq_millihz = frequency > UINT32_MAX ? UINT32_MAX : (uint32_t)frequency;
  enc_speed.speed_millirpm = (int32_t)speed;
  enc_speed.direction = (velocity_sum > 0) - (velocity_sum < 0);
  enc_speed.last_cycles = last_cycles;
  enc_speed.update_count++;
  enc_speed.valid = true;
  __set_PRIMASK(primask);
#else /* MOTOR_POS_SENSOR_HALL */
  const uint8_t pole_pairs = motor_parameter_pole_pairs_get();

  if (motor_rotor_feedback_override_enabled || pole_pairs == 0U)
  {
    primask = __get_PRIMASK();
    __disable_irq();
    motor_rotor_feedback_tracker.tracking = false;
    motor_rotor_feedback_tracker.residue = 0;
    __set_PRIMASK(primask);
    return;
  }

  {
    motor_hall_angle_estimator_t estimator;
    int16_t electrical_delta;
    int32_t scaled_delta;
    int32_t mechanical_delta;
    int32_t residue;

    if (!motor_hall_angle_estimator_read(&estimator) || !estimator.valid)
    {
      primask = __get_PRIMASK();
      __disable_irq();
      motor_rotor_feedback_tracker.tracking = false;
      motor_rotor_feedback_tracker.residue = 0;
      __set_PRIMASK(primask);
      return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (!motor_rotor_feedback_tracker.tracking)
    {
      motor_rotor_feedback_tracker.last_electrical_u16 =
        estimator.electrical_angle_u16;
      motor_rotor_feedback_tracker.residue = 0;
      motor_rotor_feedback_tracker.tracking = true;
      __set_PRIMASK(primask);
      return;
    }

    /* 1 kHz 采样下，只要电频率 < 500 Hz 就不发生 Nyquist 混叠。 */
    electrical_delta = (int16_t)(estimator.electrical_angle_u16 -
                                 motor_rotor_feedback_tracker.last_electrical_u16);
    scaled_delta = (int32_t)electrical_delta +
                   motor_rotor_feedback_tracker.residue;
    mechanical_delta = scaled_delta / (int32_t)pole_pairs;
    residue = scaled_delta - mechanical_delta * (int32_t)pole_pairs;
    motor_rotor_feedback_tracker.mechanical_angle_u16 = (uint16_t)(
      (int32_t)motor_rotor_feedback_tracker.mechanical_angle_u16 +
      mechanical_delta);
    motor_rotor_feedback_tracker.residue = residue;
    motor_rotor_feedback_tracker.last_electrical_u16 =
      estimator.electrical_angle_u16;
    __set_PRIMASK(primask);
  }
#endif
}
