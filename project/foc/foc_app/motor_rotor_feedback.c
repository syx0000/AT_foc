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
/* 编码器模式：用相邻两帧 abs_pos 差分估算速度 */
static volatile struct {
  uint32_t last_abs_pos;          /**< 上一帧单圈位置 */
  uint32_t last_cycles;           /**< 上一帧时间戳（CPU 周期） */
  int32_t  speed_millirpm;        /**< 有符号机械转速，单位 milli-rpm */
  uint32_t elec_freq_millihz;     /**< 电频率绝对值，单位 mHz */
  int8_t   direction;             /**< 1=正向，-1=反向，0=未知 */
  uint32_t update_count;          /**< 速度更新次数，供速度反馈层判断新样本 */
  uint32_t last_frame_count;      /**< 上次使用的编码器帧计数 */
  bool     initialized;           /**< 已采到第一帧 */
} enc_speed;
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
  enc_speed.last_abs_pos    = 0U;
  enc_speed.last_cycles     = 0U;
  enc_speed.speed_millirpm  = 0;
  enc_speed.elec_freq_millihz = 0U;
  enc_speed.direction       = 0;
  enc_speed.initialized     = false;
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
    const enc_data_t *enc = encoder_rs485_get_data();
    uint8_t pole_pairs = motor_parameter_pole_pairs_get();
    uint32_t mechanical_speed_millirpm;

    if (pole_pairs == 0U) return false;
    if (!enc->valid)
    {
      feedback->valid      = false;
      feedback->speed_valid = false;
      return true;
    }

    /* 单圈机械角：17bit → U16（右移1位） */
    uint16_t mech_u16 = encoder_abs_to_angle_q16(enc->abs_pos);

    /* 电角度 = (机械角 - 编码器零位偏移) × 极对数（取模 U16 自然溢出） */
    motor_parameter_t parameter;
    uint16_t offset = 0U;
    if (motor_parameter_active_read(&parameter))
      offset = parameter.encoder_offset_u16;
    uint16_t elec_u16 = (uint16_t)((uint32_t)((uint16_t)(mech_u16 - offset)) * pole_pairs);

    primask = __get_PRIMASK();
    __disable_irq();
    uint32_t speed_mhz   = enc_speed.elec_freq_millihz;
    int8_t   direction   = enc_speed.direction;
    bool     spd_valid   = enc_speed.initialized;
    uint32_t spd_count   = enc_speed.update_count;
    uint32_t spd_cycles  = enc_speed.last_cycles;
    feedback->mechanical_angle_u16    = motor_rotor_feedback_tracker.mechanical_angle_u16;
    feedback->mechanical_angle_valid  = motor_rotor_feedback_tracker.tracking;
    __set_PRIMASK(primask);

    mechanical_speed_millirpm = (uint32_t)(
      ((uint64_t)speed_mhz * 60U) / pole_pairs);

    feedback->source                    = MOTOR_ROTOR_FEEDBACK_SOURCE_ABSOLUTE_ENCODER;
    feedback->electrical_angle_u16      = elec_u16;
    feedback->mechanical_speed_millirpm = (direction < 0) ?
      -(int32_t)mechanical_speed_millirpm : (int32_t)mechanical_speed_millirpm;
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
    uint32_t mechanical_speed_millirpm;

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

void motor_rotor_feedback_process_1khz(void)
{
  uint8_t pole_pairs;
  uint32_t primask;

  if (motor_rotor_feedback_override_enabled)
  {
    primask = __get_PRIMASK();
    __disable_irq();
    motor_rotor_feedback_tracker.tracking = false;
    motor_rotor_feedback_tracker.residue = 0;
    __set_PRIMASK(primask);
    return;
  }

  pole_pairs = motor_parameter_pole_pairs_get();
  if (pole_pairs == 0U)
  {
    primask = __get_PRIMASK();
    __disable_irq();
    motor_rotor_feedback_tracker.tracking = false;
    motor_rotor_feedback_tracker.residue = 0;
    __set_PRIMASK(primask);
    return;
  }

#if (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_ENCODER)
  {
    const enc_data_t *enc = encoder_rs485_get_data();
    uint32_t now_cycles;
    int32_t  pos_delta;
    uint32_t dt_cycles;
    /* 电频率：pos_delta(单圈) / ENC_SINGLE_MAX 圈 / dt_s = 转速(Hz电) */
    /* 转速(mHz电) = pos_delta * core_clock * 1000 / (ENC_SINGLE_MAX * dt_cycles) */
    uint64_t freq_num;
    uint32_t freq_millihz;
    int8_t   dir;

    if (!enc->valid)
    {
      primask = __get_PRIMASK();
      __disable_irq();
      motor_rotor_feedback_tracker.tracking = false;
      motor_rotor_feedback_tracker.residue = 0;
      enc_speed.initialized = false;
      __set_PRIMASK(primask);
      return;
    }

    now_cycles = motor_timebase_cycles_get();

    primask = __get_PRIMASK();
    __disable_irq();

    if (!enc_speed.initialized)
    {
      enc_speed.last_abs_pos  = enc->abs_pos;
      enc_speed.last_cycles   = now_cycles;
      enc_speed.initialized   = true;
      enc_speed.direction     = 0;
      enc_speed.elec_freq_millihz = 0U;
      enc_speed.speed_millirpm    = 0;
      motor_rotor_feedback_tracker.last_electrical_u16 =
        (uint16_t)((uint32_t)(enc->abs_pos >> 1u) * pole_pairs);
      motor_rotor_feedback_tracker.tracking = false;
      motor_rotor_feedback_tracker.residue  = 0;
      __set_PRIMASK(primask);
      return;
    }

    dt_cycles = now_cycles - enc_speed.last_cycles;

    /* 有符号差分，处理跨圈回绕（131072 步一圈） */
    pos_delta = (int32_t)enc->abs_pos - (int32_t)enc_speed.last_abs_pos;
    if (pos_delta >  (int32_t)(ENC_SINGLE_MAX / 2u))
      pos_delta -= (int32_t)ENC_SINGLE_MAX;
    else if (pos_delta < -(int32_t)(ENC_SINGLE_MAX / 2u))
      pos_delta += (int32_t)ENC_SINGLE_MAX;

    if (pos_delta > 0)       dir =  1;
    else if (pos_delta < 0)  dir = -1;
    else                     dir =  0;

    /* 避免除零：dt_cycles 应至少 1 */
    if (dt_cycles == 0U) dt_cycles = 1U;

    /* 机械频率(mHz) = |pos_delta| * system_core_clock * 1000
     *                 / (ENC_SINGLE_MAX * dt_cycles)
     * 电频率(mHz)   = 机械频率 × 极对数                         */
    uint32_t abs_delta = (pos_delta < 0) ? (uint32_t)(-pos_delta) : (uint32_t)pos_delta;
    freq_num = (uint64_t)abs_delta * (uint64_t)system_core_clock * 1000ULL;
    freq_num = (freq_num / ((uint64_t)ENC_SINGLE_MAX * (uint64_t)dt_cycles)) *
               (uint64_t)pole_pairs;
    freq_millihz = (freq_num > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : (uint32_t)freq_num;

    enc_speed.elec_freq_millihz = freq_millihz;
    enc_speed.direction         = dir;
    enc_speed.speed_millirpm    = (dir < 0) ?
      -(int32_t)((uint64_t)freq_millihz * 60U / pole_pairs) :
       (int32_t)((uint64_t)freq_millihz * 60U / pole_pairs);
    enc_speed.last_abs_pos = enc->abs_pos;
    enc_speed.last_cycles  = now_cycles;
    enc_speed.update_count++;

    /* 机械角累加器：直接使用编码器单圈机械角 Q16（右移1位） */
    uint16_t mech_now = (uint16_t)(enc->abs_pos >> 1u);
    if (!motor_rotor_feedback_tracker.tracking)
    {
      motor_rotor_feedback_tracker.last_electrical_u16 =
        (uint16_t)((uint32_t)mech_now * pole_pairs);
      motor_rotor_feedback_tracker.mechanical_angle_u16 = mech_now;
      motor_rotor_feedback_tracker.residue  = 0;
      motor_rotor_feedback_tracker.tracking = true;
    }
    else
    {
      motor_rotor_feedback_tracker.mechanical_angle_u16 = mech_now;
    }

    __set_PRIMASK(primask);
  }

#else /* MOTOR_POS_SENSOR_HALL */
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
