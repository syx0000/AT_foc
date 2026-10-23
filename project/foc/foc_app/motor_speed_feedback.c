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

/* 观测用8点滑动平均(1 kHz → 8 ms窗口)，对应demo SysTick中的speed_ma_fliter。 */
#define MOTOR_SPEED_DISPLAY_FILTER_SIZE 8U
static int32_t motor_speed_display_buffer[MOTOR_SPEED_DISPLAY_FILTER_SIZE];
static int64_t motor_speed_display_sum;
static uint8_t motor_speed_display_index;
static uint8_t motor_speed_display_count;

/**
 * @brief 清空观测用滑动平均窗口。
 * @param 无。
 * @return 无。
 */
static void motor_speed_display_reset(void)
{
  uint8_t i;

  for (i = 0U; i < MOTOR_SPEED_DISPLAY_FILTER_SIZE; i++)
  {
    motor_speed_display_buffer[i] = 0;
  }
  motor_speed_display_sum = 0;
  motor_speed_display_index = 0U;
  motor_speed_display_count = 0U;
  motor_speed_feedback.display_speed_millirpm = 0;
}

/**
 * @brief 向观测用滑动平均窗口压入一个样本并更新平均值。
 * @param speed_millirpm 当前1 kHz周期的速度反馈，单位0.001 rpm。
 * @return 无。
 */
static void motor_speed_display_push(int32_t speed_millirpm)
{
  if (motor_speed_display_count >= MOTOR_SPEED_DISPLAY_FILTER_SIZE)
  {
    motor_speed_display_sum -=
      motor_speed_display_buffer[motor_speed_display_index];
  }
  else
  {
    motor_speed_display_count++;
  }
  motor_speed_display_buffer[motor_speed_display_index] = speed_millirpm;
  motor_speed_display_sum += speed_millirpm;
  motor_speed_display_index++;
  if (motor_speed_display_index >= MOTOR_SPEED_DISPLAY_FILTER_SIZE)
  {
    motor_speed_display_index = 0U;
  }
  motor_speed_feedback.display_speed_millirpm = (int32_t)(
    motor_speed_display_sum / (int64_t)motor_speed_display_count);
}

void motor_speed_feedback_init(void)
{
  motor_speed_feedback.raw_speed_millirpm = 0;
  motor_speed_feedback.filtered_speed_millirpm = 0;
  motor_speed_feedback.electrical_frequency_millihz = 0U;
  motor_speed_feedback.sample_count = 0U;
  motor_speed_feedback.direction = 0;
  motor_speed_feedback.valid = false;
  motor_speed_feedback_last_update_count = 0U;
  motor_speed_display_reset();
}

void motor_speed_feedback_process_1khz(void)
{
  motor_rotor_feedback_t rotor_feedback;
  int32_t raw_speed_millirpm;
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
    /* 反馈失效按0 rpm继续平均，与控制侧"无效即0"保持一致。 */
    motor_speed_display_push(0);
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
    /* Rotor feedback is already the continuous Hall-PLL estimate. */
    motor_speed_feedback.filtered_speed_millirpm = raw_speed_millirpm;
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
  motor_speed_display_push(motor_speed_feedback.filtered_speed_millirpm);
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
