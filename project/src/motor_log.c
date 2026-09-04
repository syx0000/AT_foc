/**
 * @file    motor_log.c
 * @brief   电机分级日志 功能实现
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#include "motor_log.h"

volatile uint8_t motor_log_level = MOTOR_LOG_LEVEL_INFO;

void motor_log_level_set(motor_log_level_t level)
{
  if ((uint8_t)level <= (uint8_t)MOTOR_LOG_LEVEL_INFO)
  {
    motor_log_level = (uint8_t)level;
  }
}

motor_log_level_t motor_log_level_get(void)
{
  return (motor_log_level_t)motor_log_level;
}
