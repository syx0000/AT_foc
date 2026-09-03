#ifndef MOTOR_RAMP_H
#define MOTOR_RAMP_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  int32_t current;
  int32_t target;
  uint32_t rate_per_second;
  uint32_t remainder;
} motor_ramp_t;

/**
 * @brief 初始化有符号线性斜坡。
 * @param ramp 斜坡状态，不允许为空。
 * @param initial 初始值。
 * @param target 目标值。
 * @param rate_per_second 每秒最大变化量，必须大于0。
 * @return 参数有效时返回true，否则返回false。
 */
bool motor_ramp_init(motor_ramp_t *ramp, int32_t initial, int32_t target,
                     uint32_t rate_per_second);

/**
 * @brief 更新斜坡目标和变化率，保留当前输出连续性。
 * @param ramp 已初始化的斜坡状态，不允许为空。
 * @param target 新目标值。
 * @param rate_per_second 每秒最大变化量，必须大于0。
 * @return 参数有效时返回true，否则返回false。
 */
bool motor_ramp_target_set(motor_ramp_t *ramp, int32_t target,
                           uint32_t rate_per_second);

/**
 * @brief 按固定调用频率推进一次有符号斜坡。
 * @param ramp 已初始化的斜坡状态，不允许为空。
 * @param update_frequency_hz 函数每秒调用次数，必须大于0。
 * @return 更新后的当前值；参数无效时返回0。
 */
int32_t motor_ramp_process(motor_ramp_t *ramp,
                           uint32_t update_frequency_hz);

#endif /* MOTOR_RAMP_H */
