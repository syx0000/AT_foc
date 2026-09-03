#ifndef MOTOR_TIMEBASE_H
#define MOTOR_TIMEBASE_H

#include <stdint.h>

/**
 * @brief 初始化电机控制高精度时间基准。
 * @param 无。
 * @return 无。
 * @details 开启Cortex-M4 DWT周期计数器，记录当前CPU频率并从0开始计数。
 * @note 必须在系统时钟配置完成后调用。
 */
void motor_timebase_init(void);

/**
 * @brief 获取当前CPU周期时间戳。
 * @param 无。
 * @return DWT 32位周期计数值，192 MHz下约22.37秒回绕一次。
 */
uint32_t motor_timebase_cycles_get(void);

/**
 * @brief 计算从指定周期时间戳到当前时刻的周期数。
 * @param start_cycles 之前由motor_timebase_cycles_get()取得的时间戳。
 * @return 已经过的CPU周期数，支持一次32位自然回绕。
 */
uint32_t motor_timebase_cycles_elapsed(uint32_t start_cycles);

/**
 * @brief 将CPU周期数转换为微秒。
 * @param cycles 需要转换的CPU周期数。
 * @return 向下取整后的微秒数。
 */
uint32_t motor_timebase_cycles_to_us(uint32_t cycles);

/**
 * @brief 获取当前微秒时间戳。
 * @param 无。
 * @return 从DWT初始化开始计算的微秒数。
 * @note 该时间戳随32位DWT周期计数器回绕，仅用于短时间间隔测量。
 */
uint32_t motor_timebase_us_get(void);

#endif /* MOTOR_TIMEBASE_H */
