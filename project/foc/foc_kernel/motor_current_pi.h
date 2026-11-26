/**
 * @file    motor_current_pi.h
 * @brief   电流PI定点算法 公共数据结构和接口声明
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#ifndef MOTOR_CURRENT_PI_H
#define MOTOR_CURRENT_PI_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  int32_t proportional_gain_q15; /**< 比例增益，Q15格式，物理单位V/A。 */
  int32_t integral_gain_q15;     /**< 单采样积分增益，Q15格式，物理单位V/A。 */
  int64_t integral_q15;          /**< 积分器状态，Q15 mV。 */
  int32_t integral_mv;           /**< 积分器状态，单位mV。 */
  int32_t output_mv;             /**< 最近一次PI输出，单位mV。 */
} motor_current_pi_axis_t;

/**
 * @brief 初始化单轴电流PI控制器。
 * @param controller 控制器状态，不允许为空。
 * @param proportional_gain_q15 比例增益Q15；输入mA时输出自然为mV。
 * @param integral_gain_q15 每个采样周期的积分增益Q15。
 * @return 无。
 */
void motor_current_pi_axis_init(motor_current_pi_axis_t *controller,
                                int32_t proportional_gain_q15,
                                int32_t integral_gain_q15);

/**
 * @brief 预置PI积分器和输出，用于控制模式无扰切换。
 * @param controller 已初始化的控制器状态，不允许为空。
 * @param output_mv 期望的初始输出电压，单位mV。
 * @return 参数有效时返回true，否则返回false。
 */
bool motor_current_pi_axis_output_seed(motor_current_pi_axis_t *controller,
                                       int32_t output_mv);

/* Apply the residual from final PWM voltage minus the total requested voltage. */
void motor_current_pi_axis_back_calculate(motor_current_pi_axis_t *controller,
                                         int64_t residual_mv,
                                         uint16_t gain_q15, uint16_t deadband_mv);

/**
 * @brief 执行一次位置式电流PI计算。
 * @param controller 已初始化的控制器状态，不允许为空。
 * @param reference_ma 电流指令，单位mA。
 * @param feedback_ma 电流反馈，单位mA。
 * @return 联合限幅前的电压指令，单位mV。
 */
int32_t motor_current_pi_axis_process(motor_current_pi_axis_t *controller,
                                      int32_t reference_ma,
                                      int32_t feedback_ma);

#endif
