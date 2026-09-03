#ifndef MOTOR_CURRENT_PI_H
#define MOTOR_CURRENT_PI_H

#include <stdint.h>

typedef struct
{
  int32_t proportional_gain_q15; /**< 比例增益，Q15格式，物理单位V/A。 */
  int32_t integral_gain_q15;     /**< 单采样积分增益，Q15格式，物理单位V/A。 */
  int32_t output_limit_mv;       /**< 输出电压绝对限幅，单位mV。 */
  int32_t integral_mv;           /**< 积分器状态，单位mV。 */
  int32_t output_mv;             /**< 最近一次PI输出，单位mV。 */
} motor_current_pi_axis_t;

/**
 * @brief 初始化单轴电流PI控制器。
 * @param controller 控制器状态，不允许为空。
 * @param proportional_gain_q15 比例增益Q15；输入mA时输出自然为mV。
 * @param integral_gain_q15 每个采样周期的积分增益Q15。
 * @param output_limit_mv 输出电压绝对限幅，单位mV，必须大于0。
 * @return 无。
 */
void motor_current_pi_axis_init(motor_current_pi_axis_t *controller,
                                int32_t proportional_gain_q15,
                                int32_t integral_gain_q15,
                                int32_t output_limit_mv);

/**
 * @brief 清零单轴电流PI积分器和输出。
 * @param controller 控制器状态；传入空指针时忽略。
 * @return 无。
 */
void motor_current_pi_axis_reset(motor_current_pi_axis_t *controller);

/**
 * @brief 执行一次位置式电流PI计算。
 * @param controller 已初始化的控制器状态，不允许为空。
 * @param reference_ma 电流指令，单位mA。
 * @param feedback_ma 电流反馈，单位mA。
 * @return 限幅后的电压指令，单位mV。
 * @details 比例项和积分项均使用64位中间值；饱和且误差继续推动饱和时暂停积分，
 *          误差有助于退出饱和时允许积分回退。
 */
int32_t motor_current_pi_axis_process(motor_current_pi_axis_t *controller,
                                      int32_t reference_ma,
                                      int32_t feedback_ma);

#endif
