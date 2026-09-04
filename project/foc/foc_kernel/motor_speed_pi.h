#ifndef MOTOR_SPEED_PI_H
#define MOTOR_SPEED_PI_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  int32_t proportional_gain_q20; /**< 比例增益Q20，输入milli-rpm时输出mA。 */
  int32_t integral_gain_q20;     /**< 每个1 kHz周期的积分增益Q20，输入milli-rpm时输出mA。 */
  int32_t output_limit_ma;       /**< Iq电流指令绝对限幅，单位mA。 */
  int32_t integral_ma;           /**< 积分器状态，单位mA。 */
  int32_t output_ma;             /**< 最近一次限幅后的Iq输出，单位mA。 */
} motor_speed_pi_t;

/**
 * @brief 初始化1 kHz速度PI控制器。
 * @param controller 速度PI实例，不允许为空。
 * @param proportional_gain_q20 比例增益Q20，物理单位mA/milli-rpm。
 * @param integral_gain_q20 单个1 ms周期的积分增益Q20，物理单位mA/milli-rpm。
 * @param output_limit_ma Iq输出绝对限幅，单位mA，必须大于0。
 * @return 参数有效并完成初始化时返回true，否则返回false。
 * @details 初始化仅保存参数并清零状态，不读取传感器、不操作PWM。
 */
bool motor_speed_pi_init(motor_speed_pi_t *controller,
                         int32_t proportional_gain_q20,
                         int32_t integral_gain_q20,
                         int32_t output_limit_ma);

/**
 * @brief 清零速度PI积分器和输出。
 * @param controller 速度PI实例；传入NULL时忽略。
 * @return 无。
 */
void motor_speed_pi_reset(motor_speed_pi_t *controller);

/**
 * @brief 预置速度PI输出，用于控制模式无扰切换。
 * @param controller 已初始化的速度PI实例，不允许为空。
 * @param output_ma 期望初始Iq输出，单位mA，自动限制到输出范围。
 * @return 参数有效时返回true，否则返回false。
 * @details 同时预置积分器与输出，使第一次闭环计算从已有转矩电流附近开始。
 */
bool motor_speed_pi_output_seed(motor_speed_pi_t *controller,
                                int32_t output_ma);

/**
 * @brief 将PI状态跟踪到外部实际采用的Iq指令。
 * @param controller 已初始化的速度PI实例，不允许为空。
 * @param applied_output_ma 经过外部斜坡或保护限幅后实际采用的Iq，单位mA。
 * @return 无。
 * @details 用实际输出与PI请求值的差修正积分器，避免外部限幅导致积分饱和。
 */
void motor_speed_pi_output_track(motor_speed_pi_t *controller,
                                 int32_t applied_output_ma);

/**
 * @brief 执行一次1 kHz速度PI计算。
 * @param controller 已初始化的速度PI实例，不允许为空。
 * @param reference_millirpm 目标有符号机械转速，单位0.001 rpm。
 * @param feedback_millirpm 实际有符号机械转速，单位0.001 rpm。
 * @return 限幅后的Iq电流指令，单位mA；参数无效时返回0。
 * @details 使用64位中间结果；输出饱和且误差继续推动饱和时暂停积分，误差帮助退出饱和时允许积分回退。
 */
int32_t motor_speed_pi_process(motor_speed_pi_t *controller,
                               int32_t reference_millirpm,
                               int32_t feedback_millirpm);

#endif /* MOTOR_SPEED_PI_H */
