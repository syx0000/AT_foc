#ifndef MOTOR_SPEED_FEEDBACK_H
#define MOTOR_SPEED_FEEDBACK_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  int32_t raw_speed_millirpm;       /**< 未滤波有符号机械转速，单位0.001 rpm。 */
  int32_t filtered_speed_millirpm;  /**< 低通滤波后的有符号机械转速，单位0.001 rpm。 */
  uint32_t electrical_frequency_millihz; /**< Hall测得的电频率，单位mHz。 */
  uint32_t sample_count;            /**< 已处理的新Hall测速结果累计次数。 */
  int8_t direction;                 /**< Hall方向：1为正向，-1为反向，0为未知。 */
  bool valid;                       /**< Hall状态、方向、频率和边沿时效均有效时为true。 */
} motor_speed_feedback_t;

/**
 * @brief 初始化Hall机械转速反馈模块。
 * @param 无。
 * @return 无。
 * @details 清零原始转速、滤波转速、方向、累计次数和有效标志，不操作Hall与PWM硬件。
 */
void motor_speed_feedback_init(void);

/**
 * @brief 执行一次1 kHz机械转速换算与低通滤波。
 * @param 无。
 * @return 无。
 * @details 读取Hall电频率和方向，按电频率乘60再除以电机极对数换算为有符号机械milli-rpm；
 *          仅当Hall边沿产生新测速结果时更新IIR滤波，边沿之间保持最近结果；
 *          Hall无效、频率为0、方向未知或边沿超时时立即将反馈置为无效并归零。
 * @note 仅供1 kHz SysTick中断调用。
 */
void motor_speed_feedback_process_1khz(void);

/**
 * @brief 原子读取机械转速反馈快照。
 * @param feedback 输出原始/滤波转速、电频率、方向、累计次数和有效性，不允许为空。
 * @return 参数有效时返回true；传入NULL时返回false。
 */
bool motor_speed_feedback_read(motor_speed_feedback_t *feedback);

#endif /* MOTOR_SPEED_FEEDBACK_H */
