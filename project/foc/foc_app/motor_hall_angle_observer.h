#ifndef MOTOR_HALL_ANGLE_OBSERVER_H
#define MOTOR_HALL_ANGLE_OBSERVER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint16_t state_angle_u16[8]; /**< 进入各Hall状态时的环形IIR平均电角度。 */
  uint32_t state_count[8];     /**< 各Hall状态的有效边沿累计次数。 */
} motor_hall_angle_observer_t;

/**
 * @brief 初始化Hall边沿电角度观测器。
 * @param 无。
 * @return 无。
 * @details 清零六个有效Hall状态的最近角度和采样次数，不操作任何硬件。
 */
void motor_hall_angle_observer_init(void);

/**
 * @brief 在Hall边沿中断完成端口采样后记录开环电角度。
 * @param 无。
 * @return 无。
 * @details 仅在开环控制处于RUNNING且Hall状态有效时记录；使用1/8步长的有符号
 *          环形IIR滤波，可跨越0/65535边界，用于稳定标定Hall状态边界角度。
 */
void motor_hall_angle_observer_edge_process(void);

/**
 * @brief 原子读取Hall边沿电角度观测结果。
 * @param observer 输出各状态最近角度和累计次数，不允许为空。
 * @return 参数有效时返回true，传入NULL时返回false。
 */
bool motor_hall_angle_observer_read(motor_hall_angle_observer_t *observer);

#endif /* MOTOR_HALL_ANGLE_OBSERVER_H */
