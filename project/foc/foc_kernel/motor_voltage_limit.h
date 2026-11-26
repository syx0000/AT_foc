/**
 * @file    motor_voltage_limit.h
 * @brief   dq电压矢量限幅算法 公共数据结构和接口声明
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#ifndef MOTOR_VOLTAGE_LIMIT_H
#define MOTOR_VOLTAGE_LIMIT_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 对 dq 矢量执行圆限幅。
 * @param direct_mv 输入/输出d轴电压，单位mV，不允许为空。
 * @param quadrature_mv 输入/输出q轴电压，单位mV，不允许为空。
 * @param limit_mv dq合成电压上限，单位mV，必须大于0。
 * @return 电压被缩放时返回true；无需限幅或参数无效时返回false。
 * @details 整数模长向上取整，保证限幅后真实幅值不超限。
 */
bool motor_voltage_limit_apply(int32_t *direct_mv,
                               int32_t *quadrature_mv,
                               int32_t limit_mv);

/* Maximum magnitude left on one axis after allocating the other axis. */
int32_t motor_vector_axis_headroom(int32_t occupied, int32_t limit);

#endif /* MOTOR_VOLTAGE_LIMIT_H */
