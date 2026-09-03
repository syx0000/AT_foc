#ifndef MOTOR_VOLTAGE_LIMIT_H
#define MOTOR_VOLTAGE_LIMIT_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 对dq两轴电压执行无开方的保守矢量限幅。
 * @param direct_mv 输入/输出d轴电压，单位mV，不允许为空。
 * @param quadrature_mv 输入/输出q轴电压，单位mV，不允许为空。
 * @param limit_mv dq合成电压上限，单位mV，必须大于0。
 * @return 电压被缩放时返回true；无需限幅或参数无效时返回false。
 * @details 使用max+min/2作为真实矢量幅值的上界，保证限幅后真实幅值不超限。
 */
bool motor_voltage_limit_apply(int32_t *direct_mv,
                               int32_t *quadrature_mv,
                               int32_t limit_mv);

#endif /* MOTOR_VOLTAGE_LIMIT_H */
