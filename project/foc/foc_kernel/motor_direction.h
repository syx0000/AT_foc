#ifndef MOTOR_DIRECTION_H
#define MOTOR_DIRECTION_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 获取逻辑方向配置对应的符号。
 * @param direction_inverted false表示逻辑方向与物理正向一致，true表示相反。
 * @return 未反转返回1，反转返回-1。
 */
int8_t motor_direction_sign_get(bool direction_inverted);

/**
 * @brief 将逻辑有符号量转换为物理有符号量，或执行相同的逆变换。
 * @param logical_value 逻辑坐标系输入值。
 * @param direction_inverted 是否反转逻辑方向。
 * @param physical_value 输出物理坐标系值，不允许为空。
 * @return 转换成功时返回true；输出为空或反转INT32_MIN会溢出时返回false。
 * @details 方向变换自身互逆，因此也可用于物理反馈转换到逻辑坐标系。
 */
bool motor_direction_transform_s32(int32_t logical_value,
                                   bool direction_inverted,
                                   int32_t *physical_value);

#endif /* MOTOR_DIRECTION_H */
