/**
 * @file    motor_hall_angle_math.h
 * @brief   Hall角度定点算法 公共数据结构和接口声明
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#ifndef MOTOR_HALL_ANGLE_MATH_H
#define MOTOR_HALL_ANGLE_MATH_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 根据Hall物理方向解析本次边沿对应的转子电角度。
 * @param positive_next 8元素正向Hall后继表，不允许为空。
 * @param entry_angle_u16 8元素正向状态进入角表，不允许为空。
 * @param current_state 本次边沿后的Hall状态，范围1..6。
 * @param physical_direction 物理方向，只允许1或-1。
 * @param rotor_offset_u16 Hall边界到转子dq坐标系的U16角度补偿。
 * @param boundary_angle_u16 输出补偿后的边界电角度，不允许为空。
 * @return 输入合法时返回true，否则返回false。
 * @details 正向取当前状态进入角；反向取当前状态正向后继的进入角。
 */
bool motor_hall_angle_boundary_get(const uint8_t positive_next[8],
                                   const uint16_t entry_angle_u16[8],
                                   uint8_t current_state,
                                   int8_t physical_direction,
                                   uint16_t rotor_offset_u16,
                                   uint16_t *boundary_angle_u16);

/**
 * @brief 计算每个快速控制周期的有符号U32相位累加步长。
 * @param electrical_frequency_millihz 电频率绝对值，单位mHz且必须大于0。
 * @param update_frequency_hz 快速控制更新频率，单位Hz且必须大于0。
 * @param physical_direction 物理方向，只允许1或-1。
 * @param phase_step 输出Q16.16角度累加器使用的有符号步长，不允许为空。
 * @return 参数合法且步长不超过INT32_MAX时返回true，否则返回false。
 */
bool motor_hall_phase_step_get(uint32_t electrical_frequency_millihz,
                               uint32_t update_frequency_hz,
                               int8_t physical_direction,
                               int32_t *phase_step);

bool motor_hall_sector_span_get(const uint8_t positive_next[8],
                                const uint16_t entry_angle_u16[8],
                                uint8_t state,
                                uint16_t *span_u16);

/**
 * @brief 根据有符号电角速度和固定系统延迟计算Hall动态超前补偿角。
 * @param electrical_frequency_millihz 电角频率绝对值，单位mHz。
 * @param physical_direction 物理方向，只允许1或-1。
 * @param delay_us 待补偿的固定系统延迟，单位us；0表示禁用动态补偿。
 * @param maximum_compensation_u16 最大补偿角绝对值，U16一周制且必须小于半周。
 * @param compensation_u16 输出有符号动态补偿角，正转为正、反转为负。
 * @return 参数合法时返回true。
 */
bool motor_hall_dynamic_compensation_get(
  uint32_t electrical_frequency_millihz,
  int8_t physical_direction,
  uint32_t delay_us,
  uint16_t maximum_compensation_u16,
  int32_t *compensation_u16);

#endif /* MOTOR_HALL_ANGLE_MATH_H */
