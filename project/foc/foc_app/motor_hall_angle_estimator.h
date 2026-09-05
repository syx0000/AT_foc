/**
 * @file    motor_hall_angle_estimator.h
 * @brief   Hall转子电角度估算器 公共数据结构和接口声明
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#ifndef MOTOR_HALL_ANGLE_ESTIMATOR_H
#define MOTOR_HALL_ANGLE_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  int32_t dynamic_compensation_u16;    /**< 按电角速度和延迟计算的有符号角度补偿。 */
  int32_t phase_step;                  /**< 当前Q16.16有符号快速环相位步长。 */
  int32_t edge_error_u16;              /**< 最近一次Hall边沿的有符号预测角误差。 */
  int32_t edge_correction_remaining_u16; /**< 尚未完成的有符号平滑角修正。 */
  uint32_t sector_interval_cycles;     /**< 当前扇区采用的历史持续周期。 */
  uint16_t electrical_angle_u16;       /**< Hall边沿插值并补偿为转子坐标系的电角度。 */
  uint32_t electrical_frequency_millihz; /**< Hall完整周期电频率，单位mHz。 */
  uint8_t hall_state;                  /**< 当前Hall组合状态1..6。 */
  int8_t direction;                    /**< 物理方向：1正向、-1反向、0未知。 */
  bool valid;                          /**< 方向、频率有效且Hall未超时时为true。 */
  bool sector_interpolation_active;    /**< 当前扇区正在使用独立跨度/周期插值。 */
} motor_hall_angle_estimator_t;

/**
 * @brief 初始化Hall电角度估算器。
 * @param 无。
 * @return 无。
 * @details 清除角度、频率、相位累加器和有效标志，不操作PWM。
 */
void motor_hall_angle_estimator_init(void);

/**
 * @brief 在Hall边沿采样完成后校正估算角度和插值步长。
 * @param 无。
 * @return 无。
 * @details 使用实测状态边沿角度校正相位，再叠加开环标定到转子坐标系的固定
 *          角度补偿；正向使用当前状态进入角，反向使用当前状态正向后继的进入角。
 */
void motor_hall_angle_estimator_edge_process(void);

/**
 * @brief 执行一次10 kHz Hall边沿间电角度插值。
 * @param 无。
 * @return 无。
 * @details 按最近完整Hall周期频率和物理方向推进32位相位累加器；超过100 ms没有边沿则失效。
 */
void motor_hall_angle_estimator_fast_process(void);

/**
 * @brief 原子读取Hall电角度估算结果。
 * @param estimator 输出角度、频率、Hall状态和有效性，不允许为空。
 * @return 参数有效时返回true，传入NULL时返回false。
 */
bool motor_hall_angle_estimator_read(motor_hall_angle_estimator_t *estimator);

#endif /* MOTOR_HALL_ANGLE_ESTIMATOR_H */
