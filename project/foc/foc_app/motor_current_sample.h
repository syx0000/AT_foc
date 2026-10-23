/**
 * @file    motor_current_sample.h
 * @brief   三相电流采样与保护 公共数据结构和接口声明
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#ifndef MOTOR_CURRENT_SAMPLE_H
#define MOTOR_CURRENT_SAMPLE_H

#include <stdbool.h>
#include <stdint.h>
#include "motor_adc_port.h"

typedef struct
{
  int32_t phase_a_ma;          /**< A相电流，单位mA。 */
  int32_t phase_b_ma;          /**< B相电流，单位mA。 */
  int32_t phase_c_ma;          /**< 根据Ia+Ib+Ic=0重构的C相电流，单位mA。 */
  int16_t phase_a_q15;         /**< 以128 A为1.0的A相Q15电流。 */
  int16_t phase_b_q15;         /**< 以128 A为1.0的B相Q15电流。 */
  int16_t phase_c_q15;         /**< 以128 A为1.0的C相Q15电流。 */
  uint32_t sample_count;       /**< 零偏生效后的累计采样次数。 */
  uint32_t overcurrent_count;  /**< 当前连续超过55 A的样本数。 */
  bool overcurrent_fault;      /**< 软件过流故障锁存标志。 */
  int32_t overcurrent_trip_phase_a_ma; /**< 首次触发软件过流时的A相电流。 */
  int32_t overcurrent_trip_phase_b_ma; /**< 首次触发软件过流时的B相电流。 */
  int32_t overcurrent_trip_phase_c_ma; /**< 首次触发软件过流时的C相电流。 */
} motor_current_sample_state_t;

/**
 * @brief 初始化正式电流采样状态。
 * @param 无。
 * @return 无。
 * @details 清除零偏有效标志、电流快照、连续超限计数及过流故障锁存。
 */
void motor_current_sample_init(void);

/**
 * @brief 设置正式电流采样使用的两相零偏。
 * @param phase_a_offset_raw SOA校准平均值。
 * @param phase_b_offset_raw SOB校准平均值。
 * @return true表示零偏合法并已生效；false表示校验失败，原零偏失效。
 */
bool motor_current_sample_offsets_set(
  uint16_t phase_a_offset_raw,
  uint16_t phase_b_offset_raw);

/**
 * @brief 处理一次10 kHz快速ADC采样并执行软件过流监测。
 * @param sample 本次SOA、SOB快速ADC原始采样，不允许为空。
 * @return true仅表示本次首次触发55 A连续500点过流故障；其余返回false。
 * @details 零偏未生效时不计算。生成三相mA及Q15电流，C相按
 *          Ic=-(Ia+Ib)重构；任一相超限则累计，恢复到阈值内立即清零。
 */
bool motor_current_sample_process(const motor_adc_fast_sample_t *sample);

/**
 * @brief 原子读取最近一次正式电流采样状态。
 * @param state 输出三相电流、计数和故障状态，不允许为空。
 * @return true表示零偏已生效且至少处理过一次采样；否则返回false。
 */
bool motor_current_sample_state_read(motor_current_sample_state_t *state);

/**
 * @brief 在三相电流恢复安全且零偏有效时清除软件过流锁存。
 * @param 无。
 * @return 满足清除条件并完成清除时返回true，否则返回false。
 */
bool motor_current_sample_fault_clear(void);

#endif /* MOTOR_CURRENT_SAMPLE_H */
