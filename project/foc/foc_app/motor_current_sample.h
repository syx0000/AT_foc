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
} motor_current_sample_state_t;

/**
 * @brief 初始化正式电流采样状态。
 * @param 无。
 * @return 无。
 * @details 清除零偏有效标志、电流快照、连续超限计数及过流故障锁存。
 */
void motor_current_sample_init(void);

/**
 * @brief 检查两相ADC零偏是否满足启动条件。
 * @param phase_a_offset_raw SOA校准平均值。
 * @param phase_b_offset_raw SOB校准平均值。
 * @return true表示两相均在2048±160且相互差值不超过80；否则返回false。
 */
bool motor_current_sample_offsets_valid(
  uint16_t phase_a_offset_raw,
  uint16_t phase_b_offset_raw);

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
 * @brief 将单相ADC原始值转换为有符号电流毫安值。
 * @param adc_raw 当前12位ADC原始值。
 * @param offset_raw 该相启动零偏校准值。
 * @return 相电流，单位mA；实测确认ADC值相对零偏减小为电流正方向。
 * @details 按3.3 V参考、2.5 mΩ分流电阻和10 V/V CSA增益进行64位整数换算。
 */
int32_t motor_current_sample_raw_to_ma(
  uint16_t adc_raw,
  uint16_t offset_raw);

/**
 * @brief 将物理相电流转换为FOC使用的Q15标幺值。
 * @param current_ma 相电流，单位mA。
 * @return 以固定128 A为1.0的Q15值，输出限制在-32768至32767。
 * @details Q15基准与50 A指令限幅相互独立，修改控制限幅不会改变PI等内核
 *          参数的物理标度；128 A基准下1 A正好对应256个Q15单位。
 */
int16_t motor_current_sample_ma_to_q15(int32_t current_ma);

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

#endif /* MOTOR_CURRENT_SAMPLE_H */
