#ifndef MOTOR_CURRENT_TRANSFORM_H
#define MOTOR_CURRENT_TRANSFORM_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  int16_t alpha_q15;
  int16_t beta_q15;
  int16_t direct_q15;
  int16_t quadrature_q15;
  uint16_t electrical_angle_u16;
  uint32_t sample_count;
  bool valid;
} motor_current_transform_state_t;

typedef struct
{
  int16_t direct_average_q15;
  int16_t quadrature_average_q15;
  int16_t direct_minimum_q15;
  int16_t direct_maximum_q15;
  int16_t quadrature_minimum_q15;
  int16_t quadrature_maximum_q15;
  uint32_t sample_count;
} motor_current_transform_statistics_t;

/**
 * @brief 初始化电流坐标变换状态。
 * @param 无。
 * @return 无。
 * @details 清零alpha-beta、dq、角度和样本计数，不改变ADC或PWM状态。
 */
void motor_current_transform_init(void);

/**
 * @brief 执行一次10 kHz电流Clarke/Park变换。
 * @param 无。
 * @return 无。
 * @details 读取正式三相电流和Hall插值角度；两者有效时计算alpha-beta及dq电流。
 */
void motor_current_transform_fast_process(void);

/**
 * @brief 原子读取最近一次电流坐标变换结果。
 * @param state 输出alpha-beta、dq、角度、样本计数和有效性，不允许为空。
 * @return 参数有效时返回true，传入NULL时返回false。
 */
bool motor_current_transform_state_read(motor_current_transform_state_t *state);

/**
 * @brief 原子提取并清零dq电流统计窗口。
 * @param statistics 输出窗口内Id/Iq平均值、最小值、最大值和样本数，不允许为空。
 * @return 窗口内存在有效样本时返回true；无样本或传入NULL时返回false。
 * @details 10 kHz处理持续累加；调用时短暂关中断完成快照和窗口复位，适合每秒调用一次。
 */
bool motor_current_transform_statistics_snapshot_reset(
  motor_current_transform_statistics_t *statistics);

#endif /* MOTOR_CURRENT_TRANSFORM_H */
