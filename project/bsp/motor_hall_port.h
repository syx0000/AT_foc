#ifndef MOTOR_HALL_PORT_H
#define MOTOR_HALL_PORT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint8_t state;              /**< 原始Hall组合，bit0=HA、bit1=HB、bit2=HC。 */
  uint32_t edge_count;        /**< 三路Hall任意边沿的累计捕获次数。 */
  uint32_t timestamp_cycles;  /**< 本次原始边沿对应的DWT周期时间戳。 */
} motor_hall_port_sample_t;

/**
 * @brief 初始化Hall硬件端口状态。
 * @param 无。
 * @return 无。
 * @details 读取PB5/PB6/PB7初始电平并清零边沿计数；GPIO和EXINT配置仍由WorkBench生成代码完成。
 */
void motor_hall_port_init(void);

/**
 * @brief 在Hall外部中断中捕获最新三路电平。
 * @param 无。
 * @return 无。
 * @details 同时读取HA/HB/HC，生成3位状态，记录DWT时间戳并递增统一边沿计数。
 * @note 仅供EXINT9_5_IRQHandler调用，函数不清除EXINT标志。
 */
void motor_hall_port_edge_capture(void);

/**
 * @brief 原子读取最近一次原始Hall采样快照。
 * @param sample 输出原始组合状态、边沿计数和时间戳，不允许为空。
 * @return 参数有效时返回true，传入NULL时返回false。
 */
bool motor_hall_port_sample_read(motor_hall_port_sample_t *sample);

#endif /* MOTOR_HALL_PORT_H */
