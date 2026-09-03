#ifndef MOTOR_SLOW_SENSOR_H
#define MOTOR_SLOW_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint16_t bus_voltage_0p1v; /**< 母线电压，单位0.1 V。 */
  int16_t motor_temperature_c; /**< 电机绕组温度，单位°C。 */
  int16_t mos_temperature_c; /**< MOS功率板温度，单位°C。 */
  bool motor_temperature_valid; /**< 电机NTC输入及换算有效。 */
  bool mos_temperature_valid; /**< MOS NTC输入及换算有效。 */
  uint32_t sample_count; /**< 最近处理的慢速ADC样本序号。 */
} motor_slow_sensor_state_t;

/**
 * @brief 初始化慢速ADC业务换算状态。
 * @param 无。
 * @return 无。
 */
void motor_slow_sensor_init(void);

/**
 * @brief 处理最近一次慢速ADC快照。
 * @param 无。
 * @return true表示处理了一个新DMA样本；没有新样本或快照无效时返回false。
 * @details 将SO3按21:1分压换算为0.1 V，将两路10 kΩ NTC通过5°C
 *          查表和线性插值换算为摄氏度。该函数应由主循环调用，不在DMA中断
 *          内执行。
 */
bool motor_slow_sensor_process(void);

/**
 * @brief 读取最近一次慢速传感器业务值。
 * @param state 输出母线电压、温度及有效性，不允许为空。
 * @return true表示至少处理过一个慢速ADC样本；否则返回false。
 */
bool motor_slow_sensor_state_read(motor_slow_sensor_state_t *state);

#endif /* MOTOR_SLOW_SENSOR_H */
