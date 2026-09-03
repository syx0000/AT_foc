#ifndef MOTOR_CURRENT_CALIBRATION_H
#define MOTOR_CURRENT_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>
#include "motor_adc_port.h"

#define MOTOR_CURRENT_CALIBRATION_DEFAULT_SAMPLES 1024U
#define MOTOR_CURRENT_CALIBRATION_MAX_SAMPLES     4096U
#define MOTOR_CURRENT_CALIBRATION_MAX_TIMEOUT_MS 10000U

typedef struct
{
  uint16_t phase_a_offset_raw; /**< SOA零电流平均ADC值。 */
  uint16_t phase_b_offset_raw; /**< SOB零电流平均ADC值。 */
  uint32_t sample_count;       /**< 本次平均使用的有效采样数。 */
} motor_current_calibration_result_t;

/**
 * @brief 阻塞执行一次电流零偏校准并返回结果。
 * @param sample_count 需要累计的快速ADC样本数，范围1至4096。
 * @param timeout_ms 等待校准完成的超时时间，单位毫秒，必须大于0。
 * @param result 输出两相零偏和实际样本数，不允许为空。
 * @return true表示在超时前完成校准；false表示参数非法、已有校准正在执行或
 *         等待超时，此时result内容无效。
 * @details 函数只阻塞调用线程，10 kHz ADC中断必须保持开启并持续调用
 *          motor_current_calibration_sample_process()。超时后自动终止本次校准。
 * @note 调用前必须确认电机静止、PWM关闭，并已唤醒门极驱动器使电流放大器
 *       输出稳定。若系统启用看门狗，等待循环还需接入统一喂狗接口。
 */
bool motor_current_calibration_run(
  uint32_t sample_count,
  uint32_t timeout_ms,
  motor_current_calibration_result_t *result);

/**
 * @brief 向正在运行的校准过程加入一个快速ADC样本。
 * @param sample motor_adc_port捕获的SOA/SOB原始采样，不允许为空。
 * @return 无。
 * @details 在10 kHz ADC中断内调用，仅执行整数累加；没有校准任务时立即返回。
 */
void motor_current_calibration_sample_process(
  const motor_adc_fast_sample_t *sample);

#endif /* MOTOR_CURRENT_CALIBRATION_H */
