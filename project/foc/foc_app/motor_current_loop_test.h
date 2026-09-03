#ifndef MOTOR_CURRENT_LOOP_TEST_H
#define MOTOR_CURRENT_LOOP_TEST_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
  int32_t direct_average_ma;
  int32_t quadrature_average_ma;
  int32_t direct_peak_ma;
  int32_t quadrature_peak_ma;
  int32_t direct_voltage_mv;
  int32_t quadrature_voltage_mv;
  uint32_t sample_count;
} motor_current_loop_test_result_t;
/**
 * @brief 阻塞执行固定0度、Id=2 A、Iq=0的电流PI测试。
 * @param result 输出平均/峰值电流、最终电压和样本数，不允许为空。
 * @return 完成1秒测试返回true；状态异常、过流或PWM故障返回false。
 * @details 测试结束前始终关闭PWM；任一相达到10 A立即紧急关断。
 */
bool motor_current_loop_test_run(motor_current_loop_test_result_t *result);
/**
 * @brief 执行一次10 kHz固定角度电流PI测试更新。
 * @param 无。
 * @return 无。
 * @details 仅在阻塞测试激活时运行，由快速ADC中断调用。
 */
void motor_current_loop_test_fast_process(void);
#endif
