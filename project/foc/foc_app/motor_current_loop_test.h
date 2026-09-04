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
 * @brief 使用指定PI增益阻塞执行固定角度Id电流环测试。
 * @param result 输出平均/峰值电流、最终电压和样本数，不允许为空。
 * @param direct_kp_q15 d轴比例增益，Q15格式，必须大于0。
 * @param quadrature_kp_q15 q轴比例增益，Q15格式，必须大于0。
 * @param integral_gain_q15 d/q轴共用单步积分增益，Q15格式，必须大于0。
 * @return 测试完成且未触发安全保护返回true，否则返回false。
 * @details 供候选电机参数试运行使用；测试结束前始终关闭PWM。
 */
bool motor_current_loop_test_run_with_gains(
  motor_current_loop_test_result_t *result,
  int32_t direct_kp_q15,
  int32_t quadrature_kp_q15,
  int32_t integral_gain_q15);
/**
 * @brief 执行一次10 kHz固定角度电流PI测试更新。
 * @param 无。
 * @return 无。
 * @details 仅在阻塞测试激活时运行，由快速ADC中断调用。
 */
void motor_current_loop_test_fast_process(void);
#endif
