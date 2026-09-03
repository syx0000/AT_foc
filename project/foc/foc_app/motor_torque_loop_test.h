#ifndef MOTOR_TORQUE_LOOP_TEST_H
#define MOTOR_TORQUE_LOOP_TEST_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  MOTOR_TORQUE_TEST_STATUS_OK = 0,
  MOTOR_TORQUE_TEST_STATUS_INVALID_ARGUMENT,
  MOTOR_TORQUE_TEST_STATUS_PWM_BUSY,
  MOTOR_TORQUE_TEST_STATUS_OPEN_LOOP_START_FAILED,
  MOTOR_TORQUE_TEST_STATUS_BOOTSTRAP_FAILED,
  MOTOR_TORQUE_TEST_STATUS_BOOTSTRAP_TIMEOUT,
  MOTOR_TORQUE_TEST_STATUS_SENSOR_INVALID,
  MOTOR_TORQUE_TEST_STATUS_HALL_LOST,
  MOTOR_TORQUE_TEST_STATUS_OVERCURRENT,
  MOTOR_TORQUE_TEST_STATUS_PWM_FAULT
} motor_torque_loop_test_status_t;

typedef struct
{
  motor_torque_loop_test_status_t status;
  int32_t direct_average_ma;
  int32_t quadrature_average_ma;
  int32_t direct_peak_ma;
  int32_t quadrature_peak_ma;
  int32_t final_direct_voltage_mv;
  int32_t final_quadrature_voltage_mv;
  int32_t direct_voltage_average_mv;
  int32_t quadrature_voltage_average_mv;
  uint32_t final_frequency_millihz;
  int32_t final_phase_a_ma;
  int32_t final_phase_b_ma;
  int32_t final_phase_c_ma;
  uint8_t final_hall_state;
  uint32_t sample_count;
} motor_torque_loop_test_result_t;

/**
 * @brief 阻塞执行开环起转并切换至Hall角度电流闭环的低转矩测试。
 * @param result 输出状态、dq电流、稳定段平均/最终电压、Hall频率及故障快照。
 * @return 完成闭环测试返回true；起转超时、Hall失效、过流或PWM故障返回false。
 * @details 先复用开环流程建立转速，Hall估算达到8 Hz后切换为Id=0、Iq=1 A
 *          的闭环控制；接管电流在200 ms内平滑过渡到目标，统计不包含过渡段。
 *          输出电压限制为母线电压的15%，退出前始终关闭PWM。
 */
bool motor_torque_loop_test_run(motor_torque_loop_test_result_t *result);

/**
 * @brief 执行一次10 kHz Hall角度转矩电流环更新。
 * @param 无。
 * @return 无。
 * @details 仅在阻塞测试进入闭环阶段后运行，由快速ADC中断调用；每次完成
 *          电流Clarke/Park、dq双PI、反Park、SVPWM及安全检查。
 */
void motor_torque_loop_test_fast_process(void);

#endif /* MOTOR_TORQUE_LOOP_TEST_H */
