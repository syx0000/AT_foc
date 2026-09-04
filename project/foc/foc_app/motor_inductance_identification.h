#ifndef MOTOR_INDUCTANCE_IDENTIFICATION_H
#define MOTOR_INDUCTANCE_IDENTIFICATION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  MOTOR_INDUCTANCE_IDENT_OK = 0,
  MOTOR_INDUCTANCE_IDENT_INVALID_ARGUMENT,
  MOTOR_INDUCTANCE_IDENT_INVALID_STATE,
  MOTOR_INDUCTANCE_IDENT_BUS_VOLTAGE_INVALID,
  MOTOR_INDUCTANCE_IDENT_PWM_ENABLE_FAILED,
  MOTOR_INDUCTANCE_IDENT_OVERCURRENT,
  MOTOR_INDUCTANCE_IDENT_DRIVER_FAULT,
  MOTOR_INDUCTANCE_IDENT_CURRENT_TOO_LOW,
  MOTOR_INDUCTANCE_IDENT_IMPEDANCE_INVALID
} motor_inductance_identification_status_t;

typedef struct
{
  motor_inductance_identification_status_t status;
  uint32_t direct_inductance_uh;
  uint32_t quadrature_inductance_uh;
  uint32_t direct_current_amplitude_ma;
  uint32_t quadrature_current_amplitude_ma;
  uint16_t injection_voltage_mv;
} motor_inductance_identification_result_t;

/**
 * @brief 阻塞执行600 Hz交流注入，辨识d、q轴电感。
 * @param phase_resistance_mohm 已辨识的单相电阻，单位mΩ。
 * @param result 输出Ld、Lq、电流幅值和实际注入电压，不允许为空。
 * @return 两轴测量均有效时返回true；状态、电流或功率级异常时返回false。
 * @details 每轴先等待200 ms，再以10 kHz同步检波1秒；注入幅值0.5 V，
 *          任一相达到10 A立即关闭PWM。函数返回前始终关闭PWM。
 */
bool motor_inductance_identification_run(
  uint32_t phase_resistance_mohm,
  motor_inductance_identification_result_t *result);

/**
 * @brief 执行一次10 kHz交流注入和同步检波。
 * @param 无。
 * @return 无。
 * @details 仅在阻塞辨识接口激活期间工作，由快速ADC中断调用。
 */
void motor_inductance_identification_fast_process(void);

#endif
