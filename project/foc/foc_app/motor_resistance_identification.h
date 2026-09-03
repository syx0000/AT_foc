#ifndef MOTOR_RESISTANCE_IDENTIFICATION_H
#define MOTOR_RESISTANCE_IDENTIFICATION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  MOTOR_RESISTANCE_IDENT_OK = 0,
  MOTOR_RESISTANCE_IDENT_INVALID_STATE,
  MOTOR_RESISTANCE_IDENT_BUS_VOLTAGE_INVALID,
  MOTOR_RESISTANCE_IDENT_PWM_ENABLE_FAILED,
  MOTOR_RESISTANCE_IDENT_OVERCURRENT,
  MOTOR_RESISTANCE_IDENT_DRIVER_FAULT,
  MOTOR_RESISTANCE_IDENT_CURRENT_TOO_LOW
} motor_resistance_identification_status_t;

typedef struct
{
  motor_resistance_identification_status_t status;
  uint16_t applied_voltage_mv;
  int32_t phase_a_average_ma;
  int32_t direct_average_ma;
  uint32_t resistance_via_phase_a_mohm;
  uint32_t resistance_via_direct_mohm;
  uint32_t resistance_average_mohm;
  uint32_t sample_count;
} motor_resistance_identification_result_t;

/**
 * @brief 阻塞执行一次静止转子相电阻辨识。
 * @param result 输出施加电压、平均电流、电阻和状态，不允许为空。
 * @return 辨识完成且电流有效时返回true；安全检查或测量失败时返回false。
 * @details 固定电角度0度、Uq=0，Ud以1 mV/ms升至3 A目标或1 V上限；
 *          稳定200 ms后以1 ms周期测量1秒。运行期间中断和硬件Break保持有效，
 *          任一相达到10 A或驱动器故障会立即关闭PWM。函数返回前始终关闭PWM。
 */
bool motor_resistance_identification_run(
  motor_resistance_identification_result_t *result);

#endif /* MOTOR_RESISTANCE_IDENTIFICATION_H */
