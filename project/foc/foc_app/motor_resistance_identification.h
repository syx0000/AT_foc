/**
 * @file    motor_resistance_identification.h
 * @brief   电机相电阻辨识 公共数据结构和接口声明
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

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
  uint16_t applied_voltage_mv;     /**< 高电流点指令电压，单位mV。 */
  int32_t phase_a_average_ma;      /**< 高电流点A相平均电流，单位mA。 */
  int32_t direct_average_ma;       /**< 同phase_a_average_ma，保留兼容。 */
  uint16_t low_voltage_mv;         /**< 低电流点指令电压，单位mV。 */
  int32_t low_current_ma;          /**< 低电流点A相平均电流，单位mA。 */
  int32_t offset_voltage_mv;       /**< 两点直线截距：死区与零偏等效压降，单位mV。 */
  uint32_t resistance_via_phase_a_mohm; /**< 旧单点法V/I结果，含死区误差，仅对照。 */
  uint32_t resistance_via_direct_mohm;  /**< 两点差分法结果。 */
  uint32_t resistance_average_mohm;     /**< 最终采用值，等于两点差分结果。 */
  uint32_t sample_count;           /**< 两个工作点累计测量样本数。 */
} motor_resistance_identification_result_t;

/**
 * @brief 阻塞执行一次静止转子相电阻辨识。
 * @param result 输出两点电压电流、电阻和状态，不允许为空。
 * @return 辨识完成且电流有效时返回true；安全检查或测量失败时返回false。
 * @details 固定电角度0度、Uq=0，Ud以1 mV/ms先升至低电流点、再升至高电流点
 *          （上限见MOTOR_RESISTANCE_IDENT_MAX_VOLTAGE_MV），每点稳定后以1 ms
 *          周期测量。Rs=(V2-V1)/(I2-I1)，同方向两点的死区压降和电流零偏相减
 *          抵消。运行期间中断和硬件Break保持有效，任一相达到中止电流或驱动器
 *          故障会立即关闭PWM。函数返回前始终关闭PWM。
 */
bool motor_resistance_identification_run(
  motor_resistance_identification_result_t *result);

#endif /* MOTOR_RESISTANCE_IDENTIFICATION_H */
