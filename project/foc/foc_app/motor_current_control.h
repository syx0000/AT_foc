#ifndef MOTOR_CURRENT_CONTROL_H
#define MOTOR_CURRENT_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  MOTOR_CURRENT_CONTROL_STOPPED = 0,
  MOTOR_CURRENT_CONTROL_RUNNING,
  MOTOR_CURRENT_CONTROL_FAULT
} motor_current_control_state_t;

typedef enum
{
  MOTOR_CURRENT_CONTROL_FAULT_NONE = 0,
  MOTOR_CURRENT_CONTROL_FAULT_SAMPLE,
  MOTOR_CURRENT_CONTROL_FAULT_HALL,
  MOTOR_CURRENT_CONTROL_FAULT_OVERCURRENT,
  MOTOR_CURRENT_CONTROL_FAULT_PWM
} motor_current_control_fault_t;

typedef struct
{
  int32_t direct_kp_q15;
  int32_t quadrature_kp_q15;
  int32_t integral_gain_q15;
  uint32_t voltage_limit_percent;
  int32_t absolute_voltage_limit_mv;
  int32_t command_limit_ma;
  int32_t abort_current_ma;
} motor_current_control_config_t;

typedef struct
{
  int32_t direct_reference_ma;
  int32_t quadrature_reference_ma;
} motor_current_control_command_t;

typedef struct
{
  motor_current_control_state_t state;
  motor_current_control_fault_t fault;
  motor_current_control_command_t command;
  int32_t direct_feedback_ma;
  int32_t quadrature_feedback_ma;
  int32_t direct_voltage_mv;
  int32_t quadrature_voltage_mv;
  uint16_t electrical_angle_u16;
  uint32_t electrical_frequency_millihz;
  uint32_t sample_count;
  bool voltage_limited;
} motor_current_control_status_t;

/** @brief 使用motor_control_config.h默认参数初始化。 @param 无。 @return 无。 */
void motor_current_control_init(void);

/**
 * @brief 在停止状态更新PI、限压与电流保护配置。
 * @param config 控制器配置，不允许为空。
 * @return 参数有效且控制器已停止时返回true，否则返回false。
 */
bool motor_current_control_config_set(
  const motor_current_control_config_t *config);

/**
 * @brief 从PWM关闭状态以零初始电压启动Hall角度电流闭环。
 * @param command 初始Id/Iq指令，单位mA，不允许为空。
 * @return Hall、母线、指令与PWM安全条件均有效时返回true。
 */
bool motor_current_control_start(const motor_current_control_command_t *command);

/**
 * @brief 从其他控制模式无扰接管已经开启的PWM。
 * @param command 接管瞬间Id/Iq指令，单位mA，不允许为空。
 * @param direct_voltage_seed_mv 接管前实际d轴电压，单位mV。
 * @param quadrature_voltage_seed_mv 接管前实际q轴电压，单位mV。
 * @return Hall、母线、指令与PWM状态有效时返回true。
 */
bool motor_current_control_handover(
  const motor_current_control_command_t *command,
  int32_t direct_voltage_seed_mv,
  int32_t quadrature_voltage_seed_mv);

/**
 * @brief 原子更新运行中的Id/Iq指令。
 * @param command 新Id/Iq指令，单位mA，不允许为空。
 * @return 指令有效且控制器正在运行时返回true。
 */
bool motor_current_control_command_set(
  const motor_current_control_command_t *command);

/** @brief 关闭PWM并停止控制器。 @param 无。 @return 无。 */
void motor_current_control_stop(void);

/**
 * @brief 执行一次10 kHz电流闭环。
 * @param 无。
 * @return 无。
 * @details 完成Clarke/Park、双PI、联合限压、反Park及SVPWM；异常时紧急停机。
 */
void motor_current_control_fast_process(void);

/**
 * @brief 原子读取控制状态。
 * @param status 输出指令、反馈、电压、角度、频率及故障，不允许为空。
 * @return 参数有效时返回true，否则返回false。
 */
bool motor_current_control_status_read(motor_current_control_status_t *status);

#endif /* MOTOR_CURRENT_CONTROL_H */
