#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <stdbool.h>
#include <stdint.h>
#include "motor_current_control.h"
#include "motor_open_loop.h"
typedef enum {
  MOTOR_CONTROL_STATE_STARTUP = 0,
  MOTOR_CONTROL_STATE_READY,
  MOTOR_CONTROL_STATE_OPEN_LOOP,
  MOTOR_CONTROL_STATE_CURRENT_CONTROL,
  MOTOR_CONTROL_STATE_SPEED_CONTROL,
  MOTOR_CONTROL_STATE_FAULT
} motor_control_state_t;
typedef struct {
  motor_control_state_t state;
  uint32_t fault_code;
} motor_control_status_t;
/** @brief 初始化统一电机状态机为STARTUP。 @param 无。 @return 无。 */
void motor_control_init(void);
/** @brief 启动校准成功后进入READY。 @param 无。 @return 成功返回true。 */
bool motor_control_ready_set(void);
/** @brief 从READY启动正式开环。 @param command 开环指令。 @return 成功返回true。 */
bool motor_control_open_loop_start(const motor_open_loop_command_t *command);
/** @brief 更新运行中的开环指令。 @param command 新指令。 @return 成功返回true。 */
bool motor_control_open_loop_command_set(const motor_open_loop_command_t *command);
/** @brief 从运行中的开环无扰切换到Hall电流闭环。 @param command Id/Iq指令。 @return 成功返回true。 */
bool motor_control_current_control_start(
  const motor_current_control_command_t *command);
/** @brief 更新运行中的Id/Iq指令。 @param command 新指令。 @return 成功返回true。 */
bool motor_control_current_control_command_set(
  const motor_current_control_command_t *command);
/**
 * @brief 从运行中的正向开环无扰切换到Hall速度闭环。
 * @param target_speed_rpm 正向目标机械转速，单位rpm。
 * @return 开环、Hall反馈、电流环及目标均有效时返回true。
 */
bool motor_control_speed_control_start(int32_t target_speed_rpm);
/**
 * @brief 更新运行中的速度闭环目标。
 * @param target_speed_rpm 新的正向目标机械转速，单位rpm。
 * @return 速度环正在运行且目标有效时返回true。
 */
bool motor_control_speed_control_target_set(int32_t target_speed_rpm);
/** @brief 安全停止并返回READY。 @param 无。 @return 无。 */
void motor_control_stop(void);
/** @brief 锁存故障并紧急关闭PWM。 @param fault_code 非零故障码。 @return 无。 */
void motor_control_fault_set(uint32_t fault_code);
/** @brief 安全条件恢复后清除故障并进入READY。 @param 无。 @return 成功返回true。 */
bool motor_control_fault_clear(void);
/** @brief 轮询底层控制器状态并同步故障。 @param 无。 @return 无。 */
void motor_control_poll(void);
/** @brief 读取统一状态。 @param status 输出状态。 @return 参数有效返回true。 */
bool motor_control_status_read(motor_control_status_t *status);
#endif
