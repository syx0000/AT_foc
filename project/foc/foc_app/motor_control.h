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

#define MOTOR_FAULT_HISTORY_DEPTH 5U

typedef enum
{
  MOTOR_FAULT_NONE = 0,
  MOTOR_FAULT_UNKNOWN,
  MOTOR_FAULT_CONTROL_SUBSYSTEM,
  MOTOR_FAULT_READY_TRANSITION,
  MOTOR_FAULT_CURRENT_OFFSET_INVALID,
  MOTOR_FAULT_CURRENT_CALIBRATION_TIMEOUT,
  MOTOR_FAULT_DRIVER_NOT_READY,
  MOTOR_FAULT_CURRENT_HANDOVER,
  MOTOR_FAULT_SPEED_START,
  MOTOR_FAULT_OPEN_LOOP,
  MOTOR_FAULT_CURRENT_SAMPLE,
  MOTOR_FAULT_HALL,
  MOTOR_FAULT_OVERCURRENT,
  MOTOR_FAULT_PWM,
  MOTOR_FAULT_SPEED_FEEDBACK,
  MOTOR_FAULT_SPEED_CURRENT_CONTROL,
  MOTOR_FAULT_SPEED_COMMAND,
  MOTOR_FAULT_REVERSE_DIRECTION,
  MOTOR_FAULT_STALL,
  MOTOR_FAULT_OVERSPEED,
  MOTOR_FAULT_HARDWARE_BREAK
} motor_fault_code_t;

typedef struct
{
  motor_fault_code_t first_code;
  motor_fault_code_t last_code;
  uint32_t code_mask;
  uint32_t occurrence_count;
  uint32_t sequence;
} motor_fault_record_t;

typedef enum
{
  MOTOR_FAULT_CLEAR_OK = 0,
  MOTOR_FAULT_CLEAR_NOT_FAULTED,
  MOTOR_FAULT_CLEAR_PWM_ENABLED,
  MOTOR_FAULT_CLEAR_RESET_REQUIRED,
  MOTOR_FAULT_CLEAR_CURRENT_UNSAFE,
  MOTOR_FAULT_CLEAR_DRIVER_ACTIVE
} motor_fault_clear_result_t;

typedef struct {
  motor_control_state_t state;
  motor_fault_code_t fault_code;
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
 * @brief 从同方向运行的开环无扰切换到Hall速度闭环。
 * @param target_speed_rpm 有符号逻辑目标机械转速，单位rpm。
 * @return 开环、Hall反馈、电流环、目标和物理反向安全门均有效时返回true。
 */
bool motor_control_speed_control_start(int32_t target_speed_rpm);
/**
 * @brief 更新运行中的速度闭环目标。
 * @param target_speed_rpm 新的有符号逻辑目标机械转速，单位rpm。
 * @return 速度环运行、目标有效且不要求在线跨零反转时返回true。
 */
bool motor_control_speed_control_target_set(int32_t target_speed_rpm);
/** @brief 安全停止并返回READY。 @param 无。 @return 无。 */
void motor_control_stop(void);
/** @brief 锁存故障并紧急关闭PWM。 @param fault_code 非零故障码。 @return 无。 */
void motor_control_fault_set(motor_fault_code_t fault_code);
/**
 * @brief 从中断上下文投递电机故障并立即关闭PWM。
 * @param fault_code 待投递的非零统一故障码。
 * @return 无。
 * @details 本接口不打印日志、不修改故障历史，仅原子置位待处理故障；主循环中的
 *          motor_control_poll()负责锁存、记录并打印故障。可在ADC、Break等ISR中调用。
 */
void motor_control_fault_notify_from_isr(motor_fault_code_t fault_code);
/** @brief 安全条件恢复后清除故障并进入READY。 @param 无。 @return 成功返回true。 */
bool motor_control_fault_clear(void);
/**
 * @brief 尝试清除当前故障并返回明确的恢复判定结果。
 * @param 无。
 * @return OK表示已恢复READY；启动自检故障返回RESET_REQUIRED，其他值表示安全条件未满足。
 */
motor_fault_clear_result_t motor_control_fault_clear_ex(void);
/**
 * @brief 返回故障清除结果的可读名称。
 * @param result 故障清除结果。
 * @return 静态只读名称字符串。
 */
const char *motor_control_fault_clear_result_name_get(
  motor_fault_clear_result_t result);
/**
 * @brief 返回统一电机故障码的可读名称。
 * @param fault_code 待转换的统一故障码。
 * @return 静态只读名称字符串，未知枚举返回"invalid"。
 */
const char *motor_control_fault_name_get(motor_fault_code_t fault_code);
/**
 * @brief 按从新到旧的顺序读取RAM故障历史。
 * @param newest_index 0表示最近一次，最大为4。
 * @param record 输出首错、末错、错误位图、次数和事件序号，不允许为空。
 * @return 对应历史存在时返回true，索引越界或尚无该记录返回false。
 */
bool motor_control_fault_history_read(uint32_t newest_index,
                                      motor_fault_record_t *record);
/** @brief 轮询底层控制器状态并同步故障。 @param 无。 @return 无。 */
void motor_control_poll(void);
/** @brief 读取统一状态。 @param status 输出状态。 @return 参数有效返回true。 */
bool motor_control_status_read(motor_control_status_t *status);
#endif
