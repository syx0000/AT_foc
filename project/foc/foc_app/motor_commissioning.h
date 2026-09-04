#ifndef MOTOR_COMMISSIONING_H
#define MOTOR_COMMISSIONING_H
#include <stdbool.h>
#include <stdint.h>

typedef enum {
  MOTOR_COMMISSIONING_TASK_NONE = 0,
  MOTOR_COMMISSIONING_TASK_CURRENT_OFFSET,
  MOTOR_COMMISSIONING_TASK_RESISTANCE,
  MOTOR_COMMISSIONING_TASK_INDUCTANCE,
  MOTOR_COMMISSIONING_TASK_HALL,
  MOTOR_COMMISSIONING_TASK_HALL_OFFSET,
  MOTOR_COMMISSIONING_TASK_CURRENT_PI,
  MOTOR_COMMISSIONING_TASK_CURRENT_D,
  MOTOR_COMMISSIONING_TASK_CURRENT_Q,
  MOTOR_COMMISSIONING_TASK_CURRENT_HANDOVER,
  MOTOR_COMMISSIONING_TASK_FULL
} motor_commissioning_task_t;
typedef enum {
  MOTOR_COMMISSIONING_IDLE = 0,
  MOTOR_COMMISSIONING_RUNNING,
  MOTOR_COMMISSIONING_WAIT_ACCEPT,
  MOTOR_COMMISSIONING_COMPLETE,
  MOTOR_COMMISSIONING_FAULT
} motor_commissioning_state_t;

typedef enum
{
  MOTOR_COMMISSIONING_ERROR_NONE = 0,
  MOTOR_COMMISSIONING_ERROR_ABORTED,
  MOTOR_COMMISSIONING_ERROR_CURRENT_OFFSET,
  MOTOR_COMMISSIONING_ERROR_RESISTANCE,
  MOTOR_COMMISSIONING_ERROR_INDUCTANCE,
  MOTOR_COMMISSIONING_ERROR_HALL_SCAN,
  MOTOR_COMMISSIONING_ERROR_HALL_OFFSET,
  MOTOR_COMMISSIONING_ERROR_CURRENT_PI,
  MOTOR_COMMISSIONING_ERROR_CURRENT_D,
  MOTOR_COMMISSIONING_ERROR_CURRENT_Q,
  MOTOR_COMMISSIONING_ERROR_PARAMETER
} motor_commissioning_error_t;

typedef struct {
  motor_commissioning_state_t state;
  motor_commissioning_task_t task;
  uint32_t step;
  motor_commissioning_error_t error;
  uint32_t error_detail;
  uint32_t run_count;
} motor_commissioning_status_t;

/** @brief 初始化统一辨识任务状态。 @param 无。 @return 无。 */
void motor_commissioning_init(void);
/** @brief 阻塞执行完整流程或一个辨识/测试单项。 @param task 任务类型。 @return 成功返回true。 */
bool motor_commissioning_run(motor_commissioning_task_t task);
/** @brief 在Hall外部中断中提交边沿状态和开环角度。 @param 无。 @return 无。 */
void motor_commissioning_hall_edge_process(void);
/** @brief 请求中止任务；阻塞步骤之间生效并立即关闭PWM。 @param 无。 @return 有任务时返回true。 */
bool motor_commissioning_abort(void);
/** @brief 原子读取任务状态。 @param status 输出状态。 @return 参数有效返回true。 */
bool motor_commissioning_status_read(motor_commissioning_status_t *status);

/**
 * @brief 获取自动辨识阶段错误的可读名称。
 * @param error 自动辨识阶段错误码。
 * @return 静态只读名称字符串。
 */
const char *motor_commissioning_error_name_get(
  motor_commissioning_error_t error);

/**
 * @brief 按失败阶段解释底层详细错误码。
 * @param error 自动辨识阶段错误码。
 * @param detail 对应辨识或验证模块返回的原始状态码。
 * @return 静态只读名称字符串；未知组合返回unknown。
 */
const char *motor_commissioning_error_detail_name_get(
  motor_commissioning_error_t error, uint32_t detail);

/**
 * @brief 通知辨识管理器候选参数已经接受或丢弃。
 * @param accepted true表示候选参数已接受，false表示候选参数已丢弃。
 * @return 当前处于待确认状态时返回true，否则不改变状态并返回false。
 * @details 接受后状态进入COMPLETE；丢弃后恢复IDLE，便于重新执行单项辨识。
 */
bool motor_commissioning_review_complete(bool accepted);
#endif
