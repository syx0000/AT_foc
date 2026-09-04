#ifndef MOTOR_SPEED_CONTROL_H
#define MOTOR_SPEED_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  MOTOR_SPEED_CONTROL_STOPPED = 0,
  MOTOR_SPEED_CONTROL_RUNNING,
  MOTOR_SPEED_CONTROL_FAULT
} motor_speed_control_state_t;

typedef enum
{
  MOTOR_SPEED_CONTROL_FAULT_NONE = 0,
  MOTOR_SPEED_CONTROL_FAULT_FEEDBACK,
  MOTOR_SPEED_CONTROL_FAULT_CURRENT_CONTROL,
  MOTOR_SPEED_CONTROL_FAULT_COMMAND,
  MOTOR_SPEED_CONTROL_FAULT_REVERSE_DIRECTION,
  MOTOR_SPEED_CONTROL_FAULT_STALL,
  MOTOR_SPEED_CONTROL_FAULT_OVERSPEED
} motor_speed_control_fault_t;

typedef struct
{
  int32_t proportional_gain_q20;
  int32_t integral_gain_q20;
  int32_t current_limit_ma;
  int32_t maximum_speed_rpm;
  uint32_t acceleration_rpm_per_s;
} motor_speed_control_config_t;

typedef struct
{
  motor_speed_control_state_t state;
  motor_speed_control_fault_t fault;
  int32_t target_speed_millirpm;
  int32_t ramped_speed_millirpm;
  int32_t feedback_speed_millirpm;
  int32_t quadrature_current_command_ma; /**< 速度PI输出的逻辑Iq指令，单位mA。 */
  uint32_t stall_time_ms;
  uint32_t update_count;
} motor_speed_control_status_t;

/**
 * @brief 使用motor_control_config.h默认参数初始化1 kHz速度控制器。
 * @param 无。
 * @return 无。
 * @details 初始化速度PI、速度斜坡和状态，不操作电流环与PWM。
 */
void motor_speed_control_init(void);

/**
 * @brief 在停止状态更新速度PI、Iq限幅、最高转速和加速度配置。
 * @param config 新配置，不允许为空。
 * @return 参数有效且控制器已停止时返回true，否则返回false。
 */
bool motor_speed_control_config_set(
  const motor_speed_control_config_t *config);

/**
 * @brief 从已经运行的Hall电流环启动速度外环。
 * @param target_speed_millirpm 有符号逻辑目标机械转速，单位0.001 rpm。
 * @param initial_quadrature_current_ma 接管前物理Iq指令，单位mA，用于PI无扰预置。
 * @return 电流环与速度反馈有效、目标范围和当前反馈方向一致时返回true，否则返回false。
 */
bool motor_speed_control_start(int32_t target_speed_millirpm,
                               int32_t initial_quadrature_current_ma);

/**
 * @brief 更新运行中的目标机械转速。
 * @param target_speed_millirpm 新的有符号逻辑目标机械转速，单位0.001 rpm且不能为0。
 * @return 控制器运行、目标有效且不要求在线跨零反转时返回true，否则返回false。
 */
bool motor_speed_control_target_set(int32_t target_speed_millirpm);

/**
 * @brief 停止速度外环并清零PI，不直接关闭电流环或PWM。
 * @param 无。
 * @return 无。
 */
void motor_speed_control_stop(void);

/**
 * @brief 执行一次1 kHz速度外环。
 * @param 无。
 * @return 无。
 * @details 推进逻辑目标速度斜坡，经速度PI生成逻辑Iq，再按方向配置转换为物理Iq写入10 kHz电流环；
 *          速度反馈或电流环失效时进入FAULT并停止电流控制。
 * @note 仅供1 kHz周期中断调用。
 */
void motor_speed_control_process_1khz(void);

/**
 * @brief 原子读取速度控制状态。
 * @param status 输出目标/斜坡/反馈转速、Iq指令、状态和故障，不允许为空。
 * @return 参数有效时返回true；传入NULL时返回false。
 */
bool motor_speed_control_status_read(motor_speed_control_status_t *status);

#endif /* MOTOR_SPEED_CONTROL_H */
