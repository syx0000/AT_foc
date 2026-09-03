#ifndef MOTOR_OPEN_LOOP_H
#define MOTOR_OPEN_LOOP_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  MOTOR_OPEN_LOOP_STOPPED = 0,
  MOTOR_OPEN_LOOP_ALIGNING,
  MOTOR_OPEN_LOOP_RUNNING,
  MOTOR_OPEN_LOOP_FAULT
} motor_open_loop_state_t;

typedef struct
{
  int32_t maximum_voltage_mv;             /**< dq合成电压上限，单位mV。 */
  int32_t maximum_frequency_millihz;      /**< 正反向电频率绝对值上限，单位mHz。 */
  int32_t start_frequency_millihz;        /**< 对齐结束后的起始频率绝对值，单位mHz。 */
  int32_t alignment_voltage_mv;           /**< 转子预定位d轴电压，单位mV。 */
  uint16_t alignment_angle_u16;           /**< 转子预定位电角度，U16一周制。 */
  uint32_t alignment_time_ms;             /**< 转子预定位时间，单位ms。 */
} motor_open_loop_config_t;

typedef struct
{
  int32_t direct_voltage_mv;              /**< 运行d轴电压指令，单位mV。 */
  int32_t quadrature_voltage_mv;          /**< 运行q轴电压指令，单位mV。 */
  int32_t target_frequency_millihz;       /**< 目标电频率，负值表示反转，单位mHz。 */
  uint32_t acceleration_millihz_per_s;    /**< 电频率变化率，单位mHz/s。 */
} motor_open_loop_command_t;

typedef struct
{
  motor_open_loop_state_t state;
  uint16_t electrical_angle_u16;
  int32_t target_frequency_millihz;
  int32_t actual_frequency_millihz;
  int32_t applied_direct_voltage_mv;
  int32_t applied_quadrature_voltage_mv;
  uint16_t duty_a_q15;
  uint16_t duty_b_q15;
  uint16_t duty_c_q15;
  bool voltage_limited;
} motor_open_loop_status_t;

/**
 * @brief 使用板级默认安全参数初始化开环控制器。
 * @param 无。
 * @return 无。
 * @details 清除控制状态和运行指令，不操作PWM输出。
 */
void motor_open_loop_init(void);

/**
 * @brief 更新开环安全配置。
 * @param config 最大电压/频率、起始频率和预定位参数，不允许为空。
 * @return 参数有效且控制器处于停止状态时返回true，否则返回false。
 */
bool motor_open_loop_config_set(const motor_open_loop_config_t *config);

/**
 * @brief 按运行指令启动开环控制。
 * @param command dq电压、目标电频率和加速度指令，不允许为空。
 * @return 指令及硬件状态满足安全条件并成功开启PWM时返回true。
 */
bool motor_open_loop_start(const motor_open_loop_command_t *command);

/**
 * @brief 运行中更新开环指令。
 * @param command 新的dq电压、目标电频率和加速度指令，不允许为空。
 * @return 指令有效且控制器正在对齐或运行时返回true，否则返回false。
 */
bool motor_open_loop_command_set(const motor_open_loop_command_t *command);

/** @brief 关闭PWM并停止开环控制。 @param 无。 @return 无。 */
void motor_open_loop_stop(void);

/**
 * @brief 释放开环控制权但保持当前PWM连续输出。
 * @param 无。
 * @return 开环正在运行且PWM已开启时返回true，否则返回false。
 */
bool motor_open_loop_control_release(void);

/**
 * @brief 执行一次10 kHz开环角度、频率斜坡和PWM更新。
 * @param 无。
 * @return 无。
 */
void motor_open_loop_fast_process(void);

/**
 * @brief 原子读取开环状态。
 * @param status 输出状态、目标/实际频率、电压、角度和占空比，不允许为空。
 * @return 参数有效时返回true，否则返回false。
 */
bool motor_open_loop_status_read(motor_open_loop_status_t *status);

#endif /* MOTOR_OPEN_LOOP_H */
