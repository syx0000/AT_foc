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
  motor_open_loop_state_t state;
  uint16_t electrical_angle_u16;
  uint32_t electrical_frequency_millihz;
  uint16_t duty_a_q15;
  uint16_t duty_b_q15;
  uint16_t duty_c_q15;
} motor_open_loop_status_t;

/**
 * @brief 初始化开环电压控制器。
 * @param 无。
 * @return 无。
 * @details 清除角度、频率和占空比状态；不会操作门极驱动器或PWM输出。
 */
void motor_open_loop_init(void);

/**
 * @brief 启动低电压开环试转流程。
 * @param 无。
 * @return 电流校准有效、无过流且PWM安全条件满足时返回true，否则返回false。
 * @details 先施加固定d轴电压进行转子预定位，然后自动切换到q轴旋转电压。
 *          本函数会实际开启PWM，调用前必须确保电机架空且电源已限流。
 */
bool motor_open_loop_start(void);

/**
 * @brief 停止开环试转。
 * @param 无。
 * @return 无。
 * @details 关闭TMR1主输出并清除运行状态，门极驱动器保持唤醒。
 */
void motor_open_loop_stop(void);

/**
 * @brief 执行一次10 kHz开环控制更新。
 * @param 无。
 * @return 无。
 * @details 在快速ADC中断中调用；完成预定位计时、电频率线性爬升、反Park、
 *          SVPWM及三相比较值写入。未启动或故障状态下不写PWM。
 */
void motor_open_loop_fast_process(void);

/**
 * @brief 原子读取开环控制器状态快照。
 * @param status 输出状态、角度、频率和最近占空比，不允许为空。
 * @return 参数有效时返回true，传入NULL时返回false。
 */
bool motor_open_loop_status_read(motor_open_loop_status_t *status);

#endif /* MOTOR_OPEN_LOOP_H */
