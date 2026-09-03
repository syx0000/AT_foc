#ifndef MOTOR_PWM_PORT_H
#define MOTOR_PWM_PORT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint16_t phase_a; /**< A相TMR1 CH1比较值，有效范围0..9599。 */
  uint16_t phase_b; /**< B相TMR1 CH2比较值，有效范围0..9599。 */
  uint16_t phase_c; /**< C相TMR1 CH3比较值，有效范围0..9599。 */
} motor_pwm_compare_t;

/**
 * @brief 将电机功率级置于初始化安全状态。
 * @param 无。
 * @return 无。
 * @details 拉低EN_GATE、关闭TMR1主输出MOE、清零三相比较值，并使之前的PWM命令失效。
 *          TMR1计数器保持运行，确保10 kHz ADC触发不被中断。
 * @note 必须在wk_tmr1_init()之后立即调用一次。
 */
void motor_pwm_port_init(void);

/**
 * @brief 控制外部门极驱动器使能引脚。
 * @param enable true表示拉高EN_GATE；false表示拉低EN_GATE。
 * @return 无。
 * @details 关闭门极驱动器时同步关闭TMR1 MOE；开启时只唤醒驱动器，不会开启PWM输出。
 * @note 拉高EN_GATE后，调用者必须等待驱动器唤醒时间并检查故障输入，之后才能调用
 *       motor_pwm_port_output_enable()。
 */
void motor_pwm_port_gate_driver_set(bool enable);

/**
 * @brief 检查门极驱动器故障输入并清除TMR1历史Break标志。
 * @param 无。
 * @return nFAULT为高且BIF清除成功返回true；故障仍有效或BIF重新置位返回false。
 * @details 先读取PB12的低有效nFAULT；只有输入恢复为高才清除TMR_BRK_FLAG，随后立即
 *          回读BIF，防止把持续故障误判为历史锁存。
 * @note 调用者应在EN_GATE拉高并等待DRV8353唤醒至少2 ms后调用。本函数不会开启PWM。
 */
bool motor_pwm_port_fault_clear(void);

/**
 * @brief 开启TMR1三相互补PWM输出。
 * @param 无。
 * @return 成功开启MOE返回true；安全条件未满足返回false。
 * @details 只有门极驱动器已使能、三相比较命令有效、nFAULT无效且BIF已清除时，
 *          才允许开启输出。开启MOE前会先使能PB12低有效硬件Break并再次检查故障。
 */
bool motor_pwm_port_output_enable(void);

/**
 * @brief 关闭PWM输出，但不关闭门极驱动器。
 * @param 无。
 * @return 无。
 * @details 关闭TMR1 MOE，保持EN_GATE和TMR1计数器不变；已写入的比较命令仍然有效，
 *          可用于受控恢复输出。
 */
void motor_pwm_port_output_disable(void);

/**
 * @brief 立即将整个电机功率级置于安全状态。
 * @param 无。
 * @return 无。
 * @details 拉低EN_GATE、关闭TMR1 MOE、清零三相比较值并使旧命令失效。再次启动PWM前，
 *          必须重新写入比较命令并重新执行显式使能流程。
 */
void motor_pwm_port_emergency_stop(void);

/**
 * @brief 写入A、B、C三相原始PWM比较值。
 * @param compare 三相比较命令指针；传入NULL时忽略本次调用。
 * @return 无。
 * @details 每相输入均限制在0..MOTOR_PWM_COMPARE_MAX，并依次写入TMR1 CH1/CH2/CH3；
 *          成功写入后将PWM命令标记为有效。
 * @note 本函数只写比较值，不会开启EN_GATE或MOE。
 */
void motor_pwm_port_compare_set(const motor_pwm_compare_t *compare);

/**
 * @brief 获取软件记录的PWM输出状态。
 * @param 无。
 * @return 本模块已开启MOE返回true，否则返回false。
 * @note 返回值是模块内部状态，不是TMR1寄存器或外部门极驱动器故障状态的直接回读值。
 */
bool motor_pwm_port_output_is_enabled(void);

#endif /* MOTOR_PWM_PORT_H */
