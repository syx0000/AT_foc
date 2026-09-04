#ifndef MOTOR_BOARD_CONFIG_H
#define MOTOR_BOARD_CONFIG_H

/* 板级电机硬件映射集中配置；修改前必须与原理图及WorkBench外设配置同步核对。 */

#include "at32f45x.h"
#include "at32f45x_wk_config.h"

/* TMR1时钟192 MHz，中心对齐，ARR=9599，对应PWM频率10 kHz。 */
#define MOTOR_PWM_TIMER                    TMR1
#define MOTOR_PWM_PERIOD_COUNTS            9600U
#define MOTOR_PWM_COMPARE_MAX              (MOTOR_PWM_PERIOD_COUNTS - 1U)
#define MOTOR_PWM_FREQUENCY_HZ             10000U
#define MOTOR_PWM_DEADTIME_COUNTS           80U

/* 三相互补输出映射，来源于当前WorkBench配置。 */
#define MOTOR_PWM_PHASE_A_HIGH_PORT         GPIOA
#define MOTOR_PWM_PHASE_A_HIGH_PIN          GPIO_PINS_8
#define MOTOR_PWM_PHASE_A_LOW_PORT          GPIOA
#define MOTOR_PWM_PHASE_A_LOW_PIN           GPIO_PINS_7
#define MOTOR_PWM_PHASE_B_HIGH_PORT         GPIOA
#define MOTOR_PWM_PHASE_B_HIGH_PIN          GPIO_PINS_9
#define MOTOR_PWM_PHASE_B_LOW_PORT          GPIOB
#define MOTOR_PWM_PHASE_B_LOW_PIN           GPIO_PINS_0
#define MOTOR_PWM_PHASE_C_HIGH_PORT         GPIOA
#define MOTOR_PWM_PHASE_C_HIGH_PIN          GPIO_PINS_10
#define MOTOR_PWM_PHASE_C_LOW_PORT          GPIOB
#define MOTOR_PWM_PHASE_C_LOW_PIN           GPIO_PINS_1

/* DRV8353使能：PC15，高电平有效。 */
#define MOTOR_GATE_ENABLE_PORT              EN_GATE_GPIO_PORT
#define MOTOR_GATE_ENABLE_PIN               EN_GATE_PIN
#define MOTOR_GATE_ENABLE_ACTIVE_HIGH       1U
#define MOTOR_GATE_DRIVER_WAKE_DELAY_MS      2U
/* TMR1硬件刹车输入：PB12，低电平有效；在PWM输出使能前由端口层开启。 */
#define MOTOR_PWM_BREAK_PORT                GPIOB
#define MOTOR_PWM_BREAK_PIN                 GPIO_PINS_12
#define MOTOR_PWM_BREAK_ACTIVE_LOW          1U

#endif /* MOTOR_BOARD_CONFIG_H */
