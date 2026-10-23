/**
 * @file    motor_board_config.h
 * @brief   电机板级硬件映射配置 公共数据结构和接口声明
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#ifndef MOTOR_BOARD_CONFIG_H
#define MOTOR_BOARD_CONFIG_H

/* 板级电机硬件映射集中配置；修改前必须与原理图及WorkBench外设配置同步核对。 */

#include "at32f45x.h"
#include "at32f45x_wk_config.h"

/* TMR1时钟192 MHz，中心对齐，ARR=5999，对应PWM频率16 kHz。 */
#define MOTOR_PWM_TIMER                    TMR1
#define MOTOR_PWM_PERIOD_COUNTS            6000U
#define MOTOR_PWM_COMPARE_MAX              (MOTOR_PWM_PERIOD_COUNTS - 1U)
#define MOTOR_PWM_FREQUENCY_HZ             16000U
#define MOTOR_PWM_DEADTIME_COUNTS           80U

/* Hall输入捕获方式（对齐demo HALL_CAPTURE_MODE）：
 * 0 = EXINT双边沿中断 + GPIO读取，无输入滤波；
 * 1 = TMR4 CH1/CH2/CH3 XOR捕获，经硬件数字滤波后触发中断，再读取GPIO状态。 */
#ifndef MOTOR_HALL_CAPTURE_MODE
#define MOTOR_HALL_CAPTURE_MODE             1U
#endif
#define MOTOR_HALL_CAPTURE_MODE_EXINT       0U
#define MOTOR_HALL_CAPTURE_MODE_TIMER_XOR   1U
#if (MOTOR_HALL_CAPTURE_MODE > 1U)
#error "MOTOR_HALL_CAPTURE_MODE must be 0 or 1"
#endif
#define MOTOR_HALL_CAPTURE_TIMER            TMR4
#define MOTOR_HALL_CAPTURE_CRM_CLK          CRM_TMR4_PERIPH_CLOCK
#define MOTOR_HALL_CAPTURE_IRQn             TMR4_GLOBAL_IRQn
/* 数字滤波：DTS=TMR时钟/4，滤波值0xF(fDTS/32采样、连续8次一致)，
 * TMR4时钟192 MHz时约5.3 us。demo为TMR_CLOCK_DIV2+0x6(约0.25 us)。 */
#define MOTOR_HALL_CAPTURE_FILTER_CLK_DIV   TMR_CLOCK_DIV4
#define MOTOR_HALL_CAPTURE_FILTER           0xFU
/* HA=PB8(TMR4_CH3)、HB=PB6(TMR4_CH1)、HC=PB7(TMR4_CH2)，复用MUX2。 */
#define MOTOR_HALL_A_PIN_SOURCE             GPIO_PINS_SOURCE8
#define MOTOR_HALL_B_PIN_SOURCE             GPIO_PINS_SOURCE6
#define MOTOR_HALL_C_PIN_SOURCE             GPIO_PINS_SOURCE7
#define MOTOR_HALL_GPIO_MUX                 GPIO_MUX_2

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
