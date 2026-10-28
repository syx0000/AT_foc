/**
 * @file    motor_can.h
 * @brief   ZXQ CAN电机协议（移植自AT_foc_hall_demo mc_can_motor） 接口声明
 * @author  随永祥
 * @date    2026-10-24
 * @version 1.0
 */

#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H

/* ZXQ标准帧ID：一台电机编译为1（左），另一台为2（右）。 */
#ifndef MOTOR_CAN_NODE_ID
#define MOTOR_CAN_NODE_ID 1
#endif
#if (MOTOR_CAN_NODE_ID < 1) || (MOTOR_CAN_NODE_ID > 127)
#error "MOTOR_CAN_NODE_ID must be 1..127"
#endif

/* ZXQ每500 ms查询一次；CAN启动电机后超过该时间无有效指令则停机。 */
#define MOTOR_CAN_TIMEOUT_MS  1500U
#define MOTOR_CAN_FEEDBACK_MS   10U

/**
 * @brief 初始化CAN1：PA11/PA12，HEXT 8 MHz，经典CAN 500 kbit/s，采样点75%。
 * @param 无。
 * @return 无。
 */
void motor_can_init(void);

/**
 * @brief 主循环调用：处理接收指令、通信超时停机及周期反馈发送。
 * @param 无。
 * @return 无。
 */
void motor_can_task(void);

/**
 * @brief 1 kHz SysTick调用，为通信超时和反馈周期提供毫秒计数。
 * @param 无。
 * @return 无。
 */
void motor_can_tick(void);

#endif /* MOTOR_CAN_H */
