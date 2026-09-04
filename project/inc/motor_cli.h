/**
 * @file    motor_cli.h
 * @brief   电机调试串口命令 公共数据结构和接口声明
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#ifndef MOTOR_CLI_H
#define MOTOR_CLI_H
/**
 * @brief 轮询USART1调试命令并在主循环解析执行。
 * @param 无。
 * @return 无。
 */
void motor_cli_poll(void);
#endif
