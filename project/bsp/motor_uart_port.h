#ifndef MOTOR_UART_PORT_H
#define MOTOR_UART_PORT_H
#include <stdbool.h>
#include <stdint.h>
#define MOTOR_UART_PORT_RECEIVE_BUFFER_SIZE 128U
typedef struct {
  uint8_t data[MOTOR_UART_PORT_RECEIVE_BUFFER_SIZE];
  uint16_t length;
} motor_uart_port_frame_t;
/** @brief 初始化USART1 RX DMA端口并关闭未使用的TX DMA。 @param 无。 @return 无。 */
void motor_uart_port_init(void);
/** @brief USART1空闲中断提交DMA数据并重新装载接收。 @param 无。 @return 无。 */
void motor_uart_port_idle_capture(void);
/** @brief 主循环读取一帧。 @param frame 输出数据和长度。 @return 有帧返回true。 */
bool motor_uart_port_frame_read(motor_uart_port_frame_t *frame);
/** @brief 读取邮箱忙导致的丢帧数。 @param 无。 @return 累计丢帧数。 */
uint32_t motor_uart_port_overflow_count_get(void);
#endif
