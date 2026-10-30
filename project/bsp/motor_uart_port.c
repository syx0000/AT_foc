/**
 * @file    motor_uart_port.c
 * @brief   电机调试串口端口 功能实现
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#include <stddef.h>
#include "at32f45x.h"
#include "motor_uart_port.h"
#include "wk_dma.h"
static uint8_t motor_uart_port_dma_buffer[MOTOR_UART_PORT_RECEIVE_BUFFER_SIZE];
static volatile motor_uart_port_frame_t motor_uart_port_mailbox;
static volatile bool motor_uart_port_frame_pending;
static volatile uint32_t motor_uart_port_overflow_count;
static void motor_uart_port_dma_restart(void)
{
  dma_channel_enable(DMA1_CHANNEL4, FALSE);
  wk_dma_channel_config(DMA1_CHANNEL4, (uint32_t)&USART1->dt,
    (uint32_t)motor_uart_port_dma_buffer, MOTOR_UART_PORT_RECEIVE_BUFFER_SIZE);
  dma_channel_enable(DMA1_CHANNEL4, TRUE);
}
void motor_uart_port_init(void)
{
  motor_uart_port_mailbox.length = 0U;
  motor_uart_port_frame_pending = false;
  motor_uart_port_overflow_count = 0U;
  usart_dma_transmitter_enable(USART1, FALSE);
  dma_channel_enable(DMA1_CHANNEL5, FALSE);
  motor_uart_port_dma_restart();
}
void motor_uart_port_idle_capture(void)
{
  uint16_t length;
  uint16_t index;
  dma_channel_enable(DMA1_CHANNEL4, FALSE);
  length = (uint16_t)(MOTOR_UART_PORT_RECEIVE_BUFFER_SIZE -
                      dma_data_number_get(DMA1_CHANNEL4));
  if ((length != 0U) && (!motor_uart_port_frame_pending))
  {
    for (index = 0U; index < length; index++)
      motor_uart_port_mailbox.data[index] = motor_uart_port_dma_buffer[index];
    motor_uart_port_mailbox.length = length;
    motor_uart_port_frame_pending = true;
  }
  else if (length != 0U)
  {
    motor_uart_port_overflow_count++;
  }
  motor_uart_port_dma_restart();
}
bool motor_uart_port_frame_read(motor_uart_port_frame_t *frame)
{
  uint32_t primask;
  uint16_t index;
  if ((frame == NULL) || (!motor_uart_port_frame_pending)) return false;
  primask = __get_PRIMASK();
  __disable_irq();
  frame->length = motor_uart_port_mailbox.length;
  for (index = 0U; index < frame->length; index++)
    frame->data[index] = motor_uart_port_mailbox.data[index];
  motor_uart_port_frame_pending = false;
  if (primask == 0U) __enable_irq();
  return true;
}
uint32_t motor_uart_port_overflow_count_get(void)
{
  return motor_uart_port_overflow_count;
}

/* Single DMA-owned frame; caller keeps its sample queued while busy. */
static char motor_uart_trace_buffer[320];
static bool motor_uart_trace_busy;
bool motor_uart_port_trace_ready(void)
{
  return !motor_uart_trace_busy ||
    (dma_data_number_get(DMA1_CHANNEL5)==0U &&
     usart_flag_get(USART1, USART_TDC_FLAG)!=RESET);
}
void motor_uart_port_trace_flush(void)
{
  if (!motor_uart_trace_busy) return;
  while (dma_data_number_get(DMA1_CHANNEL5) != 0U) {}
  while (usart_flag_get(USART1, USART_TDC_FLAG) == RESET) {}
  dma_channel_enable(DMA1_CHANNEL5, FALSE);
  usart_dma_transmitter_enable(USART1, FALSE);
  motor_uart_trace_busy = false;
}
bool motor_uart_port_trace_write(const char *data, uint16_t length)
{
  uint16_t i;
  if (!data || length==0U || length > sizeof(motor_uart_trace_buffer)) return false;
  if (motor_uart_trace_busy)
  {
    if (!motor_uart_port_trace_ready()) return false;
    motor_uart_port_trace_flush();
  }
  for (i=0; i<length; ++i) motor_uart_trace_buffer[i]=data[i];
  dma_channel_enable(DMA1_CHANNEL5, FALSE);
  dma_flag_clear(DMA1_FDT5_FLAG);
  wk_dma_channel_config(DMA1_CHANNEL5, (uint32_t)&USART1->dt,
    (uint32_t)motor_uart_trace_buffer, length);
  motor_uart_trace_busy = true;
  usart_dma_transmitter_enable(USART1, TRUE);
  dma_channel_enable(DMA1_CHANNEL5, TRUE);
  return true;
}
