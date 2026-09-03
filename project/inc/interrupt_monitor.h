#ifndef INTERRUPT_MONITOR_H
#define INTERRUPT_MONITOR_H

#include <stdint.h>

typedef struct
{
  volatile uint32_t systick;
  volatile uint32_t adc_slow_dma_complete;
  volatile uint32_t adc_fast_complete;
  volatile uint32_t adc_trigger_fail;
  volatile uint32_t tmr1_channel4;
  volatile uint32_t hall_a_edge;
  volatile uint32_t hall_b_edge;
  volatile uint32_t hall_c_edge;
  volatile uint32_t can1_rx;
  volatile uint32_t can1_error;
  volatile uint32_t usart1_idle;
  volatile uint32_t usart1_error;
  volatile uint32_t usart3_idle;
  volatile uint32_t usart3_error;
} interrupt_monitor_counters_t;

extern interrupt_monitor_counters_t interrupt_monitor_counters;

void interrupt_monitor_poll(void);

#endif /* INTERRUPT_MONITOR_H */
