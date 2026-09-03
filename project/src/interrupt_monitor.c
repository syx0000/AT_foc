#include "at32f45x.h"
#include "interrupt_monitor.h"
#include "motor_log.h"

#define INTERRUPT_MONITOR_REPORT_TICKS 1000U

interrupt_monitor_counters_t interrupt_monitor_counters;

void interrupt_monitor_poll(void)
{
  interrupt_monitor_counters_t snapshot;

  if (interrupt_monitor_counters.systick < INTERRUPT_MONITOR_REPORT_TICKS)
  {
    return;
  }

  __disable_irq();
  snapshot = interrupt_monitor_counters;
  interrupt_monitor_counters.systick = 0U;
  interrupt_monitor_counters.adc_slow_dma_complete = 0U;
  interrupt_monitor_counters.adc_fast_complete = 0U;
  interrupt_monitor_counters.adc_trigger_fail = 0U;
  interrupt_monitor_counters.tmr1_channel4 = 0U;
  interrupt_monitor_counters.hall_a_edge = 0U;
  interrupt_monitor_counters.hall_b_edge = 0U;
  interrupt_monitor_counters.hall_c_edge = 0U;
  interrupt_monitor_counters.can1_rx = 0U;
  interrupt_monitor_counters.can1_error = 0U;
  interrupt_monitor_counters.usart1_idle = 0U;
  interrupt_monitor_counters.usart1_error = 0U;
  interrupt_monitor_counters.usart3_idle = 0U;
  interrupt_monitor_counters.usart3_error = 0U;
  __enable_irq();

  LOGI("IRQ Hz: tick=%lu tmr1_c4=%lu adc_fast=%lu adc_slow_dma=%lu adc_fail=%lu\r\n",
       snapshot.systick,
       snapshot.tmr1_channel4,
       snapshot.adc_fast_complete,
       snapshot.adc_slow_dma_complete,
       snapshot.adc_trigger_fail);
  LOGI("IRQ Hz: hall=%lu/%lu/%lu can=%lu/%lu uart1=%lu/%lu uart3=%lu/%lu\r\n",
       snapshot.hall_a_edge,
       snapshot.hall_b_edge,
       snapshot.hall_c_edge,
       snapshot.can1_rx,
       snapshot.can1_error,
       snapshot.usart1_idle,
       snapshot.usart1_error,
       snapshot.usart3_idle,
       snapshot.usart3_error);
  LOGI("--------------------\r\n");
}
