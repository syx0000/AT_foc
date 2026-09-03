#include "at32f45x.h"
#include "motor_performance_monitor.h"
#include "motor_timebase.h"

uint32_t motor_performance_monitor_begin(void)
{
  return motor_timebase_cycles_get();
}

uint32_t motor_performance_monitor_end(
  motor_performance_counter_t *counter,
  uint32_t start_cycles)
{
  uint32_t elapsed_cycles;

  if (counter == 0)
  {
    return 0U;
  }

  elapsed_cycles = motor_timebase_cycles_elapsed(start_cycles);
  counter->latest_cycles = elapsed_cycles;
  if (elapsed_cycles > counter->max_cycles)
  {
    counter->max_cycles = elapsed_cycles;
  }
  counter->call_count++;

  return elapsed_cycles;
}

void motor_performance_monitor_snapshot_reset(
  motor_performance_counter_t *counter,
  motor_performance_snapshot_t *snapshot)
{
  uint32_t primask;

  if ((counter == 0) || (snapshot == 0))
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  snapshot->latest_cycles = counter->latest_cycles;
  snapshot->max_cycles = counter->max_cycles;
  snapshot->call_count = counter->call_count;
  counter->latest_cycles = 0U;
  counter->max_cycles = 0U;
  counter->call_count = 0U;
  __set_PRIMASK(primask);
}
