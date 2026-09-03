#include "at32f45x.h"
#include "motor_timebase.h"

static uint32_t motor_timebase_cycles_per_us = 1U;

void motor_timebase_init(void)
{
  uint32_t core_clock_hz = system_core_clock;

  if (core_clock_hz < 1000000U)
  {
    core_clock_hz = 1000000U;
  }

  motor_timebase_cycles_per_us = core_clock_hz / 1000000U;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  __DSB();
  __ISB();
}

uint32_t motor_timebase_cycles_get(void)
{
  return DWT->CYCCNT;
}

uint32_t motor_timebase_cycles_elapsed(uint32_t start_cycles)
{
  return DWT->CYCCNT - start_cycles;
}

uint32_t motor_timebase_cycles_to_us(uint32_t cycles)
{
  return cycles / motor_timebase_cycles_per_us;
}

uint32_t motor_timebase_us_get(void)
{
  return motor_timebase_cycles_to_us(DWT->CYCCNT);
}
