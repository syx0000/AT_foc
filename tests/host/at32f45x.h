/* Host-only interrupt stand-ins for controller regression tests. */
#ifndef TEST_AT32F45X_H
#define TEST_AT32F45X_H
#include <stdint.h>
extern uint32_t system_core_clock;
static uint32_t test_primask;
static uint32_t __get_PRIMASK(void) { return test_primask; }
static void __disable_irq(void) { test_primask = 1; }
static void (*test_irq_hook)(void);
static void __set_PRIMASK(uint32_t value) {
  uint32_t old=test_primask;
  test_primask=value;
  if(old && !value && test_irq_hook) {
    void (*hook)(void)=test_irq_hook;
    test_irq_hook=0;
    hook();
  }
}
static void __enable_irq(void) { __set_PRIMASK(0); }
#endif
