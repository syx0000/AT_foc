#ifndef MOTOR_TRACE_H
#define MOTOR_TRACE_H
#include <stdbool.h>
#include <stdint.h>
/* Called after the ADC control cycle. No formatting or UART work in ISR. */
void motor_trace_fast_sample(void);
void motor_trace_poll(void);
bool motor_trace_command(const char *line);
#endif
