#ifndef __MC_IDENT_DIAG_H
#define __MC_IDENT_DIAG_H

#include <stdint.h>

typedef enum
{
  IDENT_DIAG_NONE = 0,
  IDENT_DIAG_PRECHECK,
  IDENT_DIAG_START,
  IDENT_DIAG_STEP,
  IDENT_DIAG_SAMPLE,
  IDENT_DIAG_OVERCURRENT,
  IDENT_DIAG_DUTY_LIMIT,
  IDENT_DIAG_TIMEOUT,
  IDENT_DIAG_INVALID_RESULT,
  IDENT_DIAG_EXTERNAL_ABORT,
  IDENT_DIAG_SUCCESS
} ident_diag_event_type;

#if defined(USE_UART_LOG) && defined(MOTOR_PARAM_IDENTIFY)
void motor_ident_diag_reset(void);
void motor_ident_diag_capture(ident_diag_event_type event);
void motor_ident_diag_fault_if_unset(ident_diag_event_type event);
void motor_ident_diag_task(void);
#endif

#endif
