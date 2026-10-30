#include "mc_lib.h"
#include "mc_ident_diag.h"

#if defined(USE_HALL_HOST) && defined(MOTOR_PARAM_IDENTIFY)

#define IDENT_DIAG_QUEUE_SIZE       (8U)
#define IDENT_DIAG_QUEUE_MASK       (IDENT_DIAG_QUEUE_SIZE - 1U)

typedef struct
{
  uint8_t event;
  uint32_t error;
  uint32_t rs_bits;
  uint32_t ls_bits;
} ident_diag_record_type;

static ident_diag_record_type ident_diag_queue[IDENT_DIAG_QUEUE_SIZE];
static volatile uint8_t ident_diag_head;
static volatile uint8_t ident_diag_tail;
static volatile ident_diag_event_type ident_diag_last_fault;

void motor_ident_diag_reset(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  ident_diag_head = 0U;
  ident_diag_tail = 0U;
  ident_diag_last_fault = IDENT_DIAG_NONE;
  __set_PRIMASK(primask);
}

void motor_ident_diag_capture(ident_diag_event_type event)
{
  ident_diag_record_type record;
  uint8_t next;
  uint32_t primask;

  record.event = (uint8_t)event;
  record.error = (uint32_t)error_code;
  record.rs_bits = motor_param_ident.Rs.u32;
  record.ls_bits = motor_param_ident.Ls.u32;

  primask = __get_PRIMASK();
  __disable_irq();
  next = (uint8_t)((ident_diag_head + 1U) & IDENT_DIAG_QUEUE_MASK);
  if (next == ident_diag_tail)
  {
    ident_diag_tail = (uint8_t)((ident_diag_tail + 1U) & IDENT_DIAG_QUEUE_MASK);
  }
  ident_diag_queue[ident_diag_head] = record;
  ident_diag_head = next;
  __set_PRIMASK(primask);
}

void motor_ident_diag_fault_if_unset(ident_diag_event_type event)
{
  uint32_t primask = __get_PRIMASK();
  flag_status capture = RESET;

  __disable_irq();
  if (ident_diag_last_fault == IDENT_DIAG_NONE)
  {
    ident_diag_last_fault = event;
    capture = SET;
  }
  __set_PRIMASK(primask);

  if (capture != RESET)
  {
    motor_ident_diag_capture(event);
  }
}

static const char *motor_ident_diag_event_name(uint8_t event)
{
  switch ((ident_diag_event_type)event)
  {
    case IDENT_DIAG_OVERCURRENT:    return "OVERCURRENT";
    case IDENT_DIAG_DUTY_LIMIT:     return "DUTY_LIMIT";
    case IDENT_DIAG_TIMEOUT:        return "TIMEOUT";
    case IDENT_DIAG_INVALID_RESULT: return "INVALID_RESULT";
    case IDENT_DIAG_EXTERNAL_ABORT: return "EXTERNAL_ABORT";
    default:                        return "UNKNOWN";
  }
}

void motor_ident_diag_task(void)
{
  ident_diag_record_type record;
  float_u32_union rs;
  float_u32_union ls;
  uint32_t primask;


  primask = __get_PRIMASK();
  __disable_irq();
  if (ident_diag_tail == ident_diag_head)
  {
    __set_PRIMASK(primask);
    return;
  }

  record = ident_diag_queue[ident_diag_tail];
  ident_diag_tail = (uint8_t)((ident_diag_tail + 1U) & IDENT_DIAG_QUEUE_MASK);
  __set_PRIMASK(primask);

  rs.u32 = record.rs_bits;
  ls.u32 = record.ls_bits;

  if (record.event == IDENT_DIAG_SUCCESS)
  {
    printf("ID RESULT Rs=%ldmOhm Ls=%lduH\r\n",
           (long)(rs.f * 1000.0f), (long)(ls.f * 1000000.0f));
  }
  else if ((record.event == IDENT_DIAG_TIMEOUT) ||
           (record.event == IDENT_DIAG_INVALID_RESULT) ||
           (record.event == IDENT_DIAG_EXTERNAL_ABORT) ||
           (record.event == IDENT_DIAG_OVERCURRENT) ||
           (record.event == IDENT_DIAG_DUTY_LIMIT))
  {
    printf("ID FAILED reason=%s err=0x%08lX\r\n",
           motor_ident_diag_event_name(record.event),
           (unsigned long)record.error);
  }
}

#endif
