#include "mc_lib.h"
#include "mc_ident_diag.h"

#if defined(USE_UART_LOG) && defined(MOTOR_PARAM_IDENTIFY)

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

static flag_status motor_ident_diag_command_match(uint8_t *data, uint8_t length,
                                                   const char *command, uint8_t command_length)
{
  uint8_t index;

  while ((length > 0U) &&
         ((data[length - 1U] == '\r') || (data[length - 1U] == '\n') ||
          (data[length - 1U] == ' ') || (data[length - 1U] == '\t')))
  {
    length--;
  }

  if (length != command_length)
  {
    return RESET;
  }

  for (index = 0U; index < command_length; index++)
  {
    uint8_t received = data[index];
    if ((received >= 'A') && (received <= 'Z'))
    {
      received = (uint8_t)(received + ('a' - 'A'));
    }
    if (received != (uint8_t)command[index])
    {
      return RESET;
    }
  }

  return SET;
}

static void motor_ident_diag_command_task(void)
{
  uint8_t command[RCP_MAX_FRAME_SIZE + 1U];
  uint8_t length;
  uint32_t primask;

  if (uart_log_rx_take(command, &length) == RESET)
  {
    return;
  }

  if (motor_ident_diag_command_match(command, length, "clear", 5U) != RESET)
  {
    if (esc_state != ESC_STATE_ERROR)
    {
      printf("ERR clear state=%u error=0x%08lX\r\n",
             (unsigned int)esc_state, (unsigned long)error_code);
      return;
    }

    error_code = MC_NO_ERROR;
    printf("OK clear\r\n");
    return;
  }

  if (motor_ident_diag_command_match(command, length, "ident", 5U) == RESET)
  {
    printf("ERR command; use ident or clear\r\n");
    return;
  }

  if (esc_state != ESC_STATE_SAFETY_READY)
  {
    printf("ERR ident state=%u error=0x%08lX\r\n",
           (unsigned int)esc_state, (unsigned long)error_code);
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  motor_param_ident.state_flag = PROCESSING;
  esc_state = ESC_STATE_WINDING_PARAM_ID;
  __set_PRIMASK(primask);
  printf("OK ident\r\n");
}

void motor_ident_diag_task(void)
{
  ident_diag_record_type record;
  float_u32_union rs;
  float_u32_union ls;
  uint32_t primask;

  motor_ident_diag_command_task();

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
