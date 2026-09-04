#include <stdio.h>
#include <string.h>
#include "motor_cli.h"
#include "motor_control_config.h"
#include "motor_current_control.h"
#include "motor_control.h"
#include "motor_log.h"
#include "motor_open_loop.h"
#include "motor_speed_feedback.h"
#include "motor_speed_control.h"
#include "motor_uart_port.h"
#include "product_version.h"

#define MOTOR_CLI_LINE_SIZE 96U
static char motor_cli_line[MOTOR_CLI_LINE_SIZE];
static uint16_t motor_cli_line_length;

static void motor_cli_command_execute(char *line)
{
  unsigned int level;
  long direct_mv;
  long quadrature_mv;
  long frequency_millihz;
  long direct_current_ma;
  long quadrature_current_ma;
  long speed_rpm;
  unsigned long acceleration_millihz_per_s;
  motor_open_loop_command_t open_command;
  motor_open_loop_status_t open_loop;
  motor_current_control_status_t current_control;
  motor_current_control_command_t current_command;
  motor_control_status_t motor;
  motor_speed_feedback_t speed_feedback;
  motor_speed_control_status_t speed_control;

  if (strcmp(line, "help") == 0)
  {
    printf("OK commands: help version status fault fault clear log level <0..4> motor stop open start/set <vd_mv> <vq_mv> <freq_mhz> <accel_mhz_s> open stop current start/set <id_ma> <iq_ma> current stop speed start/set <rpm> speed stop\r\n");
  }
  else if (strcmp(line, "version") == 0)
  {
    printf("OK fw=%s hw=%s\r\n", FIRMWARE_VERSION_STRING,
           HARDWARE_VERSION_STRING);
  }
  else if (strcmp(line, "status") == 0)
  {
    (void)motor_open_loop_status_read(&open_loop);
    (void)motor_current_control_status_read(&current_control);
    (void)motor_control_status_read(&motor);
    (void)motor_speed_feedback_read(&speed_feedback);
    (void)motor_speed_control_status_read(&speed_control);
    printf("OK motor=%u motor_fault=%lu open=%u freq_mhz=%ld/%ld speed_rpm=%ld/%ld speed_valid=%u speed_ctrl=%u/%u iq_cmd=%ld current=%u fault=%u id=%ld/%ld iq=%ld/%ld uart_drop=%lu\r\n",
      (unsigned int)motor.state, (unsigned long)motor.fault_code,
      (unsigned int)open_loop.state,
      (long)open_loop.actual_frequency_millihz,
      (long)open_loop.target_frequency_millihz,
      (long)(speed_feedback.filtered_speed_millirpm / 1000L),
      (long)(speed_control.target_speed_millirpm / 1000L),
      (unsigned int)speed_feedback.valid,
      (unsigned int)speed_control.state,
      (unsigned int)speed_control.fault,
      (long)speed_control.quadrature_current_command_ma,
      (unsigned int)current_control.state,
      (unsigned int)current_control.fault,
      (long)current_control.direct_feedback_ma,
      (long)current_control.command.direct_reference_ma,
      (long)current_control.quadrature_feedback_ma,
      (long)current_control.command.quadrature_reference_ma,
      (unsigned long)motor_uart_port_overflow_count_get());
  }
  else if (strcmp(line, "motor stop") == 0)
  {
    motor_control_stop();
    printf("OK motor stopped\r\n");
  }
  else if (strcmp(line, "fault") == 0)
  {
    (void)motor_control_status_read(&motor);
    printf("OK state=%u fault=%lu\r\n", (unsigned int)motor.state,
           (unsigned long)motor.fault_code);
  }
  else if (strcmp(line, "fault clear") == 0)
  {
    if (motor_control_fault_clear())
      printf("OK fault cleared\r\n");
    else
      printf("ERR 7 fault_clear_denied\r\n");
  }
  else if (strcmp(line, "open stop") == 0)
  {
    motor_control_stop();
    printf("OK open stopped\r\n");
  }
  else if (sscanf(line, "open start %ld %ld %ld %lu", &direct_mv,
                  &quadrature_mv, &frequency_millihz,
                  &acceleration_millihz_per_s) == 4)
  {
    open_command.direct_voltage_mv = (int32_t)direct_mv;
    open_command.quadrature_voltage_mv = (int32_t)quadrature_mv;
    open_command.target_frequency_millihz = (int32_t)frequency_millihz;
    open_command.acceleration_millihz_per_s =
      (uint32_t)acceleration_millihz_per_s;
    if (motor_control_open_loop_start(&open_command))
      printf("OK open started\r\n");
    else
      printf("ERR 5 motor_not_ready_or_invalid_command\r\n");
  }
  else if (sscanf(line, "open set %ld %ld %ld %lu", &direct_mv,
                  &quadrature_mv, &frequency_millihz,
                  &acceleration_millihz_per_s) == 4)
  {
    open_command.direct_voltage_mv = (int32_t)direct_mv;
    open_command.quadrature_voltage_mv = (int32_t)quadrature_mv;
    open_command.target_frequency_millihz = (int32_t)frequency_millihz;
    open_command.acceleration_millihz_per_s =
      (uint32_t)acceleration_millihz_per_s;
    if (motor_control_open_loop_command_set(&open_command))
      printf("OK open updated\r\n");
    else
      printf("ERR 6 open_not_running_or_invalid_command\r\n");
  }
  else if (strcmp(line, "current stop") == 0)
  {
    motor_control_stop();
    printf("OK current stopped\r\n");
  }
  else if (strcmp(line, "speed stop") == 0)
  {
    motor_control_stop();
    printf("OK speed stopped\r\n");
  }
  else if (sscanf(line, "speed start %ld", &speed_rpm) == 1)
  {
    if ((speed_rpm <= 0) ||
        (speed_rpm > MOTOR_SPEED_CONTROL_MAXIMUM_SPEED_RPM))
    {
      printf("ERR 3 invalid_parameter\r\n");
      return;
    }
    if (motor_control_speed_control_start((int32_t)speed_rpm))
      printf("OK speed started target=%ld rpm\r\n", speed_rpm);
    else
      printf("ERR 10 open_not_running_or_speed_feedback_invalid\r\n");
  }
  else if (sscanf(line, "speed set %ld", &speed_rpm) == 1)
  {
    if ((speed_rpm <= 0) ||
        (speed_rpm > MOTOR_SPEED_CONTROL_MAXIMUM_SPEED_RPM))
    {
      printf("ERR 3 invalid_parameter\r\n");
      return;
    }
    if (motor_control_speed_control_target_set((int32_t)speed_rpm))
      printf("OK speed updated target=%ld rpm\r\n", speed_rpm);
    else
      printf("ERR 11 speed_not_running_or_invalid_command\r\n");
  }
  else if (sscanf(line, "current start %ld %ld", &direct_current_ma,
                  &quadrature_current_ma) == 2)
  {
    if ((direct_current_ma > MOTOR_CLI_CURRENT_COMMAND_LIMIT_MA) ||
        (direct_current_ma < -MOTOR_CLI_CURRENT_COMMAND_LIMIT_MA) ||
        (quadrature_current_ma > MOTOR_CLI_CURRENT_COMMAND_LIMIT_MA) ||
        (quadrature_current_ma < -MOTOR_CLI_CURRENT_COMMAND_LIMIT_MA))
    {
      printf("ERR 3 invalid_parameter\r\n");
      return;
    }
    current_command.direct_reference_ma = (int32_t)direct_current_ma;
    current_command.quadrature_reference_ma = (int32_t)quadrature_current_ma;
    if (motor_control_current_control_start(&current_command))
      printf("OK current started\r\n");
    else
      printf("ERR 8 open_not_running_hall_invalid_or_bad_command\r\n");
  }
  else if (sscanf(line, "current set %ld %ld", &direct_current_ma,
                  &quadrature_current_ma) == 2)
  {
    if ((direct_current_ma > MOTOR_CLI_CURRENT_COMMAND_LIMIT_MA) ||
        (direct_current_ma < -MOTOR_CLI_CURRENT_COMMAND_LIMIT_MA) ||
        (quadrature_current_ma > MOTOR_CLI_CURRENT_COMMAND_LIMIT_MA) ||
        (quadrature_current_ma < -MOTOR_CLI_CURRENT_COMMAND_LIMIT_MA))
    {
      printf("ERR 3 invalid_parameter\r\n");
      return;
    }
    current_command.direct_reference_ma = (int32_t)direct_current_ma;
    current_command.quadrature_reference_ma = (int32_t)quadrature_current_ma;
    if (motor_control_current_control_command_set(&current_command))
      printf("OK current updated\r\n");
    else
      printf("ERR 9 current_not_running_or_invalid_command\r\n");
  }
  else if (sscanf(line, "log level %u", &level) == 1)
  {
    if (level <= (unsigned int)MOTOR_LOG_LEVEL_INFO)
    {
      motor_log_level_set((motor_log_level_t)level);
      printf("OK log_level=%u\r\n", level);
    }
    else
    {
      printf("ERR 3 invalid_parameter\r\n");
    }
  }
  else
  {
    printf("ERR 1 unknown_command\r\n");
  }
}

void motor_cli_poll(void)
{
  motor_uart_port_frame_t frame;
  uint16_t index;
  char character;

  if (!motor_uart_port_frame_read(&frame)) return;
  for (index = 0U; index < frame.length; index++)
  {
    character = (char)frame.data[index];
    if ((character == '\r') || (character == '\n'))
    {
      if (motor_cli_line_length != 0U)
      {
        motor_cli_line[motor_cli_line_length] = '\0';
        motor_cli_command_execute(motor_cli_line);
        motor_cli_line_length = 0U;
      }
    }
    else if (motor_cli_line_length < (MOTOR_CLI_LINE_SIZE - 1U))
    {
      motor_cli_line[motor_cli_line_length++] = character;
    }
    else
    {
      motor_cli_line_length = 0U;
      printf("ERR 2 line_too_long\r\n");
    }
  }
}
