#include <stdio.h>
#include <string.h>
#include "motor_cli.h"
#include "motor_control_config.h"
#include "motor_commissioning.h"
#include "motor_hall_decoder.h"
#include "motor_hall_port.h"
#include "motor_current_control.h"
#include "motor_control.h"
#include "motor_log.h"
#include "motor_open_loop.h"
#include "motor_parameter.h"
#include "motor_parameter_storage.h"
#include "motor_speed_feedback.h"
#include "motor_speed_control.h"
#include "motor_uart_port.h"
#include "product_version.h"
#include "wk_system.h"

#define MOTOR_CLI_LINE_SIZE 96U
static char motor_cli_line[MOTOR_CLI_LINE_SIZE];
static uint16_t motor_cli_line_length;

typedef struct
{
  const char *name;
  motor_parameter_field_t field;
} motor_cli_parameter_field_t;

static const motor_cli_parameter_field_t motor_cli_parameter_fields[] =
{
  {"pole_pairs", MOTOR_PARAMETER_FIELD_POLE_PAIRS},
  {"direction_inverted", MOTOR_PARAMETER_FIELD_DIRECTION_INVERTED},
  {"phase_resistance_mohm", MOTOR_PARAMETER_FIELD_PHASE_RESISTANCE_MOHM},
  {"direct_inductance_uh", MOTOR_PARAMETER_FIELD_DIRECT_INDUCTANCE_UH},
  {"quadrature_inductance_uh", MOTOR_PARAMETER_FIELD_QUADRATURE_INDUCTANCE_UH},
  {"current_loop_bandwidth_hz", MOTOR_PARAMETER_FIELD_CURRENT_LOOP_BANDWIDTH_HZ},
  {"current_d_kp_q15", MOTOR_PARAMETER_FIELD_CURRENT_D_KP_Q15},
  {"current_q_kp_q15", MOTOR_PARAMETER_FIELD_CURRENT_Q_KP_Q15},
  {"current_ki_q15", MOTOR_PARAMETER_FIELD_CURRENT_KI_Q15},
  {"speed_kp_q20", MOTOR_PARAMETER_FIELD_SPEED_KP_Q20},
  {"speed_ki_q20", MOTOR_PARAMETER_FIELD_SPEED_KI_Q20},
  {"hall_rotor_offset_u16", MOTOR_PARAMETER_FIELD_HALL_ROTOR_OFFSET_U16}
};

#define MOTOR_CLI_PARAMETER_FIELD_COUNT \
  (sizeof(motor_cli_parameter_fields) / sizeof(motor_cli_parameter_fields[0]))

/**
 * @brief 按串口参数名查找运行时参数字段。
 * @param name 用户输入的零结尾参数名，不允许为空。
 * @param field 输出字段枚举，不允许为空。
 * @return 找到完全匹配的参数名时返回true，否则返回false。
 */
static bool motor_cli_parameter_field_find(
  const char *name, motor_parameter_field_t *field)
{
  uint32_t index;

  if ((name == NULL) || (field == NULL)) return false;
  for (index = 0U; index < MOTOR_CLI_PARAMETER_FIELD_COUNT; index++)
  {
    if (strcmp(name, motor_cli_parameter_fields[index].name) == 0)
    {
      *field = motor_cli_parameter_fields[index].field;
      return true;
    }
  }
  return false;
}

/**
 * @brief 打印一份完整运行时电机参数。
 * @param label 参数来源标签，例如active或candidate，不允许为空。
 * @param parameter 待打印参数，不允许为空。
 * @return 无。
 */
static void motor_cli_parameter_print(const char *label,
                                      const motor_parameter_t *parameter)
{
  printf("OK params %s\r\n", label);
  printf("pole_pairs=%u direction_inverted=%u rs_mohm=%lu ld_uh=%lu lq_uh=%lu bandwidth_hz=%lu\r\n",
         (unsigned int)parameter->pole_pairs,
         (unsigned int)parameter->direction_inverted,
         (unsigned long)parameter->phase_resistance_mohm,
         (unsigned long)parameter->direct_inductance_uh,
         (unsigned long)parameter->quadrature_inductance_uh,
         (unsigned long)parameter->current_loop_bandwidth_hz);
  printf("current_pi_q15=%ld/%ld/%ld speed_pi_q20=%ld/%ld hall_offset_u16=%u\r\n",
         (long)parameter->current_d_kp_q15,
         (long)parameter->current_q_kp_q15,
         (long)parameter->current_ki_q15,
         (long)parameter->speed_kp_q20,
         (long)parameter->speed_ki_q20,
         (unsigned int)parameter->hall_rotor_offset_u16);
  printf("hall_next=%u/%u/%u/%u/%u/%u hall_angle=%u/%u/%u/%u/%u/%u\r\n",
         (unsigned int)parameter->hall_positive_next[1],
         (unsigned int)parameter->hall_positive_next[2],
         (unsigned int)parameter->hall_positive_next[3],
         (unsigned int)parameter->hall_positive_next[4],
         (unsigned int)parameter->hall_positive_next[5],
         (unsigned int)parameter->hall_positive_next[6],
         (unsigned int)parameter->hall_entry_angle_u16[1],
         (unsigned int)parameter->hall_entry_angle_u16[2],
         (unsigned int)parameter->hall_entry_angle_u16[3],
         (unsigned int)parameter->hall_entry_angle_u16[4],
         (unsigned int)parameter->hall_entry_angle_u16[5],
         (unsigned int)parameter->hall_entry_angle_u16[6]);
}

/**
 * @brief 打印活动参数与候选参数的逐字段差异。
 * @param 无。
 * @return 无。
 */
static void motor_cli_parameter_diff_print(void)
{
  motor_parameter_t active;
  motor_parameter_t candidate;
  motor_parameter_diff_t diff;
  int32_t active_value;
  int32_t candidate_value;
  uint32_t index;

  (void)motor_parameter_active_read(&active);
  (void)motor_parameter_candidate_read(&candidate);
  (void)motor_parameter_diff_read(&diff);
  if (!diff.any_changed)
  {
    printf("OK diff none\r\n");
    return;
  }
  printf("OK diff\r\n");
  for (index = 0U; index < MOTOR_CLI_PARAMETER_FIELD_COUNT; index++)
  {
    if ((diff.scalar_fields &
         (1UL << motor_cli_parameter_fields[index].field)) != 0U)
    {
      (void)motor_parameter_field_value_read(
        &active, motor_cli_parameter_fields[index].field, &active_value);
      (void)motor_parameter_field_value_read(
        &candidate, motor_cli_parameter_fields[index].field,
        &candidate_value);
      printf("%s: active=%ld candidate=%ld\r\n",
             motor_cli_parameter_fields[index].name,
             (long)active_value, (long)candidate_value);
    }
  }
  if (diff.hall_sequence_changed)
    printf("hall_sequence: changed\r\n");
  if (diff.hall_angle_changed)
    printf("hall_angle: changed\r\n");
}

/**
 * @brief 在电机安全停止且没有其他候选修改时切换逻辑方向。
 * @param inverted false设置normal，true设置reverse。
 * @return 候选方向成功写入并应用为活动参数时返回true，否则返回false。
 * @details 方向命令不会隐式接受其他待确认候选参数；失败时恢复候选参数。
 */
static bool motor_cli_direction_set(bool inverted)
{
  motor_parameter_diff_t diff;

  if ((!motor_parameter_diff_read(&diff)) || diff.any_changed)
  {
    return false;
  }
  if (!motor_parameter_candidate_field_set(
        MOTOR_PARAMETER_FIELD_DIRECTION_INVERTED, inverted ? 1 : 0))
  {
    return false;
  }
  if (!motor_parameter_candidate_accept())
  {
    motor_parameter_candidate_discard();
    return false;
  }
  return true;
}

/**
 * @brief 参数切换后按当前原始Hall电平重载运行时解码器。
 * @param 无。
 * @return 活动参数及Hall采样有效且解码器初始化成功时返回true。
 */
static bool motor_cli_parameter_runtime_reload(void)
{
  motor_parameter_t parameter;
  motor_hall_port_sample_t hall;

  return motor_parameter_active_read(&parameter) &&
    motor_hall_port_sample_read(&hall) &&
    motor_hall_decoder_init(parameter.hall_positive_next, system_core_clock,
                            hall.state, hall.timestamp_cycles) &&
    motor_current_control_parameter_reload() &&
    motor_speed_control_parameter_reload();
}

/**
 * @brief 执行辨识任务并保证成功、拒绝或失败均有明确CLI回显。
 * @param task 待执行的完整流程或单项任务。
 * @return 无。
 * @details 当前辨识为阻塞式；成功回显在任务完成后发送，入口条件不满足时
 *          立即返回电机状态，执行失败时同时返回辨识阶段及详细错误。
 */
static void motor_cli_commissioning_execute(motor_commissioning_task_t task)
{
  motor_control_status_t motor;
  motor_commissioning_status_t commissioning;

  if (motor_commissioning_run(task))
  {
    printf("OK commissioning complete task=%u, review diff then accept/discard\r\n",
           (unsigned int)task);
    return;
  }

  (void)motor_control_status_read(&motor);
  (void)motor_commissioning_status_read(&commissioning);
  printf("ERR commissioning rejected_or_failed task=%u motor=%u state=%u error=%s(%u) detail=%s(%lu)\r\n",
         (unsigned int)task, (unsigned int)motor.state,
         (unsigned int)commissioning.state,
         motor_commissioning_error_name_get(commissioning.error),
         (unsigned int)commissioning.error,
         motor_commissioning_error_detail_name_get(
           commissioning.error, commissioning.error_detail),
         (unsigned long)commissioning.error_detail);
}

static void motor_cli_command_execute(char *line)
{
  unsigned int level;
  long direct_mv;
  long quadrature_mv;
  long frequency_millihz;
  long direct_current_ma;
  long quadrature_current_ma;
  long speed_rpm;
  long parameter_value;
  char parameter_name[32];
  unsigned long acceleration_millihz_per_s;
  motor_open_loop_command_t open_command;
  motor_open_loop_status_t open_loop;
  motor_current_control_status_t current_control;
  motor_current_control_command_t current_command;
  motor_control_status_t motor;
  motor_speed_feedback_t speed_feedback;
  motor_speed_control_status_t speed_control;
  motor_parameter_t parameter;
  motor_parameter_field_t parameter_field;
  motor_commissioning_status_t commissioning;
  motor_parameter_storage_status_t parameter_storage;
  motor_fault_clear_result_t fault_clear_result;
  motor_fault_record_t fault_record;
  uint32_t fault_index;

  if (strcmp(line, "help") == 0)
  {
    printf("OK commands:\r\n");
    printf("  help | version | status\r\n");
    printf("  fault | fault clear | log level <0..4>\r\n");
    printf("  motor stop | motor direction [normal|reverse]\r\n");
    printf("  open start|set <vd_mv> <vq_mv> <freq_mhz> <accel_mhz_s>\r\n");
    printf("  open stop\r\n");
    printf("  current start|set <id_ma> <iq_ma> | current stop\r\n");
    printf("  speed start|set <signed_rpm> | speed stop\r\n");
    printf("  motor params active|candidate|load|defaults|storage\r\n");
    printf("  motor param set <name> <value>\r\n");
    printf("  motor commissioning start|status|abort|diff|accept|discard|save\r\n");
    printf("  calibrate current_offset\r\n");
    printf("  calibrate hall sequence|angle|offset\r\n");
    printf("  identify resistance|inductance | calculate current_pi\r\n");
    printf("  test current_d|current_q|current_handover\r\n");
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
    printf("OK motor=%u motor_fault=%s(%u) direction=%s open=%u freq_mhz=%ld/%ld speed_rpm=%ld/%ld speed_valid=%u speed_ctrl=%u/%u logical_iq_cmd=%ld current=%u fault=%u id=%ld/%ld iq=%ld/%ld uart_drop=%lu\r\n",
      (unsigned int)motor.state,
      motor_control_fault_name_get(motor.fault_code),
      (unsigned int)motor.fault_code,
      motor_parameter_direction_inverted_get() ? "reverse" : "normal",
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
  else if (strcmp(line, "motor direction") == 0)
  {
    printf("OK direction=%s reverse_control_verified=%u\r\n",
           motor_parameter_direction_inverted_get() ? "reverse" : "normal",
           (unsigned int)MOTOR_REVERSE_CONTROL_VERIFIED);
  }
  else if (strcmp(line, "motor direction normal") == 0)
  {
    if (motor_cli_direction_set(false))
      printf("OK direction=normal\r\n");
    else
      printf("ERR 13 direction_requires_ready_pwm_off_and_no_pending_diff\r\n");
  }
  else if (strcmp(line, "motor direction reverse") == 0)
  {
    if (motor_cli_direction_set(true))
      printf("OK direction=reverse reverse_control_verified=%u\r\n",
             (unsigned int)MOTOR_REVERSE_CONTROL_VERIFIED);
    else
      printf("ERR 13 direction_requires_ready_pwm_off_and_no_pending_diff\r\n");
  }
  else if (strcmp(line, "motor params active") == 0)
  {
    (void)motor_parameter_active_read(&parameter);
    motor_cli_parameter_print("active", &parameter);
  }
  else if (strcmp(line, "motor params candidate") == 0)
  {
    (void)motor_parameter_candidate_read(&parameter);
    motor_cli_parameter_print("candidate", &parameter);
  }
  else if (strcmp(line, "motor params storage") == 0)
  {
    (void)motor_parameter_storage_status_read(&parameter_storage);
    printf("OK storage source=%u sequence=%lu slot_a=%u slot_b=%u\r\n",
      (unsigned int)parameter_storage.source,
      (unsigned long)parameter_storage.sequence,
      (unsigned int)parameter_storage.slot_a_valid,
      (unsigned int)parameter_storage.slot_b_valid);
  }
  else if (strcmp(line, "motor params load") == 0)
  {
    if (motor_parameter_storage_load() && motor_cli_parameter_runtime_reload())
      printf("OK parameters loaded from flash\r\n");
    else
      printf("ERR parameter_load_requires_ready_pwm_off_and_valid_slot\r\n");
  }
  else if (strcmp(line, "motor params defaults") == 0)
  {
    if (motor_parameter_storage_defaults() &&
        motor_cli_parameter_runtime_reload())
      printf("OK runtime parameters restored to defaults, use save to persist\r\n");
    else
      printf("ERR parameter_defaults_requires_ready_pwm_off\r\n");
  }
  else if (strcmp(line, "motor commissioning diff") == 0)
  {
    motor_cli_parameter_diff_print();
  }
  else if (strcmp(line, "motor commissioning status") == 0)
  {
    (void)motor_commissioning_status_read(&commissioning);
    printf("OK commissioning state=%u task=%u step=%lu error=%s(%u) detail=%s(%lu) runs=%lu\r\n",
      commissioning.state,commissioning.task,commissioning.step,
      motor_commissioning_error_name_get(commissioning.error),
      (unsigned int)commissioning.error,
      motor_commissioning_error_detail_name_get(
        commissioning.error, commissioning.error_detail),
      (unsigned long)commissioning.error_detail,
      commissioning.run_count);
  }
  else if (strcmp(line, "motor commissioning abort") == 0)
  {
    printf(motor_commissioning_abort()?"OK commissioning abort requested\r\n":"ERR commissioning_not_running\r\n");
  }
  else if (strcmp(line, "motor commissioning start") == 0)
  {
    motor_cli_commissioning_execute(MOTOR_COMMISSIONING_TASK_FULL);
  }
  else if (strcmp(line, "calibrate current_offset") == 0)
  {
    motor_cli_commissioning_execute(MOTOR_COMMISSIONING_TASK_CURRENT_OFFSET);
  }
  else if ((strcmp(line, "calibrate hall sequence") == 0) ||
           (strcmp(line, "calibrate hall angle") == 0))
  {
    motor_cli_commissioning_execute(MOTOR_COMMISSIONING_TASK_HALL);
  }
  else if (strcmp(line, "calibrate hall offset") == 0)
  {
    motor_cli_commissioning_execute(MOTOR_COMMISSIONING_TASK_HALL_OFFSET);
  }
  else if (strcmp(line, "identify resistance") == 0)
  {
    motor_cli_commissioning_execute(MOTOR_COMMISSIONING_TASK_RESISTANCE);
  }
  else if (strcmp(line, "identify inductance") == 0)
  {
    motor_cli_commissioning_execute(MOTOR_COMMISSIONING_TASK_INDUCTANCE);
  }
  else if (strcmp(line, "calculate current_pi") == 0)
  {
    motor_cli_commissioning_execute(MOTOR_COMMISSIONING_TASK_CURRENT_PI);
  }
  else if (strcmp(line, "test current_d") == 0)
  {
    motor_cli_commissioning_execute(MOTOR_COMMISSIONING_TASK_CURRENT_D);
  }
  else if (strcmp(line, "test current_q") == 0)
  {
    motor_cli_commissioning_execute(MOTOR_COMMISSIONING_TASK_CURRENT_Q);
  }
  else if (strcmp(line, "test current_handover") == 0)
  {
    motor_cli_commissioning_execute(MOTOR_COMMISSIONING_TASK_CURRENT_HANDOVER);
  }
  else if (strcmp(line, "motor commissioning accept") == 0)
  {
    if (motor_parameter_candidate_accept())
    {
      (void)motor_commissioning_review_complete(true);
      if (motor_cli_parameter_runtime_reload())
        printf("OK candidate applied to runtime parameter manager\r\n");
      else
        printf("ERR candidate_applied_but_hall_decoder_reload_failed\r\n");
    }
    else
      printf("ERR 12 accept_requires_ready_pwm_off_and_valid_candidate\r\n");
  }
  else if (strcmp(line, "motor commissioning discard") == 0)
  {
    motor_parameter_candidate_discard();
    (void)motor_commissioning_review_complete(false);
    printf("OK candidate discarded\r\n");
  }
  else if (strcmp(line, "motor commissioning save") == 0)
  {
    if (motor_parameter_storage_save())
      printf("OK active parameters saved to flash\r\n");
    else
      printf("ERR parameter_save_requires_ready_pwm_off_or_flash_failed\r\n");
  }
  else if (sscanf(line, "motor param set %31s %ld",
                  parameter_name, &parameter_value) == 2)
  {
    if ((!motor_cli_parameter_field_find(parameter_name, &parameter_field)) ||
        (!motor_parameter_candidate_field_set(
          parameter_field, (int32_t)parameter_value)))
    {
      printf("ERR 3 invalid_parameter\r\n");
      return;
    }
    printf("OK candidate %s=%ld\r\n", parameter_name, parameter_value);
  }
  else if (strcmp(line, "fault") == 0)
  {
    (void)motor_control_status_read(&motor);
    printf("OK state=%u fault=%s(%u)\r\n", (unsigned int)motor.state,
           motor_control_fault_name_get(motor.fault_code),
           (unsigned int)motor.fault_code);
    for (fault_index = 0U; fault_index < MOTOR_FAULT_HISTORY_DEPTH;
         fault_index++)
    {
      if (!motor_control_fault_history_read(fault_index, &fault_record)) break;
      printf("#%lu sequence=%lu first=%s(%u) last=%s(%u) mask=0x%08lX count=%lu\r\n",
        (unsigned long)fault_index,
        (unsigned long)fault_record.sequence,
        motor_control_fault_name_get(fault_record.first_code),
        (unsigned int)fault_record.first_code,
        motor_control_fault_name_get(fault_record.last_code),
        (unsigned int)fault_record.last_code,
        (unsigned long)fault_record.code_mask,
        (unsigned long)fault_record.occurrence_count);
    }
  }
  else if (strcmp(line, "fault clear") == 0)
  {
    fault_clear_result = motor_control_fault_clear_ex();
    if (fault_clear_result == MOTOR_FAULT_CLEAR_OK)
      printf("OK fault cleared\r\n");
    else
      printf("ERR fault_clear_denied reason=%s(%u)\r\n",
        motor_control_fault_clear_result_name_get(fault_clear_result),
        (unsigned int)fault_clear_result);
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
    if ((speed_rpm == 0) ||
        ((int64_t)speed_rpm > MOTOR_SPEED_CONTROL_MAXIMUM_SPEED_RPM) ||
        ((int64_t)speed_rpm < -MOTOR_SPEED_CONTROL_MAXIMUM_SPEED_RPM))
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
    if ((speed_rpm == 0) ||
        ((int64_t)speed_rpm > MOTOR_SPEED_CONTROL_MAXIMUM_SPEED_RPM) ||
        ((int64_t)speed_rpm < -MOTOR_SPEED_CONTROL_MAXIMUM_SPEED_RPM))
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
