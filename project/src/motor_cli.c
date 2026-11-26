/**
 * @file    motor_cli.c
 * @brief   电机调试串口命令 功能实现
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#include <stdio.h>
#include <string.h>
#include "at32f45x.h"
#include "motor_cli.h"
#include "motor_control_config.h"
#include "motor_commissioning.h"
#include "motor_hall_angle_estimator.h"
#include "motor_hall_decoder.h"
#include "motor_hall_port.h"
#include "motor_current_control.h"
#include "motor_current_startup.h"
#include "motor_current_sample.h"
#include "motor_control.h"
#include "motor_direction.h"
#include "motor_log.h"
#include "motor_trace.h"
#include "motor_open_loop.h"
#include "motor_parameter.h"
#include "motor_parameter_storage.h"
#include "motor_pwm_port.h"
#include "motor_rotor_feedback.h"
#include "motor_speed_feedback.h"
#include "motor_speed_control.h"
#include "motor_timebase.h"
#include "motor_uart_port.h"
#include "product_version.h"
#include "wk_system.h"
#include "motor_board_config.h"
#if (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_ENCODER)
#include "encoder_rs485.h"
#endif

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
#define DEBUG_CLI(id, name, def, low, high) {#name, MOTOR_PARAMETER_FIELD_##id},
  MOTOR_PARAMETER_DEBUG_FIELDS(DEBUG_CLI)
#undef DEBUG_CLI
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
  {"hall_rotor_offset_u16", MOTOR_PARAMETER_FIELD_HALL_ROTOR_OFFSET_U16},
  {"encoder_electrical_zero_u16", MOTOR_PARAMETER_FIELD_ENCODER_ELECTRICAL_ZERO_U16},
  {"hall_delay_us", MOTOR_PARAMETER_FIELD_HALL_DELAY_COMPENSATION_US},
  {"flux_linkage_uwb", MOTOR_PARAMETER_FIELD_FLUX_LINKAGE_UWB},
  {"speed_maximum_rpm", MOTOR_PARAMETER_FIELD_SPEED_MAXIMUM_RPM},
  {"speed_acceleration_rpm_per_s",
   MOTOR_PARAMETER_FIELD_SPEED_ACCELERATION_RPM_PER_S},
  {"speed_current_limit_ma", MOTOR_PARAMETER_FIELD_SPEED_CURRENT_LIMIT_MA},
  {"speed_deceleration_rpm_per_s", MOTOR_PARAMETER_FIELD_SPEED_DECELERATION_RPM_PER_S},
  {"speed_startup_iq_slew_ma_s", MOTOR_PARAMETER_FIELD_SPEED_STARTUP_IQ_SLEW_MA_S}
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
  printf("current_pi_q15=%ld/%ld/%ld speed_pi_q20=%ld/%ld hall_offset_u16=%u hall_delay_us=%u flux_linkage_uwb=%lu\r\n",
         (long)parameter->current_d_kp_q15,
         (long)parameter->current_q_kp_q15,
         (long)parameter->current_ki_q15,
         (long)parameter->speed_kp_q20,
         (long)parameter->speed_ki_q20,
         (unsigned int)parameter->hall_rotor_offset_u16,
         (unsigned int)parameter->hall_delay_compensation_us,
         (unsigned long)parameter->flux_linkage_uwb);
  printf("speed_maximum_rpm=%lu speed_acceleration_rpm_per_s=%lu speed_current_limit_ma=%lu\r\n",
         (unsigned long)parameter->speed_maximum_rpm,
         (unsigned long)parameter->speed_acceleration_rpm_per_s,
         (unsigned long)parameter->speed_current_limit_ma);
#define DEBUG_PRINT(id, name, def, low, high) printf(#name "=%lu\r\n", (unsigned long)parameter->name);
  MOTOR_PARAMETER_DEBUG_FIELDS(DEBUG_PRINT)
#undef DEBUG_PRINT
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
         (1ULL << motor_cli_parameter_fields[index].field)) != 0U)
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
  {
    printf("hall_sequence: active=%u/%u/%u/%u/%u/%u candidate=%u/%u/%u/%u/%u/%u\r\n",
           (unsigned int)active.hall_positive_next[1],
           (unsigned int)active.hall_positive_next[2],
           (unsigned int)active.hall_positive_next[3],
           (unsigned int)active.hall_positive_next[4],
           (unsigned int)active.hall_positive_next[5],
           (unsigned int)active.hall_positive_next[6],
           (unsigned int)candidate.hall_positive_next[1],
           (unsigned int)candidate.hall_positive_next[2],
           (unsigned int)candidate.hall_positive_next[3],
           (unsigned int)candidate.hall_positive_next[4],
           (unsigned int)candidate.hall_positive_next[5],
           (unsigned int)candidate.hall_positive_next[6]);
  }
  if (diff.hall_angle_changed)
  {
    printf("hall_angle: active=%u/%u/%u/%u/%u/%u candidate=%u/%u/%u/%u/%u/%u\r\n",
           (unsigned int)active.hall_entry_angle_u16[1],
           (unsigned int)active.hall_entry_angle_u16[2],
           (unsigned int)active.hall_entry_angle_u16[3],
           (unsigned int)active.hall_entry_angle_u16[4],
           (unsigned int)active.hall_entry_angle_u16[5],
           (unsigned int)active.hall_entry_angle_u16[6],
           (unsigned int)candidate.hall_entry_angle_u16[1],
           (unsigned int)candidate.hall_entry_angle_u16[2],
           (unsigned int)candidate.hall_entry_angle_u16[3],
           (unsigned int)candidate.hall_entry_angle_u16[4],
           (unsigned int)candidate.hall_entry_angle_u16[5],
           (unsigned int)candidate.hall_entry_angle_u16[6]);
  }
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

#if (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_ENCODER)
#define ENC_ALIGN_RAMP_MS          200U
#define ENC_ALIGN_SETTLE_MS        400U
#define ENC_ALIGN_SAMPLE_COUNT     256U
#define ENC_ALIGN_SAMPLE_TIMEOUT   500U
#define ENC_ALIGN_MAX_SPREAD       (1U << (ENC_SINGLE_BITS - 14U))
#define ENC_ALIGN_SCAN_MS          109U
#define ENC_ALIGN_SCAN_POINTS      12U
#define ENC_ALIGN_SCAN_ANGLE       49152U

typedef enum
{
  ENC_ALIGN_IDLE = 0,
  ENC_ALIGN_RAMP,
  ENC_ALIGN_SETTLE,
  ENC_ALIGN_SAMPLE,
  ENC_ALIGN_SCAN
} enc_align_state_t;

static struct
{
  enc_align_state_t state;
  uint32_t phase_start_cycles;
  uint32_t first_pos;
  uint32_t last_update;
  int32_t delta_sum;
  int32_t delta_min;
  int32_t delta_max;
  uint32_t samples;
  uint32_t previous_pos;
  uint32_t scan_point;
  int32_t expected_step;
  int32_t voltage_mv;
  uint16_t zero;
  uint8_t pole_pairs;
} enc_align;

static int32_t enc_align_delta(uint32_t pos, uint32_t ref)
{
  int32_t d = (int32_t)pos - (int32_t)ref;
  if (d > (int32_t)(ENC_SINGLE_MAX / 2U)) d -= (int32_t)ENC_SINGLE_MAX;
  else if (d < -(int32_t)(ENC_SINGLE_MAX / 2U)) d += (int32_t)ENC_SINGLE_MAX;
  return d;
}

static void enc_align_abort(const char *reason)
{
  motor_control_stop();
  enc_align.state = ENC_ALIGN_IDLE;
  printf("ERR encoder_align_failed reason=%s\r\n", reason);
}

static void enc_align_phase_set(enc_align_state_t state)
{
  enc_align.phase_start_cycles = motor_timebase_cycles_get();
  enc_align.state = state;
}

static void enc_align_start(int32_t voltage_mv)
{
  motor_parameter_t candidate;
  enc_data_t enc;
  const uint8_t pole_pairs = motor_parameter_pole_pairs_get();

  if (enc_align.state != ENC_ALIGN_IDLE)
  {
    printf("ERR encoder_align_busy\r\n");
    return;
  }
  if (!motor_parameter_candidate_read(&candidate) || pole_pairs == 0U ||
      candidate.pole_pairs != pole_pairs)
  {
    printf("ERR encoder_align_requires_matching_pole_pairs\r\n");
    return;
  }
  (void)encoder_rs485_read(&enc);
  if (!enc.valid || motor_timebase_cycles_elapsed(enc.timestamp_cycles) >
      2U * system_core_clock / MOTOR_PWM_FREQUENCY_HZ)
  {
    printf("ERR encoder_align_requires_fresh_frame\r\n");
    return;
  }
  if (!motor_control_encoder_alignment_start())
  {
    printf("ERR encoder_align_open_loop_rejected\r\n");
    return;
  }

  (void)memset(&enc_align, 0, sizeof(enc_align));
  enc_align.voltage_mv = voltage_mv;
  enc_align.pole_pairs = pole_pairs;
  enc_align.expected_step = (int32_t)((uint64_t)ENC_SINGLE_MAX * ENC_ALIGN_SCAN_ANGLE /
    65536ULL / pole_pairs / (ENC_ALIGN_SCAN_POINTS - 1U));
  enc_align_phase_set(ENC_ALIGN_RAMP);
  printf("OK encoder align started vd=%ld mV scan_points=%u\r\n",
         (long)voltage_mv, (unsigned int)ENC_ALIGN_SCAN_POINTS);
}

static void enc_align_scan_next(void)
{
  ++enc_align.scan_point;
  if (enc_align.scan_point == ENC_ALIGN_SCAN_POINTS)
  {
    motor_control_stop();
    enc_align.state = ENC_ALIGN_IDLE;
    if (!motor_parameter_encoder_zero_apply(enc_align.zero, enc_align.pole_pairs))
    {
      printf("ERR encoder_align_failed reason=zero_apply_rejected\r\n");
      return;
    }
    printf("OK encoder align zero_u16=%u active=1 saved=0\r\n",
           (unsigned int)enc_align.zero);
    return;
  }

  /* Encoder positions have already been mapped to physical electrical direction. */
  const uint16_t angle = (uint16_t)(enc_align.scan_point * ENC_ALIGN_SCAN_ANGLE /
                                   (ENC_ALIGN_SCAN_POINTS - 1U));
  if (!motor_open_loop_alignment_vector_set(enc_align.voltage_mv, angle))
  {
    enc_align_abort("scan_voltage_rejected");
    return;
  }
  enc_align_phase_set(ENC_ALIGN_SCAN);
}

static void enc_align_poll(void)
{
  motor_control_status_t motor;
  enc_data_t enc;

  if (enc_align.state == ENC_ALIGN_IDLE) return;

  (void)motor_control_status_read(&motor);
  if (motor.state != MOTOR_CONTROL_STATE_OPEN_LOOP || !motor_pwm_port_fault_check())
  {
    enc_align_abort("open_loop_lost");
    return;
  }
  (void)encoder_rs485_read(&enc);
  if (!enc.valid || motor_timebase_cycles_elapsed(enc.timestamp_cycles) >
      2U * system_core_clock / MOTOR_PWM_FREQUENCY_HZ)
  {
    enc_align_abort("encoder_invalid");
    return;
  }
  const uint32_t elapsed_ms = motor_timebase_cycles_to_us(
    motor_timebase_cycles_elapsed(enc_align.phase_start_cycles)) / 1000U;

  switch (enc_align.state)
  {
    case ENC_ALIGN_RAMP:
    {
      const uint32_t ramp_ms = elapsed_ms < ENC_ALIGN_RAMP_MS ? elapsed_ms : ENC_ALIGN_RAMP_MS;
      const int32_t voltage = enc_align.voltage_mv * (int32_t)ramp_ms / (int32_t)ENC_ALIGN_RAMP_MS;
      if (!motor_open_loop_alignment_vector_set(voltage, 0U))
        enc_align_abort("ramp_voltage_rejected");
      else if (ramp_ms == ENC_ALIGN_RAMP_MS)
        enc_align_phase_set(ENC_ALIGN_SETTLE);
      break;
    }
    case ENC_ALIGN_SETTLE:
      if (elapsed_ms >= ENC_ALIGN_SETTLE_MS)
      {
        enc_align.last_update = enc.update_count;
        enc_align_phase_set(ENC_ALIGN_SAMPLE);
      }
      break;
    case ENC_ALIGN_SAMPLE:
      if (elapsed_ms >= ENC_ALIGN_SAMPLE_TIMEOUT)
      {
        enc_align_abort("sample_timeout");
        break;
      }
      if (enc.update_count != enc_align.last_update)
      {
        enc_align.last_update = enc.update_count;
        if (enc_align.samples == 0U) enc_align.first_pos = enc.abs_pos;
        const int32_t delta = enc_align_delta(enc.abs_pos, enc_align.first_pos);
        enc_align.delta_sum += delta;
        if (delta < enc_align.delta_min) enc_align.delta_min = delta;
        if (delta > enc_align.delta_max) enc_align.delta_max = delta;
        if (enc_align.delta_max - enc_align.delta_min > (int32_t)ENC_ALIGN_MAX_SPREAD)
        {
          enc_align_abort("rotor_not_still");
          break;
        }
        if (++enc_align.samples == ENC_ALIGN_SAMPLE_COUNT)
        {
          const uint32_t mean_pos = (uint32_t)((int32_t)enc_align.first_pos +
            enc_align.delta_sum / (int32_t)ENC_ALIGN_SAMPLE_COUNT) & (ENC_SINGLE_MAX - 1U);
          enc_align.zero = encoder_abs_to_electrical_angle_q16(mean_pos, enc_align.pole_pairs);
          enc_align.previous_pos = enc.abs_pos;
          enc_align_scan_next();
        }
      }
      break;
    case ENC_ALIGN_SCAN:
      if (elapsed_ms >= ENC_ALIGN_SCAN_MS)
      {
        const int32_t delta = enc_align_delta(enc.abs_pos, enc_align.previous_pos);
        if (delta < enc_align.expected_step / 3 || delta > enc_align.expected_step * 2)
        {
          enc_align_abort("scan_displacement");
          printf("ERR encoder_align_scan point=%lu delta=%ld expected=%ld\r\n",
                 (unsigned long)enc_align.scan_point, (long)delta, (long)enc_align.expected_step);
          break;
        }
        enc_align.previous_pos = enc.abs_pos;
        enc_align_scan_next();
      }
      break;
    default:
      break;
  }
}
#endif

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
  motor_current_startup_status_t current_startup;
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
  int32_t logical_open_actual_millihz;
  int32_t logical_open_target_millihz;
  int32_t logical_current_feedback_ma;
  int32_t logical_current_reference_ma;

  if (strcmp(line, "help") == 0)
  {
    printf("OK commands:\r\n");
    printf("  help | version | status | reset | encoder | encoder test | encoder align [vd_mv] | encoder zero\r\n");
    printf("  debug <0|1> | logid [10|20|30|40] | lograte <1..500> | logdetail <0|1|2> | logcapture [1..128]\r\n");
    printf("  fault | fault clear | log level <0..4>\r\n");
    printf("  motor stop | motor direction [normal|reverse]\r\n");
    printf("  open start|set <vd_mv> <vq_mv> <freq_mhz> <accel_mhz_s>\r\n");
    printf("  open stop\r\n");
    printf("  current start|set <id_ma> <iq_ma> | current stop\r\n");
    printf("  speed start|set <signed_rpm> | speed stop\r\n");
    printf("  motor params active|candidate|load|defaults|storage\r\n");
    printf("  motor params profile 60bldc140-24v (candidate initial values only)\r\n");
    printf("  motor param set <name> <value>\r\n");
    printf("  motor commissioning status|abort|diff|accept|discard|save\r\n");
    printf("  calibrate current_offset\r\n");
    printf("  calibrate hall sequence|angle|offset\r\n");
    printf("  identify resistance|inductance|flux | calculate current_pi\r\n");
  }
  else if (strcmp(line, "version") == 0)
  {
    printf("OK fw=%s hw=%s\r\n", FIRMWARE_VERSION_STRING,
           HARDWARE_VERSION_STRING);
  }
#if (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_ENCODER)
  else if (strcmp(line, "encoder") == 0)
  {
    enc_data_t enc;
    enc_stats_t enc_stats;
    motor_rotor_feedback_t enc_feedback;

    (void)encoder_rs485_read(&enc);
    (void)encoder_rs485_stats_read(&enc_stats);
    (void)memset(&enc_feedback, 0, sizeof(enc_feedback));
    (void)motor_rotor_feedback_read(&enc_feedback);
    printf("OK enc valid=%u pos=%lu sf=0x%02X result=%u frames=%lu "
           "req=%lu ok=%lu crc=%lu timeout=%lu sf_err=%lu "
           "rx_len=%u raw=%02X %02X %02X %02X %02X %02X %02X %02X "
           "elec=%u mech=%u speed_rpm=%ld fb_valid=%u spd_valid=%u\r\n",
      (unsigned int)enc.valid, (unsigned long)enc.abs_pos,
      (unsigned int)enc.status, (unsigned int)encoder_rs485_get_result(),
      (unsigned long)enc.update_count,
      (unsigned long)enc_stats.request_count,
      (unsigned long)enc_stats.ok_count,
      (unsigned long)enc_stats.crc_error_count,
      (unsigned long)enc_stats.timeout_count,
      (unsigned long)enc_stats.status_error_count,
      (unsigned int)enc_stats.last_rx_len,
      enc_stats.last_raw[0], enc_stats.last_raw[1], enc_stats.last_raw[2],
      enc_stats.last_raw[3], enc_stats.last_raw[4], enc_stats.last_raw[5],
      enc_stats.last_raw[6], enc_stats.last_raw[7],
      (unsigned int)enc_feedback.electrical_angle_u16,
      (unsigned int)enc_feedback.mechanical_angle_u16,
      (long)(enc_feedback.mechanical_speed_millirpm / 1000L),
      (unsigned int)enc_feedback.valid,
      (unsigned int)enc_feedback.speed_valid);
  }
  else if (strcmp(line, "encoder zero") == 0)
  {
    (void)motor_control_status_read(&motor);
    if ((motor.state != MOTOR_CONTROL_STATE_READY) ||
        motor_pwm_port_output_is_enabled())
      printf("ERR encoder_zero_requires_motor_ready_and_pwm_off\r\n");
    else if (!motor_rotor_feedback_encoder_zero_here())
      printf("ERR encoder_zero_requires_fresh_frame\r\n");
    else
      printf("OK encoder mechanical_zero=0 storage=ram\r\n");
  }
  else if (strncmp(line, "encoder align", 13) == 0)
  {
    long align_mv = (long)MOTOR_OPEN_LOOP_ALIGNMENT_VOLTAGE_MV;
    if ((line[13] != '\0') && (sscanf(&line[13], "%ld", &align_mv) != 1))
      printf("ERR encoder_align_usage: encoder align [vd_mv]\r\n");
    else if ((align_mv <= 0) || (align_mv > (long)MOTOR_OPEN_LOOP_MAXIMUM_VOLTAGE_MV))
      printf("ERR encoder_align_voltage_range 1..%ld\r\n",
             (long)MOTOR_OPEN_LOOP_MAXIMUM_VOLTAGE_MV);
    else
      enc_align_start((int32_t)align_mv);
  }
  else if (strcmp(line, "encoder test") == 0)
  {
    enc_poll_test_t t;
    uint8_t n;

    (void)encoder_rs485_poll_test(&t);
    printf("OK enctest tx_done=%u rx_len=%u idle_high=%u%% err=0x%02lX rx=",
      (unsigned int)t.tx_done, (unsigned int)t.rx_len,
      (unsigned int)t.rx_idle_high_pct, (unsigned long)t.err_sts);
    n = (t.rx_len > sizeof(t.rx)) ? (uint8_t)sizeof(t.rx) : t.rx_len;
    for (uint8_t i = 0U; i < n; i++)
      printf("%02X ", t.rx[i]);
    printf("\r\n  usart sts=%08lX ctrl1=%08lX ctrl3=%08lX "
           "dma_tx ctrl=%08lX cnt=%lu dma_rx ctrl=%08lX cnt=%lu\r\n",
      (unsigned long)t.usart_sts, (unsigned long)t.usart_ctrl1,
      (unsigned long)t.usart_ctrl3,
      (unsigned long)t.dma_tx_ctrl, (unsigned long)t.dma_tx_cnt,
      (unsigned long)t.dma_rx_ctrl, (unsigned long)t.dma_rx_cnt);
    printf("  wire tx_end=%lu.%lu us low=%lu falls=%lu first_low=%lu.%lu us "
           "last_low=%lu.%lu us usart_rx=%lu baudr=%lu "
           "pa_cfgr=%08lX pa_muxl=%08lX pa_pull=%08lX\r\n",
      (unsigned long)(t.wire_tx_end_100ns / 10U),
      (unsigned long)(t.wire_tx_end_100ns % 10U),
      (unsigned long)t.wire_low_samples, (unsigned long)t.wire_fall_edges,
      (unsigned long)(t.wire_first_low_100ns / 10U),
      (unsigned long)(t.wire_first_low_100ns % 10U),
      (unsigned long)(t.wire_last_low_100ns / 10U),
      (unsigned long)(t.wire_last_low_100ns % 10U),
      (unsigned long)t.wire_usart_rx_count,
      (unsigned long)USART3->baudr,
      (unsigned long)t.wire_gpioa_cfgr, (unsigned long)t.wire_gpioa_muxl,
      (unsigned long)t.wire_gpioa_pull);
  }
#endif
  else if (strcmp(line, "reset") == 0)
  {
    (void)motor_control_status_read(&motor);
    if ((motor.state == MOTOR_CONTROL_STATE_SPEED_PREPARING) ||
        (motor.state == MOTOR_CONTROL_STATE_CURRENT_STARTING) ||
        (motor.state == MOTOR_CONTROL_STATE_OPEN_LOOP) ||
        (motor.state == MOTOR_CONTROL_STATE_CURRENT_CONTROL) ||
        (motor.state == MOTOR_CONTROL_STATE_SPEED_CONTROL) ||
        motor_pwm_port_output_is_enabled())
    {
      printf("ERR reset_requires_motor_stopped_and_pwm_off\r\n");
      return;
    }
    printf("OK resetting\r\n");
    __DSB();
    NVIC_SystemReset();
    while (1) { }
  }
  else if (motor_trace_command(line))
  {
    /* Trace command processed. */
  }
  else if (strcmp(line, "status") == 0)
  {
    (void)motor_open_loop_status_read(&open_loop);
    (void)motor_current_control_status_read(&current_control);
    (void)motor_control_status_read(&motor);
    (void)motor_speed_feedback_read(&speed_feedback);
    (void)motor_speed_control_status_read(&speed_control);
    (void)motor_current_startup_status_read(&current_startup);
    (void)motor_direction_transform_s32(
      open_loop.actual_frequency_millihz,
      motor_parameter_direction_inverted_get(), &logical_open_actual_millihz);
    (void)motor_direction_transform_s32(
      open_loop.target_frequency_millihz,
      motor_parameter_direction_inverted_get(), &logical_open_target_millihz);
    (void)motor_direction_transform_s32(
      current_control.quadrature_feedback_ma,
      motor_parameter_direction_inverted_get(), &logical_current_feedback_ma);
    (void)motor_direction_transform_s32(
      current_control.command.quadrature_reference_ma,
      motor_parameter_direction_inverted_get(), &logical_current_reference_ma);
    printf("OK motor=%u motor_fault=%s(%u) direction=%s startup=%u/%u edges=%u open=%u freq_mhz=%ld/%ld speed_rpm=%ld/%ld speed_valid=%u speed_ctrl=%u/%u logical_iq_cmd=%ld current=%u fault=%u id=%ld/%ld iq=%ld/%ld uart_drop=%lu\r\n",
      (unsigned int)motor.state,
      motor_control_fault_name_get(motor.fault_code),
      (unsigned int)motor.fault_code,
      motor_parameter_direction_inverted_get() ? "reverse" : "normal",
      (unsigned int)current_startup.state,
      (unsigned int)current_startup.fault,
      (unsigned int)current_startup.qualified_edges,
      (unsigned int)open_loop.state,
      (long)logical_open_actual_millihz,
      (long)logical_open_target_millihz,
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
      (long)logical_current_feedback_ma,
      (long)logical_current_reference_ma,
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
  else if (strcmp(line, "motor params profile 60bldc140-24v") == 0)
  {
    if (motor_parameter_candidate_24v_profile())
      printf("OK 24V candidate only: Rs=80mOhm Ld/Lq=50uH bandwidth=888Hz PI=9141/9141/914; Hall/flux/speed_PI retained UNVERIFIED; calibrate before motion, accept/save separately\r\n");
    else printf("ERR profile_requires_ready_pwm_off\r\n");
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
  else if (strcmp(line, "identify flux") == 0)
  {
    motor_cli_commissioning_execute(MOTOR_COMMISSIONING_TASK_FLUX);
  }
  else if (strcmp(line, "calculate current_pi") == 0)
  {
    motor_cli_commissioning_execute(MOTOR_COMMISSIONING_TASK_CURRENT_PI);
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
    /* 超出最高转速时按最高转速执行，不报错。 */
    if (speed_rpm > (long)motor_speed_control_maximum_rpm_get())
      speed_rpm = (long)motor_speed_control_maximum_rpm_get();
    else if (speed_rpm < -(long)motor_speed_control_maximum_rpm_get())
      speed_rpm = -(long)motor_speed_control_maximum_rpm_get();
    if (speed_rpm == 0)
    {
      printf("ERR 3 invalid_parameter\r\n");
      return;
    }
    if (motor_control_speed_control_start((int32_t)speed_rpm))
      printf("OK speed requested target=%ld rpm (startup preparation may be pending)\r\n", speed_rpm);
    else
      printf("ERR 10 motor_not_ready_or_speed_start_failed\r\n");
  }
  else if (sscanf(line, "speed set %ld", &speed_rpm) == 1)
  {
    /* 超出最高转速时按最高转速执行，不报错。 */
    if (speed_rpm > (long)motor_speed_control_maximum_rpm_get())
      speed_rpm = (long)motor_speed_control_maximum_rpm_get();
    else if (speed_rpm < -(long)motor_speed_control_maximum_rpm_get())
      speed_rpm = -(long)motor_speed_control_maximum_rpm_get();
    if (speed_rpm == 0)
    {
      printf("ERR 3 invalid_parameter\r\n");
      return;
    }
    if (motor_control_speed_control_target_set((int32_t)speed_rpm))
      printf("OK speed updated target=%ld rpm\r\n", speed_rpm);
    else
      printf("ERR 11 speed_not_running_invalid_or_direction_change\r\n");
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

  motor_trace_poll();
#if (MOTOR_POS_SENSOR_TYPE == MOTOR_POS_SENSOR_ENCODER)
  enc_align_poll();
#endif
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
