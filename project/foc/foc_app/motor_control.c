/**
 * @file    motor_control.c
 * @brief   电机统一控制状态机 功能实现
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#include <stddef.h>
#include "at32f45x.h"
#include "motor_control.h"
#include "motor_control_config.h"
#include "motor_current_control.h"
#include "motor_current_startup.h"
#include "motor_current_sample.h"
#include "motor_current_transform.h"
#include "motor_direction.h"
#include "motor_foc_math.h"
#include "motor_hall_angle_estimator.h"
#include "motor_hall_decoder.h"
#include "motor_rotor_feedback.h"
#include "motor_log.h"
#include "motor_parameter.h"
#include "motor_pwm_port.h"
#include "motor_slow_sensor.h"
#include "motor_speed_control.h"
#include "motor_speed_feedback.h"
#include "motor_board_config.h"
#include "wk_system.h"
#include "motor_flying_seed.h"
#include "motor_hall_port.h"
#include "motor_timebase.h"
static volatile motor_control_status_t motor_control_status;
static motor_fault_record_t motor_fault_history[MOTOR_FAULT_HISTORY_DEPTH];
static uint32_t motor_fault_history_count;
static uint32_t motor_fault_history_newest;
static uint32_t motor_fault_sequence;
static volatile uint32_t motor_fault_pending_mask;
static bool motor_bus_undervoltage_pending;
static uint32_t motor_bus_undervoltage_start_cycles;
static int32_t motor_speed_prepare_target_rpm;
static uint32_t motor_speed_prepare_cycles;
static uint32_t motor_speed_prepare_sample_count;
static uint8_t motor_speed_prepare_phase;

static motor_fault_clear_result_t motor_control_driver_recover(void)
{
  motor_current_sample_state_t before;
  motor_current_sample_state_t after;
  bool had_sample = motor_current_sample_state_read(&before);
  motor_pwm_port_recovery_prepare();
  motor_pwm_port_gate_driver_set(true);
  wk_delay_ms(MOTOR_GATE_DRIVER_WAKE_DELAY_MS);
  if (!motor_pwm_port_fault_check())
  {
    motor_pwm_port_emergency_stop();
    return MOTOR_FAULT_CLEAR_DRIVER_ACTIVE;
  }
  /* ADC triggering continues with MOE off. Require a fresh CSA sample after wake. */
  (void)motor_current_sample_state_read(&before);
  wk_delay_ms(1U);
  if (!had_sample || !motor_current_sample_state_read(&after) ||
      after.sample_count == before.sample_count || !motor_current_sample_fault_clear())
  {
    motor_pwm_port_emergency_stop();
    return MOTOR_FAULT_CLEAR_CURRENT_UNSAFE;
  }
  if (motor_fault_pending_mask != 0U || !motor_pwm_port_fault_check())
  {
    motor_pwm_port_emergency_stop();
    return MOTOR_FAULT_CLEAR_DRIVER_ACTIVE;
  }
  return MOTOR_FAULT_CLEAR_OK;
}

static bool motor_control_bus_voltage_check(void)
{
  motor_slow_sensor_state_t sensor;
  if (!motor_slow_sensor_state_read(&sensor))
  {
    motor_bus_undervoltage_pending = false;
    return true;
  }
  if (sensor.bus_voltage_0p1v < MOTOR_BUS_UNDERVOLTAGE_0P1V)
  {
    if (!motor_bus_undervoltage_pending)
    {
      motor_bus_undervoltage_start_cycles = motor_timebase_cycles_get();
      motor_bus_undervoltage_pending = true;
    }
    if (motor_timebase_cycles_to_us(motor_timebase_cycles_elapsed(
          motor_bus_undervoltage_start_cycles)) >=
        MOTOR_BUS_UNDERVOLTAGE_CONFIRM_MS * 1000U)
    {
      motor_control_fault_set(MOTOR_FAULT_BUS_UNDERVOLTAGE);
      return false;
    }
    return true;
  }
  motor_bus_undervoltage_pending = false;
  if (sensor.bus_voltage_0p1v > MOTOR_BUS_OVERVOLTAGE_0P1V)
  {
    motor_control_fault_set(MOTOR_FAULT_BUS_OVERVOLTAGE);
    return false;
  }
  return true;
}

/* Check before PWM startup and during each main-loop poll. */
static bool motor_control_mos_temperature_check(void)
{
  motor_slow_sensor_state_t sensor;
  (void)motor_slow_sensor_process();
  if ((!motor_slow_sensor_state_read(&sensor)) ||
      (!sensor.mos_temperature_valid))
  {
    motor_control_fault_set(MOTOR_FAULT_MOS_TEMPERATURE_SENSOR);
    return false;
  }
  if (sensor.mos_temperature_c >= MOTOR_MOS_OVERTEMPERATURE_C)
  {
    LOGE("MOS overtemperature: temperature=%d C\r\n",
         (int)sensor.mos_temperature_c);
    motor_control_fault_set(MOTOR_FAULT_MOS_OVERTEMPERATURE);
    return false;
  }
  return true;
}

static bool motor_control_open_command_to_physical(
  const motor_open_loop_command_t *logical_command,
  motor_open_loop_command_t *physical_command)
{
  if ((logical_command == NULL) || (physical_command == NULL)) return false;
  *physical_command = *logical_command;
  return motor_direction_transform_s32(
    logical_command->target_frequency_millihz,
    motor_parameter_direction_inverted_get(),
    &physical_command->target_frequency_millihz);
}

static bool motor_control_current_command_to_physical(
  const motor_current_control_command_t *logical_command,
  motor_current_control_command_t *physical_command)
{
  if ((logical_command == NULL) || (physical_command == NULL)) return false;
  *physical_command = *logical_command;
  return motor_direction_transform_s32(
    logical_command->quadrature_reference_ma,
    motor_parameter_direction_inverted_get(),
    &physical_command->quadrature_reference_ma);
}

const char *motor_control_fault_name_get(motor_fault_code_t fault_code)
{
  switch (fault_code)
  {
    case MOTOR_FAULT_NONE: return "none";
    case MOTOR_FAULT_UNKNOWN: return "unknown";
    case MOTOR_FAULT_CONTROL_SUBSYSTEM: return "control_subsystem";
    case MOTOR_FAULT_READY_TRANSITION: return "ready_transition";
    case MOTOR_FAULT_CURRENT_OFFSET_INVALID: return "current_offset_invalid";
    case MOTOR_FAULT_CURRENT_CALIBRATION_TIMEOUT: return "current_calibration_timeout";
    case MOTOR_FAULT_DRIVER_NOT_READY: return "driver_not_ready";
    case MOTOR_FAULT_CURRENT_HANDOVER: return "current_handover";
    case MOTOR_FAULT_SPEED_START: return "speed_start";
    case MOTOR_FAULT_OPEN_LOOP: return "open_loop";
    case MOTOR_FAULT_CURRENT_SAMPLE: return "current_sample";
    case MOTOR_FAULT_HALL: return "hall";
    case MOTOR_FAULT_OVERCURRENT: return "overcurrent";
    case MOTOR_FAULT_PWM: return "pwm";
    case MOTOR_FAULT_SPEED_FEEDBACK: return "speed_feedback";
    case MOTOR_FAULT_SPEED_CURRENT_CONTROL: return "speed_current_control";
    case MOTOR_FAULT_SPEED_COMMAND: return "speed_command";
    case MOTOR_FAULT_REVERSE_DIRECTION: return "reverse_direction";
    case MOTOR_FAULT_STALL: return "stall";
    case MOTOR_FAULT_OVERSPEED: return "overspeed";
    case MOTOR_FAULT_HARDWARE_BREAK: return "hardware_break";
    case MOTOR_FAULT_MOS_OVERTEMPERATURE: return "mos_overtemperature";
    case MOTOR_FAULT_MOS_TEMPERATURE_SENSOR: return "mos_temperature_sensor";
    case MOTOR_FAULT_BUS_UNDERVOLTAGE: return "bus_undervoltage";
    case MOTOR_FAULT_BUS_OVERVOLTAGE: return "bus_overvoltage";
    default: return "invalid";
  }
}

/**
 * @brief 校验有符号逻辑转速并执行物理反向安全门判断。
 * @param target_speed_rpm 待校验逻辑机械转速，单位rpm且不能为0。
 * @return 范围合法且不会触发未验证物理反向控制时返回true。
 */
static bool motor_control_speed_command_valid(int32_t target_speed_rpm)
{
  int32_t physical_speed_rpm;
  int64_t absolute_speed_rpm = (target_speed_rpm < 0) ?
    -(int64_t)target_speed_rpm : (int64_t)target_speed_rpm;

  if ((target_speed_rpm == 0) ||
      (absolute_speed_rpm > motor_speed_control_maximum_rpm_get()) ||
      (absolute_speed_rpm > (INT32_MAX / 1000)) ||
      (!motor_direction_transform_s32(
        target_speed_rpm, motor_parameter_direction_inverted_get(),
        &physical_speed_rpm)))
  {
    return false;
  }
#if (MOTOR_REVERSE_CONTROL_VERIFIED == 0U)
  if (physical_speed_rpm < 0) return false;
#endif
  return true;
}

void motor_control_init(void)
{
  motor_bus_undervoltage_pending = false;
  uint32_t index;

  motor_control_status.state = MOTOR_CONTROL_STATE_STARTUP;
  motor_control_status.fault_code = MOTOR_FAULT_NONE;
  motor_fault_history_count = 0U;
  motor_fault_history_newest = 0U;
  motor_fault_sequence = 0U;
  motor_fault_pending_mask = 0U;
  motor_current_startup_init();
  for (index = 0U; index < MOTOR_FAULT_HISTORY_DEPTH; index++)
  {
    motor_fault_history[index].first_code = MOTOR_FAULT_NONE;
    motor_fault_history[index].last_code = MOTOR_FAULT_NONE;
    motor_fault_history[index].code_mask = 0U;
    motor_fault_history[index].occurrence_count = 0U;
    motor_fault_history[index].sequence = 0U;
  }
}
bool motor_control_ready_set(void)
{
  if ((motor_control_status.state != MOTOR_CONTROL_STATE_STARTUP) ||
      motor_pwm_port_output_is_enabled()) return false;
  motor_control_status.state = MOTOR_CONTROL_STATE_READY;
  return true;
}
bool motor_control_open_loop_start(const motor_open_loop_command_t *command)
{
  motor_open_loop_command_t physical_command;
  if (!motor_control_mos_temperature_check()) return false;
  if ((motor_control_status.state != MOTOR_CONTROL_STATE_READY) ||
      (!motor_control_open_command_to_physical(command, &physical_command)) ||
      (!motor_open_loop_start(&physical_command))) return false;
  motor_control_status.state = MOTOR_CONTROL_STATE_OPEN_LOOP;
  return true;
}
bool motor_control_open_loop_command_set(const motor_open_loop_command_t *command)
{
  motor_open_loop_command_t physical_command;
  return (motor_control_status.state == MOTOR_CONTROL_STATE_OPEN_LOOP) &&
         motor_control_open_command_to_physical(command, &physical_command) &&
         motor_open_loop_command_set(&physical_command);
}
bool motor_control_current_control_start(
  const motor_current_control_command_t *command)
{
  motor_current_control_command_t physical_command;
  motor_open_loop_status_t open_loop;
  motor_rotor_feedback_t rotor_feedback;
  motor_slow_sensor_state_t sensor;
  motor_alpha_beta_q15_t alpha_beta;
  motor_direct_quadrature_q15_t voltage_dq;
  uint32_t bus_mv;
  int32_t direct_seed_mv;
  int32_t quadrature_seed_mv;
  uint32_t primask;
  bool handover_ok;

  if (!motor_control_mos_temperature_check()) return false;
  (void)motor_slow_sensor_process();
  if (!motor_control_current_command_to_physical(command, &physical_command))
    return false;
  if (motor_control_status.state == MOTOR_CONTROL_STATE_READY)
  {
    if (!motor_current_startup_start(&physical_command)) return false;
    motor_control_status.state = MOTOR_CONTROL_STATE_CURRENT_STARTING;
    return true;
  }
  if ((motor_control_status.state != MOTOR_CONTROL_STATE_OPEN_LOOP) ||
      (!motor_open_loop_status_read(&open_loop)) ||
      (open_loop.state != MOTOR_OPEN_LOOP_RUNNING) ||
      (!motor_rotor_feedback_read(&rotor_feedback)) ||
      (!rotor_feedback.valid) ||
      (!motor_slow_sensor_state_read(&sensor))) return false;
  bus_mv = (uint32_t)sensor.bus_voltage_0p1v * 100U;
  alpha_beta = motor_foc_inverse_park_q15(
    (int16_t)(((int64_t)open_loop.applied_direct_voltage_mv * 32768) / bus_mv),
    (int16_t)(((int64_t)open_loop.applied_quadrature_voltage_mv * 32768) / bus_mv),
    open_loop.electrical_angle_u16);
  voltage_dq = motor_foc_park_q15(
    alpha_beta, rotor_feedback.electrical_angle_u16);
  direct_seed_mv = (int32_t)(((int64_t)voltage_dq.direct_q15 * bus_mv) / 32768);
  quadrature_seed_mv = (int32_t)(
    ((int64_t)voltage_dq.quadrature_q15 * bus_mv) / 32768);
  /* 开环释放和电流环接管之间不允许被快速控制中断打断。 */
  primask = __get_PRIMASK();
  __disable_irq();
  handover_ok = motor_open_loop_control_release() &&
    motor_current_control_handover(&physical_command, direct_seed_mv,
                                   quadrature_seed_mv);
  __set_PRIMASK(primask);
  if (!handover_ok)
  {
    motor_control_fault_set(MOTOR_FAULT_CURRENT_HANDOVER);
    return false;
  }
  motor_control_status.state = MOTOR_CONTROL_STATE_CURRENT_CONTROL;
  return true;
}
bool motor_control_current_control_command_set(
  const motor_current_control_command_t *command)
{
  motor_current_control_command_t physical_command;
  if (!motor_control_current_command_to_physical(command, &physical_command))
    return false;
  if (motor_control_status.state == MOTOR_CONTROL_STATE_CURRENT_STARTING)
    return motor_current_startup_command_set(&physical_command);
  return (motor_control_status.state == MOTOR_CONTROL_STATE_CURRENT_CONTROL) &&
         motor_current_control_command_set(&physical_command);
}
static bool motor_control_speed_control_start_now(int32_t target_speed_rpm)
{
  motor_hall_sample_t startup_hall;
  uint32_t startup_primask;
  motor_current_control_command_t current_command;
  motor_current_control_status_t current_status;
  motor_current_transform_state_t current_transform;
  int32_t initial_quadrature_current_ma;
  motor_rotor_feedback_t flying_rotor;
  motor_parameter_t flying_parameter;
  motor_slow_sensor_state_t flying_sensor;
  int32_t flying_vq_mv;
  int32_t flying_limit_mv;

  if (!motor_control_mos_temperature_check()) return false;
  if (!motor_control_speed_command_valid(target_speed_rpm))
  {
    return false;
  }

  if (motor_control_status.state == MOTOR_CONTROL_STATE_READY)
  {
    startup_primask = __get_PRIMASK();
    __disable_irq();
    if (motor_hall_decoder_sample_read(&startup_hall) && startup_hall.valid &&
        motor_speed_feedback_standstill_confirmed())
    {
      motor_hall_decoder_motion_reset(motor_timebase_cycles_get());
      motor_hall_angle_estimator_motion_reset();
      motor_speed_feedback_init();
    }
    __set_PRIMASK(startup_primask);
    /* Iq=0启动电流环(静止时转子反馈回退为Hall扇区中心角)，
     * 随后立即启动速度PI，由其积分建立启动转矩。 */
    current_command.direct_reference_ma = 0;
    current_command.quadrature_reference_ma = 0;
    flying_vq_mv = 0;
    if (!motor_rotor_feedback_read(&flying_rotor) ||
        !motor_parameter_active_read(&flying_parameter) ||
        !motor_slow_sensor_state_read(&flying_sensor)) return false;
    if (flying_rotor.speed_valid)
    {
      if (((uint64_t)flying_rotor.electrical_frequency_millihz * 60U) >
          ((uint64_t)MOTOR_SPEED_OVERSPEED_RPM * flying_parameter.pole_pairs * 1000U))
        return false;
      /* Configured flux gives a back-EMF feedforward seed; the current loop
       * corrects it immediately. This is not a measured-voltage match. */
      flying_limit_mv = (int32_t)((uint32_t)flying_sensor.bus_voltage_0p1v *
        MOTOR_CURRENT_CONTROL_VOLTAGE_LIMIT_PERCENT);
      if (flying_limit_mv > MOTOR_CURRENT_CONTROL_HARD_VOLTAGE_LIMIT_MV)
        flying_limit_mv = MOTOR_CURRENT_CONTROL_HARD_VOLTAGE_LIMIT_MV;
      if (!motor_flying_seed_mv(flying_parameter.flux_linkage_uwb,
          flying_rotor.electrical_frequency_millihz, flying_rotor.direction,
          flying_limit_mv, &flying_vq_mv)) return false;
    }
    startup_primask = __get_PRIMASK();
    __disable_irq();
    motor_speed_feedback_process_1khz();
    __set_PRIMASK(startup_primask);
    startup_primask = __get_PRIMASK();
    __disable_irq();
    if (motor_control_status.state != MOTOR_CONTROL_STATE_READY ||
        motor_fault_pending_mask != 0U ||
        !motor_current_control_start_seeded(&current_command, 0, flying_vq_mv, 0U))
    {
      __set_PRIMASK(startup_primask);
      return false;
    }
    if (!motor_speed_control_start(target_speed_rpm * 1000L, 0))
    {
      motor_current_control_stop();
      __set_PRIMASK(startup_primask);
      return false;
    }
    motor_control_status.state = MOTOR_CONTROL_STATE_SPEED_CONTROL;
    __set_PRIMASK(startup_primask);
    return true;
  }

  if (motor_control_status.state == MOTOR_CONTROL_STATE_OPEN_LOOP)
  {
    if ((!motor_current_transform_state_read(&current_transform)) ||
        (!current_transform.valid))
    {
      return false;
    }
    initial_quadrature_current_ma = (int32_t)(
      ((int64_t)current_transform.quadrature_q15 *
       MOTOR_CURRENT_Q15_BASE_MA) / 32768L);
    if (initial_quadrature_current_ma > MOTOR_CURRENT_COMMAND_MAX_MA)
      initial_quadrature_current_ma = MOTOR_CURRENT_COMMAND_MAX_MA;
    else if (initial_quadrature_current_ma < -MOTOR_CURRENT_COMMAND_MAX_MA)
      initial_quadrature_current_ma = -MOTOR_CURRENT_COMMAND_MAX_MA;

    current_command.direct_reference_ma = 0;
    if (!motor_direction_transform_s32(
          initial_quadrature_current_ma,
          motor_parameter_direction_inverted_get(),
          &current_command.quadrature_reference_ma)) return false;
    if (!motor_control_current_control_start(&current_command)) return false;
  }
  else if (motor_control_status.state == MOTOR_CONTROL_STATE_CURRENT_CONTROL)
  {
    if ((!motor_current_control_status_read(&current_status)) ||
        (current_status.state != MOTOR_CURRENT_CONTROL_RUNNING))
    {
      return false;
    }
    initial_quadrature_current_ma =
      current_status.command.quadrature_reference_ma;
  }
  else
  {
    return false;
  }

  if (!motor_speed_control_start(target_speed_rpm * 1000L,
                                 initial_quadrature_current_ma))
  {
    motor_control_fault_set(MOTOR_FAULT_SPEED_START);
    return false;
  }
  motor_control_status.state = MOTOR_CONTROL_STATE_SPEED_CONTROL;
  return true;
}
bool motor_control_speed_control_start(int32_t target_speed_rpm)
{
  uint32_t primask;
  if (motor_control_status.state != MOTOR_CONTROL_STATE_READY)
    return motor_control_speed_control_start_now(target_speed_rpm);
  if (!motor_control_speed_command_valid(target_speed_rpm) ||
      !motor_control_mos_temperature_check() || !motor_control_bus_voltage_check()) return false;
  primask = __get_PRIMASK();
  __disable_irq();
  if (motor_control_status.state != MOTOR_CONTROL_STATE_READY || motor_fault_pending_mask)
  { __set_PRIMASK(primask); return false; }
  motor_speed_prepare_target_rpm = target_speed_rpm;
  motor_speed_prepare_phase = 0U;
  motor_speed_prepare_cycles = motor_timebase_cycles_get();
  motor_pwm_port_output_disable();
  motor_pwm_port_gate_driver_set(true);
  motor_control_status.state = MOTOR_CONTROL_STATE_SPEED_PREPARING;
  __set_PRIMASK(primask);
  return true;
}

static void motor_control_speed_prepare_poll(void)
{
  motor_current_sample_state_t sample;
  motor_speed_feedback_t speed;
  motor_pwm_compare_t low = {0U, 0U, 0U};
  uint32_t elapsed = motor_timebase_cycles_to_us(
    motor_timebase_cycles_elapsed(motor_speed_prepare_cycles));
  if (motor_control_status.state != MOTOR_CONTROL_STATE_SPEED_PREPARING) return;
  if (elapsed > (motor_speed_prepare_phase == 0U ? 1500000U : 100000U))
  { motor_control_fault_set(MOTOR_FAULT_SPEED_START); return; }
  if (motor_speed_prepare_phase == 0U)
  {
    if (elapsed < MOTOR_GATE_DRIVER_WAKE_DELAY_MS * 1000U) return;
    if (!motor_pwm_port_fault_check() || !motor_current_sample_state_read(&sample))
    { motor_control_fault_set(MOTOR_FAULT_SPEED_START); return; }
    if (!motor_speed_feedback_standstill_confirmed() &&
        (!motor_speed_feedback_read(&speed) || !speed.valid)) return;
    motor_speed_prepare_sample_count = sample.sample_count;
    if (motor_speed_feedback_standstill_confirmed())
    {
      motor_pwm_port_compare_set(&low);
      motor_speed_prepare_phase = 1U;
    }
    else motor_speed_prepare_phase = 3U;
    motor_speed_prepare_cycles = motor_timebase_cycles_get();
    return;
  }
  if (elapsed < 1000U) return;
  if (motor_speed_prepare_phase == 1U)
  {
    /* Let zero compare values latch with MOE off, then recheck standstill. */
    if (!motor_speed_feedback_standstill_confirmed())
      motor_speed_prepare_phase = 3U;
    else if (!motor_pwm_port_fault_check() || !motor_pwm_port_output_enable())
    { motor_control_fault_set(MOTOR_FAULT_PWM); return; }
    else motor_speed_prepare_phase = 2U;
    motor_speed_prepare_cycles = motor_timebase_cycles_get();
    return;
  }
  if (motor_speed_prepare_phase == 2U)
  {
    motor_pwm_port_output_disable();
    motor_speed_prepare_phase = 3U;
    if (!motor_current_sample_state_read(&sample))
    { motor_control_fault_set(MOTOR_FAULT_CURRENT_SAMPLE); return; }
    motor_speed_prepare_sample_count = sample.sample_count;
    motor_speed_prepare_cycles = motor_timebase_cycles_get();
    return;
  }
  if (!motor_pwm_port_fault_check())
  { motor_control_fault_set(MOTOR_FAULT_DRIVER_NOT_READY); return; }
  if (!motor_current_sample_state_read(&sample) ||
      sample.sample_count == motor_speed_prepare_sample_count) return;
  motor_control_status.state = MOTOR_CONTROL_STATE_READY;
  if (!motor_control_speed_control_start_now(motor_speed_prepare_target_rpm))
    motor_control_fault_set(MOTOR_FAULT_SPEED_START);
}

bool motor_control_speed_control_target_set(int32_t target_speed_rpm)
{
  if (motor_control_status.state == MOTOR_CONTROL_STATE_SPEED_PREPARING)
  {
    if (!motor_control_speed_command_valid(target_speed_rpm)) return false;
    motor_speed_prepare_target_rpm = target_speed_rpm;
    return true;
  }
  if ((motor_control_status.state != MOTOR_CONTROL_STATE_SPEED_CONTROL) ||
      (!motor_control_speed_command_valid(target_speed_rpm)))
  {
    return false;
  }

  /* The speed controller decelerates to zero and confirms standstill before reversal. */
  return motor_speed_control_target_set(target_speed_rpm * 1000L);
}
void motor_control_stop(void)
{
  uint32_t primask = __get_PRIMASK();
  motor_control_state_t previous_state;
  __disable_irq();
  previous_state = motor_control_status.state;
  motor_speed_control_stop();
  motor_current_startup_stop();
  motor_current_control_stop();
  motor_open_loop_stop();
  motor_speed_prepare_phase = 0U;
  if ((previous_state == MOTOR_CONTROL_STATE_SPEED_PREPARING) ||
      (previous_state == MOTOR_CONTROL_STATE_CURRENT_STARTING) ||
      (previous_state == MOTOR_CONTROL_STATE_OPEN_LOOP) ||
      (previous_state == MOTOR_CONTROL_STATE_CURRENT_CONTROL) ||
      (previous_state == MOTOR_CONTROL_STATE_SPEED_CONTROL))
    motor_control_status.state = MOTOR_CONTROL_STATE_READY;
  __set_PRIMASK(primask);
}
void motor_control_fault_set(motor_fault_code_t fault_code)
{
  motor_fault_record_t *record;
  uint32_t bit;
  uint32_t primask;
  bool should_log = false;

  if ((fault_code <= MOTOR_FAULT_NONE) ||
      (fault_code > MOTOR_FAULT_BUS_OVERVOLTAGE))
    fault_code = MOTOR_FAULT_UNKNOWN;
  motor_pwm_port_emergency_stop();
  bit = 1UL << (uint32_t)fault_code;
  primask = __get_PRIMASK();
  __disable_irq();
  if ((motor_control_status.state != MOTOR_CONTROL_STATE_FAULT) ||
      (motor_fault_history_count == 0U))
  {
    if (motor_fault_history_count != 0U)
      motor_fault_history_newest =
        (motor_fault_history_newest + 1U) % MOTOR_FAULT_HISTORY_DEPTH;
    if (motor_fault_history_count < MOTOR_FAULT_HISTORY_DEPTH)
      motor_fault_history_count++;
    motor_fault_sequence++;
    record = &motor_fault_history[motor_fault_history_newest];
    record->first_code = fault_code;
    record->last_code = fault_code;
    record->code_mask = bit;
    record->occurrence_count = 1U;
    record->sequence = motor_fault_sequence;
    motor_control_status.fault_code = fault_code;
    should_log = true;
  }
  else
  {
    record = &motor_fault_history[motor_fault_history_newest];
    if ((record->code_mask & bit) == 0U)
    {
      record->code_mask |= bit;
      record->last_code = fault_code;
      record->occurrence_count++;
      should_log = true;
    }
  }
  motor_control_status.state = MOTOR_CONTROL_STATE_FAULT;
  __set_PRIMASK(primask);
  if (should_log)
  {
    LOGE("Motor fault: first=%s(%u) new=%s(%u) mask=0x%08lX sequence=%lu\r\n",
      motor_control_fault_name_get(record->first_code),
      (unsigned int)record->first_code,
      motor_control_fault_name_get(fault_code),
      (unsigned int)fault_code,
      (unsigned long)record->code_mask,
      (unsigned long)record->sequence);
  }
}

void motor_control_fault_notify_from_isr(motor_fault_code_t fault_code)
{
  uint32_t bit;

  motor_pwm_port_emergency_stop();
  if ((fault_code <= MOTOR_FAULT_NONE) ||
      (fault_code > MOTOR_FAULT_BUS_OVERVOLTAGE))
  {
    fault_code = MOTOR_FAULT_UNKNOWN;
  }
  bit = 1UL << (uint32_t)fault_code;
  motor_fault_pending_mask |= bit;
  __DSB();
}
const char *motor_control_fault_clear_result_name_get(
  motor_fault_clear_result_t result)
{
  switch (result)
  {
    case MOTOR_FAULT_CLEAR_OK: return "ok";
    case MOTOR_FAULT_CLEAR_NOT_FAULTED: return "not_faulted";
    case MOTOR_FAULT_CLEAR_PWM_ENABLED: return "pwm_enabled";
    case MOTOR_FAULT_CLEAR_RESET_REQUIRED: return "reset_required";
    case MOTOR_FAULT_CLEAR_CURRENT_UNSAFE: return "current_unsafe";
    case MOTOR_FAULT_CLEAR_DRIVER_ACTIVE: return "driver_fault_active";
    case MOTOR_FAULT_CLEAR_MOS_TEMPERATURE_UNSAFE: return "mos_temperature_unsafe";
    case MOTOR_FAULT_CLEAR_BUS_VOLTAGE_UNSAFE: return "bus_voltage_unsafe";
    case MOTOR_FAULT_CLEAR_HALL_UNSAFE: return "hall_unsafe";
    default: return "invalid";
  }
}

motor_fault_clear_result_t motor_control_fault_clear_ex(void)
{
  motor_slow_sensor_state_t sensor;
  motor_fault_clear_result_t recovery;
  uint32_t recovery_primask;
  motor_parameter_t hall_parameter;
  uint8_t hall_raw;
  if (motor_control_status.state != MOTOR_CONTROL_STATE_FAULT)
    return MOTOR_FAULT_CLEAR_NOT_FAULTED;
  if (motor_pwm_port_output_is_enabled())
    return MOTOR_FAULT_CLEAR_PWM_ENABLED;
  hall_raw=motor_hall_port_raw_state_get();
  if (hall_raw<1U || hall_raw>6U) return MOTOR_FAULT_CLEAR_HALL_UNSAFE;
  (void)motor_slow_sensor_process();
  if ((!motor_slow_sensor_state_read(&sensor)) ||
      (!sensor.mos_temperature_valid) ||
      (sensor.mos_temperature_c > MOTOR_MOS_TEMPERATURE_RECOVERY_C))
    return MOTOR_FAULT_CLEAR_MOS_TEMPERATURE_UNSAFE;
  if ((sensor.bus_voltage_0p1v < MOTOR_BUS_UNDERVOLTAGE_0P1V) ||
      (sensor.bus_voltage_0p1v > MOTOR_BUS_OVERVOLTAGE_0P1V))
    return MOTOR_FAULT_CLEAR_BUS_VOLTAGE_UNSAFE;
  if ((motor_control_status.fault_code == MOTOR_FAULT_READY_TRANSITION) ||
      (motor_control_status.fault_code == MOTOR_FAULT_CURRENT_OFFSET_INVALID) ||
      (motor_control_status.fault_code == MOTOR_FAULT_CURRENT_CALIBRATION_TIMEOUT) ||
      (motor_control_status.fault_code == MOTOR_FAULT_DRIVER_NOT_READY))
    return MOTOR_FAULT_CLEAR_RESET_REQUIRED;
  motor_speed_control_stop();
  motor_current_startup_stop();
  motor_current_control_stop();
  motor_open_loop_stop();
  recovery = motor_control_driver_recover();
  if (recovery != MOTOR_FAULT_CLEAR_OK) return recovery;
  recovery_primask = __get_PRIMASK();
  __disable_irq();
  if (motor_fault_pending_mask != 0U || !motor_pwm_port_fault_check())
  {
    motor_pwm_port_emergency_stop();
    __set_PRIMASK(recovery_primask);
    return MOTOR_FAULT_CLEAR_DRIVER_ACTIVE;
  }
  motor_control_status.fault_code = MOTOR_FAULT_NONE;
  if (motor_parameter_active_read(&hall_parameter))
  {
    (void)motor_hall_decoder_init(hall_parameter.hall_positive_next,system_core_clock,
      motor_hall_port_raw_state_get(),motor_timebase_cycles_get());
    motor_hall_angle_estimator_motion_reset();
    motor_speed_feedback_init();
  }
  motor_control_status.state = MOTOR_CONTROL_STATE_READY;
  __set_PRIMASK(recovery_primask);
  return MOTOR_FAULT_CLEAR_OK;
}

void motor_control_poll(void)
{
  motor_open_loop_status_t open_loop;
  motor_current_control_status_t current_control;
  motor_speed_control_status_t speed_control;
  uint32_t pending_mask;
  uint32_t fault_index;
  uint32_t primask;
  motor_current_sample_state_t current;
  motor_current_startup_status_t current_startup;
  motor_hall_sample_t hall;
  motor_hall_angle_estimator_t estimator;
  bool hall_read_ok;
  bool estimator_read_ok;

  primask = __get_PRIMASK();
  __disable_irq();
  pending_mask = motor_fault_pending_mask;
  motor_fault_pending_mask = 0U;
  __set_PRIMASK(primask);
  for (fault_index = 1U;
       fault_index <= (uint32_t)MOTOR_FAULT_BUS_OVERVOLTAGE;
       fault_index++)
  {
    if ((pending_mask & (1UL << fault_index)) != 0U)
    {
      if ((fault_index == (uint32_t)MOTOR_FAULT_OVERCURRENT) &&
          motor_current_sample_state_read(&current))
      {
        LOGE("Overcurrent trip diag: trip=%ld/%ld/%ld mA current=%ld/%ld/%ld mA count=%lu latched=%u\r\n",
          current.overcurrent_trip_phase_a_ma,
          current.overcurrent_trip_phase_b_ma,
          current.overcurrent_trip_phase_c_ma,
          current.phase_a_ma, current.phase_b_ma, current.phase_c_ma,
          (unsigned long)current.overcurrent_count,
          current.overcurrent_fault ? 1U : 0U);
      }
      motor_control_fault_set((motor_fault_code_t)fault_index);
    }
  }

  /* A pending ISR fault is authoritative.  Do not inspect stale subsystem
     states in the same poll and append a secondary handover fault. */
  if (motor_control_status.state == MOTOR_CONTROL_STATE_FAULT) return;

  if ((motor_control_status.state != MOTOR_CONTROL_STATE_STARTUP) &&
      (!motor_control_mos_temperature_check())) return;
  if ((motor_control_status.state != MOTOR_CONTROL_STATE_STARTUP) &&
      (!motor_control_bus_voltage_check())) return;
  if (motor_control_status.state == MOTOR_CONTROL_STATE_SPEED_PREPARING)
  { motor_control_speed_prepare_poll(); return; }

  (void)motor_open_loop_status_read(&open_loop);
  (void)motor_current_control_status_read(&current_control);
  (void)motor_speed_control_status_read(&speed_control);
  (void)motor_current_startup_status_read(&current_startup);
  if ((motor_control_status.state == MOTOR_CONTROL_STATE_CURRENT_STARTING) &&
      (current_startup.state == MOTOR_CURRENT_STARTUP_COMPLETE))
  {
    motor_control_status.state = MOTOR_CONTROL_STATE_CURRENT_CONTROL;
  }
  else if ((motor_control_status.state == MOTOR_CONTROL_STATE_CURRENT_STARTING) &&
           (current_startup.state == MOTOR_CURRENT_STARTUP_FAULT))
  {
    LOGE("Current startup fault: reason=%u elapsed_ms=%lu edges=%u\r\n",
      (unsigned int)current_startup.fault,
      (unsigned long)current_startup.elapsed_ms,
      (unsigned int)current_startup.qualified_edges);
    motor_control_fault_set(MOTOR_FAULT_CURRENT_HANDOVER);
    return;
  }
  if (open_loop.state == MOTOR_OPEN_LOOP_FAULT)
    motor_control_fault_set(MOTOR_FAULT_OPEN_LOOP);
  if (current_control.state == MOTOR_CURRENT_CONTROL_FAULT)
  {
    if (current_control.fault == MOTOR_CURRENT_CONTROL_FAULT_SAMPLE)
      motor_control_fault_set(MOTOR_FAULT_CURRENT_SAMPLE);
    else if (current_control.fault == MOTOR_CURRENT_CONTROL_FAULT_HALL)
    {
      hall_read_ok = motor_hall_decoder_sample_read(&hall);
      estimator_read_ok = motor_hall_angle_estimator_read(&estimator);
      LOGE("Hall fault diag: read=%u/%u valid=%u/%u state=%u/%u dir=%d/%d freq_mhz=%lu/%lu edges=%lu invalid=%lu duplicate=%lu iq=%ld/%ld\r\n",
        hall_read_ok ? 1U : 0U, estimator_read_ok ? 1U : 0U,
        (hall_read_ok && hall.valid) ? 1U : 0U,
        (estimator_read_ok && estimator.valid) ? 1U : 0U,
        hall_read_ok ? (unsigned int)hall.state : 0U,
        estimator_read_ok ? (unsigned int)estimator.hall_state : 0U,
        hall_read_ok ? (int)hall.direction : 0,
        estimator_read_ok ? (int)estimator.direction : 0,
        hall_read_ok ? (unsigned long)hall.electrical_frequency_millihz : 0UL,
        estimator_read_ok ?
          (unsigned long)estimator.electrical_frequency_millihz : 0UL,
        hall_read_ok ? (unsigned long)hall.edge_count : 0UL,
        hall_read_ok ? (unsigned long)hall.invalid_transition_count : 0UL,
        hall_read_ok ? (unsigned long)hall.duplicate_count : 0UL,
        (long)current_control.quadrature_feedback_ma,
        (long)current_control.command.quadrature_reference_ma);
      motor_control_fault_set(MOTOR_FAULT_HALL);
    }
    else if (current_control.fault == MOTOR_CURRENT_CONTROL_FAULT_OVERCURRENT)
      motor_control_fault_set(MOTOR_FAULT_OVERCURRENT);
    else if (current_control.fault == MOTOR_CURRENT_CONTROL_FAULT_PWM)
      motor_control_fault_set(MOTOR_FAULT_PWM);
    else
      motor_control_fault_set(MOTOR_FAULT_CONTROL_SUBSYSTEM);
    return;
  }
  if (speed_control.state == MOTOR_SPEED_CONTROL_FAULT)
  {
    if (speed_control.fault == MOTOR_SPEED_CONTROL_FAULT_CURRENT_CONTROL)
      motor_control_fault_set(MOTOR_FAULT_SPEED_CURRENT_CONTROL);
    else if (speed_control.fault == MOTOR_SPEED_CONTROL_FAULT_COMMAND)
      motor_control_fault_set(MOTOR_FAULT_SPEED_COMMAND);
    else if (speed_control.fault == MOTOR_SPEED_CONTROL_FAULT_OVERSPEED)
      motor_control_fault_set(MOTOR_FAULT_OVERSPEED);
    else if (speed_control.fault == MOTOR_SPEED_CONTROL_FAULT_STALL)
      motor_control_fault_set(MOTOR_FAULT_STALL);
    else
      motor_control_fault_set(MOTOR_FAULT_CONTROL_SUBSYSTEM);
  }
}

bool motor_control_fault_history_read(uint32_t newest_index,
                                      motor_fault_record_t *record)
{
  uint32_t history_index;
  uint32_t primask;

  if ((record == NULL) || (newest_index >= MOTOR_FAULT_HISTORY_DEPTH))
    return false;
  primask = __get_PRIMASK();
  __disable_irq();
  if (newest_index >= motor_fault_history_count)
  {
    __set_PRIMASK(primask);
    return false;
  }
  history_index = (motor_fault_history_newest + MOTOR_FAULT_HISTORY_DEPTH -
                   newest_index) % MOTOR_FAULT_HISTORY_DEPTH;
  *record = motor_fault_history[history_index];
  __set_PRIMASK(primask);
  return true;
}
bool motor_control_status_read(motor_control_status_t *status)
{
  uint32_t primask;
  if (status == NULL) return false;
  primask = __get_PRIMASK();
  __disable_irq();
  *status = motor_control_status;
  if (primask == 0U) __enable_irq();
  return true;
}
