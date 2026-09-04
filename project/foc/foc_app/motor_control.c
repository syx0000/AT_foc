#include <stddef.h>
#include "at32f45x.h"
#include "motor_control.h"
#include "motor_control_config.h"
#include "motor_current_control.h"
#include "motor_current_sample.h"
#include "motor_direction.h"
#include "motor_foc_math.h"
#include "motor_hall_angle_estimator.h"
#include "motor_log.h"
#include "motor_parameter.h"
#include "motor_pwm_port.h"
#include "motor_slow_sensor.h"
#include "motor_speed_control.h"
static volatile motor_control_status_t motor_control_status;
static motor_fault_record_t motor_fault_history[MOTOR_FAULT_HISTORY_DEPTH];
static uint32_t motor_fault_history_count;
static uint32_t motor_fault_history_newest;
static uint32_t motor_fault_sequence;
static volatile uint32_t motor_fault_pending_mask;

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
      (absolute_speed_rpm > MOTOR_SPEED_CONTROL_MAXIMUM_SPEED_RPM) ||
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
  uint32_t index;

  motor_control_status.state = MOTOR_CONTROL_STATE_STARTUP;
  motor_control_status.fault_code = MOTOR_FAULT_NONE;
  motor_fault_history_count = 0U;
  motor_fault_history_newest = 0U;
  motor_fault_sequence = 0U;
  motor_fault_pending_mask = 0U;
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
  if ((motor_control_status.state != MOTOR_CONTROL_STATE_READY) ||
      (!motor_open_loop_start(command))) return false;
  motor_control_status.state = MOTOR_CONTROL_STATE_OPEN_LOOP;
  return true;
}
bool motor_control_open_loop_command_set(const motor_open_loop_command_t *command)
{
  return (motor_control_status.state == MOTOR_CONTROL_STATE_OPEN_LOOP) &&
         motor_open_loop_command_set(command);
}
bool motor_control_current_control_start(
  const motor_current_control_command_t *command)
{
  motor_open_loop_status_t open_loop;
  motor_hall_angle_estimator_t hall;
  motor_slow_sensor_state_t sensor;
  motor_alpha_beta_q15_t alpha_beta;
  motor_direct_quadrature_q15_t voltage_dq;
  uint32_t bus_mv;
  int32_t direct_seed_mv;
  int32_t quadrature_seed_mv;

  (void)motor_slow_sensor_process();
  if ((motor_control_status.state != MOTOR_CONTROL_STATE_OPEN_LOOP) ||
      (!motor_open_loop_status_read(&open_loop)) ||
      (open_loop.state != MOTOR_OPEN_LOOP_RUNNING) ||
      (!motor_hall_angle_estimator_read(&hall)) || (!hall.valid) ||
      (!motor_slow_sensor_state_read(&sensor))) return false;
  bus_mv = (uint32_t)sensor.bus_voltage_0p1v * 100U;
  alpha_beta = motor_foc_inverse_park_q15(
    (int16_t)(((int64_t)open_loop.applied_direct_voltage_mv * 32768) / bus_mv),
    (int16_t)(((int64_t)open_loop.applied_quadrature_voltage_mv * 32768) / bus_mv),
    open_loop.electrical_angle_u16);
  voltage_dq = motor_foc_park_q15(alpha_beta, hall.electrical_angle_u16);
  direct_seed_mv = (int32_t)(((int64_t)voltage_dq.direct_q15 * bus_mv) / 32768);
  quadrature_seed_mv = (int32_t)(
    ((int64_t)voltage_dq.quadrature_q15 * bus_mv) / 32768);
  if (!motor_open_loop_control_release()) return false;
  if (!motor_current_control_handover(command, direct_seed_mv,
                                       quadrature_seed_mv))
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
  return (motor_control_status.state == MOTOR_CONTROL_STATE_CURRENT_CONTROL) &&
         motor_current_control_command_set(command);
}
bool motor_control_speed_control_start(int32_t target_speed_rpm)
{
  motor_current_control_command_t current_command;

  if (!motor_control_speed_command_valid(target_speed_rpm))
  {
    return false;
  }
  current_command.direct_reference_ma = 0;
  current_command.quadrature_reference_ma = 0;
  if (!motor_control_current_control_start(&current_command))
  {
    return false;
  }
  if (!motor_speed_control_start(target_speed_rpm * 1000L, 0))
  {
    motor_control_fault_set(MOTOR_FAULT_SPEED_START);
    return false;
  }
  motor_control_status.state = MOTOR_CONTROL_STATE_SPEED_CONTROL;
  return true;
}
bool motor_control_speed_control_target_set(int32_t target_speed_rpm)
{
  if ((motor_control_status.state != MOTOR_CONTROL_STATE_SPEED_CONTROL) ||
      (!motor_control_speed_command_valid(target_speed_rpm)))
  {
    return false;
  }
  return motor_speed_control_target_set(target_speed_rpm * 1000L);
}
void motor_control_stop(void)
{
  motor_control_state_t previous_state = motor_control_status.state;
  motor_speed_control_stop();
  motor_current_control_stop();
  motor_open_loop_stop();
  if ((previous_state == MOTOR_CONTROL_STATE_OPEN_LOOP) ||
      (previous_state == MOTOR_CONTROL_STATE_CURRENT_CONTROL) ||
      (previous_state == MOTOR_CONTROL_STATE_SPEED_CONTROL))
    motor_control_status.state = MOTOR_CONTROL_STATE_READY;
}
void motor_control_fault_set(motor_fault_code_t fault_code)
{
  motor_fault_record_t *record;
  uint32_t bit;
  uint32_t primask;
  bool should_log = false;

  if ((fault_code <= MOTOR_FAULT_NONE) ||
      (fault_code > MOTOR_FAULT_HARDWARE_BREAK))
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
      (fault_code > MOTOR_FAULT_HARDWARE_BREAK))
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
    default: return "invalid";
  }
}

motor_fault_clear_result_t motor_control_fault_clear_ex(void)
{
  if (motor_control_status.state != MOTOR_CONTROL_STATE_FAULT)
    return MOTOR_FAULT_CLEAR_NOT_FAULTED;
  if (motor_pwm_port_output_is_enabled())
    return MOTOR_FAULT_CLEAR_PWM_ENABLED;
  if ((motor_control_status.fault_code == MOTOR_FAULT_READY_TRANSITION) ||
      (motor_control_status.fault_code == MOTOR_FAULT_CURRENT_OFFSET_INVALID) ||
      (motor_control_status.fault_code == MOTOR_FAULT_CURRENT_CALIBRATION_TIMEOUT) ||
      (motor_control_status.fault_code == MOTOR_FAULT_DRIVER_NOT_READY))
    return MOTOR_FAULT_CLEAR_RESET_REQUIRED;
  motor_speed_control_stop();
  motor_current_control_stop();
  motor_open_loop_stop();
  if (!motor_current_sample_fault_clear())
    return MOTOR_FAULT_CLEAR_CURRENT_UNSAFE;
  if (!motor_pwm_port_fault_clear()) return MOTOR_FAULT_CLEAR_DRIVER_ACTIVE;
  motor_control_status.fault_code = MOTOR_FAULT_NONE;
  motor_control_status.state = MOTOR_CONTROL_STATE_READY;
  return MOTOR_FAULT_CLEAR_OK;
}

bool motor_control_fault_clear(void)
{
  return (motor_control_fault_clear_ex() == MOTOR_FAULT_CLEAR_OK);
}
void motor_control_poll(void)
{
  motor_open_loop_status_t open_loop;
  motor_current_control_status_t current_control;
  motor_speed_control_status_t speed_control;
  uint32_t pending_mask;
  uint32_t fault_index;
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  pending_mask = motor_fault_pending_mask;
  motor_fault_pending_mask = 0U;
  __set_PRIMASK(primask);
  for (fault_index = 1U;
       fault_index <= (uint32_t)MOTOR_FAULT_HARDWARE_BREAK;
       fault_index++)
  {
    if ((pending_mask & (1UL << fault_index)) != 0U)
    {
      motor_control_fault_set((motor_fault_code_t)fault_index);
    }
  }

  (void)motor_open_loop_status_read(&open_loop);
  (void)motor_current_control_status_read(&current_control);
  (void)motor_speed_control_status_read(&speed_control);
  if (open_loop.state == MOTOR_OPEN_LOOP_FAULT)
    motor_control_fault_set(MOTOR_FAULT_OPEN_LOOP);
  if (current_control.state == MOTOR_CURRENT_CONTROL_FAULT)
  {
    if (current_control.fault == MOTOR_CURRENT_CONTROL_FAULT_SAMPLE)
      motor_control_fault_set(MOTOR_FAULT_CURRENT_SAMPLE);
    else if (current_control.fault == MOTOR_CURRENT_CONTROL_FAULT_HALL)
      motor_control_fault_set(MOTOR_FAULT_HALL);
    else if (current_control.fault == MOTOR_CURRENT_CONTROL_FAULT_OVERCURRENT)
      motor_control_fault_set(MOTOR_FAULT_OVERCURRENT);
    else if (current_control.fault == MOTOR_CURRENT_CONTROL_FAULT_PWM)
      motor_control_fault_set(MOTOR_FAULT_PWM);
    else
      motor_control_fault_set(MOTOR_FAULT_CONTROL_SUBSYSTEM);
  }
  if (speed_control.state == MOTOR_SPEED_CONTROL_FAULT)
  {
    if (speed_control.fault == MOTOR_SPEED_CONTROL_FAULT_FEEDBACK)
      motor_control_fault_set(MOTOR_FAULT_SPEED_FEEDBACK);
    else if (speed_control.fault == MOTOR_SPEED_CONTROL_FAULT_CURRENT_CONTROL)
      motor_control_fault_set(MOTOR_FAULT_SPEED_CURRENT_CONTROL);
    else if (speed_control.fault == MOTOR_SPEED_CONTROL_FAULT_COMMAND)
      motor_control_fault_set(MOTOR_FAULT_SPEED_COMMAND);
    else if (speed_control.fault == MOTOR_SPEED_CONTROL_FAULT_REVERSE_DIRECTION)
      motor_control_fault_set(MOTOR_FAULT_REVERSE_DIRECTION);
    else if (speed_control.fault == MOTOR_SPEED_CONTROL_FAULT_STALL)
      motor_control_fault_set(MOTOR_FAULT_STALL);
    else if (speed_control.fault == MOTOR_SPEED_CONTROL_FAULT_OVERSPEED)
      motor_control_fault_set(MOTOR_FAULT_OVERSPEED);
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
