#ifndef MOTOR_CURRENT_STARTUP_H
#define MOTOR_CURRENT_STARTUP_H

#include <stdbool.h>
#include <stdint.h>
#include "motor_current_control.h"

typedef enum
{
  MOTOR_CURRENT_STARTUP_STOPPED = 0,
  MOTOR_CURRENT_STARTUP_ALIGNING,
  MOTOR_CURRENT_STARTUP_IF_RAMP,
  MOTOR_CURRENT_STARTUP_HANDOVER_WAIT,
  MOTOR_CURRENT_STARTUP_ANGLE_BLEND,
  MOTOR_CURRENT_STARTUP_COMPLETE,
  MOTOR_CURRENT_STARTUP_FAULT,
  MOTOR_CURRENT_STARTUP_CURRENT_RAMP,
  MOTOR_CURRENT_STARTUP_HALL_DIRECT
} motor_current_startup_state_t;

typedef enum
{
  MOTOR_CURRENT_STARTUP_FAULT_NONE = 0,
  MOTOR_CURRENT_STARTUP_FAULT_START,
  MOTOR_CURRENT_STARTUP_FAULT_CURRENT_CONTROL,
  MOTOR_CURRENT_STARTUP_FAULT_HALL_TIMEOUT,
  MOTOR_CURRENT_STARTUP_FAULT_BLEND_ESTIMATOR_INVALID,
  MOTOR_CURRENT_STARTUP_FAULT_BLEND_HALL_INVALID,
  MOTOR_CURRENT_STARTUP_FAULT_BLEND_DIRECTION,
  MOTOR_CURRENT_STARTUP_FAULT_BLEND_TRANSITION
} motor_current_startup_fault_t;

typedef struct
{
  motor_current_startup_state_t state;
  motor_current_startup_fault_t fault;
  uint32_t elapsed_ms;
  uint8_t qualified_edges;
  int8_t requested_direction;
  bool hall_direct;
  bool speed_control_ready;
} motor_current_startup_status_t;

void motor_current_startup_init(void);
bool motor_current_startup_start(
  const motor_current_control_command_t *physical_command);
bool motor_current_startup_speed_start(
  const motor_current_control_command_t *physical_command);
bool motor_current_startup_current_command_release(void);
bool motor_current_startup_reverse_start(
  const motor_current_control_command_t *physical_command);
bool motor_current_startup_command_set(
  const motor_current_control_command_t *physical_command);
void motor_current_startup_stop(void);
void motor_current_startup_process_1khz(void);
void motor_current_startup_fast_process(void);
bool motor_current_startup_status_read(motor_current_startup_status_t *status);

#endif
