#include <stddef.h>
#include "at32f45x.h"
#include "motor_board_config.h"
#include "motor_control_config.h"
#include "motor_current_startup.h"
#include "motor_hall_angle_math.h"
#include "motor_hall_angle_estimator.h"
#include "motor_hall_decoder.h"
#include "motor_parameter.h"
#include "motor_rotor_feedback.h"
#include "motor_timebase.h"

/* Allow brief estimator invalid/direction glitches after angle handover.
 * The fast loop runs at PWM frequency, so 100 cycles is about 5 ms at
 * 20 kHz.  A single inconsistent Hall snapshot must not stop the motor. */
#define MOTOR_CURRENT_STARTUP_FEEDBACK_DEBOUNCE_CYCLES 160U /* 10 ms @16 kHz */

static volatile motor_current_startup_status_t motor_current_startup_status;
static motor_current_control_command_t motor_current_startup_target;
static uint32_t motor_current_startup_phase_accumulator;
static int32_t motor_current_startup_frequency_millihz;
static uint32_t motor_current_startup_last_hall_update;
static uint32_t motor_current_startup_last_invalid_count;
static uint32_t motor_current_startup_blend_cycles;
static int16_t motor_current_startup_blend_initial_error;
static int8_t motor_current_startup_direction;
static uint32_t motor_current_startup_feedback_invalid_cycles;
static uint32_t motor_current_startup_direction_invalid_cycles;
static bool motor_current_startup_limited_if;
static bool motor_current_startup_hall_direct;
static uint16_t motor_current_startup_hall_direct_angle_u16;
static bool motor_current_startup_speed_start_requested;
static bool motor_current_startup_current_command_released;
static volatile uint32_t motor_current_startup_blend_complete_ms;
static uint32_t motor_current_startup_target_speed_millirpm;

/* At standstill the Hall decoder has a valid sector but no speed/direction,
 * so the interpolating estimator is deliberately invalid.  Use the calibrated
 * midpoint of the present Hall sector as the initial rotor angle instead of
 * pulling every start to one fixed electrical angle. */
static bool motor_current_startup_hall_center_angle_get(
  uint8_t hall_state, uint16_t *angle_u16)
{
  motor_parameter_t parameter;
  uint16_t sector_span_u16;

  if ((angle_u16 == NULL) ||
      (!motor_parameter_active_read(&parameter)) ||
      (!motor_hall_sector_span_get(parameter.hall_positive_next,
                                   parameter.hall_entry_angle_u16,
                                   hall_state, &sector_span_u16)))
    return false;
  *angle_u16 = (uint16_t)(parameter.hall_entry_angle_u16[hall_state] +
                          sector_span_u16 / 2U +
                          parameter.hall_rotor_offset_u16);
  return true;
}

static int32_t motor_current_startup_ramp(int32_t current, int32_t target,
                                         int32_t step)
{
  if (current < target)
  {
    current += step;
    if (current > target) current = target;
  }
  else if (current > target)
  {
    current -= step;
    if (current < target) current = target;
  }
  return current;
}

static int8_t motor_current_startup_forced_direction_get(void)
{
  if (motor_current_startup_frequency_millihz > 0) return 1;
  if (motor_current_startup_frequency_millihz < 0) return -1;
  return motor_current_startup_direction;
}

static void motor_current_startup_fault_set(motor_current_startup_fault_t fault)
{
  motor_current_startup_status.fault = fault;
  motor_current_startup_status.state = MOTOR_CURRENT_STARTUP_FAULT;
  motor_rotor_feedback_override_clear();
  motor_current_control_stop();
}

static void motor_current_startup_handover_retry(void)
{
  motor_hall_sample_t hall;
  uint16_t if_angle =
    (uint16_t)(motor_current_startup_phase_accumulator >> 16);

  motor_current_startup_status.state = MOTOR_CURRENT_STARTUP_HANDOVER_WAIT;
  motor_current_startup_status.qualified_edges = 0U;
  motor_current_startup_status.requested_direction = 0;
  motor_current_startup_status.hall_direct = false;
  motor_current_startup_status.speed_control_ready = false;
  motor_current_startup_blend_cycles = 0U;
  motor_current_startup_feedback_invalid_cycles = 0U;
  motor_current_startup_direction_invalid_cycles = 0U;
  if (motor_hall_decoder_sample_read(&hall))
  {
    motor_current_startup_last_hall_update = hall.frequency_update_count;
    motor_current_startup_last_invalid_count = hall.invalid_transition_count;
  }
  motor_rotor_feedback_override_set(if_angle,
    motor_current_startup_frequency_millihz,
    motor_current_startup_forced_direction_get());
}

void motor_current_startup_init(void)
{
  motor_current_startup_status.state = MOTOR_CURRENT_STARTUP_STOPPED;
  motor_current_startup_status.fault = MOTOR_CURRENT_STARTUP_FAULT_NONE;
  motor_current_startup_status.elapsed_ms = 0U;
  motor_current_startup_status.qualified_edges = 0U;
  motor_current_startup_status.hall_direct = false;
  motor_current_startup_status.speed_control_ready = false;
  motor_current_startup_phase_accumulator =
    (uint32_t)MOTOR_CURRENT_STARTUP_ALIGNMENT_ANGLE_U16 << 16;
  motor_current_startup_frequency_millihz = 0;
  motor_current_startup_last_hall_update = 0U;
  motor_current_startup_last_invalid_count = 0U;
  motor_current_startup_blend_cycles = 0U;
  motor_current_startup_blend_initial_error = 0;
  motor_current_startup_feedback_invalid_cycles = 0U;
  motor_current_startup_direction_invalid_cycles = 0U;
  motor_current_startup_limited_if = false;
  motor_current_startup_hall_direct = false;
  motor_current_startup_hall_direct_angle_u16 = 0U;
  motor_current_startup_speed_start_requested = false;
  motor_current_startup_current_command_released = false;
  motor_current_startup_blend_complete_ms = 0U;
  motor_current_startup_target_speed_millirpm = 0U;
}

static bool motor_current_startup_start_internal(
  const motor_current_control_command_t *physical_command,
  bool speed_start_requested)
{
  motor_current_control_command_t alignment_command;
  motor_hall_sample_t hall;
  uint32_t primask;
  bool hall_center_valid;

  if ((physical_command == NULL) ||
      (physical_command->quadrature_reference_ma == 0) ||
      (motor_current_startup_status.state != MOTOR_CURRENT_STARTUP_STOPPED))
    return false;
  motor_current_startup_target = *physical_command;
  motor_current_startup_limited_if = false;
  motor_current_startup_direction =
    (physical_command->quadrature_reference_ma < 0) ? -1 : 1;
  motor_current_startup_status.requested_direction =
    motor_current_startup_direction;
  motor_current_startup_phase_accumulator =
    (uint32_t)MOTOR_CURRENT_STARTUP_ALIGNMENT_ANGLE_U16 << 16;
  motor_current_startup_frequency_millihz = 0;
  motor_current_startup_status.elapsed_ms = 0U;
  motor_current_startup_status.qualified_edges = 0U;
  motor_current_startup_status.fault = MOTOR_CURRENT_STARTUP_FAULT_NONE;
  motor_current_startup_feedback_invalid_cycles = 0U;
  motor_current_startup_direction_invalid_cycles = 0U;
  motor_current_startup_speed_start_requested = speed_start_requested;
  motor_current_startup_current_command_released = false;
  motor_current_startup_blend_complete_ms = 0U;
  /* A valid stationary Hall sector is sufficient to establish rotor angle.
   * Keep I/F only as a fallback for an invalid Hall input; speed starts and
   * opposite-direction starts both use signed Iq from the sector centre. */
  motor_current_startup_limited_if = false;

  /* A stopped decoder still contains the previous run direction and its
   * six-edge speed window.  Starting in the opposite direction must not expose
   * those stale values to Hall-direct startup or the speed loop. */
  primask = __get_PRIMASK();
  __disable_irq();
  motor_hall_decoder_motion_reset(motor_timebase_cycles_get());
  motor_hall_angle_estimator_motion_reset();
  __set_PRIMASK(primask);

  hall_center_valid = motor_hall_decoder_sample_read(&hall) && hall.valid &&
    motor_current_startup_hall_center_angle_get(
      hall.state, &motor_current_startup_hall_direct_angle_u16);
  /* Use the calibrated Hall-sector centre for every stationary start.  The
   * sign of Iq selects torque direction without rotating an open-loop field. */
  motor_current_startup_hall_direct =
    (MOTOR_CURRENT_STARTUP_HALL_DIRECT_ENABLE != 0U) &&
    (!motor_current_startup_limited_if) && hall_center_valid;
  if (motor_current_startup_hall_direct)
  {
    motor_current_startup_last_hall_update = hall.frequency_update_count;
    motor_current_startup_last_invalid_count = hall.invalid_transition_count;
  }
  else
  {
    motor_current_startup_last_hall_update = 0U;
    motor_current_startup_last_invalid_count = 0U;
  }
  if (motor_current_startup_limited_if && hall_center_valid)
  {
    motor_current_startup_phase_accumulator =
      (uint32_t)motor_current_startup_hall_direct_angle_u16 << 16;
    motor_current_startup_frequency_millihz =
      motor_current_startup_direction *
      (int32_t)MOTOR_CURRENT_STARTUP_INITIAL_FREQUENCY_MILLIHZ;
  }
  motor_rotor_feedback_override_set(
    hall_center_valid ? motor_current_startup_hall_direct_angle_u16 :
                        MOTOR_CURRENT_STARTUP_ALIGNMENT_ANGLE_U16,
    0, motor_current_startup_direction);
  /* Start from zero current.  The 1 kHz startup state machine ramps the
     alignment current up after ADC sampling has settled, avoiding a full
     d-axis current step when the bridge first becomes active. */
  alignment_command.direct_reference_ma = 0;
  alignment_command.quadrature_reference_ma = 0;
  /*
   * Enable the bridge from the neutral 50% duty vector first.  Applying the
   * alignment voltage before the first live ADC samples can place both phase
   * current channels outside their valid sampling window (observed as 0/0
   * ADC codes and approximately +66 A/+66 A reconstructed current).  After
   * the settle interval the current PI ramps from zero to the alignment
   * current using valid feedback.
   */
  if (!motor_current_control_start_seeded(
        &alignment_command,
        MOTOR_CURRENT_STARTUP_ALIGNMENT_VOLTAGE_MV,
        0,
        MOTOR_CURRENT_STARTUP_SAMPLE_SETTLE_CYCLES))
  {
    motor_rotor_feedback_override_clear();
    motor_current_startup_status.state = MOTOR_CURRENT_STARTUP_FAULT;
    motor_current_startup_status.fault = MOTOR_CURRENT_STARTUP_FAULT_START;
    return false;
  }
  motor_current_startup_status.state = motor_current_startup_hall_direct ?
    MOTOR_CURRENT_STARTUP_HALL_DIRECT :
    ((motor_current_startup_limited_if && hall_center_valid) ?
      MOTOR_CURRENT_STARTUP_IF_RAMP : MOTOR_CURRENT_STARTUP_ALIGNING);
  motor_current_startup_status.hall_direct = motor_current_startup_hall_direct;
  motor_current_startup_status.speed_control_ready = false;
  return true;
}

bool motor_current_startup_start(
  const motor_current_control_command_t *physical_command)
{
  return motor_current_startup_start_internal(physical_command, false);
}

bool motor_current_startup_speed_start(
  const motor_current_control_command_t *physical_command,
  uint32_t target_speed_rpm)
{
  if ((target_speed_rpm == 0U) ||
      (target_speed_rpm > (UINT32_MAX / 1000U)))
    return false;
  motor_current_startup_target_speed_millirpm = target_speed_rpm * 1000U;
  return motor_current_startup_start_internal(physical_command, true);
}

bool motor_current_startup_current_command_release(void)
{
  if ((!motor_current_startup_hall_direct) ||
      (!motor_current_startup_speed_start_requested) ||
      (!motor_current_startup_status.speed_control_ready))
    return false;
  motor_current_startup_current_command_released = true;
  return true;
}

bool motor_current_startup_reverse_start(
  const motor_current_control_command_t *physical_command)
{
  motor_current_control_status_t current;
  motor_hall_angle_estimator_t estimator;
  motor_hall_sample_t hall;
  int8_t requested_direction;

  if ((physical_command == NULL) ||
      (physical_command->quadrature_reference_ma == 0) ||
      ((motor_current_startup_status.state != MOTOR_CURRENT_STARTUP_COMPLETE) &&
       (motor_current_startup_status.state != MOTOR_CURRENT_STARTUP_STOPPED)) ||
      (!motor_current_control_status_read(&current)) ||
      (current.state != MOTOR_CURRENT_CONTROL_RUNNING) ||
      (!motor_hall_angle_estimator_read(&estimator)) || (!estimator.valid) ||
      (estimator.direction == 0)) return false;

  requested_direction =
    (physical_command->quadrature_reference_ma < 0) ? -1 : 1;
  if (requested_direction == estimator.direction) return false;
  motor_current_startup_target = *physical_command;
  motor_current_startup_direction = requested_direction;
  motor_current_startup_phase_accumulator =
    (uint32_t)estimator.electrical_angle_u16 << 16;
  motor_current_startup_frequency_millihz = estimator.direction *
    (int32_t)estimator.electrical_frequency_millihz;
  motor_current_startup_status.elapsed_ms = 0U;
  motor_current_startup_status.qualified_edges = 0U;
  motor_current_startup_status.fault = MOTOR_CURRENT_STARTUP_FAULT_NONE;
  motor_current_startup_status.requested_direction = requested_direction;
  motor_current_startup_status.state = MOTOR_CURRENT_STARTUP_HANDOVER_WAIT;
  motor_current_startup_blend_cycles = 0U;
  motor_current_startup_feedback_invalid_cycles = 0U;
  motor_current_startup_direction_invalid_cycles = 0U;
  motor_current_startup_limited_if = true;
  motor_current_startup_hall_direct = false;
  motor_current_startup_speed_start_requested = false;
  motor_current_startup_current_command_released = false;
  motor_current_startup_status.hall_direct = false;
  motor_current_startup_status.speed_control_ready = false;
  if (motor_hall_decoder_sample_read(&hall))
  {
    motor_current_startup_last_hall_update = hall.frequency_update_count;
    motor_current_startup_last_invalid_count = hall.invalid_transition_count;
  }
  motor_rotor_feedback_override_set(estimator.electrical_angle_u16,
    motor_current_startup_frequency_millihz, estimator.direction);
  return true;
}

bool motor_current_startup_command_set(
  const motor_current_control_command_t *physical_command)
{
  if ((physical_command == NULL) ||
      (motor_current_startup_status.state == MOTOR_CURRENT_STARTUP_STOPPED) ||
      (motor_current_startup_status.state == MOTOR_CURRENT_STARTUP_FAULT) ||
      (motor_current_startup_status.state == MOTOR_CURRENT_STARTUP_COMPLETE))
    return false;
  if (((physical_command->quadrature_reference_ma < 0) ? -1 : 1) !=
      motor_current_startup_direction) return false;
  motor_current_startup_target = *physical_command;
  return true;
}

void motor_current_startup_stop(void)
{
  motor_rotor_feedback_override_clear();
  if ((motor_current_startup_status.state != MOTOR_CURRENT_STARTUP_STOPPED) &&
      (motor_current_startup_status.state != MOTOR_CURRENT_STARTUP_COMPLETE))
    motor_current_control_stop();
  motor_current_startup_status.state = MOTOR_CURRENT_STARTUP_STOPPED;
  motor_current_startup_status.fault = MOTOR_CURRENT_STARTUP_FAULT_NONE;
  motor_current_startup_status.requested_direction = 0;
  motor_current_startup_feedback_invalid_cycles = 0U;
  motor_current_startup_direction_invalid_cycles = 0U;
  motor_current_startup_limited_if = false;
  motor_current_startup_hall_direct = false;
  motor_current_startup_speed_start_requested = false;
  motor_current_startup_current_command_released = false;
  motor_current_startup_status.hall_direct = false;
  motor_current_startup_status.speed_control_ready = false;
  motor_current_startup_target_speed_millirpm = 0U;
}

void motor_current_startup_process_1khz(void)
{
  motor_current_control_status_t current;
  motor_current_control_command_t command;
  motor_hall_sample_t hall;
  motor_hall_angle_estimator_t estimator;
  int32_t target_frequency;
  int32_t startup_quadrature_target;
  int32_t startup_current_magnitude;
  uint32_t commanded_frequency_millihz;
  uint32_t measured_frequency_millihz;
  uint32_t frequency_lag_millihz;
  int64_t startup_current_boost_ma;
  int32_t hall_direct_quadrature_target;
  int32_t hall_direct_current_magnitude;
  int32_t hall_direct_current_step_ma;
  uint32_t hall_direct_boost_elapsed_ms;
  uint32_t measured_speed_millirpm;
  uint32_t speed_matched_current_ma;
  uint8_t pole_pairs;

  if ((motor_current_startup_status.state == MOTOR_CURRENT_STARTUP_STOPPED) ||
      (motor_current_startup_status.state == MOTOR_CURRENT_STARTUP_COMPLETE) ||
      (motor_current_startup_status.state == MOTOR_CURRENT_STARTUP_FAULT)) return;
  motor_current_startup_status.elapsed_ms++;
  if ((!motor_current_control_status_read(&current)) ||
      (current.state != MOTOR_CURRENT_CONTROL_RUNNING))
  {
    motor_current_startup_fault_set(
      MOTOR_CURRENT_STARTUP_FAULT_CURRENT_CONTROL);
    return;
  }
  command = current.command;
  if (motor_current_startup_hall_direct &&
      ((motor_current_startup_status.state ==
        MOTOR_CURRENT_STARTUP_HALL_DIRECT) ||
       (motor_current_startup_status.state ==
        MOTOR_CURRENT_STARTUP_ANGLE_BLEND)))
  {
    hall_direct_quadrature_target =
      motor_current_startup_target.quadrature_reference_ma;
    hall_direct_current_step_ma =
      MOTOR_CURRENT_STARTUP_HALL_DIRECT_CURRENT_RAMP_MA_PER_MS;
    if (motor_current_startup_speed_start_requested &&
        (!motor_current_startup_status.speed_control_ready))
    {
      hall_direct_current_magnitude =
        (hall_direct_quadrature_target < 0) ?
          -hall_direct_quadrature_target : hall_direct_quadrature_target;
      if (motor_current_startup_status.elapsed_ms >
          MOTOR_SPEED_HALL_START_BOOST_DELAY_MS)
      {
        hall_direct_boost_elapsed_ms =
          motor_current_startup_status.elapsed_ms -
          MOTOR_SPEED_HALL_START_BOOST_DELAY_MS;
        startup_current_boost_ma =
          (int64_t)hall_direct_boost_elapsed_ms *
          MOTOR_SPEED_HALL_START_BOOST_MA_PER_MS;
        if (startup_current_boost_ma >
            (MOTOR_SPEED_HALL_START_MAXIMUM_CURRENT_MA -
             hall_direct_current_magnitude))
          startup_current_boost_ma =
            MOTOR_SPEED_HALL_START_MAXIMUM_CURRENT_MA -
            hall_direct_current_magnitude;
        hall_direct_current_magnitude +=
          (int32_t)startup_current_boost_ma;
      }

      /* Above half the requested speed, reduce the startup-current ceiling
       * linearly to zero at the target.  The time ramp still controls torque
       * build-up below that point; taking the minimum provides load capacity
       * without accelerating open-loop through the requested speed. */
      pole_pairs = motor_parameter_pole_pairs_get();
      if ((pole_pairs != 0U) &&
          motor_hall_angle_estimator_read(&estimator) && estimator.valid &&
          (estimator.direction == motor_current_startup_direction))
      {
        speed_matched_current_ma =
          MOTOR_SPEED_HALL_START_MAXIMUM_CURRENT_MA;
        measured_speed_millirpm = (uint32_t)(
          ((uint64_t)estimator.electrical_frequency_millihz * 60U) /
          pole_pairs);
        if (measured_speed_millirpm >=
            motor_current_startup_target_speed_millirpm)
        {
          speed_matched_current_ma = 0U;
        }
        else if ((uint64_t)measured_speed_millirpm * 2U >
                 motor_current_startup_target_speed_millirpm)
        {
          speed_matched_current_ma = (uint32_t)(
            ((uint64_t)MOTOR_SPEED_HALL_START_MAXIMUM_CURRENT_MA * 2U *
             (motor_current_startup_target_speed_millirpm -
              measured_speed_millirpm)) /
            motor_current_startup_target_speed_millirpm);
        }
        if ((uint32_t)hall_direct_current_magnitude >
            speed_matched_current_ma)
        {
          hall_direct_current_magnitude =
            (int32_t)speed_matched_current_ma;
          hall_direct_current_step_ma =
            MOTOR_SPEED_HALL_START_SPEED_MATCH_RELEASE_MA_PER_MS;
        }
      }
      hall_direct_quadrature_target = motor_current_startup_direction *
                                      hall_direct_current_magnitude;
    }
    if (!motor_current_startup_current_command_released)
    {
      command.direct_reference_ma = motor_current_startup_ramp(
        command.direct_reference_ma,
        motor_current_startup_target.direct_reference_ma,
        MOTOR_CURRENT_STARTUP_HALL_DIRECT_CURRENT_RAMP_MA_PER_MS);
      command.quadrature_reference_ma = motor_current_startup_ramp(
        command.quadrature_reference_ma,
        hall_direct_quadrature_target,
        hall_direct_current_step_ma);
      if (!motor_current_control_command_set(&command))
      {
        motor_current_startup_fault_set(
          MOTOR_CURRENT_STARTUP_FAULT_CURRENT_CONTROL);
        return;
      }
    }
    /* The fast loop owns the angle blend once it has started. */
    if (motor_current_startup_status.state ==
        MOTOR_CURRENT_STARTUP_ANGLE_BLEND)
      return;

    if ((!motor_hall_decoder_sample_read(&hall)) || (!hall.valid) ||
        (!motor_current_startup_hall_center_angle_get(
          hall.state, &motor_current_startup_hall_direct_angle_u16)))
    {
      if (motor_current_startup_status.elapsed_ms >=
          MOTOR_CURRENT_STARTUP_TIMEOUT_MS)
        motor_current_startup_fault_set(
          MOTOR_CURRENT_STARTUP_FAULT_HALL_TIMEOUT);
      return;
    }
    /* Below the interpolation threshold, follow the calibrated Hall lookup
     * table one sector at a time.  This is the sensored low-speed behavior
     * used by the smooth reference drive. */
    motor_rotor_feedback_override_set(
      motor_current_startup_hall_direct_angle_u16,
      (hall.direction == 0) ? 0 :
        hall.direction * (int32_t)hall.electrical_frequency_millihz,
      (hall.direction == 0) ? motor_current_startup_direction :
                              hall.direction);
    /* Current control is allowed to produce continuous torque at standstill.
     * Once its requested current has been reached with a valid Hall sector,
     * hand angle ownership back to rotor feedback.  Its static-Hall fallback
     * keeps using the sector center until interpolation becomes available.
     * Speed startup still requires coherent motion before handover. */
    if ((!motor_current_startup_speed_start_requested) &&
        (command.direct_reference_ma ==
         motor_current_startup_target.direct_reference_ma) &&
        (command.quadrature_reference_ma ==
         motor_current_startup_target.quadrature_reference_ma))
    {
      motor_rotor_feedback_override_clear();
      motor_current_startup_limited_if = false;
      motor_current_startup_status.state = MOTOR_CURRENT_STARTUP_COMPLETE;
      return;
    }
    if ((hall.invalid_transition_count !=
         motor_current_startup_last_invalid_count) ||
        ((hall.direction != 0) &&
         (hall.direction != motor_current_startup_direction)))
    {
      motor_current_startup_status.qualified_edges = 0U;
      motor_current_startup_last_invalid_count = hall.invalid_transition_count;
      motor_current_startup_last_hall_update = hall.frequency_update_count;
    }
    else if (hall.frequency_update_count !=
             motor_current_startup_last_hall_update)
    {
      motor_current_startup_last_hall_update = hall.frequency_update_count;
      if (motor_current_startup_status.qualified_edges < 255U)
        motor_current_startup_status.qualified_edges++;
    }
    if ((motor_current_startup_status.qualified_edges >=
         MOTOR_CURRENT_STARTUP_HALL_DIRECT_REQUIRED_EDGES) &&
        motor_hall_angle_estimator_read(&estimator) && estimator.valid &&
        (estimator.direction == motor_current_startup_direction) &&
        (estimator.electrical_frequency_millihz >=
         MOTOR_CURRENT_STARTUP_MINIMUM_HALL_FREQUENCY_MILLIHZ))
    {
      motor_current_startup_phase_accumulator =
        (uint32_t)motor_current_startup_hall_direct_angle_u16 << 16;
      motor_current_startup_frequency_millihz = estimator.direction *
        (int32_t)estimator.electrical_frequency_millihz;
      motor_current_startup_blend_initial_error = (int16_t)(
        estimator.electrical_angle_u16 -
        motor_current_startup_hall_direct_angle_u16);
      motor_current_startup_blend_cycles = 0U;
      motor_current_startup_status.state =
        MOTOR_CURRENT_STARTUP_ANGLE_BLEND;
    }
    else if (motor_current_startup_status.elapsed_ms >=
             MOTOR_CURRENT_STARTUP_TIMEOUT_MS)
    {
      motor_current_startup_fault_set(
        MOTOR_CURRENT_STARTUP_FAULT_HALL_TIMEOUT);
    }
    return;
  }
  if (motor_current_startup_status.state ==
      MOTOR_CURRENT_STARTUP_CURRENT_RAMP)
  {
    /* Do not let the speed PI take over on the first coarse Hall samples.
     * Hold the captured Hall angle/current until interpolation has run for a
     * short settling interval and feedback remains coherent. */
    if (motor_current_startup_speed_start_requested &&
        (!motor_current_startup_status.speed_control_ready))
    {
      if (motor_hall_angle_estimator_read(&estimator) && estimator.valid &&
          (estimator.direction == motor_current_startup_direction) &&
          (estimator.electrical_frequency_millihz >=
           MOTOR_CURRENT_STARTUP_MINIMUM_HALL_FREQUENCY_MILLIHZ) &&
          ((motor_current_startup_status.elapsed_ms -
            motor_current_startup_blend_complete_ms) >=
           MOTOR_SPEED_HALL_START_SETTLE_MS))
        motor_current_startup_status.speed_control_ready = true;
      return;
    }
    if (motor_current_startup_current_command_released)
    {
      motor_rotor_feedback_override_clear();
      motor_current_startup_status.state = MOTOR_CURRENT_STARTUP_COMPLETE;
      return;
    }
    command.direct_reference_ma = motor_current_startup_ramp(
      command.direct_reference_ma,
      motor_current_startup_target.direct_reference_ma,
      MOTOR_CURRENT_STARTUP_HANDOVER_CURRENT_RAMP_MA_PER_MS);
    command.quadrature_reference_ma = motor_current_startup_ramp(
      command.quadrature_reference_ma,
      motor_current_startup_target.quadrature_reference_ma,
      MOTOR_CURRENT_STARTUP_HANDOVER_CURRENT_RAMP_MA_PER_MS);
    if (!motor_current_control_command_set(&command))
    {
      motor_current_startup_fault_set(
        MOTOR_CURRENT_STARTUP_FAULT_CURRENT_CONTROL);
      return;
    }
    if ((command.direct_reference_ma ==
         motor_current_startup_target.direct_reference_ma) &&
        (command.quadrature_reference_ma ==
         motor_current_startup_target.quadrature_reference_ma))
    {
      /* The torque transient is over; the live Hall estimator is already
         being mirrored by the override, so removing it is angle-continuous. */
      motor_rotor_feedback_override_clear();
      motor_current_startup_limited_if = false;
      motor_current_startup_status.state = MOTOR_CURRENT_STARTUP_COMPLETE;
    }
    return;
  }
  if (motor_current_startup_status.state == MOTOR_CURRENT_STARTUP_ALIGNING)
  {
    command.direct_reference_ma = motor_current_startup_ramp(
      command.direct_reference_ma,
      MOTOR_CURRENT_STARTUP_ALIGNMENT_CURRENT_MA,
      MOTOR_CURRENT_STARTUP_CURRENT_RAMP_MA_PER_MS);
    command.quadrature_reference_ma = motor_current_startup_ramp(
      command.quadrature_reference_ma, 0,
      MOTOR_CURRENT_STARTUP_CURRENT_RAMP_MA_PER_MS);
    if (!motor_current_control_command_set(&command))
    {
      motor_current_startup_fault_set(
        MOTOR_CURRENT_STARTUP_FAULT_CURRENT_CONTROL);
      return;
    }
    if (motor_current_startup_status.elapsed_ms <
        MOTOR_CURRENT_STARTUP_ALIGNMENT_TIME_MS) return;
    motor_current_startup_status.state = MOTOR_CURRENT_STARTUP_IF_RAMP;
    motor_current_startup_status.elapsed_ms = 0U;
    motor_current_startup_frequency_millihz =
      motor_current_startup_direction *
      (int32_t)MOTOR_CURRENT_STARTUP_INITIAL_FREQUENCY_MILLIHZ;
  }

  command.direct_reference_ma = motor_current_startup_ramp(
    command.direct_reference_ma,
    motor_current_startup_target.direct_reference_ma,
    MOTOR_CURRENT_STARTUP_CURRENT_RAMP_MA_PER_MS);
  startup_quadrature_target = motor_current_startup_target.quadrature_reference_ma;
  /*
   * Keep the I/F pull-in current throughout the angle blend.  Reducing Iq
   * from the pull-in value to a small user command while the electrical
   * angle is still moving from I/F to Hall can remove the synchronising
   * torque and stall the rotor before handover completes.
   */
  if ((motor_current_startup_direction > 0) &&
      (startup_quadrature_target < MOTOR_CURRENT_STARTUP_IF_MINIMUM_CURRENT_MA))
    startup_quadrature_target = MOTOR_CURRENT_STARTUP_IF_MINIMUM_CURRENT_MA;
  else if ((motor_current_startup_direction < 0) &&
           (startup_quadrature_target > -MOTOR_CURRENT_STARTUP_IF_MINIMUM_CURRENT_MA))
    startup_quadrature_target = -MOTOR_CURRENT_STARTUP_IF_MINIMUM_CURRENT_MA;

  startup_current_magnitude = (startup_quadrature_target < 0) ?
    -startup_quadrature_target : startup_quadrature_target;
  commanded_frequency_millihz =
    (motor_current_startup_frequency_millihz < 0) ?
      (uint32_t)(-motor_current_startup_frequency_millihz) :
      (uint32_t)motor_current_startup_frequency_millihz;
  if (motor_current_startup_limited_if)
  {
    startup_current_boost_ma = 0;
    if (startup_current_magnitude > MOTOR_SPEED_REVERSE_IF_CURRENT_LIMIT_MA)
      startup_current_magnitude = MOTOR_SPEED_REVERSE_IF_CURRENT_LIMIT_MA;
  }
  else if (motor_hall_decoder_sample_read(&hall) && hall.valid &&
      (hall.direction == motor_current_startup_direction) &&
      (hall.electrical_frequency_millihz != 0U) &&
      (commanded_frequency_millihz != 0U))
  {
    measured_frequency_millihz = hall.electrical_frequency_millihz;
    frequency_lag_millihz =
      (measured_frequency_millihz < commanded_frequency_millihz) ?
      (commanded_frequency_millihz - measured_frequency_millihz) : 0U;
    startup_current_boost_ma =
      ((int64_t)(MOTOR_CURRENT_STARTUP_IF_MAXIMUM_CURRENT_MA -
                 startup_current_magnitude) * frequency_lag_millihz) /
      commanded_frequency_millihz;
  }
  else
  {
    /* A loaded stationary rotor has no speed feedback yet.  Increase torque
       with time instead of applying the 10 A ceiling as an immediate step. */
    startup_current_boost_ma =
      (int64_t)motor_current_startup_status.elapsed_ms *
      MOTOR_CURRENT_STARTUP_IF_BOOST_MA_PER_MS;
  }
  if (startup_current_boost_ma >
      (MOTOR_CURRENT_STARTUP_IF_MAXIMUM_CURRENT_MA -
       startup_current_magnitude))
    startup_current_boost_ma = MOTOR_CURRENT_STARTUP_IF_MAXIMUM_CURRENT_MA -
                               startup_current_magnitude;
  startup_current_magnitude += (int32_t)startup_current_boost_ma;
  startup_quadrature_target = motor_current_startup_direction *
                              startup_current_magnitude;
  command.quadrature_reference_ma = motor_current_startup_ramp(
    command.quadrature_reference_ma,
    startup_quadrature_target,
    motor_current_startup_limited_if ?
      MOTOR_SPEED_REVERSE_CURRENT_RAMP_MA_PER_MS :
      MOTOR_CURRENT_STARTUP_CURRENT_RAMP_MA_PER_MS);
  if (!motor_current_control_command_set(&command))
  {
    motor_current_startup_fault_set(
      MOTOR_CURRENT_STARTUP_FAULT_CURRENT_CONTROL);
    return;
  }
  target_frequency = motor_current_startup_direction *
    (int32_t)MOTOR_CURRENT_STARTUP_HANDOVER_FREQUENCY_MILLIHZ;
  motor_current_startup_frequency_millihz = motor_current_startup_ramp(
    motor_current_startup_frequency_millihz, target_frequency,
    MOTOR_CURRENT_STARTUP_FREQUENCY_RAMP_MILLIHZ_PER_MS);

  if ((!motor_hall_decoder_sample_read(&hall)) ||
      (!motor_hall_angle_estimator_read(&estimator))) return;
  if ((hall.invalid_transition_count !=
       motor_current_startup_last_invalid_count) ||
      (hall.direction != motor_current_startup_direction))
  {
    motor_current_startup_status.qualified_edges = 0U;
    motor_current_startup_last_invalid_count = hall.invalid_transition_count;
    motor_current_startup_last_hall_update = hall.frequency_update_count;
  }
  else if (hall.frequency_update_count !=
           motor_current_startup_last_hall_update)
  {
    motor_current_startup_last_hall_update = hall.frequency_update_count;
    if (motor_current_startup_status.qualified_edges < 255U)
      motor_current_startup_status.qualified_edges++;
  }
  if (motor_current_startup_status.state == MOTOR_CURRENT_STARTUP_IF_RAMP)
    motor_current_startup_status.state = MOTOR_CURRENT_STARTUP_HANDOVER_WAIT;
  if ((motor_current_startup_status.state ==
       MOTOR_CURRENT_STARTUP_HANDOVER_WAIT) &&
      (motor_current_startup_status.qualified_edges >=
       (motor_current_startup_limited_if ?
          MOTOR_SPEED_REVERSE_HALL_REQUIRED_EDGES :
          MOTOR_CURRENT_STARTUP_HALL_REQUIRED_EDGES)) && estimator.valid &&
      (estimator.direction == motor_current_startup_direction) &&
      (estimator.electrical_frequency_millihz >=
       MOTOR_CURRENT_STARTUP_MINIMUM_HALL_FREQUENCY_MILLIHZ))
  {
    /*
     * The I/F phase is intentionally independent of the Hall estimator and
     * can therefore have a fixed phase offset at capture.  Requiring the two
     * angles to happen to be within 30 electrical degrees prevented a valid
     * rotating motor from ever entering the existing shortest-path blend.
     */
    /*
     * Capture the phase displacement once.  During blending the output will
     * follow the Hall estimator trajectory while this fixed displacement is
     * faded out.  Recomputing Hall-minus-I/F on every PWM cycle couples two
     * different speed trajectories and can pull the rotor back and forth.
     */
    motor_current_startup_blend_initial_error = (int16_t)(
      estimator.electrical_angle_u16 -
      (uint16_t)(motor_current_startup_phase_accumulator >> 16));
    motor_current_startup_blend_cycles = 0U;
    motor_current_startup_status.state = MOTOR_CURRENT_STARTUP_ANGLE_BLEND;
  }
  if ((motor_current_startup_status.elapsed_ms >=
       MOTOR_CURRENT_STARTUP_TIMEOUT_MS) &&
      (motor_current_startup_status.state != MOTOR_CURRENT_STARTUP_ANGLE_BLEND))
    motor_current_startup_fault_set(MOTOR_CURRENT_STARTUP_FAULT_HALL_TIMEOUT);
}

void motor_current_startup_fast_process(void)
{
  motor_hall_angle_estimator_t estimator;
  motor_hall_sample_t hall;
  uint16_t if_angle;
  uint16_t output_angle;
  int64_t phase_step;

  if ((motor_current_startup_status.state == MOTOR_CURRENT_STARTUP_STOPPED) ||
      (motor_current_startup_status.state == MOTOR_CURRENT_STARTUP_COMPLETE) ||
      (motor_current_startup_status.state == MOTOR_CURRENT_STARTUP_FAULT)) return;
  /* The 1 kHz task updates the low-speed Hall lookup angle whenever the
   * discrete state changes.  Do not let the old I/F phase accumulator
   * overwrite that angle between Hall-direct updates. */
  if (motor_current_startup_status.state ==
      MOTOR_CURRENT_STARTUP_HALL_DIRECT)
    return;
  if (motor_current_startup_status.state ==
      MOTOR_CURRENT_STARTUP_CURRENT_RAMP)
  {
    /* Keep one angle source for the entire post-capture torque ramp.  Clearing
       the override at the same instant that Iq starts falling allowed a small
       estimator discontinuity to pull the rotor backwards. */
    if (!motor_hall_angle_estimator_read(&estimator))
      return;
    if (!estimator.valid)
    {
      motor_current_startup_direction_invalid_cycles = 0U;
      if (++motor_current_startup_feedback_invalid_cycles <
          MOTOR_CURRENT_STARTUP_FEEDBACK_DEBOUNCE_CYCLES)
        return;
      /* The rotor has not remained captured by Hall feedback.  Resume the
         I/F field and require a fresh run of qualified edges instead of
         latching a fault on the first failed handover attempt. */
      if (motor_current_startup_hall_direct)
      {
        motor_current_startup_status.state =
          MOTOR_CURRENT_STARTUP_HALL_DIRECT;
        motor_current_startup_status.qualified_edges = 0U;
      }
      else
      {
        motor_current_startup_handover_retry();
      }
      return;
    }
    motor_current_startup_feedback_invalid_cycles = 0U;
    if (estimator.direction != motor_current_startup_direction)
    {
      if (++motor_current_startup_direction_invalid_cycles <
          MOTOR_CURRENT_STARTUP_FEEDBACK_DEBOUNCE_CYCLES)
        return;
      if (motor_current_startup_hall_direct)
      {
        motor_current_startup_status.state =
          MOTOR_CURRENT_STARTUP_HALL_DIRECT;
        motor_current_startup_status.qualified_edges = 0U;
      }
      else
      {
        motor_current_startup_handover_retry();
      }
      return;
    }
    motor_current_startup_direction_invalid_cycles = 0U;
    motor_rotor_feedback_override_set(estimator.electrical_angle_u16,
      (int32_t)estimator.electrical_frequency_millihz,
      estimator.direction);
    return;
  }
  if (motor_current_startup_status.state != MOTOR_CURRENT_STARTUP_ALIGNING)
  {
    phase_step = ((int64_t)motor_current_startup_frequency_millihz *
                  4294967296LL) /
      ((int64_t)MOTOR_PWM_FREQUENCY_HZ * 1000L);
    motor_current_startup_phase_accumulator += (uint32_t)phase_step;
  }
  if_angle = (uint16_t)(motor_current_startup_phase_accumulator >> 16);
  output_angle = if_angle;
  if (motor_current_startup_status.state == MOTOR_CURRENT_STARTUP_ANGLE_BLEND)
  {
    /*
     * The Hall decoder snapshot is protected by a sequence counter.  The
     * ADC/control interrupt may preempt a Hall edge update, in which case a
     * read deliberately returns false instead of waiting for the lower
     * priority Hall interrupt.  That is a transient snapshot collision, not
     * a lost Hall signal.  Keep the previous override for this PWM cycle and
     * retry on the next one, just as the 1 kHz qualification path does.
     */
    if ((!motor_hall_angle_estimator_read(&estimator)) ||
        (!motor_hall_decoder_sample_read(&hall)))
    {
      return;
    }
    if (!estimator.valid)
    {
      /*
       * Do not hold the last blended angle while Hall feedback is invalid.
       * Holding it freezes the rotating field and prevents a stalled rotor
       * from being pulled in again.  Resume the live I/F angle immediately
       * and require a fresh sequence of qualified Hall edges before retrying
       * handover.
       */
      motor_current_startup_status.state = motor_current_startup_hall_direct ?
        MOTOR_CURRENT_STARTUP_HALL_DIRECT :
        MOTOR_CURRENT_STARTUP_HANDOVER_WAIT;
      motor_current_startup_status.qualified_edges = 0U;
      motor_current_startup_blend_cycles = 0U;
      if (!motor_current_startup_hall_direct)
        motor_rotor_feedback_override_set(if_angle,
          motor_current_startup_frequency_millihz,
          motor_current_startup_forced_direction_get());
      return;
    }
    if (!hall.valid)
    {
      motor_current_startup_fault_set(
        MOTOR_CURRENT_STARTUP_FAULT_BLEND_HALL_INVALID);
      return;
    }
    if ((estimator.direction != motor_current_startup_direction) ||
        (hall.direction != motor_current_startup_direction))
    {
      /*
       * A lightly loaded rotor can rock back by one Hall sector while the
       * commanded angle is being transferred from I/F to Hall feedback.
       * One legal reverse transition is recoverable and must not latch a
       * startup fault.  Fall back to the I/F angle and require a fresh run
       * of same-direction Hall edges before attempting the blend again.
       * Persistent reverse motion will consequently fail through the
       * normal startup timeout.
       */
      motor_current_startup_status.state = motor_current_startup_hall_direct ?
        MOTOR_CURRENT_STARTUP_HALL_DIRECT :
        MOTOR_CURRENT_STARTUP_HANDOVER_WAIT;
      motor_current_startup_status.qualified_edges = 0U;
      motor_current_startup_last_hall_update = hall.frequency_update_count;
      motor_current_startup_last_invalid_count =
        hall.invalid_transition_count;
      motor_current_startup_blend_cycles = 0U;
      if (!motor_current_startup_hall_direct)
        motor_rotor_feedback_override_set(if_angle,
          motor_current_startup_frequency_millihz,
          motor_current_startup_forced_direction_get());
      return;
    }
    else if (hall.invalid_transition_count !=
        motor_current_startup_last_invalid_count)
    {
      motor_current_startup_fault_set(
        MOTOR_CURRENT_STARTUP_FAULT_BLEND_TRANSITION);
      return;
    }
    output_angle = (uint16_t)(estimator.electrical_angle_u16 - (int32_t)(
      ((int64_t)motor_current_startup_blend_initial_error *
       (MOTOR_CURRENT_STARTUP_BLEND_CYCLES -
        motor_current_startup_blend_cycles)) /
      MOTOR_CURRENT_STARTUP_BLEND_CYCLES));
    motor_current_startup_blend_cycles++;
    if (motor_current_startup_blend_cycles >=
        MOTOR_CURRENT_STARTUP_BLEND_CYCLES)
    {
      /* Hall angle is authoritative.  Keep mirroring it through the following
         current ramp and release the override only after torque has settled. */
      motor_current_startup_status.state =
        MOTOR_CURRENT_STARTUP_CURRENT_RAMP;
      motor_current_startup_blend_complete_ms =
        motor_current_startup_status.elapsed_ms;
      return;
    }
  }
  motor_rotor_feedback_override_set(output_angle,
    motor_current_startup_frequency_millihz,
    motor_current_startup_forced_direction_get());
}

bool motor_current_startup_status_read(motor_current_startup_status_t *status)
{
  uint32_t primask;
  if (status == NULL) return false;
  primask = __get_PRIMASK();
  __disable_irq();
  *status = motor_current_startup_status;
  __set_PRIMASK(primask);
  return true;
}
