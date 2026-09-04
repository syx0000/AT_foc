#include <stddef.h>
#include "at32f45x.h"
#include "motor_board_config.h"
#include "motor_control_config.h"
#include "motor_hall_angle_estimator.h"
#include "motor_hall_angle_math.h"
#include "motor_hall_decoder.h"
#include "motor_parameter.h"
#include "motor_timebase.h"

static volatile motor_hall_angle_estimator_t motor_hall_angle_estimator;
static uint32_t motor_hall_phase_accumulator;
static int32_t motor_hall_phase_step;
static uint32_t motor_hall_last_edge_cycles;

void motor_hall_angle_estimator_init(void)
{
  motor_hall_angle_estimator.electrical_angle_u16 = 0U;
  motor_hall_angle_estimator.electrical_frequency_millihz = 0U;
  motor_hall_angle_estimator.hall_state = 0U;
  motor_hall_angle_estimator.direction = 0;
  motor_hall_angle_estimator.valid = false;
  motor_hall_phase_accumulator = 0U;
  motor_hall_phase_step = 0U;
  motor_hall_last_edge_cycles = motor_timebase_cycles_get();
}

void motor_hall_angle_estimator_edge_process(void)
{
  motor_hall_sample_t hall_sample;
  motor_parameter_t parameter;
  uint16_t boundary_angle_u16;

  if ((!motor_hall_decoder_sample_read(&hall_sample)) ||
      (!motor_parameter_active_read(&parameter)) ||
      (!hall_sample.valid) ||
      (hall_sample.direction == 0) ||
      (hall_sample.electrical_frequency_millihz == 0U))
  {
    motor_hall_angle_estimator.valid = false;
    motor_hall_angle_estimator.direction = 0;
    return;
  }

  if ((!motor_hall_angle_boundary_get(
        parameter.hall_positive_next, parameter.hall_entry_angle_u16,
        hall_sample.state, hall_sample.direction,
        parameter.hall_rotor_offset_u16, &boundary_angle_u16)) ||
      (!motor_hall_phase_step_get(
        hall_sample.electrical_frequency_millihz,
        MOTOR_PWM_FREQUENCY_HZ, hall_sample.direction,
        &motor_hall_phase_step)))
  {
    motor_hall_angle_estimator.valid = false;
    motor_hall_angle_estimator.direction = 0;
    return;
  }
  motor_hall_phase_accumulator = (uint32_t)boundary_angle_u16 << 16;
  motor_hall_last_edge_cycles = hall_sample.timestamp_cycles;
  motor_hall_angle_estimator.electrical_angle_u16 = boundary_angle_u16;
  motor_hall_angle_estimator.electrical_frequency_millihz =
    hall_sample.electrical_frequency_millihz;
  motor_hall_angle_estimator.hall_state = hall_sample.state;
  motor_hall_angle_estimator.direction = hall_sample.direction;
  motor_hall_angle_estimator.valid = true;
}

void motor_hall_angle_estimator_fast_process(void)
{
  uint32_t timeout_cycles =
    (system_core_clock / 1000U) * MOTOR_HALL_SIGNAL_TIMEOUT_MS;

  if (!motor_hall_angle_estimator.valid)
  {
    return;
  }

  if (motor_timebase_cycles_elapsed(motor_hall_last_edge_cycles) > timeout_cycles)
  {
    motor_hall_angle_estimator.valid = false;
    motor_hall_angle_estimator.direction = 0;
    return;
  }

  motor_hall_phase_accumulator += (uint32_t)motor_hall_phase_step;
  motor_hall_angle_estimator.electrical_angle_u16 =
    (uint16_t)(motor_hall_phase_accumulator >> 16);
}

bool motor_hall_angle_estimator_read(motor_hall_angle_estimator_t *estimator)
{
  uint32_t primask;

  if (estimator == NULL)
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *estimator = motor_hall_angle_estimator;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return true;
}
