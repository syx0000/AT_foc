#include <stddef.h>
#include "at32f45x.h"
#include "motor_board_config.h"
#include "motor_control_config.h"
#include "motor_hall_angle_estimator.h"
#include "motor_hall_port.h"
#include "motor_timebase.h"

static const uint16_t motor_hall_edge_angle_u16[8] =
{
  0U,
  MOTOR_HALL_EDGE_ANGLE_STATE_1_U16,
  MOTOR_HALL_EDGE_ANGLE_STATE_2_U16,
  MOTOR_HALL_EDGE_ANGLE_STATE_3_U16,
  MOTOR_HALL_EDGE_ANGLE_STATE_4_U16,
  MOTOR_HALL_EDGE_ANGLE_STATE_5_U16,
  MOTOR_HALL_EDGE_ANGLE_STATE_6_U16,
  0U
};

static volatile motor_hall_angle_estimator_t motor_hall_angle_estimator;
static uint32_t motor_hall_phase_accumulator;
static uint32_t motor_hall_phase_step;
static uint32_t motor_hall_last_edge_cycles;

void motor_hall_angle_estimator_init(void)
{
  motor_hall_angle_estimator.electrical_angle_u16 = 0U;
  motor_hall_angle_estimator.electrical_frequency_millihz = 0U;
  motor_hall_angle_estimator.hall_state = 0U;
  motor_hall_angle_estimator.valid = false;
  motor_hall_phase_accumulator = 0U;
  motor_hall_phase_step = 0U;
  motor_hall_last_edge_cycles = motor_timebase_cycles_get();
}

void motor_hall_angle_estimator_edge_process(void)
{
  motor_hall_sample_t hall_sample;

  if ((!motor_hall_port_sample_read(&hall_sample)) ||
      (!hall_sample.valid) ||
      (hall_sample.direction != 1) ||
      (hall_sample.electrical_frequency_millihz == 0U))
  {
    motor_hall_angle_estimator.valid = false;
    return;
  }

  motor_hall_phase_accumulator =
    (uint32_t)motor_hall_edge_angle_u16[hall_sample.state] << 16;
  motor_hall_phase_step =
    (uint32_t)(((uint64_t)hall_sample.electrical_frequency_millihz << 32) /
               ((uint64_t)MOTOR_PWM_FREQUENCY_HZ * 1000U));
  motor_hall_last_edge_cycles = hall_sample.timestamp_cycles;
  motor_hall_angle_estimator.electrical_angle_u16 =
    motor_hall_edge_angle_u16[hall_sample.state];
  motor_hall_angle_estimator.electrical_frequency_millihz =
    hall_sample.electrical_frequency_millihz;
  motor_hall_angle_estimator.hall_state = hall_sample.state;
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
    return;
  }

  motor_hall_phase_accumulator += motor_hall_phase_step;
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
