#include <stddef.h>
#include "at32f45x.h"
#include "motor_hall_angle_observer.h"
#include "motor_hall_port.h"
#include "motor_open_loop.h"

static volatile motor_hall_angle_observer_t motor_hall_angle_observer;

void motor_hall_angle_observer_init(void)
{
  uint32_t index;

  for (index = 0U; index < 8U; index++)
  {
    motor_hall_angle_observer.state_angle_u16[index] = 0U;
    motor_hall_angle_observer.state_count[index] = 0U;
  }
}

void motor_hall_angle_observer_edge_process(void)
{
  motor_hall_sample_t hall_sample;
  motor_open_loop_status_t open_loop_status;
  uint8_t state;
  uint16_t filtered_angle;
  int16_t angle_error;

  if ((!motor_hall_port_sample_read(&hall_sample)) ||
      (!motor_open_loop_status_read(&open_loop_status)) ||
      (!hall_sample.valid) ||
      (open_loop_status.state != MOTOR_OPEN_LOOP_RUNNING))
  {
    return;
  }

  state = hall_sample.state;
  if (motor_hall_angle_observer.state_count[state] == 0U)
  {
    filtered_angle = open_loop_status.electrical_angle_u16;
  }
  else
  {
    filtered_angle = motor_hall_angle_observer.state_angle_u16[state];
    angle_error = (int16_t)(open_loop_status.electrical_angle_u16 - filtered_angle);
    filtered_angle = (uint16_t)(filtered_angle + (angle_error / 8));
  }

  motor_hall_angle_observer.state_angle_u16[state] = filtered_angle;
  motor_hall_angle_observer.state_count[state]++;
}

bool motor_hall_angle_observer_read(motor_hall_angle_observer_t *observer)
{
  uint32_t primask;

  if (observer == NULL)
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *observer = motor_hall_angle_observer;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return true;
}
