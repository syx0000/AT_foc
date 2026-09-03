#include <stddef.h>
#include "at32f45x.h"
#include "motor_current_transform.h"
#include "motor_current_sample.h"
#include "motor_foc_math.h"
#include "motor_hall_angle_estimator.h"

static volatile motor_current_transform_state_t motor_current_transform_state;
static volatile int64_t motor_current_direct_sum_q15;
static volatile int64_t motor_current_quadrature_sum_q15;
static volatile int16_t motor_current_direct_minimum_q15;
static volatile int16_t motor_current_direct_maximum_q15;
static volatile int16_t motor_current_quadrature_minimum_q15;
static volatile int16_t motor_current_quadrature_maximum_q15;
static volatile uint32_t motor_current_statistics_sample_count;

void motor_current_transform_init(void)
{
  motor_current_transform_state.alpha_q15 = 0;
  motor_current_transform_state.beta_q15 = 0;
  motor_current_transform_state.direct_q15 = 0;
  motor_current_transform_state.quadrature_q15 = 0;
  motor_current_transform_state.electrical_angle_u16 = 0U;
  motor_current_transform_state.sample_count = 0U;
  motor_current_transform_state.valid = false;
  motor_current_direct_sum_q15 = 0;
  motor_current_quadrature_sum_q15 = 0;
  motor_current_direct_minimum_q15 = 0;
  motor_current_direct_maximum_q15 = 0;
  motor_current_quadrature_minimum_q15 = 0;
  motor_current_quadrature_maximum_q15 = 0;
  motor_current_statistics_sample_count = 0U;
}

void motor_current_transform_fast_process(void)
{
  motor_current_sample_state_t current;
  motor_hall_angle_estimator_t angle;
  motor_alpha_beta_q15_t alpha_beta;
  motor_direct_quadrature_q15_t direct_quadrature;

  if ((!motor_current_sample_state_read(&current)) ||
      (!motor_hall_angle_estimator_read(&angle)) ||
      (!angle.valid))
  {
    motor_current_transform_state.valid = false;
    return;
  }

  alpha_beta = motor_foc_clarke_q15(current.phase_a_q15,
                                    current.phase_b_q15);
  direct_quadrature = motor_foc_park_q15(alpha_beta,
                                         angle.electrical_angle_u16);
  motor_current_transform_state.alpha_q15 = alpha_beta.alpha_q15;
  motor_current_transform_state.beta_q15 = alpha_beta.beta_q15;
  motor_current_transform_state.direct_q15 = direct_quadrature.direct_q15;
  motor_current_transform_state.quadrature_q15 = direct_quadrature.quadrature_q15;
  motor_current_transform_state.electrical_angle_u16 = angle.electrical_angle_u16;
  motor_current_transform_state.sample_count++;
  motor_current_transform_state.valid = true;

  if (motor_current_statistics_sample_count == 0U)
  {
    motor_current_direct_minimum_q15 = direct_quadrature.direct_q15;
    motor_current_direct_maximum_q15 = direct_quadrature.direct_q15;
    motor_current_quadrature_minimum_q15 = direct_quadrature.quadrature_q15;
    motor_current_quadrature_maximum_q15 = direct_quadrature.quadrature_q15;
  }
  else
  {
    if (direct_quadrature.direct_q15 < motor_current_direct_minimum_q15)
      motor_current_direct_minimum_q15 = direct_quadrature.direct_q15;
    if (direct_quadrature.direct_q15 > motor_current_direct_maximum_q15)
      motor_current_direct_maximum_q15 = direct_quadrature.direct_q15;
    if (direct_quadrature.quadrature_q15 < motor_current_quadrature_minimum_q15)
      motor_current_quadrature_minimum_q15 = direct_quadrature.quadrature_q15;
    if (direct_quadrature.quadrature_q15 > motor_current_quadrature_maximum_q15)
      motor_current_quadrature_maximum_q15 = direct_quadrature.quadrature_q15;
  }
  motor_current_direct_sum_q15 += direct_quadrature.direct_q15;
  motor_current_quadrature_sum_q15 += direct_quadrature.quadrature_q15;
  motor_current_statistics_sample_count++;
}

bool motor_current_transform_state_read(motor_current_transform_state_t *state)
{
  uint32_t primask;

  if (state == NULL)
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *state = motor_current_transform_state;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return true;
}

bool motor_current_transform_statistics_snapshot_reset(
  motor_current_transform_statistics_t *statistics)
{
  uint32_t primask;
  uint32_t sample_count;
  int64_t direct_sum;
  int64_t quadrature_sum;

  if (statistics == NULL)
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  sample_count = motor_current_statistics_sample_count;
  direct_sum = motor_current_direct_sum_q15;
  quadrature_sum = motor_current_quadrature_sum_q15;
  statistics->direct_minimum_q15 = motor_current_direct_minimum_q15;
  statistics->direct_maximum_q15 = motor_current_direct_maximum_q15;
  statistics->quadrature_minimum_q15 = motor_current_quadrature_minimum_q15;
  statistics->quadrature_maximum_q15 = motor_current_quadrature_maximum_q15;
  motor_current_direct_sum_q15 = 0;
  motor_current_quadrature_sum_q15 = 0;
  motor_current_statistics_sample_count = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (sample_count == 0U)
  {
    statistics->sample_count = 0U;
    statistics->direct_average_q15 = 0;
    statistics->quadrature_average_q15 = 0;
    return false;
  }

  statistics->sample_count = sample_count;
  statistics->direct_average_q15 = (int16_t)(direct_sum / sample_count);
  statistics->quadrature_average_q15 = (int16_t)(quadrature_sum / sample_count);
  return true;
}
