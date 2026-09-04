#include <stddef.h>
#include "motor_foc_math.h"
#include "motor_hall_offset_identification.h"

#define MOTOR_HALL_OFFSET_QUARTER_TURN_U16 16384L
#define MOTOR_HALL_OFFSET_DEGREES_X10_PER_TURN 3600L

static int32_t motor_hall_offset_angle_find(int32_t y, int32_t x)
{
  int32_t low = -MOTOR_HALL_OFFSET_QUARTER_TURN_U16;
  int32_t high = MOTOR_HALL_OFFSET_QUARTER_TURN_U16;
  uint32_t iteration;

  for (iteration = 0U; iteration < 16U; iteration++)
  {
    int32_t middle = low + ((high - low) / 2L);
    motor_sin_cos_q15_t trigonometry =
      motor_foc_sin_cos_q15((uint16_t)middle);
    int64_t comparison =
      ((int64_t)y * trigonometry.cos_q15) -
      ((int64_t)x * trigonometry.sin_q15);

    if (comparison > 0)
      low = middle + 1L;
    else
      high = middle;
  }
  return low;
}

bool motor_hall_offset_identify(
  const motor_hall_offset_identification_input_t *input,
  motor_hall_offset_identification_result_t *result)
{
  int32_t correction;
  uint32_t absolute_correction;

  if (result == NULL) return false;
  result->status = MOTOR_HALL_OFFSET_IDENT_INVALID_ARGUMENT;
  if ((input == NULL) ||
      (input->minimum_quadrature_voltage_mv == 0U) ||
      (input->maximum_correction_u16 == 0U) ||
      (input->maximum_correction_u16 >= MOTOR_HALL_OFFSET_QUARTER_TURN_U16))
  {
    return false;
  }
  if (input->quadrature_voltage_mv <
      (int32_t)input->minimum_quadrature_voltage_mv)
  {
    result->status = MOTOR_HALL_OFFSET_IDENT_QUADRATURE_VOLTAGE_TOO_LOW;
    return false;
  }

  correction = motor_hall_offset_angle_find(
    -input->direct_voltage_mv, input->quadrature_voltage_mv);
  absolute_correction = (correction < 0) ?
    (uint32_t)(-correction) : (uint32_t)correction;
  if (absolute_correction > input->maximum_correction_u16)
  {
    result->status = MOTOR_HALL_OFFSET_IDENT_CORRECTION_EXCEEDED;
    return false;
  }

  result->correction_u16 = (int16_t)correction;
  result->candidate_offset_u16 =
    (uint16_t)(input->current_offset_u16 + correction);
  result->correction_degrees_x10 = (uint16_t)(
    ((absolute_correction * MOTOR_HALL_OFFSET_DEGREES_X10_PER_TURN) +
     32768U) / 65536U);
  result->status = MOTOR_HALL_OFFSET_IDENT_OK;
  return true;
}
