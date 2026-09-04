#include <assert.h>
#include <stdio.h>
#include "motor_hall_offset_identification.h"

int main(void)
{
  motor_hall_offset_identification_input_t input;
  motor_hall_offset_identification_result_t result;

  input.direct_voltage_mv = -2910;
  input.quadrature_voltage_mv = 3333;
  input.current_offset_u16 = 0U;
  input.maximum_correction_u16 = 10923U;
  input.minimum_quadrature_voltage_mv = 500U;
  assert(motor_hall_offset_identify(&input, &result));
  assert(result.status == MOTOR_HALL_OFFSET_IDENT_OK);
  assert((result.correction_u16 >= 7460) &&
         (result.correction_u16 <= 7500));
  assert(result.candidate_offset_u16 == (uint16_t)result.correction_u16);
  assert((result.correction_degrees_x10 >= 409U) &&
         (result.correction_degrees_x10 <= 413U));

  input.direct_voltage_mv = 0;
  input.quadrature_voltage_mv = 3000;
  input.current_offset_u16 = 7482U;
  assert(motor_hall_offset_identify(&input, &result));
  assert(result.correction_u16 == 0);
  assert(result.candidate_offset_u16 == 7482U);

  input.direct_voltage_mv = 1000;
  input.quadrature_voltage_mv = 3000;
  assert(motor_hall_offset_identify(&input, &result));
  assert(result.correction_u16 < 0);

  input.direct_voltage_mv = -3000;
  input.quadrature_voltage_mv = 100;
  assert(!motor_hall_offset_identify(&input, &result));
  assert(result.status == MOTOR_HALL_OFFSET_IDENT_QUADRATURE_VOLTAGE_TOO_LOW);

  input.direct_voltage_mv = -6000;
  input.quadrature_voltage_mv = 1000;
  assert(!motor_hall_offset_identify(&input, &result));
  assert(result.status == MOTOR_HALL_OFFSET_IDENT_CORRECTION_EXCEEDED);

  printf("motor_hall_offset_identification_test: PASS\n");
  return 0;
}
