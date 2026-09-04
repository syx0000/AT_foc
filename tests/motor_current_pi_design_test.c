#include <assert.h>
#include <stdio.h>
#include "motor_current_pi_design.h"

int main(void)
{
  motor_current_pi_design_result_t result;
  assert(motor_current_pi_design(273U, 225U, 206U, 200U, 10000U, &result));
  assert(result.direct_kp_q15 == 9265);
  assert(result.quadrature_kp_q15 == 8483);
  assert(result.integral_gain_q15 == 1124);
  assert(!motor_current_pi_design(0U, 225U, 206U, 200U, 10000U, &result));
  assert(!motor_current_pi_design(273U, 225U, 206U, 10000U, 10000U, &result));
  printf("motor_current_pi_design_test: PASS\n");
  return 0;
}
