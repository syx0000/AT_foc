#include "motor_foc_math.h"

#define MOTOR_Q15_ONE                32767
#define MOTOR_Q15_HALF               16384
#define MOTOR_Q15_SQRT3_BY_2         28378
#define MOTOR_Q15_ONE_BY_SQRT3       18919
#define MOTOR_SINE_LUT_INTERVALS     64U
#define MOTOR_SINE_LUT_INDEX_SHIFT   8U
#define MOTOR_SINE_LUT_FRACTION_MASK 0xFFU

static const int16_t motor_sine_quarter_lut[MOTOR_SINE_LUT_INTERVALS + 1U] =
{
      0,   804,  1608,  2410,  3212,  4011,  4808,  5602,
   6393,  7179,  7962,  8739,  9512, 10278, 11039, 11793,
  12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530,
  18204, 18868, 19519, 20159, 20787, 21403, 22005, 22594,
  23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790,
  27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956,
  30273, 30571, 30852, 31113, 31356, 31580, 31785, 31971,
  32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757,
  32767
};

/**
 * @brief 查询第一象限内的Q15正弦值。
 * @param quarter_angle_u14 第一象限角度，0..16384对应0..90度。
 * @return 插值后的正弦Q15值，范围0..32767。
 */
static int16_t motor_foc_sine_quarter_get(uint16_t quarter_angle_u14)
{
  uint16_t index;
  uint16_t fraction;
  int32_t value;
  int32_t delta;

  if (quarter_angle_u14 >= 16384U)
  {
    return MOTOR_Q15_ONE;
  }

  index = quarter_angle_u14 >> MOTOR_SINE_LUT_INDEX_SHIFT;
  fraction = quarter_angle_u14 & MOTOR_SINE_LUT_FRACTION_MASK;
  value = motor_sine_quarter_lut[index];
  delta = (int32_t)motor_sine_quarter_lut[index + 1U] - value;
  value += (delta * fraction + 128) >> 8;
  return (int16_t)value;
}

/**
 * @brief 将32位数限制到Q15有符号范围。
 * @param value 待限制数值。
 * @return 限制后的int16_t数值。
 */
static int16_t motor_foc_q15_saturate(int32_t value)
{
  if (value > 32767)
  {
    return 32767;
  }
  if (value < -32768)
  {
    return -32768;
  }
  return (int16_t)value;
}

motor_sin_cos_q15_t motor_foc_sin_cos_q15(uint16_t electrical_angle_u16)
{
  motor_sin_cos_q15_t result;
  uint16_t quadrant = electrical_angle_u16 >> 14;
  uint16_t quadrant_angle = electrical_angle_u16 & 0x3FFFU;
  int16_t sine_forward = motor_foc_sine_quarter_get(quadrant_angle);
  int16_t sine_reverse = motor_foc_sine_quarter_get(16384U - quadrant_angle);

  switch (quadrant)
  {
    case 0U:
      result.sin_q15 = sine_forward;
      result.cos_q15 = sine_reverse;
      break;
    case 1U:
      result.sin_q15 = sine_reverse;
      result.cos_q15 = (int16_t)-sine_forward;
      break;
    case 2U:
      result.sin_q15 = (int16_t)-sine_forward;
      result.cos_q15 = (int16_t)-sine_reverse;
      break;
    default:
      result.sin_q15 = (int16_t)-sine_reverse;
      result.cos_q15 = sine_forward;
      break;
  }

  return result;
}

motor_alpha_beta_q15_t motor_foc_inverse_park_q15(int16_t voltage_d_q15,
                                                  int16_t voltage_q_q15,
                                                  uint16_t electrical_angle_u16)
{
  motor_alpha_beta_q15_t result;
  motor_sin_cos_q15_t angle = motor_foc_sin_cos_q15(electrical_angle_u16);
  int32_t alpha;
  int32_t beta;

  alpha = ((int32_t)voltage_d_q15 * angle.cos_q15 -
           (int32_t)voltage_q_q15 * angle.sin_q15) >> 15;
  beta = ((int32_t)voltage_d_q15 * angle.sin_q15 +
          (int32_t)voltage_q_q15 * angle.cos_q15) >> 15;
  result.alpha_q15 = motor_foc_q15_saturate(alpha);
  result.beta_q15 = motor_foc_q15_saturate(beta);
  return result;
}

motor_alpha_beta_q15_t motor_foc_clarke_q15(int16_t phase_a_q15,
                                             int16_t phase_b_q15)
{
  motor_alpha_beta_q15_t result;
  int32_t beta_input = (int32_t)phase_a_q15 + (int32_t)phase_b_q15 * 2;
  int32_t beta = (int32_t)(((int64_t)beta_input * MOTOR_Q15_ONE_BY_SQRT3) >> 15);

  result.alpha_q15 = phase_a_q15;
  result.beta_q15 = motor_foc_q15_saturate(beta);
  return result;
}

motor_direct_quadrature_q15_t motor_foc_park_q15(
  motor_alpha_beta_q15_t alpha_beta,
  uint16_t electrical_angle_u16)
{
  motor_direct_quadrature_q15_t result;
  motor_sin_cos_q15_t angle = motor_foc_sin_cos_q15(electrical_angle_u16);
  int32_t direct;
  int32_t quadrature;

  direct = (int32_t)(((int64_t)alpha_beta.alpha_q15 * angle.cos_q15 +
                      (int64_t)alpha_beta.beta_q15 * angle.sin_q15) >> 15);
  quadrature = (int32_t)((-(int64_t)alpha_beta.alpha_q15 * angle.sin_q15 +
                           (int64_t)alpha_beta.beta_q15 * angle.cos_q15) >> 15);
  result.direct_q15 = motor_foc_q15_saturate(direct);
  result.quadrature_q15 = motor_foc_q15_saturate(quadrature);
  return result;
}

motor_svpwm_duty_q15_t motor_foc_svpwm_q15(motor_alpha_beta_q15_t voltage)
{
  motor_svpwm_duty_q15_t result;
  int32_t phase_a = voltage.alpha_q15;
  int32_t beta_scaled = ((int32_t)voltage.beta_q15 * MOTOR_Q15_SQRT3_BY_2) >> 15;
  int32_t phase_b = -(int32_t)voltage.alpha_q15 / 2 + beta_scaled;
  int32_t phase_c = -(int32_t)voltage.alpha_q15 / 2 - beta_scaled;
  int32_t maximum = phase_a;
  int32_t minimum = phase_a;
  int32_t common_mode;
  int32_t duty_a;
  int32_t duty_b;
  int32_t duty_c;

  if (phase_b > maximum) maximum = phase_b;
  if (phase_c > maximum) maximum = phase_c;
  if (phase_b < minimum) minimum = phase_b;
  if (phase_c < minimum) minimum = phase_c;

  common_mode = -(maximum + minimum) / 2;
  duty_a = MOTOR_Q15_HALF + phase_a + common_mode;
  duty_b = MOTOR_Q15_HALF + phase_b + common_mode;
  duty_c = MOTOR_Q15_HALF + phase_c + common_mode;

  if (duty_a < 0) duty_a = 0;
  if (duty_b < 0) duty_b = 0;
  if (duty_c < 0) duty_c = 0;
  if (duty_a > MOTOR_Q15_ONE) duty_a = MOTOR_Q15_ONE;
  if (duty_b > MOTOR_Q15_ONE) duty_b = MOTOR_Q15_ONE;
  if (duty_c > MOTOR_Q15_ONE) duty_c = MOTOR_Q15_ONE;

  result.phase_a = (uint16_t)duty_a;
  result.phase_b = (uint16_t)duty_b;
  result.phase_c = (uint16_t)duty_c;
  return result;
}
