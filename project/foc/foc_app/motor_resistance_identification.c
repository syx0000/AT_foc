/**
 * @file    motor_resistance_identification.c
 * @brief   电机相电阻辨识 功能实现
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#include <stddef.h>
#include "at32f45x_conf.h"
#include "motor_board_config.h"
#include "motor_control_config.h"
#include "motor_current_sample.h"
#include "motor_foc_math.h"
#include "motor_pwm_port.h"
#include "motor_resistance_identification.h"
#include "motor_slow_sensor.h"
#include "wk_system.h"

/**
 * @brief 将绝对值限制在32位无符号范围内返回。
 * @param value 有符号电流值。
 * @return 输入值的非负绝对值。
 */
static uint32_t motor_resistance_identification_abs(int32_t value)
{
  return (uint32_t)((value < 0) ? -value : value);
}

/**
 * @brief 将固定d轴电压写入三相PWM比较寄存器。
 * @param voltage_q15 d轴电压相对母线电压的Q15标幺值。
 * @return 无。
 */
static void motor_resistance_identification_voltage_apply(uint16_t voltage_q15)
{
  motor_alpha_beta_q15_t voltage;
  motor_svpwm_duty_q15_t duty;
  motor_pwm_compare_t compare;

  voltage = motor_foc_inverse_park_q15((int16_t)voltage_q15, 0, 0U);
  duty = motor_foc_svpwm_q15(voltage);
  compare.phase_a = (uint16_t)(((uint32_t)duty.phase_a *
                                MOTOR_PWM_PERIOD_COUNTS) / 32767U);
  compare.phase_b = (uint16_t)(((uint32_t)duty.phase_b *
                                MOTOR_PWM_PERIOD_COUNTS) / 32767U);
  compare.phase_c = (uint16_t)(((uint32_t)duty.phase_c *
                                MOTOR_PWM_PERIOD_COUNTS) / 32767U);
  motor_pwm_port_compare_set(&compare);
}

/**
 * @brief 检查辨识期间电流和功率级是否仍处于安全状态。
 * @param current 最近一次正式三相电流状态，不允许为空。
 * @return 三相电流均低于10 A、无软件过流且nFAULT/MOE正常时返回true。
 */
static bool motor_resistance_identification_safe(
  const motor_current_sample_state_t *current)
{
  if ((current == NULL) || current->overcurrent_fault ||
      (motor_resistance_identification_abs(current->phase_a_ma) >=
       MOTOR_RESISTANCE_IDENT_ABORT_CURRENT_MA) ||
      (motor_resistance_identification_abs(current->phase_b_ma) >=
       MOTOR_RESISTANCE_IDENT_ABORT_CURRENT_MA) ||
      (motor_resistance_identification_abs(current->phase_c_ma) >=
       MOTOR_RESISTANCE_IDENT_ABORT_CURRENT_MA))
  {
    return false;
  }

  return (gpio_input_data_bit_read(MOTOR_PWM_BREAK_PORT,
                                   MOTOR_PWM_BREAK_PIN) != RESET) &&
         (MOTOR_PWM_TIMER->brk_bit.oen != 0U);
}

/**
 * @brief 检查安全条件，失败时急停并写入对应状态。
 * @param result 辨识结果，失败时写入status。
 * @return 安全返回true；过流或驱动器故障返回false且PWM已关闭。
 */
static bool motor_resistance_identification_check(
  motor_resistance_identification_result_t *result,
  motor_current_sample_state_t *current)
{
  if ((!motor_current_sample_state_read(current)) ||
      (!motor_resistance_identification_safe(current)))
  {
    result->status = (MOTOR_PWM_TIMER->brk_bit.oen == 0U) ?
      MOTOR_RESISTANCE_IDENT_DRIVER_FAULT : MOTOR_RESISTANCE_IDENT_OVERCURRENT;
    motor_pwm_port_emergency_stop();
    return false;
  }
  return true;
}

/**
 * @brief 从当前电压继续升压到目标电流，稳定后测量一个工作点。
 * @param result 辨识结果，失败时写入status。
 * @param bus_voltage_mv 母线电压，单位mV。
 * @param target_current_ma 本工作点A相目标电流，单位mA。
 * @param requested_voltage_mv 输入为当前指令电压，输出为本点最终指令电压。
 * @param voltage_q15 输出本点最终施加的Q15电压。
 * @param average_ma 输出本点A相平均电流，单位mA。
 * @return 测量完成返回true；安全检查失败返回false且PWM已关闭。
 * @details 以1 mV/ms升压直到A相电流达到目标或电压达到上限，稳定后以1 ms
 *          周期平均。达到电压上限时不报错，由调用方检查两点电流差。
 */
static bool motor_resistance_identification_point(
  motor_resistance_identification_result_t *result,
  uint32_t bus_voltage_mv, int32_t target_current_ma,
  uint32_t *requested_voltage_mv, uint16_t *voltage_q15,
  int32_t *average_ma)
{
  motor_current_sample_state_t current;
  uint32_t index;
  int64_t current_sum_ma = 0;

  while (*requested_voltage_mv < MOTOR_RESISTANCE_IDENT_MAX_VOLTAGE_MV)
  {
    if (!motor_resistance_identification_check(result, &current)) return false;
    if (current.phase_a_ma >= target_current_ma) break;
    *requested_voltage_mv += MOTOR_RESISTANCE_IDENT_VOLTAGE_STEP_MV;
    *voltage_q15 = (uint16_t)(((uint64_t)*requested_voltage_mv * 32768U) /
                              bus_voltage_mv);
    motor_resistance_identification_voltage_apply(*voltage_q15);
    wk_delay_ms(1U);
  }

  for (index = 0U; index < MOTOR_RESISTANCE_IDENT_SETTLE_MS; index++)
  {
    wk_delay_ms(1U);
    if (!motor_resistance_identification_check(result, &current)) return false;
  }

  for (index = 0U; index < MOTOR_RESISTANCE_IDENT_MEASURE_MS; index++)
  {
    wk_delay_ms(1U);
    if (!motor_resistance_identification_check(result, &current)) return false;
    current_sum_ma += current.phase_a_ma;
  }
  *average_ma = (int32_t)(current_sum_ma /
                          (int64_t)MOTOR_RESISTANCE_IDENT_MEASURE_MS);
  result->sample_count += MOTOR_RESISTANCE_IDENT_MEASURE_MS;
  return true;
}

bool motor_resistance_identification_run(
  motor_resistance_identification_result_t *result)
{
  motor_slow_sensor_state_t sensor;
  motor_current_sample_state_t current;
  uint32_t bus_voltage_mv;
  uint32_t requested_voltage_mv = 0U;
  uint16_t voltage_q15 = 0U;
  int64_t low_voltage_uv;
  int64_t high_voltage_uv;
  int32_t delta_current_ma;

  if (result == NULL)
  {
    return false;
  }

  result->status = MOTOR_RESISTANCE_IDENT_INVALID_STATE;
  result->applied_voltage_mv = 0U;
  result->phase_a_average_ma = 0;
  result->direct_average_ma = 0;
  result->low_voltage_mv = 0U;
  result->low_current_ma = 0;
  result->offset_voltage_mv = 0;
  result->resistance_via_phase_a_mohm = 0U;
  result->resistance_via_direct_mohm = 0U;
  result->resistance_average_mohm = 0U;
  result->sample_count = 0U;

  (void)motor_slow_sensor_process();
  if ((!motor_slow_sensor_state_read(&sensor)) ||
      (sensor.bus_voltage_0p1v < MOTOR_BUS_UNDERVOLTAGE_0P1V) ||
      (sensor.bus_voltage_0p1v > MOTOR_BUS_OVERVOLTAGE_0P1V))
  {
    result->status = MOTOR_RESISTANCE_IDENT_BUS_VOLTAGE_INVALID;
    return false;
  }
  if ((!motor_current_sample_state_read(&current)) ||
      current.overcurrent_fault || motor_pwm_port_output_is_enabled())
  {
    return false;
  }

  bus_voltage_mv = (uint32_t)sensor.bus_voltage_0p1v * 100U;
  motor_resistance_identification_voltage_apply(0U);
  if (!motor_pwm_port_output_enable())
  {
    result->status = MOTOR_RESISTANCE_IDENT_PWM_ENABLE_FAILED;
    motor_pwm_port_emergency_stop();
    return false;
  }

  /* 两点差分：死区压降和电流零偏在同方向两工作点上近似相同，相减抵消。 */
  if (!motor_resistance_identification_point(
        result, bus_voltage_mv, MOTOR_RESISTANCE_IDENT_LOW_CURRENT_MA,
        &requested_voltage_mv, &voltage_q15, &result->low_current_ma))
  {
    return false;
  }
  low_voltage_uv = ((int64_t)voltage_q15 * bus_voltage_mv * 1000) / 32768;
  result->low_voltage_mv = (uint16_t)(low_voltage_uv / 1000);

  if (!motor_resistance_identification_point(
        result, bus_voltage_mv, MOTOR_RESISTANCE_IDENT_TARGET_CURRENT_MA,
        &requested_voltage_mv, &voltage_q15, &result->phase_a_average_ma))
  {
    return false;
  }
  high_voltage_uv = ((int64_t)voltage_q15 * bus_voltage_mv * 1000) / 32768;
  result->applied_voltage_mv = (uint16_t)(high_voltage_uv / 1000);
  result->direct_average_ma = result->phase_a_average_ma;

  motor_pwm_port_output_disable();

  if (result->phase_a_average_ma < MOTOR_RESISTANCE_IDENT_MIN_CURRENT_MA)
  {
    result->status = MOTOR_RESISTANCE_IDENT_CURRENT_TOO_LOW;
    return false;
  }
  /* 旧单点结果保留作对照，包含死区压降误差。 */
  result->resistance_via_phase_a_mohm = (uint32_t)(
    high_voltage_uv / result->phase_a_average_ma);

  delta_current_ma = result->phase_a_average_ma - result->low_current_ma;
  if ((result->low_current_ma < MOTOR_RESISTANCE_IDENT_MIN_CURRENT_MA) ||
      (delta_current_ma < MOTOR_RESISTANCE_IDENT_MIN_DELTA_CURRENT_MA) ||
      (high_voltage_uv <= low_voltage_uv))
  {
    result->status = MOTOR_RESISTANCE_IDENT_CURRENT_TOO_LOW;
    return false;
  }
  result->resistance_via_direct_mohm = (uint32_t)(
    (high_voltage_uv - low_voltage_uv) / delta_current_ma);
  /* 截距 = 死区压降 + 零偏等效压降，用于诊断。 */
  result->offset_voltage_mv = (int32_t)(
    (high_voltage_uv - (int64_t)result->resistance_via_direct_mohm *
     result->phase_a_average_ma) / 1000);
  result->resistance_average_mohm = result->resistance_via_direct_mohm;
  result->status = MOTOR_RESISTANCE_IDENT_OK;
  return true;
}
