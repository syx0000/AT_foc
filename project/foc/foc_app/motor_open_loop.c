#include <stddef.h>
#include "motor_open_loop.h"
#include "motor_board_config.h"
#include "motor_current_sample.h"
#include "motor_foc_math.h"
#include "motor_pwm_port.h"

#define MOTOR_OPEN_LOOP_ALIGN_VOLTAGE_Q15       983
#define MOTOR_OPEN_LOOP_RUN_VOLTAGE_Q15        1311
#define MOTOR_OPEN_LOOP_ALIGN_TICKS             5000U
#define MOTOR_OPEN_LOOP_START_FREQUENCY_MILLIHZ 2000U
#define MOTOR_OPEN_LOOP_TARGET_FREQUENCY_MILLIHZ 10000U
#define MOTOR_OPEN_LOOP_RAMP_TICKS              30000U

static volatile motor_open_loop_status_t motor_open_loop_status;
static uint32_t motor_open_loop_phase_accumulator;
static uint32_t motor_open_loop_state_ticks;

/**
 * @brief 将Q15占空比换算为TMR1比较值。
 * @param duty_q15 占空比，0..32767对应0..100%。
 * @return 限制在0..MOTOR_PWM_COMPARE_MAX的定时器比较值。
 */
static uint16_t motor_open_loop_duty_to_compare(uint16_t duty_q15)
{
  uint32_t compare = ((uint32_t)duty_q15 * MOTOR_PWM_PERIOD_COUNTS + 16383U) /
                     32767U;
  if (compare > MOTOR_PWM_COMPARE_MAX)
  {
    compare = MOTOR_PWM_COMPARE_MAX;
  }
  return (uint16_t)compare;
}

/**
 * @brief 计算并写入一次开环三相电压命令。
 * @param voltage_d_q15 d轴电压Q15标幺值。
 * @param voltage_q_q15 q轴电压Q15标幺值。
 * @param electrical_angle_u16 电角度，0..65535对应一周。
 * @return 无。
 */
static void motor_open_loop_voltage_apply(int16_t voltage_d_q15,
                                          int16_t voltage_q_q15,
                                          uint16_t electrical_angle_u16)
{
  motor_alpha_beta_q15_t voltage;
  motor_svpwm_duty_q15_t duty;
  motor_pwm_compare_t compare;

  voltage = motor_foc_inverse_park_q15(voltage_d_q15,
                                       voltage_q_q15,
                                       electrical_angle_u16);
  duty = motor_foc_svpwm_q15(voltage);
  compare.phase_a = motor_open_loop_duty_to_compare(duty.phase_a);
  compare.phase_b = motor_open_loop_duty_to_compare(duty.phase_b);
  compare.phase_c = motor_open_loop_duty_to_compare(duty.phase_c);
  motor_pwm_port_compare_set(&compare);

  motor_open_loop_status.electrical_angle_u16 = electrical_angle_u16;
  motor_open_loop_status.duty_a_q15 = duty.phase_a;
  motor_open_loop_status.duty_b_q15 = duty.phase_b;
  motor_open_loop_status.duty_c_q15 = duty.phase_c;
}

void motor_open_loop_init(void)
{
  motor_open_loop_status.state = MOTOR_OPEN_LOOP_STOPPED;
  motor_open_loop_status.electrical_angle_u16 = 0U;
  motor_open_loop_status.electrical_frequency_millihz = 0U;
  motor_open_loop_status.duty_a_q15 = 0U;
  motor_open_loop_status.duty_b_q15 = 0U;
  motor_open_loop_status.duty_c_q15 = 0U;
  motor_open_loop_phase_accumulator = 0U;
  motor_open_loop_state_ticks = 0U;
}

bool motor_open_loop_start(void)
{
  motor_current_sample_state_t current_state;

  if ((!motor_current_sample_state_read(&current_state)) ||
      current_state.overcurrent_fault ||
      motor_pwm_port_output_is_enabled())
  {
    return false;
  }

  motor_open_loop_phase_accumulator = 0U;
  motor_open_loop_state_ticks = 0U;
  motor_open_loop_status.electrical_frequency_millihz = 0U;
  motor_open_loop_voltage_apply(MOTOR_OPEN_LOOP_ALIGN_VOLTAGE_Q15, 0, 0U);

  if (!motor_pwm_port_output_enable())
  {
    motor_open_loop_status.state = MOTOR_OPEN_LOOP_FAULT;
    motor_pwm_port_emergency_stop();
    return false;
  }

  motor_open_loop_status.state = MOTOR_OPEN_LOOP_ALIGNING;
  return true;
}

void motor_open_loop_stop(void)
{
  motor_pwm_port_output_disable();
  motor_open_loop_status.state = MOTOR_OPEN_LOOP_STOPPED;
  motor_open_loop_status.electrical_frequency_millihz = 0U;
}

void motor_open_loop_fast_process(void)
{
  uint32_t frequency_millihz;
  uint32_t phase_step;

  if ((motor_open_loop_status.state != MOTOR_OPEN_LOOP_ALIGNING) &&
      (motor_open_loop_status.state != MOTOR_OPEN_LOOP_RUNNING))
  {
    return;
  }

  if (!motor_pwm_port_output_is_enabled())
  {
    motor_open_loop_status.state = MOTOR_OPEN_LOOP_FAULT;
    return;
  }

  if (motor_open_loop_status.state == MOTOR_OPEN_LOOP_ALIGNING)
  {
    motor_open_loop_state_ticks++;
    if (motor_open_loop_state_ticks < MOTOR_OPEN_LOOP_ALIGN_TICKS)
    {
      return;
    }
    motor_open_loop_state_ticks = 0U;
    motor_open_loop_status.state = MOTOR_OPEN_LOOP_RUNNING;
  }

  if (motor_open_loop_state_ticks < MOTOR_OPEN_LOOP_RAMP_TICKS)
  {
    frequency_millihz = MOTOR_OPEN_LOOP_START_FREQUENCY_MILLIHZ +
      (uint32_t)(((uint64_t)(MOTOR_OPEN_LOOP_TARGET_FREQUENCY_MILLIHZ -
                            MOTOR_OPEN_LOOP_START_FREQUENCY_MILLIHZ) *
                  motor_open_loop_state_ticks) / MOTOR_OPEN_LOOP_RAMP_TICKS);
    motor_open_loop_state_ticks++;
  }
  else
  {
    frequency_millihz = MOTOR_OPEN_LOOP_TARGET_FREQUENCY_MILLIHZ;
  }

  phase_step = (uint32_t)(((uint64_t)frequency_millihz << 32) /
                          ((uint64_t)MOTOR_PWM_FREQUENCY_HZ * 1000U));
  motor_open_loop_phase_accumulator += phase_step;
  motor_open_loop_status.electrical_frequency_millihz = frequency_millihz;
  motor_open_loop_voltage_apply(0, MOTOR_OPEN_LOOP_RUN_VOLTAGE_Q15,
                                (uint16_t)(motor_open_loop_phase_accumulator >> 16));
}

bool motor_open_loop_status_read(motor_open_loop_status_t *status)
{
  uint32_t primask;

  if (status == NULL)
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *status = motor_open_loop_status;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return true;
}
