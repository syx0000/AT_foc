#include <stddef.h>
#include "at32f45x_conf.h"
#include "motor_pwm_port.h"
#include "motor_board_config.h"

static bool motor_pwm_gate_driver_enabled;
static bool motor_pwm_command_valid;
static bool motor_pwm_output_enabled;

/**
 * @brief 将单相PWM原始比较值限制在定时器有效范围内。
 * @param compare 请求的比较值，单位为定时器计数。
 * @return 输入未越界时返回原值；超过上限时返回MOTOR_PWM_COMPARE_MAX。
 */
static uint16_t motor_pwm_compare_clamp(uint16_t compare)
{
  if (compare > MOTOR_PWM_COMPARE_MAX)
  {
    return (uint16_t)MOTOR_PWM_COMPARE_MAX;
  }
  return compare;
}

void motor_pwm_port_init(void)
{
  gpio_bits_reset(MOTOR_GATE_ENABLE_PORT, MOTOR_GATE_ENABLE_PIN);
  tmr_output_enable(MOTOR_PWM_TIMER, FALSE);
  tmr_channel_value_set(MOTOR_PWM_TIMER, TMR_SELECT_CHANNEL_1, 0U);
  tmr_channel_value_set(MOTOR_PWM_TIMER, TMR_SELECT_CHANNEL_2, 0U);
  tmr_channel_value_set(MOTOR_PWM_TIMER, TMR_SELECT_CHANNEL_3, 0U);
  motor_pwm_gate_driver_enabled = false;
  motor_pwm_command_valid = false;
  motor_pwm_output_enabled = false;
}

void motor_pwm_port_gate_driver_set(bool enable)
{
  if (enable)
  {
    gpio_bits_set(MOTOR_GATE_ENABLE_PORT, MOTOR_GATE_ENABLE_PIN);
    motor_pwm_gate_driver_enabled = true;
  }
  else
  {
    gpio_bits_reset(MOTOR_GATE_ENABLE_PORT, MOTOR_GATE_ENABLE_PIN);
    tmr_output_enable(MOTOR_PWM_TIMER, FALSE);
    motor_pwm_gate_driver_enabled = false;
    motor_pwm_output_enabled = false;
  }
}

bool motor_pwm_port_fault_clear(void)
{
  if (gpio_input_data_bit_read(MOTOR_PWM_BREAK_PORT,
                               MOTOR_PWM_BREAK_PIN) == RESET)
  {
    return false;
  }

  tmr_flag_clear(MOTOR_PWM_TIMER, TMR_BRK_FLAG);
  __DSB();

  return (tmr_flag_get(MOTOR_PWM_TIMER, TMR_BRK_FLAG) == RESET);
}

bool motor_pwm_port_output_enable(void)
{
  if ((!motor_pwm_gate_driver_enabled) ||
      (!motor_pwm_command_valid) ||
      (!motor_pwm_port_fault_clear()))
  {
    return false;
  }

  MOTOR_PWM_TIMER->brk_bit.brken = TRUE;
  __DSB();

  if (!motor_pwm_port_fault_clear())
  {
    return false;
  }

  tmr_output_enable(MOTOR_PWM_TIMER, TRUE);
  motor_pwm_output_enabled = (MOTOR_PWM_TIMER->brk_bit.oen != 0U);
  return motor_pwm_output_enabled;
}

void motor_pwm_port_output_disable(void)
{
  tmr_output_enable(MOTOR_PWM_TIMER, FALSE);
  motor_pwm_output_enabled = false;
}

void motor_pwm_port_emergency_stop(void)
{
  gpio_bits_reset(MOTOR_GATE_ENABLE_PORT, MOTOR_GATE_ENABLE_PIN);
  tmr_output_enable(MOTOR_PWM_TIMER, FALSE);
  tmr_channel_value_set(MOTOR_PWM_TIMER, TMR_SELECT_CHANNEL_1, 0U);
  tmr_channel_value_set(MOTOR_PWM_TIMER, TMR_SELECT_CHANNEL_2, 0U);
  tmr_channel_value_set(MOTOR_PWM_TIMER, TMR_SELECT_CHANNEL_3, 0U);
  motor_pwm_gate_driver_enabled = false;
  motor_pwm_command_valid = false;
  motor_pwm_output_enabled = false;
}

void motor_pwm_port_compare_set(const motor_pwm_compare_t *compare)
{
  if (compare == NULL)
  {
    return;
  }
  tmr_channel_value_set(MOTOR_PWM_TIMER, TMR_SELECT_CHANNEL_1,
                        motor_pwm_compare_clamp(compare->phase_a));
  tmr_channel_value_set(MOTOR_PWM_TIMER, TMR_SELECT_CHANNEL_2,
                        motor_pwm_compare_clamp(compare->phase_b));
  tmr_channel_value_set(MOTOR_PWM_TIMER, TMR_SELECT_CHANNEL_3,
                        motor_pwm_compare_clamp(compare->phase_c));
  motor_pwm_command_valid = true;
}

bool motor_pwm_port_output_is_enabled(void)
{
  return motor_pwm_output_enabled;
}
