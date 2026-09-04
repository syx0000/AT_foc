#include <stddef.h>
#include <string.h>
#include "at32f45x.h"
#include "motor_control.h"
#include "motor_control_config.h"
#include "motor_parameter.h"
#include "motor_pwm_port.h"

#define MOTOR_PARAMETER_POLE_PAIRS_MAX             64U
#define MOTOR_PARAMETER_RESISTANCE_MOHM_MAX     100000U
#define MOTOR_PARAMETER_INDUCTANCE_UH_MAX      1000000U
#define MOTOR_PARAMETER_CURRENT_BANDWIDTH_HZ_MAX   2000U

static volatile motor_parameter_t motor_parameter_active;
static motor_parameter_t motor_parameter_candidate;

/**
 * @brief 生成与当前编译配置完全一致的默认电机参数。
 * @param parameter 输出默认参数，不允许为空。
 * @return 无。
 */
static void motor_parameter_default_build(motor_parameter_t *parameter)
{
  static const uint8_t hall_positive_next[8] =
  {
    0U,
    MOTOR_HALL_POSITIVE_NEXT_STATE_1,
    MOTOR_HALL_POSITIVE_NEXT_STATE_2,
    MOTOR_HALL_POSITIVE_NEXT_STATE_3,
    MOTOR_HALL_POSITIVE_NEXT_STATE_4,
    MOTOR_HALL_POSITIVE_NEXT_STATE_5,
    MOTOR_HALL_POSITIVE_NEXT_STATE_6,
    0U
  };
  static const uint16_t hall_entry_angle_u16[8] =
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

  parameter->pole_pairs = MOTOR_POLE_PAIRS;
  parameter->direction_inverted = MOTOR_DIRECTION_INVERTED;
  parameter->reserved = 0U;
  parameter->phase_resistance_mohm = MOTOR_PHASE_RESISTANCE_MOHM;
  parameter->direct_inductance_uh = MOTOR_DIRECT_INDUCTANCE_UH;
  parameter->quadrature_inductance_uh = MOTOR_QUADRATURE_INDUCTANCE_UH;
  parameter->current_loop_bandwidth_hz = MOTOR_CURRENT_LOOP_BANDWIDTH_HZ;
  parameter->current_d_kp_q15 = MOTOR_CURRENT_PI_D_KP_Q15;
  parameter->current_q_kp_q15 = MOTOR_CURRENT_PI_Q_KP_Q15;
  parameter->current_ki_q15 = MOTOR_CURRENT_PI_KI_Q15;
  parameter->speed_kp_q20 = MOTOR_SPEED_PI_KP_Q20;
  parameter->speed_ki_q20 = MOTOR_SPEED_PI_KI_Q20;
  parameter->hall_rotor_offset_u16 = MOTOR_HALL_ROTOR_ANGLE_OFFSET_U16;
  (void)memcpy(parameter->hall_positive_next, hall_positive_next,
               sizeof(hall_positive_next));
  (void)memcpy(parameter->hall_entry_angle_u16, hall_entry_angle_u16,
               sizeof(hall_entry_angle_u16));
}

/**
 * @brief 校验Hall正向跳转表是否由状态1..6构成唯一闭环。
 * @param positive_next 待校验的8元素正向后继表，不允许为空。
 * @return 六个状态各有唯一前驱且从状态1经过6步回到状态1时返回true。
 */
static bool motor_parameter_hall_sequence_validate(
  const uint8_t positive_next[MOTOR_PARAMETER_HALL_STATE_COUNT])
{
  bool visited[8] = {false, false, false, false,
                     false, false, false, false};
  uint8_t predecessor_count[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
  uint8_t state;
  uint8_t index;
  uint8_t changed_bits;

  if ((positive_next[0] != 0U) || (positive_next[7] != 0U))
  {
    return false;
  }
  for (state = 1U; state <= 6U; state++)
  {
    if ((positive_next[state] < 1U) || (positive_next[state] > 6U) ||
        (positive_next[state] == state))
    {
      return false;
    }
    changed_bits = (uint8_t)(state ^ positive_next[state]);
    if ((changed_bits != 1U) && (changed_bits != 2U) &&
        (changed_bits != 4U))
    {
      return false;
    }
    predecessor_count[positive_next[state]]++;
  }
  for (state = 1U; state <= 6U; state++)
  {
    if (predecessor_count[state] != 1U)
    {
      return false;
    }
  }

  state = 1U;
  for (index = 0U; index < 6U; index++)
  {
    if (visited[state])
    {
      return false;
    }
    visited[state] = true;
    state = positive_next[state];
  }
  return (state == 1U);
}

void motor_parameter_init(void)
{
  motor_parameter_t defaults;

  motor_parameter_default_build(&defaults);
  motor_parameter_active = defaults;
  motor_parameter_candidate = defaults;
}

bool motor_parameter_active_read(motor_parameter_t *parameter)
{
  uint32_t primask;

  if (parameter == NULL)
  {
    return false;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  *parameter = motor_parameter_active;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return true;
}

bool motor_parameter_direction_inverted_get(void)
{
  return (motor_parameter_active.direction_inverted != 0U);
}

uint8_t motor_parameter_pole_pairs_get(void)
{
  return motor_parameter_active.pole_pairs;
}

bool motor_parameter_candidate_read(motor_parameter_t *parameter)
{
  if (parameter == NULL)
  {
    return false;
  }
  *parameter = motor_parameter_candidate;
  return true;
}

bool motor_parameter_validate(const motor_parameter_t *parameter)
{
  return (parameter != NULL) &&
    (parameter->pole_pairs > 0U) &&
    (parameter->pole_pairs <= MOTOR_PARAMETER_POLE_PAIRS_MAX) &&
    (parameter->direction_inverted <= 1U) &&
    (parameter->reserved == 0U) &&
    (parameter->phase_resistance_mohm > 0U) &&
    (parameter->phase_resistance_mohm <=
     MOTOR_PARAMETER_RESISTANCE_MOHM_MAX) &&
    (parameter->direct_inductance_uh > 0U) &&
    (parameter->direct_inductance_uh <=
     MOTOR_PARAMETER_INDUCTANCE_UH_MAX) &&
    (parameter->quadrature_inductance_uh > 0U) &&
    (parameter->quadrature_inductance_uh <=
     MOTOR_PARAMETER_INDUCTANCE_UH_MAX) &&
    (parameter->current_loop_bandwidth_hz > 0U) &&
    (parameter->current_loop_bandwidth_hz <=
     MOTOR_PARAMETER_CURRENT_BANDWIDTH_HZ_MAX) &&
    (parameter->current_d_kp_q15 >= 0) &&
    (parameter->current_q_kp_q15 >= 0) &&
    (parameter->current_ki_q15 >= 0) &&
    (parameter->speed_kp_q20 >= 0) &&
    (parameter->speed_ki_q20 >= 0) &&
    (parameter->hall_positive_next[0] == 0U) &&
    (parameter->hall_positive_next[7] == 0U) &&
    (parameter->hall_entry_angle_u16[0] == 0U) &&
    (parameter->hall_entry_angle_u16[7] == 0U) &&
    motor_parameter_hall_sequence_validate(
      parameter->hall_positive_next);
}

bool motor_parameter_candidate_set(const motor_parameter_t *parameter)
{
  if (!motor_parameter_validate(parameter))
  {
    return false;
  }
  motor_parameter_candidate = *parameter;
  return true;
}

bool motor_parameter_candidate_field_set(motor_parameter_field_t field,
                                         int32_t value)
{
  motor_parameter_t updated = motor_parameter_candidate;

  switch (field)
  {
    case MOTOR_PARAMETER_FIELD_POLE_PAIRS:
      if ((value < 0) || (value > 255)) return false;
      updated.pole_pairs = (uint8_t)value;
      break;
    case MOTOR_PARAMETER_FIELD_DIRECTION_INVERTED:
      if ((value != 0) && (value != 1)) return false;
      updated.direction_inverted = (uint8_t)value;
      break;
    case MOTOR_PARAMETER_FIELD_PHASE_RESISTANCE_MOHM:
      if (value < 0) return false;
      updated.phase_resistance_mohm = (uint32_t)value;
      break;
    case MOTOR_PARAMETER_FIELD_DIRECT_INDUCTANCE_UH:
      if (value < 0) return false;
      updated.direct_inductance_uh = (uint32_t)value;
      break;
    case MOTOR_PARAMETER_FIELD_QUADRATURE_INDUCTANCE_UH:
      if (value < 0) return false;
      updated.quadrature_inductance_uh = (uint32_t)value;
      break;
    case MOTOR_PARAMETER_FIELD_CURRENT_LOOP_BANDWIDTH_HZ:
      if (value < 0) return false;
      updated.current_loop_bandwidth_hz = (uint32_t)value;
      break;
    case MOTOR_PARAMETER_FIELD_CURRENT_D_KP_Q15:
      updated.current_d_kp_q15 = value;
      break;
    case MOTOR_PARAMETER_FIELD_CURRENT_Q_KP_Q15:
      updated.current_q_kp_q15 = value;
      break;
    case MOTOR_PARAMETER_FIELD_CURRENT_KI_Q15:
      updated.current_ki_q15 = value;
      break;
    case MOTOR_PARAMETER_FIELD_SPEED_KP_Q20:
      updated.speed_kp_q20 = value;
      break;
    case MOTOR_PARAMETER_FIELD_SPEED_KI_Q20:
      updated.speed_ki_q20 = value;
      break;
    case MOTOR_PARAMETER_FIELD_HALL_ROTOR_OFFSET_U16:
      if ((value < 0) || (value > 65535)) return false;
      updated.hall_rotor_offset_u16 = (uint16_t)value;
      break;
    default:
      return false;
  }

  return motor_parameter_candidate_set(&updated);
}

bool motor_parameter_field_value_read(const motor_parameter_t *parameter,
                                      motor_parameter_field_t field,
                                      int32_t *value)
{
  if ((parameter == NULL) || (value == NULL))
  {
    return false;
  }

  switch (field)
  {
    case MOTOR_PARAMETER_FIELD_POLE_PAIRS:
      *value = parameter->pole_pairs;
      break;
    case MOTOR_PARAMETER_FIELD_DIRECTION_INVERTED:
      *value = parameter->direction_inverted;
      break;
    case MOTOR_PARAMETER_FIELD_PHASE_RESISTANCE_MOHM:
      *value = (int32_t)parameter->phase_resistance_mohm;
      break;
    case MOTOR_PARAMETER_FIELD_DIRECT_INDUCTANCE_UH:
      *value = (int32_t)parameter->direct_inductance_uh;
      break;
    case MOTOR_PARAMETER_FIELD_QUADRATURE_INDUCTANCE_UH:
      *value = (int32_t)parameter->quadrature_inductance_uh;
      break;
    case MOTOR_PARAMETER_FIELD_CURRENT_LOOP_BANDWIDTH_HZ:
      *value = (int32_t)parameter->current_loop_bandwidth_hz;
      break;
    case MOTOR_PARAMETER_FIELD_CURRENT_D_KP_Q15:
      *value = parameter->current_d_kp_q15;
      break;
    case MOTOR_PARAMETER_FIELD_CURRENT_Q_KP_Q15:
      *value = parameter->current_q_kp_q15;
      break;
    case MOTOR_PARAMETER_FIELD_CURRENT_KI_Q15:
      *value = parameter->current_ki_q15;
      break;
    case MOTOR_PARAMETER_FIELD_SPEED_KP_Q20:
      *value = parameter->speed_kp_q20;
      break;
    case MOTOR_PARAMETER_FIELD_SPEED_KI_Q20:
      *value = parameter->speed_ki_q20;
      break;
    case MOTOR_PARAMETER_FIELD_HALL_ROTOR_OFFSET_U16:
      *value = parameter->hall_rotor_offset_u16;
      break;
    default:
      return false;
  }
  return true;
}

bool motor_parameter_candidate_accept(void)
{
  motor_control_status_t control;
  uint32_t primask;

  if ((!motor_parameter_validate(&motor_parameter_candidate)) ||
      (!motor_control_status_read(&control)) ||
      (control.state != MOTOR_CONTROL_STATE_READY) ||
      motor_pwm_port_output_is_enabled())
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  motor_parameter_active = motor_parameter_candidate;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return true;
}

void motor_parameter_candidate_discard(void)
{
  motor_parameter_t active;

  (void)motor_parameter_active_read(&active);
  motor_parameter_candidate = active;
}

bool motor_parameter_diff_read(motor_parameter_diff_t *diff)
{
  motor_parameter_t active;
  int32_t active_value;
  int32_t candidate_value;
  uint32_t field;

  if (diff == NULL)
  {
    return false;
  }
  (void)motor_parameter_active_read(&active);
  diff->scalar_fields = 0U;
  for (field = 0U; field < (uint32_t)MOTOR_PARAMETER_FIELD_COUNT; field++)
  {
    (void)motor_parameter_field_value_read(
      &active, (motor_parameter_field_t)field, &active_value);
    (void)motor_parameter_field_value_read(
      &motor_parameter_candidate, (motor_parameter_field_t)field,
      &candidate_value);
    if (active_value != candidate_value)
    {
      diff->scalar_fields |= (1UL << field);
    }
  }
  diff->hall_sequence_changed =
    (memcmp(active.hall_positive_next,
            motor_parameter_candidate.hall_positive_next,
            sizeof(active.hall_positive_next)) != 0);
  diff->hall_angle_changed =
    (memcmp(active.hall_entry_angle_u16,
            motor_parameter_candidate.hall_entry_angle_u16,
            sizeof(active.hall_entry_angle_u16)) != 0);
  diff->any_changed = (diff->scalar_fields != 0U) ||
    diff->hall_sequence_changed || diff->hall_angle_changed;
  return true;
}
