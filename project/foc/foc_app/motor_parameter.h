#ifndef MOTOR_PARAMETER_H
#define MOTOR_PARAMETER_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_PARAMETER_HALL_STATE_COUNT 8U

typedef enum
{
  MOTOR_PARAMETER_FIELD_POLE_PAIRS = 0,
  MOTOR_PARAMETER_FIELD_DIRECTION_INVERTED,
  MOTOR_PARAMETER_FIELD_PHASE_RESISTANCE_MOHM,
  MOTOR_PARAMETER_FIELD_DIRECT_INDUCTANCE_UH,
  MOTOR_PARAMETER_FIELD_QUADRATURE_INDUCTANCE_UH,
  MOTOR_PARAMETER_FIELD_CURRENT_LOOP_BANDWIDTH_HZ,
  MOTOR_PARAMETER_FIELD_CURRENT_D_KP_Q15,
  MOTOR_PARAMETER_FIELD_CURRENT_Q_KP_Q15,
  MOTOR_PARAMETER_FIELD_CURRENT_KI_Q15,
  MOTOR_PARAMETER_FIELD_SPEED_KP_Q20,
  MOTOR_PARAMETER_FIELD_SPEED_KI_Q20,
  MOTOR_PARAMETER_FIELD_HALL_ROTOR_OFFSET_U16,
  MOTOR_PARAMETER_FIELD_COUNT
} motor_parameter_field_t;

typedef struct
{
  uint8_t pole_pairs;
  uint8_t direction_inverted;
  uint16_t reserved;
  uint32_t phase_resistance_mohm;
  uint32_t direct_inductance_uh;
  uint32_t quadrature_inductance_uh;
  uint32_t current_loop_bandwidth_hz;
  int32_t current_d_kp_q15;
  int32_t current_q_kp_q15;
  int32_t current_ki_q15;
  int32_t speed_kp_q20;
  int32_t speed_ki_q20;
  uint16_t hall_rotor_offset_u16;
  uint8_t hall_positive_next[MOTOR_PARAMETER_HALL_STATE_COUNT];
  uint16_t hall_entry_angle_u16[MOTOR_PARAMETER_HALL_STATE_COUNT];
} motor_parameter_t;

typedef struct
{
  uint32_t scalar_fields;
  bool hall_sequence_changed;
  bool hall_angle_changed;
  bool any_changed;
} motor_parameter_diff_t;

/**
 * @brief 使用motor_control_config.h默认值初始化活动参数和候选参数。
 * @param 无。
 * @return 无。
 * @details 初始化后活动参数与候选参数完全一致，不操作PWM、Flash或任何外设。
 */
void motor_parameter_init(void);

/**
 * @brief 原子读取当前活动电机参数。
 * @param parameter 输出完整活动参数，不允许为空。
 * @return 参数有效时返回true；传入NULL时返回false。
 */
bool motor_parameter_active_read(motor_parameter_t *parameter);

/**
 * @brief 原子读取当前活动逻辑方向配置。
 * @param 无。
 * @return false表示逻辑方向与物理正向一致，true表示逻辑方向反转。
 */
bool motor_parameter_direction_inverted_get(void);

/**
 * @brief 原子读取当前活动电机极对数。
 * @param 无。
 * @return 已校验的电机极对数，范围1..64。
 */
uint8_t motor_parameter_pole_pairs_get(void);

/**
 * @brief 读取尚未生效的候选电机参数。
 * @param parameter 输出完整候选参数，不允许为空。
 * @return 参数有效时返回true；传入NULL时返回false。
 */
bool motor_parameter_candidate_read(motor_parameter_t *parameter);

/**
 * @brief 用完整参数结构替换候选参数。
 * @param parameter 待校验并写入的完整候选参数，不允许为空。
 * @return 所有字段及Hall关系合法时返回true，否则保持原候选参数并返回false。
 */
bool motor_parameter_candidate_set(const motor_parameter_t *parameter);

/**
 * @brief 修改候选参数中的一个标量字段。
 * @param field 待修改字段枚举。
 * @param value 新值；无符号字段要求非负且不得超出字段范围。
 * @return 修改后的完整候选参数合法时返回true，否则回退并返回false。
 */
bool motor_parameter_candidate_field_set(motor_parameter_field_t field,
                                         int32_t value);

/**
 * @brief 从指定参数结构读取一个标量字段。
 * @param parameter 待读取参数，不允许为空。
 * @param field 待读取字段枚举。
 * @param value 输出统一的32位有符号显示值，不允许为空。
 * @return 字段和参数有效时返回true，否则返回false。
 */
bool motor_parameter_field_value_read(const motor_parameter_t *parameter,
                                      motor_parameter_field_t field,
                                      int32_t *value);

/**
 * @brief 将完整候选参数应用为活动参数。
 * @param 无。
 * @return 候选参数合法且统一电机状态为READY、PWM关闭时返回true，否则返回false。
 * @details 本阶段只更新RAM活动参数；现有控制模块尚未切换为读取该结构，也不会写Flash。
 */
bool motor_parameter_candidate_accept(void);

/**
 * @brief 丢弃所有候选修改并恢复为当前活动参数。
 * @param 无。
 * @return 无。
 */
void motor_parameter_candidate_discard(void);

/**
 * @brief 比较活动参数与候选参数。
 * @param diff 输出标量字段位图、Hall表变化及总变化标志，不允许为空。
 * @return 参数有效时返回true；传入NULL时返回false。
 */
bool motor_parameter_diff_read(motor_parameter_diff_t *diff);

/**
 * @brief 校验完整电机参数的范围和Hall六步闭环关系。
 * @param parameter 待校验参数，不允许为空。
 * @return 参数完整合法时返回true，否则返回false。
 */
bool motor_parameter_validate(const motor_parameter_t *parameter);

#endif /* MOTOR_PARAMETER_H */
