/**
 * @file    motor_rotor_feedback.h
 * @brief   通用转子反馈接口 公共数据结构和接口声明
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#ifndef MOTOR_ROTOR_FEEDBACK_H
#define MOTOR_ROTOR_FEEDBACK_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  MOTOR_ROTOR_FEEDBACK_SOURCE_HALL = 0,
  MOTOR_ROTOR_FEEDBACK_SOURCE_ABSOLUTE_ENCODER,
  MOTOR_ROTOR_FEEDBACK_SOURCE_CURRENT_STARTUP
} motor_rotor_feedback_source_t;

typedef struct
{
  motor_rotor_feedback_source_t source; /**< 当前反馈来源。 */
  uint16_t electrical_angle_u16;        /**< 转子电角度，U16一周制。 */
  uint16_t mechanical_angle_u16;        /**< 单圈机械角度，U16一周制。 */
  int32_t mechanical_speed_millirpm;    /**< 有符号物理机械转速，单位0.001 rpm。 */
  uint32_t electrical_frequency_millihz;/**< 电频率绝对值，单位mHz。 */
  uint32_t timestamp_cycles;            /**< 最近一次反馈事件的周期时间戳。 */
  uint32_t update_count;                /**< 新速度测量累计次数。 */
  uint8_t source_state;                 /**< 反馈源原始状态；Hall为1..6。 */
  int8_t direction;                     /**< 物理方向：1正向、-1反向、0未知。 */
  bool mechanical_angle_valid;          /**< 单圈机械角度是否有效。 */
  bool valid;                           /**< FOC电角度、方向和速度是否有效。 */
} motor_rotor_feedback_t;

/**
 * @brief 初始化统一转子反馈接口。
 * @param 无。
 * @return 无。
 * @details 当前版本默认使用Hall适配器，不操作GPIO、PWM或反馈源硬件。
 */
void motor_rotor_feedback_init(void);

/**
 * @brief 1 kHz 累加转子电角度得到连续单圈机械角。
 * @param 无。
 * @return 无。
 * @details 每毫秒采样一次估算电角度，按极对数换算并累加到 U16 机械角；反馈失效或
 *          override 生效时冻结追踪起点，恢复后再重新起算，mechanical_angle_u16
 *          在失效期间保持最后有效值。
 * @note 仅供 SysTick 1 kHz 中断调用，且必须在 motor_hall_angle_estimator 更新之后。
 */
void motor_rotor_feedback_process_1khz(void);

/**
 * @brief 读取当前统一转子反馈快照。
 * @param feedback 输出电角度、机械速度、方向、时效和反馈源，不允许为空。
 * @return 参数有效且底层反馈快照可读取时返回true；不代表feedback->valid一定为true。
 * @details 当前Hall适配器保持原Hall角度和测速行为；单圈机械角由
 *          motor_rotor_feedback_process_1khz 通过累加电角度增量得到，因此
 *          mechanical_angle_valid 反映累加器是否已建立追踪起点；上电或反馈失效
 *          期间可能为 false，但一旦建立便保持连续。后续 SPI 编码器实现应保持
 *          相同输出语义。
 */
bool motor_rotor_feedback_read(motor_rotor_feedback_t *feedback);

void motor_rotor_feedback_override_set(uint16_t electrical_angle_u16,
                                       int32_t electrical_frequency_millihz,
                                       int8_t direction);
void motor_rotor_feedback_override_clear(void);
bool motor_rotor_feedback_override_active(void);

#endif /* MOTOR_ROTOR_FEEDBACK_H */
