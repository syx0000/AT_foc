#ifndef MOTOR_HALL_OFFSET_IDENTIFICATION_H
#define MOTOR_HALL_OFFSET_IDENTIFICATION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  MOTOR_HALL_OFFSET_IDENT_OK = 0,
  MOTOR_HALL_OFFSET_IDENT_INVALID_ARGUMENT,
  MOTOR_HALL_OFFSET_IDENT_QUADRATURE_VOLTAGE_TOO_LOW,
  MOTOR_HALL_OFFSET_IDENT_CORRECTION_EXCEEDED
} motor_hall_offset_identification_status_t;

typedef struct
{
  int32_t direct_voltage_mv;
  int32_t quadrature_voltage_mv;
  uint16_t current_offset_u16;
  uint16_t maximum_correction_u16;
  uint16_t minimum_quadrature_voltage_mv;
} motor_hall_offset_identification_input_t;

typedef struct
{
  motor_hall_offset_identification_status_t status;
  int16_t correction_u16;
  uint16_t candidate_offset_u16;
  uint16_t correction_degrees_x10;
} motor_hall_offset_identification_result_t;

/**
 * @brief 由稳定正Iq闭环下的平均Vd/Vq计算Hall转子角度补偿。
 * @param input 平均电压、当前补偿及安全门槛。
 * @param result 输出带符号修正量和自然回绕后的候选补偿。
 * @return 输入有效且修正量未超过限制时返回true。
 * @details 修正角为atan2(-Vd,Vq)。要求Vq为正且具有足够幅值；算法仅使用
 *          定点三角函数和整数二分，不依赖硬件及浮点数学库。
 */
bool motor_hall_offset_identify(
  const motor_hall_offset_identification_input_t *input,
  motor_hall_offset_identification_result_t *result);

#endif /* MOTOR_HALL_OFFSET_IDENTIFICATION_H */
