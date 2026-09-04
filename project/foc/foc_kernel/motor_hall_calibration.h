#ifndef MOTOR_HALL_CALIBRATION_H
#define MOTOR_HALL_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_HALL_CALIBRATION_STATE_COUNT 8U

typedef enum
{
  MOTOR_HALL_CALIBRATION_OK = 0,
  MOTOR_HALL_CALIBRATION_INVALID_ARGUMENT,
  MOTOR_HALL_CALIBRATION_INVALID_STATE,
  MOTOR_HALL_CALIBRATION_DUPLICATE_STATE,
  MOTOR_HALL_CALIBRATION_NON_ADJACENT_STATE,
  MOTOR_HALL_CALIBRATION_MULTIPLE_SUCCESSOR,
  MOTOR_HALL_CALIBRATION_TIMESTAMP_INVALID,
  MOTOR_HALL_CALIBRATION_INCOMPLETE_SEQUENCE,
  MOTOR_HALL_CALIBRATION_SECTOR_ANGLE_INVALID,
  MOTOR_HALL_CALIBRATION_EDGE_DEVIATION_EXCEEDED,
  MOTOR_HALL_CALIBRATION_REVERSE_MISMATCH,
  MOTOR_HALL_CALIBRATION_SEQUENCE_ERROR
} motor_hall_calibration_error_t;

typedef enum
{
  MOTOR_HALL_CALIBRATION_IDLE = 0,
  MOTOR_HALL_CALIBRATION_FORWARD_SCAN,
  MOTOR_HALL_CALIBRATION_FORWARD_READY,
  MOTOR_HALL_CALIBRATION_REVERSE_SCAN,
  MOTOR_HALL_CALIBRATION_COMPLETE,
  MOTOR_HALL_CALIBRATION_FAULT
} motor_hall_calibration_state_t;

typedef struct
{
  uint32_t minimum_samples_per_state;
  uint16_t minimum_sector_width_u16;
  uint16_t maximum_sector_width_u16;
  uint16_t maximum_edge_deviation_u16;
  uint16_t maximum_forward_reverse_error_u16;
  uint32_t maximum_edge_interval_cycles; /**< 0表示不检查相邻事件间隔。 */
} motor_hall_calibration_config_t;

typedef struct
{
  uint16_t reference_angle_u16;
  int64_t delta_sum;
  int16_t minimum_delta;
  int16_t maximum_delta;
  uint32_t count;
} motor_hall_calibration_angle_accumulator_t;

typedef struct
{
  motor_hall_calibration_config_t config;
  motor_hall_calibration_angle_accumulator_t forward_angle[8];
  motor_hall_calibration_angle_accumulator_t reverse_angle[8];
  uint8_t positive_next[8];
  uint32_t last_timestamp_cycles;
  motor_hall_calibration_state_t state;
  motor_hall_calibration_error_t error;
  bool timestamp_valid;
} motor_hall_calibration_t;

typedef struct
{
  uint8_t positive_next[8];
  uint16_t entry_angle_u16[8];
  uint16_t sector_width_u16[8];
  uint32_t forward_samples[8];
  uint32_t reverse_samples[8];
  uint16_t maximum_edge_deviation_u16;
  uint16_t maximum_forward_reverse_error_u16;
  bool reverse_verified;
  bool valid;
} motor_hall_calibration_result_t;

/**
 * @brief 初始化一份独立Hall标定算法上下文。
 * @param calibration 输出并持有标定过程状态，不允许为空。
 * @param config 样本数、扇区宽度、重复性和事件间隔判据，不允许为空。
 * @return 配置合法时返回OK，否则返回INVALID_ARGUMENT。
 * @details 只清理RAM上下文，不读取GPIO、时间基准、PWM或运行时参数。
 */
motor_hall_calibration_error_t motor_hall_calibration_init(
  motor_hall_calibration_t *calibration,
  const motor_hall_calibration_config_t *config);

/**
 * @brief 开始正向或反向Hall边沿扫描。
 * @param calibration 已初始化上下文，不允许为空。
 * @param direction 扫描方向；1开始正向，-1在正向完成后开始反向。
 * @return 状态与方向合法时返回OK，否则返回对应错误并保持原结果。
 */
motor_hall_calibration_error_t motor_hall_calibration_scan_begin(
  motor_hall_calibration_t *calibration, int8_t direction);

/**
 * @brief 向当前扫描加入一个Hall状态跳变及对应开环电角度。
 * @param calibration 正在扫描的上下文，不允许为空。
 * @param previous_state 边沿前Hall组合状态，范围1..6。
 * @param current_state 边沿后Hall组合状态，范围1..6。
 * @param open_loop_angle_u16 边沿时刻的开环电角度，U16一周制。
 * @param timestamp_cycles 边沿32位周期时间戳，允许自然回绕。
 * @return 样本被接受时返回OK；异常会锁存FAULT并返回具体错误。
 * @details 正向样本按current_state累计进入角，反向样本按previous_state累计同一物理边界。
 */
motor_hall_calibration_error_t motor_hall_calibration_edge_add(
  motor_hall_calibration_t *calibration, uint8_t previous_state,
  uint8_t current_state, uint16_t open_loop_angle_u16,
  uint32_t timestamp_cycles);

/**
 * @brief 结束当前扫描并校验已采集数据。
 * @param calibration 正在扫描的上下文，不允许为空。
 * @return 正向闭环或反向一致性通过时返回OK，否则锁存FAULT并返回原因。
 * @details 正向完成后进入FORWARD_READY；反向完成后进入COMPLETE。
 */
motor_hall_calibration_error_t motor_hall_calibration_scan_end(
  motor_hall_calibration_t *calibration);

/**
 * @brief 读取完整Hall标定结果。
 * @param calibration 已完成正反向扫描的上下文，不允许为空。
 * @param result 输出跳转表、边界角、扇区宽度、样本数及误差，不允许为空。
 * @return 仅在状态为COMPLETE且所有判据通过时返回true，否则输出valid=false。
 */
bool motor_hall_calibration_result_get(
  const motor_hall_calibration_t *calibration,
  motor_hall_calibration_result_t *result);

#endif /* MOTOR_HALL_CALIBRATION_H */
