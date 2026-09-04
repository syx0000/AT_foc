#ifndef MOTOR_PARAMETER_STORAGE_H
#define MOTOR_PARAMETER_STORAGE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  MOTOR_PARAMETER_STORAGE_DEFAULTS = 0,
  MOTOR_PARAMETER_STORAGE_SLOT_A,
  MOTOR_PARAMETER_STORAGE_SLOT_B,
  MOTOR_PARAMETER_STORAGE_ERROR
} motor_parameter_storage_source_t;

typedef struct
{
  motor_parameter_storage_source_t source;
  uint32_t sequence;
  bool slot_a_valid;
  bool slot_b_valid;
} motor_parameter_storage_status_t;

/**
 * @brief 启动时扫描Flash双槽并加载最新有效参数。
 * @param 无。
 * @return 找到并加载有效记录返回true；空Flash或异常时保留编译默认值并返回false。
 */
bool motor_parameter_storage_init(void);

/**
 * @brief 将当前活动参数掉电安全地保存到非活动Flash槽。
 * @param 无。
 * @return 电机READY、PWM关闭且擦写及回读校验全部成功时返回true。
 */
bool motor_parameter_storage_save(void);

/**
 * @brief 从Flash重新加载最新有效参数到活动区和候选区。
 * @param 无。
 * @return 安全状态且存在有效记录时返回true，否则保持RAM参数不变并返回false。
 */
bool motor_parameter_storage_load(void);

/**
 * @brief 将RAM活动参数和候选参数恢复为编译期默认值。
 * @param 无。
 * @return 电机READY且PWM关闭时返回true；本操作不会自动擦除或写入Flash。
 */
bool motor_parameter_storage_defaults(void);

/**
 * @brief 读取最近一次扫描或保存后的双槽状态。
 * @param status 输出来源、序号以及A/B槽有效性，不允许为空。
 * @return 参数有效时返回true。
 */
bool motor_parameter_storage_status_read(motor_parameter_storage_status_t *status);

#endif
