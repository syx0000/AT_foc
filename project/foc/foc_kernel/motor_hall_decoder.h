#ifndef MOTOR_HALL_DECODER_H
#define MOTOR_HALL_DECODER_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_HALL_DECODER_STATE_COUNT 8U
#define MOTOR_HALL_DECODER_SPEED_WINDOW_SIZE 6U

typedef struct
{
  uint8_t state;              /**< Hall组合状态，bit0=HA、bit1=HB、bit2=HC。 */
  bool valid;                 /**< 状态为1..6时有效；0和7表示非法。 */
  uint32_t edge_count;        /**< 三路Hall任意边沿的累计捕获次数。 */
  uint32_t timestamp_cycles;  /**< 最近一次合法相邻跳变的周期时间戳。 */
  uint32_t positive_count;    /**< 按活动正向表变化的累计次数。 */
  uint32_t negative_count;    /**< 按活动正向表反向变化的累计次数。 */
  uint32_t invalid_transition_count; /**< 非法电平或非相邻跳变累计次数。 */
  uint32_t duplicate_count;   /**< 边沿事件发生但组合状态未变化的累计次数。 */
  uint32_t electrical_frequency_millihz; /**< 最近6段滑动平均电频率，单位mHz。 */
  int8_t direction;           /**< 物理方向：1为正向，-1为反向，0为未知。 */
  uint32_t frequency_update_count; /**< 新的边沿测速结果累计次数。 */
} motor_hall_sample_t;

/**
 * @brief 初始化Hall运行时解码器。
 * @param positive_next 8元素正向状态后继表，状态0和7应填0。
 * @param core_clock_hz 时间戳对应的CPU周期频率，单位Hz且必须大于0。
 * @param initial_state 当前原始Hall状态，范围0..7。
 * @param initial_timestamp_cycles 当前状态对应的32位周期时间戳。
 * @return 参数合法时返回true，否则清空解码器并返回false。
 * @details 函数复制后继表，调用方后续修改原数组不会影响解码器。
 */
bool motor_hall_decoder_init(
  const uint8_t positive_next[MOTOR_HALL_DECODER_STATE_COUNT],
  uint32_t core_clock_hz, uint8_t initial_state,
  uint32_t initial_timestamp_cycles);

/**
 * @brief 处理一次原始Hall边沿采样。
 * @param state 本次三路Hall组合状态，范围0..7。
 * @param edge_count 硬件端口累计边沿次数，仅用于输出诊断。
 * @param timestamp_cycles 本次边沿的32位周期时间戳，允许自然回绕。
 * @return 合法相邻跳变时返回true；重复、非法电平或跨状态跳变返回false。
 * @details 合法同向边沿使用最近最多6段周期滑动平均更新电频率。
 */
bool motor_hall_decoder_edge_process(uint8_t state, uint32_t edge_count,
                                     uint32_t timestamp_cycles);

/**
 * @brief 一致性读取Hall解码结果。
 * @param sample 输出状态、方向、频率及诊断计数，不允许为空。
 * @return 解码器已成功初始化且参数有效时返回true，否则返回false。
 * @details 使用序列锁避免主循环读取时被Hall中断更新而得到撕裂数据。
 */
bool motor_hall_decoder_sample_read(motor_hall_sample_t *sample);

#endif /* MOTOR_HALL_DECODER_H */
