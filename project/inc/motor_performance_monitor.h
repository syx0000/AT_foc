#ifndef MOTOR_PERFORMANCE_MONITOR_H
#define MOTOR_PERFORMANCE_MONITOR_H

#include <stdint.h>

typedef struct
{
  volatile uint32_t latest_cycles; /**< 最近一次执行耗时，单位为CPU周期。 */
  volatile uint32_t max_cycles;    /**< 当前统计区间最大耗时，单位为CPU周期。 */
  volatile uint32_t call_count;    /**< 当前统计区间完成的测量次数。 */
} motor_performance_counter_t;

typedef struct
{
  uint32_t latest_cycles; /**< 快照时最近一次执行耗时。 */
  uint32_t max_cycles;    /**< 快照区间内的最大执行耗时。 */
  uint32_t call_count;    /**< 快照区间内的测量次数。 */
} motor_performance_snapshot_t;

/**
 * @brief 获取一次性能测量的起始周期时间戳。
 * @param 无。
 * @return 当前DWT周期计数值。
 * @note 不会清零DWT，可供多个统计点并行使用。
 */
uint32_t motor_performance_monitor_begin(void);

/**
 * @brief 结束一次测量并更新指定统计器。
 * @param counter 需要更新的独立性能统计器，不允许为空。
 * @param start_cycles 由motor_performance_monitor_begin()取得的起始时间戳。
 * @return 本次测量消耗的CPU周期数；counter为空时返回0。
 * @details 更新最近耗时、区间最大耗时和调用次数。每个中断或任务应使用
 *          独立counter，避免多个执行上下文同时写同一实例。
 */
uint32_t motor_performance_monitor_end(
  motor_performance_counter_t *counter,
  uint32_t start_cycles);

/**
 * @brief 原子读取并清零一个性能统计器。
 * @param counter 需要读取和清零的性能统计器，不允许为空。
 * @param snapshot 用于接收统计结果的快照，不允许为空。
 * @return 无。
 * @details 短暂屏蔽中断，避免读取过程中统计器被中断更新；函数会恢复调用前
 *          的中断屏蔽状态。
 */
void motor_performance_monitor_snapshot_reset(
  motor_performance_counter_t *counter,
  motor_performance_snapshot_t *snapshot);

#endif /* MOTOR_PERFORMANCE_MONITOR_H */
