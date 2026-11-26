/**
 * @file inertia_identification.h
 * @brief 电机轴等效惯量驱动/自由减速辨识，ADC采集、后台计算。
 *
 * 输入机械域速度和实测D/Q电流，输出有界冻结段数据及kg*m^2结果。
 * ADC独占tick；后台短临界区提交启动/复制capture，不操作硬件、不阻塞。
 */
#ifndef INERTIA_IDENTIFICATION_H
#define INERTIA_IDENTIFICATION_H
#include <stdbool.h>
#include <stdint.h>
/* 最多10个双向周期，每周期正驱动/正滑行/负驱动/负滑行4段。 */
#define INERTIA_IDENT_MAX_CYCLES 10U
#define INERTIA_IDENT_SEGMENTS 4U
/* 每方向6阶段，12为启动等待，13为最终自由停转。 */
#define INERTIA_IDENT_START_PHASE 12U
#define INERTIA_IDENT_STOP_PHASE 13U

typedef enum
{
    INERTIA_IDENT_IDLE = 0,
    INERTIA_IDENT_RUNNING,
    INERTIA_IDENT_COLLECTED,
    INERTIA_IDENT_SUCCESS,
    INERTIA_IDENT_CANCELLED,
    INERTIA_IDENT_TIMEOUT,
    INERTIA_IDENT_FEEDBACK,
    INERTIA_IDENT_OVERCURRENT,
    INERTIA_IDENT_OUTPUT_ERROR,
    INERTIA_IDENT_INVALID_RESULT,
    INERTIA_IDENT_OVERSPEED
} inertia_identification_status_t;

typedef struct
{
    bool confirmed;
    uint32_t sample_rate_hz;
    uint32_t block_samples; /**< 真实帧1ms块，整数平均后处理速度门限。 */
    uint32_t start_samples;
    uint32_t coast_settle_samples;
    uint32_t stop_hold_samples;
    uint32_t timeout_samples; /**< 各阶段持续上限，不因坏帧重置。 */
    uint32_t total_samples; /**< 整次上限，包含启动、所有周期及最终停转。 */
    uint32_t cycles;
    uint32_t warmup_cycles;
    int32_t iq_step_ma;
    int32_t speed_low_mrpm; /**< 驱动/减速共享测量带下界，电机mrpm。 */
    int32_t speed_high_mrpm;
    int32_t speed_turn_mrpm; /**< 驱动继续至此才切Iq=0，留减速建立余量。 */
    int32_t maximum_speed_mrpm;
    int32_t stop_speed_mrpm;
    int32_t maximum_current_ma;
    uint32_t pole_pairs;
    uint32_t maximum_error_percent; /**< 双向差异及跨周期一致性阈值。 */
    uint32_t maximum_reject_percent; /**< 每测量段坏块/(真实块+坏块)上限。 */
} inertia_identification_config_t;

/** ADC整数回归统计：t单位1ms块，速度mrpm，电流mA。 */
typedef struct
{
    uint32_t samples;
    uint32_t bad_blocks;
    int64_t time_sum;
    int64_t time_squared_sum;
    int64_t speed_sum;
    int64_t speed_squared_sum;
    int64_t time_speed_sum;
    int64_t iq_sum;
    int64_t id_iq_sum; /**< 实测Id*Iq的块平均，mA^2。 */
} inertia_identification_segment_t;

typedef struct
{
    inertia_identification_status_t status;
    uint32_t cycle;
    uint32_t phase;
    uint32_t flux_uwb;
    uint32_t ld_nh;
    uint32_t lq_nh;
    bool fixed_bus;
    inertia_identification_segment_t segment[INERTIA_IDENT_MAX_CYCLES][INERTIA_IDENT_SEGMENTS];
} inertia_identification_capture_t;

typedef struct
{
    inertia_identification_config_t config;
    inertia_identification_capture_t capture;
    uint32_t stage_samples;
    uint32_t total_samples;
    uint32_t stop_samples;
    uint32_t block_count;
    int64_t speed_sum;
    int64_t iq_sum;
    int64_t id_iq_sum;
    bool block_bad;
} inertia_identification_t;

typedef struct
{
    int32_t speed_mrpm;
    int32_t iq_ma;
    int32_t id_ma;
    bool valid; /**< 真实帧且输出无限幅；预测允许闭环但不作为辨识测量。 */
} inertia_identification_sample_t;

typedef struct
{
    inertia_identification_status_t status;
    uint32_t cycle;
    uint32_t phase;
    bool fixed_bus;
    bool stored;
    float inertia_kg_m2; /**< 电机轴总等效惯量，包含当前机械负载及传动。 */
} inertia_identification_result_t;

/** @brief 初始化。@param ident 输出。@param config 只读配置。@return 有效为true。
 * @note 启动独占；清空数据并校验整数累加、阶段及存储边界。
 */
bool inertia_identification_init(inertia_identification_t *ident,
    const inertia_identification_config_t *config);
/** @brief 提交辨识。@param ident 实例。@param flux_uwb 本次冻结磁链微Wb。
 * @param ld_nh D电感nH。@param lq_nh Q电感nH。@param fixed_bus 磁链来源标记。
 * @return 接受为true。@note 后台短临界区发布前调用，无功率操作。
 */
bool inertia_identification_start(inertia_identification_t *ident, uint32_t flux_uwb,
    uint32_t ld_nh, uint32_t lq_nh, bool fixed_bus);
/** @brief 本阶段电流目标。@param ident 实例。@return 机械域Iq mA。
 * @note ADC只读；启动、滑行及停转阶段为0，正常退出不恢复旧目标。
 */
int32_t inertia_identification_target(const inertia_identification_t *ident);
/** @brief 同拍输入采集。@param ident ADC实例。@param sample 同步输入。
 * @return 无。@note ADC独占，只整数有界运算，无日志/等待/动态内存。
 */
void inertia_identification_tick(inertia_identification_t *ident,
    const inertia_identification_sample_t *sample);
/** @brief 后台回归及周期一致性验收。@param config 本次配置。@param capture 冻结副本。
 * @param result 输出。@return 无。@note 后台浮点计算，不在临界区，不改变控制。
 */
void inertia_identification_evaluate(const inertia_identification_config_t *config,
    const inertia_identification_capture_t *capture, inertia_identification_result_t *result);
#endif
