/**
 * @file inertia_identification.c
 * @brief 双向驱动/滑行惯量，回归加速度与实测电磁转矩差。
 *
 * ADC统计真实帧1ms块，后台计算J=(Te_drive-Te_coast)/(a_drive-a_coast)。
 * 共用速度带减小摩擦偏差，不声称摩擦随速度/温度变化能被严格消除。
 */
#include "inertia_identification.h"
#include <stddef.h>
#include <math.h>
#include <string.h>
/* 限10秒单阶段、100秒整次、1ms块，保证t^2及t*速度整数累加不溢出。 */
#define INERTIA_MAX_STAGE_BLOCKS 10000U
#define INERTIA_MAX_TOTAL_BLOCKS 100000U
/* 块内速度/电流输入数值边界；不是产品额定值。 */
#define INERTIA_MAX_SPEED_MRPM 10000000L
#define INERTIA_MAX_CURRENT_MA 10000L
#define INERTIA_RPM_TO_RAD_S 0.10471975511965977f
/* 太小加速度差无法可靠除法；kg*m^2数值范围为首版保存边界。 */
#define INERTIA_MIN_ACCELERATION 1.0f
#define INERTIA_MIN_KG_M2 0.000000001f
#define INERTIA_MAX_KG_M2 0.1f

/** @brief 校验配置。@param config 非空配置。@return 有效为true。
 * @note 启动/后台，1ms节拍及有界计数是本算法前提。
 */
static bool inertia_config_valid(const inertia_identification_config_t *config)
{
    return config->confirmed && config->sample_rate_hz >= 1000U &&
        config->sample_rate_hz <= 100000U && config->sample_rate_hz % 1000U == 0U &&
        config->block_samples == config->sample_rate_hz / 1000U &&
        config->cycles > config->warmup_cycles && config->cycles <= INERTIA_IDENT_MAX_CYCLES &&
        config->start_samples > 0U && config->coast_settle_samples > 0U &&
        config->stop_hold_samples > 0U && config->timeout_samples > config->stop_hold_samples &&
        config->timeout_samples / config->block_samples <= INERTIA_MAX_STAGE_BLOCKS &&
        config->start_samples < config->timeout_samples &&
        config->coast_settle_samples < config->timeout_samples &&
        config->total_samples > config->timeout_samples &&
        config->total_samples / config->block_samples <= INERTIA_MAX_TOTAL_BLOCKS &&
        config->iq_step_ma > 0 && config->iq_step_ma < config->maximum_current_ma &&
        config->maximum_current_ma <= INERTIA_MAX_CURRENT_MA &&
        config->speed_low_mrpm > config->stop_speed_mrpm && config->stop_speed_mrpm > 0 &&
        config->speed_high_mrpm > config->speed_low_mrpm &&
        config->speed_turn_mrpm > config->speed_high_mrpm &&
        config->maximum_speed_mrpm > config->speed_turn_mrpm &&
        config->maximum_speed_mrpm <= INERTIA_MAX_SPEED_MRPM &&
        config->pole_pairs > 0U && config->pole_pairs <= 255U &&
        config->maximum_error_percent > 0U && config->maximum_error_percent <= 50U &&
        config->maximum_reject_percent > 0U && config->maximum_reject_percent <= 50U;
}

bool inertia_identification_init(inertia_identification_t *ident,
    const inertia_identification_config_t *config)
{
    if (ident == NULL || config == NULL)
    {
        return false;
    }
    *ident = (inertia_identification_t){0};
    ident->config = *config;
    return inertia_config_valid(config);
}

bool inertia_identification_start(inertia_identification_t *ident, uint32_t flux_uwb,
    uint32_t ld_nh, uint32_t lq_nh, bool fixed_bus)
{
    if (ident == NULL || !inertia_config_valid(&ident->config) || flux_uwb == 0U ||
        flux_uwb > 1000000U || ld_nh == 0U || lq_nh == 0U ||
        ident->capture.status == INERTIA_IDENT_RUNNING)
    {
        return false;
    }
    memset(&ident->capture, 0, sizeof(ident->capture));
    ident->stage_samples = 0U;
    ident->total_samples = 0U;
    ident->stop_samples = 0U;
    ident->block_count = 0U;
    ident->speed_sum = 0;
    ident->iq_sum = 0;
    ident->id_iq_sum = 0;
    ident->block_bad = false;
    ident->capture.status = INERTIA_IDENT_RUNNING;
    ident->capture.phase = INERTIA_IDENT_START_PHASE;
    ident->capture.flux_uwb = flux_uwb;
    ident->capture.ld_nh = ld_nh;
    ident->capture.lq_nh = lq_nh;
    ident->capture.fixed_bus = fixed_bus;
    return true;
}

int32_t inertia_identification_target(const inertia_identification_t *ident)
{
    if (ident == NULL || ident->capture.phase >= INERTIA_IDENT_START_PHASE ||
        ident->capture.phase % 6U >= 3U)
    {
        return 0;
    }
    return ident->capture.phase < 6U ? ident->config.iq_step_ma : -ident->config.iq_step_ma;
}

/** @brief 切阶段。@param ident ADC实例。@param phase 下一阶段。
 * @return 无。@note ADC独占；只清阶段计时，不清整次计时或冻结段。
 */
static void inertia_phase_set(inertia_identification_t *ident, uint32_t phase)
{
    ident->capture.phase = phase;
    ident->stage_samples = 0U;
}

/** @brief 取得当前驱动/滑行段。@param ident 实例。@return 合法段或NULL。
 * @note ADC只读索引，启动/尾段/停转不采回归数据。
 */
static inertia_identification_segment_t *inertia_segment(inertia_identification_t *ident)
{
    uint32_t phase = ident->capture.phase;
    if (ident->capture.cycle >= ident->config.cycles || phase >= INERTIA_IDENT_START_PHASE ||
        (phase % 6U != 1U && phase % 6U != 5U))
    {
        return NULL;
    }
    uint32_t index = (phase / 6U) * 2U + (phase % 6U == 5U ? 1U : 0U);
    return &ident->capture.segment[ident->capture.cycle][index];
}

/** @brief 回归统计收一个真实1ms块。@param ident ADC实例。@param speed 机械mrpm。
 * @return 无。@note 时间用阶段实际拍号，坏块跳过不会压缩回归时间轴。
 */
static void inertia_segment_add(inertia_identification_t *ident, int32_t speed)
{
    inertia_identification_segment_t *segment = inertia_segment(ident);
    if (segment == NULL)
    {
        return;
    }
    int64_t time = ident->stage_samples / ident->config.block_samples;
    ++segment->samples;
    segment->time_sum += time;
    segment->time_squared_sum += time * time;
    segment->speed_sum += speed;
    segment->speed_squared_sum += (int64_t)speed * speed;
    segment->time_speed_sum += time * speed;
    segment->iq_sum += ident->iq_sum / ident->config.block_samples;
    segment->id_iq_sum += ident->id_iq_sum / ident->config.block_samples;
}

/** @brief 处理驱动及滑行阶段。@param ident ADC实例。@param speed 方向投影速度mrpm。
 * @return 无。@note 只由完整真实块推进，越过整个采集带拒绝而不伪造段。
 */
static void inertia_motion_step(inertia_identification_t *ident, int32_t speed)
{
    uint32_t phase = ident->capture.phase % 6U;
    const inertia_identification_config_t *config = &ident->config;
    if (phase == 0U && speed >= config->speed_low_mrpm)
    {
        if (speed >= config->speed_high_mrpm)
        {
            ident->capture.status = INERTIA_IDENT_INVALID_RESULT;
        }
        else
        {
            inertia_phase_set(ident, ident->capture.phase + 1U);
        }
    }
    else if (phase == 1U && speed >= config->speed_high_mrpm)
    {
        inertia_phase_set(ident, ident->capture.phase + 1U);
    }
    else if (phase == 2U && speed >= config->speed_turn_mrpm)
    {
        inertia_phase_set(ident, ident->capture.phase + 1U);
    }
    else if (phase == 3U && ident->stage_samples >= config->coast_settle_samples)
    {
        inertia_phase_set(ident, ident->capture.phase + 1U);
    }
    else if (phase == 4U && speed <= config->speed_high_mrpm)
    {
        if (speed <= config->speed_low_mrpm)
        {
            ident->capture.status = INERTIA_IDENT_INVALID_RESULT;
        }
        else
        {
            inertia_phase_set(ident, ident->capture.phase + 1U);
        }
    }
    else if (phase == 5U && speed <= config->speed_low_mrpm)
    {
        uint32_t next = ident->capture.phase + 1U;
        if (next == INERTIA_IDENT_START_PHASE)
        {
            ++ident->capture.cycle;
            next = ident->capture.cycle < config->cycles ? 0U : INERTIA_IDENT_STOP_PHASE;
        }
        inertia_phase_set(ident, next);
    }
}

/** @brief 完整块门限调度。@param ident 实例。@param speed 机械域mrpm。
 * @return 无。@note 启动/最终停转允许坏块等待但计时不停止。
 */
static void inertia_block_step(inertia_identification_t *ident, int32_t speed)
{
    uint32_t phase = ident->capture.phase;
    if (phase == INERTIA_IDENT_START_PHASE)
    {
        if (ident->stage_samples >= ident->config.start_samples)
        {
            inertia_phase_set(ident, 0U);
        }
    }
    else if (phase == INERTIA_IDENT_STOP_PHASE)
    {
        if (speed >= -ident->config.stop_speed_mrpm && speed <= ident->config.stop_speed_mrpm)
        {
            ident->stop_samples += ident->config.block_samples;
            if (ident->stop_samples >= ident->config.stop_hold_samples)
            {
                ident->capture.status = INERTIA_IDENT_COLLECTED;
            }
        }
        else
        {
            ident->stop_samples = 0U;
        }
    }
    else
    {
        inertia_segment_add(ident, speed);
        inertia_motion_step(ident, phase < 6U ? speed : -speed);
    }
}

void inertia_identification_tick(inertia_identification_t *ident,
    const inertia_identification_sample_t *sample)
{
    if (ident == NULL || sample == NULL || ident->capture.status != INERTIA_IDENT_RUNNING)
    {
        return;
    }
    if (++ident->stage_samples >= ident->config.timeout_samples ||
        ++ident->total_samples >= ident->config.total_samples)
    {
        ident->capture.status = INERTIA_IDENT_TIMEOUT;
        return;
    }
    if (sample->speed_mrpm > ident->config.maximum_speed_mrpm ||
        sample->speed_mrpm < -ident->config.maximum_speed_mrpm)
    {
        ident->capture.status = INERTIA_IDENT_OVERSPEED;
        return;
    }
    if (sample->iq_ma > ident->config.maximum_current_ma ||
        sample->iq_ma < -ident->config.maximum_current_ma ||
        sample->id_ma > ident->config.maximum_current_ma ||
        sample->id_ma < -ident->config.maximum_current_ma)
    {
        ident->capture.status = INERTIA_IDENT_OVERCURRENT;
        return;
    }
    ident->speed_sum += sample->speed_mrpm;
    ident->iq_sum += sample->iq_ma;
    ident->id_iq_sum += (int64_t)sample->id_ma * sample->iq_ma;
    ident->block_bad |= !sample->valid;
    if (++ident->block_count < ident->config.block_samples)
    {
        return;
    }
    if (!ident->block_bad)
    {
        int32_t speed = (int32_t)(ident->speed_sum / ident->config.block_samples);
        inertia_block_step(ident, speed);
    }
    else
    {
        inertia_identification_segment_t *segment = inertia_segment(ident);
        if (segment != NULL)
        {
            ++segment->bad_blocks;
        }
        ident->stop_samples = 0U;
    }
    ident->block_count = 0U;
    ident->speed_sum = 0;
    ident->iq_sum = 0;
    ident->id_iq_sum = 0;
    ident->block_bad = false;
}

/** @brief 单段机械加速度和电磁转矩。@param config 本次配置。@param capture 冻结物理参数。
 * @param segment 冻结统计。@param acceleration rad/s^2输出。@param torque N*m输出。
 * @return 可解释为true。@note 后台浮点回归；最少10个真实1ms块。
 */
static bool inertia_segment_evaluate(const inertia_identification_config_t *config,
    const inertia_identification_capture_t *capture,
    const inertia_identification_segment_t *segment,
    float *acceleration, float *torque)
{
    uint64_t total = (uint64_t)segment->samples + segment->bad_blocks;
    if (segment->samples < 10U || (uint64_t)segment->bad_blocks * 100U >
        total * config->maximum_reject_percent)
    {
        return false;
    }
    double count = segment->samples;
    double denominator = count * segment->time_squared_sum -
        (double)segment->time_sum * segment->time_sum;
    if (!(denominator > 0.0))
    {
        return false;
    }
    double slope = (count * segment->time_speed_sum -
        (double)segment->time_sum * segment->speed_sum) / denominator;
    double variance = (double)segment->speed_squared_sum -
        (double)segment->speed_sum * segment->speed_sum / count;
    double covariance = (double)segment->time_speed_sum -
        (double)segment->time_sum * segment->speed_sum / count;
    double residual = variance - slope * covariance;
    /* 微小负残差来自浮点抵消；超出速度带的相对RMS误差拒绝回归。 */
    double error = residual > 0.0 ? sqrt(residual / count) : 0.0;
    if (!isfinite(error) || error * 100.0 >
        (double)(config->speed_high_mrpm - config->speed_low_mrpm) *
            config->maximum_error_percent)
    {
        return false;
    }
    /* mrpm/ms = rpm/s，回归斜率直接乘2pi/60，不乘极对数。 */
    *acceleration = (float)slope * INERTIA_RPM_TO_RAD_S;
    double iq = (double)segment->iq_sum / count / 1000.0;
    double id_iq = (double)segment->id_iq_sum / count / 1000000.0;
    double flux = (double)capture->flux_uwb / 1000000.0;
    double delta_l = ((double)capture->ld_nh - capture->lq_nh) / 1000000000.0;
    *torque = (float)(1.5 * config->pole_pairs * (flux * iq + delta_l * id_iq));
    return isfinite(*acceleration) && isfinite(*torque);
}

void inertia_identification_evaluate(const inertia_identification_config_t *config,
    const inertia_identification_capture_t *capture, inertia_identification_result_t *result)
{
    if (config == NULL || capture == NULL || result == NULL)
    {
        return;
    }
    *result = (inertia_identification_result_t)
    {
        .status = capture->status, .cycle = capture->cycle, .phase = capture->phase,
        .fixed_bus = capture->fixed_bus
    };
    if (capture->status != INERTIA_IDENT_COLLECTED || !inertia_config_valid(config) ||
        capture->cycle != config->cycles)
    {
        return;
    }
    result->status = INERTIA_IDENT_INVALID_RESULT;
    float values[INERTIA_IDENT_MAX_CYCLES];
    float sum = 0.0f;
    uint32_t used = 0U;
    for (uint32_t cycle = config->warmup_cycles; cycle < config->cycles; ++cycle)
    {
        float acceleration[INERTIA_IDENT_SEGMENTS];
        float torque[INERTIA_IDENT_SEGMENTS];
        for (uint32_t segment = 0U; segment < INERTIA_IDENT_SEGMENTS; ++segment)
        {
            if (!inertia_segment_evaluate(config, capture, &capture->segment[cycle][segment],
                &acceleration[segment], &torque[segment]))
            {
                return;
            }
        }
        /* 同一速度带仍可能因加速度非线性而均值不一致，拒绝明显不匹配的配对。 */
        for (uint32_t direction = 0U; direction < 2U; ++direction)
        {
            const inertia_identification_segment_t *drive =
                &capture->segment[cycle][direction * 2U];
            const inertia_identification_segment_t *coast =
                &capture->segment[cycle][direction * 2U + 1U];
            double speed_difference = (double)drive->speed_sum / drive->samples -
                (double)coast->speed_sum / coast->samples;
            if (fabs(speed_difference) * 100.0 >
                (double)(config->speed_high_mrpm - config->speed_low_mrpm) *
                    config->maximum_error_percent)
            {
                return;
            }
        }
        float delta_pos = acceleration[0] - acceleration[1];
        float delta_neg = acceleration[2] - acceleration[3];
        if (acceleration[0] <= 0.0f || acceleration[1] >= 0.0f ||
            acceleration[2] >= 0.0f || acceleration[3] <= 0.0f ||
            delta_pos <= INERTIA_MIN_ACCELERATION || delta_neg >= -INERTIA_MIN_ACCELERATION)
        {
            return;
        }
        float pos = (torque[0] - torque[1]) / delta_pos;
        float neg = (torque[2] - torque[3]) / delta_neg;
        float mean = (pos + neg) * 0.5f;
        if (!isfinite(pos) || !isfinite(neg) || pos < INERTIA_MIN_KG_M2 ||
            neg < INERTIA_MIN_KG_M2 || mean > INERTIA_MAX_KG_M2 ||
            fabsf(pos - neg) * 100.0f > mean * config->maximum_error_percent)
        {
            return;
        }
        values[used++] = mean;
        sum += mean;
    }
    float mean = sum / (float)used;
    for (uint32_t index = 0U; index < used; ++index)
    {
        if (fabsf(values[index] - mean) * 100.0f > mean * config->maximum_error_percent)
        {
            return;
        }
    }
    result->inertia_kg_m2 = mean;
    result->status = INERTIA_IDENT_SUCCESS;
}
