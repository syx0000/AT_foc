#ifndef MOTOR_ADC_PORT_H
#define MOTOR_ADC_PORT_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_ADC_SLOW_DMA_BUFFER_SIZE 4U

typedef struct
{
  uint16_t phase_a_raw; /**< ADC1注入组SOA原始值。 */
  uint16_t phase_b_raw; /**< ADC2注入组SOB原始值。 */
  uint32_t sample_count; /**< 上电后快速采样捕获次数。 */
} motor_adc_fast_sample_t;

typedef struct
{
  uint16_t motor_temperature_raw; /**< ADC1普通组rank1，TEMP_MOTOR原始值。 */
  uint16_t phase_c_raw;           /**< ADC2普通组rank1，SOC原始值。 */
  uint16_t mos_temperature_raw;   /**< ADC1普通组rank2，TEMP_MOS原始值。 */
  uint16_t bus_voltage_raw;       /**< ADC2普通组rank2，SO3母线电压原始值。 */
  uint32_t sample_count;          /**< 上电后慢速DMA采样捕获次数。 */
} motor_adc_slow_sample_t;

extern volatile uint16_t
  motor_adc_ordinary_dma_buffer[MOTOR_ADC_SLOW_DMA_BUFFER_SIZE];

/**
 * @brief 初始化电机ADC端口层的软件状态。
 * @param 无。
 * @return 无。
 * @details 清零快速及慢速采样快照和计数器，不修改WorkBench生成的ADC、DMA
 *          或定时器硬件配置。
 */
void motor_adc_port_init(void);

/**
 * @brief 捕获一次双ADC注入组快速采样。
 * @param sample 可选输出；非空时返回本次SOA、SOB原始值和累计采样次数。
 * @return 无。
 * @details 读取ADC1注入通道SOA和ADC2注入通道SOB的12位原始值，并更新快速
 *          采样快照。应在ADC预注入转换完成中断中调用。
 */
void motor_adc_port_fast_sample_capture(motor_adc_fast_sample_t *sample);

/**
 * @brief 读取最近一次快速ADC采样快照。
 * @param sample 输出采样数据，不允许为空。
 * @return true表示已有有效快照；false表示参数为空或尚未完成首次采样。
 * @details 通过短临界区复制快照，保证主循环读取时字段属于同一次采样。
 */
bool motor_adc_port_fast_sample_read(motor_adc_fast_sample_t *sample);

/**
 * @brief 捕获一次双ADC普通组DMA慢速采样。
 * @param 无。
 * @return 无。
 * @details 从4个半字DMA缓冲区复制两组同步转换结果。当前Mode 1排列为ADC1
 *          rank1、ADC2 rank1、ADC1 rank2、ADC2 rank2。SO3硬件信号作为
 *          母线电压原始值输出。应在DMA完整传输中断中调用。
 */
void motor_adc_port_slow_sample_capture(void);

/**
 * @brief 读取最近一次慢速ADC采样快照。
 * @param sample 输出采样数据，不允许为空。
 * @return true表示已有有效快照；false表示参数为空或尚未完成首次采样。
 * @details 通过短临界区复制快照，避免与DMA完成中断更新发生竞争。
 */
bool motor_adc_port_slow_sample_read(motor_adc_slow_sample_t *sample);

#endif /* MOTOR_ADC_PORT_H */
