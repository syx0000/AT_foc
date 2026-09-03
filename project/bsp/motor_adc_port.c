#include "at32f45x.h"
#include "motor_adc_port.h"

volatile uint16_t
  motor_adc_ordinary_dma_buffer[MOTOR_ADC_SLOW_DMA_BUFFER_SIZE];

static volatile motor_adc_fast_sample_t motor_adc_fast_sample;
static volatile motor_adc_slow_sample_t motor_adc_slow_sample;

/**
 * @brief 恢复调用前的中断屏蔽状态。
 * @param primask 进入临界区前读取的PRIMASK值。
 * @return 无。
 * @details 避免读取函数在原本已关闭中断的上下文中错误地重新开启中断。
 */
static void motor_adc_port_irq_restore(uint32_t primask)
{
  __set_PRIMASK(primask);
}

void motor_adc_port_init(void)
{
  uint32_t index;

  for (index = 0U; index < MOTOR_ADC_SLOW_DMA_BUFFER_SIZE; index++)
  {
    motor_adc_ordinary_dma_buffer[index] = 0U;
  }

  motor_adc_fast_sample.phase_a_raw = 0U;
  motor_adc_fast_sample.phase_b_raw = 0U;
  motor_adc_fast_sample.sample_count = 0U;
  motor_adc_slow_sample.motor_temperature_raw = 0U;
  motor_adc_slow_sample.phase_c_raw = 0U;
  motor_adc_slow_sample.mos_temperature_raw = 0U;
  motor_adc_slow_sample.bus_voltage_raw = 0U;
  motor_adc_slow_sample.sample_count = 0U;
}

void motor_adc_port_fast_sample_capture(motor_adc_fast_sample_t *sample)
{
  motor_adc_fast_sample.phase_a_raw =
    adc_preempt_conversion_data_get(ADC1, ADC_PREEMPT_CHANNEL_1);
  motor_adc_fast_sample.phase_b_raw =
    adc_preempt_conversion_data_get(ADC2, ADC_PREEMPT_CHANNEL_1);
  motor_adc_fast_sample.sample_count++;

  if (sample != 0)
  {
    sample->phase_a_raw = motor_adc_fast_sample.phase_a_raw;
    sample->phase_b_raw = motor_adc_fast_sample.phase_b_raw;
    sample->sample_count = motor_adc_fast_sample.sample_count;
  }
}

bool motor_adc_port_fast_sample_read(motor_adc_fast_sample_t *sample)
{
  uint32_t primask;

  if (sample == 0)
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  sample->phase_a_raw = motor_adc_fast_sample.phase_a_raw;
  sample->phase_b_raw = motor_adc_fast_sample.phase_b_raw;
  sample->sample_count = motor_adc_fast_sample.sample_count;
  motor_adc_port_irq_restore(primask);

  return sample->sample_count != 0U;
}

void motor_adc_port_slow_sample_capture(void)
{
  motor_adc_slow_sample.motor_temperature_raw =
    motor_adc_ordinary_dma_buffer[0];
  motor_adc_slow_sample.phase_c_raw = motor_adc_ordinary_dma_buffer[1];
  motor_adc_slow_sample.mos_temperature_raw =
    motor_adc_ordinary_dma_buffer[2];
  motor_adc_slow_sample.bus_voltage_raw =
    motor_adc_ordinary_dma_buffer[3];
  motor_adc_slow_sample.sample_count++;
}

bool motor_adc_port_slow_sample_read(motor_adc_slow_sample_t *sample)
{
  uint32_t primask;

  if (sample == 0)
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  sample->motor_temperature_raw = motor_adc_slow_sample.motor_temperature_raw;
  sample->phase_c_raw = motor_adc_slow_sample.phase_c_raw;
  sample->mos_temperature_raw = motor_adc_slow_sample.mos_temperature_raw;
  sample->bus_voltage_raw = motor_adc_slow_sample.bus_voltage_raw;
  sample->sample_count = motor_adc_slow_sample.sample_count;
  motor_adc_port_irq_restore(primask);

  return sample->sample_count != 0U;
}
