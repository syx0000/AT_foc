#include "at32f45x.h"
#include "motor_adc_port.h"
#include "motor_control_config.h"
#include "motor_slow_sensor.h"

#define MOTOR_NTC_TABLE_STEP_C             5
#define MOTOR_NTC_ADC_OPEN_MIN_COUNTS   4000U

/* 10 kΩ@25°C、B=3950 K、10 kΩ上拉，-40~150°C，每5°C一点。 */
static const uint16_t mos_ntc_adc_table[] = {
  3996, 3955, 3900, 3830, 3740, 3629, 3495, 3337, 3156, 2955,
  2738, 2510, 2278, 2048, 1825, 1614, 1419, 1241, 1081, 940,
  815, 707, 613, 532, 462, 401, 350, 305, 267, 234,
  206, 181, 160, 142, 126, 112, 100, 89, 80
};

/* 10 kΩ@25°C、B=3435 K、10 kΩ上拉，-20~200°C，每5°C一点。 */
static const uint16_t motor_ntc_adc_table[] = {
  3627, 3507, 3368, 3210, 3037, 2850, 2654, 2451, 2248, 2048,
  1854, 1669, 1496, 1337, 1191, 1059, 940, 834, 740, 657,
  584, 519, 462, 412, 368, 329, 295, 265, 238, 215,
  194, 176, 159, 145, 132, 120, 110, 101, 92, 85,
  78, 72, 66, 61, 57
};

static motor_slow_sensor_state_t motor_slow_sensor_state;

/**
 * @brief 使用递减ADC查表值换算NTC温度。
 * @param adc_raw NTC分压ADC原始值。
 * @param table ADC查表数组，温度从低到高、ADC值从高到低排列。
 * @param table_size 查表数组元素数量。
 * @param minimum_temperature_c 表中第一个点对应的最低温度。
 * @param temperature_c 输出插值温度，不允许为空。
 * @return true表示输入有效并完成换算；ADC为0或达到开路阈值时返回false。
 */
static bool motor_slow_sensor_ntc_convert(
  uint16_t adc_raw,
  const uint16_t *table,
  uint32_t table_size,
  int16_t minimum_temperature_c,
  int16_t *temperature_c)
{
  uint32_t index;
  int32_t temperature;

  if ((table == 0) || (temperature_c == 0) || (table_size < 2U) ||
      (adc_raw == 0U) || (adc_raw >= MOTOR_NTC_ADC_OPEN_MIN_COUNTS))
  {
    return false;
  }

  if (adc_raw >= table[0])
  {
    *temperature_c = minimum_temperature_c;
    return true;
  }
  if (adc_raw <= table[table_size - 1U])
  {
    *temperature_c = (int16_t)(minimum_temperature_c +
      (int32_t)(table_size - 1U) * MOTOR_NTC_TABLE_STEP_C);
    return true;
  }

  for (index = 0U; index < (table_size - 1U); index++)
  {
    if (adc_raw >= table[index + 1U])
    {
      temperature = minimum_temperature_c +
        (int32_t)index * MOTOR_NTC_TABLE_STEP_C;
      temperature += ((int32_t)table[index] - adc_raw) *
        MOTOR_NTC_TABLE_STEP_C /
        ((int32_t)table[index] - table[index + 1U]);
      *temperature_c = (int16_t)temperature;
      return true;
    }
  }

  return false;
}

void motor_slow_sensor_init(void)
{
  motor_slow_sensor_state.bus_voltage_0p1v = 0U;
  motor_slow_sensor_state.motor_temperature_c = 0;
  motor_slow_sensor_state.mos_temperature_c = 0;
  motor_slow_sensor_state.motor_temperature_valid = false;
  motor_slow_sensor_state.mos_temperature_valid = false;
  motor_slow_sensor_state.sample_count = 0U;
}

bool motor_slow_sensor_process(void)
{
  motor_adc_slow_sample_t adc_sample;

  if ((!motor_adc_port_slow_sample_read(&adc_sample)) ||
      (adc_sample.sample_count == motor_slow_sensor_state.sample_count))
  {
    return false;
  }

  motor_slow_sensor_state.bus_voltage_0p1v = (uint16_t)(
    ((uint32_t)adc_sample.bus_voltage_raw * 33U *
     MOTOR_BUS_VOLTAGE_DIVIDER_RATIO +
     (MOTOR_ADC_FULL_SCALE_COUNTS / 2U)) /
    MOTOR_ADC_FULL_SCALE_COUNTS);
  motor_slow_sensor_state.motor_temperature_valid =
    motor_slow_sensor_ntc_convert(adc_sample.motor_temperature_raw,
      motor_ntc_adc_table,
      sizeof(motor_ntc_adc_table) / sizeof(motor_ntc_adc_table[0]),
      -20, &motor_slow_sensor_state.motor_temperature_c);
  motor_slow_sensor_state.mos_temperature_valid =
    motor_slow_sensor_ntc_convert(adc_sample.mos_temperature_raw,
      mos_ntc_adc_table,
      sizeof(mos_ntc_adc_table) / sizeof(mos_ntc_adc_table[0]),
      -40, &motor_slow_sensor_state.mos_temperature_c);
  motor_slow_sensor_state.sample_count = adc_sample.sample_count;

  return true;
}

bool motor_slow_sensor_state_read(motor_slow_sensor_state_t *state)
{
  if (state == 0)
  {
    return false;
  }

  *state = motor_slow_sensor_state;
  return state->sample_count != 0U;
}
