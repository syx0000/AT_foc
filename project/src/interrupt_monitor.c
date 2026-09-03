#include "at32f45x.h"
#include "interrupt_monitor.h"
#include "motor_log.h"
#include "motor_timebase.h"
#include "motor_adc_port.h"
#include "motor_current_sample.h"
#include "motor_slow_sensor.h"
#include "motor_open_loop.h"
#include "motor_pwm_port.h"
#include "motor_hall_port.h"
#include "motor_hall_angle_observer.h"
#include "motor_hall_angle_estimator.h"
#include "motor_current_transform.h"

#define INTERRUPT_MONITOR_REPORT_TICKS 1000U
#define INTERRUPT_MONITOR_ADC_FAST_HZ  10000U

interrupt_monitor_counters_t interrupt_monitor_counters;
motor_performance_counter_t adc_fast_performance_counter;

void interrupt_monitor_poll(void)
{
  interrupt_monitor_counters_t snapshot;
  motor_performance_snapshot_t adc_fast_snapshot;
  uint32_t adc_fast_period_cycles;
  uint32_t adc_fast_max_load_x100;
  motor_adc_fast_sample_t adc_fast_sample;
  motor_adc_slow_sample_t adc_slow_sample;
  bool adc_fast_sample_valid;
  bool adc_slow_sample_valid;
  motor_current_sample_state_t current_state;
  bool current_state_valid;
  motor_slow_sensor_state_t slow_sensor_state;
  bool slow_sensor_state_valid;
  motor_open_loop_status_t open_loop_status;
  bool open_loop_status_valid;
  motor_hall_sample_t hall_sample;
  bool hall_sample_valid;
  motor_hall_angle_observer_t hall_angle_observer;
  bool hall_angle_observer_valid;
  motor_hall_angle_estimator_t hall_angle_estimator;
  bool hall_angle_estimator_valid;
  motor_current_transform_state_t current_transform;
  bool current_transform_valid;
  motor_current_transform_statistics_t current_transform_statistics;
  bool current_transform_statistics_valid;

  if (interrupt_monitor_counters.systick < INTERRUPT_MONITOR_REPORT_TICKS)
  {
    return;
  }

  __disable_irq();
  snapshot = interrupt_monitor_counters;
  interrupt_monitor_counters.systick = 0U;
  interrupt_monitor_counters.adc_slow_dma_complete = 0U;
  interrupt_monitor_counters.adc_fast_complete = 0U;
  interrupt_monitor_counters.adc_trigger_fail = 0U;
  interrupt_monitor_counters.tmr1_channel4 = 0U;
  interrupt_monitor_counters.hall_a_edge = 0U;
  interrupt_monitor_counters.hall_b_edge = 0U;
  interrupt_monitor_counters.hall_c_edge = 0U;
  interrupt_monitor_counters.can1_rx = 0U;
  interrupt_monitor_counters.can1_error = 0U;
  interrupt_monitor_counters.usart1_idle = 0U;
  interrupt_monitor_counters.usart1_error = 0U;
  interrupt_monitor_counters.usart3_idle = 0U;
  interrupt_monitor_counters.usart3_error = 0U;
  __enable_irq();

  motor_performance_monitor_snapshot_reset(&adc_fast_performance_counter,
                                            &adc_fast_snapshot);
  adc_fast_sample_valid = motor_adc_port_fast_sample_read(&adc_fast_sample);
  adc_slow_sample_valid = motor_adc_port_slow_sample_read(&adc_slow_sample);
  current_state_valid = motor_current_sample_state_read(&current_state);
  slow_sensor_state_valid = motor_slow_sensor_state_read(&slow_sensor_state);
  open_loop_status_valid = motor_open_loop_status_read(&open_loop_status);
  hall_sample_valid = motor_hall_port_sample_read(&hall_sample);
  hall_angle_observer_valid =
    motor_hall_angle_observer_read(&hall_angle_observer);
  hall_angle_estimator_valid =
    motor_hall_angle_estimator_read(&hall_angle_estimator);
  current_transform_valid =
    motor_current_transform_state_read(&current_transform);
  current_transform_statistics_valid =
    motor_current_transform_statistics_snapshot_reset(
      &current_transform_statistics);

  adc_fast_period_cycles = system_core_clock / INTERRUPT_MONITOR_ADC_FAST_HZ;
  adc_fast_max_load_x100 =
    (adc_fast_snapshot.max_cycles * 10000U) / adc_fast_period_cycles;

  LOGI("IRQ Hz: tick=%lu tmr1_c4=%lu adc_fast=%lu adc_slow_dma=%lu adc_fail=%lu\r\n",
       snapshot.systick,
       snapshot.tmr1_channel4,
       snapshot.adc_fast_complete,
       snapshot.adc_slow_dma_complete,
       snapshot.adc_trigger_fail);
  LOGI("ADC fast cost: current=%lu cycles/%lu us max=%lu cycles/%lu us load=%lu.%02lu%% samples=%lu\r\n",
       adc_fast_snapshot.latest_cycles,
       motor_timebase_cycles_to_us(adc_fast_snapshot.latest_cycles),
       adc_fast_snapshot.max_cycles,
       motor_timebase_cycles_to_us(adc_fast_snapshot.max_cycles),
       adc_fast_max_load_x100 / 100U,
       adc_fast_max_load_x100 % 100U,
       adc_fast_snapshot.call_count);
  if (adc_fast_sample_valid && adc_slow_sample_valid)
  {
    LOGI("ADC raw: soa=%u sob=%u temp_motor=%u soc=%u temp_mos=%u bus_voltage=%u\r\n",
         (unsigned int)adc_fast_sample.phase_a_raw,
         (unsigned int)adc_fast_sample.phase_b_raw,
         (unsigned int)adc_slow_sample.motor_temperature_raw,
         (unsigned int)adc_slow_sample.phase_c_raw,
         (unsigned int)adc_slow_sample.mos_temperature_raw,
         (unsigned int)adc_slow_sample.bus_voltage_raw);
  }
  if (current_state_valid)
  {
    LOGI("Current: ia=%ld ib=%ld ic=%ld mA q15=%d/%d/%d oc=%lu fault=%u\r\n",
         (long)current_state.phase_a_ma,
         (long)current_state.phase_b_ma,
         (long)current_state.phase_c_ma,
         (int)current_state.phase_a_q15,
         (int)current_state.phase_b_q15,
         (int)current_state.phase_c_q15,
         (unsigned long)current_state.overcurrent_count,
         (unsigned int)current_state.overcurrent_fault);
  }
  if (slow_sensor_state_valid)
  {
    LOGI("Sensors: bus=%u.%u V motor_temp=%d C/valid=%u mos_temp=%d C/valid=%u\r\n",
         (unsigned int)(slow_sensor_state.bus_voltage_0p1v / 10U),
         (unsigned int)(slow_sensor_state.bus_voltage_0p1v % 10U),
         (int)slow_sensor_state.motor_temperature_c,
         (unsigned int)slow_sensor_state.motor_temperature_valid,
         (int)slow_sensor_state.mos_temperature_c,
         (unsigned int)slow_sensor_state.mos_temperature_valid);
  }
  if (open_loop_status_valid)
  {
    LOGI("Open loop: state=%u angle=%u frequency=%ld/%ld mHz voltage=%ld/%ld mV duty=%u/%u/%u MOE=%u\r\n",
         (unsigned int)open_loop_status.state,
         (unsigned int)open_loop_status.electrical_angle_u16,
         (long)open_loop_status.actual_frequency_millihz,
         (long)open_loop_status.target_frequency_millihz,
         (long)open_loop_status.applied_direct_voltage_mv,
         (long)open_loop_status.applied_quadrature_voltage_mv,
         (unsigned int)open_loop_status.duty_a_q15,
         (unsigned int)open_loop_status.duty_b_q15,
         (unsigned int)open_loop_status.duty_c_q15,
         motor_pwm_port_output_is_enabled() ? 1U : 0U);
  }
  if (hall_sample_valid)
  {
    LOGI("Hall: state=%u (HA/HB/HC=%u/%u/%u) valid=%u edges=%lu direction=%lu/%lu invalid=%lu duplicate=%lu frequency=%lu.%03lu Hz\r\n",
         (unsigned int)hall_sample.state,
         (unsigned int)(hall_sample.state & 0x01U),
         (unsigned int)((hall_sample.state >> 1) & 0x01U),
         (unsigned int)((hall_sample.state >> 2) & 0x01U),
         (unsigned int)hall_sample.valid,
         (unsigned long)hall_sample.edge_count,
         (unsigned long)hall_sample.positive_count,
         (unsigned long)hall_sample.negative_count,
         (unsigned long)hall_sample.invalid_transition_count,
         (unsigned long)hall_sample.duplicate_count,
         (unsigned long)(hall_sample.electrical_frequency_millihz / 1000U),
         (unsigned long)(hall_sample.electrical_frequency_millihz % 1000U));
  }
  if (hall_angle_observer_valid)
  {
    LOGI("Hall edge angle: s1=%u s2=%u s3=%u s4=%u s5=%u s6=%u counts=%lu/%lu/%lu/%lu/%lu/%lu\r\n",
         (unsigned int)hall_angle_observer.state_angle_u16[1],
         (unsigned int)hall_angle_observer.state_angle_u16[2],
         (unsigned int)hall_angle_observer.state_angle_u16[3],
         (unsigned int)hall_angle_observer.state_angle_u16[4],
         (unsigned int)hall_angle_observer.state_angle_u16[5],
         (unsigned int)hall_angle_observer.state_angle_u16[6],
         (unsigned long)hall_angle_observer.state_count[1],
         (unsigned long)hall_angle_observer.state_count[2],
         (unsigned long)hall_angle_observer.state_count[3],
         (unsigned long)hall_angle_observer.state_count[4],
         (unsigned long)hall_angle_observer.state_count[5],
         (unsigned long)hall_angle_observer.state_count[6]);
  }
  if (hall_angle_estimator_valid && open_loop_status_valid)
  {
    int16_t angle_error =
      (int16_t)(hall_angle_estimator.electrical_angle_u16 -
                open_loop_status.electrical_angle_u16);
    LOGI("Hall estimator: valid=%u state=%u angle=%u error=%d frequency=%lu.%03lu Hz\r\n",
         (unsigned int)hall_angle_estimator.valid,
         (unsigned int)hall_angle_estimator.hall_state,
         (unsigned int)hall_angle_estimator.electrical_angle_u16,
         (int)angle_error,
         (unsigned long)(hall_angle_estimator.electrical_frequency_millihz / 1000U),
         (unsigned long)(hall_angle_estimator.electrical_frequency_millihz % 1000U));
  }
  if (current_transform_valid && current_transform.valid)
  {
    LOGI("Current dq: alpha=%d beta=%d id=%d iq=%d q15 angle=%u samples=%lu\r\n",
         (int)current_transform.alpha_q15,
         (int)current_transform.beta_q15,
         (int)current_transform.direct_q15,
         (int)current_transform.quadrature_q15,
         (unsigned int)current_transform.electrical_angle_u16,
         (unsigned long)current_transform.sample_count);
  }
  if (current_transform_statistics_valid)
  {
    LOGI("Current dq stats: id_avg=%d min/max=%d/%d iq_avg=%d min/max=%d/%d q15 samples=%lu\r\n",
         (int)current_transform_statistics.direct_average_q15,
         (int)current_transform_statistics.direct_minimum_q15,
         (int)current_transform_statistics.direct_maximum_q15,
         (int)current_transform_statistics.quadrature_average_q15,
         (int)current_transform_statistics.quadrature_minimum_q15,
         (int)current_transform_statistics.quadrature_maximum_q15,
         (unsigned long)current_transform_statistics.sample_count);
  }
  LOGI("IRQ Hz: hall=%lu/%lu/%lu can=%lu/%lu uart1=%lu/%lu uart3=%lu/%lu\r\n",
       snapshot.hall_a_edge,
       snapshot.hall_b_edge,
       snapshot.hall_c_edge,
       snapshot.can1_rx,
       snapshot.can1_error,
       snapshot.usart1_idle,
       snapshot.usart1_error,
       snapshot.usart3_idle,
       snapshot.usart3_error);
  LOGI("--------------------\r\n");
}
