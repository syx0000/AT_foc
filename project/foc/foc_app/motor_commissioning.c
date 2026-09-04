#include <stddef.h>
#include "at32f45x.h"
#include "motor_board_config.h"
#include "motor_commissioning.h"
#include "motor_control.h"
#include "motor_control_config.h"
#include "motor_current_calibration.h"
#include "motor_current_loop_test.h"
#include "motor_current_pi_design.h"
#include "motor_current_sample.h"
#include "motor_hall_calibration.h"
#include "motor_hall_decoder.h"
#include "motor_hall_port.h"
#include "motor_inductance_identification.h"
#include "motor_log.h"
#include "motor_open_loop.h"
#include "motor_parameter.h"
#include "motor_pwm_port.h"
#include "motor_resistance_identification.h"
#include "motor_timebase.h"
#include "motor_torque_loop_test.h"
#include "wk_system.h"

#define MOTOR_COMMISSIONING_HALL_VOLTAGE_MV 1200L
#define MOTOR_COMMISSIONING_HALL_FREQUENCY_MILLIHZ 5000L
#define MOTOR_COMMISSIONING_HALL_TIMEOUT_MS 12000U
#define MOTOR_COMMISSIONING_HALL_SAMPLES 3U

static volatile motor_commissioning_status_t commissioning_status;
static volatile bool commissioning_abort_requested;
static volatile bool commissioning_hall_collecting;
static volatile uint8_t commissioning_hall_previous_state;
static motor_hall_calibration_t commissioning_hall;

/** @brief 原子更新当前任务步骤号。@param step 从1开始的步骤号。@return 无。 */
static void commissioning_step_set(uint32_t step)
{
  commissioning_status.step = step;
}

/** @brief 按当前活动参数和原始Hall状态重载运行时解码器。 @param 无。 @return 成功返回true。 */
static bool commissioning_hall_decoder_reload(void)
{
  motor_parameter_t p; motor_hall_port_sample_t raw;
  return motor_parameter_active_read(&p) && motor_hall_port_sample_read(&raw) &&
    motor_hall_decoder_init(p.hall_positive_next,system_core_clock,
                            raw.state,raw.timestamp_cycles);
}

/** @brief 使用候选参数完成低功率验证后无条件恢复原活动参数。 @param d_result Id测试结果。 @param q_result Iq测试结果。 @return 两项测试及恢复均成功时返回true。 */
static bool commissioning_candidate_trial_run(
  motor_current_loop_test_result_t *d_result,
  motor_torque_loop_test_result_t *q_result)
{
  motor_parameter_t backup;
  motor_parameter_t trial;
  bool tests_ok = false;
  bool restore_ok;

  if (!motor_parameter_trial_begin(&backup)) return false;
  if (motor_parameter_active_read(&trial) &&
      commissioning_hall_decoder_reload())
  {
    tests_ok = motor_current_loop_test_run_with_gains(
      d_result, trial.current_d_kp_q15, trial.current_q_kp_q15,
      trial.current_ki_q15) &&
      motor_torque_loop_test_run_with_gains(
        q_result, trial.current_d_kp_q15, trial.current_q_kp_q15,
        trial.current_ki_q15);
  }
  motor_control_stop();
  restore_ok=motor_parameter_trial_end(&backup) &&
             commissioning_hall_decoder_reload();
  return tests_ok && restore_ok;
}

void motor_commissioning_init(void)
{
  commissioning_status.state = MOTOR_COMMISSIONING_IDLE;
  commissioning_status.task = MOTOR_COMMISSIONING_TASK_NONE;
  commissioning_status.step = 0U; commissioning_status.error = 0U;
  commissioning_status.run_count = 0U;
  commissioning_abort_requested = false; commissioning_hall_collecting = false;
}

static bool commissioning_ready(void)
{
  motor_control_status_t status;
  return motor_control_status_read(&status) &&
    (status.state == MOTOR_CONTROL_STATE_READY) &&
    (!motor_pwm_port_output_is_enabled());
}

static bool commissioning_offset_run(void)
{
  motor_current_calibration_result_t result;
  if (!motor_current_calibration_run(1024U, 500U, &result) ||
      !motor_current_sample_offsets_set(result.phase_a_offset_raw,
                                        result.phase_b_offset_raw)) return false;
  LOGI("Commissioning current offset: PASS a=%u b=%u samples=%lu\r\n",
    result.phase_a_offset_raw, result.phase_b_offset_raw, result.sample_count);
  return true;
}

static bool commissioning_resistance_run(void)
{
  motor_resistance_identification_result_t result;
  if (!motor_resistance_identification_run(&result)) return false;
  if (!motor_parameter_candidate_field_set(
      MOTOR_PARAMETER_FIELD_PHASE_RESISTANCE_MOHM,
      (int32_t)result.resistance_average_mohm)) return false;
  LOGI("Commissioning Rs: PASS phase=%lu mOhm voltage=%u mV current=%ld mA samples=%lu candidate=1\r\n",
    result.resistance_average_mohm, result.applied_voltage_mv,
    result.phase_a_average_ma, result.sample_count);
  return true;
}

static bool commissioning_inductance_run(void)
{
  motor_parameter_t p; motor_inductance_identification_result_t result;
  if (!motor_parameter_candidate_read(&p) ||
      !motor_inductance_identification_run(p.phase_resistance_mohm, &result)) return false;
  p.direct_inductance_uh = result.direct_inductance_uh;
  p.quadrature_inductance_uh = result.quadrature_inductance_uh;
  if (!motor_parameter_candidate_set(&p)) return false;
  LOGI("Commissioning L: PASS ld=%lu lq=%lu uH id_amp=%lu iq_amp=%lu mA candidate=1\r\n",
    result.direct_inductance_uh, result.quadrature_inductance_uh,
    result.direct_current_amplitude_ma, result.quadrature_current_amplitude_ma);
  return true;
}

static bool commissioning_pi_run(void)
{
  motor_parameter_t p; motor_current_pi_design_result_t result;
  if (!motor_parameter_candidate_read(&p) ||
      !motor_current_pi_design(p.phase_resistance_mohm, p.direct_inductance_uh,
        p.quadrature_inductance_uh, p.current_loop_bandwidth_hz,
        MOTOR_PWM_FREQUENCY_HZ, &result)) return false;
  p.current_d_kp_q15=result.direct_kp_q15; p.current_q_kp_q15=result.quadrature_kp_q15;
  p.current_ki_q15=result.integral_gain_q15;
  if (!motor_parameter_candidate_set(&p)) return false;
  LOGI("Commissioning current PI: PASS kp_d=%ld kp_q=%ld ki=%ld candidate=1\r\n",
    result.direct_kp_q15,result.quadrature_kp_q15,result.integral_gain_q15);
  return true;
}

/**
 * @brief 使用候选参数中的PI增益执行固定角度Id验证。
 * @param result 输出Id验证结果，不允许为空。
 * @return 候选参数有效且测试通过返回true。
 */
static bool commissioning_current_d_run(
  motor_current_loop_test_result_t *result)
{
  motor_parameter_t parameter;

  return motor_parameter_candidate_read(&parameter) &&
    motor_current_loop_test_run_with_gains(
      result, parameter.current_d_kp_q15, parameter.current_q_kp_q15,
      parameter.current_ki_q15);
}

/**
 * @brief 使用候选参数中的PI增益执行开环到Hall电流环接管验证。
 * @param result 输出Iq及接管验证结果，不允许为空。
 * @return 候选参数有效且测试通过返回true。
 */
static bool commissioning_current_q_run(
  motor_torque_loop_test_result_t *result)
{
  motor_parameter_t parameter;

  return motor_parameter_candidate_read(&parameter) &&
    motor_torque_loop_test_run_with_gains(
      result, parameter.current_d_kp_q15, parameter.current_q_kp_q15,
      parameter.current_ki_q15);
}

/** @brief 打印Id测试结果。@param result 测试结果，不允许为空。@return 无。 */
static void commissioning_current_d_result_print(
  const motor_current_loop_test_result_t *result)
{
  LOGI("Commissioning Id: PASS average=%ld/%ld mA peak=%ld/%ld mA voltage=%ld/%ld mV samples=%lu\r\n",
    result->direct_average_ma, result->quadrature_average_ma,
    result->direct_peak_ma, result->quadrature_peak_ma,
    result->direct_voltage_mv, result->quadrature_voltage_mv,
    result->sample_count);
}

/** @brief 打印Iq和接管测试结果。@param result 测试结果，不允许为空。@return 无。 */
static void commissioning_current_q_result_print(
  const motor_torque_loop_test_result_t *result)
{
  LOGI("Commissioning Iq/handover: PASS average=%ld/%ld mA peak=%ld/%ld mA voltage=%ld/%ld mV frequency=%lu mHz samples=%lu\r\n",
    result->direct_average_ma, result->quadrature_average_ma,
    result->direct_peak_ma, result->quadrature_peak_ma,
    result->final_direct_voltage_mv, result->final_quadrature_voltage_mv,
    result->final_frequency_millihz, result->sample_count);
}

void motor_commissioning_hall_edge_process(void)
{
  motor_hall_port_sample_t sample; motor_open_loop_status_t open;
  uint8_t state; bool enough=true;
  if (!commissioning_hall_collecting ||
      !motor_hall_port_sample_read(&sample) || (sample.state==0U) || (sample.state==7U) ||
      !motor_open_loop_status_read(&open)) return;
  if (sample.state == commissioning_hall_previous_state) return;
  if (motor_hall_calibration_edge_add(&commissioning_hall,
      commissioning_hall_previous_state, sample.state,
      open.electrical_angle_u16, sample.timestamp_cycles) != 0)
    commissioning_hall_collecting=false;
  commissioning_hall_previous_state=sample.state;
  for(state=1U;state<=6U;state++)
  {
    uint32_t count=(commissioning_hall.state==MOTOR_HALL_CALIBRATION_FORWARD_SCAN)?
      commissioning_hall.forward_angle[state].count:commissioning_hall.reverse_angle[state].count;
    if(count<MOTOR_COMMISSIONING_HALL_SAMPLES) enough=false;
  }
  if(enough) commissioning_hall_collecting=false;
}

static bool commissioning_hall_direction_run(int8_t direction)
{
  motor_open_loop_command_t command; motor_hall_port_sample_t sample;
  uint32_t start=motor_timebase_cycles_get(); uint32_t timeout=(system_core_clock/1000U)*MOTOR_COMMISSIONING_HALL_TIMEOUT_MS;
  if ((motor_hall_calibration_scan_begin(&commissioning_hall,direction) !=
       MOTOR_HALL_CALIBRATION_OK) ||
      !motor_hall_port_sample_read(&sample)) return false;
  commissioning_hall_previous_state=sample.state; commissioning_hall_collecting=true;
  command.direct_voltage_mv=0; command.quadrature_voltage_mv=MOTOR_COMMISSIONING_HALL_VOLTAGE_MV;
  command.target_frequency_millihz=direction*MOTOR_COMMISSIONING_HALL_FREQUENCY_MILLIHZ;
  command.acceleration_millihz_per_s=MOTOR_COMMISSIONING_HALL_FREQUENCY_MILLIHZ;
  if (!motor_control_open_loop_start(&command)) { commissioning_hall_collecting=false; return false; }
  while (commissioning_hall_collecting && !commissioning_abort_requested &&
         motor_timebase_cycles_elapsed(start)<timeout) wk_delay_ms(1U);
  commissioning_hall_collecting=false; motor_control_stop();
  return !commissioning_abort_requested &&
    (commissioning_hall.state != MOTOR_HALL_CALIBRATION_FAULT) &&
    (motor_timebase_cycles_elapsed(start)<timeout) &&
    (motor_hall_calibration_scan_end(&commissioning_hall)==0);
}

static bool commissioning_hall_run(void)
{
  motor_hall_calibration_config_t config={MOTOR_COMMISSIONING_HALL_SAMPLES,7000U,15000U,1200U,2000U,0U};
  motor_hall_calibration_result_t result; motor_parameter_t p;
  /* ISR在每个状态达到样本数后仍会继续采集，因此使用固定扫描时间形成多圈样本。 */
  if (motor_hall_calibration_init(&commissioning_hall,&config)!=0) return false;
  commissioning_hall_collecting=true;
  /* direction_run以超时作为失败，故由边沿处理在全部状态达到门槛时结束采集。 */
  if (!commissioning_hall_direction_run(1) || !commissioning_hall_direction_run(-1) ||
      !motor_hall_calibration_result_get(&commissioning_hall,&result) ||
      !motor_parameter_candidate_read(&p)) return false;
  for (uint8_t i=0U;i<8U;i++){p.hall_positive_next[i]=result.positive_next[i];p.hall_entry_angle_u16[i]=result.entry_angle_u16[i];}
  if (!motor_parameter_candidate_set(&p)) return false;
  LOGI("Commissioning Hall: PASS max_dev=%u reverse_error=%u candidate=1\r\n",
    result.maximum_edge_deviation_u16,result.maximum_forward_reverse_error_u16);
  return true;
}

bool motor_commissioning_run(motor_commissioning_task_t task)
{
  bool ok=false; motor_current_loop_test_result_t d_result;
  motor_torque_loop_test_result_t q_result;
  if ((task==MOTOR_COMMISSIONING_TASK_NONE) || !commissioning_ready() ||
      (commissioning_status.state==MOTOR_COMMISSIONING_RUNNING)) return false;
  commissioning_status.state=MOTOR_COMMISSIONING_RUNNING;
  commissioning_status.task=task; commissioning_status.step=0U;
  commissioning_status.error=0U; commissioning_abort_requested=false;
  LOGI("Commissioning: started task=%u\r\n",(unsigned int)task);
  commissioning_step_set(1U);
  if (task==MOTOR_COMMISSIONING_TASK_CURRENT_OFFSET) ok=commissioning_offset_run();
  else if(task==MOTOR_COMMISSIONING_TASK_RESISTANCE) ok=commissioning_resistance_run();
  else if(task==MOTOR_COMMISSIONING_TASK_INDUCTANCE) ok=commissioning_inductance_run();
  else if(task==MOTOR_COMMISSIONING_TASK_HALL) ok=commissioning_hall_run();
  else if(task==MOTOR_COMMISSIONING_TASK_CURRENT_PI) ok=commissioning_pi_run();
  else if(task==MOTOR_COMMISSIONING_TASK_CURRENT_D)
  {
    ok=commissioning_current_d_run(&d_result);
    if(ok) commissioning_current_d_result_print(&d_result);
  }
  else if((task==MOTOR_COMMISSIONING_TASK_CURRENT_Q)||(task==MOTOR_COMMISSIONING_TASK_CURRENT_HANDOVER))
  {
    ok=commissioning_current_q_run(&q_result);
    if(ok) commissioning_current_q_result_print(&q_result);
  }
  else if(task==MOTOR_COMMISSIONING_TASK_FULL)
  {
    ok=commissioning_offset_run();
    if (ok) { commissioning_step_set(2U); ok=commissioning_resistance_run(); }
    if (ok) { commissioning_step_set(3U); ok=commissioning_inductance_run(); }
    if (ok) { commissioning_step_set(4U); ok=commissioning_hall_run(); }
    if (ok) { commissioning_step_set(5U); ok=commissioning_pi_run(); }
    if (ok)
    {
      commissioning_step_set(6U);
      ok=commissioning_candidate_trial_run(&d_result,&q_result);
      if(ok)
      {
        commissioning_current_d_result_print(&d_result);
        commissioning_current_q_result_print(&q_result);
      }
    }
  }
  motor_control_stop(); commissioning_hall_collecting=false;
  commissioning_status.run_count++;
  if(ok && !commissioning_abort_requested)
  {
    commissioning_status.state=MOTOR_COMMISSIONING_WAIT_ACCEPT;
    LOGI("Commissioning: PASS task=%u run=%lu, review diff then accept/discard\r\n",
      (unsigned int)task,commissioning_status.run_count);
    return true;
  }
  commissioning_status.state=MOTOR_COMMISSIONING_FAULT;
  commissioning_status.error=commissioning_abort_requested?1U:2U;
  LOGE("Commissioning: FAIL task=%u error=%lu\r\n",
    (unsigned int)task,commissioning_status.error);
  return false;
}

bool motor_commissioning_abort(void)
{
  if(commissioning_status.state!=MOTOR_COMMISSIONING_RUNNING) return false;
  commissioning_abort_requested=true; commissioning_hall_collecting=false;
  motor_pwm_port_emergency_stop(); return true;
}

bool motor_commissioning_status_read(motor_commissioning_status_t *status)
{
  uint32_t primask;
  if(status==NULL) return false;
  primask=__get_PRIMASK();__disable_irq();*status=commissioning_status;__set_PRIMASK(primask);
  return true;
}

bool motor_commissioning_review_complete(bool accepted)
{
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  if (commissioning_status.state != MOTOR_COMMISSIONING_WAIT_ACCEPT)
  {
    __set_PRIMASK(primask);
    return false;
  }
  commissioning_status.state = accepted ? MOTOR_COMMISSIONING_COMPLETE :
                                          MOTOR_COMMISSIONING_IDLE;
  commissioning_status.task = accepted ? commissioning_status.task :
                                          MOTOR_COMMISSIONING_TASK_NONE;
  commissioning_status.step = 0U;
  commissioning_status.error = 0U;
  __set_PRIMASK(primask);
  return true;
}
