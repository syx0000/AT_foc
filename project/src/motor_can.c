/**
 * @file    motor_can.c
 * @brief   ZXQ CAN电机协议（移植自AT_foc_hall_demo mc_can_motor） 功能实现
 * @author  随永祥
 * @date    2026-10-24
 * @version 1.0
 * @details 协议与demo一致（标准帧，ID=MOTOR_CAN_NODE_ID）：
 *          接收 0x00(DLC1)        查询，回送一组反馈；
 *               0x02 int32(BE)    目标转速，单位0.25 rpm，0为停机；
 *               0x14 uint16(BE)   速度环Iq限幅，单位10 mA。
 *          发送 [0]=0,[1]=0x00 uint16 故障码（motor_fault_code_t）；
 *               [0]=0,[1]=0x01 int32  转速，单位0.25 rpm；
 *               [0]=0,[1]=0x05 uint16 |Iq|，单位10 mA。
 *          指令均在主循环执行，与CLI共用motor_control状态机和安全检查。
 */

#include "at32f45x.h"
#include "motor_can.h"
#include "motor_control.h"
#include "motor_control_config.h"
#include "motor_current_control.h"
#include "motor_speed_control.h"
#include "motor_speed_feedback.h"

#define MOTOR_CAN_RX_BUDGET 8U

static volatile uint32_t motor_can_ms;
static uint32_t motor_can_last_command_ms;
static uint32_t motor_can_last_feedback_ms;
static bool motor_can_controls_motor;
static uint8_t motor_can_feedback_pending;

void motor_can_tick(void)
{
  motor_can_ms++;
}

void motor_can_init(void)
{
  gpio_init_type gpio;
  can_bittime_type timing;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_CAN1_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio);
  gpio.gpio_mode = GPIO_MODE_MUX;
  gpio.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio.gpio_pull = GPIO_PULL_NONE;
  gpio.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio.gpio_pins = GPIO_PINS_11 | GPIO_PINS_12;
  gpio_init(GPIOA, &gpio);
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE11, GPIO_MUX_9);
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE12, GPIO_MUX_9);

  /* HEXT = 8 MHz；位时间 = div * (BTS1 + BTS2)。
   * 8 MHz / (1 * (12 + 4)) = 500 kbit/s，采样点75%。 */
  crm_can_clock_select(CRM_CAN1, CRM_CAN_CLOCK_SOURCE_HEXT);
  can_software_reset(CAN1, TRUE);
  can_bittime_default_para_init(&timing);
  timing.bittime_div = 1;
  timing.ac_bts1_size = 12;
  timing.ac_bts2_size = 4;
  timing.ac_rsaw_size = 2;
  can_bittime_set(CAN1, &timing);
  can_software_reset(CAN1, FALSE);
  can_retransmission_limit_set(CAN1, CAN_RE_TRANS_TIMES_2);
  can_rearbitration_limit_set(CAN1, CAN_RE_ARBI_TIMES_2);
  can_mode_set(CAN1, CAN_MODE_COMMUNICATE);
  can_receive_all_enable(CAN1, TRUE);
  can_rxbuf_overflow_mode_set(CAN1, CAN_RXBUF_OVERFLOW_BE_LOSE);
}

/** @brief 读取大端32位数。 */
static uint32_t motor_can_be32(const uint8_t *data)
{
  return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
         ((uint32_t)data[2] << 8) | (uint32_t)data[3];
}

/**
 * @brief 执行转速指令；超出最高转速时按最高转速执行。
 * @param rpm 有符号逻辑目标转速，0表示停机。
 * @return 指令被接受返回true。
 */
static bool motor_can_speed_apply(int32_t rpm)
{
  motor_control_status_t status;
  int32_t maximum_rpm = motor_speed_control_maximum_rpm_get();

  if (rpm == 0)
  {
    motor_control_stop();
    motor_can_controls_motor = false;
    return true;
  }
  if (rpm > maximum_rpm) rpm = maximum_rpm;
  else if (rpm < -maximum_rpm) rpm = -maximum_rpm;
  /* 速度帧不清故障、不打断辨识，只在READY或速度环运行时生效。 */
  if (!motor_control_status_read(&status)) return false;
  if (status.state == MOTOR_CONTROL_STATE_READY)
  {
    if (!motor_control_speed_control_start(rpm)) return false;
  }
  else if (status.state == MOTOR_CONTROL_STATE_SPEED_CONTROL)
  {
    if (!motor_control_speed_control_target_set(rpm)) return false;
  }
  else
  {
    return false;
  }
  motor_can_controls_motor = true;
  return true;
}

/**
 * @brief 解析一帧接收数据。
 * @param rx 接收帧。
 * @return 本节点有效指令返回true，用于刷新通信超时。
 */
static bool motor_can_command_apply(const can_rxbuf_type *rx)
{
  uint32_t raw;

  if ((rx->id_type != CAN_ID_STANDARD) || (rx->id != MOTOR_CAN_NODE_ID) ||
      (rx->frame_type != CAN_FRAME_DATA))
    return false;
#ifdef SUPPORT_CAN_FD
  if (rx->fd_format != CAN_FORMAT_CLASSIC) return false;
#endif
  if ((rx->data_length == CAN_DLC_BYTES_1) && (rx->data[0] == 0x00U))
  {
    motor_can_feedback_pending = 3U;
    return true;
  }
  if ((rx->data[0] == 0x02U) &&
      ((rx->data_length == CAN_DLC_BYTES_5) ||
       (rx->data_length == CAN_DLC_BYTES_8)))
  {
    raw = motor_can_be32(&rx->data[1]);
    return motor_can_speed_apply((int32_t)raw / 4);
  }
  if ((rx->data_length == CAN_DLC_BYTES_3) && (rx->data[0] == 0x14U))
  {
    raw = ((uint32_t)rx->data[1] << 8) | (uint32_t)rx->data[2];
    return motor_speed_control_current_limit_set((int32_t)(raw * 10U));
  }
  return false;
}

/** @brief 每次调用发送一帧待发反馈，发送成功后递减待发计数。 */
static void motor_can_feedback_send(void)
{
  can_txbuf_type tx = {0};
  motor_control_status_t control;
  motor_speed_feedback_t speed;
  motor_current_control_status_t current;
  uint32_t value;
  int32_t quadrature_ma;

  if (motor_can_feedback_pending == 0U) return;
  tx.id = MOTOR_CAN_NODE_ID;
  tx.id_type = CAN_ID_STANDARD;
  tx.frame_type = CAN_FRAME_DATA;
  tx.data_length = CAN_DLC_BYTES_4;
  if (motor_can_feedback_pending == 3U)
  {
    tx.data[1] = 0x00U;
    value = (motor_control_status_read(&control) &&
             (control.state == MOTOR_CONTROL_STATE_FAULT)) ?
            (uint32_t)control.fault_code : 0U;
    tx.data[2] = (uint8_t)(value >> 8);
    tx.data[3] = (uint8_t)value;
  }
  else if (motor_can_feedback_pending == 2U)
  {
    tx.data[1] = 0x01U;
    tx.data_length = CAN_DLC_BYTES_6;
    value = motor_speed_feedback_read(&speed) ?
      (uint32_t)((speed.filtered_speed_millirpm / 1000) * 4) : 0U;
    tx.data[2] = (uint8_t)(value >> 24);
    tx.data[3] = (uint8_t)(value >> 16);
    tx.data[4] = (uint8_t)(value >> 8);
    tx.data[5] = (uint8_t)value;
  }
  else
  {
    tx.data[1] = 0x05U;
    /* ZXQ按无符号16位读取电流：上报转矩电流绝对值。 */
    quadrature_ma = (motor_current_control_status_read(&current) &&
                     (current.state == MOTOR_CURRENT_CONTROL_RUNNING)) ?
                    current.quadrature_feedback_ma : 0;
    if (quadrature_ma < 0) quadrature_ma = -quadrature_ma;
    value = (uint32_t)quadrature_ma / 10U;
    tx.data[2] = (uint8_t)(value >> 8);
    tx.data[3] = (uint8_t)value;
  }
  if ((can_txbuf_write(CAN1, CAN_TXBUF_PTB, &tx) == SUCCESS) &&
      (can_txbuf_transmit(CAN1, CAN_TRANSMIT_PTB) == SUCCESS))
    motor_can_feedback_pending--;
}

void motor_can_task(void)
{
  can_rxbuf_type rx;
  uint8_t budget = MOTOR_CAN_RX_BUDGET;
  uint32_t now = motor_can_ms;

  while ((budget-- != 0U) &&
         (can_rxbuf_status_get(CAN1) != CAN_RXBUF_STATUS_EMPTY))
  {
    /* can_rxbuf_read()内部已释放该接收缓冲。 */
    if (can_rxbuf_read(CAN1, &rx) != SUCCESS) break;
    if (rx.data_length == CAN_DLC_BYTES_0) continue;
    if (motor_can_command_apply(&rx)) motor_can_last_command_ms = now;
  }
  if (motor_can_controls_motor &&
      ((uint32_t)(now - motor_can_last_command_ms) >= MOTOR_CAN_TIMEOUT_MS))
  {
    motor_control_stop();
    motor_can_controls_motor = false;
  }
  if ((motor_can_feedback_pending == 0U) &&
      ((uint32_t)(now - motor_can_last_feedback_ms) >= MOTOR_CAN_FEEDBACK_MS))
  {
    motor_can_feedback_pending = 3U;
    motor_can_last_feedback_ms = now;
  }
  motor_can_feedback_send();
}
