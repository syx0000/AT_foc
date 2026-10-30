#include "mc_lib.h"
#include "mc_can_motor.h"

static volatile uint32_t can_ms;
static uint32_t last_command_ms;
static uint32_t last_feedback_ms;
static uint8_t can_controls_motor;
static uint8_t feedback_pending;

void mc_can_motor_tick(void)
{
  can_ms++;
}

void mc_can_motor_init(void)
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

  /* HEXT = 8 MHz; AT32 bit time = divider * (BTS1 + BTS2).
     8 MHz / (1 * (6 + 2)) = 1 Mbit/s, sample point 75%. */
  crm_can_clock_select(CRM_CAN1, CRM_CAN_CLOCK_SOURCE_HEXT);
  can_software_reset(CAN1, TRUE);
  can_bittime_default_para_init(&timing);
  timing.bittime_div = 1;
  timing.ac_bts1_size = 6;
  timing.ac_bts2_size = 2;
  timing.ac_rsaw_size = 2;
  can_bittime_set(CAN1, &timing);
  can_software_reset(CAN1, FALSE);
  can_retransmission_limit_set(CAN1, CAN_RE_TRANS_TIMES_2);
  can_rearbitration_limit_set(CAN1, CAN_RE_ARBI_TIMES_2);
  can_mode_set(CAN1, CAN_MODE_COMMUNICATE);
  can_receive_all_enable(CAN1, TRUE);
  can_rxbuf_overflow_mode_set(CAN1, CAN_RXBUF_OVERFLOW_BE_LOSE);
}

static void motor_stop(void)
{
  start_stop_btn_flag = RESET;
  speed_ramp.cmd_final = 0;
  speed_ramp.command = 0;
  current.Iqdref.q = 0;
  pid_spd.integral = 0;
}

static uint8_t apply_command(const can_rxbuf_type *rx)
{
  uint32_t raw;
  int32_t rpm;
  int16_t limit;
  int32_t integral_limit;

  if ((rx->id_type != CAN_ID_STANDARD) || (rx->id != MC_CAN_NODE_ID) ||
      (rx->frame_type != CAN_FRAME_DATA))
    return 0;
#ifdef SUPPORT_CAN_FD
  if (rx->fd_format != CAN_FORMAT_CLASSIC)
    return 0;
#endif
  if ((rx->data_length == CAN_DLC_BYTES_1) && (rx->data[0] == 0x00))
  {
    feedback_pending = 3;
    return 1;
  }
  if ((rx->data[0] == 0x02) &&
      ((rx->data_length == CAN_DLC_BYTES_5) ||
       (rx->data_length == CAN_DLC_BYTES_8)))
  {
    raw = ((uint32_t)rx->data[1] << 24) | ((uint32_t)rx->data[2] << 16) |
          ((uint32_t)rx->data[3] << 8) | rx->data[4];
    /* Match AT_foc_hall: clamp signed speed commands to the configured maximum. */
    rpm = (int32_t)raw / 4;
    if (rpm > MAX_SPEED_RPM) rpm = MAX_SPEED_RPM;
    else if (rpm < -MAX_SPEED_RPM) rpm = -MAX_SPEED_RPM;
    if (rpm == 0)
    {
      motor_stop();
      can_controls_motor = 0;
      return 1;
    }
    /* Never acknowledge/reset a fault or interrupt calibration via a speed frame. */
    if ((error_code != MC_NO_ERROR) ||
        ((esc_state != ESC_STATE_SAFETY_READY) && (esc_state != ESC_STATE_RUNNING)) ||
        (ctrl_mode != SPEED_CTRL) || (ctrl_mode_cmd != SPEED_CTRL))
      return 0;
    speed_ramp.cmd_final = rpm;
    start_stop_btn_flag = SET;
    can_controls_motor = 1;
    return 1;
  }
  if ((rx->data_length == CAN_DLC_BYTES_3) && (rx->data[0] == 0x14))
  {
    raw = ((uint32_t)rx->data[1] << 8) | rx->data[2];
    /* Wire unit 10 mA, constrained by the configured board maximum. */
    if ((float)raw > MAX_CURRENT * 100.0f)
      raw = (uint32_t)(MAX_CURRENT * 100.0f);
    limit = (int16_t)((float)raw * 32767.0f / (CURRENT_BASE * 100.0f));
    current.max_current = limit;
    current.min_current = -limit;
    pid_spd.upper_limit_output = limit;
    pid_spd.lower_limit_output = -limit;
    integral_limit = (int32_t)((int64_t)limit * ((int64_t)1 << pid_spd.ki_shift));
    pid_spd.upper_limit_integral = integral_limit;
    pid_spd.lower_limit_integral = -integral_limit;
    if (pid_spd.integral > integral_limit) pid_spd.integral = integral_limit;
    if (pid_spd.integral < -integral_limit) pid_spd.integral = -integral_limit;
    if (current.Iqdref.q > limit) current.Iqdref.q = limit;
    if (current.Iqdref.q < -limit) current.Iqdref.q = -limit;
    return 1;
  }
  return 0;
}

static void send_feedback(void)
{
  can_txbuf_type tx = {0};
  uint32_t value;
  int32_t amps;
  if (feedback_pending == 0) return;
  tx.id = MC_CAN_NODE_ID;
  tx.id_type = CAN_ID_STANDARD;
  tx.frame_type = CAN_FRAME_DATA;
  tx.data_length = CAN_DLC_BYTES_4;
  if (feedback_pending == 3)
  {
    tx.data[1] = 0x00;
    value = (uint32_t)error_code;
    tx.data[2] = (uint8_t)(value >> 8);
    tx.data[3] = (uint8_t)value;
  }
  else if (feedback_pending == 2)
  {
    tx.data[1] = 0x01;
    tx.data_length = CAN_DLC_BYTES_6;
    value = (uint32_t)(rotor_speed_val * 4);
    tx.data[2] = (uint8_t)(value >> 24);
    tx.data[3] = (uint8_t)(value >> 16);
    tx.data[4] = (uint8_t)(value >> 8);
    tx.data[5] = (uint8_t)value;
  }
  else
  {
    tx.data[1] = 0x05;
    /* ZXQ's 16-bit current reader is unsigned: report torque-current magnitude. */
    amps = Ival.q;
    if (amps < 0) amps = -amps;
    value = (uint32_t)((float)amps * CURRENT_BASE * 100.0f / 32767.0f);
    tx.data[2] = (uint8_t)(value >> 8);
    tx.data[3] = (uint8_t)value;
  }
  if (can_txbuf_write(CAN1, CAN_TXBUF_PTB, &tx) == SUCCESS)
  {
    if (can_txbuf_transmit(CAN1, CAN_TRANSMIT_PTB) == SUCCESS)
      feedback_pending--;
  }
}

void mc_can_motor_task(void)
{
  can_rxbuf_type rx;
  uint8_t budget = 8;
  uint32_t primask;
  while ((budget-- != 0) && (can_rxbuf_status_get(CAN1) != CAN_RXBUF_STATUS_EMPTY))
  {
    if (can_rxbuf_read(CAN1, &rx) != SUCCESS) break;
    /* can_rxbuf_read() already releases this frame in the AT32 driver. */
    if (rx.data_length == CAN_DLC_BYTES_0) continue;
    /* Update shared PI/state atomically against the PWM/speed/SysTick IRQs. */
    primask = __get_PRIMASK();
    __disable_irq();
    if (apply_command(&rx)) last_command_ms = can_ms;
    __set_PRIMASK(primask);
  }
  if (can_controls_motor && ((uint32_t)(can_ms - last_command_ms) >= MC_CAN_TIMEOUT_MS))
  {
    primask = __get_PRIMASK();
    __disable_irq();
    motor_stop();
    can_controls_motor = 0;
    __set_PRIMASK(primask);
  }
  if ((feedback_pending == 0) &&
      ((uint32_t)(can_ms - last_feedback_ms) >= MC_CAN_FEEDBACK_MS))
  {
    feedback_pending = 3;
    last_feedback_ms = can_ms;
  }
  send_feedback();
}
