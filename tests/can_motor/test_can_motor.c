/* Compile the actual firmware module with the host stand-ins. */
#include "../../motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/src/mc_can_motor.c"

/* Freestanding host executable: no CRT/Windows SDK needed. */
int _fltused;
void *memset(void *p, int c, unsigned long long n) {
  volatile unsigned char *q = p; while (n--) *q++ = (unsigned char)c; return p;
}
void *memcpy(void *d, const void *s, unsigned long long n) {
  volatile unsigned char *q = d; const unsigned char *p = s;
  while (n--) *q++ = *p++; return d;
}
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)

int main(void)
{
  can_rxbuf_type rx = {0};
  rx.id = MC_CAN_NODE_ID;
  rx.data_length = 5;
  rx.data[0] = 2;
  rx.data[3] = 7; rx.data[4] = 0xd0; /* +500 rpm */
  ctrl_mode = ctrl_mode_cmd = SPEED_CTRL;
  esc_state = ESC_STATE_SAFETY_READY;
  pid_spd.ki_shift = 15;
  CHECK(apply_command(&rx) && speed_ramp.cmd_final == 500 && start_stop_btn_flag);
  rx.data[1] = 0xff; rx.data[2] = 0xff; rx.data[3] = 0xf8; rx.data[4] = 0x30;
  CHECK(apply_command(&rx) && speed_ramp.cmd_final == -500);
  rx.id++;
  CHECK(!apply_command(&rx) && speed_ramp.cmd_final == -500);
  rx.id = MC_CAN_NODE_ID;
  rx.data_length = 4;
  CHECK(!apply_command(&rx));
  rx.data_length = 5; rx.frame_type = 1;
  CHECK(!apply_command(&rx));
  rx.frame_type = 0; rx.fd_format = 1;
  CHECK(!apply_command(&rx));
  rx.fd_format = 0;
  rx.data[1] = 0; rx.data[2] = 0; rx.data[3] = 0x3e; rx.data[4] = 0x84; /* 4001 */
  CHECK(!apply_command(&rx));
  rx.data[3] = 7; rx.data[4] = 0xd0;
  error_code = 8;
  CHECK(!apply_command(&rx) && error_code == 8);
  error_code = 0;
  rx.data_length = 3; rx.data[0] = 0x14; rx.data[1] = 1; rx.data[2] = 0xf4;
  pid_spd.integral = 1000000000;
  current.Iqdref.q = 30000;
  CHECK(apply_command(&rx));
  CHECK(current.max_current == 2482 && current.min_current == -2482); /* 5 A */
  CHECK(current.Iqdref.q == 2482 && pid_spd.integral == 2482 * 32768);
  rx.data[1] = rx.data[2] = 0xff;
  CHECK(apply_command(&rx) && current.max_current == 24823); /* cap 50 A */
  rx.data_length = 8; rx.data[0] = 2;
  rx.data[1] = rx.data[2] = rx.data[3] = rx.data[4] = 0;
  CHECK(apply_command(&rx) && !start_stop_btn_flag && !speed_ramp.command);
  rx.data_length = 5; rx.data[3] = 7; rx.data[4] = 0xd0;
  CHECK(apply_command(&rx));
  last_command_ms = can_ms;
  can_ms += MC_CAN_TIMEOUT_MS - 1;
  mc_can_motor_task(); CHECK(start_stop_btn_flag);
  mc_can_motor_tick(); mc_can_motor_task();
  CHECK(!start_stop_btn_flag && !can_controls_motor);
  /* Isolate the subsequent query-response checks from periodic telemetry. */
  feedback_pending = 0; tx_count = 0; last_feedback_ms = can_ms;
  /* Consecutive frames must both be consumed, without a second release. */
  rx_queue[0] = rx;
  rx_queue[1] = rx; rx_queue[1].data[3] = 0; rx_queue[1].data[4] = 0;
  rx_tail = 2;
  mc_can_motor_task(); CHECK(rx_head == 2 && !start_stop_btn_flag);
  rx.data_length = 1; rx.data[0] = 0;
  CHECK(apply_command(&rx));
  tx_busy = 1; mc_can_motor_task(); CHECK(feedback_pending == 3);
  tx_busy = 0; rotor_speed_val = -500; Ival.q = -2482;
  mc_can_motor_task(); mc_can_motor_task(); mc_can_motor_task();
  CHECK(tx_count == 3 && feedback_pending == 0);
  CHECK(tx_queue[0].data[1] == 0);
  CHECK(tx_queue[1].data_length == 6 && tx_queue[1].data[1] == 1);
  CHECK(tx_queue[1].data[2] == 0xff && tx_queue[1].data[4] == 0xf8 && tx_queue[1].data[5] == 0x30);
  CHECK(tx_queue[2].data[1] == 5 && tx_queue[2].data[2] == 1);
  can_ms += MC_CAN_FEEDBACK_MS;
  mc_can_motor_task(); CHECK(tx_count == 4 && tx_queue[3].data[1] == 0);
  return 0;
}
