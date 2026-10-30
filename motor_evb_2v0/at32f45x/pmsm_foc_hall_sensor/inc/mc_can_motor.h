#ifndef MC_CAN_MOTOR_H
#define MC_CAN_MOTOR_H

/* ZXQ standard CAN ID: build one motor as 1 (left), the other as 2 (right). */
#ifndef MC_CAN_NODE_ID
#define MC_CAN_NODE_ID 1
#endif
#if (MC_CAN_NODE_ID < 1) || (MC_CAN_NODE_ID > 127)
#error "MC_CAN_NODE_ID must be 1..127"
#endif
/* ZXQ sends a query every 500 ms. Loss of all valid commands stops the motor. */
#define MC_CAN_TIMEOUT_MS 1500U
#define MC_CAN_FEEDBACK_MS 10U
void mc_can_motor_serial_takeover(void);
void mc_can_motor_init(void);
void mc_can_motor_task(void);
void mc_can_motor_tick(void);
#endif
