#ifndef MOTOR_PROFILE_60BLDC140_24V_H
#define MOTOR_PROFILE_60BLDC140_24V_H
/* 60BLDC140-24030-14J30: latest board active values:
 * phase Rs=86 mOhm, Ld=71 uH, Lq=60 uH, current bandwidth=2000 Hz.
 * Hall/offset/speed PI are retained. Inherited flux is the current initial
 * value for the new motor and can be replaced by later identification. */
#define MOTOR_24V_PROFILE_POLE_PAIRS       4U
#define MOTOR_24V_PROFILE_RS_MOHM         86U
#define MOTOR_24V_PROFILE_LD_UH           71U
#define MOTOR_24V_PROFILE_LQ_UH           60U
#define MOTOR_24V_PROFILE_BANDWIDTH_HZ   2000U
#define MOTOR_24V_PROFILE_MAXIMUM_RPM   4000U
#define MOTOR_24V_PROFILE_CURRENT_MA   60000U
#define MOTOR_24V_PROFILE_ACCEL_RPM_S  30000U
#define MOTOR_24V_PROFILE_DECEL_RPM_S  30000U
#define MOTOR_24V_PROFILE_IQ_SLEW_MA_S 200000U
#endif
