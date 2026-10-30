#ifndef MOTOR_PROFILE_60BLDC140_24V_H
#define MOTOR_PROFILE_60BLDC140_24V_H
/* 60BLDC140-24030-14J30: datasheet PH-PH values: 160 mOhm / 100 uH.
 * FOC equivalent-star initial values use half the terminal measurements.
 * Provisional SPMSM assumption Ld=Lq. Confirm Rs/Ld/Lq on the actual motor.
 * Hall/offset/speed PI are retained. Inherited flux is the current initial
 * value for the new motor and can be replaced by later identification. */
#define MOTOR_24V_PROFILE_POLE_PAIRS       4U
#define MOTOR_24V_PROFILE_RS_MOHM         80U
#define MOTOR_24V_PROFILE_LD_UH           50U
#define MOTOR_24V_PROFILE_LQ_UH           50U
#define MOTOR_24V_PROFILE_BANDWIDTH_HZ   888U
#define MOTOR_24V_PROFILE_MAXIMUM_RPM   3000U
#define MOTOR_24V_PROFILE_CURRENT_MA   35000U
#define MOTOR_24V_PROFILE_ACCEL_RPM_S  30000U
#define MOTOR_24V_PROFILE_DECEL_RPM_S  30000U
#define MOTOR_24V_PROFILE_IQ_SLEW_MA_S 200000U
#endif
