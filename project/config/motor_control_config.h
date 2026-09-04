#ifndef MOTOR_CONTROL_CONFIG_H
#define MOTOR_CONTROL_CONFIG_H

/* ADC与电流采样硬件参数。 */
#define MOTOR_ADC_FULL_SCALE_COUNTS          4095U
#define MOTOR_ADC_REFERENCE_UV            3300000UL
#define MOTOR_CURRENT_SHUNT_UOHM              2500U
#define MOTOR_CURRENT_CSA_GAIN                   10U
/* 实测确认：相电流正方向对应ADC值相对零偏减小。 */
#define MOTOR_CURRENT_ADC_POLARITY_POSITIVE       0U

/* 电流零偏校准判据。 */
#define MOTOR_CURRENT_OFFSET_NOMINAL_COUNTS    2048U
#define MOTOR_CURRENT_OFFSET_TOLERANCE_COUNTS   160U
#define MOTOR_CURRENT_OFFSET_DIFFERENCE_MAX      80U

/* 电机能力、控制限幅与保护参数，单位均为mA。 */
#define MOTOR_RATED_CONTINUOUS_CURRENT_MA      30000L
#define MOTOR_CURRENT_COMMAND_MAX_MA           50000L
#define MOTOR_SOFTWARE_OVERCURRENT_MA          55000L
#define MOTOR_SOFTWARE_OVERCURRENT_SAMPLES       500U
/* FOC定点标幺基准独立于指令限幅；128 A对应Q15的1.0，1 A正好对应256。 */
#define MOTOR_CURRENT_Q15_BASE_MA             128000L

/* 电机本体参数：4对极，机械转速=rpm=(电频率Hz×60)/极对数。 */
#define MOTOR_POLE_PAIRS                           4U
/* 设备逻辑方向默认与当前电气正方向一致；1表示逻辑方向反转。 */
#define MOTOR_DIRECTION_INVERTED                   0U

/* 母线电压参数，保护阈值单位为0.1 V。 */
#define MOTOR_BUS_VOLTAGE_DIVIDER_RATIO           21U
#define MOTOR_BUS_UNDERVOLTAGE_0P1V               240U
#define MOTOR_BUS_OVERVOLTAGE_0P1V                600U

/* 温度传感器与保护参数，温度单位为摄氏度。 */
#define MOTOR_MOS_NTC_R25_OHM                   10000U
#define MOTOR_MOS_NTC_B_K                        3950U
#define MOTOR_WINDING_NTC_R25_OHM               10000U
#define MOTOR_WINDING_NTC_B_K                    3435U
#define MOTOR_TEMPERATURE_WARNING_C                90
#define MOTOR_TEMPERATURE_SHUTDOWN_C              100
#define MOTOR_MOS_TEMPERATURE_WARNING_C             90
#define MOTOR_MOS_TEMPERATURE_SHUTDOWN_C           100

/* 10 Hz空载正向开环实测Hall状态进入边沿角度，U16一周制。 */
#define MOTOR_HALL_POSITIVE_NEXT_STATE_1             5U
#define MOTOR_HALL_POSITIVE_NEXT_STATE_2             3U
#define MOTOR_HALL_POSITIVE_NEXT_STATE_3             1U
#define MOTOR_HALL_POSITIVE_NEXT_STATE_4             6U
#define MOTOR_HALL_POSITIVE_NEXT_STATE_5             4U
#define MOTOR_HALL_POSITIVE_NEXT_STATE_6             2U
#define MOTOR_HALL_EDGE_ANGLE_STATE_1_U16         51938U
#define MOTOR_HALL_EDGE_ANGLE_STATE_2_U16         30960U
#define MOTOR_HALL_EDGE_ANGLE_STATE_3_U16         40216U
#define MOTOR_HALL_EDGE_ANGLE_STATE_4_U16          7705U
#define MOTOR_HALL_EDGE_ANGLE_STATE_5_U16         63928U
#define MOTOR_HALL_EDGE_ANGLE_STATE_6_U16         19215U
/* 由35 Hz闭环平均Vd/Vq=-2910/3333 mV计算，补偿开环标定引入的41.1度超前角。 */
#define MOTOR_HALL_ROTOR_ANGLE_OFFSET_U16           7482U
#define MOTOR_HALL_SIGNAL_TIMEOUT_MS                100U

/* Hall机械转速反馈：1 kHz更新，IIR滤波系数为1/8。 */
#define MOTOR_SPEED_FEEDBACK_TIMEOUT_MS              100U
#define MOTOR_SPEED_FEEDBACK_FILTER_SHIFT              3U

/* 速度外环固定以1 kHz执行；PI输出为现有电流环的Iq指令。 */
#define MOTOR_SPEED_LOOP_FREQUENCY_HZ                1000U
#define MOTOR_SPEED_PI_KP_Q20                         5243L
#define MOTOR_SPEED_PI_KI_Q20                           33L
#define MOTOR_SPEED_CONTROL_CURRENT_LIMIT_MA          2000L
#define MOTOR_SPEED_CONTROL_MAXIMUM_SPEED_RPM         3000L
#define MOTOR_SPEED_CONTROL_ACCELERATION_RPM_PER_S     300U
/* 速度闭环保护阈值：当前仅支持正向Hall角度标定。 */
#define MOTOR_SPEED_STALL_MIN_TARGET_RPM               200U
#define MOTOR_SPEED_STALL_MAX_FEEDBACK_RPM              50U
#define MOTOR_SPEED_STALL_CURRENT_PERCENT               90U
#define MOTOR_SPEED_STALL_TIMEOUT_MS                   2000U
#define MOTOR_SPEED_OVERSPEED_RPM                      3300U

/* 正式开环控制器默认安全配置。 */
#define MOTOR_OPEN_LOOP_MAXIMUM_VOLTAGE_MV          4800L
#define MOTOR_OPEN_LOOP_MAXIMUM_FREQUENCY_MILLIHZ 100000L
#define MOTOR_OPEN_LOOP_START_FREQUENCY_MILLIHZ     2000L
#define MOTOR_OPEN_LOOP_ALIGNMENT_VOLTAGE_MV        1440L
#define MOTOR_OPEN_LOOP_ALIGNMENT_ANGLE_U16            0U
#define MOTOR_OPEN_LOOP_ALIGNMENT_TIME_MS             500U

/* 阻塞式相电阻辨识参数。 */
#define MOTOR_RESISTANCE_IDENT_TARGET_CURRENT_MA   3000L
#define MOTOR_RESISTANCE_IDENT_ABORT_CURRENT_MA   10000L
#define MOTOR_RESISTANCE_IDENT_MAX_VOLTAGE_MV      1000U
#define MOTOR_RESISTANCE_IDENT_VOLTAGE_STEP_MV        1U
#define MOTOR_RESISTANCE_IDENT_SETTLE_MS            200U
#define MOTOR_RESISTANCE_IDENT_MEASURE_MS           1000U
#define MOTOR_RESISTANCE_IDENT_MIN_CURRENT_MA        500L

/* 阻塞式dq轴电感辨识参数。 */
#define MOTOR_INDUCTANCE_IDENT_FREQUENCY_HZ           600U
#define MOTOR_INDUCTANCE_IDENT_VOLTAGE_MV             500U
#define MOTOR_INDUCTANCE_IDENT_SETTLE_SAMPLES        2000U
#define MOTOR_INDUCTANCE_IDENT_MEASURE_SAMPLES      10000U
#define MOTOR_INDUCTANCE_IDENT_ABORT_CURRENT_MA     10000L

/* 首版电流PI：200 Hz带宽，参数来自Rs=273 mΩ、Ld=225 µH、Lq=206 µH。 */
#define MOTOR_PHASE_RESISTANCE_MOHM                   273U
#define MOTOR_DIRECT_INDUCTANCE_UH                    225U
#define MOTOR_QUADRATURE_INDUCTANCE_UH                206U
#define MOTOR_CURRENT_LOOP_BANDWIDTH_HZ               200U
#define MOTOR_CURRENT_PI_D_KP_Q15                    9265L
#define MOTOR_CURRENT_PI_Q_KP_Q15                    8482L
#define MOTOR_CURRENT_PI_KI_Q15                      1124L
#define MOTOR_CURRENT_PI_OUTPUT_LIMIT_MV             4800L
/* 正常调制度当前取15%；现有线性控制允许最大50%，理论SVPWM边界约57.7%。 */
#define MOTOR_CURRENT_CONTROL_VOLTAGE_LIMIT_PERCENT    15U
/* 48 V母线在线性SVPWM理论边界Vbus/sqrt(3)下的dq电压矢量硬上限。 */
#define MOTOR_CURRENT_CONTROL_HARD_VOLTAGE_LIMIT_MV 27700L
#define MOTOR_CURRENT_CONTROL_ABORT_CURRENT_MA       50000L
/* 调试CLI初期限制，正式通信协议接入状态机后再按权限放宽。 */
#define MOTOR_CLI_CURRENT_COMMAND_LIMIT_MA             2000L
#define MOTOR_CURRENT_LOOP_TEST_DIRECT_REFERENCE_MA  2000L
#define MOTOR_CURRENT_LOOP_TEST_DURATION_SAMPLES    10000U
#define MOTOR_CURRENT_LOOP_TEST_ABORT_CURRENT_MA    10000L

/* Hall角度闭环转矩测试参数：第三级使用1 A、5秒观察15%限压下的空载转速。 */
#define MOTOR_TORQUE_TEST_QUADRATURE_REFERENCE_MA    1000L
#define MOTOR_TORQUE_TEST_REFERENCE_RAMP_SAMPLES     2000U
#define MOTOR_TORQUE_TEST_DURATION_SAMPLES          50000U
#define MOTOR_TORQUE_TEST_ABORT_CURRENT_MA           10000L
#define MOTOR_TORQUE_TEST_BOOTSTRAP_TIMEOUT_MS        6000U
#define MOTOR_TORQUE_TEST_HANDOVER_FREQUENCY_MILLIHZ 8000U
#define MOTOR_TORQUE_TEST_BUS_VOLTAGE_LIMIT_PERCENT     15U
#define MOTOR_TORQUE_TEST_ABSOLUTE_VOLTAGE_LIMIT_MV   7200L

/* 编译期配置一致性检查：配置错误必须在生成固件前暴露。 */
#if (MOTOR_POLE_PAIRS == 0U)
#error "MOTOR_POLE_PAIRS must be greater than zero"
#endif

#if (MOTOR_DIRECTION_INVERTED > 1U)
#error "MOTOR_DIRECTION_INVERTED must be 0 or 1"
#endif

#if (MOTOR_SPEED_LOOP_FREQUENCY_HZ != 1000U)
#error "motor_speed_control is designed for a 1 kHz update rate"
#endif

#if ((MOTOR_CURRENT_CONTROL_VOLTAGE_LIMIT_PERCENT == 0U) || \
     (MOTOR_CURRENT_CONTROL_VOLTAGE_LIMIT_PERCENT > 50U))
#error "current control modulation limit must be within 1..50 percent"
#endif

#if ((MOTOR_CURRENT_CONTROL_HARD_VOLTAGE_LIMIT_MV <= 0L) || \
     (MOTOR_CURRENT_CONTROL_HARD_VOLTAGE_LIMIT_MV > 27700L))
#error "48 V linear SVPWM hard voltage limit must not exceed 27700 mV"
#endif

#if ((MOTOR_SPEED_CONTROL_CURRENT_LIMIT_MA <= 0L) || \
     (MOTOR_SPEED_CONTROL_CURRENT_LIMIT_MA > MOTOR_CURRENT_COMMAND_MAX_MA))
#error "speed control current limit exceeds current-loop command limit"
#endif

#if (MOTOR_SPEED_STALL_CURRENT_PERCENT > 100U)
#error "stall current percentage must not exceed 100 percent"
#endif

#if (MOTOR_SPEED_STALL_MIN_TARGET_RPM >= MOTOR_SPEED_CONTROL_MAXIMUM_SPEED_RPM)
#error "stall target threshold must be below maximum command speed"
#endif

#if (MOTOR_SPEED_OVERSPEED_RPM <= MOTOR_SPEED_CONTROL_MAXIMUM_SPEED_RPM)
#error "overspeed threshold must be above maximum command speed"
#endif

#endif /* MOTOR_CONTROL_CONFIG_H */
