/**
 * @file    motor_control_config.h
 * @brief   电机控制参数配置 公共数据结构和接口声明
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

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
#define MOTOR_CURRENT_COMMAND_MAX_MA           50000L
#define MOTOR_SOFTWARE_OVERCURRENT_MA          55000L
#define MOTOR_SOFTWARE_OVERCURRENT_SAMPLES       800U /* 50 ms @16 kHz */
/* FOC定点标幺基准独立于指令限幅；128 A对应Q15的1.0，1 A正好对应256。 */
#define MOTOR_CURRENT_Q15_BASE_MA             128000L

/* 电机本体参数：4对极，机械转速=rpm=(电频率Hz×60)/极对数。 */
#define MOTOR_POLE_PAIRS                           4U
/* 设备逻辑方向默认与当前电气正方向一致；1表示逻辑方向反转。 */
#define MOTOR_DIRECTION_INVERTED                   0U
/* 反向中低速闭环已完成上板验证；在线跨零换向和高速性能仍不在开放范围。 */
#define MOTOR_REVERSE_CONTROL_VERIFIED             1U

/* 母线电压参数，保护阈值单位为0.1 V。 */
#define MOTOR_BUS_VOLTAGE_DIVIDER_RATIO           21U
#define MOTOR_BUS_UNDERVOLTAGE_0P1V               240U
#define MOTOR_BUS_OVERVOLTAGE_0P1V                600U

/* 10 Hz空载正向开环实测Hall状态进入边沿角度，U16一周制。 */
#define MOTOR_HALL_POSITIVE_NEXT_STATE_1             5U
#define MOTOR_HALL_POSITIVE_NEXT_STATE_2             3U
#define MOTOR_HALL_POSITIVE_NEXT_STATE_3             1U
#define MOTOR_HALL_POSITIVE_NEXT_STATE_4             6U
#define MOTOR_HALL_POSITIVE_NEXT_STATE_5             4U
#define MOTOR_HALL_POSITIVE_NEXT_STATE_6             2U
#define MOTOR_HALL_EDGE_ANGLE_STATE_1_U16         58890U
#define MOTOR_HALL_EDGE_ANGLE_STATE_2_U16         38627U
#define MOTOR_HALL_EDGE_ANGLE_STATE_3_U16         48251U
#define MOTOR_HALL_EDGE_ANGLE_STATE_4_U16         14958U
#define MOTOR_HALL_EDGE_ANGLE_STATE_5_U16          6537U
#define MOTOR_HALL_EDGE_ANGLE_STATE_6_U16         26968U
/* 由35 Hz闭环平均Vd/Vq=-2910/3333 mV计算，补偿开环标定引入的41.1度超前角。 */
#define MOTOR_HALL_ROTOR_ANGLE_OFFSET_U16            448U
#define MOTOR_HALL_DYNAMIC_COMPENSATION_DELAY_US      75U
#define MOTOR_HALL_DYNAMIC_COMPENSATION_MAXIMUM_U16 5461U /* 30度电角 */
#define MOTOR_HALL_SIGNAL_TIMEOUT_MS               1000U
/* Hall PLL scheduling: retain strong period filtering at low speed, then
 * continuously increase phase-loop bandwidth and shorten edge correction. */
#define MOTOR_HALL_PLL_LOW_FREQUENCY_MILLIHZ        13333U
#define MOTOR_HALL_PLL_HIGH_FREQUENCY_MILLIHZ       40000U
#define MOTOR_HALL_PLL_LOW_SPEED_FILTER_SHIFT           3U
#define MOTOR_HALL_PLL_MID_SPEED_FILTER_SHIFT           2U
#define MOTOR_HALL_PLL_HIGH_SPEED_FILTER_SHIFT          1U
#define MOTOR_HALL_PLL_LOW_BANDWIDTH_HZ                  3U
#define MOTOR_HALL_PLL_HIGH_BANDWIDTH_HZ                20U
#define MOTOR_HALL_PLL_LOW_EDGE_BLEND_CYCLES           255U /* uint8上限，约16 ms @16 kHz */
#define MOTOR_HALL_PLL_HIGH_EDGE_BLEND_CYCLES           48U /* 3 ms @16 kHz */
#define MOTOR_HALL_PLL_MINIMUM_TIMEOUT_MS             100U
#define MOTOR_HALL_PLL_TIMEOUT_SECTORS                   3U
/* Hall角度解算方式：
 * 0 = Hall-PLL + 逐扇区插值 + 边沿渐进修正（原方案）；
 * 1 = demo方式：边沿硬对齐扇区边界角，按测得电频率固定步长累加，
 *     超前边界角70°电角后冻结累加（对应mclib max_angle_err）。 */
#ifndef MOTOR_HALL_ANGLE_MODE
#define MOTOR_HALL_ANGLE_MODE                            1U
#endif
#define MOTOR_HALL_ANGLE_MODE_PLL                        0U
#define MOTOR_HALL_ANGLE_MODE_DEMO                       1U
#define MOTOR_HALL_DEMO_MAX_ANGLE_ERR_U16            12743U /* 70°电角 */
#if (MOTOR_HALL_ANGLE_MODE > 1U)
#error "MOTOR_HALL_ANGLE_MODE must be 0 or 1"
#endif
/* 16 kHz current loop: tolerate brief Hall input/decoder glitches (10 ms). */
#define MOTOR_CURRENT_CONTROL_HALL_INVALID_CYCLES     160U

/* Hall机械转速反馈：1 kHz更新，IIR滤波系数为1/8。 */
#define MOTOR_SPEED_FEEDBACK_TIMEOUT_MS             1000U

/* 速度外环固定以1 kHz执行；PI输出为现有电流环的Iq指令。 */
#define MOTOR_SPEED_LOOP_FREQUENCY_HZ                1000U
/* 速度启动方式：
 * 0 = 参考demo：静止即启动电流环(Iq=0,Hall扇区角)，速度PI全程直接输出Iq；
 * 1 = 旧方案：Hall-direct启动电流爬升，满足边沿/融合条件后再交给速度PI。 */
#ifndef MOTOR_SPEED_START_MODE
#define MOTOR_SPEED_START_MODE                           0U
#endif
#define MOTOR_SPEED_START_MODE_DIRECT                    0U
#define MOTOR_SPEED_START_MODE_HALL_HANDOVER             1U
#if (MOTOR_SPEED_START_MODE == MOTOR_SPEED_START_MODE_DIRECT)
/* 对应demo pid_spd Kp=2000/4096, Ki=50/32768 (CURRENT_BASE≈66 A)。 */
#define MOTOR_SPEED_PI_KP_Q20                          800L
#define MOTOR_SPEED_PI_KI_Q20                            3L
#else
#define MOTOR_SPEED_PI_KP_Q20                         4500L
#define MOTOR_SPEED_PI_KI_Q20                           45L
#endif
/* 直接模式堵转保护：|Iq|≥阈值且转速低于门限持续超时则故障停机。 */
#define MOTOR_SPEED_STALL_CURRENT_MA                 12000L
#define MOTOR_SPEED_STALL_SPEED_RPM                     30U
#define MOTOR_SPEED_STALL_TIME_MS                     1000U
#define MOTOR_SPEED_CONTROL_CURRENT_LIMIT_MA         15000L
/* Limit speed-loop torque steps caused by discrete Hall speed updates. */
#define MOTOR_SPEED_CONTROL_CURRENT_SLEW_MA_PER_S    20000U
/* Release excess startup torque faster than normal torque build-up. */
#define MOTOR_SPEED_CONTROL_CURRENT_RELEASE_MA_PER_S 40000U
/* Ignore the Hall speed staircase when deciding whether torque may be
 * released quickly during an acceleration ramp. */
#define MOTOR_SPEED_CONTROL_RELEASE_OVERSPEED_MARGIN_RPM 100U
/* Disable fast torque release while consecutive Hall samples show decel. */
#define MOTOR_SPEED_CONTROL_RELEASE_DECEL_MARGIN_RPM      50U
#define MOTOR_SPEED_HALL_START_INITIAL_CURRENT_MA       500L
#define MOTOR_SPEED_HALL_START_MAXIMUM_CURRENT_MA \
  MOTOR_SPEED_CONTROL_CURRENT_LIMIT_MA
#define MOTOR_SPEED_HALL_START_BOOST_DELAY_MS           150U
#define MOTOR_SPEED_HALL_START_BOOST_MA_PER_MS           15L
#define MOTOR_SPEED_HALL_START_SPEED_MATCH_RELEASE_MA_PER_MS 200L
/* After six qualified Hall edges and angle blending, keep Hall control active
 * before the speed PI takes over so its feedback filter can settle. */
#define MOTOR_SPEED_HALL_START_SETTLE_MS                   10U
#define MOTOR_SPEED_CONTROL_MAXIMUM_SPEED_RPM         4000L
#if (MOTOR_SPEED_START_MODE == MOTOR_SPEED_START_MODE_DIRECT)
#define MOTOR_SPEED_CONTROL_ACCELERATION_RPM_PER_S    3000U /* demo: 3 rpm/ms */
#else
#define MOTOR_SPEED_CONTROL_ACCELERATION_RPM_PER_S    1000U
#endif
/* 速度闭环保护阈值：当前仅支持正向Hall角度标定。 */
#define MOTOR_SPEED_OVERSPEED_RPM                      4400U

/* 正式开环控制器默认安全配置。 */
#define MOTOR_OPEN_LOOP_MAXIMUM_VOLTAGE_MV          4800L
#define MOTOR_OPEN_LOOP_MAXIMUM_FREQUENCY_MILLIHZ 100000L
#define MOTOR_OPEN_LOOP_START_FREQUENCY_MILLIHZ     2000L
#define MOTOR_OPEN_LOOP_ALIGNMENT_VOLTAGE_MV        1440L
#define MOTOR_OPEN_LOOP_ALIGNMENT_ANGLE_U16            0U
#define MOTOR_OPEN_LOOP_ALIGNMENT_TIME_MS             500U

/* 阻塞式相电阻辨识参数。 */
/* 两点差分法：先测低电流点，再测高电流点，Rs=(V2-V1)/(I2-I1)。 */
#define MOTOR_RESISTANCE_IDENT_LOW_CURRENT_MA      1500L
#define MOTOR_RESISTANCE_IDENT_TARGET_CURRENT_MA   3000L
#define MOTOR_RESISTANCE_IDENT_MIN_DELTA_CURRENT_MA 800L
#define MOTOR_RESISTANCE_IDENT_ABORT_CURRENT_MA   10000L
/* 留足死区压降（16 kHz约0.43 V）后仍能到达3 A。 */
#define MOTOR_RESISTANCE_IDENT_MAX_VOLTAGE_MV      2000U
#define MOTOR_RESISTANCE_IDENT_VOLTAGE_STEP_MV        1U
#define MOTOR_RESISTANCE_IDENT_SETTLE_MS            200U
#define MOTOR_RESISTANCE_IDENT_MEASURE_MS           1000U
#define MOTOR_RESISTANCE_IDENT_MIN_CURRENT_MA        500L

/* 阻塞式dq轴电感辨识参数。 */
#define MOTOR_INDUCTANCE_IDENT_FREQUENCY_HZ           600U
#define MOTOR_INDUCTANCE_IDENT_VOLTAGE_MV             500U
#define MOTOR_INDUCTANCE_IDENT_SETTLE_SAMPLES        3200U /* 200 ms @16 kHz */
#define MOTOR_INDUCTANCE_IDENT_MEASURE_SAMPLES      16000U /* 1 s @16 kHz */
#define MOTOR_INDUCTANCE_IDENT_ABORT_CURRENT_MA     10000L

/* 电流PI：888 Hz带宽，参数来自Rs=268 mΩ、Ld=222 µH、Lq=206 µH。 */
#define MOTOR_PHASE_RESISTANCE_MOHM                   268U
#define MOTOR_DIRECT_INDUCTANCE_UH                    222U
#define MOTOR_QUADRATURE_INDUCTANCE_UH                206U
#define MOTOR_FLUX_LINKAGE_UWB                      19494U
#define MOTOR_CURRENT_LOOP_BANDWIDTH_HZ               888U
#define MOTOR_CURRENT_PI_D_KP_Q15                   40588L
#define MOTOR_CURRENT_PI_Q_KP_Q15                   37663L
#define MOTOR_CURRENT_PI_KI_Q15                      3063L /* 10 kHz时4900，按10/16缩放 */
#define MOTOR_CURRENT_PI_OUTPUT_LIMIT_MV             4800L
/* 正式电流环放开到整数百分比下的线性SVPWM上限；理论边界为Vbus/sqrt(3)=57.735%。 */
#define MOTOR_CURRENT_CONTROL_VOLTAGE_LIMIT_PERCENT    57U
/* 48 V母线在线性SVPWM理论边界Vbus/sqrt(3)下的dq电压矢量硬上限。 */
#define MOTOR_CURRENT_CONTROL_HARD_VOLTAGE_LIMIT_MV 27700L
#define MOTOR_CURRENT_CONTROL_ABORT_CURRENT_MA       50000L

/* Shadow sensorless observer: diagnostic-only back-EMF/PLL path.
 * 16 kHz电流环每2个周期调用一次，即8 kHz/125 us。 */
#define MOTOR_SENSORLESS_OBSERVER_DECIMATION             2U
#define MOTOR_SENSORLESS_OBSERVER_SAMPLE_FREQUENCY_HZ  8000U
#define MOTOR_SENSORLESS_OBSERVER_SAMPLE_PERIOD_US     125U
#define MOTOR_SENSORLESS_OBSERVER_CURRENT_FILTER_SHIFT   2U
#define MOTOR_SENSORLESS_OBSERVER_BEMF_FILTER_SHIFT      3U
#define MOTOR_SENSORLESS_OBSERVER_PLL_FILTER_SHIFT       3U
#define MOTOR_SENSORLESS_OBSERVER_PLL_PHASE_SHIFT        3U
#define MOTOR_SENSORLESS_OBSERVER_MIN_BEMF_MV          250U
#define MOTOR_SENSORLESS_OBSERVER_MIN_FREQUENCY_MILLIHZ 2000L
#define MOTOR_SENSORLESS_OBSERVER_MAX_FREQUENCY_MILLIHZ 200000L
#define MOTOR_SENSORLESS_OBSERVER_VALID_SAMPLES           20U

/* Independent current-mode I/F startup and Hall handover. */
#define MOTOR_CURRENT_STARTUP_HALL_DIRECT_ENABLE           1U
#define MOTOR_CURRENT_STARTUP_ALIGNMENT_CURRENT_MA    1000L
#define MOTOR_CURRENT_STARTUP_ALIGNMENT_VOLTAGE_MV       0L
#define MOTOR_CURRENT_STARTUP_SAMPLE_SETTLE_CYCLES       8U /* 0.5 ms @16 kHz */
#define MOTOR_CURRENT_STARTUP_ALIGNMENT_ANGLE_U16        0U
#define MOTOR_CURRENT_STARTUP_ALIGNMENT_TIME_MS        250U
/* Begin almost stationary after alignment.  A 2 Hz electrical-frequency
 * step started rotating the forced field before Iq had built up. */
#define MOTOR_CURRENT_STARTUP_INITIAL_FREQUENCY_MILLIHZ  200L
#define MOTOR_CURRENT_STARTUP_HANDOVER_FREQUENCY_MILLIHZ 12000L
/* Four electrical hertz is 60 mechanical rpm with four pole pairs.  This
 * permits low-speed commands to hand over without first overshooting 120 rpm. */
#define MOTOR_CURRENT_STARTUP_MINIMUM_HALL_FREQUENCY_MILLIHZ 4000U
#define MOTOR_CURRENT_STARTUP_FREQUENCY_RAMP_MILLIHZ_PER_MS 20L
#define MOTOR_CURRENT_STARTUP_CURRENT_RAMP_MA_PER_MS     600L
/* After Hall capture, unload the pull-in torque gently so a low requested
   current does not decelerate the rotor below the Hall feedback threshold. */
#define MOTOR_CURRENT_STARTUP_HANDOVER_CURRENT_RAMP_MA_PER_MS 5L
#define MOTOR_CURRENT_STARTUP_HALL_DIRECT_CURRENT_RAMP_MA_PER_MS 10L
#define MOTOR_CURRENT_STARTUP_HALL_DIRECT_REQUIRED_EDGES          6U
#define MOTOR_CURRENT_STARTUP_IF_MINIMUM_CURRENT_MA     1600L
/* I/F startup current is raised only when the measured Hall speed falls
 * behind the commanded rotating field.  With no Hall edges, increase the
 * available pull-in torque gradually up to this ceiling. */
#define MOTOR_CURRENT_STARTUP_IF_MAXIMUM_CURRENT_MA    10000L
#define MOTOR_CURRENT_STARTUP_IF_BOOST_MA_PER_MS          30L
/* Require two complete six-step electrical revolutions before handover.
 * Six edges allowed capture while the rotor was still rocking, which made
 * low-current starts repeatedly accelerate and decelerate. */
#define MOTOR_CURRENT_STARTUP_HALL_REQUIRED_EDGES        12U
/* 1000 PWM cycles = about 50 ms at 20 kHz.  Spread the I/F-to-Hall angle
 * correction over a longer interval to reduce the handover torque step. */
#define MOTOR_CURRENT_STARTUP_BLEND_CYCLES              640U /* 40 ms @16 kHz */
#define MOTOR_CURRENT_STARTUP_TIMEOUT_MS               3000U
#define MOTOR_SPEED_REVERSE_ENTRY_RPM                    150U
#define MOTOR_SPEED_REVERSE_IF_CURRENT_LIMIT_MA         2000L
#define MOTOR_SPEED_REVERSE_CURRENT_RAMP_MA_PER_MS        10L
/* Opposite-direction stationary starts already require consecutive Hall
 * edges with the requested sign.  Begin the existing 50 ms angle blend
 * early, before the free-running I/F phase accumulates a large displacement
 * from the measured rotor angle. */
#define MOTOR_SPEED_REVERSE_HALL_REQUIRED_EDGES             2U
/* 调试CLI初期限制，正式通信协议接入状态机后再按权限放宽。 */
#define MOTOR_CLI_CURRENT_COMMAND_LIMIT_MA            10000L
#define MOTOR_CURRENT_LOOP_TEST_DIRECT_REFERENCE_MA  2000L
#define MOTOR_CURRENT_LOOP_TEST_DURATION_SAMPLES    16000U /* 1 s @16 kHz */
#define MOTOR_CURRENT_LOOP_TEST_ABORT_CURRENT_MA    10000L

/* Hall角度闭环转矩测试参数：第三级使用1 A、5秒观察15%限压下的空载转速。 */
#define MOTOR_TORQUE_TEST_QUADRATURE_REFERENCE_MA    1000L
#define MOTOR_TORQUE_TEST_REFERENCE_RAMP_SAMPLES     3200U /* 200 ms @16 kHz */
#define MOTOR_TORQUE_TEST_DURATION_SAMPLES          80000U /* 5 s @16 kHz */
#define MOTOR_TORQUE_TEST_ABORT_CURRENT_MA           10000L
#define MOTOR_TORQUE_TEST_BOOTSTRAP_TIMEOUT_MS        6000U
#define MOTOR_TORQUE_TEST_HANDOVER_FREQUENCY_MILLIHZ 8000U
#define MOTOR_TORQUE_TEST_BUS_VOLTAGE_LIMIT_PERCENT     15U
#define MOTOR_TORQUE_TEST_ABSOLUTE_VOLTAGE_LIMIT_MV   7200L

/* Hall转子角度补偿辨识：正Iq低功率闭环平均电压的有效性和单次修正限制。 */
#define MOTOR_HALL_OFFSET_MINIMUM_VQ_MV                 500U
#define MOTOR_HALL_OFFSET_MAXIMUM_CORRECTION_U16      10923U /* 60度电角 */

/* 编译期配置一致性检查：配置错误必须在生成固件前暴露。 */
#if (MOTOR_POLE_PAIRS == 0U)
#error "MOTOR_POLE_PAIRS must be greater than zero"
#endif

#if (MOTOR_DIRECTION_INVERTED > 1U)
#error "MOTOR_DIRECTION_INVERTED must be 0 or 1"
#endif

#if (MOTOR_REVERSE_CONTROL_VERIFIED > 1U)
#error "MOTOR_REVERSE_CONTROL_VERIFIED must be 0 or 1"
#endif

#if ((MOTOR_HALL_OFFSET_MINIMUM_VQ_MV == 0U) || \
     (MOTOR_HALL_OFFSET_MINIMUM_VQ_MV > \
      MOTOR_TORQUE_TEST_ABSOLUTE_VOLTAGE_LIMIT_MV))
#error "Hall offset minimum Vq must fit the torque-test voltage limit"
#endif

#if ((MOTOR_HALL_OFFSET_MAXIMUM_CORRECTION_U16 == 0U) || \
     (MOTOR_HALL_OFFSET_MAXIMUM_CORRECTION_U16 >= 16384U))
#error "Hall offset correction limit must be within 0..90 electrical degrees"
#endif

#if ((MOTOR_HALL_PLL_LOW_FREQUENCY_MILLIHZ == 0U) || \
     (MOTOR_HALL_PLL_LOW_FREQUENCY_MILLIHZ >= \
      MOTOR_HALL_PLL_HIGH_FREQUENCY_MILLIHZ) || \
     (MOTOR_HALL_PLL_LOW_SPEED_FILTER_SHIFT > 8U) || \
     (MOTOR_HALL_PLL_MID_SPEED_FILTER_SHIFT > 8U) || \
     (MOTOR_HALL_PLL_HIGH_SPEED_FILTER_SHIFT > 8U) || \
     (MOTOR_HALL_PLL_LOW_BANDWIDTH_HZ == 0U) || \
     (MOTOR_HALL_PLL_LOW_BANDWIDTH_HZ > \
      MOTOR_HALL_PLL_HIGH_BANDWIDTH_HZ) || \
     (MOTOR_HALL_PLL_HIGH_BANDWIDTH_HZ > 1000U) || \
     (MOTOR_HALL_PLL_LOW_EDGE_BLEND_CYCLES > 255U) || \
     (MOTOR_HALL_PLL_HIGH_EDGE_BLEND_CYCLES == 0U) || \
     (MOTOR_HALL_PLL_LOW_EDGE_BLEND_CYCLES < \
      MOTOR_HALL_PLL_HIGH_EDGE_BLEND_CYCLES))
#error "Hall PLL scheduling configuration is invalid"
#endif

#if (MOTOR_SPEED_LOOP_FREQUENCY_HZ != 1000U)
#error "motor_speed_control is designed for a 1 kHz update rate"
#endif

#if ((MOTOR_CURRENT_CONTROL_VOLTAGE_LIMIT_PERCENT == 0U) || \
     (MOTOR_CURRENT_CONTROL_VOLTAGE_LIMIT_PERCENT > 57U))
#error "current control modulation limit must be within 1..57 percent"
#endif

#if ((MOTOR_CURRENT_CONTROL_HARD_VOLTAGE_LIMIT_MV <= 0L) || \
     (MOTOR_CURRENT_CONTROL_HARD_VOLTAGE_LIMIT_MV > 27700L))
#error "48 V linear SVPWM hard voltage limit must not exceed 27700 mV"
#endif

#if ((MOTOR_FLUX_LINKAGE_UWB == 0U) || \
     (MOTOR_FLUX_LINKAGE_UWB > 1000000U))
#error "MOTOR_FLUX_LINKAGE_UWB is outside the supported range"
#endif

#if ((MOTOR_SENSORLESS_OBSERVER_DECIMATION == 0U) || \
     (MOTOR_SENSORLESS_OBSERVER_SAMPLE_PERIOD_US * \
      MOTOR_SENSORLESS_OBSERVER_SAMPLE_FREQUENCY_HZ != 1000000U) || \
     (MOTOR_SENSORLESS_OBSERVER_CURRENT_FILTER_SHIFT > 8U) || \
     (MOTOR_SENSORLESS_OBSERVER_BEMF_FILTER_SHIFT > 8U) || \
     (MOTOR_SENSORLESS_OBSERVER_PLL_FILTER_SHIFT > 8U) || \
     (MOTOR_SENSORLESS_OBSERVER_PLL_PHASE_SHIFT > 8U) || \
     (MOTOR_SENSORLESS_OBSERVER_MIN_BEMF_MV == 0U) || \
     (MOTOR_SENSORLESS_OBSERVER_MIN_FREQUENCY_MILLIHZ <= 0L) || \
     (MOTOR_SENSORLESS_OBSERVER_MAX_FREQUENCY_MILLIHZ <= \
      MOTOR_SENSORLESS_OBSERVER_MIN_FREQUENCY_MILLIHZ) || \
     (MOTOR_SENSORLESS_OBSERVER_VALID_SAMPLES == 0U))
#error "sensorless observer configuration is invalid"
#endif

#if ((MOTOR_SPEED_CONTROL_CURRENT_LIMIT_MA <= 0L) || \
     (MOTOR_SPEED_CONTROL_CURRENT_LIMIT_MA > MOTOR_CURRENT_COMMAND_MAX_MA))
#error "speed control current limit exceeds current-loop command limit"
#endif

#if ((MOTOR_SPEED_CONTROL_CURRENT_RELEASE_MA_PER_S < \
      MOTOR_SPEED_CONTROL_CURRENT_SLEW_MA_PER_S) || \
     (MOTOR_SPEED_CONTROL_RELEASE_OVERSPEED_MARGIN_RPM == 0U) || \
     (MOTOR_SPEED_CONTROL_RELEASE_DECEL_MARGIN_RPM == 0U))
#error "speed control current release configuration is invalid"
#endif

#if ((MOTOR_SPEED_HALL_START_INITIAL_CURRENT_MA <= 0L) || \
     (MOTOR_SPEED_HALL_START_MAXIMUM_CURRENT_MA < \
      MOTOR_SPEED_HALL_START_INITIAL_CURRENT_MA) || \
     (MOTOR_SPEED_HALL_START_MAXIMUM_CURRENT_MA > \
      MOTOR_SPEED_CONTROL_CURRENT_LIMIT_MA) || \
     (MOTOR_SPEED_HALL_START_BOOST_MA_PER_MS <= 0L) || \
     (MOTOR_SPEED_HALL_START_BOOST_DELAY_MS >= \
      MOTOR_CURRENT_STARTUP_TIMEOUT_MS) || \
     (MOTOR_SPEED_HALL_START_SETTLE_MS == 0U) || \
     (MOTOR_SPEED_HALL_START_SETTLE_MS >= \
      MOTOR_CURRENT_STARTUP_TIMEOUT_MS))
#error "Hall speed-start current configuration is invalid"
#endif

#if (MOTOR_SPEED_OVERSPEED_RPM <= MOTOR_SPEED_CONTROL_MAXIMUM_SPEED_RPM)
#error "overspeed threshold must be above maximum command speed"
#endif

#if ((MOTOR_CURRENT_STARTUP_ALIGNMENT_CURRENT_MA <= 0L) || \
     (MOTOR_CURRENT_STARTUP_ALIGNMENT_CURRENT_MA > MOTOR_CURRENT_COMMAND_MAX_MA))
#error "current startup alignment current is outside the current-loop limit"
#endif

#if (MOTOR_CURRENT_STARTUP_HALL_DIRECT_ENABLE > 1U)
#error "Hall-direct startup enable must be 0 or 1"
#endif

#if ((MOTOR_CURRENT_STARTUP_ALIGNMENT_VOLTAGE_MV < 0L) || \
     (MOTOR_CURRENT_STARTUP_ALIGNMENT_VOLTAGE_MV > \
      MOTOR_CURRENT_CONTROL_HARD_VOLTAGE_LIMIT_MV) || \
     (MOTOR_CURRENT_STARTUP_SAMPLE_SETTLE_CYCLES == 0U) || \
     (MOTOR_CURRENT_STARTUP_SAMPLE_SETTLE_CYCLES >= \
      MOTOR_SOFTWARE_OVERCURRENT_SAMPLES))
#error "current startup sampling settle configuration is invalid"
#endif

#if ((MOTOR_CURRENT_STARTUP_INITIAL_FREQUENCY_MILLIHZ <= 0L) || \
     (MOTOR_CURRENT_STARTUP_INITIAL_FREQUENCY_MILLIHZ > \
      MOTOR_CURRENT_STARTUP_HANDOVER_FREQUENCY_MILLIHZ))
#error "current startup initial frequency must not exceed handover frequency"
#endif

#if ((MOTOR_CURRENT_STARTUP_FREQUENCY_RAMP_MILLIHZ_PER_MS <= 0L) || \
     (MOTOR_CURRENT_STARTUP_CURRENT_RAMP_MA_PER_MS <= 0L) || \
     (MOTOR_CURRENT_STARTUP_HANDOVER_CURRENT_RAMP_MA_PER_MS <= 0L) || \
     (MOTOR_CURRENT_STARTUP_HALL_DIRECT_CURRENT_RAMP_MA_PER_MS <= 0L) || \
     (MOTOR_CURRENT_STARTUP_HALL_DIRECT_REQUIRED_EDGES == 0U) || \
     (MOTOR_CURRENT_STARTUP_IF_MINIMUM_CURRENT_MA <= 0L) || \
     (MOTOR_CURRENT_STARTUP_IF_MAXIMUM_CURRENT_MA < \
      MOTOR_CURRENT_STARTUP_IF_MINIMUM_CURRENT_MA) || \
     (MOTOR_CURRENT_STARTUP_IF_MAXIMUM_CURRENT_MA > \
      MOTOR_CLI_CURRENT_COMMAND_LIMIT_MA) || \
     (MOTOR_CURRENT_STARTUP_IF_BOOST_MA_PER_MS <= 0L) || \
     (MOTOR_CURRENT_STARTUP_IF_MINIMUM_CURRENT_MA > \
      MOTOR_CLI_CURRENT_COMMAND_LIMIT_MA) || \
     (MOTOR_CURRENT_STARTUP_HALL_REQUIRED_EDGES < 6U) || \
     (MOTOR_CURRENT_STARTUP_BLEND_CYCLES == 0U))
#error "current startup ramp, Hall edge count, and blend length must be valid"
#endif

#if (MOTOR_CURRENT_STARTUP_TIMEOUT_MS <= MOTOR_CURRENT_STARTUP_ALIGNMENT_TIME_MS)
#error "current startup timeout must exceed alignment time"
#endif

#endif /* MOTOR_CONTROL_CONFIG_H */
