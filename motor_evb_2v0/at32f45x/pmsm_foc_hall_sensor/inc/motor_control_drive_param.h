/**
  **************************************************************************
  * @file     motor_control_drive_param.h
  * @brief    Motor-related, drive-related and control-related parmeters, such as number of motor poles, maximum sensing voltage/current and pid speed parameters
  *           User defined motor drive modes (current sampling mode, sensor mode, etc.)
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

#ifndef __MOTOR_CONTROL_DRIVE_PARAM_H
#define __MOTOR_CONTROL_DRIVE_PARAM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mc_lib.h"

/* internal clock or external crytal */
//#define INTERNAL_CLOCK_SOURCE

/* choose MOTOR_EVB_BOARD version */
#define AT_MOTOR_EVB_V2


/* gate driver low side inverting logic input or non-inverting logic input*/
/* DRV8353 uses non-inverting inputs, so GATE_DRIVER_LOW_SIDE_INVERT is NOT defined */

/* FOC control */
#define FOC_CONTROL

/* choose current sensing circuit */
#define THREE_SHUNT
//#define TWO_SHUNT
//#define ONE_SHUNT

/*
 * The target board phase-current amplifiers rise for positive inverter-to-motor
 * current, opposite to the motor library's default ADC polarity.
 */
#define INVERT_PHASE_CURRENT_POLARITY

#ifndef ONE_SHUNT
#ifdef TWO_SHUNT              /* choose current sensing circuit for TWO-SHUNT */
#define U_V_SHUNT
//#define V_W_SHUNT
//#define U_W_SHUNT
#endif
//#define MOS_RDS_SHUNT         /* use MOSFET RDS(ON) as current sensing, AT-MOTOR EVB need be modified */
#define TWO_ADC_CONVERTERS    /* use two ADC converters */
#endif

/* hall sensors */
#define HALL_SENSORS


#ifdef HALL_SENSORS
#define LOW_SPEED_VOLT_CTRL      /* if need low speed control or position control */
#endif

/* Enable Ia/Ib/Ic current signal low-pass filtering mode or not */
#define AC_CURRENT_LP_FILTER  0

/* Enable D/Q-axis current signal low-pass filtering mode or not */
#if defined THREE_SHUNT || defined TWO_SHUNT
#define CURRENT_LP_FILTER   0
#elif defined ONE_SHUNT
#define CURRENT_LP_FILTER   1
#endif

/* motor type */
#define SPMSM

#if defined SPMSM
//#define FIELD_WEAKENING      /* Enable field weakening control for SPMSM motor or not */
#endif

//#define CURRENT_DECOUPLE_CTRL  /* Enable current decouple feature during torque (current) control or not */

//#define BRAKING_RESISTOR     /* Add braking resistor in EVB or not */

/* Enable Winding parameter identification or not */
#define MOTOR_PARAM_IDENTIFY

/* Select one UART mode: binary Motor Monitor or ASCII log. */
//#define USE_MOTOR_MONITOR      /* CTRL_SOURCE should be changed to CTRL_SOURCE_EXTERNAL if no motor monitor is used. */
#define USE_UART_LOG

#if defined(USE_MOTOR_MONITOR) && defined(USE_UART_LOG)
#error "Select only one UART mode"
#elif defined(USE_MOTOR_MONITOR)
#define UI_UART_BAUDRATE                (1500000UL)
#elif defined(USE_UART_LOG)
#define UI_UART_BAUDRATE                (921600UL)
#else
#error "Select USE_MOTOR_MONITOR or USE_UART_LOG"
#endif

/********************************* Motor-related parameter *********************************/
#define POLE_PAIRS                      (8/2)        /* 60BLDC140: 8 poles */
#define RS_LL                           (0.39997f)     /* line-to-line = 2 x 0.268 ohm (phase) */
#define LS_LL                           (0.000381854f)  /* line-to-line = 2 x 214 uH (phase avg) */
#define LD_LQ_RATIO                     (1.078f)     /* Ld/Lq = 222/206 */
#define KE                              (0.00849f)   /* V/rpm, motor spec 8.4 V/kRPM */
#define NOMINAL_CURRENT                 (11.5f)      /* A, motor rated current */

/*** Quadrature encoder ***/
#define ENCODER_PPR                     (1000)         /* Number of pulses per revolution */
#define ENC_IDX_COUNT                   (11)           /* Number of counts per index for JK42SBL01; default: 1 or 2 or 4 */
#define ENC_STALL_TIME                  (1000)         /* ms */

/* hall learn table */
#define HALL_LEARN_DIR                  (0)            /* Polarity, 0 or 1 */
#define HALL_LEARN_0_STATE              (5)
#define HALL_LEARN_1_STATE              (1)
#define HALL_LEARN_2_STATE              (3)
#define HALL_LEARN_3_STATE              (2)
#define HALL_LEARN_4_STATE              (6)
#define HALL_LEARN_5_STATE              (4)

/********************************* Drive-related parameter *********************************/
/* basic */
#define VDC_RATED                       (48.0f)
#define BAT_LOW_VOLT                    (12.0f)              /*!< minimum allowable battery voltage For E_BIKE_SCOOTER use only*/
#define V_SENSE_GAIN                    (1.0f/21.0f)         /* divider ratio 1/21 */
#define ADC_REFERENCE_VOLT              (3.3f)
#define ADC_DIGITAL_SCALE_12BITS        (4095.0f)
/* Clock */
#if defined AT32F413xx
#define SYSTEM_CORE_CLOCK               (200000000)
#elif defined AT32F421xx
#define SYSTEM_CORE_CLOCK               (120000000)
#elif defined AT32F415xx || defined AT32F423xx
#define SYSTEM_CORE_CLOCK               (150000000)
#elif defined AT32F403Axx || defined AT32F407xx
#define SYSTEM_CORE_CLOCK               (240000000)
#elif defined AT32F435xx || defined AT32F437xx
#define SYSTEM_CORE_CLOCK               (288000000)
#elif defined AT32F425xx
#define SYSTEM_CORE_CLOCK               (96000000)
#elif defined AT32L021xx
#define SYSTEM_CORE_CLOCK               (80000000)
#elif defined AT32M412xx || defined AT32M416xx || defined AT32F422xx || defined AT32F426xx
#define SYSTEM_CORE_CLOCK               (180000000)
#elif defined AT32F455xx || defined AT32F456xx || defined AT32F457xx
#define SYSTEM_CORE_CLOCK               (192000000)
#elif defined AT32F402xx || defined AT32F405xx
#define SYSTEM_CORE_CLOCK               (216000000)
#endif

#define TMR_CLK                         (SYSTEM_CORE_CLOCK)      //system_core_clock  
#define DEADTIME_CLK_SFT_BITS           ((tmr_clock_division_type) 2)
#define DEADTIME_NS                     ((uint16_t)300)      /* in nsec; check the MOSFET/IGBT/IPM specification
                                                                SYSTEM_CORE_CLOCK = 288MHz, range is [0...1700];
                                                                SYSTEM_CORE_CLOCK = 240MHz, range is [0...2000];
                                                                SYSTEM_CORE_CLOCK = 200MHz, range is [0...2500];
                                                                SYSTEM_CORE_CLOCK = 150MHz, range is [0...3000];
                                                                SYSTEM_CORE_CLOCK = 120MHz, range is [0...4000];
                                                                SYSTEM_CORE_CLOCK =  96MHz, range is [0...5000];
                                                                SYSTEM_CORE_CLOCK =  80MHz, range is [0...5000]; */
#define MIN_INTERVAL_TIME               (2500)    /* ns; pwm shift time for one shunt */
#define ADC_TRIG_DELAY_TIME             (0.4f)    /* us; adc trigger delay time for gate drive + MOS delay */
#define ADC_TRIG_LAG_TIME               (4.0f)    /* us; adc trigger lag time for sensing current at only two phase low; TWO_ADC_CONVERTERS && THREE_SHUNT used only  */
#if defined AT32M412xx || defined AT32M416xx
#define SW_OP_INP_MODE_LEAD_TIME        (5.5)     /* us; leading time of changing OP inp mode before adc trigger */
#endif

/* Current */
#define MAX_CURRENT                     (50.0f)
#define MIN_CURRENT                     (-MAX_CURRENT)
#define DC_MAX_CURRENT                  (60.0f)
#define CURRENT_SPAN_SHIFT              ((uint8_t) 1)
#ifdef ONE_SHUNT
#define R_SHUNT                         (0.005f)
#define OP_GAIN                         (39.0f/(1+39)*(1+9.1/1))                        // 9.8475 (9.8488)
#define CURR_OFFSET_VOLT                (ADC_REFERENCE_VOLT*(1.0f/(1+39)*(1+9.1f/1)))    // 0.83325 V  0.824
#else
#define R_SHUNT                         (0.0025f)     /* 2.5 mOhm */
#define OP_GAIN                         (10.0f)       /* DRV8353 built-in CSA */
#define CURR_OFFSET_VOLT                (1.65f)       /* VREF/2 */
#endif

#define RDC_SHUNT                       (0.005f)
#define DC_OP_GAIN                      (39.0f/(1+39)*(1+9.1/1))                        // 9.8475 (9.8488)
#define IDC_OFFSET_VOLT                 (ADC_REFERENCE_VOLT*(1.0f/(1+39)*(1+9.1/1)))    // 0.83325 V  0.824

/* EMF */
#define EMF_SENSE_GAIN                  (3.9f/(3.9f+37.4f))   //0.09443

/* Protection */
/* Power */
#define OVER_POWER_THRESHOLD            (300)

/* Current */
/* software over current protection by ADC windowns */
#define OVER_CURRENT_SW                 (55.0f)  /* A */
/* over current setting in hardware */
#if defined AT32M412xx || defined AT32M416xx
#define DAC_VREF_SOURCE                 (DAC_VDDA)
#define OCP_CURRENT                     (50.0)
#define BUS_CURR_CMP_OCP_VOLT           ((OCP_CURRENT*RDC_SHUNT*DC_OP_GAIN)+IDC_OFFSET_VOLT) //3.04
#endif

/* Bus voltage */
#define OVER_VOLT_THRESHOLD             (60)
#define UNDER_VOLT_THRESHOLD            (24)
/* Temperature sensing section */
/* V[V]=V0+dV/dT[V/Celsius]*(T-T0)[Celsius] */
#define V0_V                            (0.480f) /*!< in Volts */
#define T0_C                            (0) /*!< in Celsius degrees */
#define dV_dT                           (0.024f) /*!< V/Celsius degrees */
#define OVER_TEMP_THRESHOLD             (70) /*!< Celsius degrees */
/* error code mask */
#define MC_ERROR_MASK                   ((err_code_type) (MC_OVER_VOLT_ERROR | MC_UNDER_VOLT_ERROR | MC_OVER_TEMP_ERROR | MC_OVER_CURRENT_ERROR | MC_ENCODER_ERROR | MC_HALL_ERROR | MC_PARAM_IDENT_ERROR | MC_HALL_LEARN_ERROR | MC_STARTUP_ERROR))






/********************************* Control-related parameter *********************************/
#define PWM_FREQ                        (16000)  /* Hz */
#define MOTOR_CONTROL_MODE              (OPEN_LOOP_CTRL)
#define CTRL_SOURCE                     (CTRL_SOURCE_SOFTWARE)

/* current tuning parameter */
#define TUNE_TARGET_CURRENT             (1.0f)
#define TUNE_CURRENT_TOTAL_PERIOD       (100)
#define TUNE_CURRENT_STEP_PERIOD        (2)

/* SPEED */
#define SPEED_LOOP_FREQ                 (1000) /* Hz */
#define MIN_SPEED_RPM                   (200)
#define MAX_SPEED_RPM                   (4000)
#define STABLE_SPEED_RPM                (50)
#define SLICK_SPEED_RPM                 (50)
#define MIN_SENSE_SPEED                 (10)
#define MIN_POSCTL_SPD                  (100)
#define ACC_SPD_SLOPE                   (15)    /* (rpm/ms) */
#define DEC_SPD_SLOPE                   (15)

/* POSITION */
#define POSITION_LOOP_FREQ              (200)     /* Hz */
#define MAX_POSITION_ANGLE              (360000)   /* Degree */
#define MIN_POSITION_ANGLE              (-MAX_POSITION_ANGLE)
#ifdef HALL_SENSORS
#define CMD_TO_VAL_GAP                  (20000)   /* hall eletric angle(32767 = 360 E-degree);  position is close target command*/
#else
#define CMD_TO_VAL_GAP                  (1000)    /* counts (ex: encoder CPR = 4000; 1000/4000*360 = 90 M-degree);  position is close target command*/
#endif
#define SMALL_POS_CMD_GAP               (100)
#define ROTOR_LOCK_ANGLE_GAP            (120.0f)

/* open loop control */
#define OLC_ANGLE_INC                   (0)
#define OLC_VOLT                        (0.0f)      /* (V) */

/* hall learn */
#define LEARN_OLC_VOLT                  (0.8f)     /* (V) */
#define LEARN_OLC_ANGLE_INC             (5)
#define LEARN_TIME                      (10000)    /* (ms) */
#define LEARN_ALIGN_TIME                (500)      /* (ms) */

/* encoder parameter */
#define ALIGN_VOLT                      (0.8f)     /* (V) */
#define ENC_OFFSET                      (0)

/* magnetic encoder calibration */
#define ENC_OLC_VOLT                    (0.8f)   /* (V) */
#define ENC_OLC_ANGLE_INC               (1)
#define ENC_OLC_MOTOR_REV               (1.1)    /* motor turns 1.1 revolutions */
#define ENC_OLC_TIME                    (ENC_OLC_MOTOR_REV*32767*POLE_PAIRS/(ENC_OLC_ANGLE_INC*(PWM_FREQ/1000)))    /* (ms) */

/* current pid auto-tune */
#define AUTO_TUNE_CURR_BANDWIDTH        (3500)     /* 2*pi*freq */

/* Id/Iq pid parameter */
#define PID_ID_KP_DEFUALT               (6047)
#define PID_ID_KI_DEFUALT               (3414)
#define PID_ID_KP_GAIN_DIV              (4096)
#define PID_ID_KP_GAIN_DIV_LOG          (LOG2(PID_ID_KP_GAIN_DIV))
#define PID_ID_KI_GAIN_DIV              (32768)
#define PID_ID_KI_GAIN_DIV_LOG          (LOG2(PID_ID_KI_GAIN_DIV))

#define PID_IQ_KP_DEFUALT               (6047)
#define PID_IQ_KI_DEFUALT               (3414)
#define PID_IQ_KP_GAIN_DIV              (4096)
#define PID_IQ_KP_GAIN_DIV_LOG          (LOG2(PID_IQ_KP_GAIN_DIV))
#define PID_IQ_KI_GAIN_DIV              (32768)
#define PID_IQ_KI_GAIN_DIV_LOG          (LOG2(PID_IQ_KI_GAIN_DIV))

/* speed pid parameter */
#define PID_SPD_KP_DEFUALT              (2000)
#define PID_SPD_KI_DEFUALT              (50)
#define PID_SPD_KD_DEFUALT              (0)
#define PID_SPD_KP_GAIN_DIV             (4096)
#define PID_SPD_KP_GAIN_DIV_LOG         (LOG2(PID_SPD_KP_GAIN_DIV))
#define PID_SPD_KI_GAIN_DIV             (32768)
#define PID_SPD_KI_GAIN_DIV_LOG         (LOG2(PID_SPD_KI_GAIN_DIV))

/* position pid parameter */
#define PID_POS_KP_DEFUALT              (80)
#define PID_POS_KI_DEFUALT              (0)
#define PID_POS_KI_DEFUALT_STABLE       (800)     /* position is close to target, enhance the ki_gain to reach the target quickly */
#define PID_POS_KD_DEFUALT              (0)
#define PID_POS_KP_GAIN_DIV             (32768)
#define PID_POS_KP_GAIN_DIV_LOG         (LOG2(PID_POS_KP_GAIN_DIV))
#define PID_POS_KI_GAIN_DIV             (65536)
#define PID_POS_KI_GAIN_DIV_LOG         (LOG2(PID_POS_KI_GAIN_DIV))
#define PID_POS_KD_GAIN_DIV             (65536)
#define PID_POS_KD_GAIN_DIV_LOG         (LOG2(PID_POS_KD_GAIN_DIV))

/* field weakening parameter */
#define FW_MAX_ID_CURR                  (1.4f)   /* unit: A  */
#define FW_KP_GAIN                      (100) /*!< Default Kp gain */
#define FW_KI_GAIN                      (300) /*!< Default Ki gain */
#define FW_KP_GAIN_DIV                  (2048)
#define FW_KP_GAIN_DIV_LOG              (LOG2(FW_KP_GAIN_DIV))
#define FW_KI_GAIN_DIV                  (32768)
#define FW_KI_GAIN_DIV_LOG              (LOG2(FW_KI_GAIN_DIV))

/* low pass filter parameter */
#define CURR_LP_BANDWIDTH               (6000.0f)  /* 2*pi*freq */
#define AC_CURR_LP_BANDWIDTH            (8000.0f)  /* 2*pi*freq */
#define OBS_SPD_LP_BANDWIDTH            (300.0f)   /* 2*pi*freq */

/* State observer parameter */
#define OBS_GAIN1                       (20000)
#define OBS_GAIN2                       (-20000)
/* PLL gains */
#define PLL_KP_GAIN                     (1500)
#define PLL_KI_GAIN                     (5)
#define PLL_KP_GAIN_DIV                 (32768)
#define PLL_KP_GAIN_DIV_LOG             (LOG2(PLL_KP_GAIN_DIV))
#define PLL_KI_GAIN_DIV                 (32768)
#define PLL_KI_GAIN_DIV_LOG             (LOG2(PLL_KI_GAIN_DIV))

/* start-up parameter */
#define STARTUP_CURRENT                 (0.5f)   /* A */
#define STARTUP_VOLTAGE                 (1.0f)   /* V */
/* Open loop startup */
#define STARTUP_MAX_SPD                 (400)    /* rpm */
#define STARTUP_OL_SLOPE                (800)    /* rpm/s */
/* Fixed alpha-axis startup */
#define STARTUP_ALIGN_TIME              (1000)   /* ms */
#define STARTUP_START_TIME              (10)     /* ms */
/* Angle init parameter */
#define DETECT_PULSE_WIDTH              (20.0f)  /* us;  range [1...10000] */

/* Detect the startup error parameter */
#define DETECT_DELAY_TIME               (300)    /* msec; A startup error is triggered when motor reverses to "DETECT_REVERSE_SPEED" rpm in ESC_STATE_RUNNING state after "DETECT_DELAY_TIME" */
#define DETECT_REVERSE_SPEED            (10)     /* rpm; A startup error is triggered when motor reverses to "DETECT_REVERSE_SPEED" rpm in ESC_STATE_RUNNING state after "DETECT_DELAY_TIME" */
#define DETECT_MAX_SPEED                (MAX_SPEED_RPM*1.5) /* rpm; A startup error is triggered when motor runs at "DETECT_MAX_SPEED" rpm in ESC_STATE_RUNNING state after "DETECT_DELAY_TIME" */

/* SP */
#define SP_MAX_VOLT                     (3.3f)   /* V */
#define SP_THRESHOLD                    (0.1f)   /* V */
#define SP_RUN_VALUE                    (0.3f)   /* V */
#define SP_STOP_VALUE                   (0.11f)  /* V */

/* E_BIKE_SCOOTER mode */
#define REVERSE_MAX_SPEED_RPM           (-500)    /* RPM; For E_BIKE_SCOOTER use only */
#define REVERSE_CURRENT                 (-0.3f)   /* A; For E_BIKE_SCOOTER use only */
#define BRAKING_CURRENT                 (-0.3f)   /* A; For E_BIKE_SCOOTER use only */
#define MAX_LOCK_CURRENT                (3.0f)    /* A; For E_BIKE_SCOOTER use only */
#define ANTI_THEFT_INIT_CURRENT         (1.0f)    /* A; For E_BIKE_SCOOTER use only */
#define ANTI_THEFT_INC_CURRENT          (0.01f)    /* A; For E_BIKE_SCOOTER use only */
#define ANTI_THEFT_DEC_CURRENT          (0.01f)   /* A; For E_BIKE_SCOOTER use only */
#define PARKING_LOCK_INIT_CURRENT       (1.0f)    /* A; For E_BIKE_SCOOTER use only */
#define PARKING_LOCK_INC_CURRENT        (0.01f)    /* A; For E_BIKE_SCOOTER use only */
#define PARKING_LOCK_DEC_CURRENT        (0.01f)   /* A; For E_BIKE_SCOOTER use only */
#define ANTI_THEFT_LOCK_TIME            (60000)   /* ms; For E_BIKE_SCOOTER use only */
#define PARKING_LOCK_TIME               (30000)   /* ms; For E_BIKE_SCOOTER use only */

/* low speed voltage control parameter */
#define HYSTERESIS_LOW_SPEED            (300)     /* rpm */
#define HYSTERESIS_HIGH_SPEED           (400)     /* rpm */
#define VD_VOLT_LOW_SPD                 (0.0f)    /* V */
/* low speed pid parameter for voltage control */
#define PID_SPD_VOLT_KP_DEFUALT         (10000)
#define PID_SPD_VOLT_KI_DEFUALT         (300)
#define PID_SPD_VOLT_KD_DEFUALT         (0)
#define PID_SPD_VOLT_KP_GAIN_DIV        (1024)
#define PID_SPD_VOLT_KP_GAIN_DIV_LOG    (LOG2(PID_SPD_VOLT_KP_GAIN_DIV))
#define PID_SPD_VOLT_KI_GAIN_DIV        (2048)
#define PID_SPD_VOLT_KI_GAIN_DIV_LOG    (LOG2(PID_SPD_VOLT_KI_GAIN_DIV))

/* MPTA MTPV table parameters for IPMSM use only */
#define MTPA_MTPV_TABLE_MIN_SPEED       (2400)  /* rpm */
#define MTPA_MTPV_TABLE_MAX_SPEED       (7000)  /* rpm */
#define MTPA_MTPV_TABLE_SPEED_STEP      (100)   /* rpm */
#define MTPA_MTPV_TABLE_MAX_TORQUE      (2.2f)  /* Nm */
#define MTPA_MTPV_TABLE_TORQUE_STEP     (0.2f)  /* Nm */
#define MTPA_MTPV_TABLE_ROWS_NUM        (47)
#define MTPA_MTPV_TABLE_COLS_NUM        (12)
#define REVERSE_TORQUE                  (-0.5f)   /* Nm; For E_BIKE_SCOOTER use only */
#define BRAKING_TORQUE                  (-0.1f)   /* Nm; For E_BIKE_SCOOTER use only */

/* Sensing phases change point pf three-shunt or MOSFET rds current sensing */
#define HIGH_PWM_DUTY_CYCLE             (0.92f)   /* TWO_ADC_CONVERTERS used only */

/* headwind and tailwind detection */
#define WIND_DETECT_TIME                (200)    /* ms */
#define TAILWIND_SPEED                  (100)    /* rpm */
#define HEADWIND_BRAKE_TIME             (750)    /* ms */

/* led blink setting */
#define LED_BLINK_PERIOD                (500)    /* ms */

/* Maximum duty setting depend on current senisng circut */
#if defined ONE_SHUNT
#define MAX_DUTY_PCT                    (1.0f)
#elif defined MOS_RDS_SHUNT
#if (defined THREE_SHUNT) && (defined TWO_ADC_CONVERTERS)
#define MAX_DUTY_PCT                    (1.0f)
#else
#define MAX_DUTY_PCT                    (0.9f)
#endif
#elif defined THREE_SHUNT
#define MAX_DUTY_PCT                    (1.0f)
#elif defined TWO_SHUNT
#define MAX_DUTY_PCT                    (0.95f)
#endif


#ifdef __cplusplus
}
#endif

#endif

