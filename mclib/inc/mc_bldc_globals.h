/**
  **************************************************************************
  * @file     mc_bldc_globals.h
  * @brief    Global variables declaration and default values, global functions declaration header file
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

#ifndef __MC_BLDC_GLOBALS_H
#define __MC_BLDC_GLOBALS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mc_lib.h"
#include "mc_flash_data_table.h"

/* Motor-related parameters calculation */
#define RS                         (float)(RS_LL/2.0f)
#define LS                         (float)(LS_LL/2.0f)

#define CCW                        (1)
#define CW                         (0)

#define CTRL_DIR_CW                (1)
#define CTRL_DIR_CCW               (-1)

/* math */
#define SQRT3_2                    ((int16_t) (0x7FFF*0.866025404))
#define INV_SQRT_3                 ((int16_t) (0x7FFF*0.57735027))
#define INT16_MAX_DIV100           ((int32_t) (32767.0*32767.0*0.01))

/*********************** Motor-related parameters calculation (DON'T EDIT) ***********************/
#define Z_BASE                     (double)(VOLTAGE_BASE/CURRENT_BASE)
#define DECAY_CONST                ((int16_t) 0x7FFF*((LS_LL*PWM_FREQ-RS_LL)/(LS_LL*PWM_FREQ)))
#define KE_COMPENSATE              ((int32_t)(32767.0*32767.0/KE/VDC_RATED*0.8))

/*********************** Control-related parameters calculation (DON'T EDIT) ***********************/
#define MAX_CAP_COUNT              (0xFFFF)                                              /* Maximum number of Hall signals capture counts */
#define CAPTURE_CLK                (MAX_CAP_COUNT*MIN_SENSE_SPEED/60*POLE_PAIRS*(6/2))   /* 2 times small Hall cycle time */
#define TIM_CAP_CLK_DIV            ((uint16_t)((double)TMR_CLK/CAPTURE_CLK))
#define MIN_SPD_HALL_INR           ((int32_t)(MAX_CAP_COUNT*MIN_SENSE_SPEED/2))

#define START_CURRENT_PU           ((int16_t)(32767.0*START_CURRENT/CURRENT_BASE))
#define TUNE_TARGET_CURRENT_PU     ((int16_t)(32767.0*TUNE_TARGET_CURRENT/CURRENT_BASE))
#define START_VOLTAGE_DUTY_PU      ((int16_t)(32767.0*START_VOLTAGE/VDC_RATED))

#define MAX_SPD_CNT                ((int32_t)(60.0*PWM_FREQ/MIN_SENSE_SPEED/POLE_PAIRS/6))   //Computable max. speed count : (0x7FFFFFFFL/6/PWM_PERIOD)
/* external speed command calculation */
#ifdef PWM_INPUT
#define PWM_IN_TMR_FREQ            (TMR_CLK/(PWM_DUTY_TMR_DIV+1))

#define PWM_IN_NEUTRAL_CVAL        ((int16_t)(PWM_IN_TMR_FREQ/1000000*PWM_IN_NEUTRAL_WIDTH))
#define PWM_IN_MAX_CVAL            ((int16_t)(PWM_IN_TMR_FREQ/1000000*PWM_IN_MAX_WIDTH))
#define PWM_IN_MIN_CVAL            ((int16_t)(PWM_IN_TMR_FREQ/1000000*PWM_IN_MIN_WIDTH))

#define DEADZONE_PW_CVAL           ((int16_t)(PWM_IN_TMR_FREQ/1000000*DEADZONE_PULSEWIDTH))
#define TOLERANCE_PW_CVAL          ((int16_t)(PWM_IN_TMR_FREQ/1000000*TOLERANCE_PULSEWIDTH))
#define SPD_CURVE_SWITCH_PW_CVAL   ((int16_t)(PWM_IN_TMR_FREQ/1000000*SPD_CURVE_SWITCH_WIDTH))


#define PWM_IN_GAIN                (512)
#define PWM_IN_GAIN_LOG            (LOG2(PWM_IN_GAIN))
#endif
/* SP */
#define SP_OFFSET                  ((int16_t)(4095.0f*SP_THRESHOLD/3.3f))
#define SP_RUN_POINT               ((int16_t)(4095.0f*(SP_RUN_VALUE-SP_THRESHOLD)/3.3f))
#define SP_STOP_POINT              ((int16_t)(4095.0f*(SP_STOP_VALUE-SP_THRESHOLD)/3.3f))
#define SP_MAX_POINT              ((int16_t)(4095.0f*(SP_MAX_VOLT-SP_THRESHOLD)/3.3f))
#define SP_TO_I_CMD                ((int32_t)(32767.0f*32767.0f*(MAX_CURRENT/CURRENT_BASE)/(4095.0f*(1.0f-(SP_THRESHOLD/SP_MAX_VOLT)))))
#define SP_TO_SPD_CMD              ((int32_t)(32767.0f*(MAX_SPEED_RPM-MIN_SENSE_SPEED)/(4095.0f*(1.0f-(SP_THRESHOLD/SP_MAX_VOLT)))/2.0f))
/* open loop control */
#define OLC_VOLT_CMD               ((int16_t)(32767.0*OLC_INIT_VOLT/VDC_RATED))
#define OLC_VOLT_INC_CMD           ((int16_t)(32767.0*OLC_VOLT_INC/VDC_RATED))
/* lock start-up */
#define LOCK_VOLT_CMD              ((int16_t)(32767.0*LOCK_VOLT/VDC_RATED))
/* hall learning */
#define LEARN_OLC_VOLT             (OLC_INIT_VOLT)
#define LEARN_OLC_VOLT_CMD         (((int16_t)32767.0*LEARN_OLC_VOLT/VDC_RATED))
#define LEARN_OLC_PERIOD           ((int16_t)10*PWM_FREQ/OLC_INIT_SPD/POLE_PAIRS)     /*!< pwm cycle */
#define LEARN_CHECK_TIMES          (6*POLE_PAIRS)
#define HALL_LEARN_CHECK_FLAG      (SET)
#define HALL_LEARN_TABLE_LENGTH    (6)
/* EMF sample */
#define EMF_CHANGE_DUTY_H          ((uint16_t)(PWM_PERIOD * EMF_CHANGE_PERCENT_H/100.0))
#define EMF_CHANGE_DUTY_L          ((uint16_t)(PWM_PERIOD * EMF_CHANGE_PERCENT_L/100.0))
#define MIN_SAMPLE_INTERVAL        ((uint16_t)MIN_SAMPLE_INTERVAL_US*TMR_CLK/1000000*2)
#define EMF_LOW_SPD_SAMPLE_POINT   ((uint16_t)(PWM_PERIOD - EMF_SAMPLE_LEAD_PWM))
#define EMF_PHASE_ADV_SPD_PERIOD   ((uint32_t)(PWM_CVAL_TO_RPM/EMF_PHASE_ADV_SPD))
#define EMF_MIN_DELAY              ((uint16_t)EMF_MIN_DELAY_US*(TMR_CLK/1000000.0)*2.0)
#define EMF_AVOID_NOISE_INIT_TIMES ((int16_t)(EMF_AVOID_NOISE_INIT_PERIOD*1000.0/PWM_PERIOD_US))
#define EMF_BLANK_TIME_CG_INTERVAL ((int32_t)PWM_CVAL_TO_RPM/EMF_BLANK_TIME_CHANGED_RPM)

#define PWM_CVAL_TO_RPM            ((uint32_t)(60.0/6.0/POLE_PAIRS*TMR_CLK))
#define PWM_CVAL_TO_US             ((int32_t)(32767.0*1000000.0/PWM_PERIOD/PWM_FREQ))
#define PWM_CYCLE_TO_US            ((int32_t)(1000000.0/PWM_FREQ))
#define US_TO_RPM                  ((uint32_t)(60.0*1000000.0/6.0/POLE_PAIRS))
#define RPM_TO_SPEED_PU            ((int32_t)(32768.0*26213.0/MAX_SPEED_RPM))
/*********************** Drive-related parameters calculation (DON'T EDIT) ***********************/
#define I_SENSE_GAIN               (R_SHUNT*OP_GAIN)                           /*!< 0.0445 */
/* Basic */
#define CURRENT_BASE               ((float)((ADC_REFERENCE_VOLT-CURR_OFFSET_VOLT)/I_SENSE_GAIN))       /*!< 55.8 A*/
#define VOLTAGE_BASE               ((float)(ADC_REFERENCE_VOLT/V_SENSE_GAIN))  /*!< 63.987 V */

#define PWM_PERIOD                 ((uint16_t)(TMR_CLK/PWM_FREQ))              /*!<  6666(PWM:30K Hz) */
#define PWM_PERIOD_DIV_100         ((uint16_t)(TMR_CLK/PWM_FREQ/100))
#define ANGLE_INIT_PERIOD          ((uint16_t)(0xFFFF))

#define HALF_PWM_PERIOD            ((uint16_t)(TMR_CLK/PWM_FREQ/2)/2)
#define DOUBLE_PWM_PERIOD          ((uint16_t)(PWM_PERIOD*2))
#define IDEAL_1V2_ADC_VALUE        ((int16_t)(4095*1.2f/ADC_REFERENCE_VOLT))
/* Dead-time */
#define DEADTIME_CLK_DIV           (1 << DEADTIME_CLK_SFT_BITS)
#define DEADTIME_CLK               (TMR_CLK/DEADTIME_CLK_DIV)
#define DEADTIME                   ((uint8_t)((uint64_t)DEADTIME_CLK*DEADTIME_NS/1000000000uL))
#define PWM_PERIOD_US              ((uint16_t)(1000000*1/PWM_FREQ))                              /*!< us */
/* Current */
#define NOMINAL_CURRENT_PU         ((int16_t) (0x7FFF*NOMINAL_CURRENT/CURRENT_BASE))
#define MAX_CURRENT_PU             ((int16_t)(0x7FFF*MAX_CURRENT/CURRENT_BASE))
#define MIN_CURRENT_PU             ((int16_t)(0x7FFF*MIN_CURRENT/CURRENT_BASE))
#define CURRENT_SPAN               (((uint16_t) (0x7FFF*(3.3f/(3.3f-CURR_OFFSET_VOLT)))) >> CURRENT_SPAN_SHIFT)
#define CURR_OFFSET_VALUE          ((uint16_t)(4095*CURR_OFFSET_VOLT/3.3))
#define I_SAMP_MIN_CNT             ((int16_t)(TMR_CLK/1000000*I_SAMP_MIN_TIME/1000))
#define I_SAMP_DLY_CNT             ((int16_t)(TMR_CLK/1000000*I_SAMP_DELAY/1000))
/* EMF */
#define EMF_HALF_VDC_GAIN          (float)(VOLTAGE_BASE/2.0f/3.3f*EMF_SENSE_GAIN)
#define GATE_DELAY_COUNT           ((int16_t)(GATE_DELAY_TIME/1000000*TMR_CLK))
#define EMF_SIG_RISING_COUNT       ((int16_t)(TMR_CLK/1000000*EMF_SIG_RISING_TIME))
#define EMF_SIG_FALLING_COUNT      ((int16_t)(TMR_CLK/1000000*EMF_SIG_FALLING_TIME))
#define EMF_RISE_BLANK_CNT          ((int16_t)(TMR_CLK/1000000*EMF_RISE_BLANK_TIME))
#define EMF_FALL_BLANK_CNT_HIGH_SPD ((int16_t)(TMR_CLK/1000000*EMF_FALL_BLANK_TIME_HIGH_SPD))
#define EMF_FALL_BLANK_CNT_LOW_SPD  ((int16_t)(TMR_CLK/1000000*EMF_FALL_BLANK_TIME_LOW_SPD))
/* Protection */
#define CURR_OFFSET_VOLT_d         ((uint16_t)(CURR_OFFSET_VOLT*ADC_DIGITAL_SCALE_12BITS/ADC_REFERENCE_VOLT))
#define TEMPERATURE_THRESHOLD_d    ((uint16_t)((V0_V+dV_dT*(OVER_TEMP_THRESHOLD-T0_C))*ADC_DIGITAL_SCALE_12BITS/ADC_REFERENCE_VOLT))
#define OVERVOLTAGE_THRESHOLD_d    ((uint16_t)(OVER_VOLT_THRESHOLD*ADC_DIGITAL_SCALE_12BITS/VOLTAGE_BASE))
#define UNDERVOLTAGE_THRESHOLD_d   ((uint16_t)(UNDER_VOLT_THRESHOLD*ADC_DIGITAL_SCALE_12BITS/VOLTAGE_BASE))

#define OVERCURRENT_THRESHOLD_d    ((uint16_t)(OVER_CURRENT_SW*(ADC_DIGITAL_SCALE_12BITS-CURR_OFFSET_VOLT_d)/CURRENT_BASE + CURR_OFFSET_VOLT_d))
#define UNDERCURRENT_THRESHOLD_d   ((uint16_t) 0)
//#if (DAC_VREF_SOURCE)
//#define DAC_OCP_REF                ((uint16_t) (63.0*BUS_CURR_CMP_OCP_VOLT/1.2f))
//#else
#define DAC_OCP_REF                ((uint16_t) (63.0*BUS_CURR_CMP_OCP_VOLT/ADC_REFERENCE_VOLT))
//#endif
/* MOS Temperature */
#define TEMPER_A                   ((int32_t) (0xFFFF*ADC_REFERENCE_VOLT/(ADC_DIGITAL_SCALE_12BITS*dV_dT)))
#define TEMPER_B                   ((int32_t) (0xFFFF*-V0_V/dV_dT))
#define TEMPER_C                   ((int32_t) (0xFFFF*T0_C))

/**************** define pwm mode pattern ******************/
#define AH_BL_PWM_MODE_CM1            TMR_PWM_1PWMA_2LOW      /*AH - BL*/
#define AH_CL_PWM_MODE_CM1            TMR_PWM_1PWMA_2OFF      /*AH - CL*/
#define BH_CL_PWM_MODE_CM1            TMR_PWM_1OFF_2PWMA      /*BH - CL*/
#define BH_AL_PWM_MODE_CM1            TMR_PWM_1LOW_2PWMA      /*BH - AL*/
#define CH_AL_PWM_MODE_CM1            TMR_PWM_1LOW_2OFF       /*CH - AL*/
#define CH_BL_PWM_MODE_CM1            TMR_PWM_1OFF_2LOW       /*CH - BL*/
#define AH_BCL_PWM_MODE_CM1           TMR_PWM_1PWMA_2LOW      /*AH - BCL*/
#define BRAKE_PWM_MODE_CM1            TMR_PWM_1PWMA_2PWMA

#define AH_BL_PWM_MODE_CM2            TMR_PWM_3OFF            /*AH - BL*/
#define AH_CL_PWM_MODE_CM2            TMR_PWM_3LOW            /*AH - CL*/
#define BH_CL_PWM_MODE_CM2            TMR_PWM_3LOW            /*BH - CL*/
#define BH_AL_PWM_MODE_CM2            TMR_PWM_3OFF            /*BH - AL*/
#define CH_AL_PWM_MODE_CM2            TMR_PWM_3PWMA           /*CH - AL*/
#define CH_BL_PWM_MODE_CM2            TMR_PWM_3PWMA           /*CH - BL*/
#define AH_BCL_PWM_MODE_CM2           TMR_PWM_3LOW            /*AH - BCL*/
#define BRAKE_PWM_MODE_CM2            TMR_PWM_3PWMA

/**************** define drive output enable pattern ******************/
#if defined COMPLEMENT && defined GATE_DRIVER_LOW_SIDE_INVERT
#define AH_BL_PWM_OUT_CCTRL           TMR_PWM_C_OUT_L_INV_1TOP_2BOTTOM_3OFF    /*AH - BL*/
#define AH_CL_PWM_OUT_CCTRL           TMR_PWM_C_OUT_L_INV_1TOP_2OFF_3BOTTOM    /*AH - CL*/
#define BH_CL_PWM_OUT_CCTRL           TMR_PWM_C_OUT_L_INV_1OFF_2TOP_3BOTTOM    /*BH - CL*/
#define BH_AL_PWM_OUT_CCTRL           TMR_PWM_C_OUT_L_INV_1BOTTOM_2TOP_3OFF    /*BH - AL*/
#define CH_AL_PWM_OUT_CCTRL           TMR_PWM_C_OUT_L_INV_1BOTTOM_2OFF_3TOP    /*CH - AL*/
#define CH_BL_PWM_OUT_CCTRL           TMR_PWM_C_OUT_L_INV_1OFF_2BOTTOM_3TOP    /*CH - BL*/
#define AH_BCL_PWM_OUT_CCTRL          TMR_PWM_C_OUT_L_INV_1TOP_2BOTTOM_3BOTTOM /*AH - BCL*/
#define BRAKE_PWM_OUT_CCTRL           TMR_PWM_C_OUT_L_INV_1TOP_2TOP_3TOP
#elif defined GATE_DRIVER_LOW_SIDE_INVERT
#define AH_BL_PWM_OUT_CCTRL           TMR_PWM_OUT_L_INV_1TOP_2BOTTOM_3OFF      /*AH - BL*/
#define AH_CL_PWM_OUT_CCTRL           TMR_PWM_OUT_L_INV_1TOP_2OFF_3BOTTOM      /*AH - CL*/
#define BH_CL_PWM_OUT_CCTRL           TMR_PWM_OUT_L_INV_1OFF_2TOP_3BOTTOM      /*BH - CL*/
#define BH_AL_PWM_OUT_CCTRL           TMR_PWM_OUT_L_INV_1BOTTOM_2TOP_3OFF      /*BH - AL*/
#define CH_AL_PWM_OUT_CCTRL           TMR_PWM_OUT_L_INV_1BOTTOM_2OFF_3TOP      /*CH - AL*/
#define CH_BL_PWM_OUT_CCTRL           TMR_PWM_OUT_L_INV_1OFF_2BOTTOM_3TOP      /*CH - BL*/
#define AH_BCL_PWM_OUT_CCTRL          TMR_PWM_OUT_L_INV_1TOP_2BOTTOM_3BOTTOM   /*AH - BCL*/
#define BRAKE_PWM_OUT_CCTRL           TMR_PWM_C_OUT_L_INV_1TOP_2TOP_3TOP
#elif defined COMPLEMENT
#define AH_BL_PWM_OUT_CCTRL           TMR_PWM_C_OUT_1TOP_2BOTTOM_3OFF          /*AH - BL*/
#define AH_CL_PWM_OUT_CCTRL           TMR_PWM_C_OUT_1TOP_2OFF_3BOTTOM          /*AH - CL*/
#define BH_CL_PWM_OUT_CCTRL           TMR_PWM_C_OUT_1OFF_2TOP_3BOTTOM          /*BH - CL*/
#define BH_AL_PWM_OUT_CCTRL           TMR_PWM_C_OUT_1BOTTOM_2TOP_3OFF          /*BH - AL*/
#define CH_AL_PWM_OUT_CCTRL           TMR_PWM_C_OUT_1BOTTOM_2OFF_3TOP          /*CH - AL*/
#define CH_BL_PWM_OUT_CCTRL           TMR_PWM_C_OUT_1OFF_2BOTTOM_3TOP          /*CH - BL*/
#define AH_BCL_PWM_OUT_CCTRL          TMR_PWM_C_OUT_1TOP_2BOTTOM_3BOTTOM       /*AH - BCL*/
#define BRAKE_PWM_OUT_CCTRL           TMR_PWM_C_OUT_1TOP_2TOP_3TOP
#else
#define AH_BL_PWM_OUT_CCTRL           TMR_PWM_OUT_1TOP_2BOTTOM_3OFF            /*AH - BL*/
#define AH_CL_PWM_OUT_CCTRL           TMR_PWM_OUT_1TOP_2OFF_3BOTTOM            /*AH - CL*/
#define BH_CL_PWM_OUT_CCTRL           TMR_PWM_OUT_1OFF_2TOP_3BOTTOM            /*BH - CL*/
#define BH_AL_PWM_OUT_CCTRL           TMR_PWM_OUT_1BOTTOM_2TOP_3OFF            /*BH - AL*/
#define CH_AL_PWM_OUT_CCTRL           TMR_PWM_OUT_1BOTTOM_2OFF_3TOP            /*CH - AL*/
#define CH_BL_PWM_OUT_CCTRL           TMR_PWM_OUT_1OFF_2BOTTOM_3TOP            /*CH - BL*/
#define AH_BCL_PWM_OUT_CCTRL          TMR_PWM_OUT_1TOP_2BOTTOM_3BOTTOM         /*AH - BCL*/
#define BRAKE_PWM_OUT_CCTRL           TMR_PWM_C_OUT_1TOP_2TOP_3TOP
#endif

/**************** define pwm mode pattern for running mode ******************/
#define TMR_PWM_MODE_CM1_MASK            0x7070
#define TMR_PWM_MODE_CM2_MASK            0x0070
#define TMR_PWM_1LOW_2OFF                0x0050
#define TMR_PWM_1PWMA_2OFF               0x0060
#define TMR_PWM_1OFF_2LOW                0x5000
#define TMR_PWM_1OFF_2PWMA               0x6000
#define TMR_PWM_1LOW_2PWMA               0x6050
#define TMR_PWM_1PWMA_2LOW               0x5060
#define TMR_PWM_1PWMA_2PWMA              0x6060

#define TMR_PWM_3OFF                     0x0000
#define TMR_PWM_3LOW                     0x0050
#define TMR_PWM_3PWMA                    0x0060

#define TMR_PWM_OUT_MODE_MASK            0x0DDD
/**************** define drive output enable pattern (inverting logic input)******************/
#define TMR_PWM_OUT_L_INV_1TOP_2BOTTOM_3OFF    0x08C9
#define TMR_PWM_OUT_L_INV_1TOP_2OFF_3BOTTOM    0x0C89
#define TMR_PWM_OUT_L_INV_1BOTTOM_2TOP_3OFF    0x089C
#define TMR_PWM_OUT_L_INV_1BOTTOM_2OFF_3TOP    0x098C
#define TMR_PWM_OUT_L_INV_1OFF_2BOTTOM_3TOP    0x09C8
#define TMR_PWM_OUT_L_INV_1OFF_2TOP_3BOTTOM    0x0C98
#define TMR_PWM_OUT_L_INV_1TOP_2BOTTOM_3BOTTOM    0x0CC9

/**************** define drive output enable pattern(inverting logic input)(complement) ******************/
#define TMR_PWM_C_OUT_L_INV_1TOP_2BOTTOM_3OFF  0x08CD
#define TMR_PWM_C_OUT_L_INV_1TOP_2OFF_3BOTTOM  0x0C8D
#define TMR_PWM_C_OUT_L_INV_1BOTTOM_2TOP_3OFF  0x08DC
#define TMR_PWM_C_OUT_L_INV_1BOTTOM_2OFF_3TOP  0x0D8C
#define TMR_PWM_C_OUT_L_INV_1OFF_2BOTTOM_3TOP  0x0DC8
#define TMR_PWM_C_OUT_L_INV_1OFF_2TOP_3BOTTOM  0x0CD8

#define TMR_PWM_C_OUT_L_INV_1TOP_2BOTTOM_3BOTTOM  0x0CCD
#define TMR_PWM_C_OUT_L_INV_1TOP_2TOP_3TOP        0x0DDD

/**************** define drive output enable pattern(non-inverting logic input)******************/
#define TMR_PWM_OUT_1TOP_2BOTTOM_3OFF    0x0041
#define TMR_PWM_OUT_1TOP_2OFF_3BOTTOM    0x0401
#define TMR_PWM_OUT_1BOTTOM_2TOP_3OFF    0x0014
#define TMR_PWM_OUT_1BOTTOM_2OFF_3TOP    0x0104
#define TMR_PWM_OUT_1OFF_2BOTTOM_3TOP    0x0140
#define TMR_PWM_OUT_1OFF_2TOP_3BOTTOM    0x0410

#define TMR_PWM_OUT_1TOP_2BOTTOM_3BOTTOM    0x0441
/**************** define drive output enable pattern(non-inverting logic input)(complement) ******************/
#define TMR_PWM_C_OUT_1TOP_2BOTTOM_3OFF  0x0045
#define TMR_PWM_C_OUT_1TOP_2OFF_3BOTTOM  0x0405
#define TMR_PWM_C_OUT_1BOTTOM_2TOP_3OFF  0x0054
#define TMR_PWM_C_OUT_1BOTTOM_2OFF_3TOP  0x0504
#define TMR_PWM_C_OUT_1OFF_2BOTTOM_3TOP  0x0540
#define TMR_PWM_C_OUT_1OFF_2TOP_3BOTTOM  0x0450

#define TMR_PWM_C_OUT_1TOP_2BOTTOM_3BOTTOM  0x0445
#define TMR_PWM_C_OUT_1TOP_2TOP_3TOP     0x0555

/**************** angle initial pattern******************/
#define TMR_PWM_OUT_ANGLE_INIT_MASK      0x0555

#define ANGLE_INIT_AH_BL_CCTRL           0x0041
#define ANGLE_INIT_AH_CL_CCTRL           0x0401
#define ANGLE_INIT_BH_AL_CCTRL           0x0014
#define ANGLE_INIT_CH_AL_CCTRL           0x0104
#define ANGLE_INIT_CH_BL_CCTRL           0x0140
#define ANGLE_INIT_BH_CL_CCTRL           0x0410

/* motor parameter identify init parameter */
#define PWM_FREQ_ID                        30000  /* Hz */
#define PWM_PERIOD_ID                      ((uint16_t) (TMR_CLK/PWM_FREQ_ID/2))
#define HALF_PWM_PERIOD_ID                 ((uint16_t) (TMR_CLK/PWM_FREQ_ID/2)/2)
#define DOUBLE_PWM_PERIOD_ID               ((uint16_t) (PWM_PERIOD_ID*2-1))
#define ADC_TRIG_POS_ID                    ((uint16_t) (PWM_PERIOD_ID-1))
#define ADC_TRIG_DELAY_ID                  (0.0)   /* us */
#define ADC_TRIG_DELAY_COUNT               ((uint16_t) (ADC_TRIG_DELAY_ID*TMR_CLK/1000000))   /* ns */
#define SYSTEM_TICK_FREQ                   1000
#define ALIGN_TIME                         0.5f   /* second */
#define ALIGN_TIME_COUNT                   ((uint16_t)(ALIGN_TIME*SYSTEM_TICK_FREQ))
#define THRESHOLD_CURRENT                  ((double)(CURRENT_BASE*0.003f))
#define THRESHOLD_CURRENT_PU               ((int16_t)(0x7FFF*THRESHOLD_CURRENT/CURRENT_BASE))
#define ID_TIMEOUT                         (10000) /* ms */

#define GAIN_RATIO_MIN_VOLT                ((uint16_t) (BAT_LOW_VOLT/VOLTAGE_BASE*4095.0))
#define VBUS_GAIN_RATIO                    4096
#define VBUS_GAIN_LOG                      (LOG2(VBUS_GAIN_RATIO))
#define VDC_RATED_ADC                      ((int16_t) (VDC_RATED*ADC_DIGITAL_SCALE_12BITS/VOLTAGE_BASE))

/* auto-tune current PI parameter*/
#define PI                                 3.14159265358979f
#define CURRENT_BW_LIMIT                   ((uint16_t)(2.0f*PI*PWM_FREQ/15))

/* select pid controller function*/
#define pid_controller                     pid_controller_static_clamp   //pid_controller_dyna_clamp

/**
  * @brief  Macro to compute logarithm of two
  */
#define LOG2(x) \
  ((x) >= 65536 ? 16 : \
   ((x) >= 2*2*2*2*2*2*2*2*2*2*2*2*2*2*2 ? 15 : \
    ((x) >= 2*2*2*2*2*2*2*2*2*2*2*2*2*2 ? 14 : \
     ((x) >= 2*2*2*2*2*2*2*2*2*2*2*2*2 ? 13 : \
      ((x) >= 2*2*2*2*2*2*2*2*2*2*2*2 ? 12 : \
       ((x) >= 2*2*2*2*2*2*2*2*2*2*2 ? 11 : \
        ((x) >= 2*2*2*2*2*2*2*2*2*2 ? 10 : \
         ((x) >= 2*2*2*2*2*2*2*2*2 ? 9 : \
          ((x) >= 2*2*2*2*2*2*2*2 ? 8 : \
           ((x) >= 2*2*2*2*2*2*2 ? 7 : \
            ((x) >= 2*2*2*2*2*2 ? 6 : \
             ((x) >= 2*2*2*2*2 ? 5 : \
              ((x) >= 2*2*2*2 ? 4 : \
               ((x) >= 2*2*2 ? 3 : \
                ((x) >= 2*2 ? 2 : \
                 ((x) >= 2 ? 1 : \
                  ((x) >= 1 ? 0 : -1)))))))))))))))))

/****************  extern parameter ****************/
extern const char s_fwVer[32];
extern const adc_channel_select_type emf_detect_adc_channel[7];
extern firmware_id_type firmware_id;
extern motor_control_mode ctrl_mode;
extern err_code_type error_code, error_code_mask;
extern uint32_t tmr_pwm_channel_mode[7][2];
extern uint32_t tmr_pwm_output_mode[7];
extern uint16_t next_hall_state[2][7];
extern const uint32_t init_tmr_pwm_channel_mode[7][2];
extern const uint32_t init_tmr_pwm_output_mode[7];
extern const uint16_t init_next_hall_state[2][7];
extern const uint16_t output_hall_state[2][7];
extern const uint16_t pwm_pattern[2][7];
extern uint8_t hall_sequence_seen_states[8];
extern uint16_t hall_learn_state_table[6];
extern uint16_t hall_learn_sequence_table[6];
extern uint16_t next_hall_learn_state_table[7];
extern hall_learn_process_type hall_learn_process_state;
extern uint8_t brake_mode;
extern uint16_t emf_edge;
extern uint32_t emf_phase_adv_test; //test
#if defined AT32F421xx || defined AT32F415xx || defined AT32M412xx
extern const cmp_inverting_type emf_detect_cmp_channel[7];
extern const cmp_polarity_type emf_detect_cmp_polarity[7];
extern cmp_init_type cmp_init_struct;
#endif
/*****************  type parameter *****************/
extern speed_type rotor_speed;
extern pid_ctrl_type pid_is, pid_spd,pid_spd_volt;
extern ramp_cmd_type speed_ramp;
extern current_type current;
extern angle_init_type angle_init;
extern adc_sample_type adc_sample;
extern hall_sensor_type hall;
extern hall_learn_type hall_learn;
extern olc_type openloop;
extern lowpass_filter_type speed_LPF;
#ifdef CURRENT_LP_FILTER
extern lowpass_filter_type current_LPF;
extern int16_t ibus_filterd;
#endif
extern pwm_in_type pwm_input;
extern sound_type calibration_tone;
extern blank_trigger_type blank_trigger;
extern blank_type blank;
/* basic */
extern int16_t volt_cmd;
extern uint16_t pwm_pr;
extern int16_t pwm_comp_value;
extern ctrl_source_type ctrl_source;
extern int16_t max_current_pu;
extern int16_t min_current_pu;
extern int16_t sys_counter;
extern __IO uint16_t ic1value;
extern __IO uint16_t pre_ic1value;
extern int16_t vdc_ratio;
/* start-up */
extern int16_t start_current_cmd;
extern int16_t start_volt_cmd;
extern uint16_t start_period;
extern uint8_t lock_state;
/* usart/user interface*/
extern RCP_Frame_t rx_data_command;
extern TCP_Frame_t tx_data_response;
extern usart_data_index usart_data_idx;
extern usart_config_type ui_usart;
extern ui_wave_param_type ui_wave_param;
extern flag_status cmd_response_rdy;
extern int16_t *monitor_data[2];
extern uint8_t monitor_data_buffer[2][DATA_BUFFER_FRAME_SIZE];
extern uint8_t extra_data_buffer[DATA_BUFFER_EXTRA_SIZE];
extern uint8_t monitor_data_buffer_num;
extern uint16_t buffer_index;
extern uint8_t ui_receive_cmd_index;
extern int32_t save_param_buffer[INT32_SIZE_ARRAY];
/* measure */
extern int16_t iMosTemperature;
extern uint32_t motor_power_meas;
/* flag */
extern flag_status param_initial_rdy, curr_offset_rdy;
extern flag_status start_stop_btn_flag;
extern flag_status change_phase_flag;
extern flag_status bldc_rdy;
extern flag_status low_spd_switch_flag;
extern flag_status closeloop_rdy;
extern flag_status current_loop_ctrl;
extern flag_status const_current_ctrl;
extern flag_status calc_spd_rdy;
/* esc state */
extern esc_state_type esc_state_old;
extern esc_state_type esc_state;
extern int16_t sense_hall_steps;
extern start_state_type start_state;
/* EMF */
extern int16_t zero_cross_point;
extern int16_t zcp_lowspd_fall,zcp_lowspd_rise,zcp_highspd_fall,zcp_highspd_rise;
extern int16_t lowspd_sample_end,highspd_sample_end;
extern int16_t emf_adc_value;
extern volatile uint16_t emf_avoid_noise_counter;
extern volatile uint8_t emf_comp_state;
extern volatile int16_t emf_comp_hall[3], read_gpio_num;
extern int16_t ke_compen;
extern uint8_t EMF_switch_sample_position;
/* speed */
extern int16_t sp_value;
extern moving_average_type *interval_moving_average_fliter;
extern moving_average_type *pwm_in_average_fliter;
extern moving_average_type *spd_cmd_average_filter;
extern uint32_t spd_cval;
extern uint32_t spd_last_cval;
extern int32_t spd_total_cval;
extern int32_t i32_speed_filterd;
/* for tunning */
extern uint16_t I_tune_count;
extern int16_t i_tune_dc_volt;
extern int16_t current_tune_target_current;
extern uint16_t current_tune_total_period;
extern uint16_t current_tune_step_period;

extern i_auto_tune_type I_auto_tune;
extern motor_param_id_type motor_param_ident;
extern uint16_t vbus_gain;
extern int32_t *intCoeffs32_p;
extern uint16_t pwm_in_pulse_rising_old, pwm_in_high_width;

#ifdef __cplusplus
}
#endif

#endif
