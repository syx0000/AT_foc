/**
  **************************************************************************
  * @file     mc_lib.h
  * @brief    Unified header file management
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

#ifndef __MC_LIB_H
#define __MC_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************** Definition (DON'T EDIT) ***********************/

#include <stdlib.h>
#include "stdio.h"
#include "at32f45x.h"
#include "system_at32f45x.h"
#include "at32f45x_clock.h"
#include "at32f45x_mc.h"
#include "motor_control_drive_param.h"
#include "arm_math.h"
#include "mc_delay.h"
#include "mc_type.h"

#ifdef AT_MOTOR_EVB_V2
#include "mc_hwio_v2.h"
#elif defined AT_MOTOR_EVB_V1
#include "mc_hwio_v1.h"
#endif

#include "mc_curr_fdbk.h"
#include "mc_comm_uart.h"
#include "mc_pid_controller.h"
#include "mc_math.h"
#include "user_interface_foc.h"
#include "mc_flash_data_table.h"
#include "motor_control_foc.h"
#include "mc_foc_kernal.h"
#include "mc_esc_state.h"

#if defined INCREM_ENCODER || defined MAGNET_ENCODER_W_ABZ || defined MAGNET_ENCODER_WO_ABZ
#include "mc_encoder.h"
#elif defined HALL_SENSORS
#include "mc_hall.h"
#elif defined SENSORLESS
#include "mc_foc_sensorless.h"
#endif

#if defined MAGNET_ENCODER_W_ABZ || defined MAGNET_ENCODER_WO_ABZ
#include "AS5047P.h"
#endif

#include "mc_foc_globals.h"
#include "mc_foc.h"

#ifdef FIELD_WEAKENING
#include "mc_field_weakening.h"
#endif

#ifdef CURRENT_DECOUPLE_CTRL
#include "mc_curr_decoupling.h"
#endif

#ifdef IPM_MTPA_MTPV_CTRL
#include "MTPA_MTPV_table.h"
#endif

#if defined SIX_STEP_CONTROL && defined HALL_SENSORS
#define FIRMWARE_ID BLDC_HALL_SENSOR
#elif defined SIX_STEP_CONTROL && defined SENSORLESS && defined BLDC_SENSORLESS_ADC
#define FIRMWARE_ID BLDC_SENSOR_LESS_ADC
#elif defined SIX_STEP_CONTROL && defined SENSORLESS && defined BLDC_SENSORLESS_COMP
#define FIRMWARE_ID BLDC_SENSOR_LESS_COMP
#elif defined FOC_CONTROL && defined HALL_SENSORS && defined SPMSM
#define FIRMWARE_ID SPMSM_HALL_SENSOR
#elif defined FOC_CONTROL && defined INCREM_ENCODER && defined SPMSM
#define FIRMWARE_ID SPMSM_ENCODER
#elif defined FOC_CONTROL && defined SENSORLESS && defined OPENLOOP_STARTUP && defined SPMSM
#define FIRMWARE_ID SPMSM_SENSOR_LESS_OPEN_LOOP
#elif defined FOC_CONTROL && defined SENSORLESS && defined ALIGN_AND_GO_STARTUP && defined SPMSM
#define FIRMWARE_ID SPMSM_SENSOR_LESS_ALIGN_AND_GO
#elif defined FOC_CONTROL && defined SENSORLESS && defined INIT_ANGLE_STARTUP && defined SPMSM
#define FIRMWARE_ID SPMSM_SENSOR_LESS_ANGLE_INIT
#elif defined FOC_CONTROL && (defined MAGNET_ENCODER_W_ABZ || defined MAGNET_ENCODER_WO_ABZ) && defined SPMSM
#define FIRMWARE_ID SPMSM_ABS_ENCODER
#elif defined FOC_CONTROL && defined HALL_SENSORS && defined IPMSM
#define FIRMWARE_ID IPMSM_HALL_SENSOR
#elif defined FOC_CONTROL && defined INCREM_ENCODER && defined IPMSM
#define FIRMWARE_ID IPMSM_ENCODER
#elif defined FOC_CONTROL && (defined MAGNET_ENCODER_W_ABZ || defined MAGNET_ENCODER_WO_ABZ) && defined IPMSM
#define FIRMWARE_ID IPMSM_ABS_ENCODER
#endif

#ifdef __cplusplus
}
#endif

#endif
