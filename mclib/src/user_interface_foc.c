/**
  **************************************************************************
  * @file     user_interface_foc.c
  * @brief    Communication interface related functions
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

#include "mc_lib.h"
#include "user_interface_foc.h"

/** @addtogroup Motor_Control_Library
  * @{
  */

/** @defgroup user_interface_foc
  * @brief Communication interface related functions.
  * @{
  */

int32_t flash_buf_int[SECTOR_SIZE / 4];
int16_t flash_buf_short[SECTOR_SIZE / 2];
MC_Protocol_REG_t MonitorRegID_1, MonitorRegID_2;

/**
  * @brief  Check CRC of receving data, and save data buffer from DMA rx
  * @param  none
  * @retval none
  */
void ui_rx_receive_handler(void)
{
  uint16_t index;
  uint8_t bErrorCode;
  uint8_t* pErrorCode = &bErrorCode;

  rx_data_command.Size = usart_rx_buffer[1];
  rx_data_command.Code = usart_rx_buffer[2];

  for(index = 0; index < (rx_data_command.Size - 1); index++)
  {
    rx_data_command.Buffer[index] = usart_rx_buffer[index + RCP_FRAME_START_ADRESS];
  }

  rx_data_command.FrameCRC = usart_rx_buffer[index + RCP_FRAME_START_ADRESS];

  if(RCP_CalcCRC(&rx_data_command) == rx_data_command.FrameCRC)
  {
    cmd_response_rdy = RESET;

    if(ui_receive_cmd_index > 0x0F)
    {
      ui_receive_cmd_index = 1;
    }

    /* decode received data */
    RCP_ReceivedFrameID(&rx_data_command);
    ui_receive_cmd_index++;
  }
  else
  {
    bErrorCode = ERROR_CODE_BAD_CRC;
    TCP_SendFrame(&tx_data_response, TCP_CODE_NACK, pErrorCode, 8);
  }
}

/**
  * @brief  save and packet extra data
  * @param  none
  * @retval none
  */
void save_extra_buffer(TCP_Frame_t * pFrame)
{
  uint16_t index = 0, len = 0;

  len = pFrame -> Size_LB + (pFrame -> Size_HB << 8) + 3;

  while(index <= len)
  {
    switch (index)
    {
    case 0:
      extra_data_buffer[index] = (uint8_t) pFrame -> Code;
      break;

    case 1:
      extra_data_buffer[index] = (uint8_t) pFrame -> Size_LB;
      break;

    case 2:
      extra_data_buffer[index] = (uint8_t) pFrame -> Size_HB;
      break;

    default:
      if(index < len)
      {
        extra_data_buffer[index] = (uint8_t) pFrame -> Buffer[index - 3];
      }
      else
      {
        extra_data_buffer[index] = (uint8_t) pFrame -> FrameCRC;
      }

      break;
    }

    index++;
  }

  cmd_response_rdy = SET;
}

/**
  * @brief  Function used to decode received data
  * @param  pFrame : receive data frame
  * @retval none
  */
void RCP_ReceivedFrameID( RCP_Frame_t * pFrame )
{
  uint8_t bErrorCode;
  uint8_t* pErrorCode = &bErrorCode;
  uint8_t bCmdID;
  MC_Protocol_REG_t bRegID;
  flag_status ch1_data_adress_set_rdy, ch2_data_adress_set_rdy;
  uint8_t bData[2];
  uint8_t* pData = &bData[0];

  switch(pFrame -> Code)
  {
  case MC_PROTOCOL_CODE_GET_BOARD_INFO:
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, FW_VER_LEN_SIZE);
    break;

  case MC_PROTOCOL_CODE_EXECUTE_CMD:
    bCmdID = (uint8_t)(pFrame -> Buffer[0]);
    RCP_ExecCmd(bCmdID, pFrame);
    break;

  case MC_PROTOCOL_CODE_GET_REG:
    bRegID = (MC_Protocol_REG_t)(pFrame -> Buffer[0]);
    RCP_GetReg(bRegID, pFrame);
    break;

  case MC_PROTOCOL_CODE_SET_REG:
    bRegID = (MC_Protocol_REG_t)(pFrame -> Buffer[0]);
    RCP_SetReg(bRegID, pFrame);
    break;

  case MC_PROTOCOL_CODE_SAVE_MONITOR_DATA:
    MonitorRegID_1 = (MC_Protocol_REG_t)(pFrame -> Buffer[0]);
    MonitorRegID_2 = (MC_Protocol_REG_t)(pFrame -> Buffer[1]);
    ch1_data_adress_set_rdy = RCP_SetMonitorDataAdress(MonitorRegID_1, 0);
    ch2_data_adress_set_rdy = RCP_SetMonitorDataAdress(MonitorRegID_2, 1);

    if((ch1_data_adress_set_rdy & ch2_data_adress_set_rdy) == 1)
    {
      TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    }
    else
    {
      bErrorCode = ERROR_CODE_GET_WRITE_ONLY;
      TCP_SendFrame(&tx_data_response, TCP_CODE_NACK, pErrorCode, 8);
    }

    break;

  case MC_PROTOCOL_CODE_LOAD_MONITOR_DATA:
    *pData = MonitorRegID_1;
    *(pData+1) = MonitorRegID_2;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  default:
    bErrorCode = ERROR_BAD_FRAME_ID;
    TCP_SendFrame(&tx_data_response, TCP_CODE_NACK, pErrorCode, 8);
  }
}

/**
  * @brief  Allow to execute a command coming from the user.
  * @param  CmdID : Code of command to execute.
  *         See MC_PROTOCOL_CMD_xxx for code definition.
  * @param  pFrame : receive data frame
  * @retval none
  */
void RCP_ExecCmd(uint8_t CmdID, RCP_Frame_t * pFrame)
{
  uint8_t bErrorCode;
  uint8_t* pErrorCode = &bErrorCode;

  switch(CmdID)
  {
  case MC_PROTOCOL_CMD_START_MOTOR:
    start_stop_btn_flag = SET;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_CMD_STOP_MOTOR:
    start_stop_btn_flag = RESET;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_CMD_ENCODER_ALIGN:
#if defined INCREM_ENCODER || defined MAGNET_ENCODER_W_ABZ || defined MAGNET_ENCODER_WO_ABZ
    if (esc_state == ESC_STATE_SAFETY_READY)
    {
      encoder.align = PROCESSING;
      esc_state = ESC_STATE_ENC_ALIGN;
    }
#endif
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_CMD_WRITE_FLASH:
    write_flash_cmd();
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_CMD_FAULT_ACK:
    error_code = MC_NO_ERROR;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_CMD_WINDING_IDENTIFY:
#ifdef MOTOR_PARAM_IDENTIFY
    if (esc_state == ESC_STATE_SAFETY_READY)
    {
      motor_param_ident.state_flag = PROCESSING;
      esc_state = ESC_STATE_WINDING_PARAM_ID;
    }
#endif
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_CMD_AUTO_TUNE_CURR_PI:
    if (esc_state == ESC_STATE_SAFETY_READY)
    {
      I_auto_tune.state_flag = PROCESSING;
    }
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_CMD_HALL_LEARN:
#ifdef HALL_SENSORS
    if (esc_state == ESC_STATE_SAFETY_READY)
    {
      hall_learn.start_flag = SET;
    }
#endif
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;
  case MC_PROTOCOL_CMD_ENCODER_CALIBRATION:
#if defined MAGNET_ENCODER_W_ABZ || defined MAGNET_ENCODER_WO_ABZ
    if (esc_state == ESC_STATE_SAFETY_READY && ctrl_mode == OPEN_LOOP_CTRL)
    {
      encoder.calibrate_flag = SET;
    }
#endif
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;
//  case MC_PROTOCOL_CMD_MTPA_TABLE:
//#ifdef MTPA_DUMMY_DATA
//    mtpa.mtpa_table.state_flag = PROCESSING;
//    mtpa.mtpa_table.save_flag = FALSE;
//#endif
//#ifdef IPM_MTPA_MTPV_TABLE
//    if (esc_state == ESC_STATE_SAFETY_READY)
//    {
//      mtpa.mtpa_table.state_flag = PROCESSING;
//      mtpa.mtpa_table.save_flag = FALSE;
//    }
//#endif
//    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
//    break;

  default:
    bErrorCode = ERROR_CODE_WRONG_CMD;
    TCP_SendFrame(&tx_data_response, TCP_CODE_NACK, pErrorCode, 8);
    break;
  }
}

/**
  * @brief  Allow to execute a GetReg command coming from the user.
  * @param  reg_id : Code of register to update.
  *         See MC_PROTOCOL_REG_xxx for code definition.
  * @param  pFrame : receive data frame
  * @retval none
  */
void RCP_GetReg(MC_Protocol_REG_t RegID, RCP_Frame_t * pFrame)
{
  uint8_t bErrorCode;
  uint8_t* pErrorCode = &bErrorCode;
  uint8_t bData[10];
  uint8_t* pData = &bData[0];
  int32_t wData;

  switch(RegID)
  {
  case MC_PROTOCOL_REG_CURRENT_BASE:
    pData = (uint8_t*)&(intCoeffs32[MC_PROTOCOL_REG_CURRENT_BASE]);
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_VOLTAGE_BASE:
    pData = (uint8_t*)&(intCoeffs32[MC_PROTOCOL_REG_VOLTAGE_BASE]);
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_FIRMWARE_ID:
    pData = (uint8_t*)&(intCoeffs32[MC_PROTOCOL_REG_FIRMWARE_ID]);
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 8);
    break;

  case MC_PROTOCOL_REG_CURRENT_TUNE_TARGET_I:
    pData = (uint8_t*)&current_tune_target_current;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_CURRENT_TUNE_TOTAL_PERIOD:
    pData = (uint8_t*)&current_tune_total_period;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_CURRENT_TUNE_STEP_PERIOD:
    pData = (uint8_t*)&current_tune_step_period;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_ESC_STATUS:
    pData = (uint8_t*)&esc_state;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 8);
    break;

  case MC_PROTOCOL_REG_CONTROL_MODE:
    pData = (uint8_t*)&ctrl_mode;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 8);
    break;

  case MC_PROTOCOL_REG_CTRL_SOURCE:
    pData = (uint8_t*)&ctrl_source;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 8);
    break;

  case MC_PROTOCOL_REG_SPEED_REF:
    pData = (uint8_t*)&speed_ramp.cmd_final;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 32);
    break;

  case MC_PROTOCOL_REG_SPEED_KP:
    pData = (uint8_t*)&pid_spd.kp_gain;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_SPEED_KI:
    pData = (uint8_t*)&pid_spd.ki_gain;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_SPEED_KP_DIV:
    pData = (uint8_t*)&pid_spd.kp_shift;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_SPEED_KI_DIV:
    pData = (uint8_t*)&pid_spd.ki_shift;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_TORQUE_REF:
#if defined E_BIKE_SCOOTER || defined TORQUE_CTRL_WITH_SPEED_LIMIT
    pData = (uint8_t*)&Iq_ref_cmd;
#else
    pData = (uint8_t*)&current.Iqdref.q;
#endif
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_TORQUE_KP:
    pData = (uint8_t*)&pid_iq.kp_gain;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_TORQUE_KI:
    pData = (uint8_t*)&pid_iq.ki_gain;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_TORQUE_KP_DIV:
    pData = (uint8_t*)&pid_iq.kp_shift;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_TORQUE_KI_DIV:
    pData = (uint8_t*)&pid_iq.ki_shift;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_FLUX_REF:
    pData = (uint8_t*)&current.Iqdref.d;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_FLUX_KP:
    pData = (uint8_t*)&pid_id.kp_gain;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_FLUX_KI:
    pData = (uint8_t*)&pid_id.ki_gain;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_FLUX_KP_DIV:
    pData = (uint8_t*)&pid_id.kp_shift;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_FLUX_KI_DIV:
    pData = (uint8_t*)&pid_id.ki_shift;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_BUS_VOLTAGE_MEAS:
    pData = (uint8_t*)&ui_wave_param.iBusVoltage_meas;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_MOS_TEMP_MEAS:
    pData = (uint8_t*)&ui_wave_param.iMosTemperature_meas;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

//  case MC_PROTOCOL_REG_MOTOR_POWER_MEAS:
//    wData = 0;
//    pData = (uint8_t*)&wData;
//    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
//    break;

//  case MC_PROTOCOL_REG_MAX_MOTOR_POWER:
//    wData = 0;
//    pData = (uint8_t*)&wData;
//    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
//    break;

  case MC_PROTOCOL_REG_SPEED_MEAS:
    pData = (uint8_t*)&rotor_speed_val_filt;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 32);
    break;

  case MC_PROTOCOL_REG_TORQUE_MEAS:
    pData = (uint8_t*)&current.Iqd.q;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_FLUX_MEAS:
    pData = (uint8_t*)&current.Iqd.d;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_MAX_APP_SPEED:
    pData = (uint8_t*)&(intCoeffs32[MC_PROTOCOL_REG_MAX_APP_SPEED]);
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 32);
    break;

  case MC_PROTOCOL_REG_MIN_APP_SPEED:
    pData = (uint8_t*)&(intCoeffs32[MC_PROTOCOL_REG_MIN_APP_SPEED]);
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 32);
    break;

  case MC_PROTOCOL_REG_SPEED_ACC:
    pData = (uint8_t*)&speed_ramp.acc_slope;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_SPEED_DEC:
    pData = (uint8_t*)&speed_ramp.dec_slope;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_MAX_CURRENT:
    pData = (uint8_t*)&current.max_current;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_MIN_CURRENT:
    pData = (uint8_t*)&current.min_current;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_FLUX_WEK_KP:
    pData = (uint8_t*)&field_weakening.pid_fw.kp_gain;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_FLUX_WEK_KI:
    pData = (uint8_t*)&field_weakening.pid_fw.ki_gain;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_MAX_FLUX_WEK_I:
    pData = (uint8_t*)&field_weakening.Id_max;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_ENCODER_CAL_PROC_STATE:
    pData = (uint8_t*)&encoder.calibrate_state;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 8);
    break;

  case MC_PROTOCOL_REG_OBS_C1:
    pData = (uint8_t*)&state_observer.c2;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_OBS_C2:
    pData = (uint8_t*)&state_observer.c4;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_PLL_KP:
    pData = (uint8_t*)&state_observer.pid_pll.kp_gain;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_PLL_KI:
    pData = (uint8_t*)&state_observer.pid_pll.ki_gain;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_PLL_KP_DIV:
    pData = (uint8_t*)&state_observer.pid_pll.kp_shift;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_PLL_KI_DIV:
    pData = (uint8_t*)&state_observer.pid_pll.ki_shift;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_OPEN_LOOP_VOLT:
    pData = (uint8_t*)&openloop.volt.q;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_OPEN_LOOP_ANGLE_INCRE:
    pData = (uint8_t*)&openloop.inc;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_ENCODER_ALIGN_VOLT:
    pData = (uint8_t*)&encoder.volt.q;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_ENCODER_OFFSET:
    pData = (uint8_t*)&encoder.offset;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 32);
    break;

  case MC_PROTOCOL_REG_ERROR_CODE:
    pData = (uint8_t*)&error_code;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_IA:
    pData = (uint8_t*)&current.Iabc.a;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_IB:
    pData = (uint8_t*)&current.Iabc.b;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_IC:
    pData = (uint8_t*)&current.Iabc.c;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_I_ALPHA:
    pData = (uint8_t*)&current.Ialphabeta.alpha;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_I_BETA:
    pData = (uint8_t*)&current.Ialphabeta.beta;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_IQ_FILTER:
    pData = (uint8_t*)&current.Iqd_LPF.q;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_ID_FILTER:
    pData = (uint8_t*)&current.Iqd_LPF.d;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_VQ:
    pData = (uint8_t*)&volt_cmd.Vqd.q;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_VD:
    pData = (uint8_t*)&volt_cmd.Vqd.d;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_V_ALPHA:
    pData = (uint8_t*)&volt_cmd.Valphabeta.alpha;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_V_BETA:
    pData = (uint8_t*)&volt_cmd.Valphabeta.beta;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_ELEC_ANGLE_MEAS:
    pData = (uint8_t*)&elec_angle_val;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_ROTOR_SPEED_MEAS_FILTER:
    pData = (uint8_t*)&rotor_speed_val_filt;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_OBS_ELEC_ANGLE:
    pData = (uint8_t*)&state_observer.elec_angle;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_OBS_ROTOR_SPEED:
    pData = (uint8_t*)&state_observer.motor_speed.filtered;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_OBS_I_ALPHA:
    pData = (uint8_t*)&state_observer.hIalpha_est;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_OBS_I_BETA:
    pData = (uint8_t*)&state_observer.hIbeta_est;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_OBS_BEMF_ALPHA:
    pData = (uint8_t*)&state_observer.hBemf_alpha_est;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_OBS_BEMF_BETA:
    pData = (uint8_t*)&state_observer.hBemf_beta_est;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_VQ_REF:
    pData = (uint8_t*)&volt_cmd.Vqd.q;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_VD_REF:
    pData = (uint8_t*)&volt_cmd.Vqd.d;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_STARTUP_MAX_SPD:
    pData = (uint8_t*)&startup.max_speed;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_STARTUP_OPEN_LOOP_SLOPE:
    wData = (int32_t)(startup.elec_spd_to_rpm * obs_speed_LPF.sample_freq / startup.ol_delay_count);
    pData = (uint8_t*)&wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_STARTUP_ALIGN_TIME:
    wData = (int16_t)(startup.align_count * 1000 / obs_speed_LPF.sample_freq);
    pData = (uint8_t*)&wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_HALL_LEARN_VOLT:
    pData = (uint8_t*)&hall_learn.learn_volt;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_STARTUP_START_TIME:
    wData = (int16_t)(startup.start_count * 1000 / obs_speed_LPF.sample_freq);
    pData = (uint8_t*)&wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_FLUX_WEK_KP_DIV:
    pData = (uint8_t*)&field_weakening.pid_fw.kp_shift;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_FLUX_WEK_KI_DIV:
    pData = (uint8_t*)&field_weakening.pid_fw.ki_shift;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_STARTUP_START_CURRENT:
    pData = (uint8_t*)&startup.start_current;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_ENCODER_RESOLUTION:
    pData = (uint8_t*)&(intCoeffs32[MC_PROTOCOL_REG_ENCODER_RESOLUTION]);
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 32);
    break;

  case MC_PROTOCOL_REG_MAX_APP_ANGLE:
    pData = (uint8_t*)&(intCoeffs32[MC_PROTOCOL_REG_MAX_APP_ANGLE]);
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 32);
    break;

  case MC_PROTOCOL_REG_MIN_APP_ANGLE:
    pData = (uint8_t*)&(intCoeffs32[MC_PROTOCOL_REG_MIN_APP_ANGLE]);
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 32);
    break;

  case MC_PROTOCOL_REG_POSITION_REF:
    pData = (uint8_t*)&angle.cmd_final;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 32);
    break;

  case MC_PROTOCOL_REG_POSITION_KP:
    pData = (uint8_t*)&pid_pos.kp_gain;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_POSITION_KI:
    pData = (uint8_t*)&pid_pos.ki_gain_1st;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_POSITION_KI_STABLE:
    pData = (uint8_t*)&pid_pos.ki_gain_2nd;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_POSITION_KD:
    pData = (uint8_t*)&pid_pos.kd_gain;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_POSITION_KP_DIV:
    pData = (uint8_t*)&pid_pos.kp_shift;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_POSITION_KI_DIV:
    pData = (uint8_t*)&pid_pos.ki_shift;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_POSITION_KD_DIV:
    pData = (uint8_t*)&pid_pos.kd_shift;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_POSITION_MEAS:
    pData = (uint8_t*)&angle.val;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 32);
    break;

  case MC_PROTOCOL_REG_ENCODER_CALIBRATION:
    pData = (uint8_t*)&encoder.calibrate;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_ENCODER_ALIGN:
    pData = (uint8_t*)&encoder.align;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;
  case MC_PROTOCOL_REG_HALL_LEARN_0_STATE:
    pData = (uint8_t*)&hall_learn_state_table[0];
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_HALL_LEARN_1_STATE:
    pData = (uint8_t*)&hall_learn_state_table[1];
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_HALL_LEARN_2_STATE:
    pData = (uint8_t*)&hall_learn_state_table[2];
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_HALL_LEARN_3_STATE:
    pData = (uint8_t*)&hall_learn_state_table[3];
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_HALL_LEARN_4_STATE:
    pData = (uint8_t*)&hall_learn_state_table[4];
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_HALL_LEARN_5_STATE:
    pData = (uint8_t*)&hall_learn_state_table[5];
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_HALL_LEARN_PROCESS_STATE:
    pData = (uint8_t*)&hall_learn.process_state;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 8);
    break;

  case MC_PROTOCOL_REG_HALL_LEARN_DIR:
    pData = (uint8_t*)&hall_learn.dir;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 8);
    break;

  case MC_PROTOCOL_REG_IDENT_PROCESS_STATE:
    pData = (uint8_t*)&motor_param_ident.state_flag;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 8);
    break;

  case MC_PROTOCOL_REG_MTPA_DATA_FRAME:
    pData = (uint8_t*)&mtpa.mtpa_table;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, TCP_LEN_MTPA);
    mtpa.mtpa_table.save_flag = FALSE;
    break;

  case MC_PROTOCOL_REG_SPEED_VOLT_KP:
    pData = (uint8_t*)&pid_spd_volt.kp_gain;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_SPEED_VOLT_KI:
    pData = (uint8_t*)&pid_spd_volt.ki_gain;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_SPEED_VOLT_KP_DIV:
    pData = (uint8_t*)&pid_spd_volt.kp_shift;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_SPEED_VOLT_KI_DIV:
    pData = (uint8_t*)&pid_spd_volt.ki_shift;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_MAX_TORQUE:
    pData = (uint8_t*)&(intCoeffs32[MC_PROTOCOL_REG_MAX_TORQUE]);
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_TORQUE_REFERENCE:
#if defined E_BIKE_SCOOTER || defined TORQUE_CTRL_WITH_SPEED_LIMIT
    pData = (uint8_t*)&Iq_ref_cmd;
#else
    pData = (uint8_t*)&current.Tref;
#endif
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_SAMPLING_RATE:
    pData = (uint8_t*)&ui_wave_param.sampling_rate;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_ENCODER_CALIBRATE_VOLT:
    pData = (uint8_t*)&encoder.calibrate_volt;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_STARTUP_START_VOLT:
    pData = (uint8_t*)&startup.start_volt;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 16);
    break;

  case MC_PROTOCOL_REG_STARTUP_MODE:
    pData = (uint8_t*)&startup.startup_mode;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 8);
    break;

  case MC_PROTOCOL_REG_RS:
    pData = (uint8_t*)&motor_param_ident.Rs.u32;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 32);
    break;

  case MC_PROTOCOL_REG_LS:
    pData = (uint8_t*)&motor_param_ident.Ls.u32;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, pData, 32);
    break;

  default:
    bErrorCode = ERROR_CODE_GET_WRITE_ONLY;
    TCP_SendFrame(&tx_data_response, TCP_CODE_NACK, pErrorCode, 8);
    break;
  }
}

/**
  * @brief  Allow to execute a SetReg command coming from the user.
  * @param  reg_id : Code of register to update.
  *         See MC_PROTOCOL_REG_xxx for code definition.
  * @param  pFrame : receive data frame
  * @retval none
  */
void RCP_SetReg(MC_Protocol_REG_t RegID, RCP_Frame_t * pFrame)
{
  uint8_t bErrorCode;
  uint8_t* pErrorCode = &bErrorCode;
  int32_t wData;

  switch(RegID)
  {
  case MC_PROTOCOL_REG_CURRENT_TUNE_TARGET_I:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    current_tune_target_current = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_CURRENT_TUNE_TOTAL_PERIOD:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    current_tune_total_period = (uint16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_CURRENT_TUNE_STEP_PERIOD:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    current_tune_step_period = (uint16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_CONTROL_MODE:
    wData = pFrame -> Buffer[1];
    ctrl_mode_cmd = (motor_control_mode)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_CTRL_SOURCE:
    wData = pFrame -> Buffer[1];
    ctrl_source = (ctrl_source_type)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_SPEED_REF:
    wData = (int32_t)(pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8) + ((pFrame -> Buffer[3]) << 16) + ((pFrame -> Buffer[4]) << 24));
#if defined E_BIKE_SCOOTER
    if(wData < 0)
    {
      bErrorCode = ERROR_CODE_WRONG_SET;
      TCP_SendFrame(&tx_data_response, TCP_CODE_NACK, pErrorCode, 8);
    }
    else
#elif defined SENSORLESS
    if (abs(wData)< MIN_SPEED_RPM)
    {
      bErrorCode = ERROR_CODE_WRONG_SET;
      TCP_SendFrame(&tx_data_response, TCP_CODE_NACK, pErrorCode, 8);
    }
    else
#endif
    {
      speed_ramp.cmd_final = (int32_t)wData;
      TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    }
    break;

  case MC_PROTOCOL_REG_SPEED_KP:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    pid_spd.kp_gain = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_SPEED_KI:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    pid_spd.ki_gain = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_SPEED_KP_DIV:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    pid_spd.kp_shift = (uint16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_SPEED_KI_DIV:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    pid_spd.ki_shift = (uint16_t)wData;
    pid_spd.upper_limit_integral = (int32_t)(pid_spd.upper_limit_output << pid_spd.ki_shift);
    pid_spd.lower_limit_integral = (int32_t)(-(-pid_spd.lower_limit_output << pid_spd.ki_shift));
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_TORQUE_REF:
    wData = (int16_t)(pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8));

    /* current cmd limitation */
#if defined E_BIKE_SCOOTER
    if(wData > current.max_current || wData < 0)
#else
    if(wData > current.max_current || wData < current.min_current)
#endif
    {
      bErrorCode = ERROR_CODE_WRONG_SET;
      TCP_SendFrame(&tx_data_response, TCP_CODE_NACK, pErrorCode, 8);
    }
    else
    {
#if defined E_BIKE_SCOOTER || defined TORQUE_CTRL_WITH_SPEED_LIMIT
      Iq_ref_cmd = (int16_t)wData;
#else
      current.Iqdref.q  = (int16_t)wData;
#endif
      TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    }

    break;

  case MC_PROTOCOL_REG_TORQUE_KP:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    pid_iq.kp_gain = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_TORQUE_KI:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    pid_iq.ki_gain = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_TORQUE_KP_DIV:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    pid_iq.kp_shift = (uint16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_TORQUE_KI_DIV:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    pid_iq.ki_shift = (uint16_t)wData;
    pid_iq.upper_limit_integral = (int32_t)(pid_iq.upper_limit_output << pid_iq.ki_shift);
    pid_iq.lower_limit_integral = (int32_t)(-(-pid_iq.lower_limit_output << pid_iq.ki_shift));
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_FLUX_REF:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    current.Iqdref.d = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_FLUX_KP:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    pid_id.kp_gain = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_FLUX_KI:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    pid_id.ki_gain = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_FLUX_KP_DIV:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    pid_id.kp_shift = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_FLUX_KI_DIV:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    pid_id.ki_shift = (int16_t)wData;
    pid_id.upper_limit_integral = (int32_t)(pid_id.upper_limit_output << pid_id.ki_shift);
    pid_id.lower_limit_integral = (int32_t)(-(-pid_id.lower_limit_output << pid_id.ki_shift));
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

//    case MC_PROTOCOL_REG_MAX_MOTOR_POWER:
//        wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
//        break;
  case MC_PROTOCOL_REG_SPEED_ACC:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    speed_ramp.acc_slope = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_SPEED_DEC:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    speed_ramp.dec_slope = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_MAX_CURRENT:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    current.max_current = (int16_t)wData;
#ifndef IPM_MTPA_MTPV_CTRL
    pid_spd.upper_limit_output = (int16_t)wData;
    pid_spd.upper_limit_integral = (int32_t)(pid_spd.upper_limit_output<< pid_spd.ki_shift);
#endif
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_MIN_CURRENT:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    current.min_current = (int16_t)wData;
#ifndef IPM_MTPA_MTPV_CTRL
    pid_spd.lower_limit_output = (int16_t)wData;
    pid_spd.lower_limit_integral = (int32_t)(-(-pid_spd.lower_limit_output << pid_spd.ki_shift));
#endif
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_FLUX_WEK_KP:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    field_weakening.pid_fw.kp_gain = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_FLUX_WEK_KI:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    field_weakening.pid_fw.ki_gain = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_MAX_FLUX_WEK_I:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    field_weakening.Id_max = (int16_t)wData;
    field_weakening.pid_fw.lower_limit_output = (int16_t) - wData;
    field_weakening.pid_fw.lower_limit_integral = (int32_t) - (wData << field_weakening.pid_fw.ki_shift);
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_OBS_C1:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    state_observer.c2 = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_OBS_C2:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    state_observer.c4 = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_PLL_KP:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    state_observer.pid_pll.kp_gain = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_PLL_KI:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    state_observer.pid_pll.ki_gain = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_PLL_KP_DIV:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    state_observer.pid_pll.kp_shift = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_PLL_KI_DIV:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    state_observer.pid_pll.ki_shift = (int16_t)wData;
    state_observer.pid_pll.upper_limit_integral = (int32_t)(INT16_MAX << state_observer.pid_pll.ki_shift);
    state_observer.pid_pll.lower_limit_integral = (int32_t)(-(INT16_MAX << state_observer.pid_pll.ki_shift));
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_OPEN_LOOP_VOLT:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    openloop.volt.q = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_OPEN_LOOP_ANGLE_INCRE:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);;
    openloop.inc = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_ENCODER_ALIGN_VOLT:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    encoder.volt.q = (int16_t)wData;
    encoder.volt.d = (int16_t)(wData * 2);
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_VQ_REF:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    volt_cmd.Vqd.q = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_VD_REF:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    volt_cmd.Vqd.d = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_STARTUP_MAX_SPD:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    startup.max_speed = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_STARTUP_OPEN_LOOP_SLOPE:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    startup.ol_delay_count = ((uint16_t)((uint32_t)startup.elec_spd_to_rpm * obs_speed_LPF.sample_freq / wData));
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_HALL_LEARN_VOLT:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    hall_learn.learn_volt = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_STARTUP_ALIGN_TIME:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    startup.align_count = ((uint32_t)wData * obs_speed_LPF.sample_freq / 1000);
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_STARTUP_START_TIME:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    startup.start_count = ((uint32_t)wData * obs_speed_LPF.sample_freq / 1000);
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_FLUX_WEK_KP_DIV:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    field_weakening.pid_fw.kp_shift = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_FLUX_WEK_KI_DIV:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    field_weakening.pid_fw.ki_shift = (int16_t)wData;
    field_weakening.pid_fw.lower_limit_integral = (int32_t)(-(-field_weakening.pid_fw.lower_limit_output << field_weakening.pid_fw.ki_shift));
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_STARTUP_START_CURRENT:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    startup.start_current = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);

  case MC_PROTOCOL_REG_POSITION_REF:
    wData = (int32_t)(pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8) + ((pFrame -> Buffer[3]) << 16) + ((pFrame -> Buffer[4]) << 24));
    angle.cmd_final = (int32_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_POSITION_KP:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    pid_pos.kp_gain = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_POSITION_KI:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    pid_pos.ki_gain_1st = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_POSITION_KI_STABLE:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    pid_pos.ki_gain_2nd = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_POSITION_KD:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    pid_pos.kd_gain = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_POSITION_KP_DIV:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    pid_pos.kp_shift = (uint16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_POSITION_KI_DIV:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    pid_pos.ki_shift = (uint16_t)wData;
    pid_pos.upper_limit_integral = (int32_t)(pid_pos.upper_limit_output << pid_pos.ki_shift);
    pid_pos.lower_limit_integral = (int32_t)(-(-pid_pos.lower_limit_output << pid_pos.ki_shift));
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_POSITION_KD_DIV:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    pid_pos.kd_shift = (uint16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_SPEED_VOLT_KP:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    pid_spd_volt.kp_gain = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_SPEED_VOLT_KI:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    pid_spd_volt.ki_gain = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_SPEED_VOLT_KP_DIV:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    pid_spd_volt.kp_shift = (uint16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_SPEED_VOLT_KI_DIV:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    pid_spd_volt.ki_shift = (uint16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_TORQUE_REFERENCE:
    wData = (int16_t)(pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8));
#if defined E_BIKE_SCOOTER || defined TORQUE_CTRL_WITH_SPEED_LIMIT
    Iq_ref_cmd = (int16_t)wData;
#else
    current.Tref = (int16_t)wData;
#endif
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_ENCODER_CALIBRATE_VOLT:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    encoder.calibrate_volt = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  case MC_PROTOCOL_REG_STARTUP_START_VOLT:
    wData = pFrame -> Buffer[1] + ((pFrame -> Buffer[2]) << 8);
    startup.start_volt = (int16_t)wData;
    TCP_SendFrame(&tx_data_response, TCP_CODE_ACK, 0, 0);
    break;

  default:
    bErrorCode = ERROR_CODE_GET_WRITE_ONLY;
    TCP_SendFrame(&tx_data_response, TCP_CODE_NACK, pErrorCode, 8);
    break;
  }
}

/**
  * @brief  Save monitor data adress
  * @param  reg_id : register ID
  * @param  channel : waveform drawing channel(0 or 1)
  * @retval status of saving data adress, the returned value can be:SET or RESET.
  */
flag_status RCP_SetMonitorDataAdress(MC_Protocol_REG_t reg_id, uint8_t channel)
{
  uint8_t bErrorCode;
  uint8_t* pErrorCode = &bErrorCode;

  switch(reg_id)
  {
  case MC_PROTOCOL_REG_TORQUE_REF:
    monitor_data[channel] = &current.Iqdref.q;
    break;

  case MC_PROTOCOL_REG_FLUX_REF:
    monitor_data[channel] = &current.Iqdref.d;
    break;

  case MC_PROTOCOL_REG_BUS_VOLTAGE_MEAS:
    monitor_data[channel] = &ui_wave_param.iBusVoltage_meas;
    break;

  case MC_PROTOCOL_REG_MOS_TEMP_MEAS:
    monitor_data[channel] = &ui_wave_param.iMosTemperature_meas;
    break;

  case MC_PROTOCOL_REG_TORQUE_MEAS:
    monitor_data[channel] = &current.Iqd.q;
    break;

  case MC_PROTOCOL_REG_FLUX_MEAS:
    monitor_data[channel] = &current.Iqd.d;
    break;

  case MC_PROTOCOL_REG_IA:
    monitor_data[channel] = &current.Iabc.a;
    break;

  case MC_PROTOCOL_REG_IB:
    monitor_data[channel] = &current.Iabc.b;
    break;

  case MC_PROTOCOL_REG_IC:
    monitor_data[channel] = &current.Iabc.c;
    break;

  case MC_PROTOCOL_REG_I_ALPHA:
    monitor_data[channel] = &current.Ialphabeta.alpha;
    break;

  case MC_PROTOCOL_REG_I_BETA:
    monitor_data[channel] = &current.Ialphabeta.beta;
    break;

  case MC_PROTOCOL_REG_IQ_FILTER:
    monitor_data[channel] = &current.Iqd_LPF.q;
    break;

  case MC_PROTOCOL_REG_ID_FILTER:
    monitor_data[channel] = &current.Iqd_LPF.d;
    break;

  case MC_PROTOCOL_REG_VQ:
    monitor_data[channel] = &volt_cmd.Vqd.q;
    break;

  case MC_PROTOCOL_REG_VD:
    monitor_data[channel] = &volt_cmd.Vqd.d;
    break;

  case MC_PROTOCOL_REG_V_ALPHA:
    monitor_data[channel] = &volt_cmd.Valphabeta.alpha;
    break;

  case MC_PROTOCOL_REG_V_BETA:
    monitor_data[channel] = &volt_cmd.Valphabeta.beta;
    break;

  case MC_PROTOCOL_REG_ELEC_ANGLE_MEAS:
    monitor_data[channel] = &elec_angle_val;
    break;

  case MC_PROTOCOL_REG_ROTOR_SPEED_MEAS_FILTER:
    monitor_data[channel] = (int16_t*)(&rotor_speed_val_filt);
    break;

  case MC_PROTOCOL_REG_OBS_ELEC_ANGLE:
    monitor_data[channel] = &state_observer.elec_angle;
    break;

  case MC_PROTOCOL_REG_OBS_ROTOR_SPEED:
    monitor_data[channel] = (int16_t*)(&state_observer.motor_speed.filtered);
    break;

  case MC_PROTOCOL_REG_OBS_I_ALPHA:
    monitor_data[channel] = &state_observer.hIalpha_est;
    break;

  case MC_PROTOCOL_REG_OBS_I_BETA:
    monitor_data[channel] = &state_observer.hIbeta_est;
    break;

  case MC_PROTOCOL_REG_OBS_BEMF_ALPHA:
    monitor_data[channel] = &state_observer.hBemf_alpha_est;
    break;

  case MC_PROTOCOL_REG_OBS_BEMF_BETA:
    monitor_data[channel] = &state_observer.hBemf_beta_est;
    break;

  case MC_PROTOCOL_REG_SPD_REF_PU:
    monitor_data[channel] = &ui_wave_param.speed_reference_pu;
    break;

  case MC_PROTOCOL_REG_SPD_MEAS_PU:
    monitor_data[channel] = &ui_wave_param.speed_meas_filter_pu;
    break;

  case MC_PROTOCOL_REG_USER_DEFINED_A:
    /* modified here to change monitor parameter 1 */
    monitor_data[channel] = &ui_wave_param.user_define_a;
    break;

  case MC_PROTOCOL_REG_USER_DEFINED_B:
    /* modified here to change monitor parameter 2 */
    monitor_data[channel] = &ui_wave_param.user_define_b;
    break;

  case MC_PROTOCOL_REG_POS_REF_PU:
    monitor_data[channel] = &ui_wave_param.position_reference_pu;
    break;

  case MC_PROTOCOL_REG_POS_MEAS_PU:
    monitor_data[channel] = &ui_wave_param.position_meas_pu;
    break;

  case MC_PROTOCOL_REG_TORQUE_REFERENCE:
    monitor_data[channel] = &current.Tref;
    break;

  default:
    bErrorCode = ERROR_CODE_GET_WRITE_ONLY;
    TCP_SendFrame(&tx_data_response, TCP_CODE_NACK, pErrorCode, 8);
    return (RESET);
  }

  return (SET);
}

/**
  * @brief  Save data in the flash memory
  * @param  none
  * @retval none
  */
void write_flash_cmd()
{
  save_param_buffer[MC_PROTOCOL_REG_NONE] = 0;
  save_param_buffer[MC_PROTOCOL_REG_CURRENT_BASE] = (int32_t)(CURRENT_BASE * 100);
  save_param_buffer[MC_PROTOCOL_REG_VOLTAGE_BASE] = (int32_t) (VOLTAGE_BASE * 100);
  save_param_buffer[MC_PROTOCOL_REG_FIRMWARE_ID] = firmware_id;
  save_param_buffer[MC_PROTOCOL_REG_CURRENT_TUNE_TARGET_I] = current_tune_target_current;
  save_param_buffer[MC_PROTOCOL_REG_CURRENT_TUNE_TOTAL_PERIOD] = current_tune_total_period;
  save_param_buffer[MC_PROTOCOL_REG_CURRENT_TUNE_STEP_PERIOD] = current_tune_step_period;
  save_param_buffer[MC_PROTOCOL_REG_ESC_STATUS] = esc_state;
  save_param_buffer[MC_PROTOCOL_REG_CONTROL_MODE] = 0; //ctrl_mode;
  save_param_buffer[MC_PROTOCOL_REG_CTRL_SOURCE] = ctrl_source;
  save_param_buffer[MC_PROTOCOL_REG_SPEED_REF] = speed_ramp.cmd_final;
  save_param_buffer[MC_PROTOCOL_REG_SPEED_KP] = pid_spd.kp_gain;
  save_param_buffer[MC_PROTOCOL_REG_SPEED_KI] = pid_spd.ki_gain;
  save_param_buffer[MC_PROTOCOL_REG_SPEED_KP_DIV] = pid_spd.kp_shift;
  save_param_buffer[MC_PROTOCOL_REG_SPEED_KI_DIV] = pid_spd.ki_shift;
  save_param_buffer[MC_PROTOCOL_REG_TORQUE_REF] = current.Iqdref.q;
  save_param_buffer[MC_PROTOCOL_REG_FLUX_REF] = current.Iqdref.d;
  save_param_buffer[MC_PROTOCOL_REG_TORQUE_KP] = pid_iq.kp_gain;
  save_param_buffer[MC_PROTOCOL_REG_TORQUE_KI] = pid_iq.ki_gain;
  save_param_buffer[MC_PROTOCOL_REG_TORQUE_KP_DIV] = pid_iq.kp_shift;
  save_param_buffer[MC_PROTOCOL_REG_TORQUE_KI_DIV] = pid_iq.ki_shift;
  save_param_buffer[MC_PROTOCOL_REG_FLUX_KP] = pid_id.kp_gain;
  save_param_buffer[MC_PROTOCOL_REG_FLUX_KI] = pid_iq.ki_gain;
  save_param_buffer[MC_PROTOCOL_REG_FLUX_KP_DIV] = pid_id.kp_shift;
  save_param_buffer[MC_PROTOCOL_REG_FLUX_KI_DIV] = pid_id.ki_shift;
  save_param_buffer[MC_PROTOCOL_REG_BUS_VOLTAGE_MEAS] = 0;
  save_param_buffer[MC_PROTOCOL_REG_MOS_TEMP_MEAS] = 0;
  save_param_buffer[MC_PROTOCOL_REG_MOTOR_POWER_MEAS] = 0;
  save_param_buffer[MC_PROTOCOL_REG_MAX_MOTOR_POWER] = 0;
  save_param_buffer[MC_PROTOCOL_REG_SPEED_MEAS] = rotor_speed_val_filt;
  save_param_buffer[MC_PROTOCOL_REG_TORQUE_MEAS] = current.Iqd.q;
  save_param_buffer[MC_PROTOCOL_REG_FLUX_MEAS] = current.Iqd.d;
  save_param_buffer[MC_PROTOCOL_REG_MAX_APP_SPEED] = MAX_SPEED_RPM;
  save_param_buffer[MC_PROTOCOL_REG_MIN_APP_SPEED] = MIN_SPEED_RPM;
  save_param_buffer[MC_PROTOCOL_REG_SPEED_ACC] = speed_ramp.acc_slope;
  save_param_buffer[MC_PROTOCOL_REG_SPEED_DEC] = speed_ramp.dec_slope;
  save_param_buffer[MC_PROTOCOL_REG_MAX_CURRENT] = current.max_current;
  save_param_buffer[MC_PROTOCOL_REG_MIN_CURRENT] = current.min_current;
  save_param_buffer[MC_PROTOCOL_REG_START_CURRENT] = 0;
  save_param_buffer[MC_PROTOCOL_REG_START_PERIOD] = 0;
  save_param_buffer[MC_PROTOCOL_REG_EMF_OFF_OFFSET_RISE] = 0;
  save_param_buffer[MC_PROTOCOL_REG_EMF_OFF_OFFSET_FALL] = 0;
  save_param_buffer[MC_PROTOCOL_REG_FLUX_WEK_KP] = field_weakening.pid_fw.kp_gain;
  save_param_buffer[MC_PROTOCOL_REG_FLUX_WEK_KI] = field_weakening.pid_fw.ki_gain;
  save_param_buffer[MC_PROTOCOL_REG_MAX_FLUX_WEK_I] = field_weakening.Id_max;
  save_param_buffer[MC_PROTOCOL_REG_OBS_C1] = state_observer.c2;
  save_param_buffer[MC_PROTOCOL_REG_OBS_C2] = state_observer.c4;
  save_param_buffer[MC_PROTOCOL_REG_PLL_KP] = state_observer.pid_pll.kp_gain;
  save_param_buffer[MC_PROTOCOL_REG_PLL_KI] = state_observer.pid_pll.ki_gain;
  save_param_buffer[MC_PROTOCOL_REG_PLL_KP_DIV] = state_observer.pid_pll.kp_shift;
  save_param_buffer[MC_PROTOCOL_REG_PLL_KI_DIV] = state_observer.pid_pll.ki_shift;
  save_param_buffer[MC_PROTOCOL_REG_OPEN_LOOP_VOLT] = openloop.volt.q;
  save_param_buffer[MC_PROTOCOL_REG_OPEN_LOOP_ANGLE_INCRE] = openloop.inc;
  save_param_buffer[MC_PROTOCOL_REG_ENCODER_ALIGN_VOLT] = encoder.volt.q;
  save_param_buffer[MC_PROTOCOL_REG_ERROR_CODE] = 0;
  save_param_buffer[MC_PROTOCOL_REG_IA] = 0;
  save_param_buffer[MC_PROTOCOL_REG_IB] = 0;
  save_param_buffer[MC_PROTOCOL_REG_IC] = 0;
  save_param_buffer[MC_PROTOCOL_REG_I_ALPHA] = 0;
  save_param_buffer[MC_PROTOCOL_REG_I_BETA] = 0;
  save_param_buffer[MC_PROTOCOL_REG_IQ_FILTER] = 0;
  save_param_buffer[MC_PROTOCOL_REG_ID_FILTER] = 0;
  save_param_buffer[MC_PROTOCOL_REG_VQ] = 0;
  save_param_buffer[MC_PROTOCOL_REG_VD] = 0;
  save_param_buffer[MC_PROTOCOL_REG_V_ALPHA] = 0;
  save_param_buffer[MC_PROTOCOL_REG_V_BETA] = 0;
  save_param_buffer[MC_PROTOCOL_REG_ELEC_ANGLE_MEAS] = 0;
  save_param_buffer[MC_PROTOCOL_REG_ROTOR_SPEED_MEAS_FILTER] = 0;
  save_param_buffer[MC_PROTOCOL_REG_OBS_ELEC_ANGLE] = 0;
  save_param_buffer[MC_PROTOCOL_REG_OBS_ROTOR_SPEED] = 0;
  save_param_buffer[MC_PROTOCOL_REG_OBS_I_ALPHA] = 0;
  save_param_buffer[MC_PROTOCOL_REG_OBS_I_BETA] = 0;
  save_param_buffer[MC_PROTOCOL_REG_OBS_BEMF_ALPHA] = 0;
  save_param_buffer[MC_PROTOCOL_REG_OBS_BEMF_BETA] = 0;
  save_param_buffer[MC_PROTOCOL_REG_VQ_REF] = volt_cmd.Vqd.q;
  save_param_buffer[MC_PROTOCOL_REG_VD_REF] = volt_cmd.Vqd.d;
  save_param_buffer[MC_PROTOCOL_REG_STARTUP_MAX_SPD] = startup.max_speed ;
  save_param_buffer[MC_PROTOCOL_REG_I_TUNE_VDC_RATE] = I_auto_tune.Vdc_rated_adc;
  save_param_buffer[MC_PROTOCOL_REG_STARTUP_OPEN_LOOP_SLOPE] = ((uint16_t)((uint32_t)startup.elec_spd_to_rpm * obs_speed_LPF.sample_freq / startup.ol_delay_count));
  save_param_buffer[MC_PROTOCOL_REG_STARTUP_ALIGN_TIME] = (int16_t)(startup.align_count * 1000 / obs_speed_LPF.sample_freq);
  save_param_buffer[MC_PROTOCOL_REG_HALL_LEARN_VOLT] = hall_learn.learn_volt;
  save_param_buffer[MC_PROTOCOL_REG_STARTUP_START_TIME] = (int16_t)(startup.start_count * 1000 / obs_speed_LPF.sample_freq);
  save_param_buffer[MC_PROTOCOL_REG_FLUX_WEK_KP_DIV] = field_weakening.pid_fw.kp_shift;
  save_param_buffer[MC_PROTOCOL_REG_FLUX_WEK_KI_DIV] = field_weakening.pid_fw.ki_shift;
  save_param_buffer[MC_PROTOCOL_REG_EMF_ON_OFFSET_RISE] = 0;
  save_param_buffer[MC_PROTOCOL_REG_EMF_ON_OFFSET_FALL] = 0;
  save_param_buffer[MC_PROTOCOL_REG_STARTUP_START_CURRENT] = (int32_t)startup.start_current;
  save_param_buffer[MC_PROTOCOL_REG_SPD_REF_PU] = 0;
  save_param_buffer[MC_PROTOCOL_REG_SPD_MEAS_PU] = 0;
  save_param_buffer[MC_PROTOCOL_REG_USER_DEFINED_A] = 0;
  save_param_buffer[MC_PROTOCOL_REG_USER_DEFINED_B] = 0;
  save_param_buffer[MC_PROTOCOL_REG_ENCODER_RESOLUTION] = ENC_CPR_NBR;
  save_param_buffer[MC_PROTOCOL_REG_MAX_APP_ANGLE] = MAX_POSITION_ANGLE;
  save_param_buffer[MC_PROTOCOL_REG_MIN_APP_ANGLE] = MIN_POSITION_ANGLE;
  save_param_buffer[MC_PROTOCOL_REG_POS_REF_PU] = 0;
  save_param_buffer[MC_PROTOCOL_REG_POS_MEAS_PU] = 0;
  save_param_buffer[MC_PROTOCOL_REG_POSITION_REF] = angle.cmd_final;
  save_param_buffer[MC_PROTOCOL_REG_POSITION_KP] = pid_pos.kp_gain;
  save_param_buffer[MC_PROTOCOL_REG_POSITION_KI] = pid_pos.ki_gain_1st;
  save_param_buffer[MC_PROTOCOL_REG_POSITION_KI_STABLE] = pid_pos.ki_gain_2nd;
  save_param_buffer[MC_PROTOCOL_REG_POSITION_KD] = pid_pos.kd_gain;
  save_param_buffer[MC_PROTOCOL_REG_POSITION_KP_DIV] = pid_pos.kp_shift;
  save_param_buffer[MC_PROTOCOL_REG_POSITION_KI_DIV] = pid_pos.ki_shift;
  save_param_buffer[MC_PROTOCOL_REG_POSITION_KD_DIV] = pid_pos.kd_shift;
  save_param_buffer[MC_PROTOCOL_REG_POSITION_MEAS] = angle.val;
#if defined MAGNET_ENCODER_W_ABZ || defined MAGNET_ENCODER_WO_ABZ
  save_param_buffer[MC_PROTOCOL_REG_ENCODER_CALIBRATION] = encoder.calibrate;
  save_param_buffer[MC_PROTOCOL_REG_ENCODER_ALIGN] = encoder.align;
  save_param_buffer[MC_PROTOCOL_REG_ENCODER_OFFSET] = encoder.offset;
  save_param_buffer[MC_PROTOCOL_REG_ENCODER_CALIBRATE_VOLT] = encoder.calibrate_volt;
#endif
  save_param_buffer[MC_PROTOCOL_REG_HALL_LEARN_0_STATE] = hall_learn_state_table[0];
  save_param_buffer[MC_PROTOCOL_REG_HALL_LEARN_1_STATE] = hall_learn_state_table[1];
  save_param_buffer[MC_PROTOCOL_REG_HALL_LEARN_2_STATE] = hall_learn_state_table[2];
  save_param_buffer[MC_PROTOCOL_REG_HALL_LEARN_3_STATE] = hall_learn_state_table[3];
  save_param_buffer[MC_PROTOCOL_REG_HALL_LEARN_4_STATE] = hall_learn_state_table[4];
  save_param_buffer[MC_PROTOCOL_REG_HALL_LEARN_5_STATE] = hall_learn_state_table[5];
  save_param_buffer[MC_PROTOCOL_REG_HALL_LEARN_DIR] = hall_learn.dir;

  save_param_buffer[MC_PROTOCOL_REG_MTPA_DATA_FRAME] = mtpa.mtpa_table.state_flag;
  save_param_buffer[MC_PROTOCOL_REG_MAX_TORQUE] = (int32_t)(MTPA_MTPV_TABLE_MAX_TORQUE*100);
  save_param_buffer[MC_PROTOCOL_REG_TORQUE_REFERENCE] = current.Tref;

  save_param_buffer[MC_PROTOCOL_REG_IDENT_PROCESS_STATE] = motor_param_ident.state_flag;
  save_param_buffer[MC_PROTOCOL_REG_SPEED_VOLT_KP] = pid_spd_volt.kp_gain;
  save_param_buffer[MC_PROTOCOL_REG_SPEED_VOLT_KI] = pid_spd_volt.ki_gain;
  save_param_buffer[MC_PROTOCOL_REG_SPEED_VOLT_KP_DIV] = pid_spd_volt.kp_shift;
  save_param_buffer[MC_PROTOCOL_REG_SPEED_VOLT_KI_DIV] = pid_spd_volt.ki_shift;
  save_param_buffer[MC_PROTOCOL_REG_STARTUP_START_VOLT] = startup.start_volt;
  save_param_buffer[MC_PROTOCOL_REG_RS] = motor_param_ident.Rs.u32;
  save_param_buffer[MC_PROTOCOL_REG_LS] = motor_param_ident.Ls.u32;
  /* write data to flash */
  flash_write_int(MC_VectStoreAddr_UINT32, save_param_buffer, INT32_SIZE_ARRAY);
}

/**
  * @brief  read data using halfword mode
  * @param  read_addr: the address of reading
  * @param  p_buffer: the buffer of reading data
  * @param  num_read: the number of reading data
  * @retval none
  */
void flash_read(uint32_t read_addr, int32_t *p_buffer, uint16_t num_read)
{
  uint16_t i;
  for(i = 0; i < num_read; i++)
  {
    p_buffer[i] = *(int32_t*)(read_addr);
    read_addr += 4;
  }
}

/**
  * @brief  write data using word mode without checking
  * @param  write_addr: the address of writing
  * @param  p_buffer: the buffer of writing data
  * @param  num_write: the number of writing data
  * @retval none
  */
void flash_write_nocheck(uint32_t write_addr, int32_t *p_buffer, uint16_t num_write)
{
  uint16_t i;

  for(i = 0; i < num_write; i++)
  {
    flash_word_program(write_addr, p_buffer[i]);
    write_addr += 4;
  }
}

/**
  * @brief  write data using word mode without checking
  * @param  write_addr: the address of writing
  * @param  p_buffer: the buffer of writing data
  * @param  num_write: the number of writing data
  * @retval none
  */
void flash_write_nocheck_short(uint32_t write_addr, int16_t *p_buffer, uint16_t num_write)
{
  uint16_t i;

  for(i = 0; i < num_write; i++)
  {
    flash_halfword_program(write_addr, p_buffer[i]);
    write_addr += 2;
  }
}

/**
  * @brief  write data using halfword mode with checking
  * @param  write_addr: the address of writing
  * @param  p_buffer: the buffer of writing data
  * @param  num_write: the number of writing data
  * @retval none
  */
void flash_write_int(uint32_t write_addr, int32_t *p_buffer, uint16_t num_write)
{
  uint32_t offset_addr;
  uint32_t sector_position;
  uint16_t sector_offset;
  uint16_t sector_remain;
  uint16_t i;

  flash_unlock();
  offset_addr = write_addr - FLASH_BASE;
  sector_position = offset_addr / SECTOR_SIZE;
  sector_offset = (offset_addr % SECTOR_SIZE) / 4;
  sector_remain = SECTOR_SIZE / 4 - sector_offset;

  if(num_write <= sector_remain)
  {
    sector_remain = num_write;
  }

  while(1)
  {
    for(i = 0; i < sector_remain; i++)
    {
      if(flash_buf_int[sector_offset + i] != 0xFFFFFFFF)
      {
        break;
      }
    }

    if(i < sector_remain)
    {
      flash_sector_erase(sector_position * SECTOR_SIZE + FLASH_BASE);

      for(i = 0; i < sector_remain; i++)
      {
        flash_buf_int[i + sector_offset] = p_buffer[i];
      }

      flash_write_nocheck(sector_position * SECTOR_SIZE + FLASH_BASE, flash_buf_int, SECTOR_SIZE / 4);
    }
    else
    {
      flash_write_nocheck(write_addr, p_buffer, sector_remain);
    }

    if(num_write == sector_remain)
    {
      break;
    }
    else
    {
      sector_position++;
      sector_offset = 0;
      p_buffer += sector_remain;
      write_addr += (sector_remain * 4);
      num_write -= sector_remain;

      if(num_write > (SECTOR_SIZE / 4))
      {
        sector_remain = SECTOR_SIZE / 4;
      }
      else
      {
        sector_remain = num_write;
      }
    }
  }

  flash_lock();
}

/**
  * @brief  write data using halfword mode with checking
  * @param  write_addr: the address of writing
  * @param  p_buffer: the buffer of writing data
  * @param  num_write: the number of writing data
  * @retval none
  */
void flash_write_short(uint32_t write_addr, int16_t *p_buffer, uint16_t num_write)
{
  uint32_t offset_addr;
  uint32_t sector_position;
  uint16_t sector_offset;
  uint16_t sector_remain;
  uint16_t i;

  flash_unlock();
  offset_addr = write_addr - FLASH_BASE;
  sector_position = offset_addr / SECTOR_SIZE;
  sector_offset = (offset_addr % SECTOR_SIZE) / 2;
  sector_remain = SECTOR_SIZE / 2 - sector_offset;

  if(num_write <= sector_remain)
  {
    sector_remain = num_write;
  }

  while(1)
  {
    for(i = 0; i < sector_remain; i++)
    {
      if(flash_buf_short[sector_offset + i] != 0x7FFF)
      {
        break;
      }
    }

    if(i < sector_remain)
    {
      flash_sector_erase(sector_position * SECTOR_SIZE + FLASH_BASE);

      for(i = 0; i < sector_remain; i++)
      {
        flash_buf_short[i + sector_offset] = p_buffer[i];
      }

      flash_write_nocheck_short(sector_position * SECTOR_SIZE + FLASH_BASE, flash_buf_short, SECTOR_SIZE / 2);
    }
    else
    {
      flash_write_nocheck_short(write_addr, p_buffer, sector_remain);
    }

    if(num_write == sector_remain)
    {
      break;
    }
    else
    {
      sector_position++;
      sector_offset = 0;
      p_buffer += sector_remain;
      write_addr += (sector_remain * 2);
      num_write -= sector_remain;

      if(num_write > (SECTOR_SIZE / 2))
      {
        sector_remain = SECTOR_SIZE / 2;
      }
      else
      {
        sector_remain = num_write;
      }
    }
  }

  flash_lock();
}

/**
  * @brief  Packet tansmit data frame
  * @param  pFrame : tansmit data frame
  * @param  code : feedback Code (ACK or NACK)
  * @param  wData : payload data
  * @param  len : payload data length
  * @retval none
  */
void TCP_SendFrame( TCP_Frame_t * pFrame, uint8_t code, uint8_t* wData, uint16_t len)
{
  uint16_t len_bytes;

  len_bytes = len >> 3;

  pFrame -> Code = code + ui_receive_cmd_index;
  pFrame -> Size_LB = (uint8_t)(len_bytes & 0xFF);
  pFrame -> Size_HB = (uint8_t)(len_bytes >> 8);

  switch(len)
  {
  case TCP_LEN_NONE:
    break;

  case TCP_LEN_BYTE:
    pFrame -> Buffer[0] = (uint8_t)wData[0];
    break;

  case TCP_LEN_HALF_WORD:
    pFrame -> Buffer[0] = (uint8_t)wData[0];
    pFrame -> Buffer[1] = (uint8_t)wData[1];
    break;

  case TCP_LEN_WORD:
    pFrame -> Buffer[0] = (uint8_t)wData[0];
    pFrame -> Buffer[1] = (uint8_t)wData[1];
    pFrame -> Buffer[2] = (uint8_t)wData[2];
    pFrame -> Buffer[3] = (uint8_t)wData[3];
    break;

  case TCP_LEN_MTPA:
    pFrame -> Buffer[2] = (uint8_t)wData[0];
    pFrame -> Buffer[3] = (uint8_t)wData[1];
    pFrame -> Buffer[4] = (uint8_t)wData[2];
    pFrame -> Buffer[5] = (uint8_t)wData[3];

    pFrame -> Buffer[0] = (uint8_t)wData[4];
    pFrame -> Buffer[1] = (uint8_t)wData[5];

    pFrame -> Buffer[6] = (uint8_t)wData[6];
    pFrame -> Buffer[7] = (uint8_t)wData[7];
    pFrame -> Buffer[8] = (uint8_t)wData[8];
    pFrame -> Buffer[9] = (uint8_t)wData[9];
    break;

  default:

    /* GetBoardInfo */
    for (uint8_t i = 0; (i < 29) && (s_fwVer[i] != '\t'); i++)
    {
      pFrame -> Buffer[i] = s_fwVer[i];
    }

    break;
  }

  pFrame -> FrameCRC = TCP_CalcCRC(pFrame);
  save_extra_buffer(pFrame);
}

/**
  * @brief  Calcultion CRC of receive data
  * @param  pFrame : receive data frame
  * @retval none
  */
uint8_t RCP_CalcCRC(RCP_Frame_t * pFrame)
{
  uint8_t nCRC = 0;
  uint16_t nSum = 0;

  if( pFrame == NULL )
  {
  }
  else
  {
    nSum += pFrame->Size;
    nSum += pFrame->Code;

    for (uint8_t idx = 0; idx < (pFrame->Size - 1); idx++ )
    {
      nSum += pFrame->Buffer[idx];
    }

    nCRC = (uint8_t)(nSum & 0xFF) ;
    nCRC += (uint8_t) (nSum >> 8) ;
  }

  return nCRC ;
}

/**
  * @brief  Calcultion CRC of transmit data
  * @param  pFrame : tansmit data frame
  * @retval none
  */
uint8_t TCP_CalcCRC(TCP_Frame_t * pFrame)
{
  uint8_t nCRC = 0, idx = 0;
  uint16_t nSum = 0, buffer_size = 0;

  if( pFrame == NULL )
  {
  }
  else
  {
    buffer_size = pFrame->Size_LB + (pFrame->Size_HB << 8);

    nSum += pFrame->Size_LB;
    nSum += pFrame->Size_HB;
    nSum += pFrame->Code;

    for ( idx = 0; idx < buffer_size; idx++ )
    {
      nSum += pFrame -> Buffer[idx];
    }

    nCRC = (uint8_t)(nSum & 0xFF) ;
    nCRC += (uint8_t) (nSum >> 8) ;
  }

  return nCRC;
}

/**
  * @brief  Send full data to user
  * @param  none
  * @retval none
  */
void tx_send_monitor_data()
{
  int16_t index = 0;

  /* disable DMA UART TX*/
  dma_channel_enable(DMA_UART_TX_CHANNEL, FALSE);

  /* set transmit table */
  DMA_UART_TX_CHANNEL->maddr = (uint32_t) &monitor_data_buffer[monitor_data_buffer_num][0];

  /* put extra data response */
  if(cmd_response_rdy != RESET)
  {
    while(index < DATA_BUFFER_EXTRA_SIZE)
    {
      monitor_data_buffer[monitor_data_buffer_num][DATA_BUFFER_SYNC_SIZE + DATA_BUFFER_SIZE + index] = extra_data_buffer[index];
      index++;
    }
  }

  /* set transmission data number */
  dma_data_number_set(DMA_UART_TX_CHANNEL, DATA_BUFFER_FRAME_SIZE);
  /* enable DMA UART TX*/
  dma_channel_enable(DMA_UART_TX_CHANNEL, TRUE);
}

/**
  * @brief  Save monitor data(for dual-channel waveform drawing)
  * @param  none
  * @retval none
  */
void ui_save_monitor_data(void)
{
  monitor_data_buffer[monitor_data_buffer_num][buffer_index++] = (uint8_t)(*monitor_data[0] & 0xFF);
  monitor_data_buffer[monitor_data_buffer_num][buffer_index++] = (uint8_t)(*monitor_data[0] >> 8);
  monitor_data_buffer[monitor_data_buffer_num][buffer_index++] = (uint8_t)(*monitor_data[1] & 0xFF);
  monitor_data_buffer[monitor_data_buffer_num][buffer_index++] = (uint8_t)(*monitor_data[1] >> 8);

  if(buffer_index > (DATA_BUFFER_SYNC_SIZE + DATA_BUFFER_SIZE - 2))
  {
    tx_send_monitor_data();
    monitor_data_buffer_num ^= 1;
    buffer_index = 10;
  }
}


