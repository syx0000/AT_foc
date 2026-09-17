/**
  **************************************************************************
  * @file     mc_curr_decoupling.h
  * @brief    Declaration of field weakening related functions.
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

#ifndef __MC_CURR_DECOUPLING_H
#define __MC_CURR_DECOUPLING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mc_lib.h"

void curr_decoupling_ctrl(curr_decoupling_type *decouple_handler, qd_type Iqdref);
int16_t voltage_comp(void);
void foc_vq_limitation_decoupling(voltage_type *volt_handler, pid_ctrl_type *pid_handler, curr_decoupling_type *decouple_handler);

#ifdef __cplusplus
}
#endif

#endif
