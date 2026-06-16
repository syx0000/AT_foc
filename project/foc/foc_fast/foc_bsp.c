/**
 * @file    foc_bsp.c (AT32 minimal port)
 * @brief   FOC board support package - minimal stub for AT32F456
 *
 * Original: cubemx_yxsui/foc/foc_fast/foc_bsp.c (1604 lines, STM32H7+HPMicro)
 *
 * Migration approach:
 * - HPMicro hardware init functions: empty stubs (AT32 uses WorkBench)
 * - pwm_ccr_set: moved to wk_tmr.c
 * - g_theta_offset_*, NPP, g_vdc_raw: moved to encoder_calc.c / foc_data.c / wk_adc.c
 * - dbg_cmd_set / dbg_log_print: not migrated (HAL UART dependency, 1400 lines)
 *
 * AT32 hardware initialization is handled by WorkBench-generated code:
 *   wk_adc.c, wk_tmr.c, wk_dma.c, wk_usart.c, wk_can.c, wk_gpio.c
 */

#include "foc_bsp.h"
#include "foc_controller.h"
#include "foc_api.h"
#include "foc_data.h"
#include "ifly_fault.h"
#include "ifly_test.h"
#include "flash_port.h"
#include "wk_adc.h"
#include "wk_tmr.h"
#include "wk_usart.h"
#include "wk_can.h"
#include "can_wly.h"
#include "can_debug.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* Debug buffer defined in wk_usart.c, extern'd via foc_bsp.h */

/* ============================================================
 * Debug log control flags (set by dbg_cmd_set 'logid'/'logtest'/'logfreq')
 * Migrated from cubemx_yxsui/foc/foc_fast/foc_bsp.c:39-41
 * ============================================================ */
volatile uint16_t dbgLogFlag   = 0;   /* log id: 10/11/30/40/50/100/110/120/150/151/165 */
volatile uint16_t testLogFlag  = 0;   /* test mode flag */
volatile uint16_t logPriodMs   = 100; /* log print period in ms (default 100ms) */

/* ISR timing instrumentation (DWT cycle stamps @192MHz) */
volatile uint32_t g_adc_isr_cycles = 0;
volatile uint32_t g_adc_isr_cycles_max = 0;
volatile uint32_t g_adc_isr_in_cycles = 0;   /* ADC ISR entry DWT */
volatile uint32_t g_adc_isr_out_cycles = 0;  /* ADC ISR exit DWT */
volatile uint32_t g_tim1_cc4_cycles = 0;     /* TMR1 CC4 ISR entry DWT */
volatile uint32_t g_tim1_cc4_exit_cycles = 0;/* TMR1 CC4 ISR exit DWT */
volatile uint32_t g_tim1_enc_done_cycles = 0;/* DPT encoder IDLE done DWT */

/* ============================================================
 * HPMicro platform stubs (kept empty for AT32 - hardware init in wk_*.c)
 * ============================================================ */
void seiInterruptReset(void)         { /* AT32: not needed */ }
void led_init(void)                  { /* AT32: GPIO init in wk_gpio.c */ }
void break_motor_operation_init(void){ /* AT32: TMR1 BRK in wk_tmr.c */ }
void sto_motor_operation_init(void)  { /* AT32: not needed */ }
void isr_gpio(void)                  { /* AT32: stub */ }
void pwm_pins_init(void)             { /* AT32: TMR1 GPIO in wk_tmr.c */ }
void isr_pwm0_counter(void)          { /* AT32: stub */ }
void adc_pins_init(void)             { /* AT32: ADC GPIO in wk_adc.c */ }
void adc_isr_enable(void)            { /* AT32: enabled in wk_adc.c */ }
void isr_adc(void)                   { /* AT32: handler in at32f45x_int.c */ }

void pwmv2_duty_init(PWMV2_Type *ptr, uint32_t PWM_PRD, uint8_t CMP_SHADOW_REGISTER_UPDATE_TYPE, uint8_t CMP_PWM_REGISTER_UPDATE_TYPE, uint8_t CMP_SOURCE)
{
    (void)ptr; (void)PWM_PRD; (void)CMP_SHADOW_REGISTER_UPDATE_TYPE; (void)CMP_PWM_REGISTER_UPDATE_TYPE; (void)CMP_SOURCE;
}

void bldc_foc_pwmset(BLDC_CONTROL_PWMOUT_PARA *par) { (void)par; }

void adc_init_udc_temp(ADC16_Type *ptr, uint8_t udc_channel, uint8_t temp_channel, uint32_t sample_cycle)
{ (void)ptr; (void)udc_channel; (void)temp_channel; (void)sample_cycle; }

void adc_cfg_init(ADC16_Type *ptr, uint8_t channel, uint32_t sample_cycle, uint32_t ADC_MODULE, uint32_t ADC_TRG)
{ (void)ptr; (void)channel; (void)sample_cycle; (void)ADC_MODULE; (void)ADC_TRG; }

void init_trigger_mux(TRGM_Type *ptr, uint8_t TRAG_INPUT, uint8_t TRAG_INPUT_FOR_ADC)
{ (void)ptr; (void)TRAG_INPUT; (void)TRAG_INPUT_FOR_ADC; }

void init_trigger_cfg(ADC16_Type *ptr, uint8_t trig_ch, uint8_t channel, bool inten, uint32_t ADC_MODULE, uint8_t ADC_PREEMPT_TRIG_LEN)
{ (void)ptr; (void)trig_ch; (void)channel; (void)inten; (void)ADC_MODULE; (void)ADC_PREEMPT_TRIG_LEN; }

void adc_module_cfg(adc_type *adc_typ, uint8_t adc_module, ADC16_Type *adc_base_ptr)
{ (void)adc_typ; (void)adc_module; (void)adc_base_ptr; }

void pwmv2_trigfor_adc_init(PWMV2_Type *ptr, uint32_t PWM_PRD, uint32_t PWM_CNT, uint8_t CMP_SHADOW_REGISTER_UPDATE_TYPE, uint8_t CMP_PWM_REGISTER_UPDATE_TYPE, uint8_t PWM_TRIGOUT_CH_ADC, uint8_t CMP_SOURCE, uint8_t PWM_CH_TRIG_ADC)
{ (void)ptr; (void)PWM_PRD; (void)PWM_CNT; (void)CMP_SHADOW_REGISTER_UPDATE_TYPE; (void)CMP_PWM_REGISTER_UPDATE_TYPE; (void)PWM_TRIGOUT_CH_ADC; (void)CMP_SOURCE; (void)PWM_CH_TRIG_ADC; }

void pwmv2_trigfor_sei_init(PWMV2_Type *ptr, uint32_t PWM_PRD, uint32_t PWM_CNT, uint8_t CMP_SHADOW_REGISTER_UPDATE_TYPE, uint8_t CMP_PWM_REGISTER_UPDATE_TYPE, uint8_t PWM_TRIGOUT_CH_SEI, uint8_t CMP_SOURCE, uint8_t PWM_CH_TRIG_SEI)
{ (void)ptr; (void)PWM_PRD; (void)PWM_CNT; (void)CMP_SHADOW_REGISTER_UPDATE_TYPE; (void)CMP_PWM_REGISTER_UPDATE_TYPE; (void)PWM_TRIGOUT_CH_SEI; (void)CMP_SOURCE; (void)PWM_CH_TRIG_SEI; }

uint32_t motor_encoder_spi(uint8_t in_out) { (void)in_out; return 0; }

/**
 * @brief  Get system clock in milliseconds
 * @retval ms count since boot (matches HAL_GetTick semantics)
 */
uint64_t get_clock_cpu_ms(void)
{
    extern volatile uint32_t systick_ms;
    return systick_ms;
}

/**
 * @brief  Get version ID (placeholder, returns AT32 platform code)
 */
uint8_t get_ver_id(void)
{
    return 0xA3;  /* 'A3' = AT32 */
}

/* ============================================================
 * Debug command parser / log printer
 * ============================================================
 * Original: dbg_cmd_set() (950 lines) parsed serial commands
 *           dbg_log_print() (495 lines) periodic status print
 * Both depend heavily on STM32 HAL UART API and FDCAN/can_debug/can_wly.
 * Skipped in initial port - re-enable when porting CAN protocol layer.
 */
void dbg_cmd_set(void)
{
    char *loc;
    char *token;

    if (usart_rx_len == 0) return;

    /* version */
    if (NULL != strstr((const char *)dbgRecvBuf, "version")) {
        printf("AT32F456 FOC v1.0 built %s %s\r\n", __DATE__, __TIME__);
    }

    /* reset */
    if (NULL != strstr((const char *)dbgRecvBuf, "reset")) {
        printf("System reset...\r\n");
        wk_delay_ms(20);
        fault_safe_shutdown();
        wk_delay_ms(50);
        NVIC_SystemReset();
    }

    /* CurrentPID Kp<x>Ki<x>Kd<x> */
    if (NULL != strstr((char *)dbgRecvBuf, "CurrentPID")) {
        printf("CurrentPID1:%d, %d, %d\r\n",
               controller_eyou.IncPID_QAxis.P,
               controller_eyou.IncPID_QAxis.I,
               controller_eyou.IncPID_QAxis.D);
        loc = strstr((char *)dbgRecvBuf, "Kp");
        if (loc) {
            int32_t kp = atoi(loc + 2);
            loc = strstr((char *)dbgRecvBuf, "Ki");
            int32_t ki = loc ? atoi(loc + 2) : 0;
            loc = strstr((char *)dbgRecvBuf, "Kd");
            int32_t kd = loc ? atoi(loc + 2) : 0;
            controller_eyou.IncPID_QAxis.P = kp;
            controller_eyou.IncPID_QAxis.I = ki;
            controller_eyou.IncPID_QAxis.D = kd;
            controller_eyou.IncPID_DAxis.P = kp;
            controller_eyou.IncPID_DAxis.I = ki;
            controller_eyou.IncPID_DAxis.D = kd;
            controller_eyou.FlashData.Current_Kp = kp;
            controller_eyou.FlashData.Current_Ki = ki;
            controller_eyou.FlashData.Current_Kd = kd;
            printf("CurrentPID2:%d, %d, %d\r\n",
                   controller_eyou.IncPID_QAxis.P,
                   controller_eyou.IncPID_QAxis.I,
                   controller_eyou.IncPID_QAxis.D);
        }
    }

    /* SpeedPID Kp<x>Ki<x>Kd<x> */
    if (NULL != strstr((char *)dbgRecvBuf, "SpeedPID")) {
        printf("SpeedPID:%d, %d, %d\r\n",
               controller_eyou.IncPID_Speed.P,
               controller_eyou.IncPID_Speed.I,
               controller_eyou.IncPID_Speed.D);
        loc = strstr((char *)dbgRecvBuf, "Kp");
        if (loc) {
            int32_t kp = atoi(loc + 2);
            loc = strstr((char *)dbgRecvBuf, "Ki");
            int32_t ki = loc ? atoi(loc + 2) : 0;
            loc = strstr((char *)dbgRecvBuf, "Kd");
            int32_t kd = loc ? atoi(loc + 2) : 0;
            controller_eyou.IncPID_Speed.P = kp;
            controller_eyou.IncPID_Speed.I = ki;
            controller_eyou.IncPID_Speed.D = kd;
            controller_eyou.FlashData.Speed_Kp = kp;
            controller_eyou.FlashData.Speed_Ki = ki;
            controller_eyou.FlashData.Speed_Kd = kd;
            printf("SpeedPID:%d, %d, %d\r\n",
                   controller_eyou.IncPID_Speed.P,
                   controller_eyou.IncPID_Speed.I,
                   controller_eyou.IncPID_Speed.D);
        }
    }

    /* PositionPID Kp<x>Ki<x>Kd<x> */
    if (NULL != strstr((char *)dbgRecvBuf, "PositionPID")) {
        printf("PositionPID:%d, %d, %d\r\n",
               controller_eyou.IncPID_Position.P,
               controller_eyou.IncPID_Position.I,
               controller_eyou.IncPID_Position.D);
        loc = strstr((char *)dbgRecvBuf, "Kp");
        if (loc) {
            int32_t kp = atoi(loc + 2);
            loc = strstr((char *)dbgRecvBuf, "Ki");
            int32_t ki = loc ? atoi(loc + 2) : 0;
            loc = strstr((char *)dbgRecvBuf, "Kd");
            int32_t kd = loc ? atoi(loc + 2) : 0;
            controller_eyou.IncPID_Position.P = kp;
            controller_eyou.IncPID_Position.I = ki;
            controller_eyou.IncPID_Position.D = kd;
            controller_eyou.FlashData.Position_Kp = kp;
            controller_eyou.FlashData.Position_Ki = ki;
            controller_eyou.FlashData.Position_Kd = kd;
            printf("PositionPID:%d, %d, %d\r\n",
                   controller_eyou.IncPID_Position.P,
                   controller_eyou.IncPID_Position.I,
                   controller_eyou.IncPID_Position.D);
        }
    }

    /* Run cmd<0/1> M<mode> tar<value>
     * mode: 2=torque, 3=velocity, 4=position
     * tar: torque(A), velocity(rpm), position(deg) */
    if (NULL != strstr((char *)dbgRecvBuf, "Run")) {
        loc = strstr((char *)dbgRecvBuf, "cmd");
        int cmd_val = loc ? atoi(loc + 3) : 0;
        loc = strstr((char *)dbgRecvBuf, "M");
        int mode_val = loc ? atoi(loc + 1) : 2;
        char *tar_str = strstr((char *)dbgRecvBuf, "tar");
        float tar_value = tar_str ? atof(tar_str + 3) : 0.0f;

        if (cmd_val == 0) {
            fault_safe_shutdown();
            printf("Runcmd0: safe shutdown initiated\r\n");
            memset((uint8_t *)dbgRecvBuf, 0, usart_rx_len);
            usart_rx_len = 0;
            return;
        }

        if (fault_brake_is_active()) {
            printf("enable: brake in progress, ignored\r\n");
        } else if (controller_eyou.ServoErrFlag.All_Flag != 0) {
            printf("enable: fault active (0x%08lX), clear first\r\n",
                   (unsigned long)controller_eyou.ServoErrFlag.All_Flag);
        } else {
            uint8_t already_running = (controller_eyou.foc_run >= 1 &&
                                       controller_eyou.controller_mode == mode_val);
            if (!already_running) {
                ResetControlData(&controller_eyou);
                controller_eyou.foc_run = cmd_val;
                controller_eyou.controller_mode = mode_val;
                TMR1->brk |= TIM_BDTR_MOE;
                TMR1->cctrl |= 0x0555u;
            }

            if (mode_val == PROFILE_TORQUE_MODE) {
                float tq_nm = tar_value;
                int32_t iq = (int32_t)(can_wly_Nm_to_iA(tq_nm) * 1024.0f);
                int32_t max_cur = (int32_t)controller_eyou.FlashData.MaxCurrent;
                if (iq > max_cur) {
                    iq = max_cur;
                    printf("  iq cmd clamped to +%d\r\n", (int)max_cur);
                }
                if (iq < -max_cur) {
                    iq = -max_cur;
                    printf("  iq cmd clamped to -%d\r\n", (int)max_cur);
                }
                controller_eyou.I_q_ref = iq;
                printf("run mod_Target: %d, %.3f Nm\r\n", controller_eyou.controller_mode, tq_nm);
            } else if (mode_val == PROFILE_VELOCITY_MOCE) {
                float vel_rpm = tar_value;
                controller_eyou.velocity_ref = (int32_t)(vel_rpm * 1024.0f * FOC_GEAR_RATIO);
                printf("run mod_Target: %d, %.2f rpm\r\n", controller_eyou.controller_mode, vel_rpm);
            } else if (mode_val == PROFILE_POSITION_MODE) {
                float pos_deg = tar_value;
                int32_t pos = (int32_t)(pos_deg * 1024.0f);
                if (controller_eyou.FlashData.PositionLimitFlag == 50) {
                    if (pos > controller_eyou.FlashData.MaxPositionLimit) {
                        pos = controller_eyou.FlashData.MaxPositionLimit;
                        printf("  pos cmd clamped to MaxPos=%.2f deg\r\n", pos / 1024.0f);
                    }
                    if (pos < controller_eyou.FlashData.MinPositionLimit) {
                        pos = controller_eyou.FlashData.MinPositionLimit;
                        printf("  pos cmd clamped to MinPos=%.2f deg\r\n", pos / 1024.0f);
                    }
                }
                controller_eyou.position_ref = pos;
                printf("run mod_Target: %d, %.2f deg\r\n", controller_eyou.controller_mode, pos_deg);
            } else {
                printf("run mod_Target: %d, %.2f\r\n", controller_eyou.controller_mode, tar_value);
            }
        }
    }

    /* enable<0/1>: PWM output enable/disable */
    if (NULL != strstr((char *)dbgRecvBuf, "enable")) {
        loc = strstr((char *)dbgRecvBuf, "enable");
        int en = atoi(loc + 6);
        if (en) {
            if (fault_brake_is_active()) {
                printf("enable: brake in progress, ignored\r\n");
            } else if (controller_eyou.ServoErrFlag.All_Flag != 0) {
                printf("enable: fault active (0x%08lX), clear first\r\n",
                       (unsigned long)controller_eyou.ServoErrFlag.All_Flag);
            } else {
                ResetControlData(&controller_eyou);
                controller_eyou.I_q_ref = 0;
                controller_eyou.velocity_ref = 0;
                controller_eyou.position_ref = controller_eyou.real_position_out;
                controller_eyou.controller_mode = PROFILE_TORQUE_MODE;
                controller_eyou.foc_run = 2;
                TMR1->brk |= TIM_BDTR_MOE;
                TMR1->cctrl |= 0x0555u;
                printf("PWM enabled, mode=Torque, I_q_ref=0 (CCER=0x%04X)\r\n",
                       (unsigned int)(TMR1->cctrl & 0xFFFF));
            }
        } else {
            fault_safe_shutdown();
            printf("PWM disable requested, safe shutdown initiated\r\n");
        }
    }

    /* openloop<0/1/2>: 0=off, 1=auto-rotate, 2=encoder-follow */
    if (NULL != strstr((char *)dbgRecvBuf, "openloop")) {
        loc = strstr((char *)dbgRecvBuf, "openloop");
        int mode = atoi(loc + 8);
        extern volatile uint8_t g_foc_openloop_enable;
        extern uint8_t open_loop_mode;
        if (mode == 0) {
            g_foc_openloop_enable = 0;
            controller_eyou.foc_run = 0;
            printf("openloop OFF\r\n");
        } else {
            controller_eyou.foc_run = 0;
            g_foc_openloop_enable = 1;
            open_loop_mode = (mode == 2) ? 1 : 0;
            TMR1->brk |= TIM_BDTR_MOE;
            TMR1->cctrl |= 0x0555u;
            printf("openloop ON, mode=%d\r\n", open_loop_mode);
        }
    }

    /* vd<value> vq<value>: open-loop voltage (Q10) */
    if (NULL != strstr((char *)dbgRecvBuf, "vd")) {
        loc = strstr((char *)dbgRecvBuf, "vd");
        extern int16_t v_d_test, v_q_test;
        v_d_test = (int16_t)atoi(loc + 2);
        printf("v_d_test=%d\r\n", v_d_test);
    }
    if (NULL != strstr((char *)dbgRecvBuf, "vq")) {
        loc = strstr((char *)dbgRecvBuf, "vq");
        extern int16_t v_d_test, v_q_test;
        v_q_test = (int16_t)atoi(loc + 2);
        printf("v_q_test=%d\r\n", v_q_test);
    }

    /* logid<N>: set debug log id */
    if (NULL != (loc = strstr((char *)dbgRecvBuf, "logid"))) {
        uint16_t id = (uint16_t)atoi(loc + 5);
        dbgLogFlag = id;
        printf("logid:%d\r\n", id);
    }

    /* logfreq<ms>: set debug log period */
    if (NULL != (loc = strstr((char *)dbgRecvBuf, "logfreq"))) {
        uint16_t ms = (uint16_t)atoi(loc + 7);
        if (ms < 10) ms = 10;
        logPriodMs = ms;
        printf("logfreq:%d\r\n", ms);
    }

    /* Cali: electrical angle calibration + Flash write */
    if (NULL != strstr((char *)dbgRecvBuf, "Cali")) {
        uint8_t old_run = controller_eyou.foc_run;
        controller_eyou.foc_run = 0;
        controller_eyou.ServoErrFlag.All_Flag = 0;
        wk_delay_ms(10);

        /* Re-enable PWM output (may be off after fault) */
        TMR1->cctrl |= 0x0555u;
        TMR1->brk |= TIM_BDTR_MOE;

        ElecAngleEstimate(&controller_eyou);

        if (Flash_EraseSector() != HAL_OK) {
            printf("Cali: Flash erase FAIL\r\n");
        } else {
            WriteDataToFlash();
            printf("Cali done, offset=%u\r\n", controller_eyou.FlashData.elec_offset);
        }

        controller_eyou.foc_run = old_run;
    }

    /* logtest<N> */
    if (NULL != strstr((const char *)dbgRecvBuf, "logtest")) {
        loc = strstr((char *)dbgRecvBuf, "logtest");
        token = strtok(loc, "logtest");
        testLogFlag = atoi((char *)token);
        printf("logtest:%d\r\n", testLogFlag);
    }

    /* bwtest<1-11>: bandwidth test */
    if (NULL != strstr((const char *)dbgRecvBuf, "bwtest")) {
        loc = strstr((char *)dbgRecvBuf, "bwtest");
        token = strtok(loc, "bwtest");
        int which = atoi((char *)token);
        printf("bwtest:%d\r\n", which);
        extern void TestAutoPhaseComp(void);
        if (which == 1) TestCurrentLoopBandwidth();
        else if (which == 2) TestSpeedLoopBandwidth();
        else if (which == 3) TestMotorParamsIdent();
        else if (which == 4) TestFluxIdent();
        else if (which == 5) TestInertiaIdent();
        else if (which == 6) TestAutoTuneCurrent();
        else if (which == 7) TestAutoTuneSpeed();
        else if (which == 8) TestAutoTunePosition();
        else if (which == 9) TestPositionLoopBandwidth();
        else if (which == 10) TestDeadtimeCalibration();
        else if (which == 11) TestAutoPhaseComp();
    }

    /* injectV<mV>: d-axis voltage inject test 5s */
    if (NULL != strstr((char *)dbgRecvBuf, "injectV")) {
        loc = strstr((char *)dbgRecvBuf, "injectV");
        token = strtok(loc, "injectV");
        int32_t mv = atoi((char *)token);
        float v_d = mv / 1000.0f;
        printf("inject test: V_d=%.3fV, theta=0, duration=5s\r\n", v_d);
        uint8_t old_run = controller_eyou.foc_run;
        controller_eyou.foc_run = 1;
        controller_eyou.ident_test.enable = 1;
        controller_eyou.ident_test.amplitude = 0;
        controller_eyou.ident_test.settle_samples = 0;
        controller_eyou.ident_test.measure_samples = 0xFFFFFFFF;
        controller_eyou.ident_test.sample_count = 0;
        controller_eyou.V_d = (int32_t)(v_d * 1024);
        controller_eyou.V_q = 0;
        controller_eyou.theta_elec = 0;
        for (int i = 0; i < 50; i++) {
            wk_delay_ms(100);
            float i_a_amp = controller_eyou.I_a / 1024.0f;
            float i_d_amp = controller_eyou.I_d / 1024.0f;
            float r_a = (fabsf(i_a_amp)>0.01f)?(v_d/fabsf(i_a_amp)):0.0f;
            float r_d = (fabsf(i_d_amp)>0.01f)?(v_d/fabsf(i_d_amp)):0.0f;
            printf("[%2d] Ia=%6.3fA Id=%6.3fA R(Ia)=%.4f R(Id)=%.4f CCR=%lu/%lu/%lu\r\n",
                   i, i_a_amp, i_d_amp, r_a, r_d,
                   (unsigned long)TMR1->c1dt, (unsigned long)TMR1->c2dt, (unsigned long)TMR1->c3dt);
        }
        controller_eyou.ident_test.enable = 0;
        controller_eyou.V_d = 0; controller_eyou.V_q = 0;
        set_phase_voltage(&controller_eyou, 0, 0, 0);
        controller_eyou.foc_run = old_run;
        printf("inject test done\r\n");
    }

    /* canrxdbg<0/1> */
    if (NULL != strstr((char *)dbgRecvBuf, "canrxdbg")) {
        loc = strstr((char *)dbgRecvBuf, "canrxdbg");
        token = strtok(loc, "canrxdbg");
        g_can_rx_debug = (uint8_t)atoi((char *)token);
        printf("CAN RX debug: %s\r\n", g_can_rx_debug ? "ON" : "OFF");
    }

    /* OverloadA<x>W<y>S<z> */
    if (NULL != strstr((char *)dbgRecvBuf, "Overload")) {
        loc = strstr((char *)dbgRecvBuf, "A");
        if (loc != NULL && loc > strstr((char *)dbgRecvBuf, "Overload")) {
            uint16_t a_val = (uint16_t)atoi(loc + 1);
            char *wloc = strstr(loc, "W");
            char *sloc = wloc ? strstr(wloc, "S") : NULL;
            if (wloc && sloc && a_val > 0 && a_val <= 255) {
                uint16_t w_val = (uint16_t)atoi(wloc + 1);
                uint16_t s_val = (uint16_t)atoi(sloc + 1);
                if (w_val > 0 && w_val <= 60 && s_val > w_val && s_val <= 60) {
                    g_overload_current_A = a_val;
                    g_overload_warn_s    = w_val;
                    g_overload_stop_s    = s_val;
                }
            }
        }
        printf("Overload: %uA, warn=%us, stop=%us\r\n",
               g_overload_current_A, g_overload_warn_s, g_overload_stop_s);
    }

    /* Phase angle compensation commands */
    {
        extern int16_t g_theta_offset_pos, g_theta_offset_neg;
        extern int16_t g_theta_comp_pos, g_theta_comp_neg;
        if (NULL != (loc = strstr((char *)dbgRecvBuf, "offsetpos"))) {
            g_theta_offset_pos = (int16_t)atoi(loc + strlen("offsetpos"));
            printf("offset_pos=%d (x0.1deg)\r\n", g_theta_offset_pos);
        }
        if (NULL != (loc = strstr((char *)dbgRecvBuf, "offsetneg"))) {
            g_theta_offset_neg = (int16_t)atoi(loc + strlen("offsetneg"));
            printf("offset_neg=%d (x0.1deg)\r\n", g_theta_offset_neg);
        }
        if (NULL != (loc = strstr((char *)dbgRecvBuf, "comppos"))) {
            g_theta_comp_pos = (int16_t)atoi(loc + strlen("comppos"));
            printf("comp_pos=%d (x0.1)\r\n", g_theta_comp_pos);
        }
        if (NULL != (loc = strstr((char *)dbgRecvBuf, "compneg"))) {
            g_theta_comp_neg = (int16_t)atoi(loc + strlen("compneg"));
            printf("comp_neg=%d (x0.1)\r\n", g_theta_comp_neg);
        }
    }

    /* savephasecomp */
    if (NULL != strstr((char *)dbgRecvBuf, "savephasecomp")) {
        SavePhaseCompToFlash();
        printf("Phase comp saved to Flash\r\n");
    }

    /* Single-tone injection test commands */
    {
        extern uint32_t can_wly_get_test_freq(void);
        extern void can_wly_set_test_freq(uint32_t hz);
        extern uint32_t can_wly_get_test_ampl(void);
        extern void can_wly_set_test_ampl(uint32_t q10);
        extern void can_wly_test_start(void);
        extern void can_wly_test_stop(void);
        extern uint32_t can_wly_get_test_tx_ok(void);
        extern uint32_t can_wly_get_test_tx_fail(void);

        if (NULL != strstr((char *)dbgRecvBuf, "testfreq")) {
            loc = strstr((char *)dbgRecvBuf, "testfreq");
            token = strtok(loc, "testfreq");
            can_wly_set_test_freq((uint32_t)atoi((char *)token));
            printf("test_freq=%u Hz\r\n", (unsigned)can_wly_get_test_freq());
        }
        if (NULL != strstr((char *)dbgRecvBuf, "testampl")) {
            loc = strstr((char *)dbgRecvBuf, "testampl");
            token = strtok(loc, "testampl");
            can_wly_set_test_ampl((uint32_t)atoi((char *)token));
            printf("test_ampl=%u Q10 (%.2fA)\r\n",
                   (unsigned)can_wly_get_test_ampl(), can_wly_get_test_ampl()/1024.0f);
        }
        if (NULL != strstr((char *)dbgRecvBuf, "teststart")) {
            can_wly_test_start();
            printf("test started: %uHz, %.2fA\r\n",
                   (unsigned)can_wly_get_test_freq(), can_wly_get_test_ampl()/1024.0f);
        }
        if (NULL != strstr((char *)dbgRecvBuf, "teststop")) {
            uint32_t ok = can_wly_get_test_tx_ok();
            uint32_t fail = can_wly_get_test_tx_fail();
            can_wly_test_stop();
            printf("test stopped: tx_ok=%u, tx_fail=%u\r\n", (unsigned)ok, (unsigned)fail);
        }
    }

    /* canstat: AT32 CAN status */
    if (NULL != strstr((char *)dbgRecvBuf, "canstat")) {
        uint32_t tec = (CAN1->sts >> 16) & 0xFF;
        uint32_t rec = (CAN1->sts >> 24) & 0xFF;
        uint32_t busoff = (CAN1->sts & (1<<2)) ? 1 : 0;
        uint32_t errp = (CAN1->sts & (1<<1)) ? 1 : 0;
        printf("CAN1: TxErr=%lu RxErr=%lu BusOff=%lu ErrPassive=%lu\r\n",
               (unsigned long)tec, (unsigned long)rec,
               (unsigned long)busoff, (unsigned long)errp);
        printf("  TxFailCnt=%lu NodeID=%u\r\n",
               (unsigned long)can_wly_get_tx_fail_count(), can_wly_get_node_id());
    }

    /* PLACEHOLDER_CANTEST_MIT */
    /* cantest<N>: CAN protocol self-test (1-19) */
    if (NULL != strstr((char *)dbgRecvBuf, "cantest")) {
        extern volatile uint8_t g_cantest_stub;
        loc = strstr((char *)dbgRecvBuf, "cantest");
        token = strtok(loc, "cantest");
        int tc = atoi((char *)token);
        printf("=== cantest%d ===\r\n", tc);
        g_cantest_stub = 1;
        switch (tc) {
        case 1: { uint8_t d[]={0x6D,0x8D,0x01}; fdcan_rx_user(0x200,d,3); break; }
        case 2: { uint8_t d[]={0x00,0x00,0x80,0x00,0x80,0x01}; fdcan_rx_user(0x400,d,6); break; }
        case 3: { fdcan_rx_user(0x080,NULL,0); break; }
        case 4: { uint8_t d[]={0x40,0x00,0x20,0x00,0,0,0,0}; fdcan_rx_user(0x600+can_wly_get_node_id(),d,8); break; }
        case 5: { union{float f;uint8_t b[4];}cv; cv.f=-10.0f;
                  uint8_t d[]={0x23,0x00,0x20,0x00,cv.b[0],cv.b[1],cv.b[2],cv.b[3]};
                  fdcan_rx_user(0x600+can_wly_get_node_id(),d,8); break; }
        case 6: { uint8_t d[]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFA};
                  fdcan_rx_user(0x700+can_wly_get_node_id(),d,8); break; }
        case 7: { uint8_t d[]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
                  controller_eyou.ServoErrFlag.All_Flag=0;
                  fdcan_rx_user(0x700+can_wly_get_node_id(),d,8);
                  g_cantest_stub=0;
                  for(int i=0;i<250;i++){wk_delay_ms(1);can_wly_tick_1ms();}
                  g_cantest_stub=1;
                  printf("  CommErr=%d\r\n",(int)controller_eyou.ServoErrFlag.Bit.CommunicateErr);
                  controller_eyou.ServoErrFlag.All_Flag=0; break; }
        case 8: { uint8_t d[]={0x00,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x40,0x00,0x40,0x01};
                  fdcan_rx_user(0x500,d,12);
                  printf("  mit_kp=%.2f kd=%.2f\r\n",controller_eyou.mit_kp,controller_eyou.mit_kd); break; }
        case 9: { uint8_t d[]={0x00,0x00,0x80,0x00,0x80,0x00,0x80,0xFF,0xFF,0x00,0x00,0x01};
                  fdcan_rx_user(0x500,d,12);
                  printf("  kp=%.2f(exp500) kd=%.2f(exp0)\r\n",controller_eyou.mit_kp,controller_eyou.mit_kd); break; }
        case 10: { uint8_t d[]={0x00,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x00,0xFF,0xFF,0x01};
                   fdcan_rx_user(0x500,d,12);
                   printf("  kp=%.2f(exp0) kd=%.2f(exp20)\r\n",controller_eyou.mit_kp,controller_eyou.mit_kd); break; }
        case 11: { uint8_t d[]={0x00,0x00,0xC0,0x00,0xC0,0x00,0x80,0x99,0x19,0xA3,0x30,0x01};
                   fdcan_rx_user(0x500,d,12);
                   printf("  p=%.4f(exp3.5) v=%.4f(exp10)\r\n",controller_eyou.mit_p_des,controller_eyou.mit_v_des); break; }
        case 12: { uint8_t d[]={0x00,0x00,0x40,0x00,0x40,0x00,0x80,0x99,0x19,0xA3,0x30,0x01};
                   fdcan_rx_user(0x500,d,12);
                   printf("  p=%.4f(exp-3.5) v=%.4f(exp-10)\r\n",controller_eyou.mit_p_des,controller_eyou.mit_v_des); break; }
        case 13: { uint8_t d[]={0x00,0x00,0x80,0x00,0x80,0xFF,0xFF,0x00,0x00,0x00,0x00,0x01};
                   fdcan_rx_user(0x500,d,12);
                   printf("  t_ff=%.3fA(exp~168)\r\n",controller_eyou.mit_t_ff); break; }
        case 14: { uint8_t d[]={0x00,0x00,0x80,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x01};
                   fdcan_rx_user(0x500,d,12);
                   printf("  t_ff=%.3fA(exp~-168)\r\n",controller_eyou.mit_t_ff); break; }
        case 15: { uint8_t d[]={0xFF,0xFF,0xFF,0x00,0x80,0x00,0x80,0x00,0x40,0x00,0x80,0x01};
                   fdcan_rx_user(0x500,d,12);
                   printf("  p=%.4f(exp~7)\r\n",controller_eyou.mit_p_des); break; }
        case 16: { uint8_t d[]={0x00,0x00,0x00,0x00,0x80,0x00,0x80,0x00,0x40,0x00,0x80,0x01};
                   fdcan_rx_user(0x500,d,12);
                   printf("  p=%.4f(exp~-7)\r\n",controller_eyou.mit_p_des); break; }
        case 17: { uint8_t d[]={0x00,0x00,0x80,0xFF,0xFF,0x00,0x80,0x00,0x20,0xFF,0xFF,0x01};
                   fdcan_rx_user(0x500,d,12);
                   printf("  v=%.4f(exp~20)\r\n",controller_eyou.mit_v_des); break; }
        case 18: { uint8_t d[]={0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x20,0xFF,0xFF,0x01};
                   fdcan_rx_user(0x500,d,12);
                   printf("  v=%.4f(exp~-20)\r\n",controller_eyou.mit_v_des); break; }
        case 19: { uint8_t d[]={0x00,0x00,0x80,0x00,0x80,0x8F,0x82,0x33,0x33,0x00,0x40,0x01};
                   fdcan_rx_user(0x500,d,12);
                   printf("  t_ff=%.3f kp=%.2f(exp100) kd=%.2f(exp5)\r\n",
                          controller_eyou.mit_t_ff,controller_eyou.mit_kp,controller_eyou.mit_kd); break; }
        default: printf("  valid: 1-19\r\n"); break;
        }
        printf("  mode=%d foc_run=%d\r\n", controller_eyou.controller_mode, controller_eyou.foc_run);
        g_cantest_stub = 0;
        printf("=== cantest%d done ===\r\n", tc);
    }

    /* mit<N>: MIT mode test (0-7) */
    if (NULL != strstr((char *)dbgRecvBuf, "mit")) {
        extern volatile uint8_t g_cantest_stub;
        loc = strstr((char *)dbgRecvBuf, "mit");
        token = strtok(loc, "mit");
        int step = atoi((char *)token);
        printf("=== mit%d ===\r\n", step);
        g_cantest_stub = 1;
        switch (step) {
        case 0: {
            uint8_t d1[]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB};
            uint8_t d2[]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
            uint8_t d3[]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
            uint8_t d4[]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFA};
            fdcan_rx_user(0x701,d1,8); wk_delay_ms(20);
            fdcan_rx_user(0x701,d2,8); wk_delay_ms(20);
            fdcan_rx_user(0x701,d3,8); wk_delay_ms(20);
            fdcan_rx_user(0x701,d4,8);
            printf("  foc_run=%d\r\n", controller_eyou.foc_run);
            break; }
        case 1: { uint8_t d[]={0xFF,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0x00,0x00,0x00,0x00,0x01};
                  for(int i=0;i<10;i++){fdcan_rx_user(0x500,d,12);wk_delay_ms(10);}
                  printf("  mode=%d Iq=%d\r\n",controller_eyou.controller_mode,controller_eyou.I_q); break; }
        case 2: { uint8_t d[]={0xFF,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0x1E,0x05,0xCC,0x0C,0x01};
                  for(int i=0;i<10;i++){fdcan_rx_user(0x500,d,12);wk_delay_ms(10);}
                  printf("  kp=%.2f kd=%.2f\r\n",controller_eyou.mit_kp,controller_eyou.mit_kd); break; }
        case 3: { uint8_t d[]={0xB2,0x9A,0x81,0xFF,0x7F,0xFF,0x7F,0x1E,0x05,0xCC,0x0C,0x01};
                  for(int i=0;i<10;i++){fdcan_rx_user(0x500,d,12);wk_delay_ms(10);}
                  printf("  p_des=%.4f\r\n",controller_eyou.mit_p_des); break; }
        case 4: { uint8_t d[]={0x4C,0x65,0x7E,0xFF,0x7F,0xFF,0x7F,0x1E,0x05,0xCC,0x0C,0x01};
                  for(int i=0;i<10;i++){fdcan_rx_user(0x500,d,12);wk_delay_ms(10);}
                  printf("  p_des=%.4f\r\n",controller_eyou.mit_p_des); break; }
        case 5: { uint8_t d[]={0xB2,0x9A,0x81,0xFF,0x7F,0xFF,0x7F,0x99,0x19,0x99,0x19,0x01};
                  for(int i=0;i<10;i++){fdcan_rx_user(0x500,d,12);wk_delay_ms(10);}
                  printf("  kp=%.2f kd=%.2f\r\n",controller_eyou.mit_kp,controller_eyou.mit_kd); break; }
        case 6: { uint8_t d[]={0xFF,0xFF,0x7F,0x32,0x83,0xFF,0x7F,0x00,0x00,0x99,0x19,0x01};
                  for(int i=0;i<10;i++){fdcan_rx_user(0x500,d,12);wk_delay_ms(10);}
                  printf("  v_des=%.4f\r\n",controller_eyou.mit_v_des); break; }
        case 7: { uint8_t d[]={0xFF,0xFF,0x7F,0xFF,0x7F,0x19,0x80,0x00,0x00,0x00,0x00,0x01};
                  for(int i=0;i<10;i++){fdcan_rx_user(0x500,d,12);wk_delay_ms(10);}
                  printf("  t_ff=%.2f Iq=%d\r\n",controller_eyou.mit_t_ff/1024.0f,controller_eyou.I_q); break; }
        default: printf("  valid: 0-7\r\n"); break;
        }
        g_cantest_stub = 0;
        printf("=== mit%d done ===\r\n", step);
    }

    /* OTA commands (stub - OTA subsystem not yet ported) */
    if (NULL != strstr((char *)dbgRecvBuf, "otaswap")) {
        printf("Reboot...\r\n"); wk_delay_ms(50); NVIC_SystemReset();
    } else if (NULL != strstr((char *)dbgRecvBuf, "otabegin")) {
        printf("OTA_ERR not_ported\r\n");
    } else if (NULL != strstr((char *)dbgRecvBuf, "otaend")) {
        printf("OTA_ERR not_ported\r\n");
    } else if (NULL != strstr((char *)dbgRecvBuf, "otaabort")) {
        printf("OTA_ERR not_ported\r\n");
    }

    /* getparams: print all key parameters */
    if (NULL != strstr((char *)dbgRecvBuf, "getparams")) {
        extern uint8_t NPP;
        extern int16_t g_theta_offset_pos, g_theta_offset_neg;
        extern int16_t g_theta_comp_pos, g_theta_comp_neg;
        printf("PARAMS_BEGIN\r\n");
        printf("NPP=%d elec_offset=%u\r\n", NPP, controller_eyou.FlashData.elec_offset);
        printf("Ia_off=%u Ib_off=%u\r\n", controller_eyou.FlashData.Ia_offset, controller_eyou.FlashData.Ib_offset);
        printf("CurKp=%u CurKi=%u CurKd=%u\r\n", controller_eyou.IncPID_QAxis.P, controller_eyou.IncPID_QAxis.I, controller_eyou.IncPID_QAxis.D);
        printf("SpdKp=%u SpdKi=%u SpdKd=%u\r\n", controller_eyou.IncPID_Speed.P, controller_eyou.IncPID_Speed.I, controller_eyou.IncPID_Speed.D);
        printf("PosKp=%u PosKi=%u PosKd=%u\r\n", controller_eyou.IncPID_Position.P, controller_eyou.IncPID_Position.I, controller_eyou.IncPID_Position.D);
        printf("OffPos=%d OffNeg=%d CompPos=%d CompNeg=%d\r\n",
               g_theta_offset_pos, g_theta_offset_neg, g_theta_comp_pos, g_theta_comp_neg);
        printf("MaxCur=%d MaxSpd=%ld\r\n", (int)controller_eyou.FlashData.MaxCurrent, (long)controller_eyou.FlashData.MaxSpeed);
        printf("PARAMS_END\r\n");
    }

    /* Clear receive buffer */
    memset((uint8_t *)dbgRecvBuf, 0, usart_rx_len);
    usart_rx_len = 0;
}

/* ============================================================
 * Debug log print - periodic state output (selectable by dbgLogFlag)
 * Migrated from cubemx_yxsui/foc/foc_fast/foc_bsp.c:1109-1604
 * Set log id via 'logid<N>' command, frequency via 'logfreq<ms>'
 * ============================================================ */
extern volatile uint16_t dbgLogFlag;
extern volatile uint16_t logPriodMs;

/* External symbols from foc layer (declared in various foc headers) */
extern volatile uint32_t g_adc_isr_cycles;
extern volatile uint32_t g_adc_isr_cycles_max;

void dbg_log_print(void)
{
    switch (dbgLogFlag) {
    case 1:
        controller_eyou.velocity_ref = 0;
        printf("dbg_log_print test\r\n");
        dbgLogFlag = 0;
        break;

    case 10:
        /* Mech/elec position + speed */
        printf("Angle_elec_360: %d, %d, %d, %d, %d\r\n",
               controller_eyou.now_mechposition,
               controller_eyou.theta_elec,
               controller_eyou.real_position_out,
               controller_eyou.real_position,
               controller_eyou.dtheta_mech / 1024);
        can_debug_send_log();
        break;

    case 11: {
        /* Output encoder debug: inner_raw update check */
        DPT_Angles angles;
        DPT_GetLatestAngles(&angles);
        printf("Out_enc: inner_raw=%lu outer_raw=%lu | pos_out=%ld pos_out_pre=%ld | old_cnt=%ld circle=%d\r\n",
               (unsigned long)angles.inner_raw,
               (unsigned long)angles.outer_raw,
               (long)controller_eyou.real_position_out,
               (long)controller_eyou.real_position_out_pre,
               (long)controller_eyou.old_angle_count_out,
               controller_eyou.circle_count_out);
        can_debug_send_log();
        break;
    }

    case 30:
        printf("current_get: %d,%d\r\n", controller_eyou.V_q, controller_eyou.V_d);
        can_debug_send_log();
        break;

    case 40:
        printf("current_pi: %d, %d, %d, %d, %d, %d, %d\r\n",
               can_wly_iq_fb_get(),
               controller_eyou.I_d,
               controller_eyou.V_q,
               controller_eyou.V_d,
               controller_eyou.I_q_ref,
               controller_eyou.I_d_ref,
               controller_eyou.I_q_ref_filterd);
        can_debug_send_log();
        break;

    case 50:
        printf("speed: %d, %d, %d, %d, %d\r\n",
               controller_eyou.velocity_ref / 1024,
               controller_eyou.velocity_ref_filterd / 1024,
               controller_eyou.dtheta_mech / 1024,
               (controller_eyou.dtheta_mech_out * 10) / 1024,
               (controller_eyou.velocity_ref - controller_eyou.dtheta_mech) / 1024);
        can_debug_send_log();
        break;

    case 60:
        printf("%d, %d, %d\r\n",
               controller_eyou.CCR2, controller_eyou.CCR3, controller_eyou.CCR4);
        can_debug_send_log();
        break;

    case 70:
        printf("%d, %d, %d, %d, %d, %d\r\n",
               controller_eyou.CCR2, controller_eyou.CCR3, controller_eyou.CCR4,
               controller_eyou.I_a, controller_eyou.I_b, controller_eyou.I_c);
        can_debug_send_log();
        break;

    case 90:
        printf("%d, %d, %d\r\n",
               controller_eyou.Ia_raw, controller_eyou.Ib_raw, controller_eyou.Ic_raw);
        can_debug_send_log();
        break;

    case 100:
        printf("position: %f, %f, %f, %d\r\n",
               controller_eyou.position_ref / 1024.0,
               controller_eyou.real_position_out / 1024.0,
               (controller_eyou.position_ref - controller_eyou.real_position_out) / 1024.0,
               controller_eyou.FlashData.mech_offest_out);
        can_debug_send_log();
        break;

    case 110: {
        /* ADC ISR timing breakdown (us @192MHz, format: last/max) */
        uint32_t f = 192;
        printf("adc_isr_us tot:%lu/%lu read:%lu/%lu enc:%lu/%lu pos:%lu/%lu vel:%lu/%lu cur:%lu/%lu\r\n",
               (unsigned long)(g_adc_isr_cycles / f),     (unsigned long)(g_adc_isr_cycles_max / f),
               (unsigned long)(g_adc_isr_t_read / f),     (unsigned long)(g_adc_isr_t_read_max / f),
               (unsigned long)(g_adc_isr_t_enc / f),      (unsigned long)(g_adc_isr_t_enc_max / f),
               (unsigned long)(g_adc_isr_t_pos / f),      (unsigned long)(g_adc_isr_t_pos_max / f),
               (unsigned long)(g_adc_isr_t_vel / f),      (unsigned long)(g_adc_isr_t_vel_max / f),
               (unsigned long)(g_adc_isr_t_cur / f),      (unsigned long)(g_adc_isr_t_cur_max / f));
        break;
    }

    case 120: {
        /* Open-loop test status (1 Hz throttled) */
        extern int16_t v_d_test, v_q_test;
        static uint32_t t120 = 0;
        uint32_t now = systick_ms;
        if (now - t120 < 1000) break;
        t120 = now;
        printf("OpenLoop: theta=%u I_a=%d I_b=%d I_c=%d V_d=%d V_q=%d\r\n",
               (unsigned)controller_eyou.theta_elec,
               controller_eyou.I_a,
               controller_eyou.I_b,
               controller_eyou.I_c,
               v_d_test, v_q_test);
        break;
    }

    case 130: {
        /* DPT encoder statistics (1 Hz) */
        static uint32_t t130 = 0;
        uint32_t now = systick_ms;
        if (now - t130 < 1000) break;
        t130 = now;

        uint32_t ok, crc_err, len_err, busy;
        DPT_GetAsyncStats(&ok, &crc_err, &len_err, &busy);

        DPT_Angles angles;
        DPT_GetLatestAngles(&angles);

        printf("DPT: in=%.2f out=%.2f | ok=%lu crc=%lu len=%lu busy=%lu\r\n",
               angles.inner_deg, angles.outer_deg,
               (unsigned long)ok, (unsigned long)crc_err,
               (unsigned long)len_err, (unsigned long)busy);
        break;
    }

    case 140: {
        /* CC4 / Enc / ADC relative timing (T0 = ADC ISR entry, 1 Hz)
         * AT32 @192MHz, DWT cycles / 192 = us */
        static uint32_t t140 = 0;
        uint32_t now = systick_ms;
        if (now - t140 < 1000) break;
        t140 = now;

        extern volatile uint32_t g_adc_isr_in_cycles, g_adc_isr_out_cycles;
        extern volatile uint32_t g_tim1_cc4_cycles, g_tim1_cc4_exit_cycles;
        extern volatile uint32_t g_tim1_enc_done_cycles;
        uint32_t t_adc_in   = g_adc_isr_in_cycles;
        uint32_t t_adc_out  = g_adc_isr_out_cycles;
        uint32_t t_cc4_in   = g_tim1_cc4_cycles;
        uint32_t t_cc4_out  = g_tim1_cc4_exit_cycles;
        uint32_t t_enc_done = g_tim1_enc_done_cycles;

        int32_t d_adc_out  = (int32_t)(t_adc_out  - t_adc_in) / 192;
        int32_t d_cc4_in   = (int32_t)(t_cc4_in   - t_adc_in) / 192;
        int32_t d_cc4_out  = (int32_t)(t_cc4_out  - t_adc_in) / 192;
        int32_t d_enc_done = (int32_t)(t_enc_done - t_adc_in) / 192;
        /* Encoder done may be from previous period; negative → add 100us */
        if (d_enc_done < 0) d_enc_done += 100;
        if (d_cc4_in < 0) d_cc4_in += 100;
        if (d_cc4_out < 0) d_cc4_out += 100;

        printf("T0=ADC_in | ADC_out=%+ldus CC4_in=%+ldus CC4_out=%+ldus Enc_done=%+ldus\r\n",
               (long)d_adc_out, (long)d_cc4_in, (long)d_cc4_out, (long)d_enc_done);
        break;
    }

    case 150: {
        /* ADC sampling rate + raw current + offset (1 Hz throttled) */
        static uint32_t t150 = 0;
        static uint32_t last_count = 0;
        uint32_t now = systick_ms;
        uint32_t elapsed = now - t150;
        if (elapsed < 1000) break;
        t150 = now;

        uint32_t cnt = g_foc_current.sample_count;
        uint32_t delta = cnt - last_count;
        last_count = cnt;
        uint32_t rate = (delta * 1000) / elapsed;

        printf("ADC Rate=%luHz Cnt=%lu | Ia=%ld Ib=%ld | OffA=%ld OffB=%ld\r\n",
               (unsigned long)rate, (unsigned long)cnt,
               (long)g_foc_current.i_a_raw, (long)g_foc_current.i_b_raw,
               (long)g_adc_offset_a, (long)g_adc_offset_b);
        break;
    }

    case 151: {
        /* Vdc + Temperature monitoring (1 Hz) with conversion */
        static uint32_t t151 = 0;
        uint32_t now = systick_ms;
        if (now - t151 < 1000) break;
        t151 = now;
        adc_convert();
        extern ifly_Err_Pro_Type motorProValue;
        uint32_t udc_01v    = motorProValue.Udc;
        int16_t  t_board_c  = motorProValue.board_temp;
        int16_t  t_motor_c  = motorProValue.motor_temp;
        printf("Udc=%lu(0.1V) Tboard=%d(0.1C) Tmotor=%d(0.1C) | Raw: VDC=%lu Tmos=%lu Tmot=%lu\r\n",
               (unsigned long)udc_01v,
               (int)(t_board_c * 10),
               (int)(t_motor_c * 10),
               (unsigned long)g_vdc_raw,
               (unsigned long)g_temp_mos_raw,
               (unsigned long)g_temp_motor_raw);
        break;
    }

    case 160:
        /* Write Flash: save current FlashData */
        WriteDataToFlash();
        printf("WriteDataToFlash\r\n");
        dbgLogFlag = 0;
        break;

    case 161:
        /* Erase Flash sector */
        if (Flash_EraseSector() == HAL_OK) {
            printf("Flash erase OK\r\n");
        } else {
            printf("Flash erase FAIL\r\n");
        }
        dbgLogFlag = 0;
        break;

    case 162: {
        /* Dump FlashData: RAM vs Flash comparison */
        FlashSavedData flash_copy;
        Flash_ReadData(FLASH_USER_START_ADDR, &flash_copy, sizeof(FlashSavedData));
        FlashSavedData *ram = &controller_eyou.FlashData;
        FlashSavedData *fls = &flash_copy;
        extern int16_t g_theta_offset_pos, g_theta_offset_neg;
        extern int16_t g_theta_comp_pos, g_theta_comp_neg;
        printf("===== FlashData Dump (RAM vs Flash) =====\r\n");
        printf("                    RAM              Flash\r\n");
        printf("[Header]\r\n");
        printf("  Ver             %-16u %u\r\n", ram->StructVersion, fls->StructVersion);
        printf("  CurFlag         0x%02X             0x%02X\r\n", ram->CurrentFlag, fls->CurrentFlag);
        printf("  AngFlag         0x%02X             0x%02X\r\n", ram->AngleOffsetFlag, fls->AngleOffsetFlag);
        printf("  PidFlag         0x%02X             0x%02X\r\n", ram->PidFlag, fls->PidFlag);
        printf("  ArrFlag         0x%02X             0x%02X\r\n", ram->ArrivedFlag, fls->ArrivedFlag);
        printf("  RunFlag         0x%02X             0x%02X\r\n", ram->RunDataFlag, fls->RunDataFlag);
        printf("  PosLimFlag      0x%02X             0x%02X\r\n", ram->PositionLimitFlag, fls->PositionLimitFlag);
        printf("  PrtFlag         0x%02X             0x%02X\r\n", ram->ProteckKeyFlag, fls->ProteckKeyFlag);
        printf("[Iofs]\r\n");
        printf("  Ia              %-16u %u\r\n", ram->Ia_offset, fls->Ia_offset);
        printf("  Ib              %-16u %u\r\n", ram->Ib_offset, fls->Ib_offset);
        printf("  Ic              %-16u %u\r\n", ram->Ic_offset, fls->Ic_offset);
        printf("[Angle]\r\n");
        printf("  elec            %-16u %u\r\n", ram->elec_offset, fls->elec_offset);
        printf("  PhaseOrder      %-16u %u\r\n", ram->PhaseOrder, fls->PhaseOrder);
        printf("  mech            %-16ld %ld\r\n", (long)ram->mech_offest, (long)fls->mech_offest);
        printf("  mech_out        %-16ld %ld\r\n", (long)ram->mech_offest_out, (long)fls->mech_offest_out);
        printf("[PosPID]\r\n");
        printf("  Kp              %-16lu %lu\r\n", (unsigned long)ram->Position_Kp, (unsigned long)fls->Position_Kp);
        printf("  Ki              %-16lu %lu\r\n", (unsigned long)ram->Position_Ki, (unsigned long)fls->Position_Ki);
        printf("  Kd              %-16lu %lu\r\n", (unsigned long)ram->Position_Kd, (unsigned long)fls->Position_Kd);
        printf("  Lim             %-16ld %ld\r\n", (long)ram->Pid_PositionLimit, (long)fls->Pid_PositionLimit);
        printf("  FF_Kp           %-16ld %ld\r\n", (long)ram->PosErrFF_Kp, (long)fls->PosErrFF_Kp);
        printf("[SpdPID]\r\n");
        printf("  Kp              %-16lu %lu\r\n", (unsigned long)ram->Speed_Kp, (unsigned long)fls->Speed_Kp);
        printf("  Ki              %-16lu %lu\r\n", (unsigned long)ram->Speed_Ki, (unsigned long)fls->Speed_Ki);
        printf("  Kd              %-16lu %lu\r\n", (unsigned long)ram->Speed_Kd, (unsigned long)fls->Speed_Kd);
        printf("  Lim             %-16ld %ld\r\n", (long)ram->Pid_SpeedLimit, (long)fls->Pid_SpeedLimit);
        printf("[CurPID]\r\n");
        printf("  Kp              %-16lu %lu\r\n", (unsigned long)ram->Current_Kp, (unsigned long)fls->Current_Kp);
        printf("  Ki              %-16lu %lu\r\n", (unsigned long)ram->Current_Ki, (unsigned long)fls->Current_Ki);
        printf("  Kd              %-16lu %lu\r\n", (unsigned long)ram->Current_Kd, (unsigned long)fls->Current_Kd);
        printf("  Lim             %-16ld %ld\r\n", (long)ram->Pid_CurrentLimit, (long)fls->Pid_CurrentLimit);
        printf("[Arrive]\r\n");
        printf("  Pos(0.1d)       %-16u %u\r\n", ram->PositionArrivedValue, fls->PositionArrivedValue);
        printf("  Spd(0.1rpm)     %-16u %u\r\n", ram->SpeedArrivedValue, fls->SpeedArrivedValue);
        printf("  Cur(0.1A)       %-16u %u\r\n", ram->CurrentArrivedValue, fls->CurrentArrivedValue);
        printf("[Run]\r\n");
        printf("  Mode            %-16u %u\r\n", ram->RunMode, fls->RunMode);
        printf("  MaxSpd(0.1rpm)  %-16ld %ld\r\n", (long)ram->MaxSpeed, (long)fls->MaxSpeed);
        printf("  MaxCur(0.1A)    %-16u %u\r\n", ram->MaxCurrent, fls->MaxCurrent);
        printf("  PosMax          %-16ld %ld\r\n", (long)ram->MaxPositionLimit, (long)fls->MaxPositionLimit);
        printf("  PosMin          %-16ld %ld\r\n", (long)ram->MinPositionLimit, (long)fls->MinPositionLimit);
        printf("[Prtct]\r\n");
        printf("  Sto1            %-16u %u\r\n", ram->Sto_1_protectKey, fls->Sto_1_protectKey);
        printf("  Sto2            %-16u %u\r\n", ram->Sto_2_protectKey, fls->Sto_2_protectKey);
        printf("  BusVol          %-16u %u\r\n", ram->BusVolProteckKey, fls->BusVolProteckKey);
        printf("  LockRot         %-16u %u\r\n", ram->LockedRotorProtectKey, fls->LockedRotorProtectKey);
        printf("  StoState        %-16lu %lu\r\n", (unsigned long)ram->stoStateFlag, (unsigned long)fls->stoStateFlag);
        printf("[Misc]\r\n");
        printf("  BrakeT          %-16u %u\r\n", ram->brake_time, fls->brake_time);
        printf("  Crc             0x%08lX       0x%08lX\r\n", (unsigned long)ram->Crc, (unsigned long)fls->Crc);
        printf("[PhaseComp]\r\n");
        printf("  PhCompFlag      0x%02X             0x%02X\r\n", ram->PhaseCompFlag, fls->PhaseCompFlag);
        printf("  OffsetPos(0.1d) %-16d %d\r\n",
               (int)g_theta_offset_pos, (int)(int16_t)(fls->temp5 & 0xFFFF));
        printf("  OffsetNeg(0.1d) %-16d %d\r\n",
               (int)g_theta_offset_neg, (int)(int16_t)((fls->temp5 >> 16) & 0xFFFF));
        printf("  CompPos(0.1)    %-16d %d\r\n",
               (int)g_theta_comp_pos, (int)(int16_t)(fls->temp6 & 0xFFFF));
        printf("  CompNeg(0.1)    %-16d %d\r\n",
               (int)g_theta_comp_neg, (int)(int16_t)((fls->temp6 >> 16) & 0xFFFF));
        printf("[Size] sizeof(FlashSavedData)=%u\r\n", (unsigned)sizeof(FlashSavedData));
        printf("===== End =====\r\n");
        dbgLogFlag = 0;
        break;
    }

    case 163:
        /* Clear all fault flags */
        ClearFaults(1);
        printf("All faults cleared, ready to restart\r\n");
        dbgLogFlag = 0;
        break;

    case 164:
        /* Set current position as zero (homing) */
        controller_eyou.controller_mode = HOMING_MODE;
        Reset_objReset_Output_Encoder(1);
        controller_eyou.UserDataSaveFlag = 1;
        printf("Reset_objReset_Output_Encoder: %ld\r\n",
               (long)controller_eyou.FlashData.mech_offest_out);
        Reset_objReset_Output_Encoder(0);
        dbgLogFlag = 0;
        break;

    case 165:
        /* Show fault flags */
        printf("ServoErrFlag = 0x%08lX\r\n",
               (unsigned long)controller_eyou.ServoErrFlag.All_Flag);
        if (controller_eyou.ServoErrFlag.All_Flag != 0) {
            extern void print_fault_types_pub(void);
            print_fault_types_pub();
        } else {
            printf("  No faults\r\n");
        }
        dbgLogFlag = 0;
        break;

    case 200: {
        /* Torque diagnostic snapshot (50ms throttle) */
        static uint32_t t200 = 0;
        if (systick_ms - t200 < 50) break;
        t200 = systick_ms;
        ControllerStruct *c = &controller_eyou;
        extern volatile int16_t g_vs_limit;
        extern ifly_Err_Pro_Type motorProValue;
        extern Portection_Value Threshold;

        int32_t spd_ref_x100  = (int32_t)((float)c->velocity_ref         / (FOC_GEAR_RATIO*1024.0f/100.0f));
        int32_t spd_filt_x100 = (int32_t)((float)c->velocity_ref_filterd / (FOC_GEAR_RATIO*1024.0f/100.0f));
        int32_t spd_mech_x100 = c->dtheta_mech / (1024/100);
        int32_t spd_err_x100  = (int32_t)(((float)c->velocity_ref_filterd/FOC_GEAR_RATIO - (float)c->dtheta_mech) / (1024.0f/100.0f));
        int32_t vs_q10 = (int32_t)sqrtf((float)c->V_d*c->V_d + (float)c->V_q*c->V_q);
        float udc_v = (float)motorProValue.Udc * 0.1f;
        int mod_pct = (udc_v>1.0f) ? (int)(((float)vs_q10/1024.0f)*1.732f/udc_v*100.0f) : 0;

        /* Saturation flags */
        int sat_spd = (c->IncPID_Speed.OutPut >= c->IncPID_Speed.OutputMax-1) ? 1
                    : (c->IncPID_Speed.OutPut <= -c->IncPID_Speed.OutputMax+1) ? -1 : 0;
        int sat_iq  = (c->IncPID_QAxis.OutPut >= c->IncPID_QAxis.OutputMax-1) ? 1
                    : (c->IncPID_QAxis.OutPut <= -c->IncPID_QAxis.OutputMax+1) ? -1 : 0;
        int sat_id  = (c->IncPID_DAxis.OutPut >= c->IncPID_DAxis.OutputMax-1) ? 1
                    : (c->IncPID_DAxis.OutPut <= -c->IncPID_DAxis.OutputMax+1) ? -1 : 0;
        int sat_vs  = (vs_q10 >= g_vs_limit - 16) ? 1 : 0;
        int sat_pos = (c->IncPID_Position.OutPut >= c->IncPID_Position.OutputMax-1) ? 1
                    : (c->IncPID_Position.OutPut <= -c->IncPID_Position.OutputMax+1) ? -1 : 0;

        /* BEMF estimate */
        float omega_e = (float)c->dtheta_mech * c->bemf_omega_e_k;
        float vd_bemf = -omega_e * c->ident_test.Lq * ((float)c->I_q / 1024.0f);
        float vq_bemf = omega_e * (c->ident_test.Ld * ((float)c->I_d / 1024.0f) + c->ident_test.flux_psi);

        printf("[L200/1] mode=%d run=%d err=0x%08lX | sref=%ld sflt=%ld smech=%ld serr=%ld (0.01rpm) | Imax=%u(Q10)\r\n",
               c->controller_mode, c->foc_run, (unsigned long)c->ServoErrFlag.All_Flag,
               (long)spd_ref_x100,(long)spd_filt_x100,(long)spd_mech_x100,(long)spd_err_x100,
               (unsigned)c->FlashData.MaxCurrent);
        printf("[L200/2] SpdPID Kp=%u Ki=%u Div=%u | aim=%ld now=%ld err=%ld out=%ld/%ld sat=%d -> Iq_ref=%ld(Q10)\r\n",
               (unsigned)c->IncPID_Speed.P, (unsigned)c->IncPID_Speed.I, (unsigned)c->IncPID_Speed.PID_Div,
               (long)c->IncPID_Speed.AimValue, (long)c->IncPID_Speed.NowValue,
               (long)c->IncPID_Speed.iError, (long)c->IncPID_Speed.OutPut,
               (long)c->IncPID_Speed.OutputMax, sat_spd, (long)c->I_q_ref);
        printf("[L200/3] CurPID Kp=%u Ki=%u Div=%u | Iqref_f=%ld Iq=%ld Iq_err=%ld Id=%ld Idref=%ld | Vq=%ld Vd=%ld Vs=%ld(Q10) Vlim=%d sat:Iq=%d Id=%d Vs=%d | Udc=%.1fV mod=%d%% | BEMF_ff=%d Vd_ff=%.2fV Vq_ff=%.2fV psi=%.4f Lq=%.2fmH\r\n",
               (unsigned)c->IncPID_QAxis.P, (unsigned)c->IncPID_QAxis.I, (unsigned)c->IncPID_QAxis.PID_Div,
               (long)c->I_q_ref_filterd, (long)c->I_q, (long)(c->I_q_ref_filterd - c->I_q), (long)c->I_d, (long)c->I_d_ref,
               (long)c->V_q, (long)c->V_d, (long)vs_q10, (int)g_vs_limit,
               sat_iq, sat_id, sat_vs, udc_v, mod_pct,
               USE_BEMF_FF, vd_bemf, vq_bemf, c->ident_test.flux_psi, c->ident_test.Lq * 1000.0f);
        printf("[L200/4] Ia=%ld Ib=%ld Ic=%ld(Q10) | Ialpha=%ld Ibeta=%ld | raw a=%lu b=%lu | offA=%u offB=%u | theta_e=%ld order=%u\r\n",
               (long)c->I_a,(long)c->I_b,(long)c->I_c,
               (long)c->I_alpha,(long)c->I_beta,
               (unsigned long)c->Ia_raw,(unsigned long)c->Ib_raw,
               (unsigned)c->FlashData.Ia_offset,(unsigned)c->FlashData.Ib_offset,
               (long)c->theta_elec,(unsigned)c->FlashData.PhaseOrder);
        printf("[L200/4b] WMAG=%d id_weak=%ld Us_filt=%lu vs_excess=%ld trig=%d\r\n",
               USE_WEAK_MAGN, (long)c->compensation_weak, (unsigned long)c->Us, (long)c->voltage_error,
               (c->compensation_weak < 0) ? 1 : 0);
        {
            int dc2 = (PWM_T > 0) ? (int)((int64_t)c->CCR2 * 1000 / PWM_T) : 0;
            int dc3 = (PWM_T > 0) ? (int)((int64_t)c->CCR3 * 1000 / PWM_T) : 0;
            int dc4 = (PWM_T > 0) ? (int)((int64_t)c->CCR4 * 1000 / PWM_T) : 0;
            printf("[L200/5] CCR=%lu/%lu/%lu (%d.%d/%d.%d/%d.%d%%) PWM_T=%d | SpdRamp now=%ld step=%ld vref=%ld | CurRamp now=%ld step=%ld iqref=%ld\r\n",
                   (unsigned long)c->CCR2,(unsigned long)c->CCR3,(unsigned long)c->CCR4,
                   dc2/10,dc2%10, dc3/10,dc3%10, dc4/10,dc4%10, PWM_T,
                   (long)c->SpeedSmooth.NowVelocityRef,(long)c->SpeedSmooth.MaxVelAccEveryPrd,(long)c->velocity_ref,
                   (long)c->CurrentSmooth.NowCurrentRef,(long)c->CurrentSmooth.MaxCurAccEveryPrd,(long)c->I_q_ref);
        }
        printf("[L200/6] MaxSpd=%ld(load Q10) OverI=%u(Q10) OverUdc=%u(0.1V) | PosRef=%ld PosOut=%ld dPosOut_eq=%ld(Q10rpm) | PosPID out=%ld/%ld sat=%d\r\n",
               (long)c->FlashData.MaxSpeed, (unsigned)Threshold.OverCurrent, (unsigned)Threshold.OverUdc,
               (long)c->position_ref, (long)c->real_position_out, (long)((float)c->dtheta_mech/FOC_GEAR_RATIO),
               (long)c->IncPID_Position.OutPut, (long)c->IncPID_Position.OutputMax, sat_pos);
        break;
    }

    default:
        break;
    }
}
