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
        controller_eyou.foc_run = 0;
        TIM1_PWM_Stop();
        wk_delay_ms(50);
        NVIC_SystemReset();
    }

    /* CurrentPID Kp<x>Ki<x>Kd<x> */
    if (NULL != strstr((char *)dbgRecvBuf, "CurrentPID")) {
        printf("CurPID:%d,%d,%d\r\n",
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
            printf("CurPID set:%d,%d,%d\r\n", kp, ki, kd);
        }
    }

    /* SpeedPID Kp<x>Ki<x>Kd<x> */
    if (NULL != strstr((char *)dbgRecvBuf, "SpeedPID")) {
        printf("SpdPID:%d,%d,%d\r\n",
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
            printf("SpdPID set:%d,%d,%d\r\n", kp, ki, kd);
        }
    }

    /* PositionPID Kp<x>Ki<x>Kd<x> */
    if (NULL != strstr((char *)dbgRecvBuf, "PositionPID")) {
        printf("PosPID:%d,%d,%d\r\n",
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
            printf("PosPID set:%d,%d,%d\r\n", kp, ki, kd);
        }
    }

    /* Run cmd<0/1> M<mode> tar<value>
     * mode: 2=torque, 3=velocity, 4=position
     * tar: torque(Nm), velocity(rpm), position(deg) */
    if (NULL != strstr((char *)dbgRecvBuf, "Run")) {
        loc = strstr((char *)dbgRecvBuf, "cmd");
        int cmd_val = loc ? atoi(loc + 3) : 0;
        loc = strstr((char *)dbgRecvBuf, "M");
        int mode_val = loc ? atoi(loc + 1) : 2;
        char *tar_str = strstr((char *)dbgRecvBuf, "tar");
        float tar_value = tar_str ? atof(tar_str + 3) : 0.0f;

        if (cmd_val == 0) {
            controller_eyou.foc_run = 0;
            TIM1_PWM_Stop();
            printf("Run: stopped\r\n");
        } else {
            ResetControlData(&controller_eyou);
            controller_eyou.foc_run = cmd_val;
            controller_eyou.controller_mode = mode_val;
            TMR1->brk |= TIM_BDTR_MOE;
            TMR1->cctrl |= 0x0555u;

            if (mode_val == PROFILE_TORQUE_MODE) {
                int32_t iq = (int32_t)(tar_value * 10.0f * 1024.0f);
                controller_eyou.I_q_ref = iq;
                printf("Run: torque tar=%.3f iq=%ld\r\n", tar_value, (long)iq);
            } else if (mode_val == PROFILE_VELOCITY_MOCE) {
                controller_eyou.velocity_ref = (int32_t)(tar_value * 1024.0f * 25.0f);
                printf("Run: vel tar=%.2f rpm\r\n", tar_value);
            } else if (mode_val == PROFILE_POSITION_MODE) {
                controller_eyou.position_ref = (int32_t)(tar_value * 1024.0f);
                printf("Run: pos tar=%.2f deg\r\n", tar_value);
            }
        }
    }

    /* enable<0/1>: PWM output enable/disable */
    if (NULL != strstr((char *)dbgRecvBuf, "enable")) {
        loc = strstr((char *)dbgRecvBuf, "enable");
        int en = atoi(loc + 6);
        if (en) {
            ResetControlData(&controller_eyou);
            TMR1->brk |= TIM_BDTR_MOE;
            TMR1->cctrl |= 0x0555u;
            printf("PWM enabled\r\n");
        } else {
            controller_eyou.foc_run = 0;
            TIM1_PWM_Stop();
            printf("PWM disabled\r\n");
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
        printf("logid=%d\r\n", id);
    }

    /* logfreq<ms>: set debug log period */
    if (NULL != (loc = strstr((char *)dbgRecvBuf, "logfreq"))) {
        uint16_t ms = (uint16_t)atoi(loc + 7);
        if (ms < 10) ms = 10;
        logPriodMs = ms;
        printf("logfreq=%dms\r\n", ms);
    }

    /* getparams: print all key parameters */
    if (NULL != strstr((char *)dbgRecvBuf, "getparams")) {
        extern uint8_t NPP;
        printf("NPP=%d elec_offset=%u\r\n", NPP, controller_eyou.FlashData.elec_offset);
        printf("Ia_off=%u Ib_off=%u\r\n", controller_eyou.FlashData.Ia_offset, controller_eyou.FlashData.Ib_offset);
        printf("CurPID: %d %d %d\r\n", controller_eyou.IncPID_QAxis.P, controller_eyou.IncPID_QAxis.I, controller_eyou.IncPID_QAxis.D);
        printf("SpdPID: %d %d %d\r\n", controller_eyou.IncPID_Speed.P, controller_eyou.IncPID_Speed.I, controller_eyou.IncPID_Speed.D);
        printf("PosPID: %d %d %d\r\n", controller_eyou.IncPID_Position.P, controller_eyou.IncPID_Position.I, controller_eyou.IncPID_Position.D);
        printf("MaxCur=%d MaxSpd=%ld\r\n", (int)controller_eyou.FlashData.MaxCurrent, (long)controller_eyou.FlashData.MaxSpeed);
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
        printf("Angle: now_mech=%d theta_e=%d pos_out=%ld pos=%ld spd=%ld\r\n",
               controller_eyou.now_mechposition,
               controller_eyou.theta_elec,
               (long)controller_eyou.real_position_out,
               (long)controller_eyou.real_position,
               (long)controller_eyou.dtheta_mech / 1024);
        break;

    case 11: {
        /* Output encoder debug: inner_raw update check */
        DPT_Angles angles;
        DPT_GetLatestAngles(&angles);
        printf("Out_enc: in_raw=%lu out_raw=%lu | pos_out=%ld pos_pre=%ld | old_cnt=%ld circle=%d\r\n",
               (unsigned long)angles.inner_raw,
               (unsigned long)angles.outer_raw,
               (long)controller_eyou.real_position_out,
               (long)controller_eyou.real_position_out_pre,
               (long)controller_eyou.old_angle_count_out,
               controller_eyou.circle_count_out);
        break;
    }

    case 30:
        /* DQ-axis voltage */
        printf("Vdq: V_q=%ld V_d=%ld\r\n",
               (long)controller_eyou.V_q, (long)controller_eyou.V_d);
        break;

    case 40:
        /* Current PI: Iq/Id/Vq/Vd/Iq_ref/Id_ref/Iq_filt */
        printf("CurPI: %ld, %ld, %ld, %ld, %ld, %ld, %ld\r\n",
               (long)controller_eyou.I_q,
               (long)controller_eyou.I_d,
               (long)controller_eyou.V_q,
               (long)controller_eyou.V_d,
               (long)controller_eyou.I_q_ref,
               (long)controller_eyou.I_d_ref,
               (long)controller_eyou.I_q_ref_filterd);
        break;

    case 50:
        /* Speed: ref(rpm), filt(rpm), motor(rpm), output(0.1rpm), err(rpm) */
        printf("Spd: %ld, %ld, %ld, %ld, %ld\r\n",
               (long)controller_eyou.velocity_ref / 1024,
               (long)controller_eyou.velocity_ref_filterd / 1024,
               (long)controller_eyou.dtheta_mech / 1024,
               (long)(controller_eyou.dtheta_mech_out * 10) / 1024,
               (long)(controller_eyou.velocity_ref - controller_eyou.dtheta_mech) / 1024);
        break;

    case 60:
        /* CCR values (PWM compare) */
        printf("CCR: %d, %d, %d\r\n",
               controller_eyou.CCR2, controller_eyou.CCR3, controller_eyou.CCR4);
        break;

    case 70:
        /* CCR + phase currents */
        printf("CCR+I: %d,%d,%d  I:%ld,%ld,%ld\r\n",
               controller_eyou.CCR2, controller_eyou.CCR3, controller_eyou.CCR4,
               (long)controller_eyou.I_a, (long)controller_eyou.I_b, (long)controller_eyou.I_c);
        break;

    case 90:
        /* Phase current raw */
        printf("Iraw: a=%u b=%u c=%u\r\n",
               (unsigned)controller_eyou.Ia_raw,
               (unsigned)controller_eyou.Ib_raw,
               (unsigned)controller_eyou.Ic_raw);
        break;

    case 100:
        /* Position loop status (deg) */
        printf("Pos: ref=%.3f act=%.3f err=%.3f\r\n",
               controller_eyou.position_ref / 1024.0f,
               controller_eyou.real_position_out / 1024.0f,
               (controller_eyou.position_ref - controller_eyou.real_position_out) / 1024.0f);
        break;

    case 110: {
        /* ADC ISR timing breakdown (us @192MHz, format: last/max) */
        uint32_t f = 192;
        printf("ISR_us tot:%lu/%lu read:%lu/%lu enc:%lu/%lu pos:%lu/%lu vel:%lu/%lu cur:%lu/%lu\r\n",
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
        printf("OpenLoop: theta=%u I_a=%ld I_b=%ld I_c=%ld V_d=%d V_q=%d\r\n",
               (unsigned)controller_eyou.theta_elec,
               (long)controller_eyou.I_a,
               (long)controller_eyou.I_b,
               (long)controller_eyou.I_c,
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

        printf("ADC: rate=%luHz cnt=%lu Ia=%ld Ib=%ld OffA=%ld OffB=%ld\r\n",
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
        extern volatile uint16_t g_udc_volt;
        int16_t t_mos = TemperatureInquiry((uint16_t)g_temp_mos_raw);
        int16_t t_mot = MotorTemperatureInquiry((uint16_t)g_temp_motor_raw);
        printf("Mon: VDC=%uV(raw=%lu) Tmos=%dC Tmot=%dC so_c=%u\r\n",
               (unsigned)g_udc_volt, (unsigned long)g_vdc_raw,
               (int)t_mos, (int)t_mot,
               (unsigned)g_so_c_raw);
        break;
    }

    case 160:
        /* Write Flash: save current FlashData */
        WriteDataToFlash();
        printf("WriteDataToFlash done\r\n");
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
        printf("===== FlashData (RAM / Flash) =====\r\n");
        printf("  Ver       %u / %u\r\n", ram->StructVersion, fls->StructVersion);
        printf("  Ia_off    %u / %u\r\n", ram->Ia_offset, fls->Ia_offset);
        printf("  Ib_off    %u / %u\r\n", ram->Ib_offset, fls->Ib_offset);
        printf("  elec_off  %u / %u\r\n", ram->elec_offset, fls->elec_offset);
        printf("  CurPID    %lu/%lu/%lu / %lu/%lu/%lu\r\n",
               (unsigned long)ram->Current_Kp, (unsigned long)ram->Current_Ki, (unsigned long)ram->Current_Kd,
               (unsigned long)fls->Current_Kp, (unsigned long)fls->Current_Ki, (unsigned long)fls->Current_Kd);
        printf("  SpdPID    %lu/%lu/%lu / %lu/%lu/%lu\r\n",
               (unsigned long)ram->Speed_Kp, (unsigned long)ram->Speed_Ki, (unsigned long)ram->Speed_Kd,
               (unsigned long)fls->Speed_Kp, (unsigned long)fls->Speed_Ki, (unsigned long)fls->Speed_Kd);
        printf("  PosPID    %lu/%lu/%lu / %lu/%lu/%lu\r\n",
               (unsigned long)ram->Position_Kp, (unsigned long)ram->Position_Ki, (unsigned long)ram->Position_Kd,
               (unsigned long)fls->Position_Kp, (unsigned long)fls->Position_Ki, (unsigned long)fls->Position_Kd);
        printf("  MaxCur=%u MaxSpd=%ld\r\n", ram->MaxCurrent, (long)ram->MaxSpeed);
        printf("  Crc       0x%08lX / 0x%08lX\r\n", (unsigned long)ram->Crc, (unsigned long)fls->Crc);
        printf("===== End =====\r\n");
        dbgLogFlag = 0;
        break;
    }

    case 163:
        /* Clear all fault flags */
        ClearFaults(1);
        printf("All faults cleared\r\n");
        dbgLogFlag = 0;
        break;

    case 164:
        /* Set current position as zero (homing) */
        controller_eyou.controller_mode = HOMING_MODE;
        Reset_objReset_Output_Encoder(1);
        controller_eyou.UserDataSaveFlag = 1;
        printf("Home set: mech_off_out=%ld\r\n",
               (long)controller_eyou.FlashData.mech_offest_out);
        Reset_objReset_Output_Encoder(0);
        dbgLogFlag = 0;
        break;

    case 165:
        /* Show fault flags */
        printf("ServoErrFlag = 0x%08lX\r\n",
               (unsigned long)controller_eyou.ServoErrFlag.All_Flag);
        if (controller_eyou.ServoErrFlag.All_Flag == 0) {
            printf("  No faults\r\n");
        }
        dbgLogFlag = 0;
        break;

    default:
        break;
    }
}
