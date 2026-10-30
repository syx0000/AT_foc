#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#define USE_HALL_HOST
#define MC_NO_ERROR 0
#define MC_HALL_ERROR 4
#define MAX_SPEED_RPM 4000
#define MAX_CURRENT 50.0f
#define CURRENT_BASE 66.0f
#define V_PHASE_BASE 40.0f
#define VOLTAGE_BASE 69.3f
#define VDC_RATED 48.0f
#define SQRT_3 1.7320508f
#define PWM_FREQ 16000
#define TMR_CLK 192000000U
#define POLE_PAIRS 4
#define ADC_BUS_VOLT_IDX 0
#define SET 1
#define RESET 0
#define CTRL_SOURCE_SOFTWARE 0
#define COMM_UART 0
#define USART_TDC_FLAG 1
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)(x))
typedef enum { OPEN_LOOP_CTRL, TORQUE_CTRL, SPEED_CTRL } motor_control_mode;
enum { ESC_STATE_SAFETY_READY, ESC_STATE_RUNNING, ESC_STATE_ERROR, ESC_STATE_IDLE,
       ESC_STATE_FREE_RUN, ESC_STATE_WINDING_PARAM_ID, ESC_STATE_AUTO_LEARN };
enum { UNDO, PROCESSING, SUCCEED, FAILED };
enum { PROCESS_0_LOCK, PROCESS_4_FINISH, PROCESS_5_ERROR };
typedef struct { int16_t d,q; } qd_type;
typedef struct { int16_t a,b,c; } abc_type;
typedef struct {
 int16_t kp_gain,ki_gain,kd_gain,upper_limit_output,lower_limit_output;
 int32_t integral,upper_limit_integral,lower_limit_integral,error,pre_error,out_start;
 uint16_t kp_shift,ki_shift;
} pid_ctrl_type;
static pid_ctrl_type pid_id,pid_iq,pid_spd;
static struct {int16_t max_current,min_current; qd_type Iqdref,Iqd; abc_type Iabc;} current;
static struct {int32_t cmd_final,command; int16_t acc_slope,dec_slope;} speed_ramp;
static struct {uint32_t pr;} pwm_timer={5999};
#define PWM_ADVANCE_TIMER (&pwm_timer)
static struct {abc_type UF; uint16_t half_duty,double_duty;} pwm_duty;
static struct {qd_type Vqd;} volt_cmd;
static struct {qd_type volt; int16_t inc,theta;} openloop;
static struct {int16_t elec_angle_val;} rotor_angle_hall;
static struct {uint8_t state;} hall;
static struct {int16_t learn_angle_inc,learn_volt,align_time; int process_state,start_flag,dir,check_flag;} hall_learn;
typedef union {float f; uint32_t u32;} float_union;
static struct {float_union Rs,Ls,Rs_Old,Ls_Old; int state_flag;} motor_param_ident;
static struct {uint16_t bandwidth,BW_limit,Vdc_rated_adc; int state_flag;} I_auto_tune;
static int esc_state,esc_state_old,error_code,start_stop_btn_flag,foc_rdy,curr_offset_rdy;
static motor_control_mode ctrl_mode,ctrl_mode_cmd,ctrl_mode_old;
static int ctrl_source;
static int32_t rotor_speed_val;
static uint16_t adc_in_tab[1],hall_learn_state_table[6];
static int takeover_count,off_count,save_count,reset_count;
static void mc_can_motor_serial_takeover(void) {takeover_count++;}
static void gate_driver_emergency_shutdown(void) {off_count++;}
static void motor_parameter_id_abort(void) {motor_param_ident.state_flag=FAILED;}
static int gate_driver_fault_recover(void) {return 1;}
static int gate_driver_enable(void) {return 1;}
static void gate_driver_disable(void) {}
static void enable_pwm_timer(void *p) {(void)p;}
static int I_offset_init(void *p) {(void)p;return 1;}
static void param_clear(void) {speed_ramp.cmd_final=0; current.Iqdref.q=0;}
static int usart_flag_get(int a,int b) {(void)a;(void)b;return 1;}
static void NVIC_SystemReset(void) {reset_count++;}
#define INT32_SIZE_ARRAY 16
static int32_t flashmock[16],save_param_buffer[16];
#define MC_VectStoreAddr_UINT32 ((uintptr_t)flashmock)
static void write_flash_cmd(void) {int i;save_count++;for(i=0;i<16;i++) flashmock[i]=save_param_buffer[i];}

static int hall_restore_count;
static void foc_hall_table_mapping(void) {hall_restore_count++;}
static void hall_timer_init(void) {}
