/* Exercise the production parser, control gates and capture path without a board. */
static int inject_stop_during_parse;
#include "../../motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/src/mc_host.c"
int _fltused;
void *memcpy(void *d,const void *s,size_t n) {unsigned char *q=d;const unsigned char *p=s;while(n--)*q++=*p++;return d;}
void *memset(void *d,int c,size_t n) {unsigned char *q=d;while(n--)*q++=(unsigned char)c;return d;}
char *strcpy(char *d,const char *s) {char *q=d;while((*q++=*s++)){}return d;}
int strcmp(const char *a,const char *b) {while(*a && *a==*b){a++;b++;}return (unsigned char)*a-(unsigned char)*b;}
int printf(const char *f,...) {(void)f;return 0;}
long strtol(const char *s,char **end,int base) {
 uint64_t v=0;int sign=1;const char *start=s;(void)base;
 if(*s=='-' || *s=='+') {if(*s=='-')sign=-1;s++;}
 const char *digits=s;
 while(*s>='0' && *s<='9') {v=v*10+(unsigned)(*s++-'0');if(v>2147483648ULL)errno=1;}
 if(s==digits) {*end=(char*)start;return 0;}
 *end=(char*)s;if((sign==1 && v>2147483647ULL) || v>2147483648ULL)errno=1;
 if(inject_stop_during_parse) {
   const char *request="motor stop\n"; inject_stop_during_parse=0;
   while(*request) mc_host_rx_byte((uint8_t)*request++);
 }
 return (long)(sign==1?(int64_t)v:-(int64_t)v);
}
#define CHECK(x) do {if(!(x)) return __LINE__;}while(0)
static void send(const char *s) {while(*s)mc_host_rx_byte((uint8_t)*s++);mc_host_rx_byte('\n');mc_host_task();}
static void ready(void) {
 esc_state=esc_state_old=ESC_STATE_SAFETY_READY;error_code=0;foc_rdy=0;start_stop_btn_flag=0;
 operation=0;hall_learn.start_flag=0;I_auto_tune.state_flag=UNDO;motor_param_ident.state_flag=UNDO;
}
int main(void) {
 int i;uint32_t count;
 ready();curr_offset_rdy=1;I_auto_tune.BW_limit=20000;adc_in_tab[0]=1418;
 pwm_duty.half_duty=3000;pwm_duty.double_duty=12000;
 pwm_duty.UF.a=pwm_duty.UF.b=pwm_duty.UF.c=3000;
 hall.state=1;pid_spd.ki_shift=15;pid_id.ki_shift=15;pid_iq.ki_shift=15;
 send("motor param set demo_current_limit_ma 2000");CHECK(ma(current.max_current)>1990 && ma(current.max_current)<=2000);
 send("motor param set demo_id_kp 1234");CHECK(pid_id.kp_gain==1234);
 send("motor param set demo_id_kp 40000");CHECK(pid_id.kp_gain==1234);
 send("motor param set demo_id_kp 123x");CHECK(pid_id.kp_gain==1234);
 send("motor param set demo_id_kp 2147483648");CHECK(pid_id.kp_gain==1234);
 send("motor param set demo_id_kp 100 extra");CHECK(pid_id.kp_gain==1234);
 esc_state_old=ESC_STATE_FREE_RUN;send("motor param set demo_id_kp 20");CHECK(pid_id.kp_gain==1234);
 ready();send("speed set 100");CHECK(!start_stop_btn_flag);
 send("speed start 100");CHECK(start_stop_btn_flag && speed_ramp.cmd_final==100 && ctrl_mode==SPEED_CTRL);
 CHECK(takeover_count==1);
 send("motor param set demo_id_kp 21");CHECK(pid_id.kp_gain==1234);
 esc_state=esc_state_old=ESC_STATE_RUNNING;foc_rdy=1;
 send("current start 0 1000");CHECK(ctrl_mode==SPEED_CTRL);
 send("speed set -200");CHECK(speed_ramp.cmd_final==-200);
 send("speed set -2147483648");CHECK(speed_ramp.cmd_final==-200);
 send("motor stop");CHECK(!start_stop_btn_flag && !foc_rdy && off_count && speed_ramp.cmd_final==0);
 ready();send("current start 0 1500");CHECK(ctrl_mode==TORQUE_CTRL && current.Iqdref.q>0);
 ready();send("current start 0 10000");CHECK(!start_stop_btn_flag);
 send("motor param set demo_current_limit_ma 0");CHECK(current.max_current==0 && pid_spd.upper_limit_integral==0);
 send("current start 0 1");CHECK(!start_stop_btn_flag);
 ready();send("open start 0 1000 5000 1000");CHECK(!start_stop_btn_flag);
 send("open start 0 1000 5000 0");CHECK(start_stop_btn_flag && ctrl_mode==OPEN_LOOP_CTRL && openloop.inc==10);
 CHECK(mv(openloop.volt.q)>995 && mv(openloop.volt.q)<1005);
 ready();send("motor commissioning accept");CHECK(!operation);
 send("identify flux");CHECK(!operation);
 send("identify winding");CHECK(operation==1 && esc_state==ESC_STATE_WINDING_PARAM_ID);
 send("motor param set demo_id_kp 30");CHECK(pid_id.kp_gain==1234);
 send("motor commissioning abort");CHECK(operation==0 && motor_param_ident.state_flag==FAILED);
 ready();hall_learn_state_table[0]=5;hall_learn.dir=1;
 send("calibrate hall angle");CHECK(operation==2 && hall_learn.start_flag);
 hall_learn_state_table[0]=3;hall_learn.dir=0;
 send("motor stop");CHECK(!operation && !hall_learn.start_flag);
 CHECK(hall_restore_count==1 && hall_learn_state_table[0]==5 && hall_learn.dir==1);
 ready();send("calculate current_pi");CHECK(operation==3 && I_auto_tune.state_flag==PROCESSING);
 I_auto_tune.state_flag=SUCCEED;mc_host_task();CHECK(!operation);
 ready();send("motor commissioning save");CHECK(save_count==1);
 send("debug 0");send("logid 10");send("logdetail 2");send("logcapture 128");CHECK(cap_armed);
 esc_state=esc_state_old=ESC_STATE_RUNNING;ctrl_mode=TORQUE_CTRL;foc_rdy=1;
 current.Iqd.q=1000;current.Iqdref.q=2000;
 for(i=0;i<128;i++)mc_host_sample();CHECK(!cap_armed && cap_count==128);
 CHECK(capture[127].us-capture[0].us>=7937 && capture[127].us-capture[0].us<=7938);
 CHECK((capture[0].flags & 257)==257 && capture[0].v[0]>4000);
 for(i=0;i<128;i++)mc_host_task();CHECK(cap_count==0);
 send("debug 0");send("logid 30");send("lograte 500");send("debug 1");
 for(i=0;i<32;i++)mc_host_sample();CHECK(live_head!=live_tail && live[live_tail].v[0]==5000);
 CHECK(live[live_tail].v[6]>23900 && live[live_tail].v[6]<24100);
 send("debug 0");ready();
 mc_host_rx_byte('s');mc_host_rx_byte('p');mc_host_task();CHECK(!start_stop_btn_flag);
 const char *suffix="eed start 250\r\n";while(*suffix)mc_host_rx_byte(*suffix++);mc_host_task();
 CHECK(start_stop_btn_flag && speed_ramp.cmd_final==250);
 send("motor stop");ready();count=rx_errors;
 for(i=0;i<140;i++)mc_host_rx_byte('x');mc_host_rx_byte('\n');mc_host_task();CHECK(rx_errors==count+1);
 send("speed start 300");CHECK(start_stop_btn_flag && speed_ramp.cmd_final==300);
 send("motor stop");ready();
 const char *batch="motor param set demo_id_kp 31\nmotor param set demo_id_ki 32\n";
 while(*batch)mc_host_rx_byte(*batch++);mc_host_task();mc_host_task();CHECK(pid_id.kp_gain==31 && pid_id.ki_gain==32);
 mc_host_rx_error();send("speed start 100");CHECK(!start_stop_btn_flag);
 send("speed start 101");CHECK(start_stop_btn_flag && speed_ramp.cmd_final==101);
 send("motor stop");ready();
 const char *queued="speed start 1\nspeed start 2\nspeed start 3\nspeed start 4\nmotor stop\n";
 while(*queued)mc_host_rx_byte(*queued++);mc_host_task();
 CHECK(!start_stop_btn_flag && rx_head==rx_tail && !rx_stop_pending);
 /* A stop IRQ arriving after dequeue but during numeric parsing must win. */
 ready();inject_stop_during_parse=1;send("speed start 100");
 CHECK(!start_stop_btn_flag && !foc_rdy && mc_host_stop_requested());
 mc_host_task();CHECK(!mc_host_stop_requested());
 /* Receiving stop must cut output before the next main-loop iteration. */
 ready();start_stop_btn_flag=1;foc_rdy=1;esc_state=esc_state_old=ESC_STATE_RUNNING;
 const char *stop_irq="motor stop\n";
 while(*stop_irq)mc_host_rx_byte((uint8_t)*stop_irq++);
 CHECK(!start_stop_btn_flag && !foc_rdy && mc_host_stop_requested());
 mc_host_task();CHECK(!operation && !mc_host_stop_requested());
 /* Failed and timed-out winding ID must restore both live and old model values. */
 ready();motor_param_ident.Rs.f=0.08f;motor_param_ident.Ls.f=0.00005f;
 send("identify winding");motor_param_ident.Rs.f=0.02f;motor_param_ident.Ls.f=0.000001f;
 error_code=MC_HALL_ERROR;esc_state=ESC_STATE_ERROR;mc_host_task();
 CHECK(!operation && motor_param_ident.state_flag==FAILED);
 CHECK(motor_param_ident.Rs.f==0.08f && motor_param_ident.Ls.f==0.00005f);
 CHECK(motor_param_ident.Rs_Old.f==0.08f && motor_param_ident.Ls_Old.f==0.00005f);
 ready();send("identify winding");motor_param_ident.Rs.f=0.03f;
 host_ms=operation_started+30001U;mc_host_task();
 CHECK(!operation && motor_param_ident.Rs.f==0.08f && !foc_rdy);
 /* Successful ID results must remain active, including after a later stop. */
 ready();send("identify winding");motor_param_ident.Rs.f=0.09f;
 motor_param_ident.state_flag=SUCCEED;mc_host_task();CHECK(!operation);
 send("motor stop");CHECK(motor_param_ident.Rs.f==0.09f);
 /* Reject unrepresentable voltage and vector magnitude before int16 conversion. */
 ready();adc_in_tab[0]=154;send("open start 0 4800 5000 0");
 CHECK(!start_stop_btn_flag && openloop.volt.q>=0);
 ready();send("open start 1500 1500 5000 0");CHECK(!start_stop_btn_flag);
 ready();send("open start 0 800 5000 0");
 CHECK(start_stop_btn_flag && openloop.volt.q>0 && mv(openloop.volt.q)>=795);
 send("motor stop");ready();send("open start 0 -800 5000 0");
 CHECK(start_stop_btn_flag && openloop.volt.q<0 && mv(openloop.volt.q)<=-795);
 send("motor stop");ready();adc_in_tab[0]=1418;
 send("open start 4800 4800 5000 0");CHECK(start_stop_btn_flag && openloop.volt.d>0 && openloop.volt.q>0);
 send("motor stop");
 return 0;
}
