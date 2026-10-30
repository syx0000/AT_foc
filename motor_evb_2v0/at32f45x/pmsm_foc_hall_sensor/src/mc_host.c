/* Text adapter for the shared Hall host. No candidate parameters or new ID algorithms. */
#include "mc_lib.h"
#include "mc_host.h"
#include "mc_can_motor.h"
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#if defined(USE_HALL_HOST)
#define RX_SIZE 128U
#define RX_QUEUE 4U
#define LIVE_SIZE 8U
#define CAP_SIZE 128U
static char rx_queue[RX_QUEUE][RX_SIZE], rx_line[RX_SIZE];
static volatile uint8_t rx_head, rx_tail, rx_stop_pending;
static uint8_t rx_length, rx_discard;
static volatile uint32_t rx_errors, host_ms;
static uint8_t operation; /* 1 winding, 2 hall, 3 tuning */
static uint32_t operation_started;
static uint16_t hall_backup[6];
static int8_t hall_direction_backup;
static float winding_rs_backup, winding_ls_backup;
static void restore_hall_learning(void)
{
  unsigned i;
  for(i=0;i<6;i++) hall_learn_state_table[i]=hall_backup[i];
  hall_learn.dir=hall_direction_backup; hall_learn.check_flag=RESET;
  foc_hall_table_mapping(); hall_timer_init();
}

typedef struct { uint32_t seq, us, drops, flags; int32_t v[10]; } host_frame;
static host_frame live[LIVE_SIZE], capture[CAP_SIZE];
static volatile uint16_t live_head, live_tail, cap_count;
static volatile uint8_t trace_enabled, cap_armed;
static uint16_t cap_sent;
static volatile uint16_t trace_id=10, trace_detail, trace_rate=200;
static uint32_t sample_us, sample_cycles, trace_phase, trace_seq, trace_drops;
static int16_t position_previous;
static int64_t position_sum;
static uint8_t position_ready;

int mc_host_stop_requested(void) { return rx_stop_pending!=0U; }
void mc_host_tick(void) { host_ms++; }
void mc_host_rx_error(void) { rx_discard=1; rx_length=0; rx_errors++; }
void mc_host_rx_byte(uint8_t b)
{
  uint8_t next;
  if (b=='\r' || b=='\n') {
    if (rx_discard) { rx_discard=0; rx_length=0; return; }
    if (!rx_length) return;
    rx_line[rx_length]=0;
    if(!strcmp(rx_line,"motor stop") || !strcmp(rx_line,"speed stop") ||
       !strcmp(rx_line,"current stop") || !strcmp(rx_line,"open stop") ||
       !strcmp(rx_line,"motor commissioning abort")) {
      rx_stop_pending=1;
      /* ISR-safe register operations only. Cleanup remains in the main loop. */
      start_stop_btn_flag=RESET; foc_rdy=RESET;
      gate_driver_emergency_shutdown();
      rx_length=0; return;
    }
    next=(uint8_t)((rx_head+1U)%RX_QUEUE);
    if(next==rx_tail) rx_errors++;
    else { rx_line[rx_length]=0; memcpy(rx_queue[rx_head],rx_line,rx_length+1U); rx_head=next; }
    rx_length=0; return;
  }
  if(rx_discard) return;
  if(b<32U || b>126U || rx_length>=RX_SIZE-1U) { mc_host_rx_error(); return; }
  rx_line[rx_length++]=(char)b;
}
static int stopped(void)
{
  return !mc_host_stop_requested() && esc_state==ESC_STATE_SAFETY_READY && esc_state_old==esc_state && !start_stop_btn_flag && !foc_rdy &&
         error_code==MC_NO_ERROR && !operation && !hall_learn.start_flag &&
         I_auto_tune.state_flag!=PROCESSING && motor_param_ident.state_flag!=PROCESSING;
}
static int32_t ma(int32_t pu) { return (int32_t)((float)pu*CURRENT_BASE*1000.0f/32767.0f); }
static int16_t pu_ma(int32_t value) { return (int16_t)((float)value*32767.0f/(CURRENT_BASE*1000.0f)); }
/* Voltage commands are per-unit of actual Vbus/sqrt(3), not ADC full scale. */
static float phase_mv(void) { return (float)adc_in_tab[ADC_BUS_VOLT_IDX]*VOLTAGE_BASE*1000.0f/(4095.0f*SQRT_3); }
static int32_t mv(int32_t pu) { return (int32_t)((float)pu*phase_mv()/32767.0f); }
/* Snapshot the live bus once while interrupts are masked; reject before narrowing. */
static int open_voltage_convert(int32_t vd,int32_t vq,int16_t *d,int16_t *q)
{
  float available=phase_mv();
  float d_pu,q_pu;
  int64_t requested_sq=(int64_t)vd*vd+(int64_t)vq*vq;
  if(available<1000.0f || (float)requested_sq>available*available) return 0;
  d_pu=(float)vd*32767.0f/available; q_pu=(float)vq*32767.0f/available;
  if(d_pu>32767.0f || d_pu< -32767.0f || q_pu>32767.0f || q_pu< -32767.0f) return 0;
  *d=(int16_t)d_pu; *q=(int16_t)q_pu; return 1;
}
static void stop(void)
{
  uint32_t mask=__get_PRIMASK();
  __disable_irq();
  mc_can_motor_serial_takeover();
  start_stop_btn_flag=RESET; foc_rdy=RESET;
  gate_driver_emergency_shutdown();
  speed_ramp.cmd_final=0; speed_ramp.command=0;
  current.Iqdref.d=0; current.Iqdref.q=0;
  pid_spd.integral=0;
  hall_learn.start_flag=RESET;
  I_auto_tune.state_flag=UNDO;
  if(operation==1 || motor_param_ident.state_flag==PROCESSING) motor_parameter_id_abort();
  if(operation==1) {
    motor_param_ident.Rs.f=winding_rs_backup; motor_param_ident.Ls.f=winding_ls_backup;
    motor_param_ident.Rs_Old.f=winding_rs_backup; motor_param_ident.Ls_Old.f=winding_ls_backup;
  }
  if(operation==2) restore_hall_learning();
  if(esc_state!=ESC_STATE_ERROR) esc_state=ESC_STATE_FREE_RUN;
  __set_PRIMASK(mask);
  operation=0;
}
static void limit_set(int32_t value)
{
  int16_t pu=pu_ma(value);
  int32_t integral=(int32_t)((int64_t)pu*((int64_t)1<<pid_spd.ki_shift));
  current.max_current=pu; current.min_current=-pu;
  pid_spd.upper_limit_output=pu; pid_spd.lower_limit_output=-pu;
  pid_spd.upper_limit_integral=integral; pid_spd.lower_limit_integral=-integral;
  pid_spd.integral=0; current.Iqdref.q=0;
}
typedef struct { const char *name; int16_t *value; int32_t low,high; } native_parameter;
static native_parameter params[]={
  {"demo_id_kp",&pid_id.kp_gain,0,32767}, {"demo_id_ki",&pid_id.ki_gain,0,32767},
  {"demo_iq_kp",&pid_iq.kp_gain,0,32767}, {"demo_iq_ki",&pid_iq.ki_gain,0,32767},
  {"demo_speed_kp",&pid_spd.kp_gain,0,32767}, {"demo_speed_ki",&pid_spd.ki_gain,0,32767},
  {"demo_id_kp_shift",(int16_t*)&pid_id.kp_shift,0,15},
  {"demo_id_ki_shift",(int16_t*)&pid_id.ki_shift,0,15},
  {"demo_iq_kp_shift",(int16_t*)&pid_iq.kp_shift,0,15},
  {"demo_iq_ki_shift",(int16_t*)&pid_iq.ki_shift,0,15},
  {"demo_speed_kp_shift",(int16_t*)&pid_spd.kp_shift,0,15},
  {"demo_speed_ki_shift",(int16_t*)&pid_spd.ki_shift,0,15},
  {"demo_acc_rpm_ms",&speed_ramp.acc_slope,1,1000},
  {"demo_dec_rpm_ms",&speed_ramp.dec_slope,1,1000},
  {"demo_hall_angle_inc",&hall_learn.learn_angle_inc,1,100},
  {"demo_hall_align_ms",&hall_learn.align_time,10,10000}
};
static void integral_reset(pid_ctrl_type *p)
{
  p->integral=0; p->pre_error=0; p->error=0;
  p->upper_limit_integral=(int32_t)((int64_t)p->upper_limit_output*((int64_t)1<<p->ki_shift));
  p->lower_limit_integral=(int32_t)((int64_t)p->lower_limit_output*((int64_t)1<<p->ki_shift));
}
static void parameters(void)
{
  unsigned i;
  printf("OK params active firmware=demo parameter_mode=direct\r\n");
  for(i=0;i<sizeof(params)/sizeof(params[0]);i++) printf("%s=%d\r\n",params[i].name,*params[i].value);
  printf("demo_rs_mohm=%ld demo_ls_uh=%ld demo_bandwidth_rad_s=%u\r\n",
    (long)(motor_param_ident.Rs.f*1000.0f),(long)(motor_param_ident.Ls.f*1000000.0f),I_auto_tune.bandwidth);
  printf("demo_tune_bus_mv=%ld demo_current_limit_ma=%ld demo_hall_scan_mv=%ld\r\n",
    (long)((float)I_auto_tune.Vdc_rated_adc*VOLTAGE_BASE*1000.0f/4095.0f),
    (long)ma(current.max_current),(long)((float)hall_learn.learn_volt*VDC_RATED*1000.0f/(32767.0f*SQRT_3)));
  printf("pole_pairs=%u speed_maximum_rpm=%u can_node_id=%u compile_only=1\r\n",POLE_PAIRS,MAX_SPEED_RPM,MC_CAN_NODE_ID);
  printf("hall_angle=%u/%u/%u/%u/%u/%u\r\n",hall_learn_state_table[0],hall_learn_state_table[1],
    hall_learn_state_table[2],hall_learn_state_table[3],hall_learn_state_table[4],hall_learn_state_table[5]);
}
static int parameter_set(const char *name,int32_t value)
{
  unsigned i;
  for(i=0;i<sizeof(params)/sizeof(params[0]);i++) if(!strcmp(name,params[i].name)) {
    if(value<params[i].low || value>params[i].high) return 0;
    *params[i].value=(int16_t)value;
    integral_reset(&pid_id); integral_reset(&pid_iq); integral_reset(&pid_spd); return 1;
  }
  if(!strcmp(name,"demo_rs_mohm") && value>=1 && value<=10000) {
    motor_param_ident.Rs.f=value/1000.0f; motor_param_ident.Rs_Old.f=motor_param_ident.Rs.f; return 1;
  }
  if(!strcmp(name,"demo_ls_uh") && value>=1 && value<=100000) {
    motor_param_ident.Ls.f=value/1000000.0f; motor_param_ident.Ls_Old.f=motor_param_ident.Ls.f; return 1;
  }
  if(!strcmp(name,"demo_bandwidth_rad_s") && value>=1 && value<=I_auto_tune.BW_limit) {
    I_auto_tune.bandwidth=(uint16_t)value; return 1;
  }
  if(!strcmp(name,"demo_current_limit_ma") && value>=0 && value<=(int32_t)(MAX_CURRENT*1000.0f)) {
    limit_set(value); return 1;
  }
  if(!strcmp(name,"demo_tune_bus_mv") && value>=18000 && value<=60000) {
    I_auto_tune.Vdc_rated_adc=(uint16_t)((float)value*4095.0f/(VOLTAGE_BASE*1000.0f)); return 1;
  }
  if(!strcmp(name,"demo_hall_scan_mv") && value>=100 && value<=4800) {
    hall_learn.learn_volt=(int16_t)((float)value*32767.0f*SQRT_3/(VDC_RATED*1000.0f)); return 1;
  }
  return 0;
}
static int number(const char *s,int32_t *out)
{
  char *end; long v;
  errno=0; v=strtol(s,&end,10);
  if(errno || end==s || *end || v<(-2147483647L-1L) || v>2147483647L) return 0;
  *out=(int32_t)v; return 1;
}
static void trace_status(void)
{
  printf("OK logid=%u detail=%u rate=%u enabled=%u\r\n",trace_id,trace_detail,trace_rate,trace_enabled);
}
static int running_mode(motor_control_mode mode)
{
  return !mc_host_stop_requested() && esc_state==ESC_STATE_RUNNING && esc_state_old==esc_state && ctrl_mode==mode && !operation && error_code==MC_NO_ERROR;
}
static void command(char *line)
{
  char *arg[8], *p=line; unsigned n=0,i; int32_t value[5]={0}; uint32_t mask;
  while(*p) { while(*p==' ') p++; if(!*p) break; if(n==8) goto bad; arg[n++]=p;
    while(*p && *p!=' ') p++; if(*p) *p++=0; }
  if(!n) return;
#define IS(s) (!strcmp(arg[0],s))
#define AT(i,s) (n>(i) && !strcmp(arg[i],s))
  if(IS("version") && n==1) { printf("OK fw=demo-host-1 firmware=demo protocol=hall-host-1 parameter_mode=direct\r\n"); return; }
  if(IS("help") && n==1) { printf("OK commands: version status fault fault clear motor stop reset speed start|set current start|set open start|set motor params active motor param set identify winding calibrate hall angle calibrate current_offset calculate current_pi motor commissioning save|abort debug logid logdetail lograte logcapture\r\n"); return; }
  if((IS("status") || IS("fault")) && n==1) {
    printf("OK state=%u mode=%u error=0x%08lX speed_rpm=%ld iq_ma=%ld bus_mv=%ld rx_errors=%lu commissioning=%u\r\n",
      esc_state,ctrl_mode,(unsigned long)error_code,(long)rotor_speed_val,(long)ma(current.Iqd.q),
      (long)((float)adc_in_tab[ADC_BUS_VOLT_IDX]*VOLTAGE_BASE*1000.0f/4095.0f),(unsigned long)rx_errors,operation); return;
  }
  if((IS("motor") && AT(1,"stop") && n==2) ||
     ((IS("speed") || IS("current") || IS("open")) && AT(1,"stop") && n==2) ||
     (IS("motor") && AT(1,"commissioning") && AT(2,"abort") && n==3)) {
    stop(); printf("OK stopped\r\n"); return;
  }
  if(IS("fault") && AT(1,"clear") && n==2) {
    if(esc_state!=ESC_STATE_ERROR || start_stop_btn_flag || foc_rdy) goto unsafe;
    if(!gate_driver_fault_recover()) { printf("ERR driver_fault_active\r\n"); return; }
    mask=__get_PRIMASK(); __disable_irq();
    error_code=MC_NO_ERROR; esc_state=ESC_STATE_IDLE; start_stop_btn_flag=RESET;
    __set_PRIMASK(mask); printf("OK fault clear stopped\r\n"); return;
  }
  if(IS("reset") && n==1) { if(!stopped()) goto unsafe; printf("OK resetting\r\n");
    while(usart_flag_get(COMM_UART,USART_TDC_FLAG)==RESET) {} NVIC_SystemReset(); return; }
  if(IS("motor") && AT(1,"params") && AT(2,"active") && n==3) { parameters(); return; }
  if(IS("motor") && AT(1,"params") && AT(2,"storage") && n==3) {
    printf("OK storage=vendor_flash parameter_mode=direct save=explicit hall_learn_autosave=1\r\n"); return;
  }
  if(IS("motor") && AT(1,"param") && AT(2,"set") && n==5) {
    if(!number(arg[4],&value[0])) goto bad;
    mask=__get_PRIMASK(); __disable_irq();
    if(!stopped()) { __set_PRIMASK(mask); goto unsafe; }
    i=(unsigned)parameter_set(arg[3],value[0]); __set_PRIMASK(mask);
    if(!i) goto bad; printf("OK parameter applied name=%s value=%ld saved=0\r\n",arg[3],(long)value[0]); return;
  }
  if(IS("motor") && AT(1,"commissioning") && AT(2,"save") && n==3) {
    if(!stopped()) goto unsafe;
    write_flash_cmd();
    for(i=0;i<INT32_SIZE_ARRAY;i++) if(((volatile int32_t*)MC_VectStoreAddr_UINT32)[i]!=save_param_buffer[i]) {
      printf("ERR flash_verify_failed\r\n"); return;
    }
    printf("OK parameters saved vendor_flash bandwidth_not_persisted=1\r\n"); return;
  }
  if((IS("speed") || IS("current") || IS("open")) && (AT(1,"start") || AT(1,"set"))) {
    motor_control_mode mode=IS("speed")?SPEED_CTRL:(IS("current")?TORQUE_CTRL:OPEN_LOOP_CTRL);
    int16_t open_d=0,open_q=0;
    unsigned count=IS("speed")?1U:(IS("current")?2U:4U);
    if(n!=count+2U) goto bad;
    for(i=0;i<count;i++) if(!number(arg[i+2U],&value[i])) goto bad;
    if(mode==SPEED_CTRL && (value[0]>MAX_SPEED_RPM || value[0]<-MAX_SPEED_RPM)) goto bad;
    if(mode==TORQUE_CTRL && (value[0]>ma(current.max_current) || value[0]<-ma(current.max_current) ||
      value[1]>ma(current.max_current) || value[1]<-ma(current.max_current))) goto bad;
    if(mode==OPEN_LOOP_CTRL && (value[0]>4800 || value[0]<-4800 || value[1]>4800 || value[1]<-4800 ||
      value[2]>100000 || value[2]<-100000 || value[3]!=0)) {
      printf("ERR open_range_or_slew_unsupported use_accel_zero\r\n"); return;
    }
    mask=__get_PRIMASK(); __disable_irq();
    if(!stopped() && !running_mode(mode)) { __set_PRIMASK(mask); goto unsafe; }
    if(AT(1,"set") && !running_mode(mode)) { __set_PRIMASK(mask); goto unsafe; }
    if(mode==OPEN_LOOP_CTRL && !open_voltage_convert(value[0],value[1],&open_d,&open_q)) {
      __set_PRIMASK(mask); printf("ERR open_voltage_exceeds_bus_or_invalid_bus\r\n"); return;
    }
    mc_can_motor_serial_takeover();
    if(stopped()) { param_clear(); ctrl_mode=mode; ctrl_mode_cmd=mode; ctrl_mode_old=mode; ctrl_source=CTRL_SOURCE_SOFTWARE; }
    if(mode==SPEED_CTRL) speed_ramp.cmd_final=value[0];
    if(mode==TORQUE_CTRL) { current.Iqdref.d=pu_ma(value[0]); current.Iqdref.q=pu_ma(value[1]); }
    if(mode==OPEN_LOOP_CTRL) {
      openloop.volt.d=open_d;
      openloop.volt.q=open_q;
      openloop.inc=(int16_t)((int64_t)value[2]*32768/(1000*PWM_FREQ));
    }
    start_stop_btn_flag=SET; __set_PRIMASK(mask); printf("OK run requested mode=%s\r\n",arg[0]); return;
  }
  if((IS("identify") && AT(1,"winding") && n==2) ||
     (IS("calibrate") && AT(1,"hall") && AT(2,"angle") && n==3) ||
     (IS("calculate") && AT(1,"current_pi") && n==2)) {
    mask=__get_PRIMASK(); __disable_irq();
    if(!stopped()) { __set_PRIMASK(mask); goto unsafe; }
    trace_enabled=0; cap_armed=0; cap_count=0; cap_sent=0; live_tail=live_head;
    operation=IS("identify")?1U:(IS("calibrate")?2U:3U); operation_started=host_ms;
    if(operation==1) {
      winding_rs_backup=motor_param_ident.Rs.f; winding_ls_backup=motor_param_ident.Ls.f;
      motor_param_ident.state_flag=PROCESSING; esc_state=ESC_STATE_WINDING_PARAM_ID; }
    if(operation==2) {
      for(i=0;i<6;i++) hall_backup[i]=hall_learn_state_table[i];
      hall_direction_backup=hall_learn.dir; hall_learn.start_flag=SET;
    }
    if(operation==3) I_auto_tune.state_flag=PROCESSING;
    __set_PRIMASK(mask); printf("OK commissioning started\r\n"); return;
  }
  if(IS("calibrate") && AT(1,"current_offset") && n==2) {
    if(!stopped()) goto unsafe;
    if(!gate_driver_enable()) { printf("ERR commissioning rejected_or_failed driver_fault\r\n"); return; }
    enable_pwm_timer(&pwm_duty); curr_offset_rdy=I_offset_init(&current); gate_driver_disable();
    if(curr_offset_rdy) printf("OK commissioning complete current_offset\r\n");
    else printf("ERR commissioning rejected_or_failed current_offset\r\n"); return;
  }
  if((IS("debug") || IS("logid") || IS("logdetail") || IS("lograte") || IS("logcapture")) && n==2) {
    if(!number(arg[1],&value[0])) goto bad;
    if(IS("debug") && (value[0]==0 || (value[0]==1 && !operation))) {
      mask=__get_PRIMASK(); __disable_irq(); trace_enabled=0; cap_armed=0; cap_count=0; cap_sent=0;
      live_head=live_tail=0; trace_phase=0; trace_seq=0; trace_drops=0; trace_enabled=(uint8_t)value[0]; __set_PRIMASK(mask);
      if(!value[0]) printf("OK debug 0\r\n"); else trace_status(); return;
    }
    if(trace_enabled || cap_armed || cap_count) { printf("ERR stop_debug_first\r\n"); return; }
    if(IS("logid") && (value[0]==10 || value[0]==20 || value[0]==30 || value[0]==40)) { mask=__get_PRIMASK(); __disable_irq(); trace_id=(uint16_t)value[0]; trace_detail=0; position_sum=0; position_ready=0; __set_PRIMASK(mask); trace_status(); return; }
    if(IS("logdetail") && value[0]>=0 && value[0]<=2 && trace_id==10) { trace_detail=(uint16_t)value[0]; trace_status(); return; }
    if(IS("lograte") && value[0]>=10 && value[0]<=500) { trace_rate=(uint16_t)value[0]; trace_status(); return; }
    if(IS("logcapture") && value[0]==128 && trace_id==10 && !operation) {
      cap_sent=0; cap_count=0; trace_seq=0; trace_drops=0; cap_armed=1;
      printf("OK capture armed count=128\r\n"); return;
    }
  }
bad: printf("ERR unsupported_or_invalid_command\r\n"); return;
unsafe: printf("ERR commissioning rejected_or_failed reason=state_requires_stopped_or_same_run_mode\r\n");
#undef IS
#undef AT
}
static int32_t pterm(pid_ctrl_type *p) { return (int32_t)(((int64_t)p->kp_gain*p->error)>>p->kp_shift); }
static void fill(host_frame *f)
{
  pid_ctrl_type *p=trace_detail==1?&pid_id:&pid_iq;
  unsigned i; int32_t d; int64_t position_mdeg;
  for(i=0;i<10;i++) f->v[i]=0;
  f->seq=trace_seq++; f->us=sample_us; f->drops=trace_drops;
  f->flags=(foc_rdy?4U:0U) | (curr_offset_rdy?8U:0U);
  if(foc_rdy && esc_state==ESC_STATE_RUNNING && (ctrl_mode==SPEED_CTRL || ctrl_mode==TORQUE_CTRL)) f->flags|=1U|256U;
  if(adc_in_tab[ADC_BUS_VOLT_IDX]) f->flags|=16U;
  if(hall.state>0 && hall.state<7 && !(error_code & MC_HALL_ERROR)) f->flags|=32U|128U;
  if(current.Iqdref.q>=current.max_current || current.Iqdref.q<=current.min_current) f->flags|=64U;
  if(trace_id==10 && trace_detail) {
    d=trace_detail==1?current.Iqdref.d:current.Iqdref.q;
    f->v[0]=ma(d); f->v[1]=ma(trace_detail==1?current.Iqd.d:current.Iqd.q);
    f->v[2]=f->v[0]-f->v[1]; f->v[3]=mv(pterm(p)); f->v[4]=mv(p->integral>>p->ki_shift);
    f->v[5]=mv(p->out_start); f->v[6]=mv(trace_detail==1?volt_cmd.Vqd.d:volt_cmd.Vqd.q);
    f->v[7]=f->v[3]+f->v[4];
  } else if(trace_id==10) {
    f->v[0]=ma(current.Iqdref.d); f->v[1]=ma(current.Iqd.d); f->v[2]=ma(current.Iqdref.q);
    f->v[3]=ma(current.Iqd.q); f->v[4]=mv(volt_cmd.Vqd.d); f->v[5]=mv(volt_cmd.Vqd.q);
  } else if(trace_id==20) {
    f->v[0]=speed_ramp.cmd_final*1000; f->v[1]=speed_ramp.command*1000; f->v[2]=rotor_speed_val*1000;
    f->v[3]=ma(current.Iqdref.q); f->v[4]=ma(current.Iqd.q); f->v[5]=ma(pterm(&pid_spd));
    f->v[6]=ma(pid_spd.integral>>pid_spd.ki_shift); f->v[7]=ma(pid_spd.out_start); f->v[8]=ma(current.max_current);
  } else if(trace_id==30) {
    f->v[0]=(int32_t)pwm_duty.UF.a*10000/(2*pwm_duty.half_duty);
    f->v[1]=(int32_t)pwm_duty.UF.b*10000/(2*pwm_duty.half_duty);
    f->v[2]=(int32_t)pwm_duty.UF.c*10000/(2*pwm_duty.half_duty);
    f->v[3]=ma(current.Iabc.a); f->v[4]=ma(current.Iabc.b); f->v[5]=ma(current.Iabc.c);
    f->v[6]=(int32_t)((float)adc_in_tab[ADC_BUS_VOLT_IDX]*VOLTAGE_BASE*1000.0f/4095.0f);
  } else {
    f->v[0]=ma(current.Iqd.q); f->v[1]=rotor_speed_val*1000;
    position_mdeg=position_sum*360000/(32768*POLE_PAIRS);
    if(position_mdeg>2147483647LL || position_mdeg<(-2147483647LL-1)) {
      f->flags&=~128U; f->v[2]=0;
    } else f->v[2]=(int32_t)position_mdeg;
    f->v[3]=ma(current.Iqdref.q); f->v[4]=speed_ramp.command*1000;
  }
}
void mc_host_sample(void)
{
  int16_t angle=(int16_t)(rotor_angle_hall.elec_angle_val & 32767); int32_t delta;
  uint16_t next;
  sample_cycles+=(PWM_ADVANCE_TIMER->pr+1U)*2U;
  sample_us+=sample_cycles/(TMR_CLK/1000000U);
  sample_cycles%=TMR_CLK/1000000U;
  if(!position_ready) { position_previous=angle; position_ready=1; }
  delta=(int32_t)angle-position_previous;
  if(delta>16384) delta-=32768; else if(delta< -16384) delta+=32768;
  if(hall.state>0 && hall.state<7) position_sum+=delta;
  position_previous=angle;
  if(cap_armed) {
    if(cap_count<CAP_SIZE) { fill(&capture[cap_count]); cap_count++; }
    if(cap_count==CAP_SIZE) cap_armed=0;
  } else if(trace_enabled) {
    trace_phase+=trace_rate;
    if(trace_phase<PWM_FREQ) return;
    trace_phase-=PWM_FREQ; next=(uint16_t)((live_head+1U)%LIVE_SIZE);
    if(next==live_tail) { trace_drops++; trace_seq++; return; }
    fill(&live[live_head]); live_head=next;
  }
}
static void emit(host_frame *f,unsigned captured)
{
  unsigned i;
  printf("@T,%u,%u,%lu,%lu,%lu,%lu,%u",trace_id,trace_detail,(unsigned long)f->seq,
    (unsigned long)f->us,(unsigned long)f->drops,(unsigned long)f->flags,captured);
  for(i=0;i<10;i++) printf(",%ld",(long)f->v[i]); printf("\r\n");
}
void mc_host_task(void)
{
  char line[RX_SIZE]; uint32_t mask; host_frame f; unsigned done=0,failed=0;
  if(rx_stop_pending) {
    mask=__get_PRIMASK(); __disable_irq(); rx_stop_pending=0; rx_tail=rx_head; __set_PRIMASK(mask);
    stop(); printf("OK stopped\r\n");
  }
  if(rx_tail!=rx_head) {
    mask=__get_PRIMASK(); __disable_irq(); strcpy(line,rx_queue[rx_tail]); rx_tail=(uint8_t)((rx_tail+1U)%RX_QUEUE); __set_PRIMASK(mask);
    command(line);
  }
  if(operation) {
    if(operation==1) { done=motor_param_ident.state_flag==SUCCEED; failed=motor_param_ident.state_flag==FAILED; }
    if(operation==2) { done=hall_learn.process_state==PROCESS_4_FINISH && !hall_learn.start_flag;
      failed=hall_learn.process_state==PROCESS_5_ERROR && !hall_learn.start_flag; }
    if(operation==3) { done=I_auto_tune.state_flag==SUCCEED; failed=I_auto_tune.state_flag==FAILED; }
    if(error_code!=MC_NO_ERROR) failed=1;
    if((uint32_t)(host_ms-operation_started)>30000U) failed=1;
    if(failed) {
      /* All failures use the same hardware/model rollback as an explicit abort. */
      stop();
      printf("ERR commissioning rejected_or_failed\r\n");
    } else if(done) {
      operation=0;
      printf("OK commissioning complete applied=1\r\n");
    }
  }
  if(!cap_armed && cap_count==CAP_SIZE) {
    emit(&capture[cap_sent++],1);
    if(cap_sent==CAP_SIZE) { cap_count=0; cap_sent=0; printf("OK capture complete\r\n"); }
  } else if(live_tail!=live_head) {
    mask=__get_PRIMASK(); __disable_irq(); f=live[live_tail]; live_tail=(uint16_t)((live_tail+1U)%LIVE_SIZE); __set_PRIMASK(mask);
    emit(&f,0);
  }
}
#endif
