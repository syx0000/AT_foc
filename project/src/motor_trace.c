/* Grouped motor telemetry; ISR sampling and main-loop UART transport. */
#include <stdio.h>
#include <string.h>
#include "at32f45x.h"
#include "motor_trace.h"
#include "motor_current_control.h"
#include "motor_current_sample.h"
#include "motor_speed_control.h"
#include "motor_speed_feedback.h"
#include "motor_hall_decoder.h"
#include "motor_parameter.h"
#include "motor_slow_sensor.h"
#include "motor_pwm_port.h"
#include "motor_board_config.h"
#include "motor_control_config.h"
#include "motor_timebase.h"
#include "motor_uart_port.h"

#define TRACE_RING 8U
#define TRACE_CAPTURE 128U
#define TRACE_VALUES 10U
typedef struct {
  uint32_t seq, us, flags;
  int32_t v[TRACE_VALUES];
} trace_frame_t;
static trace_frame_t trace_ring[TRACE_RING], trace_capture[TRACE_CAPTURE];
static volatile uint32_t trace_head, trace_tail, trace_drops, trace_seq;
static volatile uint32_t trace_capture_count, trace_capture_target;
static uint32_t trace_capture_sent;
static volatile bool trace_enabled;
static volatile uint32_t trace_id=20U, trace_rate=200U, trace_detail;
static uint32_t trace_ticks, trace_last_current_count;
static uint32_t trace_pos_positive, trace_pos_negative, trace_pos_errors;
static int64_t trace_position_edges;
static bool trace_pos_initialized, trace_pos_valid;
static uint32_t trace_pos_poles;

/* CVP position origin is the first sample after selecting the group.
 * Hall sector resolution, not an encoder or an absolute shaft position. */
static void trace_position(const motor_hall_sample_t *h, uint32_t poles)
{
  if (!trace_pos_initialized) {
    trace_pos_initialized=true; trace_position_edges=0;
    trace_pos_positive=h->positive_count; trace_pos_negative=h->negative_count;
    trace_pos_errors=h->invalid_transition_count; trace_pos_poles=poles;
    trace_pos_valid=h->valid && !h->health_fault && poles!=0;
  }
  if (poles!=trace_pos_poles || h->invalid_transition_count!=trace_pos_errors ||
      h->health_fault || !h->valid) trace_pos_valid=false;
  /* modulo counters also handle uint32 wrap; a decoder reset invalidates P */
  if ((int32_t)(h->positive_count-trace_pos_positive)<0 ||
      (int32_t)(h->negative_count-trace_pos_negative)<0) trace_pos_valid=false;
  trace_position_edges+=(int32_t)(h->positive_count-trace_pos_positive);
  trace_position_edges-=(int32_t)(h->negative_count-trace_pos_negative);
  trace_pos_positive=h->positive_count; trace_pos_negative=h->negative_count;
}

void motor_trace_fast_sample(void)
{
  trace_frame_t f;
  motor_current_control_status_t c;
  motor_speed_control_status_t s;
  motor_speed_feedback_t speed;
  motor_current_sample_state_t phase;
  motor_slow_sensor_state_t slow;
  motor_hall_sample_t hall;
  uint32_t poles, flags=0;
  int32_t sign;
  bool capture=trace_capture_target!=0U;
  if (!trace_enabled && !capture) return;
  if (!capture) {
    trace_ticks+=trace_rate;
    if (trace_ticks<MOTOR_PWM_FREQUENCY_HZ) return;
    trace_ticks-=MOTOR_PWM_FREQUENCY_HZ;
  }
  memset(&f,0,sizeof(f));
  f.us=motor_timebase_us_get(); f.seq=trace_seq++;
  (void)motor_current_control_status_read(&c);
  if (c.state==MOTOR_CURRENT_CONTROL_RUNNING) flags|=1U;
  if (c.sample_count!=trace_last_current_count) flags|=256U;
  trace_last_current_count=c.sample_count;
  if (c.state==MOTOR_CURRENT_CONTROL_RUNNING && c.voltage_limited) flags|=2U;
  if (motor_pwm_port_output_is_enabled()) flags|=4U;
  if (trace_id==10U) {
    if (trace_detail==0U) {
      f.v[0]=c.command.direct_reference_ma; f.v[1]=c.direct_feedback_ma;
      f.v[2]=c.command.quadrature_reference_ma; f.v[3]=c.quadrature_feedback_ma;
      f.v[4]=c.direct_voltage_mv; f.v[5]=c.quadrature_voltage_mv;
    } else if (trace_detail==1U) {
      f.v[0]=c.command.direct_reference_ma; f.v[1]=c.direct_feedback_ma;
      f.v[2]=f.v[0]-f.v[1]; f.v[3]=c.direct_p_mv; f.v[4]=c.direct_i_mv;
      f.v[5]=c.direct_request_mv; f.v[6]=c.direct_voltage_mv;
    } else {
      f.v[0]=c.command.quadrature_reference_ma; f.v[1]=c.quadrature_feedback_ma;
      f.v[2]=f.v[0]-f.v[1]; f.v[3]=c.quadrature_p_mv; f.v[4]=c.quadrature_i_mv;
      f.v[5]=c.quadrature_request_mv; f.v[6]=c.quadrature_voltage_mv;
    }
    if (trace_detail!=0U) f.v[7]=f.v[3]+f.v[4];
    f.v[9]=(int32_t)c.sample_count;
  } else if (trace_id==30U) {
    if (motor_current_sample_state_read(&phase)) flags|=8U;
    else memset(&phase,0,sizeof(phase));
    /* Compare preload written by this control cycle, applied next PWM update.
     * Current belongs to the ADC sample preceding that update. */
    f.v[0]=(int32_t)(MOTOR_PWM_TIMER->c1dt*10000U/MOTOR_PWM_PERIOD_COUNTS);
    f.v[1]=(int32_t)(MOTOR_PWM_TIMER->c2dt*10000U/MOTOR_PWM_PERIOD_COUNTS);
    f.v[2]=(int32_t)(MOTOR_PWM_TIMER->c3dt*10000U/MOTOR_PWM_PERIOD_COUNTS);
    f.v[3]=phase.phase_a_ma; f.v[4]=phase.phase_b_ma; f.v[5]=phase.phase_c_ma;
    if (motor_slow_sensor_state_read(&slow)) {f.v[6]=slow.bus_voltage_0p1v*100; flags|=16U;}
    f.v[9]=(int32_t)phase.sample_count;
  } else {
    (void)motor_speed_control_status_read(&s);
    if (!motor_speed_feedback_read(&speed)) memset(&speed,0,sizeof(speed));
    if (speed.valid) flags|=32U;
    if (s.state==MOTOR_SPEED_CONTROL_RUNNING && s.current_limited) flags|=64U;
    sign=motor_parameter_direction_inverted_get() ? -1 : 1;
    if (trace_id==20U) {
      f.v[0]=s.target_speed_millirpm; f.v[1]=s.ramped_speed_millirpm;
      f.v[2]=speed.filtered_speed_millirpm; f.v[3]=s.quadrature_current_command_ma;
      f.v[4]=c.quadrature_feedback_ma*sign;
      f.v[5]=s.proportional_ma; f.v[6]=s.integral_ma;
      f.v[7]=s.pi_output_ma; f.v[8]=s.current_limit_ma;
      f.v[9]=(int32_t)s.update_count;
    } else {
      f.v[0]=c.quadrature_feedback_ma*sign; f.v[1]=speed.filtered_speed_millirpm;
      poles=motor_parameter_pole_pairs_get();
      if (motor_hall_decoder_sample_read(&hall)) {
        trace_position(&hall,poles);
        if (poles) {
          int64_t position=trace_position_edges*60000/(int64_t)poles*sign;
          if (position>INT32_MAX || position<INT32_MIN) trace_pos_valid=false;
          else f.v[2]=(int32_t)position;
        }
        if (trace_pos_valid) flags|=128U;
      }
      f.v[3]=s.quadrature_current_command_ma; f.v[4]=s.ramped_speed_millirpm;
      f.v[9]=(int32_t)c.sample_count;
    }
  }
  f.flags=flags;
  if (capture) {
    trace_capture[trace_capture_count]=f;
    trace_capture_count++;
    if (trace_capture_count>=trace_capture_target) trace_capture_target=0;
  } else if (trace_head-trace_tail<TRACE_RING) {
    trace_ring[trace_head%TRACE_RING]=f;
    __DMB(); trace_head++;
  } else trace_drops++;
}

void motor_trace_poll(void)
{
  trace_frame_t f;
  char line[320];
  int n, i;
  bool capture=trace_capture_count!=0U && trace_capture_target==0U;
  if (trace_capture_target) return;
  if (!motor_uart_port_trace_ready()) return;
  if (capture) {
    if (trace_capture_sent>=trace_capture_count) {
      trace_capture_count=0; trace_capture_sent=0;
      printf("OK capture complete\r\n"); return;
    }
    f=trace_capture[trace_capture_sent];
  } else {
    if (trace_tail==trace_head) return;
    f=trace_ring[trace_tail%TRACE_RING];
  }
  n=snprintf(line,sizeof(line),"@T,%lu,%lu,%lu,%lu,%lu,%lu,%u",
    (unsigned long)trace_id,(unsigned long)trace_detail,(unsigned long)f.seq,
    (unsigned long)f.us,(unsigned long)trace_drops,(unsigned long)f.flags,capture?1U:0U);
  for (i=0;i<TRACE_VALUES;i++) n+=snprintf(line+n,sizeof(line)-(unsigned)n,",%ld",(long)f.v[i]);
  n+=snprintf(line+n,sizeof(line)-(unsigned)n,"\r\n");
  if (motor_uart_port_trace_write(line,(uint16_t)n)) {
    if (capture) trace_capture_sent++; else trace_tail++;
  }
}

static bool trace_number(const char *s, uint32_t *v)
{
  uint32_t n=0;
  if (*s<'0' || *s>'9') return false;
  while (*s) {
    if (*s<'0' || *s>'9') return false;
    n=n*10U+(uint32_t)(*s++-'0');
    if (n>16000U) return false;
  }
  *v=n; return true;
}
static void trace_clear(void)
{
  trace_enabled=false; trace_capture_target=0; __DMB();
  trace_head=trace_tail=trace_seq=trace_drops=trace_ticks=0;
  trace_capture_count=trace_capture_sent=0;
}
static void trace_status(void)
{
  printf("OK logid=%lu detail=%lu rate=%lu enabled=%u drops=%lu capture=%lu/%lu format=@T,id,detail,seq,us,drops,flags,capture,v0..v9\r\n",
    (unsigned long)trace_id,(unsigned long)trace_detail,(unsigned long)trace_rate,
    trace_enabled?1U:0U,(unsigned long)trace_drops,(unsigned long)trace_capture_count,
    (unsigned long)trace_capture_target);
  if (trace_id==10U && trace_detail==0U)
    printf("TRACE fields=id_ref_ma,id_ma,iq_ref_ma,iq_ma,ud_mv,uq_mv,reserved,reserved,reserved,current_sample_count\r\n");
  else if (trace_id==10U)
    printf("TRACE fields=ref_ma,feedback_ma,error_ma,p_mv,i_mv,axis_limited_mv,vector_limited_mv,raw_pi_mv,reserved,current_sample_count\r\n");
  else if (trace_id==20U)
    printf("TRACE fields=target_mrpm,ramp_mrpm,feedback_mrpm,iq_cmd_ma,iq_feedback_ma,p_ma,i_ma,pi_limited_ma,limit_ma,speed_update_count\r\n");
  else if (trace_id==30U)
    printf("TRACE fields=duty_u_0p01pct,duty_v_0p01pct,duty_w_0p01pct,ia_ma,ib_ma,ic_reconstructed_ma,vbus_mv,reserved,reserved,current_sample_count\r\n");
  else
    printf("TRACE fields=logical_iq_ma,logical_speed_mrpm,relative_position_mdeg,logical_iq_cmd_ma,ramp_mrpm,reserved,reserved,reserved,reserved,current_sample_count\r\n");
}
bool motor_trace_command(const char *line)
{
  uint32_t n;
  if (strcmp(line,"logid")==0) {trace_status();return true;}
  if (strcmp(line,"debug 0")==0) {trace_clear();motor_uart_port_trace_flush();printf("OK debug 0\r\n");return true;}
  if (strcmp(line,"debug 1")==0) {trace_clear();trace_enabled=true;trace_status();return true;}
  if (strncmp(line,"logid ",6)==0) {
    if (!trace_number(line+6,&n) || (n!=10 && n!=20 && n!=30 && n!=40)) goto invalid;
    trace_clear(); trace_id=n; trace_detail=0;
    trace_rate=(n==10 || n==30)?500U:200U;
    trace_pos_initialized=false; trace_status(); return true;
  }
  if (strncmp(line,"lograte ",8)==0) {
    if (!trace_number(line+8,&n) || n<1 || n>500) goto invalid;
    trace_clear(); trace_rate=n; trace_status(); return true;
  }
  if (strncmp(line,"logdetail ",10)==0) {
    if (!trace_number(line+10,&n) || n>2 || trace_id!=10) goto invalid;
    trace_clear(); trace_detail=n; trace_status(); return true;
  }
  if (strcmp(line,"logcapture")==0 || strncmp(line,"logcapture ",11)==0) {
    n=TRACE_CAPTURE;
    if (line[10] && !trace_number(line+11,&n)) goto invalid;
    if (trace_id!=10 || n<1 || n>TRACE_CAPTURE) goto invalid;
    trace_clear();
    printf("OK capture armed samples=%lu rate=%u\r\n",(unsigned long)n,MOTOR_PWM_FREQUENCY_HZ);
    trace_capture_target=n; return true;
  }
  return false;
invalid:
  printf("ERR invalid_trace_command\r\n");return true;
}
