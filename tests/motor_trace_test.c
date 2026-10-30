/* Exercise the actual trace sampler with deterministic peripheral stand-ins. */
#include <stddef.h>
#include <stdint.h>
#define __AT32F45x_WK_CONFIG_H
#define __DMB() ((void)0)
static struct {uint32_t c1dt,c2dt,c3dt;} fake_timer;
#define TMR1 (&fake_timer)
int strcmp(const char *,const char *);
int strncmp(const char *,const char *,size_t);
int snprintf(char *,size_t,const char *,...);
#include "../project/src/motor_trace.c"

static motor_current_control_status_t current;
static motor_speed_control_status_t speed_control;
static motor_hall_sample_t hall={0};
static bool reverse, tx_busy;
static uint32_t now, sent;
int strcmp(const char *a,const char *b) {while(*a && *a==*b){a++;b++;}return (unsigned char)*a-(unsigned char)*b;}
int strncmp(const char *a,const char *b,size_t n) {while(n && *a && *a==*b){a++;b++;n--;}return n?(unsigned char)*a-(unsigned char)*b:0;}
unsigned long strtoul(const char *s,char **end,int base) {unsigned long n=0;(void)base;while(*s>='0' && *s<='9')n=n*10+(*s++-'0');*end=(char *)s;return n;}
int snprintf(char *s,size_t n,const char *fmt,...) {(void)fmt;if(n>1){s[0]='x';s[1]=0;}return 1;}
uint32_t motor_timebase_us_get(void){return now++;}
bool motor_current_control_status_read(motor_current_control_status_t *s){*s=current;return true;}
bool motor_speed_control_status_read(motor_speed_control_status_t *s){*s=speed_control;return true;}
bool motor_speed_feedback_read(motor_speed_feedback_t *s){memset(s,0,sizeof(*s));s->valid=true;s->filtered_speed_millirpm=-123000;return true;}
bool motor_current_sample_state_read(motor_current_sample_state_t *s){memset(s,0,sizeof(*s));s->phase_a_ma=1000;s->phase_b_ma=-250;s->phase_c_ma=-750;return true;}
bool motor_slow_sensor_state_read(motor_slow_sensor_state_t *s){memset(s,0,sizeof(*s));s->bus_voltage_0p1v=240;return true;}
bool motor_hall_decoder_sample_read(motor_hall_sample_t *s){*s=hall;return true;}
bool motor_parameter_direction_inverted_get(void){return reverse;}
uint8_t motor_parameter_pole_pairs_get(void){return 4;}
bool motor_pwm_port_output_is_enabled(void){return true;}
bool motor_uart_port_trace_write(const char *s,uint16_t n){(void)s;(void)n;if(tx_busy)return false;sent++;return true;}
bool motor_uart_port_trace_ready(void){return !tx_busy;}
void motor_uart_port_trace_flush(void){}
#define CHECK(x) do {if(!(x))return __LINE__;}while(0)
static void tick(uint32_t n){while(n--){current.sample_count++;motor_trace_fast_sample();}}
int main(void)
{
  uint32_t i;
  current.state=MOTOR_CURRENT_CONTROL_RUNNING;
  current.command.direct_reference_ma=100;
  current.command.quadrature_reference_ma=2000;
  current.quadrature_feedback_ma=1900;
  current.quadrature_p_mv=300;current.quadrature_i_mv=150;
  current.quadrature_request_mv=450;current.quadrature_voltage_mv=400;
  CHECK(motor_trace_command("logid 10"));
  CHECK(trace_id==10 && trace_rate==500);
  CHECK(motor_trace_command("debug 1"));
  tick(31);CHECK(trace_head==0);tick(1);CHECK(trace_head==1);
  CHECK(trace_ring[0].v[2]==2000 && trace_ring[0].v[3]==1900);
  CHECK((trace_ring[0].flags & 261)==261);
  tx_busy=true;motor_trace_poll();CHECK(trace_tail==0);
  tx_busy=false;motor_trace_poll();CHECK(trace_tail==1 && sent==1);
  tick(32*10);CHECK(trace_head-trace_tail==TRACE_RING && trace_drops==2);
  CHECK(motor_trace_command("debug 0"));CHECK(!trace_enabled && trace_head==0);
  CHECK(motor_trace_command("lograte 0"));CHECK(trace_rate==500);
  CHECK(motor_trace_command("logid 99"));CHECK(trace_id==10);
  CHECK(motor_trace_command("lograte 500x"));CHECK(trace_rate==500);
  CHECK(!motor_trace_command("speed stop"));
  CHECK(motor_trace_command("logdetail 2"));
  CHECK(motor_trace_command("logcapture 128"));
  tick(128);CHECK(trace_capture_count==128 && !trace_capture_target);
  CHECK(trace_capture[0].v[3]==300 && trace_capture[0].v[4]==150);
  CHECK(trace_capture[0].v[5]==450 && trace_capture[0].v[6]==400);
  tick(100);CHECK(trace_capture_count==128 && !trace_enabled);
  for(now=0;now<129;now++)motor_trace_poll();
  CHECK(!trace_capture_count && sent==129);
  CHECK(motor_trace_command("logid 30"));
  fake_timer.c1dt=3000;fake_timer.c2dt=1500;fake_timer.c3dt=4500;
  motor_trace_command("debug 1");tick(32);
  CHECK(trace_ring[0].v[0]==5000 && trace_ring[0].v[2]==7500);
  CHECK(trace_ring[0].v[5]==-750 && trace_ring[0].v[6]==24000);
  motor_trace_command("logid 40");reverse=true;hall.valid=true;
  hall.positive_count=100;motor_trace_command("debug 1");tick(80);
  CHECK(trace_ring[0].v[0]==-1900 && trace_ring[0].v[2]==0);
  CHECK(trace_ring[0].flags & 128);
  hall.positive_count++;tick(80);
  CHECK(trace_ring[1].v[2]==-15000);
  hall.invalid_transition_count++;tick(80);CHECK(!(trace_ring[2].flags & 128));
  motor_trace_command("logid 20");
  motor_trace_command("lograte 333");motor_trace_command("debug 1");
  for(i=0;i<16000;i++){tick(1);motor_trace_poll();}
  CHECK(trace_seq==333 && trace_drops==0);
  return 0;
}
