#include <string.h>
#include "../project/foc/foc_app/motor_speed_feedback.c"
static motor_rotor_feedback_t rotor;
static motor_hall_sample_t hall;
static uint32_t now;
uint8_t motor_hall_port_raw_state_get(void) { return hall.state; }
uint32_t motor_hall_port_quiet_us(void) { return now-hall.timestamp_cycles; }
bool motor_rotor_feedback_read(motor_rotor_feedback_t *p) { *p=rotor; return true; }
bool motor_hall_decoder_sample_read(motor_hall_sample_t *p) { *p=hall; return true; }
uint32_t motor_timebase_cycles_get(void) { return now; }
uint32_t motor_timebase_cycles_elapsed(uint32_t start) { return now-start; }
uint32_t motor_timebase_cycles_to_us(uint32_t cycles) { return cycles; }
bool motor_parameter_direction_inverted_get(void) { return false; }
#define CHECK(x) do { if(!(x)) return __LINE__; } while(0)
int main(void) {
  motor_speed_feedback_t s;
  motor_speed_feedback_init();
  rotor.valid=true; rotor.speed_valid=true; rotor.direction=1;
  rotor.mechanical_speed_millirpm=100000; rotor.update_count=1;
  motor_speed_feedback_process_1khz();
  CHECK(motor_speed_feedback_read(&s) && s.valid && s.filtered_speed_millirpm==100000);
  rotor.speed_valid=false; rotor.direction=0; rotor.mechanical_speed_millirpm=0;
  motor_speed_feedback_process_1khz();
  CHECK(motor_speed_feedback_read(&s) && !s.valid && s.filtered_speed_millirpm==0);
  hall.valid=true; hall.state=1;
  now=999999; CHECK(!motor_speed_feedback_standstill_confirmed());
  now=1000000; CHECK(motor_speed_feedback_standstill_confirmed());
  hall.valid=false; CHECK(!motor_speed_feedback_standstill_confirmed());
  motor_speed_feedback_init();
  CHECK(motor_speed_feedback_read(&s) && !s.valid && s.direction==0);
  return 0;
}
