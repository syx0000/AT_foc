#include <string.h>
#include "../project/foc/foc_app/motor_rotor_feedback.c"
static motor_hall_sample_t h;
static motor_hall_angle_estimator_t e;
static motor_parameter_t p;
static uint8_t raw;
bool motor_hall_angle_estimator_read(motor_hall_angle_estimator_t *out) { *out=e; return true; }
bool motor_hall_decoder_sample_read(motor_hall_sample_t *out) { *out=h; return true; }
bool motor_parameter_active_read(motor_parameter_t *out) { *out=p; return true; }
uint8_t motor_parameter_pole_pairs_get(void) { return 4; }
uint8_t motor_hall_port_raw_state_get(void) { return raw; }
uint32_t motor_timebase_cycles_get(void) { return 0; }
#define CHECK(x) do { if(!(x)) return __LINE__; } while(0)
int main(void) {
  static const uint8_t next[8]={0,5,3,1,6,4,2,0};
  static const uint16_t a[8]={0,58890,38627,48251,14958,6537,26968,0};
  motor_rotor_feedback_t f;
  memcpy(p.hall_positive_next,next,sizeof(next));
  memcpy(p.hall_entry_angle_u16,a,sizeof(a));
  motor_rotor_feedback_init();
  h.valid=true; h.state=1; raw=1;
  CHECK(motor_rotor_feedback_read(&f) && f.valid && !f.speed_valid);
  raw=0; CHECK(motor_rotor_feedback_read(&f) && !f.valid);
  raw=7; CHECK(motor_rotor_feedback_read(&f) && !f.valid);
  raw=5; CHECK(motor_rotor_feedback_read(&f) && !f.valid);
  raw=1; h.health_fault=true; CHECK(motor_rotor_feedback_read(&f) && !f.valid);
  h.health_fault=false; e.valid=true; e.direction=1; e.electrical_frequency_millihz=10000;
  CHECK(motor_rotor_feedback_read(&f) && f.valid && f.speed_valid);
  return 0;
}
