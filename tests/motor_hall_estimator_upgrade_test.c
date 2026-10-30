#include "../project/foc/foc_app/motor_hall_angle_estimator.c"
uint32_t system_core_clock=192000000U;
static uint32_t now;
static motor_parameter_t p;
static motor_hall_sample_t h;
bool motor_parameter_active_read(motor_parameter_t *out) { *out=p; return true; }
bool motor_hall_decoder_sample_read(motor_hall_sample_t *out) { *out=h; return true; }
uint32_t motor_timebase_cycles_get(void) { return now; }
uint32_t motor_timebase_cycles_elapsed(uint32_t start) { return now-start; }
#define CHECK(x) do { if(!(x)) return __LINE__; } while(0)
int main(void) {
  static const uint8_t next[8]={0,5,3,1,6,4,2,0};
  static const uint16_t angles[8]={0,58890,38627,48251,14958,6537,26968,0};
  uint16_t boundary;
  memcpy(p.hall_positive_next,next,sizeof(next));
  memcpy(p.hall_entry_angle_u16,angles,sizeof(angles));
  p.hall_rotor_offset_u16=448;
  motor_hall_angle_estimator_init();
  h.valid=true; h.direction=1; h.state=5; h.previous_state=1;
  h.electrical_frequency_millihz=10000; h.timestamp_cycles=1;
  motor_hall_angle_estimator_edge_process();
  CHECK(motor_hall_angle_estimator.valid && motor_hall_correction_remaining==0);
  boundary=(uint16_t)(angles[4]+448);
  h.state=4; h.previous_state=5; h.timestamp_cycles=2;
  motor_hall_phase_accumulator=(uint32_t)(uint16_t)(boundary-1000)<<16;
  motor_hall_angle_estimator_edge_process();
  CHECK(motor_hall_correction_remaining==1000);
  CHECK((uint16_t)(motor_hall_phase_accumulator>>16)==(uint16_t)(boundary-1000));
  now=100;
  motor_hall_angle_estimator_fast_process();
  CHECK(motor_hall_correction_remaining==980);
  h.timestamp_cycles=3;
  motor_hall_phase_accumulator=(uint32_t)(uint16_t)(boundary-10000)<<16;
  motor_hall_angle_estimator_edge_process();
  CHECK(!motor_hall_correction_remaining && (uint16_t)(motor_hall_phase_accumulator>>16)==boundary);
  h.direction=-1; h.timestamp_cycles=4;
  motor_hall_angle_estimator_edge_process();
  CHECK((uint16_t)(motor_hall_phase_accumulator>>16)==(uint16_t)(angles[next[4]]+448));
  CHECK(motor_hall_phase_step<0 && !motor_hall_correction_remaining);
  h.timestamp_cycles=5; h.electrical_frequency_millihz=9000;
  motor_hall_angle_estimator_edge_process(); CHECK(!motor_hall_correction_remaining);
  motor_hall_angle_estimator_motion_reset();
  CHECK(!motor_hall_angle_estimator.valid && !motor_hall_correction_remaining);
  return 0;
}
