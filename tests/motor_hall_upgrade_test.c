#include "motor_hall_decoder.h"
#include "motor_hall_angle_math.h"
#define CHECK(x) do { if(!(x)) return __LINE__; } while(0)
static const uint8_t next[8]={0,5,3,1,6,4,2,0};
static const uint16_t entry[8]={0,58890,38627,48251,14958,6537,26968,0};
int main(void) {
  motor_hall_sample_t s;
  uint16_t f[8],r[8],a;
  CHECK(motor_hall_decoder_init(next,4000000,1,0));
  CHECK(motor_hall_decoder_edge_process_captured(5,1,999999,1000));
  CHECK(motor_hall_decoder_sample_read(&s) && s.electrical_frequency_millihz==0);
  CHECK(motor_hall_decoder_edge_process_captured(4,2,777777,2000));
  CHECK(motor_hall_decoder_sample_read(&s) && s.interval_cycles==4000 && s.electrical_frequency_millihz==166666);
  CHECK(motor_hall_decoder_edge_process_captured(6,3,1,4000));
  CHECK(motor_hall_decoder_sample_read(&s) && s.interval_cycles==8000 && s.electrical_frequency_millihz==111111);
  CHECK(motor_hall_decoder_edge_process_captured(4,4,2,5000));
  CHECK(motor_hall_decoder_sample_read(&s) && s.direction==-1 && s.electrical_frequency_millihz==0);
  CHECK(motor_hall_decoder_edge_process_captured(5,5,3,6000));
  CHECK(motor_hall_decoder_sample_read(&s) && s.electrical_frequency_millihz==166666);
  CHECK(motor_hall_decoder_edge_process_captured(1,6,4,2006000));
  CHECK(motor_hall_decoder_sample_read(&s) && s.electrical_frequency_millihz==0);
  CHECK(motor_hall_decoder_edge_process_captured(3,7,5,2007000));
  CHECK(motor_hall_decoder_sample_read(&s) && s.electrical_frequency_millihz==166666);
  CHECK(motor_hall_decoder_init(next,4000000,1,0));
  CHECK(motor_hall_decoder_edge_process_captured(5,1,1,0xfffffff0U));
  CHECK(motor_hall_decoder_edge_process_captured(4,2,2,984U));
  CHECK(motor_hall_decoder_sample_read(&s) && s.interval_cycles==4000);
  for(unsigned i=0;i<6;i++) (void)motor_hall_decoder_edge_process(0,10+i,100+i);
  CHECK(motor_hall_decoder_sample_read(&s) && s.health_fault);
  CHECK(motor_hall_direction_tables_build(next,entry,448,f,r));
  for(unsigned i=1;i<=6;i++) {
    CHECK(motor_hall_angle_boundary_get(next,entry,i,1,448,&a) && f[i]==a);
    CHECK(motor_hall_angle_boundary_get(next,entry,i,-1,448,&a) && r[i]==a);
    CHECK(r[i]==f[next[i]]);
  }
  CHECK(motor_hall_correction_step(3000)==182 && motor_hall_correction_step(-3000)==-182);
  CHECK(motor_hall_correction_step(1)==1 && motor_hall_correction_step(-1)==-1);
  { int32_t error=2731; unsigned count=0;
    while(error && count++<100) error-=motor_hall_correction_step(error);
    CHECK(error==0);
  }
  return 0;
}
