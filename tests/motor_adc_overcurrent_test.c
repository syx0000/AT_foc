#include "../project/foc/foc_app/motor_current_sample.c"
static bool enabled;
bool motor_pwm_port_output_is_enabled(void) { return enabled; }
#define CHECK(x) do { if (!(x)) return __LINE__; } while(0)
int main(void)
{
  CHECK(MOTOR_CURRENT_COMMAND_MAX_MA == 60000L);
  CHECK(MOTOR_SOFTWARE_OVERCURRENT_MA == 64000L);
  CHECK(MOTOR_ADC_OVERCURRENT_HIGH_COUNTS < MOTOR_ADC_FULL_SCALE_COUNTS);
  CHECK(MOTOR_ADC_OVERCURRENT_LOW_COUNTS > 0U);
  motor_adc_fast_sample_t sample = {2048,2048,0};
  motor_current_sample_init();
  CHECK(!motor_current_sample_process(&sample,true));
  CHECK(motor_current_sample_offsets_set(2048,2048));
  enabled=true;
  for(unsigned i=0;i<29;i++) CHECK(!motor_current_sample_process(&sample,true));
  CHECK(!motor_current_sample_process(&sample,false));
  CHECK(motor_current_state.overcurrent_count==0);
  for(unsigned i=0;i<29;i++) CHECK(!motor_current_sample_process(&sample,true));
  CHECK(motor_current_sample_process(&sample,true));
  CHECK(motor_current_state.overcurrent_fault);
  CHECK(!motor_current_sample_process(&sample,true));
  CHECK(motor_current_sample_fault_clear());
  for(unsigned i=0;i<29;i++) CHECK(!motor_current_sample_process(&sample,true));
  enabled=false; CHECK(!motor_current_sample_process(&sample,true));
  CHECK(motor_current_state.overcurrent_count==0);
  enabled=true;
  for(unsigned i=0;i<29;i++) CHECK(!motor_current_sample_process(&sample,true));
  CHECK(motor_current_sample_process(&sample,true));
  return 0;
}
