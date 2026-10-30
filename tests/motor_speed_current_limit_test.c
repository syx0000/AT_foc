/* Exercise the actual controller with host current-loop/feedback stand-ins. */
#include "../project/foc/foc_app/motor_speed_control.c"
#include "motor_flying_seed.h"
static motor_current_control_status_t test_current;
static unsigned test_stops;
static int32_t test_speed;
static bool test_speed_valid=true;
static bool test_standstill;
static uint32_t test_iq_slew=10000;
static bool irq_saw_ready;
static int32_t irq_saw_target;
static void test_pending_systick(void) {
  irq_saw_ready=motor_speed_startup_slew_active;
  irq_saw_target=motor_speed_control_status.target_speed_millirpm;
  motor_speed_control_process_1khz();
}
bool motor_speed_feedback_standstill_confirmed(void) { return test_standstill; }
bool motor_parameter_active_read(motor_parameter_t *p)
{ (void)p; return false; }
bool motor_current_control_status_read(motor_current_control_status_t *s)
{ *s = test_current; return true; }
bool motor_current_control_command_set(const motor_current_control_command_t *c)
{ test_current.command = *c; return true; }
void motor_current_control_stop(void) { ++test_stops; }
bool motor_speed_feedback_read(motor_speed_feedback_t *s)
{ s->valid = test_speed_valid; s->filtered_speed_millirpm = test_speed; s->raw_speed_millirpm = test_speed; return true; }
bool motor_parameter_direction_inverted_get(void) { return false; }
uint32_t motor_parameter_debug_get(motor_parameter_field_t f)
{ return f == MOTOR_PARAMETER_FIELD_SPEED_STARTUP_IQ_SLEW_MA_S ? test_iq_slew :
    (f == MOTOR_PARAMETER_FIELD_STALL_TIME_MS ? 1000U : 30U); }
void *memcpy(void *d, const void *s, unsigned long long n)
{ unsigned char *a=d; const unsigned char *b=s; while(n--) *a++=*b++; return d; }
void *memset(void *d, int c, unsigned long long n)
{ unsigned char *a=d; while(n--) *a++=(unsigned char)c; return d; }
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
int main(void)
{
  int32_t seed;
  CHECK(motor_flying_seed_mv(19494,20000,1,13680,&seed) && seed==2449);
  CHECK(motor_flying_seed_mv(19494,20000,-1,13680,&seed) && seed==-2449);
  CHECK(motor_flying_seed_mv(1000000,1000000,1,13680,&seed) && seed==13680);
  CHECK(!motor_flying_seed_mv(0,20000,1,13680,&seed));
  CHECK(!motor_flying_seed_mv(100,20000,0,13680,&seed));
  motor_speed_control_status.state = MOTOR_SPEED_CONTROL_RUNNING;
  test_current.state = MOTOR_CURRENT_CONTROL_RUNNING;
  test_current.command.quadrature_reference_ma = -10000;
  motor_speed_control_status.quadrature_current_command_ma = -10000;
  motor_speed_controller.integral_ma = -10000;
  motor_speed_controller.output_ma = -10000;
  CHECK(motor_speed_control_current_limit_set(5000));
  CHECK(test_current.command.quadrature_reference_ma == -5000);
  CHECK(motor_speed_controller.integral_ma == -5000);
  CHECK(!motor_speed_control_current_limit_set(-1));
  CHECK(motor_speed_controller.output_limit_ma == 5000);
  CHECK(motor_speed_control_current_limit_set(0));
  CHECK(test_current.command.quadrature_reference_ma == 0);
  CHECK(motor_speed_controller.integral_ma == 0 && motor_speed_controller.output_ma == 0);
  motor_speed_control_config.acceleration_rpm_per_s = 3000;
  motor_speed_control_config.maximum_speed_rpm = 4000;
  motor_speed_control_config.deceleration_rpm_per_s = 1000;
  for (unsigned i=0; i<1500; ++i) motor_speed_control_process_1khz();
  CHECK(test_stops == 0 && test_current.command.quadrature_reference_ma == 0);
  CHECK(motor_speed_control_status.state == MOTOR_SPEED_CONTROL_RUNNING);
  CHECK(motor_speed_control_current_limit_set(5000));
  motor_speed_controller.proportional_gain_q20 = 1048576;
  motor_speed_ramp.current = motor_speed_ramp.target = 100000;
  motor_speed_control_status.target_speed_millirpm = 100000;
  motor_speed_run_direction = 1;
  motor_speed_control_process_1khz();
  CHECK(test_current.command.quadrature_reference_ma > 0);
  for (unsigned i=0; i<2000; ++i) motor_speed_control_process_1khz();
  CHECK(test_stops == 0 && motor_speed_control_status.state == MOTOR_SPEED_CONTROL_RUNNING);
  CHECK(test_current.command.quadrature_reference_ma == 5000);
  motor_speed_ramp.current = motor_speed_ramp.target = -100000;
  motor_speed_control_status.target_speed_millirpm = -100000;
  motor_speed_run_direction = -1;
  for (unsigned i=0; i<2000; ++i) motor_speed_control_process_1khz();
  CHECK(test_stops == 0 && motor_speed_control_status.state == MOTOR_SPEED_CONTROL_RUNNING);
  CHECK(test_current.command.quadrature_reference_ma == -5000);
  CHECK(motor_speed_control_current_limit_set(0));
  CHECK(test_current.command.quadrature_reference_ma == 0);
  CHECK(motor_speed_control_current_limit_set(100000));
  CHECK(motor_speed_controller.output_limit_ma == MOTOR_SPEED_CONTROL_CURRENT_LIMIT_MA);
  CHECK(test_primask == 0);
  test_primask = 1;
  CHECK(motor_speed_control_current_limit_set(0) && test_primask == 1);
  test_primask = 0;
  motor_speed_run_direction=1;
  motor_speed_ramp.current=100000;
  motor_speed_control_status.target_speed_millirpm=-100000;
  test_speed=100000;
  motor_speed_control_process_1khz();
  CHECK(motor_speed_ramp.current==99000); /* deceleration 1 rpm/ms */
  for(unsigned i=0;i<200;i++) motor_speed_control_process_1khz();
  CHECK(motor_speed_ramp.current==0 && motor_speed_run_direction==1);
  test_speed=0;
  for(unsigned i=0;i<99;i++) {
    CHECK(motor_speed_control_target_set(-100000)); /* repeated CAN target */
    motor_speed_control_process_1khz();
    CHECK(motor_speed_ramp.current==0);
  }
  test_speed=31000; motor_speed_control_process_1khz();
  CHECK(motor_speed_reverse_zero_ms==0);
  test_speed=0;
  for(unsigned i=0;i<100;i++) motor_speed_control_process_1khz();
  CHECK(motor_speed_run_direction==-1 && motor_speed_ramp.current==0); /* zero limit holds reference */
  CHECK(test_stops==0);
  CHECK(motor_speed_control_current_limit_set(5000));
  motor_speed_startup_slew_active=true;
  motor_speed_startup_stable_ms=0;
  motor_speed_control_status.quadrature_current_command_ma=0;
  motor_speed_run_direction=1;
  motor_speed_ramp.current=motor_speed_ramp.target=100000;
  motor_speed_control_status.target_speed_millirpm=100000;
  motor_speed_controller.integral_ma=0;
  test_speed=0;
  motor_speed_control_process_1khz();
  CHECK(test_current.command.quadrature_reference_ma==10);
  CHECK(motor_speed_controller.integral_ma==0);
  for(unsigned i=0;i<499;i++) motor_speed_control_process_1khz();
  CHECK(test_current.command.quadrature_reference_ma==5000);
  CHECK(motor_speed_startup_slew_active); /* blocked: no premature exit */
  CHECK(motor_speed_control_current_limit_set(100));
  CHECK(test_current.command.quadrature_reference_ma==100);
  CHECK(motor_speed_control_current_limit_set(0));
  CHECK(test_current.command.quadrature_reference_ma==0);
  motor_speed_control_process_1khz();
  CHECK(test_current.command.quadrature_reference_ma==0);
  CHECK(motor_speed_control_current_limit_set(5000));
  motor_speed_run_direction=-1;
  motor_speed_ramp.current=motor_speed_ramp.target=-100000;
  motor_speed_control_status.target_speed_millirpm=-100000;
  motor_speed_control_process_1khz();
  CHECK(test_current.command.quadrature_reference_ma==-10);
  test_speed=-100000;
  for(unsigned i=0;i<101;i++) motor_speed_control_process_1khz();
  CHECK(!motor_speed_startup_slew_active);
  /* After smoothing has exited, zero-current recovery must rearm it. */
  CHECK(motor_speed_control_current_limit_set(0));
  test_speed=0;
  CHECK(motor_speed_control_current_limit_set(5000));
  CHECK(motor_speed_startup_slew_active);
  motor_speed_control_process_1khz();
  CHECK(test_current.command.quadrature_reference_ma==-10);
  CHECK(motor_speed_control_current_limit_set(0));
  CHECK(motor_speed_control_current_limit_set(5000));
  test_iq_slew=1500;
  for(unsigned i=0;i<1000;i++) motor_speed_control_process_1khz();
  CHECK(test_current.command.quadrature_reference_ma==-1500);
  motor_speed_control_status.target_speed_millirpm=100000;
  motor_speed_ramp.current=0;
  test_speed_valid=false;
  for(unsigned i=0;i<150;i++) motor_speed_control_process_1khz();
  CHECK(motor_speed_run_direction==-1 && motor_speed_reverse_zero_ms==0);
  test_standstill=true;
  for(unsigned i=0;i<100;i++) motor_speed_control_process_1khz();
  CHECK(motor_speed_run_direction==1);
  test_speed_valid=true; test_speed=0;
  motor_speed_control_status.target_speed_millirpm=100000;
  motor_speed_ramp.current=100000;
  test_iq_slew=10000;
  CHECK(motor_speed_control_current_limit_set(500));
  motor_speed_control_status.quadrature_current_command_ma=500;
  test_current.command.quadrature_reference_ma=500;
  motor_speed_startup_slew_active=false;
  CHECK(motor_speed_control_current_limit_set(5000));
  motor_speed_control_process_1khz();
  CHECK(test_current.command.quadrature_reference_ma==510);
  motor_speed_control_stop();
  CHECK(!motor_speed_startup_slew_active &&
        motor_speed_control_status.quadrature_current_command_ma==0);
  test_speed=-100000;
  CHECK(motor_speed_control_start(100000,0));
  CHECK(motor_speed_run_direction==-1);
  CHECK(motor_speed_ramp.current==-100000);
  motor_speed_control_process_1khz();
  CHECK(motor_speed_ramp.current==-99000); /* opposite coast decelerates first */
  motor_speed_control_stop();
  test_speed=100000; test_speed_valid=true; test_iq_slew=10000;
  CHECK(motor_speed_control_current_limit_set(5000));
  motor_speed_control_config.acceleration_rpm_per_s=30000;
  motor_speed_control_config.deceleration_rpm_per_s=30000;
  test_irq_hook=test_pending_systick;
  CHECK(motor_speed_control_start(200000,0));
  CHECK(irq_saw_ready && irq_saw_target==200000);
  CHECK(test_current.command.quadrature_reference_ma==10);
  test_irq_hook=test_pending_systick;
  CHECK(motor_speed_control_target_set(-200000));
  CHECK(irq_saw_target==-200000 && test_primask==0);
  test_irq_hook=test_pending_systick;
  motor_speed_control_stop();
  CHECK(irq_saw_target==0 && !irq_saw_ready && test_primask==0);
  test_primask=1;
  CHECK(motor_speed_control_start(200000,0) && test_primask==1);
  motor_speed_control_stop();
  CHECK(test_primask==1);
  test_primask=0;
  motor_speed_control_status.state=MOTOR_SPEED_CONTROL_RUNNING;
  motor_speed_run_direction=1;
  motor_speed_startup_slew_active=false;
  motor_speed_control_config.current_limit_ma=5000;
  motor_speed_controller.output_limit_ma=5000;
  motor_speed_ramp.current=100000;
  motor_speed_control_status.target_speed_millirpm=1000000;
  motor_speed_control_status.quadrature_current_command_ma=5000;
  test_speed=0;
  motor_speed_control_process_1khz();
  CHECK(motor_speed_ramp.current==100000); /* blocked and limited: hold target */
  test_speed=99000;
  motor_speed_control_process_1khz();
  CHECK(motor_speed_ramp.current==130000); /* feedback caught up: resume */
  motor_speed_control_status.target_speed_millirpm=50000;
  test_speed=0;
  motor_speed_control_process_1khz();
  CHECK(motor_speed_ramp.current==100000); /* deceleration is never frozen */
  return 0;
}
