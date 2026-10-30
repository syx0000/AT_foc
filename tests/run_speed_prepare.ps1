$ErrorActionPreference='Stop'
Push-Location (Join-Path $PSScriptRoot '..')
try {
    $source=Get-Content project/foc/foc_app/motor_control.c -Raw
    $body=[regex]::Match($source,'(?s)static void motor_control_speed_prepare_poll\(void\).*?\n\}').Value
    if (-not $body) { throw 'Preparation implementation not found' }
    $prefix=@'
#include "motor_control.h"
#include "motor_current_sample.h"
#include "motor_speed_feedback.h"
#include "motor_pwm_port.h"
#define MOTOR_GATE_DRIVER_WAKE_DELAY_MS 2U
static motor_control_status_t motor_control_status;
static unsigned motor_speed_prepare_cycles,motor_speed_prepare_sample_count,now,count;
static unsigned char motor_speed_prepare_phase;
static int motor_speed_prepare_target_rpm=500;
static bool standstill=true,driver_ok=true,pwm;
static unsigned starts,precharges;
unsigned motor_timebase_cycles_get(void) { return now; }
unsigned motor_timebase_cycles_elapsed(unsigned start) { return now-start; }
unsigned motor_timebase_cycles_to_us(unsigned cycles) { return cycles; }
bool motor_current_sample_state_read(motor_current_sample_state_t *s) { s->sample_count=count; return true; }
bool motor_speed_feedback_standstill_confirmed(void) { return standstill; }
bool motor_speed_feedback_read(motor_speed_feedback_t *s) { s->valid=!standstill; return true; }
bool motor_pwm_port_fault_check(void) { return driver_ok; }
void motor_pwm_port_compare_set(const motor_pwm_compare_t *c) { (void)c; }
bool motor_pwm_port_output_enable(void) { precharges++; pwm=true; return true; }
void motor_pwm_port_output_disable(void) { pwm=false; }
void motor_control_fault_set(motor_fault_code_t f) { motor_control_status.state=MOTOR_CONTROL_STATE_FAULT; motor_control_status.fault_code=f; pwm=false; }
static bool motor_control_speed_control_start_now(int rpm) { (void)rpm; starts++; motor_control_status.state=MOTOR_CONTROL_STATE_SPEED_CONTROL; return true; }
'@
    $suffix=@'
#define CHECK(x) do { if(!(x)) return __LINE__; } while(0)
int main(void) {
  motor_control_status.state=MOTOR_CONTROL_STATE_SPEED_PREPARING;
  now=1999; motor_control_speed_prepare_poll(); CHECK(!pwm && !starts);
  now=2000; count++; motor_control_speed_prepare_poll(); CHECK(motor_speed_prepare_phase==1 && !pwm);
  now=3000; motor_control_speed_prepare_poll(); CHECK(pwm && precharges==1);
  now=4000; count++; motor_control_speed_prepare_poll(); CHECK(!pwm && motor_speed_prepare_phase==3);
  now=5000; motor_control_speed_prepare_poll(); CHECK(starts==0);
  count++; motor_control_speed_prepare_poll(); CHECK(starts==1);
  motor_control_status.state=MOTOR_CONTROL_STATE_SPEED_PREPARING;
  motor_speed_prepare_phase=0; motor_speed_prepare_cycles=now; standstill=false;
  now+=2000; count++; motor_control_speed_prepare_poll(); CHECK(motor_speed_prepare_phase==3 && !pwm);
  now+=1000; count++; motor_control_speed_prepare_poll(); CHECK(starts==2 && precharges==1);
  motor_control_status.state=MOTOR_CONTROL_STATE_READY;
  motor_control_speed_prepare_poll(); CHECK(starts==2); /* cancelled */
  motor_control_status.state=MOTOR_CONTROL_STATE_SPEED_PREPARING;
  motor_speed_prepare_phase=0; motor_speed_prepare_cycles=now; driver_ok=false;
  now+=2000; motor_control_speed_prepare_poll(); CHECK(motor_control_status.state==MOTOR_CONTROL_STATE_FAULT && !pwm);
  driver_ok=true; motor_control_status.state=MOTOR_CONTROL_STATE_SPEED_PREPARING;
  motor_speed_prepare_phase=3; motor_speed_prepare_cycles=now;
  now+=100001; motor_control_speed_prepare_poll(); CHECK(motor_control_status.state==MOTOR_CONTROL_STATE_FAULT);
  return 0;
}
'@
    [IO.File]::WriteAllText((Join-Path (Get-Location) 'tmp/test_speed_prepare.c'),$prefix+"`n"+$body+"`n"+$suffix)
    & 'C:/Program Files/LLVM/bin/clang.exe' -O1 -ffreestanding -fno-builtin -I project/foc/foc_app -I project/bsp tmp/test_speed_prepare.c -fuse-ld=lld -nostdlib '-Wl,/entry:main,/subsystem:console' -o tmp/test_speed_prepare.exe
    if ($LASTEXITCODE -ne 0) { throw 'Preparation test compilation failed' }
    & ./tmp/test_speed_prepare.exe
    if ($LASTEXITCODE -ne 0) { throw "Preparation test failed: $LASTEXITCODE" }
    Write-Output 'Speed preparation tests passed'
} finally { Pop-Location }
