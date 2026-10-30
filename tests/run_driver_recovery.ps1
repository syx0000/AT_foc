$ErrorActionPreference='Stop'
Push-Location (Join-Path $PSScriptRoot '..')
try {
    $source=Get-Content project/foc/foc_app/motor_control.c -Raw
    $body=[regex]::Match($source,'(?s)static motor_fault_clear_result_t motor_control_driver_recover\(void\).*?\n\}').Value
    if (-not $body) { throw 'Recovery implementation not found' }
    $pwm=Get-Content project/bsp/motor_pwm_port.c -Raw
    $check=[regex]::Match($pwm,'(?s)bool motor_pwm_port_fault_check\(void\).*?\n\}').Value
    if (-not $check) { throw 'Read-only PWM check not found' }
    $prefix=@'
#include "motor_control.h"
#include "motor_current_sample.h"
#define MOTOR_GATE_DRIVER_WAKE_DELAY_MS 2U
static unsigned motor_fault_pending_mask;
static bool gate,pwm,driver_ok=true,fresh=true,current_ok=true,break_latched;
static bool inject_break;
static unsigned count,shutdowns;
bool motor_current_sample_state_read(motor_current_sample_state_t *s) { s->sample_count=count; return true; }
bool motor_current_sample_fault_clear(void) { return current_ok; }
void motor_pwm_port_output_disable(void) { pwm=false; }
void motor_pwm_port_gate_driver_set(bool e) { gate=e; }
void wk_delay_ms(unsigned ms) { if(fresh) count+=ms; if(inject_break) break_latched=true; }
void motor_pwm_port_recovery_prepare(void) { pwm=false; gate=false; break_latched=false; }
#define motor_pwm_gate_driver_enabled gate
#define MOTOR_PWM_BREAK_PORT 0
#define MOTOR_PWM_BREAK_PIN 0
#define MOTOR_PWM_TIMER 0
#define TMR_BRK_FLAG 0
#define RESET 0
int gpio_input_data_bit_read(int port,int pin) { (void)port; (void)pin; return driver_ok; }
int tmr_flag_get(int timer,int flag) { (void)timer; (void)flag; return break_latched; }
void motor_pwm_port_emergency_stop(void) { gate=false; pwm=false; shutdowns++; }
'@
    $suffix=@'
#define CHECK(x) do { if(!(x)) return __LINE__; } while(0)
int main(void) {
  CHECK(motor_control_driver_recover()==MOTOR_FAULT_CLEAR_OK && gate && !pwm);
  driver_ok=false;
  CHECK(motor_control_driver_recover()==MOTOR_FAULT_CLEAR_DRIVER_ACTIVE && !gate && !pwm);
  driver_ok=true; fresh=false;
  CHECK(motor_control_driver_recover()==MOTOR_FAULT_CLEAR_CURRENT_UNSAFE && !gate);
  fresh=true; current_ok=false;
  CHECK(motor_control_driver_recover()==MOTOR_FAULT_CLEAR_CURRENT_UNSAFE && !gate);
  current_ok=true; motor_fault_pending_mask=1;
  CHECK(motor_control_driver_recover()==MOTOR_FAULT_CLEAR_DRIVER_ACTIVE && !gate);
  CHECK(shutdowns==4);
  motor_fault_pending_mask=0; inject_break=true;
  CHECK(motor_control_driver_recover()==MOTOR_FAULT_CLEAR_DRIVER_ACTIVE);
  CHECK(break_latched && !gate && !pwm); /* new pulse must not be acknowledged */
  return 0;
}
'@
    [IO.File]::WriteAllText((Join-Path (Get-Location) 'tmp/test_driver_recovery.c'),$prefix+"`n"+$check+"`n"+$body+"`n"+$suffix)
    & 'C:/Program Files/LLVM/bin/clang.exe' -O1 -ffreestanding -fno-builtin -I project/foc/foc_app -I project/bsp tmp/test_driver_recovery.c -fuse-ld=lld -nostdlib '-Wl,/entry:main,/subsystem:console' -o tmp/test_driver_recovery.exe
    if ($LASTEXITCODE -ne 0) { throw 'Recovery test compilation failed' }
    & ./tmp/test_driver_recovery.exe
    if ($LASTEXITCODE -ne 0) { throw "Recovery test failed: $LASTEXITCODE" }
    Write-Output 'Driver recovery tests passed'
} finally { Pop-Location }
