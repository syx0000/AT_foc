$ErrorActionPreference='Stop'
Push-Location (Join-Path $PSScriptRoot '..')
try {
    $source=Get-Content project/bsp/motor_hall_port.c -Raw
    $body=''
    foreach($pattern in @('void motor_hall_port_overflow_capture','static uint64_t motor_hall_timer_now','uint32_t motor_hall_port_quiet_us','void motor_hall_port_edge_capture')) {
        $match=[regex]::Match($source,'(?s)'+$pattern+'\(void\).*?\n\}')
        if(-not $match.Success) { throw "Missing implementation: $pattern" }
        $body+=$match.Value+"`n"
    }
    $prefix=@'
#include "at32f45x.h"
#include "motor_hall_port.h"
static motor_hall_port_sample_t motor_hall_port_sample;
static uint64_t motor_hall_timer_epoch,motor_hall_capture_ticks;
static unsigned counter,captured,dwt;
static bool overflow;
#define MOTOR_HALL_CAPTURE_TIMER 0
#define TMR_OVF_FLAG 0
#define TMR_SELECT_CHANNEL_1 0
#define RESET 0
unsigned tmr_counter_value_get(int t) { (void)t; return counter; }
int tmr_flag_get(int t,int f) { (void)t;(void)f;return overflow; }
void tmr_flag_clear(int t,int f) { (void)t;(void)f;overflow=false; }
unsigned tmr_channel_value_get(int t,int c) { (void)t;(void)c;return captured; }
unsigned motor_timebase_cycles_get(void) { return dwt; }
uint8_t motor_hall_port_raw_state_get(void) { return 1; }
'@
    $suffix=@'
#define CHECK(x) do { if(!(x)) return __LINE__; } while(0)
int main(void) {
  counter=500; captured=450; dwt=999999;
  motor_hall_port_edge_capture(); CHECK(motor_hall_port_sample.capture_us==450);
  CHECK(motor_hall_port_quiet_us()==50);
  counter=5; overflow=true; captured=65530;
  motor_hall_port_edge_capture(); CHECK(motor_hall_port_sample.capture_us==65530);
  CHECK(motor_hall_port_quiet_us()==11);
  motor_hall_port_overflow_capture(); CHECK(!overflow && motor_hall_timer_epoch==65536);
  counter=20; captured=10;
  motor_hall_port_edge_capture(); CHECK(motor_hall_port_sample.capture_us==65546);
  counter=200; dwt=123456789;
  motor_hall_port_edge_capture(); CHECK(motor_hall_port_sample.capture_us==65546);
  CHECK(motor_hall_port_quiet_us()==190 && test_primask==0);
  return 0;
}
'@
    [IO.File]::WriteAllText((Join-Path (Get-Location) 'tmp/test_hall_port.c'),$prefix+"`n"+$body+"`n"+$suffix)
    & 'C:/Program Files/LLVM/bin/clang.exe' -O1 -ffreestanding -fno-builtin -I tests/host -I project/bsp tmp/test_hall_port.c -fuse-ld=lld -nostdlib '-Wl,/entry:main,/subsystem:console' -o tmp/test_hall_port.exe
    if($LASTEXITCODE -ne 0) { throw 'Capture test compilation failed' }
    & ./tmp/test_hall_port.exe
    if($LASTEXITCODE -ne 0) { throw "Capture test failed: $LASTEXITCODE" }
    Write-Output 'Hardware capture rollover and latency tests passed'
} finally { Pop-Location }
