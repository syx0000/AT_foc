$ErrorActionPreference='Stop'
Push-Location (Join-Path $PSScriptRoot '..')
try {
    New-Item -ItemType Directory -Force tmp | Out-Null
    $source=Get-Content project/src/at32f45x_int.c -Raw
    $function=[regex]::Match($source,'static void hall_edge_dispatch\([^)]*\)\s*\{[\s\S]*?\r?\n\}').Value
    if (!$function) { throw 'Hall dispatch function not found' }
    $irq=[regex]::Match($source,'void TMR4_GLOBAL_IRQHandler\(void\)\s*\{[\s\S]*?\r?\n\}').Value
    if (!$irq) { throw 'Hall IRQ function not found' }
    $prefix=@'
#include <stdint.h>
#include <stdbool.h>
typedef struct { uint8_t state; uint32_t edge_count,timestamp_cycles,capture_us; } motor_hall_port_sample_t;
static bool read_ok=true, decoded;
static unsigned captures,calibrations,estimates;
#define MOTOR_HALL_CAPTURE_TIMER 0
#define RESET 0
#define TMR_OVF_FLAG 1U
#define TMR_C1_FLAG 2U
static unsigned flags,overflows;
int tmr_interrupt_flag_get(int timer,unsigned flag) { (void)timer; return (flags & flag)!=0; }
void tmr_flag_clear(int timer,unsigned flag) { (void)timer; flags &= ~flag; }
void motor_hall_port_overflow_capture(void) { overflows++; flags &= ~TMR_OVF_FLAG; }
void motor_hall_port_edge_capture(void) { captures++; }
bool motor_hall_port_sample_read(motor_hall_port_sample_t *s) { s->state=6; return read_ok; }
bool motor_hall_decoder_edge_process_captured(uint8_t s,uint32_t e,uint32_t t,uint32_t c) { (void)s;(void)e;(void)t;(void)c; return decoded; }
void motor_commissioning_hall_edge_process(void) { calibrations++; }
void motor_hall_angle_estimator_edge_process(void) { estimates++; }
'@
    $suffix=@'
#define CHECK(x) do { if(!(x)) return __LINE__; } while(0)
int main(void) {
  motor_hall_port_sample_t sample={0};
  decoded=false; hall_edge_dispatch(&sample);
  CHECK(captures==1 && calibrations==1 && estimates==0);
  decoded=true; hall_edge_dispatch(&sample);
  CHECK(captures==2 && calibrations==2 && estimates==1);
  read_ok=false; hall_edge_dispatch(&sample);
  CHECK(captures==3 && calibrations==2 && estimates==1);
  read_ok=true; decoded=false;
  flags=TMR_C1_FLAG; TMR4_GLOBAL_IRQHandler();
  CHECK(captures==4 && calibrations==3 && estimates==1 && flags==0);
  TMR4_GLOBAL_IRQHandler(); CHECK(captures==4);
  flags=TMR_OVF_FLAG; TMR4_GLOBAL_IRQHandler();
  CHECK(overflows==1 && captures==4 && flags==0);
  flags=TMR_OVF_FLAG|TMR_C1_FLAG; TMR4_GLOBAL_IRQHandler();
  CHECK(overflows==2 && captures==5 && calibrations==4 && flags==0);
  return 0;
}
'@
    Set-Content tmp/hall_commissioning_dispatch_test.c ($prefix+"`n"+$function+"`n"+$irq+"`n"+$suffix) -Encoding ASCII
    & 'C:/Program Files/LLVM/bin/clang.exe' -O1 -ffreestanding -fno-builtin -fuse-ld=lld -nostdlib '-Wl,/entry:main,/subsystem:console' tmp/hall_commissioning_dispatch_test.c -o tmp/hall_commissioning_dispatch_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Hall dispatch test compilation failed' }
    & ./tmp/hall_commissioning_dispatch_test.exe
    if ($LASTEXITCODE -ne 0) { throw "Hall dispatch test failed: $LASTEXITCODE" }
    Write-Output 'Hall commissioning dispatch tests passed'
} finally { Pop-Location }
