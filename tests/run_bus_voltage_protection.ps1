$ErrorActionPreference='Stop'
Push-Location (Join-Path $PSScriptRoot '..')
try {
    # Compile the production check body with sensor/fault stand-ins.
    $source=Get-Content project/foc/foc_app/motor_control.c -Raw
    $body=[regex]::Match($source,'(?s)static bool motor_control_bus_voltage_check\(void\).*?\n\}').Value
    if (-not $body) { throw 'Production voltage check not found' }
    $prefix=@'
#include "motor_control.h"
#include "motor_control_config.h"
#include "motor_slow_sensor.h"
static motor_slow_sensor_state_t sample;
static bool ready = true;
static motor_fault_code_t fault;
static bool motor_bus_undervoltage_pending;
static uint32_t motor_bus_undervoltage_start_cycles;
static uint32_t now;
uint32_t motor_timebase_cycles_get(void) { return now; }
uint32_t motor_timebase_cycles_elapsed(uint32_t start) { return now-start; }
uint32_t motor_timebase_cycles_to_us(uint32_t cycles) { return cycles; }
bool motor_slow_sensor_state_read(motor_slow_sensor_state_t *s) { *s=sample; return ready; }
void motor_control_fault_set(motor_fault_code_t f) { fault=f; }
'@
    $suffix=@'
#define CHECK(x) do { if (!(x)) return __LINE__; } while(0)
int main(void) {
  sample.bus_voltage_0p1v=180; CHECK(motor_control_bus_voltage_check());
  sample.bus_voltage_0p1v=600; CHECK(motor_control_bus_voltage_check());
  sample.bus_voltage_0p1v=179; CHECK(motor_control_bus_voltage_check());
  now=299999; CHECK(motor_control_bus_voltage_check());
  sample.bus_voltage_0p1v=180; CHECK(motor_control_bus_voltage_check());
  sample.bus_voltage_0p1v=179; CHECK(motor_control_bus_voltage_check());
  now+=299999; CHECK(motor_control_bus_voltage_check());
  CHECK(fault==MOTOR_FAULT_NONE);
  now++; CHECK(!motor_control_bus_voltage_check());
  CHECK(fault==MOTOR_FAULT_BUS_UNDERVOLTAGE);
  sample.bus_voltage_0p1v=601; CHECK(!motor_control_bus_voltage_check());
  CHECK(fault==MOTOR_FAULT_BUS_OVERVOLTAGE);
  fault=MOTOR_FAULT_NONE; sample.bus_voltage_0p1v=240;
  CHECK(motor_control_bus_voltage_check() && fault==MOTOR_FAULT_NONE);
  ready=false; CHECK(motor_control_bus_voltage_check());
  ready=true; sample.bus_voltage_0p1v=179; now=0xffff0000U;
  CHECK(motor_control_bus_voltage_check());
  now+=300000U; CHECK(!motor_control_bus_voltage_check());
  CHECK(fault==MOTOR_FAULT_BUS_UNDERVOLTAGE);
  return 0;
}
'@
    New-Item -ItemType Directory -Force tmp | Out-Null
    [IO.File]::WriteAllText((Join-Path (Get-Location) 'tmp/test_bus_voltage.c'),$prefix+"`n"+$body+"`n"+$suffix)
    & 'C:/Program Files/LLVM/bin/clang.exe' -O1 -ffreestanding -fno-builtin -I project/config -I project/foc/foc_app tmp/test_bus_voltage.c -fuse-ld=lld -nostdlib '-Wl,/entry:main,/subsystem:console' -o tmp/test_bus_voltage.exe
    if ($LASTEXITCODE -ne 0) { throw 'Voltage test compilation failed' }
    & ./tmp/test_bus_voltage.exe
    if ($LASTEXITCODE -ne 0) { throw "Voltage test failed at line $LASTEXITCODE" }
    Write-Output 'Bus voltage boundary tests passed'
} finally { Pop-Location }
