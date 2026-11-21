$ErrorActionPreference='Stop'
Push-Location (Join-Path $PSScriptRoot '..')
try {
  New-Item -ItemType Directory -Force tmp | Out-Null
  $source=Get-Content project/foc/foc_app/motor_commissioning.c -Raw
  $function=[regex]::Match($source,'static bool commissioning_flux_result_print\([\s\S]*?\r?\n\}').Value
  if (!$function) { throw 'Flux calculation not found' }
  $constants=([regex]::Matches($source,'(?m)^#define MOTOR_COMMISSIONING_(?:FLUX_\w+|TWO_PI_X1000000)[^\r\n]*') | ForEach-Object { $_.Value }) -join "`n"
  $prefix=@'
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#define LOGI(...) ((void)0)
#define MOTOR_COMMISSIONING_ERROR_FLUX 10
typedef struct { uint32_t phase_resistance_mohm,direct_inductance_uh,flux_linkage_uwb; uint8_t pole_pairs; } motor_parameter_t;
typedef struct { int32_t direct_average_ma,quadrature_average_ma,direct_voltage_average_mv,quadrature_voltage_average_mv; uint32_t final_frequency_millihz; int8_t final_direction; bool final_speed_valid; } commissioning_iq_result_t;
static motor_parameter_t candidate={100,100,0,4};
static unsigned detail,writes;
bool motor_parameter_candidate_read(motor_parameter_t *p) { *p=candidate; return true; }
bool motor_parameter_candidate_set(const motor_parameter_t *p) { candidate=*p; writes++; return true; }
static void commissioning_error_set(int error,unsigned value) { (void)error; detail=value; }
'@
  $suffix=@'
#define CHECK(x) do { if(!(x)) return __LINE__; } while(0)
int main(void) {
  commissioning_iq_result_t m={1000,1000,0,5000,100000,1,true};
  CHECK(commissioning_flux_result_print(&m));
  uint32_t forward=candidate.flux_linkage_uwb;
  CHECK(forward>7600 && forward<7800);
  m.final_direction=-1; m.quadrature_average_ma=-1000; m.quadrature_voltage_average_mv=-5000;
  CHECK(commissioning_flux_result_print(&m));
  CHECK(candidate.flux_linkage_uwb==forward && writes==2);
  m.quadrature_voltage_average_mv=0;
  CHECK(!commissioning_flux_result_print(&m) && detail==4 && writes==2);
  m.final_direction=0;
  CHECK(!commissioning_flux_result_print(&m) && detail==3);
  m.final_direction=-1; m.final_speed_valid=false;
  CHECK(!commissioning_flux_result_print(&m) && detail==3);
  m.final_speed_valid=true; m.final_frequency_millihz=9999;
  CHECK(!commissioning_flux_result_print(&m) && detail==3);
  return 0;
}
'@
  Set-Content tmp/flux_direction_test.c ($prefix+"`n"+$constants+"`n"+$function+"`n"+$suffix) -Encoding ASCII
  & 'C:/Program Files/LLVM/bin/clang.exe' -O1 -ffreestanding -fno-builtin -fuse-ld=lld -nostdlib '-Wl,/entry:main,/subsystem:console' tmp/flux_direction_test.c -o tmp/flux_direction_test.exe
  if ($LASTEXITCODE -ne 0) { throw 'Flux direction test compilation failed' }
  & ./tmp/flux_direction_test.exe
  if ($LASTEXITCODE -ne 0) { throw "Flux direction test failed: $LASTEXITCODE" }
  Write-Output 'Flux forward/reverse and invalid back-EMF tests passed'
} finally { Pop-Location }
