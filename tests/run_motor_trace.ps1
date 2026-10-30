$ErrorActionPreference='Stop'
Push-Location (Join-Path $PSScriptRoot '..')
try {
  New-Item -ItemType Directory -Force tmp | Out-Null
  & 'C:/Program Files/LLVM/bin/clang.exe' -O1 -ffreestanding -fno-builtin -I tests/host -I project/inc -I project/config -I project/bsp -I project/foc/foc_app -I project/foc/foc_kernel tests/motor_trace_test.c -fuse-ld=lld -nostdlib '-Wl,/entry:main,/subsystem:console' -o tmp/motor_trace_test.exe
  if ($LASTEXITCODE -ne 0) { throw 'Trace test compilation failed' }
  & ./tmp/motor_trace_test.exe
  if ($LASTEXITCODE -ne 0) { throw "Trace regression failed at line $LASTEXITCODE" }
  Write-Output 'Trace sampler, queue, capture and CVP tests passed'
} finally { Pop-Location }
