$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '..')
try {
    New-Item -ItemType Directory -Force tmp | Out-Null
    & 'C:/Program Files/LLVM/bin/clang.exe' -O1 -ffreestanding -fno-builtin -ffunction-sections -I tests/host -I project/config -I project/inc -I project/foc/foc_app -I project/foc/foc_kernel tests/motor_speed_current_limit_test.c project/foc/foc_kernel/motor_speed_pi.c project/foc/foc_kernel/motor_ramp.c project/foc/foc_kernel/motor_direction.c -fuse-ld=lld -nostdlib '-Wl,/entry:main,/subsystem:console,/opt:ref' -o tmp/motor_speed_current_limit_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host test compilation failed' }
    & ./tmp/motor_speed_current_limit_test.exe
    if ($LASTEXITCODE -ne 0) { throw "Current limit regression failed at line $LASTEXITCODE" }
    Write-Output 'Speed current limit tests passed'
} finally { Pop-Location }
