$ErrorActionPreference='Stop'
Push-Location (Join-Path $PSScriptRoot '..')
try {
    New-Item -ItemType Directory -Force tmp | Out-Null
    & 'C:/Program Files/LLVM/bin/clang.exe' -O1 -ffreestanding -fno-builtin -D__AT32F45x_WK_CONFIG_H -I tests/host -I project/inc -I project/config -I project/bsp -I project/foc/foc_app -I project/foc/foc_kernel tests/motor_debug_parameters_test.c project/foc/foc_kernel/motor_current_pi_design.c -fuse-ld=lld -nostdlib '-Wl,/entry:main,/subsystem:console' -o tmp/debug_parameters_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Parameter test compilation failed' }
    & ./tmp/debug_parameters_test.exe
    if ($LASTEXITCODE -ne 0) { throw "Parameter tests failed: $LASTEXITCODE" }
    Write-Output 'Parameter and Flash migration tests passed'
} finally { Pop-Location }
