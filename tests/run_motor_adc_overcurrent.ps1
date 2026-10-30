$ErrorActionPreference='Stop'
Push-Location (Join-Path $PSScriptRoot '..')
try {
    New-Item -ItemType Directory -Force tmp | Out-Null
    & 'C:/Program Files/LLVM/bin/clang.exe' -O1 -ffreestanding -fno-builtin -I tests/host -I project/config -I project/bsp -I project/foc/foc_app tests/motor_adc_overcurrent_test.c -fuse-ld=lld -nostdlib '-Wl,/entry:main,/subsystem:console' -o tmp/motor_adc_overcurrent_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Overcurrent test compilation failed' }
    & ./tmp/motor_adc_overcurrent_test.exe
    if ($LASTEXITCODE -ne 0) { throw "Overcurrent regression failed at line $LASTEXITCODE" }
    Write-Output 'ADC overcurrent tests passed'
} finally { Pop-Location }
