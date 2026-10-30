$ErrorActionPreference='Stop'
Push-Location (Join-Path $PSScriptRoot '..')
try {
    & 'C:/Program Files/LLVM/bin/clang.exe' -O1 -ffreestanding -fno-builtin -I tests/host -I project/foc/foc_kernel tests/motor_hall_upgrade_test.c project/foc/foc_kernel/motor_hall_decoder.c project/foc/foc_kernel/motor_hall_angle_math.c -fuse-ld=lld -nostdlib '-Wl,/entry:main,/subsystem:console' -o tmp/motor_hall_upgrade_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Hall test compilation failed' }
    & ./tmp/motor_hall_upgrade_test.exe
    if ($LASTEXITCODE -ne 0) { throw "Hall test failed: $LASTEXITCODE" }
    Write-Output 'Hall capture, reversal, table and correction tests passed'
} finally { Pop-Location }
