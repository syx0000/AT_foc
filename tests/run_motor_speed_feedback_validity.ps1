$ErrorActionPreference='Stop'
Push-Location (Join-Path $PSScriptRoot '..')
try {
    & 'C:/Program Files/LLVM/bin/clang.exe' -O1 -ffreestanding -fno-builtin -I tests/host -I project/config -I project/inc -I project/bsp -I project/foc/foc_app -I project/foc/foc_kernel tests/motor_speed_feedback_validity_test.c project/foc/foc_kernel/motor_direction.c -fuse-ld=lld -nostdlib '-Wl,/entry:main,/subsystem:console' -o tmp/motor_speed_feedback_validity_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Feedback test compilation failed' }
    & ./tmp/motor_speed_feedback_validity_test.exe
    if ($LASTEXITCODE -ne 0) { throw "Feedback test failed: $LASTEXITCODE" }
    Write-Output 'Speed feedback validity tests passed'
} finally { Pop-Location }
