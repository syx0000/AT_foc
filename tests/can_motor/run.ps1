$ErrorActionPreference = 'Stop'
$taskRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
Push-Location $taskRoot
try {
    & 'C:/Program Files/LLVM/bin/clang.exe' -O1 -ffreestanding -fno-builtin -fno-stack-protector -I tests/can_motor -I motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/inc tests/can_motor/test_can_motor.c -fuse-ld=lld -nostdlib '-Wl,/entry:main,/subsystem:console' -o tests/can_motor/test_can_motor.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host test compilation failed' }
    & ./tests/can_motor/test_can_motor.exe
    if ($LASTEXITCODE -ne 0) { throw "Protocol test failed at source line $LASTEXITCODE" }
    Write-Output 'CAN protocol tests passed'
} finally {
    Pop-Location
}
