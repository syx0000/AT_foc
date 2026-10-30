$ErrorActionPreference='Stop'
$taskRoot=(Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
Push-Location $taskRoot
try {
    New-Item -ItemType Directory -Force tmp | Out-Null
    & 'C:/Program Files/LLVM/bin/clang.exe' -O1 -ffreestanding -fno-builtin -fno-stack-protector -I tests/host_text -I motor_evb_2v0/at32f45x/pmsm_foc_hall_sensor/inc tests/host_text/test_host.c -fuse-ld=lld -nostdlib '-Wl,/entry:main,/subsystem:console' -o tmp/host_text_test.exe
    if($LASTEXITCODE -ne 0) {throw 'Host text test compilation failed'}
    & ./tmp/host_text_test.exe
    if($LASTEXITCODE -ne 0) {throw "Host text regression failed at line $LASTEXITCODE"}
    Write-Output 'Host text parser, state gates, units and 128-point capture passed'
} finally {Pop-Location}
