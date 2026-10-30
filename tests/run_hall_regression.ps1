$ErrorActionPreference='Stop'
Push-Location (Join-Path $PSScriptRoot '..')
try {
    $cases=@(
        @{name='decoder';sources=@('tests/motor_hall_decoder_test.c','project/foc/foc_kernel/motor_hall_decoder.c')},
        @{name='direction_angle';sources=@('tests/motor_direction_angle_test.c','project/foc/foc_kernel/motor_direction.c','project/foc/foc_kernel/motor_hall_angle_math.c')},
        @{name='hall_calibration';sources=@('tests/motor_hall_calibration_test.c','project/foc/foc_kernel/motor_hall_calibration.c')},
        @{name='estimator_upgrade';sources=@('tests/motor_hall_estimator_upgrade_test.c','project/foc/foc_kernel/motor_hall_angle_math.c')},
        @{name='raw_health';sources=@('tests/motor_hall_raw_health_test.c','project/foc/foc_kernel/motor_hall_angle_math.c')}
    )
    foreach($case in $cases) {
        $target="tmp/test_$($case.name).exe"
        & 'C:/Program Files/LLVM/bin/clang.exe' -O1 -ffreestanding -fno-builtin -D__AT32F45x_WK_CONFIG_H -I tests/host -I project/config -I project/inc -I project/bsp -I project/foc/foc_app -I project/foc/foc_kernel @($case.sources) -fuse-ld=lld -nostdlib '-Wl,/entry:main,/subsystem:console' -o $target
        if($LASTEXITCODE -ne 0) { throw "$($case.name) compilation failed" }
        & "./$target"
        if($LASTEXITCODE -ne 0) { throw "$($case.name) failed: $LASTEXITCODE" }
        Write-Output "$($case.name) tests passed"
    }
} finally { Pop-Location }
