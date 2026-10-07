param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
Push-Location $projectRoot
try {
    New-Item -ItemType Directory -Force -Path 'build/friction_bench_tests' | Out-Null
    $sources = @('tests/friction_bench/test_friction.c', 'application/friction_bench/friction.c',
        'application/parameters/friction_profile.c', 'application/parameters/friction_profile_service.c',
        'module/loop_stats/loop_stats.c', 'module/event_history/event_history.c', 'application/runtime/runtime_events.c',
        'application/friction_bench/friction_control.c', 'application/friction_bench/friction_monitor.c',
        'module/speed_ready/speed_ready.c', 'module/motor/DJImotor/dji_motor.c',
        'module/algorithm/controller.c', 'module/motor/dji_speed_group/dji_speed_group.c',
        'bsp/control_platform/control_platform.c', 'module/vofa/vofa.c', 'application/friction_bench/friction_debug.c')
    $flags = @('-std=c11', '-Wall', '-Wextra', '-Wno-unused-function', '-Wno-unused-variable',
        '-Iapplication/parameters',
        '-Imodule/loop_stats', '-Imodule/event_history', '-Iapplication/runtime',
        '-Itests/friction_bench/stubs', '-Iapplication/friction_bench', '-Imodule/motor/DJImotor',
        '-Imodule/motor/dji_speed_group', '-Ibsp/control_platform', '-Imodule/motor', '-Imodule/algorithm', '-Imodule/vofa', '-Imodule/speed_ready')
    foreach ($variant in @('configured', 'unconfigured')) {
        $outputPath = "build/friction_bench_tests/$variant.exe"
        [string[]]$variantFlags = if ($variant -eq 'configured') { @('-include', 'tests/friction_bench/test_config.h') } else { @('-DTEST_UNCONFIGURED') }
        & $Compiler @flags @variantFlags @sources '-o' $outputPath '-lm'
        if ($LASTEXITCODE -ne 0) { throw "Host compile failed: $variant" }
        & (Join-Path $projectRoot $outputPath)
        if ($LASTEXITCODE -ne 0) { throw "Host tests failed: $variant" }
    }
} finally { Pop-Location }
