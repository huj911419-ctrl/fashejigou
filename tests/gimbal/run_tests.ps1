param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
Push-Location $projectRoot
try {
    New-Item -ItemType Directory -Force 'build/gimbal_tests' | Out-Null
    $flags = @('-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic',
        '-Imodule/control_authority',
        '-Imodule/dbus_input', '-Imodule/dm_velocity', '-Imodule/yaw_rate',
        '-Imodule/loop_stats', '-Imodule/event_history', '-Iapplication/runtime',
        '-Iapplication/gimbal_bench', '-Iapplication/gimbal_control', '-Iapplication/robot_cmd', '-Ibsp/control_platform')
    $sources = @('module/dbus_input/dbus_input.c', 'module/dm_velocity/dm_velocity.c',
        'module/control_authority/control_authority.c', 'application/robot_cmd/gimbal_arbiter.c',
        'module/yaw_rate/yaw_rate.c', 'module/loop_stats/loop_stats.c', 'application/robot_cmd/gimbal_remote.c')
    & $Compiler @flags @sources 'tests/gimbal/test_gimbal_core.c' '-o' 'build/gimbal_tests/core.exe' '-lm'
    if ($LASTEXITCODE -ne 0) { throw 'Gimbal core compile failed' }
    & (Join-Path $projectRoot 'build/gimbal_tests/core.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Gimbal core tests failed' }
    & $Compiler @flags '-Ibsp/gimbal_port' @sources `
        'module/event_history/event_history.c' 'application/runtime/runtime_events.c' `
        'application/gimbal_control/gimbal_control.c' 'application/gimbal_bench/gimbal.c' 'tests/gimbal/test_gimbal_application.c' `
        '-o' 'build/gimbal_tests/application.exe' '-lm'
    if ($LASTEXITCODE -ne 0) { throw 'Gimbal application compile failed' }
    & (Join-Path $projectRoot 'build/gimbal_tests/application.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Gimbal application tests failed' }
    & $Compiler @flags @sources 'application/gimbal_control/gimbal_control.c' `
        'tests/gimbal/test_gimbal_instances.c' '-o' 'build/gimbal_tests/instances.exe' '-lm'
    if ($LASTEXITCODE -ne 0) { throw 'Gimbal instance compile failed' }
    & (Join-Path $projectRoot 'build/gimbal_tests/instances.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Gimbal instance tests failed' }
    & $Compiler @flags @sources 'application/gimbal_control/gimbal_control.c' `
        'tests/gimbal/test_gimbal_commands.c' '-o' 'build/gimbal_tests/commands.exe' '-lm'
    if ($LASTEXITCODE -ne 0) { throw 'Gimbal command compile failed' }
    & (Join-Path $projectRoot 'build/gimbal_tests/commands.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Gimbal command tests failed' }
} finally { Pop-Location }
