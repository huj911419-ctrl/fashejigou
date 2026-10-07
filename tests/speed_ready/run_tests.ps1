param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
Push-Location $projectRoot
try {
    New-Item -ItemType Directory -Force -Path 'build/speed_ready_tests' | Out-Null
    $flags = @('-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic',
        '-Imodule/speed_ready', '-Imodule/loop_stats', '-Iapplication/friction_bench')
    $sources = @('tests/speed_ready/test_speed_ready.c', 'module/speed_ready/speed_ready.c',
        'application/friction_bench/friction_monitor.c')
    & $Compiler @flags @sources '-o' 'build/speed_ready_tests/speed_ready.exe' '-lm'
    if ($LASTEXITCODE -ne 0) { throw 'Readiness host compile failed' }
    & (Join-Path $projectRoot 'build/speed_ready_tests/speed_ready.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Readiness host tests failed' }
} finally { Pop-Location }
