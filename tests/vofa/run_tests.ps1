param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
Push-Location $projectRoot
try {
    New-Item -ItemType Directory -Force -Path 'build/vofa_tests' | Out-Null
    & $Compiler '-std=c11' '-Wall' '-Wextra' '-Werror' '-pedantic' '-Imodule/vofa' 'tests/vofa/test_vofa.c' 'module/vofa/vofa.c' '-o' 'build/vofa_tests/vofa.exe' '-lm'
    if ($LASTEXITCODE -ne 0) { throw 'VOFA host compile failed' }
    & (Join-Path $projectRoot 'build/vofa_tests/vofa.exe')
    if ($LASTEXITCODE -ne 0) { throw 'VOFA host tests failed' }
} finally { Pop-Location }
