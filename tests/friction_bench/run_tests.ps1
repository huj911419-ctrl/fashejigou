param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
# Keep legacy entry paths; the common runner owns all source lists and checks.
& python (Join-Path $projectRoot 'tests/verify_offline.py') --compiler $Compiler
if ($LASTEXITCODE -ne 0) { throw 'Offline verification failed' }
