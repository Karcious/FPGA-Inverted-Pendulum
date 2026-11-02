param([string]$Compiler = 'g++')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$outputDir = Join-Path $projectRoot 'build'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$executable = Join-Path $outputDir 'test_io.exe'
& $Compiler -std=c++11 -I (Join-Path $projectRoot 'tests') -I (Join-Path $projectRoot 'tof_ble') (Join-Path $projectRoot 'tests/test_io.cpp') (Join-Path $projectRoot 'tof_ble/VL53L1X.cpp') -o $executable
if ($LASTEXITCODE -ne 0) { throw 'Test compile failed' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'Sensor I2C tests failed' }
