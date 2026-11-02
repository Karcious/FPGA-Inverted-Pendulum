﻿param([Parameter(Mandatory=$true)][string]$Port,[string]$ArduinoCli = 'arduino-cli',[string]$ConfigFile = '')
$ErrorActionPreference = 'Stop'
$cliArgs = @()
if ($ConfigFile) { $cliArgs += @('--config-file', $ConfigFile) }
$projectRoot = Split-Path -Parent $PSScriptRoot
$firmwarePath = Join-Path $projectRoot 'build'
if (-not (Test-Path -LiteralPath (Join-Path $firmwarePath 'tof_ble.ino.bin'))) { throw 'Build firmware first' }
& $ArduinoCli @cliArgs upload --fqbn 'esp32:esp32:esp32' --port $Port --input-dir $firmwarePath (Join-Path $projectRoot 'tof_ble')
if ($LASTEXITCODE -ne 0) { throw 'ESP32 upload failed' }
