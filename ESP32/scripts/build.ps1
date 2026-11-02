﻿param([string]$ArduinoCli = 'arduino-cli',[string]$ConfigFile = '')
$ErrorActionPreference = 'Stop'
$cliArgs = @()
if ($ConfigFile) { $cliArgs += @('--config-file', $ConfigFile) }
$projectRoot = Split-Path -Parent $PSScriptRoot
& $ArduinoCli @cliArgs compile --fqbn 'esp32:esp32:esp32' --output-dir (Join-Path $projectRoot 'build') (Join-Path $projectRoot 'tof_ble')
if ($LASTEXITCODE -ne 0) { throw 'ESP32 compile failed' }
