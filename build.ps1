# Build one device's firmware:  .\build.ps1 -Device bulb   |   .\build.ps1 -Device panel
#
# Devices live in devices/<name>/ (device.cmake says which SDK target). Each builds
# into its own build-<device>/ with its own sdkconfig.<device>, so they never collide.
#
#   esp8266 target (bulb)  ESP8266_RTOS_SDK via CMake + Ninja, driven directly (not
#     idf.py) with two workarounds this Windows host needs:
#       1. .stubbin/ on PATH provides a stub `mconf-idf.exe` so the SDK's kconfig init
#          check passes (menuconfig itself is unavailable; a plain build uses
#          confgen.py). See .stubbin/README.md.
#       2. -DCMAKE_POLICY_VERSION_MINIMUM=3.5 lets CMake 4.x configure the SDK's old
#          submodules (mbedtls etc.) that declare cmake_minimum_required < 3.5.
#     SDK: $env:ESP8266_SDK_PATH, else ..\ESP8266_RTOS_SDK. Prereqs on PATH:
#     xtensa-lx106-elf-gcc, cmake, ninja, python (with the SDK's requirements.txt).
#
#   anything else (panel)  ESP-IDF v6 via idf.py. Activated from $env:RC_IDF_ACTIVATE
#     if set, else the ESP-IDF Installation Manager profile
#     (C:\Espressif\tools\Microsoft.v6*.PowerShell_profile.ps1), else an esp-idf
#     export.ps1 ($env:IDF_V6_PATH, or C:\esp\v6*\esp-idf).
#
# $env:BULB_BUILD_ID pins the build id; $env:BULB_SELFTEST=1 builds the bulb self-test.
param([string]$Device)
$ErrorActionPreference = 'Stop'

$here = $PSScriptRoot
$devices = Get-ChildItem -Directory -LiteralPath (Join-Path $here 'devices') | ForEach-Object Name
if (-not $Device -or $devices -notcontains $Device) {
    throw "usage: .\build.ps1 -Device <$($devices -join '|')>"
}

# What the device is (one source of truth: devices/<device>/device.cmake).
$desc = Get-Content -Raw -LiteralPath (Join-Path $here "devices\$Device\device.cmake")
function DeviceSetting($name) {
    if ($desc -match "set\($name\s+([^\s\)]+)") { return $Matches[1] }
    throw "devices\$Device\device.cmake has no $name"
}
$target  = DeviceSetting 'DEVICE_TARGET'
$project = DeviceSetting 'DEVICE_PROJECT'
Write-Host "device $Device ($target) -> $project.bin"

# Stamp a unique build id into esp_app_desc.version via version.txt (the SDK reads
# it before git describe). This is the DFU2 build identity: the device and the update
# server both derive the 8-byte build id as sha256(version)[:8] (the ESP8266 toolchain
# does not populate app_elf_sha256). A fresh random token per build guarantees every
# image is distinguishable. Set BULB_BUILD_ID to pin it.
if ($env:BULB_BUILD_ID) {
    $buildId = $env:BULB_BUILD_ID
} else {
    $buildId = 'rc-' + -join ((1..6) | ForEach-Object { '{0:x2}' -f (Get-Random -Maximum 256) })
}
Set-Content -LiteralPath (Join-Path $here 'version.txt') -Value $buildId -Encoding ascii -NoNewline
Write-Host "build id (version.txt): $buildId"

$build_dir = Join-Path $here "build-$Device"
New-Item -ItemType Directory -Force -Path $build_dir | Out-Null

if ($target -eq 'esp8266') {
    # An inherited IDF_PATH may be an ESP-IDF checkout (or a Git-Bash path native CMake
    # cannot resolve), so use it only if it really is the ESP8266 SDK.
    $sdk = $null
    foreach ($c in @($env:ESP8266_SDK_PATH, $env:IDF_PATH, (Join-Path $here '..\ESP8266_RTOS_SDK'))) {
        if ($c -and (Test-Path -LiteralPath (Join-Path $c 'components\esp8266') -PathType Container)) {
            $sdk = (Resolve-Path -LiteralPath $c).Path; break
        }
    }
    if (-not $sdk) { throw "ESP8266_RTOS_SDK not found. Set ESP8266_SDK_PATH, or place it at ..\ESP8266_RTOS_SDK" }
    $env:IDF_PATH = $sdk
    Write-Host "IDF_PATH = $sdk"
    $env:PATH = (Join-Path $here '.stubbin') + [IO.Path]::PathSeparator + $env:PATH

    # Auto-heal a poisoned cache: if a previous configure fell back to a host
    # compiler (e.g. clang) instead of the xtensa cross-compiler, the cached
    # CMAKE_C_COMPILER is sticky and will keep failing. Wipe it so we reconfigure.
    $cache = Join-Path $build_dir 'CMakeCache.txt'
    if (Test-Path -LiteralPath $cache) {
        $cc = Select-String -LiteralPath $cache -Pattern '^CMAKE_C_COMPILER:'
        if (-not ($cc -and $cc.Line -match 'xtensa-lx106')) {
            Write-Host "Stale/host-compiler cache detected; clearing build-$Device/ for a fresh configure."
            Remove-Item -LiteralPath $cache -Force
            $cmf = Join-Path $build_dir 'CMakeFiles'
            if (Test-Path -LiteralPath $cmf) { Remove-Item -LiteralPath $cmf -Recurse -Force }
        }
    }

    Push-Location $build_dir
    # Native tools write notes/warnings to stderr; under 'Stop', PowerShell 5.1 turns
    # those into terminating errors when output is redirected. Judge by exit code.
    $ErrorActionPreference = 'Continue'
    try {
        cmake -G Ninja "-DDEVICE=$Device" '-DIDF_TARGET=esp8266' '-DCMAKE_POLICY_VERSION_MINIMUM=3.5' $here
        if ($LASTEXITCODE -ne 0) { throw "cmake failed with exit code $LASTEXITCODE" }
        ninja
        if ($LASTEXITCODE -ne 0) { throw "ninja failed with exit code $LASTEXITCODE" }
    } finally {
        Pop-Location
        $ErrorActionPreference = 'Stop'
    }
} else {
    $activate = $env:RC_IDF_ACTIVATE
    if (-not $activate) {
        $activate = Get-ChildItem 'C:\Espressif\tools\Microsoft.v6*.PowerShell_profile.ps1' -ErrorAction SilentlyContinue |
            Sort-Object Name | Select-Object -Last 1 -ExpandProperty FullName
    }
    if (-not $activate) {
        $idf = if ($env:IDF_V6_PATH) { $env:IDF_V6_PATH } else {
            Get-ChildItem 'C:\esp\v6*\esp-idf' -Directory -ErrorAction SilentlyContinue |
                Sort-Object FullName | Select-Object -Last 1 -ExpandProperty FullName }
        if ($idf) { $activate = Join-Path $idf 'export.ps1' }
    }
    if (-not $activate -or -not (Test-Path -LiteralPath $activate)) {
        throw "ESP-IDF v6 not found. Set RC_IDF_ACTIVATE to its activation script (export.ps1)."
    }
    Write-Host "activating ESP-IDF: $activate"
    $ErrorActionPreference = 'Continue'   # see the cmake call above
    . $activate *> $null
    idf.py -C $here -B $build_dir "-DDEVICE=$Device" build
    $rc = $LASTEXITCODE
    $ErrorActionPreference = 'Stop'
    if ($rc -ne 0) { throw "idf.py build failed with exit code $rc" }
}

Write-Host ''
Write-Host "artifacts in ${build_dir}:"
foreach ($a in "$project.bin", 'bootloader\bootloader.bin', 'partition_table\partition-table.bin') {
    if (Test-Path -LiteralPath (Join-Path $build_dir $a)) { Write-Host $a } else { Write-Host "$a (missing)" }
}

# App size against this device's OTA slot (ota_0 in its partition table).
$appBin = Join-Path $build_dir "$project.bin"
$ota0 = Select-String -LiteralPath (Join-Path $here "devices\$Device\partitions.csv") -Pattern '^\s*ota_0\s*,' |
    Select-Object -First 1
if ((Test-Path -LiteralPath $appBin) -and $ota0) {
    $s = (Get-Item -LiteralPath $appBin).Length
    $slot = [Convert]::ToInt32(($ota0.Line -split ',')[4].Trim(), 16)
    'app: {0} bytes ({1:F1} KB), {2:F1}% of {3} KB OTA slot' -f `
        $s, ($s / 1024), (100 * $s / $slot), [int]($slot / 1024)
}
