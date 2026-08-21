# Build the bulb firmware with ESP8266_RTOS_SDK via CMake + Ninja.
#
# This drives CMake directly (not idf.py) and applies two workarounds needed on
# this Windows host:
#   1. .stubbin/ on PATH provides a stub `mconf-idf.exe` so the SDK's kconfig
#      init check passes (menuconfig itself is unavailable; a plain build uses
#      confgen.py). See .stubbin/README.md.
#   2. -DCMAKE_POLICY_VERSION_MINIMUM=3.5 lets CMake 4.x configure the SDK's old
#      submodules (mbedtls etc.) that declare cmake_minimum_required < 3.5.
#
# Prereqs on PATH: xtensa-lx106-elf-gcc, cmake, ninja, python (with the SDK's
# requirements.txt installed: pyserial click future setuptools ...).
$ErrorActionPreference = 'Stop'

$here = $PSScriptRoot

# Resolve IDF_PATH to a native Windows path. An inherited value may be a
# Git-Bash path like /c/Users/... which native CMake cannot resolve (the SDK's
# CMakeLists does include($ENV{IDF_PATH}/...)), so only honor an override that
# actually exists as a Windows directory; otherwise derive it from this script.
$sdkDefault = Join-Path $here '..\ESP8266_RTOS_SDK'
if ($env:IDF_PATH -and (Test-Path -LiteralPath $env:IDF_PATH -PathType Container)) {
    $env:IDF_PATH = (Resolve-Path -LiteralPath $env:IDF_PATH).Path
} elseif (Test-Path -LiteralPath $sdkDefault -PathType Container) {
    $env:IDF_PATH = (Resolve-Path -LiteralPath $sdkDefault).Path
} else {
    throw "ESP8266_RTOS_SDK not found. Set IDF_PATH to the SDK, or place it at $sdkDefault"
}
Write-Host "IDF_PATH = $($env:IDF_PATH)"

$env:PATH = (Join-Path $here '.stubbin') + [IO.Path]::PathSeparator + $env:PATH

$build_dir = Join-Path $here 'build'
New-Item -ItemType Directory -Force -Path $build_dir | Out-Null

# Auto-heal a poisoned cache: if a previous configure fell back to a host
# compiler (e.g. clang) instead of the xtensa cross-compiler, the cached
# CMAKE_C_COMPILER is sticky and will keep failing. Wipe it so we reconfigure.
$cache = Join-Path $build_dir 'CMakeCache.txt'
if (Test-Path -LiteralPath $cache) {
    $cc = Select-String -LiteralPath $cache -Pattern '^CMAKE_C_COMPILER:' -SimpleMatch:$false
    if (-not ($cc -and $cc.Line -match 'xtensa')) {
        Write-Host 'Stale/host-compiler cache detected; clearing build/ for a fresh configure.'
        Remove-Item -LiteralPath $cache -Force
        $cmf = Join-Path $build_dir 'CMakeFiles'
        if (Test-Path -LiteralPath $cmf) { Remove-Item -LiteralPath $cmf -Recurse -Force }
    }
}

Push-Location $build_dir
try {
    cmake -G Ninja '-DIDF_TARGET=esp8266' '-DCMAKE_POLICY_VERSION_MINIMUM=3.5' $here
    if ($LASTEXITCODE -ne 0) { throw "cmake failed with exit code $LASTEXITCODE" }
    ninja
    if ($LASTEXITCODE -ne 0) { throw "ninja failed with exit code $LASTEXITCODE" }
} finally {
    Pop-Location
}

Write-Host ''
Write-Host "artifacts in ${build_dir}:"
foreach ($a in 'bulb-firmware.bin', 'bootloader\bootloader.bin', 'partition_table\partition-table.bin') {
    if (Test-Path -LiteralPath (Join-Path $build_dir $a)) { Write-Host $a } else { Write-Host "$a (missing)" }
}

$appBin = Join-Path $build_dir 'bulb-firmware.bin'
if (Test-Path -LiteralPath $appBin) {
    $s = (Get-Item -LiteralPath $appBin).Length
    $slot = 0xF0000
    'app: {0} bytes ({1:F1} KB), {2:F1}% of {3} KB OTA slot' -f `
        $s, ($s / 1024), (100 * $s / $slot), [int]($slot / 1024)
}
