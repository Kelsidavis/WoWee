<#
.SYNOPSIS
    Builds the wowee project (Windows equivalent of build.sh).

.DESCRIPTION
    Creates a build directory, runs CMake configure + build, and creates a
    directory junction for the Data folder so the binary can find assets.
#>

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $ScriptDir

Write-Host "Building wowee..."

# Create build directory if it doesn't exist
if (-not (Test-Path "build")) {
    New-Item -ItemType Directory -Path "build" | Out-Null
}
Set-Location "build"

# Configure with CMake
Write-Host "Configuring with CMake..."
& cmake .. -DCMAKE_BUILD_TYPE=Release
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

# Build with all cores
$numProcs = $env:NUMBER_OF_PROCESSORS
if (-not $numProcs) { $numProcs = 4 }
Write-Host "Building with $numProcs cores..."
& cmake --build . --parallel $numProcs
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

# Ensure Data junction exists in bin directory
$binData = Join-Path (Get-Location) "bin\Data"
if (-not (Test-Path $binData)) {
    $target = (Resolve-Path (Join-Path (Get-Location) "..\Data")).Path
    cmd /c mklink /J "$binData" "$target"
}

Write-Host ""
Write-Host "Build complete! Binary: build\bin\wowee.exe"
Write-Host "Run with: cd build\bin && .\wowee.exe"
