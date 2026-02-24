<#
.SYNOPSIS
    Extracts WoW MPQ archives for use with wowee (Windows equivalent of extract_assets.sh).

.DESCRIPTION
    Builds the asset_extract tool (if needed) and runs it against a WoW Data directory.

.PARAMETER MpqDir
    Path to the WoW client's Data directory containing .MPQ files.

.PARAMETER Expansion
    Expansion hint: classic, turtle, tbc, wotlk. Auto-detected if omitted.

.EXAMPLE
    .\extract_assets.ps1 "C:\Games\WoW-3.3.5a\Data"
    .\extract_assets.ps1 "C:\Games\WoW-1.12\Data" classic
    .\extract_assets.ps1 "D:\TurtleWoW\Data" turtle
#>

param(
    [Parameter(Mandatory=$true, Position=0)]
    [string]$MpqDir,

    [Parameter(Position=1)]
    [string]$Expansion = "auto"
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$BuildDir  = Join-Path $ScriptDir "build"
$Binary    = Join-Path $BuildDir "bin\asset_extract.exe"
$OutputDir = Join-Path $ScriptDir "Data"

# --- Validate arguments ---
if (-not (Test-Path $MpqDir -PathType Container)) {
    Write-Error "Error: Directory not found: $MpqDir"
    exit 1
}

# Check for .MPQ files
$mpqFiles = Get-ChildItem -Path $MpqDir -Filter "*.MPQ" -ErrorAction SilentlyContinue
$mpqFilesLower = Get-ChildItem -Path $MpqDir -Filter "*.mpq" -ErrorAction SilentlyContinue
if (-not $mpqFiles -and -not $mpqFilesLower) {
    Write-Error "Error: No .MPQ files found in: $MpqDir`nMake sure this is the WoW Data/ directory (not the WoW root)."
    exit 1
}

# Note about existing CSV DBCs
if (Test-Path (Join-Path $OutputDir "expansions")) {
    $csvPattern = $null
    if ($Expansion -ne "auto") {
        $csvPattern = Join-Path $OutputDir "expansions\$Expansion\db\*.csv"
    } else {
        $csvPattern = Join-Path $OutputDir "expansions\*\db\*.csv"
    }
    if ($csvPattern -and (Get-ChildItem -Path $csvPattern -ErrorAction SilentlyContinue)) {
        Write-Host "Note: Found CSV DBCs. DBC extraction is optional; visual assets are still required.`n"
    }
}

# --- Build asset_extract if needed ---
if (-not (Test-Path $Binary)) {
    Write-Host "Building asset_extract..."

    # Check for cmake
    if (-not (Get-Command cmake -ErrorAction SilentlyContinue)) {
        Write-Error "Error: cmake not found. Install CMake and ensure it is on your PATH."
        exit 1
    }

    if (-not (Test-Path $BuildDir)) {
        & cmake -S $ScriptDir -B $BuildDir -DCMAKE_BUILD_TYPE=Release
        if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    }

    $numProcs = $env:NUMBER_OF_PROCESSORS
    if (-not $numProcs) { $numProcs = 4 }
    & cmake --build $BuildDir --target asset_extract --parallel $numProcs
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

    Write-Host ""
}

if (-not (Test-Path $Binary)) {
    Write-Error "Error: Failed to build asset_extract"
    exit 1
}

# --- Run extraction ---
Write-Host "Extracting assets from: $MpqDir"
Write-Host "Output directory:       $OutputDir"
Write-Host ""

$extraArgs = @("--mpq-dir", $MpqDir, "--output", $OutputDir)

if ($Expansion -ne "auto") {
    $extraArgs += "--expansion"
    $extraArgs += $Expansion
}

# Skip DBC extraction if CSVs already present
$skipDbc = $false
if ($Expansion -ne "auto") {
    $csvPath = Join-Path $OutputDir "expansions\$Expansion\db\*.csv"
    if (Get-ChildItem -Path $csvPath -ErrorAction SilentlyContinue) { $skipDbc = $true }
} else {
    $csvPath = Join-Path $OutputDir "expansions\*\db\*.csv"
    if (Get-ChildItem -Path $csvPath -ErrorAction SilentlyContinue) { $skipDbc = $true }
}
if ($skipDbc) {
    $extraArgs += "--skip-dbc"
}

& $Binary @extraArgs
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host ""
Write-Host "Done! Assets extracted to $OutputDir"
Write-Host "You can now run wowee."
