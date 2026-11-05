# run-linux.ps1 - Build WoWee for Linux (amd64) inside a Docker container.
#
# The build leaves one file: build\linux\wowee-<version>-<arch>.AppImage.
#
# Usage (run from project root):
#   .\container\run-linux.ps1 [-RebuildImage]
#
# The CMake build tree is kept in the Docker volume wowee-linux-build, so a
# second run compiles only what changed. `docker volume rm wowee-linux-build`
# starts over.
#
# Environment variables:
#   WOWEE_FFX_SDK_REPO  - FidelityFX SDK git repo URL (passed through to container)
#   WOWEE_FFX_SDK_REF   - FidelityFX SDK git ref / tag      (passed through to container)

param(
    [switch]$RebuildImage
)

$ErrorActionPreference = "Stop"

$ScriptDir   = Split-Path -Parent $MyInvocation.MyCommand.Definition
$ProjectRoot = (Resolve-Path "$ScriptDir\..").Path

$ImageName   = "wowee-builder-linux"
$BuildVolume = "wowee-linux-build"
$BuildOutput = "$ProjectRoot\build\linux"

# Revision label of the builder image: an image without the current one is
# rebuilt instead of run. Changing it rebuilds every existing image.
$ImageRev    = "appimage-1"

# Verify Docker is available
if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
    Write-Error "docker is not installed or not in PATH."
    exit 1
}

# Is the image there, and does it carry the current revision label?
$labels = docker image inspect --format '{{.Config.Labels}}' $ImageName 2>$null
$imageCurrent = ($LASTEXITCODE -eq 0) -and ("$labels" -like "*wowee.builder.rev:$ImageRev*")

# Build the image (skip if present, current, and -RebuildImage not given)
if ($RebuildImage -or -not $imageCurrent) {
    Write-Host "==> Building Docker image: $ImageName"
    docker build `
        -f "$ScriptDir\builder-linux.Dockerfile" `
        --label "wowee.builder.rev=$ImageRev" `
        -t $ImageName `
        "$ScriptDir"
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
} else {
    Write-Host "==> Using existing Docker image: $ImageName"
}

# Create output directory on the host
New-Item -ItemType Directory -Force -Path $BuildOutput | Out-Null

Write-Host "==> Starting Linux build (output: $BuildOutput)"

$dockerArgs = @(
    "run", "--rm",
    "--mount", "type=bind,src=$ProjectRoot,dst=/src,readonly",
    "--mount", "type=bind,src=$BuildOutput,dst=/out",
    "--mount", "type=volume,src=$BuildVolume,dst=/build"
)

if ($env:WOWEE_FFX_SDK_REPO) {
    $dockerArgs += @("--env", "WOWEE_FFX_SDK_REPO=$env:WOWEE_FFX_SDK_REPO")
}
if ($env:WOWEE_FFX_SDK_REF) {
    $dockerArgs += @("--env", "WOWEE_FFX_SDK_REF=$env:WOWEE_FFX_SDK_REF")
}

$dockerArgs += $ImageName

& docker @dockerArgs
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

$appImages = @(Get-ChildItem -Path $BuildOutput -Filter "wowee-*.AppImage" -File)
if ($appImages.Count -eq 0) {
    Write-Error "the container finished but left no AppImage in $BuildOutput."
    exit 1
}
Write-Host "==> Linux build complete: $($appImages.FullName -join ', ')"
