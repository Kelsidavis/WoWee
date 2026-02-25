<#
.SYNOPSIS
    Converts raw RGBA texture dumps to PNG for visual inspection (Windows equivalent of debug_texture.sh).

.PARAMETER Width
    Texture width in pixels. Defaults to 1024.

.PARAMETER Height
    Texture height in pixels. Defaults to 1024.

.EXAMPLE
    .\debug_texture.ps1
    .\debug_texture.ps1 -Width 2048 -Height 2048
#>

param(
    [int]$Width = 1024,
    [int]$Height = 1024
)

$TempDir = $env:TEMP

Write-Host "Converting debug textures (${Width}x${Height})..."

# Find raw dumps — filenames include dimensions (e.g. wowee_composite_debug_1024x1024.raw)
$rawFiles = Get-ChildItem -Path $TempDir -Filter "wowee_*_debug*.raw" -ErrorAction SilentlyContinue

if (-not $rawFiles) {
    Write-Host "No debug dumps found in $TempDir"
    Write-Host "  (looked for $TempDir\wowee_*_debug*.raw)"
    exit 0
}

foreach ($rawItem in $rawFiles) {
    $raw = $rawItem.FullName
    $png = $raw -replace '\.raw$', '.png'

    # Try ImageMagick first, fall back to ffmpeg
    if (Get-Command magick -ErrorAction SilentlyContinue) {
        & magick -size "${Width}x${Height}" -depth 8 "rgba:$raw" "$png" 2>$null
        if ($LASTEXITCODE -eq 0) {
            Write-Host "Created $png (${Width}x${Height})"
        } else {
            Write-Host "Failed to convert $raw"
        }
    } elseif (Get-Command convert -ErrorAction SilentlyContinue) {
        & convert -size "${Width}x${Height}" -depth 8 "rgba:$raw" "$png" 2>$null
        if ($LASTEXITCODE -eq 0) {
            Write-Host "Created $png (${Width}x${Height})"
        } else {
            Write-Host "Failed to convert $raw"
        }
    } elseif (Get-Command ffmpeg -ErrorAction SilentlyContinue) {
        & ffmpeg -y -f rawvideo -pix_fmt rgba -s "${Width}x${Height}" -i "$raw" "$png" 2>$null
        if ($LASTEXITCODE -eq 0) {
            Write-Host "Created $png (${Width}x${Height})"
        } else {
            Write-Host "Failed to convert $raw"
        }
    } else {
        Write-Host "Need 'magick' (ImageMagick) or 'ffmpeg' to convert $raw"
        Write-Host "  Install: winget install ImageMagick.ImageMagick"
    }
}
