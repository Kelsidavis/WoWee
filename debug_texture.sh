#!/bin/bash
# Convert raw RGBA texture dumps to PNG for visual inspection
# Usage: ./debug_texture.sh [width] [height]
# Defaults to 1024x1024 if not specified

W=${1:-1024}
H=${2:-1024}

echo "Converting debug textures (${W}x${H})..."

TMPD="${TMPDIR:-/tmp}"

# Find raw dumps — filenames include dimensions (e.g. wowee_composite_debug_1024x1024.raw)
shopt -s nullglob
RAW_FILES=("$TMPD"/wowee_*_debug*.raw)
shopt -u nullglob

if [ ${#RAW_FILES[@]} -eq 0 ]; then
    echo "No debug dumps found in $TMPD"
    echo "  (looked for $TMPD/wowee_*_debug*.raw)"
    exit 0
fi

for raw in "${RAW_FILES[@]}"; do
    png="${raw%.raw}.png"
    # Try ImageMagick first, fall back to ffmpeg
    if command -v convert &>/dev/null; then
        convert -size ${W}x${H} -depth 8 rgba:"$raw" "$png" 2>/dev/null && \
            echo "Created $png (${W}x${H})" || \
            echo "Failed to convert $raw"
    elif command -v ffmpeg &>/dev/null; then
        ffmpeg -y -f rawvideo -pix_fmt rgba -s ${W}x${H} -i "$raw" "$png" 2>/dev/null && \
            echo "Created $png (${W}x${H})" || \
            echo "Failed to convert $raw"
    else
        echo "Need 'convert' (ImageMagick) or 'ffmpeg' to convert $raw"
        echo "  Install: sudo apt install imagemagick"
    fi
done
