#!/bin/bash
# Regenerate the iOS/iPadOS app icon set from the macOS app icon.
#
# The iPad and the Mac should show the same icon, so the source of truth is
# assets/Wowee.png - the artwork the macOS release workflow turns into
# Wowee.icns (tools/macos/create_icns.sh).  This script renders every slot
# the asset catalog needs from that one file.
#
# The artwork is a rounded square with transparent corners, which iOS does
# not accept: an app icon has to be a fully opaque square that the system
# masks itself.  flatten_icon.py fills the transparent corners by continuing
# the tile's own edge outwards, so the icon looks the same on the tablet as
# it does in the Dock.
#
# Usage: tools/ios/make_appicons.sh [source.png] [iconset-dir]
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SOURCE_PNG="${1:-${REPO_ROOT}/assets/Wowee.png}"
ICONSET="${2:-${REPO_ROOT}/ios/Assets.xcassets/AppIcon.appiconset}"

if [ ! -f "${SOURCE_PNG}" ]; then
    echo "no such icon source: ${SOURCE_PNG}" >&2
    exit 1
fi
if [ ! -d "${ICONSET}" ]; then
    echo "no such icon set: ${ICONSET}" >&2
    exit 1
fi

TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT

python3 "${REPO_ROOT}/tools/ios/flatten_icon.py" "${SOURCE_PNG}" "${TMP}/flat.png"

# filename -> pixel size (the layout the catalog's Contents.json declares)
while read -r name size; do
    [ -z "${name}" ] && continue
    sips -z "${size}" "${size}" "${TMP}/flat.png" \
        --out "${TMP}/${name}" >/dev/null
    # sips always writes an alpha channel; iOS app icons must not have one.
    python3 "${REPO_ROOT}/tools/ios/flatten_icon.py" \
        "${TMP}/${name}" "${ICONSET}/${name}" --strip-alpha >/dev/null
    echo "  ${name} (${size}x${size})"
done <<'EOF'
icon-20@1x.png 20
icon-20@2x.png 40
icon-20@3x.png 60
icon-29@1x.png 29
icon-29@2x.png 58
icon-29@3x.png 87
icon-40@1x.png 40
icon-40@2x.png 80
icon-40@3x.png 120
icon-60@2x.png 120
icon-60@3x.png 180
icon-76@1x.png 76
icon-76@2x.png 152
icon-83.5@2x.png 167
icon-1024.png 1024
EOF

echo "icon set written from ${SOURCE_PNG} to ${ICONSET}"
