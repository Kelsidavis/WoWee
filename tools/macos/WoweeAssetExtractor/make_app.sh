#!/bin/bash
# Build WoweeAssetExtractor.app from the SwiftPM executable.
#
# SwiftPM produces a bare binary. A bare binary can draw a window, but it is
# not an app: it gets no Dock icon, no activation, and - the one that matters
# here - no drag and drop, because AppKit refuses a drag onto a process that
# LaunchServices does not know about. So the binary is wrapped in a bundle.
#
#     ./make_app.sh                     # build/WoweeAssetExtractor.app
#     ./make_app.sh --output ~/Desktop  # somewhere else
#     ./make_app.sh --identity "Developer ID Application: ..."
#
# Without an identity the bundle is signed ad-hoc, which is enough to run it
# locally but not to hand to anyone else.
set -euo pipefail

cd "$(dirname "$0")"

APP_NAME="WoweeAssetExtractor"
DISPLAY_NAME="Extracteur d'assets WoWee"
BUNDLE_ID="com.wowee.asset-extractor-ui"
VERSION="1.0.0"
OUTPUT_DIR="build"
IDENTITY="-"
ICON=""

while [ $# -gt 0 ]; do
    case "$1" in
        --output)   OUTPUT_DIR="$2"; shift 2 ;;
        --identity) IDENTITY="$2"; shift 2 ;;
        --version)  VERSION="$2"; shift 2 ;;
        --icon)     ICON="$2"; shift 2 ;;
        --help)
            sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        *)
            echo "Unknown option: $1" >&2
            exit 1 ;;
    esac
done

echo "==> Building (release)"
swift build -c release --product "${APP_NAME}"
BINARY="$(swift build -c release --product "${APP_NAME}" --show-bin-path)/${APP_NAME}"

if [ ! -x "${BINARY}" ]; then
    echo "ERROR: swift build produced no executable at ${BINARY}" >&2
    exit 1
fi

APP="${OUTPUT_DIR}/${APP_NAME}.app"
echo "==> Assembling ${APP}"
rm -rf "${APP}"
mkdir -p "${APP}/Contents/MacOS" "${APP}/Contents/Resources"
cp "${BINARY}" "${APP}/Contents/MacOS/${APP_NAME}"

# LSMinimumSystemVersion matches the release workflow's own floor so this app
# does not refuse to launch on a Mac that Wowee.app itself supports.
cat > "${APP}/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN"
  "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleExecutable</key><string>${APP_NAME}</string>
    <key>CFBundleIdentifier</key><string>${BUNDLE_ID}</string>
    <key>CFBundleName</key><string>${APP_NAME}</string>
    <key>CFBundleDisplayName</key><string>${DISPLAY_NAME}</string>
    <key>CFBundlePackageType</key><string>APPL</string>
    <key>CFBundleShortVersionString</key><string>${VERSION}</string>
    <key>CFBundleVersion</key><string>${VERSION}</string>
    <key>CFBundleIconFile</key><string>AppIcon</string>
    <key>LSMinimumSystemVersion</key><string>13.0</string>
    <key>NSHighResolutionCapable</key><true/>
    <!-- A window, not an agent. The shipped AppleScript applet sets
         LSUIElement and so has nowhere to show a failure; this one is a
         normal app precisely so that it can. -->
    <key>LSUIElement</key><false/>
</dict>
</plist>
PLIST

if [ -n "${ICON}" ] && [ -f "${ICON}" ]; then
    cp "${ICON}" "${APP}/Contents/Resources/AppIcon.icns"
    echo "==> Icon: ${ICON}"
fi

echo "==> Signing (${IDENTITY})"
if [ "${IDENTITY}" = "-" ]; then
    codesign --force --sign - "${APP}"
else
    codesign --force --sign "${IDENTITY}" --options runtime --timestamp "${APP}"
fi
codesign --verify --strict --verbose=2 "${APP}"

echo ""
echo "Built ${APP}"
echo "Run it with: open \"${APP}\""
