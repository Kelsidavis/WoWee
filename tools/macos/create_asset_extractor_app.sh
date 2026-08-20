#!/bin/bash
set -euo pipefail

APP_PATH="${1:?usage: create_asset_extractor_app.sh <app-path> <version> <icon.icns>}"
VERSION="${2:?usage: create_asset_extractor_app.sh <app-path> <version> <icon.icns>}"
ICON="${3:?usage: create_asset_extractor_app.sh <app-path> <version> <icon.icns>}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

rm -rf "${APP_PATH}"
osacompile -o "${APP_PATH}" "${SCRIPT_DIR}/asset_extractor.applescript"
cp "${SCRIPT_DIR}/asset_extractor_launcher.sh" \
    "${APP_PATH}/Contents/Resources/extract-assets-terminal.sh"
chmod +x "${APP_PATH}/Contents/Resources/extract-assets-terminal.sh"
cp "${ICON}" "${APP_PATH}/Contents/Resources/Wowee.icns"

PLIST="${APP_PATH}/Contents/Info.plist"

# osacompile's generated metadata varies between macOS runner versions. Replace
# each value instead of assuming the key already exists (PlistBuddy's `Set`
# fails when a key is absent).
plist_add_string() {
    local key="$1"
    local value="$2"
    /usr/libexec/PlistBuddy -c "Delete :${key}" "${PLIST}" 2>/dev/null || true
    /usr/libexec/PlistBuddy -c "Add :${key} string ${value}" "${PLIST}"
}

plist_add_string CFBundleIdentifier com.wowee.asset-extractor
plist_add_string CFBundleName "Wowee Asset Extractor"
plist_add_string CFBundleDisplayName "Wowee Asset Extractor"
plist_add_string CFBundleIconFile Wowee.icns
plist_add_string CFBundleVersion "${VERSION}"
plist_add_string CFBundleShortVersionString "${VERSION}"
# Kept even though the applet no longer sends an Apple event: without it macOS
# refuses one silently rather than prompting, which is the failure this app
# already shipped once. If anything here ever talks to another app again, the
# consent dialog is the thing that should happen.
plist_add_string NSAppleEventsUsageDescription \
    "Wowee Asset Extractor opens a Terminal window to show extraction progress."
/usr/libexec/PlistBuddy -c "Delete :LSUIElement" "${PLIST}" 2>/dev/null || true
/usr/libexec/PlistBuddy -c "Add :LSUIElement bool true" "${PLIST}"
