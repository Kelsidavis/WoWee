#!/bin/bash
set -euo pipefail

APP_PATH="${1:?usage: sign_app.sh <app> [identity] [entitlements]}"
IDENTITY="${2:--}"
ENTITLEMENTS="${3:-}"

if [ -n "${ENTITLEMENTS}" ] && [ ! -f "${ENTITLEMENTS}" ]; then
    echo "ERROR: entitlements file not found: ${ENTITLEMENTS}" >&2
    exit 1
fi

# Homebrew bottles and downloaded resources can carry read-only modes or
# provenance/quarantine attributes. Both interfere with deterministic bundle
# sealing, so normalize the staged copy before applying any signature.
chmod -R u+w "${APP_PATH}"
xattr -cr "${APP_PATH}"

if [ "${IDENTITY}" = "-" ]; then
    codesign --force --deep --sign - "${APP_PATH}"
    exit 0
fi

MAIN_EXECUTABLE="$(/usr/libexec/PlistBuddy -c 'Print :CFBundleExecutable' \
    "${APP_PATH}/Contents/Info.plist")"
MAIN_EXECUTABLE_PATH="${APP_PATH}/Contents/MacOS/${MAIN_EXECUTABLE}"

# Sign every Mach-O component before the containing app bundle. Hardened
# runtime and a trusted timestamp are required for Apple notarization. Skip
# CFBundleExecutable here because codesign resolves that path to the containing
# bundle; signing the outer app below signs the main executable after all nested
# code is ready.
while IFS= read -r -d '' component; do
    if [ "${component}" != "${MAIN_EXECUTABLE_PATH}" ] && \
       file -b "${component}" | grep -q 'Mach-O'; then
        codesign --force --sign "${IDENTITY}" \
            --options runtime --timestamp "${component}"
    fi
done < <(find "${APP_PATH}/Contents" -type f -print0)

# Entitlements belong on the outer signature and nowhere else: it is the one
# that seals the main executable, and that executable is the process whose
# capabilities are being asked for. The nested dylibs above never call for any
# of them.
#
# Spelled as two calls rather than one with an argument array, because macOS
# ships bash 3.2, where an empty array expanded under `set -u` is an error.
if [ -n "${ENTITLEMENTS}" ]; then
    codesign --force --sign "${IDENTITY}" --entitlements "${ENTITLEMENTS}" \
        --options runtime --timestamp "${APP_PATH}"
else
    codesign --force --sign "${IDENTITY}" \
        --options runtime --timestamp "${APP_PATH}"
fi
