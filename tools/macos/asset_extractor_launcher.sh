#!/bin/bash
set -euo pipefail

finish() {
    status=$?
    echo ""
    if [ "${status}" -eq 0 ]; then
        echo "Asset extraction finished successfully."
        echo "You can now open Wowee.app."
    else
        echo "Asset extraction failed (exit ${status})."
    fi
    echo ""
    read -r -p "Press Return to close this window..." _ || true
    exit "${status}"
}
trap finish EXIT

DIST_ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
APP_PATH="${DIST_ROOT}/Wowee.app"
# Beside this app first, then the two places an app is installed. ~/Applications
# is where #117's reporter had put both, and only /Applications was looked at.
for candidate in "/Applications/Wowee.app" "${HOME}/Applications/Wowee.app"; do
    [ -d "${APP_PATH}" ] && break
    [ -d "${candidate}" ] && APP_PATH="${candidate}"
done

EXTRACTOR="${APP_PATH}/Contents/MacOS/asset_extract"
BUNDLED_DATA="${APP_PATH}/Contents/Resources/Data"
OUTPUT_ROOT="${HOME}/Library/Application Support/Wowee/Data"

if [ ! -x "${EXTRACTOR}" ]; then
    echo "Could not find the bundled asset extractor."
    echo "Keep Wowee Asset Extractor.app beside Wowee.app, or install Wowee.app"
    echo "in /Applications or ~/Applications."
    exit 1
fi

WOW_DATA_DIR="$(osascript <<'APPLESCRIPT'
try
    set selectedFolder to choose folder with prompt "Select your World of Warcraft Data folder (the folder containing MPQ files)."
    return POSIX path of selectedFolder
on error number -128
    return ""
end try
APPLESCRIPT
)"

if [ -z "${WOW_DATA_DIR}" ]; then
    echo "No Data folder selected."
    exit 1
fi

mkdir -p "${OUTPUT_ROOT}"

# Seed expansion profiles and other redistributable configuration without
# writing into the signed application. ditto merges with existing extractions.
if [ -d "${BUNDLED_DATA}" ]; then
    ditto "${BUNDLED_DATA}" "${OUTPUT_ROOT}"
fi

echo "Wow data: ${WOW_DATA_DIR}"
echo "Output:   ${OUTPUT_ROOT}"
echo ""

"${EXTRACTOR}" \
    --mpq-dir "${WOW_DATA_DIR}" \
    --output "${OUTPUT_ROOT}" \
    --expansion-subdir
