#!/usr/bin/env bash
# Copy a prepared data directory onto an iPad or iPhone.
#
#   tools/ios-copy-data.sh --udid <udid> --source ~/Data-ipad
#
# The client's data root is the app's Documents/Data, and the files have to be
# loose: the client opens them by path, one at a time, through std::ifstream.
# There is no zip reader in it, so an archive cannot be copied and unpacked
# later - it has to arrive as 199,302 individual files.
#
# That is the slow part, not the size. Apple's file sharing over USB is slow
# with many small files, so this uses devicectl's copy, which is the same
# mechanism but without Finder's overhead, and it reports progress as it goes.
#
# The expansion tables and the opcode tables are copied too, so the client
# finds an expansion and knows which wire protocol to speak.

set -euo pipefail

UDID=""
SOURCE=""
DOMAIN=""

usage() {
    cat <<'EOF'
Usage: tools/ios-copy-data.sh --udid <udid> --source <dir> [--domain <id>]

  --udid <id>       The device's UDID (xcrun devicectl list devices)
  --source <dir>    A prepared data directory - the one holding manifest.json
  --domain <id>     The app data container. Found automatically when omitted.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --udid)   UDID="$2"; shift 2 ;;
        --source) SOURCE="$2"; shift 2 ;;
        --domain) DOMAIN="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
    esac
done

if [[ -z "$UDID" || -z "$SOURCE" ]]; then
    echo "--udid and --source are required." >&2
    usage
    exit 1
fi

SOURCE="$(cd "$SOURCE" && pwd)"
if [[ ! -f "$SOURCE/manifest.json" ]]; then
    echo "no manifest.json in $SOURCE - that is not a prepared data directory." >&2
    exit 1
fi

if [[ -z "$DOMAIN" ]]; then
    DOMAIN="$(xcrun devicectl device info details --device "$UDID" 2>/dev/null \
        | grep -oE 'com\.wowee\.client\.Data-[A-Za-z0-9]+' | head -1)"
fi
if [[ -z "$DOMAIN" ]]; then
    echo "Could not find the app's data container. Open the app once first," >&2
    echo "or pass --domain. xcrun devicectl device info apps --device $UDID" >&2
    exit 1
fi

FILES="$(find "$SOURCE" -type f | wc -l | tr -d ' ')"
SIZE="$(du -sh "$SOURCE" | cut -f1)"
echo "Copying $SIZE in $FILES files to $DOMAIN on $UDID"
echo "This is many small files and it takes a while. Let it run."

xcrun devicectl device copy to \
    --device "$UDID" \
    --domain-type appDataContainer \
    --domain-identifier "$DOMAIN" \
    --source "$SOURCE" \
    --destination "Documents/Data"

echo
echo "Done. The client reads its data root from Documents/Data."
