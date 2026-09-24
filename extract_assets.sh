#!/bin/bash
set -euo pipefail

# Usage: ./extract_assets.sh /path/to/WoW/Data [expansion]
#
# Extracts WoW MPQ archives into an expansion-isolated directory for wowee.
#
# Arguments:
#   $1  Path to WoW's Data directory (containing .MPQ files)
#   $2  Expansion hint: classic, turtle, tbc, wotlk (optional, auto-detected if omitted)
#
# Options:
#   --upscale[=N]  After extracting, run tools/upscale_textures.py over the
#                  tree: every texture of every foliage model, upscaled N times
#                  (default 4) into DXT5 .dds sidecars beside the BLPs. Revert
#                  with `python3 tools/upscale_textures.py --clean`.
#
# Examples:
#   ./extract_assets.sh /mnt/games/WoW-3.3.5a/Data
#   ./extract_assets.sh /mnt/games/WoW-1.12/Data classic
#   ./extract_assets.sh /mnt/games/TurtleWoW/Data turtle
#   ./extract_assets.sh /mnt/games/WoW-2.4.3/Data tbc

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
OUTPUT_DIR="${SCRIPT_DIR}/Data"

# Prefer pre-built binary next to this script (release archives), then build dir
if [ -x "${SCRIPT_DIR}/asset_extract" ]; then
    BINARY="${SCRIPT_DIR}/asset_extract"
else
    BINARY="${BUILD_DIR}/bin/asset_extract"
fi

# --- Options, in any position; what is left is positional ---
UPSCALE_SCALE=""
POSITIONAL=()
for arg in "$@"; do
    case "$arg" in
        --upscale)    UPSCALE_SCALE=4 ;;
        --upscale=*)  UPSCALE_SCALE="${arg#*=}" ;;
        *)            POSITIONAL+=("$arg") ;;
    esac
done
# Empty arrays and `set -u` disagree in bash 3.2, which is the bash macOS
# ships and the one `#!/bin/bash` picks there. Expanding an empty array is an
# unbound variable to it, so the array is only expanded when it has something
# in it - which is exactly the no-arguments case the usage text is for.
set -- ${POSITIONAL[@]+"${POSITIONAL[@]}"}

# --- Validate arguments ---
#
# After the options are parsed, not before. This read ${#POSITIONAL[@]} above
# the loop that declares it, so `set -u` stopped the script on its own usage
# check: every run of it, with arguments or without, ended at
# "POSITIONAL: unbound variable" and nothing was ever extracted.
if [ $# -lt 1 ]; then
    echo "Usage: $0 /path/to/WoW/Data [classic|turtle|tbc|wotlk] [--upscale[=N]]"
    echo ""
    echo "Point this at your WoW client's Data directory."
    echo "The expansion is auto-detected if not specified."
    exit 1
fi

MPQ_DIR="$1"
EXPANSION="${2:-auto}"

if [ ! -d "$MPQ_DIR" ]; then
    echo "Error: Directory not found: $MPQ_DIR"
    exit 1
fi

# If CSV DBCs are already present, extracting binary DBCs is optional.
# Users still need to extract visual assets from their own MPQs.
if [ -d "${OUTPUT_DIR}/expansions" ]; then
    if [ "$EXPANSION" != "auto" ] && [ -d "${OUTPUT_DIR}/expansions/${EXPANSION}/db" ]; then
        if ls "${OUTPUT_DIR}/expansions/${EXPANSION}/db"/*.csv >/dev/null 2>&1; then
            echo "Note: Found CSV DBCs in ${OUTPUT_DIR}/expansions/${EXPANSION}/db/"
            echo "      DBC extraction is optional; visual assets are still required."
            echo ""
        fi
    elif ls "${OUTPUT_DIR}"/expansions/*/db/*.csv >/dev/null 2>&1; then
        echo "Note: Found CSV DBCs under ${OUTPUT_DIR}/expansions/*/db/"
        echo "      DBC extraction is optional; visual assets are still required."
        echo ""
    fi
fi

# Quick sanity check: look for any .MPQ files
if ! compgen -G "$MPQ_DIR"/*.MPQ > /dev/null 2>&1 && ! compgen -G "$MPQ_DIR"/*.mpq > /dev/null 2>&1; then
    echo "Error: No .MPQ files found in: $MPQ_DIR"
    echo "Make sure this is the WoW Data/ directory (not the WoW root)."
    exit 1
fi

# --- Build asset_extract if needed ---
if [ ! -f "$BINARY" ]; then
    # --- Check for StormLib (only required to build) ---
    STORMLIB_FOUND=false
    # The library is libstorm; a source build lands in /usr/local, which
    # Fedora's loader cache does not list.
    if { ldconfig -p 2>/dev/null || /sbin/ldconfig -p 2>/dev/null; } | grep -qiE 'lib(storm|stormlib)\.'; then
        STORMLIB_FOUND=true
    elif compgen -G "/usr/local/lib*/libstorm*" > /dev/null 2>&1; then
        STORMLIB_FOUND=true
    elif pkg-config --exists stormlib 2>/dev/null || pkg-config --exists storm 2>/dev/null; then
        STORMLIB_FOUND=true
    elif [ -f "$(brew --prefix 2>/dev/null)/lib/libstorm.dylib" ] 2>/dev/null; then
        STORMLIB_FOUND=true
    fi
    if [ "$STORMLIB_FOUND" = false ]; then
        echo "Error: StormLib not found."
        echo "  Ubuntu/Debian: sudo apt install libstorm-dev"
        echo "  Fedora/Nobara: build from source - see README, Fedora / Nobara"
        echo "  macOS:         brew install stormlib"
        echo "  From source:   https://github.com/ladislav-zezula/StormLib"
        exit 1
    fi

    echo "Building asset_extract..."
    if [ ! -d "$BUILD_DIR" ]; then
        CMAKE_EXTRA_ARGS=()
        # On macOS, Homebrew installs to a non-default prefix that CMake
        # can't find automatically - pass it explicitly.
        if command -v brew &>/dev/null; then
            BREW="$(brew --prefix)"
            CMAKE_EXTRA_ARGS+=(
                "-DCMAKE_PREFIX_PATH=$BREW"
                "-DOPENSSL_ROOT_DIR=$(brew --prefix openssl@3)"
            )
            export PKG_CONFIG_PATH="$BREW/lib/pkgconfig:$(brew --prefix ffmpeg)/lib/pkgconfig:$(brew --prefix openssl@3)/lib/pkgconfig:$(brew --prefix vulkan-loader 2>/dev/null)/lib/pkgconfig:$(brew --prefix shaderc 2>/dev/null)/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
        fi
        cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release "${CMAKE_EXTRA_ARGS[@]}"
    fi
    NPROC=$(nproc 2>/dev/null || sysctl -n hw.logicalcpu 2>/dev/null || echo 4)
    cmake --build "$BUILD_DIR" --target asset_extract -- -j"$NPROC"
    echo ""
fi

if [ ! -f "$BINARY" ]; then
    echo "Error: Failed to build asset_extract"
    exit 1
fi

# --- Run extraction ---
echo "Extracting assets from: $MPQ_DIR"
echo "Output directory:       $OUTPUT_DIR"
echo ""

EXTRA_ARGS=()
if [ "$EXPANSION" != "auto" ]; then
    EXTRA_ARGS+=(--expansion "$EXPANSION")
fi

if [ "$EXPANSION" != "auto" ] && [ -d "${OUTPUT_DIR}/expansions/${EXPANSION}/db" ] && ls "${OUTPUT_DIR}/expansions/${EXPANSION}/db"/*.csv >/dev/null 2>&1; then
    EXTRA_ARGS+=(--skip-dbc)
fi

"$BINARY" --mpq-dir "$MPQ_DIR" --output "$OUTPUT_DIR" --expansion-subdir "${EXTRA_ARGS[@]}"

echo ""
if [ "$EXPANSION" = "auto" ]; then
    echo "Done! Assets extracted under $OUTPUT_DIR/expansions/<detected-expansion>"
else
    echo "Done! Assets extracted to $OUTPUT_DIR/expansions/$EXPANSION"
fi

# --- Optional upscale pass ---
#
# Last, and over the finished tree: the pass reads each model's texture list to
# decide what belongs together, so it needs the models and the textures already
# on disk. It never fails the extraction - the assets are extracted either way,
# and the sidecars it writes can be regenerated or deleted at any time.
if [ -n "$UPSCALE_SCALE" ]; then
    echo ""
    UPSCALE_OK=true
    if ! command -v python3 >/dev/null 2>&1; then
        echo "Skipping upscale: python3 not found."
        UPSCALE_OK=false
    elif ! python3 -c "import numpy, PIL" >/dev/null 2>&1; then
        echo "Skipping upscale: the pass needs Pillow and NumPy."
        echo "  python3 -m pip install --user pillow numpy"
        UPSCALE_OK=false
    fi
    if [ "$UPSCALE_OK" = true ] && [ ! -x "${BUILD_DIR}/bin/blp_convert" ]; then
        if [ -d "$BUILD_DIR" ]; then
            echo "Building blp_convert (the upscale pass decodes BLPs with it)..."
            cmake --build "$BUILD_DIR" --target blp_convert || true
        fi
        if [ ! -x "${BUILD_DIR}/bin/blp_convert" ]; then
            echo "Skipping upscale: blp_convert is not built."
            echo "  cmake -S \"$SCRIPT_DIR\" -B \"$BUILD_DIR\" && cmake --build \"$BUILD_DIR\" --target blp_convert"
            UPSCALE_OK=false
        fi
    fi
    if [ "$UPSCALE_OK" = true ]; then
        # Never fatal: the assets are extracted either way, and every sidecar
        # the pass writes can be regenerated or deleted afterwards.
        python3 "${SCRIPT_DIR}/tools/upscale_textures.py" \
            --data-dir "$OUTPUT_DIR" --scale "$UPSCALE_SCALE" || true
    fi
else
    echo ""
    echo "Textures are as they shipped. To upscale the foliage (optional, reversible):"
    echo "  $0 \"$MPQ_DIR\" ${EXPANSION} --upscale"
    echo "  python3 tools/upscale_textures.py --data-dir \"$OUTPUT_DIR\" --clean   # to undo"
fi

echo "You can now run wowee."
