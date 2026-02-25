#!/bin/bash
set -euo pipefail

# Usage: ./extract_assets.sh /path/to/WoW/Data [expansion]
#
# Extracts WoW MPQ archives into the Data/ directory for use with wowee.
#
# Arguments:
#   $1  Path to WoW's Data directory (containing .MPQ files)
#   $2  Expansion hint: classic, turtle, tbc, wotlk (optional, auto-detected if omitted)
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

# --- Validate arguments ---
if [ $# -lt 1 ]; then
    echo "Usage: $0 /path/to/WoW/Data [classic|turtle|tbc|wotlk]"
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
    if ldconfig -p 2>/dev/null | grep -qi stormlib; then
        STORMLIB_FOUND=true
    elif pkg-config --exists stormlib 2>/dev/null; then
        STORMLIB_FOUND=true
    elif [ -f "$(brew --prefix 2>/dev/null)/lib/libstorm.dylib" ] 2>/dev/null; then
        STORMLIB_FOUND=true
    fi
    if [ "$STORMLIB_FOUND" = false ]; then
        echo "Error: StormLib not found."
        echo "  Ubuntu/Debian: sudo apt install libstorm-dev"
        echo "  macOS:         brew install stormlib"
        echo "  From source:   https://github.com/ladislav-zezula/StormLib"
        exit 1
    fi

    echo "Building asset_extract..."
    if [ ! -d "$BUILD_DIR" ]; then
        CMAKE_EXTRA_ARGS=()
        # On macOS, Homebrew installs to a non-default prefix that CMake
        # can't find automatically — pass it explicitly.
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
elif ls "${OUTPUT_DIR}"/expansions/*/db/*.csv >/dev/null 2>&1; then
    EXTRA_ARGS+=(--skip-dbc)
fi

"$BINARY" --mpq-dir "$MPQ_DIR" --output "$OUTPUT_DIR" "${EXTRA_ARGS[@]}"

echo ""
echo "Done! Assets extracted to $OUTPUT_DIR"
echo "You can now run wowee."
