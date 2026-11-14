#!/usr/bin/env bash
# Build a Wowee AppImage from an existing CMake build tree.
#
#   container/appimage/build-appimage.sh [build-dir]      # default: ./build
#
# Output: <build-dir>/wowee-<version>-<arch>.AppImage (any older one there is removed).
# The downloaded tools and the staging directory stay in <build-dir>/appimage/.
# The container build (container/run-linux.sh) and the CI workflows run this script.
#
# Staged beside the executable, where the client looks: the client, the asset
# tools if they were built, assets/ without the soundtrack, the tracked Data/
# files, the bundled addons and the optional AMD FSR3 runtime.
#
# Not in it: the game data (extract it with the asset tools into
# ~/.local/share/wowee/Data), and libvulkan, which has to match the GPU driver
# of the machine that runs the AppImage.
#
# The shared libraries the binary links (SDL3, ffmpeg, ...) are copied in, so it
# runs with the versions it was built against, not the host's. glibc is not
# bundled: build on the oldest distribution it has to run on.
#
# Environment overrides: OUT_DIR (the tools and staging directory), ARCH,
# VERSION, LINUXDEPLOY, APPIMAGETOOL, and GIT_ROOT, the git checkout the tracked Data/ files and the version come from
# when this script's own tree has no .git.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(cd "$HERE/../.." && pwd)"
BUILD_DIR="$(cd "${1:-$SRC_DIR/build}" && pwd)"
BIN_DIR="$BUILD_DIR/bin"
OUT_DIR="${OUT_DIR:-$BUILD_DIR/appimage}"
APPDIR="$OUT_DIR/AppDir"
ARCH="${ARCH:-$(uname -m)}"
GIT_ROOT="${GIT_ROOT:-$SRC_DIR}"

# safe.directory: the checkout may be a bind mount owned by another user.
git_root() { git -c safe.directory='*' -C "$GIT_ROOT" "$@"; }

VERSION="${VERSION:-$(git_root describe --tags --always --dirty 2>/dev/null || echo dev)}"

# Pinned, and checked against these hashes after download.
LINUXDEPLOY_TAG=1-alpha-20250213-2
APPIMAGETOOL_TAG=1.9.1
declare -A LINUXDEPLOY_SHA256=(
    [x86_64]=4648f278ab3ef31f819e67c30d50f462640e5365a77637d7e6f2ad9fd0b4522a
    [aarch64]=06706ac8189797dccd36bd384105892cb5e6e71f784f4df526cc958adc223cd6
)
declare -A APPIMAGETOOL_SHA256=(
    [x86_64]=ed4ce84f0d9caff66f50bcca6ff6f35aae54ce8135408b3fa33abfc3cb384eb0
    [aarch64]=f0837e7448a0c1e4e650a93bb3e85802546e60654ef287576f46c71c126a9158
)

if [ ! -x "$BIN_DIR/wowee" ]; then
    echo "error: $BIN_DIR/wowee not found - build the project first." >&2
    exit 1
fi
if [ -z "${LINUXDEPLOY_SHA256[$ARCH]:-}" ]; then
    echo "error: unsupported architecture '$ARCH' (x86_64 and aarch64 only)." >&2
    exit 1
fi

mkdir -p "$OUT_DIR"

# Download a tool once, verify it on every use, and print its path.
fetch_tool() {
    local name="$1" url="$2" sha="$3"
    local path="$OUT_DIR/$name"
    if [ -x "$path" ]; then
        echo "$sha  $path" | sha256sum -c - >&2 || return 1
    else
        echo "Downloading $name" >&2
        # Command substitutions do not inherit errexit by default. Propagate
        # failures explicitly so an unverified download cannot become a tool.
        curl -fsSL -o "$path.part" "$url" || return 1
        echo "$sha  $path.part" | sha256sum -c - >&2 || return 1
        chmod +x "$path.part" || return 1
        mv "$path.part" "$path" || return 1
    fi
    echo "$path"
}

LINUXDEPLOY="${LINUXDEPLOY:-$(fetch_tool "linuxdeploy-$ARCH.AppImage" \
    "https://github.com/linuxdeploy/linuxdeploy/releases/download/$LINUXDEPLOY_TAG/linuxdeploy-$ARCH.AppImage" \
    "${LINUXDEPLOY_SHA256[$ARCH]}")}"
APPIMAGETOOL="${APPIMAGETOOL:-$(fetch_tool "appimagetool-$ARCH.AppImage" \
    "https://github.com/AppImage/appimagetool/releases/download/$APPIMAGETOOL_TAG/appimagetool-$ARCH.AppImage" \
    "${APPIMAGETOOL_SHA256[$ARCH]}")}"

# The tools are AppImages themselves; unpack-and-run needs no FUSE.
export APPIMAGE_EXTRACT_AND_RUN=1

# ---- Stage the files the client expects beside its executable ----
rm -rf "$APPDIR"
mkdir -p "$APPDIR/usr/bin"
APP_BIN="$APPDIR/usr/bin"

install -m 755 "$BIN_DIR/wowee" "$APP_BIN/"
EXTRA_EXECUTABLES=()
for tool in asset_extract wowee_assets; do
    if [ -x "$BIN_DIR/$tool" ]; then
        install -m 755 "$BIN_DIR/$tool" "$APP_BIN/"
        EXTRA_EXECUTABLES+=(--executable "$APP_BIN/$tool")
    else
        echo "note: $tool was not built; the AppImage will not include it." >&2
    fi
done

# The optional AMD FSR3 runtime (WOWEE_BUILD_AMD_FSR3_RUNTIME). The client
# loads it from its own directory.
if [ -f "$BIN_DIR/libamd_fidelityfx_vk.so" ]; then
    install -m 755 "$BIN_DIR/libamd_fidelityfx_vk.so" "$APP_BIN/"
fi

# assets/ without the soundtrack.
cp -a "$BIN_DIR/assets" "$APP_BIN/assets"
rm -rf "$APP_BIN/assets/Original Music"

# Data/: only the tracked expansion profiles; a development tree can also hold
# a multi-gigabyte extraction there.
if git_root rev-parse --git-dir >/dev/null 2>&1; then
    git_root ls-files -z Data \
        | tar -C "$GIT_ROOT" --null -T - -cf - \
        | tar -C "$APP_BIN" -xf -
else
    echo "note: not a git checkout, copying all of Data/." >&2
    cp -a "$SRC_DIR/Data" "$APP_BIN/Data"
fi

if [ -d "$BIN_DIR/addons" ]; then
    cp -a "$BIN_DIR/addons" "$APP_BIN/addons"
else
    cp -a "$SRC_DIR/addons" "$APP_BIN/addons"
fi

# linuxdeploy names the icon after its file; the desktop file says "wowee".
cp "$SRC_DIR/assets/Wowee.png" "$OUT_DIR/wowee.png"

# ---- Bundle the shared libraries and lay out the AppDir ----
# linuxdeploy asks dpkg which package owns each file it deploys, copies that
# package's licence, and warns "Could not find copyright files" when there is
# none. That is always so for the client, its tools and the icon, whose licence
# is added below, so those warnings are filtered out; any other stays, such as
# one for a library no package owns (SDL3, when not installed as a package as
# the builder image does). awk rather than grep -v: awk always succeeds, so
# with pipefail the pipeline's status is linuxdeploy's.
echo "note: hiding linuxdeploy's 'Could not find copyright files' warnings for the client, its tools and the icon; their licence is added below." >&2
"$LINUXDEPLOY" --appdir "$APPDIR" \
    --executable "$APP_BIN/wowee" ${EXTRA_EXECUTABLES[@]+"${EXTRA_EXECUTABLES[@]}"} \
    --desktop-file "$HERE/wowee.desktop" \
    --icon-file "$OUT_DIR/wowee.png" \
    --custom-apprun "$HERE/AppRun" 2>&1 \
    | awk -v bin="Could not find copyright files for file $APPDIR/usr/bin/" \
          -v icon="Could not find copyright files for file $OUT_DIR/wowee.png" \
          'index($0, bin) == 0 && index($0, icon) == 0 { print; fflush() }'

# Licences for what no package supplied: this project's own, and SDL3's when it
# was not installed as a package, as in a native build (in the container
# linuxdeploy copied it).
DOC_DIR="$APPDIR/usr/share/doc"
install -d "$DOC_DIR/wowee"
install -m 644 "$SRC_DIR/LICENSE" "$SRC_DIR/NOTICE" "$DOC_DIR/wowee/"
if compgen -G "$APPDIR/usr/lib/libSDL3.so*" >/dev/null \
        && ! compgen -G "$DOC_DIR/*sdl3*/copyright" >/dev/null; then
    for sdl_license in /usr/local/share/licenses/SDL3/LICENSE.txt \
                       /usr/share/licenses/SDL3/LICENSE.txt; do
        if [ -f "$sdl_license" ]; then
            install -d "$DOC_DIR/SDL3"
            install -m 644 "$sdl_license" "$DOC_DIR/SDL3/"
            break
        fi
    done
    [ -d "$DOC_DIR/SDL3" ] || echo "note: SDL3 is bundled but its LICENSE.txt was not found." >&2
fi

# libvulkan comes from the host, to match its GPU driver and not this build
# machine's; the dynamic linker finds the system's copy.
rm -f "$APPDIR"/usr/lib/libvulkan.so*

# ---- Pack ----
OUTPUT="$BUILD_DIR/wowee-$VERSION-$ARCH.AppImage"
rm -f "$BUILD_DIR"/wowee-*.AppImage
ARCH="$ARCH" VERSION="$VERSION" "$APPIMAGETOOL" --no-appstream "$APPDIR" "$OUTPUT"
echo "Built $OUTPUT"
