#!/bin/bash
# Linux amd64 build entrypoint - runs INSIDE the linux container.
# Mounts:
#   /src   (ro)     - project source
#   /out   (rw)     - host ./build/linux; receives the AppImage and nothing else
#   /build (volume) - the CMake build tree, kept between runs so the next one
#                     compiles only what changed

set -euo pipefail

SRC=/src
OUT=/out
BUILD=/build
NPROC=$(nproc)

# Commit the source checkout records for a submodule path; empty when /src is not a
# git checkout. The copied tree drops .git, so ask the bind-mounted source.
recorded_commit() {
    git -c safe.directory='*' -C "${SRC}" rev-parse -q --verify "HEAD:$1" 2>/dev/null || true
}

# Shallow-fetch one ref (branch, tag or commit SHA) of a repository into a directory.
fetch_at_ref() {
    local repo="$1" ref="$2" dir="$3"
    git init -q "$dir" &&
        git -C "$dir" fetch -q --depth 1 "$repo" "$ref" &&
        git -C "$dir" checkout -q --detach FETCH_HEAD
}

echo "==> [linux] Copying source tree..."
mkdir -p /wowee-build-src
tar -C "${SRC}" \
    --exclude='./build' --exclude='./logs' --exclude='./cache' \
    --exclude='./container' --exclude='./.git' \
    --exclude='./Data/character' --exclude='./Data/creature' \
    --exclude='./Data/db' --exclude='./Data/environment' \
    --exclude='./Data/interface' --exclude='./Data/item' \
    --exclude='./Data/misc' --exclude='./Data/sound' \
    --exclude='./Data/spell' --exclude='./Data/terrain' \
    --exclude='./Data/world' \
    -cf - . | tar -C /wowee-build-src -xf -

cd /wowee-build-src

echo "==> [linux] Fetching external SDKs (if needed)..."
if [ ! -f extern/FidelityFX-FSR2/src/ffx-fsr2-api/ffx_fsr2.h ]; then
    FSR2_REF="$(recorded_commit extern/FidelityFX-FSR2)"
    fetch_at_ref https://github.com/GPUOpen-Effects/FidelityFX-FSR2.git \
        "${FSR2_REF:-master}" extern/FidelityFX-FSR2 || echo "Warning: FSR2 clone failed - continuing without FSR2"
fi

SDK_REPO="${WOWEE_FFX_SDK_REPO:-https://github.com/Kelsidavis/FidelityFX-SDK.git}"
SDK_REF="${WOWEE_FFX_SDK_REF:-$(recorded_commit extern/FidelityFX-SDK)}"
SDK_REF="${SDK_REF:-main}"
if [ ! -f "extern/FidelityFX-SDK/Kits/FidelityFX/framegeneration/fsr3/include/ffx_frameinterpolation.h" ]; then
    fetch_at_ref "${SDK_REPO}" "${SDK_REF}" extern/FidelityFX-SDK \
        || echo "Warning: FidelityFX-SDK clone failed - continuing without FSR3"
fi

# WOWEE_BUILD_TESTS=OFF to not compile tests as they aren't needed here
echo "==> [linux] Configuring with CMake (Release, Ninja, amd64)..."
cmake -S . -B "${BUILD}" \
    -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_COMPILER=gcc \
    -DCMAKE_CXX_COMPILER=g++ \
    -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON \
    -DWOWEE_BUILD_TESTS=OFF

echo "==> [linux] Building with ${NPROC} cores..."
cmake --build "${BUILD}" --parallel "${NPROC}"

echo "==> [linux] Packaging the AppImage..."
# Run from the mounted checkout: the copy being built here has neither .git nor container/.
"${SRC}/container/appimage/build-appimage.sh" "${BUILD}"

# Replace whatever an earlier run left in /out with this build's AppImage.
rm -f "${OUT}"/wowee-*.AppImage
install -m 755 "${BUILD}"/wowee-*.AppImage "${OUT}/"

echo ""
echo "==> [linux] Build complete."
ls -1 "${OUT}"/wowee-*.AppImage
