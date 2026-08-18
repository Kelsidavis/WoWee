#!/usr/bin/env bash
# Builds the Android dependencies CMake cannot fetch for itself.
#
# SDL2 comes down through FetchContent in CMakeLists; OpenSSL does not build
# with CMake, so it is built here once and handed to the client build with
# -DOPENSSL_ROOT_DIR, the same way the macOS build points at Homebrew's copy.
#
#   tools/build-android-deps.sh [abi] [api]
#
# Defaults to arm64-v8a on API 29. The result lands in build-android-deps/<abi>,
# which is gitignored.
set -euo pipefail

ABI="${1:-arm64-v8a}"
API="${2:-29}"
OPENSSL_VERSION="${OPENSSL_VERSION:-3.5.1}"

case "$ABI" in
    arm64-v8a)   OSSL_TARGET=android-arm64 ;;
    armeabi-v7a) OSSL_TARGET=android-arm ;;
    x86_64)      OSSL_TARGET=android-x86_64 ;;
    *) echo "unknown ABI: $ABI" >&2; exit 2 ;;
esac

NDK="${ANDROID_NDK_ROOT:-${ANDROID_NDK_HOME:-}}"
if [ -z "$NDK" ]; then
    # The newest side-by-side NDK under the SDK, if one is installed.
    NDK=$(ls -d "${ANDROID_HOME:-$HOME/Android/Sdk}"/ndk/* 2>/dev/null | sort -V | tail -1 || true)
fi
if [ -z "$NDK" ] || [ ! -d "$NDK" ]; then
    echo "No NDK. Set ANDROID_NDK_ROOT, or install one:" >&2
    echo "  sdkmanager 'ndk;28.2.13676358'" >&2
    exit 1
fi

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$ROOT/build-android-deps"
PREFIX="$WORK/$ABI"
mkdir -p "$WORK"

HOST_TAG=linux-x86_64
case "$(uname -s)" in Darwin) HOST_TAG=darwin-x86_64 ;; esac
export ANDROID_NDK_ROOT="$NDK"
export PATH="$NDK/toolchains/llvm/prebuilt/$HOST_TAG/bin:$PATH"

if [ -f "$PREFIX/lib/libcrypto.a" ]; then
    echo "OpenSSL already built: $PREFIX"
else
    TARBALL="$WORK/openssl-$OPENSSL_VERSION.tar.gz"
    SRC="$WORK/openssl-$OPENSSL_VERSION"
    [ -f "$TARBALL" ] || curl -fL --retry 3 -o "$TARBALL" \
        "https://github.com/openssl/openssl/releases/download/openssl-$OPENSSL_VERSION/openssl-$OPENSSL_VERSION.tar.gz"
    [ -d "$SRC" ] || tar -xzf "$TARBALL" -C "$WORK"

    # no-shared because the client links it statically and an Android APK that
    # carries one fewer .so is one fewer thing to package. no-tests and
    # no-apps cut most of the build.
    (cd "$SRC" && ./Configure "$OSSL_TARGET" no-shared no-tests no-apps \
        -D__ANDROID_API__="$API" --prefix="$PREFIX" --openssldir="$PREFIX/ssl")
    make -C "$SRC" -j"$(( $(nproc) / 2 ))"
    make -C "$SRC" install_sw
fi

echo
echo "Configure the client with:"
echo "  -DOPENSSL_ROOT_DIR=$PREFIX"
