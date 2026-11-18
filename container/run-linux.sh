#!/usr/bin/env bash
# run-linux.sh - Build WoWee for Linux (amd64) inside a Docker container.
#
# The build leaves one file: build/linux/wowee-<version>-<arch>.AppImage.
#
# Usage (run from project root):
#   ./container/run-linux.sh [--rebuild-image]
#
# The CMake build tree is kept in the Docker volume wowee-linux-build, so a
# second run compiles only what changed. `docker volume rm wowee-linux-build`
# starts over.
#
# Environment variables:
#   WOWEE_FFX_SDK_REPO  - FidelityFX SDK git repo URL (passed through to container)
#   WOWEE_FFX_SDK_REF   - FidelityFX SDK git ref / tag      (passed through to container)
#   REBUILD_IMAGE       - Set to 1 to force a fresh docker build (same as --rebuild-image)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE_NAME="wowee-builder-linux"
BUILD_VOLUME="wowee-linux-build"
BUILD_OUTPUT="${PROJECT_ROOT}/build/linux"

# Revision label of the builder image: an image without the current one is
# rebuilt instead of run. Changing it rebuilds every existing image.
IMAGE_REV="appimage-1"

# Parse arguments
REBUILD_IMAGE="${REBUILD_IMAGE:-0}"
for arg in "$@"; do
  case "$arg" in
    --rebuild-image) REBUILD_IMAGE=1 ;;
    *) echo "Unknown argument: $arg" >&2; exit 1 ;;
  esac
done

# Verify Docker is available
if ! command -v docker &>/dev/null; then
  echo "Error: docker is not installed or not in PATH." >&2
  exit 1
fi

image_is_current() {
  docker image inspect --format '{{.Config.Labels}}' "$IMAGE_NAME" 2>/dev/null \
    | grep -q "wowee.builder.rev:${IMAGE_REV}"
}

# Build the image (skip if present, current, and --rebuild-image not given)
if [[ "$REBUILD_IMAGE" == "1" ]] || ! image_is_current; then
  echo "==> Building Docker image: ${IMAGE_NAME}"
  docker build \
    -f "${SCRIPT_DIR}/builder-linux.Dockerfile" \
    --label "wowee.builder.rev=${IMAGE_REV}" \
    -t "$IMAGE_NAME" \
    "${SCRIPT_DIR}"
else
  echo "==> Using existing Docker image: ${IMAGE_NAME}"
fi

# Create output directory on the host
mkdir -p "$BUILD_OUTPUT"

echo "==> Starting Linux build (output: ${BUILD_OUTPUT})"

docker run --rm \
  --mount "type=bind,src=${PROJECT_ROOT},dst=/src,readonly" \
  --mount "type=bind,src=${BUILD_OUTPUT},dst=/out" \
  --mount "type=volume,src=${BUILD_VOLUME},dst=/build" \
  ${WOWEE_FFX_SDK_REPO:+--env "WOWEE_FFX_SDK_REPO=${WOWEE_FFX_SDK_REPO}"} \
  ${WOWEE_FFX_SDK_REF:+--env "WOWEE_FFX_SDK_REF=${WOWEE_FFX_SDK_REF}"} \
  "$IMAGE_NAME"

# An unmatched pattern must give an empty array, not itself, for the check below.
shopt -s nullglob
APPIMAGES=("${BUILD_OUTPUT}"/wowee-*.AppImage)
if (( ${#APPIMAGES[@]} == 0 )); then
  echo "Error: the container finished but left no AppImage in ${BUILD_OUTPUT}." >&2
  exit 1
fi
echo "==> Linux build complete: ${APPIMAGES[*]}"
