#!/bin/bash

set -eu 
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

podman build \
	-f "${SCRIPT_DIR}/builder-ubuntu.Dockerfile" \
	-t wowee-builder-ubuntu

BUILD_DIR="$(mktemp --tmpdir -d wowee.XXXXX \
	--suffix=".$(cd "${PROJECT_ROOT}"; git rev-parse --short HEAD)")"
podman run \
	--mount "type=bind,src=${PROJECT_ROOT},dst=/WoWee-src,ro=true" \
	--mount "type=bind,src=${BUILD_DIR},dst=/build" \
	localhost/wowee-builder-ubuntu \
	./build-wowee.sh
