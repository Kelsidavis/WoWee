FROM ubuntu:24.04 AS sdk-fetcher

# Stage 1: Fetch macOS SDK from Apple's public software update catalog.
# This avoids requiring the user to supply the SDK tarball manually.
# The SDK is downloaded, extracted, and packaged as a .tar.gz.

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        python3 \
        python3-defusedxml \
        cpio \
        tar \
        gzip \
        xz-utils && \
    rm -rf /var/lib/apt/lists/*

COPY macos/sdk-fetcher.py /opt/sdk-fetcher.py
RUN python3 /opt/sdk-fetcher.py /opt/sdk

# ---------------------------------------------------------------------------

FROM ubuntu:24.04 AS builder

# Stage 2: macOS cross-compile image using osxcross + Clang 18.
#
# Target triplets (auto-detected from osxcross):
#   arm64-apple-darwinNN  (Apple Silicon)
#   x86_64-apple-darwinNN (Intel)
# Default: arm64. Override with MACOS_ARCH=x86_64 env var at run time.

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        cmake \
        ninja-build \
        git \
        python3 \
        curl \
        wget \
        xz-utils \
        zip \
        unzip \
        tar \
        make \
        patch \
        libssl-dev \
        zlib1g-dev \
        pkg-config \
        libbz2-dev \
        libxml2-dev \
        libz-dev \
        liblzma-dev \
        uuid-dev \
        python3-lxml \
        gnupg \
        software-properties-common && \
    wget -qO- https://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add - && \
    echo "deb http://apt.llvm.org/noble/ llvm-toolchain-noble-18 main" > /etc/apt/sources.list.d/llvm-18.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        clang-18 \
        lld-18 \
        llvm-18 && \
    ln -sf /usr/bin/clang-18 /usr/bin/clang && \
    ln -sf /usr/bin/clang++-18 /usr/bin/clang++ && \
    ln -sf /usr/bin/lld-18 /usr/bin/lld && \
    ln -sf /usr/bin/ld.lld-18 /usr/bin/ld.lld && \
    ln -sf /usr/bin/llvm-ar-18 /usr/bin/llvm-ar && \
    rm -rf /var/lib/apt/lists/*

# Build osxcross with SDK from stage 1
RUN git clone --depth 1 https://github.com/tpoechtrager/osxcross.git /opt/osxcross

COPY --from=sdk-fetcher /opt/sdk/ /opt/osxcross/tarballs/

ENV MACOSX_DEPLOYMENT_TARGET=13.0
RUN cd /opt/osxcross && \
    UNATTENDED=1 ./build.sh && \
    rm -rf /opt/osxcross/build /opt/osxcross/tarballs

ENV PATH="/opt/osxcross/target/bin:${PATH}"
ENV OSXCROSS_TARGET_DIR="/opt/osxcross/target"
ENV MACOSX_DEPLOYMENT_TARGET=13.0

# Create unprefixed symlinks for macOS tools that vcpkg/CMake expect
RUN cd /opt/osxcross/target/bin && \
    for tool in install_name_tool otool lipo codesign; do \
      src="$(ls *-apple-darwin*-"${tool}" 2>/dev/null | head -1)"; \
      if [ -n "$src" ]; then \
        ln -sf "$src" "$tool"; \
      fi; \
    done

# Custom osxcross toolchain + vcpkg triplets
COPY macos/osxcross-toolchain.cmake /opt/osxcross-toolchain.cmake
COPY macos/triplets/ /opt/vcpkg-triplets/

# Extra tools needed by vcpkg's Mach-O rpath fixup and ffmpeg x86 asm
RUN apt-get update && \
    apt-get install -y --no-install-recommends file nasm && \
    rm -rf /var/lib/apt/lists/*

# vcpkg - macOS cross triplets (arm64-osx-cross / x64-osx-cross)
ENV VCPKG_ROOT=/opt/vcpkg
RUN git clone --depth 1 https://github.com/microsoft/vcpkg.git "${VCPKG_ROOT}" && \
    "${VCPKG_ROOT}/bootstrap-vcpkg.sh" -disableMetrics

# Pre-install deps for both arches; the launcher script picks the right one at run time.
RUN "${VCPKG_ROOT}/vcpkg" install \
        sdl3[vulkan] \
        openssl \
        glm \
        zlib \
        ffmpeg \
    --triplet arm64-osx-cross \
    --overlay-triplets=/opt/vcpkg-triplets

RUN "${VCPKG_ROOT}/vcpkg" install \
        sdl3[vulkan] \
        openssl \
        glm \
        zlib \
        ffmpeg \
    --triplet x64-osx-cross \
    --overlay-triplets=/opt/vcpkg-triplets

# Vulkan SDK headers (MoltenVK is the runtime - headers only needed to compile)
RUN apt-get update && \
    apt-get install -y --no-install-recommends libvulkan-dev glslang-tools && \
    rm -rf /var/lib/apt/lists/*

COPY build-macos.sh /build-platform.sh
RUN chmod +x /build-platform.sh

ENTRYPOINT ["/build-platform.sh"]
