FROM ubuntu:24.04

# Windows cross-compile using LLVM-MinGW - best-in-class Clang/LLD toolchain
# targeting x86_64-w64-mingw32. Produces native .exe/.dll without MSVC or Wine.
# LLVM-MinGW ships: clang, clang++, lld, libc++ / libunwind headers, winpthreads.

ENV DEBIAN_FRONTEND=noninteractive
ENV LLVM_MINGW_VERSION=20240619
ENV LLVM_MINGW_URL=https://github.com/mstorsjo/llvm-mingw/releases/download/${LLVM_MINGW_VERSION}/llvm-mingw-${LLVM_MINGW_VERSION}-ucrt-ubuntu-20.04-x86_64.tar.xz

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        build-essential \
        cmake \
        ninja-build \
        git \
        python3 \
        curl \
        zip \
        unzip \
        tar \
        xz-utils \
        pkg-config \
        nasm \
        libssl-dev \
        zlib1g-dev && \
    rm -rf /var/lib/apt/lists/*

# Install LLVM-MinGW toolchain
RUN curl -fsSL "${LLVM_MINGW_URL}" -o /tmp/llvm-mingw.tar.xz && \
    tar -xf /tmp/llvm-mingw.tar.xz -C /opt && \
    mv /opt/llvm-mingw-${LLVM_MINGW_VERSION}-ucrt-ubuntu-20.04-x86_64 /opt/llvm-mingw && \
    rm /tmp/llvm-mingw.tar.xz

ENV PATH="/opt/llvm-mingw/bin:${PATH}"

# Windows dependencies via vcpkg (static, x64-mingw-static triplet)
ENV VCPKG_ROOT=/opt/vcpkg
RUN git clone --depth 1 https://github.com/microsoft/vcpkg.git "${VCPKG_ROOT}" && \
    "${VCPKG_ROOT}/bootstrap-vcpkg.sh" -disableMetrics

ENV VCPKG_DEFAULT_TRIPLET=x64-mingw-static
RUN "${VCPKG_ROOT}/vcpkg" install \
        sdl3[vulkan] \
        openssl \
        glm \
        zlib \
        ffmpeg \
    --triplet x64-mingw-static

# Vulkan SDK headers (loader is linked statically via SDL3's vulkan surface)
RUN apt-get update && \
    apt-get install -y --no-install-recommends libvulkan-dev glslang-tools && \
    rm -rf /var/lib/apt/lists/*

# Provide a no-op powershell.exe so vcpkg's MinGW applocal post-build hook
# exits cleanly.  The x64-mingw-static triplet is fully static (no DLLs to
# copy), so the script has nothing to do - it just needs to not fail.
RUN printf '#!/bin/sh\nexit 0\n' > /usr/local/bin/powershell.exe && \
    chmod +x /usr/local/bin/powershell.exe

COPY build-windows.sh /build-platform.sh
RUN chmod +x /build-platform.sh

ENTRYPOINT ["/build-platform.sh"]
