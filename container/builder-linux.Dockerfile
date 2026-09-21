FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        cmake \
        ninja-build \
        build-essential \
        pkg-config \
        git \
        python3 \
        glslang-tools \
        spirv-tools \
        libglm-dev \
        libssl-dev \
        zlib1g-dev \
        libavformat-dev \
        libavcodec-dev \
        libswscale-dev \
        libavutil-dev \
        libvulkan-dev \
        vulkan-tools \
        libstorm-dev \
        libunicorn-dev \
        libx11-dev \
        libxext-dev \
        libxrandr-dev \
        libxcursor-dev \
        libxi-dev \
        libxfixes-dev \
        libxss-dev \
        libxkbcommon-dev \
        libwayland-dev \
        wayland-protocols \
        libdecor-0-dev && \
    rm -rf /var/lib/apt/lists/*

# SDL3 from source: Ubuntu 24.04 has no libsdl3-dev, which landed in 25.04.
# The same release CI builds, and the X11 and Wayland headers above are what
# it needs to find a video backend.
RUN git clone --depth 1 --branch release-3.2.24 \
        https://github.com/libsdl-org/SDL.git /tmp/SDL3 && \
    cmake -S /tmp/SDL3 -B /tmp/SDL3/build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DSDL_SHARED=ON -DSDL_STATIC=OFF -DSDL_TEST_LIBRARY=OFF && \
    cmake --build /tmp/SDL3/build && \
    cmake --install /tmp/SDL3/build && \
    ldconfig && \
    rm -rf /tmp/SDL3

COPY build-linux.sh /build-platform.sh
RUN chmod +x /build-platform.sh

ENTRYPOINT ["/build-platform.sh"]
