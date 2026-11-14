FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# ca-certificates: git is installed without its recommended packages, and with
# no CA bundle the https clones below and in build-linux.sh fail certificate
# verification
# curl: container/appimage/build-appimage.sh downloads linuxdeploy and
# appimagetool with it
# file: needed by linuxdeploy
# glslc: CMake compiles the shaders with it and with nothing else; without it
# only the committed .spv files exist, and eight shaders have none
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        file \
        cmake \
        ninja-build \
        build-essential \
        pkg-config \
        git \
        python3 \
        glslc \
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
# Installed as a .deb rather than with a bare `cmake --install`: linuxdeploy
# asks dpkg which package owns each library it bundles and copies that
# package's licence, and finds none for a file nothing owns.
RUN SDL_TAG=release-3.2.24 && \
    git clone --depth 1 --branch "${SDL_TAG}" \
        https://github.com/libsdl-org/SDL.git /tmp/SDL3 && \
    cmake -S /tmp/SDL3 -B /tmp/SDL3/build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DSDL_SHARED=ON -DSDL_STATIC=OFF -DSDL_TEST_LIBRARY=OFF && \
    cmake --build /tmp/SDL3/build && \
    DESTDIR=/tmp/sdl3-pkg cmake --install /tmp/SDL3/build && \
    install -D -m 644 /tmp/SDL3/LICENSE.txt \
        /tmp/sdl3-pkg/usr/share/doc/sdl3-local/copyright && \
    install -d /tmp/sdl3-pkg/DEBIAN && \
    printf 'Package: sdl3-local\nVersion: %s\nArchitecture: %s\nMaintainer: local build <root@localhost>\nDescription: SDL3 built from source\n' \
        "${SDL_TAG#release-}" "$(dpkg --print-architecture)" \
        > /tmp/sdl3-pkg/DEBIAN/control && \
    dpkg-deb --root-owner-group --build /tmp/sdl3-pkg /tmp/sdl3-local.deb && \
    dpkg -i /tmp/sdl3-local.deb && \
    ldconfig && \
    rm -rf /tmp/SDL3 /tmp/sdl3-pkg /tmp/sdl3-local.deb

COPY build-linux.sh /build-platform.sh
RUN chmod +x /build-platform.sh

ENTRYPOINT ["/build-platform.sh"]
