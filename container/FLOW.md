# Container Build Flow

Comprehensive documentation of the Docker-based build pipeline for each target platform.

---

## Architecture Overview

Each platform follows the same two-phase pattern:

1. **Image Build** (one-time, cached by Docker) - installs compilers, toolchains, and pre-builds vcpkg dependencies.
2. **Container Run** (each build) - copies source into the container, runs CMake configure + build, outputs artifacts to the host.

```
Host                              Docker
─────────────────────────────────────────────────────────────
run-{platform}.sh/.ps1
  │
  ├─ docker build                 builder-{platform}.Dockerfile
  │    (cached after first run)     ├─ install compilers
  │                                 ├─ install vcpkg + packages
  │                                 └─ COPY build-{platform}.sh
  │
  └─ docker run                   build-{platform}.sh (entrypoint)
       ├─ bind /src (readonly)      ├─ tar copy source → /wowee-build-src
       └─ bind /out (writable)      ├─ git clone FidelityFX SDKs
                                    ├─ cmake -S . -B /out
                                    ├─ cmake --build /out
                                    └─ artifacts appear in /out
```

---

## Linux Build Flow

**Image:** `wowee-builder-linux`  
**Dockerfile:** `builder-linux.Dockerfile`  
**Toolchain:** GCC + Ninja (native amd64)  
**Base:** Ubuntu 24.04

### Docker Image Build Steps

| Step | What | Why |
|------|------|-----|
| 1 | `apt-get install` cmake, ninja-build, build-essential, pkg-config, git, python3 | Core build tools |
| 2 | `apt-get install` glslang-tools, spirv-tools | Vulkan shader compilation |
| 3 | `apt-get install` libglm-dev, libssl-dev, zlib1g-dev | Runtime dependencies (system packages) |
| 4 | `apt-get install` libavformat-dev, libavcodec-dev, libswscale-dev, libavutil-dev | FFmpeg libraries |
| 5 | `apt-get install` libvulkan-dev, vulkan-tools | Vulkan SDK |
| 6 | `apt-get install` libstorm-dev, libunicorn-dev | MPQ archive + CPU emulation |
| 7 | `apt-get install` libx11-dev, libxext-dev, libxkbcommon-dev, libwayland-dev, libdecor-0-dev and the other X11 headers | What SDL3 needs to build its video backends |
| 8 | Build SDL3 `release-3.2.24` from source and install it | Ubuntu 24.04 has no libsdl3-dev |
| 9 | COPY `build-linux.sh` → `/build-platform.sh` | Container entrypoint |

### Container Run Steps (build-linux.sh)

```
1. tar copy /src → /wowee-build-src (excludes build/, .git/, large Data/ dirs)
2. git clone FidelityFX-FSR2 (if missing)
3. git clone FidelityFX-SDK (if missing)
4. cmake configure:
     -G Ninja
     -DCMAKE_BUILD_TYPE=Release
     -DCMAKE_C_COMPILER=gcc
     -DCMAKE_CXX_COMPILER=g++
     -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON
5. cmake --build (parallel)
6. Create Data symlink: build/linux/bin/Data → ../../../Data
```

### Output
- `build/linux/bin/wowee` - ELF 64-bit x86-64 executable
- `build/linux/bin/Data` - symlink to project Data/ directory

---

## macOS Build Flow

**Image:** `wowee-builder-macos`  
**Dockerfile:** `builder-macos.Dockerfile` (multi-stage)  
**Toolchain:** osxcross (Clang 18 + Apple ld64)  
**Base:** Ubuntu 24.04  
**Targets:** arm64-apple-darwin24.5 (default), x86_64-apple-darwin24.5

### Docker Image Build - Stage 1: SDK Fetcher

The macOS SDK is fetched automatically from Apple's public software update catalog.
No manual download required.

| Step | What | Why |
|------|------|-----|
| 1 | `FROM ubuntu:24.04 AS sdk-fetcher` | Lightweight stage for SDK download |
| 2 | `apt-get install` ca-certificates, python3, cpio, tar, gzip, xz-utils | SDK extraction tools |
| 3 | COPY `macos/sdk-fetcher.py` → `/opt/sdk-fetcher.py` | Python script that scrapes Apple's SUCATALOG |
| 4 | `python3 /opt/sdk-fetcher.py /opt/sdk` | Downloads, extracts, and packages MacOSX15.5.sdk.tar.gz |

**SDK Fetcher internals** (`macos/sdk-fetcher.py`):
1. Queries Apple SUCATALOG URLs for the latest macOS package
2. Downloads the `CLTools_macOSNMOS_SDK.pkg` package
3. Extracts the XAR archive (using `bsdtar` or pure-Python fallback)
4. Decompresses the PBZX payload stream
5. Extracts via `cpio` to get the SDK directory
6. Packages as `MacOSX<version>.sdk.tar.gz`

### Docker Image Build - Stage 2: Builder

| Step | What | Why |
|------|------|-----|
| 1 | `FROM ubuntu:24.04 AS builder` | Full build environment |
| 2 | `apt-get install` cmake, ninja-build, git, python3, curl, wget, xz-utils, zip, unzip, tar, make, patch, libssl-dev, zlib1g-dev, pkg-config, libbz2-dev, libxml2-dev, uuid-dev | Build tools + osxcross build deps |
| 3 | Install Clang 18 from LLVM apt repo (`llvm-toolchain-jammy-18`) | Cross-compiler backend |
| 4 | Symlink clang-18 → clang, clang++-18 → clang++, etc. | osxcross expects unversioned names |
| 5 | `git clone osxcross` → `/opt/osxcross` | Apple cross-compile toolchain wrapper |
| 6 | `COPY --from=sdk-fetcher /opt/sdk/ → /opt/osxcross/tarballs/` | SDK from stage 1 |
| 7 | `UNATTENDED=1 ./build.sh` | Builds osxcross (LLVM wrappers + cctools + ld64) |
| 8 | Create unprefixed symlinks (install_name_tool, otool, lipo, codesign) | vcpkg/CMake need these without arch prefix |
| 9 | COPY `macos/osxcross-toolchain.cmake` → `/opt/osxcross-toolchain.cmake` | Auto-detecting CMake toolchain |
| 10 | COPY `macos/triplets/` → `/opt/vcpkg-triplets/` | vcpkg cross-compile triplet definitions |
| 11 | `apt-get install` file, nasm | Mach-O detection + ffmpeg x86 asm |
| 12 | Bootstrap vcpkg → `/opt/vcpkg` | Package manager |
| 13 | `vcpkg install` sdl3, openssl, glm, zlib, ffmpeg `--triplet arm64-osx-cross` | arm64 dependencies |
| 14 | `vcpkg install` same packages `--triplet x64-osx-cross` | x86_64 dependencies |
| 15 | `apt-get install` libvulkan-dev, glslang-tools | Vulkan headers (for compilation, not runtime) |
| 16 | COPY `build-macos.sh` → `/build-platform.sh` | Container entrypoint |

### Custom Toolchain Files

**`macos/osxcross-toolchain.cmake`** - Auto-detecting CMake toolchain:
- Detects SDK path via `file(GLOB)` in `/opt/osxcross/target/SDK/MacOSX*.sdk`
- Detects darwin version from compiler binary names (e.g., `arm64-apple-darwin24.5-clang`)
- Picks architecture from `CMAKE_OSX_ARCHITECTURES`
- Sets `CMAKE_C_COMPILER`, `CMAKE_CXX_COMPILER`, `CMAKE_AR`, `CMAKE_RANLIB`, `CMAKE_STRIP`

**`macos/triplets/arm64-osx-cross.cmake`**:
```cmake
set(VCPKG_TARGET_ARCHITECTURE arm64)
set(VCPKG_LIBRARY_LINKAGE static)
set(VCPKG_CMAKE_SYSTEM_NAME Darwin)
set(VCPKG_CHAINLOAD_TOOLCHAIN_FILE /opt/osxcross-toolchain.cmake)
```

### Container Run Steps (build-macos.sh)

```
1. Determine arch from MACOS_ARCH env (default: arm64)
2. Pick vcpkg triplet: arm64-osx-cross or x64-osx-cross
3. Auto-detect darwin target from osxcross binaries
4. tar copy /src → /wowee-build-src
5. git clone FidelityFX-FSR2 + FidelityFX-SDK (if missing)
6. cmake configure:
     -G Ninja
     -DCMAKE_BUILD_TYPE=Release
     -DCMAKE_SYSTEM_NAME=Darwin
     -DCMAKE_OSX_ARCHITECTURES=${ARCH}
     -DCMAKE_C_COMPILER=osxcross clang
     -DCMAKE_CXX_COMPILER=osxcross clang++
     -DCMAKE_TOOLCHAIN_FILE=vcpkg.cmake
     -DVCPKG_TARGET_TRIPLET=arm64-osx-cross
     -DVCPKG_OVERLAY_TRIPLETS=/opt/vcpkg-triplets
7. cmake --build (parallel)
```

### CMakeLists.txt Integration

The main CMakeLists.txt has a macOS cross-compile branch that:
- Finds Vulkan headers via vcpkg (`VulkanHeaders` package) instead of the Vulkan SDK
- Adds `-undefined dynamic_lookup` linker flag for Vulkan loader symbols (resolved at runtime via MoltenVK)

### Output
- `build/macos/bin/wowee` - Mach-O 64-bit arm64 (or x86_64) executable (~40 MB)

---

## Windows Build Flow

**Image:** `wowee-builder-windows`  
**Dockerfile:** `builder-windows.Dockerfile`  
**Toolchain:** LLVM-MinGW (Clang + LLD) targeting x86_64-w64-mingw32-ucrt  
**Base:** Ubuntu 24.04

### Docker Image Build Steps

| Step | What | Why |
|------|------|-----|
| 1 | `apt-get install` ca-certificates, build-essential, cmake, ninja-build, git, python3, curl, zip, unzip, tar, xz-utils, pkg-config, nasm, libssl-dev, zlib1g-dev | Build tools |
| 2 | Download + extract LLVM-MinGW (v20240619 ucrt) → `/opt/llvm-mingw` | Clang/LLD cross-compiler for Windows |
| 3 | Add `/opt/llvm-mingw/bin` to PATH | Makes `x86_64-w64-mingw32-clang` etc. available |
| 4 | Bootstrap vcpkg → `/opt/vcpkg` | Package manager |
| 5 | `vcpkg install` sdl3, openssl, glm, zlib, ffmpeg `--triplet x64-mingw-static` | Static Windows dependencies |
| 6 | `apt-get install` libvulkan-dev, glslang-tools | Vulkan headers + shader tools |
| 7 | Create no-op `powershell.exe` stub | vcpkg MinGW post-build hook needs it |
| 8 | COPY `build-windows.sh` → `/build-platform.sh` | Container entrypoint |

### Container Run Steps (build-windows.sh)

```
1. Set up no-op powershell.exe (if not already present)
2. tar copy /src → /wowee-build-src
3. git clone FidelityFX-FSR2 + FidelityFX-SDK (if missing)
4. Generate Vulkan import library:
     a. Extract vk* symbols from vulkan_core.h
     b. Create vulkan-1.def file
     c. Run dlltool to create libvulkan-1.a
5. Lock PKG_CONFIG_LIBDIR to vcpkg packages only
6. cmake configure:
     -G Ninja
     -DCMAKE_BUILD_TYPE=Release
     -DCMAKE_SYSTEM_NAME=Windows
     -DCMAKE_C_COMPILER=x86_64-w64-mingw32-clang
     -DCMAKE_CXX_COMPILER=x86_64-w64-mingw32-clang++
     -DCMAKE_RC_COMPILER=x86_64-w64-mingw32-windres
     -DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=lld
     -DCMAKE_TOOLCHAIN_FILE=vcpkg.cmake
     -DVCPKG_TARGET_TRIPLET=x64-mingw-static
     -DVCPKG_APPLOCAL_DEPS=OFF
7. cmake --build (parallel)
```

### Vulkan Import Library Generation

Windows applications link against `vulkan-1.dll` (the Khronos Vulkan loader). Since the LLVM-MinGW toolchain doesn't ship a Vulkan import library, the build script generates one:

1. Parses `vulkan_core.h` for `VKAPI_CALL vk*` function names
2. Creates a `.def` file mapping symbols to `vulkan-1.dll`
3. Uses `dlltool` to produce `libvulkan-1.a` (PE import library)

This allows the linker to resolve Vulkan symbols at build time, while deferring actual loading to the runtime DLL.

### Output
- `build/windows/bin/wowee.exe` - PE32+ x86-64 executable (~135 MB)

---

## Shared Patterns

### Source Tree Copy

All three platforms use the same tar-based copy with exclusions:
```bash
tar -C /src \
    --exclude='./build' --exclude='./logs' --exclude='./cache' \
    --exclude='./container' --exclude='./.git' \
    --exclude='./Data/character' --exclude='./Data/creature' \
    --exclude='./Data/db' --exclude='./Data/environment' \
    --exclude='./Data/interface' --exclude='./Data/item' \
    --exclude='./Data/misc' --exclude='./Data/sound' \
    --exclude='./Data/spell' --exclude='./Data/terrain' \
    --exclude='./Data/world' \
    -cf - . | tar -C /wowee-build-src -xf -
```

**Kept:** `Data/opcodes/`, `Data/expansions/` (small, needed at build time for configuration).  
**Excluded:** Large game asset directories (character, creature, environment, etc.) not needed for compilation.

### FidelityFX SDK Fetch

All platforms clone the same two repos at build time:
1. **FidelityFX-FSR2** - FSR 2.0 upscaling
2. **FidelityFX-SDK** - FSR 3.0 frame generation (repo URL/ref configurable via env vars)

### .dockerignore

The `.dockerignore` at the project root minimizes the Docker build context by excluding:
- `build/`, `cache/`, `logs/`, `.git/`
- Large external dirs (`extern/FidelityFX-*`)
- IDE files, documentation, host-only scripts
- SDK tarballs (`*.tar.xz`, `*.tar.gz`, etc.)

---

## Timing Estimates

These are approximate times on a 4-core machine with 16 GB RAM:

| Phase | Linux | macOS | Windows |
|-------|-------|-------|---------|
| Docker image build (first time) | ~5 min | ~25 min | ~15 min |
| Docker image build (cached) | seconds | seconds | seconds |
| Source copy + SDK fetch | ~10 sec | ~10 sec | ~10 sec |
| CMake configure | ~20 sec | ~30 sec | ~30 sec |
| Compilation | ~8 min | ~8 min | ~8 min |
| **Total (first build)** | **~14 min** | **~34 min** | **~24 min** |
| **Total (subsequent)** | **~9 min** | **~9 min** | **~9 min** |

macOS image is slowest because osxcross builds a subset of LLVM + cctools, and vcpkg packages are compiled for two architectures (arm64 + x64).
