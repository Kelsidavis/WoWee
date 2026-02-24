# WoWee Build Instructions

This document provides platform-specific build instructions for WoWee.

---

## 🐧 Linux (Ubuntu / Debian)

### Install Dependencies

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config git \
  libsdl2-dev libglew-dev libglm-dev \
  libssl-dev zlib1g-dev \
  libvulkan-dev vulkan-tools glslc \
  libavcodec-dev libavformat-dev libavutil-dev libswscale-dev \
  libunicorn-dev libstorm-dev libx11-dev
```

---

## 🐧 Linux (Arch)

### Install Dependencies

```bash
sudo pacman -S --needed \
  base-devel cmake pkgconf git \
  sdl2 glew glm openssl zlib \
  vulkan-devel vulkan-tools shaderc \
  ffmpeg unicorn stormlib
```

---

## 🐧 Linux (All Distros)

### Clone Repository

Always clone with submodules:

```bash
git clone --recurse-submodules https://github.com/Kelsidavis/WoWee.git
cd WoWee
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

### Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

### Asset Extraction (Linux)

After building, extract assets from your WoW client:

```bash
./extract_assets.sh /path/to/WoW/Data wotlk
```

Supports `classic`, `tbc`, `wotlk` targets (auto-detected if omitted).

---

## 🍎 macOS

### Install Dependencies

Vulkan on macOS is provided via MoltenVK (a Vulkan-to-Metal translation layer),
which is included in the `vulkan-loader` Homebrew package.

```bash
brew install cmake pkg-config sdl2 glew glm openssl@3 zlib ffmpeg unicorn \
  stormlib vulkan-loader vulkan-headers shaderc
```

Optional (for creating redistributable `.app` bundles):

```bash
brew install dylibbundler
```

### Clone & Build

```bash
git clone --recurse-submodules https://github.com/Kelsidavis/WoWee.git
cd WoWee

BREW=$(brew --prefix)
export PKG_CONFIG_PATH="$BREW/lib/pkgconfig:$(brew --prefix ffmpeg)/lib/pkgconfig:$(brew --prefix openssl@3)/lib/pkgconfig:$(brew --prefix vulkan-loader)/lib/pkgconfig:$(brew --prefix shaderc)/lib/pkgconfig"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$BREW" \
  -DOPENSSL_ROOT_DIR="$(brew --prefix openssl@3)"
cmake --build build -j"$(sysctl -n hw.logicalcpu)"
```

### Asset Extraction (macOS)

The script will auto-build `asset_extract` if needed (requires `stormlib`).
It automatically detects Homebrew and passes the correct paths to CMake.

```bash
./extract_assets.sh /path/to/WoW/Data wotlk
```

Supports `classic`, `tbc`, `wotlk` targets (auto-detected if omitted).

---

## 🪟 Windows (MSYS2 — Recommended)

MSYS2 provides all dependencies as pre-built packages.

### Install MSYS2

Download and install from <https://www.msys2.org/>, then open a **MINGW64** shell.

### Install Dependencies

```bash
pacman -S --needed \
  mingw-w64-x86_64-cmake \
  mingw-w64-x86_64-gcc \
  mingw-w64-x86_64-ninja \
  mingw-w64-x86_64-pkgconf \
  mingw-w64-x86_64-SDL2 \
  mingw-w64-x86_64-glew \
  mingw-w64-x86_64-glm \
  mingw-w64-x86_64-openssl \
  mingw-w64-x86_64-zlib \
  mingw-w64-x86_64-ffmpeg \
  mingw-w64-x86_64-unicorn \
  mingw-w64-x86_64-vulkan-loader \
  mingw-w64-x86_64-vulkan-headers \
  mingw-w64-x86_64-shaderc \
  mingw-w64-x86_64-stormlib \
  git
```

### Clone & Build

```bash
git clone --recurse-submodules https://github.com/Kelsidavis/WoWee.git
cd WoWee
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel $(nproc)
```

---

## 🪟 Windows (Visual Studio 2022)

For users who prefer Visual Studio over MSYS2.

### Install

- Visual Studio 2022 with **Desktop development with C++** workload
- CMake tools for Windows (included in VS workload)
- [LunarG Vulkan SDK](https://vulkan.lunarg.com/) (provides Vulkan headers, loader, and glslc)

### vcpkg Dependencies

```powershell
vcpkg install sdl2 glew glm openssl zlib ffmpeg stormlib --triplet x64-windows
```

### Clone

```powershell
git clone --recurse-submodules https://github.com/Kelsidavis/WoWee.git
cd WoWee
```

### Build

Open the folder in Visual Studio (it will detect CMake automatically)
or build from Developer PowerShell:

```powershell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="[vcpkg root]/scripts/buildsystems/vcpkg.cmake"
cmake --build build --config Release
```

---

## 🪟 Asset Extraction (Windows)

After building (via either MSYS2 or Visual Studio), extract assets from your WoW client:

```powershell
.\extract_assets.ps1 "C:\Games\WoW-3.3.5a\Data"
```

Or double-click `extract_assets.bat` and provide the path when prompted.
You can also specify an expansion: `.\extract_assets.ps1 "C:\Games\WoW\Data" wotlk`

---

## ⚠️ Notes

- Case matters on Linux (`WoWee` not `wowee`).
- Always use `--recurse-submodules` when cloning.
- If you encounter missing headers for ImGui, run:
  ```bash
  git submodule update --init --recursive
  ```
