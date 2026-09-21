# WoWee Build Instructions

This document provides platform-specific build instructions for WoWee.

The desktop client is built on **SDL3** and needs a **Vulkan 1.3** driver at
runtime (MoltenVK on macOS). A device that reports only Vulkan 1.2 is refused at
startup, and the log lists every device the loader offered with its version.

---

## 🐧 Linux (Ubuntu / Debian)

### Install Dependencies

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config git \
  libglm-dev \
  libssl-dev zlib1g-dev \
  libvulkan-dev vulkan-tools glslc \
  libavcodec-dev libavformat-dev libavutil-dev libswscale-dev \
  libx11-dev
```

SDL3 is packaged as `libsdl3-dev` on Ubuntu 25.04 and later and on Debian 13:

```bash
sudo apt install -y libsdl3-dev
```

Ubuntu 24.04 has no SDL3 package. Build it from source the way CI does, with
the X11 and Wayland headers it builds against:

```bash
sudo apt install -y \
  libxext-dev libxrandr-dev libxcursor-dev libxi-dev libxfixes-dev \
  libxss-dev libxkbcommon-dev libwayland-dev wayland-protocols libdecor-0-dev

git clone --depth 1 --branch release-3.2.24 https://github.com/libsdl-org/SDL.git /tmp/SDL3
cmake -S /tmp/SDL3 -B /tmp/SDL3/build -DCMAKE_BUILD_TYPE=Release \
  -DSDL_SHARED=ON -DSDL_STATIC=OFF -DSDL_TEST_LIBRARY=OFF
cmake --build /tmp/SDL3/build --parallel "$(nproc)"
sudo cmake --install /tmp/SDL3/build
sudo ldconfig
```

Optional features:

```bash
sudo apt install -y libunicorn-dev  # Warden execution
sudo apt install -y libstorm-dev    # asset tools (see Asset Extraction below)
```

---

## 🐧 Linux (Arch)

### Install Dependencies

```bash
sudo pacman -S --needed \
  base-devel cmake pkgconf git \
  sdl3 glm openssl zlib libx11 \
  vulkan-headers vulkan-icd-loader vulkan-tools shaderc \
  ffmpeg
```

> **Note:** `vulkan-headers` provides the `vulkan/vulkan.h` development headers required
> at build time. `vulkan-devel` is a group that includes these on some distros but is not
> available by name on Arch - install `vulkan-headers` and `vulkan-icd-loader` explicitly.

Install `unicorn` for optional Warden execution. StormLib is available from the
AUR as `stormlib-git` if you need the asset tools.

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

The client needs assets built from your own World of Warcraft installation.
With StormLib installed the build produces three ways to do it, all driving the
same extractor:

- **The client itself.** Started with nothing extracted, it opens the asset
  builder instead of the login screen. Once assets exist, **more options** →
  **add or rebuild assets...** on the login screen opens it again.
- **The asset manager window**, `build/bin/wowee_assets`. It takes the game
  folder, a later client and the destination as optional arguments
  (`wowee_assets [game folder] [later client] [destination]`); see
  [`docs/asset-manager.md`](docs/asset-manager.md).
- **The script**, for the command line:

  ```bash
  ./extract_assets.sh /path/to/WoW/Data wotlk
  ```

  Supports `classic`, `turtle`, `tbc`, `wotlk` targets (auto-detected if omitted).

The builder and `wowee_assets` write to the per-user data directory by
default, `$XDG_DATA_HOME/wowee/Data` (or `~/.local/share/wowee/Data`). The
script writes to `Data/` in the checkout, which `build.sh` links into
`build/bin`. At startup the client uses `WOW_DATA_PATH` if it is set, then the
per-user directory if it holds an extraction, then `Data/` in its own directory.

---

## 🍎 macOS

### Install Dependencies

Vulkan on macOS uses the Vulkan loader plus MoltenVK's Vulkan-to-Metal driver.

```bash
brew install cmake pkg-config sdl3 glm openssl@3 zlib ffmpeg \
  vulkan-loader vulkan-headers molten-vk shaderc
```

Optional features:

```bash
brew install unicorn  # Warden execution
brew install stormlib # asset tools (see Asset Extraction below)
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

### Minimum macOS version

The command above leaves `CMAKE_OSX_DEPLOYMENT_TARGET` unset, so the binary is
stamped with the build machine's own OS version and will not launch on anything
older. That is fine for a local build.

Release CI does not rely on that default — it pins `13.0` and bundles every
non-system dylib into the `.app` (`tools/macos/bundle_dependencies.py`), so
shipped DMGs are not limited to whatever the runner happened to be on. To
reproduce a release-like build locally, pass the same flag:

```bash
cmake -S . -B build -DCMAKE_OSX_DEPLOYMENT_TARGET=13.0 ...
```

Note that pinning below your Homebrew dylibs' own target makes `ld` warn on
every link (`building for macOS-13.0, but linking with dylib ... built for
newer version 26.0`). Those warnings are expected in a local pinned build:
Homebrew builds for the host OS, and it is the bundling step in CI, not the
flag, that makes the shipped app run on 13.0.

### Asset Extraction (macOS)

The same three routes as on Linux: the client opens the asset builder when
nothing is extracted (and from **more options** → **add or rebuild assets...**
afterwards), `build/bin/wowee_assets` is the standalone window, and the script
works from the command line. All three need `stormlib`.

The script will auto-build `asset_extract` if needed.
It automatically detects Homebrew and passes the correct paths to CMake.

```bash
./extract_assets.sh /path/to/WoW/Data wotlk
```

Supports `classic`, `turtle`, `tbc`, `wotlk` targets (auto-detected if omitted).

The builder and `wowee_assets` write to
`~/Library/Application Support/Wowee/Data` by default; the script writes to
`Data/` in the checkout.

### Running a downloaded macOS release

GitHub release DMGs are Developer ID signed, notarized by Apple, and stapled.
Gatekeeper should accept a release downloaded from the official WoWee GitHub
repository without an **Open Anyway** exception.

The DMG holds `Wowee.app` and `Wowee Asset Extractor.app`. `Wowee.app` carries
`asset_extract` and the `wowee_assets` window, and the extractor app opens that
window. Assets are written to `~/Library/Application Support/Wowee/Data`,
outside the signed bundle, so they survive an upgrade.

Maintainers can find the CI credential contract and verification commands in
[`docs/macos-distribution.md`](docs/macos-distribution.md).

---

## 🪟 Windows (MSYS2 - Recommended)

MSYS2 provides the normal client dependencies as pre-built packages. StormLib
is built separately only when the optional asset tools are needed.

### Install MSYS2

Download and install from <https://www.msys2.org/>, then open a **MINGW64** shell.

### Install Dependencies

```bash
pacman -S --needed \
  mingw-w64-x86_64-cmake \
  mingw-w64-x86_64-gcc \
  mingw-w64-x86_64-ninja \
  mingw-w64-x86_64-pkgconf \
  mingw-w64-x86_64-sdl3 \
  mingw-w64-x86_64-glm \
  mingw-w64-x86_64-openssl \
  mingw-w64-x86_64-zlib \
  mingw-w64-x86_64-ffmpeg \
  mingw-w64-x86_64-unicorn \
  mingw-w64-x86_64-vulkan-loader \
  mingw-w64-x86_64-vulkan-headers \
  mingw-w64-x86_64-shaderc \
  git
```

The normal client does not require StormLib. To build the asset tools
(`asset_extract`, `wowee_assets`, `mpq_build` and the client's built-in asset
builder), install the static-link dependencies and build StormLib using the
same configuration as CI:

```bash
pacman -S --needed \
  mingw-w64-x86_64-libtommath \
  mingw-w64-x86_64-libtomcrypt

git clone --depth 1 https://github.com/ladislav-zezula/StormLib.git /tmp/StormLib
cmake -S /tmp/StormLib -B /tmp/StormLib/build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$MINGW_PREFIX" \
  -DBUILD_SHARED_LIBS=OFF
cmake --build /tmp/StormLib/build --parallel
cmake --install /tmp/StormLib/build
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
vcpkg install "sdl3[vulkan]" glm openssl zlib ffmpeg stormlib --triplet x64-windows
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

With StormLib available, the client opens the asset builder when nothing is
extracted, and `wowee_assets.exe` in `build\bin` is the standalone window. Both
write to `%LOCALAPPDATA%\Wowee\Data` by default. Double-clicking `asset_extract.exe` hands
over to `wowee_assets.exe` when it sits beside it.

From the command line, after building (via either MSYS2 or Visual Studio):

```powershell
.\extract_assets.ps1 "C:\Games\WoW-3.3.5a\Data"
```

Or double-click `extract_assets.bat` and provide the path when prompted.
You can also specify an expansion: `.\extract_assets.ps1 "C:\Games\WoW\Data" wotlk`

The script writes to `Data\` in the checkout.

---

## ⚙️ Build Options

Pass these to the configure step as `-D<option>=ON|OFF`.

| Option | Default | What it does |
|---|---|---|
| `WOWEE_BUILD_TESTS` | `ON` | Builds the Catch2 unit tests (see [TESTING.md](TESTING.md)) |
| `WOWEE_WARNINGS_AS_ERRORS` | `ON` | Treats compiler warnings as errors in the client |
| `WOWEE_ENABLE_ASAN` | `OFF` | AddressSanitizer + UBSan on the client and the tests |
| `WOWEE_SHADER_DEBUG_INFO` | `OFF` | Compiles shaders unoptimised with debug info, so GPU validation can name a line. Rewrites the tracked `.spv` files in place |
| `WOWEE_BUILD_EDITOR` | `OFF` | Adds the standalone world editor to the default build. Without it, `cmake --build build --target wowee_editor` still builds it on demand |
| `WOWEE_BUILD_FRAMEXML_RUN` | `OFF` | Builds `framexml_run`, the headless FrameXML runner (see [TESTING.md](TESTING.md)). Compiles the client a second time |
| `WOWEE_ENABLE_AMD_FSR2` | `OFF` | AMD FidelityFX FSR2 SDK backend, when the SDK is under `extern/` |
| `WOWEE_ENABLE_AMD_FSR3_FRAMEGEN` | `OFF` | AMD FidelityFX SDK FSR3 frame generation probe, when the SDK is under `extern/` |
| `WOWEE_BUILD_AMD_FSR3_RUNTIME` | `OFF` | Builds AMD's native FidelityFX Vulkan runtime from `extern/FidelityFX-SDK/Kits` |

---

## ⚠️ Notes

- Case matters on Linux (`WoWee` not `wowee`).
- Always use `--recurse-submodules` when cloning.
- If you encounter missing headers for ImGui, run:
  ```bash
  git submodule update --init --recursive
  ```
- The client's own upscaling (FSR 1 and the FSR 3 temporal upscaler) is
  in-tree and needs no external SDK.
- The AMD FidelityFX backends are off by default and are compiled only with
  the `WOWEE_ENABLE_AMD_*` options above. `build.sh` / `rebuild.sh` /
  `build.ps1` / `rebuild.ps1` still fetch both SDKs when they are missing:
  - `https://github.com/GPUOpen-Effects/FidelityFX-FSR2.git` into `extern/FidelityFX-FSR2`
  - `https://github.com/Kelsidavis/FidelityFX-SDK.git` into `extern/FidelityFX-SDK`
    (`WOWEE_FFX_SDK_REPO` and `WOWEE_FFX_SDK_REF` choose another repository or ref)
- With `WOWEE_ENABLE_AMD_FSR2=ON`, if the SDK checkout is missing its generated
  Vulkan permutation headers, CMake bootstraps them from:
  - `third_party/fsr2_vk_permutations`
- With `WOWEE_ENABLE_AMD_FSR2=ON` and no SDK headers, CMake warns and the
  build uses the internal FSR2 fallback path.
