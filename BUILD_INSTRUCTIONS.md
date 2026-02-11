# Build Instructions

This project builds as a native C++ client for WoW 3.3.5a in online mode.

## 1. Install Dependencies

### Ubuntu / Debian

```bash
sudo apt update
sudo apt install -y \
  cmake build-essential pkg-config git \
  libsdl2-dev libglew-dev libglm-dev \
  libssl-dev zlib1g-dev \
  libavformat-dev libavcodec-dev libswscale-dev libavutil-dev \
  libstorm-dev
```

If `libstorm-dev` is unavailable in your distro repos, build StormLib from source:

```bash
cd /tmp
git clone https://github.com/ladislav-zezula/StormLib.git
cd StormLib
mkdir build && cd build
cmake ..
make -j"$(nproc)"
sudo make install
sudo ldconfig
```

### Fedora

```bash
sudo dnf install -y \
  cmake gcc-c++ make pkg-config git \
  SDL2-devel glew-devel glm-devel \
  openssl-devel zlib-devel \
  ffmpeg-devel \
  StormLib-devel
```

### Arch

```bash
sudo pacman -S --needed \
  cmake base-devel pkgconf git \
  sdl2 glew glm openssl zlib ffmpeg stormlib
```

## 2. Clone + Prepare

```bash
git clone git@github.com:Kelsidavis/wowee.git
cd wowee
git clone https://github.com/ocornut/imgui.git extern/imgui
```

## 3. Configure + Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

Binary output:

```text
build/bin/wowee
```

## 4. Provide WoW 3.3.5a Data

Supply your own legally obtained data files in either:

1. `./Data/`
2. Path pointed to by `WOW_DATA_PATH`

Example:

```text
Data/
  common.MPQ
  common-2.MPQ
  expansion.MPQ
  lichking.MPQ
  patch.MPQ
  patch-2.MPQ
  patch-3.MPQ
  enUS/
```

## 5. Run

```bash
./build/bin/wowee
```

## 6. Local AzerothCore (Optional)

If you are using a local AzerothCore Docker stack, start it first and then connect from the client realm screen.

See:

- `docs/server-setup.md`

## Troubleshooting

### `StormLib` not found

Install distro package or build from source (section 1).

### `ImGui` missing

Ensure `extern/imgui` exists:

```bash
git clone https://github.com/ocornut/imgui.git extern/imgui
```

### MPQ/Data not found at runtime

Place data under `./Data` or set:

```bash
export WOW_DATA_PATH=/path/to/WoW/Data
```

### Clean rebuild

```bash
rm -rf build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```
