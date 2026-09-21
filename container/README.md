# Container Builds

Build WoWee for **Linux**, **macOS**, or **Windows** with a single command.  
All builds run inside Docker - no toolchains to install on your host.

## Prerequisites

- [Docker](https://docs.docker.com/get-docker/) (Docker Desktop on Windows/macOS, or Docker Engine on Linux)
- ~20 GB free disk space (toolchains + vcpkg packages are cached in the Docker image)

## Quick Start

Run **from the project root directory**.

### Linux (native amd64)

```bash
# Bash / Linux / macOS terminal
./container/run-linux.sh
```
```powershell
# PowerShell (Windows)
.\container\run-linux.ps1
```

Output: `build/linux/bin/wowee`

### macOS (cross-compile, arm64 default)

```bash
./container/run-macos.sh
```
```powershell
.\container\run-macos.ps1
```

Output: `build/macos/bin/wowee`

For Intel (x86_64):
```bash
MACOS_ARCH=x86_64 ./container/run-macos.sh
```
```powershell
.\container\run-macos.ps1 -Arch x86_64
```

### Windows (cross-compile, x86_64)

```bash
./container/run-windows.sh
```
```powershell
.\container\run-windows.ps1
```

Output: `build/windows/bin/wowee.exe`

## Options

| Option | Bash | PowerShell | Description |
|--------|------|------------|-------------|
| Rebuild image | `--rebuild-image` | `-RebuildImage` | Force a fresh Docker image build |
| macOS arch | `MACOS_ARCH=x86_64` | `-Arch x86_64` | Build for Intel instead of Apple Silicon |
| FidelityFX SDK repo | `WOWEE_FFX_SDK_REPO=<url>` | `$env:WOWEE_FFX_SDK_REPO="<url>"` | Custom FidelityFX SDK git URL |
| FidelityFX SDK ref | `WOWEE_FFX_SDK_REF=<ref>` | `$env:WOWEE_FFX_SDK_REF="<ref>"` | Custom FidelityFX SDK git ref/tag |

## Docker Image Caching

The first build takes longer because Docker builds the toolchain image (installing compilers, vcpkg packages, etc.). Subsequent builds reuse the cached image and only run the compilation step.

To force a full image rebuild:
```bash
./container/run-linux.sh --rebuild-image
```

## Output Locations

| Target | Binary | Size |
|--------|--------|------|
| Linux | `build/linux/bin/wowee` | ~135 MB |
| macOS | `build/macos/bin/wowee` | ~40 MB |
| Windows | `build/windows/bin/wowee.exe` | ~135 MB |

## File Structure

```
container/
├── run-linux.sh / .ps1          # Host launchers (bash / PowerShell)
├── run-macos.sh / .ps1
├── run-windows.sh / .ps1
├── build-linux.sh               # Container entrypoints (run inside Docker)
├── build-macos.sh
├── build-windows.sh
├── builder-linux.Dockerfile     # Docker image definitions
├── builder-macos.Dockerfile
├── builder-windows.Dockerfile
├── macos/
│   ├── sdk-fetcher.py           # Auto-fetches macOS SDK from Apple's catalog
│   ├── osxcross-toolchain.cmake # CMake toolchain for osxcross
│   └── triplets/                # vcpkg cross-compile triplets
│       ├── arm64-osx-cross.cmake
│       └── x64-osx-cross.cmake
├── README.md                    # This file
└── FLOW.md                      # Detailed build flow documentation
```

## Troubleshooting

**"docker is not installed or not in PATH"**  
Install Docker and ensure the `docker` command is available in your terminal.

**Build fails on first run**  
Some vcpkg packages (ffmpeg, SDL3) take a while to compile. Ensure you have enough RAM (4 GB+) and disk space.

**macOS build: "could not find osxcross compiler"**  
The Docker image may not have built correctly. Run with `--rebuild-image` to rebuild from scratch.

**Windows build: linker errors about vulkan-1.dll**  
The build script auto-generates a Vulkan import library. If this fails, ensure the Docker image has `libvulkan-dev` installed (it should, by default).
