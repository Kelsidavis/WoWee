# Android Port Plan

**Status:** phase 1 done. `libwowee.so` builds for `arm64-v8a` and the desktop build is unchanged.
Next: phase 2, the Activity and a first run. Branch `android`.
**Target:** a real phone or tablet, touch-driven, installed as an APK.
**Constraint set by the project:** it must fit the existing asset-extraction release style - the
player extracts from their own WoW install and the release ships the client, not the data.

Measured 2026-08-18 against the tree at `65be6354f`. Numbers here are from this working copy, not
estimates.

---

## 1. What is already portable

The unusual thing about this codebase is how little of the port is code. Both roots the client
needs are already chosen at runtime, and the three libraries that matter all have first-class
Android support.

| Thing | Evidence | Consequence for Android |
|---|---|---|
| Windowing and input | SDL2 (`src/core/window.cpp`, `SDL_Init`, `SDL_CreateWindow`, `SDL_Vulkan_LoadLibrary`) | SDL2 is an official Android backend. Surface, lifecycle and the GL/Vulkan context come free |
| Graphics | Vulkan throughout, no GL path | Android's native API. No translation layer |
| Shaders | 81 `.spv` shipped in `assets/shaders/` beside their GLSL | SPIR-V is portable. Nothing to recompile at install time |
| Build | CMake, already cross-compiling | `CMAKE_CROSSCOMPILING` branches exist for Windows and Darwin; the NDK is a third toolchain file, not a new system |
| Data location | `WOW_DATA_PATH`, default `./Data` (`application.cpp:288`) | The Java side sets one environment variable. **No change to the asset layer** |
| Config location | `WOWEE_CONFIG_ROOT` wins over everything (`config_paths.cpp:65`) | Points at the app's private directory. Same - one variable |
| Asset lookup | Manifest-driven: `Data/manifest.json`, 199,468 entries, path → file + size + hash | Works off any directory the process can read |
| GPU features required | `samplerAnisotropy`, `shaderStorageImageWriteWithoutFormat` (`vk_context.cpp:457`) | Both are ordinary on Android Vulkan 1.1 hardware |
| Upscaling | FSR2/FSR3 already behind `WOWEE_HAS_AMD_FSR2` 0/1 (`CMakeLists.txt:70,107`) | The off path is built and tested. Nothing to strip |
| Warden | x86 emulation behind `HAVE_UNICORN`, with a stub branch that compiles without it | Leave Unicorn out. The stub is already covered by `tools/unicorn_stub_check.py` |

Anyone proposing "abstract the windowing layer" or "add a Vulkan backend" has not read this code.

---

## 2. The four things that actually block it

### Blocker 1: 35 GB of assets, and the release ships none of them

This is the whole problem. Everything else is work; this is a constraint.

| Directory | Size |
|---|---|
| `Data/expansions` | 16 GB: tbc 7.2, turtle 5.5, wotlk 3.0, classic 0.05 |
| `Data/sound` | 6.0 GB |
| `Data/terrain` | 5.8 GB |
| `Data/world` | 2.4 GB |
| `Data/creature` | 1.8 GB |
| `Data/item` | 1.1 GB |
| `Data/interface` | 495 MB |
| `Data/character` | 375 MB |
| `Data/db` | 87 MB |
| **Total** | **35 GB**, or **19 GB** for a base install with no expansion overlays |

What that rules out immediately:

- **Bundling in the APK.** Play caps the base at 200 MB and asset packs at 1-2 GB. Not close.
- **Play Asset Delivery.** Same ceiling, and the data is the player's own extraction, not ours to
  distribute. The release style exists precisely because we do not ship Blizzard's files.

What it leaves, and this is the part that fits the release style rather than fighting it:

**The player extracts on a desktop, as they do today, and copies `Data/` to the device.** The app
is pointed at that directory with `WOW_DATA_PATH`. The extraction tooling
(`tools/asset_pipeline_gui.py`, `asset_extract`) stays desktop-only and unchanged. It already
supports Linux, macOS and Windows, and it is the same artefact either way.

The Android-specific question is *where* on the device, and scoped storage decides it:

| Location | Permission | Works with native `fopen` | Verdict |
|---|---|---|---|
| `getExternalFilesDir()`, `/sdcard/Android/data/<pkg>/files/` | none | yes | **Use this.** User copies over MTP, adb push, or a file manager |
| Arbitrary path with `MANAGE_EXTERNAL_STORAGE` | Play-restricted, needs justification | yes | Only if a side-loaded build is acceptable |
| Storage Access Framework tree URI | user-granted | **no**. SAF hands out file descriptors, not paths | Would require rewriting `AssetManager` around fds. Do not |

Consequences worth stating plainly: a 19 GB base install does not fit on a 64 GB phone with
anything else on it, and `/sdcard/Android/data` is wiped when the app is uninstalled. Both are the
player's problem to be told about clearly, not ours to engineer around.

**Open question worth measuring before phase 3:** the manifest has 199,468 entries and the client
opens files individually. Android's storage stack is slower per-open than a desktop filesystem, and
the terrain and model streaming already have per-frame budgets tuned against an SSD.

### Blocker 2: the interface is mouse and keyboard throughout

| Measure | Value |
|---|---|
| Bindable UI actions | 16 (`keybinding_manager.hpp`, `enum class Action`) - the toggles only |
| Files touching mouse buttons or key state | 18 |
| Client actions by how they are wired | 6 polled inside a panel's own draw, 8 routed, 10 handled inline (`tools/keybinding_route_check.py`) |
| Touch handling anywhere in the tree | **none**. No `SDL_FINGER*`, no `SDL_TOUCH*` |

The interface itself is FrameXML now, which helps: it is Lua, and its buttons are already driven
through one widget layer with hit testing this client owns. What has no touch equivalent at all:

- **Right-button mouse look**, which is how the camera is aimed
- **Right-click to interact**, loot, and open a vendor
- **Drag and drop** on the cursor - bags, action bars, trade
- **Hover**, which every tooltip depends on
- **Modifier-click** - shift-split a stack, ctrl-compare an item
- **Text entry** for chat, which needs the soft keyboard and an IME hook

This is the largest piece of design work in the port and it is not a porting task; it is an
interface design task with an implementation behind it.

### Blocker 3: the renderer is written for a desktop discrete GPU

Not fatal, but every one of these wants measuring on a tiler before it is trusted:

- Worker threads come from `std::thread::hardware_concurrency()` in the terrain, character and WMO
  renderers. A phone's big.LITTLE core count is not a thread budget.
- MSAA is offered to 8x and the default is 2x. Full-resolution MSAA on a tile-based GPU costs
  bandwidth in a way it does not on desktop.
- The terrain mega-buffers, the HiZ pyramid and the descriptor pools are sized against desktop VRAM.
- FSR2/FSR3 stay off. That is already a supported configuration.

### Blocker 4: the entry point and the platform gates

Smallest of the four. `src/main.cpp` is a plain `int main(argc, argv)`; SDL on Android wants
`SDL_main` and the process is a shared library loaded by an Activity, not an executable. Twelve
files carry `__APPLE__` / `_WIN32` / `__linux__` gates that need an Android arm - most will fall to
the Linux branch, which is the point of checking rather than assuming.

---

## 3. Order of work

Each phase ends somewhere a person can see the result. No phase depends on the one after it.

| Phase | Ends when | Needs |
|---|---|---|
| ~~**1. Toolchain**~~ | **done** - see below | |
| ~~**2. It starts**~~ | **APK built and statically verified - see below. The half that needs a device is unfinished.** | |
| **3. It draws the world** | A character logs in and the world renders at a watchable frame rate on one real device, with GPU budgets re-measured rather than inherited | Phase 2, a device |
| **4. It plays** | Touch scheme for camera, movement, interaction and the cursor; soft keyboard for chat | Phase 3, and the design decisions in blocker 2 |
| **5. It ships** | APK, storage-permission flow, and a written path for the player to get 19 GB onto the device | Phase 4 |

Phase 1 is a day's work. Phase 4 is the port.

---

## 4. Decisions taken, so they are not re-litigated

- **Assets are never bundled and never downloaded by us.** The player extracts, as on desktop. This
  follows from the release style, not from the file size.
- **No SAF.** It would mean rewriting the asset layer around file descriptors to gain a directory
  picker.
- **No Unicorn, no Warden emulation.** The stub branch exists and is checked.
- **No GL fallback.** Vulkan 1.1 is the floor; a device without it is not a target.
- **The extraction tooling stays desktop-only.** It is Python and tkinter, and the player has a
  desktop already - they need one to own the game files.

---

## 5. Phase 1, as built

```
sdkmanager 'ndk;28.2.13676358'
tools/build-android-deps.sh                    # OpenSSL for arm64, once

cmake -B build-android -S . \
  -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK_ROOT/build/cmake/android.toolchain.cmake \
  -DANDROID_ABI=arm64-v8a -DANDROID_PLATFORM=android-33 \
  -DCMAKE_BUILD_TYPE=Release \
  -DOPENSSL_ROOT_DIR=$PWD/build-android-deps/arm64-v8a \
  -DCMAKE_FIND_ROOT_PATH=$PWD/build-android-deps/arm64-v8a
cmake --build build-android --target wowee
```

The result links against `liblog`, `libandroid`, `libvulkan`, `libz`, `libdl`, `libSDL2`, `libm`
and `libc`, and exports `SDL_main`, which is what SDLActivity calls in phase 2.

### What the phase actually cost

| Problem | Fix |
|---|---|
| No system SDL2 | FetchContent, pinned to `release-2.32.10`. **Not** the 2.30 the desktop carries: 2.30 calls `ALooper_pollAll`, which NDK 28 marks unavailable |
| No system glm | FetchContent, pinned to 1.0.1 |
| OpenSSL does not build with CMake | `tools/build-android-deps.sh` builds it once and the client is pointed at it with `OPENSSL_ROOT_DIR`, the way the macOS build points at Homebrew's |
| `find_package` ignored that prefix | The NDK toolchain confines finds to its sysroot. `CMAKE_FIND_ROOT_PATH` has to name the prefix as well |
| Host glibc headers in the cross build | `pkg_check_modules(FFMPEG ...)` runs the **host** pkg-config and returns the host include path. Skipped on Android, where the no-FFmpeg path was already supported. The same trap the Windows cross build hit with Vulkan |
| `<SDL2/SDL.h>` not found | SDL's build tree puts headers at the top of `include/`. A directory holding one link named `SDL2` fixes the spelling without touching 30-odd files |
| miniaudio unused parameters | Its OpenSL and AAudio backends only compile on Android. `-Wno-unused-parameter` on the one unit that compiles it, rather than patching a vendored header |
| X11 | `__linux__` is defined on Android. Three sites in `main.cpp` and one `target_link_libraries` needed `AND NOT ANDROID`. The other Linux branches - `/proc/self/exe`, `sched.h`, `pthread.h` - are correct there and were left alone |
| `backtrace()` | Bionic declares it only from API 33. The crash log keeps everything but its stack section |
| `undefined symbol: main` | On Android the client is a shared library, not a program. `add_library(wowee SHARED ...)` under `if(ANDROID)` |

### The decision phase 1 forced: minSdk 33

Four Vulkan entry points the renderer calls - `vkCreateRenderPass2`, `vkWaitSemaphores`,
`vkGetDeviceBufferMemoryRequirements`, `vkGetDeviceImageMemoryRequirements` - are Vulkan 1.2 and
1.3 core. Android's `libvulkan.so` stub exports them by API level, measured against this NDK:

| API level | Of the four, exported |
|---|---|
| 29 | 0 |
| 31 | 2 |
| 33 and up | 4 |

So the floor is **Android 13**, not the Vulkan 1.1 this document assumed in section 1. The
alternative is loading those four through `vkGetDeviceProcAddr` and keeping a lower minSdk, which
is worth doing only if a device that matters is stuck below 13.


---

## 6. Phase 2, as built

`android/` is a Gradle project that produces an installable APK. `./gradlew :app:assembleDebug`
from that directory, with `ANDROID_HOME` and the NDK set, gives 45 MB at
`android/app/build/outputs/apk/debug/app-debug.apk`. OpenSSL has to exist first:
`tools/build-android-deps.sh arm64-v8a` builds it, and `gradle.properties` points at where it lands.

### Where the client's files go

The client finds its shaders, its interface and its expansion profiles through paths relative to
the working directory, which Android does not give a process. The renderer alone opens
`assets/shaders/*.spv` from about thirty call sites. So there is one root, and it holds the layout
of a desktop install:

```
<getExternalFilesDir>/
    assets/     unpacked from the APK
    Data/       the four expansion profiles from the APK, plus what the player copies in
    config/     settings.cfg and the logs
```

`WoweeActivity` sets `WOWEE_RESOURCE_ROOT`, `WOW_DATA_PATH` and `WOWEE_CONFIG_ROOT` to point into
it before `super.onCreate` loads the library, and `main()` enters that root first thing. The two
config variables were already read by `config_paths.cpp` and `logger.cpp`; only the root is new.

It is the external files directory rather than the internal one for one reason: the player has to
be able to reach it. Game data is extracted on a PC and copied to `Data/` under this root, which
`adb push` and a USB cable can both write.

### What the APK carries

The same payload the desktop and macOS releases ship, assembled by the same rules: `assets/`
without the proprietary music, and the eighteen git-tracked `Data/` JSONs. No game data, and
nothing that downloads any. A recursive glob for those JSONs is wrong and was caught doing it -
it also matches the overlay manifests a local extraction generates, which added 15 MB of
someone else's files to the APK. The include list names them.

Assets are unpacked when the APK is newer than the last unpack, not when the destination is
missing. Skipping files that already exist means a new client keeps running the old client's
shaders.

### What was verified without a device

- Every one of the 639 symbols `libwowee.so` leaves undefined resolves against the API 33 platform
  stubs plus the two libraries in the APK. Nothing is waiting to fail at `dlopen`.
- `SDL_main` is exported, which is the symbol `SDLActivity` looks up by name.
- The APK declares `com.wowee.client.WoweeActivity` as its launcher, ships `arm64-v8a` only, and
  both that class and `SDLActivity` are in the dex.
- `checkSdlJava` compares the nine committed `org/libsdl/app` Java files against the SDL that
  CMakeLists.txt fetches, and fails the build on drift. A mismatched pair fails at runtime with a
  missing native method and says nothing at build time. Canaried.

### What is not verified

The half of this phase that reads "reaches the login screen". There is no device attached, and the
emulator is not a way around it on this machine: `/dev/kvm` does not exist because VirtualBox's
`vboxdrv` holds the virtualization extensions and neither `kvm_amd` nor `kvm_intel` is loaded.
Without that, an image emulates every instruction on the way to a Vulkan renderer. An arm64 image
would do so even on a machine with KVM.

Everything above is static evidence that the library will load. None of it is evidence that it
runs. What that needs is one arm64 device on Android 13 or later:

```
adb install -r android/app/build/outputs/apk/debug/app-debug.apk
adb push <extracted Data>/. /sdcard/Android/data/com.wowee.client/files/Data/
adb logcat -s Wowee SDL wowee
```

### Getting data onto a device

A full extraction is 18 GB and no phone is going to hold it, so `tools/android/`
cuts one down. `make_minimal_data.py --source ~/Data --out ~/Data-login --profile login`
gives 787 MB, 23,072 of 199,468 files, which is the login screen, character select and
character creation with no world in it. A `world` profile with one continent is 7.5 GB.
`tools/android/README.md` has the detail.

The subset gets its own `manifest.json`. The client finds its data root by looking for one
and resolves every path through it, so a copied subtree without a rewritten manifest is not
a smaller install, it is a broken one.

`check_profile_coverage.py` reads every asset path the client names, in `src/` and in the
interface's Lua and XML, and reports what a profile would drop. All three profiles come back
clean. The `login` subset also loads the interface identically to the full extraction under
`framexml_run`: 139 files, 0 failed, same missing-API counts.

That is as far as this can be taken without hardware. `world` is 7.5 GB because
`world/wmo`, `creature` and `item` are shared across zones; narrowing them by map means
walking each ADT for the models it references, which is phase 3 work and is not written.

### A hole to close in phase 5

`Android/data/<package>/files` is awkward to reach from a phone's own file manager on Android 11
and up. `adb push` and most USB connections still write it, but the player-facing answer is a
first-run importer that asks for the extracted `Data/` folder through the storage access
framework, which sidesteps the question entirely.
