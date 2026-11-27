# WoWee on iPadOS / iOS

The iPadOS/iOS port. It mirrors `android/`: the shared C++ core is built by the
top-level `CMakeLists.txt` (with `CMAKE_SYSTEM_NAME=iOS`), and this directory
holds the Apple-side app shell, assets staging and build configuration.

See `docs/plan-ipados.md` for the plan and the measured device state.

## Status

Builds, installs, signs and plays in-world on an iPad Air 11" (M3). The app
bundle is built by CMake and driven with `xcodebuild`; there is no separate
`.xcodeproj` in this directory.

## Layout

- `ios/Info.plist` - bundle configuration: landscape, file sharing, Photos
  access, scene manifest
- `ios/LaunchScreen.storyboard` - launch screen
- `ios/Assets.xcassets` - app icons
- `cmake/ios.toolchain.cmake` - the iOS cross-compile toolchain

## Build

Requires macOS with Xcode and the iOS SDK. Three inputs sit beside the
source tree:

- **A vcpkg tree** (any clone with the ports registry) providing `openssl`,
  `glm` and `zlib` for the `arm64-ios` triplet. The vcpkg toolchain loads
  `cmake/ios.toolchain.cmake` through `VCPKG_CHAINLOAD_TOOLCHAIN_FILE`, so
  one configure command wires both.
- **MoltenVK 1.4.2**, linked statically so Vulkan runs on Metal. It is not
  part of this repository: clone the upstream release, which ships the
  prebuilt xcframework, into `extern/MoltenVK`:

  ```
  git clone --depth 1 --branch v1.4.2 \
      https://github.com/KhronosGroup/MoltenVK extern/MoltenVK
  ```

  The link falls back to dynamic lookup when the xcframework is absent, so
  a clone without it configures but does not run.
- **SDL 3.2.24**, fetched by CMake itself; nothing to install.

Configure and build (Release is the default - see the comment in
`cmake/ios.toolchain.cmake` for why a Debug build is not a measurement):

```
cmake -S . -B build-ios \
  -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake \
  -DVCPKG_CHAINLOAD_TOOLCHAIN_FILE=$PWD/cmake/ios.toolchain.cmake \
  -DVCPKG_TARGET_TRIPLET=arm64-ios \
  -DCMAKE_XCODE_ATTRIBUTE_DEVELOPMENT_TEAM=<your-team-id> \
  -DCMAKE_XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY="iPhone Developer"
xcodebuild -project build-ios/wowee.xcodeproj -scheme wowee \
  -configuration Release -destination 'id=<udid>' build
```

If `CMakeLists.txt` changed, build twice: ZERO_CHECK regenerates the
`.xcodeproj` mid-build and the first compile can run with stale settings.

### Shaders

CMake compiles every `assets/shaders/*.glsl` to `.spv` itself, but only
when it can find `glslc` (`find_program(GLSLC glslc ...)`); without it
the compile rules are never generated and the committed `.spv` are used
as they are. Eight shaders have never been committed - `grass`,
`grass_cull`, `hiz_build`, `m2_cull_hiz`, `world_map_fog` and the two
`editor_water` stages - so a build machine without `glslc` produces an
app that logs three load errors at startup and runs with grass and M2
occlusion culling off.

Any `glslc` works (Vulkan SDK, shaderc). Without Homebrew on this Mac
the route used here is the vcpkg `shaderc` port in classic mode -
manifest mode refuses package arguments, so run it from the vcpkg root,
and `pkg-config` must be on `PATH` (the corsix tools tree has it):

```
cd <vcpkg> && ./vcpkg install shaderc --triplet arm64-osx
cmake -B build-ios -DGLSLC=<vcpkg>/installed/arm64-osx/tools/shaderc/glslc
```

Two quirks worth knowing:

- The first build after `GLSLC` appears recompiles **all** the shaders
  and rewrites the 84 committed `.spv` in place (same source, different
  compiler revision - the diffs are in the instruction stream, not the
  header). `git checkout -- assets/shaders/` puts them back; later
  builds are incremental and do not touch them.
- The eight new `.spv` are covered by the `*.spv` ignore rule, so they
  stay local until force-added. A clone on a machine without `glslc`
  still lacks them.

### Signing

Signing is automatic, with the team passed at configure time as above
(`XCODE_ATTRIBUTE_DEVELOPMENT_TEAM`; `CODE_SIGN_STYLE` is `Automatic` in
the generated project). The device must be registered to that team. One
gotcha with no fix in the project: the Mac's login keychain re-locks
periodically and CodeSign then fails with `errSecInternalComponent` until
it is unlocked again - unlock the keychain and rebuild.

**Build Release.** A Debug build is not a measurement of anything: it runs
the Khronos validation layer, which intercepts and checks every Vulkan call
(`enableValidation` follows `NDEBUG`, and `NDEBUG` is not defined in a
Debug build), and it compiles at `-Og` rather than `-O3`. The renderer makes
thousands of Vulkan calls per frame, so the difference is not a few percent.
A Debug build is for the crash it was built to catch.

The app bundle is written by the build. Install and run it with `devicectl`:

```
xcrun devicectl device install app --device <udid> <app>
xcrun devicectl device process launch --device <udid> com.wowee.client
```

MoltenVK is linked statically from `extern/MoltenVK` (cloned in Build above),
so Vulkan runs on Metal.

### Measuring

The per-stage and GPU breakdowns are logged at INFO, and the log keeps
warnings by default. Two keys in `settings.cfg` change that, and they are read
from the config file because an environment variable cannot reach an app on
iOS:

```
loglevel=info
frameprofile=1
```

`frameprofile=1` reports the same breakdowns at warning level, so they
survive the default log level. Both work in a Release build.

## Game data

The game data is the player's own: a full extraction is 19-35 GB, which no
tablet holds. Cut it down on the desktop first with the same tool Android
uses - it is platform-agnostic, and produces a tree with its own
`manifest.json` describing exactly what it kept:

```
tools/android/make_minimal_data.py \
    --source ~/Library/Application\ Support/Wowee/Data/expansions/wotlk \
    --out ~/Data-ipad \
    --profile world --maps all
```

`--source` is the directory holding `manifest.json`, which for a desktop
extraction is `Data/expansions/<id>/` - not `Data/`. The tool says so itself
if it is pointed at the wrong one.

Then copy the result onto the device. The client opens its data one file at
a time through `std::ifstream` - there is no archive reader - so the tree
arrives as ~200,000 loose files, and Apple's file sharing is slow with that
many. The script uses `devicectl`'s copy and reports progress:

```
tools/ios-copy-data.sh --udid <udid> --source ~/Data-ipad
```

Finder works too (Files > On My iPad > WoWee > Documents) and is slower.
Either way the files land in `Documents/Data`, which is where the app points
`WOW_DATA_PATH`; it finds the expansion under `Data/expansions/<id>/`.

| Profile | Size | Reaches |
|---|---|---|
| `login` | 890 MB | Login screen, character select and creation. No world. |
| `world --maps azeroth` | 7.3 GB | One continent |
| `world --maps all` | 11.8 GB | Every map, no sound |
| `full` | 18 GB | Everything, sound included |

Neither `login` nor `world` carries any sound: the matcher is an allowlist and
`sound/` is in none of their include lists. The client names 265 sound files
and `sound/` is 6 GB; the game runs without them.

`full` is `**` minus the macOS `.app` bundles, the manual and the cinematics,
so it keeps sound on its own - `--with-sound` is only for the combination of
one continent *and* sound, which no stock profile expresses.

`tools/android/README.md` documents the profiles and the checker that verifies
one against the paths the client names.

## App icon

The tablet shows the same mark as the Mac: `assets/Wowee.png` is the source of
truth for both. The macOS release turns it into `Wowee.icns`
(`tools/macos/create_icns.sh`); the iOS set is rendered from the same file by
`tools/ios/make_appicons.sh`, which writes every slot the asset catalog
declares into `ios/Assets.xcassets/AppIcon.appiconset/`.

Two things the iOS set needs that the Mac one does not:

- **No transparency.** iOS wants an opaque square and applies its own rounded
  mask; the artwork is a rounded tile with transparent corners.
  `tools/ios/flatten_icon.py` fills those corners by continuing the tile's own
  edge outwards, and strips the alpha channel afterwards.
- **A declaration.** `actool` compiles the catalog into `Assets.car` and
  drops `AppIcon60x60@2x.png` / `AppIcon76x76@2x~ipad.png` into the bundle,
  but the icon only appears on the home screen if `ios/Info.plist` names it
  (`CFBundleIcons` and `CFBundleIcons~ipad`). Without those keys the bundle
  has an icon and still draws a blank placeholder.

Regenerate after changing the artwork:

```
tools/ios/make_appicons.sh
```

## Configuration

`core::getConfigRoot()` resolves to the app's `Documents/config/` on iOS.
Nothing may write to `HOME`; there it is outside the sandbox and the write
fails with `EPERM`. The log is at `Documents/config/logs/wowee.log`.

Screenshots are saved to `Documents/config/screenshots/` and also into the
photo library, so they show up in Photos alongside the tablet's own. `/share`
offers the last one through the iOS share sheet.

## Testing notes

What the port has been exercised on:

- **Device and OS:** iPad Air 11" (M3, `iPad15,3`), iPadOS 27.2.1, landscape,
  Release build, external keyboard and trackpad/mouse.
- **Game data:** WotLK 3.3.5a (`Data/expansions/wotlk`), reduced with
  `tools/android/make_minimal_data.py`. Login, character select and
  in-world play are verified. The Vanilla 1.12 and TBC 2.4.3 protocol paths
  are the shared desktop code and were not exercised on the device.
- **Performance:** thermal-bound, not software-bound. The iPad Air is
  fanless: a cold device boosts for the first ~60-90 seconds in world
  (60 fps at 1180x820 with light shafts off), then the power governor
  settles the clocks and the frame lands at a sustained ~43-44 fps on the
  same scene - the same chunks, draws and lights at both rates, with every
  GPU pass uniformly ~1.5x slower and every CPU stage ~1.25x slower, which
  is the signature of the silicon slowing down rather than the workload
  growing. A hot device boosts for seconds, not minutes (verified by
  restarting the app with the device still warm), and charging while
  playing lowers the sustained rate further. `lightshafts=0` removes a
  ~2.3 ms compute pass and with it about 11 fps of the gap; lower
  render resolutions raise the sustained floor the same way, at the cost
  of sharpness. Warden checks were a separate, periodic dip on top of this
  baseline (~10-15 fps for ~20 s per minute before the utility-QoS fix);
  the sustained rate itself is unchanged by them. `frameprofile=1` logs
  the per-stage CPU and GPU times that made this attribution;
  `docs/plan-ipados.md` section 8 has the lever list.
- **Isolation:** every source file the port touches compiles in a host
  (non-iOS) configuration, and the platform-specific pieces sit behind
  `WOWEE_IOS` / `__ANDROID__` / `if(CMAKE_SYSTEM_NAME STREQUAL "iOS")`.

## Known not to work yet

- **The whole window is rotated 90 degrees on an external display.** This is
  an iPadOS limitation, not a client bug: it reproduces in unrelated apps —
  CorsixTH and Draw Things ([issue #177](https://github.com/drawthingsai/draw-things-community/issues/177)),
  the latter on iPadOS 27.2 with an extended USB-C display in both Windowed
  Apps and Stage Manager, in both iPad orientations, with resizing, moving and
  reopening all ineffective. The Draw Things report also confirms the system
  menu bar and Dock stay upright: it is the app's own `UIWindowScene` that is
  rotated, which is why its window buttons turn with the content. iPadOS
  presents the external display with a portrait-ordered
  `UIScreen` (a 2560x1440 landscape panel comes through as 1440x2560 points /
  2880x5120 pixels) and rotates the entire `UIWindow` — including its own
  red/yellow/green window buttons — 90 degrees when the app is shown there.
  Because the rotation is applied to the window by the system compositor, it
  cannot be corrected by the client: rotating the final composite would leave
  the system chrome sideways, and the swapchain extent follows the surface
  (`currentTransform=0x1, supportedTransforms=0x1`, so there is no
  pre-transform to request on MoltenVK). App-side attempts to force a landscape
  window size, set the `CAMetalLayer` drawable size, request landscape scene
  geometry, or allow portrait in `Info.plist` / `SDL_HINT_ORIENTATIONS` were all
  tried on device and all left the window rotated. The built-in screen is
  correct, and moving the window between displays is safe (a transient
  0-height surface during the move is rejected rather than crashing). There is
  no known app-side fix; this needs a change in iPadOS's external-display
  handling.
- **Touch input is untested on the iPad.** Android's touch controls are
  compiled in and hidden behind the same mouse-present check, but the port
  has only ever been driven by keyboard and mouse; fingers have not been
  tried.
- **A fresh clone used to lack eight compiled shaders.** The build machine
  had no `glslc`, so `grass_cull.comp`, `m2_cull_hiz.comp` and
  `world_map_fog.frag` (GPU grass culling, M2 occlusion culling and the
  world-map fog overlay) had no `.spv` and logged a load error. All 92
  `.spv` are now tracked and un-ignored, and a check verifies every `.glsl`
  has one; a machine without `glslc` builds from them. See **Shaders** above.
- **No sound** with the `login` and `world` data profiles; only `full`
  carries `sound/`.
- **Doodads drop out of very dense scenes**: the M2 instance buffer holds
  32,768 instances and logs `instance buffer full` when it runs out.
- **Warden anti-cheat modules do not run** (its loader cannot execute the
  Windows module); the client warns and plays.
- **20 fps in-world**, which is a measurement, not a mystery: the frame is
  GPU-bound (see section 8 of the plan), the same 50.00 ms frame holds for
  hours, and a frame cap (`frame_cap`) and a lower render scale
  (the resolution selector) are both already in place and were measured.
