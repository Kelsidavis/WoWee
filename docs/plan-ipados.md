# iPadOS Port Plan

**Status:** running on device (iPad Air 11" M3, iPadOS 27.2). Login, character
select and in-world play all verified.
**Target:** iPadOS (and iOS) on arm64, touch-driven, installed as an IPA.
**Goal:** mirror the Android port's architecture: shared C++ core built by the
top-level CMake, Apple-side app shell, SDL3 + Vulkan via MoltenVK.

## 1. What is already portable

Same as Android (see `docs/plan-android.md` §1): SDL3 windowing/input, Vulkan
renderer, SPIR-V shaders, CMake cross-build, runtime data/config paths, FSR2
off path, Warden stub. iPadOS adds Apple-specific constraints below but no
new abstraction in the core.

## 2. Key differences from Android

| Area | Android approach | iPadOS approach |
|---|---|---|
| Windowing/input | SDLActivity (Java) | SDL3 UIKit backend (Objective-C), app bundle |
| Graphics | native Vulkan | MoltenVK (Vulkan -> Metal) |
| Build | Gradle + NDK | Xcode + CMake toolchain file |
| Packaging | APK/AAB | IPA via Xcode archive |
| Distribution | sideload / Play | TestFlight / App Store (or sideload) |
| Soft keyboard | SDL text input | UIKeyboard / SDL text input |
| Files/storage | `getExternalFiles()` | App sandbox + File Sharing (iTunes/Finder) or iCloud |
| Touch | `src/ui/touch_controls.cpp` | reuse same module |

## 3. Blockers

1. ~~**No macOS/Xcode in this environment.**~~ Resolved: builds happen on the
   user's Mac with Xcode; the repo's CI still only does scaffold + cross-compile
   headers.
2. ~~**MoltenVK.**~~ Resolved: MoltenVK 1.4.2 static xcframework vendored in
   `extern/MoltenVK/`.
3. **Code signing & provisioning.** Signing team `5NLYCB4UAX` works, but the
   Mac's login keychain re-locks periodically and then CodeSign fails with
   `errSecInternalComponent`. The user must unlock the Mac before a build can
   finish. Nothing to fix in the project.
4. **Touch UX.** Reuse Android's `touch_controls.cpp`; iPad has larger screen
   and (optionally) keyboard/trackpad/mouse. Deferred - the iPad is played
   with keyboard + mouse for now.
5. **App lifecycle / background.** iOS suspends aggressively; need to handle
   `SDL_APP_WILLENTERBACKGROUND` to pause network/render.
6. **Large data (35 GB).** Same constraint as Android, and the same answer:
   `tools/android/make_minimal_data.py` cuts the extraction down to a profile
   that fits (787 MB for the login screen, 7.5 GB for one continent, 12 GB for
   every map). The tool is platform-agnostic - it reads `manifest.json` and
   writes a tree with its own - so the iPad uses it unchanged. The result is
   copied into `Documents/Data` over USB with Finder. See `ios/README.md`.

## 4. Work breakdown

- Phase 0 (done): scaffold `ios/`, `docs/plan-ipados.md`, CMake iOS guards,
  `src/core/ios_platform.{hpp,mm}`.
- Phase 1 (done): CMake iOS toolchain file (`cmake/ios.toolchain.cmake`),
  `libwowee` + MoltenVK built and linked.
- Phase 2 (done): Xcode app bundle, Info.plist, LaunchScreen, SDL main glue,
  first run to login screen, and through to in-world play.
- Phase 3 (pending): touch controls (reuse Android module), soft keyboard,
  safe areas.
- Phase 4 (partial): the scene delegate forwards the lifecycle; background
  pause still to do.
- Phase 5 (pending): TestFlight/App Store packaging, release workflow.
- Data sizing (done): the extraction is cut down with the existing profile
  tool, and the app bundle carries only the assets the client reads (the
  project's own soundtrack and the README screenshots are not in it).

## 5. GitHub workflow

Fork `Kelsidavis/WoWee` to the personal account, develop on the `ipados`
branch, open PR upstream when stable. See README/CONTRIBUTING.md.

## 6. Open questions

- Use SwiftUI/UIKit app shell or SDL's default UIKit main? Prefer SDL's.
- Mouse/trackpad support on iPad (pointer) - leverage SDL mouse events.
- Whether to ship as iPad-only or universal (iPhone). Start iPad-only.

## 7. What the iPad build is made of

- **Build:** `xcodebuild -project build-ios/wowee.xcodeproj -scheme wowee
  -configuration Debug -destination 'id=<udid>' build`, with
  `PATH="$HOME/Library/Python/3.9/bin:/Users/ale/Documents/corsix-ios/tools/pkgconf/bin:$PATH"`.
  vcpkg dependencies come from `/Users/ale/Documents/corsix-ios/vcpkg`
  (triplet `arm64-ios`); SDL 3.2.24 is fetched by CMake rather than taken from
  vcpkg.
- **Install / run / inspect:**
  - install: `xcrun devicectl device install app --device <udid> <app>`
  - launch: `xcrun devicectl device process launch --device <udid> com.wowee.client`
    (`--terminate-existing` restarts it)
  - screenshot: `xcrun devicectl device capture screenshot --device <udid> --destination <path.png>`
  - log: `Documents/config/logs/wowee.log` under the `appDataContainer`
    domain; config: `Documents/config/settings.cfg` the same way.
  - crash reports: `--domain-type systemCrashLogs`.
- **Orientation:** landscape, forced through `SDL_HINT_ORIENTATIONS` and the
  Info.plist (the plist is generated at CMake configure time, so a plist edit
  needs a reconfigure).
- **Configuration root:** `core::getConfigRoot()` resolves to the app's
  `Documents/config/` on iOS. Nothing may write to `HOME`; on iOS that is
  outside the sandbox and the write fails with `EPERM`. See commit `f5968c1c`.

## 8. Performance on the M3 (iPad Air 11")

Measured 2026-11-10 on an iPad Air 11" (M3), 2360x1640.

- **The numbers below were measured on a Debug build, and a Debug build is not
  a measurement of anything.** `enableValidation` follows `NDEBUG`
  (`include/rendering/vk_context.hpp`), so a Debug build loads the Khronos
  validation layer, which intercepts and checks every Vulkan call - and the
  renderer makes thousands of them per frame. A Debug build also compiles at
  `-Og` rather than `-O3`. `cmake/ios.toolchain.cmake` now defaults to
  Release, for that reason.

  It also inflates the GPU numbers, which is the part that makes it
  misleading rather than merely slow: the marks are
  `VK_PIPELINE_STAGE_BOTTOM_OF_PIPE` timestamps, so they measure elapsed
  GPU-queue time *including the gaps*. If the CPU cannot record the frame
  fast enough the GPU starves, and the gap is charged to whichever pass it
  lands in. "The world pass is 63.5 ms of 76" may be starvation rather
  than fragment work.

  So the first thing a Release build has to answer is whether the frame is
  still 76 ms of GPU time. If it is not, nothing below needs chasing.
- *(The next two bullets were measured on that Debug build and are kept only
  as history - the Release numbers supersede them, in "Release measurements"
  below. Debug `endFrame` was 75-80 ms; Release `endFrame` is 33 ms with
  synchronous submits and 0.04 ms with them turned off.)*
- **The frame is GPU-bound.** Every render stage is under 50 ms except
  `endFrame`, which sits continuously at **75-80 ms** - post-process, the UI
  overlay, submit and the present wait. That is ~13 fps at native resolution,
  and ~9 fps in the denser parts of the world. Vsync cannot help when the GPU
  itself is the bottleneck.
- **The 30 -> 20 -> 15 -> 12 fps steps are vsync quantisation, not a cap.**
  They are 60 Hz vsync intervals of 2, 3, 4 and 5 (33, 50, 67 and 83 ms),
  and a 76 ms GPU frame lands in the fifth one. The frame cap is unlimited
  by default (`frameCapFpsForChoice(0)` returns 0).
- **The resolution selector now sets the internal resolution, upscaled by
  FSR1** (`56eb6fae`), and the list it offers is derived from the device's
  own screen (`00c74bf4`). On a platform that can resize its window
  nothing changes; on the iPad the world is drawn at the requested size and
  stretched onto the surface. Measured on the device:
  `FSR: initializing at 1036x720 -> 2360x1640 (scale=0.67)`.
- **FSR1 upscaling is already wired** and should work on iPad:
  `setFSREnabled` / `setFSRQuality` in `PostProcessPipeline`, with the
  client's own EASU/RCAS shaders (`fsr_easu.frag.spv`, `fsr_rcas.frag.spv`)
  bundled in the app. On Apple only *frame generation* is forced off; the
  upscaling path itself is not. Enabling FSR1 forces MSAA off
  (`MsaaChangeRequest`). Quality choices map to a render scale of
  {0.77, 0.67, 0.59, 1.00} (`fsrScaleForChoice` in
  `include/ui/graphics_choices.hpp`).
- **FSR2/FSR3** need `shaderStorageImageWriteWithoutFormat` and
  `shaderInt16`; `setFSR2Enabled` refuses with a warning when the device
  cannot provide them.
- **MetalFX is Metal-only, but MoltenVK can hand over Metal objects.**
  MetalFX (`MTLFXSpatialScaler`, `MTLFXTemporalScaler`) has no Vulkan
  surface, so it cannot be called from the renderer directly. The bridge is
  `VK_EXT_metal_objects`, which **MoltenVK 1.4.2 supports on iOS 8.0+**
  (`MoltenVK/MoltenVK/Layers/MVKExtensions.def`:
  `MVK_EXTENSION(EXT_metal_objects, EXT_METAL_OBJECTS, DEVICE, 10.11, 8.0, 1.0)`).
  Its `getMetalObjects` implementation exports the device, the command queue,
  buffers, textures and shared events (`MVKDevice.mm`), so the pieces a
  MetalFX pass needs are all reachable.

  What is *not* settled is the surrounding work, which is why this is not
  simply "add MetalFX":
  - The extension is a **device** extension and is **off by default**.
    MoltenVK enables extensions only when asked
    (`MVKExtension::MVKExtension` sets `enabled = enableForPlatform && ...`),
    so the client must enable it explicitly alongside its other
    `enable_extension_if_present` calls.
  - MetalFX encodes into an `MTLCommandBuffer`; the client's frame is a
    Vulkan command buffer on MoltenVK's queue. Ordering the two needs
    `MTLSharedEvent` plumbing that `VK_EXT_metal_objects` also provides, but
    nothing in this codebase does it yet.
  - The client's render targets would have to be created so that the Metal
    side can use them, and the final upscale has to land back in a Vulkan
    image the present path can read.
  - `MTLFXTemporalScaler` additionally wants depth and motion vectors. The
    client has a motion-vector pass (`dispatchMotionVectors`), so that part
    is in reach; `MTLFXSpatialScaler` wants colour only.

  So the route is open but it is a project, not a flag. FSR1 is the
  Vulkan-native equivalent and is already wired.

### Release measurements (2026-11-12 to 11-13)

Everything here is Release, in-world, standing still outdoors with an
unmoved camera, at 786x546 internal upscaled by FSR1, `shadow_distance=350`,
after a 50 s settle. Only settled windows are compared: a world 18 s after
`SMSG_LOGIN_VERIFY_WORLD` reads GPU 15 ms against 35-40 ms settled, and
believed a false result once.

**The pushed `view_distance=1600` never applied** (found 2026-11-13, see
"the config push was being reverted" below): the effective value in every
run on this page was **1000**, and the same is true of the other bound
keys. The numbers are still internally comparable - every run was reverted
the same way - but they are measurements of the reverted settings, not of
the pushed ones.

**The frame is exactly 50.00 ms - 20.00 fps.** 201 frames per 10 s window,
reproduced across runs in two different zones and over an 8-hour overnight
run. The model that fits everything measured:

    frame ~= CPU record (renderWorld 2.4-4.9ms, prepare 0.06ms, plus
             update/UI) + GPU (35-43ms) serialised

GPU marks (BOTTOM_OF_PIPE, so they resolve to the render pass and include
any queue gap): world 19-22, shadows 7.4, volumetric fog 4.0-4.1,
post-process 1.95, interface 0.33-2.15. Recording is free - command
recording 0.02 ms, present 0.01 ms, terrain CPU 0.12, wmo 0.35, m2 2.1-4.8.

**The 30 -> 20 fps drop is a step, not decay.** Login holds 30.00 fps for
3 minutes, the world holds 29.45-29.85 for ~30 s, then steps to 20.43 and
sits at exactly 20.00 for 12 minutes. Two causes: a ~33.3 ms floor inside
`vkQueueSubmit` when the GPU is light (the menu submit blocks 31.9 ms while
the GPU is at 3.15 ms - and it is not present-mode pacing: the menu is
30.00 fps with IMMEDIATE and 29.95 with FIFO), and world streaming raising
GPU time 15.5 -> 35.1 ms, so CPU 14 + GPU 35 lands on 50.

**Pass ablation** (`passablation=1`, 50 s settle `ablationsettle`, 13
phases, 4.0 s each + 0.75 s warmup, drift across the run -0.01 ms):

| pass off | ms/frame | worth |
|---|---|---|
| shadows | 43.84 | **+6.15** |
| wmo | 46.52 | **+3.47** |
| volumetric fog | 47.29 | **+2.70** |
| terrain / doodads / characters / sky / sun shafts / doodads past 250 y | 49.99-50.03 | 0 |
| ground clutter alone | 62.89 | -12.90 (noise) |
| grass | 53.19 | -3.20 (noise) |

The zeros are real, not a wiring miss - each skip gates its actual draw
path (the `skipTerrain`/`skipM2`/... block in `renderer.cpp`) - and they
make sense: at 786x546 with Hi-Z holding drawn chunks to ~700 of 12k-24k
resident, terrain and the instanced doodads are sub-millisecond of fill.
The two negative rows are single hitches polluting a 64-76 frame mean (one
~900 ms frame is +12.9 ms of mean), not passes that cost negative time.
**Everything selectable in the scene pass is worth ~12 ms total.**

**`asyncsubmit=1` frees the CPU and changes no fps.** The MoltenVK config
reaches the driver (`synchronousQueueSubmits=1` read back, turned off
through the deprecated `vkGet/SetMoltenVKConfigurationMVK`):

- `endFrame` 32.99 ms/frame (65% of the window) -> **0.04 ms (0%)**
- `renderWorld` 4.87 -> 2.45 ms/frame
- fps: **20.00 -> 20.00**, frame 50.00 ms either way
- GPU marks 35 -> 40-44 ms (world 22, shadows 11.0, fog 4.1, post 1.95,
  interface 2.15)

So the 33.3 ms synchronous-submit floor is a menu/idle phenomenon; in the
world the frame is bound by GPU time, and once the CPU no longer blocks in
submit the frame tracks the GPU mark instead. The mark rose with the
change - either it reads truer now that the CPU inserts no sync points
between passes, or the GCD submission path adds bubbles; either way the
frame time did not move. **60 fps now means GPU 43 -> under 17 ms.**

**The device lost did not reproduce.** One `beginFrame` fence wait came
back `-4` a second after a swapchain rebuild (2026-11-12 21:51:25,
`currentExtent=2360x1640` then fence `-4`). The overnight run after it:
91,817 log lines, 8 hours in-world, 0 device lost, no background/foreground
transitions, and the new lifecycle logging shows swapchain-dirty only at
startup (the vsync toggle and window events 519/532).

**`pom=0` is dead** (measured earlier: 50.0/38.8 GPU vs 50.0/35.2
baseline).

### The config push was being reverted at world entry (found 2026-11-13)

The 50.00 ms frame was vsync after all, and every hand-pushed config was
only half-applied:

- **`cvars.cfg` is replayed over `settings.cfg` at world entry.**
  `AddonManager`'s init calls `applyStoredCVarSideEffects`
  (`addon_manager.cpp:574` -> `lua_system_api.cpp:1775`), which iterates
  the CVar store and runs each binding's side effect. The upstream binding
  `{"gxvsync","vsync"}` (`lua_system_api.cpp:1234`, commit `0ed6532b`) turns
  a stored `gxvsync=1` into `setSettingValue("vsync","1")` ->
  `setVsync(true)` -> **FIFO**. The log timing is exact: `VSync enabled`
  13 ms after "FrameXML: ... loaded".
- **It writes back.** `setSettingValue` saves into `settings.cfg`, which is
  why a pushed `vsync=0` came back as `vsync=1` on the device.
- **It reverted every bound key we pushed**, so the runs above measured
  `view_distance=1000` (not 1600), `ground_clutter_density=60` (not 100),
  `ground_clutter_distance=110` (not 140), `particle_density=50` (not 100),
  `environment_detail=70` (not 100). The design is deliberate - the store
  is authoritative over the file by intent (`settings_panel.cpp:187`) - so
  the fix for measurement runs is to **push both files**: `gxvsync=0` in
  `cvars.cfg` alongside `vsync=0` in `settings.cfg`.
- With FIFO at 60 Hz a ~41 ms GPU frame quantises to exactly 3 vblanks =
  **50.00 ms = 20.00 fps** - the number reproduced across every run. The
  "step, not decay" 30 -> 20 observation above is the same quantisation.

### Instrumented run: fence vs acquire, three images, 60 fps (2026-11-13)

`beginFrame` now times its two waits separately (10 s windows, gated on
`frameprofile`), and the swapchain asks for 3 images instead of 2 (two
images + two frames in flight means the acquire waits for the display -
one frame in flight in practice). Release, both configs pushed with
`vsync=0`/`gxvsync=0`, present mode stayed **IMMEDIATE** through world
entry (zero `VSync enabled` lines).

**Standing still: 60 fps.** Four settled windows at 59.999 / 60.005 /
59.983 / 60.003 fps (16.667 ms), against 20.00 fps before. The waits:

- **acquire avg 0.016-0.04 ms, max 0.16 ms** - dead, as intended.
- **fence avg 3.7-4.0 ms** - the remaining frame-slot wait.
- `beginFrame` 8.8 ms/frame (52%) at 60 fps; `endFrame` 0.06 ms.

GPU last frame at 60 fps: **15.29 ms** across 5 passes (world 8.27,
shadows 4.55, fog 1.63, post 0.71, interface 0.14). When the camera moves
fps falls to 30-44 and the world pass grows to 21 ms (GPU 33 ms total) -
so the standing-still frame is done and the remaining work is the world
pass under motion, not the pipeline. 0 device lost over the run.

### Open performance work (next session)

Standing still is at 60 fps; the frame is no longer serialised. What
remains, in order:

1. **The world pass under motion** - 8.3 ms still -> 21 ms with a moving
   camera (fps 30-44). The pass ablation under `asyncsubmit=1` (the table
   above predates it and was measured while the CPU was serialised in
   submit) is the tool: attribute that 21 ms - WMO (3.5 ms still), water,
   particles, Hi-Z work, or queue starvation.
2. **The fence wait** - 3.7-4.0 ms/frame avg at 60 fps, the whole
   remaining `beginFrame` cost. Worth knowing what the GPU is still doing
   4 ms after the CPU is ready; may fall out of (1).
3. **Shadows** (6.15 ms wall-clock / 11 ms of mark in the old table;
   `shadow_distance=350`, `extShadowQuality` iOS default 1024) and
   **volumetric fog** (2.7 ms) - the two measured scene levers, still
   untested on Release with the pipeline unserialized.
4. **View distance** (`view_distance` effective 1000, never the pushed
   1600 - see the revert finding) and the internal resolution (786x546;
   lower is more fps). The levers still untested on Release at all: 16x
   aniso / particles 100% / water refraction, and a working pipeline
   cache on iOS.
5. Hi-Z occlusion culling is disabled at `renderer.cpp:3764` -
   re-enabling it is a draw-count lever the ablation cannot see (it only
   switches passes off, it does not add culling).
6. ~~`glslc` for the 8 missing shaders~~ - **done**: the vcpkg `shaderc`
   port (classic mode, `pkg-config` on `PATH`) provides `glslc`, and
   `-DGLSLC=...` opens CMake's gate (method in `ios/README.md`,
   "Shaders"). The first build recompiled all 90 shaders and rewrote the
   84 committed `.spv` (same source, different compiler revision) -
   restored with `git checkout -- assets/shaders/`; later builds are
   incremental. Verified on device: the 20:17:00 launch logged **0
   ERROR lines**, where every earlier launch failed on
   `grass_cull.comp.spv`, `m2_cull_hiz.comp.spv` and
   `world_map_fog.frag.spv` - grass and M2 occlusion culling now have
   their pipelines, so re-measure the world pass under motion with
   culling on. Open follow-up: the eight `.spv` are gitignored, so a
   clone without `glslc` still lacks them - force-add them or generate
   them in CI before the PR.
7. Measurement hygiene: **push both `settings.cfg` and `cvars.cfg`** to
   the device, or the CVar store silently reverts the bound keys.

`tools/ios-fsr-measure.sh` measures against a baseline and prints both.
