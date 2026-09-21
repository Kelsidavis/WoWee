# AMD FSR2 Integration Notes

The Upscaling setting has three modes: `Off`, `FSR 1` (spatial; the in-tree `fsr_easu`/`fsr_rcas` shaders) and `FSR 3` (temporal; the FSR2 path in `PostProcessPipeline`). The FSR 3 mode has two backends at runtime:

- `AMD FidelityFX SDK` backend (used when compiled in and its context creates).
- `Internal fallback` backend (used when AMD SDK prerequisites are not met, which is the default build).

The AMD SDK backends are off by default in CMake. They give neither a performance nor a quality gain here today, and FSR3 frame generation is broken on RADV/Mesa (`54a1a9cd`).

## SDK Location

AMD SDK checkout path:

`extern/FidelityFX-FSR2`

FidelityFX SDK checkout path (framegen extern):

`extern/FidelityFX-SDK` (default branch `main` from WoWee's fork in build scripts)

`build.sh` / `rebuild.sh` (and the PowerShell equivalents) clone both when missing, but the default configure does not use them until one of the build flags below is turned on.

Override knobs for local build scripts:

- `WOWEE_FFX_SDK_REPO` (default: `https://github.com/Kelsidavis/FidelityFX-SDK.git`)
- `WOWEE_FFX_SDK_REF` (default: `main`)

Detection expects:

- `extern/FidelityFX-FSR2/src/ffx-fsr2-api/ffx_fsr2.h`
- `extern/FidelityFX-FSR2/src/ffx-fsr2-api/vk/shaders/ffx_fsr2_accumulate_pass_permutations.h`
- If permutation headers are missing in the SDK checkout, WoWee CMake copies a vendored snapshot from:
  - `third_party/fsr2_vk_permutations`

## Build Flags

- `WOWEE_ENABLE_AMD_FSR2=OFF` (default): set ON to attempt AMD backend integration.
- `WOWEE_ENABLE_AMD_FSR3_FRAMEGEN=OFF` (default): set ON to build the AMD FSR3 framegen interface probe when FidelityFX-SDK headers are present.
- `WOWEE_BUILD_AMD_FSR3_RUNTIME=OFF` (default): set ON (with the framegen flag) to add the `wowee_fsr3_official_runtime_copy` target, which builds the Path A runtime from `extern/FidelityFX-SDK/Kits` and copies it beside the binary.
- `WOWEE_HAS_AMD_FSR2` compile define:
  - `1` when AMD SDK prerequisites are present.
  - `0` when missing, in which case internal fallback remains active.
- `WOWEE_HAS_AMD_FSR3_FRAMEGEN` compile define:
  - `1` when FidelityFX-SDK FI/OF/FSR3+VK headers are detected.
  - `0` when headers are missing (probe target disabled).

Runtime note:

- The persisted `Frame generation` setting exists only in a build with `WOWEE_HAS_AMD_FSR3_FRAMEGEN=1`, and only with the FSR 3 mode selected.
- Frame generation is attempted only on AMD GPUs; on others the runtime is not created.
- macOS clears the saved frame generation flag at startup (there is no AMD runtime there); the upscaling mode itself is kept.
- Runtime loader is Path A only (official AMD runtime library).
- The Path A runtime build auto-runs `tools/generate_ffx_sdk_vk_permutations.sh` (when `bash` is found) to ensure required VK permutation headers exist for FSR2/FSR3 upscaler shader blobs. That script downloads DXC on Linux x86_64 and Windows (MSYS2) when it is missing; elsewhere set `DXC=/path/to/dxc` or put `dxc` in `PATH`.
- You can point to an explicit runtime binary with:
  - `WOWEE_FFX_SDK_RUNTIME_LIB=/absolute/path/to/libamd_fidelityfx_vk.so` (or `.dll` / `.dylib`).
- If no official runtime is found, frame generation is disabled cleanly (Path C).


## Current Status

- AMD FSR2 Vulkan dispatch path is integrated and used when compiled in.
- The internal fallback renders the scene at the reduced resolution without jitter and upscales it with the `fsr2_sharpen` (RCAS) pass. The `fsr2_motion`/`fsr2_accumulate` compute pipelines are built, but motion vectors are dispatched only on the AMD path and the accumulate pass is not dispatched.
- With the FSR 3 mode selected, the settings panel shows which backend was compiled in (`FSR3 backend: AMD FidelityFX SDK` or `Internal fallback`) and, when framegen headers are present, the runtime's status.
- FSR 3 works with MSAA: the scene target is multisampled and the render pass resolves colour and depth. Only a device without depth resolve has to drop MSAA, and the log says so.
- With FSR 3 and FXAA both on, the frame is upscaled, then sharpened by RCAS into FXAA's full-resolution scene image, then smoothed by FXAA.
- FSR 3 is refused (with a log line) on a device without `shaderStorageImageWriteWithoutFormat` and `shaderInt16`, and is only turned on once the player is in the world.
- The FSR 3 jitter sign is persisted (`fsr2_jitter_sign`); its settings row is built only in debug builds.

## FSR Defaults

- Upscaling mode default: `Off`
- Quality default: `Native (100%)`
- UI quality order: `Ultra Quality (77%)`, `Quality (67%)`, `Balanced (59%)`, `Native (100%)`
- Default sharpness: `1.6`
- Default FSR2 jitter sign: `0.38`
- Performance preset is intentionally removed.

## CI Notes

- CI configures with the defaults, so the AMD backends are off and no SDK is cloned.
- The Linux, macOS and Windows jobs still build `wowee_fsr2_amd_vk` and `wowee_fsr3_framegen_amd_vk_probe` when CMake generates them, and skip the step otherwise.
- Some upstream SDK checkouts do not include generated Vulkan permutation headers.
- WoWee bootstraps those headers from the vendored snapshot so AMD backend builds remain cross-platform and deterministic.
- If SDK headers are missing entirely, WoWee still falls back to the internal backend.
