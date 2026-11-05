# iPadOS Port Plan

**Status:** scaffold. Branch `ipados`. No device build yet.
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
| Files/storage | `getExternalFilesDir()` | App sandbox + File Sharing (iTunes/Finder) or iCloud |
| Touch | `src/ui/touch_controls.cpp` | reuse same module |

## 3. Blockers

1. **No macOS/Xcode in this environment.** All builds must happen on a Mac.
   This repo's CI cannot build iOS; only scaffold + cross-compile headers.
2. **MoltenVK.** Need MoltenVK.framework for iOS (arm64) and to link it.
   Vulkan headers via vcpkg (already supported for Darwin cross-compile).
3. **Code signing & provisioning.** Requires an Apple Developer account for
   device install / TestFlight.
4. **Touch UX.** Reuse Android's `touch_controls.cpp`; iPad has larger screen
   and (optionally) keyboard/trackpad/mouse. Add pointer support.
5. **App lifecycle / background.** iOS suspends aggressively; need to handle
   `SDL_APP_WILLENTERBACKGROUND` to pause network/render.
6. **Large data (35 GB).** Same constraint as Android: player copies `Data/`
   into the app sandbox via File Sharing. Document clearly.

## 4. Work breakdown

- Phase 0 (done): scaffold `ios/`, `docs/plan-ipados.md`, CMake iOS guards,
  `src/core/ios_platform.{hpp,mm}`.
- Phase 1: CMake iOS toolchain file (`cmake/ios.toolchain.cmake`), build
  `libwowee` + MoltenVK on a Mac, verify link.
- Phase 2: Xcode app bundle, Info.plist, LaunchScreen, SDL main glue,
  first run to login screen.
- Phase 3: touch controls (reuse Android module), soft keyboard, safe areas.
- Phase 4: lifecycle handling, background pause, memory warnings.
- Phase 5: TestFlight/App Store packaging, release workflow.

## 5. GitHub workflow

Recommended: fork `Kelsidavis/WoWee` to personal account, develop on `ipados`
branch, open PR upstream when stable. See README/CONTRIBUTING.md.

## 6. Open questions

- Use SwiftUI/UIKit app shell or SDL's default UIKit main? Prefer SDL's.
- Mouse/trackpad support on iPad (pointer) - leverage SDL mouse events.
- Whether to ship as iPad-only or universal (iPhone). Start iPad-only.
```

Now create cmake/ios.toolchain.cmake.<|end|><|start|>startup to=write<|channel|>analysis <|constrain|>xml<|message|><rfl:parameter name="path">
/home/stangal01/WoWee/cmake/ios.toolchain.cmake