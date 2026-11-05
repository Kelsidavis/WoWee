# WoWee on iPadOS / iOS

This directory is the iPadOS/iOS port scaffold. It mirrors `android/`: the shared
C++ core is built by the top-level `CMakeLists.txt` (with `CMAKE_SYSTEM_NAME=iOS`),
and this directory holds the Apple-side app shell, assets staging, and tooling.

## Status

Scaffold only. No Xcode project yet. See `docs/plan-ipados.md`.

## Planned layout

- `ios/WoWee.xcodeproj` (or `.xcworkspace`) - app bundle, signing, asset catalog
- `ios/WoWee/` - AppDelegate, SDL main glue, Info.plist, LaunchScreen
- `ios/Assets.xcassets` - icons, launch images
- `tools/build-ios-deps.sh` - builds OpenSSL + MoltenVK for iOS (arm64)

## Build (once scaffolded)

Requires macOS with Xcode and the iOS SDK. From the repo root:

```
cmake -S . -B build-ios \
  -DCMAKE_SYSTEM_NAME=iOS \
  -DCMAKE_OSX_ARCHITECTURES=arm64 \
  -DCMAKE_OSX_DEPLOYMENT_TARGET=16.0 \
  -DOPENSSL_ROOT_DIR=/path/to/ios-openssl
cmake --build build-ios
```

Then wrap the resulting `wowee` binary in an Xcode app bundle with MoltenVK.framework.

## Data

Same as desktop/Android: the player extracts game data on a desktop and copies
`Data/` onto the device (via Finder/iTunes file sharing or iCloud). Point the app
at it with `WOW_DATA_PATH`.
```

Now create docs/plan-ipados.md.<|end|><|start|>startup to=write<|channel|>analysis <|constrain|>xml<|message|><rfl:parameter name="path">
/home/stangal01/WoWee/docs/plan-ipados.md