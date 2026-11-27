#pragma once

namespace wowee {
namespace core {

// iPadOS/iOS platform shims.  Mirrors macos_platform.hpp.
void iosPreparePlatform();

// Point the client at the app's Documents directory, which is the location
// exposed over USB/Finder via UIFileSharingEnabled, so the player can copy
// their extracted Data/ in.  Sets WOWEE_RESOURCE_ROOT, WOW_DATA_PATH and
// WOWEE_CONFIG_ROOT, mirroring WoweeActivity on Android.
void iosSetupDataPaths();

// Register a callback invoked when the OS issues a memory warning.  The
// client uses this to drop caches (textures, DBC, file cache) before the
// system terminates it.  Pass nullptr to clear.
void iosSetMemoryWarningCallback(void (*callback)());

// Log every UIWindow in the active scene (class, frame, hidden, level,
// key state, root view controller) plus a follow-up sample 3s later.
// Diagnostic for black-screen bugs: shows whether something is covering
// the SDL window or the window never became visible.
void iosLogWindowStatus(const char* stage);

// First connected UIWindowScene as an opaque pointer (nil if none is
// connected yet).
void* iosFirstUiWindowScene();

// SDL 3.2.24 builds its UIWindow with initWithFrame: and never assigns a
// scene, so in a scene-based app it is never displayed.  Grab the UIWindow
// from SDL_PROP_WINDOW_UIKIT_WINDOW_POINTER, attach it to the connected
// UIWindowScene and make it key/visible.
void iosAttachWindowToScene(void* sdlWindow);

// Log the UIKit window SDL built for the given SDL_Window (read from
// SDL_PROP_WINDOW_UIKIT_WINDOW_POINTER): pointer, attached scene, frame,
// hidden/key state, layer parent. Diagnostic for black-screen bugs.
void iosLogSdlWindow(void* sdlWindow);

// Show a native alert dialog.  Used to surface missing-data warnings on
// iOS, where the client's ImGui UI may not be available yet.
void iosShowAlert(const char* title, const char* message);

// Present the iOS share sheet for a file the client just wrote (a screenshot
// or a recording), so the player can send it to Photos, AirDrop, Files or
// Mail without leaving the app.  No-op off iOS.
void iosShareFile(const char* path);

// Save a PNG into the device's photo library, so a screenshot taken in game
// shows up in the Photos app alongside the ones the tablet takes itself.
// Requires NSPhotoLibraryAddUsageDescription in Info.plist.  No-op off iOS.
void iosSaveScreenshotToPhotos(const char* path);

// Store argc/argv from main() so the scene delegate can invoke SDL_main.
void iosStoreArgs(int argc, char** argv);

// Minimal scene delegate class name for Info.plist.
extern const char* iosSceneDelegateClassName();

} // namespace core
} // namespace wowee