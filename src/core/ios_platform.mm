#include "core/ios_platform.hpp"

#include "core/env.hpp"
#include "core/logger.hpp"
#include "core/thermal_state.hpp"
#include <cstdlib>
#include <cstring>

#if defined(__APPLE__)
#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>
#import <Photos/Photos.h>
#include <SDL3/SDL_system.h>
#include <SDL3/SDL_video.h>

extern "C" int SDL_main(int argc, char* argv[]);
extern "C" void SDL_SetMainReady(void);

// Globals for argc/argv, accessible from the scene delegate.
int gIosArgc = 0;
char** gIosArgv = nullptr;

// Custom scene delegate in the main binary, so UIKit can always find it.
@interface WoWeeSceneDelegate : UIResponder <UISceneDelegate>
@end

@implementation WoWeeSceneDelegate
- (void)scene:(UIScene *)scene willConnectToSession:(UISceneSession *)session options:(UISceneConnectionOptions *)connectionOptions {
    if (![scene isKindOfClass:[UIWindowScene class]]) return;

    // Order probe: if this logs *after* "Window initialized", SDL created
    // its UIWindow before the scene existed - a window with no scene is
    // never displayed on iOS 13+, which is a black screen.
    //
    // Point the log at Documents/config first: this callback runs before
    // SDL_main's iosSetupDataPaths, and the logger latches its file on the
    // first line it writes - without this the rest of the run lands in
    // Library/Logs where nobody looks.
    if (!getenv("WOWEE_CONFIG_ROOT")) {
        NSArray* paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
        NSString* docs = paths.firstObject;
        if (docs) {
            NSString* config = [docs stringByAppendingPathComponent:@"config"];
            [[NSFileManager defaultManager] createDirectoryAtPath:config
                                      withIntermediateDirectories:YES
                                                       attributes:nil
                                                            error:nil];
            wowee::core::setEnvVar("WOWEE_CONFIG_ROOT", config.UTF8String);
        }
    }
    LOG_WARNING("Scene willConnect: ", (void*)scene,
                " state=", (int)scene.activationState);

    NSBundle *bundle = [NSBundle mainBundle];
    [[NSFileManager defaultManager] changeCurrentDirectoryPath:[bundle resourcePath]];

    // Do NOT create a launch window or call SDL_main here.  SDL's app
    // delegate (didFinishLaunchingWithOptions) handles both: it creates
    // the launch window and schedules postFinishLaunch -> SDL_main.  Doing
    // it here too caused a second (stuck) launch window and a blocked
    // main thread.
}

// SDL's own scene delegate forwards these to SDL so the app lifecycle
// (foreground/background events, event pump state) tracks the scene.  We
// replace that delegate, so forward them here or SDL never learns the
// scene became active.
- (void)sceneDidBecomeActive:(UIScene *)scene {
    SDL_OnApplicationDidEnterForeground();
}
- (void)sceneWillResignActive:(UIScene *)scene {
    SDL_OnApplicationWillEnterBackground();
}
- (void)sceneWillEnterForeground:(UIScene *)scene {
    SDL_OnApplicationWillEnterForeground();
}
- (void)sceneDidEnterBackground:(UIScene *)scene {
    SDL_OnApplicationDidEnterBackground();
}
@end
#endif

namespace wowee {
namespace core {

namespace {
void (*gMemoryWarningCallback)() = nullptr;
}

void iosPreparePlatform() {
#if defined(__APPLE__)
    @autoreleasepool {
        [UIApplication sharedApplication].idleTimerDisabled = YES;

        // SDL creates its UIWindow in SDL_CreateWindow, and UIKit only
        // attaches a window to a scene that is already connected - a window
        // created before UIWindowScene connects gets no scene and is never
        // displayed (black screen while everything else runs).  SDL_main can
        // start before the system connects the scene, so pump the run loop
        // here until one exists.
        UIScene* scene = nil;
        for (UIScene* s in [UIApplication sharedApplication].connectedScenes) {
            if ([s isKindOfClass:[UIWindowScene class]]) { scene = s; break; }
        }
        if (!scene) {
            LOG_WARNING("UIWindowScene not connected yet - waiting");
            NSDate* deadline = [NSDate dateWithTimeIntervalSinceNow:3.0];
            while (!scene && [deadline timeIntervalSinceNow] > 0.0) {
                [[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode
                                         beforeDate:[NSDate dateWithTimeIntervalSinceNow:0.05]];
                for (UIScene* s in [UIApplication sharedApplication].connectedScenes) {
                    if ([s isKindOfClass:[UIWindowScene class]]) { scene = s; break; }
                }
            }
            LOG_WARNING(scene ? "UIWindowScene connected after pumping the run loop"
                              : "UIWindowScene STILL missing after 3s - window will not display");
        } else {
            LOG_WARNING("UIWindowScene already connected before window create");
        }

        [[NSNotificationCenter defaultCenter]
            addObserverForName:UIApplicationDidReceiveMemoryWarningNotification
                        object:nil
                         queue:[NSOperationQueue mainQueue]
                    usingBlock:^(NSNotification* /*note*/) {
                        if (gMemoryWarningCallback) gMemoryWarningCallback();
                    }];

        LOG_INFO("iOS platform prepared");
    }
#endif
}

void iosSetupDataPaths() {
#if defined(__APPLE__)
    @autoreleasepool {
        NSArray* paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
        NSString* docs = paths.firstObject;
        if (!docs) return;

        NSString* data = [docs stringByAppendingPathComponent:@"Data"];
        NSString* config = [docs stringByAppendingPathComponent:@"config"];

        [[NSFileManager defaultManager] createDirectoryAtPath:data
            withIntermediateDirectories:YES attributes:nil error:nil];
        [[NSFileManager defaultManager] createDirectoryAtPath:config
            withIntermediateDirectories:YES attributes:nil error:nil];

        // The client finds an expansion by scanning Data/expansions/ for a
        // directory holding an expansion.json, and the tables themselves are
        // copied into place by syncClientTables once the directory exists.
        //
        // So all this has to do is make the directories. Without them the
        // client's tables have nowhere to go, no expansion is found, and a
        // game data folder the player copied in is never opened.
        NSString* bundleData = [[[NSBundle mainBundle] resourcePath]
            stringByAppendingPathComponent:@"Data"];
        NSString* bundleExpansions = [bundleData stringByAppendingPathComponent:@"expansions"];
        NSFileManager* fm = [NSFileManager defaultManager];
        for (NSString* entry in [fm contentsOfDirectoryAtPath:bundleExpansions error:nil]) {
            NSString* to = [data stringByAppendingPathComponent:
                [@"expansions" stringByAppendingPathComponent:entry]];
            [fm createDirectoryAtPath:to withIntermediateDirectories:YES
                           attributes:nil error:nil];
        }

        NSString* resources = [[NSBundle mainBundle] resourcePath];
        if (resources) {
            wowee::core::setEnvVar("WOWEE_RESOURCE_ROOT", resources.UTF8String);
        }
        wowee::core::setEnvVar("WOW_DATA_PATH", data.UTF8String);
        wowee::core::setEnvVar("WOWEE_CONFIG_ROOT", config.UTF8String);

        // The extracted game data is tens of gigabytes and the player can
        // always extract it again, so it must not go into the iCloud backup:
        // that would fill the user's iCloud quota with a copy of their own
        // WoW install and slow every backup down. The flag is a resource
        // value on the directory, so it has to be set on the directory itself
        // rather than on its contents.
        NSURL* dataUrl = [NSURL fileURLWithPath:data isDirectory:YES];
        [dataUrl setResourceValue:@YES forKey:NSURLIsExcludedFromBackupKey error:nil];

        LOG_INFO("iOS data root: ", docs.UTF8String);
        LOG_INFO("iOS resource root: ", resources ? resources.UTF8String : "(null)");
    }
#endif
}

void iosSetMemoryWarningCallback(void (*callback)()) {
#if defined(__APPLE__)
    gMemoryWarningCallback = callback;
#endif
}

void iosLogWindowStatus(const char* stage) {
#if defined(__APPLE__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
    static dispatch_once_t once;
    dispatch_once(&once, ^{
        // An external display can be attached after launch. Refresh the
        // portrait/landscape dump at that moment, otherwise only the startup
        // samples exist and a mid-run rotation has no diagnostic to read.
        [NSNotificationCenter.defaultCenter
            addObserverForName:UIScreenDidConnectNotification
                        object:nil queue:NSOperationQueue.mainQueue
                   usingBlock:^(NSNotification*) { iosLogWindowStatus("screen+"); }];
        [NSNotificationCenter.defaultCenter
            addObserverForName:UIScreenDidDisconnectNotification
                        object:nil queue:NSOperationQueue.mainQueue
                   usingBlock:^(NSNotification*) { iosLogWindowStatus("screen-"); }];
    });
    @autoreleasepool {
        // Dump every UIWindow in every connected scene, plus (via deprecated
        // but functional UIApplication APIs, silenced locally) any window that
        // belongs to NO scene - SDL's window ends up in one of these buckets,
        // and a black screen means it is in the wrong one.
        NSMutableString* out = [NSMutableString
            stringWithFormat:@"Window status [%s]:", stage ? stage : "?"];

        // Every UIScreen UIKit knows about. An external display either gets
        // its own UIScene (multiple scenes enabled) or the main scene is
        // mirrored to it - the two cases need different fixes when the
        // picture is rotated, so log both the screens and the scenes.
        NSArray<UIScreen*>* screens = [UIScreen screens];
        [out appendFormat:@" %d screen(s)", (int)screens.count];
        for (UIScreen* s in screens) {
            [out appendFormat:@"\n  screen %p main=%d bounds=%@ mode=%@",
                               (void*)s, s == [UIScreen mainScreen] ? 1 : 0,
                               NSStringFromCGRect(s.bounds),
                               NSStringFromCGSize(s.currentMode.size)];
            // Log the modes an external screen offers. A landscape monitor
            // whose selected mode is portrait-ordered (972x1518) may have a
            // landscape-ordered mode (1518x972) we can switch to, which would
            // let SDL build the swapchain in the right orientation.
            if (s != [UIScreen mainScreen]) {
                for (UIScreenMode* mode in s.availableModes) {
                    [out appendFormat:@"\n    mode=%@ available=%d",
                                       NSStringFromCGSize(mode.size),
                                       mode == s.currentMode ? 1 : 0];
                }
            }
        }
#pragma clang diagnostic pop

        NSSet<UIScene*>* scenes = [UIApplication sharedApplication].connectedScenes;
        [out appendFormat:@" %d scene(s)", (int)scenes.count];
        for (UIScene* s in scenes) {
            if (![s isKindOfClass:[UIWindowScene class]]) {
                [out appendFormat:@"\n  scene %p (not a UIWindowScene)", (void*)s];
                continue;
            }
            UIWindowScene* ws = (UIWindowScene*)s;
            [out appendFormat:@"\n  scene %p state=%d bounds=%@ windows=%d",
                               (void*)s, (int)ws.activationState,
                               NSStringFromCGRect(ws.coordinateSpace.bounds),
                               (int)ws.windows.count];
            for (UIWindow* w in ws.windows) {
                [out appendFormat:@"\n    <%@ %p> frame=%@ hidden=%d alpha=%.2f "
                                    @"level=%.1f key=%d root=%@",
                                   NSStringFromClass([w class]), (void*)w,
                                   NSStringFromCGRect(w.frame), w.hidden ? 1 : 0,
                                   w.alpha, w.windowLevel, w.isKeyWindow ? 1 : 0,
                                   w.rootViewController
                                       ? NSStringFromClass([w.rootViewController class])
                                       : @"nil"];
            }
        }
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
        // A window created without a scene never shows up in scene.windows -
        // this is where SDL's window would be if UIKit_GetActiveWindowScene()
        // returned nil at creation time.
        NSArray<UIWindow*>* allWindows = [UIApplication sharedApplication].windows;
        [out appendFormat:@"\n  UIApplication.windows: %d", (int)allWindows.count];
        for (UIWindow* w in allWindows) {
            [out appendFormat:@"\n    <%@ %p> scene=%p hidden=%d key=%d",
                               NSStringFromClass([w class]), (void*)w,
                               (void*)w.windowScene, w.hidden ? 1 : 0,
                               w.isKeyWindow ? 1 : 0];
        }
#pragma clang diagnostic pop
        // LOG_WARNING, not LOG_INFO: the config file drops the runtime level
        // to warning before the window exists, so an INFO line never shows up.
        LOG_WARNING(out.UTF8String);

        // One follow-up sample a few seconds later, once the app has settled:
        // the launch window fades out only after the main run loop starts
        // spinning, so the first sample can legitimately still show it.
        if (!stage || strncmp(stage, "t+", 2) != 0) {
            dispatch_after(dispatch_time(DISPATCH_TIME_NOW, (int64_t)(3 * NSEC_PER_SEC)),
                           dispatch_get_main_queue(), ^{
                             iosLogWindowStatus("t+3s");
                           });
        }
    }
#endif
}

void* iosFirstUiWindowScene() {
#if defined(__APPLE__)
    @autoreleasepool {
        for (UIScene* s in [UIApplication sharedApplication].connectedScenes) {
            if ([s isKindOfClass:[UIWindowScene class]]) {
                return (__bridge void*)s;
            }
        }
    }
#endif
    return nullptr;
}

void iosAttachWindowToScene(void* sdlWindow) {
#if defined(__APPLE__)
    @autoreleasepool {
        if (!sdlWindow) {
            LOG_WARNING("Scene attach: no SDL window");
            return;
        }
        SDL_Window* w = static_cast<SDL_Window*>(sdlWindow);
        SDL_PropertiesID props = SDL_GetWindowProperties(w);
        UIWindow* uiw = (__bridge UIWindow*)SDL_GetPointerProperty(
            props, SDL_PROP_WINDOW_UIKIT_WINDOW_POINTER, nullptr);
        if (!uiw) {
            LOG_WARNING("Scene attach: SDL window has no UIKit window");
            return;
        }
        UIWindowScene* scene = nil;
        for (UIScene* s in [UIApplication sharedApplication].connectedScenes) {
            if ([s isKindOfClass:[UIWindowScene class]]) {
                scene = (UIWindowScene*)s;
                break;
            }
        }
        if (!scene) {
            LOG_WARNING("Scene attach: no UIWindowScene connected");
            return;
        }
        if (uiw.windowScene != scene) {
            // SDL 3.2.24 never does this; assigning is what puts the window
            // into scene.windows, without which iOS never renders it.
            uiw.windowScene = scene;
        }
        [uiw makeKeyAndVisible];
        LOG_WARNING("Scene attach: uiwindow=", (void*)uiw,
                    " scene=", (void*)uiw.windowScene,
                    " hidden=", uiw.hidden ? 1 : 0,
                    " key=", uiw.isKeyWindow ? 1 : 0,
                    " frame=", NSStringFromCGRect(uiw.frame));
    }
#endif
}

void iosLogSdlWindow(void* sdlWindow) {
#if defined(__APPLE__)
    @autoreleasepool {
        if (!sdlWindow) {
            LOG_WARNING("SDL window probe: no SDL window");
            return;
        }
        SDL_Window* w = static_cast<SDL_Window*>(sdlWindow);
        SDL_PropertiesID props = SDL_GetWindowProperties(w);
        UIWindow* uiw = (__bridge UIWindow*)SDL_GetPointerProperty(
            props, SDL_PROP_WINDOW_UIKIT_WINDOW_POINTER, nullptr);
        if (!uiw) {
            LOG_WARNING("SDL window probe: SDL has no UIKit window in its properties");
            return;
        }
        LOG_WARNING("SDL window probe: uiwindow=", (void*)uiw,
                    " scene=", (void*)uiw.windowScene,
                    " hidden=", uiw.hidden ? 1 : 0,
                    " key=", uiw.isKeyWindow ? 1 : 0,
                    " frame=", NSStringFromCGRect(uiw.frame),
                    " superlayer=", (void*)uiw.layer.superlayer);
    }
#endif
}

void iosStoreArgs(int argc, char** argv) {
#if defined(__APPLE__)
    gIosArgc = argc;
    gIosArgv = argv;
#endif
}

void iosShowAlert(const char* title, const char* message) {
#if defined(__APPLE__)
    @autoreleasepool {
        dispatch_async(dispatch_get_main_queue(), ^{
            UIAlertController* alert = [UIAlertController
                alertControllerWithTitle:(title ? [NSString stringWithUTF8String:title] : @"WoWee")
                               message:(message ? [NSString stringWithUTF8String:message] : @"")
                        preferredStyle:UIAlertControllerStyleAlert];
            [alert addAction:[UIAlertAction actionWithTitle:@"OK"
                                                     style:UIAlertActionStyleDefault
                                                   handler:nil]];
            UIWindow* keyWindow = nil;
            if (@available(iOS 13.0, *)) {
                for (UIScene* scene in [UIApplication sharedApplication].connectedScenes) {
                    if ([scene isKindOfClass:[UIWindowScene class]] && scene.activationState == UISceneActivationStateForegroundActive) {
                        NSArray<UIWindow*>* ws = ((UIWindowScene*)scene).windows;
                        if (ws.count > 0) { keyWindow = ws.firstObject; break; }
                    }
                }
            }
            if (!keyWindow) {
                for (UIScene* scene in [UIApplication sharedApplication].connectedScenes) {
                    if ([scene isKindOfClass:[UIWindowScene class]]) {
                        NSArray<UIWindow*>* ws = ((UIWindowScene*)scene).windows;
                        if (ws.count > 0) { keyWindow = ws.firstObject; break; }
                    }
                }
            }
            UIViewController* root = keyWindow.rootViewController;
            if (root) [root presentViewController:alert animated:YES completion:nil];
        });
    }
#endif
}

const char* iosSceneDelegateClassName() {
    return "WoWeeSceneDelegate";
}

void iosShareFile(const char* path) {
#if defined(__APPLE__)
    if (!path || !*path) return;
    @autoreleasepool {
        NSString* file = [NSString stringWithUTF8String:path];
        NSURL* url = [NSURL fileURLWithPath:file];
        if (!url || ![[NSFileManager defaultManager] fileExistsAtPath:file]) {
            LOG_WARNING("Share: no such file: ", file);
            return;
        }

        dispatch_async(dispatch_get_main_queue(), ^{
            UIActivityViewController* sheet =
                [[UIActivityViewController alloc] initWithActivityItems:@[url]
                                                 applicationActivities:nil];

            // On iPad a share sheet is a popover and needs an anchor, or UIKit
            // throws instead of presenting it.
            UIWindow* keyWindow = nil;
            for (UIScene* scene in [UIApplication sharedApplication].connectedScenes) {
                if ([scene isKindOfClass:[UIWindowScene class]] &&
                    scene.activationState == UISceneActivationStateForegroundActive) {
                    keyWindow = ((UIWindowScene*)scene).windows.firstObject;
                    if (keyWindow) break;
                }
            }
            UIViewController* root = keyWindow.rootViewController;
            if (!root) return;
            if (sheet.popoverPresentationController) {
                sheet.popoverPresentationController.sourceView = root.view;
                sheet.popoverPresentationController.sourceRect =
                    CGRectMake(CGRectGetMidX(root.view.bounds),
                               CGRectGetMidY(root.view.bounds), 1, 1);
                sheet.popoverPresentationController.permittedArrowDirections = 0;
            }
            [root presentViewController:sheet animated:YES completion:nil];
        });
    }
#endif
}

void iosSaveScreenshotToPhotos(const char* path) {
#if defined(__APPLE__)
    if (!path || !*path) return;
    @autoreleasepool {
        NSString* file = [NSString stringWithUTF8String:path];
        if (![[NSFileManager defaultManager] fileExistsAtPath:file]) {
            LOG_WARNING("Photos: no such file: ", file);
            return;
        }

        // PHPhotoLibrary is asked for add-only access; the prompt appears on
        // its own the first time. A denial is not an error worth failing the
        // screenshot over - the file is on disk either way.
        [PHPhotoLibrary requestAuthorizationForAccessLevel:PHAccessLevelAddOnly
                                                   handler:^(PHAuthorizationStatus status) {
            if (status != PHAuthorizationStatusAuthorized &&
                status != PHAuthorizationStatusLimited) {
                LOG_WARNING("Photos: not authorized (", (int)status,
                            "); the screenshot is only in the app's Documents");
                return;
            }
            NSURL* url = [NSURL fileURLWithPath:file];
            [[PHPhotoLibrary sharedPhotoLibrary] performChanges:^{
                [PHAssetChangeRequest creationRequestForAssetFromImageAtFileURL:url];
            } completionHandler:^(BOOL success, NSError* error) {
                if (success) {
                    LOG_INFO("Photos: screenshot saved to the photo library");
                } else {
                    LOG_WARNING("Photos: could not save the screenshot: ",
                                error ? error.localizedDescription.UTF8String : "unknown");
                }
            }];
        }];
    }
#endif
}

const char* thermalStateName() {
    // What the OS is doing about heat right now, in the same words
    // Instruments uses: nominal is cool, critical is the system pulling
    // clocks down as far as it can. Sampled per call rather than cached -
    // the point of the number is that it moves while the game is running.
    switch ([NSProcessInfo processInfo].thermalState) {
        case NSProcessInfoThermalStateNominal:  return "nominal";
        case NSProcessInfoThermalStateFair:     return "fair";
        case NSProcessInfoThermalStateSerious:  return "serious";
        case NSProcessInfoThermalStateCritical: return "critical";
        default:                                return "unknown";
    }
}

} // namespace core
} // namespace wowee