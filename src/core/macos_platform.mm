#include "core/macos_platform.hpp"

#include "core/logger.hpp"

#import <Foundation/Foundation.h>

namespace wowee {
namespace core {

void disablePressAndHoldAccents() {
    @autoreleasepool {
        NSUserDefaults* defaults = [NSUserDefaults standardUserDefaults];
        [defaults registerDefaults:@{@"ApplePressAndHoldEnabled": @NO}];

        // The registration domain is searched last, so an explicit global
        // "defaults write -g ApplePressAndHoldEnabled -bool true" would still
        // win.  Fall back to this process' own application domain, which
        // outranks NSGlobalDomain.
        if ([defaults boolForKey:@"ApplePressAndHoldEnabled"]) {
            [defaults setBool:NO forKey:@"ApplePressAndHoldEnabled"];
            LOG_INFO("Press-and-hold accents were enabled globally; "
                     "overriding for this application");
        }
    }
}

} // namespace core
} // namespace wowee
