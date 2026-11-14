#include "core/macos_platform.hpp"

#include "core/logger.hpp"
#include "core/thermal_state.hpp"

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

const char* thermalStateName() {
    // The same reading iOS reports, and the one Instruments' energy log is
    // built on - sampled per call, so the performance overlay shows the
    // device moving between states while the game runs.
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
