#include "core/ios_platform.hpp"

#include "core/logger.hpp"

#if defined(__APPLE__)
#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>
#endif

namespace wowee {
namespace core {

void iosPreparePlatform() {
#if defined(__APPLE__)
    @autoreleasepool {
        // Placeholder for iOS-specific setup (status bar, safe areas, keyboard).
        LOG_INFO("iOS platform prepared");
    }
#endif
}

} // namespace core
} // namespace wowee