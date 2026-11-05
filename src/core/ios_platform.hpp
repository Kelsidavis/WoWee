#pragma once

namespace wowee {
namespace core {

// iPadOS/iOS platform shims.  Mirrors macos_platform.hpp.  Real implementations
// (status bar, safe areas, on-screen keyboard, app lifecycle) land here as the
// port progresses.
void iosPreparePlatform();

} // namespace core
} // namespace wowee