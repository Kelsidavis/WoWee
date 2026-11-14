#pragma once

namespace wowee {
namespace core {

/// How hot the device says it is, as a name: "nominal", "fair", "serious",
/// "critical", or "unknown" where the OS has no such call.
///
/// NSProcessInfo.thermalState on Apple platforms - the same reading
/// Instruments' energy log is built on - so a slow frame can be told apart
/// from a throttled device at a glance, in the performance overlay, without
/// a profiler attached.
#if defined(__APPLE__)
const char* thermalStateName();
#else
inline const char* thermalStateName() { return "unknown"; }
#endif

} // namespace core
} // namespace wowee
