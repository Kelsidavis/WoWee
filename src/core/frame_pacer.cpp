#include "core/frame_pacer.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <timeapi.h>
#endif

namespace wowee::core {
namespace {

/// The least the spin will ever be asked to cover.
///
/// A sleep returns late by an amount the platform decides and does not
/// promise, so the margin is measured at runtime rather than written down
/// here - this is only the floor, for a platform that manages to return on
/// time and would otherwise leave nothing to absorb the next one that does
/// not.
constexpr std::int64_t kMinSpinMarginNs = 200'000;   // 0.2ms

}  // namespace

std::int64_t FramePacer::nowNs() {
    // steady_clock, not high_resolution_clock. The latter is an alias for
    // system_clock on libstdc++, which is wall time: it steps when the
    // machine's clock is corrected, and a frame delta taken across the step
    // is wrong by the correction - or negative.
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

std::int64_t FramePacer::tickNs() {
    const std::int64_t now = nowNs();
    if (!started_) {
        started_ = true;
        lastNs_ = now;
        return 0;
    }
    const std::int64_t delta = now - lastNs_;
    lastNs_ = now;
    // Monotonic means this cannot go backwards, but a clock that is only
    // almost monotonic is a real thing on virtualised hardware, and one
    // negative frame moves everything that integrates over it the wrong way.
    return delta < 0 ? 0 : delta;
}

float FramePacer::toSeconds(std::int64_t deltaNs, float capSeconds) {
    const float seconds = static_cast<float>(static_cast<double>(deltaNs) / 1'000'000'000.0);
    return std::min(seconds, capSeconds);
}

void FramePacer::waitForCap(int fps) {
    if (fps <= 0 || !started_) return;

    // Integer division: at 144 this is 6'944'444ns, and the remainder is a
    // third of a microsecond. Computing the target in float seconds and
    // converting back was the other place precision went.
    const std::int64_t frameNs = 1'000'000'000LL / fps;
    const std::int64_t deadline = lastNs_ + frameNs;

    if (nowNs() >= deadline) return;   // already over budget; do not add to it

    // Sleep to within the current slack estimate, measuring how far past the
    // request the sleep actually returns. One sleep, not a loop: each one
    // costs another overshoot, so re-sleeping the remainder is how a wait
    // drifts further late the more careful it tries to be.
    const std::int64_t sleepUntil = deadline - sleepSlackNs_;
    const std::int64_t beforeSleep = nowNs();
    if (sleepUntil > beforeSleep) {
        const std::int64_t asked = sleepUntil - beforeSleep;
        std::this_thread::sleep_for(std::chrono::nanoseconds(asked));
        const std::int64_t overshoot = (nowNs() - beforeSleep) - asked;

        // Track the worst recent overshoot rather than the average: the
        // margin has to cover the bad frames, since those are the ones that
        // show. It decays so that a single hitch under load does not leave
        // the client spinning a millisecond a frame forever after.
        if (overshoot > sleepSlackNs_) {
            sleepSlackNs_ = std::min(overshoot, frameNs);   // never a whole frame
        } else {
            sleepSlackNs_ -= (sleepSlackNs_ - overshoot) / 32;
        }
        sleepSlackNs_ = std::max(sleepSlackNs_, kMinSpinMarginNs);
    }

    // The rest by hand. yield() rather than a bare loop so a single-core
    // machine, or one with the render thread pinned beside something else,
    // is not starved for the last of every frame.
    while (nowNs() < deadline) {
        std::this_thread::yield();
    }
}

SleepPrecisionScope::SleepPrecisionScope() {
#ifdef _WIN32
    // 1ms is what every timer-sensitive Windows program asks for and what
    // the scheduler is tuned to give. Failure is not worth reporting: the
    // frame pacer spins the last of each wait anyway, so a coarse sleep
    // costs CPU rather than accuracy.
    raised_ = (timeBeginPeriod(1) == TIMERR_NOERROR);
#endif
}

SleepPrecisionScope::~SleepPrecisionScope() {
#ifdef _WIN32
    if (raised_) timeEndPeriod(1);
#endif
}

}  // namespace wowee::core
