/// Frame timing at high refresh rates.
///
/// At 60Hz a frame is 16.7ms and a millisecond of error anywhere is noise.
/// At 144Hz the budget is 6.94ms and the same millisecond is fourteen per
/// cent of it, which is what these check for.

#include <catch_amalgamated.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <thread>

#include "core/frame_pacer.hpp"

using wowee::core::FramePacer;

TEST_CASE("the clock moves forward and only forward", "[pacing]") {
    const std::int64_t a = FramePacer::nowNs();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    const std::int64_t b = FramePacer::nowNs();
    CHECK(b > a);
    // Two milliseconds of sleep cannot read as less than one of elapsed
    // time on any clock worth using.
    CHECK(b - a > 1'000'000);
}

TEST_CASE("the first tick has no previous frame to measure from", "[pacing]") {
    FramePacer pacer;
    CHECK(pacer.tickNs() == 0);
}

TEST_CASE("a tick measures the gap since the last one", "[pacing]") {
    FramePacer pacer;
    (void)pacer.tickNs();   // the baseline
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    const std::int64_t delta = pacer.tickNs();
    CHECK(delta > 4'000'000);    // at least 4ms
    CHECK(delta < 100'000'000);  // and not absurd
}

TEST_CASE("the frame start is the reading the delta was taken against", "[pacing]") {
    FramePacer pacer;
    (void)pacer.tickNs();
    const std::int64_t first = pacer.lastTickNs();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    (void)pacer.tickNs();
    const std::int64_t second = pacer.lastTickNs();
    // Not a fresh clock reading: it is the one the delta was measured from,
    // so the cap and the delta cannot disagree about where the frame began.
    CHECK(second > first);
    CHECK(FramePacer::nowNs() >= second);
}

TEST_CASE("seconds come out of nanoseconds, and a stall is capped", "[pacing]") {
    // 144Hz, to the nanosecond.
    CHECK(FramePacer::toSeconds(6'944'444) == Catch::Approx(0.006944444f).epsilon(0.0001));
    CHECK(FramePacer::toSeconds(16'666'667) == Catch::Approx(0.016666667f).epsilon(0.0001));
    CHECK(FramePacer::toSeconds(0) == Catch::Approx(0.0f));

    // A machine back from sleep hands over an enormous delta, and everything
    // that integrates over it would teleport.
    CHECK(FramePacer::toSeconds(5'000'000'000) == Catch::Approx(0.1f));
    CHECK(FramePacer::toSeconds(5'000'000'000, 0.25f) == Catch::Approx(0.25f));
}

TEST_CASE("no cap means no wait", "[pacing]") {
    FramePacer pacer;
    (void)pacer.tickNs();
    const std::int64_t start = FramePacer::nowNs();
    pacer.waitForCap(0);
    pacer.waitForCap(-1);
    // Returning at all is the check; a wait here would be an unbounded one.
    CHECK(FramePacer::nowNs() - start < 50'000'000);
}

TEST_CASE("a pacer that has never ticked has no frame to pace", "[pacing]") {
    FramePacer pacer;
    const std::int64_t start = FramePacer::nowNs();
    pacer.waitForCap(144);   // no baseline yet: must not wait on lastNs_ of 0
    CHECK(FramePacer::nowNs() - start < 2'000'000);
}

TEST_CASE("a frame already over budget is not delayed further", "[pacing]") {
    // A frame that took 50ms has blown a 144Hz budget seven times over.
    // Waiting any longer would turn one slow frame into two.
    // The loop's order: tick ends a frame, the next frame's work runs, then
    // the cap is asked to hold. No second tick - that is what would move the
    // frame start forward and hide the overrun.
    FramePacer pacer;
    (void)pacer.tickNs();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));   // slow frame
    const std::int64_t before = FramePacer::nowNs();
    pacer.waitForCap(144);
    CHECK(FramePacer::nowNs() - before < 2'000'000);
}

/// The best a plain sleep to this deadline manages on the machine running the
/// test.
///
/// The overshoot bound below is measured against this rather than written
/// down. A desktop returns from a sleep about a millisecond late; a shared CI
/// runner was seen three milliseconds late on every attempt, which is the
/// scheduler's quantum and nothing the pacer can undercut. A constant either
/// fails there or proves nothing here. Best of several, because the worst of
/// several on a loaded machine is unbounded.
std::int64_t plainSleepOvershootNs(std::int64_t budgetNs) {
    std::int64_t best = budgetNs;
    for (int i = 0; i < 5; ++i) {
        const std::int64_t start = FramePacer::nowNs();
        std::this_thread::sleep_for(std::chrono::nanoseconds(budgetNs));
        best = std::min(best, FramePacer::nowNs() - start - budgetNs);
    }
    return std::max<std::int64_t>(best, 0);
}

TEST_CASE("the cap holds a 144Hz frame to its budget", "[pacing]") {
    constexpr std::int64_t kFrameNs = 1'000'000'000LL / 144;   // 6'944'444
    // How late a plain sleep has to land before the tight bound stops meaning
    // anything: past this the scheduler's quantum is wider than the margin
    // being measured.
    constexpr std::int64_t kUntestable = 1'500'000;

    // Up to three attempts, because both halves of the comparison are
    // measured on a machine shared with whatever else is running. A CI run
    // read a plain sleep at 1.37ms - just inside the bound that decides the
    // tight check is worth making - and then overshot 4.4ms under pacing,
    // which is the load moving between the two measurements rather than the
    // cap failing. A cap that returns early, or that sleeps the whole budget
    // and takes the overshoot on top, misses on every attempt.
    constexpr int kAttempts = 3;
    std::int64_t bestOvershoot = kFrameNs;
    std::int64_t plain = 0;
    bool held = false;
    for (int attempt = 0; attempt < kAttempts && !held; ++attempt) {
        // Several frames first: the spin margin is learned, so the first one
        // or two may still be adjusting to what this machine's sleep does.
        FramePacer pacer;
        (void)pacer.tickNs();
        for (int i = 0; i < 3; ++i) { pacer.waitForCap(144); (void)pacer.tickNs(); }

        bestOvershoot = kFrameNs;
        for (int i = 0; i < 5; ++i) {
            const std::int64_t start = pacer.lastTickNs();
            pacer.waitForCap(144);
            (void)pacer.tickNs();
            const std::int64_t took = pacer.lastTickNs() - start;

            // Never short, every time: returning early is the cap failing to
            // cap, and it is also what dropping the spin at the end of the
            // wait would do - so this is the check that holds the spin in
            // place.
            CHECK(took >= kFrameNs);
            bestOvershoot = std::min(bestOvershoot, took - kFrameNs);
        }

        // And not long. Sleeping the whole budget and taking the overshoot on
        // top is the mistake the learned margin and the spin exist to avoid,
        // so the bar is a fraction of what that mistake costs on this machine.
        plain = plainSleepOvershootNs(kFrameNs);
        held = plain < kUntestable
                   ? bestOvershoot < std::max<std::int64_t>(plain / 4, 200'000)
                   : bestOvershoot < 2 * plain;
    }

    INFO("best overshoot " << bestOvershoot << "ns, plain sleep " << plain << "ns");
    if (plain < kUntestable) {
        CHECK(bestOvershoot < std::max<std::int64_t>(plain / 4, 200'000));
    } else {
        // A shared runner whose scheduler quantum is wider than the margin
        // being measured - three milliseconds on every attempt, on the macOS
        // CI machine. Nothing about the spin is provable against a clock that
        // coarse, and a fixed bound here only fails the run. What is left to
        // ask is that the cap is not worse than the sleep it is built on.
        WARN("sleeps land " << plain << "ns late here; the tight bound is not testable");
        CHECK(bestOvershoot < 2 * plain);
    }
}
