// The ablation's phase walk: one pass off at a time, and what it reports.
//
// This is the only instrument that can attribute a frame on this platform -
// MoltenVK resolves every GPU timestamp inside a render pass to the same
// clock, so the scene pass reports as one number and the marks after the first
// read two microseconds each. An instrument that answers questions no other
// instrument can answer is one whose own arithmetic has to be checked, or a
// wrong number goes unnoticed because there is nothing to check it against.

#include <catch_amalgamated.hpp>

#include <cmath>
#include <cstddef>
#include <string>

#include "rendering/pass_ablation.hpp"

using wowee::rendering::AblationPass;
using wowee::rendering::PassAblation;

namespace {

/// Feed exactly one phase: frames of `ms` until the phase clock has run out,
/// and not one frame more. One frame past the boundary lands in the next
/// phase, where it is averaged into a pass it was not measuring.
void runPhase(PassAblation& ablation, double ms, double phaseMs = 100.0,
              double warmupMs = 0.0) {
    const int frames = static_cast<int>(std::ceil((phaseMs + warmupMs) / ms));
    for (int i = 0; i < frames; ++i) ablation.frame(ms);
}

}  // namespace

TEST_CASE("a run walks every pass and ends on the baseline") {
    PassAblation ablation(100.0, 10.0, 0.0);

    REQUIRE(ablation.current() == AblationPass::None);
    REQUIRE(ablation.running());

    // Nothing is skipped during the baseline.
    CHECK_FALSE(ablation.skip(AblationPass::Terrain));
    CHECK_FALSE(ablation.skip(AblationPass::Shadows));
    // "None" is a phase, not a pass, and is never skipped.
    CHECK_FALSE(ablation.skip(AblationPass::None));

    const AblationPass expected[] = {
        AblationPass::Terrain,    AblationPass::Grass, AblationPass::WMO,
        AblationPass::M2,         AblationPass::Clutter,
        AblationPass::FarDoodads,
        AblationPass::Characters, AblationPass::Sky,   AblationPass::Shadows,
        AblationPass::VolumetricFog, AblationPass::SunShafts,
    };
    // Every pass in the walk, and the two baselines that bracket it.
    REQUIRE(sizeof(expected) / sizeof(expected[0]) + 2 == PassAblation::phaseCount());
    for (AblationPass pass : expected) {
        runPhase(ablation, 10.0, 100.0, 10.0);
        REQUIRE(ablation.running());
        REQUIRE(ablation.current() == pass);
        // Exactly one pass is off at a time.
        int off = 0;
        for (AblationPass p : expected) {
            if (ablation.skip(p)) ++off;
        }
        CHECK(off == 1);
        CHECK(ablation.skip(pass));
    }

    runPhase(ablation, 10.0, 100.0, 10.0);
    CHECK(ablation.running());
    CHECK(ablation.current() == AblationPass::None);

    runPhase(ablation, 10.0, 100.0, 10.0);
    CHECK_FALSE(ablation.running());
    // Nothing is switched off once the run is over: a finished ablation must
    // not leave the world missing a pass.
    for (AblationPass p : expected) CHECK_FALSE(ablation.skip(p));
}

TEST_CASE("no report until the run finishes") {
    PassAblation ablation(100.0, 10.0, 0.0);
    runPhase(ablation, 10.0, 100.0, 10.0);
    CHECK(ablation.report().empty());
}

TEST_CASE("the warm-up frames are thrown away") {
    // Every phase is ten expensive frames - the resident set still turning
    // over, the pipelines still warming - and then eighty settled ones. With
    // the warm-up set to exactly the expensive stretch, the report is the
    // settled number; without it, the ten drag the average up.
    PassAblation warmed(800.0, 200.0, 0.0);
    PassAblation raw(1000.0, 0.0, 0.0);
    for (PassAblation* a : {&warmed, &raw}) {
        for (std::size_t phase = 0; phase < PassAblation::phaseCount(); ++phase) {
            for (int i = 0; i < 10; ++i) a->frame(20.0);   // 200ms of warm-up
            for (int i = 0; i < 80; ++i) a->frame(10.0);   // 800ms settled
        }
    }
    REQUIRE_FALSE(warmed.running());
    REQUIRE_FALSE(raw.running());

    // Both baselines within a run are identical, so the drift is zero and the
    // baseline is whatever that run chose to average.
    CHECK(warmed.report().find("baseline 10.00ms/frame") != std::string::npos);
    CHECK(raw.report().find("baseline 11.11ms/frame") != std::string::npos);
}

TEST_CASE("a pass is worth the frame time it takes away, and drift is reported") {
    PassAblation ablation(100.0, 0.0, 0.0);

    // Baseline at 20ms, terrain phase at 8ms, everything else back at 20ms,
    // and the closing baseline at 21ms - a millisecond of drift over the run.
    const double phaseMs[] = {20.0, 8.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 21.0};
    REQUIRE(sizeof(phaseMs) / sizeof(phaseMs[0]) == PassAblation::phaseCount());
    for (double ms : phaseMs) runPhase(ablation, ms);

    REQUIRE_FALSE(ablation.running());
    const std::string report = ablation.report();

    // Baseline is the mean of the two, so 20.5, and terrain is worth 12.5.
    CHECK(report.find("baseline 20.50ms/frame") != std::string::npos);
    CHECK(report.find("-terrain: 8.00ms/frame, worth 12.50ms") != std::string::npos);
    // A pass that changed nothing is worth half the drift, not nothing - and
    // the drift line is what says that number is noise.
    CHECK(report.find("-sky: 20.00ms/frame, worth 0.50ms") != std::string::npos);
    CHECK(report.find("drift across the run: 1.00ms") != std::string::npos);
}

TEST_CASE("nothing is measured until the world has settled") {
    // Ten seconds of settling, then phases. Entering a zone costs frames of a
    // hundred milliseconds while it streams; none of them may reach a phase.
    PassAblation ablation(100.0, 0.0, 10000.0);

    for (int i = 0; i < 99; ++i) ablation.frame(100.0);   // 9.9s of streaming
    CHECK(ablation.settling());
    CHECK(ablation.current() == AblationPass::None);
    CHECK_FALSE(ablation.skip(AblationPass::Terrain));

    ablation.frame(100.0);                                 // crosses 10s
    CHECK_FALSE(ablation.settling());
    // Still on the baseline: the settling frames were discarded, not banked.
    CHECK(ablation.current() == AblationPass::None);

    runPhase(ablation, 20.0);
    CHECK(ablation.current() == AblationPass::Terrain);
}

TEST_CASE("a run the world moved under is called what it is") {
    PassAblation ablation(100.0, 0.0, 0.0);

    // The shape of the first real run: a cheap baseline taken before the world
    // was up, a ruinous terrain phase taken while the zone streamed in, and a
    // closing baseline nowhere near the opening one.
    const double phaseMs[] = {5.0, 123.0, 21.0, 18.0, 17.0, 17.0, 17.0, 20.0, 20.0, 17.0, 17.0, 17.0, 20.0};
    REQUIRE(sizeof(phaseMs) / sizeof(phaseMs[0]) == PassAblation::phaseCount());
    for (double ms : phaseMs) runPhase(ablation, ms);

    const std::string report = ablation.report();
    REQUIRE_FALSE(report.empty());
    CHECK(report.find("this table is not a measurement") != std::string::npos);
}

TEST_CASE("a steady run is not called into question") {
    PassAblation ablation(100.0, 0.0, 0.0);
    const double phaseMs[] = {20.0, 8.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 21.0};
    REQUIRE(sizeof(phaseMs) / sizeof(phaseMs[0]) == PassAblation::phaseCount());
    for (double ms : phaseMs) runPhase(ablation, ms);
    CHECK(ablation.report().find("not a measurement") == std::string::npos);
}
