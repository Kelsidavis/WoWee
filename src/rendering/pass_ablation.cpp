#include "rendering/pass_ablation.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace wowee::rendering {

namespace {

/// The order a run walks. A baseline at each end: the first is what every
/// pass is measured against, the second says how much the world moved under
/// the run - a zone streaming in, a cloud crossing the sun - which is the
/// error bar on everything between them.
constexpr AblationPass kOrder[] = {
    AblationPass::None,
    AblationPass::Terrain,
    AblationPass::Grass,
    AblationPass::WMO,
    AblationPass::M2,
    AblationPass::Clutter,
    AblationPass::FarDoodads,
    AblationPass::Characters,
    AblationPass::Sky,
    AblationPass::Shadows,
    AblationPass::VolumetricFog,
    AblationPass::SunShafts,
    AblationPass::None,
};

}  // namespace

const char* ablationPassName(AblationPass pass) {
    switch (pass) {
        case AblationPass::None:       return "baseline";
        case AblationPass::Terrain:    return "terrain";
        case AblationPass::Grass:      return "grass";
        case AblationPass::WMO:        return "wmo";
        case AblationPass::M2:         return "doodads (clutter included)";
        case AblationPass::Clutter:    return "ground clutter alone";
        case AblationPass::FarDoodads: return "doodads past 250 yards";
        case AblationPass::Characters: return "characters";
        case AblationPass::Sky:        return "sky";
        case AblationPass::Shadows:    return "shadows";
        case AblationPass::VolumetricFog: return "volumetric fog";
        case AblationPass::SunShafts:  return "sun shafts";
    }
    return "?";
}

std::size_t PassAblation::phaseCount() {
    return sizeof(kOrder) / sizeof(kOrder[0]);
}

PassAblation::PassAblation(double phaseMs, double warmupMs, double settleMs)
    : phaseMs_(phaseMs),
      warmupMs_(warmupMs),
      settleMs_(settleMs),
      samples_(phaseCount()) {}

AblationPass PassAblation::current() const {
    return phase_ < phaseCount() ? kOrder[phase_] : AblationPass::None;
}

bool PassAblation::skip(AblationPass pass) const {
    return running() && pass != AblationPass::None && kOrder[phase_] == pass;
}

void PassAblation::frame(double frameMs) {
    if (!running()) return;
    if (settling()) {
        settledMs_ += frameMs;
        return;
    }
    elapsedMs_ += frameMs;
    if (elapsedMs_ > warmupMs_) {
        samples_[phase_].frames += 1;
        samples_[phase_].totalMs += frameMs;
    }
    if (elapsedMs_ >= warmupMs_ + phaseMs_) {
        ++phase_;
        elapsedMs_ = 0.0;
    }
}

std::string PassAblation::report() const {
    if (running()) return {};

    const auto mean = [&](std::size_t i) {
        return samples_[i].frames > 0
                   ? samples_[i].totalMs / samples_[i].frames
                   : 0.0;
    };

    // Both baselines, averaged, so a run that drifted charges half the drift
    // to each end rather than all of it to the passes measured late.
    const double first = mean(0);
    const double last = mean(phaseCount() - 1);
    const double baseline = (first + last) * 0.5;

    std::ostringstream out;
    out << std::fixed << std::setprecision(2);
    out << "pass ablation: baseline " << baseline << "ms/frame ("
        << first << " at the start, " << last << " at the end)";
    for (std::size_t i = 1; i + 1 < phaseCount(); ++i) {
        const double ms = mean(i);
        out << "\n    -" << ablationPassName(kOrder[i]) << ": " << ms
            << "ms/frame, worth " << (baseline - ms) << "ms"
            << " (" << samples_[i].frames << " frames)";
    }
    // The drift is the honest floor on all of it: a difference smaller than
    // the gap between the two baselines is not a measurement.
    const double drift = last - first;
    out << "\n    drift across the run: " << drift
        << "ms - anything smaller than that is noise";
    // And when the drift swamps everything it is not a floor, it is a verdict.
    // A run taken while walking, or while a zone streamed in, produces a table
    // that looks like numbers and is not one, and the first run of this said
    // terrain was worth minus 110 milliseconds. Say so here rather than leave
    // it to be read off the last line.
    double largest = 0.0;
    for (std::size_t i = 1; i + 1 < phaseCount(); ++i) {
        const double worth = baseline - mean(i);
        if (worth > largest) largest = worth;
    }
    if (std::abs(drift) > largest) {
        out << "\n    the world moved more than the passes did - stand still,"
               " outdoors, and run it again; this table is not a measurement";
    }
    return out.str();
}

}  // namespace wowee::rendering
