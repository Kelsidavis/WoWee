#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace wowee::rendering {

/// A world pass an ablation run can switch off for a few seconds.
enum class AblationPass {
    None = 0,
    Terrain,
    Grass,
    WMO,
    M2,
    Clutter,
    FarDoodads,
    Characters,
    Sky,
    Shadows,
    VolumetricFog,
    SunShafts,
};

const char* ablationPassName(AblationPass pass);

/// What a pass costs, measured by taking it away.
///
/// The GPU's own timestamps cannot answer that on this Mac. MoltenVK resolves
/// a timestamp against the Metal encoder that contains it, so every mark
/// inside one render pass reads the same clock - the first mark in the scene
/// pass absorbs the whole pass and the six after it report two microseconds
/// apiece. A profile taken that way said terrain cost 14ms and that the
/// characters, the buildings, the water and every doodad in Stormwind came to
/// twenty microseconds between them. The three passes that do read sensibly -
/// shadows, post-process, interface - are the three that have a render pass to
/// themselves, which is the tell.
///
/// So it is measured from the other side: run with a pass switched off and see
/// what the frame does. One run walks the list, holds each phase long enough
/// for the average to settle, and returns to the baseline at the end so drift
/// over the run is visible rather than charged to the last pass measured.
///
/// The phase clock is the frames themselves, not a wall clock: a phase that
/// renders nothing runs faster and would otherwise be sampled for fewer frames
/// than the one it is compared against.
class PassAblation {
public:
    /// `phaseMs` is how long each phase is sampled for, `warmupMs` how much of
    /// the front of it is thrown away - a phase's first frames still carry the
    /// previous phase's resident set and its pipeline warm-up. `settleMs` is
    /// the stretch before any of it starts.
    PassAblation(double phaseMs = 4000.0, double warmupMs = 750.0,
                 double settleMs = 10000.0);

    /// One frame's wall time, from the top of a frame to the top of the next.
    /// Feed only frames that drew the world: the first run of this walked its
    /// baseline through character select at 5ms a frame and its terrain phase
    /// through the zone streaming in at 123ms, and reported that terrain was
    /// worth minus 110 milliseconds.
    void frame(double frameMs);

    /// Still standing at the door. The world has to be drawn, and drawn for a
    /// while, before any of these numbers mean anything - a zone streams in
    /// for several seconds after it is entered and every frame in that stretch
    /// belongs to the streaming, not to whatever pass happened to be off.
    bool settling() const { return settledMs_ < settleMs_; }

    /// Is this pass switched off for the phase now running?
    bool skip(AblationPass pass) const;

    /// Still walking the list.
    bool running() const { return phase_ < phaseCount(); }

    /// Which phase is running, for a caller that wants to say so.
    AblationPass current() const;

    /// The table. Empty until every phase has been held.
    std::string report() const;

    /// Number of phases in a run, the two baselines included.
    static std::size_t phaseCount();

    /// Which phase this is, counting from one, for saying so out loud.
    std::size_t phaseNumber() const { return phase_ + 1; }

    /// Roughly how long a whole run takes, in milliseconds of world frames.
    /// The caller says this at the start so the run is not quit halfway - the
    /// first attempt at ten phases was abandoned eight phases in, with nothing
    /// logged the whole time to say how far it had got.
    double expectedRunMs() const {
        return settleMs_ + static_cast<double>(phaseCount()) * (warmupMs_ + phaseMs_);
    }

private:
    struct Sample {
        int frames = 0;
        double totalMs = 0.0;
    };

    double phaseMs_;
    double warmupMs_;
    double settleMs_;
    double settledMs_ = 0.0;
    std::size_t phase_ = 0;
    double elapsedMs_ = 0.0;
    std::vector<Sample> samples_;
};

}  // namespace wowee::rendering
