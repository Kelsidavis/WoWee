#include "pipeline/grass_population.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "core/coordinates.hpp"

namespace wowee {
namespace pipeline {

namespace {

/// Integer hash, from the lattice cell and a salt. No floating point anywhere
/// in the identity of a blade: the same cell has to hash the same on every
/// machine and in every build, or a rebuild would move blades.
uint32_t hashCell(int32_t cx, int32_t cy, uint32_t salt) {
    uint32_t h = static_cast<uint32_t>(cx) * 0x9e3779b9u;
    h ^= static_cast<uint32_t>(cy) * 0x85ebca6bu;
    h ^= salt;
    h ^= h >> 16;
    h *= 0x7feb352du;
    h ^= h >> 15;
    h *= 0x846ca68bu;
    h ^= h >> 16;
    return h;
}

float hashUnit(int32_t cx, int32_t cy, uint32_t salt) {
    return static_cast<float>(hashCell(cx, cy, salt) & 0xffffffu) /
           static_cast<float>(0xffffff);
}

float smoothstep01(float t) {
    return t * t * (3.0f - 2.0f * t);
}

/// The one base cell that survives to octave `level` inside the given
/// level-`level` block, found by descending one octave at a time. Each step
/// picks one of the four child blocks by hash, so the survivors of octave k+1
/// are always a subset of octave k's: a blade that exists far away is the
/// same blade, in the same place, that exists nearby. That subset property is
/// what lets a blade fade by its own level's distance instead of being
/// re-rolled - and popped - on every rebuild.
void descendSurvivor(int32_t bx, int32_t by, int level, uint32_t seed,
                     int32_t& outX, int32_t& outY) {
    for (int j = level; j >= 1; --j) {
        const uint32_t h = hashCell(bx, by, seed ^ (0x51ABu + static_cast<uint32_t>(j)));
        bx = bx * 2 + static_cast<int32_t>(h & 1u);
        by = by * 2 + static_cast<int32_t>((h >> 1) & 1u);
    }
    outX = bx;
    outY = by;
}

/// The highest octave this base cell survives to, capped at maxLevel.
/// Expected cost is O(1): three cells in four stop at the first check.
int survivorLevel(int32_t cx, int32_t cy, int fromLevel, int maxLevel, uint32_t seed) {
    int level = fromLevel;
    while (level < maxLevel) {
        const int next = level + 1;
        int32_t sx = 0;
        int32_t sy = 0;
        descendSurvivor(cx >> next, cy >> next, next, seed, sx, sy);
        if (sx != cx || sy != cy) break;
        level = next;
    }
    return level;
}

/// Smooth value noise over a coarse world lattice, 0..1. Built on the same
/// integer hash as everything else, so it is world-anchored and identical on
/// every rebuild - a drift of deep meadow stays where it is.
float smoothNoise(float x, float y, float scale, uint32_t salt) {
    const float fx = x / scale;
    const float fy = y / scale;
    const auto cx = static_cast<int32_t>(std::floor(fx));
    const auto cy = static_cast<int32_t>(std::floor(fy));
    const float tx = smoothstep01(fx - static_cast<float>(cx));
    const float ty = smoothstep01(fy - static_cast<float>(cy));

    const float a = hashUnit(cx, cy, salt);
    const float b = hashUnit(cx + 1, cy, salt);
    const float c = hashUnit(cx, cy + 1, salt);
    const float d = hashUnit(cx + 1, cy + 1, salt);
    const float top = a + (b - a) * tx;
    const float bottom = c + (d - c) * tx;
    return top + (bottom - top) * ty;
}

} // namespace

void GrassPopulationBuilder::begin(float centerX, float centerY, float radius,
                                   const GrassPopulationParams& params, size_t maxBlades) {
    params_ = params;
    out_.clear();
    maxBlades_ = maxBlades;
    centerX_ = centerX;
    centerY_ = centerY;
    radius_ = radius;
    active_ = false;
    complete_ = true;
    if (params.spacing <= 0.0f || radius <= 0.0f) return;

    // How many octaves the window spans. Ring k covers distances
    // [r0*2^(k-1), r0*2^k) on a lattice 2^k cells coarse, so each ring costs
    // roughly the same number of blocks whatever its distance - a window's
    // build cost grows with the log of its radius, not its area. With no r0
    // the whole window is one full-density ring.
    const float r0 = params_.fullDensityRadius;
    maxRing_ = 0;
    if (r0 > 0.0f) {
        float edge = r0;
        while (edge < radius && maxRing_ < 24) {
            edge *= 2.0f;
            ++maxRing_;
        }
    }

    // If the window holds more candidates than the cap allows, thin every one
    // of them by the same factor rather than filling until full and stopping.
    // The loop walks south to north, so stopping deletes the northern rows
    // outright - the field simply ends on a line, and which line depends on
    // nothing the player can see. Thinning uniformly degrades the whole window
    // instead, and being hash-driven it stays stable frame to frame.
    //
    // The candidate estimate sums each ring's annulus over its own lattice
    // pitch, which is the octave scheme's whole point: a ring at any
    // distance holds about as many candidates as the ring before it.
    double effectiveCells = 0.0;
    const auto s = static_cast<double>(params_.spacing);
    for (int k = 0; k <= maxRing_; ++k) {
        const double inner =
            (k == 0 || r0 <= 0.0f) ? 0.0
                                   : static_cast<double>(r0) * static_cast<double>(1 << (k - 1));
        double outer = (r0 > 0.0f) ? static_cast<double>(r0) * static_cast<double>(1 << k)
                                   : static_cast<double>(radius);
        outer = std::min(outer, static_cast<double>(radius));
        const double sk = s * static_cast<double>(1 << k);
        if (outer > inner) {
            effectiveCells += 3.14159265 * (outer * outer - inner * inner) / (sk * sk);
        }
    }
    // 0.9, not 0.95: the estimate above is the mean of a hash-driven draw,
    // and a small cap's fluctuation is a few percent of itself. The margin
    // has to absorb it, or the cap gets hit anyway and cuts the north side
    // off - the exact failure thinning exists to prevent.
    capFactor_ =
        (effectiveCells > static_cast<double>(maxBlades))
            ? 0.9f * static_cast<float>(static_cast<double>(maxBlades) / effectiveCells)
            : 1.0f;

    ring_ = 0;
    enterRing();
    active_ = true;
    complete_ = false;
}

void GrassPopulationBuilder::enterRing() {
    const float r0 = params_.fullDensityRadius;

    ringInner_ = (ring_ == 0 || r0 <= 0.0f)
                     ? 0.0f
                     : r0 * static_cast<float>(1 << (ring_ - 1));
    ringEdge_ = (r0 > 0.0f && ring_ < maxRing_)
                    ? r0 * static_cast<float>(1 << ring_)
                    : radius_;
    // Slack reaches past the ring's own edge so a blade the player is riding
    // toward is already in the buffer before its fade-in distance arrives,
    // however stale the window centre has gone. The fade starts at 0.65 of a
    // blade's range, so the band itself absorbs 0.35 of the ring edge and
    // the slack only carries what is left of the step - which for every ring
    // past the first is nothing. The top ring's edge is the window itself,
    // whose margin the caller already built in.
    const float slack =
        std::max(0.0f, std::max(params_.ringSlack, 0.0f) - 0.35f * ringEdge_);
    ringOuter_ = (ring_ < maxRing_) ? std::min(ringEdge_ + slack, radius_) : radius_;

    // Anchor the lattice to the world rather than to the centre. Walking east
    // must slide a window over a population that was always there, not shift
    // every blade along with the player. The bounds reach two base cells past
    // the annulus because the jitter deliberately does: a cell just outside
    // a ring's edge can still throw its blade inside, and a bound cut at the
    // edge silently dropped exactly those blades along every ring boundary.
    const float blockSize = params_.spacing * static_cast<float>(1 << ring_);
    const float reach = ringOuter_ + 2.0f * params_.spacing;
    minBlockX_ = static_cast<int32_t>(std::floor((centerX_ - reach) / blockSize));
    maxBlockX_ = static_cast<int32_t>(std::floor((centerX_ + reach) / blockSize));
    cursorY_ = static_cast<int32_t>(std::floor((centerY_ - reach) / blockSize));
    maxBlockY_ = static_cast<int32_t>(std::floor((centerY_ + reach) / blockSize));
    cursorX_ = minBlockX_;
}

bool GrassPopulationBuilder::step(const SuitabilitySampler& sample,
                                  const ProfileLookupFn& profileFor, size_t cellBudget) {
    if (!active_) return true;
    if (!sample) {
        active_ = false;
        complete_ = true;
        return true;
    }

    const GrassPopulationParams& params = params_;
    const float spacing = params.spacing;
    const float densityScale = std::max(params.densityScale, 0.0f);
    const float r0 = params.fullDensityRadius;

    size_t walked = 0;
    while (ring_ <= maxRing_) {
        const float innerSq = ringInner_ * ringInner_;
        const float outerSq = ringOuter_ * ringOuter_;
        const float edgeSq = ringEdge_ * ringEdge_;
        const float blockSize = spacing * static_cast<float>(1 << ring_);
        // A block whose centre is further outside the annulus than half its
        // own diagonal plus the jitter's reach cannot contain its survivor;
        // rejected before any hashing, which is most of an annulus's bbox.
        const float margin = 0.7071f * blockSize + 2.0f * spacing;

        for (int32_t by = cursorY_; by <= maxBlockY_; ++by) {
            for (int32_t bx = cursorX_; bx <= maxBlockX_; ++bx) {
                if (walked >= cellBudget) {
                    cursorY_ = by;
                    cursorX_ = bx;
                    return false;
                }
                ++walked;

                const float bcx = (static_cast<float>(bx) + 0.5f) * blockSize;
                const float bcy = (static_cast<float>(by) + 0.5f) * blockSize;
                const float bdx = bcx - centerX_;
                const float bdy = bcy - centerY_;
                const float bd = std::sqrt(bdx * bdx + bdy * bdy);
                if (bd + margin < ringInner_ || bd - margin > ringOuter_) continue;

                // The one base cell that survives to this octave. Ring 0's
                // blocks are the cells themselves.
                int32_t cx = bx;
                int32_t cy = by;
                if (ring_ > 0) descendSurvivor(bx, by, ring_, params.seed, cx, cy);

                // Off the lattice, hard enough that no lattice shows through.
                //
                // A jitter of half a cell keeps every blade inside its own
                // cell, which is stratified sampling: it enforces a minimum
                // spacing between neighbours and the field comes out in rows,
                // like a planted crop. Jittering past the cell lets blades
                // cluster and leave gaps, which is what a meadow does.
                // Alternate rows are also offset half a cell, so the columns
                // that ran north to south have no straight line left to run
                // along. All hashed from the base cell, so a blade is the
                // same blade at every octave it survives to.
                const float rowShift = (cy & 1) ? 0.5f : 0.0f;
                const float jx = (hashUnit(cx, cy, params.seed ^ 0x01u) - 0.5f) * 1.9f;
                const float jy = (hashUnit(cx, cy, params.seed ^ 0x02u) - 0.5f) * 1.9f;
                const float wx = (static_cast<float>(cx) + 0.5f + rowShift + jx) * spacing;
                const float wy = (static_cast<float>(cy) + 0.5f + jy) * spacing;

                // The annulus proper. Each blade is generated exactly once, by
                // the ring its distance falls in.
                const float ddx = wx - centerX_;
                const float ddy = wy - centerY_;
                const float distSq = ddx * ddx + ddy * ddy;
                if (distSq < innerSq || distSq > outerSq) continue;

                // The slack strip past this ring's edge belongs to the next
                // ring wherever the cell survives that far - the next ring's
                // own walk will generate it.
                if (r0 > 0.0f && ring_ < maxRing_ && distSq >= edgeSq) {
                    int32_t sx = 0;
                    int32_t sy = 0;
                    descendSurvivor(cx >> (ring_ + 1), cy >> (ring_ + 1), ring_ + 1,
                                    params.seed, sx, sy);
                    if (sx == cx && sy == cy) continue;
                }

                const float keep = hashUnit(cx, cy, params.seed ^ 0x03u);
                if (keep >= densityScale * capFactor_) continue;

                const GrassSuitability fit = sample(wx, wy);
                if (fit.suitability <= 0.0f) continue;

                // What grows here, from what the map plants here - and where
                // "here" is, for the regional overrides. Scree thins further
                // than meadow does on ground both call suitable.
                const GrassProfileRef profile =
                    profileFor ? profileFor(fit.effectId, fit.areaId) : GrassProfileRef{};

                // Thin by suitability rather than cutting at a threshold: this
                // is what makes a blend boundary fade out instead of ending on
                // a line. Wildness thins the verges the same way: a stand
                // beside a road or a wall keeps about a seventh of its blades,
                // easing back to all of them in the open. The floor is
                // deliberate - trodden-looking sparse growth beside a path
                // reads as worn, bare ground as paved.
                const float verges = 0.15f + 0.85f * fit.wildness;
                if (keep >= fit.suitability * densityScale * profile.densityScale * capFactor_ *
                                verges) {
                    continue;
                }

                if (out_.size() >= maxBlades_) {
                    active_ = false;
                    complete_ = false;
                    return true;
                }

                GrassBladeSample blade;
                blade.x = wx;
                blade.y = wy;
                blade.z = fit.rootHeight;
                // How far out this blade may draw: its own octave level's
                // distance, permanent because the level is a property of the
                // cell. The shader grows and sinks it by live distance, so it
                // eases into view on approach; the top level is uncapped and
                // fades at the global range instead.
                if (r0 > 0.0f) {
                    const int level = survivorLevel(cx, cy, ring_, maxRing_, params.seed);
                    blade.fadeDistance =
                        (level < maxRing_) ? r0 * static_cast<float>(1 << level) : 0.0f;
                }
                // Verges are short and the open field is deep. Suitability is
                // already the distance-to-road signal - it blends down toward
                // every road and dirt texture - so grass shortens approaching
                // one instead of standing knee-high to the wheel ruts. On top
                // of that, drifts of deep meadow twenty-odd yards across, from
                // world-anchored noise, so open ground is not one uniform pile.
                const float verge = 0.55f + 0.45f * std::min(fit.suitability * 1.6f, 1.0f);
                const float depthNoise = smoothNoise(wx, wy, 23.0f, params.seed ^ 0x07u);
                const float meadowDepth =
                    0.8f +
                    0.85f * smoothstep01(std::clamp((depthNoise - 0.4f) / 0.35f, 0.0f, 1.0f));

                blade.height = params.baseHeight * profile.heightScale * verge * meadowDepth *
                               // Short beside the built and the paved, full
                               // height in the wild - the same signal that
                               // thinned the stand above, so verges look worn
                               // rather than merely sparse.
                               (0.45f + 0.55f * fit.wildness) *
                               (1.0f - params.heightVariation +
                                2.0f * params.heightVariation *
                                    hashUnit(cx, cy, params.seed ^ 0x04u));
                blade.facing = hashUnit(cx, cy, params.seed ^ 0x05u) * core::coords::TWO_PI;
                // Width varies per blade too; a sward of one width reads as
                // extruded, whatever the heights do.
                blade.width = params.baseWidth * profile.widthScale *
                              (0.7f + 0.8f * hashUnit(cx, cy, params.seed ^ 0x08u));
                blade.phase = hashUnit(cx, cy, params.seed ^ 0x06u);
                blade.profileIndex = profile.index;
                blade.groundShadow = fit.groundShadow;
                blade.groundHighlight = fit.groundHighlight;
                blade.hasGroundColor = fit.hasGroundColor;
                blade.submerged = fit.submerged;
                out_.push_back(blade);
            }
            // Resuming mid-row starts the inner loop at the cursor; every
            // later row starts at the ring's edge.
            cursorX_ = minBlockX_;
        }
        // The next octave, if the window reaches one.
        ++ring_;
        if (ring_ <= maxRing_) enterRing();
    }

    active_ = false;
    complete_ = true;
    return true;
}

bool populateArea(float centerX, float centerY, float radius,
                  const GrassPopulationParams& params,
                  const SuitabilitySampler& sample,
                  std::vector<GrassBladeSample>& out,
                  size_t maxBlades,
                  const ProfileLookupFn& profileFor) {
    GrassPopulationBuilder builder;
    builder.begin(centerX, centerY, radius, params, maxBlades);
    while (!builder.step(sample, profileFor, std::numeric_limits<size_t>::max())) {
    }
    out = std::move(builder.blades());
    return builder.complete();
}

} // namespace pipeline
} // namespace wowee
