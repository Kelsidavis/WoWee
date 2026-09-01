#pragma once

// Whether a wall stands between the camera and something in the world.
//
// The nameplates asked this first: a name that reads through a wall puts a
// player in a room they are not in. Chat bubbles have exactly the same fault
// and the same answer, so the query lives here rather than in one of them,
// where the second copy would have drifted from the first.
//
// It is a collision query against the WMO geometry, so it is remembered per
// subject rather than run every frame for everything on screen.

#include <cstdint>
#include <unordered_map>

#include <glm/glm.hpp>

#include "rendering/wmo_renderer.hpp"

namespace wowee {
namespace ui {

/// Per-subject line of sight, recomputed only when the answer could have
/// changed.
class SightCache {
public:
    /// True when something solid stands between `from` and `to`.
    ///
    /// `key` is whatever identifies the subject to the caller - a guid, in
    /// both of the places this is used. Each caller keeps its own cache, so
    /// two views of the same unit at different heights do not fight over one
    /// entry.
    bool blocked(rendering::WMORenderer* wmo, uint64_t key,
                 const glm::vec3& from, const glm::vec3& to) {
        if (!wmo) return false;
        // Bounded, because every unit ever seen would otherwise keep an entry
        // for the life of the process. Dropping the lot costs one
        // recomputation each for the handful currently on screen.
        if (entries_.size() > kMaxEntries) entries_.clear();
        Entry& e = entries_[key];
        // Recomputed when either end has moved far enough to change the
        // answer, and every so often regardless, since the geometry between
        // them streams in and out.
        const bool moved = glm::distance(e.from, from) > kMovedUnits ||
                           glm::distance(e.to, to) > kMovedUnits;
        if (moved || e.age >= kMaxAgeFrames) {
            e.from = from;
            e.to = to;
            e.age = 0;
            e.blocked = wmo->segmentBlocked(from, to);
        } else {
            ++e.age;
        }
        return e.blocked;
    }

private:
    static constexpr std::size_t kMaxEntries = 512;
    static constexpr float kMovedUnits = 0.5f;
    static constexpr int kMaxAgeFrames = 15;

    struct Entry {
        glm::vec3 from{};
        glm::vec3 to{};
        int age = 1000;
        bool blocked = false;
    };
    std::unordered_map<uint64_t, Entry> entries_;
};

}  // namespace ui
}  // namespace wowee
