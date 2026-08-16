#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Movie Credits Roll catalog (.wmvc) -
// novel replacement for the embedded credit-roll text
// vanilla WoW carried inside the cinematic-renderer
// blob (the post-cinematic credits that scroll up the
// screen after each expansion intro). Each entry binds
// one credits category (Production / Music / Voice
// Acting / etc.) for one cinematic to its ordered list
// of credit lines.
//
// First catalog with a variable-length STRING array
// payload - previous variable-length formats used int
// arrays (WCMR waypoints, WCMG members, WPTT spell
// rank-arrays, WBAB rank chains, WRPR unlocked items
// + recipes). The lines[] field is serialized as
// count + (length + bytes)* per line.
//
// Cross-references with previously-added formats:
//   WCMS: cinematicId references the WCMS cinematic
//         catalog (the cinematic these credits play
//         after).
//
// Binary layout (little-endian):
//   magic[4]            = "WMVC"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     rollId (uint32)
//     nameLen + name
//     descLen + description
//     cinematicId (uint32)
//     category (uint8)            - Production / Music /
//                                    Audio / Engineering
//                                    / Art / Voice /
//                                    Special
//     pad0 / pad1 / pad2 (uint8)
//     orderHint (uint16)          - sort key for the
//                                    cinematic's credit
//                                    order (lower =
//                                    earlier in roll)
//     pad4 / pad5 (uint8)
//     iconColorRGBA (uint32)
//     lineCount (uint32)
//     lines (count × { uint32 strLen + bytes })
struct WoweeMovieCredits {
    enum Category : uint8_t {
        Production  = 0,    // Director, Producer, etc.
        Music       = 1,    // Composer, Orchestra
        Audio       = 2,    // Sound Design, Foley
        Engineering = 3,    // Tools, Pipeline
        Art         = 4,    // Concept, Modeling, Anim
        Voice       = 5,    // Voice cast
        Special     = 6,    // Special Thanks, Dedication
    };

    struct Entry {
        uint32_t rollId = 0;
        std::string name;
        std::string description;
        uint32_t cinematicId = 0;
        uint8_t category = Production;
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint8_t pad2 = 0;
        uint16_t orderHint = 0;
        uint8_t pad4 = 0;
        uint8_t pad5 = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
        std::vector<std::string> lines;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t rollId) const;

    // Returns all credit-roll entries for one cinematic,
    // sorted by orderHint. Used by the credit renderer
    // to assemble the full scroll for one cinematic.
    [[nodiscard]] std::vector<const Entry*> findByCinematic(uint32_t cinematicId) const;
};

class WoweeMovieCreditsLoader {
public:
    static bool save(const WoweeMovieCredits& cat,
                     const std::string& basePath);
    static WoweeMovieCredits load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-mvc* variants.
    //
    //   makeWotLKIntro    - 5 categories for the WotLK
    //                        cinematic (Production /
    //                        Direction / Music / Voice
    //                        Acting / Special Thanks)
    //                        with 3-4 lines each.
    //   makeQuestCinema   - 3 categories for a typical
    //                        per-quest cinematic (Quest
    //                        Designer / Voice Cast /
    //                        Cinematic Director).
    //   makeStarterRoll   - 4 categories demonstrating
    //                        roll structure (Production
    //                        / Engineering / Art /
    //                        Special Thanks).
    static WoweeMovieCredits makeWotLKIntro(const std::string& catalogName);
    static WoweeMovieCredits makeQuestCinema(const std::string& catalogName);
    static WoweeMovieCredits makeStarterRoll(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
