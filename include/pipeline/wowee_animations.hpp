#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Animation Data catalog (.wani) - novel
// replacement for Blizzard's AnimationData.dbc plus the
// hard-coded animation-id tables in M2 models. The 46th
// open format added to the editor.
//
// Defines named animations (Stand, Walk, Run, Cast, Death,
// SitGround, Mount, ...) with fallback chains, behavior
// tier (default / mounted / aerial / sitting), and weapon-
// flag bitmasks that select the right animation variant
// when the model is wielding 1H / 2H / dual / unarmed /
// ranged.
//
// Cross-references with previously-added formats:
//   WANI.entry.fallbackId → WANI.entry.animationId
//                            (graceful degradation when the
//                             requested animation is absent)
//
// Binary layout (little-endian):
//   magic[4]            = "WANI"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     animationId (uint32)
//     nameLen + name
//     descLen + description
//     fallbackId (uint32)
//     behaviorId (uint32)
//     behaviorTier (uint8) / pad[3]
//     flags (uint32)
//     weaponFlags (uint32)
//     loopDurationMs (uint32)        - 0 = oneshot
struct WoweeAnimation {
    enum BehaviorTier : uint8_t {
        Default  = 0,    // standing on the ground
        Mounted  = 1,    // riding a vehicle / mount
        Sitting  = 2,    // sitting in a chair / on the ground
        Aerial   = 3,    // flying / hovering
        Swimming = 4,    // underwater
    };

    // Flag bits - animation behavior modifiers.
    static constexpr uint32_t kFlagLooped         = 0x00000001;
    static constexpr uint32_t kFlagBlendableCycle = 0x00000002;
    static constexpr uint32_t kFlagInterruptable  = 0x00000004;
    static constexpr uint32_t kFlagMovementSync   = 0x00000008;
    static constexpr uint32_t kFlagOneShot        = 0x00000010;
    static constexpr uint32_t kFlagPreserveAtEnd  = 0x00000020;

    // Weapon-flag bits - which wielded weapon this anim
    // applies to. Match the WoW weapon class enum.
    static constexpr uint32_t kWeaponUnarmed   = 0x00000001;
    static constexpr uint32_t kWeapon1HMelee   = 0x00000002;
    static constexpr uint32_t kWeapon2HMelee   = 0x00000004;
    static constexpr uint32_t kWeaponDualWield = 0x00000008;
    static constexpr uint32_t kWeaponBow       = 0x00000010;
    static constexpr uint32_t kWeaponCrossbow  = 0x00000020;
    static constexpr uint32_t kWeaponRifle     = 0x00000040;
    static constexpr uint32_t kWeaponWand      = 0x00000080;
    static constexpr uint32_t kWeaponPolearm   = 0x00000100;
    static constexpr uint32_t kWeaponShield    = 0x00000200;
    static constexpr uint32_t kWeaponAny       = 0xFFFFFFFFu;

    struct Entry {
        uint32_t animationId = 0;
        std::string name;
        std::string description;
        uint32_t fallbackId = 0;       // WANI cross-ref
        uint32_t behaviorId = 0;
        uint8_t behaviorTier = Default;
        uint32_t flags = 0;
        uint32_t weaponFlags = kWeaponAny;
        uint32_t loopDurationMs = 0;   // 0 = oneshot
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t animationId) const;

    static const char* behaviorTierName(uint8_t t);
};

class WoweeAnimationLoader {
public:
    static bool save(const WoweeAnimation& cat,
                     const std::string& basePath);
    static WoweeAnimation load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-animations* variants.
    //
    //   makeStarter  - 5 essential animations every model
    //                   needs (Stand / Walk / Run / Death /
    //                   AttackUnarmed).
    //   makeCombat   - 8 combat animations covering 1H/2H/
    //                   dual-wield melee + bow/rifle/crossbow
    //                   ranged + channeled spell + parry.
    //   makeMovement - 6 movement animations (Walk / Run /
    //                   Sprint / Swim / Mount / Fly) with
    //                   behavior-tier transitions.
    static WoweeAnimation makeStarter(const std::string& catalogName);
    static WoweeAnimation makeCombat(const std::string& catalogName);
    static WoweeAnimation makeMovement(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
