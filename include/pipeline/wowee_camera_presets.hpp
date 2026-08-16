#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Camera Presets catalog (.wcam) -
// novel format covering what vanilla WoW handled
// with hard-coded camera profiles in the client's
// CameraMgr (the standard third-person camera, the
// flight-path camera, vehicle cameras and
// cinematic cameras were all bespoke C++ classes
// with no data-driven extension point). Each WCAM
// entry binds one camera preset to its FOV,
// distance, pitch/yaw offsets, shoulder offset,
// focus-bone tracking target, motion damping
// curve, and intended purpose (Cinematic / Combat
// / Mounted / Vehicle / Cutscene / PhotoMode).
//
// Cross-references with previously-added formats:
//   None directly - camera presets are
//   self-contained client-side configurations
//   selected by the camera controller based on
//   gameplay context.
//
// Binary layout (little-endian):
//   magic[4]            = "WCAM"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     presetId (uint32)
//     nameLen + name
//     purposeKind (uint8)          - 0=Cinematic /
//                                     1=Combat /
//                                     2=Mounted /
//                                     3=Vehicle /
//                                     4=Cutscene /
//                                     5=PhotoMode
//     motionDamping (uint8)        - 0..255 (0 =
//                                     instant
//                                     follow / 255 =
//                                     maximum lag)
//     pad0 (uint16)
//     fovDegrees (float)           - typical 60..90
//     distanceFromTarget (float)   - meters from
//                                     focus point
//     pitchDegrees (float)         - vertical angle
//     yawOffsetDegrees (float)     - horizontal
//                                     offset from
//                                     character
//                                     facing
//     shoulderOffsetMeters (float) - over-shoulder
//                                     offset
//                                     (negative =
//                                     left, positive
//                                     = right)
//     focusBoneId (uint32)         - M2 bone index
//                                     (typical:
//                                     0=root, 12=
//                                     chest, 50=
//                                     head); 0xFFFF
//                                     = follow root
struct WoweeCameraPresets {
    enum PurposeKind : uint8_t {
        Cinematic = 0,
        Combat    = 1,
        Mounted   = 2,
        Vehicle   = 3,
        Cutscene  = 4,
        PhotoMode = 5,
    };

    static constexpr uint32_t kFollowRootBoneId = 0xFFFFu;

    struct Entry {
        uint32_t presetId = 0;
        std::string name;
        uint8_t purposeKind = Combat;
        uint8_t motionDamping = 0;
        uint16_t pad0 = 0;
        float fovDegrees = 0.f;
        float distanceFromTarget = 0.f;
        float pitchDegrees = 0.f;
        float yawOffsetDegrees = 0.f;
        float shoulderOffsetMeters = 0.f;
        uint32_t focusBoneId = 0;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t presetId) const;

    // Returns all presets of one purpose - used by
    // the camera controller when context changes
    // (e.g. entering a vehicle picks the first
    // Vehicle-purpose preset).
    [[nodiscard]] std::vector<const Entry*> findByPurpose(uint8_t purposeKind) const;
};

class WoweeCameraPresetsLoader {
public:
    static bool save(const WoweeCameraPresets& cat,
                     const std::string& basePath);
    static WoweeCameraPresets load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-cam* variants.
    //
    //   makeCombatPresets - 3 combat camera variants
    //                        (default 75 FOV / wide
    //                        90 FOV ranged-DPS / tight
    //                        60 FOV melee shoulder-
    //                        cam). Low motion damping
    //                        (10) for responsive
    //                        tracking.
    //   makeMountedPresets - 2 mounted-camera variants
    //                        (ground-mount slightly-
    //                        pulled-back vs flying-
    //                        mount higher pitch).
    //                        Medium damping (60) for
    //                        smooth turning.
    //   makeCinematicPresets - 3 cinematic angles
    //                          (over-shoulder dialogue
    //                          / wide establishing
    //                          / close-up portrait).
    //                          High damping (180)
    //                          for film-quality
    //                          motion.
    static WoweeCameraPresets makeCombatPresets(const std::string& catalogName);
    static WoweeCameraPresets makeMountedPresets(const std::string& catalogName);
    static WoweeCameraPresets makeCinematicPresets(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
