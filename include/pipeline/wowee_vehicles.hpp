#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Vehicle catalog (.wvhc) - novel replacement for
// Blizzard's Vehicle.dbc + VehicleSeat.dbc + the
// AzerothCore-style vehicle_template SQL tables. The 43rd
// open format added to the editor.
//
// Defines drivable / rideable vehicles: tanks, demolishers,
// motorcycles, gryphon mounts with passengers, choppers,
// siege weapons, multi-passenger transports. Each vehicle
// pairs a creature template (the rendered model) with a
// fixed seat layout - driver / passenger seats with their
// own attachment points, control flags, and per-seat
// abilities mounted to the action bar.
//
// Cross-references with previously-added formats:
//   WVHC.entry.creatureId      → WCRT.creatureId
//                                  (the rendered vehicle model)
//   WVHC.entry.flightCapabilityId → WMNT.mountId  (optional -
//                                  shared fly-speed table for
//                                  flying vehicles)
//   WVHC.seat.controlSpellId   → WSPL.spellId
//                                  (action-bar slot for the seat)
//   WVHC.seat.exitSpellId      → WSPL.spellId
//                                  (eject ability)
//
// Binary layout (little-endian):
//   magic[4]            = "WVHC"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     vehicleId (uint32)
//     creatureId (uint32)
//     nameLen + name
//     descLen + description
//     vehicleKind (uint8) / movementKind (uint8) /
//       seatCount (uint8) / pad[1]
//     turnSpeed (float)
//     pitchSpeed (float)
//     flightCapabilityId (uint32)   - 0 = ground vehicle
//     powerType (uint8) / pad[3]
//     maxPower (uint32)
//     seats (seatCount × VehicleSeat):
//       seatIndex (uint8) / seatFlags (uint8) /
//         attachmentId (uint8) / pad[1]
//       controlSpellId (uint32)
//       exitSpellId (uint32)
//       passengerYaw (float)
struct WoweeVehicle {
    enum VehicleKind : uint8_t {
        Mount         = 0,    // 1-rider creature mount
        Chopper       = 1,    // motorcycle / 2-seater hog
        Tank          = 2,    // tracked siege tank
        Demolisher    = 3,    // catapult / trebuchet
        Gunship       = 4,    // multi-seat aerial / sea craft
        FlyingMount   = 5,    // single-seat flying mount
        TransportRail = 6,    // tram / zeppelin (rail-bound)
        SiegeWeapon   = 7,    // ballista / cannon emplacement
    };

    enum MovementKind : uint8_t {
        Ground       = 0,
        Air          = 1,
        Water        = 2,
        Submerged    = 3,    // operates fully underwater
        AmphibiousAW = 4,    // air + water
        AmphibiousGW = 5,    // ground + water
    };

    enum PowerType : uint8_t {
        Mana   = 0,
        Energy = 1,    // chopper boost / standard energy bar
        Pyrite = 2,    // Ulduar-specific charge resource
        Heat   = 3,    // tank / cannon overheat resource
        None   = 4,    // simple mounts have no power bar
    };

    // Per-seat flags - seatFlags is a bitmask.
    static constexpr uint8_t kSeatDriver       = 0x01;
    static constexpr uint8_t kSeatGunner       = 0x02;
    static constexpr uint8_t kSeatPassenger    = 0x04;
    static constexpr uint8_t kSeatHidesPlayer  = 0x08;  // model swap
    static constexpr uint8_t kSeatNoEjectByCC  = 0x10;  // cc immune

    struct Seat {
        uint8_t seatIndex = 0;
        uint8_t seatFlags = kSeatPassenger;
        uint8_t attachmentId = 0;        // M2 attachment slot
        uint32_t controlSpellId = 0;     // WSPL cross-ref
        uint32_t exitSpellId = 0;        // WSPL cross-ref
        float passengerYaw = 0.0f;       // facing offset (radians)
    };

    struct Entry {
        uint32_t vehicleId = 0;
        uint32_t creatureId = 0;          // WCRT cross-ref
        std::string name;
        std::string description;
        uint8_t vehicleKind = Mount;
        uint8_t movementKind = Ground;
        float turnSpeed = 3.14f;          // radians/sec
        float pitchSpeed = 1.0f;          // radians/sec
        uint32_t flightCapabilityId = 0;  // WMNT cross-ref or 0
        uint8_t powerType = None;
        uint32_t maxPower = 100;
        std::vector<Seat> seats;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t vehicleId) const;

    static const char* vehicleKindName(uint8_t k);
    static const char* movementKindName(uint8_t m);
    static const char* powerTypeName(uint8_t p);
};

class WoweeVehicleLoader {
public:
    static bool save(const WoweeVehicle& cat,
                     const std::string& basePath);
    static WoweeVehicle load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-vehicles* variants.
    //
    //   makeStarter - 3 vehicles: 1-seat chopper, 2-seat
    //                  flying mount, 1-seat ground tank with
    //                  driver-only seat config.
    //   makeSiege   - 3 siege weapons (Demolisher / Catapult /
    //                  Cannon) with multi-seat layouts and
    //                  per-seat control spells.
    //   makeFlying  - 3 flying mounts (Wyrm / Drake / Gryphon)
    //                  with FlightCapability cross-refs to
    //                  WMNT entries.
    static WoweeVehicle makeStarter(const std::string& catalogName);
    static WoweeVehicle makeSiege(const std::string& catalogName);
    static WoweeVehicle makeFlying(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
