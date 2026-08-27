#include "game/cata_movement.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>

namespace wowee {
namespace game {

namespace {

// The core's names, in the core's spelling. Kept as a table rather than a
// switch so that adding an element the core grows is one line.
const std::unordered_map<std::string, MovementElement>& nameTable() {
    static const std::unordered_map<std::string, MovementElement> table = {
        {"MSEHasGuidByte0", MovementElement::HasGuidByte0},
        {"MSEHasGuidByte1", MovementElement::HasGuidByte1},
        {"MSEHasGuidByte2", MovementElement::HasGuidByte2},
        {"MSEHasGuidByte3", MovementElement::HasGuidByte3},
        {"MSEHasGuidByte4", MovementElement::HasGuidByte4},
        {"MSEHasGuidByte5", MovementElement::HasGuidByte5},
        {"MSEHasGuidByte6", MovementElement::HasGuidByte6},
        {"MSEHasGuidByte7", MovementElement::HasGuidByte7},
        {"MSEHasMovementFlags", MovementElement::HasMovementFlags},
        {"MSEHasMovementFlags2", MovementElement::HasMovementFlags2},
        {"MSEHasTimestamp", MovementElement::HasTimestamp},
        {"MSEHasOrientation", MovementElement::HasOrientation},
        {"MSEHasTransportData", MovementElement::HasTransportData},
        {"MSEHasTransportGuidByte0", MovementElement::HasTransportGuidByte0},
        {"MSEHasTransportGuidByte1", MovementElement::HasTransportGuidByte1},
        {"MSEHasTransportGuidByte2", MovementElement::HasTransportGuidByte2},
        {"MSEHasTransportGuidByte3", MovementElement::HasTransportGuidByte3},
        {"MSEHasTransportGuidByte4", MovementElement::HasTransportGuidByte4},
        {"MSEHasTransportGuidByte5", MovementElement::HasTransportGuidByte5},
        {"MSEHasTransportGuidByte6", MovementElement::HasTransportGuidByte6},
        {"MSEHasTransportGuidByte7", MovementElement::HasTransportGuidByte7},
        {"MSEHasTransportTime2", MovementElement::HasTransportTime2},
        {"MSEHasVehicleId", MovementElement::HasVehicleId},
        {"MSEHasPitch", MovementElement::HasPitch},
        {"MSEHasFallData", MovementElement::HasFallData},
        {"MSEHasFallDirection", MovementElement::HasFallDirection},
        {"MSEHasSplineElevation", MovementElement::HasSplineElevation},
        {"MSEHasSpline", MovementElement::HasSpline},
        {"MSEHasHeightChangeFailed", MovementElement::HasHeightChangeFailed},
        {"MSEGuidByte0", MovementElement::GuidByte0},
        {"MSEGuidByte1", MovementElement::GuidByte1},
        {"MSEGuidByte2", MovementElement::GuidByte2},
        {"MSEGuidByte3", MovementElement::GuidByte3},
        {"MSEGuidByte4", MovementElement::GuidByte4},
        {"MSEGuidByte5", MovementElement::GuidByte5},
        {"MSEGuidByte6", MovementElement::GuidByte6},
        {"MSEGuidByte7", MovementElement::GuidByte7},
        {"MSEMovementFlags", MovementElement::MovementFlags},
        {"MSEMovementFlags2", MovementElement::MovementFlags2},
        {"MSETimestamp", MovementElement::Timestamp},
        {"MSEPositionX", MovementElement::PositionX},
        {"MSEPositionY", MovementElement::PositionY},
        {"MSEPositionZ", MovementElement::PositionZ},
        {"MSEOrientation", MovementElement::Orientation},
        {"MSETransportGuidByte0", MovementElement::TransportGuidByte0},
        {"MSETransportGuidByte1", MovementElement::TransportGuidByte1},
        {"MSETransportGuidByte2", MovementElement::TransportGuidByte2},
        {"MSETransportGuidByte3", MovementElement::TransportGuidByte3},
        {"MSETransportGuidByte4", MovementElement::TransportGuidByte4},
        {"MSETransportGuidByte5", MovementElement::TransportGuidByte5},
        {"MSETransportGuidByte6", MovementElement::TransportGuidByte6},
        {"MSETransportGuidByte7", MovementElement::TransportGuidByte7},
        {"MSETransportPositionX", MovementElement::TransportPositionX},
        {"MSETransportPositionY", MovementElement::TransportPositionY},
        {"MSETransportPositionZ", MovementElement::TransportPositionZ},
        {"MSETransportOrientation", MovementElement::TransportOrientation},
        {"MSETransportSeat", MovementElement::TransportSeat},
        {"MSETransportTime", MovementElement::TransportTime},
        {"MSETransportTime2", MovementElement::TransportTime2},
        {"MSETransportVehicleId", MovementElement::TransportVehicleId},
        {"MSEPitch", MovementElement::Pitch},
        {"MSEFallTime", MovementElement::FallTime},
        {"MSEFallVerticalSpeed", MovementElement::FallVerticalSpeed},
        {"MSEFallCosAngle", MovementElement::FallCosAngle},
        {"MSEFallSinAngle", MovementElement::FallSinAngle},
        {"MSEFallHorizontalSpeed", MovementElement::FallHorizontalSpeed},
        {"MSESplineElevation", MovementElement::SplineElevation},
        {"MSECounter", MovementElement::Counter},
        {"MSEZeroBit", MovementElement::ZeroBit},
        {"MSEOneBit", MovementElement::OneBit},
        {"MSEFlushBits", MovementElement::FlushBits},
        {"MSEExtraElement", MovementElement::ExtraElement},
    };
    return table;
}

constexpr int guidIndex(MovementElement e, MovementElement base) {
    return static_cast<int>(e) - static_cast<int>(base);
}

}  // namespace

MovementElement movementElementFromName(const std::string& name) {
    const auto& table = nameTable();
    const auto it = table.find(name);
    return it == table.end() ? MovementElement::Unknown : it->second;
}

bool readCataMovement(network::Packet& packet,
                      const std::vector<MovementElement>& sequence,
                      MovementInfo& out,
                      CataMovementExtra& extra) {
    // The presence bits gate the value fields that come later, so they are
    // carried across the whole walk rather than consumed where they are read.
    bool hasMovementFlags = false;
    bool hasMovementFlags2 = false;
    bool hasTimestamp = false;
    bool hasOrientation = false;
    bool hasTransportData = false;
    bool hasTransportTime2 = false;
    bool hasVehicleId = false;
    bool hasPitch = false;
    bool hasFallData = false;
    bool hasFallDirection = false;
    bool hasSplineElevation = false;

    network::Packet::GuidBytes guid{};
    network::Packet::GuidBytes transportGuid{};
    uint32_t transportVehicleId = 0;

    // Every value read below needs bytes behind it. Checked before each rather
    // than after, because a short packet that half fills a position hands back
    // a plausible coordinate.
    const auto need = [&packet](size_t bytes) { return packet.hasRemaining(bytes); };

    for (MovementElement element : sequence) {
        switch (element) {
            case MovementElement::HasGuidByte0: case MovementElement::HasGuidByte1:
            case MovementElement::HasGuidByte2: case MovementElement::HasGuidByte3:
            case MovementElement::HasGuidByte4: case MovementElement::HasGuidByte5:
            case MovementElement::HasGuidByte6: case MovementElement::HasGuidByte7:
                guid[guidIndex(element, MovementElement::HasGuidByte0)] =
                    packet.readBit() ? 1u : 0u;
                break;

            case MovementElement::HasTransportGuidByte0: case MovementElement::HasTransportGuidByte1:
            case MovementElement::HasTransportGuidByte2: case MovementElement::HasTransportGuidByte3:
            case MovementElement::HasTransportGuidByte4: case MovementElement::HasTransportGuidByte5:
            case MovementElement::HasTransportGuidByte6: case MovementElement::HasTransportGuidByte7:
                if (hasTransportData)
                    transportGuid[guidIndex(element, MovementElement::HasTransportGuidByte0)] =
                        packet.readBit() ? 1u : 0u;
                break;

            case MovementElement::GuidByte0: case MovementElement::GuidByte1:
            case MovementElement::GuidByte2: case MovementElement::GuidByte3:
            case MovementElement::GuidByte4: case MovementElement::GuidByte5:
            case MovementElement::GuidByte6: case MovementElement::GuidByte7: {
                const int i = guidIndex(element, MovementElement::GuidByte0);
                if (guid[i] != 0) {
                    if (!need(1)) return false;
                    guid[i] ^= packet.readUInt8();
                }
                break;
            }

            case MovementElement::TransportGuidByte0: case MovementElement::TransportGuidByte1:
            case MovementElement::TransportGuidByte2: case MovementElement::TransportGuidByte3:
            case MovementElement::TransportGuidByte4: case MovementElement::TransportGuidByte5:
            case MovementElement::TransportGuidByte6: case MovementElement::TransportGuidByte7: {
                const int i = guidIndex(element, MovementElement::TransportGuidByte0);
                if (hasTransportData && transportGuid[i] != 0) {
                    if (!need(1)) return false;
                    transportGuid[i] ^= packet.readUInt8();
                }
                break;
            }

            // Inverted: a set bit means the field is absent. Five fields read
            // this way and the transport, fall and vehicle bits do not, which
            // is the half of this format that cannot be inferred.
            case MovementElement::HasMovementFlags:   hasMovementFlags = !packet.readBit(); break;
            case MovementElement::HasMovementFlags2:  hasMovementFlags2 = !packet.readBit(); break;
            case MovementElement::HasTimestamp:       hasTimestamp = !packet.readBit(); break;
            case MovementElement::HasOrientation:     hasOrientation = !packet.readBit(); break;
            case MovementElement::HasPitch:           hasPitch = !packet.readBit(); break;
            case MovementElement::HasSplineElevation: hasSplineElevation = !packet.readBit(); break;

            case MovementElement::HasTransportData:   hasTransportData = packet.readBit(); break;
            case MovementElement::HasFallData:        hasFallData = packet.readBit(); break;
            case MovementElement::HasTransportTime2:
                if (hasTransportData) hasTransportTime2 = packet.readBit();
                break;
            case MovementElement::HasVehicleId:
                if (hasTransportData) hasVehicleId = packet.readBit();
                break;
            case MovementElement::HasFallDirection:
                if (hasFallData) hasFallDirection = packet.readBit();
                break;

            case MovementElement::HasSpline:
                extra.hasSpline = packet.readBit();
                break;
            case MovementElement::HasHeightChangeFailed:
                packet.readBit();
                break;

            case MovementElement::MovementFlags:
                if (hasMovementFlags) out.flags = static_cast<uint32_t>(packet.readBits(30));
                break;
            case MovementElement::MovementFlags2:
                if (hasMovementFlags2) out.flags2 = static_cast<uint16_t>(packet.readBits(12));
                break;

            case MovementElement::Timestamp:
                if (hasTimestamp) {
                    if (!need(4)) return false;
                    out.time = packet.readUInt32();
                }
                break;

            case MovementElement::PositionX:
                if (!need(4)) return false;
                out.x = packet.readFloat();
                break;
            case MovementElement::PositionY:
                if (!need(4)) return false;
                out.y = packet.readFloat();
                break;
            case MovementElement::PositionZ:
                if (!need(4)) return false;
                out.z = packet.readFloat();
                break;
            case MovementElement::Orientation:
                if (hasOrientation) {
                    if (!need(4)) return false;
                    out.orientation = packet.readFloat();
                }
                break;

            case MovementElement::TransportPositionX:
                if (hasTransportData) { if (!need(4)) return false; out.transportX = packet.readFloat(); }
                break;
            case MovementElement::TransportPositionY:
                if (hasTransportData) { if (!need(4)) return false; out.transportY = packet.readFloat(); }
                break;
            case MovementElement::TransportPositionZ:
                if (hasTransportData) { if (!need(4)) return false; out.transportZ = packet.readFloat(); }
                break;
            case MovementElement::TransportOrientation:
                if (hasTransportData) { if (!need(4)) return false; out.transportO = packet.readFloat(); }
                break;
            case MovementElement::TransportSeat:
                if (hasTransportData) {
                    if (!need(1)) return false;
                    out.transportSeat = static_cast<int8_t>(packet.readUInt8());
                }
                break;
            case MovementElement::TransportTime:
                if (hasTransportData) { if (!need(4)) return false; out.transportTime = packet.readUInt32(); }
                break;
            case MovementElement::TransportTime2:
                if (hasTransportData && hasTransportTime2) {
                    if (!need(4)) return false;
                    out.transportTime2 = packet.readUInt32();
                }
                break;
            case MovementElement::TransportVehicleId:
                if (hasTransportData && hasVehicleId) {
                    if (!need(4)) return false;
                    transportVehicleId = packet.readUInt32();
                }
                break;

            case MovementElement::Pitch:
                if (hasPitch) {
                    if (!need(4)) return false;
                    out.pitch = packet.readFloat();
                }
                break;

            case MovementElement::FallTime:
                if (hasFallData) { if (!need(4)) return false; out.fallTime = packet.readUInt32(); }
                break;
            case MovementElement::FallVerticalSpeed:
                if (hasFallData) { if (!need(4)) return false; out.jumpVelocity = packet.readFloat(); }
                break;
            case MovementElement::FallCosAngle:
                if (hasFallData && hasFallDirection) {
                    if (!need(4)) return false;
                    out.jumpCosAngle = packet.readFloat();
                }
                break;
            case MovementElement::FallSinAngle:
                if (hasFallData && hasFallDirection) {
                    if (!need(4)) return false;
                    out.jumpSinAngle = packet.readFloat();
                }
                break;
            case MovementElement::FallHorizontalSpeed:
                if (hasFallData && hasFallDirection) {
                    if (!need(4)) return false;
                    out.jumpXYSpeed = packet.readFloat();
                }
                break;

            case MovementElement::SplineElevation:
                if (hasSplineElevation) {
                    if (!need(4)) return false;
                    extra.splineElevation = packet.readFloat();
                }
                break;

            case MovementElement::Counter:
                if (!need(4)) return false;
                extra.counter = packet.readUInt32();
                break;

            case MovementElement::ZeroBit:
            case MovementElement::OneBit:
                packet.readBit();
                break;

            case MovementElement::FlushBits:
                // The writer padded its bit byte out here. Only the eleven
                // SMSG layouts carry this, which is why the core's own reader
                // has no case for it: it never reads one.
                packet.resetBitReader();
                break;

            case MovementElement::ExtraElement:
                // Opcode-specific and not decoded here. Whatever follows in
                // this payload is now at an unknown offset, so the read stops
                // rather than returning fields read past it.
                extra.sawExtraElement = true;
                return false;

            case MovementElement::Unknown:
            default:
                return false;
        }
    }

    extra.guid = network::Packet::guidFromBytes(guid);
    out.transportGuid = network::Packet::guidFromBytes(transportGuid);
    (void)transportVehicleId;  // MovementInfo has no field for it yet
    return true;
}

bool CataMovementSequences::loadFromJson(const std::string& path) {
    sequences_.clear();
    unknownElements_.clear();

    std::ifstream file(path);
    if (!file.is_open()) return false;

    nlohmann::json root;
    try {
        file >> root;
    } catch (const nlohmann::json::exception&) {
        return false;
    }

    const auto sequences = root.find("sequences");
    if (sequences == root.end() || !sequences->is_object()) return false;

    for (const auto& [opcodeName, elements] : sequences->items()) {
        if (!elements.is_array()) continue;
        std::vector<MovementElement> decoded;
        decoded.reserve(elements.size());
        for (const auto& name : elements) {
            if (!name.is_string()) continue;
            const std::string text = name.get<std::string>();
            const MovementElement element = movementElementFromName(text);
            if (element == MovementElement::Unknown) unknownElements_.push_back(text);
            decoded.push_back(element);
        }
        sequences_.emplace(opcodeName, std::move(decoded));
    }
    return !sequences_.empty();
}

const std::vector<MovementElement>* CataMovementSequences::find(
    const std::string& opcodeName) const {
    const auto it = sequences_.find(opcodeName);
    return it == sequences_.end() ? nullptr : &it->second;
}

}  // namespace game
}  // namespace wowee
