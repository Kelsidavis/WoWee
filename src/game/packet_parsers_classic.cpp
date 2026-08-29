#include "game/packet_parsers.hpp"
#include "game/spline_packet.hpp"
#include "core/logger.hpp"
#include <cstdio>
#include <functional>
#include <string>

namespace wowee {
namespace game {

namespace {

std::string formatPacketBytes(const network::Packet& packet, size_t startPos) {
    const auto& rawData = packet.getData();
    if (startPos >= rawData.size()) {
        return {};
    }

    std::string hex;
    hex.reserve((rawData.size() - startPos) * 3);
    for (size_t i = startPos; i < rawData.size(); ++i) {
        char buf[4];
        std::snprintf(buf, sizeof(buf), "%02x ", rawData[i]);
        hex += buf;
    }
    if (!hex.empty()) {
        hex.pop_back();
    }
    return hex;
}

bool skipClassicSpellCastTargets(network::Packet& packet, uint64_t* primaryTargetGuid = nullptr) {
    if (!packet.hasRemaining(2)) {
        return false;
    }

    const uint16_t targetFlags = packet.readUInt16();

    const auto readPackedTargetGuid = [&](bool capture) -> bool {
        if (!packet.hasFullPackedGuid()) {
            return false;
        }
        const uint64_t guid = packet.readPackedGuid();
        if (capture && primaryTargetGuid && *primaryTargetGuid == 0) {
            *primaryTargetGuid = guid;
        }
        return true;
    };

    // Common Classic/Turtle SpellCastTargets payloads.
    if ((targetFlags & 0x0002) != 0 && !readPackedTargetGuid(true)) {   // UNIT
        return false;
    }
    if ((targetFlags & 0x0004) != 0 && !readPackedTargetGuid(false)) {  // UNIT_MINIPET/extra guid
        return false;
    }
    if ((targetFlags & 0x0010) != 0 && !readPackedTargetGuid(false)) {  // ITEM
        return false;
    }
    if ((targetFlags & 0x0800) != 0 && !readPackedTargetGuid(true)) {   // OBJECT
        return false;
    }
    if ((targetFlags & 0x8000) != 0 && !readPackedTargetGuid(false)) {  // CORPSE
        return false;
    }

    if ((targetFlags & 0x0020) != 0) {                                  // SOURCE_LOCATION
        if (!packet.hasRemaining(12)) {
            return false;
        }
        (void)packet.readFloat();
        (void)packet.readFloat();
        (void)packet.readFloat();
    }
    if ((targetFlags & 0x0040) != 0) {                                  // DEST_LOCATION
        if (!packet.hasRemaining(12)) {
            return false;
        }
        (void)packet.readFloat();
        (void)packet.readFloat();
        (void)packet.readFloat();
    }

    if ((targetFlags & 0x1000) != 0) {                                  // TRADE_ITEM
        if (!packet.hasRemaining(1)) {
            return false;
        }
        (void)packet.readUInt8();
    }

    if ((targetFlags & 0x2000) != 0) {                                  // STRING
        const auto& rawData = packet.getData();
        size_t pos = packet.getReadPos();
        while (pos < rawData.size() && rawData[pos] != 0) {
            ++pos;
        }
        if (pos >= rawData.size()) {
            return false;
        }
        packet.setReadPos(pos + 1);
    }

    return true;
}

} // namespace

// ============================================================================
// Classic 1.12.1 movement flag constants
// Key differences from TBC:
// - SPLINE_ENABLED at 0x00400000 (TBC/WotLK: 0x08000000)
// - No FLYING flag (flight was added in TBC)
// - ONTRANSPORT at 0x02000000 (not used for pitch in Classic)
// Same as TBC: ON_TRANSPORT=0x200, JUMPING=0x2000, SWIMMING=0x200000,
//              SPLINE_ELEVATION=0x04000000
// ============================================================================
namespace ClassicMoveFlags {
    constexpr uint32_t ONTRANSPORT      = 0x02000000;  // Gates transport data (vmangos authoritative)
    constexpr uint32_t JUMPING          = 0x00002000;  // Gates jump data
    constexpr uint32_t SWIMMING         = 0x00200000;  // Gates pitch
    constexpr uint32_t SPLINE_ENABLED   = 0x00400000;  // TBC/WotLK: 0x08000000
    constexpr uint32_t SPLINE_ELEVATION = 0x04000000;  // Same as TBC
}

uint32_t classicWireMoveFlags(uint32_t internalFlags) {
    uint32_t wireFlags = internalFlags;

    // Internal movement state is tracked with WotLK-era bits. Classic/Turtle
    // movement packets still use the older transport/jump flag layout.
    const uint32_t kInternalOnTransport = static_cast<uint32_t>(MovementFlags::ONTRANSPORT);
    const uint32_t kInternalFalling =
        static_cast<uint32_t>(MovementFlags::FALLING) |
        static_cast<uint32_t>(MovementFlags::FALLINGFAR);
    const uint32_t kClassicConflicts =
        static_cast<uint32_t>(MovementFlags::ASCENDING) |
        static_cast<uint32_t>(MovementFlags::CAN_FLY) |
        static_cast<uint32_t>(MovementFlags::FLYING) |
        static_cast<uint32_t>(MovementFlags::HOVER);

    wireFlags &= ~kClassicConflicts;

    if ((internalFlags & kInternalOnTransport) != 0) {
        wireFlags &= ~kInternalOnTransport;
        wireFlags |= ClassicMoveFlags::ONTRANSPORT;
    }

    if ((internalFlags & kInternalFalling) != 0) {
        wireFlags &= ~kInternalFalling;
        wireFlags |= ClassicMoveFlags::JUMPING;
    }

    return wireFlags;
}

// ============================================================================
// Classic parseMovementBlock
// Key differences from TBC:
// - NO moveFlags2 (TBC reads u8, WotLK reads u16)
// - SPLINE_ENABLED at 0x00400000 (not 0x08000000)
// - Transport data: NO timestamp (TBC adds u32 timestamp)
// - Pitch: only SWIMMING (no ONTRANSPORT secondary pitch, no FLYING)
// Same as TBC: u8 UpdateFlags, JUMPING=0x2000, 8 speeds, no pitchRate
// ============================================================================
bool ClassicPacketParsers::parseMovementBlock(network::Packet& packet, UpdateBlock& block) {
    auto rem = [&]() -> size_t { return packet.getRemainingSize(); };
    if (rem() < 1) return false;

    // Classic: UpdateFlags is uint8 (same as TBC)
    uint8_t updateFlags = packet.readUInt8();
    block.updateFlags = static_cast<uint16_t>(updateFlags);

    LOG_DEBUG("  [Classic] UpdateFlags: 0x", std::hex, static_cast<int>(updateFlags), std::dec);

    const uint8_t UPDATEFLAG_TRANSPORT       = 0x02;
    const uint8_t UPDATEFLAG_MELEE_ATTACKING = 0x04;
    const uint8_t UPDATEFLAG_HIGHGUID        = 0x08;
    const uint8_t UPDATEFLAG_ALL             = 0x10;
    const uint8_t UPDATEFLAG_LIVING          = 0x20;
    const uint8_t UPDATEFLAG_HAS_POSITION    = 0x40;
    // These bits are NOT the same as the other expansions' and must not be
    // merged with them. 1.12 assigns HIGHGUID 0x8 and ALL 0x10; 2.4.3 moved
    // HIGHGUID to 0x10 and put LOWGUID at 0x8, and 0x4 means MELEE_ATTACKING
    // here and HAS_TARGET there. Three tables that look like three copies of
    // one, and reading a stationary object's position off the wrong bit is a
    // misparse of everything after it.

    if (updateFlags & UPDATEFLAG_LIVING) {
        // Minimum: moveFlags(4)+time(4)+position(16)+fallTime(4)+speeds(24) = 52 bytes
        if (rem() < 52) return false;

        // Movement flags (u32 only - NO extra flags byte in Classic)
        uint32_t moveFlags = packet.readUInt32();
        /*uint32_t time =*/ packet.readUInt32();

        // Position
        block.x = packet.readFloat();
        block.y = packet.readFloat();
        block.z = packet.readFloat();
        block.orientation = packet.readFloat();
        block.hasMovement = true;

        LOG_DEBUG("  [Classic] LIVING: (", block.x, ", ", block.y, ", ", block.z,
                  "), o=", block.orientation, " moveFlags=0x", std::hex, moveFlags, std::dec);

        // Transport data (Classic: ONTRANSPORT=0x02000000, no timestamp)
        if (moveFlags & ClassicMoveFlags::ONTRANSPORT) {
            if (rem() < 1) return false;
            block.onTransport = true;
            block.transportGuid = packet.readPackedGuid();
            if (rem() < 16) return false; // 4 floats
            block.transportX = packet.readFloat();
            block.transportY = packet.readFloat();
            block.transportZ = packet.readFloat();
            block.transportO = packet.readFloat();
        }

        // Pitch (Classic: only SWIMMING, no FLYING or ONTRANSPORT pitch)
        if (moveFlags & ClassicMoveFlags::SWIMMING) {
            if (rem() < 4) return false;
            /*float pitch =*/ packet.readFloat();
        }

        // Fall time (always present)
        if (rem() < 4) return false;
        /*uint32_t fallTime =*/ packet.readUInt32();

        // Jumping (Classic: JUMPING=0x2000, same as TBC)
        if (moveFlags & ClassicMoveFlags::JUMPING) {
            if (rem() < 16) return false;
            /*float jumpVelocity =*/ packet.readFloat();
            /*float jumpSinAngle =*/ packet.readFloat();
            /*float jumpCosAngle =*/ packet.readFloat();
            /*float jumpXYSpeed =*/ packet.readFloat();
        }

        // Spline elevation
        if (moveFlags & ClassicMoveFlags::SPLINE_ELEVATION) {
            if (rem() < 4) return false;
            /*float splineElevation =*/ packet.readFloat();
        }

        // Speeds (Classic: 6 values - no flight speeds, no pitchRate)
        if (rem() < 24) return false;
        block.walkSpeed     = packet.readFloat();
        block.runSpeed      = packet.readFloat();
        block.runBackSpeed  = packet.readFloat();
        block.swimSpeed     = packet.readFloat();
        block.swimBackSpeed = packet.readFloat();
        block.turnRate      = packet.readFloat();
        block.moveFlags = moveFlags;

        // Spline data (Classic: SPLINE_ENABLED=0x00400000)
        if (moveFlags & ClassicMoveFlags::SPLINE_ENABLED) {
            SplineBlockData splineData;
            if (!parseClassicMoveUpdateSpline(packet, splineData)) return false;
        }
    }
    else if (updateFlags & UPDATEFLAG_HAS_POSITION) {
        if (rem() < 16) return false;
        block.x = packet.readFloat();
        block.y = packet.readFloat();
        block.z = packet.readFloat();
        block.orientation = packet.readFloat();
        block.hasMovement = true;

        LOG_DEBUG("  [Classic] STATIONARY: (", block.x, ", ", block.y, ", ", block.z, ")");
    }

    // High GUID
    if (updateFlags & UPDATEFLAG_HIGHGUID) {
        if (rem() < 4) return false;
        /*uint32_t highGuid =*/ packet.readUInt32();
    }

    // ALL/SELF extra uint32
    if (updateFlags & UPDATEFLAG_ALL) {
        if (rem() < 4) return false;
        /*uint32_t unkAll =*/ packet.readUInt32();
    }

    // Current melee target as packed guid
    if (updateFlags & UPDATEFLAG_MELEE_ATTACKING) {
        if (rem() < 1) return false;
        /*uint64_t meleeTargetGuid =*/ packet.readPackedGuid();
    }

    // Transport progress / world time
    if (updateFlags & UPDATEFLAG_TRANSPORT) {
        if (rem() < 4) return false;
        /*uint32_t transportTime =*/ packet.readUInt32();
    }

    return true;
}

// ============================================================================
// Classic writeMovementPayload
// Key differences from TBC:
// - NO flags2 byte (TBC writes u8)
// - Transport data: NO timestamp
// - Pitch: only SWIMMING (no ONTRANSPORT pitch)
// ============================================================================
void ClassicPacketParsers::writeMovementPayload(network::Packet& packet, const MovementInfo& info) {
    const uint32_t wireFlags = classicWireMoveFlags(info.flags);

    // Movement flags (uint32)
    packet.writeUInt32(wireFlags);

    // Classic: NO flags2 byte (TBC has u8, WotLK has u16)

    // Timestamp
    packet.writeUInt32(info.time);

    // Position
    packet.writeFloat(info.x);
    packet.writeFloat(info.y);
    packet.writeFloat(info.z);
    packet.writeFloat(info.orientation);

    // Transport data (Classic ONTRANSPORT = 0x02000000, no timestamp)
    if (wireFlags & ClassicMoveFlags::ONTRANSPORT) {
        // Packed, which is the same wire format in every expansion: a mask
        // byte saying which of the eight are non-zero, then those bytes.
        packet.writePackedGuid(info.transportGuid);

        // Transport local position
        packet.writeFloat(info.transportX);
        packet.writeFloat(info.transportY);
        packet.writeFloat(info.transportZ);
        packet.writeFloat(info.transportO);

        // Classic: NO transport timestamp
        // Classic: NO transport seat byte
    }

    // Pitch (Classic: only SWIMMING)
    if (wireFlags & ClassicMoveFlags::SWIMMING) {
        packet.writeFloat(info.pitch);
    }

    // Fall time (always present)
    packet.writeUInt32(info.fallTime);

    // Jump data (Classic JUMPING = 0x2000)
    if (wireFlags & ClassicMoveFlags::JUMPING) {
        packet.writeFloat(info.jumpVelocity);
        packet.writeFloat(info.jumpSinAngle);
        packet.writeFloat(info.jumpCosAngle);
        packet.writeFloat(info.jumpXYSpeed);
    }
}

// ============================================================================
// Classic buildMovementPacket
// Classic/TBC: client movement packets do NOT include PackedGuid prefix
// (WotLK added PackedGuid to client packets)
// ============================================================================
network::Packet ClassicPacketParsers::buildMovementPacket(LogicalOpcode opcode,
                                                           const MovementInfo& info,
                                                           uint64_t /*playerGuid*/) {
    network::Packet packet(wireOpcode(opcode));

    // Classic: NO PackedGuid prefix for client packets
    writeMovementPayload(packet, info);

    return packet;
}

// ============================================================================
// Classic buildCastSpell
// Vanilla 1.12.x: NO castCount prefix, NO castFlags byte
// Format: uint32 spellId + uint16 targetFlags + [PackedGuid if unit target]
// ============================================================================
network::Packet ClassicPacketParsers::buildCastSpell(uint32_t spellId, uint64_t targetGuid, uint8_t /*castCount*/) {
    network::Packet packet(wireOpcode(LogicalOpcode::CMSG_CAST_SPELL));

    packet.writeUInt32(spellId);

    // SpellCastTargets - vanilla/CMaNGOS uses uint16 target mask (WotLK uses uint32)
    if (targetGuid != 0) {
        packet.writeUInt16(0x02); // TARGET_FLAG_UNIT

        // Write packed GUID
        packet.writePackedGuid(targetGuid);
    } else {
        packet.writeUInt16(0x00); // TARGET_FLAG_SELF
    }

    return packet;
}

network::Packet ClassicPacketParsers::buildCastGameObjectSpell(uint32_t spellId, uint64_t targetGuid, uint8_t /*castCount*/) {
    network::Packet packet(wireOpcode(LogicalOpcode::CMSG_CAST_SPELL));
    packet.writeUInt32(spellId);
    packet.writeUInt16(0x0800); // TARGET_FLAG_GAMEOBJECT
    packet.writePackedGuid(targetGuid);
    LOG_DEBUG("[Classic] Built CMSG_CAST_SPELL: spell=", spellId, " gameObject=0x",
              std::hex, targetGuid, std::dec, " size=", packet.getSize());
    return packet;
}

// ============================================================================
// Classic CMSG_USE_ITEM
// Vanilla 1.12.x: bag(u8) + slot(u8) + spellIndex(u8) + SpellCastTargets(u16)
// NO spellId, itemGuid, glyphIndex, or castFlags fields (those are WotLK)
// ============================================================================
network::Packet ClassicPacketParsers::buildUseItem(uint8_t bagIndex, uint8_t slotIndex,
                                                   uint64_t /*itemGuid*/, uint32_t /*spellId*/,
                                                   uint64_t targetGuid, uint64_t itemTargetGuid) {
    network::Packet packet(wireOpcode(LogicalOpcode::CMSG_USE_ITEM));
    packet.writeUInt8(bagIndex);
    packet.writeUInt8(slotIndex);
    packet.writeUInt8(0);       // spell_index (which item spell to trigger, usually 0)
    if (itemTargetGuid != 0) {
        packet.writeUInt16(0x0010); // TARGET_FLAG_ITEM
        packet.writePackedGuid(itemTargetGuid);
    } else if (targetGuid != 0) {
        packet.writeUInt16(0x0002); // TARGET_FLAG_UNIT
        packet.writePackedGuid(targetGuid);
    } else {
        packet.writeUInt16(0x0000); // TARGET_FLAG_SELF
    }
    return packet;
}

// ============================================================================
// Classic parseSpellStart - Vanilla 1.12 SMSG_SPELL_START
//
// Key differences from TBC:
//   - GUIDs are PackedGuid (variable-length byte mask + non-zero bytes),
//     NOT full uint64 as in TBC/WotLK.
//   - castFlags is uint16 (NOT uint32 as in TBC/WotLK).
//   - SpellCastTargets uses uint16 targetFlags (NOT uint32 as in TBC).
//
// Format: PackedGuid(casterObj) + PackedGuid(casterUnit)
//       + uint32(spellId) + uint16(castFlags) + uint32(castTime)
//       + uint16(targetFlags) [+ PackedGuid(unitTarget) if TARGET_FLAG_UNIT]
// ============================================================================
bool ClassicPacketParsers::parseSpellStart(network::Packet& packet, SpellStartData& data) {
    data = SpellStartData{};

    auto rem = [&]() { return packet.getRemainingSize(); };
    const size_t startPos = packet.getReadPos();
    if (rem() < 2) return false;

    if (!packet.hasFullPackedGuid()) {
        packet.setReadPos(startPos);
        return false;
    }
    data.casterGuid = packet.readPackedGuid();
    if (!packet.hasFullPackedGuid()) {
        packet.setReadPos(startPos);
        return false;
    }
    data.casterUnit = packet.readPackedGuid();

    // Vanilla/Turtle SMSG_SPELL_START does not include castCount here.
    // Layout after the two packed GUIDs is spellId(u32) + castFlags(u16) + castTime(u32).
    if (rem() < 10) return false;
    data.castCount = 0;
    data.spellId   = packet.readUInt32();
    data.castFlags = packet.readUInt16();   // uint16 in Vanilla (uint32 in TBC/WotLK)
    data.castTime  = packet.readUInt32();

    // SpellCastTargets: consume ALL target payload types so subsequent reads stay aligned.
    // Previously only UNIT(0x02)/OBJECT(0x800) were handled; DEST_LOCATION(0x40),
    // SOURCE_LOCATION(0x20), and ITEM(0x10) bytes were silently skipped, corrupting
    // castFlags/castTime for every AOE/ground-targeted spell (Rain of Fire, Blizzard, etc.).
    {
        uint64_t targetGuid = 0;
        // skipClassicSpellCastTargets reads uint16 targetFlags and all payloads.
        // Non-fatal on truncation: self-cast spells have zero-byte targets.
        skipClassicSpellCastTargets(packet, &targetGuid);
        data.targetGuid = targetGuid;
    }

    LOG_DEBUG("[Classic] Spell start: spell=", data.spellId, " castTime=", data.castTime, "ms",
              " targetGuid=0x", std::hex, data.targetGuid, std::dec);
    return true;
}

// ============================================================================
// Classic parseSpellGo - Vanilla 1.12 SMSG_SPELL_GO
//
// Same GUID and castFlags format differences as parseSpellStart:
//   - GUIDs are PackedGuid (not full uint64)
//   - castFlags is uint16 (not uint32)
//   - Hit/miss target GUIDs are also PackedGuid in Vanilla
//
// Format: PackedGuid(casterObj) + PackedGuid(casterUnit)
//       + uint32(spellId) + uint16(castFlags)
//       + uint8(hitCount) + [PackedGuid(hitTarget) × hitCount]
//       + uint8(missCount) + [PackedGuid(missTarget) + uint8(missType)] × missCount
// ============================================================================
bool ClassicPacketParsers::parseSpellGo(network::Packet& packet, SpellGoData& data) {
    // Always reset output to avoid stale targets when callers reuse buffers.
    data = SpellGoData{};

    auto rem = [&]() { return packet.getRemainingSize(); };
    const size_t startPos = packet.getReadPos();
    const bool traceSmallSpellGo = (packet.getSize() - startPos) <= 48;
    const auto traceFailure = [&](const char* stage, size_t pos, uint32_t value = 0) {
        if (!traceSmallSpellGo) {
            return;
        }
        static uint32_t smallSpellGoTraceCount = 0;
        ++smallSpellGoTraceCount;
        if (smallSpellGoTraceCount > 12 && (smallSpellGoTraceCount % 50) != 0) {
            return;
        }
        LOG_WARNING("[Classic] Spell go trace: stage=", stage,
                    " pos=", pos,
                    " size=", packet.getSize() - startPos,
                    " spell=", data.spellId,
                    " castFlags=0x", std::hex, data.castFlags, std::dec,
                    " value=", value,
                    " bytes=[", formatPacketBytes(packet, startPos), "]");
    };
    if (rem() < 2) return false;

    if (!packet.hasFullPackedGuid()) return false;
    data.casterGuid = packet.readPackedGuid();
    if (!packet.hasFullPackedGuid()) return false;
    data.casterUnit = packet.readPackedGuid();

    // Vanilla/Turtle SMSG_SPELL_GO does not include castCount here.
    // Layout after the two packed GUIDs is spellId(u32) + castFlags(u16).
    if (rem() < 6) return false;
    data.castCount = 0;
    data.spellId   = packet.readUInt32();
    data.castFlags = packet.readUInt16();   // uint16 in Vanilla (uint32 in TBC/WotLK)

    const size_t countsPos = packet.getReadPos();
    uint64_t ignoredTargetGuid = 0;
    std::function<bool(bool)> parseHitAndMissLists = [&](bool allowTargetsFallback) -> bool {
        packet.setReadPos(countsPos);
        data.hitTargets.clear();
        data.missTargets.clear();
        ignoredTargetGuid = 0;

        // hitCount is mandatory in SMSG_SPELL_GO. Missing byte means truncation.
        if (rem() < 1) {
            LOG_WARNING("[Classic] Spell go: missing hitCount after fixed fields");
            traceFailure("missing_hit_count", packet.getReadPos());
            packet.setReadPos(startPos);
            return false;
        }
        const uint8_t rawHitCount = packet.readUInt8();
        if (rawHitCount > 128) {
            LOG_WARNING("[Classic] Spell go: hitCount capped (requested=", static_cast<int>(rawHitCount), ")");
        }
        if (rem() < static_cast<size_t>(rawHitCount) + 1u) {
            static uint32_t badHitCountTrunc = 0;
            ++badHitCountTrunc;
            if (badHitCountTrunc <= 10 || (badHitCountTrunc % 100) == 0) {
                LOG_WARNING("[Classic] Spell go: invalid hitCount/remaining (hits=", static_cast<int>(rawHitCount),
                            " remaining=", rem(), " occurrence=", badHitCountTrunc, ")");
            }
            traceFailure("invalid_hit_count", packet.getReadPos(), rawHitCount);
            packet.setReadPos(startPos);
            return false;
        }

        const auto parseHitList = [&](bool usePackedGuids) -> bool {
            packet.setReadPos(countsPos + 1); // after hitCount
            data.hitTargets.clear();
            const uint8_t storedHitLimit = std::min<uint8_t>(rawHitCount, 128);
            data.hitTargets.reserve(storedHitLimit);
            for (uint16_t i = 0; i < rawHitCount; ++i) {
                uint64_t targetGuid = 0;
                if (usePackedGuids) {
                    if (!packet.hasFullPackedGuid()) {
                        return false;
                    }
                    targetGuid = packet.readPackedGuid();
                } else {
                    if (rem() < 8) {
                        return false;
                    }
                    targetGuid = packet.readUInt64();
                }
                if (i < storedHitLimit) {
                    data.hitTargets.push_back(targetGuid);
                }
            }
            data.hitCount = static_cast<uint8_t>(data.hitTargets.size());
            return true;
        };

        // Full eight-byte GUIDs first, because that is what the wire carries.
        // Turtle's own server writes this list as `*data << ihit.targetGUID`,
        // and ObjectGuid's stream operator is `buf << uint64(GetRawValue())`;
        // packed is written explicitly, through GetPackGUID, and SendSpellGo
        // uses it for exactly two fields - the caster and the caster unit -
        // which are the two this parser already reads as packed above.
        // mangos-zero writes it the same way, so this holds for vanilla cores
        // generally.
        //
        // #135 turned this order round on the reading that the layout
        // documented above says packed. It does not: the packed guids it names
        // are the two at the head. Packed first would consume a full guid's
        // bytes by its mask and shift every field after the lists.
        if (!parseHitList(false) && !parseHitList(true)) {
            LOG_WARNING("[Classic] Spell go: truncated hit targets at index 0/", static_cast<int>(rawHitCount));
            traceFailure("truncated_hit_target", packet.getReadPos(), rawHitCount);
            packet.setReadPos(startPos);
            return false;
        }

        std::function<bool(size_t, bool)> parseMissListFrom = [&](size_t missStartPos,
                                                                  bool allowMidTargetsFallback) -> bool {
            packet.setReadPos(missStartPos);
            data.missTargets.clear();

            if (rem() < 1) {
                LOG_WARNING("[Classic] Spell go: missing missCount after hit target list");
                traceFailure("missing_miss_count", packet.getReadPos());
                packet.setReadPos(startPos);
                return false;
            }
            const uint8_t rawMissCount = packet.readUInt8();
            if (rawMissCount > 128) {
                LOG_WARNING("[Classic] Spell go: missCount capped (requested=", static_cast<int>(rawMissCount), ")");
                traceFailure("miss_count_capped", packet.getReadPos() - 1, rawMissCount);
            }
            if (rem() < static_cast<size_t>(rawMissCount) * 2u) {
                if (allowMidTargetsFallback) {
                    packet.setReadPos(missStartPos);
                    if (skipClassicSpellCastTargets(packet, &ignoredTargetGuid)) {
                        traceFailure("mid_targets_fallback", missStartPos, ignoredTargetGuid != 0 ? 1u : 0u);
                        return parseMissListFrom(packet.getReadPos(), false);
                    }
                }
                if (allowTargetsFallback) {
                    packet.setReadPos(countsPos);
                    if (skipClassicSpellCastTargets(packet, &ignoredTargetGuid)) {
                        traceFailure("pre_targets_fallback", countsPos, ignoredTargetGuid != 0 ? 1u : 0u);
                        return parseHitAndMissLists(false);
                    }
                }

                static uint32_t badMissCountTrunc = 0;
                ++badMissCountTrunc;
                if (badMissCountTrunc <= 10 || (badMissCountTrunc % 100) == 0) {
                    LOG_WARNING("[Classic] Spell go: invalid missCount/remaining (misses=", static_cast<int>(rawMissCount),
                                " remaining=", rem(), " occurrence=", badMissCountTrunc, ")");
                }
                traceFailure("invalid_miss_count", packet.getReadPos(), rawMissCount);
                packet.setReadPos(startPos);
                return false;
            }

            const uint8_t storedMissLimit = std::min<uint8_t>(rawMissCount, 128);
            data.missTargets.reserve(storedMissLimit);
            bool truncatedMissTargets = false;
            const auto parseMissEntry = [&](SpellGoMissEntry& m, bool usePackedGuid) -> bool {
                if (usePackedGuid) {
                    if (!packet.hasFullPackedGuid()) {
                        return false;
                    }
                    m.targetGuid = packet.readPackedGuid();
                } else {
                    if (rem() < 8) {
                        return false;
                    }
                    m.targetGuid = packet.readUInt64();
                }
                return true;
            };
            for (uint16_t i = 0; i < rawMissCount; ++i) {
                SpellGoMissEntry m;
                const size_t missEntryPos = packet.getReadPos();
                // Full first here too, and for the same reason: the miss
                // list is written with the same operator as the hit list.
                if (!parseMissEntry(m, false)) {
                    packet.setReadPos(missEntryPos);
                    if (!parseMissEntry(m, true)) {
                        LOG_WARNING("[Classic] Spell go: truncated miss targets at index ", i,
                                    "/", static_cast<int>(rawMissCount));
                        traceFailure("truncated_miss_target", packet.getReadPos(), i);
                        truncatedMissTargets = true;
                        break;
                    }
                }
                if (rem() < 1) {
                    LOG_WARNING("[Classic] Spell go: missing missType at miss index ", i,
                                "/", static_cast<int>(rawMissCount));
                    traceFailure("missing_miss_type", packet.getReadPos(), i);
                    truncatedMissTargets = true;
                    break;
                }
                m.missType = packet.readUInt8();
                if (m.missType == 11) {
                    if (rem() < 1) {
                        LOG_WARNING("[Classic] Spell go: truncated reflect payload at miss index ", i,
                                    "/", static_cast<int>(rawMissCount));
                        traceFailure("truncated_reflect", packet.getReadPos(), i);
                        truncatedMissTargets = true;
                        break;
                    }
                    (void)packet.readUInt8();
                }
                if (i < storedMissLimit) {
                    data.missTargets.push_back(m);
                }
            }
            if (truncatedMissTargets) {
                packet.setReadPos(startPos);
                return false;
            }
            data.missCount = static_cast<uint8_t>(data.missTargets.size());
            return true;
        };

        return parseMissListFrom(packet.getReadPos(), true);
    };

    if (!parseHitAndMissLists(true)) {
        return false;
    }

    // SpellCastTargets follows the miss list - consume all target bytes so that
    // any subsequent fields (e.g. castFlags extras) are not misaligned.
    skipClassicSpellCastTargets(packet, &data.targetGuid);

    LOG_DEBUG("[Classic] Spell go: spell=", data.spellId, " hits=", static_cast<int>(data.hitCount),
              " misses=", static_cast<int>(data.missCount));
    return true;
}

// ============================================================================
// Classic parseAttackerStateUpdate - Vanilla 1.12 SMSG_ATTACKERSTATEUPDATE
//
// Identical to TBC format except GUIDs are PackedGuid (not full uint64).
// Format: uint32(hitInfo) + PackedGuid(attacker) + PackedGuid(target)
//       + int32(totalDamage) + uint8(subDamageCount)
//       + [per sub: uint32(schoolMask) + float(damage) + uint32(intDamage)
//                 + uint32(absorbed) + uint32(resisted)]
//       + uint32(victimState) + int32(overkill) [+ uint32(blocked)]
// ============================================================================
bool ClassicPacketParsers::parseAttackerStateUpdate(network::Packet& packet, AttackerStateUpdateData& data) {
    data = AttackerStateUpdateData{};

    auto rem = [&]() { return packet.getRemainingSize(); };
    if (rem() < 5) return false;  // hitInfo + at least one packed guid mask byte

    const size_t startPos = packet.getReadPos();
    if (!data.readCommonHead(packet, startPos)) return false;

    if (rem() < 8) {
        packet.setReadPos(startPos);
        return false;
    }
    data.victimState = packet.readUInt32();
    data.overkill    = static_cast<int32_t>(packet.readUInt32());

    if (rem() >= 4) {
        data.blocked = packet.readUInt32();
    }

    LOG_DEBUG("[Classic] Melee hit: ", data.totalDamage, " damage",
              data.isCrit() ? " (CRIT)" : "",
              data.isMiss() ? " (MISS)" : "");
    return true;
}

// ============================================================================
// Classic parseSpellDamageLog - Vanilla 1.12 SMSG_SPELLNONMELEEDAMAGELOG
//
// Identical to TBC except GUIDs are PackedGuid (not full uint64).
// Format: PackedGuid(target) + PackedGuid(caster) + uint32(spellId)
//       + uint32(damage) + uint8(schoolMask) + uint32(absorbed) + uint32(resisted)
//       + uint8(periodicLog) + uint8(unused) + uint32(blocked) + uint32(flags)
// ============================================================================
bool parseSpellDamageLogPreWotlk(network::Packet& packet, SpellDamageLogData& data,
                                 const char* tag) {
    data = SpellDamageLogData{};
    auto rem = [&]() { return packet.getRemainingSize(); };
    if (rem() < 2 || !packet.hasFullPackedGuid()) return false;

    data.targetGuid   = packet.readPackedGuid(); // PackedGuid before WotLK
    if (rem() < 1 || !packet.hasFullPackedGuid()) return false;
    data.attackerGuid = packet.readPackedGuid();

    // uint32(spellId) + uint32(damage) + uint8(schoolMask) + uint32(absorbed)
    // + uint32(resisted) + uint8 + uint8 + uint32(blocked) + uint32(flags) = 21 bytes.
    // No overkill: that is the field WotLK adds.
    if (rem() < 21) return false;
    data.spellId    = packet.readUInt32();
    data.damage     = packet.readUInt32();
    data.schoolMask = packet.readUInt8();
    data.absorbed   = packet.readUInt32();
    data.resisted   = packet.readUInt32();
    packet.readUInt8();    // periodicLog
    packet.readUInt8();    // unused
    packet.readUInt32();   // blocked
    const uint32_t flags = packet.readUInt32();
    data.isCrit     = (flags & 0x02) != 0;

    LOG_DEBUG(tag, " Spell damage: spellId=", data.spellId, " dmg=", data.damage,
              data.isCrit ? " CRIT" : "");
    return true;
}

bool ClassicPacketParsers::parseSpellDamageLog(network::Packet& packet, SpellDamageLogData& data) {
    return parseSpellDamageLogPreWotlk(packet, data, "[Classic]");
}

// ============================================================================
// Classic parseSpellHealLog - Vanilla 1.12 SMSG_SPELLHEALLOG
//
// Identical to TBC except GUIDs are PackedGuid (not full uint64).
// Format: PackedGuid(target) + PackedGuid(caster) + uint32(spellId)
//       + uint32(heal) + uint32(overheal) + uint8(crit)
// ============================================================================
bool ClassicPacketParsers::parseSpellHealLog(network::Packet& packet, SpellHealLogData& data) {
    auto rem = [&]() { return packet.getRemainingSize(); };
    if (rem() < 2 || !packet.hasFullPackedGuid()) return false;

    data.targetGuid = packet.readPackedGuid(); // PackedGuid in Vanilla
    if (rem() < 1 || !packet.hasFullPackedGuid()) return false;
    data.casterGuid = packet.readPackedGuid(); // PackedGuid in Vanilla

    if (rem() < 13) return false;  // uint32 + uint32 + uint32 + uint8 = 13 bytes
    data.spellId  = packet.readUInt32();
    data.heal     = packet.readUInt32();
    data.overheal = packet.readUInt32();
    data.isCrit   = (packet.readUInt8() != 0);

    LOG_DEBUG("[Classic] Spell heal: spellId=", data.spellId, " heal=", data.heal,
              data.isCrit ? " CRIT" : "");
    return true;
}

// ============================================================================
// Classic parseAuraUpdate - Vanilla 1.12 SMSG_AURA_UPDATE / SMSG_AURA_UPDATE_ALL
//
// Classic has SMSG_AURA_UPDATE (TBC does not - TBC uses a different aura system
// and the TBC override returns false with a warning).  Classic inherits TBC's
// override by default, so this override is needed to restore aura tracking.
//
// Classic aura flags differ from WotLK:
//   0x01/0x02/0x04 = effect indices active   (same as WotLK)
//   0x08 = CANCELABLE / NOT-NEGATIVE         (WotLK: 0x08 = NOT_CASTER)
//   0x10 = DURATION                          (WotLK: 0x20 = DURATION)
//   0x20 = NOT_CASTER                        (WotLK: no caster GUID at all if 0x08)
//   0x40 = POSITIVE                          (WotLK: 0x40 = EFFECT_AMOUNTS)
//
// Key differences from WotLK parser:
//   - No caster GUID field in Classic SMSG_AURA_UPDATE packets
//   - DURATION bit is 0x10, not 0x20
//   - No effect amounts field (WotLK 0x40 = EFFECT_AMOUNTS does not exist here)
//
// Format: PackedGuid(entity) + [uint8(slot) + uint32(spellId)
//         [+ uint8(flags) + uint8(level) + uint8(charges)
//          + [uint32(maxDuration) + uint32(duration) if flags & 0x10]]*
// ============================================================================
bool ClassicPacketParsers::parseAuraUpdate(network::Packet& packet, AuraUpdateData& data, bool isAll) {
    auto rem = [&]() { return packet.getRemainingSize(); };
    if (rem() < 1) return false;

    data.guid = packet.readPackedGuid();

    while (rem() > 0) {
        if (rem() < 1) break;
        uint8_t slot = packet.readUInt8();
        if (rem() < 4) break;
        uint32_t spellId = packet.readUInt32();

        AuraSlot aura;
        if (spellId != 0) {
            aura.spellId = spellId;
            if (rem() < 3) { data.updates.emplace_back(slot, aura); break; }
            aura.flags   = packet.readUInt8();
            aura.level   = packet.readUInt8();
            aura.charges = packet.readUInt8();

            // Classic DURATION flag is 0x10 (WotLK uses 0x20)
            if ((aura.flags & 0x10) && rem() >= 8) {
                aura.maxDurationMs = static_cast<int32_t>(packet.readUInt32());
                aura.durationMs    = static_cast<int32_t>(packet.readUInt32());
            }
            // No caster GUID field in Classic (WotLK added it gated by 0x08 NOT_CASTER)
            // No effect amounts field in Classic (WotLK added it gated by 0x40)
        }

        data.updates.emplace_back(slot, aura);
        if (!isAll) break;
    }

    LOG_DEBUG("[Classic] Aura update for 0x", std::hex, data.guid, std::dec,
              ": ", data.updates.size(), " slots");
    return true;
}

// ============================================================================
// Classic SMSG_NAME_QUERY_RESPONSE format (1.12 / vmangos):
//   uint64 guid (full, GetObjectGuid)
//   CString name
//   CString realmName (usually empty = single \0 byte)
//   uint32 race
//   uint32 gender
//   uint32 class
//
// TBC Variant A (inherited from TbcPacketParsers) skips the realmName CString,
// causing it to misread the uint32 race field (absorbs the realmName \0 byte
// as the low byte), producing race=0 and shifted gender/class values.
// ============================================================================
bool ClassicPacketParsers::parseNameQueryResponse(network::Packet& packet, NameQueryResponseData& data) {
    data = NameQueryResponseData{};

    auto rem = [&]() { return packet.getRemainingSize(); };
    if (rem() < 8) return false;

    data.guid = packet.readUInt64();     // full uint64, not PackedGuid
    data.name = packet.readString();     // null-terminated name
    if (rem() == 0) return !data.name.empty();

    data.realmName = packet.readString(); // null-terminated realm name (usually "")
    if (rem() < 12) return !data.name.empty();

    uint32_t race   = packet.readUInt32();
    uint32_t gender = packet.readUInt32();
    uint32_t cls    = packet.readUInt32();
    data.race    = static_cast<uint8_t>(race   & 0xFF);
    data.gender  = static_cast<uint8_t>(gender & 0xFF);
    data.classId = static_cast<uint8_t>(cls    & 0xFF);
    data.found   = 0;

    LOG_DEBUG("[Classic] Name query response: ", data.name,
              " (race=", static_cast<int>(data.race), " gender=", static_cast<int>(data.gender),
              " class=", static_cast<int>(data.classId), ")");
    return !data.name.empty();
}

// ============================================================================
// Classic SMSG_CAST_FAILED: no castCount byte (added in TBC/WotLK)
// Format: spellId(u32) + result(u8)
// ============================================================================
bool ClassicPacketParsers::parseCastFailed(network::Packet& packet, CastFailedData& data) {
    data.castCount = 0;
    data.spellId = packet.readUInt32();
    uint8_t vanillaResult = packet.readUInt8();
    // Vanilla enum starts at 0=AFFECTING_COMBAT (no SUCCESS entry).
    // WotLK enum starts at 0=SUCCESS, 1=AFFECTING_COMBAT.
    // Shift +1 to align with WotLK result strings.
    data.result = vanillaResult + 1;
    // Spell-focus and totem failures append ids naming the missing station/tool
    readCastResultArgs(packet, data.result, data.miscArg, data.miscArg2);
    LOG_DEBUG("[Classic] Cast failed: spell=", data.spellId, " vanillaResult=", static_cast<int>(vanillaResult));
    return true;
}

// ============================================================================
// Classic SMSG_CAST_RESULT: same layout as parseCastFailed (spellId + result),
// but the result enum starts at 0=AFFECTING_COMBAT (no SUCCESS entry).
// Apply the same +1 shift used in parseCastFailed so the result codes
// align with WotLK's getSpellCastResultString table.
// ============================================================================
bool ClassicPacketParsers::parseCastResult(network::Packet& packet, uint32_t& spellId, uint8_t& result,
                                           uint32_t& miscArg, uint32_t& miscArg2) {
    if (!packet.hasRemaining(5)) return false;
    spellId = packet.readUInt32();
    uint8_t vanillaResult = packet.readUInt8();
    // Shift +1: Vanilla result 0=AFFECTING_COMBAT maps to WotLK result 1=AFFECTING_COMBAT
    result = vanillaResult + 1;
    // Spell-focus and totem failures append ids naming the missing station/tool
    readCastResultArgs(packet, result, miscArg, miscArg2);
    LOG_DEBUG("[Classic] Cast result: spell=", spellId, " vanillaResult=", static_cast<int>(vanillaResult));
    return true;
}

// ============================================================================
// Classic 1.12.1 parseCharEnum
// Differences from TBC:
// - Equipment: 20 items, but NO enchantment field per slot
//   Classic: displayId(u32) + inventoryType(u8) = 5 bytes/slot
//   TBC/WotLK: displayId(u32) + inventoryType(u8) + enchant(u32) = 9 bytes/slot
// - After flags: uint8 firstLogin (same as TBC)
// ============================================================================
bool ClassicPacketParsers::parseCharEnum(network::Packet& packet, CharEnumResponse& response) {
    // Vanilla's char enum has no per-item enchantment field, which is the only
    // way it differs from TBC's.
    return parseCharEnumPreWotlk(packet, response, /*hasEnchantment=*/false, "[Classic]");
}

// ============================================================================
// Classic 1.12.1 parseMessageChat
// Differences from WotLK:
// - NO uint32 unknown field after senderGuid
// - CHANNEL type: channelName + rank(u32) + senderGuid (not just channelName)
// - No ACHIEVEMENT/GUILD_ACHIEVEMENT types
// ============================================================================
bool ClassicPacketParsers::parseMessageChat(network::Packet& packet, MessageChatData& data) {
    if (packet.getSize() < 10) {
        LOG_ERROR("[Classic] SMSG_MESSAGECHAT packet too small: ", packet.getSize(), " bytes");
        return false;
    }

    // Read chat type
    uint8_t typeVal = packet.readUInt8();
    data.type = static_cast<ChatType>(typeVal);

    // Read language
    uint32_t langVal = packet.readUInt32();
    data.language = static_cast<ChatLanguage>(langVal);

    // Classic: NO uint32 unknown field here (WotLK has one)

    // Type-specific data (matches CMaNGOS-Classic BuildChatPacket)
    switch (data.type) {
        case ChatType::MONSTER_EMOTE: {
            // Length-prefixed, bounds-checked against the packet, terminator stripped.
            // Truncated here means the name length is a lie, and reading on would
            // consume it as the leading bytes of the GUID below.
            if (!packet.readSizedString(data.senderName)) return false;
            data.receiverGuid = packet.readUInt64();
            break;
        }

        case ChatType::SAY:
        case ChatType::PARTY:
        case ChatType::YELL:
        {
            // senderGuid(u64) + senderGuid(u64) - written twice by server
            data.senderGuid = packet.readUInt64();
            /*duplicateGuid*/ packet.readUInt64();
            break;
        }

        case ChatType::WHISPER:
        case ChatType::WHISPER_INFORM:
        case ChatType::GUILD:
        case ChatType::OFFICER:
        case ChatType::RAID:
        case ChatType::RAID_LEADER:
        case ChatType::RAID_WARNING:
        case ChatType::EMOTE:
        case ChatType::TEXT_EMOTE: {
            // Classic's default chat body carries one sender GUID.
            data.senderGuid = packet.readUInt64();
            break;
        }

        case ChatType::MONSTER_SAY:
        case ChatType::MONSTER_YELL: {
            // senderGuid(u64) + nameLen(u32) + name + targetGuid(u64)
            data.senderGuid = packet.readUInt64();
            // Length-prefixed, bounds-checked against the packet, terminator stripped.
            // Truncated here means the name length is a lie, and reading on would
            // consume it as the leading bytes of the GUID below.
            if (!packet.readSizedString(data.senderName)) return false;
            data.receiverGuid = packet.readUInt64();
            break;
        }

        case ChatType::CHANNEL: {
            // channelName(string) + rank(u32) + senderGuid(u64)
            data.channelName = packet.readString();
            /*uint32_t rank =*/ packet.readUInt32();
            data.senderGuid = packet.readUInt64();
            break;
        }

        default: {
            // All other types: senderGuid(u64) + senderGuid(u64) - written twice
            data.senderGuid = packet.readUInt64();
            /*duplicateGuid*/ packet.readUInt64();
            break;
        }
    }

    // Read message length
    uint32_t messageLen = packet.readUInt32();

    // Against what is actually left, not only against a ceiling. readUInt8
    // answers zero at the end of the buffer without advancing, so a declared
    // length longer than the packet built a message padded with NULs out of
    // nothing and reported success - and a length of 8192 or more skipped the
    // message entirely and then read its first byte as the chat tag. Neither
    // overruns the buffer; both hand the interface a fabricated line.
    if (messageLen >= 8192 || messageLen > packet.getRemainingSize()) {
        LOG_WARNING("[Classic] Chat message length ", messageLen,
                    " does not fit the ", packet.getRemainingSize(),
                    " bytes left; dropping the message");
        return false;
    }

    // Read message
    if (messageLen > 0) {
        data.message.resize(messageLen);
        for (uint32_t i = 0; i < messageLen; ++i) {
            data.message[i] = static_cast<char>(packet.readUInt8());
        }
        // Remove null terminator if present
        if (!data.message.empty() && data.message.back() == '\0') {
            data.message.pop_back();
        }
    }

    // Read chat tag
    if (packet.hasData()) {
        data.chatTag = packet.readUInt8();
    }

    LOG_DEBUG("[Classic] SMSG_MESSAGECHAT: type=", getChatTypeString(data.type),
             " sender=", data.senderName.empty() ? std::to_string(data.senderGuid) : data.senderName);

    return true;
}

// ============================================================================
// Classic CMSG_JOIN_CHANNEL / CMSG_LEAVE_CHANNEL
// Classic format: just string channelName + string password (no channelId/hasVoice/joinedByZone)
// ============================================================================

network::Packet ClassicPacketParsers::buildJoinChannel(const std::string& channelName, const std::string& password) {
    network::Packet packet(wireOpcode(Opcode::CMSG_JOIN_CHANNEL));
    packet.writeString(channelName);
    packet.writeString(password);
    LOG_DEBUG("[Classic] Built CMSG_JOIN_CHANNEL: channel=", channelName);
    return packet;
}

network::Packet ClassicPacketParsers::buildLeaveChannel(const std::string& channelName) {
    network::Packet packet(wireOpcode(Opcode::CMSG_LEAVE_CHANNEL));
    packet.writeString(channelName);
    LOG_DEBUG("[Classic] Built CMSG_LEAVE_CHANNEL: channel=", channelName);
    return packet;
}

// ============================================================================
// Classic guild roster parser
// Differences from WotLK:
// - No rankCount field (fixed 10 ranks, read rights only)
// - No per-rank bank tab data
// - No gender byte per member
// ============================================================================

bool ClassicPacketParsers::parseGuildRoster(network::Packet& packet, GuildRosterData& data) {
    // Vanilla sends ten ranks and nothing but their rights, and no gender
    // byte. Its own copy of this checked no read against what was left, so a
    // truncated roster read past the end of the packet.
    return parseGuildRosterBody(packet, data, {/*rankCount=*/.rankCount=false, /*gender=*/.gender=false});
}

// ============================================================================
// Classic guild query response parser
// Differences from WotLK:
// - No trailing rankCount uint32
// ============================================================================

bool ClassicPacketParsers::parseGuildQueryResponse(network::Packet& packet, GuildQueryResponseData& data) {
    if (packet.getSize() < 8) {
        LOG_ERROR("Classic SMSG_GUILD_QUERY_RESPONSE too small: ", packet.getSize());
        return false;
    }
    data.guildId = packet.readUInt32();
    data.guildName = packet.readString();
    for (auto& rankName : data.rankNames) {
        rankName = packet.readString();
    }
    data.emblemStyle = packet.readUInt32();
    data.emblemColor = packet.readUInt32();
    data.borderStyle = packet.readUInt32();
    data.borderColor = packet.readUInt32();
    data.backgroundColor = packet.readUInt32();
    // Classic: NO trailing rankCount
    data.rankCount = 10;
    LOG_INFO("Parsed Classic SMSG_GUILD_QUERY_RESPONSE: guild=", data.guildName);
    return true;
}

// ============================================================================
// GameObject Query - Classic has no extra strings before data[]
// WotLK has iconName + castBarCaption + unk1 between names and data[].
// Vanilla: entry, type, displayId, name[4], data[24]
// ============================================================================

bool ClassicPacketParsers::parseGameObjectQueryResponse(network::Packet& packet, GameObjectQueryResponseData& data) {
    // Vanilla has no 2.0.3 block at all: the data fields follow the names.
    return parseGameObjectQueryBody(packet, data, /*extraStrings=*/0);
}

// ============================================================================
// Gossip - Classic has no menuId, and quest items lack questFlags + isRepeatable
// ============================================================================

bool ClassicPacketParsers::parseGossipMessage(network::Packet& packet, GossipMessageData& data) {
    size_t remaining = packet.getRemainingSize();
    if (remaining < 8 + 4 + 4) {
        LOG_ERROR("Classic SMSG_GOSSIP_MESSAGE too small: ", remaining, " bytes");
        return false;
    }

    data.npcGuid = packet.readUInt64();
    // Classic: NO menuId field (WotLK adds uint32 menuId here)
    data.menuId = 0;
    data.titleTextId = packet.readUInt32();
    uint32_t optionCount = packet.readUInt32();

    // Cap option count to reasonable maximum
    constexpr uint32_t kMaxGossipOptions = 256;
    if (optionCount > kMaxGossipOptions) {
        LOG_WARNING("Classic SMSG_GOSSIP_MESSAGE optionCount=", optionCount, " exceeds max ",
                    kMaxGossipOptions, ", capping");
        optionCount = kMaxGossipOptions;
    }

    data.options.clear();
    data.options.reserve(optionCount);
    for (uint32_t i = 0; i < optionCount; ++i) {
        // Sanity check: ensure minimum bytes available for option (id(4)+icon(1)+isCoded(1)+text(1))
        remaining = packet.getRemainingSize();
        if (remaining < 7) {
            LOG_WARNING("Classic gossip option ", i, " truncated (", remaining, " bytes left)");
            break;
        }

        GossipOption opt;
        opt.id = packet.readUInt32();
        opt.icon = packet.readUInt8();
        opt.isCoded = (packet.readUInt8() != 0);
        // Classic/Vanilla: NO boxMoney or boxText fields (commented out in mangoszero)
        opt.boxMoney = 0;
        opt.text = packet.readString();
        opt.boxText = "";
        data.options.push_back(opt);
    }

    // Ensure we have at least 4 bytes for questCount
    remaining = packet.getRemainingSize();
    if (remaining < 4) {
        LOG_WARNING("Classic SMSG_GOSSIP_MESSAGE truncated before questCount");
        return !data.options.empty();  // Return true if we got at least some options
    }

    uint32_t questCount = packet.readUInt32();

    // Cap quest count to reasonable maximum
    constexpr uint32_t kMaxGossipQuests = 256;
    if (questCount > kMaxGossipQuests) {
        LOG_WARNING("Classic SMSG_GOSSIP_MESSAGE questCount=", questCount, " exceeds max ",
                    kMaxGossipQuests, ", capping");
        questCount = kMaxGossipQuests;
    }

    data.quests.clear();
    data.quests.reserve(questCount);
    for (uint32_t i = 0; i < questCount; ++i) {
        // Sanity check: ensure minimum bytes available for quest (id(4)+icon(4)+level(4)+title(1))
        remaining = packet.getRemainingSize();
        if (remaining < 13) {
            LOG_WARNING("Classic gossip quest ", i, " truncated (", remaining, " bytes left)");
            break;
        }

        GossipQuestItem quest;
        quest.questId = packet.readUInt32();
        quest.questIcon = packet.readUInt32();
        quest.questLevel = static_cast<int32_t>(packet.readUInt32());
        // Classic: NO questFlags, NO isRepeatable
        quest.questFlags = 0;
        quest.isRepeatable = 0;
        quest.title = normalizeWowTextTokens(packet.readString());
        data.quests.push_back(quest);
    }

    LOG_DEBUG("Classic Gossip: ", optionCount, " options, ", questCount, " quests");
    return true;
}

// ============================================================================
// Classic CMSG_SEND_MAIL - Vanilla 1.12 format
// Differences from WotLK:
// - Single uint64 itemGuid instead of uint8 attachmentCount + item array
// - Trailing uint64 unk3 + uint8 unk4 (clients > 1.9.4)
// ============================================================================
network::Packet ClassicPacketParsers::buildSendMail(uint64_t mailboxGuid,
                                                     const std::string& recipient,
                                                     const std::string& subject,
                                                     const std::string& body,
                                                     uint64_t money, uint64_t cod,
                                                     const std::vector<uint64_t>& itemGuids) {
    network::Packet packet(wireOpcode(Opcode::CMSG_SEND_MAIL));
    packet.writeUInt64(mailboxGuid);
    packet.writeString(recipient);
    packet.writeString(subject);
    packet.writeString(body);
    packet.writeUInt32(0);       // stationery
    packet.writeUInt32(0);       // unknown
    // Vanilla supports only one item attachment (single uint64 GUID)
    uint64_t singleItemGuid = itemGuids.empty() ? 0 : itemGuids[0];
    packet.writeUInt64(singleItemGuid);
    packet.writeUInt32(money);
    packet.writeUInt32(cod);
    packet.writeUInt64(0);       // unk3 (clients > 1.9.4)
    packet.writeUInt8(0);        // unk4 (clients > 1.9.4)
    return packet;
}

// ============================================================================
// Classic SMSG_MAIL_LIST_RESULT - Vanilla 1.12 format (per vmangos)
// Key differences from WotLK:
// - uint8 count (not uint32 totalCount + uint8 shownCount)
// - No msgSize prefix per entry
// - Subject comes before item data
// - Single inline item (not attachment count + array)
// - uint8 stackCount (not uint32)
// - No enchantment array (single permanentEnchant uint32)
// ============================================================================
bool ClassicPacketParsers::parseMailList(network::Packet& packet,
                                         std::vector<MailMessage>& inbox) {
    size_t remaining = packet.getRemainingSize();
    if (remaining < 1) return false;

    uint8_t count = packet.readUInt8();
    LOG_INFO("SMSG_MAIL_LIST_RESULT (Classic): count=", static_cast<int>(count));

    inbox.clear();
    inbox.reserve(count);

    for (uint8_t i = 0; i < count; ++i) {
        remaining = packet.getRemainingSize();
        if (remaining < 5) {
            LOG_WARNING("Classic mail entry ", i, " truncated (", remaining, " bytes left)");
            break;
        }

        MailMessage msg;

        // vmangos HandleGetMailList format:
        // u32 messageId, u8 messageType, sender (guid or u32),
        // string subject, u32 itemTextId, u32 package, u32 stationery,
        // item fields (entry, enchant, randomProp, suffixFactor,
        //              u8 stackCount, u32 charges, u32 maxDur, u32 dur),
        // u32 money, u32 cod, u32 flags, float expirationTime,
        // u32 mailTemplateId (build-dependent)
        msg.messageId = packet.readUInt32();
        msg.messageType = packet.readUInt8();

        switch (msg.messageType) {
            case 0: msg.senderGuid = packet.readUInt64(); break;
            default: msg.senderEntry = packet.readUInt32(); break;
        }

        msg.subject = packet.readString();

        uint32_t itemTextId = packet.readUInt32();
        (void)itemTextId;
        packet.readUInt32(); // package (unused)
        msg.stationeryId = packet.readUInt32();

        // Single inline item (Vanilla: one item per mail)
        uint32_t itemEntry = packet.readUInt32();
        uint32_t permanentEnchant = packet.readUInt32();
        uint32_t randomPropertyId = packet.readUInt32();
        uint32_t suffixFactor = packet.readUInt32();
        uint8_t stackCount = packet.readUInt8();
        packet.readUInt32(); // charges
        uint32_t maxDurability = packet.readUInt32();
        uint32_t durability = packet.readUInt32();

        if (itemEntry != 0) {
            MailAttachment att;
            att.slot = 0;
            att.itemGuidLow = 0; // Not provided in Vanilla list
            att.itemId = itemEntry;
            att.enchantId = permanentEnchant;
            att.randomPropertyId = randomPropertyId;
            att.randomSuffix = suffixFactor;
            att.stackCount = stackCount;
            att.chargesOrDurability = durability;
            att.maxDurability = maxDurability;
            msg.attachments.push_back(att);
        }

        msg.money = packet.readUInt32();
        msg.cod = packet.readUInt32();
        msg.flags = packet.readUInt32();
        msg.expirationTime = packet.readFloat();
        msg.mailTemplateId = packet.readUInt32();

        msg.read = (msg.flags & 0x01) != 0;
        inbox.push_back(std::move(msg));
    }

    LOG_INFO("Parsed ", inbox.size(), " mail messages");
    return true;
}

// ============================================================================
// Classic CMSG_MAIL_TAKE_ITEM - Vanilla only sends mailboxGuid + mailId
// (no itemSlot - Vanilla only supports 1 item per mail)
// ============================================================================
network::Packet ClassicPacketParsers::buildMailTakeItem(uint64_t mailboxGuid,
                                                         uint32_t mailId,
                                                         uint32_t /*itemSlot*/) {
    network::Packet packet(wireOpcode(Opcode::CMSG_MAIL_TAKE_ITEM));
    packet.writeUInt64(mailboxGuid);
    packet.writeUInt32(mailId);
    return packet;
}

// ============================================================================
// Classic CMSG_MAIL_DELETE - Vanilla only sends mailboxGuid + mailId
// (no mailTemplateId field)
// ============================================================================
network::Packet ClassicPacketParsers::buildMailDelete(uint64_t mailboxGuid,
                                                       uint32_t mailId,
                                                       uint32_t /*mailTemplateId*/) {
    network::Packet packet(wireOpcode(Opcode::CMSG_MAIL_DELETE));
    packet.writeUInt64(mailboxGuid);
    packet.writeUInt32(mailId);
    return packet;
}

// ============================================================================
// Classic SMSG_ITEM_QUERY_SINGLE_RESPONSE
// Vanilla has NO SoundOverrideSubclass, NO Flags2, NO ScalingStatDistribution,
// NO ScalingStatValue, and only 2 damage types (not 5).
// ============================================================================
network::Packet ClassicPacketParsers::buildItemQuery(uint32_t entry, uint64_t guid) {
    // Vanilla CMSG_ITEM_QUERY_SINGLE: uint32 entry + uint64 guid (same as WotLK)
    network::Packet packet(wireOpcode(Opcode::CMSG_ITEM_QUERY_SINGLE));
    packet.writeUInt32(entry);
    packet.writeUInt64(guid);
    LOG_DEBUG("[Classic] Built CMSG_ITEM_QUERY_SINGLE: entry=", entry, " guid=0x", std::hex, guid, std::dec);
    return packet;
}

/// SMSG_ITEM_QUERY_SINGLE_RESPONSE as classic and TBC send it.
///
/// The two readers were 154 lines each and differed in one field, which is now
/// the parameter. WotLK is deliberately not folded in: that reader scores two
/// candidate price layouts against each other to decide which the server sent,
/// which is a different mechanism rather than a longer version of this one.
bool parseItemQueryPreWotlk(network::Packet& packet, ItemQueryResponseData& data,
                            bool hasSoundOverrideSubclass, const char* tag) {    // Validate minimum packet size: entry(4)
    if (packet.getSize() < 4) {
        LOG_ERROR(tag, " SMSG_ITEM_QUERY_SINGLE_RESPONSE: packet too small (", packet.getSize(), " bytes)");
        return false;
    }

    data.entry = packet.readUInt32();

    // High bit set means item not found
    if (data.entry & 0x80000000) {
        data.entry &= ~0x80000000;
        return true;
    }

    // itemClass(4) + subClass(4), plus the sound override where it is sent.
    if (!packet.hasRemaining(hasSoundOverrideSubclass ? 12u : 8u)) {
        LOG_ERROR(tag, " SMSG_ITEM_QUERY_SINGLE_RESPONSE: truncated before names (entry=", data.entry, ")");
        return false;
    }

    uint32_t itemClass = packet.readUInt32();
    uint32_t subClass = packet.readUInt32();
    // TBC sends a SoundOverrideSubclass here (int32, -1 = no override) and
    // vanilla does not. It is the only field the two layouts differ by;
    // everything after it is the same run in the same order.
    if (hasSoundOverrideSubclass) packet.readUInt32();

    data.itemClass = itemClass;
    data.subClass = subClass;
    data.subclassName = getItemSubclassName(itemClass, subClass);

    // 4 name strings
    data.name = packet.readString();
    packet.readString(); // name2
    packet.readString(); // name3
    packet.readString(); // name4

    data.displayInfoId = packet.readUInt32();
    data.quality = packet.readUInt32();

    // Validate minimum size for fixed fields: Flags(4) + BuyPrice(4) + SellPrice(4) + inventoryType(4)
    if (!packet.hasRemaining(16)) {
        LOG_ERROR(tag, " SMSG_ITEM_QUERY_SINGLE_RESPONSE: truncated before inventoryType (entry=", data.entry, ")");
        return false;
    }

    data.itemFlags = packet.readUInt32(); // Flags
    // Vanilla: NO Flags2
    packet.readUInt32(); // BuyPrice
    data.sellPrice = packet.readUInt32(); // SellPrice

    data.inventoryType = packet.readUInt32();

    // readCommonRequirements guards its own fourteen fields at 56 bytes, so
    // this is only an early exit with a clearer message. It said 13x4 and the
    // run had grown to fourteen; classic asked for 52 here and TBC for 56.
    if (!packet.hasRemaining(56)) {
        LOG_ERROR(tag, " SMSG_ITEM_QUERY_SINGLE_RESPONSE: truncated before stats (entry=", data.entry, ")");
        return false;
    }

    // The run every expansion sends identically, read in one place.
    if (!data.readCommonRequirements(packet)) return false;

    // Vanilla: 10 stat pairs, NO statsCount prefix (10×8 = 80 bytes)
    if (!packet.hasRemaining(80)) {
        LOG_WARNING(tag, " SMSG_ITEM_QUERY_SINGLE_RESPONSE: truncated in stats section (entry=", data.entry, ")");
        // Read what we can
    }
    for (uint32_t i = 0; i < 10; i++) {
        if (!packet.hasRemaining(8)) {
            LOG_WARNING(tag, " SMSG_ITEM_QUERY_SINGLE_RESPONSE: stat ", i, " truncated (entry=", data.entry, ")");
            break;
        }
        uint32_t statType = packet.readUInt32();
        int32_t statValue = static_cast<int32_t>(packet.readUInt32());
        data.applyStat(statType, statValue);
    }

    // Vanilla: NO ScalingStatDistribution, NO ScalingStatValue

    // Vanilla: 5 damage types (same count as WotLK)
    bool haveWeaponDamage = false;
    for (int i = 0; i < 5; i++) {
        // Each damage entry is dmgMin(4) + dmgMax(4) + damageType(4) = 12 bytes
        if (!packet.hasRemaining(12)) {
            LOG_WARNING(tag, " SMSG_ITEM_QUERY_SINGLE_RESPONSE: damage ", i, " truncated (entry=", data.entry, ")");
            break;
        }
        float dmgMin = packet.readFloat();
        float dmgMax = packet.readFloat();
        uint32_t damageType = packet.readUInt32();
        if (!haveWeaponDamage && dmgMax > 0.0f) {
            // Prefer physical damage (type 0) when present.
            if (damageType == 0 || data.damageMax <= 0.0f) {
                data.damageMin = dmgMin;
                data.damageMax = dmgMax;
                haveWeaponDamage = (damageType == 0);
            }
        }
    }

    // Validate minimum size for armor field (4 bytes)
    if (!packet.hasRemaining(4)) {
        LOG_WARNING(tag, " SMSG_ITEM_QUERY_SINGLE_RESPONSE: truncated before armor (entry=", data.entry, ")");
        return true;  // Have core fields; armor is important but optional
    }
    data.armor = static_cast<int32_t>(packet.readUInt32());

    // Remaining tail can vary by core. Read resistances + delay when present.
    if (packet.hasRemaining(28)) {
        data.holyRes   = static_cast<int32_t>(packet.readUInt32()); // HolyRes
        data.fireRes   = static_cast<int32_t>(packet.readUInt32()); // FireRes
        data.natureRes = static_cast<int32_t>(packet.readUInt32()); // NatureRes
        data.frostRes  = static_cast<int32_t>(packet.readUInt32()); // FrostRes
        data.shadowRes = static_cast<int32_t>(packet.readUInt32()); // ShadowRes
        data.arcaneRes = static_cast<int32_t>(packet.readUInt32()); // ArcaneRes
        data.delayMs = packet.readUInt32();
    }

    // AmmoType + RangedModRange (2 fields, 8 bytes)
    if (packet.hasRemaining(8)) {
        packet.readUInt32(); // AmmoType
        packet.readFloat();  // RangedModRange
    }

    // 2 item spells in Vanilla (3 fields each: SpellId, Trigger, Charges)
    // Actually vanilla has 5 spells: SpellId, Trigger, Charges, Cooldown, Category, CatCooldown = 24 bytes each
    for (int i = 0; i < 5; i++) {
        if (!packet.hasRemaining(24)) break;
        data.spells[i].spellId = packet.readUInt32();
        data.spells[i].spellTrigger = packet.readUInt32();
        packet.readUInt32(); // SpellCharges
        packet.readUInt32(); // SpellCooldown
        packet.readUInt32(); // SpellCategory
        packet.readUInt32(); // SpellCategoryCooldown
    }

    // Bonding type
    if (packet.hasRemaining(4))
        data.bindType = packet.readUInt32();

    // Description (flavor/lore text)
    if (packet.hasData())
        data.description = packet.readString();

    // Post-description: PageText, LanguageID, PageMaterial, StartQuest
    if (packet.hasRemaining(16)) {
        data.pageTextId = packet.readUInt32(); // PageText
        packet.readUInt32(); // LanguageID
        data.pageMaterial = packet.readUInt32(); // PageMaterial
        data.startQuestId = packet.readUInt32(); // StartQuest
    }

    data.valid = !data.name.empty();
    LOG_DEBUG("[Classic] Item query response: ", data.name, " (quality=", data.quality,
             " invType=", data.inventoryType, " stack=", data.maxStack, ")");
    return true;
}

bool ClassicPacketParsers::parseItemQueryResponse(network::Packet& packet,
                                                  ItemQueryResponseData& data) {
    return parseItemQueryPreWotlk(packet, data, /*hasSoundOverrideSubclass=*/false,
                                  "Classic");
}

// ============================================================================
// Turtle WoW (build 7234) parseMovementBlock
//
// Turtle WoW is a heavily modified vanilla (1.12.1) server.  Through hex dump
// analysis the wire format is nearly identical to Classic with one key addition:
//
//   LIVING section:
//     moveFlags       u32     (NO moveFlags2 - confirmed by position alignment)
//     time            u32
//     position        4×float
//     transport       guarded by moveFlags & 0x02000000 (Classic flag)
//                     packed GUID + 4 floats + u32 timestamp (TBC-style addition)
//     pitch           guarded by SWIMMING (0x200000)
//     fallTime        u32
//     jump data       guarded by JUMPING  (0x2000)
//     splineElev      guarded by 0x04000000
//     speeds          6 floats (walk/run/runBack/swim/swimBack/turnRate)
//     spline          guarded by 0x00400000 (Classic flag) OR 0x08000000 (TBC flag)
//
//   Tail (same as Classic):
//     LOWGUID  → 1×u32
//     HIGHGUID → 1×u32
//
// The ONLY confirmed difference from pure Classic is:
//   Transport data includes a u32 timestamp after the 4 transport floats
//   (Classic omits this; TBC/WotLK include it).  Without this, entities on
//   transports cause a 4-byte desync that cascades to later blocks.
// ============================================================================
namespace TurtleMoveFlags {
    constexpr uint32_t ONTRANSPORT      = 0x02000000;  // Classic transport flag
    constexpr uint32_t JUMPING          = 0x00002000;
    constexpr uint32_t SWIMMING         = 0x00200000;
    constexpr uint32_t SPLINE_ELEVATION = 0x04000000;
    constexpr uint32_t SPLINE_CLASSIC   = 0x00400000;  // Classic spline enabled
    constexpr uint32_t SPLINE_TBC       = 0x08000000;  // TBC spline enabled
}

bool TurtlePacketParsers::parseMovementBlock(network::Packet& packet, UpdateBlock& block) {
    auto rem = [&]() -> size_t { return packet.getRemainingSize(); };
    if (rem() < 1) return false;

    uint8_t updateFlags = packet.readUInt8();
    block.updateFlags = static_cast<uint16_t>(updateFlags);

    LOG_DEBUG("  [Turtle] UpdateFlags: 0x", std::hex, static_cast<int>(updateFlags), std::dec);

    const uint8_t UPDATEFLAG_TRANSPORT       = 0x02;
    const uint8_t UPDATEFLAG_MELEE_ATTACKING = 0x04;
    const uint8_t UPDATEFLAG_HIGHGUID        = 0x08;
    const uint8_t UPDATEFLAG_ALL             = 0x10;
    const uint8_t UPDATEFLAG_LIVING          = 0x20;
    const uint8_t UPDATEFLAG_HAS_POSITION    = 0x40;
    // These bits are NOT the same as the other expansions' and must not be
    // merged with them. 1.12 assigns HIGHGUID 0x8 and ALL 0x10; 2.4.3 moved
    // HIGHGUID to 0x10 and put LOWGUID at 0x8, and 0x4 means MELEE_ATTACKING
    // here and HAS_TARGET there. Three tables that look like three copies of
    // one, and reading a stationary object's position off the wrong bit is a
    // misparse of everything after it.

    if (updateFlags & UPDATEFLAG_LIVING) {
        // Minimum: moveFlags(4)+time(4)+position(16)+fallTime(4)+speeds(24) = 52 bytes
        if (rem() < 52) return false;
        size_t livingStart = packet.getReadPos();

        uint32_t moveFlags = packet.readUInt32();
        // Turtle: NO moveFlags2 (confirmed by hex dump - positions are only correct without it)
        /*uint32_t time =*/ packet.readUInt32();

        // Position
        block.x = packet.readFloat();
        block.y = packet.readFloat();
        block.z = packet.readFloat();
        block.orientation = packet.readFloat();
        block.hasMovement = true;

        LOG_DEBUG("  [Turtle] LIVING: (", block.x, ", ", block.y, ", ", block.z,
                  "), o=", block.orientation, " moveFlags=0x", std::hex, moveFlags, std::dec);

        // Transport - Classic flag position 0x02000000
        if (moveFlags & TurtleMoveFlags::ONTRANSPORT) {
            if (rem() < 1) return false; // PackedGuid mask byte
            block.onTransport = true;
            block.transportGuid = packet.readPackedGuid();
            if (rem() < 20) return false; // 4 floats + u32 timestamp
            block.transportX = packet.readFloat();
            block.transportY = packet.readFloat();
            block.transportZ = packet.readFloat();
            block.transportO = packet.readFloat();
            /*uint32_t transportTime =*/ packet.readUInt32();  // Turtle adds TBC-style timestamp
        }

        // Pitch (swimming only, Classic-style)
        if (moveFlags & TurtleMoveFlags::SWIMMING) {
            if (rem() < 4) return false;
            /*float pitch =*/ packet.readFloat();
        }

        // Fall time (always present)
        if (rem() < 4) return false;
        /*uint32_t fallTime =*/ packet.readUInt32();

        // Jump data
        if (moveFlags & TurtleMoveFlags::JUMPING) {
            if (rem() < 16) return false;
            /*float jumpVelocity =*/ packet.readFloat();
            /*float jumpSinAngle =*/ packet.readFloat();
            /*float jumpCosAngle =*/ packet.readFloat();
            /*float jumpXYSpeed =*/ packet.readFloat();
        }

        // Spline elevation
        if (moveFlags & TurtleMoveFlags::SPLINE_ELEVATION) {
            if (rem() < 4) return false;
            /*float splineElevation =*/ packet.readFloat();
        }

        // Turtle: 6 speeds (same as Classic - no flight speeds)
        if (rem() < 24) return false; // 6 × float
        float walkSpeed = packet.readFloat();
        float runSpeed = packet.readFloat();
        float runBackSpeed = packet.readFloat();
        float swimSpeed = packet.readFloat();
        float swimBackSpeed = packet.readFloat();
        float turnRate = packet.readFloat();

        block.runSpeed = runSpeed;
        block.walkSpeed = walkSpeed;
        block.runBackSpeed = runBackSpeed;
        block.swimSpeed = swimSpeed;
        block.swimBackSpeed = swimBackSpeed;
        block.turnRate = turnRate;

        LOG_DEBUG("  [Turtle] Speeds: walk=", walkSpeed, " run=", runSpeed,
                  " runBack=", runBackSpeed, " swim=", swimSpeed,
                  " swimBack=", swimBackSpeed, " turn=", turnRate);

        // Spline data - check both Classic (0x00400000) and TBC (0x08000000) flag positions
        bool hasSpline = (moveFlags & TurtleMoveFlags::SPLINE_CLASSIC) ||
                         (moveFlags & TurtleMoveFlags::SPLINE_TBC);
        if (hasSpline) {
            SplineBlockData splineData;
            if (!parseClassicMoveUpdateSpline(packet, splineData)) return false;
        }

        LOG_DEBUG("  [Turtle] LIVING block consumed ", packet.getReadPos() - livingStart,
                  " bytes, readPos now=", packet.getReadPos());
    }
    else if (updateFlags & UPDATEFLAG_HAS_POSITION) {
        if (rem() < 16) return false;
        block.x = packet.readFloat();
        block.y = packet.readFloat();
        block.z = packet.readFloat();
        block.orientation = packet.readFloat();
        block.hasMovement = true;

        LOG_DEBUG("  [Turtle] STATIONARY: (", block.x, ", ", block.y, ", ", block.z, ")");
    }

    // High GUID - 1×u32
    if (updateFlags & UPDATEFLAG_HIGHGUID) {
        if (rem() < 4) return false;
        /*uint32_t highGuid =*/ packet.readUInt32();
    }

    if (updateFlags & UPDATEFLAG_ALL) {
        if (rem() < 4) return false;
        /*uint32_t unkAll =*/ packet.readUInt32();
    }

    if (updateFlags & UPDATEFLAG_MELEE_ATTACKING) {
        if (rem() < 1) return false;
        /*uint64_t meleeTargetGuid =*/ packet.readPackedGuid();
    }

    if (updateFlags & UPDATEFLAG_TRANSPORT) {
        if (rem() < 4) return false;
        /*uint32_t transportTime =*/ packet.readUInt32();
    }

    return true;
}

bool TurtlePacketParsers::parseUpdateObject(network::Packet& packet, UpdateObjectData& data) {
    constexpr uint32_t kMaxReasonableUpdateBlocks = 4096;

    auto parseWithLayout = [&](bool withHasTransportByte, UpdateObjectData& out) -> bool {
        out = UpdateObjectData{};
        const size_t start = packet.getReadPos();
        if (packet.getSize() - start < 4) return false;

        out.blockCount = packet.readUInt32();
        if (out.blockCount > kMaxReasonableUpdateBlocks) {
            packet.setReadPos(start);
            return false;
        }

        if (withHasTransportByte) {
            if (!packet.hasData()) {
                packet.setReadPos(start);
                return false;
            }
            /*uint8_t hasTransport =*/ packet.readUInt8();
        }

        uint32_t remainingBlockCount = out.blockCount;

        if (packet.hasRemaining(1)) {
            uint8_t firstByte = packet.readUInt8();
            if (firstByte == static_cast<uint8_t>(UpdateType::OUT_OF_RANGE_OBJECTS)) {
                if (remainingBlockCount == 0) {
                    packet.setReadPos(start);
                    return false;
                }
                --remainingBlockCount;
                if (!packet.hasRemaining(4)) {
                    packet.setReadPos(start);
                    return false;
                }
                uint32_t count = packet.readUInt32();
                if (count > kMaxReasonableUpdateBlocks) {
                    packet.setReadPos(start);
                    return false;
                }
                for (uint32_t i = 0; i < count; ++i) {
                    if (!packet.hasData()) {
                        packet.setReadPos(start);
                        return false;
                    }
                    out.outOfRangeGuids.push_back(packet.readPackedGuid());
                }
            } else {
                packet.setReadPos(packet.getReadPos() - 1);
            }
        }

        out.blockCount = remainingBlockCount;
        out.blocks.reserve(out.blockCount);
        for (uint32_t i = 0; i < out.blockCount; ++i) {
            if (!packet.hasData()) {
                // If we already parsed some blocks, keep them (layout is confirmed valid).
                if (!out.blocks.empty()) break;
                packet.setReadPos(start);
                return false;
            }

            const size_t blockStart = packet.getReadPos();
            uint8_t updateTypeVal = packet.readUInt8();
            if (updateTypeVal > static_cast<uint8_t>(UpdateType::NEAR_OBJECTS)) {
                if (!out.blocks.empty()) break;
                packet.setReadPos(start);
                return false;
            }

            const UpdateType updateType = static_cast<UpdateType>(updateTypeVal);
            UpdateBlock block;
            block.updateType = updateType;
            bool ok = false;

            auto parseMovementVariant = [&](auto&& movementParser, const char* layoutName) -> bool {
                packet.setReadPos(blockStart + 1);
                block = UpdateBlock{};
                block.updateType = updateType;

                switch (updateType) {
                    case UpdateType::MOVEMENT:
                        block.guid = packet.readUInt64();
                        if (!movementParser(packet, block)) return false;
                        LOG_DEBUG("[Turtle] Parsed MOVEMENT block via ", layoutName, " layout");
                        return true;
                    case UpdateType::CREATE_OBJECT:
                    case UpdateType::CREATE_OBJECT2:
                        block.guid = packet.readPackedGuid();
                        if (!packet.hasData()) return false;
                        block.objectType = static_cast<ObjectType>(packet.readUInt8());
                        if (!movementParser(packet, block)) return false;
                        if (!UpdateObjectParser::parseUpdateFields(packet, block)) return false;
                        LOG_DEBUG("[Turtle] Parsed CREATE block via ", layoutName, " layout");
                        return true;
                    default:
                        return false;
                }
            };

            switch (updateType) {
                case UpdateType::VALUES:
                    block.guid = packet.readPackedGuid();
                    ok = UpdateObjectParser::parseUpdateFields(packet, block);
                    break;
                case UpdateType::MOVEMENT:
                case UpdateType::CREATE_OBJECT:
                case UpdateType::CREATE_OBJECT2:
                    ok = parseMovementVariant(
                        [this](network::Packet& p, UpdateBlock& b) {
                            return this->TurtlePacketParsers::parseMovementBlock(p, b);
                        }, "turtle");
                    if (!ok) {
                        ok = parseMovementVariant(
                            [this](network::Packet& p, UpdateBlock& b) {
                                return this->ClassicPacketParsers::parseMovementBlock(p, b);
                            }, "classic");
                    }
                    if (!ok) {
                        ok = parseMovementVariant(
                            [this](network::Packet& p, UpdateBlock& b) {
                                return this->TbcPacketParsers::parseMovementBlock(p, b);
                            }, "tbc");
                    }
                    // NOTE: Do NOT fall back to WotLK parseMovementBlock here.
                    // WotLK uses uint16 updateFlags and 9 speeds vs Classic's uint8
                    // and 6 speeds. A false-positive WotLK parse consumes wrong bytes,
                    // corrupting subsequent update fields and losing NPC data.
                    break;
                case UpdateType::OUT_OF_RANGE_OBJECTS:
                case UpdateType::NEAR_OBJECTS:
                    ok = true;
                    break;
                default:
                    ok = false;
                    break;
            }

            if (!ok) {
                static int turtleBlockErrors = 0;
                if (++turtleBlockErrors <= 5) {
                    LOG_WARNING("[Turtle] SMSG_UPDATE_OBJECT block parse failed",
                                " blockIndex=", i, " of ", out.blockCount,
                                " updateType=", updateTypeName(updateType),
                                " readPos=", packet.getReadPos(),
                                " blockStart=", blockStart,
                                " packetSize=", packet.getSize(),
                                " (", out.blocks.size(), " blocks kept)");
                }
                // Keep successfully parsed blocks instead of discarding all.
                // Cannot re-sync within the packet, so stop parsing here.
                break;
            }

            out.blocks.push_back(std::move(block));
        }

        return true;
    };

    const size_t startPos = packet.getReadPos();
    UpdateObjectData parsed;
    if (parseWithLayout(true, parsed)) {
        data = std::move(parsed);
        return true;
    }

    packet.setReadPos(startPos);
    if (parseWithLayout(false, parsed)) {
        LOG_DEBUG("[Turtle] SMSG_UPDATE_OBJECT parsed without has_transport byte fallback");
        data = std::move(parsed);
        return true;
    }

    packet.setReadPos(startPos);
    if (ClassicPacketParsers::parseUpdateObject(packet, parsed)) {
        LOG_DEBUG("[Turtle] SMSG_UPDATE_OBJECT parsed via full classic fallback");
        data = std::move(parsed);
        return true;
    }

    packet.setReadPos(startPos);
    if (TbcPacketParsers::parseUpdateObject(packet, parsed)) {
        LOG_DEBUG("[Turtle] SMSG_UPDATE_OBJECT parsed via full TBC fallback");
        data = std::move(parsed);
        return true;
    }

    packet.setReadPos(startPos);
    return false;
}

bool TurtlePacketParsers::parseMonsterMove(network::Packet& packet, MonsterMoveData& data) {
    // Turtle realms can emit vanilla-like, TBC-like, and WotLK-like monster move
    // bodies. Try the lower-expansion layouts first before the WotLK parser that
    // expects an extra unk byte after the packed GUID.
    size_t start = packet.getReadPos();
    if (MonsterMoveParser::parseVanilla(packet, data)) {
        return true;
    }

    packet.setReadPos(start);
    if (TbcPacketParsers::parseMonsterMove(packet, data)) {
        LOG_DEBUG("[Turtle] SMSG_MONSTER_MOVE parsed via TBC fallback layout");
        return true;
    }

    auto looksLikeWotlkMonsterMove = [&](network::Packet& probe) -> bool {
        const size_t probeStart = probe.getReadPos();
        uint64_t guid = probe.readPackedGuid();
        if (guid == 0) {
            probe.setReadPos(probeStart);
            return false;
        }
        if (probe.getReadPos() >= probe.getSize()) {
            probe.setReadPos(probeStart);
            return false;
        }
        uint8_t unk = probe.readUInt8();
        if (unk > 1) {
            probe.setReadPos(probeStart);
            return false;
        }
        if (probe.getReadPos() + 12 + 4 + 1 > probe.getSize()) {
            probe.setReadPos(probeStart);
            return false;
        }
        probe.readFloat(); probe.readFloat(); probe.readFloat(); // xyz
        probe.readUInt32(); // splineId
        uint8_t moveType = probe.readUInt8();
        probe.setReadPos(probeStart);
        return moveType >= 1 && moveType <= 4;
    };

    packet.setReadPos(start);
    if (!looksLikeWotlkMonsterMove(packet)) {
        packet.setReadPos(start);
        return false;
    }

    packet.setReadPos(start);
    if (MonsterMoveParser::parse(packet, data)) {
        LOG_DEBUG("[Turtle] SMSG_MONSTER_MOVE parsed via WotLK fallback layout");
        return true;
    }

    packet.setReadPos(start);
    return false;
}

// ============================================================================
// Classic/Vanilla quest giver status
//
// Vanilla sends status as uint32 with different enum values:
//   0=NONE, 1=UNAVAILABLE, 2=CHAT, 3=INCOMPLETE, 4=REWARD_REP, 5=AVAILABLE
// WotLK uses uint8 with:
//   0=NONE, 1=UNAVAILABLE, 2=LOW_LEVEL_AVAILABLE, 3=LOW_LEVEL_REWARD_REP,
//   4=LOW_LEVEL_AVAILABLE_REP, 5=INCOMPLETE, 6=REWARD_REP, 7=AVAILABLE_REP,
//   8=AVAILABLE, 9=REWARD2, 10=REWARD
//
// Read uint32, translate to WotLK enum values.
// ============================================================================
uint8_t ClassicPacketParsers::readQuestGiverStatus(network::Packet& packet) {
    uint32_t vanillaStatus = packet.readUInt32();
    switch (vanillaStatus) {
        case 0: return 0;  // NONE
        case 1: return 1;  // UNAVAILABLE
        case 2: return 0;  // CHAT → NONE (no marker)
        case 3: return 5;  // INCOMPLETE → WotLK INCOMPLETE
        case 4: return 6;  // REWARD_REP → WotLK REWARD_REP
        case 5: return 8;  // AVAILABLE → WotLK AVAILABLE
        case 6: return 10; // REWARD → WotLK REWARD
        default: return 0;
    }
}

// ============================================================================
// Classic CMSG_QUESTGIVER_QUERY_QUEST - Vanilla 1.12 format
// WotLK appends a trailing unk1 byte; Vanilla servers don't expect it and
// some reject or misparse the 13-byte packet, preventing quest details from
// being sent back.  Classic format: guid(8) + questId(4) = 12 bytes.
// ============================================================================
network::Packet buildQueryQuestPacketPreWotlk(uint64_t npcGuid, uint32_t questId) {
    network::Packet packet(wireOpcode(Opcode::CMSG_QUESTGIVER_QUERY_QUEST));
    packet.writeUInt64(npcGuid);
    packet.writeUInt32(questId);
    // No trailing isDialogContinued byte; WotLK is the only version with one.
    return packet;
}

network::Packet buildAcceptQuestPacketPreWotlk(uint64_t npcGuid, uint32_t questId) {
    network::Packet packet(wireOpcode(Opcode::CMSG_QUESTGIVER_ACCEPT_QUEST));
    packet.writeUInt64(npcGuid);
    packet.writeUInt32(questId);
    // No trailing unk1 uint32.
    return packet;
}

network::Packet ClassicPacketParsers::buildQueryQuestPacket(uint64_t npcGuid, uint32_t questId) {
    return buildQueryQuestPacketPreWotlk(npcGuid, questId);
}

network::Packet ClassicPacketParsers::buildAcceptQuestPacket(uint64_t npcGuid, uint32_t questId) {
    return buildAcceptQuestPacketPreWotlk(npcGuid, questId);
}

// ============================================================================
// ClassicPacketParsers::parseCreatureQueryResponse
//
// Classic 1.12 SMSG_CREATURE_QUERY_RESPONSE lacks the iconName CString field
// that TBC 2.4.3 and WotLK 3.3.5a include between subName and typeFlags.
// Without this override, the TBC/WotLK parser reads typeFlags bytes as the
// iconName string, shifting typeFlags/creatureType/family/rank by 1-4 bytes.
// ============================================================================
bool ClassicPacketParsers::parseCreatureQueryResponse(network::Packet& packet,
                                                       CreatureQueryResponseData& data) {
    // Validate minimum packet size: entry(4)
    if (packet.getSize() < 4) {
        LOG_ERROR("Classic SMSG_CREATURE_QUERY_RESPONSE: packet too small (", packet.getSize(), " bytes)");
        return false;
    }

    data.entry = packet.readUInt32();
    if (data.entry & 0x80000000) {
        data.entry &= ~0x80000000;
        data.name = "";
        return true;
    }

    data.name = packet.readString();
    packet.readString(); // name2
    packet.readString(); // name3
    packet.readString(); // name4
    data.subName = packet.readString();
    // NOTE: NO iconName field in Classic 1.12 - goes straight to typeFlags
    if (!packet.hasRemaining(16)) {
        LOG_WARNING("Classic SMSG_CREATURE_QUERY_RESPONSE: truncated at typeFlags (entry=", data.entry, ")");
        data.typeFlags = 0;
        data.creatureType = 0;
        data.family = 0;
        data.rank = 0;
        return true;  // Have name/sub fields; base fields are important but optional
    }
    data.typeFlags    = packet.readUInt32();
    data.creatureType = packet.readUInt32();
    data.family       = packet.readUInt32();
    data.rank         = packet.readUInt32();

    LOG_DEBUG("Classic SMSG_CREATURE_QUERY_RESPONSE: ", data.name, " type=", data.creatureType,
              " rank=", data.rank);
    return true;
}

} // namespace game
} // namespace wowee
