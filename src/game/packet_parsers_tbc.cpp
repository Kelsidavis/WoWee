#include "game/packet_parsers.hpp"
#include "game/spline_packet.hpp"
#include "core/logger.hpp"

namespace wowee {
namespace game {

// ============================================================================
// TBC 2.4.3 movement flag constants (shifted relative to WotLK 3.3.5a)
// ============================================================================
namespace TbcMoveFlags {
    constexpr uint32_t ON_TRANSPORT     = 0x00000200;  // Gates transport data (same as WotLK)
    constexpr uint32_t JUMPING          = 0x00002000;  // Gates jump data (WotLK: FALLING=0x1000)
    constexpr uint32_t SWIMMING         = 0x00200000;  // Same as WotLK
    constexpr uint32_t ONTRANSPORT      = 0x02000000;  // Secondary pitch check
    constexpr uint32_t SPLINE_ELEVATION = 0x04000000;  // Same as WotLK
    constexpr uint32_t SPLINE_ENABLED   = 0x08000000;  // Same as WotLK
}

// ============================================================================
// TBC parseMovementBlock
// Key differences from WotLK:
// - UpdateFlags is uint8 (not uint16)
// - No VEHICLE (0x0080), POSITION (0x0100), ROTATION (0x0200) flags
// - moveFlags2 is uint8 (not uint16)
// - No transport seat byte
// - No interpolated movement (flags2 & 0x0200) check
// - Pitch check: SWIMMING, else ONTRANSPORT(0x02000000)
// - Spline data: has splineId, no durationMod/durationModNext/verticalAccel/effectStartTime/splineMode
// - Trailer order matches CMaNGOS TBC BuildMovementUpdate:
//   LOWGUID, HIGHGUID, attacking target, transport time
// ============================================================================
bool TbcPacketParsers::parseMovementBlock(network::Packet& packet, UpdateBlock& block) {
    auto rem = [&]() -> size_t { return packet.getRemainingSize(); };
    if (rem() < 1) return false;

    // TBC 2.4.3: UpdateFlags is uint8 (1 byte)
    uint8_t updateFlags = packet.readUInt8();
    block.updateFlags = static_cast<uint16_t>(updateFlags);

    LOG_DEBUG("  [TBC] UpdateFlags: 0x", std::hex, static_cast<int>(updateFlags), std::dec);

    // TBC UpdateFlag bit values (same as lower byte of WotLK):
    // 0x01 = SELF
    // 0x02 = TRANSPORT
    // 0x04 = HAS_TARGET
    // 0x08 = LOWGUID
    // 0x10 = HIGHGUID
    // 0x20 = LIVING
    // 0x40 = HAS_POSITION (stationary)
    const uint8_t UPDATEFLAG_LIVING              = 0x20;
    const uint8_t UPDATEFLAG_HAS_POSITION        = 0x40;
    // These bits are NOT the same as the other expansions' and must not be
    // merged with them. 1.12 assigns HIGHGUID 0x8 and ALL 0x10; 2.4.3 moved
    // HIGHGUID to 0x10 and put LOWGUID at 0x8, and 0x4 means MELEE_ATTACKING
    // here and HAS_TARGET there. Three tables that look like three copies of
    // one, and reading a stationary object's position off the wrong bit is a
    // misparse of everything after it.
    const uint8_t UPDATEFLAG_HAS_TARGET          = 0x04;
    const uint8_t UPDATEFLAG_TRANSPORT           = 0x02;
    const uint8_t UPDATEFLAG_LOWGUID             = 0x08;
    const uint8_t UPDATEFLAG_HIGHGUID            = 0x10;

    if (updateFlags & UPDATEFLAG_LIVING) {
        // Minimum: moveFlags(4)+moveFlags2(1)+time(4)+position(16)+fallTime(4)+speeds(32) = 61
        if (rem() < 61) return false;

        // Full movement block for living units
        uint32_t moveFlags = packet.readUInt32();
        uint8_t moveFlags2 = packet.readUInt8();  // TBC: uint8, not uint16
        (void)moveFlags2;
        /*uint32_t time =*/ packet.readUInt32();

        // Position
        block.x = packet.readFloat();
        block.y = packet.readFloat();
        block.z = packet.readFloat();
        block.orientation = packet.readFloat();
        block.hasMovement = true;

        LOG_DEBUG("  [TBC] LIVING: (", block.x, ", ", block.y, ", ", block.z,
                  "), o=", block.orientation, " moveFlags=0x", std::hex, moveFlags, std::dec);

        // Transport data
        if (moveFlags & TbcMoveFlags::ON_TRANSPORT) {
            if (rem() < 1) return false;
            block.onTransport = true;
            block.transportGuid = packet.readPackedGuid();
            if (rem() < 20) return false; // 4 floats + 1 uint32
            block.transportX = packet.readFloat();
            block.transportY = packet.readFloat();
            block.transportZ = packet.readFloat();
            block.transportO = packet.readFloat();
            /*uint32_t tTime =*/ packet.readUInt32();
        }

        // Pitch: SWIMMING, or else ONTRANSPORT (TBC-specific secondary pitch)
        if (moveFlags & TbcMoveFlags::SWIMMING) {
            if (rem() < 4) return false;
            /*float pitch =*/ packet.readFloat();
        } else if (moveFlags & TbcMoveFlags::ONTRANSPORT) {
            if (rem() < 4) return false;
            /*float pitch =*/ packet.readFloat();
        }

        // Fall time (always present)
        if (rem() < 4) return false;
        /*uint32_t fallTime =*/ packet.readUInt32();

        // Jumping (TBC: JUMPING=0x2000, WotLK: FALLING=0x1000)
        if (moveFlags & TbcMoveFlags::JUMPING) {
            if (rem() < 16) return false;
            /*float jumpVelocity =*/ packet.readFloat();
            /*float jumpSinAngle =*/ packet.readFloat();
            /*float jumpCosAngle =*/ packet.readFloat();
            /*float jumpXYSpeed =*/ packet.readFloat();
        }

        // Spline elevation (same bit as WotLK)
        if (moveFlags & TbcMoveFlags::SPLINE_ELEVATION) {
            if (rem() < 4) return false;
            /*float splineElevation =*/ packet.readFloat();
        }

        // Speeds (TBC: 8 values - walk, run, runBack, swim, swimBack, fly, flyBack, turn)
        if (rem() < 32) return false;
        block.walkSpeed       = packet.readFloat();
        block.runSpeed        = packet.readFloat();
        block.runBackSpeed    = packet.readFloat();
        block.swimSpeed       = packet.readFloat();
        block.swimBackSpeed   = packet.readFloat();
        block.flightSpeed     = packet.readFloat();
        block.flightBackSpeed = packet.readFloat();
        block.turnRate        = packet.readFloat();
        block.moveFlags = moveFlags;

        // Spline data (TBC/WotLK: SPLINE_ENABLED = 0x08000000)
        if (moveFlags & TbcMoveFlags::SPLINE_ENABLED) {
            if (rem() < 4) return false;
            uint32_t splineFlags = packet.readUInt32();
            LOG_DEBUG("  [TBC] Spline: flags=0x", std::hex, splineFlags, std::dec);

            if (splineFlags & 0x00010000) { // FINAL_POINT
                if (rem() < 12) return false;
                /*float finalX =*/ packet.readFloat();
                /*float finalY =*/ packet.readFloat();
                /*float finalZ =*/ packet.readFloat();
            } else if (splineFlags & 0x00020000) { // FINAL_TARGET
                if (rem() < 8) return false;
                /*uint64_t finalTarget =*/ packet.readUInt64();
            } else if (splineFlags & 0x00040000) { // FINAL_ANGLE
                if (rem() < 4) return false;
                /*float finalAngle =*/ packet.readFloat();
            }

            // TBC spline: timePassed, duration, id, pointCount
            if (rem() < 16) return false;
            /*uint32_t timePassed =*/ packet.readUInt32();
            /*uint32_t duration =*/ packet.readUInt32();
            /*uint32_t splineId =*/ packet.readUInt32();

            uint32_t pointCount = packet.readUInt32();
            // Cap waypoints to prevent DoS from malformed packets allocating huge arrays
            if (pointCount > 256) return false;

            // points + endPoint (no splineMode in TBC)
            if (rem() < static_cast<size_t>(pointCount) * 12 + 12) return false;
            for (uint32_t i = 0; i < pointCount; i++) {
                /*float px =*/ packet.readFloat();
                /*float py =*/ packet.readFloat();
                /*float pz =*/ packet.readFloat();
            }

            /*float endPointX =*/ packet.readFloat();
            /*float endPointY =*/ packet.readFloat();
            /*float endPointZ =*/ packet.readFloat();
        }
    }
    else if (updateFlags & UPDATEFLAG_HAS_POSITION) {
        if (rem() < 16) return false;
        block.x = packet.readFloat();
        block.y = packet.readFloat();
        block.z = packet.readFloat();
        block.orientation = packet.readFloat();
        block.hasMovement = true;

        LOG_DEBUG("  [TBC] STATIONARY: (", block.x, ", ", block.y, ", ", block.z, ")");
    }

    // LOWGUID (0x08) - CMaNGOS TBC writes one uint32.
    if (updateFlags & UPDATEFLAG_LOWGUID) {
        if (rem() < 4) return false;
        /*uint32_t lowGuidOrTypeMarker =*/ packet.readUInt32();
    }

    // HIGHGUID (0x10)
    if (updateFlags & UPDATEFLAG_HIGHGUID) {
        if (rem() < 4) return false;
        /*uint32_t highGuidOrZero =*/ packet.readUInt32();
    }

    // Attacking target (0x04)
    if (updateFlags & UPDATEFLAG_HAS_TARGET) {
        if (rem() < 1) return false;
        /*uint64_t targetGuid =*/ packet.readPackedGuid();
    }

    // Transport time/path progress (0x02)
    if (updateFlags & UPDATEFLAG_TRANSPORT) {
        if (rem() < 4) return false;
        /*uint32_t transportTime =*/ packet.readUInt32();
    }

    return true;
}

// ============================================================================
// TBC writeMovementPayload
// Key differences from WotLK:
// - flags2 is uint8 (not uint16)
// - No transport seat byte
// - No interpolated movement (flags2 & 0x0200) write
// - Pitch check uses TBC flag positions
// ============================================================================
void TbcPacketParsers::writeMovementPayload(network::Packet& packet, const MovementInfo& info) {
    // Movement flags (uint32, same as WotLK)
    packet.writeUInt32(info.flags);

    // TBC: flags2 is uint8 (WotLK: uint16)
    packet.writeUInt8(static_cast<uint8_t>(info.flags2 & 0xFF));

    // Timestamp
    packet.writeUInt32(info.time);

    // Position
    packet.writeFloat(info.x);
    packet.writeFloat(info.y);
    packet.writeFloat(info.z);
    packet.writeFloat(info.orientation);

    // Transport data (TBC ON_TRANSPORT = 0x200, same bit as WotLK)
    if (info.flags & TbcMoveFlags::ON_TRANSPORT) {
        // Packed transport GUID
        packet.writePackedGuid(info.transportGuid);

        // Transport local position
        packet.writeFloat(info.transportX);
        packet.writeFloat(info.transportY);
        packet.writeFloat(info.transportZ);
        packet.writeFloat(info.transportO);

        // Transport time
        packet.writeUInt32(info.transportTime);

        // TBC: NO transport seat byte
        // TBC: NO interpolated movement time
    }

    // Pitch: SWIMMING or else ONTRANSPORT (TBC flag positions)
    if (info.flags & TbcMoveFlags::SWIMMING) {
        packet.writeFloat(info.pitch);
    } else if (info.flags & TbcMoveFlags::ONTRANSPORT) {
        packet.writeFloat(info.pitch);
    }

    // Fall time (always present)
    packet.writeUInt32(info.fallTime);

    // Jump data (TBC JUMPING = 0x2000, WotLK FALLING = 0x1000)
    if (info.flags & TbcMoveFlags::JUMPING) {
        packet.writeFloat(info.jumpVelocity);
        packet.writeFloat(info.jumpSinAngle);
        packet.writeFloat(info.jumpCosAngle);
        packet.writeFloat(info.jumpXYSpeed);
    }
}

// ============================================================================
// TBC buildMovementPacket
// Classic/TBC: client movement packets do NOT include PackedGuid prefix
// (WotLK added PackedGuid to client packets)
// ============================================================================
network::Packet TbcPacketParsers::buildMovementPacket(LogicalOpcode opcode,
                                                       const MovementInfo& info,
                                                       uint64_t /*playerGuid*/) {
    network::Packet packet(wireOpcode(opcode));

    // TBC: NO PackedGuid prefix for client packets
    writeMovementPayload(packet, info);

    return packet;
}

// ============================================================================
// TBC parseCharEnum
// Differences from WotLK:
// - After flags: uint8 firstLogin (not uint32 customization + uint8 unknown)
// - Equipment: 20 items (not 23)
// ============================================================================
bool TbcPacketParsers::parseCharEnum(network::Packet& packet, CharEnumResponse& response) {
    // TBC equipment entries carry an enchantment; the rest is shared with
    // Classic.
    return parseCharEnumPreWotlk(packet, response, /*hasEnchantment=*/true, "[TBC]");
}

// ============================================================================
// TBC parseUpdateObject
// Key difference from WotLK: u8 has_transport byte after blockCount
// (WotLK removed this field)
// ============================================================================
bool TbcPacketParsers::parseUpdateObject(network::Packet& packet, UpdateObjectData& data) {
    constexpr uint32_t kMaxReasonableUpdateBlocks = 4096;
    auto parseWithLayout = [&](bool withHasTransportByte, UpdateObjectData& out) -> bool {
        out = UpdateObjectData{};
        size_t start = packet.getReadPos();
        if (packet.getSize() - start < 4) return false;

        out.blockCount = packet.readUInt32();
        if (out.blockCount > kMaxReasonableUpdateBlocks) {
            packet.setReadPos(start);
            return false;
        }

        if (withHasTransportByte) {
            if (packet.getReadPos() >= packet.getSize()) {
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
                    if (packet.getReadPos() >= packet.getSize()) {
                        packet.setReadPos(start);
                        return false;
                    }
                    uint64_t guid = packet.readPackedGuid();
                    out.outOfRangeGuids.push_back(guid);
                }
            } else {
                packet.setReadPos(packet.getReadPos() - 1);
            }
        }

        out.blockCount = remainingBlockCount;
        out.blocks.reserve(out.blockCount);
        for (uint32_t i = 0; i < out.blockCount; ++i) {
            if (packet.getReadPos() >= packet.getSize()) {
                packet.setReadPos(start);
                return false;
            }

            UpdateBlock block;
            uint8_t updateTypeVal = packet.readUInt8();
            if (updateTypeVal > static_cast<uint8_t>(UpdateType::NEAR_OBJECTS)) {
                if (!out.blocks.empty()) break;
                packet.setReadPos(start);
                return false;
            }
            block.updateType = static_cast<UpdateType>(updateTypeVal);

            bool ok = false;
            switch (block.updateType) {
                case UpdateType::VALUES: {
                    block.guid = packet.readPackedGuid();
                    ok = UpdateObjectParser::parseUpdateFields(packet, block);
                    break;
                }
                case UpdateType::MOVEMENT: {
                    block.guid = packet.readUInt64();
                    ok = this->parseMovementBlock(packet, block);
                    break;
                }
                case UpdateType::CREATE_OBJECT:
                case UpdateType::CREATE_OBJECT2: {
                    block.guid = packet.readPackedGuid();
                    if (packet.getReadPos() >= packet.getSize()) {
                        ok = false;
                        break;
                    }
                    uint8_t objectTypeVal = packet.readUInt8();
                    block.objectType = static_cast<ObjectType>(objectTypeVal);
                    ok = this->parseMovementBlock(packet, block);
                    if (ok) ok = UpdateObjectParser::parseUpdateFields(packet, block);
                    break;
                }
                case UpdateType::OUT_OF_RANGE_OBJECTS:
                case UpdateType::NEAR_OBJECTS:
                    ok = true;
                    break;
                default:
                    ok = false;
                    break;
            }

            if (!ok) {
                static int tbcBlockErrors = 0;
                if (++tbcBlockErrors <= 5) {
                    LOG_WARNING("[TBC] SMSG_UPDATE_OBJECT block parse failed",
                                " blockIndex=", i, " of ", out.blockCount,
                                " updateType=", static_cast<int>(block.updateType),
                                " readPos=", packet.getReadPos(),
                                " packetSize=", packet.getSize(),
                                " (", out.blocks.size(), " blocks kept)");
                }
                if (out.blocks.empty()) {
                    packet.setReadPos(start);
                    return false;
                }
                break;
            }
            out.blocks.push_back(block);
        }
        return true;
    };

    size_t startPos = packet.getReadPos();
    UpdateObjectData parsed;
    if (parseWithLayout(true, parsed)) {
        data = std::move(parsed);
        return true;
    }

    packet.setReadPos(startPos);
    if (parseWithLayout(false, parsed)) {
        LOG_DEBUG("[TBC] SMSG_UPDATE_OBJECT parsed without has_transport byte fallback");
        data = std::move(parsed);
        return true;
    }

    packet.setReadPos(startPos);
    return false;
}

// ============================================================================
// TBC 2.4.3 SMSG_GOSSIP_MESSAGE
// Identical to WotLK except each quest entry lacks questFlags(u32) and
// isRepeatable(u8) that WotLK added. Without this override the WotLK parser
// reads those 5 bytes as part of the quest title, corrupting all gossip quests.
// ============================================================================
bool TbcPacketParsers::parseGossipMessage(network::Packet& packet, GossipMessageData& data) {
    if (!packet.hasRemaining(16)) return false;

    data.npcGuid = packet.readUInt64();
    data.menuId = packet.readUInt32();      // TBC added menuId (Classic doesn't have it)
    data.titleTextId = packet.readUInt32();
    uint32_t optionCount = packet.readUInt32();

    // Cap option count to reasonable maximum
    constexpr uint32_t kMaxGossipOptions = 256;
    if (optionCount > kMaxGossipOptions) {
        LOG_WARNING("[TBC] SMSG_GOSSIP_MESSAGE optionCount=", optionCount, " exceeds max ",
                    kMaxGossipOptions, ", capping");
        optionCount = kMaxGossipOptions;
    }

    data.options.clear();
    data.options.reserve(optionCount);
    for (uint32_t i = 0; i < optionCount; ++i) {
        // Sanity check: ensure minimum bytes available for option
        // (id(4)+icon(1)+isCoded(1)+boxMoney(4)+text(1)+boxText(1))
        size_t remaining = packet.getRemainingSize();
        if (remaining < 12) {
            LOG_WARNING("[TBC] gossip option ", i, " truncated (", remaining, " bytes left)");
            break;
        }

        GossipOption opt;
        opt.id = packet.readUInt32();
        opt.icon = packet.readUInt8();
        opt.isCoded = (packet.readUInt8() != 0);
        opt.boxMoney = packet.readUInt32();
        opt.text = packet.readString();
        opt.boxText = packet.readString();
        data.options.push_back(opt);
    }

    // Ensure we have at least 4 bytes for questCount
    size_t remaining = packet.getRemainingSize();
    if (remaining < 4) {
        LOG_WARNING("[TBC] SMSG_GOSSIP_MESSAGE truncated before questCount");
        return !data.options.empty();  // Return true if we got at least some options
    }

    uint32_t questCount = packet.readUInt32();

    // Cap quest count to reasonable maximum
    constexpr uint32_t kMaxGossipQuests = 256;
    if (questCount > kMaxGossipQuests) {
        LOG_WARNING("[TBC] SMSG_GOSSIP_MESSAGE questCount=", questCount, " exceeds max ",
                    kMaxGossipQuests, ", capping");
        questCount = kMaxGossipQuests;
    }

    data.quests.clear();
    data.quests.reserve(questCount);
    for (uint32_t i = 0; i < questCount; ++i) {
        // Sanity check: ensure minimum bytes available for quest
        // (id(4)+icon(4)+level(4)+title(1))
        remaining = packet.getRemainingSize();
        if (remaining < 13) {
            LOG_WARNING("[TBC] gossip quest ", i, " truncated (", remaining, " bytes left)");
            break;
        }

        GossipQuestItem quest;
        quest.questId = packet.readUInt32();
        quest.questIcon = packet.readUInt32();
        quest.questLevel = static_cast<int32_t>(packet.readUInt32());
        // TBC 2.4.3: NO questFlags(u32) and NO isRepeatable(u8) here
        // WotLK adds these 5 bytes - reading them from TBC garbles the quest title
        quest.questFlags = 0;
        quest.isRepeatable = 0;
        quest.title = normalizeWowTextTokens(packet.readString());
        data.quests.push_back(quest);
    }

    LOG_DEBUG("[TBC] Gossip: ", optionCount, " options, ", questCount, " quests");
    return true;
}

// ============================================================================
// TBC 2.4.3 SMSG_MONSTER_MOVE
// Identical to WotLK except WotLK added a uint8 unk byte immediately after the
// packed GUID (toggles MOVEMENTFLAG2_UNK7). TBC does NOT have this byte.
// Without this override, all NPC movement positions/durations are offset by 1
// byte and parse as garbage.
// ============================================================================
bool TbcPacketParsers::parseMonsterMove(network::Packet& packet, MonsterMoveData& data) {
    data.guid = packet.readPackedGuid();
    if (data.guid == 0) return false;
    // No unk byte here in TBC 2.4.3

    if (!packet.hasRemaining(12)) return false;
    data.x = packet.readFloat();
    data.y = packet.readFloat();
    data.z = packet.readFloat();

    if (!packet.hasRemaining(4)) return false;
    packet.readUInt32(); // splineId

    if (packet.getReadPos() >= packet.getSize()) return false;
    {
        bool stopped = false;
        if (!parseMonsterMoveFacing(packet, data, stopped)) return false;
        if (stopped) return true;
    }

    if (!packet.hasRemaining(4)) return false;
    data.splineFlags = packet.readUInt32();

    // Consolidated spline body parser (TBC uses different uncompressed mask)
    {
        SplineBlockData spline;
        if (!parseMonsterMoveSplineBody(packet, spline, data.splineFlags,
                                        glm::vec3(data.x, data.y, data.z), true)) {
            return false;
        }
        data.duration = spline.duration;
        if (spline.hasDest) {
            data.destX = spline.destination.x;
            data.destY = spline.destination.y;
            data.destZ = spline.destination.z;
            data.hasDest = true;
        }
    }

    LOG_DEBUG("[TBC] MonsterMove: guid=0x", std::hex, data.guid, std::dec,
              " type=", static_cast<int>(data.moveType), " dur=", data.duration, "ms",
              " dest=(", data.destX, ",", data.destY, ",", data.destZ, ")");
    return true;
}

// ============================================================================
// TBC 2.4.3 CMSG_CAST_SPELL
// Format: spellId(u32) + castCount(u8) + SpellCastTargets
// WotLK 3.3.5a adds castFlags(u8) between spellId and targets - TBC does NOT.
// ============================================================================
network::Packet TbcPacketParsers::buildCastSpell(uint32_t spellId, uint64_t targetGuid, uint8_t castCount) {
    network::Packet packet(wireOpcode(LogicalOpcode::CMSG_CAST_SPELL));
    packet.writeUInt32(spellId);
    packet.writeUInt8(castCount);
    // No castFlags byte in TBC 2.4.3

    if (targetGuid != 0) {
        packet.writeUInt32(0x02); // TARGET_FLAG_UNIT
        // Write packed GUID
        packet.writePackedGuid(targetGuid);
    } else {
        packet.writeUInt32(0x00); // TARGET_FLAG_SELF
    }

    return packet;
}

network::Packet TbcPacketParsers::buildCastGameObjectSpell(uint32_t spellId, uint64_t targetGuid, uint8_t castCount) {
    network::Packet packet(wireOpcode(LogicalOpcode::CMSG_CAST_SPELL));
    packet.writeUInt32(spellId);
    packet.writeUInt8(castCount);
    // No castFlags byte in TBC 2.4.3
    packet.writeUInt32(0x0800); // TARGET_FLAG_GAMEOBJECT
    packet.writePackedGuid(targetGuid);
    LOG_DEBUG("[TBC] Built CMSG_CAST_SPELL: spell=", spellId, " gameObject=0x",
              std::hex, targetGuid, std::dec, " castCount=", static_cast<int>(castCount),
              " size=", packet.getSize());
    return packet;
}

// ============================================================================
// TBC 2.4.3 CMSG_USE_ITEM
// Format: bag(u8) + slot(u8) + spellIndex(u8) + castCount(u8) + itemGuid(u64)
//         + SpellCastTargets
// WotLK sends spellId/glyph/castFlags fields here; TBC does not.
// ============================================================================
network::Packet TbcPacketParsers::buildUseItem(uint8_t bagIndex, uint8_t slotIndex,
                                               uint64_t itemGuid, uint32_t /*spellId*/,
                                               uint64_t targetGuid, uint64_t itemTargetGuid) {
    network::Packet packet(wireOpcode(LogicalOpcode::CMSG_USE_ITEM));
    packet.writeUInt8(bagIndex);
    packet.writeUInt8(slotIndex);
    packet.writeUInt8(0);          // item spell index
    packet.writeUInt8(0);          // cast count
    packet.writeUInt64(itemGuid);  // full 8-byte GUID
    if (itemTargetGuid != 0) {
        packet.writeUInt32(0x10);  // TARGET_FLAG_ITEM
        packet.writePackedGuid(itemTargetGuid);
    } else if (targetGuid != 0) {
        packet.writeUInt32(0x02);  // TARGET_FLAG_UNIT
        packet.writePackedGuid(targetGuid);
    } else {
        packet.writeUInt32(0x00);  // TARGET_FLAG_SELF
    }
    return packet;
}

network::Packet TbcPacketParsers::buildAcceptQuestPacket(uint64_t npcGuid, uint32_t questId) {
    return buildAcceptQuestPacketPreWotlk(npcGuid, questId);
}

// ============================================================================
// TBC 2.4.3 SMSG_QUESTGIVER_QUEST_DETAILS
//
// TBC and Classic share the same format - neither has the WotLK-specific fields
// (informUnit GUID, flags uint32, isFinished uint8) that were added in 3.x.
//
// Format:
//   npcGuid(8) + questId(4) + title + details + objectives
//   + activateAccept(1) + suggestedPlayers(4)
//   + emoteCount(4) + [delay(4)+type(4)] × emoteCount
//   + choiceCount(4) + [itemId(4)+count(4)+displayInfo(4)] × choiceCount
//   + rewardCount(4) + [itemId(4)+count(4)+displayInfo(4)] × rewardCount
//   + rewardMoney(4) + rewardXp(4)
// ============================================================================
// Verified against vmangos (Server/Packets/Quest.cpp) and cmangos-tbc
// (Entities/GossipDef.cpp):
//   guid(8) + questId(4) + title + details + objectives +
//   activateAccept(u32)                     ← uint32 on BOTH, not uint8
//   [TBC only] suggestedPlayers(u32)
//   choiceCount(u32) + choiceCount × (itemId, count, displayId)   ← variable
//   rewardCount(u32) + rewardCount × (itemId, count, displayId)   ← variable
//   money(u32)
//   trailing: Classic = rewSpell; TBC = honor + rewSpell + rewSpellCast + title
//   emote block LAST (count + pairs) - never before the reward arrays.
// QUEST_FLAGS_HIDDEN_REWARDS quests serialize counts of 0 and money 0.
bool TbcPacketParsers::parseQuestDetailsPreWotlk(network::Packet& packet, QuestDetailsData& data,
                                                 bool hasSuggestedPlayers) {
    if (packet.getSize() < 16) return false;

    data.npcGuid = packet.readUInt64();
    data.questId = packet.readUInt32();
    data.title      = normalizeWowTextTokens(packet.readString());
    data.details    = normalizeWowTextTokens(packet.readString());
    data.objectives = normalizeWowTextTokens(packet.readString());

    if (!packet.hasRemaining(4)) {
        LOG_DEBUG("Quest details tbc/classic (short): id=", data.questId, " title='", data.title, "'");
        return !data.title.empty() || data.questId != 0;
    }

    /*activateAccept*/ packet.readUInt32();
    if (hasSuggestedPlayers && packet.hasRemaining(4))
        data.suggestedPlayers = packet.readUInt32();

    // Choice reward items (variable count, up to QUEST_REWARD_CHOICES_COUNT)
    if (packet.hasRemaining(4)) {
        uint32_t choiceCount = packet.readUInt32();
        if (choiceCount > 6) choiceCount = 0; // misaligned stream guard
        for (uint32_t i = 0; i < choiceCount && packet.hasRemaining(12); ++i) {
            uint32_t itemId = packet.readUInt32();
            uint32_t count  = packet.readUInt32();
            uint32_t dispId = packet.readUInt32();
            if (itemId != 0) {
                QuestRewardItem ri;
                ri.itemId = itemId; ri.count = count; ri.displayInfoId = dispId;
                ri.choiceSlot = i;
                data.rewardChoiceItems.push_back(ri);
            }
        }
    }

    // Fixed reward items (variable count, up to QUEST_REWARDS_COUNT)
    if (packet.hasRemaining(4)) {
        uint32_t rewardCount = packet.readUInt32();
        if (rewardCount > 4) rewardCount = 0; // misaligned stream guard
        for (uint32_t i = 0; i < rewardCount && packet.hasRemaining(12); ++i) {
            uint32_t itemId = packet.readUInt32();
            uint32_t count  = packet.readUInt32();
            uint32_t dispId = packet.readUInt32();
            if (itemId != 0) {
                QuestRewardItem ri;
                ri.itemId = itemId; ri.count = count; ri.displayInfoId = dispId;
                data.rewardItems.push_back(ri);
            }
        }
    }

    if (packet.hasRemaining(4))
        data.rewardMoney = packet.readUInt32();
    // Remaining bytes are spell/honor/title trailing fields plus the emote
    // block - no XP field exists pre-WotLK.
    data.rewardXp = 0;

    LOG_DEBUG("Quest details tbc/classic: id=", data.questId, " title='", data.title,
              "' choices=", data.rewardChoiceItems.size(), " fixed=", data.rewardItems.size());
    return true;
}

// ============================================================================
// TBC 2.4.3 CMSG_QUESTGIVER_QUERY_QUEST
//
// WotLK adds a trailing uint8 isDialogContinued byte; TBC does not.
// TBC format: guid(8) + questId(4) = 12 bytes.
// ============================================================================
network::Packet TbcPacketParsers::buildQueryQuestPacket(uint64_t npcGuid, uint32_t questId) {
    return buildQueryQuestPacketPreWotlk(npcGuid, questId);
}

// ============================================================================
// TBC parseAuraUpdate - SMSG_AURA_UPDATE doesn't exist in TBC
// TBC uses inline aura update fields + SMSG_INIT_EXTRA_AURA_INFO_OBSOLETE (0x3A3) /
// SMSG_SET_EXTRA_AURA_INFO_OBSOLETE (0x3A4) instead
// ============================================================================
bool TbcPacketParsers::parseAuraUpdate(network::Packet& /*packet*/, AuraUpdateData& /*data*/, bool /*isAll*/) {
    LOG_DEBUG("[TBC] parseAuraUpdate called but SMSG_AURA_UPDATE does not exist in TBC 2.4.3");
    return false;
}

// ============================================================================
// TBC/Classic parseNameQueryResponse
//
// WotLK uses: packedGuid + uint8 found + name + realmName + u8 race + u8 gender + u8 class
// Classic/TBC commonly use: uint64 guid + [optional uint8 found] + CString name + uint32 race + uint32 gender + uint32 class
//
// Implement a robust parser that handles both classic-era variants.
// ============================================================================
static bool hasNullWithin(const network::Packet& p, size_t start, size_t maxLen) {
    const auto& d = p.getData();
    size_t end = std::min(d.size(), start + maxLen);
    for (size_t i = start; i < end; i++) {
        if (d[i] == 0) return true;
    }
    return false;
}

bool TbcPacketParsers::parseNameQueryResponse(network::Packet& packet, NameQueryResponseData& data) {
    // Default all fields
    data = NameQueryResponseData{};

    size_t start = packet.getReadPos();
    if (packet.getSize() - start < 8) return false;

    // Variant A: guid(u64) + name + race(u32) + gender(u32) + class(u32)
    {
        packet.setReadPos(start);
        data.guid = packet.readUInt64();
        data.found = 0;
        data.name = packet.readString();
        if (!data.name.empty() && packet.hasRemaining(12)) {
            uint32_t race = packet.readUInt32();
            uint32_t gender = packet.readUInt32();
            uint32_t cls = packet.readUInt32();
            data.race = static_cast<uint8_t>(race & 0xFF);
            data.gender = static_cast<uint8_t>(gender & 0xFF);
            data.classId = static_cast<uint8_t>(cls & 0xFF);
            data.realmName.clear();
            return true;
        }
    }

    // Variant B: guid(u64) + found(u8) + [if found==0: name + race(u32)+gender(u32)+class(u32)]
    {
        packet.setReadPos(start);
        data.guid = packet.readUInt64();
        if (!packet.hasRemaining(1)) {
            packet.setReadPos(start);
            return false;
        }
        uint8_t found = packet.readUInt8();
        // Guard: only treat it as a found flag if a CString likely follows.
        if ((found == 0 || found == 1) && hasNullWithin(packet, packet.getReadPos(), 64)) {
            data.found = found;
            if (data.found != 0) return true;
            data.name = packet.readString();
            if (!data.name.empty() && packet.hasRemaining(12)) {
                uint32_t race = packet.readUInt32();
                uint32_t gender = packet.readUInt32();
                uint32_t cls = packet.readUInt32();
                data.race = static_cast<uint8_t>(race & 0xFF);
                data.gender = static_cast<uint8_t>(gender & 0xFF);
                data.classId = static_cast<uint8_t>(cls & 0xFF);
                data.realmName.clear();
                return true;
            }
        }
    }

    packet.setReadPos(start);
    return false;
}

// ============================================================================
// TBC parseItemQueryResponse - SMSG_ITEM_QUERY_SINGLE_RESPONSE (2.4.3 format)
//
// Differences from WotLK (handled by base class ItemQueryResponseParser::parse):
//   - No Flags2 field (WotLK added a second flags uint32 after Flags)
//   - No BuyCount field (WotLK added this between Flags2 and BuyPrice)
//   - Stats: sends 10 fixed stat pairs with no statsCount prefix
//   - No ScalingStatDistribution / ScalingStatValue (WotLK-only heirloom scaling)
//
// Differences from Classic (ClassicPacketParsers::parseItemQueryResponse):
//   - Has SoundOverrideSubclass (int32) after subClass (Classic lacks it)
//   - Otherwise keeps the fixed 10-stat-pair item metadata layout
// ============================================================================
bool TbcPacketParsers::parseItemQueryResponse(network::Packet& packet,
                                              ItemQueryResponseData& data) {
    return parseItemQueryPreWotlk(packet, data, /*hasSoundOverrideSubclass=*/true, "TBC");
}

// ============================================================================
// TbcPacketParsers::parseMailList - TBC 2.4.3 SMSG_MAIL_LIST_RESULT
//
// Differences from WotLK 3.3.5a (base implementation):
//   - Header: uint8 count only (WotLK: uint32 totalCount + uint8 shownCount)
//   - No body field; the body is fetched separately through itemTextId
//   - Header includes itemTextId + package before stationery
//   - Attachment enchants are 6 × {charges, duration, id}
//   - Attachment stack count is uint8 rather than uint32
// ============================================================================
bool TbcPacketParsers::parseMailList(network::Packet& packet, std::vector<MailMessage>& inbox) {
    size_t remaining = packet.getRemainingSize();
    if (remaining < 1) return false;

    uint8_t count = packet.readUInt8();
    LOG_INFO("SMSG_MAIL_LIST_RESULT (TBC): count=", static_cast<int>(count));

    inbox.clear();
    inbox.reserve(count);

    for (uint8_t i = 0; i < count; ++i) {
        remaining = packet.getRemainingSize();
        if (remaining < 2) break;

        uint16_t msgSize = packet.readUInt16();
        size_t startPos = packet.getReadPos();

        MailMessage msg;
        if (msgSize < 7 || remaining < 7) {
            LOG_WARNING("[TBC] Mail entry ", static_cast<int>(i), " truncated");
            break;
        }

        msg.messageId = packet.readUInt32();
        msg.messageType = packet.readUInt8();

        // CMaNGOS includes the uint16 size field in msgSize.  Its 2.4.3 size
        // calculation always budgets an 8-byte sender even though non-player
        // mail emits a uint32 sender, so those entries overstate their size by
        // four bytes.
        const size_t senderSizeAdjustment = msg.messageType == 0 ? 0 : 4;
        if (msgSize < 2 + senderSizeAdjustment ||
            remaining < static_cast<size_t>(msgSize) - senderSizeAdjustment) {
            LOG_WARNING("[TBC] Mail entry ", static_cast<int>(i), " truncated");
            break;
        }
        const size_t payloadSize = static_cast<size_t>(msgSize) - 2 - senderSizeAdjustment;
        const size_t entryEnd = startPos + payloadSize;

        switch (msg.messageType) {
            case 0: msg.senderGuid = packet.readUInt64(); break;
            default: msg.senderEntry = packet.readUInt32(); break;
        }

        msg.cod          = packet.readUInt32();
        packet.readUInt32();         // itemTextId (body fetched separately)
        packet.readUInt32();         // package (Package.dbc)
        msg.stationeryId = packet.readUInt32();
        msg.money        = packet.readUInt32();
        msg.flags        = packet.readUInt32();
        msg.expirationTime = packet.readFloat();
        msg.mailTemplateId = packet.readUInt32();
        msg.subject      = packet.readString();
        // TBC has no separate body field at all

        uint8_t attachCount = packet.readUInt8();
        msg.attachments.reserve(attachCount);
        for (uint8_t j = 0; j < attachCount; ++j) {
            MailAttachment att;
            att.slot         = packet.readUInt8();
            att.itemGuidLow  = packet.readUInt32();
            att.itemId       = packet.readUInt32();
            for (int e = 0; e < 6; ++e) {
                packet.readUInt32(); // charges
                packet.readUInt32(); // duration
                uint32_t enchId = packet.readUInt32();
                if (e == 0) att.enchantId = enchId;
            }
            att.randomPropertyId     = packet.readUInt32();
            att.randomSuffix         = packet.readUInt32();
            att.stackCount           = packet.readUInt8();
            att.chargesOrDurability  = packet.readUInt32();
            att.maxDurability        = packet.readUInt32();
            packet.readUInt32();  // current durability (separate from chargesOrDurability)
            msg.attachments.push_back(att);
        }

        msg.read = (msg.flags & 0x01) != 0;
        inbox.push_back(std::move(msg));

        if (packet.getReadPos() < entryEnd) {
            packet.setReadPos(entryEnd);
        } else if (packet.getReadPos() > entryEnd) {
            LOG_WARNING("[TBC] Mail entry ", static_cast<int>(i),
                        " exceeded declared size by ", packet.getReadPos() - entryEnd, " bytes");
        }
    }

    return true;
}

// ============================================================================
// ---------------------------------------------------------------------------
// skipTbcSpellCastTargets - consume all SpellCastTargets payload bytes for TBC.
//
// TBC uses uint32 targetFlags (Classic: uint16). Unit/item/object/corpse targets
// are PackedGuid (same as Classic). Source/dest location is 3 floats (12 bytes)
// with no transport guid (Classic: same; WotLK adds a transport PackedGuid).
//
// This helper is used by parseSpellStart to ensure the read position advances
// past ALL target payload fields so subsequent fields (e.g. those parsed by the
// caller after spell targets) are not corrupted.
// ---------------------------------------------------------------------------
static bool skipTbcSpellCastTargets(network::Packet& packet, uint64_t* primaryTargetGuid = nullptr) {
    if (!packet.hasRemaining(4)) return false;

    constexpr uint32_t TARGET_FLAG_UNIT            = 0x00000002;
    constexpr uint32_t TARGET_FLAG_ITEM            = 0x00000010;
    constexpr uint32_t TARGET_FLAG_SOURCE_LOCATION = 0x00000020;
    constexpr uint32_t TARGET_FLAG_DEST_LOCATION   = 0x00000040;
    constexpr uint32_t TARGET_FLAG_CORPSE_ENEMY    = 0x00000200;
    constexpr uint32_t TARGET_FLAG_GAMEOBJECT      = 0x00000800;
    constexpr uint32_t TARGET_FLAG_TRADE_ITEM      = 0x00001000;
    constexpr uint32_t TARGET_FLAG_STRING          = 0x00002000;
    constexpr uint32_t TARGET_FLAG_CORPSE_ALLY     = 0x00008000;
    constexpr uint32_t TARGET_FLAG_UNIT_MINIPET    = 0x00010000;

    const uint32_t targetFlags = packet.readUInt32();

    auto readPackedGuid = [&](bool capture) -> bool {
        if (!packet.hasFullPackedGuid()) return false;
        uint64_t g = packet.readPackedGuid();
        if (capture && primaryTargetGuid && *primaryTargetGuid == 0) *primaryTargetGuid = g;
        return true;
    };
    auto skipFloats3 = [&](uint32_t flag) -> bool {
        if (!(targetFlags & flag)) return true;
        if (!packet.hasRemaining(12)) return false;
        (void)packet.readFloat(); (void)packet.readFloat(); (void)packet.readFloat();
        return true;
    };

    // Process in wire order matching cmangos-tbc SpellCastTargets::write()
    const uint32_t objectTargetFlags = TARGET_FLAG_UNIT | TARGET_FLAG_CORPSE_ENEMY |
                                       TARGET_FLAG_GAMEOBJECT | TARGET_FLAG_CORPSE_ALLY |
                                       TARGET_FLAG_UNIT_MINIPET;
    if ((targetFlags & objectTargetFlags) && !readPackedGuid(true)) return false;

    if ((targetFlags & (TARGET_FLAG_ITEM | TARGET_FLAG_TRADE_ITEM)) && !readPackedGuid(false)) {
        return false;
    }

    if (!skipFloats3(TARGET_FLAG_SOURCE_LOCATION)) return false;
    if (!skipFloats3(TARGET_FLAG_DEST_LOCATION))   return false;

    if (targetFlags & TARGET_FLAG_STRING) {
        const auto& raw = packet.getData();
        size_t pos = packet.getReadPos();
        while (pos < raw.size() && raw[pos] != 0) ++pos;
        if (pos >= raw.size()) return false;
        packet.setReadPos(pos + 1);
    }

    return true;
}

// TbcPacketParsers::parseSpellStart - TBC 2.4.3 SMSG_SPELL_START
//
// CMaNGOS TBC sends:
//   PackedGuid(caster object/item) + PackedGuid(caster unit)
//   + spellId(u32) + castCount(u8) + castFlags(u16) + castTime(u32)
//   + SpellCastTargets
// ============================================================================
bool TbcPacketParsers::parseSpellStart(network::Packet& packet, SpellStartData& data) {
    data = SpellStartData{};
    auto rem = [&]() -> size_t { return packet.getRemainingSize(); };
    const size_t startPos = packet.getReadPos();

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

    if (rem() < 11) {
        packet.setReadPos(startPos);
        return false;
    }
    data.spellId    = packet.readUInt32();
    data.castCount  = packet.readUInt8();
    data.castFlags  = packet.readUInt16();
    data.castTime   = packet.readUInt32();

    // SpellCastTargets: consume ALL target payload types to keep the read position
    // aligned for any bytes the caller may parse after this (ammo, etc.).
    {
        uint64_t targetGuid = 0;
        skipTbcSpellCastTargets(packet, &targetGuid);  // non-fatal on truncation
        data.targetGuid = targetGuid;
    }

    LOG_DEBUG("[TBC] Spell start: spell=", data.spellId, " castTime=", data.castTime, "ms",
              " targetGuid=0x", std::hex, data.targetGuid, std::dec);
    return true;
}

// ============================================================================
// TbcPacketParsers::parseSpellGo - TBC 2.4.3 SMSG_SPELL_GO
//
// CMaNGOS TBC sends:
//   PackedGuid(caster object/item) + PackedGuid(caster unit)
//   + spellId(u32) + castFlags(u16) + timestamp(u32)
//   + hit/miss target lists (full ObjectGuid values)
//   + SpellCastTargets
// ============================================================================
bool TbcPacketParsers::parseSpellGo(network::Packet& packet, SpellGoData& data) {
    // Always reset output to avoid stale targets when callers reuse buffers.
    data = SpellGoData{};

    auto rem = [&]() -> size_t { return packet.getRemainingSize(); };
    const size_t startPos = packet.getReadPos();
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

    if (rem() < 10) {
        packet.setReadPos(startPos);
        return false;
    }
    data.castCount = 0;  // SMSG_SPELL_GO does not carry castCount in CMaNGOS TBC.
    data.spellId   = packet.readUInt32();
    data.castFlags = packet.readUInt16();
    (void)packet.readUInt32(); // timestamp

    if (rem() < 1) {
        LOG_WARNING("[TBC] Spell go: missing hitCount after fixed fields");
        packet.setReadPos(startPos);
        return false;
    }

    // Cap hit targets to prevent oversized allocations from malformed spell packets.
    // 128 is well above any real WoW AOE spell target count (max ~20 in practice).
    const uint8_t rawHitCount = packet.readUInt8();
    if (rawHitCount > 128) {
        LOG_WARNING("[TBC] Spell go: hitCount capped (requested=", static_cast<int>(rawHitCount), ")");
    }
    const uint8_t storedHitLimit = std::min<uint8_t>(rawHitCount, 128);
    data.hitTargets.reserve(storedHitLimit);
    bool truncatedTargets = false;
    for (uint16_t i = 0; i < rawHitCount; ++i) {
        if (rem() < 8) {
            LOG_WARNING("[TBC] Spell go: truncated hit targets at index ", i,
                        "/", static_cast<int>(rawHitCount));
            truncatedTargets = true;
            break;
        }
        const uint64_t targetGuid = packet.readUInt64();  // full GUID in TBC
        if (i < storedHitLimit) {
            data.hitTargets.push_back(targetGuid);
        }
    }
    if (truncatedTargets) {
        packet.setReadPos(startPos);
        return false;
    }
    data.hitCount = static_cast<uint8_t>(data.hitTargets.size());

    if (rem() < 1) {
        LOG_WARNING("[TBC] Spell go: missing missCount after hit target list");
        packet.setReadPos(startPos);
        return false;
    }

    const uint8_t rawMissCount = packet.readUInt8();
    if (rawMissCount > 128) {
        LOG_WARNING("[TBC] Spell go: missCount capped (requested=", static_cast<int>(rawMissCount), ")");
    }
    const uint8_t storedMissLimit = std::min<uint8_t>(rawMissCount, 128);
    data.missTargets.reserve(storedMissLimit);
    for (uint16_t i = 0; i < rawMissCount; ++i) {
        if (rem() < 9) {
            LOG_WARNING("[TBC] Spell go: truncated miss targets at index ", i,
                        "/", static_cast<int>(rawMissCount));
            truncatedTargets = true;
            break;
        }
        SpellGoMissEntry m;
        m.targetGuid = packet.readUInt64();  // full GUID in TBC
        m.missType   = packet.readUInt8();
        if (m.missType == 11) { // SPELL_MISS_REFLECT
            if (rem() < 1) {
                LOG_WARNING("[TBC] Spell go: truncated reflect payload at miss index ", i,
                            "/", static_cast<int>(rawMissCount));
                truncatedTargets = true;
                break;
            }
            (void)packet.readUInt8(); // reflectResult
        }
        if (i < storedMissLimit) {
            data.missTargets.push_back(m);
        }
    }
    if (truncatedTargets) {
        packet.setReadPos(startPos);
        return false;
    }
    data.missCount = static_cast<uint8_t>(data.missTargets.size());

    // SpellCastTargets follows the miss list - consume all target bytes so that
    // any subsequent fields are not misaligned for ground-targeted AoE spells.
    skipTbcSpellCastTargets(packet, &data.targetGuid);

    LOG_DEBUG("[TBC] Spell go: spell=", data.spellId, " hits=", static_cast<int>(data.hitCount),
              " misses=", static_cast<int>(data.missCount));
    return true;
}

static uint8_t translateTbcCastFailure(uint8_t tbcResult) {
    // TBC has no SUCCESS entry, while WoWee's shared string table is WotLK-based.
    // Most early values line up with +1.  Later enum sections diverge; map observed
    // high-value TBC failures explicitly so user-facing errors stay sane.
    switch (tbcResult) {
        case 0x17: return 25;  // SPELL_FAILED_CHEST_IN_USE
        case 0x24: return 38;  // SPELL_FAILED_IMMUNE
        case 0x25: return 40;  // SPELL_FAILED_INTERRUPTED
        case 0x26: return 41;  // SPELL_FAILED_INTERRUPTED_COMBAT
        case 0x2E: return 49;  // SPELL_FAILED_LOW_CASTLEVEL
        case 0x7F: return 132; // SPELL_FAILED_TRY_AGAIN
        case 0x91: return 146; // SPELL_FAILED_DAMAGE_IMMUNE
        case 0x95: return 150; // SPELL_FAILED_MIN_SKILL
        default: break;
    }
    if (tbcResult == 63) return 67; // SPELL_FAILED_NOT_READY
    return static_cast<uint8_t>(tbcResult + 1);
}

// ============================================================================
// TbcPacketParsers::parseCastResult - TBC 2.4.3 SMSG_CAST_RESULT
//
// TBC format: spellId(u32) + result(u8) + castCount(u8).
// ============================================================================
bool TbcPacketParsers::parseCastResult(network::Packet& packet, uint32_t& spellId, uint8_t& result,
                                       uint32_t& miscArg, uint32_t& miscArg2) {
    if (!packet.hasRemaining(5)) return false;
    spellId = packet.readUInt32();
    uint8_t tbcResult = packet.readUInt8();
    result = translateTbcCastFailure(tbcResult);
    if (packet.hasRemaining(1)) packet.readUInt8(); // castCount
    // Spell-focus and totem failures append ids naming the missing station/tool
    readCastResultArgs(packet, result, miscArg, miscArg2);
    LOG_DEBUG("[TBC] Cast result: spell=", spellId,
              " tbcResult=", static_cast<int>(tbcResult),
              " mappedResult=", static_cast<int>(result));
    return true;
}

// ============================================================================
// TbcPacketParsers::parseCastFailed - TBC 2.4.3 SMSG_CAST_FAILED
//
// TBC format: spellId(u32) + result(u8) + castCount(u8).
// ============================================================================
bool TbcPacketParsers::parseCastFailed(network::Packet& packet, CastFailedData& data) {
    if (!packet.hasRemaining(5)) return false;
    data.castCount = 0;
    data.spellId   = packet.readUInt32();
    uint8_t tbcResult = packet.readUInt8();
    data.result = translateTbcCastFailure(tbcResult);
    if (packet.hasRemaining(1)) data.castCount = packet.readUInt8();
    // Spell-focus and totem failures append ids naming the missing station/tool
    readCastResultArgs(packet, data.result, data.miscArg, data.miscArg2);
    LOG_DEBUG("[TBC] Cast failed: spell=", data.spellId,
              " tbcResult=", static_cast<int>(tbcResult),
              " mappedResult=", static_cast<int>(data.result));
    return true;
}

// ============================================================================
// TbcPacketParsers::parseAttackerStateUpdate - TBC 2.4.3 SMSG_ATTACKERSTATEUPDATE
//
// CMaNGOS TBC writes attacker and target as packed GUIDs:
//   hitInfo(u32) + PackedGuid(attacker) + PackedGuid(target)
//   + damage(u32) + subDamageCount(u8) + sub-damage entries
//   + victimState(u32) + unknown(u32) + spellId(u32) + blocked(u32)
// ============================================================================
bool TbcPacketParsers::parseAttackerStateUpdate(network::Packet& packet, AttackerStateUpdateData& data) {
    data = AttackerStateUpdateData{};

    auto rem = [&]() { return packet.getRemainingSize(); };
    if (rem() < 5) return false;  // hitInfo + at least one packed guid mask byte

    const size_t startPos = packet.getReadPos();
    if (!data.readCommonHead(packet, startPos)) return false;

    // TBC sends victim state, an unknown field, a spell id field used by some
    // melee specials, then blocked amount.  There is no overkill field here.
    if (rem() < 16) {
        packet.setReadPos(startPos);
        return false;
    }
    data.victimState = packet.readUInt32();
    (void)packet.readUInt32(); // unknown, commonly 0
    (void)packet.readUInt32(); // spell id for some melee specials
    data.blocked = packet.readUInt32();
    data.overkill = -1;

    LOG_DEBUG("[TBC] Melee hit: ", data.totalDamage, " damage",
              data.isCrit() ? " (CRIT)" : "",
              data.isMiss() ? " (MISS)" : "");
    return true;
}

// ============================================================================
// TbcPacketParsers::parseSpellDamageLog - TBC 2.4.3 SMSG_SPELLNONMELEEDAMAGELOG
//
// CMaNGOS TBC writes target and attacker as packed GUIDs.
// ============================================================================
bool TbcPacketParsers::parseSpellDamageLog(network::Packet& packet, SpellDamageLogData& data) {
    return parseSpellDamageLogPreWotlk(packet, data, "[TBC]");
}

// ============================================================================
// TbcPacketParsers::parseSpellHealLog - TBC 2.4.3 SMSG_SPELLHEALLOG
//
// CMaNGOS TBC writes target and caster as packed GUIDs.
// ============================================================================
bool TbcPacketParsers::parseSpellHealLog(network::Packet& packet, SpellHealLogData& data) {
    data = SpellHealLogData{};
    auto rem = [&]() { return packet.getRemainingSize(); };

    if (rem() < 2 || !packet.hasFullPackedGuid()) return false;
    data.targetGuid = packet.readPackedGuid();
    if (rem() < 1 || !packet.hasFullPackedGuid()) return false;
    data.casterGuid = packet.readPackedGuid();

    if (rem() < 12) return false;
    data.spellId    = packet.readUInt32();
    data.heal       = packet.readUInt32();
    data.overheal   = packet.readUInt32();
    // TBC has no absorbed field in SMSG_SPELLHEALLOG; skip crit flag
    if (packet.getReadPos() < packet.getSize()) {
        uint8_t critFlag = packet.readUInt8();
        data.isCrit = (critFlag != 0);
    }

    LOG_DEBUG("[TBC] Spell heal: spellId=", data.spellId, " heal=", data.heal,
              data.isCrit ? " CRIT" : "");
    return true;
}

// ============================================================================
// TBC 2.4.3 SMSG_MESSAGECHAT
// CMaNGOS TBC uses the same senderGuid + unknown + type-specific body shape
// as WotLK here. In particular, system and whisper packets carry an extra
// receiverGuid before messageLen.
// ============================================================================

bool TbcPacketParsers::parseMessageChat(network::Packet& packet, MessageChatData& data) {
    return MessageChatParser::parse(packet, data);
}

// ============================================================================
// TBC 2.4.3 quest giver status
// TBC sends uint32 (like Classic), WotLK changed to uint8.
// TBC 2.4.3 enum: 0=NONE,1=UNAVAILABLE,2=CHAT,3=INCOMPLETE,4=REWARD_REP,
//   5=AVAILABLE_REP,6=AVAILABLE,7=REWARD2,8=REWARD
// ============================================================================

uint8_t TbcPacketParsers::readQuestGiverStatus(network::Packet& packet) {
    uint32_t tbcStatus = packet.readUInt32();
    switch (tbcStatus) {
        case 0: return 0;   // NONE
        case 1: return 1;   // UNAVAILABLE
        case 2: return 0;   // CHAT → NONE (no marker)
        case 3: return 5;   // INCOMPLETE → WotLK INCOMPLETE
        case 4: return 6;   // REWARD_REP → WotLK REWARD_REP
        case 5: return 7;   // AVAILABLE_REP → WotLK AVAILABLE_REP
        case 6: return 8;   // AVAILABLE → WotLK AVAILABLE
        case 7: return 10;  // REWARD2 → WotLK REWARD
        case 8: return 10;  // REWARD → WotLK REWARD
        default: return 0;
    }
}

// ============================================================================
// TBC 2.4.3 channel join/leave
// CMaNGOS TBC expects the same channelId/flags prefix shape as WotLK.
// Without this, the server consumes the first bytes of the channel name as
// metadata and joins channels named e.g. "l" instead of "General".
// ============================================================================

network::Packet TbcPacketParsers::buildJoinChannel(const std::string& channelName, const std::string& password) {
    network::Packet packet(wireOpcode(Opcode::CMSG_JOIN_CHANNEL));
    packet.writeUInt32(0);  // channelId (unused)
    packet.writeUInt8(0);   // hasVoice
    packet.writeUInt8(0);   // joinedByZone
    packet.writeString(channelName);
    packet.writeString(password);
    LOG_DEBUG("[TBC] Built CMSG_JOIN_CHANNEL: channel=", channelName);
    return packet;
}

network::Packet TbcPacketParsers::buildLeaveChannel(const std::string& channelName) {
    network::Packet packet(wireOpcode(Opcode::CMSG_LEAVE_CHANNEL));
    packet.writeUInt32(0);  // channelId (unused)
    packet.writeString(channelName);
    LOG_DEBUG("[TBC] Built CMSG_LEAVE_CHANNEL: channel=", channelName);
    return packet;
}

// ============================================================================
// TBC 2.4.3 SMSG_GAMEOBJECT_QUERY_RESPONSE
// TBC has 2 extra strings after name[4] (iconName + castBarCaption).
// WotLK has 3 (adds unk1). Classic has 0.
// ============================================================================

bool TbcPacketParsers::parseGameObjectQueryResponse(network::Packet& packet, GameObjectQueryResponseData& data) {
    // Two of the 2.0.3 strings, where WotLK reads three. See
    // parseGameObjectQueryBody for why that difference is not settled.
    return parseGameObjectQueryBody(packet, data, /*extraStrings=*/2);
}

// ============================================================================
// TBC 2.4.3 guild roster parser
// Same rank structure as WotLK (variable rankCount + goldLimit + bank tabs),
// but NO gender byte per member (WotLK added it).
// ============================================================================

bool TbcPacketParsers::parseGuildRoster(network::Packet& packet, GuildRosterData& data) {
    // TBC's roster is WotLK's without the gender byte. It was ninety-nine
    // lines saying so, and the copy dropped the bank tab rights it had just
    // read - see parseGuildRosterBody.
    return parseGuildRosterBody(packet, data, {/*rankCount=*/.rankCount=true, /*gender=*/.gender=false});
}

// ============================================================================
// TBC 2.4.3 guild query response parser
// Classic/TBC shape: guildId + name + 10 rank names + 5 emblem fields.
// WotLK adds a trailing rankCount field; do not require it here.
// ============================================================================

bool TbcPacketParsers::parseGuildQueryResponse(network::Packet& packet, GuildQueryResponseData& data) {
    if (packet.getSize() < 4) {
        LOG_ERROR("TBC SMSG_GUILD_QUERY_RESPONSE too small: ", packet.getSize());
        return false;
    }

    data.guildId = packet.readUInt32();

    if (!packet.hasData()) {
        LOG_WARNING("TBC GuildQueryResponse: truncated before guild name");
        data.guildName.clear();
        data.rankCount = 10;
        return true;
    }

    data.guildName = packet.readString();
    for (int i = 0; i < 10; ++i) {
        if (!packet.hasData()) {
            LOG_WARNING("TBC GuildQueryResponse: truncated at rank name ", i);
            data.rankNames[i].clear();
            continue;
        }
        data.rankNames[i] = packet.readString();
    }

    if (!packet.hasRemaining(20)) {
        LOG_WARNING("TBC GuildQueryResponse: truncated before emblem data");
        data.emblemStyle = 0;
        data.emblemColor = 0;
        data.borderStyle = 0;
        data.borderColor = 0;
        data.backgroundColor = 0;
        data.rankCount = 10;
        return true;
    }

    data.emblemStyle = packet.readUInt32();
    data.emblemColor = packet.readUInt32();
    data.borderStyle = packet.readUInt32();
    data.borderColor = packet.readUInt32();
    data.backgroundColor = packet.readUInt32();
    data.rankCount = 10;

    LOG_INFO("Parsed TBC SMSG_GUILD_QUERY_RESPONSE: guild=", data.guildName, " id=", data.guildId);
    return true;
}

} // namespace game
} // namespace wowee
