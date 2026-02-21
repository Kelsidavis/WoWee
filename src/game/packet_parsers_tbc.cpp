#include "game/packet_parsers.hpp"
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
    constexpr uint32_t FLYING           = 0x01000000;  // WotLK: 0x02000000
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
// - Flag 0x08 (HIGH_GUID) reads 2 u32s (Classic: 1 u32)
// ============================================================================
bool TbcPacketParsers::parseMovementBlock(network::Packet& packet, UpdateBlock& block) {
    // TBC 2.4.3: UpdateFlags is uint8 (1 byte)
    uint8_t updateFlags = packet.readUInt8();
    block.updateFlags = static_cast<uint16_t>(updateFlags);

    LOG_DEBUG("  [TBC] UpdateFlags: 0x", std::hex, (int)updateFlags, std::dec);

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
    const uint8_t UPDATEFLAG_HAS_TARGET          = 0x04;
    const uint8_t UPDATEFLAG_TRANSPORT           = 0x02;
    const uint8_t UPDATEFLAG_LOWGUID             = 0x08;
    const uint8_t UPDATEFLAG_HIGHGUID            = 0x10;

    if (updateFlags & UPDATEFLAG_LIVING) {
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
            block.onTransport = true;
            block.transportGuid = UpdateObjectParser::readPackedGuid(packet);
            block.transportX = packet.readFloat();
            block.transportY = packet.readFloat();
            block.transportZ = packet.readFloat();
            block.transportO = packet.readFloat();
            /*uint32_t tTime =*/ packet.readUInt32();
            // TBC: NO transport seat byte
            // TBC: NO interpolated movement check
        }

        // Pitch: SWIMMING, or else ONTRANSPORT (TBC-specific secondary pitch)
        if (moveFlags & TbcMoveFlags::SWIMMING) {
            /*float pitch =*/ packet.readFloat();
        } else if (moveFlags & TbcMoveFlags::ONTRANSPORT) {
            /*float pitch =*/ packet.readFloat();
        }

        // Fall time (always present)
        /*uint32_t fallTime =*/ packet.readUInt32();

        // Jumping (TBC: JUMPING=0x2000, WotLK: FALLING=0x1000)
        if (moveFlags & TbcMoveFlags::JUMPING) {
            /*float jumpVelocity =*/ packet.readFloat();
            /*float jumpSinAngle =*/ packet.readFloat();
            /*float jumpCosAngle =*/ packet.readFloat();
            /*float jumpXYSpeed =*/ packet.readFloat();
        }

        // Spline elevation (TBC: 0x02000000, WotLK: 0x04000000)
        if (moveFlags & TbcMoveFlags::SPLINE_ELEVATION) {
            /*float splineElevation =*/ packet.readFloat();
        }

        // Speeds (TBC: 8 values — walk, run, runBack, swim, fly, flyBack, swimBack, turn)
        // WotLK adds pitchRate (9 total)
        /*float walkSpeed =*/ packet.readFloat();
        float runSpeed = packet.readFloat();
        /*float runBackSpeed =*/ packet.readFloat();
        /*float swimSpeed =*/ packet.readFloat();
        /*float flySpeed =*/ packet.readFloat();
        /*float flyBackSpeed =*/ packet.readFloat();
        /*float swimBackSpeed =*/ packet.readFloat();
        /*float turnRate =*/ packet.readFloat();

        block.runSpeed = runSpeed;

        // Spline data (TBC/WotLK: SPLINE_ENABLED = 0x08000000)
        if (moveFlags & TbcMoveFlags::SPLINE_ENABLED) {
            uint32_t splineFlags = packet.readUInt32();
            LOG_DEBUG("  [TBC] Spline: flags=0x", std::hex, splineFlags, std::dec);

            if (splineFlags & 0x00010000) { // FINAL_POINT
                /*float finalX =*/ packet.readFloat();
                /*float finalY =*/ packet.readFloat();
                /*float finalZ =*/ packet.readFloat();
            } else if (splineFlags & 0x00020000) { // FINAL_TARGET
                /*uint64_t finalTarget =*/ packet.readUInt64();
            } else if (splineFlags & 0x00040000) { // FINAL_ANGLE
                /*float finalAngle =*/ packet.readFloat();
            }

            // TBC spline: timePassed, duration, id, nodes, finalNode
            // (no durationMod, durationModNext, verticalAccel, effectStartTime, splineMode)
            /*uint32_t timePassed =*/ packet.readUInt32();
            /*uint32_t duration =*/ packet.readUInt32();
            /*uint32_t splineId =*/ packet.readUInt32();

            uint32_t pointCount = packet.readUInt32();
            if (pointCount > 256) {
                static uint32_t badTbcSplineCount = 0;
                ++badTbcSplineCount;
                if (badTbcSplineCount <= 5 || (badTbcSplineCount % 100) == 0) {
                    LOG_WARNING("  [TBC] Spline pointCount=", pointCount,
                                " exceeds max, capping (occurrence=", badTbcSplineCount, ")");
                }
                pointCount = 0;
            }
            for (uint32_t i = 0; i < pointCount; i++) {
                /*float px =*/ packet.readFloat();
                /*float py =*/ packet.readFloat();
                /*float pz =*/ packet.readFloat();
            }

            // TBC: NO splineMode byte (WotLK adds it)
            /*float endPointX =*/ packet.readFloat();
            /*float endPointY =*/ packet.readFloat();
            /*float endPointZ =*/ packet.readFloat();
        }
    }
    else if (updateFlags & UPDATEFLAG_HAS_POSITION) {
        // TBC: Simple stationary position (same as WotLK STATIONARY)
        block.x = packet.readFloat();
        block.y = packet.readFloat();
        block.z = packet.readFloat();
        block.orientation = packet.readFloat();
        block.hasMovement = true;

        LOG_DEBUG("  [TBC] STATIONARY: (", block.x, ", ", block.y, ", ", block.z, ")");
    }
    // TBC: No UPDATEFLAG_POSITION (0x0100) code path

    // Target GUID
    if (updateFlags & UPDATEFLAG_HAS_TARGET) {
        /*uint64_t targetGuid =*/ UpdateObjectParser::readPackedGuid(packet);
    }

    // Transport time
    if (updateFlags & UPDATEFLAG_TRANSPORT) {
        /*uint32_t transportTime =*/ packet.readUInt32();
    }

    // TBC: No VEHICLE flag (WotLK 0x0080)
    // TBC: No ROTATION flag (WotLK 0x0200)

    // HIGH_GUID (0x08) — TBC has 2 u32s, Classic has 1 u32
    if (updateFlags & UPDATEFLAG_LOWGUID) {
        /*uint32_t unknown0 =*/ packet.readUInt32();
        /*uint32_t unknown1 =*/ packet.readUInt32();
    }

    // ALL (0x10)
    if (updateFlags & UPDATEFLAG_HIGHGUID) {
        /*uint32_t unknown2 =*/ packet.readUInt32();
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
    packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.x), sizeof(float));
    packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.y), sizeof(float));
    packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.z), sizeof(float));
    packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.orientation), sizeof(float));

    // Transport data (TBC ON_TRANSPORT = 0x200, same bit as WotLK)
    if (info.flags & TbcMoveFlags::ON_TRANSPORT) {
        // Packed transport GUID
        uint8_t transMask = 0;
        uint8_t transGuidBytes[8];
        int transGuidByteCount = 0;
        for (int i = 0; i < 8; i++) {
            uint8_t byte = static_cast<uint8_t>((info.transportGuid >> (i * 8)) & 0xFF);
            if (byte != 0) {
                transMask |= (1 << i);
                transGuidBytes[transGuidByteCount++] = byte;
            }
        }
        packet.writeUInt8(transMask);
        for (int i = 0; i < transGuidByteCount; i++) {
            packet.writeUInt8(transGuidBytes[i]);
        }

        // Transport local position
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.transportX), sizeof(float));
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.transportY), sizeof(float));
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.transportZ), sizeof(float));
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.transportO), sizeof(float));

        // Transport time
        packet.writeUInt32(info.transportTime);

        // TBC: NO transport seat byte
        // TBC: NO interpolated movement time
    }

    // Pitch: SWIMMING or else ONTRANSPORT (TBC flag positions)
    if (info.flags & TbcMoveFlags::SWIMMING) {
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.pitch), sizeof(float));
    } else if (info.flags & TbcMoveFlags::ONTRANSPORT) {
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.pitch), sizeof(float));
    }

    // Fall time (always present)
    packet.writeUInt32(info.fallTime);

    // Jump data (TBC JUMPING = 0x2000, WotLK FALLING = 0x1000)
    if (info.flags & TbcMoveFlags::JUMPING) {
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.jumpVelocity), sizeof(float));
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.jumpSinAngle), sizeof(float));
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.jumpCosAngle), sizeof(float));
        packet.writeBytes(reinterpret_cast<const uint8_t*>(&info.jumpXYSpeed), sizeof(float));
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
    uint8_t count = packet.readUInt8();

    LOG_INFO("[TBC] Parsing SMSG_CHAR_ENUM: ", (int)count, " characters");

    response.characters.clear();
    response.characters.reserve(count);

    for (uint8_t i = 0; i < count; ++i) {
        Character character;

        // GUID (8 bytes)
        character.guid = packet.readUInt64();

        // Name (null-terminated string)
        character.name = packet.readString();

        // Race, class, gender
        character.race = static_cast<Race>(packet.readUInt8());
        character.characterClass = static_cast<Class>(packet.readUInt8());
        character.gender = static_cast<Gender>(packet.readUInt8());

        // Appearance (5 bytes: skin, face, hairStyle, hairColor packed + facialFeatures)
        character.appearanceBytes = packet.readUInt32();
        character.facialFeatures = packet.readUInt8();

        // Level
        character.level = packet.readUInt8();

        // Location
        character.zoneId = packet.readUInt32();
        character.mapId = packet.readUInt32();
        character.x = packet.readFloat();
        character.y = packet.readFloat();
        character.z = packet.readFloat();

        // Guild ID
        character.guildId = packet.readUInt32();

        // Flags
        character.flags = packet.readUInt32();

        // TBC: uint8 firstLogin (WotLK: uint32 customization + uint8 unknown)
        /*uint8_t firstLogin =*/ packet.readUInt8();

        // Pet data (always present)
        character.pet.displayModel = packet.readUInt32();
        character.pet.level = packet.readUInt32();
        character.pet.family = packet.readUInt32();

        // Equipment (TBC: 20 items, WotLK: 23 items)
        character.equipment.reserve(20);
        for (int j = 0; j < 20; ++j) {
            EquipmentItem item;
            item.displayModel = packet.readUInt32();
            item.inventoryType = packet.readUInt8();
            item.enchantment = packet.readUInt32();
            character.equipment.push_back(item);
        }

        LOG_INFO("  Character ", (int)(i + 1), ": ", character.name);
        LOG_INFO("    GUID: 0x", std::hex, character.guid, std::dec);
        LOG_INFO("    ", getRaceName(character.race), " ",
                 getClassName(character.characterClass), " (",
                 getGenderName(character.gender), ")");
        LOG_INFO("    Level: ", (int)character.level);
        LOG_INFO("    Location: Zone ", character.zoneId, ", Map ", character.mapId);

        response.characters.push_back(character);
    }

    LOG_INFO("[TBC] Parsed ", response.characters.size(), " characters");
    return true;
}

// ============================================================================
// TBC parseUpdateObject
// Key difference from WotLK: u8 has_transport byte after blockCount
// (WotLK removed this field)
// ============================================================================
bool TbcPacketParsers::parseUpdateObject(network::Packet& packet, UpdateObjectData& data) {
    // Read block count
    data.blockCount = packet.readUInt32();

    // TBC/Classic: has_transport byte (WotLK removed this)
    /*uint8_t hasTransport =*/ packet.readUInt8();

    LOG_DEBUG("[TBC] SMSG_UPDATE_OBJECT: objectCount=", data.blockCount);

    // Check for out-of-range objects first
    if (packet.getReadPos() + 1 <= packet.getSize()) {
        uint8_t firstByte = packet.readUInt8();

        if (firstByte == static_cast<uint8_t>(UpdateType::OUT_OF_RANGE_OBJECTS)) {
            uint32_t count = packet.readUInt32();
            for (uint32_t i = 0; i < count; ++i) {
                uint64_t guid = UpdateObjectParser::readPackedGuid(packet);
                data.outOfRangeGuids.push_back(guid);
                LOG_DEBUG("    Out of range: 0x", std::hex, guid, std::dec);
            }
        } else {
            packet.setReadPos(packet.getReadPos() - 1);
        }
    }

    // Parse update blocks — dispatching movement via virtual parseMovementBlock()
    data.blocks.reserve(data.blockCount);
    for (uint32_t i = 0; i < data.blockCount; ++i) {
        LOG_DEBUG("Parsing block ", i + 1, " / ", data.blockCount);
        UpdateBlock block;

        // Read update type
        uint8_t updateTypeVal = packet.readUInt8();
        block.updateType = static_cast<UpdateType>(updateTypeVal);
        LOG_DEBUG("Update block: type=", (int)updateTypeVal);

        bool ok = false;
        switch (block.updateType) {
            case UpdateType::VALUES: {
                block.guid = UpdateObjectParser::readPackedGuid(packet);
                ok = UpdateObjectParser::parseUpdateFields(packet, block);
                break;
            }
            case UpdateType::MOVEMENT: {
                block.guid = UpdateObjectParser::readPackedGuid(packet);
                ok = this->parseMovementBlock(packet, block);
                break;
            }
            case UpdateType::CREATE_OBJECT:
            case UpdateType::CREATE_OBJECT2: {
                block.guid = UpdateObjectParser::readPackedGuid(packet);
                uint8_t objectTypeVal = packet.readUInt8();
                block.objectType = static_cast<ObjectType>(objectTypeVal);
                ok = this->parseMovementBlock(packet, block);
                if (ok) {
                    ok = UpdateObjectParser::parseUpdateFields(packet, block);
                }
                break;
            }
            case UpdateType::OUT_OF_RANGE_OBJECTS:
            case UpdateType::NEAR_OBJECTS:
                ok = true;
                break;
            default:
                LOG_WARNING("Unknown update type: ", (int)updateTypeVal);
                ok = false;
                break;
        }

        if (!ok) {
            LOG_WARNING("Failed to parse update block ", i + 1, " of ", data.blockCount,
                        " — keeping ", data.blocks.size(), " parsed blocks");
            break;
        }
        data.blocks.push_back(block);
    }

    return true;
}

network::Packet TbcPacketParsers::buildAcceptQuestPacket(uint64_t npcGuid, uint32_t questId) {
    network::Packet packet(wireOpcode(Opcode::CMSG_QUESTGIVER_ACCEPT_QUEST));
    packet.writeUInt64(npcGuid);
    packet.writeUInt32(questId);
    // TBC servers generally expect guid + questId only.
    return packet;
}

// ============================================================================
// TBC parseAuraUpdate - SMSG_AURA_UPDATE doesn't exist in TBC
// TBC uses inline aura update fields + SMSG_INIT_EXTRA_AURA_INFO_OBSOLETE (0x3A3) /
// SMSG_SET_EXTRA_AURA_INFO_OBSOLETE (0x3A4) instead
// ============================================================================
bool TbcPacketParsers::parseAuraUpdate(network::Packet& /*packet*/, AuraUpdateData& /*data*/, bool /*isAll*/) {
    LOG_WARNING("[TBC] parseAuraUpdate called but SMSG_AURA_UPDATE does not exist in TBC 2.4.3");
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
        if (!data.name.empty() && (packet.getSize() - packet.getReadPos()) >= 12) {
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
        if (packet.getSize() - packet.getReadPos() < 1) {
            packet.setReadPos(start);
            return false;
        }
        uint8_t found = packet.readUInt8();
        // Guard: only treat it as a found flag if a CString likely follows.
        if ((found == 0 || found == 1) && hasNullWithin(packet, packet.getReadPos(), 64)) {
            data.found = found;
            if (data.found != 0) return true;
            data.name = packet.readString();
            if (!data.name.empty() && (packet.getSize() - packet.getReadPos()) >= 12) {
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
//   - Stats: sends exactly statsCount pairs (WotLK always sends 10)
//   - No ScalingStatDistribution / ScalingStatValue (WotLK-only heirloom scaling)
//
// Differences from Classic (ClassicPacketParsers::parseItemQueryResponse):
//   - Has SoundOverrideSubclass (int32) after subClass (Classic lacks it)
//   - Has statsCount prefix (Classic reads 10 pairs with no prefix)
// ============================================================================
bool TbcPacketParsers::parseItemQueryResponse(network::Packet& packet, ItemQueryResponseData& data) {
    data.entry = packet.readUInt32();
    if (data.entry & 0x80000000) {
        data.entry &= ~0x80000000;
        return true;
    }

    uint32_t itemClass = packet.readUInt32();
    uint32_t subClass  = packet.readUInt32();
    data.itemClass = itemClass;
    data.subClass  = subClass;
    packet.readUInt32(); // SoundOverrideSubclass (int32, -1 = no override)
    data.subclassName = "";

    // Name strings
    data.name = packet.readString();
    packet.readString(); // name2
    packet.readString(); // name3
    packet.readString(); // name4

    data.displayInfoId = packet.readUInt32();
    data.quality       = packet.readUInt32();

    packet.readUInt32(); // Flags  (TBC: 1 flags field only — no Flags2)
    // TBC: NO Flags2, NO BuyCount
    packet.readUInt32(); // BuyPrice
    data.sellPrice = packet.readUInt32();

    data.inventoryType = packet.readUInt32();

    packet.readUInt32(); // AllowableClass
    packet.readUInt32(); // AllowableRace
    packet.readUInt32(); // ItemLevel
    packet.readUInt32(); // RequiredLevel
    packet.readUInt32(); // RequiredSkill
    packet.readUInt32(); // RequiredSkillRank
    packet.readUInt32(); // RequiredSpell
    packet.readUInt32(); // RequiredHonorRank
    packet.readUInt32(); // RequiredCityRank
    packet.readUInt32(); // RequiredReputationFaction
    packet.readUInt32(); // RequiredReputationRank
    packet.readUInt32(); // MaxCount
    data.maxStack       = static_cast<int32_t>(packet.readUInt32()); // Stackable
    data.containerSlots = packet.readUInt32();

    // TBC: statsCount prefix + exactly statsCount pairs (WotLK always sends 10)
    uint32_t statsCount = packet.readUInt32();
    if (statsCount > 10) statsCount = 10; // sanity cap
    for (uint32_t i = 0; i < statsCount; i++) {
        uint32_t statType  = packet.readUInt32();
        int32_t  statValue = static_cast<int32_t>(packet.readUInt32());
        switch (statType) {
            case 3: data.agility  = statValue; break;
            case 4: data.strength = statValue; break;
            case 5: data.intellect = statValue; break;
            case 6: data.spirit   = statValue; break;
            case 7: data.stamina  = statValue; break;
            default: break;
        }
    }
    // TBC: NO ScalingStatDistribution, NO ScalingStatValue (WotLK-only)

    // 5 damage entries
    bool haveWeaponDamage = false;
    for (int i = 0; i < 5; i++) {
        float    dmgMin     = packet.readFloat();
        float    dmgMax     = packet.readFloat();
        uint32_t damageType = packet.readUInt32();
        if (!haveWeaponDamage && dmgMax > 0.0f) {
            if (damageType == 0 || data.damageMax <= 0.0f) {
                data.damageMin = dmgMin;
                data.damageMax = dmgMax;
                haveWeaponDamage = (damageType == 0);
            }
        }
    }

    data.armor = static_cast<int32_t>(packet.readUInt32());

    if (packet.getSize() - packet.getReadPos() >= 28) {
        packet.readUInt32(); // HolyRes
        packet.readUInt32(); // FireRes
        packet.readUInt32(); // NatureRes
        packet.readUInt32(); // FrostRes
        packet.readUInt32(); // ShadowRes
        packet.readUInt32(); // ArcaneRes
        data.delayMs = packet.readUInt32();
    }

    data.valid = !data.name.empty();
    LOG_DEBUG("[TBC] Item query: ", data.name, " quality=", data.quality,
              " invType=", data.inventoryType, " armor=", data.armor);
    return true;
}

} // namespace game
} // namespace wowee
