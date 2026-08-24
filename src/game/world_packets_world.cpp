#include "game/world_packets.hpp"
#include "game/packet_parsers.hpp"
#include "game/opcodes.hpp"
#include "game/game_utils.hpp"
#include "game/character.hpp"
#include "auth/crypto.hpp"
#include "core/logger.hpp"
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <zlib.h>


namespace wowee {
namespace game {

bool SpellGoParser::parse(network::Packet& packet, SpellGoData& data) {
    // Always reset output to avoid stale targets when callers reuse buffers.
    data = SpellGoData{};

    // Packed GUIDs are variable-length, so only require the smallest possible
    // shape up front: 2 GUID masks + fixed fields through hitCount.
    if (!packet.hasRemaining(16)) return false;

    size_t startPos = packet.getReadPos();
    if (!packet.hasFullPackedGuid()) {
        return false;
    }
    data.casterGuid = packet.readPackedGuid();
    if (!packet.hasFullPackedGuid()) {
        packet.setReadPos(startPos);
        return false;
    }
    data.casterUnit = packet.readPackedGuid();

    // Validate remaining fixed fields up to hitCount/missCount
    if (!packet.hasRemaining(14)) { // castCount(1) + spellId(4) + castFlags(4) + timestamp(4) + hitCount(1)
        packet.setReadPos(startPos);
        return false;
    }

    data.castCount = packet.readUInt8();
    data.spellId = packet.readUInt32();
    data.castFlags = packet.readUInt32();
    // Timestamp in 3.3.5a
    packet.readUInt32();

    const uint8_t rawHitCount = packet.readUInt8();
    if (rawHitCount > 128) {
        LOG_WARNING("Spell go: hitCount capped (requested=", static_cast<int>(rawHitCount), ")");
    }
    const uint8_t storedHitLimit = std::min<uint8_t>(rawHitCount, 128);

    bool truncatedTargets = false;

    data.hitTargets.reserve(storedHitLimit);
    for (uint16_t i = 0; i < rawHitCount; ++i) {
        // WotLK 3.3.5a hit targets are full uint64 GUIDs (not PackedGuid).
        if (!packet.hasRemaining(8)) {
            LOG_WARNING("Spell go: truncated hit targets at index ", i, "/", static_cast<int>(rawHitCount));
            truncatedTargets = true;
            break;
        }
        const uint64_t targetGuid = packet.readUInt64();
        if (i < storedHitLimit) {
            data.hitTargets.push_back(targetGuid);
        }
    }
    if (truncatedTargets) {
        packet.setReadPos(startPos);
        return false;
    }
    data.hitCount = static_cast<uint8_t>(data.hitTargets.size());

    // missCount is mandatory in SMSG_SPELL_GO. Missing byte means truncation.
    if (!packet.hasRemaining(1)) {
        LOG_WARNING("Spell go: missing missCount after hit target list");
        packet.setReadPos(startPos);
        return false;
    }

    const size_t missCountPos = packet.getReadPos();
    const uint8_t rawMissCount = packet.readUInt8();
    if (rawMissCount > 20) {
        // Likely offset error - dump context bytes for diagnostics.
        const auto& raw = packet.getData();
        std::string hexCtx;
        size_t dumpStart = (missCountPos >= 8) ? missCountPos - 8 : startPos;
        size_t dumpEnd = std::min(missCountPos + 16, raw.size());
        for (size_t i = dumpStart; i < dumpEnd; ++i) {
            char buf[4];
            std::snprintf(buf, sizeof(buf), "%02x ", raw[i]);
            hexCtx += buf;
            if (i == missCountPos - 1) hexCtx += "[";
            if (i == missCountPos) hexCtx += "] ";
        }
        LOG_WARNING("Spell go: suspect missCount=", static_cast<int>(rawMissCount),
                    " spell=", data.spellId, " hits=", static_cast<int>(data.hitCount),
                    " castFlags=0x", std::hex, data.castFlags, std::dec,
                    " missCountPos=", missCountPos, " pktSize=", packet.getSize(),
                    " ctx=", hexCtx);
    }
    if (rawMissCount > 128) {
        LOG_WARNING("Spell go: missCount capped (requested=", static_cast<int>(rawMissCount),
                    ") spell=", data.spellId, " hits=", static_cast<int>(data.hitCount),
                    " remaining=", packet.getRemainingSize());
    }
    const uint8_t storedMissLimit = std::min<uint8_t>(rawMissCount, 128);

    data.missTargets.reserve(storedMissLimit);
    for (uint16_t i = 0; i < rawMissCount; ++i) {
        // WotLK 3.3.5a miss targets are full uint64 GUIDs + uint8 missType.
        // REFLECT additionally appends uint8 reflectResult.
        if (!packet.hasRemaining(9)) { // 8 GUID + 1 missType
            LOG_WARNING("Spell go: truncated miss targets at index ", i, "/", static_cast<int>(rawMissCount),
                        " spell=", data.spellId, " hits=", static_cast<int>(data.hitCount));
            truncatedTargets = true;
            break;
        }
        SpellGoMissEntry m;
        m.targetGuid = packet.readUInt64();
        m.missType = packet.readUInt8();
        if (m.missType == 11) { // SPELL_MISS_REFLECT
            if (!packet.hasRemaining(1)) {
                LOG_WARNING("Spell go: truncated reflect payload at miss index ", i, "/", static_cast<int>(rawMissCount));
                truncatedTargets = true;
                break;
            }
            (void)packet.readUInt8(); // reflectResult
        }
        if (i < storedMissLimit) {
            data.missTargets.push_back(m);
        }
    }
    data.missCount = static_cast<uint8_t>(data.missTargets.size());

    // If miss targets were truncated, salvage the successfully-parsed hit data
    // rather than discarding the entire spell. The server already applied effects;
    // we just need the hit list for UI feedback (combat text, health bars).
    if (truncatedTargets) {
        LOG_DEBUG("Spell go: salvaging ", static_cast<int>(data.hitCount), " hits despite miss truncation");
        packet.skipAll(); // consume remaining bytes
        return true;
    }

    // WotLK 3.3.5a SpellCastTargets - consume ALL target payload bytes so that
    // any trailing fields after the target section are not misaligned for
    // ground-targeted or AoE spells.  Same layout as SpellStartParser.
    if (packet.hasData()) {
        if (packet.hasRemaining(4)) {
            uint32_t targetFlags = packet.readUInt32();

            auto readPackedTarget = [&](uint64_t* out) -> bool {
                if (!packet.hasFullPackedGuid()) return false;
                uint64_t g = packet.readPackedGuid();
                if (out) *out = g;
                return true;
            };
            auto skipPackedAndFloats3 = [&]() -> bool {
                if (!packet.hasFullPackedGuid()) return false;
                packet.readPackedGuid(); // transport GUID
                if (!packet.hasRemaining(12)) return false;
                packet.readFloat(); packet.readFloat(); packet.readFloat();
                return true;
            };

            // UNIT/UNIT_MINIPET/CORPSE_ALLY/GAMEOBJECT share one object target GUID
            if (targetFlags & (0x0002u | 0x0004u | 0x0400u | 0x0800u)) {
                readPackedTarget(&data.targetGuid);
            }
            // ITEM/TRADE_ITEM share one item target GUID
            if (targetFlags & (0x0010u | 0x0100u)) {
                readPackedTarget(nullptr);
            }
            // SOURCE_LOCATION: PackedGuid (transport) + float x,y,z
            if (targetFlags & 0x0020u) {
                skipPackedAndFloats3();
            }
            // DEST_LOCATION: PackedGuid (transport) + float x,y,z
            if (targetFlags & 0x0040u) {
                skipPackedAndFloats3();
            }
            // STRING: null-terminated
            if (targetFlags & 0x0200u) {
                while (packet.hasData() && packet.readUInt8() != 0) {}
            }
        }
    }

    LOG_DEBUG("Spell go: spell=", data.spellId, " hits=", static_cast<int>(data.hitCount),
             " misses=", static_cast<int>(data.missCount));
    return true;
}

bool AuraUpdateParser::parse(network::Packet& packet, AuraUpdateData& data, bool isAll) {
    // Validation: packed GUID (1-8 bytes minimum for reading)
    if (!packet.hasRemaining(1)) return false;

    data.guid = packet.readPackedGuid();

    // Cap number of aura entries to prevent unbounded loop DoS
    uint32_t maxAuras = isAll ? 512 : 1;
    uint32_t auraCount = 0;

    while (packet.hasData() && auraCount < maxAuras) {
        // Validate we can read slot (1) + spellId (4) = 5 bytes minimum
        if (!packet.hasRemaining(5)) {
            LOG_DEBUG("Aura update: truncated entry at position ", auraCount);
            break;
        }

        uint8_t slot = packet.readUInt8();
        uint32_t spellId = packet.readUInt32();
        auraCount++;

        AuraSlot aura;
        if (spellId != 0) {
            aura.spellId = spellId;

            // Validate flags + level + charges (3 bytes)
            if (!packet.hasRemaining(3)) {
                LOG_WARNING("Aura update: truncated flags/level/charges at entry ", auraCount);
                aura.flags = 0;
                aura.level = 0;
                aura.charges = 0;
            } else {
                aura.flags = packet.readUInt8();
                aura.level = packet.readUInt8();
                aura.charges = packet.readUInt8();
            }

            if (!(aura.flags & 0x08)) { // NOT_CASTER flag
                // Validate space for packed GUID read (minimum 1 byte)
                if (!packet.hasRemaining(1)) {
                    aura.casterGuid = 0;
                } else {
                    aura.casterGuid = packet.readPackedGuid();
                }
            }

            if (aura.flags & 0x20) { // DURATION - need 8 bytes (two uint32s)
                if (!packet.hasRemaining(8)) {
                    LOG_WARNING("Aura update: truncated duration fields at entry ", auraCount);
                    aura.maxDurationMs = 0;
                    aura.durationMs = 0;
                } else {
                    aura.maxDurationMs = static_cast<int32_t>(packet.readUInt32());
                    aura.durationMs = static_cast<int32_t>(packet.readUInt32());
                }
            }

            if (aura.flags & 0x40) { // EFFECT_AMOUNTS
                // Only read amounts for active effect indices (flags 0x01, 0x02, 0x04)
                for (int i = 0; i < 3; ++i) {
                    if (aura.flags & (1 << i)) {
                        if (packet.hasRemaining(4)) {
                            packet.readUInt32();
                        } else {
                            LOG_WARNING("Aura update: truncated effect amount ", i, " at entry ", auraCount);
                            break;
                        }
                    }
                }
            }
        }

        data.updates.emplace_back(slot, aura);

        // For single update, only one entry
        if (!isAll) break;
    }

    if (auraCount >= maxAuras && packet.hasData()) {
        LOG_WARNING("Aura update: capped at ", maxAuras, " entries, remaining data ignored");
    }

    LOG_DEBUG("Aura update for 0x", std::hex, data.guid, std::dec,
              ": ", data.updates.size(), " slots");
    return true;
}

bool SpellCooldownParser::parse(network::Packet& packet, SpellCooldownData& data) {
    // Upfront validation: guid(8) + flags(1) = 9 bytes minimum
    if (!packet.hasRemaining(9)) return false;

    data.guid = packet.readUInt64();
    data.flags = packet.readUInt8();

    // Cap cooldown entries to prevent unbounded memory allocation (each entry is 8 bytes)
    uint32_t maxCooldowns = 512;
    uint32_t cooldownCount = 0;

    while (packet.hasRemaining(8) && cooldownCount < maxCooldowns) {
        uint32_t spellId = packet.readUInt32();
        uint32_t cooldownMs = packet.readUInt32();
        data.cooldowns.emplace_back(spellId, cooldownMs);
        cooldownCount++;
    }

    if (cooldownCount >= maxCooldowns && packet.hasRemaining(8)) {
        LOG_WARNING("Spell cooldowns: capped at ", maxCooldowns, " entries, remaining data ignored");
    }

    LOG_DEBUG("Spell cooldowns: ", data.cooldowns.size(), " entries");
    return true;
}

// ============================================================
// Group/Party System
// ============================================================

network::Packet GroupInvitePacket::build(const std::string& playerName) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GROUP_INVITE));
    packet.writeString(playerName);
    packet.writeUInt32(0); // unused
    LOG_DEBUG("Built CMSG_GROUP_INVITE: ", playerName);
    return packet;
}

bool GroupInviteResponseParser::parse(network::Packet& packet, GroupInviteResponseData& data,
                                      bool hasCanAccept) {
    if (!packet.hasRemaining(1)) {
        LOG_WARNING("SMSG_GROUP_INVITE: packet too small (", packet.getSize(), " bytes)");
        return false;
    }

    // WotLK has a canAccept byte before the inviter name; Classic/TBC do not.
    if (hasCanAccept)
        data.canAccept = packet.readUInt8();
    else
        data.canAccept = 1;
    data.inviterName = packet.readString();
    LOG_INFO("Group invite from: ", data.inviterName, " (canAccept=", static_cast<int>(data.canAccept), ")");
    return !data.inviterName.empty();
}

network::Packet GroupAcceptPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_GROUP_ACCEPT));
    packet.writeUInt32(0); // unused in 3.3.5a
    return packet;
}

network::Packet GroupDeclinePacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_GROUP_DECLINE));
    return packet;
}

bool GroupListParser::parse(network::Packet& packet, GroupListData& data,
                            bool hasRoles, bool hasBattleGroupFlag) {
    auto rem = [&]() { return packet.getRemainingSize(); };

    if (rem() < (hasBattleGroupFlag ? 4u : 3u)) return false;
    data.groupType = packet.readUInt8();
    if (hasBattleGroupFlag) {
        packet.readUInt8(); // isBattleGroup, sent by CMaNGOS TBC before subgroup.
    }
    data.subGroup = packet.readUInt8();
    data.flags    = packet.readUInt8();

    // WotLK 3.3.5a added a roles byte (tank/healer/dps) for the dungeon finder.
    // Classic 1.12 and TBC 2.4.3 do not have this byte.
    if (hasRoles) {
        if (rem() < 1) return false;
        data.roles = packet.readUInt8();
    } else {
        data.roles = 0;
    }

    // WotLK: LFG data gated by groupType bit 0x04 (LFD group type)
    if (hasRoles && (data.groupType & 0x04)) {
        if (rem() < 5) return false;
        packet.readUInt8();  // lfg state
        packet.readUInt32(); // lfg entry
        // WotLK 3.3.5a may or may not send the lfg flags byte - read it only if present
        if (rem() >= 13) { // enough for lfgFlags(1)+groupGuid(8)+counter(4)
            packet.readUInt8(); // lfg flags
        }
    }

    if (rem() < 8) return false;
    packet.readUInt64(); // group GUID
    if (hasRoles) {
        if (rem() < 4) return false;
        packet.readUInt32(); // update counter (WotLK)
    }

    if (rem() < 4) return false;
    data.memberCount = packet.readUInt32();
    if (data.memberCount > 40) {
        LOG_WARNING("GroupListParser: implausible memberCount=", data.memberCount, ", clamping");
        data.memberCount = 40;
    }
    data.members.reserve(data.memberCount);

    for (uint32_t i = 0; i < data.memberCount; ++i) {
        if (rem() == 0) break;
        GroupMember member;
        member.name     = packet.readString();
        if (rem() < 8) break;
        member.guid     = packet.readUInt64();
        if (rem() < 3) break;
        member.isOnline = packet.readUInt8();
        member.subGroup = packet.readUInt8();
        member.flags    = packet.readUInt8();
        // WotLK added per-member roles byte; Classic/TBC do not have it.
        if (hasRoles) {
            if (rem() < 1) break;
            member.roles = packet.readUInt8();
        } else {
            member.roles = 0;
        }
        data.members.push_back(member);
    }

    if (rem() < 8) {
        LOG_INFO("Group list: ", data.memberCount, " members (no leader GUID in packet)");
        return true;
    }
    data.leaderGuid = packet.readUInt64();

    if (data.memberCount > 0 && rem() >= 10) {
        data.lootMethod   = packet.readUInt8();
        data.looterGuid   = packet.readUInt64();
        data.lootThreshold = packet.readUInt8();
        // Dungeon difficulty (heroic/normal) - Classic doesn't send this; TBC/WotLK do
        if (rem() >= 1) data.difficultyId     = packet.readUInt8();
        // Raid difficulty - WotLK only
        if (rem() >= 1) data.raidDifficultyId = packet.readUInt8();
        // Extra byte in some 3.3.5a builds
        if (hasRoles && rem() >= 1) packet.readUInt8();
    }

    LOG_INFO("Group list: ", data.memberCount, " members, leader=0x",
             std::hex, data.leaderGuid, std::dec);
    return true;
}

bool PartyCommandResultParser::parse(network::Packet& packet, PartyCommandResultData& data) {
    // Upfront validation: command(4) + name(var) + result(4) = 8 bytes minimum (plus name string)
    if (!packet.hasRemaining(8)) return false;

    data.command = static_cast<PartyCommand>(packet.readUInt32());
    data.name = packet.readString();

    // Validate result field exists (4 bytes)
    if (!packet.hasRemaining(4)) {
        data.result = static_cast<PartyResult>(0);
        return true; // Partial read is acceptable
    }

    data.result = static_cast<PartyResult>(packet.readUInt32());
    LOG_DEBUG("Party command result: ", static_cast<int>(data.result));
    return true;
}

bool GroupDeclineResponseParser::parse(network::Packet& packet, GroupDeclineData& data) {
    // Upfront validation: playerName is a CString (minimum 1 null terminator)
    if (!packet.hasRemaining(1)) return false;

    data.playerName = packet.readString();
    LOG_INFO("Group decline from: ", data.playerName);
    return true;
}

// ============================================================
// Loot System
// ============================================================

network::Packet LootPacket::build(uint64_t targetGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_LOOT));
    packet.writeUInt64(targetGuid);
    LOG_DEBUG("Built CMSG_LOOT: target=0x", std::hex, targetGuid, std::dec);
    return packet;
}

network::Packet AutostoreLootItemPacket::build(uint8_t slotIndex) {
    network::Packet packet(wireOpcode(Opcode::CMSG_AUTOSTORE_LOOT_ITEM));
    packet.writeUInt8(slotIndex);
    return packet;
}

network::Packet UseItemPacket::build(uint8_t bagIndex, uint8_t slotIndex,
                                     uint64_t itemGuid, uint32_t spellId,
                                     uint64_t targetGuid, uint64_t itemTargetGuid,
                                     uint64_t gameObjectGuid, uint32_t glyphIndex) {
    network::Packet packet(wireOpcode(Opcode::CMSG_USE_ITEM));
    packet.writeUInt8(bagIndex);
    packet.writeUInt8(slotIndex);
    packet.writeUInt8(0);  // cast count
    packet.writeUInt32(spellId); // spell id from item data
    packet.writeUInt64(itemGuid); // full 8-byte GUID
    packet.writeUInt32(glyphIndex); // the socket, when a glyph is being placed
    packet.writeUInt8(0);  // cast flags
    if (itemTargetGuid != 0) {
        // Sharpening stones, weightstones and weapon oils enchant another item:
        // the server requires the target item in SpellCastTargets.
        packet.writeUInt32(0x10); // TARGET_FLAG_ITEM
        packet.writePackedGuid(itemTargetGuid);
    } else if (gameObjectGuid != 0) {
        // Using a key on a locked chest: the item spell (OPEN_LOCK) targets a GO.
        packet.writeUInt32(0x0800); // TARGET_FLAG_GAMEOBJECT
        packet.writePackedGuid(gameObjectGuid);
    } else if (targetGuid != 0) {
        packet.writeUInt32(0x02); // TARGET_FLAG_UNIT
        packet.writePackedGuid(targetGuid);
    } else {
        packet.writeUInt32(0x00); // TARGET_FLAG_SELF
    }
    return packet;
}

network::Packet OpenItemPacket::build(uint8_t bagIndex, uint8_t slotIndex) {
    network::Packet packet(wireOpcode(Opcode::CMSG_OPEN_ITEM));
    packet.writeUInt8(bagIndex);
    packet.writeUInt8(slotIndex);
    return packet;
}

network::Packet ReadItemPacket::build(uint8_t bagIndex, uint8_t slotIndex) {
    network::Packet packet(wireOpcode(Opcode::CMSG_READ_ITEM));
    packet.writeUInt8(bagIndex);
    packet.writeUInt8(slotIndex);
    return packet;
}

network::Packet AutoEquipItemPacket::build(uint8_t srcBag, uint8_t srcSlot) {
    network::Packet packet(wireOpcode(Opcode::CMSG_AUTOEQUIP_ITEM));
    packet.writeUInt8(srcBag);
    packet.writeUInt8(srcSlot);
    return packet;
}

network::Packet SwapItemPacket::build(uint8_t dstBag, uint8_t dstSlot, uint8_t srcBag, uint8_t srcSlot) {
    network::Packet packet(wireOpcode(Opcode::CMSG_SWAP_ITEM));
    packet.writeUInt8(dstBag);
    packet.writeUInt8(dstSlot);
    packet.writeUInt8(srcBag);
    packet.writeUInt8(srcSlot);
    return packet;
}

network::Packet SplitItemPacket::build(uint8_t srcBag, uint8_t srcSlot,
                                       uint8_t dstBag, uint8_t dstSlot, uint8_t count) {
    network::Packet packet(wireOpcode(Opcode::CMSG_SPLIT_ITEM));
    packet.writeUInt8(srcBag);
    packet.writeUInt8(srcSlot);
    packet.writeUInt8(dstBag);
    packet.writeUInt8(dstSlot);
    packet.writeUInt8(count);
    return packet;
}

network::Packet SwapInvItemPacket::build(uint8_t srcSlot, uint8_t dstSlot) {
    network::Packet packet(wireOpcode(Opcode::CMSG_SWAP_INV_ITEM));
    packet.writeUInt8(srcSlot);
    packet.writeUInt8(dstSlot);
    return packet;
}

network::Packet LootMoneyPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_LOOT_MONEY));
    return packet;
}

network::Packet LootReleasePacket::build(uint64_t lootGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_LOOT_RELEASE));
    packet.writeUInt64(lootGuid);
    return packet;
}

bool LootResponseParser::parse(network::Packet& packet, LootResponseData& data, bool isWotlkFormat) {
    data = LootResponseData{};
    size_t avail = packet.getRemainingSize();

    // Minimum is guid(8)+lootType(1) = 9 bytes.  Servers send a short packet with
    // lootType=0 (LOOT_NONE) when loot is unavailable (e.g. chest not yet opened,
    // needs a key, or another player is looting).  We treat this as an empty-loot
    // signal and return false so the caller knows not to open the loot window.
    if (avail < 9) {
        LOG_WARNING("LootResponseParser: packet too short (", avail, " bytes)");
        return false;
    }

    data.lootGuid = packet.readUInt64();
    data.lootType = packet.readUInt8();

    // Short failure packet - no gold/item data follows.
    avail = packet.getRemainingSize();
    if (avail < 5) {
        // At warning, because this is the realm declining to open the loot and
        // it is the answer to "I clicked the chest and nothing happened". The
        // caller's own line is never reached from here, so without this the
        // refusal and a response that never arrived look identical in a log.
        LOG_WARNING("LootResponseParser: lootType=", static_cast<int>(data.lootType),
                    " with no loot behind it - the realm answered the use and"
                    " offered nothing, so no window opens");
        return false;
    }

    data.gold = packet.readUInt32();
    uint8_t itemCount = packet.readUInt8();

    // Per-item wire size is 22 bytes across all expansions:
    //   slot(1)+itemId(4)+count(4)+displayInfo(4)+randSuffix(4)+randProp(4)+slotType(1) = 22
    constexpr size_t kItemSize = 22u;

    auto parseLootItemList = [&](uint8_t listCount, bool markQuestItems) -> bool {
        for (uint8_t i = 0; i < listCount; ++i) {
            size_t remaining = packet.getRemainingSize();
            if (remaining < kItemSize) {
                return false;
            }

            LootItem item;
            item.slotIndex        = packet.readUInt8();
            item.itemId           = packet.readUInt32();
            item.count            = packet.readUInt32();
            item.displayInfoId    = packet.readUInt32();
            item.randomSuffix     = packet.readUInt32();
            item.randomPropertyId = packet.readUInt32();
            item.lootSlotType     = packet.readUInt8();
            item.isQuestItem      = markQuestItems;
            data.items.push_back(item);
        }
        return true;
    };

    data.items.reserve(itemCount);
    if (!parseLootItemList(itemCount, false)) {
        LOG_WARNING("LootResponseParser: truncated regular item list");
        return false;
    }

    // Quest item section only present in WotLK 3.3.5a.
    //
    // Whether it is there is decided by the expansion this client is configured
    // for, not by the realm, and those two can disagree - a 3.3.5 profile
    // pointed at a 1.12 realm reads the byte after the regular items as a quest
    // count when it is not one, and a bogus count means a list that cannot
    // parse. That used to lose the whole response, regular items and all: the
    // loot window never opened and the only trace was one warning about a
    // section the realm never sent.
    //
    // So the optional section is parsed into its own list and merged only if it
    // parses whole. Failing it costs the quest items on a realm that really
    // sent some, and keeps the ordinary loot either way, which is the better
    // side to be wrong on.
    uint8_t questItemCount = 0;
    if (isWotlkFormat && packet.hasRemaining(1)) {
        const size_t regularItems = data.items.size();
        questItemCount = packet.readUInt8();
        data.items.reserve(regularItems + questItemCount);
        if (!parseLootItemList(questItemCount, true)) {
            LOG_WARNING("LootResponseParser: the quest item section does not parse"
                        " (asked for ", static_cast<int>(questItemCount),
                        " items) - keeping the ", regularItems,
                        " regular item(s). A realm older than the expansion this"
                        " client is set to sends no such section.");
            data.items.resize(regularItems);
            questItemCount = 0;
        }
    }

    LOG_DEBUG("Loot response: ", static_cast<int>(itemCount), " regular + ", static_cast<int>(questItemCount),
             " quest items, ", data.gold, " copper");
    return true;
}

// ============================================================
// NPC Gossip
// ============================================================

network::Packet GossipHelloPacket::build(uint64_t npcGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GOSSIP_HELLO));
    packet.writeUInt64(npcGuid);
    return packet;
}

network::Packet QuestgiverHelloPacket::build(uint64_t npcGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_QUESTGIVER_HELLO));
    packet.writeUInt64(npcGuid);
    return packet;
}

network::Packet GossipSelectOptionPacket::build(uint64_t npcGuid, uint32_t menuId, uint32_t optionId, const std::string& code) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GOSSIP_SELECT_OPTION));
    packet.writeUInt64(npcGuid);
    packet.writeUInt32(menuId);
    packet.writeUInt32(optionId);
    if (!code.empty()) {
        packet.writeString(code);
    }
    return packet;
}

network::Packet QuestgiverQueryQuestPacket::build(uint64_t npcGuid, uint32_t questId) {
    network::Packet packet(wireOpcode(Opcode::CMSG_QUESTGIVER_QUERY_QUEST));
    packet.writeUInt64(npcGuid);
    packet.writeUInt32(questId);
    packet.writeUInt8(1);  // isDialogContinued = 1 (from gossip)
    return packet;
}

network::Packet QuestgiverAcceptQuestPacket::build(uint64_t npcGuid, uint32_t questId) {
    network::Packet packet(wireOpcode(Opcode::CMSG_QUESTGIVER_ACCEPT_QUEST));
    packet.writeUInt64(npcGuid);
    packet.writeUInt32(questId);
    packet.writeUInt32(0);  // AzerothCore/WotLK expects trailing unk1
    return packet;
}

bool QuestDetailsParser::parse(network::Packet& packet, QuestDetailsData& data) {
    if (packet.getSize() < 20) return false;
    data.npcGuid = packet.readUInt64();

    // WotLK has informUnit(u64) before questId; Vanilla/TBC do not.
    // Detect: try WotLK first (read informUnit + questId), then check if title
    // string looks valid. If not, rewind and try vanilla (questId directly).
    size_t preInform = packet.getReadPos();
    /*informUnit*/ packet.readUInt64();
    data.questId = packet.readUInt32();
    data.title = normalizeWowTextTokens(packet.readString());
    if (data.title.empty() || data.questId > 100000) {
        // Likely vanilla format - rewind past informUnit
        packet.setReadPos(preInform);
        data.questId = packet.readUInt32();
        data.title = normalizeWowTextTokens(packet.readString());
    }
    data.details = normalizeWowTextTokens(packet.readString());
    data.objectives = normalizeWowTextTokens(packet.readString());

    if (!packet.hasRemaining(10)) {
        LOG_DEBUG("Quest details (short): id=", data.questId, " title='", data.title, "'");
        return true;
    }

    // WotLK 3.3.5a (AzerothCore/TrinityCore) - verified against GossipDef.cpp:
    // u8 activateAccept, u32 flags, u32 suggestedPlayers, u8 isFinished, then
    // VARIABLE-count reward arrays (count == entries actually written; only
    // non-empty slots are serialized). No portrait strings - those are 4.x+.
    /*activateAccept*/ packet.readUInt8();
    data.questFlags = packet.readUInt32();
    data.suggestedPlayers = packet.readUInt32();
    /*isFinished*/ packet.readUInt8();

    // Reward choice items: count then count × (itemId, count, displayId).
    // QUEST_FLAGS_HIDDEN_REWARDS quests send counts of 0 with no entries.
    if (packet.hasRemaining(4)) {
        uint32_t choiceCount = packet.readUInt32();
        if (choiceCount > 6) choiceCount = 0; // misaligned stream guard
        for (uint32_t i = 0; i < choiceCount; i++) {
            if (!packet.hasRemaining(12)) break;
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

    // Fixed reward items: count then count × (itemId, count, displayId)
    if (packet.hasRemaining(4)) {
        uint32_t rewardCount = packet.readUInt32();
        if (rewardCount > 4) rewardCount = 0; // misaligned stream guard
        for (uint32_t i = 0; i < rewardCount; i++) {
            if (!packet.hasRemaining(12)) break;
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

    // Money and XP rewards
    if (packet.hasRemaining(4))
        data.rewardMoney = packet.readUInt32();
    if (packet.hasRemaining(4))
        data.rewardXp = packet.readUInt32();

    // Then, unconditionally in GossipDef.cpp:472 onwards -
    //
    //     uint32 honor (already multiplied by ten)
    //     float  unused, commented "honor multiplier?"
    //     uint32 reward spell        <- the icon under "You will learn:"
    //
    // The cast id and the title id come next and are left on the wire: nothing
    // reads them, and stopping here is honest about that.
    //
    // The offer panel gates its whole reward-spell row on being given one, so
    // a quest that teaches a recipe showed no sign of it. Each read is guarded
    // on its own: an older realm that stops short leaves the spell at zero
    // rather than taking a value out of the next packet, and a realm whose
    // layout differs here yields an id no spell answers to, which reads as
    // no reward spell - the same as before.
    if (packet.hasRemaining(4)) /*honor*/            packet.readUInt32();
    if (packet.hasRemaining(4)) /*honorMultiplier*/  packet.readFloat();
    if (packet.hasRemaining(4))
        data.rewardSpellId = packet.readUInt32();

    LOG_DEBUG("Quest details: id=", data.questId, " title='", data.title,
              "' rewardSpell=", data.rewardSpellId);
    return true;
}

bool GossipMessageParser::parse(network::Packet& packet, GossipMessageData& data) {
    // Upfront validation: npcGuid(8) + menuId(4) + titleTextId(4) + optionCount(4) = 20 bytes minimum
    if (!packet.hasRemaining(20)) return false;

    data.npcGuid = packet.readUInt64();
    data.menuId = packet.readUInt32();
    data.titleTextId = packet.readUInt32();
    uint32_t optionCount = packet.readUInt32();

    // Cap option count to prevent unbounded memory allocation
    const uint32_t MAX_GOSSIP_OPTIONS = 64;
    if (optionCount > MAX_GOSSIP_OPTIONS) {
        LOG_WARNING("GossipMessageParser: optionCount capped (requested=", optionCount, ")");
        optionCount = MAX_GOSSIP_OPTIONS;
    }

    data.options.clear();
    data.options.reserve(optionCount);
    for (uint32_t i = 0; i < optionCount; ++i) {
        // Each option: id(4) + icon(1) + isCoded(1) + boxMoney(4) + text(var) + boxText(var)
        // Minimum: 10 bytes + 2 empty strings (2 null terminators) = 12 bytes
        if (!packet.hasRemaining(12)) {
            LOG_WARNING("GossipMessageParser: truncated options at index ", i, "/", optionCount);
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

    // Validate questCount field exists (4 bytes)
    if (!packet.hasRemaining(4)) {
        LOG_DEBUG("Gossip: ", data.options.size(), " options (no quest data)");
        return true;
    }

    uint32_t questCount = packet.readUInt32();
    // Cap quest count to prevent unbounded memory allocation
    const uint32_t MAX_GOSSIP_QUESTS = 64;
    if (questCount > MAX_GOSSIP_QUESTS) {
        LOG_WARNING("GossipMessageParser: questCount capped (requested=", questCount, ")");
        questCount = MAX_GOSSIP_QUESTS;
    }

    data.quests.clear();
    data.quests.reserve(questCount);
    for (uint32_t i = 0; i < questCount; ++i) {
        // Each quest: questId(4) + questIcon(4) + questLevel(4) + questFlags(4) + isRepeatable(1) + title(var)
        // Minimum: 17 bytes + empty string (1 null terminator) = 18 bytes
        if (!packet.hasRemaining(18)) {
            LOG_WARNING("GossipMessageParser: truncated quests at index ", i, "/", questCount);
            break;
        }
        GossipQuestItem quest;
        quest.questId = packet.readUInt32();
        quest.questIcon = packet.readUInt32();
        quest.questLevel = static_cast<int32_t>(packet.readUInt32());
        quest.questFlags = packet.readUInt32();
        quest.isRepeatable = packet.readUInt8();
        quest.title = normalizeWowTextTokens(packet.readString());
        data.quests.push_back(quest);
    }

    LOG_DEBUG("Gossip: ", data.options.size(), " options, ", data.quests.size(), " quests");
    return true;
}

// ============================================================
// Bind Point (Hearthstone)
// ============================================================

network::Packet BinderActivatePacket::build(uint64_t npcGuid) {
    network::Packet pkt(wireOpcode(Opcode::CMSG_BINDER_ACTIVATE));
    pkt.writeUInt64(npcGuid);
    return pkt;
}

bool BindPointUpdateParser::parse(network::Packet& packet, BindPointUpdateData& data) {
    if (packet.getSize() < 20) return false;
    data.x = packet.readFloat();
    data.y = packet.readFloat();
    data.z = packet.readFloat();
    data.mapId = packet.readUInt32();
    data.zoneId = packet.readUInt32();
    return true;
}

bool QuestRequestItemsParser::parse(network::Packet& packet, QuestRequestItemsData& data) {
    if (!packet.hasRemaining(20)) return false;
    data.npcGuid = packet.readUInt64();
    data.questId = packet.readUInt32();
    data.title = normalizeWowTextTokens(packet.readString());
    data.completionText = normalizeWowTextTokens(packet.readString());

    if (!packet.hasRemaining(9)) {
        LOG_DEBUG("Quest request items (short): id=", data.questId, " title='", data.title, "'");
        return true;
    }

    struct ParsedTail {
        uint32_t requiredMoney = 0;
        uint32_t completableFlags = 0;
        std::vector<QuestRewardItem> requiredItems;
        bool ok = false;
        int score = -1;
    };

    auto parseTail = [&](size_t startPos, size_t prefixSkip) -> ParsedTail {
        ParsedTail out;
        packet.setReadPos(startPos);

        if (!packet.hasRemaining(prefixSkip)) return out;
        packet.setReadPos(packet.getReadPos() + prefixSkip);

        if (!packet.hasRemaining(8)) return out;
        out.requiredMoney = packet.readUInt32();
        uint32_t requiredItemCount = packet.readUInt32();
        if (requiredItemCount > 64) return out;  // sanity guard against misalignment

        out.requiredItems.reserve(requiredItemCount);
        for (uint32_t i = 0; i < requiredItemCount; ++i) {
            if (!packet.hasRemaining(12)) return out;
            QuestRewardItem item;
            item.itemId = packet.readUInt32();
            item.count = packet.readUInt32();
            item.displayInfoId = packet.readUInt32();
            if (item.itemId != 0) out.requiredItems.push_back(item);
        }

        if (!packet.hasRemaining(4)) return out;
        out.completableFlags = packet.readUInt32();
        out.ok = true;

        // The one part of this packet that is not a guess.
        //
        // Every server build ends this message with the three constants 0x04,
        // 0x08 and 0x10 after the completable flags, and nothing else follows
        // them. So an alignment that lands exactly on those, with the packet
        // ending there, is not merely the most plausible reading - it is the
        // right one. Scoring shapes alone chose a reading whose item count was
        // 128 and whose item id was nothing, which drew a required item as an
        // empty square with no name.
        constexpr size_t kTrailerBytes = 12;
        bool trailerMatches = false;
        if (packet.getRemainingSize() == kTrailerBytes) {
            const size_t save = packet.getReadPos();
            const uint32_t a = packet.readUInt32();
            const uint32_t b = packet.readUInt32();
            const uint32_t c = packet.readUInt32();
            trailerMatches = (a == 0x04u && b == 0x08u && c == 0x10u);
            packet.setReadPos(save);
        }

        // Prefer layouts that produce plausible quest-requirement shapes.
        out.score = 0;
        if (trailerMatches) out.score += 100;
        if (requiredItemCount <= 6) out.score += 4;
        if (out.requiredItems.size() == requiredItemCount) out.score += 3;
        if ((out.completableFlags & ~0x3u) == 0) out.score += 5;
        if (out.requiredMoney == 0) out.score += 4;
        else if (out.requiredMoney <= 100000) out.score += 2;       // <=10g is common
        else if (out.requiredMoney >= 1000000) out.score -= 3;      // implausible for most quests
        if (!out.requiredItems.empty()) out.score += 1;
        size_t remaining = packet.getRemainingSize();
        if (remaining <= 16) out.score += 3;
        else if (remaining <= 32) out.score += 2;
        else if (remaining <= 64) out.score += 1;
        if (prefixSkip == 0) out.score += 1;
        else if (prefixSkip <= 12) out.score += 1;
        return out;
    };

    size_t tailStart = packet.getReadPos();
    std::vector<ParsedTail> candidates;
    candidates.reserve(25);
    for (size_t skip = 0; skip <= 24; ++skip) {
        candidates.push_back(parseTail(tailStart, skip));
    }

    const ParsedTail* chosen = nullptr;
    for (const auto& cand : candidates) {
        if (!cand.ok) continue;
        if (!chosen || cand.score > chosen->score) chosen = &cand;
    }
    if (!chosen) {
        return true;
    }

    data.requiredMoney = chosen->requiredMoney;
    data.completableFlags = chosen->completableFlags;
    data.requiredItems = chosen->requiredItems;

    LOG_DEBUG("Quest request items: id=", data.questId, " title='", data.title,
             "' items=", data.requiredItems.size(), " completable=", data.isCompletable());
    return true;
}

bool QuestOfferRewardParser::parse(network::Packet& packet, QuestOfferRewardData& data) {
    QuestPacketEra era = !isPreWotlk() ? QuestPacketEra::WOTLK
                       : isClassicLikeExpansion() ? QuestPacketEra::CLASSIC
                                                  : QuestPacketEra::TBC;
    return parse(packet, data, era);
}

bool QuestOfferRewardParser::parse(network::Packet& packet, QuestOfferRewardData& data,
                                   QuestPacketEra era) {
    if (!packet.hasRemaining(20)) return false;
    data.npcGuid = packet.readUInt64();
    data.questId = packet.readUInt32();
    data.title = normalizeWowTextTokens(packet.readString());
    data.rewardText = normalizeWowTextTokens(packet.readString());

    if (!packet.hasRemaining(8)) {
        LOG_DEBUG("Quest offer reward (short): id=", data.questId, " title='", data.title, "'");
        return true;
    }

    // WotLK 3.3.5a (AzerothCore/TrinityCore) - verified against GossipDef.cpp:
    // u8 autoFinish + u32 flags + u32 suggestedPlayers + emotes +
    // VARIABLE-count choice/reward arrays (count == entries written) +
    // money + xp + trailing (honor, spells, title, talents, reputation arrays).
    // No portrait strings - those are 4.x+.
    if (era == QuestPacketEra::WOTLK) {
        if (!packet.hasRemaining(9)) return true;
        packet.readUInt8();  // autoFinish
        packet.readUInt32(); // questFlags
        packet.readUInt32(); // suggestedPlayers

        if (!packet.hasRemaining(4)) return true;
        uint32_t emoteCount = packet.readUInt32();
        if (emoteCount > 32) return true;
        for (uint32_t i = 0; i < emoteCount; ++i) {
            if (!packet.hasRemaining(8)) return true;
            packet.readUInt32(); // delay
            packet.readUInt32(); // emote
        }

        if (!packet.hasRemaining(4)) return true;
        uint32_t choiceCount = packet.readUInt32();
        if (choiceCount > 6) return true;
        for (uint32_t i = 0; i < choiceCount; ++i) {
            if (!packet.hasRemaining(12)) return true;
            QuestRewardItem item;
            item.itemId = packet.readUInt32();
            item.count = packet.readUInt32();
            item.displayInfoId = packet.readUInt32();
            item.choiceSlot = i;
            if (item.itemId > 0)
                data.choiceRewards.push_back(item);
        }

        if (!packet.hasRemaining(4)) return true;
        uint32_t rewardCount = packet.readUInt32();
        if (rewardCount > 4) return true;
        for (uint32_t i = 0; i < rewardCount; ++i) {
            if (!packet.hasRemaining(12)) return true;
            QuestRewardItem item;
            item.itemId = packet.readUInt32();
            item.count = packet.readUInt32();
            item.displayInfoId = packet.readUInt32();
            if (item.itemId > 0)
                data.fixedRewards.push_back(item);
        }

        if (packet.hasRemaining(4))
            data.rewardMoney = packet.readUInt32();
        if (packet.hasRemaining(4))
            data.rewardXp = packet.readUInt32();
        // The trailing block, in the wire's order (GossipDef.cpp): honour,
        // an honour-multiplier float, an unused word, the reward spell and its
        // cast id, the title id, then bonus talents and arena points. The
        // honour is sent multiplied by ten and the panel shows honour points,
        // so unscale it. Everything between is stepped over; each read is
        // guarded so a short/older packet leaves the fields at zero rather than
        // reading into the next packet.
        if (packet.hasRemaining(4)) data.rewardHonor = packet.readUInt32() / 10u;
        if (packet.hasRemaining(4)) packet.readFloat();   // honour multiplier
        if (packet.hasRemaining(4)) packet.readUInt32();  // unused
        if (packet.hasRemaining(4)) packet.readUInt32();  // reward spell
        if (packet.hasRemaining(4)) packet.readUInt32();  // reward spell cast
        if (packet.hasRemaining(4)) data.rewardTitleId = packet.readUInt32();
        if (packet.hasRemaining(4)) data.rewardTalents = packet.readUInt32();
        if (packet.hasRemaining(4)) data.rewardArenaPoints = packet.readUInt32();
        // An unused word, then the three reputation arrays as in the query
        // response: five faction ids, five value indices, five overrides. Read
        // as a block so a short packet leaves them all zero.
        if (packet.hasRemaining(4)) packet.readUInt32();  // unused
        if (packet.hasRemaining(60)) {  // 15 × uint32
            for (auto& fr : data.factionRewards) fr.factionId = packet.readUInt32();
            for (auto& fr : data.factionRewards) fr.valueId = static_cast<int32_t>(packet.readUInt32());
            for (auto& fr : data.factionRewards) fr.override = static_cast<int32_t>(packet.readUInt32());
        }

        LOG_INFO("Quest offer reward: id=", data.questId, " title='", data.title,
                 "' choices=", data.choiceRewards.size(), " fixed=", data.fixedRewards.size(),
                 " money=", data.rewardMoney, " xp=", data.rewardXp);
        for (const auto& ri : data.choiceRewards)
            LOG_INFO("  choice: itemId=", ri.itemId, " count=", ri.count,
                     " displayId=", ri.displayInfoId, " slot=", ri.choiceSlot);
        for (const auto& ri : data.fixedRewards)
            LOG_INFO("  fixed: itemId=", ri.itemId, " count=", ri.count,
                     " displayId=", ri.displayInfoId);
        return true;
    }

    // Classic/TBC - verified against vmangos / cmangos-tbc GossipDef.cpp:
    //   Classic 1.12 : u32 autoFinish                          (4-byte prefix)
    //   TBC 2.4.3    : u32 autoFinish + u32 suggestedPlayers   (8-byte prefix)
    // then emoteCount + count×(delay,emote), choiceCount + count×(id,count,display),
    // rewardCount + count×(id,count,display), money, and a small trailing block
    // (Classic: flags+spell = 8 bytes; TBC: honor+0x08+spell+spellCast+title = 20).
    // Parse that layout deterministically; fall back to the prefix scanner only
    // if it doesn't validate (custom vanilla-family servers alter the prefix).

    struct ParsedTail {
        uint32_t rewardMoney = 0;
        uint32_t rewardXp = 0;
        std::vector<QuestRewardItem> choiceRewards;
        std::vector<QuestRewardItem> fixedRewards;
        bool ok = false;
        int score = -1000;
        size_t prefixSkip = 0;
        bool fixedArrays = false;
    };

    auto parseTail = [&](size_t startPos, size_t prefixSkip, bool fixedArrays) -> ParsedTail {
        ParsedTail out;
        out.prefixSkip  = prefixSkip;
        out.fixedArrays = fixedArrays;
        packet.setReadPos(startPos);

        if (!packet.hasRemaining(prefixSkip)) return out;
        packet.setReadPos(packet.getReadPos() + prefixSkip);

        if (!packet.hasRemaining(4)) return out;
        uint32_t emoteCount = packet.readUInt32();
        if (emoteCount > 32) return out;
        for (uint32_t i = 0; i < emoteCount; ++i) {
            if (!packet.hasRemaining(8)) return out;
            packet.readUInt32(); // delay
            packet.readUInt32(); // emote type
        }

        if (!packet.hasRemaining(4)) return out;
        uint32_t choiceCount = packet.readUInt32();
        if (choiceCount > 6) return out;
        uint32_t choiceSlots = fixedArrays ? 6u : choiceCount;
        out.choiceRewards.reserve(choiceCount);
        uint32_t nonZeroChoice = 0;
        for (uint32_t i = 0; i < choiceSlots; ++i) {
            if (!packet.hasRemaining(12)) return out;
            QuestRewardItem item;
            item.itemId = packet.readUInt32();
            item.count = packet.readUInt32();
            item.displayInfoId = packet.readUInt32();
            item.choiceSlot = i;
            if (item.itemId > 0) {
                out.choiceRewards.push_back(item);
                ++nonZeroChoice;
            }
        }

        if (!packet.hasRemaining(4)) return out;
        uint32_t rewardCount = packet.readUInt32();
        if (rewardCount > 4) return out;
        uint32_t rewardSlots = fixedArrays ? 4u : rewardCount;
        out.fixedRewards.reserve(rewardCount);
        uint32_t nonZeroFixed = 0;
        for (uint32_t i = 0; i < rewardSlots; ++i) {
            if (!packet.hasRemaining(12)) return out;
            QuestRewardItem item;
            item.itemId = packet.readUInt32();
            item.count = packet.readUInt32();
            item.displayInfoId = packet.readUInt32();
            if (item.itemId > 0) {
                out.fixedRewards.push_back(item);
                ++nonZeroFixed;
            }
        }

        if (packet.hasRemaining(4))
            out.rewardMoney = packet.readUInt32();
        if (packet.hasRemaining(4))
            out.rewardXp = packet.readUInt32();

        out.ok = true;
        out.score = 0;
        if (prefixSkip == 4 || prefixSkip == 8) out.score += 3;
        else if (prefixSkip == 5) out.score += 1;
        if (choiceCount <= 6) out.score += 3;
        if (rewardCount <= 4) out.score += 3;
        if (nonZeroChoice <= choiceCount) out.score += 2;
        if (nonZeroFixed <= rewardCount) out.score += 2;
        for (const auto& ri : out.choiceRewards) {
            if (ri.itemId > 0 && ri.itemId < 100000 && ri.count > 0 && ri.count < 1000 && ri.displayInfoId > 0)
                out.score += 2;
            else if (ri.itemId >= 100000)
                out.score -= 2;
        }
        for (const auto& ri : out.fixedRewards) {
            if (ri.itemId > 0 && ri.itemId < 100000 && ri.count > 0 && ri.count < 1000 && ri.displayInfoId > 0)
                out.score += 2;
            else if (ri.itemId >= 100000)
                out.score -= 2;
        }
        size_t remaining = packet.getRemainingSize();
        if (remaining == 0) out.score += 5;
        else if (remaining <= 4) out.score += 3;
        else if (remaining <= 8) out.score += 2;
        else if (remaining <= 16) out.score += 1;
        else out.score -= static_cast<int>(remaining / 4);
        if (out.rewardMoney < 5000000u) out.score += 1;
        if (out.rewardXp < 200000u) out.score += 1;
        return out;
    };

    size_t tailStart = packet.getReadPos();

    // Deterministic first: the documented layout for the active expansion.
    const size_t knownPrefix = (era == QuestPacketEra::CLASSIC) ? 4u : 8u;
    ParsedTail direct = parseTail(tailStart, knownPrefix, false);
    size_t directRemaining = packet.getRemainingSize(); // pos is at end of the direct parse
    bool directValid = direct.ok && directRemaining <= 24;
    if (directValid) {
        for (const auto& ri : direct.choiceRewards)
            if (ri.itemId >= 0x01000000u || ri.count > 1000) { directValid = false; break; }
        for (const auto& ri : direct.fixedRewards)
            if (ri.itemId >= 0x01000000u || ri.count > 1000) { directValid = false; break; }
    }

    const ParsedTail* best = nullptr;
    std::vector<ParsedTail> candidates;
    if (directValid) {
        best = &direct;
    } else {
        candidates.reserve(34);
        for (size_t skip = 0; skip <= 16; ++skip) {
            candidates.push_back(parseTail(tailStart, skip, true));
            candidates.push_back(parseTail(tailStart, skip, false));
        }
        for (const auto& cand : candidates) {
            if (!cand.ok) continue;
            if (!best || cand.score > best->score) best = &cand;
        }
    }

    if (best) {
        data.choiceRewards = best->choiceRewards;
        data.fixedRewards  = best->fixedRewards;
        data.rewardMoney   = best->rewardMoney;
        // Pre-WotLK servers do not send an XP field here; parseTail's trailing
        // read lands on flags (Classic) or honor (TBC). Never surface it as XP.
        data.rewardXp      = 0;
    }

    LOG_INFO("Quest offer reward: id=", data.questId, " title='", data.title,
             "' choices=", data.choiceRewards.size(), " fixed=", data.fixedRewards.size(),
             " money=", data.rewardMoney, " xp=", data.rewardXp,
             " prefix=", (best ? best->prefixSkip : size_t(0)),
             " score=", (best ? best->score : -1),
             (best && best->fixedArrays ? " fixed" : " var"));
    for (const auto& ri : data.choiceRewards)
        LOG_INFO("  choice: itemId=", ri.itemId, " count=", ri.count,
                 " displayId=", ri.displayInfoId, " slot=", ri.choiceSlot);
    for (const auto& ri : data.fixedRewards)
        LOG_INFO("  fixed: itemId=", ri.itemId, " count=", ri.count,
                 " displayId=", ri.displayInfoId);
    return true;
}

QuestQueryRewardsData QuestQueryRewardsParser::parse(const std::vector<uint8_t>& data,
                                                     uint8_t questLogStride) {
    // Reward arrays are ALWAYS interleaved (itemId, count) pairs, present even
    // for QUEST_FLAGS_HIDDEN_REWARDS (written as zero pairs). Field indices are
    // relative to base=8 (questId + questMethod already skipped), i.e. absolute
    // field - 2. Verified against the server serializers:
    //   Classic  (vmangos Quest.cpp):      money=abs10, pairs at abs15, choice abs23
    //   TBC      (cmangos-tbc):            money=abs11, pairs at abs19, choice abs27
    //   WotLK    (AzerothCore GossipDef):  money=abs13, pairs at abs26, choice abs34
    const size_t base = 8;
    size_t moneyField, rewardPairsField, choicePairsField;
    // The quest's start item. Counted from the same base, at abs19 for WotLK:
    // money(13), maxLevel(14), rewSpell(15), rewSpellCast(16), honorAddition(17),
    // honorMultiplier(18), srcItemId(19) - which is 24 minus the four fields
    // between it and the reward pairs, and lands exactly where the pairs start
    // counting from. Left at zero for the earlier layouts: their field order
    // between the money and the pairs has not been read off a serializer here,
    // and a guess would put an arbitrary item on the watch frame.
    size_t sourceItemField = 0;
    size_t rewardSpellField = 0;
    size_t xpIdField = 0, honorField = 0, talentsField = 0, arenaField = 0, titleField = 0;
    size_t repFactionField = 0, repValueField = 0, repOverrideField = 0;
    if (questLogStride >= 5) {        // WotLK
        moneyField = 11; rewardPairsField = 24; choicePairsField = 32;
        sourceItemField = 17;
        // XPId sits immediately before the money field in the serializer
        // (nextQuestInChain, XPId, RewOrReqMoney), so field 10 here - one below
        // money's 11. Read only where the layout is confirmed, like the spell
        // and start item above.
        xpIdField = 10;
        // Counted off the same serializer: RewHonorAddition at 15 (two past the
        // spell), CharTitleId at 19 (two past the start item), BonusTalents at
        // 21 and RewArenaPoints at 22 (three and two before the reward pairs).
        honorField = 15; titleField = 19; talentsField = 21; arenaField = 22;
        // The three reputation arrays follow the choice pairs (which end at 44):
        // RewardFactionId[5] at 44, RewardFactionValueId[5] at 49, and the
        // override[5] at 54.
        repFactionField = 44; repValueField = 49; repOverrideField = 54;
        // Counted from the same base as the four above: money is absolute 13
        // and the start item absolute 19, and the serializer writes
        // money(13), maxLevel(14), rewSpell(15) - so 13 here, which is 15 there.
        rewardSpellField = 13;
    } else if (questLogStride == 4) { // TBC
        moneyField = 9;  rewardPairsField = 17; choicePairsField = 25;
    } else {                          // Classic / Turtle
        moneyField = 8;  rewardPairsField = 13; choicePairsField = 21;
    }
    const size_t lastField = choicePairsField + 6 * 2; // exclusive end of choice pairs
    if (data.size() < base + lastField * 4u) return {};

    auto readU32At = [&data](size_t pos) -> uint32_t {
        return static_cast<uint32_t>(data[pos])
             | (static_cast<uint32_t>(data[pos + 1]) << 8)
             | (static_cast<uint32_t>(data[pos + 2]) << 16)
             | (static_cast<uint32_t>(data[pos + 3]) << 24);
    };

    QuestQueryRewardsData out;
    out.rewardMoney = static_cast<int32_t>(readU32At(base + moneyField * 4u));
    for (size_t i = 0; i < 4; ++i) {
        out.itemId[i]    = readU32At(base + (rewardPairsField + i * 2) * 4u);
        out.itemCount[i] = readU32At(base + (rewardPairsField + i * 2 + 1) * 4u);
    }
    for (size_t i = 0; i < 6; ++i) {
        out.choiceItemId[i]    = readU32At(base + (choicePairsField + i * 2) * 4u);
        out.choiceItemCount[i] = readU32At(base + (choicePairsField + i * 2 + 1) * 4u);
    }
    // Plausibility gate: a layout mismatch lands in string data or floats and
    // produces absurd ids/counts - better to keep no reward data than garbage.
    for (size_t i = 0; i < 4; ++i)
        if (out.itemId[i] > 0x00FFFFFFu || out.itemCount[i] > 0xFFFFu) return {};
    for (size_t i = 0; i < 6; ++i)
        if (out.choiceItemId[i] > 0x00FFFFFFu || out.choiceItemCount[i] > 0xFFFFu) return {};
    if (rewardSpellField) {
        const uint32_t spell = readU32At(base + rewardSpellField * 4u);
        // The same plausibility gate the items get: a layout that has slipped
        // lands in float or string data and answers something enormous.
        if (spell <= 0x000FFFFFu) out.rewardSpellId = spell;
    }
    if (xpIdField) {
        // QuestXP.dbc has ten difficulty columns (indices 0..9), so a valid index
        // is small; a slipped layout lands on an id or count and is far larger.
        const uint32_t xp = readU32At(base + xpIdField * 4u);
        if (xp <= 9u) out.xpId = xp;
    }
    // Honor, talents and arena points are direct amounts. A slipped layout lands
    // on an item id or a float and reads as something enormous, so gate each on
    // a generous ceiling - real quest honor is in the hundreds, talents are one
    // or two, arena points a few hundred.
    if (honorField) {
        const uint32_t h = readU32At(base + honorField * 4u);
        if (h <= 100000u) out.rewardHonor = h;
    }
    if (talentsField) {
        const uint32_t t = readU32At(base + talentsField * 4u);
        if (t <= 20u) out.bonusTalents = t;
    }
    if (arenaField) {
        const uint32_t a = readU32At(base + arenaField * 4u);
        if (a <= 100000u) out.arenaPoints = a;
    }
    if (titleField) {
        // A CharTitles.dbc id - a few hundred rows, so small; a slipped layout
        // reads far larger and is dropped.
        const uint32_t t = readU32At(base + titleField * 4u);
        if (t <= 500u) out.rewardTitleId = t;
    }
    // The three reputation arrays, once the packet is long enough to hold the
    // last override at field repOverrideField+4. Values are signed (a quest can
    // cost reputation), so no plausibility gate beyond the length check.
    if (repFactionField && data.size() >= base + (repOverrideField + 5) * 4u) {
        for (size_t i = 0; i < 5; ++i) {
            out.factionId[i]           = readU32At(base + (repFactionField + i) * 4u);
            out.factionValueId[i]      = static_cast<int32_t>(readU32At(base + (repValueField + i) * 4u));
            out.factionValueOverride[i]= static_cast<int32_t>(readU32At(base + (repOverrideField + i) * 4u));
        }
    }
    if (sourceItemField) {
        const uint32_t srcItem = readU32At(base + sourceItemField * 4u);
        // Same plausibility gate the rewards use: a layout mismatch lands in a
        // float or a string and reads as an enormous item id.
        if (srcItem <= 0x00FFFFFFu) out.sourceItemId = srcItem;
    }
    out.valid = true;
    return out;
}

network::Packet QuestgiverCompleteQuestPacket::build(uint64_t npcGuid, uint32_t questId) {
    network::Packet packet(wireOpcode(Opcode::CMSG_QUESTGIVER_COMPLETE_QUEST));
    packet.writeUInt64(npcGuid);
    packet.writeUInt32(questId);
    return packet;
}

network::Packet QuestgiverRequestRewardPacket::build(uint64_t npcGuid, uint32_t questId) {
    network::Packet packet(wireOpcode(Opcode::CMSG_QUESTGIVER_REQUEST_REWARD));
    packet.writeUInt64(npcGuid);
    packet.writeUInt32(questId);
    return packet;
}

network::Packet QuestgiverChooseRewardPacket::build(uint64_t npcGuid, uint32_t questId, uint32_t rewardIndex) {
    network::Packet packet(wireOpcode(Opcode::CMSG_QUESTGIVER_CHOOSE_REWARD));
    packet.writeUInt64(npcGuid);
    packet.writeUInt32(questId);
    packet.writeUInt32(rewardIndex);
    return packet;
}

// ============================================================
// Vendor
// ============================================================

network::Packet ListInventoryPacket::build(uint64_t npcGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_LIST_INVENTORY));
    packet.writeUInt64(npcGuid);
    return packet;
}

network::Packet BuyItemPacket::build(uint64_t vendorGuid, uint32_t itemId, uint32_t slot, uint32_t count) {
    network::Packet packet(wireOpcode(Opcode::CMSG_BUY_ITEM));
    packet.writeUInt64(vendorGuid);
    packet.writeUInt32(itemId);  // item entry
    packet.writeUInt32(slot);    // vendor slot index from SMSG_LIST_INVENTORY
    packet.writeUInt32(count);
    // Note: WotLK/AzerothCore expects a trailing byte; Classic/TBC do not.
    // This static helper always adds it (appropriate for CMaNGOS/AzerothCore).
    // For Classic/TBC, use the GameHandler::buyItem() path which checks expansion.
    packet.writeUInt8(0);
    return packet;
}

network::Packet SellItemPacket::build(uint64_t vendorGuid, uint64_t itemGuid, uint32_t count) {
    network::Packet packet(wireOpcode(Opcode::CMSG_SELL_ITEM));
    packet.writeUInt64(vendorGuid);
    packet.writeUInt64(itemGuid);
    packet.writeUInt32(count);
    return packet;
}

network::Packet BuybackItemPacket::build(uint64_t vendorGuid, uint32_t slot) {
    network::Packet packet(wireOpcode(Opcode::CMSG_BUYBACK_ITEM));
    packet.writeUInt64(vendorGuid);
    packet.writeUInt32(slot);
    return packet;
}

bool ListInventoryParser::parse(network::Packet& packet, ListInventoryData& data) {
    // Preserve canRepair - it was set by the gossip handler before this packet
    // arrived and is not part of the wire format.
    const bool savedCanRepair = data.canRepair;
    data = ListInventoryData{};
    data.canRepair = savedCanRepair;

    if (!packet.hasRemaining(9)) {
        LOG_WARNING("ListInventoryParser: packet too short");
        return false;
    }

    data.vendorGuid = packet.readUInt64();
    uint8_t itemCount = packet.readUInt8();

    if (itemCount == 0) {
        LOG_INFO("Vendor has nothing for sale");
        return true;
    }

    // Auto-detect whether server sends 7 fields (28 bytes/item) or 8 fields (32 bytes/item).
    // Some servers omit the extendedCost field entirely; reading 8 fields on a 7-field packet
    // misaligns every item after the first and produces garbage prices.
    size_t remaining = packet.getRemainingSize();
    const size_t bytesPerItemNoExt = 28;
    const size_t bytesPerItemWithExt = 32;
    bool hasExtendedCost = false;
    if (remaining < static_cast<size_t>(itemCount) * bytesPerItemNoExt) {
        LOG_WARNING("ListInventoryParser: truncated packet (items=", static_cast<int>(itemCount),
                    ", remaining=", remaining, ")");
        return false;
    }
    if (remaining >= static_cast<size_t>(itemCount) * bytesPerItemWithExt) {
        hasExtendedCost = true;
    }

    data.items.reserve(itemCount);
    for (uint8_t i = 0; i < itemCount; ++i) {
        const size_t perItemBytes = hasExtendedCost ? bytesPerItemWithExt : bytesPerItemNoExt;
        if (!packet.hasRemaining(perItemBytes)) {
            LOG_WARNING("ListInventoryParser: item ", static_cast<int>(i), " truncated");
            return false;
        }
        VendorItem item;
        item.slot = packet.readUInt32();
        item.itemId = packet.readUInt32();
        item.displayInfoId = packet.readUInt32();
        item.maxCount = static_cast<int32_t>(packet.readUInt32());
        item.buyPrice = packet.readUInt32();
        item.durability = packet.readUInt32();
        item.stackCount = packet.readUInt32();
        item.extendedCost = hasExtendedCost ? packet.readUInt32() : 0;
        data.items.push_back(item);
    }

    LOG_DEBUG("Vendor inventory: ", static_cast<int>(itemCount), " items (extendedCost: ", hasExtendedCost ? "yes" : "no", ")");
    return true;
}

// ============================================================
// Trainer
// ============================================================

bool TrainerListParser::parse(network::Packet& packet, TrainerListData& data, bool isClassic) {
    // WotLK per-entry: spellId(4) + state(1) + cost(4) + profDialog(4) + profButton(4) +
    //                  reqLevel(1) + reqSkill(4) + reqSkillValue(4) + chain×3(12) = 38 bytes
    // Classic per-entry: spellId(4) + state(1) + cost(4) + reqLevel(1) +
    //                    reqSkill(4) + reqSkillValue(4) + chain×3(12) + unk(4) = 34 bytes
    data = TrainerListData{};
    if (!packet.hasRemaining(16)) return false; // guid(8) + type(4) + count(4)

    data.trainerGuid = packet.readUInt64();
    data.trainerType = packet.readUInt32();
    uint32_t spellCount = packet.readUInt32();

    if (spellCount > 1000) {
        LOG_ERROR("TrainerListParser: unreasonable spell count ", spellCount);
        return false;
    }

    data.spells.reserve(spellCount);
    for (uint32_t i = 0; i < spellCount; ++i) {
        // Validate minimum entry size before reading
        const size_t minEntrySize = isClassic ? 34 : 38;
        if (!packet.hasRemaining(minEntrySize)) {
            LOG_WARNING("TrainerListParser: truncated at spell ", i);
            break;
        }

        TrainerSpell spell;
        spell.spellId   = packet.readUInt32();
        spell.state     = packet.readUInt8();
        spell.spellCost = packet.readUInt32();
        if (isClassic) {
            // Classic 1.12: reqLevel immediately after cost; no profDialog/profButton
            spell.profDialog = 0;
            spell.profButton = 0;
            spell.reqLevel   = packet.readUInt8();
        } else {
            // TBC / WotLK: profDialog + profButton before reqLevel
            spell.profDialog = packet.readUInt32();
            spell.profButton = packet.readUInt32();
            spell.reqLevel   = packet.readUInt8();
        }
        spell.reqSkill      = packet.readUInt32();
        spell.reqSkillValue = packet.readUInt32();
        spell.chainNode1    = packet.readUInt32();
        spell.chainNode2    = packet.readUInt32();
        spell.chainNode3    = packet.readUInt32();
        if (isClassic) {
            packet.readUInt32(); // trailing unk / sort index
        }
        data.spells.push_back(spell);
    }

    if (!packet.hasData()) {
        LOG_WARNING("TrainerListParser: truncated before greeting");
        data.greeting.clear();
    } else {
        data.greeting = packet.readString();
    }

    LOG_INFO("Trainer list (", isClassic ? "Classic" : "TBC/WotLK", "): ",
             spellCount, " spells, type=", data.trainerType,
             ", greeting=\"", data.greeting, "\"");
    return true;
}

network::Packet TrainerBuySpellPacket::build(uint64_t trainerGuid, uint32_t spellId) {
    network::Packet packet(wireOpcode(Opcode::CMSG_TRAINER_BUY_SPELL));
    packet.writeUInt64(trainerGuid);
    packet.writeUInt32(spellId);
    return packet;
}

// ============================================================
// Talents
// ============================================================

network::Packet LearnTalentPacket::build(uint32_t talentId, uint32_t requestedRank) {
    network::Packet packet(wireOpcode(Opcode::CMSG_LEARN_TALENT));
    packet.writeUInt32(talentId);
    // Counted from zero on the wire, from one everywhere above it. The field is
    // the index into TalentEntry::RankID, and the server reads it straight out
    // of the packet: Player::LearnTalent spends
    // `talentRank - currentTalentRank + 1` points, so the first rank of an
    // untrained talent has to arrive as 0.
    //
    // Sent as 1 it reads as the SECOND rank - two points for a player holding
    // one, and a rank whose predecessor is missing - so the handler returned
    // without a word and answered the unchanged talents. That is exactly what
    // learning a talent looked like: a confirmation, the staged point handed
    // straight back, and nothing learned.
    packet.writeUInt32(requestedRank > 0 ? requestedRank - 1 : 0);
    return packet;
}

bool BattlefieldStatusPacket::parse(network::Packet& packet, BattlefieldStatusData& data,
                                    bool classicFormat, bool wotlkFormat) {
    if (!packet.hasRemaining(4)) return false;
    data.queueSlot = packet.readUInt32();

    // The server writes the next stretch as one uint64 and says so in a comment,
    // which is what makes it easy to read as fewer, wider fields than it holds:
    //
    //     uint8 arenaType, uint8 isArena, uint32 bgTypeId, uint16 0x1F90,
    //     uint8 minLevel, uint8 maxLevel, uint32 instanceId, uint8 isRated
    //
    // Read as `uint32 instanceId, uint8 isRated` - four bytes and one where the
    // server sends seven - every field after it lands two bytes early, and the
    // status arrives as the top half of the instance id joined to the bottom
    // half of the status. The packet is the same length either way, so nothing
    // runs short and nothing reports anything; the status is simply never one
    // of the values it is compared against, and a queue that exists reads as no
    // queue at all.
    if (!classicFormat) {
        if (!packet.hasRemaining(2)) return false;
        data.arenaType = packet.readUInt8();
        packet.readUInt8();                  // 0xE for an arena, 0 otherwise
    }
    if (!packet.hasRemaining(6)) return false;
    data.bgTypeId = packet.readUInt32();
    packet.readUInt16();                     // always 0x1F90
    if (!packet.hasRemaining(7)) return false;
    data.minLevel = packet.readUInt8();
    data.maxLevel = packet.readUInt8();
    data.instanceId = packet.readUInt32();
    data.isRated = packet.readUInt8() != 0;
    if (!packet.hasRemaining(4)) return false;
    data.statusId = packet.readUInt32();

    // What follows depends on the status. Each field is guarded on its own, so
    // a status whose tail is shorter than expected leaves the rest zero rather
    // than reading past the end.
    switch (data.statusId) {
        case 1:  // queued
            if (packet.hasRemaining(4)) data.avgWaitMs = packet.readUInt32();
            if (packet.hasRemaining(4)) data.timeInQueueMs = packet.readUInt32();
            break;
        case 2:  // invited
        case 3:  // in progress
            if (packet.hasRemaining(4)) data.mapId = packet.readUInt32();
            // Eight bytes 3.3.5 added and does not use. The server's own
            // comment marks them as its own version's, so they are not read
            // before Wrath.
            if (wotlkFormat && packet.hasRemaining(8)) {
                packet.readUInt32();
                packet.readUInt32();
            }
            // For an invitation this is how long it lasts, which is the
            // countdown on the accept dialog. Read from the first field after
            // the type before this, which is the map id - so the dialog counted
            // down from a map number.
            if (packet.hasRemaining(4)) data.inviteTimeoutMs = packet.readUInt32();
            break;
        default:
            break;
    }
    return true;
}

network::Packet LfgJoinPacket::build(const std::vector<uint32_t>& dungeonIds,
                                    uint32_t roles,
                                    const std::string& comment) {
    network::Packet packet(wireOpcode(Opcode::CMSG_LFG_JOIN));
    // The order and the widths the server reads, which this had wrong in every
    // field: roles went out as one byte rather than four, and the three needs
    // between the slots and the comment were missing entirely. The four bytes
    // taken for Roles therefore swallowed the roles, both flags and the slot
    // count, the slot list came out empty, and HandleLfgJoinOpcode returns
    // without a word when it is - so the queue was discarded in silence.
    packet.writeUInt32(roles);
    packet.writeUInt8(0);                    // NoPartialClear
    packet.writeUInt8(0);                    // Achievements
    packet.writeUInt8(static_cast<uint8_t>(dungeonIds.size()));
    for (uint32_t id : dungeonIds) packet.writeUInt32(id);
    packet.writeUInt8(3);                    // Needs count, always three
    packet.writeUInt8(0);
    packet.writeUInt8(0);
    packet.writeUInt8(0);
    packet.writeString(comment);
    return packet;
}

network::Packet TalentWipeConfirmPacket::build(bool accept) {
    network::Packet packet(wireOpcode(Opcode::MSG_TALENT_WIPE_CONFIRM));
    packet.writeUInt32(accept ? 1 : 0);
    return packet;
}

// ============================================================
// Death/Respawn
// ============================================================

network::Packet RepopRequestPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_REPOP_REQUEST));
    packet.writeUInt8(1);  // request release (1 = manual)
    return packet;
}

network::Packet ReclaimCorpsePacket::build(uint64_t guid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_RECLAIM_CORPSE));
    packet.writeUInt64(guid);
    return packet;
}

network::Packet SpiritHealerActivatePacket::build(uint64_t npcGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_SPIRIT_HEALER_ACTIVATE));
    packet.writeUInt64(npcGuid);
    return packet;
}

network::Packet ResurrectResponsePacket::build(uint64_t casterGuid, bool accept) {
    network::Packet packet(wireOpcode(Opcode::CMSG_RESURRECT_RESPONSE));
    packet.writeUInt64(casterGuid);
    packet.writeUInt8(accept ? 1 : 0);
    return packet;
}

// ============================================================
// Taxi / Flight Paths
// ============================================================

} // namespace game
} // namespace wowee
