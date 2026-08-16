#include "game/group_defines.hpp"
#include "core/local_time.hpp"
#include "game/item_text.hpp"
#include "game/social_handler.hpp"
#include "addons/lua_api_registrations.hpp"
#include "ui/framexml_takeover.hpp"
#include "game/game_handler.hpp"
#include "game/game_utils.hpp"
#include "game/entity.hpp"
#include "game/packet_parsers.hpp"
#include "game/update_field_table.hpp"
#include "game/opcode_table.hpp"
#include "audio/audio_coordinator.hpp"
#include "audio/ui_sound_manager.hpp"
#include "network/world_socket.hpp"
#include "rendering/renderer.hpp"
#include "core/logger.hpp"
#include "core/application.hpp"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <vector>

namespace wowee {
namespace game {

namespace {

std::filesystem::path guildNameCachePath() {
#ifdef _WIN32
    if (const char* appData = std::getenv("APPDATA")) {
        return std::filesystem::path(appData) / "wowee" / "guild_names.tsv";
    }
#else
    if (const char* home = std::getenv("HOME")) {
        return std::filesystem::path(home) / ".wowee" / "guild_names.tsv";
    }
#endif
    return std::filesystem::path("guild_names.tsv");
}

} // namespace




// LFG join result codes from LFGJoinResult enum (WotLK 3.3.5a).
// Case 0 = success (no error message needed), returns nullptr so the caller
// knows not to display an error string.
// A uint32 on the wire, so a uint32 here - narrowing it to a byte would
// wrap a large value into a case that means something else.
static const char* lfgJoinResultString(uint32_t result) {
    switch (result) {
        case 0:  return nullptr;  // LFG_JOIN_OK
        case 1:  return "Role check failed.";
        case 2:  return "No LFG slots available for your group.";
        case 3:  return "No LFG object found.";
        case 4:  return "No slots available (player).";
        case 5:  return "No slots available (party).";
        case 6:  return "Dungeon requirements not met by all members.";
        case 7:  return "Party members are from different realms.";
        case 8:  return "Not all members are present.";
        case 9:  return "Get info timeout.";
        case 10: return "Invalid dungeon slot.";
        case 11: return "You are marked as a deserter.";
        case 12: return "A party member is marked as a deserter.";
        case 13: return "You are on a random dungeon cooldown.";
        case 14: return "A party member is on a random dungeon cooldown.";
        case 15: return "Cannot join dungeon finder.";  // LFG_JOIN_INTERNAL_ERROR
        case 16: return "No spec/role available.";
        default: return "Cannot join dungeon finder.";
    }
}

// ArenaTeamCommandErrors, as SMSG_ARENA_TEAM_COMMAND_RESULT sends them. The
// range is sparse - the server sends 0x08 for two different refusals and skips
// most of what lies between - so a table indexed by the value would be mostly
// holes and would answer the wrong sentence for anything the server adds.
static const char* arenaTeamErrorString(uint32_t error) {
    switch (error) {
        case 0x01: return "Internal arena team error.";
        case 0x02: return "You are already in an arena team of that size.";
        case 0x03: return "That player is already in an arena team of that size.";
        case 0x04: return "You have already been invited to an arena team.";
        case 0x05: return "That player has already been invited to an arena team.";
        case 0x06: return "That arena team name is invalid.";
        case 0x07: return "That arena team name already exists.";
        // The server reuses this value for the captain leaving and for acting
        // without the rank to do so.
        case 0x08: return "The captain cannot leave; permissions denied.";
        case 0x09: return "You are not in an arena team of that size.";
        case 0x0A: return "That player is not in your arena team.";
        case 0x0B: return "No player of that name was found.";
        case 0x0C: return "That player is of the opposing faction.";
        case 0x13: return "That player is ignoring you.";
        case 0x15: return "That player's level is too low.";
        case 0x16: return "That player's level is too high.";
        case 0x17: return "The arena team is full.";
        case 0x1B: return "No arena team of that name was found.";
        case 0x1E: return "Arena teams are locked for the season.";
        default:   return "The arena team command was refused.";
    }
}

static const char* lfgTeleportDeniedString(uint32_t reason) {
    switch (reason) {
        case 0:  return "You are not in a LFG group.";
        case 1:  return "You are not in the dungeon.";
        case 2:  return "You have a summon pending.";
        case 3:  return "You are dead.";
        case 4:  return "You have Deserter.";
        case 5:  return "You do not meet the requirements.";
        default: return "Teleport to dungeon denied.";
    }
}

static bool parseInspectEquipmentPayload(network::Packet& packet,
                                         uint64_t& outGuid,
                                         std::array<uint32_t, 19>& outItems,
                                         std::array<uint16_t, 19>& outEnchants) {
    const size_t start = packet.getReadPos();
    constexpr size_t kGearBytes = 19 * sizeof(uint32_t);
    constexpr uint32_t kEquipmentSlotMask = (1u << 19) - 1u;

    auto reset = [&]() {
        packet.setReadPos(start);
        outGuid = 0;
        outItems.fill(0);
        outEnchants.fill(0);
    };

    auto countSlots = [](uint32_t mask) {
        int count = 0;
        for (int slot = 0; slot < 19; ++slot) {
            if (mask & (1u << slot)) ++count;
        }
        return count;
    };

    auto parseMaskedItems = [&](const char* guidEncoding) -> bool {
        if (!packet.hasRemaining(sizeof(uint32_t))) return false;
        const uint32_t slotMask = packet.readUInt32();
        if ((slotMask & ~kEquipmentSlotMask) != 0) return false;

        const int slotCount = countSlots(slotMask);
        if (slotCount <= 0) return false;

        const size_t remaining = packet.getRemainingSize();
        const size_t candidateRecordSizes[] = {24, 20, 16, 12, 8, 4};
        size_t recordSize = 0;
        for (size_t candidate : candidateRecordSizes) {
            if (remaining == static_cast<size_t>(slotCount) * candidate) {
                recordSize = candidate;
                break;
            }
        }
        if (recordSize == 0) {
            // Some cores append a tiny trailer. Prefer the largest plausible
            // record that fits cleanly, but do not consume packets that are
            // clearly not the masked inspect format.
            for (size_t candidate : candidateRecordSizes) {
                const size_t needed = static_cast<size_t>(slotCount) * candidate;
                if (remaining >= needed && remaining - needed <= 8) {
                    recordSize = candidate;
                    break;
                }
            }
        }
        if (recordSize < sizeof(uint32_t)) return false;

        int nonZero = 0;
        for (int slot = 0; slot < 19; ++slot) {
            if ((slotMask & (1u << slot)) == 0) continue;
            if (!packet.hasRemaining(recordSize)) return false;

            const size_t recordStart = packet.getReadPos();
            const uint32_t itemEntry = packet.readUInt32();
            uint16_t enchantId = 0;
            if (recordSize >= sizeof(uint32_t) + sizeof(uint16_t) &&
                packet.hasRemaining(sizeof(uint16_t))) {
                enchantId = packet.readUInt16();
            }
            packet.setReadPos(recordStart + recordSize);

            outItems[slot] = itemEntry;
            outEnchants[slot] = enchantId;
            if (itemEntry != 0) ++nonZero;
        }

        if (nonZero == 0) return false;
        LOG_INFO("SMSG_INSPECT_RESULTS_UPDATE masked gear: guidEncoding=", guidEncoding,
                 " mask=0x", std::hex, slotMask, std::dec,
                 " slots=", slotCount, " recordSize=", recordSize,
                 " nonZero=", nonZero);
        return true;
    };

    auto readItems = [&]() -> bool {
        for (uint32_t& item : outItems) {
            if (!packet.hasRemaining(sizeof(uint32_t))) return false;
            item = packet.readUInt32();
        }
        return true;
    };

    reset();
    if (packet.hasRemaining(sizeof(uint64_t))) {
        outGuid = packet.readUInt64();
        if (outGuid != 0 && parseMaskedItems("uint64")) {
            return true;
        }
    }

    reset();
    if (packet.hasFullPackedGuid()) {
        outGuid = packet.readPackedGuid();
        if (outGuid != 0 && parseMaskedItems("packed")) {
            return true;
        }
    }

    reset();
    if (packet.getRemainingSize() >= sizeof(uint64_t) + kGearBytes) {
        outGuid = packet.readUInt64();
        if (outGuid != 0 && readItems()) {
            LOG_INFO("SMSG_INSPECT_RESULTS_UPDATE flat gear: guidEncoding=uint64 slots=19");
            return true;
        }
    }

    reset();
    if (packet.hasFullPackedGuid()) {
        outGuid = packet.readPackedGuid();
        if (outGuid != 0 && packet.getRemainingSize() >= kGearBytes && readItems()) {
            LOG_INFO("SMSG_INSPECT_RESULTS_UPDATE flat gear: guidEncoding=packed slots=19");
            return true;
        }
    }

    reset();
    return false;
}

static bool isTbcInspectTalentBitSet(const std::vector<uint8_t>& bitfield, uint32_t bitIndex) {
    const uint32_t slot = bitIndex / 7u;
    const uint32_t offset = bitIndex % 7u;
    return slot < bitfield.size() && (bitfield[slot] & (1u << offset)) != 0;
}

static bool decodeTbcInspectTalentBitfield(GameHandler& owner,
                                           uint8_t classId,
                                           const std::vector<uint8_t>& bitfield,
                                           std::array<uint32_t, 3>& outTreePoints,
                                           uint32_t& outSpentTalents) {
    outTreePoints.fill(0);
    outSpentTalents = 0;
    if (classId == 0 || bitfield.empty()) return false;

    owner.loadTalentDbc();

    const uint32_t classMask = 1u << (classId - 1u);
    std::vector<const TalentTabEntry*> classTabs;
    for (const auto& [tabId, tab] : owner.getAllTalentTabs()) {
        if (tab.classMask & classMask) {
            classTabs.push_back(&tab);
        }
    }
    std::sort(classTabs.begin(), classTabs.end(),
              [](const auto* a, const auto* b) { return a->orderIndex < b->orderIndex; });
    if (classTabs.empty()) return false;

    uint32_t tabBitStart = 0;
    for (size_t tabIndex = 0; tabIndex < classTabs.size() && tabIndex < outTreePoints.size(); ++tabIndex) {
        std::vector<const TalentEntry*> talents;
        for (const auto& [talentId, talent] : owner.getAllTalents()) {
            if (talent.tabId == classTabs[tabIndex]->tabId && talent.maxRank > 0) {
                talents.push_back(&talent);
            }
        }
        std::sort(talents.begin(), talents.end(), [](const auto* a, const auto* b) {
            if (a->row != b->row) return a->row < b->row;
            if (a->column != b->column) return a->column < b->column;
            return a->talentId < b->talentId;
        });

        uint32_t tabBits = 0;
        for (const auto* talent : talents) {
            uint8_t rank = 0;
            for (uint8_t r = 1; r <= talent->maxRank; ++r) {
                if (isTbcInspectTalentBitSet(bitfield, tabBitStart + tabBits + (r - 1u))) {
                    rank = r;
                }
            }
            outTreePoints[tabIndex] += rank;
            outSpentTalents += rank;
            tabBits += talent->maxRank;
        }
        tabBitStart += tabBits;
    }

    return true;
}

static bool parseTbcInspectTalentPayload(network::Packet& packet,
                                         GameHandler& owner,
                                         uint8_t classId,
                                         uint32_t& outUnspentTalents,
                                         uint32_t& outSpentTalents,
                                         std::array<uint32_t, 3>& outTreePoints,
                                         bool& outHasTreePoints) {
    const size_t start = packet.getReadPos();
    outUnspentTalents = 0;
    outSpentTalents = 0;
    outTreePoints.fill(0);
    outHasTreePoints = false;

    const size_t payloadSize = packet.getRemainingSize();
    if (payloadSize < sizeof(uint32_t)) {
        packet.setReadPos(start);
        return false;
    }

    auto parseTalentRecords = [](network::Packet& p, size_t count) {
        uint32_t spent = 0;
        for (size_t i = 0; i < count; ++i) {
            const uint32_t talentId = p.readUInt32();
            const uint8_t rank = p.readUInt8();
            if (talentId == 0) continue;
            spent += static_cast<uint32_t>(rank) + 1u;
        }
        return spent;
    };

    const uint32_t firstValue = packet.readUInt32();

    // TBC/CMaNGOS sends SMSG_INSPECT_TALENT as:
    //   uint32 byteCount, byte[byteCount] compact talent bitfield.
    // Older attempts treated byteCount (0x3d) as talent points; keep the
    // fallback list-style parser below for other 2.x cores, but prefer this
    // server-authored bitfield when the size lines up exactly.
    if (firstValue > 0 && firstValue <= 256 && packet.getRemainingSize() == firstValue) {
        std::vector<uint8_t> bitfield;
        bitfield.reserve(firstValue);
        for (uint32_t i = 0; i < firstValue; ++i) {
            bitfield.push_back(packet.readUInt8());
        }

        outHasTreePoints = decodeTbcInspectTalentBitfield(owner, classId, bitfield, outTreePoints, outSpentTalents);
        if (!outHasTreePoints) {
            outSpentTalents = 0;
        }
        LOG_INFO("SMSG_INSPECT_TALENT (TBC): parsed cmangos bitfield bytes=", firstValue,
                 " spent=", outSpentTalents,
                 " trees=", outTreePoints[0], "/", outTreePoints[1], "/", outTreePoints[2],
                 " class=", static_cast<int>(classId),
                 " decoded=", (outHasTreePoints ? "yes" : "no"),
                 " trailingBytes=", packet.getRemainingSize());
        return true;
    }

    // The regular talents-info packet uses uint32 unspent + uint8 count.
    // Accept that shape too so different 2.x cores still render something useful.
    if (packet.hasRemaining(1)) {
        const uint8_t nextByte = packet.readUInt8();
        const size_t afterHeaderBytes = packet.getRemainingSize();

        if (afterHeaderBytes > 0 && (afterHeaderBytes % 5u) == 0u) {
            const size_t recordCount = afterHeaderBytes / 5u;
            const uint32_t rankSpent = parseTalentRecords(packet, recordCount);
            outUnspentTalents = 0;
            outSpentTalents = firstValue != 0 ? firstValue : rankSpent;
            LOG_INFO("SMSG_INSPECT_TALENT (TBC): parsed cmangos talents records=", recordCount,
                     " spent=", outSpentTalents, " rankSpent=", rankSpent,
                     " unk=", static_cast<int>(nextByte),
                     " trailingBytes=", packet.getRemainingSize());
            return true;
        }

        const uint8_t talentCount = nextByte;
        if (packet.hasRemaining(static_cast<size_t>(talentCount) * 5u)) {
            const uint32_t spentTalents = parseTalentRecords(packet, talentCount);
            outUnspentTalents = firstValue;
            outSpentTalents = spentTalents;
            LOG_INFO("SMSG_INSPECT_TALENT (TBC): parsed talents count=", static_cast<int>(talentCount),
                     " spent=", spentTalents, " unspent=", firstValue,
                     " trailingBytes=", packet.getRemainingSize());
            return true;
        }
    }

    if ((payloadSize % 5u) == 0u) {
        packet.setReadPos(start);
        const size_t recordCount = payloadSize / 5u;
        outSpentTalents = parseTalentRecords(packet, recordCount);
        LOG_INFO("SMSG_INSPECT_TALENT (TBC): parsed bare talents records=", recordCount,
                 " spent=", outSpentTalents,
                 " trailingBytes=", packet.getRemainingSize());
        return true;
    }

    packet.setReadPos(start);
    return false;
}

static const std::string kEmptyString;

SocialHandler::SocialHandler(GameHandler& owner)
    : owner_(owner) {
    loadGuildNameCache();
}

void SocialHandler::loadGuildNameCache() {
    std::ifstream in(guildNameCachePath());
    if (!in.is_open()) return;

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        const auto separator = line.find('\t');
        if (separator == std::string::npos) continue;

        try {
            const uint32_t guildId = static_cast<uint32_t>(std::stoul(line.substr(0, separator)));
            std::string guildName = line.substr(separator + 1);
            if (guildId != 0 && !guildName.empty()) {
                guildNameCache_[guildId] = std::move(guildName);
            }
        } catch (...) {
            // Ignore stale or hand-edited cache lines.
        }
    }
}

void SocialHandler::saveGuildNameCache() const {
    const std::filesystem::path path = guildNameCachePath();
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);

    std::ofstream out(path);
    if (!out.is_open()) {
        LOG_WARNING("Could not save guild name cache to ", path.string());
        return;
    }

    for (const auto& [guildId, guildName] : guildNameCache_) {
        if (guildId != 0 && !guildName.empty()) {
            out << guildId << '\t' << guildName << '\n';
        }
    }
}

void SocialHandler::rememberGuildName(uint32_t guildId, const std::string& guildName) {
    if (guildId == 0 || guildName.empty()) return;

    auto it = guildNameCache_.find(guildId);
    if (it != guildNameCache_.end() && it->second == guildName) return;

    guildNameCache_[guildId] = guildName;
    saveGuildNameCache();
}

// ============================================================
// registerOpcodes
// ============================================================

void SocialHandler::registerOpcodes(DispatchTable& table) {
    // ---- Player info queries / social ----
    table[Opcode::SMSG_QUERY_TIME_RESPONSE] = [this](network::Packet& packet) {
        if (owner_.getState() == WorldState::IN_WORLD) handleQueryTimeResponse(packet);
    };
    table[Opcode::SMSG_PLAYED_TIME] = [this](network::Packet& packet) {
        if (owner_.getState() == WorldState::IN_WORLD) handlePlayedTime(packet);
    };
    table[Opcode::SMSG_WHO] = [this](network::Packet& packet) {
        if (owner_.getState() == WorldState::IN_WORLD) handleWho(packet);
    };
    table[Opcode::SMSG_WHOIS] = [this](network::Packet& packet) {
        if (packet.getReadPos() < packet.getSize()) {
            std::string whoisText = packet.readString();
            if (!whoisText.empty()) {
                std::string line;
                for (char c : whoisText) {
                    if (c == '\n') { if (!line.empty()) owner_.addSystemChatMessage("[Whois] " + line); line.clear(); }
                    else line += c;
                }
                if (!line.empty()) owner_.addSystemChatMessage("[Whois] " + line);
                LOG_INFO("SMSG_WHOIS: ", whoisText);
            }
        }
    };
    table[Opcode::SMSG_FRIEND_STATUS] = [this](network::Packet& packet) {
        if (owner_.getState() == WorldState::IN_WORLD) handleFriendStatus(packet);
    };
    table[Opcode::SMSG_CONTACT_LIST] = [this](network::Packet& packet) { handleContactList(packet); };
    table[Opcode::SMSG_FRIEND_LIST] = [this](network::Packet& packet) { handleFriendList(packet); };
    table[Opcode::SMSG_IGNORE_LIST] = [this](network::Packet& packet) {
        // Format: uint8 count + count × uint64 guid (no name strings in packet).
        // Names are resolved via SMSG_NAME_QUERY_RESPONSE after the list arrives.
        if (!packet.hasRemaining(1)) return;
        uint8_t ignCount = packet.readUInt8();
        owner_.ignoreListGuidsRef().clear();
        for (uint8_t i = 0; i < ignCount; ++i) {
            if (!packet.hasRemaining(8)) break;
            uint64_t ignGuid = packet.readUInt64();
            if (ignGuid != 0) {
                owner_.ignoreListGuidsRef().insert(ignGuid);
                // Query name so UI can display it later
                owner_.queryPlayerName(ignGuid);
            }
        }
        LOG_DEBUG("SMSG_IGNORE_LIST: loaded ", (int)ignCount, " ignored players");
    };
    table[Opcode::MSG_RANDOM_ROLL] = [this](network::Packet& packet) {
        if (owner_.getState() == WorldState::IN_WORLD) handleRandomRoll(packet);
    };

    // ---- Logout ----
    table[Opcode::SMSG_LOGOUT_RESPONSE] = [this](network::Packet& packet) { handleLogoutResponse(packet); };
    table[Opcode::SMSG_LOGOUT_COMPLETE] = [this](network::Packet& packet) { handleLogoutComplete(packet); };

    // ---- Inspect ----
    table[Opcode::SMSG_INSPECT_TALENT] = [this](network::Packet& packet) { handleInspectResults(packet); };
    // The honour tab's own answer, on the same opcode it was asked with.
    // AzerothCore replies with the target's guid, the honour byte, the packed
    // kill counter and three totals - MiscHandler.cpp, HandleInspectHonorStats.
    table[Opcode::MSG_INSPECT_HONOR_STATS] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(8 + 1 + 16)) return;
        const uint64_t guid = packet.readUInt64();
        // Only for whoever is being inspected. The window reads one result and
        // a stale answer for someone else would be attributed to them.
        if (inspectResult_.guid != 0 && guid != inspectResult_.guid) return;
        inspectResult_.honorRank = packet.readUInt8();
        // One field holds both days: PAIR32_LOPART is today and the high half
        // is yesterday, which is how the server rolls the counter over -
        // MAKE_PAIR32(0, kills_today) at midnight.
        const uint32_t kills = packet.readUInt32();
        inspectResult_.honorTodayKills     = kills & 0xFFFFu;
        inspectResult_.honorYesterdayKills = kills >> 16;
        inspectResult_.honorTodayContribution     = packet.readUInt32();
        inspectResult_.honorYesterdayContribution = packet.readUInt32();
        inspectResult_.honorLifetimeKills         = packet.readUInt32();
        inspectResult_.hasHonorData = true;
        if (owner_.addonEventCallbackRef())
            owner_.addonEventCallbackRef()("INSPECT_HONOR_UPDATE", {});
    };
    table[Opcode::SMSG_INSPECT_RESULTS_UPDATE] = [this](network::Packet& packet) { handleInspectResults(packet); };

    // ---- Group ----
    table[Opcode::SMSG_GROUP_INVITE] = [this](network::Packet& packet) { handleGroupInvite(packet); };
    table[Opcode::SMSG_GROUP_DECLINE] = [this](network::Packet& packet) { handleGroupDecline(packet); };
    table[Opcode::SMSG_GROUP_LIST] = [this](network::Packet& packet) { handleGroupList(packet); };
    table[Opcode::SMSG_GROUP_DESTROYED] = [this](network::Packet& /*packet*/) {
        partyData.members.clear();
        partyData.memberCount = 0;
        partyData.leaderGuid = 0;
        owner_.addUIError("Your party has been disbanded.");
        owner_.addSystemChatMessage("Your party has been disbanded.");
        if (owner_.addonEventCallbackRef()) {
            owner_.addonEventCallbackRef()("GROUP_ROSTER_UPDATE", {});
            owner_.addonEventCallbackRef()("PARTY_MEMBERS_CHANGED", {});
        }
    };
    table[Opcode::SMSG_GROUP_CANCEL] = [this](network::Packet& /*packet*/) {
        owner_.addSystemChatMessage("Group invite cancelled.");
    };
    table[Opcode::SMSG_GROUP_UNINVITE] = [this](network::Packet& packet) { handleGroupUninvite(packet); };
    table[Opcode::SMSG_PARTY_COMMAND_RESULT] = [this](network::Packet& packet) { handlePartyCommandResult(packet); };
    table[Opcode::SMSG_PARTY_MEMBER_STATS] = [this](network::Packet& packet) { handlePartyMemberStats(packet, false); };
    table[Opcode::SMSG_PARTY_MEMBER_STATS_FULL] = [this](network::Packet& packet) { handlePartyMemberStats(packet, true); };

    // ---- Ready check ----
    table[Opcode::MSG_RAID_READY_CHECK] = [this](network::Packet& packet) {
        pendingReadyCheck_ = true;
        readyCheckReadyCount_ = 0;
        readyCheckNotReadyCount_ = 0;
        readyCheckInitiator_.clear();
        readyCheckResults_.clear();
        if (packet.hasRemaining(8)) {
            uint64_t initiatorGuid = packet.readUInt64();
            auto entity = owner_.getEntityManager().getEntity(initiatorGuid);
            if (auto* unit = dynamic_cast<Unit*>(entity.get()))
                readyCheckInitiator_ = unit->getName();
        }
        if (readyCheckInitiator_.empty() && partyData.leaderGuid != 0) {
            for (const auto& member : partyData.members) {
                if (member.guid == partyData.leaderGuid) { readyCheckInitiator_ = member.name; break; }
            }
        }
        owner_.addSystemChatMessage(readyCheckInitiator_.empty()
            ? "Ready check initiated!"
            : readyCheckInitiator_ + " initiated a ready check!");
        if (owner_.addonEventCallbackRef())
            owner_.addonEventCallbackRef()("READY_CHECK", {readyCheckInitiator_});
    };
    table[Opcode::MSG_RAID_READY_CHECK_CONFIRM] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(9)) { packet.skipAll(); return; }
        uint64_t respGuid = packet.readUInt64();
        uint8_t  isReady  = packet.readUInt8();
        if (isReady) ++readyCheckReadyCount_; else ++readyCheckNotReadyCount_;
        auto nit = owner_.getPlayerNameCache().find(respGuid);
        std::string rname;
        if (nit != owner_.getPlayerNameCache().end()) rname = nit->second;
        else {
            // Only cast to Unit if the entity actually is one - a raw
            // static_pointer_cast on a GameObject would be undefined behavior.
            auto ent = owner_.getEntityManager().getEntity(respGuid);
            if (ent && (ent->getType() == ObjectType::UNIT || ent->getType() == ObjectType::PLAYER))
                rname = std::static_pointer_cast<game::Unit>(ent)->getName();
        }
        if (!rname.empty()) {
            bool found = false;
            for (auto& r : readyCheckResults_) {
                if (r.name == rname) { r.ready = (isReady != 0); found = true; break; }
            }
            if (!found) readyCheckResults_.push_back({ rname, isReady != 0 });
            char rbuf[128];
            std::snprintf(rbuf, sizeof(rbuf), "%s is %s.", rname.c_str(), isReady ? "Ready" : "Not Ready");
            owner_.addSystemChatMessage(rbuf);
        }
        if (owner_.addonEventCallbackRef()) {
            char guidBuf[32];
            snprintf(guidBuf, sizeof(guidBuf), "0x%016llX", (unsigned long long)respGuid);
            owner_.addonEventCallbackRef()("READY_CHECK_CONFIRM", {guidBuf, isReady ? "1" : "0"});
        }
    };
    table[Opcode::MSG_RAID_READY_CHECK_FINISHED] = [this](network::Packet& /*packet*/) {
        char fbuf[128];
        std::snprintf(fbuf, sizeof(fbuf), "Ready check complete: %u ready, %u not ready.",
                     readyCheckReadyCount_, readyCheckNotReadyCount_);
        owner_.addSystemChatMessage(fbuf);
        pendingReadyCheck_ = false;
        readyCheckReadyCount_ = 0;
        readyCheckNotReadyCount_ = 0;
        readyCheckResults_.clear();
        if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("READY_CHECK_FINISHED", {});
    };
    table[Opcode::SMSG_RAID_INSTANCE_INFO] = [this](network::Packet& packet) { handleRaidInstanceInfo(packet); };

    // ---- Duels ----
    table[Opcode::SMSG_DUEL_REQUESTED] = [this](network::Packet& packet) { handleDuelRequested(packet); };
    table[Opcode::SMSG_DUEL_COMPLETE] = [this](network::Packet& packet) { handleDuelComplete(packet); };
    table[Opcode::SMSG_DUEL_WINNER] = [this](network::Packet& packet) { handleDuelWinner(packet); };
    table[Opcode::SMSG_DUEL_OUTOFBOUNDS] = [this](network::Packet& /*packet*/) {
        owner_.addUIError("You are out of the duel area!");
        owner_.raiseUiError("You are out of the duel area!");
        // The interface runs the ten-second countdown to a forfeit off this
        // pair; the message alone says it once and never counts.
        if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("DUEL_OUTOFBOUNDS", {});
    };
    table[Opcode::SMSG_DUEL_INBOUNDS] = [this](network::Packet& /*packet*/) {
        // Stops that countdown. Handled as a no-op before, so a duellist who
        // stepped back inside kept a timer that was no longer true.
        if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("DUEL_INBOUNDS", {});
    };
    table[Opcode::SMSG_DUEL_COUNTDOWN] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t ms = packet.readUInt32();
            duelCountdownMs_ = (ms > 0 && ms <= 30000) ? ms : 3000;
            duelCountdownStartedAt_ = std::chrono::steady_clock::now();
        }
    };
    table[Opcode::SMSG_PARTYKILLLOG] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(16)) return;
        uint64_t killerGuid = packet.readUInt64();
        uint64_t victimGuid = packet.readUInt64();
        auto nameFor = [this](uint64_t g) -> std::string {
            auto nit = owner_.getPlayerNameCache().find(g);
            if (nit != owner_.getPlayerNameCache().end()) return nit->second;
            auto ent = owner_.getEntityManager().getEntity(g);
            if (ent && (ent->getType() == game::ObjectType::UNIT ||
                        ent->getType() == game::ObjectType::PLAYER))
                return std::static_pointer_cast<game::Unit>(ent)->getName();
            return {};
        };
        std::string killerName = nameFor(killerGuid);
        std::string victimName = nameFor(victimGuid);
        if (!killerName.empty() && !victimName.empty()) {
            char buf[256];
            std::snprintf(buf, sizeof(buf), "%s killed %s.", killerName.c_str(), victimName.c_str());
            owner_.addSystemChatMessage(buf);
        }
    };

    // ---- Guild ----
    table[Opcode::SMSG_GUILD_INFO] = [this](network::Packet& packet) { handleGuildInfo(packet); };
    table[Opcode::SMSG_GUILD_ROSTER] = [this](network::Packet& packet) { handleGuildRoster(packet); };
    table[Opcode::SMSG_GUILD_QUERY_RESPONSE] = [this](network::Packet& packet) { handleGuildQueryResponse(packet); };
    table[Opcode::SMSG_GUILD_EVENT] = [this](network::Packet& packet) { handleGuildEvent(packet); };
    table[Opcode::SMSG_GUILD_INVITE] = [this](network::Packet& packet) { handleGuildInvite(packet); };
    table[Opcode::SMSG_GUILD_COMMAND_RESULT] = [this](network::Packet& packet) { handleGuildCommandResult(packet); };
    table[Opcode::SMSG_PETITION_SHOWLIST] = [this](network::Packet& packet) { handlePetitionShowlist(packet); };
    table[Opcode::SMSG_TURN_IN_PETITION_RESULTS] = [this](network::Packet& packet) { handleTurnInPetitionResults(packet); };
    table[Opcode::SMSG_OFFER_PETITION_ERROR] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t err = packet.readUInt32();
            if (err == 1) owner_.addSystemChatMessage("Player is already in a guild.");
            else if (err == 2) owner_.addSystemChatMessage("Player already has a petition.");
            else owner_.raiseUiError("Cannot offer petition to that player.");
        }
    };
    table[Opcode::MSG_GUILD_EVENT_LOG_QUERY] = [this](network::Packet& packet) { handleGuildEventLog(packet); };
    table[Opcode::MSG_GUILD_BANK_LOG_QUERY] = [this](network::Packet& packet) { handleGuildBankLog(packet); };
    table[Opcode::SMSG_PETITION_QUERY_RESPONSE] = [this](network::Packet& packet) { handlePetitionQueryResponse(packet); };
    table[Opcode::SMSG_PETITION_SHOW_SIGNATURES] = [this](network::Packet& packet) { handlePetitionShowSignatures(packet); };
    table[Opcode::SMSG_PETITION_SIGN_RESULTS] = [this](network::Packet& packet) { handlePetitionSignResults(packet); };

    // ---- Battlefield / BG ----
    table[Opcode::SMSG_BATTLEFIELD_STATUS] = [this](network::Packet& packet) { handleBattlefieldStatus(packet); };
    table[Opcode::SMSG_BATTLEFIELD_LIST] = [this](network::Packet& packet) { handleBattlefieldList(packet); };
    table[Opcode::SMSG_BATTLEFIELD_PORT_DENIED] = [this](network::Packet& /*packet*/) {
        owner_.addUIError("Battlefield port denied.");
        owner_.addSystemChatMessage("Battlefield port denied.");
    };
    table[Opcode::MSG_BATTLEGROUND_PLAYER_POSITIONS] = [this](network::Packet& packet) {
        bgPlayerPositions_.clear();
        for (int grp = 0; grp < 2; ++grp) {
            if (!packet.hasRemaining(4)) break;
            uint32_t count = packet.readUInt32();
            for (uint32_t i = 0; i < count && packet.getRemainingSize() >= 16; ++i) {
                BgPlayerPosition pos;
                pos.guid = packet.readUInt64();
                pos.wowX = packet.readFloat();
                pos.wowY = packet.readFloat();
                pos.group = grp;
                bgPlayerPositions_.push_back(pos);
            }
        }
    };
    table[Opcode::SMSG_REMOVED_FROM_PVP_QUEUE] = [this](network::Packet& /*packet*/) {
        owner_.addSystemChatMessage("You have been removed from the PvP queue.");
    };
    table[Opcode::SMSG_GROUP_JOINED_BATTLEGROUND] = [this](network::Packet& /*packet*/) {
        owner_.addSystemChatMessage("Your group has joined the battleground.");
    };
    table[Opcode::SMSG_JOINED_BATTLEGROUND_QUEUE] = [this](network::Packet& /*packet*/) {
        owner_.addSystemChatMessage("You have joined the battleground queue.");
    };
    table[Opcode::SMSG_BATTLEGROUND_PLAYER_JOINED] = [this](network::Packet& packet) {
        if (packet.hasRemaining(8)) {
            uint64_t guid = packet.readUInt64();
            auto it = owner_.getPlayerNameCache().find(guid);
            if (it != owner_.getPlayerNameCache().end() && !it->second.empty())
                owner_.addSystemChatMessage(it->second + " has entered the battleground.");
        }
    };
    table[Opcode::SMSG_BATTLEGROUND_PLAYER_LEFT] = [this](network::Packet& packet) {
        if (packet.hasRemaining(8)) {
            uint64_t guid = packet.readUInt64();
            auto it = owner_.getPlayerNameCache().find(guid);
            if (it != owner_.getPlayerNameCache().end() && !it->second.empty())
                owner_.addSystemChatMessage(it->second + " has left the battleground.");
        }
    };

    // ---- Instance ----
    for (auto op : { Opcode::SMSG_INSTANCE_DIFFICULTY, Opcode::MSG_SET_DUNGEON_DIFFICULTY }) {
        table[op] = [this](network::Packet& packet) { handleInstanceDifficulty(packet); };
    }

    // ---- Guild / RAF / PvP AFK (moved from GameHandler) ----
    table[Opcode::SMSG_GUILD_DECLINE] = [this](network::Packet& packet) {
        if (packet.hasData()) {
            std::string name = packet.readString();
            owner_.addSystemChatMessage(name + " declined your guild invitation.");
        }
    };
    table[Opcode::SMSG_REFER_A_FRIEND_EXPIRED] = [this](network::Packet& packet) {
        owner_.addSystemChatMessage("Your Recruit-A-Friend link has expired.");
        packet.skipAll();
    };
    table[Opcode::SMSG_REFER_A_FRIEND_FAILURE] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t reason = packet.readUInt32();
            static const char* kRafErrors[] = {
                "Not eligible",            // 0
                "Target not eligible",     // 1
                "Too many referrals",      // 2
                "Wrong faction",           // 3
                "Not a recruit",           // 4
                "Recruit requirements not met", // 5
                "Level above requirement", // 6
                "Friend needs account upgrade", // 7
            };
            const char* msg = (reason < 8) ? kRafErrors[reason]
                                           : "Recruit-A-Friend failed.";
            owner_.addSystemChatMessage(std::string("Recruit-A-Friend: ") + msg);
        }
        packet.skipAll();
    };
    table[Opcode::SMSG_REPORT_PVP_AFK_RESULT] = [this](network::Packet& packet) {
        if (packet.hasRemaining(1)) {
            uint8_t result = packet.readUInt8();
            if (result == 0)
                owner_.addSystemChatMessage("AFK report submitted.");
            else
                owner_.raiseUiError("Cannot report that player as AFK right now.");
        }
        packet.skipAll();
    };
    table[Opcode::SMSG_INSTANCE_SAVE_CREATED] = [this](network::Packet& /*packet*/) {
        owner_.addSystemChatMessage("You are now saved to this instance.");
    };
    table[Opcode::SMSG_RAID_INSTANCE_MESSAGE] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(12)) return;
        uint32_t msgType = packet.readUInt32();
        uint32_t mapId   = packet.readUInt32();
        packet.readUInt32(); // diff
        std::string mapLabel = owner_.getMapName(mapId);
        if (mapLabel.empty()) mapLabel = "instance #" + std::to_string(mapId);
        if (msgType == 1 && packet.hasRemaining(4)) {
            uint32_t timeLeft = packet.readUInt32();
            owner_.addSystemChatMessage(mapLabel + " will reset in " + std::to_string(timeLeft / 60) + " minute(s).");
        } else if (msgType == 2) {
            owner_.addSystemChatMessage("You have been saved to " + mapLabel + ".");
        } else if (msgType == 3) {
            owner_.addSystemChatMessage("Welcome to " + mapLabel + ".");
        }
    };
    table[Opcode::SMSG_INSTANCE_RESET] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(4)) return;
        uint32_t mapId = packet.readUInt32();
        auto it = std::remove_if(instanceLockouts_.begin(), instanceLockouts_.end(),
            [mapId](const InstanceLockout& lo){ return lo.mapId == mapId; });
        instanceLockouts_.erase(it, instanceLockouts_.end());
        std::string mapLabel = owner_.getMapName(mapId);
        if (mapLabel.empty()) mapLabel = "instance #" + std::to_string(mapId);
        owner_.addSystemChatMessage(mapLabel + " has been reset.");
    };
    table[Opcode::SMSG_INSTANCE_RESET_FAILED] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(8)) return;
        uint32_t mapId  = packet.readUInt32();
        uint32_t reason = packet.readUInt32();
        static const char* resetFailReasons[] = {
            "Not max level.", "Offline party members.", "Party members inside.",
            "Party members changing zone.", "Heroic difficulty only."
        };
        const char* reasonMsg = (reason < 5) ? resetFailReasons[reason] : "Unknown reason.";
        std::string mapLabel = owner_.getMapName(mapId);
        if (mapLabel.empty()) mapLabel = "instance #" + std::to_string(mapId);
        owner_.addUIError("Cannot reset " + mapLabel + ": " + reasonMsg);
        owner_.addSystemChatMessage("Cannot reset " + mapLabel + ": " + reasonMsg);
    };
    table[Opcode::SMSG_INSTANCE_LOCK_WARNING_QUERY] = [this](network::Packet& packet) {
        // Nine bytes: a countdown in milliseconds, the mask of encounters
        // already completed in this instance, and whether the player was
        // previously saved to it.
        //
        // What was here read a map id, a difficulty, a time and a flag across
        // seventeen bytes, so the guard alone kept the handler from ever
        // running against the nine the server sends. It also answered on the
        // player's behalf - always accept - which is the one thing this packet
        // is asking them.
        if (!packet.hasRemaining(9)) { packet.skipAll(); return; }
        const uint32_t timerMs = packet.readUInt32();
        const uint32_t completedMask = packet.readUInt32();
        const uint8_t previouslySaved = packet.readUInt8();

        instanceLock_.active = true;
        instanceLock_.secondsLeft = static_cast<float>(timerMs) / 1000.0f;
        instanceLock_.previouslySaved = (previouslySaved != 0);
        instanceLock_.completedEncounterMask = completedMask;

        LOG_INFO("SMSG_INSTANCE_LOCK_WARNING_QUERY: ", timerMs / 1000,
                 "s to decide, completed mask 0x", std::hex, completedMask, std::dec,
                 previouslySaved ? ", previously saved" : "");

        owner_.fireAddonEvent("INSTANCE_LOCK_START", {});
    };

    // ---- LFG ----
    table[Opcode::SMSG_LFG_JOIN_RESULT] = [this](network::Packet& packet) { handleLfgJoinResult(packet); };
    table[Opcode::SMSG_LFG_QUEUE_STATUS] = [this](network::Packet& packet) { handleLfgQueueStatus(packet); };
    table[Opcode::SMSG_LFG_PROPOSAL_UPDATE] = [this](network::Packet& packet) { handleLfgProposalUpdate(packet); };
    table[Opcode::SMSG_LFG_ROLE_CHECK_UPDATE] = [this](network::Packet& packet) { handleLfgRoleCheckUpdate(packet); };
    for (auto op : { Opcode::SMSG_LFG_UPDATE_PLAYER, Opcode::SMSG_LFG_UPDATE_PARTY }) {
        table[op] = [this](network::Packet& packet) { handleLfgUpdatePlayer(packet); };
    }
    table[Opcode::SMSG_LFG_PLAYER_REWARD] = [this](network::Packet& packet) { handleLfgPlayerReward(packet); };
    table[Opcode::SMSG_LFG_BOOT_PROPOSAL_UPDATE] = [this](network::Packet& packet) { handleLfgBootProposalUpdate(packet); };
    table[Opcode::SMSG_LFG_TELEPORT_DENIED] = [this](network::Packet& packet) { handleLfgTeleportDenied(packet); };
    table[Opcode::SMSG_LFG_DISABLED] = [this](network::Packet& /*packet*/) {
        owner_.addSystemChatMessage("The Dungeon Finder is currently disabled.");
    };
    table[Opcode::SMSG_LFG_OFFER_CONTINUE] = [this](network::Packet& packet) {
        // uint32 dungeon entry, which the handler used to skip entirely - and
        // then fired the event with nothing. lfgframe.lua opens with
        // NORMAL_FONT_COLOR_CODE..displayName.."|r", and concatenating nil
        // raises, so the offer took its own handler down every time.
        //
        // The entry packs both halves the way LFGDungeonData::Entry builds it:
        // id in the low twenty-four bits, type in the top eight.
        uint32_t dungeonId = 0, typeId = 0;
        if (packet.hasRemaining(4)) {
            const uint32_t entry = packet.readUInt32();
            dungeonId = entry & 0x00FFFFFFu;
            typeId    = (entry >> 24) & 0xFFu;
        }
        std::string name = owner_.getLfgDungeonName(dungeonId);
        if (name.empty()) name = "your dungeon";
        owner_.addSystemChatMessage("Dungeon Finder: You may continue " + name + ".");
        if (owner_.addonEventCallbackRef()) {
            owner_.addonEventCallbackRef()("LFG_OFFER_CONTINUE",
                                           {name, std::to_string(dungeonId),
                                            std::to_string(typeId)});
        }
    };
    table[Opcode::SMSG_LFG_ROLE_CHOSEN] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(13)) { packet.skipAll(); return; }
        uint64_t roleGuid = packet.readUInt64();
        uint8_t  ready    = packet.readUInt8();
        uint32_t roles    = packet.readUInt32();
        std::string roleName;
        if (roles & 0x02) roleName += "Tank ";
        if (roles & 0x04) roleName += "Healer ";
        if (roles & 0x08) roleName += "DPS ";
        if (roleName.empty()) roleName = "None";
        std::string pName = "A player";
        if (auto e = owner_.getEntityManager().getEntity(roleGuid))
            if (auto u = std::dynamic_pointer_cast<Unit>(e))
                pName = u->getName();
        if (ready) owner_.addSystemChatMessage(pName + " has chosen: " + roleName);
        // Announced as well as written to chat. The role-check window lists
        // who has picked what as they pick it, and it learns that from this
        // event alone - parsed here since the chat line needed the same three
        // bits, and never fired, so the list stayed blank while the chat
        // filled up beside it.
        //
        // Booleans through eventBool: "0" is a number and true in Lua, so a
        // false argument in front of a true one cannot be spelled as text.
        if (owner_.addonEventCallbackRef()) {
            owner_.addonEventCallbackRef()("LFG_ROLE_CHECK_ROLE_CHOSEN", {
                pName,
                eventBool((roles & 0x02) != 0),
                eventBool((roles & 0x04) != 0),
                eventBool((roles & 0x08) != 0)
            });
        }
        packet.skipAll();
    };
    // The one that carries the locks. Skipped wholesale until now, which is
    // why the dungeon finder listed every dungeon as queueable and left the
    // server to refuse.
    table[Opcode::SMSG_LFG_PLAYER_INFO] = [this](network::Packet& packet) {
        handleLfgPlayerInfo(packet);
    };
    for (auto op : { Opcode::SMSG_LFG_UPDATE_SEARCH, Opcode::SMSG_UPDATE_LFG_LIST,
                     Opcode::SMSG_LFG_PARTY_INFO }) {
        table[op] = [](network::Packet& packet) { packet.skipAll(); };
    }
    table[Opcode::SMSG_OPEN_LFG_DUNGEON_FINDER] = [this](network::Packet& packet) {
        packet.skipAll();
        if (owner_.openLfgCallbackRef()) owner_.openLfgCallbackRef()();
    };

    // ---- Arena ----
    table[Opcode::SMSG_ARENA_TEAM_COMMAND_RESULT] = [this](network::Packet& packet) { handleArenaTeamCommandResult(packet); };
    table[Opcode::SMSG_ARENA_TEAM_QUERY_RESPONSE] = [this](network::Packet& packet) { handleArenaTeamQueryResponse(packet); };
    table[Opcode::SMSG_ARENA_TEAM_ROSTER] = [this](network::Packet& packet) { handleArenaTeamRoster(packet); };
    table[Opcode::SMSG_ARENA_TEAM_INVITE] = [this](network::Packet& packet) { handleArenaTeamInvite(packet); };
    table[Opcode::SMSG_ARENA_TEAM_EVENT] = [this](network::Packet& packet) { handleArenaTeamEvent(packet); };
    table[Opcode::SMSG_ARENA_TEAM_STATS] = [this](network::Packet& packet) { handleArenaTeamStats(packet); };
    table[Opcode::SMSG_ARENA_ERROR] = [this](network::Packet& packet) { handleArenaError(packet); };
    table[Opcode::MSG_PVP_LOG_DATA] = [this](network::Packet& packet) { handlePvpLogData(packet); };

    // ---- Factions / group leader ----
    table[Opcode::SMSG_INITIALIZE_FACTIONS] = [this](network::Packet& p) { handleInitializeFactions(p); };
    table[Opcode::SMSG_SET_FACTION_STANDING] = [this](network::Packet& p) { handleSetFactionStanding(p); };
    table[Opcode::SMSG_SET_FACTION_ATWAR] = [this](network::Packet& p) { handleSetFactionAtWar(p); };
    table[Opcode::SMSG_SET_FACTION_VISIBLE] = [this](network::Packet& p) { handleSetFactionVisible(p); };
    table[Opcode::SMSG_GROUP_SET_LEADER] = [this](network::Packet& p) { handleGroupSetLeader(p); };
}

// ============================================================
// Non-inline accessors requiring GameHandler
// ============================================================

std::string SocialHandler::getWhoAreaName(uint32_t zoneId) const {
    return owner_.getAreaName(zoneId);
}

std::string SocialHandler::getCurrentLfgDungeonName() const {
    return owner_.getLfgDungeonName(lfgDungeonId_);
}

bool SocialHandler::isInGuild() const {
    if (!guildName_.empty()) return true;
    const Character* ch = owner_.getActiveCharacter();
    return ch && ch->hasGuild();
}

uint32_t SocialHandler::getEntityGuildId(uint64_t guid) const {
    auto entity = owner_.getEntityManager().getEntity(guid);
    if (!entity || entity->getType() != ObjectType::PLAYER) return 0;
    const uint16_t ufUnitEnd = fieldIndex(UF::UNIT_END);
    if (ufUnitEnd == 0xFFFF) return 0;
    return entity->getField(ufUnitEnd + 3);
}

const std::string& SocialHandler::lookupGuildName(uint32_t guildId) {
    if (guildId == 0) return kEmptyString;
    auto it = guildNameCache_.find(guildId);
    if (it != guildNameCache_.end()) return it->second;

    const auto now = std::chrono::steady_clock::now();
    auto pendingIt = pendingGuildNameQueries_.find(guildId);
    const bool shouldQuery =
        pendingIt == pendingGuildNameQueries_.end() ||
        std::chrono::duration_cast<std::chrono::seconds>(now - pendingIt->second).count() >= 2;
    if (shouldQuery) {
        pendingGuildNameQueries_[guildId] = now;
        queryGuildInfo(guildId);
    }
    return kEmptyString;
}

// ============================================================
// Inspection
// ============================================================

void SocialHandler::inspectTarget() { inspectUnit(owner_.getTargetGuid()); }

// Any player, not only the targeted one: FrameXML's unit menus name the unit
// the menu was opened on, and on a party frame that is not the target.
void SocialHandler::requestInspectHonorData(uint64_t guid) {
    if (guid == 0) return;
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    const uint32_t wire = wireOpcode(Opcode::MSG_INSPECT_HONOR_STATS);
    if (wire == 0xFFFF) return;
    network::Packet pkt(static_cast<uint16_t>(wire));
    pkt.writeUInt64(guid);
    owner_.getSocket()->send(pkt);
}

void SocialHandler::inspectUnit(uint64_t guid) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) {
        LOG_WARNING("Cannot inspect: not in world or not connected");
        return;
    }
    if (guid == 0) {
        owner_.raiseUiError("You must target a player to inspect.");
        return;
    }
    auto entity = owner_.getEntityManager().getEntity(guid);
    if (!entity || entity->getType() != ObjectType::PLAYER) {
        owner_.addSystemChatMessage("You can only inspect players.");
        return;
    }
    auto packet = InspectPacket::build(guid);
    owner_.getSocket()->send(packet);
    if (isActiveExpansion("wotlk")) {
        auto achPkt = QueryInspectAchievementsPacket::build(guid);
        owner_.getSocket()->send(achPkt);
    }
    auto player = std::static_pointer_cast<Player>(entity);
    std::string name = player->getName().empty() ? "Target" : player->getName();
    owner_.addSystemChatMessage("Inspecting " + name + "...");
    LOG_INFO("Sent inspect request for player: ", name, " (GUID: 0x", std::hex, guid, std::dec, ")");
}

void SocialHandler::handleInspectResults(network::Packet& packet) {
    if (isActiveExpansion("tbc") && packet.getOpcode() == wireOpcode(Opcode::SMSG_INSPECT_RESULTS_UPDATE)) {
        uint64_t guid = 0;
        std::array<uint32_t, 19> items{};
        std::array<uint16_t, 19> enchantIds{};
        if (parseInspectEquipmentPayload(packet, guid, items, enchantIds)) {
            owner_.cacheInspectedPlayerEquipment(guid, items);

            auto entity = owner_.getEntityManager().getEntity(guid);
            std::string playerName = "Target";
            if (entity) {
                auto player = std::dynamic_pointer_cast<Player>(entity);
                if (player && !player->getName().empty()) playerName = player->getName();
            }

            // TBC servers send inspected gear separately from talent data. Make the
            // gear-only response visible to the Inspect window immediately while
            // preserving talents if a matching talent response already arrived.
            if (inspectResult_.guid != guid) {
                inspectResult_ = InspectResult{};
                inspectResult_.guid = guid;
            }
            inspectResult_.playerName = playerName;
            inspectResult_.itemEntries = items;
            inspectResult_.enchantIds = enchantIds;

            LOG_INFO("SMSG_INSPECT_RESULTS_UPDATE (TBC gear): ", playerName, " has gear in ",
                     std::count_if(items.begin(), items.end(),
                                   [](uint32_t e) { return e != 0; }), "/19 slots");
            if (owner_.addonEventCallbackRef()) {
                char guidBuf[32];
                snprintf(guidBuf, sizeof(guidBuf), "0x%016llX", (unsigned long long)guid);
                owner_.addonEventCallbackRef()("INSPECT_READY", {guidBuf});
                // INSPECT_READY is a later expansion's name and nothing in this
                // interface listens for it. The inspect paperdoll refreshes on
                // UNIT_INVENTORY_CHANGED for the unit it is showing, so the gear
                // arrived and the window was never told - it kept whatever it had
                // when it opened. Fired for the unit token this guid answers to,
                // and only when it answers to one: an inspected player who is not
                // the target has no token for the interface to match against.
                const std::string inspectUnit = owner_.guidToUnitId(guid);
                if (!inspectUnit.empty()) {
                    owner_.addonEventCallbackRef()("UNIT_INVENTORY_CHANGED", {inspectUnit});
                }
            }
            return;
        }
        LOG_DEBUG("SMSG_INSPECT_RESULTS_UPDATE (TBC gear): unrecognized payload size=",
                  packet.getSize());
    }

    if (isActiveExpansion("tbc") &&
        packet.getOpcode() == wireOpcode(Opcode::SMSG_INSPECT_TALENT)) {
        if (!packet.hasFullPackedGuid()) return;
        uint64_t guid = packet.readPackedGuid();
        if (guid == 0) return;

        auto entity = owner_.getEntityManager().getEntity(guid);
        std::string playerName = "Target";
        if (entity) {
            auto player = std::dynamic_pointer_cast<Player>(entity);
            if (player && !player->getName().empty()) playerName = player->getName();
        }

        if (inspectResult_.guid != guid) {
            inspectResult_ = InspectResult{};
            inspectResult_.guid = guid;
        }
        inspectResult_.playerName = playerName;

        uint32_t unspentTalents = 0;
        uint32_t spentTalents = 0;
        std::array<uint32_t, 3> treePoints{};
        bool hasTreePoints = false;
        const uint8_t classId = owner_.lookupPlayerClass(guid);
        const size_t talentPayloadStart = packet.getReadPos();
        if (parseTbcInspectTalentPayload(packet, owner_, classId, unspentTalents, spentTalents,
                                         treePoints, hasTreePoints)) {
            inspectResult_.totalTalents = spentTalents;
            inspectResult_.unspentTalents = unspentTalents;
            inspectResult_.hasTalentData = true;
            inspectResult_.hasTalentTreePoints = hasTreePoints;
            inspectResult_.talentTreePoints = treePoints;
            inspectResult_.talentGroups = 1;
            inspectResult_.activeTalentGroup = 0;
        } else {
            packet.setReadPos(talentPayloadStart);
            LOG_INFO("SMSG_INSPECT_TALENT (TBC): ", playerName,
                     " unparsed payload bytes=", packet.getRemainingSize());
        }

        auto gearIt = owner_.inspectedPlayerItemEntriesRef().find(guid);
        if (gearIt != owner_.inspectedPlayerItemEntriesRef().end()) {
            inspectResult_.itemEntries = gearIt->second;
        }

        LOG_INFO("SMSG_INSPECT_TALENT (TBC): ", playerName,
                 " gearCached=", (gearIt != owner_.inspectedPlayerItemEntriesRef().end() ? "yes" : "no"),
                 " payload bytes remaining=", packet.getRemainingSize());
        if (owner_.addonEventCallbackRef()) {
            char guidBuf[32];
            snprintf(guidBuf, sizeof(guidBuf), "0x%016llX", (unsigned long long)guid);
            owner_.addonEventCallbackRef()("INSPECT_READY", {guidBuf});
            const std::string inspectUnit = owner_.guidToUnitId(guid);
            if (!inspectUnit.empty()) {
                owner_.addonEventCallbackRef()("UNIT_INVENTORY_CHANGED", {inspectUnit});
            }
        }
        return;
    }

    // SMSG_INSPECT_TALENT (WotLK) is a packed guid, then
    // BuildPlayerTalentsInfoData and BuildEnchantmentsInfoData. There is no
    // leading discriminator, and reading one ate the packed guid's mask byte
    // and put everything after it a byte out.
    //
    // The branch that used to sit behind that byte re-parsed the player's own
    // talents into a second set of members on GameHandler. Nothing read them -
    // SpellHandler keeps the ones the talent frame draws from - and this
    // opcode never carries them anyway.
    const bool talentTbc = isPreWotlk();
    if (packet.getRemainingSize() < (talentTbc ? 8u : 2u)) return;
    uint64_t guid = talentTbc
        ? packet.readUInt64() : packet.readPackedGuid();
    if (guid == 0) return;

    size_t bytesLeft = packet.getRemainingSize();
    if (bytesLeft < 6) {
        LOG_WARNING("SMSG_TALENTS_INFO: too short after guid, ", bytesLeft, " bytes");
        auto entity = owner_.getEntityManager().getEntity(guid);
        std::string name = "Target";
        if (entity) {
            auto player = std::dynamic_pointer_cast<Player>(entity);
            if (player && !player->getName().empty()) name = player->getName();
        }
        owner_.addSystemChatMessage("Inspecting " + name + " (no talent data available).");
        return;
    }

    uint32_t unspentTalents = packet.readUInt32();
    uint8_t talentGroupCount = packet.readUInt8();
    uint8_t activeTalentGroup = packet.readUInt8();

    auto entity = owner_.getEntityManager().getEntity(guid);
    std::string playerName = "Target";
    if (entity) {
        auto player = std::dynamic_pointer_cast<Player>(entity);
        if (player && !player->getName().empty()) playerName = player->getName();
    }

    // Every spec is sent. Counting all of them reported a dual-spec character
    // as having spent roughly twice what they have.
    uint32_t totalTalents = 0;
    std::unordered_map<uint32_t, uint8_t> inspectTalents;
    for (uint8_t g = 0; g < talentGroupCount && g < 2; ++g) {
        bytesLeft = packet.getRemainingSize();
        if (bytesLeft < 1) break;
        uint8_t talentCount = packet.readUInt8();
        for (uint8_t t = 0; t < talentCount; ++t) {
            bytesLeft = packet.getRemainingSize();
            if (bytesLeft < 5) break;
            const uint32_t talentId = packet.readUInt32();
            const uint8_t rank = packet.readUInt8();
            if (g == activeTalentGroup) {
                // Rank is zero-based on the wire, as it is for the player's own
                // talents. The inspect talent tab was counting these and
                // throwing them away, so it drew the viewer's own tree with the
                // target's name over it.
                inspectTalents[talentId] = static_cast<uint8_t>(rank + 1u);
                totalTalents++;
            }
        }
        bytesLeft = packet.getRemainingSize();
        if (bytesLeft < 1) break;
        uint8_t glyphCount = packet.readUInt8();
        for (uint8_t gl = 0; gl < glyphCount; ++gl) {
            bytesLeft = packet.getRemainingSize();
            if (bytesLeft < 2) break;
            packet.readUInt16();
        }
    }

    // BuildEnchantmentsInfoData: a mask of which of the nineteen equipment
    // slots are filled, then per filled slot the item's entry, a mask of which
    // of its twelve enchantment slots are used, and an id per set bit.
    //
    // What was here read the slot mask and then one bare uint16 per set bit,
    // so it took the low half of the first item's entry as that slot's enchant
    // and drifted further out of step with every item after it.
    constexpr int kEquipmentSlots = 19;
    constexpr int kEnchantmentSlots = 12;
    std::array<uint32_t, 19> itemEntries{};
    std::array<uint16_t, 19> enchantIds{};
    bool sawGear = false;
    if (packet.hasRemaining(4)) {
        const uint32_t slotMask = packet.readUInt32();
        for (int slot = 0; slot < kEquipmentSlots; ++slot) {
            if (!(slotMask & (1u << slot))) continue;
            if (!packet.hasRemaining(6)) break;
            itemEntries[slot] = packet.readUInt32();
            sawGear = true;
            const uint16_t enchantMask = packet.readUInt16();
            for (int e = 0; e < kEnchantmentSlots; ++e) {
                if (!(enchantMask & (1u << e))) continue;
                if (!packet.hasRemaining(2)) break;
                const uint16_t enchId = packet.readUInt16();
                // Slot zero is the permanent enchant, which is the one the
                // paperdoll names.
                if (e == 0) enchantIds[slot] = enchId;
            }
        }
    }

    // A different player means the honour figures in hand are somebody else's.
    // The fields here are assigned one at a time rather than as a whole struct,
    // so anything not named survives - and the honour tab reads
    // HasInspectHonorData to decide whether to ask, so a stale true would show
    // the last player's kills against this one's name and never correct itself.
    if (inspectResult_.guid != guid) {
        inspectResult_.hasHonorData = false;
        inspectResult_.honorTodayKills = 0;
        inspectResult_.honorYesterdayKills = 0;
        inspectResult_.honorTodayContribution = 0;
        inspectResult_.honorYesterdayContribution = 0;
        inspectResult_.honorLifetimeKills = 0;
        inspectResult_.honorRank = 0;
    }
    inspectResult_.guid              = guid;
    inspectResult_.playerName        = playerName;
    inspectResult_.totalTalents      = totalTalents;
    inspectResult_.unspentTalents    = unspentTalents;
    inspectResult_.hasTalentData     = true;
    inspectResult_.hasTalentTreePoints = false;
    inspectResult_.talentTreePoints  = {};
    inspectResult_.talentGroups      = talentGroupCount;
    inspectResult_.activeTalentGroup = activeTalentGroup;
    inspectResult_.enchantIds        = enchantIds;
    inspectResult_.talentRanks       = std::move(inspectTalents);
    inspectResult_.classId           = owner_.lookupPlayerClass(guid);

    // The gear is in this packet. It used to be looked up from the cache that
    // Classic's SMSG_INSPECT and TBC's SMSG_INSPECT_RESULTS_UPDATE fill, and
    // nothing filled it on WotLK, so inspecting anyone showed an empty
    // paperdoll however well equipped they were.
    if (sawGear) {
        inspectResult_.itemEntries = itemEntries;
        owner_.cacheInspectedPlayerEquipment(guid, itemEntries);
    } else {
        auto gearIt = owner_.inspectedPlayerItemEntriesRef().find(guid);
        inspectResult_.itemEntries =
            (gearIt != owner_.inspectedPlayerItemEntriesRef().end()) ? gearIt->second
                                                                     : std::array<uint32_t, 19>{};
    }

    LOG_INFO("Inspect results for ", playerName, ": ", totalTalents, " talents, ",
             unspentTalents, " unspent, ", (int)talentGroupCount, " specs");
    if (owner_.addonEventCallbackRef()) {
        char guidBuf[32];
        snprintf(guidBuf, sizeof(guidBuf), "0x%016llX", (unsigned long long)guid);
        owner_.addonEventCallbackRef()("INSPECT_READY", {guidBuf});
        // The same packet carries the talents, and the talent tab of the
        // inspect window redraws on this event alone. Only INSPECT_READY was
        // fired, which is what the gear tab listens for - so inspecting anyone
        // filled the paperdoll and left the talent tab blank, however
        // completely the ranks above had been parsed.
        owner_.addonEventCallbackRef()("INSPECT_TALENT_READY", {});
        const std::string inspectUnit = owner_.guidToUnitId(guid);
        if (!inspectUnit.empty()) {
            owner_.addonEventCallbackRef()("UNIT_INVENTORY_CHANGED", {inspectUnit});
        }
    }
}

// ============================================================
// Server Info / Who / Social
// ============================================================

void SocialHandler::queryServerTime(bool announce) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    announceServerTime_ = announce;
    auto packet = QueryTimePacket::build();
    owner_.getSocket()->send(packet);
    LOG_INFO("Requested server time");
}

void SocialHandler::requestPlayedTime() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = RequestPlayedTimePacket::build(true);
    owner_.getSocket()->send(packet);
    LOG_INFO("Requested played time");
}

void SocialHandler::queryWho(const std::string& playerName) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = WhoPacket::build(0, 0, playerName);
    owner_.getSocket()->send(packet);
    LOG_INFO("Sent WHO query", playerName.empty() ? "" : " for: " + playerName);
}

void SocialHandler::addFriend(const std::string& playerName, const std::string& note) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (playerName.empty()) { owner_.raiseUiError("You must specify a player name."); return; }
    auto packet = AddFriendPacket::build(playerName, note);
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage("Sending friend request to " + playerName + "...");
    LOG_INFO("Sent friend request to: ", playerName);
}

void SocialHandler::removeFriend(const std::string& playerName) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (playerName.empty()) { owner_.raiseUiError("You must specify a player name."); return; }
    auto it = owner_.friendsCacheRef().find(playerName);
    if (it == owner_.friendsCacheRef().end()) {
        owner_.addSystemChatMessage(playerName + " is not in your friends list.");
        return;
    }
    auto packet = DelFriendPacket::build(it->second);
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage("Removing " + playerName + " from friends list...");
    LOG_INFO("Sent remove friend request for: ", playerName);
}

void SocialHandler::setFriendNote(const std::string& playerName, const std::string& note) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (playerName.empty()) { owner_.raiseUiError("You must specify a player name."); return; }
    auto it = owner_.friendsCacheRef().find(playerName);
    if (it == owner_.friendsCacheRef().end()) {
        owner_.addSystemChatMessage(playerName + " is not in your friends list.");
        return;
    }
    auto packet = SetContactNotesPacket::build(it->second, note);
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage("Updated note for " + playerName);
    LOG_INFO("Set friend note for: ", playerName);
}

void SocialHandler::addIgnore(const std::string& playerName) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (playerName.empty()) { owner_.raiseUiError("You must specify a player name."); return; }
    auto packet = AddIgnorePacket::build(playerName);
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage("Adding " + playerName + " to ignore list...");
    LOG_INFO("Sent ignore request for: ", playerName);
}

void SocialHandler::removeIgnore(const std::string& playerName) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (playerName.empty()) { owner_.raiseUiError("You must specify a player name."); return; }
    auto it = owner_.ignoreCacheRef().find(playerName);
    if (it == owner_.ignoreCacheRef().end()) {
        owner_.addSystemChatMessage(playerName + " is not in your ignore list.");
        return;
    }
    auto packet = DelIgnorePacket::build(it->second);
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage("Removing " + playerName + " from ignore list...");
    // Don't erase from ignoreCache here - wait for the server's SMSG_IGNORE_LIST
    // response to confirm. Erasing optimistically desyncs the cache if the server
    // rejects the request. (Compare with removeFriend which also waits for
    // SMSG_FRIEND_STATUS before updating its cache.)
    owner_.ignoreListGuidsRef().erase(it->second);
    LOG_INFO("Sent remove ignore request for: ", playerName);
}

void SocialHandler::randomRoll(uint32_t minRoll, uint32_t maxRoll) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (minRoll > maxRoll) std::swap(minRoll, maxRoll);
    if (maxRoll > 10000) maxRoll = 10000;
    auto packet = RandomRollPacket::build(minRoll, maxRoll);
    owner_.getSocket()->send(packet);
    LOG_INFO("Rolled ", minRoll, "-", maxRoll);
}

// ============================================================
// Logout
// ============================================================

void SocialHandler::requestLogout(bool exitAfterLogout) {
    // Exit Game has to close the program. Everything below is about leaving the
    // world tidily first, which is worth doing and is not worth being trapped
    // by: each way the request cannot be made is answered by going anyway.
    //
    // With no connection there is nobody to ask. At the login screen, or after
    // a disconnect, the button did nothing whatsoever.
    if (!owner_.getSocket()) {
        if (exitAfterLogout && owner_.logoutCompleteCallbackRef()) {
            owner_.logoutCompleteCallbackRef()(true);
        }
        return;
    }
    if (loggingOut_) {
        // Asked a second time while a logout is already running. For Exit that
        // is the player saying they would rather not wait, which is exactly
        // what ForceQuit is described as doing and what the countdown popup's
        // own button is for.
        if (exitAfterLogout) {
            LOG_WARNING("Exit Game asked for during a logout - closing now");
            if (owner_.logoutCompleteCallbackRef()) owner_.logoutCompleteCallbackRef()(true);
            return;
        }
        owner_.addSystemChatMessage("Already logging out.");
        return;
    }
    auto packet = LogoutRequestPacket::build();
    owner_.getSocket()->send(packet);
    loggingOut_ = true;
    exitAfterLogout_ = exitAfterLogout;
    LOG_WARNING("Sent logout request (exitAfterLogout=", exitAfterLogout ? "yes" : "no", ")");  // temporary
}

void SocialHandler::cancelLogout() {
    if (!owner_.getSocket()) return;
    if (!loggingOut_) { owner_.addSystemChatMessage("Not currently logging out."); return; }
    auto packet = LogoutCancelPacket::build();
    owner_.getSocket()->send(packet);
    loggingOut_ = false;
    exitAfterLogout_ = false;
    logoutCountdown_ = 0.0f;
    owner_.addSystemChatMessage("Logout cancelled.");
    LOG_INFO("Cancelled logout");
}

// ============================================================
// Guild
// ============================================================

void SocialHandler::requestGuildInfo() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = GuildInfoPacket::build();
    owner_.getSocket()->send(packet);
}

void SocialHandler::requestGuildRoster() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = GuildRosterPacket::build();
    owner_.getSocket()->send(packet);
    // Said in the log, not to the player. This is asked for far more often
    // than a person would ask for it: eight places here request a roster after
    // a guild change, and the interface asks whenever the mail frame or the
    // friends frame opens with an empty one. Every one of those put a line in
    // chat that the real client never writes.
    LOG_DEBUG("Requested guild roster");
}

// The longer "guild information" text, which is a different field from the
// message of the day and has its own opcode.
void SocialHandler::setGuildInfoText(const std::string& text) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_INFO_TEXT));
    packet.writeString(text);
    owner_.getSocket()->send(packet);
    requestGuildRoster();
}

// Turn a letter into a readable item, which is what "take" does to a mail whose
// body is the thing being kept.
void SocialHandler::takeInboxTextItem(uint32_t mailId) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    network::Packet packet(wireOpcode(Opcode::CMSG_MAIL_CREATE_TEXT_ITEM));
    packet.writeUInt64(0);            // mailbox guid - the server uses the open one
    packet.writeUInt32(mailId);
    owner_.getSocket()->send(packet);
}

void SocialHandler::setGuildMotd(const std::string& motd) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = GuildMotdPacket::build(motd);
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage("Guild MOTD updated.");
}

void SocialHandler::promoteGuildMember(const std::string& playerName) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (playerName.empty()) { owner_.raiseUiError("You must specify a player name."); return; }
    auto packet = GuildPromotePacket::build(playerName);
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage("Promoting " + playerName + "...");
}

void SocialHandler::demoteGuildMember(const std::string& playerName) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (playerName.empty()) { owner_.raiseUiError("You must specify a player name."); return; }
    auto packet = GuildDemotePacket::build(playerName);
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage("Demoting " + playerName + "...");
}

void SocialHandler::leaveGuild() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = GuildLeavePacket::build();
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage("Leaving guild...");
}

void SocialHandler::inviteToGuild(const std::string& playerName) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (playerName.empty()) { owner_.raiseUiError("You must specify a player name."); return; }
    auto packet = GuildInvitePacket::build(playerName);
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage("Inviting " + playerName + " to guild...");
}

void SocialHandler::kickGuildMember(const std::string& playerName) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = GuildRemovePacket::build(playerName);
    owner_.getSocket()->send(packet);
}

void SocialHandler::disbandGuild() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = GuildDisbandPacket::build();
    owner_.getSocket()->send(packet);
}

void SocialHandler::setGuildLeader(const std::string& name) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = GuildLeaderPacket::build(name);
    owner_.getSocket()->send(packet);
}

void SocialHandler::setGuildPublicNote(const std::string& name, const std::string& note) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = GuildSetPublicNotePacket::build(name, note);
    owner_.getSocket()->send(packet);
}

void SocialHandler::setGuildOfficerNote(const std::string& name, const std::string& note) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = GuildSetOfficerNotePacket::build(name, note);
    owner_.getSocket()->send(packet);
}

void SocialHandler::acceptGuildInvite() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    pendingGuildInvite_ = false;
    auto packet = GuildAcceptPacket::build();
    owner_.getSocket()->send(packet);
}

void SocialHandler::declineGuildInvite() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    pendingGuildInvite_ = false;
    auto packet = GuildDeclineInvitationPacket::build();
    owner_.getSocket()->send(packet);
}

void SocialHandler::queryGuildInfo(uint32_t guildId) {
    // Allow guild queries at the character screen too - the socket is
    // connected and the server accepts CMSG_GUILD_QUERY before login.
    if (!owner_.getSocket()) return;
    auto packet = GuildQueryPacket::build(guildId);
    owner_.getSocket()->send(packet);
}

void SocialHandler::createGuild(const std::string& guildName) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = GuildCreatePacket::build(guildName);
    owner_.getSocket()->send(packet);
}

void SocialHandler::addGuildRank(const std::string& rankName) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = GuildAddRankPacket::build(rankName);
    owner_.getSocket()->send(packet);
    requestGuildRoster();
}

void SocialHandler::deleteGuildRank() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = GuildDelRankPacket::build();
    owner_.getSocket()->send(packet);
    requestGuildRoster();
}

void SocialHandler::requestPetitionShowlist(uint64_t npcGuid) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = PetitionShowlistPacket::build(npcGuid);
    owner_.getSocket()->send(packet);
}

void SocialHandler::buyPetition(uint64_t npcGuid, const std::string& guildName,
                                uint32_t clientIndex) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = PetitionBuyPacket::build(npcGuid, guildName, clientIndex);
    owner_.getSocket()->send(packet);
}

void SocialHandler::signPetition(uint64_t petitionGuid) {
    if (!owner_.getSocket() || owner_.getState() != WorldState::IN_WORLD) return;
    network::Packet pkt(wireOpcode(Opcode::CMSG_PETITION_SIGN));
    pkt.writeUInt64(petitionGuid);
    pkt.writeUInt8(0);
    owner_.getSocket()->send(pkt);
}

void SocialHandler::turnInPetition(uint64_t petitionGuid) {
    if (!owner_.getSocket() || owner_.getState() != WorldState::IN_WORLD) return;
    network::Packet pkt(wireOpcode(Opcode::CMSG_TURN_IN_PETITION));
    pkt.writeUInt64(petitionGuid);
    owner_.getSocket()->send(pkt);
}

void SocialHandler::offerPetition(uint64_t petitionGuid, uint64_t targetGuid) {
    if (!owner_.getSocket() || owner_.getState() != WorldState::IN_WORLD) return;
    if (petitionGuid == 0 || targetGuid == 0) return;
    network::Packet pkt(wireOpcode(Opcode::CMSG_OFFER_PETITION));
    // The leading dword is not the petition type despite where it sits - the
    // server reads it and throws it away. Every expansion this client speaks
    // has it, so it is written unconditionally rather than gated.
    pkt.writeUInt32(0);
    pkt.writeUInt64(petitionGuid);
    pkt.writeUInt64(targetGuid);
    owner_.getSocket()->send(pkt);
}

// ============================================================
// Ready Check
// ============================================================

void SocialHandler::initiateReadyCheck() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (!isInGroup()) { owner_.raiseUiError("You must be in a group to initiate a ready check."); return; }
    auto packet = ReadyCheckPacket::build();
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage("Ready check initiated.");
}

void SocialHandler::respondToReadyCheck(bool ready) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = ReadyCheckConfirmPacket::build(ready);
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage(ready ? "You are ready." : "You are not ready.");
}

// ============================================================
// Duel
// ============================================================

void SocialHandler::acceptDuel() {
    if (!pendingDuelRequest_ || owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    pendingDuelRequest_ = false;
    auto pkt = DuelAcceptPacket::build();
    owner_.getSocket()->send(pkt);
    owner_.addSystemChatMessage("You accept the duel.");
}

void SocialHandler::forfeitDuel() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    pendingDuelRequest_ = false;
    auto packet = DuelCancelPacket::build();
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage("You have forfeited the duel.");
}

void SocialHandler::proposeDuel(uint64_t targetGuid) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (targetGuid == 0) { owner_.raiseUiError("You must target a player to challenge to a duel."); return; }
    auto packet = DuelProposedPacket::build(targetGuid);
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage("You have challenged your target to a duel.");
}

void SocialHandler::reportPlayer(uint64_t targetGuid, const std::string& reason) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (targetGuid == 0) {
        owner_.raiseUiError("You must target a player to report.");
        return;
    }
    auto packet = ComplainPacket::build(targetGuid, reason);
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage("Player report submitted.");
    LOG_INFO("Reported player: 0x", std::hex, targetGuid, std::dec, " reason=", reason);
}

void SocialHandler::handleDuelRequested(network::Packet& packet) {
    if (!packet.hasRemaining(16)) { packet.skipAll(); return; }
    duelChallengerGuid_ = packet.readUInt64();
    duelFlagGuid_       = packet.readUInt64();
    duelChallengerName_.clear();
    auto entity = owner_.getEntityManager().getEntity(duelChallengerGuid_);
    if (auto* unit = dynamic_cast<Unit*>(entity.get()))
        duelChallengerName_ = unit->getName();
    if (duelChallengerName_.empty()) {
        auto nit = owner_.getPlayerNameCache().find(duelChallengerGuid_);
        if (nit != owner_.getPlayerNameCache().end()) duelChallengerName_ = nit->second;
    }
    if (duelChallengerName_.empty()) {
        char tmp[32];
        std::snprintf(tmp, sizeof(tmp), "0x%llX", static_cast<unsigned long long>(duelChallengerGuid_));
        duelChallengerName_ = tmp;
    }
    pendingDuelRequest_ = true;
    owner_.addSystemChatMessage(duelChallengerName_ + " challenges you to a duel!");
    if (auto* ac = owner_.services().audioCoordinator)
        if (auto* sfx = ac->getUiSoundManager()) sfx->playTargetSelect();
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("DUEL_REQUESTED", {duelChallengerName_});
}

void SocialHandler::handleDuelComplete(network::Packet& packet) {
    if (!packet.hasRemaining(1)) return;
    uint8_t started = packet.readUInt8();
    pendingDuelRequest_ = false;
    duelCountdownMs_ = 0;
    if (!started) owner_.addSystemChatMessage("The duel was cancelled.");
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("DUEL_FINISHED", {});
}

void SocialHandler::handleDuelWinner(network::Packet& packet) {
    if (!packet.hasRemaining(3)) return;
    uint8_t duelType = packet.readUInt8();
    std::string winner = packet.readString();
    std::string loser  = packet.readString();
    std::string msg = (duelType == 1)
        ? loser + " has fled from the duel. " + winner + " wins!"
        : winner + " has defeated " + loser + " in a duel!";
    owner_.addSystemChatMessage(msg);
}

// ============================================================
// Party / Raid
// ============================================================

void SocialHandler::inviteToGroup(const std::string& playerName) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    LOG_WARNING(">>> Sending CMSG_GROUP_INVITE to '", playerName, "'");
    auto packet = GroupInvitePacket::build(playerName);
    owner_.getSocket()->send(packet);
}

void SocialHandler::acceptGroupInvite() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    LOG_WARNING(">>> Sending CMSG_GROUP_ACCEPT");
    pendingGroupInvite = false;
    auto packet = GroupAcceptPacket::build();
    owner_.getSocket()->send(packet);
}

void SocialHandler::declineGroupInvite() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    pendingGroupInvite = false;
    auto packet = GroupDeclinePacket::build();
    owner_.getSocket()->send(packet);
}

void SocialHandler::acceptArenaTeamInvite() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    network::Packet packet(wireOpcode(Opcode::CMSG_ARENA_TEAM_ACCEPT));
    owner_.getSocket()->send(packet);
}

void SocialHandler::declineArenaTeamInvite() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    network::Packet packet(wireOpcode(Opcode::CMSG_ARENA_TEAM_DECLINE));
    owner_.getSocket()->send(packet);
}

// The four arena team commands that had no sender. Every one is the team id
// and, for three of them, a player name - the shape AzerothCore reads in
// ArenaTeamHandler.cpp: `recvData >> arenaTeamId >> name`. Disband and the two
// invite answers below were already built; these four were named by the
// interface's own popups and slash commands and reached bindings with nothing
// behind them, so the buttons closed and nothing happened.
void SocialHandler::arenaTeamInvite(uint32_t teamId, const std::string& name) {
    if (teamId == 0 || name.empty()) return;
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    network::Packet packet(wireOpcode(Opcode::CMSG_ARENA_TEAM_INVITE));
    packet.writeUInt32(teamId);
    packet.writeString(name);
    owner_.getSocket()->send(packet);
}

void SocialHandler::arenaTeamLeave(uint32_t teamId) {
    if (teamId == 0) return;
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    network::Packet packet(wireOpcode(Opcode::CMSG_ARENA_TEAM_LEAVE));
    packet.writeUInt32(teamId);
    owner_.getSocket()->send(packet);
}

void SocialHandler::arenaTeamRemove(uint32_t teamId, const std::string& name) {
    if (teamId == 0 || name.empty()) return;
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    network::Packet packet(wireOpcode(Opcode::CMSG_ARENA_TEAM_REMOVE));
    packet.writeUInt32(teamId);
    packet.writeString(name);
    owner_.getSocket()->send(packet);
}

void SocialHandler::arenaTeamSetLeader(uint32_t teamId, const std::string& name) {
    if (teamId == 0 || name.empty()) return;
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    network::Packet packet(wireOpcode(Opcode::CMSG_ARENA_TEAM_LEADER));
    packet.writeUInt32(teamId);
    packet.writeString(name);
    owner_.getSocket()->send(packet);
}

void SocialHandler::disbandArenaTeam(uint32_t teamId) {
    if (teamId == 0) return;
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    network::Packet packet(wireOpcode(Opcode::CMSG_ARENA_TEAM_DISBAND));
    packet.writeUInt32(teamId);
    owner_.getSocket()->send(packet);
}

void SocialHandler::reportMailSpam(uint64_t senderGuid, uint32_t mailId) {
    if (senderGuid == 0) return;
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    network::Packet packet(wireOpcode(Opcode::CMSG_COMPLAIN));
    packet.writeUInt8(0);              // 0 = mail, 1 = chat
    packet.writeUInt64(senderGuid);
    packet.writeUInt32(0);
    packet.writeUInt32(mailId);
    packet.writeUInt32(0);
    owner_.getSocket()->send(packet);
}

void SocialHandler::leaveGroup() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = GroupDisbandPacket::build();
    owner_.getSocket()->send(packet);
    partyData = GroupListData{};
    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("GROUP_ROSTER_UPDATE", {});
        owner_.addonEventCallbackRef()("PARTY_MEMBERS_CHANGED", {});
    }
}

void SocialHandler::convertToRaid() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (!isInGroup()) {
        owner_.raiseUiError("You are not in a group.");
        return;
    }
    if (partyData.leaderGuid != owner_.getPlayerGuid()) {
        owner_.raiseUiError("You must be the party leader to convert to raid.");
        return;
    }
    if (partyData.groupType == 1) {
        owner_.raiseUiError("You are already in a raid group.");
        return;
    }
    auto packet = GroupRaidConvertPacket::build();
    owner_.getSocket()->send(packet);
    LOG_INFO("Sent CMSG_GROUP_RAID_CONVERT");
}

void SocialHandler::sendSetLootMethod(uint32_t method, uint32_t threshold, uint64_t masterLooterGuid) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = SetLootMethodPacket::build(method, threshold, masterLooterGuid);
    owner_.getSocket()->send(packet);
    LOG_INFO("sendSetLootMethod: method=", method, " threshold=", threshold);
}

// The server checks that this player leads the group and that the target is in
// it, so there is nothing to validate here beyond having a guid to send.
void SocialHandler::setGuildBankTabText(uint8_t tab, const std::string& text) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = GuildBankSetTabTextPacket::build(tab, text);
    owner_.getSocket()->send(packet);
}

void SocialHandler::channelModeration(Opcode op, const std::string& channelName,
                                     const std::string& targetName,
                                     bool allowEmptyTarget) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    // An empty second string is meaningless for the moderation commands - there
    // is no player called nothing - but it is how a channel password is
    // cleared, so that one caller says it means it.
    if (channelName.empty() || (targetName.empty() && !allowEmptyTarget)) return;
    auto packet = ChannelModerationPacket::build(op, channelName, targetName);
    owner_.getSocket()->send(packet);
}

void SocialHandler::promoteToLeader(uint64_t guid) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (guid == 0) return;
    auto packet = GroupSetLeaderPacket::build(guid);
    owner_.getSocket()->send(packet);
}

void SocialHandler::uninvitePlayer(const std::string& playerName) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (playerName.empty()) { owner_.raiseUiError("You must specify a player name to uninvite."); return; }
    auto packet = GroupUninvitePacket::build(playerName);
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage("Removed " + playerName + " from the group.");
}

// CMSG_LOOT_METHOD carries all three settings together, so changing one means
// resending the other two as they stand. The master looter is only meaningful
// for method 2 and is sent as an empty guid otherwise.
// Removes the lowest rank, which is what the server does with this - it takes
// no index, so there is nothing to get wrong about which one goes.
void SocialHandler::delGuildRank() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = GuildDelRankPacket::build();
    owner_.getSocket()->send(packet);
    requestGuildRoster();
}

void SocialHandler::setLootMethod(uint8_t method, uint64_t masterGuid, uint8_t threshold) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    network::Packet packet(wireOpcode(Opcode::CMSG_LOOT_METHOD));
    packet.writeUInt32(method);
    packet.writeUInt64(method == 2 ? masterGuid : 0);
    packet.writeUInt32(threshold);
    owner_.getSocket()->send(packet);
}

// MSG_PARTY_ASSIGNMENT: which assignment (0 main tank, 1 main assist), whether
// it is being given or taken away, and who.
void SocialHandler::setPartyAssignment(uint8_t assignment, uint64_t guid, bool apply) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (guid == 0) return;
    network::Packet packet(wireOpcode(Opcode::MSG_PARTY_ASSIGNMENT));
    packet.writeUInt8(assignment);
    packet.writeUInt8(apply ? 1 : 0);
    packet.writeUInt64(guid);
    owner_.getSocket()->send(packet);
}

// Move a raid member into a different group of eight. The server takes the
// player's name and the destination group as a zero-based index, while the raid
// UI counts its groups from one.
void SocialHandler::setRaidSubgroup(const std::string& playerName, uint8_t group) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (playerName.empty() || group < 1 || group > 8) return;
    network::Packet packet(wireOpcode(Opcode::CMSG_GROUP_CHANGE_SUB_GROUP));
    packet.writeString(playerName);
    packet.writeUInt8(static_cast<uint8_t>(group - 1));
    owner_.getSocket()->send(packet);
}

// Exchange two raid members' places. Used when the destination group is already
// full, which is why it is a separate packet rather than two moves.
void SocialHandler::swapRaidSubgroup(const std::string& firstName, const std::string& secondName) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (firstName.empty() || secondName.empty()) return;
    network::Packet packet(wireOpcode(Opcode::CMSG_GROUP_SWAP_SUB_GROUP));
    packet.writeString(firstName);
    packet.writeString(secondName);
    owner_.getSocket()->send(packet);
}

void SocialHandler::leaveParty() {
    // Delegates to leaveGroup which handles both the packet send AND local
    // state cleanup (clearing partyData, firing addon events). Previously
    // this method only sent the packet, leaving the client thinking it was
    // still in a group until the server pushed a group list update.
    leaveGroup();
}

void SocialHandler::setMainTank(uint64_t targetGuid) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (targetGuid == 0) { owner_.raiseUiError("You must have a target selected."); return; }
    auto packet = RaidTargetUpdatePacket::build(0, targetGuid);
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage("Main tank set.");
}

void SocialHandler::setMainAssist(uint64_t targetGuid) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (targetGuid == 0) { owner_.raiseUiError("You must have a target selected."); return; }
    auto packet = RaidTargetUpdatePacket::build(1, targetGuid);
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage("Main assist set.");
}

void SocialHandler::clearMainTank() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = RaidTargetUpdatePacket::build(0, 0);
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage("Main tank cleared.");
}

void SocialHandler::clearMainAssist() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = RaidTargetUpdatePacket::build(1, 0);
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage("Main assist cleared.");
}

void SocialHandler::setRaidMarkLocally(uint64_t guid, uint8_t icon) {
    // Mirrors what the server does for a group: an icon is unique, and a unit
    // carries at most one mark, so placing one clears any previous holder.
    if (icon == 0xFF) {
        for (uint32_t i = 0; i < kRaidMarkCount; ++i) {
            if (raidTargetGuids_[i] == guid) raidTargetGuids_[i] = 0;
        }
    } else if (icon < kRaidMarkCount) {
        for (uint32_t i = 0; i < kRaidMarkCount; ++i) {
            if (raidTargetGuids_[i] == guid) raidTargetGuids_[i] = 0;
        }
        raidTargetGuids_[icon] = guid;
    } else {
        return;
    }
    owner_.fireAddonEvent("RAID_TARGET_UPDATE", {});
}

void SocialHandler::setRaidMark(uint64_t guid, uint8_t icon) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    // Raid marks are group state on the server: it drops MSG_RAID_TARGET_UPDATE
    // from an ungrouped player and broadcasts nothing back, so a solo mark used
    // to do nothing at all. Apply it locally instead - a deliberate divergence
    // from Blizzlike behaviour, since a solo player marking their own targets
    // harms nobody and the alternative is a feature that cannot be used, or
    // tested, without a second player. Joining a group later is safe: the
    // server's full list is authoritative and replaces these marks wholesale.
    if (!isInGroup()) {
        setRaidMarkLocally(guid, icon);
        return;
    }
    if (icon == 0xFF) {
        for (int i = 0; i < 8; ++i) {
            if (raidTargetGuids_[i] == guid) {
                auto packet = RaidTargetUpdatePacket::build(static_cast<uint8_t>(i), 0);
                owner_.getSocket()->send(packet);
                break;
            }
        }
    } else if (icon < 8) {
        auto packet = RaidTargetUpdatePacket::build(icon, guid);
        owner_.getSocket()->send(packet);
    }
}

void SocialHandler::setSavedInstanceExtend(uint32_t mapId, uint32_t difficulty, bool extend) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    // uint32 mapId, uint32 difficulty, uint8 toggle - HandleSetSavedInstanceExtend.
    // The server ignores it unless the bind is permanent and the flag actually
    // changes, so there is nothing to check here that it does not check better.
    network::Packet packet(wireOpcode(Opcode::CMSG_SET_SAVED_INSTANCE_EXTEND));
    packet.writeUInt32(mapId);
    packet.writeUInt32(difficulty);
    packet.writeUInt8(extend ? 1 : 0);
    owner_.getSocket()->send(packet);
}

void SocialHandler::requestRaidInfo() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = RequestRaidInfoPacket::build();
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage("Requesting raid lockout information...");
}

// ============================================================
// Group Handlers
// ============================================================

void SocialHandler::handleGroupInvite(network::Packet& packet) {
    GroupInviteResponseData data;
    const bool hasCanAccept = !isPreWotlk();
    if (!GroupInviteResponseParser::parse(packet, data, hasCanAccept)) return;

    // The leading byte is not decoration: AzerothCore sends this same opcode
    // with a zero in it to say "you were invited and it could not be offered,
    // because you are already in a group" - its own comment beside the write,
    // and the interface carries Blizzard's sentence for exactly that case
    // (ERR_INVITED_ALREADY_IN_GROUP_SS).
    //
    // It was read off the wire and then ignored, so that refusal raised the
    // accept/decline popup, played the invitation sound and left an invite
    // pending that the server would never honour. Answering it is a refusal,
    // not an offer.
    if (!data.canAccept) {
        owner_.addSystemChatMessage(
            "|Hplayer:" + data.inviterName + "|h[" + data.inviterName +
            "]|h invited you to a group, but you could not accept because you "
            "are already in a group.");
        return;
    }

    pendingGroupInvite = true;
    pendingInviterName = data.inviterName;
    owner_.addSystemChatMessage(data.inviterName + " has invited you to a group.");
    owner_.addUIError(data.inviterName + " has invited you to join a group.");
    if (auto* ac = owner_.services().audioCoordinator)
        if (auto* sfx = ac->getUiSoundManager()) sfx->playQuestActivate();
    if (owner_.addonEventCallbackRef())
        owner_.addonEventCallbackRef()("PARTY_INVITE_REQUEST", {data.inviterName});
}

void SocialHandler::handleGroupDecline(network::Packet& packet) {
    GroupDeclineData data;
    if (!GroupDeclineResponseParser::parse(packet, data)) return;
    owner_.addSystemChatMessage(data.playerName + " has declined your group invitation.");
}

void SocialHandler::handleGroupList(network::Packet& packet) {
    const bool hasRoles = isActiveExpansion("wotlk");
    const bool hasBattleGroupFlag = isActiveExpansion("tbc");
    const uint8_t prevLootMethod = partyData.lootMethod;
    const bool wasInGroup = !partyData.isEmpty();
    partyData = GroupListData{};
    if (!GroupListParser::parse(packet, partyData, hasRoles, hasBattleGroupFlag)) return;

    // The roster carries online state in isOnline; the per-member stat updates
    // carry it in the onlineStatus bitmask. Seed the bitmask from the roster so
    // both readers agree until the first SMSG_PARTY_MEMBER_STATS arrives -
    // partyData was just reset, so every onlineStatus is otherwise zero and
    // every member would read as offline.
    for (auto& m : partyData.members) {
        if (m.isOnline) m.onlineStatus |= 0x0001;
    }

    const bool nowInGroup = !partyData.isEmpty();
    if (!nowInGroup && wasInGroup) {
        owner_.addSystemChatMessage("You are no longer in a group.");
        LOG_INFO("Left group");
    } else if (nowInGroup && !wasInGroup) {
        std::string members;
        for (const auto& m : partyData.members) {
            if (!members.empty()) members += ", ";
            members += m.name.empty() ? "?" : m.name;
        }
        owner_.addSystemChatMessage("You joined a group with: " + members);
        LOG_INFO("Joined group with ", partyData.memberCount, " members: ", members);
    }
    // Loot method change notification
    if (wasInGroup && nowInGroup && partyData.lootMethod != prevLootMethod) {
        const char* methodName = lootMethodName(partyData.lootMethod);
        owner_.addSystemChatMessage(std::string("Loot method changed to ") + methodName + ".");
        // Already noticed, and only ever said in chat. The party frames read
        // the method to decide whether to offer the master-looter menu, and
        // they reread it on this - so the change was announced to the player
        // and not to the interface standing next to them.
        if (owner_.addonEventCallbackRef())
            owner_.addonEventCallbackRef()("PARTY_LOOT_METHOD_CHANGED", {});
    }
    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("GROUP_ROSTER_UPDATE", {});
        owner_.addonEventCallbackRef()("PARTY_MEMBERS_CHANGED", {});
        // The roles came with this roster and UnitGroupRolesAssigned reads them
        // straight out of it. The player frame draws its tank or healer badge
        // on this event and nothing else, so the roles were parsed, readable,
        // and never shown.
        owner_.addonEventCallbackRef()("PLAYER_ROLES_ASSIGNED", {});
        if (partyData.groupType == 1)
            owner_.addonEventCallbackRef()("RAID_ROSTER_UPDATE", {});
    }
}

void SocialHandler::handleGroupUninvite(network::Packet& packet) {
    (void)packet;
    partyData = GroupListData{};
    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("GROUP_ROSTER_UPDATE", {});
        owner_.addonEventCallbackRef()("PARTY_MEMBERS_CHANGED", {});
        owner_.addonEventCallbackRef()("RAID_ROSTER_UPDATE", {});
    }
    owner_.addUIError("You have been removed from the group.");
    owner_.addSystemChatMessage("You have been removed from the group.");
}

void SocialHandler::handlePartyCommandResult(network::Packet& packet) {
    PartyCommandResultData data;
    if (!PartyCommandResultParser::parse(packet, data)) return;
    if (data.result != PartyResult::OK) {
        const char* errText = nullptr;
        switch (data.result) {
            case PartyResult::BAD_PLAYER_NAME:       errText = "No player named \"%s\" is currently online."; break;
            case PartyResult::TARGET_NOT_IN_GROUP:   errText = "%s is not in your group."; break;
            case PartyResult::TARGET_NOT_IN_INSTANCE:errText = "%s is not in your instance."; break;
            case PartyResult::GROUP_FULL:            errText = "Your party is full."; break;
            case PartyResult::ALREADY_IN_GROUP:      errText = "%s is already in a group."; break;
            case PartyResult::NOT_IN_GROUP:          errText = "You are not in a group."; break;
            case PartyResult::NOT_LEADER:            errText = "You are not the group leader."; break;
            case PartyResult::PLAYER_WRONG_FACTION:  errText = "%s is the wrong faction for this group."; break;
            case PartyResult::IGNORING_YOU:          errText = "%s is ignoring you."; break;
            case PartyResult::LFG_PENDING:           errText = "You cannot do that while in a LFG queue."; break;
            case PartyResult::INVITE_RESTRICTED:     errText = "Target is not accepting group invites."; break;
            default:                                 errText = "Party command failed."; break;
        }
        char buf[256];
        if (!data.name.empty() && errText && std::strstr(errText, "%s"))
            std::snprintf(buf, sizeof(buf), errText, data.name.c_str());
        else if (errText)
            std::snprintf(buf, sizeof(buf), "%s", errText);
        else
            std::snprintf(buf, sizeof(buf), "Party command failed (error %u).", static_cast<uint32_t>(data.result));
        owner_.addUIError(buf);
        owner_.addSystemChatMessage(buf);
    }
}

void SocialHandler::handlePartyMemberStats(network::Packet& packet, bool isFull) {
    auto remaining = [&]() { return packet.getRemainingSize(); };
    const bool isWotLK = isActiveExpansion("wotlk");

    if (isFull) { if (remaining() < 1) return; packet.readUInt8(); }

    const bool pmsTbc = isActiveExpansion("tbc");
    if (remaining() < (pmsTbc ? 8u : 1u)) return;
    uint64_t memberGuid = pmsTbc
        ? packet.readUInt64() : packet.readPackedGuid();
    if (remaining() < 4) return;
    uint32_t updateFlags = packet.readUInt32();

    game::GroupMember* member = nullptr;
    for (auto& m : partyData.members) {
        if (m.guid == memberGuid) { member = &m; break; }
    }
    if (!member) { packet.skipAll(); return; }

    // Track the online transition so it can be announced once the packet is
    // fully read. isOnline is what the Lua bindings read; onlineStatus is what
    // this client's own panels read. Both describe the same fact, so keep them
    // in step here - otherwise UnitIsConnected answers with the stale roster
    // snapshot for as long as the group lasts.
    const bool wasOnline = (member->onlineStatus & 0x0001) != 0;
    bool onlineChanged = false;
    if (updateFlags & 0x0001) {
        if (remaining() >= 2) {
            member->onlineStatus = packet.readUInt16();
            const bool nowOnline = (member->onlineStatus & 0x0001) != 0;
            member->isOnline = nowOnline ? 1 : 0;
            onlineChanged = (nowOnline != wasOnline);
        }
    }
    if (updateFlags & 0x0002) {
        if (isWotLK) { if (remaining() >= 4) member->curHealth = packet.readUInt32(); }
        else { if (remaining() >= 2) member->curHealth = packet.readUInt16(); }
    }
    if (updateFlags & 0x0004) {
        if (isWotLK) { if (remaining() >= 4) member->maxHealth = packet.readUInt32(); }
        else { if (remaining() >= 2) member->maxHealth = packet.readUInt16(); }
    }
    if (updateFlags & 0x0008) { if (remaining() >= 1) member->powerType = packet.readUInt8(); }
    if (updateFlags & 0x0010) { if (remaining() >= 2) member->curPower = packet.readUInt16(); }
    if (updateFlags & 0x0020) { if (remaining() >= 2) member->maxPower = packet.readUInt16(); }
    if (updateFlags & 0x0040) { if (remaining() >= 2) member->level = packet.readUInt16(); }
    if (updateFlags & 0x0080) { if (remaining() >= 2) member->zoneId = packet.readUInt16(); }
    if (updateFlags & 0x0100) {
        if (remaining() >= 4) {
            member->posX = static_cast<int16_t>(packet.readUInt16());
            member->posY = static_cast<int16_t>(packet.readUInt16());
        }
    }
    if (updateFlags & 0x0200) {
        if (remaining() >= 8) {
            uint64_t auraMask = packet.readUInt64();
            std::vector<AuraSlot> newAuras;
            for (int i = 0; i < 64; ++i) {
                if (auraMask & (uint64_t(1) << i)) {
                    AuraSlot a;
                    a.level = static_cast<uint8_t>(i);
                    if (isWotLK) {
                        if (remaining() < 5) break;
                        a.spellId = packet.readUInt32();
                        a.flags   = packet.readUInt8();
                    } else {
                        if (remaining() < 2) break;
                        a.spellId = packet.readUInt16();
                        uint8_t dt = owner_.getSpellDispelType(a.spellId);
                        if (dt > 0) a.flags = 0x80;
                    }
                    if (a.spellId != 0) newAuras.push_back(a);
                }
            }
            if (memberGuid != 0 && memberGuid != owner_.getPlayerGuid() && memberGuid != owner_.getTargetGuid()) {
                owner_.unitAurasCacheRef()[memberGuid] = std::move(newAuras);
            }
        }
    }
    // Skip pet fields and vehicle seat
    if (updateFlags & 0x0400) { if (remaining() >= 8) packet.readUInt64(); }
    if (updateFlags & 0x0800) { if (remaining() > 0) packet.readString(); }
    if (updateFlags & 0x1000) { if (remaining() >= 2) packet.readUInt16(); }
    if (updateFlags & 0x2000) { if (isWotLK) { if (remaining() >= 4) packet.readUInt32(); } else { if (remaining() >= 2) packet.readUInt16(); } }
    if (updateFlags & 0x4000) { if (isWotLK) { if (remaining() >= 4) packet.readUInt32(); } else { if (remaining() >= 2) packet.readUInt16(); } }
    if (updateFlags & 0x8000) { if (remaining() >= 1) packet.readUInt8(); }
    if (updateFlags & 0x10000) { if (remaining() >= 2) packet.readUInt16(); }
    if (updateFlags & 0x20000) { if (remaining() >= 2) packet.readUInt16(); }
    if (updateFlags & 0x40000) {
        if (remaining() >= 8) {
            uint64_t petAuraMask = packet.readUInt64();
            for (int i = 0; i < 64; ++i) {
                if (petAuraMask & (uint64_t(1) << i)) {
                    if (isWotLK) { if (remaining() < 5) break; packet.readUInt32(); packet.readUInt8(); }
                    else { if (remaining() < 2) break; packet.readUInt16(); }
                }
            }
        }
    }
    if (isWotLK && (updateFlags & 0x80000)) { if (remaining() >= 4) packet.readUInt32(); }

    member->hasPartyStats = true;

    if (owner_.addonEventCallbackRef()) {
        std::string unitId;
        if (partyData.groupType == 1) {
            for (size_t i = 0; i < partyData.members.size(); ++i) {
                if (partyData.members[i].guid == memberGuid) { unitId = "raid" + std::to_string(i + 1); break; }
            }
        } else {
            int found = 0;
            for (const auto& m : partyData.members) {
                if (m.guid == owner_.getPlayerGuid()) continue;
                ++found;
                if (m.guid == memberGuid) { unitId = "party" + std::to_string(found); break; }
            }
        }
        if (!unitId.empty()) {
            if (updateFlags & (0x0002 | 0x0004)) owner_.addonEventCallbackRef()("UNIT_HEALTH", {unitId});
            if (updateFlags & (0x0010 | 0x0020)) owner_.addonEventCallbackRef()("UNIT_POWER", {unitId});
            if (updateFlags & 0x0200) owner_.addonEventCallbackRef()("UNIT_AURA", {unitId});
        }
        // Fired without arguments, as the retail client does - the interface
        // re-reads the whole roster rather than acting on one member. Not
        // gated on unitId: the player's own stats resolve to no unit token,
        // and a member dropping is worth announcing either way.
        if (onlineChanged) {
            owner_.addonEventCallbackRef()(
                member->isOnline ? "PARTY_MEMBER_ENABLE" : "PARTY_MEMBER_DISABLE", {});
        }
    }
}

// ============================================================
// Guild Handlers
// ============================================================

void SocialHandler::handleGuildInfo(network::Packet& packet) {
    GuildInfoData data;
    if (!GuildInfoParser::parse(packet, data)) return;
    if (!data.guildName.empty()) {
        if (const Character* ch = owner_.getActiveCharacter(); ch && ch->hasGuild()) {
            rememberGuildName(ch->guildId, data.guildName);
            if (guildName_.empty()) guildName_ = data.guildName;
        }
    }
    // SMSG_GUILD_INFO is pushed by the server on every guild roster sync
    // (login, member join/leave, periodic refreshes). Only print when
    // something the user can see has actually changed; otherwise the
    // guild banner spams chat on every tick.
    const bool changed = (guildInfoData_.guildName != data.guildName) ||
                         (guildInfoData_.numMembers != data.numMembers) ||
                         (guildInfoData_.numAccounts != data.numAccounts);
    guildInfoData_ = data;
    if (changed) {
        owner_.addSystemChatMessage("Guild: " + data.guildName + " (" +
                             std::to_string(data.numMembers) + " members, " +
                             std::to_string(data.numAccounts) + " accounts)");
    }
}

void SocialHandler::handleGuildRoster(network::Packet& packet) {
    GuildRosterData data;
    if (!owner_.getPacketParsers()->parseGuildRoster(packet, data)) return;
    guildRoster_ = std::move(data);
    hasGuildRoster_ = true;
    // False: the roster in hand is the fresh one, so nothing should ask again.
    // The argument is "you may request a roster", and every reader answers it
    // with `if arg1 then GuildRoster() end` - friendsframe, the guild bank and
    // the calendar all do exactly that. Sending true here would have each of
    // them request a roster, whose reply would fire this again, forever.
    if (owner_.addonEventCallbackRef())
        owner_.addonEventCallbackRef()("GUILD_ROSTER_UPDATE", {eventBool(false)});
}

// CMSG_GUILD_RANK rewrites the rank whole: id, rights, name, gold per day, then
// six pairs of bank tab rights and slots. Everything it carries has to be sent,
// which is why the caller stages from the rank's current values rather than
// from nothing - a field left at zero here is not "unchanged", it is revoked.
void SocialHandler::saveGuildRank(uint32_t rankId, uint32_t rights, const std::string& rankName,
                                  uint32_t goldLimit, const uint32_t* tabRights,
                                  const uint32_t* tabSlots) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    network::Packet packet(wireOpcode(Opcode::CMSG_GUILD_RANK));
    packet.writeUInt32(rankId);
    packet.writeUInt32(rights);
    packet.writeString(rankName);
    packet.writeUInt32(goldLimit);
    for (int t = 0; t < 6; ++t) {
        packet.writeUInt32(tabRights ? tabRights[t] : 0);
        packet.writeUInt32(tabSlots ? tabSlots[t] : 0);
    }
    owner_.getSocket()->send(packet);
    requestGuildRoster();
}

uint32_t SocialHandler::getPlayerGuildRankIndex() const {
    if (!hasGuildRoster_) return 0xFFFFFFFFu;
    const uint64_t me = owner_.getPlayerGuid();
    for (const auto& m : guildRoster_.members)
        if (m.guid == me) return m.rankIndex;
    return 0xFFFFFFFFu;
}

uint32_t SocialHandler::getPlayerGuildRankRights() const {
    const uint32_t idx = getPlayerGuildRankIndex();
    if (idx >= guildRoster_.ranks.size()) return 0;
    return guildRoster_.ranks[idx].rights;
}

void SocialHandler::handleGuildQueryResponse(network::Packet& packet) {
    GuildQueryResponseData data;
    if (!owner_.getPacketParsers()->parseGuildQueryResponse(packet, data)) return;
    if (data.guildId != 0) {
        pendingGuildNameQueries_.erase(data.guildId);
        if (!data.guildName.empty()) {
            rememberGuildName(data.guildId, data.guildName);
        }
    }
    const Character* ch = owner_.getActiveCharacter();
    bool isLocalGuild = (ch && ch->hasGuild() && ch->guildId == data.guildId);
    if (isLocalGuild) {
        const bool wasUnknown = guildName_.empty();
        guildName_ = data.guildName;
        guildQueryData_ = data;
        guildRankNames_.clear();
        for (const auto& rankName : data.rankNames) guildRankNames_.push_back(rankName);
        if (wasUnknown && !guildName_.empty()) {
            owner_.addSystemChatMessage("Guild: <" + guildName_ + ">");
            if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("PLAYER_GUILD_UPDATE", {});
        }
    }
}

// Returns true only when the member's cached online state actually changes (and updates
// it), so callers announce genuine online/offline transitions and silently drop the
// redundant SIGNED_ON flood the server emits at login for already-online members.
bool SocialHandler::guildMemberOnlineTransition(const std::string& name, bool nowOnline) {
    for (auto& m : guildRoster_.members) {
        if (m.name == name) {
            if (m.online == nowOnline) return false;
            m.online = nowOnline;
            return true;
        }
    }
    return true;  // not in the cached roster yet - announce rather than swallow
}

void SocialHandler::handleGuildEvent(network::Packet& packet) {
    GuildEventData data;
    if (!GuildEventParser::parse(packet, data)) return;

    std::string msg;
    switch (data.eventType) {
        case GuildEvent::PROMOTION:
            if (data.numStrings >= 3)
                msg = data.strings[0] + " has promoted " + data.strings[1] + " to " + data.strings[2] + ".";
            break;
        case GuildEvent::DEMOTION:
            if (data.numStrings >= 3)
                msg = data.strings[0] + " has demoted " + data.strings[1] + " to " + data.strings[2] + ".";
            break;
        case GuildEvent::MOTD:
            // The interface writes this one itself, from the GUILD_MOTD event
            // fired further down: chatframe.lua formats GUILD_MOTD_TEMPLATE
            // and adds it. Only the client's own window needs a line here.
            if (data.numStrings >= 1 && !ui::frameXmlOwns(ui::UiElement::Chat))
                msg = "Guild MOTD: " + data.strings[0];
            break;
        case GuildEvent::JOINED:
            if (data.numStrings >= 1) msg = data.strings[0] + " has joined the guild.";
            break;
        case GuildEvent::LEFT:
            if (data.numStrings >= 1) msg = data.strings[0] + " has left the guild.";
            break;
        case GuildEvent::REMOVED:
            if (data.numStrings >= 2) msg = data.strings[1] + " has been kicked from the guild by " + data.strings[0] + ".";
            break;
        case GuildEvent::LEADER_IS:
            if (data.numStrings >= 1) msg = data.strings[0] + " is the guild leader.";
            break;
        case GuildEvent::LEADER_CHANGED:
            if (data.numStrings >= 2) msg = data.strings[0] + " has made " + data.strings[1] + " the new guild leader.";
            break;
        case GuildEvent::DISBANDED:
            msg = "Guild has been disbanded.";
            guildName_.clear();
            guildRankNames_.clear();
            guildRoster_ = GuildRosterData{};
            hasGuildRoster_ = false;
            if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("PLAYER_GUILD_UPDATE", {});
            break;
        case GuildEvent::SIGNED_ON:
            // Only announce a real offline→online transition. On login the server floods
            // SIGNED_ON for members the roster already lists as online; suppressing those
            // (state unchanged) matches the real client and stops the guild-chat spam.
            //
            // ...and only when the player wants to hear about it at all. Guild
            // Member Notify is offered by the panel and was read by nothing, so
            // a large guild announced every arrival and departure whatever it
            // said. The transition is still tracked when the message is
            // suppressed - the roster's idea of who is online is not the
            // player's idea of what is worth reading.
            if (data.numStrings >= 1 && guildMemberOnlineTransition(data.strings[0], true) &&
                addons::storedCVarValue("guildMemberNotify", "1") != "0")
                msg = "[Guild] " + data.strings[0] + " has come online.";
            break;
        case GuildEvent::SIGNED_OFF:
            if (data.numStrings >= 1 && guildMemberOnlineTransition(data.strings[0], false) &&
                addons::storedCVarValue("guildMemberNotify", "1") != "0")
                msg = "[Guild] " + data.strings[0] + " has gone offline.";
            break;
        default:
            msg = "Guild event " + std::to_string(data.eventType);
            // Was `!numStrings && numStrings >= 1` - always false (0 can't be ≥1).
            if (data.numStrings >= 1) msg += ": " + data.strings[0];
            break;
    }

    if (!msg.empty()) {
        MessageChatData chatMsg;
        chatMsg.type = ChatType::GUILD;
        chatMsg.language = ChatLanguage::UNIVERSAL;
        chatMsg.message = msg;
        owner_.addLocalChatMessage(chatMsg);
    }

    // Whether this client is about to ask for a fresh roster itself, which is
    // the switch at the end of this function. The event's argument is the
    // negation: "nobody has asked, so you may". Working it out once and using
    // it in both places is the point - FrameXML requesting when the client
    // already has costs a second roster for a large guild, and FrameXML not
    // requesting when the client has not leaves every guild panel drawing from
    // a copy that no longer says who is online.
    const bool clientWillRequest = hasGuildRoster_ && (
        data.eventType == GuildEvent::PROMOTION ||
        data.eventType == GuildEvent::DEMOTION ||
        data.eventType == GuildEvent::JOINED ||
        data.eventType == GuildEvent::LEFT ||
        data.eventType == GuildEvent::REMOVED ||
        data.eventType == GuildEvent::LEADER_CHANGED);

    if (owner_.addonEventCallbackRef()) {
        switch (data.eventType) {
            case GuildEvent::MOTD:
                owner_.addonEventCallbackRef()("GUILD_MOTD", {data.numStrings >= 1 ? data.strings[0] : ""});
                break;
            case GuildEvent::SIGNED_ON: case GuildEvent::SIGNED_OFF:
            case GuildEvent::PROMOTION: case GuildEvent::DEMOTION:
            case GuildEvent::JOINED: case GuildEvent::LEFT:
            case GuildEvent::REMOVED: case GuildEvent::LEADER_CHANGED:
            case GuildEvent::DISBANDED:
                // Somebody joined, left, was promoted or signed on, and the
                // roster this client holds no longer says so. The server sends
                // the event and not a new roster, so the request has to come
                // from somewhere. Signing on and off is the case nothing
                // covered: the switch below does not request for those, so a
                // member who had just come online stayed grey until something
                // else asked.
                owner_.addonEventCallbackRef()("GUILD_ROSTER_UPDATE",
                                               {eventBool(!clientWillRequest)});
                break;
            default: break;
        }
    }

    if (clientWillRequest) requestGuildRoster();
}

void SocialHandler::handleGuildInvite(network::Packet& packet) {
    GuildInviteResponseData data;
    if (!GuildInviteResponseParser::parse(packet, data)) return;
    pendingGuildInvite_ = true;
    pendingGuildInviterName_ = data.inviterName;
    pendingGuildInviteGuildName_ = data.guildName;
    owner_.addSystemChatMessage(data.inviterName + " has invited you to join " + data.guildName + ".");
    if (owner_.addonEventCallbackRef())
        owner_.addonEventCallbackRef()("GUILD_INVITE_REQUEST", {data.inviterName, data.guildName});
}

void SocialHandler::handleGuildCommandResult(network::Packet& packet) {
    GuildCommandResultData data;
    if (!GuildCommandResultParser::parse(packet, data)) return;
    if (data.errorCode == 0) {
        // Whether the player's own membership changed, which is a different
        // question from whether the command worked: inviting someone succeeds
        // without altering anything the panels read.
        bool membershipChanged = false;
        switch (data.command) {
            case 0:
                owner_.addSystemChatMessage("Guild created.");
                membershipChanged = true;
                break;
            case 1:
                if (!data.name.empty()) owner_.addSystemChatMessage("You have invited " + data.name + " to the guild.");
                break;
            case 2:
                owner_.addSystemChatMessage("You have left the guild.");
                guildName_.clear(); guildRankNames_.clear();
                guildRoster_ = GuildRosterData{}; hasGuildRoster_ = false;
                membershipChanged = true;
                break;
            default: break;
        }
        // Leaving a guild emptied the name, the ranks and the whole roster and
        // told nobody, so the interface's guild tab kept showing the guild the
        // player had just left - every row of it, until something else
        // happened to fire a roster update. PLAYER_GUILD_UPDATE is what
        // friendsframe and the paperdoll re-check IsInGuild on.
        //
        // The roster event carries false, which is the established contract
        // here: every reader answers a true with `if arg1 then GuildRoster()
        // end`, and the reply to that would fire this again without end.
        if (membershipChanged && owner_.addonEventCallbackRef()) {
            owner_.addonEventCallbackRef()("PLAYER_GUILD_UPDATE", {});
            owner_.addonEventCallbackRef()("GUILD_ROSTER_UPDATE", {eventBool(false)});
        }
        return;
    }
    const char* errStr = nullptr;
    switch (data.errorCode) {
        case 2:  errStr = "You are not in a guild."; break;
        case 4:  errStr = "No player named \"%s\" is online."; break;
        case 11: errStr = "\"%s\" is already in a guild."; break;
        case 13: errStr = "You are already in a guild."; break;
        case 14: errStr = "\"%s\" has already been invited to a guild."; break;
        case 16: case 17: errStr = "You are not the guild leader."; break;
        case 22: errStr = "That player is ignoring you."; break;
        default: break;
    }
    std::string errorMsg;
    if (errStr) {
        std::string fmt = errStr;
        auto pos = fmt.find("%s");
        if (pos != std::string::npos && !data.name.empty()) fmt.replace(pos, 2, data.name);
        else if (pos != std::string::npos) fmt.replace(pos, 2, "that player");
        errorMsg = fmt;
    } else {
        errorMsg = "Guild command failed";
        if (!data.name.empty()) errorMsg += " for " + data.name;
        errorMsg += " (error " + std::to_string(data.errorCode) + ")";
    }
    owner_.addUIError(errorMsg);
    owner_.addSystemChatMessage(errorMsg);
}

void SocialHandler::handlePetitionShowlist(network::Packet& packet) {
    PetitionShowlistData data;
    if (!PetitionShowlistParser::parse(packet, data)) return;
    petitionNpcGuid_ = data.npcGuid;
    petitionCost_ = data.cost;
    // The whole offer, not just the first charter's price. An arena registrar
    // offers three and the panel prices whichever tab is open.
    petitionCharters_ = data.charters;
    showPetitionDialog_ = true;
    petitionIsGuildCharter_ = data.isGuildCharter();
    // The vendor panel opens on this. showPetitionDialog_ is what this
    // client's own dialog reads, and the interface's version was told
    // nothing, so walking up to a guild master or an arena registrar opened
    // one of the two windows and never the other.
    //
    // Two panels, one opcode: the guild registrar and the arena registrar are
    // told apart by which charter is on offer, not by what was sent. Firing
    // the arena event at a tabard designer put the wrong window on screen.
    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()(
            petitionIsGuildCharter_ ? "GUILD_REGISTRAR_SHOW" : "PETITION_VENDOR_SHOW", {});
    }
}

void SocialHandler::closePetitionVendor() {
    if (!showPetitionDialog_) return;
    showPetitionDialog_ = false;
    petitionNpcGuid_ = 0;
    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()(
            petitionIsGuildCharter_ ? "GUILD_REGISTRAR_CLOSED" : "PETITION_VENDOR_CLOSED", {});
    }
}

void SocialHandler::handlePetitionQueryResponse(network::Packet& packet) {
    auto rem = [&]() { return packet.getRemainingSize(); };
    if (rem() < 12) return;
    /*uint32_t entry =*/ packet.readUInt32();
    uint64_t petGuid = packet.readUInt64();
    std::string guildName = packet.readString();
    /*std::string body =*/ packet.readString();

    // Three words the reply carries that used to be skipped along with the
    // rest of it. The first is how many signatures the charter needs - nothing
    // else on the wire says, so signaturesRequired sat at its default of nine
    // for every charter, and the counter under the signature list read "x / 9"
    // whatever the realm was configured for. CanSignPetition compares against
    // the same number, so the Sign button stayed live past the real
    // requirement.
    //
    // The third tells a guild charter from an arena one: AzerothCore writes
    // zero for a guild and the team size otherwise. GetPetitionInfo answered
    // "guild" for everything, which is the wrong heading and the wrong
    // confirmation on all three arena charters.
    uint32_t needed = 0, arenaType = 0;
    if (rem() >= 12) {
        needed = packet.readUInt32();
        /*uint32_t neededAgain =*/ packet.readUInt32();
        arenaType = packet.readUInt32();
    }
    packet.skipAll();

    if (petitionInfo_.petitionGuid != petGuid) return;
    petitionInfo_.guildName = guildName;
    if (needed > 0) petitionInfo_.signaturesRequired = needed;
    petitionInfo_.isArena = arenaType != 0;

    // Said again, because the frame was drawn when the signatures arrived and
    // everything above lands after it. Without this the charter keeps the
    // blank name and the placeholder count it opened with - the shape that
    // reads as "only right after reopening it".
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("PETITION_SHOW", {});
}

void SocialHandler::handlePetitionShowSignatures(network::Packet& packet) {
    auto rem = [&]() { return packet.getRemainingSize(); };
    if (rem() < 21) return;
    petitionInfo_ = PetitionInfo{};
    petitionInfo_.petitionGuid = packet.readUInt64();
    petitionInfo_.ownerGuid    = packet.readUInt64();
    const uint32_t petEntry    = packet.readUInt32();
    uint8_t sigCount           = packet.readUInt8();
    petitionInfo_.signatureCount = sigCount;
    petitionInfo_.signatures.reserve(sigCount);
    for (uint8_t i = 0; i < sigCount; ++i) {
        if (rem() < 12) break;
        PetitionSignature sig;
        sig.playerGuid = packet.readUInt64();
        /*uint32_t unk =*/ packet.readUInt32();
        petitionInfo_.signatures.push_back(sig);
    }
    petitionInfo_.showUI = true;

    // Ask what this charter actually is. Everything the frame needs beyond the
    // signatures - the name, the number of signatures required, and whether it
    // is a guild or an arena charter - comes only from the query reply, and
    // this client never sent the query. The opcode was in the table and the
    // reply had a handler waiting for it; nothing ever asked, so the handler
    // never ran and the frame opened on a nameless charter needing nine
    // signatures whatever it was.
    if (owner_.getSocket()) {
        auto query = PetitionQueryPacket::build(petEntry, petitionInfo_.petitionGuid);
        owner_.getSocket()->send(query);
    }

    // The signature list frame opens on this. The petition is parsed and
    // stored above - showUI is what this client's own window reads - and the
    // interface's version was never told, so a charter could not be signed
    // through it.
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("PETITION_SHOW", {});
}

void SocialHandler::handlePetitionSignResults(network::Packet& packet) {
    auto rem = [&]() { return packet.getRemainingSize(); };
    if (rem() < 20) return;
    uint64_t petGuid    = packet.readUInt64();
    uint64_t playerGuid = packet.readUInt64();
    uint32_t result     = packet.readUInt32();
    switch (result) {
        case 0:
            owner_.addSystemChatMessage("Petition signed successfully.");
            if (petitionInfo_.petitionGuid == petGuid) {
                petitionInfo_.signatureCount++;
                PetitionSignature sig; sig.playerGuid = playerGuid;
                petitionInfo_.signatures.push_back(sig);
            }
            break;
        case 1: owner_.raiseUiError("You have already signed that petition."); break;
        case 2: owner_.raiseUiError("You are already in a guild."); break;
        case 3: owner_.raiseUiError("You cannot sign your own petition."); break;
        default: owner_.addSystemChatMessage("Cannot sign petition (error " + std::to_string(result) + ")."); break;
    }
}

void SocialHandler::handleTurnInPetitionResults(network::Packet& packet) {
    uint32_t result = 0;
    if (!TurnInPetitionResultsParser::parse(packet, result)) return;
    switch (result) {
        case 0: owner_.addSystemChatMessage("Guild created successfully!"); break;
        case 1: owner_.addSystemChatMessage("Guild creation failed: already in a guild."); break;
        case 2: owner_.addSystemChatMessage("Guild creation failed: not enough signatures."); break;
        case 3: owner_.addSystemChatMessage("Guild creation failed: name already taken."); break;
        default: owner_.addSystemChatMessage("Guild creation failed (error " + std::to_string(result) + ")."); break;
    }
}

// ============================================================
// Server Info Handlers
// ============================================================

void SocialHandler::handleQueryTimeResponse(network::Packet& packet) {
    QueryTimeResponseData data;
    if (!QueryTimeResponseParser::parse(packet, data)) return;

    // The second word is how long until the daily quests reset -
    // GetNextDailyQuestsResetTime() minus now, in AzerothCore's own terms. It
    // was parsed and dropped, and GetQuestResetTime answered the time until
    // local midnight instead, with a comment saying this was a realm setting
    // nothing sends. It is sent, in the reply to a packet this client already
    // knew how to ask for.
    dailyResetOffset_ = data.timeOffset;
    dailyResetReceivedAt_ = time(nullptr);

    // Only /time says it out loud. The login-time ask is for the reset figure
    // and would otherwise greet every player with the clock.
    if (!announceServerTime_) return;
    announceServerTime_ = false;
    time_t serverTime = static_cast<time_t>(data.serverTime);
    const std::tm timeInfo = core::localTime(serverTime);
    char timeStr[64];
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeInfo);
    owner_.addSystemChatMessage("Server time: " + std::string(timeStr));
}

uint32_t SocialHandler::getSecondsUntilDailyReset() const {
    if (dailyResetOffset_ == 0 || dailyResetReceivedAt_ == 0) return 0;
    const auto elapsed = static_cast<int64_t>(time(nullptr) - dailyResetReceivedAt_);
    const int64_t left = static_cast<int64_t>(dailyResetOffset_) - elapsed;
    // Past the reset the answer is stale rather than negative; the next login
    // asks again. Zero reads as "not known", which is the honest answer.
    return left > 0 ? static_cast<uint32_t>(left) : 0;
}

void SocialHandler::handlePlayedTime(network::Packet& packet) {
    PlayedTimeData data;
    if (!PlayedTimeParser::parse(packet, data)) return;
    totalTimePlayed_ = data.totalTimePlayed;
    levelTimePlayed_ = data.levelTimePlayed;
    if (data.triggerMessage) {
        // TIME_PLAYED_MSG(totalTime, levelTime), in seconds, which is what
        // ChatFrame_SystemEventHandler dispatches to ChatFrame_DisplayTimePlayed
        // - the real client's own answer to /played, down to the localised
        // "days, hours, minutes, seconds" string. The server also sends played
        // time unasked at login, and triggerMessage is what tells the two
        // apart, so this stays inside it.
        owner_.fireAddonEvent("TIME_PLAYED_MSG",
                              {std::to_string(data.totalTimePlayed),
                               std::to_string(data.levelTimePlayed)});
        // ...and the hand-built lines below are what this client says when it
        // is drawing its own chat. With FrameXML drawing it they would be a
        // second, differently worded copy printed beside the real one - the
        // same shape as the gossip binder in quest_handler, where doing the
        // work here as well as letting the interface do it asked twice.
        if (ui::frameXmlOwns(ui::UiElement::Chat)) return;

        uint32_t totalDays = data.totalTimePlayed / 86400;
        uint32_t totalHours = (data.totalTimePlayed % 86400) / 3600;
        uint32_t totalMinutes = (data.totalTimePlayed % 3600) / 60;
        uint32_t levelDays = data.levelTimePlayed / 86400;
        uint32_t levelHours = (data.levelTimePlayed % 86400) / 3600;
        uint32_t levelMinutes = (data.levelTimePlayed % 3600) / 60;
        std::string totalMsg = "Total time played: ";
        if (totalDays > 0) totalMsg += std::to_string(totalDays) + " days, ";
        if (totalHours > 0 || totalDays > 0) totalMsg += std::to_string(totalHours) + " hours, ";
        totalMsg += std::to_string(totalMinutes) + " minutes";
        std::string levelMsg = "Time played this level: ";
        if (levelDays > 0) levelMsg += std::to_string(levelDays) + " days, ";
        if (levelHours > 0 || levelDays > 0) levelMsg += std::to_string(levelHours) + " hours, ";
        levelMsg += std::to_string(levelMinutes) + " minutes";
        owner_.addSystemChatMessage(totalMsg);
        owner_.addSystemChatMessage(levelMsg);
    }
}

void SocialHandler::handleWho(network::Packet& packet) {
    const bool hasGender = isActiveExpansion("wotlk");
    uint32_t displayCount = packet.readUInt32();
    uint32_t onlineCount = packet.readUInt32();
    whoResults_.clear();
    whoOnlineCount_ = onlineCount;
    if (displayCount == 0) { owner_.raiseUiError("No players found."); return; }
    for (uint32_t i = 0; i < displayCount; ++i) {
        if (packet.getReadPos() >= packet.getSize()) break;
        std::string playerName = packet.readString();
        std::string guildName = packet.readString();
        if (!packet.hasRemaining(12)) break;
        uint32_t level   = packet.readUInt32();
        uint32_t classId = packet.readUInt32();
        uint32_t raceId  = packet.readUInt32();
        if (hasGender && packet.hasRemaining(1)) packet.readUInt8();
        uint32_t zoneId = 0;
        if (packet.hasRemaining(4)) zoneId = packet.readUInt32();
        WhoEntry entry;
        entry.name = playerName; entry.guildName = guildName;
        entry.level = level; entry.classId = classId;
        entry.raceId = raceId; entry.zoneId = zoneId;
        whoResults_.push_back(std::move(entry));
    }
    // To the chat when nobody is showing a panel, which is what a stock client
    // does and what SetWhoToUI is for. FriendsFrame turns that on while its Who
    // tab is up and off again on the way out, so a /who typed into chat with
    // the panel closed printed nothing at all here: the rows were parsed,
    // stored, and kept.
    //
    // The formats are WoW's own, from globalstrings - and they are the
    // client's to print rather than the interface's, which is why no FrameXML
    // file formats them. The count line carries a |4 escape, which resolves on
    // the way to the screen.
    if (!whoToUI_) {
        for (const auto& e : whoResults_) {
            std::string line = "|Hplayer:" + e.name + "|h[" + e.name + "]|h: Level " +
                               std::to_string(e.level) + " " +
                               getRaceName(static_cast<Race>(e.raceId)) + std::string(" ") +
                               getClassName(static_cast<Class>(e.classId));
            if (!e.guildName.empty()) line += " <" + e.guildName + ">";
            const std::string zone = owner_.getWhoAreaName(e.zoneId);
            if (!zone.empty()) line += " - " + zone;
            owner_.addSystemChatMessage(line);
        }
        owner_.addSystemChatMessage(std::to_string(whoOnlineCount_) +
                                    " |4player:players; total");
    }

    // The who panel asked for this and redraws its rows on the answer coming
    // back. Every row was parsed and stored and only this client's own window
    // ever read them, so a /who through the interface filled a list nobody was
    // told about and the panel kept showing the previous search.
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("WHO_LIST_UPDATE", {});
}

// ============================================================
// Social (Friend/Ignore/Random Roll) Handlers
// ============================================================

void SocialHandler::handleFriendList(network::Packet& packet) {
    auto rem = [&]() { return packet.getRemainingSize(); };
    if (rem() < 1) return;
    uint8_t count = packet.readUInt8();
    owner_.contactsRef().erase(std::remove_if(owner_.contactsRef().begin(), owner_.contactsRef().end(),
        [](const ContactEntry& e){ return e.isFriend(); }), owner_.contactsRef().end());
    for (uint8_t i = 0; i < count && rem() >= 9; ++i) {
        uint64_t guid   = packet.readUInt64();
        uint8_t  status = packet.readUInt8();
        uint32_t area = 0, level = 0, classId = 0;
        if (status != 0 && rem() >= 12) {
            area    = packet.readUInt32();
            level   = packet.readUInt32();
            classId = packet.readUInt32();
        }
        owner_.friendGuidsRef().insert(guid);
        auto nit = owner_.getPlayerNameCache().find(guid);
        std::string name;
        if (nit != owner_.getPlayerNameCache().end()) {
            name = nit->second;
            owner_.friendsCacheRef()[name] = guid;
        } else {
            owner_.queryPlayerName(guid);
        }
        ContactEntry entry;
        entry.guid = guid; entry.name = name; entry.flags = 0x1;
        entry.status = status; entry.areaId = area; entry.level = level; entry.classId = classId;
        owner_.contactsRef().push_back(std::move(entry));
    }
    // The full list arrives in reply to this client asking for it, so it is the
    // "show" case as well as the "changed" case. SHOW additionally re-selects
    // the tab; UPDATE only refreshes the rows. A single friend going online
    // sends the status packet instead, and fires UPDATE alone.
    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("FRIENDLIST_UPDATE", {});
        owner_.addonEventCallbackRef()("FRIENDLIST_SHOW", {});
    }
}

void SocialHandler::handleContactList(network::Packet& packet) {
    auto rem = [&]() { return packet.getRemainingSize(); };
    if (rem() < 8) { packet.skipAll(); return; }
    owner_.lastContactListMaskRef()  = packet.readUInt32();
    owner_.lastContactListCountRef() = packet.readUInt32();
    owner_.contactsRef().clear();
    for (uint32_t i = 0; i < owner_.lastContactListCountRef() && rem() >= 8; ++i) {
        uint64_t guid  = packet.readUInt64();
        if (rem() < 4) break;
        uint32_t flags = packet.readUInt32();
        std::string note = packet.readString();
        uint8_t status = 0; uint32_t areaId = 0, level = 0, classId = 0;
        if (flags & 0x1) {
            if (rem() < 1) break;
            status = packet.readUInt8();
            if (status != 0 && rem() >= 12) {
                areaId = packet.readUInt32(); level = packet.readUInt32(); classId = packet.readUInt32();
            }
            owner_.friendGuidsRef().insert(guid);
            auto nit = owner_.getPlayerNameCache().find(guid);
            if (nit != owner_.getPlayerNameCache().end()) owner_.friendsCacheRef()[nit->second] = guid;
            else owner_.queryPlayerName(guid);
        }
        ContactEntry entry;
        entry.guid = guid; entry.flags = flags; entry.note = std::move(note);
        entry.status = status; entry.areaId = areaId; entry.level = level; entry.classId = classId;
        auto nit = owner_.getPlayerNameCache().find(guid);
        if (nit != owner_.getPlayerNameCache().end()) entry.name = nit->second;
        owner_.contactsRef().push_back(std::move(entry));
    }
    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("FRIENDLIST_UPDATE", {});
        owner_.addonEventCallbackRef()("FRIENDLIST_SHOW", {});
        if (owner_.lastContactListMaskRef() & 0x2) owner_.addonEventCallbackRef()("IGNORELIST_UPDATE", {});
    }
}

void SocialHandler::handleFriendStatus(network::Packet& packet) {
    FriendStatusData data;
    if (!FriendStatusParser::parse(packet, data)) return;

    // Single lookup - reuse iterator for name resolution and update/erase below
    auto cit = std::find_if(owner_.contactsRef().begin(), owner_.contactsRef().end(),
        [&](const ContactEntry& e){ return e.guid == data.guid; });

    // Look up player name: contacts_ (populated by SMSG_FRIEND_LIST) > playerNameCache
    std::string playerName;
    if (cit != owner_.contactsRef().end() && !cit->name.empty()) {
        playerName = cit->name;
    } else {
        auto it = owner_.getPlayerNameCache().find(data.guid);
        if (it != owner_.getPlayerNameCache().end()) playerName = it->second;
    }

    // Only update friendsCache when we have a resolved name - inserting an empty
    // key creates a phantom entry that masks the real one when the name arrives.
    if (!playerName.empty()) {
        if (data.status == 1 || data.status == 2) owner_.friendsCacheRef()[playerName] = data.guid;
        else if (data.status == 0) owner_.friendsCacheRef().erase(playerName);
    }

    // AzerothCore's FriendsResult, read off SocialMgr.h rather than guessed.
    // The numbering here was five places out: zero is a database error and not
    // a removal, one is a full list and not an addition, and the three results
    // that matter most - removed, added-online, added-offline - are 5, 6 and 7
    // where this had "already in your list", "list is full" and "is ignoring
    // you". So adding a friend announced that the list was full, and a
    // database error deleted them from it.
    enum : uint8_t {
        kDbError = 0, kListFull = 1, kOnline = 2, kOffline = 3, kNotFound = 4,
        kRemoved = 5, kAddedOnline = 6, kAddedOffline = 7, kAlready = 8,
        kSelf = 9, kEnemy = 10,
    };
    if (data.status == kRemoved) {
        if (cit != owner_.contactsRef().end())
            owner_.contactsRef().erase(cit);
    } else if (data.status == kOnline || data.status == kOffline ||
               data.status == kAddedOnline || data.status == kAddedOffline) {
        const bool nowOnline =
            (data.status == kOnline || data.status == kAddedOnline);
        // chatFlag is the FriendStatus bitmask the packet carries, and it is
        // what the friends list reads for away and busy. Offline results carry
        // no body, so the flags are cleared rather than left as they were.
        const uint8_t flags = nowOnline ? data.chatFlag : 0;
        if (cit != owner_.contactsRef().end()) {
            if (!playerName.empty() && playerName != "Unknown") cit->name = playerName;
            cit->status = flags;
            if (nowOnline) {
                cit->areaId = data.areaId;
                cit->level = data.level;
                cit->classId = data.classId;
            }
            if (!data.note.empty()) cit->note = data.note;
        } else {
            ContactEntry entry;
            entry.guid = data.guid; entry.name = playerName; entry.flags = 0x1;
            entry.status = flags;
            entry.note = data.note;
            entry.areaId = data.areaId; entry.level = data.level;
            entry.classId = data.classId;
            owner_.contactsRef().push_back(std::move(entry));
        }
    }
    switch (data.status) {
        case kDbError:      owner_.addSystemChatMessage("Your friends list could not be reached."); break;
        case kListFull:     owner_.addSystemChatMessage("Your friends list is full."); break;
        case kOnline:       owner_.addSystemChatMessage(playerName + " is now online."); break;
        case kOffline:      owner_.addSystemChatMessage(playerName + " is now offline."); break;
        case kNotFound:     owner_.addSystemChatMessage("Player not found."); break;
        case kRemoved:      owner_.addSystemChatMessage(playerName + " has been removed from your friends list."); break;
        case kAddedOnline:
        case kAddedOffline: owner_.addSystemChatMessage(playerName + " has been added to your friends list."); break;
        case kAlready:      owner_.addSystemChatMessage(playerName + " is already in your friends list."); break;
        case kSelf:         owner_.addSystemChatMessage("You cannot add yourself as a friend."); break;
        case kEnemy:        owner_.addSystemChatMessage(playerName + " belongs to the other faction."); break;
        default: break;
    }
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("FRIENDLIST_UPDATE", {});
}

void SocialHandler::handleRandomRoll(network::Packet& packet) {
    RandomRollData data;
    if (!RandomRollParser::parse(packet, data)) return;
    std::string rollerName = (data.rollerGuid == owner_.getPlayerGuid()) ? "You" : "Someone";
    if (data.rollerGuid != owner_.getPlayerGuid()) {
        auto it = owner_.getPlayerNameCache().find(data.rollerGuid);
        if (it != owner_.getPlayerNameCache().end()) rollerName = it->second;
    }
    std::string msg = rollerName + ((data.rollerGuid == owner_.getPlayerGuid()) ? " roll " : " rolls ");
    msg += std::to_string(data.result) + " (" + std::to_string(data.minRoll) + "-" + std::to_string(data.maxRoll) + ")";
    owner_.addSystemChatMessage(msg);
}

// ============================================================
// Logout Handlers
// ============================================================

void SocialHandler::handleLogoutResponse(network::Packet& packet) {
    LogoutResponseData data;
    if (!LogoutResponseParser::parse(packet, data)) return;
    if (data.result == 0) {
        if (data.instant) { owner_.addSystemChatMessage("Logging out..."); logoutCountdown_ = 0.0f; }
        else { owner_.addSystemChatMessage("Logging out in 20 seconds..."); logoutCountdown_ = 20.0f; }
        if (owner_.addonEventCallbackRef()) {
            owner_.addonEventCallbackRef()("PLAYER_LOGOUT", {});
            // The timer the server just imposed, announced so the interface can
            // show it. UIParent answers PLAYER_CAMPING with the CAMP popup and
            // PLAYER_QUITING with QUIT, both of which count down and offer to
            // cancel - and it already answers LOGOUT_CANCEL by hiding them, so
            // half of this conversation was wired and the half that starts it
            // was not. Nothing could show the twenty seconds but a line of chat.
            //
            // Which of the two is the difference between /logout and /quit,
            // which this handler already keeps in exitAfterLogout_.
            if (!data.instant) {
                owner_.addonEventCallbackRef()(
                    exitAfterLogout_ ? "PLAYER_QUITING" : "PLAYER_CAMPING", {});
            }
        }
    } else {
        // The server said no, and what that means depends on what was asked.
        //
        // For /logout it is the whole answer: the player stays in the world and
        // is told why. For Exit Game it is not. That button asks the program to
        // close, and a refusal governs whether the character leaves the world
        // tidily rather than whether this process keeps running - so printing
        // the error and staying put left a quit button that says no, which is
        // the one thing a quit button must never do.
        const bool exiting = exitAfterLogout_;
        loggingOut_ = false; exitAfterLogout_ = false; logoutCountdown_ = 0.0f;
        if (exiting) {
            LOG_WARNING("Logout refused, but Exit Game was asked for - closing anyway");
            if (owner_.logoutCompleteCallbackRef()) owner_.logoutCompleteCallbackRef()(true);
        } else {
            owner_.raiseUiError("Cannot logout right now.");
        }
    }
}

void SocialHandler::handleLogoutComplete(network::Packet& /*packet*/) {
    // The countdown finishing is not the end of it: the server says when the
    // character is actually out of the world, and only then can the client leave -
    // to character select, or out of the game entirely for /quit and /exit.
    const bool exiting = exitAfterLogout_;
    owner_.addSystemChatMessage("Logout complete.");
    loggingOut_ = false; exitAfterLogout_ = false; logoutCountdown_ = 0.0f;
    LOG_WARNING("Logout complete (exiting=", exiting ? "yes" : "no", ")");  // temporary: visible at default log level
    if (owner_.logoutCompleteCallbackRef()) owner_.logoutCompleteCallbackRef()(exiting);
}

// ============================================================
// Battleground Handlers
// ============================================================

void SocialHandler::handleBattlefieldStatus(network::Packet& packet) {
    BattlefieldStatusData status;
    if (!BattlefieldStatusPacket::parse(packet, status, isClassicLikeExpansion(),
                                        !isPreWotlk())) {
        return;
    }
    const uint32_t queueSlot = status.queueSlot;
    const uint8_t arenaType = status.arenaType;
    const uint32_t bgTypeId = status.bgTypeId;
    const uint32_t statusId = status.statusId;

    static const std::pair<uint32_t, const char*> kBgNames[] = {
        {1,"Alterac Valley"},{2,"Warsong Gulch"},{3,"Arathi Basin"},
        {4,"Nagrand Arena"},{5,"Blade's Edge Arena"},{6,"All Arenas"},
        {7,"Eye of the Storm"},{8,"Ruins of Lordaeron"},{9,"Strand of the Ancients"},
        {10,"Dalaran Sewers"},{11,"Ring of Valor"},{30,"Isle of Conquest"},{32,"Random Battleground"},
    };
    std::string bgName = "Battleground";
    for (const auto& kv : kBgNames) { if (kv.first == bgTypeId) { bgName = kv.second; break; } }
    if (bgName == "Battleground") bgName = "Battleground #" + std::to_string(bgTypeId);
    if (arenaType > 0) {
        bgName = std::to_string(arenaType) + "v" + std::to_string(arenaType) + " Arena";
        for (const auto& kv : kBgNames) { if (kv.first == bgTypeId) { bgName += " (" + std::string(kv.second) + ")"; break; } }
    }

    // An invitation with no timeout in it still has to count down from
    // something, and eighty seconds is what the server gives one.
    const uint32_t inviteTimeout =
        status.inviteTimeoutMs ? status.inviteTimeoutMs / 1000 : 80;
    const uint32_t avgWaitSec = status.avgWaitMs / 1000;
    const uint32_t timeInQueueSec = status.timeInQueueMs / 1000;

    // Server pushes SMSG_BATTLEFIELD_STATUS periodically (~30s ticks while queued)
    // and also for each queue slot at zone change / login. Only emit a chat line
    // when the slot's statusId actually transitions; otherwise the same "Queued
    // for X." or "Entered X." spams chat every tick.
    bool statusChanged = false;
    if (queueSlot < bgQueues_.size()) {
        uint32_t prevStatus = bgQueues_[queueSlot].statusId;
        bool wasInvite = (prevStatus == 2);
        statusChanged = (prevStatus != statusId) || (bgQueues_[queueSlot].bgTypeId != bgTypeId);
        bgQueues_[queueSlot].queueSlot = queueSlot;
        bgQueues_[queueSlot].bgTypeId = bgTypeId;
        bgQueues_[queueSlot].arenaType = arenaType;
        bgQueues_[queueSlot].statusId = statusId;
        bgQueues_[queueSlot].bgName = bgName;
        bgQueues_[queueSlot].minLevel = status.minLevel;
        bgQueues_[queueSlot].maxLevel = status.maxLevel;
        bgQueues_[queueSlot].instanceId = status.instanceId;
        bgQueues_[queueSlot].isRated = status.isRated;
        if (statusId == 1) { bgQueues_[queueSlot].avgWaitTimeSec = avgWaitSec; bgQueues_[queueSlot].timeInQueueSec = timeInQueueSec; }
        if (statusId == 2 && !wasInvite) { bgQueues_[queueSlot].inviteTimeout = inviteTimeout; bgQueues_[queueSlot].inviteReceivedTime = std::chrono::steady_clock::now(); }
    } else {
        statusChanged = true;
    }

    if (statusChanged) {
        switch (statusId) {
            case 1: owner_.addSystemChatMessage("Queued for " + bgName + "."); break;
            case 2: owner_.addSystemChatMessage(bgName + " is ready!"); break;
            case 3: owner_.addSystemChatMessage("Entered " + bgName + "."); break;
            default: break;
        }
    }
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("UPDATE_BATTLEFIELD_STATUS", {std::to_string(statusId)});
}

void SocialHandler::handleBattlefieldList(network::Packet& packet) {
    if (!packet.hasRemaining(5)) return;
    AvailableBgInfo info;
    info.bgTypeId = packet.readUInt32();
    info.isRegistered = packet.readUInt8() != 0;
    const bool isWotlk = isActiveExpansion("wotlk");
    const bool isTbc = isActiveExpansion("tbc");
    if (isTbc || isWotlk) { if (!packet.hasRemaining(1)) return; info.isHoliday = packet.readUInt8() != 0; }
    if (isWotlk) { if (!packet.hasRemaining(8)) return; info.minLevel = packet.readUInt32(); info.maxLevel = packet.readUInt32(); }
    if (!packet.hasRemaining(4)) return;
    uint32_t count = std::min(packet.readUInt32(), 256u);
    info.instanceIds.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!packet.hasRemaining(4)) break;
        info.instanceIds.push_back(packet.readUInt32());
    }
    bool updated = false;
    for (auto& existing : availableBgs_) { if (existing.bgTypeId == info.bgTypeId) { existing = std::move(info); updated = true; break; } }
    if (!updated) availableBgs_.push_back(std::move(info));
}

bool SocialHandler::hasPendingBgInvite() const {
    for (const auto& slot : bgQueues_) { if (slot.statusId == 2) return true; }
    return false;
}

void SocialHandler::acceptBattlefield(uint32_t queueSlot) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    const BgQueueSlot* slot = nullptr;
    if (queueSlot == 0xFFFFFFFF) { for (const auto& s : bgQueues_) { if (s.statusId == 2) { slot = &s; break; } } }
    else if (queueSlot < bgQueues_.size() && bgQueues_[queueSlot].statusId == 2) slot = &bgQueues_[queueSlot];
    if (!slot) { owner_.addSystemChatMessage("No battleground invitation pending."); return; }
    network::Packet pkt(wireOpcode(Opcode::CMSG_BATTLEFIELD_PORT));
    pkt.writeUInt8(slot->arenaType); pkt.writeUInt8(0x00); pkt.writeUInt32(slot->bgTypeId);
    pkt.writeUInt16(0x0000); pkt.writeUInt8(1);
    owner_.getSocket()->send(pkt);
    uint32_t clearSlot = slot->queueSlot;
    if (clearSlot < bgQueues_.size()) bgQueues_[clearSlot].statusId = 3;
    owner_.addSystemChatMessage("Accepting battleground invitation...");
}

// Leave the battleground the player is standing in, which is a different thing
// from declining an invitation to one - that is CMSG_BATTLEFIELD_PORT with a
// refusal, this is the player walking out of a match already joined.
// CMSG_BATTLEMASTER_JOIN: which battlemaster is being spoken to, which
// battleground, which instance of it, and whether the whole group is coming.
// An instance of zero means "first available", which is what the top row of
// the battlefield list is.
void SocialHandler::joinBattlefield(uint64_t battlemasterGuid, uint32_t bgTypeId,
                                    uint32_t instanceId, bool asGroup) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    network::Packet packet(wireOpcode(Opcode::CMSG_BATTLEMASTER_JOIN));
    packet.writeUInt64(battlemasterGuid);
    packet.writeUInt32(bgTypeId);
    packet.writeUInt32(instanceId);
    packet.writeUInt8(asGroup ? 1 : 0);
    owner_.getSocket()->send(packet);
}

// CMSG_BATTLEFIELD_LIST: ask which instances of one battleground are running.
// The reply is SMSG_BATTLEFIELD_LIST, which this client already handles - it
// just had no way to ask, because nothing reached the frame that asks.
void SocialHandler::requestBattlefieldList(uint32_t bgTypeId) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    network::Packet packet(wireOpcode(Opcode::CMSG_BATTLEFIELD_LIST));
    packet.writeUInt32(bgTypeId);
    // WotLK added two bytes: where the request came from, and whether the
    // player wants experience from it. Classic and TBC read the type id alone,
    // and trailing bytes there would be read as the next packet.
    if (!isPreWotlk()) {
        packet.writeUInt8(0);  // fromWhere: 0 = the battlemaster list frame
        packet.writeUInt8(1);  // canGainXP
    }
    owner_.getSocket()->send(packet);
}

// CMSG_REPORT_PVP_AFK: this player is not participating. The reply,
// SMSG_REPORT_PVP_AFK_RESULT, was already handled - the request was the half
// that had no way to be sent.
void SocialHandler::reportPvpAfk(uint64_t playerGuid) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (playerGuid == 0) return;
    network::Packet pkt(wireOpcode(Opcode::CMSG_REPORT_PVP_AFK));
    pkt.writeUInt64(playerGuid);
    owner_.getSocket()->send(pkt);
}

void SocialHandler::leaveBattlefield() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    network::Packet pkt(wireOpcode(Opcode::CMSG_LEAVE_BATTLEFIELD));
    owner_.getSocket()->send(pkt);
}

void SocialHandler::declineBattlefield(uint32_t queueSlot) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    const BgQueueSlot* slot = nullptr;
    if (queueSlot == 0xFFFFFFFF) { for (const auto& s : bgQueues_) { if (s.statusId == 2) { slot = &s; break; } } }
    else if (queueSlot < bgQueues_.size() && bgQueues_[queueSlot].statusId == 2) slot = &bgQueues_[queueSlot];
    if (!slot) { owner_.addSystemChatMessage("No battleground invitation pending."); return; }
    network::Packet pkt(wireOpcode(Opcode::CMSG_BATTLEFIELD_PORT));
    pkt.writeUInt8(slot->arenaType); pkt.writeUInt8(0x00); pkt.writeUInt32(slot->bgTypeId);
    pkt.writeUInt16(0x0000); pkt.writeUInt8(0);
    owner_.getSocket()->send(pkt);
    uint32_t clearSlot = slot->queueSlot;
    if (clearSlot < bgQueues_.size()) bgQueues_[clearSlot] = BgQueueSlot{};
    owner_.addSystemChatMessage("Battleground invitation declined.");
}

void SocialHandler::requestPvpLog() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    network::Packet pkt(wireOpcode(Opcode::MSG_PVP_LOG_DATA));
    owner_.getSocket()->send(pkt);
}

// ============================================================
// Instance Handlers
// ============================================================

void SocialHandler::handleRaidInstanceInfo(network::Packet& packet) {
    const bool isTbc = isActiveExpansion("tbc");
    const bool isClassic = isClassicLikeExpansion();
    const bool useTbcFormat = isTbc || isClassic;
    if (!packet.hasRemaining(4)) return;
    uint32_t count = packet.readUInt32();
    instanceLockouts_.clear();
    instanceLockouts_.reserve(count);
    const size_t kEntrySize = useTbcFormat ? 13 : 18;
    for (uint32_t i = 0; i < count; ++i) {
        if (packet.getRemainingSize() < kEntrySize) break;
        InstanceLockout lo;
        lo.mapId = packet.readUInt32(); lo.difficulty = packet.readUInt32();
        if (useTbcFormat) { lo.resetTime = packet.readUInt32(); lo.locked = packet.readUInt8() != 0; lo.extended = false; }
        else { lo.resetTime = packet.readUInt64(); lo.locked = packet.readUInt8() != 0; lo.extended = packet.readUInt8() != 0; }
        instanceLockouts_.push_back(lo);
    }
}

void SocialHandler::handleInstanceDifficulty(network::Packet& packet) {
    auto rem = [&]() { return packet.getRemainingSize(); };
    if (rem() < 4) return;
    uint32_t prevDifficulty = instanceDifficulty_;
    instanceDifficulty_ = packet.readUInt32();
    if (rem() >= 4) {
        uint32_t secondField = packet.readUInt32();
        if (rem() >= 4) instanceIsHeroic_ = (instanceDifficulty_ == 1);
        else instanceIsHeroic_ = (secondField != 0);
    } else {
        instanceIsHeroic_ = (instanceDifficulty_ == 1);
    }
    inInstance_ = true;
    if (instanceDifficulty_ != prevDifficulty) {
        static const char* kDiffLabels[] = {"Normal", "Heroic", "25-Man Normal", "25-Man Heroic"};
        const char* diffLabel = (instanceDifficulty_ < 4) ? kDiffLabels[instanceDifficulty_] : nullptr;
        if (diffLabel) owner_.addSystemChatMessage(std::string("Dungeon difficulty set to ") + diffLabel + ".");
        // Already noticed and already said - in chat, to the player. The
        // minimap hangs a difficulty banner off its own corner and rereads
        // GetInstanceInfo on this, which answers from the value just stored,
        // so the banner was the one thing not told the difficulty had changed.
        if (owner_.addonEventCallbackRef()) {
            owner_.addonEventCallbackRef()("PLAYER_DIFFICULTY_CHANGED", {});
            owner_.addonEventCallbackRef()("UPDATE_INSTANCE_INFO", {});
        }
    }
}

// ============================================================
// LFG Handlers
// ============================================================

void SocialHandler::handleLfgJoinResult(network::Packet& packet) {
    // Two uint32s, not two bytes. Reading a byte each left the result right by
    // accident - every value in that enum fits in one - and the state wrong
    // always: it came from the second byte of the result, which is zero. So a
    // queue joined successfully set the state to None, and the dungeon finder
    // never knew it was in a queue.
    if (!packet.hasRemaining(8)) return;
    const uint32_t result = packet.readUInt32();
    const uint32_t state  = packet.readUInt32();
    if (result == 0) {
        lfgState_ = static_cast<LfgState>(state);
        std::string dName = owner_.getLfgDungeonName(lfgDungeonId_);
        if (!dName.empty()) owner_.addSystemChatMessage("Dungeon Finder: Joined the queue for " + dName + ".");
        else owner_.addSystemChatMessage("Dungeon Finder: Joined the queue.");
    } else {
        const char* msg = lfgJoinResultString(result);
        std::string errMsg = std::string("Dungeon Finder: ") + (msg ? msg : "Join failed.");
        owner_.addUIError(errMsg);
        owner_.addSystemChatMessage(errMsg);
    }
    // The queue state moved, either into a queue or back out of the attempt,
    // and LFG_UPDATE is how the panel is told to re-read it - the same event
    // handleLfgUpdate fires for the same reason two handlers down. Without it
    // the state was right in this client and stale in the interface until some
    // other LFG packet happened along.
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("LFG_UPDATE", {});
}

// SMSG_LFG_PLAYER_INFO - what the random dungeons pay out, and what this
// character is not allowed to queue for.
//
// The reward block has to be walked even though none of it is kept, because
// the locks are behind it and its length depends on how many items each
// random dungeon rewards. Skipping to the end was not an option; the whole
// packet was being skipped instead.
void SocialHandler::handleLfgPlayerInfo(network::Packet& packet) {
    if (!packet.hasRemaining(1)) { packet.skipAll(); return; }
    const uint8_t randomCount = packet.readUInt8();
    lfgRewards_.clear();
    for (uint8_t i = 0; i < randomCount; ++i) {
        if (!packet.hasRemaining(4 + 1 + 4 * 4 + 1)) { packet.skipAll(); return; }
        LfgReward r;
        r.dungeonId  = packet.readUInt32() & 0x00FFFFFFu;
        r.done       = packet.readUInt8() != 0;
        r.money      = packet.readUInt32();
        r.experience = packet.readUInt32();
        packet.readUInt32();          // the variable parts, which the server
        packet.readUInt32();          // sends as zero
        const uint8_t items = packet.readUInt8();
        for (uint8_t j = 0; j < items; ++j) {
            if (!packet.hasRemaining(12)) { packet.skipAll(); return; }
            LfgRewardItem item;
            item.itemId        = packet.readUInt32();
            item.displayInfoId = packet.readUInt32();
            item.count         = packet.readUInt32();
            r.items.push_back(item);
        }
        lfgRewards_.push_back(std::move(r));
    }

    if (!packet.hasRemaining(4)) { packet.skipAll(); return; }
    const uint32_t lockCount = packet.readUInt32();
    lfgLocks_.clear();
    for (uint32_t i = 0; i < lockCount && packet.hasRemaining(8); ++i) {
        // The entry packs the dungeon and its type; the list is keyed by the
        // dungeon alone, which is what the interface asks with.
        const uint32_t entry = packet.readUInt32();
        const uint32_t status = packet.readUInt32();
        lfgLocks_[entry & 0x00FFFFFFu] = status;
    }
    packet.skipAll();

    LOG_INFO("SMSG_LFG_PLAYER_INFO: ", randomCount, " random dungeons, ",
             lfgLocks_.size(), " locked");
    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("LFG_LOCK_INFO_RECEIVED", {});
        owner_.addonEventCallbackRef()("LFG_UPDATE_RANDOM_INFO", {});
    }
}

void SocialHandler::handleLfgQueueStatus(network::Packet& packet) {
    // Thirty-one bytes, not thirty-three: dungeon, five wait times, three
    // one-byte "needed" counts, and the time queued. The old length check was
    // longer than the packet the server sends, so this returned before reading
    // anything and the queue status never updated at all - no wait times, no
    // time in queue.
    if (!packet.hasRemaining(31)) return;
    lfgDungeonId_ = packet.readUInt32();
    const int32_t avgWait  = static_cast<int32_t>(packet.readUInt32());
    const int32_t waitTime = static_cast<int32_t>(packet.readUInt32());
    lfgWaitTank_   = static_cast<int32_t>(packet.readUInt32());
    lfgWaitHealer_ = static_cast<int32_t>(packet.readUInt32());
    lfgWaitDps_    = static_cast<int32_t>(packet.readUInt32());
    // Three of these, one per role, and only one was being read - so the time
    // in queue came from two bytes of the needed-counts and two of its own.
    lfgNeedTank_   = packet.readUInt8();
    lfgNeedHealer_ = packet.readUInt8();
    lfgNeedDps_    = packet.readUInt8();
    lfgTimeInQueueMs_ = packet.readUInt32();
    lfgAvgWaitSec_ = (waitTime >= 0) ? (waitTime / 1000) : (avgWait / 1000);
    lfgState_ = LfgState::Queued;
    // Read and then kept to itself. The wait time is what the queue window
    // counts up, and LFDSearchStatus_Update is reached from this event and no
    // other - so the numbers above changed every few seconds and the display
    // built on them never moved.
    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("LFG_QUEUE_STATUS_UPDATE", {});
        // The queue state is set two lines above, and LFG_QUEUE_STATUS_UPDATE
        // is not the event that re-reads it: the minimap's dungeon eye is shown
        // by MiniMapLFG_UpdateIsShown, which asks GetLFGMode - which asks
        // GetLFGInfoServer - and is reached from LFG_UPDATE and nowhere else.
        //
        // SMSG_LFG_UPDATE_PLAYER usually arrives alongside this and does fire
        // it, which is why the eye appears at all. The case that has nothing
        // else behind it is a login or a reconnect already in the queue, where
        // this packet is what says so: the state was right and the eye was
        // never shown.
        owner_.addonEventCallbackRef()("LFG_UPDATE", {});
    }
}

void SocialHandler::handleLfgProposalUpdate(network::Packet& packet) {
    // The header is fifteen bytes, and the state is ONE of them.
    //
    // This read three uint32s where the server writes dungeon(4), state(1),
    // id(4), encounters(4), silent(1), size(1) - so the proposal id came out of
    // the state byte plus three bytes of the id, and the state itself out of
    // the id's last byte plus three of the encounter count. Every branch below
    // was taken on a number assembled from the wrong bytes, which is why the
    // pop never behaved: it was not that the state was unhandled, it was never
    // read.
    if (!packet.hasRemaining(15)) return;
    const uint32_t dungeonEntry = packet.readUInt32();
    const uint32_t proposalState = packet.readUInt8();
    const uint32_t proposalId = packet.readUInt32();
    const uint32_t encountersDone = packet.readUInt32();
    // "silent" means do NOT raise the window - the player is already in the
    // group being proposed to, so there is nothing to accept.
    const bool silent = packet.readUInt8() != 0;
    const uint8_t groupSize = packet.hasRemaining(1) ? packet.readUInt8() : 0;
    (void)encountersDone;

    // The entry packs the dungeon and its type, the same way
    // LFGDungeonData::Entry builds it.
    const uint32_t dungeonId = dungeonEntry & 0x00FFFFFFu;

    lfgProposalMembers_.clear();
    for (uint8_t i = 0; i < groupSize && packet.hasRemaining(9); ++i) {
        LfgProposalMember m;
        m.role      = packet.readUInt32();
        m.isSelf    = packet.readUInt8() != 0;
        m.inDungeon = packet.readUInt8() != 0;
        m.sameGroup = packet.readUInt8() != 0;
        m.answered  = packet.readUInt8() != 0;
        m.accepted  = packet.readUInt8() != 0;
        lfgProposalMembers_.push_back(m);
    }

    lfgDungeonId_ = dungeonId; lfgProposalId_ = proposalId;
    switch (proposalState) {
        case 0: lfgState_ = LfgState::Queued; lfgProposalId_ = 0;
            owner_.addUIError("Dungeon Finder: Group proposal failed."); owner_.addSystemChatMessage("Dungeon Finder: Group proposal failed."); break;
        case 1: { lfgState_ = LfgState::InDungeon; lfgProposalId_ = 0;
            std::string dName = owner_.getLfgDungeonName(dungeonId);
            owner_.addSystemChatMessage(dName.empty() ? "Dungeon Finder: Group found! Entering dungeon..." : "Dungeon Finder: Group found for " + dName + "! Entering dungeon..."); break; }
        case 2: { lfgState_ = LfgState::Proposal;
            std::string dName = owner_.getLfgDungeonName(dungeonId);
            owner_.addSystemChatMessage(dName.empty() ? "Dungeon Finder: A group has been found. Accept or decline." : "Dungeon Finder: A group has been found for " + dName + ". Accept or decline."); break; }
        default: break;
    }
    // The proposal dialog is built from GetLFGProposal, which already answers
    // from this - it just had no way to know a proposal had arrived.
    if (!owner_.addonEventCallbackRef()) return;
    auto fire = owner_.addonEventCallbackRef();
    fire("LFG_PROPOSAL_UPDATE", {});

    // UPDATE alone only refreshes a dialog already on screen.
    // LFDFrame_OnEvent opens it on SHOW - that branch is the one calling
    // StaticPopupSpecial_Show - and closes it on SUCCEEDED or FAILED. None of
    // the three was fired, so a pop arrived as a line of chat and nothing else,
    // with no way to accept it.
    //
    // SHOW once per proposal rather than on every update: the server resends
    // this as each member answers, and showing again would reset the dialog's
    // countdown and its per-member ticks every time.
    switch (proposalState) {
        case 0:
            shownProposalId_ = 0;
            fire("LFG_PROPOSAL_FAILED", {});
            break;
        case 1:
            shownProposalId_ = 0;
            fire("LFG_PROPOSAL_SUCCEEDED", {});
            break;
        case 2:
            // Not when the server says silent: that means the player is
            // already in the group being proposed to and has nothing to answer.
            if (!silent && shownProposalId_ != proposalId) {
                shownProposalId_ = proposalId;
                fire("LFG_PROPOSAL_SHOW", {});
            }
            break;
        default:
            break;
    }
}

void SocialHandler::handleLfgRoleCheckUpdate(network::Packet& packet) {
    // state is the uint32; the uint8 after it says whether the check is only
    // starting. This read the state and threw it away, then took the starting
    // flag for the state - so the branches below ran on 0 or 1 and never saw
    // the values they test for.
    if (!packet.hasRemaining(6)) return;
    const uint32_t roleCheckState = packet.readUInt32();
    packet.readUInt8();   // initiating

    // The dungeons being checked for, and how many are in the group. Both were
    // stopped at: the count was read and the list behind it left on the wire.
    //
    // That is not merely a blank. GetLFGRoleUpdate answered nil for the slot
    // count, and the popup's own tooltip opens with `if ( slots <= 1 )` -
    // comparing nil to a number, which raises. Hovering the role-check popup
    // took the file down.
    //
    // Each dungeon arrives as its LFGDungeons entry, which packs the type into
    // the top byte the same way the queue list does above.
    lfgRoleCheckDungeons_.clear();
    const uint8_t dungeonCount = packet.readUInt8();
    for (uint8_t i = 0; i < dungeonCount && packet.hasRemaining(4); ++i) {
        const uint32_t entry = packet.readUInt32();
        lfgRoleCheckDungeons_.push_back({entry & 0x00FFFFFFu,
                                         static_cast<uint8_t>((entry >> 24) & 0xFFu)});
    }
    lfgRoleCheckMembers_ = packet.hasRemaining(1) ? packet.readUInt8() : 0;

    if (roleCheckState == 1) lfgState_ = LfgState::Queued;
    else if (roleCheckState == 3) { lfgState_ = LfgState::None; owner_.addUIError("Dungeon Finder: Role check failed - missing required role."); owner_.addSystemChatMessage("Dungeon Finder: Role check failed - missing required role."); }
    else if (roleCheckState == 2) { lfgState_ = LfgState::RoleCheck; owner_.addSystemChatMessage("Dungeon Finder: Performing role check..."); }
    // Likewise GetLFGRoleUpdate, which reports the check in progress.
    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("LFG_ROLE_CHECK_UPDATE", {});
        // The role-check popup shows on one event and hides on another, and
        // neither was ever fired - so the interface's own popup could not
        // appear at all, whichever side is drawing it.
        //
        // State 2 is LFG_ROLECHECK_INITIALITING, where the check begins.
        // Everything else is an ending: finished, missing a role, the wrong
        // roles between them, aborted, or someone picking none.
        constexpr uint32_t kRoleCheckBeginning = 2;
        owner_.addonEventCallbackRef()(
            roleCheckState == kRoleCheckBeginning ? "LFG_ROLE_CHECK_SHOW"
                                                  : "LFG_ROLE_CHECK_HIDE", {});
    }
}

void SocialHandler::handleLfgUpdatePlayer(network::Packet& packet) {
    if (!packet.hasRemaining(1)) return;
    uint8_t updateType = packet.readUInt8();
    bool hasExtra = (updateType != 0 && updateType != 1 && updateType != 15 && updateType != 17 && updateType != 18);
    if (!hasExtra || !packet.hasRemaining(3)) {
        switch (updateType) {
            case 8:  lfgState_ = LfgState::None; owner_.addSystemChatMessage("Dungeon Finder: Removed from queue."); break;
            case 9:  lfgState_ = LfgState::Queued; owner_.addSystemChatMessage("Dungeon Finder: Proposal failed - re-queuing."); break;
            case 10: lfgState_ = LfgState::Queued; owner_.addSystemChatMessage("Dungeon Finder: A member declined the proposal."); break;
            case 15: lfgState_ = LfgState::None; owner_.addSystemChatMessage("Dungeon Finder: Left the queue."); break;
            case 18: lfgState_ = LfgState::None; owner_.addSystemChatMessage("Dungeon Finder: Your group disbanded."); break;
            default: break;
        }
        return;
    }
    packet.readUInt8(); packet.readUInt8(); packet.readUInt8();
    if (packet.hasRemaining(1)) {
        uint8_t count = packet.readUInt8();
        for (uint8_t i = 0; i < count && packet.getRemainingSize() >= 4; ++i) {
            uint32_t dungeonEntry = packet.readUInt32();
            if (i == 0) lfgDungeonId_ = dungeonEntry;
        }
    }
    switch (updateType) {
        case 6:  lfgState_ = LfgState::Queued; owner_.addSystemChatMessage("Dungeon Finder: You have joined the queue."); break;
        case 11: lfgState_ = LfgState::Proposal; owner_.addSystemChatMessage("Dungeon Finder: A group has been found!"); break;
        case 12: lfgState_ = LfgState::Queued; owner_.addSystemChatMessage("Dungeon Finder: Added to queue."); break;
        case 14: lfgState_ = LfgState::InDungeon; break;
        default: break;
    }
    // Queue state changed - GetLFGInfoServer answers the new one.
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("LFG_UPDATE", {});
}

void SocialHandler::handleLfgPlayerReward(network::Packet& packet) {
    // SMSG_LFG_PLAYER_REWARD, against WorldSession::SendLfgPlayerReward:
    //
    //   uint32 rdungeonEntry   the random entry queued for
    //   uint32 sdungeonEntry   the dungeon actually finished
    //   uint8  done
    //   uint32 1               a constant the server writes and nothing reads
    //   uint32 money           GetRewOrReqMoney(level), in copper
    //   uint32 xp              XPValue(level)
    //   uint32 0
    //   uint32 0
    //   uint8  itemNum
    //   itemNum x { uint32 itemId, uint32 displayId, uint32 count }
    //
    // This read the constant as the money, the money as the XP and the XP as
    // the item count - so the line it printed was always "1c" followed by the
    // real money labelled XP, and then it looped tens of thousands of times
    // over whatever came next looking for items.
    //
    // packet_layout_check does cover this opcode and did not catch it, for a
    // reason worth keeping: it compares field *widths*, and every field misread
    // here is four bytes wide. Delete one of them and it reports the shape
    // change immediately; shift the meaning of five fields that are all uint32
    // and the two readings line up byte for byte. A same-width misalignment is
    // the one shape that sweep cannot see.
    if (!packet.hasRemaining(4 + 4 + 1 + 4 + 4 + 4 + 4 + 4 + 1)) return;
    LfgCompletionReward reward;
    reward.randomDungeonId = packet.readUInt32();
    reward.dungeonId       = packet.readUInt32();
    reward.done            = packet.readUInt8() != 0;
    packet.readUInt32();   // the constant 1
    reward.money           = packet.readUInt32();
    reward.xp              = packet.readUInt32();
    packet.readUInt32();
    packet.readUInt32();
    const uint8_t itemNum = packet.readUInt8();
    for (uint8_t i = 0; i < itemNum && packet.hasRemaining(12); ++i) {
        LfgCompletionReward::Item item;
        item.itemId    = packet.readUInt32();
        item.displayId = packet.readUInt32();
        item.count     = packet.readUInt32();
        reward.items.push_back(item);
    }
    reward.valid = true;
    lfgCompletionReward_ = reward;

    const uint32_t money = reward.money;
    const uint32_t xp = reward.xp;
    const std::string moneyBuf = game::formatCopperPrice(money);
    std::string rewardMsg = std::string("Dungeon Finder reward: ") + moneyBuf + ", " + std::to_string(xp) + " XP";
    if (!reward.items.empty()) {
        const auto& first = reward.items.front();
        std::string itemLabel = "item #" + std::to_string(first.itemId);
        uint32_t lfgItemQuality = 1;
        if (const ItemQueryResponseData* info = owner_.getItemInfo(first.itemId)) {
            if (!info->name.empty()) itemLabel = info->name;
            lfgItemQuality = info->quality;
        }
        rewardMsg += ", " + buildItemLink(first.itemId, lfgItemQuality, itemLabel);
        if (first.count > 1) rewardMsg += " x" + std::to_string(first.count);
    }
    lfgState_ = LfgState::FinishedDungeon;

    // The reward toast. It carries no arguments - the frame reads the nine
    // values back through GetLFGCompletionReward - so the store above has to
    // be written before this fires.
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("LFG_COMPLETION_REWARD", {});

    // ...and the hand-built chat line only when this client is drawing its own
    // chat, for the same reason /played's is conditional: FrameXML says this
    // its own way, in a toast rather than a line, and printing both is two
    // announcements of one reward.
    if (!ui::frameXmlOwns(ui::UiElement::DungeonFinder)) {
        owner_.addSystemChatMessage(rewardMsg);
    }
}

void SocialHandler::handleLfgBootProposalUpdate(network::Packet& packet) {
    // The victim's GUID sits between the three flags and the counts, and was
    // not being read - so every number after it came from eight bytes earlier
    // than it should have.
    if (!packet.hasRemaining(27)) return;
    bool inProgress = packet.readUInt8() != 0;
    // Both were read and dropped. The vote dialog needs them: didVote decides
    // whether it shows the buttons or the tally, and myVote which of the two
    // is lit.
    lfgBootDidVote_ = packet.readUInt8() != 0;
    lfgBootMyVote_  = packet.readUInt8() != 0;
    lfgBootInProgress_ = inProgress;
    lfgBootVictimGuid_ = packet.readUInt64();
    uint32_t totalVotes = packet.readUInt32();
    uint32_t bootVotes = packet.readUInt32();
    uint32_t timeLeft = packet.readUInt32();
    uint32_t votesNeeded = packet.readUInt32();
    lfgBootVotes_ = bootVotes; lfgBootTotal_ = totalVotes;
    lfgBootTimeLeft_ = timeLeft; lfgBootNeeded_ = votesNeeded;
    if (packet.getReadPos() < packet.getSize()) lfgBootReason_ = packet.readString();
    // The packet carries one string and it is the reason. The victim is the
    // guid above - AzerothCore writes `data << boot.victim` and then the
    // reason, and nothing else - so a second string was being looked for that
    // is never there, and the name it was meant to fill stayed empty. Resolved
    // from the guid instead, which is how every other name here is found.
    lfgBootTargetName_ = owner_.lookupName(lfgBootVictimGuid_);
    if (lfgBootTargetName_.empty() && lfgBootVictimGuid_ != 0) {
        owner_.queryPlayerName(lfgBootVictimGuid_);
    }
    if (inProgress) { lfgState_ = LfgState::Boot; }
    else {
        const bool bootPassed = (bootVotes >= votesNeeded);
        lfgBootVotes_ = lfgBootTotal_ = lfgBootTimeLeft_ = lfgBootNeeded_ = 0;
        lfgBootTargetName_.clear(); lfgBootReason_.clear();
        lfgBootDidVote_ = lfgBootMyVote_ = lfgBootInProgress_ = false;
        lfgBootVictimGuid_ = 0;
        lfgState_ = LfgState::InDungeon;
        owner_.addSystemChatMessage(bootPassed ? "Dungeon Finder: Vote kick passed - member removed." : "Dungeon Finder: Vote kick failed.");
    }
    // The vote-to-kick dialog, whose countdown starts on this.
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("LFG_BOOT_PROPOSAL_UPDATE", {});
}

void SocialHandler::handleLfgTeleportDenied(network::Packet& packet) {
    // A uint32, read as a byte. Little-endian kept it working - every reason
    // is a small number - but the read was the wrong width and the next edit
    // to this handler would have inherited it.
    if (!packet.hasRemaining(4)) return;
    const uint32_t reason = packet.readUInt32();
    owner_.addSystemChatMessage(std::string("Dungeon Finder: ") + lfgTeleportDeniedString(reason));
}

// ============================================================
// LFG Outgoing Packets
// ============================================================

void SocialHandler::lfgJoin(const std::vector<uint32_t>& dungeonIds, uint8_t roles) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (dungeonIds.empty()) return;   // the server drops a join with no slots
    // The dungeon finder is Wrath's. Classic and TBC have no CMSG_LFG_JOIN at
    // all - TBC's 0x35C is CMSG_LFG_SET_AUTOJOIN, an unrelated packet - so the
    // logical opcode is unmapped there and wireOpcode answers 0xFFFF. Sending
    // that would put a packet the server has no handler for on the wire.
    if (wireOpcode(Opcode::CMSG_LFG_JOIN) == 0xFFFF) return;
    owner_.getSocket()->send(LfgJoinPacket::build(dungeonIds, roles));
}

void SocialHandler::lfgLeave() {
    if (!owner_.getSocket()) return;
    lfgState_ = LfgState::None;
    const uint16_t wire = wireOpcode(Opcode::CMSG_LFG_LEAVE);
    if (wire == 0xFFFF) return;   // no dungeon finder before Wrath
    network::Packet pkt(wire);
    pkt.writeUInt32(0); pkt.writeUInt32(0); pkt.writeUInt32(0);
    owner_.getSocket()->send(pkt);
}

void SocialHandler::lfgSetRoles(uint8_t roles) {
    lfgOfferedRoles_ = roles;
    // Three frames draw the same three role checkboxes - the dungeon queue, the
    // raid browser queue and the role-check popup - and every one of them ticks
    // from GetLFGRoles when this event arrives. Setting the roles without
    // firing it left whichever frame was clicked correct and the other two
    // showing what they were last drawn with, which is the shape that reads as
    // "it only takes after a relog".
    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("LFG_ROLE_UPDATE", {});
    }
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    const uint32_t wire = wireOpcode(Opcode::CMSG_LFG_SET_ROLES);
    if (wire == 0xFFFF) return;
    network::Packet pkt(static_cast<uint16_t>(wire));
    pkt.writeUInt8(roles);
    owner_.getSocket()->send(pkt);
}

void SocialHandler::lfgAcceptProposal(uint32_t proposalId, bool accept) {
    if (!owner_.getSocket()) return;
    const uint16_t wire = wireOpcode(Opcode::CMSG_LFG_PROPOSAL_RESULT);
    if (wire == 0xFFFF) return;   // no dungeon finder before Wrath
    network::Packet pkt(wire);
    pkt.writeUInt32(proposalId); pkt.writeUInt8(accept ? 1 : 0);
    owner_.getSocket()->send(pkt);
}

void SocialHandler::lfgTeleport(bool toLfgDungeon) {
    if (!owner_.getSocket()) return;
    const uint16_t wire = wireOpcode(Opcode::CMSG_LFG_TELEPORT);
    if (wire == 0xFFFF) return;   // no dungeon finder before Wrath
    network::Packet pkt(wire);
    pkt.writeUInt8(toLfgDungeon ? 0 : 1);
    owner_.getSocket()->send(pkt);
}

void SocialHandler::lfgSetBootVote(bool vote) {
    if (!owner_.getSocket()) return;
    uint16_t wireOp = wireOpcode(Opcode::CMSG_LFG_SET_BOOT_VOTE);
    if (wireOp == 0xFFFF) return;
    network::Packet pkt(wireOp);
    pkt.writeUInt8(vote ? 1 : 0);
    owner_.getSocket()->send(pkt);
}

// ============================================================
// Arena Handlers
// ============================================================

// command(4) + team name + player name + error(4).
//
// The player name was never read, so the error came out of the first four
// bytes of it - a name, so never zero, and every command result read as a
// failure however well it had gone. Where the server sends no player name the
// error slid one byte instead, which is the same fault with a different value.
// Both strings are needed anyway: the server names the team and the player
// separately because most of these results are about one acting on the other.
void SocialHandler::handleArenaTeamCommandResult(network::Packet& packet) {
    if (!packet.hasRemaining(10)) return;
    const uint32_t command = packet.readUInt32();
    const std::string teamName = packet.readString();
    const std::string playerName = packet.readString();
    if (!packet.hasRemaining(4)) return;
    const uint32_t error = packet.readUInt32();

    // ArenaTeamCommandTypes, and it is not a dense range: 2, and everything
    // between 4 and 13, are values the server never sends. The old table read
    // it as 0..5 in a row, so a quit reported itself as a remove and a change
    // of captain fell off the end.
    const char* cmdName = "arena team";
    switch (command) {
        case 0x00: cmdName = "arena team create"; break;
        case 0x01: cmdName = "arena team invite"; break;
        case 0x03: cmdName = "arena team quit"; break;
        case 0x0E: cmdName = "arena team captain"; break;
        default: break;
    }

    // Whichever name the result is about. A create names the team, an invite
    // names the player, and both are sent for the ones that concern both.
    std::string about;
    if (!teamName.empty()) about = teamName;
    if (!playerName.empty())
        about = about.empty() ? playerName : about + " / " + playerName;

    if (error == 0) {
        owner_.addSystemChatMessage(std::string(cmdName) + " succeeded" +
                                    (about.empty() ? "." : ": " + about));
        return;
    }
    owner_.addSystemChatMessage(std::string(cmdName) + " failed" +
                                (about.empty() ? "" : " (" + about + ")") +
                                ": " + arenaTeamErrorString(error));
}

void SocialHandler::handleArenaTeamQueryResponse(network::Packet& packet) {
    if (!packet.hasRemaining(4)) return;
    uint32_t teamId = packet.readUInt32();
    std::string teamName = packet.readString();
    uint32_t teamType = 0;
    if (packet.hasRemaining(4)) teamType = packet.readUInt32();
    bool known = false;
    for (auto& s : arenaTeamStats_) { if (s.teamId == teamId) { s.teamName = teamName; s.teamType = teamType; known = true; break; } }
    if (!known) {
        ArenaTeamStats stub; stub.teamId = teamId; stub.teamName = teamName; stub.teamType = teamType;
        arenaTeamStats_.push_back(std::move(stub));
    }
    // A team's name arriving is what the PvP panel redraws on. Without this the
    // arena tabs kept whatever they were built with, so a team queried after
    // the panel opened stayed nameless until it was closed and opened again.
    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("ARENA_TEAM_UPDATE", {});
    }
}

void SocialHandler::handleArenaTeamRoster(network::Packet& packet) {
    // teamId(4) + unk308(1) + memberCount(4) + teamType(4), then the members.
    //
    // The team type was never read, so the first member's guid was taken from
    // four bytes earlier - the type plus the guid's low half - and every
    // member after it followed from there. The roster was wrong from its first
    // row, and nothing about it looked like a length fault: the packet is long
    // enough either way.
    if (!packet.hasRemaining(13)) return;
    const uint32_t teamId = packet.readUInt32();
    // 3.0.8 added this, and it does more than mark a version: the server writes
    // the two trailing floats per member only when it is set.
    const bool hasTrailingFloats = packet.readUInt8() != 0;
    const uint32_t memberCount = std::min(packet.readUInt32(), 100u);
    /*uint32_t teamType =*/ packet.readUInt32();
    ArenaTeamRoster roster; roster.teamId = teamId; roster.members.reserve(memberCount);
    for (uint32_t i = 0; i < memberCount; ++i) {
        if (!packet.hasRemaining(12)) break;
        ArenaTeamMember m;
        m.guid = packet.readUInt64(); m.online = (packet.readUInt8() != 0); m.name = packet.readString();
        // Captain flag, level and class sit between the name and the record -
        // six bytes this went straight past, so the games played were the
        // captain flag, the wins were the level and class with two bytes of the
        // real figure behind them, and so on down the row.
        if (!packet.hasRemaining(6)) break;
        m.isCaptain = (packet.readUInt32() == 0);   // 0 captain, 1 member
        m.level     = packet.readUInt8();
        m.classId   = packet.readUInt8();
        if (!packet.hasRemaining(20)) break;
        m.weekGames = packet.readUInt32(); m.weekWins = packet.readUInt32();
        m.seasonGames = packet.readUInt32(); m.seasonWins = packet.readUInt32(); m.personalRating = packet.readUInt32();
        // Only when the header said so. Taking them whenever eight bytes were
        // left ate the next member's guid on every roster of more than one.
        if (hasTrailingFloats) {
            if (!packet.hasRemaining(8)) break;
            packet.readFloat(); packet.readFloat();
        }
        roster.members.push_back(std::move(m));
    }
    bool replaced = false;
    for (auto& r : arenaTeamRosters_) { if (r.teamId == teamId) { r = std::move(roster); replaced = true; break; } }
    if (!replaced) arenaTeamRosters_.push_back(std::move(roster));

    // The roster is stored and the arena panel is told, which it was not: the
    // team detail window redraws on this event and on nothing else, so a
    // roster that arrived after the window opened was held here and never
    // shown.
    //
    // Fired with no arguments, and that is the whole contract. pvpframe reads
    // arg1 as "the roster you hold is stale, ask for it again" and only
    // redraws from what arrived when arg1 is absent - so any argument at all
    // would answer this roster by requesting another one, forever. A zero
    // would do it too, zero being true in Lua.
    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("ARENA_TEAM_ROSTER_UPDATE", {});
    }
}

void SocialHandler::handleArenaTeamInvite(network::Packet& packet) {
    std::string playerName = packet.readString();
    std::string teamName = packet.readString();
    owner_.addSystemChatMessage(playerName + " has invited you to join " + teamName + ".");
}

void SocialHandler::handleArenaTeamEvent(network::Packet& packet) {
    if (!packet.hasRemaining(1)) return;
    uint8_t event = packet.readUInt8();
    uint8_t strCount = 0;
    if (packet.hasRemaining(1)) strCount = packet.readUInt8();
    std::string param1, param2;
    if (strCount >= 1 && packet.getSize() > packet.getReadPos()) param1 = packet.readString();
    if (strCount >= 2 && packet.getSize() > packet.getReadPos()) param2 = packet.readString();
    std::string msg;
    switch (event) {
        case 0: msg = param1.empty() ? "A player has joined your arena team." : param1 + " has joined your arena team."; break;
        case 1: msg = param1.empty() ? "A player has left the arena team." : param1 + " has left the arena team."; break;
        case 2: msg = (!param1.empty() && !param2.empty()) ? param1 + " has been removed from the arena team by " + param2 + "." : "A player has been removed from the arena team."; break;
        case 3: msg = param1.empty() ? "The arena team captain has changed." : param1 + " is now the arena team captain."; break;
        case 4: msg = "Your arena team has been disbanded."; break;
        case 5: msg = param1.empty() ? "Your arena team has been created." : "Arena team \"" + param1 + "\" has been created."; break;
        default: msg = "Arena team event " + std::to_string(event); if (!param1.empty()) msg += ": " + param1; break;
    }
    owner_.addSystemChatMessage(msg);
}

void SocialHandler::handleArenaTeamStats(network::Packet& packet) {
    if (!packet.hasRemaining(28)) return;
    ArenaTeamStats stats;
    stats.teamId = packet.readUInt32(); stats.rating = packet.readUInt32();
    stats.weekGames = packet.readUInt32(); stats.weekWins = packet.readUInt32();
    stats.seasonGames = packet.readUInt32(); stats.seasonWins = packet.readUInt32();
    stats.rank = packet.readUInt32();
    for (auto& s : arenaTeamStats_) {
        if (s.teamId == stats.teamId) { stats.teamName = std::move(s.teamName); stats.teamType = s.teamType; s = std::move(stats); return; }
    }
    arenaTeamStats_.push_back(std::move(stats));
}

void SocialHandler::requestArenaTeamRoster(uint32_t teamId) {
    if (!owner_.getSocket()) return;
    network::Packet pkt(wireOpcode(Opcode::CMSG_ARENA_TEAM_ROSTER));
    pkt.writeUInt32(teamId);
    owner_.getSocket()->send(pkt);
}

void SocialHandler::handleArenaError(network::Packet& packet) {
    if (!packet.hasRemaining(4)) return;
    uint32_t error = packet.readUInt32();
    std::string msg;
    switch (error) {
        case 1: msg = "The other team is not big enough."; break;
        case 2: msg = "That team is full."; break;
        case 3: msg = "Not enough members to start."; break;
        case 4: msg = "Too many members."; break;
        default: msg = "Arena error (code " + std::to_string(error) + ")"; break;
    }
    owner_.addSystemChatMessage(msg);
}

void SocialHandler::handlePvpLogData(network::Packet& packet) {
    auto remaining = [&]() { return packet.getRemainingSize(); };
    if (remaining() < 1) return;
    // The layout is settled and the dump that asked about it is gone.
    //
    // The question it existed to answer - whether there is a team byte after
    // the guid - is answered by the source rather than by a byte count, and the
    // answer is "in an arena, yes; in a battleground, no". They are two
    // different rows written by two different overrides, and the type byte at
    // the top of this packet says which one follows.
    bgScoreboard_ = BgScoreboardData{};
    bgScoreboard_.isArena = (packet.readUInt8() != 0);
    if (bgScoreboard_.isArena) {
        for (auto& arenaTeam : bgScoreboard_.arenaTeams) {
            if (remaining() < 20) { packet.skipAll(); return; }
            arenaTeam.ratingChange = packet.readUInt32();
            arenaTeam.newRating = packet.readUInt32();
            packet.readUInt32(); packet.readUInt32(); packet.readUInt32();
            arenaTeam.teamName = remaining() > 0 ? packet.readString() : "";
        }
    }
    // Ended, and the winner, BEFORE the player count. This was read after the
    // players, where there is nothing left to read it from.
    if (remaining() < 1) return;
    bgScoreboard_.hasWinner = (packet.readUInt8() != 0);
    if (bgScoreboard_.hasWinner) {
        if (remaining() < 1) return;
        bgScoreboard_.winner = packet.readUInt8();
    }

    if (remaining() < 4) return;
    uint32_t playerCount = packet.readUInt32();
    bgScoreboard_.players.reserve(playerCount);
    for (uint32_t i = 0; i < playerCount && remaining() >= 12; ++i) {
        BgPlayerScore ps;
        ps.guid = packet.readUInt64();
        // Two different rows, and the type byte at the top says which.
        //
        // BattlegroundScore::AppendToPacket - killing blows, honourable kills,
        // deaths, bonus honour, damage, healing. No team.
        // ArenaScore::AppendToPacket - killing blows, a team BYTE, damage,
        // healing. No honourable kills, deaths or honour at all.
        //
        // This read one row that was neither: a team byte from the arena
        // shape followed by the battleground's four counters, then a stat block
        // with null-terminated names that no core writes. Everything after the
        // guid was off by a byte, and damage and healing were skipped entirely
        // - which is why GetBattlefieldScore answered zero for both.
        ps.killingBlows = packet.readUInt32();
        if (bgScoreboard_.isArena) {
            if (remaining() < 1) { bgScoreboard_.players.push_back(std::move(ps)); break; }
            ps.team = packet.readUInt8();
            ps.hasTeam = true;
        } else {
            if (remaining() < 12) { bgScoreboard_.players.push_back(std::move(ps)); break; }
            ps.honorableKills = packet.readUInt32();
            ps.deaths         = packet.readUInt32();
            ps.bonusHonor     = packet.readUInt32();
            // No team on the wire for a battleground. The byte that used to
            // fill this was the low byte of the killing blows, so the faction
            // tabs were already filtering on nothing; leaving it unset is the
            // same amount of information, honestly labelled.
        }
        if (remaining() < 8) { bgScoreboard_.players.push_back(std::move(ps)); break; }
        ps.damageDone  = packet.readUInt32();
        ps.healingDone = packet.readUInt32();

        { auto ent = owner_.getEntityManager().getEntity(ps.guid);
          if (ent && (ent->getType() == game::ObjectType::PLAYER || ent->getType() == game::ObjectType::UNIT))
              { auto u = std::static_pointer_cast<game::Unit>(ent); if (!u->getName().empty()) ps.name = u->getName(); } }

        // BuildObjectivesBlock: a count and that many bare values. The names
        // this used to read are not on the wire - BattlegroundWS writes
        // uint32(2) and two numbers - so every value past the first came from
        // the wrong offset.
        if (remaining() >= 4) {
            uint32_t statCount = packet.readUInt32();
            for (uint32_t st = 0; st < statCount && remaining() >= 4; ++st) {
                ps.bgStats.emplace_back(std::string(), packet.readUInt32());
            }
        }
        bgScoreboard_.players.push_back(std::move(ps));
    }
    // The scoreboard is drawn from this and asked for it by name -
    // RequestBattlefieldScoreData sends the query, UPDATE_BATTLEFIELD_SCORE is
    // the answer arriving. Every row, every column and the winner were parsed
    // and stored, and the frame that displays them was never told.
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("UPDATE_BATTLEFIELD_SCORE", {});
}

void SocialHandler::updateLogoutCountdown(float deltaTime) {
    if (loggingOut_ && logoutCountdown_ > 0.0f) {
        logoutCountdown_ -= deltaTime;
        if (logoutCountdown_ < 0.0f) logoutCountdown_ = 0.0f;
    }
    // The bind prompt expires on its own; the server stops asking and the
    // pending bind lapses, so the dialog has to stop offering an answer.
    if (instanceLock_.active) {
        instanceLock_.secondsLeft -= deltaTime;
        if (instanceLock_.secondsLeft <= 0.0f) {
            instanceLock_.secondsLeft = 0.0f;
            instanceLock_.active = false;
            owner_.fireAddonEvent("INSTANCE_LOCK_STOP", {});
        }
    }
}

void SocialHandler::respondInstanceLock(bool accept) {
    // Accepting saves the player to the instance; declining sends them to the
    // graveyard. The server acts on whichever arrives and clears the pending
    // bind, so sending twice is worse than not sending at all.
    if (!instanceLock_.active || !owner_.getSocket()) return;
    instanceLock_.active = false;

    network::Packet resp(wireOpcode(Opcode::CMSG_INSTANCE_LOCK_RESPONSE));
    resp.writeUInt8(accept ? 1 : 0);
    owner_.getSocket()->send(resp);

    LOG_INFO("CMSG_INSTANCE_LOCK_RESPONSE: ", accept ? "accepted" : "declined");
    owner_.fireAddonEvent("INSTANCE_LOCK_STOP", {});
}

void SocialHandler::resetTransferState() {
    instanceLock_ = InstanceLockPrompt{};
    encounterUnitGuids_.fill(0);
    raidTargetGuids_.fill(0);
}

// ============================================================
// Moved opcode handlers (from GameHandler::registerOpcodeHandlers)
// ============================================================

void SocialHandler::handleInitializeFactions(network::Packet& packet) {
    if (!packet.hasRemaining(4)) return;
    uint32_t count = packet.readUInt32();
    size_t needed = static_cast<size_t>(count) * 5;
    if (!packet.hasRemaining(needed)) { packet.skipAll(); return; }
    owner_.initialFactionsRef().clear();
    owner_.initialFactionsRef().reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
        GameHandler::FactionStandingInit fs{};
        fs.flags = packet.readUInt8();
        fs.standing = static_cast<int32_t>(packet.readUInt32());
        owner_.initialFactionsRef().push_back(fs);
    }
    // Also keyed by faction, which is how everything that draws a standing
    // asks for one. This packet is indexed by ReputationListID and the map is
    // indexed by faction id, and only SMSG_SET_FACTION_STANDING wrote to the
    // map - so a standing was known from login and unreadable until it next
    // moved. The reputation tab drew every bar empty at Neutral however much
    // the character had earned, and so did this client's own panel and the
    // watched-faction bar, all three reading the same empty map.
    owner_.loadFactionNameCache();
    for (size_t repListId = 0; repListId < owner_.initialFactionsRef().size(); ++repListId) {
        const uint32_t factionId =
            owner_.getFactionIdByRepListId(static_cast<uint32_t>(repListId));
        if (factionId == 0) continue;
        owner_.factionStandingsRef()[factionId] =
            owner_.initialFactionsRef()[repListId].standing;
    }
    LOG_INFO("Reputation: ", owner_.initialFactionsRef().size(),
             " factions initialised, ", owner_.factionStandingsRef().size(),
             " resolved to a faction id");
}

void SocialHandler::handleSetFactionStanding(network::Packet& packet) {
    // Expansion wire layouts differ before the shared count + entries payload:
    //   Classic: count
    //   TBC:     RAF bonus (float), count
    //   WotLK:   RAF bonus (float), showVisual (uint8), count
    // Treating WotLK's first float byte as showVisual shifted the entire packet,
    // producing bogus counts, faction names, standings, and chat messages.
    if (isActiveExpansion("wotlk")) {
        if (!packet.hasRemaining(9)) return;
        /*float rafBonus =*/ packet.readFloat();
        /*uint8_t showVisual =*/ packet.readUInt8();
    } else if (isActiveExpansion("tbc")) {
        if (!packet.hasRemaining(8)) return;
        /*float rafBonus =*/ packet.readFloat();
    } else if (!packet.hasRemaining(4)) {
        return;
    }

    uint32_t count = packet.readUInt32();
    count = std::min(count, 128u);
    owner_.loadFactionNameCache();
    for (uint32_t i = 0; i < count && packet.hasRemaining(8); ++i) {
        // The packet carries Faction.dbc ReputationListID, not the faction ID.
        uint32_t repListId = packet.readUInt32();
        int32_t  standing  = static_cast<int32_t>(packet.readUInt32());

        uint32_t factionId = owner_.getFactionIdByRepListId(repListId);
        if (factionId == 0) {
            LOG_WARNING("SMSG_SET_FACTION_STANDING: unknown repListId=", repListId,
                        " standing=", standing);
            continue;
        }

        int32_t  oldStanding = 0;
        // SMSG_INITIALIZE_FACTIONS is indexed by ReputationListID and supplies the
        // standing before incremental updates. Without using it, the first message
        // reports the character's entire accumulated reputation as the gained amount.
        if (repListId < owner_.initialFactionsRef().size()) {
            oldStanding = owner_.initialFactionsRef()[repListId].standing;
            owner_.initialFactionsRef()[repListId].standing = standing;
        } else {
            auto it = owner_.factionStandingsRef().find(factionId);
            if (it != owner_.factionStandingsRef().end()) oldStanding = it->second;
        }
        owner_.factionStandingsRef()[factionId] = standing;
        int32_t delta = standing - oldStanding;
        if (delta != 0) {
            std::string name = owner_.getFactionName(factionId);
            char buf[256];
            std::snprintf(buf, sizeof(buf), "Reputation with %s %s by %d.",
                          name.c_str(), delta > 0 ? "increased" : "decreased", std::abs(delta));
            owner_.addSystemChatMessage(buf);
            owner_.watchedFactionIdRef() = factionId;
            if (owner_.repChangeCallbackRef()) owner_.repChangeCallbackRef()(name, delta, standing);
            // These events fire unconditionally on any rep change (not gated by callback).
            owner_.fireAddonEvent("UPDATE_FACTION", {});
            owner_.addLocalChatLine(ChatType::COMBAT_FACTION_CHANGE, std::string(buf));
        }
    }
}

void SocialHandler::handleSetFactionAtWar(network::Packet& packet) {
    if (!packet.hasRemaining(5)) { packet.skipAll(); return; }
    uint32_t repListId = packet.readUInt32();
    uint8_t  setAtWar  = packet.readUInt8();
    if (repListId < owner_.initialFactionsRef().size()) {
        if (setAtWar)
            owner_.initialFactionsRef()[repListId].flags |=  GameHandler::FACTION_FLAG_AT_WAR;
        else
            owner_.initialFactionsRef()[repListId].flags &= ~GameHandler::FACTION_FLAG_AT_WAR;
    }
}

void SocialHandler::handleSetFactionVisible(network::Packet& packet) {
    if (!packet.hasRemaining(5)) { packet.skipAll(); return; }
    uint32_t repListId = packet.readUInt32();
    uint8_t  visible   = packet.readUInt8();
    if (repListId < owner_.initialFactionsRef().size()) {
        if (visible)
            owner_.initialFactionsRef()[repListId].flags |=  GameHandler::FACTION_FLAG_VISIBLE;
        else
            owner_.initialFactionsRef()[repListId].flags &= ~GameHandler::FACTION_FLAG_VISIBLE;
    }
}

void SocialHandler::handleGroupSetLeader(network::Packet& packet) {
    if (!packet.hasData()) return;
    std::string leaderName = packet.readString();
    auto& pd = mutablePartyData();
    for (const auto& m : pd.members) {
        if (m.name == leaderName) { pd.leaderGuid = m.guid; break; }
    }
    if (!leaderName.empty())
        owner_.addSystemChatMessage(leaderName + " is now the group leader.");
    owner_.fireAddonEvent("PARTY_LEADER_CHANGED", {});
    owner_.fireAddonEvent("GROUP_ROSTER_UPDATE", {});
}

// ============================================================
// Minimap Ping
// ============================================================

void SocialHandler::sendMinimapPing(float wowX, float wowY) {
    if (owner_.getState() != WorldState::IN_WORLD) return;

    // MSG_MINIMAP_PING (CMSG direction): float posX + float posY
    // Server convention: posX = east/west axis = canonical Y (west)
    //                    posY = north/south axis = canonical X (north)
    const float serverX = wowY;  // canonical Y (west) → server posX
    const float serverY = wowX;  // canonical X (north) → server posY

    network::Packet pkt(wireOpcode(Opcode::MSG_MINIMAP_PING));
    pkt.writeFloat(serverX);
    pkt.writeFloat(serverY);
    owner_.getSocket()->send(pkt);

    // Add ping locally so the sender sees their own ping immediately
    GameHandler::MinimapPing localPing;
    localPing.senderGuid = owner_.activeCharacterGuidRef();
    localPing.wowX       = wowX;
    localPing.wowY       = wowY;
    localPing.age        = 0.0f;
    owner_.minimapPingsRef().push_back(localPing);
}

// ============================================================
// Summon Request
// ============================================================

void SocialHandler::handleSummonRequest(network::Packet& packet) {
    if (!packet.hasRemaining(16)) return;

    owner_.summonerGuidRef()        = packet.readUInt64();
    uint32_t zoneId             = packet.readUInt32();
    uint32_t timeoutMs          = packet.readUInt32();
    owner_.summonTimeoutSecRef()    = timeoutMs / 1000.0f;
    owner_.pendingSummonRequestRef()= true;

    owner_.summonerNameRef().clear();
    if (auto* unit = owner_.getUnitByGuid(owner_.summonerGuidRef())) {
        owner_.summonerNameRef() = unit->getName();
    }
    if (owner_.summonerNameRef().empty()) {
        owner_.summonerNameRef() = owner_.lookupName(owner_.summonerGuidRef());
    }
    if (owner_.summonerNameRef().empty()) {
        char tmp[32];
        std::snprintf(tmp, sizeof(tmp), "0x%llX",
                      static_cast<unsigned long long>(owner_.summonerGuidRef()));
        owner_.summonerNameRef() = tmp;
    }

    std::string msg = owner_.summonerNameRef() + " is summoning you";
    std::string zoneName = owner_.getAreaName(zoneId);
    if (!zoneName.empty())
        msg += " to " + zoneName;
    msg += '.';
    owner_.addSystemChatMessage(msg);
    LOG_INFO("SMSG_SUMMON_REQUEST: summoner=", owner_.summonerNameRef(),
             " zoneId=", zoneId, " timeout=", owner_.summonTimeoutSecRef(), "s");
    owner_.fireAddonEvent("CONFIRM_SUMMON", {});
}

/// Both answers to a summon: who summoned, then yes or no.
///
/// HandleSummonResponseOpcode reads `ObjectGuid summoner_guid` and then a bool,
/// nine bytes. Both answers were a single byte, so the server had one byte
/// where it wanted eight, ran off the end of the buffer and dropped the packet
/// - accepting a summon has never once reached it, and neither has declining
/// one. The same mistake as the battlefield answers below, made next door.
///
/// The guid is the one SMSG_SUMMON_REQUEST arrived with, which is already kept
/// so the message can name whoever cast it.
void SocialHandler::sendSummonResponse(bool accept) {
    if (!owner_.getSocket()) return;
    owner_.pendingSummonRequestRef() = false;
    network::Packet pkt(wireOpcode(Opcode::CMSG_SUMMON_RESPONSE));
    pkt.writeUInt64(owner_.summonerGuidRef());
    pkt.writeUInt8(accept ? 1 : 0);
    owner_.getSocket()->send(pkt);
}

void SocialHandler::acceptSummon() {
    if (!owner_.pendingSummonRequestRef()) return;
    sendSummonResponse(true);
    owner_.addSystemChatMessage("Accepting summon...");
    LOG_INFO("Accepted summon from ", owner_.summonerNameRef());
}

void SocialHandler::declineSummon() {
    sendSummonResponse(false);
    owner_.addSystemChatMessage("Summon declined.");
}

// ============================================================
// Battlefield Manager
// ============================================================

/// All three battlefield answers carry the battle id first.
///
/// HandleBfEntryInviteResponse reads `uint32 BattleId >> uint8 Accepted` and
/// the exit request reads a battle id alone. Both answers used to be a single
/// byte, so the server read the accept flag as the low byte of a battle id,
/// ran out of buffer and dropped the packet - accepting a Wintergrasp
/// invitation had never once reached it.
void SocialHandler::sendBfMgrResponse(Opcode op, uint32_t battleId, bool accept,
                                      bool withFlag) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    network::Packet pkt(wireOpcode(op));
    pkt.writeUInt32(battleId);
    if (withFlag) pkt.writeUInt8(accept ? 1 : 0);
    owner_.getSocket()->send(pkt);
}

void SocialHandler::acceptBfMgrInvite() {
    if (!owner_.bfMgrInvitePendingRef()) return;
    sendBfMgrResponse(Opcode::CMSG_BATTLEFIELD_MGR_ENTRY_INVITE_RESPONSE,
                      owner_.getBfMgrBattleId(), true, true);
    owner_.bfMgrInvitePendingRef() = false;
    LOG_INFO("acceptBfMgrInvite: battle ", owner_.getBfMgrBattleId());
}

void SocialHandler::declineBfMgrInvite() {
    if (!owner_.bfMgrInvitePendingRef()) return;
    sendBfMgrResponse(Opcode::CMSG_BATTLEFIELD_MGR_ENTRY_INVITE_RESPONSE,
                      owner_.getBfMgrBattleId(), false, true);
    owner_.bfMgrInvitePendingRef() = false;
    LOG_INFO("declineBfMgrInvite: battle ", owner_.getBfMgrBattleId());
}

void SocialHandler::respondBfMgrQueueInvite(uint32_t battleId, bool accept) {
    sendBfMgrResponse(Opcode::CMSG_BATTLEFIELD_MGR_QUEUE_INVITE_RESPONSE,
                      battleId, accept, true);
    owner_.bfMgrInvitePendingRef() = false;
    LOG_INFO("respondBfMgrQueueInvite: battle ", battleId,
             accept ? " accepted" : " declined");
}

void SocialHandler::requestBfMgrExit(uint32_t battleId) {
    sendBfMgrResponse(Opcode::CMSG_BATTLEFIELD_MGR_EXIT_REQUEST,
                      battleId, false, false);
    LOG_INFO("requestBfMgrExit: battle ", battleId);
}

// ============================================================
// Calendar
// ============================================================

void SocialHandler::inviteToCalendarEvent(uint64_t eventId, uint64_t inviteId,
                                          const std::string& name,
                                          bool isPreInvite, bool isGuildEvent) {
    if (!owner_.isInWorld()) return;
    // eventId, inviteId, name, isPreInvite, isGuildEvent - CalendarHandler.cpp:515.
    // The two flags are ByteBuffer bools, which are one byte each on the wire.
    network::Packet pkt(wireOpcode(Opcode::CMSG_CALENDAR_EVENT_INVITE));
    pkt.writeUInt64(eventId);
    pkt.writeUInt64(inviteId);
    pkt.writeString(name);
    pkt.writeUInt8(isPreInvite ? 1 : 0);
    pkt.writeUInt8(isGuildEvent ? 1 : 0);
    owner_.getSocket()->send(pkt);
    LOG_INFO("inviteToCalendarEvent: '", name, "' to event ", eventId,
             isPreInvite ? " (before it is created)" : "");
}

void SocialHandler::massInviteGuildToCalendarEvent(uint32_t minLevel,
                                                   uint32_t maxLevel,
                                                   uint32_t minRank) {
    if (!owner_.isInWorld()) return;
    // Its own opcode, not an invite with the name left out: the server picks
    // the members itself from these three bounds and never sees a name
    // (CalendarHandler.cpp:198, and Guild::MassInviteToEvent behind it).
    network::Packet pkt(wireOpcode(Opcode::CMSG_CALENDAR_GUILD_FILTER));
    pkt.writeUInt32(minLevel);
    pkt.writeUInt32(maxLevel);
    pkt.writeUInt32(minRank);
    owner_.getSocket()->send(pkt);
    LOG_INFO("massInviteGuildToCalendarEvent: levels ", minLevel, "-", maxLevel,
             ", rank ", minRank, " and above");
}

void SocialHandler::updateCalendarEvent(uint64_t eventId, uint64_t inviteId,
                                        const CalendarEventDraft& draft) {
    if (!owner_.isInWorld()) return;
    // The same body as adding, with the event and invite ids in front of it
    // (CalendarHandler.cpp:360). Written out rather than shared with
    // createCalendarEvent: the two are the same shape today and are two
    // different opcodes, and folding them together would make the next change
    // to one silently change the other.
    network::Packet pkt(wireOpcode(Opcode::CMSG_CALENDAR_UPDATE_EVENT));
    pkt.writeUInt64(eventId);
    pkt.writeUInt64(inviteId);
    pkt.writeString(draft.title);
    pkt.writeString(draft.description);
    pkt.writeUInt8(draft.type);
    pkt.writeUInt8(draft.repeatOption);
    pkt.writeUInt32(draft.maxInvites);
    pkt.writeUInt32(static_cast<uint32_t>(draft.dungeonId));
    pkt.writeUInt32(packWowPackedTime(draft.eventTime));
    pkt.writeUInt32(packWowPackedTime(draft.eventTime));
    pkt.writeUInt32(draft.flags);
    owner_.getSocket()->send(pkt);
    LOG_INFO("updateCalendarEvent: ", eventId, " '", draft.title, "'");
}

void SocialHandler::removeCalendarEvent(uint64_t eventId, uint64_t inviteId) {
    if (!owner_.isInWorld()) return;
    // The server reads the id and discards the rest, but the rest is still
    // written: it calls rfinish() rather than stopping, so a short packet is
    // a short read on its side rather than a tidy one.
    network::Packet pkt(wireOpcode(Opcode::CMSG_CALENDAR_REMOVE_EVENT));
    pkt.writeUInt64(eventId);
    pkt.writeUInt64(inviteId);
    pkt.writeUInt32(0);   // flags
    owner_.getSocket()->send(pkt);
    LOG_INFO("removeCalendarEvent: ", eventId);
}

void SocialHandler::setCalendarInviteStatus(uint64_t inviteeGuid,
                                            uint64_t eventId, uint64_t inviteId,
                                            uint8_t status) {
    if (!owner_.isInWorld()) return;
    // packedGuid invitee, eventId, inviteId, ownerInviteId, status
    // (CalendarHandler.cpp:695). Setting someone *else's* status, which is what
    // an event's owner does to confirm or bench them - the player's own answer
    // is CMSG_CALENDAR_EVENT_RSVP and a different packet entirely.
    network::Packet pkt(wireOpcode(Opcode::CMSG_CALENDAR_EVENT_STATUS));
    pkt.writePackedGuid(inviteeGuid);
    pkt.writeUInt64(eventId);
    pkt.writeUInt64(inviteId);
    pkt.writeUInt64(0);   // ownerInviteId
    pkt.writeUInt8(status);
    owner_.getSocket()->send(pkt);
    LOG_INFO("setCalendarInviteStatus: invitee ", inviteeGuid, " status ", status);
}

void SocialHandler::setCalendarInviteModerator(uint64_t inviteeGuid,
                                               uint64_t eventId,
                                               uint64_t inviteId, uint8_t rank) {
    if (!owner_.isInWorld()) return;
    // The same shape with a rank in place of the status
    // (CalendarHandler.cpp:727).
    network::Packet pkt(wireOpcode(Opcode::CMSG_CALENDAR_EVENT_MODERATOR_STATUS));
    pkt.writePackedGuid(inviteeGuid);
    pkt.writeUInt64(eventId);
    pkt.writeUInt64(inviteId);
    pkt.writeUInt64(0);   // ownerInviteId
    pkt.writeUInt8(rank);
    owner_.getSocket()->send(pkt);
    LOG_INFO("setCalendarInviteModerator: invitee ", inviteeGuid, " rank ", rank);
}

void SocialHandler::requestCalendarEvent(uint64_t eventId) {
    if (!owner_.isInWorld()) return;
    network::Packet pkt(wireOpcode(Opcode::CMSG_CALENDAR_GET_EVENT));
    pkt.writeUInt64(eventId);
    owner_.getSocket()->send(pkt);
    LOG_INFO("requestCalendarEvent: ", eventId);
}

void SocialHandler::createCalendarEvent(const CalendarEventDraft& draft) {
    if (!owner_.isInWorld()) return;
    // title, description, type, repeatable, maxInvites, dungeonId, then two
    // packed times and the flags - CalendarHandler.cpp:228.
    //
    // The second time is unused by the server (it is stored and never read),
    // but it is read off the wire, so leaving it out shifts the flags into it
    // and the flags off the end.
    network::Packet pkt(wireOpcode(Opcode::CMSG_CALENDAR_ADD_EVENT));
    pkt.writeString(draft.title);
    pkt.writeString(draft.description);
    pkt.writeUInt8(draft.type);
    pkt.writeUInt8(draft.repeatOption);
    pkt.writeUInt32(draft.maxInvites);
    pkt.writeUInt32(static_cast<uint32_t>(draft.dungeonId));
    pkt.writeUInt32(packWowPackedTime(draft.eventTime));
    pkt.writeUInt32(packWowPackedTime(draft.eventTime));
    pkt.writeUInt32(draft.flags);
    owner_.getSocket()->send(pkt);
    LOG_INFO("createCalendarEvent: '", draft.title, "' type ", draft.type,
             " on ", draft.eventTime.day, "/", draft.eventTime.month, "/",
             draft.eventTime.fullYear());
}

void SocialHandler::respondToCalendarInvite(uint64_t eventId, uint64_t inviteId,
                                            uint32_t status) {
    if (!owner_.isInWorld()) return;
    // uint64 eventId, uint64 inviteId, uint32 status - CalendarHandler.cpp:630.
    // The status is a CalendarInviteStatus: 1 accepted, 2 declined, 8
    // tentative, 9 removed. The server answers with an event-status alert and
    // clears the pending action, so nothing is assumed here.
    network::Packet pkt(wireOpcode(Opcode::CMSG_CALENDAR_EVENT_RSVP));
    pkt.writeUInt64(eventId);
    pkt.writeUInt64(inviteId);
    pkt.writeUInt32(status);
    owner_.getSocket()->send(pkt);
    LOG_INFO("respondToCalendarInvite: event ", eventId, " invite ", inviteId,
             " status ", status);
}

void SocialHandler::requestCalendar() {
    if (!owner_.isInWorld()) return;
    // CMSG_CALENDAR_GET_CALENDAR has no payload
    network::Packet pkt(wireOpcode(Opcode::CMSG_CALENDAR_GET_CALENDAR));
    owner_.getSocket()->send(pkt);
    LOG_INFO("requestCalendar: sent CMSG_CALENDAR_GET_CALENDAR");
    // Also request pending invite count
    network::Packet numPkt(wireOpcode(Opcode::CMSG_CALENDAR_GET_NUM_PENDING));
    owner_.getSocket()->send(numPkt);
}

// ============================================================
// Methods moved from GameHandler
// ============================================================

void SocialHandler::sendSetDifficulty(uint32_t difficulty, bool raid) {
    if (!owner_.isInWorld()) {
        LOG_WARNING("Cannot change difficulty: not in world");
        return;
    }

    // MSG_SET_DUNGEON_DIFFICULTY, not CMSG_CHANGEPLAYER_DIFFICULTY. The latter
    // is a real opcode number and the server registers it with Handle_NULL and
    // STATUS_NEVER - it is read off the wire and discarded - so every
    // difficulty change ever sent from here went nowhere. Both of the real ones
    // carry a single uint32 mode.
    const Opcode op = raid ? Opcode::MSG_SET_RAID_DIFFICULTY
                           : Opcode::MSG_SET_DUNGEON_DIFFICULTY;
    network::Packet packet(wireOpcode(op));
    packet.writeUInt32(difficulty);
    owner_.getSocket()->send(packet);
    LOG_INFO(raid ? "MSG_SET_RAID_DIFFICULTY sent: difficulty="
                  : "MSG_SET_DUNGEON_DIFFICULTY sent: difficulty=", difficulty);
}

void SocialHandler::toggleHelm() {
    if (!owner_.isInWorld()) {
        LOG_WARNING("Cannot toggle helm: not in world or not connected");
        return;
    }

    owner_.helmVisibleRef() = !owner_.helmVisibleRef();
    auto packet = ShowingHelmPacket::build(owner_.helmVisibleRef());
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage(owner_.helmVisibleRef() ? "Helm is now visible." : "Helm is now hidden.");
    // The flag alone changed nothing on screen: it was set, sent to the server
    // and never read. Mark equipment dirty so the helm model and the hair under
    // it are both rebuilt from it.
    owner_.markOnlineEquipmentDirty();
    LOG_INFO("Helm visibility toggled: ", owner_.helmVisibleRef());
}

void SocialHandler::toggleCloak() {
    if (!owner_.isInWorld()) {
        LOG_WARNING("Cannot toggle cloak: not in world or not connected");
        return;
    }

    owner_.cloakVisibleRef() = !owner_.cloakVisibleRef();
    auto packet = ShowingCloakPacket::build(owner_.cloakVisibleRef());
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage(owner_.cloakVisibleRef() ? "Cloak is now visible." : "Cloak is now hidden.");
    LOG_INFO("Cloak visibility toggled: ", owner_.cloakVisibleRef());
}

void SocialHandler::setStandState(uint8_t standState) {
    if (!owner_.isInWorld()) {
        LOG_WARNING("Cannot change stand state: not in world or not connected");
        return;
    }

    auto packet = StandStateChangePacket::build(standState);
    owner_.getSocket()->send(packet);
    LOG_INFO("Changed stand state to: ", static_cast<int>(standState));
}

void SocialHandler::sendAlterAppearance(uint32_t hairStyleEntry, uint32_t hairColor,
                                        uint32_t facialHairEntry, uint32_t skinColorEntry) {
    if (!owner_.isInWorld()) return;
    auto pkt = AlterAppearancePacket::build(hairStyleEntry, hairColor,
                                            facialHairEntry, skinColorEntry);
    owner_.getSocket()->send(pkt);
    LOG_INFO("sendAlterAppearance: hairEntry=", hairStyleEntry,
             " color=", hairColor, " facialEntry=", facialHairEntry,
             " skinEntry=", skinColorEntry);
}

void SocialHandler::deleteGmTicket() {
    if (!owner_.isInWorld()) return;
    network::Packet pkt(wireOpcode(Opcode::CMSG_GMTICKET_DELETETICKET));
    owner_.getSocket()->send(pkt);
    owner_.gmTicketActiveRef() = false;
    owner_.gmTicketTextRef().clear();
    LOG_INFO("Deleting GM ticket");
}

void SocialHandler::setGroupAssistant(uint64_t guid, bool apply) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (guid == 0) return;
    network::Packet packet(wireOpcode(Opcode::CMSG_GROUP_ASSISTANT_LEADER));
    packet.writeUInt64(guid);
    packet.writeUInt8(apply ? 1 : 0);
    owner_.getSocket()->send(packet);
}

void SocialHandler::requestContactList() {
    if (!owner_.isInWorld() || !owner_.getSocket()) return;
    // Once a second. FriendsFrame_Update calls ShowFriends every time it
    // redraws, and it redraws on every friend status line - so an unthrottled
    // send answers each one with a request for the whole list again.
    const auto now = std::chrono::steady_clock::now();
    if (now - lastContactListRequest_ < std::chrono::seconds(1)) return;
    lastContactListRequest_ = now;
    // Two names for one request. Vanilla calls it CMSG_FRIEND_LIST and sends
    // it empty; TBC renamed it CMSG_CONTACT_LIST and gave it a flags word.
    // Checked against the opcode maps: classic names only the first, TBC and
    // WotLK only the second.
    if (isClassicLikeExpansion()) {
        const uint32_t wire = wireOpcode(Opcode::CMSG_FRIEND_LIST);
        if (wire == 0xFFFF) return;
        network::Packet pkt(static_cast<uint16_t>(wire));
        owner_.getSocket()->send(pkt);
        return;
    }
    const uint32_t wire = wireOpcode(Opcode::CMSG_CONTACT_LIST);
    if (wire == 0xFFFF) return;
    network::Packet pkt(static_cast<uint16_t>(wire));
    // Which lists to send back. AzerothCore reads one uint32 and hands it
    // straight to SendSocialList; SOCIAL_FLAG_ALL is friend, ignored and muted
    // together, which is what the panel shows across its three tabs.
    constexpr uint32_t kSocialFlagAll = 0x07;
    pkt.writeUInt32(kSocialFlagAll);
    owner_.getSocket()->send(pkt);
}

void SocialHandler::requestBattlefieldPositions() {
    if (!owner_.isInWorld() || !owner_.getSocket()) return;
    // Only in one. AzerothCore drops the request outright when the player has
    // no battleground, so asking anywhere else is bytes for nothing.
    if (!owner_.isBattlegroundMap(owner_.getCurrentMapId())) return;
    // Once a second. The interface asks from WorldMapFrame_OnUpdate, so an
    // unthrottled send is one packet per frame for as long as the map is open.
    const auto now = std::chrono::steady_clock::now();
    if (now - lastBgPositionRequest_ < std::chrono::seconds(1)) return;
    lastBgPositionRequest_ = now;
    network::Packet pkt(wireOpcode(Opcode::MSG_BATTLEGROUND_PLAYER_POSITIONS));
    owner_.getSocket()->send(pkt);
}

// MSG_GUILD_EVENT_LOG_QUERY: ask, with no payload. The reply is the same
// opcode and is parsed below.
void SocialHandler::requestGuildBankLog(uint8_t tab) {
    if (!owner_.isInWorld() || !owner_.getSocket()) return;
    network::Packet pkt(wireOpcode(Opcode::MSG_GUILD_BANK_LOG_QUERY));
    pkt.writeUInt8(tab);
    owner_.getSocket()->send(pkt);
    LOG_INFO("MSG_GUILD_BANK_LOG_QUERY: tab ", static_cast<int>(tab));
}

// The reply, which shares its opcode with the request.
//
//   uint8 tab
//   uint8 count
//   per entry:
//     int8   type
//     uint64 playerGuid
//     the payload the type calls for - an item and a count, those plus a
//     destination tab for a move, or a sum of money for everything else
//     uint32 secondsAgo
//
// Nothing sent the query, so nothing ever arrived to parse: the guild bank's
// log tab counted zero transactions and drew an empty page over a log the
// server keeps.
void SocialHandler::handleGuildBankLog(network::Packet& packet) {
    if (!packet.hasRemaining(2)) { packet.skipAll(); return; }
    const uint8_t tab = packet.readUInt8();
    const uint8_t count = packet.readUInt8();
    if (tab >= guildBankLogs_.size()) { packet.skipAll(); return; }

    auto& log = guildBankLogs_[tab];
    log.clear();
    log.reserve(count);
    for (uint8_t i = 0; i < count; ++i) {
        if (!packet.hasRemaining(9)) break;
        GuildBankLogEntry e;
        e.type = packet.readUInt8();
        e.playerGuid = packet.readUInt64();
        switch (e.type) {
            case 1: case 2:                 // deposit / withdraw an item
                if (!packet.hasRemaining(8)) { packet.skipAll(); return; }
                e.itemId = packet.readUInt32();
                e.count  = packet.readUInt32();
                break;
            case 3: case 7:                 // moved between tabs
                if (!packet.hasRemaining(9)) { packet.skipAll(); return; }
                e.itemId   = packet.readUInt32();
                e.count    = packet.readUInt32();
                e.otherTab = packet.readUInt8();
                break;
            default:                        // money, in copper
                if (!packet.hasRemaining(4)) { packet.skipAll(); return; }
                e.count = packet.readUInt32();
                break;
        }
        if (!packet.hasRemaining(4)) break;
        e.secondsAgo = packet.readUInt32();
        if (e.itemId) owner_.ensureItemInfo(e.itemId);
        log.push_back(e);
    }
    packet.skipAll();
    LOG_INFO("MSG_GUILD_BANK_LOG_QUERY reply: tab ", static_cast<int>(tab),
             ", ", log.size(), " entries");
    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("GUILDBANKLOG_UPDATE", {});
    }
}

void SocialHandler::requestGuildEventLog() {
    if (!owner_.isInWorld()) return;
    network::Packet pkt(wireOpcode(Opcode::MSG_GUILD_EVENT_LOG_QUERY));
    owner_.getSocket()->send(pkt);
}

// The reply. Two of the five fields are conditional, which is the whole
// difficulty: a reader that always takes the second guid loses eight bytes on
// every join and leave, and those are most of a guild log.
//
// Types, from AzerothCore's GuildEventLogTypes: 1 invite, 2 join, 3 promote,
// 4 demote, 5 remove, 6 leave.
void SocialHandler::handleGuildEventLog(network::Packet& packet) {
    auto rem = [&]() { return packet.getRemainingSize(); };
    if (rem() < 1) return;
    const uint8_t count = packet.readUInt8();
    guildEventLog_.clear();
    guildEventLog_.reserve(count);
    for (uint8_t i = 0; i < count; ++i) {
        if (rem() < 9) break;                 // type + one guid at minimum
        GuildEventLogEntry e;
        e.type = packet.readUInt8();
        e.playerGuid = packet.readUInt64();
        // Everything but join (2) and leave (6) names a second player.
        if (e.type != 2 && e.type != 6) {
            if (rem() < 8) break;
            e.otherGuid = packet.readUInt64();
        }
        // Promote (3) and demote (4) carry the new rank.
        if (e.type == 3 || e.type == 4) {
            if (rem() < 1) break;
            e.newRank = packet.readUInt8();
        }
        if (rem() < 4) break;
        e.secondsAgo = packet.readUInt32();
        guildEventLog_.push_back(e);
    }
    LOG_INFO("Guild event log: ", guildEventLog_.size(), " entries");
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("GUILD_EVENT_LOG_UPDATE", {});
}

// CMSG_GMTICKET_SYSTEMSTATUS: is the ticket queue open at all. The reply,
// SMSG_GMTICKET_SYSTEMSTATUS, was already parsed - asking was the missing half,
// and the help frame asks every time it is shown.
void SocialHandler::requestGmSystemStatus() {
    if (!owner_.isInWorld()) return;
    network::Packet pkt(wireOpcode(Opcode::CMSG_GMTICKET_SYSTEMSTATUS));
    owner_.getSocket()->send(pkt);
}

void SocialHandler::requestGmTicket() {
    if (!owner_.isInWorld()) return;
    // CMSG_GMTICKET_GETTICKET has no payload - server responds with SMSG_GMTICKET_GETTICKET
    network::Packet pkt(wireOpcode(Opcode::CMSG_GMTICKET_GETTICKET));
    owner_.getSocket()->send(pkt);
    LOG_DEBUG("Sent CMSG_GMTICKET_GETTICKET - querying open ticket status");
}

} // namespace game
} // namespace wowee
