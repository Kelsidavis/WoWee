#include "game/quest_handler.hpp"
#include "game/game_handler.hpp"
#include "game/game_utils.hpp"
#include "game/entity.hpp"
#include "game/update_field_table.hpp"
#include "game/quest_progress.hpp"
#include "game/packet_parsers.hpp"
#include "network/world_socket.hpp"
#include "rendering/renderer.hpp"
#include "audio/audio_coordinator.hpp"
#include "audio/ui_sound_manager.hpp"
#include "pipeline/asset_manager.hpp"
#include "core/application.hpp"
#include "core/logger.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <limits>
#include <sstream>

namespace wowee {
namespace game {

QuestGiverStatus QuestHandler::getQuestGiverStatus(uint64_t guid) const {
    auto it = npcQuestStatus_.find(guid);
    return (it != npcQuestStatus_.end()) ? it->second : QuestGiverStatus::NONE;
}

void QuestHandler::reconcileItemObjectivesFromInventory(
    const std::unordered_map<uint32_t, uint32_t>& carriedCounts) {
    bool changedAny = false;
    bool maybeCompletedObjective = false;
    for (auto& quest : questLog_) {
        if (quest.complete) continue;
        for (const auto& obj : quest.itemObjectives) {
            if (obj.itemId == 0 || obj.required == 0) continue;
            // Keep the derived required-count map in sync so the
            // SMSG_QUESTUPDATE_ADD_ITEM path and this one agree.
            quest.requiredItemCounts[obj.itemId] = obj.required;

            auto cit = carriedCounts.find(obj.itemId);
            const uint32_t held = (cit != carriedCounts.end()) ? cit->second : 0;
            const uint32_t newCount = std::min(held, obj.required);
            uint32_t& tracked = quest.itemCounts[obj.itemId];
            if (tracked == newCount) continue;

            const bool wasComplete = tracked >= obj.required;
            tracked = newCount;
            changedAny = true;
            if (!wasComplete && newCount >= obj.required)
                maybeCompletedObjective = true;

            // Push the per-objective progress to the on-screen tracker.
            std::string itemLabel = "item #" + std::to_string(obj.itemId);
            if (const ItemQueryResponseData* info = owner_.getItemInfo(obj.itemId)) {
                if (!info->name.empty()) itemLabel = info->name;
            } else {
                // Name not cached yet — request it so the next refresh labels
                // the objective properly.
                owner_.queryItemInfo(obj.itemId, 0);
            }
            if (owner_.questProgressCallbackRef())
                owner_.questProgressCallbackRef()(quest.title, itemLabel, newCount, obj.required);
        }
    }

    if (changedAny && owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("QUEST_WATCH_UPDATE", {});
        owner_.addonEventCallbackRef()("QUEST_LOG_UPDATE", {});
        owner_.addonEventCallbackRef()("UNIT_QUEST_LOG_CHANGED", {"player"});
    }
    // Collecting the last item may have made a quest turn-in-able (! → ?),
    // so refresh nearby giver markers immediately.
    if (maybeCompletedObjective) requeryNearbyQuestGiverStatus();
}

void QuestHandler::requeryNearbyQuestGiverStatus() {
    if (questGiverRequeryCooldown_ > 0.0f) {
        questGiverRequeryPending_ = true;
        return;
    }
    sendQuestGiverStatusQueries();
    questGiverRequeryCooldown_ = kQuestGiverRequeryIntervalSec;
}

void QuestHandler::tickQuestGiverStatusRequery(float deltaTime) {
    if (questGiverRequeryCooldown_ <= 0.0f) return;
    questGiverRequeryCooldown_ -= deltaTime;
    if (questGiverRequeryCooldown_ > 0.0f) return;

    questGiverRequeryCooldown_ = 0.0f;
    if (questGiverRequeryPending_) {
        questGiverRequeryPending_ = false;
        sendQuestGiverStatusQueries();
        questGiverRequeryCooldown_ = kQuestGiverRequeryIntervalSec;
    }
}

void QuestHandler::sendQuestGiverStatusQueries() {
    if (!owner_.getSocket()) return;
    for (const auto& [guid, entity] : owner_.getEntityManager().getEntities()) {
        if (entity->getType() != ObjectType::UNIT) continue;
        auto unit = std::static_pointer_cast<Unit>(entity);
        if (unit->getNpcFlags() & 0x02) { // UNIT_NPC_FLAG_QUESTGIVER
            network::Packet qsPkt(wireOpcode(Opcode::CMSG_QUESTGIVER_STATUS_QUERY));
            qsPkt.writeUInt64(guid);
            owner_.getSocket()->send(qsPkt);
        }
    }
}


static std::string formatCopperAmount(uint32_t amount) {
    uint32_t gold = amount / 10000;
    uint32_t silver = (amount / 100) % 100;
    uint32_t copper = amount % 100;

    std::ostringstream oss;
    bool wrote = false;
    if (gold > 0) {
        oss << gold << "g";
        wrote = true;
    }
    if (silver > 0) {
        if (wrote) oss << " ";
        oss << silver << "s";
        wrote = true;
    }
    if (copper > 0 || !wrote) {
        if (wrote) oss << " ";
        oss << copper << "c";
    }
    return oss.str();
}

static bool isReadableQuestText(const std::string& s, size_t minLen, size_t maxLen) {
    if (s.size() < minLen || s.size() > maxLen) return false;
    bool hasAlpha = false;
    for (unsigned char c : s) {
        // Reject control characters but allow UTF-8 multi-byte sequences (0x80+)
        // so localized servers (French, German, Russian, etc.) work correctly.
        if (c < 0x20 && c != '\t' && c != '\n' && c != '\r') return false;
        if (c >= 0x20 && c <= 0x7E && std::isalpha(c)) hasAlpha = true;
        // UTF-8 continuation/lead bytes (0x80+) are allowed but don't count as alpha
        // since we only need at least one ASCII letter to distinguish from binary garbage.
    }
    return hasAlpha;
}

static bool isPlaceholderQuestTitle(const std::string& s) {
    return s.rfind("Quest #", 0) == 0;
}

static bool looksLikeQuestDescriptionText(const std::string& s) {
    int spaces = 0;
    int commas = 0;
    for (unsigned char c : s) {
        if (c == ' ') spaces++;
        if (c == ',') commas++;
    }
    const int words = spaces + 1;
    if (words > 8) return true;
    if (commas > 0 && words > 5) return true;
    if (s.find(". ") != std::string::npos) return true;
    if (s.find(':') != std::string::npos && words > 5) return true;
    return false;
}

static bool isStrongQuestTitle(const std::string& s) {
    if (!isReadableQuestText(s, 6, 72)) return false;
    if (looksLikeQuestDescriptionText(s)) return false;
    unsigned char first = static_cast<unsigned char>(s.front());
    return std::isupper(first) != 0;
}

static int scoreQuestTitle(const std::string& s) {
    if (!isReadableQuestText(s, 4, 72)) return -1000;
    if (looksLikeQuestDescriptionText(s)) return -1000;
    int score = 0;
    score += static_cast<int>(std::min<size_t>(s.size(), 32));
    unsigned char first = static_cast<unsigned char>(s.front());
    if (std::isupper(first)) score += 20;
    if (std::islower(first)) score -= 20;
    if (s.find(' ') != std::string::npos) score += 8;
    if (s.find('.') != std::string::npos) score -= 18;
    if (s.find('!') != std::string::npos || s.find('?') != std::string::npos) score -= 6;
    return score;
}

static bool readCStringAt(const std::vector<uint8_t>& data, size_t start, std::string& out, size_t& nextPos) {
    out.clear();
    if (start >= data.size()) return false;
    size_t i = start;
    while (i < data.size()) {
        uint8_t b = data[i++];
        if (b == 0) {
            nextPos = i;
            return true;
        }
        out.push_back(static_cast<char>(b));
    }
    return false;
}

struct QuestQueryTextCandidate {
    std::string title;
    std::string objectives;
    int score = -1000;
};

// Expand WoW text tokens ($b line breaks etc.) in quest strings; titles are
// single-line so their line breaks collapse to spaces instead
static std::string normalizeQuestText(std::string s, bool singleLine) {
    s = normalizeWowTextTokens(std::move(s));
    if (singleLine) {
        std::string out;
        out.reserve(s.size());
        bool lastSpace = false;
        for (char ch : s) {
            if (ch == '\n' || ch == '\r' || ch == ' ') {
                if (!lastSpace && !out.empty()) out.push_back(' ');
                lastSpace = true;
            } else {
                out.push_back(ch);
                lastSpace = false;
            }
        }
        while (!out.empty() && out.back() == ' ') out.pop_back();
        s = std::move(out);
    }
    return s;
}

static QuestQueryTextCandidate pickBestQuestQueryTexts(const std::vector<uint8_t>& data, bool classicHint) {
    QuestQueryTextCandidate best;
    if (data.size() <= 9) return best;

    std::vector<size_t> seedOffsets;
    const size_t base = 8;
    const size_t classicOffset = base + 40u * 4u;
    const size_t wotlkOffset = base + 55u * 4u;
    if (classicHint) {
        seedOffsets.push_back(classicOffset);
        seedOffsets.push_back(wotlkOffset);
    } else {
        seedOffsets.push_back(wotlkOffset);
        seedOffsets.push_back(classicOffset);
    }
    for (size_t off : seedOffsets) {
        if (off < data.size()) {
            std::string title;
            size_t next = off;
            if (readCStringAt(data, off, title, next)) {
                QuestQueryTextCandidate c;
                c.title = normalizeQuestText(title, true);
                c.score = scoreQuestTitle(c.title) + 20; // Prefer expected struct offsets

                std::string s2;
                size_t n2 = next;
                if (readCStringAt(data, next, s2, n2) && isReadableQuestText(s2, 8, 600)) {
                    c.objectives = normalizeQuestText(s2, false);
                }
                if (c.score > best.score) best = c;
            }
        }
    }

    // Fallback: scan packet for best printable C-string title candidate.
    for (size_t start = 8; start < data.size(); ++start) {
        std::string title;
        size_t next = start;
        if (!readCStringAt(data, start, title, next)) continue;

        QuestQueryTextCandidate c;
        c.title = normalizeQuestText(title, true);
        c.score = scoreQuestTitle(c.title);
        if (c.score < 0) continue;

        std::string s2, s3;
        size_t n2 = next, n3 = next;
        if (readCStringAt(data, next, s2, n2)) {
            if (isReadableQuestText(s2, 8, 600)) c.objectives = normalizeQuestText(s2, false);
            else if (readCStringAt(data, n2, s3, n3) && isReadableQuestText(s3, 8, 600)) c.objectives = normalizeQuestText(s3, false);
        }
        if (c.score > best.score) best = c;
    }

    return best;
}

struct QuestQueryObjectives {
    struct Kill { int32_t npcOrGoId; uint32_t required; };
    struct Item { uint32_t itemId; uint32_t required; };
    std::array<Kill, 4> kills{};
    std::array<Item, 6> items{};
    bool valid = false;
};

static uint32_t readU32At(const std::vector<uint8_t>& d, size_t pos) {
    return static_cast<uint32_t>(d[pos])
         | (static_cast<uint32_t>(d[pos + 1]) << 8)
         | (static_cast<uint32_t>(d[pos + 2]) << 16)
         | (static_cast<uint32_t>(d[pos + 3]) << 24);
}

static int32_t decodeQuestNpcOrGo(uint32_t raw) {
    if ((raw & 0x80000000u) != 0)
        return -static_cast<int32_t>(raw & 0x7FFFFFFFu);
    return static_cast<int32_t>(raw);
}

static bool plausibleQuestObjective(int32_t id, uint32_t required) {
    const uint32_t magnitude = id < 0 ? static_cast<uint32_t>(-static_cast<int64_t>(id))
                                      : static_cast<uint32_t>(id);
    return magnitude <= 0x00FFFFFFu && required <= 0x0000FFFFu;
}

static QuestQueryObjectives tryParseQuestObjectivesAt(const std::vector<uint8_t>& data,
                                                       size_t startPos, int nStrings,
                                                       bool wotlkLayout) {
    QuestQueryObjectives out;
    size_t pos = startPos;

    for (int si = 0; si < nStrings; ++si) {
        while (pos < data.size() && data[pos] != 0) ++pos;
        if (pos >= data.size()) return out;
        ++pos;
    }

    if (wotlkLayout) {
        // Wrath: four {npc/go, count, sourceItem, sourceCount} records,
        // followed by six {requiredItem, count} records.
        for (int i = 0; i < 4; ++i) {
            if (pos + 16 > data.size()) return out;
            out.kills[i].npcOrGoId = decodeQuestNpcOrGo(readU32At(data, pos)); pos += 4;
            out.kills[i].required  = readU32At(data, pos);                     pos += 4;
            pos += 8; // ItemDrop + source count (not inventory objectives)
            if (!plausibleQuestObjective(out.kills[i].npcOrGoId, out.kills[i].required))
                return {};
        }
        for (int i = 0; i < 6; ++i) {
            if (pos + 8 > data.size()) return out;
            out.items[i].itemId   = readU32At(data, pos); pos += 4;
            out.items[i].required = readU32At(data, pos); pos += 4;
            if (!plausibleQuestObjective(static_cast<int32_t>(out.items[i].itemId),
                                         out.items[i].required))
                return {};
        }
    } else {
        // Classic/TBC: four interleaved {npc/go, count, requiredItem, count} records.
        for (int i = 0; i < 4; ++i) {
            if (pos + 16 > data.size()) return out;
            out.kills[i].npcOrGoId = decodeQuestNpcOrGo(readU32At(data, pos)); pos += 4;
            out.kills[i].required  = readU32At(data, pos);                     pos += 4;
            out.items[i].itemId    = readU32At(data, pos);                     pos += 4;
            out.items[i].required  = readU32At(data, pos);                     pos += 4;
            if (!plausibleQuestObjective(out.kills[i].npcOrGoId, out.kills[i].required) ||
                !plausibleQuestObjective(static_cast<int32_t>(out.items[i].itemId),
                                         out.items[i].required))
                return {};
        }
    }

    out.valid = true;
    return out;
}

static QuestQueryObjectives extractQuestQueryObjectives(const std::vector<uint8_t>& data,
                                                         uint8_t questLogStride) {
    if (data.size() < 16) return {};

    const size_t base = 8;
    // Fixed-width fields between questMethod and the first localized string.
    // These counts come directly from each expansion's query response serializer.
    if (questLogStride >= 5)
        return tryParseQuestObjectivesAt(data, base + 63u * 4u, 5, true);  // WotLK
    if (questLogStride == 4)
        return tryParseQuestObjectivesAt(data, base + 41u * 4u, 4, false); // TBC
    return tryParseQuestObjectivesAt(data, base + 37u * 4u, 4, false);     // Classic/Turtle
}

// Quest log reward extraction lives in QuestQueryRewardsParser
// (world_packets_world.cpp) so the per-expansion offsets are unit-testable.

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

QuestHandler::QuestHandler(GameHandler& owner)
    : owner_(owner) {}

// ---------------------------------------------------------------------------
// Opcode registrations
// ---------------------------------------------------------------------------

void QuestHandler::registerOpcodes(DispatchTable& table) {

    // ---- SMSG_GOSSIP_MESSAGE ----
    table[Opcode::SMSG_GOSSIP_MESSAGE] = [this](network::Packet& packet) { handleGossipMessage(packet); };

    // ---- SMSG_QUESTGIVER_QUEST_LIST ----
    table[Opcode::SMSG_QUESTGIVER_QUEST_LIST] = [this](network::Packet& packet) { handleQuestgiverQuestList(packet); };

    // ---- SMSG_GOSSIP_COMPLETE ----
    table[Opcode::SMSG_GOSSIP_COMPLETE] = [this](network::Packet& packet) { handleGossipComplete(packet); };

    // ---- SMSG_NPC_TEXT_UPDATE ----
    table[Opcode::SMSG_NPC_TEXT_UPDATE] = [this](network::Packet& packet) { handleNpcTextUpdate(packet); };

    // ---- SMSG_GOSSIP_POI ----
    table[Opcode::SMSG_GOSSIP_POI] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(20)) return;
        /*uint32_t flags =*/ packet.readUInt32();
        float poiX = packet.readFloat();
        float poiY = packet.readFloat();
        uint32_t icon = packet.readUInt32();
        uint32_t data = packet.readUInt32();
        std::string name = packet.readString();
        GossipPoi poi; poi.x = poiX; poi.y = poiY; poi.icon = icon; poi.data = data; poi.name = std::move(name);
        if (gossipPois_.size() >= 200) gossipPois_.erase(gossipPois_.begin());
        gossipPois_.push_back(std::move(poi));
        LOG_DEBUG("SMSG_GOSSIP_POI: x=", poiX, " y=", poiY, " icon=", icon);
    };

    // ---- SMSG_QUESTGIVER_QUEST_DETAILS ----
    table[Opcode::SMSG_QUESTGIVER_QUEST_DETAILS] = [this](network::Packet& packet) { handleQuestDetails(packet); };

    // ---- SMSG_QUESTLOG_FULL ----
    table[Opcode::SMSG_QUESTLOG_FULL] = [this](network::Packet& /*packet*/) {
        const std::string msg = "Your quest log is full (" +
            std::to_string(maxQuestLogSlots()) + " quests). Abandon a quest to make room.";
        owner_.addUIError(msg);
        owner_.addSystemChatMessage(msg);
        // Roll back the optimistic local entries the server refused; without
        // this the local log drifts past the server cap with phantom quests.
        bool removed = false;
        for (const auto& [pendingQuestId, npcGuid] : pendingQuestAcceptNpcGuids_) {
            (void)npcGuid;
            if (findQuestLogSlotIndexFromServer(pendingQuestId) < 0) {
                removed |= std::erase_if(questLog_, [&](const QuestLogEntry& q) {
                    return q.questId == pendingQuestId;
                }) > 0;
            }
        }
        pendingQuestAcceptTimeouts_.clear();
        pendingQuestAcceptNpcGuids_.clear();
        if (removed && owner_.addonEventCallbackRef())
            owner_.addonEventCallbackRef()("QUEST_LOG_UPDATE", {});
    };

    // ---- SMSG_QUESTGIVER_REQUEST_ITEMS ----
    table[Opcode::SMSG_QUESTGIVER_REQUEST_ITEMS] = [this](network::Packet& packet) { handleQuestRequestItems(packet); };

    // ---- SMSG_QUESTGIVER_OFFER_REWARD ----
    table[Opcode::SMSG_QUESTGIVER_OFFER_REWARD] = [this](network::Packet& packet) { handleQuestOfferReward(packet); };

    // ---- SMSG_QUEST_CONFIRM_ACCEPT ----
    table[Opcode::SMSG_QUEST_CONFIRM_ACCEPT] = [this](network::Packet& packet) { handleQuestConfirmAccept(packet); };

    // ---- SMSG_QUEST_POI_QUERY_RESPONSE ----
    table[Opcode::SMSG_QUEST_POI_QUERY_RESPONSE] = [this](network::Packet& packet) { handleQuestPoiQueryResponse(packet); };

    // ---- SMSG_QUESTGIVER_STATUS ----
    table[Opcode::SMSG_QUESTGIVER_STATUS] = [this](network::Packet& packet) {
        if (packet.hasRemaining(9)) {
            uint64_t npcGuid = packet.readUInt64();
            uint8_t status = owner_.getPacketParsers()->readQuestGiverStatus(packet);
            LOG_INFO("SMSG_QUESTGIVER_STATUS: npcGuid=0x", std::hex, npcGuid, std::dec,
                     " status=", static_cast<int>(status));
            npcQuestStatus_[npcGuid] = static_cast<QuestGiverStatus>(status);
        }
    };

    // ---- SMSG_QUESTGIVER_STATUS_MULTIPLE ----
    table[Opcode::SMSG_QUESTGIVER_STATUS_MULTIPLE] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(4)) return;
        uint32_t count = packet.readUInt32();
        for (uint32_t i = 0; i < count; ++i) {
            if (!packet.hasRemaining(9)) break;
            uint64_t npcGuid = packet.readUInt64();
            uint8_t status = owner_.getPacketParsers()->readQuestGiverStatus(packet);
            npcQuestStatus_[npcGuid] = static_cast<QuestGiverStatus>(status);
        }
    };

    // ---- SMSG_QUESTUPDATE_FAILED ----
    table[Opcode::SMSG_QUESTUPDATE_FAILED] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t questId = packet.readUInt32();
            std::string questTitle;
            for (const auto& q : questLog_)
                if (q.questId == questId && !q.title.empty()) { questTitle = q.title; break; }
            owner_.addSystemChatMessage(questTitle.empty() ? std::string("Quest failed!")
                                                           : ('"' + questTitle + "\" failed!"));
        }
    };

    // ---- SMSG_QUESTUPDATE_FAILEDTIMER ----
    table[Opcode::SMSG_QUESTUPDATE_FAILEDTIMER] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t questId = packet.readUInt32();
            std::string questTitle;
            for (const auto& q : questLog_)
                if (q.questId == questId && !q.title.empty()) { questTitle = q.title; break; }
            owner_.addSystemChatMessage(questTitle.empty() ? std::string("Quest timed out!")
                                                           : ('"' + questTitle + "\" has timed out."));
        }
    };

    // ---- SMSG_QUESTGIVER_QUEST_FAILED ----
    table[Opcode::SMSG_QUESTGIVER_QUEST_FAILED] = [this](network::Packet& packet) {
        // uint32 questId + uint32 reason
        if (packet.hasRemaining(8)) {
            uint32_t questId = packet.readUInt32();
            uint32_t reason = packet.readUInt32();
            std::string questTitle;
            for (const auto& q : questLog_)
                if (q.questId == questId && !q.title.empty()) { questTitle = q.title; break; }
            const char* reasonStr = nullptr;
            switch (reason) {
                case 1: reasonStr = "failed conditions"; break;
                case 2: reasonStr = "inventory full"; break;
                case 3: reasonStr = "too far away"; break;
                case 4: reasonStr = "another quest is blocking"; break;
                case 5: reasonStr = "wrong time of day"; break;
                case 6: reasonStr = "wrong race"; break;
                case 7: reasonStr = "wrong class"; break;
            }
            std::string msg = questTitle.empty() ? "Quest" : ('"' + questTitle + '"');
            msg += " failed";
            if (reasonStr) msg += std::string(": ") + reasonStr;
            msg += '.';
            owner_.addSystemChatMessage(msg);
        }
    };

    // ---- SMSG_QUESTGIVER_QUEST_INVALID ----
    table[Opcode::SMSG_QUESTGIVER_QUEST_INVALID] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t failReason = packet.readUInt32();
            pendingTurnInRewardRequest_ = false;
            const char* reasonStr = "Unknown";
            switch (failReason) {
                case 0: reasonStr = "Don't have quest"; break;
                case 1: reasonStr = "Quest level too low"; break;
                case 4: reasonStr = "Insufficient money"; break;
                case 5: reasonStr = "Inventory full"; break;
                case 13: reasonStr = "Already on that quest"; break;
                case 18: reasonStr = "Already completed quest"; break;
                case 19: reasonStr = "Can't take any more quests"; break;
            }
            LOG_WARNING("Quest invalid: reason=", failReason, " (", reasonStr, ")");
            if (!pendingQuestAcceptTimeouts_.empty()) {
                std::vector<uint32_t> pendingQuestIds;
                pendingQuestIds.reserve(pendingQuestAcceptTimeouts_.size());
                for (const auto& pending : pendingQuestAcceptTimeouts_) {
                    pendingQuestIds.push_back(pending.first);
                }
                for (uint32_t questId : pendingQuestIds) {
                    const uint64_t npcGuid = pendingQuestAcceptNpcGuids_.count(questId) != 0
                        ? pendingQuestAcceptNpcGuids_[questId] : 0;
                    if (failReason == 13) {
                        std::string fallbackTitle = "Quest #" + std::to_string(questId);
                        std::string fallbackObjectives;
                        if (currentQuestDetails_.questId == questId) {
                            if (!currentQuestDetails_.title.empty()) fallbackTitle = currentQuestDetails_.title;
                            fallbackObjectives = currentQuestDetails_.objectives;
                        }
                        addQuestToLocalLogIfMissing(questId, fallbackTitle, fallbackObjectives);
                        triggerQuestAcceptResync(questId, npcGuid, "already-on-quest");
                    } else if (failReason == 18) {
                        triggerQuestAcceptResync(questId, npcGuid, "already-completed");
                    }
                    clearPendingQuestAccept(questId);
                }
            }
            // Only show error to user for real errors (not informational messages)
            if (failReason != 13 && failReason != 18) {  // Don't spam "already on/completed"
                owner_.addSystemChatMessage(std::string("Quest unavailable: ") + reasonStr);
            }
        }
    };

    // ---- SMSG_QUESTGIVER_QUEST_COMPLETE ----
    table[Opcode::SMSG_QUESTGIVER_QUEST_COMPLETE] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t questId = packet.readUInt32();
            LOG_INFO("Quest completed: questId=", questId);
            if (pendingTurnInQuestId_ == questId) {
                pendingTurnInQuestId_ = 0;
                pendingTurnInNpcGuid_ = 0;
                pendingTurnInRewardRequest_ = false;
            }
            for (auto it = questLog_.begin(); it != questLog_.end(); ++it) {
                if (it->questId == questId) {
                    // Fire toast callback before erasing
                    if (owner_.questCompleteCallbackRef()) {
                        owner_.questCompleteCallbackRef()(questId, it->title);
                    }
                    // Play quest-complete sound
                    if (auto* ac = owner_.services().audioCoordinator) {
                        if (auto* sfx = ac->getUiSoundManager())
                            sfx->playQuestComplete();
                    }
                    questLog_.erase(it);
                    owner_.setQuestTracked(questId, false);
                    owner_.setQuestShownOnMap(questId, false);
                    LOG_INFO("  Removed quest ", questId, " from quest log");
                    if (owner_.addonEventCallbackRef())
                        owner_.addonEventCallbackRef()("QUEST_TURNED_IN", {std::to_string(questId)});
                    break;
                }
            }
        }
        if (owner_.addonEventCallbackRef()) {
            owner_.addonEventCallbackRef()("QUEST_LOG_UPDATE", {});
            owner_.addonEventCallbackRef()("UNIT_QUEST_LOG_CHANGED", {"player"});
        }
        // Re-query all nearby quest giver NPCs so markers refresh
        requeryNearbyQuestGiverStatus();
    };

    // ---- SMSG_QUESTUPDATE_ADD_KILL ----
    table[Opcode::SMSG_QUESTUPDATE_ADD_KILL] = [this](network::Packet& packet) {
        size_t rem = packet.getRemainingSize();
        if (rem >= 12) {
            uint32_t questId = packet.readUInt32();
            clearPendingQuestAccept(questId);
            uint32_t entry = normalizeQuestObjectiveEntry(packet.readUInt32());
            uint32_t count = packet.readUInt32();
            uint32_t reqCount = 0;
            if (packet.hasRemaining(4)) {
                reqCount = packet.readUInt32();
            }

            LOG_INFO("Quest kill update: questId=", questId, " entry=", entry,
                     " count=", count, "/", reqCount);

            for (auto& quest : questLog_) {
                if (quest.questId == questId) {
                    if (reqCount == 0) {
                        auto it = quest.killCounts.find(entry);
                        if (it != quest.killCounts.end()) reqCount = it->second.second;
                    }
                    // Fall back to killObjectives (parsed from SMSG_QUEST_QUERY_RESPONSE).
                    if (reqCount == 0) {
                        for (const auto& obj : quest.killObjectives) {
                            if (obj.npcOrGoId == 0 || obj.required == 0) continue;
                            uint32_t objEntry = static_cast<uint32_t>(
                                obj.npcOrGoId > 0 ? obj.npcOrGoId : -obj.npcOrGoId);
                            if (objEntry == entry) {
                                reqCount = obj.required;
                                break;
                            }
                        }
                    }
                    // Some quests (e.g. escort/event quests) report kill credit updates without
                    // a corresponding objective count in SMSG_QUEST_QUERY_RESPONSE. Fall back to
                    // current count so the progress display shows "N/N" instead of "N/0".
                    if (reqCount == 0) reqCount = count;
                    count = std::min(count, reqCount);
                    quest.killCounts[entry] = {count, reqCount};

                    std::string creatureName = owner_.getCachedCreatureName(entry);
                    std::string progressMsg = quest.title + ": ";
                    if (!creatureName.empty()) {
                        progressMsg += creatureName + " ";
                    }
                    progressMsg += std::to_string(count) + "/" + std::to_string(reqCount);
                    owner_.addSystemChatMessage(progressMsg);

                    if (owner_.questProgressCallbackRef()) {
                        owner_.questProgressCallbackRef()(quest.title, creatureName, count, reqCount);
                    }
                    if (owner_.addonEventCallbackRef()) {
                        owner_.addonEventCallbackRef()("QUEST_WATCH_UPDATE", {std::to_string(questId)});
                        owner_.addonEventCallbackRef()("QUEST_LOG_UPDATE", {});
                        owner_.addonEventCallbackRef()("UNIT_QUEST_LOG_CHANGED", {"player"});
                    }

                    LOG_INFO("Updated kill count for quest ", questId, ": ",
                             count, "/", reqCount);
                    // When this objective just finished, a quest may have become
                    // turn-in-able — refresh nearby giver markers (! → ?) now
                    // instead of only after leaving and re-entering the area.
                    if (reqCount > 0 && count >= reqCount) requeryNearbyQuestGiverStatus();
                    break;
                }
            }
        } else if (rem >= 4) {
            // Swapped mapping fallback: treat as QUESTUPDATE_COMPLETE packet.
            uint32_t questId = packet.readUInt32();
            clearPendingQuestAccept(questId);
            LOG_INFO("Quest objectives completed (compat via ADD_KILL): questId=", questId);
            for (auto& quest : questLog_) {
                if (quest.questId == questId) {
                    quest.complete = true;
                    owner_.addSystemChatMessage("Quest Complete: " + quest.title);
                    break;
                }
            }
            requeryNearbyQuestGiverStatus();
        }
    };

    // ---- SMSG_QUESTUPDATE_ADD_ITEM ----
    table[Opcode::SMSG_QUESTUPDATE_ADD_ITEM] = [this](network::Packet& packet) {
        if (packet.hasRemaining(8)) {
            uint32_t itemId = packet.readUInt32();
            uint32_t count = packet.readUInt32();
            owner_.queryItemInfo(itemId, 0);

            std::string itemLabel = "item #" + std::to_string(itemId);
            uint32_t questItemQuality = 1;
            if (const ItemQueryResponseData* info = owner_.getItemInfo(itemId)) {
                if (!info->name.empty()) itemLabel = info->name;
                questItemQuality = info->quality;
            }

            bool updatedAny = false;
            bool maybeCompletedObjective = false;
            for (auto& quest : questLog_) {
                if (quest.complete) continue;
                bool tracksItem =
                    quest.requiredItemCounts.count(itemId) > 0 ||
                    quest.itemCounts.count(itemId) > 0;
                // Also check itemObjectives parsed from SMSG_QUEST_QUERY_RESPONSE in case
                // requiredItemCounts hasn't been populated yet (race during quest accept).
                if (!tracksItem) {
                    for (const auto& obj : quest.itemObjectives) {
                        if (obj.itemId == itemId && obj.required > 0) {
                            quest.requiredItemCounts.emplace(itemId, obj.required);
                            tracksItem = true;
                            break;
                        }
                    }
                }
                if (!tracksItem) continue;
                // SMSG_QUESTUPDATE_ADD_ITEM carries the amount added by this
                // loot operation, not the new absolute objective count.
                const uint32_t required = quest.requiredItemCounts.count(itemId) != 0
                    ? quest.requiredItemCounts[itemId] : std::numeric_limits<uint32_t>::max();
                const uint32_t current = quest.itemCounts[itemId];
                quest.itemCounts[itemId] = current >= required - std::min(required, count)
                    ? required
                    : current + count;
                updatedAny = true;
                // An unknown requirement (no SMSG_QUEST_QUERY_RESPONSE yet) can't
                // be compared against, so assume it may have just filled rather
                // than leaving those quests without a marker refresh. The requery
                // is rate-limited, so guessing wide costs nothing.
                const bool requiredKnown = required != std::numeric_limits<uint32_t>::max();
                if (!requiredKnown || quest.itemCounts[itemId] >= required) {
                    maybeCompletedObjective = true;
                }
            }
            owner_.addSystemChatMessage("Quest item: " + buildItemLink(itemId, questItemQuality, itemLabel) + " (" + std::to_string(count) + ")");

            if (owner_.questProgressCallbackRef() && updatedAny) {
                for (const auto& quest : questLog_) {
                    if (quest.complete) continue;
                    if (quest.itemCounts.count(itemId) == 0) continue;
                    uint32_t required = 0;
                    auto rIt = quest.requiredItemCounts.find(itemId);
                    if (rIt != quest.requiredItemCounts.end()) required = rIt->second;
                    if (required == 0) {
                        for (const auto& obj : quest.itemObjectives) {
                            if (obj.itemId == itemId) { required = obj.required; break; }
                        }
                    }
                    if (required == 0) required = count;
                    owner_.questProgressCallbackRef()(quest.title, itemLabel, count, required);
                    break;
                }
            }

            if (owner_.addonEventCallbackRef() && updatedAny) {
                owner_.addonEventCallbackRef()("QUEST_WATCH_UPDATE", {});
                owner_.addonEventCallbackRef()("QUEST_LOG_UPDATE", {});
                owner_.addonEventCallbackRef()("UNIT_QUEST_LOG_CHANGED", {"player"});
            }
            // Refresh nearby giver markers now if collecting this item may have
            // made a quest turn-in-able (! → ?), rather than on leave/return.
            if (maybeCompletedObjective) requeryNearbyQuestGiverStatus();
            LOG_INFO("Quest item update: itemId=", itemId, " count=", count,
                     " trackedQuestsUpdated=", updatedAny);
        }
    };

    // ---- SMSG_QUESTUPDATE_COMPLETE ----
    table[Opcode::SMSG_QUESTUPDATE_COMPLETE] = [this](network::Packet& packet) {
        size_t rem = packet.getRemainingSize();
        if (rem >= 12) {
            uint32_t questId = packet.readUInt32();
            clearPendingQuestAccept(questId);
            uint32_t entry = normalizeQuestObjectiveEntry(packet.readUInt32());
            uint32_t count = packet.readUInt32();
            uint32_t reqCount = 0;
            if (packet.hasRemaining(4)) reqCount = packet.readUInt32();
            if (reqCount == 0) reqCount = count;
            count = std::min(count, reqCount);
            LOG_INFO("Quest kill update (compat via COMPLETE): questId=", questId,
                     " entry=", entry, " count=", count, "/", reqCount);
            for (auto& quest : questLog_) {
                if (quest.questId == questId) {
                    quest.killCounts[entry] = {count, reqCount};
                    owner_.addSystemChatMessage(quest.title + ": " + std::to_string(count) +
                                                 "/" + std::to_string(reqCount));
                    break;
                }
            }
        } else if (rem >= 4) {
            uint32_t questId = packet.readUInt32();
            clearPendingQuestAccept(questId);
            LOG_INFO("Quest objectives completed: questId=", questId);

            for (auto& quest : questLog_) {
                if (quest.questId == questId) {
                    quest.complete = true;
                    owner_.addSystemChatMessage("Quest Complete: " + quest.title);
                    LOG_INFO("Marked quest ", questId, " as complete");
                    break;
                }
            }
        }
    };

    // ---- SMSG_QUEST_FORCE_REMOVE ----
    table[Opcode::SMSG_QUEST_FORCE_REMOVE] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(4)) {
            LOG_WARNING("SMSG_QUEST_FORCE_REMOVE/SET_REST_START too short");
            return;
        }
        uint32_t value = packet.readUInt32();

        // WotLK uses this opcode as SMSG_SET_REST_START
        if (!isClassicLikeExpansion() && !isActiveExpansion("tbc")) {
            bool nowResting = (value != 0);
            if (nowResting != owner_.isRestingRef()) {
                owner_.isRestingRef() = nowResting;
                owner_.addSystemChatMessage(owner_.isRestingRef() ? "You are now resting."
                                                              : "You are no longer resting.");
                if (owner_.addonEventCallbackRef())
                    owner_.addonEventCallbackRef()("PLAYER_UPDATE_RESTING", {});
            }
            return;
        }

        // Classic/TBC: treat as QUEST_FORCE_REMOVE (uint32 questId)
        uint32_t questId = value;
        clearPendingQuestAccept(questId);
        pendingQuestQueryIds_.erase(questId);
        if (questId == 0) {
            return;
        }

        bool removed = false;
        std::string removedTitle;
        for (auto it = questLog_.begin(); it != questLog_.end(); ++it) {
            if (it->questId == questId) {
                removedTitle = it->title;
                questLog_.erase(it);
                removed = true;
                break;
            }
        }
        if (currentQuestDetails_.questId == questId) {
            questDetailsOpen_ = false;
            questDetailsOpenTime_ = std::chrono::steady_clock::time_point{};
            currentQuestDetails_ = QuestDetailsData{};
            removed = true;
        }
        if (currentQuestRequestItems_.questId == questId) {
            questRequestItemsOpen_ = false;
            currentQuestRequestItems_ = QuestRequestItemsData{};
            removed = true;
        }
        if (currentQuestOfferReward_.questId == questId) {
            questOfferRewardOpen_ = false;
            currentQuestOfferReward_ = QuestOfferRewardData{};
            removed = true;
        }
        if (removed) {
            owner_.setQuestTracked(questId, false);
            owner_.setQuestShownOnMap(questId, false);
            if (!removedTitle.empty()) {
                owner_.addSystemChatMessage("Quest removed: " + removedTitle);
            } else {
                owner_.addSystemChatMessage("Quest removed (ID " + std::to_string(questId) + ").");
            }
            if (owner_.addonEventCallbackRef()) {
                owner_.addonEventCallbackRef()("QUEST_LOG_UPDATE", {});
                owner_.addonEventCallbackRef()("UNIT_QUEST_LOG_CHANGED", {"player"});
                owner_.addonEventCallbackRef()("QUEST_REMOVED", {std::to_string(questId)});
            }
        }
    };

    // ---- SMSG_QUEST_QUERY_RESPONSE ----
    table[Opcode::SMSG_QUEST_QUERY_RESPONSE] = [this](network::Packet& packet) {
        if (packet.getSize() < 8) {
            LOG_WARNING("SMSG_QUEST_QUERY_RESPONSE: packet too small (", packet.getSize(), " bytes)");
            return;
        }

        uint32_t questId = packet.readUInt32();
        packet.readUInt32(); // questMethod

        const uint8_t questLogStride = owner_.getPacketParsers()
                                           ? owner_.getPacketParsers()->questLogStride() : 5;
        const bool isClassicLayout = questLogStride <= 4;

        // questLevel is the third field (after questId + questMethod) in every
        // expansion's SMSG_QUEST_QUERY_RESPONSE; -1 = player-scaling (WotLK)
        int32_t questLevel = 0;
        if (packet.getData().size() >= 12) {
            questLevel = static_cast<int32_t>(readU32At(packet.getData(), 8));
            if (questLevel < -1 || questLevel > 255) questLevel = 0; // sanity: wrong layout
        }

        // ZoneOrSort follows questLevel: offset 12 in Classic/TBC, 16 in WotLK
        // (which inserted MinLevel before it). >0 = AreaTable zone id, <0 =
        // QuestSort.dbc category. Used to group the quest log by zone.
        int32_t zoneOrSort = 0;
        {
            const size_t zosOffset = (questLogStride >= 5) ? 16 : 12;
            if (packet.getData().size() >= zosOffset + 4) {
                zoneOrSort = static_cast<int32_t>(readU32At(packet.getData(), zosOffset));
                if (zoneOrSort < -100000 || zoneOrSort > 100000) zoneOrSort = 0; // sanity
            }
        }

        const QuestQueryTextCandidate parsed = pickBestQuestQueryTexts(packet.getData(), isClassicLayout);
        const QuestQueryObjectives objs = extractQuestQueryObjectives(packet.getData(), questLogStride);
        const QuestQueryRewardsData rwds = QuestQueryRewardsParser::parse(packet.getData(), questLogStride);

        for (auto& q : questLog_) {
            if (q.questId != questId) continue;

            if (questLevel != 0) q.level = questLevel;
            if (zoneOrSort != 0) q.zoneOrSort = zoneOrSort;

            const int existingScore = scoreQuestTitle(q.title);
            const bool parsedStrong = isStrongQuestTitle(parsed.title);
            const bool parsedLongEnough = parsed.title.size() >= 6;
            const bool notShorterThanExisting =
                isPlaceholderQuestTitle(q.title) || q.title.empty() || parsed.title.size() + 2 >= q.title.size();
            const bool shouldReplaceTitle =
                parsed.score > -1000 &&
                parsedStrong &&
                parsedLongEnough &&
                notShorterThanExisting &&
                (isPlaceholderQuestTitle(q.title) || q.title.empty() || parsed.score >= existingScore + 12);

            if (shouldReplaceTitle && !parsed.title.empty()) {
                q.title = parsed.title;
            }
            if (!parsed.objectives.empty() &&
                (q.objectives.empty() || q.objectives.size() < 16)) {
                q.objectives = parsed.objectives;
            }

            // Store structured kill/item objectives for later kill-count restoration.
            if (objs.valid) {
                std::unordered_set<uint32_t> validKillKeys;
                std::unordered_set<uint32_t> validItemKeys;
                for (int i = 0; i < 4; ++i) {
                    q.killObjectives[i].npcOrGoId = objs.kills[i].npcOrGoId;
                    q.killObjectives[i].required  = objs.kills[i].required;
                    if (objs.kills[i].npcOrGoId != 0 && objs.kills[i].required != 0) {
                        validKillKeys.insert(static_cast<uint32_t>(
                            objs.kills[i].npcOrGoId > 0 ? objs.kills[i].npcOrGoId
                                                       : -objs.kills[i].npcOrGoId));
                    }
                }
                for (int i = 0; i < 6; ++i) {
                    q.itemObjectives[i].itemId   = objs.items[i].itemId;
                    q.itemObjectives[i].required = objs.items[i].required;
                    if (objs.items[i].itemId != 0 && objs.items[i].required != 0)
                        validItemKeys.insert(objs.items[i].itemId);
                }
                // Remove entries produced by an earlier malformed/heuristic decode while
                // retaining live progress for objectives that are still part of the quest.
                std::erase_if(q.killCounts, [&](const auto& value) {
                    return validKillKeys.count(value.first) == 0;
                });
                std::erase_if(q.itemCounts, [&](const auto& value) {
                    return validItemKeys.count(value.first) == 0;
                });
                std::erase_if(q.requiredItemCounts, [&](const auto& value) {
                    return validItemKeys.count(value.first) == 0;
                });
                applyPackedKillCountsFromFields(q);
                for (int i = 0; i < 4; ++i) {
                    int32_t id = objs.kills[i].npcOrGoId;
                    if (id == 0 || objs.kills[i].required == 0) continue;
                    if (id > 0) owner_.queryCreatureInfo(static_cast<uint32_t>(id), 0);
                    else        owner_.queryGameObjectInfo(static_cast<uint32_t>(-id), 0);
                }
                for (int i = 0; i < 6; ++i) {
                    if (objs.items[i].itemId != 0 && objs.items[i].required != 0)
                        owner_.queryItemInfo(objs.items[i].itemId, 0);
                }
                LOG_DEBUG("Quest ", questId, " objectives parsed: kills=[",
                          objs.kills[0].npcOrGoId, "/", objs.kills[0].required, ", ",
                          objs.kills[1].npcOrGoId, "/", objs.kills[1].required, ", ",
                          objs.kills[2].npcOrGoId, "/", objs.kills[2].required, ", ",
                          objs.kills[3].npcOrGoId, "/", objs.kills[3].required, "]");
            }

            // Store reward data and pre-fetch item info for icons.
            if (rwds.valid) {
                q.rewardMoney = rwds.rewardMoney;
                for (int i = 0; i < 4; ++i) {
                    q.rewardItems[i].itemId = rwds.itemId[i];
                    q.rewardItems[i].count  = (rwds.itemId[i] != 0) ? rwds.itemCount[i] : 0;
                    if (rwds.itemId[i] != 0) owner_.queryItemInfo(rwds.itemId[i], 0);
                }
                for (int i = 0; i < 6; ++i) {
                    q.rewardChoiceItems[i].itemId = rwds.choiceItemId[i];
                    q.rewardChoiceItems[i].count  = (rwds.choiceItemId[i] != 0) ? rwds.choiceItemCount[i] : 0;
                    if (rwds.choiceItemId[i] != 0) owner_.queryItemInfo(rwds.choiceItemId[i], 0);
                }
            }
            break;
        }

        pendingQuestQueryIds_.erase(questId);
    };

    // ---- SMSG_QUESTUPDATE_ADD_PVP_KILL ----
    table[Opcode::SMSG_QUESTUPDATE_ADD_PVP_KILL] = [this](network::Packet& packet) {
        if (packet.hasRemaining(12)) {
            uint32_t questId = packet.readUInt32();
            uint32_t count   = packet.readUInt32();
            uint32_t reqCount = packet.readUInt32();

            constexpr uint32_t PVP_KILL_ENTRY = 0u;
            for (auto& quest : questLog_) {
                if (quest.questId != questId) continue;

                if (reqCount == 0) {
                    auto it = quest.killCounts.find(PVP_KILL_ENTRY);
                    if (it != quest.killCounts.end()) reqCount = it->second.second;
                }
                if (reqCount == 0) {
                    for (const auto& obj : quest.killObjectives) {
                        if (obj.npcOrGoId == 0 && obj.required > 0) {
                            reqCount = obj.required;
                            break;
                        }
                    }
                }
                if (reqCount == 0) reqCount = count;
                count = std::min(count, reqCount);
                quest.killCounts[PVP_KILL_ENTRY] = {count, reqCount};

                std::string progressMsg = quest.title + ": PvP kills " +
                    std::to_string(count) + "/" + std::to_string(reqCount);
                owner_.addSystemChatMessage(progressMsg);
                break;
            }
        }
    };

    // ---- Completed quests response (moved from GameHandler) ----
    table[Opcode::SMSG_QUERY_QUESTS_COMPLETED_RESPONSE] = [this](network::Packet& packet) {
        if (packet.hasRemaining(4)) {
            uint32_t count = packet.readUInt32();
            if (count <= 4096) {
                for (uint32_t i = 0; i < count; ++i) {
                    if (!packet.hasRemaining(4)) break;
                    uint32_t questId = packet.readUInt32();
                    owner_.completedQuestsRef().insert(questId);
                }
                LOG_DEBUG("SMSG_QUERY_QUESTS_COMPLETED_RESPONSE: ", count, " completed quests");
            }
        }
        packet.skipAll();
    };
}

// ---------------------------------------------------------------------------
// Public API methods
// ---------------------------------------------------------------------------

void QuestHandler::selectGossipOption(uint32_t optionId) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || !gossipWindowOpen_) return;
    LOG_INFO("selectGossipOption: optionId=", optionId,
             " npcGuid=0x", std::hex, currentGossip_.npcGuid, std::dec,
             " menuId=", currentGossip_.menuId,
             " numOptions=", currentGossip_.options.size());
    auto packet = GossipSelectOptionPacket::build(currentGossip_.npcGuid, currentGossip_.menuId, optionId);
    owner_.getSocket()->send(packet);

    for (const auto& opt : currentGossip_.options) {
        if (opt.id != optionId) continue;
        LOG_INFO("  matched option: id=", opt.id, " icon=", (int)opt.icon, " text='", opt.text, "'");

        std::string text = opt.text;
        std::string textLower = text;
        std::transform(textLower.begin(), textLower.end(), textLower.begin(),
                       [](unsigned char c){ return static_cast<char>(std::tolower(c)); });

        // Icon- and text-based NPC interaction fallbacks.
        // Use flags to avoid sending the same activation packet twice when
        // both the icon and text match (e.g., banker icon 6 + "deposit box").
        bool sentBanker = false;
        bool sentAuction = false;

        if (opt.icon == 6) {
            auto pkt = BankerActivatePacket::build(currentGossip_.npcGuid);
            owner_.getSocket()->send(pkt);
            sentBanker = true;
            LOG_INFO("Sent CMSG_BANKER_ACTIVATE (icon) for npc=0x", std::hex, currentGossip_.npcGuid, std::dec);
        }

        if (!sentAuction && (text == "GOSSIP_OPTION_AUCTIONEER" || textLower.find("auction") != std::string::npos)) {
            auto pkt = AuctionHelloPacket::build(currentGossip_.npcGuid);
            owner_.getSocket()->send(pkt);
            sentAuction = true;
            LOG_INFO("Sent MSG_AUCTION_HELLO for npc=0x", std::hex, currentGossip_.npcGuid, std::dec);
        }

        if (!sentBanker && (text == "GOSSIP_OPTION_BANKER" || textLower.find("deposit box") != std::string::npos)) {
            auto pkt = BankerActivatePacket::build(currentGossip_.npcGuid);
            owner_.getSocket()->send(pkt);
            sentBanker = true;
            LOG_INFO("Sent CMSG_BANKER_ACTIVATE (text) for npc=0x", std::hex, currentGossip_.npcGuid, std::dec);
        }

        const bool isVendor = (text == "GOSSIP_OPTION_VENDOR" ||
                               (textLower.find("browse") != std::string::npos &&
                                (textLower.find("goods") != std::string::npos || textLower.find("wares") != std::string::npos)));
        const bool isArmorer = (text == "GOSSIP_OPTION_ARMORER" || textLower.find("repair") != std::string::npos);
        if (isVendor || isArmorer) {
            if (isArmorer) {
                owner_.setVendorCanRepair(true);
            }
            auto pkt = ListInventoryPacket::build(currentGossip_.npcGuid);
            owner_.getSocket()->send(pkt);
            LOG_DEBUG("Sent CMSG_LIST_INVENTORY (gossip) to npc=0x", std::hex, currentGossip_.npcGuid, std::dec);
        }

        if (textLower.find("make this inn your home") != std::string::npos ||
            textLower.find("set your home") != std::string::npos) {
            auto bindPkt = BinderActivatePacket::build(currentGossip_.npcGuid);
            owner_.getSocket()->send(bindPkt);
            LOG_INFO("Sent CMSG_BINDER_ACTIVATE for npc=0x", std::hex, currentGossip_.npcGuid, std::dec);
        }

        // Stable master detection
        if (text == "GOSSIP_OPTION_STABLE" ||
            textLower.find("stable") != std::string::npos ||
            textLower.find("my pet") != std::string::npos) {
            owner_.stableMasterGuidRef() = currentGossip_.npcGuid;
            owner_.stableWindowOpenRef() = false;
            auto listPkt = ListStabledPetsPacket::build(currentGossip_.npcGuid);
            owner_.getSocket()->send(listPkt);
            LOG_INFO("Sent MSG_LIST_STABLED_PETS (gossip) to npc=0x",
                     std::hex, currentGossip_.npcGuid, std::dec);
        }
        break;
    }
}

void QuestHandler::selectGossipQuest(uint32_t questId) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || !gossipWindowOpen_) return;

    const QuestLogEntry* activeQuest = nullptr;
    for (const auto& q : questLog_) {
        if (q.questId == questId) {
            activeQuest = &q;
            break;
        }
    }

    // Validate against server-auth quest slot fields
    auto questInServerLogSlots = [&](uint32_t qid) -> bool {
        if (qid == 0 || owner_.lastPlayerFieldsRef().empty()) return false;
        const uint16_t ufQuestStart = fieldIndex(UF::PLAYER_QUEST_LOG_START);
        const uint8_t qStride = owner_.getPacketParsers() ? owner_.getPacketParsers()->questLogStride() : 5;
        const uint16_t ufQuestEnd = ufQuestStart + 25 * qStride;
        for (const auto& [key, val] : owner_.lastPlayerFieldsRef()) {
            if (key < ufQuestStart || key >= ufQuestEnd) continue;
            if ((key - ufQuestStart) % qStride != 0) continue;
            if (val == qid) return true;
        }
        return false;
    };
    const bool questInServerLog = questInServerLogSlots(questId);
    if (questInServerLog && !activeQuest) {
        addQuestToLocalLogIfMissing(questId, "Quest #" + std::to_string(questId), "");
        requestQuestQuery(questId, false);
        for (const auto& q : questLog_) {
            if (q.questId == questId) {
                activeQuest = &q;
                break;
            }
        }
    }
    const bool activeQuestConfirmedByServer = questInServerLog;
    const bool shouldStartProgressFlow = activeQuestConfirmedByServer;
    if (shouldStartProgressFlow) {
        pendingTurnInQuestId_ = questId;
        pendingTurnInNpcGuid_ = currentGossip_.npcGuid;
        pendingTurnInRewardRequest_ = activeQuest ? activeQuest->complete : false;
        auto packet = QuestgiverCompleteQuestPacket::build(currentGossip_.npcGuid, questId);
        owner_.getSocket()->send(packet);
    } else {
        pendingTurnInQuestId_ = 0;
        pendingTurnInNpcGuid_ = 0;
        pendingTurnInRewardRequest_ = false;
        auto packet = owner_.getPacketParsers()
            ? owner_.getPacketParsers()->buildQueryQuestPacket(currentGossip_.npcGuid, questId)
            : QuestgiverQueryQuestPacket::build(currentGossip_.npcGuid, questId);
        owner_.getSocket()->send(packet);
    }

    gossipWindowOpen_ = false;
}

bool QuestHandler::requestQuestQuery(uint32_t questId, bool force) {
    if (questId == 0 || owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return false;
    if (!force && pendingQuestQueryIds_.count(questId)) return false;

    network::Packet pkt(wireOpcode(Opcode::CMSG_QUEST_QUERY));
    pkt.writeUInt32(questId);
    owner_.getSocket()->send(pkt);
    pendingQuestQueryIds_.insert(questId);

    // WotLK supports CMSG_QUEST_POI_QUERY to get objective map locations.
    if (owner_.getPacketParsers() && owner_.getPacketParsers()->questLogStride() == 5) {
        const uint32_t wirePoiQuery = wireOpcode(Opcode::CMSG_QUEST_POI_QUERY);
        if (wirePoiQuery != 0xFFFF) {
            network::Packet poiPkt(static_cast<uint16_t>(wirePoiQuery));
            poiPkt.writeUInt32(1);          // count = 1
            poiPkt.writeUInt32(questId);
            owner_.getSocket()->send(poiPkt);
        }
    }
    return true;
}

void QuestHandler::setQuestTracked(uint32_t questId, bool tracked) {
    if (tracked) {
        trackedQuestIds_.insert(questId);
    } else {
        trackedQuestIds_.erase(questId);
    }
}

void QuestHandler::acceptQuest() {
    if (!questDetailsOpen_ || owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    const uint32_t questId = currentQuestDetails_.questId;
    if (questId == 0) return;
    uint64_t npcGuid = currentQuestDetails_.npcGuid;
    if (pendingQuestAcceptTimeouts_.count(questId) != 0) {
        LOG_DEBUG("Ignoring duplicate quest accept while pending: questId=", questId);
        triggerQuestAcceptResync(questId, npcGuid, "duplicate-accept");
        questDetailsOpen_ = false;
        questDetailsOpenTime_ = std::chrono::steady_clock::time_point{};
        currentQuestDetails_ = QuestDetailsData{};
        return;
    }
    const bool inLocalLog = hasQuestInLog(questId);
    const int serverSlot = findQuestLogSlotIndexFromServer(questId);
    if (serverSlot >= 0) {
        LOG_INFO("Quest already in server quest log: questId=", questId,
                 " slot=", serverSlot, " inLocalLog=", inLocalLog);
        // Ensure it's in our local log even if server already has it
        addQuestToLocalLogIfMissing(questId, currentQuestDetails_.title, currentQuestDetails_.objectives);
        requestQuestQuery(questId, false);
        // Re-query NPC status from server
        if (npcGuid && owner_.getSocket()) {
            network::Packet qsPkt(wireOpcode(Opcode::CMSG_QUESTGIVER_STATUS_QUERY));
            qsPkt.writeUInt64(npcGuid);
            owner_.getSocket()->send(qsPkt);
        }
        questDetailsOpen_ = false;
        questDetailsOpenTime_ = std::chrono::steady_clock::time_point{};
        currentQuestDetails_ = QuestDetailsData{};
        return;
    }
    if (inLocalLog) {
        LOG_WARNING("Quest accept local/server mismatch, allowing re-accept: questId=", questId);
        std::erase_if(questLog_, [&](const QuestLogEntry& q) { return q.questId == questId; });
    }

    // The server caps the quest log (20 slots in Vanilla, 25 in TBC/WotLK).
    // Accepting past the cap only earns SMSG_QUESTLOG_FULL, so stop it here
    // with a clearer message and no optimistic local entry to roll back.
    const int maxSlots = maxQuestLogSlots();
    if (static_cast<int>(questLog_.size()) >= maxSlots) {
        const std::string msg = "Your quest log is full (" + std::to_string(maxSlots) +
                                " quests). Abandon a quest to make room.";
        owner_.addUIError(msg);
        owner_.addSystemChatMessage(msg);
        if (auto* ac = owner_.services().audioCoordinator)
            if (auto* sfx = ac->getUiSoundManager()) sfx->playError();
        return;
    }

    network::Packet packet = owner_.getPacketParsers()
        ? owner_.getPacketParsers()->buildAcceptQuestPacket(npcGuid, questId)
        : QuestgiverAcceptQuestPacket::build(npcGuid, questId);
    owner_.getSocket()->send(packet);
    pendingQuestAcceptTimeouts_[questId] = 5.0f;
    pendingQuestAcceptNpcGuids_[questId] = npcGuid;

    // Immediately add to local quest log using available details
    addQuestToLocalLogIfMissing(questId, currentQuestDetails_.title, currentQuestDetails_.objectives);

    // Auto-track newly accepted quests so their objectives appear in the
    // on-screen tracker right away (matches retail behavior). Only the
    // player-driven accept path lands here — quests loaded on login/resync go
    // through addQuestToLocalLogIfMissing without tracking, so the tracker's
    // "show all when nothing tracked" fallback still applies to those.
    owner_.setQuestTracked(questId, true);

    // Play quest-accept sound
    if (auto* ac = owner_.services().audioCoordinator) {
        if (auto* sfx = ac->getUiSoundManager())
            sfx->playQuestActivate();
    }

    questDetailsOpen_ = false;
    questDetailsOpenTime_ = std::chrono::steady_clock::time_point{};
    currentQuestDetails_ = QuestDetailsData{};

    // Re-query quest giver status so marker updates (! → ?)
    if (npcGuid) {
        network::Packet qsPkt(wireOpcode(Opcode::CMSG_QUESTGIVER_STATUS_QUERY));
        qsPkt.writeUInt64(npcGuid);
        owner_.getSocket()->send(qsPkt);
    }
}

void QuestHandler::declineQuest() {
    questDetailsOpen_ = false;
    questDetailsOpenTime_ = std::chrono::steady_clock::time_point{};
    currentQuestDetails_ = QuestDetailsData{};
}

void QuestHandler::closeGossip() {
    gossipWindowOpen_ = false;
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("GOSSIP_CLOSED", {});
    currentGossip_ = GossipMessageData{};
}

void QuestHandler::offerQuestFromItem(uint64_t itemGuid, uint32_t questId) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (itemGuid == 0 || questId == 0) {
        owner_.addSystemChatMessage("Cannot start quest right now.");
        return;
    }
    // Send CMSG_QUESTGIVER_QUERY_QUEST with the item GUID as the "questgiver."
    // The server responds with SMSG_QUESTGIVER_QUEST_DETAILS which handleQuestDetails()
    // picks up and opens the Accept/Decline dialog.
    auto queryPkt = owner_.getPacketParsers()
        ? owner_.getPacketParsers()->buildQueryQuestPacket(itemGuid, questId)
        : QuestgiverQueryQuestPacket::build(itemGuid, questId);
    owner_.getSocket()->send(queryPkt);
    LOG_INFO("offerQuestFromItem: itemGuid=0x", std::hex, itemGuid, std::dec,
             " questId=", questId);
}

void QuestHandler::completeQuest() {
    if (!questRequestItemsOpen_ || owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    pendingTurnInQuestId_ = currentQuestRequestItems_.questId;
    pendingTurnInNpcGuid_ = currentQuestRequestItems_.npcGuid;
    pendingTurnInRewardRequest_ = currentQuestRequestItems_.isCompletable();

    LOG_DEBUG("completeQuest: questId=", currentQuestRequestItems_.questId,
              " completable=", currentQuestRequestItems_.isCompletable());

    auto packet = QuestgiverCompleteQuestPacket::build(
        currentQuestRequestItems_.npcGuid, currentQuestRequestItems_.questId);
    owner_.getSocket()->send(packet);
    questRequestItemsOpen_ = false;
    currentQuestRequestItems_ = QuestRequestItemsData{};
}

void QuestHandler::closeQuestRequestItems() {
    pendingTurnInRewardRequest_ = false;
    questRequestItemsOpen_ = false;
    currentQuestRequestItems_ = QuestRequestItemsData{};
}

void QuestHandler::chooseQuestReward(uint32_t rewardIndex) {
    if (!questOfferRewardOpen_ || owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    uint64_t npcGuid = currentQuestOfferReward_.npcGuid;
    LOG_INFO("Completing quest: questId=", currentQuestOfferReward_.questId,
             " npcGuid=", npcGuid, " rewardIndex=", rewardIndex);
    auto packet = QuestgiverChooseRewardPacket::build(
        npcGuid, currentQuestOfferReward_.questId, rewardIndex);
    owner_.getSocket()->send(packet);
    pendingTurnInQuestId_ = 0;
    pendingTurnInNpcGuid_ = 0;
    pendingTurnInRewardRequest_ = false;
    questOfferRewardOpen_ = false;
    currentQuestOfferReward_ = QuestOfferRewardData{};

    // Re-query quest giver status so markers update
    if (npcGuid) {
        network::Packet qsPkt(wireOpcode(Opcode::CMSG_QUESTGIVER_STATUS_QUERY));
        qsPkt.writeUInt64(npcGuid);
        owner_.getSocket()->send(qsPkt);
    }
}

void QuestHandler::closeQuestOfferReward() {
    pendingTurnInRewardRequest_ = false;
    questOfferRewardOpen_ = false;
    currentQuestOfferReward_ = QuestOfferRewardData{};
}

void QuestHandler::abandonQuest(uint32_t questId) {
    clearPendingQuestAccept(questId);
    int localIndex = -1;
    for (size_t i = 0; i < questLog_.size(); ++i) {
        if (questLog_[i].questId == questId) {
            localIndex = static_cast<int>(i);
            break;
        }
    }

    int slotIndex = findQuestLogSlotIndexFromServer(questId);
    if (slotIndex < 0 && localIndex >= 0) {
        slotIndex = localIndex;
        LOG_WARNING("Abandon quest using local slot fallback: questId=", questId, " slot=", slotIndex);
    }

    if (slotIndex >= 0 && slotIndex < 25) {
        if (owner_.getState() == WorldState::IN_WORLD && owner_.getSocket()) {
            network::Packet pkt(wireOpcode(Opcode::CMSG_QUESTLOG_REMOVE_QUEST));
            pkt.writeUInt8(static_cast<uint8_t>(slotIndex));
            owner_.getSocket()->send(pkt);
        }
    } else {
        LOG_WARNING("Abandon quest failed: no quest-log slot found for questId=", questId);
    }

    if (localIndex >= 0) {
        questLog_.erase(questLog_.begin() + static_cast<ptrdiff_t>(localIndex));
        if (owner_.addonEventCallbackRef()) {
            owner_.addonEventCallbackRef()("QUEST_LOG_UPDATE", {});
            owner_.addonEventCallbackRef()("UNIT_QUEST_LOG_CHANGED", {"player"});
            owner_.addonEventCallbackRef()("QUEST_REMOVED", {std::to_string(questId)});
        }
    }

    // Re-query nearby quest giver NPCs so markers refresh (e.g. "?" → "!")
    if (owner_.getSocket()) {
        for (const auto& [guid, entity] : owner_.getEntityManager().getEntities()) {
            if (entity->getType() != ObjectType::UNIT) continue;
            auto unit = std::static_pointer_cast<Unit>(entity);
            if (unit->getNpcFlags() & 0x02) {
                network::Packet qsPkt(wireOpcode(Opcode::CMSG_QUESTGIVER_STATUS_QUERY));
                qsPkt.writeUInt64(guid);
                owner_.getSocket()->send(qsPkt);
            }
        }
    }

    // Remove any quest POI minimap markers for this quest.
    gossipPois_.erase(
        std::remove_if(gossipPois_.begin(), gossipPois_.end(),
            [questId](const GossipPoi& p) { return p.data == questId; }),
        gossipPois_.end());
}

void QuestHandler::shareQuestWithParty(uint32_t questId) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) {
        owner_.addSystemChatMessage("Cannot share quest: not in world.");
        return;
    }
    if (!owner_.isInGroup()) {
        owner_.addSystemChatMessage("You must be in a group to share a quest.");
        return;
    }
    network::Packet pkt(wireOpcode(Opcode::CMSG_PUSHQUESTTOPARTY));
    pkt.writeUInt32(questId);
    owner_.getSocket()->send(pkt);
    // Local feedback: find quest title
    for (const auto& q : questLog_) {
        if (q.questId == questId && !q.title.empty()) {
            owner_.addSystemChatMessage("Sharing quest: " + q.title);
            return;
        }
    }
    owner_.addSystemChatMessage("Quest shared.");
}

void QuestHandler::acceptSharedQuest() {
    if (!pendingSharedQuest_ || !owner_.getSocket()) return;
    pendingSharedQuest_ = false;
    network::Packet pkt(wireOpcode(Opcode::CMSG_QUEST_CONFIRM_ACCEPT));
    pkt.writeUInt32(sharedQuestId_);
    owner_.getSocket()->send(pkt);
    // Auto-track the accepted shared quest, same as a normal accept.
    owner_.setQuestTracked(sharedQuestId_, true);
    owner_.addSystemChatMessage("Accepted: " + sharedQuestTitle_);
}

void QuestHandler::declineSharedQuest() {
    pendingSharedQuest_ = false;
    // No response packet needed — just dismiss the UI
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

int QuestHandler::maxQuestLogSlots() const {
    return isClassicLikeExpansion() ? 20 : 25;
}

const std::string& QuestHandler::getQuestSortName(uint32_t sortId) {
    static const std::string kEmpty;
    if (!questSortDbcLoaded_) {
        questSortDbcLoaded_ = true;
        auto* am = owner_.services().assetManager;
        if (am && am->isInitialized()) {
            auto dbc = am->loadDBC("QuestSort.dbc");
            // ID(0) + Name locstring whose English text sits at field 1
            if (dbc && dbc->isLoaded() && dbc->getFieldCount() >= 2) {
                for (uint32_t i = 0; i < dbc->getRecordCount(); ++i) {
                    uint32_t id = dbc->getUInt32(i, 0);
                    std::string name = dbc->getString(i, 1);
                    if (id != 0 && !name.empty())
                        questSortNames_[id] = std::move(name);
                }
                LOG_INFO("Loaded ", questSortNames_.size(), " quest sort names");
            }
        }
    }
    auto it = questSortNames_.find(sortId);
    return it != questSortNames_.end() ? it->second : kEmpty;
}

bool QuestHandler::hasQuestInLog(uint32_t questId) const {
    for (const auto& q : questLog_) {
        if (q.questId == questId) return true;
    }
    return false;
}

int QuestHandler::findQuestLogSlotIndexFromServer(uint32_t questId) const {
    if (questId == 0 || owner_.lastPlayerFieldsRef().empty()) return -1;
    const uint16_t ufQuestStart = fieldIndex(UF::PLAYER_QUEST_LOG_START);
    const uint8_t qStride = owner_.getPacketParsers() ? owner_.getPacketParsers()->questLogStride() : 5;
    const uint16_t maxSlots = static_cast<uint16_t>(maxQuestLogSlots());
    for (uint16_t slot = 0; slot < maxSlots; ++slot) {
        const uint16_t idField = ufQuestStart + slot * qStride;
        auto it = owner_.lastPlayerFieldsRef().find(idField);
        if (it != owner_.lastPlayerFieldsRef().end() && it->second == questId) {
            return static_cast<int>(slot);
        }
    }
    return -1;
}

void QuestHandler::addQuestToLocalLogIfMissing(uint32_t questId, const std::string& title, const std::string& objectives) {
    if (questId == 0 || hasQuestInLog(questId)) return;
    QuestLogEntry entry;
    entry.questId = questId;
    entry.title = title.empty() ? ("Quest #" + std::to_string(questId))
                                : normalizeQuestText(title, true);
    entry.objectives = normalizeQuestText(objectives, false);
    questLog_.push_back(std::move(entry));
    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("QUEST_ACCEPTED", {std::to_string(questId)});
        owner_.addonEventCallbackRef()("QUEST_LOG_UPDATE", {});
        owner_.addonEventCallbackRef()("UNIT_QUEST_LOG_CHANGED", {"player"});
    }
}

bool QuestHandler::resyncQuestLogFromServerSlots(bool forceQueryMetadata) {
    if (owner_.lastPlayerFieldsRef().empty()) return false;

    const uint16_t ufQuestStart = fieldIndex(UF::PLAYER_QUEST_LOG_START);
    // Unmapped for this build's field table. applyQuestStateFromFields checks
    // this and this did not, so the slot arithmetic below ran from 0xFFFF and
    // every lookup missed — which reaches the same erase with an empty set.
    if (ufQuestStart == 0xFFFF) return false;
    const uint8_t qStride = owner_.getPacketParsers() ? owner_.getPacketParsers()->questLogStride() : 5;

    // Whether the quest area of the player's fields has arrived at all, which
    // is a different question from whether any player field has.
    //
    // This used to answer "done" on the first frame the field map was
    // non-empty, and the erase below then measured the local log against an
    // empty set and emptied it — a partial update block carrying a health or a
    // position change was enough. The resync runs once, so there was no second
    // chance: the quest log stayed empty for the session and the tracker with
    // it.
    //
    // Any key inside the quest slot range counts, including a zero one: a slot
    // written as zero is the server saying that slot is empty, which is real
    // information. Only the absence of every one of them means the block has
    // not come yet.
    const uint32_t questAreaEnd = static_cast<uint32_t>(ufQuestStart) + 25u * qStride;
    bool sawQuestArea = false;
    for (const auto& [key, value] : owner_.lastPlayerFieldsRef()) {
        (void)value;
        if (key >= ufQuestStart && key < questAreaEnd) { sawQuestArea = true; break; }
    }
    if (!sawQuestArea) return false;

    std::unordered_map<uint32_t, bool> serverQuestComplete;
    serverQuestComplete.reserve(25);
    for (uint16_t slot = 0; slot < 25; ++slot) {
        const uint16_t idField    = ufQuestStart + slot * qStride;
        const uint16_t stateField = ufQuestStart + slot * qStride + 1;
        auto it = owner_.lastPlayerFieldsRef().find(idField);
        if (it == owner_.lastPlayerFieldsRef().end()) continue;
        uint32_t questId = it->second;
        if (questId == 0) continue;

        bool complete = false;
        if (qStride >= 2) {
            auto stateIt = owner_.lastPlayerFieldsRef().find(stateField);
            if (stateIt != owner_.lastPlayerFieldsRef().end()) {
                complete = isQuestSlotComplete(qStride, stateIt->second);
            }
        }
        serverQuestComplete[questId] = complete;
    }

    std::unordered_set<uint32_t> serverQuestIds;
    serverQuestIds.reserve(serverQuestComplete.size());
    for (const auto& [qid, _] : serverQuestComplete) serverQuestIds.insert(qid);

    const size_t localBefore = questLog_.size();
    std::erase_if(questLog_, [&](const QuestLogEntry& q) {
        return q.questId == 0 || serverQuestIds.count(q.questId) == 0;
    });
    const size_t removed = localBefore - questLog_.size();

    size_t added = 0;
    for (uint32_t questId : serverQuestIds) {
        if (hasQuestInLog(questId)) continue;
        addQuestToLocalLogIfMissing(questId, "Quest #" + std::to_string(questId), "");
        ++added;
    }

    size_t marked = 0;
    for (auto& quest : questLog_) {
        auto it = serverQuestComplete.find(quest.questId);
        if (it == serverQuestComplete.end()) continue;
        if (it->second && !quest.complete) {
            quest.complete = true;
            ++marked;
            LOG_DEBUG("Quest ", quest.questId, " marked complete from update fields");
        }
    }

    if (forceQueryMetadata) {
        for (uint32_t questId : serverQuestIds) {
            requestQuestQuery(questId, false);
        }
    }

    LOG_INFO("Quest log resync from server slots: server=", serverQuestIds.size(),
             " localBefore=", localBefore, " removed=", removed, " added=", added,
             " markedComplete=", marked);
    return true;
}

void QuestHandler::applyQuestStateFromFields(const FlatFieldMap& fields) {
    const uint16_t ufQuestStart = fieldIndex(UF::PLAYER_QUEST_LOG_START);
    if (ufQuestStart == 0xFFFF) return;

    const uint8_t qStride = owner_.getPacketParsers() ? owner_.getPacketParsers()->questLogStride() : 5;
    if (qStride < 2) return;

    for (uint16_t slot = 0; slot < 25; ++slot) {
        const uint16_t idField    = ufQuestStart + slot * qStride;
        const uint16_t stateField = idField + 1;
        auto idIt = fields.find(idField);
        if (idIt == fields.end()) continue;
        uint32_t questId = idIt->second;
        if (questId == 0) continue;

        // Add quest to local log only if we have a pending accept for it
        if (!hasQuestInLog(questId) && pendingQuestAcceptTimeouts_.count(questId) != 0) {
            addQuestToLocalLogIfMissing(questId, "Quest #" + std::to_string(questId), "");
            requestQuestQuery(questId, false);
            // Re-query quest giver status for the NPC that gave us this quest
            auto pendingIt = pendingQuestAcceptNpcGuids_.find(questId);
            if (pendingIt != pendingQuestAcceptNpcGuids_.end() && pendingIt->second != 0 && owner_.getSocket()) {
                network::Packet qsPkt(wireOpcode(Opcode::CMSG_QUESTGIVER_STATUS_QUERY));
                qsPkt.writeUInt64(pendingIt->second);
                owner_.getSocket()->send(qsPkt);
            }
            clearPendingQuestAccept(questId);
        }

        // VALUES updates are authoritative quest progress too. Some servers do
        // not emit (or clients may miss) a separate ADD_KILL notification for
        // every counter change, so refresh the tracker from the quest slot.
        for (auto& quest : questLog_) {
            if (quest.questId == questId) {
                applyPackedKillCountsFromFields(quest);
                break;
            }
        }

        auto stateIt = fields.find(stateField);
        if (stateIt == fields.end()) continue;
        bool serverComplete = isQuestSlotComplete(qStride, stateIt->second);
        if (!serverComplete) continue;

        for (auto& quest : questLog_) {
            if (quest.questId == questId && !quest.complete) {
                quest.complete = true;
                LOG_INFO("Quest ", questId, " marked complete from VALUES update field state");
                break;
            }
        }
    }
}

void QuestHandler::applyPackedKillCountsFromFields(QuestLogEntry& quest) {
    if (owner_.lastPlayerFieldsRef().empty()) return;

    const uint16_t ufQuestStart = fieldIndex(UF::PLAYER_QUEST_LOG_START);
    if (ufQuestStart == 0xFFFF) return;

    const uint8_t qStride = owner_.getPacketParsers() ? owner_.getPacketParsers()->questLogStride() : 5;
    if (qStride < 3) return;

    int slot = findQuestLogSlotIndexFromServer(quest.questId);
    if (slot < 0) return;

    const uint16_t countField1 = ufQuestStart + static_cast<uint16_t>(slot) * qStride
                               + questObjectiveCountFieldOffset(qStride);
    const uint16_t countField2 = (qStride >= 5)
                                     ? static_cast<uint16_t>(countField1 + 1)
                                     : static_cast<uint16_t>(0xFFFF);

    auto f1It = owner_.lastPlayerFieldsRef().find(countField1);
    if (f1It == owner_.lastPlayerFieldsRef().end()) return;
    const uint32_t packed1 = f1It->second;

    uint32_t packed2 = 0;
    if (countField2 != 0xFFFF) {
        auto f2It = owner_.lastPlayerFieldsRef().find(countField2);
        if (f2It != owner_.lastPlayerFieldsRef().end()) packed2 = f2It->second;
    }

    const auto counts = decodeQuestObjectiveCounts(qStride, packed1, packed2);

    // Apply kill objective counts (indices 0-3).
    for (int i = 0; i < 4; ++i) {
        const auto& obj = quest.killObjectives[i];
        if (obj.npcOrGoId == 0 || obj.required == 0) continue;
        const uint32_t entryKey = static_cast<uint32_t>(
            obj.npcOrGoId > 0 ? obj.npcOrGoId : -obj.npcOrGoId);
        if (counts[i] == 0 && quest.killCounts.count(entryKey)) continue;
        const uint32_t current = std::min(counts[i], obj.required);
        quest.killCounts[entryKey] = {current, obj.required};
        LOG_DEBUG("Quest ", quest.questId, " objective[", i, "]: npcOrGo=",
                  obj.npcOrGoId, " count=", current, "/", obj.required);
    }

    // Item collection progress is not stored in PLAYER_QUEST_LOG counters;
    // those fields contain the four creature/GO counters in every expansion.
    for (int i = 0; i < 6; ++i) {
        const auto& obj = quest.itemObjectives[i];
        if (obj.itemId == 0 || obj.required == 0) continue;
        quest.requiredItemCounts.emplace(obj.itemId, obj.required);
    }
}

void QuestHandler::clearPendingQuestAccept(uint32_t questId) {
    pendingQuestAcceptTimeouts_.erase(questId);
    pendingQuestAcceptNpcGuids_.erase(questId);
}

void QuestHandler::triggerQuestAcceptResync(uint32_t questId, uint64_t npcGuid, const char* reason) {
    if (questId == 0 || !owner_.getSocket() || owner_.getState() != WorldState::IN_WORLD) return;

    LOG_INFO("Quest accept resync: questId=", questId, " reason=", reason ? reason : "unknown");
    requestQuestQuery(questId, true);

    if (npcGuid != 0) {
        network::Packet qsPkt(wireOpcode(Opcode::CMSG_QUESTGIVER_STATUS_QUERY));
        qsPkt.writeUInt64(npcGuid);
        owner_.getSocket()->send(qsPkt);

        auto queryPkt = owner_.getPacketParsers()
            ? owner_.getPacketParsers()->buildQueryQuestPacket(npcGuid, questId)
            : QuestgiverQueryQuestPacket::build(npcGuid, questId);
        owner_.getSocket()->send(queryPkt);
    }
}

// ---------------------------------------------------------------------------
// Packet handlers
// ---------------------------------------------------------------------------

void QuestHandler::handleGossipMessage(network::Packet& packet) {
    bool ok = owner_.getPacketParsers() ? owner_.getPacketParsers()->parseGossipMessage(packet, currentGossip_)
                                    : GossipMessageParser::parse(packet, currentGossip_);
    if (!ok) return;
    if (questDetailsOpen_) return; // Don't reopen gossip while viewing quest
    gossipWindowOpen_ = true;
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("GOSSIP_SHOW", {});
    owner_.closeVendor(); // Close vendor if gossip opens

    // Classify gossip quests and update quest log + overhead NPC markers.
    classifyGossipQuests(true);

    // Query NPC body text if not cached
    if (currentGossip_.titleTextId > 0 && !npcTextCache_.count(currentGossip_.titleTextId)) {
        uint32_t wireOp = wireOpcode(Opcode::CMSG_NPC_TEXT_QUERY);
        if (wireOp != 0xFFFF && owner_.getSocket()) {
            network::Packet qPkt(static_cast<uint16_t>(wireOp));
            qPkt.writeUInt32(currentGossip_.titleTextId);
            qPkt.writeUInt64(currentGossip_.npcGuid);
            owner_.getSocket()->send(qPkt);
        }
    }

    // Play NPC greeting voice
    if (owner_.npcGreetingCallbackRef() && currentGossip_.npcGuid != 0) {
        auto entity = owner_.getEntityManager().getEntity(currentGossip_.npcGuid);
        if (entity) {
            glm::vec3 npcPos(entity->getX(), entity->getY(), entity->getZ());
            owner_.npcGreetingCallbackRef()(currentGossip_.npcGuid, npcPos);
        }
    }
}

void QuestHandler::handleQuestgiverQuestList(network::Packet& packet) {
    if (!packet.hasRemaining(8)) return;

    GossipMessageData data;
    data.npcGuid = packet.readUInt64();
    data.menuId = 0;
    data.titleTextId = 0;

    std::string header = packet.readString();
    if (packet.hasRemaining(8)) {
        (void)packet.readUInt32(); // emoteDelay / unk
        (void)packet.readUInt32(); // emote / unk
    }
    (void)header;

    // questCount is uint8 in all WoW versions for SMSG_QUESTGIVER_QUEST_LIST.
    uint32_t questCount = 0;
    if (packet.hasRemaining(1)) {
        questCount = packet.readUInt8();
    }

    const bool hasQuestFlagsField = !isClassicLikeExpansion() && !isActiveExpansion("tbc");

    data.quests.reserve(questCount);
    for (uint32_t i = 0; i < questCount; ++i) {
        if (!packet.hasRemaining(12)) break;
        GossipQuestItem q;
        q.questId = packet.readUInt32();
        q.questIcon = packet.readUInt32();
        q.questLevel = static_cast<int32_t>(packet.readUInt32());

        if (hasQuestFlagsField && packet.hasRemaining(5)) {
            q.questFlags = packet.readUInt32();
            q.isRepeatable = packet.readUInt8();
        } else {
            q.questFlags = 0;
            q.isRepeatable = 0;
        }
        q.title = normalizeWowTextTokens(packet.readString());
        if (q.questId != 0) {
            data.quests.push_back(std::move(q));
        }
    }

    currentGossip_ = std::move(data);
    gossipWindowOpen_ = true;
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("GOSSIP_SHOW", {});
    owner_.closeVendor();

    classifyGossipQuests(false);

    LOG_INFO("Questgiver quest list: npc=0x", std::hex, currentGossip_.npcGuid, std::dec,
             " quests=", currentGossip_.quests.size());
}

// Shared quest-icon classification for gossip windows. Derives NPC quest status
// from icon values so overhead markers stay aligned with what the NPC offers.
// updateQuestLog: if true, also patches quest log completion state (gossip handler
// does this because it has the freshest data; quest-list handler skips it because
// completion updates arrive via separate packets).
void QuestHandler::classifyGossipQuests(bool updateQuestLog) {
    // Icon values come from the server's QUEST_STATUS enum, not a client constant,
    // so these magic numbers are protocol-defined and stable across expansions.
    auto isCompletable = [](uint32_t icon) { return icon == 5 || icon == 6 || icon == 10; };
    auto isIncomplete  = [](uint32_t icon) { return icon == 3 || icon == 4; };
    auto isAvailable   = [](uint32_t icon) { return icon == 2 || icon == 7 || icon == 8; };

    bool hasAvailable = false, hasReward = false, hasIncomplete = false;
    for (const auto& q : currentGossip_.quests) {
        bool completable = isCompletable(q.questIcon);
        bool incomplete  = isIncomplete(q.questIcon);
        bool available   = isAvailable(q.questIcon);
        hasAvailable |= available;
        hasReward |= completable;
        hasIncomplete |= incomplete;

        if (updateQuestLog) {
            for (auto& entry : questLog_) {
                if (entry.questId == q.questId) {
                    entry.complete = completable;
                    entry.title = normalizeQuestText(q.title, true);
                    if (q.questLevel > 0 && entry.level <= 0) entry.level = q.questLevel;
                    break;
                }
            }
        }
    }
    if (currentGossip_.npcGuid != 0) {
        QuestGiverStatus status = QuestGiverStatus::NONE;
        if (hasReward) status = QuestGiverStatus::REWARD;
        else if (hasAvailable) status = QuestGiverStatus::AVAILABLE;
        else if (hasIncomplete) status = QuestGiverStatus::INCOMPLETE;
        if (status != QuestGiverStatus::NONE)
            npcQuestStatus_[currentGossip_.npcGuid] = status;
    }
}

void QuestHandler::handleGossipComplete(network::Packet& packet) {
    (void)packet;

    // Play farewell sound before closing
    if (owner_.npcFarewellCallbackRef() && currentGossip_.npcGuid != 0) {
        auto entity = owner_.getEntityManager().getEntity(currentGossip_.npcGuid);
        if (entity && entity->getType() == ObjectType::UNIT) {
            glm::vec3 pos(entity->getX(), entity->getY(), entity->getZ());
            owner_.npcFarewellCallbackRef()(currentGossip_.npcGuid, pos);
        }
    }

    gossipWindowOpen_ = false;
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("GOSSIP_CLOSED", {});
    currentGossip_ = GossipMessageData{};
}

void QuestHandler::handleNpcTextUpdate(network::Packet& packet) {
    if (!packet.hasRemaining(4)) return;
    uint32_t textId = packet.readUInt32();
    // 8 text entries: each has probability(4) + text0 + text1 + lang(4) + 6 emote fields(24)
    for (int i = 0; i < 8; ++i) {
        if (!packet.hasRemaining(4)) break;
        /*float prob =*/ packet.readFloat();
        if (!packet.hasData()) break;
        std::string text0 = packet.readString();
        if (!packet.hasData()) break;
        std::string text1 = packet.readString();
        if (!packet.hasRemaining(28)) break; // lang(4) + 6×emote(4) = 28
        packet.setReadPos(packet.getReadPos() + 28);
        if (!text0.empty() && !npcTextCache_.count(textId)) {
            npcTextCache_[textId] = std::move(text0);
        } else if (!text1.empty() && !npcTextCache_.count(textId)) {
            npcTextCache_[textId] = std::move(text1);
        }
    }
    LOG_DEBUG("NPC text update: id=", textId,
             " cached=", npcTextCache_.count(textId) ? "yes" : "no");
}

const std::string& QuestHandler::getNpcText(uint32_t textId) const {
    static const std::string empty;
    auto it = npcTextCache_.find(textId);
    return (it != npcTextCache_.end()) ? it->second : empty;
}

void QuestHandler::handleQuestPoiQueryResponse(network::Packet& packet) {
    // WotLK 3.3.5a SMSG_QUEST_POI_QUERY_RESPONSE format:
    //   uint32 questCount
    //   per quest:
    //     uint32 questId
    //     uint32 poiCount
    //     per poi:
    //       uint32 poiId
    //       int32  objIndex      (-1 = no specific objective)
    //       uint32 mapId
    //       uint32 areaId
    //       uint32 floorId
    //       uint32 unk1
    //       uint32 unk2
    //       uint32 pointCount
    //       per point: int32 x, int32 y
    if (!packet.hasRemaining(4)) return;
    const uint32_t questCount = packet.readUInt32();
    for (uint32_t qi = 0; qi < questCount; ++qi) {
        if (!packet.hasRemaining(8)) return;
        const uint32_t questId  = packet.readUInt32();
        const uint32_t poiCount = packet.readUInt32();

        // Remove any previously added POI markers for this quest
        gossipPois_.erase(
            std::remove_if(gossipPois_.begin(), gossipPois_.end(),
                [questId](const GossipPoi& p) {
                    return p.data == questId;
                }),
            gossipPois_.end());

        // Find the quest title for the marker label.
        std::string questTitle;
        for (const auto& q : questLog_) {
            if (q.questId == questId) { questTitle = q.title; break; }
        }

        for (uint32_t pi = 0; pi < poiCount; ++pi) {
            if (!packet.hasRemaining(32)) return;
            packet.readUInt32();  // poiId
            const int32_t objIndex = static_cast<int32_t>(packet.readUInt32());
            const uint32_t mapId    = packet.readUInt32();
            packet.readUInt32();  // areaId
            packet.readUInt32();  // floorId
            packet.readUInt32();  // unk1
            packet.readUInt32();  // unk2
            const uint32_t pointCount = packet.readUInt32();
            if (pointCount == 0) continue;
            if (packet.getRemainingSize() < pointCount * 8) return;
            float sumX = 0.0f, sumY = 0.0f;
            for (uint32_t pt = 0; pt < pointCount; ++pt) {
                const int32_t px = static_cast<int32_t>(packet.readUInt32());
                const int32_t py = static_cast<int32_t>(packet.readUInt32());
                sumX += static_cast<float>(px);
                sumY += static_cast<float>(py);
            }
            // Skip POIs for maps other than the player's current map.
            if (mapId != owner_.currentMapIdRef()) continue;
            GossipPoi poi;
            // QuestPOIPoint uses the opposite axis order from the canonical
            // world coordinates used by movement and the map renderer. Store
            // it canonically here so canonicalToRender() does not place every
            // objective far outside its zone bounds.
            poi.x    = sumY / static_cast<float>(pointCount);
            poi.y    = sumX / static_cast<float>(pointCount);
            poi.icon = 6;  // generic quest POI icon
            poi.data = questId;
            poi.questObjectiveIndex = objIndex;
            poi.name = questTitle.empty() ? "Quest objective" : questTitle;
            LOG_DEBUG("Quest POI: questId=", questId, " mapId=", mapId,
                      " centroid=(", poi.x, ",", poi.y, ") title=", poi.name);
            if (gossipPois_.size() >= 200) gossipPois_.erase(gossipPois_.begin());
            gossipPois_.push_back(std::move(poi));
        }
    }
}

void QuestHandler::handleQuestDetails(network::Packet& packet) {
    QuestDetailsData data;
    bool ok = owner_.getPacketParsers() ? owner_.getPacketParsers()->parseQuestDetails(packet, data)
                                    : QuestDetailsParser::parse(packet, data);
    if (!ok) {
        LOG_WARNING("Failed to parse SMSG_QUESTGIVER_QUEST_DETAILS");
        return;
    }
    currentQuestDetails_ = data;
    for (auto& q : questLog_) {
        if (q.questId != data.questId) continue;
        if (!data.title.empty() && (isPlaceholderQuestTitle(q.title) || data.title.size() >= q.title.size())) {
            q.title = normalizeQuestText(data.title, true);
        }
        if (!data.objectives.empty() && (q.objectives.empty() || data.objectives.size() > q.objectives.size())) {
            q.objectives = normalizeQuestText(data.objectives, false);
        }
        break;
    }
    // Pre-fetch item info for all reward items
    for (const auto& item : data.rewardChoiceItems) owner_.queryItemInfo(item.itemId, 0);
    for (const auto& item : data.rewardItems)       owner_.queryItemInfo(item.itemId, 0);
    // Delay opening the window slightly to allow item queries to complete
    questDetailsOpenTime_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    gossipWindowOpen_ = false;
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("QUEST_DETAIL", {});
}

void QuestHandler::handleQuestRequestItems(network::Packet& packet) {
    QuestRequestItemsData data;
    if (!QuestRequestItemsParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_QUESTGIVER_REQUEST_ITEMS");
        return;
    }
    clearPendingQuestAccept(data.questId);

    if (pendingTurnInRewardRequest_ &&
        data.questId == pendingTurnInQuestId_ &&
        data.npcGuid == pendingTurnInNpcGuid_ &&
        data.isCompletable() &&
        owner_.getSocket()) {
        auto rewardReq = QuestgiverRequestRewardPacket::build(data.npcGuid, data.questId);
        owner_.getSocket()->send(rewardReq);
        pendingTurnInRewardRequest_ = false;
    }

    currentQuestRequestItems_ = data;
    questRequestItemsOpen_ = true;
    gossipWindowOpen_ = false;
    questDetailsOpen_ = false;
    questDetailsOpenTime_ = std::chrono::steady_clock::time_point{};

    // Query item names for required items
    for (const auto& item : data.requiredItems) {
        owner_.queryItemInfo(item.itemId, 0);
    }

    // Server-authoritative turn-in requirements
    for (auto& q : questLog_) {
        if (q.questId != data.questId) continue;
        q.complete = data.isCompletable();
        q.requiredItemCounts.clear();

        std::ostringstream oss;
        if (!data.completionText.empty()) {
            oss << data.completionText;
            if (!data.requiredItems.empty() || data.requiredMoney > 0) oss << "\n\n";
        }
        if (!data.requiredItems.empty()) {
            oss << "Required items:";
            for (const auto& item : data.requiredItems) {
                std::string itemLabel = "Item " + std::to_string(item.itemId);
                if (const auto* info = owner_.getItemInfo(item.itemId)) {
                    if (!info->name.empty()) itemLabel = info->name;
                }
                q.requiredItemCounts[item.itemId] = item.count;
                oss << "\n- " << itemLabel << " x" << item.count;
            }
        }
        if (data.requiredMoney > 0) {
            if (!data.requiredItems.empty()) oss << "\n";
            oss << "\nRequired money: " << formatCopperAmount(data.requiredMoney);
        }
        q.objectives = oss.str();
        break;
    }
}

void QuestHandler::handleQuestOfferReward(network::Packet& packet) {
    QuestOfferRewardData data;
    if (!QuestOfferRewardParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_QUESTGIVER_OFFER_REWARD");
        return;
    }
    clearPendingQuestAccept(data.questId);
    LOG_INFO("Quest offer reward: questId=", data.questId, " title=\"", data.title, "\"");
    if (pendingTurnInQuestId_ == data.questId) {
        pendingTurnInQuestId_ = 0;
        pendingTurnInNpcGuid_ = 0;
        pendingTurnInRewardRequest_ = false;
    }
    currentQuestOfferReward_ = data;
    questOfferRewardOpen_ = true;
    questRequestItemsOpen_ = false;
    gossipWindowOpen_ = false;
    questDetailsOpen_ = false;
    questDetailsOpenTime_ = std::chrono::steady_clock::time_point{};
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("QUEST_COMPLETE", {});

    // Query item names for reward items
    for (const auto& item : data.choiceRewards)
        owner_.queryItemInfo(item.itemId, 0);
    for (const auto& item : data.fixedRewards)
        owner_.queryItemInfo(item.itemId, 0);
}

void QuestHandler::handleQuestConfirmAccept(network::Packet& packet) {
    size_t rem = packet.getRemainingSize();
    if (rem < 4) return;

    sharedQuestId_    = packet.readUInt32();
    sharedQuestTitle_ = packet.readString();
    if (packet.hasRemaining(8)) {
        sharedQuestSharerGuid_ = packet.readUInt64();
    }

    sharedQuestSharerName_.clear();
    auto entity = owner_.getEntityManager().getEntity(sharedQuestSharerGuid_);
    if (auto* unit = dynamic_cast<Unit*>(entity.get())) {
        sharedQuestSharerName_ = unit->getName();
    }
    if (sharedQuestSharerName_.empty()) {
        auto nit = owner_.getPlayerNameCache().find(sharedQuestSharerGuid_);
        if (nit != owner_.getPlayerNameCache().end())
            sharedQuestSharerName_ = nit->second;
    }
    if (sharedQuestSharerName_.empty()) {
        char tmp[32];
        std::snprintf(tmp, sizeof(tmp), "0x%llX",
                      static_cast<unsigned long long>(sharedQuestSharerGuid_));
        sharedQuestSharerName_ = tmp;
    }

    pendingSharedQuest_ = true;
    owner_.addSystemChatMessage(sharedQuestSharerName_ + " has shared the quest \"" +
                                sharedQuestTitle_ + "\" with you.");
    LOG_INFO("SMSG_QUEST_CONFIRM_ACCEPT: questId=", sharedQuestId_,
             " title=", sharedQuestTitle_, " sharer=", sharedQuestSharerName_);
}

} // namespace game
} // namespace wowee
