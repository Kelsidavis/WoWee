#include "game/game_handler.hpp"
#include "game/packet_parsers.hpp"
#include "game/transport_manager.hpp"
#include "game/warden_crypto.hpp"
#include "game/warden_memory.hpp"
#include "game/warden_module.hpp"
#include "game/opcodes.hpp"
#include "game/update_field_table.hpp"
#include "game/expansion_profile.hpp"
#include "rendering/renderer.hpp"
#include "audio/activity_sound_manager.hpp"
#include "audio/combat_sound_manager.hpp"
#include "audio/spell_sound_manager.hpp"
#include "audio/ui_sound_manager.hpp"
#include "pipeline/dbc_layout.hpp"
#include "network/world_socket.hpp"
#include "network/packet.hpp"
#include "auth/crypto.hpp"
#include "core/coordinates.hpp"
#include "core/application.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "core/logger.hpp"
#include <glm/gtx/quaternion.hpp>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <ctime>
#include <random>
#include <zlib.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <array>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <openssl/sha.h>
#include <openssl/hmac.h>

namespace wowee {
namespace game {

namespace {
const char* worldStateName(WorldState state) {
    switch (state) {
        case WorldState::DISCONNECTED: return "DISCONNECTED";
        case WorldState::CONNECTING: return "CONNECTING";
        case WorldState::CONNECTED: return "CONNECTED";
        case WorldState::CHALLENGE_RECEIVED: return "CHALLENGE_RECEIVED";
        case WorldState::AUTH_SENT: return "AUTH_SENT";
        case WorldState::AUTHENTICATED: return "AUTHENTICATED";
        case WorldState::READY: return "READY";
        case WorldState::CHAR_LIST_REQUESTED: return "CHAR_LIST_REQUESTED";
        case WorldState::CHAR_LIST_RECEIVED: return "CHAR_LIST_RECEIVED";
        case WorldState::ENTERING_WORLD: return "ENTERING_WORLD";
        case WorldState::IN_WORLD: return "IN_WORLD";
        case WorldState::FAILED: return "FAILED";
    }
    return "UNKNOWN";
}

bool isAuthCharPipelineOpcode(LogicalOpcode op) {
    switch (op) {
        case Opcode::SMSG_AUTH_CHALLENGE:
        case Opcode::SMSG_AUTH_RESPONSE:
        case Opcode::SMSG_CLIENTCACHE_VERSION:
        case Opcode::SMSG_TUTORIAL_FLAGS:
        case Opcode::SMSG_WARDEN_DATA:
        case Opcode::SMSG_CHAR_ENUM:
        case Opcode::SMSG_CHAR_CREATE:
        case Opcode::SMSG_CHAR_DELETE:
            return true;
        default:
            return false;
    }
}

bool isActiveExpansion(const char* expansionId) {
    auto& app = core::Application::getInstance();
    auto* registry = app.getExpansionRegistry();
    if (!registry) return false;
    auto* profile = registry->getActive();
    if (!profile) return false;
    return profile->id == expansionId;
}

bool isClassicLikeExpansion() {
    return isActiveExpansion("classic") || isActiveExpansion("turtle");
}

std::string formatCopperAmount(uint32_t amount) {
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

bool readCStringAt(const std::vector<uint8_t>& data, size_t start, std::string& out, size_t& nextPos) {
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

std::string asciiLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

std::vector<std::string> splitWowPath(const std::string& wowPath) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : wowPath) {
        if (c == '\\' || c == '/') {
            if (!cur.empty()) {
                out.push_back(cur);
                cur.clear();
            }
            continue;
        }
        cur.push_back(c);
    }
    if (!cur.empty()) out.push_back(cur);
    return out;
}

int pathCaseScore(const std::string& name) {
    int score = 0;
    for (unsigned char c : name) {
        if (std::islower(c)) score += 2;
        else if (std::isupper(c)) score -= 1;
    }
    return score;
}

std::string resolveCaseInsensitiveDataPath(const std::string& dataRoot, const std::string& wowPath) {
    if (dataRoot.empty() || wowPath.empty()) return std::string();
    std::filesystem::path cur(dataRoot);
    std::error_code ec;
    if (!std::filesystem::exists(cur, ec) || !std::filesystem::is_directory(cur, ec)) {
        return std::string();
    }

    for (const std::string& segment : splitWowPath(wowPath)) {
        std::string wanted = asciiLower(segment);
        std::filesystem::path bestPath;
        int bestScore = std::numeric_limits<int>::min();
        bool found = false;

        for (const auto& entry : std::filesystem::directory_iterator(cur, ec)) {
            if (ec) break;
            std::string name = entry.path().filename().string();
            if (asciiLower(name) != wanted) continue;
            int score = pathCaseScore(name);
            if (!found || score > bestScore) {
                found = true;
                bestScore = score;
                bestPath = entry.path();
            }
        }
        if (!found) return std::string();
        cur = bestPath;
    }

    if (!std::filesystem::exists(cur, ec) || std::filesystem::is_directory(cur, ec)) {
        return std::string();
    }
    return cur.string();
}

std::vector<uint8_t> readFileBinary(const std::string& fsPath) {
    std::ifstream in(fsPath, std::ios::binary);
    if (!in) return {};
    in.seekg(0, std::ios::end);
    std::streamoff size = in.tellg();
    if (size <= 0) return {};
    in.seekg(0, std::ios::beg);
    std::vector<uint8_t> data(static_cast<size_t>(size));
    in.read(reinterpret_cast<char*>(data.data()), size);
    if (!in) return {};
    return data;
}

bool hmacSha1Matches(const uint8_t seedBytes[4], const std::string& text, const uint8_t expected[20]) {
    uint8_t out[SHA_DIGEST_LENGTH];
    unsigned int outLen = 0;
    HMAC(EVP_sha1(),
         seedBytes, 4,
         reinterpret_cast<const uint8_t*>(text.data()),
         static_cast<int>(text.size()),
         out, &outLen);
    return outLen == SHA_DIGEST_LENGTH && std::memcmp(out, expected, SHA_DIGEST_LENGTH) == 0;
}

const std::unordered_map<std::string, std::array<uint8_t, 20>>& knownDoorHashes() {
    static const std::unordered_map<std::string, std::array<uint8_t, 20>> k = {
        {"world\\lordaeron\\stratholme\\activedoodads\\doors\\nox_door_plague.m2",
         {0xB4,0x45,0x2B,0x6D,0x95,0xC9,0x8B,0x18,0x6A,0x70,0xB0,0x08,0xFA,0x07,0xBB,0xAE,0xF3,0x0D,0xF7,0xA2}},
        {"world\\kalimdor\\onyxiaslair\\doors\\onyxiasgate01.m2",
         {0x75,0x19,0x5E,0x4A,0xED,0xA0,0xBC,0xAF,0x04,0x8C,0xA0,0xE3,0x4D,0x95,0xA7,0x0D,0x4F,0x53,0xC7,0x46}},
        {"world\\generic\\human\\activedoodads\\doors\\deadminedoor02.m2",
         {0x3D,0xFF,0x01,0x1B,0x9A,0xB1,0x34,0xF3,0x7F,0x88,0x50,0x97,0xE6,0x95,0x35,0x1B,0x91,0x95,0x35,0x64}},
        {"world\\kalimdor\\silithus\\activedoodads\\ahnqirajdoor\\ahnqirajdoor02.m2",
         {0xDB,0xD4,0xF4,0x07,0xC4,0x68,0xCC,0x36,0x13,0x4E,0x62,0x1D,0x16,0x01,0x78,0xFD,0xA4,0xD0,0xD2,0x49}},
        {"world\\kalimdor\\diremaul\\activedoodads\\doors\\diremaulsmallinstancedoor.m2",
         {0x0D,0xC8,0xDB,0x46,0xC8,0x55,0x49,0xC0,0xFF,0x1A,0x60,0x0F,0x6C,0x23,0x63,0x57,0xC3,0x05,0x78,0x1A}},
    };
    return k;
}

bool isReadableQuestText(const std::string& s, size_t minLen, size_t maxLen) {
    if (s.size() < minLen || s.size() > maxLen) return false;
    bool hasAlpha = false;
    for (unsigned char c : s) {
        if (c < 0x20 || c > 0x7E) return false;
        if (std::isalpha(c)) hasAlpha = true;
    }
    return hasAlpha;
}

bool isPlaceholderQuestTitle(const std::string& s) {
    return s.rfind("Quest #", 0) == 0;
}

bool looksLikeQuestDescriptionText(const std::string& s) {
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

bool isStrongQuestTitle(const std::string& s) {
    if (!isReadableQuestText(s, 6, 72)) return false;
    if (looksLikeQuestDescriptionText(s)) return false;
    unsigned char first = static_cast<unsigned char>(s.front());
    return std::isupper(first) != 0;
}

int scoreQuestTitle(const std::string& s) {
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

struct QuestQueryTextCandidate {
    std::string title;
    std::string objectives;
    int score = -1000;
};

QuestQueryTextCandidate pickBestQuestQueryTexts(const std::vector<uint8_t>& data, bool classicHint) {
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
                c.title = title;
                c.score = scoreQuestTitle(title) + 20; // Prefer expected struct offsets

                std::string s2;
                size_t n2 = next;
                if (readCStringAt(data, next, s2, n2) && isReadableQuestText(s2, 8, 600)) {
                    c.objectives = s2;
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
        c.title = title;
        c.score = scoreQuestTitle(title);
        if (c.score < 0) continue;

        std::string s2, s3;
        size_t n2 = next, n3 = next;
        if (readCStringAt(data, next, s2, n2)) {
            if (isReadableQuestText(s2, 8, 600)) c.objectives = s2;
            else if (readCStringAt(data, n2, s3, n3) && isReadableQuestText(s3, 8, 600)) c.objectives = s3;
        }
        if (c.score > best.score) best = c;
    }

    return best;
}
} // namespace


GameHandler::GameHandler() {
    LOG_DEBUG("GameHandler created");

    setActiveOpcodeTable(&opcodeTable_);
    setActiveUpdateFieldTable(&updateFieldTable_);

    // Initialize packet parsers (WotLK default, may be replaced for other expansions)
    packetParsers_ = std::make_unique<WotlkPacketParsers>();

    // Initialize transport manager
    transportManager_ = std::make_unique<TransportManager>();

    // Initialize Warden module manager
    wardenModuleManager_ = std::make_unique<WardenModuleManager>();

    // Default spells always available
    knownSpells.insert(6603);  // Attack
    knownSpells.insert(8690);  // Hearthstone

    // Default action bar layout
    actionBar[0].type = ActionBarSlot::SPELL;
    actionBar[0].id = 6603;   // Attack in slot 1
    actionBar[11].type = ActionBarSlot::SPELL;
    actionBar[11].id = 8690;  // Hearthstone in slot 12
}

GameHandler::~GameHandler() {
    disconnect();
}

void GameHandler::setPacketParsers(std::unique_ptr<PacketParsers> parsers) {
    packetParsers_ = std::move(parsers);
}

bool GameHandler::connect(const std::string& host,
                          uint16_t port,
                          const std::vector<uint8_t>& sessionKey,
                          const std::string& accountName,
                          uint32_t build,
                          uint32_t realmId) {

    if (sessionKey.size() != 40) {
        LOG_ERROR("Invalid session key size: ", sessionKey.size(), " (expected 40)");
        fail("Invalid session key");
        return false;
    }

    LOG_INFO("========================================");
    LOG_INFO("   CONNECTING TO WORLD SERVER");
    LOG_INFO("========================================");
    LOG_INFO("Host: ", host);
    LOG_INFO("Port: ", port);
    LOG_INFO("Account: ", accountName);
    LOG_INFO("Build: ", build);

    // Store authentication data
    this->sessionKey = sessionKey;
    this->accountName = accountName;
    this->build = build;
    this->realmId_ = realmId;

    // Diagnostic: dump session key for AUTH_REJECT debugging
    {
        std::string hex;
        for (uint8_t b : sessionKey) { char buf[4]; snprintf(buf, sizeof(buf), "%02x", b); hex += buf; }
        LOG_INFO("GameHandler session key (", sessionKey.size(), "): ", hex);
    }
    requiresWarden_ = false;
    wardenGateSeen_ = false;
    wardenGateElapsed_ = 0.0f;
    wardenGateNextStatusLog_ = 2.0f;
    wardenPacketsAfterGate_ = 0;
    wardenCharEnumBlockedLogged_ = false;
    wardenCrypto_.reset();
    wardenState_ = WardenState::WAIT_MODULE_USE;
    wardenModuleHash_.clear();
    wardenModuleKey_.clear();
    wardenModuleSize_ = 0;
    wardenModuleData_.clear();
    wardenLoadedModule_.reset();

    // Generate random client seed
    this->clientSeed = generateClientSeed();
    LOG_DEBUG("Generated client seed: 0x", std::hex, clientSeed, std::dec);

    // Create world socket
    socket = std::make_unique<network::WorldSocket>();

    // Set up packet callback
    socket->setPacketCallback([this](const network::Packet& packet) {
        network::Packet mutablePacket = packet;
        handlePacket(mutablePacket);
    });

    // Connect to world server
    setState(WorldState::CONNECTING);

    if (!socket->connect(host, port)) {
        LOG_ERROR("Failed to connect to world server");
        fail("Connection failed");
        return false;
    }

    setState(WorldState::CONNECTED);
    LOG_INFO("Connected to world server, waiting for SMSG_AUTH_CHALLENGE...");

    return true;
}

void GameHandler::disconnect() {
    if (onTaxiFlight_) {
        taxiRecoverPending_ = true;
    } else {
        taxiRecoverPending_ = false;
    }
    if (socket) {
        socket->disconnect();
        socket.reset();
    }
    activeCharacterGuid_ = 0;
    playerNameCache.clear();
    pendingNameQueries.clear();
    transportAttachments_.clear();
    serverUpdatedTransportGuids_.clear();
    requiresWarden_ = false;
    wardenGateSeen_ = false;
    wardenGateElapsed_ = 0.0f;
    wardenGateNextStatusLog_ = 2.0f;
    wardenPacketsAfterGate_ = 0;
    wardenCharEnumBlockedLogged_ = false;
    wardenCrypto_.reset();
    wardenState_ = WardenState::WAIT_MODULE_USE;
    wardenModuleHash_.clear();
    wardenModuleKey_.clear();
    wardenModuleSize_ = 0;
    wardenModuleData_.clear();
    wardenLoadedModule_.reset();
    setState(WorldState::DISCONNECTED);
    LOG_INFO("Disconnected from world server");
}

void GameHandler::resetDbcCaches() {
    spellNameCacheLoaded_ = false;
    spellNameCache_.clear();
    skillLineDbcLoaded_ = false;
    skillLineNames_.clear();
    skillLineCategories_.clear();
    skillLineAbilityLoaded_ = false;
    spellToSkillLine_.clear();
    taxiDbcLoaded_ = false;
    taxiNodes_.clear();
    taxiPathEdges_.clear();
    taxiPathNodes_.clear();
    talentDbcLoaded_ = false;
    talentCache_.clear();
    talentTabCache_.clear();
    LOG_INFO("GameHandler: DBC caches cleared for expansion switch");
}

bool GameHandler::isConnected() const {
    return socket && socket->isConnected();
}

void GameHandler::update(float deltaTime) {
    // Timing profiling (log every 60 frames to reduce spam)
    static int profileCounter = 0;
    static float socketTime = 0.0f;
    static float taxiTime = 0.0f;
    static float distanceCheckTime = 0.0f;
    static float entityUpdateTime = 0.0f;
    static float totalTime = 0.0f;

    auto updateStart = std::chrono::high_resolution_clock::now();

    // Fire deferred char-create callback (outside ImGui render)
    if (pendingCharCreateResult_) {
        pendingCharCreateResult_ = false;
        if (charCreateCallback_) {
            charCreateCallback_(pendingCharCreateSuccess_, pendingCharCreateMsg_);
        }
    }

    if (!socket) {
        return;
    }

    // Update socket (processes incoming data and triggers callbacks)
    auto socketStart = std::chrono::high_resolution_clock::now();
    if (socket) {
        socket->update();
    }
    auto socketEnd = std::chrono::high_resolution_clock::now();
    socketTime += std::chrono::duration<float, std::milli>(socketEnd - socketStart).count();

    // Detect server-side disconnect (socket closed during update)
    if (socket && !socket->isConnected() && state != WorldState::DISCONNECTED) {
        LOG_WARNING("Server closed connection in state: ", worldStateName(state));
        disconnect();
        return;
    }

    // Post-gate visibility: determine whether server goes silent or closes after Warden requirement.
    if (wardenGateSeen_ && socket && socket->isConnected()) {
        wardenGateElapsed_ += deltaTime;
        if (wardenGateElapsed_ >= wardenGateNextStatusLog_) {
            LOG_DEBUG("Warden gate status: elapsed=", wardenGateElapsed_,
                     "s connected=", socket->isConnected() ? "yes" : "no",
                     " packetsAfterGate=", wardenPacketsAfterGate_);
            wardenGateNextStatusLog_ += 30.0f;
        }
    }

    // Validate target still exists
    if (targetGuid != 0 && !entityManager.hasEntity(targetGuid)) {
        clearTarget();
    }

    if (auctionSearchDelayTimer_ > 0.0f) {
        auctionSearchDelayTimer_ -= deltaTime;
        if (auctionSearchDelayTimer_ < 0.0f) auctionSearchDelayTimer_ = 0.0f;
    }

    for (auto it = pendingQuestAcceptTimeouts_.begin(); it != pendingQuestAcceptTimeouts_.end();) {
        it->second -= deltaTime;
        if (it->second <= 0.0f) {
            const uint32_t questId = it->first;
            const uint64_t npcGuid = pendingQuestAcceptNpcGuids_.count(questId) != 0
                ? pendingQuestAcceptNpcGuids_[questId] : 0;
            triggerQuestAcceptResync(questId, npcGuid, "timeout");
            it = pendingQuestAcceptTimeouts_.erase(it);
            pendingQuestAcceptNpcGuids_.erase(questId);
        } else {
            ++it;
        }
    }

    if (pendingMoneyDeltaTimer_ > 0.0f) {
        pendingMoneyDeltaTimer_ -= deltaTime;
        if (pendingMoneyDeltaTimer_ <= 0.0f) {
            pendingMoneyDeltaTimer_ = 0.0f;
            pendingMoneyDelta_ = 0;
        }
    }
    if (autoAttackRangeWarnCooldown_ > 0.0f) {
        autoAttackRangeWarnCooldown_ = std::max(0.0f, autoAttackRangeWarnCooldown_ - deltaTime);
    }

    if (pendingLoginQuestResync_) {
        pendingLoginQuestResyncTimeout_ -= deltaTime;
        if (resyncQuestLogFromServerSlots(true)) {
            pendingLoginQuestResync_ = false;
            pendingLoginQuestResyncTimeout_ = 0.0f;
        } else if (pendingLoginQuestResyncTimeout_ <= 0.0f) {
            pendingLoginQuestResync_ = false;
            pendingLoginQuestResyncTimeout_ = 0.0f;
            LOG_WARNING("Quest login resync timed out waiting for player quest slot fields");
        }
    }

    for (auto it = pendingGameObjectLootRetries_.begin(); it != pendingGameObjectLootRetries_.end();) {
        it->timer -= deltaTime;
        if (it->timer <= 0.0f) {
            if (it->remainingRetries > 0 && state == WorldState::IN_WORLD && socket) {
                // Keep server-side position/facing fresh before retrying GO use.
                sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
                auto usePacket = GameObjectUsePacket::build(it->guid);
                socket->send(usePacket);
                if (it->sendLoot) {
                    auto lootPacket = LootPacket::build(it->guid);
                    socket->send(lootPacket);
                }
                --it->remainingRetries;
                it->timer = 0.20f;
            }
        }
        if (it->remainingRetries == 0) {
            it = pendingGameObjectLootRetries_.erase(it);
        } else {
            ++it;
        }
    }

    for (auto it = pendingGameObjectLootOpens_.begin(); it != pendingGameObjectLootOpens_.end();) {
        it->timer -= deltaTime;
        if (it->timer <= 0.0f) {
            if (state == WorldState::IN_WORLD && socket) {
                lootTarget(it->guid);
            }
            it = pendingGameObjectLootOpens_.erase(it);
        } else {
            ++it;
        }
    }

    if (pendingLootMoneyNotifyTimer_ > 0.0f) {
        pendingLootMoneyNotifyTimer_ -= deltaTime;
        if (pendingLootMoneyNotifyTimer_ <= 0.0f) {
            pendingLootMoneyNotifyTimer_ = 0.0f;
            bool alreadyAnnounced = false;
            if (pendingLootMoneyGuid_ != 0) {
                auto it = localLootState_.find(pendingLootMoneyGuid_);
                if (it != localLootState_.end()) {
                    alreadyAnnounced = it->second.moneyTaken;
                    it->second.moneyTaken = true;
                }
            }
            if (!alreadyAnnounced && pendingLootMoneyAmount_ > 0) {
                addSystemChatMessage("Looted: " + formatCopperAmount(pendingLootMoneyAmount_));
                auto* renderer = core::Application::getInstance().getRenderer();
                if (renderer) {
                    if (auto* sfx = renderer->getUiSoundManager()) {
                        if (pendingLootMoneyAmount_ >= 10000) {
                            sfx->playLootCoinLarge();
                        } else {
                            sfx->playLootCoinSmall();
                        }
                    }
                }
                if (pendingLootMoneyGuid_ != 0) {
                    recentLootMoneyAnnounceCooldowns_[pendingLootMoneyGuid_] = 1.5f;
                }
            }
            pendingLootMoneyGuid_ = 0;
            pendingLootMoneyAmount_ = 0;
        }
    }

    for (auto it = recentLootMoneyAnnounceCooldowns_.begin(); it != recentLootMoneyAnnounceCooldowns_.end();) {
        it->second -= deltaTime;
        if (it->second <= 0.0f) {
            it = recentLootMoneyAnnounceCooldowns_.erase(it);
        } else {
            ++it;
        }
    }

    // Auto-inspect throttling (fallback for player equipment visuals).
    if (inspectRateLimit_ > 0.0f) {
        inspectRateLimit_ = std::max(0.0f, inspectRateLimit_ - deltaTime);
    }
    if (state == WorldState::IN_WORLD && socket && inspectRateLimit_ <= 0.0f && !pendingAutoInspect_.empty()) {
        uint64_t guid = *pendingAutoInspect_.begin();
        pendingAutoInspect_.erase(pendingAutoInspect_.begin());
        if (guid != 0 && guid != playerGuid && entityManager.hasEntity(guid)) {
            auto pkt = InspectPacket::build(guid);
            socket->send(pkt);
            inspectRateLimit_ = 2.0f; // throttle to avoid compositing stutter
            LOG_DEBUG("Sent CMSG_INSPECT for player 0x", std::hex, guid, std::dec);
        }
    }

    // Send periodic heartbeat if in world
    if (state == WorldState::IN_WORLD) {
        timeSinceLastPing += deltaTime;
        timeSinceLastMoveHeartbeat_ += deltaTime;

        if (timeSinceLastPing >= pingInterval) {
            if (socket) {
                sendPing();
            }
            timeSinceLastPing = 0.0f;
        }

        const bool classicLikeCombatSync =
            autoAttackRequested_ && (isClassicLikeExpansion() || isActiveExpansion("tbc"));
        float heartbeatInterval = (onTaxiFlight_ || taxiActivatePending_ || taxiClientActive_)
                                      ? 0.25f
                                      : (classicLikeCombatSync ? 0.05f : moveHeartbeatInterval_);
        if (timeSinceLastMoveHeartbeat_ >= heartbeatInterval) {
            sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
            timeSinceLastMoveHeartbeat_ = 0.0f;
        }

        // Update cast timer (Phase 3)
        if (pendingGameObjectInteractGuid_ != 0 &&
            (autoAttacking || autoAttackRequested_)) {
            pendingGameObjectInteractGuid_ = 0;
            casting = false;
            currentCastSpellId = 0;
            castTimeRemaining = 0.0f;
            addSystemChatMessage("Interrupted.");
        }
        if (casting && castTimeRemaining > 0.0f) {
            castTimeRemaining -= deltaTime;
            if (castTimeRemaining <= 0.0f) {
                if (pendingGameObjectInteractGuid_ != 0) {
                    uint64_t interactGuid = pendingGameObjectInteractGuid_;
                    pendingGameObjectInteractGuid_ = 0;
                    performGameObjectInteractionNow(interactGuid);
                }
                casting = false;
                currentCastSpellId = 0;
                castTimeRemaining = 0.0f;
            }
        }

        // Update spell cooldowns (Phase 3)
        for (auto it = spellCooldowns.begin(); it != spellCooldowns.end(); ) {
            it->second -= deltaTime;
            if (it->second <= 0.0f) {
                it = spellCooldowns.erase(it);
            } else {
                ++it;
            }
        }

        // Update action bar cooldowns
        for (auto& slot : actionBar) {
            if (slot.cooldownRemaining > 0.0f) {
                slot.cooldownRemaining -= deltaTime;
                if (slot.cooldownRemaining < 0.0f) slot.cooldownRemaining = 0.0f;
            }
        }

        // Update combat text (Phase 2)
        updateCombatText(deltaTime);

        // Update taxi landing cooldown
        if (taxiLandingCooldown_ > 0.0f) {
            taxiLandingCooldown_ -= deltaTime;
        }
        if (taxiStartGrace_ > 0.0f) {
            taxiStartGrace_ -= deltaTime;
        }
        if (playerTransportStickyTimer_ > 0.0f) {
            playerTransportStickyTimer_ -= deltaTime;
            if (playerTransportStickyTimer_ <= 0.0f) {
                playerTransportStickyTimer_ = 0.0f;
                playerTransportStickyGuid_ = 0;
            }
        }

        // Taxi logic timing
        auto taxiStart = std::chrono::high_resolution_clock::now();

        // Detect taxi flight landing: UNIT_FLAG_TAXI_FLIGHT (0x00000100) cleared
        if (onTaxiFlight_) {
            updateClientTaxi(deltaTime);
            auto playerEntity = entityManager.getEntity(playerGuid);
            auto unit = std::dynamic_pointer_cast<Unit>(playerEntity);
            if (unit &&
                (unit->getUnitFlags() & 0x00000100) == 0 &&
                !taxiClientActive_ &&
                !taxiActivatePending_ &&
                taxiStartGrace_ <= 0.0f) {
                onTaxiFlight_ = false;
                taxiLandingCooldown_ = 2.0f;  // 2 second cooldown to prevent re-entering
                if (taxiMountActive_ && mountCallback_) {
                    mountCallback_(0);
                }
                taxiMountActive_ = false;
                taxiMountDisplayId_ = 0;
                currentMountDisplayId_ = 0;
                taxiClientActive_ = false;
                taxiClientPath_.clear();
                taxiRecoverPending_ = false;
                movementInfo.flags = 0;
                movementInfo.flags2 = 0;
                if (socket) {
                    sendMovement(Opcode::MSG_MOVE_STOP);
                    sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
                }
                LOG_INFO("Taxi flight landed");
            }
        }

        // Safety: if taxi flight ended but mount is still active, force dismount.
        // Guard against transient taxi-state flicker.
        if (!onTaxiFlight_ && taxiMountActive_) {
            bool serverStillTaxi = false;
            auto playerEntity = entityManager.getEntity(playerGuid);
            auto playerUnit = std::dynamic_pointer_cast<Unit>(playerEntity);
            if (playerUnit) {
                serverStillTaxi = (playerUnit->getUnitFlags() & 0x00000100) != 0;
            }

            if (taxiStartGrace_ > 0.0f || serverStillTaxi || taxiClientActive_ || taxiActivatePending_) {
                onTaxiFlight_ = true;
            } else {
                if (mountCallback_) mountCallback_(0);
                taxiMountActive_ = false;
                taxiMountDisplayId_ = 0;
                currentMountDisplayId_ = 0;
                movementInfo.flags = 0;
                movementInfo.flags2 = 0;
                if (socket) {
                    sendMovement(Opcode::MSG_MOVE_STOP);
                    sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
                }
                LOG_INFO("Taxi dismount cleanup");
            }
        }

        // Keep non-taxi mount state server-authoritative.
        // Some server paths don't emit explicit mount field updates in lockstep
        // with local visual state changes, so reconcile continuously.
        if (!onTaxiFlight_ && !taxiMountActive_) {
            auto playerEntity = entityManager.getEntity(playerGuid);
            auto playerUnit = std::dynamic_pointer_cast<Unit>(playerEntity);
            if (playerUnit) {
                uint32_t serverMountDisplayId = playerUnit->getMountDisplayId();
                if (serverMountDisplayId != currentMountDisplayId_) {
                    LOG_INFO("Mount reconcile: server=", serverMountDisplayId,
                             " local=", currentMountDisplayId_);
                    currentMountDisplayId_ = serverMountDisplayId;
                    if (mountCallback_) {
                        mountCallback_(serverMountDisplayId);
                    }
                }
            }
        }

        if (taxiRecoverPending_ && state == WorldState::IN_WORLD) {
            auto playerEntity = entityManager.getEntity(playerGuid);
            if (playerEntity) {
                playerEntity->setPosition(taxiRecoverPos_.x, taxiRecoverPos_.y,
                                          taxiRecoverPos_.z, movementInfo.orientation);
                movementInfo.x = taxiRecoverPos_.x;
                movementInfo.y = taxiRecoverPos_.y;
                movementInfo.z = taxiRecoverPos_.z;
                if (socket) {
                    sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
                }
                taxiRecoverPending_ = false;
                LOG_INFO("Taxi recovery applied");
            }
        }

        if (taxiActivatePending_) {
            taxiActivateTimer_ += deltaTime;
            if (taxiActivateTimer_ > 5.0f) {
                // If client taxi simulation is already active, server reply may be missing/late.
                // Do not cancel the flight in that case; clear pending state and continue.
                if (onTaxiFlight_ || taxiClientActive_ || taxiMountActive_) {
                    taxiActivatePending_ = false;
                    taxiActivateTimer_ = 0.0f;
                } else {
                taxiActivatePending_ = false;
                taxiActivateTimer_ = 0.0f;
                if (taxiMountActive_ && mountCallback_) {
                    mountCallback_(0);
                }
                taxiMountActive_ = false;
                taxiMountDisplayId_ = 0;
                taxiClientActive_ = false;
                taxiClientPath_.clear();
                onTaxiFlight_ = false;
                LOG_WARNING("Taxi activation timed out");
                }
            }
        }

        auto taxiEnd = std::chrono::high_resolution_clock::now();
        taxiTime += std::chrono::duration<float, std::milli>(taxiEnd - taxiStart).count();

        // Update transport manager
        if (transportManager_) {
            transportManager_->update(deltaTime);
            updateAttachedTransportChildren(deltaTime);
        }

        // Distance check timing
        auto distanceStart = std::chrono::high_resolution_clock::now();

        // Leave combat if auto-attack target is too far away (leash range)
        // and keep melee intent tightly synced while stationary.
        if (autoAttackRequested_ && autoAttackTarget != 0) {
            auto targetEntity = entityManager.getEntity(autoAttackTarget);
            if (targetEntity) {
                // Use latest server-authoritative target position to avoid stale
                // interpolation snapshots masking out-of-range states.
                const float targetX = targetEntity->getLatestX();
                const float targetY = targetEntity->getLatestY();
                const float targetZ = targetEntity->getLatestZ();
                float dx = movementInfo.x - targetX;
                float dy = movementInfo.y - targetY;
                float dz = movementInfo.z - targetZ;
                float dist = std::sqrt(dx * dx + dy * dy);
                float dist3d = std::sqrt(dx * dx + dy * dy + dz * dz);
                const bool classicLike = isClassicLikeExpansion() || isActiveExpansion("tbc");
                if (dist > 40.0f) {
                    stopAutoAttack();
                    LOG_INFO("Left combat: target too far (", dist, " yards)");
                } else if (state == WorldState::IN_WORLD && socket) {
                    bool allowResync = true;
                    const float meleeRange = classicLike ? 5.25f : 5.75f;
                    if (dist3d > meleeRange) {
                        autoAttackOutOfRange_ = true;
                        autoAttackOutOfRangeTime_ += deltaTime;
                        if (autoAttackRangeWarnCooldown_ <= 0.0f) {
                            addSystemChatMessage("Target is too far away.");
                            autoAttackRangeWarnCooldown_ = 1.25f;
                        }
                        // Stop chasing stale swings when the target remains out of range.
                        if (autoAttackOutOfRangeTime_ > 2.0f && dist3d > 9.0f) {
                            stopAutoAttack();
                            addSystemChatMessage("Auto-attack stopped: target out of range.");
                            allowResync = false;
                        }
                    } else {
                        autoAttackOutOfRange_ = false;
                        autoAttackOutOfRangeTime_ = 0.0f;
                    }

                    if (allowResync) {
                        autoAttackResendTimer_ += deltaTime;
                        autoAttackFacingSyncTimer_ += deltaTime;

                        // Re-request swing more aggressively until server confirms active loop.
                        float resendInterval = 1.0f;
                        if (!autoAttacking || autoAttackOutOfRange_) {
                            resendInterval = classicLike ? 0.25f : 0.50f;
                        }
                        if (autoAttackResendTimer_ >= resendInterval) {
                            autoAttackResendTimer_ = 0.0f;
                            auto pkt = AttackSwingPacket::build(autoAttackTarget);
                            socket->send(pkt);
                        }

                        // Keep server-facing aligned with our current melee target.
                        // Some vanilla-family realms become strict about front-arc checks unless
                        // the client sends explicit facing updates while stationary.
                        const float facingSyncInterval = classicLike ? 0.10f : 0.20f;
                        if (autoAttackFacingSyncTimer_ >= facingSyncInterval) {
                            autoAttackFacingSyncTimer_ = 0.0f;
                            float toTargetX = targetX - movementInfo.x;
                            float toTargetY = targetY - movementInfo.y;
                            bool sentMovement = false;
                            if (std::abs(toTargetX) > 0.01f || std::abs(toTargetY) > 0.01f) {
                                float desired = std::atan2(-toTargetY, toTargetX);
                                float diff = desired - movementInfo.orientation;
                                while (diff > static_cast<float>(M_PI)) diff -= 2.0f * static_cast<float>(M_PI);
                                while (diff < -static_cast<float>(M_PI)) diff += 2.0f * static_cast<float>(M_PI);
                                const float facingThreshold = classicLike ? 0.035f : 0.12f; // ~2deg / ~7deg
                                if (std::abs(diff) > facingThreshold) {
                                    movementInfo.orientation = desired;
                                    sendMovement(Opcode::MSG_MOVE_SET_FACING);
                                    // Follow facing update with a heartbeat to tighten server range/facing checks.
                                    sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
                                    sentMovement = true;
                                }
                            } else if (classicLike) {
                                // Keep stationary melee position/facing fresh for strict vanilla-family checks.
                                sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
                                sentMovement = true;
                            }

                            // Even when facing is already correct, keep position fresh while
                            // trying to connect melee hits so servers don't require a step.
                            if (!sentMovement && (!autoAttacking || autoAttackOutOfRange_)) {
                                sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
                            }
                        }
                    }
                }
            }
        }

        // Keep active melee attackers visually facing the player as positions change.
        // Some servers don't stream frequent orientation updates during combat.
        if (!hostileAttackers_.empty()) {
            for (uint64_t attackerGuid : hostileAttackers_) {
                auto attacker = entityManager.getEntity(attackerGuid);
                if (!attacker) continue;
                float dx = movementInfo.x - attacker->getX();
                float dy = movementInfo.y - attacker->getY();
                if (std::abs(dx) < 0.01f && std::abs(dy) < 0.01f) continue;
                attacker->setOrientation(std::atan2(-dy, dx));
            }
        }

        // Close vendor/gossip/taxi window if player walks too far from NPC
        if (vendorWindowOpen && currentVendorItems.vendorGuid != 0) {
            auto npc = entityManager.getEntity(currentVendorItems.vendorGuid);
            if (npc) {
                float dx = movementInfo.x - npc->getX();
                float dy = movementInfo.y - npc->getY();
                float dist = std::sqrt(dx * dx + dy * dy);
                if (dist > 15.0f) {
                    closeVendor();
                    LOG_INFO("Vendor closed: walked too far from NPC");
                }
            }
        }
        if (gossipWindowOpen && currentGossip.npcGuid != 0) {
            auto npc = entityManager.getEntity(currentGossip.npcGuid);
            if (npc) {
                float dx = movementInfo.x - npc->getX();
                float dy = movementInfo.y - npc->getY();
                float dist = std::sqrt(dx * dx + dy * dy);
                if (dist > 15.0f) {
                    closeGossip();
                    LOG_INFO("Gossip closed: walked too far from NPC");
                }
            }
        }
        if (taxiWindowOpen_ && taxiNpcGuid_ != 0) {
            auto npc = entityManager.getEntity(taxiNpcGuid_);
            if (npc) {
                float dx = movementInfo.x - npc->getX();
                float dy = movementInfo.y - npc->getY();
                float dist = std::sqrt(dx * dx + dy * dy);
                if (dist > 15.0f) {
                    closeTaxi();
                    LOG_INFO("Taxi window closed: walked too far from NPC");
                }
            }
        }
        if (trainerWindowOpen_ && currentTrainerList_.trainerGuid != 0) {
            auto npc = entityManager.getEntity(currentTrainerList_.trainerGuid);
            if (npc) {
                float dx = movementInfo.x - npc->getX();
                float dy = movementInfo.y - npc->getY();
                float dist = std::sqrt(dx * dx + dy * dy);
                if (dist > 15.0f) {
                    closeTrainer();
                    LOG_INFO("Trainer closed: walked too far from NPC");
                }
            }
        }

        auto distanceEnd = std::chrono::high_resolution_clock::now();
        distanceCheckTime += std::chrono::duration<float, std::milli>(distanceEnd - distanceStart).count();

        // Entity update timing
        auto entityStart = std::chrono::high_resolution_clock::now();

        // Update entity movement interpolation (keeps targeting in sync with visuals)
        // Only update entities within reasonable distance for performance
        const float updateRadiusSq = 150.0f * 150.0f;  // 150 unit radius
        auto playerEntity = entityManager.getEntity(playerGuid);
        glm::vec3 playerPos = playerEntity ? glm::vec3(playerEntity->getX(), playerEntity->getY(), playerEntity->getZ()) : glm::vec3(0.0f);

        for (auto& [guid, entity] : entityManager.getEntities()) {
            // Always update player
            if (guid == playerGuid) {
                entity->updateMovement(deltaTime);
                continue;
            }
            // Keep selected/engaged target interpolation exact for UI targeting circle.
            if (guid == targetGuid || guid == autoAttackTarget) {
                entity->updateMovement(deltaTime);
                continue;
            }

            // Distance cull other entities (use latest position to avoid culling by stale origin)
            glm::vec3 entityPos(entity->getLatestX(), entity->getLatestY(), entity->getLatestZ());
            float distSq = glm::dot(entityPos - playerPos, entityPos - playerPos);
            if (distSq < updateRadiusSq) {
                entity->updateMovement(deltaTime);
            }
        }

        auto entityEnd = std::chrono::high_resolution_clock::now();
        entityUpdateTime += std::chrono::duration<float, std::milli>(entityEnd - entityStart).count();
    }

    auto updateEnd = std::chrono::high_resolution_clock::now();
    totalTime += std::chrono::duration<float, std::milli>(updateEnd - updateStart).count();

    // Log profiling every 60 frames
    if (++profileCounter >= 60) {
        LOG_DEBUG("UPDATE PROFILE (60 frames): socket=", socketTime / 60.0f, "ms taxi=", taxiTime / 60.0f,
                 "ms distance=", distanceCheckTime / 60.0f, "ms entity=", entityUpdateTime / 60.0f,
                 "ms TOTAL=", totalTime / 60.0f, "ms");
        profileCounter = 0;
        socketTime = 0.0f;
        taxiTime = 0.0f;
        distanceCheckTime = 0.0f;
        entityUpdateTime = 0.0f;
        totalTime = 0.0f;
    }
}

void GameHandler::handlePacket(network::Packet& packet) {
    if (packet.getSize() < 1) {
        LOG_DEBUG("Received empty world packet (ignored)");
        return;
    }

    uint16_t opcode = packet.getOpcode();

    // Vanilla compatibility aliases:
    // - 0x006B: can be SMSG_COMPRESSED_MOVES on some vanilla-family servers
    //           and SMSG_WEATHER on others
    // - 0x0103: SMSG_PLAY_MUSIC (some vanilla-family servers)
    //
    // We gate these by payload shape so expansion-native mappings remain intact.
    if (opcode == 0x006B) {
        // Try compressed movement batch first:
        // [u8 subSize][u16 subOpcode][subPayload...] ...
        // where subOpcode is typically SMSG_MONSTER_MOVE / SMSG_MONSTER_MOVE_TRANSPORT.
        const auto& data = packet.getData();
        if (packet.getReadPos() + 3 <= data.size()) {
            size_t pos = packet.getReadPos();
            uint8_t subSize = data[pos];
            if (subSize >= 2 && pos + 1 + subSize <= data.size()) {
                uint16_t subOpcode = static_cast<uint16_t>(data[pos + 1]) |
                                     (static_cast<uint16_t>(data[pos + 2]) << 8);
                uint16_t monsterMoveWire = wireOpcode(Opcode::SMSG_MONSTER_MOVE);
                uint16_t monsterMoveTransportWire = wireOpcode(Opcode::SMSG_MONSTER_MOVE_TRANSPORT);
                if ((monsterMoveWire != 0xFFFF && subOpcode == monsterMoveWire) ||
                    (monsterMoveTransportWire != 0xFFFF && subOpcode == monsterMoveTransportWire)) {
                    LOG_INFO("Opcode 0x006B interpreted as SMSG_COMPRESSED_MOVES (subOpcode=0x",
                             std::hex, subOpcode, std::dec, ")");
                    handleCompressedMoves(packet);
                    return;
                }
            }
        }

        // Expected weather payload: uint32 weatherType, float intensity, uint8 abrupt
        if (packet.getSize() - packet.getReadPos() >= 9) {
            uint32_t wType = packet.readUInt32();
            float wIntensity = packet.readFloat();
            uint8_t abrupt = packet.readUInt8();
            bool plausibleWeather =
                (wType <= 3) &&
                std::isfinite(wIntensity) &&
                (wIntensity >= 0.0f && wIntensity <= 1.5f) &&
                (abrupt <= 1);
            if (plausibleWeather) {
                weatherType_ = wType;
                weatherIntensity_ = wIntensity;
                const char* typeName =
                    (wType == 1) ? "Rain" :
                    (wType == 2) ? "Snow" :
                    (wType == 3) ? "Storm" : "Clear";
                LOG_INFO("Weather changed (0x006B alias): type=", wType,
                         " (", typeName, "), intensity=", wIntensity,
                         ", abrupt=", static_cast<int>(abrupt));
                return;
            }
            // Not weather-shaped: rewind and fall through to normal opcode table handling.
            packet.setReadPos(0);
        }
    } else if (opcode == 0x0103) {
        // Expected play-music payload: uint32 sound/music id
        if (packet.getSize() - packet.getReadPos() == 4) {
            uint32_t soundId = packet.readUInt32();
            LOG_INFO("SMSG_PLAY_MUSIC (0x0103 alias): soundId=", soundId);
            return;
        }
    } else if (opcode == 0x0480) {
        // Observed on this WotLK profile immediately after CMSG_BUYBACK_ITEM.
        // Treat as vendor/buyback transaction result (7-byte payload on this core).
        if (packet.getSize() - packet.getReadPos() >= 7) {
            uint8_t opType = packet.readUInt8();
            uint8_t resultCode = packet.readUInt8();
            uint8_t slotOrCount = packet.readUInt8();
            uint32_t itemId = packet.readUInt32();
            LOG_INFO("Vendor txn result (0x480): opType=", static_cast<int>(opType),
                     " result=", static_cast<int>(resultCode),
                     " slot/count=", static_cast<int>(slotOrCount),
                     " itemId=", itemId,
                     " pendingBuybackSlot=", pendingBuybackSlot_,
                     " pendingBuyItemId=", pendingBuyItemId_,
                     " pendingBuyItemSlot=", pendingBuyItemSlot_);

            if (pendingBuybackSlot_ >= 0) {
                if (resultCode == 0) {
                    // Success: remove the bought-back slot from our local UI cache.
                    if (pendingBuybackSlot_ < static_cast<int>(buybackItems_.size())) {
                        buybackItems_.erase(buybackItems_.begin() + pendingBuybackSlot_);
                    }
                } else {
                    const char* msg = "Buyback failed.";
                    // Best-effort mapping; keep raw code visible for unknowns.
                    switch (resultCode) {
                        case 2: msg = "Buyback failed: not enough money."; break;
                        case 4: msg = "Buyback failed: vendor too far away."; break;
                        case 5: msg = "Buyback failed: item unavailable."; break;
                        case 6: msg = "Buyback failed: inventory full."; break;
                        case 8: msg = "Buyback failed: requirements not met."; break;
                        default: break;
                    }
                    addSystemChatMessage(std::string(msg) + " (code " + std::to_string(resultCode) + ")");
                }
                pendingBuybackSlot_ = -1;
                pendingBuybackWireSlot_ = 0;

                // Refresh vendor list so UI state stays in sync after buyback result.
                if (currentVendorItems.vendorGuid != 0 && socket && state == WorldState::IN_WORLD) {
                    auto pkt = ListInventoryPacket::build(currentVendorItems.vendorGuid);
                    socket->send(pkt);
                }
            } else if (pendingBuyItemId_ != 0) {
                if (resultCode != 0) {
                    const char* msg = "Purchase failed.";
                    switch (resultCode) {
                        case 2: msg = "Purchase failed: not enough money."; break;
                        case 4: msg = "Purchase failed: vendor too far away."; break;
                        case 5: msg = "Purchase failed: item sold out."; break;
                        case 6: msg = "Purchase failed: inventory full."; break;
                        case 8: msg = "Purchase failed: requirements not met."; break;
                        default: break;
                    }
                    addSystemChatMessage(std::string(msg) + " (code " + std::to_string(resultCode) + ")");
                }
                pendingBuyItemId_ = 0;
                pendingBuyItemSlot_ = 0;
            }
            return;
        }
    } else if (opcode == 0x046A) {
        // Server-specific vendor/buyback state packet (observed 25-byte records).
        // Consume to keep stream aligned; currently not used for gameplay logic.
        if (packet.getSize() - packet.getReadPos() >= 25) {
            packet.setReadPos(packet.getReadPos() + 25);
            return;
        }
    }

    auto preLogicalOp = opcodeTable_.fromWire(opcode);
    if (wardenGateSeen_ && (!preLogicalOp || *preLogicalOp != Opcode::SMSG_WARDEN_DATA)) {
        ++wardenPacketsAfterGate_;
    }
    if (preLogicalOp && isAuthCharPipelineOpcode(*preLogicalOp)) {
        LOG_INFO("AUTH/CHAR RX opcode=0x", std::hex, opcode, std::dec,
                 " state=", worldStateName(state),
                 " size=", packet.getSize());
    }

    LOG_DEBUG("Received world packet: opcode=0x", std::hex, opcode, std::dec,
              " size=", packet.getSize(), " bytes");

    // Translate wire opcode to logical opcode via expansion table
    auto logicalOp = opcodeTable_.fromWire(opcode);

    if (!logicalOp) {
        static std::unordered_set<uint16_t> loggedUnknownWireOpcodes;
        if (loggedUnknownWireOpcodes.insert(opcode).second) {
            LOG_WARNING("Unhandled world opcode: 0x", std::hex, opcode, std::dec,
                        " state=", static_cast<int>(state),
                        " size=", packet.getSize());
        }
        return;
    }

    switch (*logicalOp) {
        case Opcode::SMSG_AUTH_CHALLENGE:
            if (state == WorldState::CONNECTED) {
                handleAuthChallenge(packet);
            } else {
                LOG_WARNING("Unexpected SMSG_AUTH_CHALLENGE in state: ", worldStateName(state));
            }
            break;

        case Opcode::SMSG_AUTH_RESPONSE:
            if (state == WorldState::AUTH_SENT) {
                handleAuthResponse(packet);
            } else {
                LOG_WARNING("Unexpected SMSG_AUTH_RESPONSE in state: ", worldStateName(state));
            }
            break;

        case Opcode::SMSG_CHAR_CREATE:
            handleCharCreateResponse(packet);
            break;

        case Opcode::SMSG_CHAR_DELETE: {
            uint8_t result = packet.readUInt8();
            lastCharDeleteResult_ = result;
            bool success = (result == 0x00 || result == 0x47); // Common success codes
            LOG_INFO("SMSG_CHAR_DELETE result: ", (int)result, success ? " (success)" : " (failed)");
            requestCharacterList();
            if (charDeleteCallback_) charDeleteCallback_(success);
            break;
        }

        case Opcode::SMSG_CHAR_ENUM:
            if (state == WorldState::CHAR_LIST_REQUESTED) {
                handleCharEnum(packet);
            } else {
                LOG_WARNING("Unexpected SMSG_CHAR_ENUM in state: ", worldStateName(state));
            }
            break;

        case Opcode::SMSG_CHARACTER_LOGIN_FAILED:
            handleCharLoginFailed(packet);
            break;

        case Opcode::SMSG_LOGIN_VERIFY_WORLD:
            if (state == WorldState::ENTERING_WORLD || state == WorldState::IN_WORLD) {
                handleLoginVerifyWorld(packet);
            } else {
                LOG_WARNING("Unexpected SMSG_LOGIN_VERIFY_WORLD in state: ", worldStateName(state));
            }
            break;

        case Opcode::SMSG_LOGIN_SETTIMESPEED:
            // Can be received during login or at any time after
            handleLoginSetTimeSpeed(packet);
            break;

        case Opcode::SMSG_CLIENTCACHE_VERSION:
            // Early pre-world packet in some realms (e.g. Warmane profile)
            handleClientCacheVersion(packet);
            break;

        case Opcode::SMSG_TUTORIAL_FLAGS:
            // Often sent during char-list stage (8x uint32 tutorial flags)
            handleTutorialFlags(packet);
            break;

        case Opcode::SMSG_WARDEN_DATA:
            handleWardenData(packet);
            break;

        case Opcode::SMSG_ACCOUNT_DATA_TIMES:
            // Can be received at any time after authentication
            handleAccountDataTimes(packet);
            break;

        case Opcode::SMSG_MOTD:
            // Can be received at any time after entering world
            handleMotd(packet);
            break;

        case Opcode::SMSG_NOTIFICATION:
            // Vanilla/Classic server notification (single string)
            handleNotification(packet);
            break;

        case Opcode::SMSG_PONG:
            // Can be received at any time after entering world
            handlePong(packet);
            break;

        case Opcode::SMSG_UPDATE_OBJECT:
            LOG_DEBUG("Received SMSG_UPDATE_OBJECT, state=", static_cast<int>(state), " size=", packet.getSize());
            // Can be received after entering world
            if (state == WorldState::IN_WORLD) {
                handleUpdateObject(packet);
            }
            break;

        case Opcode::SMSG_COMPRESSED_UPDATE_OBJECT:
            LOG_DEBUG("Received SMSG_COMPRESSED_UPDATE_OBJECT, state=", static_cast<int>(state), " size=", packet.getSize());
            // Compressed version of UPDATE_OBJECT
            if (state == WorldState::IN_WORLD) {
                handleCompressedUpdateObject(packet);
            }
            break;
        case Opcode::SMSG_DESTROY_OBJECT:
            // Can be received after entering world
            if (state == WorldState::IN_WORLD) {
                handleDestroyObject(packet);
            }
            break;

        case Opcode::SMSG_MESSAGECHAT:
            // Can be received after entering world
            if (state == WorldState::IN_WORLD) {
                handleMessageChat(packet);
            }
            break;

        case Opcode::SMSG_TEXT_EMOTE:
            if (state == WorldState::IN_WORLD) {
                handleTextEmote(packet);
            }
            break;
        case Opcode::SMSG_EMOTE: {
            if (state != WorldState::IN_WORLD) break;
            // SMSG_EMOTE: uint32 emoteAnim, uint64 sourceGuid
            if (packet.getSize() - packet.getReadPos() < 12) break;
            uint32_t emoteAnim = packet.readUInt32();
            uint64_t sourceGuid = packet.readUInt64();
            if (emoteAnimCallback_ && sourceGuid != 0) {
                emoteAnimCallback_(sourceGuid, emoteAnim);
            }
            break;
        }

        case Opcode::SMSG_CHANNEL_NOTIFY:
            // Accept during ENTERING_WORLD too — server auto-joins channels before VERIFY_WORLD
            if (state == WorldState::IN_WORLD || state == WorldState::ENTERING_WORLD) {
                handleChannelNotify(packet);
            }
            break;

        case Opcode::SMSG_QUERY_TIME_RESPONSE:
            if (state == WorldState::IN_WORLD) {
                handleQueryTimeResponse(packet);
            }
            break;

        case Opcode::SMSG_PLAYED_TIME:
            if (state == WorldState::IN_WORLD) {
                handlePlayedTime(packet);
            }
            break;

        case Opcode::SMSG_WHO:
            if (state == WorldState::IN_WORLD) {
                handleWho(packet);
            }
            break;

        case Opcode::SMSG_FRIEND_STATUS:
            if (state == WorldState::IN_WORLD) {
                handleFriendStatus(packet);
            }
            break;
        case Opcode::SMSG_CONTACT_LIST: {
            // Known variants:
            // - Full form: uint32 listMask, uint32 count, then variable-size entries.
            // - Minimal/legacy keepalive-ish form observed on some servers: 1 byte.
            size_t remaining = packet.getSize() - packet.getReadPos();
            if (remaining >= 8) {
                lastContactListMask_ = packet.readUInt32();
                lastContactListCount_ = packet.readUInt32();
            } else if (remaining == 1) {
                /*uint8_t marker =*/ packet.readUInt8();
                lastContactListMask_ = 0;
                lastContactListCount_ = 0;
            } else if (remaining > 0) {
                // Unknown short variant: consume to keep stream aligned, no warning spam.
                packet.setReadPos(packet.getSize());
            }
            break;
        }
        case Opcode::SMSG_FRIEND_LIST:
        case Opcode::SMSG_IGNORE_LIST:
            // Legacy social list variants; CONTACT_LIST is primary in modern flow.
            packet.setReadPos(packet.getSize());
            break;

        case Opcode::MSG_RANDOM_ROLL:
            if (state == WorldState::IN_WORLD) {
                handleRandomRoll(packet);
            }
            break;
        case Opcode::SMSG_ITEM_PUSH_RESULT:
            // Item received notification (new item in bags, loot, quest reward, etc.)
            // TODO: parse and show "item received" UI notification
            packet.setReadPos(packet.getSize());
            break;

        case Opcode::SMSG_LOGOUT_RESPONSE:
            handleLogoutResponse(packet);
            break;

        case Opcode::SMSG_LOGOUT_COMPLETE:
            handleLogoutComplete(packet);
            break;

        // ---- Phase 1: Foundation ----
        case Opcode::SMSG_NAME_QUERY_RESPONSE:
            handleNameQueryResponse(packet);
            break;

        case Opcode::SMSG_CREATURE_QUERY_RESPONSE:
            handleCreatureQueryResponse(packet);
            break;

        case Opcode::SMSG_ITEM_QUERY_SINGLE_RESPONSE:
            handleItemQueryResponse(packet);
            break;

        case Opcode::SMSG_INSPECT_TALENT:
            handleInspectResults(packet);
            break;
        case Opcode::SMSG_ADDON_INFO:
        case Opcode::SMSG_EXPECTED_SPAM_RECORDS:
            // Optional system payloads that are safe to consume.
            packet.setReadPos(packet.getSize());
            break;

        // ---- XP ----
        case Opcode::SMSG_LOG_XPGAIN:
            handleXpGain(packet);
            break;

        // ---- Creature Movement ----
        case Opcode::SMSG_MONSTER_MOVE:
            handleMonsterMove(packet);
            break;

        case Opcode::SMSG_COMPRESSED_MOVES:
            handleCompressedMoves(packet);
            break;

        case Opcode::SMSG_MONSTER_MOVE_TRANSPORT:
            handleMonsterMoveTransport(packet);
            break;
        case Opcode::SMSG_SPLINE_MOVE_SET_WALK_MODE:
        case Opcode::SMSG_SPLINE_MOVE_SET_RUN_MODE: {
            // Minimal parse: PackedGuid
            if (packet.getSize() - packet.getReadPos() >= 1) {
                (void)UpdateObjectParser::readPackedGuid(packet);
            }
            break;
        }
        case Opcode::SMSG_SPLINE_SET_RUN_SPEED:
        case Opcode::SMSG_SPLINE_SET_RUN_BACK_SPEED:
        case Opcode::SMSG_SPLINE_SET_SWIM_SPEED: {
            // Minimal parse: PackedGuid + float speed
            if (packet.getSize() - packet.getReadPos() < 5) break;
            uint64_t guid = UpdateObjectParser::readPackedGuid(packet);
            if (packet.getSize() - packet.getReadPos() < 4) break;
            float speed = packet.readFloat();
            if (guid == playerGuid && std::isfinite(speed) && speed > 0.1f && speed < 100.0f &&
                *logicalOp == Opcode::SMSG_SPLINE_SET_RUN_SPEED) {
                serverRunSpeed_ = speed;
            }
            break;
        }

        // ---- Speed Changes ----
        case Opcode::SMSG_FORCE_RUN_SPEED_CHANGE:
            handleForceRunSpeedChange(packet);
            break;
        case Opcode::SMSG_FORCE_MOVE_ROOT:
            handleForceMoveRootState(packet, true);
            break;
        case Opcode::SMSG_FORCE_MOVE_UNROOT:
            handleForceMoveRootState(packet, false);
            break;
        case Opcode::SMSG_CLIENT_CONTROL_UPDATE: {
            // Minimal parse: PackedGuid + uint8 allowMovement.
            if (packet.getSize() - packet.getReadPos() < 2) {
                LOG_WARNING("SMSG_CLIENT_CONTROL_UPDATE too short: ", packet.getSize(), " bytes");
                break;
            }
            uint8_t guidMask = packet.readUInt8();
            size_t guidBytes = 0;
            uint64_t controlGuid = 0;
            for (int i = 0; i < 8; ++i) {
                if (guidMask & (1u << i)) ++guidBytes;
            }
            if (packet.getSize() - packet.getReadPos() < guidBytes + 1) {
                LOG_WARNING("SMSG_CLIENT_CONTROL_UPDATE malformed (truncated packed guid)");
                packet.setReadPos(packet.getSize());
                break;
            }
            for (int i = 0; i < 8; ++i) {
                if (guidMask & (1u << i)) {
                    uint8_t b = packet.readUInt8();
                    controlGuid |= (static_cast<uint64_t>(b) << (i * 8));
                }
            }
            bool allowMovement = (packet.readUInt8() != 0);
            if (controlGuid == 0 || controlGuid == playerGuid) {
                bool changed = (serverMovementAllowed_ != allowMovement);
                serverMovementAllowed_ = allowMovement;
                if (changed && !allowMovement) {
                    // Force-stop local movement immediately when server revokes control.
                    movementInfo.flags &= ~(static_cast<uint32_t>(MovementFlags::FORWARD) |
                                            static_cast<uint32_t>(MovementFlags::BACKWARD) |
                                            static_cast<uint32_t>(MovementFlags::STRAFE_LEFT) |
                                            static_cast<uint32_t>(MovementFlags::STRAFE_RIGHT) |
                                            static_cast<uint32_t>(MovementFlags::TURN_LEFT) |
                                            static_cast<uint32_t>(MovementFlags::TURN_RIGHT));
                    sendMovement(Opcode::MSG_MOVE_STOP);
                    sendMovement(Opcode::MSG_MOVE_STOP_STRAFE);
                    sendMovement(Opcode::MSG_MOVE_STOP_TURN);
                    sendMovement(Opcode::MSG_MOVE_STOP_SWIM);
                    addSystemChatMessage("Movement disabled by server.");
                } else if (changed && allowMovement) {
                    addSystemChatMessage("Movement re-enabled.");
                }
            }
            break;
        }

        // ---- Phase 2: Combat ----
        case Opcode::SMSG_ATTACKSTART:
            handleAttackStart(packet);
            break;
        case Opcode::SMSG_ATTACKSTOP:
            handleAttackStop(packet);
            break;
        case Opcode::SMSG_ATTACKSWING_NOTINRANGE:
            autoAttackOutOfRange_ = true;
            if (autoAttackRangeWarnCooldown_ <= 0.0f) {
                addSystemChatMessage("Target is too far away.");
                autoAttackRangeWarnCooldown_ = 1.25f;
            }
            if (autoAttackRequested_ && autoAttackTarget != 0 && socket) {
                // Avoid blind immediate resend loops when target is clearly out of melee range.
                bool likelyInRange = true;
                if (auto target = entityManager.getEntity(autoAttackTarget)) {
                    float dx = movementInfo.x - target->getLatestX();
                    float dy = movementInfo.y - target->getLatestY();
                    float dz = movementInfo.z - target->getLatestZ();
                    float dist3d = std::sqrt(dx * dx + dy * dy + dz * dz);
                    likelyInRange = (dist3d <= 7.5f);
                }
                if (likelyInRange) {
                    auto pkt = AttackSwingPacket::build(autoAttackTarget);
                    socket->send(pkt);
                }
            }
            break;
        case Opcode::SMSG_ATTACKSWING_BADFACING:
            if (autoAttackRequested_ && autoAttackTarget != 0) {
                auto targetEntity = entityManager.getEntity(autoAttackTarget);
                if (targetEntity) {
                    float toTargetX = targetEntity->getX() - movementInfo.x;
                    float toTargetY = targetEntity->getY() - movementInfo.y;
                    if (std::abs(toTargetX) > 0.01f || std::abs(toTargetY) > 0.01f) {
                        movementInfo.orientation = std::atan2(-toTargetY, toTargetX);
                        sendMovement(Opcode::MSG_MOVE_SET_FACING);
                        sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
                    }
                }
                if (socket) {
                    auto pkt = AttackSwingPacket::build(autoAttackTarget);
                    socket->send(pkt);
                }
            }
            break;
        case Opcode::SMSG_ATTACKSWING_NOTSTANDING:
        case Opcode::SMSG_ATTACKSWING_CANT_ATTACK:
            autoAttackOutOfRange_ = false;
            autoAttackOutOfRangeTime_ = 0.0f;
            break;
        case Opcode::SMSG_ATTACKERSTATEUPDATE:
            handleAttackerStateUpdate(packet);
            break;
        case Opcode::SMSG_AI_REACTION: {
            // SMSG_AI_REACTION: uint64 guid, uint32 reaction
            if (packet.getSize() - packet.getReadPos() < 12) break;
            uint64_t guid = packet.readUInt64();
            uint32_t reaction = packet.readUInt32();
            // Reaction 2 commonly indicates aggro.
            if (reaction == 2 && npcAggroCallback_) {
                auto entity = entityManager.getEntity(guid);
                if (entity) {
                    npcAggroCallback_(guid, glm::vec3(entity->getX(), entity->getY(), entity->getZ()));
                }
            }
            break;
        }
        case Opcode::SMSG_SPELLNONMELEEDAMAGELOG:
            handleSpellDamageLog(packet);
            break;
        case Opcode::SMSG_PLAY_SPELL_VISUAL: {
            // Minimal parse: uint64 casterGuid, uint32 visualId
            if (packet.getSize() - packet.getReadPos() < 12) break;
            packet.readUInt64();
            packet.readUInt32();
            break;
        }
        case Opcode::SMSG_SPELLHEALLOG:
            handleSpellHealLog(packet);
            break;

        // ---- Phase 3: Spells ----
        case Opcode::SMSG_INITIAL_SPELLS:
            handleInitialSpells(packet);
            break;
        case Opcode::SMSG_CAST_FAILED:
            handleCastFailed(packet);
            break;
        case Opcode::SMSG_SPELL_START:
            handleSpellStart(packet);
            break;
        case Opcode::SMSG_SPELL_GO:
            handleSpellGo(packet);
            break;
        case Opcode::SMSG_SPELL_FAILURE:
            // Spell failed mid-cast
            casting = false;
            currentCastSpellId = 0;
            break;
        case Opcode::SMSG_SPELL_COOLDOWN:
            handleSpellCooldown(packet);
            break;
        case Opcode::SMSG_COOLDOWN_EVENT:
            handleCooldownEvent(packet);
            break;
        case Opcode::SMSG_CANCEL_AUTO_REPEAT:
            break; // Server signals to stop a repeating spell (wand/shoot); no client action needed
        case Opcode::SMSG_AURA_UPDATE:
            handleAuraUpdate(packet, false);
            break;
        case Opcode::SMSG_AURA_UPDATE_ALL:
            handleAuraUpdate(packet, true);
            break;
        case Opcode::SMSG_LEARNED_SPELL:
            handleLearnedSpell(packet);
            break;
        case Opcode::SMSG_SUPERCEDED_SPELL:
            handleSupercededSpell(packet);
            break;
        case Opcode::SMSG_REMOVED_SPELL:
            handleRemovedSpell(packet);
            break;
        case Opcode::SMSG_SEND_UNLEARN_SPELLS:
            handleUnlearnSpells(packet);
            break;

        // ---- Talents ----
        case Opcode::SMSG_TALENTS_INFO:
            handleTalentsInfo(packet);
            break;

        // ---- Phase 4: Group ----
        case Opcode::SMSG_GROUP_INVITE:
            handleGroupInvite(packet);
            break;
        case Opcode::SMSG_GROUP_DECLINE:
            handleGroupDecline(packet);
            break;
        case Opcode::SMSG_GROUP_LIST:
            handleGroupList(packet);
            break;
        case Opcode::SMSG_GROUP_UNINVITE:
            handleGroupUninvite(packet);
            break;
        case Opcode::SMSG_PARTY_COMMAND_RESULT:
            handlePartyCommandResult(packet);
            break;
        case Opcode::MSG_RAID_READY_CHECK:
            // Server ready-check prompt (minimal handling for now).
            packet.setReadPos(packet.getSize());
            break;
        case Opcode::MSG_RAID_READY_CHECK_CONFIRM:
            // Ready-check responses from members.
            packet.setReadPos(packet.getSize());
            break;
        case Opcode::SMSG_RAID_INSTANCE_INFO:
            // Raid lockout list (not yet surfaced in UI).
            packet.setReadPos(packet.getSize());
            break;
        case Opcode::SMSG_DUEL_REQUESTED:
            // Duel request UI flow not implemented yet.
            packet.setReadPos(packet.getSize());
            break;
        case Opcode::SMSG_PARTYKILLLOG:
            // Classic-era packet: killer GUID + victim GUID.
            // XP and combat state are handled by other packets; consume to avoid warning spam.
            packet.setReadPos(packet.getSize());
            break;

        // ---- Guild ----
        case Opcode::SMSG_GUILD_INFO:
            handleGuildInfo(packet);
            break;
        case Opcode::SMSG_GUILD_ROSTER:
            handleGuildRoster(packet);
            break;
        case Opcode::SMSG_GUILD_QUERY_RESPONSE:
            handleGuildQueryResponse(packet);
            break;
        case Opcode::SMSG_GUILD_EVENT:
            handleGuildEvent(packet);
            break;
        case Opcode::SMSG_GUILD_INVITE:
            handleGuildInvite(packet);
            break;
        case Opcode::SMSG_GUILD_COMMAND_RESULT:
            handleGuildCommandResult(packet);
            break;

        // ---- Phase 5: Loot/Gossip/Vendor ----
        case Opcode::SMSG_LOOT_RESPONSE:
            handleLootResponse(packet);
            break;
        case Opcode::SMSG_LOOT_RELEASE_RESPONSE:
            handleLootReleaseResponse(packet);
            break;
        case Opcode::SMSG_LOOT_REMOVED:
            handleLootRemoved(packet);
            break;
        case Opcode::SMSG_GOSSIP_MESSAGE:
            handleGossipMessage(packet);
            break;
        case Opcode::SMSG_QUESTGIVER_QUEST_LIST:
            handleQuestgiverQuestList(packet);
            break;
        case Opcode::SMSG_BINDPOINTUPDATE: {
            BindPointUpdateData data;
            if (BindPointUpdateParser::parse(packet, data)) {
                LOG_INFO("Bindpoint updated: mapId=", data.mapId,
                         " pos=(", data.x, ", ", data.y, ", ", data.z, ")");
                glm::vec3 canonical = core::coords::serverToCanonical(
                    glm::vec3(data.x, data.y, data.z));
                // Only show message if bind point was already set (not initial login sync)
                bool wasSet = hasHomeBind_;
                hasHomeBind_ = true;
                homeBindMapId_ = data.mapId;
                homeBindPos_ = canonical;
                if (bindPointCallback_) {
                    bindPointCallback_(data.mapId, canonical.x, canonical.y, canonical.z);
                }
                if (wasSet) {
                    addSystemChatMessage("Your home has been set.");
                }
            } else {
                LOG_WARNING("Failed to parse SMSG_BINDPOINTUPDATE");
            }
            break;
        }
        case Opcode::SMSG_GOSSIP_COMPLETE:
            handleGossipComplete(packet);
            break;
        case Opcode::SMSG_SPIRIT_HEALER_CONFIRM: {
            if (packet.getSize() - packet.getReadPos() < 8) {
                LOG_WARNING("SMSG_SPIRIT_HEALER_CONFIRM too short");
                break;
            }
            uint64_t npcGuid = packet.readUInt64();
            LOG_INFO("Spirit healer confirm from 0x", std::hex, npcGuid, std::dec);
            if (npcGuid) {
                resurrectCasterGuid_ = npcGuid;
                resurrectRequestPending_ = true;
            }
            break;
        }
        case Opcode::SMSG_RESURRECT_REQUEST: {
            if (packet.getSize() - packet.getReadPos() < 8) {
                LOG_WARNING("SMSG_RESURRECT_REQUEST too short");
                break;
            }
            uint64_t casterGuid = packet.readUInt64();
            LOG_INFO("Resurrect request from 0x", std::hex, casterGuid, std::dec);
            if (casterGuid) {
                resurrectCasterGuid_ = casterGuid;
                resurrectRequestPending_ = true;
            }
            break;
        }
        case Opcode::SMSG_TIME_SYNC_REQ: {
            if (packet.getSize() - packet.getReadPos() < 4) {
                LOG_WARNING("SMSG_TIME_SYNC_REQ too short");
                break;
            }
            uint32_t reason = packet.readUInt32();
            LOG_DEBUG("Time sync request reason: ", reason);
            resurrectPending_ = false;
            resurrectRequestPending_ = false;
            break;
        }
        case Opcode::SMSG_LIST_INVENTORY:
            handleListInventory(packet);
            break;
        case Opcode::SMSG_TRAINER_LIST:
            handleTrainerList(packet);
            break;
        case Opcode::SMSG_TRAINER_BUY_SUCCEEDED: {
            uint64_t guid = packet.readUInt64();
            uint32_t spellId = packet.readUInt32();
            (void)guid;

            // Add to known spells immediately for prerequisite re-evaluation
            // (SMSG_LEARNED_SPELL may come separately, but we need immediate update)
            if (!knownSpells.count(spellId)) {
                knownSpells.insert(spellId);
                LOG_INFO("Added spell ", spellId, " to known spells (trainer purchase)");
            }

            const std::string& name = getSpellName(spellId);
            if (!name.empty())
                addSystemChatMessage("You have learned " + name + ".");
            else
                addSystemChatMessage("Spell learned.");
            break;
        }
        case Opcode::SMSG_TRAINER_BUY_FAILED: {
            // Server rejected the spell purchase
            // Packet format: uint64 trainerGuid, uint32 spellId, uint32 errorCode
            uint64_t trainerGuid = packet.readUInt64();
            uint32_t spellId = packet.readUInt32();
            uint32_t errorCode = 0;
            if (packet.getSize() - packet.getReadPos() >= 4) {
                errorCode = packet.readUInt32();
            }
            LOG_WARNING("Trainer buy spell failed: guid=", trainerGuid,
                       " spellId=", spellId, " error=", errorCode);

            const std::string& spellName = getSpellName(spellId);
            std::string msg = "Cannot learn ";
            if (!spellName.empty()) msg += spellName;
            else msg += "spell #" + std::to_string(spellId);

            // Common error reasons
            if (errorCode == 0) msg += " (not enough money)";
            else if (errorCode == 1) msg += " (not enough skill)";
            else if (errorCode == 2) msg += " (already known)";
            else if (errorCode != 0) msg += " (error " + std::to_string(errorCode) + ")";

            addSystemChatMessage(msg);
            break;
        }

        // Silently ignore common packets we don't handle yet
        case Opcode::SMSG_INIT_WORLD_STATES: {
            // Minimal parse: uint32 mapId, uint32 zoneId, uint16 count, repeated (uint32 key, uint32 val)
            if (packet.getSize() - packet.getReadPos() < 10) {
                LOG_WARNING("SMSG_INIT_WORLD_STATES too short: ", packet.getSize(), " bytes");
                break;
            }
            worldStateMapId_ = packet.readUInt32();
            worldStateZoneId_ = packet.readUInt32();
            uint16_t count = packet.readUInt16();
            size_t needed = static_cast<size_t>(count) * 8;
            if (packet.getSize() - packet.getReadPos() < needed) {
                LOG_WARNING("SMSG_INIT_WORLD_STATES truncated: expected ", needed,
                            " bytes of state pairs, got ", packet.getSize() - packet.getReadPos());
                packet.setReadPos(packet.getSize());
                break;
            }
            worldStates_.clear();
            worldStates_.reserve(count);
            for (uint16_t i = 0; i < count; ++i) {
                uint32_t key = packet.readUInt32();
                uint32_t val = packet.readUInt32();
                worldStates_[key] = val;
            }
            break;
        }
        case Opcode::SMSG_INITIALIZE_FACTIONS: {
            // Minimal parse: uint32 count, repeated (uint8 flags, int32 standing)
            if (packet.getSize() - packet.getReadPos() < 4) {
                LOG_WARNING("SMSG_INITIALIZE_FACTIONS too short: ", packet.getSize(), " bytes");
                break;
            }
            uint32_t count = packet.readUInt32();
            size_t needed = static_cast<size_t>(count) * 5;
            if (packet.getSize() - packet.getReadPos() < needed) {
                LOG_WARNING("SMSG_INITIALIZE_FACTIONS truncated: expected ", needed,
                            " bytes of faction data, got ", packet.getSize() - packet.getReadPos());
                packet.setReadPos(packet.getSize());
                break;
            }
            initialFactions_.clear();
            initialFactions_.reserve(count);
            for (uint32_t i = 0; i < count; ++i) {
                FactionStandingInit fs{};
                fs.flags = packet.readUInt8();
                fs.standing = static_cast<int32_t>(packet.readUInt32());
                initialFactions_.push_back(fs);
            }
            break;
        }
        case Opcode::SMSG_FEATURE_SYSTEM_STATUS:
        case Opcode::SMSG_SET_FLAT_SPELL_MODIFIER:
        case Opcode::SMSG_SET_PCT_SPELL_MODIFIER:
        case Opcode::SMSG_SPELL_DELAYED:
        case Opcode::SMSG_EQUIPMENT_SET_SAVED:
        case Opcode::SMSG_PERIODICAURALOG:
        case Opcode::SMSG_SPELLENERGIZELOG:
        case Opcode::SMSG_ENVIRONMENTAL_DAMAGE_LOG:
        case Opcode::SMSG_SET_PROFICIENCY:
        case Opcode::SMSG_ACTION_BUTTONS:
            break;

        case Opcode::SMSG_LEVELUP_INFO:
        case Opcode::SMSG_LEVELUP_INFO_ALT: {
            // Server-authoritative level-up event.
            // First field is always the new level in Classic/TBC/WotLK-era layouts.
            if (packet.getSize() - packet.getReadPos() >= 4) {
                uint32_t newLevel = packet.readUInt32();
                if (newLevel > 0) {
                    uint32_t oldLevel = serverPlayerLevel_;
                    serverPlayerLevel_ = std::max(serverPlayerLevel_, newLevel);
                    for (auto& ch : characters) {
                        if (ch.guid == playerGuid) {
                            ch.level = serverPlayerLevel_;
                            break;
                        }
                    }
                    if (newLevel > oldLevel && levelUpCallback_) {
                        levelUpCallback_(newLevel);
                    }
                }
            }
            // Remaining payload (hp/mana/stat deltas) is optional for our client.
            packet.setReadPos(packet.getSize());
            break;
        }

        case Opcode::SMSG_PLAY_SOUND:
            if (packet.getSize() - packet.getReadPos() >= 4) {
                uint32_t soundId = packet.readUInt32();
                LOG_DEBUG("SMSG_PLAY_SOUND id=", soundId);
            }
            break;

        case Opcode::SMSG_LOOT_MONEY_NOTIFY: {
            // Format: uint32 money + uint8 soleLooter
            if (packet.getSize() - packet.getReadPos() >= 4) {
                uint32_t amount = packet.readUInt32();
                if (packet.getSize() - packet.getReadPos() >= 1) {
                    /*uint8_t soleLooter =*/ packet.readUInt8();
                }
                playerMoneyCopper_ += amount;
                pendingMoneyDelta_ = amount;
                pendingMoneyDeltaTimer_ = 2.0f;
                LOG_INFO("Looted ", amount, " copper (total: ", playerMoneyCopper_, ")");
                uint64_t notifyGuid = pendingLootMoneyGuid_ != 0 ? pendingLootMoneyGuid_ : currentLoot.lootGuid;
                pendingLootMoneyGuid_ = 0;
                pendingLootMoneyAmount_ = 0;
                pendingLootMoneyNotifyTimer_ = 0.0f;
                bool alreadyAnnounced = false;
                auto it = localLootState_.find(notifyGuid);
                if (it != localLootState_.end()) {
                    alreadyAnnounced = it->second.moneyTaken;
                    it->second.moneyTaken = true;
                }
                if (!alreadyAnnounced) {
                    addSystemChatMessage("Looted: " + formatCopperAmount(amount));
                    auto* renderer = core::Application::getInstance().getRenderer();
                    if (renderer) {
                        if (auto* sfx = renderer->getUiSoundManager()) {
                            if (amount >= 10000) {
                                sfx->playLootCoinLarge();
                            } else {
                                sfx->playLootCoinSmall();
                            }
                        }
                    }
                    if (notifyGuid != 0) {
                        recentLootMoneyAnnounceCooldowns_[notifyGuid] = 1.5f;
                    }
                }
            }
            break;
        }
        case Opcode::SMSG_LOOT_CLEAR_MONEY:
        case Opcode::SMSG_NPC_TEXT_UPDATE:
            break;
        case Opcode::SMSG_SELL_ITEM: {
            // uint64 vendorGuid, uint64 itemGuid, uint8 result
            if ((packet.getSize() - packet.getReadPos()) >= 17) {
                uint64_t vendorGuid = packet.readUInt64();
                uint64_t itemGuid = packet.readUInt64(); // itemGuid
                uint8_t result = packet.readUInt8();
                LOG_INFO("SMSG_SELL_ITEM: vendorGuid=0x", std::hex, vendorGuid,
                         " itemGuid=0x", itemGuid, std::dec,
                         " result=", static_cast<int>(result));
                if (result == 0) {
                    pendingSellToBuyback_.erase(itemGuid);
                } else {
                    bool removedPending = false;
                    auto it = pendingSellToBuyback_.find(itemGuid);
                    if (it != pendingSellToBuyback_.end()) {
                        for (auto bit = buybackItems_.begin(); bit != buybackItems_.end(); ++bit) {
                            if (bit->itemGuid == itemGuid) {
                                buybackItems_.erase(bit);
                                break;
                            }
                        }
                        pendingSellToBuyback_.erase(it);
                        removedPending = true;
                    }
                    if (!removedPending) {
                        // Some cores return a non-item GUID on sell failure; drop the newest
                        // optimistic entry if it is still pending so stale rows don't block buyback.
                        if (!buybackItems_.empty()) {
                            uint64_t frontGuid = buybackItems_.front().itemGuid;
                            if (pendingSellToBuyback_.erase(frontGuid) > 0) {
                                buybackItems_.pop_front();
                                removedPending = true;
                            }
                        }
                    }
                    if (!removedPending && !pendingSellToBuyback_.empty()) {
                        // Last-resort desync recovery.
                        pendingSellToBuyback_.clear();
                        buybackItems_.clear();
                    }
                    static const char* sellErrors[] = {
                        "OK", "Can't find item", "Can't sell item",
                        "Can't find vendor", "You don't own that item",
                        "Unknown error", "Only empty bag"
                    };
                    const char* msg = (result < 7) ? sellErrors[result] : "Unknown sell error";
                    addSystemChatMessage(std::string("Sell failed: ") + msg);
                    LOG_WARNING("SMSG_SELL_ITEM error: ", (int)result, " (", msg, ")");
                }
            }
            break;
        }
        case Opcode::SMSG_INVENTORY_CHANGE_FAILURE: {
            if ((packet.getSize() - packet.getReadPos()) >= 1) {
                uint8_t error = packet.readUInt8();
                if (error != 0) {
                    LOG_WARNING("SMSG_INVENTORY_CHANGE_FAILURE: error=", (int)error);
                    // InventoryResult enum (AzerothCore 3.3.5a)
                    const char* errMsg = nullptr;
                    switch (error) {
                        case 1:  errMsg = "You must reach level %d to use that item."; break;
                        case 2:  errMsg = "You don't have the required skill."; break;
                        case 3:  errMsg = "That item doesn't go in that slot."; break;
                        case 4:  errMsg = "That bag is full."; break;
                        case 5:  errMsg = "Can't put bags in bags."; break;
                        case 6:  errMsg = "Can't trade equipped bags."; break;
                        case 7:  errMsg = "That slot only holds ammo."; break;
                        case 8:  errMsg = "You can't use that item."; break;
                        case 9:  errMsg = "No equipment slot available."; break;
                        case 10: errMsg = "You can never use that item."; break;
                        case 11: errMsg = "You can never use that item."; break;
                        case 12: errMsg = "No equipment slot available."; break;
                        case 13: errMsg = "Can't equip with a two-handed weapon."; break;
                        case 14: errMsg = "Can't dual-wield."; break;
                        case 15: errMsg = "That item doesn't go in that bag."; break;
                        case 16: errMsg = "That item doesn't go in that bag."; break;
                        case 17: errMsg = "You can't carry any more of those."; break;
                        case 18: errMsg = "No equipment slot available."; break;
                        case 19: errMsg = "Can't stack those items."; break;
                        case 20: errMsg = "That item can't be equipped."; break;
                        case 21: errMsg = "Can't swap items."; break;
                        case 22: errMsg = "That slot is empty."; break;
                        case 23: errMsg = "Item not found."; break;
                        case 24: errMsg = "Can't drop soulbound items."; break;
                        case 25: errMsg = "Out of range."; break;
                        case 26: errMsg = "Need to split more than 1."; break;
                        case 27: errMsg = "Split failed."; break;
                        case 28: errMsg = "Not enough reagents."; break;
                        case 29: errMsg = "Not enough money."; break;
                        case 30: errMsg = "Not a bag."; break;
                        case 31: errMsg = "Can't destroy non-empty bag."; break;
                        case 32: errMsg = "You don't own that item."; break;
                        case 33: errMsg = "You can only have one quiver."; break;
                        case 34: errMsg = "No free bank slots."; break;
                        case 35: errMsg = "No bank here."; break;
                        case 36: errMsg = "Item is locked."; break;
                        case 37: errMsg = "You are stunned."; break;
                        case 38: errMsg = "You are dead."; break;
                        case 39: errMsg = "Can't do that right now."; break;
                        case 40: errMsg = "Internal bag error."; break;
                        case 49: errMsg = "Loot is gone."; break;
                        case 50: errMsg = "Inventory is full."; break;
                        case 51: errMsg = "Bank is full."; break;
                        case 52: errMsg = "That item is sold out."; break;
                        case 58: errMsg = "That object is busy."; break;
                        case 60: errMsg = "Can't do that in combat."; break;
                        case 61: errMsg = "Can't do that while disarmed."; break;
                        case 63: errMsg = "Requires a higher rank."; break;
                        case 64: errMsg = "Requires higher reputation."; break;
                        case 67: errMsg = "That item is unique-equipped."; break;
                        case 69: errMsg = "Not enough honor points."; break;
                        case 70: errMsg = "Not enough arena points."; break;
                        case 77: errMsg = "Too much gold."; break;
                        case 78: errMsg = "Can't do that during arena match."; break;
                        case 80: errMsg = "Requires a personal arena rating."; break;
                        case 87: errMsg = "Requires a higher level."; break;
                        case 88: errMsg = "Requires the right talent."; break;
                        default: break;
                    }
                    std::string msg = errMsg ? errMsg : "Inventory error (" + std::to_string(error) + ").";
                    addSystemChatMessage(msg);
                }
            }
            break;
        }
        case Opcode::SMSG_BUY_FAILED: {
            // vendorGuid(8) + itemId(4) + errorCode(1)
            if (packet.getSize() - packet.getReadPos() >= 13) {
                uint64_t vendorGuid = packet.readUInt64();
                uint32_t itemIdOrSlot = packet.readUInt32();
                uint8_t errCode = packet.readUInt8();
                LOG_INFO("SMSG_BUY_FAILED: vendorGuid=0x", std::hex, vendorGuid, std::dec,
                         " item/slot=", itemIdOrSlot,
                         " err=", static_cast<int>(errCode),
                         " pendingBuybackSlot=", pendingBuybackSlot_,
                         " pendingBuybackWireSlot=", pendingBuybackWireSlot_,
                         " pendingBuyItemId=", pendingBuyItemId_,
                         " pendingBuyItemSlot=", pendingBuyItemSlot_);
                if (pendingBuybackSlot_ >= 0) {
                    // Some cores require probing absolute buyback slots until a live entry is found.
                    if (errCode == 0) {
                        constexpr uint16_t kWotlkCmsgBuybackItemOpcode = 0x290;
                        constexpr uint32_t kBuybackSlotEnd = 85;
                        if (pendingBuybackWireSlot_ >= 74 && pendingBuybackWireSlot_ < kBuybackSlotEnd &&
                            socket && state == WorldState::IN_WORLD && currentVendorItems.vendorGuid != 0) {
                            ++pendingBuybackWireSlot_;
                            LOG_INFO("Buyback retry: vendorGuid=0x", std::hex, currentVendorItems.vendorGuid,
                                     std::dec, " uiSlot=", pendingBuybackSlot_,
                                     " wireSlot=", pendingBuybackWireSlot_);
                            network::Packet retry(kWotlkCmsgBuybackItemOpcode);
                            retry.writeUInt64(currentVendorItems.vendorGuid);
                            retry.writeUInt32(pendingBuybackWireSlot_);
                            socket->send(retry);
                            break;
                        }
                        // Exhausted slot probe: drop stale local row and advance.
                        if (pendingBuybackSlot_ < static_cast<int>(buybackItems_.size())) {
                            buybackItems_.erase(buybackItems_.begin() + pendingBuybackSlot_);
                        }
                        pendingBuybackSlot_ = -1;
                        pendingBuybackWireSlot_ = 0;
                        if (currentVendorItems.vendorGuid != 0 && socket && state == WorldState::IN_WORLD) {
                            auto pkt = ListInventoryPacket::build(currentVendorItems.vendorGuid);
                            socket->send(pkt);
                        }
                        break;
                    }
                    pendingBuybackSlot_ = -1;
                    pendingBuybackWireSlot_ = 0;
                }

                const char* msg = "Purchase failed.";
                switch (errCode) {
                    case 0: msg = "Purchase failed: item not found."; break;
                    case 2: msg = "You don't have enough money."; break;
                    case 4: msg = "Seller is too far away."; break;
                    case 5: msg = "That item is sold out."; break;
                    case 6: msg = "You can't carry any more items."; break;
                    default: break;
                }
                addSystemChatMessage(msg);
            }
            break;
        }
        case Opcode::MSG_RAID_TARGET_UPDATE:
            break;
        case Opcode::SMSG_WEATHER: {
            // Format: uint32 weatherType, float intensity, uint8 isAbrupt
            if (packet.getSize() - packet.getReadPos() >= 9) {
                uint32_t wType = packet.readUInt32();
                float wIntensity = packet.readFloat();
                /*uint8_t isAbrupt =*/ packet.readUInt8();
                weatherType_ = wType;
                weatherIntensity_ = wIntensity;
                const char* typeName = (wType == 1) ? "Rain" : (wType == 2) ? "Snow" : (wType == 3) ? "Storm" : "Clear";
                LOG_INFO("Weather changed: type=", wType, " (", typeName, "), intensity=", wIntensity);
            }
            break;
        }
        case Opcode::SMSG_GAMEOBJECT_QUERY_RESPONSE:
            handleGameObjectQueryResponse(packet);
            break;
        case Opcode::SMSG_GAMEOBJECT_PAGETEXT:
            handleGameObjectPageText(packet);
            break;
        case Opcode::SMSG_PAGE_TEXT_QUERY_RESPONSE:
            handlePageTextQueryResponse(packet);
            break;
        case Opcode::SMSG_QUESTGIVER_STATUS: {
            if (packet.getSize() - packet.getReadPos() >= 9) {
                uint64_t npcGuid = packet.readUInt64();
                uint8_t status = packetParsers_->readQuestGiverStatus(packet);
                npcQuestStatus_[npcGuid] = static_cast<QuestGiverStatus>(status);
                LOG_DEBUG("SMSG_QUESTGIVER_STATUS: guid=0x", std::hex, npcGuid, std::dec, " status=", (int)status);
            }
            break;
        }
        case Opcode::SMSG_QUESTGIVER_STATUS_MULTIPLE: {
            if (packet.getSize() - packet.getReadPos() >= 4) {
                uint32_t count = packet.readUInt32();
                for (uint32_t i = 0; i < count; ++i) {
                    if (packet.getSize() - packet.getReadPos() < 9) break;
                    uint64_t npcGuid = packet.readUInt64();
                    uint8_t status = packetParsers_->readQuestGiverStatus(packet);
                    npcQuestStatus_[npcGuid] = static_cast<QuestGiverStatus>(status);
                }
                LOG_DEBUG("SMSG_QUESTGIVER_STATUS_MULTIPLE: ", count, " entries");
            }
            break;
        }
        case Opcode::SMSG_QUESTGIVER_QUEST_DETAILS:
            handleQuestDetails(packet);
            break;
        case Opcode::SMSG_QUESTGIVER_QUEST_INVALID: {
            // Quest query failed - parse failure reason
            if (packet.getSize() - packet.getReadPos() >= 4) {
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
                            if (currentQuestDetails.questId == questId) {
                                if (!currentQuestDetails.title.empty()) fallbackTitle = currentQuestDetails.title;
                                fallbackObjectives = currentQuestDetails.objectives;
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
                    addSystemChatMessage(std::string("Quest unavailable: ") + reasonStr);
                }
            }
            break;
        }
        case Opcode::SMSG_QUESTGIVER_QUEST_COMPLETE: {
            // Mark quest as complete in local log
            if (packet.getSize() - packet.getReadPos() >= 4) {
                uint32_t questId = packet.readUInt32();
                LOG_INFO("Quest completed: questId=", questId);
                if (pendingTurnInQuestId_ == questId) {
                    pendingTurnInQuestId_ = 0;
                    pendingTurnInNpcGuid_ = 0;
                    pendingTurnInRewardRequest_ = false;
                }
                for (auto it = questLog_.begin(); it != questLog_.end(); ++it) {
                    if (it->questId == questId) {
                        questLog_.erase(it);
                        LOG_INFO("  Removed quest ", questId, " from quest log");
                        break;
                    }
                }
            }
            // Re-query all nearby quest giver NPCs so markers refresh
            if (socket) {
                for (const auto& [guid, entity] : entityManager.getEntities()) {
                    if (entity->getType() != ObjectType::UNIT) continue;
                    auto unit = std::static_pointer_cast<Unit>(entity);
                    if (unit->getNpcFlags() & 0x02) {
                        network::Packet qsPkt(wireOpcode(Opcode::CMSG_QUESTGIVER_STATUS_QUERY));
                        qsPkt.writeUInt64(guid);
                        socket->send(qsPkt);
                    }
                }
            }
            break;
        }
        case Opcode::SMSG_QUESTUPDATE_ADD_KILL: {
            // Quest kill count update
            // Compatibility: some classic-family opcode tables swap ADD_KILL and COMPLETE.
            size_t rem = packet.getSize() - packet.getReadPos();
            if (rem >= 12) {
                uint32_t questId = packet.readUInt32();
                clearPendingQuestAccept(questId);
                uint32_t entry = packet.readUInt32();   // Creature entry
                uint32_t count = packet.readUInt32();   // Current kills
                uint32_t reqCount = 0;
                if (packet.getSize() - packet.getReadPos() >= 4) {
                    reqCount = packet.readUInt32();     // Required kills (if present)
                }

                LOG_INFO("Quest kill update: questId=", questId, " entry=", entry,
                         " count=", count, "/", reqCount);

                // Update quest log with kill count
                for (auto& quest : questLog_) {
                    if (quest.questId == questId) {
                        // Preserve prior required count if this packet variant omits it.
                        if (reqCount == 0) {
                            auto it = quest.killCounts.find(entry);
                            if (it != quest.killCounts.end()) reqCount = it->second.second;
                            if (reqCount == 0) reqCount = count;
                        }
                        quest.killCounts[entry] = {count, reqCount};

                        std::string progressMsg = quest.title + ": " +
                                                std::to_string(count) + "/" +
                                                std::to_string(reqCount);
                        addSystemChatMessage(progressMsg);

                        LOG_INFO("Updated kill count for quest ", questId, ": ",
                                 count, "/", reqCount);
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
                        addSystemChatMessage("Quest Complete: " + quest.title);
                        break;
                    }
                }
            }
            break;
        }
        case Opcode::SMSG_QUESTUPDATE_ADD_ITEM: {
            // Quest item count update: itemId + count
            if (packet.getSize() - packet.getReadPos() >= 8) {
                uint32_t itemId = packet.readUInt32();
                uint32_t count = packet.readUInt32();
                queryItemInfo(itemId, 0);

                std::string itemLabel = "item #" + std::to_string(itemId);
                if (const ItemQueryResponseData* info = getItemInfo(itemId)) {
                    if (!info->name.empty()) itemLabel = info->name;
                }

                bool updatedAny = false;
                for (auto& quest : questLog_) {
                    if (quest.complete) continue;
                    const bool tracksItem =
                        quest.requiredItemCounts.count(itemId) > 0 ||
                        quest.itemCounts.count(itemId) > 0;
                    if (!tracksItem) continue;
                    quest.itemCounts[itemId] = count;
                    updatedAny = true;
                }
                addSystemChatMessage("Quest item: " + itemLabel + " (" + std::to_string(count) + ")");
                LOG_INFO("Quest item update: itemId=", itemId, " count=", count,
                         " trackedQuestsUpdated=", updatedAny);
            }
            break;
        }
        case Opcode::SMSG_QUESTUPDATE_COMPLETE: {
            // Quest objectives completed - mark as ready to turn in.
            // Compatibility: some classic-family opcode tables swap COMPLETE and ADD_KILL.
            size_t rem = packet.getSize() - packet.getReadPos();
            if (rem >= 12) {
                uint32_t questId = packet.readUInt32();
                clearPendingQuestAccept(questId);
                uint32_t entry = packet.readUInt32();
                uint32_t count = packet.readUInt32();
                uint32_t reqCount = 0;
                if (packet.getSize() - packet.getReadPos() >= 4) reqCount = packet.readUInt32();
                if (reqCount == 0) reqCount = count;
                LOG_INFO("Quest kill update (compat via COMPLETE): questId=", questId,
                         " entry=", entry, " count=", count, "/", reqCount);
                for (auto& quest : questLog_) {
                    if (quest.questId == questId) {
                        quest.killCounts[entry] = {count, reqCount};
                        addSystemChatMessage(quest.title + ": " + std::to_string(count) +
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
                        addSystemChatMessage("Quest Complete: " + quest.title);
                        LOG_INFO("Marked quest ", questId, " as complete");
                        break;
                    }
                }
            }
            break;
        }
        case Opcode::SMSG_QUEST_FORCE_REMOVE: {
            // Minimal parse: uint32 questId
            if (packet.getSize() - packet.getReadPos() < 4) {
                LOG_WARNING("SMSG_QUEST_FORCE_REMOVE too short");
                break;
            }
            uint32_t questId = packet.readUInt32();
            clearPendingQuestAccept(questId);
            pendingQuestQueryIds_.erase(questId);
            if (questId == 0) {
                // Some servers emit a zero-id variant during world bootstrap.
                // Treat as no-op to avoid false "Quest removed" spam.
                break;
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
            if (currentQuestDetails.questId == questId) {
                questDetailsOpen = false;
                currentQuestDetails = QuestDetailsData{};
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
                if (!removedTitle.empty()) {
                    addSystemChatMessage("Quest removed: " + removedTitle);
                } else {
                    addSystemChatMessage("Quest removed (ID " + std::to_string(questId) + ").");
                }
            }
            break;
        }
        case Opcode::SMSG_QUEST_QUERY_RESPONSE: {
            if (packet.getSize() < 8) {
                LOG_WARNING("SMSG_QUEST_QUERY_RESPONSE: packet too small (", packet.getSize(), " bytes)");
                break;
            }

            uint32_t questId = packet.readUInt32();
            packet.readUInt32(); // questMethod

            const bool isClassicLayout = packetParsers_ && packetParsers_->questLogStride() == 3;
            const QuestQueryTextCandidate parsed = pickBestQuestQueryTexts(packet.getData(), isClassicLayout);

            for (auto& q : questLog_) {
                if (q.questId != questId) continue;

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
                break;
            }

            pendingQuestQueryIds_.erase(questId);
            break;
        }
        case Opcode::SMSG_QUESTLOG_FULL: {
            LOG_INFO("***** RECEIVED SMSG_QUESTLOG_FULL *****");
            LOG_INFO("  Packet size: ", packet.getSize());
            LOG_INFO("  Server uses SMSG_QUESTLOG_FULL for quest log sync!");
            // TODO: Parse quest log entries from this packet
            break;
        }
        case Opcode::SMSG_QUESTGIVER_REQUEST_ITEMS:
            handleQuestRequestItems(packet);
            break;
        case Opcode::SMSG_QUESTGIVER_OFFER_REWARD:
            handleQuestOfferReward(packet);
            break;
        case Opcode::SMSG_GROUP_SET_LEADER:
            LOG_DEBUG("Ignoring known opcode: 0x", std::hex, opcode, std::dec);
            break;

        // ---- Teleport / Transfer ----
        case Opcode::MSG_MOVE_TELEPORT_ACK:
            handleTeleportAck(packet);
            break;
        case Opcode::SMSG_TRANSFER_PENDING: {
            // SMSG_TRANSFER_PENDING: uint32 mapId, then optional transport data
            uint32_t pendingMapId = packet.readUInt32();
            LOG_INFO("SMSG_TRANSFER_PENDING: mapId=", pendingMapId);
            // Optional: if remaining data, there's a transport entry + mapId
            if (packet.getReadPos() + 8 <= packet.getSize()) {
                uint32_t transportEntry = packet.readUInt32();
                uint32_t transportMapId = packet.readUInt32();
                LOG_INFO("  Transport entry=", transportEntry, " transportMapId=", transportMapId);
            }
            break;
        }
        case Opcode::SMSG_NEW_WORLD:
            handleNewWorld(packet);
            break;
        case Opcode::SMSG_TRANSFER_ABORTED: {
            uint32_t mapId = packet.readUInt32();
            uint8_t reason = (packet.getReadPos() < packet.getSize()) ? packet.readUInt8() : 0;
            LOG_WARNING("SMSG_TRANSFER_ABORTED: mapId=", mapId, " reason=", (int)reason);
            addSystemChatMessage("Transfer aborted.");
            break;
        }

        // ---- Taxi / Flight Paths ----
        case Opcode::SMSG_SHOWTAXINODES:
            handleShowTaxiNodes(packet);
            break;
        case Opcode::SMSG_ACTIVATETAXIREPLY:
            handleActivateTaxiReply(packet);
            break;
        case Opcode::SMSG_STANDSTATE_UPDATE:
            // Server confirms stand state change (sit/stand/sleep/kneel)
            // TODO: parse uint8 standState and update player entity
            packet.setReadPos(packet.getSize());
            break;
        case Opcode::SMSG_NEW_TAXI_PATH:
            // Empty packet - server signals a new flight path was learned
            // The actual node details come in the next SMSG_SHOWTAXINODES
            addSystemChatMessage("New flight path discovered!");
            break;

        // ---- Arena / Battleground ----
        case Opcode::SMSG_BATTLEFIELD_STATUS:
            handleBattlefieldStatus(packet);
            break;
        case Opcode::SMSG_BATTLEFIELD_LIST:
            LOG_INFO("Received SMSG_BATTLEFIELD_LIST");
            break;
        case Opcode::SMSG_BATTLEFIELD_PORT_DENIED:
            addSystemChatMessage("Battlefield port denied.");
            break;
        case Opcode::MSG_BATTLEGROUND_PLAYER_POSITIONS:
            // Optional map position updates for BG objectives/players.
            packet.setReadPos(packet.getSize());
            break;
        case Opcode::SMSG_REMOVED_FROM_PVP_QUEUE:
            addSystemChatMessage("You have been removed from the PvP queue.");
            break;
        case Opcode::SMSG_GROUP_JOINED_BATTLEGROUND:
            addSystemChatMessage("Your group has joined the battleground.");
            break;
        case Opcode::SMSG_JOINED_BATTLEGROUND_QUEUE:
            addSystemChatMessage("You have joined the battleground queue.");
            break;
        case Opcode::SMSG_BATTLEGROUND_PLAYER_JOINED:
            LOG_INFO("Battleground player joined");
            break;
        case Opcode::SMSG_BATTLEGROUND_PLAYER_LEFT:
            LOG_INFO("Battleground player left");
            break;
        case Opcode::SMSG_ARENA_TEAM_COMMAND_RESULT:
            handleArenaTeamCommandResult(packet);
            break;
        case Opcode::SMSG_ARENA_TEAM_QUERY_RESPONSE:
            handleArenaTeamQueryResponse(packet);
            break;
        case Opcode::SMSG_ARENA_TEAM_ROSTER:
            LOG_INFO("Received SMSG_ARENA_TEAM_ROSTER");
            break;
        case Opcode::SMSG_ARENA_TEAM_INVITE:
            handleArenaTeamInvite(packet);
            break;
        case Opcode::SMSG_ARENA_TEAM_EVENT:
            handleArenaTeamEvent(packet);
            break;
        case Opcode::SMSG_ARENA_TEAM_STATS:
            LOG_INFO("Received SMSG_ARENA_TEAM_STATS");
            break;
        case Opcode::SMSG_ARENA_ERROR:
            handleArenaError(packet);
            break;
        case Opcode::MSG_PVP_LOG_DATA:
            LOG_INFO("Received MSG_PVP_LOG_DATA");
            break;
        case Opcode::MSG_INSPECT_ARENA_TEAMS:
            LOG_INFO("Received MSG_INSPECT_ARENA_TEAMS");
            break;
        case Opcode::MSG_TALENT_WIPE_CONFIRM:
            // Talent reset confirmation payload is not needed client-side right now.
            packet.setReadPos(packet.getSize());
            break;

        // ---- MSG_MOVE_* opcodes (server relays other players' movement) ----
        case Opcode::MSG_MOVE_START_FORWARD:
        case Opcode::MSG_MOVE_START_BACKWARD:
        case Opcode::MSG_MOVE_STOP:
        case Opcode::MSG_MOVE_START_STRAFE_LEFT:
        case Opcode::MSG_MOVE_START_STRAFE_RIGHT:
        case Opcode::MSG_MOVE_STOP_STRAFE:
        case Opcode::MSG_MOVE_JUMP:
        case Opcode::MSG_MOVE_START_TURN_LEFT:
        case Opcode::MSG_MOVE_START_TURN_RIGHT:
        case Opcode::MSG_MOVE_STOP_TURN:
        case Opcode::MSG_MOVE_SET_FACING:
        case Opcode::MSG_MOVE_FALL_LAND:
        case Opcode::MSG_MOVE_HEARTBEAT:
        case Opcode::MSG_MOVE_START_SWIM:
        case Opcode::MSG_MOVE_STOP_SWIM:
            if (state == WorldState::IN_WORLD) {
                handleOtherPlayerMovement(packet);
            }
            break;

        // ---- Mail ----
        case Opcode::SMSG_SHOW_MAILBOX:
            handleShowMailbox(packet);
            break;
        case Opcode::SMSG_MAIL_LIST_RESULT:
            handleMailListResult(packet);
            break;
        case Opcode::SMSG_SEND_MAIL_RESULT:
            handleSendMailResult(packet);
            break;
        case Opcode::SMSG_RECEIVED_MAIL:
            handleReceivedMail(packet);
            break;
        case Opcode::MSG_QUERY_NEXT_MAIL_TIME:
            handleQueryNextMailTime(packet);
            break;
        case Opcode::SMSG_CHANNEL_LIST:
            // Channel member listing currently not rendered in UI.
            packet.setReadPos(packet.getSize());
            break;
        case Opcode::SMSG_INSPECT_RESULTS_UPDATE:
            handleInspectResults(packet);
            break;

        // ---- Bank ----
        case Opcode::SMSG_SHOW_BANK:
            handleShowBank(packet);
            break;
        case Opcode::SMSG_BUY_BANK_SLOT_RESULT:
            handleBuyBankSlotResult(packet);
            break;

        // ---- Guild Bank ----
        case Opcode::SMSG_GUILD_BANK_LIST:
            handleGuildBankList(packet);
            break;

        // ---- Auction House ----
        case Opcode::MSG_AUCTION_HELLO:
            handleAuctionHello(packet);
            break;
        case Opcode::SMSG_AUCTION_LIST_RESULT:
            handleAuctionListResult(packet);
            break;
        case Opcode::SMSG_AUCTION_OWNER_LIST_RESULT:
            handleAuctionOwnerListResult(packet);
            break;
        case Opcode::SMSG_AUCTION_BIDDER_LIST_RESULT:
            handleAuctionBidderListResult(packet);
            break;
        case Opcode::SMSG_AUCTION_COMMAND_RESULT:
            handleAuctionCommandResult(packet);
            break;
        case Opcode::SMSG_AUCTION_OWNER_NOTIFICATION:
        case Opcode::SMSG_AUCTION_BIDDER_NOTIFICATION:
            // Auction notification payloads are informational; ignore until UI support lands.
            packet.setReadPos(packet.getSize());
            break;
        case Opcode::SMSG_TAXINODE_STATUS:
            // Node status cache not implemented yet.
            packet.setReadPos(packet.getSize());
            break;
        case Opcode::SMSG_INIT_EXTRA_AURA_INFO_OBSOLETE:
        case Opcode::SMSG_SET_EXTRA_AURA_INFO_OBSOLETE:
            // Extra aura metadata (icons/durations) not yet consumed by aura UI.
            packet.setReadPos(packet.getSize());
            break;
        case Opcode::MSG_MOVE_WORLDPORT_ACK:
            // Client uses this outbound; treat inbound variant as no-op for robustness.
            packet.setReadPos(packet.getSize());
            break;
        case Opcode::MSG_MOVE_TIME_SKIPPED:
            // Observed custom server packet (8 bytes). Safe-consume for now.
            packet.setReadPos(packet.getSize());
            break;

        default:
            // In pre-world states we need full visibility (char create/login handshakes).
            // In-world we keep de-duplication to avoid heavy log I/O in busy areas.
            if (state != WorldState::IN_WORLD) {
                static std::unordered_set<uint32_t> loggedUnhandledByState;
                const uint32_t key = (static_cast<uint32_t>(static_cast<uint8_t>(state)) << 16) |
                                     static_cast<uint32_t>(opcode);
                if (loggedUnhandledByState.insert(key).second) {
                    LOG_WARNING("Unhandled world opcode: 0x", std::hex, opcode, std::dec,
                                " state=", static_cast<int>(state),
                                " size=", packet.getSize());
                    const auto& data = packet.getData();
                    std::string hex;
                    size_t limit = std::min<size_t>(data.size(), 48);
                    hex.reserve(limit * 3);
                    for (size_t i = 0; i < limit; ++i) {
                        char b[4];
                        snprintf(b, sizeof(b), "%02x ", data[i]);
                        hex += b;
                    }
                    LOG_INFO("Unhandled opcode payload hex (first ", limit, " bytes): ", hex);
                }
            } else {
                static std::unordered_set<uint16_t> loggedUnhandledOpcodes;
                if (loggedUnhandledOpcodes.insert(static_cast<uint16_t>(opcode)).second) {
                    LOG_WARNING("Unhandled world opcode: 0x", std::hex, opcode, std::dec);
                }
            }
            break;
    }
}

void GameHandler::handleAuthChallenge(network::Packet& packet) {
    LOG_INFO("Handling SMSG_AUTH_CHALLENGE");

    AuthChallengeData challenge;
    if (!AuthChallengeParser::parse(packet, challenge)) {
        fail("Failed to parse SMSG_AUTH_CHALLENGE");
        return;
    }

    if (!challenge.isValid()) {
        fail("Invalid auth challenge data");
        return;
    }

    // Store server seed
    serverSeed = challenge.serverSeed;
    LOG_DEBUG("Server seed: 0x", std::hex, serverSeed, std::dec);

    setState(WorldState::CHALLENGE_RECEIVED);

    // Send authentication session
    sendAuthSession();
}

void GameHandler::sendAuthSession() {
    LOG_INFO("Sending CMSG_AUTH_SESSION");

    // Build authentication packet
    auto packet = AuthSessionPacket::build(
        build,
        accountName,
        clientSeed,
        sessionKey,
        serverSeed,
        realmId_
    );

    LOG_DEBUG("CMSG_AUTH_SESSION packet size: ", packet.getSize(), " bytes");

    // Send packet (unencrypted - this is the last unencrypted packet)
    socket->send(packet);

    // Enable encryption IMMEDIATELY after sending AUTH_SESSION
    // AzerothCore enables encryption before sending AUTH_RESPONSE,
    // so we need to be ready to decrypt the response
    LOG_INFO("Enabling encryption immediately after AUTH_SESSION");
    socket->initEncryption(sessionKey, build);

    setState(WorldState::AUTH_SENT);
    LOG_INFO("CMSG_AUTH_SESSION sent, encryption enabled, waiting for AUTH_RESPONSE...");
}

void GameHandler::handleAuthResponse(network::Packet& packet) {
    LOG_INFO("Handling SMSG_AUTH_RESPONSE");

    AuthResponseData response;
    if (!AuthResponseParser::parse(packet, response)) {
        fail("Failed to parse SMSG_AUTH_RESPONSE");
        return;
    }

    if (!response.isSuccess()) {
        std::string reason = std::string("Authentication failed: ") +
                           getAuthResultString(response.result);
        fail(reason);
        return;
    }

    // Encryption was already enabled after sending AUTH_SESSION
    LOG_INFO("AUTH_RESPONSE OK - world authentication successful");

    setState(WorldState::AUTHENTICATED);

    LOG_INFO("========================================");
    LOG_INFO("   WORLD AUTHENTICATION SUCCESSFUL!");
    LOG_INFO("========================================");
    LOG_INFO("Connected to world server");
    LOG_INFO("Ready for character operations");

    setState(WorldState::READY);

    // Request character list automatically
    requestCharacterList();

    // Call success callback
    if (onSuccess) {
        onSuccess();
    }
}

void GameHandler::requestCharacterList() {
    if (requiresWarden_) {
        // Gate already surfaced via failure callback/chat; avoid per-frame warning spam.
        wardenCharEnumBlockedLogged_ = true;
        return;
    }

    if (state == WorldState::FAILED || !socket || !socket->isConnected()) {
        return;
    }

    if (state != WorldState::READY && state != WorldState::AUTHENTICATED &&
        state != WorldState::CHAR_LIST_RECEIVED) {
        LOG_WARNING("Cannot request character list in state: ", worldStateName(state));
        return;
    }

    LOG_INFO("Requesting character list from server...");

    // Prevent the UI from showing/selecting stale characters while we wait for the new SMSG_CHAR_ENUM.
    // This matters after character create/delete where the old list can linger for a few frames.
    characters.clear();

    // Build CMSG_CHAR_ENUM packet (no body, just opcode)
    auto packet = CharEnumPacket::build();

    // Send packet
    socket->send(packet);

    setState(WorldState::CHAR_LIST_REQUESTED);
    LOG_INFO("CMSG_CHAR_ENUM sent, waiting for character list...");
}

void GameHandler::handleCharEnum(network::Packet& packet) {
    LOG_INFO("Handling SMSG_CHAR_ENUM");

    CharEnumResponse response;
    // IMPORTANT: Do not infer packet formats from numeric build alone.
    // Turtle WoW uses a "high" build but classic-era world packet formats.
    bool parsed = packetParsers_ ? packetParsers_->parseCharEnum(packet, response)
                                 : CharEnumParser::parse(packet, response);
    if (!parsed) {
        fail("Failed to parse SMSG_CHAR_ENUM");
        return;
    }

    // Store characters
    characters = response.characters;

    setState(WorldState::CHAR_LIST_RECEIVED);

    LOG_INFO("========================================");
    LOG_INFO("   CHARACTER LIST RECEIVED");
    LOG_INFO("========================================");
    LOG_INFO("Found ", characters.size(), " character(s)");

    if (characters.empty()) {
        LOG_INFO("No characters on this account");
    } else {
        LOG_INFO("Characters:");
        for (size_t i = 0; i < characters.size(); ++i) {
            const auto& character = characters[i];
            LOG_INFO("  [", i + 1, "] ", character.name);
            LOG_INFO("      GUID: 0x", std::hex, character.guid, std::dec);
            LOG_INFO("      ", getRaceName(character.race), " ",
                     getClassName(character.characterClass));
            LOG_INFO("      Level ", (int)character.level);
        }
    }

    LOG_INFO("Ready to select character");
}

void GameHandler::createCharacter(const CharCreateData& data) {

    // Online mode: send packet to server
    if (!socket) {
        LOG_WARNING("Cannot create character: not connected");
        if (charCreateCallback_) {
            charCreateCallback_(false, "Not connected to server");
        }
        return;
    }

    if (requiresWarden_) {
        std::string msg = "Server requires anti-cheat/Warden; character creation blocked.";
        LOG_WARNING("Blocking CMSG_CHAR_CREATE while Warden gate is active");
        if (charCreateCallback_) {
            charCreateCallback_(false, msg);
        }
        return;
    }

    if (state != WorldState::CHAR_LIST_RECEIVED) {
        std::string msg = "Character list not ready yet. Wait for SMSG_CHAR_ENUM.";
        LOG_WARNING("Blocking CMSG_CHAR_CREATE in state=", worldStateName(state),
                    " (awaiting CHAR_LIST_RECEIVED)");
        if (charCreateCallback_) {
            charCreateCallback_(false, msg);
        }
        return;
    }

    auto packet = CharCreatePacket::build(data);
    socket->send(packet);
    LOG_INFO("CMSG_CHAR_CREATE sent for: ", data.name);
}

void GameHandler::handleCharCreateResponse(network::Packet& packet) {
    CharCreateResponseData data;
    if (!CharCreateResponseParser::parse(packet, data)) {
        LOG_ERROR("Failed to parse SMSG_CHAR_CREATE");
        return;
    }

    if (data.result == CharCreateResult::SUCCESS || data.result == CharCreateResult::IN_PROGRESS) {
        LOG_INFO("Character created successfully (code=", static_cast<int>(data.result), ")");
        requestCharacterList();
        if (charCreateCallback_) {
            charCreateCallback_(true, "Character created!");
        }
    } else {
        std::string msg;
        switch (data.result) {
            case CharCreateResult::CHAR_ERROR: msg = "Server error"; break;
            case CharCreateResult::FAILED: msg = "Creation failed"; break;
            case CharCreateResult::NAME_IN_USE: msg = "Name already in use"; break;
            case CharCreateResult::DISABLED: msg = "Character creation disabled"; break;
            case CharCreateResult::PVP_TEAMS_VIOLATION: msg = "PvP faction violation"; break;
            case CharCreateResult::SERVER_LIMIT: msg = "Server character limit reached"; break;
            case CharCreateResult::ACCOUNT_LIMIT: msg = "Account character limit reached"; break;
            case CharCreateResult::SERVER_QUEUE: msg = "Server is queued"; break;
            case CharCreateResult::ONLY_EXISTING: msg = "Only existing characters allowed"; break;
            case CharCreateResult::EXPANSION: msg = "Expansion required"; break;
            case CharCreateResult::EXPANSION_CLASS: msg = "Expansion required for this class"; break;
            case CharCreateResult::LEVEL_REQUIREMENT: msg = "Level requirement not met"; break;
            case CharCreateResult::UNIQUE_CLASS_LIMIT: msg = "Unique class limit reached"; break;
            case CharCreateResult::RESTRICTED_RACECLASS: msg = "Race/class combination not allowed"; break;
            case CharCreateResult::IN_PROGRESS: msg = "Character creation in progress..."; break;
            case CharCreateResult::CHARACTER_CHOOSE_RACE: msg = "Please choose a different race"; break;
            case CharCreateResult::CHARACTER_ARENA_LEADER: msg = "Arena team leader restriction"; break;
            case CharCreateResult::CHARACTER_DELETE_MAIL: msg = "Character has mail"; break;
            case CharCreateResult::CHARACTER_SWAP_FACTION: msg = "Faction swap restriction"; break;
            case CharCreateResult::CHARACTER_RACE_ONLY: msg = "Race-only restriction"; break;
            case CharCreateResult::CHARACTER_GOLD_LIMIT: msg = "Gold limit reached"; break;
            case CharCreateResult::FORCE_LOGIN: msg = "Force login required"; break;
            case CharCreateResult::CHARACTER_IN_GUILD: msg = "Character is in a guild"; break;
            // Name validation errors
            case CharCreateResult::NAME_FAILURE: msg = "Invalid name"; break;
            case CharCreateResult::NAME_NO_NAME: msg = "Please enter a name"; break;
            case CharCreateResult::NAME_TOO_SHORT: msg = "Name is too short"; break;
            case CharCreateResult::NAME_TOO_LONG: msg = "Name is too long"; break;
            case CharCreateResult::NAME_INVALID_CHARACTER: msg = "Name contains invalid characters"; break;
            case CharCreateResult::NAME_MIXED_LANGUAGES: msg = "Name mixes languages"; break;
            case CharCreateResult::NAME_PROFANE: msg = "Name contains profanity"; break;
            case CharCreateResult::NAME_RESERVED: msg = "Name is reserved"; break;
            case CharCreateResult::NAME_INVALID_APOSTROPHE: msg = "Invalid apostrophe in name"; break;
            case CharCreateResult::NAME_MULTIPLE_APOSTROPHES: msg = "Name has multiple apostrophes"; break;
            case CharCreateResult::NAME_THREE_CONSECUTIVE: msg = "Name has 3+ consecutive same letters"; break;
            case CharCreateResult::NAME_INVALID_SPACE: msg = "Invalid space in name"; break;
            case CharCreateResult::NAME_CONSECUTIVE_SPACES: msg = "Name has consecutive spaces"; break;
            default: msg = "Unknown error (code " + std::to_string(static_cast<int>(data.result)) + ")"; break;
        }
        LOG_WARNING("Character creation failed: ", msg, " (code=", static_cast<int>(data.result), ")");
        if (charCreateCallback_) {
            charCreateCallback_(false, msg);
        }
    }
}

void GameHandler::deleteCharacter(uint64_t characterGuid) {
    if (!socket) {
        if (charDeleteCallback_) charDeleteCallback_(false);
        return;
    }

    network::Packet packet(wireOpcode(Opcode::CMSG_CHAR_DELETE));
    packet.writeUInt64(characterGuid);
    socket->send(packet);
    LOG_INFO("CMSG_CHAR_DELETE sent for GUID: 0x", std::hex, characterGuid, std::dec);
}

const Character* GameHandler::getActiveCharacter() const {
    if (activeCharacterGuid_ == 0) return nullptr;
    for (const auto& ch : characters) {
        if (ch.guid == activeCharacterGuid_) return &ch;
    }
    return nullptr;
}

const Character* GameHandler::getFirstCharacter() const {
    if (characters.empty()) return nullptr;
    return &characters.front();
}














void GameHandler::handleCharLoginFailed(network::Packet& packet) {
    uint8_t reason = packet.readUInt8();

    static const char* reasonNames[] = {
        "Login failed",          // 0
        "World server is down",  // 1
        "Duplicate character",   // 2 (session still active)
        "No instance servers",   // 3
        "Login disabled",        // 4
        "Character not found",   // 5
        "Locked for transfer",   // 6
        "Locked by billing",     // 7
        "Using remote",          // 8
    };
    const char* msg = (reason < 9) ? reasonNames[reason] : "Unknown reason";

    LOG_ERROR("SMSG_CHARACTER_LOGIN_FAILED: reason=", (int)reason, " (", msg, ")");

    // Allow the player to re-select a character
    setState(WorldState::CHAR_LIST_RECEIVED);

    if (charLoginFailCallback_) {
        charLoginFailCallback_(msg);
    }
}

void GameHandler::selectCharacter(uint64_t characterGuid) {
    if (state != WorldState::CHAR_LIST_RECEIVED) {
        LOG_WARNING("Cannot select character in state: ", (int)state);
        return;
    }

    // Make the selected character authoritative in GameHandler.
    // This avoids relying on UI/Application ordering for appearance-dependent logic.
    activeCharacterGuid_ = characterGuid;

    LOG_INFO("========================================");
    LOG_INFO("   ENTERING WORLD");
    LOG_INFO("========================================");
    LOG_INFO("Character GUID: 0x", std::hex, characterGuid, std::dec);

    // Find character name for logging
    for (const auto& character : characters) {
        if (character.guid == characterGuid) {
            LOG_INFO("Character: ", character.name);
            LOG_INFO("Level ", (int)character.level, " ",
                     getRaceName(character.race), " ",
                     getClassName(character.characterClass));
            playerRace_ = character.race;
            break;
        }
    }

    // Store player GUID
    playerGuid = characterGuid;

    // Reset per-character state so previous character data doesn't bleed through
    inventory = Inventory();
    onlineItems_.clear();
    itemInfoCache_.clear();
    pendingItemQueries_.clear();
    equipSlotGuids_ = {};
    backpackSlotGuids_ = {};
    invSlotBase_ = -1;
    packSlotBase_ = -1;
    lastPlayerFields_.clear();
    onlineEquipDirty_ = false;
    playerMoneyCopper_ = 0;
    playerArmorRating_ = 0;
    knownSpells.clear();
    spellCooldowns.clear();
    actionBar = {};
    playerAuras.clear();
    targetAuras.clear();
    playerXp_ = 0;
    playerNextLevelXp_ = 0;
    serverPlayerLevel_ = 1;
    std::fill(playerExploredZones_.begin(), playerExploredZones_.end(), 0u);
    hasPlayerExploredZones_ = false;
    playerSkills_.clear();
    questLog_.clear();
    pendingQuestQueryIds_.clear();
    pendingLoginQuestResync_ = false;
    pendingLoginQuestResyncTimeout_ = 0.0f;
    pendingQuestAcceptTimeouts_.clear();
    pendingQuestAcceptNpcGuids_.clear();
    npcQuestStatus_.clear();
    hostileAttackers_.clear();
    combatText.clear();
    autoAttacking = false;
    autoAttackTarget = 0;
    casting = false;
    currentCastSpellId = 0;
    pendingGameObjectInteractGuid_ = 0;
    castTimeRemaining = 0.0f;
    castTimeTotal = 0.0f;
    playerDead_ = false;
    releasedSpirit_ = false;
    targetGuid = 0;
    focusGuid = 0;
    lastTargetGuid = 0;
    tabCycleStale = true;
    entityManager = EntityManager();

    // Build CMSG_PLAYER_LOGIN packet
    auto packet = PlayerLoginPacket::build(characterGuid);

    // Send packet
    socket->send(packet);

    setState(WorldState::ENTERING_WORLD);
    LOG_INFO("CMSG_PLAYER_LOGIN sent, entering world...");
}

void GameHandler::handleLoginSetTimeSpeed(network::Packet& packet) {
    // SMSG_LOGIN_SETTIMESPEED (0x042)
    // Structure: uint32 gameTime, float timeScale
    // gameTime: Game time in seconds since epoch
    // timeScale: Time speed multiplier (typically 0.0166 for 1 day = 1 hour)

    if (packet.getSize() < 8) {
        LOG_WARNING("SMSG_LOGIN_SETTIMESPEED: packet too small (", packet.getSize(), " bytes)");
        return;
    }

    uint32_t gameTimePacked = packet.readUInt32();
    float timeScale = packet.readFloat();

    // Store for celestial/sky system use
    gameTime_ = static_cast<float>(gameTimePacked);
    timeSpeed_ = timeScale;

    LOG_INFO("Server time: gameTime=", gameTime_, "s, timeSpeed=", timeSpeed_);
    LOG_INFO("  (1 game day = ", (1.0f / timeSpeed_) / 60.0f, " real minutes)");
}

void GameHandler::handleLoginVerifyWorld(network::Packet& packet) {
    LOG_INFO("Handling SMSG_LOGIN_VERIFY_WORLD");
    const bool initialWorldEntry = (state == WorldState::ENTERING_WORLD);

    LoginVerifyWorldData data;
    if (!LoginVerifyWorldParser::parse(packet, data)) {
        fail("Failed to parse SMSG_LOGIN_VERIFY_WORLD");
        return;
    }

    if (!data.isValid()) {
        fail("Invalid world entry data");
        return;
    }

    // Successfully entered the world (or teleported)
    currentMapId_ = data.mapId;
    setState(WorldState::IN_WORLD);

    LOG_INFO("========================================");
    LOG_INFO("   SUCCESSFULLY ENTERED WORLD!");
    LOG_INFO("========================================");
    LOG_INFO("Map ID: ", data.mapId);
    LOG_INFO("Position: (", data.x, ", ", data.y, ", ", data.z, ")");
    LOG_INFO("Orientation: ", data.orientation, " radians");
    LOG_INFO("Player is now in the game world");

    // Initialize movement info with world entry position (server → canonical)
    glm::vec3 canonical = core::coords::serverToCanonical(glm::vec3(data.x, data.y, data.z));
    movementInfo.x = canonical.x;
    movementInfo.y = canonical.y;
    movementInfo.z = canonical.z;
    movementInfo.orientation = core::coords::serverToCanonicalYaw(data.orientation);
    movementInfo.flags = 0;
    movementInfo.flags2 = 0;
    movementClockStart_ = std::chrono::steady_clock::now();
    lastMovementTimestampMs_ = 0;
    movementInfo.time = nextMovementTimestampMs();
    resurrectPending_ = false;
    resurrectRequestPending_ = false;
    onTaxiFlight_ = false;
    taxiMountActive_ = false;
    taxiActivatePending_ = false;
    taxiClientActive_ = false;
    taxiClientPath_.clear();
    taxiRecoverPending_ = false;
    taxiStartGrace_ = 0.0f;
    currentMountDisplayId_ = 0;
    taxiMountDisplayId_ = 0;
    if (mountCallback_) {
        mountCallback_(0);
    }

    // Send CMSG_SET_ACTIVE_MOVER (required by some servers)
    if (playerGuid != 0 && socket) {
        auto activeMoverPacket = SetActiveMoverPacket::build(playerGuid);
        socket->send(activeMoverPacket);
        LOG_INFO("Sent CMSG_SET_ACTIVE_MOVER for player 0x", std::hex, playerGuid, std::dec);
    }

    // Notify application to load terrain for this map/position (online mode)
    if (worldEntryCallback_) {
        worldEntryCallback_(data.mapId, data.x, data.y, data.z);
    }

    // Auto-join default chat channels
    autoJoinDefaultChannels();

    // Auto-query guild info on login
    const Character* activeChar = getActiveCharacter();
    if (activeChar && activeChar->hasGuild() && socket) {
        auto gqPacket = GuildQueryPacket::build(activeChar->guildId);
        socket->send(gqPacket);
        auto grPacket = GuildRosterPacket::build();
        socket->send(grPacket);
        LOG_INFO("Auto-queried guild info (guildId=", activeChar->guildId, ")");
    }

    // If we disconnected mid-taxi, attempt to recover to destination after login.
    if (taxiRecoverPending_ && taxiRecoverMapId_ == data.mapId) {
        float dx = movementInfo.x - taxiRecoverPos_.x;
        float dy = movementInfo.y - taxiRecoverPos_.y;
        float dz = movementInfo.z - taxiRecoverPos_.z;
        float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist > 5.0f) {
            // Keep pending until player entity exists; update() will apply.
            LOG_INFO("Taxi recovery pending: dist=", dist);
        } else {
            taxiRecoverPending_ = false;
        }
    }

    if (initialWorldEntry) {
        pendingQuestAcceptTimeouts_.clear();
        pendingQuestAcceptNpcGuids_.clear();
        pendingQuestQueryIds_.clear();
        pendingLoginQuestResync_ = true;
        pendingLoginQuestResyncTimeout_ = 10.0f;
        LOG_INFO("Queued quest log resync for login (from server quest slots)");
    }
}

void GameHandler::handleClientCacheVersion(network::Packet& packet) {
    if (packet.getSize() < 4) {
        LOG_WARNING("SMSG_CLIENTCACHE_VERSION too short: ", packet.getSize(), " bytes");
        return;
    }

    uint32_t version = packet.readUInt32();
    LOG_INFO("SMSG_CLIENTCACHE_VERSION: ", version);
}

void GameHandler::handleTutorialFlags(network::Packet& packet) {
    if (packet.getSize() < 32) {
        LOG_WARNING("SMSG_TUTORIAL_FLAGS too short: ", packet.getSize(), " bytes");
        return;
    }

    std::array<uint32_t, 8> flags{};
    for (uint32_t& v : flags) {
        v = packet.readUInt32();
    }

    LOG_INFO("SMSG_TUTORIAL_FLAGS: [",
             flags[0], ", ", flags[1], ", ", flags[2], ", ", flags[3], ", ",
             flags[4], ", ", flags[5], ", ", flags[6], ", ", flags[7], "]");
}

bool GameHandler::loadWardenCRFile(const std::string& moduleHashHex) {
    wardenCREntries_.clear();

    // Look for .cr file in warden cache
    std::string cacheBase;
#ifdef _WIN32
    if (const char* h = std::getenv("APPDATA")) cacheBase = std::string(h) + "\\wowee\\warden_cache";
    else cacheBase = ".\\warden_cache";
#else
    if (const char* h = std::getenv("HOME")) cacheBase = std::string(h) + "/.local/share/wowee/warden_cache";
    else cacheBase = "./warden_cache";
#endif
    std::string crPath = cacheBase + "/" + moduleHashHex + ".cr";

    std::ifstream crFile(crPath, std::ios::binary);
    if (!crFile) {
        LOG_WARNING("Warden: No .cr file found at ", crPath);
        return false;
    }

    // Get file size
    crFile.seekg(0, std::ios::end);
    auto fileSize = crFile.tellg();
    crFile.seekg(0, std::ios::beg);

    // Header: [4 memoryRead][4 pageScanCheck][9 opcodes] = 17 bytes
    constexpr size_t CR_HEADER_SIZE = 17;
    constexpr size_t CR_ENTRY_SIZE = 68; // seed[16]+reply[20]+clientKey[16]+serverKey[16]

    if (static_cast<size_t>(fileSize) < CR_HEADER_SIZE) {
        LOG_ERROR("Warden: .cr file too small (", fileSize, " bytes)");
        return false;
    }

    // Read header: [4 memoryRead][4 pageScanCheck][9 opcodes]
    crFile.seekg(8); // skip memoryRead + pageScanCheck
    crFile.read(reinterpret_cast<char*>(wardenCheckOpcodes_), 9);
    {
        std::string opcHex;
        // CMaNGOS WindowsScanType order:
        // 0 READ_MEMORY, 1 FIND_MODULE_BY_NAME, 2 FIND_MEM_IMAGE_CODE_BY_HASH,
        // 3 FIND_CODE_BY_HASH, 4 HASH_CLIENT_FILE, 5 GET_LUA_VARIABLE,
        // 6 API_CHECK, 7 FIND_DRIVER_BY_NAME, 8 CHECK_TIMING_VALUES
        const char* names[] = {"MEM","MODULE","PAGE_A","PAGE_B","MPQ","LUA","PROC","DRIVER","TIMING"};
        for (int i = 0; i < 9; i++) {
            char s[16]; snprintf(s, sizeof(s), "%s=0x%02X ", names[i], wardenCheckOpcodes_[i]); opcHex += s;
        }
        LOG_INFO("Warden: Check opcodes: ", opcHex);
    }

    size_t entryCount = (static_cast<size_t>(fileSize) - CR_HEADER_SIZE) / CR_ENTRY_SIZE;
    if (entryCount == 0) {
        LOG_ERROR("Warden: .cr file has no entries");
        return false;
    }

    wardenCREntries_.resize(entryCount);
    for (size_t i = 0; i < entryCount; i++) {
        auto& e = wardenCREntries_[i];
        crFile.read(reinterpret_cast<char*>(e.seed), 16);
        crFile.read(reinterpret_cast<char*>(e.reply), 20);
        crFile.read(reinterpret_cast<char*>(e.clientKey), 16);
        crFile.read(reinterpret_cast<char*>(e.serverKey), 16);
    }

    LOG_INFO("Warden: Loaded ", entryCount, " CR entries from ", crPath);
    return true;
}

void GameHandler::handleWardenData(network::Packet& packet) {
    const auto& data = packet.getData();
    if (!wardenGateSeen_) {
        wardenGateSeen_ = true;
        wardenGateElapsed_ = 0.0f;
        wardenGateNextStatusLog_ = 2.0f;
        wardenPacketsAfterGate_ = 0;
    }

    // Initialize Warden crypto from session key on first packet
    if (!wardenCrypto_) {
        wardenCrypto_ = std::make_unique<WardenCrypto>();
        if (sessionKey.size() != 40) {
            LOG_ERROR("Warden: No valid session key (size=", sessionKey.size(), "), cannot init crypto");
            wardenCrypto_.reset();
            return;
        }
        if (!wardenCrypto_->initFromSessionKey(sessionKey)) {
            LOG_ERROR("Warden: Failed to initialize crypto from session key");
            wardenCrypto_.reset();
            return;
        }
        wardenState_ = WardenState::WAIT_MODULE_USE;
    }

    // Decrypt the payload
    std::vector<uint8_t> decrypted = wardenCrypto_->decrypt(data);

    // Log decrypted data
    {
        std::string hex;
        size_t logSize = std::min(decrypted.size(), size_t(256));
        hex.reserve(logSize * 3);
        for (size_t i = 0; i < logSize; ++i) {
            char b[4]; snprintf(b, sizeof(b), "%02x ", decrypted[i]); hex += b;
        }
        if (decrypted.size() > 64)
            hex += "... (" + std::to_string(decrypted.size() - 64) + " more)";
        LOG_INFO("Warden: Decrypted (", decrypted.size(), " bytes): ", hex);
    }

    if (decrypted.empty()) {
        LOG_WARNING("Warden: Empty decrypted payload");
        return;
    }

    uint8_t wardenOpcode = decrypted[0];

    // Helper to send an encrypted Warden response
    auto sendWardenResponse = [&](const std::vector<uint8_t>& plaintext) {
        std::vector<uint8_t> encrypted = wardenCrypto_->encrypt(plaintext);
        network::Packet response(wireOpcode(Opcode::CMSG_WARDEN_DATA));
        for (uint8_t byte : encrypted) {
            response.writeUInt8(byte);
        }
        if (socket && socket->isConnected()) {
            socket->send(response);
            LOG_INFO("Warden: Sent response (", plaintext.size(), " bytes plaintext)");
        }
    };

    switch (wardenOpcode) {
        case 0x00: { // WARDEN_SMSG_MODULE_USE
            // Format: [1 opcode][16 moduleHash][16 moduleKey][4 moduleSize]
            if (decrypted.size() < 37) {
                LOG_ERROR("Warden: MODULE_USE too short (", decrypted.size(), " bytes, need 37)");
                return;
            }

            wardenModuleHash_.assign(decrypted.begin() + 1, decrypted.begin() + 17);
            wardenModuleKey_.assign(decrypted.begin() + 17, decrypted.begin() + 33);
            wardenModuleSize_ = static_cast<uint32_t>(decrypted[33])
                              | (static_cast<uint32_t>(decrypted[34]) << 8)
                              | (static_cast<uint32_t>(decrypted[35]) << 16)
                              | (static_cast<uint32_t>(decrypted[36]) << 24);
            wardenModuleData_.clear();

            {
                std::string hashHex;
                for (auto b : wardenModuleHash_) { char s[4]; snprintf(s, 4, "%02x", b); hashHex += s; }
                LOG_INFO("Warden: MODULE_USE hash=", hashHex, " size=", wardenModuleSize_);

                // Try to load pre-computed challenge/response entries
                loadWardenCRFile(hashHex);
            }

            // Respond with MODULE_MISSING (opcode 0x00) to request the module data
            std::vector<uint8_t> resp = { 0x00 }; // WARDEN_CMSG_MODULE_MISSING
            sendWardenResponse(resp);
            wardenState_ = WardenState::WAIT_MODULE_CACHE;
            LOG_INFO("Warden: Sent MODULE_MISSING, waiting for module data chunks");
            break;
        }

        case 0x01: { // WARDEN_SMSG_MODULE_CACHE (module data chunk)
            // Format: [1 opcode][2 chunkSize LE][chunkSize bytes data]
            if (decrypted.size() < 3) {
                LOG_ERROR("Warden: MODULE_CACHE too short");
                return;
            }

            uint16_t chunkSize = static_cast<uint16_t>(decrypted[1])
                               | (static_cast<uint16_t>(decrypted[2]) << 8);

            if (decrypted.size() < 3u + chunkSize) {
                LOG_ERROR("Warden: MODULE_CACHE chunk truncated (claimed ", chunkSize,
                          ", have ", decrypted.size() - 3, ")");
                return;
            }

            wardenModuleData_.insert(wardenModuleData_.end(),
                                     decrypted.begin() + 3,
                                     decrypted.begin() + 3 + chunkSize);

            LOG_INFO("Warden: MODULE_CACHE chunk ", chunkSize, " bytes, total ",
                     wardenModuleData_.size(), "/", wardenModuleSize_);

            // Check if module download is complete
            if (wardenModuleData_.size() >= wardenModuleSize_) {
                LOG_INFO("Warden: Module download complete (",
                         wardenModuleData_.size(), " bytes)");
                wardenState_ = WardenState::WAIT_HASH_REQUEST;

                // Cache raw module to disk
                {
#ifdef _WIN32
                    std::string cacheDir;
                    if (const char* h = std::getenv("APPDATA")) cacheDir = std::string(h) + "\\wowee\\warden_cache";
                    else cacheDir = ".\\warden_cache";
#else
                    std::string cacheDir;
                    if (const char* h = std::getenv("HOME")) cacheDir = std::string(h) + "/.local/share/wowee/warden_cache";
                    else cacheDir = "./warden_cache";
#endif
                    std::filesystem::create_directories(cacheDir);

                    std::string hashHex;
                    for (auto b : wardenModuleHash_) { char s[4]; snprintf(s, 4, "%02x", b); hashHex += s; }
                    std::string cachePath = cacheDir + "/" + hashHex + ".wdn";

                    std::ofstream wf(cachePath, std::ios::binary);
                    if (wf) {
                        wf.write(reinterpret_cast<const char*>(wardenModuleData_.data()), wardenModuleData_.size());
                        LOG_INFO("Warden: Cached module to ", cachePath);
                    }
                }

                // Load the module (decrypt, decompress, parse, relocate)
                wardenLoadedModule_ = std::make_shared<WardenModule>();
                if (wardenLoadedModule_->load(wardenModuleData_, wardenModuleHash_, wardenModuleKey_)) { // codeql[cpp/weak-cryptographic-algorithm]
                    LOG_INFO("Warden: Module loaded successfully (image size=",
                             wardenLoadedModule_->getModuleSize(), " bytes)");
                } else {
                    LOG_ERROR("Warden: Module loading FAILED");
                    wardenLoadedModule_.reset();
                }

                // Send MODULE_OK (opcode 0x01)
                std::vector<uint8_t> resp = { 0x01 }; // WARDEN_CMSG_MODULE_OK
                sendWardenResponse(resp);
                LOG_INFO("Warden: Sent MODULE_OK");
            }
            // No response for intermediate chunks
            break;
        }

        case 0x05: { // WARDEN_SMSG_HASH_REQUEST
            // Format: [1 opcode][16 seed]
            if (decrypted.size() < 17) {
                LOG_ERROR("Warden: HASH_REQUEST too short (", decrypted.size(), " bytes, need 17)");
                return;
            }

            std::vector<uint8_t> seed(decrypted.begin() + 1, decrypted.begin() + 17);

            // --- Try CR lookup (pre-computed challenge/response entries) ---
            if (!wardenCREntries_.empty()) {
                const WardenCREntry* match = nullptr;
                for (const auto& entry : wardenCREntries_) {
                    if (std::memcmp(entry.seed, seed.data(), 16) == 0) {
                        match = &entry;
                        break;
                    }
                }

                if (match) {
                    LOG_INFO("Warden: Found matching CR entry for seed");

                    // Log the reply we're sending
                    {
                        std::string replyHex;
                        for (int i = 0; i < 20; i++) {
                            char s[4]; snprintf(s, 4, "%02x", match->reply[i]); replyHex += s;
                        }
                        LOG_INFO("Warden: Sending pre-computed reply=", replyHex);
                    }

                    // Send HASH_RESULT (opcode 0x04 + 20-byte reply)
                    std::vector<uint8_t> resp;
                    resp.push_back(0x04);
                    resp.insert(resp.end(), match->reply, match->reply + 20);
                    sendWardenResponse(resp);

                    // Switch to new RC4 keys from the CR entry
                    // clientKey = encrypt (client→server), serverKey = decrypt (server→client)
                    std::vector<uint8_t> newEncryptKey(match->clientKey, match->clientKey + 16);
                    std::vector<uint8_t> newDecryptKey(match->serverKey, match->serverKey + 16);
                    wardenCrypto_->replaceKeys(newEncryptKey, newDecryptKey);

                    LOG_INFO("Warden: Switched to CR key set");

                    wardenState_ = WardenState::WAIT_CHECKS;
                    break;
                } else {
                    LOG_WARNING("Warden: Seed not found in ", wardenCREntries_.size(), " CR entries");
                }
            }

            // --- Fallback: compute hash from loaded module ---
            LOG_WARNING("Warden: No CR match, computing hash from loaded module");

            if (!wardenLoadedModule_ || !wardenLoadedModule_->isLoaded()) {
                LOG_ERROR("Warden: No loaded module and no CR match — cannot compute hash");
                wardenState_ = WardenState::WAIT_CHECKS;
                break;
            }

            {
                const uint8_t* moduleImage = static_cast<const uint8_t*>(wardenLoadedModule_->getModuleMemory());
                size_t moduleImageSize = wardenLoadedModule_->getModuleSize();
                const auto& decompressedData = wardenLoadedModule_->getDecompressedData();

                // --- Empirical test: try multiple SHA1 computations and check against first CR entry ---
                if (!wardenCREntries_.empty()) {
                    const auto& firstCR = wardenCREntries_[0];
                    std::string expectedHex;
                    for (int i = 0; i < 20; i++) { char s[4]; snprintf(s, 4, "%02x", firstCR.reply[i]); expectedHex += s; }
                    LOG_INFO("Warden: Empirical test — expected reply from CR[0]=", expectedHex);

                    // Test 1: SHA1(moduleImage)
                    {
                        std::vector<uint8_t> data(moduleImage, moduleImage + moduleImageSize);
                        auto h = auth::Crypto::sha1(data);
                        bool match = (std::memcmp(h.data(), firstCR.reply, 20) == 0);
                        std::string hex; for (auto b : h) { char s[4]; snprintf(s, 4, "%02x", b); hex += s; }
                        LOG_INFO("Warden:   SHA1(moduleImage)=", hex, match ? " MATCH!" : "");
                    }
                    // Test 2: SHA1(seed || moduleImage)
                    {
                        std::vector<uint8_t> data;
                        data.insert(data.end(), seed.begin(), seed.end());
                        data.insert(data.end(), moduleImage, moduleImage + moduleImageSize);
                        auto h = auth::Crypto::sha1(data);
                        bool match = (std::memcmp(h.data(), firstCR.reply, 20) == 0);
                        std::string hex; for (auto b : h) { char s[4]; snprintf(s, 4, "%02x", b); hex += s; }
                        LOG_INFO("Warden:   SHA1(seed||image)=", hex, match ? " MATCH!" : "");
                    }
                    // Test 3: SHA1(moduleImage || seed)
                    {
                        std::vector<uint8_t> data(moduleImage, moduleImage + moduleImageSize);
                        data.insert(data.end(), seed.begin(), seed.end());
                        auto h = auth::Crypto::sha1(data);
                        bool match = (std::memcmp(h.data(), firstCR.reply, 20) == 0);
                        std::string hex; for (auto b : h) { char s[4]; snprintf(s, 4, "%02x", b); hex += s; }
                        LOG_INFO("Warden:   SHA1(image||seed)=", hex, match ? " MATCH!" : "");
                    }
                    // Test 4: SHA1(decompressedData)
                    {
                        auto h = auth::Crypto::sha1(decompressedData);
                        bool match = (std::memcmp(h.data(), firstCR.reply, 20) == 0);
                        std::string hex; for (auto b : h) { char s[4]; snprintf(s, 4, "%02x", b); hex += s; }
                        LOG_INFO("Warden:   SHA1(decompressed)=", hex, match ? " MATCH!" : "");
                    }
                    // Test 5: SHA1(rawModuleData)
                    {
                        auto h = auth::Crypto::sha1(wardenModuleData_);
                        bool match = (std::memcmp(h.data(), firstCR.reply, 20) == 0);
                        std::string hex; for (auto b : h) { char s[4]; snprintf(s, 4, "%02x", b); hex += s; }
                        LOG_INFO("Warden:   SHA1(rawModule)=", hex, match ? " MATCH!" : "");
                    }
                    // Test 6: Check if all CR replies are the same (constant hash)
                    {
                        bool allSame = true;
                        for (size_t i = 1; i < wardenCREntries_.size(); i++) {
                            if (std::memcmp(wardenCREntries_[i].reply, firstCR.reply, 20) != 0) {
                                allSame = false;
                                break;
                            }
                        }
                        LOG_INFO("Warden:   All ", wardenCREntries_.size(), " CR replies identical? ", allSame ? "YES" : "NO");
                    }
                }

                // --- Compute the hash: SHA1(moduleImage) is the most likely candidate ---
                // The module's hash response is typically SHA1 of the loaded module image.
                // This is a constant per module (seed is not used in the hash, only for key derivation).
                std::vector<uint8_t> imageData(moduleImage, moduleImage + moduleImageSize);
                auto reply = auth::Crypto::sha1(imageData);

                {
                    std::string hex;
                    for (auto b : reply) { char s[4]; snprintf(s, 4, "%02x", b); hex += s; }
                    LOG_INFO("Warden: Sending SHA1(moduleImage)=", hex);
                }

                // Send HASH_RESULT (opcode 0x04 + 20-byte hash)
                std::vector<uint8_t> resp;
                resp.push_back(0x04);
                resp.insert(resp.end(), reply.begin(), reply.end());
                sendWardenResponse(resp);

                // Derive new RC4 keys from the seed using SHA1Randx
                std::vector<uint8_t> seedVec(seed.begin(), seed.end());
                // Pad seed to at least 2 bytes for SHA1Randx split
                // SHA1Randx splits input in half: first_half and second_half
                uint8_t newEncryptKey[16], newDecryptKey[16];
                WardenCrypto::sha1RandxGenerate(seedVec, newEncryptKey, newDecryptKey);

                std::vector<uint8_t> ek(newEncryptKey, newEncryptKey + 16);
                std::vector<uint8_t> dk(newDecryptKey, newDecryptKey + 16);
                wardenCrypto_->replaceKeys(ek, dk);
                for (auto& b : newEncryptKey) b = 0;
                for (auto& b : newDecryptKey) b = 0;
                LOG_INFO("Warden: Derived and applied key update from seed");
            }

            wardenState_ = WardenState::WAIT_CHECKS;
            break;
        }

        case 0x02: { // WARDEN_SMSG_CHEAT_CHECKS_REQUEST
            LOG_INFO("Warden: CHEAT_CHECKS_REQUEST (", decrypted.size(), " bytes)");

            if (decrypted.size() < 3) {
                LOG_ERROR("Warden: CHEAT_CHECKS_REQUEST too short");
                break;
            }

            // --- Parse string table ---
            // Format: [1 opcode][string table: (len+data)*][0x00 end][check data][xorByte]
            size_t pos = 1;
            std::vector<std::string> strings;
            while (pos < decrypted.size()) {
                uint8_t slen = decrypted[pos++];
                if (slen == 0) break; // end of string table
                if (pos + slen > decrypted.size()) break;
                strings.emplace_back(reinterpret_cast<const char*>(decrypted.data() + pos), slen);
                pos += slen;
            }
            LOG_INFO("Warden: String table: ", strings.size(), " entries");
            for (size_t i = 0; i < strings.size(); i++) {
                LOG_INFO("Warden:   [", i, "] = \"", strings[i], "\"");
            }

            // XOR byte is the last byte of the packet
            uint8_t xorByte = decrypted.back();
            LOG_INFO("Warden: XOR byte = 0x", [&]{ char s[4]; snprintf(s,4,"%02x",xorByte); return std::string(s); }());

            // Check type enum indices
            enum CheckType { CT_MEM=0, CT_PAGE_A=1, CT_PAGE_B=2, CT_MPQ=3, CT_LUA=4,
                             CT_DRIVER=5, CT_TIMING=6, CT_PROC=7, CT_MODULE=8, CT_UNKNOWN=9 };
            const char* checkTypeNames[] = {"MEM","PAGE_A","PAGE_B","MPQ","LUA","DRIVER","TIMING","PROC","MODULE","UNKNOWN"};
            size_t checkEnd = decrypted.size() - 1; // exclude xorByte

            auto decodeCheckType = [&](uint8_t raw) -> CheckType {
                uint8_t decoded = raw ^ xorByte;
                if (decoded == wardenCheckOpcodes_[0]) return CT_MEM;    // READ_MEMORY
                if (decoded == wardenCheckOpcodes_[1]) return CT_MODULE; // FIND_MODULE_BY_NAME
                if (decoded == wardenCheckOpcodes_[2]) return CT_PAGE_A; // FIND_MEM_IMAGE_CODE_BY_HASH
                if (decoded == wardenCheckOpcodes_[3]) return CT_PAGE_B; // FIND_CODE_BY_HASH
                if (decoded == wardenCheckOpcodes_[4]) return CT_MPQ;    // HASH_CLIENT_FILE
                if (decoded == wardenCheckOpcodes_[5]) return CT_LUA;    // GET_LUA_VARIABLE
                if (decoded == wardenCheckOpcodes_[6]) return CT_PROC;   // API_CHECK
                if (decoded == wardenCheckOpcodes_[7]) return CT_DRIVER; // FIND_DRIVER_BY_NAME
                if (decoded == wardenCheckOpcodes_[8]) return CT_TIMING; // CHECK_TIMING_VALUES
                return CT_UNKNOWN;
            };
            auto isKnownWantedCodeScan = [&](const uint8_t seedBytes[4], const uint8_t reqHash[20],
                                            uint32_t offset, uint8_t length) -> bool {
                auto hashPattern = [&](const uint8_t* pattern, size_t patternLen) {
                    uint8_t out[SHA_DIGEST_LENGTH];
                    unsigned int outLen = 0;
                    HMAC(EVP_sha1(),
                         seedBytes, 4,
                         pattern, patternLen,
                         out, &outLen);
                    return outLen == SHA_DIGEST_LENGTH && std::memcmp(out, reqHash, SHA_DIGEST_LENGTH) == 0;
                };

                // DB sanity check: "Warden packet process code search sanity check" (id=85)
                static const uint8_t kPacketProcessSanityPattern[] = {
                    0x33, 0xD2, 0x33, 0xC9, 0xE8, 0x87, 0x07, 0x1B, 0x00, 0xE8
                };
                if (offset == 13856 && length == sizeof(kPacketProcessSanityPattern) &&
                    hashPattern(kPacketProcessSanityPattern, sizeof(kPacketProcessSanityPattern))) {
                    return true;
                }

                // Scripted sanity check: "Warden Memory Read check" in wardenwin.cpp
                static const uint8_t kWardenMemoryReadPattern[] = {
                    0x56, 0x57, 0xFC, 0x8B, 0x54, 0x24, 0x14, 0x8B,
                    0x74, 0x24, 0x10, 0x8B, 0x44, 0x24, 0x0C, 0x8B,
                    0xCA, 0x8B, 0xF8, 0xC1, 0xE9, 0x02, 0x74, 0x02,
                    0xF3, 0xA5, 0xB1, 0x03, 0x23, 0xCA, 0x74, 0x02,
                    0xF3, 0xA4, 0x5F, 0x5E, 0xC3
                };
                if (length == sizeof(kWardenMemoryReadPattern) &&
                    hashPattern(kWardenMemoryReadPattern, sizeof(kWardenMemoryReadPattern))) {
                    return true;
                }

                return false;
            };
            auto resolveWardenString = [&](uint8_t oneBasedIndex) -> std::string {
                if (oneBasedIndex == 0) return std::string();
                size_t idx = static_cast<size_t>(oneBasedIndex - 1);
                if (idx >= strings.size()) return std::string();
                return strings[idx];
            };
            auto requestSizes = [&](CheckType ct) {
                switch (ct) {
                    case CT_TIMING: return std::vector<size_t>{0};
                    case CT_MEM:    return std::vector<size_t>{6};
                    case CT_PAGE_A: return std::vector<size_t>{24, 29};
                    case CT_PAGE_B: return std::vector<size_t>{24, 29};
                    case CT_MPQ:    return std::vector<size_t>{1};
                    case CT_LUA:    return std::vector<size_t>{1};
                    case CT_DRIVER: return std::vector<size_t>{25};
                    case CT_PROC:   return std::vector<size_t>{30};
                    case CT_MODULE: return std::vector<size_t>{24};
                    default:        return std::vector<size_t>{};
                }
            };
            std::unordered_map<size_t, bool> parseMemo;
            std::function<bool(size_t)> canParseFrom = [&](size_t checkPos) -> bool {
                if (checkPos == checkEnd) return true;
                if (checkPos > checkEnd) return false;
                auto it = parseMemo.find(checkPos);
                if (it != parseMemo.end()) return it->second;

                CheckType ct = decodeCheckType(decrypted[checkPos]);
                if (ct == CT_UNKNOWN) {
                    parseMemo[checkPos] = false;
                    return false;
                }

                size_t payloadPos = checkPos + 1;
                for (size_t reqSize : requestSizes(ct)) {
                    if (payloadPos + reqSize > checkEnd) continue;
                    if (canParseFrom(payloadPos + reqSize)) {
                        parseMemo[checkPos] = true;
                        return true;
                    }
                }

                parseMemo[checkPos] = false;
                return false;
            };
            auto isBoundaryAfter = [&](size_t start, size_t consume) -> bool {
                size_t next = start + consume;
                if (next == checkEnd) return true;
                if (next > checkEnd) return false;
                return decodeCheckType(decrypted[next]) != CT_UNKNOWN;
            };

            // --- Parse check entries and build response ---
            std::vector<uint8_t> resultData;
            int checkCount = 0;

            while (pos < checkEnd) {
                CheckType ct = decodeCheckType(decrypted[pos]);
                pos++;
                checkCount++;

                LOG_INFO("Warden: Check #", checkCount, " type=", checkTypeNames[ct],
                         " at offset ", pos - 1);

                switch (ct) {
                    case CT_TIMING: {
                        // No additional request data
                        // Response: [uint8 result=1][uint32 ticks]
                        resultData.push_back(0x01);
                        uint32_t ticks = static_cast<uint32_t>(
                            std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now().time_since_epoch()).count());
                        resultData.push_back(ticks & 0xFF);
                        resultData.push_back((ticks >> 8) & 0xFF);
                        resultData.push_back((ticks >> 16) & 0xFF);
                        resultData.push_back((ticks >> 24) & 0xFF);
                        break;
                    }
                    case CT_MEM: {
                        // Request: [1 stringIdx][4 offset][1 length]
                        if (pos + 6 > checkEnd) { pos = checkEnd; break; }
                        uint8_t strIdx = decrypted[pos++];
                        std::string moduleName = resolveWardenString(strIdx);
                        uint32_t offset = decrypted[pos] | (uint32_t(decrypted[pos+1])<<8)
                                        | (uint32_t(decrypted[pos+2])<<16) | (uint32_t(decrypted[pos+3])<<24);
                        pos += 4;
                        uint8_t readLen = decrypted[pos++];
                        LOG_INFO("Warden:   MEM offset=0x", [&]{char s[12];snprintf(s,12,"%08x",offset);return std::string(s);}(),
                                 " len=", (int)readLen);
                        if (!moduleName.empty()) {
                            LOG_INFO("Warden:   MEM module=\"", moduleName, "\"");
                        }

                        // Lazy-load WoW.exe PE image on first MEM_CHECK
                        if (!wardenMemory_) {
                            wardenMemory_ = std::make_unique<WardenMemory>();
                            if (!wardenMemory_->load(static_cast<uint16_t>(build))) {
                                LOG_WARNING("Warden: Could not load WoW.exe for MEM_CHECK");
                            }
                        }

                        // Read bytes from PE image (includes patched runtime globals)
                        std::vector<uint8_t> memBuf(readLen, 0);
                        if (wardenMemory_->isLoaded() && wardenMemory_->readMemory(offset, readLen, memBuf.data())) {
                            LOG_INFO("Warden:   MEM_CHECK served from PE image");
                        } else {
                            LOG_WARNING("Warden:   MEM_CHECK fallback to zeros for 0x",
                                        [&]{char s[12];snprintf(s,12,"%08x",offset);return std::string(s);}());
                        }
                        resultData.push_back(0x00);
                        resultData.insert(resultData.end(), memBuf.begin(), memBuf.end());
                        break;
                    }
                    case CT_PAGE_A: {
                        // Classic has seen two PAGE_A layouts in the wild:
                        // short: [4 seed][20 sha1] = 24 bytes
                        // long:  [4 seed][20 sha1][4 addr][1 len] = 29 bytes
                        // Prefer the variant that allows the full remaining stream to parse.
                        constexpr size_t kPageAShort = 24;
                        constexpr size_t kPageALong = 29;
                        size_t consume = 0;

                        if (pos + kPageAShort <= checkEnd && canParseFrom(pos + kPageAShort)) {
                            consume = kPageAShort;
                        }
                        if (pos + kPageALong <= checkEnd && canParseFrom(pos + kPageALong) && consume == 0) {
                            consume = kPageALong;
                        }
                        if (consume == 0 && isBoundaryAfter(pos, kPageAShort)) consume = kPageAShort;
                        if (consume == 0 && isBoundaryAfter(pos, kPageALong)) consume = kPageALong;

                        if (consume == 0) {
                            size_t remaining = checkEnd - pos;
                            if (remaining >= kPageAShort && remaining < kPageALong) consume = kPageAShort;
                            else if (remaining >= kPageALong) consume = kPageALong;
                            else {
                                LOG_WARNING("Warden:   PAGE_A check truncated (remaining=", remaining,
                                            "), consuming remainder");
                                pos = checkEnd;
                                resultData.push_back(0x00);
                                break;
                            }
                        }

                        uint8_t pageResult = 0x00;
                        if (consume >= 29) {
                            const uint8_t* p = decrypted.data() + pos;
                            uint8_t seedBytes[4] = { p[0], p[1], p[2], p[3] };
                            uint8_t reqHash[20];
                            std::memcpy(reqHash, p + 4, 20);
                            uint32_t off = uint32_t(p[24]) | (uint32_t(p[25]) << 8) |
                                           (uint32_t(p[26]) << 16) | (uint32_t(p[27]) << 24);
                            uint8_t len = p[28];
                            if (isKnownWantedCodeScan(seedBytes, reqHash, off, len)) {
                                pageResult = 0x4A; // PatternFound
                            }
                        }
                        LOG_INFO("Warden:   PAGE_A request bytes=", consume,
                                 " result=0x", [&]{char s[4];snprintf(s,4,"%02x",pageResult);return std::string(s);}());
                        pos += consume;
                        resultData.push_back(pageResult);
                        break;
                    }
                    case CT_PAGE_B: {
                        constexpr size_t kPageBShort = 24;
                        constexpr size_t kPageBLong = 29;
                        size_t consume = 0;

                        if (pos + kPageBShort <= checkEnd && canParseFrom(pos + kPageBShort)) {
                            consume = kPageBShort;
                        }
                        if (pos + kPageBLong <= checkEnd && canParseFrom(pos + kPageBLong) && consume == 0) {
                            consume = kPageBLong;
                        }
                        if (consume == 0 && isBoundaryAfter(pos, kPageBShort)) consume = kPageBShort;
                        if (consume == 0 && isBoundaryAfter(pos, kPageBLong)) consume = kPageBLong;

                        if (consume == 0) {
                            size_t remaining = checkEnd - pos;
                            if (remaining >= kPageBShort && remaining < kPageBLong) consume = kPageBShort;
                            else if (remaining >= kPageBLong) consume = kPageBLong;
                            else { pos = checkEnd; break; }
                        }
                        uint8_t pageResult = 0x00;
                        if (consume >= 29) {
                            const uint8_t* p = decrypted.data() + pos;
                            uint8_t seedBytes[4] = { p[0], p[1], p[2], p[3] };
                            uint8_t reqHash[20];
                            std::memcpy(reqHash, p + 4, 20);
                            uint32_t off = uint32_t(p[24]) | (uint32_t(p[25]) << 8) |
                                           (uint32_t(p[26]) << 16) | (uint32_t(p[27]) << 24);
                            uint8_t len = p[28];
                            if (isKnownWantedCodeScan(seedBytes, reqHash, off, len)) {
                                pageResult = 0x4A; // PatternFound
                            }
                        }
                        LOG_INFO("Warden:   PAGE_B request bytes=", consume,
                                 " result=0x", [&]{char s[4];snprintf(s,4,"%02x",pageResult);return std::string(s);}());
                        pos += consume;
                        resultData.push_back(pageResult);
                        break;
                    }
                    case CT_MPQ: {
                        // HASH_CLIENT_FILE request: [1 stringIdx]
                        if (pos + 1 > checkEnd) { pos = checkEnd; break; }
                        uint8_t strIdx = decrypted[pos++];
                        std::string filePath = resolveWardenString(strIdx);
                        LOG_INFO("Warden:   MPQ file=\"", (filePath.empty() ? "?" : filePath), "\"");

                        bool found = false;
                        std::vector<uint8_t> hash(20, 0);
                        if (!filePath.empty()) {
                            std::string normalizedPath = asciiLower(filePath);
                            std::replace(normalizedPath.begin(), normalizedPath.end(), '/', '\\');
                            auto knownIt = knownDoorHashes().find(normalizedPath);
                            if (knownIt != knownDoorHashes().end()) {
                                found = true;
                                hash.assign(knownIt->second.begin(), knownIt->second.end());
                            }

                            auto* am = core::Application::getInstance().getAssetManager();
                            if (am && am->isInitialized() && !found) {
                                // Use a case-insensitive direct filesystem resolution first.
                                // Manifest entries may point at uppercase duplicate trees with
                                // different content/hashes than canonical client files.
                                std::vector<uint8_t> fileData;
                                std::string resolvedFsPath =
                                    resolveCaseInsensitiveDataPath(am->getDataPath(), filePath);
                                if (!resolvedFsPath.empty()) {
                                    fileData = readFileBinary(resolvedFsPath);
                                }
                                if (fileData.empty()) {
                                    fileData = am->readFile(filePath);
                                }

                                if (!fileData.empty()) {
                                    found = true;
                                    hash = auth::Crypto::sha1(fileData);
                                }
                            }
                        }

                        // Response: [uint8 result][20 sha1]
                        // result=0 => found/success, result=1 => not found/failure
                        resultData.push_back(found ? 0x00 : 0x01);
                        resultData.insert(resultData.end(), hash.begin(), hash.end());
                        break;
                    }
                    case CT_LUA: {
                        // Request: [1 stringIdx]
                        if (pos + 1 > checkEnd) { pos = checkEnd; break; }
                        uint8_t strIdx = decrypted[pos++];
                        std::string luaVar = resolveWardenString(strIdx);
                        LOG_INFO("Warden:   LUA str=\"", (luaVar.empty() ? "?" : luaVar), "\"");
                        // Response: [uint8 result=0][uint16 len=0]
                        // Lua string doesn't exist
                        resultData.push_back(0x01); // not found
                        break;
                    }
                    case CT_DRIVER: {
                        // Request: [4 seed][20 sha1][1 stringIdx]
                        if (pos + 25 > checkEnd) { pos = checkEnd; break; }
                        pos += 24; // skip seed + sha1
                        uint8_t strIdx = decrypted[pos++];
                        std::string driverName = resolveWardenString(strIdx);
                        LOG_INFO("Warden:   DRIVER=\"", (driverName.empty() ? "?" : driverName), "\"");
                        // Response: [uint8 result=1] (driver NOT found = clean)
                        resultData.push_back(0x01);
                        break;
                    }
                    case CT_MODULE: {
                        // FIND_MODULE_BY_NAME request: [4 seed][20 sha1] = 24 bytes
                        int moduleSize = 24;
                        if (pos + moduleSize > checkEnd) {
                            size_t remaining = checkEnd - pos;
                            LOG_WARNING("Warden:   MODULE check truncated (remaining=", remaining,
                                        ", expected=", moduleSize, "), consuming remainder");
                            pos = checkEnd;
                        } else {
                            const uint8_t* p = decrypted.data() + pos;
                            uint8_t seedBytes[4] = { p[0], p[1], p[2], p[3] };
                            uint8_t reqHash[20];
                            std::memcpy(reqHash, p + 4, 20);
                            pos += moduleSize;

                            // CMaNGOS uppercases module names before hashing.
                            // DB module scans:
                            //   KERNEL32.DLL (wanted=true)
                            //   WPESPY.DLL / SPEEDHACK-I386.DLL / TAMIA.DLL (wanted=false)
                            bool shouldReportFound = false;
                            if (hmacSha1Matches(seedBytes, "KERNEL32.DLL", reqHash)) {
                                shouldReportFound = true;
                            } else if (hmacSha1Matches(seedBytes, "WPESPY.DLL", reqHash) ||
                                       hmacSha1Matches(seedBytes, "SPEEDHACK-I386.DLL", reqHash) ||
                                       hmacSha1Matches(seedBytes, "TAMIA.DLL", reqHash)) {
                                shouldReportFound = false;
                            }
                            resultData.push_back(shouldReportFound ? 0x4A : 0x01);
                            break;
                        }
                        // Truncated module request fallback: module NOT loaded = clean
                        resultData.push_back(0x01);
                        break;
                    }
                    case CT_PROC: {
                        // API_CHECK request:
                        // [4 seed][20 sha1][1 stringIdx][1 stringIdx2][4 offset] = 30 bytes
                        int procSize = 30;
                        if (pos + procSize > checkEnd) { pos = checkEnd; break; }
                        pos += procSize;
                        // Response: [uint8 result=1] (proc NOT found = clean)
                        resultData.push_back(0x01);
                        break;
                    }
                    default: {
                        LOG_WARNING("Warden: Unknown check type, cannot parse remaining");
                        pos = checkEnd; // stop parsing
                        break;
                    }
                }
            }

            LOG_INFO("Warden: Parsed ", checkCount, " checks, result data size=", resultData.size());

            // --- Compute checksum: XOR of 5 uint32s from SHA1(resultData) ---
            auto resultHash = auth::Crypto::sha1(resultData);
            uint32_t checksum = 0;
            for (int i = 0; i < 5; i++) {
                uint32_t word = resultHash[i*4]
                              | (uint32_t(resultHash[i*4+1]) << 8)
                              | (uint32_t(resultHash[i*4+2]) << 16)
                              | (uint32_t(resultHash[i*4+3]) << 24);
                checksum ^= word;
            }

            // --- Build response: [0x02][uint16 length][uint32 checksum][resultData] ---
            uint16_t resultLen = static_cast<uint16_t>(resultData.size());
            std::vector<uint8_t> resp;
            resp.push_back(0x02);
            resp.push_back(resultLen & 0xFF);
            resp.push_back((resultLen >> 8) & 0xFF);
            resp.push_back(checksum & 0xFF);
            resp.push_back((checksum >> 8) & 0xFF);
            resp.push_back((checksum >> 16) & 0xFF);
            resp.push_back((checksum >> 24) & 0xFF);
            resp.insert(resp.end(), resultData.begin(), resultData.end());
            sendWardenResponse(resp);
            LOG_INFO("Warden: Sent CHEAT_CHECKS_RESULT (", resp.size(), " bytes, ",
                     checkCount, " checks, checksum=0x",
                     [&]{char s[12];snprintf(s,12,"%08x",checksum);return std::string(s);}(), ")");
            break;
        }

        case 0x03: // WARDEN_SMSG_MODULE_INITIALIZE
            LOG_INFO("Warden: MODULE_INITIALIZE (", decrypted.size(), " bytes, no response needed)");
            break;

        default:
            LOG_INFO("Warden: Unknown opcode 0x", std::hex, (int)wardenOpcode, std::dec,
                     " (state=", (int)wardenState_, ", size=", decrypted.size(), ")");
            break;
    }
}

void GameHandler::handleAccountDataTimes(network::Packet& packet) {
    LOG_DEBUG("Handling SMSG_ACCOUNT_DATA_TIMES");

    AccountDataTimesData data;
    if (!AccountDataTimesParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_ACCOUNT_DATA_TIMES");
        return;
    }

    LOG_DEBUG("Account data times received (server time: ", data.serverTime, ")");
}

void GameHandler::handleMotd(network::Packet& packet) {
    LOG_INFO("Handling SMSG_MOTD");

    MotdData data;
    if (!MotdParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_MOTD");
        return;
    }

    if (!data.isEmpty()) {
        LOG_INFO("========================================");
        LOG_INFO("   MESSAGE OF THE DAY");
        LOG_INFO("========================================");
        for (const auto& line : data.lines) {
            LOG_INFO(line);
            addSystemChatMessage(std::string("MOTD: ") + line);
        }
        // Add a visual separator after MOTD block so subsequent messages don't
        // appear glued to the last MOTD line.
        MessageChatData spacer;
        spacer.type = ChatType::SYSTEM;
        spacer.language = ChatLanguage::UNIVERSAL;
        spacer.message = "";
        addLocalChatMessage(spacer);
        LOG_INFO("========================================");
    }
}

void GameHandler::handleNotification(network::Packet& packet) {
    // SMSG_NOTIFICATION: single null-terminated string
    std::string message = packet.readString();
    if (!message.empty()) {
        LOG_INFO("Server notification: ", message);
        addSystemChatMessage(message);
    }
}

void GameHandler::sendPing() {
    if (state != WorldState::IN_WORLD) {
        return;
    }

    // Increment sequence number
    pingSequence++;

    LOG_DEBUG("Sending CMSG_PING (heartbeat)");
    LOG_DEBUG("  Sequence: ", pingSequence);

    // Build and send ping packet
    auto packet = PingPacket::build(pingSequence, lastLatency);
    socket->send(packet);
}

void GameHandler::handlePong(network::Packet& packet) {
    LOG_DEBUG("Handling SMSG_PONG");

    PongData data;
    if (!PongParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_PONG");
        return;
    }

    // Verify sequence matches
    if (data.sequence != pingSequence) {
        LOG_WARNING("SMSG_PONG sequence mismatch: expected ", pingSequence,
                    ", got ", data.sequence);
        return;
    }

    LOG_DEBUG("Heartbeat acknowledged (sequence: ", data.sequence, ")");
}

uint32_t GameHandler::nextMovementTimestampMs() {
    auto now = std::chrono::steady_clock::now();
    uint64_t elapsed = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now - movementClockStart_).count()) + 1ULL;
    if (elapsed > std::numeric_limits<uint32_t>::max()) {
        movementClockStart_ = now;
        elapsed = 1ULL;
    }

    uint32_t candidate = static_cast<uint32_t>(elapsed);
    if (candidate <= lastMovementTimestampMs_) {
        candidate = lastMovementTimestampMs_ + 1U;
        if (candidate == 0) {
            movementClockStart_ = now;
            candidate = 1U;
        }
    }

    lastMovementTimestampMs_ = candidate;
    return candidate;
}

void GameHandler::sendMovement(Opcode opcode) {
    if (state != WorldState::IN_WORLD) {
        LOG_WARNING("Cannot send movement in state: ", (int)state);
        return;
    }

    // Block manual movement while taxi is active/mounted, but always allow
    // stop/heartbeat opcodes so stuck states can be recovered.
    bool taxiAllowed =
        (opcode == Opcode::MSG_MOVE_HEARTBEAT) ||
        (opcode == Opcode::MSG_MOVE_STOP) ||
        (opcode == Opcode::MSG_MOVE_STOP_STRAFE) ||
        (opcode == Opcode::MSG_MOVE_STOP_TURN) ||
        (opcode == Opcode::MSG_MOVE_STOP_SWIM);
    if (!serverMovementAllowed_ && !taxiAllowed) return;
    if ((onTaxiFlight_ || taxiMountActive_) && !taxiAllowed) return;
    if (resurrectPending_ && !taxiAllowed) return;

    // Always send a strictly increasing non-zero client movement clock value.
    movementInfo.time = nextMovementTimestampMs();

    // Update movement flags based on opcode
    switch (opcode) {
        case Opcode::MSG_MOVE_START_FORWARD:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::FORWARD);
            break;
        case Opcode::MSG_MOVE_START_BACKWARD:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::BACKWARD);
            break;
        case Opcode::MSG_MOVE_STOP:
            movementInfo.flags &= ~(static_cast<uint32_t>(MovementFlags::FORWARD) |
                                    static_cast<uint32_t>(MovementFlags::BACKWARD));
            break;
        case Opcode::MSG_MOVE_START_STRAFE_LEFT:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::STRAFE_LEFT);
            break;
        case Opcode::MSG_MOVE_START_STRAFE_RIGHT:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::STRAFE_RIGHT);
            break;
        case Opcode::MSG_MOVE_STOP_STRAFE:
            movementInfo.flags &= ~(static_cast<uint32_t>(MovementFlags::STRAFE_LEFT) |
                                    static_cast<uint32_t>(MovementFlags::STRAFE_RIGHT));
            break;
        case Opcode::MSG_MOVE_JUMP:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::FALLING);
            break;
        case Opcode::MSG_MOVE_START_TURN_LEFT:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::TURN_LEFT);
            break;
        case Opcode::MSG_MOVE_START_TURN_RIGHT:
            movementInfo.flags |= static_cast<uint32_t>(MovementFlags::TURN_RIGHT);
            break;
        case Opcode::MSG_MOVE_STOP_TURN:
            movementInfo.flags &= ~(static_cast<uint32_t>(MovementFlags::TURN_LEFT) |
                                    static_cast<uint32_t>(MovementFlags::TURN_RIGHT));
            break;
        case Opcode::MSG_MOVE_FALL_LAND:
            movementInfo.flags &= ~static_cast<uint32_t>(MovementFlags::FALLING);
            break;
        case Opcode::MSG_MOVE_HEARTBEAT:
            // No flag changes — just sends current position
            break;
        default:
            break;
    }

    if (onTaxiFlight_ || taxiMountActive_ || taxiActivatePending_ || taxiClientActive_) {
        sanitizeMovementForTaxi();
    }

    // Add transport data if player is on a transport
    if (isOnTransport()) {
        // Keep authoritative world position synchronized to parent transport transform
        // so heartbeats/corrections don't drag the passenger through geometry.
        if (transportManager_) {
            glm::vec3 composed = transportManager_->getPlayerWorldPosition(playerTransportGuid_, playerTransportOffset_);
            movementInfo.x = composed.x;
            movementInfo.y = composed.y;
            movementInfo.z = composed.z;
        }
        movementInfo.flags |= static_cast<uint32_t>(MovementFlags::ONTRANSPORT);
        movementInfo.transportGuid = playerTransportGuid_;
        movementInfo.transportX = playerTransportOffset_.x;
        movementInfo.transportY = playerTransportOffset_.y;
        movementInfo.transportZ = playerTransportOffset_.z;
        movementInfo.transportTime = movementInfo.time;
        movementInfo.transportSeat = -1;
        movementInfo.transportTime2 = movementInfo.time;

        // ONTRANSPORT expects local orientation (player yaw relative to transport yaw).
        // Keep internal yaw canonical; convert to server yaw on the wire.
        float transportYawCanonical = 0.0f;
        if (transportManager_) {
            if (auto* tr = transportManager_->getTransport(playerTransportGuid_); tr) {
                if (tr->hasServerYaw) {
                    transportYawCanonical = tr->serverYaw;
                } else {
                    transportYawCanonical = glm::eulerAngles(tr->rotation).z;
                }
            }
        }

        movementInfo.transportO =
            core::coords::normalizeAngleRad(movementInfo.orientation - transportYawCanonical);
    } else {
        // Clear transport flag if not on transport
        movementInfo.flags &= ~static_cast<uint32_t>(MovementFlags::ONTRANSPORT);
        movementInfo.transportGuid = 0;
        movementInfo.transportSeat = -1;
    }

    LOG_DEBUG("Sending movement packet: opcode=0x", std::hex,
              wireOpcode(opcode), std::dec,
              (isOnTransport() ? " ONTRANSPORT" : ""));

    // Convert canonical → server coordinates for the wire
    MovementInfo wireInfo = movementInfo;
    glm::vec3 serverPos = core::coords::canonicalToServer(glm::vec3(wireInfo.x, wireInfo.y, wireInfo.z));
    wireInfo.x = serverPos.x;
    wireInfo.y = serverPos.y;
    wireInfo.z = serverPos.z;

    // Convert canonical → server yaw for the wire
    wireInfo.orientation = core::coords::canonicalToServerYaw(wireInfo.orientation);

    // Also convert transport local position to server coordinates if on transport
    if (isOnTransport()) {
        glm::vec3 serverTransportPos = core::coords::canonicalToServer(
            glm::vec3(wireInfo.transportX, wireInfo.transportY, wireInfo.transportZ));
        wireInfo.transportX = serverTransportPos.x;
        wireInfo.transportY = serverTransportPos.y;
        wireInfo.transportZ = serverTransportPos.z;
        // transportO is a local delta; server<->canonical swap negates delta yaw.
        wireInfo.transportO = core::coords::normalizeAngleRad(-wireInfo.transportO);
    }

    // Build and send movement packet (expansion-specific format)
    auto packet = packetParsers_
        ? packetParsers_->buildMovementPacket(opcode, wireInfo, playerGuid)
        : MovementPacket::build(opcode, wireInfo, playerGuid);
    socket->send(packet);
}

void GameHandler::sanitizeMovementForTaxi() {
    constexpr uint32_t kClearTaxiFlags =
        static_cast<uint32_t>(MovementFlags::FORWARD) |
        static_cast<uint32_t>(MovementFlags::BACKWARD) |
        static_cast<uint32_t>(MovementFlags::STRAFE_LEFT) |
        static_cast<uint32_t>(MovementFlags::STRAFE_RIGHT) |
        static_cast<uint32_t>(MovementFlags::TURN_LEFT) |
        static_cast<uint32_t>(MovementFlags::TURN_RIGHT) |
        static_cast<uint32_t>(MovementFlags::PITCH_UP) |
        static_cast<uint32_t>(MovementFlags::PITCH_DOWN) |
        static_cast<uint32_t>(MovementFlags::FALLING) |
        static_cast<uint32_t>(MovementFlags::FALLINGFAR) |
        static_cast<uint32_t>(MovementFlags::SWIMMING);

    movementInfo.flags &= ~kClearTaxiFlags;
    movementInfo.fallTime = 0;
    movementInfo.jumpVelocity = 0.0f;
    movementInfo.jumpSinAngle = 0.0f;
    movementInfo.jumpCosAngle = 0.0f;
    movementInfo.jumpXYSpeed = 0.0f;
    movementInfo.pitch = 0.0f;
}

void GameHandler::forceClearTaxiAndMovementState() {
    taxiActivatePending_ = false;
    taxiActivateTimer_ = 0.0f;
    taxiClientActive_ = false;
    taxiClientPath_.clear();
    taxiRecoverPending_ = false;
    taxiStartGrace_ = 0.0f;
    onTaxiFlight_ = false;

    if (taxiMountActive_ && mountCallback_) {
        mountCallback_(0);
    }
    taxiMountActive_ = false;
    taxiMountDisplayId_ = 0;
    currentMountDisplayId_ = 0;
    resurrectPending_ = false;
    resurrectRequestPending_ = false;
    playerDead_ = false;
    releasedSpirit_ = false;
    repopPending_ = false;
    pendingSpiritHealerGuid_ = 0;
    resurrectCasterGuid_ = 0;

    movementInfo.flags = 0;
    movementInfo.flags2 = 0;
    movementInfo.transportGuid = 0;
    clearPlayerTransport();

    if (socket && state == WorldState::IN_WORLD) {
        sendMovement(Opcode::MSG_MOVE_STOP);
        sendMovement(Opcode::MSG_MOVE_STOP_STRAFE);
        sendMovement(Opcode::MSG_MOVE_STOP_TURN);
        sendMovement(Opcode::MSG_MOVE_STOP_SWIM);
        sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
    }

    LOG_INFO("Force-cleared taxi/movement state");
}

void GameHandler::setPosition(float x, float y, float z) {
    movementInfo.x = x;
    movementInfo.y = y;
    movementInfo.z = z;
}

void GameHandler::setOrientation(float orientation) {
    movementInfo.orientation = orientation;
}

void GameHandler::handleUpdateObject(network::Packet& packet) {
    UpdateObjectData data;
    if (!packetParsers_->parseUpdateObject(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_UPDATE_OBJECT");
        return;
    }

    auto extractPlayerAppearance = [&](const std::map<uint16_t, uint32_t>& fields,
                                       uint8_t& outRace,
                                       uint8_t& outGender,
                                       uint32_t& outAppearanceBytes,
                                       uint8_t& outFacial) -> bool {
        outRace = 0;
        outGender = 0;
        outAppearanceBytes = 0;
        outFacial = 0;

        auto readField = [&](uint16_t idx, uint32_t& out) -> bool {
            if (idx == 0xFFFF) return false;
            auto it = fields.find(idx);
            if (it == fields.end()) return false;
            out = it->second;
            return true;
        };

        uint32_t bytes0 = 0;
        uint32_t pbytes = 0;
        uint32_t pbytes2 = 0;

        const uint16_t ufBytes0 = fieldIndex(UF::UNIT_FIELD_BYTES_0);
        const uint16_t ufPbytes = fieldIndex(UF::PLAYER_BYTES);
        const uint16_t ufPbytes2 = fieldIndex(UF::PLAYER_BYTES_2);

        bool haveBytes0 = readField(ufBytes0, bytes0);
        bool havePbytes = readField(ufPbytes, pbytes);
        bool havePbytes2 = readField(ufPbytes2, pbytes2);

        // Heuristic fallback: Turtle can run with unusual build numbers; if the JSON table is missing,
        // try to locate plausible packed fields by scanning.
        if (!haveBytes0) {
            for (const auto& [idx, v] : fields) {
                uint8_t race = static_cast<uint8_t>(v & 0xFF);
                uint8_t cls = static_cast<uint8_t>((v >> 8) & 0xFF);
                uint8_t gender = static_cast<uint8_t>((v >> 16) & 0xFF);
                uint8_t power = static_cast<uint8_t>((v >> 24) & 0xFF);
                if (race >= 1 && race <= 20 &&
                    cls >= 1 && cls <= 20 &&
                    gender <= 1 &&
                    power <= 10) {
                    bytes0 = v;
                    haveBytes0 = true;
                    break;
                }
            }
        }
        if (!havePbytes) {
            for (const auto& [idx, v] : fields) {
                uint8_t skin = static_cast<uint8_t>(v & 0xFF);
                uint8_t face = static_cast<uint8_t>((v >> 8) & 0xFF);
                uint8_t hair = static_cast<uint8_t>((v >> 16) & 0xFF);
                uint8_t color = static_cast<uint8_t>((v >> 24) & 0xFF);
                if (skin <= 50 && face <= 50 && hair <= 100 && color <= 50) {
                    pbytes = v;
                    havePbytes = true;
                    break;
                }
            }
        }
        if (!havePbytes2) {
            for (const auto& [idx, v] : fields) {
                uint8_t facial = static_cast<uint8_t>(v & 0xFF);
                if (facial <= 100) {
                    pbytes2 = v;
                    havePbytes2 = true;
                    break;
                }
            }
        }

        if (!haveBytes0 || !havePbytes) return false;

        outRace = static_cast<uint8_t>(bytes0 & 0xFF);
        outGender = static_cast<uint8_t>((bytes0 >> 16) & 0xFF);
        outAppearanceBytes = pbytes;
        outFacial = havePbytes2 ? static_cast<uint8_t>(pbytes2 & 0xFF) : 0;
        return true;
    };

    auto maybeDetectCoinageIndex = [&](const std::map<uint16_t, uint32_t>& oldFields,
                                       const std::map<uint16_t, uint32_t>& newFields) {
        if (pendingMoneyDelta_ == 0 || pendingMoneyDeltaTimer_ <= 0.0f) return;
        if (oldFields.empty() || newFields.empty()) return;

        constexpr uint32_t kMaxPlausibleCoinage = 2147483647u;
        std::vector<uint16_t> candidates;
        candidates.reserve(8);

        for (const auto& [idx, newVal] : newFields) {
            auto itOld = oldFields.find(idx);
            if (itOld == oldFields.end()) continue;
            uint32_t oldVal = itOld->second;
            if (newVal < oldVal) continue;
            uint32_t delta = newVal - oldVal;
            if (delta != pendingMoneyDelta_) continue;
            if (newVal > kMaxPlausibleCoinage) continue;
            candidates.push_back(idx);
        }

        if (candidates.empty()) return;

        uint16_t current = fieldIndex(UF::PLAYER_FIELD_COINAGE);
        uint16_t chosen = candidates[0];
        if (std::find(candidates.begin(), candidates.end(), current) != candidates.end()) {
            chosen = current;
        } else {
            std::sort(candidates.begin(), candidates.end());
            chosen = candidates[0];
        }

        if (chosen != current && current != 0xFFFF) {
            updateFieldTable_.setIndex(UF::PLAYER_FIELD_COINAGE, chosen);
            LOG_WARNING("Auto-detected PLAYER_FIELD_COINAGE index: ", chosen, " (was ", current, ")");
        }

        pendingMoneyDelta_ = 0;
        pendingMoneyDeltaTimer_ = 0.0f;
    };

    // Process out-of-range objects first
    for (uint64_t guid : data.outOfRangeGuids) {
        if (entityManager.hasEntity(guid)) {
            const bool isKnownTransport = transportGuids_.count(guid) > 0;
            if (isKnownTransport) {
                // Keep transports alive across out-of-range flapping.
                // Boats/zeppelins are global movers and removing them here can make
                // them disappear until a later movement snapshot happens to recreate them.
                const bool playerAboardNow = (playerTransportGuid_ == guid);
                const bool stickyAboard = (playerTransportStickyGuid_ == guid && playerTransportStickyTimer_ > 0.0f);
                const bool movementSaysAboard = (movementInfo.transportGuid == guid);
                LOG_INFO("Preserving transport on out-of-range: 0x",
                         std::hex, guid, std::dec,
                         " now=", playerAboardNow,
                         " sticky=", stickyAboard,
                         " movement=", movementSaysAboard);
                continue;
            }

            LOG_DEBUG("Entity went out of range: 0x", std::hex, guid, std::dec);
            // Trigger despawn callbacks before removing entity
            auto entity = entityManager.getEntity(guid);
            if (entity) {
                if (entity->getType() == ObjectType::UNIT && creatureDespawnCallback_) {
                    creatureDespawnCallback_(guid);
                } else if (entity->getType() == ObjectType::PLAYER && playerDespawnCallback_) {
                    playerDespawnCallback_(guid);
                    otherPlayerVisibleItemEntries_.erase(guid);
                    otherPlayerVisibleDirty_.erase(guid);
                    otherPlayerMoveTimeMs_.erase(guid);
                    inspectedPlayerItemEntries_.erase(guid);
                    pendingAutoInspect_.erase(guid);
                } else if (entity->getType() == ObjectType::GAMEOBJECT && gameObjectDespawnCallback_) {
                    gameObjectDespawnCallback_(guid);
                }
            }
            transportGuids_.erase(guid);
            serverUpdatedTransportGuids_.erase(guid);
            clearTransportAttachment(guid);
            if (playerTransportGuid_ == guid) {
                clearPlayerTransport();
            }
            entityManager.removeEntity(guid);
        }
    }

    // Process update blocks
    for (const auto& block : data.blocks) {
        switch (block.updateType) {
            case UpdateType::CREATE_OBJECT:
            case UpdateType::CREATE_OBJECT2: {
                // Create new entity
                std::shared_ptr<Entity> entity;

                switch (block.objectType) {
                    case ObjectType::PLAYER:
                        entity = std::make_shared<Player>(block.guid);
                        break;

                    case ObjectType::UNIT:
                        entity = std::make_shared<Unit>(block.guid);
                        break;

                    case ObjectType::GAMEOBJECT:
                        entity = std::make_shared<GameObject>(block.guid);
                        break;

                    default:
                        entity = std::make_shared<Entity>(block.guid);
                        entity->setType(block.objectType);
                        break;
                }

                // Set position from movement block (server → canonical)
                if (block.hasMovement) {
                    glm::vec3 pos = core::coords::serverToCanonical(glm::vec3(block.x, block.y, block.z));
                    float oCanonical = core::coords::serverToCanonicalYaw(block.orientation);
                    entity->setPosition(pos.x, pos.y, pos.z, oCanonical);
                    LOG_DEBUG("  Position: (", pos.x, ", ", pos.y, ", ", pos.z, ")");
                    if (block.guid == playerGuid && block.runSpeed > 0.1f && block.runSpeed < 100.0f) {
                        serverRunSpeed_ = block.runSpeed;
                    }
                    // Track player-on-transport state
                    if (block.guid == playerGuid) {
                        if (block.onTransport) {
                            setPlayerOnTransport(block.transportGuid, glm::vec3(0.0f));
                            // Convert transport offset from server → canonical coordinates
                            glm::vec3 serverOffset(block.transportX, block.transportY, block.transportZ);
                            playerTransportOffset_ = core::coords::serverToCanonical(serverOffset);
                            if (transportManager_ && transportManager_->getTransport(playerTransportGuid_)) {
                                glm::vec3 composed = transportManager_->getPlayerWorldPosition(playerTransportGuid_, playerTransportOffset_);
                                entity->setPosition(composed.x, composed.y, composed.z, oCanonical);
                                movementInfo.x = composed.x;
                                movementInfo.y = composed.y;
                                movementInfo.z = composed.z;
                            }
                            LOG_INFO("Player on transport: 0x", std::hex, playerTransportGuid_, std::dec,
                                    " offset=(", playerTransportOffset_.x, ", ", playerTransportOffset_.y, ", ", playerTransportOffset_.z, ")");
                        } else {
                            if (playerTransportGuid_ != 0) {
                                LOG_INFO("Player left transport");
                            }
                            clearPlayerTransport();
                        }
                    }

                    // Track transport-relative children so they follow parent transport motion.
                    if (block.guid != playerGuid &&
                        (block.objectType == ObjectType::UNIT || block.objectType == ObjectType::GAMEOBJECT)) {
                        if (block.onTransport && block.transportGuid != 0) {
                            glm::vec3 localOffset = core::coords::serverToCanonical(
                                glm::vec3(block.transportX, block.transportY, block.transportZ));
                            const bool hasLocalOrientation = (block.updateFlags & 0x0020) != 0; // UPDATEFLAG_LIVING
                            float localOriCanonical = core::coords::normalizeAngleRad(-block.transportO);
                            setTransportAttachment(block.guid, block.objectType, block.transportGuid,
                                                   localOffset, hasLocalOrientation, localOriCanonical);
                            if (transportManager_ && transportManager_->getTransport(block.transportGuid)) {
                                glm::vec3 composed = transportManager_->getPlayerWorldPosition(block.transportGuid, localOffset);
                                entity->setPosition(composed.x, composed.y, composed.z, entity->getOrientation());
                            }
                        } else {
                            clearTransportAttachment(block.guid);
                        }
                    }
                }

                // Set fields
                for (const auto& field : block.fields) {
                    entity->setField(field.first, field.second);
                }

                // Add to manager
                entityManager.addEntity(block.guid, entity);

                // For the local player, capture the full initial field state (CREATE_OBJECT carries the
                // large baseline update-field set, including visible item fields on many cores).
                // Later VALUES updates often only include deltas and may never touch visible item fields.
                if (block.guid == playerGuid && block.objectType == ObjectType::PLAYER) {
                    lastPlayerFields_ = entity->getFields();
                    maybeDetectVisibleItemLayout();
                }

                // Auto-query names (Phase 1)
                if (block.objectType == ObjectType::PLAYER) {
                    queryPlayerName(block.guid);
                    if (block.guid != playerGuid) {
                        updateOtherPlayerVisibleItems(block.guid, entity->getFields());
                    }
                } else if (block.objectType == ObjectType::UNIT) {
                    auto it = block.fields.find(fieldIndex(UF::OBJECT_FIELD_ENTRY));
                    if (it != block.fields.end() && it->second != 0) {
                        auto unit = std::static_pointer_cast<Unit>(entity);
                        unit->setEntry(it->second);
                        // Set name from cache immediately if available
                        std::string cached = getCachedCreatureName(it->second);
                        if (!cached.empty()) {
                            unit->setName(cached);
                        }
                        queryCreatureInfo(it->second, block.guid);
                    }
                }

                // Extract health/mana/power from fields (Phase 2) — single pass
                if (block.objectType == ObjectType::UNIT || block.objectType == ObjectType::PLAYER) {
                    auto unit = std::static_pointer_cast<Unit>(entity);
                    constexpr uint32_t UNIT_DYNFLAG_DEAD = 0x0008;
                    constexpr uint32_t UNIT_DYNFLAG_LOOTABLE = 0x0001;
                    bool unitInitiallyDead = false;
                    const uint16_t ufHealth = fieldIndex(UF::UNIT_FIELD_HEALTH);
                    const uint16_t ufPowerBase = fieldIndex(UF::UNIT_FIELD_POWER1);
                    const uint16_t ufMaxHealth = fieldIndex(UF::UNIT_FIELD_MAXHEALTH);
                    const uint16_t ufMaxPowerBase = fieldIndex(UF::UNIT_FIELD_MAXPOWER1);
                    const uint16_t ufLevel = fieldIndex(UF::UNIT_FIELD_LEVEL);
                    const uint16_t ufFaction = fieldIndex(UF::UNIT_FIELD_FACTIONTEMPLATE);
                    const uint16_t ufFlags = fieldIndex(UF::UNIT_FIELD_FLAGS);
                    const uint16_t ufDynFlags = fieldIndex(UF::UNIT_DYNAMIC_FLAGS);
                    const uint16_t ufDisplayId = fieldIndex(UF::UNIT_FIELD_DISPLAYID);
                    const uint16_t ufMountDisplayId = fieldIndex(UF::UNIT_FIELD_MOUNTDISPLAYID);
                    const uint16_t ufNpcFlags = fieldIndex(UF::UNIT_NPC_FLAGS);
                    const uint16_t ufBytes0 = fieldIndex(UF::UNIT_FIELD_BYTES_0);
                    for (const auto& [key, val] : block.fields) {
                        // Check all specific fields BEFORE power/maxpower range checks.
                        // In Classic, power indices (23-27) are adjacent to maxHealth (28),
                        // and maxPower indices (29-33) are adjacent to level (34) and faction (35).
                        // A range check like "key >= powerBase && key < powerBase+7" would
                        // incorrectly capture maxHealth/level/faction in Classic's tight layout.
                        if (key == ufHealth) {
                            unit->setHealth(val);
                            if (block.objectType == ObjectType::UNIT && val == 0) {
                                unitInitiallyDead = true;
                            }
                            if (block.guid == playerGuid && val == 0) {
                                playerDead_ = true;
                                LOG_INFO("Player logged in dead");
                            }
                        } else if (key == ufMaxHealth) { unit->setMaxHealth(val); }
                        else if (key == ufLevel) {
                            unit->setLevel(val);
                        } else if (key == ufFaction) { unit->setFactionTemplate(val); }
                        else if (key == ufFlags) { unit->setUnitFlags(val); }
                        else if (key == ufBytes0) {
                            unit->setPowerType(static_cast<uint8_t>((val >> 24) & 0xFF));
                        } else if (key == ufDisplayId) { unit->setDisplayId(val); }
                        else if (key == ufNpcFlags) { unit->setNpcFlags(val); }
                        else if (key == ufDynFlags) {
                            unit->setDynamicFlags(val);
                            if (block.objectType == ObjectType::UNIT &&
                                ((val & UNIT_DYNFLAG_DEAD) != 0 || (val & UNIT_DYNFLAG_LOOTABLE) != 0)) {
                                unitInitiallyDead = true;
                            }
                        }
                        // Power/maxpower range checks AFTER all specific fields
                        else if (key >= ufPowerBase && key < ufPowerBase + 7) {
                            unit->setPowerByType(static_cast<uint8_t>(key - ufPowerBase), val);
                        } else if (key >= ufMaxPowerBase && key < ufMaxPowerBase + 7) {
                            unit->setMaxPowerByType(static_cast<uint8_t>(key - ufMaxPowerBase), val);
                        }
                        else if (key == ufMountDisplayId) {
                            if (block.guid == playerGuid) {
                                uint32_t old = currentMountDisplayId_;
                                currentMountDisplayId_ = val;
                                if (val != old && mountCallback_) mountCallback_(val);
                                if (old == 0 && val != 0) {
                                    // Just mounted — find the mount aura (indefinite duration, self-cast)
                                    mountAuraSpellId_ = 0;
                                    for (const auto& a : playerAuras) {
                                        if (!a.isEmpty() && a.maxDurationMs < 0 && a.casterGuid == playerGuid) {
                                            mountAuraSpellId_ = a.spellId;
                                        }
                                    }
                                    // Classic/vanilla fallback: scan UNIT_FIELD_AURAS from same update block
                                    if (mountAuraSpellId_ == 0) {
                                        const uint16_t ufAuras = fieldIndex(UF::UNIT_FIELD_AURAS);
                                        if (ufAuras != 0xFFFF) {
                                            for (const auto& [fk, fv] : block.fields) {
                                                if (fk >= ufAuras && fk < ufAuras + 48 && fv != 0) {
                                                    mountAuraSpellId_ = fv;
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                    LOG_INFO("Mount detected: displayId=", val, " auraSpellId=", mountAuraSpellId_);
                                }
                                if (old != 0 && val == 0) {
                                    mountAuraSpellId_ = 0;
                                    for (auto& a : playerAuras)
                                        if (!a.isEmpty() && a.maxDurationMs < 0) a = AuraSlot{};
                                }
                            }
                            unit->setMountDisplayId(val);
                        } else if (key == ufNpcFlags) { unit->setNpcFlags(val); }
                    }
                    if (block.guid == playerGuid) {
                        constexpr uint32_t UNIT_FLAG_TAXI_FLIGHT = 0x00000100;
                        if ((unit->getUnitFlags() & UNIT_FLAG_TAXI_FLIGHT) != 0 && !onTaxiFlight_ && taxiLandingCooldown_ <= 0.0f) {
                            onTaxiFlight_ = true;
                            taxiStartGrace_ = std::max(taxiStartGrace_, 2.0f);
                            sanitizeMovementForTaxi();
                            applyTaxiMountForCurrentNode();
                        }
                    }
                    if (block.guid == playerGuid &&
                        (unit->getDynamicFlags() & UNIT_DYNFLAG_DEAD) != 0) {
                        playerDead_ = true;
                        LOG_INFO("Player logged in dead (dynamic flags)");
                    }
                    // Detect ghost state on login via PLAYER_FLAGS
                    if (block.guid == playerGuid) {
                        constexpr uint32_t PLAYER_FLAGS_GHOST = 0x00000010;
                        auto pfIt = block.fields.find(fieldIndex(UF::PLAYER_FLAGS));
                        if (pfIt != block.fields.end() && (pfIt->second & PLAYER_FLAGS_GHOST) != 0) {
                            releasedSpirit_ = true;
                            playerDead_ = true;
                            LOG_INFO("Player logged in as ghost (PLAYER_FLAGS)");
                        }
                    }
                    // Determine hostility from faction template for online creatures
                    if (unit->getFactionTemplate() != 0) {
                        unit->setHostile(isHostileFaction(unit->getFactionTemplate()));
                    }
                // Trigger creature spawn callback for units/players with displayId
                    if (block.objectType == ObjectType::UNIT && unit->getDisplayId() == 0) {
                        LOG_DEBUG("[Spawn] UNIT guid=0x", std::hex, block.guid, std::dec,
                                  " has displayId=0 — no spawn (entry=", unit->getEntry(), ")");
                    }
                    if ((block.objectType == ObjectType::UNIT || block.objectType == ObjectType::PLAYER) && unit->getDisplayId() != 0) {
                        if (block.objectType == ObjectType::PLAYER && block.guid == playerGuid) {
                            // Skip local player — spawned separately via spawnPlayerCharacter()
                        } else if (block.objectType == ObjectType::PLAYER) {
                            if (playerSpawnCallback_) {
                                uint8_t race = 0, gender = 0, facial = 0;
                                uint32_t appearanceBytes = 0;
                                // Use the entity's accumulated field state, not just this block's changed fields.
                                if (extractPlayerAppearance(entity->getFields(), race, gender, appearanceBytes, facial)) {
                                    playerSpawnCallback_(block.guid, unit->getDisplayId(), race, gender,
                                                        appearanceBytes, facial,
                                                        unit->getX(), unit->getY(), unit->getZ(), unit->getOrientation());
                                }
                            }
                        } else if (creatureSpawnCallback_) {
                            LOG_DEBUG("[Spawn] UNIT guid=0x", std::hex, block.guid, std::dec,
                                      " displayId=", unit->getDisplayId(), " at (",
                                      unit->getX(), ",", unit->getY(), ",", unit->getZ(), ")");
                            creatureSpawnCallback_(block.guid, unit->getDisplayId(),
                                unit->getX(), unit->getY(), unit->getZ(), unit->getOrientation());
                            if (unitInitiallyDead && npcDeathCallback_) {
                                npcDeathCallback_(block.guid);
                            }
                        }
                        // Query quest giver status for NPCs with questgiver flag (0x02)
                        if (block.objectType == ObjectType::UNIT && (unit->getNpcFlags() & 0x02) && socket) {
                            network::Packet qsPkt(wireOpcode(Opcode::CMSG_QUESTGIVER_STATUS_QUERY));
                            qsPkt.writeUInt64(block.guid);
                            socket->send(qsPkt);
                        }
                    }
                }
                // Extract displayId and entry for gameobjects (3.3.5a: GAMEOBJECT_DISPLAYID = field 8)
                if (block.objectType == ObjectType::GAMEOBJECT) {
                    auto go = std::static_pointer_cast<GameObject>(entity);
                    auto itDisp = block.fields.find(fieldIndex(UF::GAMEOBJECT_DISPLAYID));
                    if (itDisp != block.fields.end()) {
                        go->setDisplayId(itDisp->second);
                    }
                    auto itEntry = block.fields.find(fieldIndex(UF::OBJECT_FIELD_ENTRY));
                    if (itEntry != block.fields.end() && itEntry->second != 0) {
                        go->setEntry(itEntry->second);
                        auto cacheIt = gameObjectInfoCache_.find(itEntry->second);
                        if (cacheIt != gameObjectInfoCache_.end()) {
                            go->setName(cacheIt->second.name);
                        }
                        queryGameObjectInfo(itEntry->second, block.guid);
                    }
                    // Detect transport GameObjects via UPDATEFLAG_TRANSPORT (0x0002)
                    if (block.updateFlags & 0x0002) {
                        transportGuids_.insert(block.guid);
                        LOG_INFO("Detected transport GameObject: 0x", std::hex, block.guid, std::dec,
                                 " entry=", go->getEntry(),
                                 " displayId=", go->getDisplayId(),
                                 " pos=(", go->getX(), ", ", go->getY(), ", ", go->getZ(), ")");
                        // Note: TransportSpawnCallback will be invoked from Application after WMO instance is created
                    }
                    if (go->getDisplayId() != 0 && gameObjectSpawnCallback_) {
                        gameObjectSpawnCallback_(block.guid, go->getEntry(), go->getDisplayId(),
                            go->getX(), go->getY(), go->getZ(), go->getOrientation());
                    }
                    // Fire transport move callback for transports (position update on re-creation)
                    if (transportGuids_.count(block.guid) && transportMoveCallback_) {
                        serverUpdatedTransportGuids_.insert(block.guid);
                        transportMoveCallback_(block.guid,
                            go->getX(), go->getY(), go->getZ(), go->getOrientation());
                    }
                }
                // Track online item objects (CONTAINER = bags, also tracked as items)
                if (block.objectType == ObjectType::ITEM || block.objectType == ObjectType::CONTAINER) {
                    auto entryIt = block.fields.find(fieldIndex(UF::OBJECT_FIELD_ENTRY));
                    auto stackIt = block.fields.find(fieldIndex(UF::ITEM_FIELD_STACK_COUNT));
                    if (entryIt != block.fields.end() && entryIt->second != 0) {
                        OnlineItemInfo info;
                        info.entry = entryIt->second;
                        info.stackCount = (stackIt != block.fields.end()) ? stackIt->second : 1;
                        onlineItems_[block.guid] = info;
                        queryItemInfo(info.entry, block.guid);
                    }
                    // Extract container slot GUIDs for bags
                    if (block.objectType == ObjectType::CONTAINER) {
                        extractContainerFields(block.guid, block.fields);
                    }
                }

                // Extract XP / inventory slot / skill fields for player entity
                if (block.guid == playerGuid && block.objectType == ObjectType::PLAYER) {
                    // Store baseline snapshot on first update
                    static bool baselineStored = false;
                    static std::map<uint16_t, uint32_t> baselineFields;

                    if (!baselineStored) {
                        baselineFields = block.fields;
                        baselineStored = true;
                        LOG_INFO("===== BASELINE PLAYER FIELDS STORED =====");
                        LOG_INFO("  Total fields: ", block.fields.size());
                    }

                    // Diff against baseline to find changes
                    std::vector<uint16_t> changedIndices;
                    std::vector<uint16_t> newIndices;
                    std::vector<uint16_t> removedIndices;

                    for (const auto& [idx, val] : block.fields) {
                        auto it = baselineFields.find(idx);
                        if (it == baselineFields.end()) {
                            newIndices.push_back(idx);
                        } else if (it->second != val) {
                            changedIndices.push_back(idx);
                        }
                    }

                    for (const auto& [idx, val] : baselineFields) {
                        if (block.fields.find(idx) == block.fields.end()) {
                            removedIndices.push_back(idx);
                        }
                    }

                    // Auto-detect coinage index using the previous snapshot vs this full snapshot.
                    maybeDetectCoinageIndex(lastPlayerFields_, block.fields);

                    lastPlayerFields_ = block.fields;
                    detectInventorySlotBases(block.fields);

                    // Debug: Show field changes
                    LOG_INFO("Player update with ", block.fields.size(), " fields");

                    if (!changedIndices.empty() || !newIndices.empty() || !removedIndices.empty()) {
                        LOG_INFO("  ===== FIELD CHANGES DETECTED =====");
                        if (!changedIndices.empty()) {
                            LOG_INFO("  Changed fields (", changedIndices.size(), "):");
                            std::sort(changedIndices.begin(), changedIndices.end());
                            for (size_t i = 0; i < std::min(size_t(30), changedIndices.size()); ++i) {
                                uint16_t idx = changedIndices[i];
                                uint32_t oldVal = baselineFields[idx];
                                uint32_t newVal = block.fields.at(idx);
                                LOG_INFO("    [", idx, "]: ", oldVal, " -> ", newVal,
                                         " (0x", std::hex, oldVal, " -> 0x", newVal, std::dec, ")");
                            }
                            if (changedIndices.size() > 30) {
                                LOG_INFO("    ... (", changedIndices.size() - 30, " more)");
                            }
                        }
                        if (!newIndices.empty()) {
                            LOG_INFO("  New fields (", newIndices.size(), "):");
                            std::sort(newIndices.begin(), newIndices.end());
                            for (size_t i = 0; i < std::min(size_t(20), newIndices.size()); ++i) {
                                uint16_t idx = newIndices[i];
                                uint32_t val = block.fields.at(idx);
                                LOG_INFO("    [", idx, "]: ", val, " (0x", std::hex, val, std::dec, ")");
                            }
                            if (newIndices.size() > 20) {
                                LOG_INFO("    ... (", newIndices.size() - 20, " more)");
                            }
                        }
                        if (!removedIndices.empty()) {
                            LOG_INFO("  Removed fields (", removedIndices.size(), "):");
                            std::sort(removedIndices.begin(), removedIndices.end());
                            for (size_t i = 0; i < std::min(size_t(20), removedIndices.size()); ++i) {
                                uint16_t idx = removedIndices[i];
                                uint32_t val = baselineFields.at(idx);
                                LOG_INFO("    [", idx, "]: was ", val, " (0x", std::hex, val, std::dec, ")");
                            }
                        }
                    }

                    uint16_t maxField = 0;
                    for (const auto& [key, val] : block.fields) {
                        if (key > maxField) maxField = key;
                    }

                    LOG_INFO("  Highest field index: ", maxField);

                    bool slotsChanged = false;
                    const uint16_t ufPlayerXp = fieldIndex(UF::PLAYER_XP);
                    const uint16_t ufPlayerNextXp = fieldIndex(UF::PLAYER_NEXT_LEVEL_XP);
                    const uint16_t ufPlayerLevel = fieldIndex(UF::UNIT_FIELD_LEVEL);
                    const uint16_t ufCoinage = fieldIndex(UF::PLAYER_FIELD_COINAGE);
                    const uint16_t ufArmor = fieldIndex(UF::UNIT_FIELD_RESISTANCES);
                    for (const auto& [key, val] : block.fields) {
                        if (key == ufPlayerXp) { playerXp_ = val; }
                        else if (key == ufPlayerNextXp) { playerNextLevelXp_ = val; }
                        else if (key == ufPlayerLevel) {
                            serverPlayerLevel_ = val;
                            for (auto& ch : characters) {
                                if (ch.guid == playerGuid) { ch.level = val; break; }
                            }
                        }
                        else if (key == ufCoinage) {
                            playerMoneyCopper_ = val;
                            LOG_INFO("Money set from update fields: ", val, " copper");
                        }
                        else if (ufArmor != 0xFFFF && key == ufArmor) {
                            playerArmorRating_ = static_cast<int32_t>(val);
                            LOG_INFO("Armor rating from update fields: ", playerArmorRating_);
                        }
                        // Do not synthesize quest-log entries from raw update-field slots.
                        // Slot layouts differ on some classic-family realms and can produce
                        // phantom "already accepted" quests that block quest acceptance.
                    }
                    if (applyInventoryFields(block.fields)) slotsChanged = true;
                    if (slotsChanged) rebuildOnlineInventory();
                    maybeDetectVisibleItemLayout();
                    extractSkillFields(lastPlayerFields_);
                    extractExploredZoneFields(lastPlayerFields_);
                }
                break;
            }

            case UpdateType::VALUES: {
                // Update existing entity fields
                auto entity = entityManager.getEntity(block.guid);
                if (entity) {
                    if (block.hasMovement) {
                        glm::vec3 pos = core::coords::serverToCanonical(glm::vec3(block.x, block.y, block.z));
                        float oCanonical = core::coords::serverToCanonicalYaw(block.orientation);
                        entity->setPosition(pos.x, pos.y, pos.z, oCanonical);

                        if (block.guid != playerGuid &&
                            (entity->getType() == ObjectType::UNIT || entity->getType() == ObjectType::GAMEOBJECT)) {
                            if (block.onTransport && block.transportGuid != 0) {
                                glm::vec3 localOffset = core::coords::serverToCanonical(
                                    glm::vec3(block.transportX, block.transportY, block.transportZ));
                                const bool hasLocalOrientation = (block.updateFlags & 0x0020) != 0; // UPDATEFLAG_LIVING
                                float localOriCanonical = core::coords::normalizeAngleRad(-block.transportO);
                                setTransportAttachment(block.guid, entity->getType(), block.transportGuid,
                                                       localOffset, hasLocalOrientation, localOriCanonical);
                                if (transportManager_ && transportManager_->getTransport(block.transportGuid)) {
                                    glm::vec3 composed = transportManager_->getPlayerWorldPosition(block.transportGuid, localOffset);
                                    entity->setPosition(composed.x, composed.y, composed.z, entity->getOrientation());
                                }
                            } else {
                                clearTransportAttachment(block.guid);
                            }
                        }
                    }

                    for (const auto& field : block.fields) {
                        entity->setField(field.first, field.second);
                    }

                    if (entity->getType() == ObjectType::PLAYER && block.guid != playerGuid) {
                        updateOtherPlayerVisibleItems(block.guid, entity->getFields());
                    }

                    // Update cached health/mana/power values (Phase 2) — single pass
                    if (entity->getType() == ObjectType::UNIT || entity->getType() == ObjectType::PLAYER) {
                        auto unit = std::static_pointer_cast<Unit>(entity);
                        constexpr uint32_t UNIT_DYNFLAG_DEAD = 0x0008;
                        constexpr uint32_t UNIT_DYNFLAG_LOOTABLE = 0x0001;
                        uint32_t oldDisplayId = unit->getDisplayId();
                        bool displayIdChanged = false;
                        bool npcDeathNotified = false;
                        bool npcRespawnNotified = false;
                        const uint16_t ufHealth = fieldIndex(UF::UNIT_FIELD_HEALTH);
                        const uint16_t ufPowerBase = fieldIndex(UF::UNIT_FIELD_POWER1);
                        const uint16_t ufMaxHealth = fieldIndex(UF::UNIT_FIELD_MAXHEALTH);
                        const uint16_t ufMaxPowerBase = fieldIndex(UF::UNIT_FIELD_MAXPOWER1);
                        const uint16_t ufLevel = fieldIndex(UF::UNIT_FIELD_LEVEL);
                        const uint16_t ufFaction = fieldIndex(UF::UNIT_FIELD_FACTIONTEMPLATE);
                        const uint16_t ufFlags = fieldIndex(UF::UNIT_FIELD_FLAGS);
                        const uint16_t ufDynFlags = fieldIndex(UF::UNIT_DYNAMIC_FLAGS);
                        const uint16_t ufDisplayId = fieldIndex(UF::UNIT_FIELD_DISPLAYID);
                        const uint16_t ufMountDisplayId = fieldIndex(UF::UNIT_FIELD_MOUNTDISPLAYID);
                        const uint16_t ufNpcFlags = fieldIndex(UF::UNIT_NPC_FLAGS);
                        const uint16_t ufBytes0 = fieldIndex(UF::UNIT_FIELD_BYTES_0);
                        for (const auto& [key, val] : block.fields) {
                            if (key == ufHealth) {
                                uint32_t oldHealth = unit->getHealth();
                                unit->setHealth(val);
                                if (val == 0) {
                                    if (block.guid == autoAttackTarget) {
                                        stopAutoAttack();
                                    }
                                    hostileAttackers_.erase(block.guid);
                                    if (block.guid == playerGuid) {
                                        playerDead_ = true;
                                        releasedSpirit_ = false;
                                        stopAutoAttack();
                                        LOG_INFO("Player died!");
                                    }
                                    if (entity->getType() == ObjectType::UNIT && npcDeathCallback_) {
                                        npcDeathCallback_(block.guid);
                                        npcDeathNotified = true;
                                    }
                                } else if (oldHealth == 0 && val > 0) {
                                    if (block.guid == playerGuid) {
                                        playerDead_ = false;
                                        if (!releasedSpirit_) {
                                            LOG_INFO("Player resurrected!");
                                        } else {
                                            LOG_INFO("Player entered ghost form");
                                        }
                                    }
                                    if (entity->getType() == ObjectType::UNIT && npcRespawnCallback_) {
                                        npcRespawnCallback_(block.guid);
                                        npcRespawnNotified = true;
                                    }
                                }
                            // Specific fields checked BEFORE power/maxpower range checks
                            // (Classic packs maxHealth/level/faction adjacent to power indices)
                            } else if (key == ufMaxHealth) { unit->setMaxHealth(val); }
                            else if (key == ufBytes0) {
                                unit->setPowerType(static_cast<uint8_t>((val >> 24) & 0xFF));
                            } else if (key == ufFlags) { unit->setUnitFlags(val); }
                            else if (key == ufDynFlags) {
                                uint32_t oldDyn = unit->getDynamicFlags();
                                unit->setDynamicFlags(val);
                                if (block.guid == playerGuid) {
                                    bool wasDead = (oldDyn & UNIT_DYNFLAG_DEAD) != 0;
                                    bool nowDead = (val & UNIT_DYNFLAG_DEAD) != 0;
                                    if (!wasDead && nowDead) {
                                        playerDead_ = true;
                                        releasedSpirit_ = false;
                                        LOG_INFO("Player died (dynamic flags)");
                                    } else if (wasDead && !nowDead) {
                                        playerDead_ = false;
                                        releasedSpirit_ = false;
                                        LOG_INFO("Player resurrected (dynamic flags)");
                                    }
                                } else if (entity->getType() == ObjectType::UNIT) {
                                    bool wasDead = (oldDyn & UNIT_DYNFLAG_DEAD) != 0;
                                    bool nowDead = (val & UNIT_DYNFLAG_DEAD) != 0;
                                    if (!wasDead && nowDead) {
                                        if (!npcDeathNotified && npcDeathCallback_) {
                                            npcDeathCallback_(block.guid);
                                            npcDeathNotified = true;
                                        }
                                    } else if (wasDead && !nowDead) {
                                        if (!npcRespawnNotified && npcRespawnCallback_) {
                                            npcRespawnCallback_(block.guid);
                                            npcRespawnNotified = true;
                                        }
                                    }
                                }
                            } else if (key == ufLevel) {
                                uint32_t oldLvl = unit->getLevel();
                                unit->setLevel(val);
                                if (block.guid != playerGuid &&
                                    entity->getType() == ObjectType::PLAYER &&
                                    val > oldLvl && oldLvl > 0 &&
                                    otherPlayerLevelUpCallback_) {
                                    otherPlayerLevelUpCallback_(block.guid, val);
                                }
                            }
                            else if (key == ufFaction) {
                                unit->setFactionTemplate(val);
                                unit->setHostile(isHostileFaction(val));
                            } else if (key == ufDisplayId) {
                                if (val != unit->getDisplayId()) {
                                    unit->setDisplayId(val);
                                    displayIdChanged = true;
                                }
                            } else if (key == ufMountDisplayId) {
                                if (block.guid == playerGuid) {
                                    uint32_t old = currentMountDisplayId_;
                                    currentMountDisplayId_ = val;
                                    if (val != old && mountCallback_) mountCallback_(val);
                                    if (old == 0 && val != 0) {
                                        mountAuraSpellId_ = 0;
                                        for (const auto& a : playerAuras) {
                                            if (!a.isEmpty() && a.maxDurationMs < 0 && a.casterGuid == playerGuid) {
                                                mountAuraSpellId_ = a.spellId;
                                            }
                                        }
                                        // Classic/vanilla fallback: scan UNIT_FIELD_AURAS from same update block
                                        if (mountAuraSpellId_ == 0) {
                                            const uint16_t ufAuras = fieldIndex(UF::UNIT_FIELD_AURAS);
                                            if (ufAuras != 0xFFFF) {
                                                for (const auto& [fk, fv] : block.fields) {
                                                    if (fk >= ufAuras && fk < ufAuras + 48 && fv != 0) {
                                                        mountAuraSpellId_ = fv;
                                                        break;
                                                    }
                                                }
                                            }
                                        }
                                        LOG_INFO("Mount detected (values update): displayId=", val, " auraSpellId=", mountAuraSpellId_);
                                    }
                                    if (old != 0 && val == 0) {
                                        mountAuraSpellId_ = 0;
                                        for (auto& a : playerAuras)
                                            if (!a.isEmpty() && a.maxDurationMs < 0) a = AuraSlot{};
                                    }
                                }
                                unit->setMountDisplayId(val);
                            } else if (key == ufNpcFlags) { unit->setNpcFlags(val); }
                            // Power/maxpower range checks AFTER all specific fields
                            else if (key >= ufPowerBase && key < ufPowerBase + 7) {
                                unit->setPowerByType(static_cast<uint8_t>(key - ufPowerBase), val);
                            } else if (key >= ufMaxPowerBase && key < ufMaxPowerBase + 7) {
                                unit->setMaxPowerByType(static_cast<uint8_t>(key - ufMaxPowerBase), val);
                            }
                        }

                        // Some units/players are created without displayId and get it later via VALUES.
                        if ((entity->getType() == ObjectType::UNIT || entity->getType() == ObjectType::PLAYER) &&
                            displayIdChanged &&
                            unit->getDisplayId() != 0 &&
                            unit->getDisplayId() != oldDisplayId) {
                            if (entity->getType() == ObjectType::PLAYER && block.guid == playerGuid) {
                                // Skip local player — spawned separately
                            } else if (entity->getType() == ObjectType::PLAYER) {
                                if (playerSpawnCallback_) {
                                    uint8_t race = 0, gender = 0, facial = 0;
                                    uint32_t appearanceBytes = 0;
                                    // Use the entity's accumulated field state, not just this block's changed fields.
                                    if (extractPlayerAppearance(entity->getFields(), race, gender, appearanceBytes, facial)) {
                                        playerSpawnCallback_(block.guid, unit->getDisplayId(), race, gender,
                                                            appearanceBytes, facial,
                                                            unit->getX(), unit->getY(), unit->getZ(), unit->getOrientation());
                                    }
                                }
                            } else if (creatureSpawnCallback_) {
                                creatureSpawnCallback_(block.guid, unit->getDisplayId(),
                                    unit->getX(), unit->getY(), unit->getZ(), unit->getOrientation());
                                bool isDeadNow = (unit->getHealth() == 0) ||
                                    ((unit->getDynamicFlags() & (UNIT_DYNFLAG_DEAD | UNIT_DYNFLAG_LOOTABLE)) != 0);
                                if (isDeadNow && !npcDeathNotified && npcDeathCallback_) {
                                    npcDeathCallback_(block.guid);
                                    npcDeathNotified = true;
                                }
                            }
                            if (entity->getType() == ObjectType::UNIT && (unit->getNpcFlags() & 0x02) && socket) {
                                network::Packet qsPkt(wireOpcode(Opcode::CMSG_QUESTGIVER_STATUS_QUERY));
                                qsPkt.writeUInt64(block.guid);
                                socket->send(qsPkt);
                            }
                        }
                    }
                    // Update XP / inventory slot / skill fields for player entity
                    if (block.guid == playerGuid) {
                        std::map<uint16_t, uint32_t> oldFieldsSnapshot = lastPlayerFields_;
                        if (block.hasMovement && block.runSpeed > 0.1f && block.runSpeed < 100.0f) {
                            serverRunSpeed_ = block.runSpeed;
                            // Some server dismount paths update run speed without updating mount display field.
                            if (!onTaxiFlight_ && !taxiMountActive_ &&
                                currentMountDisplayId_ != 0 && block.runSpeed <= 8.5f) {
                                LOG_INFO("Auto-clearing mount from movement speed update: speed=", block.runSpeed,
                                         " displayId=", currentMountDisplayId_);
                                currentMountDisplayId_ = 0;
                                if (mountCallback_) {
                                    mountCallback_(0);
                                }
                            }
                        }
                        for (const auto& [key, val] : block.fields) {
                            lastPlayerFields_[key] = val;
                        }
                        maybeDetectCoinageIndex(oldFieldsSnapshot, lastPlayerFields_);
                        maybeDetectVisibleItemLayout();
                        detectInventorySlotBases(block.fields);
                        bool slotsChanged = false;
                        const uint16_t ufPlayerXp = fieldIndex(UF::PLAYER_XP);
                        const uint16_t ufPlayerNextXp = fieldIndex(UF::PLAYER_NEXT_LEVEL_XP);
                        const uint16_t ufPlayerLevel = fieldIndex(UF::UNIT_FIELD_LEVEL);
                        const uint16_t ufCoinage = fieldIndex(UF::PLAYER_FIELD_COINAGE);
                        const uint16_t ufPlayerFlags = fieldIndex(UF::PLAYER_FLAGS);
                        const uint16_t ufArmor = fieldIndex(UF::UNIT_FIELD_RESISTANCES);
                        for (const auto& [key, val] : block.fields) {
                            if (key == ufPlayerXp) {
                                playerXp_ = val;
                                LOG_INFO("XP updated: ", val);
                            }
                            else if (key == ufPlayerNextXp) {
                                playerNextLevelXp_ = val;
                                LOG_INFO("Next level XP updated: ", val);
                            }
                            else if (key == ufPlayerLevel) {
                                serverPlayerLevel_ = val;
                                LOG_INFO("Level updated: ", val);
                                for (auto& ch : characters) {
                                    if (ch.guid == playerGuid) {
                                        ch.level = val;
                                        break;
                                    }
                                }
                            }
                            else if (key == ufCoinage) {
                                playerMoneyCopper_ = val;
                                LOG_INFO("Money updated via VALUES: ", val, " copper");
                            }
                            else if (ufArmor != 0xFFFF && key == ufArmor) {
                                playerArmorRating_ = static_cast<int32_t>(val);
                            }
                            else if (key == ufPlayerFlags) {
                                constexpr uint32_t PLAYER_FLAGS_GHOST = 0x00000010;
                                bool wasGhost = releasedSpirit_;
                                bool nowGhost = (val & PLAYER_FLAGS_GHOST) != 0;
                                if (!wasGhost && nowGhost) {
                                    releasedSpirit_ = true;
                                    LOG_INFO("Player entered ghost form (PLAYER_FLAGS)");
                                } else if (wasGhost && !nowGhost) {
                                    releasedSpirit_ = false;
                                    playerDead_ = false;
                                    repopPending_ = false;
                                    resurrectPending_ = false;
                                    LOG_INFO("Player resurrected (PLAYER_FLAGS ghost cleared)");
                                }
                            }
                        }
                        // Do not auto-create quests from VALUES quest-log slot fields for the
                        // same reason as CREATE_OBJECT2 above (can be misaligned per realm).
                        if (applyInventoryFields(block.fields)) slotsChanged = true;
                        if (slotsChanged) rebuildOnlineInventory();
                        extractSkillFields(lastPlayerFields_);
                        extractExploredZoneFields(lastPlayerFields_);
                    }

                    // Update item stack count for online items
                    if (entity->getType() == ObjectType::ITEM || entity->getType() == ObjectType::CONTAINER) {
                        for (const auto& [key, val] : block.fields) {
                            if (key == fieldIndex(UF::ITEM_FIELD_STACK_COUNT)) {
                                auto it = onlineItems_.find(block.guid);
                                if (it != onlineItems_.end()) it->second.stackCount = val;
                            }
                        }
                        // Update container slot GUIDs on bag content changes
                        if (entity->getType() == ObjectType::CONTAINER) {
                            extractContainerFields(block.guid, block.fields);
                        }
                        rebuildOnlineInventory();
                    }
                    if (block.hasMovement && entity->getType() == ObjectType::GAMEOBJECT) {
                        if (transportGuids_.count(block.guid) && transportMoveCallback_) {
                            serverUpdatedTransportGuids_.insert(block.guid);
                            transportMoveCallback_(block.guid, entity->getX(), entity->getY(),
                                                   entity->getZ(), entity->getOrientation());
                        } else if (gameObjectMoveCallback_) {
                            gameObjectMoveCallback_(block.guid, entity->getX(), entity->getY(),
                                                    entity->getZ(), entity->getOrientation());
                        }
                    }

                    LOG_DEBUG("Updated entity fields: 0x", std::hex, block.guid, std::dec);
                } else {
                }
                break;
            }

            case UpdateType::MOVEMENT: {
                // Diagnostic: Log if we receive MOVEMENT blocks for transports
                if (transportGuids_.count(block.guid)) {
                    LOG_INFO("MOVEMENT update for transport 0x", std::hex, block.guid, std::dec,
                             " pos=(", block.x, ", ", block.y, ", ", block.z, ")");
                }

                // Update entity position (server → canonical)
                auto entity = entityManager.getEntity(block.guid);
                if (entity) {
                    glm::vec3 pos = core::coords::serverToCanonical(glm::vec3(block.x, block.y, block.z));
                    float oCanonical = core::coords::serverToCanonicalYaw(block.orientation);
                    entity->setPosition(pos.x, pos.y, pos.z, oCanonical);
                    LOG_DEBUG("Updated entity position: 0x", std::hex, block.guid, std::dec);

                    if (block.guid != playerGuid &&
                        (entity->getType() == ObjectType::UNIT || entity->getType() == ObjectType::GAMEOBJECT)) {
                        if (block.onTransport && block.transportGuid != 0) {
                            glm::vec3 localOffset = core::coords::serverToCanonical(
                                glm::vec3(block.transportX, block.transportY, block.transportZ));
                            const bool hasLocalOrientation = (block.updateFlags & 0x0020) != 0; // UPDATEFLAG_LIVING
                            float localOriCanonical = core::coords::normalizeAngleRad(-block.transportO);
                            setTransportAttachment(block.guid, entity->getType(), block.transportGuid,
                                                   localOffset, hasLocalOrientation, localOriCanonical);
                            if (transportManager_ && transportManager_->getTransport(block.transportGuid)) {
                                glm::vec3 composed = transportManager_->getPlayerWorldPosition(block.transportGuid, localOffset);
                                entity->setPosition(composed.x, composed.y, composed.z, entity->getOrientation());
                            }
                        } else {
                            clearTransportAttachment(block.guid);
                        }
                    }

                    if (block.guid == playerGuid) {
                        movementInfo.orientation = oCanonical;

                        // Track player-on-transport state from MOVEMENT updates
                        if (block.onTransport) {
                            setPlayerOnTransport(block.transportGuid, glm::vec3(0.0f));
                            // Convert transport offset from server → canonical coordinates
                            glm::vec3 serverOffset(block.transportX, block.transportY, block.transportZ);
                            playerTransportOffset_ = core::coords::serverToCanonical(serverOffset);
                            if (transportManager_ && transportManager_->getTransport(playerTransportGuid_)) {
                                glm::vec3 composed = transportManager_->getPlayerWorldPosition(playerTransportGuid_, playerTransportOffset_);
                                entity->setPosition(composed.x, composed.y, composed.z, oCanonical);
                                movementInfo.x = composed.x;
                                movementInfo.y = composed.y;
                                movementInfo.z = composed.z;
                            } else {
                                movementInfo.x = pos.x;
                                movementInfo.y = pos.y;
                                movementInfo.z = pos.z;
                            }
                            LOG_INFO("Player on transport (MOVEMENT): 0x", std::hex, playerTransportGuid_, std::dec);
                        } else {
                            movementInfo.x = pos.x;
                            movementInfo.y = pos.y;
                            movementInfo.z = pos.z;
                            if (playerTransportGuid_ != 0) {
                                LOG_INFO("Player left transport (MOVEMENT)");
                                clearPlayerTransport();
                            }
                        }
                    }

                    // Fire transport move callback if this is a known transport
                    if (transportGuids_.count(block.guid) && transportMoveCallback_) {
                        serverUpdatedTransportGuids_.insert(block.guid);
                        transportMoveCallback_(block.guid, pos.x, pos.y, pos.z, oCanonical);
                    }
                    // Fire move callback for non-transport gameobjects.
                    if (entity->getType() == ObjectType::GAMEOBJECT &&
                        transportGuids_.count(block.guid) == 0 &&
                        gameObjectMoveCallback_) {
                        gameObjectMoveCallback_(block.guid, entity->getX(), entity->getY(),
                                                entity->getZ(), entity->getOrientation());
                    }
                    // Fire move callback for non-player units (creatures).
                    // SMSG_MONSTER_MOVE handles smooth interpolated movement, but many
                    // servers (especially vanilla/Turtle WoW) communicate NPC positions
                    // via MOVEMENT blocks instead. Use duration=0 for an instant snap.
                    if (block.guid != playerGuid &&
                        entity->getType() == ObjectType::UNIT &&
                        transportGuids_.count(block.guid) == 0 &&
                        creatureMoveCallback_) {
                        creatureMoveCallback_(block.guid, pos.x, pos.y, pos.z, 0);
                    }
                } else {
                    LOG_WARNING("MOVEMENT update for unknown entity: 0x", std::hex, block.guid, std::dec);
                }
                break;
            }

            default:
                break;
        }
    }

    tabCycleStale = true;
    // Entity count logging disabled

    // Late inventory base detection once items are known
    if (playerGuid != 0 && invSlotBase_ < 0 && !lastPlayerFields_.empty() && !onlineItems_.empty()) {
        detectInventorySlotBases(lastPlayerFields_);
        if (invSlotBase_ >= 0) {
            if (applyInventoryFields(lastPlayerFields_)) {
                rebuildOnlineInventory();
            }
        }
    }
}

void GameHandler::handleCompressedUpdateObject(network::Packet& packet) {
    LOG_DEBUG("Handling SMSG_COMPRESSED_UPDATE_OBJECT, packet size: ", packet.getSize());

    // First 4 bytes = decompressed size
    if (packet.getSize() < 4) {
        LOG_WARNING("SMSG_COMPRESSED_UPDATE_OBJECT too small");
        return;
    }

    uint32_t decompressedSize = packet.readUInt32();
    LOG_DEBUG("  Decompressed size: ", decompressedSize);

    if (decompressedSize == 0 || decompressedSize > 1024 * 1024) {
        LOG_WARNING("Invalid decompressed size: ", decompressedSize);
        return;
    }

    // Remaining data is zlib compressed
    size_t compressedSize = packet.getSize() - packet.getReadPos();
    const uint8_t* compressedData = packet.getData().data() + packet.getReadPos();

    // Decompress
    std::vector<uint8_t> decompressed(decompressedSize);
    uLongf destLen = decompressedSize;
    int ret = uncompress(decompressed.data(), &destLen, compressedData, compressedSize);

    if (ret != Z_OK) {
        LOG_WARNING("Failed to decompress UPDATE_OBJECT: zlib error ", ret);
        return;
    }

    // Create packet from decompressed data and parse it
    network::Packet decompressedPacket(wireOpcode(Opcode::SMSG_UPDATE_OBJECT), decompressed);
    handleUpdateObject(decompressedPacket);
}

void GameHandler::handleDestroyObject(network::Packet& packet) {
    LOG_DEBUG("Handling SMSG_DESTROY_OBJECT");

    DestroyObjectData data;
    if (!DestroyObjectParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_DESTROY_OBJECT");
        return;
    }

    // Remove entity
    if (entityManager.hasEntity(data.guid)) {
        if (transportGuids_.count(data.guid) > 0) {
            const bool playerAboardNow = (playerTransportGuid_ == data.guid);
            const bool stickyAboard = (playerTransportStickyGuid_ == data.guid && playerTransportStickyTimer_ > 0.0f);
            const bool movementSaysAboard = (movementInfo.transportGuid == data.guid);
            if (playerAboardNow || stickyAboard || movementSaysAboard) {
                serverUpdatedTransportGuids_.erase(data.guid);
                LOG_INFO("Preserving in-use transport on destroy: 0x", std::hex, data.guid, std::dec,
                         " now=", playerAboardNow,
                         " sticky=", stickyAboard,
                         " movement=", movementSaysAboard);
                return;
            }
        }
        // Mirror out-of-range handling: invoke render-layer despawn callbacks before entity removal.
        auto entity = entityManager.getEntity(data.guid);
        if (entity) {
            if (entity->getType() == ObjectType::UNIT && creatureDespawnCallback_) {
                creatureDespawnCallback_(data.guid);
            } else if (entity->getType() == ObjectType::GAMEOBJECT && gameObjectDespawnCallback_) {
                gameObjectDespawnCallback_(data.guid);
            }
        }
        if (transportGuids_.count(data.guid) > 0) {
            transportGuids_.erase(data.guid);
            serverUpdatedTransportGuids_.erase(data.guid);
            if (playerTransportGuid_ == data.guid) {
                clearPlayerTransport();
            }
        }
        clearTransportAttachment(data.guid);
        entityManager.removeEntity(data.guid);
        LOG_INFO("Destroyed entity: 0x", std::hex, data.guid, std::dec,
                 " (", (data.isDeath ? "death" : "despawn"), ")");
    } else {
        LOG_DEBUG("Destroy object for unknown entity: 0x", std::hex, data.guid, std::dec);
    }

    // Clean up auto-attack and target if destroyed entity was our target
    if (data.guid == autoAttackTarget) {
        stopAutoAttack();
    }
    if (data.guid == targetGuid) {
        targetGuid = 0;
    }
    hostileAttackers_.erase(data.guid);

    // Remove online item/container tracking
    containerContents_.erase(data.guid);
    if (onlineItems_.erase(data.guid)) {
        rebuildOnlineInventory();
    }

    // Clean up quest giver status
    npcQuestStatus_.erase(data.guid);

    tabCycleStale = true;
    // Entity count logging disabled
}

void GameHandler::sendChatMessage(ChatType type, const std::string& message, const std::string& target) {
    if (state != WorldState::IN_WORLD) {
        LOG_WARNING("Cannot send chat in state: ", (int)state);
        return;
    }

    if (message.empty()) {
        LOG_WARNING("Cannot send empty chat message");
        return;
    }

    LOG_INFO("Sending chat message: [", getChatTypeString(type), "] ", message);

    // Determine language based on character (for now, use COMMON)
    ChatLanguage language = ChatLanguage::COMMON;

    // Build and send packet
    auto packet = MessageChatPacket::build(type, language, message, target);
    socket->send(packet);

    // Add local echo so the player sees their own message immediately
    MessageChatData echo;
    echo.senderGuid = playerGuid;
    echo.language = language;
    echo.message = message;

    // Look up player name
    auto nameIt = playerNameCache.find(playerGuid);
    if (nameIt != playerNameCache.end()) {
        echo.senderName = nameIt->second;
    }

    if (type == ChatType::WHISPER) {
        echo.type = ChatType::WHISPER_INFORM;
        echo.senderName = target;  // "To [target]: message"
    } else {
        echo.type = type;
    }

    if (type == ChatType::CHANNEL) {
        echo.channelName = target;
    }

    addLocalChatMessage(echo);
}

void GameHandler::handleMessageChat(network::Packet& packet) {
    LOG_DEBUG("Handling SMSG_MESSAGECHAT");

    MessageChatData data;
    if (!packetParsers_->parseMessageChat(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_MESSAGECHAT");
        return;
    }

    // Skip server echo of our own messages (we already added a local echo)
    if (data.senderGuid == playerGuid && data.senderGuid != 0) {
        // Still track whisper sender for /r even if it's our own whisper-inform
        if (data.type == ChatType::WHISPER && !data.senderName.empty()) {
            lastWhisperSender_ = data.senderName;
        }
        return;
    }

    // Resolve sender name from entity/cache if not already set by parser
    if (data.senderName.empty() && data.senderGuid != 0) {
        // Check player name cache first
        auto nameIt = playerNameCache.find(data.senderGuid);
        if (nameIt != playerNameCache.end()) {
            data.senderName = nameIt->second;
        } else {
            // Try entity name
            auto entity = entityManager.getEntity(data.senderGuid);
            if (entity) {
                if (entity->getType() == ObjectType::PLAYER) {
                    auto player = std::dynamic_pointer_cast<Player>(entity);
                    if (player && !player->getName().empty()) {
                        data.senderName = player->getName();
                    }
                } else if (entity->getType() == ObjectType::UNIT) {
                    auto unit = std::dynamic_pointer_cast<Unit>(entity);
                    if (unit && !unit->getName().empty()) {
                        data.senderName = unit->getName();
                    }
                }
            }
        }

        // If still unknown, proactively query the server so the UI can show names soon after.
        if (data.senderName.empty()) {
            queryPlayerName(data.senderGuid);
        }
    }

    // Add to chat history
    chatHistory.push_back(data);

    // Limit chat history size
    if (chatHistory.size() > maxChatHistory) {
        chatHistory.erase(chatHistory.begin());
    }

    // Track whisper sender for /r command
    if (data.type == ChatType::WHISPER && !data.senderName.empty()) {
        lastWhisperSender_ = data.senderName;
    }

    // Trigger chat bubble for SAY/YELL messages from others
    if (chatBubbleCallback_ && data.senderGuid != 0) {
        if (data.type == ChatType::SAY || data.type == ChatType::YELL ||
            data.type == ChatType::MONSTER_SAY || data.type == ChatType::MONSTER_YELL) {
            bool isYell = (data.type == ChatType::YELL || data.type == ChatType::MONSTER_YELL);
            chatBubbleCallback_(data.senderGuid, data.message, isYell);
        }
    }

    // Log the message
    std::string senderInfo;
    if (!data.senderName.empty()) {
        senderInfo = data.senderName;
    } else if (data.senderGuid != 0) {
        senderInfo = "Unknown-" + std::to_string(data.senderGuid);
    } else {
        senderInfo = "System";
    }

    std::string channelInfo;
    if (!data.channelName.empty()) {
        channelInfo = "[" + data.channelName + "] ";
    }

    LOG_DEBUG("[", getChatTypeString(data.type), "] ", channelInfo, senderInfo, ": ", data.message);
}

void GameHandler::sendTextEmote(uint32_t textEmoteId, uint64_t targetGuid) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = TextEmotePacket::build(textEmoteId, targetGuid);
    socket->send(packet);
}

void GameHandler::handleTextEmote(network::Packet& packet) {
    TextEmoteData data;
    if (!TextEmoteParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_TEXT_EMOTE");
        return;
    }

    // Skip our own text emotes (we already have local echo)
    if (data.senderGuid == playerGuid && data.senderGuid != 0) {
        return;
    }

    // Resolve sender name
    std::string senderName;
    auto nameIt = playerNameCache.find(data.senderGuid);
    if (nameIt != playerNameCache.end()) {
        senderName = nameIt->second;
    } else {
        auto entity = entityManager.getEntity(data.senderGuid);
        if (entity) {
            auto unit = std::dynamic_pointer_cast<Unit>(entity);
            if (unit) senderName = unit->getName();
        }
    }
    if (senderName.empty()) {
        senderName = "Unknown";
        queryPlayerName(data.senderGuid);
    }

    // Resolve emote text from DBC using third-person "others see" templates
    const std::string* targetPtr = data.targetName.empty() ? nullptr : &data.targetName;
    std::string emoteText = rendering::Renderer::getEmoteTextByDbcId(data.textEmoteId, senderName, targetPtr);
    if (emoteText.empty()) {
        // Fallback if DBC lookup fails
        emoteText = data.targetName.empty()
            ? senderName + " performs an emote."
            : senderName + " performs an emote at " + data.targetName + ".";
    }

    MessageChatData chatMsg;
    chatMsg.type = ChatType::TEXT_EMOTE;
    chatMsg.language = ChatLanguage::COMMON;
    chatMsg.senderGuid = data.senderGuid;
    chatMsg.senderName = senderName;
    chatMsg.message = emoteText;

    chatHistory.push_back(chatMsg);
    if (chatHistory.size() > maxChatHistory) {
        chatHistory.erase(chatHistory.begin());
    }

    // Trigger emote animation on sender's entity via callback
    uint32_t animId = rendering::Renderer::getEmoteAnimByDbcId(data.textEmoteId);
    if (animId != 0 && emoteAnimCallback_) {
        emoteAnimCallback_(data.senderGuid, animId);
    }

    LOG_INFO("TEXT_EMOTE from ", senderName, " (emoteId=", data.textEmoteId, ", anim=", animId, ")");
}

void GameHandler::joinChannel(const std::string& channelName, const std::string& password) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = packetParsers_ ? packetParsers_->buildJoinChannel(channelName, password)
                                 : JoinChannelPacket::build(channelName, password);
    socket->send(packet);
    LOG_INFO("Requesting to join channel: ", channelName);
}

void GameHandler::leaveChannel(const std::string& channelName) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = packetParsers_ ? packetParsers_->buildLeaveChannel(channelName)
                                 : LeaveChannelPacket::build(channelName);
    socket->send(packet);
    LOG_INFO("Requesting to leave channel: ", channelName);
}

std::string GameHandler::getChannelByIndex(int index) const {
    if (index < 1 || index > static_cast<int>(joinedChannels_.size())) return "";
    return joinedChannels_[index - 1];
}

int GameHandler::getChannelIndex(const std::string& channelName) const {
    for (int i = 0; i < static_cast<int>(joinedChannels_.size()); ++i) {
        if (joinedChannels_[i] == channelName) return i + 1;  // 1-based
    }
    return 0;
}

void GameHandler::handleChannelNotify(network::Packet& packet) {
    ChannelNotifyData data;
    if (!ChannelNotifyParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_CHANNEL_NOTIFY");
        return;
    }

    switch (data.notifyType) {
        case ChannelNotifyType::YOU_JOINED: {
            // Add to active channels if not already present
            bool found = false;
            for (const auto& ch : joinedChannels_) {
                if (ch == data.channelName) { found = true; break; }
            }
            if (!found) {
                joinedChannels_.push_back(data.channelName);
            }
            MessageChatData msg;
            msg.type = ChatType::SYSTEM;
            msg.message = "Joined channel: " + data.channelName;
            addLocalChatMessage(msg);
            LOG_INFO("Joined channel: ", data.channelName);
            break;
        }
        case ChannelNotifyType::YOU_LEFT: {
            joinedChannels_.erase(
                std::remove(joinedChannels_.begin(), joinedChannels_.end(), data.channelName),
                joinedChannels_.end());
            MessageChatData msg;
            msg.type = ChatType::SYSTEM;
            msg.message = "Left channel: " + data.channelName;
            addLocalChatMessage(msg);
            LOG_INFO("Left channel: ", data.channelName);
            break;
        }
        case ChannelNotifyType::PLAYER_ALREADY_MEMBER: {
            // Server says we're already in this channel (e.g. server auto-joined us)
            // Still track it in our channel list
            bool found = false;
            for (const auto& ch : joinedChannels_) {
                if (ch == data.channelName) { found = true; break; }
            }
            if (!found) {
                joinedChannels_.push_back(data.channelName);
                LOG_INFO("Already in channel: ", data.channelName);
            }
            break;
        }
        case ChannelNotifyType::NOT_IN_AREA: {
            LOG_DEBUG("Cannot join channel ", data.channelName, " (not in area)");
            break;
        }
        default:
            LOG_DEBUG("Channel notify type ", static_cast<int>(data.notifyType),
                     " for channel ", data.channelName);
            break;
    }
}

void GameHandler::autoJoinDefaultChannels() {
    LOG_INFO("autoJoinDefaultChannels: general=", chatAutoJoin.general,
             " trade=", chatAutoJoin.trade, " localDefense=", chatAutoJoin.localDefense,
             " lfg=", chatAutoJoin.lfg, " local=", chatAutoJoin.local);
    if (chatAutoJoin.general) joinChannel("General");
    if (chatAutoJoin.trade) joinChannel("Trade");
    if (chatAutoJoin.localDefense) joinChannel("LocalDefense");
    if (chatAutoJoin.lfg) joinChannel("LookingForGroup");
    if (chatAutoJoin.local) joinChannel("Local");
}

void GameHandler::setTarget(uint64_t guid) {
    if (guid == targetGuid) return;

    // Save previous target
    if (targetGuid != 0) {
        lastTargetGuid = targetGuid;
    }

    targetGuid = guid;

    // Inform server of target selection (Phase 1)
    if (state == WorldState::IN_WORLD && socket) {
        auto packet = SetSelectionPacket::build(guid);
        socket->send(packet);
    }

    if (guid != 0) {
        LOG_INFO("Target set: 0x", std::hex, guid, std::dec);
    }
}

void GameHandler::clearTarget() {
    if (targetGuid != 0) {
        LOG_INFO("Target cleared");
    }
    targetGuid = 0;
    tabCycleIndex = -1;
    tabCycleStale = true;
}

std::shared_ptr<Entity> GameHandler::getTarget() const {
    if (targetGuid == 0) return nullptr;
    return entityManager.getEntity(targetGuid);
}

void GameHandler::setFocus(uint64_t guid) {
    focusGuid = guid;
    if (guid != 0) {
        auto entity = entityManager.getEntity(guid);
        if (entity) {
            std::string name = "Unknown";
            if (entity->getType() == ObjectType::PLAYER) {
                auto player = std::dynamic_pointer_cast<Player>(entity);
                if (player && !player->getName().empty()) {
                    name = player->getName();
                }
            }
            addSystemChatMessage("Focus set: " + name);
            LOG_INFO("Focus set: 0x", std::hex, guid, std::dec);
        }
    }
}

void GameHandler::clearFocus() {
    if (focusGuid != 0) {
        addSystemChatMessage("Focus cleared.");
        LOG_INFO("Focus cleared");
    }
    focusGuid = 0;
}

std::shared_ptr<Entity> GameHandler::getFocus() const {
    if (focusGuid == 0) return nullptr;
    return entityManager.getEntity(focusGuid);
}

void GameHandler::targetLastTarget() {
    if (lastTargetGuid == 0) {
        addSystemChatMessage("No previous target.");
        return;
    }

    // Swap current and last target
    uint64_t temp = targetGuid;
    setTarget(lastTargetGuid);
    lastTargetGuid = temp;
}

void GameHandler::targetEnemy(bool reverse) {
    // Get list of hostile entities
    std::vector<uint64_t> hostiles;
    auto& entities = entityManager.getEntities();

    for (const auto& [guid, entity] : entities) {
        if (entity->getType() == ObjectType::UNIT) {
            // Check if hostile (this is simplified - would need faction checking)
            auto unit = std::dynamic_pointer_cast<Unit>(entity);
            if (unit && guid != playerGuid) {
                hostiles.push_back(guid);
            }
        }
    }

    if (hostiles.empty()) {
        addSystemChatMessage("No enemies in range.");
        return;
    }

    // Find current target in list
    auto it = std::find(hostiles.begin(), hostiles.end(), targetGuid);

    if (it == hostiles.end()) {
        // Not currently targeting a hostile, target first one
        setTarget(reverse ? hostiles.back() : hostiles.front());
    } else {
        // Cycle to next/previous
        if (reverse) {
            if (it == hostiles.begin()) {
                setTarget(hostiles.back());
            } else {
                setTarget(*(--it));
            }
        } else {
            ++it;
            if (it == hostiles.end()) {
                setTarget(hostiles.front());
            } else {
                setTarget(*it);
            }
        }
    }
}

void GameHandler::targetFriend(bool reverse) {
    // Get list of friendly entities (players)
    std::vector<uint64_t> friendlies;
    auto& entities = entityManager.getEntities();

    for (const auto& [guid, entity] : entities) {
        if (entity->getType() == ObjectType::PLAYER && guid != playerGuid) {
            friendlies.push_back(guid);
        }
    }

    if (friendlies.empty()) {
        addSystemChatMessage("No friendly targets in range.");
        return;
    }

    // Find current target in list
    auto it = std::find(friendlies.begin(), friendlies.end(), targetGuid);

    if (it == friendlies.end()) {
        // Not currently targeting a friend, target first one
        setTarget(reverse ? friendlies.back() : friendlies.front());
    } else {
        // Cycle to next/previous
        if (reverse) {
            if (it == friendlies.begin()) {
                setTarget(friendlies.back());
            } else {
                setTarget(*(--it));
            }
        } else {
            ++it;
            if (it == friendlies.end()) {
                setTarget(friendlies.front());
            } else {
                setTarget(*it);
            }
        }
    }
}

void GameHandler::inspectTarget() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot inspect: not in world or not connected");
        return;
    }

    if (targetGuid == 0) {
        addSystemChatMessage("You must target a player to inspect.");
        return;
    }

    auto target = getTarget();
    if (!target || target->getType() != ObjectType::PLAYER) {
        addSystemChatMessage("You can only inspect players.");
        return;
    }

    auto packet = InspectPacket::build(targetGuid);
    socket->send(packet);

    auto player = std::static_pointer_cast<Player>(target);
    std::string name = player->getName().empty() ? "Target" : player->getName();
    addSystemChatMessage("Inspecting " + name + "...");
    LOG_INFO("Sent inspect request for player: ", name, " (GUID: 0x", std::hex, targetGuid, std::dec, ")");
}

void GameHandler::queryServerTime() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot query time: not in world or not connected");
        return;
    }

    auto packet = QueryTimePacket::build();
    socket->send(packet);
    LOG_INFO("Requested server time");
}

void GameHandler::requestPlayedTime() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot request played time: not in world or not connected");
        return;
    }

    auto packet = RequestPlayedTimePacket::build(true);
    socket->send(packet);
    LOG_INFO("Requested played time");
}

void GameHandler::queryWho(const std::string& playerName) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot query who: not in world or not connected");
        return;
    }

    auto packet = WhoPacket::build(0, 0, playerName);
    socket->send(packet);
    LOG_INFO("Sent WHO query", playerName.empty() ? "" : " for: " + playerName);
}

void GameHandler::addFriend(const std::string& playerName, const std::string& note) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot add friend: not in world or not connected");
        return;
    }

    if (playerName.empty()) {
        addSystemChatMessage("You must specify a player name.");
        return;
    }

    auto packet = AddFriendPacket::build(playerName, note);
    socket->send(packet);
    addSystemChatMessage("Sending friend request to " + playerName + "...");
    LOG_INFO("Sent friend request to: ", playerName);
}

void GameHandler::removeFriend(const std::string& playerName) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot remove friend: not in world or not connected");
        return;
    }

    if (playerName.empty()) {
        addSystemChatMessage("You must specify a player name.");
        return;
    }

    // Look up GUID from cache
    auto it = friendsCache.find(playerName);
    if (it == friendsCache.end()) {
        addSystemChatMessage(playerName + " is not in your friends list.");
        LOG_WARNING("Friend not found in cache: ", playerName);
        return;
    }

    auto packet = DelFriendPacket::build(it->second);
    socket->send(packet);
    addSystemChatMessage("Removing " + playerName + " from friends list...");
    LOG_INFO("Sent remove friend request for: ", playerName, " (GUID: 0x", std::hex, it->second, std::dec, ")");
}

void GameHandler::setFriendNote(const std::string& playerName, const std::string& note) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot set friend note: not in world or not connected");
        return;
    }

    if (playerName.empty()) {
        addSystemChatMessage("You must specify a player name.");
        return;
    }

    // Look up GUID from cache
    auto it = friendsCache.find(playerName);
    if (it == friendsCache.end()) {
        addSystemChatMessage(playerName + " is not in your friends list.");
        return;
    }

    auto packet = SetContactNotesPacket::build(it->second, note);
    socket->send(packet);
    addSystemChatMessage("Updated note for " + playerName);
    LOG_INFO("Set friend note for: ", playerName);
}

void GameHandler::randomRoll(uint32_t minRoll, uint32_t maxRoll) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot roll: not in world or not connected");
        return;
    }

    if (minRoll > maxRoll) {
        std::swap(minRoll, maxRoll);
    }

    if (maxRoll > 10000) {
        maxRoll = 10000;  // Cap at reasonable value
    }

    auto packet = RandomRollPacket::build(minRoll, maxRoll);
    socket->send(packet);
    LOG_INFO("Rolled ", minRoll, "-", maxRoll);
}

void GameHandler::addIgnore(const std::string& playerName) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot add ignore: not in world or not connected");
        return;
    }

    if (playerName.empty()) {
        addSystemChatMessage("You must specify a player name.");
        return;
    }

    auto packet = AddIgnorePacket::build(playerName);
    socket->send(packet);
    addSystemChatMessage("Adding " + playerName + " to ignore list...");
    LOG_INFO("Sent ignore request for: ", playerName);
}

void GameHandler::removeIgnore(const std::string& playerName) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot remove ignore: not in world or not connected");
        return;
    }

    if (playerName.empty()) {
        addSystemChatMessage("You must specify a player name.");
        return;
    }

    // Look up GUID from cache
    auto it = ignoreCache.find(playerName);
    if (it == ignoreCache.end()) {
        addSystemChatMessage(playerName + " is not in your ignore list.");
        LOG_WARNING("Ignored player not found in cache: ", playerName);
        return;
    }

    auto packet = DelIgnorePacket::build(it->second);
    socket->send(packet);
    addSystemChatMessage("Removing " + playerName + " from ignore list...");
    ignoreCache.erase(it);
    LOG_INFO("Sent remove ignore request for: ", playerName, " (GUID: 0x", std::hex, it->second, std::dec, ")");
}

void GameHandler::requestLogout() {
    if (!socket) {
        LOG_WARNING("Cannot logout: not connected");
        return;
    }

    if (loggingOut_) {
        addSystemChatMessage("Already logging out.");
        return;
    }

    auto packet = LogoutRequestPacket::build();
    socket->send(packet);
    loggingOut_ = true;
    LOG_INFO("Sent logout request");
}

void GameHandler::cancelLogout() {
    if (!socket) {
        LOG_WARNING("Cannot cancel logout: not connected");
        return;
    }

    if (!loggingOut_) {
        addSystemChatMessage("Not currently logging out.");
        return;
    }

    auto packet = LogoutCancelPacket::build();
    socket->send(packet);
    loggingOut_ = false;
    addSystemChatMessage("Logout cancelled.");
    LOG_INFO("Cancelled logout");
}

void GameHandler::setStandState(uint8_t standState) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot change stand state: not in world or not connected");
        return;
    }

    auto packet = StandStateChangePacket::build(standState);
    socket->send(packet);
    LOG_INFO("Changed stand state to: ", (int)standState);
}

void GameHandler::toggleHelm() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot toggle helm: not in world or not connected");
        return;
    }

    helmVisible_ = !helmVisible_;
    auto packet = ShowingHelmPacket::build(helmVisible_);
    socket->send(packet);
    addSystemChatMessage(helmVisible_ ? "Helm is now visible." : "Helm is now hidden.");
    LOG_INFO("Helm visibility toggled: ", helmVisible_);
}

void GameHandler::toggleCloak() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot toggle cloak: not in world or not connected");
        return;
    }

    cloakVisible_ = !cloakVisible_;
    auto packet = ShowingCloakPacket::build(cloakVisible_);
    socket->send(packet);
    addSystemChatMessage(cloakVisible_ ? "Cloak is now visible." : "Cloak is now hidden.");
    LOG_INFO("Cloak visibility toggled: ", cloakVisible_);
}

void GameHandler::followTarget() {
    if (state != WorldState::IN_WORLD) {
        LOG_WARNING("Cannot follow: not in world");
        return;
    }

    if (targetGuid == 0) {
        addSystemChatMessage("You must target someone to follow.");
        return;
    }

    auto target = getTarget();
    if (!target) {
        addSystemChatMessage("Invalid target.");
        return;
    }

    // Set follow target
    followTargetGuid_ = targetGuid;

    // Get target name
    std::string targetName = "Target";
    if (target->getType() == ObjectType::PLAYER) {
        auto player = std::static_pointer_cast<Player>(target);
        if (!player->getName().empty()) {
            targetName = player->getName();
        }
    } else if (target->getType() == ObjectType::UNIT) {
        auto unit = std::static_pointer_cast<Unit>(target);
        targetName = unit->getName();
    }

    addSystemChatMessage("Now following " + targetName + ".");
    LOG_INFO("Following target: ", targetName, " (GUID: 0x", std::hex, targetGuid, std::dec, ")");
}

void GameHandler::assistTarget() {
    if (state != WorldState::IN_WORLD) {
        LOG_WARNING("Cannot assist: not in world");
        return;
    }

    if (targetGuid == 0) {
        addSystemChatMessage("You must target someone to assist.");
        return;
    }

    auto target = getTarget();
    if (!target) {
        addSystemChatMessage("Invalid target.");
        return;
    }

    // Get target name
    std::string targetName = "Target";
    if (target->getType() == ObjectType::PLAYER) {
        auto player = std::static_pointer_cast<Player>(target);
        if (!player->getName().empty()) {
            targetName = player->getName();
        }
    } else if (target->getType() == ObjectType::UNIT) {
        auto unit = std::static_pointer_cast<Unit>(target);
        targetName = unit->getName();
    }

    // Try to read target GUID from update fields (UNIT_FIELD_TARGET)
    uint64_t assistTargetGuid = 0;
    const auto& fields = target->getFields();
    auto it = fields.find(fieldIndex(UF::UNIT_FIELD_TARGET_LO));
    if (it != fields.end()) {
        assistTargetGuid = it->second;
        auto it2 = fields.find(fieldIndex(UF::UNIT_FIELD_TARGET_HI));
        if (it2 != fields.end()) {
            assistTargetGuid |= (static_cast<uint64_t>(it2->second) << 32);
        }
    }

    if (assistTargetGuid == 0) {
        addSystemChatMessage(targetName + " has no target.");
        LOG_INFO("Assist: ", targetName, " has no target");
        return;
    }

    // Set our target to their target
    setTarget(assistTargetGuid);
    LOG_INFO("Assisting ", targetName, ", now targeting GUID: 0x", std::hex, assistTargetGuid, std::dec);
}

void GameHandler::togglePvp() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot toggle PvP: not in world or not connected");
        return;
    }

    auto packet = TogglePvpPacket::build();
    socket->send(packet);
    // Check current PVP state from player's UNIT_FIELD_FLAGS (index 59)
    // UNIT_FLAG_PVP = 0x00001000
    auto entity = entityManager.getEntity(playerGuid);
    bool currentlyPvp = false;
    if (entity) {
        currentlyPvp = (entity->getField(59) & 0x00001000) != 0;
    }
    // We're toggling, so report the NEW state
    if (currentlyPvp) {
        addSystemChatMessage("PvP flag disabled.");
    } else {
        addSystemChatMessage("PvP flag enabled.");
    }
    LOG_INFO("Toggled PvP flag");
}

void GameHandler::requestGuildInfo() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot request guild info: not in world or not connected");
        return;
    }

    auto packet = GuildInfoPacket::build();
    socket->send(packet);
    LOG_INFO("Requested guild info");
}

void GameHandler::requestGuildRoster() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot request guild roster: not in world or not connected");
        return;
    }

    auto packet = GuildRosterPacket::build();
    socket->send(packet);
    addSystemChatMessage("Requesting guild roster...");
    LOG_INFO("Requested guild roster");
}

void GameHandler::setGuildMotd(const std::string& motd) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot set guild MOTD: not in world or not connected");
        return;
    }

    auto packet = GuildMotdPacket::build(motd);
    socket->send(packet);
    addSystemChatMessage("Guild MOTD updated.");
    LOG_INFO("Set guild MOTD: ", motd);
}

void GameHandler::promoteGuildMember(const std::string& playerName) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot promote guild member: not in world or not connected");
        return;
    }

    if (playerName.empty()) {
        addSystemChatMessage("You must specify a player name.");
        return;
    }

    auto packet = GuildPromotePacket::build(playerName);
    socket->send(packet);
    addSystemChatMessage("Promoting " + playerName + "...");
    LOG_INFO("Promoting guild member: ", playerName);
}

void GameHandler::demoteGuildMember(const std::string& playerName) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot demote guild member: not in world or not connected");
        return;
    }

    if (playerName.empty()) {
        addSystemChatMessage("You must specify a player name.");
        return;
    }

    auto packet = GuildDemotePacket::build(playerName);
    socket->send(packet);
    addSystemChatMessage("Demoting " + playerName + "...");
    LOG_INFO("Demoting guild member: ", playerName);
}

void GameHandler::leaveGuild() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot leave guild: not in world or not connected");
        return;
    }

    auto packet = GuildLeavePacket::build();
    socket->send(packet);
    addSystemChatMessage("Leaving guild...");
    LOG_INFO("Leaving guild");
}

void GameHandler::inviteToGuild(const std::string& playerName) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot invite to guild: not in world or not connected");
        return;
    }

    if (playerName.empty()) {
        addSystemChatMessage("You must specify a player name.");
        return;
    }

    auto packet = GuildInvitePacket::build(playerName);
    socket->send(packet);
    addSystemChatMessage("Inviting " + playerName + " to guild...");
    LOG_INFO("Inviting to guild: ", playerName);
}

void GameHandler::initiateReadyCheck() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot initiate ready check: not in world or not connected");
        return;
    }

    if (!isInGroup()) {
        addSystemChatMessage("You must be in a group to initiate a ready check.");
        return;
    }

    auto packet = ReadyCheckPacket::build();
    socket->send(packet);
    addSystemChatMessage("Ready check initiated.");
    LOG_INFO("Initiated ready check");
}

void GameHandler::respondToReadyCheck(bool ready) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot respond to ready check: not in world or not connected");
        return;
    }

    auto packet = ReadyCheckConfirmPacket::build(ready);
    socket->send(packet);
    addSystemChatMessage(ready ? "You are ready." : "You are not ready.");
    LOG_INFO("Responded to ready check: ", ready ? "ready" : "not ready");
}

void GameHandler::forfeitDuel() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot forfeit duel: not in world or not connected");
        return;
    }

    auto packet = DuelCancelPacket::build();
    socket->send(packet);
    addSystemChatMessage("You have forfeited the duel.");
    LOG_INFO("Forfeited duel");
}

void GameHandler::toggleAfk(const std::string& message) {
    afkStatus_ = !afkStatus_;
    afkMessage_ = message;

    if (afkStatus_) {
        if (message.empty()) {
            addSystemChatMessage("You are now AFK.");
        } else {
            addSystemChatMessage("You are now AFK: " + message);
        }
        // If DND was active, turn it off
        if (dndStatus_) {
            dndStatus_ = false;
            dndMessage_.clear();
        }
    } else {
        addSystemChatMessage("You are no longer AFK.");
        afkMessage_.clear();
    }

    LOG_INFO("AFK status: ", afkStatus_, ", message: ", message);
}

void GameHandler::toggleDnd(const std::string& message) {
    dndStatus_ = !dndStatus_;
    dndMessage_ = message;

    if (dndStatus_) {
        if (message.empty()) {
            addSystemChatMessage("You are now DND (Do Not Disturb).");
        } else {
            addSystemChatMessage("You are now DND: " + message);
        }
        // If AFK was active, turn it off
        if (afkStatus_) {
            afkStatus_ = false;
            afkMessage_.clear();
        }
    } else {
        addSystemChatMessage("You are no longer DND.");
        dndMessage_.clear();
    }

    LOG_INFO("DND status: ", dndStatus_, ", message: ", message);
}

void GameHandler::replyToLastWhisper(const std::string& message) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot send whisper: not in world or not connected");
        return;
    }

    if (lastWhisperSender_.empty()) {
        addSystemChatMessage("No one has whispered you yet.");
        return;
    }

    if (message.empty()) {
        addSystemChatMessage("You must specify a message to send.");
        return;
    }

    // Send whisper using the standard message chat function
    sendChatMessage(ChatType::WHISPER, message, lastWhisperSender_);
    LOG_INFO("Replied to ", lastWhisperSender_, ": ", message);
}

void GameHandler::uninvitePlayer(const std::string& playerName) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot uninvite player: not in world or not connected");
        return;
    }

    if (playerName.empty()) {
        addSystemChatMessage("You must specify a player name to uninvite.");
        return;
    }

    auto packet = GroupUninvitePacket::build(playerName);
    socket->send(packet);
    addSystemChatMessage("Removed " + playerName + " from the group.");
    LOG_INFO("Uninvited player: ", playerName);
}

void GameHandler::leaveParty() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot leave party: not in world or not connected");
        return;
    }

    auto packet = GroupDisbandPacket::build();
    socket->send(packet);
    addSystemChatMessage("You have left the group.");
    LOG_INFO("Left party/raid");
}

void GameHandler::setMainTank(uint64_t targetGuid) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot set main tank: not in world or not connected");
        return;
    }

    if (targetGuid == 0) {
        addSystemChatMessage("You must have a target selected.");
        return;
    }

    // Main tank uses index 0
    auto packet = RaidTargetUpdatePacket::build(0, targetGuid);
    socket->send(packet);
    addSystemChatMessage("Main tank set.");
    LOG_INFO("Set main tank: 0x", std::hex, targetGuid, std::dec);
}

void GameHandler::setMainAssist(uint64_t targetGuid) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot set main assist: not in world or not connected");
        return;
    }

    if (targetGuid == 0) {
        addSystemChatMessage("You must have a target selected.");
        return;
    }

    // Main assist uses index 1
    auto packet = RaidTargetUpdatePacket::build(1, targetGuid);
    socket->send(packet);
    addSystemChatMessage("Main assist set.");
    LOG_INFO("Set main assist: 0x", std::hex, targetGuid, std::dec);
}

void GameHandler::clearMainTank() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot clear main tank: not in world or not connected");
        return;
    }

    // Clear main tank by setting GUID to 0
    auto packet = RaidTargetUpdatePacket::build(0, 0);
    socket->send(packet);
    addSystemChatMessage("Main tank cleared.");
    LOG_INFO("Cleared main tank");
}

void GameHandler::clearMainAssist() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot clear main assist: not in world or not connected");
        return;
    }

    // Clear main assist by setting GUID to 0
    auto packet = RaidTargetUpdatePacket::build(1, 0);
    socket->send(packet);
    addSystemChatMessage("Main assist cleared.");
    LOG_INFO("Cleared main assist");
}

void GameHandler::requestRaidInfo() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot request raid info: not in world or not connected");
        return;
    }

    auto packet = RequestRaidInfoPacket::build();
    socket->send(packet);
    addSystemChatMessage("Requesting raid lockout information...");
    LOG_INFO("Requested raid info");
}

void GameHandler::proposeDuel(uint64_t targetGuid) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot propose duel: not in world or not connected");
        return;
    }

    if (targetGuid == 0) {
        addSystemChatMessage("You must target a player to challenge to a duel.");
        return;
    }

    auto packet = DuelProposedPacket::build(targetGuid);
    socket->send(packet);
    addSystemChatMessage("You have challenged your target to a duel.");
    LOG_INFO("Proposed duel to target: 0x", std::hex, targetGuid, std::dec);
}

void GameHandler::initiateTrade(uint64_t targetGuid) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot initiate trade: not in world or not connected");
        return;
    }

    if (targetGuid == 0) {
        addSystemChatMessage("You must target a player to trade with.");
        return;
    }

    auto packet = InitiateTradePacket::build(targetGuid);
    socket->send(packet);
    addSystemChatMessage("Requesting trade with target.");
    LOG_INFO("Initiated trade with target: 0x", std::hex, targetGuid, std::dec);
}

void GameHandler::stopCasting() {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("Cannot stop casting: not in world or not connected");
        return;
    }

    if (!casting) {
        return; // Not casting anything
    }

    // Send cancel cast packet only for real spell casts.
    if (pendingGameObjectInteractGuid_ == 0 && currentCastSpellId != 0) {
        auto packet = CancelCastPacket::build(currentCastSpellId);
        socket->send(packet);
    }

    // Reset casting state
    casting = false;
    currentCastSpellId = 0;
    pendingGameObjectInteractGuid_ = 0;
    castTimeRemaining = 0.0f;
    castTimeTotal = 0.0f;

    LOG_INFO("Cancelled spell cast");
}

void GameHandler::releaseSpirit() {
    if (socket && state == WorldState::IN_WORLD) {
        auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (repopPending_ && now - static_cast<int64_t>(lastRepopRequestMs_) < 1000) {
            return;
        }
        auto packet = RepopRequestPacket::build();
        socket->send(packet);
        releasedSpirit_ = true;
        repopPending_ = true;
        lastRepopRequestMs_ = static_cast<uint64_t>(now);
        LOG_INFO("Sent CMSG_REPOP_REQUEST (Release Spirit)");
    }
}

void GameHandler::activateSpiritHealer(uint64_t npcGuid) {
    if (state != WorldState::IN_WORLD || !socket) return;
    pendingSpiritHealerGuid_ = npcGuid;
    auto packet = SpiritHealerActivatePacket::build(npcGuid);
    socket->send(packet);
    resurrectPending_ = true;
    LOG_INFO("Sent CMSG_SPIRIT_HEALER_ACTIVATE for 0x", std::hex, npcGuid, std::dec);
}

void GameHandler::acceptResurrect() {
    if (state != WorldState::IN_WORLD || !socket || !resurrectRequestPending_) return;
    // Send spirit healer activate (correct response to SMSG_SPIRIT_HEALER_CONFIRM)
    auto activate = SpiritHealerActivatePacket::build(resurrectCasterGuid_);
    socket->send(activate);
    LOG_INFO("Sent CMSG_SPIRIT_HEALER_ACTIVATE (0x21C) for 0x",
             std::hex, resurrectCasterGuid_, std::dec);
    resurrectRequestPending_ = false;
    resurrectPending_ = true;
}

void GameHandler::declineResurrect() {
    if (state != WorldState::IN_WORLD || !socket || !resurrectRequestPending_) return;
    auto resp = ResurrectResponsePacket::build(resurrectCasterGuid_, false);
    socket->send(resp);
    LOG_INFO("Sent CMSG_RESURRECT_RESPONSE (decline) for 0x",
             std::hex, resurrectCasterGuid_, std::dec);
    resurrectRequestPending_ = false;
}

void GameHandler::tabTarget(float playerX, float playerY, float playerZ) {
    // Helper: returns true if the entity is a living hostile that can be tab-targeted.
    auto isValidTabTarget = [&](const std::shared_ptr<Entity>& e) -> bool {
        if (!e) return false;
        const uint64_t guid = e->getGuid();
        auto* unit = dynamic_cast<Unit*>(e.get());
        if (!unit) return false;           // Not a unit (shouldn't happen after type filter)
        if (unit->getHealth() == 0) return false;  // Dead / corpse
        const bool hostileByFaction = unit->isHostile();
        const bool hostileByCombat = isAggressiveTowardPlayer(guid);
        if (!hostileByFaction && !hostileByCombat) return false;
        return true;
    };

    // Rebuild cycle list if stale (entity added/removed since last tab press).
    if (tabCycleStale) {
        tabCycleList.clear();
        tabCycleIndex = -1;

        struct EntityDist { uint64_t guid; float distance; };
        std::vector<EntityDist> sortable;

        for (const auto& [guid, entity] : entityManager.getEntities()) {
            auto t = entity->getType();
            if (t != ObjectType::UNIT && t != ObjectType::PLAYER) continue;
            if (guid == playerGuid) continue;
            if (!isValidTabTarget(entity)) continue;  // Skip dead / non-hostile
            float dx = entity->getX() - playerX;
            float dy = entity->getY() - playerY;
            float dz = entity->getZ() - playerZ;
            sortable.push_back({guid, std::sqrt(dx*dx + dy*dy + dz*dz)});
        }

        std::sort(sortable.begin(), sortable.end(),
                  [](const EntityDist& a, const EntityDist& b) { return a.distance < b.distance; });

        for (const auto& ed : sortable) {
            tabCycleList.push_back(ed.guid);
        }
        tabCycleStale = false;
    }

    if (tabCycleList.empty()) {
        clearTarget();
        return;
    }

    // Advance through the cycle, skipping any entry that has since died or
    // turned friendly (e.g. NPC killed between two tab presses).
    int tries = static_cast<int>(tabCycleList.size());
    while (tries-- > 0) {
        tabCycleIndex = (tabCycleIndex + 1) % static_cast<int>(tabCycleList.size());
        uint64_t guid = tabCycleList[tabCycleIndex];
        auto entity = entityManager.getEntity(guid);
        if (isValidTabTarget(entity)) {
            setTarget(guid);
            return;
        }
    }

    // All cached entries are stale — clear target and force a fresh rebuild next time.
    tabCycleStale = true;
    clearTarget();
}

void GameHandler::addLocalChatMessage(const MessageChatData& msg) {
    chatHistory.push_back(msg);
    if (chatHistory.size() > maxChatHistory) {
        chatHistory.pop_front();
    }
}

// ============================================================
// Phase 1: Name Queries
// ============================================================

void GameHandler::queryPlayerName(uint64_t guid) {
    if (playerNameCache.count(guid) || pendingNameQueries.count(guid)) return;
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_INFO("queryPlayerName: skipped guid=0x", std::hex, guid, std::dec,
                 " state=", worldStateName(state), " socket=", (socket ? "yes" : "no"));
        return;
    }

    LOG_INFO("queryPlayerName: sending CMSG_NAME_QUERY for guid=0x", std::hex, guid, std::dec);
    pendingNameQueries.insert(guid);
    auto packet = NameQueryPacket::build(guid);
    socket->send(packet);
}

void GameHandler::queryCreatureInfo(uint32_t entry, uint64_t guid) {
    if (creatureInfoCache.count(entry) || pendingCreatureQueries.count(entry)) return;
    if (state != WorldState::IN_WORLD || !socket) return;

    pendingCreatureQueries.insert(entry);
    auto packet = CreatureQueryPacket::build(entry, guid);
    socket->send(packet);
}

void GameHandler::queryGameObjectInfo(uint32_t entry, uint64_t guid) {
    if (gameObjectInfoCache_.count(entry) || pendingGameObjectQueries_.count(entry)) return;
    if (state != WorldState::IN_WORLD || !socket) return;

    pendingGameObjectQueries_.insert(entry);
    auto packet = GameObjectQueryPacket::build(entry, guid);
    socket->send(packet);
}

std::string GameHandler::getCachedPlayerName(uint64_t guid) const {
    auto it = playerNameCache.find(guid);
    return (it != playerNameCache.end()) ? it->second : "";
}

std::string GameHandler::getCachedCreatureName(uint32_t entry) const {
    auto it = creatureInfoCache.find(entry);
    return (it != creatureInfoCache.end()) ? it->second.name : "";
}

void GameHandler::handleNameQueryResponse(network::Packet& packet) {
    NameQueryResponseData data;
    if (!packetParsers_ || !packetParsers_->parseNameQueryResponse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_NAME_QUERY_RESPONSE (size=", packet.getSize(), ")");
        return;
    }

    pendingNameQueries.erase(data.guid);

    LOG_INFO("Name query response: guid=0x", std::hex, data.guid, std::dec,
             " found=", (int)data.found, " name='", data.name, "'",
             " race=", (int)data.race, " class=", (int)data.classId);

    if (data.isValid()) {
        playerNameCache[data.guid] = data.name;
        // Update entity name
        auto entity = entityManager.getEntity(data.guid);
        if (entity && entity->getType() == ObjectType::PLAYER) {
            auto player = std::static_pointer_cast<Player>(entity);
            player->setName(data.name);
        }

        // Backfill chat history entries that arrived before we knew the name.
        for (auto& msg : chatHistory) {
            if (msg.senderGuid == data.guid && msg.senderName.empty()) {
                msg.senderName = data.name;
            }
        }

        // Backfill mail inbox sender names
        for (auto& mail : mailInbox_) {
            if (mail.messageType == 0 && mail.senderGuid == data.guid) {
                mail.senderName = data.name;
            }
        }
    }
}

void GameHandler::handleCreatureQueryResponse(network::Packet& packet) {
    CreatureQueryResponseData data;
    if (!CreatureQueryResponseParser::parse(packet, data)) return;

    pendingCreatureQueries.erase(data.entry);

    if (data.isValid()) {
        creatureInfoCache[data.entry] = data;
        // Update all unit entities with this entry
        for (auto& [guid, entity] : entityManager.getEntities()) {
            if (entity->getType() == ObjectType::UNIT) {
                auto unit = std::static_pointer_cast<Unit>(entity);
                if (unit->getEntry() == data.entry) {
                    unit->setName(data.name);
                }
            }
        }
    }
}

// ============================================================
// GameObject Query
// ============================================================

void GameHandler::handleGameObjectQueryResponse(network::Packet& packet) {
    GameObjectQueryResponseData data;
    bool ok = packetParsers_ ? packetParsers_->parseGameObjectQueryResponse(packet, data)
                             : GameObjectQueryResponseParser::parse(packet, data);
    if (!ok) return;

    pendingGameObjectQueries_.erase(data.entry);

    if (data.isValid()) {
        gameObjectInfoCache_[data.entry] = data;
        // Update all gameobject entities with this entry
        for (auto& [guid, entity] : entityManager.getEntities()) {
            if (entity->getType() == ObjectType::GAMEOBJECT) {
                auto go = std::static_pointer_cast<GameObject>(entity);
                if (go->getEntry() == data.entry) {
                    go->setName(data.name);
                }
            }
        }

        // MO_TRANSPORT (type 15): assign TaxiPathNode path if available
        if (data.type == 15 && data.hasData && data.data[0] != 0 && transportManager_) {
            uint32_t taxiPathId = data.data[0];
            if (transportManager_->hasTaxiPath(taxiPathId)) {
                if (transportManager_->assignTaxiPathToTransport(data.entry, taxiPathId)) {
                    LOG_DEBUG("MO_TRANSPORT entry=", data.entry, " assigned TaxiPathNode path ", taxiPathId);
                }
            } else {
                LOG_DEBUG("MO_TRANSPORT entry=", data.entry, " taxiPathId=", taxiPathId,
                         " not found in TaxiPathNode.dbc");
            }
        }
    }
}

void GameHandler::handleGameObjectPageText(network::Packet& packet) {
    if (packet.getSize() - packet.getReadPos() < 8) return;
    uint64_t guid = packet.readUInt64();
    auto entity = entityManager.getEntity(guid);
    if (!entity || entity->getType() != ObjectType::GAMEOBJECT) return;

    auto go = std::static_pointer_cast<GameObject>(entity);
    uint32_t entry = go->getEntry();
    if (entry == 0) return;

    auto cacheIt = gameObjectInfoCache_.find(entry);
    if (cacheIt == gameObjectInfoCache_.end()) {
        queryGameObjectInfo(entry, guid);
        return;
    }

    const GameObjectQueryResponseData& info = cacheIt->second;
    uint32_t pageId = 0;
    // AzerothCore layout:
    // type 9 (TEXT): data[0]=pageID
    // type 10 (GOOBER): data[7]=pageId
    if (info.type == 9) pageId = info.data[0];
    else if (info.type == 10) pageId = info.data[7];

    if (pageId != 0 && socket && state == WorldState::IN_WORLD) {
        auto req = PageTextQueryPacket::build(pageId, guid);
        socket->send(req);
        return;
    }

    if (!info.name.empty()) {
        addSystemChatMessage(info.name);
    }
}

void GameHandler::handlePageTextQueryResponse(network::Packet& packet) {
    PageTextQueryResponseData data;
    if (!PageTextQueryResponseParser::parse(packet, data)) return;

    if (!data.text.empty()) {
        std::istringstream iss(data.text);
        std::string line;
        bool wrote = false;
        while (std::getline(iss, line)) {
            if (line.empty()) continue;
            addSystemChatMessage(line);
            wrote = true;
        }
        if (!wrote) {
            addSystemChatMessage(data.text);
        }
    }
}

// ============================================================
// Item Query
// ============================================================

void GameHandler::queryItemInfo(uint32_t entry, uint64_t guid) {
    if (itemInfoCache_.count(entry) || pendingItemQueries_.count(entry)) return;
    if (state != WorldState::IN_WORLD || !socket) return;

    pendingItemQueries_.insert(entry);
    // Some cores reject CMSG_ITEM_QUERY_SINGLE when the GUID is 0.
    // If we don't have the item object's GUID (e.g. visible equipment decoding),
    // fall back to the player's GUID to keep the request non-zero.
    uint64_t queryGuid = (guid != 0) ? guid : playerGuid;
    auto packet = packetParsers_
        ? packetParsers_->buildItemQuery(entry, queryGuid)
        : ItemQueryPacket::build(entry, queryGuid);
    socket->send(packet);
    LOG_INFO("queryItemInfo: entry=", entry, " guid=0x", std::hex, queryGuid, std::dec,
             " pending=", pendingItemQueries_.size());
}

void GameHandler::handleItemQueryResponse(network::Packet& packet) {
    ItemQueryResponseData data;
    bool parsed = packetParsers_
        ? packetParsers_->parseItemQueryResponse(packet, data)
        : ItemQueryResponseParser::parse(packet, data);
    if (!parsed) {
        LOG_WARNING("handleItemQueryResponse: parse failed, size=", packet.getSize());
        return;
    }

    pendingItemQueries_.erase(data.entry);
    LOG_INFO("handleItemQueryResponse: entry=", data.entry, " valid=", data.valid,
             " name='", data.name, "' displayInfoId=", data.displayInfoId,
             " pending=", pendingItemQueries_.size());

    if (data.valid) {
        itemInfoCache_[data.entry] = data;
        rebuildOnlineInventory();
        maybeDetectVisibleItemLayout();

        // Selectively re-emit only players whose equipment references this item entry
        const uint32_t resolvedEntry = data.entry;
        for (const auto& [guid, entries] : otherPlayerVisibleItemEntries_) {
            for (uint32_t e : entries) {
                if (e == resolvedEntry) {
                    emitOtherPlayerEquipment(guid);
                    break;
                }
            }
        }
        // Same for inspect-based entries
        if (playerEquipmentCallback_) {
            for (const auto& [guid, entries] : inspectedPlayerItemEntries_) {
                bool relevant = false;
                for (uint32_t e : entries) {
                    if (e == resolvedEntry) { relevant = true; break; }
                }
                if (!relevant) continue;
                std::array<uint32_t, 19> displayIds{};
                std::array<uint8_t, 19> invTypes{};
                for (int s = 0; s < 19; s++) {
                    uint32_t entry = entries[s];
                    if (entry == 0) continue;
                    auto infoIt = itemInfoCache_.find(entry);
                    if (infoIt == itemInfoCache_.end()) continue;
                    displayIds[s] = infoIt->second.displayInfoId;
                    invTypes[s] = static_cast<uint8_t>(infoIt->second.inventoryType);
                }
                playerEquipmentCallback_(guid, displayIds, invTypes);
            }
        }
    }
}

void GameHandler::handleInspectResults(network::Packet& packet) {
    // SMSG_TALENTS_INFO (0x3F4) format:
    // uint8 talentType: 0 = own talents (sent on login/respec), 1 = inspect result
    // If type==1: PackedGUID of inspected player
    // Then: uint32 unspentTalents, uint8 talentGroupCount, uint8 activeTalentGroup
    // Per talent group: uint8 talentCount, [talentId(u32) + rank(u8)]..., uint8 glyphCount, [glyphId(u16)]...
    if (packet.getSize() - packet.getReadPos() < 1) return;

    uint8_t talentType = packet.readUInt8();

    if (talentType == 0) {
        // Own talent info — silently consume (sent on login, talent changes, respecs)
        LOG_DEBUG("SMSG_TALENTS_INFO: received own talent data, ignoring");
        return;
    }

    // talentType == 1: inspect result
    if (packet.getSize() - packet.getReadPos() < 2) return;

    uint64_t guid = UpdateObjectParser::readPackedGuid(packet);
    if (guid == 0) return;

    size_t bytesLeft = packet.getSize() - packet.getReadPos();
    if (bytesLeft < 6) {
        LOG_WARNING("SMSG_TALENTS_INFO: too short after guid, ", bytesLeft, " bytes");
        auto entity = entityManager.getEntity(guid);
        std::string name = "Target";
        if (entity) {
            auto player = std::dynamic_pointer_cast<Player>(entity);
            if (player && !player->getName().empty()) name = player->getName();
        }
        addSystemChatMessage("Inspecting " + name + " (no talent data available).");
        return;
    }

    uint32_t unspentTalents = packet.readUInt32();
    uint8_t talentGroupCount = packet.readUInt8();
    uint8_t activeTalentGroup = packet.readUInt8();

    // Resolve player name
    auto entity = entityManager.getEntity(guid);
    std::string playerName = "Target";
    if (entity) {
        auto player = std::dynamic_pointer_cast<Player>(entity);
        if (player && !player->getName().empty()) playerName = player->getName();
    }

    // Parse talent groups
    uint32_t totalTalents = 0;
    for (uint8_t g = 0; g < talentGroupCount && g < 2; ++g) {
        bytesLeft = packet.getSize() - packet.getReadPos();
        if (bytesLeft < 1) break;

        uint8_t talentCount = packet.readUInt8();
        for (uint8_t t = 0; t < talentCount; ++t) {
            bytesLeft = packet.getSize() - packet.getReadPos();
            if (bytesLeft < 5) break;
            packet.readUInt32(); // talentId
            packet.readUInt8();  // rank
            totalTalents++;
        }

        bytesLeft = packet.getSize() - packet.getReadPos();
        if (bytesLeft < 1) break;
        uint8_t glyphCount = packet.readUInt8();
        for (uint8_t gl = 0; gl < glyphCount; ++gl) {
            bytesLeft = packet.getSize() - packet.getReadPos();
            if (bytesLeft < 2) break;
            packet.readUInt16(); // glyphId
        }
    }

    // Parse enchantment slot mask + enchant IDs
    bytesLeft = packet.getSize() - packet.getReadPos();
    if (bytesLeft >= 4) {
        uint32_t slotMask = packet.readUInt32();
        for (int slot = 0; slot < 19; ++slot) {
            if (slotMask & (1u << slot)) {
                bytesLeft = packet.getSize() - packet.getReadPos();
                if (bytesLeft < 2) break;
                packet.readUInt16(); // enchantId
            }
        }
    }

    // Display inspect results
    std::string msg = "Inspect: " + playerName;
    msg += " - " + std::to_string(totalTalents) + " talent points spent";
    if (unspentTalents > 0) {
        msg += ", " + std::to_string(unspentTalents) + " unspent";
    }
    if (talentGroupCount > 1) {
        msg += " (dual spec, active: " + std::to_string(activeTalentGroup + 1) + ")";
    }
    addSystemChatMessage(msg);

    LOG_INFO("Inspect results for ", playerName, ": ", totalTalents, " talents, ",
             unspentTalents, " unspent, ", (int)talentGroupCount, " specs");
}

uint64_t GameHandler::resolveOnlineItemGuid(uint32_t itemId) const {
    if (itemId == 0) return 0;
    uint64_t found = 0;
    for (const auto& [guid, info] : onlineItems_) {
        if (info.entry != itemId) continue;
        if (found != 0) {
            return 0; // Ambiguous
        }
        found = guid;
    }
    return found;
}

void GameHandler::detectInventorySlotBases(const std::map<uint16_t, uint32_t>& fields) {
    if (invSlotBase_ >= 0 && packSlotBase_ >= 0) return;
    if (fields.empty()) return;

    std::vector<uint16_t> matchingPairs;
    matchingPairs.reserve(32);

    for (const auto& [idx, low] : fields) {
        if ((idx % 2) != 0) continue;
        auto itHigh = fields.find(static_cast<uint16_t>(idx + 1));
        if (itHigh == fields.end()) continue;
        uint64_t guid = (uint64_t(itHigh->second) << 32) | low;
        if (guid == 0) continue;
        // Primary signal: GUID pairs that match spawned ITEM objects.
        if (!onlineItems_.empty() && onlineItems_.count(guid)) {
            matchingPairs.push_back(idx);
        }
    }

    // Fallback signal (when ITEM objects haven't been seen yet):
    // collect any plausible non-zero GUID pairs and derive a base by density.
    if (matchingPairs.empty()) {
        for (const auto& [idx, low] : fields) {
            if ((idx % 2) != 0) continue;
            auto itHigh = fields.find(static_cast<uint16_t>(idx + 1));
            if (itHigh == fields.end()) continue;
            uint64_t guid = (uint64_t(itHigh->second) << 32) | low;
            if (guid == 0) continue;
            // Heuristic: item GUIDs tend to be non-trivial and change often; ignore tiny values.
            if (guid < 0x10000ull) continue;
            matchingPairs.push_back(idx);
        }
    }

    if (matchingPairs.empty()) return;
    std::sort(matchingPairs.begin(), matchingPairs.end());

    if (invSlotBase_ < 0) {
        // The lowest matching field is the first EQUIPPED slot (not necessarily HEAD).
        // With 2+ matches we can derive the true base: all matches must be at
        // even offsets from the base, spaced 2 fields per slot.
        const int knownBase = static_cast<int>(fieldIndex(UF::PLAYER_FIELD_INV_SLOT_HEAD));
        constexpr int slotStride = 2;
        bool allAlign = true;
        for (uint16_t p : matchingPairs) {
            if (p < knownBase || (p - knownBase) % slotStride != 0) {
                allAlign = false;
                break;
            }
        }
        if (allAlign) {
            invSlotBase_ = knownBase;
        } else {
            // Fallback: if we have 2+ matches, derive base from their spacing
            if (matchingPairs.size() >= 2) {
                uint16_t lo = matchingPairs[0];
                // lo must be base + 2*slotN, and slotN is 0..22
                // Try each possible slot for 'lo' and see if all others also land on valid slots
                for (int s = 0; s <= 22; s++) {
                    int candidate = lo - s * slotStride;
                    if (candidate < 0) break;
                    bool ok = true;
                    for (uint16_t p : matchingPairs) {
                        int off = p - candidate;
                        if (off < 0 || off % slotStride != 0 || off / slotStride > 22) {
                            ok = false;
                            break;
                        }
                    }
                    if (ok) {
                        invSlotBase_ = candidate;
                        break;
                    }
                }
                if (invSlotBase_ < 0) invSlotBase_ = knownBase;
            } else {
                invSlotBase_ = knownBase;
            }
        }
        packSlotBase_ = invSlotBase_ + (game::Inventory::NUM_EQUIP_SLOTS * 2);
        LOG_INFO("Detected inventory field base: equip=", invSlotBase_,
                 " pack=", packSlotBase_);
    }
}

bool GameHandler::applyInventoryFields(const std::map<uint16_t, uint32_t>& fields) {
    bool slotsChanged = false;
    int equipBase = (invSlotBase_ >= 0) ? invSlotBase_ : static_cast<int>(fieldIndex(UF::PLAYER_FIELD_INV_SLOT_HEAD));
    int packBase = (packSlotBase_ >= 0) ? packSlotBase_ : static_cast<int>(fieldIndex(UF::PLAYER_FIELD_PACK_SLOT_1));

    for (const auto& [key, val] : fields) {
        if (key >= equipBase && key <= equipBase + (game::Inventory::NUM_EQUIP_SLOTS * 2 - 1)) {
            int slotIndex = (key - equipBase) / 2;
            bool isLow = ((key - equipBase) % 2 == 0);
            if (slotIndex < static_cast<int>(equipSlotGuids_.size())) {
                uint64_t& guid = equipSlotGuids_[slotIndex];
                if (isLow) guid = (guid & 0xFFFFFFFF00000000ULL) | val;
                else guid = (guid & 0x00000000FFFFFFFFULL) | (uint64_t(val) << 32);
                slotsChanged = true;
            }
        } else if (key >= packBase && key <= packBase + (game::Inventory::BACKPACK_SLOTS * 2 - 1)) {
            int slotIndex = (key - packBase) / 2;
            bool isLow = ((key - packBase) % 2 == 0);
            if (slotIndex < static_cast<int>(backpackSlotGuids_.size())) {
                uint64_t& guid = backpackSlotGuids_[slotIndex];
                if (isLow) guid = (guid & 0xFFFFFFFF00000000ULL) | val;
                else guid = (guid & 0x00000000FFFFFFFFULL) | (uint64_t(val) << 32);
                slotsChanged = true;
            }
        }

        // Bank slots: 28 slots × 2 fields = 56 fields starting at PLAYER_FIELD_BANK_SLOT_1
        int bankBase = static_cast<int>(fieldIndex(UF::PLAYER_FIELD_BANK_SLOT_1));
        if (bankBase != 0xFFFF && key >= static_cast<uint16_t>(bankBase) &&
            key <= static_cast<uint16_t>(bankBase) + (game::Inventory::BANK_SLOTS * 2 - 1)) {
            int slotIndex = (key - bankBase) / 2;
            bool isLow = ((key - bankBase) % 2 == 0);
            if (slotIndex < static_cast<int>(bankSlotGuids_.size())) {
                uint64_t& guid = bankSlotGuids_[slotIndex];
                if (isLow) guid = (guid & 0xFFFFFFFF00000000ULL) | val;
                else guid = (guid & 0x00000000FFFFFFFFULL) | (uint64_t(val) << 32);
                slotsChanged = true;
            }
        }

        // Bank bag slots: 7 slots × 2 fields = 14 fields starting at PLAYER_FIELD_BANKBAG_SLOT_1
        int bankBagBase = static_cast<int>(fieldIndex(UF::PLAYER_FIELD_BANKBAG_SLOT_1));
        if (bankBagBase != 0xFFFF && key >= static_cast<uint16_t>(bankBagBase) &&
            key <= static_cast<uint16_t>(bankBagBase) + (game::Inventory::BANK_BAG_SLOTS * 2 - 1)) {
            int slotIndex = (key - bankBagBase) / 2;
            bool isLow = ((key - bankBagBase) % 2 == 0);
            if (slotIndex < static_cast<int>(bankBagSlotGuids_.size())) {
                uint64_t& guid = bankBagSlotGuids_[slotIndex];
                if (isLow) guid = (guid & 0xFFFFFFFF00000000ULL) | val;
                else guid = (guid & 0x00000000FFFFFFFFULL) | (uint64_t(val) << 32);
                slotsChanged = true;
            }
        }
    }

    return slotsChanged;
}

void GameHandler::extractContainerFields(uint64_t containerGuid, const std::map<uint16_t, uint32_t>& fields) {
    const uint16_t numSlotsIdx = fieldIndex(UF::CONTAINER_FIELD_NUM_SLOTS);
    const uint16_t slot1Idx = fieldIndex(UF::CONTAINER_FIELD_SLOT_1);
    if (numSlotsIdx == 0xFFFF || slot1Idx == 0xFFFF) return;

    auto& info = containerContents_[containerGuid];

    // Read number of slots
    auto numIt = fields.find(numSlotsIdx);
    if (numIt != fields.end()) {
        info.numSlots = std::min(numIt->second, 36u);
    }

    // Read slot GUIDs (each is 2 uint32 fields: lo + hi)
    for (const auto& [key, val] : fields) {
        if (key < slot1Idx) continue;
        int offset = key - slot1Idx;
        int slotIndex = offset / 2;
        if (slotIndex >= 36) continue;
        bool isLow = (offset % 2 == 0);
        uint64_t& guid = info.slotGuids[slotIndex];
        if (isLow) guid = (guid & 0xFFFFFFFF00000000ULL) | val;
        else guid = (guid & 0x00000000FFFFFFFFULL) | (uint64_t(val) << 32);
    }
}

void GameHandler::rebuildOnlineInventory() {

    inventory = Inventory();

    // Equipment slots
    for (int i = 0; i < 23; i++) {
        uint64_t guid = equipSlotGuids_[i];
        if (guid == 0) continue;

        auto itemIt = onlineItems_.find(guid);
        if (itemIt == onlineItems_.end()) continue;

        ItemDef def;
        def.itemId = itemIt->second.entry;
        def.stackCount = itemIt->second.stackCount;
        def.maxStack = 1;

        auto infoIt = itemInfoCache_.find(itemIt->second.entry);
        if (infoIt != itemInfoCache_.end()) {
            def.name = infoIt->second.name;
            def.quality = static_cast<ItemQuality>(infoIt->second.quality);
            def.inventoryType = infoIt->second.inventoryType;
            def.maxStack = std::max(1, infoIt->second.maxStack);
            def.displayInfoId = infoIt->second.displayInfoId;
            def.subclassName = infoIt->second.subclassName;
            def.damageMin = infoIt->second.damageMin;
            def.damageMax = infoIt->second.damageMax;
            def.delayMs = infoIt->second.delayMs;
            def.armor = infoIt->second.armor;
            def.stamina = infoIt->second.stamina;
            def.strength = infoIt->second.strength;
            def.agility = infoIt->second.agility;
            def.intellect = infoIt->second.intellect;
            def.spirit = infoIt->second.spirit;
        } else {
            def.name = "Item " + std::to_string(def.itemId);
            queryItemInfo(def.itemId, guid);
        }

        inventory.setEquipSlot(static_cast<EquipSlot>(i), def);
    }

    // Backpack slots
    for (int i = 0; i < 16; i++) {
        uint64_t guid = backpackSlotGuids_[i];
        if (guid == 0) continue;

        auto itemIt = onlineItems_.find(guid);
        if (itemIt == onlineItems_.end()) continue;

        ItemDef def;
        def.itemId = itemIt->second.entry;
        def.stackCount = itemIt->second.stackCount;
        def.maxStack = 1;

        auto infoIt = itemInfoCache_.find(itemIt->second.entry);
        if (infoIt != itemInfoCache_.end()) {
            def.name = infoIt->second.name;
            def.quality = static_cast<ItemQuality>(infoIt->second.quality);
            def.inventoryType = infoIt->second.inventoryType;
            def.maxStack = std::max(1, infoIt->second.maxStack);
            def.displayInfoId = infoIt->second.displayInfoId;
            def.subclassName = infoIt->second.subclassName;
            def.damageMin = infoIt->second.damageMin;
            def.damageMax = infoIt->second.damageMax;
            def.delayMs = infoIt->second.delayMs;
            def.armor = infoIt->second.armor;
            def.stamina = infoIt->second.stamina;
            def.strength = infoIt->second.strength;
            def.agility = infoIt->second.agility;
            def.intellect = infoIt->second.intellect;
            def.spirit = infoIt->second.spirit;
        } else {
            def.name = "Item " + std::to_string(def.itemId);
            queryItemInfo(def.itemId, guid);
        }

        inventory.setBackpackSlot(i, def);
    }

    // Bag contents (BAG1-BAG4 are equip slots 19-22)
    for (int bagIdx = 0; bagIdx < 4; bagIdx++) {
        uint64_t bagGuid = equipSlotGuids_[19 + bagIdx];
        if (bagGuid == 0) continue;

        // Determine bag size from container fields or item template
        int numSlots = 0;
        auto contIt = containerContents_.find(bagGuid);
        if (contIt != containerContents_.end()) {
            numSlots = static_cast<int>(contIt->second.numSlots);
        }
        if (numSlots <= 0) {
            auto bagItemIt = onlineItems_.find(bagGuid);
            if (bagItemIt != onlineItems_.end()) {
                auto bagInfoIt = itemInfoCache_.find(bagItemIt->second.entry);
                if (bagInfoIt != itemInfoCache_.end()) {
                    numSlots = bagInfoIt->second.containerSlots;
                }
            }
        }
        if (numSlots <= 0) continue;

        // Set the bag size in the inventory bag data
        inventory.setBagSize(bagIdx, numSlots);

        // Also set bagSlots on the equipped bag item (for UI display)
        auto& bagEquipSlot = inventory.getEquipSlot(static_cast<EquipSlot>(19 + bagIdx));
        if (!bagEquipSlot.empty()) {
            ItemDef bagDef = bagEquipSlot.item;
            bagDef.bagSlots = numSlots;
            inventory.setEquipSlot(static_cast<EquipSlot>(19 + bagIdx), bagDef);
        }

        // Populate bag slot items
        if (contIt == containerContents_.end()) continue;
        const auto& container = contIt->second;
        for (int s = 0; s < numSlots && s < 36; s++) {
            uint64_t itemGuid = container.slotGuids[s];
            if (itemGuid == 0) continue;

            auto itemIt = onlineItems_.find(itemGuid);
            if (itemIt == onlineItems_.end()) continue;

            ItemDef def;
            def.itemId = itemIt->second.entry;
            def.stackCount = itemIt->second.stackCount;
            def.maxStack = 1;

            auto infoIt = itemInfoCache_.find(itemIt->second.entry);
            if (infoIt != itemInfoCache_.end()) {
                def.name = infoIt->second.name;
                def.quality = static_cast<ItemQuality>(infoIt->second.quality);
                def.inventoryType = infoIt->second.inventoryType;
                def.maxStack = std::max(1, infoIt->second.maxStack);
                def.displayInfoId = infoIt->second.displayInfoId;
                def.subclassName = infoIt->second.subclassName;
                def.damageMin = infoIt->second.damageMin;
                def.damageMax = infoIt->second.damageMax;
                def.delayMs = infoIt->second.delayMs;
                def.armor = infoIt->second.armor;
                def.stamina = infoIt->second.stamina;
                def.strength = infoIt->second.strength;
                def.agility = infoIt->second.agility;
                def.intellect = infoIt->second.intellect;
                def.spirit = infoIt->second.spirit;
                def.bagSlots = infoIt->second.containerSlots;
            } else {
                def.name = "Item " + std::to_string(def.itemId);
                queryItemInfo(def.itemId, itemGuid);
            }

            inventory.setBagSlot(bagIdx, s, def);
        }
    }

    // Bank slots (28 main slots)
    for (int i = 0; i < 28; i++) {
        uint64_t guid = bankSlotGuids_[i];
        if (guid == 0) { inventory.clearBankSlot(i); continue; }

        auto itemIt = onlineItems_.find(guid);
        if (itemIt == onlineItems_.end()) continue;

        ItemDef def;
        def.itemId = itemIt->second.entry;
        def.stackCount = itemIt->second.stackCount;
        def.maxStack = 1;

        auto infoIt = itemInfoCache_.find(itemIt->second.entry);
        if (infoIt != itemInfoCache_.end()) {
            def.name = infoIt->second.name;
            def.quality = static_cast<ItemQuality>(infoIt->second.quality);
            def.inventoryType = infoIt->second.inventoryType;
            def.maxStack = std::max(1, infoIt->second.maxStack);
            def.displayInfoId = infoIt->second.displayInfoId;
            def.subclassName = infoIt->second.subclassName;
            def.damageMin = infoIt->second.damageMin;
            def.damageMax = infoIt->second.damageMax;
            def.delayMs = infoIt->second.delayMs;
            def.armor = infoIt->second.armor;
            def.stamina = infoIt->second.stamina;
            def.strength = infoIt->second.strength;
            def.agility = infoIt->second.agility;
            def.intellect = infoIt->second.intellect;
            def.spirit = infoIt->second.spirit;
            def.sellPrice = infoIt->second.sellPrice;
            def.bagSlots = infoIt->second.containerSlots;
        } else {
            def.name = "Item " + std::to_string(def.itemId);
            queryItemInfo(def.itemId, guid);
        }

        inventory.setBankSlot(i, def);
    }

    // Bank bag contents (7 bank bag slots)
    for (int bagIdx = 0; bagIdx < 7; bagIdx++) {
        uint64_t bagGuid = bankBagSlotGuids_[bagIdx];
        if (bagGuid == 0) { inventory.setBankBagSize(bagIdx, 0); continue; }

        int numSlots = 0;
        auto contIt = containerContents_.find(bagGuid);
        if (contIt != containerContents_.end()) {
            numSlots = static_cast<int>(contIt->second.numSlots);
        }
        if (numSlots <= 0) {
            auto bagItemIt = onlineItems_.find(bagGuid);
            if (bagItemIt != onlineItems_.end()) {
                auto bagInfoIt = itemInfoCache_.find(bagItemIt->second.entry);
                if (bagInfoIt != itemInfoCache_.end()) {
                    numSlots = bagInfoIt->second.containerSlots;
                }
            }
        }
        if (numSlots <= 0) continue;

        inventory.setBankBagSize(bagIdx, numSlots);

        if (contIt == containerContents_.end()) continue;
        const auto& container = contIt->second;
        for (int s = 0; s < numSlots && s < 36; s++) {
            uint64_t itemGuid = container.slotGuids[s];
            if (itemGuid == 0) continue;

            auto itemIt = onlineItems_.find(itemGuid);
            if (itemIt == onlineItems_.end()) continue;

            ItemDef def;
            def.itemId = itemIt->second.entry;
            def.stackCount = itemIt->second.stackCount;
            def.maxStack = 1;

            auto infoIt = itemInfoCache_.find(itemIt->second.entry);
            if (infoIt != itemInfoCache_.end()) {
                def.name = infoIt->second.name;
                def.quality = static_cast<ItemQuality>(infoIt->second.quality);
                def.inventoryType = infoIt->second.inventoryType;
                def.maxStack = std::max(1, infoIt->second.maxStack);
                def.displayInfoId = infoIt->second.displayInfoId;
                def.subclassName = infoIt->second.subclassName;
                def.damageMin = infoIt->second.damageMin;
                def.damageMax = infoIt->second.damageMax;
                def.delayMs = infoIt->second.delayMs;
                def.armor = infoIt->second.armor;
                def.stamina = infoIt->second.stamina;
                def.strength = infoIt->second.strength;
                def.agility = infoIt->second.agility;
                def.intellect = infoIt->second.intellect;
                def.spirit = infoIt->second.spirit;
                def.sellPrice = infoIt->second.sellPrice;
                def.bagSlots = infoIt->second.containerSlots;
            } else {
                def.name = "Item " + std::to_string(def.itemId);
                queryItemInfo(def.itemId, itemGuid);
            }

            inventory.setBankBagSlot(bagIdx, s, def);
        }
    }

    // Only mark equipment dirty if equipped item displayInfoIds actually changed
    std::array<uint32_t, 19> currentEquipDisplayIds{};
    for (int i = 0; i < 19; i++) {
        const auto& slot = inventory.getEquipSlot(static_cast<EquipSlot>(i));
        if (!slot.empty()) currentEquipDisplayIds[i] = slot.item.displayInfoId;
    }
    if (currentEquipDisplayIds != lastEquipDisplayIds_) {
        lastEquipDisplayIds_ = currentEquipDisplayIds;
        onlineEquipDirty_ = true;
    }

    LOG_DEBUG("Rebuilt online inventory: equip=", [&](){
        int c = 0; for (auto g : equipSlotGuids_) if (g) c++; return c;
    }(), " backpack=", [&](){
        int c = 0; for (auto g : backpackSlotGuids_) if (g) c++; return c;
    }());
}

void GameHandler::maybeDetectVisibleItemLayout() {
    if (visibleItemLayoutVerified_) return;
    if (lastPlayerFields_.empty()) return;

    std::array<uint32_t, 19> equipEntries{};
    int nonZero = 0;
    // Prefer authoritative equipped item entry IDs derived from item objects (onlineItems_),
    // because Inventory::ItemDef may not be populated yet if templates haven't been queried.
    for (int i = 0; i < 19; i++) {
        uint64_t itemGuid = equipSlotGuids_[i];
        if (itemGuid != 0) {
            auto it = onlineItems_.find(itemGuid);
            if (it != onlineItems_.end() && it->second.entry != 0) {
                equipEntries[i] = it->second.entry;
            }
        }
        if (equipEntries[i] == 0) {
            const auto& slot = inventory.getEquipSlot(static_cast<EquipSlot>(i));
            equipEntries[i] = slot.empty() ? 0u : slot.item.itemId;
        }
        if (equipEntries[i] != 0) nonZero++;
    }
    if (nonZero < 2) return;

    const uint16_t maxKey = lastPlayerFields_.rbegin()->first;
    int bestBase = -1;
    int bestStride = 0;
    int bestMatches = 0;
    int bestMismatches = 9999;
    int bestScore = -999999;

    const int strides[] = {2, 3, 4, 1};
    for (int stride : strides) {
        for (const auto& [baseIdxU16, _v] : lastPlayerFields_) {
            const int base = static_cast<int>(baseIdxU16);
            if (base + 18 * stride > static_cast<int>(maxKey)) continue;

            int matches = 0;
            int mismatches = 0;
            for (int s = 0; s < 19; s++) {
                uint32_t want = equipEntries[s];
                if (want == 0) continue;
                const uint16_t idx = static_cast<uint16_t>(base + s * stride);
                auto it = lastPlayerFields_.find(idx);
                if (it == lastPlayerFields_.end()) continue;
                if (it->second == want) {
                    matches++;
                } else if (it->second != 0) {
                    mismatches++;
                }
            }

            int score = matches * 2 - mismatches * 3;
            if (score > bestScore ||
                (score == bestScore && matches > bestMatches) ||
                (score == bestScore && matches == bestMatches && mismatches < bestMismatches) ||
                (score == bestScore && matches == bestMatches && mismatches == bestMismatches && base < bestBase)) {
                bestScore = score;
                bestMatches = matches;
                bestMismatches = mismatches;
                bestBase = base;
                bestStride = stride;
            }
        }
    }

    if (bestMatches >= 2 && bestBase >= 0 && bestStride > 0 && bestMismatches <= 1) {
        visibleItemEntryBase_ = bestBase;
        visibleItemStride_ = bestStride;
        visibleItemLayoutVerified_ = true;
        LOG_INFO("Detected PLAYER_VISIBLE_ITEM entry layout: base=", visibleItemEntryBase_,
                 " stride=", visibleItemStride_, " (matches=", bestMatches,
                 " mismatches=", bestMismatches, " score=", bestScore, ")");

        // Backfill existing player entities already in view.
        for (const auto& [guid, ent] : entityManager.getEntities()) {
            if (!ent || ent->getType() != ObjectType::PLAYER) continue;
            if (guid == playerGuid) continue;
            updateOtherPlayerVisibleItems(guid, ent->getFields());
        }
    }
    // If heuristic didn't find a match, keep using the default WotLK layout (base=284, stride=2).
}

void GameHandler::updateOtherPlayerVisibleItems(uint64_t guid, const std::map<uint16_t, uint32_t>& fields) {
    if (guid == 0 || guid == playerGuid) return;
    if (visibleItemEntryBase_ < 0 || visibleItemStride_ <= 0) {
        // Layout not detected yet — queue this player for inspect as fallback.
        if (socket && state == WorldState::IN_WORLD) {
            pendingAutoInspect_.insert(guid);
            LOG_DEBUG("Queued player 0x", std::hex, guid, std::dec, " for auto-inspect (layout not detected)");
        }
        return;
    }

    std::array<uint32_t, 19> newEntries{};
    for (int s = 0; s < 19; s++) {
        uint16_t idx = static_cast<uint16_t>(visibleItemEntryBase_ + s * visibleItemStride_);
        auto it = fields.find(idx);
        if (it != fields.end()) newEntries[s] = it->second;
    }

    bool changed = false;
    auto& old = otherPlayerVisibleItemEntries_[guid];
    if (old != newEntries) {
        old = newEntries;
        changed = true;
    }

    // Request item templates for any new visible entries.
    for (uint32_t entry : newEntries) {
        if (entry == 0) continue;
        if (!itemInfoCache_.count(entry) && !pendingItemQueries_.count(entry)) {
            queryItemInfo(entry, 0);
        }
    }

    // If the server isn't sending visible item fields (all zeros), fall back to inspect.
    bool any = false;
    for (uint32_t e : newEntries) { if (e != 0) { any = true; break; } }
    if (!any && socket && state == WorldState::IN_WORLD) {
        pendingAutoInspect_.insert(guid);
    }

    if (changed) {
        otherPlayerVisibleDirty_.insert(guid);
        emitOtherPlayerEquipment(guid);
    }
}

void GameHandler::emitOtherPlayerEquipment(uint64_t guid) {
    if (!playerEquipmentCallback_) return;
    auto it = otherPlayerVisibleItemEntries_.find(guid);
    if (it == otherPlayerVisibleItemEntries_.end()) return;

    std::array<uint32_t, 19> displayIds{};
    std::array<uint8_t, 19> invTypes{};
    bool anyEntry = false;

    for (int s = 0; s < 19; s++) {
        uint32_t entry = it->second[s];
        if (entry == 0) continue;
        anyEntry = true;
        auto infoIt = itemInfoCache_.find(entry);
        if (infoIt == itemInfoCache_.end()) continue;
        displayIds[s] = infoIt->second.displayInfoId;
        invTypes[s] = static_cast<uint8_t>(infoIt->second.inventoryType);
    }

    playerEquipmentCallback_(guid, displayIds, invTypes);
    otherPlayerVisibleDirty_.erase(guid);

    // If we had entries but couldn't resolve any templates, also try inspect as a fallback.
    bool anyResolved = false;
    for (uint32_t did : displayIds) { if (did != 0) { anyResolved = true; break; } }
    if (anyEntry && !anyResolved) {
        pendingAutoInspect_.insert(guid);
    }
}

void GameHandler::emitAllOtherPlayerEquipment() {
    if (!playerEquipmentCallback_) return;
    for (const auto& [guid, _] : otherPlayerVisibleItemEntries_) {
        emitOtherPlayerEquipment(guid);
    }
}

// ============================================================
// Phase 2: Combat
// ============================================================

void GameHandler::startAutoAttack(uint64_t targetGuid) {
    // Can't attack yourself
    if (targetGuid == playerGuid) return;
    if (targetGuid == 0) return;

    // Dismount when entering combat
    if (isMounted()) {
        dismount();
    }

    // Client-side melee range gate to avoid starting "swing forever" loops when
    // target is already clearly out of range.
    if (auto target = entityManager.getEntity(targetGuid)) {
        float dx = movementInfo.x - target->getLatestX();
        float dy = movementInfo.y - target->getLatestY();
        float dz = movementInfo.z - target->getLatestZ();
        float dist3d = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist3d > 8.0f) {
            if (autoAttackRangeWarnCooldown_ <= 0.0f) {
                addSystemChatMessage("Target is too far away.");
                autoAttackRangeWarnCooldown_ = 1.25f;
            }
            return;
        }
    }

    autoAttackRequested_ = true;
    // Keep combat animation/state server-authoritative. We only flip autoAttacking
    // on SMSG_ATTACKSTART where attackerGuid == playerGuid.
    autoAttacking = false;
    autoAttackTarget = targetGuid;
    autoAttackOutOfRange_ = false;
    autoAttackOutOfRangeTime_ = 0.0f;
    autoAttackResendTimer_ = 0.0f;
    autoAttackFacingSyncTimer_ = 0.0f;
    if (state == WorldState::IN_WORLD && socket) {
        auto packet = AttackSwingPacket::build(targetGuid);
        socket->send(packet);
    }
    LOG_INFO("Starting auto-attack on 0x", std::hex, targetGuid, std::dec);
}

void GameHandler::stopAutoAttack() {
    if (!autoAttacking && !autoAttackRequested_) return;
    autoAttackRequested_ = false;
    autoAttacking = false;
    autoAttackTarget = 0;
    autoAttackOutOfRange_ = false;
    autoAttackOutOfRangeTime_ = 0.0f;
    autoAttackResendTimer_ = 0.0f;
    autoAttackFacingSyncTimer_ = 0.0f;
    if (state == WorldState::IN_WORLD && socket) {
        auto packet = AttackStopPacket::build();
        socket->send(packet);
    }
    LOG_INFO("Stopping auto-attack");
}

void GameHandler::addCombatText(CombatTextEntry::Type type, int32_t amount, uint32_t spellId, bool isPlayerSource) {
    CombatTextEntry entry;
    entry.type = type;
    entry.amount = amount;
    entry.spellId = spellId;
    entry.age = 0.0f;
    entry.isPlayerSource = isPlayerSource;
    combatText.push_back(entry);
}

void GameHandler::updateCombatText(float deltaTime) {
    for (auto& entry : combatText) {
        entry.age += deltaTime;
    }
    combatText.erase(
        std::remove_if(combatText.begin(), combatText.end(),
                       [](const CombatTextEntry& e) { return e.isExpired(); }),
        combatText.end());
}

void GameHandler::autoTargetAttacker(uint64_t attackerGuid) {
    if (attackerGuid == 0 || attackerGuid == playerGuid) return;
    if (targetGuid != 0) return;
    if (!entityManager.hasEntity(attackerGuid)) return;
    setTarget(attackerGuid);
}

void GameHandler::handleAttackStart(network::Packet& packet) {
    AttackStartData data;
    if (!AttackStartParser::parse(packet, data)) return;

    if (data.attackerGuid == playerGuid) {
        autoAttackRequested_ = true;
        autoAttacking = true;
        autoAttackTarget = data.victimGuid;
    } else if (data.victimGuid == playerGuid && data.attackerGuid != 0) {
        hostileAttackers_.insert(data.attackerGuid);
        autoTargetAttacker(data.attackerGuid);

        // Play aggro sound when NPC attacks player
        if (npcAggroCallback_) {
            auto entity = entityManager.getEntity(data.attackerGuid);
            if (entity && entity->getType() == ObjectType::UNIT) {
                glm::vec3 pos(entity->getX(), entity->getY(), entity->getZ());
                npcAggroCallback_(data.attackerGuid, pos);
            }
        }
    }

    // Force both participants to face each other at combat start.
    // Uses atan2(-dy, dx): canonical orientation convention where the West/Y
    // component is negated (renderYaw = orientation + 90°, model-forward = render+X).
    auto attackerEnt = entityManager.getEntity(data.attackerGuid);
    auto victimEnt   = entityManager.getEntity(data.victimGuid);
    if (attackerEnt && victimEnt) {
        float dx = victimEnt->getX() - attackerEnt->getX();
        float dy = victimEnt->getY() - attackerEnt->getY();
        if (std::abs(dx) > 0.01f || std::abs(dy) > 0.01f) {
            attackerEnt->setOrientation(std::atan2(-dy,  dx));   // attacker → victim
            victimEnt->setOrientation  (std::atan2( dy, -dx));   // victim   → attacker
        }
    }
}

void GameHandler::handleAttackStop(network::Packet& packet) {
    AttackStopData data;
    if (!AttackStopParser::parse(packet, data)) return;

    // Keep intent, but clear server-confirmed active state until ATTACKSTART resumes.
    if (data.attackerGuid == playerGuid) {
        autoAttacking = false;
        LOG_DEBUG("SMSG_ATTACKSTOP received (keeping auto-attack intent)");
        if (autoAttackRequested_ && autoAttackTarget != 0 && socket) {
            // Classic-family servers may emit transient ATTACKSTOP when range/facing jitters.
            // Reassert melee intent immediately instead of waiting for periodic resend.
            auto pkt = AttackSwingPacket::build(autoAttackTarget);
            socket->send(pkt);
        }
    } else if (data.victimGuid == playerGuid) {
        hostileAttackers_.erase(data.attackerGuid);
    }
}

void GameHandler::dismount() {
    if (!socket) return;
    // Clear local mount state immediately (optimistic dismount).
    // Server will confirm via SMSG_UPDATE_OBJECT with mountDisplayId=0.
    uint32_t savedMountAura = mountAuraSpellId_;
    if (currentMountDisplayId_ != 0 || taxiMountActive_) {
        if (mountCallback_) {
            mountCallback_(0);
        }
        currentMountDisplayId_ = 0;
        taxiMountActive_ = false;
        taxiMountDisplayId_ = 0;
        mountAuraSpellId_ = 0;
        LOG_INFO("Dismount: cleared local mount state");
    }
    // CMSG_CANCEL_MOUNT_AURA exists in TBC+ (0x0375). Classic/Vanilla doesn't have it.
    uint16_t cancelMountWire = wireOpcode(Opcode::CMSG_CANCEL_MOUNT_AURA);
    if (cancelMountWire != 0xFFFF) {
        network::Packet pkt(cancelMountWire);
        socket->send(pkt);
        LOG_INFO("Sent CMSG_CANCEL_MOUNT_AURA");
    } else if (savedMountAura != 0) {
        // Fallback for Classic/Vanilla: cancel the mount aura by spell ID
        auto pkt = CancelAuraPacket::build(savedMountAura);
        socket->send(pkt);
        LOG_INFO("Sent CMSG_CANCEL_AURA (mount spell ", savedMountAura, ") — Classic fallback");
    } else {
        // No tracked mount aura — try cancelling all indefinite self-cast auras
        // (mount aura detection may have missed if aura arrived after mount field)
        for (const auto& a : playerAuras) {
            if (!a.isEmpty() && a.maxDurationMs < 0 && a.casterGuid == playerGuid) {
                auto pkt = CancelAuraPacket::build(a.spellId);
                socket->send(pkt);
                LOG_INFO("Sent CMSG_CANCEL_AURA (spell ", a.spellId, ") — brute force dismount");
            }
        }
    }
}

void GameHandler::handleForceRunSpeedChange(network::Packet& packet) {
    // Packed GUID
    uint64_t guid = UpdateObjectParser::readPackedGuid(packet);
    // uint32 counter
    uint32_t counter = packet.readUInt32();

    // Determine format from remaining bytes:
    //   5 bytes remaining = uint8(1) + float(4)  — standard 3.3.5a
    //   8 bytes remaining = uint32(4) + float(4) — some forks
    //   4 bytes remaining = float(4)             — no unknown field
    size_t remaining = packet.getSize() - packet.getReadPos();
    if (remaining >= 8) {
        packet.readUInt32();  // unknown (extended format)
    } else if (remaining >= 5) {
        packet.readUInt8();   // unknown (standard 3.3.5a)
    }
    // float newSpeed
    float newSpeed = packet.readFloat();

    LOG_INFO("SMSG_FORCE_RUN_SPEED_CHANGE: guid=0x", std::hex, guid, std::dec,
             " counter=", counter, " speed=", newSpeed);

    if (guid != playerGuid) return;

    // Always ACK the speed change to prevent server stall.
    // Packet format mirrors movement packets: packed guid + counter + movement info + new speed.
    if (socket && !isClassicLikeExpansion()) {
        network::Packet ack(wireOpcode(Opcode::CMSG_FORCE_RUN_SPEED_CHANGE_ACK));
        const bool legacyGuidAck =
            isActiveExpansion("classic") || isActiveExpansion("tbc") || isActiveExpansion("turtle");
        if (legacyGuidAck) {
            ack.writeUInt64(playerGuid);  // CMaNGOS expects full GUID for force speed ACKs
        } else {
            MovementPacket::writePackedGuid(ack, playerGuid);
        }
        ack.writeUInt32(counter);

        MovementInfo wire = movementInfo;
        wire.time = nextMovementTimestampMs();
        if (wire.hasFlag(MovementFlags::ONTRANSPORT)) {
            wire.transportTime = wire.time;
            wire.transportTime2 = wire.time;
        }
        glm::vec3 serverPos = core::coords::canonicalToServer(glm::vec3(wire.x, wire.y, wire.z));
        wire.x = serverPos.x;
        wire.y = serverPos.y;
        wire.z = serverPos.z;
        if (wire.hasFlag(MovementFlags::ONTRANSPORT)) {
            glm::vec3 serverTransport =
                core::coords::canonicalToServer(glm::vec3(wire.transportX, wire.transportY, wire.transportZ));
            wire.transportX = serverTransport.x;
            wire.transportY = serverTransport.y;
            wire.transportZ = serverTransport.z;
        }
        if (packetParsers_) {
            packetParsers_->writeMovementPayload(ack, wire);
        } else {
            MovementPacket::writeMovementPayload(ack, wire);
        }

        ack.writeFloat(newSpeed);
        socket->send(ack);
    }

    // Validate speed - reject garbage/NaN values but still ACK
    if (std::isnan(newSpeed) || newSpeed < 0.1f || newSpeed > 100.0f) {
        LOG_WARNING("Ignoring invalid run speed: ", newSpeed);
        return;
    }

    serverRunSpeed_ = newSpeed;

    // Server can auto-dismount (e.g. entering no-mount areas) and only send a speed change.
    // Keep client mount visuals in sync with server-authoritative movement speed.
    if (!onTaxiFlight_ && !taxiMountActive_ && currentMountDisplayId_ != 0 && newSpeed <= 8.5f) {
        LOG_INFO("Auto-clearing mount from speed change: speed=", newSpeed,
                 " displayId=", currentMountDisplayId_);
        currentMountDisplayId_ = 0;
        if (mountCallback_) {
            mountCallback_(0);
        }
    }
}

void GameHandler::handleForceMoveRootState(network::Packet& packet, bool rooted) {
    // Packet is server movement control update:
    // packedGuid + uint32 counter + [optional unknown field(s)].
    // We always ACK with current movement state, same pattern as speed-change ACKs.
    if (packet.getSize() - packet.getReadPos() < 2) return;
    uint64_t guid = UpdateObjectParser::readPackedGuid(packet);
    if (packet.getSize() - packet.getReadPos() < 4) return;
    uint32_t counter = packet.readUInt32();

    LOG_INFO(rooted ? "SMSG_FORCE_MOVE_ROOT" : "SMSG_FORCE_MOVE_UNROOT",
             ": guid=0x", std::hex, guid, std::dec, " counter=", counter);

    if (guid != playerGuid) return;

    // Keep local movement flags aligned with server authoritative root state.
    if (rooted) {
        movementInfo.flags |= static_cast<uint32_t>(MovementFlags::ROOT);
    } else {
        movementInfo.flags &= ~static_cast<uint32_t>(MovementFlags::ROOT);
    }

    if (!socket || isClassicLikeExpansion()) return;
    uint16_t ackWire = wireOpcode(rooted ? Opcode::CMSG_FORCE_MOVE_ROOT_ACK
                                         : Opcode::CMSG_FORCE_MOVE_UNROOT_ACK);
    if (ackWire == 0xFFFF) return;

    network::Packet ack(ackWire);
    const bool legacyGuidAck =
        isActiveExpansion("classic") || isActiveExpansion("tbc") || isActiveExpansion("turtle");
    if (legacyGuidAck) {
        ack.writeUInt64(playerGuid);  // CMaNGOS expects full GUID for root/unroot ACKs
    } else {
        MovementPacket::writePackedGuid(ack, playerGuid);
    }
    ack.writeUInt32(counter);

    MovementInfo wire = movementInfo;
    wire.time = nextMovementTimestampMs();
    if (wire.hasFlag(MovementFlags::ONTRANSPORT)) {
        wire.transportTime = wire.time;
        wire.transportTime2 = wire.time;
    }
    glm::vec3 serverPos = core::coords::canonicalToServer(glm::vec3(wire.x, wire.y, wire.z));
    wire.x = serverPos.x;
    wire.y = serverPos.y;
    wire.z = serverPos.z;
    if (wire.hasFlag(MovementFlags::ONTRANSPORT)) {
        glm::vec3 serverTransport =
            core::coords::canonicalToServer(glm::vec3(wire.transportX, wire.transportY, wire.transportZ));
        wire.transportX = serverTransport.x;
        wire.transportY = serverTransport.y;
        wire.transportZ = serverTransport.z;
    }
    if (packetParsers_) packetParsers_->writeMovementPayload(ack, wire);
    else MovementPacket::writeMovementPayload(ack, wire);

    socket->send(ack);
}

// ============================================================
// Arena / Battleground Handlers
// ============================================================

void GameHandler::handleBattlefieldStatus(network::Packet& packet) {
    if (packet.getSize() - packet.getReadPos() < 4) return;
    uint32_t queueSlot = packet.readUInt32();

    // Minimal packet = just queueSlot + arenaType(1) when status is NONE
    if (packet.getSize() - packet.getReadPos() < 1) {
        LOG_INFO("Battlefield status: queue slot ", queueSlot, " cleared");
        return;
    }

    uint8_t arenaType = packet.readUInt8();
    if (packet.getSize() - packet.getReadPos() < 1) return;

    // Unknown byte
    packet.readUInt8();
    if (packet.getSize() - packet.getReadPos() < 4) return;
    uint32_t bgTypeId = packet.readUInt32();

    if (packet.getSize() - packet.getReadPos() < 2) return;
    uint16_t unk2 = packet.readUInt16();
    (void)unk2;

    if (packet.getSize() - packet.getReadPos() < 4) return;
    uint32_t clientInstanceId = packet.readUInt32();
    (void)clientInstanceId;

    if (packet.getSize() - packet.getReadPos() < 1) return;
    uint8_t isRatedArena = packet.readUInt8();
    (void)isRatedArena;

    if (packet.getSize() - packet.getReadPos() < 4) return;
    uint32_t statusId = packet.readUInt32();

    std::string bgName = "Battleground #" + std::to_string(bgTypeId);
    if (arenaType > 0) {
        bgName = std::to_string(arenaType) + "v" + std::to_string(arenaType) + " Arena";
    }

    switch (statusId) {
        case 0: // STATUS_NONE
            LOG_INFO("Battlefield status: NONE for ", bgName);
            break;
        case 1: // STATUS_WAIT_QUEUE
            addSystemChatMessage("Queued for " + bgName + ".");
            LOG_INFO("Battlefield status: WAIT_QUEUE for ", bgName);
            break;
        case 2: // STATUS_WAIT_JOIN
            addSystemChatMessage(bgName + " is ready! Type /join to enter.");
            LOG_INFO("Battlefield status: WAIT_JOIN for ", bgName);
            break;
        case 3: // STATUS_IN_PROGRESS
            addSystemChatMessage("Entered " + bgName + ".");
            LOG_INFO("Battlefield status: IN_PROGRESS for ", bgName);
            break;
        case 4: // STATUS_WAIT_LEAVE
            LOG_INFO("Battlefield status: WAIT_LEAVE for ", bgName);
            break;
        default:
            LOG_INFO("Battlefield status: unknown (", statusId, ") for ", bgName);
            break;
    }
}

void GameHandler::handleArenaTeamCommandResult(network::Packet& packet) {
    if (packet.getSize() - packet.getReadPos() < 8) return;
    uint32_t command = packet.readUInt32();
    std::string name = packet.readString();
    uint32_t error = packet.readUInt32();

    static const char* commands[] = { "create", "invite", "leave", "remove", "disband", "leader" };
    std::string cmdName = (command < 6) ? commands[command] : "unknown";

    if (error == 0) {
        addSystemChatMessage("Arena team " + cmdName + " successful" +
                           (name.empty() ? "." : ": " + name));
    } else {
        addSystemChatMessage("Arena team " + cmdName + " failed" +
                           (name.empty() ? "." : " for " + name + "."));
    }
    LOG_INFO("Arena team command: ", cmdName, " name=", name, " error=", error);
}

void GameHandler::handleArenaTeamQueryResponse(network::Packet& packet) {
    if (packet.getSize() - packet.getReadPos() < 4) return;
    uint32_t teamId = packet.readUInt32();
    std::string teamName = packet.readString();
    LOG_INFO("Arena team query response: id=", teamId, " name=", teamName);
}

void GameHandler::handleArenaTeamInvite(network::Packet& packet) {
    std::string playerName = packet.readString();
    std::string teamName = packet.readString();
    addSystemChatMessage(playerName + " has invited you to join " + teamName + ".");
    LOG_INFO("Arena team invite from ", playerName, " to ", teamName);
}

void GameHandler::handleArenaTeamEvent(network::Packet& packet) {
    if (packet.getSize() - packet.getReadPos() < 1) return;
    uint8_t event = packet.readUInt8();

    static const char* events[] = {
        "joined", "left", "removed", "leader changed",
        "disbanded", "created"
    };
    std::string eventName = (event < 6) ? events[event] : "unknown event";

    // Read string params (up to 3)
    uint8_t strCount = 0;
    if (packet.getSize() - packet.getReadPos() >= 1) {
        strCount = packet.readUInt8();
    }

    std::string param1, param2;
    if (strCount >= 1 && packet.getSize() > packet.getReadPos()) param1 = packet.readString();
    if (strCount >= 2 && packet.getSize() > packet.getReadPos()) param2 = packet.readString();

    std::string msg = "Arena team " + eventName;
    if (!param1.empty()) msg += ": " + param1;
    if (!param2.empty()) msg += " (" + param2 + ")";
    addSystemChatMessage(msg);
    LOG_INFO("Arena team event: ", eventName, " ", param1, " ", param2);
}

void GameHandler::handleArenaError(network::Packet& packet) {
    if (packet.getSize() - packet.getReadPos() < 4) return;
    uint32_t error = packet.readUInt32();

    std::string msg;
    switch (error) {
        case 1: msg = "The other team is not big enough."; break;
        case 2: msg = "That team is full."; break;
        case 3: msg = "Not enough members to start."; break;
        case 4: msg = "Too many members."; break;
        default: msg = "Arena error (code " + std::to_string(error) + ")"; break;
    }
    addSystemChatMessage(msg);
    LOG_INFO("Arena error: ", error, " - ", msg);
}

void GameHandler::handleOtherPlayerMovement(network::Packet& packet) {
    // Server relays MSG_MOVE_* for other players: PackedGuid + MovementInfo
    uint64_t moverGuid = UpdateObjectParser::readPackedGuid(packet);
    if (moverGuid == playerGuid || moverGuid == 0) {
        return; // Skip our own echoes
    }

    // Read movement info (expansion-specific format)
    // For classic: moveFlags(u32) + time(u32) + pos(4xf32) + [transport] + [pitch] + fallTime(u32) + [jump] + [splineElev]
    MovementInfo info = {};
    info.flags = packet.readUInt32();
    // WotLK has u16 flags2, TBC has u8, Classic has none.
    // Do NOT use build-number thresholds here (Turtle uses classic formats with a high build).
    uint8_t flags2Size = packetParsers_ ? packetParsers_->movementFlags2Size() : 2;
    if (flags2Size == 2) info.flags2 = packet.readUInt16();
    else if (flags2Size == 1) info.flags2 = packet.readUInt8();
    info.time = packet.readUInt32();
    info.x = packet.readFloat();
    info.y = packet.readFloat();
    info.z = packet.readFloat();
    info.orientation = packet.readFloat();

    // Update entity position in entity manager
    auto entity = entityManager.getEntity(moverGuid);
    if (!entity) {
        return;
    }

    // Convert server coords to canonical
    glm::vec3 canonical = core::coords::serverToCanonical(glm::vec3(info.x, info.y, info.z));
    float canYaw = core::coords::serverToCanonicalYaw(info.orientation);
    // Compute a smoothed interpolation window for this player.
    // Using a raw packet delta causes jitter when timing spikes (e.g. 50ms then 300ms).
    // An exponential moving average of intervals gives a stable playback speed that
    // dead-reckoning in Entity::updateMovement() can bridge without a visible freeze.
    uint32_t durationMs = 120;
    auto itPrev = otherPlayerMoveTimeMs_.find(moverGuid);
    if (itPrev != otherPlayerMoveTimeMs_.end()) {
        uint32_t rawDt = info.time - itPrev->second;  // wraps naturally on uint32_t
        if (rawDt >= 20 && rawDt <= 2000) {
            float fDt = static_cast<float>(rawDt);
            // EMA: smooth the interval so single spike packets don't stutter playback.
            auto& smoothed = otherPlayerSmoothedIntervalMs_[moverGuid];
            if (smoothed < 1.0f) smoothed = fDt;  // first observation — seed directly
            smoothed = 0.7f * smoothed + 0.3f * fDt;
            // Clamp to sane WoW movement rates: ~10 Hz (100ms) normal, up to 2Hz (500ms) slow
            float clamped = std::max(60.0f, std::min(500.0f, smoothed));
            durationMs = static_cast<uint32_t>(clamped);
        }
    }
    otherPlayerMoveTimeMs_[moverGuid] = info.time;

    entity->startMoveTo(canonical.x, canonical.y, canonical.z, canYaw, durationMs / 1000.0f);

    // Notify renderer
    if (creatureMoveCallback_) {
        creatureMoveCallback_(moverGuid, canonical.x, canonical.y, canonical.z, durationMs);
    }
}

void GameHandler::handleCompressedMoves(network::Packet& packet) {
    // Vanilla/Classic SMSG_COMPRESSED_MOVES: raw concatenated sub-packets, NOT zlib.
    // Evidence: observed 1-byte "00" packets which are not valid zlib streams.
    // Each sub-packet: uint8 size (of opcode[2]+payload), uint16 opcode, uint8[] payload.
    // size=0 → invalid/empty, signals end of batch.
    const auto& data = packet.getData();
    size_t dataLen = data.size();

    // Wire opcodes for sub-packet routing
    uint16_t monsterMoveWire          = wireOpcode(Opcode::SMSG_MONSTER_MOVE);
    uint16_t monsterMoveTransportWire = wireOpcode(Opcode::SMSG_MONSTER_MOVE_TRANSPORT);

    // Track unhandled sub-opcodes once per compressed packet (avoid log spam)
    std::unordered_set<uint16_t> unhandledSeen;

    size_t pos = 0;
    while (pos < dataLen) {
        if (pos + 1 > dataLen) break;
        uint8_t subSize = data[pos];
        if (subSize < 2) break;  // size=0 or 1 → empty/end-of-batch sentinel
        if (pos + 1 + subSize > dataLen) {
            LOG_WARNING("SMSG_COMPRESSED_MOVES: sub-packet overruns buffer at pos=", pos);
            break;
        }
        uint16_t subOpcode = static_cast<uint16_t>(data[pos + 1]) |
                             (static_cast<uint16_t>(data[pos + 2]) << 8);
        size_t payloadLen = subSize - 2;
        size_t payloadStart = pos + 3;

        std::vector<uint8_t> subPayload(data.begin() + payloadStart,
                                        data.begin() + payloadStart + payloadLen);
        network::Packet subPacket(subOpcode, subPayload);

        if (subOpcode == monsterMoveWire) {
            handleMonsterMove(subPacket);
        } else if (subOpcode == monsterMoveTransportWire) {
            handleMonsterMoveTransport(subPacket);
        } else {
            if (unhandledSeen.insert(subOpcode).second) {
                LOG_INFO("SMSG_COMPRESSED_MOVES: unhandled sub-opcode 0x",
                         std::hex, subOpcode, std::dec, " payloadLen=", payloadLen);
            }
        }

        pos = payloadStart + payloadLen;
    }
}

void GameHandler::handleMonsterMove(network::Packet& packet) {
    MonsterMoveData data;
    // Turtle WoW (1.17+) compresses each SMSG_MONSTER_MOVE individually:
    // format: uint32 decompressedSize + zlib data (zlib magic = 0x78 ??)
    const auto& rawData = packet.getData();
    bool isCompressed = rawData.size() >= 6 &&
                        rawData[4] == 0x78 &&
                        (rawData[5] == 0x01 || rawData[5] == 0x9C ||
                         rawData[5] == 0xDA || rawData[5] == 0x5E);
    if (isCompressed) {
        uint32_t decompSize = static_cast<uint32_t>(rawData[0]) |
                              (static_cast<uint32_t>(rawData[1]) << 8) |
                              (static_cast<uint32_t>(rawData[2]) << 16) |
                              (static_cast<uint32_t>(rawData[3]) << 24);
        if (decompSize == 0 || decompSize > 65536) {
            LOG_WARNING("SMSG_MONSTER_MOVE: bad decompSize=", decompSize);
            return;
        }
        std::vector<uint8_t> decompressed(decompSize);
        uLongf destLen = decompSize;
        int ret = uncompress(decompressed.data(), &destLen,
                             rawData.data() + 4, rawData.size() - 4);
        if (ret != Z_OK) {
            LOG_WARNING("SMSG_MONSTER_MOVE: zlib error ", ret);
            return;
        }
        decompressed.resize(destLen);
        // Dump ALL bytes for format diagnosis (remove once confirmed)
        static int dumpCount = 0;
        if (dumpCount < 10) {
            ++dumpCount;
            std::string hex;
            for (size_t i = 0; i < destLen; ++i) {
                char buf[4]; snprintf(buf, sizeof(buf), "%02X ", decompressed[i]); hex += buf;
            }
            LOG_INFO("MonsterMove decomp[", destLen, "]: ", hex);
        }
        // Some Turtle WoW compressed move payloads include an inner
        // sub-packet wrapper: uint8 size + uint16 opcode + payload.
        // Do not key this on expansion opcode mappings; strip by structure.
        std::vector<uint8_t> parseBytes = decompressed;
        if (destLen >= 3) {
            uint8_t subSize = decompressed[0];
            size_t wrappedLen = static_cast<size_t>(subSize) + 1; // size byte + subSize bytes
            uint16_t innerOpcode = static_cast<uint16_t>(decompressed[1]) |
                                   (static_cast<uint16_t>(decompressed[2]) << 8);
            uint16_t monsterMoveWire = wireOpcode(Opcode::SMSG_MONSTER_MOVE);
            bool looksLikeMonsterMoveWrapper =
                (innerOpcode == 0x00DD) || (innerOpcode == monsterMoveWire);
            // Strict case: one exact wrapped sub-packet in this decompressed blob.
            if (subSize >= 2 && wrappedLen == destLen && looksLikeMonsterMoveWrapper) {
                size_t payloadStart = 3;
                size_t payloadLen = static_cast<size_t>(subSize) - 2;
                parseBytes.assign(decompressed.begin() + payloadStart,
                                  decompressed.begin() + payloadStart + payloadLen);
            }
        }

        network::Packet decompPacket(packet.getOpcode(), parseBytes);
        if (!packetParsers_->parseMonsterMove(decompPacket, data)) {
            LOG_WARNING("Failed to parse vanilla SMSG_MONSTER_MOVE (decompressed ",
                        destLen, " bytes, parseBytes ", parseBytes.size(), " bytes)");
            return;
        }
    } else if (!packetParsers_->parseMonsterMove(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_MONSTER_MOVE");
        return;
    }

    // Update entity position in entity manager
    auto entity = entityManager.getEntity(data.guid);
    if (!entity) {
        return;
    }

    if (data.hasDest) {
        // Convert destination from server to canonical coords
        glm::vec3 destCanonical = core::coords::serverToCanonical(
            glm::vec3(data.destX, data.destY, data.destZ));

        // Calculate facing angle
        float orientation = entity->getOrientation();
        if (data.moveType == 4) {
            // FacingAngle - server specifies exact angle
            orientation = core::coords::serverToCanonicalYaw(data.facingAngle);
        } else if (data.moveType == 3) {
            // FacingTarget - face toward the target entity.
            // Canonical orientation uses atan2(-dy, dx): the West/Y component
            // must be negated because renderYaw = orientation + 90° and
            // model-forward = render +X, so the sign convention flips.
            auto target = entityManager.getEntity(data.facingTarget);
            if (target) {
                float dx = target->getX() - entity->getX();
                float dy = target->getY() - entity->getY();
                if (std::abs(dx) > 0.01f || std::abs(dy) > 0.01f) {
                    orientation = std::atan2(-dy, dx);
                }
            }
        } else {
            // Normal move - face toward destination.
            float dx = destCanonical.x - entity->getX();
            float dy = destCanonical.y - entity->getY();
            if (std::abs(dx) > 0.01f || std::abs(dy) > 0.01f) {
                orientation = std::atan2(-dy, dx);
            }
        }

        // Anti-backward-glide: if the computed orientation is more than 90° away from
        // the actual travel direction, snap to the travel direction.  FacingTarget
        // (moveType 3) is deliberately different from travel dir, so skip it there.
        if (data.moveType != 3) {
            glm::vec3 startCanonical = core::coords::serverToCanonical(
                glm::vec3(data.x, data.y, data.z));
            float travelDx = destCanonical.x - startCanonical.x;
            float travelDy = destCanonical.y - startCanonical.y;
            float travelLen = std::sqrt(travelDx * travelDx + travelDy * travelDy);
            if (travelLen > 0.5f) {
                float travelAngle = std::atan2(-travelDy, travelDx);
                float diff = orientation - travelAngle;
                // Normalise diff to [-π, π]
                while (diff >  static_cast<float>(M_PI)) diff -= 2.0f * static_cast<float>(M_PI);
                while (diff < -static_cast<float>(M_PI)) diff += 2.0f * static_cast<float>(M_PI);
                if (std::abs(diff) > static_cast<float>(M_PI) * 0.5f) {
                    orientation = travelAngle;
                }
            }
        }

        // Interpolate entity position alongside renderer (so targeting matches visual)
        entity->startMoveTo(destCanonical.x, destCanonical.y, destCanonical.z,
                            orientation, data.duration / 1000.0f);

        // Notify renderer to smoothly move the creature
        if (creatureMoveCallback_) {
            creatureMoveCallback_(data.guid,
                destCanonical.x, destCanonical.y, destCanonical.z,
                data.duration);
        }
    } else if (data.moveType == 1) {
        // Stop at current position
        glm::vec3 posCanonical = core::coords::serverToCanonical(
            glm::vec3(data.x, data.y, data.z));
        entity->setPosition(posCanonical.x, posCanonical.y, posCanonical.z,
                            entity->getOrientation());

        if (creatureMoveCallback_) {
            creatureMoveCallback_(data.guid,
                posCanonical.x, posCanonical.y, posCanonical.z, 0);
        }
    }
}

void GameHandler::handleMonsterMoveTransport(network::Packet& packet) {
    // Parse transport-relative creature movement (NPCs on boats/zeppelins)
    // Packet structure: mover GUID + transport GUID + spline data (local coords)

    uint64_t moverGuid = packet.readUInt64();
    uint8_t unk = packet.readUInt8();  // Unknown byte (usually 0)
    uint64_t transportGuid = packet.readUInt64();

    // Transport-local coordinates (server space)
    float localX = packet.readFloat();
    float localY = packet.readFloat();
    float localZ = packet.readFloat();

    LOG_INFO("SMSG_MONSTER_MOVE_TRANSPORT: mover=0x", std::hex, moverGuid,
             " transport=0x", transportGuid, std::dec,
             " localPos=(", localX, ", ", localY, ", ", localZ, ")");

    // Compose world position: worldPos = transportTransform * localPos
    auto entity = entityManager.getEntity(moverGuid);
    if (!entity) {
        LOG_WARNING("  NPC 0x", std::hex, moverGuid, std::dec, " not found in entity manager");
        return;
    }

    if (transportManager_) {
        // Use TransportManager to compose world position from local offset
        glm::vec3 localPosCanonical = core::coords::serverToCanonical(glm::vec3(localX, localY, localZ));
        setTransportAttachment(moverGuid, entity->getType(), transportGuid, localPosCanonical, false, 0.0f);
        glm::vec3 worldPos = transportManager_->getPlayerWorldPosition(transportGuid, localPosCanonical);

        entity->setPosition(worldPos.x, worldPos.y, worldPos.z, entity->getOrientation());

        LOG_INFO("  Composed NPC world position: (", worldPos.x, ", ", worldPos.y, ", ", worldPos.z, ")");

        if (entity->getType() == ObjectType::UNIT && creatureMoveCallback_) {
            creatureMoveCallback_(moverGuid, worldPos.x, worldPos.y, worldPos.z, 0);
        }
    } else {
        LOG_WARNING("  TransportManager not available for NPC position composition");
    }

    // TODO: Parse full spline data for smooth NPC movement on transport
    // Then update entity position and call creatureMoveCallback_

    // Suppress unused variable warning for now
    (void)unk;
}

void GameHandler::handleAttackerStateUpdate(network::Packet& packet) {
    AttackerStateUpdateData data;
    if (!AttackerStateUpdateParser::parse(packet, data)) return;

    bool isPlayerAttacker = (data.attackerGuid == playerGuid);
    bool isPlayerTarget = (data.targetGuid == playerGuid);
    if (!isPlayerAttacker && !isPlayerTarget) return;  // Not our combat

    if (isPlayerAttacker && meleeSwingCallback_) {
        meleeSwingCallback_();
    }
    if (!isPlayerAttacker && npcSwingCallback_) {
        npcSwingCallback_(data.attackerGuid);
    }

    if (isPlayerTarget && data.attackerGuid != 0) {
        hostileAttackers_.insert(data.attackerGuid);
        autoTargetAttacker(data.attackerGuid);
    }

    // Play combat sounds via CombatSoundManager + character vocalizations
    if (auto* renderer = core::Application::getInstance().getRenderer()) {
        if (auto* csm = renderer->getCombatSoundManager()) {
            auto weaponSize = audio::CombatSoundManager::WeaponSize::MEDIUM;
            if (data.isMiss()) {
                csm->playWeaponMiss(false);
            } else if (data.victimState == 1 || data.victimState == 2) {
                // Dodge/parry — swing whoosh but no impact
                csm->playWeaponSwing(weaponSize, false);
            } else {
                // Hit — swing + flesh impact
                csm->playWeaponSwing(weaponSize, data.isCrit());
                csm->playImpact(weaponSize, audio::CombatSoundManager::ImpactType::FLESH, data.isCrit());
            }
        }
        // Character vocalizations
        if (auto* asm_ = renderer->getActivitySoundManager()) {
            if (isPlayerAttacker && !data.isMiss() && data.victimState != 1 && data.victimState != 2) {
                asm_->playAttackGrunt();
            }
            if (isPlayerTarget && !data.isMiss() && data.victimState != 1 && data.victimState != 2) {
                asm_->playWound(data.isCrit());
            }
        }
    }

    if (data.isMiss()) {
        addCombatText(CombatTextEntry::MISS, 0, 0, isPlayerAttacker);
    } else if (data.victimState == 1) {
        addCombatText(CombatTextEntry::DODGE, 0, 0, isPlayerAttacker);
    } else if (data.victimState == 2) {
        addCombatText(CombatTextEntry::PARRY, 0, 0, isPlayerAttacker);
    } else {
        auto type = data.isCrit() ? CombatTextEntry::CRIT_DAMAGE : CombatTextEntry::MELEE_DAMAGE;
        addCombatText(type, data.totalDamage, 0, isPlayerAttacker);
    }

    (void)isPlayerTarget; // Used for future incoming damage display
}

void GameHandler::handleSpellDamageLog(network::Packet& packet) {
    SpellDamageLogData data;
    if (!SpellDamageLogParser::parse(packet, data)) return;

    bool isPlayerSource = (data.attackerGuid == playerGuid);
    bool isPlayerTarget = (data.targetGuid == playerGuid);
    if (!isPlayerSource && !isPlayerTarget) return;  // Not our combat

    if (isPlayerTarget && data.attackerGuid != 0) {
        hostileAttackers_.insert(data.attackerGuid);
        autoTargetAttacker(data.attackerGuid);
    }

    auto type = data.isCrit ? CombatTextEntry::CRIT_DAMAGE : CombatTextEntry::SPELL_DAMAGE;
    addCombatText(type, static_cast<int32_t>(data.damage), data.spellId, isPlayerSource);
}

void GameHandler::handleSpellHealLog(network::Packet& packet) {
    SpellHealLogData data;
    if (!SpellHealLogParser::parse(packet, data)) return;

    bool isPlayerSource = (data.casterGuid == playerGuid);
    bool isPlayerTarget = (data.targetGuid == playerGuid);
    if (!isPlayerSource && !isPlayerTarget) return;  // Not our combat

    auto type = data.isCrit ? CombatTextEntry::CRIT_HEAL : CombatTextEntry::HEAL;
    addCombatText(type, static_cast<int32_t>(data.heal), data.spellId, isPlayerSource);
}

// ============================================================
// Phase 3: Spells
// ============================================================

void GameHandler::castSpell(uint32_t spellId, uint64_t targetGuid) {
    // Attack (6603) routes to auto-attack instead of cast
    if (spellId == 6603) {
        uint64_t target = targetGuid != 0 ? targetGuid : this->targetGuid;
        if (target != 0) {
            if (autoAttacking) {
                stopAutoAttack();
            } else {
                startAutoAttack(target);
            }
        }
        return;
    }

    if (state != WorldState::IN_WORLD || !socket) return;

    // Casting any spell while mounted → dismount instead
    if (isMounted()) {
        dismount();
        return;
    }

    if (casting) return; // Already casting

    // Hearthstone: cast spell directly (server checks item in inventory)
    // Using CMSG_CAST_SPELL is more reliable than CMSG_USE_ITEM which
    // depends on slot indices matching between client and server.
    uint64_t target = targetGuid != 0 ? targetGuid : this->targetGuid;
    // Self-targeted spells like hearthstone should not send a target
    if (spellId == 8690) target = 0;

    // Warrior Charge (ranks 1-3): client-side range check + charge callback
    // Must face target and validate range BEFORE sending packet to server
    if (spellId == 100 || spellId == 6178 || spellId == 11578) {
        if (target == 0) {
            addSystemChatMessage("You have no target.");
            return;
        }
        auto entity = entityManager.getEntity(target);
        if (!entity) {
            addSystemChatMessage("You have no target.");
            return;
        }
        float tx = entity->getX(), ty = entity->getY(), tz = entity->getZ();
        float dx = tx - movementInfo.x;
        float dy = ty - movementInfo.y;
        float dz = tz - movementInfo.z;
        float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist < 8.0f) {
            addSystemChatMessage("Target is too close.");
            return;
        }
        if (dist > 25.0f) {
            addSystemChatMessage("Out of range.");
            return;
        }
        // Face the target before sending the cast packet to avoid "not in front" rejection
        float yaw = std::atan2(dy, dx);
        movementInfo.orientation = yaw;
        sendMovement(Opcode::MSG_MOVE_SET_FACING);
        if (chargeCallback_) {
            chargeCallback_(target, tx, ty, tz);
        }
    }

    // Instant melee abilities: client-side range + facing check to avoid server "not in front" errors
    {
        uint32_t sid = spellId;
        bool isMeleeAbility =
            sid == 78   || sid == 284   || sid == 285  || sid == 1608  ||  // Heroic Strike
            sid == 11564 || sid == 11565 || sid == 11566 || sid == 11567 ||
            sid == 25286 || sid == 29707 || sid == 30324 ||
            sid == 772  || sid == 6546  || sid == 6547 || sid == 6548 ||   // Rend
            sid == 11572 || sid == 11573 || sid == 11574 || sid == 25208 ||
            sid == 6572 || sid == 6574 || sid == 7379 || sid == 11600 ||   // Revenge
            sid == 11601 || sid == 25288 || sid == 25269 || sid == 30357 ||
            sid == 845  || sid == 7369  || sid == 11608 || sid == 11609 || // Cleave
            sid == 20569 || sid == 25231 || sid == 47519 || sid == 47520 ||
            sid == 12294 || sid == 21551 || sid == 21552 || sid == 21553 || // Mortal Strike
            sid == 25248 || sid == 30330 || sid == 47485 || sid == 47486 ||
            sid == 23922 || sid == 23923 || sid == 23924 || sid == 23925 || // Shield Slam
            sid == 25258 || sid == 30356 || sid == 47487 || sid == 47488;
        if (isMeleeAbility && target != 0) {
            auto entity = entityManager.getEntity(target);
            if (entity) {
                float dx = entity->getX() - movementInfo.x;
                float dy = entity->getY() - movementInfo.y;
                float dz = entity->getZ() - movementInfo.z;
                float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (dist > 8.0f) {
                    addSystemChatMessage("Out of range.");
                    return;
                }
                // Face the target to prevent "not in front" rejection
                float yaw = std::atan2(dy, dx);
                movementInfo.orientation = yaw;
                sendMovement(Opcode::MSG_MOVE_SET_FACING);
            }
        }
    }

    auto packet = packetParsers_
        ? packetParsers_->buildCastSpell(spellId, target, ++castCount)
        : CastSpellPacket::build(spellId, target, ++castCount);
    socket->send(packet);
    LOG_INFO("Casting spell: ", spellId, " on 0x", std::hex, target, std::dec);
}

void GameHandler::cancelCast() {
    if (!casting) return;
    // GameObject interaction cast is client-side timing only.
    if (pendingGameObjectInteractGuid_ == 0 &&
        state == WorldState::IN_WORLD && socket &&
        currentCastSpellId != 0) {
        auto packet = CancelCastPacket::build(currentCastSpellId);
        socket->send(packet);
    }
    pendingGameObjectInteractGuid_ = 0;
    casting = false;
    currentCastSpellId = 0;
    castTimeRemaining = 0.0f;
}

void GameHandler::cancelAura(uint32_t spellId) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = CancelAuraPacket::build(spellId);
    socket->send(packet);
}

void GameHandler::setActionBarSlot(int slot, ActionBarSlot::Type type, uint32_t id) {
    if (slot < 0 || slot >= ACTION_BAR_SLOTS) return;
    actionBar[slot].type = type;
    actionBar[slot].id = id;
    saveCharacterConfig();
}

float GameHandler::getSpellCooldown(uint32_t spellId) const {
    auto it = spellCooldowns.find(spellId);
    return (it != spellCooldowns.end()) ? it->second : 0.0f;
}

void GameHandler::handleInitialSpells(network::Packet& packet) {
    InitialSpellsData data;
    if (!InitialSpellsParser::parse(packet, data)) return;

    knownSpells = {data.spellIds.begin(), data.spellIds.end()};

    // Debug: check if specific spells are in initial spells
    bool has527 = knownSpells.count(527u);
    bool has988 = knownSpells.count(988u);
    bool has1180 = knownSpells.count(1180u);
    LOG_INFO("Initial spells include: 527=", has527, " 988=", has988, " 1180=", has1180);

    // Ensure Attack (6603) and Hearthstone (8690) are always present
    knownSpells.insert(6603u);
    knownSpells.insert(8690u);

    // Set initial cooldowns
    for (const auto& cd : data.cooldowns) {
        if (cd.cooldownMs > 0) {
            spellCooldowns[cd.spellId] = cd.cooldownMs / 1000.0f;
        }
    }

    // Load saved action bar or use defaults (Attack slot 1, Hearthstone slot 12)
    actionBar[0].type = ActionBarSlot::SPELL;
    actionBar[0].id = 6603;  // Attack
    actionBar[11].type = ActionBarSlot::SPELL;
    actionBar[11].id = 8690;  // Hearthstone
    loadCharacterConfig();

    LOG_INFO("Learned ", knownSpells.size(), " spells");
}

void GameHandler::handleCastFailed(network::Packet& packet) {
    CastFailedData data;
    bool ok = packetParsers_ ? packetParsers_->parseCastFailed(packet, data)
                             : CastFailedParser::parse(packet, data);
    if (!ok) return;

    casting = false;
    currentCastSpellId = 0;
    castTimeRemaining = 0.0f;

    // Add system message about failed cast with readable reason
    int powerType = -1;
    auto playerEntity = entityManager.getEntity(playerGuid);
    if (auto playerUnit = std::dynamic_pointer_cast<Unit>(playerEntity)) {
        powerType = playerUnit->getPowerType();
    }
    const char* reason = getSpellCastResultString(data.result, powerType);
    MessageChatData msg;
    msg.type = ChatType::SYSTEM;
    msg.language = ChatLanguage::UNIVERSAL;
    if (reason) {
        msg.message = reason;
    } else {
        msg.message = "Spell cast failed (error " + std::to_string(data.result) + ")";
    }
    addLocalChatMessage(msg);
}

void GameHandler::handleSpellStart(network::Packet& packet) {
    SpellStartData data;
    if (!SpellStartParser::parse(packet, data)) return;

    // If this is the player's own cast, start cast bar
    if (data.casterUnit == playerGuid && data.castTime > 0) {
        casting = true;
        currentCastSpellId = data.spellId;
        castTimeTotal = data.castTime / 1000.0f;
        castTimeRemaining = castTimeTotal;

        // Play precast (channeling) sound
        if (auto* renderer = core::Application::getInstance().getRenderer()) {
            if (auto* ssm = renderer->getSpellSoundManager()) {
                ssm->playPrecast(audio::SpellSoundManager::MagicSchool::ARCANE, audio::SpellSoundManager::SpellPower::MEDIUM);
            }
        }
    }
}

void GameHandler::handleSpellGo(network::Packet& packet) {
    SpellGoData data;
    if (!SpellGoParser::parse(packet, data)) return;

    // Cast completed
    if (data.casterUnit == playerGuid) {
        // Play cast-complete sound before clearing state
        if (auto* renderer = core::Application::getInstance().getRenderer()) {
            if (auto* ssm = renderer->getSpellSoundManager()) {
                ssm->playCast(audio::SpellSoundManager::MagicSchool::ARCANE);
            }
        }

        // Instant melee abilities → trigger attack animation
        uint32_t sid = data.spellId;
        bool isMeleeAbility =
            sid == 78   || sid == 284   || sid == 285  || sid == 1608  ||  // Heroic Strike ranks
            sid == 11564 || sid == 11565 || sid == 11566 || sid == 11567 ||
            sid == 25286 || sid == 29707 || sid == 30324 ||
            sid == 772  || sid == 6546  || sid == 6547 || sid == 6548 ||   // Rend ranks
            sid == 11572 || sid == 11573 || sid == 11574 || sid == 25208 ||
            sid == 6572 || sid == 6574 || sid == 7379 || sid == 11600 ||   // Revenge ranks
            sid == 11601 || sid == 25288 || sid == 25269 || sid == 30357 ||
            sid == 845  || sid == 7369  || sid == 11608 || sid == 11609 || // Cleave ranks
            sid == 20569 || sid == 25231 || sid == 47519 || sid == 47520 ||
            sid == 12294 || sid == 21551 || sid == 21552 || sid == 21553 || // Mortal Strike ranks
            sid == 25248 || sid == 30330 || sid == 47485 || sid == 47486 ||
            sid == 23922 || sid == 23923 || sid == 23924 || sid == 23925 || // Shield Slam ranks
            sid == 25258 || sid == 30356 || sid == 47487 || sid == 47488;
        if (isMeleeAbility && meleeSwingCallback_) {
            meleeSwingCallback_();
        }

        casting = false;
        currentCastSpellId = 0;
        castTimeRemaining = 0.0f;
    }
}

void GameHandler::handleSpellCooldown(network::Packet& packet) {
    SpellCooldownData data;
    if (!SpellCooldownParser::parse(packet, data)) return;

    for (const auto& [spellId, cooldownMs] : data.cooldowns) {
        float seconds = cooldownMs / 1000.0f;
        spellCooldowns[spellId] = seconds;
        // Update action bar cooldowns
        for (auto& slot : actionBar) {
            if (slot.type == ActionBarSlot::SPELL && slot.id == spellId) {
                slot.cooldownTotal = seconds;
                slot.cooldownRemaining = seconds;
            }
        }
    }
}

void GameHandler::handleCooldownEvent(network::Packet& packet) {
    uint32_t spellId = packet.readUInt32();
    // Cooldown finished
    spellCooldowns.erase(spellId);
    for (auto& slot : actionBar) {
        if (slot.type == ActionBarSlot::SPELL && slot.id == spellId) {
            slot.cooldownRemaining = 0.0f;
        }
    }
}

void GameHandler::handleAuraUpdate(network::Packet& packet, bool isAll) {
    AuraUpdateData data;
    if (!AuraUpdateParser::parse(packet, data, isAll)) return;

    // Determine which aura list to update
    std::vector<AuraSlot>* auraList = nullptr;
    if (data.guid == playerGuid) {
        auraList = &playerAuras;
    } else if (data.guid == targetGuid) {
        auraList = &targetAuras;
    }

    if (auraList) {
        if (isAll) {
            auraList->clear();
        }
        uint64_t nowMs = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
        for (auto [slot, aura] : data.updates) {
            // Stamp client timestamp so the UI can count down duration locally
            if (aura.durationMs >= 0) {
                aura.receivedAtMs = nowMs;
            }
            // Ensure vector is large enough
            while (auraList->size() <= slot) {
                auraList->push_back(AuraSlot{});
            }
            (*auraList)[slot] = aura;
        }

        // If player is mounted but we haven't identified the mount aura yet,
        // check newly added auras (aura update may arrive after mountDisplayId)
        if (data.guid == playerGuid && currentMountDisplayId_ != 0 && mountAuraSpellId_ == 0) {
            for (const auto& [slot, aura] : data.updates) {
                if (!aura.isEmpty() && aura.maxDurationMs < 0 && aura.casterGuid == playerGuid) {
                    mountAuraSpellId_ = aura.spellId;
                    LOG_INFO("Mount aura detected from aura update: spellId=", aura.spellId);
                }
            }
        }
    }
}

void GameHandler::handleLearnedSpell(network::Packet& packet) {
    uint32_t spellId = packet.readUInt32();
    knownSpells.insert(spellId);
    LOG_INFO("Learned spell: ", spellId);

    // Check if this spell corresponds to a talent rank
    for (const auto& [talentId, talent] : talentCache_) {
        for (int rank = 0; rank < 5; ++rank) {
            if (talent.rankSpells[rank] == spellId) {
                // Found the talent! Update the rank for the active spec
                uint8_t newRank = rank + 1; // rank is 0-indexed in array, but stored as 1-indexed
                learnedTalents_[activeTalentSpec_][talentId] = newRank;
                LOG_INFO("Talent learned: id=", talentId, " rank=", (int)newRank,
                         " (spell ", spellId, ") in spec ", (int)activeTalentSpec_);
                return;
            }
        }
    }
}

void GameHandler::handleRemovedSpell(network::Packet& packet) {
    uint32_t spellId = packet.readUInt32();
    knownSpells.erase(spellId);
    LOG_INFO("Removed spell: ", spellId);
}

void GameHandler::handleSupercededSpell(network::Packet& packet) {
    // Old spell replaced by new rank (e.g., Fireball Rank 1 -> Fireball Rank 2)
    uint32_t oldSpellId = packet.readUInt32();
    uint32_t newSpellId = packet.readUInt32();

    // Remove old spell
    knownSpells.erase(oldSpellId);

    // Add new spell
    knownSpells.insert(newSpellId);

    LOG_INFO("Spell superceded: ", oldSpellId, " -> ", newSpellId);

    const std::string& newName = getSpellName(newSpellId);
    if (!newName.empty()) {
        addSystemChatMessage("Upgraded to " + newName);
    }
}

void GameHandler::handleUnlearnSpells(network::Packet& packet) {
    // Sent when unlearning multiple spells (e.g., spec change, respec)
    uint32_t spellCount = packet.readUInt32();
    LOG_INFO("Unlearning ", spellCount, " spells");

    for (uint32_t i = 0; i < spellCount && packet.getSize() - packet.getReadPos() >= 4; ++i) {
        uint32_t spellId = packet.readUInt32();
        knownSpells.erase(spellId);
        LOG_INFO("  Unlearned spell: ", spellId);
    }

    if (spellCount > 0) {
        addSystemChatMessage("Unlearned " + std::to_string(spellCount) + " spells");
    }
}

// ============================================================
// Talents
// ============================================================

void GameHandler::handleTalentsInfo(network::Packet& packet) {
    TalentsInfoData data;
    if (!TalentsInfoParser::parse(packet, data)) return;

    // Ensure talent DBCs are loaded
    loadTalentDbc();

    // Validate spec number
    if (data.talentSpec > 1) {
        LOG_WARNING("Invalid talent spec: ", (int)data.talentSpec);
        return;
    }

    // Store talents for this spec
    unspentTalentPoints_[data.talentSpec] = data.unspentPoints;

    // Clear and rebuild learned talents map for this spec
    // Note: If a talent appears in the packet, it's learned (ranks are 0-indexed)
    learnedTalents_[data.talentSpec].clear();
    for (const auto& talent : data.talents) {
        learnedTalents_[data.talentSpec][talent.talentId] = talent.currentRank;
    }

    LOG_INFO("Talents loaded: spec=", (int)data.talentSpec,
             " unspent=", (int)unspentTalentPoints_[data.talentSpec],
             " learned=", learnedTalents_[data.talentSpec].size());

    // If this is the first spec received, set it as active
    static bool firstSpecReceived = false;
    if (!firstSpecReceived) {
        firstSpecReceived = true;
        activeTalentSpec_ = data.talentSpec;

        // Show message to player about active spec
        if (unspentTalentPoints_[data.talentSpec] > 0) {
            std::string msg = "You have " + std::to_string(unspentTalentPoints_[data.talentSpec]) +
                             " unspent talent point";
            if (unspentTalentPoints_[data.talentSpec] > 1) msg += "s";
            msg += " in spec " + std::to_string(data.talentSpec + 1);
            addSystemChatMessage(msg);
        }
    }
}

void GameHandler::learnTalent(uint32_t talentId, uint32_t requestedRank) {
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("learnTalent: Not in world or no socket connection");
        return;
    }

    LOG_INFO("Requesting to learn talent: id=", talentId, " rank=", requestedRank);

    auto packet = LearnTalentPacket::build(talentId, requestedRank);
    socket->send(packet);
}

void GameHandler::switchTalentSpec(uint8_t newSpec) {
    if (newSpec > 1) {
        LOG_WARNING("Invalid talent spec: ", (int)newSpec);
        return;
    }

    if (newSpec == activeTalentSpec_) {
        LOG_INFO("Already on spec ", (int)newSpec);
        return;
    }

    // For now, just switch locally. In a real implementation, we'd send
    // MSG_TALENT_WIPE_CONFIRM to the server to trigger a spec switch.
    // The server would respond with new SMSG_TALENTS_INFO for the new spec.
    activeTalentSpec_ = newSpec;

    LOG_INFO("Switched to talent spec ", (int)newSpec,
             " (unspent=", (int)unspentTalentPoints_[newSpec],
             ", learned=", learnedTalents_[newSpec].size(), ")");

    std::string msg = "Switched to spec " + std::to_string(newSpec + 1);
    if (unspentTalentPoints_[newSpec] > 0) {
        msg += " (" + std::to_string(unspentTalentPoints_[newSpec]) + " unspent point";
        if (unspentTalentPoints_[newSpec] > 1) msg += "s";
        msg += ")";
    }
    addSystemChatMessage(msg);
}

// ============================================================
// Phase 4: Group/Party
// ============================================================

void GameHandler::inviteToGroup(const std::string& playerName) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = GroupInvitePacket::build(playerName);
    socket->send(packet);
    LOG_INFO("Inviting ", playerName, " to group");
}

void GameHandler::acceptGroupInvite() {
    if (state != WorldState::IN_WORLD || !socket) return;
    pendingGroupInvite = false;
    auto packet = GroupAcceptPacket::build();
    socket->send(packet);
    LOG_INFO("Accepted group invite");
}

void GameHandler::declineGroupInvite() {
    if (state != WorldState::IN_WORLD || !socket) return;
    pendingGroupInvite = false;
    auto packet = GroupDeclinePacket::build();
    socket->send(packet);
    LOG_INFO("Declined group invite");
}

void GameHandler::leaveGroup() {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = GroupDisbandPacket::build();
    socket->send(packet);
    partyData = GroupListData{};
    LOG_INFO("Left group");
}

void GameHandler::handleGroupInvite(network::Packet& packet) {
    GroupInviteResponseData data;
    if (!GroupInviteResponseParser::parse(packet, data)) return;

    pendingGroupInvite = true;
    pendingInviterName = data.inviterName;
    LOG_INFO("Group invite from: ", data.inviterName);
    if (!data.inviterName.empty()) {
        addSystemChatMessage(data.inviterName + " has invited you to a group.");
    }
}

void GameHandler::handleGroupDecline(network::Packet& packet) {
    GroupDeclineData data;
    if (!GroupDeclineResponseParser::parse(packet, data)) return;

    MessageChatData msg;
    msg.type = ChatType::SYSTEM;
    msg.language = ChatLanguage::UNIVERSAL;
    msg.message = data.playerName + " has declined your group invitation.";
    addLocalChatMessage(msg);
}

void GameHandler::handleGroupList(network::Packet& packet) {
    if (!GroupListParser::parse(packet, partyData)) return;

    if (partyData.isEmpty()) {
        LOG_INFO("No longer in a group");
        addSystemChatMessage("You are no longer in a group.");
    } else {
        LOG_INFO("In group with ", partyData.memberCount, " members");
        addSystemChatMessage("You are now in a group with " + std::to_string(partyData.memberCount) + " members.");
    }
}

void GameHandler::handleGroupUninvite(network::Packet& packet) {
    (void)packet;
    partyData = GroupListData{};
    LOG_INFO("Removed from group");

    MessageChatData msg;
    msg.type = ChatType::SYSTEM;
    msg.language = ChatLanguage::UNIVERSAL;
    msg.message = "You have been removed from the group.";
    addLocalChatMessage(msg);
}

void GameHandler::handlePartyCommandResult(network::Packet& packet) {
    PartyCommandResultData data;
    if (!PartyCommandResultParser::parse(packet, data)) return;

    if (data.result != PartyResult::OK) {
        MessageChatData msg;
        msg.type = ChatType::SYSTEM;
        msg.language = ChatLanguage::UNIVERSAL;
        msg.message = "Party command failed (error " + std::to_string(static_cast<uint32_t>(data.result)) + ")";
        if (!data.name.empty()) msg.message += " for " + data.name;
        addLocalChatMessage(msg);
    }
}

// ============================================================
// Guild Handlers
// ============================================================

void GameHandler::kickGuildMember(const std::string& playerName) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = GuildRemovePacket::build(playerName);
    socket->send(packet);
    LOG_INFO("Kicking guild member: ", playerName);
}

void GameHandler::disbandGuild() {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = GuildDisbandPacket::build();
    socket->send(packet);
    LOG_INFO("Disbanding guild");
}

void GameHandler::setGuildLeader(const std::string& name) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = GuildLeaderPacket::build(name);
    socket->send(packet);
    LOG_INFO("Setting guild leader: ", name);
}

void GameHandler::setGuildPublicNote(const std::string& name, const std::string& note) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = GuildSetPublicNotePacket::build(name, note);
    socket->send(packet);
    LOG_INFO("Setting public note for ", name, ": ", note);
}

void GameHandler::setGuildOfficerNote(const std::string& name, const std::string& note) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = GuildSetOfficerNotePacket::build(name, note);
    socket->send(packet);
    LOG_INFO("Setting officer note for ", name, ": ", note);
}

void GameHandler::acceptGuildInvite() {
    if (state != WorldState::IN_WORLD || !socket) return;
    pendingGuildInvite_ = false;
    auto packet = GuildAcceptPacket::build();
    socket->send(packet);
    LOG_INFO("Accepted guild invite");
}

void GameHandler::declineGuildInvite() {
    if (state != WorldState::IN_WORLD || !socket) return;
    pendingGuildInvite_ = false;
    auto packet = GuildDeclineInvitationPacket::build();
    socket->send(packet);
    LOG_INFO("Declined guild invite");
}

void GameHandler::queryGuildInfo(uint32_t guildId) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = GuildQueryPacket::build(guildId);
    socket->send(packet);
    LOG_INFO("Querying guild info: guildId=", guildId);
}

void GameHandler::handleGuildInfo(network::Packet& packet) {
    GuildInfoData data;
    if (!GuildInfoParser::parse(packet, data)) return;

    addSystemChatMessage("Guild: " + data.guildName + " (" +
                         std::to_string(data.numMembers) + " members, " +
                         std::to_string(data.numAccounts) + " accounts)");
}

void GameHandler::handleGuildRoster(network::Packet& packet) {
    GuildRosterData data;
    if (!packetParsers_->parseGuildRoster(packet, data)) return;

    guildRoster_ = std::move(data);
    hasGuildRoster_ = true;
    LOG_INFO("Guild roster received: ", guildRoster_.members.size(), " members");
}

void GameHandler::handleGuildQueryResponse(network::Packet& packet) {
    GuildQueryResponseData data;
    if (!packetParsers_->parseGuildQueryResponse(packet, data)) return;

    guildName_ = data.guildName;
    guildRankNames_.clear();
    for (uint32_t i = 0; i < 10; ++i) {
        guildRankNames_.push_back(data.rankNames[i]);
    }
    LOG_INFO("Guild name set to: ", guildName_);
    addSystemChatMessage("Guild: <" + guildName_ + ">");
}

void GameHandler::handleGuildEvent(network::Packet& packet) {
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
            if (data.numStrings >= 1)
                msg = "Guild MOTD: " + data.strings[0];
            break;
        case GuildEvent::JOINED:
            if (data.numStrings >= 1)
                msg = data.strings[0] + " has joined the guild.";
            break;
        case GuildEvent::LEFT:
            if (data.numStrings >= 1)
                msg = data.strings[0] + " has left the guild.";
            break;
        case GuildEvent::REMOVED:
            if (data.numStrings >= 2)
                msg = data.strings[1] + " has been kicked from the guild by " + data.strings[0] + ".";
            break;
        case GuildEvent::LEADER_IS:
            if (data.numStrings >= 1)
                msg = data.strings[0] + " is the guild leader.";
            break;
        case GuildEvent::LEADER_CHANGED:
            if (data.numStrings >= 2)
                msg = data.strings[0] + " has made " + data.strings[1] + " the new guild leader.";
            break;
        case GuildEvent::DISBANDED:
            msg = "Guild has been disbanded.";
            guildName_.clear();
            guildRankNames_.clear();
            guildRoster_ = GuildRosterData{};
            hasGuildRoster_ = false;
            break;
        case GuildEvent::SIGNED_ON:
            if (data.numStrings >= 1)
                msg = "[Guild] " + data.strings[0] + " has come online.";
            break;
        case GuildEvent::SIGNED_OFF:
            if (data.numStrings >= 1)
                msg = "[Guild] " + data.strings[0] + " has gone offline.";
            break;
        default:
            msg = "Guild event " + std::to_string(data.eventType);
            break;
    }

    if (!msg.empty()) {
        MessageChatData chatMsg;
        chatMsg.type = ChatType::GUILD;
        chatMsg.language = ChatLanguage::UNIVERSAL;
        chatMsg.message = msg;
        addLocalChatMessage(chatMsg);
    }

    // Auto-refresh roster after membership/rank changes
    switch (data.eventType) {
        case GuildEvent::PROMOTION:
        case GuildEvent::DEMOTION:
        case GuildEvent::JOINED:
        case GuildEvent::LEFT:
        case GuildEvent::REMOVED:
        case GuildEvent::LEADER_CHANGED:
            if (hasGuildRoster_) requestGuildRoster();
            break;
        default:
            break;
    }
}

void GameHandler::handleGuildInvite(network::Packet& packet) {
    GuildInviteResponseData data;
    if (!GuildInviteResponseParser::parse(packet, data)) return;

    pendingGuildInvite_ = true;
    pendingGuildInviterName_ = data.inviterName;
    pendingGuildInviteGuildName_ = data.guildName;
    LOG_INFO("Guild invite from: ", data.inviterName, " to guild: ", data.guildName);
    addSystemChatMessage(data.inviterName + " has invited you to join " + data.guildName + ".");
}

void GameHandler::handleGuildCommandResult(network::Packet& packet) {
    GuildCommandResultData data;
    if (!GuildCommandResultParser::parse(packet, data)) return;

    if (data.errorCode != 0) {
        std::string msg = "Guild command failed";
        if (!data.name.empty()) msg += " for " + data.name;
        msg += " (error " + std::to_string(data.errorCode) + ")";
        addSystemChatMessage(msg);
    }
}

// ============================================================
// Phase 5: Loot, Gossip, Vendor
// ============================================================

void GameHandler::lootTarget(uint64_t guid) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = LootPacket::build(guid);
    socket->send(packet);
}

void GameHandler::lootItem(uint8_t slotIndex) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = AutostoreLootItemPacket::build(slotIndex);
    socket->send(packet);
}

void GameHandler::closeLoot() {
    if (!lootWindowOpen) return;
    lootWindowOpen = false;
    if (currentLoot.lootGuid != 0 && targetGuid == currentLoot.lootGuid) {
        clearTarget();
    }
    if (state == WorldState::IN_WORLD && socket) {
        auto packet = LootReleasePacket::build(currentLoot.lootGuid);
        socket->send(packet);
    }
    currentLoot = LootResponseData{};
}

void GameHandler::interactWithNpc(uint64_t guid) {
    if (state != WorldState::IN_WORLD || !socket) return;
    auto packet = GossipHelloPacket::build(guid);
    socket->send(packet);
}

void GameHandler::interactWithGameObject(uint64_t guid) {
    if (guid == 0) return;
    if (state != WorldState::IN_WORLD || !socket) return;
    // Do not overlap an actual spell cast.
    if (casting && currentCastSpellId != 0) return;
    // Always clear melee intent before GO interactions.
    stopAutoAttack();
    // Interact immediately; server drives any real cast/channel feedback.
    pendingGameObjectInteractGuid_ = 0;
    performGameObjectInteractionNow(guid);
}

void GameHandler::performGameObjectInteractionNow(uint64_t guid) {
    if (guid == 0) return;
    if (state != WorldState::IN_WORLD || !socket) return;
    bool turtleMode = isActiveExpansion("turtle");

    // Rate-limit to prevent spamming the server
    static uint64_t lastInteractGuid = 0;
    static std::chrono::steady_clock::time_point lastInteractTime{};
    auto now = std::chrono::steady_clock::now();
    // Keep duplicate suppression, but allow quick retry clicks.
    int64_t minRepeatMs = turtleMode ? 150 : 150;
    if (guid == lastInteractGuid &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - lastInteractTime).count() < minRepeatMs) {
        return;
    }
    lastInteractGuid = guid;
    lastInteractTime = now;

    // Ensure GO interaction isn't blocked by stale or active melee state.
    stopAutoAttack();
    auto entity = entityManager.getEntity(guid);
    uint32_t goEntry = 0;
    uint32_t goType = 0;
    std::string goName;

    if (entity) {
        if (entity->getType() == ObjectType::GAMEOBJECT) {
            auto go = std::static_pointer_cast<GameObject>(entity);
            goEntry = go->getEntry();
            goName = go->getName();
            if (auto* info = getCachedGameObjectInfo(goEntry)) goType = info->type;
            if (goType == 5 && !goName.empty()) {
                std::string lower = goName;
                std::transform(lower.begin(), lower.end(), lower.begin(),
                               [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                if (lower.rfind("doodad_", 0) != 0) {
                    addSystemChatMessage(goName);
                }
            }
        }
        // Face object and send heartbeat before use so strict servers don't require
        // a nudge movement to accept interaction.
        float dx = entity->getX() - movementInfo.x;
        float dy = entity->getY() - movementInfo.y;
        float dz = entity->getZ() - movementInfo.z;
        float dist3d = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist3d > 6.0f) {
            addSystemChatMessage("Too far away.");
            return;
        }
        if (std::abs(dx) > 0.01f || std::abs(dy) > 0.01f) {
            movementInfo.orientation = std::atan2(-dy, dx);
            sendMovement(Opcode::MSG_MOVE_SET_FACING);
        }
        sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
    }

    auto packet = GameObjectUsePacket::build(guid);
    socket->send(packet);

    // For mailbox GameObjects (type 19), open mail UI and request mail list.
    // In Vanilla/Classic there is no SMSG_SHOW_MAILBOX — the server just sends
    // animation/sound and expects the client to request the mail list.
    bool isMailbox = false;
    bool chestLike = false;
    // Stock-like behavior: GO use opens GO loot context. Keep eager CMSG_LOOT only
    // as Classic/Turtle fallback behavior.
    bool shouldSendLoot = isActiveExpansion("classic") || isActiveExpansion("turtle");
    if (entity && entity->getType() == ObjectType::GAMEOBJECT) {
        auto go = std::static_pointer_cast<GameObject>(entity);
        auto* info = getCachedGameObjectInfo(go->getEntry());
        if (info && info->type == 19) {
            isMailbox = true;
            shouldSendLoot = false;
            LOG_INFO("Mailbox interaction: opening mail UI and requesting mail list");
            mailboxGuid_ = guid;
            mailboxOpen_ = true;
            hasNewMail_ = false;
            selectedMailIndex_ = -1;
            showMailCompose_ = false;
            refreshMailList();
        } else if (info && info->type == 3) {
            chestLike = true;
        } else if (turtleMode) {
            // Turtle compatibility: keep eager loot open behavior.
            shouldSendLoot = true;
        }
    }
    if (!chestLike && !goName.empty()) {
        std::string lower = goName;
        std::transform(lower.begin(), lower.end(), lower.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        chestLike = (lower.find("chest") != std::string::npos);
    }
    // For WotLK chest-like gameobjects, report use but let server open loot.
    if (!isMailbox && chestLike) {
        if (isActiveExpansion("wotlk")) {
            network::Packet reportUse(wireOpcode(Opcode::CMSG_GAMEOBJ_REPORT_USE));
            reportUse.writeUInt64(guid);
            socket->send(reportUse);
        }
    }
    if (shouldSendLoot) {
        lootTarget(guid);
    }
    // Retry use briefly to survive packet loss/order races. Keep loot retries only
    // when we intentionally use eager loot-open mode.
    const bool retryLoot = shouldSendLoot && (turtleMode || isActiveExpansion("classic"));
    const bool retryUse = turtleMode || isActiveExpansion("classic");
    if (retryUse || retryLoot) {
        pendingGameObjectLootRetries_.push_back(PendingLootRetry{guid, 0.15f, 2, retryLoot});
    }
}

void GameHandler::selectGossipOption(uint32_t optionId) {
    if (state != WorldState::IN_WORLD || !socket || !gossipWindowOpen) return;
    LOG_INFO("selectGossipOption: optionId=", optionId,
             " npcGuid=0x", std::hex, currentGossip.npcGuid, std::dec,
             " menuId=", currentGossip.menuId,
             " numOptions=", currentGossip.options.size());
    auto packet = GossipSelectOptionPacket::build(currentGossip.npcGuid, currentGossip.menuId, optionId);
    socket->send(packet);

    for (const auto& opt : currentGossip.options) {
        if (opt.id != optionId) continue;
        LOG_INFO("  matched option: id=", opt.id, " icon=", (int)opt.icon, " text='", opt.text, "'");

        // Icon-based NPC interaction fallbacks
        // Some servers need the specific activate packet in addition to gossip select
        if (opt.icon == 6) {
            // GOSSIP_ICON_MONEY_BAG = banker
            auto pkt = BankerActivatePacket::build(currentGossip.npcGuid);
            socket->send(pkt);
            LOG_INFO("Sent CMSG_BANKER_ACTIVATE for npc=0x", std::hex, currentGossip.npcGuid, std::dec);
        }

        // Text-based NPC type detection for servers using placeholder strings
        std::string text = opt.text;
        std::string textLower = text;
        std::transform(textLower.begin(), textLower.end(), textLower.begin(),
                       [](unsigned char c){ return static_cast<char>(std::tolower(c)); });

        if (text == "GOSSIP_OPTION_AUCTIONEER" || textLower.find("auction") != std::string::npos) {
            auto pkt = AuctionHelloPacket::build(currentGossip.npcGuid);
            socket->send(pkt);
            LOG_INFO("Sent MSG_AUCTION_HELLO for npc=0x", std::hex, currentGossip.npcGuid, std::dec);
        }

        if (text == "GOSSIP_OPTION_BANKER" || textLower.find("deposit box") != std::string::npos) {
            auto pkt = BankerActivatePacket::build(currentGossip.npcGuid);
            socket->send(pkt);
            LOG_INFO("Sent CMSG_BANKER_ACTIVATE for npc=0x", std::hex, currentGossip.npcGuid, std::dec);
        }

        if (textLower.find("make this inn your home") != std::string::npos ||
            textLower.find("set your home") != std::string::npos) {
            auto bindPkt = BinderActivatePacket::build(currentGossip.npcGuid);
            socket->send(bindPkt);
            LOG_INFO("Sent CMSG_BINDER_ACTIVATE for npc=0x", std::hex, currentGossip.npcGuid, std::dec);
        }
        break;
    }
}

void GameHandler::selectGossipQuest(uint32_t questId) {
    if (state != WorldState::IN_WORLD || !socket || !gossipWindowOpen) return;

    // Keep quest-log fallback for servers that don't provide stable icon semantics.
    const QuestLogEntry* activeQuest = nullptr;
    for (const auto& q : questLog_) {
        if (q.questId == questId) {
            activeQuest = &q;
            break;
        }
    }

    // Validate against server-auth quest slot fields to avoid stale local entries
    // forcing turn-in flow for quests that are not actually accepted.
    auto questInServerLogSlots = [&](uint32_t qid) -> bool {
        if (qid == 0 || lastPlayerFields_.empty()) return false;
        const uint16_t ufQuestStart = fieldIndex(UF::PLAYER_QUEST_LOG_START);
        const uint8_t qStride = packetParsers_ ? packetParsers_->questLogStride() : 5;
        const uint16_t ufQuestEnd = ufQuestStart + 25 * qStride;
        for (const auto& [key, val] : lastPlayerFields_) {
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
    // Only trust server quest-log slots for deciding "already accepted" flow.
    // Gossip icon values can differ across cores/expansions and misclassify
    // available quests as active, which blocks acceptance.
    const bool shouldStartProgressFlow = activeQuestConfirmedByServer;
    if (shouldStartProgressFlow) {
        pendingTurnInQuestId_ = questId;
        pendingTurnInNpcGuid_ = currentGossip.npcGuid;
        pendingTurnInRewardRequest_ = activeQuest ? activeQuest->complete : false;
        auto packet = QuestgiverCompleteQuestPacket::build(currentGossip.npcGuid, questId);
        socket->send(packet);
    } else {
        pendingTurnInQuestId_ = 0;
        pendingTurnInNpcGuid_ = 0;
        pendingTurnInRewardRequest_ = false;
        auto packet = packetParsers_
            ? packetParsers_->buildQueryQuestPacket(currentGossip.npcGuid, questId)
            : QuestgiverQueryQuestPacket::build(currentGossip.npcGuid, questId);
        socket->send(packet);
    }

    gossipWindowOpen = false;
}

bool GameHandler::requestQuestQuery(uint32_t questId, bool force) {
    if (questId == 0 || state != WorldState::IN_WORLD || !socket) return false;
    if (!force && pendingQuestQueryIds_.count(questId)) return false;

    network::Packet pkt(wireOpcode(Opcode::CMSG_QUEST_QUERY));
    pkt.writeUInt32(questId);
    socket->send(pkt);
    pendingQuestQueryIds_.insert(questId);
    return true;
}

void GameHandler::handleQuestDetails(network::Packet& packet) {
    QuestDetailsData data;
    bool ok = packetParsers_ ? packetParsers_->parseQuestDetails(packet, data)
                              : QuestDetailsParser::parse(packet, data);
    if (!ok) {
        LOG_WARNING("Failed to parse SMSG_QUESTGIVER_QUEST_DETAILS");
        return;
    }
    currentQuestDetails = data;
    for (auto& q : questLog_) {
        if (q.questId != data.questId) continue;
        if (!data.title.empty() && (isPlaceholderQuestTitle(q.title) || data.title.size() >= q.title.size())) {
            q.title = data.title;
        }
        if (!data.objectives.empty() && (q.objectives.empty() || data.objectives.size() > q.objectives.size())) {
            q.objectives = data.objectives;
        }
        break;
    }
    questDetailsOpen = true;
    gossipWindowOpen = false;
}

bool GameHandler::hasQuestInLog(uint32_t questId) const {
    for (const auto& q : questLog_) {
        if (q.questId == questId) return true;
    }
    return false;
}

int GameHandler::findQuestLogSlotIndexFromServer(uint32_t questId) const {
    if (questId == 0 || lastPlayerFields_.empty()) return -1;
    const uint16_t ufQuestStart = fieldIndex(UF::PLAYER_QUEST_LOG_START);
    const uint8_t qStride = packetParsers_ ? packetParsers_->questLogStride() : 5;
    for (uint16_t slot = 0; slot < 25; ++slot) {
        const uint16_t idField = ufQuestStart + slot * qStride;
        auto it = lastPlayerFields_.find(idField);
        if (it != lastPlayerFields_.end() && it->second == questId) {
            return static_cast<int>(slot);
        }
    }
    return -1;
}

void GameHandler::addQuestToLocalLogIfMissing(uint32_t questId, const std::string& title, const std::string& objectives) {
    if (questId == 0 || hasQuestInLog(questId)) return;
    QuestLogEntry entry;
    entry.questId = questId;
    entry.title = title.empty() ? ("Quest #" + std::to_string(questId)) : title;
    entry.objectives = objectives;
    questLog_.push_back(std::move(entry));
}

bool GameHandler::resyncQuestLogFromServerSlots(bool forceQueryMetadata) {
    if (lastPlayerFields_.empty()) return false;

    const uint16_t ufQuestStart = fieldIndex(UF::PLAYER_QUEST_LOG_START);
    const uint8_t qStride = packetParsers_ ? packetParsers_->questLogStride() : 5;
    std::unordered_set<uint32_t> serverQuestIds;
    serverQuestIds.reserve(25);
    for (uint16_t slot = 0; slot < 25; ++slot) {
        const uint16_t idField = ufQuestStart + slot * qStride;
        auto it = lastPlayerFields_.find(idField);
        if (it == lastPlayerFields_.end()) continue;
        if (it->second != 0) serverQuestIds.insert(it->second);
    }

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

    if (forceQueryMetadata) {
        for (uint32_t questId : serverQuestIds) {
            requestQuestQuery(questId, false);
        }
    }

    LOG_INFO("Quest log resync from server slots: server=", serverQuestIds.size(),
             " localBefore=", localBefore, " removed=", removed, " added=", added);
    return true;
}

void GameHandler::clearPendingQuestAccept(uint32_t questId) {
    pendingQuestAcceptTimeouts_.erase(questId);
    pendingQuestAcceptNpcGuids_.erase(questId);
}

void GameHandler::triggerQuestAcceptResync(uint32_t questId, uint64_t npcGuid, const char* reason) {
    if (questId == 0 || !socket || state != WorldState::IN_WORLD) return;

    LOG_INFO("Quest accept resync: questId=", questId, " reason=", reason ? reason : "unknown");
    requestQuestQuery(questId, true);

    if (npcGuid != 0) {
        network::Packet qsPkt(wireOpcode(Opcode::CMSG_QUESTGIVER_STATUS_QUERY));
        qsPkt.writeUInt64(npcGuid);
        socket->send(qsPkt);

        auto queryPkt = packetParsers_
            ? packetParsers_->buildQueryQuestPacket(npcGuid, questId)
            : QuestgiverQueryQuestPacket::build(npcGuid, questId);
        socket->send(queryPkt);
    }
}

void GameHandler::acceptQuest() {
    if (!questDetailsOpen || state != WorldState::IN_WORLD || !socket) return;
    const uint32_t questId = currentQuestDetails.questId;
    if (questId == 0) return;
    uint64_t npcGuid = currentQuestDetails.npcGuid;
    if (pendingQuestAcceptTimeouts_.count(questId) != 0) {
        LOG_DEBUG("Ignoring duplicate quest accept while pending: questId=", questId);
        triggerQuestAcceptResync(questId, npcGuid, "duplicate-accept");
        questDetailsOpen = false;
        currentQuestDetails = QuestDetailsData{};
        return;
    }
    const bool inLocalLog = hasQuestInLog(questId);
    const int serverSlot = findQuestLogSlotIndexFromServer(questId);
    if (serverSlot >= 0) {
        LOG_INFO("Ignoring duplicate quest accept already in server quest log: questId=", questId,
                 " slot=", serverSlot);
        questDetailsOpen = false;
        currentQuestDetails = QuestDetailsData{};
        return;
    }
    if (inLocalLog) {
        LOG_WARNING("Quest accept local/server mismatch, allowing re-accept: questId=", questId);
        std::erase_if(questLog_, [&](const QuestLogEntry& q) { return q.questId == questId; });
    }

    network::Packet packet = packetParsers_
        ? packetParsers_->buildAcceptQuestPacket(npcGuid, questId)
        : QuestgiverAcceptQuestPacket::build(npcGuid, questId);
    socket->send(packet);
    pendingQuestAcceptTimeouts_[questId] = 5.0f;
    pendingQuestAcceptNpcGuids_[questId] = npcGuid;

    questDetailsOpen = false;
    currentQuestDetails = QuestDetailsData{};

    // Re-query quest giver status so marker updates (! → ?)
    if (npcGuid) {
        network::Packet qsPkt(wireOpcode(Opcode::CMSG_QUESTGIVER_STATUS_QUERY));
        qsPkt.writeUInt64(npcGuid);
        socket->send(qsPkt);
    }
}

void GameHandler::declineQuest() {
    questDetailsOpen = false;
    currentQuestDetails = QuestDetailsData{};
}

void GameHandler::abandonQuest(uint32_t questId) {
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
        // Best-effort fallback if update fields are stale/missing.
        slotIndex = localIndex;
        LOG_WARNING("Abandon quest using local slot fallback: questId=", questId, " slot=", slotIndex);
    }

    if (slotIndex >= 0 && slotIndex < 25) {
        if (state == WorldState::IN_WORLD && socket) {
            network::Packet pkt(wireOpcode(Opcode::CMSG_QUESTLOG_REMOVE_QUEST));
            pkt.writeUInt8(static_cast<uint8_t>(slotIndex));
            socket->send(pkt);
        }
    } else {
        LOG_WARNING("Abandon quest failed: no quest-log slot found for questId=", questId);
    }

    if (localIndex >= 0) {
        questLog_.erase(questLog_.begin() + static_cast<ptrdiff_t>(localIndex));
    }
}

void GameHandler::handleQuestRequestItems(network::Packet& packet) {
    QuestRequestItemsData data;
    if (!QuestRequestItemsParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_QUESTGIVER_REQUEST_ITEMS");
        return;
    }
    clearPendingQuestAccept(data.questId);

    // Expansion-safe fallback: COMPLETE_QUEST is the default flow.
    // If a server echoes REQUEST_ITEMS again while still completable,
    // request the reward explicitly once.
    if (pendingTurnInRewardRequest_ &&
        data.questId == pendingTurnInQuestId_ &&
        data.npcGuid == pendingTurnInNpcGuid_ &&
        data.isCompletable() &&
        socket) {
        auto rewardReq = QuestgiverRequestRewardPacket::build(data.npcGuid, data.questId);
        socket->send(rewardReq);
        pendingTurnInRewardRequest_ = false;
    }

    currentQuestRequestItems_ = data;
    questRequestItemsOpen_ = true;
    gossipWindowOpen = false;
    questDetailsOpen = false;

    // Query item names for required items
    for (const auto& item : data.requiredItems) {
        queryItemInfo(item.itemId, 0);
    }

    // Server-authoritative turn-in requirements: sync quest-log summary so
    // UI doesn't show stale/inferred objective numbers.
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
                if (const auto* info = getItemInfo(item.itemId)) {
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

void GameHandler::handleQuestOfferReward(network::Packet& packet) {
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
    gossipWindowOpen = false;
    questDetailsOpen = false;

    // Query item names for reward items
    for (const auto& item : data.choiceRewards)
        queryItemInfo(item.itemId, 0);
    for (const auto& item : data.fixedRewards)
        queryItemInfo(item.itemId, 0);
}

void GameHandler::completeQuest() {
    if (!questRequestItemsOpen_ || state != WorldState::IN_WORLD || !socket) return;
    pendingTurnInQuestId_ = currentQuestRequestItems_.questId;
    pendingTurnInNpcGuid_ = currentQuestRequestItems_.npcGuid;
    pendingTurnInRewardRequest_ = currentQuestRequestItems_.isCompletable();

    // Default quest turn-in flow used by all branches.
    auto packet = QuestgiverCompleteQuestPacket::build(
        currentQuestRequestItems_.npcGuid, currentQuestRequestItems_.questId);
    socket->send(packet);
    questRequestItemsOpen_ = false;
    currentQuestRequestItems_ = QuestRequestItemsData{};
}

void GameHandler::closeQuestRequestItems() {
    pendingTurnInRewardRequest_ = false;
    questRequestItemsOpen_ = false;
    currentQuestRequestItems_ = QuestRequestItemsData{};
}

void GameHandler::chooseQuestReward(uint32_t rewardIndex) {
    if (!questOfferRewardOpen_ || state != WorldState::IN_WORLD || !socket) return;
    uint64_t npcGuid = currentQuestOfferReward_.npcGuid;
    LOG_INFO("Completing quest: questId=", currentQuestOfferReward_.questId,
             " npcGuid=", npcGuid, " rewardIndex=", rewardIndex);
    auto packet = QuestgiverChooseRewardPacket::build(
        npcGuid, currentQuestOfferReward_.questId, rewardIndex);
    socket->send(packet);
    pendingTurnInQuestId_ = 0;
    pendingTurnInNpcGuid_ = 0;
    pendingTurnInRewardRequest_ = false;
    questOfferRewardOpen_ = false;
    currentQuestOfferReward_ = QuestOfferRewardData{};

    // Re-query quest giver status so markers update
    if (npcGuid) {
        network::Packet qsPkt(wireOpcode(Opcode::CMSG_QUESTGIVER_STATUS_QUERY));
        qsPkt.writeUInt64(npcGuid);
        socket->send(qsPkt);
    }
}

void GameHandler::closeQuestOfferReward() {
    pendingTurnInRewardRequest_ = false;
    questOfferRewardOpen_ = false;
    currentQuestOfferReward_ = QuestOfferRewardData{};
}

void GameHandler::closeGossip() {
    gossipWindowOpen = false;
    currentGossip = GossipMessageData{};
}

void GameHandler::openVendor(uint64_t npcGuid) {
    if (state != WorldState::IN_WORLD || !socket) return;
    buybackItems_.clear();
    auto packet = ListInventoryPacket::build(npcGuid);
    socket->send(packet);
}

void GameHandler::closeVendor() {
    vendorWindowOpen = false;
    currentVendorItems = ListInventoryData{};
    buybackItems_.clear();
    pendingSellToBuyback_.clear();
    pendingBuybackSlot_ = -1;
    pendingBuybackWireSlot_ = 0;
    pendingBuyItemId_ = 0;
    pendingBuyItemSlot_ = 0;
}

void GameHandler::buyItem(uint64_t vendorGuid, uint32_t itemId, uint32_t slot, uint32_t count) {
    if (state != WorldState::IN_WORLD || !socket) return;
    LOG_INFO("Buy request: vendorGuid=0x", std::hex, vendorGuid, std::dec,
             " itemId=", itemId, " slot=", slot, " count=", count,
             " wire=0x", std::hex, wireOpcode(Opcode::CMSG_BUY_ITEM), std::dec);
    pendingBuyItemId_ = itemId;
    pendingBuyItemSlot_ = slot;
    // Build directly to avoid helper-signature drift across branches (3-arg vs 4-arg helper).
    network::Packet packet(wireOpcode(Opcode::CMSG_BUY_ITEM));
    packet.writeUInt64(vendorGuid);
    packet.writeUInt32(itemId); // item entry
    packet.writeUInt32(slot);   // vendor slot index
    packet.writeUInt32(count);
    // WotLK/AzerothCore expects a trailing byte here.
    packet.writeUInt8(0);
    socket->send(packet);
}

void GameHandler::buyBackItem(uint32_t buybackSlot) {
    if (state != WorldState::IN_WORLD || !socket || currentVendorItems.vendorGuid == 0) return;
    // AzerothCore/WotLK expects absolute buyback inventory slot IDs, not 0-based UI row index.
    // BUYBACK_SLOT_START is 74 in this protocol family.
    constexpr uint32_t kBuybackSlotStart = 74;
    uint32_t wireSlot = kBuybackSlotStart + buybackSlot;
    // This request is independent from normal buy path; avoid stale pending buy context in logs.
    pendingBuyItemId_ = 0;
    pendingBuyItemSlot_ = 0;
    // Build directly so this compiles even when Opcode::CMSG_BUYBACK_ITEM / BuybackItemPacket
    // are not available in some branches.
    constexpr uint16_t kWotlkCmsgBuybackItemOpcode = 0x290;
    LOG_INFO("Buyback request: vendorGuid=0x", std::hex, currentVendorItems.vendorGuid,
             std::dec, " uiSlot=", buybackSlot, " wireSlot=", wireSlot,
             " source=absolute-buyback-slot",
             " wire=0x", std::hex, kWotlkCmsgBuybackItemOpcode, std::dec);
    pendingBuybackSlot_ = static_cast<int>(buybackSlot);
    pendingBuybackWireSlot_ = wireSlot;
    network::Packet packet(kWotlkCmsgBuybackItemOpcode);
    packet.writeUInt64(currentVendorItems.vendorGuid);
    packet.writeUInt32(wireSlot);
    socket->send(packet);
}

void GameHandler::sellItem(uint64_t vendorGuid, uint64_t itemGuid, uint32_t count) {
    if (state != WorldState::IN_WORLD || !socket) return;
    LOG_INFO("Sell request: vendorGuid=0x", std::hex, vendorGuid,
             " itemGuid=0x", itemGuid, std::dec,
             " count=", count, " wire=0x", std::hex, wireOpcode(Opcode::CMSG_SELL_ITEM), std::dec);
    auto packet = SellItemPacket::build(vendorGuid, itemGuid, count);
    socket->send(packet);
}

void GameHandler::sellItemBySlot(int backpackIndex) {
    if (backpackIndex < 0 || backpackIndex >= inventory.getBackpackSize()) return;
    const auto& slot = inventory.getBackpackSlot(backpackIndex);
    if (slot.empty()) return;

    uint32_t sellPrice = slot.item.sellPrice;
    if (sellPrice == 0) {
        if (auto* info = getItemInfo(slot.item.itemId); info && info->valid) {
            sellPrice = info->sellPrice;
        }
    }
    if (sellPrice == 0) {
        addSystemChatMessage("Cannot sell: this item has no vendor value.");
        return;
    }

    uint64_t itemGuid = backpackSlotGuids_[backpackIndex];
    if (itemGuid == 0) {
        itemGuid = resolveOnlineItemGuid(slot.item.itemId);
    }
    LOG_DEBUG("sellItemBySlot: slot=", backpackIndex,
              " item=", slot.item.name,
              " itemGuid=0x", std::hex, itemGuid, std::dec,
              " vendorGuid=0x", std::hex, currentVendorItems.vendorGuid, std::dec);
    if (itemGuid != 0 && currentVendorItems.vendorGuid != 0) {
        BuybackItem sold;
        sold.itemGuid = itemGuid;
        sold.item = slot.item;
        sold.count = 1;
        buybackItems_.push_front(sold);
        if (buybackItems_.size() > 12) buybackItems_.pop_back();
        pendingSellToBuyback_[itemGuid] = sold;
        sellItem(currentVendorItems.vendorGuid, itemGuid, 1);
    } else if (itemGuid == 0) {
        addSystemChatMessage("Cannot sell: item not found in inventory.");
        LOG_WARNING("Sell failed: missing item GUID for slot ", backpackIndex);
    } else {
        addSystemChatMessage("Cannot sell: no vendor.");
    }
}

void GameHandler::autoEquipItemBySlot(int backpackIndex) {
    if (backpackIndex < 0 || backpackIndex >= inventory.getBackpackSize()) return;
    const auto& slot = inventory.getBackpackSlot(backpackIndex);
    if (slot.empty()) return;

    if (state == WorldState::IN_WORLD && socket) {
        // WoW inventory: equipment 0-18, bags 19-22, backpack 23-38
        auto packet = AutoEquipItemPacket::build(0xFF, static_cast<uint8_t>(23 + backpackIndex));
        socket->send(packet);
    }
}

void GameHandler::autoEquipItemInBag(int bagIndex, int slotIndex) {
    if (bagIndex < 0 || bagIndex >= inventory.NUM_BAG_SLOTS) return;
    if (slotIndex < 0 || slotIndex >= inventory.getBagSize(bagIndex)) return;

    if (state == WorldState::IN_WORLD && socket) {
        // Bag items: bag = equip slot 19+bagIndex, slot = index within bag
        auto packet = AutoEquipItemPacket::build(
            static_cast<uint8_t>(19 + bagIndex), static_cast<uint8_t>(slotIndex));
        socket->send(packet);
    }
}

void GameHandler::sellItemInBag(int bagIndex, int slotIndex) {
    if (bagIndex < 0 || bagIndex >= inventory.NUM_BAG_SLOTS) return;
    if (slotIndex < 0 || slotIndex >= inventory.getBagSize(bagIndex)) return;
    const auto& slot = inventory.getBagSlot(bagIndex, slotIndex);
    if (slot.empty()) return;

    uint32_t sellPrice = slot.item.sellPrice;
    if (sellPrice == 0) {
        if (auto* info = getItemInfo(slot.item.itemId); info && info->valid) {
            sellPrice = info->sellPrice;
        }
    }
    if (sellPrice == 0) {
        addSystemChatMessage("Cannot sell: this item has no vendor value.");
        return;
    }

    // Resolve item GUID from container contents
    uint64_t itemGuid = 0;
    uint64_t bagGuid = equipSlotGuids_[19 + bagIndex];
    if (bagGuid != 0) {
        auto it = containerContents_.find(bagGuid);
        if (it != containerContents_.end() && slotIndex < static_cast<int>(it->second.numSlots)) {
            itemGuid = it->second.slotGuids[slotIndex];
        }
    }
    if (itemGuid == 0) {
        itemGuid = resolveOnlineItemGuid(slot.item.itemId);
    }

    if (itemGuid != 0 && currentVendorItems.vendorGuid != 0) {
        BuybackItem sold;
        sold.itemGuid = itemGuid;
        sold.item = slot.item;
        sold.count = 1;
        buybackItems_.push_front(sold);
        if (buybackItems_.size() > 12) buybackItems_.pop_back();
        pendingSellToBuyback_[itemGuid] = sold;
        sellItem(currentVendorItems.vendorGuid, itemGuid, 1);
    } else if (itemGuid == 0) {
        addSystemChatMessage("Cannot sell: item not found.");
    } else {
        addSystemChatMessage("Cannot sell: no vendor.");
    }
}

void GameHandler::unequipToBackpack(EquipSlot equipSlot) {
    if (state != WorldState::IN_WORLD || !socket) return;

    int freeSlot = inventory.findFreeBackpackSlot();
    if (freeSlot < 0) {
        addSystemChatMessage("Cannot unequip: no free backpack slots.");
        return;
    }

    // Use SWAP_ITEM for cross-container moves. For inventory slots we address bag as 0xFF.
    uint8_t srcBag = 0xFF;
    uint8_t srcSlot = static_cast<uint8_t>(equipSlot);
    uint8_t dstBag = 0xFF;
    uint8_t dstSlot = static_cast<uint8_t>(23 + freeSlot);

    LOG_INFO("UnequipToBackpack: equipSlot=", (int)srcSlot,
             " -> backpackIndex=", freeSlot, " (dstSlot=", (int)dstSlot, ")");

    auto packet = SwapItemPacket::build(dstBag, dstSlot, srcBag, srcSlot);
    socket->send(packet);
}

void GameHandler::swapContainerItems(uint8_t srcBag, uint8_t srcSlot, uint8_t dstBag, uint8_t dstSlot) {
    if (!socket || !socket->isConnected()) return;
    LOG_INFO("swapContainerItems: src(bag=", (int)srcBag, " slot=", (int)srcSlot,
             ") -> dst(bag=", (int)dstBag, " slot=", (int)dstSlot, ")");
    auto packet = SwapItemPacket::build(dstBag, dstSlot, srcBag, srcSlot);
    socket->send(packet);
}

void GameHandler::swapBagSlots(int srcBagIndex, int dstBagIndex) {
    if (srcBagIndex < 0 || srcBagIndex > 3 || dstBagIndex < 0 || dstBagIndex > 3) return;
    if (srcBagIndex == dstBagIndex) return;

    // Local swap for immediate visual feedback
    auto srcEquip = static_cast<game::EquipSlot>(static_cast<int>(game::EquipSlot::BAG1) + srcBagIndex);
    auto dstEquip = static_cast<game::EquipSlot>(static_cast<int>(game::EquipSlot::BAG1) + dstBagIndex);
    auto srcItem = inventory.getEquipSlot(srcEquip).item;
    auto dstItem = inventory.getEquipSlot(dstEquip).item;
    inventory.setEquipSlot(srcEquip, dstItem);
    inventory.setEquipSlot(dstEquip, srcItem);

    // Also swap bag contents locally
    inventory.swapBagContents(srcBagIndex, dstBagIndex);

    // Send to server using CMSG_SWAP_ITEM with INVENTORY_SLOT_BAG_0 (255)
    // CMSG_SWAP_INV_ITEM doesn't support bag equip slots (19-22)
    if (socket && socket->isConnected()) {
        uint8_t srcSlot = static_cast<uint8_t>(19 + srcBagIndex);
        uint8_t dstSlot = static_cast<uint8_t>(19 + dstBagIndex);
        LOG_INFO("swapBagSlots: bag ", srcBagIndex, " (slot ", (int)srcSlot,
                 ") <-> bag ", dstBagIndex, " (slot ", (int)dstSlot, ")");
        auto packet = SwapItemPacket::build(255, dstSlot, 255, srcSlot);
        socket->send(packet);
    }
}

void GameHandler::destroyItem(uint8_t bag, uint8_t slot, uint8_t count) {
    if (state != WorldState::IN_WORLD || !socket) return;
    if (count == 0) count = 1;

    // AzerothCore WotLK expects CMSG_DESTROYITEM(bag:u8, slot:u8, count:u32).
    // This opcode is currently not modeled as a logical opcode in our table.
    constexpr uint16_t kCmsgDestroyItem = 0x111;
    network::Packet packet(kCmsgDestroyItem);
    packet.writeUInt8(bag);
    packet.writeUInt8(slot);
    packet.writeUInt32(static_cast<uint32_t>(count));
    LOG_DEBUG("Destroy item request: bag=", (int)bag, " slot=", (int)slot,
              " count=", (int)count, " wire=0x", std::hex, kCmsgDestroyItem, std::dec);
    socket->send(packet);
}

void GameHandler::useItemBySlot(int backpackIndex) {
    if (backpackIndex < 0 || backpackIndex >= inventory.getBackpackSize()) return;
    const auto& slot = inventory.getBackpackSlot(backpackIndex);
    if (slot.empty()) {
        LOG_WARNING("useItemBySlot: slot ", backpackIndex, " is empty");
        return;
    }

    LOG_INFO("useItemBySlot: backpackIndex=", backpackIndex, " itemId=", slot.item.itemId,
             " wowSlot=", 23 + backpackIndex);

    uint64_t itemGuid = backpackSlotGuids_[backpackIndex];
    if (itemGuid == 0) {
        itemGuid = resolveOnlineItemGuid(slot.item.itemId);
    }
    LOG_INFO("useItemBySlot: itemGuid=0x", std::hex, itemGuid, std::dec);

    if (itemGuid != 0 && state == WorldState::IN_WORLD && socket) {
        // WoW inventory: equipment 0-18, bags 19-22, backpack 23-38
        auto packet = packetParsers_
            ? packetParsers_->buildUseItem(0xFF, static_cast<uint8_t>(23 + backpackIndex), itemGuid)
            : UseItemPacket::build(0xFF, static_cast<uint8_t>(23 + backpackIndex), itemGuid);
        LOG_INFO("useItemBySlot: sending CMSG_USE_ITEM, packetSize=", packet.getSize());
        socket->send(packet);
    } else if (itemGuid == 0) {
        LOG_WARNING("Use item failed: missing item GUID for slot ", backpackIndex);
    }
}

void GameHandler::useItemInBag(int bagIndex, int slotIndex) {
    if (bagIndex < 0 || bagIndex >= inventory.NUM_BAG_SLOTS) return;
    if (slotIndex < 0 || slotIndex >= inventory.getBagSize(bagIndex)) return;
    const auto& slot = inventory.getBagSlot(bagIndex, slotIndex);
    if (slot.empty()) return;

    // Resolve item GUID from container contents
    uint64_t itemGuid = 0;
    uint64_t bagGuid = equipSlotGuids_[19 + bagIndex];
    if (bagGuid != 0) {
        auto it = containerContents_.find(bagGuid);
        if (it != containerContents_.end() && slotIndex < static_cast<int>(it->second.numSlots)) {
            itemGuid = it->second.slotGuids[slotIndex];
        }
    }
    if (itemGuid == 0) {
        itemGuid = resolveOnlineItemGuid(slot.item.itemId);
    }

    LOG_INFO("useItemInBag: bag=", bagIndex, " slot=", slotIndex, " itemId=", slot.item.itemId,
             " itemGuid=0x", std::hex, itemGuid, std::dec);

    if (itemGuid != 0 && state == WorldState::IN_WORLD && socket) {
        // WoW bag addressing: bagIndex = equip slot of bag container (19-22)
        // For CMSG_USE_ITEM: bag = 19+bagIndex, slot = slot within bag
        uint8_t wowBag = static_cast<uint8_t>(19 + bagIndex);
        auto packet = packetParsers_
            ? packetParsers_->buildUseItem(wowBag, static_cast<uint8_t>(slotIndex), itemGuid)
            : UseItemPacket::build(wowBag, static_cast<uint8_t>(slotIndex), itemGuid);
        LOG_INFO("useItemInBag: sending CMSG_USE_ITEM, bag=", (int)wowBag, " slot=", slotIndex,
                 " packetSize=", packet.getSize());
        socket->send(packet);
    } else if (itemGuid == 0) {
        LOG_WARNING("Use item in bag failed: missing item GUID for bag ", bagIndex, " slot ", slotIndex);
    }
}

void GameHandler::useItemById(uint32_t itemId) {
    if (itemId == 0) return;
    LOG_INFO("useItemById: searching for itemId=", itemId, " in backpack (", inventory.getBackpackSize(), " slots)");
    // Search backpack first
    for (int i = 0; i < inventory.getBackpackSize(); i++) {
        const auto& slot = inventory.getBackpackSlot(i);
        if (!slot.empty() && slot.item.itemId == itemId) {
            LOG_INFO("useItemById: found itemId=", itemId, " at backpack slot ", i);
            useItemBySlot(i);
            return;
        }
    }
    // Search bags
    for (int bag = 0; bag < inventory.NUM_BAG_SLOTS; bag++) {
        int bagSize = inventory.getBagSize(bag);
        for (int slot = 0; slot < bagSize; slot++) {
            const auto& bagSlot = inventory.getBagSlot(bag, slot);
            if (!bagSlot.empty() && bagSlot.item.itemId == itemId) {
                LOG_INFO("useItemById: found itemId=", itemId, " in bag ", bag, " slot ", slot);
                useItemInBag(bag, slot);
                return;
            }
        }
    }
    LOG_WARNING("useItemById: itemId=", itemId, " not found in inventory");
}

void GameHandler::unstuck() {
    if (unstuckCallback_) {
        unstuckCallback_();
        addSystemChatMessage("Unstuck: snapped upward. Use /unstuckgy for full teleport.");
    }
}

void GameHandler::unstuckGy() {
    if (unstuckGyCallback_) {
        unstuckGyCallback_();
        addSystemChatMessage("Unstuck: teleported to safe location.");
    }
}

void GameHandler::handleLootResponse(network::Packet& packet) {
    if (!LootResponseParser::parse(packet, currentLoot)) return;
    lootWindowOpen = true;
    localLootState_[currentLoot.lootGuid] = LocalLootState{currentLoot, false};

    // Query item info so loot window can show names instead of IDs
    for (const auto& item : currentLoot.items) {
        queryItemInfo(item.itemId, 0);
    }

    if (currentLoot.gold > 0) {
        if (state == WorldState::IN_WORLD && socket) {
            // Auto-loot gold by sending CMSG_LOOT_MONEY (server handles the rest)
            bool suppressFallback = false;
            auto cooldownIt = recentLootMoneyAnnounceCooldowns_.find(currentLoot.lootGuid);
            if (cooldownIt != recentLootMoneyAnnounceCooldowns_.end() && cooldownIt->second > 0.0f) {
                suppressFallback = true;
            }
            pendingLootMoneyGuid_ = suppressFallback ? 0 : currentLoot.lootGuid;
            pendingLootMoneyAmount_ = suppressFallback ? 0 : currentLoot.gold;
            pendingLootMoneyNotifyTimer_ = suppressFallback ? 0.0f : 0.4f;
            auto pkt = LootMoneyPacket::build();
            socket->send(pkt);
            currentLoot.gold = 0;
        }
    }

    // Auto-loot items when enabled
    if (autoLoot_ && state == WorldState::IN_WORLD && socket) {
        for (const auto& item : currentLoot.items) {
            auto pkt = AutostoreLootItemPacket::build(item.slotIndex);
            socket->send(pkt);
        }
    }
}

void GameHandler::handleLootReleaseResponse(network::Packet& packet) {
    (void)packet;
    localLootState_.erase(currentLoot.lootGuid);
    lootWindowOpen = false;
    currentLoot = LootResponseData{};
}

void GameHandler::handleLootRemoved(network::Packet& packet) {
    uint8_t slotIndex = packet.readUInt8();
    for (auto it = currentLoot.items.begin(); it != currentLoot.items.end(); ++it) {
        if (it->slotIndex == slotIndex) {
            std::string itemName = "item #" + std::to_string(it->itemId);
            if (const ItemQueryResponseData* info = getItemInfo(it->itemId)) {
                if (!info->name.empty()) {
                    itemName = info->name;
                }
            }
            std::ostringstream msg;
            msg << "Looted: " << itemName;
            if (it->count > 1) {
                msg << " x" << it->count;
            }
            addSystemChatMessage(msg.str());
            currentLoot.items.erase(it);
            break;
        }
    }
}

void GameHandler::handleGossipMessage(network::Packet& packet) {
    bool ok = packetParsers_ ? packetParsers_->parseGossipMessage(packet, currentGossip)
                             : GossipMessageParser::parse(packet, currentGossip);
    if (!ok) return;
    if (questDetailsOpen) return; // Don't reopen gossip while viewing quest
    gossipWindowOpen = true;
    vendorWindowOpen = false; // Close vendor if gossip opens

    // Update known quest-log entries based on gossip quests.
    // Do not synthesize new "active quest" entries from gossip alone.
    bool hasAvailableQuest = false;
    bool hasRewardQuest = false;
    bool hasIncompleteQuest = false;
    auto questIconIsCompletable = [](uint32_t icon) {
        return icon == 5 || icon == 6 || icon == 10;
    };
    auto questIconIsIncomplete = [](uint32_t icon) {
        return icon == 3 || icon == 4;
    };
    auto questIconIsAvailable = [](uint32_t icon) {
        return icon == 2 || icon == 7 || icon == 8;
    };

    for (const auto& questItem : currentGossip.quests) {
        // WotLK gossip questIcon is an integer enum, NOT a bitmask:
        //   2 = yellow !  (available, not yet accepted)
        //   4 = gray ?    (active, objectives incomplete)
        //   5 = gold ?    (complete, ready to turn in)
        // Bit-masking these values is wrong: 4 & 0x04 = true, treating incomplete
        // quests as completable and causing the server to reject the turn-in request.
        bool isCompletable = questIconIsCompletable(questItem.questIcon);
        bool isIncomplete  = questIconIsIncomplete(questItem.questIcon);
        bool isAvailable   = questIconIsAvailable(questItem.questIcon);

        hasAvailableQuest |= isAvailable;
        hasRewardQuest |= isCompletable;
        hasIncompleteQuest |= isIncomplete;

        // Update existing quest entry if present
        for (auto& quest : questLog_) {
            if (quest.questId == questItem.questId) {
                quest.complete = isCompletable;
                quest.title = questItem.title;
                LOG_INFO("Updated quest ", questItem.questId, " in log: complete=", isCompletable);
                break;
            }
        }
    }

    // Keep overhead marker aligned with what this gossip actually offers.
    if (currentGossip.npcGuid != 0) {
        QuestGiverStatus derivedStatus = QuestGiverStatus::NONE;
        if (hasRewardQuest) derivedStatus = QuestGiverStatus::REWARD;
        else if (hasAvailableQuest) derivedStatus = QuestGiverStatus::AVAILABLE;
        else if (hasIncompleteQuest) derivedStatus = QuestGiverStatus::INCOMPLETE;
        if (derivedStatus != QuestGiverStatus::NONE) {
            npcQuestStatus_[currentGossip.npcGuid] = derivedStatus;
        }
    }

    // Play NPC greeting voice
    if (npcGreetingCallback_ && currentGossip.npcGuid != 0) {
        auto entity = entityManager.getEntity(currentGossip.npcGuid);
        if (entity) {
            glm::vec3 npcPos(entity->getX(), entity->getY(), entity->getZ());
            npcGreetingCallback_(currentGossip.npcGuid, npcPos);
        }
    }
}

void GameHandler::handleQuestgiverQuestList(network::Packet& packet) {
    if (packet.getSize() - packet.getReadPos() < 8) return;

    GossipMessageData data;
    data.npcGuid = packet.readUInt64();
    data.menuId = 0;
    data.titleTextId = 0;

    // Server text (header/greeting) and optional emote fields.
    std::string header = packet.readString();
    if (packet.getSize() - packet.getReadPos() >= 8) {
        (void)packet.readUInt32(); // emoteDelay / unk
        (void)packet.readUInt32(); // emote / unk
    }
    (void)header;

    auto readQuestCount = [&](network::Packet& pkt) -> uint32_t {
        size_t rem = pkt.getSize() - pkt.getReadPos();
        if (rem >= 4) {
            size_t p = pkt.getReadPos();
            uint32_t c = pkt.readUInt32();
            if (c <= 64) return c;
            pkt.setReadPos(p);
        }
        if (rem >= 1) {
            return static_cast<uint32_t>(pkt.readUInt8());
        }
        return 0;
    };

    uint32_t questCount = readQuestCount(packet);
    data.quests.reserve(questCount);
    for (uint32_t i = 0; i < questCount; ++i) {
        if (packet.getSize() - packet.getReadPos() < 12) break;
        GossipQuestItem q;
        q.questId = packet.readUInt32();
        q.questIcon = packet.readUInt32();
        q.questLevel = static_cast<int32_t>(packet.readUInt32());

        // WotLK includes questFlags + isRepeatable; Classic variants may omit.
        size_t titlePos = packet.getReadPos();
        if (packet.getSize() - packet.getReadPos() >= 5) {
            q.questFlags = packet.readUInt32();
            q.isRepeatable = packet.readUInt8();
            q.title = normalizeWowTextTokens(packet.readString());
            if (q.title.empty()) {
                packet.setReadPos(titlePos);
                q.questFlags = 0;
                q.isRepeatable = 0;
                q.title = normalizeWowTextTokens(packet.readString());
            }
        } else {
            q.questFlags = 0;
            q.isRepeatable = 0;
            q.title = normalizeWowTextTokens(packet.readString());
        }
        if (q.questId != 0) {
            data.quests.push_back(std::move(q));
        }
    }

    currentGossip = std::move(data);
    gossipWindowOpen = true;
    vendorWindowOpen = false;

    bool hasAvailableQuest = false;
    bool hasRewardQuest = false;
    bool hasIncompleteQuest = false;
    auto questIconIsCompletable = [](uint32_t icon) {
        return icon == 5 || icon == 6 || icon == 10;
    };
    auto questIconIsIncomplete = [](uint32_t icon) {
        return icon == 3 || icon == 4;
    };
    auto questIconIsAvailable = [](uint32_t icon) {
        return icon == 2 || icon == 7 || icon == 8;
    };

    for (const auto& questItem : currentGossip.quests) {
        bool isCompletable = questIconIsCompletable(questItem.questIcon);
        bool isIncomplete  = questIconIsIncomplete(questItem.questIcon);
        bool isAvailable   = questIconIsAvailable(questItem.questIcon);
        hasAvailableQuest |= isAvailable;
        hasRewardQuest |= isCompletable;
        hasIncompleteQuest |= isIncomplete;
    }
    if (currentGossip.npcGuid != 0) {
        QuestGiverStatus derivedStatus = QuestGiverStatus::NONE;
        if (hasRewardQuest) derivedStatus = QuestGiverStatus::REWARD;
        else if (hasAvailableQuest) derivedStatus = QuestGiverStatus::AVAILABLE;
        else if (hasIncompleteQuest) derivedStatus = QuestGiverStatus::INCOMPLETE;
        if (derivedStatus != QuestGiverStatus::NONE) {
            npcQuestStatus_[currentGossip.npcGuid] = derivedStatus;
        }
    }

    LOG_INFO("Questgiver quest list: npc=0x", std::hex, currentGossip.npcGuid, std::dec,
             " quests=", currentGossip.quests.size());
}

void GameHandler::handleGossipComplete(network::Packet& packet) {
    (void)packet;

    // Play farewell sound before closing
    if (npcFarewellCallback_ && currentGossip.npcGuid != 0) {
        auto entity = entityManager.getEntity(currentGossip.npcGuid);
        if (entity && entity->getType() == ObjectType::UNIT) {
            glm::vec3 pos(entity->getX(), entity->getY(), entity->getZ());
            npcFarewellCallback_(currentGossip.npcGuid, pos);
        }
    }

    gossipWindowOpen = false;
    currentGossip = GossipMessageData{};
}

void GameHandler::handleListInventory(network::Packet& packet) {
    if (!ListInventoryParser::parse(packet, currentVendorItems)) return;
    vendorWindowOpen = true;
    gossipWindowOpen = false; // Close gossip if vendor opens

    // Play vendor sound
    if (npcVendorCallback_ && currentVendorItems.vendorGuid != 0) {
        auto entity = entityManager.getEntity(currentVendorItems.vendorGuid);
        if (entity && entity->getType() == ObjectType::UNIT) {
            glm::vec3 pos(entity->getX(), entity->getY(), entity->getZ());
            npcVendorCallback_(currentVendorItems.vendorGuid, pos);
        }
    }

    // Query item info for all vendor items so we can show names
    for (const auto& item : currentVendorItems.items) {
        queryItemInfo(item.itemId, 0);
    }
}

// ============================================================
// Trainer
// ============================================================

void GameHandler::handleTrainerList(network::Packet& packet) {
    if (!TrainerListParser::parse(packet, currentTrainerList_)) return;
    trainerWindowOpen_ = true;
    gossipWindowOpen = false;

    // Debug: log known spells
    LOG_INFO("Known spells count: ", knownSpells.size());
    if (knownSpells.size() <= 50) {
        std::string spellList;
        for (uint32_t id : knownSpells) {
            if (!spellList.empty()) spellList += ", ";
            spellList += std::to_string(id);
        }
        LOG_INFO("Known spells: ", spellList);
    }

    // Check if specific prerequisite spells are known
    bool has527 = knownSpells.count(527u);
    bool has25312 = knownSpells.count(25312u);
    LOG_INFO("Prerequisite check: 527=", has527, " 25312=", has25312);

    // Debug: log first few trainer spells to see their state
    LOG_INFO("Trainer spells received: ", currentTrainerList_.spells.size(), " spells");
    for (size_t i = 0; i < std::min(size_t(5), currentTrainerList_.spells.size()); ++i) {
        const auto& s = currentTrainerList_.spells[i];
        LOG_INFO("  Spell[", i, "]: id=", s.spellId, " state=", (int)s.state,
                 " cost=", s.spellCost, " reqLvl=", (int)s.reqLevel,
                 " chain=(", s.chainNode1, ",", s.chainNode2, ",", s.chainNode3, ")");
    }


    // Ensure caches are populated
    loadSpellNameCache();
    loadSkillLineDbc();
    loadSkillLineAbilityDbc();
    categorizeTrainerSpells();
}

void GameHandler::trainSpell(uint32_t spellId) {
    LOG_INFO("trainSpell called: spellId=", spellId, " state=", (int)state, " socket=", (socket ? "yes" : "no"));
    if (state != WorldState::IN_WORLD || !socket) {
        LOG_WARNING("trainSpell: Not in world or no socket connection");
        return;
    }

    // Find spell cost in trainer list
    uint32_t spellCost = 0;
    for (const auto& spell : currentTrainerList_.spells) {
        if (spell.spellId == spellId) {
            spellCost = spell.spellCost;
            break;
        }
    }
    LOG_INFO("Player money: ", playerMoneyCopper_, " copper, spell cost: ", spellCost, " copper");

    LOG_INFO("Sending CMSG_TRAINER_BUY_SPELL: guid=", currentTrainerList_.trainerGuid,
             " spellId=", spellId);
    auto packet = TrainerBuySpellPacket::build(
        currentTrainerList_.trainerGuid,
        spellId);
    socket->send(packet);
    LOG_INFO("CMSG_TRAINER_BUY_SPELL sent");
}

void GameHandler::closeTrainer() {
    trainerWindowOpen_ = false;
    currentTrainerList_ = TrainerListData{};
    trainerTabs_.clear();
}

void GameHandler::loadSpellNameCache() {
    if (spellNameCacheLoaded_) return;
    spellNameCacheLoaded_ = true;

    auto* am = core::Application::getInstance().getAssetManager();
    if (!am || !am->isInitialized()) return;

    auto dbc = am->loadDBC("Spell.dbc");
    if (!dbc || !dbc->isLoaded()) {
        LOG_WARNING("Trainer: Could not load Spell.dbc for spell names");
        return;
    }

    if (dbc->getFieldCount() < 154) {
        LOG_WARNING("Trainer: Spell.dbc has too few fields");
        return;
    }

    const auto* spellL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("Spell") : nullptr;
    uint32_t count = dbc->getRecordCount();
    for (uint32_t i = 0; i < count; ++i) {
        uint32_t id = dbc->getUInt32(i, spellL ? (*spellL)["ID"] : 0);
        if (id == 0) continue;
        std::string name = dbc->getString(i, spellL ? (*spellL)["Name"] : 136);
        std::string rank = dbc->getString(i, spellL ? (*spellL)["Rank"] : 153);
        if (!name.empty()) {
            spellNameCache_[id] = {std::move(name), std::move(rank)};
        }
    }
    LOG_INFO("Trainer: Loaded ", spellNameCache_.size(), " spell names from Spell.dbc");
}

void GameHandler::loadSkillLineAbilityDbc() {
    if (skillLineAbilityLoaded_) return;
    skillLineAbilityLoaded_ = true;

    auto* am = core::Application::getInstance().getAssetManager();
    if (!am || !am->isInitialized()) return;

    auto slaDbc = am->loadDBC("SkillLineAbility.dbc");
    if (slaDbc && slaDbc->isLoaded()) {
        const auto* slaL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("SkillLineAbility") : nullptr;
        for (uint32_t i = 0; i < slaDbc->getRecordCount(); i++) {
            uint32_t skillLineId = slaDbc->getUInt32(i, slaL ? (*slaL)["SkillLineID"] : 1);
            uint32_t spellId = slaDbc->getUInt32(i, slaL ? (*slaL)["SpellID"] : 2);
            if (spellId > 0 && skillLineId > 0) {
                spellToSkillLine_[spellId] = skillLineId;
            }
        }
        LOG_INFO("Trainer: Loaded ", spellToSkillLine_.size(), " skill line abilities");
    }
}

void GameHandler::categorizeTrainerSpells() {
    trainerTabs_.clear();

    static constexpr uint32_t SKILLLINE_CATEGORY_CLASS = 7;

    // Group spells by skill line (category 7 = class spec tabs)
    std::map<uint32_t, std::vector<const TrainerSpell*>> specialtySpells;
    std::vector<const TrainerSpell*> generalSpells;

    for (const auto& spell : currentTrainerList_.spells) {
        auto slIt = spellToSkillLine_.find(spell.spellId);
        if (slIt != spellToSkillLine_.end()) {
            uint32_t skillLineId = slIt->second;
            auto catIt = skillLineCategories_.find(skillLineId);
            if (catIt != skillLineCategories_.end() && catIt->second == SKILLLINE_CATEGORY_CLASS) {
                specialtySpells[skillLineId].push_back(&spell);
                continue;
            }
        }
        generalSpells.push_back(&spell);
    }

    // Sort by spell name within each group
    auto byName = [this](const TrainerSpell* a, const TrainerSpell* b) {
        return getSpellName(a->spellId) < getSpellName(b->spellId);
    };

    // Build named tabs sorted alphabetically
    std::vector<std::pair<std::string, std::vector<const TrainerSpell*>>> named;
    for (auto& [skillLineId, spells] : specialtySpells) {
        auto nameIt = skillLineNames_.find(skillLineId);
        std::string tabName = (nameIt != skillLineNames_.end()) ? nameIt->second : "Specialty";
        std::sort(spells.begin(), spells.end(), byName);
        named.push_back({std::move(tabName), std::move(spells)});
    }
    std::sort(named.begin(), named.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });

    for (auto& [name, spells] : named) {
        trainerTabs_.push_back({std::move(name), std::move(spells)});
    }

    // General tab last
    if (!generalSpells.empty()) {
        std::sort(generalSpells.begin(), generalSpells.end(), byName);
        trainerTabs_.push_back({"General", std::move(generalSpells)});
    }

    LOG_INFO("Trainer: Categorized into ", trainerTabs_.size(), " tabs");
}

void GameHandler::loadTalentDbc() {
    if (talentDbcLoaded_) return;
    talentDbcLoaded_ = true;

    auto* am = core::Application::getInstance().getAssetManager();
    if (!am || !am->isInitialized()) return;

    // Load Talent.dbc
    auto talentDbc = am->loadDBC("Talent.dbc");
    if (talentDbc && talentDbc->isLoaded()) {
        // Talent.dbc structure (WoW 3.3.5a):
        // 0: TalentID
        // 1: TalentTabID
        // 2: Row (tier)
        // 3: Column
        // 4-8: RankID[0-4] (spell IDs for ranks 1-5)
        // 9-11: PrereqTalent[0-2]
        // 12-14: PrereqRank[0-2]
        // (other fields less relevant for basic functionality)

        const auto* talL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("Talent") : nullptr;
        const uint32_t tID = talL ? (*talL)["ID"] : 0;
        const uint32_t tTabID = talL ? (*talL)["TabID"] : 1;
        const uint32_t tRow = talL ? (*talL)["Row"] : 2;
        const uint32_t tCol = talL ? (*talL)["Column"] : 3;
        const uint32_t tRank0 = talL ? (*talL)["RankSpell0"] : 4;
        const uint32_t tPrereq0 = talL ? (*talL)["PrereqTalent0"] : 9;
        const uint32_t tPrereqR0 = talL ? (*talL)["PrereqRank0"] : 12;

        uint32_t count = talentDbc->getRecordCount();
        for (uint32_t i = 0; i < count; ++i) {
            TalentEntry entry;
            entry.talentId = talentDbc->getUInt32(i, tID);
            if (entry.talentId == 0) continue;

            entry.tabId = talentDbc->getUInt32(i, tTabID);
            entry.row = static_cast<uint8_t>(talentDbc->getUInt32(i, tRow));
            entry.column = static_cast<uint8_t>(talentDbc->getUInt32(i, tCol));

            // Rank spells (1-5 ranks)
            for (int r = 0; r < 5; ++r) {
                entry.rankSpells[r] = talentDbc->getUInt32(i, tRank0 + r);
            }

            // Prerequisites
            for (int p = 0; p < 3; ++p) {
                entry.prereqTalent[p] = talentDbc->getUInt32(i, tPrereq0 + p);
                entry.prereqRank[p] = static_cast<uint8_t>(talentDbc->getUInt32(i, tPrereqR0 + p));
            }

            // Calculate max rank
            entry.maxRank = 0;
            for (int r = 0; r < 5; ++r) {
                if (entry.rankSpells[r] != 0) {
                    entry.maxRank = r + 1;
                }
            }

            talentCache_[entry.talentId] = entry;
        }
        LOG_INFO("Loaded ", talentCache_.size(), " talents from Talent.dbc");
    } else {
        LOG_WARNING("Could not load Talent.dbc");
    }

    // Load TalentTab.dbc
    auto tabDbc = am->loadDBC("TalentTab.dbc");
    if (tabDbc && tabDbc->isLoaded()) {
        // TalentTab.dbc structure (WoW 3.3.5a):
        // 0: TalentTabID
        // 1-17: Name (16 localized strings + flags = 17 fields)
        // 18: SpellIconID
        // 19: RaceMask
        // 20: ClassMask
        // 21: PetTalentMask
        // 22: OrderIndex
        // 23-39: BackgroundFile (16 localized strings + flags = 17 fields)

        const auto* ttL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("TalentTab") : nullptr;
        uint32_t count = tabDbc->getRecordCount();
        for (uint32_t i = 0; i < count; ++i) {
            TalentTabEntry entry;
            entry.tabId = tabDbc->getUInt32(i, ttL ? (*ttL)["ID"] : 0);
            if (entry.tabId == 0) continue;

            entry.name = tabDbc->getString(i, ttL ? (*ttL)["Name"] : 1);
            entry.classMask = tabDbc->getUInt32(i, ttL ? (*ttL)["ClassMask"] : 20);
            entry.orderIndex = static_cast<uint8_t>(tabDbc->getUInt32(i, ttL ? (*ttL)["OrderIndex"] : 22));
            entry.backgroundFile = tabDbc->getString(i, ttL ? (*ttL)["BackgroundFile"] : 23);

            talentTabCache_[entry.tabId] = entry;

            // Log first few tabs to debug class mask issue
            if (talentTabCache_.size() <= 10) {
                LOG_INFO("  Tab ", entry.tabId, ": ", entry.name, " (classMask=0x", std::hex, entry.classMask, std::dec, ")");
            }
        }
        LOG_INFO("Loaded ", talentTabCache_.size(), " talent tabs from TalentTab.dbc");
    } else {
        LOG_WARNING("Could not load TalentTab.dbc");
    }
}

static const std::string EMPTY_STRING;

const std::string& GameHandler::getSpellName(uint32_t spellId) const {
    auto it = spellNameCache_.find(spellId);
    return (it != spellNameCache_.end()) ? it->second.name : EMPTY_STRING;
}

const std::string& GameHandler::getSpellRank(uint32_t spellId) const {
    auto it = spellNameCache_.find(spellId);
    return (it != spellNameCache_.end()) ? it->second.rank : EMPTY_STRING;
}

const std::string& GameHandler::getSkillLineName(uint32_t spellId) const {
    auto slIt = spellToSkillLine_.find(spellId);
    if (slIt == spellToSkillLine_.end()) return EMPTY_STRING;
    auto nameIt = skillLineNames_.find(slIt->second);
    return (nameIt != skillLineNames_.end()) ? nameIt->second : EMPTY_STRING;
}

// ============================================================
// Single-player local combat
// ============================================================








// ============================================================
// XP tracking
// ============================================================

// WotLK 3.3.5a XP-to-next-level table (from player_xp_for_level)
static const uint32_t XP_TABLE[] = {
    0,       // level 0 (unused)
    400,     900,     1400,    2100,    2800,    3600,    4500,    5400,    6500,    7600,     // 1-10
    8700,    9800,    11000,   12300,   13600,   15000,   16400,   17800,   19300,   20800,    // 11-20
    22400,   24000,   25500,   27200,   28900,   30500,   32200,   33900,   36300,   38800,    // 21-30
    41600,   44600,   48000,   51400,   55000,   58700,   62400,   66200,   70200,   74300,    // 31-40
    78500,   82800,   87100,   91600,   96300,   101000,  105800,  110700,  115700,  120900,   // 41-50
    126100,  131500,  137000,  142500,  148200,  154000,  159900,  165800,  172000,  290000,   // 51-60
    317000,  349000,  386000,  428000,  475000,  527000,  585000,  648000,  717000,  1523800,  // 61-70
    1539600, 1555700, 1571800, 1587900, 1604200, 1620700, 1637400, 1653900, 1670800           // 71-79
};
static constexpr uint32_t XP_TABLE_SIZE = sizeof(XP_TABLE) / sizeof(XP_TABLE[0]);

uint32_t GameHandler::xpForLevel(uint32_t level) {
    if (level == 0 || level >= XP_TABLE_SIZE) return 0;
    return XP_TABLE[level];
}

uint32_t GameHandler::killXp(uint32_t playerLevel, uint32_t victimLevel) {
    if (playerLevel == 0 || victimLevel == 0) return 0;

    // Gray level check (too low = 0 XP)
    int32_t grayLevel;
    if (playerLevel <= 5)        grayLevel = 0;
    else if (playerLevel <= 39)  grayLevel = static_cast<int32_t>(playerLevel) - 5 - static_cast<int32_t>(playerLevel) / 10;
    else if (playerLevel <= 59)  grayLevel = static_cast<int32_t>(playerLevel) - 1 - static_cast<int32_t>(playerLevel) / 5;
    else                         grayLevel = static_cast<int32_t>(playerLevel) - 9;

    if (static_cast<int32_t>(victimLevel) <= grayLevel) return 0;

    // Base XP = 45 + 5 * victimLevel (WoW-like ZeroDifference formula)
    uint32_t baseXp = 45 + 5 * victimLevel;

    // Level difference multiplier
    int32_t diff = static_cast<int32_t>(victimLevel) - static_cast<int32_t>(playerLevel);
    float multiplier = 1.0f + diff * 0.05f;
    if (multiplier < 0.1f) multiplier = 0.1f;
    if (multiplier > 2.0f) multiplier = 2.0f;

    return static_cast<uint32_t>(baseXp * multiplier);
}



void GameHandler::handleXpGain(network::Packet& packet) {
    XpGainData data;
    if (!XpGainParser::parse(packet, data)) return;

    // Server already updates PLAYER_XP via update fields,
    // but we can show combat text for XP gains
    addCombatText(CombatTextEntry::HEAL, static_cast<int32_t>(data.totalXp), 0, true);

    std::string msg = "You gain " + std::to_string(data.totalXp) + " experience.";
    if (data.groupBonus > 0) {
        msg += " (+" + std::to_string(data.groupBonus) + " group bonus)";
    }
    addSystemChatMessage(msg);
}


void GameHandler::addMoneyCopper(uint32_t amount) {
    if (amount == 0) return;
    playerMoneyCopper_ += amount;
    uint32_t gold = amount / 10000;
    uint32_t silver = (amount / 100) % 100;
    uint32_t copper = amount % 100;
    std::string msg = "You receive ";
    msg += std::to_string(gold) + "g ";
    msg += std::to_string(silver) + "s ";
    msg += std::to_string(copper) + "c.";
    addSystemChatMessage(msg);
}

void GameHandler::addSystemChatMessage(const std::string& message) {
    if (message.empty()) return;
    MessageChatData msg;
    msg.type = ChatType::SYSTEM;
    msg.language = ChatLanguage::UNIVERSAL;
    msg.message = message;
    addLocalChatMessage(msg);
}

// ============================================================
// Teleport Handler
// ============================================================

void GameHandler::handleTeleportAck(network::Packet& packet) {
    // MSG_MOVE_TELEPORT_ACK (server→client): packedGuid + u32 counter + u32 time
    // followed by movement info with the new position
    if (packet.getSize() - packet.getReadPos() < 4) {
        LOG_WARNING("MSG_MOVE_TELEPORT_ACK too short");
        return;
    }

    uint64_t guid = UpdateObjectParser::readPackedGuid(packet);
    if (packet.getSize() - packet.getReadPos() < 4) return;
    uint32_t counter = packet.readUInt32();

    // Read the movement info embedded in the teleport
    // Format: u32 flags, u16 flags2, u32 time, float x, float y, float z, float o
    if (packet.getSize() - packet.getReadPos() < 4 + 2 + 4 + 4 * 4) {
        LOG_WARNING("MSG_MOVE_TELEPORT_ACK: not enough data for movement info");
        return;
    }

    packet.readUInt32();  // moveFlags
    packet.readUInt16();  // moveFlags2
    uint32_t moveTime = packet.readUInt32();
    float serverX = packet.readFloat();
    float serverY = packet.readFloat();
    float serverZ = packet.readFloat();
    float orientation = packet.readFloat();

    LOG_INFO("MSG_MOVE_TELEPORT_ACK: guid=0x", std::hex, guid, std::dec,
             " counter=", counter,
             " pos=(", serverX, ", ", serverY, ", ", serverZ, ")");

    // Update our position
    glm::vec3 canonical = core::coords::serverToCanonical(glm::vec3(serverX, serverY, serverZ));
    movementInfo.x = canonical.x;
    movementInfo.y = canonical.y;
    movementInfo.z = canonical.z;
    movementInfo.orientation = core::coords::serverToCanonicalYaw(orientation);
    movementInfo.flags = 0;

    // Send the ack back to the server
    // Client→server MSG_MOVE_TELEPORT_ACK: u64 guid + u32 counter + u32 time
    if (socket && !isClassicLikeExpansion()) {
        network::Packet ack(wireOpcode(Opcode::MSG_MOVE_TELEPORT_ACK));
        const bool legacyGuidAck =
            isActiveExpansion("classic") || isActiveExpansion("tbc") || isActiveExpansion("turtle");
        if (legacyGuidAck) {
            ack.writeUInt64(playerGuid);  // CMaNGOS expects full GUID for teleport ACK
        } else {
            MovementPacket::writePackedGuid(ack, playerGuid);
        }
        ack.writeUInt32(counter);
        ack.writeUInt32(moveTime);
        socket->send(ack);
        LOG_INFO("Sent MSG_MOVE_TELEPORT_ACK response");
    }

    // Notify application of teleport — the callback decides whether to do
    // a full world reload (map change) or just update position (same map).
    if (worldEntryCallback_) {
        worldEntryCallback_(currentMapId_, serverX, serverY, serverZ);
    }
}

void GameHandler::handleNewWorld(network::Packet& packet) {
    // SMSG_NEW_WORLD: uint32 mapId, float x, y, z, orientation
    if (packet.getSize() - packet.getReadPos() < 20) {
        LOG_WARNING("SMSG_NEW_WORLD too short");
        return;
    }

    uint32_t mapId = packet.readUInt32();
    float serverX = packet.readFloat();
    float serverY = packet.readFloat();
    float serverZ = packet.readFloat();
    float orientation = packet.readFloat();

    LOG_INFO("SMSG_NEW_WORLD: mapId=", mapId,
             " pos=(", serverX, ", ", serverY, ", ", serverZ, ")",
             " orient=", orientation);

    // Detect same-map spirit healer resurrection: the server uses SMSG_NEW_WORLD
    // to reposition the player at the graveyard on the same map.  A full world
    // reload is not needed and causes terrain to vanish, making the player fall
    // forever.  Just reposition and send the ack.
    const bool isSameMap       = (mapId == currentMapId_);
    const bool isResurrection  = resurrectPending_;
    if (isSameMap && isResurrection) {
        LOG_INFO("SMSG_NEW_WORLD same-map resurrection — skipping world reload");

        glm::vec3 canonical = core::coords::serverToCanonical(glm::vec3(serverX, serverY, serverZ));
        movementInfo.x = canonical.x;
        movementInfo.y = canonical.y;
        movementInfo.z = canonical.z;
        movementInfo.orientation = core::coords::serverToCanonicalYaw(orientation);
        movementInfo.flags  = 0;
        movementInfo.flags2 = 0;

        resurrectPending_       = false;
        resurrectRequestPending_ = false;
        releasedSpirit_         = false;
        playerDead_             = false;
        repopPending_           = false;
        pendingSpiritHealerGuid_ = 0;
        resurrectCasterGuid_    = 0;
        hostileAttackers_.clear();
        stopAutoAttack();
        tabCycleStale = true;

        if (socket) {
            network::Packet ack(wireOpcode(Opcode::MSG_MOVE_WORLDPORT_ACK));
            socket->send(ack);
            LOG_INFO("Sent MSG_MOVE_WORLDPORT_ACK (resurrection)");
        }
        return;
    }

    currentMapId_ = mapId;

    // Update player position
    glm::vec3 canonical = core::coords::serverToCanonical(glm::vec3(serverX, serverY, serverZ));
    movementInfo.x = canonical.x;
    movementInfo.y = canonical.y;
    movementInfo.z = canonical.z;
    movementInfo.orientation = core::coords::serverToCanonicalYaw(orientation);
    movementInfo.flags = 0;
    movementInfo.flags2 = 0;
    serverMovementAllowed_ = true;
    resurrectPending_ = false;
    resurrectRequestPending_ = false;
    onTaxiFlight_ = false;
    taxiMountActive_ = false;
    taxiActivatePending_ = false;
    taxiClientActive_ = false;
    taxiClientPath_.clear();
    taxiRecoverPending_ = false;
    taxiStartGrace_ = 0.0f;
    currentMountDisplayId_ = 0;
    taxiMountDisplayId_ = 0;
    if (mountCallback_) {
        mountCallback_(0);
    }

    // Clear world state for the new map
    entityManager.clear();
    hostileAttackers_.clear();
    worldStates_.clear();
    worldStateMapId_ = mapId;
    worldStateZoneId_ = 0;
    stopAutoAttack();
    casting = false;
    currentCastSpellId = 0;
    pendingGameObjectInteractGuid_ = 0;
    castTimeRemaining = 0.0f;

    // Send MSG_MOVE_WORLDPORT_ACK to tell the server we're ready
    if (socket) {
        network::Packet ack(wireOpcode(Opcode::MSG_MOVE_WORLDPORT_ACK));
        socket->send(ack);
        LOG_INFO("Sent MSG_MOVE_WORLDPORT_ACK");
    }

    // Reload terrain at new position
    if (worldEntryCallback_) {
        worldEntryCallback_(mapId, serverX, serverY, serverZ);
    }
}

// ============================================================
// Taxi / Flight Path Handlers
// ============================================================

void GameHandler::loadTaxiDbc() {
    if (taxiDbcLoaded_) return;
    taxiDbcLoaded_ = true;

    auto* am = core::Application::getInstance().getAssetManager();
    if (!am || !am->isInitialized()) return;

    auto nodesDbc = am->loadDBC("TaxiNodes.dbc");
    if (nodesDbc && nodesDbc->isLoaded()) {
        const auto* tnL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("TaxiNodes") : nullptr;
        uint32_t fieldCount = nodesDbc->getFieldCount();
        for (uint32_t i = 0; i < nodesDbc->getRecordCount(); i++) {
            TaxiNode node;
            node.id = nodesDbc->getUInt32(i, tnL ? (*tnL)["ID"] : 0);
            node.mapId = nodesDbc->getUInt32(i, tnL ? (*tnL)["MapID"] : 1);
            node.x = nodesDbc->getFloat(i, tnL ? (*tnL)["X"] : 2);
            node.y = nodesDbc->getFloat(i, tnL ? (*tnL)["Y"] : 3);
            node.z = nodesDbc->getFloat(i, tnL ? (*tnL)["Z"] : 4);
            node.name = nodesDbc->getString(i, tnL ? (*tnL)["Name"] : 5);
            const uint32_t mountAllianceField = tnL ? (*tnL)["MountDisplayIdAlliance"] : 22;
            const uint32_t mountHordeField = tnL ? (*tnL)["MountDisplayIdHorde"] : 23;
            const uint32_t mountAllianceFB = tnL ? (*tnL)["MountDisplayIdAllianceFallback"] : 20;
            const uint32_t mountHordeFB = tnL ? (*tnL)["MountDisplayIdHordeFallback"] : 21;
            if (fieldCount > mountHordeField) {
                node.mountDisplayIdAlliance = nodesDbc->getUInt32(i, mountAllianceField);
                node.mountDisplayIdHorde = nodesDbc->getUInt32(i, mountHordeField);
                if (node.mountDisplayIdAlliance == 0 && node.mountDisplayIdHorde == 0 && fieldCount > mountHordeFB) {
                    node.mountDisplayIdAlliance = nodesDbc->getUInt32(i, mountAllianceFB);
                    node.mountDisplayIdHorde = nodesDbc->getUInt32(i, mountHordeFB);
                }
            }
            uint32_t nodeId = node.id;
            if (nodeId > 0) {
                taxiNodes_[nodeId] = std::move(node);
            }
            if (nodeId == 195) {
                std::string fields;
                for (uint32_t f = 0; f < fieldCount; f++) {
                    fields += std::to_string(f) + ":" + std::to_string(nodesDbc->getUInt32(i, f)) + " ";
                }
                LOG_INFO("TaxiNodes[195] fields: ", fields);
            }
        }
        LOG_INFO("Loaded ", taxiNodes_.size(), " taxi nodes from TaxiNodes.dbc");
    } else {
        LOG_WARNING("Could not load TaxiNodes.dbc");
    }

    auto pathDbc = am->loadDBC("TaxiPath.dbc");
    if (pathDbc && pathDbc->isLoaded()) {
        const auto* tpL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("TaxiPath") : nullptr;
        for (uint32_t i = 0; i < pathDbc->getRecordCount(); i++) {
            TaxiPathEdge edge;
            edge.pathId = pathDbc->getUInt32(i, tpL ? (*tpL)["ID"] : 0);
            edge.fromNode = pathDbc->getUInt32(i, tpL ? (*tpL)["FromNode"] : 1);
            edge.toNode = pathDbc->getUInt32(i, tpL ? (*tpL)["ToNode"] : 2);
            edge.cost = pathDbc->getUInt32(i, tpL ? (*tpL)["Cost"] : 3);
            taxiPathEdges_.push_back(edge);
        }
        LOG_INFO("Loaded ", taxiPathEdges_.size(), " taxi path edges from TaxiPath.dbc");
    } else {
        LOG_WARNING("Could not load TaxiPath.dbc");
    }

    auto pathNodeDbc = am->loadDBC("TaxiPathNode.dbc");
    if (pathNodeDbc && pathNodeDbc->isLoaded()) {
        const auto* tpnL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("TaxiPathNode") : nullptr;
        for (uint32_t i = 0; i < pathNodeDbc->getRecordCount(); i++) {
            TaxiPathNode node;
            node.id = pathNodeDbc->getUInt32(i, tpnL ? (*tpnL)["ID"] : 0);
            node.pathId = pathNodeDbc->getUInt32(i, tpnL ? (*tpnL)["PathID"] : 1);
            node.nodeIndex = pathNodeDbc->getUInt32(i, tpnL ? (*tpnL)["NodeIndex"] : 2);
            node.mapId = pathNodeDbc->getUInt32(i, tpnL ? (*tpnL)["MapID"] : 3);
            node.x = pathNodeDbc->getFloat(i, tpnL ? (*tpnL)["X"] : 4);
            node.y = pathNodeDbc->getFloat(i, tpnL ? (*tpnL)["Y"] : 5);
            node.z = pathNodeDbc->getFloat(i, tpnL ? (*tpnL)["Z"] : 6);
            taxiPathNodes_[node.pathId].push_back(node);
        }
        // Sort waypoints by nodeIndex for each path
        for (auto& [pathId, nodes] : taxiPathNodes_) {
            std::sort(nodes.begin(), nodes.end(),
                [](const TaxiPathNode& a, const TaxiPathNode& b) {
                    return a.nodeIndex < b.nodeIndex;
                });
        }
        LOG_INFO("Loaded ", pathNodeDbc->getRecordCount(), " taxi path waypoints from TaxiPathNode.dbc");
    } else {
        LOG_WARNING("Could not load TaxiPathNode.dbc");
    }
}

void GameHandler::handleShowTaxiNodes(network::Packet& packet) {
    ShowTaxiNodesData data;
    if (!ShowTaxiNodesParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_SHOWTAXINODES");
        return;
    }

    loadTaxiDbc();

    // Detect newly discovered flight paths by comparing with stored mask
    if (taxiMaskInitialized_) {
        for (uint32_t i = 0; i < TLK_TAXI_MASK_SIZE; ++i) {
            uint32_t newBits = data.nodeMask[i] & ~knownTaxiMask_[i];
            if (newBits == 0) continue;
            for (uint32_t bit = 0; bit < 32; ++bit) {
                if (newBits & (1u << bit)) {
                    uint32_t nodeId = i * 32 + bit + 1;
                    auto it = taxiNodes_.find(nodeId);
                    if (it != taxiNodes_.end()) {
                        addSystemChatMessage("Discovered flight path: " + it->second.name);
                    }
                }
            }
        }
    }

    // Update stored mask
    for (uint32_t i = 0; i < TLK_TAXI_MASK_SIZE; ++i) {
        knownTaxiMask_[i] = data.nodeMask[i];
    }
    taxiMaskInitialized_ = true;

    currentTaxiData_ = data;
    taxiNpcGuid_ = data.npcGuid;
    taxiWindowOpen_ = true;
    gossipWindowOpen = false;
    buildTaxiCostMap();
    auto it = taxiNodes_.find(data.nearestNode);
    if (it != taxiNodes_.end()) {
        LOG_INFO("Taxi node ", data.nearestNode, " mounts: A=", it->second.mountDisplayIdAlliance,
                 " H=", it->second.mountDisplayIdHorde);
    }
    LOG_INFO("Taxi window opened, nearest node=", data.nearestNode);
}

void GameHandler::applyTaxiMountForCurrentNode() {
    if (taxiMountActive_ || !mountCallback_) return;
    auto it = taxiNodes_.find(currentTaxiData_.nearestNode);
    if (it == taxiNodes_.end()) {
        // Node not in DBC (custom server nodes, missing data) — use hardcoded fallback.
        bool isAlliance = true;
        switch (playerRace_) {
            case Race::ORC: case Race::UNDEAD: case Race::TAUREN: case Race::TROLL:
            case Race::GOBLIN: case Race::BLOOD_ELF:
                isAlliance = false; break;
            default: break;
        }
        uint32_t mountId = isAlliance ? 1210u : 1310u;
        taxiMountDisplayId_ = mountId;
        taxiMountActive_ = true;
        LOG_INFO("Taxi mount fallback (node ", currentTaxiData_.nearestNode, " not in DBC): displayId=", mountId);
        mountCallback_(mountId);
        return;
    }

    bool isAlliance = true;
    switch (playerRace_) {
        case Race::ORC:
        case Race::UNDEAD:
        case Race::TAUREN:
        case Race::TROLL:
        case Race::GOBLIN:
        case Race::BLOOD_ELF:
            isAlliance = false;
            break;
        default:
            isAlliance = true;
            break;
    }
    uint32_t mountId = isAlliance ? it->second.mountDisplayIdAlliance
                                  : it->second.mountDisplayIdHorde;
    if (mountId == 541) mountId = 0;  // Placeholder/invalid in some DBC sets
    if (mountId == 0) {
        mountId = isAlliance ? it->second.mountDisplayIdHorde
                             : it->second.mountDisplayIdAlliance;
        if (mountId == 541) mountId = 0;
    }
    if (mountId == 0) {
        auto& app = core::Application::getInstance();
        uint32_t gryphonId = app.getGryphonDisplayId();
        uint32_t wyvernId = app.getWyvernDisplayId();
        if (isAlliance && gryphonId != 0) mountId = gryphonId;
        if (!isAlliance && wyvernId != 0) mountId = wyvernId;
        if (mountId == 0) {
            mountId = (isAlliance ? wyvernId : gryphonId);
        }
    }
    if (mountId == 0) {
        // Fallback: any non-zero mount display from the node.
        if (it->second.mountDisplayIdAlliance != 0) mountId = it->second.mountDisplayIdAlliance;
        else if (it->second.mountDisplayIdHorde != 0) mountId = it->second.mountDisplayIdHorde;
    }
    if (mountId == 0) {
        // 3.3.5a fallback display IDs (real CreatureDisplayInfo entries).
        // Alliance taxi gryphons commonly use 1210-1213.
        // Horde taxi wyverns commonly use 1310-1312.
        static const uint32_t kAllianceTaxiDisplays[] = {1210u, 1211u, 1212u, 1213u};
        static const uint32_t kHordeTaxiDisplays[] = {1310u, 1311u, 1312u};
        mountId = isAlliance ? kAllianceTaxiDisplays[0] : kHordeTaxiDisplays[0];
    }

    // Last resort legacy fallback.
    if (mountId == 0) {
        mountId = isAlliance ? 30412u : 30413u;
    }
    if (mountId != 0) {
        taxiMountDisplayId_ = mountId;
        taxiMountActive_ = true;
        LOG_INFO("Taxi mount apply: displayId=", mountId);
        mountCallback_(mountId);
    }
}

void GameHandler::startClientTaxiPath(const std::vector<uint32_t>& pathNodes) {
    taxiClientPath_.clear();
    taxiClientIndex_ = 0;
    taxiClientActive_ = false;
    taxiClientSegmentProgress_ = 0.0f;

    // Build full spline path using TaxiPathNode waypoints (not just node positions)
    for (size_t i = 0; i + 1 < pathNodes.size(); i++) {
        uint32_t fromNode = pathNodes[i];
        uint32_t toNode = pathNodes[i + 1];
        // Find the pathId connecting these nodes
        uint32_t pathId = 0;
        for (const auto& edge : taxiPathEdges_) {
            if (edge.fromNode == fromNode && edge.toNode == toNode) {
                pathId = edge.pathId;
                break;
            }
        }
        if (pathId == 0) {
            LOG_WARNING("No taxi path found from node ", fromNode, " to ", toNode);
            continue;
        }
        // Get spline waypoints for this path segment
        auto pathIt = taxiPathNodes_.find(pathId);
        if (pathIt != taxiPathNodes_.end()) {
            for (const auto& wpNode : pathIt->second) {
                glm::vec3 serverPos(wpNode.x, wpNode.y, wpNode.z);
                glm::vec3 canonical = core::coords::serverToCanonical(serverPos);
                taxiClientPath_.push_back(canonical);
            }
        } else {
            LOG_WARNING("No spline waypoints found for taxi pathId ", pathId);
        }
    }

    if (taxiClientPath_.size() < 2) {
        // Fallback: use TaxiNodes directly when TaxiPathNode spline data is missing.
        taxiClientPath_.clear();
        for (uint32_t nodeId : pathNodes) {
            auto nodeIt = taxiNodes_.find(nodeId);
            if (nodeIt == taxiNodes_.end()) continue;
            glm::vec3 serverPos(nodeIt->second.x, nodeIt->second.y, nodeIt->second.z);
            taxiClientPath_.push_back(core::coords::serverToCanonical(serverPos));
        }
    }

    if (taxiClientPath_.size() < 2) {
        LOG_WARNING("Taxi path too short: ", taxiClientPath_.size(), " waypoints");
        return;
    }

    // Set initial orientation to face the first non-degenerate flight segment.
    glm::vec3 start = taxiClientPath_[0];
    glm::vec3 dir(0.0f);
    float dirLen = 0.0f;
    for (size_t i = 1; i < taxiClientPath_.size(); i++) {
        dir = taxiClientPath_[i] - start;
        dirLen = glm::length(dir);
        if (dirLen >= 0.001f) {
            break;
        }
    }

    float initialOrientation = movementInfo.orientation;
    float initialRenderYaw = movementInfo.orientation;
    float initialPitch = 0.0f;
    float initialRoll = 0.0f;
    if (dirLen >= 0.001f) {
        initialOrientation = std::atan2(dir.y, dir.x);
        glm::vec3 renderDir = core::coords::canonicalToRender(dir);
        initialRenderYaw = std::atan2(renderDir.y, renderDir.x);
        glm::vec3 dirNorm = dir / dirLen;
        initialPitch = std::asin(std::clamp(dirNorm.z, -1.0f, 1.0f));
    }

    movementInfo.x = start.x;
    movementInfo.y = start.y;
    movementInfo.z = start.z;
    movementInfo.orientation = initialOrientation;
    sanitizeMovementForTaxi();

    auto playerEntity = entityManager.getEntity(playerGuid);
    if (playerEntity) {
        playerEntity->setPosition(start.x, start.y, start.z, initialOrientation);
    }

    if (taxiOrientationCallback_) {
        taxiOrientationCallback_(initialRenderYaw, initialPitch, initialRoll);
    }

    LOG_INFO("Taxi flight started with ", taxiClientPath_.size(), " spline waypoints");
    taxiClientActive_ = true;
}

void GameHandler::updateClientTaxi(float deltaTime) {
    if (!taxiClientActive_ || taxiClientPath_.size() < 2) return;
    auto playerEntity = entityManager.getEntity(playerGuid);

    auto finishTaxiFlight = [&]() {
            // Snap player to the last waypoint (landing position) before clearing state.
            // Without this, the player would be left at whatever mid-flight position
            // they were at when the path completion was detected.
            if (!taxiClientPath_.empty()) {
                const auto& landingPos = taxiClientPath_.back();
                if (playerEntity) {
                    playerEntity->setPosition(landingPos.x, landingPos.y, landingPos.z,
                                              movementInfo.orientation);
                }
                movementInfo.x = landingPos.x;
                movementInfo.y = landingPos.y;
                movementInfo.z = landingPos.z;
                LOG_INFO("Taxi landing: snapped to final waypoint (",
                         landingPos.x, ", ", landingPos.y, ", ", landingPos.z, ")");
            }
            taxiClientActive_ = false;
            onTaxiFlight_ = false;
            taxiLandingCooldown_ = 2.0f;  // 2 second cooldown to prevent re-entering
            if (taxiMountActive_ && mountCallback_) {
                mountCallback_(0);
            }
            taxiMountActive_ = false;
            taxiMountDisplayId_ = 0;
            currentMountDisplayId_ = 0;
            taxiClientPath_.clear();
            taxiRecoverPending_ = false;
            movementInfo.flags = 0;
            movementInfo.flags2 = 0;
            if (socket) {
                sendMovement(Opcode::MSG_MOVE_STOP);
                sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
            }
            LOG_INFO("Taxi flight landed (client path)");
    };

    if (taxiClientIndex_ + 1 >= taxiClientPath_.size()) {
        finishTaxiFlight();
        return;
    }

    float remainingDistance = taxiClientSegmentProgress_ + (taxiClientSpeed_ * deltaTime);
    glm::vec3 start(0.0f);
    glm::vec3 end(0.0f);
    glm::vec3 dir(0.0f);
    float segmentLen = 0.0f;
    float t = 0.0f;

    // Consume as many tiny/finished segments as needed this frame so taxi doesn't stall
    // on dense/degenerate node clusters near takeoff/landing.
    while (true) {
        if (taxiClientIndex_ + 1 >= taxiClientPath_.size()) {
            finishTaxiFlight();
            return;
        }

        start = taxiClientPath_[taxiClientIndex_];
        end = taxiClientPath_[taxiClientIndex_ + 1];
        dir = end - start;
        segmentLen = glm::length(dir);

        if (segmentLen < 0.01f) {
            taxiClientIndex_++;
            continue;
        }

        if (remainingDistance >= segmentLen) {
            remainingDistance -= segmentLen;
            taxiClientIndex_++;
            taxiClientSegmentProgress_ = 0.0f;
            continue;
        }

        taxiClientSegmentProgress_ = remainingDistance;
        t = taxiClientSegmentProgress_ / segmentLen;
        break;
    }

    // Use Catmull-Rom spline for smooth interpolation between waypoints
    // Get surrounding points for spline curve
    glm::vec3 p0 = (taxiClientIndex_ > 0) ? taxiClientPath_[taxiClientIndex_ - 1] : start;
    glm::vec3 p1 = start;
    glm::vec3 p2 = end;
    glm::vec3 p3 = (taxiClientIndex_ + 2 < taxiClientPath_.size()) ?
                   taxiClientPath_[taxiClientIndex_ + 2] : end;

    // Catmull-Rom spline formula for smooth curves
    float t2 = t * t;
    float t3 = t2 * t;
    glm::vec3 nextPos = 0.5f * (
        (2.0f * p1) +
        (-p0 + p2) * t +
        (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
        (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3
    );

    // Calculate smooth direction for orientation (tangent to spline)
    glm::vec3 tangent = 0.5f * (
        (-p0 + p2) +
        2.0f * (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t +
        3.0f * (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t2
    );
    float tangentLen = glm::length(tangent);
    if (tangentLen < 0.0001f) {
        tangent = dir;
        tangentLen = glm::length(tangent);
        if (tangentLen < 0.0001f) {
            tangent = glm::vec3(std::cos(movementInfo.orientation), std::sin(movementInfo.orientation), 0.0f);
            tangentLen = glm::length(tangent);
        }
    }

    // Calculate yaw from horizontal direction
    float targetOrientation = std::atan2(tangent.y, tangent.x);

    // Calculate pitch from vertical component (altitude change)
    glm::vec3 tangentNorm = tangent / std::max(tangentLen, 0.0001f);
    float pitch = std::asin(std::clamp(tangentNorm.z, -1.0f, 1.0f));

    // Calculate roll (banking) from rate of yaw change
    float currentOrientation = movementInfo.orientation;
    float orientDiff = targetOrientation - currentOrientation;
    // Normalize angle difference to [-PI, PI]
    while (orientDiff > 3.14159265f) orientDiff -= 6.28318530f;
    while (orientDiff < -3.14159265f) orientDiff += 6.28318530f;
    // Bank proportional to turn rate (scaled for visual effect)
    float roll = -orientDiff * 2.5f;
    roll = std::clamp(roll, -0.7f, 0.7f);  // Limit to ~40 degrees

    // Smooth rotation transition (lerp towards target)
    float smoothOrientation = currentOrientation + orientDiff * std::min(1.0f, deltaTime * 3.0f);

    if (playerEntity) {
        playerEntity->setPosition(nextPos.x, nextPos.y, nextPos.z, smoothOrientation);
    }
    movementInfo.x = nextPos.x;
    movementInfo.y = nextPos.y;
    movementInfo.z = nextPos.z;
    movementInfo.orientation = smoothOrientation;

    // Update mount rotation with yaw/pitch/roll. Use render-space tangent yaw to
    // avoid canonical<->render convention mismatches.
    if (taxiOrientationCallback_) {
        glm::vec3 renderTangent = core::coords::canonicalToRender(tangent);
        float renderYaw = std::atan2(renderTangent.y, renderTangent.x);
        taxiOrientationCallback_(renderYaw, pitch, roll);
    }
}

void GameHandler::handleActivateTaxiReply(network::Packet& packet) {
    ActivateTaxiReplyData data;
    if (!ActivateTaxiReplyParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_ACTIVATETAXIREPLY");
        return;
    }

    // Guard against stray/mis-mapped packets being treated as taxi replies.
    // We only consume a reply while an activation request is pending.
    if (!taxiActivatePending_) {
        LOG_DEBUG("Ignoring stray taxi reply: result=", data.result);
        return;
    }

    if (data.result == 0) {
        // Some cores can emit duplicate success replies (e.g. basic + express activate).
        // Ignore repeats once taxi is already active and no activation is pending.
        if (onTaxiFlight_ && !taxiActivatePending_) {
            return;
        }
        onTaxiFlight_ = true;
        taxiStartGrace_ = std::max(taxiStartGrace_, 2.0f);
        sanitizeMovementForTaxi();
        taxiWindowOpen_ = false;
        taxiActivatePending_ = false;
        taxiActivateTimer_ = 0.0f;
        applyTaxiMountForCurrentNode();
        if (socket) {
            sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
        }
        LOG_INFO("Taxi flight started!");
    } else {
        // If local taxi motion already started, treat late failure as stale and ignore.
        if (onTaxiFlight_ || taxiClientActive_) {
            LOG_WARNING("Ignoring stale taxi failure reply while flight is active: result=", data.result);
            taxiActivatePending_ = false;
            taxiActivateTimer_ = 0.0f;
            return;
        }
        LOG_WARNING("Taxi activation failed, result=", data.result);
        addSystemChatMessage("Cannot take that flight path.");
        taxiActivatePending_ = false;
        taxiActivateTimer_ = 0.0f;
        if (taxiMountActive_ && mountCallback_) {
            mountCallback_(0);
        }
        taxiMountActive_ = false;
        taxiMountDisplayId_ = 0;
        onTaxiFlight_ = false;
    }
}

void GameHandler::closeTaxi() {
    taxiWindowOpen_ = false;

    // Closing the taxi UI must not cancel an active/pending flight.
    // The window can auto-close due distance checks while takeoff begins.
    if (taxiActivatePending_ || onTaxiFlight_ || taxiClientActive_) {
        return;
    }

    // If we optimistically mounted during node selection, dismount now
    if (taxiMountActive_ && mountCallback_) {
        mountCallback_(0);  // Dismount
    }
    taxiMountActive_ = false;
    taxiMountDisplayId_ = 0;

    // Clear any pending activation
    taxiActivatePending_ = false;
    onTaxiFlight_ = false;

    // Set cooldown to prevent auto-mount trigger from re-applying taxi mount
    // (The UNIT_FLAG_TAXI_FLIGHT check in handleUpdateObject won't re-trigger during cooldown)
    taxiLandingCooldown_ = 2.0f;
}

void GameHandler::buildTaxiCostMap() {
    taxiCostMap_.clear();
    uint32_t startNode = currentTaxiData_.nearestNode;
    if (startNode == 0) return;

    // Build adjacency list with costs from all edges (path may traverse unknown nodes)
    struct AdjEntry { uint32_t node; uint32_t cost; };
    std::unordered_map<uint32_t, std::vector<AdjEntry>> adj;
    for (const auto& edge : taxiPathEdges_) {
        adj[edge.fromNode].push_back({edge.toNode, edge.cost});
    }

    // BFS from startNode, accumulating costs along the path
    std::deque<uint32_t> queue;
    queue.push_back(startNode);
    taxiCostMap_[startNode] = 0;

    while (!queue.empty()) {
        uint32_t cur = queue.front();
        queue.pop_front();
        for (const auto& next : adj[cur]) {
            if (taxiCostMap_.find(next.node) == taxiCostMap_.end()) {
                taxiCostMap_[next.node] = taxiCostMap_[cur] + next.cost;
                queue.push_back(next.node);
            }
        }
    }
}

uint32_t GameHandler::getTaxiCostTo(uint32_t destNodeId) const {
    auto it = taxiCostMap_.find(destNodeId);
    return (it != taxiCostMap_.end()) ? it->second : 0;
}

void GameHandler::activateTaxi(uint32_t destNodeId) {
    if (!socket || state != WorldState::IN_WORLD) return;

    // One-shot taxi activation until server replies or timeout.
    if (taxiActivatePending_ || onTaxiFlight_) {
        return;
    }

    uint32_t startNode = currentTaxiData_.nearestNode;
    if (startNode == 0 || destNodeId == 0 || startNode == destNodeId) return;

    // If already mounted, dismount before starting a taxi flight.
    if (isMounted()) {
        LOG_INFO("Taxi activate: dismounting current mount");
        if (mountCallback_) mountCallback_(0);
        currentMountDisplayId_ = 0;
        dismount();
    }

    addSystemChatMessage("Taxi: requesting flight...");

    // BFS to find path from startNode to destNodeId
    std::unordered_map<uint32_t, std::vector<uint32_t>> adj;
    for (const auto& edge : taxiPathEdges_) {
        adj[edge.fromNode].push_back(edge.toNode);
    }

    std::unordered_map<uint32_t, uint32_t> parent;
    std::deque<uint32_t> queue;
    queue.push_back(startNode);
    parent[startNode] = startNode;

    bool found = false;
    while (!queue.empty()) {
        uint32_t cur = queue.front();
        queue.pop_front();
        if (cur == destNodeId) { found = true; break; }
        for (uint32_t next : adj[cur]) {
            if (parent.find(next) == parent.end()) {
                parent[next] = cur;
                queue.push_back(next);
            }
        }
    }

    if (!found) {
        LOG_WARNING("No taxi path found from node ", startNode, " to ", destNodeId);
        addSystemChatMessage("No flight path available to that destination.");
        return;
    }

    std::vector<uint32_t> path;
    for (uint32_t n = destNodeId; n != startNode; n = parent[n]) {
        path.push_back(n);
    }
    path.push_back(startNode);
    std::reverse(path.begin(), path.end());

    LOG_INFO("Taxi path: ", path.size(), " nodes, from ", startNode, " to ", destNodeId);

    LOG_INFO("Taxi activate: npc=0x", std::hex, taxiNpcGuid_, std::dec,
             " start=", startNode, " dest=", destNodeId, " pathLen=", path.size());
    if (!path.empty()) {
        std::string pathStr;
        for (size_t i = 0; i < path.size(); i++) {
            pathStr += std::to_string(path[i]);
            if (i + 1 < path.size()) pathStr += "->";
        }
        LOG_INFO("Taxi path nodes: ", pathStr);
    }

    uint32_t totalCost = getTaxiCostTo(destNodeId);
    LOG_INFO("Taxi activate: start=", startNode, " dest=", destNodeId, " cost=", totalCost);

    // Some servers only accept basic CMSG_ACTIVATETAXI.
    auto basicPkt = ActivateTaxiPacket::build(taxiNpcGuid_, startNode, destNodeId);
    socket->send(basicPkt);

    // AzerothCore in this setup rejects/misparses CMSG_ACTIVATETAXIEXPRESS (0x312),
    // so keep taxi activation on the basic packet only.

    // Optimistically start taxi visuals; server will correct if it denies.
    taxiWindowOpen_ = false;
    taxiActivatePending_ = true;
    taxiActivateTimer_ = 0.0f;
    taxiStartGrace_ = 2.0f;
    if (!onTaxiFlight_) {
        onTaxiFlight_ = true;
        sanitizeMovementForTaxi();
        applyTaxiMountForCurrentNode();
    }
    if (socket) {
        sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
    }

    // Trigger terrain precache immediately (non-blocking).
    if (taxiPrecacheCallback_) {
        std::vector<glm::vec3> previewPath;
        // Build full spline path using TaxiPathNode waypoints
        for (size_t i = 0; i + 1 < path.size(); i++) {
            uint32_t fromNode = path[i];
            uint32_t toNode = path[i + 1];
            // Find the pathId connecting these nodes
            uint32_t pathId = 0;
            for (const auto& edge : taxiPathEdges_) {
                if (edge.fromNode == fromNode && edge.toNode == toNode) {
                    pathId = edge.pathId;
                    break;
                }
            }
            if (pathId == 0) continue;
            // Get spline waypoints for this path segment
            auto pathIt = taxiPathNodes_.find(pathId);
            if (pathIt != taxiPathNodes_.end()) {
                for (const auto& wpNode : pathIt->second) {
                    glm::vec3 serverPos(wpNode.x, wpNode.y, wpNode.z);
                    glm::vec3 canonical = core::coords::serverToCanonical(serverPos);
                    previewPath.push_back(canonical);
                }
            }
        }
        if (previewPath.size() >= 2) {
            taxiPrecacheCallback_(previewPath);
        }
    }

    // Flight starts immediately; upload callback stays opportunistic/non-blocking.
    if (taxiFlightStartCallback_) {
        taxiFlightStartCallback_();
    }
    startClientTaxiPath(path);
    // We run taxi movement locally immediately; don't keep a long-lived pending state.
    if (taxiClientActive_) {
        taxiActivatePending_ = false;
        taxiActivateTimer_ = 0.0f;
    }

    addSystemChatMessage("Flight started.");

    // Save recovery target in case of disconnect during taxi.
    auto destIt = taxiNodes_.find(destNodeId);
    if (destIt != taxiNodes_.end()) {
        taxiRecoverMapId_ = destIt->second.mapId;
        taxiRecoverPos_ = core::coords::serverToCanonical(
            glm::vec3(destIt->second.x, destIt->second.y, destIt->second.z));
        taxiRecoverPending_ = false;
    }
}

// ============================================================
// Server Info Command Handlers
// ============================================================

void GameHandler::handleQueryTimeResponse(network::Packet& packet) {
    QueryTimeResponseData data;
    if (!QueryTimeResponseParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_QUERY_TIME_RESPONSE");
        return;
    }

    // Convert Unix timestamp to readable format
    time_t serverTime = static_cast<time_t>(data.serverTime);
    struct tm* timeInfo = localtime(&serverTime);
    char timeStr[64];
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", timeInfo);

    std::string msg = "Server time: " + std::string(timeStr);
    addSystemChatMessage(msg);
    LOG_INFO("Server time: ", data.serverTime, " (", timeStr, ")");
}

void GameHandler::handlePlayedTime(network::Packet& packet) {
    PlayedTimeData data;
    if (!PlayedTimeParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_PLAYED_TIME");
        return;
    }

    if (data.triggerMessage) {
        // Format total time played
        uint32_t totalDays = data.totalTimePlayed / 86400;
        uint32_t totalHours = (data.totalTimePlayed % 86400) / 3600;
        uint32_t totalMinutes = (data.totalTimePlayed % 3600) / 60;

        // Format level time played
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

        addSystemChatMessage(totalMsg);
        addSystemChatMessage(levelMsg);
    }

    LOG_INFO("Played time: total=", data.totalTimePlayed, "s, level=", data.levelTimePlayed, "s");
}

void GameHandler::handleWho(network::Packet& packet) {
    // Parse WHO response
    uint32_t displayCount = packet.readUInt32();
    uint32_t onlineCount = packet.readUInt32();

    LOG_INFO("WHO response: ", displayCount, " players displayed, ", onlineCount, " total online");

    if (displayCount == 0) {
        addSystemChatMessage("No players found.");
        return;
    }

    addSystemChatMessage(std::to_string(onlineCount) + " player(s) online:");

    for (uint32_t i = 0; i < displayCount; ++i) {
        std::string playerName = packet.readString();
        std::string guildName = packet.readString();
        uint32_t level = packet.readUInt32();
        uint32_t classId = packet.readUInt32();
        uint32_t raceId = packet.readUInt32();
        packet.readUInt8();   // gender (unused)
        packet.readUInt32();  // zoneId (unused)

        std::string msg = "  " + playerName;
        if (!guildName.empty()) {
            msg += " <" + guildName + ">";
        }
        msg += " - Level " + std::to_string(level);

        addSystemChatMessage(msg);
        LOG_INFO("  ", playerName, " (", guildName, ") Lv", level, " Class:", classId, " Race:", raceId);
    }
}

void GameHandler::handleFriendStatus(network::Packet& packet) {
    FriendStatusData data;
    if (!FriendStatusParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_FRIEND_STATUS");
        return;
    }

    // Look up player name from GUID
    std::string playerName;
    auto it = playerNameCache.find(data.guid);
    if (it != playerNameCache.end()) {
        playerName = it->second;
    } else {
        playerName = "Unknown";
    }

    // Update friends cache
    if (data.status == 1 || data.status == 2) {  // Added or online
        friendsCache[playerName] = data.guid;
    } else if (data.status == 0) {  // Removed
        friendsCache.erase(playerName);
    }

    // Status messages
    switch (data.status) {
        case 0:
            addSystemChatMessage(playerName + " has been removed from your friends list.");
            break;
        case 1:
            addSystemChatMessage(playerName + " has been added to your friends list.");
            break;
        case 2:
            addSystemChatMessage(playerName + " is now online.");
            break;
        case 3:
            addSystemChatMessage(playerName + " is now offline.");
            break;
        case 4:
            addSystemChatMessage("Player not found.");
            break;
        case 5:
            addSystemChatMessage(playerName + " is already in your friends list.");
            break;
        case 6:
            addSystemChatMessage("Your friends list is full.");
            break;
        case 7:
            addSystemChatMessage(playerName + " is ignoring you.");
            break;
        default:
            LOG_INFO("Friend status: ", (int)data.status, " for ", playerName);
            break;
    }

    LOG_INFO("Friend status update: ", playerName, " status=", (int)data.status);
}

void GameHandler::handleRandomRoll(network::Packet& packet) {
    RandomRollData data;
    if (!RandomRollParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_RANDOM_ROLL");
        return;
    }

    // Get roller name
    std::string rollerName;
    if (data.rollerGuid == playerGuid) {
        rollerName = "You";
    } else {
        auto it = playerNameCache.find(data.rollerGuid);
        if (it != playerNameCache.end()) {
            rollerName = it->second;
        } else {
            rollerName = "Someone";
        }
    }

    // Build message
    std::string msg = rollerName;
    if (data.rollerGuid == playerGuid) {
        msg += " roll ";
    } else {
        msg += " rolls ";
    }
    msg += std::to_string(data.result);
    msg += " (" + std::to_string(data.minRoll) + "-" + std::to_string(data.maxRoll) + ")";

    addSystemChatMessage(msg);
    LOG_INFO("Random roll: ", rollerName, " rolled ", data.result, " (", data.minRoll, "-", data.maxRoll, ")");
}

void GameHandler::handleLogoutResponse(network::Packet& packet) {
    LogoutResponseData data;
    if (!LogoutResponseParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_LOGOUT_RESPONSE");
        return;
    }

    if (data.result == 0) {
        // Success - logout initiated
        if (data.instant) {
            addSystemChatMessage("Logging out...");
        } else {
            addSystemChatMessage("Logging out in 20 seconds...");
        }
        LOG_INFO("Logout response: success, instant=", (int)data.instant);
    } else {
        // Failure
        addSystemChatMessage("Cannot logout right now.");
        loggingOut_ = false;
        LOG_WARNING("Logout failed, result=", data.result);
    }
}

void GameHandler::handleLogoutComplete(network::Packet& /*packet*/) {
    addSystemChatMessage("Logout complete.");
    loggingOut_ = false;
    LOG_INFO("Logout complete");
    // Server will disconnect us
}

uint32_t GameHandler::generateClientSeed() {
    // Generate cryptographically random seed
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint32_t> dis(1, 0xFFFFFFFF);
    return dis(gen);
}

void GameHandler::setState(WorldState newState) {
    if (state != newState) {
        LOG_DEBUG("World state: ", (int)state, " -> ", (int)newState);
        state = newState;
    }
}

void GameHandler::fail(const std::string& reason) {
    LOG_ERROR("World connection failed: ", reason);
    setState(WorldState::FAILED);

    if (onFailure) {
        onFailure(reason);
    }
}


// ============================================================
// Player Skills
// ============================================================

static const std::string kEmptySkillName;

const std::string& GameHandler::getSkillName(uint32_t skillId) const {
    auto it = skillLineNames_.find(skillId);
    return (it != skillLineNames_.end()) ? it->second : kEmptySkillName;
}

uint32_t GameHandler::getSkillCategory(uint32_t skillId) const {
    auto it = skillLineCategories_.find(skillId);
    return (it != skillLineCategories_.end()) ? it->second : 0;
}

void GameHandler::loadSkillLineDbc() {
    if (skillLineDbcLoaded_) return;
    skillLineDbcLoaded_ = true;

    auto* am = core::Application::getInstance().getAssetManager();
    if (!am || !am->isInitialized()) return;

    auto dbc = am->loadDBC("SkillLine.dbc");
    if (!dbc || !dbc->isLoaded()) {
        LOG_WARNING("GameHandler: Could not load SkillLine.dbc");
        return;
    }

    const auto* slL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("SkillLine") : nullptr;
    for (uint32_t i = 0; i < dbc->getRecordCount(); i++) {
        uint32_t id = dbc->getUInt32(i, slL ? (*slL)["ID"] : 0);
        uint32_t category = dbc->getUInt32(i, slL ? (*slL)["Category"] : 1);
        std::string name = dbc->getString(i, slL ? (*slL)["Name"] : 3);
        if (id > 0 && !name.empty()) {
            skillLineNames_[id] = name;
            skillLineCategories_[id] = category;
        }
    }
    LOG_INFO("GameHandler: Loaded ", skillLineNames_.size(), " skill line names");
}

void GameHandler::extractSkillFields(const std::map<uint16_t, uint32_t>& fields) {
    loadSkillLineDbc();

    const uint16_t PLAYER_SKILL_INFO_START = fieldIndex(UF::PLAYER_SKILL_INFO_START);
    static constexpr int MAX_SKILL_SLOTS = 128;

    std::map<uint32_t, PlayerSkill> newSkills;

    for (int slot = 0; slot < MAX_SKILL_SLOTS; slot++) {
        uint16_t baseField = PLAYER_SKILL_INFO_START + slot * 3;

        auto idIt = fields.find(baseField);
        if (idIt == fields.end()) continue;

        uint32_t raw0 = idIt->second;
        uint16_t skillId = raw0 & 0xFFFF;
        if (skillId == 0) continue;

        auto valIt = fields.find(baseField + 1);
        if (valIt == fields.end()) continue;

        uint32_t raw1 = valIt->second;
        uint16_t value = raw1 & 0xFFFF;
        uint16_t maxValue = (raw1 >> 16) & 0xFFFF;

        PlayerSkill skill;
        skill.skillId = skillId;
        skill.value = value;
        skill.maxValue = maxValue;
        newSkills[skillId] = skill;
    }

    // Detect increases and emit chat messages
    for (const auto& [skillId, skill] : newSkills) {
        if (skill.value == 0) continue;
        auto oldIt = playerSkills_.find(skillId);
        if (oldIt != playerSkills_.end() && skill.value > oldIt->second.value) {
            // Filter out racial, generic, and hidden skills from announcements
            // Category 5 = Attributes (Defense, etc.)
            // Category 10 = Languages (Orcish, Common, etc.)
            // Category 12 = Not Displayed (generic/hidden)
            auto catIt = skillLineCategories_.find(skillId);
            if (catIt != skillLineCategories_.end()) {
                uint32_t category = catIt->second;
                if (category == 5 || category == 10 || category == 12) {
                    continue; // Skip announcement for racial/generic skills
                }
            }

            const std::string& name = getSkillName(skillId);
            std::string skillName = name.empty() ? ("Skill #" + std::to_string(skillId)) : name;
            addSystemChatMessage("Your skill in " + skillName + " has increased to " + std::to_string(skill.value) + ".");
        }
    }

    playerSkills_ = std::move(newSkills);
}

void GameHandler::extractExploredZoneFields(const std::map<uint16_t, uint32_t>& fields) {
    if (playerExploredZones_.size() != PLAYER_EXPLORED_ZONES_COUNT) {
        playerExploredZones_.assign(PLAYER_EXPLORED_ZONES_COUNT, 0u);
    }

    bool foundAny = false;
    for (size_t i = 0; i < PLAYER_EXPLORED_ZONES_COUNT; i++) {
        const uint16_t fieldIdx = static_cast<uint16_t>(fieldIndex(UF::PLAYER_EXPLORED_ZONES_START) + i);
        auto it = fields.find(fieldIdx);
        if (it == fields.end()) continue;
        playerExploredZones_[i] = it->second;
        foundAny = true;
    }

    if (foundAny) {
        hasPlayerExploredZones_ = true;
    }
}

std::string GameHandler::getCharacterConfigDir() {
    std::string dir;
#ifdef _WIN32
    const char* appdata = std::getenv("APPDATA");
    dir = appdata ? std::string(appdata) + "\\wowee\\characters" : "characters";
#else
    const char* home = std::getenv("HOME");
    dir = home ? std::string(home) + "/.wowee/characters" : "characters";
#endif
    return dir;
}

void GameHandler::saveCharacterConfig() {
    const Character* ch = getActiveCharacter();
    if (!ch || ch->name.empty()) return;

    std::string dir = getCharacterConfigDir();
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);

    std::string path = dir + "/" + ch->name + ".cfg";
    std::ofstream out(path);
    if (!out.is_open()) {
        LOG_WARNING("Could not save character config to ", path);
        return;
    }

    out << "character_guid=" << playerGuid << "\n";
    out << "gender=" << static_cast<int>(ch->gender) << "\n";
    // For male/female, derive from gender; only nonbinary has a meaningful separate choice
    bool saveUseFemaleModel = (ch->gender == Gender::NONBINARY) ? ch->useFemaleModel
                                                                 : (ch->gender == Gender::FEMALE);
    out << "use_female_model=" << (saveUseFemaleModel ? 1 : 0) << "\n";
    for (int i = 0; i < ACTION_BAR_SLOTS; i++) {
        out << "action_bar_" << i << "_type=" << static_cast<int>(actionBar[i].type) << "\n";
        out << "action_bar_" << i << "_id=" << actionBar[i].id << "\n";
    }

    // Save quest log
    out << "quest_log_count=" << questLog_.size() << "\n";
    for (size_t i = 0; i < questLog_.size(); i++) {
        const auto& quest = questLog_[i];
        out << "quest_" << i << "_id=" << quest.questId << "\n";
        out << "quest_" << i << "_title=" << quest.title << "\n";
        out << "quest_" << i << "_complete=" << (quest.complete ? 1 : 0) << "\n";
    }

    LOG_INFO("Character config saved to ", path);
}

void GameHandler::loadCharacterConfig() {
    const Character* ch = getActiveCharacter();
    if (!ch || ch->name.empty()) return;

    std::string path = getCharacterConfigDir() + "/" + ch->name + ".cfg";
    std::ifstream in(path);
    if (!in.is_open()) return;

    uint64_t savedGuid = 0;
    std::array<int, ACTION_BAR_SLOTS> types{};
    std::array<uint32_t, ACTION_BAR_SLOTS> ids{};
    bool hasSlots = false;
    int savedGender = -1;
    int savedUseFemaleModel = -1;

    std::string line;
    while (std::getline(in, line)) {
        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq + 1);

        if (key == "character_guid") {
            try { savedGuid = std::stoull(val); } catch (...) {}
        } else if (key == "gender") {
            try { savedGender = std::stoi(val); } catch (...) {}
        } else if (key == "use_female_model") {
            try { savedUseFemaleModel = std::stoi(val); } catch (...) {}
        } else if (key.rfind("action_bar_", 0) == 0) {
            // Parse action_bar_N_type or action_bar_N_id
            size_t firstUnderscore = 11; // length of "action_bar_"
            size_t secondUnderscore = key.find('_', firstUnderscore);
            if (secondUnderscore == std::string::npos) continue;
            int slot = -1;
            try { slot = std::stoi(key.substr(firstUnderscore, secondUnderscore - firstUnderscore)); } catch (...) { continue; }
            if (slot < 0 || slot >= ACTION_BAR_SLOTS) continue;
            std::string suffix = key.substr(secondUnderscore + 1);
            try {
                if (suffix == "type") {
                    types[slot] = std::stoi(val);
                    hasSlots = true;
                } else if (suffix == "id") {
                    ids[slot] = static_cast<uint32_t>(std::stoul(val));
                    hasSlots = true;
                }
            } catch (...) {}
        }
    }

    // Validate guid matches current character
    if (savedGuid != 0 && savedGuid != playerGuid) {
        LOG_WARNING("Character config guid mismatch for ", ch->name, ", using defaults");
        return;
    }

    // Apply saved gender and body type (allows nonbinary to persist even though server only stores male/female)
    if (savedGender >= 0 && savedGender <= 2) {
        for (auto& character : characters) {
            if (character.guid == playerGuid) {
                character.gender = static_cast<Gender>(savedGender);
                if (character.gender == Gender::NONBINARY) {
                    // Only nonbinary characters have a meaningful body type choice
                    if (savedUseFemaleModel >= 0) {
                        character.useFemaleModel = (savedUseFemaleModel != 0);
                    }
                } else {
                    // Male/female always use the model matching their gender
                    character.useFemaleModel = (character.gender == Gender::FEMALE);
                }
                LOG_INFO("Applied saved gender: ", getGenderName(character.gender),
                         ", body type: ", (character.useFemaleModel ? "feminine" : "masculine"));
                break;
            }
        }
    }

    if (hasSlots) {
        for (int i = 0; i < ACTION_BAR_SLOTS; i++) {
            actionBar[i].type = static_cast<ActionBarSlot::Type>(types[i]);
            actionBar[i].id = ids[i];
        }
        LOG_INFO("Character config loaded from ", path);
    }
}

void GameHandler::setTransportAttachment(uint64_t childGuid, ObjectType type, uint64_t transportGuid,
                                         const glm::vec3& localOffset, bool hasLocalOrientation,
                                         float localOrientation) {
    if (childGuid == 0 || transportGuid == 0) {
        return;
    }

    TransportAttachment& attachment = transportAttachments_[childGuid];
    attachment.type = type;
    attachment.transportGuid = transportGuid;
    attachment.localOffset = localOffset;
    attachment.hasLocalOrientation = hasLocalOrientation;
    attachment.localOrientation = localOrientation;
}

void GameHandler::clearTransportAttachment(uint64_t childGuid) {
    if (childGuid == 0) {
        return;
    }
    transportAttachments_.erase(childGuid);
}

void GameHandler::updateAttachedTransportChildren(float /*deltaTime*/) {
    if (!transportManager_ || transportAttachments_.empty()) {
        return;
    }

    constexpr float kPosEpsilonSq = 0.0001f;
    constexpr float kOriEpsilon = 0.001f;
    std::vector<uint64_t> stale;
    stale.reserve(8);

    for (const auto& [childGuid, attachment] : transportAttachments_) {
        auto entity = entityManager.getEntity(childGuid);
        if (!entity) {
            stale.push_back(childGuid);
            continue;
        }

        ActiveTransport* transport = transportManager_->getTransport(attachment.transportGuid);
        if (!transport) {
            continue;
        }

        glm::vec3 composed = transportManager_->getPlayerWorldPosition(
            attachment.transportGuid, attachment.localOffset);

        float composedOrientation = entity->getOrientation();
        if (attachment.hasLocalOrientation) {
            float baseYaw = transport->hasServerYaw ? transport->serverYaw : 0.0f;
            composedOrientation = baseYaw + attachment.localOrientation;
        }

        glm::vec3 oldPos(entity->getX(), entity->getY(), entity->getZ());
        float oldOrientation = entity->getOrientation();
        glm::vec3 delta = composed - oldPos;
        const bool positionChanged = glm::dot(delta, delta) > kPosEpsilonSq;
        const bool orientationChanged = std::abs(composedOrientation - oldOrientation) > kOriEpsilon;
        if (!positionChanged && !orientationChanged) {
            continue;
        }

        entity->setPosition(composed.x, composed.y, composed.z, composedOrientation);

        if (attachment.type == ObjectType::UNIT) {
            if (creatureMoveCallback_) {
                creatureMoveCallback_(childGuid, composed.x, composed.y, composed.z, 0);
            }
        } else if (attachment.type == ObjectType::GAMEOBJECT) {
            if (gameObjectMoveCallback_) {
                gameObjectMoveCallback_(childGuid, composed.x, composed.y, composed.z, composedOrientation);
            }
        }
    }

    for (uint64_t guid : stale) {
        transportAttachments_.erase(guid);
    }
}

// ============================================================
// Mail System
// ============================================================

void GameHandler::closeMailbox() {
    mailboxOpen_ = false;
    mailboxGuid_ = 0;
    mailInbox_.clear();
    selectedMailIndex_ = -1;
    showMailCompose_ = false;
}

void GameHandler::refreshMailList() {
    if (state != WorldState::IN_WORLD || !socket || mailboxGuid_ == 0) return;
    auto packet = GetMailListPacket::build(mailboxGuid_);
    socket->send(packet);
}

void GameHandler::sendMail(const std::string& recipient, const std::string& subject,
                           const std::string& body, uint32_t money, uint32_t cod) {
    if (state != WorldState::IN_WORLD) {
        LOG_WARNING("sendMail: not in world");
        return;
    }
    if (!socket) {
        LOG_WARNING("sendMail: no socket");
        return;
    }
    if (mailboxGuid_ == 0) {
        LOG_WARNING("sendMail: mailboxGuid_ is 0 (mailbox closed?)");
        return;
    }
    auto packet = packetParsers_->buildSendMail(mailboxGuid_, recipient, subject, body, money, cod);
    LOG_INFO("sendMail: to='", recipient, "' subject='", subject, "' money=", money,
             " mailboxGuid=", mailboxGuid_);
    socket->send(packet);
}

void GameHandler::mailTakeMoney(uint32_t mailId) {
    if (state != WorldState::IN_WORLD || !socket || mailboxGuid_ == 0) return;
    auto packet = MailTakeMoneyPacket::build(mailboxGuid_, mailId);
    socket->send(packet);
}

void GameHandler::mailTakeItem(uint32_t mailId, uint32_t itemIndex) {
    if (state != WorldState::IN_WORLD || !socket || mailboxGuid_ == 0) return;
    auto packet = packetParsers_->buildMailTakeItem(mailboxGuid_, mailId, itemIndex);
    socket->send(packet);
}

void GameHandler::mailDelete(uint32_t mailId) {
    if (state != WorldState::IN_WORLD || !socket || mailboxGuid_ == 0) return;
    // Find mail template ID for this mail
    uint32_t templateId = 0;
    for (const auto& m : mailInbox_) {
        if (m.messageId == mailId) {
            templateId = m.mailTemplateId;
            break;
        }
    }
    auto packet = packetParsers_->buildMailDelete(mailboxGuid_, mailId, templateId);
    socket->send(packet);
}

void GameHandler::mailMarkAsRead(uint32_t mailId) {
    if (state != WorldState::IN_WORLD || !socket || mailboxGuid_ == 0) return;
    auto packet = MailMarkAsReadPacket::build(mailboxGuid_, mailId);
    socket->send(packet);
}

void GameHandler::handleShowMailbox(network::Packet& packet) {
    if (packet.getSize() - packet.getReadPos() < 8) {
        LOG_WARNING("SMSG_SHOW_MAILBOX too short");
        return;
    }
    uint64_t guid = packet.readUInt64();
    LOG_INFO("SMSG_SHOW_MAILBOX: guid=0x", std::hex, guid, std::dec);
    mailboxGuid_ = guid;
    mailboxOpen_ = true;
    hasNewMail_ = false;
    selectedMailIndex_ = -1;
    showMailCompose_ = false;
    // Request inbox contents
    refreshMailList();
}

void GameHandler::handleMailListResult(network::Packet& packet) {
    size_t remaining = packet.getSize() - packet.getReadPos();
    if (remaining < 1) {
        LOG_WARNING("SMSG_MAIL_LIST_RESULT too short (", remaining, " bytes)");
        return;
    }

    // Delegate parsing to expansion-aware packet parser
    packetParsers_->parseMailList(packet, mailInbox_);

    // Resolve sender names (needs GameHandler context, so done here)
    for (auto& msg : mailInbox_) {
        if (msg.messageType == 0 && msg.senderGuid != 0) {
            msg.senderName = getCachedPlayerName(msg.senderGuid);
            if (msg.senderName.empty()) {
                queryPlayerName(msg.senderGuid);
                msg.senderName = "Unknown";
            }
        } else if (msg.messageType == 2) {
            msg.senderName = "Auction House";
        } else if (msg.messageType == 3) {
            msg.senderName = getCachedCreatureName(msg.senderEntry);
            if (msg.senderName.empty()) msg.senderName = "NPC";
        } else {
            msg.senderName = "System";
        }
    }

    // Open the mailbox UI if it isn't already open (Vanilla has no SMSG_SHOW_MAILBOX).
    if (!mailboxOpen_) {
        LOG_INFO("Opening mailbox UI (triggered by SMSG_MAIL_LIST_RESULT)");
        mailboxOpen_ = true;
        hasNewMail_ = false;
        selectedMailIndex_ = -1;
        showMailCompose_ = false;
    }
}

void GameHandler::handleSendMailResult(network::Packet& packet) {
    if (packet.getSize() - packet.getReadPos() < 12) {
        LOG_WARNING("SMSG_SEND_MAIL_RESULT too short");
        return;
    }

    uint32_t mailId = packet.readUInt32();
    uint32_t command = packet.readUInt32();
    uint32_t error = packet.readUInt32();

    // Commands: 0=send, 1=moneyTaken, 2=itemTaken, 3=returnedToSender, 4=deleted, 5=madePermanent
    // Vanilla errors: 0=OK, 1=equipError, 2=cannotSendToSelf, 3=notEnoughMoney, 4=recipientNotFound, 5=notYourTeam, 6=internalError
    static const char* cmdNames[] = {"Send", "TakeMoney", "TakeItem", "Return", "Delete", "MadePermanent"};
    const char* cmdName = (command < 6) ? cmdNames[command] : "Unknown";

    LOG_INFO("SMSG_SEND_MAIL_RESULT: mailId=", mailId, " cmd=", cmdName, " error=", error);

    if (error == 0) {
        // Success
        switch (command) {
            case 0: // Send
                addSystemChatMessage("Mail sent successfully.");
                showMailCompose_ = false;
                refreshMailList();
                break;
            case 1: // Money taken
                addSystemChatMessage("Money received from mail.");
                refreshMailList();
                break;
            case 2: // Item taken
                addSystemChatMessage("Item received from mail.");
                refreshMailList();
                break;
            case 4: // Deleted
                selectedMailIndex_ = -1;
                refreshMailList();
                break;
            default:
                refreshMailList();
                break;
        }
    } else {
        // Error
        std::string errMsg = "Mail error: ";
        switch (error) {
            case 1: errMsg += "Equipment error."; break;
            case 2: errMsg += "You cannot send mail to yourself."; break;
            case 3: errMsg += "Not enough money."; break;
            case 4: errMsg += "Recipient not found."; break;
            case 5: errMsg += "Cannot send to the opposing faction."; break;
            case 6: errMsg += "Internal mail error."; break;
            case 14: errMsg += "Disabled for trial accounts."; break;
            case 15: errMsg += "Recipient's mailbox is full."; break;
            case 16: errMsg += "Cannot send wrapped items COD."; break;
            case 17: errMsg += "Mail and chat suspended."; break;
            case 18: errMsg += "Too many attachments."; break;
            case 19: errMsg += "Invalid attachment."; break;
            default: errMsg += "Unknown error (" + std::to_string(error) + ")."; break;
        }
        addSystemChatMessage(errMsg);
    }
}

void GameHandler::handleReceivedMail(network::Packet& packet) {
    // Server notifies us that new mail arrived
    if (packet.getSize() - packet.getReadPos() >= 4) {
        float nextMailTime = packet.readFloat();
        (void)nextMailTime;
    }
    LOG_INFO("SMSG_RECEIVED_MAIL: New mail arrived!");
    hasNewMail_ = true;
    addSystemChatMessage("New mail has arrived.");
    // If mailbox is open, refresh
    if (mailboxOpen_) {
        refreshMailList();
    }
}

void GameHandler::handleQueryNextMailTime(network::Packet& packet) {
    // Server response to MSG_QUERY_NEXT_MAIL_TIME
    // If there's pending mail, the packet contains a float with time until next mail delivery
    // A value of 0.0 or the presence of mail entries means there IS mail waiting
    size_t remaining = packet.getSize() - packet.getReadPos();
    if (remaining >= 4) {
        float nextMailTime = packet.readFloat();
        // In Vanilla: 0x00000000 = has mail, 0xC7A8C000 (big negative) = no mail
        uint32_t rawValue;
        std::memcpy(&rawValue, &nextMailTime, sizeof(uint32_t));
        if (rawValue == 0 || nextMailTime >= 0.0f) {
            hasNewMail_ = true;
            LOG_INFO("MSG_QUERY_NEXT_MAIL_TIME: Player has pending mail");
        } else {
            LOG_INFO("MSG_QUERY_NEXT_MAIL_TIME: No pending mail (value=", nextMailTime, ")");
        }
    }
}

glm::vec3 GameHandler::getComposedWorldPosition() {
    if (playerTransportGuid_ != 0 && transportManager_) {
        return transportManager_->getPlayerWorldPosition(playerTransportGuid_, playerTransportOffset_);
    }
    // Not on transport, return normal movement position
    return glm::vec3(movementInfo.x, movementInfo.y, movementInfo.z);
}

// ============================================================
// Bank System
// ============================================================

void GameHandler::openBank(uint64_t guid) {
    if (!isConnected()) return;
    auto pkt = BankerActivatePacket::build(guid);
    socket->send(pkt);
}

void GameHandler::closeBank() {
    bankOpen_ = false;
    bankerGuid_ = 0;
}

void GameHandler::buyBankSlot() {
    if (!isConnected() || !bankOpen_) return;
    auto pkt = BuyBankSlotPacket::build(bankerGuid_);
    socket->send(pkt);
}

void GameHandler::depositItem(uint8_t srcBag, uint8_t srcSlot) {
    if (!isConnected() || !bankOpen_) return;
    auto pkt = AutoBankItemPacket::build(srcBag, srcSlot);
    socket->send(pkt);
}

void GameHandler::withdrawItem(uint8_t srcBag, uint8_t srcSlot) {
    if (!isConnected() || !bankOpen_) return;
    auto pkt = AutoStoreBankItemPacket::build(srcBag, srcSlot);
    socket->send(pkt);
}

void GameHandler::handleShowBank(network::Packet& packet) {
    if (packet.getSize() - packet.getReadPos() < 8) return;
    bankerGuid_ = packet.readUInt64();
    bankOpen_ = true;
    gossipWindowOpen = false;  // Close gossip when bank opens
    // Bank items are already tracked via update fields (bank slot GUIDs)
    // Trigger rebuild to populate bank slots in inventory
    rebuildOnlineInventory();
    LOG_INFO("SMSG_SHOW_BANK: banker=0x", std::hex, bankerGuid_, std::dec);
}

void GameHandler::handleBuyBankSlotResult(network::Packet& packet) {
    if (packet.getSize() - packet.getReadPos() < 4) return;
    uint32_t result = packet.readUInt32();
    if (result == 0) {
        addSystemChatMessage("Bank slot purchased.");
        inventory.setPurchasedBankBagSlots(inventory.getPurchasedBankBagSlots() + 1);
    } else {
        addSystemChatMessage("Cannot purchase bank slot.");
    }
}

// ============================================================
// Guild Bank System
// ============================================================

void GameHandler::openGuildBank(uint64_t guid) {
    if (!isConnected()) return;
    auto pkt = GuildBankerActivatePacket::build(guid);
    socket->send(pkt);
}

void GameHandler::closeGuildBank() {
    guildBankOpen_ = false;
    guildBankerGuid_ = 0;
}

void GameHandler::queryGuildBankTab(uint8_t tabId) {
    if (!isConnected() || !guildBankOpen_) return;
    guildBankActiveTab_ = tabId;
    auto pkt = GuildBankQueryTabPacket::build(guildBankerGuid_, tabId, true);
    socket->send(pkt);
}

void GameHandler::buyGuildBankTab() {
    if (!isConnected() || !guildBankOpen_) return;
    uint8_t nextTab = static_cast<uint8_t>(guildBankData_.tabs.size());
    auto pkt = GuildBankBuyTabPacket::build(guildBankerGuid_, nextTab);
    socket->send(pkt);
}

void GameHandler::depositGuildBankMoney(uint32_t amount) {
    if (!isConnected() || !guildBankOpen_) return;
    auto pkt = GuildBankDepositMoneyPacket::build(guildBankerGuid_, amount);
    socket->send(pkt);
}

void GameHandler::withdrawGuildBankMoney(uint32_t amount) {
    if (!isConnected() || !guildBankOpen_) return;
    auto pkt = GuildBankWithdrawMoneyPacket::build(guildBankerGuid_, amount);
    socket->send(pkt);
}

void GameHandler::guildBankWithdrawItem(uint8_t tabId, uint8_t bankSlot, uint8_t destBag, uint8_t destSlot) {
    if (!isConnected() || !guildBankOpen_) return;
    auto pkt = GuildBankSwapItemsPacket::buildBankToInventory(guildBankerGuid_, tabId, bankSlot, destBag, destSlot);
    socket->send(pkt);
}

void GameHandler::guildBankDepositItem(uint8_t tabId, uint8_t bankSlot, uint8_t srcBag, uint8_t srcSlot) {
    if (!isConnected() || !guildBankOpen_) return;
    auto pkt = GuildBankSwapItemsPacket::buildInventoryToBank(guildBankerGuid_, tabId, bankSlot, srcBag, srcSlot);
    socket->send(pkt);
}

void GameHandler::handleGuildBankList(network::Packet& packet) {
    GuildBankData data;
    if (!GuildBankListParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_GUILD_BANK_LIST");
        return;
    }
    guildBankData_ = data;
    guildBankOpen_ = true;
    guildBankActiveTab_ = data.tabId;

    // Ensure item info for all guild bank items
    for (const auto& item : data.tabItems) {
        if (item.itemEntry != 0) ensureItemInfo(item.itemEntry);
    }

    LOG_INFO("SMSG_GUILD_BANK_LIST: tab=", (int)data.tabId,
             " items=", data.tabItems.size(),
             " tabs=", data.tabs.size(),
             " money=", data.money);
}

// ============================================================
// Auction House System
// ============================================================

void GameHandler::openAuctionHouse(uint64_t guid) {
    if (!isConnected()) return;
    auto pkt = AuctionHelloPacket::build(guid);
    socket->send(pkt);
}

void GameHandler::closeAuctionHouse() {
    auctionOpen_ = false;
    auctioneerGuid_ = 0;
}

void GameHandler::auctionSearch(const std::string& name, uint8_t levelMin, uint8_t levelMax,
                                 uint32_t quality, uint32_t itemClass, uint32_t itemSubClass,
                                 uint32_t invTypeMask, uint8_t usableOnly, uint32_t offset)
{
    if (!isConnected() || !auctionOpen_) return;
    if (auctionSearchDelayTimer_ > 0.0f) {
        addSystemChatMessage("Please wait before searching again.");
        return;
    }
    pendingAuctionTarget_ = AuctionResultTarget::BROWSE;
    auto pkt = AuctionListItemsPacket::build(auctioneerGuid_, offset, name,
                                              levelMin, levelMax, invTypeMask,
                                              itemClass, itemSubClass, quality, usableOnly, 0);
    socket->send(pkt);
}

void GameHandler::auctionSellItem(uint64_t itemGuid, uint32_t stackCount,
                                    uint32_t bid, uint32_t buyout, uint32_t duration)
{
    if (!isConnected() || !auctionOpen_) return;
    auto pkt = AuctionSellItemPacket::build(auctioneerGuid_, itemGuid, stackCount, bid, buyout, duration);
    socket->send(pkt);
}

void GameHandler::auctionPlaceBid(uint32_t auctionId, uint32_t amount) {
    if (!isConnected() || !auctionOpen_) return;
    auto pkt = AuctionPlaceBidPacket::build(auctioneerGuid_, auctionId, amount);
    socket->send(pkt);
}

void GameHandler::auctionBuyout(uint32_t auctionId, uint32_t buyoutPrice) {
    auctionPlaceBid(auctionId, buyoutPrice);
}

void GameHandler::auctionCancelItem(uint32_t auctionId) {
    if (!isConnected() || !auctionOpen_) return;
    auto pkt = AuctionRemoveItemPacket::build(auctioneerGuid_, auctionId);
    socket->send(pkt);
}

void GameHandler::auctionListOwnerItems(uint32_t offset) {
    if (!isConnected() || !auctionOpen_) return;
    pendingAuctionTarget_ = AuctionResultTarget::OWNER;
    auto pkt = AuctionListOwnerItemsPacket::build(auctioneerGuid_, offset);
    socket->send(pkt);
}

void GameHandler::auctionListBidderItems(uint32_t offset) {
    if (!isConnected() || !auctionOpen_) return;
    pendingAuctionTarget_ = AuctionResultTarget::BIDDER;
    auto pkt = AuctionListBidderItemsPacket::build(auctioneerGuid_, offset);
    socket->send(pkt);
}

void GameHandler::handleAuctionHello(network::Packet& packet) {
    size_t pktSize = packet.getSize();
    size_t readPos = packet.getReadPos();
    LOG_INFO("handleAuctionHello: packetSize=", pktSize, " readPos=", readPos);
    // Hex dump first 20 bytes for debugging
    const auto& rawData = packet.getData();
    std::string hex;
    size_t dumpLen = std::min<size_t>(rawData.size(), 20);
    for (size_t i = 0; i < dumpLen; ++i) {
        char b[4]; snprintf(b, sizeof(b), "%02x ", rawData[i]);
        hex += b;
    }
    LOG_INFO("  hex dump: ", hex);
    AuctionHelloData data;
    if (!AuctionHelloParser::parse(packet, data)) {
        LOG_WARNING("Failed to parse MSG_AUCTION_HELLO response, size=", pktSize, " readPos=", readPos);
        return;
    }
    auctioneerGuid_ = data.auctioneerGuid;
    auctionHouseId_ = data.auctionHouseId;
    auctionOpen_ = true;
    gossipWindowOpen = false;  // Close gossip when auction house opens
    auctionActiveTab_ = 0;
    auctionBrowseResults_ = AuctionListResult{};
    auctionOwnerResults_ = AuctionListResult{};
    auctionBidderResults_ = AuctionListResult{};
    LOG_INFO("MSG_AUCTION_HELLO: auctioneer=0x", std::hex, data.auctioneerGuid, std::dec,
             " house=", data.auctionHouseId, " enabled=", (int)data.enabled);
}

void GameHandler::handleAuctionListResult(network::Packet& packet) {
    AuctionListResult result;
    if (!AuctionListResultParser::parse(packet, result)) {
        LOG_WARNING("Failed to parse SMSG_AUCTION_LIST_RESULT");
        return;
    }

    auctionBrowseResults_ = result;
    auctionSearchDelayTimer_ = result.searchDelay / 1000.0f;

    // Ensure item info for all auction items
    for (const auto& entry : result.auctions) {
        if (entry.itemEntry != 0) ensureItemInfo(entry.itemEntry);
    }

    LOG_INFO("SMSG_AUCTION_LIST_RESULT: ", result.auctions.size(), " items, total=", result.totalCount);
}

void GameHandler::handleAuctionOwnerListResult(network::Packet& packet) {
    AuctionListResult result;
    if (!AuctionListResultParser::parse(packet, result)) {
        LOG_WARNING("Failed to parse SMSG_AUCTION_OWNER_LIST_RESULT");
        return;
    }
    auctionOwnerResults_ = result;
    for (const auto& entry : result.auctions) {
        if (entry.itemEntry != 0) ensureItemInfo(entry.itemEntry);
    }
    LOG_INFO("SMSG_AUCTION_OWNER_LIST_RESULT: ", result.auctions.size(), " items");
}

void GameHandler::handleAuctionBidderListResult(network::Packet& packet) {
    AuctionListResult result;
    if (!AuctionListResultParser::parse(packet, result)) {
        LOG_WARNING("Failed to parse SMSG_AUCTION_BIDDER_LIST_RESULT");
        return;
    }
    auctionBidderResults_ = result;
    for (const auto& entry : result.auctions) {
        if (entry.itemEntry != 0) ensureItemInfo(entry.itemEntry);
    }
    LOG_INFO("SMSG_AUCTION_BIDDER_LIST_RESULT: ", result.auctions.size(), " items");
}

void GameHandler::handleAuctionCommandResult(network::Packet& packet) {
    AuctionCommandResult result;
    if (!AuctionCommandResultParser::parse(packet, result)) {
        LOG_WARNING("Failed to parse SMSG_AUCTION_COMMAND_RESULT");
        return;
    }

    const char* actions[] = {"Create", "Cancel", "Bid", "Buyout"};
    const char* actionName = (result.action < 4) ? actions[result.action] : "Unknown";

    if (result.errorCode == 0) {
        std::string msg = std::string("Auction ") + actionName + " successful.";
        addSystemChatMessage(msg);
        // Refresh appropriate list
        if (result.action == 0) auctionListOwnerItems();
        else if (result.action == 1) auctionListOwnerItems();
    } else {
        const char* errors[] = {"OK", "Inventory", "Not enough money", "Item not found",
                                "Higher bid", "Increment", "Not enough items",
                                "DB error", "Restricted account"};
        const char* errName = (result.errorCode < 9) ? errors[result.errorCode] : "Unknown";
        std::string msg = std::string("Auction ") + actionName + " failed: " + errName;
        addSystemChatMessage(msg);
    }
    LOG_INFO("SMSG_AUCTION_COMMAND_RESULT: action=", actionName,
             " error=", result.errorCode);
}

} // namespace game
} // namespace wowee
