#include "game/warden_handler.hpp"
#include "game/warden_check_responder.hpp"
#include "game/game_handler.hpp"
#include "game/game_utils.hpp"
#include "game/warden_crypto.hpp"
#include "game/warden_memory.hpp"
#include "game/warden_module.hpp"
#include "network/world_socket.hpp"
#include "network/packet.hpp"
#include "auth/crypto.hpp"
#include "core/application.hpp"
#include "game/expansion_profile.hpp"
#include "pipeline/asset_manager.hpp"
#include "core/logger.hpp"
#include "game/warden_constants.hpp"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>

namespace wowee {
namespace game {

namespace {

// Whether a hash request may be answered without a hand-placed .cr file -
// from the built-in table or by the module itself. On unless
// WOWEE_WARDEN_ANSWER_HASH=0. Answering is what makes a server start sending
// cheat checks, and a wrong check answer is penalised with a ban where an
// unanswered hash request is tolerated - so the answers are graded against
// AzerothCore's own check list in test_warden_check_grader, and this is the
// switch back to silence should a server grade differently.
bool wardenAnswerHashEnabled() {
    const char* env = std::getenv("WOWEE_WARDEN_ANSWER_HASH");
    return !(env && std::string(env) == "0");
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

} // anonymous namespace

// ---------------------------------------------------------------------------
// Construction / init
// ---------------------------------------------------------------------------

WardenHandler::WardenHandler(GameHandler& owner)
    : owner_(owner) {}

// ---------------------------------------------------------------------------
// Opcode registration
// ---------------------------------------------------------------------------

void WardenHandler::registerOpcodes(DispatchTable& table) {
    table[Opcode::SMSG_WARDEN_DATA] = [this](network::Packet& packet) { handleWardenData(packet); };
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

void WardenHandler::reset() {
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
    // A response still being built answers the session that just ended; sent
    // on the next one it would go out under that session's keys.
    if (wardenResponsePending_) {
        wardenPendingEncrypted_.wait();
        wardenPendingEncrypted_ = {};
        wardenResponsePending_ = false;
    }
}

// ---------------------------------------------------------------------------
// Update (called from GameHandler::update)
// ---------------------------------------------------------------------------

void WardenHandler::update(float deltaTime) {
    // Drain pending async Warden response (built on background thread to avoid 5s stalls)
    if (wardenResponsePending_) {
        auto status = wardenPendingEncrypted_.wait_for(std::chrono::milliseconds(0));
        if (status == std::future_status::ready) {
            auto plaintext = wardenPendingEncrypted_.get();
            wardenResponsePending_ = false;
            if (!plaintext.empty() && wardenCrypto_) {
                std::vector<uint8_t> encrypted = wardenCrypto_->encrypt(plaintext);
                network::Packet response(wireOpcode(Opcode::CMSG_WARDEN_DATA));
                for (uint8_t byte : encrypted) {
                    response.writeUInt8(byte);
                }
                if (owner_.getSocket() && owner_.getSocket()->isConnected()) {
                    owner_.getSocket()->send(response);
                    LOG_DEBUG("Warden: Sent async CHEAT_CHECKS_RESULT (", plaintext.size(), " bytes plaintext)");
                }
            }
        }
    }

    // Post-gate visibility
    if (wardenGateSeen_ && owner_.getSocket() && owner_.getSocket()->isConnected()) {
        wardenGateElapsed_ += deltaTime;
        if (wardenGateElapsed_ >= wardenGateNextStatusLog_) {
            LOG_DEBUG("Warden gate status: elapsed=", wardenGateElapsed_,
                     "s connected=", owner_.getSocket()->isConnected() ? "yes" : "no",
                     " packetsAfterGate=", wardenPacketsAfterGate_);
            wardenGateNextStatusLog_ += 30.0f;
        }
    }
}

// ---------------------------------------------------------------------------
// loadWardenCRFile
// ---------------------------------------------------------------------------

bool WardenHandler::loadWardenCRFile(const std::string& moduleHashHex) {
    const std::string crPath = wardenCRPath(moduleHashHex);
    std::string source = crPath;
    if (!readWardenCR(crPath, wardenCheckOpcodes_, wardenCREntries_)) {
        if (!wardenAnswerHashEnabled() ||
            !knownWardenCR(moduleHashHex, wardenCheckOpcodes_, wardenCREntries_)) {
            LOG_WARNING("Warden: No usable .cr file at ", crPath,
                        " - hash requests go unanswered. Servers that tolerate that are unaffected; "
                        "otherwise turn Warden off on the server");
            return false;
        }
        source = "the built-in TrinityCore/AzerothCore module table";
    }

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
        LOG_DEBUG("Warden: Check opcodes: ", opcHex);
    }

    LOG_INFO("Warden: Loaded ", wardenCREntries_.size(), " CR entries from ", source);
    return true;
}

// ---------------------------------------------------------------------------
// handleWardenData - main Warden packet dispatcher
// ---------------------------------------------------------------------------

void WardenHandler::handleWardenData(network::Packet& packet) {
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
        if (owner_.getSessionKey().size() != 40) {
            LOG_ERROR("Warden: No valid session key (size=", owner_.getSessionKey().size(), "), cannot init crypto");
            wardenCrypto_.reset();
            return;
        }
        if (!wardenCrypto_->initFromSessionKey(owner_.getSessionKey())) {
            LOG_ERROR("Warden: Failed to initialize crypto from session key");
            wardenCrypto_.reset();
            return;
        }
        wardenState_ = WardenState::WAIT_MODULE_USE;
    }

    // Decrypt the payload
    std::vector<uint8_t> decrypted = wardenCrypto_->decrypt(data);

    // Avoid expensive hex formatting when DEBUG logs are disabled.
    if (core::Logger::getInstance().shouldLog(core::LogLevel::DEBUG)) {
        std::string hex;
        size_t logSize = std::min(decrypted.size(), size_t(256));
        hex.reserve(logSize * 3);
        for (size_t i = 0; i < logSize; ++i) {
            char b[4];
            snprintf(b, sizeof(b), "%02x ", decrypted[i]);
            hex += b;
        }
        if (decrypted.size() > 64) {
            hex += "... (" + std::to_string(decrypted.size() - 64) + " more)";
        }
        LOG_DEBUG("Warden: Decrypted (", decrypted.size(), " bytes): ", hex);
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
        if (owner_.getSocket() && owner_.getSocket()->isConnected()) {
            owner_.getSocket()->send(response);
            LOG_DEBUG("Warden: Sent response (", plaintext.size(), " bytes plaintext)");
        }
    };

    switch (wardenOpcode) {
        case WARDEN_SMSG_MODULE_USE: { // MODULE_USE
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
                LOG_DEBUG("Warden: MODULE_USE hash=", hashHex, " size=", wardenModuleSize_);

                // Try to load pre-computed challenge/response entries
                loadWardenCRFile(hashHex);
            }

            // Respond with MODULE_MISSING to request the module data
            std::vector<uint8_t> resp = { WARDEN_CMSG_MODULE_MISSING };
            sendWardenResponse(resp);
            wardenState_ = WardenState::WAIT_MODULE_CACHE;
            LOG_DEBUG("Warden: Sent MODULE_MISSING for module size=", wardenModuleSize_, ", waiting for data chunks");
            break;
        }

        case WARDEN_SMSG_MODULE_CACHE: { // MODULE_CACHE (module data chunk)
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

            LOG_DEBUG("Warden: MODULE_CACHE chunk ", chunkSize, " bytes, total ",
                     wardenModuleData_.size(), "/", wardenModuleSize_);

            // Check if module download is complete
            if (wardenModuleData_.size() >= wardenModuleSize_) {
                LOG_INFO("Warden: Module download complete (",
                         wardenModuleData_.size(), " bytes)");
                wardenState_ = WardenState::WAIT_HASH_REQUEST;

                // Cache raw module to disk
                {
                    const std::string cacheDir = wardenCacheDir();
                    std::error_code ec;
                    std::filesystem::create_directories(cacheDir, ec);

                    std::string hashHex;
                    for (auto b : wardenModuleHash_) { char s[4]; snprintf(s, 4, "%02x", b); hashHex += s; }
                    std::string cachePath = cacheDir + "/" + hashHex + ".wdn";

                    std::ofstream wf(cachePath, std::ios::binary);
                    if (wf) {
                        wf.write(reinterpret_cast<const char*>(wardenModuleData_.data()), wardenModuleData_.size());
                        LOG_DEBUG("Warden: Cached module to ", cachePath);
                    }
                }

                // Load the module (decrypt, decompress, parse, relocate, init)
                wardenLoadedModule_ = std::make_shared<WardenModule>();
                // Check the signature against the key this realm actually
                // signs with, when its profile names one.
                if (auto* reg = core::Application::getInstance().getExpansionRegistry()) {
                    if (const auto* profile = reg->getActive();
                        profile && !profile->wardenRsaModulus.empty()) {
                        wardenLoadedModule_->setRsaModulus(profile->wardenRsaModulus);
                    }
                }
                // Inject crypto and socket so module callbacks (sendPacket, generateRC4)
                // can reach the network layer during initializeModule().
                wardenLoadedModule_->setCallbackDependencies(
                    wardenCrypto_.get(),
                    [this](const uint8_t* data, size_t len) {
                        if (!wardenCrypto_ || !owner_.getSocket()) return;
                        std::vector<uint8_t> plaintext(data, data + len);
                        auto encrypted = wardenCrypto_->encrypt(plaintext);
                        network::Packet pkt(wireOpcode(Opcode::CMSG_WARDEN_DATA));
                        for (uint8_t b : encrypted) pkt.writeUInt8(b);
                        owner_.getSocket()->send(pkt);
                        LOG_DEBUG("Warden: Module sendPacket callback sent ", len, " bytes");
                    });
                if (wardenLoadedModule_->load(wardenModuleData_, wardenModuleHash_, wardenModuleKey_)) { // codeql[cpp/weak-cryptographic-algorithm]
                    LOG_INFO("Warden: Module loaded successfully (image size=",
                             wardenLoadedModule_->getModuleSize(), " bytes)");
                } else {
                    LOG_ERROR("Warden: Module loading FAILED");
                    wardenLoadedModule_.reset();
                }

                // Send MODULE_OK
                std::vector<uint8_t> resp = { WARDEN_CMSG_MODULE_OK };
                sendWardenResponse(resp);
                LOG_DEBUG("Warden: Sent MODULE_OK");
            }
            // No response for intermediate chunks
            break;
        }

        case WARDEN_SMSG_HASH_REQUEST: { // HASH_REQUEST
            // Format: [1 opcode][16 seed]
            if (decrypted.size() < 17) {
                LOG_ERROR("Warden: HASH_REQUEST too short (", decrypted.size(), " bytes, need 17)");
                return;
            }

            std::vector<uint8_t> seed(decrypted.begin() + 1, decrypted.begin() + 17);
            auto applyWardenSeedRekey = [&](const std::vector<uint8_t>& rekeySeed) {
                // Derive new RC4 keys from the seed using SHA1Randx.
                uint8_t newEncryptKey[16], newDecryptKey[16];
                WardenCrypto::sha1RandxGenerate(rekeySeed, newEncryptKey, newDecryptKey);

                std::vector<uint8_t> ek(newEncryptKey, newEncryptKey + 16);
                std::vector<uint8_t> dk(newDecryptKey, newDecryptKey + 16);
                wardenCrypto_->replaceKeys(ek, dk);
                for (auto& b : newEncryptKey) b = 0;
                for (auto& b : newDecryptKey) b = 0;
                LOG_DEBUG("Warden: Derived and applied key update from seed");
            };

            // --- Try CR lookup (pre-computed challenge/response entries) ---
            if (!wardenCREntries_.empty()) {
                const WardenCRRecord* match = nullptr;
                for (const auto& entry : wardenCREntries_) {
                    if (std::memcmp(entry.seed, seed.data(), 16) == 0) {
                        match = &entry;
                        break;
                    }
                }

                if (match) {
                    LOG_DEBUG("Warden: HASH_REQUEST - CR entry MATCHED, sending pre-computed reply");

                    // Send HASH_RESULT
                    std::vector<uint8_t> resp;
                    resp.push_back(WARDEN_CMSG_HASH_RESULT);
                    resp.insert(resp.end(), match->reply, match->reply + 20);
                    sendWardenResponse(resp);

                    // Switch to new RC4 keys from the CR entry
                    // clientKey = encrypt (client→server), serverKey = decrypt (server→client)
                    std::vector<uint8_t> newEncryptKey(match->clientKey, match->clientKey + 16);
                    std::vector<uint8_t> newDecryptKey(match->serverKey, match->serverKey + 16);
                    wardenCrypto_->replaceKeys(newEncryptKey, newDecryptKey);

                    LOG_DEBUG("Warden: Switched to CR key set");

                    wardenState_ = WardenState::WAIT_CHECKS;
                    break;
                }
                LOG_DEBUG("Warden: Seed not found in ", wardenCREntries_.size(), " CR entries");
            }

            // --- No CR match: ask the module itself ---
            // The module computes its own reply and the keys the exchange
            // switches to, so where it runs its answer is the right one for
            // any server. Unless turned off; see wardenAnswerHashEnabled.
            {
                uint8_t reply[20];
                std::vector<uint8_t> rc4State;
                if (wardenAnswerHashEnabled() && wardenLoadedModule_ &&
                    wardenLoadedModule_->answerHashRequest(seed.data(), owner_.getSessionKey(),
                                                           reply, rc4State)) {
                    std::vector<uint8_t> resp;
                    resp.push_back(WARDEN_CMSG_HASH_RESULT);
                    resp.insert(resp.end(), reply, reply + 20);
                    sendWardenResponse(resp);
                    wardenCrypto_->replaceState(rc4State);
                    LOG_INFO("Warden: HASH_REQUEST answered by the module's own handler");
                    wardenState_ = WardenState::WAIT_CHECKS;
                    break;
                }
            }

            // --- No CR match: decide strategy based on server strictness ---
            {
                std::string seedHex;
                for (auto b : seed) { char s[4]; snprintf(s, 4, "%02x", b); seedHex += s; }

                bool isTurtle = isActiveExpansion("turtle");
                bool isClassic = (owner_.getBuild() <= 6005) && !isTurtle;
                const char* turtleFallbackEnv = std::getenv("WOWEE_WARDEN_TURTLE_NO_CR");
                const bool turtleFallbackAllowed =
                    turtleFallbackEnv && std::string(turtleFallbackEnv) == "fallback";

                if (isTurtle && !turtleFallbackAllowed) {
                    // Turtle-derived servers close the world connection when given an
                    // invented HASH_RESULT. Without a verified .cr entry, preserving
                    // the cipher state and waiting for follow-up checks is the only
                    // safe protocol behavior.
                    LOG_WARNING("Warden: HASH_REQUEST seed=", seedHex,
                                " - no CR match, skipping response for Turtle "
                                "(set WOWEE_WARDEN_TURTLE_NO_CR=fallback to test the legacy fallback)");
                    wardenState_ = WardenState::WAIT_CHECKS;
                    break;
                }

                if (!isTurtle && !isClassic) {
                    // WotLK/TBC: don't respond to HASH_REQUEST without a valid CR match.
                    // ChromieCraft/AzerothCore tolerates the silence (no ban, no kick),
                    // but REJECTS a wrong hash and closes the connection immediately.
                    // Staying silent lets the server continue the session without Warden checks.
                    //
                    // Said at warning level: if a server stops tolerating the
                    // silence, the session ends right after this, and a log
                    // that stops at the missing .cr file cannot say why.
                    LOG_WARNING("Warden: HASH_REQUEST seed=", seedHex,
                                " - no CR match, not answering; a server that requires an "
                                "answer will disconnect after this");
                    wardenState_ = WardenState::WAIT_CHECKS;
                    break;
                }

                LOG_WARNING("Warden: No CR match (seed=", seedHex,
                            "), sending fallback hash (lenient server)");

                std::vector<uint8_t> fallbackReply;
                if (wardenLoadedModule_ && wardenLoadedModule_->isLoaded()) {
                    const uint8_t* moduleImage = static_cast<const uint8_t*>(wardenLoadedModule_->getModuleMemory());
                    size_t moduleImageSize = wardenLoadedModule_->getModuleSize();
                    if (moduleImage && moduleImageSize > 0) {
                        std::vector<uint8_t> imageData(moduleImage, moduleImage + moduleImageSize);
                        fallbackReply = auth::Crypto::sha1(imageData);
                    }
                }
                if (fallbackReply.empty()) {
                    if (!wardenModuleData_.empty())
                        fallbackReply = auth::Crypto::sha1(wardenModuleData_);
                    else
                        fallbackReply.assign(20, 0);
                }

                std::vector<uint8_t> resp;
                resp.push_back(0x04); // WARDEN_CMSG_HASH_RESULT
                resp.insert(resp.end(), fallbackReply.begin(), fallbackReply.end());
                sendWardenResponse(resp);
                applyWardenSeedRekey(seed);
            }

            wardenState_ = WardenState::WAIT_CHECKS;
            break;
        }

        case WARDEN_SMSG_CHEAT_CHECKS_REQUEST: { // CHEAT_CHECKS_REQUEST
            LOG_DEBUG("Warden: CHEAT_CHECKS_REQUEST (", decrypted.size(), " bytes)");

            if (decrypted.size() < 3) {
                LOG_ERROR("Warden: CHEAT_CHECKS_REQUEST too short");
                break;
            }

            // Memory checks are served from the WoW.exe image, loaded here on
            // the main thread before anything reads it.
            if (!wardenMemory_) {
                wardenMemory_ = std::make_unique<WardenMemory>();
                std::string expansionId;
                if (auto* reg = core::Application::getInstance().getExpansionRegistry()) {
                    if (const auto* profile = reg->getActive()) expansionId = profile->id;
                }
                if (!wardenMemory_->load(static_cast<uint16_t>(owner_.getBuild()), isActiveExpansion("turtle"),
                                         expansionId)) {
                    LOG_WARNING("Warden: Could not load WoW.exe for MEM_CHECK");
                }
            }

            WardenCheckContext ctx;
            ctx.opcodes = wardenCheckOpcodes_;
            ctx.memory = wardenMemory_.get();
            if (wardenLoadedModule_ && wardenLoadedModule_->isLoaded()) {
                ctx.moduleImage = static_cast<const uint8_t*>(wardenLoadedModule_->getModuleMemory());
                ctx.moduleImageSize = wardenLoadedModule_->getModuleSize();
            }
            ctx.turtle = isActiveExpansion("turtle");
            ctx.readGameFile = [this](const std::string& wowPath) -> std::vector<uint8_t> {
                auto* am = owner_.services().assetManager;
                if (!am || !am->isInitialized()) return {};
                std::vector<uint8_t> data;
                const std::string fsPath = resolveCaseInsensitiveDataPath(am->getDataPath(), wowPath);
                if (!fsPath.empty()) data = readFileBinary(fsPath);
                if (data.empty()) data = am->readFile(wowPath);
                return data;
            };

            // A code scan searches the whole image and can take seconds, so a
            // request holding one is answered off the main thread; update()
            // encrypts and sends what comes back.
            if (wardenRequestHasCodeScan(decrypted, wardenCheckOpcodes_) && !wardenResponsePending_) {
                LOG_DEBUG("Warden: PAGE_A/PAGE_B detected - building response async to avoid main-loop stall");
                wardenPendingEncrypted_ = std::async(std::launch::async, [decrypted, ctx]() {
                    return buildWardenCheckResponse(decrypted, ctx, "async");
                });
                wardenResponsePending_ = true;
                break;
            }

            sendWardenResponse(buildWardenCheckResponse(decrypted, ctx, "sync"));
            break;
        }

        case 0x03: // WARDEN_SMSG_MODULE_INITIALIZE
            LOG_DEBUG("Warden: MODULE_INITIALIZE (", decrypted.size(), " bytes, no response needed)");
            break;

        default:
            LOG_DEBUG("Warden: Unknown opcode 0x", std::hex, (int)wardenOpcode, std::dec,
                     " (state=", (int)wardenState_, ", size=", decrypted.size(), ")");
            break;
    }
}

} // namespace game
} // namespace wowee
