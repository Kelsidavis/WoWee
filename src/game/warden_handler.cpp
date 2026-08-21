#include "game/warden_handler.hpp"
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
#include <openssl/sha.h>
#include <openssl/hmac.h>

namespace wowee {
namespace game {

namespace {

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

// Pre-computed HMAC-SHA1 hashes of known door M2 models that Warden checks
// to verify the client hasn't modified collision geometry (wall-hack detection).
// These hashes match the unmodified 3.3.5a client data files.
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
                    LOG_WARNING("Warden: Sent async CHEAT_CHECKS_RESULT (", plaintext.size(), " bytes plaintext)");
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
        LOG_DEBUG("Warden: Check opcodes: ", opcHex);
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
                const WardenCREntry* match = nullptr;
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
                }                     LOG_DEBUG("Warden: Seed not found in ", wardenCREntries_.size(), " CR entries");
               
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
                    LOG_DEBUG("Warden: HASH_REQUEST seed=", seedHex,
                                " - no CR match, skipping response (server tolerates silence)");
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
            LOG_DEBUG("Warden: String table: ", strings.size(), " entries");
            for (size_t i = 0; i < strings.size(); i++) {
                LOG_DEBUG("Warden:   [", i, "] = \"", strings[i], "\"");
            }

            // XOR byte is the last byte of the packet
            uint8_t xorByte = decrypted.back();
            LOG_DEBUG("Warden: XOR byte = 0x", [&]{ char s[4]; snprintf(s,4,"%02x",xorByte); return std::string(s); }());

            // Quick-scan for PAGE_A/PAGE_B checks (these trigger 5-second brute-force searches)
            {
                bool hasSlowChecks = false;
                for (size_t i = pos; i < decrypted.size() - 1; i++) {
                    uint8_t d = decrypted[i] ^ xorByte;
                    if (d == wardenCheckOpcodes_[2] || d == wardenCheckOpcodes_[3]) {
                        hasSlowChecks = true;
                        break;
                    }
                }
                if (hasSlowChecks && !wardenResponsePending_) {
                    LOG_WARNING("Warden: PAGE_A/PAGE_B detected - building response async to avoid main-loop stall");
                    // Ensure wardenMemory_ is loaded on main thread before launching async task
                    if (!wardenMemory_) {
                        wardenMemory_ = std::make_unique<WardenMemory>();
                        if (!wardenMemory_->load(static_cast<uint16_t>(owner_.getBuild()), isActiveExpansion("turtle"))) {
                            LOG_WARNING("Warden: Could not load WoW.exe for MEM_CHECK");
                        }
                    }
                    // Capture state by value (decrypted, strings) and launch async.
                    // The async task returns plaintext response bytes; main thread encrypts+sends in update().
                    size_t capturedPos = pos;
                    wardenPendingEncrypted_ = std::async(std::launch::async,
                        [this, decrypted, strings, xorByte, capturedPos]() -> std::vector<uint8_t> {
                            // This runs on a background thread - same logic as the synchronous path below.
                            // BEGIN: duplicated check processing (kept in sync with synchronous path)
                            enum CheckType { CT_MEM=0, CT_PAGE_A=1, CT_PAGE_B=2, CT_MPQ=3, CT_LUA=4,
                                             CT_DRIVER=5, CT_TIMING=6, CT_PROC=7, CT_MODULE=8, CT_UNKNOWN=9 };
                            size_t checkEnd = decrypted.size() - 1;
                            size_t pos = capturedPos;

                            auto decodeCheckType = [&](uint8_t raw) -> CheckType {
                                uint8_t decoded = raw ^ xorByte;
                                if (decoded == wardenCheckOpcodes_[0]) return CT_MEM;
                                if (decoded == wardenCheckOpcodes_[1]) return CT_MODULE;
                                if (decoded == wardenCheckOpcodes_[2]) return CT_PAGE_A;
                                if (decoded == wardenCheckOpcodes_[3]) return CT_PAGE_B;
                                if (decoded == wardenCheckOpcodes_[4]) return CT_MPQ;
                                if (decoded == wardenCheckOpcodes_[5]) return CT_LUA;
                                if (decoded == wardenCheckOpcodes_[6]) return CT_PROC;
                                if (decoded == wardenCheckOpcodes_[7]) return CT_DRIVER;
                                if (decoded == wardenCheckOpcodes_[8]) return CT_TIMING;
                                return CT_UNKNOWN;
                            };
                            auto resolveString = [&](uint8_t idx) -> std::string {
                                if (idx == 0) return {};
                                size_t i = idx - 1;
                                return i < strings.size() ? strings[i] : std::string();
                            };
                            auto isKnownWantedCodeScan = [&](const uint8_t seed[4], const uint8_t hash[20],
                                                             uint32_t off, uint8_t len) -> bool {
                                auto tryMatch = [&](const uint8_t* pat, size_t patLen) {
                                    uint8_t out[SHA_DIGEST_LENGTH]; unsigned int outLen = 0;
                                    HMAC(EVP_sha1(), seed, 4, pat, patLen, out, &outLen);
                                    return outLen == SHA_DIGEST_LENGTH && !std::memcmp(out, hash, SHA_DIGEST_LENGTH);
                                };
                                static constexpr uint8_t p1[] = {0x33,0xD2,0x33,0xC9,0xE8,0x87,0x07,0x1B,0x00,0xE8};
                                if (off == 13856 && len == sizeof(p1) && tryMatch(p1, sizeof(p1))) return true;
                                static constexpr uint8_t p2[] = {0x56,0x57,0xFC,0x8B,0x54,0x24,0x14,0x8B,
                                    0x74,0x24,0x10,0x8B,0x44,0x24,0x0C,0x8B,0xCA,0x8B,0xF8,0xC1,
                                    0xE9,0x02,0x74,0x02,0xF3,0xA5,0xB1,0x03,0x23,0xCA,0x74,0x02,
                                    0xF3,0xA4,0x5F,0x5E,0xC3};
                                if (len == sizeof(p2) && tryMatch(p2, sizeof(p2))) return true;
                                return false;
                            };

                            std::vector<uint8_t> resultData;
                            int checkCount = 0;
                            int checkTypeCounts[10] = {};

                            #define WARDEN_ASYNC_HANDLER 1
                            // The check processing loop is identical to the synchronous path.
                            // See the synchronous case 0x02 below for the canonical version.
                            while (pos < checkEnd) {
                                CheckType ct = decodeCheckType(decrypted[pos]);
                                pos++;
                                checkCount++;
                                if (ct <= CT_UNKNOWN) checkTypeCounts[ct]++;

                                switch (ct) {
                                case CT_TIMING: {
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
                                    if (pos + 6 > checkEnd) { pos = checkEnd; break; }
                                    uint8_t strIdx = decrypted[pos++];
                                    std::string moduleName = resolveString(strIdx);
                                    uint32_t offset = decrypted[pos] | (uint32_t(decrypted[pos+1])<<8)
                                                    | (uint32_t(decrypted[pos+2])<<16) | (uint32_t(decrypted[pos+3])<<24);
                                    pos += 4;
                                    uint8_t readLen = decrypted[pos++];
                                    LOG_WARNING("Warden:   MEM offset=0x", [&]{char s[12];snprintf(s,12,"%08x",offset);return std::string(s);}(),
                                             " len=", (int)readLen,
                                             (strIdx ? " module=\"" + moduleName + "\"" : ""));
                                    if (offset == WARDEN_TICKCOUNT_ADDRESS && readLen == 4 && wardenMemory_ && wardenMemory_->isLoaded()) {
                                        uint32_t now = static_cast<uint32_t>(
                                            std::chrono::duration_cast<std::chrono::milliseconds>(
                                                std::chrono::steady_clock::now().time_since_epoch()).count());
                                        wardenMemory_->writeLE32(0xCF0BC8, now - 2000);
                                    }
                                    std::vector<uint8_t> memBuf(readLen, 0);
                                    bool memOk = wardenMemory_ && wardenMemory_->isLoaded() &&
                                                 wardenMemory_->readMemory(offset, readLen, memBuf.data());
                                    if (memOk) {
                                        const char* region = "?";
                                        if (offset >= KUSER_SHARED_DATA_BASE && offset < KUSER_SHARED_DATA_END) region = "KUSER";
                                        else if (offset >= PE_TEXT_SECTION_BASE && offset < PE_TEXT_SECTION_END) region = ".text/.code";
                                        else if (offset >= PE_RDATA_SECTION_BASE && offset < PE_DATA_RAW_SECTION_BASE) region = ".rdata";
                                        else if (offset >= PE_DATA_RAW_SECTION_BASE && offset < PE_BSS_SECTION_BASE) region = ".data(raw)";
                                        else if (offset >= PE_BSS_SECTION_BASE && offset < PE_BSS_SECTION_END) region = ".data(BSS)";
                                        bool allZero = true;
                                        for (int i = 0; i < (int)readLen; i++) { if (memBuf[i] != 0) { allZero = false; break; } }
                                        std::string hexDump;
                                        for (int i = 0; i < (int)readLen; i++) { char hx[4]; snprintf(hx,4,"%02x ",memBuf[i]); hexDump += hx; }
                                        LOG_WARNING("Warden:   MEM_CHECK served: [", hexDump, "] region=", region,
                                                    (allZero && offset >= PE_BSS_SECTION_BASE ? " \xe2\x98\x85""BSS_ZERO\xe2\x98\x85" : ""));
                                        if (offset == WARDEN_WIN_VERSION_ADDRESS && readLen == 12)
                                            LOG_WARNING("Warden:   Applying 4-byte ULONG alignment padding for WinVersionGet");
                                        resultData.push_back(WARDEN_MEM_CHECK_SUCCESS);
                                        resultData.insert(resultData.end(), memBuf.begin(), memBuf.end());
                                    } else {
                                        LOG_WARNING("Warden:   MEM_CHECK -> 0xE9 (unmapped 0x",
                                                    [&]{char s[12];snprintf(s,12,"%08x",offset);return std::string(s);}(), ")");
                                        resultData.push_back(WARDEN_MEM_CHECK_UNMAPPED);
                                    }
                                    break;
                                }
                                case CT_PAGE_A:
                                case CT_PAGE_B: {
                                    constexpr size_t kPageSize = 29;
                                    const char* pageName = (ct == CT_PAGE_A) ? "PAGE_A" : "PAGE_B";
                                    bool isImageOnly = (ct == CT_PAGE_A);
                                    if (pos + kPageSize > checkEnd) { pos = checkEnd; resultData.push_back(0x00); break; }
                                    const uint8_t* p = decrypted.data() + pos;
                                    const uint8_t* seed = p;
                                    const uint8_t* sha1 = p + 4;
                                    uint32_t off = uint32_t(p[24])|(uint32_t(p[25])<<8)|(uint32_t(p[26])<<16)|(uint32_t(p[27])<<24);
                                    uint8_t patLen = p[28];
                                    bool found = false;
                                    bool turtleFallback = false;
                                    if (isKnownWantedCodeScan(seed, sha1, off, patLen)) {
                                        found = true;
                                    } else if (wardenMemory_ && wardenMemory_->isLoaded() && patLen > 0) {
                                        bool hintOnly = (ct == CT_PAGE_A && isActiveExpansion("turtle"));
                                        found = wardenMemory_->searchCodePattern(seed, sha1, patLen, isImageOnly, off, hintOnly);
                                        if (!found && !hintOnly && wardenLoadedModule_ && wardenLoadedModule_->isLoaded()) {
                                            const uint8_t* modMem = static_cast<const uint8_t*>(wardenLoadedModule_->getModuleMemory());
                                            size_t modSize = wardenLoadedModule_->getModuleSize();
                                            if (modMem && modSize >= patLen) {
                                                for (size_t i = 0; i < modSize - patLen + 1; i++) {
                                                    uint8_t h[20]; unsigned int hl = 0;
                                                    HMAC(EVP_sha1(), seed, 4, modMem+i, patLen, h, &hl);
                                                    if (hl == 20 && !std::memcmp(h, sha1, 20)) { found = true; break; }
                                                }
                                            }
                                        }
                                    }
                                    if (!found && ct == CT_PAGE_A && isActiveExpansion("turtle") && off < 0x600000) {
                                        found = true;
                                        turtleFallback = true;
                                    }
                                    uint8_t pageResult = found ? 0x4A : 0x00;
                                    LOG_WARNING("Warden:   ", pageName, " offset=0x",
                                                [&]{char s[12];snprintf(s,12,"%08x",off);return std::string(s);}(),
                                                " patLen=", (int)patLen, " found=", found ? "yes" : "no",
                                                turtleFallback ? " (turtle-fallback)" : "");
                                    pos += kPageSize;
                                    resultData.push_back(pageResult);
                                    break;
                                }
                                case CT_MPQ: {
                                    if (pos + 1 > checkEnd) { pos = checkEnd; break; }
                                    uint8_t strIdx = decrypted[pos++];
                                    std::string filePath = resolveString(strIdx);
                                    LOG_WARNING("Warden:   MPQ file=\"", (filePath.empty() ? "?" : filePath), "\"");
                                    bool found = false;
                                    std::vector<uint8_t> hash(20, 0);
                                    if (!filePath.empty()) {
                                        std::string np = asciiLower(filePath);
                                        std::replace(np.begin(), np.end(), '/', '\\');
                                        auto knownIt = knownDoorHashes().find(np);
                                        if (knownIt != knownDoorHashes().end()) { found = true; hash.assign(knownIt->second.begin(), knownIt->second.end()); }
                                        auto* am = owner_.services().assetManager;
                                        if (am && am->isInitialized() && !found) {
                                            std::vector<uint8_t> fd;
                                            std::string rp = resolveCaseInsensitiveDataPath(am->getDataPath(), filePath);
                                            if (!rp.empty()) fd = readFileBinary(rp);
                                            if (fd.empty()) fd = am->readFile(filePath);
                                            if (!fd.empty()) { found = true; hash = auth::Crypto::sha1(fd); }
                                        }
                                    }
                                    LOG_WARNING("Warden:   MPQ result=", (found ? "FOUND" : "NOT_FOUND"));
                                    if (found) { resultData.push_back(0x00); resultData.insert(resultData.end(), hash.begin(), hash.end()); }
                                    else { resultData.push_back(0x01); }
                                    break;
                                }
                                case CT_LUA: {
                                    if (pos + 1 > checkEnd) { pos = checkEnd; break; }
                                    pos++; resultData.push_back(0x01); break;
                                }
                                case CT_DRIVER: {
                                    if (pos + 25 > checkEnd) { pos = checkEnd; break; }
                                    pos += 24;
                                    uint8_t strIdx = decrypted[pos++];
                                    std::string dn = resolveString(strIdx);
                                    LOG_WARNING("Warden:   DRIVER=\"", (dn.empty() ? "?" : dn), "\" -> 0x00(not found)");
                                    resultData.push_back(0x00); break;
                                }
                                case CT_MODULE: {
                                    if (pos + 24 > checkEnd) { pos = checkEnd; resultData.push_back(0x00); break; }
                                    const uint8_t* p = decrypted.data() + pos;
                                    uint8_t sb[4] = {p[0],p[1],p[2],p[3]};
                                    uint8_t rh[20]; std::memcpy(rh, p+4, 20);
                                    pos += 24;
                                    bool isWanted = hmacSha1Matches(sb, "KERNEL32.DLL", rh);
                                    std::string mn = isWanted ? "KERNEL32.DLL" : "?";
                                    if (!isWanted) {
                                        // Cheat modules (unwanted - report not found)
                                        if (hmacSha1Matches(sb,"WPESPY.DLL",rh)) mn = "WPESPY.DLL";
                                        else if (hmacSha1Matches(sb,"TAMIA.DLL",rh)) mn = "TAMIA.DLL";
                                        else if (hmacSha1Matches(sb,"PRXDRVPE.DLL",rh)) mn = "PRXDRVPE.DLL";
                                        else if (hmacSha1Matches(sb,"SPEEDHACK-I386.DLL",rh)) mn = "SPEEDHACK-I386.DLL";
                                        else if (hmacSha1Matches(sb,"D3DHOOK.DLL",rh)) mn = "D3DHOOK.DLL";
                                        else if (hmacSha1Matches(sb,"NJUMD.DLL",rh)) mn = "NJUMD.DLL";
                                        // System DLLs (wanted - report found)
                                        else if (hmacSha1Matches(sb,"USER32.DLL",rh)) { mn = "USER32.DLL"; isWanted = true; }
                                        else if (hmacSha1Matches(sb,"NTDLL.DLL",rh)) { mn = "NTDLL.DLL"; isWanted = true; }
                                        else if (hmacSha1Matches(sb,"WS2_32.DLL",rh)) { mn = "WS2_32.DLL"; isWanted = true; }
                                        else if (hmacSha1Matches(sb,"WSOCK32.DLL",rh)) { mn = "WSOCK32.DLL"; isWanted = true; }
                                        else if (hmacSha1Matches(sb,"ADVAPI32.DLL",rh)) { mn = "ADVAPI32.DLL"; isWanted = true; }
                                        else if (hmacSha1Matches(sb,"SHELL32.DLL",rh)) { mn = "SHELL32.DLL"; isWanted = true; }
                                        else if (hmacSha1Matches(sb,"GDI32.DLL",rh)) { mn = "GDI32.DLL"; isWanted = true; }
                                        else if (hmacSha1Matches(sb,"OPENGL32.DLL",rh)) { mn = "OPENGL32.DLL"; isWanted = true; }
                                        else if (hmacSha1Matches(sb,"WINMM.DLL",rh)) { mn = "WINMM.DLL"; isWanted = true; }
                                    }
                                    uint8_t mr = isWanted ? 0x4A : 0x00;
                                    LOG_WARNING("Warden:   MODULE \"", mn, "\" -> 0x",
                                                [&]{char s[4];snprintf(s,4,"%02x",mr);return std::string(s);}(),
                                                isWanted ? "(found)" : "(not found)");
                                    resultData.push_back(mr); break;
                                }
                                case CT_PROC: {
                                    if (pos + 30 > checkEnd) { pos = checkEnd; break; }
                                    pos += 30; resultData.push_back(0x01); break;
                                }
                                default: pos = checkEnd; break;
                                }
                            }
                            #undef WARDEN_ASYNC_HANDLER

                            // Log summary
                            {
                                std::string summary;
                                const char* ctNames[] = {"MEM","PAGE_A","PAGE_B","MPQ","LUA","DRIVER","TIMING","PROC","MODULE","UNK"};
                                for (int i = 0; i < 10; i++) {
                                    if (checkTypeCounts[i] > 0) {
                                        if (!summary.empty()) summary += " ";
                                        summary += ctNames[i]; summary += "="; summary += std::to_string(checkTypeCounts[i]);
                                    }
                                }
                                LOG_WARNING("Warden: (async) Parsed ", checkCount, " checks [", summary,
                                            "] resultSize=", resultData.size());
                                std::string fullHex;
                                for (size_t bi = 0; bi < resultData.size(); bi++) {
                                    char hx[4]; snprintf(hx, 4, "%02x ", resultData[bi]); fullHex += hx;
                                    if ((bi + 1) % 32 == 0 && bi + 1 < resultData.size()) fullHex += "\n                    ";
                                }
                                LOG_WARNING("Warden: RESPONSE_HEX [", fullHex, "]");
                            }

                            // Build plaintext response: [0x02][uint16 len][uint32 checksum][resultData]
                            auto resultHash = auth::Crypto::sha1(resultData);
                            uint32_t checksum = 0;
                            for (int i = 0; i < 5; i++) {
                                uint32_t word = resultHash[i*4] | (uint32_t(resultHash[i*4+1])<<8)
                                              | (uint32_t(resultHash[i*4+2])<<16) | (uint32_t(resultHash[i*4+3])<<24);
                                checksum ^= word;
                            }
                            uint16_t rl = static_cast<uint16_t>(resultData.size());
                            std::vector<uint8_t> resp;
                            resp.push_back(0x02);
                            resp.push_back(rl & 0xFF); resp.push_back((rl >> 8) & 0xFF);
                            resp.push_back(checksum & 0xFF); resp.push_back((checksum >> 8) & 0xFF);
                            resp.push_back((checksum >> 16) & 0xFF); resp.push_back((checksum >> 24) & 0xFF);
                            resp.insert(resp.end(), resultData.begin(), resultData.end());
                            return resp; // plaintext; main thread will encrypt + send
                        });
                    wardenResponsePending_ = true;
                    break; // exit case 0x02 - response will be sent from update()
                }
            }

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
                static constexpr uint8_t kPacketProcessSanityPattern[] = {
                    0x33, 0xD2, 0x33, 0xC9, 0xE8, 0x87, 0x07, 0x1B, 0x00, 0xE8
                };
                if (offset == 13856 && length == sizeof(kPacketProcessSanityPattern) &&
                    hashPattern(kPacketProcessSanityPattern, sizeof(kPacketProcessSanityPattern))) {
                    return true;
                }

                // Scripted sanity check: "Warden Memory Read check" in wardenwin.cpp
                static constexpr uint8_t kWardenMemoryReadPattern[] = {
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

                LOG_DEBUG("Warden: Check #", checkCount, " type=", checkTypeNames[ct],
                         " at offset ", pos - 1);

                switch (ct) {
                    case CT_TIMING: {
                        // No additional request data
                        // Response: [uint8 result][uint32 ticks]
                        resultData.push_back(0x01);
                        uint32_t ticks = static_cast<uint32_t>(
                            std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now().time_since_epoch()).count());
                        resultData.push_back(ticks & 0xFF);
                        resultData.push_back((ticks >> 8) & 0xFF);
                        resultData.push_back((ticks >> 16) & 0xFF);
                        resultData.push_back((ticks >> 24) & 0xFF);
                        LOG_WARNING("Warden:   (sync) TIMING ticks=", ticks);
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
                        LOG_WARNING("Warden:   (sync) MEM offset=0x", [&]{char s[12];snprintf(s,12,"%08x",offset);return std::string(s);}(),
                                 " len=", (int)readLen,
                                 moduleName.empty() ? "" : (" module=\"" + moduleName + "\""));

                        // Lazy-load WoW.exe PE image on first MEM_CHECK
                        if (!wardenMemory_) {
                            wardenMemory_ = std::make_unique<WardenMemory>();
                            if (!wardenMemory_->load(static_cast<uint16_t>(owner_.getBuild()), isActiveExpansion("turtle"))) {
                                LOG_WARNING("Warden: Could not load WoW.exe for MEM_CHECK");
                            }
                        }

                        // Dynamically update LastHardwareAction before reading
                        if (offset == 0x00CF0BC8 && readLen == 4 && wardenMemory_ && wardenMemory_->isLoaded()) {
                            uint32_t now = static_cast<uint32_t>(
                                std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now().time_since_epoch()).count());
                            wardenMemory_->writeLE32(0xCF0BC8, now - 2000);
                        }

                        // Read bytes from PE image (includes patched runtime globals)
                        std::vector<uint8_t> memBuf(readLen, 0);
                        if (wardenMemory_->isLoaded() && wardenMemory_->readMemory(offset, readLen, memBuf.data())) {
                            LOG_DEBUG("Warden:   MEM_CHECK served from PE image");
                            resultData.push_back(0x00);
                            resultData.insert(resultData.end(), memBuf.begin(), memBuf.end());
                        } else {
                            // Address not in PE/KUSER - return 0xE9 (not readable).
                            LOG_WARNING("Warden:   (sync) MEM_CHECK -> 0xE9 (unmapped 0x",
                                        [&]{char s[12];snprintf(s,12,"%08x",offset);return std::string(s);}(), ")");
                            resultData.push_back(0xE9);
                        }
                        break;
                    }
                    case CT_PAGE_A: {
                        // Classic has seen two PAGE_A layouts in the wild:
                        // short: [4 seed][20 sha1] = 24 bytes
                        // long:  [4 seed][20 sha1][4 addr][1 len] = 29 bytes
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
                                pageResult = 0x4A;
                            } else if (wardenMemory_ && wardenMemory_->isLoaded() && len > 0) {
                                if (wardenMemory_->searchCodePattern(seedBytes, reqHash, len, true, off))
                                    pageResult = 0x4A;
                            }
                            // Turtle PAGE_A fallback: runtime-patched offsets aren't in the
                            // on-disk PE. Server expects "found" for code integrity checks.
                            if (pageResult == 0x00 && isActiveExpansion("turtle") && off < 0x600000) {
                                pageResult = 0x4A;
                                LOG_WARNING("Warden:   PAGE_A turtle-fallback for offset=0x",
                                            [&]{char s[12];snprintf(s,12,"%08x",off);return std::string(s);}());
                            }
                        }
                        if (consume >= 29) {
                            uint32_t off2 = uint32_t((decrypted.data()+pos)[24]) | (uint32_t((decrypted.data()+pos)[25])<<8) |
                                            (uint32_t((decrypted.data()+pos)[26])<<16) | (uint32_t((decrypted.data()+pos)[27])<<24);
                            uint8_t len2 = (decrypted.data()+pos)[28];
                            LOG_WARNING("Warden:   (sync) PAGE_A offset=0x",
                                        [&]{char s[12];snprintf(s,12,"%08x",off2);return std::string(s);}(),
                                        " patLen=", (int)len2,
                                        " result=0x", [&]{char s[4];snprintf(s,4,"%02x",pageResult);return std::string(s);}());
                        } else {
                            LOG_WARNING("Warden:   (sync) PAGE_A (short ", consume, "b) result=0x",
                                        [&]{char s[4];snprintf(s,4,"%02x",pageResult);return std::string(s);}());
                        }
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
                        LOG_DEBUG("Warden:   PAGE_B request bytes=", consume,
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
                        LOG_WARNING("Warden:   (sync) MPQ file=\"", (filePath.empty() ? "?" : filePath), "\"");

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

                            auto* am = owner_.services().assetManager;
                            if (am && am->isInitialized() && !found) {
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

                        if (found) {
                            resultData.push_back(0x00);
                            resultData.insert(resultData.end(), hash.begin(), hash.end());
                        } else {
                            resultData.push_back(0x01);
                        }
                        LOG_WARNING("Warden:   (sync) MPQ result=", found ? "FOUND" : "NOT_FOUND");
                        break;
                    }
                    case CT_LUA: {
                        // Request: [1 stringIdx]
                        if (pos + 1 > checkEnd) { pos = checkEnd; break; }
                        uint8_t strIdx = decrypted[pos++];
                        std::string luaVar = resolveWardenString(strIdx);
                        LOG_WARNING("Warden:   (sync) LUA str=\"", (luaVar.empty() ? "?" : luaVar), "\"");
                        resultData.push_back(0x01); // not found
                        break;
                    }
                    case CT_DRIVER: {
                        // Request: [4 seed][20 sha1][1 stringIdx]
                        if (pos + 25 > checkEnd) { pos = checkEnd; break; }
                        pos += 24; // skip seed + sha1
                        uint8_t strIdx = decrypted[pos++];
                        std::string driverName = resolveWardenString(strIdx);
                        LOG_WARNING("Warden:   (sync) DRIVER=\"", (driverName.empty() ? "?" : driverName), "\" -> 0x00(not found)");
                        resultData.push_back(0x00);
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

                            bool shouldReportFound = false;
                            std::string modName = "?";
                            // Wanted system modules
                            if (hmacSha1Matches(seedBytes, "KERNEL32.DLL", reqHash)) { modName = "KERNEL32.DLL"; shouldReportFound = true; }
                            else if (hmacSha1Matches(seedBytes, "USER32.DLL", reqHash)) { modName = "USER32.DLL"; shouldReportFound = true; }
                            else if (hmacSha1Matches(seedBytes, "NTDLL.DLL", reqHash)) { modName = "NTDLL.DLL"; shouldReportFound = true; }
                            else if (hmacSha1Matches(seedBytes, "WS2_32.DLL", reqHash)) { modName = "WS2_32.DLL"; shouldReportFound = true; }
                            else if (hmacSha1Matches(seedBytes, "WSOCK32.DLL", reqHash)) { modName = "WSOCK32.DLL"; shouldReportFound = true; }
                            else if (hmacSha1Matches(seedBytes, "ADVAPI32.DLL", reqHash)) { modName = "ADVAPI32.DLL"; shouldReportFound = true; }
                            else if (hmacSha1Matches(seedBytes, "SHELL32.DLL", reqHash)) { modName = "SHELL32.DLL"; shouldReportFound = true; }
                            else if (hmacSha1Matches(seedBytes, "GDI32.DLL", reqHash)) { modName = "GDI32.DLL"; shouldReportFound = true; }
                            else if (hmacSha1Matches(seedBytes, "OPENGL32.DLL", reqHash)) { modName = "OPENGL32.DLL"; shouldReportFound = true; }
                            else if (hmacSha1Matches(seedBytes, "WINMM.DLL", reqHash)) { modName = "WINMM.DLL"; shouldReportFound = true; }
                            // Unwanted cheat modules
                            else if (hmacSha1Matches(seedBytes, "WPESPY.DLL", reqHash)) modName = "WPESPY.DLL";
                            else if (hmacSha1Matches(seedBytes, "SPEEDHACK-I386.DLL", reqHash)) modName = "SPEEDHACK-I386.DLL";
                            else if (hmacSha1Matches(seedBytes, "TAMIA.DLL", reqHash)) modName = "TAMIA.DLL";
                            else if (hmacSha1Matches(seedBytes, "PRXDRVPE.DLL", reqHash)) modName = "PRXDRVPE.DLL";
                            else if (hmacSha1Matches(seedBytes, "D3DHOOK.DLL", reqHash)) modName = "D3DHOOK.DLL";
                            else if (hmacSha1Matches(seedBytes, "NJUMD.DLL", reqHash)) modName = "NJUMD.DLL";
                            LOG_WARNING("Warden:   (sync) MODULE \"", modName,
                                        "\" -> 0x", [&]{char s[4];snprintf(s,4,"%02x",shouldReportFound?0x4A:0x00);return std::string(s);}(),
                                        "(", shouldReportFound ? "found" : "not found", ")");
                            resultData.push_back(shouldReportFound ? 0x4A : 0x00);
                            break;
                        }
                        // Truncated module request fallback: module NOT loaded = clean
                        resultData.push_back(0x00);
                        break;
                    }
                    case CT_PROC: {
                        // API_CHECK request:
                        // [4 seed][20 sha1][1 stringIdx][1 stringIdx2][4 offset] = 30 bytes
                        int procSize = 30;
                        if (pos + procSize > checkEnd) { pos = checkEnd; break; }
                        pos += procSize;
                        LOG_WARNING("Warden:   (sync) PROC check -> 0x01(not found)");
                        resultData.push_back(0x01);
                        break;
                    }
                    default: {
                        uint8_t rawByte = decrypted[pos - 1];
                        uint8_t decoded = rawByte ^ xorByte;
                        LOG_WARNING("Warden: Unknown check type raw=0x",
                                    [&]{char s[4];snprintf(s,4,"%02x",rawByte);return std::string(s);}(),
                                    " decoded=0x",
                                    [&]{char s[4];snprintf(s,4,"%02x",decoded);return std::string(s);}(),
                                    " xorByte=0x",
                                    [&]{char s[4];snprintf(s,4,"%02x",xorByte);return std::string(s);}(),
                                    " opcodes=[",
                                    [&]{std::string r;for(uint8_t wardenCheckOpcode : wardenCheckOpcodes_){char s[6];snprintf(s,6,"0x%02x ",wardenCheckOpcode);r+=s;}return r;}(),
                                    "] pos=", pos, "/", checkEnd);
                        pos = checkEnd; // stop parsing
                        break;
                    }
                }
            }

            // Log synchronous round summary at WARNING level for diagnostics
            {
                LOG_WARNING("Warden: (sync) Parsed ", checkCount, " checks, resultSize=", resultData.size());
                std::string fullHex;
                for (size_t bi = 0; bi < resultData.size(); bi++) {
                    char hx[4]; snprintf(hx, 4, "%02x ", resultData[bi]); fullHex += hx;
                    if ((bi + 1) % 32 == 0 && bi + 1 < resultData.size()) fullHex += "\n                    ";
                }
                LOG_WARNING("Warden: (sync) RESPONSE_HEX [", fullHex, "]");
            }

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
            LOG_DEBUG("Warden: Sent CHEAT_CHECKS_RESULT (", resp.size(), " bytes, ",
                     checkCount, " checks, checksum=0x",
                     [&]{char s[12];snprintf(s,12,"%08x",checksum);return std::string(s);}(), ")");
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
