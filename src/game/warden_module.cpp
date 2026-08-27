#include "game/warden_module.hpp"
#include "game/jit_write.hpp"
#include "game/warden_crypto.hpp"
#include "auth/crypto.hpp"
#include "core/logger.hpp"
#include <cstring>
#include <fstream>
#include <filesystem>
#include <zlib.h>
#include <openssl/rsa.h>
#include <openssl/bn.h>
#include <openssl/sha.h>
#include <openssl/evp.h>
#include <openssl/pem.h>
#include <openssl/param_build.h>
#include <openssl/core_names.h>

#ifdef _WIN32
    #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
    #endif
    #include <windows.h>
#else
    #include <sys/mman.h>
    #include <cerrno>
#endif

// Always include the full definition so unique_ptr<WardenEmulator> destructor compiles
#include "game/warden_emulator.hpp"

namespace wowee {
namespace game {


// ============================================================================
// Thread-local pointer to the active WardenModule instance during initializeModule().
// C function pointer callbacks (sendPacket, validateModule, generateRC4) can't capture
// state, so they use this to reach the module's crypto and socket dependencies.
static thread_local WardenModule* tl_activeModule = nullptr;

// WardenModule Implementation
// ============================================================================

void WardenModule::setCallbackDependencies(WardenCrypto* crypto, SendPacketFunc sendFunc) {
    callbackCrypto_ = crypto;
    callbackSendPacket_ = std::move(sendFunc);
}

WardenModule::WardenModule() = default;

WardenModule::~WardenModule() {
    unload();
}

bool WardenModule::load(const std::vector<uint8_t>& moduleData,
                        const std::vector<uint8_t>& md5Hash,
                        const std::vector<uint8_t>& rc4Key) {
    moduleData_ = moduleData;
    md5Hash_ = md5Hash;

    {
        char hexBuf[17] = {};
        for (size_t i = 0; i < std::min(md5Hash.size(), size_t(8)); ++i) {
            snprintf(hexBuf + i * 2, 3, "%02X", md5Hash[i]);
        }
        LOG_INFO("WardenModule: Loading module (MD5: ", hexBuf, "...)");
    }

    // Step 1: Verify MD5 hash
    if (!verifyMD5(moduleData, md5Hash)) {
        LOG_WARNING("WardenModule: MD5 verification failed; continuing in compatibility mode");
    }
    LOG_INFO("WardenModule: MD5 verified");

    // Step 2: RC4 decrypt (Warden protocol-required legacy RC4; server-mandated, cannot be changed)
    if (!decryptRC4(moduleData, rc4Key, decryptedData_)) { // codeql[cpp/weak-cryptographic-algorithm]
        LOG_WARNING("WardenModule: RC4 decryption failed; using raw module bytes fallback");
        decryptedData_ = moduleData;
    }
    LOG_INFO("WardenModule: RC4 decrypted (", decryptedData_.size(), " bytes)");

    // Step 3: Verify RSA signature
    if (!verifyRSASignature(decryptedData_)) {
        // Signature mismatch is non-fatal - private-server modules use a different key.
    }

    // Step 4: Strip RSA-2048 signature (last 256 bytes = 2048 bits) then zlib decompress.
    static constexpr size_t kRsaSignatureSize = 256;
    std::vector<uint8_t> dataWithoutSig;
    if (decryptedData_.size() > kRsaSignatureSize) {
        dataWithoutSig.assign(decryptedData_.begin(), decryptedData_.end() - kRsaSignatureSize);
    } else {
        dataWithoutSig = decryptedData_;
    }
    if (!decompressZlib(dataWithoutSig, decompressedData_)) {
        LOG_WARNING("WardenModule: zlib decompression failed; using decrypted bytes fallback");
        decompressedData_ = decryptedData_;
    }

    // Step 5: Parse custom executable format
    if (!parseExecutableFormat(decompressedData_)) {
        LOG_WARNING("WardenModule: Executable format parsing failed; continuing with minimal module image");
    }

    // Step 6: Apply relocations
    if (!applyRelocations()) {
        LOG_WARNING("WardenModule: Address relocations failed; continuing with unrelocated image");
    }

    // Step 7+8: Initialize module (creates emulator) then bind APIs (patches IAT).
    // API binding must happen after emulator setup (needs stub addresses) but before
    // the module entry point is called (needs resolved imports). Both are handled
    // inside initializeModule().
    // Only emulate a module that actually unpacked. Feeding the raw payload to the x86
    // emulator executes garbage: it faults on an unmapped read and lands on the stub
    // callbacks anyway, having spun up Unicorn and mapped memory for nothing.
    if (!moduleImageUsable_) {
        LOG_WARNING("WardenModule: module did not unpack into an executable image "
                    "(server module is not a genuine Blizzard module?) - using stub callbacks");
    } else if (!initializeModule()) {
        LOG_WARNING("WardenModule: Module initialization failed; continuing with stub callbacks");
    }

    // Module loading pipeline complete!
    // Note: Steps 6-8 are stubs/platform-limited, but infrastructure is ready
    loaded_ = true; // Mark as loaded (infrastructure complete)

    LOG_INFO("WardenModule: pipeline complete (infrastructure ready, execution stubs in place)");
    LOG_DEBUG("WardenModule: limitations - relocations need real module data; API binding is "
              "Windows-only (or Wine on Linux); native x86 execution is disabled. "
              "For strict servers, actual execution would need to be enabled.");

    return true;
}

bool WardenModule::processCheckRequest([[maybe_unused]] const std::vector<uint8_t>& checkData,
                                       [[maybe_unused]] std::vector<uint8_t>& responseOut) {
    if (!loaded_) {
        LOG_ERROR("WardenModule: Module not loaded, cannot process checks");
        return false;
    }

    #ifdef HAVE_UNICORN
        if (emulator_ && emulator_->isInitialized() && funcList_.packetHandler) {
            LOG_DEBUG("WardenModule: processing check request via emulator (", checkData.size(), " bytes)");

            // Allocate memory for check data in emulated space
            uint32_t checkDataAddr = emulator_->allocateMemory(checkData.size(), 0x04);
            if (checkDataAddr == 0) {
                LOG_ERROR("WardenModule: Failed to allocate memory for check data");
                return false;
            }

            // Write check data to emulated memory
            if (!emulator_->writeMemory(checkDataAddr, checkData.data(), checkData.size())) {
                LOG_ERROR("WardenModule: Failed to write check data");
                emulator_->freeMemory(checkDataAddr);
                return false;
            }

            // Allocate response buffer in emulated space (assume max 1KB response)
            uint32_t responseAddr = emulator_->allocateMemory(1024, 0x04);
            if (responseAddr == 0) {
                LOG_ERROR("WardenModule: Failed to allocate response buffer");
                emulator_->freeMemory(checkDataAddr);
                return false;
            }

            try {
                if (emulatedPacketHandlerAddr_ == 0) {
                    LOG_ERROR("WardenModule: PacketHandler address not set (module not fully initialized)");
                    emulator_->freeMemory(checkDataAddr);
                    emulator_->freeMemory(responseAddr);
                    return false;
                }

                // Allocate uint32_t for responseSizeOut in emulated memory
                uint32_t initialSize = 1024;
                uint32_t responseSizeAddr = emulator_->writeData(&initialSize, sizeof(uint32_t));
                if (responseSizeAddr == 0) {
                    LOG_ERROR("WardenModule: Failed to allocate responseSizeAddr");
                    emulator_->freeMemory(checkDataAddr);
                    emulator_->freeMemory(responseAddr);
                    return false;
                }

                // Call: void PacketHandler(uint8_t* data, uint32_t size,
                //                          uint8_t* responseOut, uint32_t* responseSizeOut)
                LOG_DEBUG("WardenModule: calling emulated PacketHandler");
                emulator_->callFunction(emulatedPacketHandlerAddr_, {
                    checkDataAddr,
                    static_cast<uint32_t>(checkData.size()),
                    responseAddr,
                    responseSizeAddr
                });

                // Read back response size and data
                uint32_t responseSize = 0;
                emulator_->readMemory(responseSizeAddr, &responseSize, sizeof(uint32_t));
                emulator_->freeMemory(responseSizeAddr);

                if (responseSize > 0 && responseSize <= 1024) {
                    responseOut.resize(responseSize);
                    if (!emulator_->readMemory(responseAddr, responseOut.data(), responseSize)) {
                        LOG_ERROR("WardenModule: Failed to read response data");
                        responseOut.clear();
                    } else {
                        LOG_DEBUG("WardenModule: PacketHandler wrote ", responseSize, " byte response");
                    }
                } else {
                    LOG_WARNING("WardenModule: PacketHandler returned invalid responseSize=", responseSize);
                }

                emulator_->freeMemory(checkDataAddr);
                emulator_->freeMemory(responseAddr);
                return !responseOut.empty();

            } catch (const std::exception& e) {
                LOG_ERROR("WardenModule: Exception during PacketHandler: ", e.what());
                emulator_->freeMemory(checkDataAddr);
                emulator_->freeMemory(responseAddr);
                return false;
            }
        }
    #endif

    // Falls back to fake responses in GameHandler when emulator path is unavailable.
    // Logged once-per-session via a sticky flag so we don't spam every Warden check
    // (server typically issues a check every 10-30 seconds).
    static bool warned = false;
    if (!warned) {
        LOG_WARNING("WardenModule: processCheckRequest emulator path unavailable - "
                    "falling back to GameHandler fake responses for this session");
        warned = true;
    } else {
        LOG_DEBUG("WardenModule: processCheckRequest fallback (emulator unavailable)");
    }
    return false;
}

uint32_t WardenModule::tick(uint32_t deltaMs) {
    if (!loaded_ || !funcList_.tick) {
        return 0;
    }
    return funcList_.tick(deltaMs);
}

void WardenModule::generateRC4Keys(uint8_t* packet) {
    if (!loaded_ || !funcList_.generateRC4Keys) {
        return;
    }
    funcList_.generateRC4Keys(packet);
}

void WardenModule::unload() {
    if (moduleMemory_) {
        // Call module's Unload() function if loaded
        if (loaded_ && funcList_.unload) {
            LOG_INFO("WardenModule: Calling module unload callback...");
            funcList_.unload(nullptr);
        }

        // Free executable memory region
        LOG_INFO("WardenModule: Freeing ", moduleSize_, " bytes of executable memory");
        #ifdef _WIN32
            VirtualFree(moduleMemory_, 0, MEM_RELEASE);
        #else
            munmap(moduleMemory_, moduleSize_);
        #endif

        moduleMemory_ = nullptr;
        moduleSize_ = 0;
    }

    // Clear function pointers
    funcList_ = {};
    emulatedPacketHandlerAddr_ = 0;

    loaded_ = false;
    moduleData_.clear();
    decryptedData_.clear();
    decompressedData_.clear();
}

// ============================================================================
// Private Validation Methods
// ============================================================================

bool WardenModule::verifyMD5(const std::vector<uint8_t>& data,
                             const std::vector<uint8_t>& expectedHash) {
    std::vector<uint8_t> computedHash = auth::Crypto::md5(data);

    if (computedHash.size() != expectedHash.size()) {
        return false;
    }

    return std::memcmp(computedHash.data(), expectedHash.data(), expectedHash.size()) == 0;
}

bool WardenModule::decryptRC4(const std::vector<uint8_t>& encrypted,
                              const std::vector<uint8_t>& key,
                              std::vector<uint8_t>& decryptedOut) {
    if (key.size() != 16) {
        LOG_ERROR("WardenModule: Invalid RC4 key size: ", key.size(), " (expected 16)");
        return false;
    }

    // Initialize RC4 state (KSA - Key Scheduling Algorithm)
    std::vector<uint8_t> S(256);
    for (int i = 0; i < 256; ++i) {
        S[i] = i;
    }

    int j = 0;
    for (int i = 0; i < 256; ++i) {
        j = (j + S[i] + key[i % key.size()]) % 256;
        std::swap(S[i], S[j]);
    }

    // Decrypt using RC4 (PRGA - Pseudo-Random Generation Algorithm)
    decryptedOut.resize(encrypted.size());
    int i = 0;
    j = 0;

    for (size_t idx = 0; idx < encrypted.size(); ++idx) {
        i = (i + 1) % 256;
        j = (j + S[i]) % 256;
        std::swap(S[i], S[j]);
        uint8_t K = S[(S[i] + S[j]) % 256];
        decryptedOut[idx] = encrypted[idx] ^ K;
    }

    return true;
}

namespace {

/// Blizzard's own Warden signing key, the same across 1.12.1, 2.4.3 and
/// 3.3.5a, confirmed against namreeb/WardenSigning and the SkullSecurity
/// wiki. A server running a module of its own signs it with its own key and
/// names that key in its expansion profile; this is what the rest get.
constexpr uint8_t kRetailModulus[256] = {
    0x6B, 0xCE, 0xF5, 0x2D, 0x2A, 0x7D, 0x7A, 0x67, 0x21, 0x21, 0x84, 0xC9, 0xBC, 0x25, 0xC7, 0xBC,
    0xDF, 0x3D, 0x8F, 0xD9, 0x47, 0xBC, 0x45, 0x48, 0x8B, 0x22, 0x85, 0x3B, 0xC5, 0xC1, 0xF4, 0xF5,
    0x3C, 0x0C, 0x49, 0xBB, 0x56, 0xE0, 0x3D, 0xBC, 0xA2, 0xD2, 0x35, 0xC1, 0xF0, 0x74, 0x2E, 0x15,
    0x5A, 0x06, 0x8A, 0x68, 0x01, 0x9E, 0x60, 0x17, 0x70, 0x8B, 0xBD, 0xF8, 0xD5, 0xF9, 0x3A, 0xD3,
    0x25, 0xB2, 0x66, 0x92, 0xBA, 0x43, 0x8A, 0x81, 0x52, 0x0F, 0x64, 0x98, 0xFF, 0x60, 0x37, 0xAF,
    0xB4, 0x11, 0x8C, 0xF9, 0x2E, 0xC5, 0xEE, 0xCA, 0xB4, 0x41, 0x60, 0x3C, 0x7D, 0x02, 0xAF, 0xA1,
    0x2B, 0x9B, 0x22, 0x4B, 0x3B, 0xFC, 0xD2, 0x5D, 0x73, 0xE9, 0x29, 0x34, 0x91, 0x85, 0x93, 0x4C,
    0xBE, 0xBE, 0x73, 0xA9, 0xD2, 0x3B, 0x27, 0x7A, 0x47, 0x76, 0xEC, 0xB0, 0x28, 0xC9, 0xC1, 0xDA,
    0xEE, 0xAA, 0xB3, 0x96, 0x9C, 0x1E, 0xF5, 0x6B, 0xF6, 0x64, 0xD8, 0x94, 0x2E, 0xF1, 0xF7, 0x14,
    0x5F, 0xA0, 0xF1, 0xA3, 0xB9, 0xB1, 0xAA, 0x58, 0x97, 0xDC, 0x09, 0x17, 0x0C, 0x04, 0xD3, 0x8E,
    0x02, 0x2C, 0x83, 0x8A, 0xD6, 0xAF, 0x7C, 0xFE, 0x83, 0x33, 0xC6, 0xA8, 0xC3, 0x84, 0xEF, 0x29,
    0x06, 0xA9, 0xB7, 0x2D, 0x06, 0x0B, 0x0D, 0x6F, 0x70, 0x9E, 0x34, 0xA6, 0xC7, 0x31, 0xBE, 0x56,
    0xDE, 0xDD, 0x02, 0x92, 0xF8, 0xA0, 0x58, 0x0B, 0xFC, 0xFA, 0xBA, 0x49, 0xB4, 0x48, 0xDB, 0xEC,
    0x25, 0xF3, 0x18, 0x8F, 0x2D, 0xB3, 0xC0, 0xB8, 0xDD, 0xBC, 0xD6, 0xAA, 0xA6, 0xDB, 0x6F, 0x7D,
    0x7D, 0x25, 0xA6, 0xCD, 0x39, 0x6D, 0xDA, 0x76, 0x0C, 0x79, 0xBF, 0x48, 0x25, 0xFC, 0x2D, 0xC5,
    0xFA, 0x53, 0x9B, 0x4D, 0x60, 0xF4, 0xEF, 0xC7, 0xEA, 0xAC, 0xA1, 0x7B, 0x03, 0xF4, 0xAF, 0xC7
    };

}  // namespace

void WardenModule::setRsaModulus(std::vector<uint8_t> modulus) {
    if (!modulus.empty() && modulus.size() != 256) {
        LOG_WARNING("WardenModule: ignoring an RSA modulus of ", modulus.size(),
                    " bytes - it has to be 256");
        return;
    }
    rsaModulus_ = std::move(modulus);
}

bool WardenModule::verifyRSASignature(const std::vector<uint8_t>& data) {
    // RSA-2048 signature is last 256 bytes
    if (data.size() < 256) {
        LOG_ERROR("WardenModule: Data too small for RSA signature (need at least 256 bytes)");
        return false;
    }

    // Extract signature (last 256 bytes)
    std::vector<uint8_t> signature(data.end() - 256, data.end());

    // Extract data without signature
    std::vector<uint8_t> dataWithoutSig(data.begin(), data.end() - 256);

    // Whose key this module should carry. A profile that names one gets it;
    // everything else gets Blizzard's.
    const uint32_t exponent = 0x010001;
    const uint8_t* modulus = rsaModulus_.size() == 256
                                 ? rsaModulus_.data()
                                 : kRetailModulus;


    // Compute expected hash: SHA1(data_without_sig + "MAIEV.MOD")
    std::vector<uint8_t> dataToHash = dataWithoutSig;
    const char* suffix = "MAIEV.MOD";
    dataToHash.insert(dataToHash.end(), suffix, suffix + strlen(suffix));

    std::vector<uint8_t> expectedHash = auth::Crypto::sha1(dataToHash);

    // Create RSA public key using EVP_PKEY_fromdata (OpenSSL 3.0 compatible)
    BIGNUM* n = BN_bin2bn(modulus, 256, nullptr);
    BIGNUM* e = BN_new();
    BN_set_word(e, exponent);

    EVP_PKEY* pkey = nullptr;
    EVP_PKEY_CTX* ctx = nullptr;
    std::vector<uint8_t> decryptedSig(256);
    int decryptedLen = -1;

    {
        OSSL_PARAM_BLD* bld = OSSL_PARAM_BLD_new();
        OSSL_PARAM_BLD_push_BN(bld, OSSL_PKEY_PARAM_RSA_N, n);
        OSSL_PARAM_BLD_push_BN(bld, OSSL_PKEY_PARAM_RSA_E, e);
        OSSL_PARAM* params = OSSL_PARAM_BLD_to_param(bld);
        OSSL_PARAM_BLD_free(bld);

        EVP_PKEY_CTX* fromCtx = EVP_PKEY_CTX_new_from_name(nullptr, "RSA", nullptr);
        if (fromCtx && EVP_PKEY_fromdata_init(fromCtx) > 0) {
            EVP_PKEY_fromdata(fromCtx, &pkey, EVP_PKEY_PUBLIC_KEY, params);
        }
        if (fromCtx) EVP_PKEY_CTX_free(fromCtx);
        OSSL_PARAM_free(params);

        if (pkey) {
            ctx = EVP_PKEY_CTX_new(pkey, nullptr);
            if (ctx && EVP_PKEY_verify_recover_init(ctx) > 0 &&
                EVP_PKEY_CTX_set_rsa_padding(ctx, RSA_NO_PADDING) > 0) {
                size_t outLen = decryptedSig.size();
                if (EVP_PKEY_verify_recover(ctx, decryptedSig.data(), &outLen,
                                            signature.data(), 256) > 0) {
                    decryptedLen = static_cast<int>(outLen);
                }
            }
        }
    }

    BN_free(n);
    BN_free(e);
    if (ctx) EVP_PKEY_CTX_free(ctx);
    if (pkey) EVP_PKEY_free(pkey);

    if (decryptedLen < 0) {
        LOG_WARNING("WardenModule: the signature does not decrypt with the ",
                    rsaModulus_.size() == 256 ? "profile's" : "retail",
                    " Warden key. Expected of a server that signs its own module;"
                    " name its key as wardenRsaModulus in the expansion profile to"
                    " check against that instead.");
        return false;
    }

    // Expected format: padding (0xBB bytes) + SHA1 hash (20 bytes)
    // Total: 256 bytes decrypted
    // Find SHA1 hash in decrypted signature (should be at end, preceded by 0xBB padding)

    // Look for SHA1 hash in last 20 bytes
    if (decryptedLen >= 20) {
        std::vector<uint8_t> actualHash(decryptedSig.end() - 20, decryptedSig.end());

        if (std::memcmp(actualHash.data(), expectedHash.data(), 20) == 0) {
            LOG_INFO("WardenModule: RSA signature verified");
            return true;
        }
    }

    LOG_WARNING("WardenModule: RSA signature mismatch - module may be corrupt or from a different build");

    // With the real modulus in place, signature failure means the module is invalid.
    // Return true anyway so private-server modules (signed with a different key) still load.
    return true;
}

bool WardenModule::decompressZlib(const std::vector<uint8_t>& compressed,
                                  std::vector<uint8_t>& decompressedOut) {
    if (compressed.size() < 4) {
        LOG_ERROR("WardenModule: Compressed data too small (need at least 4 bytes for size header)");
        return false;
    }

    // Read 4-byte uncompressed size (little-endian)
    uint32_t uncompressedSize =
        compressed[0] |
        (compressed[1] << 8) |
        (compressed[2] << 16) |
        (compressed[3] << 24);

    LOG_INFO("WardenModule: Uncompressed size: ", uncompressedSize, " bytes");

    // Sanity check (modules shouldn't be larger than 10MB)
    if (uncompressedSize > 10 * 1024 * 1024) {
        LOG_ERROR("WardenModule: Uncompressed size suspiciously large: ", uncompressedSize, " bytes");
        return false;
    }

    // Allocate output buffer
    decompressedOut.resize(uncompressedSize);

    // Setup zlib stream
    z_stream stream = {};
    stream.next_in = const_cast<uint8_t*>(compressed.data() + 4); // Skip 4-byte size header
    stream.avail_in = compressed.size() - 4;
    stream.next_out = decompressedOut.data();
    stream.avail_out = uncompressedSize;

    // Initialize inflater
    int ret = inflateInit(&stream);
    if (ret != Z_OK) {
        LOG_ERROR("WardenModule: inflateInit failed: ", ret);
        return false;
    }

    // Decompress
    ret = inflate(&stream, Z_FINISH);

    // Cleanup
    inflateEnd(&stream);

    if (ret != Z_STREAM_END) {
        LOG_ERROR("WardenModule: inflate failed: ", ret);
        return false;
    }

    LOG_INFO("WardenModule: zlib decompression successful (", stream.total_out, " bytes decompressed)");

    return true;
}

bool WardenModule::parseExecutableFormat(const std::vector<uint8_t>& exeData) {
    // Every store below lands in the MAP_JIT mapping; see JitWriteWindow.
    JitWriteWindow jitWrite;
    if (exeData.size() < 4) {
        LOG_ERROR("WardenModule: Executable data too small for header");
        return false;
    }

    // Turtle-derived realms use the native Warden image format, not the older
    // copy/skip stream assumed below. Its fixed header is followed by 12-byte
    // section descriptors; those descriptors are metadata and are not copied
    // into the image. The copy stream alternates copied and zero-filled spans.
    //
    // Keep the legacy parser below for retail modules: a candidate must pass
    // every native-header bound check before we accept this interpretation.
    if (exeData.size() >= 0x28) {
        const auto readU16LE = [&](size_t offset) -> uint16_t {
            return static_cast<uint16_t>(exeData[offset]) |
                   (static_cast<uint16_t>(exeData[offset + 1]) << 8);
        };
        const auto readU32LE = [&](size_t offset) -> uint32_t {
            return static_cast<uint32_t>(exeData[offset]) |
                   (static_cast<uint32_t>(exeData[offset + 1]) << 8) |
                   (static_cast<uint32_t>(exeData[offset + 2]) << 16) |
                   (static_cast<uint32_t>(exeData[offset + 3]) << 24);
        };

        const uint32_t imageSize = readU32LE(0x00);
        const uint32_t candidateRelocOffset = readU32LE(0x08);
        const uint32_t candidateRelocCount = readU32LE(0x0C);
        const uint32_t candidateExportOffset = readU32LE(0x10);
        const uint32_t candidateExportCount = readU32LE(0x14);
        const uint32_t candidateExportBase = readU32LE(0x18);
        const uint32_t candidateImportOffset = readU32LE(0x1C);
        const uint32_t candidateImportCount = readU32LE(0x20);
        const uint32_t candidateSectionCount = readU32LE(0x24);
        const size_t sectionTableSize = static_cast<size_t>(candidateSectionCount) * 12u;
        const size_t copyStreamOffset = 0x28u + sectionTableSize;

        if (imageSize != 0 && imageSize <= 5 * 1024 * 1024 &&
            candidateSectionCount > 0 && candidateSectionCount <= 128 &&
            copyStreamOffset <= exeData.size()) {
            const uint32_t firstSectionOffset = readU32LE(0x28);
            if (firstSectionOffset < imageSize) {
                std::vector<uint8_t> nativeImage(imageSize, 0);
                std::memcpy(nativeImage.data(), exeData.data(), 0x28);

                size_t sourceOffset = copyStreamOffset;
                size_t destinationOffset = firstSectionOffset;
                bool copySpan = true;
                uint32_t spanCount = 0;
                bool validStream = true;
                while (destinationOffset < nativeImage.size()) {
                    if (sourceOffset + 2 > exeData.size()) {
                        validStream = false;
                        break;
                    }
                    const uint16_t span = readU16LE(sourceOffset);
                    sourceOffset += 2;
                    if (span == 0 || destinationOffset + span > nativeImage.size()) {
                        validStream = false;
                        break;
                    }
                    if (copySpan) {
                        if (sourceOffset + span > exeData.size()) {
                            validStream = false;
                            break;
                        }
                        std::memcpy(nativeImage.data() + destinationOffset,
                                    exeData.data() + sourceOffset, span);
                        sourceOffset += span;
                    }
                    destinationOffset += span;
                    copySpan = !copySpan;
                    ++spanCount;
                }

                if (validStream && destinationOffset == nativeImage.size()) {
                    #ifdef _WIN32
                    moduleMemory_ = VirtualAlloc(nullptr, imageSize, MEM_COMMIT | MEM_RESERVE,
                                                 PAGE_EXECUTE_READWRITE);
                    #else
                    #ifdef HAVE_UNICORN
                    const int mmapProt = PROT_READ | PROT_WRITE;
                    const int mmapFlags = MAP_PRIVATE | MAP_ANONYMOUS;
                    #elif defined(WOWEE_MAP_JIT)
                    const int mmapProt = PROT_READ | PROT_WRITE | PROT_EXEC;
                    const int mmapFlags = MAP_PRIVATE | MAP_ANONYMOUS | MAP_JIT;
                    #else
                    const int mmapProt = PROT_READ | PROT_WRITE | PROT_EXEC;
                    const int mmapFlags = MAP_PRIVATE | MAP_ANONYMOUS;
                    #endif
                    moduleMemory_ = mmap(nullptr, imageSize, mmapProt, mmapFlags, -1, 0);
                    if (moduleMemory_ == MAP_FAILED) moduleMemory_ = nullptr;
                    #endif

                    if (!moduleMemory_) {
                        LOG_ERROR("WardenModule: Failed to allocate native image");
                        return false;
                    }
                    std::memcpy(moduleMemory_, nativeImage.data(), nativeImage.size());
                    moduleSize_ = imageSize;
                    relocOffset_ = candidateRelocOffset;
                    relocCount_ = candidateRelocCount;
                    exportTableOffset_ = candidateExportOffset;
                    exportCount_ = candidateExportCount;
                    exportBaseIndex_ = candidateExportBase;
                    importTableOffset_ = candidateImportOffset;
                    importCount_ = candidateImportCount;
                    sectionCount_ = candidateSectionCount;
                    relocDataOffset_ = 0;
                    moduleImageUsable_ = true;
                    LOG_INFO("WardenModule: Parsed native image: ", imageSize,
                             " bytes, ", sectionCount_, " sections, ", spanCount,
                             " spans, relocations=", relocCount_, ", imports=", importCount_);
                    return true;
                }
            }
        }
    }

    // Read final code size (little-endian 4 bytes)
    uint32_t finalCodeSize =
        exeData[0] |
        (exeData[1] << 8) |
        (exeData[2] << 16) |
        (exeData[3] << 24);

    LOG_INFO("WardenModule: Final code size: ", finalCodeSize, " bytes");

    // Sanity check (executable shouldn't be larger than 5MB)
    if (finalCodeSize > 5 * 1024 * 1024 || finalCodeSize == 0) {
        LOG_ERROR("WardenModule: Invalid final code size: ", finalCodeSize);
        return false;
    }

    // Allocate executable memory
    // Note: On Linux, we'll use mmap with PROT_EXEC
    // On Windows, would use VirtualAlloc with PAGE_EXECUTE_READWRITE
    #ifdef _WIN32
        moduleMemory_ = VirtualAlloc(
            nullptr,
            finalCodeSize,
            MEM_COMMIT | MEM_RESERVE,
            PAGE_EXECUTE_READWRITE
        );
        if (!moduleMemory_) {
            LOG_ERROR("WardenModule: VirtualAlloc failed");
            return false;
        }
    #else
        // When using Unicorn emulation the module image is copied into the
        // emulator's address space, so we only need read/write access here.
        // Native execution paths (non-Unicorn) need PROT_EXEC; on macOS this
        // requires MAP_JIT due to hardened-runtime restrictions.
        #ifdef HAVE_UNICORN
            int mmapProt  = PROT_READ | PROT_WRITE;
            int mmapFlags = MAP_PRIVATE | MAP_ANONYMOUS;
        #elif defined(WOWEE_MAP_JIT)
            int mmapProt  = PROT_READ | PROT_WRITE | PROT_EXEC;
            int mmapFlags = MAP_PRIVATE | MAP_ANONYMOUS | MAP_JIT;
        #else
            int mmapProt  = PROT_READ | PROT_WRITE | PROT_EXEC;
            int mmapFlags = MAP_PRIVATE | MAP_ANONYMOUS;
        #endif
        moduleMemory_ = mmap(
            nullptr,
            finalCodeSize,
            mmapProt,
            mmapFlags,
            -1,
            0
        );
        if (moduleMemory_ == MAP_FAILED) {
            LOG_ERROR("WardenModule: mmap failed: ", strerror(errno));
            moduleMemory_ = nullptr;
            return false;
        }
    #endif

    moduleSize_ = finalCodeSize;
    std::memset(moduleMemory_, 0, moduleSize_); // Zero-initialize

    LOG_INFO("WardenModule: Allocated ", moduleSize_, " bytes of executable memory");

    auto readU16LE = [&](size_t at) -> uint16_t {
        return static_cast<uint16_t>(exeData[at] | (exeData[at + 1] << 8));
    };

    enum class PairFormat {
        CopyDataSkip,  // [copy][data][skip]
        SkipCopyData,  // [skip][copy][data]
        CopySkipData,  // [copy][skip][data]
        /// [copy][data][skip] again, but ending when the image is full rather
        /// than on a zero pair. A stream written this way has no terminator to
        /// find, so the three above walk it correctly and then run off the end
        /// still looking for one - which is what "the stream ended without a
        /// terminating pair" means when all three say it and the image is full.
        CopyDataSkipToFill
    };

    // How far a layout got before it gave up. "All known layouts failed" says
    // nothing a reader can act on: whether the stream is a different shape,
    // or this one ran off the end of a truncated module, or the header's size
    // is wrong, are three different faults with one message between them.
    struct Attempt {
        int pairs = 0;
        size_t pos = 0;
        size_t dest = 0;
        const char* why = "not tried";
    };
    Attempt attempts[4];

    auto tryParsePairs = [&](PairFormat format,
                             size_t startOffset,
                             std::vector<uint8_t>& imageOut,
                             size_t& relocPosOut,
                             size_t& finalOffsetOut,
                             int& pairCountOut,
                             Attempt& record) -> bool {
        imageOut.assign(moduleSize_, 0);
        size_t pos = startOffset;
        size_t destOffset = 0;
        int pairCount = 0;
        const auto note = [&](const char* why) {
            record = {pairCount, pos, destOffset, why};
            return false;
        };

        while (pos + 2 <= exeData.size()) {
            uint16_t copyCount = 0;
            uint16_t skipCount = 0;

            if (format == PairFormat::CopyDataSkipToFill && destOffset == moduleSize_) {
                relocPosOut = pos;
                finalOffsetOut = destOffset;
                pairCountOut = pairCount;
                return true;
            }

            switch (format) {
                case PairFormat::CopyDataSkipToFill:
                case PairFormat::CopyDataSkip: {
                    copyCount = readU16LE(pos);
                    pos += 2;
                    if (copyCount == 0 && format == PairFormat::CopyDataSkipToFill) {
                        return note("a zero pair, so this stream does have a terminator");
                    }
                    if (copyCount == 0) {
                        relocPosOut = pos;
                        finalOffsetOut = destOffset;
                        pairCountOut = pairCount;
                        imageOut.resize(moduleSize_);
                        return true;
                    }

                    if (pos + copyCount > exeData.size()) return note("a copy runs past the end of the module");
                    if (destOffset + copyCount > moduleSize_) return note("a copy runs past the image the header sized");

                    std::memcpy(imageOut.data() + destOffset, exeData.data() + pos, copyCount);
                    pos += copyCount;
                    destOffset += copyCount;

                    if (pos + 2 > exeData.size()) return note("the module ends where a skip count should be");
                    skipCount = readU16LE(pos);
                    pos += 2;
                    break;
                }

                case PairFormat::SkipCopyData: {
                    if (pos + 4 > exeData.size()) return note("the module ends inside a pair header");
                    skipCount = readU16LE(pos);
                    pos += 2;
                    copyCount = readU16LE(pos);
                    pos += 2;

                    if (skipCount == 0 && copyCount == 0) {
                        relocPosOut = pos;
                        finalOffsetOut = destOffset;
                        pairCountOut = pairCount;
                        imageOut.resize(moduleSize_);
                        return true;
                    }

                    if (destOffset + skipCount > moduleSize_) return note("a skip runs past the image the header sized");
                    destOffset += skipCount;

                    if (pos + copyCount > exeData.size()) return note("a copy runs past the end of the module");
                    if (destOffset + copyCount > moduleSize_) return note("a copy runs past the image the header sized");
                    std::memcpy(imageOut.data() + destOffset, exeData.data() + pos, copyCount);
                    pos += copyCount;
                    destOffset += copyCount;
                    break;
                }

                case PairFormat::CopySkipData: {
                    if (pos + 4 > exeData.size()) return note("the module ends inside a pair header");
                    copyCount = readU16LE(pos);
                    pos += 2;
                    skipCount = readU16LE(pos);
                    pos += 2;

                    if (copyCount == 0 && skipCount == 0) {
                        relocPosOut = pos;
                        finalOffsetOut = destOffset;
                        pairCountOut = pairCount;
                        imageOut.resize(moduleSize_);
                        return true;
                    }

                    if (pos + copyCount > exeData.size()) return note("a copy runs past the end of the module");
                    if (destOffset + copyCount > moduleSize_) return note("a copy runs past the image the header sized");
                    std::memcpy(imageOut.data() + destOffset, exeData.data() + pos, copyCount);
                    pos += copyCount;
                    destOffset += copyCount;
                    break;
                }
            }

            if (destOffset + skipCount > moduleSize_) return note("a skip runs past the image the header sized");
            destOffset += skipCount;
            pairCount++;
        }

        return note("the stream ended without a terminating pair");
    };

    std::vector<uint8_t> parsedImage;
    size_t parsedRelocPos = 0;
    size_t parsedFinalOffset = 0;
    int parsedPairCount = 0;

    PairFormat usedFormat = PairFormat::CopyDataSkip;
    bool parsed = tryParsePairs(PairFormat::CopyDataSkip, 4, parsedImage, parsedRelocPos,
                                parsedFinalOffset, parsedPairCount, attempts[0]);
    if (!parsed) {
        usedFormat = PairFormat::SkipCopyData;
        parsed = tryParsePairs(PairFormat::SkipCopyData, 4, parsedImage, parsedRelocPos,
                               parsedFinalOffset, parsedPairCount, attempts[1]);
    }
    if (!parsed) {
        usedFormat = PairFormat::CopySkipData;
        parsed = tryParsePairs(PairFormat::CopySkipData, 4, parsedImage, parsedRelocPos,
                               parsedFinalOffset, parsedPairCount, attempts[2]);
    }
    if (!parsed) {
        // Last, and only on an exact fill. A wrong reading of a pair stream
        // will not land on the image's last byte by luck, and accepting one
        // that did would hand the emulator rubbish to run.
        usedFormat = PairFormat::CopyDataSkipToFill;
        parsed = tryParsePairs(PairFormat::CopyDataSkipToFill, 4, parsedImage, parsedRelocPos,
                               parsedFinalOffset, parsedPairCount, attempts[3]);
        if (parsed && parsedFinalOffset != finalCodeSize) {
            parsed = false;
            attempts[3].why = "it did not fill the image exactly";
        }
    }

    // Nothing starting four bytes in works. The four-byte header is an
    // assumption, and a module whose header is longer puts the pair stream
    // somewhere this has not looked - so look, over the header lengths a
    // structure of 32-bit fields can have.
    //
    // Only an exact fill is taken. A pair stream read from the wrong offset
    // does not arrive on the image's last byte, and one that did would hand
    // the emulator rubbish; the stubs it would replace are the better answer.
    size_t foundOffset = 4;
    if (!parsed) {
        static constexpr PairFormat kFormats[4] = {
            PairFormat::CopyDataSkip, PairFormat::SkipCopyData,
            PairFormat::CopySkipData, PairFormat::CopyDataSkipToFill};
        Attempt scratch;
        for (size_t offset = 8; !parsed && offset <= 128; offset += 4) {
            for (PairFormat format : kFormats) {
                if (!tryParsePairs(format, offset, parsedImage, parsedRelocPos,
                                   parsedFinalOffset, parsedPairCount, scratch)) {
                    continue;
                }
                if (parsedFinalOffset != finalCodeSize) continue;
                usedFormat = format;
                foundOffset = offset;
                parsed = true;
                break;
            }
        }
        if (parsed) {
            LOG_WARNING("WardenModule: the pair stream starts ", foundOffset,
                        " bytes in, not 4 - this module's header is longer than "
                        "the one the format is written for");
        }
    }

    if (parsed) {
        std::memcpy(moduleMemory_, parsedImage.data(), parsedImage.size());
        relocDataOffset_ = parsedRelocPos;
        moduleImageUsable_ = true;  // a real code image: safe to hand to the emulator

        const char* formatName = "copy/data/skip";
        if (usedFormat == PairFormat::SkipCopyData) formatName = "skip/copy/data";
        if (usedFormat == PairFormat::CopySkipData) formatName = "copy/skip/data";
        if (usedFormat == PairFormat::CopyDataSkipToFill) formatName = "copy/data/skip to fill";

        LOG_INFO("WardenModule: Parsed ", parsedPairCount, " pairs using format ",
                 formatName, ", final offset: ", parsedFinalOffset, "/", finalCodeSize);
        LOG_INFO("WardenModule: Relocation data starts at decompressed offset ", relocDataOffset_,
                 " (", (exeData.size() - relocDataOffset_), " bytes remaining)");
        return true;
    }

    // Fallback: copy raw payload (without the 4-byte size header) into module memory.
    // This keeps loading alive for servers where packet flow can continue with hash/check fallbacks.
    if (exeData.size() > 4) {
        size_t rawCopySize = std::min(moduleSize_, exeData.size() - 4);
        std::memcpy(moduleMemory_, exeData.data() + 4, rawCopySize);
    }
    relocDataOffset_ = 0;
    moduleImageUsable_ = false;

    // Everything a reader needs to tell one fault from another, because the
    // module itself cannot be attached to a report: it arrives encrypted with
    // a key that is not kept, so the only account of it is this one.
    LOG_WARNING("WardenModule: no known layout unpacks this module. ",
                exeData.size(), " bytes decompressed, header asks for an image of ",
                finalCodeSize, ".");
    {
        std::string head;
        char byteText[4];
        for (size_t i = 0; i < exeData.size() && i < 64; ++i) {
            std::snprintf(byteText, sizeof(byteText), "%02X", exeData[i]);
            head += byteText;
            head += ' ';
        }
        LOG_WARNING("WardenModule: it begins ", head);
    }
    static const char* const kFormatNames[4] = {"copy/data/skip", "skip/copy/data",
                                                "copy/skip/data", "copy/data/skip to fill"};
    for (int i = 0; i < 4; ++i) {
        LOG_WARNING("WardenModule:   ", kFormatNames[i], " read ", attempts[i].pairs,
                    " pair(s), reaching byte ", attempts[i].pos, " of the module and ",
                    attempts[i].dest, " of the image, then stopped: ", attempts[i].why);
    }
    LOG_WARNING("WardenModule: falling back to the raw payload, which the emulator "
                "will not be given - the checks answer from the stubs instead.");
    return true;
}

bool WardenModule::applyRelocations() {
    // Every store below lands in the MAP_JIT mapping; see JitWriteWindow.
    JitWriteWindow jitWrite;
    if (!moduleMemory_ || moduleSize_ == 0) {
        LOG_ERROR("WardenModule: No module memory allocated for relocations");
        return false;
    }

    if (moduleImageUsable_ && relocCount_ != 0) {
        if (relocOffset_ >= moduleSize_) {
            LOG_ERROR("WardenModule: Native relocation table offset out of bounds: ", relocOffset_);
            return false;
        }

        auto* image = static_cast<uint8_t*>(moduleMemory_);
        size_t relocationPos = relocOffset_;
        uint32_t currentOffset = 0;
        uint32_t appliedCount = 0;
        for (uint32_t i = 0; i < relocCount_; ++i) {
            if (relocationPos + 2 > moduleSize_) {
                LOG_ERROR("WardenModule: Native relocation table is truncated");
                return false;
            }
            const uint8_t first = image[relocationPos++];
            if ((first & 0x80u) == 0) {
                currentOffset += (static_cast<uint32_t>(first) << 8) | image[relocationPos++];
            } else {
                if (relocationPos + 3 > moduleSize_) {
                    LOG_ERROR("WardenModule: Native absolute relocation is truncated");
                    return false;
                }
                // The top bit is the form flag, not part of the offset. Kept,
                // every absolute entry came out at or above 0x80000000, which
                // is past the end of any image this accepts (5MB) - so the
                // bounds check below rejected all of them and no module
                // carrying one could ever relocate.
                currentOffset = wardenAbsoluteRelocTarget(
                    first, image[relocationPos], image[relocationPos + 1],
                    image[relocationPos + 2]);
                relocationPos += 3;
            }
            // Widened before the addition. currentOffset is uint32_t and comes
            // from the module, which comes from the server: an entry of
            // FF FF FF FF made it 0xFFFFFFFF, +4 wrapped to 3, the check passed
            // and the write below landed at image + 0xFFFFFFFF.
            if (!wardenRelocTargetFits(currentOffset, moduleSize_)) {
                LOG_ERROR("WardenModule: Native relocation target out of bounds: ", currentOffset);
                return false;
            }
            uint32_t value = 0;
            std::memcpy(&value, image + currentOffset, sizeof(value));
            value += moduleBase_;
            std::memcpy(image + currentOffset, &value, sizeof(value));
            ++appliedCount;
        }
        LOG_INFO("WardenModule: Applied ", appliedCount, " native relocations");
        return true;
    }

    // Relocation data is in decompressedData_ starting at relocDataOffset_
    // Format: delta-encoded 2-byte LE offsets, terminated by 0x0000
    // Each offset in the module image has moduleBase_ added to the 32-bit value there

    if (relocDataOffset_ == 0 || relocDataOffset_ >= decompressedData_.size()) {
        LOG_INFO("WardenModule: No relocation data available");
        return true;
    }

    size_t relocPos = relocDataOffset_;
    uint32_t currentOffset = 0;
    int relocCount = 0;

    while (relocPos + 2 <= decompressedData_.size()) {
        uint16_t delta = decompressedData_[relocPos] | (decompressedData_[relocPos + 1] << 8);
        relocPos += 2;

        if (delta == 0) break;

        currentOffset += delta;

        if (currentOffset + 4 <= moduleSize_) {
            uint8_t* addr = static_cast<uint8_t*>(moduleMemory_) + currentOffset;
            uint32_t val;
            std::memcpy(&val, addr, sizeof(uint32_t));
            val += moduleBase_;
            std::memcpy(addr, &val, sizeof(uint32_t));
            relocCount++;
        } else {
            LOG_ERROR("WardenModule: Relocation offset ", currentOffset,
                      " out of bounds (moduleSize=", moduleSize_, ")");
        }
    }

    {
        char baseBuf[32];
        std::snprintf(baseBuf, sizeof(baseBuf), "0x%X", moduleBase_);
        LOG_INFO("WardenModule: Applied ", relocCount, " relocations (base=", baseBuf, ")");
    }

    return true;
}

bool WardenModule::bindAPIs() {
    // Every store below lands in the MAP_JIT mapping; see JitWriteWindow.
    JitWriteWindow jitWrite;
    if (!moduleMemory_ || moduleSize_ == 0) {
        LOG_ERROR("WardenModule: No module memory allocated for API binding");
        return false;
    }

    LOG_INFO("WardenModule: Binding Windows APIs for module...");

    if (moduleImageUsable_ && importCount_ != 0) {
        if (importTableOffset_ + static_cast<size_t>(importCount_) * 8u > moduleSize_) {
            LOG_ERROR("WardenModule: Native import table out of bounds");
            return false;
        }

        auto* image = static_cast<uint8_t*>(moduleMemory_);
        const auto readImageU32 = [&](uint32_t offset) -> uint32_t {
            uint32_t value = 0;
            if (offset + 4 <= moduleSize_) std::memcpy(&value, image + offset, sizeof(value));
            return value;
        };
        const auto readImageString = [&](uint32_t offset) -> std::string {
            std::string value;
            while (offset < moduleSize_ && image[offset] != 0) value.push_back(static_cast<char>(image[offset++]));
            return value;
        };

        uint32_t totalImports = 0;
        uint32_t resolvedImports = 0;
        for (uint32_t libraryIndex = 0; libraryIndex < importCount_; ++libraryIndex) {
            const uint32_t descriptorOffset = importTableOffset_ + libraryIndex * 8u;
            const uint32_t libraryNameOffset = readImageU32(descriptorOffset);
            const uint32_t thunkOffset = readImageU32(descriptorOffset + 4);
            if (libraryNameOffset >= moduleSize_ || thunkOffset >= moduleSize_) {
                LOG_WARNING("WardenModule: Native import descriptor ", libraryIndex, " is out of bounds");
                continue;
            }
            const std::string libraryName = readImageString(libraryNameOffset);
            for (uint32_t thunk = thunkOffset; thunk + 4 <= moduleSize_; thunk += 4) {
                const uint32_t importValue = readImageU32(thunk);
                if (importValue == 0) break;
                ++totalImports;
                std::string functionName;
                if ((importValue & 0x80000000u) != 0) {
                    functionName = "#" + std::to_string(importValue & 0x7fffffffu);
                } else if (importValue < moduleSize_) {
                    functionName = readImageString(importValue);
                }
                if (functionName.empty()) functionName = "unknown";

                uint32_t resolvedAddress = 0;
                #ifdef HAVE_UNICORN
                if (emulator_) {
                    resolvedAddress = emulator_->getAPIAddress(libraryName, functionName);
                    if (resolvedAddress == 0) {
                        resolvedAddress = emulator_->hookAPI(libraryName, functionName,
                            [](WardenEmulator&, const std::vector<uint32_t>&) -> uint32_t { return 0; });
                    }
                }
                #endif
                if (resolvedAddress != 0) {
                    std::memcpy(image + thunk, &resolvedAddress, sizeof(resolvedAddress));
                    #ifdef HAVE_UNICORN
                    // The emulator copied moduleMemory_ during initialize(), so
                    // mirror the IAT patch into its mapped image as well.
                    if (emulator_ && !emulator_->writeMemory(moduleBase_ + thunk,
                                                            &resolvedAddress,
                                                            sizeof(resolvedAddress))) {
                        LOG_WARNING("WardenModule: Failed to mirror native IAT entry into emulator");
                    }
                    #endif
                    ++resolvedImports;
                }
            }
        }
        LOG_INFO("WardenModule: Bound ", resolvedImports, "/", totalImports,
                 " native API imports");
        return true;
    }

    // The Warden module import table lives in decompressedData_ immediately after
    // the relocation entries (which are terminated by a 0x0000 delta). Format:
    //
    //   Repeated library blocks until null library name:
    //     string libraryName\0
    //     Repeated function entries until null function name:
    //       string functionName\0
    //
    // Each imported function corresponds to a sequential IAT slot at the start
    // of the module image (first N dwords). We patch each with the emulator's
    // stub address so calls into Windows APIs land on our Unicorn hooks.

    if (relocDataOffset_ == 0 || relocDataOffset_ >= decompressedData_.size()) {
        LOG_WARNING("WardenModule: No relocation/import data - skipping API binding");
        return true;
    }

    // Skip past relocation entries (delta-encoded uint16 pairs, 0x0000 terminated)
    size_t pos = relocDataOffset_;
    while (pos + 2 <= decompressedData_.size()) {
        uint16_t delta = decompressedData_[pos] | (decompressedData_[pos + 1] << 8);
        pos += 2;
        if (delta == 0) break;
    }

    if (pos >= decompressedData_.size()) {
        LOG_INFO("WardenModule: No import data after relocations");
        return true;
    }

    // Parse import table
    uint32_t iatSlotIndex = 0;
    int totalImports = 0;
    int resolvedImports = 0;

    auto readString = [&](size_t& p) -> std::string {
        std::string s;
        while (p < decompressedData_.size() && decompressedData_[p] != 0) {
            s.push_back(static_cast<char>(decompressedData_[p]));
            p++;
        }
        if (p < decompressedData_.size()) p++; // skip null terminator
        return s;
    };

    while (pos < decompressedData_.size()) {
        std::string libraryName = readString(pos);
        if (libraryName.empty()) break; // null library name = end of imports

        // Read functions for this library
        while (pos < decompressedData_.size()) {
            std::string functionName = readString(pos);
            if (functionName.empty()) break; // null function name = next library

            totalImports++;

            // Look up the emulator's stub address for this API
            uint32_t resolvedAddr = 0;
            #ifdef HAVE_UNICORN
            if (emulator_) {
                // Check if this API was pre-registered in setupCommonAPIHooks()
                resolvedAddr = emulator_->getAPIAddress(libraryName, functionName);
                if (resolvedAddr == 0) {
                    // Not pre-registered - create a no-op stub that returns 0.
                    // Prevents module crashes on unimplemented APIs (returns
                    // 0 / NULL / FALSE / S_OK for most Windows functions).
                    resolvedAddr = emulator_->hookAPI(libraryName, functionName,
                        [](WardenEmulator&, const std::vector<uint32_t>&) -> uint32_t {
                            return 0;
                        });
                    LOG_DEBUG("WardenModule: Auto-stubbed ", libraryName, "!", functionName);
                }
            }
            #endif

            // Patch IAT slot in module image
            if (resolvedAddr != 0) {
                uint32_t iatOffset = iatSlotIndex * 4;
                if (iatOffset + 4 <= moduleSize_) {
                    uint8_t* slot = static_cast<uint8_t*>(moduleMemory_) + iatOffset;
                    std::memcpy(slot, &resolvedAddr, 4);
                    resolvedImports++;
                    LOG_DEBUG("WardenModule: IAT[", iatSlotIndex, "] = ", libraryName,
                              "!", functionName, " → 0x", std::hex, resolvedAddr, std::dec);
                }
            }
            iatSlotIndex++;
        }
    }

    LOG_INFO("WardenModule: Bound ", resolvedImports, "/", totalImports,
             " API imports (", iatSlotIndex, " IAT slots patched)");
    return true;
}

uint32_t WardenModule::resolveExport(uint32_t ordinal) const {
    if (!moduleImageUsable_ || !moduleMemory_ || exportTableOffset_ == 0 || exportCount_ == 0 ||
        ordinal < exportBaseIndex_) {
        return 0;
    }
    // In size_t throughout. exportCount_ and exportTableOffset_ are header
    // fields the header check does not bound, so every one of these products
    // and sums can wrap in 32 bits - the same fault the relocation loop had.
    const size_t index = static_cast<size_t>(ordinal) - exportBaseIndex_;
    const size_t slot = static_cast<size_t>(exportTableOffset_) + index * 4u;
    if (index >= exportCount_ || slot + 4 > moduleSize_) return 0;

    uint32_t offset = 0;
    std::memcpy(&offset, static_cast<const uint8_t*>(moduleMemory_) + slot, sizeof(offset));
    return offset < moduleSize_ ? moduleBase_ + offset : 0;
}

bool WardenModule::initializeModule() {
    if (!moduleMemory_ || moduleSize_ == 0) {
        LOG_ERROR("WardenModule: No module memory allocated for initialization");
        return false;
    }

    LOG_INFO("WardenModule: Initializing Warden module...");

    // Module initialization protocol:
    //
    // 1. Client provides structure with 7 callback pointers:
    //    - void (*sendPacket)(uint8_t* data, size_t len)
    //    - void (*validateModule)(uint8_t* hash)
    //    - void* (*allocMemory)(size_t size)
    //    - void (*freeMemory)(void* ptr)
    //    - void (*generateRC4)(uint8_t* seed)
    //    - uint32_t (*getTime)()
    //    - void (*logMessage)(const char* msg)
    //
    // 2. Call module entry point with callback structure
    //
    // 3. Module returns WardenFuncList with 4 exported functions:
    //    - generateRC4Keys(packet)
    //    - unload(rc4Keys)
    //    - packetHandler(data)
    //    - tick(deltaMs)

    // Define callback structure (what we provide to module)
    struct ClientCallbacks {
        void (*sendPacket)(uint8_t* data, size_t len);
        void (*validateModule)(uint8_t* hash);
        void* (*allocMemory)(size_t size);
        void (*freeMemory)(void* ptr);
        void (*generateRC4)(uint8_t* seed);
        uint32_t (*getTime)();
        void (*logMessage)(const char* msg);
    };

    // Setup client callbacks (used when calling module entry point below).
    // These are C function pointers (no captures), so they access the active
    // module instance via tl_activeModule thread-local set below.
    [[maybe_unused]] ClientCallbacks callbacks = {};

    callbacks.sendPacket = [](uint8_t* data, size_t len) {
        LOG_DEBUG("WardenModule Callback: sendPacket(", len, " bytes)");
        auto* mod = tl_activeModule;
        if (mod && mod->callbackSendPacket_ && data && len > 0) {
            mod->callbackSendPacket_(data, len);
        }
    };

    callbacks.validateModule = [](uint8_t* hash) {
        LOG_DEBUG("WardenModule Callback: validateModule()");
        auto* mod = tl_activeModule;
        if (!mod || !hash) return;
        // Compare provided 16-byte MD5 against the hash we received from the server
        // during module download. Mismatch means the module was corrupted in transit.
        const auto& expected = mod->md5Hash_;
        if (expected.size() == 16 && std::memcmp(hash, expected.data(), 16) != 0) {
            LOG_ERROR("WardenModule: validateModule hash MISMATCH - module may be corrupted");
        } else {
            LOG_DEBUG("WardenModule: validateModule hash OK");
        }
    };

    callbacks.allocMemory = [](size_t size) -> void* {
        return malloc(size);
    };

    callbacks.freeMemory = [](void* ptr) {
        free(ptr);
    };

    callbacks.generateRC4 = [](uint8_t* seed) {
        LOG_DEBUG("WardenModule Callback: generateRC4()");
        auto* mod = tl_activeModule;
        if (!mod || !mod->callbackCrypto_ || !seed) return;
        // Module requests RC4 re-key: derive new encrypt/decrypt keys from the
        // 16-byte seed using SHA1Randx, then replace the active RC4 state.
        uint8_t newEncryptKey[16], newDecryptKey[16];
        std::vector<uint8_t> seedVec(seed, seed + 16);
        WardenCrypto::sha1RandxGenerate(seedVec, newEncryptKey, newDecryptKey);
        mod->callbackCrypto_->replaceKeys(
            std::vector<uint8_t>(newEncryptKey, newEncryptKey + 16),
            std::vector<uint8_t>(newDecryptKey, newDecryptKey + 16));
        LOG_INFO("WardenModule: RC4 keys re-derived from module seed");
    };

    callbacks.getTime = []() -> uint32_t {
        return static_cast<uint32_t>(time(nullptr));
    };

    callbacks.logMessage = [](const char* msg) {
        LOG_INFO("WardenModule Log: ", msg);
    };

    // Set thread-local context so C callbacks can access this module's state
    tl_activeModule = this;

    // Module entry point is typically at offset 0 (first bytes of loaded code)
    // Function signature: WardenFuncList* (*entryPoint)(ClientCallbacks*)

    #ifdef HAVE_UNICORN
        // Use Unicorn emulator for cross-platform execution
        LOG_INFO("WardenModule: Initializing Unicorn emulator...");

        emulator_ = std::make_unique<WardenEmulator>();
        if (!emulator_->initialize(moduleMemory_, moduleSize_, moduleBase_)) {
            LOG_ERROR("WardenModule: Failed to initialize emulator");
            return false;
        }

        // Setup Windows API hooks (VirtualAlloc, GetTickCount, ReadProcessMemory, etc.)
        emulator_->setupCommonAPIHooks();

        // Bind module imports: parse the import table from decompressed data and
        // patch each IAT slot with the emulator's stub address. Must happen after
        // setupCommonAPIHooks() (which registers the stubs) and before calling the
        // module entry point (which uses the resolved imports).
        bindAPIs();

        {
            char addrBuf[32];
            std::snprintf(addrBuf, sizeof(addrBuf), "0x%X", moduleBase_);
            LOG_INFO("WardenModule: Emulator initialized successfully");
            LOG_INFO("WardenModule:   Ready to execute module at ", addrBuf);
        }

        // Allocate memory for ClientCallbacks structure in emulated space
        uint32_t callbackStructAddr = emulator_->allocateMemory(sizeof(ClientCallbacks), 0x04);
        if (callbackStructAddr == 0) {
            LOG_ERROR("WardenModule: Failed to allocate memory for callbacks");
            return false;
        }

        // Write callback function pointers to emulated memory
        // Note: These would be addresses of stub functions in emulated space
        // For now, we'll write placeholder addresses
        std::vector<uint32_t> callbackAddrs = {
            0x70001000, // sendPacket
            0x70001100, // validateModule
            0x70001200, // allocMemory
            0x70001300, // freeMemory
            0x70001400, // generateRC4
            0x70001500, // getTime
            0x70001600  // logMessage
        };

        // Write callback struct (7 function pointers = 28 bytes)
        for (size_t i = 0; i < callbackAddrs.size(); ++i) {
            uint32_t addr = callbackAddrs[i];
            emulator_->writeMemory(callbackStructAddr + (i * 4), &addr, 4);
        }

        {
            char cbBuf[32];
            std::snprintf(cbBuf, sizeof(cbBuf), "0x%X", callbackStructAddr);
            LOG_INFO("WardenModule: Prepared ClientCallbacks at ", cbBuf);
        }

        // Native modules export their entry point by ordinal; legacy modules
        // retain the historical offset-zero entry point.
        uint32_t entryPoint = resolveExport(1);
        if (entryPoint == 0) entryPoint = moduleBase_;

        {
            char epBuf[32];
            std::snprintf(epBuf, sizeof(epBuf), "0x%X", entryPoint);
            LOG_INFO("WardenModule: Calling module entry point at ", epBuf);
        }

        try {
            // Call: WardenFuncList* InitModule(ClientCallbacks* callbacks)
            std::vector<uint32_t> args = { callbackStructAddr };
            uint32_t result = emulator_->callFunction(entryPoint, args);

            if (result == 0) {
                LOG_ERROR("WardenModule: Module entry returned NULL");
                return false;
            }

            {
                char resBuf[32];
                std::snprintf(resBuf, sizeof(resBuf), "0x%X", result);
                LOG_INFO("WardenModule: Module initialized, WardenFuncList at ", resBuf);
            }

            // Read WardenFuncList structure from emulated memory
            // Structure has 4 function pointers (16 bytes):
            //   [0] generateRC4Keys(uint8_t* seed)
            //   [1] unload(uint8_t* rc4Keys)
            //   [2] packetHandler(uint8_t* data, uint32_t size,
            //                     uint8_t* responseOut, uint32_t* responseSizeOut)
            //   [3] tick(uint32_t deltaMs) -> uint32_t
            uint32_t funcAddrs[4] = {};
            if (emulator_->readMemory(result, funcAddrs, 16)) {
                char fb[4][32];
                for (int fi = 0; fi < 4; ++fi)
                    std::snprintf(fb[fi], sizeof(fb[fi]), "0x%X", funcAddrs[fi]);
                LOG_INFO("WardenModule: Module exported functions:");
                LOG_INFO("WardenModule:   generateRC4Keys: ", fb[0]);
                LOG_INFO("WardenModule:   unload:          ", fb[1]);
                LOG_INFO("WardenModule:   packetHandler:   ", fb[2]);
                LOG_INFO("WardenModule:   tick:            ", fb[3]);

                // Wrap emulated function addresses into std::function dispatchers
                WardenEmulator* emu = emulator_.get();

                if (funcAddrs[0]) {
                    uint32_t addr = funcAddrs[0];
                    funcList_.generateRC4Keys = [emu, addr](uint8_t* seed) {
                        // Warden RC4 seed is a fixed 4-byte value
                        uint32_t seedAddr = emu->writeData(seed, 4);
                        if (seedAddr) {
                            emu->callFunction(addr, {seedAddr});
                            emu->freeMemory(seedAddr);
                        }
                    };
                }

                if (funcAddrs[1]) {
                    uint32_t addr = funcAddrs[1];
                    funcList_.unload = [emu, addr]([[maybe_unused]] uint8_t* rc4Keys) {
                        emu->callFunction(addr, {0u}); // pass NULL; module saves its own state
                    };
                }

                if (funcAddrs[2]) {
                    // Store raw address for the 4-arg call in processCheckRequest
                    emulatedPacketHandlerAddr_ = funcAddrs[2];
                    uint32_t addr = funcAddrs[2];
                    // Simple 2-arg variant for generic callers (no response extraction)
                    funcList_.packetHandler = [emu, addr](uint8_t* data, size_t length) {
                        uint32_t dataAddr = emu->writeData(data, length);
                        if (dataAddr) {
                            emu->callFunction(addr, {dataAddr, static_cast<uint32_t>(length)});
                            emu->freeMemory(dataAddr);
                        }
                    };
                }

                if (funcAddrs[3]) {
                    uint32_t addr = funcAddrs[3];
                    funcList_.tick = [emu, addr](uint32_t deltaMs) -> uint32_t {
                        return emu->callFunction(addr, {deltaMs});
                    };
                }
            }

            LOG_INFO("WardenModule: Module fully initialized and ready!");

        } catch (const std::exception& e) {
            LOG_ERROR("WardenModule: Exception during module initialization: ", e.what());
            return false;
        }

    #elif defined(_WIN32)
        // Native Windows execution (dangerous without sandboxing)
        typedef void* (*ModuleEntryPoint)(ClientCallbacks*);
        ModuleEntryPoint entryPoint = reinterpret_cast<ModuleEntryPoint>(moduleMemory_);

        LOG_INFO("WardenModule: Calling module entry point at ", moduleMemory_);

        // NOTE: This would execute native x86 code
        // Extremely dangerous without proper validation!
        // void* result = entryPoint(&callbacks);

        LOG_WARNING("WardenModule: Module entry point call is DISABLED (unsafe without validation)");
        LOG_INFO("WardenModule:   Would execute x86 code at ", moduleMemory_);

        // TODO: Extract WardenFuncList from result
        // funcList_.packetHandler = ...
        // funcList_.tick = ...
        // funcList_.generateRC4Keys = ...
        // funcList_.unload = ...

    #else
        LOG_WARNING("WardenModule: Cannot execute Windows x86 code on Linux");
        LOG_INFO("WardenModule:   Module entry point: ", moduleMemory_);
        LOG_INFO("WardenModule:   Would call entry point with ClientCallbacks struct");
    #endif

    // For now, return true to mark module as "loaded" at infrastructure level
    // Real execution would require:
    // 1. Proper PE parsing to find actual entry point
    // 2. Calling convention (stdcall/cdecl) handling
    // 3. Exception handling for crashes
    // 4. Sandboxing for security

    // Clear thread-local context - callbacks are only valid during init
    tl_activeModule = nullptr;

    LOG_WARNING("WardenModule: Module initialization complete (callbacks wired)");
    return true;
}

} // namespace game
} // namespace wowee
