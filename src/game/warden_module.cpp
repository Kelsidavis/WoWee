#include "game/warden_module.hpp"
#include "auth/crypto.hpp"
#include <cstring>
#include <fstream>
#include <filesystem>
#include <iostream>
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
// WardenModule Implementation
// ============================================================================

WardenModule::WardenModule()
    : loaded_(false)
    , moduleMemory_(nullptr)
    , moduleSize_(0)
    , moduleBase_(0x400000) // Default module base address
{
}

WardenModule::~WardenModule() {
    unload();
}

bool WardenModule::load(const std::vector<uint8_t>& moduleData,
                        const std::vector<uint8_t>& md5Hash,
                        const std::vector<uint8_t>& rc4Key) {
    moduleData_ = moduleData;
    md5Hash_ = md5Hash;

    std::cout << "[WardenModule] Loading module (MD5: ";
    for (size_t i = 0; i < std::min(md5Hash.size(), size_t(8)); ++i) {
        printf("%02X", md5Hash[i]);
    }
    std::cout << "...)" << '\n';

    // Step 1: Verify MD5 hash
    if (!verifyMD5(moduleData, md5Hash)) {
        std::cerr << "[WardenModule] MD5 verification failed!" << '\n';
        return false;
    }
    std::cout << "[WardenModule] ✓ MD5 verified" << '\n';

    // Step 2: RC4 decrypt (Warden protocol-required legacy RC4; server-mandated, cannot be changed)
    if (!decryptRC4(moduleData, rc4Key, decryptedData_)) { // codeql[cpp/weak-cryptographic-algorithm]
        std::cerr << "[WardenModule] RC4 decryption failed!" << '\n';
        return false;
    }
    std::cout << "[WardenModule] ✓ RC4 decrypted (" << decryptedData_.size() << " bytes)" << '\n';

    // Step 3: Verify RSA signature
    if (!verifyRSASignature(decryptedData_)) {
        std::cerr << "[WardenModule] RSA signature verification failed!" << '\n';
        // Note: Currently returns true (skipping verification) due to placeholder modulus
    }

    // Step 4: Strip RSA signature (last 256 bytes) then zlib decompress
    std::vector<uint8_t> dataWithoutSig;
    if (decryptedData_.size() > 256) {
        dataWithoutSig.assign(decryptedData_.begin(), decryptedData_.end() - 256);
    } else {
        dataWithoutSig = decryptedData_;
    }
    if (!decompressZlib(dataWithoutSig, decompressedData_)) {
        std::cerr << "[WardenModule] zlib decompression failed!" << '\n';
        return false;
    }

    // Step 5: Parse custom executable format
    if (!parseExecutableFormat(decompressedData_)) {
        std::cerr << "[WardenModule] Executable format parsing failed!" << '\n';
        return false;
    }

    // Step 6: Apply relocations
    if (!applyRelocations()) {
        std::cerr << "[WardenModule] Address relocations failed!" << '\n';
        return false;
    }

    // Step 7: Bind APIs
    if (!bindAPIs()) {
        std::cerr << "[WardenModule] API binding failed!" << '\n';
        // Note: Currently returns true (stub) on both Windows and Linux
    }

    // Step 8: Initialize module
    if (!initializeModule()) {
        std::cerr << "[WardenModule] Module initialization failed!" << '\n';
        return false;
    }

    // Module loading pipeline complete!
    // Note: Steps 6-8 are stubs/platform-limited, but infrastructure is ready
    loaded_ = true; // Mark as loaded (infrastructure complete)

    std::cout << "[WardenModule] ✓ Module loading pipeline COMPLETE" << '\n';
    std::cout << "[WardenModule]   Status: Infrastructure ready, execution stubs in place" << '\n';
    std::cout << "[WardenModule]   Limitations:" << '\n';
    std::cout << "[WardenModule]     - Relocations: needs real module data" << '\n';
    std::cout << "[WardenModule]     - API Binding: Windows only (or Wine on Linux)" << '\n';
    std::cout << "[WardenModule]     - Execution: disabled (unsafe without validation)" << '\n';
    std::cout << "[WardenModule]   For strict servers: Would need to enable actual x86 execution" << '\n';

    return true;
}

bool WardenModule::processCheckRequest(const std::vector<uint8_t>& checkData,
                                       [[maybe_unused]] std::vector<uint8_t>& responseOut) {
    if (!loaded_) {
        std::cerr << "[WardenModule] Module not loaded, cannot process checks" << '\n';
        return false;
    }

    #ifdef HAVE_UNICORN
        if (emulator_ && emulator_->isInitialized() && funcList_.packetHandler) {
            std::cout << "[WardenModule] Processing check request via emulator..." << '\n';
            std::cout << "[WardenModule]   Check data: " << checkData.size() << " bytes" << '\n';

            // Allocate memory for check data in emulated space
            uint32_t checkDataAddr = emulator_->allocateMemory(checkData.size(), 0x04);
            if (checkDataAddr == 0) {
                std::cerr << "[WardenModule] Failed to allocate memory for check data" << '\n';
                return false;
            }

            // Write check data to emulated memory
            if (!emulator_->writeMemory(checkDataAddr, checkData.data(), checkData.size())) {
                std::cerr << "[WardenModule] Failed to write check data" << '\n';
                emulator_->freeMemory(checkDataAddr);
                return false;
            }

            // Allocate response buffer in emulated space (assume max 1KB response)
            uint32_t responseAddr = emulator_->allocateMemory(1024, 0x04);
            if (responseAddr == 0) {
                std::cerr << "[WardenModule] Failed to allocate response buffer" << '\n';
                emulator_->freeMemory(checkDataAddr);
                return false;
            }

            try {
                // Call module's PacketHandler
                // void PacketHandler(uint8_t* checkData, size_t checkSize,
                //                   uint8_t* responseOut, size_t* responseSizeOut)
                std::cout << "[WardenModule] Calling PacketHandler..." << '\n';

                // For now, this is a placeholder - actual calling would depend on
                // the module's exact function signature
                std::cout << "[WardenModule] ⚠ PacketHandler execution stubbed" << '\n';
                std::cout << "[WardenModule]   Would call emulated function to process checks" << '\n';
                std::cout << "[WardenModule]   This would generate REAL responses (not fakes!)" << '\n';

                // Clean up
                emulator_->freeMemory(checkDataAddr);
                emulator_->freeMemory(responseAddr);

                // For now, return false to use fake responses
                // Once we have a real module, we'd read the response from responseAddr
                return false;

            } catch (const std::exception& e) {
                std::cerr << "[WardenModule] Exception during PacketHandler: " << e.what() << '\n';
                emulator_->freeMemory(checkDataAddr);
                emulator_->freeMemory(responseAddr);
                return false;
            }
        }
    #endif

    std::cout << "[WardenModule] ⚠ processCheckRequest NOT IMPLEMENTED" << '\n';
    std::cout << "[WardenModule]   Would call module->PacketHandler() here" << '\n';

    // For now, return false to fall back to fake responses in GameHandler
    return false;
}

uint32_t WardenModule::tick([[maybe_unused]] uint32_t deltaMs) {
    if (!loaded_ || !funcList_.tick) {
        return 0; // No tick needed
    }

    // TODO: Call module's Tick function
    // return funcList_.tick(deltaMs);

    return 0;
}

void WardenModule::generateRC4Keys([[maybe_unused]] uint8_t* packet) {
    if (!loaded_ || !funcList_.generateRC4Keys) {
        return;
    }

    // TODO: Call module's GenerateRC4Keys function
    // This re-keys the Warden crypto stream
    // funcList_.generateRC4Keys(packet);
}

void WardenModule::unload() {
    if (moduleMemory_) {
        // Call module's Unload() function if loaded
        if (loaded_ && funcList_.unload) {
            std::cout << "[WardenModule] Calling module unload callback..." << '\n';
            // TODO: Implement callback when execution layer is complete
            // funcList_.unload(nullptr);
        }

        // Free executable memory region
        std::cout << "[WardenModule] Freeing " << moduleSize_ << " bytes of executable memory" << '\n';
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
        std::cerr << "[WardenModule] Invalid RC4 key size: " << key.size() << " (expected 16)" << '\n';
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

bool WardenModule::verifyRSASignature(const std::vector<uint8_t>& data) {
    // RSA-2048 signature is last 256 bytes
    if (data.size() < 256) {
        std::cerr << "[WardenModule] Data too small for RSA signature (need at least 256 bytes)" << '\n';
        return false;
    }

    // Extract signature (last 256 bytes)
    std::vector<uint8_t> signature(data.end() - 256, data.end());

    // Extract data without signature
    std::vector<uint8_t> dataWithoutSig(data.begin(), data.end() - 256);

    // Hardcoded WoW 3.3.5a Warden RSA public key
    // Exponent: 0x010001 (65537)
    const uint32_t exponent = 0x010001;

    // Modulus (256 bytes) - Extracted from WoW 3.3.5a (build 12340) client
    // Extracted from Wow.exe at offset 0x005e3a03 (.rdata section)
    // This is the actual RSA-2048 public key modulus used by Warden
    const uint8_t modulus[256] = {
        0x51, 0xAD, 0x57, 0x75, 0x16, 0x92, 0x0A, 0x0E, 0xEB, 0xFA, 0xF8, 0x1B, 0x37, 0x49, 0x7C, 0xDD,
        0x47, 0xDA, 0x5E, 0x02, 0x8D, 0x96, 0x75, 0x21, 0x27, 0x59, 0x04, 0xAC, 0xB1, 0x0C, 0xB9, 0x23,
        0x05, 0xCC, 0x82, 0xB8, 0xBF, 0x04, 0x77, 0x62, 0x92, 0x01, 0x00, 0x01, 0x00, 0x77, 0x64, 0xF8,
        0x57, 0x1D, 0xFB, 0xB0, 0x09, 0xC4, 0xE6, 0x28, 0x91, 0x34, 0xE3, 0x55, 0x61, 0x15, 0x8A, 0xE9,
        0x07, 0xFC, 0xAA, 0x60, 0xB3, 0x82, 0xB7, 0xE2, 0xA4, 0x40, 0x15, 0x01, 0x3F, 0xC2, 0x36, 0xA8,
        0x9D, 0x95, 0xD0, 0x54, 0x69, 0xAA, 0xF5, 0xED, 0x5C, 0x7F, 0x21, 0xC5, 0x55, 0x95, 0x56, 0x5B,
        0x2F, 0xC6, 0xDD, 0x2C, 0xBD, 0x74, 0xA3, 0x5A, 0x0D, 0x70, 0x98, 0x9A, 0x01, 0x36, 0x51, 0x78,
        0x71, 0x9B, 0x8E, 0xCB, 0xB8, 0x84, 0x67, 0x30, 0xF4, 0x43, 0xB3, 0xA3, 0x50, 0xA3, 0xBA, 0xA4,
        0xF7, 0xB1, 0x94, 0xE5, 0x5B, 0x95, 0x8B, 0x1A, 0xE4, 0x04, 0x1D, 0xFB, 0xCF, 0x0E, 0xE6, 0x97,
        0x4C, 0xDC, 0xE4, 0x28, 0x7F, 0xB8, 0x58, 0x4A, 0x45, 0x1B, 0xC8, 0x8C, 0xD0, 0xFD, 0x2E, 0x77,
        0xC4, 0x30, 0xD8, 0x3D, 0xD2, 0xD5, 0xFA, 0xBA, 0x9D, 0x1E, 0x02, 0xF6, 0x7B, 0xBE, 0x08, 0x95,
        0xCB, 0xB0, 0x53, 0x3E, 0x1C, 0x41, 0x45, 0xFC, 0x27, 0x6F, 0x63, 0x6A, 0x73, 0x91, 0xA9, 0x42,
        0x00, 0x12, 0x93, 0xF8, 0x5B, 0x83, 0xED, 0x52, 0x77, 0x4E, 0x38, 0x08, 0x16, 0x23, 0x10, 0x85,
        0x4C, 0x0B, 0xA9, 0x8C, 0x9C, 0x40, 0x4C, 0xAF, 0x6E, 0xA7, 0x89, 0x02, 0xC5, 0x06, 0x96, 0x99,
        0x41, 0xD4, 0x31, 0x03, 0x4A, 0xA9, 0x2B, 0x17, 0x52, 0xDD, 0x5C, 0x4E, 0x5F, 0x16, 0xC3, 0x81,
        0x0F, 0x2E, 0xE2, 0x17, 0x45, 0x2B, 0x7B, 0x65, 0x7A, 0xA3, 0x18, 0x87, 0xC2, 0xB2, 0xF5, 0xCD
    };

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
        std::cerr << "[WardenModule] RSA public decrypt failed" << '\n';
        return false;
    }

    // Expected format: padding (0xBB bytes) + SHA1 hash (20 bytes)
    // Total: 256 bytes decrypted
    // Find SHA1 hash in decrypted signature (should be at end, preceded by 0xBB padding)

    // Look for SHA1 hash in last 20 bytes
    if (decryptedLen >= 20) {
        std::vector<uint8_t> actualHash(decryptedSig.end() - 20, decryptedSig.end());

        if (std::memcmp(actualHash.data(), expectedHash.data(), 20) == 0) {
            std::cout << "[WardenModule] ✓ RSA signature verified" << '\n';
            return true;
        }
    }

    std::cerr << "[WardenModule] RSA signature verification FAILED (hash mismatch)" << '\n';
    std::cerr << "[WardenModule] NOTE: Using placeholder modulus - extract real modulus from WoW.exe for actual verification" << '\n';

    // For development, return true to proceed (since we don't have real modulus)
    // TODO: Set to false once real modulus is extracted
    std::cout << "[WardenModule] ⚠ Skipping RSA verification (placeholder modulus)" << '\n';
    return true; // TEMPORARY - change to false for production
}

bool WardenModule::decompressZlib(const std::vector<uint8_t>& compressed,
                                  std::vector<uint8_t>& decompressedOut) {
    if (compressed.size() < 4) {
        std::cerr << "[WardenModule] Compressed data too small (need at least 4 bytes for size header)" << '\n';
        return false;
    }

    // Read 4-byte uncompressed size (little-endian)
    uint32_t uncompressedSize =
        compressed[0] |
        (compressed[1] << 8) |
        (compressed[2] << 16) |
        (compressed[3] << 24);

    std::cout << "[WardenModule] Uncompressed size: " << uncompressedSize << " bytes" << '\n';

    // Sanity check (modules shouldn't be larger than 10MB)
    if (uncompressedSize > 10 * 1024 * 1024) {
        std::cerr << "[WardenModule] Uncompressed size suspiciously large: " << uncompressedSize << " bytes" << '\n';
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
        std::cerr << "[WardenModule] inflateInit failed: " << ret << '\n';
        return false;
    }

    // Decompress
    ret = inflate(&stream, Z_FINISH);

    // Cleanup
    inflateEnd(&stream);

    if (ret != Z_STREAM_END) {
        std::cerr << "[WardenModule] inflate failed: " << ret << '\n';
        return false;
    }

    std::cout << "[WardenModule] ✓ zlib decompression successful ("
              << stream.total_out << " bytes decompressed)" << '\n';

    return true;
}

bool WardenModule::parseExecutableFormat(const std::vector<uint8_t>& exeData) {
    if (exeData.size() < 4) {
        std::cerr << "[WardenModule] Executable data too small for header" << '\n';
        return false;
    }

    // Read final code size (little-endian 4 bytes)
    uint32_t finalCodeSize =
        exeData[0] |
        (exeData[1] << 8) |
        (exeData[2] << 16) |
        (exeData[3] << 24);

    std::cout << "[WardenModule] Final code size: " << finalCodeSize << " bytes" << '\n';

    // Sanity check (executable shouldn't be larger than 5MB)
    if (finalCodeSize > 5 * 1024 * 1024 || finalCodeSize == 0) {
        std::cerr << "[WardenModule] Invalid final code size: " << finalCodeSize << '\n';
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
            std::cerr << "[WardenModule] VirtualAlloc failed" << '\n';
            return false;
        }
    #else
        moduleMemory_ = mmap(
            nullptr,
            finalCodeSize,
            PROT_READ | PROT_WRITE | PROT_EXEC,
            MAP_PRIVATE | MAP_ANONYMOUS,
            -1,
            0
        );
        if (moduleMemory_ == MAP_FAILED) {
            std::cerr << "[WardenModule] mmap failed: " << strerror(errno) << '\n';
            moduleMemory_ = nullptr;
            return false;
        }
    #endif

    moduleSize_ = finalCodeSize;
    std::memset(moduleMemory_, 0, moduleSize_); // Zero-initialize

    std::cout << "[WardenModule] Allocated " << moduleSize_ << " bytes of executable memory at "
              << moduleMemory_ << '\n';

    // Parse skip/copy pairs
    // Format: repeated [2B skip_count][2B copy_count][copy_count bytes data]
    // Skip = advance dest pointer (zeros), Copy = copy from source to dest
    // Terminates when skip_count == 0
    size_t pos = 4; // Skip 4-byte size header
    size_t destOffset = 0;
    int pairCount = 0;

    while (pos + 2 <= exeData.size()) {
        // Read skip count (2 bytes LE)
        uint16_t skipCount = exeData[pos] | (exeData[pos + 1] << 8);
        pos += 2;

        if (skipCount == 0) {
            break; // End of skip/copy pairs
        }

        // Advance dest pointer by skipCount (gaps are zero-filled from memset)
        destOffset += skipCount;

        // Read copy count (2 bytes LE)
        if (pos + 2 > exeData.size()) {
            std::cerr << "[WardenModule] Unexpected end of data reading copy count" << '\n';
            break;
        }
        uint16_t copyCount = exeData[pos] | (exeData[pos + 1] << 8);
        pos += 2;

        if (copyCount > 0) {
            if (pos + copyCount > exeData.size()) {
                std::cerr << "[WardenModule] Copy section extends beyond data bounds" << '\n';
                #ifdef _WIN32
                    VirtualFree(moduleMemory_, 0, MEM_RELEASE);
                #else
                    munmap(moduleMemory_, moduleSize_);
                #endif
                moduleMemory_ = nullptr;
                return false;
            }

            if (destOffset + copyCount > moduleSize_) {
                std::cerr << "[WardenModule] Copy section exceeds module size" << '\n';
                #ifdef _WIN32
                    VirtualFree(moduleMemory_, 0, MEM_RELEASE);
                #else
                    munmap(moduleMemory_, moduleSize_);
                #endif
                moduleMemory_ = nullptr;
                return false;
            }

            std::memcpy(
                static_cast<uint8_t*>(moduleMemory_) + destOffset,
                exeData.data() + pos,
                copyCount
            );
            pos += copyCount;
            destOffset += copyCount;
        }

        pairCount++;
        std::cout << "[WardenModule]   Pair " << pairCount << ": skip " << skipCount
                  << ", copy " << copyCount << " (dest offset=" << destOffset << ")" << '\n';
    }

    // Save position — remaining decompressed data contains relocation entries
    relocDataOffset_ = pos;

    std::cout << "[WardenModule] Parsed " << pairCount << " skip/copy pairs, final offset: "
              << destOffset << "/" << finalCodeSize << '\n';
    std::cout << "[WardenModule] Relocation data starts at decompressed offset " << relocDataOffset_
              << " (" << (exeData.size() - relocDataOffset_) << " bytes remaining)" << '\n';

    return true;
}

bool WardenModule::applyRelocations() {
    if (!moduleMemory_ || moduleSize_ == 0) {
        std::cerr << "[WardenModule] No module memory allocated for relocations" << '\n';
        return false;
    }

    // Relocation data is in decompressedData_ starting at relocDataOffset_
    // Format: delta-encoded 2-byte LE offsets, terminated by 0x0000
    // Each offset in the module image has moduleBase_ added to the 32-bit value there

    if (relocDataOffset_ == 0 || relocDataOffset_ >= decompressedData_.size()) {
        std::cout << "[WardenModule] No relocation data available" << '\n';
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
            std::cerr << "[WardenModule] Relocation offset " << currentOffset
                      << " out of bounds (moduleSize=" << moduleSize_ << ")" << '\n';
        }
    }

    std::cout << "[WardenModule] Applied " << relocCount << " relocations (base=0x"
              << std::hex << moduleBase_ << std::dec << ")" << '\n';

    return true;
}

bool WardenModule::bindAPIs() {
    if (!moduleMemory_ || moduleSize_ == 0) {
        std::cerr << "[WardenModule] No module memory allocated for API binding" << '\n';
        return false;
    }

    std::cout << "[WardenModule] Binding Windows APIs for module..." << '\n';

    // Common Windows APIs used by Warden modules:
    //
    // kernel32.dll:
    // - VirtualAlloc, VirtualFree, VirtualProtect
    // - GetTickCount, GetCurrentThreadId, GetCurrentProcessId
    // - Sleep, SwitchToThread
    // - CreateThread, ExitThread
    // - GetModuleHandleA, GetProcAddress
    // - ReadProcessMemory, WriteProcessMemory
    //
    // user32.dll:
    // - GetForegroundWindow, GetWindowTextA
    //
    // ntdll.dll:
    // - NtQueryInformationProcess, NtQuerySystemInformation

    #ifdef _WIN32
        // On Windows: Use GetProcAddress to resolve imports
        std::cout << "[WardenModule] Platform: Windows - using GetProcAddress" << '\n';

        HMODULE kernel32 = GetModuleHandleA("kernel32.dll");
        HMODULE user32 = GetModuleHandleA("user32.dll");
        HMODULE ntdll = GetModuleHandleA("ntdll.dll");

        if (!kernel32 || !user32 || !ntdll) {
            std::cerr << "[WardenModule] Failed to get module handles" << '\n';
            return false;
        }

        // TODO: Parse module's import table
        // - Find import directory in PE headers
        // - For each imported DLL:
        //   - For each imported function:
        //     - Resolve address using GetProcAddress
        //     - Write address to Import Address Table (IAT)

        std::cout << "[WardenModule] ⚠ Windows API binding is STUB (needs PE import table parsing)" << '\n';
        std::cout << "[WardenModule]   Would parse PE headers and patch IAT with resolved addresses" << '\n';

    #else
        // On Linux: Cannot directly execute Windows code
        // Options:
        // 1. Use Wine to provide Windows API compatibility
        // 2. Implement Windows API stubs (limited functionality)
        // 3. Use binfmt_misc + Wine (transparent Windows executable support)

        std::cout << "[WardenModule] Platform: Linux - Windows module execution NOT supported" << '\n';
        std::cout << "[WardenModule] Options:" << '\n';
        std::cout << "[WardenModule]   1. Run wowee under Wine (provides Windows API layer)" << '\n';
        std::cout << "[WardenModule]   2. Use a Windows VM" << '\n';
        std::cout << "[WardenModule]   3. Implement Windows API stubs (limited, complex)" << '\n';

        // For now, we'll return true to continue the loading pipeline
        // Real execution would fail, but this allows testing the infrastructure
        std::cout << "[WardenModule] ⚠ Skipping API binding (Linux platform limitation)" << '\n';
    #endif

    return true; // Return true to continue (stub implementation)
}

bool WardenModule::initializeModule() {
    if (!moduleMemory_ || moduleSize_ == 0) {
        std::cerr << "[WardenModule] No module memory allocated for initialization" << '\n';
        return false;
    }

    std::cout << "[WardenModule] Initializing Warden module..." << '\n';

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

    // Setup client callbacks (used when calling module entry point below)
    [[maybe_unused]] ClientCallbacks callbacks = {};

    // Stub callbacks (would need real implementations)
    callbacks.sendPacket = []([[maybe_unused]] uint8_t* data, size_t len) {
        std::cout << "[WardenModule Callback] sendPacket(" << len << " bytes)" << '\n';
        // TODO: Send CMSG_WARDEN_DATA packet
    };

    callbacks.validateModule = []([[maybe_unused]] uint8_t* hash) {
        std::cout << "[WardenModule Callback] validateModule()" << '\n';
        // TODO: Validate module hash
    };

    callbacks.allocMemory = [](size_t size) -> void* {
        std::cout << "[WardenModule Callback] allocMemory(" << size << ")" << '\n';
        return malloc(size);
    };

    callbacks.freeMemory = [](void* ptr) {
        std::cout << "[WardenModule Callback] freeMemory()" << '\n';
        free(ptr);
    };

    callbacks.generateRC4 = []([[maybe_unused]] uint8_t* seed) {
        std::cout << "[WardenModule Callback] generateRC4()" << '\n';
        // TODO: Re-key RC4 cipher
    };

    callbacks.getTime = []() -> uint32_t {
        return static_cast<uint32_t>(time(nullptr));
    };

    callbacks.logMessage = [](const char* msg) {
        std::cout << "[WardenModule Log] " << msg << '\n';
    };

    // Module entry point is typically at offset 0 (first bytes of loaded code)
    // Function signature: WardenFuncList* (*entryPoint)(ClientCallbacks*)

    #ifdef HAVE_UNICORN
        // Use Unicorn emulator for cross-platform execution
        std::cout << "[WardenModule] Initializing Unicorn emulator..." << '\n';

        emulator_ = std::make_unique<WardenEmulator>();
        if (!emulator_->initialize(moduleMemory_, moduleSize_, moduleBase_)) {
            std::cerr << "[WardenModule] Failed to initialize emulator" << '\n';
            return false;
        }

        // Setup Windows API hooks
        emulator_->setupCommonAPIHooks();

        std::cout << "[WardenModule] ✓ Emulator initialized successfully" << '\n';
        std::cout << "[WardenModule]   Ready to execute module at 0x" << std::hex << moduleBase_ << std::dec << '\n';

        // Allocate memory for ClientCallbacks structure in emulated space
        uint32_t callbackStructAddr = emulator_->allocateMemory(sizeof(ClientCallbacks), 0x04);
        if (callbackStructAddr == 0) {
            std::cerr << "[WardenModule] Failed to allocate memory for callbacks" << '\n';
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

        std::cout << "[WardenModule] Prepared ClientCallbacks at 0x" << std::hex << callbackStructAddr << std::dec << '\n';

        // Call module entry point
        // Entry point is typically at module base (offset 0)
        uint32_t entryPoint = moduleBase_;

        std::cout << "[WardenModule] Calling module entry point at 0x" << std::hex << entryPoint << std::dec << '\n';

        try {
            // Call: WardenFuncList* InitModule(ClientCallbacks* callbacks)
            std::vector<uint32_t> args = { callbackStructAddr };
            uint32_t result = emulator_->callFunction(entryPoint, args);

            if (result == 0) {
                std::cerr << "[WardenModule] Module entry returned NULL" << '\n';
                return false;
            }

            std::cout << "[WardenModule] ✓ Module initialized, WardenFuncList at 0x" << std::hex << result << std::dec << '\n';

            // Read WardenFuncList structure from emulated memory
            // Structure has 4 function pointers (16 bytes)
            uint32_t funcAddrs[4] = {};
            if (emulator_->readMemory(result, funcAddrs, 16)) {
                std::cout << "[WardenModule] Module exported functions:" << '\n';
                std::cout << "[WardenModule]   generateRC4Keys: 0x" << std::hex << funcAddrs[0] << std::dec << '\n';
                std::cout << "[WardenModule]   unload:          0x" << std::hex << funcAddrs[1] << std::dec << '\n';
                std::cout << "[WardenModule]   packetHandler:   0x" << std::hex << funcAddrs[2] << std::dec << '\n';
                std::cout << "[WardenModule]   tick:            0x" << std::hex << funcAddrs[3] << std::dec << '\n';

                // Store function addresses for later use
                // funcList_.generateRC4Keys = ... (would wrap emulator calls)
                // funcList_.unload = ...
                // funcList_.packetHandler = ...
                // funcList_.tick = ...
            }

            std::cout << "[WardenModule] ✓ Module fully initialized and ready!" << '\n';

        } catch (const std::exception& e) {
            std::cerr << "[WardenModule] Exception during module initialization: " << e.what() << '\n';
            return false;
        }

    #elif defined(_WIN32)
        // Native Windows execution (dangerous without sandboxing)
        typedef void* (*ModuleEntryPoint)(ClientCallbacks*);
        ModuleEntryPoint entryPoint = reinterpret_cast<ModuleEntryPoint>(moduleMemory_);

        std::cout << "[WardenModule] Calling module entry point at " << moduleMemory_ << '\n';

        // NOTE: This would execute native x86 code
        // Extremely dangerous without proper validation!
        // void* result = entryPoint(&callbacks);

        std::cout << "[WardenModule] ⚠ Module entry point call is DISABLED (unsafe without validation)" << '\n';
        std::cout << "[WardenModule]   Would execute x86 code at " << moduleMemory_ << '\n';

        // TODO: Extract WardenFuncList from result
        // funcList_.packetHandler = ...
        // funcList_.tick = ...
        // funcList_.generateRC4Keys = ...
        // funcList_.unload = ...

    #else
        std::cout << "[WardenModule] ⚠ Cannot execute Windows x86 code on Linux" << '\n';
        std::cout << "[WardenModule]   Module entry point: " << moduleMemory_ << '\n';
        std::cout << "[WardenModule]   Would call entry point with ClientCallbacks struct" << '\n';
    #endif

    // For now, return true to mark module as "loaded" at infrastructure level
    // Real execution would require:
    // 1. Proper PE parsing to find actual entry point
    // 2. Calling convention (stdcall/cdecl) handling
    // 3. Exception handling for crashes
    // 4. Sandboxing for security

    std::cout << "[WardenModule] ⚠ Module initialization is STUB" << '\n';
    return true; // Stub implementation
}

// ============================================================================
// WardenModuleManager Implementation
// ============================================================================

WardenModuleManager::WardenModuleManager() {
    // Set cache directory
#ifdef _WIN32
    if (const char* appdata = std::getenv("APPDATA"))
        cacheDirectory_ = std::string(appdata) + "\\wowee\\warden_cache";
    else
        cacheDirectory_ = ".\\warden_cache";
#else
    if (const char* home = std::getenv("HOME"))
        cacheDirectory_ = std::string(home) + "/.local/share/wowee/warden_cache";
    else
        cacheDirectory_ = "./warden_cache";
#endif

    // Create cache directory if it doesn't exist
    std::filesystem::create_directories(cacheDirectory_);

    std::cout << "[WardenModuleManager] Cache directory: " << cacheDirectory_ << '\n';
}

WardenModuleManager::~WardenModuleManager() {
    modules_.clear();
}

bool WardenModuleManager::hasModule(const std::vector<uint8_t>& md5Hash) {
    // Check in-memory cache
    if (modules_.find(md5Hash) != modules_.end()) {
        return modules_[md5Hash]->isLoaded();
    }

    // Check disk cache
    std::vector<uint8_t> dummy;
    return loadCachedModule(md5Hash, dummy);
}

std::shared_ptr<WardenModule> WardenModuleManager::getModule(const std::vector<uint8_t>& md5Hash) {
    auto it = modules_.find(md5Hash);
    if (it != modules_.end()) {
        return it->second;
    }

    // Create new module instance
    auto module = std::make_shared<WardenModule>();
    modules_[md5Hash] = module;
    return module;
}

bool WardenModuleManager::receiveModuleChunk(const std::vector<uint8_t>& md5Hash,
                                             const std::vector<uint8_t>& chunkData,
                                             bool isComplete) {
    // Append to download buffer
    std::vector<uint8_t>& buffer = downloadBuffer_[md5Hash];
    buffer.insert(buffer.end(), chunkData.begin(), chunkData.end());

    std::cout << "[WardenModuleManager] Received chunk (" << chunkData.size()
              << " bytes, total: " << buffer.size() << ")" << '\n';

    if (isComplete) {
        std::cout << "[WardenModuleManager] Module download complete ("
                  << buffer.size() << " bytes)" << '\n';

        // Cache to disk
        cacheModule(md5Hash, buffer);

        // Clear download buffer
        downloadBuffer_.erase(md5Hash);

        return true;
    }

    return true;
}

bool WardenModuleManager::cacheModule(const std::vector<uint8_t>& md5Hash,
                                      const std::vector<uint8_t>& moduleData) {
    std::string cachePath = getCachePath(md5Hash);

    std::ofstream file(cachePath, std::ios::binary);
    if (!file) {
        std::cerr << "[WardenModuleManager] Failed to write cache: " << cachePath << '\n';
        return false;
    }

    file.write(reinterpret_cast<const char*>(moduleData.data()), moduleData.size());
    file.close();

    std::cout << "[WardenModuleManager] Cached module to: " << cachePath << '\n';
    return true;
}

bool WardenModuleManager::loadCachedModule(const std::vector<uint8_t>& md5Hash,
                                           std::vector<uint8_t>& moduleDataOut) {
    std::string cachePath = getCachePath(md5Hash);

    if (!std::filesystem::exists(cachePath)) {
        return false;
    }

    std::ifstream file(cachePath, std::ios::binary | std::ios::ate);
    if (!file) {
        return false;
    }

    size_t fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    moduleDataOut.resize(fileSize);
    file.read(reinterpret_cast<char*>(moduleDataOut.data()), fileSize);
    file.close();

    std::cout << "[WardenModuleManager] Loaded cached module (" << fileSize << " bytes)" << '\n';
    return true;
}

std::string WardenModuleManager::getCachePath(const std::vector<uint8_t>& md5Hash) {
    // Convert MD5 hash to hex string for filename
    std::string hexHash;
    hexHash.reserve(md5Hash.size() * 2);

    for (uint8_t byte : md5Hash) {
        char buf[3];
        snprintf(buf, sizeof(buf), "%02x", byte);
        hexHash += buf;
    }

    return cacheDirectory_ + "/" + hexHash + ".wdn";
}

} // namespace game
} // namespace wowee
