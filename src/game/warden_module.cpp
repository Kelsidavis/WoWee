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

#ifndef _WIN32
    #include <sys/mman.h>
    #include <cerrno>
#endif

namespace wowee {
namespace game {

// ============================================================================
// WardenModule Implementation
// ============================================================================

WardenModule::WardenModule()
    : loaded_(false)
    , moduleMemory_(nullptr)
    , moduleSize_(0)
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
    std::cout << "...)" << std::endl;

    // Step 1: Verify MD5 hash
    if (!verifyMD5(moduleData, md5Hash)) {
        std::cerr << "[WardenModule] MD5 verification failed!" << std::endl;
        return false;
    }
    std::cout << "[WardenModule] ✓ MD5 verified" << std::endl;

    // Step 2: RC4 decrypt
    if (!decryptRC4(moduleData, rc4Key, decryptedData_)) {
        std::cerr << "[WardenModule] RC4 decryption failed!" << std::endl;
        return false;
    }
    std::cout << "[WardenModule] ✓ RC4 decrypted (" << decryptedData_.size() << " bytes)" << std::endl;

    // Step 3: Verify RSA signature
    if (!verifyRSASignature(decryptedData_)) {
        std::cerr << "[WardenModule] RSA signature verification failed!" << std::endl;
        // Note: Currently returns true (skipping verification) due to placeholder modulus
    }

    // Step 4: zlib decompress
    if (!decompressZlib(decryptedData_, decompressedData_)) {
        std::cerr << "[WardenModule] zlib decompression failed!" << std::endl;
        return false;
    }

    // Step 5: Parse custom executable format
    if (!parseExecutableFormat(decompressedData_)) {
        std::cerr << "[WardenModule] Executable format parsing failed!" << std::endl;
        return false;
    }

    // Step 6: Apply relocations
    if (!applyRelocations()) {
        std::cerr << "[WardenModule] Address relocations failed!" << std::endl;
        return false;
    }

    // Step 7: Bind APIs
    if (!bindAPIs()) {
        std::cerr << "[WardenModule] API binding failed!" << std::endl;
        // Note: Currently returns true (stub) on both Windows and Linux
    }

    // Step 8: Initialize module
    if (!initializeModule()) {
        std::cerr << "[WardenModule] Module initialization failed!" << std::endl;
        return false;
    }

    // Module loading pipeline complete!
    // Note: Steps 6-8 are stubs/platform-limited, but infrastructure is ready
    loaded_ = true; // Mark as loaded (infrastructure complete)

    std::cout << "[WardenModule] ✓ Module loading pipeline COMPLETE" << std::endl;
    std::cout << "[WardenModule]   Status: Infrastructure ready, execution stubs in place" << std::endl;
    std::cout << "[WardenModule]   Limitations:" << std::endl;
    std::cout << "[WardenModule]     - Relocations: needs real module data" << std::endl;
    std::cout << "[WardenModule]     - API Binding: Windows only (or Wine on Linux)" << std::endl;
    std::cout << "[WardenModule]     - Execution: disabled (unsafe without validation)" << std::endl;
    std::cout << "[WardenModule]   For strict servers: Would need to enable actual x86 execution" << std::endl;

    return true;
}

bool WardenModule::processCheckRequest(const std::vector<uint8_t>& checkData,
                                       std::vector<uint8_t>& responseOut) {
    if (!loaded_) {
        std::cerr << "[WardenModule] Module not loaded, cannot process checks" << std::endl;
        return false;
    }

    // TODO: Call module's PacketHandler function
    // This would execute native x86 code to:
    // - Parse check opcodes (0xF3 MEM_CHECK, 0xB2 PAGE_CHECK, etc.)
    // - Read actual memory from process
    // - Compute real SHA1 hashes
    // - Scan MPQ files
    // - Generate authentic response data

    std::cout << "[WardenModule] ⚠ processCheckRequest NOT IMPLEMENTED" << std::endl;
    std::cout << "[WardenModule]   Would call module->PacketHandler() here" << std::endl;

    // For now, return false to fall back to fake responses in GameHandler
    return false;
}

uint32_t WardenModule::tick(uint32_t deltaMs) {
    if (!loaded_ || !funcList_.tick) {
        return 0; // No tick needed
    }

    // TODO: Call module's Tick function
    // return funcList_.tick(deltaMs);

    return 0;
}

void WardenModule::generateRC4Keys(uint8_t* packet) {
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
            std::cout << "[WardenModule] Calling module unload callback..." << std::endl;
            // TODO: Implement callback when execution layer is complete
            // funcList_.unload(nullptr);
        }

        // Free executable memory region
        std::cout << "[WardenModule] Freeing " << moduleSize_ << " bytes of executable memory" << std::endl;
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
        std::cerr << "[WardenModule] Invalid RC4 key size: " << key.size() << " (expected 16)" << std::endl;
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
        std::cerr << "[WardenModule] Data too small for RSA signature (need at least 256 bytes)" << std::endl;
        return false;
    }

    // Extract signature (last 256 bytes)
    std::vector<uint8_t> signature(data.end() - 256, data.end());

    // Extract data without signature
    std::vector<uint8_t> dataWithoutSig(data.begin(), data.end() - 256);

    // Hardcoded WoW 3.3.5a Warden RSA public key
    // Exponent: 0x010001 (65537)
    const uint32_t exponent = 0x010001;

    // Modulus (256 bytes) - This is the actual public key from WoW 3.3.5a client
    // TODO: Extract this from WoW.exe binary at offset (varies by build)
    // For now, using a placeholder that will fail verification
    // To get the real modulus: extract from WoW.exe using a hex editor or IDA Pro
    const uint8_t modulus[256] = {
        // PLACEHOLDER - Replace with actual modulus from WoW 3.3.5a (build 12340)
        // This can be extracted from the WoW client binary
        // The actual value varies by client version and build
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    // Compute expected hash: SHA1(data_without_sig + "MAIEV.MOD")
    std::vector<uint8_t> dataToHash = dataWithoutSig;
    const char* suffix = "MAIEV.MOD";
    dataToHash.insert(dataToHash.end(), suffix, suffix + strlen(suffix));

    std::vector<uint8_t> expectedHash = auth::Crypto::sha1(dataToHash);

    // Create RSA public key structure
    RSA* rsa = RSA_new();
    if (!rsa) {
        std::cerr << "[WardenModule] Failed to create RSA structure" << std::endl;
        return false;
    }

    BIGNUM* n = BN_bin2bn(modulus, 256, nullptr);
    BIGNUM* e = BN_new();
    BN_set_word(e, exponent);

    #if OPENSSL_VERSION_NUMBER >= 0x10100000L
        // OpenSSL 1.1.0+
        RSA_set0_key(rsa, n, e, nullptr);
    #else
        // OpenSSL 1.0.x
        rsa->n = n;
        rsa->e = e;
    #endif

    // Decrypt signature using public key
    std::vector<uint8_t> decryptedSig(256);
    int decryptedLen = RSA_public_decrypt(
        256,
        signature.data(),
        decryptedSig.data(),
        rsa,
        RSA_NO_PADDING
    );

    RSA_free(rsa);

    if (decryptedLen < 0) {
        std::cerr << "[WardenModule] RSA public decrypt failed" << std::endl;
        return false;
    }

    // Expected format: padding (0xBB bytes) + SHA1 hash (20 bytes)
    // Total: 256 bytes decrypted
    // Find SHA1 hash in decrypted signature (should be at end, preceded by 0xBB padding)

    // Look for SHA1 hash in last 20 bytes
    if (decryptedLen >= 20) {
        std::vector<uint8_t> actualHash(decryptedSig.end() - 20, decryptedSig.end());

        if (std::memcmp(actualHash.data(), expectedHash.data(), 20) == 0) {
            std::cout << "[WardenModule] ✓ RSA signature verified" << std::endl;
            return true;
        }
    }

    std::cerr << "[WardenModule] RSA signature verification FAILED (hash mismatch)" << std::endl;
    std::cerr << "[WardenModule] NOTE: Using placeholder modulus - extract real modulus from WoW.exe for actual verification" << std::endl;

    // For development, return true to proceed (since we don't have real modulus)
    // TODO: Set to false once real modulus is extracted
    std::cout << "[WardenModule] ⚠ Skipping RSA verification (placeholder modulus)" << std::endl;
    return true; // TEMPORARY - change to false for production
}

bool WardenModule::decompressZlib(const std::vector<uint8_t>& compressed,
                                  std::vector<uint8_t>& decompressedOut) {
    if (compressed.size() < 4) {
        std::cerr << "[WardenModule] Compressed data too small (need at least 4 bytes for size header)" << std::endl;
        return false;
    }

    // Read 4-byte uncompressed size (little-endian)
    uint32_t uncompressedSize =
        compressed[0] |
        (compressed[1] << 8) |
        (compressed[2] << 16) |
        (compressed[3] << 24);

    std::cout << "[WardenModule] Uncompressed size: " << uncompressedSize << " bytes" << std::endl;

    // Sanity check (modules shouldn't be larger than 10MB)
    if (uncompressedSize > 10 * 1024 * 1024) {
        std::cerr << "[WardenModule] Uncompressed size suspiciously large: " << uncompressedSize << " bytes" << std::endl;
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
        std::cerr << "[WardenModule] inflateInit failed: " << ret << std::endl;
        return false;
    }

    // Decompress
    ret = inflate(&stream, Z_FINISH);

    // Cleanup
    inflateEnd(&stream);

    if (ret != Z_STREAM_END) {
        std::cerr << "[WardenModule] inflate failed: " << ret << std::endl;
        return false;
    }

    std::cout << "[WardenModule] ✓ zlib decompression successful ("
              << stream.total_out << " bytes decompressed)" << std::endl;

    return true;
}

bool WardenModule::parseExecutableFormat(const std::vector<uint8_t>& exeData) {
    if (exeData.size() < 4) {
        std::cerr << "[WardenModule] Executable data too small for header" << std::endl;
        return false;
    }

    // Read final code size (little-endian 4 bytes)
    uint32_t finalCodeSize =
        exeData[0] |
        (exeData[1] << 8) |
        (exeData[2] << 16) |
        (exeData[3] << 24);

    std::cout << "[WardenModule] Final code size: " << finalCodeSize << " bytes" << std::endl;

    // Sanity check (executable shouldn't be larger than 5MB)
    if (finalCodeSize > 5 * 1024 * 1024 || finalCodeSize == 0) {
        std::cerr << "[WardenModule] Invalid final code size: " << finalCodeSize << std::endl;
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
            std::cerr << "[WardenModule] VirtualAlloc failed" << std::endl;
            return false;
        }
    #else
        #include <sys/mman.h>
        moduleMemory_ = mmap(
            nullptr,
            finalCodeSize,
            PROT_READ | PROT_WRITE | PROT_EXEC,
            MAP_PRIVATE | MAP_ANONYMOUS,
            -1,
            0
        );
        if (moduleMemory_ == MAP_FAILED) {
            std::cerr << "[WardenModule] mmap failed: " << strerror(errno) << std::endl;
            moduleMemory_ = nullptr;
            return false;
        }
    #endif

    moduleSize_ = finalCodeSize;
    std::memset(moduleMemory_, 0, moduleSize_); // Zero-initialize

    std::cout << "[WardenModule] Allocated " << moduleSize_ << " bytes of executable memory at "
              << moduleMemory_ << std::endl;

    // Parse skip/copy sections
    size_t pos = 4; // Skip 4-byte size header
    size_t destOffset = 0;
    bool isSkipSection = true; // Alternates: skip, copy, skip, copy, ...
    int sectionCount = 0;

    while (pos + 2 <= exeData.size()) {
        // Read 2-byte section length (little-endian)
        uint16_t sectionLength = exeData[pos] | (exeData[pos + 1] << 8);
        pos += 2;

        if (sectionLength == 0) {
            break; // End of sections
        }

        if (pos + sectionLength > exeData.size()) {
            std::cerr << "[WardenModule] Section extends beyond data bounds" << std::endl;
            #ifdef _WIN32
                VirtualFree(moduleMemory_, 0, MEM_RELEASE);
            #else
                munmap(moduleMemory_, moduleSize_);
            #endif
            moduleMemory_ = nullptr;
            return false;
        }

        if (isSkipSection) {
            // Skip section - advance destination offset without copying
            destOffset += sectionLength;
            std::cout << "[WardenModule]   Skip section: " << sectionLength << " bytes (dest offset now "
                      << destOffset << ")" << std::endl;
        } else {
            // Copy section - copy code to module memory
            if (destOffset + sectionLength > moduleSize_) {
                std::cerr << "[WardenModule] Copy section exceeds module size" << std::endl;
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
                sectionLength
            );

            std::cout << "[WardenModule]   Copy section: " << sectionLength << " bytes to offset "
                      << destOffset << std::endl;

            destOffset += sectionLength;
        }

        pos += sectionLength;
        isSkipSection = !isSkipSection; // Alternate
        sectionCount++;
    }

    std::cout << "[WardenModule] ✓ Parsed " << sectionCount << " sections, final offset: "
              << destOffset << "/" << finalCodeSize << std::endl;

    if (destOffset != finalCodeSize) {
        std::cerr << "[WardenModule] WARNING: Final offset " << destOffset
                  << " doesn't match expected size " << finalCodeSize << std::endl;
    }

    return true;
}

bool WardenModule::applyRelocations() {
    if (!moduleMemory_ || moduleSize_ == 0) {
        std::cerr << "[WardenModule] No module memory allocated for relocations" << std::endl;
        return false;
    }

    // Relocations are embedded in the decompressed data after the executable sections
    // Format: Delta-encoded offsets with high-bit continuation
    //
    // Each offset is encoded as variable-length bytes:
    // - If high bit (0x80) is set, read next byte and combine
    // - Continue until byte without high bit
    // - Final value is delta from previous relocation offset
    //
    // For each relocation offset:
    // - Read 4-byte pointer at that offset
    // - Add module base address to make it absolute
    // - Write back to memory

    std::cout << "[WardenModule] Applying relocations to module at " << moduleMemory_ << std::endl;

    // NOTE: Relocation data format and location varies by module
    // Without a real module to test against, we can't implement this accurately
    // This is a placeholder that would need to be filled in with actual logic
    // once we have real Warden module data to analyze

    std::cout << "[WardenModule] ⚠ Relocation application is STUB (needs real module data)" << std::endl;
    std::cout << "[WardenModule]   Would parse delta-encoded offsets and fix absolute references" << std::endl;

    // Placeholder: Assume no relocations or already position-independent
    // Real implementation would:
    // 1. Find relocation table in module data
    // 2. Decode delta offsets
    // 3. For each offset: *(uint32_t*)(moduleMemory + offset) += (uintptr_t)moduleMemory_

    return true; // Return true to continue (stub implementation)
}

bool WardenModule::bindAPIs() {
    if (!moduleMemory_ || moduleSize_ == 0) {
        std::cerr << "[WardenModule] No module memory allocated for API binding" << std::endl;
        return false;
    }

    std::cout << "[WardenModule] Binding Windows APIs for module..." << std::endl;

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
        std::cout << "[WardenModule] Platform: Windows - using GetProcAddress" << std::endl;

        HMODULE kernel32 = GetModuleHandleA("kernel32.dll");
        HMODULE user32 = GetModuleHandleA("user32.dll");
        HMODULE ntdll = GetModuleHandleA("ntdll.dll");

        if (!kernel32 || !user32 || !ntdll) {
            std::cerr << "[WardenModule] Failed to get module handles" << std::endl;
            return false;
        }

        // TODO: Parse module's import table
        // - Find import directory in PE headers
        // - For each imported DLL:
        //   - For each imported function:
        //     - Resolve address using GetProcAddress
        //     - Write address to Import Address Table (IAT)

        std::cout << "[WardenModule] ⚠ Windows API binding is STUB (needs PE import table parsing)" << std::endl;
        std::cout << "[WardenModule]   Would parse PE headers and patch IAT with resolved addresses" << std::endl;

    #else
        // On Linux: Cannot directly execute Windows code
        // Options:
        // 1. Use Wine to provide Windows API compatibility
        // 2. Implement Windows API stubs (limited functionality)
        // 3. Use binfmt_misc + Wine (transparent Windows executable support)

        std::cout << "[WardenModule] Platform: Linux - Windows module execution NOT supported" << std::endl;
        std::cout << "[WardenModule] Options:" << std::endl;
        std::cout << "[WardenModule]   1. Run wowee under Wine (provides Windows API layer)" << std::endl;
        std::cout << "[WardenModule]   2. Use a Windows VM" << std::endl;
        std::cout << "[WardenModule]   3. Implement Windows API stubs (limited, complex)" << std::endl;

        // For now, we'll return true to continue the loading pipeline
        // Real execution would fail, but this allows testing the infrastructure
        std::cout << "[WardenModule] ⚠ Skipping API binding (Linux platform limitation)" << std::endl;
    #endif

    return true; // Return true to continue (stub implementation)
}

bool WardenModule::initializeModule() {
    if (!moduleMemory_ || moduleSize_ == 0) {
        std::cerr << "[WardenModule] No module memory allocated for initialization" << std::endl;
        return false;
    }

    std::cout << "[WardenModule] Initializing Warden module..." << std::endl;

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

    // Setup client callbacks
    ClientCallbacks callbacks = {};

    // Stub callbacks (would need real implementations)
    callbacks.sendPacket = [](uint8_t* data, size_t len) {
        std::cout << "[WardenModule Callback] sendPacket(" << len << " bytes)" << std::endl;
        // TODO: Send CMSG_WARDEN_DATA packet
    };

    callbacks.validateModule = [](uint8_t* hash) {
        std::cout << "[WardenModule Callback] validateModule()" << std::endl;
        // TODO: Validate module hash
    };

    callbacks.allocMemory = [](size_t size) -> void* {
        std::cout << "[WardenModule Callback] allocMemory(" << size << ")" << std::endl;
        return malloc(size);
    };

    callbacks.freeMemory = [](void* ptr) {
        std::cout << "[WardenModule Callback] freeMemory()" << std::endl;
        free(ptr);
    };

    callbacks.generateRC4 = [](uint8_t* seed) {
        std::cout << "[WardenModule Callback] generateRC4()" << std::endl;
        // TODO: Re-key RC4 cipher
    };

    callbacks.getTime = []() -> uint32_t {
        return static_cast<uint32_t>(time(nullptr));
    };

    callbacks.logMessage = [](const char* msg) {
        std::cout << "[WardenModule Log] " << msg << std::endl;
    };

    // Module entry point is typically at offset 0 (first bytes of loaded code)
    // Function signature: WardenFuncList* (*entryPoint)(ClientCallbacks*)

    #ifdef _WIN32
        typedef void* (*ModuleEntryPoint)(ClientCallbacks*);
        ModuleEntryPoint entryPoint = reinterpret_cast<ModuleEntryPoint>(moduleMemory_);

        std::cout << "[WardenModule] Calling module entry point at " << moduleMemory_ << std::endl;

        // NOTE: This would execute native x86 code
        // Extremely dangerous without proper validation!
        // void* result = entryPoint(&callbacks);

        std::cout << "[WardenModule] ⚠ Module entry point call is DISABLED (unsafe without validation)" << std::endl;
        std::cout << "[WardenModule]   Would execute x86 code at " << moduleMemory_ << std::endl;

        // TODO: Extract WardenFuncList from result
        // funcList_.packetHandler = ...
        // funcList_.tick = ...
        // funcList_.generateRC4Keys = ...
        // funcList_.unload = ...

    #else
        std::cout << "[WardenModule] ⚠ Cannot execute Windows x86 code on Linux" << std::endl;
        std::cout << "[WardenModule]   Module entry point: " << moduleMemory_ << std::endl;
        std::cout << "[WardenModule]   Would call entry point with ClientCallbacks struct" << std::endl;
    #endif

    // For now, return true to mark module as "loaded" at infrastructure level
    // Real execution would require:
    // 1. Proper PE parsing to find actual entry point
    // 2. Calling convention (stdcall/cdecl) handling
    // 3. Exception handling for crashes
    // 4. Sandboxing for security

    std::cout << "[WardenModule] ⚠ Module initialization is STUB" << std::endl;
    return true; // Stub implementation
}

// ============================================================================
// WardenModuleManager Implementation
// ============================================================================

WardenModuleManager::WardenModuleManager() {
    // Set cache directory: ~/.local/share/wowee/warden_cache/
    const char* home = std::getenv("HOME");
    if (home) {
        cacheDirectory_ = std::string(home) + "/.local/share/wowee/warden_cache";
    } else {
        cacheDirectory_ = "./warden_cache";
    }

    // Create cache directory if it doesn't exist
    std::filesystem::create_directories(cacheDirectory_);

    std::cout << "[WardenModuleManager] Cache directory: " << cacheDirectory_ << std::endl;
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
              << " bytes, total: " << buffer.size() << ")" << std::endl;

    if (isComplete) {
        std::cout << "[WardenModuleManager] Module download complete ("
                  << buffer.size() << " bytes)" << std::endl;

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
        std::cerr << "[WardenModuleManager] Failed to write cache: " << cachePath << std::endl;
        return false;
    }

    file.write(reinterpret_cast<const char*>(moduleData.data()), moduleData.size());
    file.close();

    std::cout << "[WardenModuleManager] Cached module to: " << cachePath << std::endl;
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

    std::cout << "[WardenModuleManager] Loaded cached module (" << fileSize << " bytes)" << std::endl;
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
