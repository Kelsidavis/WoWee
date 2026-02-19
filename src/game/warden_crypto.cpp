#include "game/warden_crypto.hpp"
#include "auth/crypto.hpp"
#include "core/logger.hpp"
#include <cstring>
#include <algorithm>

namespace wowee {
namespace game {

WardenCrypto::WardenCrypto()
    : initialized_(false)
    , decryptRC4_i_(0)
    , decryptRC4_j_(0)
    , encryptRC4_i_(0)
    , encryptRC4_j_(0) {
}

WardenCrypto::~WardenCrypto() {
}

void WardenCrypto::sha1RandxGenerate(const std::vector<uint8_t>& seed,
                                      uint8_t* outputEncryptKey,
                                      uint8_t* outputDecryptKey) {
    // SHA1Randx / WardenKeyGenerator algorithm (from MaNGOS/VMaNGOS)
    // Split the 40-byte session key in half
    size_t half = seed.size() / 2;

    // o1 = SHA1(first half of session key)
    std::vector<uint8_t> firstHalf(seed.begin(), seed.begin() + half);
    auto o1 = auth::Crypto::sha1(firstHalf);

    // o2 = SHA1(second half of session key)
    std::vector<uint8_t> secondHalf(seed.begin() + half, seed.end());
    auto o2 = auth::Crypto::sha1(secondHalf);

    // o0 = 20 zero bytes initially
    std::vector<uint8_t> o0(20, 0);

    uint32_t taken = 20; // Force FillUp on first Generate

    // Lambda: FillUp - compute o0 = SHA1(o1 || o0 || o2)
    auto fillUp = [&]() {
        std::vector<uint8_t> combined;
        combined.reserve(60);
        combined.insert(combined.end(), o1.begin(), o1.end());
        combined.insert(combined.end(), o0.begin(), o0.end());
        combined.insert(combined.end(), o2.begin(), o2.end());
        o0 = auth::Crypto::sha1(combined);
        taken = 0;
    };

    // Generate first 16 bytes → encrypt key (client→server)
    for (int i = 0; i < 16; ++i) {
        if (taken == 20) fillUp();
        outputEncryptKey[i] = o0[taken++];
    }

    // Generate next 16 bytes → decrypt key (server→client)
    for (int i = 0; i < 16; ++i) {
        if (taken == 20) fillUp();
        outputDecryptKey[i] = o0[taken++];
    }
}

bool WardenCrypto::initFromSessionKey(const std::vector<uint8_t>& sessionKey) {
    if (sessionKey.size() != 40) {
        LOG_ERROR("Warden: Session key must be 40 bytes, got ", sessionKey.size());
        return false;
    }

    uint8_t encryptKey[16];
    uint8_t decryptKey[16];
    sha1RandxGenerate(sessionKey, encryptKey, decryptKey);

    // Warden protocol compatibility note:
    // Blizzard's Warden stream crypto is RC4-based; this cannot be upgraded
    // without breaking protocol interoperability with supported servers.
    std::vector<uint8_t> ek(encryptKey, encryptKey + 16);
    std::vector<uint8_t> dk(decryptKey, decryptKey + 16);

    encryptRC4State_.resize(256);
    decryptRC4State_.resize(256);

    initRC4(ek, encryptRC4State_, encryptRC4_i_, encryptRC4_j_);
    initRC4(dk, decryptRC4State_, decryptRC4_i_, decryptRC4_j_);

    // Scrub temporary key material after RC4 state initialization.
    std::fill(ek.begin(), ek.end(), 0);
    std::fill(dk.begin(), dk.end(), 0);
    std::fill(std::begin(encryptKey), std::end(encryptKey), 0);
    std::fill(std::begin(decryptKey), std::end(decryptKey), 0);

    initialized_ = true;
    LOG_INFO("Warden: Crypto initialized from session key");
    return true;
}

void WardenCrypto::replaceKeys(const std::vector<uint8_t>& newEncryptKey,
                                const std::vector<uint8_t>& newDecryptKey) {
    encryptRC4State_.resize(256);
    decryptRC4State_.resize(256);
    initRC4(newEncryptKey, encryptRC4State_, encryptRC4_i_, encryptRC4_j_);
    initRC4(newDecryptKey, decryptRC4State_, decryptRC4_i_, decryptRC4_j_);
    LOG_INFO("Warden: RC4 keys replaced (module-specific keys)");
}

std::vector<uint8_t> WardenCrypto::decrypt(const std::vector<uint8_t>& data) {
    if (!initialized_) {
        LOG_WARNING("Warden: Attempted to decrypt without initialization");
        return data;
    }

    std::vector<uint8_t> result(data.size());
    processRC4(data.data(), result.data(), data.size(),
               decryptRC4State_, decryptRC4_i_, decryptRC4_j_);
    return result;
}

std::vector<uint8_t> WardenCrypto::encrypt(const std::vector<uint8_t>& data) {
    if (!initialized_) {
        LOG_WARNING("Warden: Attempted to encrypt without initialization");
        return data;
    }

    std::vector<uint8_t> result(data.size());
    processRC4(data.data(), result.data(), data.size(),
               encryptRC4State_, encryptRC4_i_, encryptRC4_j_);
    return result;
}

void WardenCrypto::initRC4(const std::vector<uint8_t>& key,
                            std::vector<uint8_t>& state,
                            uint8_t& i, uint8_t& j) {
    for (int idx = 0; idx < 256; ++idx) {
        state[idx] = static_cast<uint8_t>(idx);
    }

    j = 0;
    for (int idx = 0; idx < 256; ++idx) {
        j = (j + state[idx] + key[idx % key.size()]) & 0xFF;
        std::swap(state[idx], state[j]);
    }

    i = 0;
    j = 0;
}

void WardenCrypto::processRC4(const uint8_t* input, uint8_t* output, size_t length,
                               std::vector<uint8_t>& state, uint8_t& i, uint8_t& j) {
    for (size_t idx = 0; idx < length; ++idx) {
        i = (i + 1) & 0xFF;
        j = (j + state[i]) & 0xFF;
        std::swap(state[i], state[j]);

        uint8_t k = state[(state[i] + state[j]) & 0xFF];
        output[idx] = input[idx] ^ k;
    }
}

} // namespace game
} // namespace wowee
