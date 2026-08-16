#include "auth/rc4.hpp"
#include "core/logger.hpp"
#include <cstring>

namespace wowee {
namespace auth {

RC4::RC4() {
    // Initialize state to identity
    for (int i = 0; i < 256; ++i) {
        state[i] = static_cast<uint8_t>(i);
    }
}

void RC4::init(const std::vector<uint8_t>& key) {
    if (key.empty()) {
        LOG_ERROR("RC4: Cannot initialize with empty key");
        return;
    }

    // Reset indices
    x = 0;
    y = 0;

    // Initialize state
    for (int i = 0; i < 256; ++i) {
        state[i] = static_cast<uint8_t>(i);
    }

    // Key scheduling algorithm (KSA)
    uint8_t j = 0;
    for (int i = 0; i < 256; ++i) {
        j = j + state[i] + key[i % key.size()];

        // Swap state[i] and state[j]
        uint8_t temp = state[i];
        state[i] = state[j];
        state[j] = temp;
    }

    LOG_DEBUG("RC4: Initialized with ", key.size(), "-byte key");
}

void RC4::process(uint8_t* data, size_t length) {
    if (!data || length == 0) {
        return;
    }

    // Pseudo-random generation algorithm (PRGA)
    for (size_t n = 0; n < length; ++n) {
        // Increment indices
        x = x + 1;
        y = y + state[x];

        // Swap state[x] and state[y]
        uint8_t temp = state[x];
        state[x] = state[y];
        state[y] = temp;

        // Generate keystream byte and XOR with data
        uint8_t keystreamByte = state[(state[x] + state[y]) & 0xFF];
        data[n] ^= keystreamByte;
    }
}

void RC4::drop(size_t count) {
    // Drop keystream bytes by processing zeros
    std::vector<uint8_t> dummy(count, 0);
    process(dummy.data(), count);

    LOG_DEBUG("RC4: Dropped ", count, " keystream bytes");
}

} // namespace auth
} // namespace wowee
