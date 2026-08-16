#pragma once

#include <cstdint>
#include <vector>

namespace wowee {
namespace auth {

/**
 * RC4 Stream Cipher
 *
 * Used for encrypting/decrypting World of Warcraft packet headers.
 * Only the packet headers are encrypted; packet bodies remain plaintext.
 *
 * Implementation based on standard RC4 algorithm with 256-byte state.
 */
class RC4 {
public:
    RC4();
    ~RC4() = default;

    /**
     * Initialize the RC4 cipher with a key
     *
     * @param key Key bytes for initialization
     */
    void init(const std::vector<uint8_t>& key);

    /**
     * Process bytes through the RC4 cipher
     * Encrypts or decrypts data in-place (RC4 is symmetric)
     *
     * @param data Pointer to data to process
     * @param length Number of bytes to process
     */
    void process(uint8_t* data, size_t length);

    /**
     * Drop the first N bytes of keystream
     * WoW protocol requires dropping first 1024 bytes
     *
     * @param count Number of bytes to drop
     */
    void drop(size_t count);

private:
    uint8_t state[256];  // RC4 state array
    uint8_t x = 0;       // First index
    uint8_t y = 0;       // Second index
};

} // namespace auth
} // namespace wowee
