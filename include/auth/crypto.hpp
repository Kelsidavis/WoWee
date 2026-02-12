#pragma once

#include <vector>
#include <cstdint>
#include <string>

namespace wowee {
namespace auth {

class Crypto {
public:
    static std::vector<uint8_t> sha1(const std::vector<uint8_t>& data);
    static std::vector<uint8_t> sha1(const std::string& data);

    /**
     * MD5 hash (16 bytes)
     */
    static std::vector<uint8_t> md5(const std::vector<uint8_t>& data);
    static std::vector<uint8_t> md5(const std::string& data);

    /**
     * HMAC-SHA1 message authentication code
     *
     * @param key Secret key
     * @param data Data to authenticate
     * @return 20-byte HMAC-SHA1 hash
     */
    static std::vector<uint8_t> hmacSHA1(const std::vector<uint8_t>& key,
                                          const std::vector<uint8_t>& data);
};

} // namespace auth
} // namespace wowee
