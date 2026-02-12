#include "auth/crypto.hpp"
#include <openssl/sha.h>
#include <openssl/md5.h>
#include <openssl/hmac.h>

namespace wowee {
namespace auth {

std::vector<uint8_t> Crypto::sha1(const std::vector<uint8_t>& data) {
    std::vector<uint8_t> hash(SHA_DIGEST_LENGTH);
    SHA1(data.data(), data.size(), hash.data());
    return hash;
}

std::vector<uint8_t> Crypto::sha1(const std::string& data) {
    std::vector<uint8_t> bytes(data.begin(), data.end());
    return sha1(bytes);
}

std::vector<uint8_t> Crypto::md5(const std::vector<uint8_t>& data) {
    std::vector<uint8_t> hash(MD5_DIGEST_LENGTH);
    MD5(data.data(), data.size(), hash.data());
    return hash;
}

std::vector<uint8_t> Crypto::md5(const std::string& data) {
    std::vector<uint8_t> bytes(data.begin(), data.end());
    return md5(bytes);
}

std::vector<uint8_t> Crypto::hmacSHA1(const std::vector<uint8_t>& key,
                                        const std::vector<uint8_t>& data) {
    std::vector<uint8_t> hash(SHA_DIGEST_LENGTH);
    unsigned int length = 0;

    HMAC(EVP_sha1(),
         key.data(), key.size(),
         data.data(), data.size(),
         hash.data(), &length);

    return hash;
}

} // namespace auth
} // namespace wowee
