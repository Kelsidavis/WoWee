#include "auth/pin_auth.hpp"
#include "auth/crypto.hpp"
#include <algorithm>
#include <random>
#include <stdexcept>
#include <vector>

namespace wowee {
namespace auth {

static std::array<uint8_t, 16> randomSalt16() {
    std::array<uint8_t, 16> out{};
    std::random_device rd;
    for (auto& b : out) {
        b = static_cast<uint8_t>(rd() & 0xFFu);
    }
    return out;
}

static std::array<uint8_t, 10> remapPinGrid(uint32_t seed) {
    // Generates a permutation of digits 0..9 from a seed.
    // Based on:
    // https://gtker.com/wow_messages/docs/auth/pin.html
    uint32_t v = seed;
    std::array<uint8_t, 10> remapped{};
    uint8_t used = 0;
    for (int i = 0; i < 10; ++i) {
        uint32_t divisor = 10 - i;
        uint32_t remainder = v % divisor;
        v /= divisor;

        uint32_t index = 0;
        for (uint32_t j = 0; j < 10; ++j) {
            if (used & (1u << j)) {
                continue;
            }
            if (index == remainder) {
                used = static_cast<uint8_t>(used | (1u << j));
                remapped[i] = static_cast<uint8_t>(j);
                break;
            }
            ++index;
        }
    }
    return remapped;
}

static std::vector<uint8_t> randomizePinDigits(const std::string& pinDigits,
                                               const std::array<uint8_t, 10>& remapped) {
    // Transforms each pin digit into an index in the remapped permutation.
    // Based on:
    // https://gtker.com/wow_messages/docs/auth/pin.html
    std::vector<uint8_t> out;
    out.reserve(pinDigits.size());

    for (char c : pinDigits) {
        uint8_t d = static_cast<uint8_t>(c - '0');
        uint8_t idx = 0xFF;
        for (uint8_t j = 0; j < 10; ++j) {
            if (remapped[j] == d) { idx = j; break; }
        }
        if (idx == 0xFF) {
            throw std::runtime_error("PIN digit not found in remapped grid");
        }
        out.push_back(static_cast<uint8_t>(idx + 0x30)); // ASCII '0'+idx
    }

    return out;
}

PinProof computePinProof(const std::string& pinDigits,
                         uint32_t pinGridSeed,
                         const std::array<uint8_t, 16>& serverSalt) {
    if (pinDigits.size() < 4 || pinDigits.size() > 10) {
        throw std::runtime_error("PIN must be 4-10 digits");
    }
    if (!std::all_of(pinDigits.begin(), pinDigits.end(),
                     [](unsigned char c) { return c >= '0' && c <= '9'; })) {
        throw std::runtime_error("PIN must contain only digits");
    }

    const auto remapped = remapPinGrid(pinGridSeed);
    const auto randomizedAsciiDigits = randomizePinDigits(pinDigits, remapped);

    // server_hash = SHA1(server_salt || randomized_pin_ascii)
    std::vector<uint8_t> serverHashInput;
    serverHashInput.reserve(serverSalt.size() + randomizedAsciiDigits.size());
    serverHashInput.insert(serverHashInput.end(), serverSalt.begin(), serverSalt.end());
    serverHashInput.insert(serverHashInput.end(), randomizedAsciiDigits.begin(), randomizedAsciiDigits.end());
    const auto serverHash = Crypto::sha1(serverHashInput); // 20 bytes

    PinProof proof;
    proof.clientSalt = randomSalt16();

    // final_hash = SHA1(client_salt || server_hash)
    std::vector<uint8_t> finalInput;
    finalInput.reserve(proof.clientSalt.size() + serverHash.size());
    finalInput.insert(finalInput.end(), proof.clientSalt.begin(), proof.clientSalt.end());
    finalInput.insert(finalInput.end(), serverHash.begin(), serverHash.end());
    const auto finalHash = Crypto::sha1(finalInput);
    std::copy_n(finalHash.begin(), proof.hash.size(), proof.hash.begin());

    return proof;
}

} // namespace auth
} // namespace wowee

