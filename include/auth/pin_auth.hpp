#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace wowee {
namespace auth {

struct PinProof {
    std::array<uint8_t, 16> clientSalt{};
    std::array<uint8_t, 20> hash{};
};

// Implements the "PIN" security extension used in the WoW auth protocol (securityFlags & 0x01).
// Algorithm based on documented client behavior:
// - Remap digits using pinGridSeed (a permutation of 0..9)
// - Convert user-entered PIN digits into randomized indices in that permutation
// - Compute: pin_hash = SHA1(client_salt || SHA1(server_salt || randomized_pin_ascii))
//
// PIN must be 4-10 ASCII digits.
PinProof computePinProof(const std::string& pinDigits,
                         uint32_t pinGridSeed,
                         const std::array<uint8_t, 16>& serverSalt);

} // namespace auth
} // namespace wowee

