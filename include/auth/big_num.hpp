#pragma once

#include <vector>
#include <cstdint>
#include <string>
#include <openssl/bn.h>

namespace wowee {
namespace auth {

// Wrapper around OpenSSL BIGNUM for big integer arithmetic
class BigNum {
public:
    BigNum();
    explicit BigNum(uint32_t value);
    explicit BigNum(const std::vector<uint8_t>& bytes, bool littleEndian = true);
    ~BigNum();

    // Copy/move operations
    BigNum(const BigNum& other);
    BigNum& operator=(const BigNum& other);
    BigNum(BigNum&& other) noexcept;
    BigNum& operator=(BigNum&& other) noexcept;

    // Factory methods
    static BigNum fromRandom(int bytes);
    static BigNum fromHex(const std::string& hex);

    // Arithmetic operations
    [[nodiscard]] BigNum add(const BigNum& other) const;
    [[nodiscard]] BigNum subtract(const BigNum& other) const;
    [[nodiscard]] BigNum multiply(const BigNum& other) const;
    [[nodiscard]] BigNum mod(const BigNum& modulus) const;
    [[nodiscard]] BigNum modPow(const BigNum& exponent, const BigNum& modulus) const;

    // Comparison
    [[nodiscard]] bool equals(const BigNum& other) const;
    [[nodiscard]] bool isZero() const;

    // Conversion
    [[nodiscard]] std::vector<uint8_t> toArray(bool littleEndian = true, int minSize = 0) const;
    [[nodiscard]] std::string toHex() const;

    // Direct access (for advanced operations)
    BIGNUM* getBN() { return bn; }
    [[nodiscard]] const BIGNUM* getBN() const { return bn; }

private:
    BIGNUM* bn;
};

} // namespace auth
} // namespace wowee
