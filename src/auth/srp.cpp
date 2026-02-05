#include "auth/srp.hpp"
#include "auth/crypto.hpp"
#include "core/logger.hpp"
#include <algorithm>
#include <cctype>
#include <sstream>
#include <iomanip>

namespace wowee {
namespace auth {

SRP::SRP() : k(K_VALUE) {
    LOG_DEBUG("SRP instance created");
}

void SRP::initialize(const std::string& username, const std::string& password) {
    LOG_DEBUG("Initializing SRP with username: ", username);

    // Store credentials for later use
    stored_username = username;
    stored_password = password;

    initialized = true;
    LOG_DEBUG("SRP initialized");
}

void SRP::feed(const std::vector<uint8_t>& B_bytes,
               const std::vector<uint8_t>& g_bytes,
               const std::vector<uint8_t>& N_bytes,
               const std::vector<uint8_t>& salt_bytes) {

    if (!initialized) {
        LOG_ERROR("SRP not initialized! Call initialize() first.");
        return;
    }

    LOG_DEBUG("Feeding SRP challenge data");
    LOG_DEBUG("  B size: ", B_bytes.size(), " bytes");
    LOG_DEBUG("  g size: ", g_bytes.size(), " bytes");
    LOG_DEBUG("  N size: ", N_bytes.size(), " bytes");
    LOG_DEBUG("  salt size: ", salt_bytes.size(), " bytes");

    // Store server values (all little-endian)
    this->B = BigNum(B_bytes, true);
    this->g = BigNum(g_bytes, true);
    this->N = BigNum(N_bytes, true);
    this->s = BigNum(salt_bytes, true);

    LOG_DEBUG("SRP challenge data loaded");

    // Now compute everything in sequence

    // 1. Compute auth hash: H(I:P)
    std::vector<uint8_t> auth_hash = computeAuthHash(stored_username, stored_password);

    // 2. Compute x = H(s | H(I:P))
    std::vector<uint8_t> x_input;
    x_input.insert(x_input.end(), salt_bytes.begin(), salt_bytes.end());
    x_input.insert(x_input.end(), auth_hash.begin(), auth_hash.end());
    std::vector<uint8_t> x_bytes = Crypto::sha1(x_input);
    x = BigNum(x_bytes, true);
    LOG_DEBUG("Computed x (salted password hash)");

    // 3. Generate client ephemeral (a, A)
    computeClientEphemeral();

    // 4. Compute session key (S, K)
    computeSessionKey();

    // 5. Compute proofs (M1, M2)
    computeProofs(stored_username);

    // Log key values for debugging auth issues
    auto hexStr = [](const std::vector<uint8_t>& v, size_t maxBytes = 8) -> std::string {
        std::ostringstream ss;
        for (size_t i = 0; i < std::min(v.size(), maxBytes); ++i)
            ss << std::hex << std::setfill('0') << std::setw(2) << (int)v[i];
        if (v.size() > maxBytes) ss << "...";
        return ss.str();
    };
    auto A_wire = A.toArray(true, 32);
    auto s_dbg = s.toArray(true);
    auto B_dbg = B.toArray(true);
    LOG_INFO("SRP ready: A=", hexStr(A_wire), " M1=", hexStr(M1),
             " s_nat=", s_dbg.size(), " A_nat=", A.toArray(true).size(),
             " B_nat=", B_dbg.size());
}

std::vector<uint8_t> SRP::computeAuthHash(const std::string& username,
                                           const std::string& password) const {
    // Convert to uppercase (WoW requirement)
    std::string upperUser = username;
    std::string upperPass = password;
    std::transform(upperUser.begin(), upperUser.end(), upperUser.begin(), ::toupper);
    std::transform(upperPass.begin(), upperPass.end(), upperPass.begin(), ::toupper);

    // H(I:P)
    std::string combined = upperUser + ":" + upperPass;
    return Crypto::sha1(combined);
}

void SRP::computeClientEphemeral() {
    LOG_DEBUG("Computing client ephemeral");

    // Generate random private ephemeral a (19 bytes = 152 bits)
    // Keep trying until we get a valid A
    int attempts = 0;
    while (attempts < 100) {
        a = BigNum::fromRandom(19);

        // A = g^a mod N
        A = g.modPow(a, N);

        // Ensure A is not zero
        if (!A.mod(N).isZero()) {
            LOG_DEBUG("Generated valid client ephemeral after ", attempts + 1, " attempts");
            break;
        }
        attempts++;
    }

    if (attempts >= 100) {
        LOG_ERROR("Failed to generate valid client ephemeral after 100 attempts!");
    }
}

void SRP::computeSessionKey() {
    LOG_DEBUG("Computing session key");

    // u = H(A | B) - scrambling parameter
    // Use natural BigNum sizes to match TrinityCore's UpdateBigNumbers behavior
    std::vector<uint8_t> A_bytes_u = A.toArray(true);
    std::vector<uint8_t> B_bytes_u = B.toArray(true);

    std::vector<uint8_t> AB;
    AB.insert(AB.end(), A_bytes_u.begin(), A_bytes_u.end());
    AB.insert(AB.end(), B_bytes_u.begin(), B_bytes_u.end());

    std::vector<uint8_t> u_bytes = Crypto::sha1(AB);
    u = BigNum(u_bytes, true);

    LOG_DEBUG("Scrambler u calculated");

    // Compute session key: S = (B - kg^x)^(a + ux) mod N

    // Step 1: kg^x mod N
    BigNum gx = g.modPow(x, N);
    BigNum kgx = k.multiply(gx);

    // Step 2: B - kg^x (add k*N first to prevent negative result)
    BigNum kN = k.multiply(N);
    BigNum diff = B.add(kN).subtract(kgx);

    // Step 3: a + ux
    BigNum ux = u.multiply(x);
    BigNum aux = a.add(ux);

    // Step 4: (B + kN - kg^x)^(a + ux) mod N
    S = diff.modPow(aux, N);

    LOG_DEBUG("Session key S calculated");

    // Interleave the session key to create K
    // Split S into even and odd bytes, hash each half, then interleave
    std::vector<uint8_t> S_bytes = S.toArray(true, 32);  // 32 bytes for WoW

    std::vector<uint8_t> S1, S2;
    for (size_t i = 0; i < 16; ++i) {
        S1.push_back(S_bytes[i * 2]);       // Even indices
        S2.push_back(S_bytes[i * 2 + 1]);   // Odd indices
    }

    // Hash each half
    std::vector<uint8_t> S1_hash = Crypto::sha1(S1);  // 20 bytes
    std::vector<uint8_t> S2_hash = Crypto::sha1(S2);  // 20 bytes

    // Interleave the hashes to create K (40 bytes total)
    K.clear();
    K.reserve(40);
    for (size_t i = 0; i < 20; ++i) {
        K.push_back(S1_hash[i]);
        K.push_back(S2_hash[i]);
    }

    LOG_DEBUG("Interleaved session key K created (", K.size(), " bytes)");
}

void SRP::computeProofs(const std::string& username) {
    LOG_DEBUG("Computing authentication proofs");

    // Convert username to uppercase
    std::string upperUser = username;
    std::transform(upperUser.begin(), upperUser.end(), upperUser.begin(), ::toupper);

    // Compute H(N) and H(g) using natural BigNum sizes
    // This matches TrinityCore/AzerothCore's UpdateBigNumbers behavior
    std::vector<uint8_t> N_bytes = N.toArray(true);
    std::vector<uint8_t> g_bytes = g.toArray(true);

    std::vector<uint8_t> N_hash = Crypto::sha1(N_bytes);
    std::vector<uint8_t> g_hash = Crypto::sha1(g_bytes);

    // XOR them: H(N) ^ H(g)
    std::vector<uint8_t> Ng_xor(20);
    for (size_t i = 0; i < 20; ++i) {
        Ng_xor[i] = N_hash[i] ^ g_hash[i];
    }

    // Compute H(username)
    std::vector<uint8_t> user_hash = Crypto::sha1(upperUser);

    // Get A, B, and salt as byte arrays — natural sizes for hash inputs
    std::vector<uint8_t> A_bytes = A.toArray(true);
    std::vector<uint8_t> B_bytes = B.toArray(true);
    std::vector<uint8_t> s_bytes = s.toArray(true);

    // M1 = H( H(N)^H(g) | H(I) | s | A | B | K )
    std::vector<uint8_t> M1_input;
    M1_input.insert(M1_input.end(), Ng_xor.begin(), Ng_xor.end());
    M1_input.insert(M1_input.end(), user_hash.begin(), user_hash.end());
    M1_input.insert(M1_input.end(), s_bytes.begin(), s_bytes.end());
    M1_input.insert(M1_input.end(), A_bytes.begin(), A_bytes.end());
    M1_input.insert(M1_input.end(), B_bytes.begin(), B_bytes.end());
    M1_input.insert(M1_input.end(), K.begin(), K.end());

    M1 = Crypto::sha1(M1_input);

    LOG_DEBUG("Client proof M1 calculated (", M1.size(), " bytes)");
    LOG_DEBUG("  M1 hash input sizes: Ng_xor=20 user=20 s=", s_bytes.size(),
              " A=", A_bytes.size(), " B=", B_bytes.size(), " K=", K.size());

    // M2 = H( A | M1 | K )
    std::vector<uint8_t> M2_input;
    M2_input.insert(M2_input.end(), A_bytes.begin(), A_bytes.end());
    M2_input.insert(M2_input.end(), M1.begin(), M1.end());
    M2_input.insert(M2_input.end(), K.begin(), K.end());

    M2 = Crypto::sha1(M2_input);

    LOG_DEBUG("Expected server proof M2 calculated (", M2.size(), " bytes)");
}

std::vector<uint8_t> SRP::getA() const {
    if (A.isZero()) {
        LOG_WARNING("Client ephemeral A not yet computed!");
    }
    return A.toArray(true, 32);  // 32 bytes, little-endian
}

std::vector<uint8_t> SRP::getM1() const {
    if (M1.empty()) {
        LOG_WARNING("Client proof M1 not yet computed!");
    }
    return M1;
}

bool SRP::verifyServerProof(const std::vector<uint8_t>& serverM2) const {
    if (M2.empty()) {
        LOG_ERROR("Expected server proof M2 not computed!");
        return false;
    }

    if (serverM2.size() != M2.size()) {
        LOG_ERROR("Server proof size mismatch: ", serverM2.size(), " vs ", M2.size());
        return false;
    }

    bool match = std::equal(M2.begin(), M2.end(), serverM2.begin());

    if (match) {
        LOG_INFO("Server proof verified successfully!");
    } else {
        LOG_ERROR("Server proof verification FAILED!");
    }

    return match;
}

std::vector<uint8_t> SRP::getSessionKey() const {
    if (K.empty()) {
        LOG_WARNING("Session key K not yet computed!");
    }
    return K;
}

} // namespace auth
} // namespace wowee
