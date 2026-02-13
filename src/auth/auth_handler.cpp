#include "auth/auth_handler.hpp"
#include "auth/pin_auth.hpp"
#include "auth/integrity.hpp"
#include "network/tcp_socket.hpp"
#include "network/packet.hpp"
#include "core/logger.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cctype>
#include <cstdlib>

namespace wowee {
namespace auth {

AuthHandler::AuthHandler() {
    LOG_DEBUG("AuthHandler created");
}

AuthHandler::~AuthHandler() {
    disconnect();
}

bool AuthHandler::connect(const std::string& host, uint16_t port) {
    auto trimHost = [](std::string s) {
        auto isSpace = [](unsigned char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; };
        size_t b = 0;
        while (b < s.size() && isSpace(static_cast<unsigned char>(s[b]))) ++b;
        size_t e = s.size();
        while (e > b && isSpace(static_cast<unsigned char>(s[e - 1]))) --e;
        return s.substr(b, e - b);
    };

    const std::string hostTrimmed = trimHost(host);
    LOG_INFO("Connecting to auth server: ", hostTrimmed, ":", port);

    socket = std::make_unique<network::TCPSocket>();

    // Set up packet callback
    socket->setPacketCallback([this](const network::Packet& packet) {
        // Create a mutable copy for handling
        network::Packet mutablePacket = packet;
        handlePacket(mutablePacket);
    });

    if (!socket->connect(hostTrimmed, port)) {
        LOG_ERROR("Failed to connect to auth server");
        setState(AuthState::FAILED);
        return false;
    }

    setState(AuthState::CONNECTED);
    LOG_INFO("Connected to auth server");
    return true;
}

void AuthHandler::disconnect() {
    if (socket) {
        socket->disconnect();
        socket.reset();
    }
    setState(AuthState::DISCONNECTED);
    LOG_INFO("Disconnected from auth server");
}

bool AuthHandler::isConnected() const {
    return socket && socket->isConnected();
}

void AuthHandler::requestRealmList() {
    if (!isConnected()) {
        LOG_ERROR("Cannot request realm list: not connected to auth server");
        return;
    }

    if (state != AuthState::AUTHENTICATED) {
        LOG_ERROR("Cannot request realm list: not authenticated (state: ", (int)state, ")");
        return;
    }

    LOG_INFO("Requesting realm list");
    sendRealmListRequest();
}

void AuthHandler::authenticate(const std::string& user, const std::string& pass) {
    authenticate(user, pass, std::string());
}

void AuthHandler::authenticate(const std::string& user, const std::string& pass, const std::string& pin) {
    if (!isConnected()) {
        LOG_ERROR("Cannot authenticate: not connected to auth server");
        fail("Not connected");
        return;
    }

    if (state != AuthState::CONNECTED) {
        LOG_ERROR("Cannot authenticate: invalid state");
        fail("Invalid state");
        return;
    }

    LOG_INFO("Starting authentication for user: ", user);

    username = user;
    password = pass;
    pendingSecurityCode_ = pin;
    securityFlags_ = 0;
    pinGridSeed_ = 0;
    pinServerSalt_ = {};
    checksumSalt_ = {};

    // Initialize SRP
    srp = std::make_unique<SRP>();
    srp->initialize(username, password);

    // Send LOGON_CHALLENGE
    sendLogonChallenge();
}

void AuthHandler::authenticateWithHash(const std::string& user, const std::vector<uint8_t>& authHash) {
    authenticateWithHash(user, authHash, std::string());
}

void AuthHandler::authenticateWithHash(const std::string& user, const std::vector<uint8_t>& authHash, const std::string& pin) {
    if (!isConnected()) {
        LOG_ERROR("Cannot authenticate: not connected to auth server");
        fail("Not connected");
        return;
    }

    if (state != AuthState::CONNECTED) {
        LOG_ERROR("Cannot authenticate: invalid state");
        fail("Invalid state");
        return;
    }

    LOG_INFO("Starting authentication for user (with hash): ", user);

    username = user;
    password.clear();
    pendingSecurityCode_ = pin;
    securityFlags_ = 0;
    pinGridSeed_ = 0;
    pinServerSalt_ = {};
    checksumSalt_ = {};

    // Initialize SRP with pre-computed hash
    srp = std::make_unique<SRP>();
    srp->initializeWithHash(username, authHash);

    // Send LOGON_CHALLENGE
    sendLogonChallenge();
}

void AuthHandler::sendLogonChallenge() {
    LOG_DEBUG("Sending LOGON_CHALLENGE");

    auto packet = LogonChallengePacket::build(username, clientInfo);
    socket->send(packet);

    setState(AuthState::CHALLENGE_SENT);
}

void AuthHandler::handleLogonChallengeResponse(network::Packet& packet) {
    LOG_DEBUG("Handling LOGON_CHALLENGE response");

    LogonChallengeResponse response;
    if (!LogonChallengeResponseParser::parse(packet, response)) {
        fail("Server sent an invalid response - it may use an incompatible protocol version");
        return;
    }

    if (!response.isSuccess()) {
        if (response.result == AuthResult::BUILD_INVALID || response.result == AuthResult::BUILD_UPDATE) {
            std::ostringstream ss;
            ss << "LOGON_CHALLENGE failed: version mismatch (client v"
               << (int)clientInfo.majorVersion << "."
               << (int)clientInfo.minorVersion << "."
               << (int)clientInfo.patchVersion
               << " build " << clientInfo.build
               << ", auth protocol " << (int)clientInfo.protocolVersion << ")";
            fail(ss.str());
        } else {
            fail(std::string("LOGON_CHALLENGE failed: ") + getAuthResultString(response.result));
        }
        return;
    }

    if (response.securityFlags != 0) {
        LOG_WARNING("Server sent security flags: 0x", std::hex, (int)response.securityFlags, std::dec);
        if (response.securityFlags & 0x01) LOG_WARNING("  PIN required");
        if (response.securityFlags & 0x02) LOG_WARNING("  Matrix card required (not supported)");
        if (response.securityFlags & 0x04) LOG_WARNING("  Authenticator required (not supported)");
    }

    LOG_INFO("Challenge: N=", response.N.size(), "B g=", response.g.size(), "B salt=",
             response.salt.size(), "B secFlags=0x", std::hex, (int)response.securityFlags, std::dec);

    // Feed SRP with server challenge data
    srp->feed(response.B, response.g, response.N, response.salt);

    securityFlags_ = response.securityFlags;
    checksumSalt_ = response.checksumSalt;
    if (securityFlags_ & 0x01) {
        pinGridSeed_ = response.pinGridSeed;
        pinServerSalt_ = response.pinSalt;
    }

    setState(AuthState::CHALLENGE_RECEIVED);

    // If a security code is required, wait for user input.
    if (((securityFlags_ & 0x04) || (securityFlags_ & 0x01)) && pendingSecurityCode_.empty()) {
        setState((securityFlags_ & 0x04) ? AuthState::AUTHENTICATOR_REQUIRED : AuthState::PIN_REQUIRED);
        return;
    }

    sendLogonProof();
}

void AuthHandler::sendLogonProof() {
    LOG_DEBUG("Sending LOGON_PROOF");

    auto A = srp->getA();
    auto M1 = srp->getM1();

    std::array<uint8_t, 16> pinClientSalt{};
    std::array<uint8_t, 20> pinHash{};
    const std::array<uint8_t, 16>* pinClientSaltPtr = nullptr;
    const std::array<uint8_t, 20>* pinHashPtr = nullptr;
    std::array<uint8_t, 20> crcHash{};
    const std::array<uint8_t, 20>* crcHashPtr = nullptr;

    if (securityFlags_ & 0x01) {
        try {
            PinProof proof = computePinProof(pendingSecurityCode_, pinGridSeed_, pinServerSalt_);
            pinClientSalt = proof.clientSalt;
            pinHash = proof.hash;
            pinClientSaltPtr = &pinClientSalt;
            pinHashPtr = &pinHash;
        } catch (const std::exception& e) {
            fail(std::string("PIN required but invalid: ") + e.what());
            return;
        }
    }

    // Legacy client integrity hash (aka "CRC hash"). Some servers enforce this for classic builds.
    // We compute it when checksumSalt was provided (always present on success challenge) and files exist.
    {
        std::vector<std::string> candidateDirs;
        if (const char* env = std::getenv("WOWEE_INTEGRITY_DIR")) {
            if (env && *env) candidateDirs.push_back(env);
        }
        // Default local extraction layout
        candidateDirs.push_back("Data/misc");
        // Common turtle repack location used in this workspace
        if (const char* home = std::getenv("HOME")) {
            if (home && *home) {
                candidateDirs.push_back(std::string(home) + "/Downloads/twmoa_1180");
                candidateDirs.push_back(std::string(home) + "/twmoa_1180");
            }
        }

        const char* candidateExes[] = { "WoW.exe", "TurtleWoW.exe", "Wow.exe" };
        bool ok = false;
        std::string lastErr;
        for (const auto& dir : candidateDirs) {
            for (const char* exe : candidateExes) {
                std::string err;
                if (computeIntegrityHashWin32WithExe(checksumSalt_, A, dir, exe, crcHash, err)) {
                    crcHashPtr = &crcHash;
                    LOG_INFO("Integrity hash computed from ", dir, " (", exe, ")");
                    ok = true;
                    break;
                }
                lastErr = err;
            }
            if (ok) break;
        }
        if (!ok) {
            LOG_WARNING("Integrity hash not computed (", lastErr,
                        "). Server may reject classic clients without it. "
                        "Set WOWEE_INTEGRITY_DIR to your client folder.");
        }
    }

    if (clientInfo.protocolVersion < 8) {
        // Legacy proof format: no securityFlags byte on the wire, but CRC/integrity hash still applies.
        auto packet = LogonProofPacket::buildLegacy(A, M1, crcHashPtr);
        socket->send(packet);
    } else {
        auto packet = LogonProofPacket::build(A, M1, securityFlags_, crcHashPtr, pinClientSaltPtr, pinHashPtr);
        socket->send(packet);

        if (securityFlags_ & 0x04) {
            // TrinityCore-style Google Authenticator token: send immediately after proof.
            const std::string token = pendingSecurityCode_;
            auto tokPkt = AuthenticatorTokenPacket::build(token);
            socket->send(tokPkt);
        }
    }

    setState(AuthState::PROOF_SENT);
}

void AuthHandler::submitPin(const std::string& pin) {
    submitSecurityCode(pin);
}

void AuthHandler::submitSecurityCode(const std::string& code) {
    pendingSecurityCode_ = code;
    // If we're waiting on a security code, continue immediately.
    if (state == AuthState::PIN_REQUIRED || state == AuthState::AUTHENTICATOR_REQUIRED) {
        sendLogonProof();
    }
}

void AuthHandler::handleLogonProofResponse(network::Packet& packet) {
    LOG_DEBUG("Handling LOGON_PROOF response");

    LogonProofResponse response;
    if (!LogonProofResponseParser::parse(packet, response)) {
        fail("Server sent an invalid login response - it may use an incompatible protocol");
        return;
    }

    if (!response.isSuccess()) {
        std::string reason = "Login failed: ";
        reason += getAuthResultString(static_cast<AuthResult>(response.status));
        fail(reason);
        return;
    }

    // Verify server proof
    if (!srp->verifyServerProof(response.M2)) {
        fail("Server identity verification failed - the server may be running an incompatible version");
        return;
    }

    // Authentication successful!
    sessionKey = srp->getSessionKey();
    setState(AuthState::AUTHENTICATED);

    LOG_INFO("========================================");
    LOG_INFO("   AUTHENTICATION SUCCESSFUL!");
    LOG_INFO("========================================");
    LOG_INFO("User: ", username);
    LOG_INFO("Session key size: ", sessionKey.size(), " bytes");

    if (onSuccess) {
        onSuccess(sessionKey);
    }
}

void AuthHandler::sendRealmListRequest() {
    LOG_DEBUG("Sending REALM_LIST request");

    auto packet = RealmListPacket::build();
    socket->send(packet);

    setState(AuthState::REALM_LIST_REQUESTED);
}

void AuthHandler::handleRealmListResponse(network::Packet& packet) {
    LOG_DEBUG("Handling REALM_LIST response");

    RealmListResponse response;
    if (!RealmListResponseParser::parse(packet, response)) {
        LOG_ERROR("Failed to parse REALM_LIST response");
        return;
    }

    realms = response.realms;
    setState(AuthState::REALM_LIST_RECEIVED);

    LOG_INFO("========================================");
    LOG_INFO("   REALM LIST RECEIVED!");
    LOG_INFO("========================================");
    LOG_INFO("Total realms: ", realms.size());

    for (size_t i = 0; i < realms.size(); ++i) {
        const auto& realm = realms[i];
        LOG_INFO("Realm ", (i + 1), ": ", realm.name);
        LOG_INFO("  Address: ", realm.address);
        LOG_INFO("  ID: ", (int)realm.id);
        LOG_INFO("  Population: ", realm.population);
        LOG_INFO("  Characters: ", (int)realm.characters);
        if (realm.hasVersionInfo()) {
            LOG_INFO("  Version: ", (int)realm.majorVersion, ".",
                     (int)realm.minorVersion, ".", (int)realm.patchVersion,
                     " (build ", realm.build, ")");
        }
    }

    if (onRealmList) {
        onRealmList(realms);
    }
}

void AuthHandler::handlePacket(network::Packet& packet) {
    if (packet.getSize() < 1) {
        LOG_WARNING("Received empty packet");
        return;
    }

    // Read opcode
    uint8_t opcodeValue = packet.readUInt8();
    // Note: packet now has read position advanced past opcode

    AuthOpcode opcode = static_cast<AuthOpcode>(opcodeValue);

    // Hex dump first bytes for diagnostics
    {
        const auto& raw = packet.getData();
        std::ostringstream hs;
        for (size_t i = 0; i < std::min<size_t>(raw.size(), 40); ++i)
            hs << std::hex << std::setfill('0') << std::setw(2) << (int)raw[i];
        if (raw.size() > 40) hs << "...";
        LOG_INFO("Auth pkt 0x", std::hex, (int)opcodeValue, std::dec,
                 " (", raw.size(), "B): ", hs.str());
    }

    switch (opcode) {
        case AuthOpcode::LOGON_CHALLENGE:
            if (state == AuthState::CHALLENGE_SENT) {
                handleLogonChallengeResponse(packet);
            } else {
                // Some servers send a short LOGON_CHALLENGE failure packet if auth times out while we wait for 2FA/PIN.
                LogonChallengeResponse response;
                if (LogonChallengeResponseParser::parse(packet, response) && !response.isSuccess()) {
                    std::ostringstream ss;
                    ss << "LOGON_CHALLENGE failed";
                    if (state == AuthState::PIN_REQUIRED) {
                        ss << " while waiting for 2FA/PIN code";
                    }
                    if (response.result == AuthResult::BUILD_INVALID || response.result == AuthResult::BUILD_UPDATE) {
                        ss << ": version mismatch (client v"
                           << (int)clientInfo.majorVersion << "."
                           << (int)clientInfo.minorVersion << "."
                           << (int)clientInfo.patchVersion
                           << " build " << clientInfo.build
                           << ", auth protocol " << (int)clientInfo.protocolVersion << ")";
                    } else {
                        ss << ": " << getAuthResultString(response.result)
                           << " (code 0x" << std::hex << std::setw(2) << std::setfill('0')
                           << static_cast<unsigned>(response.result) << std::dec << ")";
                    }
                    fail(ss.str());
                } else {
                    LOG_WARNING("Unexpected LOGON_CHALLENGE response in state: ", (int)state);
                }
            }
            break;

        case AuthOpcode::LOGON_PROOF:
            if (state == AuthState::PROOF_SENT) {
                handleLogonProofResponse(packet);
            } else {
                LOG_WARNING("Unexpected LOGON_PROOF response in state: ", (int)state);
            }
            break;

        case AuthOpcode::REALM_LIST:
            if (state == AuthState::REALM_LIST_REQUESTED) {
                handleRealmListResponse(packet);
            } else {
                LOG_WARNING("Unexpected REALM_LIST response in state: ", (int)state);
            }
            break;

        default:
            LOG_WARNING("Unhandled auth opcode: 0x", std::hex, (int)opcodeValue, std::dec);
            break;
    }
}

void AuthHandler::update(float /*deltaTime*/) {
    if (!socket) {
        return;
    }

    // Update socket (processes incoming data and calls packet callback)
    socket->update();

    // If the server drops the TCP connection mid-auth, surface it as an auth failure immediately
    // (otherwise the UI just hits its generic timeout).
    if (!socket->isConnected()) {
        if (state != AuthState::DISCONNECTED &&
            state != AuthState::FAILED &&
            state != AuthState::AUTHENTICATED &&
            state != AuthState::REALM_LIST_RECEIVED) {
            fail("Disconnected by auth server");
        }
    }
}

void AuthHandler::setState(AuthState newState) {
    if (state != newState) {
        LOG_DEBUG("Auth state: ", (int)state, " -> ", (int)newState);
        state = newState;
    }
}

void AuthHandler::fail(const std::string& reason) {
    LOG_ERROR("Authentication failed: ", reason);
    setState(AuthState::FAILED);

    if (onFailure) {
        onFailure(reason);
    }
}

} // namespace auth
} // namespace wowee
