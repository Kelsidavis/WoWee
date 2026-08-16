#pragma once

#include "auth/srp.hpp"
#include "auth/auth_packets.hpp"
#include <memory>
#include <string>
#include <functional>
#include <array>

namespace wowee {
namespace network { class TCPSocket; class Packet; }

namespace auth {

struct Realm;

// Authentication state
enum class AuthState {
    DISCONNECTED,
    CONNECTED,
    CHALLENGE_SENT,
    CHALLENGE_RECEIVED,
    PIN_REQUIRED,
    AUTHENTICATOR_REQUIRED,
    PROOF_SENT,
    AUTHENTICATED,
    REALM_LIST_REQUESTED,
    REALM_LIST_RECEIVED,
    FAILED
};

// Authentication callbacks
using AuthSuccessCallback = std::function<void(const std::vector<uint8_t>& sessionKey)>;
using AuthFailureCallback = std::function<void(const std::string& reason)>;
using RealmListCallback = std::function<void(const std::vector<Realm>& realms)>;

class AuthHandler {
public:
    AuthHandler();
    ~AuthHandler();

    // Connection
    [[nodiscard]] bool connect(const std::string& host, uint16_t port = 3724);
    void disconnect();
    [[nodiscard]] bool isConnected() const;

    // Authentication
    void authenticate(const std::string& username, const std::string& password);
    void authenticate(const std::string& username, const std::string& password, const std::string& pin);
    void authenticateWithHash(const std::string& username, const std::vector<uint8_t>& authHash);
    void authenticateWithHash(const std::string& username, const std::vector<uint8_t>& authHash, const std::string& pin);
    // Generic continuation for PIN / authenticator-required servers.
    void submitSecurityCode(const std::string& code);

    // Set client version info (call before authenticate)
    void setClientInfo(const ClientInfo& info) { clientInfo = info; }
    [[nodiscard]] const ClientInfo& getClientInfo() const { return clientInfo; }

    // Realm list
    void requestRealmList();
    [[nodiscard]] const std::vector<Realm>& getRealms() const { return realms; }

    // State
    [[nodiscard]] AuthState getState() const { return state; }
    [[nodiscard]] const std::vector<uint8_t>& getSessionKey() const { return sessionKey; }
    [[nodiscard]] const std::string& getUsername() const { return username; }

    /** True when the last failure is consistent with an auth-protocol mismatch
     *  (rather than bad credentials), so the caller may retry on another
     *  protocol version. See protocolFailureSuspected_. */
    [[nodiscard]] bool lastFailureWasProtocol() const { return protocolFailureSuspected_; }

    // Callbacks
    void setOnSuccess(AuthSuccessCallback callback) { onSuccess = callback; }
    void setOnFailure(AuthFailureCallback callback) { onFailure = callback; }
    void setOnRealmList(RealmListCallback callback) { onRealmList = callback; }

    // Update (call each frame)
    void update(float deltaTime);

private:
    void sendLogonChallenge();
    void handleLogonChallengeResponse(network::Packet& packet);
    void sendLogonProof();
    void handleLogonProofResponse(network::Packet& packet);
    void sendRealmListRequest();
    void handleRealmListResponse(network::Packet& packet);
    void handlePacket(network::Packet& packet);

    void setState(AuthState newState);
    /** protocolRelated: the failure is consistent with the server speaking a
     *  different auth protocol than clientInfo.protocolVersion (bad handshake
     *  parse, build/version rejection, mid-handshake drop) rather than the
     *  credentials being wrong. Drives the caller's protocol fallback. */
    void fail(const std::string& reason, bool protocolRelated = false);

    std::unique_ptr<network::TCPSocket> socket;
    std::unique_ptr<SRP> srp;

    AuthState state = AuthState::DISCONNECTED;
    std::string username;
    std::string password;
    ClientInfo clientInfo;

    // True when the last failure looks like an auth-protocol mismatch rather
    // than a credential/account problem. Never set for wrong-password, banned,
    // suspended, or account-in-use results - retrying those risks lockouts.
    bool protocolFailureSuspected_ = false;

    std::vector<uint8_t> sessionKey;
    std::vector<Realm> realms;

    // Callbacks
    AuthSuccessCallback onSuccess;
    AuthFailureCallback onFailure;
    RealmListCallback onRealmList;

    // Receive buffer
    std::vector<uint8_t> receiveBuffer;

    // Challenge security extension (PIN)
    uint8_t securityFlags_ = 0;
    uint32_t pinGridSeed_ = 0;
    std::array<uint8_t, 16> pinServerSalt_{}; // from LOGON_CHALLENGE response
    std::array<uint8_t, 16> checksumSalt_{};  // from LOGON_CHALLENGE response (integrity salt)
    std::string pendingSecurityCode_;
};

} // namespace auth
} // namespace wowee
