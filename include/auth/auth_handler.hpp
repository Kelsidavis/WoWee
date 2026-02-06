#pragma once

#include "auth/srp.hpp"
#include "auth/auth_packets.hpp"
#include <memory>
#include <string>
#include <functional>

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
    bool connect(const std::string& host, uint16_t port = 3724);
    void disconnect();
    bool isConnected() const;

    // Authentication
    void authenticate(const std::string& username, const std::string& password);
    void authenticateWithHash(const std::string& username, const std::vector<uint8_t>& authHash);

    // Realm list
    void requestRealmList();
    const std::vector<Realm>& getRealms() const { return realms; }

    // State
    AuthState getState() const { return state; }
    const std::vector<uint8_t>& getSessionKey() const { return sessionKey; }
    const std::string& getUsername() const { return username; }

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
    void fail(const std::string& reason);

    std::unique_ptr<network::TCPSocket> socket;
    std::unique_ptr<SRP> srp;

    AuthState state = AuthState::DISCONNECTED;
    std::string username;
    std::string password;
    ClientInfo clientInfo;

    std::vector<uint8_t> sessionKey;
    std::vector<Realm> realms;

    // Callbacks
    AuthSuccessCallback onSuccess;
    AuthFailureCallback onFailure;
    RealmListCallback onRealmList;

    // Receive buffer
    std::vector<uint8_t> receiveBuffer;
};

} // namespace auth
} // namespace wowee
