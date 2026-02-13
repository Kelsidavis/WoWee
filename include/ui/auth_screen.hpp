#pragma once

#include "auth/auth_handler.hpp"
#include "rendering/video_player.hpp"
#include <string>
#include <vector>
#include <functional>

namespace wowee { namespace ui {

/**
 * Authentication screen UI
 *
 * Allows user to enter credentials and connect to auth server
 */
class AuthScreen {
public:
    AuthScreen();

    /**
     * Render the UI
     * @param authHandler Reference to auth handler
     */
    void render(auth::AuthHandler& authHandler);

    /**
     * Set callback for successful authentication
     */
    void setOnSuccess(std::function<void()> callback) { onSuccess = callback; }


    /**
     * Check if authentication is in progress
     */
    bool isAuthenticating() const { return authenticating; }

    void stopLoginMusic();

    /**
     * Get status message
     */
    const std::string& getStatusMessage() const { return statusMessage; }

private:
    struct ServerProfile {
        std::string hostname;
        int port = 3724;
        std::string username;
        std::string passwordHash;  // SHA1 hex (UPPER(user):UPPER(pass))
        std::string expansionId;   // "wotlk", "tbc", "classic", "turtle", ...
    };

    // UI state
    char hostname[256] = "127.0.0.1";
    char username[256] = "";
    char password[256] = "";
    char pinCode[32] = "";
    int port = 3724;
    int expansionIndex = 0;     // Index into expansion registry profiles
    bool authenticating = false;
    bool showPassword = false;

    // Status
    std::string statusMessage;
    bool statusIsError = false;
    std::string failureReason;    // Specific reason from auth handler
    float authTimer = 0.0f;       // Timeout tracker
    static constexpr float AUTH_TIMEOUT = 10.0f;

    // Saved password hash (SHA1(UPPER(user):UPPER(pass)) as hex)
    std::string savedPasswordHash;
    bool usingStoredHash = false;
    static constexpr const char* PASSWORD_PLACEHOLDER = "\x01\x01\x01\x01\x01\x01\x01\x01";

    // Saved server-specific profiles
    std::vector<ServerProfile> servers_;
    int selectedServerIndex_ = -1;  // -1 = custom/unlisted

    // Callbacks
    std::function<void()> onSuccess;

    /**
     * Attempt authentication
     */
    void attemptAuth(auth::AuthHandler& authHandler);

    /**
     * Update status message
     */
    void setStatus(const std::string& message, bool isError = false);

    /**
     * Persist/restore login fields
     */
    void saveLoginInfo(bool includePasswordHash);
    void loadLoginInfo();
    static std::string getConfigPath();
    bool loginInfoLoaded = false;

    static std::string makeServerKey(const std::string& host, int port);
    void selectServerProfile(int index);
    void upsertCurrentServerProfile(bool includePasswordHash);
    std::string currentExpansionId() const;

    // Background video
    bool videoInitAttempted = false;
    rendering::VideoPlayer backgroundVideo;

    bool musicInitAttempted = false;
    bool musicPlaying = false;
};

}} // namespace wowee::ui
