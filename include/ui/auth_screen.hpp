#pragma once

#include "auth/auth_handler.hpp"
#include "rendering/video_player.hpp"
#include <string>
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
    // UI state
    char hostname[256] = "127.0.0.1";
    char username[256] = "";
    char password[256] = "";
    int port = 3724;
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
    void saveLoginInfo();
    void loadLoginInfo();
    static std::string getConfigPath();
    bool loginInfoLoaded = false;

    // Background video
    bool videoInitAttempted = false;
    rendering::VideoPlayer backgroundVideo;

    bool musicInitAttempted = false;
    bool musicPlaying = false;
};

}} // namespace wowee::ui
