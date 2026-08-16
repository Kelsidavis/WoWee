#pragma once

#include "ui/settings_panel.hpp"
#include <future>

#include "ui/ui_services.hpp"
#include "auth/auth_handler.hpp"
#include <vulkan/vulkan.h>
#include <string>
#include <vector>
#include <functional>
#include <utility>

namespace wowee { namespace rendering { class VkContext; } }

namespace wowee { namespace ui {

/**
 * Authentication screen UI
 *
 * Allows user to enter credentials and connect to auth server
 */
class AuthScreen {
public:
    AuthScreen();
    /// Releases the background image. It is the only Vulkan resource this
    /// screen owns outright -- bgSampler comes from the context's cache and
    /// bgDescriptorSet belongs to the ImGui backend.
    ~AuthScreen();

    /**
     * Render the UI
     * @param authHandler Reference to auth handler
     */
    void render(auth::AuthHandler& authHandler);

    /**
     * Set callback for successful authentication
     */
    void setOnSuccess(std::function<void()> callback) { onSuccess = std::move(callback); }

    /// Set services (dependency injection)
    void setServices(const UIServices& services) { services_ = services; }


    /**
     * Check if authentication is in progress
     */
    [[nodiscard]] bool isAuthenticating() const { return authenticating; }

    void stopLoginMusic();

    /**
     * Get status message
     */
    [[nodiscard]] const std::string& getStatusMessage() const { return statusMessage; }

    /// Say why the player is looking at this screen. Public because a
    /// disconnect is announced from outside: the world tears itself down on the
    /// way here and there is nowhere else left to say it.
    void setStatus(const std::string& message, bool isError = false,
                   bool prominent = false);

private:
    UIServices services_;  // Injected service references

    struct ServerProfile {
        std::string hostname;
        int port = 3724;
        std::string username;
        std::string passwordHash;  // SHA1 hex (UPPER(user):UPPER(pass))
        std::string expansionId;   // "wotlk", "tbc", "classic", "turtle", ...
        std::string assetProfileId; // empty=match protocol, "legacy"=root manifest
    };

    // UI state
    char hostname[256] = "localhost";
    char username[256] = "";
    char password[256] = "";
    char pinCode[32] = "";
    int port = 3724;
    int expansionIndex = 0;     // Index into expansion registry profiles
    std::string assetProfileId_; // Empty follows expansionIndex automatically
    bool authenticating = false;
    bool showPassword = false;
    bool pinAutoSubmitted_ = false;
    bool securityPromptFocused_ = false;

    // Status
    std::string statusMessage;
    /// Drawn across the screen rather than only in the panel. For the things
    /// that happened to the player rather than to a form they filled in.
    bool statusProminent = false;
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
     * Attempt authentication (starts a fresh attempt, resetting the protocol
     * fallback chain).
     */
    void attemptAuth(auth::AuthHandler& authHandler);

    /**
     * Connect + authenticate using authProtocols_[authProtocolAttempt_].
     * Shared by the initial attempt and each protocol fallback retry.
     */
    void beginAuthAttempt(auth::AuthHandler& authHandler);

    // Auth protocol versions to try, in order. Vanilla-family servers disagree
    // on this byte - vmangos-derived 1.12 realms speak protocol 8 while stock
    // mangos/cmangos 1.12 speak 3 - and the profile can only name one of them,
    // so a mismatch is retried on the next candidate instead of hard-failing.
    // Retries only fire for protocol-shaped failures (see
    // AuthHandler::lastFailureWasProtocol) - never for a rejected password.
    std::vector<uint8_t> authProtocols_;
    size_t authProtocolAttempt_ = 0;

    /**
     * Update status message
     */

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
    [[nodiscard]] std::string currentExpansionId() const;

    // Background image (Vulkan)
    bool bgInitAttempted = false;
    // The background is a 1408x768 PNG in a 2.4MB file; decoding it on the
    // frame it first appears cost ~190ms. Decode on a worker and upload when it
    // arrives - the screen simply renders without it until then.
    struct DecodedBackground {
        std::vector<unsigned char> pixels;
        int width = 0;
        int height = 0;
    };
    std::future<DecodedBackground> bgDecodeFuture;
    bool bgDecodeStarted = false;
    bool uploadBackgroundImage(const unsigned char* pixels);
    rendering::VkContext* bgVkCtx = nullptr;
    VkImage bgImage = VK_NULL_HANDLE;
    VkDeviceMemory bgMemory = VK_NULL_HANDLE;
    VkImageView bgImageView = VK_NULL_HANDLE;
    VkSampler bgSampler = VK_NULL_HANDLE;
    VkDescriptorSet bgDescriptorSet = VK_NULL_HANDLE;
    int bgWidth = 0;
    int bgHeight = 0;

    bool musicInitAttempted = false;
    bool musicPlaying = false;
    bool missingIntroTracksLogged_ = false;
    bool introTracksScanned_ = false;
    std::string loginTrackPath_;
    bool loginMusicVolumeAdjusted_ = false;
    int savedMusicVolume_ = 30;

    // ----- Login-screen graphics settings popup -----
    bool showLoginSettings_ = false;

    // Local copies of the settings keys we expose in the login popup.
    // Loaded on first open; saved on Apply.
    /// The graphics the login screen offers, kept apart from SettingsPanel's
    /// fields on purpose.
    ///
    /// It mirrors eighteen of them, and that mirroring caused four bugs: a
    /// parallax dropdown offering two entries of a three-entry scale, a preset
    /// table that had drifted from the panel's in four of ten columns, four
    /// keys with no clamp and a fifth clamped to a different floor, and a set
    /// of defaults that disagreed with the schema's. Every one of them is fixed
    /// by the two sides reading one thing rather than by this struct going
    /// away: the scale, the preset table, the ranges and the defaults are all
    /// shared now, and settings_apply_on_load fails if any of them drifts
    /// again.
    ///
    /// What is left is eighteen fields of plain state and a loader that clamps
    /// them, which is what this screen needs before a SettingsPanel exists to
    /// hold anything - it runs before the game screen does. Collapsing it means
    /// reworking the first screen every player sees, for a mirror whose every
    /// known failure is already held by a check. That is a trade worth
    /// declining until something here drifts that the checks do not catch.
    struct LoginGraphicsState {
        int  preset          = 2;   // 0=Custom 1=Low 2=Medium 3=High 4=Ultra
        bool shadows         = true;
        float shadowDistance = 300.0f;
        float viewDistance   = kDefaultViewDistance;
        /// How far distance fog takes the sky's colour, 0 to 1. See
        /// LightingManager::setFogSkyBlend.
        float fogSkyBlend    = 0.7f;
        float fogStrength    = 0.4f;
        bool  sharpStars     = true;
        int  antiAliasing    = 1;   // 0=Off 1=2x 2=4x 3=8x
        bool fxaa            = false;
        bool normalMapping   = true;
        bool pom             = true;
        int  pomQuality      = 1;   // 0=Low 1=Medium 2=High
        int  upscalingMode   = 0;   // 0=Off 1=FSR1 2=FSR3
        bool waterRefraction = true;
        int  groundClutter   = kDefaultGroundClutter; // 0-150
        int  brightness      = 50;  // 0-100
        bool vsync           = true;
        bool fullscreen      = false;
    };
    LoginGraphicsState loginGfx_;
    bool loginGfxLoaded_ = false;

    void renderLoginSettingsWindow();
    void loadLoginGraphicsState();
    void saveLoginGraphicsState();
    static void applyPresetToState(LoginGraphicsState& s, int preset);
};

}} // namespace wowee::ui
