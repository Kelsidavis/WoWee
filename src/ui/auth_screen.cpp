#include "ui/auth_screen.hpp"
#include <future>
#include <chrono>
#include "rendering/pom_quality.hpp"
#include "ui/graphics_presets.hpp"
#include "ui/ui_colors.hpp"
#include "ui/settings_panel.hpp"
#include "auth/crypto.hpp"
#include "core/application.hpp"
#include "core/config_paths.hpp"
#include "core/logger.hpp"
#include "core/version.hpp"
#include "core/window.hpp"
#include "rendering/renderer.hpp"
#include "rendering/vk_context.hpp"
#include "pipeline/asset_manager.hpp"
#include "audio/audio_coordinator.hpp"
#include "audio/music_manager.hpp"
#include "game/expansion_profile.hpp"
#include <imgui.h>
#include <imgui_impl_vulkan.h>
#include "stb_image.h"
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <iomanip>
#include <unordered_map>

namespace wowee { namespace ui {

static std::string trimAscii(std::string s) {
    auto isSpace = [](unsigned char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; };
    size_t b = 0;
    while (b < s.size() && isSpace(static_cast<unsigned char>(s[b]))) ++b;
    size_t e = s.size();
    while (e > b && isSpace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

static std::string hexEncode(const std::vector<uint8_t>& data) {
    std::ostringstream ss;
    for (uint8_t b : data)
        ss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(b);
    return ss.str();
}

static std::vector<uint8_t> hexDecode(const std::string& hex) {
    std::vector<uint8_t> bytes;
    for (size_t i = 0; i + 1 < hex.size(); i += 2) {
        try {
            uint8_t b = static_cast<uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16));
            bytes.push_back(b);
        } catch (...) {
            return {};
        }
    }
    return bytes;
}

AuthScreen::AuthScreen() {
}

AuthScreen::~AuthScreen() {
    // The background image was uploaded with plain vkCreateImage and
    // vkAllocateMemory and nothing released it, so it and its view outlived
    // the device on every run. bgSampler is the context's, cached and shared,
    // and bgDescriptorSet belongs to the ImGui backend, which frees its own
    // pool -- neither is this class's to destroy.
    if (!bgVkCtx) return;
    VkDevice device = bgVkCtx->getDevice();
    if (device == VK_NULL_HANDLE) return;
    if (bgImageView) { vkDestroyImageView(device, bgImageView, nullptr); bgImageView = VK_NULL_HANDLE; }
    if (bgImage)     { vkDestroyImage(device, bgImage, nullptr);         bgImage = VK_NULL_HANDLE; }
    if (bgMemory)    { vkFreeMemory(device, bgMemory, nullptr);          bgMemory = VK_NULL_HANDLE; }
}

std::string AuthScreen::makeServerKey(const std::string& host, int port) {
    std::ostringstream ss;
    ss << host << ":" << port;
    return ss.str();
}

std::string AuthScreen::currentExpansionId() const {
    auto* reg = core::Application::getInstance().getExpansionRegistry();
    if (reg && reg->getActive()) {
        return reg->getActive()->id;
    }
    return "wotlk";
}

void AuthScreen::selectServerProfile(int index) {
    if (index < 0 || index >= static_cast<int>(servers_.size())) {
        selectedServerIndex_ = -1;
        return;
    }

    selectedServerIndex_ = index;
    const auto& s = servers_[index];

    std::snprintf(hostname, sizeof(hostname), "%s", s.hostname.c_str());
    hostname[sizeof(hostname) - 1] = '\0';
    port = s.port;

    std::snprintf(username, sizeof(username), "%s", s.username.c_str());
    username[sizeof(username) - 1] = '\0';

    savedPasswordHash = s.passwordHash;
    usingStoredHash = !savedPasswordHash.empty();
    if (usingStoredHash) {
        std::snprintf(password, sizeof(password), "%s", PASSWORD_PLACEHOLDER);
        password[sizeof(password) - 1] = '\0';
    } else {
        password[0] = '\0';
    }

    auto& app = core::Application::getInstance();
    if (!s.expansionId.empty()) {
        auto* expReg = app.getExpansionRegistry();
        if (expReg && expReg->setActive(s.expansionId)) {
            auto& profiles = expReg->getAllProfiles();
            for (int i = 0; i < static_cast<int>(profiles.size()); ++i) {
                if (profiles[i].id == s.expansionId) { expansionIndex = i; break; }
            }
        }
    }
    assetProfileId_ = s.assetProfileId;
    if (!app.setAssetExpansionOverride(assetProfileId_)) {
        assetProfileId_.clear();
        app.setAssetExpansionOverride({});
    }
    app.reloadExpansionData();
}

void AuthScreen::upsertCurrentServerProfile(bool includePasswordHash) {
    const std::string hostStr = hostname;
    if (hostStr.empty() || port <= 0) {
        return;
    }

    const std::string key = makeServerKey(hostStr, port);
    int foundIndex = -1;
    for (int i = 0; i < static_cast<int>(servers_.size()); ++i) {
        if (makeServerKey(servers_[i].hostname, servers_[i].port) == key) {
            foundIndex = i;
            break;
        }
    }

    ServerProfile s;
    s.hostname = hostStr;
    s.port = port;
    s.username = username;
    s.expansionId = currentExpansionId();
    s.assetProfileId = assetProfileId_;
    if (includePasswordHash && !savedPasswordHash.empty()) {
        s.passwordHash = savedPasswordHash;
    } else if (foundIndex >= 0) {
        // Preserve existing stored hash if we aren't updating it.
        s.passwordHash = servers_[foundIndex].passwordHash;
    }

    if (foundIndex >= 0) {
        servers_[foundIndex] = std::move(s);
        selectedServerIndex_ = foundIndex;
    } else {
        servers_.push_back(std::move(s));
        selectedServerIndex_ = static_cast<int>(servers_.size()) - 1;
    }

    // Keep deterministic ordering (and stable combo ordering) across runs.
    std::sort(servers_.begin(), servers_.end(),
              [](const ServerProfile& a, const ServerProfile& b) {
                  if (a.hostname != b.hostname) return a.hostname < b.hostname;
                  return a.port < b.port;
              });

    // Fix up index after sort.
    for (int i = 0; i < static_cast<int>(servers_.size()); ++i) {
        if (makeServerKey(servers_[i].hostname, servers_[i].port) == key) {
            selectedServerIndex_ = i;
            break;
        }
    }
}

void AuthScreen::render(auth::AuthHandler& authHandler) {
    // Load saved login info on first render
    if (!loginInfoLoaded) {
        loadLoginInfo();
        loginInfoLoaded = true;
        auto* registry = core::Application::getInstance().getExpansionRegistry();
        if (registry && registry->getActive()) {
            const auto& profiles = registry->getAllProfiles();
            for (int i = 0; i < static_cast<int>(profiles.size()); ++i) {
                if (profiles[i].id == registry->getActive()->id) {
                    expansionIndex = i;
                    break;
                }
            }
        }
    }

    // Kick the decode off once, then upload on whichever frame it lands.
    if (!bgDecodeStarted) {
        bgDecodeStarted = true;
        std::string imgPath = "assets/krayonsignin.png";
        if (!std::filesystem::exists(imgPath))
            imgPath = (std::filesystem::current_path() / imgPath).string();
        bgDecodeFuture = std::async(std::launch::async, [imgPath]() {
            DecodedBackground out;
            int channels = 0;
            stbi_set_flip_vertically_on_load(false);
            unsigned char* data = stbi_load(imgPath.c_str(), &out.width, &out.height, &channels, 4);
            if (data) {
                out.pixels.assign(data, data + static_cast<size_t>(out.width) * out.height * 4);
                        }
            return out;
        });
    }
    if (!bgInitAttempted && bgDecodeFuture.valid() &&
        bgDecodeFuture.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        bgInitAttempted = true;
        DecodedBackground decoded = bgDecodeFuture.get();
        if (!decoded.pixels.empty()) {
            bgWidth = decoded.width;
            bgHeight = decoded.height;
            uploadBackgroundImage(decoded.pixels.data());
        } else {
            LOG_WARNING("Auth screen: failed to decode background image");
        }
    }
    if (bgDescriptorSet) {
        ImVec2 screen = ImGui::GetIO().DisplaySize;
        float screenW = screen.x;
        float screenH = screen.y;
        float imgW = static_cast<float>(bgWidth);
        float imgH = static_cast<float>(bgHeight);
        if (imgW > 0.0f && imgH > 0.0f) {
            float screenAspect = screenW / screenH;
            float imgAspect = imgW / imgH;
            ImVec2 uv0(0.0f, 0.0f);
            ImVec2 uv1(1.0f, 1.0f);
            if (imgAspect > screenAspect) {
                float scale = screenAspect / imgAspect;
                float crop = (1.0f - scale) * 0.5f;
                uv0.x = crop;
                uv1.x = 1.0f - crop;
            } else if (imgAspect < screenAspect) {
                float scale = imgAspect / screenAspect;
                float crop = (1.0f - scale) * 0.5f;
                uv0.y = crop;
                uv1.y = 1.0f - crop;
            }
            ImDrawList* bg = ImGui::GetBackgroundDrawList();
            bg->AddImage(reinterpret_cast<ImTextureID>(bgDescriptorSet),
                         ImVec2(0, 0), ImVec2(screenW, screenH), uv0, uv1);
        }
    }

    // Build version, bottom-left over the login art (as the retail client does).
    {
        ImVec2 screen = ImGui::GetIO().DisplaySize;
        const char* version = core::kVersionString;
        ImVec2 textSize = ImGui::CalcTextSize(version);
        ImVec2 pos(12.0f, screen.y - textSize.y - 10.0f);
        ImDrawList* fg = ImGui::GetForegroundDrawList();
        fg->AddText(ImVec2(pos.x + 1.0f, pos.y + 1.0f), IM_COL32(0, 0, 0, 160), version);
        fg->AddText(pos, IM_COL32(200, 200, 200, 180), version);
    }

    auto& app = core::Application::getInstance();
    auto* ac = app.getAudioCoordinator();
    if (!musicInitAttempted) {
        musicInitAttempted = true;
        auto* assets = app.getAssetManager();
        if (ac) {
            auto* music = ac->getMusicManager();
            if (music && assets && assets->isInitialized() && !music->isInitialized()) {
                music->initialize(assets);
            }
        }
    }
    // Login screen music
    if (ac) {
        auto* music = ac->getMusicManager();
        if (music) {
            if (!loginMusicVolumeAdjusted_) {
                savedMusicVolume_ = music->getVolume();
                // Reduce music to 80% during login so UI button clicks and error sounds
                // remain audible over the background track
                int loginVolume = (savedMusicVolume_ * 80) / 100;
                if (loginVolume < 0) loginVolume = 0;
                if (loginVolume > 100) loginVolume = 100;
                music->setVolume(loginVolume);
                loginMusicVolumeAdjusted_ = true;
            }
            music->update(ImGui::GetIO().DeltaTime);
            if (!music->isPlaying() && !music->isLoading()) {
                if (!introTracksScanned_) {
                    introTracksScanned_ = true;
                    const std::filesystem::path relative =
                        std::filesystem::path("assets") / "Original Music" /
                        "TavernAllianceREMIX.mp3";
                    if (std::filesystem::exists(relative)) {
                        loginTrackPath_ = relative.string();
                    } else {
                        const auto absolute = std::filesystem::current_path() / relative;
                        if (std::filesystem::exists(absolute)) {
                            loginTrackPath_ = absolute.string();
                        }
                    }
                }

                if (!loginTrackPath_.empty()) {
                    music->playFilePath(loginTrackPath_, true, 1800.0f);
                    // The read runs on a worker, so the track is not playing yet.
                    // Treat a load in flight as success or the track gets dropped below.
                    musicPlaying = music->isPlaying() || music->isLoading();
                    if (musicPlaying) {
                        LOG_INFO("AuthScreen: Playing login intro track: ", loginTrackPath_);
                    } else {
                        // Avoid retrying a bad file every frame. There is deliberately
                        // no fallback: the login screen is constrained to this track.
                        loginTrackPath_.clear();
                    }
                } else if (!missingIntroTracksLogged_) {
                    LOG_WARNING("AuthScreen: Login track not found: assets/Original Music/TavernAllianceREMIX.mp3");
                    missingIntroTracksLogged_ = true;
                }
            }
        }
    }

    ImGui::SetNextWindowSize(ImVec2(500, 400), ImGuiCond_FirstUseEver);
    ImGui::Begin("Authentication", nullptr, ImGuiWindowFlags_NoCollapse);

    ImGui::Text("Connect to Server");
    ImGui::Separator();
    ImGui::Spacing();

    // Server settings
    ImGui::Text("Server Settings");
    {
        std::string preview;
        if (selectedServerIndex_ >= 0 && selectedServerIndex_ < static_cast<int>(servers_.size())) {
            preview = makeServerKey(servers_[selectedServerIndex_].hostname, servers_[selectedServerIndex_].port);
        } else {
            preview = makeServerKey(hostname, port) + " (custom)";
        }

        if (ImGui::BeginCombo("Server", preview.c_str())) {
            bool customSelected = (selectedServerIndex_ < 0);
            if (ImGui::Selectable("Custom...", customSelected)) {
                selectedServerIndex_ = -1;
            }
            if (customSelected) ImGui::SetItemDefaultFocus();

            ImGui::Separator();
            for (int i = 0; i < static_cast<int>(servers_.size()); ++i) {
                std::string label = makeServerKey(servers_[i].hostname, servers_[i].port);
                if (!servers_[i].username.empty()) {
                    label += "  (" + servers_[i].username + ")";
                }
                bool selected = (selectedServerIndex_ == i);
                if (ImGui::Selectable(label.c_str(), selected)) {
                    selectServerProfile(i);
                }
                if (selected) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
    }

    bool hostChanged = ImGui::InputText("Hostname", hostname, sizeof(hostname));
    bool portChanged = ImGui::InputInt("Port", &port);
    if (hostChanged || portChanged) {
        selectedServerIndex_ = -1;
    }
    if (port < 1) port = 1;
    if (port > 65535) port = 65535;

    // Expansion selector (populated from ExpansionRegistry)
    auto* registry = core::Application::getInstance().getExpansionRegistry();
    if (registry && !registry->getAllProfiles().empty()) {
        auto& profiles = registry->getAllProfiles();
        // Build combo items: "WotLK (3.3.5a)"
        std::string preview;
        if (expansionIndex >= 0 && expansionIndex < static_cast<int>(profiles.size())) {
            preview = profiles[expansionIndex].shortName + " (" + profiles[expansionIndex].versionString() + ")";
        }
        if (ImGui::BeginCombo("Expansion", preview.c_str())) {
            for (int i = 0; i < static_cast<int>(profiles.size()); ++i) {
                std::string label = profiles[i].shortName + " (" + profiles[i].versionString() + ")";
                bool selected = (expansionIndex == i);
                if (ImGui::Selectable(label.c_str(), selected)) {
                    expansionIndex = i;
                    registry->setActive(profiles[i].id);
                    core::Application::getInstance().reloadExpansionData();
                }
                if (selected) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        const auto* protocolProfile = registry->getActive();
        std::string assetPreview = protocolProfile
            ? "Match protocol (" + protocolProfile->shortName + ")"
            : "Match protocol";
        if (assetProfileId_ == "legacy") {
            assetPreview = "Legacy root Data";
        } else if (!assetProfileId_.empty()) {
            if (const auto* assetProfile = registry->getProfile(assetProfileId_)) {
                assetPreview = assetProfile->shortName + " assets";
            }
        }

        if (ImGui::BeginCombo("Assets", assetPreview.c_str())) {
            const bool matching = assetProfileId_.empty();
            if (ImGui::Selectable("Match protocol", matching)) {
                assetProfileId_.clear();
                core::Application::getInstance().setAssetExpansionOverride({});
                core::Application::getInstance().reloadExpansionData();
            }
            if (matching) ImGui::SetItemDefaultFocus();

            ImGui::Separator();
            for (const auto& candidate : profiles) {
                if (!std::filesystem::exists(candidate.dataPath + "/manifest.json")) continue;
                const std::string label = candidate.shortName + " assets";
                const bool selected = assetProfileId_ == candidate.id;
                if (ImGui::Selectable(label.c_str(), selected)) {
                    assetProfileId_ = candidate.id;
                    core::Application::getInstance().setAssetExpansionOverride(assetProfileId_);
                    core::Application::getInstance().reloadExpansionData();
                }
                if (selected) ImGui::SetItemDefaultFocus();
            }

            const char* dataPathEnv = std::getenv("WOW_DATA_PATH");
            const std::filesystem::path baseData = dataPathEnv ? dataPathEnv : "./Data";
            if (std::filesystem::exists(baseData / "manifest.json")) {
                const bool selected = assetProfileId_ == "legacy";
                if (ImGui::Selectable("Legacy root Data", selected)) {
                    assetProfileId_ = "legacy";
                    core::Application::getInstance().setAssetExpansionOverride("legacy");
                    core::Application::getInstance().reloadExpansionData();
                }
                if (selected) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        if (!assetProfileId_.empty()) {
            ImGui::TextDisabled("Override active; cross-expansion DBC/model formats may differ.");
        }
    } else {
        ImGui::Text("Expansion: WotLK 3.3.5a (default)");
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Credentials
    ImGui::Text("Credentials");
    ImGui::InputText("Username", username, sizeof(username));

    // Password with visibility toggle
    ImGuiInputTextFlags passwordFlags = showPassword ? 0 : ImGuiInputTextFlags_Password;
    ImGui::InputText("Password", password, sizeof(password), passwordFlags);
    ImGui::SameLine();
    if (ImGui::Checkbox("Show", &showPassword)) {
        // Checkbox state changed
    }

    const auto authState = authHandler.getState();
    const bool waitingForSecurityCode = (authState == auth::AuthState::PIN_REQUIRED ||
                                         authState == auth::AuthState::AUTHENTICATOR_REQUIRED);

    // Optional 2FA / PIN field (some servers require this; e.g. Turtle WoW uses Google Authenticator).
    // Keep it visible pre-connect so we can send LOGON_PROOF immediately after the SRP challenge.
    {
        ImGuiInputTextFlags pinFlags = ImGuiInputTextFlags_Password;
        if (waitingForSecurityCode) {
            pinFlags |= ImGuiInputTextFlags_EnterReturnsTrue;
            if (!securityPromptFocused_) {
                ImGui::SetKeyboardFocusHere();
                securityPromptFocused_ = true;
            }
        }
        const bool submitFromField = ImGui::InputText("2FA / PIN", pinCode, sizeof(pinCode), pinFlags);
        if (waitingForSecurityCode && submitFromField) {
            const std::string code = trimAscii(pinCode);
            if (!code.empty()) {
                authHandler.submitSecurityCode(code);
                pinCode[0] = '\0';
                pinAutoSubmitted_ = true;
            }
        }
        ImGui::SameLine();
        ImGui::TextDisabled("(optional)");
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Across the middle of the screen, at three times the panel's size.
    //
    // A disconnect is not a form being wrong - the player did not ask to be
    // here and may not have been looking - so it is said where it cannot be
    // missed, rather than as another red line inside the login box. It is
    // cleared like any other status, by typing or by connecting again.
    if (statusProminent && !statusMessage.empty()) {
        ImDrawList* fg = ImGui::GetForegroundDrawList();
        const ImVec2 screen = ImGui::GetIO().DisplaySize;
        constexpr float kScale = 3.0f;
        const float fontSize = ImGui::GetFontSize() * kScale;
        const ImVec2 textSize = ImGui::GetFont()->CalcTextSizeA(
            fontSize, FLT_MAX, 0.0f, statusMessage.c_str());
        const ImVec2 at((screen.x - textSize.x) * 0.5f, screen.y * 0.14f);
        const ImVec2 pad(24.0f, 14.0f);
        fg->AddRectFilled(ImVec2(at.x - pad.x, at.y - pad.y),
                          ImVec2(at.x + textSize.x + pad.x, at.y + textSize.y + pad.y),
                          IM_COL32(0, 0, 0, 190), 6.0f);
        // The shadow first, because at this size the text sits over whatever
        // the login screen is drawing behind it.
        fg->AddText(ImGui::GetFont(), fontSize, ImVec2(at.x + 2.0f, at.y + 2.0f),
                    IM_COL32(0, 0, 0, 220), statusMessage.c_str());
        fg->AddText(ImGui::GetFont(), fontSize, at,
                    IM_COL32(255, 90, 90, 255), statusMessage.c_str());
    }

    // Connection status
    if (!statusMessage.empty()) {
        if (statusIsError) {
            ImGui::PushStyleColor(ImGuiCol_Text, ui::colors::kRed);
        } else {
            ImGui::PushStyleColor(ImGuiCol_Text, ui::colors::kBrightGreen);
        }
        ImGui::TextWrapped("%s", statusMessage.c_str());
        ImGui::PopStyleColor();
        ImGui::Spacing();
    }

    // Connect button
    if (authenticating) {
        auto state = authHandler.getState();
        if (state != auth::AuthState::PIN_REQUIRED && state != auth::AuthState::AUTHENTICATOR_REQUIRED) {
            pinAutoSubmitted_ = false;
            securityPromptFocused_ = false;
            authTimer += ImGui::GetIO().DeltaTime;

            // Show progress with elapsed time
            char progressBuf[128];
            snprintf(progressBuf, sizeof(progressBuf), "Authenticating... (%.0fs)", authTimer);
            ImGui::Text("%s", progressBuf);
        } else {
            ImGui::TextWrapped("This server requires a 2FA / PIN code. Enter it and submit quickly (the server may time out).");

            // If the user already typed a code before clicking Connect, submit immediately once.
            if (!pinAutoSubmitted_) {
                bool digitsOnly = true;
                size_t len = std::strlen(pinCode);
                for (size_t i = 0; i < len; ++i) {
                    if (pinCode[i] < '0' || pinCode[i] > '9') { digitsOnly = false; break; }
                }
                // Auto-submit if the user prefilled a plausible code.
                // PIN-grid: 4-10 digits. Authenticator (TOTP): typically 6 digits.
                if (digitsOnly && ((len >= 4 && len <= 10) || len == 6)) {
                    authHandler.submitSecurityCode(pinCode);
                    pinCode[0] = '\0';
                    pinAutoSubmitted_ = true;
                }
            }

            ImGui::SameLine();
            if (ImGui::Button("Submit 2FA/PIN")) {
                const std::string code = trimAscii(pinCode);
                if (!code.empty()) {
                    authHandler.submitSecurityCode(code);
                    // Don't keep the code around longer than needed.
                    pinCode[0] = '\0';
                    pinAutoSubmitted_ = true;
                }
            }
        }

        // Check authentication status
        if (state == auth::AuthState::AUTHENTICATED) {
            setStatus("Authentication successful!", false);
            authenticating = false;

            // Compute and save password hash if user typed a fresh password
            if (!usingStoredHash) {
                std::string upperUser = username;
                std::string upperPass = password;
                auto toUp = [](unsigned char c) { return static_cast<char>(std::toupper(c)); };
                std::transform(upperUser.begin(), upperUser.end(), upperUser.begin(), toUp);
                std::transform(upperPass.begin(), upperPass.end(), upperPass.begin(), toUp);
                std::string combined = upperUser + ":" + upperPass;
                auto hash = auth::Crypto::sha1(combined);
                savedPasswordHash = hexEncode(hash);
            }
            saveLoginInfo(true);

            // Call success callback
            if (onSuccess) {
                onSuccess();
            }
        } else if (state == auth::AuthState::FAILED) {
            // Protocol fallback: a 1.12 realm that speaks the other vanilla auth
            // protocol byte rejects or drops our handshake rather than replying
            // usefully. Retry once on the next candidate before giving up - but
            // only for protocol-shaped failures, never for a rejected password
            // (retrying those can trip server-side lockouts).
            const bool haveFallback = (authProtocolAttempt_ + 1 < authProtocols_.size());
            if (haveFallback && authHandler.lastFailureWasProtocol()) {
                LOG_INFO("Auth failed on protocol ",
                         static_cast<int>(authProtocols_[authProtocolAttempt_]),
                         " (", failureReason, ") - retrying with protocol ",
                         static_cast<int>(authProtocols_[authProtocolAttempt_ + 1]));
                ++authProtocolAttempt_;
                authHandler.disconnect();
                beginAuthAttempt(authHandler);
            } else {
                if (!failureReason.empty()) {
                    setStatus(failureReason, true);
                } else {
                    setStatus("Authentication failed", true);
                }
                authenticating = false;
            }
        } else if (state != auth::AuthState::PIN_REQUIRED && state != auth::AuthState::AUTHENTICATOR_REQUIRED
                   && authTimer >= AUTH_TIMEOUT) {
            setStatus("Connection timed out - server did not respond", true);
            authenticating = false;
            authHandler.disconnect();
        }
    } else {
        if (ImGui::Button("Connect", ImVec2(160, 40))) {
            attemptAuth(authHandler);
        }

        ImGui::SameLine();
        if (ImGui::Button("Clear", ImVec2(160, 40))) {
            statusMessage.clear();
            statusProminent = false;
        }

        ImGui::SameLine();
        if (ImGui::Button("Settings", ImVec2(160, 40))) {
            showLoginSettings_ = true;
        }

        // The only way out of the login screen was the window's own close
        // button: Escape here clears the field it is in rather than leaving,
        // and there is no game menu until a character is in the world.
        ImGui::SameLine();
        if (ImGui::Button("Quit", ImVec2(160, 40))) {
            if (auto* window = core::Application::getInstance().getWindow()) {
                window->setShouldClose(true);
            }
        }
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Info text
    ImGui::TextWrapped("Enter your account credentials to connect to the authentication server.");
    ImGui::TextWrapped("Default port is 3724.");

    ImGui::End();

    renderLoginSettingsWindow();
}

void AuthScreen::stopLoginMusic() {
    auto& app = core::Application::getInstance();
    auto* ac = app.getAudioCoordinator();
    if (!ac) return;
    auto* music = ac->getMusicManager();
    if (!music) return;
    if (musicPlaying) {
        music->stopMusic(500.0f);
        musicPlaying = false;
    }
    if (loginMusicVolumeAdjusted_) {
        music->setVolume(savedMusicVolume_);
        loginMusicVolumeAdjusted_ = false;
    }
}

void AuthScreen::attemptAuth(auth::AuthHandler& authHandler) {
    // Validate inputs
    if (strlen(username) == 0) {
        setStatus("Username cannot be empty", true);
        return;
    }

    // Check if using stored hash (password field contains placeholder)
    bool useHash = usingStoredHash && std::strcmp(password, PASSWORD_PLACEHOLDER) == 0;

    if (!useHash && strlen(password) == 0) {
        setStatus("Password cannot be empty", true);
        return;
    }

    if (strlen(hostname) == 0) {
        setStatus("Hostname cannot be empty", true);
        return;
    }

    // Build the auth-protocol candidate chain for this attempt. The profile's
    // own value is always tried first; vanilla-family profiles get the other
    // vanilla protocol byte appended as a fallback, because vanilla-family
    // servers disagree on it (Turtle/vmangos-derived: 8, older mangos/cmangos:
    // 3) and the profile can only name one. TBC/WotLK have no such ambiguity.
    authProtocols_.clear();
    authProtocolAttempt_ = 0;
    {
        uint8_t primary = 8;
        bool vanillaFamily = false;
        auto* reg = core::Application::getInstance().getExpansionRegistry();
        if (reg) {
            if (auto* profile = reg->getActive()) {
                primary = profile->protocolVersion;
                vanillaFamily = (profile->id == "classic" || profile->id == "turtle");
            }
        }
        authProtocols_.push_back(primary);
        if (vanillaFamily) {
            const uint8_t alternate = (primary == 3) ? uint8_t{8} : uint8_t{3};
            if (alternate != primary) authProtocols_.push_back(alternate);
        }
    }

    beginAuthAttempt(authHandler);
}

void AuthScreen::beginAuthAttempt(auth::AuthHandler& authHandler) {
    const bool useHash = usingStoredHash && std::strcmp(password, PASSWORD_PLACEHOLDER) == 0;
    const uint8_t protocolVersion = (authProtocolAttempt_ < authProtocols_.size())
        ? authProtocols_[authProtocolAttempt_] : uint8_t{8};
    const bool isRetry = (authProtocolAttempt_ > 0);

    std::stringstream ss;
    if (isRetry) {
        ss << "Retrying " << hostname << ":" << port
           << " with auth protocol " << static_cast<int>(protocolVersion) << "...";
    } else {
        ss << "Connecting to " << hostname << ":" << port << "...";
    }
    setStatus(ss.str(), false);

    // Wire up failure callback to capture specific error reason
    failureReason.clear();
    authHandler.setOnFailure([this](const std::string& reason) {
        failureReason = reason;
    });

    // Configure client version from active expansion profile
    auto* reg = core::Application::getInstance().getExpansionRegistry();
    if (reg) {
        auto* profile = reg->getActive();
        if (profile) {
            auth::ClientInfo info;
            info.majorVersion = profile->majorVersion;
            info.minorVersion = profile->minorVersion;
            info.patchVersion = profile->patchVersion;
            info.build = profile->build;
            info.protocolVersion = protocolVersion;
            info.game = profile->game;
            info.platform = profile->platform;
            info.os = profile->os;
            info.locale = profile->locale;
            info.timezone = profile->timezone;
            // Vanilla-family servers send the legacy realm-list layout even
            // when the auth handshake itself uses protocol v8 (vmangos), so key
            // this on the expansion rather than on the protocol byte in play.
            info.legacyVanillaRealmList = (profile->id == "classic" ||
                                           profile->id == "turtle" ||
                                           profile->protocolVersion <= 3);
            authHandler.setClientInfo(info);
        }
    }

    if (authHandler.connect(hostname, static_cast<uint16_t>(port))) {
        authenticating = true;
        authTimer = 0.0f;
        setStatus(isRetry ? "Reconnected, authenticating..." : "Connected, authenticating...", false);
        pinAutoSubmitted_ = false;
        securityPromptFocused_ = false;

        // Save login info for next session
        saveLoginInfo(false);

        const std::string pinStr = trimAscii(pinCode);

        // Send authentication credentials
        if (useHash) {
            auto hashBytes = hexDecode(savedPasswordHash);
            authHandler.authenticateWithHash(username, hashBytes, pinStr);
        } else {
            usingStoredHash = false;
            authHandler.authenticate(username, password, pinStr);
        }

        // Don't keep the code around longer than needed.
        pinCode[0] = '\0';
    } else {
        std::stringstream errSs;
        errSs << "Failed to connect to " << hostname << ":" << port
              << " - check that the server is online and the address is correct";
        setStatus(errSs.str(), true);
        authenticating = false;
    }
}

void AuthScreen::setStatus(const std::string& message, bool isError, bool prominent) {
    statusMessage = message;
    statusIsError = isError;
    statusProminent = prominent;
}

std::string AuthScreen::getConfigPath() {
    return core::getConfigRoot() + "/login.cfg";
}

void AuthScreen::saveLoginInfo(bool includePasswordHash) {
    upsertCurrentServerProfile(includePasswordHash);

    std::string path = getConfigPath();
    std::filesystem::path dir = std::filesystem::path(path).parent_path();
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);

    std::ofstream out(path);
    if (!out.is_open()) {
        LOG_WARNING("Could not save login info to ", path);
        return;
    }

    out << "version=3\n";
    out << "active=" << makeServerKey(hostname, port) << "\n";

    for (const auto& s : servers_) {
        out << "\n[server " << makeServerKey(s.hostname, s.port) << "]\n";
        out << "username=" << s.username << "\n";
        if (!s.passwordHash.empty()) {
            out << "password_hash=" << s.passwordHash << "\n";
        }
        if (!s.expansionId.empty()) {
            out << "expansion=" << s.expansionId << "\n";
        }
        if (!s.assetProfileId.empty()) {
            out << "assets=" << s.assetProfileId << "\n";
        }
    }

    LOG_INFO("Login info saved to ", path);
}

void AuthScreen::loadLoginInfo() {
    std::string path = getConfigPath();
    std::ifstream in(path);
    if (!in.is_open()) return;

    std::string file((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

    // If this looks like the old flat format, migrate it into a single server entry.
    if (file.find("[server ") == std::string::npos) {
        std::unordered_map<std::string, std::string> kv;
        std::istringstream ss(file);
        std::string line;
        while (std::getline(ss, line)) {
            line = trimAscii(line);
            if (line.empty() || line[0] == '#') continue;
            size_t eq = line.find('=');
            if (eq == std::string::npos) continue;
            kv[trimAscii(line.substr(0, eq))] = trimAscii(line.substr(eq + 1));
        }

        std::string host = kv["hostname"];
        int p = 3724;
        try { if (!kv["port"].empty()) p = std::stoi(kv["port"]); } catch (...) {}
        if (!host.empty()) {
            ServerProfile s;
            s.hostname = host;
            s.port = p;
            s.username = kv["username"];
            s.passwordHash = kv["password_hash"];
            s.expansionId = kv["expansion"];
            s.assetProfileId = kv["assets"];
            servers_.push_back(std::move(s));
            selectServerProfile(0);
        }

        LOG_INFO("Login info loaded from ", path, " (migrated legacy profile -> v3)");
        return;
    }

    servers_.clear();
    selectedServerIndex_ = -1;

    std::string activeKey;
    ServerProfile current;
    bool inServer = false;

    auto flushServer = [&]() {
        if (!inServer) return;
        if (!current.hostname.empty() && current.port > 0) {
            servers_.push_back(current);
        }
        current = ServerProfile{};
        inServer = false;
    };

    std::istringstream ss(file);
    std::string line;
    while (std::getline(ss, line)) {
        line = trimAscii(line);
        if (line.empty() || line[0] == '#') continue;

        if (line.front() == '[' && line.back() == ']') {
            flushServer();
            std::string inside = line.substr(1, line.size() - 2);
            inside = trimAscii(inside);
            const std::string prefix = "server ";
            if (inside.rfind(prefix, 0) == 0) {
                std::string key = trimAscii(inside.substr(prefix.size()));
                // Parse host:port (split on last ':', allow [ipv6]:port).
                std::string hostPart = key;
                int portPart = 3724;
                if (!key.empty() && key.front() == '[') {
                    auto rb = key.find(']');
                    if (rb != std::string::npos) {
                        hostPart = key.substr(1, rb - 1);
                        auto colon = key.find(':', rb);
                        if (colon != std::string::npos) {
                            try { portPart = std::stoi(key.substr(colon + 1)); } catch (...) {}
                        }
                    }
                } else {
                    auto colon = key.rfind(':');
                    if (colon != std::string::npos) {
                        hostPart = key.substr(0, colon);
                        try { portPart = std::stoi(key.substr(colon + 1)); } catch (...) {}
                    }
                }

                current.hostname = hostPart;
                current.port = portPart;
                inServer = true;
            }
            continue;
        }

        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string key = trimAscii(line.substr(0, eq));
        std::string val = trimAscii(line.substr(eq + 1));

        if (!inServer) {
            if (key == "active") activeKey = val;
            continue;
        }

        if (key == "username") current.username = val;
        else if (key == "password_hash") current.passwordHash = val;
        else if (key == "expansion") current.expansionId = val;
        else if (key == "assets") current.assetProfileId = val;
    }
    flushServer();

    if (!servers_.empty()) {
        std::sort(servers_.begin(), servers_.end(),
                  [](const ServerProfile& a, const ServerProfile& b) {
                      if (a.hostname != b.hostname) return a.hostname < b.hostname;
                      return a.port < b.port;
                  });

        if (!activeKey.empty()) {
            for (int i = 0; i < static_cast<int>(servers_.size()); ++i) {
                if (makeServerKey(servers_[i].hostname, servers_[i].port) == activeKey) {
                    selectServerProfile(i);
                    break;
                }
            }
        }

        if (selectedServerIndex_ < 0) {
            selectServerProfile(0);
        }
    }

    LOG_INFO("Login info loaded from ", path);
}

static uint32_t findMemType(VkPhysicalDevice pd, uint32_t filter, VkMemoryPropertyFlags props) {
    VkPhysicalDeviceMemoryProperties mp;
    vkGetPhysicalDeviceMemoryProperties(pd, &mp);
    for (uint32_t i = 0; i < mp.memoryTypeCount; i++) {
        if ((filter & (1 << i)) && (mp.memoryTypes[i].propertyFlags & props) == props) return i;
    }
    LOG_ERROR("AuthScreen: no suitable memory type found");
    return UINT32_MAX;
}

// Takes pixels already decoded on a worker thread and does the GPU-side work,
// which has to happen on the main thread.
bool AuthScreen::uploadBackgroundImage(const unsigned char* data) {
    auto& app = core::Application::getInstance();
    auto* renderer = app.getRenderer();
    if (!renderer) return false;
    bgVkCtx = renderer->getVkContext();
    if (!bgVkCtx) return false;
    if (!data) return false;

    VkDevice device = bgVkCtx->getDevice();
    VkPhysicalDevice physDevice = bgVkCtx->getPhysicalDevice();
    VkDeviceSize imageSize = static_cast<VkDeviceSize>(bgWidth) * bgHeight * 4;

    // Staging buffer
    VkBuffer stagingBuffer;
    VkDeviceMemory stagingMemory;
    {
        VkBufferCreateInfo bufInfo{};
        bufInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufInfo.size = imageSize;
        bufInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
        bufInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        vkCreateBuffer(device, &bufInfo, nullptr, &stagingBuffer);

        VkMemoryRequirements memReqs;
        vkGetBufferMemoryRequirements(device, stagingBuffer, &memReqs);
        VkMemoryAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memReqs.size;
        allocInfo.memoryTypeIndex = findMemType(physDevice, memReqs.memoryTypeBits,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        vkAllocateMemory(device, &allocInfo, nullptr, &stagingMemory);
        vkBindBufferMemory(device, stagingBuffer, stagingMemory, 0);

        void* mapped;
        vkMapMemory(device, stagingMemory, 0, imageSize, 0, &mapped);
        memcpy(mapped, data, imageSize);
        vkUnmapMemory(device, stagingMemory);
    }
    // The pixels belong to the caller's decoded buffer, not to stb_image.

    // Create VkImage
    {
        VkImageCreateInfo imgInfo{};
        imgInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        imgInfo.imageType = VK_IMAGE_TYPE_2D;
        imgInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
        imgInfo.extent = {.width = static_cast<uint32_t>(bgWidth), .height = static_cast<uint32_t>(bgHeight), .depth = 1};
        imgInfo.mipLevels = 1;
        imgInfo.arrayLayers = 1;
        imgInfo.samples = VK_SAMPLE_COUNT_1_BIT;
        imgInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
        imgInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        imgInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        imgInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        vkCreateImage(device, &imgInfo, nullptr, &bgImage);

        VkMemoryRequirements memReqs;
        vkGetImageMemoryRequirements(device, bgImage, &memReqs);
        VkMemoryAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memReqs.size;
        allocInfo.memoryTypeIndex = findMemType(physDevice, memReqs.memoryTypeBits,
            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        vkAllocateMemory(device, &allocInfo, nullptr, &bgMemory);
        vkBindImageMemory(device, bgImage, bgMemory, 0);
    }

    // Transfer
    bgVkCtx->immediateSubmit([&](VkCommandBuffer cmd) {
        VkImageMemoryBarrier barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = bgImage;
        barrier.subresourceRange = {.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .baseMipLevel = 0, .levelCount = 1, .baseArrayLayer = 0, .layerCount = 1};
        barrier.srcAccessMask = 0;
        barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);

        VkBufferImageCopy region{};
        region.imageSubresource = {.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .mipLevel = 0, .baseArrayLayer = 0, .layerCount = 1};
        region.imageExtent = {.width = static_cast<uint32_t>(bgWidth), .height = static_cast<uint32_t>(bgHeight), .depth = 1};
        vkCmdCopyBufferToImage(cmd, stagingBuffer, bgImage,
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

        barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);
    });

    vkDestroyBuffer(device, stagingBuffer, nullptr);
    vkFreeMemory(device, stagingMemory, nullptr);

    // Image view
    {
        VkImageViewCreateInfo viewInfo{};
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = bgImage;
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
        viewInfo.subresourceRange = {.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .baseMipLevel = 0, .levelCount = 1, .baseArrayLayer = 0, .layerCount = 1};
        vkCreateImageView(device, &viewInfo, nullptr, &bgImageView);
    }

    // Sampler
    {
        VkSamplerCreateInfo samplerInfo{};
        samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
        samplerInfo.magFilter = VK_FILTER_LINEAR;
        samplerInfo.minFilter = VK_FILTER_LINEAR;
        samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        bgSampler = bgVkCtx->getOrCreateSampler(samplerInfo);
    }

    bgDescriptorSet = ImGui_ImplVulkan_AddTexture(bgSampler, bgImageView,
        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

    LOG_INFO("Auth screen background loaded: ", bgWidth, "x", bgHeight);
    return true;
}
// ---------------------------------------------------------------------------
// Login-screen graphics settings popup
// ---------------------------------------------------------------------------

void AuthScreen::applyPresetToState(LoginGraphicsState& s, int preset) {
    // The numbers come from the one table both screens read. They were written
    // out here as well until they disagreed with it in four of ten columns, so
    // that choosing High here and High in game gave two different pictures.
    //
    // Custom is preset 0 and is not a set of values, so it is left alone.
    const int index = preset - 1;
    if (index < 0 || index >= kGraphicsPresetCount) return;
    const GraphicsPresetValues& p = kGraphicsPresets[index];

    s.viewDistance   = p.viewDistance;
    s.shadows        = p.shadows;
    s.shadowDistance = p.shadowDistance;
    s.antiAliasing   = p.antiAliasing;
    s.fxaa           = p.fxaa;
    s.normalMapping  = p.normalMapping;
    s.pom            = p.parallax;
    s.pomQuality     = p.parallaxQuality;
    s.groundClutter  = p.groundClutter;

    // Not in the table because the in-game preset has no opinion about them
    // either: upscaling and water refraction are the player's, and brightness,
    // vsync and fullscreen are not quality settings. A preset that reset them
    // would undo a display choice every time one was picked.
}

void AuthScreen::loadLoginGraphicsState() {
    std::ifstream file(SettingsPanel::getSettingsPath());
    if (!file.is_open()) {
        // File doesn't exist yet - keep struct defaults (Medium equivalent)
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq + 1);

        // Clamped to the same ranges GameScreen::loadSettings uses.
        //
        // Four of these had no clamp at all until 2026-08-15 - the preset
        // index, the parallax quality, the upscaling mode and the brightness -
        // so a value the file held outside its range was kept here, shown
        // against a control that could not represent it, and written back on
        // Apply. The shadow distance had one, but it started at 50 where the
        // schema and the game both start at 40, so a saved 40 came back as 50.
        //
        // Without this a value the file holds outside the range is kept, shown
        // against a slider that cannot represent it, and written straight back
        // on Apply - while the game clamps the same value on the way in. A
        // config carrying view_distance=0 showed the slider pinned at the far
        // left reading 0 and stayed there for good, with the world drawing at
        // 400 regardless. Reported as settings "always saved as low".
        const auto clampF = [](float v, float lo, float hi) {
            return v < lo ? lo : (v > hi ? hi : v);
        };
        const auto clampI = [](int v, int lo, int hi) {
            return v < lo ? lo : (v > hi ? hi : v);
        };
        if (key == "graphics_preset")       loginGfx_.preset        = clampI(std::stoi(val), 0, kGraphicsPresetCount);
        else if (key == "shadows")          loginGfx_.shadows        = (val == "1");
        else if (key == "shadow_distance")  loginGfx_.shadowDistance = clampF(std::stof(val), 40.0f, 500.0f);
        else if (key == "view_distance")    loginGfx_.viewDistance   = clampF(std::stof(val), 400.0f, 2400.0f);
        else if (key == "fog_sky_blend")    loginGfx_.fogSkyBlend    = clampF(std::stof(val), 0.0f, 1.0f);
        else if (key == "fog_strength")     loginGfx_.fogStrength    = clampF(std::stof(val), 0.0f, 2.0f);
        else if (key == "sharp_stars")      loginGfx_.sharpStars     = (val == "1");
        else if (key == "antialiasing")     loginGfx_.antiAliasing   = clampI(std::stoi(val), 0, 3);
        else if (key == "fxaa")             loginGfx_.fxaa           = (val == "1");
        else if (key == "normal_mapping")   loginGfx_.normalMapping  = (val == "1");
        else if (key == "pom")              loginGfx_.pom            = (val == "1");
        else if (key == "pom_quality")      loginGfx_.pomQuality     = clampI(std::stoi(val), 0, rendering::kPomQualityCount - 1);
        else if (key == "upscaling_mode")   loginGfx_.upscalingMode  = clampI(std::stoi(val), 0, 2);
        else if (key == "water_refraction") loginGfx_.waterRefraction = (val == "1");
        else if (key == "ground_clutter_density") loginGfx_.groundClutter = clampI(std::stoi(val), 0, 150);
        else if (key == "brightness")       loginGfx_.brightness     = clampI(std::stoi(val), 0, 100);
        else if (key == "vsync")            loginGfx_.vsync          = (val == "1");
        else if (key == "fullscreen")       loginGfx_.fullscreen     = (val == "1");
    }
}

void AuthScreen::saveLoginGraphicsState() {
    // Read the full settings file into a map to preserve non-graphics keys.
    std::map<std::string, std::string> cfg;
    std::ifstream in(SettingsPanel::getSettingsPath());
    if (in.is_open()) {
        std::string line;
        while (std::getline(in, line)) {
            auto eq = line.find('=');
            if (eq != std::string::npos)
                cfg[line.substr(0, eq)] = line.substr(eq + 1);
        }
        in.close();
    }

    // Overwrite graphics keys.
    cfg["graphics_preset"]       = std::to_string(loginGfx_.preset);
    cfg["shadows"]               = loginGfx_.shadows        ? "1" : "0";
    cfg["shadow_distance"]       = std::to_string(static_cast<int>(loginGfx_.shadowDistance));
    cfg["view_distance"]         = std::to_string(static_cast<int>(loginGfx_.viewDistance));
    cfg["fog_sky_blend"]         = std::to_string(loginGfx_.fogSkyBlend);
    cfg["fog_strength"]          = std::to_string(loginGfx_.fogStrength);
    cfg["sharp_stars"]           = loginGfx_.sharpStars      ? "1" : "0";
    cfg["antialiasing"]          = std::to_string(loginGfx_.antiAliasing);
    cfg["fxaa"]                  = loginGfx_.fxaa           ? "1" : "0";
    cfg["normal_mapping"]        = loginGfx_.normalMapping  ? "1" : "0";
    cfg["pom"]                   = loginGfx_.pom            ? "1" : "0";
    cfg["pom_quality"]           = std::to_string(loginGfx_.pomQuality);
    cfg["upscaling_mode"]        = std::to_string(loginGfx_.upscalingMode);
    cfg["water_refraction"]      = loginGfx_.waterRefraction ? "1" : "0";
    cfg["ground_clutter_density"]= std::to_string(loginGfx_.groundClutter);
    cfg["brightness"]            = std::to_string(loginGfx_.brightness);
    cfg["vsync"]                 = loginGfx_.vsync           ? "1" : "0";
    cfg["fullscreen"]            = loginGfx_.fullscreen      ? "1" : "0";

    // Write everything back.
    std::ofstream out(SettingsPanel::getSettingsPath());
    if (!out.is_open()) return;
    for (const auto& [k, v] : cfg)
        out << k << "=" << v << "\n";
}

void AuthScreen::renderLoginSettingsWindow() {
    if (showLoginSettings_) {
        ImGui::OpenPopup("Graphics Settings");
        showLoginSettings_ = false;
        loginGfxLoaded_ = false; // Reload from disk each time the popup opens.
    }

    ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(500, 560), ImGuiCond_Always);

    if (ImGui::BeginPopupModal("Graphics Settings", nullptr, ImGuiWindowFlags_NoResize)) {
        if (!loginGfxLoaded_) {
            loadLoginGraphicsState();
            loginGfxLoaded_ = true;
        }

        ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.0f, 1.0f), "Graphics Settings");
        ImGui::TextWrapped("Adjust settings below or reset to a safe preset. Changes take effect on next login.");
        ImGui::Separator();
        ImGui::Spacing();

        // Preset selector
        const char* presetNames[] = {"Custom", "Low", "Medium", "High", "Ultra"};
        ImGui::Text("Preset:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(160.0f);
        if (ImGui::Combo("##preset", &loginGfx_.preset, presetNames, 5)) {
            if (loginGfx_.preset != 0) // 0 = Custom - don't override manually set values
                applyPresetToState(loginGfx_, loginGfx_.preset);
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Shadow settings
        ImGui::Checkbox("Shadows", &loginGfx_.shadows);
        if (loginGfx_.shadows) {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(200.0f);
            float sd = loginGfx_.shadowDistance;
            if (ImGui::SliderFloat("Shadow Distance", &sd, 50.0f, 600.0f, "%.0f"))
                loginGfx_.shadowDistance = sd;
        }

        ImGui::SetNextItemWidth(240.0f);
        ImGui::SliderFloat("View Distance", &loginGfx_.viewDistance,
                           400.0f, 2400.0f, "%.0f");

        // Anti-aliasing
        // The same four the settings schema offers. This list had three, so
        // 8x could be set in the game and never shown or chosen here - and
        // picking any mode here wrote a value this list could not name back.
        const char* aaNames[] = {"Off", "2x MSAA", "4x MSAA", "8x MSAA"};
        ImGui::Text("Anti-Aliasing:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(130.0f);
        ImGui::Combo("##aa", &loginGfx_.antiAliasing, aaNames, IM_ARRAYSIZE(aaNames));

        ImGui::Checkbox("FXAA",           &loginGfx_.fxaa);
        ImGui::Checkbox("Normal Mapping", &loginGfx_.normalMapping);

        // POM
        ImGui::Checkbox("Parallax Occlusion Mapping (POM)", &loginGfx_.pom);
        if (loginGfx_.pom) {
            // The names and the count both come from the one scale, so
            // this cannot go back to offering two entries of a three entry
            // setting and writing the wrong index for the ones it did offer.
            ImGui::Text("  POM Quality:");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(110.0f);
            ImGui::Combo("##pomq", &loginGfx_.pomQuality,
                         rendering::kPomQualityLabels, rendering::kPomQualityCount);
        }

        ImGui::Checkbox("Water Refraction",  &loginGfx_.waterRefraction);

        // Ground clutter density
        ImGui::Text("Ground Clutter:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(200.0f);
        // 150, not 200: GameScreen::loadSettings clamps there, so the last
        // fifty units of this slider were silently discarded at login.
        ImGui::SliderInt("##clutter", &loginGfx_.groundClutter, 0, 150);

        // Brightness
        ImGui::Text("Brightness:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(200.0f);
        ImGui::SliderInt("##brightness", &loginGfx_.brightness, 0, 100);

        ImGui::Checkbox("V-Sync",      &loginGfx_.vsync);
        ImGui::Checkbox("Fullscreen",  &loginGfx_.fullscreen);

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Action buttons
        if (ImGui::Button("Reset to Medium", ImVec2(160, 32))) {
            applyPresetToState(loginGfx_, 2);
            loginGfx_.preset = 2;
        }
        ImGui::SameLine();

        float rightEdge = ImGui::GetContentRegionAvail().x;
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + rightEdge - 220.0f);
        if (ImGui::Button("Cancel", ImVec2(100, 32))) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Apply", ImVec2(100, 32))) {
            saveLoginGraphicsState();
            if (services_.window &&
                services_.window->isVsyncEnabled() != loginGfx_.vsync) {
                services_.window->setVsync(loginGfx_.vsync);
            }
            ImGui::CloseCurrentPopup();
        }

        ImGui::EndPopup();
    }
}

}} // namespace wowee::ui
