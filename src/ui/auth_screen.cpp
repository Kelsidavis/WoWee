#include "ui/auth_screen.hpp"
#include "auth/crypto.hpp"
#include "core/application.hpp"
#include "core/logger.hpp"
#include "rendering/renderer.hpp"
#include "pipeline/asset_manager.hpp"
#include "audio/music_manager.hpp"
#include <imgui.h>
#include <filesystem>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <algorithm>
#include <iomanip>
#include <array>
#include <random>

namespace wowee { namespace ui {

static std::string hexEncode(const std::vector<uint8_t>& data) {
    std::ostringstream ss;
    for (uint8_t b : data)
        ss << std::hex << std::setfill('0') << std::setw(2) << (int)b;
    return ss.str();
}

static std::vector<uint8_t> hexDecode(const std::string& hex) {
    std::vector<uint8_t> bytes;
    for (size_t i = 0; i + 1 < hex.size(); i += 2) {
        uint8_t b = static_cast<uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16));
        bytes.push_back(b);
    }
    return bytes;
}

AuthScreen::AuthScreen() {
}

void AuthScreen::render(auth::AuthHandler& authHandler) {
    // Load saved login info on first render
    if (!loginInfoLoaded) {
        loadLoginInfo();
        loginInfoLoaded = true;
    }

    if (!videoInitAttempted) {
        videoInitAttempted = true;
        std::string videoPath = "assets/startscreen.mp4";
        if (!std::filesystem::exists(videoPath)) {
            videoPath = (std::filesystem::current_path() / "assets/startscreen.mp4").string();
        }
        backgroundVideo.open(videoPath);
    }
    backgroundVideo.update(ImGui::GetIO().DeltaTime);
    if (backgroundVideo.isReady()) {
        ImVec2 screen = ImGui::GetIO().DisplaySize;
        float screenW = screen.x;
        float screenH = screen.y;
        float videoW = static_cast<float>(backgroundVideo.getWidth());
        float videoH = static_cast<float>(backgroundVideo.getHeight());
        if (videoW > 0.0f && videoH > 0.0f) {
            float screenAspect = screenW / screenH;
            float videoAspect = videoW / videoH;
            ImVec2 uv0(0.0f, 0.0f);
            ImVec2 uv1(1.0f, 1.0f);
            if (videoAspect > screenAspect) {
                float scale = screenAspect / videoAspect;
                float crop = (1.0f - scale) * 0.5f;
                uv0.x = crop;
                uv1.x = 1.0f - crop;
            } else if (videoAspect < screenAspect) {
                float scale = videoAspect / screenAspect;
                float crop = (1.0f - scale) * 0.5f;
                uv0.y = crop;
                uv1.y = 1.0f - crop;
            }
            ImDrawList* bg = ImGui::GetBackgroundDrawList();
            bg->AddImage(static_cast<ImTextureID>(static_cast<uintptr_t>(backgroundVideo.getTextureId())),
                         ImVec2(0, 0), ImVec2(screenW, screenH), uv0, uv1);
        }
    }

    auto& app = core::Application::getInstance();
    auto* renderer = app.getRenderer();
    if (!musicInitAttempted) {
        musicInitAttempted = true;
        auto* assets = app.getAssetManager();
        if (renderer) {
            auto* music = renderer->getMusicManager();
            if (music && assets && assets->isInitialized() && !music->isInitialized()) {
                music->initialize(assets);
            }
        }
    }
    if (renderer) {
        auto* music = renderer->getMusicManager();
        if (music) {
            music->update(ImGui::GetIO().DeltaTime);
            if (!music->isPlaying()) {
                static std::mt19937 rng(std::random_device{}());
                static const std::array<const char*, 3> kLoginTracks = {
                    "Raise the Mug, Sound the Warcry.mp3",
                    "Wanderwewill.mp3",
                    "Gold on the Tide in Booty Bay.mp3"
                };

                std::vector<std::string> availableTracks;
                availableTracks.reserve(kLoginTracks.size());
                for (const char* track : kLoginTracks) {
                    std::filesystem::path localPath = std::filesystem::path("assets") / track;
                    if (std::filesystem::exists(localPath)) {
                        availableTracks.push_back(localPath.string());
                        continue;
                    }

                    std::filesystem::path cwdPath = std::filesystem::current_path() / "assets" / track;
                    if (std::filesystem::exists(cwdPath)) {
                        availableTracks.push_back(cwdPath.string());
                    }
                }

                if (!availableTracks.empty()) {
                    std::uniform_int_distribution<size_t> pick(0, availableTracks.size() - 1);
                    const std::string& path = availableTracks[pick(rng)];
                    music->playFilePath(path, true);
                    LOG_INFO("AuthScreen: Playing login intro track: ", path);
                    musicPlaying = music->isPlaying();
                } else {
                    LOG_WARNING("AuthScreen: No login intro tracks found in assets/");
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
    ImGui::InputText("Hostname", hostname, sizeof(hostname));
    ImGui::InputInt("Port", &port);
    if (port < 1) port = 1;
    if (port > 65535) port = 65535;

    // Compatibility mode dropdown
    const char* compatModes[] = { "3.3.5a" };
    ImGui::Combo("Compatibility Mode", &compatibilityMode, compatModes, IM_ARRAYSIZE(compatModes));

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

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Connection status
    if (!statusMessage.empty()) {
        if (statusIsError) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.3f, 0.3f, 1.0f));
        } else {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.3f, 1.0f, 0.3f, 1.0f));
        }
        ImGui::TextWrapped("%s", statusMessage.c_str());
        ImGui::PopStyleColor();
        ImGui::Spacing();
    }

    // Connect button
    if (authenticating) {
        authTimer += ImGui::GetIO().DeltaTime;

        // Show progress with elapsed time
        char progressBuf[128];
        snprintf(progressBuf, sizeof(progressBuf), "Authenticating... (%.0fs)", authTimer);
        ImGui::Text("%s", progressBuf);

        // Check authentication status
        auto state = authHandler.getState();
        if (state == auth::AuthState::AUTHENTICATED) {
            setStatus("Authentication successful!", false);
            authenticating = false;

            // Compute and save password hash if user typed a fresh password
            if (!usingStoredHash) {
                std::string upperUser = username;
                std::string upperPass = password;
                std::transform(upperUser.begin(), upperUser.end(), upperUser.begin(), ::toupper);
                std::transform(upperPass.begin(), upperPass.end(), upperPass.begin(), ::toupper);
                std::string combined = upperUser + ":" + upperPass;
                auto hash = auth::Crypto::sha1(combined);
                savedPasswordHash = hexEncode(hash);
            }
            saveLoginInfo();

            // Call success callback
            if (onSuccess) {
                onSuccess();
            }
        } else if (state == auth::AuthState::FAILED) {
            if (!failureReason.empty()) {
                setStatus(failureReason, true);
            } else {
                setStatus("Authentication failed", true);
            }
            authenticating = false;
        } else if (authTimer >= AUTH_TIMEOUT) {
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
        }
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Info text
    ImGui::TextWrapped("Enter your account credentials to connect to the authentication server.");
    ImGui::TextWrapped("Default port is 3724.");

    ImGui::End();
}

void AuthScreen::stopLoginMusic() {
    if (!musicPlaying) return;
    auto& app = core::Application::getInstance();
    auto* renderer = app.getRenderer();
    if (!renderer) return;
    auto* music = renderer->getMusicManager();
    if (!music) return;
    music->stopMusic(500.0f);
    musicPlaying = false;
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

    // Attempt connection
    std::stringstream ss;
    ss << "Connecting to " << hostname << ":" << port << "...";
    setStatus(ss.str(), false);

    // Wire up failure callback to capture specific error reason
    failureReason.clear();
    authHandler.setOnFailure([this](const std::string& reason) {
        failureReason = reason;
    });

    if (authHandler.connect(hostname, static_cast<uint16_t>(port))) {
        authenticating = true;
        authTimer = 0.0f;
        setStatus("Connected, authenticating...", false);

        // Save login info for next session
        saveLoginInfo();

        // Send authentication credentials
        if (useHash) {
            auto hashBytes = hexDecode(savedPasswordHash);
            authHandler.authenticateWithHash(username, hashBytes);
        } else {
            usingStoredHash = false;
            authHandler.authenticate(username, password);
        }
    } else {
        std::stringstream errSs;
        errSs << "Failed to connect to " << hostname << ":" << port
              << " - check that the server is online and the address is correct";
        setStatus(errSs.str(), true);
    }
}

void AuthScreen::setStatus(const std::string& message, bool isError) {
    statusMessage = message;
    statusIsError = isError;
}

std::string AuthScreen::getConfigPath() {
    std::string dir;
#ifdef _WIN32
    const char* appdata = std::getenv("APPDATA");
    dir = appdata ? std::string(appdata) + "\\wowee" : ".";
#else
    const char* home = std::getenv("HOME");
    dir = home ? std::string(home) + "/.wowee" : ".";
#endif
    return dir + "/login.cfg";
}

void AuthScreen::saveLoginInfo() {
    std::string path = getConfigPath();
    std::filesystem::path dir = std::filesystem::path(path).parent_path();
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);

    std::ofstream out(path);
    if (!out.is_open()) {
        LOG_WARNING("Could not save login info to ", path);
        return;
    }

    out << "hostname=" << hostname << "\n";
    out << "port=" << port << "\n";
    out << "username=" << username << "\n";
    if (!savedPasswordHash.empty()) {
        out << "password_hash=" << savedPasswordHash << "\n";
    }

    LOG_INFO("Login info saved to ", path);
}

void AuthScreen::loadLoginInfo() {
    std::string path = getConfigPath();
    std::ifstream in(path);
    if (!in.is_open()) return;

    std::string line;
    while (std::getline(in, line)) {
        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq + 1);

        if (key == "hostname" && !val.empty()) {
            strncpy(hostname, val.c_str(), sizeof(hostname) - 1);
            hostname[sizeof(hostname) - 1] = '\0';
        } else if (key == "port") {
            try { port = std::stoi(val); } catch (...) {}
        } else if (key == "username" && !val.empty()) {
            strncpy(username, val.c_str(), sizeof(username) - 1);
            username[sizeof(username) - 1] = '\0';
        } else if (key == "password_hash" && !val.empty()) {
            savedPasswordHash = val;
        }
    }

    // If we have a saved hash, fill password with placeholder
    if (!savedPasswordHash.empty()) {
        strncpy(password, PASSWORD_PLACEHOLDER, sizeof(password) - 1);
        password[sizeof(password) - 1] = '\0';
        usingStoredHash = true;
    }

    LOG_INFO("Login info loaded from ", path);
}

}} // namespace wowee::ui
