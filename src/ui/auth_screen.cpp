#include "ui/auth_screen.hpp"
#include "auth/crypto.hpp"
#include "core/application.hpp"
#include "core/logger.hpp"
#include "rendering/renderer.hpp"
#include "rendering/vk_context.hpp"
#include "pipeline/asset_manager.hpp"
#include "audio/music_manager.hpp"
#include "game/expansion_profile.hpp"
#include <imgui.h>
#include <imgui_impl_vulkan.h>
#include "stb_image.h"
#include <filesystem>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <filesystem>
#include <algorithm>
#include <iomanip>
#include <array>
#include <random>
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

    if (!s.expansionId.empty()) {
        auto* expReg = core::Application::getInstance().getExpansionRegistry();
        if (expReg && expReg->setActive(s.expansionId)) {
            auto& profiles = expReg->getAllProfiles();
            for (int i = 0; i < static_cast<int>(profiles.size()); ++i) {
                if (profiles[i].id == s.expansionId) { expansionIndex = i; break; }
            }
            core::Application::getInstance().reloadExpansionData();
        }
    }
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
    }

    if (!bgInitAttempted) {
        bgInitAttempted = true;
        loadBackgroundImage();
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
            if (!loginMusicVolumeAdjusted_) {
                savedMusicVolume_ = music->getVolume();
                int loginVolume = (savedMusicVolume_ * 80) / 100; // reduce auth music by 20%
                if (loginVolume < 0) loginVolume = 0;
                if (loginVolume > 100) loginVolume = 100;
                music->setVolume(loginVolume);
                loginMusicVolumeAdjusted_ = true;
            }
            music->update(ImGui::GetIO().DeltaTime);
            if (!music->isPlaying()) {
                static std::mt19937 rng(std::random_device{}());
                if (!introTracksScanned_) {
                    introTracksScanned_ = true;

                    // Tracks in assets/ root
                    static const std::array<const char*, 1> kRootTracks = {
                        "Raise the Mug, Sound the Warcry.mp3",
                    };
                    // Tracks in assets/Original Music/
                    static const std::array<const char*, 11> kOriginalTracks = {
                        "Gold on the Tide in Booty Bay.mp3",
                        "Lanterns Over Lordaeron.mp3",
                        "Loot the Dogs.mp3",
                        "One More Pull.mp3",
                        "Roll Need Greed.mp3",
                        "RunBackPolka.mp3",
                        "The Barrens Has No End.mp3",
                        "The Bone Collector.mp3",
                        "Wanderwewill.mp3",
                        "WHO PULLED_.mp3",
                        "You No Take Candle!.mp3",
                    };

                    auto tryAddTrack = [&](const std::filesystem::path& base, const char* track) {
                        std::filesystem::path p = base / track;
                        if (std::filesystem::exists(p)) {
                            introTracks_.push_back(p.string());
                        }
                    };
                    for (const char* track : kRootTracks) {
                        tryAddTrack("assets", track);
                        if (introTracks_.empty()) {
                            tryAddTrack(std::filesystem::current_path() / "assets", track);
                        }
                    }
                    for (const char* track : kOriginalTracks) {
                        tryAddTrack(std::filesystem::path("assets") / "Original Music", track);
                        tryAddTrack(std::filesystem::current_path() / "assets" / "Original Music", track);
                    }

                    std::sort(introTracks_.begin(), introTracks_.end());
                    introTracks_.erase(std::unique(introTracks_.begin(), introTracks_.end()), introTracks_.end());
                }

                if (!introTracks_.empty()) {
                    std::uniform_int_distribution<size_t> pick(0, introTracks_.size() - 1);
                    const size_t idx = pick(rng);
                    const std::string path = introTracks_[idx];
                    music->playFilePath(path, true, 1800.0f);
                    musicPlaying = music->isPlaying();
                    if (musicPlaying) {
                        LOG_INFO("AuthScreen: Playing login intro track: ", path);
                    } else {
                        // Drop bad paths to avoid retrying the same failed file every frame.
                        introTracks_.erase(introTracks_.begin() + idx);
                    }
                } else if (!missingIntroTracksLogged_) {
                    LOG_WARNING("AuthScreen: No login intro tracks found in assets/");
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

    // Optional 2FA / PIN field (some servers require this; e.g. Turtle WoW uses Google Authenticator).
    // Keep it visible pre-connect so we can send LOGON_PROOF immediately after the SRP challenge.
    {
        ImGuiInputTextFlags pinFlags = ImGuiInputTextFlags_Password;
        if (authHandler.getState() == auth::AuthState::PIN_REQUIRED) {
            pinFlags |= ImGuiInputTextFlags_EnterReturnsTrue;
        }
        ImGui::InputText("2FA / PIN", pinCode, sizeof(pinCode), pinFlags);
        ImGui::SameLine();
        ImGui::TextDisabled("(optional)");
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
        auto state = authHandler.getState();
        if (state != auth::AuthState::PIN_REQUIRED && state != auth::AuthState::AUTHENTICATOR_REQUIRED) {
            pinAutoSubmitted_ = false;
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
                authHandler.submitSecurityCode(pinCode);
                // Don't keep the code around longer than needed.
                pinCode[0] = '\0';
                pinAutoSubmitted_ = true;
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
                std::transform(upperUser.begin(), upperUser.end(), upperUser.begin(), ::toupper);
                std::transform(upperPass.begin(), upperPass.end(), upperPass.begin(), ::toupper);
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
            if (!failureReason.empty()) {
                setStatus(failureReason, true);
            } else {
                setStatus("Authentication failed", true);
            }
            authenticating = false;
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
    auto& app = core::Application::getInstance();
    auto* renderer = app.getRenderer();
    if (!renderer) return;
    auto* music = renderer->getMusicManager();
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

    // Attempt connection
    std::stringstream ss;
    ss << "Connecting to " << hostname << ":" << port << "...";
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
            info.protocolVersion = profile->protocolVersion;
            info.game = profile->game;
            info.platform = profile->platform;
            info.os = profile->os;
            info.locale = profile->locale;
            info.timezone = profile->timezone;
            authHandler.setClientInfo(info);
        }
    }

    if (authHandler.connect(hostname, static_cast<uint16_t>(port))) {
        authenticating = true;
        authTimer = 0.0f;
        setStatus("Connected, authenticating...", false);
        pinAutoSubmitted_ = false;

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

    out << "version=2\n";
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
            servers_.push_back(std::move(s));
            selectServerProfile(0);
        }

        LOG_INFO("Login info loaded from ", path, " (migrated v1 -> v2)");
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
    return 0;
}

bool AuthScreen::loadBackgroundImage() {
    auto& app = core::Application::getInstance();
    auto* renderer = app.getRenderer();
    if (!renderer) return false;
    bgVkCtx = renderer->getVkContext();
    if (!bgVkCtx) return false;

    std::string imgPath = "assets/krayonsignin.png";
    if (!std::filesystem::exists(imgPath))
        imgPath = (std::filesystem::current_path() / imgPath).string();

    int channels;
    stbi_set_flip_vertically_on_load(false);
    unsigned char* data = stbi_load(imgPath.c_str(), &bgWidth, &bgHeight, &channels, 4);
    if (!data) {
        LOG_WARNING("Auth screen: failed to load background image: ", imgPath);
        return false;
    }

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
    stbi_image_free(data);

    // Create VkImage
    {
        VkImageCreateInfo imgInfo{};
        imgInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        imgInfo.imageType = VK_IMAGE_TYPE_2D;
        imgInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
        imgInfo.extent = {static_cast<uint32_t>(bgWidth), static_cast<uint32_t>(bgHeight), 1};
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
        barrier.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        barrier.srcAccessMask = 0;
        barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);

        VkBufferImageCopy region{};
        region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
        region.imageExtent = {static_cast<uint32_t>(bgWidth), static_cast<uint32_t>(bgHeight), 1};
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
        viewInfo.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
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
        vkCreateSampler(device, &samplerInfo, nullptr, &bgSampler);
    }

    bgDescriptorSet = ImGui_ImplVulkan_AddTexture(bgSampler, bgImageView,
        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

    LOG_INFO("Auth screen background loaded: ", bgWidth, "x", bgHeight);
    return true;
}

void AuthScreen::destroyBackgroundImage() {
    if (!bgVkCtx) return;
    VkDevice device = bgVkCtx->getDevice();
    vkDeviceWaitIdle(device);
    if (bgDescriptorSet) { ImGui_ImplVulkan_RemoveTexture(bgDescriptorSet); bgDescriptorSet = VK_NULL_HANDLE; }
    if (bgSampler) { vkDestroySampler(device, bgSampler, nullptr); bgSampler = VK_NULL_HANDLE; }
    if (bgImageView) { vkDestroyImageView(device, bgImageView, nullptr); bgImageView = VK_NULL_HANDLE; }
    if (bgImage) { vkDestroyImage(device, bgImage, nullptr); bgImage = VK_NULL_HANDLE; }
    if (bgMemory) { vkFreeMemory(device, bgMemory, nullptr); bgMemory = VK_NULL_HANDLE; }
}

}} // namespace wowee::ui
