#include "ui/auth_screen.hpp"
#include "auth/crypto.hpp"
#include "core/application.hpp"
#include "core/logger.hpp"
#include "rendering/renderer.hpp"
#include "pipeline/asset_manager.hpp"
#include "audio/music_manager.hpp"
#include "game/expansion_profile.hpp"
#include <imgui.h>
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
        if (state != auth::AuthState::PIN_REQUIRED) {
            authTimer += ImGui::GetIO().DeltaTime;

            // Show progress with elapsed time
            char progressBuf[128];
            snprintf(progressBuf, sizeof(progressBuf), "Authenticating... (%.0fs)", authTimer);
            ImGui::Text("%s", progressBuf);
        } else {
            ImGui::TextWrapped("This server requires a PIN. Enter your PIN to continue.");
            ImGui::InputText("PIN", pinCode, sizeof(pinCode), ImGuiInputTextFlags_Password);
            ImGui::SameLine();
            if (ImGui::Button("Submit PIN")) {
                authHandler.submitPin(pinCode);
                // Don't keep the PIN around longer than needed.
                pinCode[0] = '\0';
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
        } else if (state != auth::AuthState::PIN_REQUIRED && authTimer >= AUTH_TIMEOUT) {
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
            authHandler.setClientInfo(info);
        }
    }

    if (authHandler.connect(hostname, static_cast<uint16_t>(port))) {
        authenticating = true;
        authTimer = 0.0f;
        setStatus("Connected, authenticating...", false);

        // Save login info for next session
        saveLoginInfo(false);

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

}} // namespace wowee::ui
