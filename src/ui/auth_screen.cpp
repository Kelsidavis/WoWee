#include "ui/auth_screen.hpp"
#include <future>
#include <chrono>
#include "rendering/pom_quality.hpp"
#include "ui/graphics_presets.hpp"
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

    hostname_.setText(s.hostname);
    setPort(s.port);
    username_.setText(s.username);

    savedPasswordHash = s.passwordHash;
    usingStoredHash = !savedPasswordHash.empty();
    password_.setText(usingStoredHash ? PASSWORD_PLACEHOLDER : "");

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
    const std::string hostStr = hostname_.text();
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
    s.username = username_.text();
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

// ---------------------------------------------------------------------------
// The card
// ---------------------------------------------------------------------------

namespace {

// The card's layout, written in units and drawn at a scale taken from the
// window, so what follows is its proportions rather than its pixels.
constexpr float kCardWidth     = 432.0f;
constexpr float kCardPadX      = 34.0f;
constexpr float kCardPadTop    = 26.0f;
constexpr float kCardPadBottom = 20.0f;
constexpr float kTitleSize     = 44.0f;
constexpr float kLabelSize     = 14.0f;
constexpr float kBodySize      = 15.0f;
constexpr float kSmallSize     = 13.0f;
constexpr float kFieldHeight   = 38.0f;
constexpr float kButtonHeight  = 46.0f;
constexpr float kRowGap        = 14.0f;
constexpr float kLabelGap      = 3.0f;

/// A code that looks like one someone finished typing.
///
/// PIN grids run four to ten digits and an authenticator is six, which is why
/// six is named twice: it is in range either way and the check reads clearer
/// saying so than leaving it implied.
bool looksLikeSecurityCode(const std::string& code) {
    if (code.empty()) return false;
    for (char c : code)
        if (c < '0' || c > '9') return false;
    return (code.size() >= 4 && code.size() <= 10) || code.size() == 6;
}

} // namespace

void AuthScreen::setPort(int value) {
    port = std::clamp(value, 1, 65535);
    char buf[8];
    std::snprintf(buf, sizeof(buf), "%d", port);
    portText_.setText(buf);
}

void AuthScreen::render(auth::AuthHandler& authHandler) {
    // Load saved login info on first render
    if (!loginInfoLoaded) {
        loadLoginInfo();
        loginInfoLoaded = true;
        if (portText_.empty()) setPort(port);
        if (hostname_.empty()) hostname_.setText("localhost");
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

    drawBackdrop();
    updateMusic();

    const ImVec2 screen = ImGui::GetIO().DisplaySize;
    // A share of the window rather than a count of pixels, so the card is the
    // same size on a phone, on a laptop and on a 4K monitor. The floor keeps
    // it legible in a small window; the ceiling keeps it from swallowing a
    // large one.
    const float scale = std::clamp(std::min(screen.x / 1280.0f, screen.y / 760.0f), 0.62f, 2.6f);
    ui_.begin(ImGui::GetIO().DeltaTime, scale);
    ui_.setLayer(PaperLayer::Page);

    // Everything about authenticating that is not drawing. It runs before the
    // card so the card can simply read the state it leaves behind.
    const auto authState = authHandler.getState();
    const bool waitingForSecurityCode = (authState == auth::AuthState::PIN_REQUIRED ||
                                         authState == auth::AuthState::AUTHENTICATOR_REQUIRED);

    if (authenticating) {
        if (!waitingForSecurityCode) {
            pinAutoSubmitted_ = false;
            securityPromptFocused_ = false;
            authTimer += ImGui::GetIO().DeltaTime;
        } else if (!pinAutoSubmitted_ && looksLikeSecurityCode(pinCode_.text())) {
            // The player typed a code before pressing the button; send it now
            // rather than making them press a second one.
            authHandler.submitSecurityCode(pinCode_.text());
            pinCode_.clear();
            pinAutoSubmitted_ = true;
        }

        if (authState == auth::AuthState::AUTHENTICATED) {
            setStatus("Authentication successful!", false);
            authenticating = false;

            // Compute and save password hash if user typed a fresh password
            if (!usingStoredHash) {
                std::string upperUser = username_.text();
                std::string upperPass = password_.text();
                auto toUp = [](unsigned char c) { return static_cast<char>(std::toupper(c)); };
                std::transform(upperUser.begin(), upperUser.end(), upperUser.begin(), toUp);
                std::transform(upperPass.begin(), upperPass.end(), upperPass.begin(), toUp);
                std::string combined = upperUser + ":" + upperPass;
                auto hash = auth::Crypto::sha1(combined);
                savedPasswordHash = hexEncode(hash);
            }
            saveLoginInfo(true);

            if (onSuccess) {
                onSuccess();
            }
        } else if (authState == auth::AuthState::FAILED) {
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
                setStatus(failureReason.empty() ? "Authentication failed" : failureReason, true);
                authenticating = false;
            }
        } else if (!waitingForSecurityCode && authTimer >= AUTH_TIMEOUT) {
            setStatus("Connection timed out - server did not respond", true);
            authenticating = false;
            authHandler.disconnect();
        }
    }

    renderProminentStatus(screen.x, screen.y);

    // Whether a dropdown was down when the frame began. Escape closes one
    // from inside the control, so asking afterwards would find it already
    // shut and go on to close the settings sheet behind it - two things
    // dismissed by one press.
    const bool popupWasOpen = ui_.popupOpen();

    // The card hears nothing while the settings sheet is over it, but it is
    // still drawn - a modal that blanks what it covers loses the player's
    // place.
    //
    // Asked once. The gear is on the card, so rendering it can turn the sheet
    // on halfway through, and asking again afterwards popped an inertness
    // that had never been pushed.
    const bool sheetWasOpen = settingsOpen_;
    if (sheetWasOpen) ui_.pushInert();
    renderCard(authHandler, screen.x, screen.y);
    if (sheetWasOpen) ui_.popInert();

    if (settingsOpen_) renderLoginSettingsSheet(screen.x, screen.y);

    // Escape backs out of one thing per press: the open dropdown, then the
    // settings sheet, then the keyboard.
    if (!popupWasOpen && ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
        if (settingsOpen_) {
            settingsOpen_ = false;
        } else {
            ui_.clearFocus();
        }
    }

    // Enter with the keyboard in none of the boxes still means log in. The
    // screen opens with nothing focused - deliberately, so a phone does not
    // meet it with the on-screen keyboard already up over the card - and
    // pressing Enter there should do the obvious thing rather than nothing.
    if (!settingsOpen_ && !authenticating && !ui_.wantsTextInput() &&
        (ImGui::IsKeyPressed(ImGuiKey_Enter, false) ||
         ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false))) {
        attemptAuth(authHandler);
    }

    // Build version, bottom-left over the login art, as the retail client
    // does. On the overlay because the art behind it is a photograph and it
    // needs its own shadow to stay readable over any part of one.
    {
        ui_.setLayer(PaperLayer::Overlay);
        const float size = ui_.px(kSmallSize);
        const ImVec2 at(ui_.px(14), screen.y - ui_.lineHeight(size) - ui_.px(8));
        ui_.text(ImVec2(at.x + 1.0f, at.y + 1.0f), core::kVersionString, size,
                 IM_COL32(0, 0, 0, 150));
        ui_.text(at, core::kVersionString, size, IM_COL32(0xEC, 0xE2, 0xCC, 0xC8));
        ui_.setLayer(PaperLayer::Page);
    }

    ui_.end();
}

void AuthScreen::renderProminentStatus(float screenW, float screenH) {
    // Across the top of the screen, on its own strip of paper.
    //
    // A disconnect is not a form being wrong - the player did not ask to be
    // here and may not have been looking - so it is said where it cannot be
    // missed rather than as another red line inside the card. It is cleared
    // like any other status: by typing, or by connecting again.
    if (!statusProminent || statusMessage.empty()) return;

    ui_.setLayer(PaperLayer::Overlay);
    const float size = ui_.px(30);
    const float pad = ui_.px(26);
    const float width = std::min(screenW - ui_.px(80),
                                 ui_.textWidth(statusMessage.c_str(), size) + pad * 2.0f);
    const float textW = width - pad * 2.0f;
    const float textH = ui_.wrappedHeight(textW, statusMessage.c_str(), size);
    const float x0 = (screenW - width) * 0.5f;
    const float y0 = screenH * 0.10f;
    const ImVec2 a(x0, y0);
    const ImVec2 b(x0 + width, y0 + textH + pad);

    ui_.sheet(a, b, /*taped=*/false);
    ui_.wrapped(ImVec2(a.x + pad, a.y + pad * 0.5f), textW, statusMessage.c_str(), size,
                ui_.theme().crayonRed);
    ui_.setLayer(PaperLayer::Page);
}

void AuthScreen::renderCard(auth::AuthHandler& authHandler, float screenW, float screenH) {
    const PaperTheme& theme = ui_.theme();
    const auto px = [this](float units) { return ui_.px(units); };

    const auto authState = authHandler.getState();
    const bool waitingForSecurityCode = (authState == auth::AuthState::PIN_REQUIRED ||
                                         authState == auth::AuthState::AUTHENTICATOR_REQUIRED);

    const float labelSize = px(kLabelSize);
    const float bodySize = px(kBodySize);
    const float smallSize = px(kSmallSize);
    const float labelRow = ui_.lineHeight(labelSize) + px(kLabelGap);
    const float fieldRow = labelRow + px(kFieldHeight);
    const float smallRow = ui_.lineHeight(smallSize);
    const float contentW = px(kCardWidth) - px(kCardPadX) * 2.0f;

    // A security prompt outranks the disclosure: when the server is waiting
    // for a code, the box for it belongs in the middle of the card and not
    // three rows down behind a link.
    const bool codeInMain = waitingForSecurityCode;
    const bool codeInAdvanced = !codeInMain;

    const float statusH = statusMessage.empty()
        ? 0.0f
        : ui_.wrappedHeight(contentW, statusMessage.c_str(), bodySize);

    // The card is sized before it is drawn, so every row that might not
    // appear has to be asked about here on the same terms the drawing asks -
    // otherwise the sheet comes out taller than what is on it.
    auto* registry = core::Application::getInstance().getExpansionRegistry();
    const bool haveExpansions = registry && !registry->getAllProfiles().empty();

    float advancedH = 0.0f;
    if (advancedOpen_) {
        advancedH = px(kRowGap) + px(2);                     // the rule above it
        advancedH += fieldRow + px(kRowGap);                 // saved servers
        advancedH += fieldRow + px(kRowGap);                 // address and port
        if (haveExpansions) {
            advancedH += fieldRow + px(kRowGap);             // expansion
            advancedH += fieldRow + px(kRowGap);             // assets
            if (!assetProfileId_.empty()) advancedH += smallRow + px(4);
        } else {
            advancedH += smallRow + px(kRowGap);             // the one line instead
        }
        if (codeInAdvanced) advancedH += fieldRow + px(kRowGap);
    }

    // The title, its rule and the gap under it, measured once and laid out from
    // the same number below. Measuring it as the em while drawing it as the
    // ink left the card short by a third of a title's height, and everything
    // that sits at the bottom of it - the graphics button, the close button -
    // was drawn over the sheet's own bottom edge.
    const float titleBlockH = ui_.inkHeight(px(kTitleSize), true) * 1.02f + px(18);

    float contentH = titleBlockH;                            // title and its underline
    contentH += fieldRow + px(kRowGap);                      // account
    contentH += fieldRow + px(kRowGap);                      // password
    if (codeInMain) contentH += fieldRow + px(kRowGap);
    if (statusH > 0.0f) contentH += statusH + px(10);
    contentH += px(kButtonHeight) + px(kRowGap);
    contentH += smallRow + px(kRowGap);                      // the disclosure link
    contentH += advancedH;
    contentH += px(10) + smallRow;                           // the footer rule and row

    const float cardW = px(kCardWidth);
    const float cardH = px(kCardPadTop) + contentH + px(kCardPadBottom);
    const ImVec2 cardA((screenW - cardW) * 0.5f,
                       std::max(px(12), (screenH - cardH) * 0.48f));
    const ImVec2 cardB(cardA.x + cardW, cardA.y + cardH);

    ui_.sheet(cardA, cardB);
    ui_.claimMouse(cardA, cardB);

    Column col{cardA.x + px(kCardPadX), cardB.x - px(kCardPadX), cardA.y + px(kCardPadTop)};
    const float centreX = (cardA.x + cardB.x) * 0.5f;

    // ---- title ----------------------------------------------------------
    {
        const float titleSize = px(kTitleSize);
        const float top = col.y;
        ui_.textCentered(centreX, top, "WoWee", titleSize, theme.ink, /*titleFace=*/true);
        const float titleW = ui_.textWidth("WoWee", titleSize, true);
        // Below the title's own descenders, which is further down than the
        // size names: a size is an em and the face draws taller than one.
        const float underlineY = top + ui_.inkHeight(titleSize, true) * 1.02f;
        ui_.squiggle(ImVec2(centreX - titleW * 0.62f, underlineY),
                     ImVec2(centreX + titleW * 0.62f, underlineY),
                     theme.crayonRed, px(2.0f), 0x51A1u);
        // The height the card was measured with, so the two cannot drift.
        col.y = top + titleBlockH;
    }

    // ---- credentials ----------------------------------------------------
    const auto labelledField = [&](const char* label, const char* id, TextEdit& edit,
                                   const PaperUI::FieldOpts& opts) {
        ui_.text(col.at(), label, labelSize, theme.inkSoft);
        col.gap(labelRow);
        const auto [a, b] = col.row(px(kFieldHeight));
        const PaperUI::FieldResult r = ui_.field(id, a, b, edit, opts);
        col.gap(px(kRowGap));
        return r;
    };

    bool submit = false;

    {
        PaperUI::FieldOpts opts;
        opts.placeholder = "account name";
        const PaperUI::FieldResult r = labelledField("Account", "account", username_, opts);
        if (r.changed) statusProminent = false;
        if (r.submitted) ui_.focus("password");
    }

    {
        // The eye sits on the label row, right-aligned, so the box itself is
        // the same shape as the one above it.
        ui_.text(col.at(), "Password", labelSize, theme.inkSoft);
        const float eyeR = labelSize * 0.72f;
        if (ui_.glyphButton("reveal", ImVec2(col.x1 - eyeR, col.y + labelSize * 0.5f), eyeR,
                            showPassword ? PaperUI::Glyph::Eye : PaperUI::Glyph::EyeClosed)) {
            showPassword = !showPassword;
        }
        col.gap(labelRow);

        const bool hadPlaceholder = usingStoredHash && password_.text() == PASSWORD_PLACEHOLDER;
        const auto [a, b] = col.row(px(kFieldHeight));
        PaperUI::FieldOpts opts;
        opts.password = !showPassword;
        opts.placeholder = "password";
        const PaperUI::FieldResult r = ui_.field("password", a, b, password_, opts);
        col.gap(px(kRowGap));

        // A remembered password is a stored hash behind eight placeholder
        // bytes; there is nothing in the box worth editing. Taking the
        // keyboard therefore offers the whole of it up for replacement, so
        // the first keystroke starts a new password rather than appending to
        // a value that is not one.
        if (hadPlaceholder && r.focused && !passwordFocused_) password_.selectAll();
        passwordFocused_ = r.focused;

        if (r.changed) statusProminent = false;
        if (r.submitted) submit = true;
    }

    if (codeInMain) {
        ui_.text(col.at(), "Security code", labelSize, theme.inkSoft);
        ui_.textRight(col.x1, col.y + (labelSize - smallSize) * 0.5f,
                      authState == auth::AuthState::AUTHENTICATOR_REQUIRED ? "authenticator"
                                                                           : "PIN",
                      smallSize, theme.pencil);
        col.gap(labelRow);
        const auto [a, b] = col.row(px(kFieldHeight));
        PaperUI::FieldOpts opts;
        opts.password = true;
        opts.digitsOnly = true;
        opts.placeholder = "code";
        const PaperUI::FieldResult r = ui_.field("pin", a, b, pinCode_, opts);
        col.gap(px(kRowGap));
        if (r.submitted) submit = true;

        if (!securityPromptFocused_) {
            ui_.focus("pin");
            securityPromptFocused_ = true;
        }
    }

    // ---- status ---------------------------------------------------------
    if (statusH > 0.0f) {
        ui_.wrapped(col.at(), contentW, statusMessage.c_str(), bodySize,
                    statusIsError ? theme.crayonRed : theme.crayonGreen);
        col.gap(statusH + px(10));
    }

    // ---- the one button --------------------------------------------------
    {
        const float w = px(200);
        const auto [a, b] = col.cell((contentW - w) * 0.5f, w, px(kButtonHeight));
        col.gap(px(kRowGap));

        char label[64];
        bool enabled = true;
        if (codeInMain) {
            std::snprintf(label, sizeof(label), "Submit Code");
        } else if (authenticating) {
            std::snprintf(label, sizeof(label), "Connecting  %.0fs", authTimer);
            enabled = false;
        } else {
            std::snprintf(label, sizeof(label), "Log In");
        }
        if (ui_.button("login", a, b, label, PaperUI::ButtonKind::Primary, enabled)) submit = true;
    }

    // ---- the disclosure --------------------------------------------------
    {
        const char* label = advancedOpen_ ? "fewer options" : "more options";
        const float w = ui_.textWidth(label, smallSize);
        if (ui_.link("advanced", ImVec2(centreX - w * 0.5f, col.y), label, smallSize)) {
            advancedOpen_ = !advancedOpen_;
        }
        col.gap(smallRow + px(kRowGap));
    }

    // ---- server, expansion, assets ---------------------------------------
    if (advancedOpen_) {
        ui_.rule(ImVec2(col.x0, col.y), ImVec2(col.x1, col.y), paperFade(theme.pencil, 0.6f),
                 px(1.0f), 0x2C71u);
        col.gap(px(kRowGap) + px(2));

        auto& app = core::Application::getInstance();

        // Saved servers.
        {
            ui_.text(col.at(), "Realm", labelSize, theme.inkSoft);
            col.gap(labelRow);
            std::vector<std::string> rows;
            rows.reserve(servers_.size() + 1);
            rows.emplace_back("Somewhere else...");
            for (const auto& s : servers_) {
                std::string row = makeServerKey(s.hostname, s.port);
                if (!s.username.empty()) row += "   " + s.username;
                rows.push_back(std::move(row));
            }
            std::string preview = (selectedServerIndex_ >= 0 &&
                                   selectedServerIndex_ < static_cast<int>(servers_.size()))
                ? makeServerKey(servers_[selectedServerIndex_].hostname,
                                servers_[selectedServerIndex_].port)
                : makeServerKey(hostname_.text(), port) + "   (not saved)";

            int choice = selectedServerIndex_ + 1;  // row 0 is "somewhere else"
            const auto [a, b] = col.row(px(kFieldHeight));
            if (ui_.dropdown("servers", a, b, preview, rows, &choice)) {
                if (choice == 0) selectedServerIndex_ = -1;
                else selectServerProfile(choice - 1);
            }
            col.gap(px(kRowGap));
        }

        // Address and port, on one row, because they are one address.
        {
            const float portW = px(84);
            const float gap = px(12);
            const float hostW = contentW - portW - gap;
            ui_.text(col.at(), "Address", labelSize, theme.inkSoft);
            ui_.text(ImVec2(col.x0 + hostW + gap, col.y), "Port", labelSize, theme.inkSoft);
            col.gap(labelRow);

            const float y = col.y;
            PaperUI::FieldOpts hostOpts;
            hostOpts.placeholder = "logon.example.com";
            const PaperUI::FieldResult h = ui_.field(
                "hostname", ImVec2(col.x0, y), ImVec2(col.x0 + hostW, y + px(kFieldHeight)),
                hostname_, hostOpts);

            PaperUI::FieldOpts portOpts;
            portOpts.digitsOnly = true;
            portOpts.placeholder = "3724";
            const PaperUI::FieldResult p = ui_.field(
                "port", ImVec2(col.x1 - portW, y), ImVec2(col.x1, y + px(kFieldHeight)),
                portText_, portOpts);
            col.gap(px(kFieldHeight) + px(kRowGap));

            if (h.changed || p.changed) selectedServerIndex_ = -1;
            if (h.submitted || p.submitted) submit = true;
            // An empty box is the default port rather than an error: it is a
            // field halfway through being retyped, and there is only one port
            // anyone means by leaving it blank.
            port = portText_.empty()
                ? 3724
                : std::clamp(std::atoi(portText_.c_str()), 1, 65535);
        }

        // Expansion.
        if (haveExpansions) {
            const auto& profiles = registry->getAllProfiles();

            ui_.text(col.at(), "Expansion", labelSize, theme.inkSoft);
            col.gap(labelRow);
            std::vector<std::string> rows;
            rows.reserve(profiles.size());
            for (const auto& p : profiles)
                rows.push_back(p.shortName + "  (" + p.versionString() + ")");
            const std::string preview =
                (expansionIndex >= 0 && expansionIndex < static_cast<int>(profiles.size()))
                    ? rows[static_cast<size_t>(expansionIndex)]
                    : std::string("choose one");
            {
                const auto [a, b] = col.row(px(kFieldHeight));
                if (ui_.dropdown("expansion", a, b, preview, rows, &expansionIndex)) {
                    registry->setActive(profiles[static_cast<size_t>(expansionIndex)].id);
                    app.reloadExpansionData();
                }
                col.gap(px(kRowGap));
            }

            // Assets. The rows are built alongside the ids they stand for,
            // because only some expansions have data on disk and the index
            // the dropdown reports means nothing without them.
            const auto* protocolProfile = registry->getActive();
            std::vector<std::string> rows2;
            std::vector<std::string> ids;
            rows2.emplace_back(protocolProfile
                                   ? "Match protocol  (" + protocolProfile->shortName + ")"
                                   : "Match protocol");
            ids.emplace_back();
            for (const auto& candidate : profiles) {
                if (!std::filesystem::exists(candidate.dataPath + "/manifest.json")) continue;
                rows2.push_back(candidate.shortName + " assets");
                ids.push_back(candidate.id);
            }
            const char* dataPathEnv = std::getenv("WOW_DATA_PATH");
            const std::filesystem::path baseData = dataPathEnv ? dataPathEnv : "./Data";
            if (std::filesystem::exists(baseData / "manifest.json")) {
                rows2.emplace_back("Legacy root Data");
                ids.emplace_back("legacy");
            }

            int assetChoice = 0;
            for (int i = 0; i < static_cast<int>(ids.size()); ++i)
                if (ids[static_cast<size_t>(i)] == assetProfileId_) { assetChoice = i; break; }

            ui_.text(col.at(), "Assets", labelSize, theme.inkSoft);
            col.gap(labelRow);
            {
                const auto [a, b] = col.row(px(kFieldHeight));
                if (ui_.dropdown("assets", a, b, rows2[static_cast<size_t>(assetChoice)], rows2,
                                 &assetChoice)) {
                    assetProfileId_ = ids[static_cast<size_t>(assetChoice)];
                    app.setAssetExpansionOverride(assetProfileId_);
                    app.reloadExpansionData();
                }
                col.gap(px(kRowGap));
            }
            if (!assetProfileId_.empty()) {
                ui_.text(col.at(), "Cross-expansion DBC and model formats may differ.", smallSize,
                         theme.pencil);
                col.gap(smallRow + px(4));
            }
        } else {
            ui_.text(col.at(), "Expansion: WotLK 3.3.5a (default)", smallSize, theme.pencil);
            col.gap(smallRow + px(kRowGap));
        }

        if (codeInAdvanced) {
            ui_.text(col.at(), "Security code", labelSize, theme.inkSoft);
            ui_.textRight(col.x1, col.y + (labelSize - smallSize) * 0.5f, "if your realm asks",
                          smallSize, theme.pencil);
            col.gap(labelRow);
            const auto [a, b] = col.row(px(kFieldHeight));
            PaperUI::FieldOpts opts;
            opts.password = true;
            opts.digitsOnly = true;
            opts.placeholder = "PIN or authenticator";
            if (ui_.field("pin", a, b, pinCode_, opts).submitted) submit = true;
            col.gap(px(kRowGap));
        }
    }

    // ---- footer ----------------------------------------------------------
    {
        col.gap(px(10));
        ui_.rule(ImVec2(col.x0, col.y - px(6)), ImVec2(col.x1, col.y - px(6)),
                 paperFade(theme.pencil, 0.5f), px(1.0f), 0x77A3u);

        std::string where = makeServerKey(hostname_.text(), port);
        if (auto* registry = core::Application::getInstance().getExpansionRegistry())
            if (const auto* active = registry->getActive())
                where += "  \xc2\xb7  " + active->shortName;
        ui_.text(col.at(), where.c_str(), smallSize, theme.pencil);

        const float r = smallSize * 0.86f;
        const float cy = col.y + smallRow * 0.5f - px(1);
        if (ui_.glyphButton("quit", ImVec2(col.x1 - r, cy), r, PaperUI::Glyph::Cross)) {
            if (auto* window = core::Application::getInstance().getWindow())
                window->setShouldClose(true);
        }
        if (ui_.glyphButton("settings", ImVec2(col.x1 - r * 3.4f, cy), r,
                            PaperUI::Glyph::Gear)) {
            settingsOpen_ = true;
            loginGfxLoaded_ = false;  // reload from disk each time it opens
            // The sheet lands under the pointer, and one of its controls is
            // wherever this gear is. Without this the press carries into it:
            // at 1280x760 the gear sits on the ground-clutter slider, which
            // takes the value it was dropped on and drags while held.
            ui_.swallowPress();
        }
    }

    if (submit && !authenticating) {
        attemptAuth(authHandler);
    } else if (submit && waitingForSecurityCode) {
        const std::string code = trimAscii(pinCode_.text());
        if (!code.empty()) {
            authHandler.submitSecurityCode(code);
            pinCode_.clear();
            pinAutoSubmitted_ = true;
        }
    }
}

// ---------------------------------------------------------------------------
// Background art and login music
// ---------------------------------------------------------------------------

// The art is drawn straight into ImGui's background list rather than through
// PaperUI: it is behind everything by definition, and the realm and character
// screens ask for it before they have begun a frame of their own.

void AuthScreen::drawBackdrop() {
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
                stbi_image_free(data);
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
    if (!bgDescriptorSet) return;

    const ImVec2 screen = ImGui::GetIO().DisplaySize;
    const float imgW = static_cast<float>(bgWidth);
    const float imgH = static_cast<float>(bgHeight);
    if (imgW <= 0.0f || imgH <= 0.0f) return;

    const float screenAspect = screen.x / screen.y;
    const float imgAspect = imgW / imgH;
    ImVec2 uv0(0.0f, 0.0f);
    ImVec2 uv1(1.0f, 1.0f);
    if (imgAspect > screenAspect) {
        const float crop = (1.0f - screenAspect / imgAspect) * 0.5f;
        uv0.x = crop;
        uv1.x = 1.0f - crop;
    } else if (imgAspect < screenAspect) {
        const float crop = (1.0f - imgAspect / screenAspect) * 0.5f;
        uv0.y = crop;
        uv1.y = 1.0f - crop;
    }
    ImGui::GetBackgroundDrawList()->AddImage(reinterpret_cast<ImTextureID>(bgDescriptorSet),
                                             ImVec2(0, 0), ImVec2(screen.x, screen.y), uv0, uv1);
}

void AuthScreen::updateMusic() {
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
    if (!ac) return;
    auto* music = ac->getMusicManager();
    if (!music) return;

    if (!loginMusicVolumeAdjusted_) {
        savedMusicVolume_ = music->getVolume();
        // Reduce music to 80% during login so UI button clicks and error sounds
        // remain audible over the background track
        int loginVolume = (savedMusicVolume_ * 80) / 100;
        loginVolume = std::clamp(loginVolume, 0, 100);
        music->setVolume(loginVolume);
        loginMusicVolumeAdjusted_ = true;
    }
    music->update(ImGui::GetIO().DeltaTime);
    if (music->isPlaying() || music->isLoading()) return;

    if (!introTracksScanned_) {
        introTracksScanned_ = true;
        const std::filesystem::path relative =
            std::filesystem::path("assets") / "Original Music" / "TavernAllianceREMIX.mp3";
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
    if (username_.empty()) {
        setStatus("Enter an account name", true);
        return;
    }

    // Check if using stored hash (password field contains placeholder)
    const bool useHash = usingStoredHash && password_.text() == PASSWORD_PLACEHOLDER;

    if (!useHash && password_.empty()) {
        setStatus("Enter a password", true);
        return;
    }

    if (hostname_.empty()) {
        setStatus("Enter an address to connect to", true);
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
    const bool useHash = usingStoredHash && password_.text() == PASSWORD_PLACEHOLDER;
    const uint8_t protocolVersion = (authProtocolAttempt_ < authProtocols_.size())
        ? authProtocols_[authProtocolAttempt_] : uint8_t{8};
    const bool isRetry = (authProtocolAttempt_ > 0);

    std::stringstream ss;
    if (isRetry) {
        ss << "Retrying " << hostname_.text() << ":" << port
           << " with auth protocol " << static_cast<int>(protocolVersion) << "...";
    } else {
        ss << "Connecting to " << hostname_.text() << ":" << port << "...";
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

    if (authHandler.connect(hostname_.text(), static_cast<uint16_t>(port))) {
        authenticating = true;
        authTimer = 0.0f;
        setStatus(isRetry ? "Reconnected, authenticating..." : "Connected, authenticating...", false);
        pinAutoSubmitted_ = false;
        securityPromptFocused_ = false;

        // Save login info for next session
        saveLoginInfo(false);

        const std::string pinStr = trimAscii(pinCode_.text());

        // Send authentication credentials
        if (useHash) {
            auto hashBytes = hexDecode(savedPasswordHash);
            authHandler.authenticateWithHash(username_.text(), hashBytes, pinStr);
        } else {
            usingStoredHash = false;
            authHandler.authenticate(username_.text(), password_.text(), pinStr);
        }

        // Don't keep the code around longer than needed.
        pinCode_.clear();
    } else {
        std::stringstream errSs;
        errSs << "Failed to connect to " << hostname_.text() << ":" << port
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
    out << "active=" << makeServerKey(hostname_.text(), port) << "\n";

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
        VkImageMemoryBarrier2 barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
        barrier.srcStageMask = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
        barrier.dstStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;
        barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = bgImage;
        barrier.subresourceRange = {.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .baseMipLevel = 0, .levelCount = 1, .baseArrayLayer = 0, .layerCount = 1};
        barrier.srcAccessMask = 0;
        barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        VkDependencyInfo barrierDep{.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
        barrierDep.dependencyFlags = 0;
        barrierDep.imageMemoryBarrierCount = 1;
        barrierDep.pImageMemoryBarriers = &barrier;
        rendering::cmdPipelineBarrier2(cmd, barrierDep);

        VkBufferImageCopy region{};
        region.imageSubresource = {.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .mipLevel = 0, .baseArrayLayer = 0, .layerCount = 1};
        region.imageExtent = {.width = static_cast<uint32_t>(bgWidth), .height = static_cast<uint32_t>(bgHeight), .depth = 1};
        vkCmdCopyBufferToImage(cmd, stagingBuffer, bgImage,
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

        barrier.srcStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;
        barrier.dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        VkDependencyInfo toReadDep{.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
        toReadDep.imageMemoryBarrierCount = 1;
        toReadDep.pImageMemoryBarriers = &barrier;
        rendering::cmdPipelineBarrier2(cmd, toReadDep);
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
    s.grass          = p.grass;
    s.grassDensity   = p.grassDensity;
    s.grassHeight    = p.grassHeight;
    s.grassDistance  = p.grassDistance;

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
        else if (key == "grass_enabled")    loginGfx_.grass          = (val == "1");
        else if (key == "grass_density")    loginGfx_.grassDensity   = clampI(std::stoi(val), 0, 300);
        else if (key == "grass_height")     loginGfx_.grassHeight    = clampI(std::stoi(val), 50, 300);
        else if (key == "grass_distance")   loginGfx_.grassDistance  = clampI(std::stoi(val), 30, 800);
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
    cfg["grass_enabled"]         = loginGfx_.grass ? "1" : "0";
    cfg["grass_density"]         = std::to_string(loginGfx_.grassDensity);
    cfg["grass_height"]          = std::to_string(loginGfx_.grassHeight);
    cfg["grass_distance"]        = std::to_string(loginGfx_.grassDistance);
    cfg["brightness"]            = std::to_string(loginGfx_.brightness);
    cfg["vsync"]                 = loginGfx_.vsync           ? "1" : "0";
    cfg["fullscreen"]            = loginGfx_.fullscreen      ? "1" : "0";

    // Write everything back.
    std::ofstream out(SettingsPanel::getSettingsPath());
    if (!out.is_open()) return;
    for (const auto& [k, v] : cfg)
        out << k << "=" << v << "\n";
}

void AuthScreen::renderLoginSettingsSheet(float screenW, float screenH) {
    if (!loginGfxLoaded_) {
        loadLoginGraphicsState();
        loginGfxLoaded_ = true;
    }

    const PaperTheme& theme = ui_.theme();
    const auto px = [this](float units) { return ui_.px(units); };

    constexpr float kPad = 30.0f;
    constexpr float kTitle = 30.0f;
    constexpr float kLabel = 13.5f;
    constexpr float kSmall = 12.5f;
    constexpr float kControl = 34.0f;
    constexpr float kCheckBox = 19.0f;
    constexpr float kCheckStep = 30.0f;
    constexpr float kButton = 42.0f;

    const float labelRow = ui_.lineHeight(px(kLabel)) + px(3);
    const float controlRow = labelRow + px(kControl);
    const float smallRow = ui_.lineHeight(px(kSmall));

    // Seven switches on the left, six things with a range on the right. The
    // taller of the two columns is what the sheet has to be tall enough for.
    const float leftH = 7.0f * px(kCheckStep);
    const float rightH = 6.0f * (controlRow + px(8));
    const float contentH = px(kTitle) * 1.05f + smallRow + px(16)   // title block
                         + controlRow + px(20)                       // preset
                         + std::max(leftH, rightH) + px(22)
                         + px(kButton);
    const float w = px(560);
    const float h = px(kPad) * 2.0f + contentH;

    ui_.setLayer(PaperLayer::Overlay);
    ui_.scrim(0.45f);

    const ImVec2 a((screenW - w) * 0.5f, std::max(px(10), (screenH - h) * 0.5f));
    const ImVec2 b(a.x + w, a.y + h);
    ui_.sheet(a, b, /*taped=*/false);
    // The sheet eats every click that lands on it, including the ones that
    // land on none of its controls.
    ui_.claimMouse(a, b);

    Column col{a.x + px(kPad), b.x - px(kPad), a.y + px(kPad)};

    ui_.text(col.at(), "Graphics", px(kTitle), theme.ink, /*titleFace=*/true);
    {
        const float r = px(kSmall) * 0.95f;
        if (ui_.glyphButton("gfx.close", ImVec2(col.x1 - r, col.y + r), r,
                            PaperUI::Glyph::Cross)) {
            settingsOpen_ = false;
        }
    }
    col.gap(px(kTitle) * 1.05f);
    ui_.text(col.at(), "These take effect the next time you log in.", px(kSmall), theme.pencil);
    col.gap(smallRow + px(8));
    ui_.rule(ImVec2(col.x0, col.y), ImVec2(col.x1, col.y), paperFade(theme.pencil, 0.6f),
             px(1.0f), 0x9A17u);
    col.gap(px(8));

    // ---- preset -----------------------------------------------------------
    {
        ui_.text(col.at(), "Preset", px(kLabel), theme.inkSoft);
        col.gap(labelRow);
        const std::vector<std::string> names = {"Custom", "Low", "Medium", "High", "Ultra"};
        int preset = std::clamp(loginGfx_.preset, 0, static_cast<int>(names.size()) - 1);
        const auto [pa, pb] = col.cell(0.0f, px(220), px(kControl));
        if (ui_.dropdown("gfx.preset", pa, pb, names[static_cast<size_t>(preset)], names,
                         &preset)) {
            loginGfx_.preset = preset;
            // Custom is not a set of values, it is the absence of one; picking
            // it leaves everything where the player put it.
            if (preset != 0) applyPresetToState(loginGfx_, preset);
        }
        col.gap(px(20));
    }

    const float columnGap = px(30);
    const float columnW = (col.width() - columnGap) * 0.5f;
    const float columnTop = col.y;

    // ---- the switches -----------------------------------------------------
    {
        Column left{col.x0, col.x0 + columnW, columnTop};
        const auto toggle = [&](const char* id, const char* label, bool* value) {
            ui_.checkbox(id, ImVec2(left.x0, left.y), px(kCheckBox), label, value);
            left.gap(px(kCheckStep));
        };
        toggle("gfx.shadows", "Shadows", &loginGfx_.shadows);
        toggle("gfx.fxaa", "FXAA", &loginGfx_.fxaa);
        toggle("gfx.normals", "Normal mapping", &loginGfx_.normalMapping);
        toggle("gfx.pom", "Parallax occlusion", &loginGfx_.pom);
        toggle("gfx.water", "Water refraction", &loginGfx_.waterRefraction);
        toggle("gfx.vsync", "V-Sync", &loginGfx_.vsync);
        toggle("gfx.fullscreen", "Fullscreen", &loginGfx_.fullscreen);
    }

    // ---- the ranges -------------------------------------------------------
    {
        Column right{col.x1 - columnW, col.x1, columnTop};
        const auto labelled = [&](const char* label) {
            ui_.text(right.at(), label, px(kLabel), theme.inkSoft);
            right.gap(labelRow);
            return right.row(px(kControl));
        };

        {
            const auto [ra, rb] = labelled("Anti-aliasing");
            // The same four the settings schema offers. This list once had
            // three, so 8x could be set in the game, never shown here, and
            // silently rewritten by anything picked here instead.
            const std::vector<std::string> names = {"Off", "2x MSAA", "4x MSAA", "8x MSAA"};
            int aa = std::clamp(loginGfx_.antiAliasing, 0, 3);
            if (ui_.dropdown("gfx.aa", ra, rb, names[static_cast<size_t>(aa)], names, &aa))
                loginGfx_.antiAliasing = aa;
            right.gap(px(8));
        }
        {
            const auto [ra, rb] = labelled("Parallax quality");
            // The names and the count both come from the one scale, so this
            // cannot go back to offering two entries of a three-entry setting
            // and writing the wrong index for the ones it did offer.
            std::vector<std::string> names;
            for (int i = 0; i < rendering::kPomQualityCount; ++i)
                names.emplace_back(rendering::kPomQualityLabels[i]);
            int quality = std::clamp(loginGfx_.pomQuality, 0, rendering::kPomQualityCount - 1);
            if (ui_.dropdown("gfx.pomq", ra, rb, names[static_cast<size_t>(quality)], names,
                             &quality))
                loginGfx_.pomQuality = quality;
            right.gap(px(8));
        }
        {
            const auto [ra, rb] = labelled("Shadow distance");
            ui_.sliderFloat("gfx.shadowdist", ra, rb, &loginGfx_.shadowDistance, 50.0f, 600.0f);
            right.gap(px(8));
        }
        {
            const auto [ra, rb] = labelled("View distance");
            ui_.sliderFloat("gfx.viewdist", ra, rb, &loginGfx_.viewDistance, 400.0f, 2400.0f);
            right.gap(px(8));
        }
        {
            // 150, not 200: GameScreen::loadSettings clamps there, so the last
            // fifty units of this slider were silently discarded at login.
            const auto [ra, rb] = labelled("Ground clutter");
            ui_.sliderInt("gfx.clutter", ra, rb, &loginGfx_.groundClutter, 0, 150);
            right.gap(px(8));
        }
        {
            const auto [ra, rb] = labelled("Brightness");
            ui_.sliderInt("gfx.brightness", ra, rb, &loginGfx_.brightness, 0, 100);
            right.gap(px(8));
        }
    }

    col.y = columnTop + std::max(leftH, rightH) + px(22);

    // ---- the way out ------------------------------------------------------
    {
        const float y = col.y;
        const float bh = px(kButton);
        if (ui_.button("gfx.reset", ImVec2(col.x0, y), ImVec2(col.x0 + px(170), y + bh),
                       "Reset to Medium")) {
            applyPresetToState(loginGfx_, 2);
            loginGfx_.preset = 2;
        }
        const float applyW = px(130);
        const float cancelW = px(110);
        if (ui_.button("gfx.cancel", ImVec2(col.x1 - applyW - px(12) - cancelW, y),
                       ImVec2(col.x1 - applyW - px(12), y + bh), "Cancel")) {
            settingsOpen_ = false;
        }
        if (ui_.button("gfx.apply", ImVec2(col.x1 - applyW, y), ImVec2(col.x1, y + bh), "Apply",
                       PaperUI::ButtonKind::Primary)) {
            saveLoginGraphicsState();
            if (services_.window && services_.window->isVsyncEnabled() != loginGfx_.vsync) {
                services_.window->setVsync(loginGfx_.vsync);
            }
            settingsOpen_ = false;
        }
    }

    ui_.setLayer(PaperLayer::Page);
}

}} // namespace wowee::ui
