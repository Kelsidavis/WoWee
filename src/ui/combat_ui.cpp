// ============================================================
// CombatUI - extracted from GameScreen
// Owns all combat-related UI rendering: cast bar, cooldown tracker,
// raid warning overlay, combat text, DPS meter, buff bar,
// battleground score HUD, combat log, threat window, BG scoreboard.
// ============================================================
#include "core/local_time.hpp"
#include "ui/combat_ui.hpp"
#include "addons/lua_api_registrations.hpp"
#include "ui/buff_bar_layout.hpp"
#include "ui/framexml_takeover.hpp"
#include "ui/settings_panel.hpp"
#include "ui/spellbook_screen.hpp"
#include "ui/inventory_screen.hpp"
#include "ui/ui_colors.hpp"
#include "ui/ui_helpers.hpp"
#include "core/application.hpp"
#include "core/logger.hpp"
#include "core/coordinates.hpp"
#include "rendering/renderer.hpp"
#include "rendering/camera.hpp"
#include "game/game_handler.hpp"
#include "game/bg_score_defs.hpp"
#include "pipeline/asset_manager.hpp"
#include "audio/audio_coordinator.hpp"
#include "audio/audio_engine.hpp"
#include "audio/ui_sound_manager.hpp"
#include <imgui.h>
#include <imgui_internal.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace {
    using namespace wowee::ui::colors;
    using namespace wowee::ui::helpers;
    constexpr auto& kColorRed         = kRed;
    constexpr auto& kColorGreen       = kGreen;
    constexpr auto& kColorBrightGreen = kBrightGreen;
    constexpr auto& kColorYellow      = kYellow;
} // anonymous namespace

namespace wowee {
namespace ui {


// ============================================================
// Cast Bar
// ============================================================


// ============================================================
// Cooldown Tracker - floating panel showing all active spell CDs
// ============================================================

void CombatUI::renderCooldownTracker(game::GameHandler& gameHandler,
                                     const SettingsPanel& settings,
                                     const SpellIconFn& getSpellIcon) {
    if (!settings.showCooldownTracker_) return;

    const auto& cooldowns = gameHandler.getSpellCooldowns();
    if (cooldowns.empty()) return;

    // Collect spells with remaining cooldown > 0.5s (skip GCD noise)
    struct CDEntry { uint32_t spellId; float remaining; };
    std::vector<CDEntry> active;
    active.reserve(16);
    for (const auto& [sid, rem] : cooldowns) {
        if (rem > 0.5f) active.push_back({.spellId = sid, .remaining = rem});
    }
    if (active.empty()) return;

    // Sort: longest remaining first
    std::sort(active.begin(), active.end(), [](const CDEntry& a, const CDEntry& b) {
        return a.remaining > b.remaining;
    });

    auto* assetMgr = services_.assetManager;
    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    constexpr float TRACKER_W = 200.0f;
    constexpr int MAX_SHOWN = 12;
    float posX = screenW * 0.5f + 260.0f;
    float posY = screenH - 290.0f;  // above the action bar area, clear of minimap/quest tracker

    ImGui::SetNextWindowPos(ImVec2(posX, posY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(TRACKER_W, 0.0f), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.75f);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoNav |
                             ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize |
                             ImGuiWindowFlags_NoBringToFrontOnFocus;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(4.0f, 4.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4.0f, 2.0f));

    if (ImGui::Begin("##CooldownTracker", nullptr, flags)) {
        ImGui::TextDisabled("Cooldowns");
        ImGui::Separator();

        int shown = 0;
        for (const auto& cd : active) {
            if (shown >= MAX_SHOWN) break;

            const std::string& name = gameHandler.getSpellName(cd.spellId);
            if (name.empty()) continue;  // skip unnamed spells (internal/passive)

            // Small icon if available
            VkDescriptorSet icon = assetMgr ? getSpellIcon(cd.spellId, assetMgr) : VK_NULL_HANDLE;
            if (icon) {
                ImGui::Image((ImTextureID)(uintptr_t)icon, ImVec2(14, 14));
                ImGui::SameLine(0, 3);
            }

            // Name (truncated) + remaining time
            char timeStr[16];
            if (cd.remaining >= 60.0f)
                snprintf(timeStr, sizeof(timeStr), "%dm%ds", static_cast<int>(cd.remaining) / 60, static_cast<int>(cd.remaining) % 60);
            else
                snprintf(timeStr, sizeof(timeStr), "%.0fs", cd.remaining);

            // Color: red > 30s, orange > 10s, yellow > 5s, green otherwise
            ImVec4 cdColor = cd.remaining > 30.0f ? kColorRed :
                             cd.remaining > 10.0f ? ImVec4(1.0f, 0.6f, 0.2f, 1.0f) :
                             cd.remaining > 5.0f  ? kColorYellow :
                                                    colors::kActiveGreen;

            // Truncate name to fit
            std::string displayName = name;
            if (displayName.size() > 16) displayName = displayName.substr(0, 15) + "\xe2\x80\xa6"; // ellipsis

            ImGui::TextColored(ImVec4(0.9f, 0.9f, 0.9f, 1.0f), "%s", displayName.c_str());
            ImGui::SameLine(TRACKER_W - 48.0f);
            ImGui::TextColored(cdColor, "%s", timeStr);

            ++shown;
        }
    }
    ImGui::End();
    ImGui::PopStyleVar(3);
}


// ============================================================
// Raid Warning / Boss Emote Center-Screen Overlay
// ============================================================

void CombatUI::renderRaidWarningOverlay(game::GameHandler& gameHandler) {
    // FrameXML's RaidWarningFrame and RaidBossEmoteFrame answer
    // CHAT_MSG_RAID_WARNING and CHAT_MSG_RAID_BOSS_EMOTE, both of which this
    // client fires - so when it owns them, collect no entries and the empty
    // check below returns before anything is drawn. The gate is here rather
    // than at the top of the function because the same scan plays the whisper
    // sound, which belongs to no element and has to keep happening.
    const bool ownsWarnings = frameXmlOwns(UiElement::RaidWarning);

    // Scan chat history for new RAID_WARNING / RAID_BOSS_EMOTE messages
    const auto& chatHistory = gameHandler.getChatHistory();
    size_t newCount = chatHistory.size();
    if (newCount > raidWarnChatSeenCount_) {
        // Walk only the new messages (deque - iterate from back by skipping old ones)
        size_t toScan = newCount - raidWarnChatSeenCount_;
        size_t startIdx = newCount > toScan ? newCount - toScan : 0;
        for (size_t i = startIdx; i < newCount; ++i) {
            const auto& msg = chatHistory[i];
            if (!ownsWarnings &&
                (msg.type == game::ChatType::RAID_WARNING ||
                 msg.type == game::ChatType::RAID_BOSS_EMOTE ||
                 msg.type == game::ChatType::MONSTER_EMOTE)) {
                bool isBoss = (msg.type != game::ChatType::RAID_WARNING);
                // Limit display text length to avoid giant overlay
                std::string text = msg.message;
                if (text.size() > 200) text = text.substr(0, 200) + "...";
                raidWarnEntries_.push_back({.text = text, .age = 0.0f, .isBossEmote = isBoss});
                if (raidWarnEntries_.size() > 3)
                    raidWarnEntries_.erase(raidWarnEntries_.begin());
            }
            // Whisper audio notification
            if (msg.type == game::ChatType::WHISPER) {
                if (auto* ac = services_.audioCoordinator) {
                    if (auto* ui = ac->getUiSoundManager())
                        ui->playWhisperReceived();
                }
            }
        }
        raidWarnChatSeenCount_ = newCount;
    }

    // Age and remove expired entries
    float dt = ImGui::GetIO().DeltaTime;
    for (auto& e : raidWarnEntries_) e.age += dt;
    raidWarnEntries_.erase(
        std::remove_if(raidWarnEntries_.begin(), raidWarnEntries_.end(),
            [](const RaidWarnEntry& e){ return e.age >= RaidWarnEntry::LIFETIME; }),
        raidWarnEntries_.end());

    if (raidWarnEntries_.empty()) return;

    ImGuiIO& io = ImGui::GetIO();
    float screenW = io.DisplaySize.x;
    float screenH = io.DisplaySize.y;
    ImDrawList* fg = ImGui::GetForegroundDrawList();

    // Stack entries vertically near upper-center (below target frame area)
    float baseY = screenH * 0.28f;
    for (const auto& e : raidWarnEntries_) {
        float alpha = std::clamp(1.0f - (e.age / RaidWarnEntry::LIFETIME), 0.0f, 1.0f);
        // Fade in quickly, hold, then fade out last 20%
        if (e.age < 0.3f) alpha = e.age / 0.3f;

        // Truncate to fit screen width reasonably
        const char* txt = e.text.c_str();
        const float fontSize = 22.0f;
        ImFont* font = ImGui::GetFont();

        // Word-wrap manually: compute text size, center horizontally
        float maxW = screenW * 0.7f;
        ImVec2 textSz = font->CalcTextSizeA(fontSize, maxW, maxW, txt);
        float tx = (screenW - textSz.x) * 0.5f;

        ImU32 shadowCol = IM_COL32(0, 0, 0, static_cast<int>(alpha * 200));
        ImU32 mainCol;
        if (e.isBossEmote) {
            mainCol = IM_COL32(255, 185, 60, static_cast<int>(alpha * 255));   // amber
        } else {
            // Raid warning: alternating red/yellow flash during first second
            float flashT = std::fmod(e.age * 4.0f, 1.0f);
            if (flashT < 0.5f)
                mainCol = IM_COL32(255, 50, 50, static_cast<int>(alpha * 255));
            else
                mainCol = IM_COL32(255, 220, 50, static_cast<int>(alpha * 255));
        }

        // Background dim box for readability
        float pad = 8.0f;
        fg->AddRectFilled(ImVec2(tx - pad, baseY - pad),
                           ImVec2(tx + textSz.x + pad, baseY + textSz.y + pad),
                           IM_COL32(0, 0, 0, static_cast<int>(alpha * 120)), 4.0f);

        // Shadow + main text
        fg->AddText(font, fontSize, ImVec2(tx + 2.0f, baseY + 2.0f), shadowCol, txt,
                    nullptr, maxW);
        fg->AddText(font, fontSize, ImVec2(tx,         baseY),         mainCol,  txt,
                    nullptr, maxW);

        baseY += textSz.y + 6.0f;
    }
}


// ============================================================
// Floating Combat Text
// ============================================================

// Not gated, and there is no element to gate it on.
//
// FrameXML's own floating combat text lives in Blizzard_CombatText, which is
// load-on-demand and reached from exactly one place: the float-mode dropdown in
// the Interface Options combat panel, which calls UIParentLoadAddOn for it. So
// it is absent until a player touches that setting, and drawing beside it after
// that is a latent double rather than a live one.
//
// Left alone deliberately. Covering it means a new UiElement and a suppression
// entry, which is a decision about what this interface owns rather than a fix
// to something broken - and the client's combat text is the one on screen today.
void CombatUI::renderCombatText(game::GameHandler& gameHandler) {
    const auto& entries = gameHandler.getCombatText();
    if (entries.empty()) return;

    auto* window = services_.window;
    if (!window) return;
    const float screenW = static_cast<float>(window->getWidth());
    const float screenH = static_cast<float>(window->getHeight());

    // Camera for world-space projection
    auto* appRenderer = services_.renderer;
    rendering::Camera* camera = appRenderer ? appRenderer->getCamera() : nullptr;
    glm::mat4 viewProj;
    if (camera) viewProj = camera->getProjectionMatrix() * camera->getViewMatrix();

    ImDrawList* drawList = ImGui::GetForegroundDrawList();
    ImFont* font = ImGui::GetFont();
    const float baseFontSize = ImGui::GetFontSize();

    // HUD fallback: entries without world-space anchor use classic screen-position layout.
    // We still need an ImGui window for those.
    const float hudIncomingX = screenW * 0.40f;
    const float hudOutgoingX = screenW * 0.68f;
    int hudInIdx = 0, hudOutIdx = 0;
    bool needsHudWindow = false;

    // What kind of number the player asked to see.
    //
    // The interface offers eighteen of these and read none of them, so every
    // kind was drawn together. Three cover what this client actually produces;
    // the rest name things it does not draw at all - reputation changes, aura
    // gains, combo points - and there is nothing yet for them to filter.
    //
    // Read once for the frame rather than per entry: a busy fight puts dozens
    // through this loop and the answer cannot change inside it.
    const bool showDamage =
        addons::storedCVarValue("fctDamage", "1") != "0";
    const bool showHealing =
        addons::storedCVarValue("fctHealing", "1") != "0";
    const bool showMisses =
        addons::storedCVarValue("fctDodgeParryMiss", "1") != "0";

    const auto wanted = [&](game::CombatTextEntry::Type t) {
        using T = game::CombatTextEntry;
        switch (t) {
            case T::MELEE_DAMAGE: case T::SPELL_DAMAGE: case T::CRIT_DAMAGE:
            case T::PERIODIC_DAMAGE: case T::ENVIRONMENTAL: case T::GLANCING:
            case T::CRUSHING:
                return showDamage;
            case T::HEAL: case T::CRIT_HEAL: case T::PERIODIC_HEAL:
                return showHealing;
            case T::MISS: case T::DODGE: case T::PARRY: case T::BLOCK:
            case T::EVADE: case T::IMMUNE: case T::DEFLECT: case T::REFLECT:
            case T::RESIST: case T::ABSORB:
                return showMisses;
            default:
                // Everything else - energy, experience, honour, dispels - has
                // no switch of its own here and is left alone rather than
                // hidden by somebody else's.
                return true;
        }
    };

    for (const auto& entry : entries) {
        if (!wanted(entry.type)) continue;
        const float alpha = 1.0f - (entry.age / game::CombatTextEntry::LIFETIME);
        const bool outgoing = entry.isPlayerSource;

        // --- Format text and color (identical logic for both world and HUD paths) ---
        ImVec4 color;
        char text[128];
        switch (entry.type) {
            case game::CombatTextEntry::MELEE_DAMAGE:
            case game::CombatTextEntry::SPELL_DAMAGE:
                snprintf(text, sizeof(text), "-%d", entry.amount);
                color = outgoing ?
                    ImVec4(1.0f, 1.0f, 0.3f, alpha) :
                    ImVec4(1.0f, 0.3f, 0.3f, alpha);
                break;
            case game::CombatTextEntry::CRIT_DAMAGE:
                snprintf(text, sizeof(text), "-%d!", entry.amount);
                color = outgoing ?
                    ImVec4(1.0f, 0.8f, 0.0f, alpha) :
                    ImVec4(1.0f, 0.5f, 0.0f, alpha);
                break;
            case game::CombatTextEntry::HEAL:
                snprintf(text, sizeof(text), "+%d", entry.amount);
                color = ImVec4(0.3f, 1.0f, 0.3f, alpha);
                break;
            case game::CombatTextEntry::CRIT_HEAL:
                snprintf(text, sizeof(text), "+%d!", entry.amount);
                color = ImVec4(0.3f, 1.0f, 0.3f, alpha);
                break;
            case game::CombatTextEntry::MISS:
                snprintf(text, sizeof(text), "Miss");
                color = ImVec4(0.7f, 0.7f, 0.7f, alpha);
                break;
            case game::CombatTextEntry::DODGE:
                snprintf(text, sizeof(text), outgoing ? "Dodge" : "You Dodge");
                color = outgoing ? ImVec4(0.6f, 0.6f, 0.6f, alpha)
                                 : ImVec4(0.4f, 0.9f, 1.0f, alpha);
                break;
            case game::CombatTextEntry::PARRY:
                snprintf(text, sizeof(text), outgoing ? "Parry" : "You Parry");
                color = outgoing ? ImVec4(0.6f, 0.6f, 0.6f, alpha)
                                 : ImVec4(0.4f, 0.9f, 1.0f, alpha);
                break;
            case game::CombatTextEntry::BLOCK:
                if (entry.amount > 0)
                    snprintf(text, sizeof(text), outgoing ? "Block %d" : "You Block %d", entry.amount);
                else
                    snprintf(text, sizeof(text), outgoing ? "Block" : "You Block");
                color = outgoing ? ImVec4(0.6f, 0.6f, 0.6f, alpha)
                                 : ImVec4(0.4f, 0.9f, 1.0f, alpha);
                break;
            case game::CombatTextEntry::EVADE:
                snprintf(text, sizeof(text), outgoing ? "Evade" : "You Evade");
                color = outgoing ? ImVec4(0.6f, 0.6f, 0.6f, alpha)
                                 : ImVec4(0.4f, 0.9f, 1.0f, alpha);
                break;
            case game::CombatTextEntry::PERIODIC_DAMAGE:
                snprintf(text, sizeof(text), "-%d", entry.amount);
                color = outgoing ?
                    ImVec4(1.0f, 0.9f, 0.3f, alpha) :
                    ImVec4(1.0f, 0.4f, 0.4f, alpha);
                break;
            case game::CombatTextEntry::PERIODIC_HEAL:
                snprintf(text, sizeof(text), "+%d", entry.amount);
                color = ImVec4(0.4f, 1.0f, 0.5f, alpha);
                break;
            case game::CombatTextEntry::ENVIRONMENTAL: {
                const char* envLabel = "";
                switch (entry.powerType) {
                    case 0: envLabel = "Fatigue "; break;
                    case 1: envLabel = "Drowning "; break;
                    case 2: envLabel = ""; break;
                    case 3: envLabel = "Lava "; break;
                    case 4: envLabel = "Slime "; break;
                    case 5: envLabel = "Fire "; break;
                    default: envLabel = ""; break;
                }
                snprintf(text, sizeof(text), "%s-%d", envLabel, entry.amount);
                color = ImVec4(0.9f, 0.5f, 0.2f, alpha);
                break;
            }
            case game::CombatTextEntry::ENERGIZE:
                snprintf(text, sizeof(text), "+%d", entry.amount);
                switch (entry.powerType) {
                    case 1:  color = ImVec4(1.0f, 0.2f, 0.2f, alpha); break;
                    case 2:  color = ImVec4(1.0f, 0.6f, 0.1f, alpha); break;
                    case 3:  color = ImVec4(1.0f, 0.9f, 0.2f, alpha); break;
                    case 6:  color = ImVec4(0.3f, 0.9f, 0.8f, alpha); break;
                    default: color = ImVec4(0.3f, 0.6f, 1.0f, alpha); break;
                }
                break;
            case game::CombatTextEntry::POWER_DRAIN:
                snprintf(text, sizeof(text), "-%d", entry.amount);
                switch (entry.powerType) {
                    case 1:  color = ImVec4(1.0f, 0.35f, 0.35f, alpha); break;
                    case 2:  color = ImVec4(1.0f, 0.7f, 0.2f, alpha); break;
                    case 3:  color = ImVec4(1.0f, 0.95f, 0.35f, alpha); break;
                    case 6:  color = ImVec4(0.45f, 0.95f, 0.85f, alpha); break;
                    default: color = ImVec4(0.45f, 0.75f, 1.0f, alpha); break;
                }
                break;
            case game::CombatTextEntry::XP_GAIN:
                snprintf(text, sizeof(text), "+%d XP", entry.amount);
                color = ImVec4(0.7f, 0.3f, 1.0f, alpha);
                break;
            case game::CombatTextEntry::IMMUNE:
                snprintf(text, sizeof(text), "Immune!");
                color = ImVec4(0.9f, 0.9f, 0.9f, alpha);
                break;
            case game::CombatTextEntry::ABSORB:
                if (entry.amount > 0)
                    snprintf(text, sizeof(text), "Absorbed %d", entry.amount);
                else
                    snprintf(text, sizeof(text), "Absorbed");
                color = ImVec4(0.5f, 0.8f, 1.0f, alpha);
                break;
            case game::CombatTextEntry::RESIST:
                if (entry.amount > 0)
                    snprintf(text, sizeof(text), "Resisted %d", entry.amount);
                else
                    snprintf(text, sizeof(text), "Resisted");
                color = ImVec4(0.7f, 0.7f, 0.7f, alpha);
                break;
            case game::CombatTextEntry::DEFLECT:
                snprintf(text, sizeof(text), outgoing ? "Deflect" : "You Deflect");
                color = outgoing ? ImVec4(0.7f, 0.7f, 0.7f, alpha)
                                 : ImVec4(0.5f, 0.9f, 1.0f, alpha);
                break;
            case game::CombatTextEntry::REFLECT: {
                const std::string& reflectName = entry.spellId ? gameHandler.getSpellName(entry.spellId) : "";
                if (!reflectName.empty())
                    snprintf(text, sizeof(text), outgoing ? "Reflected: %s" : "Reflect: %s", reflectName.c_str());
                else
                    snprintf(text, sizeof(text), outgoing ? "Reflected" : "You Reflect");
                color = outgoing ? ImVec4(0.85f, 0.75f, 1.0f, alpha)
                                 : ImVec4(0.75f, 0.85f, 1.0f, alpha);
                break;
            }
            case game::CombatTextEntry::PROC_TRIGGER: {
                const std::string& procName = entry.spellId ? gameHandler.getSpellName(entry.spellId) : "";
                if (!procName.empty())
                    snprintf(text, sizeof(text), "%s!", procName.c_str());
                else
                    snprintf(text, sizeof(text), "PROC!");
                color = ImVec4(1.0f, 0.85f, 0.0f, alpha);
                break;
            }
            case game::CombatTextEntry::DISPEL:
                if (entry.spellId != 0) {
                    const std::string& dispelledName = gameHandler.getSpellName(entry.spellId);
                    if (!dispelledName.empty())
                        snprintf(text, sizeof(text), "Dispel %s", dispelledName.c_str());
                    else
                        snprintf(text, sizeof(text), "Dispel");
                } else {
                    snprintf(text, sizeof(text), "Dispel");
                }
                color = ImVec4(0.6f, 0.9f, 1.0f, alpha);
                break;
            case game::CombatTextEntry::STEAL:
                if (entry.spellId != 0) {
                    const std::string& stolenName = gameHandler.getSpellName(entry.spellId);
                    if (!stolenName.empty())
                        snprintf(text, sizeof(text), "Spellsteal %s", stolenName.c_str());
                    else
                        snprintf(text, sizeof(text), "Spellsteal");
                } else {
                    snprintf(text, sizeof(text), "Spellsteal");
                }
                color = ImVec4(0.8f, 0.7f, 1.0f, alpha);
                break;
            case game::CombatTextEntry::INTERRUPT: {
                const std::string& interruptedName = entry.spellId ? gameHandler.getSpellName(entry.spellId) : "";
                if (!interruptedName.empty())
                    snprintf(text, sizeof(text), "Interrupt %s", interruptedName.c_str());
                else
                    snprintf(text, sizeof(text), "Interrupt");
                color = ImVec4(1.0f, 0.6f, 0.9f, alpha);
                break;
            }
            case game::CombatTextEntry::INSTAKILL:
                snprintf(text, sizeof(text), outgoing ? "Kill!" : "Killed!");
                color = outgoing ? ImVec4(1.0f, 0.25f, 0.25f, alpha)
                                 : ImVec4(1.0f, 0.1f, 0.1f, alpha);
                break;
            case game::CombatTextEntry::HONOR_GAIN:
                snprintf(text, sizeof(text), "+%d Honor", entry.amount);
                color = ImVec4(1.0f, 0.85f, 0.0f, alpha);
                break;
            case game::CombatTextEntry::GLANCING:
                snprintf(text, sizeof(text), "~%d", entry.amount);
                color = outgoing ?
                    ImVec4(0.75f, 0.75f, 0.5f, alpha) :
                    ImVec4(0.75f, 0.35f, 0.35f, alpha);
                break;
            case game::CombatTextEntry::CRUSHING:
                snprintf(text, sizeof(text), "%d!", entry.amount);
                color = outgoing ?
                    ImVec4(1.0f, 0.55f, 0.1f, alpha) :
                    ImVec4(1.0f, 0.15f, 0.15f, alpha);
                break;
            default:
                snprintf(text, sizeof(text), "%d", entry.amount);
                color = ImVec4(1.0f, 1.0f, 1.0f, alpha);
                break;
        }

        // --- Rendering style ---
        bool isCrit = (entry.type == game::CombatTextEntry::CRIT_DAMAGE ||
                       entry.type == game::CombatTextEntry::CRIT_HEAL);
        float renderFontSize = isCrit ? baseFontSize * 1.35f : baseFontSize;

        ImU32 shadowCol = IM_COL32(0, 0, 0, static_cast<int>(alpha * 180));
        ImU32 textCol   = ImGui::ColorConvertFloat4ToU32(color);

        // --- Try world-space anchor if we have a destination entity ---
        // Types that should always stay as HUD elements (no world anchor)
        bool isHudOnly = (entry.type == game::CombatTextEntry::XP_GAIN ||
                          entry.type == game::CombatTextEntry::HONOR_GAIN ||
                          entry.type == game::CombatTextEntry::PROC_TRIGGER);

        bool rendered = false;
        if (!isHudOnly && camera && entry.dstGuid != 0) {
            // Look up the destination entity's render position
            glm::vec3 renderPos;
            bool havePos = core::Application::getInstance().getRenderPositionForGuid(entry.dstGuid, renderPos);
            if (!havePos) {
                // Fallback to entity canonical position
                auto entity = gameHandler.getEntityManager().getEntity(entry.dstGuid);
                if (entity) {
                    auto* unit = entity->isUnit() ? static_cast<game::Unit*>(entity.get()) : nullptr;
                    if (unit) {
                        renderPos = core::coords::canonicalToRender(
                            glm::vec3(unit->getX(), unit->getY(), unit->getZ()));
                        havePos = true;
                    }
                }
            }

            if (havePos) {
                // Float upward from above the entity's head
                renderPos.z += 2.5f + entry.age * 1.2f;

                // Project to screen
                glm::vec4 clipPos = viewProj * glm::vec4(renderPos, 1.0f);
                if (clipPos.w > 0.01f) {
                    glm::vec3 ndc = glm::vec3(clipPos) / clipPos.w;
                    if (ndc.x >= -1.5f && ndc.x <= 1.5f && ndc.y >= -1.5f && ndc.y <= 1.5f) {
                        float sx = (ndc.x * 0.5f + 0.5f) * screenW;
                        float sy = (ndc.y * 0.5f + 0.5f) * screenH;

                        // Horizontal stagger using the random seed
                        sx += entry.xSeed * 40.0f;

                        // Center the text horizontally on the projected point
                        ImVec2 ts = font->CalcTextSizeA(renderFontSize, FLT_MAX, 0.0f, text);
                        sx -= ts.x * 0.5f;

                        // Clamp to screen bounds
                        sx = std::max(2.0f, std::min(sx, screenW - ts.x - 2.0f));

                        drawList->AddText(font, renderFontSize,
                                          ImVec2(sx + 1.0f, sy + 1.0f), shadowCol, text);
                        drawList->AddText(font, renderFontSize,
                                          ImVec2(sx, sy), textCol, text);
                        rendered = true;
                    }
                }
            }
        }

        // --- HUD fallback for entries without world anchor or HUD-only types ---
        if (!rendered) {
            if (!needsHudWindow) {
                needsHudWindow = true;
                ImGui::SetNextWindowPos(ImVec2(0, 0));
                ImGui::SetNextWindowSize(ImVec2(screenW, 400));
                ImGuiWindowFlags flags = ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoDecoration |
                                         ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoNav;
                ImGui::Begin("##CombatText", nullptr, flags);
            }

            float yOffset = 200.0f - entry.age * 60.0f;
            int& idx = outgoing ? hudOutIdx : hudInIdx;
            float baseX = outgoing ? hudOutgoingX : hudIncomingX;
            float xOffset = baseX + (idx % 3 - 1) * 60.0f;
            ++idx;

            ImGui::SetCursorPos(ImVec2(xOffset, yOffset));
            ImVec2 screenPos = ImGui::GetCursorScreenPos();

            ImDrawList* dl = ImGui::GetWindowDrawList();
            dl->AddText(font, renderFontSize, ImVec2(screenPos.x + 1.0f, screenPos.y + 1.0f),
                        shadowCol, text);
            dl->AddText(font, renderFontSize, screenPos, textCol, text);

            ImVec2 ts = font->CalcTextSizeA(renderFontSize, FLT_MAX, 0.0f, text);
            ImGui::Dummy(ts);
        }
    }

    if (needsHudWindow) {
        ImGui::End();
    }
}


// ============================================================
// DPS / HPS Meter
// ============================================================

void CombatUI::renderDPSMeter(game::GameHandler& gameHandler,
                              const SettingsPanel& settings,
                              float targetFrameBottom) {
    if (!settings.showDPSMeter_) return;
    if (gameHandler.getState() != game::WorldState::IN_WORLD) return;

    const float dt = ImGui::GetIO().DeltaTime;

    // Track combat duration for accurate DPS denominator in short fights
    bool inCombat = gameHandler.isInCombat();
    if (inCombat && !dpsWasInCombat_) {
        // Just entered combat - reset encounter accumulators
        dpsEncounterDamage_ = 0.0f;
        dpsEncounterHeal_   = 0.0f;
        dpsLogSeenCount_    = gameHandler.getCombatLog().size();
        dpsCombatAge_       = 0.0f;
    }
    if (inCombat) {
        dpsCombatAge_ += dt;
        // Scan any new log entries since last frame
        const auto& log = gameHandler.getCombatLog();
        while (dpsLogSeenCount_ < log.size()) {
            const auto& e = log[dpsLogSeenCount_++];
            if (!e.isPlayerSource) continue;
            switch (e.type) {
                case game::CombatTextEntry::MELEE_DAMAGE:
                case game::CombatTextEntry::SPELL_DAMAGE:
                case game::CombatTextEntry::CRIT_DAMAGE:
                case game::CombatTextEntry::PERIODIC_DAMAGE:
                case game::CombatTextEntry::GLANCING:
                case game::CombatTextEntry::CRUSHING:
                    dpsEncounterDamage_ += static_cast<float>(e.amount);
                    break;
                case game::CombatTextEntry::HEAL:
                case game::CombatTextEntry::CRIT_HEAL:
                case game::CombatTextEntry::PERIODIC_HEAL:
                    dpsEncounterHeal_ += static_cast<float>(e.amount);
                    break;
                default: break;
            }
        }
    } else if (dpsWasInCombat_) {
        // Just left combat - keep encounter totals but stop accumulating
    }
    dpsWasInCombat_ = inCombat;

    // Sum all player-source damage and healing in the current combat-text window
    float totalDamage = 0.0f, totalHeal = 0.0f;
    for (const auto& e : gameHandler.getCombatText()) {
        if (!e.isPlayerSource) continue;
        switch (e.type) {
            case game::CombatTextEntry::MELEE_DAMAGE:
            case game::CombatTextEntry::SPELL_DAMAGE:
            case game::CombatTextEntry::CRIT_DAMAGE:
            case game::CombatTextEntry::PERIODIC_DAMAGE:
            case game::CombatTextEntry::GLANCING:
            case game::CombatTextEntry::CRUSHING:
                totalDamage += static_cast<float>(e.amount);
                break;
            case game::CombatTextEntry::HEAL:
            case game::CombatTextEntry::CRIT_HEAL:
            case game::CombatTextEntry::PERIODIC_HEAL:
                totalHeal += static_cast<float>(e.amount);
                break;
            default: break;
        }
    }

    // Only show if there's something to report (rolling window or lingering encounter data)
    if (totalDamage < 1.0f && totalHeal < 1.0f && !inCombat &&
        dpsEncounterDamage_ < 1.0f && dpsEncounterHeal_ < 1.0f) return;

    // DPS window = min(combat age, combat-text lifetime) to avoid under-counting
    // at the start of a fight and over-counting when entries expire.
    float window = std::min(dpsCombatAge_, game::CombatTextEntry::LIFETIME);
    if (window < 0.1f) window = 0.1f;

    float dps = totalDamage / window;
    float hps = totalHeal / window;

    // Format numbers with K/M suffix for readability
    auto fmtNum = [](float v, char* buf, int bufSz) {
        if (v >= 1e6f)       snprintf(buf, bufSz, "%.1fM", v / 1e6f);
        else if (v >= 1000.f) snprintf(buf, bufSz, "%.1fK", v / 1000.f);
        else                 snprintf(buf, bufSz, "%.0f", v);
    };

    char dpsBuf[16], hpsBuf[16];
    fmtNum(dps, dpsBuf, sizeof(dpsBuf));
    fmtNum(hps, hpsBuf, sizeof(hpsBuf));

    // Position: small floating label just above the action bar, right of center
    auto* appWin = services_.window;
    float screenW = appWin ? static_cast<float>(appWin->getWidth())  : 1280.0f;
    float screenH = appWin ? static_cast<float>(appWin->getHeight()) : 720.0f;

    // Show encounter row when fight has been going long enough (> 3s)
    bool showEnc = (dpsCombatAge_ > 3.0f || (!inCombat && dpsEncounterDamage_ > 0.0f));
    float encDPS = (dpsCombatAge_ > 0.1f) ? dpsEncounterDamage_ / dpsCombatAge_ : 0.0f;
    float encHPS = (dpsCombatAge_ > 0.1f) ? dpsEncounterHeal_   / dpsCombatAge_ : 0.0f;

    char encDpsBuf[16], encHpsBuf[16];
    fmtNum(encDPS, encDpsBuf, sizeof(encDpsBuf));
    fmtNum(encHPS, encHpsBuf, sizeof(encHpsBuf));

    constexpr float WIN_W = 90.0f;
    // Extra rows for encounter DPS/HPS if active
    int extraRows = 0;
    if (showEnc && encDPS > 0.5f) ++extraRows;
    if (showEnc && encHPS > 0.5f) ++extraRows;
    float WIN_H = 18.0f + extraRows * 14.0f;
    if (dps > 0.5f || hps > 0.5f) WIN_H = std::max(WIN_H, 36.0f);
    // Default home is directly under the target frame, centred on it. When there
    // is no target the frame is not drawn, so fall back to the height it would
    // have occupied rather than letting the meter jump to a different spot.
    constexpr float kTargetFrameGap = 6.0f;
    constexpr float kNoTargetFallbackY = 30.0f + 96.0f;  // frame top + typical height
    const float defaultX = screenW * 0.5f - WIN_W * 0.5f;
    const float defaultY = (targetFrameBottom > 0.0f)
                         ? targetFrameBottom + kTargetFrameGap
                         : kNoTargetFallbackY + kTargetFrameGap;

    // NoMove/NoInputs are deliberately absent: the meter is draggable, which
    // needs the window to receive mouse input and own its position once moved.
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoNav;
    if (dpsMeterUserPositioned_) {
        // ImGui owns the position from here on; seed it once from the saved value.
        ImGui::SetNextWindowPos(dpsMeterPos_, ImGuiCond_Once);
        // Keep it on screen if the window was resized smaller since it was saved.
        if (dpsMeterPos_.x > screenW - WIN_W || dpsMeterPos_.y > screenH - WIN_H) {
            dpsMeterPos_.x = std::min(dpsMeterPos_.x, std::max(0.0f, screenW - WIN_W));
            dpsMeterPos_.y = std::min(dpsMeterPos_.y, std::max(0.0f, screenH - WIN_H));
            ImGui::SetNextWindowPos(dpsMeterPos_, ImGuiCond_Always);
        }
    } else {
        ImGui::SetNextWindowPos(ImVec2(defaultX, defaultY), ImGuiCond_Always);
    }
    ImGui::SetNextWindowSize(ImVec2(WIN_W, WIN_H), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.55f);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(4, 3));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.3f, 0.3f, 0.3f, 0.7f));

    if (ImGui::Begin("##DPSMeter", nullptr, flags)) {
        if (dps > 0.5f) {
            ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.15f, 1.0f), "%s", dpsBuf);
            ImGui::SameLine(0, 2);
            ImGui::TextDisabled("dps");
        }
        if (hps > 0.5f) {
            ImGui::TextColored(ImVec4(0.35f, 1.0f, 0.35f, 1.0f), "%s", hpsBuf);
            ImGui::SameLine(0, 2);
            ImGui::TextDisabled("hps");
        }
        // Encounter totals (full-fight average, shown when fight > 3s)
        if (showEnc && encDPS > 0.5f) {
            ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.25f, 0.80f), "%s", encDpsBuf);
            ImGui::SameLine(0, 2);
            ImGui::TextDisabled("enc");
        }
        if (showEnc && encHPS > 0.5f) {
            ImGui::TextColored(ImVec4(0.50f, 1.0f, 0.50f, 0.80f), "%s", encHpsBuf);
            ImGui::SameLine(0, 2);
            ImGui::TextDisabled("enc");
        }

        // Grabbing the window latches it out of auto-placement: from the next
        // frame the position is no longer forced, so ImGui's own drag handling
        // takes over and the result is remembered across sessions.
        if (!dpsMeterUserPositioned_ &&
            ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            dpsMeterUserPositioned_ = true;
        }
        if (dpsMeterUserPositioned_) dpsMeterPos_ = ImGui::GetWindowPos();

        if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
            // Right-click sends it home, so a meter dragged off-screen or into
            // the way is always recoverable without touching a config file.
            dpsMeterUserPositioned_ = false;
            dpsMeterPos_ = ImVec2(-1.0f, -1.0f);
        }
        if (ImGui::IsWindowHovered()) {
            ImGui::SetTooltip("Drag to move - right-click to reset position");
        }
    }
    ImGui::End();

    ImGui::PopStyleColor();
    ImGui::PopStyleVar(2);
}


// ============================================================
// Buff/Debuff Bar
// ============================================================

//   WSG  489 – Alliance / Horde flag captures (max 3)
//   AB   529 – Alliance / Horde resource scores (max 1600)
//   AV    30 – Alliance / Horde reinforcements
//   EotS 566 – Alliance / Horde resource scores (max 1600)
// ============================================================================
void CombatUI::renderBattlegroundScore(game::GameHandler& gameHandler) {
    // FrameXML presents the same scores through WorldStateAlwaysUpFrame, which
    // reads the shared table below via GetWorldStateUIInfo.
    if (frameXmlOwns(UiElement::BattlegroundScore)) return;

    // Only show when in a recognised battleground map
    uint32_t mapId = gameHandler.getWorldStateMapId();

    const game::BgScoreDef* def = game::findBgScoreDef(mapId);
    if (!def) return;

    auto allianceOpt = gameHandler.getWorldState(def->allianceKey);
    auto hordeOpt    = gameHandler.getWorldState(def->hordeKey);
    if (!allianceOpt && !hordeOpt) return;

    uint32_t allianceScore = allianceOpt.value_or(0);
    uint32_t hordeScore    = hordeOpt.value_or(0);
    uint32_t maxScore      = def->hardcodedMax;
    if (def->maxKey != 0) {
        if (auto mv = gameHandler.getWorldState(def->maxKey)) maxScore = *mv;
    }

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth())  : 1280.0f;

    // Width scales with screen but stays reasonable
    float frameW = 260.0f;
    float frameH = 60.0f;
    float posX   = screenW / 2.0f - frameW / 2.0f;
    float posY   = 4.0f;

    ImGui::SetNextWindowPos(ImVec2(posX, posY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(frameW, frameH), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.75f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 6.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,  ImVec2(6.0f, 4.0f));

    if (ImGui::Begin("##BGScore", nullptr,
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoBringToFrontOnFocus |
            ImGuiWindowFlags_NoSavedSettings)) {

        // BG name centred at top
        float nameW = ImGui::CalcTextSize(def->name).x;
        ImGui::SetCursorPosX((frameW - nameW) / 2.0f);
        ImGui::TextColored(colors::kBrightGold, "%s", def->name);

        // Alliance score | separator | Horde score
        float innerW  = frameW - 12.0f;
        float halfW   = innerW / 2.0f - 4.0f;

        ImGui::SetCursorPosX(6.0f);
        ImGui::BeginGroup();
        {
            // Alliance (blue)
            char aBuf[32];
            if (maxScore > 0 && strlen(def->unit) > 0)
                snprintf(aBuf, sizeof(aBuf), "\xF0\x9F\x94\xB5 %u / %u", allianceScore, maxScore);
            else
                snprintf(aBuf, sizeof(aBuf), "\xF0\x9F\x94\xB5 %u", allianceScore);
            ImGui::TextColored(colors::kLightBlue, "%s", aBuf);
        }
        ImGui::EndGroup();

        ImGui::SameLine(halfW + 16.0f);

        ImGui::BeginGroup();
        {
            // Horde (red)
            char hBuf[32];
            if (maxScore > 0 && strlen(def->unit) > 0)
                snprintf(hBuf, sizeof(hBuf), "\xF0\x9F\x94\xB4 %u / %u", hordeScore, maxScore);
            else
                snprintf(hBuf, sizeof(hBuf), "\xF0\x9F\x94\xB4 %u", hordeScore);
            ImGui::TextColored(colors::kHostileRed, "%s", hBuf);
        }
        ImGui::EndGroup();
    }
    ImGui::End();
    ImGui::PopStyleVar(2);
}


// ─── Combat Log Window ────────────────────────────────────────────────────────
void CombatUI::renderCombatLog(game::GameHandler& gameHandler,
                              SpellbookScreen& spellbookScreen) {
    if (!showCombatLog_) return;

    const auto& log = gameHandler.getCombatLog();

    ImGui::SetNextWindowSize(ImVec2(520, 320), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(160, 200), ImGuiCond_FirstUseEver);

    char title[64];
    snprintf(title, sizeof(title), "Combat Log (%zu)###CombatLog", log.size());
    if (!ImGui::Begin(title, &showCombatLog_)) {
        ImGui::End();
        return;
    }

    // Filter toggles
    static bool filterDamage  = true;
    static bool filterHeal    = true;
    static bool filterMisc    = true;
    static bool autoScroll    = true;

    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4, 2));
    ImGui::Checkbox("Damage",  &filterDamage); ImGui::SameLine();
    ImGui::Checkbox("Healing", &filterHeal);   ImGui::SameLine();
    ImGui::Checkbox("Misc",    &filterMisc);   ImGui::SameLine();
    ImGui::Checkbox("Auto-scroll", &autoScroll);
    ImGui::SameLine(ImGui::GetContentRegionAvail().x - 40.0f);
    if (ImGui::SmallButton("Clear"))
        gameHandler.clearCombatLog();
    ImGui::PopStyleVar();
    ImGui::Separator();

    // Helper: categorize entry
    auto isDamageType = [](game::CombatTextEntry::Type t) {
        using T = game::CombatTextEntry;
        return t == T::MELEE_DAMAGE || t == T::SPELL_DAMAGE ||
               t == T::CRIT_DAMAGE  || t == T::PERIODIC_DAMAGE ||
               t == T::ENVIRONMENTAL || t == T::GLANCING || t == T::CRUSHING;
    };
    auto isHealType = [](game::CombatTextEntry::Type t) {
        using T = game::CombatTextEntry;
        return t == T::HEAL || t == T::CRIT_HEAL || t == T::PERIODIC_HEAL;
    };

    // Two-column table: Time | Event description
    ImGuiTableFlags tableFlags = ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg |
                                 ImGuiTableFlags_SizingFixedFit;
    float availH = ImGui::GetContentRegionAvail().y;
    if (ImGui::BeginTable("##CombatLogTable", 2, tableFlags, ImVec2(0.0f, availH))) {
        ImGui::TableSetupScrollFreeze(0, 0);
        ImGui::TableSetupColumn("Time",  ImGuiTableColumnFlags_WidthFixed, 62.0f);
        ImGui::TableSetupColumn("Event", ImGuiTableColumnFlags_WidthStretch);

        for (const auto& e : log) {
            // Apply filters
            bool isDmg  = isDamageType(e.type);
            bool isHeal = isHealType(e.type);
            bool isMisc = !isDmg && !isHeal;
            if (isDmg  && !filterDamage) continue;
            if (isHeal && !filterHeal)   continue;
            if (isMisc && !filterMisc)   continue;

            // Format timestamp as HH:MM:SS
            char timeBuf[10];
            {
                const std::tm tm_info = core::localTime(e.timestamp);
                snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d:%02d",
                         tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec);
            }

            // Build event description and choose color
            char desc[256];
            ImVec4 color;
            using T = game::CombatTextEntry;
            const char* src = e.sourceName.empty() ? (e.isPlayerSource ? "You" : "?") : e.sourceName.c_str();
            const char* tgt = e.targetName.empty() ? "?" : e.targetName.c_str();
            const std::string& spellName = (e.spellId != 0) ? gameHandler.getSpellName(e.spellId) : std::string();
            const char* spell = spellName.empty() ? nullptr : spellName.c_str();

            switch (e.type) {
                case T::MELEE_DAMAGE:
                    snprintf(desc, sizeof(desc), "%s hits %s for %d", src, tgt, e.amount);
                    color = e.isPlayerSource ? ImVec4(1.0f, 0.9f, 0.3f, 1.0f) : colors::kSoftRed;
                    break;
                case T::CRIT_DAMAGE:
                    snprintf(desc, sizeof(desc), "%s crits %s for %d!", src, tgt, e.amount);
                    color = e.isPlayerSource ? ImVec4(1.0f, 1.0f, 0.0f, 1.0f) : colors::kBrightRed;
                    break;
                case T::SPELL_DAMAGE:
                    if (spell)
                        snprintf(desc, sizeof(desc), "%s's %s hits %s for %d", src, spell, tgt, e.amount);
                    else
                        snprintf(desc, sizeof(desc), "%s's spell hits %s for %d", src, tgt, e.amount);
                    color = e.isPlayerSource ? ImVec4(1.0f, 0.9f, 0.3f, 1.0f) : colors::kSoftRed;
                    break;
                case T::PERIODIC_DAMAGE:
                    if (spell)
                        snprintf(desc, sizeof(desc), "%s's %s ticks %s for %d", src, spell, tgt, e.amount);
                    else
                        snprintf(desc, sizeof(desc), "%s's DoT ticks %s for %d", src, tgt, e.amount);
                    color = e.isPlayerSource ? ImVec4(0.9f, 0.7f, 0.3f, 1.0f) : ImVec4(0.9f, 0.3f, 0.3f, 1.0f);
                    break;
                case T::HEAL:
                    if (spell)
                        snprintf(desc, sizeof(desc), "%s heals %s for %d (%s)", src, tgt, e.amount, spell);
                    else
                        snprintf(desc, sizeof(desc), "%s heals %s for %d", src, tgt, e.amount);
                    color = kColorGreen;
                    break;
                case T::CRIT_HEAL:
                    if (spell)
                        snprintf(desc, sizeof(desc), "%s critically heals %s for %d! (%s)", src, tgt, e.amount, spell);
                    else
                        snprintf(desc, sizeof(desc), "%s critically heals %s for %d!", src, tgt, e.amount);
                    color = kColorBrightGreen;
                    break;
                case T::PERIODIC_HEAL:
                    if (spell)
                        snprintf(desc, sizeof(desc), "%s's %s heals %s for %d", src, spell, tgt, e.amount);
                    else
                        snprintf(desc, sizeof(desc), "%s's HoT heals %s for %d", src, tgt, e.amount);
                    color = ImVec4(0.4f, 0.9f, 0.4f, 1.0f);
                    break;
                case T::MISS:
                    if (spell)
                        snprintf(desc, sizeof(desc), "%s's %s misses %s", src, spell, tgt);
                    else
                        snprintf(desc, sizeof(desc), "%s misses %s", src, tgt);
                    color = colors::kMediumGray;
                    break;
                case T::DODGE:
                    if (spell)
                        snprintf(desc, sizeof(desc), "%s dodges %s's %s", tgt, src, spell);
                    else
                        snprintf(desc, sizeof(desc), "%s dodges %s's attack", tgt, src);
                    color = colors::kMediumGray;
                    break;
                case T::PARRY:
                    if (spell)
                        snprintf(desc, sizeof(desc), "%s parries %s's %s", tgt, src, spell);
                    else
                        snprintf(desc, sizeof(desc), "%s parries %s's attack", tgt, src);
                    color = colors::kMediumGray;
                    break;
                case T::BLOCK:
                    if (spell)
                        snprintf(desc, sizeof(desc), "%s blocks %s's %s (%d blocked)", tgt, src, spell, e.amount);
                    else
                        snprintf(desc, sizeof(desc), "%s blocks %s's attack (%d blocked)", tgt, src, e.amount);
                    color = ImVec4(0.65f, 0.75f, 0.65f, 1.0f);
                    break;
                case T::EVADE:
                    if (spell)
                        snprintf(desc, sizeof(desc), "%s evades %s's %s", tgt, src, spell);
                    else
                        snprintf(desc, sizeof(desc), "%s evades %s's attack", tgt, src);
                    color = colors::kMediumGray;
                    break;
                case T::IMMUNE:
                    if (spell)
                        snprintf(desc, sizeof(desc), "%s is immune to %s", tgt, spell);
                    else
                        snprintf(desc, sizeof(desc), "%s is immune", tgt);
                    color = colors::kSilver;
                    break;
                case T::ABSORB:
                    if (spell && e.amount > 0)
                        snprintf(desc, sizeof(desc), "%s's %s absorbs %d", src, spell, e.amount);
                    else if (spell)
                        snprintf(desc, sizeof(desc), "%s absorbs %s", tgt, spell);
                    else if (e.amount > 0)
                        snprintf(desc, sizeof(desc), "%d absorbed", e.amount);
                    else
                        snprintf(desc, sizeof(desc), "Absorbed");
                    color = ImVec4(0.5f, 0.8f, 1.0f, 1.0f);
                    break;
                case T::RESIST:
                    if (spell && e.amount > 0)
                        snprintf(desc, sizeof(desc), "%s resists %s's %s (%d resisted)", tgt, src, spell, e.amount);
                    else if (spell)
                        snprintf(desc, sizeof(desc), "%s resists %s's %s", tgt, src, spell);
                    else if (e.amount > 0)
                        snprintf(desc, sizeof(desc), "%d resisted", e.amount);
                    else
                        snprintf(desc, sizeof(desc), "Resisted");
                    color = ImVec4(0.6f, 0.6f, 0.9f, 1.0f);
                    break;
                case T::DEFLECT:
                    if (spell)
                        snprintf(desc, sizeof(desc), "%s deflects %s's %s", tgt, src, spell);
                    else
                        snprintf(desc, sizeof(desc), "%s deflects %s's attack", tgt, src);
                    color = ImVec4(0.65f, 0.8f, 0.95f, 1.0f);
                    break;
                case T::REFLECT:
                    if (spell)
                        snprintf(desc, sizeof(desc), "%s reflects %s's %s", tgt, src, spell);
                    else
                        snprintf(desc, sizeof(desc), "%s reflects %s's attack", tgt, src);
                    color = ImVec4(0.8f, 0.7f, 1.0f, 1.0f);
                    break;
                case T::ENVIRONMENTAL: {
                    const char* envName = "Environmental";
                    switch (e.powerType) {
                        case 0: envName = "Fatigue"; break;
                        case 1: envName = "Drowning"; break;
                        case 2: envName = "Falling"; break;
                        case 3: envName = "Lava"; break;
                        case 4: envName = "Slime"; break;
                        case 5: envName = "Fire"; break;
                    }
                    snprintf(desc, sizeof(desc), "%s damage: %d", envName, e.amount);
                    color = ImVec4(1.0f, 0.5f, 0.2f, 1.0f);
                    break;
                }
                case T::ENERGIZE: {
                    const char* pwrName = game::powerTypeName(e.powerType);
                    if (!pwrName) pwrName = "power";
                    if (spell)
                        snprintf(desc, sizeof(desc), "%s gains %d %s (%s)", tgt, e.amount, pwrName, spell);
                    else
                        snprintf(desc, sizeof(desc), "%s gains %d %s", tgt, e.amount, pwrName);
                    color = colors::kLightBlue;
                    break;
                }
                case T::POWER_DRAIN: {
                    const char* drainName = game::powerTypeName(e.powerType);
                    if (!drainName) drainName = "power";
                    if (spell)
                        snprintf(desc, sizeof(desc), "%s loses %d %s to %s's %s", tgt, e.amount, drainName, src, spell);
                    else
                        snprintf(desc, sizeof(desc), "%s loses %d %s", tgt, e.amount, drainName);
                    color = ImVec4(0.45f, 0.75f, 1.0f, 1.0f);
                    break;
                }
                case T::XP_GAIN:
                    snprintf(desc, sizeof(desc), "You gain %d experience", e.amount);
                    color = ImVec4(0.8f, 0.6f, 1.0f, 1.0f);
                    break;
                case T::PROC_TRIGGER:
                    if (spell)
                        snprintf(desc, sizeof(desc), "%s procs!", spell);
                    else
                        snprintf(desc, sizeof(desc), "Proc triggered");
                    color = ImVec4(1.0f, 0.85f, 0.3f, 1.0f);
                    break;
                case T::DISPEL:
                    if (spell && e.isPlayerSource)
                        snprintf(desc, sizeof(desc), "You dispel %s from %s", spell, tgt);
                    else if (spell)
                        snprintf(desc, sizeof(desc), "%s dispels %s from %s", src, spell, tgt);
                    else if (e.isPlayerSource)
                        snprintf(desc, sizeof(desc), "You dispel from %s", tgt);
                    else
                        snprintf(desc, sizeof(desc), "%s dispels from %s", src, tgt);
                    color = ImVec4(0.6f, 0.9f, 1.0f, 1.0f);
                    break;
                case T::STEAL:
                    if (spell && e.isPlayerSource)
                        snprintf(desc, sizeof(desc), "You steal %s from %s", spell, tgt);
                    else if (spell)
                        snprintf(desc, sizeof(desc), "%s steals %s from %s", src, spell, tgt);
                    else if (e.isPlayerSource)
                        snprintf(desc, sizeof(desc), "You steal from %s", tgt);
                    else
                        snprintf(desc, sizeof(desc), "%s steals from %s", src, tgt);
                    color = ImVec4(0.8f, 0.7f, 1.0f, 1.0f);
                    break;
                case T::INTERRUPT:
                    if (spell && e.isPlayerSource)
                        snprintf(desc, sizeof(desc), "You interrupt %s's %s", tgt, spell);
                    else if (spell)
                        snprintf(desc, sizeof(desc), "%s interrupts %s's %s", src, tgt, spell);
                    else if (e.isPlayerSource)
                        snprintf(desc, sizeof(desc), "You interrupt %s", tgt);
                    else
                        snprintf(desc, sizeof(desc), "%s interrupted", tgt);
                    color = ImVec4(1.0f, 0.6f, 0.9f, 1.0f);
                    break;
                case T::INSTAKILL:
                    if (spell && e.isPlayerSource)
                        snprintf(desc, sizeof(desc), "You instantly kill %s with %s", tgt, spell);
                    else if (spell)
                        snprintf(desc, sizeof(desc), "%s instantly kills %s with %s", src, tgt, spell);
                    else if (e.isPlayerSource)
                        snprintf(desc, sizeof(desc), "You instantly kill %s", tgt);
                    else
                        snprintf(desc, sizeof(desc), "%s instantly kills %s", src, tgt);
                    color = colors::kBrightRed;
                    break;
                case T::HONOR_GAIN:
                    snprintf(desc, sizeof(desc), "You gain %d honor", e.amount);
                    color = colors::kBrightGold;
                    break;
                case T::GLANCING:
                    snprintf(desc, sizeof(desc), "%s glances %s for %d", src, tgt, e.amount);
                    color = e.isPlayerSource ? ImVec4(0.75f, 0.75f, 0.5f, 1.0f)
                                             : ImVec4(0.75f, 0.4f, 0.4f, 1.0f);
                    break;
                case T::CRUSHING:
                    snprintf(desc, sizeof(desc), "%s crushes %s for %d!", src, tgt, e.amount);
                    color = e.isPlayerSource ? ImVec4(1.0f, 0.55f, 0.1f, 1.0f)
                                             : ImVec4(1.0f, 0.15f, 0.15f, 1.0f);
                    break;
                default:
                    snprintf(desc, sizeof(desc), "Combat event (type %d, amount %d)", static_cast<int>(e.type), e.amount);
                    color = ui::colors::kLightGray;
                    break;
            }

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextDisabled("%s", timeBuf);
            ImGui::TableSetColumnIndex(1);
            ImGui::TextColored(color, "%s", desc);
            // Hover tooltip: show rich spell info for entries with a known spell
            if (e.spellId != 0 && ImGui::IsItemHovered()) {
                auto* assetMgrLog = services_.assetManager;
                ImGui::BeginTooltip();
                bool richOk = spellbookScreen.renderSpellInfoTooltip(e.spellId, gameHandler, assetMgrLog);
                if (!richOk) {
                    ImGui::Text("%s", spellName.c_str());
                }
                ImGui::EndTooltip();
            }
        }

        // Auto-scroll to bottom
        if (autoScroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
            ImGui::SetScrollHereY(1.0f);

        ImGui::EndTable();
    }

    ImGui::End();
}


// ─── Threat Window ────────────────────────────────────────────────────────────
void CombatUI::renderThreatWindow(game::GameHandler& gameHandler) {
    if (!showThreatWindow_) return;

    const auto* list = gameHandler.getTargetThreatList();

    ImGui::SetNextWindowSize(ImVec2(280, 220), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(10, 300), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.85f);

    if (!ImGui::Begin("Threat###ThreatWin", &showThreatWindow_,
                      ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }

    if (!list || list->empty()) {
        ImGui::TextDisabled("No threat data for current target.");
        ImGui::End();
        return;
    }

    uint32_t maxThreat = list->front().threat;

    // Pre-scan to find the player's rank and threat percentage
    uint64_t playerGuid = gameHandler.getPlayerGuid();
    int playerRank = 0;
    float playerPct = 0.0f;
    {
        int scan = 0;
        for (const auto& e : *list) {
            ++scan;
            if (e.victimGuid == playerGuid) {
                playerRank = scan;
                playerPct  = (maxThreat > 0) ? static_cast<float>(e.threat) / static_cast<float>(maxThreat) : 0.0f;
                break;
            }
            if (scan >= 10) break;
        }
    }

    // Status bar: aggro alert or rank summary
    if (playerRank == 1) {
        // Player has aggro - persistent red warning
        ImGui::TextColored(ImVec4(1.0f, 0.25f, 0.25f, 1.0f), "!! YOU HAVE AGGRO !!");
    } else if (playerRank > 1 && playerPct >= 0.8f) {
        // Close to pulling - pulsing warning
        float pulse = 0.55f + 0.45f * sinf(static_cast<float>(ImGui::GetTime()) * 5.0f);
        ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.1f, pulse), "! PULLING AGGRO (%.0f%%) !", playerPct * 100.0f);
    } else if (playerRank > 0) {
        ImGui::TextColored(ImVec4(0.6f, 0.8f, 0.6f, 1.0f), "You: #%d  %.0f%% threat", playerRank, playerPct * 100.0f);
    }

    ImGui::TextDisabled("%-19s  Threat", "Player");
    ImGui::Separator();

    int rank = 0;
    for (const auto& entry : *list) {
        ++rank;
        bool isPlayer = (entry.victimGuid == playerGuid);

        // Resolve name
        std::string victimName;
        auto entity = gameHandler.getEntityManager().getEntity(entry.victimGuid);
        if (entity) {
            if (entity->getType() == game::ObjectType::PLAYER) {
                auto p = std::static_pointer_cast<game::Player>(entity);
                victimName = p->getName().empty() ? "Player" : p->getName();
            } else if (entity->getType() == game::ObjectType::UNIT) {
                auto u = std::static_pointer_cast<game::Unit>(entity);
                victimName = u->getName().empty() ? "NPC" : u->getName();
            }
        }
        if (victimName.empty())
            victimName = "0x" + [&](){
                char buf[20]; snprintf(buf, sizeof(buf), "%llX",
                    static_cast<unsigned long long>(entry.victimGuid)); return std::string(buf); }();

        // Colour: gold for #1 (tank), red if player is highest, white otherwise
        ImVec4 col = ui::colors::kWhite;
        if (rank == 1) col = ui::colors::kTooltipGold;      // gold
        if (isPlayer && rank == 1) col = kColorRed; // red - you have aggro

        // Threat bar
        float pct = (maxThreat > 0) ? static_cast<float>(entry.threat) / static_cast<float>(maxThreat) : 0.0f;
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
            isPlayer ? ImVec4(0.8f, 0.2f, 0.2f, 0.7f) : ImVec4(0.2f, 0.5f, 0.8f, 0.5f));
        char barLabel[48];
        snprintf(barLabel, sizeof(barLabel), "%.0f%%", pct * 100.0f);
        ImGui::ProgressBar(pct, ImVec2(60, 14), barLabel);
        ImGui::PopStyleColor();
        ImGui::SameLine();

        ImGui::TextColored(col, "%-18s  %u", victimName.c_str(), entry.threat);

        if (rank >= 10) break; // cap display at 10 entries
    }

    ImGui::End();
}


// ─── BG Scoreboard ────────────────────────────────────────────────────────────
void CombatUI::renderBgScoreboard(game::GameHandler& gameHandler) {
    if (!showBgScoreboard_) return;

    const game::GameHandler::BgScoreboardData* data = gameHandler.getBgScoreboard();

    ImGui::SetNextWindowSize(ImVec2(600, 400), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(150, 100), ImGuiCond_FirstUseEver);

    const char* title = data && data->isArena ? "Arena Score###BgScore"
                                              : "Battleground Score###BgScore";
    if (!ImGui::Begin(title, &showBgScoreboard_, ImGuiWindowFlags_NoCollapse)) {
        ImGui::End();
        return;
    }

    if (!data) {
        ImGui::TextDisabled("No score data yet.");
        ImGui::TextDisabled("Use /score to request the scoreboard while in a battleground or arena.");
        ImGui::End();
        return;
    }

    // Arena team rating banner (shown only for arenas)
    if (data->isArena) {
        for (int t = 0; t < 2; ++t) {
            const auto& at = data->arenaTeams[t];
            if (at.teamName.empty()) continue;
            int32_t ratingDelta = static_cast<int32_t>(at.ratingChange);
            ImVec4 teamCol = (t == 0) ? colors::kHostileRed   // team 0: red
                                      : colors::kLightBlue;    // team 1: blue
            ImGui::TextColored(teamCol, "%s", at.teamName.c_str());
            ImGui::SameLine();
            char ratingBuf[32];
            if (ratingDelta >= 0)
                std::snprintf(ratingBuf, sizeof(ratingBuf), "Rating: %u (+%d)", at.newRating, ratingDelta);
            else
                std::snprintf(ratingBuf, sizeof(ratingBuf), "Rating: %u (%d)", at.newRating, ratingDelta);
            ImGui::TextDisabled("%s", ratingBuf);
        }
        ImGui::Separator();
    }

    // Winner banner
    if (data->hasWinner) {
        const char* winnerStr;
        ImVec4 winnerColor;
        if (data->isArena) {
            // For arenas, winner byte 0/1 refers to team index in arenaTeams[]
            const auto& winTeam = data->arenaTeams[data->winner & 1];
            winnerStr  = winTeam.teamName.empty() ? "Team 1" : winTeam.teamName.c_str();
            winnerColor = (data->winner == 0) ? colors::kHostileRed
                                              : colors::kLightBlue;
        } else {
            winnerStr  = (data->winner == 1) ? "Alliance" : "Horde";
            winnerColor = (data->winner == 1) ? colors::kLightBlue
                                              : colors::kHostileRed;
        }
        float textW = ImGui::CalcTextSize(winnerStr).x + ImGui::CalcTextSize("  Victory!").x;
        ImGui::SetCursorPosX((ImGui::GetContentRegionAvail().x - textW) * 0.5f);
        ImGui::TextColored(winnerColor, "%s", winnerStr);
        ImGui::SameLine(0, 4);
        ImGui::TextColored(colors::kBrightGold, "Victory!");
        ImGui::Separator();
    }

    // Refresh button
    if (ImGui::SmallButton("Refresh")) {
        gameHandler.requestPvpLog();
    }
    ImGui::SameLine();
    ImGui::TextDisabled("%zu players", data->players.size());

    // Score table
    constexpr ImGuiTableFlags kTableFlags =
        ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg |
        ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV |
        ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Sortable;

    // Build dynamic column count based on what BG-specific stats are present
    int numBgCols = 0;
    std::vector<std::string> bgColNames;
    for (const auto& ps : data->players) {
        for (const auto& [fieldName, val] : ps.bgStats) {
            // Extract short name after last '.' (e.g. "BattlegroundAB.AbFlagCaptures" → "Caps")
            std::string shortName = fieldName;
            auto dotPos = fieldName.rfind('.');
            if (dotPos != std::string::npos) shortName = fieldName.substr(dotPos + 1);
            bool found = false;
            for (const auto& n : bgColNames) { if (n == shortName) { found = true; break; } }
            if (!found) bgColNames.push_back(shortName);
        }
    }
    numBgCols = static_cast<int>(bgColNames.size());

    // Fixed cols: Team | Name | KB | Deaths | HKs | Honor; then BG-specific
    int totalCols = 6 + numBgCols;
    float tableH = ImGui::GetContentRegionAvail().y;
    if (ImGui::BeginTable("##BgScoreTable", totalCols, kTableFlags, ImVec2(0.0f, tableH))) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("Team",   ImGuiTableColumnFlags_WidthFixed,   56.0f);
        ImGui::TableSetupColumn("Name",   ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("KB",     ImGuiTableColumnFlags_WidthFixed,   38.0f);
        ImGui::TableSetupColumn("Deaths", ImGuiTableColumnFlags_WidthFixed,   52.0f);
        ImGui::TableSetupColumn("HKs",    ImGuiTableColumnFlags_WidthFixed,   38.0f);
        ImGui::TableSetupColumn("Honor",  ImGuiTableColumnFlags_WidthFixed,   52.0f);
        for (const auto& col : bgColNames)
            ImGui::TableSetupColumn(col.c_str(), ImGuiTableColumnFlags_WidthFixed, 52.0f);
        ImGui::TableHeadersRow();

        // Sort: Alliance first, then Horde; within each team by KB desc
        std::vector<const game::GameHandler::BgPlayerScore*> sorted;
        sorted.reserve(data->players.size());
        for (const auto& ps : data->players) sorted.push_back(&ps);
        std::stable_sort(sorted.begin(), sorted.end(),
            [](const game::GameHandler::BgPlayerScore* a,
               const game::GameHandler::BgPlayerScore* b) {
                if (a->team != b->team) return a->team > b->team;  // Alliance(1) first
                return a->killingBlows > b->killingBlows;
            });

        uint64_t playerGuid = gameHandler.getPlayerGuid();
        for (const auto* ps : sorted) {
            ImGui::TableNextRow();

            // Team
            ImGui::TableNextColumn();
            if (ps->team == 1)
                ImGui::TextColored(colors::kLightBlue, "Alliance");
            else
                ImGui::TextColored(colors::kHostileRed, "Horde");

            // Name (highlight player's own row)
            ImGui::TableNextColumn();
            bool isSelf = (ps->guid == playerGuid);
            if (isSelf) ImGui::PushStyleColor(ImGuiCol_Text, colors::kBrightGold);
            const char* nameStr = ps->name.empty() ? "Unknown" : ps->name.c_str();
            ImGui::TextUnformatted(nameStr);
            if (isSelf) ImGui::PopStyleColor();

            ImGui::TableNextColumn(); ImGui::Text("%u", ps->killingBlows);
            ImGui::TableNextColumn(); ImGui::Text("%u", ps->deaths);
            ImGui::TableNextColumn(); ImGui::Text("%u", ps->honorableKills);
            ImGui::TableNextColumn(); ImGui::Text("%u", ps->bonusHonor);

            for (const auto& col : bgColNames) {
                ImGui::TableNextColumn();
                uint32_t val = 0;
                for (const auto& [fieldName, fval] : ps->bgStats) {
                    std::string shortName = fieldName;
                    auto dotPos = fieldName.rfind('.');
                    if (dotPos != std::string::npos) shortName = fieldName.substr(dotPos + 1);
                    if (shortName == col) { val = fval; break; }
                }
                if (val > 0) ImGui::Text("%u", val);
                else ImGui::TextDisabled("-");
            }
        }
        ImGui::EndTable();
    }

    ImGui::End();
}


} // namespace ui
} // namespace wowee
