#pragma once

#include "ui/ui_services.hpp"
#include <imgui.h>
#include <vulkan/vulkan.h>
#include <string>
#include <vector>
#include <cstdint>
#include <functional>

namespace wowee {
namespace game { class GameHandler; }
namespace pipeline { class AssetManager; }
namespace ui {

class SettingsPanel;
class SpellbookScreen;
class InventoryScreen;

/**
 * Combat UI overlay manager (extracted from GameScreen)
 *
 * Owns all combat-related rendering:
 *   cast bar, cooldown tracker, raid warning overlay, floating combat text,
 *   DPS/HPS meter, buff bar, battleground score HUD, combat log,
 *   threat window, BG scoreboard.
 */
class CombatUI {
public:
    CombatUI() = default;

    // ---- Callback type for spell icon lookup (stays in GameScreen) ----
    using SpellIconFn = std::function<VkDescriptorSet(uint32_t spellId, pipeline::AssetManager*)>;

    // ---- Toggle booleans (written by slash commands / escape handler / settings) ----
    bool showCombatLog_ = false;
    bool showThreatWindow_ = false;
    bool showBgScoreboard_ = false;

    // ---- Raid Warning / Boss Emote big-text overlay ----
    struct RaidWarnEntry {
        std::string text;
        float age = 0.0f;
        bool isBossEmote = false;
        static constexpr float LIFETIME = 5.0f;
    };
    std::vector<RaidWarnEntry> raidWarnEntries_;
    bool raidWarnCallbackSet_ = false;
    size_t raidWarnChatSeenCount_ = 0;

    // ---- DPS meter state ----
    float dpsCombatAge_ = 0.0f;
    bool dpsWasInCombat_ = false;
    float dpsEncounterDamage_ = 0.0f;
    float dpsEncounterHeal_   = 0.0f;
    size_t dpsLogSeenCount_   = 0;
    // Window position. Negative x means "not placed by the user yet", in which
    // case the meter parks itself under the target frame each frame and follows
    // it as that frame resizes. The first drag latches the user's position and
    // auto-placement stops for good.
    ImVec2 dpsMeterPos_{-1.0f, -1.0f};
    bool dpsMeterUserPositioned_ = false;

    // ---- Public render methods ----
    void renderCooldownTracker(game::GameHandler& gameHandler,
                               const SettingsPanel& settings,
                               const SpellIconFn& getSpellIcon);
    void renderRaidWarningOverlay(game::GameHandler& gameHandler);
    void renderCombatText(game::GameHandler& gameHandler);
    /// targetFrameBottom: screen Y of the bottom of the target frame, or a
    /// negative value when nothing is targeted, which parks the meter at the
    /// height the frame would occupy so it does not jump around.
    void renderDPSMeter(game::GameHandler& gameHandler,
                        const SettingsPanel& settings,
                        float targetFrameBottom);

    // Saved-position access for settings persistence (see GameScreen settings I/O).
    [[nodiscard]] ImVec2 getDPSMeterPos() const { return dpsMeterUserPositioned_ ? dpsMeterPos_ : ImVec2(-1.0f, -1.0f); }
    void setDPSMeterPos(float x, float y) {
        if (x < 0.0f || y < 0.0f) return;
        dpsMeterPos_ = ImVec2(x, y);
        dpsMeterUserPositioned_ = true;
    }
    // inventoryScreen supplies the weapon item icons shown for temporary weapon enchants.
    void renderBattlegroundScore(game::GameHandler& gameHandler);
    void renderCombatLog(game::GameHandler& gameHandler,
                         SpellbookScreen& spellbookScreen);
    void renderThreatWindow(game::GameHandler& gameHandler);
    void renderBgScoreboard(game::GameHandler& gameHandler);

    // UIServices injection (Phase B singleton breaking)
    void setServices(const UIServices& services) { services_ = services; }

private:
    UIServices services_;
};

} // namespace ui
} // namespace wowee
