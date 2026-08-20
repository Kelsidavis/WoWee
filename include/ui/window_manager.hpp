// ============================================================
// WindowManager - extracted from GameScreen
// Owns all NPC interaction windows, popup dialogs, and misc
// overlay UI: loot, gossip, quest, vendor, trainer, mail, bank,
// auction house, barber, stable, taxi, escape menu, death screen,
// instance lockouts, achievements, GM ticket, books, titles,
// equipment sets, skills.
// ============================================================
#pragma once
#include "ui/ui_services.hpp"
#include <cstdint>
#include <string>
#include <unordered_map>
#include <functional>
#include <memory>
#include <vector>
#include <imgui.h>
#include <vulkan/vulkan.h>

namespace wowee {
namespace game { class GameHandler; }
namespace pipeline { class AssetManager; }
namespace rendering { class CharacterPreview; }
namespace ui {

class ChatPanel;
class SettingsPanel;
class InventoryScreen;
class SpellbookScreen;

class WindowManager {
public:
    WindowManager() = default;
    ~WindowManager();

    // Callback type for resolving spell icons (spellId, assetMgr) → VkDescriptorSet
    using SpellIconFn = std::function<VkDescriptorSet(uint32_t, pipeline::AssetManager*)>;

    // ---- NPC interaction windows ----

    // ---- Mail and banking ----

    // ---- Popup / overlay windows ----
    /// The game handler is only for the Help button, which has to reach
    /// FrameXML's help frame when that element is handed over.
    void renderEscapeMenu(SettingsPanel& settingsPanel, game::GameHandler& gameHandler);
    void renderInstanceLockouts(game::GameHandler& gameHandler);
    void renderTitlesWindow(game::GameHandler& gameHandler);
    void renderEquipSetWindow(game::GameHandler& gameHandler);
    void renderSkillsWindow(game::GameHandler& gameHandler);
    // Browse/search the GM command reference and send commands to the server.
    void renderGmCommandScreen(game::GameHandler& gameHandler);

    // ---- State owned by this manager ----

    // GM command screen
    bool showGmCommandScreen_ = false;
    char gmSearchBuf_[128] = {};
    int  gmSelectedIndex_ = -1;      // index into kGmCommands, -1 = none
    int  gmArgsForIndex_ = -1;       // which selection the arg buffers belong to
    char gmCommandBuf_[256] = {};    // manual/advanced command line
    int  gmMaxSecurity_ = 4;         // hide commands above this security level
    bool gmManualEdit_ = false;      // edit the raw command line instead of fields
    static constexpr int kGmMaxArgs = 8;
    char gmArgBuf_[kGmMaxArgs][96] = {}; // per-argument text values
    int  gmArgChoice_[kGmMaxArgs] = {};  // per-argument combo selection

    // "Max Out Character" quick action: which parts to apply.
    bool gmMaxLevel_  = true;
    bool gmMaxSpells_ = true;
    bool gmMaxTalents_ = true;
    bool gmMaxSkills_ = true;
    bool gmMaxGear_   = true;
    bool gmMaxGold_   = false;
    // Commands queued by a quick action, drained one per frame so a burst of
    // .additem/.learn commands doesn't trip server chat flood protection.
    std::vector<std::string> gmPendingCmds_;
    size_t gmPendingPos_ = 0;
    // Build + queue the max-out command sequence for the current class/expansion.
    void queueMaxOutCharacter(game::GameHandler& gameHandler);

    // Instance lockouts
    bool showInstanceLockouts_ = false;


    // Skills / Professions
    bool showSkillsWindow_ = false;

    // Titles
    bool showTitlesWindow_ = false;

    // Equipment Sets
    bool showEquipSetWindow_ = false;

    // GM Ticket

    // Death screen
    static constexpr float kForcedReleaseSec = 360.0f;

    // Escape menu
    bool showEscapeMenu = false;

    bool vendorBagsOpened_ = false;
    bool guildBankBagsOpened_ = false;

    // Barber shop
    struct BarberStyleOption {
        uint32_t entryId = 0;       // BarberShopStyle.dbc ID sent to the server
        uint8_t appearanceId = 0;   // CharSections/geoset variation used by preview
        std::string name;
    };
    std::vector<BarberStyleOption> barberHairStyles_;
    std::vector<BarberStyleOption> barberFacialStyles_;
    std::vector<BarberStyleOption> barberSkinStyles_;
    std::vector<uint8_t> barberHairColors_;
    int barberHairStyle_ = 0;
    int barberHairColor_ = 0;
    int barberFacialHair_ = 0;
    int barberSkinColor_ = 0;
    uint8_t barberOrigHairStyle_ = 0;
    uint8_t barberOrigHairColor_ = 0;
    uint8_t barberOrigFacialHair_ = 0;
    uint8_t barberOrigSkinColor_ = 0;
    uint8_t barberColorsForHairStyle_ = 0xFF;
    float barberBaseCost_ = 0.0f;
    bool barberInitialized_ = false;

    // Barber state, separated from the window that used to hold it.
    //
    // The style lists, the originals and the cost table were all built inside
    // renderBarberShopWindow, so they existed only while this client drew the
    // chair. FrameXML's barber asks the same questions through
    // GetBarberShopStyleInfo and GetBarberShopTotalCost, and with the panel
    // handed over that render never runs - the answers have to come from
    // somewhere that does not depend on who is drawing.
    void ensureBarberState(game::GameHandler& gameHandler);
    void rebuildBarberHairColors(uint8_t hairStyle, uint8_t preferredColor,
                                 uint32_t raceId, uint32_t sexId);
    static int barberFindAppearance(const std::vector<BarberStyleOption>& options, uint8_t id);
    static uint8_t barberSelectedAppearance(const std::vector<BarberStyleOption>& options,
                                            int index, uint8_t fallback);
    /// The four appearance values the current selections resolve to.
    struct BarberSelection {
        uint8_t hairStyle = 0, hairColor = 0, facialHair = 0, skin = 0;
    };
    BarberSelection barberSelection(game::GameHandler& gameHandler);

    /// The interface's own barber panel reads these through LuaServices.
    /// Selectors follow FrameXML's BarberShopFrameSelector IDs: 1 hair style,
    /// 2 hair colour, 3 facial hair, 4 skin.
    bool barberStyleInfo(game::GameHandler& gameHandler, int selector,
                         std::string& name, bool& isCurrent);
    void barberCycleStyle(game::GameHandler& gameHandler, int selector, int direction);
    uint32_t barberTotalCostCopper(game::GameHandler& gameHandler);
    void barberResetSelections(game::GameHandler& gameHandler);
    /// Buy the current selection. The Okay button on either barber calls this;
    /// FrameXML's reaches it through ApplyBarberShopStyle.
    void barberApplySelection(game::GameHandler& gameHandler);

    // ItemExtendedCost.dbc cache
    // UIServices injection (Phase B singleton breaking)
    void setServices(const UIServices& services) { services_ = services; }

private:
    UIServices services_;
    // Resolve an achievement's SpellIcon.dbc ID to an ImGui texture (lazy BLP load + cache).
};

} // namespace ui
} // namespace wowee
