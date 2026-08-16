#pragma once

#include "ui/ui_services.hpp"
#include "game/game_handler.hpp"
#include <imgui.h>
#include <string>
#include <functional>
#include <memory>

namespace wowee {
namespace pipeline { class AssetManager; }
namespace rendering { class CharacterPreview; }
namespace ui {

/**
 * Character selection screen UI
 *
 * Displays character list and allows user to select one to play
 */
class CharacterScreen {
public:
    CharacterScreen();

    /**
     * Render the UI
     * @param gameHandler Reference to game handler
     */
    void render(game::GameHandler& gameHandler);

    void setAssetManager(pipeline::AssetManager* am) {
        assetManager_ = am;
        previewInitialized_ = false;
        previewGuid_ = 0;
        previewAppearanceBytes_ = 0;
        previewFacialFeatures_ = 0;
        previewUseFemaleModel_ = false;
        previewEquipHash_ = 0;
    }

    /**
     * Set callback for character selection
     * @param callback Function to call when character is selected (receives character GUID)
     */
    void setOnCharacterSelected(std::function<void(uint64_t)> callback) {
        onCharacterSelected = callback;
    }

    void setOnCreateCharacter(std::function<void()> cb) { onCreateCharacter = std::move(cb); }
    void setOnBack(std::function<void()> cb) { onBack = std::move(cb); }
    void setOnDeleteCharacter(std::function<void(uint64_t)> cb) { onDeleteCharacter = std::move(cb); }

    /// Set services (dependency injection)
    void setServices(const UIServices& services) { services_ = services; }

    /**
     * Reset selection state (e.g., when switching servers)
     */
    void reset() {
        selectedCharacterIndex = -1;
        characterSelected = false;
        selectedCharacterGuid = 0;
        restoredLastCharacter = false;
        newlyCreatedCharacterName.clear();
        statusMessage.clear();
        statusIsError = false;
        deleteConfirmStage = 0;
        previewInitialized_ = false;
        previewGuid_ = 0;
        previewAppearanceBytes_ = 0;
        previewFacialFeatures_ = 0;
        previewUseFemaleModel_ = false;
        previewEquipHash_ = 0;
    }

    /**
     * Check if a character has been selected
     */
    [[nodiscard]] bool hasSelection() const { return characterSelected; }

    /**
     * Get selected character GUID
     */
    [[nodiscard]] uint64_t getSelectedGuid() const { return selectedCharacterGuid; }

    /**
     * Update status message
     */
    void setStatus(const std::string& message, bool isError = false);

    /**
     * Select character by name (used after character creation)
     */
    void selectCharacterByName(const std::string& name);

private:
    UIServices services_;  // Injected service references

    // UI state
    int selectedCharacterIndex = -1;
    bool characterSelected = false;
    uint64_t selectedCharacterGuid = 0;
    bool restoredLastCharacter = false;
    std::string newlyCreatedCharacterName;  // Auto-select this character if set

    // Status
    std::string statusMessage;
    bool statusIsError = false;

    // Callbacks
    std::function<void(uint64_t)> onCharacterSelected;
    std::function<void()> onCreateCharacter;
    std::function<void()> onBack;
    std::function<void(uint64_t)> onDeleteCharacter;
    int deleteConfirmStage = 0;  // 0=none, 1=first warning, 2=final warning

    /**
     * Get faction color based on race
     */
    [[nodiscard]] ImVec4 getFactionColor(game::Race race) const;

    /// AddOns management window (list + enable/disable), opened from the footer.
    bool showAddonsWindow_ = false;
    void renderAddonsWindow();

    /**
     * Persist / restore last selected character GUID
     */
    static std::string getConfigDir();
    void saveLastCharacter(uint64_t guid);
    uint64_t loadLastCharacter();

    // Preview (3D character portrait)
    pipeline::AssetManager* assetManager_ = nullptr;
    std::unique_ptr<rendering::CharacterPreview> preview_;
    bool previewInitialized_ = false;
    uint64_t previewGuid_ = 0;
    uint32_t previewAppearanceBytes_ = 0;
    uint8_t previewFacialFeatures_ = 0;
    bool previewUseFemaleModel_ = false;
    uint64_t previewEquipHash_ = 0;
};

} // namespace ui
} // namespace wowee
