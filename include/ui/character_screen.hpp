#pragma once

#include "game/game_handler.hpp"
#include <imgui.h>
#include <string>
#include <functional>

namespace wowee { namespace ui {

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

    /**
     * Check if a character has been selected
     */
    bool hasSelection() const { return characterSelected; }

    /**
     * Get selected character GUID
     */
    uint64_t getSelectedGuid() const { return selectedCharacterGuid; }

    /**
     * Update status message
     */
    void setStatus(const std::string& message);

private:
    // UI state
    int selectedCharacterIndex = -1;
    bool characterSelected = false;
    uint64_t selectedCharacterGuid = 0;
    bool restoredLastCharacter = false;

    // Status
    std::string statusMessage;

    // Callbacks
    std::function<void(uint64_t)> onCharacterSelected;
    std::function<void()> onCreateCharacter;
    std::function<void()> onBack;
    std::function<void(uint64_t)> onDeleteCharacter;
    bool confirmDelete = false;

    /**
     * Get faction color based on race
     */
    ImVec4 getFactionColor(game::Race race) const;

    /**
     * Persist / restore last selected character GUID
     */
    static std::string getConfigDir();
    void saveLastCharacter(uint64_t guid);
    uint64_t loadLastCharacter();
};

}} // namespace wowee::ui
