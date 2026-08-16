#pragma once

#include "auth/auth_handler.hpp"
#include <imgui.h>
#include <string>
#include <functional>

namespace wowee { namespace ui {

/**
 * Realm selection screen UI
 *
 * Displays available realms and allows user to select one
 */
class RealmScreen {
public:
    RealmScreen();

    /**
     * Render the UI
     * @param authHandler Reference to auth handler
     */
    void render(auth::AuthHandler& authHandler);

    /**
     * Set callback for realm selection
     * @param callback Function to call when realm is selected (receives realm name and address)
     */
    void setOnRealmSelected(std::function<void(const std::string&, const std::string&)> callback) {
        onRealmSelected = callback;
    }

    void setOnBack(std::function<void()> cb) { onBack = std::move(cb); }

    /**
     * Reset selection state (e.g., when switching servers)
     */
    void reset() {
        selectedRealmIndex = -1;
        realmSelected = false;
        autoSelectAttempted = false;
        selectedRealmName.clear();
        selectedRealmAddress.clear();
        statusMessage.clear();
    }

    /**
     * Reset for back-navigation from character screen.
     * Preserves autoSelectAttempted so single-realm auto-connect doesn't re-fire.
     */
    void resetForBack() {
        selectedRealmIndex = -1;
        realmSelected = false;
        selectedRealmName.clear();
        selectedRealmAddress.clear();
        statusMessage.clear();
    }

    /**
     * Check if a realm has been selected
     */
    [[nodiscard]] bool hasSelection() const { return realmSelected; }

    /**
     * Get selected realm info
     */
    [[nodiscard]] const std::string& getSelectedName() const { return selectedRealmName; }
    [[nodiscard]] const std::string& getSelectedAddress() const { return selectedRealmAddress; }

private:
    // UI state
    int selectedRealmIndex = -1;
    bool realmSelected = false;
    bool autoSelectAttempted = false;
    std::string selectedRealmName;
    std::string selectedRealmAddress;

    // Status
    std::string statusMessage;

    // Callbacks
    std::function<void(const std::string&, const std::string&)> onRealmSelected;
    std::function<void()> onBack;

    /**
     * Update status message
     */
    void setStatus(const std::string& message);

    /**
     * Get realm status text
     */
    [[nodiscard]] const char* getRealmStatus(uint8_t flags) const;

    /**
     * Get population color
     */
    [[nodiscard]] ImVec4 getPopulationColor(float population) const;
};

}} // namespace wowee::ui
