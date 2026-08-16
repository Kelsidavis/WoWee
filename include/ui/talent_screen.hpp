#pragma once

#include "game/game_handler.hpp"
#include <imgui.h>
#include <vulkan/vulkan.h>
#include <unordered_map>
#include <string>
#include <array>
#include <cstdint>

namespace wowee {
namespace pipeline { class AssetManager; }

namespace ui {

class TalentScreen {
public:
    void render(game::GameHandler& gameHandler);
    [[nodiscard]] bool isOpen() const { return open; }
    void toggle() { open = !open; }
    void setOpen(bool o) { open = o; }

private:
    void renderTalentTrees(game::GameHandler& gameHandler);
    void renderTalentTree(game::GameHandler& gameHandler, uint32_t tabId,
                          const std::string& bgFile);
    void renderTalent(game::GameHandler& gameHandler,
                      const game::GameHandler::TalentEntry& talent,
                      uint32_t pointsInTree);

    void loadSpellDBC(pipeline::AssetManager* assetManager);
    void loadSpellIconDBC(pipeline::AssetManager* assetManager);
    void loadGlyphPropertiesDBC(pipeline::AssetManager* assetManager);
    void renderGlyphs(game::GameHandler& gameHandler);
    VkDescriptorSet getSpellIcon(uint32_t iconId, pipeline::AssetManager* assetManager);

    bool open = false;
    bool nKeyWasDown = false;

    // DBC caches
    bool spellDbcLoaded = false;
    bool iconDbcLoaded = false;
    bool glyphDbcLoaded = false;
    std::unordered_map<uint32_t, uint32_t> spellIconIds;       // spellId -> iconId
    std::unordered_map<uint32_t, std::string> spellIconPaths;  // iconId -> path
    std::unordered_map<uint32_t, VkDescriptorSet> spellIconCache;  // iconId -> texture
    std::unordered_map<uint32_t, std::string> spellTooltips;   // spellId -> short tooltip text
    std::unordered_map<uint32_t, std::string> spellDescriptions;  // spellId -> full effect description
    std::unordered_map<uint32_t, VkDescriptorSet> bgTextureCache_;  // tabId -> bg texture

    // Resolve a rank spell's description into display-ready text: prefers the full
    // Description, falls back to the short tooltip, then substitutes WoW's $-tokens
    // via the shared GameHandler::formatSpellDescription.
    std::string describeRankSpell(game::GameHandler& gameHandler, uint32_t spellId) const;

    // Talent learn confirmation
    bool talentConfirmOpen_ = false;
    uint32_t pendingTalentId_ = 0;
    uint32_t pendingTalentRank_ = 0;
    std::string pendingTalentName_;

    // GlyphProperties.dbc cache: glyphId -> { spellId, isMajor }
    struct GlyphInfo { uint32_t spellId = 0; bool isMajor = false; };
    std::unordered_map<uint32_t, GlyphInfo> glyphProperties_;  // glyphId -> info
};

} // namespace ui
} // namespace wowee
