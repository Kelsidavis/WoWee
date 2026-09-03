#pragma once

#include "terrain_biomes.hpp"
#include <glm/glm.hpp>
#include <string>
#include <vector>

namespace wowee {
namespace editor {

class EditorApp;

enum class PaintMode { Paint, Erase, ReplaceBase };

class EditorUI {
public:
    EditorUI();

    void render(EditorApp& app);
    void processActions(EditorApp& app);
    void openNewTerrainDialog() { showNewDialog_ = true; }
    void openLoadDialog() { showLoadDialog_ = true; }
    void toggleHelp() { showHelp_ = !showHelp_; }

    PaintMode getPaintMode() const { return paintMode_; }

    // Path point capture: when active, next terrain click appends a
    // point. WaitingStart for the first, WaitingMore for any subsequent
    // (the user can keep clicking until they hit Apply or Finish).
    enum class PathCapture { None, WaitingStart, WaitingEnd, WaitingMore };
    PathCapture getPathCapture() const { return pathCapture_; }
    void setPathPoint(const glm::vec3& pos);
    // Backwards-compatible getters: start = first point, end = last.
    glm::vec3 getPathStart() const {
        return pathPoints_.empty() ? glm::vec3(0) : pathPoints_.front();
    }
    glm::vec3 getPathEnd() const {
        return pathPoints_.empty() ? glm::vec3(0) : pathPoints_.back();
    }
    const std::vector<glm::vec3>& getPathPoints() const { return pathPoints_; }
    bool isPathReady() const { return pathPoints_.size() >= 2; }
    float getPathWidth() const { return pathWidth_; }
    void clearPath() { pathPoints_.clear(); pathCapture_ = PathCapture::None; }

private:
    void renderMenuBar(EditorApp& app);
    void renderToolbar(EditorApp& app);
    void renderNewTerrainDialog(EditorApp& app);
    void renderLoadDialog(EditorApp& app);
    void renderSaveDialog(EditorApp& app);
    void renderBrushPanel(EditorApp& app);
    void renderTexturePaintPanel(EditorApp& app);
    void renderObjectPanel(EditorApp& app);
    void renderWaterPanel(EditorApp& app);
    void renderNpcPanel(EditorApp& app);
    void renderQuestPanel(EditorApp& app);
    void renderContextMenu(EditorApp& app);
    void renderMinimap(EditorApp& app);
    void renderPropertiesPanel(EditorApp& app);
    void renderStatusBar(EditorApp& app);

    bool showNewDialog_ = false;
    bool showLoadDialog_ = false;
    bool showSaveDialog_ = false;
    bool showHelp_ = false;
    bool showAbout_ = false;
    bool generateAfterCreate_ = false;

    char newMapNameBuf_[256] = "CustomZone";
    int newTileX_ = 32;
    int newTileY_ = 32;
    float newBaseHeight_ = 100.0f;
    int newBiomeIdx_ = 0;
    bool newRequested_ = false;

    char loadMapNameBuf_[256] = "Azeroth";
    int loadTileX_ = 32;
    int loadTileY_ = 48;
    bool loadRequested_ = false;

    char savePathBuf_[512] = "";

    // Paint panel
    PaintMode paintMode_ = PaintMode::Paint;
    char texFilterBuf_[128] = "";
    int texDirIdx_ = -1; // -1 = all
    std::string selectedTexture_;

    // Object panel
    char objFilterBuf_[128] = "";
    bool showM2s_ = true;
    bool showWMOs_ = true;

    // Path point capture
    PathCapture pathCapture_ = PathCapture::None;
    std::vector<glm::vec3> pathPoints_;
    int pathMode_ = 0; // 0=river, 1=road
    float pathWidth_ = 8.0f, pathDepth_ = 5.0f;
};

} // namespace editor
} // namespace wowee
