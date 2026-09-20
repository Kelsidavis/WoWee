#include "editor_app.hpp"
#include "adt_writer.hpp"
#include "zone_manifest.hpp"
#include "content_pack.hpp"
#include "wowee_terrain.hpp"
#include "texture_exporter.hpp"
#include "dbc_exporter.hpp"
#include "pipeline/wowee_model.hpp"
#include "pipeline/wowee_building.hpp"
#include "pipeline/wowee_collision.hpp"
#include "pipeline/wmo_loader.hpp"
#include "sql_exporter.hpp"
#include "server_module_gen.hpp"
#include "core/coordinates.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <nlohmann/json.hpp>
#include "rendering/vk_context.hpp"
#include "pipeline/adt_loader.hpp"
#include "pipeline/wdt_loader.hpp"
#include "pipeline/terrain_mesh.hpp"
#include "core/logger.hpp"
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_vulkan.h>
#include <algorithm>
#include <chrono>
#include <sstream>
#include <unordered_set>

namespace wowee {
namespace editor {

EditorApp::EditorApp() = default;
EditorApp::~EditorApp() { shutdown(); }

bool EditorApp::initialize(const std::string& dataPath) {
    dataPath_ = dataPath;

    core::WindowConfig wc;
    wc.title = "Wowee World Editor";
    wc.width = 1600;
    wc.height = 900;
    window_ = std::make_unique<core::Window>(wc);
    if (!window_->initialize()) {
        LOG_ERROR("Failed to initialize window");
        return false;
    }

    assetManager_ = std::make_unique<pipeline::AssetManager>();
    if (!assetManager_->initialize(dataPath)) {
        LOG_ERROR("Failed to initialize asset manager with path: ", dataPath);
        return false;
    }

    initImGui();

    auto* vkCtx = window_->getVkContext();
    camera_.getCamera().setAspectRatio(window_->getAspectRatio());
    camera_.setPosition(glm::vec3(0.0f, 0.0f, 300.0f));
    camera_.setYawPitch(0.0f, -30.0f);

    if (!viewport_.initialize(vkCtx, assetManager_.get(), &camera_.getCamera())) {
        LOG_ERROR("Failed to initialize editor viewport");
        return false;
    }

    assetBrowser_.initialize(assetManager_.get());
    npcPresets_.initialize(assetManager_.get());

    LOG_INFO("Editor initialized (data: ", dataPath, ")");
    return true;
}

void EditorApp::run() {
    auto lastTime = std::chrono::steady_clock::now();

    while (!window_->shouldClose()) {
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - lastTime).count();
        lastTime = now;
        dt = std::min(dt, 0.1f);

        processEvents();

        auto* vkCtx = window_->getVkContext();
        if (vkCtx->isSwapchainDirty()) {
            int w = window_->getWidth();
            int h = window_->getHeight();
            if (w > 0 && h > 0) {
                (void)vkCtx->recreateSwapchain(w, h);
                camera_.getCamera().setAspectRatio(static_cast<float>(w) / h);
            }
        }

        camera_.update(dt);
        updateTerrainEditing(dt);

        // Handle pending UI actions
        ui_.processActions(*this);

        updateToasts(dt);

        // Auto-save: any unsaved change (terrain edits, object/NPC placement,
        // quest edits) qualifies. Previously only terrain changes counted.
        if (autoSaveEnabled_ && terrain_.isLoaded()) {
            bool dirty = terrainEditor_.hasUnsavedChanges() || autoSavePendingChanges_;
            if (dirty) {
                autoSaveTimer_ += dt;
                if (autoSaveTimer_ >= autoSaveInterval_) {
                    autoSaveTimer_ = 0.0f;
                    autoSavePendingChanges_ = false;
                    quickSave();
                    showToast("Auto-saved", 2.0f);
                    LOG_INFO("Auto-saved zone");
                }
            }
        }

        // Refresh dirty terrain chunks
        refreshDirtyChunks();

        // Periodic GPU model cache cleanup. Models that lost all instances
        // (e.g. user removed every spawn of one creature) get evicted after
        // the renderer's grace period. Without this the editor would slowly
        // accumulate GPU memory across long sessions.
        static float modelCleanupTimer = 0.0f;
        modelCleanupTimer += dt;
        if (modelCleanupTimer >= 30.0f) {
            modelCleanupTimer = 0.0f;
            if (auto* m2 = viewport_.getM2Renderer()) m2->cleanupUnusedModels();
        }

        // Track object and NPC counts separately
        size_t objCount = objectPlacer_.objectCount();
        size_t npcCount = npcSpawner_.spawnCount();
        bool objChanged = (objCount != lastObjCount_);
        int npcSelIdx = npcSpawner_.getSelectedIndex();
        bool npcSelChanged = (npcSelIdx != lastNpcSelIdx_);
        bool npcChanged = (npcCount != lastNpcCount_) || objectsDirty_ || npcSelChanged;

        if (npcChanged) {
            // NPC markers are cheap - always update
            viewport_.updateNpcMarkers(npcSpawner_.getSpawns());
            lastNpcCount_ = npcCount;
            lastNpcSelIdx_ = npcSelIdx;
        }

        // Show gizmo arrows on selected object or NPC. NPCs only support move
        // (rotation is via the orientation slider; scale via the slider).
        auto& gizmo = viewport_.getGizmo();
        if (auto* sel = objectPlacer_.getSelected()) {
            gizmo.setTarget(sel->position, sel->scale);
        } else if (auto* npc = npcSpawner_.getSelected()) {
            gizmo.setTarget(npc->position, npc->scale);
        } else {
            gizmo.setMode(TransformMode::None);
        }

        // Patrol path visualization for the selected NPC.
        // Adds a loop-back to the start so users can see the cycle the creature follows.
        if (auto* selNpc = npcSpawner_.getSelected();
            selNpc && !selNpc->patrolPath.empty()) {
            std::vector<glm::vec3> pts;
            pts.reserve(selNpc->patrolPath.size() + 2);
            pts.push_back(selNpc->position);
            for (const auto& wp : selNpc->patrolPath) pts.push_back(wp.position);
            if (selNpc->patrolPath.size() >= 2) pts.push_back(selNpc->position);
            viewport_.setPatrolPath(pts);
        } else {
            viewport_.clearPatrolPath();
        }

        uint32_t imageIndex = 0;
        VkCommandBuffer cmd = vkCtx->beginFrame(imageIndex);
        if (cmd == VK_NULL_HANDLE) continue;

        // Rebuild objects AFTER beginFrame so instance SSBO uses correct frame index
        // Debounce: wait 0.5s after last change before rebuilding to avoid
        // clear+reload cycle on every click during rapid NPC placement
        static float rebuildTimer = 0.0f;
        if (objChanged || objectsDirty_) {
            rebuildTimer = 0.5f;
            objectsDirty_ = false;
            lastObjCount_ = objCount;
            lastNpcCount_ = npcCount;
        }
        if (rebuildTimer > 0.0f) {
            rebuildTimer -= dt;
            if (rebuildTimer <= 0.0f) {
                rebuildTimer = 0.0f;
                if (objectPlacer_.objectCount() > 0 || npcSpawner_.spawnCount() > 0) {
                    vkDeviceWaitIdle(vkCtx->getDevice());
                    viewport_.rebuildObjects(objectPlacer_.getObjects(), npcSpawner_.getSpawns());
                }
            }
        }

        // Update M2 animations AFTER beginFrame (so getCurrentFrame is correct)
        viewport_.update(dt);

        ImGui_ImplVulkan_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        ui_.render(*this);

        if (showQuitConfirm_) {
            ImGui::OpenPopup("Unsaved Changes");
            if (ImGui::BeginPopupModal("Unsaved Changes", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
                ImGui::Text("You have unsaved changes. Save before quitting?");
                ImGui::Separator();
                if (ImGui::Button("Save & Quit", ImVec2(120, 0))) {
                    quickSave();
                    window_->setShouldClose(true);
                    ImGui::CloseCurrentPopup();
                }
                ImGui::SameLine();
                if (ImGui::Button("Quit", ImVec2(80, 0))) {
                    window_->setShouldClose(true);
                    ImGui::CloseCurrentPopup();
                }
                ImGui::SameLine();
                if (ImGui::Button("Cancel", ImVec2(80, 0))) {
                    showQuitConfirm_ = false;
                    ImGui::CloseCurrentPopup();
                }
                ImGui::EndPopup();
            }
        }

        ImGui::Render();

        VkRenderPassBeginInfo rpInfo{};
        rpInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        rpInfo.renderPass = vkCtx->getImGuiRenderPass();
        rpInfo.framebuffer = vkCtx->getSwapchainFramebuffers()[imageIndex];
        rpInfo.renderArea.offset = {0, 0};
        rpInfo.renderArea.extent = vkCtx->getSwapchainExtent();

        VkClearValue clearValues[4]{};
        float cr, cg, cb;
        viewport_.getClearColor(cr, cg, cb);
        clearValues[0].color = {{cr, cg, cb, 1.0f}};
        clearValues[1].depthStencil = {1.0f, 0};
        rpInfo.clearValueCount = 2;
        rpInfo.pClearValues = clearValues;

        vkCmdBeginRenderPass(cmd, &rpInfo, VK_SUBPASS_CONTENTS_INLINE);

        auto ext = vkCtx->getSwapchainExtent();
        VkViewport vp{};
        vp.width = static_cast<float>(ext.width);
        vp.height = static_cast<float>(ext.height);
        vp.minDepth = 0.0f;
        vp.maxDepth = 1.0f;
        vkCmdSetViewport(cmd, 0, 1, &vp);

        VkRect2D scissor{};
        scissor.extent = ext;
        vkCmdSetScissor(cmd, 0, 1, &scissor);

        viewport_.render(cmd);
        ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), cmd);

        vkCmdEndRenderPass(cmd);
        vkCtx->endFrame(cmd, imageIndex);
    }
}

void EditorApp::shutdown() {
    if (!window_) return;
    auto* vkCtx = window_->getVkContext();
    if (vkCtx) vkDeviceWaitIdle(vkCtx->getDevice());

    viewport_.shutdown();
    shutdownImGui();

    if (assetManager_) {
        assetManager_->shutdown();
        assetManager_.reset();
    }
    window_.reset();
}

void EditorApp::processEvents() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        ImGui_ImplSDL3_ProcessEvent(&event);

        if (event.type == SDL_EVENT_QUIT) {
            // Confirm-on-quit fires for any unsaved change - terrain edits OR
            // object/NPC/quest changes (autoSavePendingChanges_).
            bool dirty = terrainEditor_.hasUnsavedChanges() || autoSavePendingChanges_;
            if (terrain_.isLoaded() && dirty) {
                showQuitConfirm_ = true;
            } else {
                window_->setShouldClose(true);
                return;
            }
        }

        if ((event.type >= SDL_EVENT_WINDOW_FIRST && event.type <= SDL_EVENT_WINDOW_LAST)) {
            if (event.type == SDL_EVENT_WINDOW_RESIZED) {
                window_->setSize(event.window.data1, event.window.data2);
                window_->getVkContext()->markSwapchainDirty();
            }
        }

        auto& io = ImGui::GetIO();

        if (event.type == SDL_EVENT_KEY_DOWN || event.type == SDL_EVENT_KEY_UP) {
            if (event.type == SDL_EVENT_KEY_DOWN) {
                auto sc = event.key.scancode;
                if (sc == SDL_SCANCODE_F3) setWireframe(!isWireframe());
                if (sc == SDL_SCANCODE_F5) saveBookmark("");
                if (sc == SDL_SCANCODE_HOME) centerOnTerrain();
                // Number keys switch modes (when not typing in ImGui)
                if (!io.WantCaptureKeyboard) {
                    if (sc == SDL_SCANCODE_1) setMode(EditorMode::Sculpt);
                    if (sc == SDL_SCANCODE_2) setMode(EditorMode::Paint);
                    if (sc == SDL_SCANCODE_3) setMode(EditorMode::PlaceObject);
                    if (sc == SDL_SCANCODE_4) setMode(EditorMode::Water);
                    if (sc == SDL_SCANCODE_5) setMode(EditorMode::NPC);
                    if (sc == SDL_SCANCODE_6) setMode(EditorMode::Quest);
                    // Bracket keys adjust brush size
                    if (sc == SDL_SCANCODE_LEFTBRACKET) {
                        auto& bs = terrainEditor_.brush().settings();
                        bs.radius = std::max(5.0f, bs.radius - 10.0f);
                    }
                    if (sc == SDL_SCANCODE_RIGHTBRACKET) {
                        auto& bs = terrainEditor_.brush().settings();
                        bs.radius = std::min(200.0f, bs.radius + 10.0f);
                    }
                }
                // F1 handled by UI (showHelp_ toggle)
                // F1 = toggle help
                if (sc == SDL_SCANCODE_F1 && !io.WantCaptureKeyboard)
                    ui_.toggleHelp();
                // Transform shortcuts (Blender-style)
                if (objectPlacer_.getSelected()) {
                    if (sc == SDL_SCANCODE_G) startGizmoMode(TransformMode::Move);
                    if (sc == SDL_SCANCODE_R) startGizmoMode(TransformMode::Rotate);
                    if (sc == SDL_SCANCODE_T) startGizmoMode(TransformMode::Scale);
                    if (sc == SDL_SCANCODE_X) setGizmoAxis(TransformAxis::X);
                    if (sc == SDL_SCANCODE_Y) setGizmoAxis(TransformAxis::Y);
                    if (sc == SDL_SCANCODE_Z && !(event.key.mod & SDL_KMOD_CTRL))
                        setGizmoAxis(TransformAxis::Z);
                    if (sc == SDL_SCANCODE_ESCAPE) {
                        viewport_.getGizmo().endDrag();
                        viewport_.getGizmo().setMode(TransformMode::None);
                        objectPlacer_.clearSelection();
                        npcSpawner_.clearSelection();
                        ui_.clearPath();
                        if (pendingCrater_.active) {
                            pendingCrater_.active = false;
                            showToast("Crater placement cancelled");
                        }
                    }
                }
                if (sc == SDL_SCANCODE_DELETE) {
                    if (objectPlacer_.getSelected()) {
                        objectPlacer_.deleteSelected();
                        objectsDirty_ = true; autoSavePendingChanges_ = true;
                    } else if (npcSpawner_.getSelected()) {
                        npcSpawner_.removeCreature(npcSpawner_.getSelectedIndex());
                        objectsDirty_ = true; autoSavePendingChanges_ = true;
                    }
                }
                if (sc == SDL_SCANCODE_S && (event.key.mod & SDL_KMOD_CTRL))
                    quickSave();
                if (sc == SDL_SCANCODE_E && (event.key.mod & SDL_KMOD_CTRL) &&
                    (event.key.mod & SDL_KMOD_SHIFT) && terrain_.isLoaded()) {
                    exportContentPack("output/" + loadedMap_ + ".wcp");
                }
                if (sc == SDL_SCANCODE_N && (event.key.mod & SDL_KMOD_CTRL))
                    ui_.openNewTerrainDialog();
                if (sc == SDL_SCANCODE_O && (event.key.mod & SDL_KMOD_CTRL))
                    ui_.openLoadDialog();
                if (sc == SDL_SCANCODE_A && (event.key.mod & SDL_KMOD_CTRL)) {
                    objectPlacer_.selectAll();
                    showToast("Selected " + std::to_string(objectPlacer_.selectionCount()) + " objects");
                }
                // Ctrl+D = duplicate selected object or NPC, offset by (10,10) on the ground.
                if (sc == SDL_SCANCODE_D && (event.key.mod & SDL_KMOD_CTRL)) {
                    if (auto* sel = objectPlacer_.getSelected()) {
                        std::string dupPath = sel->path;
                        glm::vec3 dupPos = sel->position + glm::vec3(10.0f, 10.0f, 0.0f);
                        glm::vec3 dupRot = sel->rotation;
                        float dupScale = sel->scale;
                        auto dupType = sel->type;
                        objectPlacer_.clearSelection();
                        objectPlacer_.setActivePath(dupPath, dupType);
                        objectPlacer_.setPlacementScale(dupScale);
                        objectPlacer_.setPlacementRotationY(dupRot.y);
                        objectPlacer_.placeObject(dupPos);
                        objectsDirty_ = true; autoSavePendingChanges_ = true;
                        showToast("Duplicated object");
                    } else if (auto* npc = npcSpawner_.getSelected()) {
                        CreatureSpawn copy = *npc;
                        copy.position += glm::vec3(10, 10, 0);
                        npcSpawner_.placeCreature(copy);
                        objectsDirty_ = true; autoSavePendingChanges_ = true;
                        showToast("Duplicated NPC");
                    }
                }
                // W: add patrol waypoint at cursor for the selected NPC (no modifiers,
                // editor must be in NPC mode and an NPC must be selected with Patrol behavior).
                if (sc == SDL_SCANCODE_W && mode_ == EditorMode::NPC &&
                    !(event.key.mod & (SDL_KMOD_CTRL | SDL_KMOD_ALT | SDL_KMOD_SHIFT))) {
                    auto* sel = npcSpawner_.getSelected();
                    if (sel && sel->behavior == CreatureBehavior::Patrol &&
                        terrainEditor_.brush().isActive()) {
                        PatrolPoint pp;
                        pp.position = terrainEditor_.brush().getPosition();
                        pp.waitTimeMs = 2000.0f;
                        sel->patrolPath.push_back(pp);
                        showToast("Added patrol waypoint #" + std::to_string(sel->patrolPath.size()));
                    }
                }
                // Ctrl+Y = Redo (alternate binding)
                if (sc == SDL_SCANCODE_Y && (event.key.mod & SDL_KMOD_CTRL)) {
                    if (terrainEditor_.history().canRedo()) {
                        terrainEditor_.redo();
                        showToast("Redo");
                    }
                }
                if (sc == SDL_SCANCODE_Z && (event.key.mod & SDL_KMOD_CTRL)) {
                    bool isRedo = (event.key.mod & SDL_KMOD_SHIFT) != 0;
                    if (isRedo) {
                        if (terrainEditor_.history().canRedo()) {
                            terrainEditor_.redo();
                            showToast("Redo");
                        }
                    } else {
                        // Ctrl+Z = Undo
                        if (mode_ == EditorMode::PlaceObject || mode_ == EditorMode::NPC) {
                            if (objectPlacer_.canUndoPlace()) {
                                objectPlacer_.undoLastPlace();
                                objectsDirty_ = true; autoSavePendingChanges_ = true;
                                showToast("Undo placement");
                            }
                        } else if (terrainEditor_.history().canUndo()) {
                            terrainEditor_.undo();
                            showToast("Undo");
                        }
                    }
                }
            }
            // Use WantTextInput (true only while typing into a text widget)
            // instead of WantCaptureKeyboard (true whenever any ImGui panel
            // has focus). Otherwise hovering over a panel silently disables
            // the WASD/QE flycam, which was the practical user complaint.
            if (!io.WantTextInput)
                camera_.processKeyEvent(event.key);
        }

        if (event.type == SDL_EVENT_MOUSE_MOTION && !io.WantCaptureMouse) {
            // Gizmo drag takes priority over camera
            auto& giz = viewport_.getGizmo();
            if (event.motion.state & SDL_BUTTON_MMASK) {
                // Middle mouse = orbit around brush/terrain point
                auto& brush = terrainEditor_.brush();
                glm::vec3 pivot = brush.isActive() ? brush.getPosition() : camera_.getCamera().getPosition() + camera_.getCamera().getForward() * 100.0f;
                camera_.processMiddleMouseMotion(event.motion.xrel, event.motion.yrel, pivot);
            } else if (giz.isDragging()) {
                auto ext = window_->getVkContext()->getSwapchainExtent();
                giz.updateDrag(glm::vec2(static_cast<float>(event.motion.x),
                                          static_cast<float>(event.motion.y)),
                               camera_.getCamera(),
                               static_cast<float>(ext.width),
                               static_cast<float>(ext.height));
                // Apply transform to selected object or NPC.
                if (auto* sel = objectPlacer_.getSelected()) {
                    if (giz.getMode() == TransformMode::Move) {
                        sel->position += giz.getMoveDelta();
                        giz.beginDrag(glm::vec2(event.motion.x, event.motion.y));
                    } else if (giz.getMode() == TransformMode::Rotate) {
                        sel->rotation += giz.getRotateDelta();
                        giz.beginDrag(glm::vec2(event.motion.x, event.motion.y));
                    } else if (giz.getMode() == TransformMode::Scale) {
                        sel->scale = std::max(0.1f, sel->scale + giz.getScaleDelta());
                        giz.beginDrag(glm::vec2(event.motion.x, event.motion.y));
                    }
                    giz.setTarget(sel->position, sel->scale);
                    objectsDirty_ = true; autoSavePendingChanges_ = true;
                } else if (auto* npc = npcSpawner_.getSelected()) {
                    if (giz.getMode() == TransformMode::Move) {
                        npc->position += giz.getMoveDelta();
                        giz.beginDrag(glm::vec2(event.motion.x, event.motion.y));
                    } else if (giz.getMode() == TransformMode::Rotate) {
                        // Apply yaw component only - NPCs only have orientation.
                        npc->orientation += glm::degrees(giz.getRotateDelta().z);
                        while (npc->orientation >= 360.0f) npc->orientation -= 360.0f;
                        while (npc->orientation < 0.0f) npc->orientation += 360.0f;
                        giz.beginDrag(glm::vec2(event.motion.x, event.motion.y));
                    } else if (giz.getMode() == TransformMode::Scale) {
                        npc->scale = std::max(0.1f, npc->scale + giz.getScaleDelta());
                        giz.beginDrag(glm::vec2(event.motion.x, event.motion.y));
                    }
                    giz.setTarget(npc->position, npc->scale);
                    objectsDirty_ = true; autoSavePendingChanges_ = true;
                }
            } else {
                camera_.processMouseMotion(event.motion.xrel, event.motion.yrel);
            }
        }

        if ((event.type == SDL_EVENT_MOUSE_BUTTON_DOWN || event.type == SDL_EVENT_MOUSE_BUTTON_UP) && !io.WantCaptureMouse) {
            // Right-click on selected objects = context menu
            if (event.button.button == SDL_BUTTON_RIGHT && event.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
                auto& giz = viewport_.getGizmo();
                if (giz.isDragging()) {
                    giz.endDrag();
                    giz.setMode(TransformMode::None);
                } else if (objectPlacer_.getSelected() || npcSpawner_.getSelected()) {
                    openContextMenu_ = true;
                } else {
                    camera_.processMouseButton(event.button);
                }
            } else if (event.button.button == SDL_BUTTON_RIGHT && event.type == SDL_EVENT_MOUSE_BUTTON_UP) {
                if (!objectPlacer_.getSelected() && !npcSpawner_.getSelected())
                    camera_.processMouseButton(event.button);
            } else {
                // Only pass to camera if gizmo not active
                auto& giz = viewport_.getGizmo();
                if (!giz.isDragging())
                    camera_.processMouseButton(event.button);
            }

            // Left click
            if (event.button.button == SDL_BUTTON_LEFT && terrain_.isLoaded()) {
                auto& giz = viewport_.getGizmo();
                // End gizmo drag on left click
                if (giz.isDragging() && event.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
                    giz.endDrag();
                    giz.setMode(TransformMode::None);
                } else if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
                    // Pending crater placement: take precedence over the
                    // mode-based click handling below so the next click
                    // anywhere on terrain spawns the crater instead of
                    // doing whatever the current mode does.
                    if (pendingCrater_.active) {
                        auto ext = window_->getVkContext()->getSwapchainExtent();
                        rendering::Ray ray = camera_.getCamera().screenToWorldRay(
                            static_cast<float>(event.button.x),
                            static_cast<float>(event.button.y),
                            static_cast<float>(ext.width),
                            static_cast<float>(ext.height));
                        glm::vec3 hitPos;
                        if (terrainEditor_.raycastTerrain(ray, hitPos)) {
                            terrainEditor_.createCrater(hitPos,
                                                          pendingCrater_.radius,
                                                          pendingCrater_.depth,
                                                          pendingCrater_.rim);
                            showToast("Crater placed");
                            pendingCrater_.active = false;
                        }
                        // If the click missed terrain, leave the mode armed
                        // so the user can try again without re-pressing the
                        // button.
                        continue;
                    }
                    // Path point capture (river/road tool)
                    // Alt+click eyedropper in paint mode
                    if (mode_ == EditorMode::Paint && (SDL_GetModState() & SDL_KMOD_ALT)) {
                        if (terrainEditor_.brush().isActive()) {
                            std::string picked = texturePainter_.pickTextureAt(
                                terrainEditor_.brush().getPosition());
                            if (!picked.empty()) {
                                texturePainter_.setActiveTexture(picked);
                                showToast("Picked: " + picked.substr(picked.rfind('\\') + 1));
                            }
                        }
                    }
                    else if (ui_.getPathCapture() != EditorUI::PathCapture::None) {
                        auto ext = window_->getVkContext()->getSwapchainExtent();
                        rendering::Ray ray = camera_.getCamera().screenToWorldRay(
                            static_cast<float>(event.button.x),
                            static_cast<float>(event.button.y),
                            static_cast<float>(ext.width),
                            static_cast<float>(ext.height));
                        glm::vec3 hitPos;
                        if (terrainEditor_.raycastTerrain(ray, hitPos)) {
                            ui_.setPathPoint(hitPos);
                            if (ui_.getPathCapture() == EditorUI::PathCapture::None && ui_.isPathReady())
                                showToast("Both points set - click Apply Path");
                            else if (ui_.getPathCapture() == EditorUI::PathCapture::WaitingEnd)
                                showToast("Start point set - click terrain for end");
                        }
                    }
                    // Ctrl+click = select (Ctrl+Shift+click = add to selection)
                    else if ((event.key.mod & SDL_KMOD_CTRL) || (SDL_GetModState() & SDL_KMOD_CTRL)) {
                        bool additive = (SDL_GetModState() & SDL_KMOD_SHIFT) != 0;
                        auto ext = window_->getVkContext()->getSwapchainExtent();
                        rendering::Ray ray = camera_.getCamera().screenToWorldRay(
                            static_cast<float>(event.button.x),
                            static_cast<float>(event.button.y),
                            static_cast<float>(ext.width),
                            static_cast<float>(ext.height));
                        if (additive) {
                            int prevSel = objectPlacer_.getSelectedIndex();
                            int hit = objectPlacer_.selectAt(ray, 200.0f);
                            if (hit >= 0) {
                                if (prevSel >= 0) objectPlacer_.addToSelection(prevSel);
                                objectPlacer_.addToSelection(hit);
                            }
                        } else {
                            objectPlacer_.selectAt(ray, 200.0f);
                        }
                    } else if (mode_ == EditorMode::NPC) {
                        auto ext = window_->getVkContext()->getSwapchainExtent();
                        rendering::Ray ray = camera_.getCamera().screenToWorldRay(
                            static_cast<float>(event.button.x),
                            static_cast<float>(event.button.y),
                            static_cast<float>(ext.width),
                            static_cast<float>(ext.height));
                        glm::vec3 hitPos;
                        if (terrainEditor_.raycastTerrain(ray, hitPos)) {
                            // Plain left-click near an existing NPC selects it instead of
                            // placing a duplicate. Shift+click forces placement.
                            bool forcePlace = (event.key.mod & SDL_KMOD_SHIFT) != 0;
                            int hit = forcePlace ? -1 : npcSpawner_.selectAt(hitPos, 4.0f);
                            if (hit < 0) {
                                auto& tmpl = npcSpawner_.getTemplate();
                                tmpl.position = hitPos;
                                npcSpawner_.placeCreature(tmpl);
                                objectsDirty_ = true; autoSavePendingChanges_ = true;
                            }
                        }
                    } else if (mode_ == EditorMode::Water) {
                        painting_ = true;
                    } else if (mode_ == EditorMode::PlaceObject) {
                        // Raycast now at click time for accurate placement
                        auto ext = window_->getVkContext()->getSwapchainExtent();
                        rendering::Ray ray = camera_.getCamera().screenToWorldRay(
                            static_cast<float>(event.button.x),
                            static_cast<float>(event.button.y),
                            static_cast<float>(ext.width),
                            static_cast<float>(ext.height));
                        glm::vec3 hitPos;
                        if (terrainEditor_.raycastTerrain(ray, hitPos)) {
                            objectPlacer_.placeObject(hitPos);
                            objectsDirty_ = true; autoSavePendingChanges_ = true;
                        }
                    } else {
                        painting_ = true;
                        if (mode_ == EditorMode::Sculpt || mode_ == EditorMode::Paint)
                            terrainEditor_.beginStroke();
                    }
                } else if (event.type == SDL_EVENT_MOUSE_BUTTON_UP) {
                    painting_ = false;
                    if (mode_ == EditorMode::Sculpt || mode_ == EditorMode::Paint)
                        terrainEditor_.endStroke();
                }
            }

            // Middle click = select object
            if (event.button.button == SDL_BUTTON_MIDDLE && event.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
                if (mode_ == EditorMode::PlaceObject && terrain_.isLoaded()) {
                    auto ext = window_->getVkContext()->getSwapchainExtent();
                    auto& io2 = ImGui::GetIO();
                    rendering::Ray ray = camera_.getCamera().screenToWorldRay(
                        io2.MousePos.x, io2.MousePos.y,
                        static_cast<float>(ext.width), static_cast<float>(ext.height));
                    objectPlacer_.selectAt(ray);
                }
            }
        }

        if (event.type == SDL_EVENT_MOUSE_WHEEL && !io.WantCaptureMouse) {
            // Ctrl+wheel rotates the placement preview instead of zooming the camera.
            // Step 15 deg, Shift makes it 5 deg for finer control.
            bool ctrl = (SDL_GetModState() & SDL_KMOD_CTRL) != 0;
            bool shift = (SDL_GetModState() & SDL_KMOD_SHIFT) != 0;
            if (ctrl && (mode_ == EditorMode::PlaceObject || mode_ == EditorMode::NPC)) {
                float step = shift ? 5.0f : 15.0f;
                if (mode_ == EditorMode::PlaceObject) {
                    float r = objectPlacer_.getPlacementRotationY() + step * event.wheel.y;
                    while (r >= 360.0f) r -= 360.0f;
                    while (r < 0.0f) r += 360.0f;
                    objectPlacer_.setPlacementRotationY(r);
                } else {
                    float r = npcSpawner_.getTemplate().orientation + step * event.wheel.y;
                    while (r >= 360.0f) r -= 360.0f;
                    while (r < 0.0f) r += 360.0f;
                    npcSpawner_.getTemplate().orientation = r;
                }
            } else {
                camera_.processMouseWheel(event.wheel.y, shift);
            }
        }
    }
}

void EditorApp::updateTerrainEditing(float dt) {
    if (!terrain_.isLoaded()) return;

    // Update brush position from mouse cursor
    auto& io = ImGui::GetIO();
    if (!io.WantCaptureMouse) {
        float mx = io.MousePos.x;
        float my = io.MousePos.y;
        auto ext = window_->getVkContext()->getSwapchainExtent();

        rendering::Ray ray = camera_.getCamera().screenToWorldRay(
            mx, my, static_cast<float>(ext.width), static_cast<float>(ext.height));

        glm::vec3 hitPos;
        if (terrainEditor_.raycastTerrain(ray, hitPos)) {
            terrainEditor_.brush().setPosition(hitPos);
            terrainEditor_.brush().setActive(true);

            // Ghost preview for object/NPC placement
            if (mode_ == EditorMode::PlaceObject && !objectPlacer_.getActivePath().empty()) {
                viewport_.setGhostPreview(
                    objectPlacer_.getActivePath(), hitPos,
                    glm::vec3(0, objectPlacer_.getPlacementRotationY(), 0),
                    objectPlacer_.getPlacementScale());
            } else if (mode_ == EditorMode::NPC && !npcSpawner_.getTemplate().modelPath.empty()) {
                viewport_.setGhostPreview(
                    npcSpawner_.getTemplate().modelPath, hitPos,
                    glm::vec3(0, 0, npcSpawner_.getTemplate().orientation),
                    npcSpawner_.getTemplate().scale);
            } else if (mode_ != EditorMode::PlaceObject && mode_ != EditorMode::NPC) {
                viewport_.clearGhostPreview();
            }

            // Brush circle indicator for sculpt/paint/water modes
            if (mode_ == EditorMode::Sculpt || mode_ == EditorMode::Paint || mode_ == EditorMode::Water) {
                viewport_.setBrushIndicator(hitPos, terrainEditor_.brush().settings().radius, true);
            } else {
                viewport_.setBrushIndicator(hitPos, 0, false);
            }

            if (painting_ && terrainEditor_.brush().settings().mode == BrushMode::Flatten) {
                static bool flattenSet = false;
                if (!flattenSet) {
                    terrainEditor_.brush().settings().flattenHeight = hitPos.z;
                    flattenSet = true;
                }
                if (!io.MouseDown[0]) flattenSet = false;
            }
        } else {
            terrainEditor_.brush().setActive(false);
            viewport_.setBrushIndicator({}, 0, false);
            viewport_.clearGhostPreview();
        }

        // Path preview for river/road tool
        if (ui_.getPathCapture() == EditorUI::PathCapture::WaitingEnd ||
            ui_.isPathReady()) {
            glm::vec3 endPt = ui_.isPathReady() ? ui_.getPathEnd()
                                                : terrainEditor_.brush().getPosition();
            viewport_.setPathPreview(ui_.getPathStart(), endPt,
                ui_.getPathWidth(), true);
        } else {
            viewport_.setPathPreview({}, {}, 0, false);
        }
    }

    if (painting_ && terrainEditor_.brush().isActive()) {
        if (mode_ == EditorMode::Sculpt) {
            terrainEditor_.applyBrush(dt);
        } else if (mode_ == EditorMode::Paint) {
            auto& brush = terrainEditor_.brush();
            auto paintMode = ui_.getPaintMode();
            std::vector<int> modified;

            if (paintMode == PaintMode::Erase) {
                modified = texturePainter_.erase(
                    brush.getPosition(), brush.settings().radius,
                    brush.settings().strength * dt * 0.5f, brush.settings().falloff);
            } else if (paintMode == PaintMode::ReplaceBase) {
                // Replace base texture of chunks under brush
                auto& texPath = texturePainter_.getActiveTexture();
                if (!texPath.empty()) {
                    // Ensure texture is in list
                    uint32_t texId = 0;
                    for (uint32_t i = 0; i < terrain_.textures.size(); i++) {
                        if (terrain_.textures[i] == texPath) { texId = i; goto found; }
                    }
                    terrain_.textures.push_back(texPath);
                    texId = static_cast<uint32_t>(terrain_.textures.size() - 1);
                    found:
                    for (int ci = 0; ci < 256; ci++) {
                        auto& chunk = terrain_.chunks[ci];
                        if (!chunk.hasHeightMap() || chunk.layers.empty()) continue;
                        glm::vec3 cpos = terrainEditor_.brush().getPosition();
                        // Rough distance check
                        auto vpos = glm::vec3(chunk.position[1], chunk.position[0], chunk.position[2]);
                        if (glm::length(glm::vec2(vpos.x - cpos.x, vpos.y - cpos.y)) < brush.settings().radius + 40.0f) {
                            chunk.layers[0].textureId = texId;
                            modified.push_back(ci);
                        }
                    }
                }
            } else {
                modified = texturePainter_.paint(
                    brush.getPosition(), brush.settings().radius,
                    brush.settings().strength * dt * 0.5f, brush.settings().falloff);
            }

            if (!modified.empty()) {
                auto mesh = terrainEditor_.regenerateMesh();
                viewport_.clearTerrain();
                viewport_.loadTerrain(mesh, terrain_.textures, loadedTileX_, loadedTileY_);
            }
        } else if (mode_ == EditorMode::Water) {
            auto& brush = terrainEditor_.brush();
            terrainEditor_.setWaterLevel(brush.getPosition(), brush.settings().radius,
                                         waterHeight_, waterType_);
            viewport_.updateWater(terrain_, loadedTileX_, loadedTileY_);
        }
    }
}

void EditorApp::refreshDirtyChunks() {
    auto dirty = terrainEditor_.consumeDirtyChunks();
    if (dirty.empty()) return;

    // Recalculate normals for modified chunks (better lighting)
    terrainEditor_.recalcNormals(dirty);

    // Regenerate full mesh and reload terrain
    auto mesh = terrainEditor_.regenerateMesh();
    viewport_.clearTerrain();
    viewport_.loadTerrain(mesh, terrain_.textures, loadedTileX_, loadedTileY_);
}

bool EditorApp::loadWMOInstance(const std::string& mapName) {
    std::string mapLower = mapName;
    std::transform(mapLower.begin(), mapLower.end(), mapLower.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    std::string wdtPath = "world\\maps\\" + mapLower + "\\" + mapLower + ".wdt";
    auto wdtData = assetManager_->readFile(wdtPath);
    if (wdtData.empty()) return false;

    auto wdtInfo = pipeline::parseWDT(wdtData);
    if (!wdtInfo.isWMOOnly() || wdtInfo.rootWMOPath.empty()) return false;

    LOG_INFO("WMO-only instance: ", mapName, " root=", wdtInfo.rootWMOPath);

    clearAllObjects();
    questEditor_.clear();
    ui_.clearPath();
    viewport_.clearTerrain();

    // Create blank terrain as a floor reference
    terrain_ = TerrainEditor::createBlankTerrain(32, 32, 0.0f, Biome::Rocky);
    terrain_.coord = {32, 32};
    terrainEditor_.setTerrain(&terrain_);
    texturePainter_.setTerrain(&terrain_);
    objectPlacer_.setTerrain(&terrain_);

    auto mesh = pipeline::TerrainMeshGenerator::generate(terrain_);
    viewport_.loadTerrain(mesh, terrain_.textures, 32, 32);

    // Place the root WMO as an object
    glm::vec3 wmoPos = core::coords::adtToWorld(
        wdtInfo.position[0], wdtInfo.position[1], wdtInfo.position[2]);
    glm::vec3 wmoRot(-wdtInfo.rotation[2], -wdtInfo.rotation[0],
                      wdtInfo.rotation[1] + 180.0f);

    PlacedObject wmo;
    wmo.type = PlaceableType::WMO;
    wmo.path = wdtInfo.rootWMOPath;
    wmo.position = wmoPos;
    wmo.rotation = wmoRot;
    wmo.scale = 1.0f;
    wmo.uniqueId = 1;
    objectPlacer_.getObjects().push_back(wmo);
    objectsDirty_ = true; autoSavePendingChanges_ = true;

    loadedMap_ = mapName;
    loadedTileX_ = 32;
    loadedTileY_ = 32;
    viewport_.setActiveMapName(mapName);

    // Position camera near the WMO
    camera_.setPosition(wmoPos + glm::vec3(0, 0, 50));
    camera_.setYawPitch(0.0f, -30.0f);

    showToast("WMO instance loaded: " + mapName);
    LOG_INFO("WMO instance loaded: ", mapName, " at (",
             wmoPos.x, ",", wmoPos.y, ",", wmoPos.z, ")");
    return true;
}

void EditorApp::loadADT(const std::string& mapName, int tileX, int tileY) {
    // WoW tile grid is 64x64 - out-of-range coords would compute paths
    // like "World\Maps\Foo\Foo_-1_-1.adt" that the asset manager refuses,
    // and would also poison the manifest.tiles entries on save.
    if (tileX < 0 || tileX > 63 || tileY < 0 || tileY > 63) {
        LOG_ERROR("loadADT rejected: tile (", tileX, ",", tileY,
                  ") out of valid 0..63 range");
        return;
    }
    // Clear previous state before loading new tile
    clearAllObjects();
    questEditor_.clear();
    ui_.clearPath();
    viewport_.clearTerrain();
    // Reset the zone manifest so previous zone's mapId/displayName/tiles
    // don't bleed into the new export. zone.json on disk (if any) will be
    // re-loaded later in this function.
    zoneManifest_ = {};

    // Prefer open format (WOT/WHM) if available
    for (const char* dir : {"custom_zones", "output"}) {
        std::string wotBase = std::string(dir) + "/" + mapName + "/" + mapName + "_" +
                              std::to_string(tileX) + "_" + std::to_string(tileY);
        if (WoweeTerrain::importOpen(wotBase, terrain_) && terrain_.isLoaded()) {
            LOG_INFO("Loaded open format terrain: ", wotBase);
            showToast("Loaded WOT/WHM: " + mapName);
            goto terrainReady;
        }
    }

    {
        std::ostringstream path;
        path << "World\\Maps\\" << mapName << "\\" << mapName
             << "_" << tileX << "_" << tileY << ".adt";

        LOG_INFO("Loading ADT: ", path.str());

        auto adtData = assetManager_->readFile(path.str());
        if (adtData.empty()) {
            // Try WMO-only instance (dungeons like Dire Maul have no ADT tiles)
            if (loadWMOInstance(mapName)) return;
            LOG_ERROR("ADT file not found: ", path.str());
            showToast("Zone not found: " + mapName + " [" + std::to_string(tileX) + "," + std::to_string(tileY) + "]");
            return;
        }

        terrain_ = pipeline::ADTLoader::load(adtData);
        if (!terrain_.isLoaded()) {
            LOG_ERROR("Failed to parse ADT: ", path.str());
            showToast("Failed to load zone (corrupt or unsupported format)");
            return;
        }
    }
    terrainReady:

    // Override internal coords with what we know from the filename
    // (instanced maps have arbitrary internal coord values)
    terrain_.coord = {tileX, tileY};

    // Recompute chunk world positions from tile coordinates
    // This fixes instanced maps where internal MCNK positions are wrong
    float tileSize = 533.33333f;
    float chunkSize = tileSize / 16.0f;
    for (int cy = 0; cy < 16; cy++) {
        for (int cx = 0; cx < 16; cx++) {
            auto& chunk = terrain_.chunks[cy * 16 + cx];
            if (!chunk.hasHeightMap()) continue;
            chunk.position[0] = (32.0f - static_cast<float>(tileX)) * tileSize - cx * chunkSize;
            chunk.position[1] = (32.0f - static_cast<float>(tileY)) * tileSize - cy * chunkSize;
        }
    }

    terrainEditor_.setTerrain(&terrain_);
    terrainEditor_.history().clear();
    texturePainter_.setTerrain(&terrain_);
    objectPlacer_.setTerrain(&terrain_);

    auto mesh = pipeline::TerrainMeshGenerator::generate(terrain_);
    if (mesh.validChunkCount == 0) {
        LOG_ERROR("ADT has no valid terrain chunks");
        showToast("Error: no valid terrain data in this tile");
        return;
    }
    viewport_.clearTerrain();
    if (!viewport_.loadTerrain(mesh, terrain_.textures, tileX, tileY)) {
        LOG_ERROR("Failed to upload terrain to GPU (", mesh.validChunkCount, " chunks)");
        showToast("Error: terrain upload failed");
        return;
    }

    loadedMap_ = mapName;
    loadedTileX_ = tileX;
    loadedTileY_ = tileY;
    viewport_.setActiveMapName(mapName);

    // Track recent zones (deduplicate, max 8)
    recentZones_.erase(std::remove_if(recentZones_.begin(), recentZones_.end(),
        [&](const RecentZone& rz) { return rz.mapName == mapName && rz.tileX == tileX && rz.tileY == tileY; }),
        recentZones_.end());
    recentZones_.insert(recentZones_.begin(), {mapName, tileX, tileY});
    if (recentZones_.size() > 8) recentZones_.resize(8);

    // Position camera at terrain center using actual chunk positions
    if (mesh.validChunkCount > 0) {
        auto& firstChunk = mesh.chunks[0];
        auto& lastChunk = mesh.chunks[255];
        float cx = (firstChunk.worldX + lastChunk.worldX) * 0.5f;
        float cy = (firstChunk.worldY + lastChunk.worldY) * 0.5f;
        float cz = firstChunk.worldZ + 300.0f;
        camera_.setPosition(glm::vec3(cx, cy, cz));
    } else {
        float centerX = (32.0f - tileY) * 533.33333f - 8.0f * 533.33333f / 16.0f;
        float centerY = (32.0f - tileX) * 533.33333f - 8.0f * 533.33333f / 16.0f;
        camera_.setPosition(glm::vec3(centerX, centerY, 400.0f));
    }
    camera_.setYawPitch(0.0f, -45.0f);

    // Import doodad/WMO placements from the ADT itself
    // ADT positions are in ADT coordinate space - convert to render coords
    // Import doodad placements - convert ADT rotation to render rotation
    // ADT stores rotation as degrees [rotX, rotY, rotZ] in WoW space
    // Render space: rX = -adtRotZ, rY = -adtRotX, rZ = adtRotY + 180
    for (const auto& dp : terrain_.doodadPlacements) {
        if (dp.nameId < terrain_.doodadNames.size()) {
            PlacedObject obj;
            obj.type = PlaceableType::M2;
            obj.path = terrain_.doodadNames[dp.nameId];
            obj.position = core::coords::adtToWorld(dp.position[0], dp.position[1], dp.position[2]);
            obj.rotation = glm::vec3(-dp.rotation[2], -dp.rotation[0], dp.rotation[1] + 180.0f);
            obj.scale = static_cast<float>(dp.scale) / 1024.0f;
            obj.uniqueId = dp.uniqueId;
            objectPlacer_.getObjects().push_back(obj);
        }
    }
    for (const auto& wp : terrain_.wmoPlacements) {
        if (wp.nameId < terrain_.wmoNames.size()) {
            PlacedObject obj;
            obj.type = PlaceableType::WMO;
            obj.path = terrain_.wmoNames[wp.nameId];
            obj.position = core::coords::adtToWorld(wp.position[0], wp.position[1], wp.position[2]);
            obj.rotation = glm::vec3(-wp.rotation[2], -wp.rotation[0], wp.rotation[1] + 180.0f);
            // MODF scale is fixed-point u16 (1024 = 1.0); fall back to 1.0
            // for older expansions where the scale slot was always 0.
            obj.scale = wp.scale > 0 ? static_cast<float>(wp.scale) / 1024.0f : 1.0f;
            obj.uniqueId = wp.uniqueId;
            objectPlacer_.getObjects().push_back(obj);
        }
    }
    if (!terrain_.doodadPlacements.empty() || !terrain_.wmoPlacements.empty()) {
        objectsDirty_ = true; autoSavePendingChanges_ = true;
        showToast("Imported " + std::to_string(terrain_.doodadPlacements.size()) +
                  " doodads + " + std::to_string(terrain_.wmoPlacements.size()) + " WMOs");
        LOG_INFO("Imported ", terrain_.doodadPlacements.size(), " doodads + ",
                 terrain_.wmoPlacements.size(), " WMOs from ADT");
    }

    LOG_INFO("ADT loaded: ", mapName, " [", tileX, ",", tileY, "]");

    // Try loading objects/NPCs/quests/manifest from zone directories
    for (const char* dir : {"output", "custom_zones"}) {
        std::string zoneBase = std::string(dir) + "/" + mapName;
        if (objectPlacer_.objectCount() == 0)
            if (objectPlacer_.loadFromFile(zoneBase + "/objects.json"))
                showToast("Loaded " + std::to_string(objectPlacer_.objectCount()) + " objects");
        if (npcSpawner_.spawnCount() == 0)
            if (npcSpawner_.loadFromFile(zoneBase + "/creatures.json"))
                showToast("Loaded " + std::to_string(npcSpawner_.spawnCount()) + " NPCs");
        if (questEditor_.questCount() == 0)
            if (questEditor_.loadFromFile(zoneBase + "/quests.json"))
                showToast("Loaded " + std::to_string(questEditor_.questCount()) + " quests");
        // Restore the previously-saved zone manifest (mapId, displayName,
        // flags, audio, etc.) so user-customized metadata persists across
        // editor sessions.
        if (zoneManifest_.mapName.empty())
            zoneManifest_.load(zoneBase + "/zone.json");
    }
    // Always set mapName from the loaded ADT in case zone.json was absent
    // or stale.
    if (zoneManifest_.mapName.empty()) zoneManifest_.mapName = mapName;
    if (objectPlacer_.objectCount() > 0 || npcSpawner_.spawnCount() > 0) {
        objectsDirty_ = true;
        autoSavePendingChanges_ = true;
    }
}

void EditorApp::createNewTerrain(const std::string& mapName, int tileX, int tileY, float baseHeight, Biome biome) {
    if (tileX < 0 || tileX > 63 || tileY < 0 || tileY > 63) {
        LOG_ERROR("createNewTerrain rejected: tile (", tileX, ",", tileY,
                  ") out of valid 0..63 range");
        return;
    }
    if (!std::isfinite(baseHeight)) baseHeight = 0.0f;
    activeBiome_ = biome;
    terrain_ = TerrainEditor::createBlankTerrain(tileX, tileY, baseHeight, biome);
    // Clear all previous state
    clearAllObjects();
    questEditor_.clear();
    ui_.clearPath();

    terrainEditor_.setTerrain(&terrain_);
    terrainEditor_.history().clear();
    texturePainter_.setTerrain(&terrain_);
    objectPlacer_.setTerrain(&terrain_);

    auto mesh = pipeline::TerrainMeshGenerator::generate(terrain_);
    viewport_.clearTerrain();
    viewport_.loadTerrain(mesh, terrain_.textures, tileX, tileY);

    loadedMap_ = mapName;
    loadedTileX_ = tileX;
    loadedTileY_ = tileY;
    viewport_.setActiveMapName(mapName);
    lastObjCount_ = 0;
    lastNpcCount_ = 0;
    objectsDirty_ = false;

    float centerX = (32.0f - tileY) * 533.33333f - 8.0f * 533.33333f / 16.0f;
    float centerY = (32.0f - tileX) * 533.33333f - 8.0f * 533.33333f / 16.0f;
    camera_.setPosition(glm::vec3(centerX, centerY, baseHeight + 300.0f));
    camera_.setYawPitch(0.0f, -45.0f);

    LOG_INFO("New terrain created: ", mapName, " [", tileX, ",", tileY, "] base=", baseHeight);
}

void EditorApp::saveADT(const std::string& path) {
    if (!terrain_.isLoaded()) {
        LOG_ERROR("No terrain to save");
        return;
    }
    objectPlacer_.syncToTerrain();
    ADTWriter::write(terrain_, path);
    terrainEditor_.markSaved();
}

void EditorApp::saveWDT(const std::string& path) {
    if (loadedMap_.empty()) return;
    ADTWriter::writeWDT(loadedMap_, loadedTileX_, loadedTileY_, path);
}

void EditorApp::exportZone(const std::string& outputDir) {
    if (!terrain_.isLoaded() || loadedMap_.empty()) return;

    std::string base = outputDir + "/" + loadedMap_;

    // Save ADT
    std::string adtPath = base + "/" + loadedMap_ + "_" +
                          std::to_string(loadedTileX_) + "_" +
                          std::to_string(loadedTileY_) + ".adt";
    saveADT(adtPath);

    // Save WDT
    std::string wdtPath = base + "/" + loadedMap_ + ".wdt";
    saveWDT(wdtPath);

    // Save creature spawns
    if (npcSpawner_.spawnCount() > 0) {
        std::string npcPath = base + "/creatures.json";
        npcSpawner_.saveToFile(npcPath);
    }

    // Save quests
    if (questEditor_.questCount() > 0) {
        std::string questPath = base + "/quests.json";
        questEditor_.saveToFile(questPath);
        std::vector<std::string> chainErrors;
        if (!questEditor_.validateChains(chainErrors)) {
            for (const auto& err : chainErrors)
                LOG_WARNING("Quest chain issue: ", err);
            // Surface chain issues to the user - silently logging means most
            // users won't notice a broken chain until they test in-game.
            showToast("Quest chains have " + std::to_string(chainErrors.size()) +
                      " issue(s) - see log", 5.0f);
        }
    }

    // Export SQL for private server integration (AzerothCore/TrinityCore)
    if (npcSpawner_.spawnCount() > 0 || questEditor_.questCount() > 0) {
        std::string sqlPath = base + "/spawns.sql";
        SQLExporter::exportAll(npcSpawner_.getSpawns(),
                               questEditor_.getQuests(),
                               sqlPath, zoneManifest_.mapId);
    }

    // Save placed objects
    if (objectPlacer_.objectCount() > 0) {
        std::string objPath = base + "/objects.json";
        objectPlacer_.saveToFile(objPath);
    }

    // Convert all referenced M2 models (placed objects + NPCs) to WOM open format.
    // This makes the exported zone self-contained and free of proprietary M2/skin files.
    {
        std::unordered_set<std::string> convertedModels;
        auto convertOne = [&](const std::string& m2Path) {
            if (m2Path.empty() || convertedModels.count(m2Path)) return;
            auto wom = pipeline::WoweeModelLoader::fromM2(m2Path, assetManager_.get());
            if (!wom.isValid()) return;
            std::string womPath = m2Path;
            std::replace(womPath.begin(), womPath.end(), '\\', '/');
            auto dot = womPath.rfind('.');
            if (dot != std::string::npos) womPath = womPath.substr(0, dot);
            pipeline::WoweeModelLoader::save(wom, base + "/models/" + womPath);
            convertedModels.insert(m2Path);
        };
        for (const auto& obj : objectPlacer_.getObjects()) {
            if (obj.type == PlaceableType::M2) convertOne(obj.path);
        }
        for (const auto& npc : npcSpawner_.getSpawns()) {
            convertOne(npc.modelPath);
        }
        if (!convertedModels.empty())
            LOG_INFO("Converted ", convertedModels.size(), " M2 models to WOM (objects + NPCs)");
    }

    // Convert placed WMO buildings to WOB open format
    if (objectPlacer_.objectCount() > 0) {
        std::unordered_set<std::string> convertedWMOs;
        for (const auto& obj : objectPlacer_.getObjects()) {
            if (obj.type == PlaceableType::WMO && !convertedWMOs.count(obj.path)) {
                std::string wobPath = obj.path;
                std::replace(wobPath.begin(), wobPath.end(), '\\', '/');
                auto dot = wobPath.rfind('.');
                if (dot != std::string::npos) wobPath = wobPath.substr(0, dot);

                auto wmoData = assetManager_->readFile(obj.path);
                if (!wmoData.empty()) {
                    auto wmoModel = pipeline::WMOLoader::load(wmoData);
                    if (wmoModel.nGroups > 0) {
                        std::string wmoBase = obj.path;
                        if (wmoBase.size() > 4) wmoBase = wmoBase.substr(0, wmoBase.size() - 4);
                        for (uint32_t gi = 0; gi < wmoModel.nGroups; gi++) {
                            char suffix[16];
                            snprintf(suffix, sizeof(suffix), "_%03u.wmo", gi);
                            auto gd = assetManager_->readFile(wmoBase + suffix);
                            if (!gd.empty()) pipeline::WMOLoader::loadGroup(gd, wmoModel, gi);
                        }
                    }
                    auto bld = pipeline::WoweeBuildingLoader::fromWMO(wmoModel, obj.path);
                    pipeline::WoweeBuildingLoader::save(bld, base + "/buildings/" + wobPath);
                } else {
                    pipeline::WoweeBuilding bld;
                    bld.name = obj.path;
                    pipeline::WoweeBuildingLoader::save(bld, base + "/buildings/" + wobPath);
                }
                convertedWMOs.insert(obj.path);
            }
        }
        if (!convertedWMOs.empty())
            LOG_INFO("Converted ", convertedWMOs.size(), " WMO buildings to WOB");
    }

    // Export used textures as PNG (open format replacement for BLP).
    // Includes terrain textures plus textures referenced by every placed M2/NPC model
    // so the exported zone has every texture it needs to render without game data.
    std::vector<std::string> usedTextures;
    {
        std::unordered_set<std::string> uniq;
        for (auto& t : TextureExporter::collectUsedTextures(terrain_)) uniq.insert(std::move(t));
        std::unordered_set<std::string> visitedM2;
        std::unordered_set<std::string> visitedWMO;
        auto addM2Tex = [&](const std::string& m2Path) {
            if (m2Path.empty() || !visitedM2.insert(m2Path).second) return;
            for (auto& t : TextureExporter::collectM2Textures(assetManager_.get(), m2Path))
                uniq.insert(std::move(t));
        };
        auto addWMOTex = [&](const std::string& wmoPath) {
            if (wmoPath.empty() || !visitedWMO.insert(wmoPath).second) return;
            for (auto& t : TextureExporter::collectWMOTextures(assetManager_.get(), wmoPath))
                uniq.insert(std::move(t));
        };
        for (const auto& obj : objectPlacer_.getObjects()) {
            if (obj.type == PlaceableType::M2) addM2Tex(obj.path);
            else if (obj.type == PlaceableType::WMO) addWMOTex(obj.path);
        }
        for (const auto& npc : npcSpawner_.getSpawns()) addM2Tex(npc.modelPath);

        usedTextures.assign(uniq.begin(), uniq.end());
        std::sort(usedTextures.begin(), usedTextures.end());
        if (!usedTextures.empty()) {
            int exported = TextureExporter::exportTexturesAsPng(
                assetManager_.get(), usedTextures, base + "/textures");
            LOG_INFO("Exported ", exported, " textures as PNG (terrain + ",
                     visitedM2.size(), " M2s + ", visitedWMO.size(), " WMOs)");
        }
    }

    // Export zone-relevant DBCs as JSON (open format replacement for DBC)
    DBCExporter::exportZoneDBCs(assetManager_.get(), base + "/data");

    // Export open terrain format alongside ADT
    std::string openBase = base + "/" + loadedMap_ + "_" +
                           std::to_string(loadedTileX_) + "_" + std::to_string(loadedTileY_);
    WoweeTerrain::exportOpen(terrain_, openBase, loadedTileX_, loadedTileY_);
    WoweeTerrain::exportNormalMap(terrain_, openBase + "_normals.png");

    // Export collision mesh (.woc) - terrain plus placed WMO/M2 meshes so that
    // movement queries on the exported zone respect buildings and large props.
    auto collision = pipeline::WoweeCollisionBuilder::fromTerrain(terrain_);
    {
        std::unordered_set<std::string> visitedWMOcol;
        std::unordered_set<std::string> visitedM2col;
        for (const auto& obj : objectPlacer_.getObjects()) {
            if (obj.type == PlaceableType::WMO && visitedWMOcol.insert(obj.path).second) {
                auto data = assetManager_->readFile(obj.path);
                if (data.empty()) continue;
                auto wmo = pipeline::WMOLoader::load(data);
                std::string wmoBase = obj.path;
                if (wmoBase.size() > 4) wmoBase = wmoBase.substr(0, wmoBase.size() - 4);
                for (uint32_t gi = 0; gi < wmo.nGroups; gi++) {
                    char suffix[16];
                    std::snprintf(suffix, sizeof(suffix), "_%03u.wmo", gi);
                    auto gd = assetManager_->readFile(wmoBase + suffix);
                    if (!gd.empty()) pipeline::WMOLoader::loadGroup(gd, wmo, gi);
                }
                glm::mat4 t = glm::translate(glm::mat4(1.0f), obj.position);
                glm::vec3 r = glm::radians(obj.rotation);
                t = glm::rotate(t, r.x, glm::vec3(1, 0, 0));
                t = glm::rotate(t, r.y, glm::vec3(0, 1, 0));
                t = glm::rotate(t, r.z, glm::vec3(0, 0, 1));
                t = glm::scale(t, glm::vec3(obj.scale));
                for (const auto& g : wmo.groups) {
                    std::vector<glm::vec3> verts;
                    verts.reserve(g.vertices.size());
                    for (const auto& v : g.vertices) verts.push_back(v.position);
                    std::vector<uint32_t> idx;
                    idx.reserve(g.indices.size());
                    for (uint16_t i : g.indices) idx.push_back(i);
                    uint8_t flags = (g.flags & 0x08) ? 0 : 0x08; // indoor flag
                    pipeline::WoweeCollisionBuilder::addMesh(collision, verts, idx, t, flags);
                }
            }
        }
    }
    if (collision.isValid())
        pipeline::WoweeCollisionBuilder::save(collision, openBase + ".woc");
    WoweeTerrain::exportAlphaMaps(terrain_, base + "/alphamaps");
    WoweeTerrain::exportWaterMask(terrain_, openBase + "_watermask.png");
    WoweeTerrain::exportHoleMask(terrain_, openBase + "_holemask.png");
    WoweeTerrain::exportHeightmapPreview(terrain_, openBase + "_heightmap.png");
    // Also save heightmap as zone thumbnail for content pack browsing
    WoweeTerrain::exportHeightmapPreview(terrain_, base + "/thumbnail.png");
    WoweeTerrain::exportZoneMap(terrain_, base + "/zone_map.png", 512);

    // Write zone info README
    {
        std::ofstream readme(base + "/README.txt");
        if (readme) {
            readme << "Zone: " << loadedMap_ << "\n";
            readme << "Tile: [" << loadedTileX_ << ", " << loadedTileY_ << "]\n";
            readme << "Objects: " << objectPlacer_.objectCount() << "\n";
            readme << "NPCs: " << npcSpawner_.spawnCount() << "\n";
            readme << "Quests: " << questEditor_.questCount() << "\n";
            readme << "Created with Wowee World Editor v1.0.0\n\n";
            readme << "\nOpen Formats (no Blizzard IP):\n";
            readme << "  .wot/.whm  - Wowee Open Terrain (heightmap + metadata)\n";
            readme << "  .wom       - Wowee Open Model (static 3D models)\n";
            readme << "  .wob       - Wowee Open Building (multi-group buildings)\n";
            readme << "  .png       - Standard textures (converted from BLP)\n";
            readme << "  .json      - Data tables, quests, creatures, objects\n";
            readme << "  .wcp       - Wowee Content Pack (distribution archive)\n\n";
            readme << "Files:\n";
            readme << "  zone.json       - Zone manifest (for client)\n";
            readme << "  " << loadedMap_ << ".wdt   - Map definition\n";
            readme << "  " << loadedMap_ << "_" << loadedTileX_ << "_" << loadedTileY_ << ".adt - Terrain tile\n";
            if (objectPlacer_.objectCount() > 0) readme << "  objects.json    - Placed M2/WMO objects\n";
            if (npcSpawner_.spawnCount() > 0) readme << "  creatures.json  - NPC/monster spawns\n";
        }
    }

    // Write zone manifest (for client loading)
    // Scan output directory for all exported tiles (includes adjacent tiles)
    ZoneManifest& manifest = zoneManifest_;
    manifest.mapName = loadedMap_;
    // Preserve user-set displayName from the Zone Metadata panel; only fill in
    // the default when the user hasn't customized it.
    if (manifest.displayName.empty()) manifest.displayName = loadedMap_;
    manifest.tiles.clear();
    manifest.tiles.push_back({loadedTileX_, loadedTileY_});
    namespace fs = std::filesystem;
    if (fs::exists(base)) {
        for (auto& entry : fs::directory_iterator(base)) {
            if (entry.path().extension() != ".adt") continue;
            std::string stem = entry.path().stem().string();
            auto lastU = stem.rfind('_');
            auto prevU = stem.rfind('_', lastU - 1);
            if (lastU != std::string::npos && prevU != std::string::npos) {
                try {
                    int tx = std::stoi(stem.substr(prevU + 1, lastU - prevU - 1));
                    int ty = std::stoi(stem.substr(lastU + 1));
                    if (tx == loadedTileX_ && ty == loadedTileY_) continue;
                    manifest.tiles.push_back({tx, ty});
                } catch (...) {}
            }
        }
    }
    manifest.hasCreatures = (npcSpawner_.spawnCount() > 0);
    manifest.baseHeight = terrain_.chunks[0].position[2];
    manifest.save(base + "/zone.json");

    lastSavePath_ = outputDir;

    // Count exported files
    int fileCount = 2; // ADT + WDT always
    fileCount += 2; // WOT + WHM always
    fileCount += 3; // heightmap + normals + watermask PNGs
    fileCount += 1; // thumbnail PNG
    fileCount += 1; // zone.json always
    fileCount += 1; // README always
    if (!usedTextures.empty()) fileCount += static_cast<int>(usedTextures.size()); // PNG textures
    if (objectPlacer_.objectCount() > 0) fileCount++; // objects.json
    if (npcSpawner_.spawnCount() > 0) fileCount++; // creatures.json
    if (questEditor_.questCount() > 0) fileCount++; // quests.json

    // Validate open format completeness
    auto validation = ContentPacker::validateZone(base);
    int score = validation.openFormatScore();
    // Write zone statistics JSON
    {
        nlohmann::json sj;
        sj["map"] = loadedMap_;
        sj["tile"] = {loadedTileX_, loadedTileY_};
        sj["objects"] = objectPlacer_.objectCount();
        sj["npcs"] = npcSpawner_.spawnCount();
        sj["quests"] = questEditor_.questCount();
        sj["textures"] = usedTextures.size();
        sj["openFormatScore"] = score;
        sj["formats"] = validation.summary();
        sj["tiles"] = static_cast<int>(manifest.tiles.size());
        auto* tr = viewport_.getTerrainRenderer();
        if (tr) {
            sj["chunks"] = tr->getChunkCount();
            sj["triangles"] = tr->getTriangleCount();
        }
        sj["editorVersion"] = "1.0.0";
        std::ofstream stats(base + "/stats.json");
        if (stats) stats << sj.dump(2) << "\n";
    }

    std::string summary = std::to_string(fileCount) + " files exported";
    if (objectPlacer_.objectCount() > 0) summary += ", " + std::to_string(objectPlacer_.objectCount()) + " obj";
    if (npcSpawner_.spawnCount() > 0) summary += ", " + std::to_string(npcSpawner_.spawnCount()) + " NPC";
    if (questEditor_.questCount() > 0) summary += ", " + std::to_string(questEditor_.questCount()) + " quest";
    summary += " (score " + std::to_string(score) + "/7)";
    showToast(summary, 5.0f);
    LOG_INFO("=== Zone Export Summary ===");
    LOG_INFO("  Output: ", base);
    LOG_INFO("  Open format score: ", score, "/7");
    LOG_INFO("  Formats: ", validation.summary());
    LOG_INFO("  Terrain: WOT/WHM + heightmap/normals PNG");
    LOG_INFO("  Textures: ", usedTextures.size(), " BLP→PNG");
    LOG_INFO("  Objects: ", objectPlacer_.objectCount(), " placed");
    LOG_INFO("  NPCs: ", npcSpawner_.spawnCount(), " creatures");
    LOG_INFO("  Quests: ", questEditor_.questCount());
    LOG_INFO("========================");

    // Any path through exportZone counts as a save - clear the pending flag
    // so the dirty asterisk and quit-confirm dialog go away.
    autoSavePendingChanges_ = false;
}

void EditorApp::exportContentPack(const std::string& destPath) {
    if (!terrain_.isLoaded()) return;
    // Save zone first
    std::string dir = lastSavePath_.empty() ? "output" : lastSavePath_;
    exportZone(dir);
    // Pack into WCP
    ContentPackInfo info;
    info.name = loadedMap_;
    info.author = project_.author.empty() ? "Kelsi Davis" : project_.author;
    info.description = project_.description.empty()
        ? "Custom zone created with Wowee World Editor" : project_.description;
    // Honor the user-edited Map ID from the zone manifest panel rather than
    // always emitting 9000.
    info.mapId = zoneManifest_.mapId != 0 ? zoneManifest_.mapId : 9000;
    if (ContentPacker::packZone(dir, loadedMap_, destPath, info)) {
        // Report on-disk size so users can sanity-check the export. WCPs of
        // a few MB are normal; tens of MB usually means lots of textures.
        std::error_code ec;
        auto bytes = std::filesystem::file_size(destPath, ec);
        if (!ec) {
            char msg[256];
            std::snprintf(msg, sizeof(msg), "Content pack exported: %s (%.1f MB)",
                          destPath.c_str(), bytes / (1024.0 * 1024.0));
            showToast(msg);
        } else {
            showToast("Content pack exported: " + destPath);
        }
    } else {
        showToast("Failed to create content pack");
    }
}

void EditorApp::exportOpenFormat(const std::string& basePath) {
    if (!terrain_.isLoaded()) return;
    std::string base = basePath + "/" + loadedMap_ + "/" + loadedMap_ + "_" +
                       std::to_string(loadedTileX_) + "_" + std::to_string(loadedTileY_);
    if (WoweeTerrain::exportOpen(terrain_, base, loadedTileX_, loadedTileY_))
        showToast("Open format exported (.wot + .whm)");
    else
        showToast("Open format export failed");
}

void EditorApp::quickSave() {
    if (!terrain_.isLoaded()) return;
    std::string dir = lastSavePath_.empty() ? "output" : lastSavePath_;
    exportZone(dir);
    autoSavePendingChanges_ = false;
}

void EditorApp::requestQuit() {
    window_->setShouldClose(true);
}

void EditorApp::showToast(const std::string& msg, float duration) {
    toasts_.push_back({msg, duration});
}

void EditorApp::updateToasts(float dt) {
    for (auto& t : toasts_) t.timer -= dt;
    toasts_.erase(std::remove_if(toasts_.begin(), toasts_.end(),
                  [](const Toast& t) { return t.timer <= 0; }), toasts_.end());
}

void EditorApp::setSkyPreset(int preset) {
    switch (preset) {
        case 0: viewport_.setTimeOfDay(12.0f); break;
        case 1: viewport_.setTimeOfDay(18.0f); break;
        case 2: viewport_.setTimeOfDay(22.0f); break;
    }
}

void EditorApp::startGizmoMode(TransformMode mode) {
    auto& giz = viewport_.getGizmo();
    giz.setMode(mode);
    auto& io = ImGui::GetIO();
    giz.beginDrag(glm::vec2(io.MousePos.x, io.MousePos.y));
}

void EditorApp::setGizmoAxis(TransformAxis axis) {
    viewport_.getGizmo().setAxis(axis);
    if (auto* sel = objectPlacer_.getSelected())
        viewport_.getGizmo().setTarget(sel->position, sel->scale);
}

void EditorApp::saveBookmark(const std::string& name) {
    CameraBookmark bm;
    bm.pos = camera_.getCamera().getPosition();
    bm.yaw = 0; bm.pitch = 0; // EditorCamera doesn't expose these directly
    bm.name = name.empty() ? ("Bookmark " + std::to_string(bookmarks_.size() + 1)) : name;
    bookmarks_.push_back(bm);
}

void EditorApp::loadBookmark(int index) {
    if (index < 0 || index >= static_cast<int>(bookmarks_.size())) return;
    camera_.setPosition(bookmarks_[index].pos);
}

void EditorApp::addAdjacentTile(int offsetX, int offsetY) {
    if (!terrain_.isLoaded()) return;
    int newX = loadedTileX_ + offsetX;
    int newY = loadedTileY_ + offsetY;
    if (newX < 0 || newX > 63 || newY < 0 || newY > 63) return;

    // Source base height could be NaN if mid-edit terrain hadn't stitched -
    // fall back to a safe default so the new tile starts clean.
    float baseHeight = terrain_.chunks[0].position[2];
    if (!std::isfinite(baseHeight)) baseHeight = 100.0f;
    auto adj = TerrainEditor::createBlankTerrain(newX, newY, baseHeight,
                                                  Biome::Grassland);

    // Stitch edge heights from current tile to adjacent tile
    if (offsetX == 1) {
        for (int cx = 0; cx < 16; cx++) {
            auto& src = terrain_.chunks[15 * 16 + cx];
            auto& dst = adj.chunks[0 * 16 + cx];
            if (!src.hasHeightMap() || !dst.hasHeightMap()) continue;
            for (int v = 0; v < 9; v++)
                dst.heightMap.heights[v] = src.heightMap.heights[8 * 17 + v];
        }
    } else if (offsetX == -1) {
        for (int cx = 0; cx < 16; cx++) {
            auto& src = terrain_.chunks[0 * 16 + cx];
            auto& dst = adj.chunks[15 * 16 + cx];
            if (!src.hasHeightMap() || !dst.hasHeightMap()) continue;
            for (int v = 0; v < 9; v++)
                dst.heightMap.heights[8 * 17 + v] = src.heightMap.heights[v];
        }
    } else if (offsetY == 1) {
        for (int cy = 0; cy < 16; cy++) {
            auto& src = terrain_.chunks[cy * 16 + 15];
            auto& dst = adj.chunks[cy * 16 + 0];
            if (!src.hasHeightMap() || !dst.hasHeightMap()) continue;
            for (int r = 0; r <= 8; r++)
                dst.heightMap.heights[r * 17] = src.heightMap.heights[r * 17 + 8];
        }
    } else if (offsetY == -1) {
        for (int cy = 0; cy < 16; cy++) {
            auto& src = terrain_.chunks[cy * 16 + 0];
            auto& dst = adj.chunks[cy * 16 + 15];
            if (!src.hasHeightMap() || !dst.hasHeightMap()) continue;
            for (int r = 0; r <= 8; r++)
                dst.heightMap.heights[r * 17 + 8] = src.heightMap.heights[r * 17];
        }
    }

    std::string base = "output/" + loadedMap_ + "/" + loadedMap_ + "_" +
                       std::to_string(newX) + "_" + std::to_string(newY);
    ADTWriter::write(adj, base + ".adt");
    WoweeTerrain::exportOpen(adj, base, newX, newY);

    showToast("Adjacent tile [" + std::to_string(newX) + "," + std::to_string(newY) + "] exported");
    LOG_INFO("Adjacent tile created at [", newX, ",", newY, "] with edge stitching (ADT+WOT/WHM)");
}

void EditorApp::flyToSelected() {
    glm::vec3 target;
    bool have = false;
    if (auto* sel = objectPlacer_.getSelected()) {
        target = sel->position;
        have = true;
    } else if (auto* npc = npcSpawner_.getSelected()) {
        target = npc->position;
        have = true;
    }
    if (!have) return;
    // Reject NaN target - would feed NaN into camera.setPosition (which
    // now refuses it but the user would get no camera movement and no
    // error message). Show the toast instead so the user knows the
    // selection is corrupt.
    if (!std::isfinite(target.x) || !std::isfinite(target.y) ||
        !std::isfinite(target.z)) {
        showToast("Selection has non-finite position; can't fly to it");
        return;
    }

    // Place camera back-and-up from the target along the current view direction
    // and aim it at the target. Distance scales with camera speed so it works
    // both for tight spawn lists and for far-flung WMOs.
    glm::vec3 fwd = camera_.getCamera().getForward();
    if (glm::length(fwd) < 0.001f) fwd = glm::vec3(1, 0, 0);
    // Project onto XY before normalizing - but if the camera is looking
    // straight up/down, the projection is zero and glm::normalize returns
    // NaN. NaN length < 0.001f is false (NaN comparisons return false), so
    // the original fallback didn't catch the case. Length-check the source
    // vector explicitly.
    glm::vec3 fwdXY(fwd.x, fwd.y, 0.0f);
    glm::vec3 back = (glm::length(fwdXY) < 0.001f)
                         ? glm::vec3(-1, 0, 0)
                         : -glm::normalize(fwdXY);

    glm::vec3 cam = target + back * 25.0f + glm::vec3(0, 0, 15);
    camera_.setPosition(cam);

    // Aim at target. Camera::getForward = (cos(yaw), sin(yaw), sin(pitch)),
    // so to make it parallel to `to` we use atan2(to.y, to.x), not (x, y).
    // The previous swap pointed the camera 90deg off from the target.
    glm::vec3 to = target - cam;
    float yaw = glm::degrees(std::atan2(to.y, to.x));
    float horiz = std::sqrt(to.x * to.x + to.y * to.y);
    float pitch = glm::degrees(std::atan2(to.z, horiz));
    camera_.setYawPitch(yaw, pitch);
}

void EditorApp::generateCompleteZone() {
    if (!terrain_.isLoaded()) return;
    showToast("Generating zone...");

    // Step 0: Reset first for clean slate
    terrainEditor_.resetToFlat();

    // Step 1: Apply noise
    terrainEditor_.applyNoise(0.005f, 30.0f, 4, 42);

    // Step 2: Smooth
    terrainEditor_.smoothEntireTile(3);

    // Step 3: Recalc normals for slope paint
    std::vector<int> allChunks;
    for (int i = 0; i < 256; i++) allChunks.push_back(i);
    terrainEditor_.recalcNormals(allChunks);

    // Step 4: Auto-paint by height - use the active biome's textures so
    // subsequent generations honor the user's biome choice. Was previously
    // hardcoded to Tanaris/Elwynn/Barrens regardless of biome, which made
    // every "Create + Generate" run look the same.
    const auto& bt = getBiomeTextures(activeBiome_);
    std::vector<TexturePainter::HeightBand> bands = {
        {90.0f,    bt.secondary},  // low (sand-like layer)
        {110.0f,   bt.base},       // mid (primary ground)
        {140.0f,   bt.accent},     // high (rocks/roots)
        {99999.0f, bt.detail},     // peak (overlay)
    };
    texturePainter_.autoPaintByHeight(bands);

    // Step 5: Slope paint (rock on cliffs) - use the biome's accent so it
    // blends with the rest of the palette.
    texturePainter_.autoPaintBySlope(0.4f, bt.accent);

    // Step 6: Add detail roughness
    terrainEditor_.addDetailNoise(1.5f, 0.08f, 77);

    // Step 6b: Final normal recalculation after detail noise
    terrainEditor_.recalcNormals(allChunks);

    // Step 7: Fill low areas with water and smooth beaches
    float waterLevel = terrain_.chunks[0].position[2] + 5.0f;
    terrainEditor_.fillWater(waterLevel, 0);
    terrainEditor_.smoothBeaches(waterLevel, 12.0f);

    // Refresh
    auto mesh = terrainEditor_.regenerateMesh();
    viewport_.clearTerrain();
    viewport_.loadTerrain(mesh, terrain_.textures, loadedTileX_, loadedTileY_);

    showToast("Zone generated!");
}

void EditorApp::randomPopulateZone(int creatureCount, int objectCount,
                                      uint32_t seed) {
    if (!terrain_.isLoaded() || loadedTileX_ < 0 || loadedTileY_ < 0) {
        showToast("Load a tile first");
        return;
    }
    // Loaded tile world bbox. Each tile is 533.33y; WoW grid centers
    // tile (32, 32) at origin (+X = -wowY tile, +Y = -wowX tile).
    constexpr float kTileSize = 533.33333f;
    float wMinX = (32.0f - loadedTileY_ - 1) * kTileSize;
    float wMaxX = (32.0f - loadedTileY_)     * kTileSize;
    float wMinY = (32.0f - loadedTileX_ - 1) * kTileSize;
    float wMaxY = (32.0f - loadedTileX_)     * kTileSize;
    float baseZ = terrain_.chunks[0].position[2];

    uint32_t rng = seed ? seed : 1u;
    auto next01 = [&]() {
        rng = rng * 1664525u + 1013904223u;
        return (rng >> 8) / float(1 << 24);
    };
    auto rangeF = [&](float a, float b) { return a + next01() * (b - a); };
    auto rangeI = [&](int a, int b) {
        return a + static_cast<int>(next01() * (b - a + 1));
    };
    static const std::vector<std::pair<const char*, uint32_t>> kRandomCreatures = {
        {"Wolf", 5}, {"Boar", 4}, {"Bear", 7}, {"Spider", 3},
        {"Bandit", 6}, {"Kobold", 4}, {"Murloc", 5}, {"Skeleton", 5},
        {"Wisp", 3}, {"Goblin", 5}, {"Stag", 4}, {"Crab", 3},
    };
    static const std::vector<const char*> kRandomObjects = {
        "World/Generic/Tree01.wmo",
        "World/Generic/Boulder.wmo",
        "World/Generic/Bush.wmo",
        "World/Generic/Stump.wmo",
        "World/Generic/Mushroom.wmo",
    };
    // Ground-snap helper: cast a downward ray from far above the (x,y)
    // position to find the actual terrain height. Without this, spawns
    // sit at baseZ which can be metres below or above the carved
    // terrain after generation.
    auto groundZ = [&](float x, float y) -> float {
        rendering::Ray ray;
        ray.origin = glm::vec3(x, y, baseZ + 500.0f);
        ray.direction = glm::vec3(0, 0, -1);
        glm::vec3 hit;
        if (terrainEditor_.raycastTerrain(ray, hit)) return hit.z;
        return baseZ;  // fall back to base if the ray misses
    };
    int placedCreatures = 0, placedObjects = 0;
    for (int n = 0; n < creatureCount; ++n) {
        const auto& [name, baseLvl] = kRandomCreatures[
            rangeI(0, static_cast<int>(kRandomCreatures.size()) - 1)];
        CreatureSpawn s;
        s.name = name;
        s.position.x = rangeF(wMinX, wMaxX);
        s.position.y = rangeF(wMinY, wMaxY);
        s.position.z = groundZ(s.position.x, s.position.y);
        int lvl = std::max(1, static_cast<int>(baseLvl) + rangeI(-1, 2));
        s.level = static_cast<uint32_t>(lvl);
        s.health = 50 + s.level * 10;
        s.orientation = rangeF(0.0f, 360.0f);
        npcSpawner_.placeCreature(s);
        placedCreatures++;
    }
    auto& objs = objectPlacer_.getObjects();
    uint32_t maxUid = 0;
    for (const auto& o : objs) maxUid = std::max(maxUid, o.uniqueId);
    for (int n = 0; n < objectCount; ++n) {
        PlacedObject o;
        o.path = kRandomObjects[
            rangeI(0, static_cast<int>(kRandomObjects.size()) - 1)];
        o.type = PlaceableType::WMO;
        o.position.x = rangeF(wMinX, wMaxX);
        o.position.y = rangeF(wMinY, wMaxY);
        o.position.z = groundZ(o.position.x, o.position.y);
        o.rotation = glm::vec3(0.0f, rangeF(0.0f, 6.28f), 0.0f);
        o.scale = rangeF(0.8f, 1.4f);
        o.uniqueId = ++maxUid;
        o.nameId = 0;
        o.selected = false;
        objs.push_back(o);
        placedObjects++;
    }
    objectsDirty_ = true;
    autoSavePendingChanges_ = true;
    viewport_.updateNpcMarkers(npcSpawner_.getSpawns());
    showToast("Populated: " + std::to_string(placedCreatures) +
              " creatures + " + std::to_string(placedObjects) + " objects");
}

void EditorApp::snapAllSpawnsToGround() {
    if (!terrain_.isLoaded()) {
        showToast("Load a tile first");
        return;
    }
    auto castDown = [&](const glm::vec3& pos, glm::vec3& hit) {
        rendering::Ray ray;
        ray.origin = pos + glm::vec3(0, 0, 500);
        ray.direction = glm::vec3(0, 0, -1);
        return terrainEditor_.raycastTerrain(ray, hit);
    };
    int snappedC = 0, snappedO = 0;
    for (auto& s : npcSpawner_.getSpawns()) {
        glm::vec3 hit;
        if (castDown(s.position, hit)) {
            s.position.z = hit.z;
            snappedC++;
        }
        for (auto& wp : s.patrolPath) {
            if (castDown(wp.position, hit)) wp.position.z = hit.z;
        }
    }
    for (auto& o : objectPlacer_.getObjects()) {
        glm::vec3 hit;
        if (castDown(o.position, hit)) {
            o.position.z = hit.z;
            snappedO++;
        }
    }
    if (snappedC > 0 || snappedO > 0) {
        objectsDirty_ = true;
        autoSavePendingChanges_ = true;
        viewport_.updateNpcMarkers(npcSpawner_.getSpawns());
    }
    showToast("Snapped " + std::to_string(snappedC) + " creature(s) + " +
              std::to_string(snappedO) + " object(s) to ground");
}

int EditorApp::auditSpawnsAgainstTerrain(float threshold) const {
    if (!terrain_.isLoaded()) return 0;
    auto castDown = [&](const glm::vec3& pos, glm::vec3& hit) {
        rendering::Ray ray;
        ray.origin = pos + glm::vec3(0, 0, 500);
        ray.direction = glm::vec3(0, 0, -1);
        return const_cast<TerrainEditor&>(terrainEditor_).raycastTerrain(ray, hit);
    };
    int issues = 0;
    for (const auto& s : npcSpawner_.getSpawns()) {
        glm::vec3 hit;
        if (castDown(s.position, hit)) {
            if (std::fabs(s.position.z - hit.z) > threshold) issues++;
        }
    }
    for (const auto& o : objectPlacer_.getObjects()) {
        glm::vec3 hit;
        if (castDown(o.position, hit)) {
            if (std::fabs(o.position.z - hit.z) > threshold) issues++;
        }
    }
    return issues;
}

void EditorApp::clearAllObjects() {
    vkDeviceWaitIdle(window_->getVkContext()->getDevice());
    objectPlacer_.clearAll();
    npcSpawner_.clearAll();
    viewport_.clearObjects();
    viewport_.updateNpcMarkers({});
    terrainEditor_.history().clear();
    lastObjCount_ = 0;
    lastNpcCount_ = 0;
    objectsDirty_ = false;
    showToast("All objects and NPCs cleared");
}

void EditorApp::centerOnTerrain() {
    if (!terrain_.isLoaded()) return;
    auto mesh = terrainEditor_.regenerateMesh();
    if (mesh.validChunkCount > 0) {
        float cx = (mesh.chunks[0].worldX + mesh.chunks[255].worldX) * 0.5f;
        float cy = (mesh.chunks[0].worldY + mesh.chunks[255].worldY) * 0.5f;
        camera_.setPosition(glm::vec3(cx, cy, terrain_.chunks[0].position[2] + 300.0f));
    }
    camera_.setYawPitch(0.0f, -45.0f);
    showToast("Camera centered on terrain");
}

void EditorApp::snapSelectedToGround() {
    if (!terrain_.isLoaded()) return;

    auto castDown = [&](const glm::vec3& pos, glm::vec3& hit) {
        rendering::Ray ray;
        ray.origin = pos + glm::vec3(0, 0, 500);
        ray.direction = glm::vec3(0, 0, -1);
        return terrainEditor_.raycastTerrain(ray, hit);
    };

    if (auto* sel = objectPlacer_.getSelected()) {
        glm::vec3 hitPos;
        if (castDown(sel->position, hitPos)) {
            sel->position.z = hitPos.z;
            objectsDirty_ = true; autoSavePendingChanges_ = true;
        }
        return;
    }

    if (auto* npc = npcSpawner_.getSelected()) {
        glm::vec3 hitPos;
        if (castDown(npc->position, hitPos)) {
            npc->position.z = hitPos.z;
            objectsDirty_ = true; autoSavePendingChanges_ = true;
        }
        // Also snap each patrol waypoint
        for (auto& wp : npc->patrolPath) {
            if (castDown(wp.position, hitPos)) wp.position.z = hitPos.z;
        }
    }
}

void EditorApp::flattenAroundSelected(float radius) {
    auto* sel = objectPlacer_.getSelected();
    if (!sel || !terrain_.isLoaded()) return;
    if (!std::isfinite(radius) || radius <= 0.0f ||
        !std::isfinite(sel->position.x) || !std::isfinite(sel->position.y) ||
        !std::isfinite(sel->position.z)) return;

    terrainEditor_.beginGeneratorUndo();
    float targetHeight = sel->position.z;
    for (int ci = 0; ci < 256; ci++) {
        auto& chunk = terrain_.chunks[ci];
        if (!chunk.hasHeightMap()) continue;
        bool modified = false;
        for (int v = 0; v < 145; v++) {
            glm::vec3 vpos = terrainEditor_.getChunkVertexWorldPos(ci, v);
            float dist = glm::length(glm::vec2(vpos.x - sel->position.x, vpos.y - sel->position.y));
            if (dist >= radius) continue;
            float t = dist / radius;
            float blend = t * t;
            float relTarget = targetHeight - chunk.position[2];
            chunk.heightMap.heights[v] = chunk.heightMap.heights[v] * blend + relTarget * (1.0f - blend);
            modified = true;
        }
        if (modified) {
            terrainEditor_.stitchChunkEdges(ci);
            terrainEditor_.markDirty(ci);
        }
    }
    terrainEditor_.endGeneratorUndo();
    showToast("Flattened terrain around object (r=" + std::to_string(static_cast<int>(radius)) + ")");
}

void EditorApp::alignSelectedToTerrain() {
    auto& indices = objectPlacer_.getSelectedIndices();
    auto& objects = objectPlacer_.getObjects();
    int count = 0;
    auto alignOne = [&](PlacedObject& obj) {
        glm::vec3 normal = terrainEditor_.sampleTerrainNormal(obj.position);
        float pitchDeg = glm::degrees(std::asin(-normal.x));
        float rollDeg = glm::degrees(std::asin(normal.y));
        obj.rotation.x = pitchDeg;
        obj.rotation.z = rollDeg;
        count++;
    };
    if (!indices.empty()) {
        for (int idx : indices) alignOne(objects[idx]);
    } else if (auto* sel = objectPlacer_.getSelected()) {
        alignOne(*sel);
    }
    if (count > 0) {
        objectsDirty_ = true; autoSavePendingChanges_ = true;
        showToast("Aligned " + std::to_string(count) + " object(s) to terrain");
    }
}

int EditorApp::batchConvertAssets(const std::string& dataDir) {
    namespace fs = std::filesystem;
    int converted = 0;

    // Collect paths from filesystem or manifest
    std::vector<std::string> assetPaths;
    if (fs::exists(dataDir)) {
        for (auto& entry : fs::recursive_directory_iterator(dataDir)) {
            if (entry.is_regular_file())
                assetPaths.push_back(fs::relative(entry.path(), dataDir).string());
        }
    }
    if (assetPaths.empty() && assetManager_) {
        for (const auto& [path, _] : assetManager_->getManifest().getEntries())
            assetPaths.push_back(path);
        LOG_INFO("Batch convert: using manifest (", assetPaths.size(), " entries)");
    }

    for (const auto& relPath : assetPaths) {
        std::string ext;
        auto dot = relPath.rfind('.');
        if (dot != std::string::npos) ext = relPath.substr(dot);
        for (char& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

        if (ext == ".m2") {
            auto wom = pipeline::WoweeModelLoader::fromM2(relPath, assetManager_.get());
            if (wom.isValid()) {
                std::string outPath = relPath;
                auto dot = outPath.rfind('.');
                if (dot != std::string::npos) outPath = outPath.substr(0, dot);
                pipeline::WoweeModelLoader::save(wom, "output/models/" + outPath);
                converted++;
            }
        } else if (ext == ".wmo") {
            auto wmoData = assetManager_->readFile(relPath);
            if (!wmoData.empty()) {
                auto wmoModel = pipeline::WMOLoader::load(wmoData);
                if (wmoModel.nGroups > 0) {
                    std::string wmoBase = relPath;
                    if (wmoBase.size() > 4) wmoBase = wmoBase.substr(0, wmoBase.size() - 4);
                    for (uint32_t gi = 0; gi < wmoModel.nGroups; gi++) {
                        char suffix[16];
                        snprintf(suffix, sizeof(suffix), "_%03u.wmo", gi);
                        auto gd = assetManager_->readFile(wmoBase + suffix);
                        if (!gd.empty()) pipeline::WMOLoader::loadGroup(gd, wmoModel, gi);
                    }
                }
                auto wob = pipeline::WoweeBuildingLoader::fromWMO(wmoModel, relPath);
                if (wob.isValid()) {
                    std::string outPath = relPath;
                    auto dot = outPath.rfind('.');
                    if (dot != std::string::npos) outPath = outPath.substr(0, dot);
                    pipeline::WoweeBuildingLoader::save(wob, "output/buildings/" + outPath);
                    converted++;
                }
            }
        }
    }
    LOG_INFO("Batch converted ", converted, " assets from ", dataDir);
    return converted;
}

void EditorApp::resetCamera() {
    camera_.setPosition(glm::vec3(0.0f, 0.0f, 300.0f));
    camera_.setYawPitch(0.0f, -30.0f);
}

void EditorApp::setWireframe(bool enabled) {
    viewport_.setWireframe(enabled);
}

bool EditorApp::isWireframe() const {
    return viewport_.isWireframe();
}

rendering::TerrainRenderer* EditorApp::getTerrainRenderer() {
    return viewport_.getTerrainRenderer();
}

void EditorApp::initImGui() {
    auto* vkCtx = window_->getVkContext();

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename = "wowee_editor_layout.ini";

    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.FrameRounding = 2.0f;
    style.GrabRounding = 2.0f;

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg] = ImVec4(0.12f, 0.12f, 0.14f, 0.95f);
    colors[ImGuiCol_TitleBg] = ImVec4(0.10f, 0.10f, 0.12f, 1.00f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.18f, 0.18f, 0.25f, 1.00f);
    colors[ImGuiCol_MenuBarBg] = ImVec4(0.14f, 0.14f, 0.18f, 1.00f);
    colors[ImGuiCol_Button] = ImVec4(0.24f, 0.28f, 0.40f, 1.00f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.30f, 0.35f, 0.50f, 1.00f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.20f, 0.24f, 0.36f, 1.00f);

    ImGui_ImplSDL3_InitForVulkan(window_->getSDLWindow());

    ImGui_ImplVulkan_InitInfo initInfo{};
    initInfo.ApiVersion = VK_API_VERSION_1_1;
    initInfo.Instance = vkCtx->getInstance();
    initInfo.PhysicalDevice = vkCtx->getPhysicalDevice();
    initInfo.Device = vkCtx->getDevice();
    initInfo.QueueFamily = vkCtx->getGraphicsQueueFamily();
    initInfo.Queue = vkCtx->getGraphicsQueue();
    initInfo.DescriptorPool = vkCtx->getImGuiDescriptorPool();
    initInfo.MinImageCount = 2;
    initInfo.ImageCount = vkCtx->getSwapchainImageCount();
    initInfo.PipelineInfoMain.RenderPass = vkCtx->getImGuiRenderPass();
    initInfo.PipelineInfoMain.MSAASamples = vkCtx->getMsaaSamples();

    ImGui_ImplVulkan_Init(&initInfo);
    imguiInitialized_ = true;
}

void EditorApp::shutdownImGui() {
    if (!imguiInitialized_) return;
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    imguiInitialized_ = false;
}

} // namespace editor
} // namespace wowee
