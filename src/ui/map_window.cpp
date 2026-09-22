#include "ui/map_window.hpp"

#include "addons/lua_api_helpers.hpp"
#include "core/application.hpp"
#include "core/config_paths.hpp"
#include "game/game_handler.hpp"
#include "game/quest_handler.hpp"
#include "game/quest_objectives.hpp"
#include "core/logger.hpp"
#include "rendering/aux_swapchain.hpp"
#include "rendering/renderer.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/world_map/world_map_facade.hpp"
#include "ui/imgui_theme.hpp"
#include "ui/interface_fonts.hpp"
#include "ui/ui_manager.hpp"

#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_vulkan.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace wowee {
namespace ui {

namespace {

// At most this often: the map changes as fast as a player walks, and every
// frame of it costs the game a map composite and an interface draw.
constexpr uint64_t kFrameIntervalNs = 1'000'000'000ull / 30;

// Left alone this long after being browsed, the map goes back to the player.
constexpr uint64_t kReturnToPlayerNs = 30ull * 1'000'000'000ull;

// The map's own background, the colour its composite pass clears to - so the
// bars either side of the letterboxed map are part of it rather than a frame.
constexpr float kBackground[4] = {13.0f / 255.0f, 20.0f / 255.0f, 31.0f / 255.0f, 1.0f};

// The shape of the map art: 1002 by 668 of its 1024 by 768 image is map.
constexpr float kMapAspect = 1002.0f / 668.0f;

std::string geometryFile() { return core::getConfigRoot() + "/map_window.cfg"; }

/// Makes `ctx` current for the scope, and puts back whatever was.
struct ContextScope {
    ImGuiContext* previous;
    explicit ContextScope(ImGuiContext* ctx) : previous(ImGui::GetCurrentContext()) {
        ImGui::SetCurrentContext(ctx);
    }
    ~ContextScope() { ImGui::SetCurrentContext(previous); }
    ContextScope(const ContextScope&) = delete;
    ContextScope& operator=(const ContextScope&) = delete;
};

}  // namespace

MapWindow::MapWindow() = default;
MapWindow::~MapWindow() { close(); }

void MapWindow::chooseGeometry(SDL_Window* gameWindow, int& x, int& y, int& w, int& h) const {
    int count = 0;
    SDL_DisplayID* displays = SDL_GetDisplays(&count);

    // Where it was last put, if that is still on a display: a monitor that has
    // since been unplugged would otherwise open the map somewhere unreachable.
    if (std::ifstream in(geometryFile()); in && (in >> x >> y >> w >> h) && w > 100 && h > 100) {
        const SDL_Point centre{x + w / 2, y + h / 2};
        for (int i = 0; i < count; ++i) {
            SDL_Rect bounds{};
            if (SDL_GetDisplayBounds(displays[i], &bounds) &&
                SDL_PointInRect(&centre, &bounds)) {
                SDL_free(displays);
                return;
            }
        }
    }

    // First time: another monitor than the game's, if there is one, since
    // that is what this window is for.
    const SDL_DisplayID gameDisplay = gameWindow ? SDL_GetDisplayForWindow(gameWindow) : 0;
    SDL_DisplayID target = gameDisplay;
    for (int i = 0; i < count; ++i) {
        if (displays[i] != gameDisplay) { target = displays[i]; break; }
    }
    SDL_free(displays);

    SDL_Rect usable{0, 0, 1280, 800};
    if (target != 0) SDL_GetDisplayUsableBounds(target, &usable);
    const bool ownMonitor = target != gameDisplay;
    w = static_cast<int>(usable.w * (ownMonitor ? 0.8f : 0.5f));
    h = static_cast<int>(static_cast<float>(w) / kMapAspect);
    if (h > usable.h * 0.85f) {
        h = static_cast<int>(usable.h * 0.85f);
        w = static_cast<int>(static_cast<float>(h) * kMapAspect);
    }
    x = usable.x + (usable.w - w) / 2;
    y = usable.y + (usable.h - h) / 2;
}

void MapWindow::saveGeometry() const {
    if (!window_) return;
    int x = 0, y = 0, w = 0, h = 0;
    SDL_GetWindowPosition(window_, &x, &y);
    SDL_GetWindowSize(window_, &w, &h);
    if (std::ofstream out(geometryFile(), std::ios::trunc); out) {
        out << x << " " << y << " " << w << " " << h << "\n";
    }
}

bool MapWindow::open(SDL_Window* gameWindow, rendering::Renderer* renderer,
                     rendering::VkContext* ctx, pipeline::AssetManager* assets,
                     const UIManager* ui) {
    if (window_) return true;
    if (!renderer || !ctx || !assets) return false;

    int x = 0, y = 0, w = 0, h = 0;
    chooseGeometry(gameWindow, x, y, w, h);
    window_ = SDL_CreateWindow("WoWee - Map", w, h,
                               SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE |
                                   SDL_WINDOW_HIGH_PIXEL_DENSITY);
    if (!window_) {
        LOG_ERROR("Map window not created: ", SDL_GetError());
        return false;
    }
    SDL_SetWindowPosition(window_, x, y);
    gameWindowId_ = gameWindow ? SDL_GetWindowID(gameWindow) : 0;
    renderer_ = renderer;
    ctx_ = ctx;

    swapchain_ = std::make_unique<rendering::AuxSwapchain>();
    if (!swapchain_->create(ctx, window_)) {
        close();
        return false;
    }

    // An ImGui context of its own. The previous one is put back afterwards:
    // this is opened from inside the game's frame.
    {
        ImGuiContext* previous = ImGui::GetCurrentContext();
        imgui_ = ImGui::CreateContext();
        ImGui::SetCurrentContext(imgui_);
        ImGuiIO& io = ImGui::GetIO();
        // No imgui.ini of its own, and hands off the pointer: the cursor is one
        // thing across every window, and the game's context already owns it.
        io.IniFilename = nullptr;
        io.LogFilename = nullptr;
        io.ConfigFlags |= ImGuiConfigFlags_NoMouseCursorChange;
        applyWoweeStyle(ImGui::GetStyle());

        // The game's own face, which the map's labels are drawn in over there.
        static const std::vector<uint8_t> kNoFace;
        const std::vector<uint8_t>& face = ui ? ui->clientFontData() : kNoFace;
        if (!face.empty()) {
            void* owned = IM_ALLOC(face.size());
            std::memcpy(owned, face.data(), face.size());
            ImFontConfig cfg;
            cfg.FontDataOwnedByAtlas = true;
            cfg.RasterizerMultiply = 1.35f;
            cfg.ExtraSizeScale = fontEmSizeScale(face.data(), face.size());
            io.Fonts->AddFontFromMemoryTTF(owned, static_cast<int>(face.size()),
                                           ui->clientFontSize(), &cfg);
        } else {
            io.Fonts->AddFontDefault();
        }

        ImGui_ImplSDL3_InitForVulkan(window_);
        ImGui_ImplVulkan_InitInfo info{};
        info.ApiVersion = VK_API_VERSION_1_1;
        info.Instance = ctx->getInstance();
        info.PhysicalDevice = ctx->getPhysicalDevice();
        info.Device = ctx->getDevice();
        info.QueueFamily = ctx->getGraphicsQueueFamily();
        info.Queue = ctx->getGraphicsQueue();
        // The game's pool, not one of its own. The map's layers make their
        // textures through whichever context is current when they load, and
        // free them through whichever is current when they go; one pool means
        // a set is always returned to the pool it came from.
        info.DescriptorPool = ctx->getImGuiDescriptorPool();
        info.MinImageCount = 2;
        info.ImageCount = std::max(2u, swapchain_->imageCount());
        info.PipelineInfoMain.RenderPass = swapchain_->renderPass();
        info.PipelineInfoMain.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
        info.CheckVkResultFn = [](VkResult err) {
            if (err != VK_SUCCESS) LOG_ERROR("Map window ImGui Vulkan error: ", static_cast<int>(err));
        };
        ImGui_ImplVulkan_Init(&info);
        ImGui::SetCurrentContext(previous);
    }

    // A map of its own, so this one and the in-game one can show different
    // places. Its textures are made under this window's context.
    map_ = std::make_unique<rendering::world_map::WorldMapFacade>();
    withContext([&] {
        if (!map_->initialize(ctx, assets)) {
            LOG_WARNING("Map window: the map did not initialise");
        }
        map_->setPersistent(true);
    });

    renderer_->setAfterInterfaceRecorder([this](VkCommandBuffer cmd) { record(cmd); });
    lastPlayerZone_ = 0;
    browsing_ = false;
    LOG_WARNING("Map window opened at ", x, ",", y, " ", w, "x", h);
    return true;
}

void MapWindow::close() {
    if (!window_) return;
    saveGeometry();
    if (renderer_) renderer_->setAfterInterfaceRecorder({});

    // Nothing of this window's may still be in flight: its pipelines, its
    // textures and its swapchain images are about to go.
    if (ctx_ && ctx_->getDevice() != VK_NULL_HANDLE) vkDeviceWaitIdle(ctx_->getDevice());

    if (imgui_) {
        ImGuiContext* previous = ImGui::GetCurrentContext();
        ImGui::SetCurrentContext(imgui_);
        // The map first, under the context its textures were made in.
        if (map_) map_->shutdown();
        ImGui_ImplVulkan_Shutdown();
        ImGui_ImplSDL3_Shutdown();
        ImGui::DestroyContext(imgui_);
        ImGui::SetCurrentContext(previous == imgui_ ? nullptr : previous);
        imgui_ = nullptr;
    } else if (map_) {
        map_->shutdown();
    }
    map_.reset();
    if (swapchain_) swapchain_->destroy();
    swapchain_.reset();

    SDL_DestroyWindow(window_);
    window_ = nullptr;
    frameReady_ = false;
    LOG_WARNING("Map window closed");
}

bool MapWindow::takeClosedByPlayer() {
    const bool was = closedByPlayer_;
    closedByPlayer_ = false;
    return was;
}

void MapWindow::withContext(const std::function<void()>& fn) {
    if (!imgui_) {
        fn();
        return;
    }
    ContextScope scope(imgui_);
    fn();
}

bool MapWindow::handleEvent(SDL_Event& event) {
    if (!window_) return false;
    // Only events that name a window can be this one's; quitting, a pad being
    // plugged in and the like are the game's.
    if (SDL_GetWindowFromEvent(&event) != window_) return false;

    if ((event.type == SDL_EVENT_KEY_DOWN || event.type == SDL_EVENT_KEY_UP) &&
        gameWindowId_ != 0) {
        event.key.windowID = gameWindowId_;
        return false;
    }

    switch (event.type) {
        case SDL_EVENT_WINDOW_CLOSE_REQUESTED:
            closedByPlayer_ = true;
            return true;
        case SDL_EVENT_WINDOW_RESIZED:
        case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED:
        case SDL_EVENT_WINDOW_DISPLAY_CHANGED:
            if (swapchain_) swapchain_->markDirty();
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
        case SDL_EVENT_MOUSE_WHEEL:
            // The player taking the map somewhere; see buildFrame.
            browsing_ = true;
            lastInteractionNs_ = SDL_GetTicksNS();
            break;
        default:
            break;
    }

    ContextScope scope(imgui_);
    ImGui_ImplSDL3_ProcessEvent(&event);
    return true;
}

void MapWindow::buildFrame(bool inWorld, const glm::vec3& playerRenderPos, float playerYawDeg,
                           uint32_t playerZoneId) {
    if (!window_ || !imgui_ || !map_) return;
    const uint64_t now = SDL_GetTicksNS();
    if (lastBuildNs_ != 0 && now - lastBuildNs_ < kFrameIntervalNs) return;
    // Minimised: nothing to draw into, and the swapchain would say so anyway.
    if (SDL_GetWindowFlags(window_) & SDL_WINDOW_MINIMIZED) return;
    lastBuildNs_ = now;

    ContextScope scope(imgui_);
    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();

    const ImVec2 display = ImGui::GetIO().DisplaySize;
    if (!inWorld) {
        // Before the world there is no player, no zone and nothing to show.
        const char* text = "The map appears once you are in the world.";
        const ImVec2 size = ImGui::CalcTextSize(text);
        ImGui::GetForegroundDrawList()->AddText(
            ImVec2((display.x - size.x) * 0.5f, (display.y - size.y) * 0.5f),
            IM_COL32(200, 190, 160, 255), text);
    } else {
        // Following the player: to their zone when they change zones, and back
        // to it when the map has been browsed and then left alone.
        if (playerZoneId != 0 && playerZoneId != lastPlayerZone_) {
            lastPlayerZone_ = playerZoneId;
            browsing_ = false;
            map_->showPlayerZone();
        } else if (browsing_ && now - lastInteractionNs_ > kReturnToPlayerNs) {
            browsing_ = false;
            map_->showPlayerZone();
        }

        // The quest log down the right, when there is room for it beside a
        // map still worth looking at; a narrow window is all map.
        const float panelW = std::clamp(display.x * 0.32f, 250.0f, 430.0f);
        const bool showPanel = display.x - panelW >= 360.0f;
        const float mapAreaW = showPanel ? display.x - panelW : display.x;

        // Letterboxed at the art's own shape, centred in what is left: stretched
        // the zone would be distorted, and every marker placed on it with it.
        float w = mapAreaW, h = mapAreaW / kMapAspect;
        if (h > display.y) {
            h = display.y;
            w = display.y * kMapAspect;
        }
        map_->setFrameRect(std::floor((mapAreaW - w) * 0.5f),
                           std::floor((display.y - h) * 0.5f), std::floor(w), std::floor(h));
        map_->render(playerRenderPos, static_cast<int>(display.x),
                     static_cast<int>(display.y), playerYawDeg);
        if (showPanel) drawQuestPanel(mapAreaW, panelW, display.y, now);
    }

    ImGui::Render();
    frameReady_ = true;
}

namespace {

/// The colour the game gives a quest's title for a player of this level: red
/// well above them, orange above, yellow near, green below, grey once it
/// gives nothing. GetQuestDifficultyColor's bands, and the server's own grey
/// line (questIsTrivial).
ImVec4 questDifficultyColour(int questLevel, int playerLevel) {
    if (questLevel <= 0 || playerLevel <= 0) return ImVec4(1.0f, 0.82f, 0.0f, 1.0f);
    const int diff = questLevel - playerLevel;
    if (diff >= 5) return ImVec4(1.0f, 0.1f, 0.1f, 1.0f);
    if (diff >= 3) return ImVec4(1.0f, 0.5f, 0.25f, 1.0f);
    if (diff >= -2) return ImVec4(1.0f, 0.82f, 0.0f, 1.0f);
    if (addons::questIsTrivial(playerLevel, questLevel)) return ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
    return ImVec4(0.25f, 0.75f, 0.25f, 1.0f);
}

}  // namespace

void MapWindow::drawQuestPanel(float x, float width, float height, uint64_t now) {
    auto* gh = core::Application::getInstance().getGameHandler();
    if (!gh) return;

    ImGui::SetNextWindowPos(ImVec2(x, 0.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(width, height), ImGuiCond_Always);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.09f, 0.075f, 0.055f, 0.97f));
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                                   ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                                   ImGuiWindowFlags_NoSavedSettings |
                                   ImGuiWindowFlags_NoBringToFrontOnFocus;
    if (!ImGui::Begin("##MapWindowQuests", nullptr, flags)) {
        ImGui::End();
        ImGui::PopStyleColor();
        return;
    }

    const auto& log = gh->getQuestLog();
    const ImVec4 gold(1.0f, 0.82f, 0.0f, 1.0f);
    ImGui::TextColored(gold, "Quest Log");
    ImGui::SameLine();
    const int slots = gh->getQuestHandler() ? gh->getQuestHandler()->maxQuestLogSlots() : 25;
    ImGui::TextDisabled("%d/%d", static_cast<int>(log.size()), slots);
    ImGui::Separator();
    if (log.empty()) ImGui::TextDisabled("No quests.");

    // By zone, as the game's log groups them, with the zone the player is in
    // first - those are the quests the map beside this is showing.
    const uint32_t hereZone = gh->getWorldStateZoneId();
    struct Group {
        std::string name;
        std::vector<const game::GameHandler::QuestLogEntry*> quests;
    };
    std::map<int32_t, Group> groups;
    for (const auto& quest : log) {
        if (quest.questId == 0) continue;
        Group& group = groups[quest.zoneOrSort];
        if (group.name.empty()) {
            if (quest.zoneOrSort > 0) group.name = gh->getAreaName(static_cast<uint32_t>(quest.zoneOrSort));
            else if (quest.zoneOrSort < 0) group.name = gh->getQuestSortName(static_cast<uint32_t>(-quest.zoneOrSort));
            if (group.name.empty()) group.name = "Other";
        }
        group.quests.push_back(&quest);
    }
    std::vector<const std::pair<const int32_t, Group>*> order;
    for (const auto& entry : groups) order.push_back(&entry);
    std::stable_sort(order.begin(), order.end(), [&](const auto* a, const auto* b) {
        const bool aHere = a->first > 0 && static_cast<uint32_t>(a->first) == hereZone;
        const bool bHere = b->first > 0 && static_cast<uint32_t>(b->first) == hereZone;
        if (aHere != bHere) return aHere;
        return a->second.name < b->second.name;
    });

    const int playerLevel = static_cast<int>(gh->getPlayerLevel());
    for (const auto* entry : order) {
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.85f, 0.75f, 0.55f, 1.0f), "%s", entry->second.name.c_str());
        for (const auto* quest : entry->second.quests) {
            ImGui::PushID(static_cast<int>(quest->questId));

            // Tracked, the same set the objectives tracker on the game's screen
            // shows; changing it here changes it there.
            bool tracked = gh->isQuestTracked(quest->questId);
            if (ImGui::Checkbox("##track", &tracked)) gh->setQuestTracked(quest->questId, tracked);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Track this quest");
            ImGui::SameLine();

            std::string label = quest->level > 0
                ? "[" + std::to_string(quest->level) + "] " + quest->title
                : quest->title;
            if (quest->failed) label += "  (Failed)";
            else if (quest->complete) label += "  (Complete)";
            const bool selected = selectedQuestId_ == quest->questId;
            ImGui::PushStyleColor(ImGuiCol_Text, questDifficultyColour(quest->level, playerLevel));
            if (ImGui::Selectable(label.c_str(), selected)) {
                // Chosen again, it lets go. Chosen fresh, the map goes to its
                // zone - as far as that zone is on the map now loaded - and
                // that counts as the player taking the map somewhere.
                selectedQuestId_ = selected ? 0 : quest->questId;
                if (!selected && quest->zoneOrSort > 0 &&
                    map_->showAreaZone(static_cast<uint32_t>(quest->zoneOrSort))) {
                    browsing_ = true;
                    lastInteractionNs_ = now;
                }
            }
            ImGui::PopStyleColor();

            ImGui::Indent(24.0f);
            for (const auto& line : game::questObjectives(*gh, *quest)) {
                if (line.finished) ImGui::TextDisabled("- %s", line.text.c_str());
                else ImGui::TextWrapped("- %s", line.text.c_str());
            }
            if (selected && !quest->description.empty()) {
                ImGui::Spacing();
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.78f, 0.74f, 0.66f, 1.0f));
                ImGui::TextWrapped("%s", quest->description.c_str());
                ImGui::PopStyleColor();
            }
            ImGui::Unindent(24.0f);
            ImGui::PopID();
        }
    }

    ImGui::End();
    ImGui::PopStyleColor();
}

void MapWindow::record(VkCommandBuffer cmd) {
    if (!frameReady_ || !swapchain_ || !map_) return;
    frameReady_ = false;
    // Not ready for another frame on its display: this one is skipped, and
    // the game goes on without waiting for it.
    if (!swapchain_->acquire()) return;

    // Outside any render pass, before the pass that draws what it composites.
    map_->compositePass(cmd);
    swapchain_->record(cmd, kBackground, [this](VkCommandBuffer c) {
        ContextScope scope(imgui_);
        ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), c);
    });
}

}  // namespace ui
}  // namespace wowee
