#include "rendering/performance_hud.hpp"
#include "rendering/renderer.hpp"
#include "core/application.hpp"
#include "core/thermal_state.hpp"
#include "rendering/post_process_pipeline.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/terrain_renderer.hpp"
#include "rendering/terrain_manager.hpp"
#include "rendering/water_renderer.hpp"
#include "rendering/skybox.hpp"
#include "rendering/celestial.hpp"
#include "rendering/starfield.hpp"
#include "rendering/clouds.hpp"
#include "rendering/lens_flare.hpp"
#include "rendering/weather.hpp"
#include "rendering/lightning.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/wmo_renderer.hpp"
#include "rendering/m2_renderer.hpp"
#include "rendering/camera.hpp"
#include "ui/ui_colors.hpp"
#include <imgui.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <iomanip>

namespace wowee {
namespace rendering {

namespace {
    using namespace wowee::ui;
    constexpr ImVec4 kSectionHeader = {0.8f, 0.8f, 0.5f, 1.0f};
    const auto& kHelpText = colors::kGray;
    const auto& kTitle    = colors::kLightGray;
} // namespace

PerformanceHUD::PerformanceHUD() {
}

PerformanceHUD::~PerformanceHUD() {
}

void PerformanceHUD::update(float deltaTime) {
    if (!enabled) {
        return;
    }

    // Store frame time
    frameTime = deltaTime;
    frameTimeHistory.push_back(deltaTime);

    // Keep history size limited
    while (frameTimeHistory.size() > MAX_FRAME_HISTORY) {
        frameTimeHistory.pop_front();
    }

    // Update stats periodically
    updateTimer += deltaTime;
    if (updateTimer >= UPDATE_INTERVAL) {
        updateTimer = 0.0f;
        calculateFPS();
    }
}

void PerformanceHUD::calculateFPS() {
    if (frameTimeHistory.empty()) {
        return;
    }

    // Current FPS (from last frame time)
    currentFPS = frameTime > 0.0001f ? 1.0f / frameTime : 0.0f;

    // Average FPS
    float sum = 0.0f;
    for (float ft : frameTimeHistory) {
        sum += ft;
    }
    float avgFrameTime = sum / frameTimeHistory.size();
    averageFPS = avgFrameTime > 0.0001f ? 1.0f / avgFrameTime : 0.0f;

    // Min/Max FPS (from last 2 seconds)
    minFPS = 10000.0f;
    maxFPS = 0.0f;
    for (float ft : frameTimeHistory) {
        if (ft > 0.0001f) {
            float fps = 1.0f / ft;
            minFPS = std::min(minFPS, fps);
            maxFPS = std::max(maxFPS, fps);
        }
    }
}

void PerformanceHUD::render(const Renderer* renderer, const Camera* camera) {
    if (!enabled || !renderer) {
        return;
    }

    // Set window position based on setting
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration |
                            ImGuiWindowFlags_AlwaysAutoResize |
                            ImGuiWindowFlags_NoSavedSettings |
                            ImGuiWindowFlags_NoFocusOnAppearing |
                            ImGuiWindowFlags_NoNav;

    const float PADDING = 10.0f;
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImVec2 work_pos = viewport->WorkPos;
    ImVec2 work_size = viewport->WorkSize;
    ImVec2 window_pos, window_pos_pivot;

    switch (position) {
        case Position::TOP_LEFT:
            window_pos.x = work_pos.x + PADDING;
            window_pos.y = work_pos.y + PADDING;
            window_pos_pivot.x = 0.0f;
            window_pos_pivot.y = 0.0f;
            break;
        case Position::TOP_RIGHT:
            window_pos.x = work_pos.x + work_size.x - PADDING;
            window_pos.y = work_pos.y + PADDING;
            window_pos_pivot.x = 1.0f;
            window_pos_pivot.y = 0.0f;
            break;
        case Position::BOTTOM_LEFT:
            window_pos.x = work_pos.x + PADDING;
            window_pos.y = work_pos.y + work_size.y - PADDING;
            window_pos_pivot.x = 0.0f;
            window_pos_pivot.y = 1.0f;
            break;
        case Position::BOTTOM_RIGHT:
            window_pos.x = work_pos.x + work_size.x - PADDING;
            window_pos.y = work_pos.y + work_size.y - PADDING;
            window_pos_pivot.x = 1.0f;
            window_pos_pivot.y = 1.0f;
            break;
    }

    ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
    ImGui::SetNextWindowBgAlpha(0.7f);  // Transparent background

    if (!ImGui::Begin("Performance", nullptr, flags)) {
        ImGui::End();
        return;
    }

    // FPS section
    if (showFPS) {
        ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "PERFORMANCE");
        ImGui::Separator();

        // Color-code FPS
        ImVec4 fpsColor;
        if (currentFPS >= 60.0f) {
            fpsColor = ImVec4(0.0f, 1.0f, 0.0f, 1.0f);  // Green
        } else if (currentFPS >= 30.0f) {
            fpsColor = ImVec4(1.0f, 1.0f, 0.0f, 1.0f);  // Yellow
        } else {
            fpsColor = ImVec4(1.0f, 0.0f, 0.0f, 1.0f);  // Red
        }

        ImGui::Text("FPS: ");
        ImGui::SameLine();
        ImGui::TextColored(fpsColor, "%.1f", currentFPS);

        ImGui::Text("Avg: %.1f", averageFPS);
        ImGui::Text("Min: %.1f", minFPS);
        ImGui::Text("Max: %.1f", maxFPS);
        ImGui::Text("Frame: %.2f ms", frameTime * 1000.0f);

        ImGui::Spacing();
        // What the GPU did with the frame, against the wall clock the frame
        // was given. Busy is the number the pass times alone do not say: the
        // bars below are how long each pass took, the top one is how much of
        // the frame the GPU spent working - the gap being either the CPU
        // keeping it waiting or the frame finishing early.
        ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 1.0f), "GPU (ms)");
        ImGui::Separator();
        {
            auto* ctx = renderer->getVkContext();
            const double wallMs = static_cast<double>(frameTime) * 1000.0;
            if (ctx && !ctx->gpuTimings().empty() && wallMs > 0.0) {
                const auto& passes = ctx->gpuTimings();
                double total = 0.0;
                for (const auto& p : passes) total += p.second;
                char head[96];
                std::snprintf(head, sizeof(head), "%.1f of %.1f ms (%.0f%% busy)",
                              total, wallMs, 100.0 * total / wallMs);
                ImGui::ProgressBar(static_cast<float>(std::min(1.0, total / wallMs)),
                                   ImVec2(210.0f, 0.0f), head);
                for (const auto& p : passes) {
                    char line[96];
                    std::snprintf(line, sizeof(line), "%s  %.2f", p.first, p.second);
                    ImGui::ProgressBar(
                        static_cast<float>(std::min(1.0, p.second / wallMs)),
                        ImVec2(210.0f, 0.0f), line);
                }
            } else {
                ImGui::TextUnformatted("no pass timings yet");
            }
        }

        // The device's own word for how it is doing with heat. A window that
        // drops to half rate because the SoC is warm and one that drops
        // because the CPU stalled look the same from inside the game; this is
        // the difference, in the OS's own terms - the same state Instruments'
        // energy log reads.
        {
            const char* thermal = core::thermalStateName();
            ImVec4 thermalColor = colors::kGray;
            if (std::strcmp(thermal, "nominal") == 0) {
                thermalColor = colors::kGreen;
            } else if (std::strcmp(thermal, "fair") == 0) {
                thermalColor = ImVec4(1.0f, 1.0f, 0.0f, 1.0f);
            } else if (std::strcmp(thermal, "serious") == 0 ||
                       std::strcmp(thermal, "critical") == 0) {
                thermalColor = ImVec4(1.0f, 0.3f, 0.3f, 1.0f);
            }
            ImGui::TextColored(thermalColor, "Thermal: %s", thermal);
        }

        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.9f, 0.8f, 0.6f, 1.0f), "CPU TIMINGS (ms)");
        ImGui::Text("Update: %.2f (Camera: %.2f)", renderer->getLastUpdateMs(), renderer->getLastCameraUpdateMs());
        ImGui::Text("Render: %.2f (Terrain: %.2f, WMO: %.2f, M2: %.2f)",
                    renderer->getLastRenderMs(),
                    renderer->getLastTerrainRenderMs(),
                    renderer->getLastWMORenderMs(),
                    renderer->getLastM2RenderMs());
        auto* wmoRenderer = renderer->getWMORenderer();
        auto* m2Renderer = renderer->getM2Renderer();
        if (wmoRenderer || m2Renderer) {
            ImGui::Text("Collision queries:");
            if (wmoRenderer) {
                ImGui::Text("  WMO: %.2f ms (%u calls)",
                            wmoRenderer->getQueryTimeMs(), wmoRenderer->getQueryCallCount());
            }
            if (m2Renderer) {
                ImGui::Text("  M2:  %.2f ms (%u calls)",
                            m2Renderer->getQueryTimeMs(), m2Renderer->getQueryCallCount());
            }
        }
        // The worst of the three prepare workers, which is what a stuttering
        // frame is usually waiting on: the main thread blocks on the slowest
        // one before the draw lists go out. Baseline is around 1-2 ms; the
        // streaming stalls this client chases show up here first.
        ImGui::Text("  worst prepare worker: %.2f ms",
                    std::max({renderer->getLastTerrainRenderMs(),
                              renderer->getLastWMORenderMs(),
                              renderer->getLastM2RenderMs()}));

        // Where the main thread's frame went, over the same ten-second window
        // the log's breakdown prints from - so an on-screen stutter can be
        // matched to a line in the log without a profiler attached.
        {
            auto stages = core::Application::getInstance().stageStatsSnapshot();
            std::vector<std::pair<std::string, const core::Application::StageStat*>> byAvg;
            for (auto& entry : stages) {
                if (entry.second.frames > 0) byAvg.emplace_back(entry.first, &entry.second);
            }
            std::sort(byAvg.begin(), byAvg.end(), [](const auto& a, const auto& b) {
                return a.second->totalMs / a.second->frames >
                       b.second->totalMs / b.second->frames;
            });
            if (!byAvg.empty()) {
                ImGui::TextColored(ImVec4(0.9f, 0.8f, 0.6f, 1.0f),
                                   "CPU STAGES (avg / worst ms)");
                const size_t show = std::min<size_t>(byAvg.size(), 8);
                for (size_t i = 0; i < show; ++i) {
                    ImGui::Text("  %-22s %6.2f / %6.2f", byAvg[i].first.c_str(),
                                byAvg[i].second->totalMs / byAvg[i].second->frames,
                                byAvg[i].second->worstMs);
                }
            }
        }

        // Frame time graph
        if (!frameTimeHistory.empty()) {
            std::vector<float> frameTimesMs;
            frameTimesMs.reserve(frameTimeHistory.size());
            for (float ft : frameTimeHistory) {
                frameTimesMs.push_back(ft * 1000.0f);  // Convert to ms
            }
            ImGui::PlotLines("##frametime", frameTimesMs.data(), static_cast<int>(frameTimesMs.size()),
                           0, nullptr, 0.0f, 33.33f, ImVec2(200, 40));
        }

        // FSR info
        if (renderer->getPostProcessPipeline()->isFSREnabled()) {
            ImGui::TextColored(colors::kGreen, "FSR 1.0: ON");
            auto* ctx = renderer->getVkContext();
            if (ctx) {
                auto ext = ctx->getSwapchainExtent();
                float sf = renderer->getPostProcessPipeline()->getFSRScaleFactor();
                uint32_t iw = static_cast<uint32_t>(ext.width * sf) & ~1u;
                uint32_t ih = static_cast<uint32_t>(ext.height * sf) & ~1u;
                ImGui::Text("  %ux%u -> %ux%u (%.0f%%)", iw, ih, ext.width, ext.height, sf * 100.0f);
            }
        }
        if (renderer->getPostProcessPipeline()->isFSR2Enabled()) {
            ImGui::TextColored(ImVec4(0.4f, 0.9f, 1.0f, 1.0f), "FSR 3 Upscale: ON");
            ImGui::Text("  JitterSign=%.2f", renderer->getPostProcessPipeline()->getFSR2JitterSign());
            const bool fgEnabled = renderer->getPostProcessPipeline()->isAmdFsr3FramegenEnabled();
            const bool fgReady = renderer->getPostProcessPipeline()->isAmdFsr3FramegenRuntimeReady();
            const bool fgActive = renderer->getPostProcessPipeline()->isAmdFsr3FramegenRuntimeActive();
            const char* fgStatus = "Disabled";
            if (fgEnabled) {
                fgStatus = fgActive ? "Active" : (fgReady ? "Ready (waiting/fallback)" : "Unavailable");
            }
            ImGui::Text("  FSR3 FG: %s (%s)", fgStatus, renderer->getPostProcessPipeline()->getAmdFsr3FramegenRuntimePath());
            const std::string& fgErr = renderer->getPostProcessPipeline()->getAmdFsr3FramegenRuntimeError();
            if (!fgErr.empty()) {
                ImGui::TextWrapped("  FG Last Error: %s", fgErr.c_str());
            }
            ImGui::Text("  FG Dispatches: %zu", renderer->getPostProcessPipeline()->getAmdFsr3FramegenDispatchCount());
            ImGui::Text("  Upscale Dispatches: %zu", renderer->getPostProcessPipeline()->getAmdFsr3UpscaleDispatchCount());
            ImGui::Text("  FG Fallbacks: %zu", renderer->getPostProcessPipeline()->getAmdFsr3FallbackCount());
        }
        if (renderer->getPostProcessPipeline()->isFXAAEnabled()) {
            if (renderer->getPostProcessPipeline()->isFSR2Enabled()) {
                ImGui::TextColored(ImVec4(0.6f, 1.0f, 0.8f, 1.0f), "FXAA: ON (FSR3+FXAA combined)");
            } else {
                ImGui::TextColored(ImVec4(0.8f, 1.0f, 0.6f, 1.0f), "FXAA: ON");
            }
        }

        ImGui::Spacing();
    }

    // Renderer stats
    if (showRenderer) {
        auto* terrainRenderer = renderer->getTerrainRenderer();
        if (terrainRenderer) {
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 1.0f, 1.0f), "RENDERING");
            ImGui::Separator();

            int totalChunks = terrainRenderer->getChunkCount();
            int rendered = terrainRenderer->getRenderedChunkCount();
            int culled = terrainRenderer->getCulledChunkCount();
            int triangles = terrainRenderer->getTriangleCount();

            ImGui::Text("Chunks: %d", totalChunks);
            ImGui::Text("Rendered: %d", rendered);
            ImGui::Text("Culled: %d", culled);

            if (totalChunks > 0) {
                float visiblePercent = (rendered * 100.0f) / totalChunks;
                ImGui::Text("Visible: %.1f%%", visiblePercent);
            }

            ImGui::Text("Triangles: %s",
                       triangles >= 1000000 ?
                       (std::to_string(triangles / 1000) + "K").c_str() :
                       std::to_string(triangles).c_str());

            ImGui::Spacing();
        }
    }

    // Terrain streaming info
    if (showTerrain) {
        auto* terrainManager = renderer->getTerrainManager();
        if (terrainManager) {
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "TERRAIN");
            ImGui::Separator();

            ImGui::Text("Loaded tiles: %d", terrainManager->getLoadedTileCount());

            auto currentTile = terrainManager->getCurrentTile();
            ImGui::Text("Current tile: [%d,%d]", currentTile.x, currentTile.y);

            ImGui::Spacing();
        }

        // Water info
        auto* waterRenderer = renderer->getWaterRenderer();
        if (waterRenderer) {
            ImGui::TextColored(ImVec4(0.2f, 0.5f, 1.0f, 1.0f), "WATER");
            ImGui::Separator();

            ImGui::Text("Surfaces: %d", waterRenderer->getSurfaceCount());
            ImGui::Text("Enabled: %s", waterRenderer->isEnabled() ? "YES" : "NO");

            ImGui::Spacing();
        }
    }

    // Skybox info
    if (showTerrain) {
        auto* skybox = renderer->getSkybox();
        if (skybox) {
            ImGui::TextColored(ImVec4(0.5f, 0.8f, 1.0f, 1.0f), "SKY");
            ImGui::Separator();

            float time = skybox->getTimeOfDay();
            int hours = static_cast<int>(time);
            int minutes = static_cast<int>((time - hours) * 60);

            ImGui::Text("Time: %02d:%02d", hours, minutes);
            ImGui::Text("Auto: %s", skybox->isTimeProgressionEnabled() ? "YES" : "NO");

            // Celestial info
            auto* celestial = renderer->getCelestial();
            if (celestial) {
                ImGui::Text("Sun/Moon: %s", celestial->isEnabled() ? "YES" : "NO");

                // Moon phase info
                float phase = celestial->getMoonPhase();
                const char* phaseName = "Unknown";
                if (phase < 0.0625f || phase >= 0.9375f) phaseName = "New";
                else if (phase < 0.1875f) phaseName = "Wax Cresc";
                else if (phase < 0.3125f) phaseName = "1st Qtr";
                else if (phase < 0.4375f) phaseName = "Wax Gibb";
                else if (phase < 0.5625f) phaseName = "Full";
                else if (phase < 0.6875f) phaseName = "Wan Gibb";
                else if (phase < 0.8125f) phaseName = "Last Qtr";
                else phaseName = "Wan Cresc";

                ImGui::Text("Moon: %s (%.0f%%)", phaseName, phase * 100.0f);
                ImGui::Text("Cycling: %s", celestial->isMoonPhaseCycling() ? "YES" : "NO");
            }

            // Star field info
            auto* starField = renderer->getStarField();
            if (starField) {
                ImGui::Text("Stars: %d (%s)", starField->getStarCount(),
                           starField->isEnabled() ? "ON" : "OFF");
            }

            // Cloud info
            auto* clouds = renderer->getClouds();
            if (clouds) {
                ImGui::Text("Clouds: %s (%.0f%%)",
                           clouds->isEnabled() ? "ON" : "OFF",
                           clouds->getDensity() * 100.0f);
            }

            // Lens flare info
            auto* lensFlare = renderer->getLensFlare();
            if (lensFlare) {
                ImGui::Text("Lens Flare: %s (%.0f%%)",
                           lensFlare->isEnabled() ? "ON" : "OFF",
                           lensFlare->getIntensity() * 100.0f);
            }

            ImGui::Spacing();
        }
    }

    // Weather info
    if (showRenderer) {
        auto* weather = renderer->getWeather();
        if (weather) {
            ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 1.0f), "WEATHER");
            ImGui::Separator();

            const char* typeName = "None";
            using WeatherType = rendering::Weather::Type;
            auto type = weather->getWeatherType();
            if (type == WeatherType::RAIN) typeName = "Rain";
            else if (type == WeatherType::SNOW) typeName = "Snow";

            ImGui::Text("Type: %s", typeName);
            if (weather->isEnabled()) {
                ImGui::Text("Particles: %d", weather->getParticleCount());
                ImGui::Text("Intensity: %.0f%%", weather->getIntensity() * 100.0f);
            }

            auto* lightning = renderer->getLightning();
            if (lightning && lightning->isEnabled()) {
                ImGui::Text("Lightning: active (%.0f%%)", lightning->getIntensity() * 100.0f);
            }

            ImGui::Spacing();
        }
    }

    // Fog info
    if (showRenderer) {
        auto* terrainRenderer = renderer->getTerrainRenderer();
        if (terrainRenderer) {
            ImGui::TextColored(ImVec4(0.7f, 0.8f, 0.9f, 1.0f), "FOG");
            ImGui::Separator();

            ImGui::Text("Distance fog: %s", terrainRenderer->isFogEnabled() ? "ON" : "OFF");

            ImGui::Spacing();
        }
    }

    // Character info
    if (showRenderer) {
        auto* charRenderer = renderer->getCharacterRenderer();
        if (charRenderer) {
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.4f, 1.0f), "CHARACTERS");
            ImGui::Separator();

            ImGui::Text("Instances: %zu", charRenderer->getInstanceCount());

            ImGui::Spacing();
        }
    }

    // WMO building info
    if (showRenderer) {
        auto* wmoRenderer = renderer->getWMORenderer();
        if (wmoRenderer) {
            ImGui::TextColored(ImVec4(0.8f, 0.7f, 0.6f, 1.0f), "WMO BUILDINGS");
            ImGui::Separator();

            ImGui::Text("Models: %u", wmoRenderer->getModelCount());
            ImGui::Text("Instances: %u", wmoRenderer->getInstanceCount());
            ImGui::Text("Triangles: %u", wmoRenderer->getTotalTriangleCount());
            ImGui::Text("Draw Calls: %u", wmoRenderer->getDrawCallCount());
            ImGui::Text("Floor Cache: %zu", wmoRenderer->getFloorCacheSize());
            ImGui::Text("Dist Culled: %u groups", wmoRenderer->getDistanceCulledGroups());
            if (wmoRenderer->isOcclusionCullingEnabled()) {
                ImGui::Text("Occl Culled: %u groups", wmoRenderer->getOcclusionCulledGroups());
            }
            if (wmoRenderer->isPortalCullingEnabled()) {
                ImGui::Text("Portal Culled: %u groups", wmoRenderer->getPortalCulledGroups());
            }

            ImGui::Spacing();
        }
    }

    // Zone info
    {
        const std::string& zoneName = renderer->getCurrentZoneName();
        if (!zoneName.empty()) {
            ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.3f, 1.0f), "ZONE");
            ImGui::Separator();
            ImGui::Text("%s", zoneName.c_str());
            ImGui::Spacing();
        }
    }

    // Camera info
    if (showCamera && camera) {
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "CAMERA");
        ImGui::Separator();

        glm::vec3 pos = camera->getPosition();
        ImGui::Text("Pos: %.1f, %.1f, %.1f", pos.x, pos.y, pos.z);

        glm::vec3 forward = camera->getForward();
        ImGui::Text("Dir: %.2f, %.2f, %.2f", forward.x, forward.y, forward.z);

        ImGui::Spacing();
    }

    // Controls help
    if (showControls) {
        ImGui::TextColored(kTitle, "CONTROLS");
        ImGui::Separator();

        ImGui::TextColored(kSectionHeader, "Movement");
        ImGui::TextColored(kHelpText, "WASD: Move/Strafe");
        ImGui::TextColored(kHelpText, "Q/E: Strafe left/right");
        ImGui::TextColored(kHelpText, "Space: Jump");
        ImGui::TextColored(kHelpText, "X: Sit/Stand");
        ImGui::TextColored(kHelpText, "~: Auto-run");
        ImGui::TextColored(kHelpText, "Z: Sheathe weapons");

        ImGui::Spacing();
        ImGui::TextColored(kSectionHeader, "UI Panels");
        ImGui::TextColored(kHelpText, "B: Bags/Inventory");
        ImGui::TextColored(kHelpText, "C: Character sheet");
        ImGui::TextColored(kHelpText, "L: Quest log");
        ImGui::TextColored(kHelpText, "N: Talents");
        ImGui::TextColored(kHelpText, "P: Spellbook");
        ImGui::TextColored(kHelpText, "M: World map");

        ImGui::Spacing();
        ImGui::TextColored(kSectionHeader, "Combat & Chat");
        ImGui::TextColored(kHelpText, "1-0,-,=: Action bar");
        ImGui::TextColored(kHelpText, "Tab: Target cycle");
        ImGui::TextColored(kHelpText, "Enter: Chat");
        ImGui::TextColored(kHelpText, "/: Chat command");

        ImGui::Spacing();
        ImGui::TextColored(kSectionHeader, "Debug");
        ImGui::TextColored(kHelpText, "F1: Toggle this HUD (debug builds)");
        ImGui::TextColored(kHelpText, "Menu: Interface - Performance overlay");
        ImGui::TextColored(kHelpText, "F4: Toggle shadows");
        ImGui::TextColored(kHelpText, "F7: Level-up FX");
        ImGui::TextColored(kHelpText, "Esc: Settings/Close");
    }

    ImGui::End();
}

} // namespace rendering
} // namespace wowee
