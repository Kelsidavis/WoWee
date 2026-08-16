// ChatBubbleManager - 3D-projected chat bubbles above entities.
// Moved from ChatPanel::renderBubbles / setupCallbacks (Phase 1.4).
#include "ui/chat/chat_bubble_manager.hpp"
#include "game/game_handler.hpp"
#include "rendering/renderer.hpp"
#include "rendering/camera.hpp"
#include "core/coordinates.hpp"
#include "core/window.hpp"
#include <imgui.h>
#include "ui/chat/chat_utils.hpp"

#include <glm/glm.hpp>

namespace wowee { namespace ui {

void ChatBubbleManager::addBubble(uint64_t senderGuid, const std::string& message, bool isYell) {
    float duration = 8.0f + static_cast<float>(message.size()) * 0.06f;
    if (isYell) duration += 2.0f;
    if (duration > 15.0f) duration = 15.0f;

    // Replace existing bubble for same sender
    for (auto& b : bubbles_) {
        if (b.senderGuid == senderGuid) {
            b.message = message;
            b.timeRemaining = duration;
            b.totalDuration = duration;
            b.isYell = isYell;
            return;
        }
    }
    // Evict oldest if too many
    if (bubbles_.size() >= kMaxBubbles) {
        bubbles_.erase(bubbles_.begin());
    }
    bubbles_.push_back({.senderGuid = senderGuid, .message = message, .timeRemaining = duration, .totalDuration = duration, .isYell = isYell});
}

void ChatBubbleManager::render(game::GameHandler& gameHandler, const UIServices& services) {
    // Off means off: the queue is dropped as well, so turning the setting off
    // clears what is on screen rather than leaving it to fade for five seconds.
    if (!show_) { bubbles_.clear(); return; }
    if (bubbles_.empty()) return;

    auto* renderer = services.renderer;
    auto* camera = renderer ? renderer->getCamera() : nullptr;
    if (!camera) return;

    auto* window = services.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    // Get delta time from ImGui
    float dt = ImGui::GetIO().DeltaTime;

    glm::mat4 viewProj = camera->getProjectionMatrix() * camera->getViewMatrix();

    // Update and render bubbles
    for (int i = static_cast<int>(bubbles_.size()) - 1; i >= 0; --i) {
        auto& bubble = bubbles_[i];
        bubble.timeRemaining -= dt;
        if (bubble.timeRemaining <= 0.0f) {
            bubbles_.erase(bubbles_.begin() + i);
            continue;
        }

        // Get entity position
        auto entity = gameHandler.getEntityManager().getEntity(bubble.senderGuid);
        if (!entity) continue;

        // Convert canonical → render coordinates, offset up by 2.5 units for bubble above head
        glm::vec3 canonical(entity->getX(), entity->getY(), entity->getZ() + 2.5f);
        glm::vec3 renderPos = core::coords::canonicalToRender(canonical);

        // Project to screen
        glm::vec4 clipPos = viewProj * glm::vec4(renderPos, 1.0f);
        if (clipPos.w <= 0.0f) continue;  // Behind camera

        glm::vec2 ndc(clipPos.x / clipPos.w, clipPos.y / clipPos.w);
        float screenX = (ndc.x * 0.5f + 0.5f) * screenW;
        // Camera bakes the Vulkan Y-flip into the projection matrix:
        // NDC y=-1 is top, y=1 is bottom - same convention as nameplate/minimap projection.
        float screenY = (ndc.y * 0.5f + 0.5f) * screenH;

        // Skip if off-screen
        if (screenX < -200.0f || screenX > screenW + 200.0f ||
            screenY < -100.0f || screenY > screenH + 100.0f) continue;

        // Fade alpha over last 2 seconds
        float alpha = 1.0f;
        if (bubble.timeRemaining < 2.0f) {
            alpha = bubble.timeRemaining / 2.0f;
        }

        // Draw bubble window - format the window ID into a stack buffer
        // so we don't allocate a fresh std::string per bubble per frame.
        char winId[40];
        std::snprintf(winId, sizeof(winId), "##ChatBubble%llu",
                      static_cast<unsigned long long>(bubble.senderGuid));
        ImGui::SetNextWindowPos(ImVec2(screenX, screenY), ImGuiCond_Always, ImVec2(0.5f, 1.0f));
        ImGui::SetNextWindowBgAlpha(0.7f * alpha);
        ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
            ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoInputs |
            ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav;

        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 8.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8, 4));

        ImGui::Begin(winId, nullptr, flags);

        ImVec4 textColor = bubble.isYell
            ? ImVec4(1.0f, 0.2f, 0.2f, alpha)
            : ImVec4(1.0f, 1.0f, 1.0f, alpha);

        ImGui::PushStyleColor(ImGuiCol_Text, textColor);
        ImGui::PushTextWrapPos(200.0f);
        ImGui::TextWrapped("%s", bubble.message.c_str());
        ImGui::PopTextWrapPos();
        ImGui::PopStyleColor();

        ImGui::End();
        ImGui::PopStyleVar(2);
    }
}

void ChatBubbleManager::setupCallback(game::GameHandler& gameHandler) {
    if (!callbackSet_) {
        // Resolve $-tokens here rather than at draw time: a bubble is built once
        // and drawn every frame. The chat panel already does this for the same
        // text, which is why an NPC line read correctly in the log and showed
        // its raw $r/$c/$g markers in the bubble over their head.
        gameHandler.setChatBubbleCallback(
            [this, &gameHandler](uint64_t guid, const std::string& msg, bool isYell) {
                addBubble(guid, chat_utils::replaceGenderPlaceholders(msg, gameHandler), isYell);
            });
        callbackSet_ = true;
    }
}

} // namespace ui
} // namespace wowee
