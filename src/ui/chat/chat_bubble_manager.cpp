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
#include "ui/sight_cache.hpp"

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

        // Not through a wall. A bubble carries a name as well as a line, so
        // one drawn over the building someone is standing behind says they are
        // in front of it - the same fault the nameplates had, answered by the
        // same query.
        static SightCache sight;
        if (sight.blocked(renderer->getWMORenderer(), bubble.senderGuid,
                          camera->getPosition(), renderPos)) {
            continue;
        }

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

        // Into the background draw list, which is where the interface is
        // drawn - and drawn into before it, so the interface covers this.
        //
        // These were ImGui windows, and an ImGui window is above every draw
        // list the interface uses, whatever the interface is doing: a bubble
        // sat on top of the auction house, the bags, the map, anything open.
        // Nothing about the position was wrong; a window was simply the one
        // thing that cannot go underneath.
        ImDrawList* dl = ImGui::GetBackgroundDrawList();
        ImFont* font = ImGui::GetFont();
        const float fontSize = ImGui::GetFontSize();
        constexpr float kWrapWidth = 200.0f;
        const ImVec2 padding(8.0f, 4.0f);

        const ImVec2 textSize =
            font->CalcTextSizeA(fontSize, FLT_MAX, kWrapWidth, bubble.message.c_str());
        const ImVec2 size(textSize.x + padding.x * 2.0f, textSize.y + padding.y * 2.0f);
        // The point is where the speaker's head is, and the bubble sits above
        // it, centred - which is what the window's pivot of (0.5, 1) did.
        const ImVec2 topLeft(screenX - size.x * 0.5f, screenY - size.y);

        ImVec4 background = ImGui::GetStyleColorVec4(ImGuiCol_WindowBg);
        background.w = 0.7f * alpha;
        dl->AddRectFilled(topLeft, ImVec2(topLeft.x + size.x, topLeft.y + size.y),
                          ImGui::GetColorU32(background), 8.0f);

        const ImVec4 textColor = bubble.isYell
            ? ImVec4(1.0f, 0.2f, 0.2f, alpha)
            : ImVec4(1.0f, 1.0f, 1.0f, alpha);
        dl->AddText(font, fontSize, ImVec2(topLeft.x + padding.x, topLeft.y + padding.y),
                    ImGui::GetColorU32(textColor), bubble.message.c_str(), nullptr,
                    kWrapWidth);
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
