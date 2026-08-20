#include "game/reputation_standing.hpp"
#include "ui/toast_manager.hpp"
#include "game/game_handler.hpp"
#include "core/application.hpp"
#include "rendering/renderer.hpp"
#include "rendering/animation_controller.hpp"
#include "audio/audio_coordinator.hpp"
#include "audio/ui_sound_manager.hpp"
#include "ui/framexml_takeover.hpp"

#include <imgui.h>
#include <algorithm>
#include <cstdio>
#include <cmath>

namespace wowee { namespace ui {

// ---------------------------------------------------------------------------
// Setup toast callbacks on GameHandler (idempotent - safe to call every frame)
// ---------------------------------------------------------------------------
void ToastManager::setupCallbacks(game::GameHandler& gameHandler) {
    // NOTE: Level-up and achievement callbacks are registered by Application
    // (which also triggers the 3D level-up effect).  Application routes to
    // triggerDing() / triggerAchievementToast() via the public API.

    // Area discovery toast callback
    if (!areaDiscoveryCallbackSet_) {
        gameHandler.setAreaDiscoveryCallback([this](const std::string& areaName, uint32_t xpGained) {
            discoveryToastName_ = areaName.empty() ? "New Area" : areaName;
            discoveryToastXP_   = xpGained;
            discoveryToastTimer_ = DISCOVERY_TOAST_DURATION;
        });
        areaDiscoveryCallbackSet_ = true;
    }

    // Quest objective progress toast callback
    if (!questProgressCallbackSet_) {
        gameHandler.setQuestProgressCallback([this](const std::string& questTitle,
                                                    const std::string& objectiveName,
                                                    uint32_t current, uint32_t required) {
            for (auto& t : questToasts_) {
                if (t.questTitle == questTitle && t.objectiveName == objectiveName) {
                    t.current  = current;
                    t.required = required;
                    t.age      = 0.0f;
                    return;
                }
            }
            if (questToasts_.size() >= 4) questToasts_.erase(questToasts_.begin());
            questToasts_.push_back({.questTitle = questTitle, .objectiveName = objectiveName, .current = current, .required = required, .age = 0.0f});
        });
        questProgressCallbackSet_ = true;
    }

    // Other-player level-up toast callback
    if (!otherPlayerLevelUpCallbackSet_) {
        gameHandler.setOtherPlayerLevelUpCallback([this](uint64_t guid, uint32_t newLevel) {
            for (auto& t : playerLevelUpToasts_) {
                if (t.guid == guid) {
                    t.newLevel = newLevel;
                    t.age = 0.0f;
                    return;
                }
            }
            if (playerLevelUpToasts_.size() >= 3)
                playerLevelUpToasts_.erase(playerLevelUpToasts_.begin());
            playerLevelUpToasts_.push_back({.guid = guid, .playerName = "", .newLevel = newLevel, .age = 0.0f});
        });
        otherPlayerLevelUpCallbackSet_ = true;
    }

    // PvP honor credit toast callback
    if (!pvpHonorCallbackSet_) {
        gameHandler.setPvpHonorCallback([this](uint32_t honor, uint64_t /*victimGuid*/, uint32_t rank) {
            if (honor == 0) return;
            pvpHonorToasts_.push_back({.honor = honor, .victimRank = rank, .age = 0.0f});
            if (pvpHonorToasts_.size() > 4)
                pvpHonorToasts_.erase(pvpHonorToasts_.begin());
        });
        pvpHonorCallbackSet_ = true;
    }

    // Item loot toast callback
    if (!itemLootCallbackSet_) {
        gameHandler.setItemLootCallback([this](uint32_t itemId, uint32_t count,
                                               uint32_t quality, const std::string& name) {
            for (auto& t : itemLootToasts_) {
                if (t.itemId == itemId) {
                    t.count += count;
                    t.age = 0.0f;
                    return;
                }
            }
            if (itemLootToasts_.size() >= 5)
                itemLootToasts_.erase(itemLootToasts_.begin());
            itemLootToasts_.push_back({.itemId = itemId, .count = count, .quality = quality, .name = name, .age = 0.0f});
        });
        itemLootCallbackSet_ = true;
    }

    // Ghost-state callback to flash "You have been resurrected!" on revival
    if (!ghostStateCallbackSet_) {
        gameHandler.setGhostStateCallback([this](bool isGhost) {
            if (!isGhost) {
                resurrectFlashTimer_ = kResurrectFlashDuration;
            }
        });
        ghostStateCallbackSet_ = true;
    }

    // Reputation change toast callback
    if (!repChangeCallbackSet_) {
        gameHandler.setRepChangeCallback([this](const std::string& name, int32_t delta, int32_t standing) {
            repToasts_.push_back({.factionName = name, .delta = delta, .standing = standing, .age = 0.0f});
            if (repToasts_.size() > 4) repToasts_.erase(repToasts_.begin());
        });
        repChangeCallbackSet_ = true;
    }

    // Quest completion toast callback
    if (!questCompleteCallbackSet_) {
        gameHandler.setQuestCompleteCallback([this](uint32_t id, const std::string& title) {
            questCompleteToasts_.push_back({.questId = id, .title = title, .age = 0.0f});
            if (questCompleteToasts_.size() > 3) questCompleteToasts_.erase(questCompleteToasts_.begin());
        });
        questCompleteCallbackSet_ = true;
    }
}

// ---------------------------------------------------------------------------
// Render early toasts (before action bars)
// ---------------------------------------------------------------------------
void ToastManager::renderEarlyToasts(float deltaTime, game::GameHandler& gameHandler) {
    // Zone entry detection - fire a toast when the renderer's zone name changes
    if (auto* rend = services_.renderer) {
        const std::string& curZone = rend->getCurrentZoneName();
        if (!curZone.empty() && curZone != lastKnownZone_) {
            if (!lastKnownZone_.empty()) {
                zoneToasts_.push_back({.zoneName = curZone, .age = 0.0f});
                if (zoneToasts_.size() > 3)
                    zoneToasts_.erase(zoneToasts_.begin());
            }
            lastKnownZone_ = curZone;
        }
    }

    renderRepToasts(deltaTime);
    renderQuestCompleteToasts(deltaTime);
    renderZoneToasts(deltaTime);
    renderAreaTriggerToasts(deltaTime, gameHandler);
}

// ---------------------------------------------------------------------------
// Render late toasts (after escape menu / settings)
// ---------------------------------------------------------------------------
void ToastManager::renderLateToasts(game::GameHandler& gameHandler) {
    renderDingEffect();
    // FrameXML's AlertFrame answers ACHIEVEMENT_EARNED, which this client
    // fires, and it is suppressed only while the achievements element is not
    // owned. So with it owned both popups appeared, one over the other, on
    // every achievement - the same shape the zone banner had before its own
    // gate went in a few lines below.
    renderDiscoveryToast();
    renderWhisperToasts();
    renderQuestProgressToasts();
    renderPlayerLevelUpToasts(gameHandler);
    renderPvpHonorToasts();
    renderItemLootToasts();
    renderResurrectFlash();
    renderZoneText(gameHandler);
}

// ============================================================
// Reputation change toasts
// ============================================================

void ToastManager::renderRepToasts(float deltaTime) {
    for (auto& e : repToasts_) e.age += deltaTime;
    repToasts_.erase(
        std::remove_if(repToasts_.begin(), repToasts_.end(),
            [](const RepToastEntry& e) { return e.age >= kRepToastLifetime; }),
        repToasts_.end());

    if (repToasts_.empty()) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    // Stack toasts in the lower-right corner (above the action bar), newest on top
    const float toastW = 220.0f;
    const float toastH = 26.0f;
    const float padY   = 4.0f;
    const float rightEdge = screenW - 14.0f;
    const float baseY = screenH - 180.0f;

    const int count = static_cast<int>(repToasts_.size());

    ImDrawList* draw = ImGui::GetForegroundDrawList();
    ImFont* font = ImGui::GetFont();
    float fontSize = ImGui::GetFontSize();

    // Compute standing tier label (Exalted, Revered, Honored, Friendly, Neutral, Unfriendly, Hostile, Hated)
    // The thresholds live with the standings - see reputation_standing.hpp.
    // This had them written out again as an if-chain, which is the same eight
    // numbers in the opposite order and no easier to check.
    auto standingLabel = [](int32_t value) -> const char* {
        return game::reputationStandingFor(value).name;
    };

    for (int i = 0; i < count; ++i) {
        const auto& e = repToasts_[i];
        // Slide in from right on appear, slide out at end
        constexpr float kSlideDur = 0.3f;
        float slideIn  = std::min(e.age, kSlideDur) / kSlideDur;
        float slideOut = std::min(std::max(0.0f, kRepToastLifetime - e.age), kSlideDur) / kSlideDur;
        float slide    = std::min(slideIn, slideOut);

        float alpha = std::clamp(slide, 0.0f, 1.0f);
        float xFull  = rightEdge - toastW;
        float xStart = screenW + 10.0f;
        float toastX = xStart + (xFull - xStart) * slide;
        float toastY = baseY - i * (toastH + padY);

        ImVec2 tl(toastX, toastY);
        ImVec2 br(toastX + toastW, toastY + toastH);

        // Background
        draw->AddRectFilled(tl, br, IM_COL32(15, 15, 20, static_cast<int>(alpha * 200)), 4.0f);
        // Border: green for gain, red for loss
        ImU32 borderCol = (e.delta > 0)
            ? IM_COL32(80, 200, 80, static_cast<int>(alpha * 220))
            : IM_COL32(200, 60, 60, static_cast<int>(alpha * 220));
        draw->AddRect(tl, br, borderCol, 4.0f, 0, 1.5f);

        // Delta text: "+250" or "-250"
        char deltaBuf[16];
        snprintf(deltaBuf, sizeof(deltaBuf), "%+d", e.delta);
        ImU32 deltaCol = (e.delta > 0) ? IM_COL32(80, 220, 80, static_cast<int>(alpha * 255))
                                       : IM_COL32(220, 70, 70, static_cast<int>(alpha * 255));
        draw->AddText(font, fontSize, ImVec2(tl.x + 6.0f, tl.y + (toastH - fontSize) * 0.5f),
                      deltaCol, deltaBuf);

        // Faction name + standing
        char nameBuf[64];
        snprintf(nameBuf, sizeof(nameBuf), "%s (%s)", e.factionName.c_str(), standingLabel(e.standing));
        draw->AddText(font, fontSize * 0.85f, ImVec2(tl.x + 44.0f, tl.y + (toastH - fontSize * 0.85f) * 0.5f),
                      IM_COL32(210, 210, 210, static_cast<int>(alpha * 220)), nameBuf);
    }
}

void ToastManager::renderQuestCompleteToasts(float deltaTime) {
    for (auto& e : questCompleteToasts_) e.age += deltaTime;
    questCompleteToasts_.erase(
        std::remove_if(questCompleteToasts_.begin(), questCompleteToasts_.end(),
            [](const QuestCompleteToastEntry& e) { return e.age >= kQuestCompleteToastLifetime; }),
        questCompleteToasts_.end());

    if (questCompleteToasts_.empty()) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    const float toastW = 260.0f;
    const float toastH = 40.0f;
    const float padY   = 4.0f;
    const float baseY  = screenH - 220.0f;  // above rep toasts

    ImDrawList* draw = ImGui::GetForegroundDrawList();
    ImFont* font = ImGui::GetFont();
    float fontSize = ImGui::GetFontSize();

    for (int i = 0; i < static_cast<int>(questCompleteToasts_.size()); ++i) {
        const auto& e = questCompleteToasts_[i];
        constexpr float kSlideDur = 0.3f;
        float slideIn  = std::min(e.age, kSlideDur) / kSlideDur;
        float slideOut = std::min(std::max(0.0f, kQuestCompleteToastLifetime - e.age), kSlideDur) / kSlideDur;
        float slide    = std::min(slideIn, slideOut);
        float alpha    = std::clamp(slide, 0.0f, 1.0f);

        float xFull  = screenW - 14.0f - toastW;
        float xStart = screenW + 10.0f;
        float toastX = xStart + (xFull - xStart) * slide;
        float toastY = baseY - i * (toastH + padY);

        ImVec2 tl(toastX, toastY);
        ImVec2 br(toastX + toastW, toastY + toastH);

        // Background + gold border (quest completion)
        draw->AddRectFilled(tl, br, IM_COL32(20, 18, 8, static_cast<int>(alpha * 210)), 5.0f);
        draw->AddRect(tl, br, IM_COL32(220, 180, 30, static_cast<int>(alpha * 230)), 5.0f, 0, 1.5f);

        // Scroll icon placeholder (gold diamond)
        float iconCx = tl.x + 18.0f;
        float iconCy = tl.y + toastH * 0.5f;
        draw->AddCircleFilled(ImVec2(iconCx, iconCy), 7.0f, IM_COL32(210, 170, 20, static_cast<int>(alpha * 230)));
        draw->AddCircle      (ImVec2(iconCx, iconCy), 7.0f, IM_COL32(255, 220, 50, static_cast<int>(alpha * 200)));

        // "Quest Complete" header in gold
        const char* header = "Quest Complete";
        draw->AddText(font, fontSize * 0.78f,
                      ImVec2(tl.x + 34.0f, tl.y + 4.0f),
                      IM_COL32(240, 200, 40, static_cast<int>(alpha * 240)), header);

        // Quest title in off-white
        const char* titleStr = e.title.empty() ? "Unknown Quest" : e.title.c_str();
        draw->AddText(font, fontSize * 0.82f,
                      ImVec2(tl.x + 34.0f, tl.y + toastH * 0.5f + 1.0f),
                      IM_COL32(220, 215, 195, static_cast<int>(alpha * 220)), titleStr);
    }
}

// ============================================================
// Zone Entry Toast
// ============================================================

void ToastManager::renderZoneToasts(float deltaTime) {
    for (auto& e : zoneToasts_) e.age += deltaTime;
    zoneToasts_.erase(
        std::remove_if(zoneToasts_.begin(), zoneToasts_.end(),
            [](const ZoneToastEntry& e) { return e.age >= kZoneToastLifetime; }),
        zoneToasts_.end());

    // Suppress toasts while the zone text overlay is showing the same zone -
    // avoids duplicate "Entering: Stormwind City" messages.
    if (zoneTextTimer_ > 0.0f) {
        zoneToasts_.erase(
            std::remove_if(zoneToasts_.begin(), zoneToasts_.end(),
                [this](const ZoneToastEntry& e) { return e.zoneName == zoneTextName_; }),
            zoneToasts_.end());
    }

    if (zoneToasts_.empty()) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    ImDrawList* draw = ImGui::GetForegroundDrawList();
    ImFont* font = ImGui::GetFont();

    for (int i = 0; i < static_cast<int>(zoneToasts_.size()); ++i) {
        const auto& e = zoneToasts_[i];
        constexpr float kSlideDur = 0.35f;
        float slideIn  = std::min(e.age, kSlideDur) / kSlideDur;
        float slideOut = std::min(std::max(0.0f, kZoneToastLifetime - e.age), kSlideDur) / kSlideDur;
        float slide    = std::min(slideIn, slideOut);
        float alpha    = std::clamp(slide, 0.0f, 1.0f);

        // Measure text to size the toast
        ImVec2 nameSz = font->CalcTextSizeA(14.0f, FLT_MAX, 0.0f, e.zoneName.c_str());
        const char* header = "Entering:";
        ImVec2 hdrSz = font->CalcTextSizeA(11.0f, FLT_MAX, 0.0f, header);

        float toastW = std::max(nameSz.x, hdrSz.x) + 28.0f;
        float toastH = 42.0f;

        // Center the toast horizontally, appear just below the zone name area (top-center)
        float toastX = (screenW - toastW) * 0.5f;
        float toastY = 56.0f + i * (toastH + 4.0f);
        // Slide down from above
        float offY = (1.0f - slide) * (-toastH - 10.0f);
        toastY += offY;

        ImVec2 tl(toastX, toastY);
        ImVec2 br(toastX + toastW, toastY + toastH);

        draw->AddRectFilled(tl, br, IM_COL32(10, 10, 16, static_cast<int>(alpha * 200)), 6.0f);
        draw->AddRect(tl, br, IM_COL32(160, 140, 80, static_cast<int>(alpha * 220)), 6.0f, 0, 1.2f);

        float cx = tl.x + toastW * 0.5f;
        draw->AddText(font, 11.0f,
            ImVec2(cx - hdrSz.x * 0.5f, tl.y + 5.0f),
            IM_COL32(180, 170, 120, static_cast<int>(alpha * 200)), header);
        draw->AddText(font, 14.0f,
            ImVec2(cx - nameSz.x * 0.5f, tl.y + toastH * 0.5f + 1.0f),
            IM_COL32(255, 230, 140, static_cast<int>(alpha * 240)), e.zoneName.c_str());
    }
}

// ─── Area Trigger Message Toasts ─────────────────────────────────────────────
void ToastManager::renderAreaTriggerToasts(float deltaTime, game::GameHandler& gameHandler) {
    // Drain any pending messages from GameHandler
    while (gameHandler.hasAreaTriggerMsg()) {
        AreaTriggerToast t;
        t.text = gameHandler.popAreaTriggerMsg();
        t.age  = 0.0f;
        areaTriggerToasts_.push_back(std::move(t));
        if (areaTriggerToasts_.size() > 4)
            areaTriggerToasts_.erase(areaTriggerToasts_.begin());
    }

    // Age and prune
    constexpr float kLifetime = 4.5f;
    for (auto& t : areaTriggerToasts_) t.age += deltaTime;
    areaTriggerToasts_.erase(
        std::remove_if(areaTriggerToasts_.begin(), areaTriggerToasts_.end(),
                       [](const AreaTriggerToast& t) { return t.age >= kLifetime; }),
        areaTriggerToasts_.end());
    if (areaTriggerToasts_.empty()) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth())  : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) :  720.0f;

    ImDrawList* draw = ImGui::GetForegroundDrawList();
    ImFont* font = ImGui::GetFont();
    constexpr float kSlideDur = 0.35f;

    for (int i = 0; i < static_cast<int>(areaTriggerToasts_.size()); ++i) {
        const auto& t = areaTriggerToasts_[i];

        float slideIn  = std::min(t.age, kSlideDur) / kSlideDur;
        float slideOut = std::min(std::max(0.0f, kLifetime - t.age), kSlideDur) / kSlideDur;
        float alpha    = std::clamp(std::min(slideIn, slideOut), 0.0f, 1.0f);

        // Measure text
        ImVec2 txtSz = font->CalcTextSizeA(13.0f, FLT_MAX, 0.0f, t.text.c_str());
        float toastW = txtSz.x + 30.0f;
        float toastH = 30.0f;

        // Center horizontally, place below zone text (center of lower-third)
        float toastX = (screenW - toastW) * 0.5f;
        float toastY = screenH * 0.62f + i * (toastH + 3.0f);
        // Slide up from below
        float offY = (1.0f - std::min(slideIn, slideOut)) * (toastH + 12.0f);
        toastY += offY;

        ImVec2 tl(toastX, toastY);
        ImVec2 br(toastX + toastW, toastY + toastH);

        draw->AddRectFilled(tl, br, IM_COL32(8, 12, 22, static_cast<int>(alpha * 190)), 5.0f);
        draw->AddRect(tl, br, IM_COL32(100, 160, 220, static_cast<int>(alpha * 200)), 5.0f, 0, 1.0f);

        float cx = tl.x + toastW * 0.5f;
        // Shadow
        draw->AddText(font, 13.0f,
            ImVec2(cx - txtSz.x * 0.5f + 1, tl.y + (toastH - txtSz.y) * 0.5f + 1),
            IM_COL32(0, 0, 0, static_cast<int>(alpha * 180)), t.text.c_str());
        // Text in light blue
        draw->AddText(font, 13.0f,
            ImVec2(cx - txtSz.x * 0.5f, tl.y + (toastH - txtSz.y) * 0.5f),
            IM_COL32(180, 220, 255, static_cast<int>(alpha * 240)), t.text.c_str());
    }
}

// ============================================================
// Level-Up Ding Animation
// ============================================================

void ToastManager::triggerDing(uint32_t newLevel, uint32_t hpDelta, uint32_t manaDelta,
                              uint32_t str, uint32_t agi, uint32_t sta,
                              uint32_t intel, uint32_t spi) {
    // Set golden burst overlay state (consumed by GameScreen)
    levelUpFlashAlpha = 1.0f;
    levelUpDisplayLevel = newLevel;

    dingTimer_     = DING_DURATION;
    dingLevel_     = newLevel;
    dingHpDelta_   = hpDelta;
    dingManaDelta_ = manaDelta;
    dingStats_[0]  = str;
    dingStats_[1]  = agi;
    dingStats_[2]  = sta;
    dingStats_[3]  = intel;
    dingStats_[4]  = spi;

    auto* ac = services_.audioCoordinator;
    if (ac) {
        if (auto* sfx = ac->getUiSoundManager()) {
            sfx->playLevelUp();
        }
    }
    if (auto* renderer = services_.renderer) {
        if (auto* ac = renderer->getAnimationController()) ac->playEmote("cheer");
    }
}

void ToastManager::renderDingEffect() {
    if (dingTimer_ <= 0.0f) return;

    float dt = ImGui::GetIO().DeltaTime;
    dingTimer_ -= dt;
    if (dingTimer_ < 0.0f) dingTimer_ = 0.0f;

    // Show "You have reached level X!" for the first 2.5s, fade out over last 0.5s.
    // The 3D visual effect is handled by Renderer::triggerLevelUpEffect (LevelUp.m2).
    constexpr float kFadeTime = 0.5f;
    float alpha = dingTimer_ < kFadeTime ? (dingTimer_ / kFadeTime) : 1.0f;
    if (alpha <= 0.0f) return;

    ImGuiIO& io = ImGui::GetIO();
    float cx = io.DisplaySize.x * 0.5f;
    float cy = io.DisplaySize.y * 0.38f;  // Upper-center, like WoW

    ImDrawList* draw = ImGui::GetForegroundDrawList();
    ImFont* font = ImGui::GetFont();
    float baseSize = ImGui::GetFontSize();
    float fontSize = baseSize * 1.8f;

    char buf[64];
    snprintf(buf, sizeof(buf), "You have reached level %u!", dingLevel_);

    ImVec2 sz = font->CalcTextSizeA(fontSize, FLT_MAX, 0.0f, buf);
    float tx = cx - sz.x * 0.5f;
    float ty = cy - sz.y * 0.5f;

    // Slight black outline for readability
    draw->AddText(font, fontSize, ImVec2(tx + 2, ty + 2),
                  IM_COL32(0, 0, 0, static_cast<int>(alpha * 180)), buf);
    // Gold text
    draw->AddText(font, fontSize, ImVec2(tx, ty),
                  IM_COL32(255, 210, 0, static_cast<int>(alpha * 255)), buf);

    // Stat gains below the main text (shown only if server sent deltas)
    bool hasStatGains = (dingHpDelta_ > 0 || dingManaDelta_ > 0 ||
                         dingStats_[0] || dingStats_[1] || dingStats_[2] ||
                         dingStats_[3] || dingStats_[4]);
    if (hasStatGains) {
        float smallSize = baseSize * 0.95f;
        float yOff = ty + sz.y + 6.0f;

        // Build stat delta string: "+150 HP  +80 Mana  +2 Str  +2 Agi ..."
        static constexpr const char* kStatLabels[] = { "Str", "Agi", "Sta", "Int", "Spi" };
        char statBuf[128];
        int written = 0;
        if (dingHpDelta_ > 0)
            written += snprintf(statBuf + written, sizeof(statBuf) - written,
                                "+%u HP  ", dingHpDelta_);
        if (dingManaDelta_ > 0)
            written += snprintf(statBuf + written, sizeof(statBuf) - written,
                                "+%u Mana  ", dingManaDelta_);
        for (int i = 0; i < 5 && written < static_cast<int>(sizeof(statBuf)) - 1; ++i) {
            if (dingStats_[i] > 0)
                written += snprintf(statBuf + written, sizeof(statBuf) - written,
                                    "+%u %s  ", dingStats_[i], kStatLabels[i]);
        }
        // Trim trailing spaces
        while (written > 0 && statBuf[written - 1] == ' ') --written;
        statBuf[written] = '\0';

        if (written > 0) {
            ImVec2 ssz = font->CalcTextSizeA(smallSize, FLT_MAX, 0.0f, statBuf);
            float stx = cx - ssz.x * 0.5f;
            draw->AddText(font, smallSize, ImVec2(stx + 1, yOff + 1),
                          IM_COL32(0, 0, 0, static_cast<int>(alpha * 160)), statBuf);
            draw->AddText(font, smallSize, ImVec2(stx, yOff),
                          IM_COL32(100, 220, 100, static_cast<int>(alpha * 230)), statBuf);
        }
    }
}

void ToastManager::playAchievementEarned() {
    // alertframes.lua registers ACHIEVEMENT_EARNED and raises the badge, so
    // there is nothing of ours left to put on screen. The sound stays:
    // FrameXML's alert frame plays one only for LFG rewards, and an
    // achievement earned in silence is worse than one announced twice.
    auto* ac = services_.audioCoordinator;
    if (ac) {
        if (auto* sfx = ac->getUiSoundManager()) {
            sfx->playAchievementAlert();
        }
    }
}

// ---------------------------------------------------------------------------
// Area discovery toast - "Discovered: <AreaName>! (+XP XP)" centered on screen
// ---------------------------------------------------------------------------

void ToastManager::renderDiscoveryToast() {
    if (discoveryToastTimer_ <= 0.0f) return;

    float dt = ImGui::GetIO().DeltaTime;
    discoveryToastTimer_ -= dt;
    if (discoveryToastTimer_ < 0.0f) discoveryToastTimer_ = 0.0f;

    // Fade: ramp up in first 0.4s, hold, fade out in last 1.0s
    float alpha;
    if (discoveryToastTimer_ > DISCOVERY_TOAST_DURATION - 0.4f)
        alpha = 1.0f - (discoveryToastTimer_ - (DISCOVERY_TOAST_DURATION - 0.4f)) / 0.4f;
    else if (discoveryToastTimer_ < 1.0f)
        alpha = discoveryToastTimer_;
    else
        alpha = 1.0f;
    alpha = std::clamp(alpha, 0.0f, 1.0f);

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth())  : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) :  720.0f;

    ImFont* font = ImGui::GetFont();
    ImDrawList* draw = ImGui::GetForegroundDrawList();

    const char* header   = "Discovered!";
    float headerSize     = 16.0f;
    float nameSize       = 28.0f;
    float xpSize         = 14.0f;

    ImVec2 headerDim = font->CalcTextSizeA(headerSize, FLT_MAX, 0.0f, header);
    ImVec2 nameDim   = font->CalcTextSizeA(nameSize,   FLT_MAX, 0.0f, discoveryToastName_.c_str());

    char xpBuf[48];
    if (discoveryToastXP_ > 0)
        snprintf(xpBuf, sizeof(xpBuf), "+%u XP", discoveryToastXP_);
    else
        xpBuf[0] = '\0';
    ImVec2 xpDim = font->CalcTextSizeA(xpSize, FLT_MAX, 0.0f, xpBuf);

    // Position slightly below zone text (at 37% down screen)
    float centreY = screenH * 0.37f;
    float headerX = (screenW - headerDim.x) * 0.5f;
    float nameX   = (screenW - nameDim.x)   * 0.5f;
    float xpX     = (screenW - xpDim.x)     * 0.5f;
    float headerY = centreY;
    float nameY   = centreY + headerDim.y + 4.0f;
    float xpY     = nameY + nameDim.y + 4.0f;

    // "Discovered!" in gold
    draw->AddText(font, headerSize, ImVec2(headerX + 1, headerY + 1),
                  IM_COL32(0, 0, 0, static_cast<int>(alpha * 160)), header);
    draw->AddText(font, headerSize, ImVec2(headerX, headerY),
                  IM_COL32(255, 215, 0, static_cast<int>(alpha * 255)), header);

    // Area name in white
    draw->AddText(font, nameSize, ImVec2(nameX + 1, nameY + 1),
                  IM_COL32(0, 0, 0, static_cast<int>(alpha * 160)), discoveryToastName_.c_str());
    draw->AddText(font, nameSize, ImVec2(nameX, nameY),
                  IM_COL32(255, 255, 255, static_cast<int>(alpha * 255)), discoveryToastName_.c_str());

    // XP gain in light green (if any)
    if (xpBuf[0] != '\0') {
        draw->AddText(font, xpSize, ImVec2(xpX + 1, xpY + 1),
                      IM_COL32(0, 0, 0, static_cast<int>(alpha * 140)), xpBuf);
        draw->AddText(font, xpSize, ImVec2(xpX, xpY),
                      IM_COL32(100, 220, 100, static_cast<int>(alpha * 230)), xpBuf);
    }
}

// ---------------------------------------------------------------------------
// Quest objective progress toasts - shown at screen bottom-right on kill/item updates
// ---------------------------------------------------------------------------

void ToastManager::renderQuestProgressToasts() {
    if (questToasts_.empty()) return;

    float dt = ImGui::GetIO().DeltaTime;
    for (auto& t : questToasts_) t.age += dt;
    questToasts_.erase(
        std::remove_if(questToasts_.begin(), questToasts_.end(),
            [](const QuestProgressToastEntry& t) { return t.age >= QUEST_TOAST_DURATION; }),
        questToasts_.end());
    if (questToasts_.empty()) return;

    ImVec2 displaySize = ImGui::GetIO().DisplaySize;
    float screenW = displaySize.x > 0.0f ? displaySize.x : 1280.0f;
    float screenH = displaySize.y > 0.0f ? displaySize.y : 720.0f;

    // Stack at bottom-right, just above action bar area
    constexpr float TOAST_W   = 240.0f;
    constexpr float TOAST_H   = 48.0f;
    constexpr float TOAST_GAP = 4.0f;
    float baseY = screenH * 0.72f;
    float toastX = screenW - TOAST_W - 14.0f;

    ImDrawList* bgDL = ImGui::GetBackgroundDrawList();
    const int count = static_cast<int>(questToasts_.size());

    for (int i = 0; i < count; ++i) {
        const auto& toast = questToasts_[i];

        float remaining = QUEST_TOAST_DURATION - toast.age;
        float alpha;
        if (toast.age < 0.2f)
            alpha = toast.age / 0.2f;
        else if (remaining < 1.0f)
            alpha = remaining;
        else
            alpha = 1.0f;
        alpha = std::clamp(alpha, 0.0f, 1.0f);

        float ty = baseY - (count - i) * (TOAST_H + TOAST_GAP);

        uint8_t bgA = static_cast<uint8_t>(200 * alpha);
        uint8_t fgA = static_cast<uint8_t>(255 * alpha);

        // Background: dark amber tint (quest color convention)
        bgDL->AddRectFilled(ImVec2(toastX, ty), ImVec2(toastX + TOAST_W, ty + TOAST_H),
                            IM_COL32(35, 25, 5, bgA), 5.0f);
        bgDL->AddRect(ImVec2(toastX, ty), ImVec2(toastX + TOAST_W, ty + TOAST_H),
                      IM_COL32(200, 160, 30, static_cast<uint8_t>(160 * alpha)), 5.0f, 0, 1.5f);

        // Quest title (gold, small)
        bgDL->AddText(ImVec2(toastX + 8.0f, ty + 5.0f),
                      IM_COL32(220, 180, 50, fgA), toast.questTitle.c_str());

        // Progress bar + text: "ObjectiveName X / Y"
        float barY  = ty + 21.0f;
        float barX0 = toastX + 8.0f;
        float barX1 = toastX + TOAST_W - 8.0f;
        float barH  = 8.0f;
        float pct   = (toast.required > 0)
            ? std::min(1.0f, static_cast<float>(toast.current) / static_cast<float>(toast.required))
            : 1.0f;
        // Bar background
        bgDL->AddRectFilled(ImVec2(barX0, barY), ImVec2(barX1, barY + barH),
                            IM_COL32(50, 40, 10, static_cast<uint8_t>(180 * alpha)), 3.0f);
        // Bar fill - green when complete, amber otherwise
        ImU32 barCol = (pct >= 1.0f) ? IM_COL32(60, 220, 80, fgA) : IM_COL32(200, 160, 30, fgA);
        bgDL->AddRectFilled(ImVec2(barX0, barY),
                            ImVec2(barX0 + (barX1 - barX0) * pct, barY + barH),
                            barCol, 3.0f);

        // Objective name + count
        char progBuf[48];
        if (!toast.objectiveName.empty())
            snprintf(progBuf, sizeof(progBuf), "%.22s: %u/%u",
                     toast.objectiveName.c_str(), toast.current, toast.required);
        else
            snprintf(progBuf, sizeof(progBuf), "%u/%u", toast.current, toast.required);
        bgDL->AddText(ImVec2(toastX + 8.0f, ty + 32.0f),
                      IM_COL32(220, 220, 200, static_cast<uint8_t>(210 * alpha)), progBuf);
    }
}

// ---------------------------------------------------------------------------
// Item loot toasts - quality-coloured strip at bottom-left when item received
// ---------------------------------------------------------------------------

void ToastManager::renderItemLootToasts() {
    if (itemLootToasts_.empty()) return;

    float dt = ImGui::GetIO().DeltaTime;
    for (auto& t : itemLootToasts_) t.age += dt;
    itemLootToasts_.erase(
        std::remove_if(itemLootToasts_.begin(), itemLootToasts_.end(),
            [](const ItemLootToastEntry& t) { return t.age >= ITEM_LOOT_TOAST_DURATION; }),
        itemLootToasts_.end());
    if (itemLootToasts_.empty()) return;

    ImVec2 displaySize = ImGui::GetIO().DisplaySize;
    float screenH = displaySize.y > 0.0f ? displaySize.y : 720.0f;

    // Quality colours (matching WoW convention)
    static const ImU32 kQualityColors[] = {
        IM_COL32(157, 157, 157, 255),  // 0 grey (poor)
        IM_COL32(255, 255, 255, 255),  // 1 white (common)
        IM_COL32( 30, 255,  30, 255),  // 2 green (uncommon)
        IM_COL32(  0, 112, 221, 255),  // 3 blue (rare)
        IM_COL32(163,  53, 238, 255),  // 4 purple (epic)
        IM_COL32(255, 128,   0, 255),  // 5 orange (legendary)
        IM_COL32(230, 204, 128, 255),  // 6 light gold (artifact)
        IM_COL32(230, 204, 128, 255),  // 7 light gold (heirloom)
    };

    // Stack at bottom-left above action bars; each item is 24 px tall
    constexpr float TOAST_W   = 260.0f;
    constexpr float TOAST_H   = 24.0f;
    constexpr float TOAST_GAP = 2.0f;
    constexpr float TOAST_X   = 14.0f;
    float baseY = screenH * 0.68f;  // slightly above the whisper toasts

    ImDrawList* bgDL = ImGui::GetBackgroundDrawList();
    const int count = static_cast<int>(itemLootToasts_.size());

    for (int i = 0; i < count; ++i) {
        const auto& toast = itemLootToasts_[i];

        float remaining = ITEM_LOOT_TOAST_DURATION - toast.age;
        float alpha;
        if (toast.age < 0.15f)
            alpha = toast.age / 0.15f;
        else if (remaining < 0.7f)
            alpha = remaining / 0.7f;
        else
            alpha = 1.0f;
        alpha = std::clamp(alpha, 0.0f, 1.0f);

        // Slide-in from left
        float slideX = (toast.age < 0.15f) ? (TOAST_W * (1.0f - toast.age / 0.15f)) : 0.0f;
        float tx = TOAST_X - slideX;
        float ty = baseY - (count - i) * (TOAST_H + TOAST_GAP);

        uint8_t bgA = static_cast<uint8_t>(180 * alpha);
        uint8_t fgA = static_cast<uint8_t>(255 * alpha);

        // Background: very dark with quality-tinted left border accent
        bgDL->AddRectFilled(ImVec2(tx, ty), ImVec2(tx + TOAST_W, ty + TOAST_H),
                            IM_COL32(12, 12, 12, bgA), 3.0f);

        // Quality colour accent bar on left edge (3px wide)
        ImU32 qualCol = kQualityColors[std::min(static_cast<uint32_t>(7u), toast.quality)];
        ImU32 qualColA = (qualCol & 0x00FFFFFFu) | (static_cast<uint32_t>(fgA) << 24u);
        bgDL->AddRectFilled(ImVec2(tx, ty), ImVec2(tx + 3.0f, ty + TOAST_H), qualColA, 3.0f);

        // "Loot:" label in dim white
        bgDL->AddText(ImVec2(tx + 7.0f, ty + 5.0f),
                      IM_COL32(160, 160, 160, static_cast<uint8_t>(200 * alpha)), "Loot:");

        // Item name in quality colour
        std::string displayName = toast.name.empty() ? ("Item #" + std::to_string(toast.itemId)) : toast.name;
        if (displayName.size() > 26) { displayName.resize(23); displayName += "..."; }
        bgDL->AddText(ImVec2(tx + 42.0f, ty + 5.0f), qualColA, displayName.c_str());

        // Count (if > 1)
        if (toast.count > 1) {
            char countBuf[12];
            snprintf(countBuf, sizeof(countBuf), "x%u", toast.count);
            bgDL->AddText(ImVec2(tx + TOAST_W - 34.0f, ty + 5.0f),
                          IM_COL32(200, 200, 200, static_cast<uint8_t>(200 * alpha)), countBuf);
        }
    }
}

// ---------------------------------------------------------------------------
// PvP honor credit toasts - shown at screen top-right on honorable kill
// ---------------------------------------------------------------------------

void ToastManager::renderPvpHonorToasts() {
    if (pvpHonorToasts_.empty()) return;

    float dt = ImGui::GetIO().DeltaTime;
    for (auto& t : pvpHonorToasts_) t.age += dt;
    pvpHonorToasts_.erase(
        std::remove_if(pvpHonorToasts_.begin(), pvpHonorToasts_.end(),
            [](const PvpHonorToastEntry& t) { return t.age >= PVP_HONOR_TOAST_DURATION; }),
        pvpHonorToasts_.end());
    if (pvpHonorToasts_.empty()) return;

    ImVec2 displaySize = ImGui::GetIO().DisplaySize;
    float screenW = displaySize.x > 0.0f ? displaySize.x : 1280.0f;

    // Stack toasts at top-right, below any minimap area
    constexpr float TOAST_W   = 180.0f;
    constexpr float TOAST_H   = 30.0f;
    constexpr float TOAST_GAP = 3.0f;
    constexpr float TOAST_TOP = 10.0f;
    float toastX = screenW - TOAST_W - 10.0f;

    ImDrawList* bgDL = ImGui::GetBackgroundDrawList();
    const int count = static_cast<int>(pvpHonorToasts_.size());

    for (int i = 0; i < count; ++i) {
        const auto& toast = pvpHonorToasts_[i];

        float remaining = PVP_HONOR_TOAST_DURATION - toast.age;
        float alpha;
        if (toast.age < 0.15f)
            alpha = toast.age / 0.15f;
        else if (remaining < 0.8f)
            alpha = remaining / 0.8f;
        else
            alpha = 1.0f;
        alpha = std::clamp(alpha, 0.0f, 1.0f);

        float ty = TOAST_TOP + i * (TOAST_H + TOAST_GAP);

        uint8_t bgA = static_cast<uint8_t>(190 * alpha);
        uint8_t fgA = static_cast<uint8_t>(255 * alpha);

        // Background: dark red (PvP theme)
        bgDL->AddRectFilled(ImVec2(toastX, ty), ImVec2(toastX + TOAST_W, ty + TOAST_H),
                            IM_COL32(28, 5, 5, bgA), 4.0f);
        bgDL->AddRect(ImVec2(toastX, ty), ImVec2(toastX + TOAST_W, ty + TOAST_H),
                      IM_COL32(200, 50, 50, static_cast<uint8_t>(160 * alpha)), 4.0f, 0, 1.2f);

        // Sword ⚔ icon (U+2694, UTF-8: e2 9a 94)
        bgDL->AddText(ImVec2(toastX + 7.0f, ty + 7.0f),
                      IM_COL32(220, 80, 80, fgA), "\xe2\x9a\x94");

        // "+N Honor" text in gold
        char buf[40];
        snprintf(buf, sizeof(buf), "+%u Honor", toast.honor);
        bgDL->AddText(ImVec2(toastX + 24.0f, ty + 8.0f),
                      IM_COL32(255, 210, 50, fgA), buf);
    }
}

// ---------------------------------------------------------------------------
// Nearby player level-up toasts - shown at screen bottom-centre
// ---------------------------------------------------------------------------

void ToastManager::renderPlayerLevelUpToasts(game::GameHandler& gameHandler) {
    if (playerLevelUpToasts_.empty()) return;

    float dt = ImGui::GetIO().DeltaTime;
    for (auto& t : playerLevelUpToasts_) {
        t.age += dt;
        // Lazy name resolution - fill in once the name cache has it
        if (t.playerName.empty() && t.guid != 0) {
            t.playerName = gameHandler.lookupName(t.guid);
        }
    }
    playerLevelUpToasts_.erase(
        std::remove_if(playerLevelUpToasts_.begin(), playerLevelUpToasts_.end(),
            [](const PlayerLevelUpToastEntry& t) {
                return t.age >= PLAYER_LEVELUP_TOAST_DURATION;
            }),
        playerLevelUpToasts_.end());
    if (playerLevelUpToasts_.empty()) return;

    ImVec2 displaySize = ImGui::GetIO().DisplaySize;
    float screenW = displaySize.x > 0.0f ? displaySize.x : 1280.0f;
    float screenH = displaySize.y > 0.0f ? displaySize.y : 720.0f;

    // Stack toasts at screen bottom-centre, above action bars
    constexpr float TOAST_W   = 230.0f;
    constexpr float TOAST_H   = 38.0f;
    constexpr float TOAST_GAP = 4.0f;
    float baseY  = screenH * 0.72f;
    float toastX = (screenW - TOAST_W) * 0.5f;

    ImDrawList* bgDL = ImGui::GetBackgroundDrawList();
    const int count = static_cast<int>(playerLevelUpToasts_.size());

    for (int i = 0; i < count; ++i) {
        const auto& toast = playerLevelUpToasts_[i];

        float remaining = PLAYER_LEVELUP_TOAST_DURATION - toast.age;
        float alpha;
        if (toast.age < 0.2f)
            alpha = toast.age / 0.2f;
        else if (remaining < 1.0f)
            alpha = remaining;
        else
            alpha = 1.0f;
        alpha = std::clamp(alpha, 0.0f, 1.0f);

        // Subtle pop-up from below during first 0.2s
        float slideY = (toast.age < 0.2f) ? (TOAST_H * (1.0f - toast.age / 0.2f)) : 0.0f;
        float ty = baseY - (count - i) * (TOAST_H + TOAST_GAP) + slideY;

        uint8_t bgA = static_cast<uint8_t>(200 * alpha);
        uint8_t fgA = static_cast<uint8_t>(255 * alpha);

        // Background: dark gold tint
        bgDL->AddRectFilled(ImVec2(toastX, ty), ImVec2(toastX + TOAST_W, ty + TOAST_H),
                            IM_COL32(30, 22, 5, bgA), 5.0f);
        // Gold border with glow at peak
        float glowStr = (toast.age < 0.5f) ? (1.0f - toast.age / 0.5f) : 0.0f;
        uint8_t borderA = static_cast<uint8_t>((160 + 80 * glowStr) * alpha);
        bgDL->AddRect(ImVec2(toastX, ty), ImVec2(toastX + TOAST_W, ty + TOAST_H),
                      IM_COL32(255, 210, 50, borderA), 5.0f, 0, 1.5f + glowStr * 1.5f);

        // Star ★ icon on left
        bgDL->AddText(ImVec2(toastX + 8.0f, ty + 10.0f),
                      IM_COL32(255, 220, 60, fgA), "\xe2\x98\x85");  // UTF-8 ★

        // "<Name> is now level X!" text
        const char* displayName = toast.playerName.empty() ? "A player" : toast.playerName.c_str();
        char buf[64];
        snprintf(buf, sizeof(buf), "%.18s is now level %u!", displayName, toast.newLevel);
        bgDL->AddText(ImVec2(toastX + 26.0f, ty + 11.0f),
                      IM_COL32(255, 230, 100, fgA), buf);
    }
}

// ---------------------------------------------------------------------------
// Resurrection flash - brief screen brightening + "You have been resurrected!"
// banner when the player transitions from ghost back to alive.
// ---------------------------------------------------------------------------

void ToastManager::renderResurrectFlash() {
    if (resurrectFlashTimer_ <= 0.0f) return;

    float dt = ImGui::GetIO().DeltaTime;
    resurrectFlashTimer_ -= dt;
    if (resurrectFlashTimer_ <= 0.0f) {
        resurrectFlashTimer_ = 0.0f;
        return;
    }

    ImVec2 displaySize = ImGui::GetIO().DisplaySize;
    float screenW = displaySize.x > 0.0f ? displaySize.x : 1280.0f;
    float screenH = displaySize.y > 0.0f ? displaySize.y : 720.0f;

    // Normalised age in [0, 1] (0 = just fired, 1 = fully elapsed)
    float t = 1.0f - resurrectFlashTimer_ / kResurrectFlashDuration;

    // Alpha envelope: fast fade-in (first 0.15s), hold, then fade-out (last 0.8s)
    float alpha;
    const float fadeIn  = 0.15f / kResurrectFlashDuration;   // ~5% of lifetime
    const float fadeOut = 0.8f  / kResurrectFlashDuration;   // ~27% of lifetime
    if (t < fadeIn)
        alpha = t / fadeIn;
    else if (t < 1.0f - fadeOut)
        alpha = 1.0f;
    else
        alpha = (1.0f - t) / fadeOut;
    alpha = std::clamp(alpha, 0.0f, 1.0f);

    ImDrawList* bg = ImGui::GetBackgroundDrawList();

    // Soft golden/white vignette - brightening instead of darkening
    uint8_t vigA = static_cast<uint8_t>(50 * alpha);
    bg->AddRectFilled(ImVec2(0, 0), ImVec2(screenW, screenH),
                      IM_COL32(200, 230, 255, vigA));

    // Centered banner panel
    constexpr float PANEL_W = 360.0f;
    constexpr float PANEL_H = 52.0f;
    float px = (screenW - PANEL_W) * 0.5f;
    float py = screenH * 0.34f;

    uint8_t bgA     = static_cast<uint8_t>(210 * alpha);
    uint8_t borderA = static_cast<uint8_t>(255 * alpha);
    uint8_t textA   = static_cast<uint8_t>(255 * alpha);

    // Background: deep blue-black
    bg->AddRectFilled(ImVec2(px, py), ImVec2(px + PANEL_W, py + PANEL_H),
                      IM_COL32(10, 18, 40, bgA), 8.0f);

    // Border glow: bright holy gold
    bg->AddRect(ImVec2(px, py), ImVec2(px + PANEL_W, py + PANEL_H),
                IM_COL32(200, 230, 100, borderA), 8.0f, 0, 2.0f);
    // Inner halo line
    bg->AddRect(ImVec2(px + 3.0f, py + 3.0f), ImVec2(px + PANEL_W - 3.0f, py + PANEL_H - 3.0f),
                IM_COL32(255, 255, 180, static_cast<uint8_t>(80 * alpha)), 6.0f, 0, 1.0f);

    // Resurrection message centered without decorative punctuation.
    const char* banner = "You have been resurrected";
    ImFont* font = ImGui::GetFont();
    float fontSize = ImGui::GetFontSize();
    ImVec2 textSz = font->CalcTextSizeA(fontSize, FLT_MAX, 0.0f, banner);
    float tx = px + (PANEL_W - textSz.x) * 0.5f;
    float ty = py + (PANEL_H - textSz.y) * 0.5f;

    // Drop shadow
    bg->AddText(font, fontSize, ImVec2(tx + 1.0f, ty + 1.0f),
                IM_COL32(0, 0, 0, static_cast<uint8_t>(180 * alpha)), banner);
    // Main text in warm gold
    bg->AddText(font, fontSize, ImVec2(tx, ty),
                IM_COL32(255, 240, 120, textA), banner);
}

// ---------------------------------------------------------------------------
// Whisper toast notifications - brief overlay when a player whispers you
// ---------------------------------------------------------------------------

void ToastManager::renderWhisperToasts() {
    if (whisperToasts_.empty()) return;

    float dt = ImGui::GetIO().DeltaTime;

    // Age and prune expired toasts
    for (auto& t : whisperToasts_) t.age += dt;
    whisperToasts_.erase(
        std::remove_if(whisperToasts_.begin(), whisperToasts_.end(),
            [](const WhisperToastEntry& t) { return t.age >= WHISPER_TOAST_DURATION; }),
        whisperToasts_.end());
    if (whisperToasts_.empty()) return;

    ImVec2 displaySize = ImGui::GetIO().DisplaySize;
    float screenH = displaySize.y > 0.0f ? displaySize.y : 720.0f;

    // Stack toasts at bottom-left, above the action bars (y ≈ screenH * 0.72)
    // Each toast is ~56px tall with a 4px gap between them.
    constexpr float TOAST_W   = 280.0f;
    constexpr float TOAST_H   = 56.0f;
    constexpr float TOAST_GAP = 4.0f;
    constexpr float TOAST_X   = 14.0f;  // left edge (won't cover action bars)
    float baseY = screenH * 0.72f;

    ImDrawList* bgDL = ImGui::GetBackgroundDrawList();

    const int count = static_cast<int>(whisperToasts_.size());
    for (int i = 0; i < count; ++i) {
        auto& toast = whisperToasts_[i];

        // Fade in over 0.25s; fade out in last 1.0s
        float alpha;
        float remaining = WHISPER_TOAST_DURATION - toast.age;
        if (toast.age < 0.25f)
            alpha = toast.age / 0.25f;
        else if (remaining < 1.0f)
            alpha = remaining;
        else
            alpha = 1.0f;
        alpha = std::clamp(alpha, 0.0f, 1.0f);

        // Slide-in from left: offset 0→0 after 0.25s
        float slideX = (toast.age < 0.25f) ? (TOAST_W * (1.0f - toast.age / 0.25f)) : 0.0f;
        float tx = TOAST_X - slideX;
        float ty = baseY - (count - i) * (TOAST_H + TOAST_GAP);

        uint8_t bgA = static_cast<uint8_t>(210 * alpha);
        uint8_t fgA = static_cast<uint8_t>(255 * alpha);

        // Background panel - dark purple tint (whisper color convention)
        bgDL->AddRectFilled(ImVec2(tx, ty), ImVec2(tx + TOAST_W, ty + TOAST_H),
                            IM_COL32(25, 10, 40, bgA), 6.0f);
        // Purple border
        bgDL->AddRect(ImVec2(tx, ty), ImVec2(tx + TOAST_W, ty + TOAST_H),
                      IM_COL32(160, 80, 220, static_cast<uint8_t>(180 * alpha)), 6.0f, 0, 1.5f);

        // "Whisper" label (small, purple-ish)
        bgDL->AddText(ImVec2(tx + 10.0f, ty + 6.0f),
                      IM_COL32(190, 110, 255, fgA), "Whisper from:");

        // Sender name (gold)
        bgDL->AddText(ImVec2(tx + 10.0f, ty + 20.0f),
                      IM_COL32(255, 210, 50, fgA), toast.sender.c_str());

        // Message preview (white, dimmer)
        bgDL->AddText(ImVec2(tx + 10.0f, ty + 36.0f),
                      IM_COL32(220, 220, 220, static_cast<uint8_t>(200 * alpha)),
                      toast.preview.c_str());
    }
}

// Zone discovery text - "Entering: <ZoneName>" fades in/out at screen centre
// ---------------------------------------------------------------------------

void ToastManager::renderZoneText(game::GameHandler& gameHandler) {
    // Poll worldStateZoneId for server-driven zone changes (fires on every zone crossing,
    // including sub-zones like Ironforge within Dun Morogh).
    uint32_t wsZoneId = gameHandler.getWorldStateZoneId();
    if (wsZoneId != 0 && wsZoneId != lastKnownWorldStateZoneId_) {
        lastKnownWorldStateZoneId_ = wsZoneId;
        std::string wsName = gameHandler.getWhoAreaName(wsZoneId);
        if (!wsName.empty()) {
            zoneTextName_  = wsName;
            zoneTextTimer_ = ZONE_TEXT_DURATION;
        }
    }

    // Also poll the renderer for zone name changes (covers map-level transitions
    // where worldStateZoneId may not change immediately).
    auto* appRenderer = services_.renderer;
    if (appRenderer) {
        const std::string& zoneName = appRenderer->getCurrentZoneName();
        if (!zoneName.empty() && zoneName != lastKnownZoneName_) {
            lastKnownZoneName_ = zoneName;
            // Only override if the worldState hasn't already queued this zone
            if (zoneTextName_ != zoneName) {
                zoneTextName_  = zoneName;
                zoneTextTimer_ = ZONE_TEXT_DURATION;
            }
        }
    }

    if (zoneTextTimer_ <= 0.0f || zoneTextName_.empty()) return;

    float dt = ImGui::GetIO().DeltaTime;
    zoneTextTimer_ -= dt;
    if (zoneTextTimer_ < 0.0f) zoneTextTimer_ = 0.0f;

    // Drawn only if FrameXML is not, but tracked either way: the zone toasts
    // above are suppressed while this timer is running, so a plain early
    // return at the top of this function would swap one duplicate for
    // another - the banner would go and "Entering: Stormwind City" would come
    // back beside FrameXML's.
    if (frameXmlOwns(UiElement::ZoneText)) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth())  : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) :  720.0f;

    // Fade: ramp up in first 0.5 s, hold, fade out in last 1.0 s
    float alpha;
    if (zoneTextTimer_ > ZONE_TEXT_DURATION - 0.5f)
        alpha = 1.0f - (zoneTextTimer_ - (ZONE_TEXT_DURATION - 0.5f)) / 0.5f;
    else if (zoneTextTimer_ < 1.0f)
        alpha = zoneTextTimer_;
    else
        alpha = 1.0f;
    alpha = std::clamp(alpha, 0.0f, 1.0f);

    ImFont* font = ImGui::GetFont();

    // "Entering:" header
    const char* header = "Entering:";
    float headerSize = 16.0f;
    float nameSize   = 26.0f;

    ImVec2 headerDim = font->CalcTextSizeA(headerSize, FLT_MAX, 0.0f, header);
    ImVec2 nameDim   = font->CalcTextSizeA(nameSize,   FLT_MAX, 0.0f, zoneTextName_.c_str());

    float centreY = screenH * 0.30f;  // upper third, like WoW
    float headerX = (screenW - headerDim.x) * 0.5f;
    float nameX   = (screenW - nameDim.x)   * 0.5f;
    float headerY = centreY;
    float nameY   = centreY + headerDim.y + 4.0f;

    ImDrawList* draw = ImGui::GetForegroundDrawList();

    // "Entering:" in gold
    draw->AddText(font, headerSize, ImVec2(headerX + 1, headerY + 1),
                  IM_COL32(0, 0, 0, static_cast<int>(alpha * 160)), header);
    draw->AddText(font, headerSize, ImVec2(headerX, headerY),
                  IM_COL32(255, 215, 0, static_cast<int>(alpha * 255)), header);

    // Zone name in white
    draw->AddText(font, nameSize, ImVec2(nameX + 1, nameY + 1),
                  IM_COL32(0, 0, 0, static_cast<int>(alpha * 160)), zoneTextName_.c_str());
    draw->AddText(font, nameSize, ImVec2(nameX, nameY),
                  IM_COL32(255, 255, 255, static_cast<int>(alpha * 255)), zoneTextName_.c_str());
}

} } // namespace wowee::ui
