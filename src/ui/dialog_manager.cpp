#include "ui/dialog_manager.hpp"
#include "game/item_text.hpp"
#include "game/inventory_slots.hpp"
#include "ui/framexml_takeover.hpp"
#include "ui/inventory_screen.hpp"
#include "ui/chat_panel.hpp"
#include "ui/chat/chat_utils.hpp"
#include "ui/ui_colors.hpp"
#include "game/game_handler.hpp"
#include "core/application.hpp"

#include <imgui.h>
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <chrono>

namespace wowee { namespace ui {

namespace {
    using namespace wowee::ui::colors;
    constexpr auto& kColorDarkGray = kDarkGray;
    constexpr auto& kColorGreen    = kGreen;
} // namespace


// ---------------------------------------------------------------------------
// Render early dialogs (group invite through LFG role check)
// ---------------------------------------------------------------------------
void DialogManager::renderDialogs(game::GameHandler& gameHandler,
                                  InventoryScreen& inventoryScreen,
                                  ChatPanel& chatPanel) {
    // The prompts FrameXML asks with a StaticPopup of its own, on events this
    // client fires: the group invite here, the trade request and summon below,
    // and the resurrect and talent wipe in renderLateDialogs. Whichever side
    // asks the question answers it - AcceptGroup, DeclineGroup,
    // AcceptResurrect, ConfirmSummon, ConfirmTalentWipe, BeginTrade and
    // CancelTrade are all bound, so FrameXML's buttons do the same thing these
    // do.
    //
    // The rest below have no FrameXML counterpart that can appear: the duel,
    // the guild invite, the battleground invites and the LFG pair are all
    // raised from events that are not fired. That claim is only true for the
    // ones named - it read "the rest" when trade and the ready check were also
    // in the list, and both were being drawn twice.
    if (!frameXmlOwns(UiElement::Dialogs)) renderGroupInvitePopup(gameHandler);
    // FrameXML answers DUEL_REQUESTED with StaticPopup_Show("DUEL_REQUESTED"),
    // and this client fires that event - so with dialogs handed over both
    // appeared. Three of the popups in this function were already gated on
    // exactly this and the rest were not.
    if (!frameXmlOwns(UiElement::Dialogs)) renderDuelRequestPopup(gameHandler);
    renderDuelCountdown(gameHandler);
    // The roll dialog belongs to the loot window, and FrameXML has four of its
    // own that open on the same roll. Whichever side draws the loot window
    // draws the roll that comes out of it.
    if (!frameXmlOwns(UiElement::Loot)) {
        // GroupLootFrame1..4, which belong to the loot element rather than to
    // dialogs - they are frames of their own, not static popups.
    if (!frameXmlOwns(UiElement::Loot))
        renderLootRollPopup(gameHandler, inventoryScreen, chatPanel);
    }
    // The trade request and the trade window belong to different elements on
    // FrameXML's side: uiparent.lua answers TRADE_REQUEST with StaticPopup
    // "TRADE", while TradeFrame opens on TRADE_SHOW. This client fires both
    // events, so each surface has to stand down for its own owner.
    if (!frameXmlOwns(UiElement::Dialogs)) renderTradeRequestPopup(gameHandler);
    if (!frameXmlOwns(UiElement::Trade)) {
        // TradeFrame is its own element, and its suppression entry names exactly
    // that frame - so with trade handed over this drew a second trade window
    // beside FrameXML's, both live, both showing the same slots.
    if (!frameXmlOwns(UiElement::Trade))
        renderTradeWindow(gameHandler, inventoryScreen, chatPanel);
    }
    if (!frameXmlOwns(UiElement::Dialogs)) renderSummonRequestPopup(gameHandler);
    // Beside the summon popup now that QUEST_ACCEPT_CONFIRM is fired.
    // uiparent.lua raises "QUEST_ACCEPT" - or "QUEST_ACCEPT_LOG_FULL", which
    // this client's version has no equivalent of - so leaving this ungated
    // asks the same question twice.
    if (!frameXmlOwns(UiElement::Dialogs)) renderSharedQuestPopup(gameHandler);
    // ItemTextFrame answers ITEM_TEXT_BEGIN, which this client fires. This is
    // the client's remaining page-text surface, and it reads different state
    // from the book window that used to sit beside it.
    if (!frameXmlOwns(UiElement::Book)) renderItemTextWindow(gameHandler);
    // StaticPopupDialogs["GUILD_INVITE"], shown from UIParent's own handler
    // for GUILD_INVITE_REQUEST.
    if (!frameXmlOwns(UiElement::Dialogs)) renderGuildInvitePopup(gameHandler);
    if (!frameXmlOwns(UiElement::ReadyCheck)) renderReadyCheckPopup(gameHandler);
    // The battleground invitation, which was the fourth and was missed.
    //
    // FrameXML answers the same three invitations below from staticpopup.lua,
    // and this client fires every event that raises them - so both were on
    // screen. This one is no different: UPDATE_BATTLEFIELD_STATUS is fired
    // from social_handler, battlefieldframe.lua raises
    // CONFIRM_BATTLEFIELD_ENTRY on it, and "dialogs" has been handed over from
    // the start. Every battleground invitation put up two accept buttons.
    //
    // tools/framexml_ungated_draws.py had it the whole time, in among the
    // surfaces this client draws on purpose - the nameplates, the toasts, the
    // damage meter - which is what a report of thirty entries costs when only
    // some of them are faults.
    if (!frameXmlOwns(UiElement::Dialogs)) renderBgInvitePopup(gameHandler);
    if (!frameXmlOwns(UiElement::Dialogs)) {
        renderBfMgrInvitePopup(gameHandler);
    }
    if (!frameXmlOwns(UiElement::DungeonFinder)) {
        // LFDDungeonReadyPopup and LFDRoleCheckPopup, both named in the dungeon
    // finder's suppression list.
    if (!frameXmlOwns(UiElement::DungeonFinder)) renderLfgProposalPopup(gameHandler);
        if (!frameXmlOwns(UiElement::DungeonFinder)) renderLfgRoleCheckPopup(gameHandler);
    }
}

// ---------------------------------------------------------------------------
// Render late dialogs (resurrect, talent wipe, pet unlearn)
// ---------------------------------------------------------------------------
void DialogManager::renderLateDialogs(game::GameHandler& gameHandler) {
    if (!frameXmlOwns(UiElement::Dialogs)) {
        // ShowResurrectRequest, from UIParent's RESURRECT_REQUEST handler.
    if (!frameXmlOwns(UiElement::Dialogs)) renderResurrectDialog(gameHandler);
        // uiparent.lua answers CONFIRM_TALENT_WIPE with StaticPopup_Show for the
    // same name, and this client fires that event - the binding for the
    // popup's own accept button says so in as many words.
    if (!frameXmlOwns(UiElement::Dialogs)) renderTalentWipeConfirmDialog(gameHandler);
    }
    // Not gated: FrameXML has the string for this one and no dialog behind it.
    // staticpopup.lua declares no CONFIRM_PET_UNLEARN, so this client's is the
    // only one and hiding it would leave the confirmation with no answer.
    renderPetUnlearnConfirmDialog(gameHandler);
}

// ============================================================
// Group Invite Popup
// ============================================================

void DialogManager::renderGroupInvitePopup(game::GameHandler& gameHandler) {
    if (!gameHandler.hasPendingGroupInvite()) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 150, 200), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(300, 0), ImGuiCond_Always);

    if (ImGui::Begin("Group Invite", nullptr, kDialogFlags)) {
        ImGui::Text("%s has invited you to a group.", gameHandler.getPendingInviterName().c_str());
        ImGui::Spacing();

        if (ImGui::Button("Accept", ImVec2(130, 30))) {
            gameHandler.acceptGroupInvite();
        }
        ImGui::SameLine();
        if (ImGui::Button("Decline", ImVec2(130, 30))) {
            gameHandler.declineGroupInvite();
        }
    }
    ImGui::End();
}

void DialogManager::renderDuelRequestPopup(game::GameHandler& gameHandler) {
    if (!gameHandler.hasPendingDuelRequest()) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 150, 250), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(300, 0), ImGuiCond_Always);

    if (ImGui::Begin("Duel Request", nullptr, kDialogFlags)) {
        ImGui::Text("%s challenges you to a duel!", gameHandler.getDuelChallengerName().c_str());
        ImGui::Spacing();

        if (ImGui::Button("Accept", ImVec2(130, 30))) {
            gameHandler.acceptDuel();
        }
        ImGui::SameLine();
        if (ImGui::Button("Decline", ImVec2(130, 30))) {
            gameHandler.forfeitDuel();
        }
    }
    ImGui::End();
}

void DialogManager::renderDuelCountdown(game::GameHandler& gameHandler) {
    float remaining = gameHandler.getDuelCountdownRemaining();
    if (remaining <= 0.0f) return;

    ImVec2 displaySize = ImGui::GetIO().DisplaySize;
    float screenW = displaySize.x > 0.0f ? displaySize.x : 1280.0f;
    float screenH = displaySize.y > 0.0f ? displaySize.y : 720.0f;

    auto* dl = ImGui::GetForegroundDrawList();
    ImFont* font = ImGui::GetFont();
    float fontSize = ImGui::GetFontSize();

    // Show integer countdown or "Fight!" when under 0.5s
    char buf[32];
    if (remaining > 0.5f) {
        snprintf(buf, sizeof(buf), "%d", static_cast<int>(std::ceil(remaining)));
    } else {
        snprintf(buf, sizeof(buf), "Fight!");
    }

    // Large font by scaling - use 4x font size for dramatic effect
    float scale = 4.0f;
    float scaledSize = fontSize * scale;
    ImVec2 textSz = font->CalcTextSizeA(scaledSize, FLT_MAX, 0.0f, buf);
    float tx = (screenW - textSz.x) * 0.5f;
    float ty = screenH * 0.35f - textSz.y * 0.5f;

    // Pulsing alpha: fades in and out per second
    float pulse = 0.75f + 0.25f * std::sin(static_cast<float>(ImGui::GetTime()) * 6.28f);
    uint8_t alpha = static_cast<uint8_t>(255 * pulse);

    // Color: golden countdown, red "Fight!"
    ImU32 color = (remaining > 0.5f)
        ? IM_COL32(255, 200, 50, alpha)
        : IM_COL32(255, 60, 60, alpha);

    // Drop shadow
    dl->AddText(font, scaledSize, ImVec2(tx + 2.0f, ty + 2.0f), IM_COL32(0, 0, 0, alpha / 2), buf);
    dl->AddText(font, scaledSize, ImVec2(tx, ty), color, buf);
}

void DialogManager::renderItemTextWindow(game::GameHandler& gameHandler) {
    if (!gameHandler.isItemTextOpen()) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) :  720.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW * 0.5f - 200, screenH * 0.15f),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(400, 300), ImGuiCond_FirstUseEver);

    bool open = true;
    if (!ImGui::Begin("Book", &open, ImGuiWindowFlags_NoCollapse)) {
        ImGui::End();
        if (!open) gameHandler.closeItemText();
        return;
    }
    if (!open) {
        ImGui::End();
        gameHandler.closeItemText();
        return;
    }

    // Parchment-toned background text. Resolve WoW $-tokens ($g himself:herself, $n
    // player name, $b line break, ...) the same way quest and chat text does.
    std::string itemText = chat_utils::replaceGenderPlaceholders(gameHandler.getItemText(), gameHandler);
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.2f, 0.1f, 0.0f, 1.0f));
    ImGui::TextWrapped("%s", itemText.c_str());
    ImGui::PopStyleColor();

    ImGui::Spacing();
    if (ImGui::Button("Close", ImVec2(80, 0))) {
        gameHandler.closeItemText();
    }

    ImGui::End();
}

void DialogManager::renderSharedQuestPopup(game::GameHandler& gameHandler) {
    if (!gameHandler.hasPendingSharedQuest()) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 175, 490), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(350, 0), ImGuiCond_Always);

    if (ImGui::Begin("Shared Quest", nullptr, kDialogFlags)) {
        ImGui::Text("%s has shared a quest with you:", gameHandler.getSharedQuestSharerName().c_str());
        ImGui::TextColored(colors::kBrightGold, "\"%s\"", gameHandler.getSharedQuestTitle().c_str());
        ImGui::Spacing();

        if (ImGui::Button("Accept", ImVec2(130, 30))) {
            gameHandler.acceptSharedQuest();
        }
        ImGui::SameLine();
        if (ImGui::Button("Decline", ImVec2(130, 30))) {
            gameHandler.declineSharedQuest();
        }
    }
    ImGui::End();
}

void DialogManager::renderSummonRequestPopup(game::GameHandler& gameHandler) {
    if (!gameHandler.hasPendingSummonRequest()) return;

    // Tick the timeout down
    float dt = ImGui::GetIO().DeltaTime;
    gameHandler.tickSummonTimeout(dt);
    if (!gameHandler.hasPendingSummonRequest()) return;  // expired

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 175, 430), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(350, 0), ImGuiCond_Always);

    if (ImGui::Begin("Summon Request", nullptr, kDialogFlags)) {
        ImGui::Text("%s is summoning you.", gameHandler.getSummonerName().c_str());
        float t = gameHandler.getSummonTimeoutSec();
        if (t > 0.0f) {
            ImGui::Text("Time remaining: %.0fs", t);
        }
        ImGui::Spacing();

        if (ImGui::Button("Accept", ImVec2(130, 30))) {
            gameHandler.acceptSummon();
        }
        ImGui::SameLine();
        if (ImGui::Button("Decline", ImVec2(130, 30))) {
            gameHandler.declineSummon();
        }
    }
    ImGui::End();
}

void DialogManager::renderTradeRequestPopup(game::GameHandler& gameHandler) {
    if (!gameHandler.hasPendingTradeRequest()) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 150, 370), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(300, 0), ImGuiCond_Always);

    if (ImGui::Begin("Trade Request", nullptr, kDialogFlags)) {
        ImGui::Text("%s wants to trade with you.", gameHandler.getTradePeerName().c_str());
        ImGui::Spacing();

        if (ImGui::Button("Accept", ImVec2(130, 30))) {
            gameHandler.acceptTradeRequest();
        }
        ImGui::SameLine();
        if (ImGui::Button("Decline", ImVec2(130, 30))) {
            gameHandler.declineTradeRequest();
        }
    }
    ImGui::End();
}

void DialogManager::renderTradeWindow(game::GameHandler& gameHandler,
                                       InventoryScreen& inventoryScreen,
                                       ChatPanel& chatPanel) {
    if (!gameHandler.isTradeOpen()) return;

    const auto& mySlots   = gameHandler.getMyTradeSlots();
    const auto& peerSlots = gameHandler.getPeerTradeSlots();
    const uint64_t myGold   = gameHandler.getMyTradeGold();
    const uint64_t peerGold = gameHandler.getPeerTradeGold();
    const auto& peerName = gameHandler.getTradePeerName();

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2.0f - 240.0f, screenH / 2.0f - 180.0f), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(480.0f, 360.0f), ImGuiCond_Once);

    bool open = true;
    if (ImGui::Begin(("Trade with " + peerName).c_str(), &open,
                     kDialogFlags)) {

        auto formatGold = [](uint64_t copper, char* buf, size_t bufsz) {
            uint64_t g = copper / 10000;
            uint64_t s = (copper % 10000) / 100;
            uint64_t c = copper % 100;
            if (g > 0) std::snprintf(buf, bufsz, "%llug %llus %lluc",
                                     (unsigned long long)g, (unsigned long long)s, (unsigned long long)c);
            else if (s > 0) std::snprintf(buf, bufsz, "%llus %lluc",
                                          (unsigned long long)s, (unsigned long long)c);
            else std::snprintf(buf, bufsz, "%lluc", (unsigned long long)c);
        };

        auto renderSlotColumn = [&](const char* label,
                                    const std::array<game::GameHandler::TradeSlot,
                                                     game::GameHandler::TRADE_SLOT_COUNT>& slots,
                                    uint64_t gold, bool isMine) {
            ImGui::Text("%s", label);
            ImGui::Separator();

            for (int i = 0; i < game::GameHandler::TRADE_SLOT_COUNT; ++i) {
                const auto& slot = slots[i];
                const bool isNonTraded = (i == game::GameHandler::TRADE_SLOT_NONTRADED);
                ImGui::PushID(i * (isMine ? 1 : -1) - (isMine ? 0 : 100));

                // The non-traded slot is set apart: it stays with its owner and only exists
                // so the other party can enchant/craft on the item placed here.
                if (isNonTraded) {
                    ImGui::Spacing();
                    ImGui::Separator();
                    ImGui::TextDisabled("Will not be traded:");
                }

                if (slot.occupied && slot.itemId != 0) {
                    const auto* info = gameHandler.getItemInfo(slot.itemId);
                    std::string name = (info && info->valid && !info->name.empty())
                        ? info->name
                        : ("Item " + std::to_string(slot.itemId));
                    if (slot.stackCount > 1)
                        name += " x" + std::to_string(slot.stackCount);
                    ImVec4 qc = (info && info->valid)
                        ? InventoryScreen::getQualityColor(static_cast<game::ItemQuality>(info->quality))
                        : ImVec4(1.0f, 0.9f, 0.5f, 1.0f);
                    if (info && info->valid && info->displayInfoId != 0) {
                        VkDescriptorSet iconTex = inventoryScreen.getItemIcon(info->displayInfoId);
                        if (iconTex) {
                            ImGui::Image((ImTextureID)(uintptr_t)iconTex, ImVec2(16, 16));
                            ImGui::SameLine();
                        }
                    }
                    if (isNonTraded) ImGui::TextColored(qc, "%s", name.c_str());
                    else             ImGui::TextColored(qc, "%d. %s", i + 1, name.c_str());
                    if (isMine && ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                        gameHandler.clearTradeItem(static_cast<uint8_t>(i));
                    }
                    if (ImGui::IsItemHovered()) {
                        if (info && info->valid) inventoryScreen.renderItemTooltip(*info);
                        else if (isMine) ImGui::SetTooltip("Double-click to remove");
                    }
                    if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
                        ImGui::GetIO().KeyShift && info && info->valid && !info->name.empty()) {
                        std::string link = game::itemChatLink(info->entry, info->quality, info->name);
                        chatPanel.insertChatLink(link);
                    }
                } else {
                    if (isNonTraded) ImGui::TextDisabled("  (empty - place item to enchant/craft)");
                    else             ImGui::TextDisabled("  %d. (empty)", i + 1);

                    // Allow dragging inventory items into trade slots via right-click context menu
                    char addItemId[16]; snprintf(addItemId, sizeof(addItemId), "##additem%d", i);
                    if (isMine && ImGui::IsItemClicked(ImGuiMouseButton_Right)) {
                        ImGui::OpenPopup(addItemId);
                    }

                    // Dropped out of a handed-over bag. The popup below lists
                    // the backpack only, so with the bags drawn by FrameXML a
                    // drag was the natural way to offer something and there was
                    // nothing here to catch it - and anything in one of the four
                    // worn bags could not be traded at all.
                    if (isMine && ImGui::IsItemHovered() &&
                        ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                        uint8_t srcBag = 0, srcSlot = 0;
                        if (frameXmlCursorWireSlot(srcBag, srcSlot)) {
                            gameHandler.setTradeItem(static_cast<uint8_t>(i), srcBag, srcSlot);
                            frameXmlPutCursorDown();
                        }
                    }
                }

                if (isMine) {
                    char addItemId[16]; snprintf(addItemId, sizeof(addItemId), "##additem%d", i);
                    // Drag-from-inventory: show small popup listing bag items
                    if (ImGui::BeginPopup(addItemId)) {
                        ImGui::TextDisabled("Add from inventory:");
                        const auto& inv = gameHandler.getInventory();
                        // Backpack slots 0-15 (bag=255)
                        for (int si = 0; si < game::Inventory::BACKPACK_SLOTS; ++si) {
                            const auto& slot = inv.getBackpackSlot(si);
                            if (slot.empty()) continue;
                            const auto* ii = gameHandler.getItemInfo(slot.item.itemId);
                            std::string iname = (ii && ii->valid && !ii->name.empty())
                                ? ii->name
                                : (!slot.item.name.empty() ? slot.item.name
                                   : ("Item " + std::to_string(slot.item.itemId)));
                            if (ImGui::Selectable(iname.c_str())) {
                                // Container 255 is the flat space the server
                                // reads with GetItemByPos, where the backpack
                                // starts after the equipment - not at zero.
                                // Sending the backpack index raw named an
                                // equipment slot, and an offer the server finds
                                // no item for cancels the trade outright.
                                gameHandler.setTradeItem(
                                    static_cast<uint8_t>(i), 255u,
                                    static_cast<uint8_t>(game::slots::backpackWireSlot(si)));
                                ImGui::CloseCurrentPopup();
                            }
                        }
                        ImGui::EndPopup();
                    }
                }
                ImGui::PopID();
            }

            // Gold row
            char gbuf[48];
            formatGold(gold, gbuf, sizeof(gbuf));
            ImGui::Spacing();
            if (isMine) {
                ImGui::Text("Gold offered: %s", gbuf);
                static char goldInput[32] = "0";
                ImGui::SetNextItemWidth(120.0f);
                if (ImGui::InputText("##goldset", goldInput, sizeof(goldInput),
                                     ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_EnterReturnsTrue)) {
                    uint64_t copper = std::strtoull(goldInput, nullptr, 10);
                    gameHandler.setTradeGold(copper);
                }
                ImGui::SameLine();
                ImGui::TextDisabled("(copper, Enter to set)");
            } else {
                ImGui::Text("Gold offered: %s", gbuf);
            }
        };

        // Two-column layout: my offer | peer offer
        float colW = ImGui::GetContentRegionAvail().x * 0.5f - 4.0f;
        ImGui::BeginChild("##myoffer", ImVec2(colW, 240.0f), true);
        renderSlotColumn("Your offer", mySlots, myGold, true);
        ImGui::EndChild();

        ImGui::SameLine();

        ImGui::BeginChild("##peroffer", ImVec2(colW, 240.0f), true);
        renderSlotColumn((peerName + "'s offer").c_str(), peerSlots, peerGold, false);
        ImGui::EndChild();

        // Buttons
        ImGui::Spacing();
        ImGui::Separator();
        float bw = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
        if (ImGui::Button("Accept Trade", ImVec2(bw, 0))) {
            gameHandler.acceptTrade();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(bw, 0))) {
            gameHandler.cancelTrade();
        }
    }
    ImGui::End();

    if (!open) {
        gameHandler.cancelTrade();
    }
}

void DialogManager::renderLootRollPopup(game::GameHandler& gameHandler,
                                         InventoryScreen& inventoryScreen,
                                         ChatPanel& chatPanel) {
    if (!gameHandler.hasPendingLootRoll()) return;

    const auto& roll = gameHandler.getPendingLootRoll();

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 175, 310), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(350, 0), ImGuiCond_Always);

    if (ImGui::Begin("Loot Roll", nullptr, kDialogFlags)) {
        // Quality color for item name
        uint8_t q = roll.itemQuality;
        ImVec4 col = ui::getQualityColor(static_cast<game::ItemQuality>(q));

        // Countdown bar
        {
            auto now = std::chrono::steady_clock::now();
            float elapsedMs = static_cast<float>(
                std::chrono::duration_cast<std::chrono::milliseconds>(now - roll.rollStartedAt).count());
            float totalMs   = static_cast<float>(roll.rollCountdownMs > 0 ? roll.rollCountdownMs : 60000);
            float fraction  = 1.0f - std::min(elapsedMs / totalMs, 1.0f);
            float remainSec = (totalMs - elapsedMs) / 1000.0f;
            if (remainSec < 0.0f) remainSec = 0.0f;

            // Color: green → yellow → red
            ImVec4 barColor;
            if (fraction > 0.5f)
                barColor = ImVec4(0.2f + (1.0f - fraction) * 1.4f, 0.85f, 0.2f, 1.0f);
            else if (fraction > 0.2f)
                barColor = ImVec4(1.0f, fraction * 1.7f, 0.1f, 1.0f);
            else {
                float pulse = 0.7f + 0.3f * std::sin(static_cast<float>(ImGui::GetTime()) * 6.0f);
                barColor = ImVec4(pulse, 0.1f * pulse, 0.1f * pulse, 1.0f);
            }

            char timeBuf[16];
            std::snprintf(timeBuf, sizeof(timeBuf), "%.0fs", remainSec);
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, barColor);
            ImGui::ProgressBar(fraction, ImVec2(-1, 12), timeBuf);
            ImGui::PopStyleColor();
        }

        ImGui::Text("An item is up for rolls:");

        // Show item icon if available
        const auto* rollInfo = gameHandler.getItemInfo(roll.itemId);
        uint32_t rollDisplayId = rollInfo ? rollInfo->displayInfoId : 0;
        VkDescriptorSet rollIcon = rollDisplayId ? inventoryScreen.getItemIcon(rollDisplayId) : VK_NULL_HANDLE;
        if (rollIcon) {
            ImGui::Image((ImTextureID)(uintptr_t)rollIcon, ImVec2(24, 24));
            ImGui::SameLine();
        }
        // Prefer live item info (arrives via SMSG_ITEM_QUERY_SINGLE_RESPONSE after the
        // roll popup opens); fall back to the name cached at SMSG_LOOT_START_ROLL time.
        const char* displayName = (rollInfo && rollInfo->valid && !rollInfo->name.empty())
            ? rollInfo->name.c_str()
            : roll.itemName.c_str();
        if (rollInfo && rollInfo->valid)
            col = ui::getQualityColor(static_cast<game::ItemQuality>(rollInfo->quality));
        ImGui::TextColored(col, "[%s]", displayName);
        if (ImGui::IsItemHovered() && rollInfo && rollInfo->valid) {
            inventoryScreen.renderItemTooltip(*rollInfo);
        }
        if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
            ImGui::GetIO().KeyShift && rollInfo && rollInfo->valid && !rollInfo->name.empty()) {
            std::string link = game::itemChatLink(rollInfo->entry, rollInfo->quality, rollInfo->name);
            chatPanel.insertChatLink(link);
        }
        ImGui::Spacing();

        // voteMask bits: 0x01=pass, 0x02=need, 0x04=greed, 0x08=disenchant.
        // CMSG_LOOT_ROLL rollType (WotLK RollVote enum):
        //   0=PASS, 1=NEED, 2=GREED, 3=DISENCHANT
        // Do NOT confuse the bit position in voteMask with the rollType to send.
        const uint8_t vm = roll.voteMask;
        bool first = true;
        if (vm & 0x02) {
            if (ImGui::Button("Need", ImVec2(80, 30)))
                gameHandler.sendLootRoll(roll.objectGuid, roll.slot, 1);
            first = false;
        }
        if (vm & 0x04) {
            if (!first) ImGui::SameLine();
            if (ImGui::Button("Greed", ImVec2(80, 30)))
                gameHandler.sendLootRoll(roll.objectGuid, roll.slot, 2);
            first = false;
        }
        if (vm & 0x08) {
            if (!first) ImGui::SameLine();
            if (ImGui::Button("Disenchant", ImVec2(95, 30)))
                gameHandler.sendLootRoll(roll.objectGuid, roll.slot, 3);
            first = false;
        }
        if (vm & 0x01) {
            if (!first) ImGui::SameLine();
            if (ImGui::Button("Pass", ImVec2(70, 30)))
                gameHandler.sendLootRoll(roll.objectGuid, roll.slot, 0);
        }

        // Live roll results from group members
        if (!roll.playerRolls.empty()) {
            ImGui::Separator();
            ImGui::TextDisabled("Rolls so far:");
            // Roll-type label + color, indexed by RollVote enum
            // (0=Pass, 1=Need, 2=Greed, 3=Disenchant).
            static constexpr const char* kRollLabels[] = {"Pass", "Need", "Greed", "Disenchant"};
            static constexpr ImVec4 kRollColors[] = {
                kColorDarkGray,                    // Pass - gray
                ImVec4(0.2f, 0.9f, 0.2f, 1.0f),  // Need  - green
                ImVec4(0.3f, 0.6f, 1.0f, 1.0f),  // Greed - blue
                ImVec4(0.7f, 0.3f, 0.9f, 1.0f),  // Disenchant - purple
            };
            auto rollTypeIndex = [](uint8_t t) -> int {
                return (t < 4) ? static_cast<int>(t) : 0;
            };

            if (ImGui::BeginTable("##lootrolls", 3,
                    ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg)) {
                ImGui::TableSetupColumn("Player", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("Type",   ImGuiTableColumnFlags_WidthFixed, 72.0f);
                ImGui::TableSetupColumn("Roll",   ImGuiTableColumnFlags_WidthFixed, 32.0f);
                for (const auto& r : roll.playerRolls) {
                    int ri = rollTypeIndex(r.rollType);
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(r.playerName.c_str());
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextColored(kRollColors[ri], "%s", kRollLabels[ri]);
                    ImGui::TableSetColumnIndex(2);
                    if (r.rollType != 0) {
                        ImGui::TextColored(kRollColors[ri], "%d", static_cast<int>(r.rollNum));
                    } else {
                        ImGui::TextDisabled("-");
                    }
                }
                ImGui::EndTable();
            }
        }
    }
    ImGui::End();
}

void DialogManager::renderGuildInvitePopup(game::GameHandler& gameHandler) {
    if (!gameHandler.hasPendingGuildInvite()) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 175, 250), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(350, 0), ImGuiCond_Always);

    if (ImGui::Begin("Guild Invite", nullptr, kDialogFlags)) {
        ImGui::TextWrapped("%s has invited you to join %s.",
                           gameHandler.getPendingGuildInviterName().c_str(),
                           gameHandler.getPendingGuildInviteGuildName().c_str());
        ImGui::Spacing();

        if (ImGui::Button("Accept", ImVec2(155, 30))) {
            gameHandler.acceptGuildInvite();
        }
        ImGui::SameLine();
        if (ImGui::Button("Decline", ImVec2(155, 30))) {
            gameHandler.declineGuildInvite();
        }
    }
    ImGui::End();
}

void DialogManager::renderReadyCheckPopup(game::GameHandler& gameHandler) {
    if (!gameHandler.hasPendingReadyCheck()) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 175, screenH / 2 - 60), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(350, 0), ImGuiCond_Always);

    if (ImGui::Begin("Ready Check", nullptr, kDialogFlags)) {
        const std::string& initiator = gameHandler.getReadyCheckInitiator();
        if (initiator.empty()) {
            ImGui::Text("A ready check has been initiated!");
        } else {
            ImGui::TextWrapped("%s has initiated a ready check!", initiator.c_str());
        }
        ImGui::Spacing();

        if (ImGui::Button("Ready", ImVec2(155, 30))) {
            gameHandler.respondToReadyCheck(true);
            gameHandler.dismissReadyCheck();
        }
        ImGui::SameLine();
        if (ImGui::Button("Not Ready", ImVec2(155, 30))) {
            gameHandler.respondToReadyCheck(false);
            gameHandler.dismissReadyCheck();
        }

        // Live player responses
        const auto& results = gameHandler.getReadyCheckResults();
        if (!results.empty()) {
            ImGui::Separator();
            if (ImGui::BeginTable("##rcresults", 2,
                    ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg)) {
                ImGui::TableSetupColumn("Player", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("Status", ImGuiTableColumnFlags_WidthFixed, 72.0f);
                for (const auto& r : results) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(r.name.c_str());
                    ImGui::TableSetColumnIndex(1);
                    if (r.ready) {
                        ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.2f, 1.0f), "Ready");
                    } else {
                        ImGui::TextColored(ImVec4(0.9f, 0.3f, 0.3f, 1.0f), "Not Ready");
                    }
                }
                ImGui::EndTable();
            }
        }
    }
    ImGui::End();
}

void DialogManager::renderBgInvitePopup(game::GameHandler& gameHandler) {
    if (!gameHandler.hasPendingBgInvite()) return;

    const auto& queues = gameHandler.getBgQueues();
    // Find the first WAIT_JOIN slot
    const game::GameHandler::BgQueueSlot* slot = nullptr;
    for (const auto& s : queues) {
        if (s.statusId == 2) { slot = &s; break; }
    }
    if (!slot) return;

    // Compute time remaining
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - slot->inviteReceivedTime).count();
    double remaining = static_cast<double>(slot->inviteTimeout) - elapsed;

    // If invite has expired, clear it silently (server will handle the queue)
    if (remaining <= 0.0) {
        gameHandler.declineBattlefield(slot->queueSlot);
        return;
    }

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 190, screenH / 2 - 70), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(380, 0), ImGuiCond_Always);

    ImGui::PushStyleColor(ImGuiCol_WindowBg,   ImVec4(0.08f, 0.08f, 0.18f, 0.95f));
    ImGui::PushStyleColor(ImGuiCol_Border,     ImVec4(0.4f, 0.4f, 1.0f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImVec4(0.15f, 0.15f, 0.4f, 1.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 6.0f);

    const ImGuiWindowFlags popupFlags =
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse;

    if (ImGui::Begin("Battleground Ready!", nullptr, popupFlags)) {
        // BG name from stored queue data
        std::string bgName = slot->bgName.empty() ? "Battleground" : slot->bgName;
        ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.2f, 1.0f), "%s", bgName.c_str());
        ImGui::TextWrapped("A spot has opened! You have %d seconds to enter.", static_cast<int>(remaining));
        ImGui::Spacing();

        // Countdown progress bar
        float frac = static_cast<float>(remaining / static_cast<double>(slot->inviteTimeout));
        frac = std::clamp(frac, 0.0f, 1.0f);
        ImVec4 barColor = frac > 0.5f ? colors::kHealthGreen
                        : frac > 0.25f ? ImVec4(0.9f, 0.7f, 0.1f, 1.0f)
                                       : colors::kDarkRed;
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, barColor);
        char countdownLabel[32];
        snprintf(countdownLabel, sizeof(countdownLabel), "%ds", static_cast<int>(remaining));
        ImGui::ProgressBar(frac, ImVec2(-1, 16), countdownLabel);
        ImGui::PopStyleColor();
        ImGui::Spacing();

        ImGui::PushStyleColor(ImGuiCol_Button,        colors::kBtnGreen);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, colors::kFriendlyGreen);
        if (ImGui::Button("Enter Battleground", ImVec2(180, 30))) {
            gameHandler.acceptBattlefield(slot->queueSlot);
        }
        ImGui::PopStyleColor(2);

        ImGui::SameLine();

        ImGui::PushStyleColor(ImGuiCol_Button,        colors::kBtnRed);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, colors::kDangerRed);
        if (ImGui::Button("Leave Queue", ImVec2(175, 30))) {
            gameHandler.declineBattlefield(slot->queueSlot);
        }
        ImGui::PopStyleColor(2);
    }
    ImGui::End();

    ImGui::PopStyleVar();
    ImGui::PopStyleColor(3);
}

void DialogManager::renderBfMgrInvitePopup(game::GameHandler& gameHandler) {
    // Only shown on WotLK servers (outdoor battlefields like Wintergrasp use the BF Manager)
    if (!gameHandler.hasBfMgrInvite()) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2.0f - 190.0f, screenH / 2.0f - 55.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(380.0f, 0.0f), ImGuiCond_Always);

    ImGui::PushStyleColor(ImGuiCol_WindowBg,      ImVec4(0.08f, 0.10f, 0.20f, 0.96f));
    ImGui::PushStyleColor(ImGuiCol_Border,        ImVec4(0.5f, 0.5f, 1.0f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImVec4(0.15f, 0.15f, 0.45f, 1.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 6.0f);

    const ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse;

    if (ImGui::Begin("Battlefield", nullptr, flags)) {
        // Resolve zone name for Wintergrasp (zoneId 4197)
        uint32_t zoneId = gameHandler.getBfMgrZoneId();
        const char* zoneName = nullptr;
        if (zoneId == 4197) zoneName = "Wintergrasp";
        else if (zoneId == 5095) zoneName = "Tol Barad";

        if (zoneName) {
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.2f, 1.0f), "%s", zoneName);
        } else {
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.2f, 1.0f), "Outdoor Battlefield");
        }
        ImGui::Spacing();
        ImGui::TextWrapped("You are invited to join the outdoor battlefield. Do you want to enter?");
        ImGui::Spacing();

        ImGui::PushStyleColor(ImGuiCol_Button,        colors::kBtnGreen);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, colors::kFriendlyGreen);
        if (ImGui::Button("Enter Battlefield", ImVec2(178, 28))) {
            gameHandler.acceptBfMgrInvite();
        }
        ImGui::PopStyleColor(2);

        ImGui::SameLine();

        ImGui::PushStyleColor(ImGuiCol_Button,        colors::kBtnRed);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, colors::kDangerRed);
        if (ImGui::Button("Decline", ImVec2(175, 28))) {
            gameHandler.declineBfMgrInvite();
        }
        ImGui::PopStyleColor(2);
    }
    ImGui::End();

    ImGui::PopStyleVar();
    ImGui::PopStyleColor(3);
}

void DialogManager::renderLfgProposalPopup(game::GameHandler& gameHandler) {
    using LfgState = game::GameHandler::LfgState;
    if (gameHandler.getLfgState() != LfgState::Proposal) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2.0f - 175.0f, screenH / 2.0f - 65.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(350.0f, 0.0f), ImGuiCond_Always);

    ImGui::PushStyleColor(ImGuiCol_WindowBg,      ImVec4(0.08f, 0.14f, 0.08f, 0.96f));
    ImGui::PushStyleColor(ImGuiCol_Border,        ImVec4(0.3f, 0.8f, 0.3f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImVec4(0.1f, 0.3f, 0.1f, 1.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 6.0f);

    const ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse;

    if (ImGui::Begin("Dungeon Finder", nullptr, flags)) {
        ImGui::TextColored(kColorGreen, "A group has been found!");
        ImGui::Spacing();
        ImGui::TextWrapped("Please accept or decline to join the dungeon.");
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::PushStyleColor(ImGuiCol_Button,        colors::kBtnGreen);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, colors::kFriendlyGreen);
        if (ImGui::Button("Accept", ImVec2(155.0f, 30.0f))) {
            gameHandler.lfgAcceptProposal(gameHandler.getLfgProposalId(), true);
        }
        ImGui::PopStyleColor(2);

        ImGui::SameLine();

        ImGui::PushStyleColor(ImGuiCol_Button,        colors::kBtnRed);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, colors::kDangerRed);
        if (ImGui::Button("Decline", ImVec2(155.0f, 30.0f))) {
            gameHandler.lfgAcceptProposal(gameHandler.getLfgProposalId(), false);
        }
        ImGui::PopStyleColor(2);
    }
    ImGui::End();

    ImGui::PopStyleVar();
    ImGui::PopStyleColor(3);
}

void DialogManager::renderLfgRoleCheckPopup(game::GameHandler& gameHandler) {
    using LfgState = game::GameHandler::LfgState;
    if (gameHandler.getLfgState() != LfgState::RoleCheck) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2.0f - 160.0f, screenH / 2.0f - 80.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(320.0f, 0.0f), ImGuiCond_Always);

    ImGui::PushStyleColor(ImGuiCol_WindowBg,      ImVec4(0.08f, 0.08f, 0.18f, 0.96f));
    ImGui::PushStyleColor(ImGuiCol_Border,        ImVec4(0.3f, 0.5f, 0.9f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImVec4(0.1f, 0.1f, 0.3f, 1.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 6.0f);

    const ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse;

    if (ImGui::Begin("Role Check##LfgRoleCheck", nullptr, flags)) {
        ImGui::TextColored(ImVec4(0.4f, 0.7f, 1.0f, 1.0f), "Confirm your role:");
        ImGui::Spacing();

        // Role checkboxes
        bool isTank   = (lfgRoles_ & 0x02) != 0;
        bool isHealer = (lfgRoles_ & 0x04) != 0;
        bool isDps    = (lfgRoles_ & 0x08) != 0;

        if (ImGui::Checkbox("Tank",   &isTank))   lfgRoles_ = (lfgRoles_ & ~0x02) | (isTank   ? 0x02 : 0);
        ImGui::SameLine(120.0f);
        if (ImGui::Checkbox("Healer", &isHealer)) lfgRoles_ = (lfgRoles_ & ~0x04) | (isHealer ? 0x04 : 0);
        ImGui::SameLine(220.0f);
        if (ImGui::Checkbox("DPS",    &isDps))    lfgRoles_ = (lfgRoles_ & ~0x08) | (isDps    ? 0x08 : 0);

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        bool hasRole = (lfgRoles_ & 0x0E) != 0;
        if (!hasRole) ImGui::BeginDisabled();

        ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.15f, 0.4f, 0.15f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.2f, 0.6f, 0.2f, 1.0f));
        if (ImGui::Button("Accept", ImVec2(140.0f, 28.0f))) {
            gameHandler.lfgSetRoles(lfgRoles_);
        }
        ImGui::PopStyleColor(2);

        if (!hasRole) ImGui::EndDisabled();

        ImGui::SameLine();

        ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.4f, 0.15f, 0.15f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.6f, 0.2f, 0.2f, 1.0f));
        if (ImGui::Button("Leave Queue", ImVec2(140.0f, 28.0f))) {
            gameHandler.lfgLeave();
        }
        ImGui::PopStyleColor(2);
    }
    ImGui::End();

    ImGui::PopStyleVar();
    ImGui::PopStyleColor(3);
}

void DialogManager::renderResurrectDialog(game::GameHandler& gameHandler) {
    if (!gameHandler.showResurrectDialog()) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    float dlgW = 300.0f;
    float dlgH = 110.0f;
    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - dlgW / 2, screenH * 0.3f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(dlgW, dlgH), ImGuiCond_Always);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 8.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.15f, 0.95f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.4f, 0.4f, 0.8f, 1.0f));

    if (ImGui::Begin("##ResurrectDialog", nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar)) {

        ImGui::Spacing();
        const std::string& casterName = gameHandler.getResurrectCasterName();
        std::string text = casterName.empty()
            ? "Return to life?"
            : casterName + " wishes to resurrect you.";
        float textW = ImGui::CalcTextSize(text.c_str()).x;
        ImGui::SetCursorPosX(std::max(4.0f, (dlgW - textW) / 2));
        ImGui::TextColored(ImVec4(0.8f, 0.9f, 1.0f, 1.0f), "%s", text.c_str());

        ImGui::Spacing();
        ImGui::Spacing();

        float btnW = 100.0f;
        float spacing = 20.0f;
        ImGui::SetCursorPosX((dlgW - btnW * 2 - spacing) / 2);

        ImGui::PushStyleColor(ImGuiCol_Button, colors::kBtnDkGreen);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, colors::kBtnDkGreenHover);
        if (ImGui::Button("Accept", ImVec2(btnW, 30))) {
            gameHandler.acceptResurrect();
        }
        ImGui::PopStyleColor(2);

        ImGui::SameLine(0, spacing);

        ImGui::PushStyleColor(ImGuiCol_Button, colors::kBtnDkRed);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, colors::kBtnDkRedHover);
        if (ImGui::Button("Decline", ImVec2(btnW, 30))) {
            gameHandler.declineResurrect();
        }
        ImGui::PopStyleColor(2);
    }
    ImGui::End();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar();
}

// ============================================================
// Talent Wipe Confirm Dialog
// ============================================================

void DialogManager::renderTalentWipeConfirmDialog(game::GameHandler& gameHandler) {
    if (!gameHandler.showTalentWipeConfirmDialog()) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    float dlgW = 340.0f;
    float dlgH = 130.0f;
    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - dlgW / 2, screenH * 0.3f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(dlgW, dlgH), ImGuiCond_Always);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 8.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.15f, 0.95f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.8f, 0.7f, 0.2f, 1.0f));

    if (ImGui::Begin("##TalentWipeDialog", nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar)) {

        ImGui::Spacing();
        uint32_t cost = gameHandler.getTalentWipeCost();
        const std::string costStr = game::formatCopperPrice(cost);

        std::string text = "Reset your talents for ";
        text += costStr;
        text += "?";
        float textW = ImGui::CalcTextSize(text.c_str()).x;
        ImGui::SetCursorPosX(std::max(4.0f, (dlgW - textW) / 2));
        ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.4f, 1.0f), "%s", text.c_str());

        ImGui::Spacing();
        ImGui::SetCursorPosX(8.0f);
        ImGui::TextDisabled("All talent points will be refunded.");
        ImGui::Spacing();

        float btnW = 110.0f;
        float spacing = 20.0f;
        ImGui::SetCursorPosX((dlgW - btnW * 2 - spacing) / 2);

        ImGui::PushStyleColor(ImGuiCol_Button, colors::kBtnDkGreen);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, colors::kBtnDkGreenHover);
        if (ImGui::Button("Confirm", ImVec2(btnW, 30))) {
            gameHandler.confirmTalentWipe();
        }
        ImGui::PopStyleColor(2);

        ImGui::SameLine(0, spacing);

        ImGui::PushStyleColor(ImGuiCol_Button, colors::kBtnDkRed);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, colors::kBtnDkRedHover);
        if (ImGui::Button("Cancel", ImVec2(btnW, 30))) {
            gameHandler.cancelTalentWipe();
        }
        ImGui::PopStyleColor(2);
    }
    ImGui::End();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar();
}

void DialogManager::renderPetUnlearnConfirmDialog(game::GameHandler& gameHandler) {
    if (!gameHandler.showPetUnlearnDialog()) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    float dlgW = 340.0f;
    float dlgH = 130.0f;
    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - dlgW / 2, screenH * 0.3f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(dlgW, dlgH), ImGuiCond_Always);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 8.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.15f, 0.95f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.8f, 0.7f, 0.2f, 1.0f));

    if (ImGui::Begin("##PetUnlearnDialog", nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar)) {

        ImGui::Spacing();
        uint32_t cost = gameHandler.getPetUnlearnCost();
        const std::string costStr = game::formatCopperPrice(cost);

        std::string text = std::string("Reset your pet's talents for ") + costStr + "?";
        float textW = ImGui::CalcTextSize(text.c_str()).x;
        ImGui::SetCursorPosX(std::max(4.0f, (dlgW - textW) / 2));
        ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.4f, 1.0f), "%s", text.c_str());

        ImGui::Spacing();
        ImGui::SetCursorPosX(8.0f);
        ImGui::TextDisabled("All pet talent points will be refunded.");
        ImGui::Spacing();

        float btnW = 110.0f;
        float spacing = 20.0f;
        ImGui::SetCursorPosX((dlgW - btnW * 2 - spacing) / 2);

        ImGui::PushStyleColor(ImGuiCol_Button, colors::kBtnDkGreen);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, colors::kBtnDkGreenHover);
        if (ImGui::Button("Confirm##petunlearn", ImVec2(btnW, 30))) {
            gameHandler.confirmPetUnlearn();
        }
        ImGui::PopStyleColor(2);

        ImGui::SameLine(0, spacing);

        ImGui::PushStyleColor(ImGuiCol_Button, colors::kBtnDkRed);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, colors::kBtnDkRedHover);
        if (ImGui::Button("Cancel##petunlearn", ImVec2(btnW, 30))) {
            gameHandler.cancelPetUnlearn();
        }
        ImGui::PopStyleColor(2);
    }
    ImGui::End();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar();
}

} // namespace ui
} // namespace wowee
