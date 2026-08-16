// ============================================================
// WindowManager - extracted from GameScreen
// Owns all NPC interaction windows, popup dialogs, etc.
// ============================================================
#include "core/local_time.hpp"
#include "ui/window_manager.hpp"
#include "ui/ui_texture_load.hpp"
#include "game/item_text.hpp"
#include "ui/framexml_takeover.hpp"
#include "ui/ui_upload_budget.hpp"
#include "game/inventory_slots.hpp"
#include "ui/chat_panel.hpp"
#include "ui/chat/chat_utils.hpp"
#include "ui/settings_panel.hpp"
#include "ui/spellbook_screen.hpp"
#include "ui/inventory_screen.hpp"
#include "ui/ui_colors.hpp"
#include "core/application.hpp"
#include "core/logger.hpp"
#include "rendering/renderer.hpp"
#include "rendering/character_preview.hpp"
#include "rendering/vk_context.hpp"
#include "core/window.hpp"
#include "game/game_handler.hpp"
#include "game/auction_filters.hpp"
#include "game/packed_time.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_layout.hpp"
#include "audio/audio_coordinator.hpp"
#include "audio/ui_sound_manager.hpp"
#include "audio/music_manager.hpp"
#include "pipeline/spell_icon_paths.hpp"
#include <imgui.h>
#include <imgui_internal.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <string>
#include <fstream>

namespace {
    using namespace wowee::ui::colors;

    // Abbreviated month names (indexed 0-11)
    constexpr const char* kMonthAbbrev[12] = {
        "Jan","Feb","Mar","Apr","May","Jun",
        "Jul","Aug","Sep","Oct","Nov","Dec"
    };

    constexpr auto& kColorRed         = kRed;
    constexpr auto& kColorGreen       = kGreen;
    constexpr auto& kColorYellow      = kYellow;
    constexpr auto& kColorGray        = kGray;
    constexpr auto& kColorDarkGray    = kDarkGray;

    // Common ImGui window flags for popup dialogs
    const ImGuiWindowFlags kDialogFlags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize;

    // Total count of an item across the backpack and equipped bags.
    uint32_t countItemInInventory(const wowee::game::Inventory& inv, uint32_t itemId) {
        return inv.countItem(itemId);
    }

} // anonymous namespace

namespace wowee {
namespace ui {

WindowManager::~WindowManager() = default;

void WindowManager::renderLootWindow(game::GameHandler& gameHandler,
                              InventoryScreen& inventoryScreen,
                              ChatPanel& chatPanel) {
    if (!gameHandler.isLootWindowOpen()) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 150, 200), ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(300, 0), ImGuiCond_Always);

    bool open = true;
    if (ImGui::Begin("Loot", &open, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize)) {
        const auto& loot = gameHandler.getCurrentLoot();

        // Gold (auto-looted on open; shown for feedback)
        if (loot.gold > 0) {
            ImGui::TextDisabled("Gold:");
            ImGui::SameLine(0, 4);
            renderCoinsText(loot.getGold(), loot.getSilver(), loot.getCopper());
            ImGui::Separator();
        }

        // Items with icons and labels
        constexpr float iconSize = 32.0f;
        int lootSlotClicked = -1;  // defer loot pickup to avoid iterator invalidation
        for (const auto& item : loot.items) {
            ImGui::PushID(item.slotIndex);

            // Get item info for name and quality
            const auto* info = gameHandler.getItemInfo(item.itemId);
            std::string itemName;
            game::ItemQuality quality = game::ItemQuality::COMMON;
            if (info && !info->name.empty()) {
                itemName = info->name;
                quality = static_cast<game::ItemQuality>(info->quality);
            } else {
                itemName = "Item #" + std::to_string(item.itemId);
            }
            ImVec4 qColor = InventoryScreen::getQualityColor(quality);
            bool startsQuest = (info && info->startQuestId != 0);

            // Get item icon
            uint32_t displayId = item.displayInfoId;
            if (displayId == 0 && info) displayId = info->displayInfoId;
            VkDescriptorSet iconTex = inventoryScreen.getItemIcon(displayId);

            ImVec2 cursor = ImGui::GetCursorScreenPos();
            float rowH = std::max(iconSize, ImGui::GetTextLineHeight() * 2.0f);

            // Invisible selectable for click handling
            if (ImGui::Selectable("##loot", false, 0, ImVec2(0, rowH))) {
                if (ImGui::GetIO().KeyShift && info && !info->name.empty()) {
                    // Shift-click: insert item link into chat
                    std::string link = game::itemChatLink(info->entry, info->quality, info->name);
                    chatPanel.insertChatLink(link);
                } else {
                    lootSlotClicked = item.slotIndex;
                }
            }
            if (ImGui::IsItemClicked(ImGuiMouseButton_Right)) {
                lootSlotClicked = item.slotIndex;
            }
            bool hovered = ImGui::IsItemHovered();

            // Show item tooltip on hover
            if (hovered && info && info->valid) {
                inventoryScreen.renderItemTooltip(*info);
            } else if (hovered && info && !info->name.empty()) {
                // Item info received but not yet fully valid - show name at minimum
                ImGui::SetTooltip("%s", info->name.c_str());
            }

            ImDrawList* drawList = ImGui::GetWindowDrawList();

            // Draw hover highlight
            if (hovered) {
                drawList->AddRectFilled(cursor,
                    ImVec2(cursor.x + ImGui::GetContentRegionAvail().x + iconSize + 8.0f,
                           cursor.y + rowH),
                    IM_COL32(255, 255, 255, 30));
            }

            // Draw icon
            if (iconTex) {
                drawList->AddImage((ImTextureID)(uintptr_t)iconTex,
                    cursor, ImVec2(cursor.x + iconSize, cursor.y + iconSize));
                drawList->AddRect(cursor, ImVec2(cursor.x + iconSize, cursor.y + iconSize),
                    ImGui::ColorConvertFloat4ToU32(qColor));
            } else {
                drawList->AddRectFilled(cursor,
                    ImVec2(cursor.x + iconSize, cursor.y + iconSize),
                    IM_COL32(40, 40, 50, 200));
                drawList->AddRect(cursor, ImVec2(cursor.x + iconSize, cursor.y + iconSize),
                    IM_COL32(80, 80, 80, 200));
            }
            // Quest-starter: gold outer glow border + "!" badge on top-right corner
            if (startsQuest) {
                drawList->AddRect(ImVec2(cursor.x - 2.0f, cursor.y - 2.0f),
                    ImVec2(cursor.x + iconSize + 2.0f, cursor.y + iconSize + 2.0f),
                    IM_COL32(255, 210, 0, 210), 0.0f, 0, 2.0f);
                drawList->AddText(ImVec2(cursor.x + iconSize - 10.0f, cursor.y + 1.0f),
                    IM_COL32(255, 210, 0, 255), "!");
            }

            // Draw item name
            float textX = cursor.x + iconSize + 6.0f;
            float textY = cursor.y + 2.0f;
            drawList->AddText(ImVec2(textX, textY),
                ImGui::ColorConvertFloat4ToU32(qColor), itemName.c_str());

            // Draw count or "Begins a Quest" label on second line
            float secondLineY = textY + ImGui::GetTextLineHeight();
            if (startsQuest) {
                drawList->AddText(ImVec2(textX, secondLineY),
                    IM_COL32(255, 210, 0, 255), "Begins a Quest");
            } else if (item.count > 1) {
                char countStr[32];
                snprintf(countStr, sizeof(countStr), "x%u", item.count);
                drawList->AddText(ImVec2(textX, secondLineY), IM_COL32(200, 200, 200, 220), countStr);
            }

            ImGui::PopID();
        }

        // Process deferred loot pickup (after loop to avoid iterator invalidation)
        if (lootSlotClicked >= 0) {
            if (gameHandler.hasMasterLootCandidates()) {
                // Master looter: open popup to choose recipient
                char popupId[32];
                snprintf(popupId, sizeof(popupId), "##MLGive%d", lootSlotClicked);
                ImGui::OpenPopup(popupId);
            } else {
                // Taken as already confirmed: this window has no bind warning
                // to show. The handler raises LOOT_BIND_CONFIRM for a
                // bind-on-pickup item and holds the request until it is
                // answered, and there is nothing here to answer it - the click
                // would do nothing at all.
                //
                // So the warning is on the interface's path only, which is the
                // one being moved to. Closing that gap means giving this window
                // a confirm of its own, the way it already has one for equip.
                gameHandler.lootItem(static_cast<uint8_t>(lootSlotClicked), true);
            }
        }

        // Master loot "Give to" popups
        if (gameHandler.hasMasterLootCandidates()) {
            for (const auto& item : loot.items) {
                char popupId[32];
                snprintf(popupId, sizeof(popupId), "##MLGive%d", item.slotIndex);
                if (ImGui::BeginPopup(popupId)) {
                    ImGui::TextDisabled("Give to:");
                    ImGui::Separator();
                    const auto& candidates = gameHandler.getMasterLootCandidates();
                    for (uint64_t candidateGuid : candidates) {
                        auto entity = gameHandler.getEntityManager().getEntity(candidateGuid);
                        auto* unit = (entity && entity->isUnit()) ? static_cast<game::Unit*>(entity.get()) : nullptr;
                        const char* cName = unit ? unit->getName().c_str() : nullptr;
                        char nameBuf[64];
                        if (!cName || cName[0] == '\0') {
                            snprintf(nameBuf, sizeof(nameBuf), "Player 0x%llx",
                                     static_cast<unsigned long long>(candidateGuid));
                            cName = nameBuf;
                        }
                        if (ImGui::MenuItem(cName)) {
                            gameHandler.lootMasterGive(item.slotIndex, candidateGuid);
                            ImGui::CloseCurrentPopup();
                        }
                    }
                    ImGui::EndPopup();
                }
            }
        }

        if (loot.items.empty() && loot.gold == 0) {
            gameHandler.closeLoot();
        }

        ImGui::Spacing();
        bool hasItems = !loot.items.empty();
        if (hasItems) {
            if (ImGui::Button("Loot All", ImVec2(-1, 0))) {
                for (const auto& item : loot.items) {
                    gameHandler.lootItem(item.slotIndex, true);
                }
            }
            ImGui::Spacing();
        }
        if (ImGui::Button("Close", ImVec2(-1, 0))) {
            gameHandler.closeLoot();
        }
    }
    ImGui::End();

    if (!open) {
        gameHandler.closeLoot();
    }
}





std::string WindowManager::formatExtendedCost(uint32_t extendedCostId, game::GameHandler& gameHandler) {
    // Read from the game handler, which is where this lives now: the original
    // interface asks the same question through GetMerchantItemCostItem and
    // cannot reach in here.
    const auto* entry = gameHandler.getExtendedCost(extendedCostId);
    if (!entry) return "[Tokens]";
    const auto& e = *entry;
    std::string result;
    if (e.honorPoints > 0) {
        result += std::to_string(e.honorPoints) + " Honor";
    }
    if (e.arenaPoints > 0) {
        if (!result.empty()) result += ", ";
        result += std::to_string(e.arenaPoints) + " Arena";
    }
    for (int j = 0; j < 5; ++j) {
        if (e.itemId[j] == 0 || e.itemCount[j] == 0) continue;
        if (!result.empty()) result += ", ";
        gameHandler.ensureItemInfo(e.itemId[j]);  // query if not cached
        const auto* itemInfo = gameHandler.getItemInfo(e.itemId[j]);
        if (itemInfo && itemInfo->valid && !itemInfo->name.empty()) {
            result += std::to_string(e.itemCount[j]) + "x " + itemInfo->name;
        } else {
            result += std::to_string(e.itemCount[j]) + "x Item#" + std::to_string(e.itemId[j]);
        }
    }
    return result.empty() ? "[Tokens]" : result;
}

void WindowManager::renderVendorWindow(game::GameHandler& gameHandler,
                               InventoryScreen& inventoryScreen,
                               ChatPanel& chatPanel) {
    if (!gameHandler.isVendorWindowOpen()) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 200, 100), ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(450, 400), ImGuiCond_Appearing);

    bool open = true;
    if (ImGui::Begin("Vendor", &open)) {
        const auto& vendor = gameHandler.getVendorItems();

        // Show player money
        uint64_t money = gameHandler.getMoneyCopper();
        ImGui::TextDisabled("Your money:"); ImGui::SameLine(0, 4);
        renderCoinsFromCopper(money);

        if (vendor.canRepair) {
            uint32_t repairCost = gameHandler.estimateRepairAllCost();
            ImGui::SameLine();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 8.0f);
            if (ImGui::SmallButton("Repair All")) {
                gameHandler.repairAll(vendor.vendorGuid, false);
            }
            if (repairCost > 0) {
                ImGui::SameLine(0, 4);
                renderCoinsFromCopper(repairCost);
            }
            if (ImGui::IsItemHovered()) {
                // Show durability summary of all equipment
                const auto& inv = gameHandler.getInventory();
                int damagedCount = 0;
                int brokenCount = 0;
                for (int s = 0; s < static_cast<int>(game::EquipSlot::BAG1); s++) {
                    const auto& slot = inv.getEquipSlot(static_cast<game::EquipSlot>(s));
                    if (slot.empty() || slot.item.maxDurability == 0) continue;
                    if (slot.item.curDurability == 0) brokenCount++;
                    else if (slot.item.curDurability < slot.item.maxDurability) damagedCount++;
                }
                if (brokenCount > 0)
                    ImGui::SetTooltip("Repair all equipped items\n%d damaged, %d broken", damagedCount, brokenCount);
                else if (damagedCount > 0)
                    ImGui::SetTooltip("Repair all equipped items\n%d item%s need repair", damagedCount, damagedCount > 1 ? "s" : "");
                else
                    ImGui::SetTooltip("All equipment is in good condition");
            }
            if (gameHandler.isInGuild()) {
                ImGui::SameLine();
                if (ImGui::SmallButton("Repair (Guild)")) {
                    gameHandler.repairAll(vendor.vendorGuid, true);
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Repair all items using guild bank funds");
                }
            }
        }
        ImGui::Separator();

        ImGui::TextColored(ui::colors::kLightGray, "Right-click bag items to sell");

        // Count grey (POOR quality) sellable items across backpack and bags
        const auto& inv = gameHandler.getInventory();
        int junkCount = 0;
        for (int i = 0; i < inv.getBackpackSize(); ++i) {
            const auto& sl = inv.getBackpackSlot(i);
            if (!sl.empty() && sl.item.quality == game::ItemQuality::POOR && sl.item.sellPrice > 0)
                ++junkCount;
        }
        for (int b = 0; b < game::Inventory::NUM_BAG_SLOTS; ++b) {
            for (int s = 0; s < inv.getBagSize(b); ++s) {
                const auto& sl = inv.getBagSlot(b, s);
                if (!sl.empty() && sl.item.quality == game::ItemQuality::POOR && sl.item.sellPrice > 0)
                    ++junkCount;
            }
        }
        if (junkCount > 0) {
            char junkLabel[64];
            snprintf(junkLabel, sizeof(junkLabel), "Sell All Junk (%d item%s)",
                     junkCount, junkCount == 1 ? "" : "s");
            if (ImGui::Button(junkLabel, ImVec2(-1, 0))) {
                for (int i = 0; i < inv.getBackpackSize(); ++i) {
                    const auto& sl = inv.getBackpackSlot(i);
                    if (!sl.empty() && sl.item.quality == game::ItemQuality::POOR && sl.item.sellPrice > 0)
                        gameHandler.sellItemBySlot(i);
                }
                for (int b = 0; b < game::Inventory::NUM_BAG_SLOTS; ++b) {
                    for (int s = 0; s < inv.getBagSize(b); ++s) {
                        const auto& sl = inv.getBagSlot(b, s);
                        if (!sl.empty() && sl.item.quality == game::ItemQuality::POOR && sl.item.sellPrice > 0)
                            gameHandler.sellItemInBag(b, s);
                    }
                }
            }
        }
        ImGui::Separator();

        const auto& buyback = gameHandler.getBuybackItems();
        if (!buyback.empty()) {
            ImGui::TextColored(ui::colors::kTooltipGold, "Buy Back");
            if (ImGui::BeginTable("BuybackTable", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                ImGui::TableSetupColumn("##icon", ImGuiTableColumnFlags_WidthFixed, 22.0f);
                ImGui::TableSetupColumn("Item", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("Price", ImGuiTableColumnFlags_WidthFixed, 110.0f);
                ImGui::TableSetupColumn("Buy", ImGuiTableColumnFlags_WidthFixed, 62.0f);
                ImGui::TableHeadersRow();
                // Show all buyback items (oldest sold first, matching server slot order)
                for (int i = 0; i < static_cast<int>(buyback.size()); ++i) {
                    const auto& entry = buyback[i];
                    gameHandler.ensureItemInfo(entry.item.itemId);
                    auto* bbInfo = gameHandler.getItemInfo(entry.item.itemId);
                    uint32_t sellPrice = entry.item.sellPrice;
                    if (sellPrice == 0) {
                        if (bbInfo && bbInfo->valid) sellPrice = bbInfo->sellPrice;
                    }
                    uint64_t price = static_cast<uint64_t>(sellPrice) *
                                     static_cast<uint64_t>(entry.count > 0 ? entry.count : 1);
                    const auto coins = game::splitCopper(price);
                    const uint32_t g = coins.gold;
                    const uint32_t s = coins.silver;
                    const uint32_t c = coins.copper;
                    bool canAfford = money >= price;
                    const bool slotReady = entry.wireSlot >= 74 && entry.wireSlot < 86;

                    ImGui::TableNextRow();
                    ImGui::PushID(8000 + i);
                    ImGui::TableSetColumnIndex(0);
                    {
                        uint32_t dispId = entry.item.displayInfoId;
                        if (bbInfo && bbInfo->valid && bbInfo->displayInfoId != 0) dispId = bbInfo->displayInfoId;
                        if (dispId != 0) {
                            VkDescriptorSet iconTex = inventoryScreen.getItemIcon(dispId);
                            if (iconTex) ImGui::Image((ImTextureID)(uintptr_t)iconTex, ImVec2(18, 18));
                        }
                    }
                    ImGui::TableSetColumnIndex(1);
                    game::ItemQuality bbQuality = entry.item.quality;
                    if (bbInfo && bbInfo->valid) bbQuality = static_cast<game::ItemQuality>(bbInfo->quality);
                    ImVec4 bbQc = InventoryScreen::getQualityColor(bbQuality);
                    const char* name = entry.item.name.empty() ? "Unknown Item" : entry.item.name.c_str();
                    if (entry.count > 1) {
                        ImGui::TextColored(bbQc, "%s x%u", name, entry.count);
                    } else {
                        ImGui::TextColored(bbQc, "%s", name);
                    }
                    if (ImGui::IsItemHovered() && bbInfo && bbInfo->valid)
                        inventoryScreen.renderItemTooltip(*bbInfo);
                    ImGui::TableSetColumnIndex(2);
                    if (canAfford) {
                        renderCoinsText(g, s, c);
                    } else {
                        ImGui::TextColored(kColorRed, "%s",
                                           game::formatCoinPrice(coins).c_str());
                    }
                    ImGui::TableSetColumnIndex(3);
                    if (!canAfford || !slotReady) ImGui::BeginDisabled();
                    char bbLabel[32];
                    snprintf(bbLabel, sizeof(bbLabel), "Buy Back##bb%d", i);
                    if (ImGui::SmallButton(bbLabel)) {
                        gameHandler.buyBackItem(static_cast<uint32_t>(i));
                    }
                    if (!canAfford || !slotReady) ImGui::EndDisabled();
                    if (!slotReady && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                        ImGui::SetTooltip("Waiting for server buyback slot");
                    }
                    ImGui::PopID();
                }
                ImGui::EndTable();
            }
            ImGui::Separator();
        }

        if (vendor.items.empty()) {
            ImGui::TextDisabled("This vendor has nothing for sale.");
        } else {
            // Search + quantity controls on one row
            ImGui::SetNextItemWidth(200.0f);
            ImGui::InputTextWithHint("##VendorSearch", "Search...", vendorSearchFilter_, sizeof(vendorSearchFilter_));
            ImGui::SameLine();
            ImGui::Text("Qty:");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(60.0f);
            static int vendorBuyQty = 1;
            ImGui::InputInt("##VendorQty", &vendorBuyQty, 1, 5);
            if (vendorBuyQty < 1) vendorBuyQty = 1;
            if (vendorBuyQty > 99) vendorBuyQty = 99;
            ImGui::Spacing();

            if (ImGui::BeginTable("VendorTable", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY)) {
                ImGui::TableSetupColumn("##icon", ImGuiTableColumnFlags_WidthFixed, 22.0f);
                ImGui::TableSetupColumn("Item", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("Price", ImGuiTableColumnFlags_WidthFixed, 120.0f);
                ImGui::TableSetupColumn("Stock", ImGuiTableColumnFlags_WidthFixed, 60.0f);
                ImGui::TableSetupColumn("Buy", ImGuiTableColumnFlags_WidthFixed, 50.0f);
                ImGui::TableHeadersRow();

                std::string vendorFilter(vendorSearchFilter_);
                // Lowercase filter for case-insensitive match
                for (char& c : vendorFilter) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

                for (int vi = 0; vi < static_cast<int>(vendor.items.size()); ++vi) {
                    const auto& item = vendor.items[vi];

                    // Proactively ensure vendor item info is loaded
                    gameHandler.ensureItemInfo(item.itemId);
                    auto* info = gameHandler.getItemInfo(item.itemId);

                    // Apply search filter
                    if (!vendorFilter.empty()) {
                        std::string nameLC = info && info->valid ? info->name : ("Item " + std::to_string(item.itemId));
                        for (char& c : nameLC) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                        if (nameLC.find(vendorFilter) == std::string::npos) {
                            ImGui::PushID(vi);
                            ImGui::PopID();
                            continue;
                        }
                    }

                    ImGui::TableNextRow();
                    ImGui::PushID(vi);

                    // Icon column
                    ImGui::TableSetColumnIndex(0);
                    {
                        uint32_t dispId = item.displayInfoId;
                        if (info && info->valid && info->displayInfoId != 0) dispId = info->displayInfoId;
                        if (dispId != 0) {
                            VkDescriptorSet iconTex = inventoryScreen.getItemIcon(dispId);
                            if (iconTex) ImGui::Image((ImTextureID)(uintptr_t)iconTex, ImVec2(18, 18));
                        }
                    }

                    // Name column
                    ImGui::TableSetColumnIndex(1);
                    if (info && info->valid) {
                        ImVec4 qc = InventoryScreen::getQualityColor(static_cast<game::ItemQuality>(info->quality));
                        ImGui::TextColored(qc, "%s", info->name.c_str());
                        if (ImGui::IsItemHovered()) {
                            inventoryScreen.renderItemTooltip(*info, &gameHandler.getInventory());
                        }
                        // Shift-click: insert item link into chat
                        if (ImGui::IsItemClicked() && ImGui::GetIO().KeyShift) {
                            std::string link = game::itemChatLink(info->entry, info->quality, info->name);
                            chatPanel.insertChatLink(link);
                        }
                    } else {
                        ImGui::Text("Item %u", item.itemId);
                    }

                    ImGui::TableSetColumnIndex(2);
                    if (item.buyPrice == 0 && item.extendedCost != 0) {
                        // Token-only item - show detailed cost from ItemExtendedCost.dbc
                        std::string costStr = formatExtendedCost(item.extendedCost, gameHandler);
                        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "%s", costStr.c_str());
                    } else {
                        const auto coins = game::splitCopper(item.buyPrice);
                        const uint32_t g = coins.gold;
                        const uint32_t s = coins.silver;
                        const uint32_t c = coins.copper;
                        bool canAfford = money >= item.buyPrice;
                        if (canAfford) {
                            renderCoinsText(g, s, c);
                        } else {
                            ImGui::TextColored(kColorRed, "%s",
                                           game::formatCoinPrice(coins).c_str());
                        }
                        // Show additional token cost if both gold and tokens are required
                        if (item.extendedCost != 0) {
                            std::string costStr = formatExtendedCost(item.extendedCost, gameHandler);
                            if (costStr != "[Tokens]") {
                                ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 0.8f), "+ %s", costStr.c_str());
                            }
                        }
                    }

                    ImGui::TableSetColumnIndex(3);
                    if (item.maxCount < 0) {
                        ImGui::TextDisabled("Inf");
                    } else if (item.maxCount == 0) {
                        ImGui::TextColored(kColorRed, "Out");
                    } else if (item.maxCount <= 5) {
                        ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.1f, 1.0f), "%d", item.maxCount);
                    } else {
                        ImGui::Text("%d", item.maxCount);
                    }

                    ImGui::TableSetColumnIndex(4);
                    bool outOfStock = (item.maxCount == 0);
                    if (outOfStock) ImGui::BeginDisabled();
                    std::string buyBtnId = "Buy##vendor_" + std::to_string(vi);
                    if (ImGui::SmallButton(buyBtnId.c_str())) {
                        int qty = vendorBuyQty;
                        if (item.maxCount > 0 && qty > item.maxCount) qty = item.maxCount;
                        uint32_t totalCost = item.buyPrice * static_cast<uint32_t>(qty);
                        if (totalCost >= 10000) { // >= 1 gold: confirm
                            vendorConfirmOpen_ = true;
                            vendorConfirmGuid_ = vendor.vendorGuid;
                            vendorConfirmItemId_ = item.itemId;
                            vendorConfirmSlot_ = item.slot;
                            vendorConfirmQty_ = static_cast<uint32_t>(qty);
                            vendorConfirmPrice_ = totalCost;
                            vendorConfirmItemName_ = (info && info->valid) ? info->name : "Item";
                        } else {
                            gameHandler.buyItem(vendor.vendorGuid, item.itemId, item.slot,
                                                static_cast<uint32_t>(qty));
                        }
                    }
                    if (outOfStock) ImGui::EndDisabled();

                    ImGui::PopID();
                }

                ImGui::EndTable();
            }
        }
    }
    ImGui::End();

    if (!open) {
        gameHandler.closeVendor();
    }

    // Vendor purchase confirmation popup for expensive items
    if (vendorConfirmOpen_) {
        ImGui::OpenPopup("Confirm Purchase##vendor");
        vendorConfirmOpen_ = false;
    }
    if (ImGui::BeginPopupModal("Confirm Purchase##vendor", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove)) {
        ImGui::Text("Buy %s", vendorConfirmItemName_.c_str());
        if (vendorConfirmQty_ > 1)
            ImGui::Text("Quantity: %u", vendorConfirmQty_);
        const auto coins = game::splitCopper(vendorConfirmPrice_);
        ImGui::Text("Cost: %s", game::formatCoinPrice(coins).c_str());
        ImGui::Spacing();
        if (ImGui::Button("Buy", ImVec2(80, 0))) {
            gameHandler.buyItem(vendorConfirmGuid_, vendorConfirmItemId_,
                                vendorConfirmSlot_, vendorConfirmQty_);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(80, 0))) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

void WindowManager::renderTrainerWindow(game::GameHandler& gameHandler,
                                SpellIconFn getSpellIcon,
                                InventoryScreen& inventoryScreen) {
    if (!gameHandler.isTrainerWindowOpen()) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    auto* assetMgr = services_.assetManager;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 225, 100), ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(500, 450), ImGuiCond_Appearing);

    bool open = true;
    if (ImGui::Begin("Trainer", &open)) {
        // If user clicked window close, short-circuit before rendering large trainer tables.
        if (!open) {
            ImGui::End();
            gameHandler.closeTrainer();
            return;
        }

        const auto& trainer = gameHandler.getTrainerSpells();
        const bool isProfessionTrainer = (trainer.trainerType == 2);

        // NPC name
        auto npcEntity = gameHandler.getEntityManager().getEntity(trainer.trainerGuid);
        if (npcEntity && npcEntity->getType() == game::ObjectType::UNIT) {
            auto unit = std::static_pointer_cast<game::Unit>(npcEntity);
            if (!unit->getName().empty()) {
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), "%s", unit->getName().c_str());
            }
        }

        // Greeting
        if (!trainer.greeting.empty()) {
            ImGui::TextWrapped("%s", trainer.greeting.c_str());
        }
        ImGui::Separator();

        // Player money
        uint64_t money = gameHandler.getMoneyCopper();
        ImGui::TextDisabled("Your money:"); ImGui::SameLine(0, 4);
        renderCoinsFromCopper(money);

        // Filter controls
        static bool showUnavailable = false;
        ImGui::Checkbox("Show unavailable spells", &showUnavailable);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::InputTextWithHint("##TrainerSearch", "Search...", trainerSearchFilter_, sizeof(trainerSearchFilter_));
        ImGui::Separator();

        if (trainer.spells.empty()) {
            ImGui::TextDisabled("This trainer has nothing to teach you.");
        } else {
            // Known spells for checking
            const auto& knownSpells = gameHandler.getKnownSpells();
            auto isKnown = [&](uint32_t id) {
                if (id == 0) return true;
                // Check if spell is in knownSpells list
                bool found = knownSpells.count(id);
                if (found) return true;

                // Also check if spell is in trainer list with state=2 (explicitly known)
                // state=0 means unavailable (could be no prereqs, wrong level, etc.) - don't count as known
                for (const auto& ts : trainer.spells) {
                    if (ts.spellId == id && ts.state == 2) {
                        return true;
                    }
                }
                return false;
            };
            uint32_t playerLevel = gameHandler.getPlayerLevel();

            // A recipe/spell may require a minimum skill value (e.g. Cooking 50).
            // The server encodes this in reqSkill/reqSkillValue and reports the
            // spell as unavailable (state=1) until the value is reached. Check it
            // against the player's own skills so we don't promote a skill-gated
            // recipe to a green "Train" button the server will reject.
            auto skillMet = [&](const game::TrainerSpell& spell) {
                if (spell.reqSkill == 0 || spell.reqSkillValue == 0) return true;
                const auto& skills = gameHandler.getPlayerSkills();
                auto it = skills.find(spell.reqSkill);
                return it != skills.end() && it->second.effectiveValue() >= spell.reqSkillValue;
            };

            // Renders spell rows into the current table
            auto renderSpellRows = [&](const std::vector<const game::TrainerSpell*>& spells) {
                for (const auto* spell : spells) {
                    // Check prerequisites client-side first
                    bool prereq1Met = isKnown(spell->chainNode1);
                    bool prereq2Met = isKnown(spell->chainNode2);
                    bool prereq3Met = isKnown(spell->chainNode3);
                    bool prereqsMet = prereq1Met && prereq2Met && prereq3Met;
                    bool levelMet = (spell->reqLevel == 0 || playerLevel >= spell->reqLevel);
                    bool alreadyKnown = isKnown(spell->spellId);

                    // Dynamically determine effective state based on current prerequisites
                    // Server sends state, but we override if prerequisites are now met
                    bool reqSkillMet = skillMet(*spell);
                    uint8_t effectiveState = spell->state;
                    if (spell->state == 1 && prereqsMet && levelMet && reqSkillMet) {
                        // Server said unavailable, but we now meet all requirements
                        effectiveState = 0;  // Treat as available
                    }

                    // Filter: skip unavailable spells if checkbox is unchecked
                    // Use effectiveState so spells with newly met prereqs aren't filtered
                    if (!showUnavailable && effectiveState == 1) {
                        continue;
                    }

                    // Apply text search filter
                    if (trainerSearchFilter_[0] != '\0') {
                        std::string trainerFilter(trainerSearchFilter_);
                        for (char& c : trainerFilter) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                        const std::string& spellName = gameHandler.getSpellName(spell->spellId);
                        std::string nameLC = spellName.empty() ? std::to_string(spell->spellId) : spellName;
                        for (char& c : nameLC) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                        if (nameLC.find(trainerFilter) == std::string::npos) {
                            ImGui::PushID(static_cast<int>(spell->spellId));
                            ImGui::PopID();
                            continue;
                        }
                    }

                    ImGui::TableNextRow();
                    ImGui::PushID(static_cast<int>(spell->spellId));

                    ImVec4 color;
                    const char* statusLabel;
                    // WotLK trainer states: 0=available, 1=unavailable, 2=known
                    if (effectiveState == 2 || alreadyKnown) {
                        color = colors::kQueueGreen;
                        statusLabel = "Known";
                    } else if (effectiveState == 0) {
                        color = ui::colors::kWhite;
                        statusLabel = "Available";
                    } else {
                        color = ImVec4(0.6f, 0.3f, 0.3f, 1.0f);
                        statusLabel = "Unavailable";
                    }

                    // Icon column - use item icon for crafting recipes, spell icon otherwise
                    ImGui::TableSetColumnIndex(0);
                    {
                        VkDescriptorSet icon = VK_NULL_HANDLE;
                        if (isProfessionTrainer) {
                            gameHandler.loadSpellNameCache();
                            auto cit = gameHandler.spellNameCacheRef().find(spell->spellId);
                            if (cit != gameHandler.spellNameCacheRef().end() && cit->second.createdItemId != 0) {
                                gameHandler.ensureItemInfo(cit->second.createdItemId);
                                const auto* itemInfo = gameHandler.getItemInfo(cit->second.createdItemId);
                                if (itemInfo && itemInfo->displayInfoId)
                                    icon = inventoryScreen.getItemIcon(itemInfo->displayInfoId);
                            }
                        }
                        if (!icon)
                            icon = getSpellIcon(spell->spellId, assetMgr);
                        if (icon) {
                            if (effectiveState == 1 && !alreadyKnown) {
                                ImGui::ImageWithBg((ImTextureID)(uintptr_t)icon, ImVec2(18, 18),
                                    ImVec2(0, 0), ImVec2(1, 1),
                                    ImVec4(0, 0, 0, 0), ImVec4(0.5f, 0.5f, 0.5f, 0.6f));
                            } else {
                                ImGui::Image((ImTextureID)(uintptr_t)icon, ImVec2(18, 18));
                            }
                        }
                    }

                    // Spell name
                    ImGui::TableSetColumnIndex(1);
                    const std::string& name = gameHandler.getSpellName(spell->spellId);
                    const std::string& rank = gameHandler.getSpellRank(spell->spellId);
                    if (!name.empty()) {
                        if (!rank.empty())
                            ImGui::TextColored(color, "%s (%s)", name.c_str(), rank.c_str());
                        else
                            ImGui::TextColored(color, "%s", name.c_str());
                    } else {
                        ImGui::TextColored(color, "Spell #%u", spell->spellId);
                    }

                    if (ImGui::IsItemHovered()) {
                        ImGui::BeginTooltip();
                        if (!name.empty()) {
                            ImGui::TextColored(kColorYellow, "%s", name.c_str());
                            if (!rank.empty()) ImGui::TextColored(kColorGray, "%s", rank.c_str());
                        }
                        const std::string& spDesc = gameHandler.getSpellDescription(spell->spellId);
                        if (!spDesc.empty()) {
                            std::string spText = gameHandler.formatSpellDescription(spell->spellId, spDesc);
                            ImGui::Spacing();
                            ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + 300.0f);
                            ImGui::TextWrapped("%s", spText.c_str());
                            ImGui::PopTextWrapPos();
                            ImGui::Spacing();
                        }
                        // Recipes: show the full item the recipe creates and the
                        // reagents it consumes, so the purchase is an informed one.
                        auto seIt = gameHandler.spellNameCacheRef().find(spell->spellId);
                        if (seIt != gameHandler.spellNameCacheRef().end()) {
                            const auto& se = seIt->second;
                            if (se.createdItemId != 0) {
                                gameHandler.ensureItemInfo(se.createdItemId);
                                const auto* prodInfo = gameHandler.getItemInfo(se.createdItemId);
                                ImGui::Separator();
                                ImGui::TextColored(kColorYellow, "Creates:");
                                if (prodInfo && prodInfo->valid) {
                                    ImGui::Indent(8.0f);
                                    inventoryScreen.renderItemTooltip(*prodInfo, &gameHandler.getInventory());
                                    ImGui::Unindent(8.0f);
                                } else {
                                    ImGui::Text("Item #%u", se.createdItemId);
                                }
                            }
                            bool tooltipHasReagents = false;
                            for (const auto& reagent : se.reagents) {
                                if (reagent.itemId != 0) { tooltipHasReagents = true; break; }
                            }
                            if (tooltipHasReagents) {
                                ImGui::Separator();
                                ImGui::TextDisabled("Reagents:");
                                for (const auto& reagent : se.reagents) {
                                    if (reagent.itemId == 0 || reagent.count == 0) continue;
                                    gameHandler.ensureItemInfo(reagent.itemId);
                                    const auto* rInfo = gameHandler.getItemInfo(reagent.itemId);
                                    uint32_t have = countItemInInventory(gameHandler.getInventory(), reagent.itemId);
                                    ImVec4 haveCol = have >= reagent.count
                                        ? colors::kLightGreen : ImVec4(1.0f, 0.6f, 0.6f, 1.0f);
                                    if (rInfo && !rInfo->name.empty())
                                        ImGui::TextColored(haveCol, "  %s (%u/%u)",
                                            rInfo->name.c_str(), have, reagent.count);
                                    else
                                        ImGui::TextColored(haveCol, "  Item #%u (%u/%u)",
                                            reagent.itemId, have, reagent.count);
                                }
                            }
                            ImGui::Spacing();
                        }
                        ImGui::TextDisabled("Status: %s", statusLabel);
                        if (spell->reqLevel > 0) {
                            ImVec4 lvlColor = levelMet ? ui::colors::kLightGray : kColorRed;
                            ImGui::TextColored(lvlColor, "Required Level: %u", spell->reqLevel);
                        }
                        if (spell->reqSkill > 0 && spell->reqSkillValue > 0) {
                            const auto& skills = gameHandler.getPlayerSkills();
                            auto skIt = skills.find(spell->reqSkill);
                            uint16_t curSkill = (skIt != skills.end()) ? skIt->second.effectiveValue() : 0;
                            const std::string& skName = gameHandler.getSkillName(spell->reqSkill);
                            ImVec4 skColor = reqSkillMet ? ui::colors::kLightGray : kColorRed;
                            if (!skName.empty())
                                ImGui::TextColored(skColor, "Requires %s %u (you have %u)",
                                    skName.c_str(), spell->reqSkillValue, curSkill);
                            else
                                ImGui::TextColored(skColor, "Required Skill: %u (need %u, have %u)",
                                    spell->reqSkill, spell->reqSkillValue, curSkill);
                        }
                        auto showPrereq = [&](uint32_t node) {
                            if (node == 0) return;
                            bool met = isKnown(node);
                            const std::string& pname = gameHandler.getSpellName(node);
                            ImVec4 pcolor = met ? colors::kQueueGreen : kColorRed;
                            if (!pname.empty())
                                ImGui::TextColored(pcolor, "Requires: %s%s", pname.c_str(), met ? " (known)" : "");
                            else
                                ImGui::TextColored(pcolor, "Requires: Spell #%u%s", node, met ? " (known)" : "");
                        };
                        showPrereq(spell->chainNode1);
                        showPrereq(spell->chainNode2);
                        showPrereq(spell->chainNode3);
                        ImGui::EndTooltip();
                    }

                    // Level
                    ImGui::TableSetColumnIndex(2);
                    ImGui::TextColored(color, "%u", spell->reqLevel);

                    // Cost
                    ImGui::TableSetColumnIndex(3);
                    if (spell->spellCost > 0) {
                        const auto coins = game::splitCopper(spell->spellCost);
                        const uint32_t g = coins.gold;
                        const uint32_t s = coins.silver;
                        const uint32_t c = coins.copper;
                        bool canAfford = money >= spell->spellCost;
                        if (canAfford) {
                            renderCoinsText(g, s, c);
                        } else {
                            ImGui::TextColored(kColorRed, "%s",
                                           game::formatCoinPrice(coins).c_str());
                        }
                    } else {
                        ImGui::TextColored(color, "Free");
                    }

                    // The server-computed trainer state is authoritative for level,
                    // skill, class, race, and prerequisite requirements.  Do not
                    // veto an available offer using the client's incomplete spell
                    // cache (weapon proficiencies in particular may not appear there).
                    ImGui::TableSetColumnIndex(4);
                    // Keep the local money check for immediate UI feedback.
                    bool canTrain = !alreadyKnown && effectiveState == 0
                                  && (money >= spell->spellCost);

                    // Debug logging for first 3 spells to see why buttons are disabled
                    static int logCount = 0;
                    static uint64_t lastTrainerGuid = 0;
                    if (trainer.trainerGuid != lastTrainerGuid) {
                        logCount = 0;
                        lastTrainerGuid = trainer.trainerGuid;
                    }
                    if (logCount < 3) {
                        LOG_INFO("Trainer button state: spellId=", spell->spellId,
                                " alreadyKnown=", alreadyKnown, " state=", static_cast<int>(spell->state),
                                " prereqsMet=", prereqsMet, " (", prereq1Met, ",", prereq2Met, ",", prereq3Met, ")",
                                " levelMet=", levelMet,
                                " reqLevel=", spell->reqLevel, " playerLevel=", playerLevel,
                                " chain1=", spell->chainNode1, " chain2=", spell->chainNode2, " chain3=", spell->chainNode3,
                                " canAfford=", (money >= spell->spellCost),
                                " canTrain=", canTrain);
                        logCount++;
                    }

                    bool isCraftRecipe = isProfessionTrainer && alreadyKnown
                                       && spell->profDialog != 0;
                    if (isCraftRecipe) {
                        // Profession trainer: known recipes show "Create" button to craft
                        bool isCasting = gameHandler.isCasting();
                        if (isCasting) ImGui::BeginDisabled();
                        if (ImGui::SmallButton("Create")) {
                            gameHandler.castSpell(spell->spellId, 0);
                        }
                        if (isCasting) ImGui::EndDisabled();
                    } else {
                        if (!canTrain) ImGui::BeginDisabled();
                        if (canTrain) {
                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.48f, 0.20f, 1.0f));
                            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.24f, 0.62f, 0.27f, 1.0f));
                            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.14f, 0.38f, 0.16f, 1.0f));
                        }
                        // Fill the action column so the enabled control has a clear,
                        // reliable hit target instead of SmallButton's text-sized box.
                        const char* actionLabel = alreadyKnown ? "Known" : "Train";
                        if (ImGui::Button(actionLabel, ImVec2(-1.0f, 0.0f))) {
                            gameHandler.trainSpell(spell->spellId);
                        }
                        if (canTrain) ImGui::PopStyleColor(3);
                        if (!canTrain) ImGui::EndDisabled();
                    }

                    ImGui::PopID();
                }
            };

            auto renderSpellTable = [&](const char* tableId, const std::vector<const game::TrainerSpell*>& spells) {
                if (ImGui::BeginTable(tableId, 5,
                        ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY)) {
                    ImGui::TableSetupColumn("##icon", ImGuiTableColumnFlags_WidthFixed, 22.0f);
                    ImGui::TableSetupColumn("Spell", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableSetupColumn("Level", ImGuiTableColumnFlags_WidthFixed, 40.0f);
                    ImGui::TableSetupColumn("Cost", ImGuiTableColumnFlags_WidthFixed, 120.0f);
                    ImGui::TableSetupColumn("##action", ImGuiTableColumnFlags_WidthFixed, 55.0f);
                    ImGui::TableHeadersRow();
                    renderSpellRows(spells);
                    ImGui::EndTable();
                }
            };

            const auto& tabs = gameHandler.getTrainerTabs();
            if (tabs.size() > 1) {
                // Multiple tabs - show tab bar
                if (ImGui::BeginTabBar("TrainerTabs")) {
                    for (size_t i = 0; i < tabs.size(); i++) {
                        char tabLabel[64];
                        snprintf(tabLabel, sizeof(tabLabel), "%s (%zu)",
                            tabs[i].name.c_str(), tabs[i].spells.size());

                        if (ImGui::BeginTabItem(tabLabel)) {
                            char tableId[32];
                            snprintf(tableId, sizeof(tableId), "TT%zu", i);
                            renderSpellTable(tableId, tabs[i].spells);
                            ImGui::EndTabItem();
                        }
                    }
                    ImGui::EndTabBar();
                }
            } else {
                // Single tab or no categorization - flat list
                std::vector<const game::TrainerSpell*> allSpells;
                allSpells.reserve(trainer.spells.size());
                for (const auto& spell : trainer.spells) {
                    allSpells.push_back(&spell);
                }
                renderSpellTable("TrainerTable", allSpells);
            }

            // What "trainable right now" means, asked twice: once to price the
            // Train All button and once to act on it. Two copies of this rule
            // is a button that promises one set of spells and trains another.
            const auto canTrainNow = [&](const game::TrainerSpell& spell) {
                const bool prereqsMet = isKnown(spell.chainNode1) &&
                                        isKnown(spell.chainNode2) &&
                                        isKnown(spell.chainNode3);
                const bool levelMet = (spell.reqLevel == 0 || playerLevel >= spell.reqLevel);
                // State 1 is "not available yet". The server sends it from the
                // last time it looked, so a prerequisite learned since is only
                // reflected here.
                uint8_t effectiveState = spell.state;
                if (spell.state == 1 && prereqsMet && levelMet && skillMet(spell)) {
                    effectiveState = 0;
                }
                return !isKnown(spell.spellId) && effectiveState == 0 &&
                       prereqsMet && levelMet && money >= spell.spellCost;
            };

            // Count how many spells are trainable right now
            int trainableCount = 0;
            uint64_t totalCost = 0;
            for (const auto& spell : trainer.spells) {
                if (canTrainNow(spell)) {
                    ++trainableCount;
                    totalCost += spell.spellCost;
                }
            }

            ImGui::Separator();
            bool canAffordAll = (money >= totalCost);
            bool hasTrainable = (trainableCount > 0) && canAffordAll;
            if (!hasTrainable) ImGui::BeginDisabled();
            const auto coins = game::splitCopper(totalCost);
            char trainAllLabel[80];
            if (trainableCount == 0) {
                snprintf(trainAllLabel, sizeof(trainAllLabel), "Train All Available (none)");
            } else {
                snprintf(trainAllLabel, sizeof(trainAllLabel),
                         "Train All Available (%d spell%s, %s)",
                         trainableCount, trainableCount == 1 ? "" : "s",
                         game::formatCoinPrice(coins).c_str());
            }
            if (ImGui::Button(trainAllLabel, ImVec2(-1.0f, 0.0f))) {
                for (const auto& spell : trainer.spells) {
                    if (canTrainNow(spell)) {
                        gameHandler.trainSpell(spell.spellId);
                    }
                }
            }
            if (!hasTrainable) ImGui::EndDisabled();

            // Profession trainer: crafting panel
            if (isProfessionTrainer) {
                ImGui::Separator();
                static int craftQuantity = 1;
                static uint32_t selectedCraftSpell = 0;

                gameHandler.loadSpellNameCache();

                // Difficulty color/label from skill thresholds - shared with
                // the standalone crafting window (crafting_window.cpp)
                auto getDifficultyColor = [&](uint32_t spellId) -> ImVec4 {
                    return recipeDifficultyColor(gameHandler, spellId);
                };
                auto getDifficultyLabel = [&](uint32_t spellId) -> const char* {
                    return recipeDifficultyLabel(gameHandler, spellId);
                };

                std::vector<const game::TrainerSpell*> craftable;
                for (const auto& spell : trainer.spells) {
                    if (isKnown(spell.spellId))
                        craftable.push_back(&spell);
                }

                // Show player skill level
                if (!craftable.empty()) {
                    auto slIt = gameHandler.spellToSkillLineRef().find(craftable[0]->spellId);
                    if (slIt != gameHandler.spellToSkillLineRef().end()) {
                        auto skIt = gameHandler.getPlayerSkills().find(slIt->second);
                        if (skIt != gameHandler.getPlayerSkills().end()) {
                            const std::string& skillName = gameHandler.getSkillName(slIt->second);
                            ImGui::TextDisabled("%s: %u / %u",
                                skillName.empty() ? "Skill" : skillName.c_str(),
                                skIt->second.effectiveValue(), skIt->second.maxValue);
                        }
                    }
                }

                int queueRemaining = gameHandler.getCraftQueueRemaining();
                if (queueRemaining > 0) {
                    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f),
                        "Crafting... %d remaining", queueRemaining);
                    ImGui::SameLine();
                    if (ImGui::SmallButton("Stop")) {
                        gameHandler.cancelCraftQueue();
                        gameHandler.cancelCast();
                    }
                } else if (!craftable.empty()) {
                    const char* previewName = "Select recipe...";
                    for (const auto* sp : craftable) {
                        if (sp->spellId == selectedCraftSpell) {
                            const std::string& n = gameHandler.getSpellName(sp->spellId);
                            if (!n.empty()) previewName = n.c_str();
                            break;
                        }
                    }
                    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.55f);
                    if (ImGui::BeginCombo("##CraftSelect", previewName)) {
                        for (const auto* sp : craftable) {
                            const std::string& n = gameHandler.getSpellName(sp->spellId);
                            const std::string& r = gameHandler.getSpellRank(sp->spellId);
                            ImVec4 diffCol = getDifficultyColor(sp->spellId);
                            char label[128];
                            if (!r.empty())
                                snprintf(label, sizeof(label), "%s (%s)##%u",
                                    n.empty() ? "???" : n.c_str(), r.c_str(), sp->spellId);
                            else
                                snprintf(label, sizeof(label), "%s##%u",
                                    n.empty() ? "???" : n.c_str(), sp->spellId);
                            ImGui::PushStyleColor(ImGuiCol_Text, diffCol);
                            if (ImGui::Selectable(label, sp->spellId == selectedCraftSpell))
                                selectedCraftSpell = sp->spellId;
                            ImGui::PopStyleColor();
                        }
                        ImGui::EndCombo();
                    }

                    // Show selected recipe details
                    bool hasAllReagents = true;
                    if (selectedCraftSpell != 0) {
                        auto cacheIt = gameHandler.spellNameCacheRef().find(selectedCraftSpell);
                        if (cacheIt != gameHandler.spellNameCacheRef().end()) {
                            const auto& spellEntry = cacheIt->second;
                            ImGui::Spacing();

                            // Difficulty
                            ImVec4 diffCol = getDifficultyColor(selectedCraftSpell);
                            const char* diffLabel = getDifficultyLabel(selectedCraftSpell);
                            ImGui::TextDisabled("Difficulty:");
                            ImGui::SameLine(0, 4);
                            ImGui::TextColored(diffCol, "%s", diffLabel);

                            // Produced item
                            if (spellEntry.createdItemId != 0) {
                                gameHandler.ensureItemInfo(spellEntry.createdItemId);
                                const auto* prodInfo = gameHandler.getItemInfo(spellEntry.createdItemId);
                                ImGui::TextDisabled("Creates:");
                                ImGui::SameLine(0, 4);
                                if (prodInfo && prodInfo->displayInfoId) {
                                    VkDescriptorSet icon = inventoryScreen.getItemIcon(prodInfo->displayInfoId);
                                    if (icon) {
                                        ImGui::Image((ImTextureID)(uintptr_t)icon, ImVec2(20, 20));
                                        ImGui::SameLine(0, 4);
                                    }
                                }
                                if (prodInfo && !prodInfo->name.empty()) {
                                    ImVec4 nameCol = InventoryScreen::getQualityColor(
                                        static_cast<game::ItemQuality>(prodInfo->quality));
                                    ImGui::TextColored(nameCol, "%s", prodInfo->name.c_str());
                                    if (prodInfo->armor > 0 || prodInfo->damageMax > 0.0f ||
                                        prodInfo->stamina != 0 || prodInfo->strength != 0 ||
                                        prodInfo->agility != 0 || prodInfo->intellect != 0 ||
                                        prodInfo->spirit != 0) {
                                        ImGui::Indent(24.0f);
                                        if (prodInfo->armor > 0)
                                            ImGui::TextDisabled("Armor: %d", prodInfo->armor);
                                        if (prodInfo->damageMax > 0.0f)
                                            ImGui::TextDisabled("Damage: %.0f - %.0f", prodInfo->damageMin, prodInfo->damageMax);
                                        auto showStat = [](const char* label, int32_t val) {
                                            if (val != 0) ImGui::TextColored(
                                                ImVec4(0.4f, 0.9f, 0.4f, 1.0f), "+%d %s", val, label);
                                        };
                                        showStat("Stamina", prodInfo->stamina);
                                        showStat("Strength", prodInfo->strength);
                                        showStat("Agility", prodInfo->agility);
                                        showStat("Intellect", prodInfo->intellect);
                                        showStat("Spirit", prodInfo->spirit);
                                        if (!prodInfo->description.empty())
                                            ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 1.0f),
                                                "\"%s\"", prodInfo->description.c_str());
                                        ImGui::Unindent(24.0f);
                                    }
                                } else {
                                    ImGui::Text("Item #%u", spellEntry.createdItemId);
                                }
                            }

                            // Reagents
                            bool hasReagents = false;
                            for (int r = 0; r < 8; ++r) {
                                if (spellEntry.reagents[r].itemId != 0) { hasReagents = true; break; }
                            }
                            if (hasReagents) {
                                ImGui::TextDisabled("Reagents:");
                                for (int r = 0; r < 8; ++r) {
                                    uint32_t rId = spellEntry.reagents[r].itemId;
                                    uint32_t rCount = spellEntry.reagents[r].count;
                                    if (rId == 0 || rCount == 0) continue;
                                    gameHandler.ensureItemInfo(rId);
                                    const auto* rInfo = gameHandler.getItemInfo(rId);
                                    // Live have/need count so consumed reagents update as you craft
                                    uint32_t have = countItemInInventory(gameHandler.getInventory(), rId);
                                    bool enough = have >= rCount;
                                    if (!enough) hasAllReagents = false;
                                    ImVec4 haveCol = enough ? colors::kLightGreen
                                                            : ImVec4(1.0f, 0.6f, 0.6f, 1.0f);
                                    ImGui::Indent(24.0f);
                                    if (rInfo && rInfo->displayInfoId) {
                                        VkDescriptorSet icon = inventoryScreen.getItemIcon(rInfo->displayInfoId);
                                        if (icon) {
                                            ImGui::Image((ImTextureID)(uintptr_t)icon, ImVec2(16, 16));
                                            ImGui::SameLine(0, 4);
                                        }
                                    }
                                    if (rInfo && !rInfo->name.empty()) {
                                        ImVec4 rCol = InventoryScreen::getQualityColor(
                                            static_cast<game::ItemQuality>(rInfo->quality));
                                        ImGui::TextColored(rCol, "%s", rInfo->name.c_str());
                                        ImGui::SameLine(0, 6);
                                        ImGui::TextColored(haveCol, "%u/%u", have, rCount);
                                    } else {
                                        ImGui::Text("Item #%u", rId);
                                        ImGui::SameLine(0, 6);
                                        ImGui::TextColored(haveCol, "%u/%u", have, rCount);
                                    }
                                    ImGui::Unindent(24.0f);
                                }
                            }
                        }
                        ImGui::Spacing();
                    }

                    // Craft controls
                    ImGui::SetNextItemWidth(50.0f);
                    ImGui::InputInt("##CraftQty", &craftQuantity, 0, 0);
                    if (craftQuantity < 1) craftQuantity = 1;
                    if (craftQuantity > 99) craftQuantity = 99;
                    ImGui::SameLine();
                    bool canCraft = selectedCraftSpell != 0 && !gameHandler.isCasting() &&
                                    hasAllReagents;
                    if (!canCraft) ImGui::BeginDisabled();
                    if (ImGui::Button("Create")) {
                        if (craftQuantity == 1)
                            gameHandler.castSpell(selectedCraftSpell, 0);
                        else
                            gameHandler.startCraftQueue(selectedCraftSpell, craftQuantity);
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Create All")) {
                        gameHandler.startCraftQueue(selectedCraftSpell, 999);
                    }
                    if (!canCraft) ImGui::EndDisabled();
                }
            }
        }
    }
    ImGui::End();

    if (!open) {
        gameHandler.closeTrainer();
    }
}

void WindowManager::renderEscapeMenu(SettingsPanel& settingsPanel,
                                     game::GameHandler& gameHandler) {
    if (!showEscapeMenu) return;

    ImGuiIO& io = ImGui::GetIO();
    float screenW = io.DisplaySize.x;
    float screenH = io.DisplaySize.y;
    ImVec2 size(260.0f, 248.0f);
    ImVec2 pos((screenW - size.x) * 0.5f, (screenH - size.y) * 0.5f);

    ImGui::SetNextWindowPos(pos, ImGuiCond_Always);
    ImGui::SetNextWindowSize(size, ImGuiCond_Always);
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar;

    if (ImGui::Begin("##EscapeMenu", nullptr, flags)) {
        ImGui::Text("Game Menu");
        ImGui::Separator();

        if (ImGui::Button("Logout", ImVec2(-1, 0))) {
            core::Application::getInstance().logoutToLogin();
            showEscapeMenu = false;
            settingsPanel.showEscapeSettingsNotice = false;
        }
        if (ImGui::Button("Quit", ImVec2(-1, 0))) {
            auto* ac = services_.audioCoordinator;
            if (ac) {
                if (auto* music = ac->getMusicManager()) {
                    music->stopMusic(0.0f);
                }
            }
            if (auto* window = services_.window) {
                window->setShouldClose(true);
            } else if (auto* window = core::Application::getInstance().getWindow()) {
                window->setShouldClose(true);
            }
            showEscapeMenu = false;
        }
        if (ImGui::Button("Settings", ImVec2(-1, 0))) {
            settingsPanel.showEscapeSettingsNotice = false;
            // The same routing as the Help button below, and for the same
            // reason: this menu is the client's own, the settings behind it
            // need not be.
            //
            // There is one settings screen, not two. Every category this client
            // adds - Graphics, Upscaling, Display, Camera, Interface, Gameplay,
            // Combat, Chat, Minimap, Nameplates - is built as a panel inside
            // the game's own options frames, so opening a second window of our
            // own beside them offered the same settings twice, in two places
            // that looked nothing like each other. The window is still what a
            // client without the interface up falls back to.
            if (frameXmlOwns(UiElement::GameMenu)) {
                gameHandler.runInterfaceCommand("VideoOptionsFrame_Toggle()");
            } else {
                settingsPanel.showSettingsWindow = true;
                settingsPanel.settingsInit = false;
            }
            showEscapeMenu = false;
        }
        if (ImGui::Button("Instance Lockouts", ImVec2(-1, 0))) {
            showInstanceLockouts_ = true;
            showEscapeMenu = false;
        }
        if (ImGui::Button("Help / GM Ticket", ImVec2(-1, 0))) {
            // This menu is this client's own; the help panel behind the
            // button is FrameXML's. The same routing as the slash command.
            gameHandler.runInterfaceCommand("ToggleHelpFrame()");
            showEscapeMenu = false;
        }

        ImGui::Spacing();
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10.0f, 10.0f));
        if (ImGui::Button("Back to Game", ImVec2(-1, 0))) {
            showEscapeMenu = false;
            settingsPanel.showEscapeSettingsNotice = false;
        }
        ImGui::PopStyleVar();
    }
    ImGui::End();
}

// ---------------------------------------------------------------------------
// Barber shop state
//
// Built once each time the chair opens, and kept out of the render function so
// that the answers survive the panel being handed to FrameXML.
// ---------------------------------------------------------------------------
int WindowManager::barberFindAppearance(const std::vector<BarberStyleOption>& options,
                                        uint8_t id) {
    const auto it = std::find_if(options.begin(), options.end(),
                                 [id](const auto& option) { return option.appearanceId == id; });
    return it == options.end() ? -1 : static_cast<int>(std::distance(options.begin(), it));
}

uint8_t WindowManager::barberSelectedAppearance(const std::vector<BarberStyleOption>& options,
                                                int index, uint8_t fallback) {
    return index >= 0 && index < static_cast<int>(options.size())
        ? options[static_cast<size_t>(index)].appearanceId : fallback;
}

void WindowManager::rebuildBarberHairColors(uint8_t hairStyle, uint8_t preferredColor,
                                            uint32_t raceId, uint32_t sexId) {
    barberHairColors_.clear();
    if (services_.assetManager) {
        auto dbc = services_.assetManager->loadDBC("CharSections.dbc");
        if (dbc && dbc->isLoaded()) {
            const auto* layout = pipeline::getActiveDBCLayout()
                ? pipeline::getActiveDBCLayout()->getLayout("CharSections") : nullptr;
            const auto fields = pipeline::detectCharSectionsFields(dbc.get(), layout);
            for (uint32_t row = 0; row < dbc->getRecordCount(); ++row) {
                if (dbc->getUInt32(row, fields.raceId) != raceId ||
                    dbc->getUInt32(row, fields.sexId) != sexId ||
                    dbc->getUInt32(row, fields.baseSection) != 3 ||
                    dbc->getUInt32(row, fields.variationIndex) != hairStyle) {
                    continue;
                }
                const uint32_t color = dbc->getUInt32(row, fields.colorIndex);
                if (color <= 255) barberHairColors_.push_back(static_cast<uint8_t>(color));
            }
        }
    }
    std::sort(barberHairColors_.begin(), barberHairColors_.end());
    barberHairColors_.erase(std::unique(barberHairColors_.begin(), barberHairColors_.end()),
                            barberHairColors_.end());
    const auto preferred = std::find(barberHairColors_.begin(), barberHairColors_.end(),
                                     preferredColor);
    barberHairColor_ = preferred == barberHairColors_.end()
        ? (barberHairColors_.empty() ? -1 : 0)
        : static_cast<int>(std::distance(barberHairColors_.begin(), preferred));
    barberColorsForHairStyle_ = hairStyle;
}

void WindowManager::ensureBarberState(game::GameHandler& gameHandler) {
    // Leaving the chair clears the state so the next visit rebuilds it against
    // whatever the character wears by then. The render function used to do this
    // on its early return, which stops happening the moment the panel is handed
    // over - and stale originals would price a change against the wrong hair.
    if (!gameHandler.isBarberShopOpen()) {
        barberInitialized_ = false;
        return;
    }
    if (barberInitialized_) return;
    const auto* ch = gameHandler.getActiveCharacter();
    if (!ch) return;

    const uint32_t raceId = static_cast<uint32_t>(ch->race);
    const uint32_t sexId = (ch->gender == game::Gender::FEMALE || ch->useFemaleModel) ? 1u : 0u;

    // BarberShopStyle IDs, rather than the visible appearance numbers, are the
    // wire values expected by a WotLK server. Build the race/sex-specific lists
    // once each time the chair opens.
    barberOrigSkinColor_ = static_cast<uint8_t>(ch->appearanceBytes & 0xFF);
    barberOrigHairStyle_ = static_cast<uint8_t>((ch->appearanceBytes >> 16) & 0xFF);
    barberOrigHairColor_ = static_cast<uint8_t>((ch->appearanceBytes >> 24) & 0xFF);
    barberOrigFacialHair_ = ch->facialFeatures;
    barberHairStyles_.clear();
    barberFacialStyles_.clear();
    barberSkinStyles_.clear();
    // Entry zero tells the server to retain the existing skin.
    barberSkinStyles_.push_back({0, barberOrigSkinColor_, "Current"});

    if (services_.assetManager) {
        auto dbc = services_.assetManager->loadDBC("BarberShopStyle.dbc");
        if (dbc && dbc->isLoaded() && dbc->getFieldCount() >= 40) {
            for (uint32_t row = 0; row < dbc->getRecordCount(); ++row) {
                if (dbc->getUInt32(row, 37) != raceId || dbc->getUInt32(row, 38) != sexId)
                    continue;
                const uint32_t appearance = dbc->getUInt32(row, 39);
                if (appearance > 255) continue;
                std::string name;
                for (uint32_t field = 2; field <= 17 && name.empty(); ++field)
                    name = dbc->getString(row, field);
                if (name.empty()) name = "Style " + std::to_string(appearance);
                BarberStyleOption option{dbc->getUInt32(row, 0),
                                         static_cast<uint8_t>(appearance), std::move(name)};
                switch (dbc->getUInt32(row, 1)) {
                    case 0: barberHairStyles_.push_back(std::move(option)); break;
                    case 2: barberFacialStyles_.push_back(std::move(option)); break;
                    case 3: barberSkinStyles_.push_back(std::move(option)); break;
                    default: break;
                }
            }
        } else {
            LOG_WARNING("Barber Shop: WotLK BarberShopStyle.dbc is unavailable or malformed");
        }

        auto baseCost = services_.assetManager->loadDBC("gtBarberShopCostBase.dbc");
        if (baseCost && baseCost->isLoaded() && baseCost->getRecordCount() > 0) {
            const uint32_t level = std::max<uint32_t>(1, ch->level);
            const uint32_t row = std::min(level, baseCost->getRecordCount()) - 1;
            barberBaseCost_ = baseCost->getFloat(row, 0);
        } else {
            barberBaseCost_ = 0.0f;
        }
    }

    auto normalizeOptions = [](std::vector<BarberStyleOption>& options) {
        std::sort(options.begin(), options.end(), [](const auto& a, const auto& b) {
            return a.appearanceId != b.appearanceId
                ? a.appearanceId < b.appearanceId : a.entryId < b.entryId;
        });
        options.erase(std::unique(options.begin(), options.end(), [](const auto& a, const auto& b) {
            return a.appearanceId == b.appearanceId;
        }), options.end());
    };
    normalizeOptions(barberHairStyles_);
    normalizeOptions(barberFacialStyles_);
    if (barberSkinStyles_.size() > 1) {
        std::sort(barberSkinStyles_.begin() + 1, barberSkinStyles_.end(),
                  [](const auto& a, const auto& b) { return a.appearanceId < b.appearanceId; });
        barberSkinStyles_.erase(std::unique(barberSkinStyles_.begin() + 1,
                                            barberSkinStyles_.end(),
                                            [](const auto& a, const auto& b) {
                                                return a.appearanceId == b.appearanceId;
                                            }), barberSkinStyles_.end());
    }

    barberHairStyle_ = barberFindAppearance(barberHairStyles_, barberOrigHairStyle_);
    barberFacialHair_ = barberFindAppearance(barberFacialStyles_, barberOrigFacialHair_);
    barberSkinColor_ = 0;
    rebuildBarberHairColors(barberOrigHairStyle_, barberOrigHairColor_, raceId, sexId);
    barberPreviewSkin_ = barberPreviewHairStyle_ = barberPreviewHairColor_ =
        barberPreviewFacialHair_ = -1;
    barberInitialized_ = true;
}

WindowManager::BarberSelection WindowManager::barberSelection(game::GameHandler& gameHandler) {
    BarberSelection out;
    ensureBarberState(gameHandler);
    const auto* ch = gameHandler.getActiveCharacter();
    if (!ch) return out;
    const uint32_t raceId = static_cast<uint32_t>(ch->race);
    const uint32_t sexId = (ch->gender == game::Gender::FEMALE || ch->useFemaleModel) ? 1u : 0u;

    out.hairStyle = barberSelectedAppearance(barberHairStyles_, barberHairStyle_,
                                             barberOrigHairStyle_);
    // The colours on offer depend on the hair style, so a style change has to
    // rebuild them before the colour selection can be read back.
    if (out.hairStyle != barberColorsForHairStyle_) {
        const uint8_t previousColor =
            barberHairColor_ >= 0 && barberHairColor_ < static_cast<int>(barberHairColors_.size())
                ? barberHairColors_[static_cast<size_t>(barberHairColor_)] : barberOrigHairColor_;
        rebuildBarberHairColors(out.hairStyle, previousColor, raceId, sexId);
    }
    out.hairColor = barberHairColor_ >= 0 &&
                    barberHairColor_ < static_cast<int>(barberHairColors_.size())
        ? barberHairColors_[static_cast<size_t>(barberHairColor_)] : barberOrigHairColor_;
    out.facialHair = barberSelectedAppearance(barberFacialStyles_, barberFacialHair_,
                                              barberOrigFacialHair_);
    out.skin = barberSelectedAppearance(barberSkinStyles_, barberSkinColor_,
                                        barberOrigSkinColor_);
    return out;
}

uint32_t WindowManager::barberTotalCostCopper(game::GameHandler& gameHandler) {
    const BarberSelection sel = barberSelection(gameHandler);
    float cost = 0.0f;
    if (sel.hairStyle != barberOrigHairStyle_)      cost += barberBaseCost_;
    else if (sel.hairColor != barberOrigHairColor_) cost += barberBaseCost_ * 0.5f;
    if (sel.facialHair != barberOrigFacialHair_)    cost += barberBaseCost_ * 0.75f;
    if (sel.skin != barberOrigSkinColor_)           cost += barberBaseCost_ * 0.75f;
    return static_cast<uint32_t>(cost);
}

void WindowManager::barberResetSelections(game::GameHandler& gameHandler) {
    ensureBarberState(gameHandler);
    const auto* ch = gameHandler.getActiveCharacter();
    if (!ch) return;
    const uint32_t raceId = static_cast<uint32_t>(ch->race);
    const uint32_t sexId = (ch->gender == game::Gender::FEMALE || ch->useFemaleModel) ? 1u : 0u;
    barberHairStyle_ = barberFindAppearance(barberHairStyles_, barberOrigHairStyle_);
    barberFacialHair_ = barberFindAppearance(barberFacialStyles_, barberOrigFacialHair_);
    barberSkinColor_ = 0;
    rebuildBarberHairColors(barberOrigHairStyle_, barberOrigHairColor_, raceId, sexId);
}

void WindowManager::barberApplySelection(game::GameHandler& gameHandler) {
    ensureBarberState(gameHandler);
    const BarberSelection sel = barberSelection(gameHandler);
    // The server is sent BarberShopStyle.dbc entry ids, not the appearance
    // numbers the preview uses - the two are different numbering spaces and
    // only the entry id means anything on the wire.
    auto entryOf = [](const std::vector<BarberStyleOption>& options, int index) {
        return index >= 0 && index < static_cast<int>(options.size())
            ? options[static_cast<size_t>(index)].entryId : 0u;
    };
    gameHandler.sendAlterAppearance(entryOf(barberHairStyles_, barberHairStyle_),
                                    sel.hairColor,
                                    entryOf(barberFacialStyles_, barberFacialHair_),
                                    entryOf(barberSkinStyles_, barberSkinColor_));
}

bool WindowManager::barberStyleInfo(game::GameHandler& gameHandler, int selector,
                                    std::string& name, bool& isCurrent) {
    ensureBarberState(gameHandler);
    const BarberSelection sel = barberSelection(gameHandler);
    switch (selector) {
        case 1:
            if (barberHairStyle_ < 0 ||
                barberHairStyle_ >= static_cast<int>(barberHairStyles_.size())) return false;
            name = barberHairStyles_[static_cast<size_t>(barberHairStyle_)].name;
            isCurrent = sel.hairStyle == barberOrigHairStyle_;
            return true;
        case 2: {
            if (barberHairColor_ < 0 ||
                barberHairColor_ >= static_cast<int>(barberHairColors_.size())) return false;
            // The colours have no names of their own in the data, so they are
            // numbered the way the character creation screen numbers them.
            name = "Color " + std::to_string(barberHairColor_ + 1);
            isCurrent = sel.hairColor == barberOrigHairColor_;
            return true;
        }
        case 3:
            if (barberFacialHair_ < 0 ||
                barberFacialHair_ >= static_cast<int>(barberFacialStyles_.size())) return false;
            name = barberFacialStyles_[static_cast<size_t>(barberFacialHair_)].name;
            isCurrent = sel.facialHair == barberOrigFacialHair_;
            return true;
        case 4:
            if (barberSkinColor_ < 0 ||
                barberSkinColor_ >= static_cast<int>(barberSkinStyles_.size())) return false;
            name = barberSkinStyles_[static_cast<size_t>(barberSkinColor_)].name;
            isCurrent = sel.skin == barberOrigSkinColor_;
            return true;
        default:
            return false;
    }
}

void WindowManager::barberCycleStyle(game::GameHandler& gameHandler, int selector,
                                     int direction) {
    ensureBarberState(gameHandler);
    const int step = direction >= 0 ? 1 : -1;
    auto advance = [step](int& index, size_t count) {
        if (count == 0) { index = -1; return; }
        const int n = static_cast<int>(count);
        index = ((index < 0 ? 0 : index) + step % n + n) % n;
    };
    switch (selector) {
        case 1: advance(barberHairStyle_, barberHairStyles_.size()); break;
        case 2: advance(barberHairColor_, barberHairColors_.size()); break;
        case 3: advance(barberFacialHair_, barberFacialStyles_.size()); break;
        case 4: advance(barberSkinColor_, barberSkinStyles_.size()); break;
        default: break;
    }
    // Re-resolve so a hair style change rebuilds the colours behind it.
    barberSelection(gameHandler);
}

void WindowManager::renderBarberShopWindow(game::GameHandler& gameHandler) {
    if (!gameHandler.isBarberShopOpen()) {
        barberInitialized_ = false;
        return;
    }

    const auto* ch = gameHandler.getActiveCharacter();
    if (!ch) return;

    auto selectedEntry = [](const std::vector<BarberStyleOption>& options, int index) {
        return index >= 0 && index < static_cast<int>(options.size())
            ? options[static_cast<size_t>(index)].entryId : 0u;
    };

    const bool firstFrame = !barberInitialized_;
    ensureBarberState(gameHandler);
    if (firstFrame) {
        if (!barberPreview_ && services_.assetManager && services_.renderer) {
            barberPreview_ = std::make_unique<rendering::CharacterPreview>();
            if (barberPreview_->initialize(services_.assetManager)) {
                services_.renderer->registerPreview(barberPreview_.get());
                barberPreview_->resetView();
            } else {
                LOG_WARNING("Barber Shop: failed to initialize character preview");
                barberPreview_.reset();
            }
        }
    }

    // Resolves the selections and rebuilds the colour list when the hair style
    // has moved, which is the same work the interface's own barber asks for.
    const BarberSelection sel = barberSelection(gameHandler);
    const uint8_t hairStyle = sel.hairStyle;
    const uint8_t hairColor = sel.hairColor;
    const uint8_t facial    = sel.facialHair;
    const uint8_t skin      = sel.skin;

    if (barberPreview_ && (barberPreviewSkin_ != skin ||
                           barberPreviewHairStyle_ != hairStyle ||
                           barberPreviewHairColor_ != hairColor ||
                           barberPreviewFacialHair_ != facial)) {
        const uint8_t face = static_cast<uint8_t>((ch->appearanceBytes >> 8) & 0xFF);
        if (barberPreview_->loadCharacter(ch->race, ch->gender, skin, face, hairStyle,
                                          hairColor, facial, ch->useFemaleModel)) {
            barberPreview_->applyEquipment(ch->equipment);
        }
        barberPreviewSkin_ = skin;
        barberPreviewHairStyle_ = hairStyle;
        barberPreviewHairColor_ = hairColor;
        barberPreviewFacialHair_ = facial;
    }
    if (barberPreview_) {
        barberPreview_->update(ImGui::GetIO().DeltaTime);
        barberPreview_->render();
        barberPreview_->requestComposite();
    }

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;
    float winW = 720.0f;
    float winH = 520.0f;
    ImGui::SetNextWindowPos(ImVec2((screenW - winW) / 2.0f, (screenH - winH) / 2.0f), ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(winW, winH), ImGuiCond_Appearing);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse;
    bool open = true;
    if (ImGui::Begin("Barber Shop", &open, flags)) {
        ImGui::Text("Choose your new look:");
        ImGui::Separator();
        ImGui::Spacing();

        const float previewW = 300.0f;
        ImGui::BeginChild("##BarberPreview", ImVec2(previewW, 410.0f), true);
        if (barberPreview_ && barberPreview_->getTextureId()) {
            const float imageW = ImGui::GetContentRegionAvail().x;
            const float imageH = imageW * static_cast<float>(barberPreview_->getHeight()) /
                                 static_cast<float>(barberPreview_->getWidth());
            ImGui::Image(reinterpret_cast<ImTextureID>(barberPreview_->getTextureId()),
                         ImVec2(imageW, std::min(imageH, 365.0f)));
            if (ImGui::IsItemHovered() && ImGui::IsMouseDragging(ImGuiMouseButton_Left))
                barberPreview_->rotate(ImGui::GetIO().MouseDelta.x * 0.2f);
            if (ImGui::IsItemHovered() && ImGui::GetIO().MouseWheel != 0.0f)
                barberPreview_->zoom(ImGui::GetIO().MouseWheel);
            ImGui::TextDisabled("Drag to rotate - Scroll to zoom");
        } else {
            ImGui::TextDisabled("Preview unavailable");
        }
        ImGui::EndChild();

        ImGui::SameLine();
        ImGui::BeginChild("##BarberControls", ImVec2(0, 410.0f), false);
        ImGui::PushItemWidth(-1);

        auto styleCombo = [](const char* label, int& selected,
                             const std::vector<BarberStyleOption>& options) {
            const char* preview = selected >= 0 && selected < static_cast<int>(options.size())
                ? options[static_cast<size_t>(selected)].name.c_str() : "Unavailable";
            if (ImGui::BeginCombo(label, preview)) {
                for (int i = 0; i < static_cast<int>(options.size()); ++i) {
                    const bool active = i == selected;
                    if (ImGui::Selectable(options[static_cast<size_t>(i)].name.c_str(), active))
                        selected = i;
                    if (active) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
        };
        styleCombo("Hair Style", barberHairStyle_, barberHairStyles_);

        const char* colorPreview = "Unavailable";
        std::string colorLabel;
        if (barberHairColor_ >= 0 && barberHairColor_ < static_cast<int>(barberHairColors_.size())) {
            colorLabel = "Color " + std::to_string(hairColor);
            colorPreview = colorLabel.c_str();
        }
        if (ImGui::BeginCombo("Hair Color", colorPreview)) {
            for (int i = 0; i < static_cast<int>(barberHairColors_.size()); ++i) {
                const std::string label = "Color " + std::to_string(barberHairColors_[i]);
                const bool active = i == barberHairColor_;
                if (ImGui::Selectable(label.c_str(), active)) barberHairColor_ = i;
                if (active) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        const bool female = (ch->gender == game::Gender::FEMALE || ch->useFemaleModel);
        const char* facialLabel = female ? "Piercings / Features" : "Facial Hair / Features";
        styleCombo(facialLabel, barberFacialHair_, barberFacialStyles_);
        if (barberSkinStyles_.size() > 1)
            styleCombo("Skin Color", barberSkinColor_, barberSkinStyles_);
        ImGui::PopItemWidth();

        ImGui::Spacing();
        ImGui::Separator();

        const bool valid = barberHairStyle_ >= 0 && barberFacialHair_ >= 0 &&
                           barberHairColor_ >= 0;
        const bool changed = valid && (hairStyle != barberOrigHairStyle_ ||
                                       hairColor != barberOrigHairColor_ ||
                                       facial != barberOrigFacialHair_ ||
                                       skin != barberOrigSkinColor_);
        float costFloat = 0.0f;
        if (hairStyle != barberOrigHairStyle_) costFloat += barberBaseCost_;
        else if (hairColor != barberOrigHairColor_) costFloat += barberBaseCost_ * 0.5f;
        if (facial != barberOrigFacialHair_) costFloat += barberBaseCost_ * 0.75f;
        if (skin != barberOrigSkinColor_) costFloat += barberBaseCost_ * 0.75f;
        const uint32_t cost = static_cast<uint32_t>(costFloat);
        const bool canAfford = gameHandler.getMoneyCopper() >= cost;

        ImGui::Text("Price:");
        ImGui::SameLine();
        if (!canAfford) ImGui::PushStyleColor(ImGuiCol_Text, kColorRed);
        renderCoinsFromCopper(cost);
        if (!canAfford) ImGui::PopStyleColor();
        if (!valid)
            ImGui::TextColored(kColorRed, "No valid barber styles were found for this character.");
        ImGui::EndChild();

        // OK / Reset / Cancel buttons
        float btnW = 80.0f;
        float totalW = btnW * 3 + ImGui::GetStyle().ItemSpacing.x * 2;
        ImGui::SetCursorPosX((ImGui::GetWindowWidth() - totalW) / 2.0f);

        if (!changed || !canAfford) ImGui::BeginDisabled();
        if (ImGui::Button("OK", ImVec2(btnW, 0))) {
            gameHandler.sendAlterAppearance(
                selectedEntry(barberHairStyles_, barberHairStyle_),
                hairColor,
                selectedEntry(barberFacialStyles_, barberFacialHair_),
                selectedEntry(barberSkinStyles_, barberSkinColor_));
            // Keep window open - server will respond with SMSG_BARBER_SHOP_RESULT
        }
        if (!changed || !canAfford) ImGui::EndDisabled();

        ImGui::SameLine();
        if (!changed) ImGui::BeginDisabled();
        if (ImGui::Button("Reset", ImVec2(btnW, 0))) {
            barberResetSelections(gameHandler);
        }
        if (!changed) ImGui::EndDisabled();

        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(btnW, 0))) {
            gameHandler.closeBarberShop();
        }
    }
    ImGui::End();

    if (!open) {
        gameHandler.closeBarberShop();
    }
}



void WindowManager::renderLogoutCountdown(game::GameHandler& gameHandler) {
    if (!gameHandler.isLoggingOut()) return;
    // FrameXML counts the same seconds down. uiparent.lua raises the CAMP and
    // QUIT popups from PLAYER_CAMPING and PLAYER_QUITING, both of which this
    // client fires from social_handler, and "dialogs" has been handed over
    // since the branch started - so every /logout put two countdowns on
    // screen, one above the other.
    if (frameXmlOwns(UiElement::Dialogs)) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth())  : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    constexpr float W = 280.0f;
    constexpr float H = 80.0f;
    ImGui::SetNextWindowPos(ImVec2((screenW - W) * 0.5f, screenH * 0.5f - H * 0.5f - 60.0f),
                            ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(W, H), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.88f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 6.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.08f, 0.08f, 0.18f, 0.95f));
    ImGui::PushStyleColor(ImGuiCol_Border,   ImVec4(0.5f, 0.5f, 0.8f, 1.0f));

    if (ImGui::Begin("##LogoutCountdown", nullptr,
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoBringToFrontOnFocus)) {

        float cd = gameHandler.getLogoutCountdown();
        if (cd > 0.0f) {
            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 6.0f);
            ImGui::SetCursorPosX((W - ImGui::CalcTextSize("Logging out in 20s...").x) * 0.5f);
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.3f, 1.0f),
                               "Logging out in %ds...", static_cast<int>(std::ceil(cd)));

            // Progress bar (20 second countdown)
            float frac = 1.0f - std::min(cd / 20.0f, 1.0f);
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.5f, 0.5f, 0.9f, 1.0f));
            ImGui::ProgressBar(frac, ImVec2(-1.0f, 8.0f), "");
            ImGui::PopStyleColor();
            ImGui::Spacing();
        } else {
            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 14.0f);
            ImGui::SetCursorPosX((W - ImGui::CalcTextSize("Logging out...").x) * 0.5f);
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.3f, 1.0f), "Logging out...");
            ImGui::Spacing();
        }

        // Cancel button - only while countdown is still running
        if (cd > 0.0f) {
            float btnW = 100.0f;
            ImGui::SetCursorPosX((W - btnW) * 0.5f);
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.5f, 0.1f, 0.1f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.7f, 0.15f, 0.15f, 1.0f));
            if (ImGui::Button("Cancel", ImVec2(btnW, 0))) {
                gameHandler.cancelLogout();
            }
            ImGui::PopStyleColor(2);
        }
    }
    ImGui::End();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar();
}

void WindowManager::renderDeathScreen(game::GameHandler& gameHandler) {
    if (!gameHandler.showDeathDialog()) {
        deathTimerRunning_ = false;
        deathElapsed_ = 0.0f;
        return;
    }
    float dt = ImGui::GetIO().DeltaTime;
    if (!deathTimerRunning_) {
        deathElapsed_ = 0.0f;
        deathTimerRunning_ = true;
    } else {
        deathElapsed_ += dt;
    }

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    // Dark red overlay covering the whole screen
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(screenW, screenH));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.15f, 0.0f, 0.0f, 0.45f));
    ImGui::Begin("##DeathOverlay", nullptr,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoInputs |
        ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoFocusOnAppearing);
    ImGui::End();
    ImGui::PopStyleColor();

    // The prompt below is FrameXML's StaticPopup "DEATH", raised from
    // PLAYER_DEAD which this client fires. The red wash above it is not - WoW
    // desaturates the screen from C, so nothing on FrameXML's side draws it and
    // it stays whoever owns the dialogs.
    if (frameXmlOwns(UiElement::Dialogs)) return;

    // "Release Spirit" dialog centered on screen
    const bool hasSelfRes = gameHandler.canSelfRes();
    float dlgW = 280.0f;
    // Extra height when self-res button is available; +20 for the "wait for res" hint
    float dlgH = hasSelfRes ? 190.0f : 150.0f;
    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - dlgW / 2, screenH * 0.35f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(dlgW, dlgH), ImGuiCond_Always);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 8.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.0f, 0.0f, 0.9f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.6f, 0.1f, 0.1f, 1.0f));

    if (ImGui::Begin("##DeathDialog", nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar)) {

        ImGui::Spacing();
        // Center "You are dead." text
        const char* deathText = "You are dead.";
        float textW = ImGui::CalcTextSize(deathText).x;
        ImGui::SetCursorPosX((dlgW - textW) / 2);
        ImGui::TextColored(colors::kBrightRed, "%s", deathText);

        // Respawn timer: show how long until the server auto-releases the spirit
        float timeLeft = kForcedReleaseSec - deathElapsed_;
        if (timeLeft > 0.0f) {
            int mins = static_cast<int>(timeLeft) / 60;
            int secs = static_cast<int>(timeLeft) % 60;
            char timerBuf[48];
            snprintf(timerBuf, sizeof(timerBuf), "Auto-release in %d:%02d", mins, secs);
            float tw = ImGui::CalcTextSize(timerBuf).x;
            ImGui::SetCursorPosX((dlgW - tw) / 2);
            ImGui::TextColored(colors::kMediumGray, "%s", timerBuf);
        }

        ImGui::Spacing();
        ImGui::Spacing();

        // Self-resurrection button (Reincarnation / Twisting Nether / Deathpact)
        if (hasSelfRes) {
            float btnW2 = 220.0f;
            ImGui::SetCursorPosX((dlgW - btnW2) / 2);
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.35f, 0.55f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.2f, 0.5f, 0.75f, 1.0f));
            if (ImGui::Button("Use Self-Resurrection", ImVec2(btnW2, 30))) {
                gameHandler.useSelfRes();
            }
            ImGui::PopStyleColor(2);
            ImGui::Spacing();
        }

        // Center the Release Spirit button
        float btnW = 180.0f;
        ImGui::SetCursorPosX((dlgW - btnW) / 2);
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.5f, 0.1f, 0.1f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.7f, 0.15f, 0.15f, 1.0f));
        if (ImGui::Button("Release Spirit", ImVec2(btnW, 30))) {
            gameHandler.releaseSpirit();
        }
        ImGui::PopStyleColor(2);

        // Hint: player can stay dead and wait for another player to cast Resurrection
        const char* resHint = "Or wait for a player to resurrect you.";
        float hw = ImGui::CalcTextSize(resHint).x;
        ImGui::SetCursorPosX((dlgW - hw) / 2);
        ImGui::TextColored(ImVec4(0.5f, 0.6f, 0.5f, 0.85f), "%s", resHint);
    }
    ImGui::End();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar();
}

void WindowManager::renderReclaimCorpseButton(game::GameHandler& gameHandler) {
    if (!gameHandler.isPlayerGhost() || !gameHandler.canReclaimCorpse()) return;
    // StaticPopup "RECOVER_CORPSE" now has the CORPSE_IN_RANGE edge it needs,
    // so the prompt appears on FrameXML's side and this one steps aside.
    if (frameXmlOwns(UiElement::Dialogs)) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    float delaySec = gameHandler.getCorpseReclaimDelaySec();
    bool onDelay = (delaySec > 0.0f);

    float btnW = 220.0f, btnH = 36.0f;
    float winH = btnH + 16.0f + (onDelay ? 20.0f : 0.0f);
    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - btnW / 2, screenH * 0.72f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(btnW + 16.0f, winH), ImGuiCond_Always);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 6.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8.0f, 8.0f));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.7f));
    if (ImGui::Begin("##ReclaimCorpse", nullptr,
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoBringToFrontOnFocus)) {
        if (onDelay) {
            // Greyed-out button while PvP reclaim timer ticks down
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.25f, 0.25f, 0.25f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.25f, 0.25f, 1.0f));
            ImGui::BeginDisabled(true);
            char delayLabel[64];
            snprintf(delayLabel, sizeof(delayLabel), "Resurrect from Corpse (%.0fs)", delaySec);
            ImGui::Button(delayLabel, ImVec2(btnW, btnH));
            ImGui::EndDisabled();
            ImGui::PopStyleColor(2);
            const char* waitMsg = "You cannot reclaim your corpse yet.";
            float tw = ImGui::CalcTextSize(waitMsg).x;
            ImGui::SetCursorPosX((btnW + 16.0f - tw) * 0.5f);
            ImGui::TextColored(ImVec4(0.8f, 0.5f, 0.2f, 1.0f), "%s", waitMsg);
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.15f, 0.35f, 0.15f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.55f, 0.25f, 1.0f));
            if (ImGui::Button("Resurrect from Corpse", ImVec2(btnW, btnH))) {
                gameHandler.reclaimCorpse();
            }
            ImGui::PopStyleColor(2);
            float corpDist = gameHandler.getCorpseDistance();
            if (corpDist >= 0.0f) {
                char distBuf[48];
                snprintf(distBuf, sizeof(distBuf), "Corpse: %.0f yards away", corpDist);
                float dw = ImGui::CalcTextSize(distBuf).x;
                ImGui::SetCursorPosX((btnW + 16.0f - dw) * 0.5f);
                ImGui::TextDisabled("%s", distBuf);
            }
        }
    }
    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(2);
}

void WindowManager::renderMailWindow(game::GameHandler& gameHandler,
                             InventoryScreen& inventoryScreen,
                             ChatPanel& chatPanel) {
    if (!gameHandler.isMailboxOpen()) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 250, 80), ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(600, 500), ImGuiCond_Appearing);

    bool open = true;
    if (ImGui::Begin("Mailbox", &open)) {
        const auto& inbox = gameHandler.getMailInbox();

        // Top bar: money + compose button
        ImGui::TextDisabled("Your money:"); ImGui::SameLine(0, 4);
        renderCoinsFromCopper(gameHandler.getMoneyCopper());
        ImGui::SameLine(ImGui::GetWindowWidth() - 100);
        if (ImGui::Button("Compose")) {
            mailRecipientBuffer_[0] = '\0';
            mailSubjectBuffer_[0] = '\0';
            mailBodyBuffer_[0] = '\0';
            mailComposeMoney_[0] = 0;
            mailComposeMoney_[1] = 0;
            mailComposeMoney_[2] = 0;
            gameHandler.openMailCompose();
        }
        ImGui::Separator();

        if (inbox.empty()) {
            ImGui::TextDisabled("No mail.");
        } else {
            // Two-panel layout: left = mail list, right = selected mail detail
            float listWidth = 220.0f;

            // Left panel - mail list
            ImGui::BeginChild("MailList", ImVec2(listWidth, 0), true);
            for (size_t i = 0; i < inbox.size(); ++i) {
                const auto& mail = inbox[i];
                ImGui::PushID(static_cast<int>(i));

                bool selected = (gameHandler.getSelectedMailIndex() == static_cast<int>(i));
                std::string label = gameHandler.getMailDisplaySubject(mail);
                if (label.empty()) label = "(No Subject)";

                // Unread indicator
                if (!mail.read) {
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 0.5f, 1.0f));
                }

                if (ImGui::Selectable(label.c_str(), selected)) {
                    gameHandler.setSelectedMailIndex(static_cast<int>(i));
                    // Mark as read
                    if (!mail.read) {
                        gameHandler.mailMarkAsRead(mail.messageId);
                    }
                }

                if (!mail.read) {
                    ImGui::PopStyleColor();
                }

                // Sub-info line
                ImGui::TextColored(kColorGray, "  From: %s",
                                   gameHandler.getMailSenderName(mail).c_str());
                if (mail.money > 0) {
                    ImGui::SameLine();
                    ImGui::TextColored(colors::kWarmGold, " [G]");
                }
                if (!mail.attachments.empty()) {
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(0.5f, 0.8f, 1.0f, 1.0f), " [A]");
                }
                // Expiry warning if within 3 days. expirationTime is days
                // remaining (server sends (expire_time - now) / DAY as a float).
                if (mail.expirationTime > 0.0f && mail.expirationTime < 3.0f) {
                    ImGui::SameLine();
                    int daysLeft = static_cast<int>(mail.expirationTime);
                    if (daysLeft == 0) {
                        ImGui::TextColored(colors::kBrightRed, " [expires today!]");
                    } else {
                        ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.1f, 1.0f),
                            " [expires in %dd]", daysLeft);
                    }
                }

                ImGui::PopID();
            }
            ImGui::EndChild();

            ImGui::SameLine();

            // Right panel - selected mail detail
            ImGui::BeginChild("MailDetail", ImVec2(0, 0), true);
            int sel = gameHandler.getSelectedMailIndex();
            if (sel >= 0 && sel < static_cast<int>(inbox.size())) {
                const auto& mail = inbox[sel];
                const std::string displaySubject = gameHandler.getMailDisplaySubject(mail);

                ImGui::TextColored(colors::kWarmGold, "%s",
                    displaySubject.empty() ? "(No Subject)" : displaySubject.c_str());
                ImGui::Text("From: %s", gameHandler.getMailSenderName(mail).c_str());

                if (mail.messageType == 2) {
                    ImGui::TextColored(ImVec4(0.8f, 0.6f, 0.2f, 1.0f), "[Auction House]");
                }

                // Show expiry date in the detail panel. expirationTime is days
                // remaining, so convert to an absolute date relative to now.
                if (mail.expirationTime > 0.0f) {
                    time_t expT = std::time(nullptr) +
                        static_cast<time_t>(mail.expirationTime * 86400.0f);
                    const std::tm tmExp = core::localTime(expT);
                    {
                        const char* mname = kMonthAbbrev[tmExp.tm_mon];
                        int daysLeft = static_cast<int>(mail.expirationTime);
                        if (mail.expirationTime < 3.0f) {
                            ImGui::TextColored(kColorRed,
                                "Expires: %s %d, %d (%d day%s!)",
                                mname, tmExp.tm_mday, 1900 + tmExp.tm_year,
                                daysLeft, daysLeft == 1 ? "" : "s");
                        } else {
                            ImGui::TextDisabled("Expires: %s %d, %d",
                                mname, tmExp.tm_mday, 1900 + tmExp.tm_year);
                        }
                    }
                }
                ImGui::Separator();

                // Body text. Auction-house mail carries an encoded invoice
                // string that the retail client renders as a money breakdown
                // rather than printing raw.
                game::AuctionMailInvoice invoice;
                if (mail.messageType == 2 &&
                    game::parseAuctionMailBody(mail.body, invoice)) {
                    if (invoice.bid > 0) {
                        ImGui::TextDisabled("Bid:"); ImGui::SameLine(0, 4);
                        renderCoinsFromCopper(invoice.bid);
                    }
                    if (invoice.buyout > 0) {
                        ImGui::TextDisabled("Buyout:"); ImGui::SameLine(0, 4);
                        renderCoinsFromCopper(invoice.buyout);
                    }
                    if (invoice.deposit > 0) {
                        ImGui::TextDisabled("Deposit:"); ImGui::SameLine(0, 4);
                        renderCoinsFromCopper(invoice.deposit);
                    }
                    if (invoice.consignment > 0) {
                        ImGui::TextDisabled("Auction House Cut:"); ImGui::SameLine(0, 4);
                        renderCoinsFromCopper(invoice.consignment);
                    }
                    ImGui::Separator();
                } else if (!mail.body.empty()) {
                    ImGui::TextWrapped("%s", mail.body.c_str());
                    ImGui::Separator();
                }

                // Money
                if (mail.money > 0) {
                    ImGui::TextDisabled("Money:"); ImGui::SameLine(0, 4);
                    renderCoinsFromCopper(mail.money);
                    ImGui::SameLine();
                    if (ImGui::SmallButton("Take Money")) {
                        gameHandler.mailTakeMoney(mail.messageId);
                    }
                }

                // COD warning
                if (mail.cod > 0) {
                    uint64_t g = mail.cod / 10000;
                    uint64_t s = (mail.cod / 100) % 100;
                    uint64_t c = mail.cod % 100;
                    ImGui::TextColored(kColorRed,
                        "COD: %llug %llus %lluc (you pay this to take items)",
                        static_cast<unsigned long long>(g),
                        static_cast<unsigned long long>(s),
                        static_cast<unsigned long long>(c));
                }

                // Attachments
                if (!mail.attachments.empty()) {
                    ImGui::Text("Attachments: %zu", mail.attachments.size());
                    ImDrawList* mailDraw = ImGui::GetWindowDrawList();
                    constexpr float MAIL_SLOT = 34.0f;
                    for (size_t j = 0; j < mail.attachments.size(); ++j) {
                        const auto& att = mail.attachments[j];
                        ImGui::PushID(static_cast<int>(j));

                        auto* info = gameHandler.getItemInfo(att.itemId);
                        game::ItemQuality quality = game::ItemQuality::COMMON;
                        std::string name = "Item " + std::to_string(att.itemId);
                        uint32_t displayInfoId = 0;
                        if (info && info->valid) {
                            quality = static_cast<game::ItemQuality>(info->quality);
                            name = info->name;
                            displayInfoId = info->displayInfoId;
                        } else {
                            gameHandler.ensureItemInfo(att.itemId);
                        }
                        ImVec4 qc = InventoryScreen::getQualityColor(quality);
                        ImU32 borderCol = ImGui::ColorConvertFloat4ToU32(qc);

                        ImVec2 pos = ImGui::GetCursorScreenPos();
                        VkDescriptorSet iconTex = displayInfoId
                            ? inventoryScreen.getItemIcon(displayInfoId) : VK_NULL_HANDLE;
                        if (iconTex) {
                            mailDraw->AddImage((ImTextureID)(uintptr_t)iconTex, pos,
                                              ImVec2(pos.x + MAIL_SLOT, pos.y + MAIL_SLOT));
                            mailDraw->AddRect(pos, ImVec2(pos.x + MAIL_SLOT, pos.y + MAIL_SLOT),
                                             borderCol, 0.0f, 0, 1.5f);
                        } else {
                            mailDraw->AddRectFilled(pos,
                                ImVec2(pos.x + MAIL_SLOT, pos.y + MAIL_SLOT),
                                IM_COL32(40, 35, 30, 220));
                            mailDraw->AddRect(pos,
                                ImVec2(pos.x + MAIL_SLOT, pos.y + MAIL_SLOT),
                                borderCol, 0.0f, 0, 1.5f);
                        }
                        if (att.stackCount > 1) {
                            char cnt[16];
                            snprintf(cnt, sizeof(cnt), "%u", att.stackCount);
                            float cw = ImGui::CalcTextSize(cnt).x;
                            mailDraw->AddText(ImVec2(pos.x + 1.0f, pos.y + 1.0f),
                                             IM_COL32(0, 0, 0, 200), cnt);
                            mailDraw->AddText(
                                ImVec2(pos.x + MAIL_SLOT - cw - 2.0f, pos.y + MAIL_SLOT - 14.0f),
                                IM_COL32(255, 255, 255, 220), cnt);
                        }

                        ImGui::InvisibleButton("##mailatt", ImVec2(MAIL_SLOT, MAIL_SLOT));
                        if (ImGui::IsItemHovered() && info && info->valid)
                            inventoryScreen.renderItemTooltip(*info);
                        if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
                            ImGui::GetIO().KeyShift && info && info->valid && !info->name.empty()) {
                            std::string link = game::itemChatLink(info->entry, info->quality, info->name);
                            chatPanel.insertChatLink(link);
                        }
                        ImGui::SameLine();
                        ImGui::TextColored(qc, "%s", name.c_str());
                        if (ImGui::IsItemHovered() && info && info->valid)
                            inventoryScreen.renderItemTooltip(*info);
                        if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
                            ImGui::GetIO().KeyShift && info && info->valid && !info->name.empty()) {
                            std::string link = game::itemChatLink(info->entry, info->quality, info->name);
                            chatPanel.insertChatLink(link);
                        }
                        ImGui::SameLine();
                        if (ImGui::SmallButton("Take")) {
                            gameHandler.mailTakeItem(mail.messageId, att.itemGuidLow);
                        }

                        ImGui::PopID();
                    }
                    // "Take All" button when there are multiple attachments
                    if (mail.attachments.size() > 1) {
                        if (ImGui::SmallButton("Take All")) {
                            for (const auto& att2 : mail.attachments) {
                                gameHandler.mailTakeItem(mail.messageId, att2.itemGuidLow);
                            }
                        }
                    }
                }

                ImGui::Spacing();
                ImGui::Separator();

                // Action buttons
                if (ImGui::Button("Delete")) {
                    gameHandler.mailDelete(mail.messageId);
                }
                ImGui::SameLine();
                if (mail.messageType == 0 && ImGui::Button("Reply")) {
                    // Pre-fill compose with sender as recipient
                    strncpy(mailRecipientBuffer_, mail.senderName.c_str(), sizeof(mailRecipientBuffer_) - 1);
                    mailRecipientBuffer_[sizeof(mailRecipientBuffer_) - 1] = '\0';
                    std::string reSubject = "Re: " + mail.subject;
                    strncpy(mailSubjectBuffer_, reSubject.c_str(), sizeof(mailSubjectBuffer_) - 1);
                    mailSubjectBuffer_[sizeof(mailSubjectBuffer_) - 1] = '\0';
                    mailBodyBuffer_[0] = '\0';
                    mailComposeMoney_[0] = 0;
                    mailComposeMoney_[1] = 0;
                    mailComposeMoney_[2] = 0;
                    gameHandler.openMailCompose();
                }
            } else {
                ImGui::TextDisabled("Select a mail to read.");
            }
            ImGui::EndChild();
        }
    }
    ImGui::End();

    if (!open) {
        gameHandler.closeMailbox();
    }
}

void WindowManager::renderMailComposeWindow(game::GameHandler& gameHandler,
                                   InventoryScreen& inventoryScreen) {
    if (!gameHandler.isMailComposeOpen()) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 190, screenH / 2 - 250), ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(400, 500), ImGuiCond_Appearing);

    bool open = true;
    if (ImGui::Begin("Send Mail", &open)) {
        ImGui::Text("To:");
        ImGui::SameLine(60);
        ImGui::SetNextItemWidth(-1);
        ImGui::InputText("##MailTo", mailRecipientBuffer_, sizeof(mailRecipientBuffer_));

        ImGui::Text("Subject:");
        ImGui::SameLine(60);
        ImGui::SetNextItemWidth(-1);
        ImGui::InputText("##MailSubject", mailSubjectBuffer_, sizeof(mailSubjectBuffer_));

        ImGui::Text("Body:");
        ImGui::InputTextMultiline("##MailBody", mailBodyBuffer_, sizeof(mailBodyBuffer_),
                                   ImVec2(-1, 120));

        // Attachments section
        int attachCount = gameHandler.getMailAttachmentCount();
        const int attachMax = gameHandler.getMaxMailAttachments();
        ImGui::Text("Attachments (%d/%d):", attachCount, attachMax);
        ImGui::SameLine();
        ImGui::TextColored(kColorGray, "Right-click items in bags to attach");

        const auto& attachments = gameHandler.getMailAttachments();
        // Show attachment slots in a grid (6 per row)
        for (int i = 0; i < attachMax; ++i) {
            if (i % 6 != 0) ImGui::SameLine();
            ImGui::PushID(i + 5000);
            const auto& att = attachments[i];
            if (att.occupied()) {
                // Show item with quality color border
                ImVec4 qualColor = ui::InventoryScreen::getQualityColor(att.item.quality);
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(qualColor.x * 0.3f, qualColor.y * 0.3f, qualColor.z * 0.3f, 0.8f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(qualColor.x * 0.5f, qualColor.y * 0.5f, qualColor.z * 0.5f, 0.9f));

                // Try to show icon
                VkDescriptorSet icon = inventoryScreen.getItemIcon(att.item.displayInfoId);
                bool clicked = false;
                if (icon) {
                    clicked = ImGui::ImageButton("##att", (ImTextureID)icon, ImVec2(30, 30));
                } else {
                    // Truncate name to fit
                    std::string label = att.item.name.substr(0, 4);
                    clicked = ImGui::Button(label.c_str(), ImVec2(36, 36));
                }
                ImGui::PopStyleColor(2);

                // Stack size, drawn over the icon the way the inbox and the bags
                // both do it. Without it an attached stack looked identical to a
                // single item, so there was no way to tell what was being sent.
                const uint32_t stack = att.item.stackCount;
                if (stack > 1) {
                    char cnt[16];
                    snprintf(cnt, sizeof(cnt), "%u", stack);
                    const ImVec2 rmax = ImGui::GetItemRectMax();
                    const float cw = ImGui::CalcTextSize(cnt).x;
                    ImDrawList* dl = ImGui::GetWindowDrawList();
                    const ImVec2 at(rmax.x - cw - 2.0f, rmax.y - 15.0f);
                    dl->AddText(ImVec2(at.x + 1.0f, at.y + 1.0f), IM_COL32(0, 0, 0, 200), cnt);
                    dl->AddText(at, IM_COL32(255, 255, 255, 230), cnt);
                }

                if (clicked) {
                    gameHandler.detachMailAttachment(i);
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::BeginTooltip();
                    ImGui::TextColored(qualColor, "%s", att.item.name.c_str());
                    if (stack > 1) {
                        ImGui::TextColored(ui::colors::kLightGray, "Stack of %u", stack);
                    }
                    ImGui::TextColored(ui::colors::kLightGray, "Click to remove");
                    ImGui::EndTooltip();
                }
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.15f, 0.15f, 0.5f));
                ImGui::Button("##empty", ImVec2(36, 36));
                ImGui::PopStyleColor();
            }
            ImGui::PopID();
        }

        ImGui::Spacing();
        ImGui::Text("Money:");
        ImGui::SameLine(60);
        ImGui::SetNextItemWidth(60);
        ImGui::InputInt("##MailGold", &mailComposeMoney_[0], 0, 0);
        if (mailComposeMoney_[0] < 0) mailComposeMoney_[0] = 0;
        ImGui::SameLine();
        ImGui::Text("g");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(40);
        ImGui::InputInt("##MailSilver", &mailComposeMoney_[1], 0, 0);
        if (mailComposeMoney_[1] < 0) mailComposeMoney_[1] = 0;
        if (mailComposeMoney_[1] > 99) mailComposeMoney_[1] = 99;
        ImGui::SameLine();
        ImGui::Text("s");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(40);
        ImGui::InputInt("##MailCopper", &mailComposeMoney_[2], 0, 0);
        if (mailComposeMoney_[2] < 0) mailComposeMoney_[2] = 0;
        if (mailComposeMoney_[2] > 99) mailComposeMoney_[2] = 99;
        ImGui::SameLine();
        ImGui::Text("c");

        uint64_t totalMoney = static_cast<uint64_t>(mailComposeMoney_[0]) * 10000 +
                              static_cast<uint64_t>(mailComposeMoney_[1]) * 100 +
                              static_cast<uint64_t>(mailComposeMoney_[2]);

        uint32_t sendCost = attachCount > 0 ? static_cast<uint32_t>(30 * attachCount) : 30u;
        ImGui::TextColored(kColorGray, "Sending cost: %uc", sendCost);

        ImGui::Spacing();
        bool canSend = (strlen(mailRecipientBuffer_) > 0);
        if (!canSend) ImGui::BeginDisabled();
        if (ImGui::Button("Send", ImVec2(80, 0))) {
            gameHandler.sendMail(mailRecipientBuffer_, mailSubjectBuffer_,
                                 mailBodyBuffer_, totalMoney);
        }
        if (!canSend) ImGui::EndDisabled();

        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(80, 0))) {
            gameHandler.closeMailCompose();
        }
    }
    ImGui::End();

    if (!open) {
        gameHandler.closeMailCompose();
    }
}

bool WindowManager::renderBankWindow(game::GameHandler& gameHandler,
                             InventoryScreen& inventoryScreen,
                             ChatPanel& chatPanel) {
    if (!gameHandler.isBankOpen()) return false;

    bool settingChanged = false;
    bool open = true;
    ImGui::SetNextWindowSize(ImVec2(480, 420), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Bank", &open)) {
        ImGui::End();
        if (!open) gameHandler.closeBank();
        return false;
    }

    auto& inv = gameHandler.getInventory();
    bool isHolding = inventoryScreen.isHoldingItem();
    constexpr float SLOT_SIZE = 42.0f;
    static constexpr float kBankPickupHold = 0.10f; // seconds
    // Persistent pickup tracking for bank (mirrors inventory_screen's pickupPending_)
    static bool bankPickupPending = false;
    static float bankPickupPressTime = 0.0f;
    static int bankPickupType = 0; // 0=main bank, 1=bank bag slot, 2=bank bag equip slot
    static int bankPickupIndex = -1;
    static int bankPickupBagIndex = -1;
    static int bankPickupBagSlotIndex = -1;

    // Helper: render a bank item slot with icon, click-and-hold pickup, drop, tooltip
    auto renderBankItemSlot = [&](const game::ItemSlot& slot, int pickType, int mainIdx,
                                   int bagIdx, int bagSlotIdx, uint8_t dstBag, uint8_t dstSlot) {
        ImDrawList* drawList = ImGui::GetWindowDrawList();
        ImVec2 pos = ImGui::GetCursorScreenPos();

        if (slot.empty()) {
            ImU32 bgCol = IM_COL32(30, 30, 30, 200);
            ImU32 borderCol = IM_COL32(60, 60, 60, 200);
            if (isHolding) {
                bgCol = IM_COL32(20, 50, 20, 200);
                borderCol = IM_COL32(0, 180, 0, 200);
            }
            drawList->AddRectFilled(pos, ImVec2(pos.x + SLOT_SIZE, pos.y + SLOT_SIZE), bgCol);
            drawList->AddRect(pos, ImVec2(pos.x + SLOT_SIZE, pos.y + SLOT_SIZE), borderCol);
            ImGui::InvisibleButton("slot", ImVec2(SLOT_SIZE, SLOT_SIZE));
            if (isHolding && ImGui::IsItemHovered() && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                inventoryScreen.dropIntoBankSlot(gameHandler, dstBag, dstSlot);
            } else if (!isHolding && ImGui::IsItemHovered() &&
                       ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                // Dropped out of a handed-over bag. FrameXML picked it up, so
                // InventoryScreen knows nothing about it - its own held item is
                // empty and the branch above never runs. The source arrives in
                // the same flat slot space dropIntoBankSlot converts to, so the
                // swap is the same one.
                uint8_t srcBag = 0, srcSlot = 0;
                if (frameXmlCursorWireSlot(srcBag, srcSlot) &&
                    !(srcBag == dstBag && srcSlot == dstSlot)) {
                    gameHandler.swapContainerItems(srcBag, srcSlot, dstBag, dstSlot);
                    frameXmlPutCursorDown();
                }
            }
        } else {
            const auto& item = slot.item;
            ImVec4 qc = InventoryScreen::getQualityColor(item.quality);
            ImU32 borderCol = ImGui::ColorConvertFloat4ToU32(qc);
            VkDescriptorSet iconTex = inventoryScreen.getItemIcon(item.displayInfoId);

            if (iconTex) {
                drawList->AddImage((ImTextureID)(uintptr_t)iconTex, pos,
                                   ImVec2(pos.x + SLOT_SIZE, pos.y + SLOT_SIZE));
                drawList->AddRect(pos, ImVec2(pos.x + SLOT_SIZE, pos.y + SLOT_SIZE),
                                  borderCol, 0.0f, 0, 2.0f);
            } else {
                ImU32 bgCol = IM_COL32(40, 35, 30, 220);
                drawList->AddRectFilled(pos, ImVec2(pos.x + SLOT_SIZE, pos.y + SLOT_SIZE), bgCol);
                drawList->AddRect(pos, ImVec2(pos.x + SLOT_SIZE, pos.y + SLOT_SIZE),
                                  borderCol, 0.0f, 0, 2.0f);
                if (!item.name.empty()) {
                    char abbr[3] = { item.name[0], item.name.size() > 1 ? item.name[1] : '\0', '\0' };
                    float tw = ImGui::CalcTextSize(abbr).x;
                    drawList->AddText(ImVec2(pos.x + (SLOT_SIZE - tw) * 0.5f, pos.y + 2.0f),
                                      ImGui::ColorConvertFloat4ToU32(qc), abbr);
                }
            }

            if (item.stackCount > 1) {
                char countStr[16];
                snprintf(countStr, sizeof(countStr), "%u", item.stackCount);
                float cw = ImGui::CalcTextSize(countStr).x;
                drawList->AddText(ImVec2(pos.x + SLOT_SIZE - cw - 2.0f, pos.y + SLOT_SIZE - 14.0f),
                                  IM_COL32(255, 255, 255, 220), countStr);
            }

            ImGui::InvisibleButton("slot", ImVec2(SLOT_SIZE, SLOT_SIZE));

            if (!isHolding) {
                // Start pickup tracking on mouse press
                if (ImGui::IsItemClicked(ImGuiMouseButton_Left)) {
                    bankPickupPending = true;
                    bankPickupPressTime = ImGui::GetTime();
                    bankPickupType = pickType;
                    bankPickupIndex = mainIdx;
                    bankPickupBagIndex = bagIdx;
                    bankPickupBagSlotIndex = bagSlotIdx;
                }
                // Check if held long enough to pick up
                if (bankPickupPending && ImGui::IsMouseDown(ImGuiMouseButton_Left) &&
                    (ImGui::GetTime() - bankPickupPressTime) >= kBankPickupHold) {
                    bool sameSlot = (bankPickupType == pickType);
                    if (pickType == 0)
                        sameSlot = sameSlot && (bankPickupIndex == mainIdx);
                    else if (pickType == 1)
                        sameSlot = sameSlot && (bankPickupBagIndex == bagIdx) && (bankPickupBagSlotIndex == bagSlotIdx);
                    else if (pickType == 2)
                        sameSlot = sameSlot && (bankPickupIndex == mainIdx);

                    if (sameSlot && ImGui::IsItemHovered()) {
                        bankPickupPending = false;
                        if (pickType == 0) {
                            inventoryScreen.pickupFromBank(inv, mainIdx);
                        } else if (pickType == 1) {
                            inventoryScreen.pickupFromBankBag(inv, bagIdx, bagSlotIdx);
                        } else if (pickType == 2) {
                            inventoryScreen.pickupFromBankBagEquip(inv, mainIdx);
                        }
                    }
                }
                // Right-click: withdraw the item to the bags (retail auto-store).
                // Maps the slot to its wire address the same way the drag path
                // does (main bank = 0xFF/39+idx, bank bag = 67+bag/slot, bank bag
                // container = 0xFF/67+idx), then lets the server pick a free slot.
                if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right) &&
                    !ImGui::GetIO().KeyShift) {
                    bankPickupPending = false;
                    uint8_t srcBag = 0xFF;
                    uint8_t srcSlot = 0;
                    if (pickType == 1) {
                        srcBag = static_cast<uint8_t>(game::slots::bankBagContainer(bagIdx));
                        srcSlot = static_cast<uint8_t>(bagSlotIdx);
                    } else if (pickType == 2) {
                        srcSlot = static_cast<uint8_t>(game::slots::bankBagWireSlot(mainIdx));
                    } else { // pickType == 0: main bank slot
                        srcSlot = static_cast<uint8_t>(game::slots::bankGeneralWireSlot(mainIdx));
                    }
                    gameHandler.withdrawItem(srcBag, srcSlot);
                }
            } else {
                // Drop/swap on mouse release
                if (ImGui::IsItemHovered() && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                    inventoryScreen.dropIntoBankSlot(gameHandler, dstBag, dstSlot);
                }
            }

            // Tooltip
            if (ImGui::IsItemHovered() && !isHolding) {
                auto* info = gameHandler.getItemInfo(item.itemId);
                if (info && info->valid)
                    inventoryScreen.renderItemTooltip(*info);
                else {
                    ImGui::BeginTooltip();
                    ImGui::TextColored(qc, "%s", item.name.c_str());
                    ImGui::EndTooltip();
                }

                // Shift-click to insert item link into chat
                if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && ImGui::GetIO().KeyShift
                    && !item.name.empty()) {
                    auto* info2 = gameHandler.getItemInfo(item.itemId);
                    uint8_t q = (info2 && info2->valid)
                        ? static_cast<uint8_t>(info2->quality)
                        : static_cast<uint8_t>(item.quality);
                    const std::string& lname = (info2 && info2->valid && !info2->name.empty())
                        ? info2->name : item.name;
                    std::string link = game::itemChatLink(item.itemId, q, lname);
                    chatPanel.insertChatLink(link);
                }
            }
        }
    };

    // Main bank slots (24 for Classic, 28 for TBC/WotLK)
    int bankSlotCount = gameHandler.getEffectiveBankSlots();
    int bankBagCount = gameHandler.getEffectiveBankBagSlots();

    // "Combine bags" is a persisted member (bankCombineBags_) so it survives
    // relaunches.
    //
    // The sort goes through the game handler, which owns the one item sort
    // there is. This kept a static deque of its own and drained it a swap per
    // frame: a function-local static outlives the character it was filled for,
    // so a sort interrupted by a logout resumed against the next character's
    // bank, and it raced the bag sort for a server that takes one swap at a
    // time. The handler refuses a second sort while one is in flight and sends
    // them at the rate the rest of the client does.
    bool sorting = gameHandler.isSortingItems();
    if (sorting) ImGui::BeginDisabled();
    if (ImGui::SmallButton(sorting ? "Sorting..." : "Sort All")) {
        gameHandler.sortBank(bankSlotCount);
    }
    if (sorting) ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("Sort the bank (main slots + bank bags) by quality (highest first),\nthen by item ID, then by stack size.");
    }

    ImGui::SameLine();
    if (ImGui::Checkbox("Combine bags", &bankCombineBags_))
        settingChanged = true;
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Show every bank slot as one continuous grid\ninstead of splitting bank bags into separate sections.");
    }

    ImGui::Separator();

    constexpr int kBankCols = 7;
    if (bankCombineBags_) {
        // Contiguous view: main bank slots followed by every bank bag's contents in one grid.
        ImGui::Text("Bank Slots");
        ImGui::Spacing();
        int col = 0;
        int uid = 0;
        auto placeCell = [&]() {
            if (col % kBankCols != 0) ImGui::SameLine();
            col++;
        };
        for (int i = 0; i < bankSlotCount; i++) {
            placeCell();
            ImGui::PushID(10000 + uid++);
            renderBankItemSlot(inv.getBankSlot(i), 0, i, -1, -1, 0xFF,
                               static_cast<uint8_t>(game::Inventory::BANK_SLOT_START + i));
            ImGui::PopID();
        }
        for (int bagIdx = 0; bagIdx < bankBagCount; bagIdx++) {
            int bagSize = inv.getBankBagSize(bagIdx);
            for (int s = 0; s < bagSize; s++) {
                placeCell();
                ImGui::PushID(10000 + uid++);
                renderBankItemSlot(inv.getBankBagSlot(bagIdx, s), 1, -1, bagIdx, s,
                                   static_cast<uint8_t>(game::Inventory::BANK_BAG_CONTAINER_START + bagIdx),
                                   static_cast<uint8_t>(s));
                ImGui::PopID();
            }
        }
    } else {
        // Grouped view: main bank slots, then one labeled section per bank bag.
        ImGui::Text("Bank Slots");
        ImGui::Spacing();
        for (int i = 0; i < bankSlotCount; i++) {
            if (i % kBankCols != 0) ImGui::SameLine();
            ImGui::PushID(i + 1000);
            renderBankItemSlot(inv.getBankSlot(i), 0, i, -1, -1, 0xFF,
                               static_cast<uint8_t>(game::Inventory::BANK_SLOT_START + i));
            ImGui::PopID();
        }
        for (int bagIdx = 0; bagIdx < bankBagCount; bagIdx++) {
            int bagSize = inv.getBankBagSize(bagIdx);
            if (bagSize <= 0) continue;

            ImGui::Spacing();
            ImGui::Text("Bank Bag %d (%d slots)", bagIdx + 1, bagSize);
            // Sorting the whole bank pools everything into the main slots, so a
            // bag being kept as a category needs its own button to stay one.
            ImGui::SameLine();
            ImGui::PushID(3000 + bagIdx);
            if (sorting) ImGui::BeginDisabled();
            if (ImGui::SmallButton("Sort")) {
                gameHandler.sortBankBag(bagIdx);
            }
            if (sorting) ImGui::EndDisabled();
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                ImGui::SetTooltip("Sort just this bag, leaving the rest of the bank alone.");
            }
            ImGui::PopID();
            for (int s = 0; s < bagSize; s++) {
                if (s % kBankCols != 0) ImGui::SameLine();
                ImGui::PushID(3000 + bagIdx * 100 + s);
                renderBankItemSlot(inv.getBankBagSlot(bagIdx, s), 1, -1, bagIdx, s,
                                   static_cast<uint8_t>(game::Inventory::BANK_BAG_CONTAINER_START + bagIdx),
                                   static_cast<uint8_t>(s));
                ImGui::PopID();
            }
        }
    }

    // Bank bag equip slots - show bag icon with pickup/drop, or a "Buy" button with its price.
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Text("Bank Bags");
    // Deferred confirmation for a bank-bag-slot purchase (opened after the row is drawn).
    static bool bankBuyConfirmOpen = false;
    static uint32_t bankBuyConfirmPrice = 0;
    static int bankBuyConfirmSlot = -1;
    uint8_t purchased = inv.getPurchasedBankBagSlots();
    for (int i = 0; i < bankBagCount; i++) {
        if (i > 0) ImGui::SameLine();
        ImGui::PushID(i + 2000);

        int bagSize = inv.getBankBagSize(i);
        if (i < purchased || bagSize > 0) {
            const auto& bagSlot = inv.getBankBagItem(i);
            // Render as an item slot: icon with pickup/drop (pickType=2 for bag equip)
            renderBankItemSlot(bagSlot, 2, i, -1, -1, 0xFF,
                               static_cast<uint8_t>(game::Inventory::BANK_BAG_CONTAINER_START + i));
        } else {
            // Unpurchased slot: show the price, but only allow buying the next one in sequence.
            uint32_t price = gameHandler.getBankBagSlotPrice(i);
            bool isNext = (i == purchased);
            ImGui::BeginGroup();
            if (!isNext) ImGui::BeginDisabled();
            if (ImGui::Button(isNext ? "Buy" : "Locked", ImVec2(50, 26)) && isNext) {
                // Ask for confirmation rather than spending gold on a single click.
                bankBuyConfirmOpen = true;
                bankBuyConfirmPrice = price;
                bankBuyConfirmSlot = i;
            }
            if (!isNext) ImGui::EndDisabled();
            // Price line under the button
            ui::renderCoinsFromCopper(price);
            ImGui::EndGroup();
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                if (isNext) ImGui::SetTooltip("Purchase this bank bag slot.");
                else ImGui::SetTooltip("Purchase the earlier bank bag slots first.");
            }
        }
        ImGui::PopID();
    }

    // "Are you sure?" dialog for a bank-bag-slot purchase.
    if (bankBuyConfirmOpen) {
        ImGui::OpenPopup("Confirm Bank Slot Purchase##bank");
        bankBuyConfirmOpen = false;
    }
    if (ImGui::BeginPopupModal("Confirm Bank Slot Purchase##bank", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove)) {
        ImGui::Text("Purchase bank bag slot %d?", bankBuyConfirmSlot + 1);
        ImGui::AlignTextToFramePadding();
        ImGui::Text("Cost:");
        ImGui::SameLine();
        ui::renderCoinsFromCopper(bankBuyConfirmPrice);
        ImGui::Spacing();
        if (ImGui::Button("Buy", ImVec2(80, 0))) {
            gameHandler.buyBankSlot();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(80, 0))) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    ImGui::End();

    if (!open) gameHandler.closeBank();
    return settingChanged;
}

void WindowManager::renderGuildBankWindow(game::GameHandler& gameHandler,
                                  InventoryScreen& inventoryScreen,
                                  ChatPanel& chatPanel) {
    if (!gameHandler.isGuildBankOpen()) return;

    bool open = true;
    ImGui::SetNextWindowSize(ImVec2(520, 500), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Guild Bank", &open)) {
        ImGui::End();
        if (!open) gameHandler.closeGuildBank();
        return;
    }

    const auto& data = gameHandler.getGuildBankData();
    uint8_t activeTab = gameHandler.getGuildBankActiveTab();

    // Money display
    const auto coins = game::splitCopper(data.money);
    const uint32_t gold = coins.gold;
    const uint32_t silver = coins.silver;
    const uint32_t copper = coins.copper;
    ImGui::TextDisabled("Guild Bank Money:"); ImGui::SameLine(0, 4);
    renderCoinsText(gold, silver, copper);

    // Tab bar
    if (!data.tabs.empty()) {
        for (size_t i = 0; i < data.tabs.size(); i++) {
            if (i > 0) ImGui::SameLine();
            bool selected = (i == activeTab);
            if (selected) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.5f, 0.8f, 1.0f));
            std::string tabLabel = data.tabs[i].tabName.empty() ? ("Tab " + std::to_string(i + 1)) : data.tabs[i].tabName;
            if (ImGui::Button(tabLabel.c_str())) {
                gameHandler.queryGuildBankTab(static_cast<uint8_t>(i));
            }
            if (selected) ImGui::PopStyleColor();
        }
    }

    // Buy tab button
    if (data.tabs.size() < 6) {
        ImGui::SameLine();
        if (ImGui::Button("Buy Tab")) {
            gameHandler.buyGuildBankTab();
        }
    }

    ImGui::Separator();

    // Fixed 98-slot tab grid (14 columns × 7 rows). The server sends only the
    // slots it has data for (often just the occupied ones, and nothing for an
    // empty tab), so render every slot and look items up by slotId - otherwise
    // an empty or sparsely-populated tab showed no slots at all.
    constexpr float GB_SLOT = 34.0f;
    constexpr int kGuildTabSlots = 98;
    const game::GuildBankItemSlot* slotLookup[kGuildTabSlots] = {};
    for (const auto& it : data.tabItems) {
        if (it.slotId < kGuildTabSlots && it.itemEntry != 0)
            slotLookup[it.slotId] = &it;
    }
    ImDrawList* gbDraw = ImGui::GetWindowDrawList();
    for (int gbSlot = 0; gbSlot < kGuildTabSlots; gbSlot++) {
        if (gbSlot % 14 != 0) ImGui::SameLine(0.0f, 2.0f);
        ImGui::PushID(gbSlot + 5000);

        ImVec2 pos = ImGui::GetCursorScreenPos();
        const game::GuildBankItemSlot* itemPtr = slotLookup[gbSlot];

        if (!itemPtr) {
            gbDraw->AddRectFilled(pos, ImVec2(pos.x + GB_SLOT, pos.y + GB_SLOT),
                                  IM_COL32(30, 30, 30, 200));
            gbDraw->AddRect(pos, ImVec2(pos.x + GB_SLOT, pos.y + GB_SLOT),
                            IM_COL32(60, 60, 60, 180));
            ImGui::InvisibleButton("##gbempty", ImVec2(GB_SLOT, GB_SLOT));
        } else {
            const auto& item = *itemPtr;
            auto* info = gameHandler.getItemInfo(item.itemEntry);
            game::ItemQuality quality = game::ItemQuality::COMMON;
            std::string name = "Item " + std::to_string(item.itemEntry);
            uint32_t displayInfoId = 0;
            if (info) {
                quality = static_cast<game::ItemQuality>(info->quality);
                name = info->name;
                displayInfoId = info->displayInfoId;
            }
            ImVec4 qc = InventoryScreen::getQualityColor(quality);
            ImU32 borderCol = ImGui::ColorConvertFloat4ToU32(qc);

            VkDescriptorSet iconTex = displayInfoId ? inventoryScreen.getItemIcon(displayInfoId) : VK_NULL_HANDLE;
            if (iconTex) {
                gbDraw->AddImage((ImTextureID)(uintptr_t)iconTex, pos,
                                 ImVec2(pos.x + GB_SLOT, pos.y + GB_SLOT));
                gbDraw->AddRect(pos, ImVec2(pos.x + GB_SLOT, pos.y + GB_SLOT),
                                borderCol, 0.0f, 0, 1.5f);
            } else {
                gbDraw->AddRectFilled(pos, ImVec2(pos.x + GB_SLOT, pos.y + GB_SLOT),
                                      IM_COL32(40, 35, 30, 220));
                gbDraw->AddRect(pos, ImVec2(pos.x + GB_SLOT, pos.y + GB_SLOT),
                                borderCol, 0.0f, 0, 1.5f);
                if (!name.empty() && name[0] != 'I') {
                    char abbr[3] = { name[0], name.size() > 1 ? name[1] : '\0', '\0' };
                    float tw = ImGui::CalcTextSize(abbr).x;
                    gbDraw->AddText(ImVec2(pos.x + (GB_SLOT - tw) * 0.5f, pos.y + 2.0f),
                                    borderCol, abbr);
                }
            }

            if (item.stackCount > 1) {
                char cnt[16];
                snprintf(cnt, sizeof(cnt), "%u", item.stackCount);
                float cw = ImGui::CalcTextSize(cnt).x;
                gbDraw->AddText(ImVec2(pos.x + 1.0f, pos.y + 1.0f), IM_COL32(0, 0, 0, 200), cnt);
                gbDraw->AddText(ImVec2(pos.x + GB_SLOT - cw - 2.0f, pos.y + GB_SLOT - 14.0f),
                                IM_COL32(255, 255, 255, 220), cnt);
            }

            ImGui::InvisibleButton("##gbslot", ImVec2(GB_SLOT, GB_SLOT));
            if (ImGui::IsItemClicked(ImGuiMouseButton_Left) && !ImGui::GetIO().KeyShift) {
                gameHandler.guildBankWithdrawItem(activeTab, item.slotId, 0xFF, 0);
            }
            if (ImGui::IsItemHovered()) {
                if (info && info->valid)
                    inventoryScreen.renderItemTooltip(*info);
                // Shift-click to insert item link into chat
                if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && ImGui::GetIO().KeyShift
                    && !name.empty() && item.itemEntry != 0) {
                    uint8_t q = static_cast<uint8_t>(quality);
                    std::string link = game::itemChatLink(item.itemEntry, q, name);
                    chatPanel.insertChatLink(link);
                }
            }
        }
        ImGui::PopID();
    }

    ImGui::Spacing();
    ImGui::TextDisabled("Left-click a slot to withdraw. Right-click a bag item to deposit.");

    // Money deposit/withdraw
    ImGui::Separator();
    ImGui::Text("Money:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(60);
    ImGui::InputInt("##gbg", &guildBankMoneyInput_[0], 0); ImGui::SameLine(); ImGui::Text("g");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(40);
    ImGui::InputInt("##gbs", &guildBankMoneyInput_[1], 0); ImGui::SameLine(); ImGui::Text("s");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(40);
    ImGui::InputInt("##gbc", &guildBankMoneyInput_[2], 0); ImGui::SameLine(); ImGui::Text("c");

    ImGui::SameLine();
    if (ImGui::Button("Deposit")) {
        uint32_t amount = guildBankMoneyInput_[0] * 10000 + guildBankMoneyInput_[1] * 100 + guildBankMoneyInput_[2];
        if (amount > 0) gameHandler.depositGuildBankMoney(amount);
    }
    ImGui::SameLine();
    if (ImGui::Button("Withdraw")) {
        uint32_t amount = guildBankMoneyInput_[0] * 10000 + guildBankMoneyInput_[1] * 100 + guildBankMoneyInput_[2];
        if (amount > 0) gameHandler.withdrawGuildBankMoney(amount);
    }

    if (data.withdrawAmount >= 0) {
        ImGui::Text("Remaining withdrawals: %d", data.withdrawAmount);
    }

    ImGui::End();

    if (!open) gameHandler.closeGuildBank();
}

void WindowManager::renderAuctionHouseWindow(game::GameHandler& gameHandler,
                                     InventoryScreen& inventoryScreen,
                                     ChatPanel& chatPanel) {
    if (!gameHandler.isAuctionHouseOpen()) return;

    bool open = true;
    ImGui::SetNextWindowSize(ImVec2(900, 600), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Auction House", &open)) {
        ImGui::End();
        if (!open) gameHandler.closeAuctionHouse();
        return;
    }

    int tab = gameHandler.getAuctionActiveTab();

    // Draw a list item icon with the WoW-style stack-count badge in the
    // bottom-right corner, so stacked auctions read as multiples (e.g. "20")
    // rather than a bare single-item icon. Leaves the cursor on SameLine for
    // the name that follows, matching the previous inline layout.
    auto drawAuctionIcon = [&inventoryScreen](uint32_t displayInfoId, uint32_t stackCount) -> bool {
        if (displayInfoId == 0) return false;
        VkDescriptorSet icon = inventoryScreen.getItemIcon(displayInfoId);
        if (!icon) return false;
        const float kIconSize = 18.0f;
        ImGui::Image((void*)(intptr_t)icon, ImVec2(kIconSize, kIconSize));
        if (stackCount > 1) {
            char cnt[16];
            snprintf(cnt, sizeof(cnt), "%u", stackCount);
            ImVec2 mn = ImGui::GetItemRectMin();
            ImVec2 mx = ImGui::GetItemRectMax();
            float cw = ImGui::CalcTextSize(cnt).x;
            float ch = ImGui::GetFontSize();
            ImVec2 tp(mx.x - cw - 1.0f, mn.y + (kIconSize - ch));
            ImDrawList* dl = ImGui::GetWindowDrawList();
            dl->AddText(ImVec2(tp.x + 1.0f, tp.y + 1.0f), IM_COL32(0, 0, 0, 210), cnt);
            dl->AddText(tp, IM_COL32(255, 255, 255, 235), cnt);
        }
        ImGui::SameLine();
        return true;
    };

    // Auction results carry an item's rolled random property separately from its
    // template. Build the same instance-aware view used by inventory tooltips so
    // "of the ..." Strength, Stamina, and secondary stat bonuses are visible.
    auto makeAuctionItem = [&gameHandler](const game::ItemQueryResponseData& info,
                                          const game::AuctionEntry& auction) {
        game::ItemDef item;
        item.itemId = auction.itemEntry;
        item.name = info.name;
        item.subclassName = info.subclassName;
        item.quality = static_cast<game::ItemQuality>(info.quality);
        item.inventoryType = static_cast<uint8_t>(info.inventoryType);
        item.stackCount = auction.stackCount;
        item.maxStack = std::max(1, info.maxStack);
        item.bagSlots = info.containerSlots;
        item.damageMin = info.damageMin;
        item.damageMax = info.damageMax;
        item.delayMs = info.delayMs;
        item.armor = info.armor;
        item.stamina = info.stamina;
        item.strength = info.strength;
        item.agility = info.agility;
        item.intellect = info.intellect;
        item.spirit = info.spirit;
        item.displayInfoId = info.displayInfoId;
        item.sellPrice = info.sellPrice;
        item.itemLevel = info.itemLevel;
        item.requiredLevel = info.requiredLevel;
        item.bindType = info.bindType;
        item.description = info.description;
        item.pageTextId = info.pageTextId;
        item.startQuestId = info.startQuestId;
        item.randomPropertyId = static_cast<int32_t>(auction.randomPropertyId);
        for (const auto& stat : info.extraStats)
            item.extraStats.push_back({stat.statType, stat.statValue});

        for (const auto& bonus : gameHandler.getRandomStatBonuses(
                 item.randomPropertyId, auction.suffixFactor)) {
            if (bonus.value == 0) continue;
            switch (bonus.statType) {
                case 3: item.agility += bonus.value; break;
                case 4: item.strength += bonus.value; break;
                case 5: item.intellect += bonus.value; break;
                case 6: item.spirit += bonus.value; break;
                case 7: item.stamina += bonus.value; break;
                default: item.extraStats.push_back({bonus.statType, bonus.value}); break;
            }
        }
        return item;
    };

    // Tab buttons
    const char* tabNames[] = {"Browse", "Bids", "Auctions"};
    for (int i = 0; i < 3; i++) {
        if (i > 0) ImGui::SameLine();
        bool selected = (tab == i);
        if (selected) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.5f, 0.8f, 1.0f));
        if (ImGui::Button(tabNames[i], ImVec2(100, 0))) {
            gameHandler.setAuctionActiveTab(i);
            if (i == 1) gameHandler.auctionListBidderItems();
            else if (i == 2) gameHandler.auctionListOwnerItems();
        }
        if (selected) ImGui::PopStyleColor();
    }

    ImGui::Separator();

    if (tab == 0) {
        // Browse tab - Search filters

        // --- Helper: resolve current UI filter state into wire-format search params ---
        //
        // The tables moved to game/auction_filters.hpp when FrameXML's own
        // filter column needed the identical lists: the index it hands back to
        // QueryAuctionItems has to mean the same thing on the way in as it did
        // on the way out, and two copies of a list whose positions are the
        // protocol drift silently.
        const auto* classMappings = game::kAuctionClasses;
        constexpr int NUM_CLASSES = game::kNumAuctionClasses;
        const auto* weaponSubs = game::kAuctionWeaponSubs;
        constexpr int NUM_WEAPON_SUBS = game::kNumAuctionWeaponSubs;
        const auto* armorSubs = game::kAuctionArmorSubs;
        constexpr int NUM_ARMOR_SUBS = game::kNumAuctionArmorSubs;
        const auto* slotMappings = game::kAuctionSlots;
        constexpr int NUM_AH_SLOTS = game::kNumAuctionSlots;

        auto getSearchClassId = [&]() -> uint32_t {
            if (auctionItemClass_ < 0 || auctionItemClass_ >= NUM_CLASSES) return 0xFFFFFFFF;
            return classMappings[auctionItemClass_].classId;
        };

        auto getSearchSubClassId = [&]() -> uint32_t {
            if (auctionItemSubClass_ < 0) return 0xFFFFFFFF;
            // Stored value is the list row minus 1 (row 0 is "All" → -1), so
            // shift back when indexing - indexing with the stored value
            // directly returned the previous row (2H swords queried as 1H).
            int row = auctionItemSubClass_ + 1;
            uint32_t cid = getSearchClassId();
            if (cid == 2 && row < NUM_WEAPON_SUBS)
                return weaponSubs[row].subId;
            if (cid == 4 && row < NUM_ARMOR_SUBS)
                return armorSubs[row].subId;
            return 0xFFFFFFFF;
        };

        auto getSearchInvType = [&]() -> uint32_t {
            if (auctionSlotFilter_ <= 0 || auctionSlotFilter_ >= NUM_AH_SLOTS)
                return 0xFFFFFFFF;
            return slotMappings[auctionSlotFilter_].invType;
        };

        auto doSearch = [&](uint32_t offset) {
            auctionBrowseOffset_ = offset;
            if (auctionLevelMin_ < 0) auctionLevelMin_ = 0;
            if (auctionLevelMax_ < 0) auctionLevelMax_ = 0;
            uint32_t q = auctionQuality_ > 0 ? static_cast<uint32_t>(auctionQuality_ - 1) : 0xFFFFFFFF;
            gameHandler.auctionSearch(auctionSearchName_,
                static_cast<uint8_t>(auctionLevelMin_),
                static_cast<uint8_t>(auctionLevelMax_),
                q, getSearchClassId(), getSearchSubClassId(), getSearchInvType(),
                auctionUsableOnly_ ? 1 : 0, offset);
        };

        // Original-style browse hierarchy: categories remain visible on the
        // left while search controls and results occupy the right pane.
        if (ImGui::BeginChild("AuctionCategories", ImVec2(185.0f, -1.0f), true)) {
            ImGui::TextUnformatted("Categories");
            ImGui::Separator();

            for (int c = 0; c < NUM_CLASSES; ++c) {
                bool selected = (auctionItemClass_ == c) ||
                                (c == 0 && auctionItemClass_ < 0);
                if (ImGui::Selectable(classMappings[c].label, selected)) {
                    auctionItemClass_ = c;
                    auctionItemSubClass_ = -1;
                    auctionBrowseOffset_ = 0;
                }

                uint32_t classId = classMappings[c].classId;
                if (selected && (classId == 2 || classId == 4)) {
                    const game::AuctionSubFilter* subs = (classId == 2) ? weaponSubs : armorSubs;
                    int numSubs = (classId == 2) ? NUM_WEAPON_SUBS : NUM_ARMOR_SUBS;
                    ImGui::Indent(14.0f);
                    for (int s = 0; s < numSubs; ++s) {
                        // Subclass index 0 is the visible "All" row; the stored
                        // value uses -1 for that wire-level wildcard.
                        int storedSub = s - 1;
                        bool subSelected = auctionItemSubClass_ == storedSub;
                        ImGui::PushID(c * 100 + s);
                        if (ImGui::Selectable(subs[s].label, subSelected)) {
                            auctionItemSubClass_ = storedSub;
                            auctionBrowseOffset_ = 0;
                        }
                        ImGui::PopID();
                    }
                    ImGui::Unindent(14.0f);
                }
            }
        }
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild("AuctionBrowsePane", ImVec2(0.0f, -1.0f), false);

        // Row 1: Name + Level range
        ImGui::SetNextItemWidth(240);
        bool enterPressed = ImGui::InputTextWithHint(
            "##AuctionItemName", "Item name (optional)", auctionSearchName_,
            sizeof(auctionSearchName_), ImGuiInputTextFlags_EnterReturnsTrue);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(50);
        ImGui::InputInt("Min Lv", &auctionLevelMin_, 0);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(50);
        ImGui::InputInt("Max Lv", &auctionLevelMax_, 0);

        // Row 2: quality, equipment slot, and usability refine the category at left.
        const char* qualities[] = {"All", "Poor", "Common", "Uncommon", "Rare", "Epic", "Legendary"};
        ImGui::SetNextItemWidth(100);
        ImGui::Combo("Quality", &auctionQuality_, qualities, 7);
        ImGui::SameLine();
        {
            const char* slotLabels[NUM_AH_SLOTS];
            for (int i = 0; i < NUM_AH_SLOTS; ++i) slotLabels[i] = slotMappings[i].label;
            ImGui::SetNextItemWidth(120);
            ImGui::Combo("Slot", &auctionSlotFilter_, slotLabels, NUM_AH_SLOTS);
        }
        ImGui::SameLine();
        ImGui::Checkbox("Usable", &auctionUsableOnly_);

        // Row 3: client-side refinements (applied to the current page) + Search.
        ImGui::Checkbox("Buyout only", &auctionBuyoutOnly_);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Hide bid-only listings on this page");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(70);
        ImGui::InputInt("Max price (g)", &auctionMaxPriceGold_, 0);
        if (auctionMaxPriceGold_ < 0) auctionMaxPriceGold_ = 0;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Hide items above this buyout on this page (0 = no cap)");
        ImGui::SameLine();
        float delay = gameHandler.getAuctionSearchDelay();
        if (delay > 0.0f) {
            char delayBuf[32];
            snprintf(delayBuf, sizeof(delayBuf), "Search (%.0fs)", delay);
            ImGui::BeginDisabled();
            ImGui::Button(delayBuf);
            ImGui::EndDisabled();
        } else {
            const char* searchLabel = auctionSearchName_[0] == '\0' ? "Browse" : "Search";
            if (ImGui::Button(searchLabel) || enterPressed) {
                doSearch(0);
            }
        }

        ImGui::Separator();

        // Results table
        const auto& results = gameHandler.getAuctionBrowseResults();
        constexpr uint32_t AH_PAGE_SIZE = 50;

        // Client-side page filters (buyout-only / max price). These refine only
        // the current server page - the server pages by 50, so this cannot see
        // beyond it. The header reflects that distinction when a filter is on.
        auto passesClientFilter = [&](const game::AuctionEntry& a) -> bool {
            if (auctionBuyoutOnly_ && a.buyoutPrice == 0) return false;
            if (auctionMaxPriceGold_ > 0) {
                uint64_t maxCopper = static_cast<uint64_t>(auctionMaxPriceGold_) * 10000ull;
                uint32_t price = a.buyoutPrice > 0 ? a.buyoutPrice
                    : (a.currentBid > 0 ? a.currentBid : a.startBid);
                if (static_cast<uint64_t>(price) > maxCopper) return false;
            }
            return true;
        };
        const bool clientFilterActive = auctionBuyoutOnly_ || auctionMaxPriceGold_ > 0;
        if (clientFilterActive) {
            size_t shown = 0;
            for (const auto& a : results.auctions) if (passesClientFilter(a)) ++shown;
            ImGui::Text("%zu shown (of %zu on page, %u total)",
                        shown, results.auctions.size(), results.totalCount);
        } else {
            ImGui::Text("%zu results (of %u total)", results.auctions.size(), results.totalCount);
        }

        // Pagination
        if (results.totalCount > AH_PAGE_SIZE) {
            ImGui::SameLine();
            uint32_t page = auctionBrowseOffset_ / AH_PAGE_SIZE + 1;
            uint32_t totalPages = (results.totalCount + AH_PAGE_SIZE - 1) / AH_PAGE_SIZE;

            if (auctionBrowseOffset_ == 0) ImGui::BeginDisabled();
            if (ImGui::SmallButton("< Prev")) {
                uint32_t newOff = (auctionBrowseOffset_ >= AH_PAGE_SIZE) ? auctionBrowseOffset_ - AH_PAGE_SIZE : 0;
                doSearch(newOff);
            }
            if (auctionBrowseOffset_ == 0) ImGui::EndDisabled();

            ImGui::SameLine();
            ImGui::Text("Page %u/%u", page, totalPages);

            ImGui::SameLine();
            if (auctionBrowseOffset_ + AH_PAGE_SIZE >= results.totalCount) ImGui::BeginDisabled();
            if (ImGui::SmallButton("Next >")) {
                doSearch(auctionBrowseOffset_ + AH_PAGE_SIZE);
            }
            if (auctionBrowseOffset_ + AH_PAGE_SIZE >= results.totalCount) ImGui::EndDisabled();
        }

        if (ImGui::BeginChild("AuctionResults", ImVec2(0, -110), true)) {
            // Column IDs keep sort specs stable independent of column order.
            enum AHColumnId : ImGuiID {
                kAHColItem = 1, kAHColLvl, kAHColRarity, kAHColQty,
                kAHColSeller, kAHColTime, kAHColBid, kAHColBuyout, kAHColAct
            };
            if (ImGui::BeginTable("AuctionTable", 9, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_Sortable)) {
                ImGui::TableSetupColumn("Item", ImGuiTableColumnFlags_WidthStretch, 0.0f, kAHColItem);
                ImGui::TableSetupColumn("Lvl", ImGuiTableColumnFlags_WidthFixed, 36, kAHColLvl);
                ImGui::TableSetupColumn("Rarity", ImGuiTableColumnFlags_WidthFixed, 74, kAHColRarity);
                ImGui::TableSetupColumn("Qty", ImGuiTableColumnFlags_WidthFixed, 40, kAHColQty);
                ImGui::TableSetupColumn("Seller", ImGuiTableColumnFlags_WidthFixed, 95, kAHColSeller);
                ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 60, kAHColTime);
                ImGui::TableSetupColumn("Bid", ImGuiTableColumnFlags_WidthFixed, 90, kAHColBid);
                ImGui::TableSetupColumn("Buyout", ImGuiTableColumnFlags_WidthFixed, 90, kAHColBuyout);
                ImGui::TableSetupColumn("##act", ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_NoSort, 60, kAHColAct);
                ImGui::TableHeadersRow();

                // Client-side sort of the current page (the server pages by
                // 50, so sorting within the page is the best we can do).
                std::vector<size_t> rowOrder(results.auctions.size());
                std::iota(rowOrder.begin(), rowOrder.end(), size_t{0});
                ImGuiTableSortSpecs* sortSpecs = ImGui::TableGetSortSpecs();
                if (sortSpecs && sortSpecs->SpecsCount > 0) {
                    auto infoOf = [&](const game::AuctionEntry& a) {
                        return gameHandler.getItemInfo(a.itemEntry);
                    };
                    auto cmpU32 = [](uint32_t x, uint32_t y) {
                        return x < y ? -1 : (x > y ? 1 : 0);
                    };
                    auto cmpBy = [&](ImGuiID col, const game::AuctionEntry& a,
                                     const game::AuctionEntry& b) -> int {
                        switch (col) {
                            case kAHColItem: {
                                auto* ia = infoOf(a); auto* ib = infoOf(b);
                                int c = (ia ? ia->name : std::string())
                                            .compare(ib ? ib->name : std::string());
                                return c < 0 ? -1 : (c > 0 ? 1 : 0);
                            }
                            case kAHColLvl: {
                                auto* ia = infoOf(a); auto* ib = infoOf(b);
                                return cmpU32(ia ? ia->requiredLevel : 0,
                                              ib ? ib->requiredLevel : 0);
                            }
                            case kAHColRarity: {
                                auto* ia = infoOf(a); auto* ib = infoOf(b);
                                return cmpU32(ia ? ia->quality : 0,
                                              ib ? ib->quality : 0);
                            }
                            case kAHColQty: return cmpU32(a.stackCount, b.stackCount);
                            case kAHColSeller: {
                                int c = gameHandler.getCachedPlayerName(a.ownerGuid)
                                            .compare(gameHandler.getCachedPlayerName(b.ownerGuid));
                                return c < 0 ? -1 : (c > 0 ? 1 : 0);
                            }
                            case kAHColTime: return cmpU32(a.timeLeftMs, b.timeLeftMs);
                            case kAHColBid:
                                return cmpU32(a.currentBid > 0 ? a.currentBid : a.startBid,
                                              b.currentBid > 0 ? b.currentBid : b.startBid);
                            case kAHColBuyout: return cmpU32(a.buyoutPrice, b.buyoutPrice);
                            default: return 0;
                        }
                    };
                    std::stable_sort(rowOrder.begin(), rowOrder.end(),
                        [&](size_t ia, size_t ib) {
                            const auto& a = results.auctions[ia];
                            const auto& b = results.auctions[ib];
                            for (int k = 0; k < sortSpecs->SpecsCount; ++k) {
                                const auto& s = sortSpecs->Specs[k];
                                int c = cmpBy(s.ColumnUserID, a, b);
                                if (c != 0)
                                    return s.SortDirection == ImGuiSortDirection_Ascending
                                        ? c < 0 : c > 0;
                            }
                            return false;
                        });
                }

                for (size_t row = 0; row < rowOrder.size(); row++) {
                    const size_t i = rowOrder[row];
                    const auto& auction = results.auctions[i];
                    if (!passesClientFilter(auction)) continue;
                    auto* info = gameHandler.getItemInfo(auction.itemEntry);
                    std::string name = info ? info->name : ("Item #" + std::to_string(auction.itemEntry));
                    // Append random suffix name (e.g., "of the Eagle") if present
                    if (auction.randomPropertyId != 0) {
                        std::string suffix = gameHandler.getRandomPropertyName(
                            static_cast<int32_t>(auction.randomPropertyId));
                        if (!suffix.empty()) name += " " + suffix;
                    }
                    game::ItemQuality quality = info ? static_cast<game::ItemQuality>(info->quality) : game::ItemQuality::COMMON;
                    ImVec4 qc = InventoryScreen::getQualityColor(quality);

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    // Item icon with stack-count badge
                    if (info && info->valid)
                        drawAuctionIcon(info->displayInfoId, auction.stackCount);
                    ImGui::TextColored(qc, "%s", name.c_str());
                    // Item tooltip on hover; hold Shift to compare against the
                    // equipped item in the same slot; shift-click inserts a link.
                    if (ImGui::IsItemHovered() && info && info->valid) {
                        inventoryScreen.renderItemTooltip(
                            makeAuctionItem(*info, auction), &gameHandler.getInventory());
                    }
                    if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
                        ImGui::GetIO().KeyShift && info && info->valid && !info->name.empty()) {
                        std::string link = game::itemChatLink(info->entry, info->quality, info->name);
                        chatPanel.insertChatLink(link);
                    }

                    ImGui::TableSetColumnIndex(1);
                    if (info && info->requiredLevel > 0)
                        ImGui::Text("%u", info->requiredLevel);
                    else
                        ImGui::TextDisabled("--");

                    ImGui::TableSetColumnIndex(2);
                    {
                        static const char* qualityNames[] = {
                            "Poor", "Common", "Uncommon", "Rare",
                            "Epic", "Legendary", "Artifact"};
                        uint32_t qi = info ? info->quality : 1;
                        if (qi > 6) qi = 6;
                        ImGui::TextColored(qc, "%s", qualityNames[qi]);
                    }

                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("%u", auction.stackCount);

                    ImGui::TableSetColumnIndex(4);
                    if (auction.ownerGuid == gameHandler.getPlayerGuid()) {
                        ImGui::TextUnformatted("You");
                    } else {
                        std::string seller = gameHandler.getCachedPlayerName(auction.ownerGuid);
                        if (!seller.empty())
                            ImGui::TextUnformatted(seller.c_str());
                        else
                            ImGui::TextDisabled("Loading...");
                    }

                    ImGui::TableSetColumnIndex(5);
                    // Time left display
                    uint32_t mins = auction.timeLeftMs / 60000;
                    if (mins > 720) ImGui::Text("Long");
                    else if (mins > 120) ImGui::Text("Medium");
                    else ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "Short");

                    ImGui::TableSetColumnIndex(6);
                    {
                        uint32_t bid = auction.currentBid > 0 ? auction.currentBid : auction.startBid;
                        renderCoinsFromCopper(bid);
                    }

                    ImGui::TableSetColumnIndex(7);
                    if (auction.buyoutPrice > 0) {
                        renderCoinsFromCopper(auction.buyoutPrice);
                    } else {
                        ImGui::TextDisabled("--");
                    }

                    ImGui::TableSetColumnIndex(8);
                    ImGui::PushID(static_cast<int>(i) + 7000);
                    if (auction.buyoutPrice > 0 && ImGui::SmallButton("Buy")) {
                        gameHandler.auctionBuyout(auction.auctionId, auction.buyoutPrice);
                    }
                    if (auction.buyoutPrice > 0) ImGui::SameLine();
                    if (ImGui::SmallButton("Bid")) {
                        uint32_t bidAmt = auction.currentBid > 0
                            ? auction.currentBid + auction.minBidIncrement
                            : auction.startBid;
                        gameHandler.auctionPlaceBid(auction.auctionId, bidAmt);
                    }
                    ImGui::PopID();
                }
                ImGui::EndTable();
            }
        }
        ImGui::EndChild();

        // Sell section
        ImGui::Separator();
        ImGui::Text("Sell Item:");

        // Item picker: every non-empty slot across the backpack and all equipped bags.
        {
            auto& inv = gameHandler.getInventory();
            // Resolve the ItemSlot for a (bagIndex, slotIndex) pair. bagIndex -1 is the backpack.
            auto slotAt = [&](int bagIndex, int slotIndex) -> const game::ItemSlot& {
                return (bagIndex < 0) ? inv.getBackpackSlot(slotIndex)
                                      : inv.getBagSlot(bagIndex, slotIndex);
            };

            std::string preview = "Select item...";
            if (auctionSellSlotIndex_ >= 0) {
                const auto& slot = slotAt(auctionSellBagIndex_, auctionSellSlotIndex_);
                if (!slot.empty()) {
                    preview = slot.item.name;
                    if (slot.item.stackCount > 1) preview += " x" + std::to_string(slot.item.stackCount);
                }
            }

            // Render one selectable row for a slot; centralised so backpack and bag loops share it.
            int comboUid = 0;
            auto renderItemRow = [&](int bagIndex, int slotIndex) {
                const auto& slot = slotAt(bagIndex, slotIndex);
                if (slot.empty()) return;
                ImGui::PushID(9000 + comboUid++);
                if (slot.item.displayInfoId != 0) {
                    VkDescriptorSet sIcon = inventoryScreen.getItemIcon(slot.item.displayInfoId);
                    if (sIcon) {
                        ImGui::Image((void*)(intptr_t)sIcon, ImVec2(16, 16));
                        ImGui::SameLine();
                    }
                }
                std::string label = slot.item.name;
                if (slot.item.stackCount > 1) label += " x" + std::to_string(slot.item.stackCount);
                ImVec4 iqc = InventoryScreen::getQualityColor(slot.item.quality);
                ImGui::PushStyleColor(ImGuiCol_Text, iqc);
                bool selected = (auctionSellBagIndex_ == bagIndex && auctionSellSlotIndex_ == slotIndex);
                if (ImGui::Selectable(label.c_str(), selected)) {
                    auctionSellBagIndex_ = bagIndex;
                    auctionSellSlotIndex_ = slotIndex;
                }
                ImGui::PopStyleColor();
                ImGui::PopID();
            };

            ImGui::SetNextItemWidth(250);
            if (ImGui::BeginCombo("##sellitem", preview.c_str())) {
                for (int i = 0; i < game::Inventory::BACKPACK_SLOTS; i++)
                    renderItemRow(-1, i);
                for (int b = 0; b < game::Inventory::NUM_BAG_SLOTS; b++) {
                    int bagSize = inv.getBagSize(b);
                    for (int s = 0; s < bagSize; s++)
                        renderItemRow(b, s);
                }
                ImGui::EndCombo();
            }
        }

        ImGui::Text("Bid:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(50);
        ImGui::InputInt("##sbg", &auctionSellBid_[0], 0); ImGui::SameLine(); ImGui::Text("g");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(35);
        ImGui::InputInt("##sbs", &auctionSellBid_[1], 0); ImGui::SameLine(); ImGui::Text("s");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(35);
        ImGui::InputInt("##sbc", &auctionSellBid_[2], 0); ImGui::SameLine(); ImGui::Text("c");

        ImGui::SameLine(0, 20);
        ImGui::Text("Buyout:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(50);
        ImGui::InputInt("##sbog", &auctionSellBuyout_[0], 0); ImGui::SameLine(); ImGui::Text("g");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(35);
        ImGui::InputInt("##sbos", &auctionSellBuyout_[1], 0); ImGui::SameLine(); ImGui::Text("s");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(35);
        ImGui::InputInt("##sboc", &auctionSellBuyout_[2], 0); ImGui::SameLine(); ImGui::Text("c");

        const char* durations[] = {"12 hours", "24 hours", "48 hours"};
        ImGui::SetNextItemWidth(90);
        ImGui::Combo("##dur", &auctionSellDuration_, durations, 3);
        ImGui::SameLine();

        // Create Auction button
        // Resolve the currently selected slot (backpack or an equipped bag) for validation + posting.
        const game::ItemSlot* sellSlot = nullptr;
        if (auctionSellSlotIndex_ >= 0) {
            auto& inv = gameHandler.getInventory();
            sellSlot = (auctionSellBagIndex_ < 0)
                ? &inv.getBackpackSlot(auctionSellSlotIndex_)
                : &inv.getBagSlot(auctionSellBagIndex_, auctionSellSlotIndex_);
        }
        bool canCreate = sellSlot && !sellSlot->empty() && sellSlot->item.guid != 0 &&
                         (auctionSellBid_[0] > 0 || auctionSellBid_[1] > 0 || auctionSellBid_[2] > 0);
        if (!canCreate) ImGui::BeginDisabled();
        if (ImGui::Button("Create Auction")) {
            uint32_t bidCopper = static_cast<uint32_t>(auctionSellBid_[0]) * 10000
                               + static_cast<uint32_t>(auctionSellBid_[1]) * 100
                               + static_cast<uint32_t>(auctionSellBid_[2]);
            uint32_t buyoutCopper = static_cast<uint32_t>(auctionSellBuyout_[0]) * 10000
                                  + static_cast<uint32_t>(auctionSellBuyout_[1]) * 100
                                  + static_cast<uint32_t>(auctionSellBuyout_[2]);
            uint32_t dur = game::kAuctionDurationMinutes[auctionSellDuration_];
            gameHandler.auctionSellItemByGuid(sellSlot->item.guid, sellSlot->item.stackCount,
                                              bidCopper, buyoutCopper, dur);
            // Clear sell inputs
            auctionSellSlotIndex_ = -1;
            auctionSellBagIndex_ = -1;
            auctionSellBid_[0] = auctionSellBid_[1] = auctionSellBid_[2] = 0;
            auctionSellBuyout_[0] = auctionSellBuyout_[1] = auctionSellBuyout_[2] = 0;
        }
        if (!canCreate) ImGui::EndDisabled();

        ImGui::EndChild(); // AuctionBrowsePane

    } else if (tab == 1) {
        // Bids tab
        const auto& results = gameHandler.getAuctionBidderResults();
        ImGui::Text("Your Bids: %zu items", results.auctions.size());

        if (ImGui::BeginTable("BidTable", 6, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn("Item", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("Qty", ImGuiTableColumnFlags_WidthFixed, 40);
            ImGui::TableSetupColumn("Your Bid", ImGuiTableColumnFlags_WidthFixed, 90);
            ImGui::TableSetupColumn("Buyout", ImGuiTableColumnFlags_WidthFixed, 90);
            ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 60);
            ImGui::TableSetupColumn("##act", ImGuiTableColumnFlags_WidthFixed, 60);
            ImGui::TableHeadersRow();

            for (size_t bi = 0; bi < results.auctions.size(); bi++) {
                const auto& a = results.auctions[bi];
                auto* info = gameHandler.getItemInfo(a.itemEntry);
                std::string name = info ? info->name : ("Item #" + std::to_string(a.itemEntry));
                if (a.randomPropertyId != 0) {
                    std::string suffix = gameHandler.getRandomPropertyName(
                        static_cast<int32_t>(a.randomPropertyId));
                    if (!suffix.empty()) name += " " + suffix;
                }
                game::ItemQuality quality = info ? static_cast<game::ItemQuality>(info->quality) : game::ItemQuality::COMMON;
                ImVec4 bqc = InventoryScreen::getQualityColor(quality);

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                if (info && info->valid)
                    drawAuctionIcon(info->displayInfoId, a.stackCount);
                // High bidder indicator
                bool isHighBidder = (a.bidderGuid != 0 && a.bidderGuid == gameHandler.getPlayerGuid());
                if (isHighBidder) {
                    ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.2f, 1.0f), "[Winning]");
                    ImGui::SameLine();
                } else if (a.bidderGuid != 0) {
                    ImGui::TextColored(ImVec4(0.9f, 0.3f, 0.3f, 1.0f), "[Outbid]");
                    ImGui::SameLine();
                }
                ImGui::TextColored(bqc, "%s", name.c_str());
                // Tooltip and shift-click
                if (ImGui::IsItemHovered() && info && info->valid)
                    inventoryScreen.renderItemTooltip(*info);
                if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
                    ImGui::GetIO().KeyShift && info && info->valid && !info->name.empty()) {
                    std::string link = game::itemChatLink(info->entry, info->quality, info->name);
                    chatPanel.insertChatLink(link);
                }
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%u", a.stackCount);
                ImGui::TableSetColumnIndex(2);
                renderCoinsFromCopper(a.currentBid);
                ImGui::TableSetColumnIndex(3);
                if (a.buyoutPrice > 0)
                    renderCoinsFromCopper(a.buyoutPrice);
                else
                    ImGui::TextDisabled("--");
                ImGui::TableSetColumnIndex(4);
                uint32_t mins = a.timeLeftMs / 60000;
                if (mins > 720) ImGui::Text("Long");
                else if (mins > 120) ImGui::Text("Medium");
                else ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "Short");

                ImGui::TableSetColumnIndex(5);
                ImGui::PushID(static_cast<int>(bi) + 7500);
                if (a.buyoutPrice > 0 && ImGui::SmallButton("Buy")) {
                    gameHandler.auctionBuyout(a.auctionId, a.buyoutPrice);
                }
                if (a.buyoutPrice > 0) ImGui::SameLine();
                if (ImGui::SmallButton("Bid")) {
                    uint32_t bidAmt = a.currentBid > 0
                        ? a.currentBid + a.minBidIncrement
                        : a.startBid;
                    gameHandler.auctionPlaceBid(a.auctionId, bidAmt);
                }
                ImGui::PopID();
            }
            ImGui::EndTable();
        }

    } else if (tab == 2) {
        // Auctions tab (your listings)
        const auto& results = gameHandler.getAuctionOwnerResults();
        ImGui::Text("Your Auctions: %zu items", results.auctions.size());

        if (ImGui::BeginTable("OwnerTable", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn("Item", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("Qty", ImGuiTableColumnFlags_WidthFixed, 40);
            ImGui::TableSetupColumn("Bid", ImGuiTableColumnFlags_WidthFixed, 90);
            ImGui::TableSetupColumn("Buyout", ImGuiTableColumnFlags_WidthFixed, 90);
            ImGui::TableSetupColumn("##cancel", ImGuiTableColumnFlags_WidthFixed, 60);
            ImGui::TableHeadersRow();

            for (size_t i = 0; i < results.auctions.size(); i++) {
                const auto& a = results.auctions[i];
                auto* info = gameHandler.getItemInfo(a.itemEntry);
                std::string name = info ? info->name : ("Item #" + std::to_string(a.itemEntry));
                if (a.randomPropertyId != 0) {
                    std::string suffix = gameHandler.getRandomPropertyName(
                        static_cast<int32_t>(a.randomPropertyId));
                    if (!suffix.empty()) name += " " + suffix;
                }
                game::ItemQuality quality = info ? static_cast<game::ItemQuality>(info->quality) : game::ItemQuality::COMMON;

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImVec4 oqc = InventoryScreen::getQualityColor(quality);
                if (info && info->valid)
                    drawAuctionIcon(info->displayInfoId, a.stackCount);
                // Bid activity indicator for seller
                if (a.bidderGuid != 0) {
                    ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.2f, 1.0f), "[Bid]");
                    ImGui::SameLine();
                }
                ImGui::TextColored(oqc, "%s", name.c_str());
                if (ImGui::IsItemHovered() && info && info->valid)
                    inventoryScreen.renderItemTooltip(*info);
                if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
                    ImGui::GetIO().KeyShift && info && info->valid && !info->name.empty()) {
                    std::string link = game::itemChatLink(info->entry, info->quality, info->name);
                    chatPanel.insertChatLink(link);
                }
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%u", a.stackCount);
                ImGui::TableSetColumnIndex(2);
                {
                    uint32_t bid = a.currentBid > 0 ? a.currentBid : a.startBid;
                    renderCoinsFromCopper(bid);
                }
                ImGui::TableSetColumnIndex(3);
                if (a.buyoutPrice > 0)
                    renderCoinsFromCopper(a.buyoutPrice);
                else
                    ImGui::TextDisabled("--");
                ImGui::TableSetColumnIndex(4);
                ImGui::PushID(static_cast<int>(i) + 8000);
                if (ImGui::SmallButton("Cancel")) {
                    gameHandler.auctionCancelItem(a.auctionId);
                }
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
    }

    ImGui::End();

    if (!open) gameHandler.closeAuctionHouse();
}

void WindowManager::renderInstanceLockouts(game::GameHandler& gameHandler) {
    if (!showInstanceLockouts_) return;

    ImGui::SetNextWindowSize(ImVec2(480, 0), ImGuiCond_Appearing);
    ImGui::SetNextWindowPos(
        ImVec2(ImGui::GetIO().DisplaySize.x / 2 - 240, 140), ImGuiCond_Appearing);

    if (!ImGui::Begin("Instance Lockouts", &showInstanceLockouts_,
                      ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }

    const auto& lockouts = gameHandler.getInstanceLockouts();

    if (lockouts.empty()) {
        ImGui::TextColored(kColorGray, "No active instance lockouts.");
    } else {
        auto difficultyLabel = [](uint32_t diff) -> const char* {
            switch (diff) {
                case 0: return "Normal";
                case 1: return "Heroic";
                case 2: return "25-Man";
                case 3: return "25-Man Heroic";
                default: return "Unknown";
            }
        };

        // Current UTC time for reset countdown
        auto nowSec = static_cast<uint64_t>(std::time(nullptr));

        if (ImGui::BeginTable("lockouts", 4,
                              ImGuiTableFlags_SizingStretchProp |
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersOuter)) {
            ImGui::TableSetupColumn("Instance",   ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("Difficulty", ImGuiTableColumnFlags_WidthFixed, 110.0f);
            ImGui::TableSetupColumn("Resets In",  ImGuiTableColumnFlags_WidthFixed, 100.0f);
            ImGui::TableSetupColumn("Status",     ImGuiTableColumnFlags_WidthFixed, 60.0f);
            ImGui::TableHeadersRow();

            for (const auto& lo : lockouts) {
                ImGui::TableNextRow();

                // Instance name - use GameHandler's Map.dbc cache (avoids duplicate DBC load)
                ImGui::TableSetColumnIndex(0);
                std::string mapName = gameHandler.getMapName(lo.mapId);
                if (!mapName.empty()) {
                    ImGui::TextUnformatted(mapName.c_str());
                } else {
                    ImGui::Text("Map %u", lo.mapId);
                }

                // Difficulty
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(difficultyLabel(lo.difficulty));

                // Reset countdown
                ImGui::TableSetColumnIndex(2);
                if (lo.resetTime > nowSec) {
                    uint64_t remaining = lo.resetTime - nowSec;
                    uint64_t days  = remaining / 86400;
                    uint64_t hours = (remaining % 86400) / 3600;
                    if (days > 0) {
                        ImGui::Text("%llud %lluh",
                            static_cast<unsigned long long>(days),
                            static_cast<unsigned long long>(hours));
                    } else {
                        uint64_t mins = (remaining % 3600) / 60;
                        ImGui::Text("%lluh %llum",
                            static_cast<unsigned long long>(hours),
                            static_cast<unsigned long long>(mins));
                    }
                } else {
                    ImGui::TextColored(kColorDarkGray, "Expired");
                }

                // Locked / Extended status
                ImGui::TableSetColumnIndex(3);
                if (lo.extended) {
                    ImGui::TextColored(ImVec4(0.3f, 0.7f, 1.0f, 1.0f), "Ext");
                } else if (lo.locked) {
                    ImGui::TextColored(colors::kSoftRed, "Locked");
                } else {
                    ImGui::TextColored(ImVec4(0.5f, 0.9f, 0.5f, 1.0f), "Open");
                }
            }

            ImGui::EndTable();
        }
    }

    ImGui::End();
}

// ============================================================================
// Battleground score frame
//
// Displays the current score for the player's battleground using world states.
// Shown in the top-centre of the screen whenever SMSG_INIT_WORLD_STATES has
// been received for a known BG map.  The layout adapts per battleground:
//
//   WSG  489 – Alliance / Horde flag captures (max 3)
//   AB   529 – Alliance / Horde resource scores (max 1600)
//   AV    30 – Alliance / Horde reinforcements
//   EotS 566 – Alliance / Horde resource scores (max 1600)
// ============================================================================
// ─── Who Results Window ───────────────────────────────────────────────────────
// ─── Combat Log Window ────────────────────────────────────────────────────────




// ─── Inspect Window ───────────────────────────────────────────────────────────
// ─── Titles Window ────────────────────────────────────────────────────────────
void WindowManager::renderTitlesWindow(game::GameHandler& gameHandler) {
    if (!showTitlesWindow_) return;

    ImGui::SetNextWindowSize(ImVec2(320, 400), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(240, 170), ImGuiCond_FirstUseEver);

    if (!ImGui::Begin("Titles", &showTitlesWindow_)) {
        ImGui::End();
        return;
    }

    const auto& knownBits = gameHandler.getKnownTitleBits();
    const int32_t chosen  = gameHandler.getChosenTitleBit();

    if (knownBits.empty()) {
        ImGui::TextDisabled("No titles earned yet.");
        ImGui::End();
        return;
    }

    ImGui::TextUnformatted("Select a title to display:");
    ImGui::Separator();

    // "No Title" option
    bool noTitle = (chosen < 0);
    if (ImGui::Selectable("(No Title)", noTitle)) {
        if (!noTitle) gameHandler.sendSetTitle(-1);
    }
    if (noTitle) {
        ImGui::SameLine();
        ImGui::TextColored(colors::kBrightGold, "<-- active");
    }

    ImGui::Separator();

    // Sort known bits for stable display order
    std::vector<uint32_t> sortedBits(knownBits.begin(), knownBits.end());
    std::sort(sortedBits.begin(), sortedBits.end());

    ImGui::BeginChild("##titlelist", ImVec2(0, 0), false);
    for (uint32_t bit : sortedBits) {
        const std::string title = gameHandler.getFormattedTitle(bit);
        const std::string display = title.empty()
            ? ("Title #" + std::to_string(bit)) : title;

        bool isActive = (chosen >= 0 && static_cast<uint32_t>(chosen) == bit);
        ImGui::PushID(static_cast<int>(bit));

        if (isActive) {
            ImGui::PushStyleColor(ImGuiCol_Text, colors::kBrightGold);
        }
        if (ImGui::Selectable(display.c_str(), isActive)) {
            if (!isActive) gameHandler.sendSetTitle(static_cast<int32_t>(bit));
        }
        if (isActive) {
            ImGui::PopStyleColor();
            ImGui::SameLine();
            ImGui::TextDisabled("<-- active");
        }

        ImGui::PopID();
    }
    ImGui::EndChild();

    ImGui::End();
}

// ─── Equipment Set Manager Window ─────────────────────────────────────────────
// Nothing sets showEquipSetWindow_ true, anywhere - this window has no way to
// be opened and never draws. Left rather than removed because the equipment-set
// packets it reads were repaired this week and FrameXML's GearManagerDialog is
// the path that reaches them now: the manager is behind the equipmentManager
// CVar, which the interface options can set and which persists since cvars.cfg
// exists. If this client ever wants its own again, the window is here.
void WindowManager::renderEquipSetWindow(game::GameHandler& gameHandler) {
    if (!showEquipSetWindow_) return;

    ImGui::SetNextWindowSize(ImVec2(280, 320), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(260, 180), ImGuiCond_FirstUseEver);

    if (!ImGui::Begin("Equipment Sets##equipsets", &showEquipSetWindow_)) {
        ImGui::End();
        return;
    }

    const auto& sets = gameHandler.getEquipmentSets();

    if (sets.empty()) {
        ImGui::TextDisabled("No equipment sets saved.");
        ImGui::Spacing();
        ImGui::TextWrapped("Create equipment sets in-game using the default WoW equipment manager (Shift+click the Equipment Sets button).");
        ImGui::End();
        return;
    }

    ImGui::TextUnformatted("Click a set to equip it:");
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::BeginChild("##equipsetlist", ImVec2(0, 0), false);
    for (const auto& set : sets) {
        ImGui::PushID(static_cast<int>(set.setId));

        // Icon placeholder (use a coloured square if no icon texture available)
        ImVec2 iconSize(32.0f, 32.0f);
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.25f, 0.20f, 0.10f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.40f, 0.30f, 0.15f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.60f, 0.45f, 0.20f, 1.0f));
        if (ImGui::Button("##icon", iconSize)) {
            gameHandler.useEquipmentSet(set.setId);
        }
        ImGui::PopStyleColor(3);

        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Equip set: %s", set.name.c_str());
        }

        ImGui::SameLine();

        // Name and equip button
        ImGui::BeginGroup();
        ImGui::TextUnformatted(set.name.c_str());
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.35f, 0.15f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.30f, 0.50f, 0.22f, 1.0f));
        if (ImGui::SmallButton("Equip")) {
            gameHandler.useEquipmentSet(set.setId);
        }
        ImGui::PopStyleColor(2);
        ImGui::EndGroup();

        ImGui::Spacing();
        ImGui::PopID();
    }
    ImGui::EndChild();

    ImGui::End();
}

void WindowManager::renderSkillsWindow(game::GameHandler& gameHandler) {
    if (!showSkillsWindow_) return;

    ImGui::SetNextWindowSize(ImVec2(380, 480), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(220, 130), ImGuiCond_FirstUseEver);

    if (!ImGui::Begin("Skills & Professions", &showSkillsWindow_)) {
        ImGui::End();
        return;
    }

    const auto& skills = gameHandler.getPlayerSkills();
    if (skills.empty()) {
        ImGui::TextDisabled("No skill data received yet.");
        ImGui::End();
        return;
    }

    // Organise skills by category
    // WoW SkillLine.dbc categories: 6=Weapon, 7=Class, 8=Armor, 9=Secondary, 11=Professions, others=Misc
    struct SkillEntry {
        uint32_t skillId;
        const game::PlayerSkill* skill;
    };
    std::map<uint32_t, std::vector<SkillEntry>> byCategory;
    for (const auto& [id, sk] : skills) {
        uint32_t cat = gameHandler.getSkillCategory(id);
        byCategory[cat].push_back({id, &sk});
    }

    static constexpr struct { uint32_t cat; const char* label; } kCatOrder[] = {
        {11, "Professions"},
        { 9, "Secondary Skills"},
        { 7, "Class Skills"},
        { 6, "Weapon Skills"},
        { 8, "Armor"},
        { 5, "Languages"},
        { 0, "Other"},
    };

    // Collect handled categories to fall back to "Other" for unknowns
    static constexpr uint32_t kKnownCats[] = {11, 9, 7, 6, 8, 5};

    // Redirect unknown categories into bucket 0
    for (auto& [cat, vec] : byCategory) {
        bool known = false;
        for (uint32_t kc : kKnownCats) if (cat == kc) { known = true; break; }
        if (!known && cat != 0) {
            auto& other = byCategory[0];
            other.insert(other.end(), vec.begin(), vec.end());
            vec.clear();
        }
    }

    ImGui::BeginChild("##skillscroll", ImVec2(0, 0), false);

    for (const auto& [cat, label] : kCatOrder) {
        auto it = byCategory.find(cat);
        if (it == byCategory.end() || it->second.empty()) continue;

        auto& entries = it->second;
        // Sort alphabetically within each category
        std::sort(entries.begin(), entries.end(), [&](const SkillEntry& a, const SkillEntry& b) {
            return gameHandler.getSkillName(a.skillId) < gameHandler.getSkillName(b.skillId);
        });

        if (ImGui::CollapsingHeader(label, ImGuiTreeNodeFlags_DefaultOpen)) {
            for (const auto& e : entries) {
                const std::string& name = gameHandler.getSkillName(e.skillId);
                const char* displayName = name.empty() ? "Unknown" : name.c_str();
                uint16_t val = e.skill->effectiveValue();
                uint16_t maxVal = e.skill->maxValue;

                ImGui::PushID(static_cast<int>(e.skillId));

                // Name column
                ImGui::TextUnformatted(displayName);
                ImGui::SameLine(170.0f);

                // Progress bar
                float fraction = (maxVal > 0) ? static_cast<float>(val) / static_cast<float>(maxVal) : 0.0f;
                char overlay[32];
                snprintf(overlay, sizeof(overlay), "%u / %u", val, maxVal);
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.20f, 0.55f, 0.20f, 1.0f));
                ImGui::ProgressBar(fraction, ImVec2(160.0f, 14.0f), overlay);
                ImGui::PopStyleColor();

                if (ImGui::IsItemHovered()) {
                    ImGui::BeginTooltip();
                    ImGui::Text("%s", displayName);
                    ImGui::Separator();
                    ImGui::Text("Base: %u", e.skill->value);
                    if (e.skill->bonusPerm > 0)
                        ImGui::Text("Permanent bonus: +%u", e.skill->bonusPerm);
                    if (e.skill->bonusTemp > 0)
                        ImGui::Text("Temporary bonus: +%u", e.skill->bonusTemp);
                    ImGui::Text("Max: %u", maxVal);
                    ImGui::EndTooltip();
                }

                ImGui::PopID();
            }
            ImGui::Spacing();
        }
    }

    ImGui::EndChild();
    ImGui::End();
}



// Recipe difficulty against the player's current skill, in the classic
// tradeskill colours. The rank comes from GameHandler so this and the
// interface's own trade skill panel cannot disagree about a recipe.
namespace {
constexpr ImVec4 kDiffOrange(1.0f, 0.5f, 0.0f, 1.0f);
constexpr ImVec4 kDiffYellow(1.0f, 1.0f, 0.0f, 1.0f);
constexpr ImVec4 kDiffGreen(0.3f, 0.8f, 0.3f, 1.0f);
constexpr ImVec4 kDiffGray(0.5f, 0.5f, 0.5f, 1.0f);
}  // namespace

ImVec4 WindowManager::recipeDifficultyColor(game::GameHandler& gameHandler, uint32_t spellId) {
    switch (gameHandler.getRecipeDifficulty(spellId)) {
        case 3:  return kDiffGray;
        case 2:  return kDiffGreen;
        case 1:  return kDiffYellow;
        default: return kDiffOrange;
    }
}

const char* WindowManager::recipeDifficultyLabel(game::GameHandler& gameHandler, uint32_t spellId) {
    switch (gameHandler.getRecipeDifficulty(spellId)) {
        case 3:  return "Trivial";
        case 2:  return "Easy";
        case 1:  return "Medium";
        default: return "Optimal";
    }
}

} // namespace ui
} // namespace wowee
