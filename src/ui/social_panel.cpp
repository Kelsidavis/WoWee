// ============================================================
// SocialPanel - extracted from GameScreen
// Owns all social/group-related UI rendering: party frames,
// boss frames, guild roster, social/friends frame, dungeon finder,
// who window, inspect window.
// ============================================================
#include "ui/framexml_takeover.hpp"
#include "ui/social_panel.hpp"
#include "ui/quest_log_screen.hpp"
#include "ui/ui_raid_icons.hpp"
#include "ui/chat_panel.hpp"
#include "ui/spellbook_screen.hpp"
#include "ui/inventory_screen.hpp"
#include "ui/ui_colors.hpp"
#include "ui/ui_helpers.hpp"
#include "core/application.hpp"
#include "core/logger.hpp"
#include "rendering/renderer.hpp"
#include "game/game_handler.hpp"
#include "game/game_utils.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_layout.hpp"
#include "ui/keybinding_manager.hpp"
#include "game/zone_manager.hpp"
#include <imgui.h>
#include <imgui_internal.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>

namespace {
    using namespace wowee::ui::colors;
    using namespace wowee::ui::helpers;
    constexpr auto& kColorRed         = kRed;
    constexpr auto& kColorGray        = kGray;
    constexpr auto& kColorDarkGray    = kDarkGray;
} // anonymous namespace

namespace wowee {
namespace ui {


void SocialPanel::renderGuildRoster(game::GameHandler& gameHandler,
                                       ChatPanel& chatPanel,
                                       InventoryScreen& inventoryScreen,
                                       SpellbookScreen& spellbookScreen,
                                       QuestLogScreen& questLogScreen,
                                       const SpellIconFn& getSpellIcon) {
    // Guild Roster toggle (customizable keybind)
    if (!chatPanel.isChatInputActive() && !ImGui::GetIO().WantTextInput &&
        !ImGui::GetIO().WantCaptureKeyboard &&
        KeybindingManager::getInstance().isActionPressed(KeybindingManager::Action::TOGGLE_GUILD_ROSTER)) {
        showGuildRoster_ = !showGuildRoster_;
        if (showGuildRoster_) {
            // Open friends tab directly if not in guild
            if (!gameHandler.isInGuild()) {
                guildRosterTab_ = 2;  // Friends tab
            } else {
                // Re-query guild name if we have guildId but no name yet
                if (gameHandler.getGuildName().empty()) {
                    const auto* ch = gameHandler.getActiveCharacter();
                    if (ch && ch->hasGuild()) {
                        gameHandler.queryGuildInfo(ch->guildId);
                    }
                }
                gameHandler.requestGuildRoster();
                gameHandler.requestGuildInfo();
            }
        }
    }

    // Charters belong to FrameXML when the element is handed over: the same
    // packets these flags are set by also fire GUILD_REGISTRAR_SHOW,
    // PETITION_VENDOR_SHOW and PETITION_SHOW, and FrameXML raises a window on
    // each. Without this gate every charter bought or signed asked twice.
    //
    // The flag is consumed either way. Left set it would sit true behind the
    // handed-over window and open this one the moment the element came back.
    const bool ownsPetitionUi = !frameXmlOwns(UiElement::Petition);

    // Petition creation dialog (shown when NPC sends SMSG_PETITION_SHOWLIST)
    if (gameHandler.hasPetitionShowlist()) {
        if (ownsPetitionUi) ImGui::OpenPopup("CreateGuildPetition");
        gameHandler.clearPetitionDialog();
    }
    if (ownsPetitionUi &&
        ImGui::BeginPopupModal("CreateGuildPetition", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("Create Guild Charter");
        ImGui::Separator();
        uint32_t cost = gameHandler.getPetitionCost();
        ImGui::TextDisabled("Cost:"); ImGui::SameLine(0, 4);
        renderCoinsFromCopper(cost);
        ImGui::Spacing();
        ImGui::Text("Guild Name:");
        ImGui::InputText("##petitionname", petitionNameBuffer_, sizeof(petitionNameBuffer_));
        ImGui::Spacing();
        if (ImGui::Button("Create", ImVec2(120, 0))) {
            if (petitionNameBuffer_[0] != '\0') {
                gameHandler.buyPetition(gameHandler.getPetitionNpcGuid(), petitionNameBuffer_);
                petitionNameBuffer_[0] = '\0';
                ImGui::CloseCurrentPopup();
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(120, 0))) {
            petitionNameBuffer_[0] = '\0';
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    // Petition signatures window (shown when a petition item is used or offered)
    if (gameHandler.hasPetitionSignaturesUI()) {
        if (ownsPetitionUi) ImGui::OpenPopup("PetitionSignatures");
        gameHandler.clearPetitionSignaturesUI();
    }
    if (ownsPetitionUi &&
        ImGui::BeginPopupModal("PetitionSignatures", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        const auto& pInfo = gameHandler.getPetitionInfo();
        if (!pInfo.guildName.empty())
            ImGui::Text("Guild Charter: %s", pInfo.guildName.c_str());
        else
            ImGui::Text("Guild Charter");
        ImGui::Separator();

        ImGui::Text("Signatures: %u / %u", pInfo.signatureCount, pInfo.signaturesRequired);
        ImGui::Spacing();

        if (!pInfo.signatures.empty()) {
            for (size_t i = 0; i < pInfo.signatures.size(); ++i) {
                const auto& sig = pInfo.signatures[i];
                // Try to resolve name from entity manager
                std::string sigName;
                if (sig.playerGuid != 0) {
                    auto entity = gameHandler.getEntityManager().getEntity(sig.playerGuid);
                    if (entity) {
                        auto* unit = entity->isUnit() ? static_cast<game::Unit*>(entity.get()) : nullptr;
                        if (unit) sigName = unit->getName();
                    }
                }
                if (sigName.empty())
                    sigName = "Player " + std::to_string(i + 1);
                ImGui::BulletText("%s", sigName.c_str());
            }
            ImGui::Spacing();
        }

        // If we're not the owner, show Sign button
        bool isOwner = (pInfo.ownerGuid == gameHandler.getPlayerGuid());
        if (!isOwner) {
            if (ImGui::Button("Sign", ImVec2(120, 0))) {
                gameHandler.signPetition(pInfo.petitionGuid);
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
        } else if (pInfo.signatureCount >= pInfo.signaturesRequired) {
            // Owner with enough sigs - turn in
            if (ImGui::Button("Turn In", ImVec2(120, 0))) {
                gameHandler.turnInPetition(pInfo.petitionGuid);
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
        }
        if (ImGui::Button("Close", ImVec2(120, 0)))
            ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    if (!showGuildRoster_) return;

    // Get zone manager for name lookup
    game::ZoneManager* zoneManager = nullptr;
    if (auto* renderer = services_.renderer) {
        zoneManager = renderer->getZoneManager();
    }

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 375, screenH / 2 - 250), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(750, 500), ImGuiCond_Once);

    std::string title = gameHandler.isInGuild() ? (gameHandler.getGuildName() + " - Social") : "Social";
    bool open = showGuildRoster_;
    if (ImGui::Begin(title.c_str(), &open, ImGuiWindowFlags_NoCollapse)) {
        // Tab bar: Roster | Guild Info
        if (ImGui::BeginTabBar("GuildTabs")) {
            if (ImGui::BeginTabItem("Roster")) {
                guildRosterTab_ = 0;
                if (!gameHandler.hasGuildRoster()) {
                    ImGui::Text("Loading roster...");
                } else {
                    const auto& roster = gameHandler.getGuildRoster();

                    // MOTD
                    if (!roster.motd.empty()) {
                        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "MOTD: %s", roster.motd.c_str());
                        ImGui::Separator();
                    }

                    // Count online
                    int onlineCount = 0;
                    for (const auto& m : roster.members) {
                        if (m.online) ++onlineCount;
                    }
                    ImGui::Text("%d members (%d online)", static_cast<int>(roster.members.size()), onlineCount);
                    ImGui::Separator();

                    const auto& rankNames = gameHandler.getGuildRankNames();

                    // Table
                    if (ImGui::BeginTable("GuildRoster", 7,
                            ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                            ImGuiTableFlags_Sortable)) {
                        ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_DefaultSort);
                        ImGui::TableSetupColumn("Rank");
                        ImGui::TableSetupColumn("Level", ImGuiTableColumnFlags_WidthFixed, 40.0f);
                        ImGui::TableSetupColumn("Class", ImGuiTableColumnFlags_WidthFixed, 70.0f);
                        ImGui::TableSetupColumn("Zone", ImGuiTableColumnFlags_WidthFixed, 120.0f);
                        ImGui::TableSetupColumn("Note");
                        ImGui::TableSetupColumn("Officer Note");
                        ImGui::TableHeadersRow();

                        // Online members first, then offline
                        auto sortedMembers = roster.members;
                        std::sort(sortedMembers.begin(), sortedMembers.end(), [](const auto& a, const auto& b) {
                            if (a.online != b.online) return a.online > b.online;
                            return a.name < b.name;
                        });

                        for (const auto& m : sortedMembers) {
                            ImGui::TableNextRow();
                            ImVec4 textColor = m.online ? ui::colors::kWhite
                                                        : kColorDarkGray;
                            ImVec4 nameColor = m.online ? classColorVec4(m.classId) : textColor;

                            ImGui::TableNextColumn();
                            ImGui::TextColored(nameColor, "%s", m.name.c_str());

                            // Right-click context menu
                            if (ImGui::IsItemClicked(ImGuiMouseButton_Right)) {
                                selectedGuildMember_ = m.name;
                                ImGui::OpenPopup("GuildMemberContext");
                            }

                            ImGui::TableNextColumn();
                            // Show rank name instead of index
                            if (m.rankIndex < rankNames.size()) {
                                ImGui::TextColored(textColor, "%s", rankNames[m.rankIndex].c_str());
                            } else {
                                ImGui::TextColored(textColor, "Rank %u", m.rankIndex);
                            }

                            ImGui::TableNextColumn();
                            ImGui::TextColored(textColor, "%u", m.level);

                            ImGui::TableNextColumn();
                            const char* className = classNameStr(m.classId);
                            ImVec4 classCol = m.online ? classColorVec4(m.classId) : textColor;
                            ImGui::TextColored(classCol, "%s", className);

                            ImGui::TableNextColumn();
                            // Zone name lookup
                            if (zoneManager) {
                                const auto* zoneInfo = zoneManager->getZoneInfo(m.zoneId);
                                if (zoneInfo && !zoneInfo->name.empty()) {
                                    ImGui::TextColored(textColor, "%s", zoneInfo->name.c_str());
                                } else {
                                    ImGui::TextColored(textColor, "%u", m.zoneId);
                                }
                            } else {
                                ImGui::TextColored(textColor, "%u", m.zoneId);
                            }

                            ImGui::TableNextColumn();
                            ImGui::TextColored(textColor, "%s", m.publicNote.c_str());

                            ImGui::TableNextColumn();
                            ImGui::TextColored(textColor, "%s", m.officerNote.c_str());
                        }
                        ImGui::EndTable();
                    }

                    // Context menu popup
                    if (ImGui::BeginPopup("GuildMemberContext")) {
                        ImGui::TextDisabled("%s", selectedGuildMember_.c_str());
                        ImGui::Separator();
                        // Social actions - only for online members
                        bool memberOnline = false;
                        for (const auto& mem : roster.members) {
                            if (mem.name == selectedGuildMember_) { memberOnline = mem.online; break; }
                        }
                        if (memberOnline) {
                            if (ImGui::MenuItem("Whisper")) {
                                chatPanel.setWhisperTarget(selectedGuildMember_);
                            }
                            if (ImGui::MenuItem("Invite to Group")) {
                                gameHandler.inviteToGroup(selectedGuildMember_);
                            }
                            ImGui::Separator();
                        }
                        if (!selectedGuildMember_.empty()) {
                            if (ImGui::MenuItem("Add Friend"))
                                gameHandler.addFriend(selectedGuildMember_);
                            if (ImGui::MenuItem("Ignore"))
                                gameHandler.addIgnore(selectedGuildMember_);
                            ImGui::Separator();
                        }
                        if (ImGui::MenuItem("Promote")) {
                            gameHandler.promoteGuildMember(selectedGuildMember_);
                        }
                        if (ImGui::MenuItem("Demote")) {
                            gameHandler.demoteGuildMember(selectedGuildMember_);
                        }
                        if (ImGui::MenuItem("Kick")) {
                            gameHandler.kickGuildMember(selectedGuildMember_);
                        }
                        ImGui::Separator();
                        if (ImGui::MenuItem("Set Public Note...")) {
                            showGuildNoteEdit_ = true;
                            editingOfficerNote_ = false;
                            guildNoteEditBuffer_[0] = '\0';
                            // Pre-fill with existing note
                            for (const auto& mem : roster.members) {
                                if (mem.name == selectedGuildMember_) {
                                    snprintf(guildNoteEditBuffer_, sizeof(guildNoteEditBuffer_), "%s", mem.publicNote.c_str());
                                    break;
                                }
                            }
                        }
                        if (ImGui::MenuItem("Set Officer Note...")) {
                            showGuildNoteEdit_ = true;
                            editingOfficerNote_ = true;
                            guildNoteEditBuffer_[0] = '\0';
                            for (const auto& mem : roster.members) {
                                if (mem.name == selectedGuildMember_) {
                                    snprintf(guildNoteEditBuffer_, sizeof(guildNoteEditBuffer_), "%s", mem.officerNote.c_str());
                                    break;
                                }
                            }
                        }
                        ImGui::Separator();
                        if (ImGui::MenuItem("Set as Leader")) {
                            gameHandler.setGuildLeader(selectedGuildMember_);
                        }
                        ImGui::EndPopup();
                    }

                    // Note edit modal
                    if (showGuildNoteEdit_) {
                        ImGui::OpenPopup("EditGuildNote");
                        showGuildNoteEdit_ = false;
                    }
                    if (ImGui::BeginPopupModal("EditGuildNote", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
                        ImGui::Text("%s %s for %s:",
                            editingOfficerNote_ ? "Officer" : "Public", "Note", selectedGuildMember_.c_str());
                        ImGui::InputText("##guildnote", guildNoteEditBuffer_, sizeof(guildNoteEditBuffer_));
                        if (ImGui::Button("Save")) {
                            if (editingOfficerNote_) {
                                gameHandler.setGuildOfficerNote(selectedGuildMember_, guildNoteEditBuffer_);
                            } else {
                                gameHandler.setGuildPublicNote(selectedGuildMember_, guildNoteEditBuffer_);
                            }
                            ImGui::CloseCurrentPopup();
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("Cancel")) {
                            ImGui::CloseCurrentPopup();
                        }
                        ImGui::EndPopup();
                    }
                }
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Guild Info")) {
                guildRosterTab_ = 1;
                const auto& infoData = gameHandler.getGuildInfoData();
                const auto& queryData = gameHandler.getGuildQueryData();
                const auto& roster = gameHandler.getGuildRoster();
                const auto& rankNames = gameHandler.getGuildRankNames();

                // Guild name (large, gold)
                ImGui::PushFont(nullptr);  // default font
                ImGui::TextColored(ui::colors::kTooltipGold, "<%s>", gameHandler.getGuildName().c_str());
                ImGui::PopFont();
                ImGui::Separator();

                // Creation date
                if (infoData.isValid()) {
                    ImGui::Text("Created: %u/%u/%u", infoData.creationDay, infoData.creationMonth, infoData.creationYear);
                    ImGui::Text("Members: %u  |  Accounts: %u", infoData.numMembers, infoData.numAccounts);
                }
                ImGui::Spacing();

                // Both of these are free text the guild types, so they carry
                // the same markup chat does - item, spell, quest and
                // achievement links, colour codes and URLs. Drawn as plain
                // text the markup showed as raw |H escapes and nothing was
                // clickable, so they go through the chat renderer instead.
                MarkupRenderContext markupCtx;
                markupCtx.gameHandler  = &gameHandler;
                markupCtx.inventory    = &inventoryScreen;
                markupCtx.spellbook    = &spellbookScreen;
                markupCtx.questLog     = &questLogScreen;
                markupCtx.assetMgr     = services_.assetManager;
                markupCtx.getSpellIcon = getSpellIcon;
                markupCtx.insertLink   = [&chatPanel](const std::string& link) {
                    chatPanel.insertChatLink(link);
                };

                // Guild description / info text
                if (!roster.guildInfo.empty()) {
                    ImGui::TextColored(colors::kSilver, "Description:");
                    guildMarkupRenderer_.render(
                        guildMarkupParser_.parse(roster.guildInfo),
                        ImGui::GetStyleColorVec4(ImGuiCol_Text), markupCtx);
                }
                ImGui::Spacing();

                // MOTD with edit button
                ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "MOTD:");
                ImGui::SameLine();
                if (!roster.motd.empty()) {
                    guildMarkupRenderer_.render(
                        guildMarkupParser_.parse(roster.motd),
                        ImGui::GetStyleColorVec4(ImGuiCol_Text), markupCtx);
                } else {
                    ImGui::TextColored(kColorDarkGray, "(not set)");
                }
                if (ImGui::Button("Set MOTD")) {
                    showMotdEdit_ = true;
                    snprintf(guildMotdEditBuffer_, sizeof(guildMotdEditBuffer_), "%s", roster.motd.c_str());
                }
                ImGui::Spacing();

                // MOTD edit modal
                if (showMotdEdit_) {
                    ImGui::OpenPopup("EditMotd");
                    showMotdEdit_ = false;
                }
                if (ImGui::BeginPopupModal("EditMotd", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
                    ImGui::Text("Set Message of the Day:");
                    ImGui::InputText("##motdinput", guildMotdEditBuffer_, sizeof(guildMotdEditBuffer_));
                    if (ImGui::Button("Save", ImVec2(120, 0))) {
                        gameHandler.setGuildMotd(guildMotdEditBuffer_);
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Cancel", ImVec2(120, 0))) {
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::EndPopup();
                }

                // Emblem info
                if (queryData.isValid()) {
                    ImGui::Separator();
                    ImGui::Text("Emblem: Style %u, Color %u  |  Border: Style %u, Color %u  |  BG: %u",
                        queryData.emblemStyle, queryData.emblemColor,
                        queryData.borderStyle, queryData.borderColor, queryData.backgroundColor);
                }

                // Rank list
                ImGui::Separator();
                ImGui::TextColored(ui::colors::kTooltipGold, "Ranks:");
                for (size_t i = 0; i < rankNames.size(); ++i) {
                    if (rankNames[i].empty()) continue;
                    // Show rank permission summary from roster data
                    if (i < roster.ranks.size()) {
                        uint32_t rights = roster.ranks[i].rights;
                        std::string perms;
                        if (rights & 0x01) perms += "Invite ";
                        if (rights & 0x02) perms += "Remove ";
                        if (rights & 0x40) perms += "Promote ";
                        if (rights & 0x80) perms += "Demote ";
                        if (rights & 0x04) perms += "OChat ";
                        if (rights & 0x10) perms += "MOTD ";
                        ImGui::Text("  %zu. %s", i + 1, rankNames[i].c_str());
                        if (!perms.empty()) {
                            ImGui::SameLine();
                            ImGui::TextColored(kColorDarkGray, "[%s]", perms.c_str());
                        }
                    } else {
                        ImGui::Text("  %zu. %s", i + 1, rankNames[i].c_str());
                    }
                }

                // Rank management buttons
                ImGui::Spacing();
                if (ImGui::Button("Add Rank")) {
                    showAddRankModal_ = true;
                    addRankNameBuffer_[0] = '\0';
                }
                ImGui::SameLine();
                if (ImGui::Button("Delete Last Rank")) {
                    gameHandler.deleteGuildRank();
                }

                // Add rank modal
                if (showAddRankModal_) {
                    ImGui::OpenPopup("AddGuildRank");
                    showAddRankModal_ = false;
                }
                if (ImGui::BeginPopupModal("AddGuildRank", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
                    ImGui::Text("New Rank Name:");
                    ImGui::InputText("##rankname", addRankNameBuffer_, sizeof(addRankNameBuffer_));
                    if (ImGui::Button("Add", ImVec2(120, 0))) {
                        if (addRankNameBuffer_[0] != '\0') {
                            gameHandler.addGuildRank(addRankNameBuffer_);
                            ImGui::CloseCurrentPopup();
                        }
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Cancel", ImVec2(120, 0))) {
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::EndPopup();
                }

                ImGui::EndTabItem();
            }

            // ---- Friends tab ----
            if (ImGui::BeginTabItem("Friends")) {
                guildRosterTab_ = 2;
                const auto& contacts = gameHandler.getContacts();

                // Add Friend row
                static char addFriendBuf[64] = {};
                ImGui::SetNextItemWidth(180.0f);
                ImGui::InputText("##addfriend", addFriendBuf, sizeof(addFriendBuf));
                ImGui::SameLine();
                if (ImGui::Button("Add Friend") && addFriendBuf[0] != '\0') {
                    gameHandler.addFriend(addFriendBuf);
                    addFriendBuf[0] = '\0';
                }
                ImGui::Separator();

                // Note-edit state
                static std::string friendNoteTarget;
                static char friendNoteBuf[256] = {};
                static bool openNotePopup = false;

                // Filter to friends only
                int friendCount = 0;
                for (size_t ci = 0; ci < contacts.size(); ++ci) {
                    const auto& c = contacts[ci];
                    if (!c.isFriend()) continue;
                    ++friendCount;

                    ImGui::PushID(static_cast<int>(ci));

                    // Status dot
                    ImU32 dotColor = c.isOnline()
                        ? IM_COL32(80, 200, 80, 255)
                        : IM_COL32(120, 120, 120, 255);
                    ImVec2 cursor = ImGui::GetCursorScreenPos();
                    ImGui::GetWindowDrawList()->AddCircleFilled(
                        ImVec2(cursor.x + 6.0f, cursor.y + 8.0f), 5.0f, dotColor);
                    ImGui::Dummy(ImVec2(14.0f, 0.0f));
                    ImGui::SameLine();

                    // Name as Selectable for right-click context menu
                    const char* displayName = c.name.empty() ? "(unknown)" : c.name.c_str();
                    ImVec4 nameCol = c.isOnline()
                        ? ui::colors::kWhite
                        : colors::kInactiveGray;
                    ImGui::PushStyleColor(ImGuiCol_Text, nameCol);
                    ImGui::Selectable(displayName, false, ImGuiSelectableFlags_AllowOverlap, ImVec2(130.0f, 0.0f));
                    ImGui::PopStyleColor();

                    // Double-click to whisper
                    if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)
                        && !c.name.empty()) {
                        chatPanel.setWhisperTarget(c.name);
                    }

                    // Right-click context menu
                    if (ImGui::BeginPopupContextItem("FriendCtx")) {
                        ImGui::TextDisabled("%s", displayName);
                        ImGui::Separator();
                        if (ImGui::MenuItem("Whisper") && !c.name.empty()) {
                            chatPanel.setWhisperTarget(c.name);
                        }
                        if (c.isOnline() && ImGui::MenuItem("Invite to Group") && !c.name.empty()) {
                            gameHandler.inviteToGroup(c.name);
                        }
                        if (ImGui::MenuItem("Edit Note")) {
                            friendNoteTarget = c.name;
                            strncpy(friendNoteBuf, c.note.c_str(), sizeof(friendNoteBuf) - 1);
                            friendNoteBuf[sizeof(friendNoteBuf) - 1] = '\0';
                            openNotePopup = true;
                        }
                        ImGui::Separator();
                        if (ImGui::MenuItem("Remove Friend")) {
                            gameHandler.removeFriend(c.name);
                        }
                        ImGui::EndPopup();
                    }

                    // Note tooltip on hover
                    if (ImGui::IsItemHovered() && !c.note.empty()) {
                        ImGui::BeginTooltip();
                        ImGui::TextDisabled("Note: %s", c.note.c_str());
                        ImGui::EndTooltip();
                    }

                    // Level, class, and status
                    if (c.isOnline()) {
                        ImGui::SameLine(150.0f);
                        const char* statusLabel =
                            (c.status == 2) ? " (AFK)" :
                            (c.status == 3) ? " (DND)" : "";
                        // Class color for the level/class display
                        ImVec4 friendClassCol = classColorVec4(static_cast<uint8_t>(c.classId));
                        const char* friendClassName = classNameStr(static_cast<uint8_t>(c.classId));
                        if (c.level > 0 && c.classId > 0) {
                            ImGui::TextColored(friendClassCol, "Lv%u %s%s", c.level, friendClassName, statusLabel);
                        } else if (c.level > 0) {
                            ImGui::TextDisabled("Lv %u%s", c.level, statusLabel);
                        } else if (*statusLabel) {
                            ImGui::TextDisabled("%s", statusLabel + 1);
                        }

                        // Tooltip: zone info
                        if (ImGui::IsItemHovered() && c.areaId != 0) {
                            ImGui::BeginTooltip();
                            if (zoneManager) {
                                const auto* zi = zoneManager->getZoneInfo(c.areaId);
                                if (zi && !zi->name.empty())
                                    ImGui::Text("Zone: %s", zi->name.c_str());
                                else
                                    ImGui::TextDisabled("Area ID: %u", c.areaId);
                            } else {
                                ImGui::TextDisabled("Area ID: %u", c.areaId);
                            }
                            ImGui::EndTooltip();
                        }
                    }

                    ImGui::PopID();
                }

                if (friendCount == 0) {
                    ImGui::TextDisabled("No friends found.");
                }

                // Note edit modal
                if (openNotePopup) {
                    ImGui::OpenPopup("EditFriendNote");
                    openNotePopup = false;
                }
                if (ImGui::BeginPopupModal("EditFriendNote", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
                    ImGui::Text("Note for %s:", friendNoteTarget.c_str());
                    ImGui::SetNextItemWidth(240.0f);
                    ImGui::InputText("##fnote", friendNoteBuf, sizeof(friendNoteBuf));
                    if (ImGui::Button("Save", ImVec2(110, 0))) {
                        gameHandler.setFriendNote(friendNoteTarget, friendNoteBuf);
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Cancel", ImVec2(110, 0))) {
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::EndPopup();
                }

                ImGui::EndTabItem();
            }

            // ---- Ignore List tab ----
            if (ImGui::BeginTabItem("Ignore")) {
                guildRosterTab_ = 3;
                const auto& contacts = gameHandler.getContacts();

                // Add Ignore row
                static char addIgnoreBuf[64] = {};
                ImGui::SetNextItemWidth(180.0f);
                ImGui::InputText("##addignore", addIgnoreBuf, sizeof(addIgnoreBuf));
                ImGui::SameLine();
                if (ImGui::Button("Ignore Player") && addIgnoreBuf[0] != '\0') {
                    gameHandler.addIgnore(addIgnoreBuf);
                    addIgnoreBuf[0] = '\0';
                }
                ImGui::Separator();

                int ignoreCount = 0;
                for (size_t ci = 0; ci < contacts.size(); ++ci) {
                    const auto& c = contacts[ci];
                    if (!c.isIgnored()) continue;
                    ++ignoreCount;

                    ImGui::PushID(static_cast<int>(ci) + 10000);
                    const char* displayName = c.name.empty() ? "(unknown)" : c.name.c_str();
                    ImGui::Selectable(displayName, false, ImGuiSelectableFlags_AllowOverlap);
                    if (ImGui::BeginPopupContextItem("IgnoreCtx")) {
                        ImGui::TextDisabled("%s", displayName);
                        ImGui::Separator();
                        if (ImGui::MenuItem("Remove Ignore")) {
                            gameHandler.removeIgnore(c.name);
                        }
                        ImGui::EndPopup();
                    }
                    ImGui::PopID();
                }

                if (ignoreCount == 0) {
                    ImGui::TextDisabled("Ignore list is empty.");
                }

                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }
    }
    ImGui::End();
    showGuildRoster_ = open;
}

void SocialPanel::renderSocialFrame(game::GameHandler& gameHandler,
                                       ChatPanel& chatPanel) {
    if (!showSocialFrame_) return;

    const auto& contacts = gameHandler.getContacts();
    // Count online friends for early-out
    int onlineCount = 0;
    for (const auto& c : contacts)
        if (c.isFriend() && c.isOnline()) ++onlineCount;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW - 230.0f, 240.0f), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(220.0f, 0.0f), ImGuiCond_Always);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 0.92f));

    // State for "Set Note" inline editing
    static int  noteEditContactIdx = -1;
    static char noteEditBuf[128]   = {};

    bool open = showSocialFrame_;
    char socialTitle[32];
    snprintf(socialTitle, sizeof(socialTitle), "Social (%d online)##SocialFrame", onlineCount);
    if (ImGui::Begin(socialTitle, &open,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar)) {

        // Get zone manager for area name lookups
        game::ZoneManager* socialZoneMgr = nullptr;
        if (auto* rend = services_.renderer)
            socialZoneMgr = rend->getZoneManager();

        if (ImGui::BeginTabBar("##SocialTabs")) {
            // ---- Friends tab ----
            if (ImGui::BeginTabItem("Friends")) {
                ImGui::BeginChild("##FriendsList", ImVec2(200, 200), false);

                // Online friends first
                int shown = 0;
                for (int pass = 0; pass < 2; ++pass) {
                    bool wantOnline = (pass == 0);
                    for (size_t ci = 0; ci < contacts.size(); ++ci) {
                        const auto& c = contacts[ci];
                        if (!c.isFriend()) continue;
                        if (c.isOnline() != wantOnline) continue;

                        ImGui::PushID(static_cast<int>(ci));

                        // Status dot
                        ImU32 dotColor;
                        if (!c.isOnline())        dotColor = IM_COL32(100, 100, 100, 200);
                        else if (c.status == 2)   dotColor = IM_COL32(255, 200,  50, 255); // AFK
                        else if (c.status == 3)   dotColor = IM_COL32(255, 120,  50, 255); // DND
                        else                      dotColor = IM_COL32( 50, 220,  50, 255); // online

                        ImVec2 dotMin = ImGui::GetCursorScreenPos();
                        dotMin.y += 4.0f;
                        ImGui::GetWindowDrawList()->AddCircleFilled(
                            ImVec2(dotMin.x + 5.0f, dotMin.y + 5.0f), 4.5f, dotColor);
                        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 14.0f);

                        const char* displayName = c.name.empty() ? "(unknown)" : c.name.c_str();
                        ImVec4 nameCol = c.isOnline()
                            ? classColorVec4(static_cast<uint8_t>(c.classId))
                            : kColorDarkGray;
                        ImGui::TextColored(nameCol, "%s", displayName);

                        if (c.isOnline() && c.level > 0) {
                            ImGui::SameLine();
                            // Show level and class name in class color
                            ImGui::TextColored(classColorVec4(static_cast<uint8_t>(c.classId)),
                                "Lv%u %s", c.level, classNameStr(static_cast<uint8_t>(c.classId)));
                        }

                        // Tooltip: zone info and note
                        if (ImGui::IsItemHovered() || (c.isOnline() && ImGui::IsItemHovered())) {
                            if (c.isOnline() && (c.areaId != 0 || !c.note.empty())) {
                                ImGui::BeginTooltip();
                                if (c.areaId != 0) {
                                    const char* zoneName = nullptr;
                                    if (socialZoneMgr) {
                                        const auto* zi = socialZoneMgr->getZoneInfo(c.areaId);
                                        if (zi && !zi->name.empty()) zoneName = zi->name.c_str();
                                    }
                                    if (zoneName)
                                        ImGui::Text("Zone: %s", zoneName);
                                    else
                                        ImGui::Text("Area ID: %u", c.areaId);
                                }
                                if (!c.note.empty())
                                    ImGui::TextDisabled("Note: %s", c.note.c_str());
                                ImGui::EndTooltip();
                            }
                        }

                        // Right-click context menu
                        if (ImGui::BeginPopupContextItem("FriendCtx")) {
                            ImGui::TextDisabled("%s", displayName);
                            ImGui::Separator();
                            if (c.isOnline()) {
                                if (ImGui::MenuItem("Whisper")) {
                                    showSocialFrame_ = false;
                                    chatPanel.setWhisperTarget(c.name);
                                }
                                if (ImGui::MenuItem("Invite to Group"))
                                    gameHandler.inviteToGroup(c.name);
                                if (c.guid != 0 && ImGui::MenuItem("Trade"))
                                    gameHandler.initiateTrade(c.guid);
                            }
                            if (ImGui::MenuItem("Set Note")) {
                                noteEditContactIdx = static_cast<int>(ci);
                                strncpy(noteEditBuf, c.note.c_str(), sizeof(noteEditBuf) - 1);
                                noteEditBuf[sizeof(noteEditBuf) - 1] = '\0';
                                ImGui::OpenPopup("##SetFriendNote");
                            }
                            if (ImGui::MenuItem("Remove Friend"))
                                gameHandler.removeFriend(c.name);
                            ImGui::EndPopup();
                        }

                        ++shown;
                        ImGui::PopID();
                    }
                    // Separator between online and offline if there are both
                    if (pass == 0 && shown > 0) {
                        ImGui::Separator();
                    }
                }

                if (shown == 0) {
                    ImGui::TextDisabled("No friends yet.");
                }

                ImGui::EndChild();

                // "Set Note" modal popup
                if (ImGui::BeginPopup("##SetFriendNote")) {
                    const std::string& noteName = (noteEditContactIdx >= 0 &&
                        noteEditContactIdx < static_cast<int>(contacts.size()))
                        ? contacts[noteEditContactIdx].name : "";
                    ImGui::TextDisabled("Note for %s:", noteName.c_str());
                    ImGui::SetNextItemWidth(180.0f);
                    bool confirm = ImGui::InputText("##noteinput", noteEditBuf, sizeof(noteEditBuf),
                        ImGuiInputTextFlags_EnterReturnsTrue);
                    ImGui::SameLine();
                    if (confirm || ImGui::Button("OK")) {
                        if (!noteName.empty())
                            gameHandler.setFriendNote(noteName, noteEditBuf);
                        noteEditContactIdx = -1;
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Cancel")) {
                        noteEditContactIdx = -1;
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::EndPopup();
                }

                ImGui::Separator();

                // Add friend
                static char addFriendBuf[64] = {};
                ImGui::SetNextItemWidth(140.0f);
                ImGui::InputText("##sf_addfriend", addFriendBuf, sizeof(addFriendBuf));
                ImGui::SameLine();
                if (ImGui::Button("+##addfriend") && addFriendBuf[0] != '\0') {
                    gameHandler.addFriend(addFriendBuf);
                    addFriendBuf[0] = '\0';
                }

                ImGui::EndTabItem();
            }

            // ---- Ignore tab ----
            if (ImGui::BeginTabItem("Ignore")) {
                const auto& ignores = gameHandler.getIgnoreCache();
                ImGui::BeginChild("##IgnoreList", ImVec2(200, 200), false);

                if (ignores.empty()) {
                    ImGui::TextDisabled("Ignore list is empty.");
                } else {
                    for (const auto& kv : ignores) {
                        ImGui::PushID(kv.first.c_str());
                        ImGui::TextUnformatted(kv.first.c_str());
                        if (ImGui::BeginPopupContextItem("IgnoreCtx")) {
                            ImGui::TextDisabled("%s", kv.first.c_str());
                            ImGui::Separator();
                            if (ImGui::MenuItem("Unignore"))
                                gameHandler.removeIgnore(kv.first);
                            ImGui::EndPopup();
                        }
                        ImGui::PopID();
                    }
                }

                ImGui::EndChild();
                ImGui::Separator();

                // Add ignore
                static char addIgnBuf[64] = {};
                ImGui::SetNextItemWidth(140.0f);
                ImGui::InputText("##sf_addignore", addIgnBuf, sizeof(addIgnBuf));
                ImGui::SameLine();
                if (ImGui::Button("+##addignore") && addIgnBuf[0] != '\0') {
                    gameHandler.addIgnore(addIgnBuf);
                    addIgnBuf[0] = '\0';
                }

                ImGui::EndTabItem();
            }

            // ---- Channels tab ----
            if (ImGui::BeginTabItem("Channels")) {
                const auto& channels = gameHandler.getJoinedChannels();
                ImGui::BeginChild("##ChannelList", ImVec2(200, 200), false);

                if (channels.empty()) {
                    ImGui::TextDisabled("Not in any channels.");
                } else {
                    for (size_t ci = 0; ci < channels.size(); ++ci) {
                        ImGui::PushID(static_cast<int>(ci));
                        ImGui::TextUnformatted(channels[ci].c_str());
                        if (ImGui::BeginPopupContextItem("ChanCtx")) {
                            ImGui::TextDisabled("%s", channels[ci].c_str());
                            ImGui::Separator();
                            if (ImGui::MenuItem("Leave Channel"))
                                gameHandler.leaveChannel(channels[ci]);
                            ImGui::EndPopup();
                        }
                        ImGui::PopID();
                    }
                }

                ImGui::EndChild();
                ImGui::Separator();

                // Join a channel
                static char joinChanBuf[64] = {};
                ImGui::SetNextItemWidth(140.0f);
                ImGui::InputText("##sf_joinchan", joinChanBuf, sizeof(joinChanBuf));
                ImGui::SameLine();
                if (ImGui::Button("+##joinchan") && joinChanBuf[0] != '\0') {
                    gameHandler.joinChannel(joinChanBuf);
                    joinChanBuf[0] = '\0';
                }

                ImGui::EndTabItem();
            }

            // ---- Arena tab (WotLK: shows per-team rating/record + roster) ----
            const auto& arenaStats = gameHandler.getArenaTeamStats();
            if (!arenaStats.empty()) {
                if (ImGui::BeginTabItem("Arena")) {
                    ImGui::BeginChild("##ArenaList", ImVec2(0, 0), false);

                    for (size_t ai = 0; ai < arenaStats.size(); ++ai) {
                        const auto& ts = arenaStats[ai];
                        ImGui::PushID(static_cast<int>(ai));

                        // Team header: "2v2: Team Name" or fallback "Team #id"
                        std::string teamLabel;
                        if (ts.teamType > 0)
                            teamLabel = std::to_string(ts.teamType) + "v" + std::to_string(ts.teamType) + ": ";
                        if (!ts.teamName.empty())
                            teamLabel += ts.teamName;
                        else
                            teamLabel += "Team #" + std::to_string(ts.teamId);
                        ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.2f, 1.0f), "%s", teamLabel.c_str());

                        ImGui::Indent(8.0f);
                        // Rating and rank
                        ImGui::Text("Rating: %u", ts.rating);
                        if (ts.rank > 0) {
                            ImGui::SameLine(0, 6);
                            ImGui::TextDisabled("(Rank #%u)", ts.rank);
                        }

                        // Weekly record
                        uint32_t weekLosses = ts.weekGames > ts.weekWins
                                              ? ts.weekGames - ts.weekWins : 0;
                        ImGui::Text("Week:   %u W / %u L", ts.weekWins, weekLosses);

                        // Season record
                        uint32_t seasLosses = ts.seasonGames > ts.seasonWins
                                              ? ts.seasonGames - ts.seasonWins : 0;
                        ImGui::Text("Season: %u W / %u L", ts.seasonWins, seasLosses);

                        // Roster members (from SMSG_ARENA_TEAM_ROSTER)
                        const auto* roster = gameHandler.getArenaTeamRoster(ts.teamId);
                        if (roster && !roster->members.empty()) {
                            ImGui::Spacing();
                            ImGui::TextDisabled("-- Roster (%zu members) --",
                                                roster->members.size());
                            ImGui::SameLine();
                            if (ImGui::SmallButton("Refresh"))
                                gameHandler.requestArenaTeamRoster(ts.teamId);

                            // Column headers
                            ImGui::Columns(4, "##arenaRosterCols", false);
                            ImGui::SetColumnWidth(0, 110.0f);
                            ImGui::SetColumnWidth(1, 60.0f);
                            ImGui::SetColumnWidth(2, 60.0f);
                            ImGui::SetColumnWidth(3, 60.0f);
                            ImGui::TextDisabled("Name");      ImGui::NextColumn();
                            ImGui::TextDisabled("Rating");    ImGui::NextColumn();
                            ImGui::TextDisabled("Week");      ImGui::NextColumn();
                            ImGui::TextDisabled("Season");    ImGui::NextColumn();
                            ImGui::Separator();

                            for (const auto& m : roster->members) {
                                // Name coloured green (online) or grey (offline)
                                if (m.online)
                                    ImGui::TextColored(ImVec4(0.4f,1.0f,0.4f,1.0f),
                                                       "%s", m.name.c_str());
                                else
                                    ImGui::TextDisabled("%s", m.name.c_str());
                                ImGui::NextColumn();

                                ImGui::Text("%u", m.personalRating);
                                ImGui::NextColumn();

                                uint32_t wL = m.weekGames > m.weekWins
                                              ? m.weekGames - m.weekWins : 0;
                                ImGui::Text("%uW/%uL", m.weekWins, wL);
                                ImGui::NextColumn();

                                uint32_t sL = m.seasonGames > m.seasonWins
                                              ? m.seasonGames - m.seasonWins : 0;
                                ImGui::Text("%uW/%uL", m.seasonWins, sL);
                                ImGui::NextColumn();
                            }
                            ImGui::Columns(1);
                        } else {
                            ImGui::Spacing();
                            if (ImGui::SmallButton("Load Roster"))
                                gameHandler.requestArenaTeamRoster(ts.teamId);
                        }

                        ImGui::Unindent(8.0f);

                        if (ai + 1 < arenaStats.size())
                            ImGui::Separator();

                        ImGui::PopID();
                    }

                    ImGui::EndChild();
                    ImGui::EndTabItem();
                }
            }

            ImGui::EndTabBar();
        }
    }
    ImGui::End();
    showSocialFrame_ = open;

    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
}

namespace {
// LFG role bitmask: 0x02=Tank, 0x04=Healer, 0x08=DPS (0x01=Leader, unused here).
constexpr uint8_t kLfgRoleTank   = 0x02;
constexpr uint8_t kLfgRoleHealer = 0x04;
constexpr uint8_t kLfgRoleDps    = 0x08;

// Which roles a class can actually fill in the Dungeon Finder (WotLK). Every
// class can DPS. An unknown class (0, not yet received) allows all roles so we
// never wrongly lock the player out before their character data arrives.
uint8_t allowedLfgRolesForClass(uint8_t classId) {
    switch (classId) {
        case 1:  return kLfgRoleTank | kLfgRoleDps;                    // Warrior
        case 2:  return kLfgRoleTank | kLfgRoleHealer | kLfgRoleDps;   // Paladin
        case 3:  return kLfgRoleDps;                                   // Hunter
        case 4:  return kLfgRoleDps;                                   // Rogue
        case 5:  return kLfgRoleHealer | kLfgRoleDps;                  // Priest
        case 6:  return kLfgRoleTank | kLfgRoleDps;                    // Death Knight
        case 7:  return kLfgRoleHealer | kLfgRoleDps;                  // Shaman
        case 8:  return kLfgRoleDps;                                   // Mage
        case 9:  return kLfgRoleDps;                                   // Warlock
        case 11: return kLfgRoleTank | kLfgRoleHealer | kLfgRoleDps;   // Druid
        default: return kLfgRoleTank | kLfgRoleHealer | kLfgRoleDps;   // unknown
    }
}
} // namespace

void SocialPanel::renderDungeonFinderWindow(game::GameHandler& gameHandler,
                                               ChatPanel& chatPanel) {
    // Toggle Dungeon Finder (customizable keybind)
    if (!chatPanel.isChatInputActive() && !ImGui::GetIO().WantTextInput &&
        KeybindingManager::getInstance().isActionPressed(KeybindingManager::Action::TOGGLE_DUNGEON_FINDER)) {
        showDungeonFinder_ = !showDungeonFinder_;
    }

    if (!showDungeonFinder_) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth())  : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) :  720.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW * 0.5f - 175.0f, screenH * 0.2f),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(350, 0), ImGuiCond_Always);

    bool open = true;
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize;
    if (!ImGui::Begin("Dungeon Finder", &open, flags)) {
        ImGui::End();
        if (!open) showDungeonFinder_ = false;
        return;
    }
    if (!open) {
        ImGui::End();
        showDungeonFinder_ = false;
        return;
    }

    using LfgState = game::GameHandler::LfgState;
    LfgState state = gameHandler.getLfgState();

    // ---- Status banner ----
    switch (state) {
        case LfgState::None:
            ImGui::TextColored(kColorGray, "Status: Not queued");
            break;
        case LfgState::RoleCheck:
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "Status: Role check in progress...");
            break;
        case LfgState::Queued: {
            int32_t avgSec  = gameHandler.getLfgAvgWaitSec();
            uint32_t qMs    = gameHandler.getLfgTimeInQueueMs();
            int      qMin   = static_cast<int>(qMs / 60000);
            int      qSec   = static_cast<int>((qMs % 60000) / 1000);
            std::string dName = gameHandler.getCurrentLfgDungeonName();
            if (!dName.empty())
                ImGui::TextColored(colors::kQueueGreen,
                                   "Status: In queue for %s (%d:%02d)", dName.c_str(), qMin, qSec);
            else
                ImGui::TextColored(colors::kQueueGreen, "Status: In queue (%d:%02d)", qMin, qSec);
            if (avgSec >= 0) {
                int aMin = avgSec / 60;
                int aSec = avgSec % 60;
                ImGui::TextColored(colors::kSilver,
                                   "Avg wait: %d:%02d", aMin, aSec);
            }
            break;
        }
        case LfgState::Proposal: {
            std::string dName = gameHandler.getCurrentLfgDungeonName();
            if (!dName.empty())
                ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.1f, 1.0f), "Status: Group found for %s!", dName.c_str());
            else
                ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.1f, 1.0f), "Status: Group found!");
            break;
        }
        case LfgState::Boot:
            ImGui::TextColored(kColorRed, "Status: Vote kick in progress");
            break;
        case LfgState::InDungeon: {
            std::string dName = gameHandler.getCurrentLfgDungeonName();
            if (!dName.empty())
                ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Status: In dungeon (%s)", dName.c_str());
            else
                ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Status: In dungeon");
            break;
        }
        case LfgState::FinishedDungeon: {
            std::string dName = gameHandler.getCurrentLfgDungeonName();
            if (!dName.empty())
                ImGui::TextColored(colors::kLightGreen, "Status: %s complete", dName.c_str());
            else
                ImGui::TextColored(colors::kLightGreen, "Status: Dungeon complete");
            break;
        }
        case LfgState::RaidBrowser:
            ImGui::TextColored(ImVec4(0.8f, 0.6f, 1.0f, 1.0f), "Status: Raid browser");
            break;
    }

    ImGui::Separator();

    // ---- Proposal accept/decline ----
    if (state == LfgState::Proposal) {
        std::string dName = gameHandler.getCurrentLfgDungeonName();
        if (!dName.empty())
            ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.3f, 1.0f),
                               "A group has been found for %s!", dName.c_str());
        else
            ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.3f, 1.0f),
                               "A group has been found for your dungeon!");
        ImGui::Spacing();
        if (ImGui::Button("Accept", ImVec2(120, 0))) {
            gameHandler.lfgAcceptProposal(gameHandler.getLfgProposalId(), true);
        }
        ImGui::SameLine();
        if (ImGui::Button("Decline", ImVec2(120, 0))) {
            gameHandler.lfgAcceptProposal(gameHandler.getLfgProposalId(), false);
        }
        ImGui::Separator();
    }

    // ---- Vote-to-kick buttons ----
    if (state == LfgState::Boot) {
        ImGui::TextColored(kColorRed, "Vote to kick in progress:");
        const std::string& bootTarget = gameHandler.getLfgBootTargetName();
        const std::string& bootReason = gameHandler.getLfgBootReason();
        if (!bootTarget.empty()) {
            ImGui::Text("Player: ");
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "%s", bootTarget.c_str());
        }
        if (!bootReason.empty()) {
            ImGui::Text("Reason: ");
            ImGui::SameLine();
            ImGui::TextWrapped("%s", bootReason.c_str());
        }
        uint32_t bootVotes   = gameHandler.getLfgBootVotes();
        uint32_t bootTotal   = gameHandler.getLfgBootTotal();
        uint32_t bootNeeded  = gameHandler.getLfgBootNeeded();
        uint32_t bootTimeLeft= gameHandler.getLfgBootTimeLeft();
        if (bootNeeded > 0) {
            ImGui::Text("Votes: %u / %u  (need %u)   %us left",
                        bootVotes, bootTotal, bootNeeded, bootTimeLeft);
        }
        ImGui::Spacing();
        if (ImGui::Button("Vote Yes (kick)", ImVec2(140, 0))) {
            gameHandler.lfgSetBootVote(true);
        }
        ImGui::SameLine();
        if (ImGui::Button("Vote No (keep)", ImVec2(140, 0))) {
            gameHandler.lfgSetBootVote(false);
        }
        ImGui::Separator();
    }

    // ---- Teleport button (in dungeon) ----
    if (state == LfgState::InDungeon) {
        if (ImGui::Button("Teleport to Dungeon", ImVec2(-1, 0))) {
            gameHandler.lfgTeleport(true);
        }
        ImGui::Separator();
    }

    // ---- Role selection (only when not queued/in dungeon) ----
    bool canConfigure = (state == LfgState::None || state == LfgState::FinishedDungeon);

    if (canConfigure) {
        // Only offer roles the player's class can perform; drop any stale bits
        // (e.g. a role picked before the class was known, or a class change).
        const uint8_t allowedRoles = allowedLfgRolesForClass(gameHandler.getPlayerClass());
        lfgRoles_ &= allowedRoles;

        ImGui::Text("Role:");
        ImGui::SameLine();
        auto roleCheckbox = [&](const char* label, uint8_t bit) {
            const bool available = (allowedRoles & bit) != 0;
            bool checked = (lfgRoles_ & bit) != 0;
            if (!available) ImGui::BeginDisabled();
            if (ImGui::Checkbox(label, &checked))
                lfgRoles_ = (lfgRoles_ & ~bit) | (checked ? bit : 0);
            if (!available) {
                ImGui::EndDisabled();
                if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                    ImGui::SetTooltip("Your class cannot fill this role.");
            }
        };
        roleCheckbox("Tank", kLfgRoleTank);
        ImGui::SameLine();
        roleCheckbox("Healer", kLfgRoleHealer);
        ImGui::SameLine();
        roleCheckbox("DPS", kLfgRoleDps);

        ImGui::Spacing();

        // ---- Dungeon selection ----
        ImGui::Text("Dungeon:");

        // Category 0=Random, 1=Classic, 2=TBC, 3=WotLK. minLvl is the lowest
        // character level allowed to queue; kept conservative (erring low) so a
        // valid queue is never blocked - the server remains the authority.
        struct DungeonEntryEx { uint32_t id; const char* name; uint8_t cat; uint8_t minLvl; };
        static const DungeonEntryEx kDungeons[] = {
            { .id = 861, .name = "Random Dungeon",               .cat = 0, .minLvl = 15 },
            { .id = 862, .name = "Random Heroic",                .cat = 0, .minLvl = 80 },
            {  .id = 36, .name = "Deadmines",                    .cat = 1, .minLvl = 15 },
            {  .id = 43, .name = "Ragefire Chasm",               .cat = 1, .minLvl = 15 },
            {  .id = 47, .name = "Razorfen Kraul",               .cat = 1, .minLvl = 24 },
            {  .id = 48, .name = "Blackfathom Deeps",            .cat = 1, .minLvl = 21 },
            {  .id = 52, .name = "Uldaman",                      .cat = 1, .minLvl = 34 },
            {  .id = 57, .name = "Dire Maul: East",              .cat = 1, .minLvl = 46 },
            {  .id = 70, .name = "Onyxia's Lair",                .cat = 1, .minLvl = 80 },
            { .id = 264, .name = "The Blood Furnace",            .cat = 2, .minLvl = 58 },
            { .id = 269, .name = "The Shattered Halls",          .cat = 2, .minLvl = 67 },
            { .id = 576, .name = "The Nexus",                    .cat = 3, .minLvl = 68 },
            { .id = 578, .name = "The Oculus",                   .cat = 3, .minLvl = 76 },
            { .id = 595, .name = "The Culling of Stratholme",    .cat = 3, .minLvl = 77 },
            { .id = 599, .name = "Halls of Stone",               .cat = 3, .minLvl = 74 },
            { .id = 600, .name = "Drak'Tharon Keep",             .cat = 3, .minLvl = 71 },
            { .id = 601, .name = "Azjol-Nerub",                  .cat = 3, .minLvl = 70 },
            { .id = 604, .name = "Gundrak",                      .cat = 3, .minLvl = 74 },
            { .id = 608, .name = "Violet Hold",                  .cat = 3, .minLvl = 72 },
            { .id = 619, .name = "Ahn'kahet: Old Kingdom",       .cat = 3, .minLvl = 71 },
            { .id = 623, .name = "Halls of Lightning",           .cat = 3, .minLvl = 77 },
            { .id = 632, .name = "The Forge of Souls",           .cat = 3, .minLvl = 79 },
            { .id = 650, .name = "Trial of the Champion",        .cat = 3, .minLvl = 79 },
            { .id = 658, .name = "Pit of Saron",                 .cat = 3, .minLvl = 79 },
            { .id = 668, .name = "Halls of Reflection",          .cat = 3, .minLvl = 79 },
        };
        static constexpr const char* kCatHeaders[] = { nullptr, "-- Classic --", "-- TBC --", "-- WotLK --" };
        constexpr int kDungeonCount = static_cast<int>(sizeof(kDungeons)/sizeof(kDungeons[0]));
        const uint32_t playerLvl = gameHandler.getPlayerLevel();

        // Format a dungeon's label with its minimum level, e.g. "Gundrak  (Lv 74+)".
        auto dungeonLabel = [](const DungeonEntryEx& d, char* buf, size_t n) {
            snprintf(buf, n, "%s  (Lv %u+)", d.name, static_cast<unsigned>(d.minLvl));
        };

        // Find current index
        int curIdx = 0;
        for (int i = 0; i < kDungeonCount; ++i) {
            if (kDungeons[i].id == lfgSelectedDungeon_) { curIdx = i; break; }
        }

        char curLabel[96];
        dungeonLabel(kDungeons[curIdx], curLabel, sizeof(curLabel));
        ImGui::SetNextItemWidth(-1);
        if (ImGui::BeginCombo("##dungeon", curLabel)) {
            uint8_t lastCat = 255;
            for (const auto& kDungeon : kDungeons) {
                if (kDungeon.cat != lastCat && kCatHeaders[kDungeon.cat]) {
                    if (lastCat != 255) ImGui::Separator();
                    ImGui::TextDisabled("%s", kCatHeaders[kDungeon.cat]);
                    lastCat = kDungeon.cat;
                } else if (kDungeon.cat != lastCat) {
                    lastCat = kDungeon.cat;
                }
                // Grey out dungeons the player is too low to queue for.
                const bool tooLow = (playerLvl != 0 && playerLvl < kDungeon.minLvl);
                char label[96];
                dungeonLabel(kDungeon, label, sizeof(label));
                bool selected = (kDungeon.id == lfgSelectedDungeon_);
                if (tooLow) {
                    ImGui::TextDisabled("%s", label);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Requires level %u.", static_cast<unsigned>(kDungeon.minLvl));
                } else {
                    if (ImGui::Selectable(label, selected))
                        lfgSelectedDungeon_ = kDungeon.id;
                    if (selected) ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        ImGui::Spacing();

        // ---- Join button ----
        const bool rolesOk = (lfgRoles_ != 0);
        const bool levelOk = (playerLvl == 0 || playerLvl >= kDungeons[curIdx].minLvl);
        const bool canJoin = rolesOk && levelOk;
        if (!canJoin) {
            ImGui::BeginDisabled();
        }
        if (ImGui::Button("Join Dungeon Finder", ImVec2(-1, 0))) {
            gameHandler.lfgJoin({lfgSelectedDungeon_}, lfgRoles_);
        }
        if (!canJoin) {
            ImGui::EndDisabled();
            if (!rolesOk)
                ImGui::TextColored(colors::kSoftRed, "Select at least one role.");
            else if (!levelOk)
                ImGui::TextColored(colors::kSoftRed, "Requires level %u for this dungeon.",
                                   static_cast<unsigned>(kDungeons[curIdx].minLvl));
        }
    }

    // ---- Leave button (when queued or role check) ----
    if (state == LfgState::Queued || state == LfgState::RoleCheck) {
        if (ImGui::Button("Leave Queue", ImVec2(-1, 0))) {
            gameHandler.lfgLeave();
        }
    }

    ImGui::End();
}

void SocialPanel::renderWhoWindow(game::GameHandler& gameHandler,
                                     ChatPanel& chatPanel) {
    if (!showWhoWindow_) return;

    const auto& results = gameHandler.getWhoResults();

    ImGui::SetNextWindowSize(ImVec2(500, 300), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(200, 180), ImGuiCond_FirstUseEver);

    char title[64];
    uint32_t onlineCount = gameHandler.getWhoOnlineCount();
    if (onlineCount > 0)
        snprintf(title, sizeof(title), "Players Online: %u###WhoWindow", onlineCount);
    else
        snprintf(title, sizeof(title), "Who###WhoWindow");

    if (!ImGui::Begin(title, &showWhoWindow_)) {
        ImGui::End();
        return;
    }

    // Search bar with Send button
    static char whoSearchBuf[64] = {};
    bool doSearch = false;
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 60.0f);
    if (ImGui::InputTextWithHint("##whosearch", "Search players...", whoSearchBuf, sizeof(whoSearchBuf),
            ImGuiInputTextFlags_EnterReturnsTrue))
        doSearch = true;
    ImGui::SameLine();
    if (ImGui::Button("Search", ImVec2(-1, 0)))
        doSearch = true;
    if (doSearch) {
        gameHandler.queryWho(std::string(whoSearchBuf));
    }
    ImGui::Separator();

    if (results.empty()) {
        ImGui::TextDisabled("No results. Type a filter above or use /who [filter].");
        ImGui::End();
        return;
    }

    // Table: Name | Guild | Level | Class | Zone
    if (ImGui::BeginTable("##WhoTable", 5,
            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
            ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchProp,
            ImVec2(0, 0))) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("Name",  ImGuiTableColumnFlags_WidthStretch, 0.22f);
        ImGui::TableSetupColumn("Guild", ImGuiTableColumnFlags_WidthStretch, 0.20f);
        ImGui::TableSetupColumn("Level", ImGuiTableColumnFlags_WidthFixed,   40.0f);
        ImGui::TableSetupColumn("Class", ImGuiTableColumnFlags_WidthStretch, 0.20f);
        ImGui::TableSetupColumn("Zone",  ImGuiTableColumnFlags_WidthStretch, 0.28f);
        ImGui::TableHeadersRow();

        for (size_t i = 0; i < results.size(); ++i) {
            const auto& e = results[i];
            ImGui::TableNextRow();
            ImGui::PushID(static_cast<int>(i));

            // Name (class-colored if class is known)
            ImGui::TableSetColumnIndex(0);
            uint8_t cid = static_cast<uint8_t>(e.classId);
            ImVec4 nameCol = classColorVec4(cid);
            ImGui::TextColored(nameCol, "%s", e.name.c_str());

            // Right-click context menu on the name
            if (ImGui::BeginPopupContextItem("##WhoCtx")) {
                ImGui::TextDisabled("%s", e.name.c_str());
                ImGui::Separator();
                if (ImGui::MenuItem("Whisper")) {
                    chatPanel.setWhisperTarget(e.name);
                }
                if (ImGui::MenuItem("Invite to Group"))
                    gameHandler.inviteToGroup(e.name);
                if (ImGui::MenuItem("Add Friend"))
                    gameHandler.addFriend(e.name);
                if (ImGui::MenuItem("Ignore"))
                    gameHandler.addIgnore(e.name);
                ImGui::EndPopup();
            }

            // Guild
            ImGui::TableSetColumnIndex(1);
            if (!e.guildName.empty())
                ImGui::TextDisabled("<%s>", e.guildName.c_str());

            // Level
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%u", e.level);

            // Class
            ImGui::TableSetColumnIndex(3);
            const char* className = game::getClassName(static_cast<game::Class>(e.classId));
            ImGui::TextColored(nameCol, "%s", className);

            // Zone
            ImGui::TableSetColumnIndex(4);
            if (e.zoneId != 0) {
                std::string zoneName = gameHandler.getWhoAreaName(e.zoneId);
                if (!zoneName.empty())
                    ImGui::TextUnformatted(zoneName.c_str());
                else {
                    char zfb[32];
                    snprintf(zfb, sizeof(zfb), "Zone #%u", e.zoneId);
                    ImGui::TextUnformatted(zfb);
                }
            }

            ImGui::PopID();
        }

        ImGui::EndTable();
    }

    ImGui::End();
}

void SocialPanel::openInspectWindow(game::GameHandler& gameHandler) {
    if (frameXmlOwns(UiElement::Inspect)) {
        // Loads Blizzard_InspectUI and shows it. The inspect request itself is
        // already on its way from the caller; this only puts a window up.
        gameHandler.runInterfaceCommand("InspectUnit(\"target\")");
    } else {
        showInspectWindow_ = true;
    }
}

void SocialPanel::renderInspectWindow(game::GameHandler& gameHandler,
                                         InventoryScreen& inventoryScreen) {
    if (!showInspectWindow_) {
        inspectWindowAutoRequestGuid_ = 0;
        return;
    }

    // Lazy-load SpellItemEnchantment.dbc for enchant name lookup
    static std::unordered_map<uint32_t, std::string> s_enchantNames;
    static bool s_enchantDbLoaded = false;
    auto* assetMgrEnchant = services_.assetManager;
    if (!s_enchantDbLoaded && assetMgrEnchant && assetMgrEnchant->isInitialized()) {
        s_enchantDbLoaded = true;
        auto dbc = assetMgrEnchant->loadDBC("SpellItemEnchantment.dbc");
        if (dbc && dbc->isLoaded()) {
            const auto* layout = pipeline::getActiveDBCLayout()
                                 ? pipeline::getActiveDBCLayout()->getLayout("SpellItemEnchantment")
                                 : nullptr;
            uint32_t idField   = layout ? (*layout)["ID"]   : 0;
            uint32_t nameField = pipeline::detectEnchantmentNameField(dbc.get(), layout);
            for (uint32_t i = 0; i < dbc->getRecordCount(); ++i) {
                uint32_t id = dbc->getUInt32(i, idField);
                if (id == 0) continue;
                std::string nm = dbc->getString(i, nameField);
                if (!nm.empty()) s_enchantNames[id] = std::move(nm);
            }
        }
    }

    // Slot index 0..18 maps to equipment slots 1..19 (WoW convention: slot 0 unused on server)
    static constexpr const char* kSlotNames[19] = {
        "Head", "Neck", "Shoulder", "Shirt", "Chest",
        "Waist", "Legs", "Feet", "Wrist", "Hands",
        "Finger 1", "Finger 2", "Trinket 1", "Trinket 2", "Back",
        "Main Hand", "Off Hand", "Ranged", "Tabard"
    };

    ImGui::SetNextWindowSize(ImVec2(360, 440), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(350, 120), ImGuiCond_FirstUseEver);

    const game::GameHandler::InspectResult* result = gameHandler.getInspectResult();
    const uint64_t targetGuid = gameHandler.getTargetGuid();
    auto target = gameHandler.getTarget();
    const bool targetIsPlayer =
        target && target->getType() == game::ObjectType::PLAYER && targetGuid != 0;
    if (targetIsPlayer &&
        inspectWindowAutoRequestGuid_ != targetGuid &&
        (!result || result->guid != targetGuid)) {
        inspectWindowAutoRequestGuid_ = targetGuid;
        gameHandler.inspectTarget();
        result = gameHandler.getInspectResult();
    }

    std::string title = result ? ("Inspect: " + result->playerName + "###InspectWin")
                                : "Inspect###InspectWin";
    if (!ImGui::Begin(title.c_str(), &showInspectWindow_, ImGuiWindowFlags_NoCollapse)) {
        ImGui::End();
        return;
    }

    if (!result) {
        ImGui::TextDisabled("No inspect data yet. Target a player and use Inspect.");
        ImGui::End();
        return;
    }

    // Player name - class-colored if entity is loaded, else gold
    {
        auto ent = gameHandler.getEntityManager().getEntity(result->guid);
        uint8_t cid = entityClassId(ent.get());
        ImVec4 nameColor = (cid != 0) ? classColorVec4(cid) : ui::colors::kTooltipGold;
        ImGui::PushStyleColor(ImGuiCol_Text, nameColor);
        ImGui::Text("%s", result->playerName.c_str());
        ImGui::PopStyleColor();
        if (cid != 0) {
            ImGui::SameLine();
            ImGui::TextColored(classColorVec4(cid), "(%s)", classNameStr(cid));
        }
    }

    ImGui::Separator();

    // Equipment list
    bool hasAnyGear = false;
    for (int s = 0; s < 19; ++s) {
        if (result->itemEntries[s] != 0) { hasAnyGear = true; break; }
    }
    const auto* visibleEquipment = game::isActiveExpansion("tbc")
        ? gameHandler.getOtherPlayerVisibleEquipment(result->guid)
        : nullptr;
    bool hasVisibleEquipment = false;
    if (visibleEquipment) {
        for (uint32_t displayId : *visibleEquipment) {
            if (displayId != 0) {
                hasVisibleEquipment = true;
                break;
            }
        }
    }

    struct InspectGearSlot {
        uint32_t entry = 0;
        uint32_t displayId = 0;
        uint16_t enchantId = 0;
        const game::ItemQueryResponseData* info = nullptr;
        bool loading = false;
    };

    const bool usingVisibleFallback = !hasAnyGear && hasVisibleEquipment;
    std::array<InspectGearSlot, 19> gearSlots{};

    for (int s = 0; s < 19; ++s) {
        if (hasAnyGear) {
            const uint32_t entry = result->itemEntries[s];
            if (entry == 0) continue;
            gearSlots[s].entry = entry;
            gearSlots[s].enchantId = result->enchantIds[s];
            gearSlots[s].info = gameHandler.getItemInfo(entry);
            if (gearSlots[s].info) {
                gearSlots[s].displayId = gearSlots[s].info->displayInfoId;
            } else {
                gearSlots[s].loading = true;
                gameHandler.ensureItemInfo(entry);
            }
        } else if (usingVisibleFallback) {
            const uint32_t entry = (*visibleEquipment)[s];
            if (entry == 0) continue;
            gearSlots[s].entry = entry;
            gearSlots[s].info = gameHandler.getItemInfo(entry);
            if (gearSlots[s].info) {
                gearSlots[s].displayId = gearSlots[s].info->displayInfoId;
            } else {
                gearSlots[s].loading = true;
                gameHandler.ensureItemInfo(entry);
            }
        }
    }

    auto renderInspectSlot = [&](int slotIndex, float size) {
        const auto& slot = gearSlots[slotIndex];
        const bool empty = slot.entry == 0 && slot.displayId == 0;
        const char* label = kSlotNames[slotIndex];
        ImDrawList* drawList = ImGui::GetWindowDrawList();
        ImVec2 pos = ImGui::GetCursorScreenPos();

        ImVec4 qColor = slot.info
            ? InventoryScreen::getQualityColor(static_cast<game::ItemQuality>(slot.info->quality))
            : ImVec4(0.45f, 0.48f, 0.56f, 1.0f);
        ImU32 borderCol = empty
            ? IM_COL32(70, 70, 80, 190)
            : ImGui::ColorConvertFloat4ToU32(qColor);
        ImU32 bgCol = empty ? IM_COL32(25, 25, 32, 190) : IM_COL32(40, 35, 30, 220);

        VkDescriptorSet iconTex = (!empty && slot.displayId != 0)
            ? inventoryScreen.getItemIcon(slot.displayId)
            : VK_NULL_HANDLE;
        if (iconTex) {
            drawList->AddImage((ImTextureID)(uintptr_t)iconTex, pos,
                               ImVec2(pos.x + size, pos.y + size));
            drawList->AddRect(pos, ImVec2(pos.x + size, pos.y + size),
                              borderCol, 0.0f, 0, 2.0f);
        } else {
            drawList->AddRectFilled(pos, ImVec2(pos.x + size, pos.y + size), bgCol);
            drawList->AddRect(pos, ImVec2(pos.x + size, pos.y + size),
                              borderCol, 0.0f, 0, empty ? 1.0f : 2.0f);

            char abbr[4] = {};
            if (slot.loading) {
                abbr[0] = '.';
                abbr[1] = '.';
            } else if (slot.info && !slot.info->name.empty()) {
                abbr[0] = slot.info->name[0];
                if (slot.info->name.size() > 1) abbr[1] = slot.info->name[1];
            } else {
                abbr[0] = label[0];
                if (label[1]) abbr[1] = label[1];
            }
            float textW = ImGui::CalcTextSize(abbr).x;
            drawList->AddText(ImVec2(pos.x + (size - textW) * 0.5f, pos.y + size * 0.3f),
                              empty ? IM_COL32(85, 85, 95, 180) : borderCol, abbr);
        }

        if (slot.enchantId != 0) {
            drawList->AddText(ImVec2(pos.x + size - 10.0f, pos.y + 1.0f),
                              IM_COL32(150, 220, 255, 240), "*");
        }

        ImGui::InvisibleButton("slot", ImVec2(size, size));
        if (ImGui::IsItemHovered()) {
            if (slot.info && slot.info->valid) {
                inventoryScreen.renderItemTooltip(*slot.info);
            } else {
                ImGui::BeginTooltip();
                ImGui::TextDisabled("%s", label);
                if (slot.entry != 0) {
                    ImGui::Text("Item #%u", static_cast<unsigned>(slot.entry));
                    ImGui::TextDisabled("Loading item details...");
                } else if (slot.displayId != 0) {
                    ImGui::Text("Display ID %u", static_cast<unsigned>(slot.displayId));
                } else {
                    ImGui::TextDisabled("Empty");
                }
                ImGui::EndTooltip();
            }
        }
    };

    auto renderInspectPaperDoll = [&]() {
        static constexpr int leftSlots[] = {0, 1, 2, 14, 4, 3, 18, 8};
        static constexpr int rightSlots[] = {9, 5, 6, 7, 10, 11, 12, 13};
        static constexpr int weaponSlots[] = {15, 16, 17};
        constexpr float slotSize = 36.0f;
        constexpr float previewW = 140.0f;

        ImGui::TextColored(ui::colors::kWarmGold, "Equipment");
        if (usingVisibleFallback) {
            ImGui::SameLine();
            ImGui::TextDisabled("(visible)");
            if (ImGui::IsItemHovered()) {
                ImGui::BeginTooltip();
                ImGui::TextDisabled("Showing public visible equipment fields.");
                ImGui::EndTooltip();
            }
        }
        ImGui::Separator();

        float contentStartX = ImGui::GetCursorPosX();
        float rightColX = contentStartX + slotSize + 8.0f + previewW + 8.0f;
        float previewStartY = ImGui::GetCursorScreenPos().y;

        for (int r = 0; r < 8; ++r) {
            ImGui::PushID(leftSlots[r]);
            renderInspectSlot(leftSlots[r], slotSize);
            ImGui::PopID();

            ImGui::SameLine(rightColX);
            ImGui::PushID(rightSlots[r]);
            renderInspectSlot(rightSlots[r], slotSize);
            ImGui::PopID();
        }

        float previewEndY = ImGui::GetCursorScreenPos().y;
        float previewX = ImGui::GetWindowPos().x + contentStartX + slotSize + 8.0f;
        float previewH = previewEndY - previewStartY;
        ImVec2 pMin(previewX, previewStartY);
        ImVec2 pMax(previewX + previewW, previewStartY + previewH);
        ImDrawList* drawList = ImGui::GetWindowDrawList();
        drawList->AddRectFilled(pMin, pMax, IM_COL32(13, 13, 25, 210));
        drawList->AddRect(pMin, pMax, IM_COL32(60, 60, 80, 200));

        std::string centerName = result->playerName;
        ImVec2 nameSize = ImGui::CalcTextSize(centerName.c_str());
        drawList->AddText(ImVec2(pMin.x + (previewW - nameSize.x) * 0.5f, pMin.y + 14.0f),
                          IM_COL32(220, 220, 235, 230), centerName.c_str());
        auto ent = gameHandler.getEntityManager().getEntity(result->guid);
        uint8_t cid = entityClassId(ent.get());
        if (cid != 0) {
            const char* cls = classNameStr(cid);
            ImVec2 classSize = ImGui::CalcTextSize(cls);
            drawList->AddText(ImVec2(pMin.x + (previewW - classSize.x) * 0.5f, pMin.y + 34.0f),
                              ImGui::ColorConvertFloat4ToU32(classColorVec4(cid)), cls);
        }
        const char* sourceText = usingVisibleFallback ? "Visible gear" : "Inspect gear";
        ImVec2 srcSize = ImGui::CalcTextSize(sourceText);
        drawList->AddText(ImVec2(pMin.x + (previewW - srcSize.x) * 0.5f, pMax.y - 24.0f),
                          IM_COL32(150, 150, 165, 210), sourceText);

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::SetCursorPosX(contentStartX + slotSize + 8.0f);
        for (int i = 0; i < 3; ++i) {
            if (i > 0) ImGui::SameLine();
            ImGui::PushID(weaponSlots[i]);
            renderInspectSlot(weaponSlots[i], slotSize);
            ImGui::PopID();
        }
    };

    if (!hasAnyGear && !hasVisibleEquipment) {
        ImGui::TextDisabled("Equipment data not yet available.");
        ImGui::TextDisabled("(Gear loads after the player is inspected in-range)");
    } else {
        // Average item level (only slots that have loaded info and are not shirt/tabard)
        // Shirt=slot3, Tabard=slot18 - excluded from gear score by WoW convention
        uint32_t iLevelSum = 0;
        int iLevelCount = 0;
        for (int s = 0; s < 19; ++s) {
            if (s == 3 || s == 18) continue; // shirt, tabard
            uint32_t entry = result->itemEntries[s];
            if (entry == 0) continue;
            const game::ItemQueryResponseData* info = gameHandler.getItemInfo(entry);
            if (info && info->valid && info->itemLevel > 0) {
                iLevelSum += info->itemLevel;
                ++iLevelCount;
            }
        }
        if (iLevelCount > 0) {
            float avgIlvl = static_cast<float>(iLevelSum) / static_cast<float>(iLevelCount);
            ImGui::TextColored(ImVec4(0.8f, 0.9f, 1.0f, 1.0f), "Avg iLvl: %.1f", avgIlvl);
            ImGui::SameLine();
            ImGui::TextDisabled("(%d/%d slots loaded)", iLevelCount,
                [&]{ int c=0; for(int s=0;s<19;++s){if(s==3||s==18)continue;if(result->itemEntries[s])++c;} return c; }());
        }
        renderInspectPaperDoll();
    }

    // Arena teams (WotLK - from MSG_INSPECT_ARENA_TEAMS)
    if (!result->arenaTeams.empty()) {
        ImGui::Separator();
        ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.2f, 1.0f), "Arena Teams");
        ImGui::Spacing();
        for (const auto& team : result->arenaTeams) {
            const char* bracket = (team.type == 2) ? "2v2"
                                : (team.type == 3) ? "3v3"
                                : (team.type == 5) ? "5v5" : "?v?";
            ImGui::TextColored(ImVec4(0.9f, 0.9f, 0.9f, 1.0f),
                               "[%s]  %s", bracket, team.name.c_str());
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.4f, 0.85f, 1.0f, 1.0f),
                               "  Rating: %u", team.personalRating);
            if (team.weekGames > 0 || team.seasonGames > 0) {
                ImGui::TextDisabled("    Week: %u/%u  Season: %u/%u",
                                    team.weekWins, team.weekGames,
                                    team.seasonWins, team.seasonGames);
            }
        }
    }

    ImGui::End();
}

} // namespace ui
} // namespace wowee
