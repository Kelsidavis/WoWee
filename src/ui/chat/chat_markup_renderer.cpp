// ChatMarkupRenderer - render parsed ChatSegments via ImGui.
// Moved from ChatPanel::render() inline lambdas (Phase 2.2).
// Item tooltip rendering extracted to ItemTooltipRenderer (Phase 6.7).
#include "ui/chat/chat_markup_renderer.hpp"
#include "addons/lua_api_helpers.hpp"
#include "ui/chat/item_tooltip_renderer.hpp"
#include "ui/ui_colors.hpp"
#include "ui/inventory_screen.hpp"
#include "ui/spellbook_screen.hpp"
#include "game/game_handler.hpp"
#include "pipeline/asset_manager.hpp"
#include <imgui.h>
#include <cstring>

#if defined(_WIN32)
// Guarded, because the build already defines it: CMakeLists adds
// WIN32_LEAN_AND_MEAN to add_compile_definitions, and redefining it is an
// error under -Werror. The five other places that reach for windows.h all
// test first; this was the one that did not.
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <windows.h>
#  include <shellapi.h>
#elif defined(__APPLE__)
#  include <spawn.h>
#  include <sys/wait.h>
extern char** environ;
#else
#  include <spawn.h>
#  include <sys/wait.h>
extern char** environ;
#endif

namespace wowee { namespace ui {

namespace {

// Open a URL in the user's default browser without invoking a shell -
// chat messages are remote-attacker-controlled, so the previous
// `system("xdg-open '" + url + "' &")` was a command-injection sink
// (a URL containing `'; rm -rf ~; '` would execute arbitrary commands).
//
// Refuses anything that isn't a plain http(s):// URL with safe ASCII -
// no shell metacharacters, no control bytes, no embedded NULs.
bool openExternalUrl(const std::string& url) {
    if (url.empty() || url.size() > 2048) return false;
    // Whitelist scheme to defang file:// / javascript: / data: etc.
    bool isHttp  = url.compare(0, 7, "http://") == 0;
    bool isHttps = url.compare(0, 8, "https://") == 0;
    if (!isHttp && !isHttps) return false;
    // Reject control bytes and the obvious shell-meta set so even a
    // future shell-using opener can't be tricked.
    for (unsigned char c : url) {
        if (c < 0x20 || c == 0x7F) return false;
        if (c == '\'' || c == '"' || c == '`' || c == '\\' ||
            c == '$'  || c == ';'  || c == '|' || c == '&'  ||
            c == '<'  || c == '>'  || c == '\n' || c == '\r') {
            return false;
        }
    }

#if defined(_WIN32)
    HINSTANCE rc = ShellExecuteA(nullptr, "open", url.c_str(),
                                 nullptr, nullptr, SW_SHOWNORMAL);
    return reinterpret_cast<INT_PTR>(rc) > 32;
#else
#  if defined(__APPLE__)
    const char* opener = "/usr/bin/open";
#  else
    const char* opener = "/usr/bin/xdg-open";
#  endif
    char* argv[] = {
        const_cast<char*>(opener),
        const_cast<char*>(url.c_str()),
        nullptr
    };
    pid_t pid = 0;
    int rc = posix_spawn(&pid, opener, nullptr, nullptr, argv, environ);
    if (rc != 0) return false;
    // Don't block the UI thread on the browser launch; reap async.
    int status = 0;
    waitpid(pid, &status, WNOHANG);
    return true;
#endif
}

// Shift-click a link to put it back into the chat input. Was written out four
// times, once per link type; a surface that has no raw input buffer of its own
// -- the guild info text, for one -- supplies insertLink instead.
void shiftClickInsert(const ChatSegment& seg, const MarkupRenderContext& ctx) {
    if (!ImGui::IsItemClicked() || !ImGui::GetIO().KeyShift) return;
    if (ctx.chatInputBuffer && ctx.moveCursorToEnd) {
        size_t curLen = strlen(ctx.chatInputBuffer);
        if (curLen + seg.rawLink.size() + 1 < ctx.chatInputBufSize) {
            strncat(ctx.chatInputBuffer, seg.rawLink.c_str(),
                    ctx.chatInputBufSize - curLen - 1);
            *ctx.moveCursorToEnd = true;
        }
    } else if (ctx.insertLink) {
        ctx.insertLink(seg.rawLink);
    }
}

} // namespace

// ---- Main segment renderer ----

void ChatMarkupRenderer::render(
    const std::vector<ChatSegment>& segments,
    const ImVec4& baseColor,
    const MarkupRenderContext& ctx) const
{
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        bool needSameLine = (i + 1 < segments.size());

        switch (seg.type) {
        case SegmentType::Text: {
            if (!seg.text.empty()) {
                ImGui::PushStyleColor(ImGuiCol_Text, baseColor);
                ImGui::TextWrapped("%s", seg.text.c_str());
                ImGui::PopStyleColor();
                if (needSameLine) ImGui::SameLine(0, 0);
            }
            break;
        }
        case SegmentType::ColoredText: {
            if (!seg.text.empty()) {
                ImGui::PushStyleColor(ImGuiCol_Text, seg.color);
                ImGui::TextWrapped("%s", seg.text.c_str());
                ImGui::PopStyleColor();
                if (needSameLine) ImGui::SameLine(0, 0);
            }
            break;
        }
        case SegmentType::ItemLink: {
            uint32_t itemEntry = seg.id;
            if (itemEntry > 0 && ctx.gameHandler) {
                ctx.gameHandler->ensureItemInfo(itemEntry);
            }
            // Show small icon before item link if available
            if (itemEntry > 0 && ctx.gameHandler && ctx.inventory) {
                const auto* chatInfo = ctx.gameHandler->getItemInfo(itemEntry);
                if (chatInfo && chatInfo->valid && chatInfo->displayInfoId != 0) {
                    VkDescriptorSet chatIcon = ctx.inventory->getItemIcon(chatInfo->displayInfoId);
                    if (chatIcon) {
                        ImGui::Image((ImTextureID)(uintptr_t)chatIcon, ImVec2(12, 12));
                        if (ImGui::IsItemHovered()) {
                            ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
                            ItemTooltipRenderer::render(itemEntry, *ctx.gameHandler, *ctx.inventory, ctx.assetMgr);
                        }
                        ImGui::SameLine(0, 2);
                    }
                }
            }
            // Let ImGui format the brackets inline - was heap-allocating a
            // bracketed copy on every chat-line draw, per visible link.
            ImGui::PushStyleColor(ImGuiCol_Text, seg.color);
            ImGui::TextWrapped("[%s]", seg.text.c_str());
            ImGui::PopStyleColor();
            if (ImGui::IsItemHovered()) {
                ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
                if (itemEntry > 0 && ctx.gameHandler && ctx.inventory) {
                    ItemTooltipRenderer::render(itemEntry, *ctx.gameHandler, *ctx.inventory, ctx.assetMgr);
                }
            }
            // Shift-click: insert entire link back into chat input
            shiftClickInsert(seg, ctx);
            if (needSameLine) ImGui::SameLine(0, 0);
            break;
        }
        case SegmentType::SpellLink: {
            // Small icon (use spell icon cache if available)
            VkDescriptorSet spellIcon = VK_NULL_HANDLE;
            if (seg.id > 0 && ctx.getSpellIcon && ctx.assetMgr) {
                spellIcon = ctx.getSpellIcon(seg.id, ctx.assetMgr);
            }
            if (spellIcon) {
                ImGui::Image((ImTextureID)(uintptr_t)spellIcon, ImVec2(12, 12));
                if (ImGui::IsItemHovered() && ctx.spellbook && ctx.gameHandler && ctx.assetMgr) {
                    ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
                    ctx.spellbook->renderSpellInfoTooltip(seg.id, *ctx.gameHandler, ctx.assetMgr);
                }
                ImGui::SameLine(0, 2);
            }
            // Let ImGui format the brackets inline - was heap-allocating a
            // bracketed copy on every chat-line draw, per visible link.
            ImGui::PushStyleColor(ImGuiCol_Text, seg.color);
            ImGui::TextWrapped("[%s]", seg.text.c_str());
            ImGui::PopStyleColor();
            if (ImGui::IsItemHovered() && ctx.spellbook && ctx.gameHandler && ctx.assetMgr) {
                ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
                ctx.spellbook->renderSpellInfoTooltip(seg.id, *ctx.gameHandler, ctx.assetMgr);
            }
            // Shift-click: insert link
            shiftClickInsert(seg, ctx);
            if (needSameLine) ImGui::SameLine(0, 0);
            break;
        }
        case SegmentType::QuestLink: {
            ImGui::PushStyleColor(ImGuiCol_Text, colors::kWarmGold);
            ImGui::TextWrapped("[%s]", seg.text.c_str());
            ImGui::PopStyleColor();
            if (ImGui::IsItemHovered()) {
                ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
                ImGui::BeginTooltip();
                ImGui::TextColored(colors::kWarmGold, "%s", seg.text.c_str());
                if (seg.extra > 0) ImGui::TextDisabled("Level %u Quest", seg.extra);
                ImGui::TextDisabled("Click quest log to view details");
                ImGui::EndTooltip();
            }
            // Plain click opens quest log; shift-click inserts the link
            // into chat input. Without the !KeyShift guard a shift-click
            // would do BOTH (open the log and insert), losing the user's
            // intent.
            if (ImGui::IsItemClicked() && !ImGui::GetIO().KeyShift &&
                seg.id > 0 && ctx.gameHandler) {
                addons::openInterfaceQuestLog(*ctx.gameHandler, seg.id);
            }
            // Shift-click: insert link
            shiftClickInsert(seg, ctx);
            if (needSameLine) ImGui::SameLine(0, 0);
            break;
        }
        case SegmentType::AchievementLink: {
            ImGui::PushStyleColor(ImGuiCol_Text, colors::kBrightGold);
            ImGui::TextWrapped("[%s]", seg.text.c_str());
            ImGui::PopStyleColor();
            if (ImGui::IsItemHovered()) {
                ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
                ImGui::SetTooltip("Achievement: %s", seg.text.c_str());
            }
            // Shift-click: insert link
            shiftClickInsert(seg, ctx);
            if (needSameLine) ImGui::SameLine(0, 0);
            break;
        }
        case SegmentType::Url: {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f, 0.7f, 1.0f, 1.0f));
            ImGui::TextWrapped("%s", seg.text.c_str());
            if (ImGui::IsItemHovered()) {
                ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
                ImGui::SetTooltip("Open: %s", seg.text.c_str());
            }
            if (ImGui::IsItemClicked()) {
                openExternalUrl(seg.text);
            }
            ImGui::PopStyleColor();
            if (needSameLine) ImGui::SameLine(0, 0);
            break;
        }
        } // switch
    }
}

} // namespace ui
} // namespace wowee
