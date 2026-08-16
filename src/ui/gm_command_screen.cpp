// ============================================================
// GM command screen - browse/search the GM command reference and
// send commands to the server (part of WindowManager). Opened from
// the micro-menu "GM" button.
//
// Each command's syntax string is parsed into labeled form fields so a
// value like "set level 60 on player Bob" is filled in, not typed as a
// raw string. The assembled command is sent to the server as a SAY chat
// message with a "." prefix (AzerothCore convention); the server does
// the real work and enforces the player's actual permission level.
// ============================================================
#include "ui/window_manager.hpp"
#include "ui/chat/gm_command_data.hpp"
#include "ui/bis_gear_data.hpp"
#include "game/game_handler.hpp"
#include "game/game_utils.hpp" // isActiveExpansion
#include "game/world_packets.hpp" // game::ChatType
#include <imgui.h>
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

namespace wowee {
namespace ui {

namespace {

const char* securityLabel(uint8_t s) {
    switch (s) {
        case 0:  return "Player";
        case 1:  return "Moderator";
        case 2:  return "Game Master";
        case 3:  return "Administrator";
        default: return "Console";
    }
}

ImVec4 securityColor(uint8_t s) {
    switch (s) {
        case 0:  return ImVec4(0.62f, 0.62f, 0.62f, 1.0f); // gray
        case 1:  return ImVec4(0.35f, 0.80f, 0.35f, 1.0f); // green
        case 2:  return ImVec4(0.35f, 0.62f, 1.00f, 1.0f); // blue
        case 3:  return ImVec4(0.78f, 0.50f, 1.00f, 1.0f); // purple
        default: return ImVec4(1.00f, 0.55f, 0.20f, 1.0f); // orange
    }
}

std::string toLower(std::string_view s) {
    std::string out(s);
    for (char& c : out) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return out;
}

// First whitespace-delimited token of a command name, used to group the list
// (e.g. "gm on"/"gm off" → "gm", "go xyz"/"go creature" → "go").
std::string_view firstToken(std::string_view name) {
    size_t sp = name.find(' ');
    return sp == std::string_view::npos ? name : name.substr(0, sp);
}

// A single argument parsed out of a command's syntax string.
enum class GmArgKind { Number, Text, Choice, Flag };
struct GmArg {
    std::string label;                 // "id", "player", ...
    GmArgKind   kind = GmArgKind::Text;
    bool        optional = false;      // was wrapped in [ ]
    std::vector<std::string> choices;  // for Choice: {"on","off"}; Flag: {word}
};

// Parse the argument portion of a command's syntax into form fields.
//   ".additem #id [#count]"        → Number "id"; Number "count" (optional)
//   ".tele name $player #location" → Text "player"; Number "location"
//   ".modify gender male/female"   → Choice {male,female}
//   ".learn #spell [all]"          → Number "spell"; Flag "all" (optional)
std::vector<GmArg> parseArgs(std::string_view name, std::string_view syntax) {
    std::vector<GmArg> args;
    std::string base = "." + std::string(name);
    std::string syn(syntax);
    // Strip the literal command prefix (".additem", ".tele name", ...) so only
    // the argument tokens remain; fall back to dropping the first token.
    std::string rest;
    if (syn.size() >= base.size() && syn.compare(0, base.size(), base) == 0) {
        rest = syn.substr(base.size());
    } else {
        size_t sp = syn.find(' ');
        rest = (sp == std::string::npos) ? std::string() : syn.substr(sp);
    }

    std::string token;
    auto flush = [&]() {
        if (token.empty()) return;
        bool optional = token.find('[') != std::string::npos;
        // Strip brackets/braces/angle wrappers.
        std::string clean;
        for (char c : token)
            if (c != '[' && c != ']' && c != '<' && c != '>' && c != '{' && c != '}')
                clean += c;
        token.clear();
        if (clean.empty() || args.size() >= 8) return;

        GmArg a;
        a.optional = optional;
        if (clean[0] == '#') {
            a.kind = GmArgKind::Number;
            a.label = clean.substr(1);
        } else if (clean[0] == '$') {
            a.kind = GmArgKind::Text;
            a.label = clean.substr(1);
        } else if (clean.find('/') != std::string::npos) {
            a.kind = GmArgKind::Choice;
            a.label = clean;
            std::string opt;
            for (char c : clean + "/") {
                if (c == '/') { if (!opt.empty()) a.choices.push_back(opt); opt.clear(); }
                else opt += c;
            }
        } else {
            // A bare word is an optional literal flag (e.g. "all", "triggered").
            a.kind = GmArgKind::Flag;
            a.label = clean;
            a.choices.push_back(clean);
        }
        args.push_back(std::move(a));
    };
    for (char c : rest) {
        if (std::isspace(static_cast<unsigned char>(c))) flush();
        else token += c;
    }
    flush();
    return args;
}

// Active expansion id as a short string for gear-table lookup.
const char* activeExpansionId() {
    if (game::isActiveExpansion("wotlk")) return "wotlk";
    if (game::isActiveExpansion("tbc"))   return "tbc";
    return "classic"; // classic / turtle
}

} // namespace

void WindowManager::queueMaxOutCharacter(game::GameHandler& gameHandler) {
    const uint8_t classId = gameHandler.getPlayerClass();
    const char* exp = activeExpansionId();
    const int maxLevel = game::isActiveExpansion("wotlk") ? 80 : game::isActiveExpansion("tbc") ? 70 : 60;

    gmPendingCmds_.clear();
    gmPendingPos_ = 0;

    // Order matters: level first (some spells/talents require it), then learn,
    // then skills, then gold, then gear.
    if (gmMaxLevel_)  gmPendingCmds_.push_back(".character level " + std::to_string(maxLevel));
    if (gmMaxSpells_) {
        gmPendingCmds_.emplace_back(".learn all my class");
        gmPendingCmds_.emplace_back(".learn all my spells");
    }
    if (gmMaxTalents_) gmPendingCmds_.emplace_back(".learn all my talents");
    if (gmMaxSkills_)  gmPendingCmds_.emplace_back(".maxskill");
    if (gmMaxGold_)    gmPendingCmds_.emplace_back(".modify money 10000000"); // 1000g
    if (gmMaxGear_) {
        for (uint32_t id : getMaxOutGear(exp, classId))
            gmPendingCmds_.push_back(".additem " + std::to_string(id));
    }

    gameHandler.addSystemChatMessage(
        "Max Out: queued " + std::to_string(gmPendingCmds_.size()) +
        " commands (" + std::string(exp) + ", class " + std::to_string(classId) + ").");
}

void WindowManager::renderGmCommandScreen(game::GameHandler& gameHandler) {
    if (!showGmCommandScreen_) return;

    // Drain queued quick-action commands one per frame so a burst of
    // .additem/.learn messages doesn't trip server chat-flood protection.
    if (gmPendingPos_ < gmPendingCmds_.size()) {
        gameHandler.sendChatMessage(game::ChatType::SAY, gmPendingCmds_[gmPendingPos_], "");
        ++gmPendingPos_;
        if (gmPendingPos_ >= gmPendingCmds_.size()) {
            gmPendingCmds_.clear();
            gmPendingPos_ = 0;
        }
    }

    ImGui::SetNextWindowSize(ImVec2(680, 500), ImGuiCond_FirstUseEver);
    bool open = true;
    if (!ImGui::Begin("GM Commands", &open)) {
        ImGui::End();
        if (!open) showGmCommandScreen_ = false;
        return;
    }
    if (!open) showGmCommandScreen_ = false;

    // ---- Quick action: Max Out Character ----
    if (ImGui::CollapsingHeader("Quick: Max Out Character", ImGuiTreeNodeFlags_DefaultOpen)) {
        const bool busy = gmPendingPos_ < gmPendingCmds_.size();
        ImGui::TextDisabled("Applies to your character for the active expansion (%s).",
                            activeExpansionId());
        ImGui::Checkbox("Max level", &gmMaxLevel_);  ImGui::SameLine();
        ImGui::Checkbox("Spells", &gmMaxSpells_);     ImGui::SameLine();
        ImGui::Checkbox("Talents", &gmMaxTalents_);   ImGui::SameLine();
        ImGui::Checkbox("Skills", &gmMaxSkills_);     ImGui::SameLine();
        ImGui::Checkbox("BiS gear", &gmMaxGear_);     ImGui::SameLine();
        ImGui::Checkbox("1000g", &gmMaxGold_);

        if (busy) ImGui::BeginDisabled();
        if (ImGui::Button(busy ? "Sending..." : "Max Out Character", ImVec2(160.0f, 0.0f)))
            queueMaxOutCharacter(gameHandler);
        if (busy) ImGui::EndDisabled();
        ImGui::SameLine();
        if (busy) {
            ImGui::Text("%zu/%zu", gmPendingPos_, gmPendingCmds_.size());
        } else {
            ImGui::TextDisabled("Sends GM commands (needs GM permissions server-side).");
        }
    }
    ImGui::Separator();

    // ---- Toolbar: search + max-level filter ----
    ImGui::SetNextItemWidth(230.0f);
    ImGui::InputTextWithHint("##gmsearch", "Search name or description...",
                             gmSearchBuf_, sizeof(gmSearchBuf_));
    ImGui::SameLine();
    ImGui::SetNextItemWidth(150.0f);
    const char* secLabels[] = {"Player", "Moderator", "Game Master", "Administrator", "Console"};
    ImGui::Combo("Max level", &gmMaxSecurity_, secLabels, IM_ARRAYSIZE(secLabels));
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Fill in the fields and press Send. Commands go to the\n"
                          "server as chat (\".\" prefix); the server enforces your\n"
                          "real permission level, so a listed command may still\n"
                          "be denied. Leave a player/name field blank to act on\n"
                          "your current target.");
    }
    ImGui::Separator();

    const std::string filter = toLower(gmSearchBuf_);

    auto passesFilters = [&](const GmCommandEntry& c) -> bool {
        if (static_cast<int>(c.security) > gmMaxSecurity_) return false;
        if (filter.empty()) return true;
        return toLower(c.name).find(filter) != std::string::npos ||
               toLower(c.help).find(filter) != std::string::npos;
    };

    auto drawRow = [&](int idx) {
        const auto& c = kGmCommands[idx];
        ImGui::PushID(idx);
        std::string label = "." + std::string(c.name);
        if (ImGui::Selectable(label.c_str(), gmSelectedIndex_ == idx))
            gmSelectedIndex_ = idx;
        if (ImGui::IsItemHovered() && !c.help.empty())
            ImGui::SetTooltip("%s\n%s", std::string(c.syntax).c_str(),
                              std::string(c.help).c_str());
        ImGui::PopID();
    };

    // ---- Left: command list ----
    if (ImGui::BeginChild("##gmlist", ImVec2(290.0f, 0.0f), true)) {
        if (!filter.empty()) {
            int shown = 0;
            for (int i = 0; i < static_cast<int>(kGmCommands.size()); ++i) {
                if (!passesFilters(kGmCommands[i])) continue;
                drawRow(i);
                ++shown;
            }
            if (shown == 0) ImGui::TextDisabled("No matching commands.");
        } else {
            std::string_view curGroup;
            bool haveGroup = false;
            bool headerOpen = false;
            for (int i = 0; i < static_cast<int>(kGmCommands.size()); ++i) {
                if (static_cast<int>(kGmCommands[i].security) > gmMaxSecurity_) continue;
                std::string_view g = firstToken(kGmCommands[i].name);
                if (!haveGroup || g != curGroup) {
                    curGroup = g;
                    haveGroup = true;
                    headerOpen = ImGui::CollapsingHeader(std::string(g).c_str());
                }
                if (headerOpen) drawRow(i);
            }
        }
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // ---- Right: form + execute ----
    if (ImGui::BeginChild("##gmdetail", ImVec2(0.0f, 0.0f), true)) {
        if (gmSelectedIndex_ < 0 || gmSelectedIndex_ >= static_cast<int>(kGmCommands.size())) {
            ImGui::TextDisabled("Select a command from the list.");
        } else {
            const auto& c = kGmCommands[gmSelectedIndex_];

            // Reset field state whenever the selected command changes.
            if (gmArgsForIndex_ != gmSelectedIndex_) {
                gmArgsForIndex_ = gmSelectedIndex_;
                gmManualEdit_ = false;
                std::memset(gmArgBuf_, 0, sizeof(gmArgBuf_));
                std::memset(gmArgChoice_, 0, sizeof(gmArgChoice_));
            }

            ImGui::TextColored(ImVec4(1.0f, 0.82f, 0.28f, 1.0f), ".%s",
                               std::string(c.name).c_str());
            ImGui::SameLine();
            ImGui::TextColored(securityColor(c.security), "[%s]", securityLabel(c.security));
            if (!c.help.empty()) ImGui::TextWrapped("%s", std::string(c.help).c_str());
            ImGui::TextDisabled("Syntax: %s", std::string(c.syntax).c_str());
            ImGui::Separator();

            const std::vector<GmArg> args = parseArgs(c.name, c.syntax);

            // Assemble the command from the field values.
            std::string assembled = "." + std::string(c.name);

            if (gmManualEdit_) {
                ImGui::TextDisabled("Manual command line");
                ImGui::SetNextItemWidth(-1.0f);
                ImGui::InputText("##gmmanual", gmCommandBuf_, sizeof(gmCommandBuf_));
                assembled = gmCommandBuf_;
            } else if (args.empty()) {
                ImGui::TextDisabled("This command takes no arguments.");
            } else {
                ImGui::TextDisabled("Arguments");
                for (int i = 0; i < static_cast<int>(args.size()); ++i) {
                    const GmArg& a = args[i];
                    ImGui::PushID(i);
                    std::string lbl = a.label + (a.optional ? " (optional)" : "");
                    if (a.kind == GmArgKind::Flag) {
                        bool on = gmArgChoice_[i] != 0;
                        if (ImGui::Checkbox(a.label.c_str(), &on)) gmArgChoice_[i] = on ? 1 : 0;
                        if (on) assembled += " " + a.choices[0];
                    } else if (a.kind == GmArgKind::Choice) {
                        // Build a "(unset)"-prefixed option list for optional choices.
                        std::vector<const char*> items;
                        if (a.optional) items.push_back("(unset)");
                        for (const auto& o : a.choices) items.push_back(o.c_str());
                        ImGui::SetNextItemWidth(180.0f);
                        ImGui::Combo(lbl.c_str(), &gmArgChoice_[i], items.data(),
                                     static_cast<int>(items.size()));
                        const char* sel = items[std::clamp(gmArgChoice_[i], 0,
                                                           static_cast<int>(items.size()) - 1)];
                        if (!(a.optional && gmArgChoice_[i] == 0))
                            assembled += " " + std::string(sel);
                    } else {
                        const bool numeric = (a.kind == GmArgKind::Number);
                        ImGui::SetNextItemWidth(180.0f);
                        ImGuiInputTextFlags f = numeric ? ImGuiInputTextFlags_CharsDecimal : 0;
                        ImGui::InputText(lbl.c_str(), gmArgBuf_[i], sizeof(gmArgBuf_[i]), f);
                        if (gmArgBuf_[i][0] != '\0') assembled += " " + std::string(gmArgBuf_[i]);
                        // Player/name fields default to the current target when blank.
                        if (!numeric &&
                            (a.label == "player" || a.label == "name" || a.label == "target")) {
                            ImGui::SameLine();
                            ImGui::TextDisabled("(blank = target)");
                        }
                    }
                    ImGui::PopID();
                }
            }

            ImGui::Separator();
            ImGui::TextDisabled("Will send:");
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.7f, 0.9f, 0.7f, 1.0f), "%s", assembled.c_str());

            if (ImGui::Button("Send", ImVec2(90.0f, 0.0f))) {
                std::string cmd = assembled;
                while (!cmd.empty() && cmd.back() == ' ') cmd.pop_back();
                if (cmd.size() > 1) {
                    gameHandler.sendChatMessage(game::ChatType::SAY, cmd, "");
                    gameHandler.addSystemChatMessage("GM command sent: " + cmd);
                }
            }
            ImGui::SameLine();
            if (ImGui::Checkbox("Edit manually", &gmManualEdit_)) {
                if (gmManualEdit_)
                    std::snprintf(gmCommandBuf_, sizeof(gmCommandBuf_), "%s", assembled.c_str());
            }
        }
    }
    ImGui::EndChild();

    ImGui::End();
}

} // namespace ui
} // namespace wowee
