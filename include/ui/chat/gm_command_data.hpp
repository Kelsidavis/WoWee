// GM command metadata - names, security levels, syntax, help text.
// Sourced from AzerothCore GM commands wiki.
// Used for tab-completion and .gmhelp display; the server handles execution.
#pragma once

#include <vector>
#include <string>
#include <algorithm>
#include <array>
#include <string_view>
#include <cstdint>

namespace wowee {
namespace ui {

struct GmCommandEntry {
    std::string_view name;       // e.g. "gm on"
    uint8_t          security;   // 0=player, 1=mod, 2=gm, 3=admin, 4=console
    std::string_view syntax;     // e.g. ".gm [on/off]"
    std::string_view help;       // short description
};

// Curated list of the most useful GM commands for a client emulator.
// The full AzerothCore list has 500+ commands - this table covers the
// ones players/GMs actually type regularly, organized by category.
// NOLINTBEGIN(modernize-use-designated-initializers) - 206 rows whose four
// columns are their field names, with the struct in view directly above.
inline constexpr std::array kGmCommands = {

    // ── GM mode & info ──────────────────────────────────────
    GmCommandEntry{"gm",              1, ".gm [on/off]",                       "Toggle GM mode or show state"},
    GmCommandEntry{"gm on",           1, ".gm on",                             "Enable GM mode"},
    GmCommandEntry{"gm off",          1, ".gm off",                            "Disable GM mode"},
    GmCommandEntry{"gm fly",          2, ".gm fly [on/off]",                   "Toggle fly mode"},
    GmCommandEntry{"gm visible",      2, ".gm visible [on/off]",              "Toggle GM visibility"},
    GmCommandEntry{"gm chat",         2, ".gm chat [on/off]",                 "Toggle GM chat badge"},
    GmCommandEntry{"gm ingame",       0, ".gm ingame",                         "List online GMs"},
    GmCommandEntry{"gm list",         3, ".gm list",                           "List all GM accounts"},

    // ── Teleportation ───────────────────────────────────────
    GmCommandEntry{"tele",            1, ".tele #location",                     "Teleport to location"},
    GmCommandEntry{"tele group",      2, ".tele group #location",              "Teleport group to location"},
    GmCommandEntry{"tele name",       2, ".tele name $player #location",       "Teleport player to location"},
    GmCommandEntry{"go xyz",          1, ".go xyz #x #y [#z [#map [#o]]]",    "Teleport to coordinates"},
    GmCommandEntry{"go creature",     1, ".go creature #guid",                 "Teleport to creature by GUID"},
    GmCommandEntry{"go creature id",  1, ".go creature id #entry",             "Teleport to creature by entry"},
    GmCommandEntry{"go gameobject",   1, ".go gameobject #guid",               "Teleport to gameobject"},
    GmCommandEntry{"go graveyard",    1, ".go graveyard #id",                  "Teleport to graveyard"},
    GmCommandEntry{"go taxinode",     1, ".go taxinode #id",                   "Teleport to taxinode"},
    GmCommandEntry{"go trigger",      1, ".go trigger #id",                    "Teleport to areatrigger"},
    GmCommandEntry{"go zonexy",       1, ".go zonexy #x #y [#zone]",          "Teleport to zone coordinates"},
    GmCommandEntry{"appear",          1, ".appear $player",                     "Teleport to player"},
    GmCommandEntry{"summon",          2, ".summon $player",                     "Summon player to you"},
    GmCommandEntry{"groupsummon",     2, ".groupsummon $player",               "Summon player and group"},
    GmCommandEntry{"recall",          2, ".recall [$player]",                   "Return to pre-teleport location"},
    GmCommandEntry{"unstuck",         2, ".unstuck $player [inn/graveyard]",   "Unstuck player"},
    GmCommandEntry{"gps",             1, ".gps",                                "Show current position info"},

    // ── Character & level ───────────────────────────────────
    GmCommandEntry{"levelup",         2, ".levelup [$player] [#levels]",       "Increase player level"},
    GmCommandEntry{"character level", 3, ".character level [$player] [#lvl]",  "Set character level"},
    GmCommandEntry{"character rename", 2, ".character rename [$name]",         "Flag character for rename"},
    GmCommandEntry{"character changefaction", 2, ".character changefaction $name", "Flag for faction change"},
    GmCommandEntry{"character changerace", 2, ".character changerace $name",   "Flag for race change"},
    GmCommandEntry{"character customize", 2, ".character customize [$name]",   "Flag for customization"},
    GmCommandEntry{"character reputation", 2, ".character reputation [$name]", "Show reputation info"},
    GmCommandEntry{"character titles", 2, ".character titles [$name]",         "Show known titles"},
    GmCommandEntry{"pinfo",           2, ".pinfo [$player]",                    "Show player account info"},
    GmCommandEntry{"guid",            2, ".guid",                               "Show target GUID"},

    // ── Reset ───────────────────────────────────────────────
    GmCommandEntry{"reset level",     3, ".reset level [$player]",             "Reset level to 1"},
    GmCommandEntry{"reset stats",     3, ".reset stats [$player]",             "Reset stats to level default"},
    GmCommandEntry{"reset spells",    3, ".reset spells [$player]",            "Reset to starting spells"},
    GmCommandEntry{"reset talents",   3, ".reset talents [$player]",           "Refund all talent points"},
    GmCommandEntry{"reset achievements", 3, ".reset achievements [$player]",   "Wipe all achievements"},
    GmCommandEntry{"reset all",       3, ".reset all spells/talents",          "Reset all players (spells or talents)"},

    // ── Items & inventory ───────────────────────────────────
    GmCommandEntry{"additem",         2, ".additem #id [#count]",              "Add item to inventory"},
    GmCommandEntry{"additem set",     2, ".additem set #setid",                "Add item set to inventory"},
    GmCommandEntry{"additemset",      2, ".additemset #setid",                 "Add item set to inventory"},
    GmCommandEntry{"repairitems",     2, ".repairitems [$player]",             "Repair all equipped items"},

    // ── Spells & auras ──────────────────────────────────────
    GmCommandEntry{"learn",           2, ".learn #spell [all]",                "Learn a spell"},
    GmCommandEntry{"unlearn",         2, ".unlearn #spell [all]",              "Unlearn a spell"},
    GmCommandEntry{"learn all my class", 2, ".learn all my class",             "Learn all class spells"},
    GmCommandEntry{"learn all my spells", 2, ".learn all my spells",           "Learn all available spells"},
    GmCommandEntry{"learn all my talents", 2, ".learn all my talents",         "Learn all talents"},
    GmCommandEntry{"learn all crafts", 2, ".learn all crafts",                 "Learn all professions"},
    GmCommandEntry{"learn all lang",  2, ".learn all lang",                    "Learn all languages"},
    GmCommandEntry{"learn all recipes", 2, ".learn all recipes [$prof]",       "Learn all recipes"},
    GmCommandEntry{"cast",            2, ".cast #spell [triggered]",           "Cast spell on target"},
    GmCommandEntry{"cast self",       2, ".cast self #spell",                  "Cast spell on self"},
    GmCommandEntry{"aura",            2, ".aura #spell",                       "Add aura to target"},
    GmCommandEntry{"unaura",          2, ".unaura #spell",                     "Remove aura from target"},
    GmCommandEntry{"cooldown",        2, ".cooldown [#spell]",                 "Remove cooldowns"},
    GmCommandEntry{"maxskill",        2, ".maxskill",                           "Max all skills for target"},
    GmCommandEntry{"setskill",        2, ".setskill #skill #level [#max]",    "Set skill level"},

    // ── Modify stats ────────────────────────────────────────
    GmCommandEntry{"modify money",    2, ".modify money #amount",              "Add/remove money"},
    GmCommandEntry{"modify hp",       2, ".modify hp #value",                  "Set current HP"},
    GmCommandEntry{"modify mana",     2, ".modify mana #value",               "Set current mana"},
    GmCommandEntry{"modify energy",   2, ".modify energy #value",             "Set current energy"},
    GmCommandEntry{"modify speed all", 2, ".modify speed all #rate",          "Set all movement speeds"},
    GmCommandEntry{"modify speed fly", 2, ".modify speed fly #rate",          "Set fly speed"},
    GmCommandEntry{"modify speed walk", 2, ".modify speed walk #rate",        "Set walk speed"},
    GmCommandEntry{"modify speed swim", 2, ".modify speed swim #rate",        "Set swim speed"},
    GmCommandEntry{"modify mount",    2, ".modify mount #id #speed",           "Display as mounted"},
    GmCommandEntry{"modify scale",    2, ".modify scale #rate",                "Set model scale"},
    GmCommandEntry{"modify honor",    2, ".modify honor #amount",              "Add honor points"},
    GmCommandEntry{"modify reputation", 2, ".modify reputation #faction #val", "Set faction reputation"},
    GmCommandEntry{"modify talentpoints", 2, ".modify talentpoints #amount",  "Set talent points"},
    GmCommandEntry{"modify gender",   2, ".modify gender male/female",         "Change gender"},
    GmCommandEntry{"modify arenapoints", 3, ".modify arenapoints #amount",     "Set arena points"},
    GmCommandEntry{"modify drunk",    2, ".modify drunk #value",               "Set drunk level (0-100)"},
    GmCommandEntry{"modify faction",  3, ".modify faction #factionid",         "Change target faction template"},
    GmCommandEntry{"modify xp",       2, ".modify xp #amount",                 "Give experience points"},
    GmCommandEntry{"modify phase",    2, ".modify phase #mask",                "Set phase mask"},

    // ── Cheats ──────────────────────────────────────────────
    GmCommandEntry{"cheat god",       2, ".cheat god [on/off]",                "Toggle god mode"},
    GmCommandEntry{"cheat casttime",  2, ".cheat casttime [on/off]",          "Toggle no cast time"},
    GmCommandEntry{"cheat cooldown",  2, ".cheat cooldown [on/off]",          "Toggle no cooldowns"},
    GmCommandEntry{"cheat power",     2, ".cheat power [on/off]",             "Toggle no mana cost"},
    GmCommandEntry{"cheat explore",   2, ".cheat explore #flag",               "Reveal/hide all maps"},
    GmCommandEntry{"cheat taxi",      2, ".cheat taxi on/off",                 "Toggle all taxi routes"},
    GmCommandEntry{"cheat waterwalk", 2, ".cheat waterwalk on/off",           "Toggle waterwalk"},
    GmCommandEntry{"cheat status",    2, ".cheat status",                      "Show active cheats"},

    // ── NPC ─────────────────────────────────────────────────
    GmCommandEntry{"npc add",         3, ".npc add #entry",                    "Spawn creature"},
    GmCommandEntry{"npc delete",      3, ".npc delete [#guid]",               "Delete creature"},
    GmCommandEntry{"npc info",        1, ".npc info",                           "Show NPC details"},
    GmCommandEntry{"npc guid",        1, ".npc guid",                           "Show NPC GUID"},
    GmCommandEntry{"npc near",        2, ".npc near [#dist]",                  "List nearby NPCs"},
    GmCommandEntry{"npc say",         2, ".npc say $message",                  "Make NPC say text"},
    GmCommandEntry{"npc yell",        2, ".npc yell $message",                 "Make NPC yell text"},
    GmCommandEntry{"npc move",        3, ".npc move [#guid]",                  "Move NPC to your position"},
    GmCommandEntry{"npc set level",   3, ".npc set level #level",             "Set NPC level"},
    GmCommandEntry{"npc set model",   3, ".npc set model #displayid",         "Set NPC model"},
    GmCommandEntry{"npc tame",        2, ".npc tame",                           "Tame selected creature"},

    // ── Game objects ────────────────────────────────────────
    GmCommandEntry{"gobject add",     3, ".gobject add #entry",                "Spawn gameobject"},
    GmCommandEntry{"gobject delete",  3, ".gobject delete #guid",              "Delete gameobject"},
    GmCommandEntry{"gobject info",    1, ".gobject info [#entry]",             "Show gameobject info"},
    GmCommandEntry{"gobject near",    3, ".gobject near [#dist]",              "List nearby objects"},
    GmCommandEntry{"gobject move",    3, ".gobject move #guid [#x #y #z]",    "Move gameobject"},
    GmCommandEntry{"gobject target",  1, ".gobject target [#id]",              "Find nearest gameobject"},
    GmCommandEntry{"gobject activate", 2, ".gobject activate #guid",           "Activate object (door/button)"},

    // ── Combat & death ──────────────────────────────────────
    GmCommandEntry{"revive",          2, ".revive",                             "Revive selected/self"},
    GmCommandEntry{"die",             2, ".die",                                "Kill selected/self"},
    GmCommandEntry{"damage",          2, ".damage #amount [#school [#spell]]", "Deal damage to target"},
    GmCommandEntry{"combatstop",      2, ".combatstop [$player]",              "Stop combat for target"},
    GmCommandEntry{"freeze",          2, ".freeze [$player]",                   "Freeze player"},
    GmCommandEntry{"unfreeze",        2, ".unfreeze [$player]",                "Unfreeze player"},
    GmCommandEntry{"dismount",        0, ".dismount",                           "Dismount if mounted"},
    GmCommandEntry{"respawn",         2, ".respawn",                            "Respawn nearby creatures/GOs"},
    GmCommandEntry{"respawn all",     2, ".respawn all",                        "Respawn all nearby"},

    // ── Quests ──────────────────────────────────────────────
    GmCommandEntry{"quest add",       2, ".quest add #id",                     "Add quest to log"},
    GmCommandEntry{"quest complete",  2, ".quest complete #id",                "Complete quest objectives"},
    GmCommandEntry{"quest remove",    2, ".quest remove #id",                  "Remove quest from log"},
    GmCommandEntry{"quest reward",    2, ".quest reward #id",                  "Grant quest reward"},
    GmCommandEntry{"quest status",    2, ".quest status #id [$name]",          "Show quest status"},

    // ── Honor & arena ───────────────────────────────────────
    GmCommandEntry{"honor add",       2, ".honor add #amount",                 "Add honor points"},
    GmCommandEntry{"honor update",    2, ".honor update",                       "Force honor field update"},
    GmCommandEntry{"achievement add", 2, ".achievement add #id",               "Add achievement to target"},

    // ── Group & guild ───────────────────────────────────────
    GmCommandEntry{"group list",      2, ".group list [$player]",              "List group members"},
    GmCommandEntry{"group revive",    2, ".group revive $player",              "Revive group members"},
    GmCommandEntry{"group disband",   2, ".group disband [$player]",           "Disband player's group"},
    GmCommandEntry{"guild create",    2, ".guild create $leader \"$name\"",    "Create guild"},
    GmCommandEntry{"guild delete",    2, ".guild delete \"$name\"",            "Delete guild"},
    GmCommandEntry{"guild invite",    2, ".guild invite $player \"$guild\"",   "Add player to guild"},
    GmCommandEntry{"guild info",      2, ".guild info",                         "Show guild info"},

    // ── Lookup & search ─────────────────────────────────────
    GmCommandEntry{"lookup item",     1, ".lookup item $name",                 "Search item by name"},
    GmCommandEntry{"lookup spell",    1, ".lookup spell $name",                "Search spell by name"},
    GmCommandEntry{"lookup spell id", 1, ".lookup spell id #id",               "Look up spell by ID"},
    GmCommandEntry{"lookup creature", 1, ".lookup creature $name",             "Search creature by name"},
    GmCommandEntry{"lookup quest",    1, ".lookup quest $name",                "Search quest by name"},
    GmCommandEntry{"lookup gobject",  1, ".lookup gobject $name",              "Search gameobject by name"},
    GmCommandEntry{"lookup area",     1, ".lookup area $name",                 "Search area by name"},
    GmCommandEntry{"lookup taxinode", 1, ".lookup taxinode $name",             "Search taxinode by name"},
    GmCommandEntry{"lookup teleport", 1, ".lookup teleport $name",            "Search teleport by name"},
    GmCommandEntry{"lookup faction",  1, ".lookup faction $name",              "Search faction by name"},
    GmCommandEntry{"lookup title",    1, ".lookup title $name",                "Search title by name"},
    GmCommandEntry{"lookup event",    1, ".lookup event $name",                "Search event by name"},
    GmCommandEntry{"lookup map",      1, ".lookup map $name",                  "Search map by name"},
    GmCommandEntry{"lookup skill",    1, ".lookup skill $name",                "Search skill by name"},

    // ── Titles ──────────────────────────────────────────────
    GmCommandEntry{"titles add",      2, ".titles add #id",                    "Add title to target"},
    GmCommandEntry{"titles remove",   2, ".titles remove #id",                 "Remove title from target"},
    GmCommandEntry{"titles current",  2, ".titles current #id",                "Set current title"},

    // ── Morph & display ─────────────────────────────────────
    GmCommandEntry{"morph",           1, ".morph #displayid",                  "Change your model"},
    GmCommandEntry{"morph target",    1, ".morph target #displayid",           "Change target model"},
    GmCommandEntry{"morph mount",     1, ".morph mount #displayid",            "Change mount model"},
    GmCommandEntry{"morph reset",     1, ".morph reset",                        "Reset target model"},

    // ── Debug & info ────────────────────────────────────────
    GmCommandEntry{"debug anim",      3, ".debug anim",                        "Debug animation"},
    GmCommandEntry{"debug arena",     3, ".debug arena",                        "Toggle arena debug"},
    GmCommandEntry{"debug bg",        3, ".debug bg",                           "Toggle BG debug"},
    GmCommandEntry{"debug los",       3, ".debug los",                          "Show line of sight info"},
    GmCommandEntry{"debug loot",      2, ".debug loot $type $id [#count]",    "Simulate loot generation"},
    GmCommandEntry{"list auras",      1, ".list auras",                         "List auras on target"},
    GmCommandEntry{"list creature",   1, ".list creature #id [#max]",          "List creature spawns"},
    GmCommandEntry{"list item",       1, ".list item #id [#max]",              "List item locations"},

    // ── Server & system ─────────────────────────────────────
    GmCommandEntry{"announce",        2, ".announce $message",                 "Broadcast to all players"},
    GmCommandEntry{"gmannounce",      2, ".gmannounce $message",              "Broadcast to online GMs"},
    GmCommandEntry{"notify",          2, ".notify $message",                    "On-screen broadcast"},
    GmCommandEntry{"server info",     0, ".server info",                        "Show server version/players"},
    GmCommandEntry{"server motd",     0, ".server motd",                        "Show message of the day"},
    GmCommandEntry{"commands",        0, ".commands",                           "List available commands"},
    GmCommandEntry{"help",            0, ".help [$cmd]",                        "Show command help"},
    GmCommandEntry{"save",            0, ".save",                               "Save your character"},
    GmCommandEntry{"saveall",         2, ".saveall",                            "Save all characters"},

    // ── Account & bans ──────────────────────────────────────
    GmCommandEntry{"account",         0, ".account",                            "Show account info"},
    GmCommandEntry{"account set gmlevel", 4, ".account set gmlevel $acct #lvl", "Set GM security level"},
    GmCommandEntry{"ban account",     2, ".ban account $name $time $reason",   "Ban account"},
    GmCommandEntry{"ban character",   2, ".ban character $name $time $reason", "Ban character"},
    GmCommandEntry{"ban ip",          2, ".ban ip $ip $time $reason",          "Ban IP address"},
    GmCommandEntry{"unban account",   3, ".unban account $name",               "Unban account"},
    GmCommandEntry{"unban character", 3, ".unban character $name",             "Unban character"},
    GmCommandEntry{"unban ip",        3, ".unban ip $ip",                       "Unban IP address"},
    GmCommandEntry{"kick",            2, ".kick [$player] [$reason]",          "Kick player from world"},
    GmCommandEntry{"mute",            2, ".mute $player $minutes [$reason]",   "Mute player chat"},
    GmCommandEntry{"unmute",          2, ".unmute [$player]",                   "Unmute player"},

    // ── Misc ────────────────────────────────────────────────
    GmCommandEntry{"distance",        3, ".distance",                           "Distance to selected target"},
    GmCommandEntry{"wchange",         3, ".wchange #type #grade",              "Change weather"},
    GmCommandEntry{"mailbox",         1, ".mailbox",                            "Open mailbox"},
    GmCommandEntry{"played",          0, ".played",                             "Show time played"},
    GmCommandEntry{"gear repair",     2, ".gear repair",                        "Repair all gear"},
    GmCommandEntry{"gear stats",      0, ".gear stats",                         "Show avg item level"},
    GmCommandEntry{"pet create",      2, ".pet create",                         "Create pet from target"},
    GmCommandEntry{"pet learn",       2, ".pet learn #spell",                  "Teach spell to pet"},

    // ── Waypoints ───────────────────────────────────────────
    GmCommandEntry{"wp add",          3, ".wp add",                             "Add waypoint at your position"},
    GmCommandEntry{"wp show",         3, ".wp show on/off",                    "Toggle waypoint display"},
    GmCommandEntry{"wp load",         3, ".wp load #pathid",                   "Load path for creature"},
    GmCommandEntry{"wp unload",       3, ".wp unload",                          "Unload creature path"},

    // ── Instance ────────────────────────────────────────────
    GmCommandEntry{"instance listbinds", 1, ".instance listbinds",             "Show instance binds"},
    GmCommandEntry{"instance unbind",    2, ".instance unbind <map|all>",      "Clear instance binds"},
    GmCommandEntry{"instance stats",     1, ".instance stats",                  "Show instance stats"},

    // ── Events ──────────────────────────────────────────────
    GmCommandEntry{"event activelist", 2, ".event activelist",                 "Show active events"},
    GmCommandEntry{"event start",      2, ".event start #id",                  "Start event"},
    GmCommandEntry{"event stop",       2, ".event stop #id",                   "Stop event"},
    GmCommandEntry{"event info",       2, ".event info #id",                   "Show event info"},

    // ── Reload (common) ─────────────────────────────────────
    GmCommandEntry{"reload all",      3, ".reload all",                        "Reload all tables"},
    GmCommandEntry{"reload creature_template", 3, ".reload creature_template #entry", "Reload creature template"},
    GmCommandEntry{"reload quest_template", 3, ".reload quest_template",       "Reload quest templates"},
    GmCommandEntry{"reload config",   3, ".reload config",                      "Reload server config"},
    GmCommandEntry{"reload game_tele", 3, ".reload game_tele",                 "Reload teleport locations"},

    // ── Ticket ──────────────────────────────────────────────
    GmCommandEntry{"ticket list",     2, ".ticket list",                        "List open GM tickets"},
    GmCommandEntry{"ticket close",    2, ".ticket close #id",                  "Close ticket"},
    GmCommandEntry{"ticket delete",   3, ".ticket delete #id",                 "Delete ticket permanently"},
    GmCommandEntry{"ticket viewid",   2, ".ticket viewid #id",                 "View ticket details"},
};
// NOLINTEND(modernize-use-designated-initializers)

/// The dot-commands that start with `prefix`, sorted.
///
/// Beside the table it reads rather than in the command file, because the
/// command file includes game_handler.hpp and a test that wanted this had to
/// write the ten lines out again instead of linking it. A test asserting
/// against its own copy of the logic cannot fail when the real one changes,
/// and that copy carried a comment saying it "matches chat_panel.cpp" - which
/// is a claim about another file that nothing checked.
inline std::vector<std::string> gmCompletionsFor(const std::string& prefix) {
    std::vector<std::string> results;
    for (const auto& cmd : kGmCommands) {
        std::string dotName = "." + std::string(cmd.name);
        if (dotName.size() >= prefix.size() &&
            dotName.compare(0, prefix.size(), prefix) == 0) {
            results.push_back(dotName);
        }
    }
    std::sort(results.begin(), results.end());
    return results;
}

} // namespace ui
} // namespace wowee
