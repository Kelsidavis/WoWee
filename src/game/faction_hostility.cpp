#include "game/faction_hostility.hpp"
#include "game/game_handler.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_layout.hpp"
#include "core/logger.hpp"
#include <unordered_map>

namespace wowee {
namespace game {

void buildFactionHostilityMap(pipeline::AssetManager& assetManager, GameHandler& gameHandler, uint8_t playerRace) {
    if (!assetManager.isInitialized()) return;

    auto ftDbc = assetManager.loadDBC("FactionTemplate.dbc");
    auto fDbc = assetManager.loadDBC("Faction.dbc");
    if (!ftDbc || !ftDbc->isLoaded()) return;

    // Race enum -> race mask bit: race 1=0x1, 2=0x2, 3=0x4, 4=0x8, 5=0x10, 6=0x20, 7=0x40, 8=0x80, 10=0x200, 11=0x400
    uint32_t playerRaceMask = 0;
    if (playerRace >= 1 && playerRace <= 8) {
        playerRaceMask = 1u << (playerRace - 1);
    } else if (playerRace == 10) {
        playerRaceMask = 0x200;  // Blood Elf
    } else if (playerRace == 11) {
        playerRaceMask = 0x400;  // Draenei
    }

    // Race -> player faction template ID
    // Human=1, Orc=2, Dwarf=3, NightElf=4, Undead=5, Tauren=6, Gnome=115, Troll=116, BloodElf=1610, Draenei=1629
    uint32_t playerFtId = 0;
    switch (playerRace) {
        case 1: playerFtId = 1; break;     // Human
        case 2: playerFtId = 2; break;     // Orc
        case 3: playerFtId = 3; break;     // Dwarf
        case 4: playerFtId = 4; break;     // Night Elf
        case 5: playerFtId = 5; break;     // Undead
        case 6: playerFtId = 6; break;     // Tauren
        case 7: playerFtId = 115; break;   // Gnome
        case 8: playerFtId = 116; break;   // Troll
        case 10: playerFtId = 1610; break; // Blood Elf
        case 11: playerFtId = 1629; break; // Draenei
        default: playerFtId = 1; break;
    }

    // The factions a player has a standing with, and whether they start at war.
    //
    // For one of these the standing decides, not the groups: the game treats
    // its units as hostile while the player is at war with the faction and as
    // friendly otherwise. A player starts at war when their starting standing
    // is Hostile or worse, unless the faction's flags force peace. This used to
    // call any starting standing below zero hostile - and every Alliance race
    // starts at -1200 with the Kurenai, Unfriendly and peace-forced, so all of
    // Telaar came out hostile and a right-click on its quest givers attacked.
    //
    // At war by default only; GameHandler::isHostileFaction reads the server's
    // own flag once the standings have arrived.
    const auto* facL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("Faction") : nullptr;
    const auto* ftL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("FactionTemplate") : nullptr;
    struct RepFaction { uint32_t repListId = 0; bool atWar = false; };
    std::unordered_map<uint32_t, RepFaction> repFactions;
    if (fDbc && fDbc->isLoaded()) {
        const uint32_t facID = facL ? (*facL)["ID"] : 0;
        const uint32_t facRepList = facL ? (*facL)["ReputationListID"] : 1;
        const uint32_t facRaceMask0 = facL ? (*facL)["ReputationRaceMask0"] : 2;
        const uint32_t facBase0 = facL ? (*facL)["ReputationBase0"] : 10;
        const uint32_t facFlags0 = facL ? (*facL)["ReputationFlags0"] : 14;
        constexpr int32_t kHostileRankTop = -3001;  // Hostile runs -6000 to -3001
        uint32_t atWarCount = 0;
        for (uint32_t i = 0; i < fDbc->getRecordCount(); i++) {
            const uint32_t repListId =
                facRepList != 0xFFFFFFFF ? fDbc->getUInt32(i, facRepList) : 0xFFFFFFFF;
            if (repListId == 0xFFFFFFFF) continue;  // no standing to have
            int32_t baseRep = 0;
            uint32_t repFlags = 0;
            for (int slot = 0; slot < 4; slot++) {
                uint32_t raceMask = fDbc->getUInt32(i, facRaceMask0 + slot);
                if (raceMask & playerRaceMask) {
                    baseRep = fDbc->getInt32(i, facBase0 + slot);
                    if (facFlags0 != 0xFFFFFFFF) repFlags = fDbc->getUInt32(i, facFlags0 + slot);
                    break;
                }
            }
            const bool atWar = (repFlags & GameHandler::FACTION_FLAG_AT_WAR) != 0 ||
                               (baseRep <= kHostileRankTop &&
                                (repFlags & GameHandler::FACTION_FLAG_PEACE_FORCED) == 0);
            repFactions[fDbc->getUInt32(i, facID)] = {repListId, atWar};
            if (atWar) ++atWarCount;
        }
        LOG_INFO("Faction.dbc: ", repFactions.size(), " factions with a standing, ",
                 atWarCount, " at war with race ", static_cast<int>(playerRace), " from the start");
    }

    // Get player faction template data
    const uint32_t ftID = ftL ? (*ftL)["ID"] : 0;
    const uint32_t ftFaction = ftL ? (*ftL)["Faction"] : 1;
    const uint32_t ftFG = ftL ? (*ftL)["FactionGroup"] : 3;
    const uint32_t ftFriend = ftL ? (*ftL)["FriendGroup"] : 4;
    const uint32_t ftEnemy = ftL ? (*ftL)["EnemyGroup"] : 5;
    const uint32_t ftEnemy0 = ftL ? (*ftL)["Enemy0"] : 6;
    uint32_t playerFriendGroup = 0;
    uint32_t playerEnemyGroup = 0;
    uint32_t playerFactionId = 0;
    // Kept apart from playerFriendGroup, which folds the two together: the
    // friendly test below needs to ask each direction on its own.
    uint32_t playerFactionGroup = 0;
    for (uint32_t i = 0; i < ftDbc->getRecordCount(); i++) {
        if (ftDbc->getUInt32(i, ftID) == playerFtId) {
            playerFriendGroup = ftDbc->getUInt32(i, ftFriend) | ftDbc->getUInt32(i, ftFG);
            playerFactionGroup = ftDbc->getUInt32(i, ftFG);
            playerEnemyGroup = ftDbc->getUInt32(i, ftEnemy);
            playerFactionId = ftDbc->getUInt32(i, ftFaction);
            break;
        }
    }

    // Build hostility map for each faction template
    std::unordered_map<uint32_t, bool> factionMap;
    // And a friendly map beside it. Not the negation of the other one: a
    // faction can be neither, and most wildlife is. A beneficial spell may only
    // be aimed at a unit that is actually friendly - "not hostile" let a neutral
    // boar hold the cast, and the server refused it as an invalid target
    // instead of the spell falling back to the caster.
    std::unordered_map<uint32_t, bool> friendlyMap;
    // Each template whose faction carries a standing, by the server's index for
    // that standing, so the live at-war flag can be asked for later.
    std::unordered_map<uint32_t, uint32_t> templateRepList;
    for (uint32_t i = 0; i < ftDbc->getRecordCount(); i++) {
        uint32_t id = ftDbc->getUInt32(i, ftID);
        uint32_t parentFaction = ftDbc->getUInt32(i, ftFaction);
        uint32_t factionGroup = ftDbc->getUInt32(i, ftFG);
        uint32_t friendGroup = ftDbc->getUInt32(i, ftFriend);
        uint32_t enemyGroup = ftDbc->getUInt32(i, ftEnemy);

        // 1. Symmetric group check
        bool hostile = (enemyGroup & playerFriendGroup) != 0
                    || (factionGroup & playerEnemyGroup) != 0;

        // 2. Monster factionGroup bit (8)
        if (!hostile && (factionGroup & 8) != 0) {
            hostile = true;
        }

        // 3. Individual enemy faction IDs
        if (!hostile && playerFactionId > 0) {
            for (uint32_t e = ftEnemy0; e <= ftEnemy0 + 3; e++) {
                if (ftDbc->getUInt32(i, e) == playerFactionId) {
                    hostile = true;
                    break;
                }
            }
        }

        // 4. If explicitly friendly (friendGroup includes player), override to non-hostile
        if (hostile && (friendGroup & playerFriendGroup) != 0) {
            hostile = false;
        }

        // Friendly is mutual regard between the two groups, asked in both
        // directions, and never true for something already hostile.
        bool friendly = !hostile &&
            (((friendGroup & playerFactionGroup) != 0) ||
             ((factionGroup & playerFriendGroup) != 0));

        // 5. A faction with a standing overrides all of it: at war is hostile,
        // anything else friendly. See repFactions above.
        if (auto rep = repFactions.find(parentFaction); rep != repFactions.end()) {
            hostile = rep->second.atWar;
            friendly = !hostile;
            templateRepList[id] = rep->second.repListId;
        }

        factionMap[id] = hostile;
        friendlyMap[id] = friendly;
    }

    uint32_t hostileCount = 0;
    for (const auto& [fid, h] : factionMap) { if (h) hostileCount++; }
    uint32_t friendlyCount = 0;
    for (const auto& [fid, f] : friendlyMap) { if (f) friendlyCount++; }
    LOG_INFO("Faction friendliness: ", friendlyCount, "/", friendlyMap.size(),
             " templates friendly; the rest are hostile or neutral");
    gameHandler.setFactionTemplateRepList(std::move(templateRepList));
    gameHandler.setFactionFriendlyMap(std::move(friendlyMap));
    gameHandler.setFactionHostileMap(std::move(factionMap));
    LOG_INFO("Faction hostility for race ", static_cast<int>(playerRace), " (FT ", playerFtId, "): ",
        hostileCount, "/", ftDbc->getRecordCount(),
        " hostile (friendGroup=0x", std::hex, playerFriendGroup, ", enemyGroup=0x", playerEnemyGroup, std::dec, ")");
}

} // namespace game
} // namespace wowee
