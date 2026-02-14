#include "pipeline/dbc_layout.hpp"
#include "core/logger.hpp"
#include <fstream>
#include <sstream>

namespace wowee {
namespace pipeline {

static const DBCLayout* g_activeDBCLayout = nullptr;

void setActiveDBCLayout(const DBCLayout* layout) { g_activeDBCLayout = layout; }
const DBCLayout* getActiveDBCLayout() { return g_activeDBCLayout; }

void DBCLayout::loadWotlkDefaults() {
    layouts_.clear();

    // Spell.dbc
    layouts_["Spell"] = {{{ "ID", 0 }, { "Attributes", 4 }, { "IconID", 133 },
        { "Name", 136 }, { "Tooltip", 139 }, { "Rank", 153 }}};

    // ItemDisplayInfo.dbc
    layouts_["ItemDisplayInfo"] = {{{ "ID", 0 }, { "LeftModel", 1 }, { "LeftModelTexture", 3 },
        { "InventoryIcon", 5 }, { "GeosetGroup1", 7 }, { "GeosetGroup3", 9 }}};

    // CharSections.dbc
    layouts_["CharSections"] = {{{ "RaceID", 1 }, { "SexID", 2 }, { "BaseSection", 3 },
        { "VariationIndex", 4 }, { "ColorIndex", 5 },
        { "Texture1", 6 }, { "Texture2", 7 }, { "Texture3", 8 }}};

    // SpellIcon.dbc (Icon.dbc in code but actually SpellIcon)
    layouts_["SpellIcon"] = {{{ "ID", 0 }, { "Path", 1 }}};

    // FactionTemplate.dbc
    layouts_["FactionTemplate"] = {{{ "ID", 0 }, { "Faction", 1 }, { "FactionGroup", 3 },
        { "FriendGroup", 4 }, { "EnemyGroup", 5 },
        { "Enemy0", 6 }, { "Enemy1", 7 }, { "Enemy2", 8 }, { "Enemy3", 9 }}};

    // Faction.dbc
    layouts_["Faction"] = {{{ "ID", 0 }, { "ReputationRaceMask0", 2 }, { "ReputationRaceMask1", 3 },
        { "ReputationRaceMask2", 4 }, { "ReputationRaceMask3", 5 },
        { "ReputationBase0", 10 }, { "ReputationBase1", 11 },
        { "ReputationBase2", 12 }, { "ReputationBase3", 13 }}};

    // AreaTable.dbc
    layouts_["AreaTable"] = {{{ "ID", 0 }, { "ExploreFlag", 3 }}};

    // CreatureDisplayInfoExtra.dbc
    layouts_["CreatureDisplayInfoExtra"] = {{{ "ID", 0 }, { "RaceID", 1 }, { "SexID", 2 },
        { "SkinID", 3 }, { "FaceID", 4 }, { "HairStyleID", 5 }, { "HairColorID", 6 },
        { "FacialHairID", 7 }, { "EquipDisplay0", 8 }, { "EquipDisplay1", 9 },
        { "EquipDisplay2", 10 }, { "EquipDisplay3", 11 }, { "EquipDisplay4", 12 },
        { "EquipDisplay5", 13 }, { "EquipDisplay6", 14 }, { "EquipDisplay7", 15 },
        { "EquipDisplay8", 16 }, { "EquipDisplay9", 17 }, { "EquipDisplay10", 18 },
        { "BakeName", 20 }}};

    // CreatureDisplayInfo.dbc
    layouts_["CreatureDisplayInfo"] = {{{ "ID", 0 }, { "ModelID", 1 }, { "ExtraDisplayId", 3 },
        { "Skin1", 6 }, { "Skin2", 7 }, { "Skin3", 8 }}};

    // TaxiNodes.dbc
    layouts_["TaxiNodes"] = {{{ "ID", 0 }, { "MapID", 1 }, { "X", 2 }, { "Y", 3 }, { "Z", 4 },
        { "Name", 5 }, { "MountDisplayIdAllianceFallback", 20 },
        { "MountDisplayIdHordeFallback", 21 },
        { "MountDisplayIdAlliance", 22 }, { "MountDisplayIdHorde", 23 }}};

    // TaxiPath.dbc
    layouts_["TaxiPath"] = {{{ "ID", 0 }, { "FromNode", 1 }, { "ToNode", 2 }, { "Cost", 3 }}};

    // TaxiPathNode.dbc
    layouts_["TaxiPathNode"] = {{{ "ID", 0 }, { "PathID", 1 }, { "NodeIndex", 2 },
        { "MapID", 3 }, { "X", 4 }, { "Y", 5 }, { "Z", 6 }}};

    // TalentTab.dbc
    layouts_["TalentTab"] = {{{ "ID", 0 }, { "Name", 1 }, { "ClassMask", 20 },
        { "OrderIndex", 22 }, { "BackgroundFile", 23 }}};

    // Talent.dbc
    layouts_["Talent"] = {{{ "ID", 0 }, { "TabID", 1 }, { "Row", 2 }, { "Column", 3 },
        { "RankSpell0", 4 }, { "PrereqTalent0", 9 }, { "PrereqRank0", 12 }}};

    // SkillLineAbility.dbc
    layouts_["SkillLineAbility"] = {{{ "SkillLineID", 1 }, { "SpellID", 2 }}};

    // SkillLine.dbc
    layouts_["SkillLine"] = {{{ "ID", 0 }, { "Category", 1 }, { "Name", 3 }}};

    // Map.dbc
    layouts_["Map"] = {{{ "ID", 0 }, { "InternalName", 1 }}};

    // CreatureModelData.dbc
    layouts_["CreatureModelData"] = {{{ "ID", 0 }, { "ModelPath", 2 }}};

    // CharHairGeosets.dbc
    layouts_["CharHairGeosets"] = {{{ "RaceID", 1 }, { "SexID", 2 },
        { "Variation", 3 }, { "GeosetID", 4 }}};

    // CharacterFacialHairStyles.dbc
    layouts_["CharacterFacialHairStyles"] = {{{ "RaceID", 0 }, { "SexID", 1 },
        { "Variation", 2 }, { "Geoset100", 3 }, { "Geoset300", 4 }, { "Geoset200", 5 }}};

    // GameObjectDisplayInfo.dbc
    layouts_["GameObjectDisplayInfo"] = {{{ "ID", 0 }, { "ModelName", 1 }}};

    // Emotes.dbc
    layouts_["Emotes"] = {{{ "ID", 0 }, { "AnimID", 2 }}};

    // EmotesText.dbc
    // Fields 3-18 are 16 EmotesTextData refs: [others+target, target+target, sender+target, ?,
    //   others+notarget, ?, sender+notarget, ?, female variants...]
    layouts_["EmotesText"] = {{{ "ID", 0 }, { "Command", 1 }, { "EmoteRef", 2 },
        { "OthersTargetTextID", 3 }, { "SenderTargetTextID", 5 },
        { "OthersNoTargetTextID", 7 }, { "SenderNoTargetTextID", 9 }}};

    // EmotesTextData.dbc
    layouts_["EmotesTextData"] = {{{ "ID", 0 }, { "Text", 1 }}};

    // Light.dbc
    layouts_["Light"] = {{{ "ID", 0 }, { "MapID", 1 }, { "X", 2 }, { "Z", 3 }, { "Y", 4 },
        { "InnerRadius", 5 }, { "OuterRadius", 6 }, { "LightParamsID", 7 },
        { "LightParamsIDRain", 8 }, { "LightParamsIDUnderwater", 9 }}};

    // LightParams.dbc
    layouts_["LightParams"] = {{{ "LightParamsID", 0 }}};

    // LightParamsBands.dbc (custom split from LightIntBand/LightFloatBand)
    layouts_["LightParamsBands"] = {{{ "BlockIndex", 1 }, { "NumKeyframes", 2 },
        { "TimeKey0", 3 }, { "Value0", 19 }}};

    // LightIntBand.dbc (same structure as LightParamsBands)
    layouts_["LightIntBand"] = {{{ "BlockIndex", 1 }, { "NumKeyframes", 2 },
        { "TimeKey0", 3 }, { "Value0", 19 }}};

    // LightFloatBand.dbc
    layouts_["LightFloatBand"] = {{{ "BlockIndex", 1 }, { "NumKeyframes", 2 },
        { "TimeKey0", 3 }, { "Value0", 19 }}};

    // WorldMapArea.dbc
    layouts_["WorldMapArea"] = {{{ "ID", 0 }, { "MapID", 1 }, { "AreaID", 2 },
        { "AreaName", 3 }, { "LocLeft", 4 }, { "LocRight", 5 }, { "LocTop", 6 },
        { "LocBottom", 7 }, { "DisplayMapID", 8 }, { "ParentWorldMapID", 10 }}};

    LOG_INFO("DBCLayout: loaded ", layouts_.size(), " WotLK default layouts");
}

bool DBCLayout::loadFromJson(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        LOG_WARNING("DBCLayout: cannot open ", path);
        return false;
    }

    std::string json((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

    layouts_.clear();
    size_t loaded = 0;
    size_t pos = 0;

    // Parse top-level object: { "DbcName": { "FieldName": index, ... }, ... }
    // Find the first '{'
    pos = json.find('{', pos);
    if (pos == std::string::npos) return false;
    ++pos;

    while (pos < json.size()) {
        // Find DBC name key
        size_t dbcKeyStart = json.find('"', pos);
        if (dbcKeyStart == std::string::npos) break;
        size_t dbcKeyEnd = json.find('"', dbcKeyStart + 1);
        if (dbcKeyEnd == std::string::npos) break;
        std::string dbcName = json.substr(dbcKeyStart + 1, dbcKeyEnd - dbcKeyStart - 1);

        // Find the nested object '{'
        size_t objStart = json.find('{', dbcKeyEnd);
        if (objStart == std::string::npos) break;

        // Find the matching '}'
        size_t objEnd = json.find('}', objStart);
        if (objEnd == std::string::npos) break;

        // Parse the inner object
        std::string inner = json.substr(objStart + 1, objEnd - objStart - 1);
        DBCFieldMap fieldMap;
        size_t ipos = 0;
        while (ipos < inner.size()) {
            size_t fkStart = inner.find('"', ipos);
            if (fkStart == std::string::npos) break;
            size_t fkEnd = inner.find('"', fkStart + 1);
            if (fkEnd == std::string::npos) break;
            std::string fieldName = inner.substr(fkStart + 1, fkEnd - fkStart - 1);

            size_t colon = inner.find(':', fkEnd);
            if (colon == std::string::npos) break;
            size_t valStart = colon + 1;
            while (valStart < inner.size() && (inner[valStart] == ' ' || inner[valStart] == '\t' ||
                   inner[valStart] == '\r' || inner[valStart] == '\n'))
                ++valStart;
            size_t valEnd = inner.find_first_of(",}\r\n", valStart);
            if (valEnd == std::string::npos) valEnd = inner.size();
            std::string valStr = inner.substr(valStart, valEnd - valStart);
            while (!valStr.empty() && (valStr.back() == ' ' || valStr.back() == '\t'))
                valStr.pop_back();

            try {
                uint32_t idx = static_cast<uint32_t>(std::stoul(valStr));
                fieldMap.fields[fieldName] = idx;
            } catch (...) {}

            ipos = valEnd + 1;
        }

        if (!fieldMap.fields.empty()) {
            layouts_[dbcName] = std::move(fieldMap);
            ++loaded;
        }

        pos = objEnd + 1;
    }

    LOG_INFO("DBCLayout: loaded ", loaded, " layouts from ", path);
    return loaded > 0;
}

const DBCFieldMap* DBCLayout::getLayout(const std::string& dbcName) const {
    auto it = layouts_.find(dbcName);
    return (it != layouts_.end()) ? &it->second : nullptr;
}

} // namespace pipeline
} // namespace wowee
