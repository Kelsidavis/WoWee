#include "rendering/animation/emote_registry.hpp"
#include "rendering/animation/animation_ids.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/dbc_layout.hpp"
#include "core/application.hpp"
#include "core/logger.hpp"

#include <algorithm>
#include <cctype>
#include <unordered_set>

namespace wowee {
namespace rendering {

// ── Helper functions (moved from animation_controller.cpp) ───────────────────

static std::vector<std::string> parseEmoteCommands(const std::string& raw) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : raw) {
        if (std::isalnum(static_cast<unsigned char>(c)) || c == '_') {
            cur.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
        } else if (!cur.empty()) {
            out.push_back(cur);
            cur.clear();
        }
    }
    if (!cur.empty()) out.push_back(cur);
    return out;
}

static bool isLoopingEmote(const std::string& command) {
    static const std::unordered_set<std::string> kLooping = {
        "dance", "train", "dead", "eat", "work", "sleep",
    };
    return kLooping.find(command) != kLooping.end();
}

// Map one-shot emote animation IDs to their persistent EMOTE_STATE_* looping variants.
// When a looping emote is played, we prefer the STATE variant if the model has it.
static uint32_t getEmoteStateVariantStatic(uint32_t oneShotAnimId) {
    static const std::unordered_map<uint32_t, uint32_t> kStateMap = {
        {anim::EMOTE_DANCE,         anim::EMOTE_STATE_DANCE},
        {anim::EMOTE_LAUGH,         anim::EMOTE_STATE_LAUGH},
        {anim::EMOTE_POINT,         anim::EMOTE_STATE_POINT},
        {anim::EMOTE_EAT,           anim::EMOTE_STATE_EAT},
        {anim::EMOTE_ROAR,          anim::EMOTE_STATE_ROAR},
        {anim::EMOTE_APPLAUD,       anim::EMOTE_STATE_APPLAUD},
        {anim::EMOTE_WORK,          anim::EMOTE_STATE_WORK},
        {anim::EMOTE_USE_STANDING,  anim::EMOTE_STATE_USE_STANDING},
        {anim::EATING_LOOP,         anim::EMOTE_STATE_EAT},
    };
    auto it = kStateMap.find(oneShotAnimId);
    return it != kStateMap.end() ? it->second : 0;
}

static std::unordered_map<uint32_t, uint32_t> makeFallbackEmotesIdMap() {
    // Emotes.dbc IDs used on classic-family MaNGOS/CMaNGOS servers.
    return {
        {1, anim::EMOTE_TALK},
        {2, anim::EMOTE_BOW},
        {3, anim::EMOTE_WAVE},
        {4, anim::EMOTE_CHEER},
        {5, anim::EMOTE_EXCLAMATION},
        {6, anim::EMOTE_QUESTION},
        {7, anim::EMOTE_EAT},
        {10, anim::EMOTE_DANCE},
        {11, anim::EMOTE_LAUGH},
        {12, anim::EMOTE_SLEEP},
        {13, anim::EMOTE_SIT_GROUND},
        {14, anim::EMOTE_RUDE},
        {15, anim::EMOTE_ROAR},
        {16, anim::EMOTE_KNEEL},
        {17, anim::EMOTE_KISS},
        {18, anim::EMOTE_CRY},
        {19, anim::EMOTE_CHICKEN},
        {20, anim::EMOTE_BEG},
        {21, anim::EMOTE_APPLAUD},
        {22, anim::EMOTE_SHOUT},
        {23, anim::EMOTE_FLEX},
        {24, anim::EMOTE_SHY},
        {25, anim::EMOTE_POINT},
        {33, anim::COMBAT_WOUND},
        {34, anim::COMBAT_CRITICAL},
        {35, anim::ATTACK_UNARMED},
        {36, anim::ATTACK_1H},
        {37, anim::ATTACK_2H},
        {38, anim::ATTACK_2H_LOOSE},
        {50, anim::SPELL_PRECAST},
        {51, anim::SPELL_CAST},
        {53, anim::BATTLE_ROAR},
        {60, anim::KICK},
        {66, anim::EMOTE_SALUTE},
        {68, anim::KNEEL_LOOP},
        {69, anim::EMOTE_USE_STANDING},
        {70, anim::EMOTE_WAVE},
        {71, anim::EMOTE_CHEER},
        {92, anim::EMOTE_EAT},
        {94, anim::EMOTE_DANCE},
        {113, anim::EMOTE_SALUTE},
        {133, anim::EMOTE_USE_STANDING_NO_SHEATHE},
        {153, anim::EMOTE_LAUGH},
        {173, anim::EMOTE_WORK},
        {193, anim::SPELL_PRECAST},
        {213, anim::READY_RIFLE},
        {214, anim::HOLD_RIFLE},
        {233, anim::EMOTE_WORK},
        {234, anim::EMOTE_CHOP},
        {253, anim::EMOTE_APPLAUD},
        {273, anim::EMOTE_TALK_EXCLAMATION},
        {274, anim::EMOTE_TALK_QUESTION},
        {275, anim::EMOTE_TRAIN},
    };
}

// Emotes.dbc IDs with EmoteSpecProc != 0 (persistent STATE_ emotes) that appear
// in the fallback map above. Used only when Emotes.dbc is unavailable.
static std::unordered_set<uint32_t> makeFallbackStateEmoteIds() {
    return {10, 12, 13, 26, 28, 64, 65, 68, 69, 173, 213, 214, 233, 234, 379, 383};
}

static std::string replacePlaceholders(const std::string& text, const std::string* targetName) {
    if (text.empty()) return text;
    std::string out;
    out.reserve(text.size() + 16);
    for (size_t i = 0; i < text.size(); ++i) {
        if (text[i] == '%' && i + 1 < text.size() && text[i + 1] == 's') {
            if (targetName && !targetName->empty()) out += *targetName;
            i++;
        } else {
            out.push_back(text[i]);
        }
    }
    return out;
}

// ── EmoteRegistry implementation ─────────────────────────────────────────────

EmoteRegistry& EmoteRegistry::instance() {
    static EmoteRegistry inst;
    return inst;
}

void EmoteRegistry::loadFromDbc() {
    if (loaded_) return;
    loaded_ = true;

    auto* assetManager = core::Application::getInstance().getAssetManager();
    if (!assetManager) {
        LOG_WARNING("Emotes: no AssetManager");
        loadFallbackEmotes();
        return;
    }

    auto emotesTextDbc = assetManager->loadDBC("EmotesText.dbc");
    auto emotesTextDataDbc = assetManager->loadDBC("EmotesTextData.dbc");
    if (!emotesTextDbc || !emotesTextDataDbc || !emotesTextDbc->isLoaded() || !emotesTextDataDbc->isLoaded()) {
        LOG_WARNING("Emotes: DBCs not available (EmotesText/EmotesTextData)");
        loadFallbackEmotes();
        return;
    }

    const auto* activeLayout = pipeline::getActiveDBCLayout();
    const auto* etdL = activeLayout ? activeLayout->getLayout("EmotesTextData") : nullptr;
    const auto* emL  = activeLayout ? activeLayout->getLayout("Emotes") : nullptr;
    const auto* etL  = activeLayout ? activeLayout->getLayout("EmotesText") : nullptr;

    std::unordered_map<uint32_t, std::string> textData;
    textData.reserve(emotesTextDataDbc->getRecordCount());
    for (uint32_t r = 0; r < emotesTextDataDbc->getRecordCount(); ++r) {
        uint32_t id = emotesTextDataDbc->getUInt32(r, etdL ? (*etdL)["ID"] : 0);
        std::string text = emotesTextDataDbc->getString(r, etdL ? (*etdL)["Text"] : 1);
        if (!text.empty()) textData.emplace(id, std::move(text));
    }

    std::unordered_map<uint32_t, uint32_t> emoteIdToAnim;
    animByEmotesId_.clear();
    stateEmoteIds_.clear();
    if (auto emotesDbc = assetManager->loadDBC("Emotes.dbc"); emotesDbc && emotesDbc->isLoaded()) {
        emoteIdToAnim.reserve(emotesDbc->getRecordCount());
        animByEmotesId_.reserve(emotesDbc->getRecordCount());
        // EmoteSpecProc: 0 = one-shot, 1 = stand-state change, 2 = looping state
        // anim. Column 4 in every expansion's Emotes.dbc.
        uint32_t specField = emL ? (*emL)["EmoteSpecProc"] : 4;
        if (specField == 0xFFFFFFFF) specField = 4;
        for (uint32_t r = 0; r < emotesDbc->getRecordCount(); ++r) {
            uint32_t emoteId = emotesDbc->getUInt32(r, emL ? (*emL)["ID"] : 0);
            uint32_t animId = emotesDbc->getUInt32(r, emL ? (*emL)["AnimID"] : 2);
            if (emotesDbc->getUInt32(r, specField) != 0) {
                stateEmoteIds_.insert(emoteId);
            }
            if (animId != 0) {
                emoteIdToAnim[emoteId] = animId;
                animByEmotesId_[emoteId] = animId;
            }
        }
        LOG_DEBUG("Emotes: loaded ", emoteIdToAnim.size(), " anim mappings from Emotes.dbc");
    } else {
        LOG_WARNING("Emotes: Emotes.dbc failed to load - all emotes will use fallback animations");
    }

    emoteTable_.clear();
    emoteTable_.reserve(emotesTextDbc->getRecordCount());
    for (uint32_t r = 0; r < emotesTextDbc->getRecordCount(); ++r) {
        uint32_t recordId = emotesTextDbc->getUInt32(r, etL ? (*etL)["ID"] : 0);
        std::string cmdRaw = emotesTextDbc->getString(r, etL ? (*etL)["Command"] : 1);
        if (cmdRaw.empty()) continue;

        uint32_t emoteRef = emotesTextDbc->getUInt32(r, etL ? (*etL)["EmoteRef"] : 2);
        uint32_t animId = 0;
        if (emoteRef != 0) {
            auto animIt = emoteIdToAnim.find(emoteRef);
            if (animIt != emoteIdToAnim.end()) {
                animId = animIt->second;
            }
            // If Emotes.dbc has AnimID=0 for this ref, leave animId=0 (text-only).
            // Previously fell back to using emoteRef as animId which is wrong.
        }

        uint32_t senderTargetTextId = emotesTextDbc->getUInt32(r, etL ? (*etL)["SenderTargetTextID"] : 5);
        uint32_t senderNoTargetTextId = emotesTextDbc->getUInt32(r, etL ? (*etL)["SenderNoTargetTextID"] : 9);
        uint32_t othersTargetTextId = emotesTextDbc->getUInt32(r, etL ? (*etL)["OthersTargetTextID"] : 3);
        uint32_t othersNoTargetTextId = emotesTextDbc->getUInt32(r, etL ? (*etL)["OthersNoTargetTextID"] : 7);

        std::string textTarget, textNoTarget, oTarget, oNoTarget;
        if (auto it = textData.find(senderTargetTextId); it != textData.end()) textTarget = it->second;
        if (auto it = textData.find(senderNoTargetTextId); it != textData.end()) textNoTarget = it->second;
        if (auto it = textData.find(othersTargetTextId); it != textData.end()) oTarget = it->second;
        if (auto it = textData.find(othersNoTargetTextId); it != textData.end()) oNoTarget = it->second;

        for (const std::string& cmd : parseEmoteCommands(cmdRaw)) {
            if (cmd.empty()) continue;
            EmoteInfo info;
            info.animId = animId;
            info.dbcId = recordId;
            info.loop = isLoopingEmote(cmd);
            info.textNoTarget = textNoTarget;
            info.textTarget = textTarget;
            info.othersNoTarget = oNoTarget;
            info.othersTarget = oTarget;
            info.command = cmd;
            emoteTable_.emplace(cmd, std::move(info));
        }
    }

    // Override emotes whose DBC chain yields animId=0.
    // /sleep uses the stand-state system in WoW rather than Emotes.dbc AnimID.
    // /laugh and /flirt should resolve from Emotes.dbc (70 and 83), but these
    // serve as backup if Emotes.dbc failed to load.
    // /fart and /stink have EmoteRef=0 in EmotesText.dbc - no Emotes.dbc link.
    static const std::unordered_map<std::string, uint32_t> kAnimOverrides = {
        {"sleep",   anim::EMOTE_SLEEP},     // 71 - stand-state emote
        {"laugh",   anim::EMOTE_LAUGH},     // 70 - backup
        {"flirt",   anim::EMOTE_SHY},       // 83 - DBC calls it SHY; it's the flirt animation
        {"fart",    anim::EMOTE_TALK},       // 60 - generic gesture (WoW has no dedicated anim)
        {"stink",   anim::EMOTE_TALK},       // 60 - generic gesture (WoW has no dedicated anim)
    };
    for (auto& [cmd, info] : emoteTable_) {
        if (info.animId == 0) {
            auto ov = kAnimOverrides.find(cmd);
            if (ov != kAnimOverrides.end()) {
                LOG_DEBUG("Emotes: override /", cmd, " → animId=", ov->second);
                info.animId = ov->second;
            }
        }
    }

    if (emoteTable_.empty()) {
        LOG_DEBUG("Emotes: DBC loaded but no commands parsed, using fallback list");
        loadFallbackEmotes();
    } else {
        LOG_DEBUG("Emotes: loaded ", emoteTable_.size(), " commands from DBC");
    }
    if (animByEmotesId_.empty()) {
        animByEmotesId_ = makeFallbackEmotesIdMap();
        stateEmoteIds_ = makeFallbackStateEmoteIds();
    }

    buildDbcIdIndex();
}

void EmoteRegistry::loadFallbackEmotes() {
    if (!emoteTable_.empty()) return;
    animByEmotesId_ = makeFallbackEmotesIdMap();
    stateEmoteIds_ = makeFallbackStateEmoteIds();
    emoteTable_ = {
        {"wave",    {anim::EMOTE_WAVE,    0, false, "You wave.", "You wave at %s.", "%s waves.", "%s waves at %s.", "wave"}},
        {"bow",     {anim::EMOTE_BOW,     0, false, "You bow down graciously.", "You bow down before %s.", "%s bows down graciously.", "%s bows down before %s.", "bow"}},
        {"laugh",   {anim::EMOTE_LAUGH,   0, false, "You laugh.", "You laugh at %s.", "%s laughs.", "%s laughs at %s.", "laugh"}},
        {"point",   {anim::EMOTE_POINT,   0, false, "You point over yonder.", "You point at %s.", "%s points over yonder.", "%s points at %s.", "point"}},
        {"cheer",   {anim::EMOTE_CHEER,   0, false, "You cheer!", "You cheer at %s.", "%s cheers!", "%s cheers at %s.", "cheer"}},
        {"dance",   {anim::EMOTE_DANCE,   0, true,  "You burst into dance.", "You dance with %s.", "%s bursts into dance.", "%s dances with %s.", "dance"}},
        {"kneel",   {anim::EMOTE_KNEEL,   0, false, "You kneel down.", "You kneel before %s.", "%s kneels down.", "%s kneels before %s.", "kneel"}},
        {"applaud", {anim::EMOTE_APPLAUD, 0, false, "You applaud. Bravo!", "You applaud at %s. Bravo!", "%s applauds. Bravo!", "%s applauds at %s. Bravo!", "applaud"}},
        {"shout",   {anim::EMOTE_SHOUT,   0, false, "You shout.", "You shout at %s.", "%s shouts.", "%s shouts at %s.", "shout"}},
        {"chicken", {anim::EMOTE_CHICKEN, 0, false, "With arms flapping, you strut around. Cluck, Cluck, Chicken!",
                     "With arms flapping, you strut around %s. Cluck, Cluck, Chicken!",
                     "%s struts around. Cluck, Cluck, Chicken!", "%s struts around %s. Cluck, Cluck, Chicken!", "chicken"}},
        {"cry",     {anim::EMOTE_CRY,     0, false, "You cry.", "You cry on %s's shoulder.", "%s cries.", "%s cries on %s's shoulder.", "cry"}},
        {"kiss",    {anim::EMOTE_KISS,    0, false, "You blow a kiss into the wind.", "You blow a kiss to %s.", "%s blows a kiss into the wind.", "%s blows a kiss to %s.", "kiss"}},
        {"roar",    {anim::EMOTE_ROAR,    0, false, "You roar with bestial vigor. So fierce!", "You roar with bestial vigor at %s. So fierce!", "%s roars with bestial vigor. So fierce!", "%s roars with bestial vigor at %s. So fierce!", "roar"}},
        {"salute",  {anim::EMOTE_SALUTE,  0, false, "You salute.", "You salute %s with respect.", "%s salutes.", "%s salutes %s with respect.", "salute"}},
        {"rude",    {anim::EMOTE_RUDE,    0, false, "You make a rude gesture.", "You make a rude gesture at %s.", "%s makes a rude gesture.", "%s makes a rude gesture at %s.", "rude"}},
        {"flex",    {anim::EMOTE_FLEX,    0, false, "You flex your muscles. Oooooh so strong!", "You flex at %s. Oooooh so strong!", "%s flexes. Oooooh so strong!", "%s flexes at %s. Oooooh so strong!", "flex"}},
        {"shy",     {anim::EMOTE_SHY,     0, false, "You smile shyly.", "You smile shyly at %s.", "%s smiles shyly.", "%s smiles shyly at %s.", "shy"}},
        {"beg",     {anim::EMOTE_BEG,     0, false, "You beg everyone around you. How pathetic.", "You beg %s. How pathetic.", "%s begs everyone around. How pathetic.", "%s begs %s. How pathetic.", "beg"}},
        {"eat",     {anim::EMOTE_EAT,     0, true,  "You begin to eat.", "You begin to eat in front of %s.", "%s begins to eat.", "%s begins to eat in front of %s.", "eat"}},
        {"talk",    {anim::EMOTE_TALK,    0, false, "You talk.", "You talk to %s.", "%s talks.", "%s talks to %s.", "talk"}},
        {"work",    {anim::EMOTE_WORK,    0, true,  "You begin to work.", "You begin to work near %s.", "%s begins to work.", "%s begins to work near %s.", "work"}},
        {"train",   {anim::EMOTE_TRAIN,   0, true,  "You let off a train whistle. Choo Choo!", "You let off a train whistle at %s. Choo Choo!", "%s lets off a train whistle. Choo Choo!", "%s lets off a train whistle at %s. Choo Choo!", "train"}},
        {"dead",    {anim::EMOTE_DEAD,    0, true,  "You play dead.", "You play dead in front of %s.", "%s plays dead.", "%s plays dead in front of %s.", "dead"}},
    };
    buildDbcIdIndex();
}

void EmoteRegistry::buildDbcIdIndex() {
    emoteByDbcId_.clear();
    for (auto& [cmd, info] : emoteTable_) {
        if (info.dbcId != 0) {
            emoteByDbcId_.emplace(info.dbcId, &info);
        }
    }
}

std::optional<EmoteRegistry::EmoteResult> EmoteRegistry::findEmote(const std::string& command) const {
    auto it = emoteTable_.find(command);
    if (it == emoteTable_.end()) return std::nullopt;
    const auto& info = it->second;
    if (info.animId == 0) return std::nullopt;
    return EmoteResult{info.animId, info.loop};
}
uint32_t EmoteRegistry::animByEmotesId(uint32_t emoteId) const {
    auto it = animByEmotesId_.find(emoteId);
    return it != animByEmotesId_.end() ? it->second : 0;
}

bool EmoteRegistry::isStateEmote(uint32_t emoteId) const {
    return stateEmoteIds_.count(emoteId) != 0;
}

uint32_t EmoteRegistry::getStateVariant(uint32_t oneShotAnimId) const {
    return getEmoteStateVariantStatic(oneShotAnimId);
}
std::string EmoteRegistry::textFor(const std::string& emoteName,
                                   const std::string* targetName) const {
    auto it = emoteTable_.find(emoteName);
    if (it != emoteTable_.end()) {
        const auto& info = it->second;
        const std::string& base = (targetName ? info.textTarget : info.textNoTarget);
        if (!base.empty()) {
            return replacePlaceholders(base, targetName);
        }
        if (targetName && !targetName->empty()) {
            return "You " + info.command + " at " + *targetName + ".";
        }
        return "You " + info.command + ".";
    }
    return "";
}

uint32_t EmoteRegistry::dbcIdFor(const std::string& emoteName) const {
    auto it = emoteTable_.find(emoteName);
    if (it != emoteTable_.end()) {
        return it->second.dbcId;
    }
    return 0;
}

std::string EmoteRegistry::textByDbcId(uint32_t dbcId,
                                       const std::string& senderName,
                                       const std::string* targetName) const {
    auto it = emoteByDbcId_.find(dbcId);
    if (it == emoteByDbcId_.end()) return "";

    const EmoteInfo& info = *it->second;

    if (targetName && !targetName->empty()) {
        if (!info.othersTarget.empty()) {
            std::string out;
            out.reserve(info.othersTarget.size() + senderName.size() + targetName->size());
            bool firstReplaced = false;
            for (size_t i = 0; i < info.othersTarget.size(); ++i) {
                if (info.othersTarget[i] == '%' && i + 1 < info.othersTarget.size() && info.othersTarget[i + 1] == 's') {
                    out += firstReplaced ? *targetName : senderName;
                    firstReplaced = true;
                    ++i;
                } else {
                    out.push_back(info.othersTarget[i]);
                }
            }
            return out;
        }
        return senderName + " " + info.command + "s at " + *targetName + ".";
    }         if (!info.othersNoTarget.empty()) {
            return replacePlaceholders(info.othersNoTarget, &senderName);
        }
        return senderName + " " + info.command + "s.";
   
}

} // namespace rendering
} // namespace wowee
