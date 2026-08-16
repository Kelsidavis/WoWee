// Combat commands: /cast, /castsequence, /use, /equip, /equipset,
//                  /startattack, /stopattack, /stopcasting, /cancelqueuedspell
// Moved from ChatPanel::sendChatMessage() if/else chain (Phase 3).
#include "ui/chat/i_chat_command.hpp"
#include "ui/chat/macro_evaluator.hpp"
#include "ui/chat_panel.hpp"
#include "game/game_handler.hpp"
#include "game/inventory.hpp"
#include <imgui.h>
#include <algorithm>
#include <cctype>
#include <sstream>

namespace wowee { namespace ui {

// --------------- helpers (local to this TU) ---------------
namespace {

// Trim leading/trailing whitespace from a string in-place.
inline void trimInPlace(std::string& s) {
    while (!s.empty() && s.front() == ' ') s.erase(s.begin());
    while (!s.empty() && s.back()  == ' ') s.pop_back();
}

// Try to parse a spell by name among known spells, optionally with a specific
// rank.  Returns the best matching spell ID, or 0 if none found.
uint32_t resolveSpellByName(game::GameHandler& gh, const std::string& spellArg, int requestedRank = -1) {
    std::string spellName = spellArg;
    // Parse optional "(Rank N)" suffix
    {
        auto rankPos = spellArg.find('(');
        if (rankPos != std::string::npos) {
            std::string rankStr = spellArg.substr(rankPos + 1);
            auto closePos = rankStr.find(')');
            if (closePos != std::string::npos) rankStr = rankStr.substr(0, closePos);
            for (char& c : rankStr) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (rankStr.rfind("rank ", 0) == 0) {
                try { requestedRank = std::stoi(rankStr.substr(5)); } catch (...) {}
            }
            spellName = spellArg.substr(0, rankPos);
            while (!spellName.empty() && spellName.back() == ' ') spellName.pop_back();
        }
    }

    std::string spellNameLower = spellName;
    for (char& c : spellNameLower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

    uint32_t bestId = 0;
    int bestRank = -1;
    for (uint32_t sid : gh.getKnownSpells()) {
        const std::string& sName = gh.getSpellName(sid);
        if (sName.empty()) continue;
        std::string sLow = sName;
        for (char& c : sLow) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (sLow != spellNameLower) continue;

        int sRank = 0;
        const std::string& rk = gh.getSpellRank(sid);
        if (!rk.empty()) {
            std::string rLow = rk;
            for (char& c : rLow) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (rLow.rfind("rank ", 0) == 0) {
                try { sRank = std::stoi(rLow.substr(5)); } catch (...) {}
            }
        }
        if (requestedRank >= 0) {
            if (sRank == requestedRank) return sid;
        } else {
            if (sRank > bestRank) { bestRank = sRank; bestId = sid; }
        }
    }
    return bestId;
}

// Try to parse numeric spell ID (with optional '#' prefix). Returns 0 if not numeric.
uint32_t parseNumericSpellId(const std::string& str) {
    std::string numStr = str;
    if (!numStr.empty() && numStr.front() == '#') numStr.erase(numStr.begin());
    bool isNumeric = !numStr.empty() &&
        std::all_of(numStr.begin(), numStr.end(),
                    [](unsigned char c){ return std::isdigit(c); });
    if (!isNumeric) return 0;
    try { return static_cast<uint32_t>(std::stoul(numStr)); } catch (...) { return 0; }
}

} // anon namespace

// --- /cast ---
class CastCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (ctx.args.empty()) return {false, false};

        std::string spellArg = ctx.args;
        trimInPlace(spellArg);

        // Evaluate WoW macro conditionals
        uint64_t castTargetOverride = static_cast<uint64_t>(-1);
        if (!spellArg.empty() && spellArg.front() == '[') {
            spellArg = evaluateMacroConditionals(spellArg, ctx.gameHandler, castTargetOverride);
            if (spellArg.empty()) return {};  // no conditional matched
            trimInPlace(spellArg);
        }

        // Strip leading '!' (force recast)
        if (!spellArg.empty() && spellArg.front() == '!') spellArg.erase(spellArg.begin());

        // Numeric spell ID
        uint32_t numId = parseNumericSpellId(spellArg);
        if (numId) {
            uint64_t targetGuid = (castTargetOverride != static_cast<uint64_t>(-1))
                ? castTargetOverride
                : (ctx.gameHandler.hasTarget() ? ctx.gameHandler.getTargetGuid() : 0);
            ctx.gameHandler.castSpell(numId, targetGuid);
            return {};
        }

        // Name-based lookup
        uint32_t bestSpellId = resolveSpellByName(ctx.gameHandler, spellArg);
        if (bestSpellId) {
            uint64_t targetGuid = (castTargetOverride != static_cast<uint64_t>(-1))
                ? castTargetOverride
                : (ctx.gameHandler.hasTarget() ? ctx.gameHandler.getTargetGuid() : 0);
            ctx.gameHandler.castSpell(bestSpellId, targetGuid);
        } else {
            // Build error message
            std::string spellName = spellArg;
            auto rp = spellArg.find('(');
            if (rp != std::string::npos) {
                spellName = spellArg.substr(0, rp);
                while (!spellName.empty() && spellName.back() == ' ') spellName.pop_back();
            }
            // Check if a specific rank was requested for the error message
            int reqRank = -1;
            if (rp != std::string::npos) {
                std::string rs = spellArg.substr(rp + 1);
                auto cp = rs.find(')');
                if (cp != std::string::npos) rs = rs.substr(0, cp);
                for (char& c : rs) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                if (rs.rfind("rank ", 0) == 0) {
                    try { reqRank = std::stoi(rs.substr(5)); } catch (...) {}
                }
            }
            game::MessageChatData sysMsg;
            sysMsg.type = game::ChatType::SYSTEM;
            sysMsg.language = game::ChatLanguage::UNIVERSAL;
            sysMsg.message = (reqRank >= 0)
                ? "You don't know '" + spellName + "' (Rank " + std::to_string(reqRank) + ")."
                : "Unknown spell: '" + spellName + "'.";
            ctx.gameHandler.addLocalChatMessage(sysMsg);
        }
        return {};
    }
    std::vector<std::string> aliases() const override { return {"cast"}; }
    std::string helpText() const override { return "Cast a spell by name or ID"; }
};

// --- /castsequence ---
class CastSequenceCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (ctx.args.empty()) return {false, false};

        std::string seqArg = ctx.args;
        trimInPlace(seqArg);

        // Macro conditionals
        uint64_t seqTgtOver = static_cast<uint64_t>(-1);
        if (!seqArg.empty() && seqArg.front() == '[') {
            seqArg = evaluateMacroConditionals(seqArg, ctx.gameHandler, seqTgtOver);
            if (seqArg.empty() && seqTgtOver == static_cast<uint64_t>(-1)) return {};
            trimInPlace(seqArg);
        }

        // Optional reset= spec
        std::string resetSpec;
        if (seqArg.rfind("reset=", 0) == 0) {
            size_t spAfter = seqArg.find(' ');
            if (spAfter != std::string::npos) {
                resetSpec = seqArg.substr(6, spAfter - 6);
                seqArg = seqArg.substr(spAfter + 1);
                trimInPlace(seqArg);
            }
        }

        // Parse comma-separated spell list
        std::vector<std::string> seqSpells;
        {
            std::string cur;
            for (char c : seqArg) {
                if (c == ',') {
                    trimInPlace(cur);
                    if (!cur.empty()) seqSpells.push_back(cur);
                    cur.clear();
                } else { cur += c; }
            }
            trimInPlace(cur);
            if (!cur.empty()) seqSpells.push_back(cur);
        }
        if (seqSpells.empty()) return {};

        // Build stable key from lowercase spell list
        std::string seqKey;
        for (size_t k = 0; k < seqSpells.size(); ++k) {
            if (k) seqKey += ',';
            std::string sl = seqSpells[k];
            for (char& c : sl) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            seqKey += sl;
        }

        auto& seqState = ctx.panel.getCastSeqTracker().get(seqKey);

        // Check reset conditions
        float nowSec = static_cast<float>(ImGui::GetTime());
        bool shouldReset = false;
        if (!resetSpec.empty()) {
            size_t rpos = 0;
            while (rpos <= resetSpec.size()) {
                size_t slash = resetSpec.find('/', rpos);
                std::string part = (slash != std::string::npos)
                    ? resetSpec.substr(rpos, slash - rpos)
                    : resetSpec.substr(rpos);
                std::string plow = part;
                for (char& c : plow) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                bool isNum = !plow.empty() && std::all_of(plow.begin(), plow.end(),
                    [](unsigned char c){ return std::isdigit(c) || c == '.'; });
                if (isNum) {
                    float rSec = 0.0f;
                    try { rSec = std::stof(plow); } catch (...) {}
                    if (rSec > 0.0f && nowSec - seqState.lastPressSec > rSec) shouldReset = true;
                } else if (plow == "target") {
                    if (ctx.gameHandler.getTargetGuid() != seqState.lastTargetGuid) shouldReset = true;
                } else if (plow == "combat") {
                    if (ctx.gameHandler.isInCombat() != seqState.lastInCombat) shouldReset = true;
                }
                if (slash == std::string::npos) break;
                rpos = slash + 1;
            }
        }
        if (shouldReset || seqState.index >= seqSpells.size()) seqState.index = 0;

        const std::string& seqSpell = seqSpells[seqState.index];
        seqState.index = (seqState.index + 1) % seqSpells.size();
        seqState.lastPressSec   = nowSec;
        seqState.lastTargetGuid = ctx.gameHandler.getTargetGuid();
        seqState.lastInCombat   = ctx.gameHandler.isInCombat();

        // Cast the selected spell
        std::string ssLow = seqSpell;
        for (char& c : ssLow) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (!ssLow.empty() && ssLow.front() == '!') ssLow.erase(ssLow.begin());

        uint64_t seqTargetGuid = (seqTgtOver != static_cast<uint64_t>(-1) && seqTgtOver != 0)
            ? seqTgtOver : (ctx.gameHandler.hasTarget() ? ctx.gameHandler.getTargetGuid() : 0);

        uint32_t numId = parseNumericSpellId(ssLow);
        if (numId) {
            ctx.gameHandler.castSpell(numId, seqTargetGuid);
        } else {
            uint32_t best = resolveSpellByName(ctx.gameHandler, ssLow);
            if (best) ctx.gameHandler.castSpell(best, seqTargetGuid);
        }
        return {};
    }
    std::vector<std::string> aliases() const override { return {"castsequence"}; }
    std::string helpText() const override { return "Cycle through a spell sequence"; }
};

// --- /use ---
class UseCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (ctx.args.empty()) return {false, false};

        std::string useArg = ctx.args;
        trimInPlace(useArg);

        // Handle macro conditionals
        if (!useArg.empty() && useArg.front() == '[') {
            uint64_t dummy = static_cast<uint64_t>(-1);
            useArg = evaluateMacroConditionals(useArg, ctx.gameHandler, dummy);
            if (useArg.empty()) return {};
            trimInPlace(useArg);
        }

        // Check for bag/slot notation: two numbers separated by whitespace
        {
            std::istringstream iss(useArg);
            int bagNum = -1, slotNum = -1;
            iss >> bagNum >> slotNum;
            if (!iss.fail() && slotNum >= 1) {
                if (bagNum == 0) {
                    ctx.gameHandler.useItemBySlot(slotNum - 1, true);
                    return {};
                } if (bagNum >= 1 && bagNum <= game::Inventory::NUM_BAG_SLOTS) {
                    ctx.gameHandler.useItemInBag(bagNum - 1, slotNum - 1, true);
                    return {};
                }
            }
        }

        // Numeric equip slot
        {
            uint32_t numId = parseNumericSpellId(useArg);  // Reuse: strips '#', checks all digits
            if (numId && numId >= 1 && numId <= static_cast<uint32_t>(game::EquipSlot::BAG4) + 1) {
                auto eslot = static_cast<game::EquipSlot>(numId - 1);
                const auto& esl = ctx.gameHandler.getInventory().getEquipSlot(eslot);
                if (!esl.empty())
                    ctx.gameHandler.useItemById(esl.item.itemId);
                return {};
            }
        }

        // Name-based search
        std::string useArgLower = useArg;
        for (char& c : useArgLower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

        bool found = false;
        const auto& inv = ctx.gameHandler.getInventory();
        // Search backpack
        for (int s = 0; s < inv.getBackpackSize() && !found; ++s) {
            const auto& slot = inv.getBackpackSlot(s);
            if (slot.empty()) continue;
            const auto* info = ctx.gameHandler.getItemInfo(slot.item.itemId);
            if (!info) continue;
            std::string nameLow = info->name;
            for (char& c : nameLow) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (nameLow == useArgLower) {
                ctx.gameHandler.useItemBySlot(s, true);
                found = true;
            }
        }
        // Search bags
        for (int b = 0; b < game::Inventory::NUM_BAG_SLOTS && !found; ++b) {
            for (int s = 0; s < inv.getBagSize(b) && !found; ++s) {
                const auto& slot = inv.getBagSlot(b, s);
                if (slot.empty()) continue;
                const auto* info = ctx.gameHandler.getItemInfo(slot.item.itemId);
                if (!info) continue;
                std::string nameLow = info->name;
                for (char& c : nameLow) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                if (nameLow == useArgLower) {
                    ctx.gameHandler.useItemInBag(b, s, true);
                    found = true;
                }
            }
        }
        if (!found) {
            game::MessageChatData sysMsg;
            sysMsg.type = game::ChatType::SYSTEM;
            sysMsg.language = game::ChatLanguage::UNIVERSAL;
            sysMsg.message = "Item not found: '" + useArg + "'.";
            ctx.gameHandler.addLocalChatMessage(sysMsg);
        }
        return {};
    }
    std::vector<std::string> aliases() const override { return {"use"}; }
    std::string helpText() const override { return "Use an item by name, ID, or bag/slot"; }
};

// --- /equip ---
class EquipCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        if (ctx.args.empty()) return {false, false};

        std::string equipArg = ctx.args;
        trimInPlace(equipArg);
        std::string equipArgLower = equipArg;
        for (char& c : equipArgLower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

        bool found = false;
        const auto& inv = ctx.gameHandler.getInventory();
        for (int s = 0; s < inv.getBackpackSize() && !found; ++s) {
            const auto& slot = inv.getBackpackSlot(s);
            if (slot.empty()) continue;
            const auto* info = ctx.gameHandler.getItemInfo(slot.item.itemId);
            if (!info) continue;
            std::string nameLow = info->name;
            for (char& c : nameLow) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (nameLow == equipArgLower) {
                ctx.gameHandler.autoEquipItemBySlot(s);
                found = true;
            }
        }
        for (int b = 0; b < game::Inventory::NUM_BAG_SLOTS && !found; ++b) {
            for (int s = 0; s < inv.getBagSize(b) && !found; ++s) {
                const auto& slot = inv.getBagSlot(b, s);
                if (slot.empty()) continue;
                const auto* info = ctx.gameHandler.getItemInfo(slot.item.itemId);
                if (!info) continue;
                std::string nameLow = info->name;
                for (char& c : nameLow) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                if (nameLow == equipArgLower) {
                    ctx.gameHandler.autoEquipItemInBag(b, s);
                    found = true;
                }
            }
        }
        if (!found) {
            game::MessageChatData sysMsg;
            sysMsg.type = game::ChatType::SYSTEM;
            sysMsg.language = game::ChatLanguage::UNIVERSAL;
            sysMsg.message = "Item not found: '" + equipArg + "'.";
            ctx.gameHandler.addLocalChatMessage(sysMsg);
        }
        return {};
    }
    std::vector<std::string> aliases() const override { return {"equip"}; }
    std::string helpText() const override { return "Auto-equip an item from inventory"; }
};

// --- /equipset ---
class EquipSetCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        const auto& sets = ctx.gameHandler.getEquipmentSets();
        auto sysSay = [&](const std::string& msg) {
            game::MessageChatData m;
            m.type = game::ChatType::SYSTEM;
            m.language = game::ChatLanguage::UNIVERSAL;
            m.message = msg;
            ctx.gameHandler.addLocalChatMessage(m);
        };
        if (ctx.args.empty()) {
            if (sets.empty()) {
                sysSay("[System] No equipment sets saved.");
            } else {
                sysSay("[System] Equipment sets:");
                for (const auto& es : sets)
                    sysSay("  " + es.name);
            }
            return {};
        }
        std::string setName = ctx.args;
        trimInPlace(setName);
        std::string setLower = setName;
        std::transform(setLower.begin(), setLower.end(), setLower.begin(), ::tolower);
        const game::GameHandler::EquipmentSetInfo* found = nullptr;
        for (const auto& es : sets) {
            std::string nameLow = es.name;
            std::transform(nameLow.begin(), nameLow.end(), nameLow.begin(), ::tolower);
            if (nameLow == setLower || nameLow.find(setLower) == 0) {
                found = &es;
                break;
            }
        }
        if (found) {
            ctx.gameHandler.useEquipmentSet(found->setId);
        } else {
            sysSay("[System] No equipment set matching '" + setName + "'.");
        }
        return {};
    }
    std::vector<std::string> aliases() const override { return {"equipset"}; }
    std::string helpText() const override { return "Equip a saved equipment set"; }
};

// --- /startattack ---
class StartAttackCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        bool condPass = true;
        uint64_t saOverride = static_cast<uint64_t>(-1);
        if (!ctx.args.empty()) {
            std::string saArg = ctx.args;
            trimInPlace(saArg);
            if (!saArg.empty() && saArg.front() == '[') {
                std::string result = evaluateMacroConditionals(saArg, ctx.gameHandler, saOverride);
                condPass = !(result.empty() && saOverride == static_cast<uint64_t>(-1));
            }
        }
        if (condPass) {
            uint64_t atkTarget = (saOverride != static_cast<uint64_t>(-1) && saOverride != 0)
                ? saOverride : (ctx.gameHandler.hasTarget() ? ctx.gameHandler.getTargetGuid() : 0);
            if (atkTarget != 0) {
                ctx.gameHandler.startAutoAttack(atkTarget);
            } else {
                game::MessageChatData msg;
                msg.type = game::ChatType::SYSTEM;
                msg.language = game::ChatLanguage::UNIVERSAL;
                msg.message = "You have no target.";
                ctx.gameHandler.addLocalChatMessage(msg);
            }
        }
        return {};
    }
    std::vector<std::string> aliases() const override { return {"startattack"}; }
    std::string helpText() const override { return "Start auto-attack"; }
};

// --- /stopattack ---
class StopAttackCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.stopAutoAttack();
        return {};
    }
    std::vector<std::string> aliases() const override { return {"stopattack"}; }
    std::string helpText() const override { return "Stop auto-attack"; }
};

// --- /stopcasting ---
class StopCastingCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.stopCasting();
        return {};
    }
    std::vector<std::string> aliases() const override { return {"stopcasting"}; }
    std::string helpText() const override { return "Stop current cast"; }
};

// --- /cancelqueuedspell, /stopspellqueue ---
class CancelQueuedSpellCommand : public IChatCommand {
public:
    ChatCommandResult execute(ChatCommandContext& ctx) override {
        ctx.gameHandler.cancelQueuedSpell();
        return {};
    }
    std::vector<std::string> aliases() const override { return {"cancelqueuedspell", "stopspellqueue"}; }
    std::string helpText() const override { return "Cancel queued spell"; }
};

// --- Registration ---
void registerCombatCommands(ChatCommandRegistry& reg) {
    reg.registerCommand(std::make_unique<CastCommand>());
    reg.registerCommand(std::make_unique<CastSequenceCommand>());
    reg.registerCommand(std::make_unique<UseCommand>());
    reg.registerCommand(std::make_unique<EquipCommand>());
    reg.registerCommand(std::make_unique<EquipSetCommand>());
    reg.registerCommand(std::make_unique<StartAttackCommand>());
    reg.registerCommand(std::make_unique<StopAttackCommand>());
    reg.registerCommand(std::make_unique<StopCastingCommand>());
    reg.registerCommand(std::make_unique<CancelQueuedSpellCommand>());
}

} // namespace ui
} // namespace wowee
