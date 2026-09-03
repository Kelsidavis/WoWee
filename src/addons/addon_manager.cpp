#include "addons/addon_manager.hpp"
#include "addons/lua_handler_globals.hpp"
#include "addons/addon_lua_snippets.hpp"
#include "addons/lua_api_registrations.hpp"

extern "C" {
#include <lua.h>
}
#include "addons/addon_globals.hpp"
#include "ui/framexml_takeover.hpp"
#include "core/logger.hpp"
#include "core/window.hpp"
#include "core/config_paths.hpp"
#include <sstream>
#include "ui/xml_parser.hpp"
#include "ui/framexml_emitter.hpp"
#include <cctype>
#include <algorithm>
#include <set>
#include <optional>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

namespace wowee::addons {

AddonManager::AddonManager() = default;
AddonManager::~AddonManager() { shutdown(); }

bool AddonManager::initialize(game::GameHandler* gameHandler, const LuaServices& services) {
    gameHandler_ = gameHandler;
    luaServices_ = services;
    // Supplied here rather than by the caller: the manager is what owns the
    // list of load-on-demand addons and the Lua state that asks for them, and
    // wiring it anywhere else would mean handing one to the other.
    luaServices_.loadAddOn = [this](const std::string& name, std::string& reason) {
        return loadAddOnByName(name, reason);
    };
    // Saved variables on demand, for an addon that has just been given
    // something worth keeping - a window moved to where the player wants it.
    // Same reasoning as the loaders above: the manager owns both the list of
    // addons and the paths their variables are written to.
    luaServices_.saveAddOnVariables = [this]() { saveAllSavedVariables(); };
    luaServices_.isAddOnLoaded = [this](const std::string& name) {
        return isAddOnLoadedByName(name);
    };
    luaServices_.setAddOnEnabled = [this](const std::string& name, bool enabled) {
        setAddonEnabled(name, enabled);
    };
    if (!luaEngine_.initialize()) return false;
    luaEngine_.setGameHandler(gameHandler);
    luaEngine_.setLuaServices(luaServices_);
    return true;
}

namespace {

/// Find a child of `base` whose name matches `name` ignoring case, or empty.
///
/// Extracted game data does not agree with itself about case: this install has
/// interface/framexml in lower case beside interface/AddOns in mixed. The asset
/// manager copes because it goes through a manifest of normalised paths, but
/// anything reaching the filesystem directly has to look, and on a
/// case-sensitive filesystem a hardcoded spelling simply misses.
std::filesystem::path resolveChild(const std::filesystem::path& base,
                                   const std::string& name) {
    std::error_code ec;
    std::filesystem::path exact = base / name;
    if (std::filesystem::exists(exact, ec)) return exact;

    auto lower = [](std::string v) {
        for (char& c : v) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        return v;
    };
    const std::string wanted = lower(name);
    for (const auto& entry : std::filesystem::directory_iterator(base, ec)) {
        if (lower(entry.path().filename().string()) == wanted) return entry.path();
    }
    return {};
}

/// Walk a relative path one component at a time, matching each without regard
/// to case.
std::filesystem::path resolvePath(const std::filesystem::path& base,
                                  const std::string& relative) {
    std::filesystem::path at = base;
    for (const auto& part : std::filesystem::path(relative)) {
        if (part.empty() || part == ".") continue;
        at = resolveChild(at, part.string());
        if (at.empty()) return {};
    }
    return at;
}

} // namespace

void AddonManager::scanAddons(const std::string& addonsPath) {
    addonsPath_ = addonsPath;
    addons_.clear();
    lodAddons_.clear();
    lodLoaded_.clear();

    // Two places are searched. The game data's own Interface\AddOns is where a
    // player's existing addons already live, and an "addons" directory beside
    // the executable is where this client's own ship without anyone having to
    // copy files into an extracted game install to try them.
    std::vector<fs::path> roots;
    {
        // Same case problem as FrameXML: this install has interface/addons in
        // lower case, and a hardcoded AddOns misses it on a case-sensitive
        // filesystem.
        //
        // Every spelling is taken, not the first that exists. This install has
        // *both* - an empty interface/AddOns beside the interface/addons that
        // holds all twenty-four Blizzard addons - and looking only until one
        // was found stopped at the empty one. Nothing load-on-demand had ever
        // loaded here: no talent frame, no macro frame, no achievements, no
        // key bindings, and Blizzard_GMChatUI reporting itself missing on
        // every login.
        std::error_code ec;
        const fs::path asked(addonsPath);
        if (fs::is_directory(asked, ec)) roots.emplace_back(asked);

        const fs::path interfaceDir = asked.parent_path();
        if (fs::is_directory(interfaceDir, ec)) {
            for (const auto& entry : fs::directory_iterator(interfaceDir, ec)) {
                if (!entry.is_directory(ec)) continue;
                std::string name = entry.path().filename().string();
                for (char& c : name) {
                    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                }
                if (name != "addons") continue;
                if (fs::equivalent(entry.path(), asked, ec)) continue;
                roots.emplace_back(entry.path());
            }
        }
    }
    std::error_code rec;
    for (const char* local : {"addons", "../addons", "../../addons"}) {
        fs::path p = fs::absolute(local, rec);
        if (fs::is_directory(p, rec)) roots.push_back(fs::weakly_canonical(p, rec));
    }

    int scannedDirs = 0, loadOnDemand = 0, noToc = 0;
    std::vector<fs::path> dirs;
    for (const auto& root : roots) {
        std::error_code ec;
        if (!fs::is_directory(root, ec)) {
            LOG_INFO("AddonManager: no AddOns directory at ", root.string());
            continue;
        }
        // Said out loud, because which directory this lands on decides what
        // gets loaded and it is resolved differently on a case-insensitive
        // filesystem. A report of the interface appearing when nobody asked
        // for it is unanswerable without knowing where the scan looked.
        LOG_WARNING("AddonManager: scanning for addons in ", root.string());
        for (const auto& entry : fs::directory_iterator(root, ec)) {
            if (entry.is_directory()) dirs.push_back(entry.path());
        }
    }
    // Sort alphabetically for deterministic load order
    std::sort(dirs.begin(), dirs.end());

    // One addon per name, however many roots supply it. Searching more than one
    // place means the same addon can be found twice - a copy staged beside the
    // executable and the original it was staged from, say - and loading both
    // runs its Lua twice, which builds two of every frame. They sit exactly on
    // top of each other, so it reads as one frame that will not hide: the
    // toggle hides the copy it has a handle to and the other stays.
    std::set<std::string> seen;
    std::set<std::string> lodSeen;
    int duplicates = 0;

    for (const auto& dir : dirs) {
        ++scannedDirs;
        std::string dirName = dir.filename().string();
        // The original interface is not an addon and must never be loaded as
        // one. It ships with a .toc of its own, so a scan that lands on
        // Data/interface rather than Data/interface/AddOns - which is what a
        // case-insensitive filesystem can produce - would find FrameXML and
        // load the whole of it, with none of the opt-in that is supposed to
        // guard it. It has exactly one way in, and this is not it.
        {
            std::string lower = dirName;
            for (char& c : lower) {
                c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            }
            if (lower == "framexml" || lower == "gluexml") {
                LOG_WARNING("AddonManager: refusing to load ", dirName,
                            " as an addon - the original interface loads only "
                            "through WOWEE_LOAD_FRAMEXML");
                continue;
            }
        }

        std::string tocPath = (dir / (dirName + ".toc")).string();
        auto toc = parseTocFile(tocPath);
        if (!toc) { ++noToc; continue; }

        if (toc->isLoadOnDemand()) {
            ++loadOnDemand;
            // Kept rather than dropped: LoadAddOn has to be able to find these,
            // and GetAddOnInfo lists them alongside the rest. They are simply
            // not run until something asks.
            if (lodSeen.insert(toc->addonName).second) {
                lodAddons_.push_back(*toc);
            }
            continue;
        }

        if (!seen.insert(toc->addonName).second) {
            ++duplicates;
            LOG_INFO("AddonManager: '", toc->addonName, "' already found elsewhere; "
                     "ignoring the copy at ", dir.string());
            continue;
        }

        LOG_INFO("AddonManager: registered addon '", toc->getTitle(),
                 "' (", toc->files.size(), " files) from ", dir.string());
        addons_.push_back(std::move(*toc));
    }

    // Say what happened even when nothing loads, which is the case that used to
    // be silent: every Blizzard addon in a stock Interface directory is
    // LoadOnDemand, so a scan can look at dozens of folders, register none of
    // them, and print one line that reads like an empty directory.
    LOG_INFO("AddonManager: scanned ", scannedDirs, " directories, registered ",
             addons_.size(), " addons (", loadOnDemand, " load-on-demand, ",
             noToc, " without a .toc, ", duplicates, " duplicate)");
    // Load persisted enable/disable choices now that we know which addons exist.
    loadEnabledState();
}

std::vector<std::string> AddonManager::deferredAddonGlobals() const {
    std::vector<std::string> names;
    for (const TocFile& addon : lodAddons_) {
        // A disabled addon is never going to load, so its names stay absent -
        // which is the same answer, arrived at for a different reason.
        //
        // The manifest is not the whole addon. A .toc commonly lists only the
        // XML and lets `<Script file="...">` inside it pull the Lua in - the
        // calendar's lists Blizzard_Calendar.xml and Localization.lua and not
        // Blizzard_Calendar.lua, where all 248 of its functions live. Reading
        // only what the manifest names left every one of those absent from
        // this list, so the stand-in answered for them: a truthy table.
        //
        // That is the shape this list exists to prevent. FrameXML guards a
        // load-on-demand addon with `if ( Calendar_Toggle ) then
        // Calendar_Toggle() end`, and a truthy table walks past the guard and
        // raises on the call - so opening the calendar from the minimap
        // errored rather than doing nothing. Twenty-four addons include their
        // Lua this way.
        std::vector<std::string> queue(addon.files.begin(), addon.files.end());
        std::set<std::string> seen;
        for (size_t qi = 0; qi < queue.size(); ++qi) {
            const std::string& rel = queue[qi];
            if (!seen.insert(rel).second) continue;
            const fs::path file = resolvePath(fs::path(addon.basePath), rel);
            if (file.empty()) continue;
            std::ifstream in(file, std::ios::binary);
            if (!in) continue;
            const std::string text((std::istreambuf_iterator<char>(in)),
                                   std::istreambuf_iterator<char>());
            const std::string ext = file.extension().string();
            if (ext == ".lua" || ext == ".Lua" || ext == ".LUA") {
                collectLuaGlobals(text, names);
                continue;
            }
            collectXmlNames(text, names);
            // And whatever this XML pulls in, appended so an include that
            // includes further files is walked as well. Bounded by `seen`,
            // because two files in one addon naming each other would
            // otherwise not stop.
            for (size_t at = text.find("<Script"); at != std::string::npos;
                 at = text.find("<Script", at + 1)) {
                const size_t fileAt = text.find("file=\"", at);
                if (fileAt == std::string::npos) continue;
                const size_t close = text.find('>', at);
                if (close != std::string::npos && fileAt > close) continue;
                const size_t start = fileAt + 6;
                const size_t end = text.find('"', start);
                if (end == std::string::npos) break;
                std::string inc = text.substr(start, end - start);
                std::replace(inc.begin(), inc.end(), '\\', '/');
                if (!inc.empty()) queue.push_back(inc);
            }
        }
    }
    // Every addon's saved variables, whether or not it loads on demand.
    //
    // A saved variable is nil in WoW until the saved file restores it or the
    // addon creates it, and addons test exactly that:
    //
    //     if ( not BlizzardStopwatchOptions ) then
    //         BlizzardStopwatchOptions = {}
    //     end
    //
    // Answered by the stand-in, that name is a truthy table, so the guard
    // never runs and the variable is never made. What follows is worse than a
    // missing setting: the addon then writes its fields into the stand-in -
    // and the stand-in is *one shared table* that every unknown global
    // resolves to, so a position saved by the stopwatch would appear on every
    // other unbound name in the interface.
    //
    // These come from all addons rather than only the deferred ones, and
    // declaring them costs nothing when the variable does exist: the fallback
    // is only consulted for a name nothing has set, so a restored variable is
    // a real global and never reaches it.
    // Both lists: an addon is in one or the other, never both, and the
    // stopwatch - the case this was written for - is a load-on-demand one.
    for (const auto* list : {&addons_, &lodAddons_}) {
        for (const TocFile& addon : *list) {
            for (const std::string& v : addon.getSavedVariables()) {
                if (!v.empty()) names.push_back(v);
            }
            for (const std::string& v : addon.getSavedVariablesPerCharacter()) {
                if (!v.empty()) names.push_back(v);
            }
        }
    }

    std::sort(names.begin(), names.end());
    names.erase(std::unique(names.begin(), names.end()), names.end());
    return names;
}

void AddonManager::loadAllAddons() {
    // The original interface, when asked for. Before any addon, because addons
    // are written against a world where FrameXML has already defined its frames
    // and its several thousand functions.
    //
    // On by default on this branch, which exists to bring it up: the elements
    // it has taken over are the ones being worked on, and needing a flag to
    // see them means every test run starts by remembering the flag.
    // WOWEE_LOAD_FRAMEXML=0 turns it off; master leaves it off unless asked.
    // Before any of it runs: what the addons that have *not* loaded will define
    // once they do. FrameXML asks for those names to decide whether to load the
    // panel behind them, and the missing-API fallback answering with a truthy
    // no-op turns every one of those tests into "already loaded" - the panel is
    // never asked for and the branch runs against a stand-in.
    luaEngine_.declareDeferredGlobals(deferredAddonGlobals());

    const char* wantFrameXml = std::getenv("WOWEE_LOAD_FRAMEXML");
    const bool loadIt = wantFrameXml ? (std::string(wantFrameXml) != "0") : true;
    if (loadIt && !frameXmlDir_.empty()) {
        // Silent while the interface builds itself, the way the real client is
        // silent behind its loading screen.
        //
        // UIDropDownMenu_Initialize calls its initialize function straight away
        // - stock behaviour - and every unit frame's initializer ends in
        // UnitPopup_ShowMenu, which finishes with PlaySound("igMainMenuOpen").
        // The player frame, four party frames and three target frames all
        // initialize as FrameXML loads, so seven copies of
        // uEscapeScreenOpen.wav were played inside thirty milliseconds and
        // stacked into one loud hit. Populating a menu is not opening one.
        //
        // A Lua hook cannot reach this: the frames initialize during the load
        // below, before any script of ours could run. The flag is cleared in
        // every exit from here, so a load that throws cannot leave the client
        // mute.
        // The screen the interface lays out against, before it lays out.
        //
        // FrameXML measures its own frames as it builds them, and a rect read
        // before the first full pass has no screen to be solved against and
        // answers zero. The window's size is known long before this, so the
        // tree is told it rather than left to answer nothing - see
        // noteScreenSize for the chat window's scroll buttons, which is where
        // it shows.
        if (luaServices_.window) {
            luaEngine_.widgets().noteScreenSize(
                static_cast<float>(luaServices_.window->getWidth()),
                static_cast<float>(luaServices_.window->getHeight()));
        }
        luaEngine_.setUiSoundsSuppressed(true);
        loadFrameXml(frameXmlDir_);
        luaEngine_.setUiSoundsSuppressed(false);
        // The client's own options, as a category in FrameXML's Interface
        // Options. After FrameXML rather than in the bootstrap, because
        // InterfaceOptions_AddCategory is FrameXML's - the bootstrap's stub for
        // it is overwritten when the real one loads, and registering against the
        // stub would put the panel nowhere.
        registerWoweeOptionsPanel();
        giveCoinAmountsClearance();
        // Populating a dropdown is not opening one. Catches the initializers
        // driven by the world-entry packet, which arrives long after this.
        if (!luaEngine_.executeString(kDropdownInitSilenceLua)) {
            LOG_WARNING("Dropdown init silence did not apply: ",
                        luaEngine_.lastError());
        }
        // A scale you cannot read is applied before you can judge it, and the
        // way out is the panel you just made unreadable.
        // Shift pressed over a tooltip that is already up. FrameXML asks
        // about the modifier once, while the tooltip is being built, and the
        // real client re-fires it in C when the key changes.
        if (!luaEngine_.executeString(kTooltipModifierRefreshLua)) {
            LOG_WARNING("Tooltip modifier refresh did not install: ",
                        luaEngine_.lastError());
        }
        // Stats that arrive after the tooltip did.
        if (!luaEngine_.executeString(kTooltipItemDataRefreshLua)) {
            LOG_WARNING("Tooltip item-data refresh did not install: ",
                        luaEngine_.lastError());
        }
        if (!luaEngine_.executeString(kOptionRangeFixesLua)) {
            LOG_WARNING("Option range fixes did not apply: ", luaEngine_.lastError());
        }
        if (!luaEngine_.executeString(kUiScaleConfirmLua)) {
            LOG_WARNING("UI scale confirmation did not apply: ",
                        luaEngine_.lastError());
        }
        // Said once, after the interface is up: anything neither handed over
        // nor hidden is about to be on screen twice.
        ui::frameXmlReportUnaccountedElements();
    }

    // Only hand the Lua VM the addons that are actually enabled, so disabled ones
    // don't appear via GetNumAddOns/IsAddOnLoaded either.
    std::vector<TocFile> enabled;
    enabled.reserve(addons_.size() + lodAddons_.size());
    for (const auto& addon : addons_) {
        if (isAddonEnabled(addon.addonName)) enabled.push_back(addon);
    }
    // Load-on-demand addons are listed too, as WoW lists them: an addon
    // manager panel shows the talent tree and the auction house alongside
    // everything else, and GetNumAddOns counts them. They carry a flag so
    // IsAddOnLoaded does not mistake being listed for being loaded.
    for (const auto& addon : lodAddons_) {
        if (isAddonEnabled(addon.addonName)) enabled.push_back(addon);
    }
    luaEngine_.setAddonList(enabled);
    int loaded = 0, failed = 0, skipped = 0;
    for (const auto& addon : addons_) {
        if (!isAddonEnabled(addon.addonName)) {
            LOG_INFO("AddonManager: skipping disabled addon: ", addon.addonName);
            skipped++;
            continue;
        }
        if (loadAddon(addon)) loaded++;
        else failed++;
    }
    addonsLoaded_ = true;
    LOG_INFO("AddonManager: loaded ", loaded, " addons",
             (failed > 0 ? (", " + std::to_string(failed) + " failed") : ""),
             (skipped > 0 ? (", " + std::to_string(skipped) + " disabled") : ""));
}

// ---- Per-addon enable/disable (persisted) ----------------------------------

bool AddonManager::isAddonEnabled(const std::string& addonName) const {
    auto it = addonEnabled_.find(addonName);
    return (it == addonEnabled_.end()) ? true : it->second;  // default: enabled
}

void AddonManager::setAddonEnabled(const std::string& addonName, bool enabled) {
    addonEnabled_[addonName] = enabled;
    saveEnabledState();
}

std::string AddonManager::enabledStatePath() {
    return core::getConfigRoot() + "/addons.cfg";
}

void AddonManager::loadEnabledState() {
    std::ifstream in(enabledStatePath());
    if (!in) return;
    std::string line;
    while (std::getline(in, line)) {
        auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string name = line.substr(0, eq);
        std::string val = line.substr(eq + 1);
        if (!name.empty()) addonEnabled_[name] = (val == "1");
    }
}

void AddonManager::saveEnabledState() const {
    std::ofstream out(enabledStatePath(), std::ios::trunc);
    if (!out) {
        LOG_WARNING("AddonManager: could not write ", enabledStatePath());
        return;
    }
    // Persist an explicit line only for addons we actually know about, so stale
    // entries for removed addons don't accumulate.
    for (const auto& addon : addons_) {
        out << addon.addonName << "=" << (isAddonEnabled(addon.addonName) ? "1" : "0") << "\n";
    }
}

std::string AddonManager::getSavedVariablesPath(const TocFile& addon) const {
    return addon.basePath + "/" + addon.addonName + ".lua.saved";
}

std::string AddonManager::getSavedVariablesPerCharacterPath(const TocFile& addon) const {
    if (characterName_.empty()) return "";
    return addon.basePath + "/" + addon.addonName + "." + characterName_ + ".lua.saved";
}

// The client's settings, as panels in FrameXML's Interface Options.
//
// Built from WoweeSettingList() rather than written out here, so a setting
// added to the schema appears without anyone editing Lua. Only the settings
// with no Blizzard control of their own are in that list - the six that are
// bound to the CVar their own Blizzard control already drives are named on the
// root panel instead, so a player looking for one is told where it is.
//
// The layout is the game's own: a root category with a child per subject, each
// laid out in sections, in two columns, with the game's fonts and its check
// buttons, sliders and dropdowns. It is not Blizzard's Interface Options panel
// reproduced - this client has settings that one never had - but it reads the
// same way round.
void AddonManager::registerWoweeOptionsPanel() {
    static const char* kPanelScript = kWoweeOptionsPanelLua;
    if (!luaEngine_.executeString(kPanelScript)) {
        LOG_WARNING("Wowee options panel did not register: ", luaEngine_.lastError());
    }
}

void AddonManager::giveCoinAmountsClearance() {
    // Move the coin amounts off the coins.
    //
    // MoneyFrame_Update anchors each amount so its right edge is exactly the
    // left edge of that denomination's coin - SetPoint("RIGHT", -iconWidth) -
    // which leaves no room at all. On screen the numbers are drawn over the
    // coins; in every measurement that can be taken here they are not, and the
    // rects are right down to the tenth of a unit: the gold amount ends at
    // 723.4 and its coin starts at 723.4, the next button begins four units
    // after the previous one ends.
    //
    // So this is clearance rather than a diagnosis, and it is worth saying so:
    // a few units of room costs nothing and fixes what is actually on screen,
    // where knowing exactly why has cost a great deal and has not.
    //
    // Hooked rather than edited into MoneyFrame.lua, so the interface's own
    // file stays Blizzard's and this stays visible as ours.
    static const char* kScript = kCoinAmountClearanceLua;
    if (!luaEngine_.executeString(kScript)) {
        LOG_WARNING("Coin amount clearance did not apply: ", luaEngine_.lastError());
    }
    // Chat lines that do not fade out from under the player.
    if (!luaEngine_.executeString(kChatNoFadeLua)) {
        LOG_WARNING("Chat fade removal did not apply: ", luaEngine_.lastError());
    }
    // The input box, in the chat window's own colours.
    if (!luaEngine_.executeString(kChatInputBackgroundLua)) {
        LOG_WARNING("Chat input background did not apply: ", luaEngine_.lastError());
    }
    // And a wheel on the row that opens the colour picker.
    if (!luaEngine_.executeString(kChatBackgroundSwatchLua)) {
        LOG_WARNING("Chat background swatch did not apply: ", luaEngine_.lastError());
    }
    // A loot roll window, filled in again when its item is answered for.
    if (!luaEngine_.executeString(kLootRollRefreshLua)) {
        LOG_WARNING("Loot roll refresh did not apply: ", luaEngine_.lastError());
    }
    // What the player last set, applied.
    //
    // The CVar store is filled from disk before any renderer, camera or audio
    // manager exists, so those values were remembered and never acted on: the
    // panels read them back and showed them correctly while the client ran on
    // its defaults. This is the first moment they can all be reached.
    if (lua_State* L = luaEngine_.getState()) {
        applyStoredCVarSideEffects(L);
    }

    // Five globals the interface reads and nothing ever created. Before the
    // greying below, so a control that turns out to work is not greyed on the
    // strength of a setting that was only ever nil.
    if (!luaEngine_.executeString(kMissingUVarsLua)) {
        LOG_WARNING("Missing uvars did not apply: ", luaEngine_.lastError());
    }

    // Who wrote this client, in the menu that opens with Escape.
    if (!luaEngine_.executeString(kAboutMenuLua)) {
        LOG_WARNING("About menu entry did not apply: ", luaEngine_.lastError());
    }

    // The compass N, which sits on top of this client's zone name.
    if (!luaEngine_.executeString(kMinimapNorthTagLua)) {
        LOG_WARNING("Minimap north tag did not apply: ", luaEngine_.lastError());
    }

    // Every control for something this client does not do, taken off the panel.
    if (!luaEngine_.executeString(kRemovedControlsLua)) {
        LOG_WARNING("Removed controls did not apply: ", luaEngine_.lastError());
    }
    // ...and the names it could not find, which is how a list of frame names
    // goes stale without anybody noticing. Warning level: an entry naming
    // nothing is a bug in the list, not a state of the interface.
    if (!luaEngine_.executeString(
            "if __WoweeRemovedControlsMissing and\n"
            "   #__WoweeRemovedControlsMissing > 0 then\n"
            "    WoweeReportMissingFixedControls(\n"
            "        table.concat(__WoweeRemovedControlsMissing, ' '))\n"
            "end\n")) {
        LOG_WARNING("Removed control report did not run: ", luaEngine_.lastError());
    }
}

bool AddonManager::loadFrameXml(const std::string& frameXmlDir) {
    std::error_code ec;
    std::filesystem::path dir(frameXmlDir);
    if (!std::filesystem::is_directory(dir, ec)) {
        // The directory itself may be spelled differently on disk.
        dir = resolvePath(std::filesystem::path(frameXmlDir).parent_path(),
                          std::filesystem::path(frameXmlDir).filename().string());
    }
    if (dir.empty() || !std::filesystem::is_directory(dir, ec)) {
        LOG_WARNING("FrameXML: no directory at ", frameXmlDir);
        return false;
    }
    const std::filesystem::path tocPath = resolveChild(dir, "FrameXML.toc");
    auto toc = tocPath.empty() ? std::nullopt : parseTocFile(tocPath.string());
    if (!toc) {
        LOG_WARNING("FrameXML: no manifest in ", dir.string());
        return false;
    }
    // Which convention this interface's handlers are written against, decided
    // before the first of them runs - an OnLoad fires during the load below,
    // so deciding afterwards would be deciding it too late.
    //
    // `## Interface: 11200` is 1.12.0, `20400` is 2.4.3, `30300` is 3.3.5a.
    // Everything before 3.0 takes no handler arguments at all and reads the
    // frame off the global `this`, the event off `event` and the payload off
    // `arg1`..`argN`; 3.0 replaced that with the arguments this client already
    // passes. See LuaEngine::setLegacyHandlerGlobals.
    //
    // Read off the manifest rather than off the active expansion profile,
    // because it is a fact about the files about to be loaded and the two can
    // disagree: a Turtle install is 1.12's interface whichever profile is
    // selected.
    //
    // A manifest that does not say gets the older convention. The two ways of
    // being wrong are not the same size - publishing three globals for an
    // interface that ignores them costs a few table writes per handler, while
    // withholding them from one that needs them means every OnLoad in the file
    // raises on its first line, so nothing registers for an event, nothing is
    // positioned and nothing hides itself.
    int interfaceVersion = 0;
    if (const auto it = toc->directives.find("Interface");
        it != toc->directives.end()) {
        interfaceVersion = std::atoi(it->second.c_str());
    }
    const bool legacyHandlers = interfaceVersion == 0 || interfaceVersion < 30000;
    setLegacyHandlerGlobals(legacyHandlers);
    // Published, because some of FrameXML's own signatures changed between
    // expansions and the snippets this client runs against them have to pick.
    // UIDropDownMenu_SetWidth is the one that bit: 1.12 takes (width, frame)
    // and 2.0 onward takes (frame, width), so calling it the later way on a
    // 1.12 interface hands the frame in as a width and the number in as the
    // frame, and the function indexes the number.
    //
    // Zero for an interface that states no version, which is the same thing
    // legacyHandlers treats as old.
    luaEngine_.executeString("__WoweeInterfaceVersion = " +
                             std::to_string(interfaceVersion));
    LOG_WARNING("FrameXML: interface ",
                interfaceVersion == 0 ? std::string("unstated")
                                      : std::to_string(interfaceVersion),
                " - handlers are given ",
                legacyHandlers ? "this, event and arg1..argN as globals"
                               : "their arguments only");

    const std::string resolvedDir = dir.string();
    // Kept, because an include that names a shared template by bare name is
    // resolved against this and nothing else. Blizzard_InspectUI asks for
    // PVPFrameTemplates.xml, which lives in FrameXML rather than beside it, and
    // with an unopenable base the fallback below silently found nothing - the
    // include failed, the file failed, and the whole addon failed to load, so
    // inspecting another player did nothing at all.
    frameXmlResolvedDir_ = resolvedDir;

    LOG_WARNING("FrameXML: attempting to load the original interface - ",
                toc->files.size(), " files from ", resolvedDir);

    // Bindings are not in the manifest - the real client loads them itself, and
    // before the interface, so that a script asking what a command is bound to
    // during load gets an answer. Without this the file was never read at all
    // and the key bindings list had nothing to list.
    if (const auto bindings = resolveChild(dir, "Bindings.xml"); !bindings.empty()) {
        if (!loadXmlFile(bindings.string(), 0)) {
            LOG_WARNING("FrameXML: could not read the key bindings: ", lastXmlError_);
        }
    }

    int lua = 0, xml = 0, failed = 0;
    // Kept and printed together at the end. Spread through the log these are
    // unreadable: the reasons land among thousands of other lines, and one
    // broken script takes down every file that references it, so what matters
    // is seeing them side by side and spotting the cause they share.
    std::vector<std::pair<std::string, std::string>> failures;
    // Timed per file. This load runs on the main thread during world entry, so
    // whatever it costs the client is frozen for - long enough and the server
    // drops the connection for want of a heartbeat. Knowing it is slow is not
    // useful; knowing which file is.
    const auto loadStart = std::chrono::steady_clock::now();
    // Generous: all 139 files together used to run in 216ms, so a single one
    // reaching this has stopped making progress. Aborting it costs that file
    // and keeps the client answering, which beats freezing until it is killed.
    luaEngine_.setChunkTimeoutMs(5000);
    struct BudgetReset {
        LuaEngine& e;
        ~BudgetReset() { e.setChunkTimeoutMs(0); }
    } budgetReset{luaEngine_};
    auto sinceMs = [](std::chrono::steady_clock::time_point from) {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - from).count();
    };
    for (const auto& filename : toc->files) {
        const auto fileStart = std::chrono::steady_clock::now();
        // Named before it is loaded, not after. Timing it afterwards says
        // nothing about the one case that matters - a file that never returns
        // prints nothing at all, and the load simply stops with the last
        // successful file as the only clue.
        // At warning level because release builds drop INFO, and this is the
        // one line that identifies a file which never returns. Noisy for 139
        // files, and worth it only while this path is still experimental.
        LOG_WARNING("FrameXML: loading ", filename);
        std::string lower = filename;
        for (char& c : lower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        const std::filesystem::path resolved = resolvePath(dir, filename);
        if (resolved.empty()) {
            LOG_WARNING("FrameXML: ", filename, " is listed but not on disk");
            ++failed;
            failures.emplace_back(filename, "listed in the manifest but not on disk");
            continue;
        }
        const std::string full = resolved.string();

        // The manifest's order is the load order and matters: GlobalStrings and
        // Constants before anything reads them, Fonts before the frames that
        // inherit from them. Following it is most of what makes this possible
        // at all.
        if (lower.size() >= 4 && lower.compare(lower.size() - 4, 4, ".lua") == 0) {
            if (luaEngine_.executeFile(full)) {
                ++lua;
            } else {
                ++failed;
                failures.emplace_back(filename, luaEngine_.lastError());
            }
        } else if (lower.size() >= 4 && lower.compare(lower.size() - 4, 4, ".xml") == 0) {
            lastXmlError_.clear();
            if (loadXmlFile(full, 0)) {
                ++xml;
            } else {
                ++failed;
                failures.emplace_back(filename, lastXmlError_.empty()
                                                    ? "(no reason recorded)"
                                                    : lastXmlError_);
            }
        }
        // Reported as it happens rather than only in the summary, because a
        // load that never reaches the summary is exactly the case worth
        // diagnosing.
        if (const auto ms = sinceMs(fileStart); ms >= 250) {
            LOG_WARNING("FrameXML: ", filename, " took ", ms, "ms");
        }
    }
    // Screen insets the panel manager reads straight off UIParent. The real
    // client supplies these; FrameXML only ever reads them, and
    // UIParentManageFramePositions adds them to a coordinate on the next line,
    // so absent they are arithmetic on nil the first time a panel opens.
    //
    // All six of them, because setting one wakes the panel manager and it then
    // reads the rest: seeding only the two offsets moved the failure from
    // LEFT_OFFSET to DEFAULT_FRAME_WIDTH one call deeper. The widths are
    // Blizzard's own defaults for a standard panel.
    luaEngine_.executeString(
        std::string("__WoweeOwnsGameMenu = ") +
        (ui::frameXmlOwns(ui::UiElement::GameMenu) ? "true" : "false") + "\n");

    // The colour of every kind of chat message, which nothing had ever sent.
    //
    // chatframe.lua builds ChatTypeInfo with sticky and flashTab and no colour
    // at all: r, g and b arrive from the client, one UPDATE_CHAT_COLOR per
    // type, and the real client sends them as the interface comes up. This one
    // never did, so every entry answered nil and every line of chat in the game
    // drew white - guild, whisper, emote, loot and system alike, all of it
    // indistinguishable.
    //
    // Fired straight into the interface rather than through ChangeChatColor.
    // That binding asks the game handler to fire the event, and the application
    // drops every event it is handed until addonsLoaded_ is true - which
    // happens at world entry, long after this runs. So all twenty-six went
    // nowhere and the table kept the white that chatframe.lua writes over it at
    // file scope, and every line of chat in the game drew the same colour.
    //
    // Sent after FrameXML has loaded, since ChatFrame_OnLoad is what registers
    // for the event. Same engine either way: the handler is what fills
    // ChatTypeInfo and repaints the lines already on screen.
    //
    // 3.3.5's own palette. Anything not named here keeps white, which is what
    // it had before, so a type left out is no worse than it was.
    //
    // Three decimals, because an event argument is only read back as a number
    // when it is short: a full float arrives as a string and would be assigned
    // straight into a colour field.
    {
        struct ChatColour { const char* type; float r, g, b; };
        static constexpr ChatColour kChatColours[] = {
            {"SYSTEM", 1.0f, 1.0f, 0.0f},    {"SAY", 1.0f, 1.0f, 1.0f},
            {"YELL", 1.0f, 0.25f, 0.25f},    {"GUILD", 0.25f, 1.0f, 0.25f},
            {"OFFICER", 0.25f, 0.75f, 0.25f},
            {"GUILD_ACHIEVEMENT", 0.25f, 1.0f, 0.25f},
            {"ACHIEVEMENT", 1.0f, 1.0f, 0.0f},
            {"PARTY", 0.67f, 0.67f, 1.0f},   {"PARTY_LEADER", 0.46f, 0.78f, 1.0f},
            {"RAID", 1.0f, 0.5f, 0.0f},      {"RAID_LEADER", 1.0f, 0.28f, 0.04f},
            {"RAID_WARNING", 1.0f, 0.28f, 0.0f},
            {"WHISPER", 1.0f, 0.5f, 1.0f},   {"WHISPER_INFORM", 1.0f, 0.5f, 1.0f},
            {"EMOTE", 1.0f, 0.5f, 0.25f},    {"TEXT_EMOTE", 1.0f, 0.5f, 0.25f},
            {"MONSTER_SAY", 1.0f, 1.0f, 1.0f},
            {"MONSTER_YELL", 1.0f, 0.25f, 0.25f},
            {"MONSTER_EMOTE", 1.0f, 0.5f, 0.25f},
            {"MONSTER_WHISPER", 1.0f, 0.72f, 0.72f},
            {"CHANNEL", 1.0f, 0.75f, 0.75f}, {"LOOT", 0.0f, 0.67f, 0.0f},
            {"MONEY", 1.0f, 1.0f, 0.0f},
            {"BG_SYSTEM_NEUTRAL", 1.0f, 1.0f, 0.5f},
            {"BG_SYSTEM_ALLIANCE", 0.25f, 0.75f, 1.0f},
            {"BG_SYSTEM_HORDE", 1.0f, 0.1f, 0.1f},
        };
        char r[16], g[16], b[16];
        for (const auto& c : kChatColours) {
            std::snprintf(r, sizeof(r), "%.3f", c.r);
            std::snprintf(g, sizeof(g), "%.3f", c.g);
            std::snprintf(b, sizeof(b), "%.3f", c.b);
            luaEngine_.fireEvent("UPDATE_CHAT_COLOR", {c.type, r, g, b});
        }
    }

    // Whether that landed. chatframe.lua sets every ChatTypeInfo entry to white
    // at file scope and the colours only arrive from here, so if this says
    // GUILD is still 1,1,1 the seeding did not reach the table - and every line
    // of chat draws the same colour with nothing else to show for it.
    {
        lua_State* L = luaEngine_.getState();
        if (L) {
            lua_getglobal(L, "ChatTypeInfo");
            if (lua_istable(L, -1)) {
                lua_getfield(L, -1, "GUILD");
                if (lua_istable(L, -1)) {
                    lua_getfield(L, -1, "r"); const double r = lua_tonumber(L, -1); lua_pop(L, 1);
                    lua_getfield(L, -1, "g"); const double g = lua_tonumber(L, -1); lua_pop(L, 1);
                    lua_getfield(L, -1, "b"); const double b = lua_tonumber(L, -1); lua_pop(L, 1);
                    // Only when it did not take. chatframe.lua writes white
                    // over every entry at file scope and the colours arrive
                    // only from here, so a GUILD still at white means the
                    // seeding never reached the table - and every line of chat
                    // draws the same colour with nothing else to show for it.
                    if (r > 0.9 && g > 0.9 && b > 0.9) {
                        LOG_WARNING("Chat colours did not take: GUILD is still ",
                                    r, ",", g, ",", b, " - every kind of chat "
                                    "message will draw the same colour");
                    }
                } else {
                    LOG_WARNING("Chat colours: ChatTypeInfo has no GUILD entry");
                }
                lua_pop(L, 1);
            } else {
                LOG_WARNING("Chat colours: ChatTypeInfo is not a table");
            }
            lua_pop(L, 1);
        }
    }

    // The three name tables the calendar reads its labels from.
    //
    // Blizzard_Calendar's own Localization.lua defines these in the real
    // client; the copy here has the file but not the definitions, so the month
    // title was blank and every weekday header with it. Built from the
    // globalstrings the interface already carries rather than written out in
    // English, so a localised data set names its own months.
    //
    // Before the addon loads, not after, so its Localization.lua still wins if
    // a data set does define them. Sunday first, which is the order
    // CALENDAR_WEEKDAY_NAMES is indexed in and the base CalendarGetDate
    // already answers in.
    luaEngine_.executeString(
        "if not CALENDAR_MONTH_NAMES then\n"
        "  local m = {'JANUARY','FEBRUARY','MARCH','APRIL','MAY','JUNE',\n"
        "             'JULY','AUGUST','SEPTEMBER','OCTOBER','NOVEMBER','DECEMBER'}\n"
        "  local w = {'SUNDAY','MONDAY','TUESDAY','WEDNESDAY','THURSDAY',\n"
        "             'FRIDAY','SATURDAY'}\n"
        "  CALENDAR_MONTH_NAMES = {}\n"
        "  CALENDAR_FULLDATE_MONTH_NAMES = {}\n"
        "  CALENDAR_WEEKDAY_NAMES = {}\n"
        "  for i, name in ipairs(m) do\n"
        "    CALENDAR_MONTH_NAMES[i] = _G['MONTH_'..name] or name\n"
        "    CALENDAR_FULLDATE_MONTH_NAMES[i] =\n"
        "      _G['FULLDATE_MONTH_'..name] or CALENDAR_MONTH_NAMES[i]\n"
        "  end\n"
        "  for i, name in ipairs(w) do\n"
        "    CALENDAR_WEEKDAY_NAMES[i] = _G['WEEKDAY_'..name] or name\n"
        "  end\n"
        "end\n");

    // The calendar's three option lists, which are spreads rather than tables.
    //
    // Each is handed straight into a vararg call -
    // CalendarCreateEventTypeDropDown_InitEventTypes(self,
    // CalendarEventGetTypes()) - so the *number* of values is the payload and
    // answering nothing leaves the dropdown with no entries at all rather than
    // raising. There is no way to pick an event type from an empty list.
    //
    // In Lua because the strings are the interface's own: reading
    // CALENDAR_TYPE_RAID and friends out of globalstrings keeps a localised
    // data set naming its own types, where a list written in C++ would be
    // English for everyone.
    //
    // The order is the wire's, which is also constants.lua's: Raid, Dungeon,
    // PvP, Meeting, Other, indexed from one by the interface and from zero on
    // the wire.
    luaEngine_.executeString(
        "function CalendarEventGetTypes()\n"
        "  return CALENDAR_TYPE_RAID, CALENDAR_TYPE_DUNGEON, CALENDAR_TYPE_PVP,\n"
        "         CALENDAR_TYPE_MEETING, CALENDAR_TYPE_OTHER\n"
        "end\n"
        "function CalendarEventGetRepeatOptions()\n"
        "  return CALENDAR_REPEAT_NEVER, CALENDAR_REPEAT_WEEKLY,\n"
        "         CALENDAR_REPEAT_BIWEEKLY\n"
        "end\n"
        // The nine the interface knows, in CALENDAR_INVITESTATUS_* order.
        // Not the ten the server has: it also carries REMOVED, which is an
        // instruction rather than a state and has no row to show.
        "function CalendarEventGetStatusOptions()\n"
        "  return CALENDAR_STATUS_INVITED, CALENDAR_STATUS_ACCEPTED,\n"
        "         CALENDAR_STATUS_DECLINED, CALENDAR_STATUS_CONFIRMED,\n"
        "         CALENDAR_STATUS_OUT, CALENDAR_STATUS_STANDBY,\n"
        "         CALENDAR_STATUS_SIGNEDUP, CALENDAR_STATUS_NOT_SIGNEDUP,\n"
        "         CALENDAR_STATUS_TENTATIVE\n"
        "end\n");

    // Report the quest dialog's own geometry, at the moment it is built.
    //
    // Reported: the dialog is placed correctly and its contents are not - the
    // panel at x 0..384 with its reward items eight hundred units to the
    // right. The readiness dump covers these frames now, but it runs early and
    // once, so it only ever catches them hidden and unanchored, which is their
    // resting state and says nothing.
    //
    // QuestFrameDetailPanel_OnShow is where it is decided: it runs
    // QuestInfo_Display twice, once into QuestDetailScrollChildFrame and once
    // into QuestInfoFadingFrame, and the second parents the rewards to a frame
    // the first is supposed to have anchored. If that anchoring did not
    // happen, everything under it lands wherever the unanchored frame sits -
    // which is the middle of the screen, and is what the numbers look like.
    //
    // Wrapped rather than hooked so this runs *after* the real work, and only
    // ever reads.
    luaEngine_.executeString(
        // type() rather than truthiness: the missing-API stand-in is a
        // callable table, so `if QuestFrameDetailPanel_OnShow then` passes for
        // a name that does not exist and wraps the stand-in - which is exactly
        // what happened on the first attempt at this, against a
        // QuestFrame_ShowQuestDetail that FrameXML does not have. The wrapper
        // then reported the geometry of a dialog nothing had built and it
        // looked like the bug.
        "if QuestFrameDetailPanel and QuestFrameDetailPanel.HookScript then\n"
        "  QuestFrameDetailPanel:HookScript('OnShow', function()\n"
        "    local function where(f, n)\n"
        "      if not f then return n .. '=absent' end\n"
        "      local p = f.GetParent and f:GetParent()\n"
        "      return string.format('%s=(%.0f,%.0f %.0fx%.0f) pts=%d parent=%s',\n"
        "        n, f:GetLeft() or -1, f:GetBottom() or -1,\n"
        "        f:GetWidth() or 0, f:GetHeight() or 0,\n"
        "        f:GetNumPoints() or 0,\n"
        "        (p and p.GetName and p:GetName()) or '?')\n"
        "    end\n"
        "    -- The strings as well as the frames. Reported as a dialog with\n"
        "    -- no content, and the frames all came back correct: the one\n"
        "    -- thing not being said was whether there was any text in it.\n"
        "    local function says(f, n)\n"
        "      if not f then return n .. '=absent' end\n"
        "      local t = f.GetText and f:GetText()\n"
        "      return string.format('%s=%d chars (%.0f,%.0f %.0fx%.0f) vis=%s',\n"
        "        n, t and #t or 0, f:GetLeft() or -1, f:GetBottom() or -1,\n"
        "        f:GetWidth() or 0, f:GetHeight() or 0,\n"
        "        tostring(f:IsVisible()))\n"
        "    end\n"
        "    __WoweeWarn('quest detail built: ' ..\n"
        "      where(QuestFrame, 'QuestFrame') .. ' | ' ..\n"
        "      where(QuestDetailScrollChildFrame, 'ScrollChild') .. ' | ' ..\n"
        "      says(QuestInfoTitleHeader, 'Title') .. ' | ' ..\n"
        "      says(QuestInfoDescriptionText, 'Description') .. ' | ' ..\n"
        "      says(QuestInfoObjectivesText, 'Objectives') .. ' | ' ..\n"
        "      where(QuestInfoRewardsFrame, 'Rewards') .. ' | ' ..\n"
        "      where(QuestInfoItem1, 'Item1'))\n"
        "  end)\n"
        "end\n");

    // Why the objectives tracker collapsed, when it collapses itself.
    //
    // Reported as "collapse the objectives and the text does not come back".
    // FrameXML collapses that frame from two places and they mean opposite
    // things: the button's OnClick, which is the player asking, and
    // WatchFrame_Update, which does it when the objective handlers report that
    // nothing was laid out - pixelsUsed of zero leaves totalOffset at its
    // starting value, and that branch also *disables* the expand button. Since
    // WatchFrame_Expand ends by calling WatchFrame_Update, an expand whose
    // update measures nothing is undone in the same breath, which is exactly
    // "it will not come back".
    //
    // userCollapsed is the discriminator and it is already there: the button's
    // OnClick sets it, and the automatic path never does. One line, only when
    // the tracker actually collapses, so it costs nothing until it happens.
    //
    // Wrapping the global rather than hooking a script, because
    // WatchFrame_Collapse is called by name from FrameXML itself - this is the
    // opposite case to the quest panels above, where what runs is the widget's
    // stored script and only HookScript reaches it.
    luaEngine_.executeString(
        "if type(WatchFrame_Collapse) == 'function' then\n"
        "  local original = WatchFrame_Collapse\n"
        "  WatchFrame_Collapse = function(self)\n"
        "    self = self or WatchFrame\n"
        "    __WoweeWarn(string.format(\n"
        "      'objectives tracker collapsing: %s (userCollapsed=%s, lines=%s)',\n"
        "      self.userCollapsed and 'the player asked' or\n"
        "        'BY ITSELF - an update measured nothing, and that also disables "
        "the expand button',\n"
        "      tostring(self.userCollapsed),\n"
        "      tostring(WatchFrameLines and WatchFrameLines:IsShown())))\n"
        "    return original(self)\n"
        "  end\n"
        "end\n");

    // The same for the progress panel, which is the one a blank parchment was
    // photographed on - Continue and Cancel, with nothing between them.
    //
    // It reports the *length* of each string as well as its rect, because the
    // two failures look identical on screen and need opposite fixes: a string
    // that is empty is missing data, and a string that has text but no height
    // is missing a measurement.
    luaEngine_.executeString(
        // HookScript on the frame, not a wrapper around the global.
        //
        // Wrapping QuestFrameProgressPanel_OnShow fired in the runner, where
        // the function is called by name, and never once in a real session -
        // because what runs on screen is the *widget's* stored script, and
        // replacing the global afterwards does not touch it. The hook has to
        // go where the client actually dispatches.
        "if QuestFrameProgressPanel and QuestFrameProgressPanel.HookScript then\n"
        "  QuestFrameProgressPanel:HookScript('OnShow', function()\n"
        "    local function say(f, n)\n"
        "      if not f then return n .. '=absent' end\n"
        "      local t = (f.GetText and f:GetText()) or ''\n"
        "      return string.format('%s len=%d (%.0f,%.0f %.0fx%.0f) vis=%s',\n"
        "        n, string.len(t), f:GetLeft() or -1, f:GetBottom() or -1,\n"
        "        f:GetWidth() or 0, f:GetHeight() or 0,\n"
        "        tostring(f:IsVisible()))\n"
        "    end\n"
        "    __WoweeWarn('quest progress built: ' ..\n"
        "      say(QuestProgressText, 'ProgressText') .. ' | ' ..\n"
        "      say(QuestProgressTitle, 'Title') .. ' | ' ..\n"
        "      say(QuestFrameProgressPanel, 'Panel'))\n"
        "  end)\n"
        "end\n");

    // The game-menu button opens this client's settings.
    //
    // ToggleGameMenu is FrameXML's own function and it shows GameMenuFrame,
    // which is suppressed - so the button did nothing at all. Replaced rather
    // than hooked, and only while this client owns that panel: once the menu is
    // FrameXML's, the original is drawn and should be the one that answers.
    //
    // After FrameXML has loaded, because a definition written before it would
    // simply be overwritten by uiparent.lua.
    luaEngine_.executeString(
        "if not __WoweeOwnsGameMenu and __WoweeOpenClientSettings then\n"
        "  ToggleGameMenu = function() __WoweeOpenClientSettings() end\n"
        "end\n");

    // Where FrameXML's own chat output goes when this client owns the chat.
    //
    // Twenty-nine places write through DEFAULT_CHAT_FRAME:AddMessage - the
    // ready check's "you were away", the battleground countdowns, the world
    // state warnings, uiparent's four. That name is ChatFrame1: chatframe.lua
    // assigns it at file scope and ChatFrame1's own OnLoad assigns it again,
    // and suppression only stops the frame being drawn, so every one of those
    // lines was being stored on a hidden frame and never seen.
    //
    // Redirected rather than replaced. A bare table would raise the moment
    // something asked for GetID, GetWidth, GetFont, SetPoint or
    // IsUserPlaced - all of which FrameXML calls on this - so the frames stay
    // frames and only AddMessage is pointed elsewhere. A field on the table
    // wins over the metatable's method, which is what makes that work.
    //
    // All ten windows, not only the first: ChatFrame_MessageEventHandler
    // writes to whichever window a message type is registered on.
    luaEngine_.executeString(
        std::string("__WoweeOwnsChat = ") +
        (ui::frameXmlOwns(ui::UiElement::Chat) ? "true" : "false") + "\n");
    luaEngine_.executeString(
        "if not __WoweeOwnsChat and __WoweeClientChatAddMessage then\n"
        "  for i = 1, 10 do\n"
        "    local f = _G['ChatFrame' .. i]\n"
        "    if type(f) == 'table' then\n"
        "      f.AddMessage = __WoweeClientChatAddMessage\n"
        "    end\n"
        "  end\n"
        "end\n");

    // This client's own slash commands, into SlashCmdList.
    //
    // The registry in ChatPanel answers about seventy names FrameXML has no
    // equivalent for - /unstuck, /coords, /whereami, /transport, /threat, the
    // GM helpers, the helm and cloak toggles. It was reached from this
    // client's own chat input and from nowhere else, so handing chat over took
    // every one of them away: FrameXML's ChatEdit_ParseText walks SlashCmdList
    // and consults nothing beyond it.
    //
    // After FrameXML has loaded, so the taken set below is complete and this
    // can never shadow a command the interface already answers - /follow going
    // to a no-op because both sides claimed it is a fault this file has seen.
    //
    // Registered whichever interface owns the chat. When this client owns it,
    // sendChatMessage tries SlashCmdList before its own registry and will now
    // find these first; that is one extra hop to the same command rather than
    // a difference in behaviour, and it keeps the two paths identical.
    luaEngine_.executeString(
        "if __WoweeClientCommandNames and __WoweeRunClientCommand then\n"
        // Every /name FrameXML already answers. The table is keyed by the
        // slash text rather than by the SlashCmdList key, because that is what
        // collides: two different keys can name the same command.
        "  local taken = {}\n"
        "  for k, v in pairs(_G) do\n"
        "    if type(k) == 'string' and type(v) == 'string' and\n"
        "       string.sub(k, 1, 6) == 'SLASH_' then\n"
        "      taken[string.lower(v)] = true\n"
        "    end\n"
        "  end\n"
        "  for _, name in ipairs(__WoweeClientCommandNames()) do\n"
        "    local slash = '/' .. name\n"
        "    if not taken[slash] then\n"
        "      local key = 'WOWEE_' .. string.upper(name)\n"
        "      _G['SLASH_' .. key .. '1'] = slash\n"
        // Straight to the registry. Going back through the chat path would
        // find the entry this handler was called from and recurse.
        "      SlashCmdList[key] = function(msg)\n"
        "        __WoweeRunClientCommand(name, msg or '')\n"
        "      end\n"
        "    end\n"
        "  end\n"
        "end\n");


    luaEngine_.executeString(
        "if UIParent and UIParent.SetAttribute then\n"
        "  local defaults = {\n"
        "    TOP_OFFSET = 0, LEFT_OFFSET = 0, CENTER_OFFSET = 0,\n"
        "    RIGHT_OFFSET = 0, RIGHT_OFFSET_BUFFER = 0,\n"
        "    DEFAULT_FRAME_WIDTH = 338,\n"
        "  }\n"
        // Written straight into the attribute table rather than through
        // SetAttribute, which fires OnAttributeChanged: the panel manager runs
        // on the first one and reads the rest before the loop has set them, so
        // seeding through the setter failed on whichever name pairs() happened
        // to leave for last. These are initial values, not changes.
        "  local store = rawget(UIParent, '__attributes')\n"
        "  if not store then store = {}; rawset(UIParent, '__attributes', store) end\n"
        "  for name, value in pairs(defaults) do\n"
        "    if store[name] == nil then store[name] = value end\n"
        "  end\n"
        "end\n");

    // The tracker's "quests in this zone" filter, which needs a table this
    // client never filled.
    //
    // watchframe.lua drops a watched quest unless CURRENT_MAP_QUESTS holds it,
    // and WatchFrame_GetCurrentMapQuests builds that from the map's quest
    // markers - QuestMapUpdateAllQuests and QuestPOIGetQuestIDByVisibleIndex,
    // both of which answer from the POIs the server has sent for whatever it
    // was asked about rather than from the zone the player is standing in. So
    // the table was never the current zone's quests, and the only way to see a
    // tracker at all was to leave the remote-zones bit set and show every
    // watched quest in the log.
    //
    // Rebuilt from the quest log instead. The log is already grouped under
    // zone headers, and the header a quest sits under is the zone it belongs
    // to - which is the question being asked, and one this client can answer
    // without the server sending anything. Quests under a profession or class
    // header match no zone name and are correctly left out.
    //
    // After FrameXML, because this replaces one of its functions. The tracker
    // re-runs it on ZONE_CHANGED_NEW_AREA, so walking out of a zone empties it
    // and walking into one fills it again.
    luaEngine_.executeString(
        "if WatchFrame_GetCurrentMapQuests and CURRENT_MAP_QUESTS then\n"
        "  WatchFrame_GetCurrentMapQuests = function()\n"
        "    table.wipe(CURRENT_MAP_QUESTS)\n"
        "    local zone = GetRealZoneText and GetRealZoneText() or nil\n"
        "    if not zone or zone == '' then return end\n"
        "    local header = nil\n"
        "    for i = 1, GetNumQuestLogEntries() do\n"
        "      local title, _, _, _, isHeader, _, _, _, questID = GetQuestLogTitle(i)\n"
        "      if isHeader then\n"
        "        header = title\n"
        // A collapsed header still holds its quests, and the tracker watches
        // them whether the log shows them or not - so this walks the whole log
        // rather than what is on screen.
        "      elseif header == zone and questID and questID ~= 0 then\n"
        "        CURRENT_MAP_QUESTS[questID] = i\n"
        "      end\n"
        "    end\n"
        "  end\n"
        "end\n");

    // The browse tab's column headers, which sort nothing on a single page.
    //
    // Clicking one calls AuctionFrameBrowse_Search rather than
    // SortAuctionApplySort - the browse list is meant to be reordered by the
    // realm and re-sent, and the ordering does travel with the query. But
    // AzerothCore only sorts when the result runs past one page, so a search
    // returning fifty rows or fewer comes back in the order the realm walked
    // its map however the headers are clicked, and every click looked inert.
    //
    // Sorted here instead, immediately before the rows are drawn. The keys are
    // the ones the header click already set, the sort is stable, and a page the
    // realm ordered is left as it is - so this corrects the case the realm
    // skipped and changes nothing in the case it did not.
    //
    // Wrapped rather than replaced: the body of that function is FrameXML's and
    // this has no business being a copy of it.
    //
    // On ADDON_LOADED, because Blizzard_AuctionUI is load-on-demand and none of
    // its functions exist until the player talks to an auctioneer - a wrapper
    // written now would find nothing to wrap and fail quietly.
    luaEngine_.executeString(
        "do\n"
        "  local f = CreateFrame('Frame')\n"
        "  f:RegisterEvent('ADDON_LOADED')\n"
        "  f:SetScript('OnEvent', function(self, event, name)\n"
        "    if name ~= 'Blizzard_AuctionUI' then return end\n"
        "    self:UnregisterEvent('ADDON_LOADED')\n"
        "    if not AuctionFrameBrowse_Update or not SortAuctionApplySort then return end\n"
        "    local inner = AuctionFrameBrowse_Update\n"
        "    AuctionFrameBrowse_Update = function(...)\n"
        "      SortAuctionApplySort('list')\n"
        "      return inner(...)\n"
        "    end\n"
        "  end)\n"
        "end\n");

    // Let the bag windows and the character sheet be dragged around.
    //
    // A deliberate departure from 3.3.5, where neither can be moved: the bags
    // arrange themselves up the right-hand side and the character sheet is a
    // fixed panel. Asked for, and harmless - the item buttons inside them are
    // what a drag starting on a slot picks up, because a drag belongs to the
    // frame the press landed on.
    luaEngine_.executeString(
        "local function draggable(f)\n"
        "  if not f then return end\n"
        "  f:SetMovable(true)\n"
        "  f:EnableMouse(true)\n"
        "  f:RegisterForDrag('LeftButton')\n"
        "  f:SetScript('OnDragStart', function(self) self:StartMoving() end)\n"
        "  f:SetScript('OnDragStop', function(self) self:StopMovingOrSizing() end)\n"
        "end\n"
        "for i = 1, 13 do draggable(_G['ContainerFrame' .. i]) end\n"
        "draggable(CharacterFrame)\n");

    // A search box on the trainer, which 3.3.5 has no equivalent of.
    //
    // A tradeskill trainer is a hundred rows deep - Georgio Bolero sends 103 -
    // and the only way through them is the scroll bar and the three type
    // filters. Typing "linen" is the question a player actually has.
    //
    // The matching is the client's, not this script's: the panel identifies a
    // service by its position in the list, and every one of the bindings that
    // takes a service index has to agree about which service index 3 is. So the
    // box hands the text to SetTrainerServiceSearch and the list it filters is
    // the one GetNumTrainerServices already counts - see shownTrainerServices,
    // where the search and the type filters meet.
    //
    // Beside the filter dropdown, on the row the panel leaves empty between the
    // greeting and the first skill.
    //
    // On ADDON_LOADED, because Blizzard_TrainerUI is load-on-demand.
    luaEngine_.executeString(
        "do\n"
        "  local f = CreateFrame('Frame')\n"
        "  f:RegisterEvent('ADDON_LOADED')\n"
        "  f:SetScript('OnEvent', function(self, event, name)\n"
        "    if name ~= 'Blizzard_TrainerUI' then return end\n"
        "    self:UnregisterEvent('ADDON_LOADED')\n"
        "    if not ClassTrainerFrame or not ClassTrainerFrameFilterDropDown then return end\n"
        // Without the binding there is nothing to search with, and a box that
        // filters nothing is worse than no box.
        "    if not SetTrainerServiceSearch then return end\n"
        "    local box = CreateFrame('EditBox', 'ClassTrainerFrameSearchBox',\n"
        "                            ClassTrainerFrame, 'InputBoxTemplate')\n"
        "    box:SetWidth(104)\n"
        "    box:SetHeight(20)\n"
        "    box:SetPoint('RIGHT', ClassTrainerFrameFilterDropDown, 'LEFT', 6, 3)\n"
        "    box:SetAutoFocus(false)\n"
        "    if box.SetMaxLetters then box:SetMaxLetters(40) end\n"
        // The word in the empty box, the way every search box in the interface
        // says what it is for. SEARCH is globalstrings' own.
        "    local hint = box:CreateFontString(nil, 'ARTWORK', 'GameFontDisableSmall')\n"
        "    hint:SetPoint('LEFT', box, 'LEFT', 4, 0)\n"
        "    hint:SetText(SEARCH or 'Search')\n"
        "    local function hinting()\n"
        "      if (box:GetText() or '') == '' and not box:HasFocus() then\n"
        "        hint:Show()\n"
        "      else\n"
        "        hint:Hide()\n"
        "      end\n"
        "    end\n"
        "    local function apply(self)\n"
        "      SetTrainerServiceSearch(self:GetText() or '')\n"
        "      hinting()\n"
        "    end\n"
        // Both, because they are not the same event here: typing raises
        // OnTextChanged and SetText raises OnTextSet, where WoW raises
        // OnTextChanged for both. Bound to one only, clearing the box left the
        // word still filtering the list behind it.
        "    box:SetScript('OnTextChanged', apply)\n"
        "    box:SetScript('OnTextSet', apply)\n"
        "    box:SetScript('OnEditFocusGained', hinting)\n"
        "    box:SetScript('OnEditFocusLost', hinting)\n"
        "    box:SetScript('OnEscapePressed', function(self)\n"
        "      self:SetText('')\n"
        "      apply(self)\n"
        "      self:ClearFocus()\n"
        "    end)\n"
        "    box:SetScript('OnEnterPressed', function(self) self:ClearFocus() end)\n"
        // A word left in the box would filter the next trainer the player walks
        // up to, who is a different NPC with a different list, and the rows it
        // hid would look like rows he does not teach.
        "    hooksecurefunc('ClassTrainerFrame_Show', function()\n"
        "      if (box:GetText() or '') ~= '' then box:SetText('') end\n"
        "      SetTrainerServiceSearch('')\n"
        "      hinting()\n"
        "    end)\n"
        "    hinting()\n"
        "  end)\n"
        "end\n");

    LOG_WARNING("FrameXML: ", lua, " Lua files and ", xml, " XML files loaded, ",
                failed, " failed in ", sinceMs(loadStart), "ms");
    for (const auto& [file, why] : failures) {
        LOG_WARNING("FrameXML:   ", file, " - ", why);
    }
    return failed == 0;
}

bool AddonManager::loadXmlFile(const std::string& path, int depth) {
    // Includes nest, and a file that includes itself would otherwise recurse
    // until the stack gives out.
    constexpr int kMaxDepth = 16;
    if (depth > kMaxDepth) {
        lastXmlError_ = "include nesting too deep";
        LOG_ERROR("AddonManager: include nesting too deep at ", path);
        return false;
    }

    std::ifstream in(path, std::ios::binary);
    if (!in) {
        lastXmlError_ = "not on disk";
        LOG_WARNING("AddonManager: XML not found: ", path);
        return false;
    }
    std::stringstream buffer;
    buffer << in.rdbuf();

    ui::XmlNode root;
    std::string error;
    if (!ui::parseXml(buffer.str(), root, error)) {
        lastXmlError_ = "XML parse: " + error;
        LOG_ERROR("AddonManager: ", path, ": ", error);
        return false;
    }

    ui::EmitResult emitted = ui::emitFrameXml(root);
    for (const auto& w : emitted.warnings) {
        LOG_WARNING("AddonManager: ", path, ": ", w);
    }

    // The Lua the XML became, on disk, when asked for.
    //
    // Everything downstream of here reads as a Lua problem - a global that is
    // nil, a frame with no size - and the answer is nearly always in what the
    // emitter wrote rather than in what the script did with it. Reading it is
    // the difference between finding a mis-substituted $parent in one grep and
    // inferring it from a frame that ended up in the wrong place.
    if (const char* dumpDir = std::getenv("WOWEE_FRAMEXML_EMIT_DIR")) {
        if (*dumpDir) {
            std::error_code ec;
            fs::create_directories(dumpDir, ec);
            const fs::path out =
                fs::path(dumpDir) / (fs::path(path).filename().string() + ".lua");
            std::ofstream f(out);
            if (f) f << emitted.lua;
        }
    }

    const fs::path dir = fs::path(path).parent_path();
    bool ok = true;

    // Resolved without regard to case, the same as the manifest's own files. A
    // Script element says MovieFrame.lua and the file on disk is
    // movieframe.lua, so joining the two naively fails - which took out most of
    // FrameXML on the first attempt, one referenced script at a time.
    auto sibling = [&](const std::string& rawName) {
        // Windows separators, because the interface is written with them. On
        // anything else a backslash is an ordinary character in a filename, so
        // "..\\..\\FrameXML\\UIPanelTemplates.xml" resolved to nothing and the
        // include silently failed - which failed the file that asked for it,
        // and the guild bank's own XML is one of the two that do.
        std::string name = rawName;
        std::replace(name.begin(), name.end(), '\\', '/');

        // Relative to the file that named it first.
        if (fs::path p = resolvePath(dir, name); !p.empty()) return p;

        // Then FrameXML itself. An addon includes a shared template by bare
        // name - inspectpvpframe.xml asks for PVPFrameTemplates.xml - and by a
        // path back out of its own folder, and both mean the same place.
        const fs::path base = fs::path(frameXmlResolvedDir_.empty()
                                           ? frameXmlDir_ : frameXmlResolvedDir_);
        if (!base.empty()) {
            if (fs::path p = resolvePath(base, fs::path(name).filename().string());
                !p.empty()) {
                return p;
            }
        }
        // Neither place has it under any spelling, which is a different thing
        // from the open failing - "cannot open" reads as a permission or a
        // lock, and this is a file that is not in the install. Turtle's
        // StaticPopup.xml asks for StaticPopup.lua and one report's extraction
        // did not contain it: every static popup in the game was dead, and
        // WorldFrame's OnUpdate with them, because STATICPOPUP_NUMDIALOGS is
        // declared in that file. Said here, where both searched places are
        // still to hand.
        LOG_WARNING("AddonManager: no file named ", rawName, " beside ",
                    fs::path(path).filename().string(), " (looked in ", dir.string(),
                    base.empty() ? std::string() : " and in " + base.string(),
                    ") - it is missing from this install");
        return dir / name;
    };

    // Order matters and is not the order the emitter reports things in. Includes
    // carry the templates a file inherits from, and scripts define the functions
    // its handlers name, so both have to be in place before any frame is built.
    // A file is only as loadable as what it pulls in, so the reason kept here is
    // the first real one - the include or script that actually broke - rather
    // than the name of whichever file happened to reference it.
    // Which element named the file does not decide how to read it - the
    // extension does.
    //
    // Turtle's OptionsFrame.xml pulls its two Lua files in with <Include>, and
    // this handed them to the XML parser: "expected an element" on
    // Options\Options.lua and Options\Functions.lua, and with them the whole
    // options framework - OptionsFrame_AddCategory, every panel that registers
    // one, and a run of names reading as missing somewhere else entirely. The
    // real client reads an include by what the file is.
    const auto isLua = [](const std::string& name) {
        return name.size() > 4 &&
               (name.compare(name.size() - 4, 4, ".lua") == 0 ||
                name.compare(name.size() - 4, 4, ".LUA") == 0);
    };
    for (const auto& inc : emitted.includeFiles) {
        const bool loaded = isLua(inc)
            ? luaEngine_.executeFile(sibling(inc).string())
            : loadXmlFile(sibling(inc).string(), depth + 1);
        if (!loaded) {
            if (ok) {
                lastXmlError_ = "include " + inc + ": " +
                                (isLua(inc) ? luaEngine_.lastError() : lastXmlError_);
            }
            ok = false;
        }
    }
    for (const auto& script : emitted.scriptFiles) {
        // A file the manifest names and the package does not ship is not a
        // failure. Addons carry these routinely - Bagnon's config lists seven
        // localisations and ships five, and a widget it no longer has - and
        // the real client loads what is there and says nothing. Reported as an
        // error, three absent translations made a working addon read as a
        // broken one in the log, which is worse than silence: it is the log
        // being wrong.
        //
        // Said once, and quietly, because a file that is missing when it
        // should be there is still worth knowing.
        if (!fs::exists(sibling(script))) {
            LOG_WARNING("AddonManager: ", path, " names ", script,
                        ", which this addon does not ship - skipped");
            continue;
        }
        // And the other way round, for the same reason.
        const bool loaded = isLua(script)
            ? luaEngine_.executeFile(sibling(script).string())
            : loadXmlFile(sibling(script).string(), depth + 1);
        if (!loaded) {
            if (ok) {
                lastXmlError_ = "script " + script + ": " +
                                (isLua(script) ? luaEngine_.lastError() : lastXmlError_);
            }
            LOG_ERROR("AddonManager: ", path, " referenced ", script, " which failed");
            ok = false;
        }
    }
    if (!emitted.lua.empty()) {
        if (!luaEngine_.executeString(emitted.lua)) {
            if (ok) lastXmlError_ = "frames: " + luaEngine_.lastError();
            LOG_ERROR("AddonManager: frames from ", path, " failed to build");
            ok = false;
        } else {
            LOG_INFO("AddonManager: built frames from ", path);
        }
    }
    return ok;
}

bool AddonManager::loadAddon(const TocFile& addon) {
    // Load SavedVariables before addon code (so globals are available at load time)
    auto savedVars = addon.getSavedVariables();
    if (!savedVars.empty()) {
        std::string svPath = getSavedVariablesPath(addon);
        luaEngine_.loadSavedVariables(svPath);
        LOG_DEBUG("AddonManager: loaded saved variables for '", addon.addonName, "'");
    }
    // Load per-character SavedVariables
    auto savedVarsPC = addon.getSavedVariablesPerCharacter();
    if (!savedVarsPC.empty()) {
        std::string svpcPath = getSavedVariablesPerCharacterPath(addon);
        if (!svpcPath.empty()) {
            luaEngine_.loadSavedVariables(svpcPath);
            LOG_DEBUG("AddonManager: loaded per-character saved variables for '", addon.addonName, "'");
        }
    }

    bool success = true;
    for (const auto& filename : addon.files) {
        std::string lower = filename;
        for (char& c : lower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

        // Through resolvePath, because a .toc names its files the way Blizzard
        // wrote them - Blizzard_TalentUI.xml - and this install has them in
        // lower case. Concatenating the two finds nothing on a case-sensitive
        // filesystem, which is every one of the Blizzard load-on-demand addons.
        const fs::path resolved = resolvePath(fs::path(addon.basePath), filename);
        const std::string fullPath =
            resolved.empty() ? (addon.basePath + "/" + filename) : resolved.string();

        if (lower.size() >= 4 && lower.substr(lower.size() - 4) == ".lua") {
            if (!luaEngine_.executeFile(fullPath)) {
                LOG_ERROR("AddonManager: '", addon.addonName, "' failed on ", filename);
                success = false;
            } else {
                LOG_INFO("AddonManager: ran ", addon.addonName, "/", filename);
            }
        } else if (lower.size() >= 4 && lower.substr(lower.size() - 4) == ".xml") {
            if (!loadXmlFile(fullPath, 0)) success = false;
        }
    }

    // Fire ADDON_LOADED event after all addon files are executed
    // This is the standard WoW pattern for addon initialization
    if (success) {
        luaEngine_.fireEvent("ADDON_LOADED", {addon.addonName});
    }
    return success;
}

namespace {
std::string lowered(std::string v) {
    for (char& c : v) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return v;
}
}  // namespace

bool AddonManager::isAddOnLoadedByName(const std::string& name) const {
    return lodLoaded_.count(lowered(name)) != 0;
}

bool AddonManager::loadAddOnByName(const std::string& name, std::string& reason) {
    const std::string key = lowered(name);
    // Already loaded is success, not an error: FrameXML calls LoadAddOn every
    // time a panel is opened and only checks the first return.
    if (lodLoaded_.count(key)) { reason.clear(); return true; }

    const TocFile* found = nullptr;
    for (const TocFile& a : lodAddons_) {
        if (lowered(a.addonName) == key) { found = &a; break; }
    }
    if (!found) {
        // Not on the on-demand list is not the same as not present. An addon
        // whose .toc omits LoadOnDemand is loaded at startup instead, and
        // FrameXML still asks for it by name: uiparent.lua calls
        // UIParentLoadAddOn("Blizzard_TokenUI"), whose .toc has no such line,
        // and that reported MISSING for an addon already running - raising the
        // "Couldn't load" popup over a UI that was working.
        for (const TocFile& a : addons_) {
            if (lowered(a.addonName) != key) continue;
            if (!isAddonEnabled(a.addonName)) { reason = "DISABLED"; return false; }
            // Only once the startup pass has actually run it. Before that it is
            // listed but not loaded, and saying otherwise would have the caller
            // use frames that do not exist yet.
            if (addonsLoaded_) { reason.clear(); return true; }
            break;
        }
        reason = "MISSING";
        return false;
    }
    if (!isAddonEnabled(found->addonName)) { reason = "DISABLED"; return false; }

    // Recorded before loading, not after: the addon's own files run during
    // this call and one of them may call LoadAddOn on the same name, which
    // would otherwise recurse until the stack gave out.
    lodLoaded_.insert(key);
    LOG_INFO("AddonManager: loading on demand: ", found->addonName);
    if (!loadAddon(*found)) {
        // CORRUPT, not a token of our own. UIParentLoadAddOn builds a
        // global name out of whatever comes back - _G["ADDON_"..reason] -
        // and hands it to format, so a reason globalstrings does not have
        // makes the report raise instead of reporting. See lua_LoadAddOn,
        // which will not let an unknown one through either.
        reason = "CORRUPT";
        LOG_WARNING("AddonManager: '", found->addonName, "' failed to load on demand");
        // Left in the loaded set deliberately. A half-run addon has already
        // built frames and set globals, and running its files a second time
        // would build them again - the duplicate-frame problem the scan goes
        // out of its way to avoid.
        return false;
    }
    // What it draws is on screen from here, which for some of them is a second
    // copy of something this client is already drawing.
    ui::frameXmlNoteAddOnLoaded(found->addonName);
    return true;
}

bool AddonManager::runScript(const std::string& code) {
    return luaEngine_.executeString(code);
}

void AddonManager::runInterfaceCommand(const std::string& lua) {
    if (lua.empty()) return;
    if (!luaEngine_.executeString(lua)) {
        LOG_WARNING("interface command failed: ", lua);
    }
}

bool AddonManager::interfaceCommandBoolean(const std::string& expression) {
    if (expression.empty()) return false;
    return luaEngine_.evaluateBoolean(expression);
}

void AddonManager::fireEvent(const std::string& event, const std::vector<std::string>& args) {
    luaEngine_.fireEvent(event, args);
}

void AddonManager::update(float deltaTime) {
    luaEngine_.dispatchOnUpdate(deltaTime);
}

void AddonManager::saveAllSavedVariables() {
    // Only what a loaded interface is holding. unloadAll saves, tears the Lua
    // state down and then scans the disk again - so the addon list is full
    // again while the state that held their variables is gone, and the save on
    // the way out of the process would write every one of those files from a
    // state that never loaded them.
    if (!addonsLoaded_) return;
    for (const auto& addon : addons_) {
        auto savedVars = addon.getSavedVariables();
        if (!savedVars.empty()) {
            std::string svPath = getSavedVariablesPath(addon);
            luaEngine_.saveSavedVariables(svPath, savedVars);
        }
        auto savedVarsPC = addon.getSavedVariablesPerCharacter();
        if (!savedVarsPC.empty()) {
            std::string svpcPath = getSavedVariablesPerCharacterPath(addon);
            if (!svpcPath.empty()) {
                luaEngine_.saveSavedVariables(svpcPath, savedVarsPC);
            }
        }
    }
}

bool AddonManager::unloadAll() {
    saveAllSavedVariables();
    addons_.clear();
    lodAddons_.clear();
    lodLoaded_.clear();
    addonsLoaded_ = false;
    // Closing the state is what disposes of the interface: the widget tree
    // goes down with it, so the frames FrameXML built are gone rather than
    // left for the next load to build a second copy on top of.
    luaEngine_.shutdown();

    if (!luaEngine_.initialize()) {
        LOG_ERROR("AddonManager: failed to reinitialize the Lua VM");
        return false;
    }
    luaEngine_.setGameHandler(gameHandler_);
    luaEngine_.setLuaServices(luaServices_);

    // The list of what is on disk is not part of the interface and has to
    // survive it - the AddOns manager on character select reads it while
    // nothing is loaded at all.
    if (!addonsPath_.empty()) scanAddons(addonsPath_);
    return true;
}

bool AddonManager::reload() {
    LOG_INFO("AddonManager: reloading all addons...");
    if (!unloadAll()) {
        LOG_ERROR("AddonManager: failed to reinitialize Lua VM during reload");
        return false;
    }
    if (!addonsPath_.empty()) {
        loadAllAddons();
    }
    LOG_INFO("AddonManager: reload complete");
    return true;
}

void AddonManager::shutdown() {
    saveAllSavedVariables();
    addons_.clear();
    luaEngine_.shutdown();
}

} // namespace wowee::addons
