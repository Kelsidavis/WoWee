#pragma once

#include "addons/lua_engine.hpp"
#include "addons/toc_parser.hpp"
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace wowee::addons {

class AddonManager {
public:
    AddonManager();
    ~AddonManager();

    bool initialize(game::GameHandler* gameHandler, const LuaServices& services = {});
    void scanAddons(const std::string& addonsPath);
    void loadAllAddons();
    /// Parse an XML file, build what it declares, and follow its includes and
    /// scripts. depth guards against a file that includes itself.
    bool loadXmlFile(const std::string& path, int depth);
    /// Load the original interface from its own manifest, in the order it
    /// states. Opt-in through WOWEE_LOAD_FRAMEXML; see loadAllAddons.
    /// Register this client's own settings as a category in FrameXML's
    /// Interface Options, built from the schema. Runs after FrameXML, because
    /// InterfaceOptions_AddCategory is FrameXML's.
    void registerWoweeOptionsPanel();
    /// Move the coin amounts off the coins they are drawn against.
    void giveCoinAmountsClearance();

    bool loadFrameXml(const std::string& frameXmlDir);
    /// Where FrameXML lives, remembered at scan time so the loader can find it.
    void setFrameXmlDir(const std::string& dir) { frameXmlDir_ = dir; }
    bool runScript(const std::string& code);
    /// Run one line of interface Lua, for a keybinding whose window FrameXML
    /// now owns. Errors are logged rather than thrown: a bad line here should
    /// cost the keypress, not the frame.
    void runInterfaceCommand(const std::string& lua);
    /// The same, for a command whose answer decides what happens next.
    bool interfaceCommandBoolean(const std::string& expression);


    void fireEvent(const std::string& event, const std::vector<std::string>& args = {});
    void update(float deltaTime);
    void shutdown();

    [[nodiscard]] const std::vector<TocFile>& getAddons() const { return addons_; }
    /// The addons that wait to be asked for. Half the interface's own panels
    /// are here - the talent tree, the achievements, the macro editor - and
    /// each is loaded whole or not at all, so one of them raising during load
    /// costs its entire panel rather than degrading it.
    [[nodiscard]] const std::vector<TocFile>& getLoadOnDemandAddons() const { return lodAddons_; }

    /// Load a load-on-demand addon by name. Names are matched without regard to
    /// case, because the interface asks for "Blizzard_TalentUI" and the
    /// directory on a case-sensitive filesystem is blizzard_talentui.
    bool loadAddOnByName(const std::string& name, std::string& reason);
    [[nodiscard]] bool isAddOnLoadedByName(const std::string& name) const;
    LuaEngine* getLuaEngine() { return &luaEngine_; }
    [[nodiscard]] bool isInitialized() const { return luaEngine_.isInitialized(); }

    // Per-addon enable/disable (persisted). Disabled addons are skipped by
    // loadAllAddons; changes take effect on the next load (world enter or /reload).
    [[nodiscard]] bool isAddonEnabled(const std::string& addonName) const;
    void setAddonEnabled(const std::string& addonName, bool enabled);
    // True once any addon has been loaded this session (so the UI can note that a
    // toggle only applies after the next reload).
    [[nodiscard]] bool addonsLoaded() const { return addonsLoaded_; }

    void saveAllSavedVariables();
    void setCharacterName(const std::string& name) { characterName_ = name; }

    /// Re-initialize the Lua VM and reload all addons (used by /reload).
    bool reload();

private:
    LuaEngine luaEngine_;
    std::vector<TocFile> addons_;
    /// Declared LoadOnDemand: known, listed, and not run until asked for.
    std::vector<TocFile> lodAddons_;
    std::set<std::string> lodLoaded_;
    game::GameHandler* gameHandler_ = nullptr;
    LuaServices luaServices_;
    std::string addonsPath_;

    /// Every global the load-on-demand addons on disk define, read out of their
    /// own files. This is the source of truth for which names must answer as
    /// absent before their addon loads; the literal list in the Lua bootstrap
    /// is a floor for an install with no addons extracted.
    [[nodiscard]] std::vector<std::string> deferredAddonGlobals() const;

    bool loadAddon(const TocFile& addon);
    [[nodiscard]] std::string getSavedVariablesPath(const TocFile& addon) const;
    [[nodiscard]] std::string getSavedVariablesPerCharacterPath(const TocFile& addon) const;
    std::string characterName_;

    // addonName -> enabled. Absent means enabled (default on).
    std::unordered_map<std::string, bool> addonEnabled_;
    std::string frameXmlDir_;
    /// The same directory as it is actually spelled on disk.
    ///
    /// The caller says ".../interface/FrameXML" and this install has
    /// ".../interface/framexml". loadFrameXml resolves that to open it, and
    /// the resolved spelling was thrown away with the local it was kept in -
    /// so every later use of the member was a path that does not exist, on any
    /// filesystem that cares about case.
    std::string frameXmlResolvedDir_;
    /// Why the last loadXmlFile returned false, so a caller loading many files
    /// can report the reasons together instead of leaving them scattered.
    std::string lastXmlError_;
    bool addonsLoaded_ = false;
    static std::string enabledStatePath();
    void loadEnabledState();
    void saveEnabledState() const;
};

} // namespace wowee::addons
