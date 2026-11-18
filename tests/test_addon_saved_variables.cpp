#include "input_test_support.hpp"
#include "addons/addon_manager.hpp"

using namespace input_test;
namespace addons = wowee::addons;

namespace {

struct PackagedAddon : Fixture {
    const fs::path code = root / "install" / "addons" / "PersistenceTest";
    const fs::path saved = root / "addons";
    Environment frameXml{"WOWEE_LOAD_FRAMEXML", "0"};
    const fs::path previousDirectory = fs::current_path();

    PackagedAddon() {
        fs::create_directories(code);
        writeFile(code / "PersistenceTest.toc",
                  "## SavedVariables: PackagedSettings\n"
                  "## SavedVariablesPerCharacter: CharacterSettings\n"
                  "PersistenceTest.lua\n");
        writeFile(code / "PersistenceTest.lua",
                  "PackagedSettings = rawget(_G, 'PackagedSettings') or { columns = 10 }\n"
                  "CharacterSettings = rawget(_G, 'CharacterSettings') or { value = 0 }\n");
        // Keep the scan away from the checkout's own addons and player data.
        fs::current_path(root / "install");
    }

    ~PackagedAddon() {
        std::error_code ec;
        fs::permissions(code, fs::perms::owner_all, fs::perm_options::add, ec);
        fs::current_path(previousDirectory, ec);
    }

    void makeReadOnly() {
#ifndef _WIN32
        for (const auto& file : fs::directory_iterator(code)) {
            fs::permissions(file.path(), fs::perms::owner_read |
                            fs::perms::group_read | fs::perms::others_read);
        }
        fs::permissions(code, fs::perms::owner_read | fs::perms::owner_exec |
                        fs::perms::group_read | fs::perms::group_exec |
                        fs::perms::others_read | fs::perms::others_exec);
#endif
    }

    void load(addons::AddonManager& manager, const std::string& character = "Alice") {
        REQUIRE(manager.initialize(nullptr));
        manager.setCharacterName(character);
        manager.scanAddons((root / "install" / "addons").string());
        REQUIRE(manager.getAddons().size() == 1);
        manager.loadAllAddons();
    }
};

} // namespace

TEST_CASE_METHOD(PackagedAddon, "Addon settings survive a restart with read-only addon code",
                 "[addons][saved-variables]") {
    makeReadOnly();
    {
        addons::AddonManager manager;
        load(manager);
        REQUIRE(manager.runScript("PackagedSettings.columns = 18; CharacterSettings.value = 42"));
        manager.saveAllSavedVariables();
        CHECK(fs::exists(saved / "PersistenceTest.lua.saved"));
        CHECK(fs::exists(saved / "PersistenceTest.Alice.lua.saved"));
        CHECK_FALSE(fs::exists(code / "PersistenceTest.lua.saved"));
        CHECK_FALSE(fs::exists(code / "PersistenceTest.Alice.lua.saved"));
    }
    {
        addons::AddonManager manager;
        load(manager);
        CHECK(manager.interfaceCommandBoolean("PackagedSettings.columns == 18"));
        CHECK(manager.interfaceCommandBoolean("CharacterSettings.value == 42"));
    }
    {
        addons::AddonManager manager;
        load(manager, "Bob");
        CHECK(manager.interfaceCommandBoolean("PackagedSettings.columns == 18"));
        CHECK(manager.interfaceCommandBoolean("CharacterSettings.value == 0"));
    }
}

TEST_CASE_METHOD(PackagedAddon, "Legacy addon settings seed config without overriding later changes",
                 "[addons][saved-variables]") {
    const fs::path legacy = code / "PersistenceTest.lua.saved";
    const fs::path legacyCharacter = code / "PersistenceTest.Alice.lua.saved";
    writeFile(legacy, "PackagedSettings = { columns = 12 }\n");
    writeFile(legacyCharacter, "CharacterSettings = { value = 24 }\n");
    makeReadOnly();
    {
        addons::AddonManager manager;
        load(manager);
        CHECK(manager.interfaceCommandBoolean("PackagedSettings.columns == 12"));
        CHECK(manager.interfaceCommandBoolean("CharacterSettings.value == 24"));
        REQUIRE(manager.runScript("PackagedSettings.columns = 16; CharacterSettings.value = 48"));
        manager.saveAllSavedVariables();
        CHECK(contents(legacy) == "PackagedSettings = { columns = 12 }\n");
        CHECK(contents(legacyCharacter) == "CharacterSettings = { value = 24 }\n");
    }
    {
        addons::AddonManager manager;
        load(manager);
        CHECK(manager.interfaceCommandBoolean("PackagedSettings.columns == 16"));
        CHECK(manager.interfaceCommandBoolean("CharacterSettings.value == 48"));
    }
}
