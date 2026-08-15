// framexml_run - load FrameXML for real and run an expression against it.
//
// Every static sweep in tools/ works on the text: which names exist, which
// arguments line up, which frames are emitted. They are at their floor, and
// the bugs that are left are not visible in text - they are what happens when
// the interface actually runs. Escape opening nothing, the chat box refusing
// focus, a panel that stays empty: each is a Lua error raised inside a handler
// and swallowed, because a handler that raises looks exactly like a handler
// that decided to do nothing.
//
// Run it a second way before believing a clean report:
//
//     WOWEE_LUA_API_FALLBACK=0 framexml_run Data
//
// By default an unknown global answers with a stand-in rather than nil, which
// keeps a file alive past a name nothing implements - and hides that it was
// needed. With the fallback off, anything the interface actually depends on
// raises and names itself. As of 2026-08-07 that run is clean: no load errors,
// no addon failures, no login errors. It found the one thing that was not -
// Blizzard_BattlefieldMinimap, held up entirely by a stand-in for a frame the
// real client creates in C++.
//
// A clean default run and a failing fallback-off run is the shape to watch
// for: it means something is leaning on the stand-in and will fall over for
// anyone who turns it off.
//
// Clicking is the other thing to do with it, and it found the Send Mail tab
// raising before its frame was built. Walk the globals for tables carrying a
// widget id and a script table, keep the Buttons and CheckButtons, and Click()
// each one - but only where IsVisible() is true. Clicking a button in a panel
// nobody opened raises for reasons that are not faults: 516 of them against
// zero for the visible ones. Open a panel, click what became visible, hide it,
// move on.
//
// This is that run, without the client. The addon manager loads the real
// FrameXML through the real emitter into a real Lua state with the real
// bindings, and the only thing missing is a game behind them - every binding
// already guards its GameHandler pointer, so a null one gives the answers of a
// player who is not logged in. That is enough for anything whose fault is in
// the interface rather than in the data, which is what the swallowed errors
// are.
//
//     framexml_run <assetPath> [expression ...]
//
//     framexml_run Data 'ToggleGameMenu()' 'ChatFrame1EditBox:Show()'
//
// Errors are collected rather than printed as they happen, so the load and
// each expression are reported separately: an error during load is a different
// question from an error the expression caused.
//
// Exit status is the number of expressions that raised, capped at 100, so a
// script can ask "did this one still work" without reading the output.

#include "addons/addon_manager.hpp"
#include "addons/lua_services.hpp"
#include "ui/settings_schema.hpp"
#include "ui/settings_panel.hpp"
#include "ui/chat/chat_settings.hpp"
#include "core/app_clock.hpp"
#include "ui/widget_renderer.hpp"
#include "pipeline/asset_manager.hpp"
#include "ui/interface_fonts.hpp"
#include "ui/link_hit.hpp"
#include "game/expansion_profile.hpp"
#include "game/game_handler.hpp"
#include "game/game_services.hpp"
#include "game/character.hpp"
#include "game/spell_handler.hpp"

#include <imgui.h>

#include <cstdio>
#include <filesystem>
#include <cstring>
#include <cstdlib>
#include "core/env.hpp"
#include <string>
#include <set>
#include <vector>

int main(int argc, char** argv) {
    // Its own log file, before anything can open one.
    //
    // The logger truncates whatever it opens, and every process using it took
    // the same path - so running this from the repository root destroyed the
    // session log of the client that had just been played, which is the one
    // file a bug report needs. A tool that quietly deletes the evidence is
    // worse than a tool that does not exist.
    // And its own config corner, for the same reason: the missing-API list and
    // the Lua error list are rewritten on exit, and both are read from when a
    // report is being diagnosed.
    // Both only if the caller has not chosen one, which setEnvVar's overwrite
    // flag says directly.
    wowee::core::setEnvVar("WOWEE_CONFIG_ROOT", "logs/framexml_run_config", false);
    wowee::core::setEnvVar("WOWEE_LOG_FILE", "framexml_run.log", false);
    if (argc < 2) {
        std::fprintf(stderr,
                     "usage: framexml_run <assetPath> [expression ...]\n"
                     "  e.g. framexml_run Data 'ToggleGameMenu()'\n");
        return 2;
    }
    const std::string assetPath = argv[1];

    // FrameXML owns nothing unless it is asked to, and a harness that owns
    // nothing takes the client's side of every handover - which is not the
    // side being tested. Set before anything reads it.
    wowee::core::setEnvVar("WOWEE_FRAMEXML_UI", "all", false);

    // The client's settings, so the panels the schema generates have values to
    // show. Without this every control was built against a nil: WoweeGetSetting
    // goes through LuaServices, nothing here set any, and all seventy-one keys
    // answered empty - which reads as the settings being broken rather than as
    // the harness never having been given them.
    //
    // Backed by the schema's own defaults rather than by a SettingsPanel. The
    // panel wants a renderer and a window; the panels only want the numbers.
    //
    // Handed to initialize rather than set afterwards. AddonManager adds its
    // own callbacks to whatever it is given and installs the result, so a
    // setLuaServices call after it replaced the lot - setAddOnEnabled with
    // them - and the load-on-demand panels stopped loading. The calendar came
    // back NOFRAME.
    // A real SettingsPanel rather than a map of strings.
    //
    // A map answers and stores, which is enough to draw a control and read it
    // back - and not enough to be the client. setSettingValue clamps, finds the
    // field through the binding table, runs the side effects, and for the
    // quality preset assigns nine other settings and re-reads the panel. None
    // of that happens in a map, so a check that a preset moves what it claims
    // to move would have been asking the harness, not the client.
    //
    // Safe with no renderer: every side effect reaches its target through
    // services_.renderer, which is null here, and each is guarded.
    static wowee::ui::SettingsPanel settingsPanel;
    static wowee::ui::ChatSettings chatSettings;
    settingsPanel.setChatSettings(&chatSettings);
    {
        // The defaults, the way a fresh install starts.
        std::size_t count = 0;
        const wowee::ui::SettingDesc* schema = wowee::ui::clientSettingsSchema(count);
        for (std::size_t i = 0; i < count; ++i) {
            settingsPanel.setSettingValue(schema[i].key,
                                          wowee::ui::settingNumberText(schema[i].defaultValue));
        }
    }
    wowee::addons::LuaServices settingServices;
    settingServices.getClientSetting = [](const std::string& key) -> std::string {
        return settingsPanel.settingValue(key);
    };
    settingServices.setClientSetting = [](const std::string& key, const std::string& value) {
        settingsPanel.setSettingValue(key, value);
    };
    wowee::addons::AddonManager mgr;
    if (!mgr.initialize(nullptr, settingServices)) {
        std::fprintf(stderr, "framexml_run: Lua would not initialise\n");
        return 2;
    }

    std::vector<std::string> errors;
    if (auto* engine = mgr.getLuaEngine()) {
        engine->setLuaErrorCallback(
            [&errors](const std::string& e) { errors.push_back(e); });
    }

    mgr.setFrameXmlDir(assetPath + "/interface/FrameXML");
    mgr.scanAddons(assetPath + "/interface/AddOns");
    mgr.loadAllAddons();

    std::printf("== load: %zu error(s)\n", errors.size());
    for (const std::string& e : errors) std::printf("   %s\n", e.c_str());

    // Then every addon that waits to be asked for, because loading one is
    // where three faults were hiding and none of them was visible any other
    // way. A load-on-demand addon is loaded whole or not at all: a raise
    // during load loses the file, so the fault presents as a panel that does
    // not exist rather than one that misbehaves - no error on screen, nothing
    // in the log, just a window that never opens. Reported separately from the
    // FrameXML load above, since a broken addon is a smaller thing than a
    // broken interface.
    std::vector<std::string> addonFailures;
    int refused = 0;
    for (const auto& addon : mgr.getLoadOnDemandAddons()) {
        // The one the client refuses on purpose, refused here too.
        //
        // That decision lives in the LoadAddOn *binding*, so asking the
        // manager directly walks straight past it - and this was loading an
        // addon the client never loads, reporting the seventy-nine globals it
        // has no server behind as though they were gaps. A harness that
        // reaches a state the client cannot is worse than one that reaches
        // less.
        if (addon.addonName == "Blizzard_Calendar") { ++refused; continue; }
        std::string why;
        if (!mgr.loadAddOnByName(addon.addonName, why)) {
            addonFailures.push_back(addon.addonName + " (" +
                                    (why.empty() ? "?" : why) + ")");
        }
    }
    std::printf("== addons: %zu of %zu load-on-demand failed (%d refused as the client does)\n",
                addonFailures.size(),
                mgr.getLoadOnDemandAddons().size() - static_cast<size_t>(refused),
                refused);
    for (const std::string& f : addonFailures) std::printf("   %s\n", f.c_str());

    // The events a login fires, which the interface does a great deal of its
    // setting up on. Without them the frames exist and are half-configured,
    // and that reads as breakage: a chat frame whose windows were never
    // updated has no stored alpha, so FCF_FadeInChatFrame does max(nil, ...)
    // and every click on a chat tab raises - which looks exactly like a real
    // fault in the chat, and is not one.
    //
    // Fired after the addons, so a load-on-demand panel that registered for
    // one of them hears it too.
    const size_t beforeEvents = errors.size();
    // One event, and only this one, because it is the only one that pays.
    //
    // Without it the chat frames are half-configured: no stored alpha, so
    // FCF_FadeInChatFrame does max(nil, ...) and every click on a chat tab
    // raises. That looks exactly like a real fault in the chat and is not one,
    // and it is the sort of false lead that costs an afternoon.
    //
    // VARIABLES_LOADED and PLAYER_ENTERING_WORLD were tried here and taken out
    // again. Both assume a character: paperdollframe does
    // strupper(UnitClass("player")) and pvpbattlegroundframe concatenates
    // UnitFactionGroup("player"), and with nobody logged in those are nil and
    // raise. That is an answer about the absence of a player rather than a
    // fault, and permanent errors in this report would drown the real ones -
    // its whole worth is that a nonzero count means something.
    // Silent through the event burst, as the client is - see
    // LuaEngine::setUiSoundsSuppressed. Without this a headless run counts
    // sounds the client would never have played.
    wowee::addons::LuaEngine::setUiSoundsSuppressed(true);
    mgr.fireEvent("UPDATE_CHAT_WINDOWS");
    wowee::addons::LuaEngine::setUiSoundsSuppressed(false);

    std::printf("== login events: %zu error(s)\n", errors.size() - beforeEvents);
    for (size_t k = beforeEvents; k < errors.size(); ++k) {
        std::printf("   %s\n", errors[k].c_str());
    }

    // Resolve the anchors, so a question about where something ended up has an
    // answer. Nothing drives a render loop here, and without this every frame
    // reports a bottom of zero and a top equal to its own height - which is
    // not a layout, it is the absence of one, and reads as a fault in whatever
    // is being examined.
    //
    // This is the tree's own layout and not the renderer's, so the two passes
    // the renderer runs first are missing: a font string is sized from its text
    // and a tooltip from its lines, and neither has a font here. Frames sized
    // by their anchors and their own dimensions are right; anything whose size
    // comes from text it holds will read as zero.
    // A font, so that a label has a width.
    //
    // ImGui's default atlas is built entirely on the CPU - no device, no
    // window, no backend - and having one turns the renderer's own layout pass
    // from unusable into usable here. That matters more than it sounds: a
    // great deal of FrameXML is positioned against a label's extent rather
    // than a number, and without metrics every one of those labels is zero
    // wide. The auction browse anchors its rarity dropdown to the BOTTOMRIGHT
    // of the "Level Range" caption, so with no font the dropdown lands sixty
    // pixels left of where it belongs, on top of the level boxes - a fault
    // that looks exactly like the real one being investigated and is not it.
    //
    // The typeface is not the game's, so widths are close rather than exact.
    // Close is the difference between "these two frames overlap" being
    // answerable and not.
    ImGui::CreateContext();
    {
        ImGuiIO& io = ImGui::GetIO();
        io.DisplaySize = ImVec2(1920.0f, 1080.0f);
        io.DeltaTime = 1.0f / 60.0f;
        // The game's own faces where they are on disk, so a label measures
        // what it will really measure. Loading a TTF into an atlas is pure
        // ImGui and needs no device, which is the whole reason this is
        // possible here.
        //
        // Without them the substitute is ImGui's built-in face and widths are
        // close rather than exact - close enough to answer "is this frame
        // laid out at all", not close enough to answer "do these two overlap",
        // which is the question that comes up about anything positioned
        // against a caption's right edge.
        int faces = 0;
        for (const char* name : {"frizqt__.ttf", "morpheus.ttf", "skurri.ttf",
                                 "arialn.ttf", "friends.ttf"}) {
            const std::string file = assetPath + "/misc/fonts/" + name;
            if (ImFont* f = io.Fonts->AddFontFromFileTTF(file.c_str(), 16.0f)) {
                wowee::ui::registerInterfaceFace(name, f);
                ++faces;
            }
        }
        if (faces == 0) io.Fonts->AddFontDefault();
        std::printf("== fonts: %d of 5 of the game's own faces\n", faces);
        io.Fonts->Build();
        unsigned char* pixels = nullptr;
        int fw = 0, fh = 0;
        io.Fonts->GetTexDataAsRGBA32(&pixels, &fw, &fh);
        io.Fonts->SetTexID(static_cast<ImTextureID>(1));
        ImGui::NewFrame();
    }

    // The renderer's layout rather than the tree's, so the passes that size a
    // label from its text and a tooltip from its lines run too. Drawing is the
    // other half and is not called: it needs a device.
    // With the assets, without a device.
    //
    // Textures were out of reach here entirely: WidgetRenderer::texture()
    // returns early without a Vulkan context, so nothing could be read and the
    // pass that sizes a region from its own image could not be checked at all.
    // Uploading needs a GPU; asking a file how big it is does not, and that
    // split is now in the renderer.
    //
    // A failure to open the assets is not fatal - every other check here works
    // without them, and this runs on machines with no game data.
    // The client does not open the data path it is given. An expansion that
    // carries its own manifest.json becomes the primary asset source, with the
    // path given here as the fallback behind it - which is how an overlay that
    // replaces models or a DBC for one expansion reaches the client at all.
    // Opening the base path directly, as this used to, reads straight past
    // every such overlay and reports the unmodified game.
    wowee::game::ExpansionRegistry assetExpansions;
    std::string primaryAssetPath = assetPath;
    std::string assetFallbackPath;
    if (assetExpansions.initialize(assetPath) > 0) {
        if (const auto* active = assetExpansions.getActive()) {
            const std::string expansionManifest = active->dataPath + "/manifest.json";
            if (!active->dataPath.empty() &&
                std::filesystem::exists(expansionManifest) &&
                active->dataPath != assetPath) {
                primaryAssetPath = active->dataPath;
                assetFallbackPath = assetPath;
            }
        }
    }

    wowee::pipeline::AssetManager assets;
    // Before initialize, exactly as application.cpp does it - the fallback
    // manifest is loaded here and initialize does not clear it.
    if (!assetFallbackPath.empty()) assets.setBaseFallbackPath(assetFallbackPath);
    const bool haveAssets = assets.initialize(primaryAssetPath);
    if (!haveAssets) {
        std::printf("== assets: none at %s; texture sizes are unavailable\n",
                    primaryAssetPath.c_str());
    } else if (!assetFallbackPath.empty()) {
        std::printf("== assets: %s over %s (expansion '%s')\n",
                    primaryAssetPath.c_str(), assetFallbackPath.c_str(),
                    assetExpansions.getActiveId().c_str());
    }
    wowee::ui::WidgetRenderer widgets;
    widgets.initialize(haveAssets ? &assets : nullptr, nullptr);

    // Layout, then the three passes the client runs off the back of it.
    //
    // Laying out alone was not the frame the client runs. OnShow is fired by
    // LuaEngine::updateVisibility, which application.cpp calls once a frame and
    // which nothing here called - so no handler that fills a panel in ever ran,
    // and every panel measured here was measured empty. That is not a small
    // gap: QuestFrame's whole QUEST_DETAIL path is `panel:Hide(); panel:Show()`
    // and every element it positions is positioned from OnShow.
    //
    // Worse than the gap was what it did to a question asked of this harness.
    // "Does OnShow fire?" answered no for the frame under test *and* for a
    // frame known to be fine, because the pass that fires it was never reached
    // - a clean-looking zero that meant only that nobody had looked.
    //
    // Same order as the client: visibility first, because a panel's OnShow is
    // what fills it in, and the size of what it filled is what the range is
    // then measured from.
    auto relayout = [&mgr, &widgets] {
        if (auto* engine = mgr.getLuaEngine()) {
            widgets.layout(engine->widgets(), 1920.0f, 1080.0f);
            engine->updateVisibility();
            engine->updateSizeChanges();
            engine->updateScrollRanges();
        }
    };
    relayout();

    int raised = 0;
    for (int i = 2; i < argc; ++i) {
        const size_t before = errors.size();
        std::printf("\n== %s\n", argv[i]);
        // --tick:N runs N frames of the interface's own per-frame work rather
        // than evaluating an expression.
        //
        // Everything else here calls a handler directly, which tests the
        // handler and nothing about whether the client would ever reach it.
        // OnUpdate is dispatched from a list, gated on the widget's visible
        // chain, and unhooked after five consecutive failures - none of which
        // a direct call goes near. A frame whose OnUpdate raises looks
        // perfectly healthy when its function is invoked by hand, and is dead
        // for the rest of the session in the running client.
        //
        // Sixteen milliseconds a tick, which is the frame this client aims at,
        // so anything measured in seconds advances at the rate it really would.
        // --player attaches a game handler with a character in it.
        //
        // Every binding that matters starts with getGameHandler(L) and returns
        // at once when it is null, which is what the runner has always had. So
        // a check could reach a handler and never reach the thing the handler
        // asks the client - UnitFactionGroup answers nothing, PickupSpell
        // picks up nothing, and the failures that follow are the harness's,
        // not the interface's.
        //
        // Nothing here talks to a server: the services are all null pointers,
        // which GameHandler accepts, and the character is set directly.
        if (std::strcmp(argv[i], "--player") == 0) {
            static wowee::game::GameServices svc;
            // The real asset manager, so bindings that read a DBC - quest reward
            // XP from QuestXP.dbc, zone names from AreaTable - answer for real
            // rather than the empty they give with no assets behind them.
            if (haveAssets) svc.assetManager = &assets;
            static wowee::game::GameHandler gh(svc);
            constexpr uint64_t kGuid = 0x0000000000000001ull;
            gh.setPlayerGuid(kGuid);
            // Through the character list, because that is where the client
            // reads it from: getPlayerRace and getPlayerClass both go via
            // getActiveCharacter, so setting playerRace_ alone left every one
            // of them answering zero and the whole thing pointless.
            wowee::game::Character ch{};
            ch.guid = kGuid;
            ch.name = "Headless";
            ch.race = wowee::game::Race::HUMAN;
            ch.characterClass = wowee::game::Class::WARRIOR;
            ch.gender = wowee::game::Gender::MALE;
            ch.level = 80;
            gh.charactersRef().clear();
            gh.charactersRef().push_back(ch);
            gh.setActiveCharacterGuid(kGuid);
            gh.playerRaceRef() = wowee::game::Race::HUMAN;
            // Base stats, so the character-sheet stat conversions (crit from
            // agility, etc.) have something to read. str, agi, sta, int, spirit.
            gh.playerStatsArr()[0] = 150;
            gh.playerStatsArr()[1] = 200;  // agility
            gh.playerStatsArr()[2] = 300;
            gh.playerStatsArr()[3] = 50;
            gh.playerStatsArr()[4] = 60;
            gh.serverPlayerLevelRef() = 80;  // the level the game-table lookups use
            gh.playerCombatRatingsRef()[1] = 148;  // CR_DEFENSE_SKILL, for UnitDefense
            gh.playerCombatRatingsRef()[2] = 100;  // CR_DODGE, for GetCombatRatingBonus
            gh.playerCombatRatingsRef()[24] = 100; // CR_ARMOR_PENETRATION, for GetArmorPenetration
            gh.playerManaRegenRef() = 172.5f;        // per second, not casting, for GetManaRegen
            gh.playerManaRegenCastingRef() = 50.0f;  // per second, while casting
            // A few spells, so the spellbook is not empty. getSpellBookTabs
            // rebuilds itself from the known set whenever the count changes,
            // so adding them is all that is needed - and without them
            // PickupSpell resolves slot 1 to nothing and every drag out of the
            // book is a no-op that looks like a broken drag.
            if (auto* sh = gh.getSpellHandler()) {
                for (uint32_t id : {133u, 168u, 116u}) sh->addKnownSpell(id);
            }
            // A spell icon path for every id, so GetSpellTexture answers
            // non-empty. Without it SpellButton_UpdateButton hides the icon and
            // SpellButton_OnDrag's `not IconTexture:IsShown()` guard returns
            // before PickupSpell - a drag that looks broken but is only a
            // spellbook with no icons, which is what the client draws over a
            // realm and the harness cannot. The path need not resolve to pixels;
            // the guard only asks whether the texture is shown.
            gh.setSpellIconPathResolver(
                [](uint32_t) { return std::string("Interface\\Icons\\INV_Misc_QuestionMark"); });
            // A quest in the log, with the text the server sends only on
            // query, so the quest-log detail has something real to lay out.
            // Without it GetQuestLogQuestText answers empty and "the
            // description is blank" cannot be told from "there was no quest".
            if (auto* qh = gh.getQuestHandler()) {
                auto& log = qh->questLogRef();
                if (log.empty()) {
                    wowee::game::QuestHandler::QuestLogEntry q{};
                    q.questId = 375;
                    q.title = "The Chill of Death";
                    q.objectives = "Speak with Apothecary Johaan in Brill.";
                    q.description =
                        "It's so cold, now. The Plague of Undeath crawls "
                        "through my veins, and yet I feel nothing but the "
                        "endless winter of the grave. Seek out Johaan - he "
                        "alone may know what has become of me.";
                    q.zoneOrSort = 130;  // Silverpine Forest
                    q.level = 1;
                    q.rewardXPId = 5;    // QuestXP.dbc row 0 (level 1) col 5 = 80
                    q.rewardHonor = 250;
                    q.rewardTalents = 1;
                    q.rewardArenaPoints = 100;
                    q.rewardTitleId = 6;  // CharTitles.dbc id 6 = "Knight %s"
                    q.factionRewards[0] = {69, 5, 0};  // faction 69, value idx 5 -> 250 rep
                    log.push_back(q);
                    gh.setSelectedQuestLogIndex(1);
                }
            }
            if (auto* engine = mgr.getLuaEngine()) engine->setGameHandler(&gh);
            // And an entity, because a character alone is not a unit.
            //
            // resolveUnit answers out of the EntityManager, so with nothing in
            // it every Unit* binding that goes through it - UnitClass,
            // UnitName, UnitLevel, UnitExists - returned no values at all, and
            // FrameXML did strupper(nil) on the second of them. A click sweep
            // over that reported four raises and every one was the harness's.
            //
            // The fields are the ones those bindings read: race and class live
            // in the low two bytes of UNIT_FIELD_BYTES_0, and the level and
            // health are read straight off the unit.
            auto player = std::make_shared<wowee::game::Player>(kGuid);
            player->setName("Headless");
            player->setLevel(80);
            player->setMaxHealth(1000);
            player->setHealth(1000);
            player->setPosition(0.0f, 0.0f, 0.0f, 0.0f);
            {
                const uint32_t race = static_cast<uint32_t>(wowee::game::Race::HUMAN);
                const uint32_t klass = static_cast<uint32_t>(wowee::game::Class::WARRIOR);
                player->setField(
                    wowee::game::fieldIndex(wowee::game::UF::UNIT_FIELD_BYTES_0),
                    race | (klass << 8));
            }
            gh.getEntityManager().addEntity(kGuid, player);
            std::printf("   attached a game handler: %s, race=%u class=%u "
                        "level=%u\n",
                        ch.name.c_str(),
                        static_cast<unsigned>(gh.getPlayerRace()),
                        static_cast<unsigned>(gh.getPlayerClass()),
                        static_cast<unsigned>(ch.level));
            continue;
        }
        // --hit:X,Y says which frame the client's own hit test lands on.
        //
        // Window pixels like --mouse, so the two agree. Reimplementing the
        // test in Lua to ask this answers a different question: the real one
        // filters on the widget's own `visible`, its clip rect and its hit
        // insets, and a Lua walk over IsVisible() sees none of that. The two
        // disagreeing is itself the finding.
        if (std::strncmp(argv[i], "--hit:", 6) == 0) {
            float hx = 0.0f, hy = 0.0f;
            std::sscanf(argv[i] + 6, "%f,%f", &hx, &hy);
            relayout();
            if (auto* engine = mgr.getLuaEngine()) {
                // Through the client's own conversion, scale and all. A raw
                // flip here answered a different question and made the two
                // disagree - which read as the drop path being broken when it
                // was this line.
                float tx = hx, ty = hy;
                wowee::ui::mouseToTreeSpace(tx, ty, 1080.0f, engine->widgets().uiScale());
                const uint32_t id = engine->widgets().hitTest(tx, ty);
                const auto* w = id ? engine->widgets().get(id) : nullptr;
                std::printf("   hit at %.0f,%.0f -> %s\n", hx, hy,
                            w ? (w->name.empty() ? "(unnamed)" : w->name.c_str())
                              : "nothing");
            }
            continue;
        }
        // --mouse:X,Y,BUTTONS moves the cursor and sets the buttons held.
        //
        // Coordinates are window pixels from the top-left, the way ImGui
        // reports them and the way the client passes them, so a position read
        // off a frame's rect has to be flipped - the widget tree's y grows
        // upward. BUTTONS is any of L, R, M; an empty field is all released.
        //
        // A drag is three of these: down on the source, moved far enough to
        // pass the threshold, then up over the target. Nothing else here can
        // exercise press-move-release, and that is where the drag machinery
        // lives - which frame owns a drag, which frame is offered the drop,
        // and whether either walks up its parents.
        if (std::strncmp(argv[i], "--mouse:", 8) == 0) {
            float mx = 0.0f, my = 0.0f;
            char held[8] = {0};
            std::sscanf(argv[i] + 8, "%f,%f,%7s", &mx, &my, held);
            wowee::addons::LuaEngine::MouseButtons buttons;
            buttons.left   = std::strchr(held, 'L') != nullptr;
            buttons.right  = std::strchr(held, 'R') != nullptr;
            buttons.middle = std::strchr(held, 'M') != nullptr;
            relayout();
            if (auto* engine = mgr.getLuaEngine()) {
                engine->dispatchMouse(mx, my, 1080.0f, buttons);
            }
            std::printf("   mouse at %.0f,%.0f holding '%s'\n", mx, my,
                        held[0] ? held : "nothing");
            if (errors.size() != before) {
                ++raised;
                for (size_t k = before; k < errors.size(); ++k) {
                    std::printf("   %s\n", errors[k].c_str());
                }
            }
            continue;
        }
        // --fire:EVENT sends one event through the engine's own dispatch.
        //
        // Calling a frame's OnEvent by hand tests the handler and nothing
        // about whether the client would reach it - which table it is
        // registered in, whether the name matches, whether anything else
        // listens. This goes the way the client goes.
        if (std::strncmp(argv[i], "--fire:", 7) == 0) {
            relayout();
            // --fire:EVENT|arg1|arg2 - the arguments matter as much as the
            // event. A handler is written against what the event carries, and
            // an event fired empty makes most of them return at their first
            // line: CHAT_MSG_SAY with no message adds nothing to the chat
            // window, which reads exactly like a chat window that is broken.
            // Pipe-separated, because a chat line is full of commas.
            std::string spec = argv[i] + 7;
            std::string event = spec;
            std::vector<std::string> eventArgs;
            if (const size_t bar = spec.find('|'); bar != std::string::npos) {
                event = spec.substr(0, bar);
                size_t at = bar + 1;
                while (at <= spec.size()) {
                    const size_t next = spec.find('|', at);
                    eventArgs.push_back(spec.substr(
                        at, next == std::string::npos ? std::string::npos : next - at));
                    if (next == std::string::npos) break;
                    at = next + 1;
                }
            }
            mgr.fireEvent(event, eventArgs);
            if (errors.size() == before) {
                std::printf("   no error\n");
            } else {
                ++raised;
                for (size_t k = before; k < errors.size(); ++k) {
                    std::printf("   %s\n", errors[k].c_str());
                }
            }
            continue;
        }
        // --lua:CODE runs one chunk through the interface's own Lua state.
        //
        // The direct handlers above each test one seam; a whole flow - show a
        // panel, select a row, let the template lay itself out - is what a real
        // report exercises, and only Lua can drive it the way the client does.
        // ShowUIPanel(QuestLogFrame); QuestLog_SetSelection(1) is the quest log
        // opening onto its first quest, which is exactly the path a "the
        // description is blank" report walks.
        if (std::strncmp(argv[i], "--lua:", 6) == 0) {
            relayout();
            if (auto* engine = mgr.getLuaEngine()) {
                const bool ok = engine->executeString(argv[i] + 6);
                if (ok && errors.size() == before) {
                    std::printf("   ran\n");
                } else {
                    ++raised;
                    if (!ok) std::printf("   %s\n", engine->lastError().c_str());
                    for (size_t k = before; k < errors.size(); ++k) {
                        std::printf("   %s\n", errors[k].c_str());
                    }
                }
            }
            continue;
        }
        // --draw actually paints, into ImGui's draw list and no further.
        //
        // Everything else here settles layout and state; drawing was the one
        // half that could not be reached, and it is where a whole class of
        // fault lives - a frame correct in every property and still not on
        // screen. The chat window holding lit messages and painting none of
        // them is exactly that shape, and no amount of asking from outside
        // could tell it from a healthy one.
        //
        // A device is needed to *present*, not to build a draw list. Textures
        // resolve to nothing without one, which costs the images and nothing
        // else: the geometry, the wrapping, the clipping and every diagnostic
        // along the way run as they do in the client.
        if (std::strcmp(argv[i], "--draw") == 0) {
            relayout();
            if (auto* engine = mgr.getLuaEngine()) {
                widgets.draw(engine->widgets(), 1920.0f, 1080.0f);
                std::printf("   drew the tree\n");
            }
            continue;
        }
        // --drawn:NAME says whether a widget reaches the draw order, and if
        // not, which test dropped it.
        //
        // "It is on screen and I cannot see it" has a dozen causes and they are
        // indistinguishable from Lua: shown, sized, positioned, coloured, and
        // absent. The draw order is where all of them end up, so asking it
        // directly replaces the guessing. Reports the same conditions the
        // builder applies, in the same order.
        if (std::strncmp(argv[i], "--drawn:", 8) == 0) {
            relayout();
            auto* engine = mgr.getLuaEngine();
            if (!engine) { std::printf("   no engine\n"); continue; }
            const auto& tree = engine->widgets();
            const std::string want = argv[i] + 8;
            const wowee::ui::Widget* w = tree.findByName(want);
            if (!w) { std::printf("   no widget named '%s'\n", want.c_str()); continue; }
            bool inOrder = false;
            for (const auto* d : tree.drawOrder()) {
                if (d == w) { inOrder = true; break; }
            }
            const char* why = "it is in the draw order";
            if (!w->visible)               why = "not visible (shown, or an ancestor, or unanchored)";
            else if (w->alpha <= 0.001f)   why = "alpha is zero";
            // The same rule the tree applies in drawOrder(). It is written
            // twice, so an edit box drawing its own text has to be excused in
            // both - this copy said "a container" about a chat box that had
            // just started drawing again.
            else if (w->kind == wowee::ui::WidgetKind::Frame && !w->hasBackdrop &&
                     !w->isStatusBar && !w->isEditBox && w->externalTexture == 0 &&
                     !(w->isMessageFrame && !w->messages.empty()) &&
                     !(w->isTooltip && !w->tooltipLines.empty()))
                why = "a frame with nothing of its own to paint (no backdrop, no "
                      "bar, no messages, no tooltip lines) - a container";
            else if (w->rectW <= 0.0f || w->rectH <= 0.0f) why = "no width or no height";
            else if (w->kind == wowee::ui::WidgetKind::Texture &&
                     w->texturePath.empty() && !w->solidColor &&
                     w->externalTexture == 0)
                why = "a texture with no image and no colour";
            else if (w->kind == wowee::ui::WidgetKind::FontString && w->text.empty())
                why = "a font string with no text";
            else if (w->buttonArt != wowee::ui::ButtonArt::None)
                why = "button art for a state the button is not in";
            std::printf("   %s: %s - %s\n", want.c_str(),
                        inOrder ? "DRAWN" : "not drawn", why);
            std::printf("      visible=%d alpha=%.2f rect=(%.0f,%.0f %.0fx%.0f) "
                        "messages=%zu backdrop=%d\n",
                        w->visible ? 1 : 0, w->alpha, w->left, w->bottom,
                        w->rectW, w->rectH, w->messages.size(),
                        w->hasBackdrop ? 1 : 0);
            continue;
        }
        // --messages dumps every message frame: how many lines it holds, how
        // long a line lives, and what alpha each is at.
        //
        // A chat window with nothing in it and a chat window whose lines have
        // all faded to nothing look identical from Lua - GetNumMessages counts
        // both, because a faded line is still in the history and comes back
        // when the frame is scrolled. The alpha is the only thing that
        // separates "nothing was said" from "everything said has gone".
        if (std::strcmp(argv[i], "--messages") == 0) {
            relayout();
            auto* engine = mgr.getLuaEngine();
            if (!engine) { std::printf("   no engine\n"); continue; }
            const auto& tree = engine->widgets();
            int frames = 0;
            for (uint32_t id = 1; id < tree.size(); ++id) {
                const auto* w = tree.get(id);
                if (!w || w->id == 0 || !w->isMessageFrame) continue;
                if (w->messages.empty()) continue;
                ++frames;
                int lit = 0;
                for (const auto& m : w->messages) if (m.color[3] > 0.0f) ++lit;
                std::printf("   %-22s %zu line(s), %d lit, duration %.0fs, "
                            "fade %.0fs, scroll %d\n",
                            w->name.empty() ? "(unnamed)" : w->name.c_str(),
                            w->messages.size(), lit, w->messageDuration,
                            w->messageFadeDuration, w->messageScroll);
                int shown = 0;
                for (const auto& m : w->messages) {
                    if (++shown > 3) break;
                    std::printf("      alpha %.2f age %.1fs  %.60s\n",
                                m.color[3], m.age, m.text.c_str());
                }
            }
            std::printf("   %d message frame(s) holding anything\n", frames);
            continue;
        }
        // --offscreen names content that is on screen in every sense except
        // where it is.
        //
        // "Off page and unreadable" is how this gets reported. A frame shown,
        // sized and filled in, sitting entirely past an edge, is invisible in
        // exactly the way a broken one is - and every property reads back
        // correct, because being outside the screen is not a property.
        //
        // Wholly outside, not merely overhanging: the action bar's end caps and
        // several borders hang off deliberately, and a frame half on screen is
        // still being read.
        if (std::strcmp(argv[i], "--offscreen") == 0) {
            relayout();
            auto* engine = mgr.getLuaEngine();
            if (!engine) { std::printf("   no engine\n"); continue; }
            const auto& tree = engine->widgets();
            const float sw = 1920.0f / tree.uiScale();
            const float sh = 1080.0f / tree.uiScale();
            int found = 0, onscreen = 0;
            for (uint32_t id = 1; id < tree.size(); ++id) {
                const auto* w = tree.get(id);
                if (!w || w->id == 0 || !w->visible) continue;
                const bool draws =
                    (w->kind == wowee::ui::WidgetKind::FontString && !w->text.empty()) ||
                    (w->kind == wowee::ui::WidgetKind::Texture && !w->texturePath.empty());
                if (!draws || w->rectW <= 0.0f || w->rectH <= 0.0f) continue;
                ++onscreen;
                const bool outside =
                    w->left + w->rectW <= 0.0f || w->left >= sw ||
                    w->bottom + w->rectH <= 0.0f || w->bottom >= sh;
                if (!outside) continue;
                ++found;
                if (found <= 25) {
                    std::printf("   offscreen: %-34s at (%.0f,%.0f %.0fx%.0f) "
                                "on a %.0fx%.0f screen\n",
                                w->name.empty() ? "(unnamed)" : w->name.c_str(),
                                w->left, w->bottom, w->rectW, w->rectH, sw, sh);
                }
            }
            if (found > 25) std::printf("   ... and %d more\n", found - 25);
            std::printf("   %d offscreen, of %d drawing\n", found, onscreen);
            continue;
        }
        // --missingart names textures whose file this install does not carry.
        //
        // A path that resolves to nothing is a blank where an icon should be,
        // and nothing reports it: the draw substitutes nothing and carries on,
        // so it looks exactly like art that is meant to be absent. Only
        // askable now that the harness can read the assets.
        //
        // Distinct paths rather than regions, because one missing file is
        // usually many regions - every action button shares a border.
        if (std::strcmp(argv[i], "--missingart") == 0) {
            relayout();
            auto* engine = mgr.getLuaEngine();
            if (!engine) { std::printf("   no engine\n"); continue; }
            const auto& tree = engine->widgets();
            std::set<std::string> seen, missing;
            for (uint32_t id = 1; id < tree.size(); ++id) {
                const auto* w = tree.get(id);
                if (!w || w->id == 0 || w->kind != wowee::ui::WidgetKind::Texture) continue;
                if (w->texturePath.empty()) continue;
                if (!seen.insert(w->texturePath).second) continue;
                float tw = 0.0f, th = 0.0f;
                if (!widgets.artResolves(w->texturePath, tw, th)) {
                    missing.insert(w->texturePath);
                }
            }
            int shown = 0;
            for (const std::string& p : missing) {
                if (++shown > 25) break;
                std::printf("   missing art: %s\n", p.c_str());
            }
            if (missing.size() > 25) {
                std::printf("   ... and %zu more\n", missing.size() - 25);
            }
            // Both numbers again: nothing missing out of nothing looked at is
            // not the same answer as nothing missing out of hundreds.
            std::printf("   %zu missing, of %zu distinct paths\n",
                        missing.size(), seen.size());
            continue;
        }
        // --unsized names content that is on screen, has something to show, and
        // has no room to show it in.
        //
        // A region with no width or no height is not drawn at all. Unlike a
        // clip, nothing about it looks wrong from Lua: it is shown, it has its
        // text or its texture, and every property reads back correctly. The
        // only thing missing is the one number that decides whether any of it
        // reaches the screen.
        //
        // Empty text is not interesting - most of the interface is labels
        // waiting for data - so only regions that have something to say and
        // nowhere to say it are counted.
        if (std::strcmp(argv[i], "--unsized") == 0) {
            relayout();
            auto* engine = mgr.getLuaEngine();
            if (!engine) { std::printf("   no engine\n"); continue; }
            const auto& tree = engine->widgets();
            int found = 0, carrying = 0;
            for (uint32_t id = 1; id < tree.size(); ++id) {
                const auto* w = tree.get(id);
                if (!w || w->id == 0 || !w->visible) continue;
                const bool hasSomething =
                    (w->kind == wowee::ui::WidgetKind::FontString && !w->text.empty()) ||
                    (w->kind == wowee::ui::WidgetKind::Texture && !w->texturePath.empty());
                if (!hasSomething) continue;
                ++carrying;
                if (w->rectW > 0.0f && w->rectH > 0.0f) continue;
                ++found;
                if (found <= 25) {
                    std::printf("   unsized: %-34s %s  %.0fx%.0f\n",
                                w->name.empty() ? "(unnamed)" : w->name.c_str(),
                                w->kind == wowee::ui::WidgetKind::FontString
                                    ? "text " : "image",
                                w->rectW, w->rectH);
                }
            }
            if (found > 25) std::printf("   ... and %d more\n", found - 25);
            // The second number for the same reason --clipped carries one: a
            // zero here is also what "nothing on screen has anything to show"
            // looks like, and that would mean the run proved nothing.
            std::printf("   %d unsized, of %d carrying something\n",
                        found, carrying);
            continue;
        }
        // --bind:KEY presses a key the way the client's binding dispatch does.
        //
        // Calling RunBinding by hand tests the script and nothing about whether
        // a key press would ever reach it, which is the part that was missing
        // entirely: 273 binding scripts loaded and none reachable.
        if (std::strncmp(argv[i], "--bind:", 7) == 0) {
            relayout();
            const char* name = argv[i] + 7;
            // A letter arrives from SDL in lower case; the binding tables are
            // upper. Only single letters need it, which is all this takes.
            int sym = 0;
            if (std::strlen(name) == 1) {
                sym = std::tolower(static_cast<unsigned char>(name[0]));
            } else if (std::strcmp(name, "ESCAPE") == 0) {
                sym = 27;
            } else if (std::strcmp(name, "SPACE") == 0) {
                sym = ' ';
            }
            if (sym == 0) {
                std::printf("   --bind: only single letters, ESCAPE and SPACE\n");
                continue;
            }
            bool ran = false;
            if (auto* engine = mgr.getLuaEngine()) {
                ran = engine->dispatchBindingKey(sym, false, false, false, true);
            }
            // Which command the key holds, said alongside the outcome.
            //
            // "Nothing happened" has two quite different causes here - no
            // command on the key at all, or a command the client performs
            // itself and the dispatch therefore declines - and reporting both
            // the same way would make a decline read as a missing binding.
            std::string command;
            if (auto* engine = mgr.getLuaEngine()) {
                command = engine->bindingCommandFor(sym, false, false, false);
            }
            if (ran) {
                std::printf("   %s -> ran %s\n", name,
                            command.empty() ? "(unnamed)" : command.c_str());
            } else if (command.empty()) {
                std::printf("   %s -> nothing bound to this key\n", name);
            } else {
                std::printf("   %s -> declined %s; the client performs it\n",
                            name, command.c_str());
            }
            if (errors.size() != before) {
                ++raised;
                for (size_t k = before; k < errors.size(); ++k) {
                    std::printf("   %s\n", errors[k].c_str());
                }
            }
            continue;
        }
        // --keyfocus names the frame a key press would be handed to.
        //
        // dispatchFrameKey gives the key to the topmost visible frame that
        // enabled the keyboard, and that frame swallows it unless it asked for
        // propagation. Everything in FrameXML that declares a key handler is a
        // dialog hidden until it is wanted, so at rest this should name nothing
        // - and if it names something that is always on screen, that frame is
        // eating every key in the game and the fault is here rather than in
        // whatever the key was supposed to do.
        if (std::strcmp(argv[i], "--keyfocus") == 0) {
            relayout();
            auto* engine = mgr.getLuaEngine();
            if (!engine) { std::printf("   no engine\n"); continue; }
            const auto& tree = engine->widgets();
            const wowee::ui::Widget* best = nullptr;
            int listening = 0;
            for (uint32_t id = 1; id < tree.size(); ++id) {
                const auto* w = tree.get(id);
                if (!w || w->id == 0 || !w->keyboardEnabled || !w->visible) continue;
                ++listening;
                if (!best) { best = w; continue; }
                if (w->effStrata > best->effStrata ||
                    (w->effStrata == best->effStrata && w->effLevel >= best->effLevel)) {
                    best = w;
                }
            }
            if (!best) {
                std::printf("   no frame is listening; the key falls through\n");
            } else {
                std::printf("   %d listening; key goes to '%s' and is %s\n",
                            listening,
                            best->name.empty() ? "(unnamed)" : best->name.c_str(),
                            best->propagateKeys ? "passed on" : "swallowed");
            }
            continue;
        }
        // --clipped names content that is drawn and then clipped entirely away.
        //
        // Everything under a scroll frame is clipped to it, and a clip that
        // does not overlap what it clips removes it from the screen leaving no
        // other trace: the region is shown, sized, positioned, and simply not
        // there. That is what a blank parchment with working buttons looks
        // like, and it is how the quest dialog hid - its text sat at x=540 on
        // a scroll frame spanning 23 to 323.
        //
        // Only worth running once panels actually build themselves, which
        // means after OnShow works. Nothing here could have found that fault,
        // because with no OnShow the panel had no content to be clipped.
        if (std::strcmp(argv[i], "--clipped") == 0) {
            relayout();
            auto* engine = mgr.getLuaEngine();
            if (!engine) { std::printf("   no engine\n"); continue; }
            const auto& tree = engine->widgets();
            int found = 0;
            int clipped = 0;
            for (uint32_t id = 1; id < tree.size(); ++id) {
                const auto* w = tree.get(id);
                if (!w || w->id == 0 || w->clipTo == 0) continue;
                ++clipped;
                if (!w->visible) continue;
                // Only things that draw. A container frame outside the window
                // is ordinary - a scroll child is routinely taller than what
                // shows it - but a label or an image is content nobody sees.
                const bool draws =
                    (w->kind == wowee::ui::WidgetKind::FontString && !w->text.empty()) ||
                    (w->kind == wowee::ui::WidgetKind::Texture && w->rectW > 0.0f);
                if (!draws) continue;
                const auto* clip = tree.get(w->clipTo);
                if (!clip) continue;
                // Scrolled out of view is normal and expected on the vertical
                // axis, so only a miss on *both* axes counts: that cannot be
                // scrolled back into sight and means the content was placed
                // somewhere its own window can never reach.
                const bool overlapX = w->left < clip->left + clip->rectW &&
                                      w->left + w->rectW > clip->left;
                const bool overlapY = w->bottom < clip->bottom + clip->rectH &&
                                      w->bottom + w->rectH > clip->bottom;
                if (overlapX && overlapY) continue;
                if (!overlapX && !overlapY) {
                    ++found;
                    std::printf("   clipped away: '%s' at (%.0f,%.0f %.0fx%.0f) "
                                "by '%s' at (%.0f,%.0f %.0fx%.0f)\n",
                                w->name.empty() ? "(unnamed)" : w->name.c_str(),
                                w->left, w->bottom, w->rectW, w->rectH,
                                clip->name.empty() ? "(unnamed)" : clip->name.c_str(),
                                clip->left, clip->bottom, clip->rectW, clip->rectH);
                } else if (!overlapX) {
                    // Sideways is the telling one. A scroll frame scrolls up
                    // and down; nothing puts content back on screen that is
                    // off to the side of its own window.
                    ++found;
                    std::printf("   clipped away sideways: '%s' x %.0f..%.0f "
                                "outside '%s' x %.0f..%.0f\n",
                                w->name.empty() ? "(unnamed)" : w->name.c_str(),
                                w->left, w->left + w->rectW,
                                clip->name.empty() ? "(unnamed)" : clip->name.c_str(),
                                clip->left, clip->left + clip->rectW);
                }
            }
            // Both numbers, because they fail in opposite directions. Zero
            // clipped away is also what "nothing is clipped at all" looks
            // like, and that would be the worse bug of the two - every scroll
            // frame spilling its whole child across the window.
            std::printf("   %d clipped away, of %d clipped to something\n",
                        found, clipped);
            continue;
        }
        if (std::strncmp(argv[i], "--tick:", 7) == 0) {
            const int ticks = std::atoi(argv[i] + 7);
            relayout();
            // Move the shared clock forward with the tick loop, not just the
            // per-frame elapsed: FadingFrame and cooldown sweeps read GetTime
            // (the wall clock), which otherwise stands still through a
            // synchronous tick run and freezes every fade the harness tries to
            // age. Advance first, so this tick's OnUpdate sees the new "now".
            for (int t = 0; t < ticks; ++t) {
                wowee::core::advanceTestClock(1.0 / 60.0);
                mgr.update(1.0f / 60.0f);
            }
            std::printf("   ticked %d frame(s)\n", ticks);
            if (errors.size() != before) {
                ++raised;
                for (size_t k = before; k < errors.size(); ++k) {
                    std::printf("   %s\n", errors[k].c_str());
                }
            }
            continue;
        }
        // Before each one, not once after the load. An expression that opens a
        // panel changes where things are, and the expression that measures it
        // is the next one along - laying out only at the start meant every
        // measurement described the interface as it was before anything had
        // been asked of it. The client lays out every frame; this is the same
        // thing at the only granularity there is here.
        relayout();
        mgr.runInterfaceCommand(argv[i]);
        if (errors.size() == before) {
            std::printf("   no error\n");
        } else {
            ++raised;
            for (size_t k = before; k < errors.size(); ++k) {
                std::printf("   %s\n", errors[k].c_str());
            }
        }
    }
    return raised > 100 ? 100 : raised;
}
