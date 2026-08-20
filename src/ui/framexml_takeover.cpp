#include "ui/framexml_takeover.hpp"

#include "core/logger.hpp"

#include <array>
#include <atomic>
#include <cstdlib>
#include <mutex>
#include <set>
#include <string>

namespace wowee::ui {

namespace {

/// `clientDraws` is false once this client's own version of an element has
/// been deleted and FrameXML is the only one left that draws it.
///
/// The unaccounted-element report exists to catch an element that is neither
/// handed over nor suppressed, because that one is on screen twice. An
/// element nothing here draws can never be, so reporting it would be a false
/// alarm - and the report is only worth reading while every line of it is
/// real. Four lines of it were not: the action bar, the stance bar and the two
/// thin bars are the only elements not owned by default, and action_bar_panel
/// was deleted whole, so each warned on every start about a window that had
/// not existed for some time.
///
/// It is also what says whether an element may have a suppression row. Hiding
/// FrameXML's frame is only right while something else is drawing, so the two
/// have to move together - see kSuppress, and handover_halves_check, which
/// fails if they disagree. Seventeen rows outlived their element before that
/// rule existed.
///
/// Five elements are left drawing: the minimap, the world map, the zone text,
/// the taxi picker and the settings panel behind the game menu. The first four
/// are shared surfaces, where this client draws into a frame FrameXML owns
/// rather than beside it, so this flag has no last one to turn off.
struct Entry {
    UiElement element;
    std::string_view name;
    bool clientDraws = true;
};

// One row per element, and the only place a name is written down.
constexpr std::array<Entry, 52> kElements{{
    {UiElement::PlayerFrame,  "playerframe", false},
    {UiElement::TargetFrame,  "targetframe", false},
    {UiElement::PetFrame,     "petframe", false},
    {UiElement::FocusFrame,   "focusframe", false},
    {UiElement::ActionBar,    "actionbar", false},
    {UiElement::StanceBar,    "stancebar", false},
    {UiElement::BagBar,       "bagbar", false},
    {UiElement::MicroMenu,    "micromenu", false},
    {UiElement::XpBar,        "xpbar", false},
    {UiElement::RepBar,       "repbar", false},
    {UiElement::CastBar,      "castbar", false},
    {UiElement::Minimap,      "minimap"},
    {UiElement::Chat,         "chat", false},
    {UiElement::QuestTracker, "questtracker", false},
    {UiElement::WorldMap,     "worldmap"},
    {UiElement::CharacterFrame, "characterframe", false},
    {UiElement::Bags,         "bags", false},
    {UiElement::Spellbook,    "spellbook", false},
    {UiElement::QuestLog,     "questlog", false},
    {UiElement::QuestGiver,   "questgiver", false},
    {UiElement::Gossip,       "gossip", false},
    {UiElement::Mail,         "mail", false},
    {UiElement::Vendor,       "vendor", false},
    {UiElement::Loot,         "loot", false},
    {UiElement::Bank,         "bank", false},
    {UiElement::PartyFrames,  "partyframes", false},
    {UiElement::Social,       "social", false},
    {UiElement::TradeSkill,   "tradeskill", false},
    {UiElement::ClassTrainer, "classtrainer", false},
    {UiElement::AuctionHouse, "auctionhouse", false},
    {UiElement::GuildBank,    "guildbank", false},
    {UiElement::Inspect,      "inspect", false},
    {UiElement::DungeonFinder, "dungeonfinder", false},
    {UiElement::Petition,     "petition", false},
    {UiElement::Buffs,        "buffs", false},
    {UiElement::Durability,   "durability", false},
    {UiElement::ZoneText,     "zonetext"},
    {UiElement::Trade,        "trade", false},
    {UiElement::ReadyCheck,   "readycheck", false},
    {UiElement::RaidWarning,  "raidwarning", false},
    {UiElement::Dialogs,      "dialogs", false},
    {UiElement::Achievements, "achievements", false},
    {UiElement::BarberShop,   "barbershop", false},
    {UiElement::Taxi,         "taxi"},
    {UiElement::Stable,       "stable", false},
    {UiElement::Book,         "book", false},
    {UiElement::GameMenu,          "gamemenu"},
    {UiElement::Help,              "help", false},
    {UiElement::BattlegroundScore, "bgscore", false},
    {UiElement::Totems,       "totems", false},
    {UiElement::Talents,      "talents", false},
    {UiElement::UiErrors,     "uierrors", false},
}};

/// Parsed once. An unknown name is reported rather than dropped: a typo would
/// otherwise read as a replacement that quietly did not happen.
const std::set<std::string>& requested() {
    static const std::set<std::string> names = [] {
        std::set<std::string> out;
        const char* raw = std::getenv("WOWEE_FRAMEXML_UI");

        // What this branch is working on, handed over without being asked.
        //
        // Every one of these has been seen drawing correctly: the player frame
        // with its art, portrait and bars; the minimap with the real map
        // inside its ring; the character sheet with the model in it; the
        // bottom bar. Requiring a flag to see them means every test run begins
        // by remembering the flag, and a run without it silently tests the old
        // interface instead. Naming any element in the environment replaces
        // this list rather than adding to it, so a single element can still be
        // looked at on its own.
        // The set a run gets when nothing is named. All of them are on screen
        // the whole time, which is why they were promoted first: a fault shows
        // immediately rather than waiting for the right NPC.
        static const auto defaults = [] {
            return std::set<std::string>{
                "playerframe", "targetframe", "minimap",
                "mainmenubar", "characterframe", "bags", "castbar",
                "spellbook", "petframe", "focusframe", "buffs", "durability",
                "zonetext", "dialogs",
                // Promoted once a panel that fails to build hands itself back.
                // Until then the cost of being wrong about one of these was
                // the whole element: this client's version hidden, FrameXML's
                // absent, and nothing on screen. Now the worst case is a log
                // line and the old window.
                //
                // These four first because they are the ones a fault shows on
                // immediately, rather than waiting for the right NPC - the
                // same reason the fourteen above went first.
                //
                //   bagbar, micromenu - the bag buttons and the micro buttons.
                //     They sit on the main bar, which FrameXML has drawn all
                //     along, so this finishes a bar that was half handed over.
                //   uierrors - the red error text. Its one unfired event,
                //     SYSMSG, has no opcode behind it anywhere; what the frame
                //     is for is UI_ERROR_MESSAGE, raised from eighty sites.
                //   raidwarning - both its events are fired.
                //
                // WOWEE_FRAMEXML_UI names the whole set, so any of these can
                // be dropped by listing the others.
                "bagbar", "micromenu", "uierrors", "raidwarning",
                // The interaction loop, on the same reasoning: these are the
                // windows a player opens dozens of times an hour, so a fault
                // in one shows on the next corpse or the next NPC rather than
                // waiting to be stumbled on.
                //
                //   loot        every corpse
                //   gossip      every NPC with something to say
                //   questgiver  every quest taken or handed in
                //   partyframes every group
                //
                // Each is clean in the readiness report, gated where this
                // client draws it, absent from the ungated-draw list, and has
                // a check row whose frames the promised-frames sweep resolves.
                "loot", "gossip", "questgiver", "partyframes",
                // The other half of the interaction loop. Hit less often than
                // a corpse, but every one of them daily, and each is a window
                // whose failure is obvious the moment it is opened rather than
                // something that degrades quietly.
                //
                //   vendor  every merchant
                //   mail    every mailbox
                //   bank    every visit
                //   trade   every exchange with another player
                //
                // The unfired events on three of them are all accounted for in
                // the readiness report: mail's two are the refund lock behind a
                // per-item timer this client is never sent, trade's carry a
                // single slot where this client only ever learns of a whole
                // side, and vendor's shares a branch with PLAYER_MONEY, which
                // is fired.
                "vendor", "mail", "bank", "trade",
                // The occasional windows. None is on screen often, but each
                // opens whole or not at all, so a fault is unmistakable the
                // first time one is reached - and none of the nine has an
                // unfired event or an unanswered call that is not accounted
                // for in the readiness report.
                //
                //   bgscore, book, gamemenu, help, readycheck,
                //   social, stable, taxi, totems
                //
                // Talents is not among them: Blizzard_TalentUI is
                // load-on-demand, so its frames do not exist until the panel
                // is opened and the safety net cannot tell that from a failure
                // to build.
                "bgscore", "book", "gamemenu", "help", "readycheck",
                "social", "stable", "taxi", "totems",
                // The panels that arrive with an addon, now that the net can
                // tell "nobody has opened it" from "it did not build" - the
                // addon has loaded and the frame is still absent.
                //
                //   achievements auctionhouse barbershop classtrainer
                //   guildbank inspect talents tradeskill
                //
                // The dungeon finder joins them without being one: LFDFrame.xml
                // is in framexml.toc, so its panel is built at load and the net
                // covered it all along.
                "achievements", "auctionhouse", "barbershop", "classtrainer",
                "guildbank", "inspect", "talents", "tradeskill",
                "dungeonfinder",
                // The last three, and the ones held back longest - not for
                // readiness, but because this client draws all three well and
                // has drawn them all along, so "builds but wrong" costs more
                // here than anywhere else.
                //
                //   chat          back, and this time with the links working.
                //                 A click is hit-tested against the link rects
                //                 the draw pass records, the handler is looked
                //                 for up the parent chain because FrameXML puts
                //                 it on the chat frame rather than the font
                //                 string, and SetItemRef and ItemRefTooltip are
                //                 both FrameXML's own.
                //   questlog      held back with the world map on the reading
                //                 that it drew a second map. It draws no map.
                //   questtracker  its two remaining names are AchievementFrame
                //                 internals, reached only after that panel has
                //                 been shown, which loads the addon that
                //                 defines them.
                //
                // The world map is below, and it is the last one in.
                "chat", "questlog", "questtracker",
                // The world map, which was the last element outside the
                // defaults and is not a replacement at all - it is a seam.
                //
                // The reading that held it back was that FrameXML would draw a
                // second map over this client's own. Both do draw: FrameXML
                // fills WorldMapDetailTile1..12 and this client's renderer
                // loads the same twelve, from the same
                // Interface\WorldMap\<folder>\<folder>N.blp. So the question
                // was never whether two maps are drawn but which is seen, and
                // the answer is in the draw lists. The widget renderer draws
                // into ImGui's BACKGROUND list, deliberately, so this client's
                // interface stays on top while the two coexist - and this
                // client's map is an ImGui window. It covers FrameXML's tiles
                // exactly where the two overlap, which is the detail frame,
                // and application.cpp hands it that frame's rect so the
                // overlap is exact.
                //
                // What that leaves is the arrangement the rect hand-off was
                // written for: FrameXML owns the window, the chrome, the zone
                // dropdown and the open/close, and this client owns the map
                // surface inside it - with the party dots, taxi nodes, quest
                // POIs and player marker it already draws, none of which are
                // gated on ownership. game_screen_hud reads the presence of
                // the rect as the statement that the map is wanted, in place
                // of its own flag, and both the M key and the micro-menu
                // button already route to ToggleFrame(WorldMapFrame).
                //
                // THE LAYER INVENTORY, WHICH IS WHAT THIS SEAM COSTS
                //
                // Everything FrameXML draws inside WorldMapDetailFrame is
                // behind this client's map window and cannot be seen, so every
                // one of those layers has to be drawn on this side or it does
                // not exist. Enumerated 2026-08-05 against the frames
                // worldmapframe.xml declares, so the next gap is a diff:
                //
                //   drawn here     corpse, death release, party dots, quest
                //                  POIs, taxi nodes, rares, chests, the
                //                  exploration mask, the player marker, the
                //                  zone highlight - and battleground team
                //                  positions, which were the one layer nobody
                //                  had checked and are drawn here now.
                //   drawn here    the battleground flag carriers, which the
                //                  same packet as the team positions carries
                //                  in a second block - count, then guid and
                //                  x/y each. This note used to say no packet
                //                  carried them; the parser reads both blocks
                //                  and tags the second, and the map and the
                //                  minimap colour a carrier apart from a
                //                  team-mate by that tag. WorldMapFlag1 and 2
                //                  are FrameXML's own frames for it and stay
                //                  behind this client's map like the rest.
                //   nothing to draw
                //                  WorldMapBlobFrame, the shaded quest areas:
                //                  this client's quest POIs are points, and a
                //                  blob needs the polygon the server sends
                //                  with them, which is not parsed.
                //                  Vehicles, which are absent generally.
                //   covered        WorldMapPing follows the player's own
                //                  marker rather than a party ping, and this
                //                  client draws that marker.
                //
                // The trap is that binding the stub looks like the fix and is
                // not: GetNumBattlefieldPositions feeds WorldMapRaid1..40, and
                // those frames cannot be seen from behind the map.
                //
                // Its two unfired events are accounted for:
                // WORLD_MAP_NAME_UPDATE is registered with no branch to handle
                // it, and CLOSE_WORLD_MAP is the server telling the map to
                // shut, which nothing here sends - the key closes it through
                // the same toggle that opens it.
                "worldmap",
                // The charter windows, found by reading the unaccounted-frame
                // sweep against what this client fires rather than against
                // what it draws. Every global the four files call is bound,
                // including the ones a charter cannot be bought or turned in
                // without.
                "petition"};
        }();

        if (!raw || !*raw) {
            out = defaults;
            LOG_WARNING("FrameXML is drawing the branch defaults; "
                        "set WOWEE_FRAMEXML_UI to choose, 'candidates' to add "
                        "the ones not yet seen drawing, or 'none' for this "
                        "client's own interface");
            return out;
        }

        std::string value(raw);
        size_t start = 0;
        while (start <= value.size()) {
            const size_t comma = value.find(',', start);
            std::string one = value.substr(
                start, comma == std::string::npos ? std::string::npos : comma - start);
            const size_t b = one.find_first_not_of(" \t");
            const size_t e = one.find_last_not_of(" \t");
            one = (b == std::string::npos) ? std::string() : one.substr(b, e - b + 1);
            if (!one.empty()) out.insert(one);
            if (comma == std::string::npos) break;
            start = comma + 1;
        }

        // "candidates" - the defaults plus every element the readiness report
        // finds clean on both counts: every global its code calls is answered,
        // and every event its frames register for is either sent or verified as
        // correctly absent.
        //
        // Clean is not the same as seen working. These are windows that open on
        // an interaction rather than sitting on screen, so a fault in one waits
        // for the right NPC and then blocks that NPC - which is exactly why
        // they are behind a word instead of in the defaults. Naming this is how
        // a run tests the batch without typing thirty names.
        //
        // The list below is generated: tools/framexml_element_readiness.py
        // prints it in this exact shape at the end of its run. Regenerate and
        // paste rather than editing by hand, or it drifts the first time an
        // element goes clean and nobody thinks to come here.
        //
        // The report leaves the four held out below out of what it prints, so
        // a regenerate-and-paste no longer puts them back. It used to print
        // them, which is how "keybindings" survived the edit that removed its
        // two neighbours.
        //
        // "social" was held out for thirty unbound Battle.net names, on the
        // reading that they sit behind BNFeaturesEnabled() and so are probably
        // never reached. They do not, and the caution was right to distrust
        // "probably": IgnoreList_Update opens with GetNumIgnores,
        // BNGetNumBlocked and BNGetNumBlockedToons on three consecutive lines
        // with nothing between them, so the ignore tab raised on the second of
        // the three. FriendsFrameTooltip_Show and FriendsFriendsList_Update are
        // unguarded the same way.
        //
        // All thirty are bound now, and answer what a client with no
        // Battle.net has: no friends, no blocks, nil rows, and verbs that do
        // nothing. tools/framexml_unbound_globals.py reports friendsframe
        // clean, which is the check this note used to ask for.
        //
        // "dungeonfinder" joins them. No missing call, and four of its five
        // unfired events are correctly absent: the source of
        // LFG_OPEN_FROM_GOSSIP is STATUS_NEVER in AzerothCore and never sent,
        // UPDATE_LFG_LIST belongs to the raid browser whose three search
        // packets are read and dropped here by decision - so this client's own
        // browser is as empty as FrameXML's would be - LFG_ROLE_UPDATE
        // refreshes role checkboxes that are client state, and
        // VOTE_KICK_REASON_NEEDED needs a message this client is not sent.
        //
        // "questtracker" joins them, and with it the last of the three held
        // out for drawing a second map. WatchFrame draws no map: it calls the
        // quest POI part of that API to place markers, and every one of those
        // is answered now. Every event it registers is fired, its whole
        // quest-watch API is bound, and its two unbound names are achievement
        // addon internals that exist once that addon loads - the same two
        // already settled under "achievements".
        //
        // "uierrors" joins them. Its one unfired event is SYSMSG, which
        // nothing backs - no opcode here produces one. What the frame is
        // actually for is UI_ERROR_MESSAGE, and addUIError raises that from
        // eighty sites now that the refusals in the social, inventory, spell,
        // quest, chat, combat and movement handlers go through it instead of
        // writing to chat alone.
        //
        // "raidwarning" joins them. Three files, no missing call, no unfired
        // event: CHAT_MSG_RAID_WARNING and CHAT_MSG_RAID_BOSS_EMOTE are both
        // sent. It sat outside both tiers on a note saying the first was not
        // fired - see the suppression entry for why that note was wrong.
        //
        // "trade" joins them. Its two unfired events are correctly unfired:
        // TRADE_PLAYER_ITEM_CHANGED and TRADE_TARGET_ITEM_CHANGED each carry
        // one slot, and this client never learns of one slot changing.
        // SMSG_TRADE_STATUS_EXTENDED carries a whole side at once, TRADE_UPDATE
        // is fired from it, and that branch calls TradeFrame_Update - a redraw
        // of every slot. The third, TRADE_POTENTIAL_BIND_ENCHANT, has a
        // commented-out body in FrameXML itself.
        //
        // "readycheck" joins them too, on a smaller footing: two files, no
        // missing call, and READY_CHECK, READY_CHECK_CONFIRM and
        // READY_CHECK_FINISHED are all fired. Its window is behind a gate in
        // dialog_manager and ReadyCheckFrame is suppressed until the element is
        // owned, so the swap is the whole change.
        //
        // "questlog" joins them. It was held back with the world map and the
        // quest tracker, on the reading that handing any of the three over
        // draws a second map over this client's own - and it draws no map. What
        // it did was call the world map API, which was unbound then and is
        // bound now, and the readiness report finds nothing missing in
        // questlogframe at all. Its own window is already gated, its
        // suppression entry lifts itself the moment the element is owned, and
        // the L key reaches ToggleFrame(QuestLogFrame) through the route table.
        //
        // "keybindings", "macro" and "timemanager" also come out, for a
        // duller reason: they are not elements. The readiness report scores
        // them because their FrameXML files have no missing calls, but there
        // is no UiElement of that name, so putting them here only produced
        // three "no element called" warnings on every run that named
        // candidates. Nothing suppresses those panels and nothing hands them
        // over; making them real means adding an enum entry and a
        // frameXmlOwns guard around this client's own version first.
        //
        // "worldmap" joins them, and it was the last element outside both
        // tiers. It was held back on the reading that handing it over would
        // draw a second map over this client's own, which is the same reason
        // the quest log and the quest tracker were held and wrong for the same
        // reason: nothing here draws a map twice. This client's map renderer is
        // a Vulkan pass told where to be, not a picture the widget tree places,
        // and application.cpp already hands it WorldMapDetailFrame's rect while
        // that frame is visible. game_screen_hud takes the presence of that
        // rect as the statement that the map is wanted, in place of its own
        // flag, and both the M key and the micro-menu button already route to
        // ToggleFrame(WorldMapFrame) when the element is owned.
        //
        // Its two unfired events are accounted for: WORLD_MAP_NAME_UPDATE is
        // registered with no branch to handle it, and CLOSE_WORLD_MAP is the
        // server telling the map to shut, which nothing here sends - the key
        // closes it through the same toggle that opens it.
        //
        // "chat" was out for a while and is back. It had forty-eight unbound
        // names in chatframe.lua, and unlike the Battle.net set they sat
        // behind no feature flag - each was a slash command handler, so each
        // was a raise the moment someone typed that command. All forty-eight
        // are bound now: implemented where this client can do the thing,
        // answered safely where it cannot. A safe answer makes the command do
        // nothing, which is honest for a feature that is not there; a missing
        // name took the chat frame down with it.
        if (out.erase("candidates") > 0) {
            out.insert(defaults.begin(), defaults.end());
            // Nothing is added on top any more. bagbar, micromenu, uierrors,
            // raidwarning and finally worldmap have all come out of here and
            // into the defaults, so "candidates" and the defaults name the
            // same set - every element is handed over. The word is kept
            // because runs and notes use it, and because the next element
            // added goes here first.
            LOG_WARNING("FrameXML: 'candidates' is the defaults now - every "
                        "element is handed over, so there is nothing left for "
                        "it to add.");
        }

        if (out.count("all") == 0) {
            for (const std::string& name : out) {
                // "none" and "all" are answers, not element names - "none" is
                // how a run says to use this client's own interface throughout,
                // and reporting it as a misspelling reads as though the flag
                // was ignored.
                bool known = (name == "mainmenubar" || name == "none" ||
                              name == "all");
                for (const Entry& e : kElements) known |= (e.name == name);
                if (!known) LOG_WARNING("WOWEE_FRAMEXML_UI: no element called '", name, "'");
            }
        }
        if (!out.empty()) {
            std::string list;
            for (const std::string& n : out) { list += n; list += ' '; }
            LOG_WARNING("FrameXML is drawing these instead of the client: ", list);
        }
        return out;
    }();
    return names;
}

} // namespace

namespace {

/// Whether a name covers this element as part of something larger.
///
/// The client draws its action bar, bag bar, micro menu and the two thin bars
/// above them as separate pieces, because it built them separately. FrameXML
/// draws all of them as MainMenuBar: one frame, one strip of art, the griffins
/// at either end. Handing over "actionbar" alone therefore leaves the client's
/// bag bar and micro menu sitting on top of FrameXML's, in the same place, and
/// the result reads as one bar drawn twice rather than a replacement that half
/// worked.
bool coveredByGroup(const std::string& name, UiElement element) {
    if (name != "mainmenubar") return false;
    switch (element) {
        case UiElement::ActionBar:
        case UiElement::StanceBar:
        case UiElement::BagBar:
        case UiElement::MicroMenu:
        case UiElement::XpBar:
        case UiElement::RepBar:
            return true;
        default:
            return false;
    }
}

} // namespace

/// Whether FrameXML was loaded at all. Owning an element it did not build
/// would hide this client's version and put nothing in its place.
static bool frameXmlLoaded() {
    static const bool on = [] {
        const char* v = std::getenv("WOWEE_LOAD_FRAMEXML");
        return v ? (std::string(v) != "0") : true;
    }();
    return on;
}

namespace {
/// Elements handed back because their frames were never built. A bit per
/// UiElement - written once from the render thread during the takeover check
/// and read from everywhere, so an atomic rather than a set.
std::atomic<uint64_t> gReleased{0};

constexpr uint64_t releaseBit(UiElement element) {
    return uint64_t{1} << static_cast<unsigned>(element);
}
} // namespace

bool frameXmlWasReleased(UiElement element) {
    return (gReleased.load(std::memory_order_relaxed) & releaseBit(element)) != 0;
}

bool frameXmlOwns(UiElement element) {
    // Nothing is owned if FrameXML was not loaded: hiding this client's own
    // version of something and putting nothing in its place is worse than
    // either interface on its own.
    if (!frameXmlLoaded()) return false;

    // The same for one element whose frames never arrived.
    if (frameXmlWasReleased(element)) return false;

    const auto& names = requested();
    if (names.empty() || names.count("none")) return false;
    if (names.count("all")) return true;
    if (names.count(std::string(uiElementName(element)))) return true;
    for (const std::string& n : names) {
        if (coveredByGroup(n, element)) return true;
    }
    return false;
}

namespace {
/// Written from the packet thread and read from the render thread.
std::atomic<bool> gWorldEntered{false};
std::atomic<bool> gCheckRequested{false};
std::atomic<bool> gMouseOwned{false};
} // namespace

namespace {
/// Main thread only: the Lua bindings that set it and the renderer that reads
/// it both run there.
std::string gCursorItem;
} // namespace

void frameXmlSetCursorItem(const std::string& iconPath) { gCursorItem = iconPath; }
const std::string& frameXmlCursorItem() { return gCursorItem; }

namespace {
std::function<bool(uint8_t&, uint8_t&)> gCursorHeldSlot;
std::function<void()> gCursorPutDown;
} // namespace

void frameXmlSetCursorBridge(std::function<bool(uint8_t&, uint8_t&)> heldSlot,
                             std::function<void()> putDown) {
    gCursorHeldSlot = std::move(heldSlot);
    gCursorPutDown = std::move(putDown);
}

bool frameXmlCursorWireSlot(uint8_t& bag, uint8_t& slot) {
    return gCursorHeldSlot && gCursorHeldSlot(bag, slot);
}

void frameXmlPutCursorDown() {
    if (gCursorPutDown) gCursorPutDown();
}

void frameXmlNoteMouseOwned(bool owned) {
    gMouseOwned.store(owned, std::memory_order_relaxed);
}
bool frameXmlOwnsMouse() { return gMouseOwned.load(std::memory_order_relaxed); }

namespace { std::atomic<bool> gCombatTextAddOn{false}; }

namespace {
/// Every load-on-demand addon that has actually loaded.
///
/// Without this, a panel from one of them cannot be told apart from a panel
/// that failed to build: both are "the frame does not exist". With it the
/// question becomes answerable - the addon loaded and the frame still is not
/// there - which is the only way the safety net can cover them.
///
/// Written from the addon loader and read from the render thread, and both are
/// rare, so a lock rather than anything clever.
std::mutex gLoadedAddOnsLock;
std::set<std::string> gLoadedAddOns;
} // namespace

bool frameXmlAddOnLoaded(std::string_view addOnName) {
    std::string lower;
    lower.reserve(addOnName.size());
    for (char ch : addOnName)
        lower += static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    std::lock_guard<std::mutex> guard(gLoadedAddOnsLock);
    return gLoadedAddOns.count(lower) != 0;
}

void frameXmlNoteAddOnLoaded(const std::string& addOnName) {
    // Matched without regard to case, because the interface asks for
    // "Blizzard_CombatText" and the directory is blizzard_combattext.
    std::string lower;
    lower.reserve(addOnName.size());
    for (char c : addOnName)
        lower += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    {
        std::lock_guard<std::mutex> guard(gLoadedAddOnsLock);
        gLoadedAddOns.insert(lower);
    }
    if (lower == "blizzard_combattext") {
        gCombatTextAddOn.store(true, std::memory_order_relaxed);
        LOG_INFO("FrameXML: Blizzard_CombatText loaded; this client's floating "
                 "combat text stands down");
    }
}

bool frameXmlDrawsCombatText() {
    // False while the client sends no COMBAT_TEXT_UPDATE, whatever is loaded.
    //
    // Standing down for Blizzard_CombatText assumed it would draw what this
    // client had stopped drawing. It cannot: every damage and healing number it
    // shows arrives on COMBAT_TEXT_UPDATE, and this client has never fired that
    // event once. What it does fire reaches the addon as the bare event name -
    // which is why entering combat still announces itself from the middle of
    // the screen while nothing else does, and why the handover read as working.
    //
    // The two are not the same element either. Blizzard's is a column beside
    // the player frame; this client's is drawn in the world at whoever dealt or
    // took the damage. Restoring COMBAT_TEXT_UPDATE would put a second copy of
    // the numbers in that column, not on top of these, and this returns to
    // gCombatTextAddOn on the day it is sent.
    return false;
}

void frameXmlNoteWorldEntry() { gWorldEntered.store(true, std::memory_order_relaxed); }
bool frameXmlWorldEntered() { return gWorldEntered.load(std::memory_order_relaxed); }

namespace { std::atomic<bool> gProbeRequested{false}; }

void frameXmlRequestCheck() {
    gCheckRequested.store(true, std::memory_order_relaxed);
    gProbeRequested.store(true, std::memory_order_relaxed);
}
bool frameXmlTakeCheckRequest() {
    return gCheckRequested.exchange(false, std::memory_order_relaxed);
}
bool frameXmlTakeProbeRequest() {
    return gProbeRequested.exchange(false, std::memory_order_relaxed);
}

bool frameXmlBuiltOnDemand(std::string_view frameName) {
    // Aura buttons, and nothing else so far. The buff frame holds up to 32 of
    // them and creates each the first time an aura needs it, so on a character
    // with no buffs none of them exist and that is the correct state.
    return frameName.rfind("BuffButton", 0) == 0 ||
           frameName.rfind("DebuffButton", 0) == 0;
}

namespace { void appendCandidateFrames(std::vector<std::string>& out); }

std::vector<std::string> frameXmlCandidateFrames() {
    // Every element this branch has *not* taken over, so the takeover check
    // reports whether each would have built if it had been.
    //
    // Derived from the check rows rather than named again here. It used to be
    // a list of its own, "so that adding a candidate is a deliberate act", and
    // it sat empty behind a comment saying everything worth swapping had been
    // handed over - which stopped being true the moment thirty-three elements
    // went behind WOWEE_FRAMEXML_UI=candidates. Now that every one of those has
    // a check row, the deliberate act is writing that row, and one list cannot
    // disagree with the other.
    std::vector<std::string> out;
    appendCandidateFrames(out);
    return out;
}

namespace {
struct Suppress {
    UiElement element;
    const char* frames;
    /// True when these frames arrive with a load-on-demand addon and so do not
    /// exist until something asks for it. Suppression still works - the pass
    /// looks each name up every frame - but the "nothing is named this" report
    /// must stay quiet about them, or it fires for all of them every run and
    /// stops being worth reading.
    bool lazy = false;
};
const Suppress kSuppress[] = {
        // What is left is the short list of elements this client still draws.
        //
        // A suppression row outlives its element. Everything else that was in
        // this table has been handed over and its own drawing deleted, and a
        // row for an element nothing draws hides FrameXML's frame with nothing
        // behind it - a blank where a working window would have been.
        // Seventeen rows were doing that, and doing it invisibly: suppression
        // is skipped for an owned element and all seventeen are owned by
        // default, so the fault could only show in a run that names a subset,
        // which is the one thing this table exists for.
        //
        // The rule is in handover_halves_check now, both ways round: an
        // element this client draws needs a row, and an element it does not
        // draw may not have one.

        // The world map's flight-map mode is this client's own taxi picker and
        // is still drawn - SMSG_SHOWTAXINODES puts the map into it. Without
        // this, talking to a flight master put TaxiFrame over this client's
        // map, each with its own set of pins.
        {UiElement::Taxi,       "TaxiFrame"},
        // The menu itself is FrameXML's. The three options frames behind it
        // are not: its Video, Sound and Interface buttons are routed to this
        // client's settings panel, because FrameXML's own are shells and the
        // sixty-odd settings live here.
        {UiElement::GameMenu,   "InterfaceOptionsFrame "
                                "VideoOptionsFrame AudioOptionsFrame"},
        // Found by the unaccounted-element check on its first run. The world
        // map was neither handed over nor hidden, so FrameXML's drew over this
        // client's own. It appears in the check list, which is what made it
        // look accounted for on every reading by eye.
        {UiElement::WorldMap,   "WorldMapFrame WorldMapDetailFrame "
                                "WorldMapButton WorldMapZoneMinimapDropDown"},
        // Both, because the sub-zone line is a separate frame that fades on its
        // own - naming only the zone would leave "Trade District" announcing
        // itself twice while "Stormwind City" announced itself once.
        {UiElement::ZoneText,   "ZoneTextFrame SubZoneTextFrame"},
        // MinimapCluster carries the ring, the zoom buttons and the zone text,
        // so hiding it hides the rest. WOWEE_FRAMEXML_UI=playerframe left
        // FrameXML's minimap drawn beside this client's own.
        {UiElement::Minimap,    "MinimapCluster"},
    };
}  // namespace

/// The frame names in one suppression entry, which are held as a single
/// space-separated string.
///
/// Both readers of kSuppress walked this themselves, and they did not agree
/// about whether `frames` may be null: the lazy one skips a null entry and the
/// other constructed a std::string from it. No entry is null today, so the
/// difference is a crash waiting for the first entry that names no frames.
void appendFrameNames(const char* frames, std::vector<std::string>& out) {
    if (!frames) return;
    const std::string all(frames);
    size_t at = 0;
    while (at < all.size()) {
        const size_t sp = all.find(' ', at);
        std::string one = all.substr(
            at, sp == std::string::npos ? std::string::npos : sp - at);
        if (!one.empty()) out.push_back(std::move(one));
        if (sp == std::string::npos) break;
        at = sp + 1;
    }
}

std::vector<std::string> frameXmlSuppressedFrames() {
    std::vector<std::string> out;
    for (const Suppress& s : kSuppress) {
        if (frameXmlOwns(s.element)) continue;   // it is the one in use
        appendFrameNames(s.frames, out);
    }
    return out;
}

std::vector<std::string> frameXmlLazySuppressedFrames() {
    std::vector<std::string> out;
    for (const Suppress& s : kSuppress) {
        if (!s.lazy) continue;
        appendFrameNames(s.frames, out);
    }
    return out;
}

namespace { void reportUncheckedElements(); }

void frameXmlReportUnaccountedElements() {
    // Every element must be one thing or the other: drawn by FrameXML with
    // this client's version gated off, or drawn by this client with FrameXML's
    // hidden. An element that is neither is drawn twice, and that is not
    // visible from either list on its own - it is the gap between them.
    //
    // Thirteen windows were in that gap and nobody noticed until they were
    // looked for: the vendor, the loot window, the bank, the party frames, the
    // friends list, the quest giver, the gossip list, the mailbox, and then
    // the trade skill, trainer, auction and guild bank panels once their
    // addons started loading. The buff bar and the durability warning made
    // fifteen. Saying so at startup is cheaper than finding the sixteenth the
    // same way.
    const std::vector<std::string> suppressed = frameXmlSuppressedFrames();
    for (const Entry& e : kElements) {
        if (frameXmlOwns(e.element)) continue;
        if (!e.clientDraws) continue;  // FrameXML's is the only one there is
        // Suppressed elements contribute frame names; an element that
        // contributes none while not being owned is unaccounted for.
        bool hasFrames = false;
        for (const Suppress& sup : kSuppress) {
            if (sup.element == e.element && sup.frames && *sup.frames) {
                hasFrames = true;
                break;
            }
        }
        if (!hasFrames) {
            LOG_WARNING("FrameXML: '", e.name, "' is neither handed over nor "
                        "suppressed - if FrameXML draws it, it is on screen "
                        "twice");
        }
    }

    reportUncheckedElements();
}

namespace {
// One row per element: what has to exist for it to have arrived. Chosen as
// the frame itself, the art that frames it, and the parts that carry live
// data - which between them separate "never built" from "built and empty"
// from "built and misplaced".
//
// At namespace scope because the unaccounted-frame sweep reads it too: a name
// mentioned here is a name somebody has considered, which is the whole of what
// that sweep needs to know.
struct Check {
    UiElement element;
    const char* frames;
    /// Built only when the player opens it - a load-on-demand addon's panel.
    /// Reported as not built, which is the correct state for one nobody has
    /// opened, and never handed back for it while that is still the answer.
    bool lazy = false;
    /// The addon a lazy panel arrives with.
    ///
    /// Named so the two states can be told apart. "The frame does not exist"
    /// means nothing on its own for these - it is the normal state before the
    /// panel is first opened - but once the addon has loaded and the frame is
    /// still absent, that is the same failure the net exists for. Without this
    /// the eight load-on-demand elements could only be handed over blind.
    const char* addOn = nullptr;
};
const Check kChecks[] = {
        {UiElement::PlayerFrame,  "PlayerFrame PlayerFrameTexture PlayerPortrait "
                                  "PlayerFrameHealthBar PlayerFrameManaBar PlayerName "
                                  "PlayerLevelText "
                                  // The numbers on the bars. Built and empty,
                                  // built and sized to nothing, and never built
                                  // are three different faults that look the
                                  // same on screen.
                                  "PlayerFrameHealthBarText PlayerFrameManaBarText"},
        {UiElement::TargetFrame,  "TargetFrame TargetFrameTextureFrame TargetFramePortrait "
                                  "TargetFrameHealthBar TargetFrameManaBar "
                                  "TargetFrameTextureFrameName TargetFrameNameBackground"},
        // The pet's cast bar belongs with the pet frame rather than with
        // the player's cast bar: this client draws it inside
        // renderPetFrame, so it goes when that does.
        {UiElement::PetFrame,     "PetFrame PetFrameHealthBar PetFrameManaBar "
                                  "PetCastingBarFrame"},
        {UiElement::Minimap,      "Minimap MinimapBorder MinimapZoomIn MinimapZoneText"},
        // The extra bars as well as the main one: this client draws its own
        // second and third bars from the settings, so naming only MainMenuBar
        // left four more stacked on top of them.
        {UiElement::ActionBar,    "MainMenuBar MainMenuBarArtFrame MainMenuBarLeftEndCap "
                                  "MainMenuBarRightEndCap ActionButton1 ActionButton12 "
                                  "MultiBarBottomLeft MultiBarBottomRight "
                                  "MultiBarLeft MultiBarRight"},
        {UiElement::BagBar,       "MainMenuBarBackpackButton CharacterBag0Slot"},
        {UiElement::MicroMenu,    "CharacterMicroButton MainMenuBarPerformanceBar"},
        // MainMenuExpBar is the bar itself; ExhaustionTick is the rested
        // marker that rides on it. Checked against the XML rather than
        // guessed - a name invented here reports NOT BUILT forever and reads
        // as a fault in the interface rather than in this list.
        {UiElement::XpBar,        "MainMenuExpBar ExhaustionTick"},
        // Both hang off the minimap cluster, so if either is in the wrong
        // place the cluster's own rect is the first thing to look at. Not
        // BuffButton1, for the same reason the Buffs entry below gives: the
        // buff frame creates each button the first time an aura needs one, so
        // a character carrying none has no BuffButton1 and this reported the
        // minimap as NOT BUILT for as long as that was true.
        {UiElement::Minimap,      "MinimapCluster BuffFrame DurabilityFrame"},
        {UiElement::RepBar,       "ReputationWatchBar ReputationWatchStatusBar"},
        {UiElement::StanceBar,    "ShapeshiftBarFrame ShapeshiftButton1"},
        {UiElement::CastBar,      "CastingBarFrame CastingBarFrameBorder CastingBarFrameText"},
        {UiElement::Chat,         "ChatFrame1 ChatFrame1EditBox GeneralDockManager"},
        // Both are hidden except during the fade, so NOT BUILT is the fault to
        // look for here and hidden is the normal state.
        {UiElement::ZoneText,     "ZoneTextFrame ZoneTextString SubZoneTextFrame"},
        {UiElement::QuestTracker, "WatchFrame WatchFrameTitle"},
        {UiElement::FocusFrame,   "FocusFrame FocusFrameHealthBar"},
        {UiElement::WorldMap,     "WorldMapFrame WorldMapDetailFrame WorldMapButton "
                                  "WorldMapZoneMinimapDropDown"},
        {UiElement::CharacterFrame, "CharacterFrame PaperDollFrame CharacterModelFrame "
                                    "CharacterNameText CharacterHeadSlot "
                                    "CharacterResistanceFrame CharacterAttributesFrame "
                                    "MagicResFrame1 CharacterMainHandSlot "
                                    // The rotate arrows sit on the model
                                    // frame's top-left corner, which is also
                                    // where the name sits - so where each one
                                    // actually lands is the question.
                                    "CharacterModelFrameRotateLeftButton "
                                    "CharacterModelFrameRotateRightButton"},
        {UiElement::Bags,         "ContainerFrame1 ContainerFrame1Item1 "
                                  "ContainerFrame1Name"},
        // A bag's background is assembled from a top, up to two middles and a
        // bottom, each a slice of one atlas positioned against the one above
        // it. When the stack is wrong the art shows a seam in the wrong place
        // and stops short of the frame, and only the individual rects and
        // slices say which piece is at fault.
        // Filed under the bag bar rather than the bags: FrameXML's containers
        // are opened from the bag bar, which this branch hands over, while the
        // "bags" element only decides whether this client's own bag window is
        // suppressed. Filed there they would never be looked at.
        {UiElement::BagBar,       "ContainerFrame2 ContainerFrame2Portrait "
                                  "ContainerFrame2BackgroundTop "
                                  "ContainerFrame2BackgroundMiddle1 "
                                  "ContainerFrame2BackgroundBottom"},
        {UiElement::Spellbook,    "SpellBookFrame SpellButton1 SpellBookSkillLineTab1"},
        // QuestLogScrollFrame, not QuestLogListScrollFrame: no frame has ever
        // been called that, so the list of quests was never hidden with the
        // window around it.
        {UiElement::QuestLog,     "QuestLogFrame QuestLogScrollFrame "
                                  "QuestLogDetailScrollFrame"},

        // ---- The elements behind WOWEE_FRAMEXML_UI=candidates ----
        //
        // Written so the release above can see them. An element with no row
        // here is one that, if its frames never arrive, draws nothing and says
        // nothing - this client's own version is hidden the moment it is
        // handed over, and there is no way back to it.
        //
        // Top-level frame first in every row: that is the one the release
        // tests. The rest are the art and the parts carrying live data, which
        // is what separates "never built" from "built and empty".
        // The QuestInfo chain is here because the quest dialog's *contents*
        // can land far from the dialog while the dialog itself is placed
        // correctly, and the four frames above cannot show that.
        //
        // QuestInfo_Display reparents and anchors each element it is handed,
        // and on the detail panel it runs twice - once into
        // QuestDetailScrollChildFrame and once into QuestInfoFadingFrame. A
        // session was reported where the reward items sat some eight hundred
        // units right of the panel, and this list said nothing about it: the
        // only evidence was which frame a click happened to hit. These four
        // are the chain that carries them, so the next dump names the one that
        // is misplaced and what it is parented to.
        {UiElement::QuestGiver,   "QuestFrame QuestFrameNpcNameText "
                                  "QuestFrameDetailPanel QuestFrameAcceptButton "
                                  "QuestDetailScrollChildFrame QuestInfoFadingFrame "
                                  "QuestInfoRewardsFrame QuestInfoItem1"},
        {UiElement::Gossip,       "GossipFrame GossipFrameNpcNameText "
                                  "GossipFrameGreetingPanel"},
        {UiElement::Mail,         "MailFrame InboxFrame MailFrameTab1"},
        {UiElement::Vendor,       "MerchantFrame MerchantItem1 MerchantMoneyFrame "
                                  "MerchantNameText"},
        {UiElement::Loot,         "LootFrame LootButton1"},
        {UiElement::Bank,         "BankFrame BankFrameTitleText"},
        // Not BuffButton1: the buff frame creates each button the first time an
        // aura needs one, so on a character carrying none they do not exist and
        // that is correct.
        {UiElement::Buffs,        "BuffFrame TemporaryEnchantFrame"},
        {UiElement::Durability,   "DurabilityFrame"},
        {UiElement::PartyFrames,  "PartyMemberFrame1 PartyMemberFrame1HealthBar "
                                  "PartyMemberFrame1Name"},
        {UiElement::Social,       "FriendsFrame FriendsFrameTitleText"},
        {UiElement::Trade,        "TradeFrame TradeFrameRecipientNameText"},
        {UiElement::ReadyCheck,   "ReadyCheckFrame ReadyCheckFrameText"},
        {UiElement::Taxi,         "TaxiFrame TaxiRouteMap"},
        {UiElement::Stable,       "PetStableFrame"},
        {UiElement::Book,         "ItemTextFrame ItemTextPageText"},
        {UiElement::GameMenu,     "GameMenuFrame GameMenuButtonLogout"},
        {UiElement::Help,         "HelpFrame TicketStatusFrame"},
        {UiElement::Totems,       "TotemFrame MultiCastActionBarFrame"},
        {UiElement::UiErrors,     "UIErrorsFrame"},

        // Load-on-demand: reported, never released. Their frames do not exist
        // until the player opens the panel, which is not a failure to build.
        {UiElement::Talents,       "PlayerTalentFrame",  true, "Blizzard_TalentUI"},
        {UiElement::TradeSkill,    "TradeSkillFrame",    true, "Blizzard_TradeSkillUI"},
        {UiElement::ClassTrainer,  "ClassTrainerFrame",  true, "Blizzard_TrainerUI"},
        {UiElement::AuctionHouse,  "AuctionFrame",       true, "Blizzard_AuctionUI"},
        {UiElement::GuildBank,     "GuildBankFrame",     true, "Blizzard_GuildBankUI"},
        {UiElement::Inspect,       "InspectFrame",       true, "Blizzard_InspectUI"},
        {UiElement::Achievements,  "AchievementFrame",   true, "Blizzard_AchievementUI"},
        {UiElement::BarberShop,    "BarberShopFrame",    true, "Blizzard_BarbershopUI"},
        // Not lazy: LFDFrame.xml is listed in framexml.toc, so
        // LFDParentFrame is built at load like any core frame. Its
        // *suppression* row is marked lazy for the popups that arrive
        // later, which is a different question from whether the panel
        // itself exists.
        {UiElement::DungeonFinder, "LFDParentFrame"},
        // Not lazy either: petitionframe.xml, guildregistrarframe.xml and
        // arenaregistrarframe.xml are all in framexml.toc. Only PetitionFrame
        // is named because the net asks about the top frame of a row and no
        // more - which of the three is on screen depends on the NPC, and a
        // registrar the player has not walked up to is not a failure.
        {UiElement::Petition,      "PetitionFrame"},
};

/// Elements handed over with no check row, which therefore get no safety net.
///
/// frameXmlReleaseUnbuiltElements walks the check rows and hands back anything
/// whose top-level frame never built, because a hidden window with nothing in
/// its place is worse than either interface alone. An element with no row is
/// invisible to that: if its frames never arrive, nothing draws it and nothing
/// says so.
///
/// Deliberately not derived from the suppression rows instead. Those name the
/// right frames, but half of them belong to load-on-demand addons - the
/// auction house, the guild bank, the achievement panel - whose frames
/// correctly do not exist until the player opens them, and handing those back
/// on the first check in world would take away every panel that works exactly
/// as intended.
///
/// So writing a check row is part of handing an element over.
void reportUncheckedElements() {
    for (const Entry& e : kElements) {
        if (!frameXmlOwns(e.element)) continue;
        bool checked = false;
        for (const Check& c : kChecks) {
            if (c.element == e.element && c.frames && *c.frames) { checked = true; break; }
        }
        if (!checked) {
            LOG_WARNING("FrameXML: '", e.name, "' is handed over with no check "
                        "row - if its frames never build, nothing draws it and "
                        "nothing notices");
        }
    }
}

/// The top-level frame of every element not currently handed over.
///
/// One name each rather than the whole row: this is a readiness line, and the
/// question it answers is whether the panel exists at all. The rest of a row
/// separates "built and empty" from "built and misplaced", which are questions
/// for an element already in use.
///
/// Load-on-demand panels are skipped. Theirs do not exist until the player
/// opens them, so reporting them as not built says nothing and would bury the
/// ones it does say something about.
void appendCandidateFrames(std::vector<std::string>& out) {
    for (const Check& c : kChecks) {
        if (c.lazy || !c.frames || !*c.frames) continue;
        if (frameXmlOwns(c.element)) continue;   // in use, and checked already
        std::string all(c.frames);
        const size_t sp = all.find(' ');
        std::string top = all.substr(0, sp == std::string::npos ? all.size() : sp);
        if (!top.empty()) out.push_back(std::move(top));
    }
}

/// Split a space-separated frame list onto the end of a vector.
void appendFrames(const char* frames, std::vector<std::string>& out) {
    if (!frames) return;
    std::string all(frames);
    size_t at = 0;
    while (at < all.size()) {
        const size_t sp = all.find(' ', at);
        const std::string one = all.substr(
            at, sp == std::string::npos ? std::string::npos : sp - at);
        if (!one.empty()) out.push_back(one);
        if (sp == std::string::npos) break;
        at = sp + 1;
    }
}
}  // namespace

std::vector<std::string> frameXmlAccountedFrames() {
    std::vector<std::string> out;
    // Both tables, and every element in them rather than only the owned ones:
    // the question is whether a name has been considered at all, not whether
    // its element won.
    for (const Suppress& s : kSuppress) appendFrames(s.frames, out);
    for (const Check& c : kChecks)      appendFrames(c.frames, out);
    return out;
}

int frameXmlReleaseUnbuiltElements(
        const std::function<bool(const std::string&)>& frameExists) {
    int released = 0;
    for (const Check& c : kChecks) {
        if (!frameXmlOwns(c.element)) continue;   // not ours, or already given back
        if (!c.frames || !*c.frames) continue;
        // A panel nobody has opened has not failed to build. Once its addon
        // has loaded, though, the frame's absence means what it means
        // everywhere else.
        if (c.lazy && !(c.addOn && frameXmlAddOnLoaded(c.addOn))) continue;

        // The first name only: the top-level frame. A panel that built and is
        // missing a label is a different fault, and handing it back for that
        // would take away one that works.
        std::string all(c.frames);
        const size_t sp = all.find(' ');
        const std::string top = all.substr(0, sp == std::string::npos ? all.size() : sp);
        if (top.empty() || frameExists(top)) continue;

        gReleased.fetch_or(releaseBit(c.element), std::memory_order_relaxed);
        ++released;
        LOG_WARNING("FrameXML: handing '", uiElementName(c.element),
                    "' back to this client - ", top, " was never built, so "
                    "nothing would have drawn it");
    }
    return released;
}

std::vector<std::string> frameXmlCheckFrames() {
    std::vector<std::string> out;
    for (const Check& c : kChecks) {
        if (!frameXmlOwns(c.element)) continue;
        std::string all(c.frames);
        size_t at = 0;
        while (at < all.size()) {
            const size_t sp = all.find(' ', at);
            const std::string one = all.substr(
                at, sp == std::string::npos ? std::string::npos : sp - at);
            if (!one.empty()) out.push_back(one);
            if (sp == std::string::npos) break;
            at = sp + 1;
        }
    }
    return out;
}

std::string_view uiElementName(UiElement element) {
    for (const Entry& e : kElements) {
        if (e.element == element) return e.name;
    }
    return "";
}

} // namespace wowee::ui
