#pragma once

// Which parts of the interface FrameXML has taken over from this client.
//
// All of them. The two interfaces drew the same things - a player frame, an
// action bar, a minimap - and while the replacement was in progress this said
// which of the client's own elements to leave out, one name at a time, so that
// each could be tried and backed out without touching the code that drew
// either. It was set through WOWEE_FRAMEXML_UI.
//
// That option is gone, because the thing it selected between is gone: the
// client's own version of all but a handful of these has been deleted, and
// `WOWEE_FRAMEXML_UI=none` therefore stopped meaning "use the interface this
// client draws" and started meaning "draw nothing at all". A screen with no
// player frame, no bars and no bags is not a fallback, and an option whose
// other setting is that is not a choice.
//
// What is left of the system is the accounting: which frames each element
// stands or falls on, which of this client's own drawing is still gated behind
// one, and the safety net that hands an element back when FrameXML's version
// of it was never built. The list of elements below is that record - it says
// what has been accounted for, not what is switched on.
//
// WOWEE_LOAD_FRAMEXML=0 still turns the interface off wholesale, and with it
// every element here: see frameXmlOwns, which answers false for all of them
// when nothing was loaded.

#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

namespace wowee::ui {

/// The client's own elements that FrameXML has an equivalent for.
enum class UiElement {
    PlayerFrame,
    TargetFrame,
    PetFrame,
    FocusFrame,
    ActionBar,
    StanceBar,
    BagBar,
    MicroMenu,
    XpBar,
    RepBar,
    CastBar,
    Minimap,
    Chat,
    QuestTracker,
    WorldMap,
    CharacterFrame,
    Bags,
    Spellbook,
    QuestLog,
    /// The quest giver, the gossip list and the mailbox. This client draws all
    /// three itself and they work; FrameXML's versions are drawn too unless
    /// they are named here, which is two of every window at every NPC.
    QuestGiver,
    Gossip,
    Mail,
    /// The rest of the windows this client draws itself. Each is named
    /// separately rather than lumped together so that handing one over later
    /// is a single word in the defaults, the way the character sheet was.
    Vendor,
    Loot,
    Bank,
    PartyFrames,
    Social,
    TradeSkill,
    ClassTrainer,
    AuctionHouse,
    GuildBank,
    Inspect,
    /// The buff and debuff bar. FrameXML's is already treated as in use - it
    /// is checked alongside the minimap cluster - but this client kept drawing
    /// its own beside it, so there were two.
    Buffs,
    /// The low-durability warning. Same story as the buffs: FrameXML's
    /// DurabilityFrame is checked as in use and this client drew its own
    /// warning beside it.
    Durability,
    /// The zone name that appears on crossing into one. Same story again:
    /// FrameXML's ZoneTextFrame listens for ZONE_CHANGED_NEW_AREA, which this
    /// client fires, so its large centred banner was raised beside this
    /// client's own smaller one and every zone crossing announced itself
    /// twice. The large one is what retail shows, so this is handed over
    /// rather than suppressed.
    ZoneText,
    /// Three more found by the unaccounted-frame sweep. Each has a working
    /// counterpart this client draws, and the first two are live duplicates:
    /// TRADE_SHOW and READY_CHECK are both fired, so FrameXML raised its
    /// window beside the client's own every time.
    ///
    /// RaidWarning is the exception and is named anyway. Its frames cannot
    /// appear today because CHAT_MSG_RAID_WARNING is never fired - but this
    /// client draws raid warnings from the chat history rather than from the
    /// event, so firing it later would put a second banner on screen with
    /// nothing to say why.
    Trade,
    ReadyCheck,
    RaidWarning,
    /// The confirmation prompts: StaticPopup1 through 4, which FrameXML reuses
    /// for every question it asks.
    ///
    /// uiparent.lua answers PARTY_INVITE_REQUEST, RESURRECT_REQUEST,
    /// CONFIRM_SUMMON and CONFIRM_TALENT_WIPE with one, and this client fires
    /// all four while drawing its own dialog for each - so every one of those
    /// was asked twice.
    ///
    /// This is the one that goes to FrameXML rather than staying here, because
    /// a StaticPopup is also how an item is deleted: the bags are handed over,
    /// so containerframe.lua raises the delete confirmation and this client has
    /// no other way to ask. Keeping our own four and hiding FrameXML's would
    /// have traded four duplicates for no way to destroy an item.
    Dialogs,
    /// Windows this client draws that FrameXML also has. The last three cannot
    /// appear today because the events that would show them are not fired -
    /// but that is a fact about the client's current reach, not a decision,
    /// and it would stop being true the moment someone fired one.
    Achievements,
    BarberShop,
    Taxi,
    Stable,
    Book,
    /// The game menu and the options panels behind it, the help window, and
    /// the battleground scoreboard. Each is reachable from the micro buttons
    /// on the bar this branch has taken over. The menu itself is FrameXML's
    /// now; the options panels behind it are still this client's settings
    /// window, which its Video, Sound and Interface buttons open.
    GameMenu,
    Help,
    BattlegroundScore,
    /// Windows whose FrameXML version only became reachable once its API was
    /// finished. Each has a counterpart this client draws, so each has to be
    /// accounted for or it appears twice.
    Totems,
    Talents,
    UiErrors,
    /// The dungeon finder. Found by the ungated-draw sweep: this client draws
    /// a proposal popup, a role-check popup and a browser window, none of them
    /// behind a gate, and FrameXML has all three.
    ///
    /// LFG_PROPOSAL_SHOW is fired, so LFDDungeonReadyPopup was already being
    /// raised beside this client's own every time a group formed.
    DungeonFinder,
    /// Charters: buying one, signing one, turning one in. Found by following
    /// the unaccounted-frame sweep to what fires, which is what that sweep had
    /// never been read against.
    ///
    /// Three of FrameXML's frames and two of this client's, on three events
    /// this client fires from one handler pair - GUILD_REGISTRAR_SHOW and
    /// PETITION_VENDOR_SHOW raise GuildRegistrarFrame and ArenaRegistrarFrame,
    /// PETITION_SHOW raises PetitionFrame, and this client opens its own
    /// "CreateGuildPetition" and "PetitionSignatures" popups on the same two
    /// packets. So every charter bought or signed asked twice.
    ///
    /// The events were added deliberately - the note beside PETITION_SHOW says
    /// the interface's version was never told, so a charter could not be
    /// signed through it. Firing them is what made the duplicate.
    Petition,
};

/// True when FrameXML is drawing this instead, so the client should not.
///
/// **Before gating a render pass on this, ask whether FrameXML draws that
/// layer at all.** For a window the answer is yes and the gate is right. For a
/// layer the *real* client draws natively, there is nothing on the other side
/// to take the work over, and gating it simply deletes it - with no error, no
/// warning, and a handover check that still passes because the gate is there.
///
/// Two of those, found 2026-08-05 and both invisible rather than loud:
///
///   * **Minimap blips.** minimap.xml declares the border, the buttons, the
///     mail and battlefield icons and the north tag - and not one frame for a
///     party member, a flight master or a corpse. Gating renderMinimapMarkers
///     left the ring drawn and nothing on it. The pass runs either way now and
///     the gate moved inward, to the mouse handling and the two indicators
///     that really are FrameXML's.
///   * **Everything inside WorldMapDetailFrame.** Not gated away but hidden:
///     the widget renderer draws into ImGui's background list and this
///     client's map is a window over it, so whatever FrameXML puts there
///     cannot be seen and this side has to draw it. The layer inventory is in
///     framexml_takeover.cpp beside the worldmap entry.
///
/// Every other gate was audited at the same time and is a window.
bool frameXmlOwns(UiElement element);

/// The name an element is switched on by, for diagnostics.
std::string_view uiElementName(UiElement element);

/// The frames worth checking for every element currently handed over.
///
/// Replacing one part of the interface at a time only works if there is an
/// answer to "did it arrive". A screenshot is the honest test but not always
/// available, and the failures so far have all been legible from the tree
/// alone: a frame that was never created, one that is hidden, one laid out to
/// nothing. These are the frames each element stands or falls on, so the check
/// can be one block of log rather than an inspection.
std::vector<std::string> frameXmlCheckFrames();

/// Every frame name the takeover system mentions anywhere, owned or not.
///
/// A name in here is a name somebody has considered. Anything FrameXML puts on
/// screen that is *not* in here is a part of the interface nobody has decided
/// about, which is how the zone banner came to be drawn beside this client's
/// own for months: it was not an element, so the element-level check had
/// nothing to iterate and never mentioned it.
std::vector<std::string> frameXmlAccountedFrames();

/// The frames worth looking at for elements not yet handed over.
///
/// Deciding whether the next element is ready means seeing whether FrameXML's
/// version of it is built, positioned and carrying data - and the check only
/// reports what is already owned, so readiness is exactly what cannot be seen.
/// These are reported alongside, marked as candidates.
std::vector<std::string> frameXmlCandidateFrames();

/// The frames to keep hidden because FrameXML draws them and this client has
/// not handed that element over.
///
/// Every FrameXML file is loaded, so every frame it declares exists and draws.
/// The takeover list only decides whether this client's own version is
/// suppressed alongside it - which for anything not yet handed over means two
/// of them on screen at once. Hiding them once after loading is not enough:
/// the interface shows them again on its own schedule.
std::vector<std::string> frameXmlSuppressedFrames();

/// The subset of those whose frames arrive with a load-on-demand addon.
///
/// They do not exist until something asks for that addon, so a report of names
/// that resolved to nothing has to leave them out - otherwise it names all of
/// them on every run and the one real typo is lost among them.
std::vector<std::string> frameXmlLazySuppressedFrames();

/// Whether a frame in the check list is one the interface builds only when it
/// has something to put in it.
///
/// A buff button does not exist until there is a buff: FrameXML creates them on
/// demand from AuraButton_Update. Reporting that as NOT BUILT alongside genuine
/// failures teaches the reader to skip the line, which costs the report the one
/// thing it is for. Named here rather than guessed from the name so that a real
/// BuffButton1 failure still reads as one.
bool frameXmlBuiltOnDemand(std::string_view frameName);

/// Note that the player has entered the world, and whether that has happened.
///
/// FrameXML does most of its arranging from PLAYER_ENTERING_WORLD: frames are
/// hidden, repositioned and filled with data that does not exist before then.
/// A diagnostic taken at load therefore describes a layout nobody ever sees,
/// which is worse than none - it looks like an answer. This is how the
/// diagnostics know to wait for the state being asked about.
/// Warn about any element that is neither handed over nor suppressed.
///
/// Such an element is drawn twice - once by this client and once by FrameXML -
/// and that is invisible from either list alone, because it is the gap between
/// them. Fifteen windows sat in that gap before anyone went looking.
void frameXmlReportUnaccountedElements();

void frameXmlNoteWorldEntry();
bool frameXmlWorldEntered();

/// Whether the interface has the cursor: a frame that takes the mouse is under
/// it, or one is holding a press.
///
/// The camera asks ImGui whether the interface wants the mouse, and FrameXML
/// draws into ImGui's background draw list - so ImGui has never heard of these
/// frames and answers no. Pressing a bag item therefore turned the camera as
/// well as pressing the item, and dragging one swung the view around.
void frameXmlNoteMouseOwned(bool owned);
bool frameXmlOwnsMouse();

/// Hand an element back to this client because FrameXML's version was never
/// built.
///
/// frameXmlOwns already refuses everything when FrameXML did not load at all,
/// on the grounds that hiding this client's own version and putting nothing in
/// its place is worse than either interface on its own. That is just as true of
/// one element as of all of them, and it is the reason so few have been handed
/// over: a panel that does not build is not a worse panel, it is no panel, and
/// there is no way back to the one that worked.
///
/// Checked against the top-level frame of each owned element - the first name
/// in its check row. Not the whole row: a frame that built and is missing a
/// label is a different fault, and releasing on that would hand back panels
/// that work.
///
/// Takes a lookup rather than reading the widget tree itself, so the table
/// stays where the rest of the takeover tables are.
int frameXmlReleaseUnbuiltElements(
    const std::function<bool(const std::string&)>& frameExists);

/// Whether an element was handed back this way. For the report - the answer
/// that matters at a call site is frameXmlOwns, which already accounts for it.
bool frameXmlWasReleased(UiElement element);

/// A load-on-demand addon has loaded, so whatever it draws is now on screen.
///
/// Element ownership cannot answer this. An element is chosen before the run
/// starts and stays chosen; a load-on-demand addon arrives in the middle of one
/// because the player asked for the feature, and until it does FrameXML draws
/// nothing for it - so this client's own version has to keep drawing and then
/// stand down. Blizzard_CombatText is the case: it loads from the float-mode
/// dropdown in the interface options and then draws floating combat text over
/// the one CombatUI is already drawing.
void frameXmlNoteAddOnLoaded(const std::string& addOnName);

/// Whether a load-on-demand addon has loaded. Case-insensitive: the interface
/// asks for "Blizzard_AuctionUI" and the directory is blizzard_auctionui.
bool frameXmlAddOnLoaded(std::string_view addOnName);
bool frameXmlDrawsCombatText();

/// The icon of the item the cursor is carrying, or empty for nothing.
///
/// Picking an item up in WoW takes it out of its slot and puts it on the
/// pointer, and that half is the client's job - FrameXML never draws it. Without
/// it a drag looked like nothing was happening at all, whether or not the move
/// went out. Set from the Lua bindings and read by the widget renderer, both on
/// the main thread.
void frameXmlSetCursorItem(const std::string& iconPath);
const std::string& frameXmlCursorItem();

/// Where the item on FrameXML's cursor came from, in the server's flat slot
/// space, and how to put it down again.
///
/// This client's own windows and FrameXML's have separate cursors: the Lua
/// bindings track theirs in lua_action_api, and InventoryScreen tracks its own
/// held item. With the bags handed over and the bank, mail or trade window
/// still this client's, a drag between them was picking an item up in one
/// system and offering it to the other, which had never heard of it - which is
/// what "cannot transfer from bags to bank" was.
///
/// Asked rather than mirrored. The cursor belongs to the bindings and this
/// calls back into them, because a copy kept here would be a second answer to
/// the same question and this repository's most common bug is exactly that.
/// Both are null until the Lua API registers them, and null again after the
/// state is torn down, so a caller must handle "no bridge" as "carrying
/// nothing".
void frameXmlSetCursorBridge(std::function<bool(uint8_t&, uint8_t&)> heldSlot,
                             std::function<void()> putDown);
bool frameXmlCursorWireSlot(uint8_t& bag, uint8_t& slot);
void frameXmlPutCursorDown();

/// Ask for the takeover check to be reported again on the next frame.
///
/// The automatic reports happen at fixed moments, and most of what goes wrong
/// is only visible in a state nobody can schedule: a target frame is only
/// wrong once something is targeted, a bag only once it is opened. Asking for
/// the report at the moment the interface looks wrong is the difference
/// between reading the state in question and guessing from an earlier one.
void frameXmlRequestCheck();

/// Consume a pending request, if there is one.
bool frameXmlTakeCheckRequest();

/// The same request, seen by the Lua side.
///
/// The widget report says whether a frame is shown; only the interface's own
/// API can say whether it should be. Two flags rather than one because the
/// renderer and the script engine each consume their own.
bool frameXmlTakeProbeRequest();

} // namespace wowee::ui
