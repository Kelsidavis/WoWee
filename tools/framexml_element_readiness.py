#!/usr/bin/env python3
"""Which UI elements could be handed to FrameXML without a call raising.

The transition works element by element: framexml_takeover.cpp names a set that
FrameXML draws and suppresses this client's own version of them, and an element
joins that set once it has been seen drawing correctly. This answers the half of
"correctly" that can be checked without looking at the screen - whether every
global the element's code calls is answered by something.

    tools/framexml_element_readiness.py

Two measures, because resolving every call was only ever half of it.

  * **calls** - globals the element's code invokes that nothing answers. These
    raise.
  * **events** - events its frames RegisterEvent for that nothing in src/ ever
    fires. These do not raise: the element simply sits there, or shows stale
    data, or never opens. Six real bugs came out of this column after the call
    column had gone quiet - a mail frame that hung after sending, bag cooldown
    swirls that never drew, an achievements panel that showed the empty state it
    was built with, a master looter menu that could not open.

Reports "no known gaps", not "works". A resolved call is one that will not
raise; it says nothing about whether the frame draws, is positioned, or is
anchored to anything. The visual half is still a run of the client.

An event in the second column is a lead, not a verdict. "Never fired" can also
mean *something else is fired in its place*: SMSG_QUESTGIVER_QUEST_LIST was
sending GOSSIP_SHOW where QUEST_GREETING belonged, which opened the gossip frame
over the quest list. Read the handler before concluding the event is absent.

WHAT COUNTS AS THE ELEMENT'S CODE
---------------------------------
This is the whole difficulty, and three scopes were tried before one was right.

  * The element's own files alone **under-reports**. QuestFrame and
    QuestLogFrame both draw their reward block through QuestInfo.lua, which is
    neither of their files and which had nine unanswered calls - so both were
    reported ready while any quest offering a spell, title or reputation reward
    would have raised.

  * The transitive call graph **over-reports**, and uselessly: nearly everything
    calls StaticPopup_Show, staticpopup.lua names the handler of every popup in
    the interface, and so every element comes out depending on all 309 of them.
    A popup's handlers are reached only when that popup is shown, which is a
    data-driven dispatch rather than a call.

  * One hop, minus shared infrastructure, is the useful middle. It catches
    QuestInfo - QuestFrame calls QuestInfo_Display directly - without dragging
    in every popup in the game. That is what found the money frame, whose six
    unanswered calls were blocking five elements at once and appeared in none of
    their own files.

KNOWN FALSE POSITIVES, which this cannot tell from a real gap:
  * Functions defined in a load-on-demand addon (AchievementFrame_*,
    BackpackTokenFrame_Update) are absent only until that addon loads.
  * Names this client answers deliberately as absent because it draws the thing
    itself. GetMapLandmarkInfo and GetMapOverlayInfo are owned by
    src/rendering/world_map/, and answering them would draw a second map over
    the first. The same goes for the barber shop: WindowManager has a working
    one that reads BarberShopStyle.dbc and sends CMSG_ALTER_APPEARANCE, so
    GetBarberShopStyleInfo and its four neighbours are absent on purpose.

    Check src/ for an owner before treating an element's remaining names as
    work. Both of these read as the last unimplemented features for several
    rounds when neither was unimplemented at all.

  * Events only the C client ever sends, where FrameXML does the same job in
    Lua. BAG_OPEN and BAG_CLOSED are the clear case: ToggleBag, OpenBag and
    CloseBag are Lua functions in containerframe.lua that show and hide the
    frames themselves, so the events are left for a bank or a merchant opening
    bags from outside - which this client does not do. Firing them would
    duplicate what the Lua already did.

    Read the caller before treating an event as missing. If the interface can
    reach the same result without it, the client is not the one failing to
    speak.

  * Events that share a branch with one already fired. GUILDBANK_UPDATE_MONEY
    is merchant's last, and it sits in the same elseif as PLAYER_MONEY - both
    just recheck the repair buttons. PLAYER_MONEY is fired, so the branch runs;
    the guild-bank variant only adds anything to a client that funds repairs
    from a guild bank, which this one does not. Applying the rule above: grep
    the handler for the event and see what else reaches the same line.

  * Whole features this client does not have, which read as a pile of events
    rather than as one absence. playerframe's eight are voice chat twice,
    vehicles four times, the Chinese anti-addiction playtime display and LFG
    role assignment - none of which exists here, and UNIT_ENTERED_VEHICLE does
    nothing but set inSeat and swap the frame art for a vehicle that cannot
    happen.

    Worth counting before working: an element showing eight missing events can
    be four features absent by design rather than eight gaps. bags, merchant
    and playerframe all read as gapped and all three are complete.

    mainmenubar is the same once its names are grouped: nine of ten calls and
    four of eight events are vehicles, CURRENCY_DISPLAY_UPDATE shares a branch
    with an event now fired, and UPDATE_BONUS_ACTIONBAR clears on both routes -
    its actionbutton branch shares with ACTIONBAR_PAGE_CHANGED, which is fired,
    and its own frame is the possess bar, which is the vehicle feature again.
    What is left is PickupPetAction. One absent feature and one cursor
    operation, reported as eighteen names.

    questlog and questtracker are the largest case of this and the easiest to
    misread: twelve and fifteen calls, and every one is the world map API -
    ClickLandmark, GetMapLandmarkInfo, GetMapOverlayInfo, ProcessMapClick,
    SetMapByID, ZoomOut, UpdateMapHighlight, the debug pair - which is absent
    on purpose because this client draws its own map, as the note above says.
    They reach it through one hop: both frames call into worldmapframe.lua.
    The one name that looks different, GetQuestLogItemDrop, is in that file too
    and sits inside `for i = 1, GetNumQuestItemDrops(...)`, which answers zero.
    Twenty-seven names, one decision already taken.

    Calls need the stronger test, because an unanswered call raises where an
    unfired event only goes unheard. Ask whether it is *reachable*, not whether
    the feature exists. mainmenubar's four vehicle calls live in
    vehiclemenubar.lua - one hop away, which is why they are counted here - and
    every one sits behind a guard that a client with no vehicles never passes:
    UnitInVehicle gates the path and UnitVehicleSkin answers nil, so the
    indicator is zero and the function returns before the call. Unreachable, as
    GetMapLandmarkInfo is behind GetNumMapLandmarks answering zero.

    minimap's Wintergrasp pair is the same, and took two hops to see. Both
    BattlefieldMgr calls sit inside `for i=1, MAX_WORLD_PVP_QUEUES`, and the
    status that gates them comes from GetWorldPVPQueueStatus, which answers nil
    three times - so it is never "queued" or "confirm". The other route in is a
    static popup shown by BATTLEFIELD_MGR_ENTRY_INVITE, which is never fired.
    Both doors are shut, and one shut door would not have been enough.

    Where the checks stop: characterframe's last four are in
    equipmentmanager.lua, and that feature is not absent here -
    GetNumEquipmentSets answers from a real set list. A player with a saved set
    runs that code and every one of those calls raises. So the same four tests
    that cleared five elements find this one real, which is the point of
    running them rather than assuming either way.

    characterframe's three events split the same way. KNOWN_TITLES_UPDATE
    shares its branch with UNIT_NAME_UPDATE and PLAYER_DAMAGE_DONE_MODS shares
    its with UNIT_STATS; both siblings are fired, so both branches already run.
    CURSOR_UPDATE was the one that was real, and is done: all thirteen sites
    now go through setCursorType, which fires on an actual change. It was
    deferred three times as unverifiable and was not - the awkward part was
    never the count but clearCursorItem, a helper with no lua_State to fire
    from, which the compiler named in one line.

    The inverse also exists and is worth naming: announced but never read. An
    event whose handler is reachable is still pointless if what that handler
    reads is stubbed, and firing it then looks like progress while changing
    nothing. Check the values the handler consults, not just that it runs.

    (Written after getting minimap's difficulty events wrong in both
    directions: first calling them worth firing, then calling GetInstanceInfo
    a stub without reading the line where it consults getInstanceDifficulty.
    It is real, parsed from SMSG_INSTANCE_DIFFICULTY, and the events were
    worth firing after all.)
"""

import collections
import glob
import os
import re
import sys

ROOT = os.path.join(os.path.dirname(__file__), "..")
FX = os.path.join(ROOT, "Data", "interface", "framexml")
ADDONS = os.path.join(ROOT, "src", "addons")

# Their unanswered names belong to a particular popup, menu or chat command and
# are reached only when that one is used - so they are not the element's.
SHARED = {
    "staticpopup.lua", "uiparent.lua", "unitpopup.lua",
    "chatframe.lua", "globalstrings.lua",
}

# Element -> the files that are unambiguously its own.
ELEMENTS = {
    "questgiver":   ["questframe.lua", "questframe.xml"],
    "gossip":       ["gossipframe.lua", "gossipframe.xml"],
    "questlog":     ["questlogframe.lua", "questlogframe.xml"],
    "mail":         ["mailframe.lua", "mailframe.xml"],
    "taxi":         ["taxiframe.lua", "taxiframe.xml"],
    "loot":         ["lootframe.lua", "lootframe.xml"],
    "vendor":       ["merchantframe.lua", "merchantframe.xml"],
    "bank":         ["bankframe.lua", "bankframe.xml"],
    "questtracker": ["watchframe.lua", "watchframe.xml"],
    "readycheck":   ["readycheck.lua", "readycheck.xml"],
    "trade":        ["tradeframe.lua", "tradeframe.xml"],
    "raidwarning":  ["raidwarning.lua", "raidwarning.xml"],
    "uierrors":     ["uierrorsframe.lua", "uierrorsframe.xml"],
    "dungeonfinder": ["lfdframe.lua", "lfdframe.xml", "lfrframe.lua",
                      "lfrframe.xml", "lfgframe.lua", "lfgframe.xml"],
    "help":         ["helpframe.lua", "helpframe.xml"],
    "social":       ["friendsframe.lua", "friendsframe.xml"],
    "partyframes":  ["partyframe.xml", "partyframetemplates.xml"],
    "micromenu":    ["mainmenubarmicrobuttons.lua", "mainmenubarmicrobuttons.xml"],
    "bagbar":       ["mainmenubarbagbuttons.lua", "mainmenubarbagbuttons.xml"],
    "gamemenu":     ["gamemenuframe.xml"],
    "worldmap":     ["worldmapframe.lua", "worldmapframe.xml"],
    # chatframe.lua is deliberately not here - it is in SHARED, because it
    # defines the slash commands the whole interface uses and its unanswered
    # names belong to whichever command was typed rather than to the chat
    # window. Its .xml is a different matter: that is the frame itself.
    #
    # voicechat.lua is left out too, and that one is a decision rather than a
    # scoping detail. There is no voice chat here and there is not going to be,
    # so every name it wants is correctly absent; counting them would put a
    # permanent floor under this element's number and make it unreadable.
    "chat":         ["floatingchatframe.lua", "floatingchatframe.xml",
                     "chatframe.xml", "chatconfigframe.lua", "chatconfigframe.xml"],
    # These four are named in framexml_takeover.cpp and were measured by
    # nothing, which is the same hole chat sat in. An element the transition can
    # be asked to hand over and that no report covers is worse than one with a
    # known gap: it reads as ready by absence.
    "bgscore":      ["worldstateframe.lua", "worldstateframe.xml"],
    "stable":       ["petstable.lua", "petstable.xml"],
    "book":         ["itemtextframe.lua", "itemtextframe.xml"],
    "totems":       ["totemframe.lua", "totemframe.xml"],
    # The twelve this client already hands over by default, and which nothing
    # had ever measured. Being enabled is not evidence of being complete - it
    # means someone once saw them draw, which is the visual half. An unanswered
    # call in one of these is a live fault in the shipping default, not a
    # candidate for a future round.
    "playerframe":  ["playerframe.lua", "playerframe.xml"],
    "targetframe":  ["targetframe.lua", "targetframe.xml"],
    "focusframe":   ["focusframe.lua", "focusframe.xml"],
    "petframe":     ["petframe.lua", "petframe.xml"],
    "castbar":      ["castingbarframe.lua", "castingbarframe.xml"],
    "buffs":        ["buffframe.lua", "buffframe.xml"],
    "minimap":      ["minimap.lua", "minimap.xml"],
    # All five tabs, not just the paperdoll. CHARACTERFRAME_SUBFRAMES
    # (characterframe.lua:1) names PaperDollFrame, PetPaperDollFrame,
    # SkillFrame, ReputationFrame and TokenFrame, and ToggleCharacter reaches
    # any of them. Listing only the paperdoll left four tabs of a *default*
    # element unscanned - the expansion that follows calls out of a root does
    # not find them, because sibling tabs do not call each other.
    "characterframe": ["characterframe.lua", "characterframe.xml",
                       "paperdollframe.lua", "paperdollframe.xml",
                       "petpaperdollframe.lua", "petpaperdollframe.xml",
                       "skillframe.lua", "skillframe.xml",
                       "reputationframe.lua", "reputationframe.xml",
                       "tokenframe.lua", "tokenframe.xml"],
    "bags":         ["containerframe.lua", "containerframe.xml"],
    "spellbook":    ["spellbookframe.lua", "spellbookframe.xml"],
    "durability":   ["durabilityframe.lua", "durabilityframe.xml"],
    "mainmenubar":  ["mainmenubar.lua", "mainmenubar.xml"],
}

# Elements whose frames arrive with a load-on-demand addon rather than with
# FrameXML. Their whole directory is the element.
ADDON_ELEMENTS = {
    "achievements": "blizzard_achievementui",
    "auctionhouse": "blizzard_auctionui",
    "barbershop":   "blizzard_barbershopui",
    "guildbank":    "blizzard_guildbankui",
    "inspect":      "blizzard_inspectui",
    "talents":      "blizzard_talentui",
    "tradeskill":   "blizzard_tradeskillui",
    "macro":        "blizzard_macroui",
    "keybindings":  "blizzard_bindingui",
    "timemanager":  "blizzard_timemanager",
    "classtrainer": "blizzard_trainerui",
}

# A handler body in XML is Lua, and holds calls that appear nowhere in any .lua.
# GuildControlSetRankFlag and TakeInboxTextItem are both only ever called from
# one, and a scan of the Lua alone reported their frames complete.
# The element names here have to be the ones framexml_takeover.cpp uses, and
# there is nothing but this check to keep them so. They drifted once: this file
# called the vendor "merchant" after the frame, framexml_takeover.cpp calls it
# "vendor" after the element, and a list of elements to hand over was written
# from a reading of this report - so "merchant" went into that list, resolved
# to no element, and the vendor window was never handed over by it. The run
# said so every time, in one warning line among many.
#
# Names checked against the takeover rather than restated, because restating is
# how the two came apart.
def _element_names():
    path = os.path.join(ROOT, "src", "ui", "framexml_takeover.cpp")
    with open(path, errors="ignore") as fh:
        src = fh.read()
    return set(re.findall(r'\{UiElement::\w+,\s*"([a-z]+)"', src))


def check_element_names():
    """Names in this file that are not elements over in the takeover."""
    known = _element_names()
    # Four are deliberately not elements. The keybinding, macro and clock
    # panels have no counterpart this client draws, so there was nothing to
    # hand over and nothing to suppress; "mainmenubar" is a file grouping - the
    # bar and the strips around it are four separate elements over there and
    # one .lua/.xml pair here. All four are read for gaps all the same.
    tool_only = {"keybindings", "macro", "timemanager", "mainmenubar"}
    return sorted((set(ELEMENTS) | set(ADDON_ELEMENTS)) - known - tool_only)


SCRIPT_BODY = re.compile(r"<(On[A-Za-z]+)>(.*?)</\1>", re.S)
SCRIPT_ATTR = re.compile(r'<On[A-Za-z]+\s+function="([A-Za-z_][\w]*)"')
CALL = re.compile(r"(?<![\w.:])([A-Z][A-Za-z0-9_]*)\s*\(")

# A Lua pattern in a string looks exactly like a call. gsub(point, "TOP(.*)",
# "BOTTOM%1") reads as a call to TOP, and did - it was three of the loot
# frame's five remaining names. Comments do the same for anything written as
# Name() in prose.
_STRINGS = re.compile(r'"(?:[^"\\\n]|\\.)*"' r"|'(?:[^'\\\n]|\\.)*'")
_COMMENT = re.compile(r"--\[\[.*?\]\]|--[^\n]*", re.S)
_XML_COMMENT = re.compile(r"<!--.*?-->", re.S)


def code_only(src):
    """The source with string literals and comments blanked out."""
    src = _COMMENT.sub(" ", src)
    return _STRINGS.sub('""', src)


def events_fired():
    """Every event name the client can send, taken from the C++ that sends them."""
    names = set()
    # include/ as well as src/. Plenty of this client's small state changes are
    # inline in a header - closeStableWindow fires PET_STABLE_CLOSED from
    # game_handler.hpp - and scanning only src/ reported those events as never
    # sent, which is the one thing this column is supposed to be trusted on.
    roots = [os.path.join(ROOT, "src"), os.path.join(ROOT, "include")]
    for root in roots:
      for dirpath, _, filenames in os.walk(root):
        for fn in filenames:
            if not fn.endswith((".cpp", ".hpp")):
                continue
            src = open(os.path.join(dirpath, fn), encoding="utf-8", errors="ignore").read()
            names |= set(re.findall(r'"([A-Z][A-Z0-9_]{3,})"', src))
    return names


def events_registered():
    """file -> the events its frames ask for."""
    want = collections.defaultdict(set)
    for path in glob.glob(os.path.join(FX, "*.lua")) + glob.glob(os.path.join(FX, "*.xml")):
        name = os.path.basename(path)
        src = open(path, encoding="utf-8", errors="ignore").read()
        want[name] |= set(re.findall(r'RegisterEvent\(\s*"([A-Z][A-Z0-9_]+)"', src))
        want[name] |= set(re.findall(r'<Event\s+name="([A-Z][A-Z0-9_]+)"', src))
    return want


def registered():
    """Every global name the client answers.

    This implementation moved to framexml_provides, which is now the one place
    that decides - six other sweeps had worked it out for themselves and only
    this one was right, because only this one read the bootstrap Lua.
    """
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from framexml_provides import globals_provided
    return globals_provided()


def scan_interface():
    """function -> defining file, and file -> names it calls.

    Skips Data/interface/gluexml. That is the login screen, a separate
    interface in its own Lua state, and it defines functions under names the
    in-game one also uses - so a one hop out of an in-game file was landing in
    glue code and reporting its account-message and credits calls as gaps in
    the quest log, the quest tracker, the social frame and the help frame.
    """
    defined_by, calls = {}, collections.defaultdict(set)
    for dirpath, _, filenames in os.walk(os.path.join(ROOT, "Data", "interface")):
        if "gluexml" in dirpath.replace("\\", "/").split("/"):
            continue
        for name in filenames:
            path = os.path.join(dirpath, name)
            if name.endswith(".lua"):
                src = code_only(open(path, encoding="utf-8", errors="ignore").read())
                for fn in re.findall(
                        r"^\s*(?:local\s+)?function\s+([A-Za-z_][\w]*)\s*\(", src, re.M):
                    defined_by.setdefault(fn, name)
                for fn in re.findall(r"^\s*([A-Za-z_][\w]*)\s*=\s*function", src, re.M):
                    defined_by.setdefault(fn, name)
                calls[name] |= set(CALL.findall(src))
            elif name.endswith(".xml"):
                # XML comments first. A whole widget can be commented out and
                # its <OnClick> still matches, which reported skillframe.xml's
                # sort button - a control that has not existed since whoever
                # commented it out. Blanking Lua comments inside the body does
                # not help; the body should not have been read at all.
                src = _XML_COMMENT.sub(" ",
                                       open(path, encoding="utf-8", errors="ignore").read())
                for body in SCRIPT_BODY.finditer(src):
                    calls[name] |= set(CALL.findall(code_only(body.group(2))))
                # A handler can be *named* instead of written out:
                # <OnClick function="Foo"/> calls Foo just as surely as
                # <OnClick>Foo()</OnClick>, and reading only the bodies missed
                # it. That is how the barber shop was reported finished while
                # ApplyBarberShopStyle -- its Okay button, and the one action
                # that commits anything -- was bound by nothing at all.
                calls[name] |= set(SCRIPT_ATTR.findall(src))
    return defined_by, calls


def main():
    have = registered()
    defined_by, calls = scan_interface()
    fired = events_fired()
    wants = events_registered()

    print("element        files  calls  events")
    ready = []
    for element, roots in sorted(ELEMENTS.items()):
        files = set(roots)
        for root in roots:
            for name in calls.get(root, ()):
                owner = defined_by.get(name)
                if owner and owner not in SHARED:
                    files.add(owner)

        missing = set()
        for f in files:
            for name in calls.get(f, ()):
                if name in have or name in defined_by:
                    continue
                # Battle.net has no counterpart on a 3.3.5 server.
                if name.startswith("BN"):
                    continue
                missing.add(name)

        # Only the element's own files: an event another file asks for belongs
        # to that file's element, not to this one.
        want = set()
        for root in roots:
            want |= wants.get(root, set())
        # Battle.net has no counterpart on a 3.3.5 server.
        dead = sorted(e for e in want if e not in fired and not e.startswith("BN_"))

        # Both caps generous, because a hidden name is worse than a long line:
        # WHO_LIST_UPDATE was a real gap that sat unprinted as social's fourth
        # event, and mainmenubar's seventh and eighth went unchecked while its
        # settled entry claimed otherwise.
        listed = " ".join(sorted(missing)[:16] + dead[:12])
        print(f"  {element:<13} {len(files):>3}   {len(missing):>4}   {len(dead):>4}  {listed}")
        if not missing and not dead:
            ready.append(element)

    for element, addon in sorted(ADDON_ELEMENTS.items()):
        root = os.path.join(ROOT, "Data", "interface", "addons", addon)
        if not os.path.isdir(root):
            continue
        missing, nfiles = set(), 0
        for dirpath, _, filenames in os.walk(root):
            for fn in filenames:
                if not fn.endswith((".lua", ".xml")):
                    continue
                nfiles += 1
                for name in calls.get(fn, ()):
                    if name in have or name in defined_by or name.startswith("BN"):
                        continue
                    missing.add(name)
        listed = " ".join(sorted(missing)[:6])
        print(f"  {element:<13} {nfiles:>3}   {len(missing):>3}  {listed}")
        if not missing:
            ready.append(element)

    total = len(ELEMENTS) + len(ADDON_ELEMENTS)
    print()
    print(f"{len(ready)} of {total} with no known gaps: {' '.join(sorted(ready))}")



    # Elements whose remaining names have each been read and found not to be
    # work. They still show a count above, because this cannot tell an absent
    # feature from a gap - that is the judgement, and it is written down here
    # so the headline number stops understating what is finished.
    #
    # Every entry names why. Re-check one if its reason stops being true.
    #
    # [checked] marks one re-verified since it was written. All eleven now are.
    # Five were wrong or incomplete, and three of those five were concealing a
    # real gap - GMRESPONSE_RECEIVED and PLAYER_ROLES_ASSIGNED are fired now,
    # MINIMAP_UPDATE_TRACKING is recorded with its trigger.
    #
    # The server source is wherever WOWEE_SERVER_SRC points. Anything blocked here
    # on "the wire format is not known" can be read off it directly - that was
    # not noticed until the end of a session spent declining to guess, and the
    # scoreboard parser turned out to have three misalignments the moment it
    # was checked against Battleground.cpp.
    #
    # Five in eleven is the number to remember when writing the next one. The
    # five that held were each grepped while being written; the five that did
    # not were carried forward from an earlier conclusion. Provenance predicted
    # it better than confidence did, every time.
    #
    # And re-check them anyway. Three have been spot-checked since this list
    # was written and two were wrong: worldmap named one event of two, and help
    # called GMRESPONSE_RECEIVED unparsed when it is parsed in full and only
    # ever said in chat - that one was a real gap sitting inside a line
    # claiming there was none. The reasons are worth more than the count
    # precisely because they can be checked; treat an unchecked entry as a
    # claim rather than a finding.
    #
    # Four checked, and the split is clean: the two that were wrong were both
    # carried forward from an earlier assertion, and the two that held were
    # both derived from a grep run while writing the entry. bgscore and mail
    # stood up; worldmap and help did not. So the question to ask of an entry
    # is not how confident it sounds but whether anyone looked while writing
    # it - which is also why each one names a file or a function rather than a
    # conclusion.
    #
    # bgscore later failed for a different reason, worth its own warning: the
    # entry was accurate and the reasoning was still wrong. "Unreachable
    # because the binding it loops over answers zero" clears the calls by
    # pointing at a stub, and a stub answering empty is not evidence a feature
    # is absent - it is how a feature gets switched off without anyone
    # noticing. When a claim rests on what another binding returns, check that
    # the return is a real answer and not a placeholder.
    settled = {
        "characterframe": "[checked] the paperdoll's own are the stat branch shared with UNIT_STATS, which is fired, plus SHOW_COMPARE_TOOLTIP - absent by design, the C client fires it on a shift-hover to open a comparison tooltip and this client has no item comparison at all. The rest arrived with the other four tabs, which this entry did not cover until their files were added: the three COMPANION_* and the two PET_* belong to the companions and mounts list on the pet tab, and all three COMPANION_ ones are fired - this note used to call the list the one real gap here, on the grounds that it needed mount and critter classification out of Spell.dbc, and rebuildCompanions had been doing that since before the note was written. The preview draws now too; DISABLE/ENABLE_XP_GAIN is the WotLK experience-lock NPC, which this client has no path to; PLAYER_PVP_RANK_CHANGED is the vanilla honor rank, gone by 3.3.5. The four remaining calls are AddSkillUp, RemoveSkillUp and BuySkillTier - the skill-point purchase panel, unreachable because GetSkillLineInfo answers nil for stepCost and rankCost - and GetText, which is a widget method rather than a global",
        "book":         "[checked] ITEM_TEXT_TRANSLATION carries a translation timer nothing derives; the text itself does arrive - ItemHandler.cpp builds SMSG_ITEM_TEXT_QUERY_RESPONSE",
        "bags":         "[checked] BAG_OPEN/CLOSED are for a C client opening bags; ToggleBag, OpenBag and CloseBag are all Lua functions in containerframe.lua",
        "vendor":       "[checked] GUILDBANK_UPDATE_MONEY shares its branch with PLAYER_MONEY, fired from inventory_handler and entity_controller",
        "playerframe":  "[checked] voice chat, vehicles and the playtime nag (PLAYER_ROLES_ASSIGNED was wrong here - roles are parsed and read, and it is fired now)",
        "mainmenubar":  "[checked] nine calls and five events are vehicles; CURRENCY_DISPLAY_UPDATE and UPDATE_BONUS_ACTIONBAR share fired branches, UPDATE_MULTI_CAST_ACTIONBAR shares one and has nil data besides",
        "minimap":      "[checked] four calls unreachable; zoom is widget state, movie recording absent, indoors redundant - MINIMAP_UPDATE_TRACKING was real and is fired now, from the player aura change - tracking is an aura, so that one site covers both routes",
        "bgscore":      "[fixed] GetWorldStateUIInfo and IsSubZonePVPPOI are bound, and GetNumWorldStateUI answers from the battleground table in game/bg_score_defs.hpp - the earlier note here cleared them as unreachable *because* that stub answered zero, which was the bug rather than the clearance: an empty answer had switched WorldStateAlwaysUpFrame off entirely. IsSubZonePVPPOI still answers false and the check below still names it: it sits behind `uiType ~= 1`, which short-circuits true for every entry that table holds, so it is unreached rather than answered - and it would matter the day a uiType 1 entry appears",
        "mail":         "[checked] both are the refund lock, which the C client raises when a still-refundable item is attached. The refund window is a per-item timer the server sends and this client does not keep - GetContainerItemPurchaseInfo answers nil for exactly that reason, and mailframe.lua only reaches the lock through it. Correctly absent rather than unfired: firing it would mean claiming a refund window that is not tracked",
        "questlog":     "[checked] every call is the world map API, absent because this client draws its own",
        "dungeonfinder": "[checked] four of the five are correctly absent. LFG_OPEN_FROM_GOSSIP's source, SMSG_OPEN_LFG_DUNGEON_FINDER, is STATUS_NEVER in AzerothCore and never sent. UPDATE_LFG_LIST is the raid browser, whose three search packets are read and dropped here by decision - this client's own browser is as empty as FrameXML's would be. LFG_ROLE_UPDATE refreshes role checkboxes, which are client state. VOTE_KICK_REASON_NEEDED needs a message this client is not sent. The fifth, LFG_QUEUE_STATUS_UPDATE, was a real gap and is fired now",
        "uierrors":     "[checked] SYSMSG is the one unfired event and nothing backs it - no opcode this client handles produces one, and the event-gap report finds no source message for it. UI_ERROR_MESSAGE, which is what the frame is actually for, is raised by addUIError from eighty sites",
        "trade":        "[checked] TRADE_PLAYER_ITEM_CHANGED and TRADE_TARGET_ITEM_CHANGED are the two unfired, and correctly so: they carry one slot each, and this client never learns of a single slot changing. SMSG_TRADE_STATUS_EXTENDED carries a whole side at once, which is what TRADE_UPDATE is fired from, and its branch calls TradeFrame_Update - a full redraw of every slot. TRADE_POTENTIAL_BIND_ENCHANT is handled by a commented-out body in FrameXML itself",
        "questtracker": "[checked] twelve are the same world map API through worldmapframe.lua; the other three are AchievementFrame internals, defined in blizzard_achievementui and absent only until it loads",
        "worldmap":     "[checked] map API is this client's; WORLD_MAP_NAME_UPDATE has no handler branch, CLOSE_WORLD_MAP needs the key to drive Lua",
        "help":         "[checked] GMRESPONSE_RECEIVED is parsed and fired, and TicketMgr.cpp does "
                        "send it. GMSURVEY_DISPLAY is unfired but NOT unbackable, which this "
                        "note used to claim: the survey's questions come from four DBCs this "
                        "install carries - GMSurveyCurrentSurvey maps language to survey, "
                        "GMSurveySurveys lists up to ten question ids, GMSurveyQuestions and "
                        "GMSurveyAnswers hold the text - and the trigger is the getSurvey byte "
                        "in SMSG_GMRESPONSE_STATUS_UPDATE. Not built: submitting means "
                        "accumulating ten answers with per-question comments for "
                        "CMSG_GMSURVEY_SUBMIT, and the panel appears only after a GM closes a "
                        "ticket. Absent by choice, which is a different thing from absent by "
                        "necessity",
        "social":       "[checked] MUTELIST_UPDATE shares both its branches with IGNORELIST_UPDATE (friendsframe.lua:1224, partymemberframe.lua:341), which is fired from social_handler.cpp:2567; VOICE_CHAT_ENABLED_UPDATE is voice chat, which this client has none of",
        "achievements": "[checked] all three are the achievement addon's own and exist once it loads - AchievementFrameTab_OnClick is assigned rather than declared (blizzard_achievementui.lua:63), which is why grepping for 'function' finds nothing. The one use in core FrameXML, alertframes.lua:260, compares ACHIEVEMENTUI_SELECTEDFILTER rather than calling it, and sits after ShowUIPanel(AchievementFrame) so the addon is loaded by then",
        "auctionhouse": "[checked] DressUpItemLink_orig is a local capturing DressUpItemLink for a hook, and DressUpItemLink is a FrameXML Lua function (dressupframe.lua:2) rather than a C binding - so the capture gets the real one and nothing is missing",
    }
    also = [e for e in settled if e not in ready]
    if also:
        print()
        print(f"{len(also)} more read and settled - the count above cannot see this:")
        for e in sorted(also):
            print(f"  {e:<13} {settled[e]}")
        print()
        print(f"{len(ready) + len(also)} of {total} finished, on that reading.")

    # A settled entry that rests on a stub.
    #
    # There are two questions to ask of one of these, not one: did anyone look,
    # and does the reason rest on a placeholder. The second cost a real gap -
    # bgscore was cleared because "both calls sit in a loop over
    # GetNumWorldStateUI, which answers zero", which was true and was not
    # evidence. The zero was lua_ReturnZero. It was not that the feature was
    # absent; it was how the feature had been switched off, and
    # WorldStateAlwaysUpFrame had been live and empty the whole time.
    #
    # So every binding an entry names by way of explanation is checked against
    # the stub list. A hit is not a fault by itself - the reason may hold for
    # other reasons too - but it is the shape that reads as settled and is not.
    stubs = ("lua_ReturnZero", "lua_ReturnNil", "lua_ReturnFalse",
             "lua_ReturnNothing", "lua_ReturnTrue", "lua_ContainerFalse",
             "lua_ContainerNoOp", "lua_GetZeroMoney")
    addon_src = ""
    for name in sorted(glob.glob(os.path.join(ROOT, "src/addons", "*.cpp"))):
        with open(name, errors="ignore") as fh:
            addon_src += fh.read()
    named = set()
    for entry in settled.values():
        named |= set(re.findall(r"\b(?:Get|Is|Can|Has)[A-Z]\w+", entry))
    resting = []
    for name in sorted(named):
        hit = re.search(r'\{\s*"' + name + r'"\s*,\s*(\w+)\s*\}', addon_src)
        if hit and hit.group(1) in stubs:
            owner = [e for e, t in settled.items() if name in t]
            resting.append((name, hit.group(1), owner))
    print()
    print(f"{len(resting)} settled entry reason(s) resting on a stub:")
    for name, stub, owner in resting:
        print(f"  {name:30} {stub:20} named by {', '.join(owner)}")
    if not resting:
        print("  (none)")

    # No tier to print any more, and no element left to promote into one.
    #
    # This used to end by printing the contents of framexml_takeover.cpp's
    # "candidates" array - the elements clean enough to try - and a
    # WOWEE_FRAMEXML_UI line for trying one. Both are gone: every element is
    # handed over unconditionally and the flag that named them has been
    # removed. What is above is the whole report now, and it is read for gaps
    # in an interface already in use rather than for permission to switch one
    # on.


if __name__ == "__main__":
    # Before anything else, because a name this file scores that names no
    # element is a row of the report about nothing, and it reads exactly like
    # a row about something.
    unknown = check_element_names()
    if unknown:
        print("These element names are not elements in framexml_takeover.cpp, "
              "so nothing they are said about is about the interface:")
        for name in unknown:
            print(f"  {name}")
        print()
    main()
