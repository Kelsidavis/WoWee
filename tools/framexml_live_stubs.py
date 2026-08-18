#!/usr/bin/env python3
"""Stubbed bindings that a handed-over element actually calls.

The readiness report counts a name as answered once it is bound, and a stub is
bound - so `HasPetSpells -> lua_ReturnNil` read as clean while it was hiding
the entire pet spell book. This asks the narrower question: which of the stubs
are on a code path that is on screen right now.

Being listed is not being wrong. Plenty of stubs are correct: the feature is
genuinely absent (vehicles, voice chat) or FrameXML tolerates the empty answer.
It is a reading list, ordered by how central the file is.

WHAT IT COULD NOT SEE UNTIL 2026-08-06

A stub written out in place. Only the named shared ones were recognised -
lua_ReturnNil and its family - so a lambda answering a literal from inside the
registration table counted as an implementation, which is exactly what a stub
is not. The count went from thirty-nine to a hundred and ninety-five.

The test is now that stripping the pushes and the return leaves nothing that
calls anything. Asking it the other way round - which sources does this body
read - kept failing in the same direction: GetActionBarPage keeps its page in a
Lua global, GetSelectedFaction and GetSelectedSkill keep theirs in C++ statics
behind an accessor, and all three are implementations that simply do not go
through the game handler.

Two of the newly visible are worth naming: GetQuestLogCompletionText answers ""
and GetQuestLogRequiredMoney answers 0, both to the quest log and the tracker.
Answering them needs SMSG_QUEST_QUERY_RESPONSE parsed further than it is - the
quest log entry carries neither field, and the packet that does carry them is
the turn-in one, which arrives only once the player is at the NPC.

THE FORTY THAT WERE CHECKED. The hundred and fifty-six that joined them have
not been.

THE FORTY IT REPORTS TODAY, ALL CHECKED

None is a defect, but three are worth knowing about because they are visible
rather than merely absent. Read once so the count can be watched rather than
re-triaged.

VISIBLE, AND DELIBERATE

  * GetBattlefieldInstanceRunTime - the scoreboard prints "Time Elapsed: 0
    seconds" rather than nothing, because worldstateframe.lua formats whatever
    it is given. The real answer is milliseconds since the *instance* started,
    which this client cannot know: it can measure since the player joined, and
    labelling that "Time Elapsed" would be confidently wrong for anyone who
    arrived late. A zero that reads as broken is better than a number that
    reads as true.
  * ShowContainerSellCursor, ShowBuybackSellCursor - the cursor does not change
    to the sell icon over a bag item at a vendor. This client's cursor has no
    such icon to change to.

ONE THAT IS ABSENT BY CHOICE RATHER THAN BY NECESSITY

The GM survey. Its questions come from four DBCs this install carries, not from
any packet - GMSurveyCurrentSurvey maps language to survey, GMSurveySurveys
lists the question ids, GMSurveyQuestions and GMSurveyAnswers hold the text -
and the trigger is the getSurvey byte in SMSG_GMRESPONSE_STATUS_UPDATE.
Submitting is what makes it work rather than merely appear, and that means
accumulating ten answers with per-question comments for CMSG_GMSURVEY_SUBMIT.
The panel is only reached after a game master closes a ticket.

Recorded because this file previously said the questions were unparseable,
which was read off the event's name rather than off the files sitting beside
it.

TWO THAT WERE CHECKED AND ARE GENUINELY ABSENT

Written down because three neighbouring claims of the same kind turned out to
be wrong - the refund window, the GM survey and vehicle state were all called
absent and all three were reachable. These two are not.

  * Voice chat. AzerothCore's handlers read the request and throw it away:
    HandleVoiceSessionEnableOpcode is two read_skips, HandleSetActiveVoiceChannel
    another two, HandleChannelVoiceOnOpcode an empty body with a comment. No
    SMSG_VOICE_* is ever sent, so there is no session to report on and nothing
    a binding could answer from.
  * Movie recording. The renderer can capture one frame - Renderer::captureScreenshot
    writes a PNG - and there is no encoder behind it. MovieRecording_* is video.

ABSENT FEATURES, WHICH IS WHAT THE STUB SAYS (28)

The world map's debug objects, zone map, battlefield flag and vehicle
positions, dungeon map floors and Wintergrasp timer; Battle.net and its friend
list; voice chat and mutes; movie recording; mail stationery; achievement
comparison; arena opponents; the multi-cast bar's offset; addon memory usage.
Each is a feature this client does not have, and the stub is the honest shape
of that.

CORRECT ANSWERS THAT LOOK LIKE STUBS (9)

IsMacClient is false because it is not one. GetAdjustedSkillPoints is zero
because WotLK has no skill points - skillframe.lua gates every purchase verb
on it, which is why BuySkillTier and AddSkillUp are unreachable rather than
unimplemented. GetCurrentMapDungeonLevel is zero because a map with no floors
is on floor zero.
"""
import re
from pathlib import Path

ROOT = Path("/home/k/Desktop/wowee")
XML = ROOT / "Data/interface"

STUBS = {"lua_ReturnNil", "lua_ReturnZero", "lua_ReturnFalse", "lua_ReturnNothing",
         "lua_ReturnTrue", "lua_ReturnEmptyString", "lua_ContainerNoOp",
         "lua_ContainerFalse", "lua_NoOp", "lua_Noop"}

# The FrameXML files that belong to elements handed over by default.
# Derived from the takeover file rather than written out: the elements handed
# over by default plus the candidates tier, mapped to files through the
# readiness tool's table, plus the shared files every panel goes through.
#
# It was a hand-made list and twice that was the bug - it named
# paperdollframe.lua as "the character sheet" and missed the other four
# subframes, and it covered only the defaults while the candidates tier was
# what was actually on screen.
def _live_files():
    import re as _re
    tk = (ROOT / "src/ui/framexml_takeover.cpp").read_text()
    defaults = set(_re.findall(r'"([a-z]+)"',
        _re.search(r"return std::set<std::string>\{(.*?)\};", tk, _re.S).group(1)))
    # The candidates tier used to add a list on top of the defaults and adds
    # nothing now - every element is in the defaults, so the loop it was read
    # out of is gone. Read as empty rather than crashing, which is what this
    # did between the loop being removed and 2026-08-05: nothing runs this
    # sweep from the build, so nothing noticed.
    _cand_loop = _re.search(r"for \(const char\* name : \{(.*?)\}\)", tk, _re.S)
    cand = set(_re.findall(r'"([a-z]+)"', _cand_loop.group(1))) if _cand_loop else set()
    rd = (ROOT / "tools/framexml_element_readiness.py").read_text()
    # Parsed as data, not run as code. These are two plain dict literals in a
    # sibling sweep, and exec() on a slice of another file is both more than
    # this needs and a finding in its own right: it hands whatever that regex
    # happened to match to the interpreter. literal_eval accepts a literal and
    # nothing else, so a stray call or import in there is an error rather than
    # something that runs.
    import ast as _ast
    ns = {}
    for _name in ("ELEMENTS", "ADDON_ELEMENTS"):
        _m = _re.search(r"^%s = (\{.*?^\})" % _name, rd, _re.S | _re.M)
        if not _m:
            raise SystemExit(f"{_name} is no longer a dict literal in "
                             "framexml_element_readiness.py - this sweep reads it as one")
        ns[_name] = _ast.literal_eval(_m.group(1))
    out = {}
    for el in defaults | cand:
        for f in ns["ELEMENTS"].get(el, []):
            out[f] = el
        addon = ns["ADDON_ELEMENTS"].get(el)
        if addon:
            d = XML / "addons" / addon
            if d.exists():
                for p in d.rglob("*"):
                    if p.suffix in (".lua", ".xml"):
                        out[p.name] = el
    for f, el in {
            "uiparent.lua": "dialogs", "staticpopup.lua": "dialogs",
            "unitpopup.lua": "unitframes", "unitframe.lua": "unitframes",
            "targetframe.lua": "targetframe", "playerframe.lua": "playerframe",
            "actionbutton.lua": "mainmenubar", "bonusactionbarframe.lua": "mainmenubar",
            "mainmenubar.lua": "mainmenubar", "mainmenubarbagbuttons.lua": "bagbar",
            "containerframe.lua": "bags", "paperdollframe.lua": "characterframe",
            "skillframe.lua": "characterframe/skills",
            "reputationframe.lua": "characterframe/rep",
            "tokenframe.lua": "characterframe/currency",
            "petpaperdollframe.lua": "characterframe/pet",
            "characterframe.lua": "characterframe", "spellbookframe.lua": "spellbook",
            "petframe.lua": "petframe", "petactionbarframe.lua": "petframe",
            "buffframe.lua": "buffs", "castingbarframe.lua": "castbar",
            "durabilityframe.lua": "durability", "zonetext.lua": "zonetext",
            "minimap.lua": "minimap", "mirrortimer.lua": "playerframe",
            "focusframe.lua": "focusframe", "gametooltip.lua": "tooltips",
            "itembuttontemplate.lua": "shared"}.items():
        out.setdefault(f, el)
    return out


OWNED = _live_files()

bound = {}
inline_stub = {}
for f in (ROOT / "src/addons").glob("*.cpp"):
    s = f.read_text(errors="ignore")
    for m in re.finditer(r'\{(?:\.\w+\s*=\s*)?"([A-Za-z0-9_]+)",\s*(?:\.\w+\s*=\s*)?(?:&)?\s*(lua_[A-Za-z0-9_]+)\}', s):
        bound[m.group(1)] = m.group(2)
    # The inline form. A binding registered as a lambda in the table is the
    # same binding to Lua, but only the named shared stubs - lua_ReturnNil and
    # its family - were recognised here, so a stub written out in place was
    # counted as an implementation. It is a stub by the same test: it never
    # reaches the game and never reads what it was passed, so it answers the
    # same thing every time it is called.
    for m in re.finditer(r'\{(?:\.\w+\s*=\s*)?"([A-Za-z0-9_]+)",\s*(?:\.\w+\s*=\s*)?\[\]\(lua_State\*\s*L?\s*\)\s*->\s*int\s*\{', s):
        depth, i = 1, m.end()
        while i < len(s) and depth:
            if s[i] == "{":
                depth += 1
            elif s[i] == "}":
                depth -= 1
            i += 1
        body = s[m.end():i - 1]
        name = m.group(1)
        bound.setdefault(name, name)
        # A stub answers with literals and consults nothing to do it. Asking
        # instead which *sources* a body reads kept getting this wrong in the
        # same direction: GetActionBarPage keeps its page in a Lua global,
        # GetSelectedFaction and GetSelectedSkill in C++ statics behind an
        # accessor, and all three are implementations that simply do not go
        # through the game handler. So the test is the other way round - strip
        # the pushes and the return, and a stub has nothing left that calls
        # anything.
        # Only the call *token* is removed, not what it was passed - otherwise
        # lua_pushnumber(L, selectedFaction()) loses the accessor inside it and
        # reads as a literal. luaReturnNil goes with them: it is how a binding
        # says nil, not something it consults.
        rest = re.sub(r"lua_push\w*\s*\(|luaReturnNil\s*\(|\breturn\b|\(void\)\s*L\s*;",
                      "", body)
        if not re.search(r"\w+\s*\(", rest):
            inline_stub[name] = "(inline)"

stubbed = {n for n, impl in bound.items() if impl in STUBS} | set(inline_stub)

hits = {}
for fname, element in OWNED.items():
    matches = list(XML.rglob(fname))
    if not matches:
        continue
    text = matches[0].read_text(errors="ignore")
    for name in stubbed:
        if re.search(rf"\b{re.escape(name)}\s*\(", text):
            hits.setdefault(name, []).append(f"{element}:{fname}")

print(f"{len(bound)} bindings, {len(stubbed)} of them stubs, "
      f"{len(hits)} reached from a handed-over element\n")
for name in sorted(hits, key=lambda n: (-len(hits[n]), n)):
    print(f"  {name:<34} {bound[name]:<20} {', '.join(sorted(set(hits[name])))}")
