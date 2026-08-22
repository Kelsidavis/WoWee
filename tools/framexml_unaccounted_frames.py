#!/usr/bin/env python3
r"""Static version of the unaccounted-frame sweep.

Known blind spot, measured rather than assumed: this reads parent="UIParent"
out of the XML, so frames built at runtime by CreateFrame are invisible to it.
Re-measured 2026-08-05 against files the loader actually opens: thirty-five
named frames are built that way and seven reach UIParent. Read against what
this client fires - the lens the main list needed - none is a duplicate:

  * ChatFrame - the literal is "ChatFrame"..i, so the real names are
    ChatFrame1..10 and all ten are suppressed by the chat element.
  * CombatLogQuickButtonFrame, CombatLogUpdateFrame - Blizzard_CombatLog is
    load-on-demand and arrives only when the combat-log tab is picked. This
    client's own combat log is behind showCombatLog_, a toggle nothing fires.
  * MacroEditBox - not a window. A hidden edit box chatframe.lua runs macro
    text through.
  * RaidPullout - built by dragging a group out of FrameXML's own raid UI.
  * WorldStateCaptureBar - cannot be built: GetWorldStateUIInfo reports uiType
    0 for every entry, which is what keeps worldstateframe.lua's world-PvP
    branch out. **This is the one to re-check after any world-state work** -
    it is the shape where finishing an API opens a window nobody asked for.

Re-measure with:

    grep -rhoP 'CreateFrame\(\s*"[A-Za-z]+"\s*,\s*"\K[A-Za-z0-9_]+' Data/interface/


Top-level FrameXML frames (parent="UIParent") whose names appear nowhere in
framexml_takeover.cpp. Each is a part of the interface nobody has decided
about: if this client draws the same thing, both are on screen.

READ IT AGAINST WHAT THE CLIENT *FIRES*, NOT AGAINST WHAT IT DRAWS

That is the mistake this report invited for months, and it is why forty-one
rows sat here unexamined. A row is only a duplicate if something can put the
FrameXML frame on screen, and for a top-level window that is almost always one
event. So the question per row is: what shows it, and does this client fire
that? Anything else - a window opened by a button in FrameXML's own panel, or
one waiting on an event nothing sends - cannot appear beside a second copy,
because it cannot appear.

Reading the thirty-seven that way on 2026-08-05 turned up one real fault, and
it was three frames of the same family. Charters: this client fires
GUILD_REGISTRAR_SHOW and PETITION_VENDOR_SHOW from one handler and
PETITION_SHOW from the next, and each raises a FrameXML window beside a popup
social_panel.cpp draws from the very same packet. Every charter bought or
signed asked twice. They are UiElement::Petition now, and the count fell to
thirty-seven.

WHAT THE THIRTY-SEVEN ARE

  * 15 dormant - nothing anywhere shows them. AutoCompleteBox, the four
    Battle.net frames, GuildControlPopupFrame, AddFriendFrame,
    FriendsFriendsFrame, SmallTextTooltip, MacOptionsCompressFrame,
    FolderPicker, StationeryPopupFrame, MovieProgressFrame, RuneFrame,
    TutorialFrameAlertButton, AutoFollowStatus.
  * 18 opened by a control in FrameXML's own interface - the chat menu, a
    dropdown's colour swatch, the game menu, the vehicle bar, an item link's
    dress-up. Reachable, and correctly so: the control that opens them belongs
    to a panel already handed over, and nothing here opens a second one on the
    same click.
  * 4 waiting on an event this client does not fire: ArenaFrame and
    BattlefieldFrame on BATTLEFIELDS_SHOW, PVPParentFrame on
    NPC_PVPQUEUE_ANYWHERE, TabardFrame on OPEN_TABARD_FRAME. None of the three
    names appears anywhere under src/ except in a comment saying so. These are
    the rows to re-read first: firing any of them is what would turn a dormant
    row into a duplicate, and nothing ties the two decisions together.
"""
import re
import sys
from pathlib import Path

import sys as _s; _s.path.insert(0, str(Path(__file__).resolve().parent))
from framexml_source import loaded_files

# The repository this file sits in, and the interface to read.
#
# ROOT was one contributor's absolute home directory, so these eight sweeps ran
# on exactly one machine and silently read nothing anywhere else - loaded_files
# on a directory that is not there returns an empty set, and a sweep with no
# input reports a clean tree.
#
# The interface directory can be named on the command line, because the one
# under Data/ is whichever expansion was extracted last and the question these
# answer is usually about a particular one:
#
#     python3 tools/framexml_unaccounted_frames.py ~/wow-1.12/interface
ROOT = Path(__file__).resolve().parent.parent
XML = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "Data/interface/framexml"
if not XML.is_dir():
    print(f"no interface at {XML} - name one on the command line")
    raise SystemExit(2)

src = (ROOT / "src/ui/framexml_takeover.cpp").read_text()
accounted = set()
for lit in re.findall(r'"([^"]*)"', src):
    for word in lit.split():
        if re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", word):
            accounted.add(word)

# <Frame name="X" ... parent="UIParent"> in either attribute order.
tops = {}
# Only files the loader opens. minigameframe, tictactoeframe, petpopup and
# questtimerframe are in no manifest and included by nothing, so the top-level
# frames they declare are never built - an element cannot be missing an entry
# for a frame that cannot exist.
for path in sorted(q for q in loaded_files(XML.parent)
                   if q.suffix.lower() == ".xml" and q.parent == XML):
    text = path.read_text(errors="ignore")
    for tag in re.finditer(r"<(Frame|Button|StatusBar|ScrollFrame|MessageFrame|"
                           r"SimpleHTML|Slider|ColorSelect|Model|PlayerModel)\b[^>]*>", text):
        blob = tag.group(0)
        if 'parent="UIParent"' not in blob:
            continue
        if 'virtual="true"' in blob:
            continue
        m = re.search(r'name="([A-Za-z][A-Za-z0-9_]*)"', blob)
        if m:
            tops.setdefault(m.group(1), path.name)

missing = {n: f for n, f in tops.items() if n not in accounted}
print(f"{len(tops)} top-level frames, {len(missing)} unaccounted:\n")
for name, f in sorted(missing.items(), key=lambda kv: kv[1]):
    print(f"  {name:<34} {f}")
