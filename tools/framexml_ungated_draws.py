#!/usr/bin/env python3
"""Draw surfaces this client renders that no handover can switch off.

Every duplicate found so far had the same shape: FrameXML draws a thing, this
client draws the same thing, and nothing connects the two. `frameXmlOwns`
gating is how a client surface steps aside -- so a surface reached without
passing a gate CANNOT step aside, whatever element set is named at runtime.

Zone text was exactly that and was found by eye. `framexml_unaccounted_frames`
could not see it: that tool walks FrameXML's frames and asks which element
mentions them, so it only finds gaps among names someone already listed. This
one walks the other side -- this client's own ImGui draw calls -- and asks
which are unreachable-from-a-gate. It needs no element list to work.

Reports, per ungated draw function, the call path that reaches it. Being
listed is not a bug: plenty of surfaces have no FrameXML counterpart (the
login screen, the settings panel, the GM console). The question to ask of
each row is "does FrameXML draw this too?" -- and for those, the answer is
either a gate or a suppression entry.

Known false-positive class, deliberately not filtered: a surface can be gated
at its *data source* rather than at its draw. `renderUIErrors` is reached
unconditionally and draws nothing, because the callback that fills its list
returns early when FrameXML owns the element. That is a real gate one layer
up, and no reachability walk over draw calls can see it. Check where a listed
surface gets its contents before believing the row.

THE TWENTY-EIGHT IT REPORTS TODAY, ALL CHECKED

Two faults were found in this list on 2026-08-05, both live since the branch
started, and both had been skimmed past twice because they sat among rows that
are fine. A report nobody can triage at a glance gets skimmed - so the triage
lives in ACCOUNTED below, one line of reasoning each, and the question this
tool now answers is whether a row is NEW.

The question that separates a fault from a row that is fine is not "does
FrameXML draw this too" but "does FrameXML draw this too *without being
asked*". A duplicate only happens when both sides put something on screen in
response to the same event. A window with its own toggle has a FrameXML
counterpart and cannot collide with it: two ways to open a thing is not two
things on screen.

  * Fixed: renderBgInvitePopup - CONFIRM_BATTLEFIELD_ENTRY, raised by
    battlefieldframe.lua from UPDATE_BATTLEFIELD_STATUS, which this client
    fires. Two accept buttons on every battleground invitation.
  * Fixed: renderLogoutCountdown - the CAMP and QUIT popups, raised by
    uiparent.lua from PLAYER_CAMPING and PLAYER_QUITING, both fired here.

WHAT THE FIRST VERSION OF THAT LIST GOT WRONG

It was a map from surface to the element it would collide with, and four of its
seven entries were claims nobody had checked. WatchFrame raises no toast when a
quest completes; AlertFrame_OnEvent in 3.3.5 handles achievements and dungeon
rewards and no loot at all; RaidInfoFrame is opened from a button. A fifth,
written while correcting the other four, said 3.3.5 had no titles UI -
PlayerTitleFrame is on the paperdoll.

Writing an unverified judgement into a tool is the thing a tool is for
catching, and the correction is why this is an allowlist rather than a map: it
holds only what was read, and says nothing about what it has not read.
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
UI = ROOT / "src/ui"

# Draw calls that put pixels on screen without going through a child widget.
DRAW = re.compile(r"ImGui::Begin\s*\(|GetForegroundDrawList|GetBackgroundDrawList")

# Entry points the application calls directly. These MUST be qualified: every
# panel class has a bare `render`, so an unqualified root list makes each panel
# its own entry point and walks straight past the gate at its call site. That
# is how the first run of this tool reported the bag windows as ungated -- they
# are gated, in game_screen.cpp, on the caller's side.
ROOTS = {"GameScreen::render", "UiManager::render"}

# Surfaces with no FrameXML counterpart -- they exist only in this client, so
# there is nothing for them to duplicate. Kept explicit so the list is audited
# rather than silently filtered by a name pattern.
NO_COUNTERPART = {
    "auth_screen.cpp", "realm_screen.cpp", "character_create_screen.cpp",
    "settings_panel.cpp", "gm_command_screen.cpp", "widget_renderer.cpp",
}


def strip_comments(text):
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


def functions(text, path):
    """Every Class::method body in a translation unit, by brace matching."""
    out = {}
    for m in re.finditer(r"^[\w:<>,&*\s]*?(\w+)::(\w+)\s*\([^;{]*?\)\s*(?:const\s*)?\{(?:\.\w+\s*=\s*)?",
                         text, re.M):
        name, start = f"{m.group(1)}::{m.group(2)}", m.end() - 1
        depth, i = 0, start
        while i < len(text):
            if text[i] == "{":
                depth += 1
            elif text[i] == "}":
                depth -= 1
                if depth == 0:
                    break
            i += 1
        out.setdefault(name, []).append((path.name, text[start:i]))
    return out


bodies = {}
for p in sorted(UI.rglob("*.cpp")):
    text = strip_comments(p.read_text(errors="ignore"))
    for name, entries in functions(text, p).items():
        bodies.setdefault(name, []).extend(entries)

# A gate is not always an element. A load-on-demand addon arrives mid-run
# because the player asked for the feature, so what it draws cannot be chosen
# before the run starts and no UiElement can describe it - frameXmlDrawsCombatText
# is that shape, and reading only frameXmlOwns calls the surface behind it
# ungated when it stands down correctly.
GATES = ("frameXmlOwns", "frameXmlDraws")

# A call is gated when a gate appears in the same `if` -- either guarding
# the statement on that line, or opening the block the call sits in. Track the
# brace depth at which each gate opened so the block's extent is known.
# Exclude only namespace-qualified names (`ImGui::Begin`), by refusing a `:`
# before the callee. Excluding `.` and `>` as well -- the obvious way to skip
# qualified calls -- drops every `ptr->method()` and `obj.method()`, which is
# how almost all cross-object calls are written here. With those excluded the
# walk never left the class it started in and reached 27 of 404 functions.
#
# The receiver is captured so the callee can be resolved to ONE class. Matching
# on the bare method name instead makes every `render()` an edge to every
# class's `render`, which routes the walk around the very gates it is looking
# for -- InventoryScreen::render appeared reachable via ChatPanel.
CALL = re.compile(r"(?<![\w:])(?:(\w+)\s*(?:->|\.))?(\w+)\s*\(")


def receiver_types(classes):
    """variable name -> class, from declarations anywhere in the UI sources.

    Covers the shapes this code uses: `std::unique_ptr<InventoryScreen>
    inventoryScreen_;`, `CombatUI* combatUi_;`, `ToastManager& toasts`.
    """
    out = {}
    pattern = re.compile(
        r"\b(?:std::(?:unique_ptr|shared_ptr)\s*<\s*)?(" +
        "|".join(sorted(classes, key=len, reverse=True)) +
        r")\s*>?\s*[*&]?\s*(\w+)\s*[;=,){]")
    for p in list(UI.rglob("*.cpp")) + list((ROOT / "include/ui").rglob("*.hpp")):
        for m in pattern.finditer(strip_comments(p.read_text(errors="ignore"))):
            out[m.group(2)] = m.group(1)
    return out


def edges(body):
    """(receiver, callee, gated) for every call in a body."""
    out = []
    gate_depths = []
    depth = 0
    # A braceless `if (!frameXmlOwns(X))` puts its body on the NEXT line, so a
    # line-local check calls that body ungated. Carry the gate forward one
    # statement. This is how renderQuestObjectiveTracker was first misreported.
    pending = False
    for line in body.split("\n"):
        gate_here = any(g in line for g in GATES)
        for m in CALL.finditer(line):
            receiver, callee = m.group(1), m.group(2)
            if callee in ("if", "for", "while", "switch", "return", "sizeof"):
                continue
            out.append((receiver, callee,
                        bool(gate_depths) or gate_here or pending))
        opened = line.count("{")
        closed = line.count("}")
        if gate_here and opened > closed:
            gate_depths.append(depth)
            pending = False
        elif gate_here:
            # ...unless the guarded statement is on this line already. A
            # one-line `if (!gate()) draw();` needs no carry, and carrying it
            # anyway gates whatever is written next: adding one such line to
            # game_screen.cpp took the DPS meter out of this report, and the
            # DPS meter has no gate at all.
            pending = not any(
                m.group(2) not in ("if", "for", "while", "switch", "return", "sizeof")
                for m in CALL.finditer(line.split(")", 1)[-1]))
        elif line.strip():
            pending = False
        depth += opened - closed
        while gate_depths and depth <= gate_depths[-1]:
            gate_depths.pop()
    return out


by_short = {}
for qualified in bodies:
    by_short.setdefault(qualified.split("::")[1], []).append(qualified)

CLASSES = {q.split("::")[0] for q in bodies}
VARS = receiver_types(CLASSES)


def resolve(caller, receiver, callee):
    """Call site -> the qualified definitions it can reach.

    Ordered most specific first: a typed receiver names exactly one class; an
    unqualified call is the caller's own class; a receiver of unknown type
    falls back to the class defining that name - but only if exactly one does.

    Only one, because linking a call to every class that happens to define the
    name manufactures paths rather than finding them. `wm->render(...)` on a
    world map facade, which is not a UI class and so has no declared type here,
    was resolved to ChatPanel::render and CharacterScreen::render among others
    - and both are gated at their real call site, so both were reported as
    draw surfaces no handover could switch off. An edge that could go five
    ways was never evidence of anything.
    """
    if receiver and receiver in VARS:
        target = f"{VARS[receiver]}::{callee}"
        return [target] if target in bodies else []
    if not receiver:
        own = f"{caller.split('::')[0]}::{callee}"
        if own in bodies:
            return [own]
    candidates = by_short.get(callee, [])
    return candidates if len(candidates) == 1 else []


draws = {n for n, es in bodies.items()
         if any(DRAW.search(b) for _, b in es)}

# A surface can also gate itself, with an early return in its own body rather
# than a check at the call site. Both forms mean the same thing at runtime, so
# both have to count -- reading only call sites reports these as ungated.
self_gated = {n for n, es in bodies.items()
              if any(any(g in b for g in GATES) for _, b in es)}

# Walk from the roots, refusing to cross a gated edge. Anything drawing that is
# still reachable cannot be switched off.
reached, queue = {}, []
for r in ROOTS & set(bodies):
    reached[r] = [r]
    queue.append(r)
while queue:
    cur = queue.pop(0)
    for _, body in bodies.get(cur, []):
        for receiver, callee, gated in edges(body):
            if gated:
                continue
            for target in resolve(cur, receiver, callee):
                if target in reached:
                    continue
                reached[target] = reached[cur] + [target]
                queue.append(target)

rows = []
for name in sorted((draws & set(reached)) - self_gated):
    files = {f for f, _ in bodies[name]}
    if files & NO_COUNTERPART:
        continue
    rows.append((name, sorted(files), reached[name]))

print(f"{len(bodies)} functions in src/ui, {len(draws)} draw directly, "
      f"{len(reached)} reachable without crossing a gate\n")
print(f"{len(rows)} draw surfaces no handover can switch off:\n")
for name, files, path in rows:
    print(f"  {name}   [{', '.join(files)}]")
    print(f"      {' -> '.join(path)}")

# ---------------------------------------------------------------------------
# The half-wired element, checked directly rather than through the walk.
#
# A handover is two halves: a suppression entry hides FrameXML's frame while
# this client owns the element, and a frameXmlOwns gate stands this client's
# own surface down when FrameXML owns it. Writing only the first is invisible
# in every element set except the one that hands that element over -- which is
# how Trade, ReadyCheck, RaidWarning and the achievement badge all shipped
# drawing twice. Four of the fifty, none noticed by eye.
# ---------------------------------------------------------------------------
takeover = (ROOT / "src/ui/framexml_takeover.cpp").read_text(errors="ignore")
suppressing = set(re.findall(r"\{UiElement::(\w+),\s*\n?\s*\"",
                             takeover[takeover.index("kSuppress"):]))
standing_down = set()
for p in (ROOT / "src").rglob("*.cpp"):
    if p.name == "framexml_takeover.cpp":
        continue
    standing_down |= set(re.findall(r"frameXmlOwns\(UiElement::(\w+)\)",
                                    p.read_text(errors="ignore")))

half_wired = sorted(suppressing - standing_down)
print(f"\n{len(suppressing)} elements suppress a FrameXML frame, "
      f"{len(standing_down)} are gated in client code")
if half_wired:
    print(f"\n{len(half_wired)} hide FrameXML's frame but never stand down themselves:")
    for name in half_wired:
        print(f"  UiElement::{name}")
else:
    print("\nEvery suppressed element also stands down. Both halves wired.")

# ── Which of them have been looked at ───────────────────────────────────────
#
# An allowlist, not a counterpart map. The first cut of this was a map from
# surface to the FrameXML element it would collide with, and four of its seven
# entries were wrong: WatchFrame raises no toast on a quest completing,
# AlertFrame in 3.3.5 shows achievements and dungeon rewards and no loot at
# all, and RaidInfoFrame is a window opened from a button. Writing an
# unverified judgement into a tool is the thing the tool is for catching.
#
# So it holds only what was checked, and the useful question becomes the other
# one: is this row NEW. Twenty-eight surfaces draw without a gate and every one
# has been read against what FrameXML raises; a twenty-ninth has not, and that
# is worth stopping for whether or not it turns out to be fine.
#
# The reason beside each is what was checked, so a row can be re-examined
# without starting over.
ACCOUNTED = {
    # Nothing on the other side at all.
    "ChatBubbleManager::render":                  "no FrameXML bubbles; engine-drawn in 3.3.5",
    "CombatUI::renderCooldownTracker":            "no counterpart",
    "CombatUI::renderDPSMeter":                   "no counterpart",
    "CombatUI::renderThreatWindow":               "no counterpart; FrameXML's threat is on the unit frame",
    "DialogManager::renderDuelCountdown":         "no counterpart; FrameXML has the request popup only",
    "GameScreen::renderEntityList":               "debug window",
    "GameScreen::renderNameplates":               "no FrameXML nameplates in 3.3.5",
    "GameScreen::renderPlayerInfo":               "debug window",
    "GameScreen::renderWeatherOverlay":           "debug overlay",
    "InventoryScreen::renderItemTargetCursor":    "cursor art",
    "ToastManager::renderAreaTriggerToasts":      "no counterpart",
    "ToastManager::renderDingEffect":             "no counterpart",
    "ToastManager::renderDiscoveryToast":         "no counterpart",
    "ToastManager::renderPvpHonorToasts":         "no counterpart",
    "ToastManager::renderRepToasts":              "no counterpart",
    "ToastManager::renderResurrectFlash":         "no counterpart",
    "ToastManager::renderWhisperToasts":          "no counterpart",
    "ToastManager::renderPlayerLevelUpToasts":    "LevelUpDisplay is Cataclysm; nothing in 3.3.5",
    "ToastManager::renderItemLootToasts":         "AlertFrame_OnEvent handles achievements and LFG rewards only",
    "ToastManager::renderQuestCompleteToasts":    "WatchFrame redraws; it raises no toast",
    "ToastManager::renderQuestProgressToasts":    "WatchFrame redraws; progress never goes through addUIError",
    # Opened deliberately. Two ways to open a thing is not two things on screen.
    "WindowManager::renderEquipSetWindow":        "a window with its own toggle",
    "WindowManager::renderSkillsWindow":          "a window with its own toggle",
    "WindowManager::renderInstanceLockouts":      "a window with its own toggle; RaidInfoFrame is too",
    "WindowManager::renderTitlesWindow":          "a window with its own toggle; FrameXML's is the paperdoll's title dropdown",
    "CombatUI::renderCombatLog":                  "a window with its own toggle; FrameXML's is ChatFrame2",
    # Gated somewhere a reachability walk cannot see.
    "ToastManager::renderZoneToasts":             "suppressed by the zone banner's own timer",
    "DialogManager::renderPetUnlearnConfirmDialog": "staticpopup.lua declares no CONFIRM_PET_UNLEARN",
}

unreviewed = sorted(r[0] for r in rows if r[0] not in ACCOUNTED)
print(f"\n{len(ACCOUNTED)} of them have been read against what FrameXML raises")
if unreviewed:
    print(f"\n{len(unreviewed)} that have not - ask of each whether FrameXML puts "
          f"the same thing on screen in response to the same event:")
    for fn in unreviewed:
        print(f"  {fn}")
else:
    print("None unreviewed.")

sys.exit(0)
