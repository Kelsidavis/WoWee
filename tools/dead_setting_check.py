#!/usr/bin/env python3
"""Options panel controls whose CVar nothing ever reads.

Every checkbox, slider and dropdown in the options panels names a CVar in
self.cvar. The control will save that CVar and read it back happily whether or
not anything acts on it, so a setting with no reader looks exactly like a
setting that works: it remembers what you chose and changes nothing.

A CVar counts as read if any of these mention it, outside the panel definition
that declares it:

  * FrameXML asking for it directly - GetCVar/GetCVarBool/SetCVar
  * a uvarInfo entry mapping it to a global, where that global is read
  * the client itself - storedCVarValue, or the name as a string literal

Names are matched case-insensitively, because the client lowercases them.

A control built in Lua rather than XML has no name this can resolve, so
greying it does not take it off the list. That under-credits by two today (the
two voice device dropdowns) and errs towards reporting a setting as dead, which
is the safe direction for a ratchet.

Run with --canary to check the sweep can still see: it plants a control naming
a CVar nothing reads and fails if that is not reported. A matcher that has gone
blind reads exactly like a clean tree.

What that canary proves is narrow, and it is worth being plain about: it shows
the sweep can still report a name that appears nowhere at all. It cannot show
the reader test is calibrated, because a test that counts too much still
reports a name it never sees. Widening what counts as a reader is checked by
the finding count, not by the canary.
"""

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PANELS = ROOT / "Data/interface/framexml"
LUA_ROOTS = [ROOT / "Data/interface"]
CPP_ROOTS = [ROOT / "src", ROOT / "include"]

# The files that declare controls. A mention inside one of these does not make
# a setting live, and both ways of being clever about that were tried:
#
#   * Counting every mention took this sweep from 29 findings to 3, because
#     those files carry a table keyed by CVar name for tooltip text, so nearly
#     every setting appears in them. The canary still passed - a planted name
#     that appears nowhere cannot detect a reader test that has gone slack.
#   * Counting only the by-name asks - GetCVar("x") - left two settings reading
#     as live whose only reader greys a neighbouring control.
#
# The second is the honest measure of the wrong thing. A panel consulting
# itself to grey a sibling changes the panel, not the game, and this sweep is
# looking for controls that change nothing in the game. cameraSmoothStyle was
# the case that prompted the question and it proves the point: it is asked for
# by name in interfaceoptionspanels.lua, and until it was wired up it still did
# not move the camera by one degree.
DECL_FILES = {"interfaceoptionspanels.xml", "interfaceoptionspanels.lua",
              "videooptionspanels.xml", "videooptionspanels.lua",
              "audiooptionspanels.xml", "audiooptionspanels.lua"}

CVAR_DECL = re.compile(r'self\.cvar\s*=\s*"([A-Za-z0-9_]+)"')
FRAME_DECL = re.compile(r'<Frame\s+name="([A-Za-z0-9_]+)"')
CONTROL_DECL = re.compile(r'<(?:CheckButton|Slider|Button|Frame)\s+name="(\$parent[A-Za-z0-9_]*|[A-Za-z0-9_]+)"')
#: A name in kRemovedControlsLua - one plain string per line.
REMOVED = re.compile(r'^\s*"([A-Za-z0-9_]+)",\s*(?:\.\w+\s*=\s*)?$', re.M)
#: `function SomeFrameName_OnLoad (self)` - the frame is the part before _On.
LUA_HANDLER = re.compile(r'function\s+([A-Za-z0-9_]+?)_On[A-Za-z]+\s*\(')
UVAR_DECL = re.compile(r'self\.uvar\s*=\s*"([A-Za-z0-9_]+)"')
UVAR_ENTRY = re.compile(r'\["([A-Z0-9_]+)"\]\s*=\s*\{[^}]*cvar\s*=\s*"([A-Za-z0-9_]+)"')


def read(p):
    try:
        return p.read_text(errors="ignore")
    except OSError:
        return ""


#: A start or end tag. Attribute values may hold ">" so they are consumed as
#: quoted runs rather than scanned for the closing bracket.
TAG = re.compile(r'<(/?)([A-Za-z][\w.]*)((?:[^>"\']|"[^"]*"|\'[^\']*\')*?)(/?)>', re.S)
NAME_ATTR = re.compile(r'\bname\s*=\s*"([^"]*)"')


def _controls_in_xml(text):
    """(cvar, control name, offset) for every self.cvar in one panel file.

    A depth stack over the tags, so a cvar is attributed to the element it is
    written inside. $parent is that element, which is not the nearest preceding
    named frame: a named sibling declared just above wins that race and gives a
    name no frame answers to. The four voice sliders are the example -
    $parentSpeakerVolume inside AudioOptionsVoicePanel sits after a named
    BindingOutput sibling, so a textual scan reads
    AudioOptionsVoicePanelBindingOutputSpeakerVolume and the frame is
    AudioOptionsVoicePanelSpeakerVolume.

    Hand-rolled rather than xml.etree, which the security scan blocks and which
    this does not need: nothing here parses anything but the repository's own
    markup, and only names and nesting are wanted. Checked against the parser
    it replaced across all 140 panel files, name for name.
    """
    text = re.sub(r"<!--.*?-->", "", text, flags=re.S)
    events = []
    for m in TAG.finditer(text):
        events.append((m.start(), "tag", m))
    for m in CVAR_DECL.finditer(text):
        events.append((m.start(), "cvar", m))
    events.sort(key=lambda e: e[0])

    stack, out = [], []
    for pos, kind, m in events:
        if kind == "cvar":
            owner = next((f for f in reversed(stack) if f), None)
            out.append((m.group(1).lower(), owner, pos))
            continue
        closing, _tag, attrs, selfclose = m.groups()
        if closing:
            if stack:
                stack.pop()
            continue
        nm = NAME_ATTR.search(attrs)
        resolved = None
        if nm:
            raw = nm.group(1)
            parent = next((f for f in reversed(stack) if f), None)
            resolved = (parent + raw[len("$parent"):]) if raw.startswith("$parent") and parent \
                       else (None if raw.startswith("$parent") else raw)
        if not selfclose:
            stack.append(resolved)
    return out


def declared_controls():
    """CVar -> (file:line, control frame name or None)."""
    out = {}
    # The options panels first. A CVar can be declared on a unit frame as well
    # as on the control that sets it - targetStatusText is declared five times
    # across focusframe, targetframe and the panel - and the control the player
    # sees is the one this sweep is about. Document order alone picks whichever
    # file sorts first, which was focusframe.
    files = sorted(PANELS.glob("*.xml"),
                   key=lambda q: (q.name not in DECL_FILES, q.name))
    for q in files:
        text = read(q)
        for cvar, ctrl, pos in _controls_in_xml(text):
            if ctrl is None:
                continue
            out.setdefault(cvar, (f"{q.name}:{text.count(chr(10), 0, pos) + 1}", ctrl))

    # Controls built in Lua rather than XML. Their frame name is in the handler
    # they are declared inside - AudioOptionsSoundPanelHardwareDropDown sets its
    # cvar in AudioOptionsSoundPanelHardwareDropDown_OnLoad - so the enclosing
    # function names the control the same way $parent does in the markup.
    # Without this a device dropdown could be greyed and still read as
    # unhandled, because nothing here knew what it was called.
    for p in sorted(PANELS.glob("*.lua")):
        text = read(p)
        for m in CVAR_DECL.finditer(text):
            line = text.count("\n", 0, m.start()) + 1
            fns = list(LUA_HANDLER.finditer(text[:m.start()]))
            ctrl = fns[-1].group(1) if fns else None
            out.setdefault(m.group(1).lower(), (f"{p.name}:{line}", ctrl))
    return out


def removed_controls():
    """Frame names the client takes off its panels.

    A control on a page the client drops whole counts as removed too - the
    player cannot reach it either way, and listing its thirteen controls
    individually as well would be the same fact written twice.
    """
    text = read(ROOT / "include/addons/addon_lua_snippets.hpp")
    start = text.find("kRemovedControlsLua")
    if start == -1:
        return set(), set()
    end = text.find(")LUA", start)
    body = text[start:end]
    cut = body.find("kRemovedCategories")
    names = set(REMOVED.findall(body[:cut] if cut != -1 else body))
    pages = set(REMOVED.findall(body[cut:])) if cut != -1 else set()
    return names, pages


def uvar_map():
    """cvar (lower) -> uvar global, from uvarInfo entries."""
    text = read(PANELS / "interfaceoptionsframe.lua")
    return {c.lower(): u for u, c in UVAR_ENTRY.findall(text)}


def gather(roots, suffixes):
    for root in roots:
        if not root.exists():
            continue
        for p in root.rglob("*"):
            if p.is_file() and p.suffix in suffixes:
                yield p


def readers(extra_snippets):
    """Lowercased names mentioned anywhere that is not a declaration site."""
    seen = set()
    globals_read = set()
    for p in gather(LUA_ROOTS, {".lua", ".xml"}):
        if p.name in DECL_FILES:
            continue
        text = read(p).lower()
        seen.add((p, text))
        globals_read.add((p, text))
    for p in gather(CPP_ROOTS, {".cpp", ".hpp", ".h"}):
        seen.add((p, read(p).lower()))
    for name, text in extra_snippets:
        seen.add((name, text.lower()))
    return seen


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--canary", action="store_true")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    controls = declared_controls()
    uvars = uvar_map()
    removed, removedPages = removed_controls()

    extra = []
    if args.canary:
        controls["woweecanarysettingnoreader"] = ("canary:0", None)

    corpus = readers(extra)

    dead = []
    handled = []
    for cvar, (where, ctrl) in sorted(controls.items()):
        needle = cvar
        found = False
        for _, text in corpus:
            if needle in text:
                found = True
                break
        if not found and cvar in uvars:
            g = uvars[cvar].lower()
            for _, text in corpus:
                if g in text:
                    found = True
                    break
        if not found:
            if ctrl and (ctrl in removed or any(ctrl.startswith(p) for p in removedPages)):
                handled.append((cvar, where))
            else:
                dead.append((cvar, where))

    total = len(controls)
    print(f"settings with no reader and still on a panel: {len(dead)} of {total} declared "
          f"({len(handled)} more are dead and taken off the panels)")
    for cvar, where in dead:
        print(f"  {cvar:38s} {where}")

    if args.canary:
        if not any(c == "woweecanarysettingnoreader" for c, _ in dead):
            print("CANARY FAILED: planted dead setting was not reported")
            return 1
        print("canary ok")
        return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
