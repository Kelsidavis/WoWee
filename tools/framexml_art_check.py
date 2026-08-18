#!/usr/bin/env python3
"""Art the interface asks for that is not in this install.

    tools/framexml_art_check.py

A missing texture is not an error anywhere. The frame is built, laid out and
drawn, and the part of it that should have art is simply not there - a button
with no face, a panel with no border, an icon slot that reads as empty rather
than as broken. The log carries one "Widget texture not found" line among
thousands, once, at whatever moment the frame first appeared.

WHAT IT READS

Both halves. `file="..."` on every texture element in the XML, and the string
argument of SetTexture/SetNormalTexture/SetBackdrop and friends in the Lua. The
Lua half is the larger one - the quest icons that change per row, the class and
race art, anything picked at runtime - and it is invisible to a check that reads
only the XML.

THE ALIAS TABLE

The interface is WotLK's; the assets are whichever expansion this install
carries, and the two do not always agree on a folder. FrameXML asks for
Interface\\GossipFrame\\, this install has Interface\\Gossip\\, and
WidgetRenderer::texture retries through a small table of folder swaps. That
table is read out of the source here rather than copied, so the two cannot
disagree - a copy would report art as missing the moment somebody added a swap,
or worse, stay quiet after one was removed.

WHAT IT CANNOT SEE

A path built by concatenation - "Interface\\Icons\\"..icon - which is most icon
art and all of it correct by construction. Nor whether a file that exists is the
right one.

Unreachable frames still count: the login splash and the tic-tac-toe minigame
are both in the list below and neither can appear.

THE ONE IT REPORTS TODAY, AND WHY IT IS NOT A GAP

interface/talentframe/ui-talentframe-spectab, from blizzard_talentui.lua:852.
That line sits behind `SELECTEDSPEC_DISPLAYTYPE == "PUSHED_OUT"`, and the
constant is set to "GOLD_INSIDE" eight hundred lines above it - so the texture
is never asked for and its absence is not a blank tab.

Eighteen others went when this started reading only the files the loader opens.
They came from focusframe, questtimerframe, minigameframe, tictactoeframe and
petpopup: in no manifest, included by nothing, and so never a request for art
at all.
"""
import pathlib
import re
import sys
import pathlib as _pathlib
sys.path.insert(0, str(_pathlib.Path(__file__).resolve().parent))
from framexml_source import without_comments, loaded_files

ROOT = pathlib.Path(__file__).resolve().parent.parent
# One literal, so sweep_guard can see this sweep needs the interface art and
# skip it where there is none. A path in separate components is invisible
# to that check, and the sweep then runs on CI, finds nothing, and is
# failed for reporting nothing.
DATA = ROOT / "Data"
_NEEDS = ROOT / "Data/interface"   # what this sweep cannot run without
INTERFACE = DATA / "interface"
RENDERER = ROOT / "src/ui/widget_renderer.cpp"

ART_ELEMENTS = ("Texture", "NormalTexture", "PushedTexture", "HighlightTexture",
                "DisabledTexture", "CheckedTexture", "DisabledCheckedTexture",
                "BarTexture")
SETTERS = ("SetTexture", "SetNormalTexture", "SetPushedTexture",
           "SetHighlightTexture", "SetDisabledTexture", "SetCheckedTexture",
           "SetStatusBarTexture", "SetBackdropTexture")


def folder_swaps():
    """The runtime retries, read out of the renderer rather than copied."""
    src = RENDERER.read_text(errors="ignore")
    return [(a.lower().replace("\\\\", "/"), b.lower().replace("\\\\", "/"))
            for a, b in re.findall(r'\{\s*(?:\.\w+\s*=\s*)?"([^"]+)"\s*,\s*(?:\.\w+\s*=\s*)?"([^"]+)"\s*\}', src)
            if "\\\\" in a and "\\\\" in b]


def on_disk():
    """Every art file, keyed without its extension - the client accepts both."""
    out = set()
    for path in INTERFACE.rglob("*"):
        if not path.is_file():
            continue
        rel = path.relative_to(DATA).as_posix().lower()
        out.add(rel.rsplit(".", 1)[0] if "." in path.name else rel)
    return out


def normalise(raw):
    # Lua source spells a separator "\\", so the text carries two characters
    # and a naive swap makes "interface//buttons//". Collapse the runs.
    q = re.sub(r"[\\/]+", "/", raw).strip().lower().lstrip("/")
    for ext in (".tga", ".blp", ".png"):
        if q.endswith(ext):
            q = q[:-len(ext)]
    return q


def references():
    """path -> the files that ask for it."""
    refs = {}
    xml_pat = re.compile(
        r"<(?:" + "|".join(ART_ELEMENTS) + r")\b[^>]*?file=\"([^\"]+)\"",
        re.I | re.S)
    # The closing quote is captured too, so a literal that is only the head of
    # a concatenation - "Interface\\PVPFrame\\PVP-Banner-"..index - can be told
    # apart from a whole path. Those are correct by construction and there is no
    # way to know from here what they end up as.
    lua_pat = re.compile(
        r"(?:" + "|".join(SETTERS) + r")\s*\(\s*\"([^\"]+)\"(\s*\.\.)?")
    # Only files the loader opens: art referenced by a file that never runs is
    # never asked for, so a missing texture there is not a missing texture.
    for path in sorted(q for q in loaded_files(INTERFACE) if q.suffix.lower() == ".xml"):
        text = re.sub(r"<!--.*?-->", "", path.read_text(errors="ignore"), flags=re.S)
        for m in xml_pat.finditer(text):
            refs.setdefault(normalise(m.group(1)), set()).add(path.name)
    for path in sorted(q for q in loaded_files(INTERFACE) if q.suffix.lower() == ".lua"):
        text = without_comments(path.read_text(errors="ignore"))
        for m in lua_pat.finditer(text):
            # A colour or a blend mode rather than a path.
            if "/" not in m.group(1) and "\\" not in m.group(1):
                continue
            if m.group(2):
                continue          # the head of a concatenation
            refs.setdefault(normalise(m.group(1)), set()).add(path.name)
    return refs


def main():
    have = on_disk()
    swaps = folder_swaps()
    refs = references()

    missing = {}
    for ref, where in refs.items():
        if ref in have:
            continue
        if any(ref.replace(a, b) in have for a, b in swaps if a in ref):
            continue
        missing[ref] = where

    print(f"{len(refs)} art paths referenced, {len(swaps)} folder swap(s) "
          f"allowed for\n")
    print(f"{len(missing)} not in this install:\n")
    for ref in sorted(missing):
        print(f"  {ref:58} {', '.join(sorted(missing[ref])[:2])}")
    if not missing:
        print("  (none)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
