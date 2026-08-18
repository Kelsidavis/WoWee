#!/usr/bin/env python3
"""What an addon calls that nothing defines.

The point of this is to answer "will this addon work" before loading it, because
the missing-API fallback makes the honest answer hard to see at runtime: an
undefined global is callable and returns nil, so an addon calling one does not
error - it quietly does nothing, which reads as a feature that is present but
broken rather than one that was never wired up.

    tools/addon_api_audit.py                 # every addon, worst last
    tools/addon_api_audit.py blizzard_talentui   # one addon, names listed
    tools/addon_api_audit.py --framexml          # the original interface, per file

The --framexml mode counts only files the client actually loads. Twelve on disk
are not in the manifest and are reached by no Include - focusframe,
minigameframe, petpopup, questtimerframe, tictactoeframe, opacitysliderframe,
bindings.xml - and their gaps were being reported as real work for as long as
this scanned the directory.

**focusframe is not an oversight, and adding it to the manifest would break
the focus frame rather than fix it.** targetframe.xml declares FocusFrame too,
inheriting TargetFrameTemplate, and targetframe.xml is in the manifest - so the
focus frame is built, and loading the stray file as well would declare it
twice. The trap is that "focusframe" is in the branch defaults, which makes it
look as though the file must be missing for a reason nobody noticed.

Counting honestly took two tries, and both mistakes are easy to repeat:

  * `CreateFrame` is registered with lua_setglobal, not through one of the
    luaL_Reg tables. Miss that pattern and the single most-called function in
    the interface reads as undefined, and every count is nonsense.
  * `self:Click()` is a method call, not a global one. Counting those turned a
    real figure of about thirty into four hundred and twenty-two.
  * A `local function Foo()` is still a definition. Blizzard's auction house
    declares two of its helpers that way, and reading only top-level `function`
    reported both as missing from a file that defines them three lines up.

Never fails the build. It measures; it does not judge.
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ADDONS = ROOT / "Data" / "interface" / "addons"
FRAMEXML = ROOT / "Data" / "interface" / "framexml"
SRC = ROOT / "src" / "addons"

# A call on something, rather than a call to a global: the character before the
# name is a colon or a dot. Lua's own `string.format` is caught by this too,
# which is correct - it is not a global this client has to provide.
# A call is a bare global, not obj.Method() and not a:Method() - but the
# lookbehind that says so also rejects the operand of a concatenation,
# because the character before the name in `.."x"..Call()` is a dot. That
# hid every function called that way, and one of them was raising in
# BarberShop_OnLoad while this reported the barber addon clean.
CALL = re.compile(r"(?:(?<![:.\w])|(?<=\.\.))([A-Z][A-Za-z0-9_]{2,})\s*\(")
DEF_FUNC = re.compile(r"^\s*(?:local\s+)?function\s+([A-Za-z_]\w*)", re.M)
# `= function` specifically, and not a bare assignment: matching any `x = ...`
# swallowed every variable in the interface and took the known-set from four
# and a half thousand names to eighteen thousand, hiding real gaps rather than
# fixing false ones.
DEF_ASSIGN = re.compile(r"^\s*(?:local\s+)?([A-Za-z_]\w*)\s*=\s*function", re.M)
# A local alias of something already defined: `local Foo_orig = Foo`.
DEF_ALIAS = re.compile(r"^\s*local\s+([A-Za-z_]\w*)\s*=\s*[A-Za-z_]\w*\s*;?\s*$", re.M)
# The same thing without `local`, which defines a *global* and is how the
# achievement frame swaps its tab handler:
#
#     AchievementFrameTab_OnClick = AchievementFrameBaseTab_OnClick;
#
# Reported as two missing names in an addon that defines both of them. The
# right-hand side is captured as well as the left, because counting this as a
# definition is only safe when what it aliases is itself a function - matching
# any `x = y` is what the note above DEF_ASSIGN warns against.
DEF_GLOBAL_ALIAS = re.compile(
    r"^\s*([A-Za-z_]\w*)\s*=\s*([A-Za-z_]\w*)\s*;?\s*$", re.M)
XML_NAME = re.compile(r'name="([^"$]+)"')


# Lua comments, which are not code. The guild bank's XML carries a call to
# GuildBankItemButton_OnUpdate with two dashes in front of it, and reading that
# as a call made a function nobody invokes look like a missing one.
BLOCK_COMMENT = re.compile(r"--\[(=*)\[.*?\]\1\]", re.S)
LINE_COMMENT = re.compile(r"--[^\n]*")


def read(path):
    try:
        return path.read_text(errors="ignore")
    except OSError:
        return ""


def without_comments(text):
    return LINE_COMMENT.sub("", BLOCK_COMMENT.sub("", text))


# Text inside a string is not code. globalstrings.lua describes the Horde as
# "opposed to members of the Alliance (Night Elves, ...)", and reading that as a
# call to Alliance() put a function nobody wrote on every missing list - along
# with Horde, Epic, Strength and a dozen more that are only ever prose.
STRINGS = re.compile(r'"(?:\\.|[^"\\\n])*"' r"|'(?:\\.|[^'\\\n])*'" r"|\[\[.*?\]\]", re.S)


def without_strings(text):
    return STRINGS.sub('""', text)


def calls_in(text):
    """Every global-looking call, ignoring anything quoted."""
    return set(CALL.findall(without_strings(text)))


def known_names():
    """Everything a Lua chunk could reasonably find already defined."""
    names = set()
    for cpp in SRC.glob("*.cpp"):
        s = read(cpp)
        names |= set(re.findall(r'\{(?:\.\w+\s*=\s*)?"(\w+)"\s*,', s))          # luaL_Reg tables
        names |= set(re.findall(r'set\("(\w+)"', s))           # region methods
        names |= set(re.findall(r'lua_setglobal\(\s*\w+\s*,\s*(?:\.\w+\s*=\s*)?"(\w+)"', s))
        names |= set(re.findall(r'"function (\w+)\(', s))      # bootstrap Lua
        names |= set(re.findall(r'"(\w+)\s*=\s*function', s))
        # The counting stubs, which are defined by looping over a Lua list of
        # names rather than one at a time. Missing them made GetNumBankSlots,
        # GetInventoryItemCount and thirty others read as undefined when they
        # answer zero perfectly well.
        for block in re.findall(r"local counting = \{(.*?)\}", s, re.S):
            names |= set(re.findall(r"'(\w+)'", block))
    for lua in FRAMEXML.glob("*.lua"):
        s = read(lua)
        names |= set(DEF_FUNC.findall(s)) | set(DEF_ASSIGN.findall(s))
    # Addons define globals for each other: the talent frame calls
    # GlyphFrame_Update and loads Blizzard_GlyphUI to get it.
    for lua in ADDONS.glob("*/*.lua"):
        s = read(lua)
        names |= set(DEF_FUNC.findall(s)) | set(DEF_ASSIGN.findall(s))
    return names


def aliasDefs(text, defined, known=()):
    """Globals defined by aliasing a function that is already defined.

    Repeated to a fixed point so a chain of aliases resolves, and only ever
    admitting a name whose right-hand side is known to be a function - a bare
    `x = y` on its own says nothing about what x is.
    """
    pairs = DEF_GLOBAL_ALIAS.findall(text)
    out = set()
    changed = True
    while changed:
        changed = False
        for name, target in pairs:
            if name in out:
                continue
            if target in defined or target in known or target in out:
                out.add(name)
                changed = True
    return out


def audit(addon_dir, known):
    body, defined = "", set()
    for f in sorted(addon_dir.glob("*.lua")) + sorted(addon_dir.glob("*.xml")):
        s = read(f)
        body += without_comments(s)
        defined |= set(DEF_FUNC.findall(s)) | set(DEF_ASSIGN.findall(s))
        defined |= set(DEF_ALIAS.findall(s))
        defined |= set(XML_NAME.findall(s))   # a frame's name is a global too
    defined |= aliasDefs(body, defined, known)
    return sorted(c for c in calls_in(body)
                  if c not in known and c not in defined)


def reachableFrameXml():
    """The interface files the client will actually load.

    The manifest names the entry points; each of those pulls in more through
    Include and Script. Anything left over is on disk and unreachable, and
    counting it reports work that does not exist - FocusFrame, for one, is
    declared in targetframe.xml, and focusframe.xml is never read.
    """
    toc = FRAMEXML / "framexml.toc"
    if not toc.is_file():
        toc = FRAMEXML / "FrameXML.toc"
    if not toc.is_file():
        return sorted(FRAMEXML.glob("*.lua"))

    reached = set()

    def walk(name):
        key = name.lower().replace("\\", "/").split("/")[-1]
        if key in reached:
            return
        reached.add(key)
        path = FRAMEXML / key
        if not path.is_file() or path.suffix.lower() != ".xml":
            return
        text = read(path)
        for ref in re.findall(r'<(?:Include|Script)\s+file="([^"]+)"', text):
            walk(ref)

    for line in read(toc).splitlines():
        line = line.strip()
        if line and not line.startswith("#"):
            walk(line)
    # Both kinds: the XML carries inline OnLoad and OnEvent bodies, which call
    # globals exactly like the Lua does.
    return sorted(p for p in FRAMEXML.iterdir()
                  if p.suffix.lower() in (".lua", ".xml") and p.name.lower() in reached)


def auditFrameXml(known):
    files = reachableFrameXml()
    rows = []
    for path in files:
        body = without_comments(read(path))
        defined = set(DEF_FUNC.findall(body)) | set(DEF_ASSIGN.findall(body))
        defined |= set(XML_NAME.findall(body))       # a frame's name is a global
        defined |= aliasDefs(body, defined, known)
        missing = sorted(c for c in calls_in(body)
                         if c not in known and c not in defined)
        rows.append((len(missing), path.name, missing))
    rows.sort()
    clean = sum(1 for n, _, _ in rows if n == 0)
    print(f"{len(files)} interface files the client loads; {clean} need nothing.\n")
    for count, name, missing in rows:
        if count == 0:
            continue
        print(f"{count:4}  {name}")
        if count <= 3:
            for m in missing:
                print(f"        {m}")
    total = len({m for _, _, ms in rows for m in ms})
    print(f"\n{len(known)} names known; {total} distinct missing across the "
          f"files that load")
    return 0


def main():
    if "--framexml" in sys.argv:
        return auditFrameXml(known_names())
    if not ADDONS.is_dir():
        print(f"no addons at {ADDONS}")
        return 0
    known = known_names()
    wanted = sys.argv[1] if len(sys.argv) > 1 else None

    rows = []
    for d in sorted(p for p in ADDONS.iterdir() if p.is_dir()):
        if wanted and d.name != wanted:
            continue
        rows.append((len(missing := audit(d, known)), d.name, missing))
    if not rows:
        print(f"no addon named {wanted}")
        return 0

    rows.sort()
    for count, name, missing in rows:
        if wanted or count == 0:
            print(f"{count:4}  {name}")
            for m in missing:
                print(f"        {m}")
        else:
            print(f"{count:4}  {name}")
    total = len({m for _, _, ms in rows for m in ms})
    print(f"\n{len(known)} names known; {total} distinct still missing "
          f"across {len(rows)} addon(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
