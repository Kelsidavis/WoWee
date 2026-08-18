#!/usr/bin/env python3
"""Client slash commands that FrameXML shadows with a handler that does nothing.

This client's chat tries SlashCmdList before its own registry and returns as
soon as it finds a handler - and dispatchSlashCommand returns true even when
that handler *errors*. So any command FrameXML defines wins, working or not.

/follow was found this way: SLASH_FOLLOW1..7 cover /f, /follow and /fol, all
landing on a no-op FollowUnit while the client's own /follow sat unreachable
behind it.

Reports only where both sides claim the same command AND FrameXML's handler
bottoms out in a stub or a missing global - that is the pairing that loses a
working feature.

Split into two lists, because a dead call is worth very different amounts
depending on what is beside it. A handler with NO live call cannot do anything
at all. A handler with a dead call beside a live one usually has the dead one on
a branch that never runs: /ignore reads BNet_GetPresenceID, which resolves to
GetAutoCompletePresenceID answering nil here and falls through to AddOrDelIgnore,
and /leave gates BNLeaveConversation behind a channel number above ten and
otherwise calls LeaveChannelByName. Both commands work.

Filtering the second list away was tried and is wrong: an IGNORE handler whose
only real call was replaced by a name nothing binds still had UnitIsPlayer and
SendSystemMessage in it, so "has a live call" stayed true and the fault
vanished from the report. Listing both is the honest arrangement.

This sweep sees which names a handler mentions, never which branch runs - so
read the whole handler before believing a hit, and before dismissing one.
"""
import re
from pathlib import Path

ROOT = Path("/home/k/Desktop/wowee")
import sys as _s; _s.path.insert(0, str(Path(__file__).resolve().parent))
from framexml_source import loaded_files

XML = ROOT / "Data/interface"

STUBS = {"lua_ReturnNil", "lua_ReturnZero", "lua_ReturnFalse", "lua_ReturnNothing",
         "lua_ContainerNoOp", "lua_ContainerFalse"}

# Bound at all - the loose pattern, because a lambda body full of braces is
# still a binding and matching only trivial ones made every real
# implementation read as missing. InspectUnit was reported dead that way.
# One source of truth for what is answered - see framexml_provides. Working
# this out per tool is how six sweeps came to disagree about it.
import sys as _sys
_sys.path.insert(0, str(Path(__file__).resolve().parent))
from framexml_provides import globals_provided

bound, noop = globals_provided(), set()
for f in (ROOT / "src/addons").glob("*.cpp"):
    s = f.read_text(errors="ignore")
    # Does nothing: a named stub, or a lambda whose whole body discards L.
    for m in re.finditer(r'\{(?:\.\w+\s*=\s*)?"([A-Za-z0-9_]+)",\s*(?:\.\w+\s*=\s*)?(?:&)?\s*(lua_[A-Za-z0-9_]+)\}', s):
        if m.group(2) in STUBS:
            noop.add(m.group(1))
    for m in re.finditer(r'\{(?:\.\w+\s*=\s*)?"([A-Za-z0-9_]+)",\s*(?:\.\w+\s*=\s*)?\[\]\(lua_State\*\s*L\)\s*->\s*int\s*\{\s*\(void\)L;\s*return 0;\s*\}\}', s):
        noop.add(m.group(1))

# Defined in FrameXML itself - ChatFrame_DisplayUsageError and ShowUIPanel are
# Lua, not bindings, and are not this client's business.
defined = set()
for path in sorted(loaded_files(XML)):
    t = path.read_text(errors="ignore")
    defined |= set(re.findall(r"\bfunction\s+([A-Za-z_][A-Za-z0-9_]*)\s*\(", t))
    defined |= set(re.findall(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*", t, re.M))

# SLASH_<NAME><n> = "/cmd"
slash = {}
for path in XML.rglob("*.lua"):
    for m in re.finditer(r'SLASH_([A-Z0-9_]+?)(\d)\s*=\s*"(/[^"]+)"', path.read_text(errors="ignore")):
        slash.setdefault(m.group(1), set()).add(m.group(3).lower())

# SlashCmdList["NAME"] = function(...) ... end  -> globals it calls
handlers = {}
for path in XML.rglob("*.lua"):
    t = path.read_text(errors="ignore")
    for m in re.finditer(r'SlashCmdList\["([A-Z0-9_]+)"\]\s*=\s*function\b(.{0,700}?)\nend', t, re.S):
        handlers.setdefault(m.group(1), "")
        handlers[m.group(1)] += m.group(2)

# The client's own registry: aliases() { return {"follow", "f"}; }
client = {}
for f in (ROOT / "src/ui/chat/commands").glob("*.cpp"):
    s = f.read_text(errors="ignore")
    for m in re.finditer(r"aliases\(\)[^{]*\{\s*return\s*\{([^}]*)\}", s):
        for a in re.findall(r'"([^"]+)"', m.group(1)):
            client["/" + a.lower()] = f.name

print(f"{len(slash)} SLASH names, {len(handlers)} handlers, "
      f"{len(client)} client commands\n")

rows = []
for name, cmds in sorted(slash.items()):
    body = handlers.get(name)
    if not body:
        continue
    overlap = sorted(c for c in cmds if c in client)
    if not overlap:
        continue
    calls = set(re.findall(r"(?<![\w.:])([A-Z][A-Za-z0-9_]*)\s*\(", body))
    dead = sorted(c for c in calls
                  if c in noop or (c not in bound and c not in defined))
    live = sorted(c for c in calls
                  if c not in noop and (c in bound or c in defined))
    # A dead call somewhere in the handler is not a shadow. Both entries this
    # ever reported were a Battle.net branch in front of a working one:
    # /ignore reads BNet_GetPresenceID, which answers nil here and falls
    # through to AddOrDelIgnore, and /leave gates BNLeaveConversation behind
    # a channel number above ten. The question is whether *every* path is
    # dead, so a handler with any live call is not reported.
    #
    # What that gives up: a handler whose live call is on the branch that
    # never runs. Rarer, and the note below says to read the whole handler
    # before believing a hit - which is advice worth taking either way.
    if dead:
        rows.append((name, overlap, dead, client[overlap[0]], bool(live)))

hard = [r for r in rows if not r[4]]
soft = [r for r in rows if r[4]]

print(f"{len(hard)} client command(s) whose handler has no live call at all:\n")
for name, cmds, dead, where, _ in hard:
    print(f"  {' '.join(cmds):<26} SlashCmdList[{name}] -> {', '.join(dead)}")
    print(f"      client has it in {where}")
if not hard:
    print("  (none)")

print(f"\n{len(soft)} with a dead call beside a live one - usually a branch "
      f"that cannot run, read each:\n")
for name, cmds, dead, where, _ in soft:
    print(f"  {' '.join(cmds):<26} SlashCmdList[{name}] -> {', '.join(dead)}")
    print(f"      client has it in {where}")
if not soft:
    print("  (none)")
