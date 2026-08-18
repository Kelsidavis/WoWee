#!/usr/bin/env python3
"""CVAR_UPDATE labels the interface tests for that this client cannot produce.

    tools/framexml_cvar_label_check.py

CVAR_UPDATE's first argument is the CVar's *label*, not its name. FrameXML uses
both within two lines of each other, which is what settles it:

    if ( (event == "CVAR_UPDATE") and (arg1 == "SHOW_TARGET_CASTBAR") ) then
        if ( GetCVar("showTargetCastbar") == "0") then

Firing the name meant every consumer compared a camelCase name against an
upper-case label and took the other branch - silently, because a string that is
not equal to another string is not an error. The health and mana numbers on
unit frames never appeared or disappeared, the free-bag-slots count never
switched on, and the target and focus cast bars never followed their setting.

Mostly the label is the name in upper snake case. Where it is not, FrameXML
says so itself and lua_system_api carries a table of those pairs. The real
client keeps the mapping in a CVar registry inside the binary, which this
client does not have, so that table is only ever as complete as the last time
someone read the interface.

WHAT IT LOOKS FOR

Every literal a CVAR_UPDATE branch compares its first argument against, and
whether any CVar name FrameXML mentions would produce it under this client's
rule - the named table first, then the mechanical transform. A label nothing
can produce is a branch that can never be taken.

WHAT IS LEFT, AND WHY

One: CHAT_WHOLE_WINDOW_CLICKABLE, in floatingchatframe. No CVar name anywhere
in the interface produces it, and the branch that reads it accepts "chatStyle"
as well - which is the name, not a label, and is what this client sends when
that setting changes. So the branch is reachable, by the other half of its own
condition, and the label has no name here to come from. The ceiling is for the
second, which will be a real gap.

WHAT IT CANNOT SEE

Whether a label is *right*. It checks that the two sides can meet, not that
they meet on the value the real client would have sent. It also only knows the
CVar names FrameXML happens to mention; one set entirely from this client's
own code would not be found, and neither would a label that no comparison in
the interface reads.
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
import sys as _s; _s.path.insert(0, str(Path(__file__).resolve().parent))
from framexml_source import loaded_files

XML = ROOT / "Data/interface"
API = ROOT / "src/addons/lua_system_api.cpp"


def mapping():
    """This client's name-to-label rule, read out of the binding that applies it."""
    src = API.read_text(errors="ignore")
    block = re.search(r"kNamed\[\]\s*=\s*\{(.*?)\};", src, re.S)
    named = dict(re.findall(r'\{(?:\.\w+\s*=\s*)?"(\w+)",\s*(?:\.\w+\s*=\s*)?"(\w+)"\}', block.group(1))) if block else {}

    def label(name):
        if name.lower() in named:
            return named[name.lower()]
        out = []
        for i, c in enumerate(name):
            if i and c.isupper() and not name[i - 1].isupper():
                out.append("_")
            out.append(c.upper())
        return "".join(out)

    return label, named


def main():
    label, named = mapping()

    # Every CVar name the interface mentions, however it mentions it.
    names, wanted = set(), {}
    for path in sorted(sorted(loaded_files(XML))):
        text = path.read_text(errors="ignore")
        names |= set(re.findall(r'(?:Get|Set)CVar\w*\(\s*"(\w+)"', text))
        names |= set(re.findall(r'\.cvar\s*=\s*"(\w+)"', text))
        # A label is compared against arg1 or against a local unpacked from
        # it, and is always upper case with underscores - but so are plenty of
        # things that have nothing to do with CVars. `arg1 == "FRIEND_REQUEST"`
        # is a Battle.net message type, and six of the first run's seven
        # findings were that shape. The comparison only counts inside the
        # branch that tested for CVAR_UPDATE, so the search runs from each
        # mention of the event to the end of the branch it opens.
        for hit in re.finditer(r'CVAR_UPDATE', text):
            tail = text[hit.end():]
            # The branch ends at the next one that tests a different event, or
            # at the end of the enclosing if - whichever comes first.
            stop = re.search(r'\n\s*(?:elseif\s*\(\s*event\b|else\b|end\b)', tail)
            body = tail[:stop.start()] if stop else tail[:400]
            for m in re.finditer(r'(?:arg1|cvar)\s*==\s*"([A-Z][A-Z0-9_]*_[A-Z0-9_]+)"', body):
                wanted.setdefault(m.group(1), set()).add(path.name)

    producible = {label(n) for n in names}
    orphans = {l: f for l, f in wanted.items() if l not in producible}

    print(f"{len(names)} CVar name(s) named in the interface, "
          f"{len(named)} of them with a label this client spells out\n")
    print(f"{len(wanted)} label(s) compared against CVAR_UPDATE's first argument\n")
    print(f"{len(orphans)} that no CVar name here would produce:\n")
    for lbl, files in sorted(orphans.items()):
        print(f"  {lbl:34} {' '.join(sorted(files))}")
    if not orphans:
        print("  (none)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
