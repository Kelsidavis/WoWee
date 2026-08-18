#!/usr/bin/env python3
"""Globals a static popup's buttons call, that nothing answers.

    tools/staticpopup_verbs_check.py

A static popup is the last thing between a player and an irreversible action:
"this item will bind to you", "abandon your pet", "log out". Its OnAccept is a
button somebody presses on purpose. If that body calls a name nothing binds, the
click raises - and because the dialog is built, shown, and takes the click
first, the failure looks like the game ignoring a decision the player already
made, rather than like a missing feature.

The unbound-globals sweep already sees all of these, and files them under
"reached only from something a player has to do first". That grouping is right
for a menu nobody opens; it is wrong for these, because the popup is *shown by
the client's own event handler* and the player is being asked to answer it.
PetRename was in that pile until the naming dialog was found taking a name,
asking for confirmation, and raising.

WHAT IT LOOKS FOR

Every StaticPopupDialogs entry, every function-valued hook in it, and every
capitalised call inside those hooks. Method calls are skipped - `self:GetText()`
and `self.editBox:SetText()` are widget methods and belong to the other sweep.

WHAT IT CANNOT SEE

Whether the popup is reachable. Some are shown only by messages this client is
never sent: the arena team ones need an arena invite, and CONFIRM_ACCEPT_SOCKETS
needs the socketing UI. A name here is a raise *if* the dialog opens, and
whether it opens is a separate question the event-gap report answers.

Nor a call inside a helper the hook calls. This reads the hook bodies only.
"""
import pathlib
import re
import sys
import pathlib as _pathlib
sys.path.insert(0, str(_pathlib.Path(__file__).resolve().parent))
from framexml_source import without_comments

ROOT = pathlib.Path(__file__).resolve().parent.parent
XML = ROOT / "Data/interface"
ADDONS = ROOT / "src/addons"


def bound_names():
    """Every global the client binds or FrameXML itself defines."""
    out = set()
    for path in ADDONS.rglob("*.cpp"):
        text = path.read_text(errors="ignore")
        out |= set(re.findall(r'\{\s*(?:\.\w+\s*=\s*)?"(\w+)"', text))
        out |= set(re.findall(r'lua_setglobal\(L_?\s*,\s*(?:\.\w+\s*=\s*)?"(\w+)"', text))
        # The bootstrap's name lists, which are Lua source inside C strings.
        out |= set(re.findall(r"'(\w+)'", text))
        out |= set(re.findall(r'^\s*"?\s*(\w+)\s*=\s*function', text, re.M))
    for path in XML.rglob("*.lua"):
        text = path.read_text(errors="ignore")
        out |= set(re.findall(r"^\s*function\s+([A-Za-z_]\w*)\s*\(", text, re.M))
        out |= set(re.findall(r"^([A-Za-z_]\w*)\s*=\s*function", text, re.M))
    return out


# A call that is not preceded by ':' or '.', which would make it a method.
CALL = re.compile(r"(?<![:.\w])([A-Z][A-Za-z0-9_]{2,})\s*\(")
HOOK = re.compile(r"(\bOn\w+|\bEditBoxOn\w+)\s*=\s*function\b")


def hooks(body):
    """Each function-valued hook in a popup's table, by name and body.

    Sliced from one hook's `function` to the next hook's name rather than by
    matching `end`, because these bodies nest conditionals and a naive match
    stops at the first one.
    """
    starts = [(m.start(), m.group(1)) for m in HOOK.finditer(body)]
    for i, (pos, name) in enumerate(starts):
        end = starts[i + 1][0] if i + 1 < len(starts) else len(body)
        yield name, body[pos:end]


#: Names checked one at a time and found unreachable, with what settled each.
#:
#: A set rather than a count, because a count only says how many: fix one,
#: introduce another, and the number never moves. handler_announce_check was
#: pinned that way and hid a real one - the guild roster being emptied on
#: leaving a guild with nobody told - for as long as the count was all that was
#: pinned.
#:
#: Each of these belongs to a popup this client cannot put on screen. Checked
#: both ways in: the event that raises it is never fired, *and* the click path
#: that raises the same popup without an event cannot run either.
EXPECTED = {
    # Recruit-a-friend level granting. LEVEL_GRANT_PROPOSED is fired nowhere
    # and there is no click path to it.
    "AcceptLevelGrant": "LEVEL_GRANT_PROPOSED unreachable",
    "DeclineLevelGrant": "LEVEL_GRANT_PROPOSED unreachable",
    # END_REFUND and END_BOUND_TRADEABLE have two ways in and neither runs.
    # The events are fired nowhere; the item socketing frame raises the same
    # two popups from its Socket button, but only behind
    # GetSocketItemRefundable and GetSocketItemBoundTradeable, which both
    # answer nil - the per-item timer behind them is server state this client
    # is never sent, and answering yes would promise a refund that does not
    # exist.
    "EndRefund": "END_REFUND unreachable by event or by click",
    "EndBoundTradeable": "END_BOUND_TRADEABLE unreachable by event or by click",
    # This client does not put an enchant on the trade window, so
    # TRADE_REPLACE_ENCHANT has nothing to fire it.
    "ReplaceTradeEnchant": "TRADE_REPLACE_ENCHANT unreachable",
}


def main():
    bound = bound_names()
    popups = ROOT / "Data/interface/framexml/staticpopup.lua"
    text = popups.read_text(errors="ignore")
    # Line comments as well as block ones. Blizzard leaves dead calls in
    # these hooks with a note beside them - CAMP's OnAccept carries
    # `--ForceLogout();` under a line saying forced logout is not
    # finished - and counting those reported names nothing calls.
    text = re.sub(r"--\[\[.*?\]\]", "", text, flags=re.S)
    text = without_comments(text)
    blocks = re.findall(r'StaticPopupDialogs\["(\w+)"\]\s*=\s*\{(.*?)\n\};',
                        text, re.S)

    missing = {}
    for name, body in blocks:
        for hook_name, hook_body in hooks(body):
            # Names the hook declares for itself are not globals it is
            # missing. INSTANCE_LOCK's OnUpdate reads its own OnCancel out of
            # the popup table and then calls it - `local OnCancel =
            # StaticPopupDialogs["INSTANCE_LOCK"].OnCancel` - which read as a
            # call to a global nothing answers.
            local = set(re.findall(r"\blocal\s+([\w,\s]+?)\s*=", hook_body))
            local = {n.strip() for group in local for n in group.split(",")}
            for call in CALL.findall(hook_body):
                if call in local:
                    continue
                if call in EXPECTED:
                    continue
                if call not in bound:
                    missing.setdefault(call, set()).add(f"{name}.{hook_name}")

    print(f"{len(blocks)} static popups parsed\n")
    print(f"{len(missing)} name(s) a popup button calls and nothing answers:\n")
    for name in sorted(missing):
        where = sorted(missing[name])
        tail = f" (+{len(where) - 3})" if len(where) > 3 else ""
        print(f"  {name:32} {', '.join(where[:3])}{tail}")
    if not missing:
        print("  (none)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
