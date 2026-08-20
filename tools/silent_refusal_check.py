#!/usr/bin/env python3
"""A binding the player reached that refused, and said nothing about it.

    tools/silent_refusal_check.py

An action the interface asks for arrives here as a Lua binding: DoTradeSkill,
PickupContainerItem, AcceptQuest. Each one can decline - the index is past the
end of a list, a window is not open, a lookup missed - and declining is often
right. What is not right is declining in silence, because from outside the
client a verb that refused and a verb that was never wired look identical: the
button is pressed, nothing happens, nothing is raised, nothing is logged.

WHY THE LOG LEVEL IS PART OF THE QUESTION

The log a bug report arrives with is warnings only. A refusal reported at info
or debug is therefore invisible in exactly the session that reproduced the
fault, which is the only session that matters. DoTradeSkill is the case that
started this: it carried the right lines, written against this exact problem -
"a craft that does nothing is indistinguishable from a button that was never
wired" - and said them at debug. A report of crafting doing nothing came back
with no line about the press at all, and the fault under it turned out to be a
use-after-free that no other sweep here could see.

So a binding that speaks below warning is listed separately rather than passed.
It has the line already; it is one word from being useful.

WHAT COUNTS AS THE POPULATION

Not every binding. A getter answering nil is an answer, not a refusal, so only
bindings named with an action verb are read - and only those that actually do
something, which is taken as calling a verb on the game handler. That is what
separates DoTradeSkill from GetTradeSkillInfo, and it is the difference between
a list worth reading and four hundred entries nobody will.

WHAT COUNTS AS A REFUSAL

A guarded return that sits before the first thing the binding does. A return
after the work is the end of the function. A return before it means the press
went nowhere.

Guards on the game handler itself are skipped. `if (!gh) return 0;` is the
idiom every binding opens with and it does not fail in play; the guards worth
reading are the ones a player's own input can trip - an index off the end of a
list, a window that is not open, a lookup that missed.

WHAT IT CANNOT SEE

Whether the refusal is correct. Most are: this is a list of decisions made
without a record, not a list of wrong decisions.

Anything that is not a Lua binding. The drop half of a drag was found the same
way and lives in the input pump in lua_engine.cpp, where a release over a frame
that takes no drop said nothing at all. That shape is real and this does not
reach it.

Speech that happens in a helper the guard calls rather than in the branch. Read
as silent here, which is a false alarm, and part of why the count is a ceiling
to ratchet down rather than a zero to hold.

Work done inside the guard's own condition is handled: a bare call on L or gh
is this file's "something else took this press" idiom and is skipped. A helper
called some other way is not, and will read as a refusal.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
BINDINGS = sorted((ROOT / "src/addons").glob("lua_*.cpp"))

# A name that does something, rather than answering about something.
ACTION_VERBS = (
    "Accept", "Apply", "Attack", "Begin", "Buy", "Buyout", "Cancel", "Cast",
    "Click", "Close", "Confirm", "Craft", "Create", "Decline", "Delete",
    "Deposit", "Do", "Drop", "Equip", "Give", "Invite", "Join", "Kick",
    "Learn", "Leave", "Loot", "Move", "Open", "Pick", "Pickup", "Place",
    "Promote", "Purchase", "Put", "Repair", "Reset", "Resurrect", "Roll",
    "Sell", "Send", "Split", "Start", "Stop", "Summon", "Take", "Toggle",
    "Train", "Use", "Withdraw",
)

# Saying so, at a level the log a report arrives with actually carries.
LOUD = re.compile(r"\b(LOG_WARNING|LOG_ERROR|raiseUiError|addUIError|luaL_error"
                  r"|addSystemChatMessage|addLocalChatMessage)\b")
# Saying so where nobody will read it.
QUIET = re.compile(r"\b(LOG_INFO|LOG_DEBUG)\b")

# An argument checked against its bounds, rather than a piece of game state
# that was not there. The interface passes the arguments, so a bound it breaks
# is a fault in this client's own tables; a lookup that missed is what a player
# can walk into, and it is the shape both real cases had.
RANGE_CHECK = re.compile(r"[<>]")

# `if (completedItemTarget(L, ...)) return;` is not a refusal. It is this
# file's idiom for "something else took this press" - the helper does the work
# and answers whether it did, and returning is how the caller steps aside. A
# bare call on L or gh, with no negation and no comparison, is that shape;
# `if (!cursorWireSlot(...))` is not, and stays.
HANDLED_ELSEWHERE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*\((?:L|gh)[,)]")

# Reading the game rather than changing it.
READ_ONLY = re.compile(r"^(get|is|has|can|num|count|find|resolve|lookup|peek)")
# A call on the game handler, whichever way this file spells it.
ACTS = re.compile(r"(?:gh|owner_|handler)\s*->\s*([A-Za-z_][A-Za-z0-9_]*)\s*\(")


def strip_noise(text, strings=True):
    """Blank comments, and string bodies when asked, preserving every offset.

    Two versions are needed and they must stay aligned. Brace matching has to
    have the strings gone: the interface bootstrap in lua_engine.cpp is Lua
    source held in C++ string literals, full of braces that are not code. But
    the binding *names* are string literals too, so a scan for them has to run
    over text where the strings survive. Blanking in place rather than deleting
    is what lets one set of offsets serve both.
    """
    out = list(text)
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if strings and (c == '"' or c == "'"):
            quote, j = c, i + 1
            while j < n:
                if text[j] == "\\":
                    j += 2
                    continue
                if text[j] == quote:
                    break
                if text[j] == "\n":
                    break
                j += 1
            for k in range(i, min(j + 1, n)):
                if text[k] != "\n":
                    out[k] = " "
            i = j + 1
            continue
        if text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            for k in range(i, j):
                out[k] = " "
            i = j
            continue
        if text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            for k in range(i, j):
                if text[k] != "\n":
                    out[k] = " "
            i = j
            continue
        i += 1
    return "".join(out)


def body_at(clean, brace_pos):
    """The span of a block whose opening brace is at brace_pos."""
    depth, i, n = 0, brace_pos, len(clean)
    while i < n:
        if clean[i] == "{":
            depth += 1
        elif clean[i] == "}":
            depth -= 1
            if depth == 0:
                return brace_pos, i + 1
        i += 1
    return brace_pos, n


def bindings(named, clean):
    """Every registered binding as (name, start, end).

    `named` has its strings intact so the names can be read; `clean` has them
    blanked so the braces can be counted. Same offsets in both.

    Both forms the files use: a name beside a lua_ function defined elsewhere,
    and a name beside a lambda written in place.
    """
    found = []
    # {"Name", [](lua_State* L) -> int { ... }}
    for m in re.finditer(r'\{\s*"([A-Za-z_][A-Za-z0-9_]*)"\s*,\s*\[\]\s*\(\s*lua_State',
                         named):
        brace = clean.find("{", m.end())
        if brace < 0:
            continue
        s, e = body_at(clean, brace)
        found.append((m.group(1), s, e))
    # {"Name", lua_Foo}, with the definition somewhere in the same file
    for m in re.finditer(r'\{\s*"([A-Za-z_][A-Za-z0-9_]*)"\s*,\s*(lua_[A-Za-z0-9_]+)\s*\}',
                         named):
        d = re.search(r"\bint\s+" + re.escape(m.group(2)) + r"\s*\(\s*lua_State[^)]*\)\s*\{",
                      clean)
        if not d:
            continue
        s, e = body_at(clean, clean.rindex("{", d.start(), d.end()))
        found.append((m.group(1), s, e))
    return found


def first_action(body):
    """Offset of the first call that changes the game, or None."""
    best = None
    for m in ACTS.finditer(body):
        if READ_ONLY.match(m.group(1)):
            continue
        if best is None or m.start() < best:
            best = m.start()
    return best


def guarded_returns(body, limit):
    """Guarded returns before `limit`, as (line_offset, condition, branch text).

    The branch is brace-matched when it has braces, so a long one is read whole
    rather than through a fixed window - the mistake the ownership sweeps made
    three times at three different widths.
    """
    out = []
    for m in re.finditer(r"\bif\s*\(", body):
        if m.start() >= limit:
            continue
        # the condition
        depth, i = 0, m.end() - 1
        while i < len(body):
            if body[i] == "(":
                depth += 1
            elif body[i] == ")":
                depth -= 1
                if depth == 0:
                    break
            i += 1
        cond = body[m.end():i]
        rest = body[i + 1:]
        stripped = rest.lstrip()
        lead = len(rest) - len(stripped)
        if stripped.startswith("{"):
            s, e = body_at(rest, lead)
            branch = rest[s:e]
        else:
            end = rest.find(";", lead)
            branch = rest[lead:end + 1] if end > 0 else stripped[:200]
        if not re.search(r"\breturn\b", branch):
            continue
        # A branch that does the work and then returns is not a refusal, it is
        # the function finishing early. PickupGuildBankItem's deposit arm is
        # one: `if (cursorWireSlot(...)) { guildBankDepositItem(...); return; }`
        # opens before the first action in the body and so came out as a silent
        # refusal on the first run.
        if ACTS.search(branch) and not READ_ONLY.match(ACTS.search(branch).group(1)):
            continue
        out.append((m.start(), cond.strip(), branch))
    return out


def main():
    state_silent, range_silent, quiet_only = [], [], []
    considered = 0

    for path in BINDINGS:
        raw = path.read_text(errors="ignore")
        named = strip_noise(raw, strings=False)
        clean = strip_noise(raw)
        for name, s, e in bindings(named, clean):
            if not any(name.startswith(v) for v in ACTION_VERBS):
                continue
            body_clean = clean[s:e]
            act = first_action(body_clean)
            if act is None:
                continue          # answers rather than acts
            considered += 1
            for off, cond, branch in guarded_returns(body_clean, act):
                # The handler guard every binding opens with, which does not
                # fail in play. The guards worth reading are the ones a
                # player's own input can trip.
                bare = cond.replace(" ", "")
                if bare in ("!gh", "!handler", "!L", "!L_", "gh==nullptr", "!gh||!L"):
                    continue
                # Only the branch's own text decides, and it comes from the
                # cleaned copy - so a comment that mentions LOG_WARNING does
                # not read as speech.
                line = raw[:s].count("\n") + body_clean[:off].count("\n") + 1
                entry = (path.name, line, name, bare[:46])
                if LOUD.search(branch):
                    continue
                if QUIET.search(branch):
                    quiet_only.append(entry)
                elif HANDLED_ELSEWHERE.match(bare):
                    continue
                elif RANGE_CHECK.search(bare):
                    range_silent.append(entry)
                else:
                    state_silent.append(entry)

    for group in (state_silent, range_silent, quiet_only):
        group.sort(key=lambda r: (r[0], r[1]))

    print(f"{considered} action binding(s) that change the game, across "
          f"{len(BINDINGS)} files\n")

    print(f"{len(state_silent)} state or lookup refusal(s) that say nothing:\n")
    for f, line, name, cond in state_silent:
        print(f"  {name:34} {f}:{line}  if ({cond})")
    if not state_silent:
        print("  (none)")

    print(f"\n{len(range_silent)} argument-bound refusal(s) that say nothing - "
          f"the interface supplies these, so a broken bound is this client's "
          f"own fault rather than something a player walks into:\n")
    for f, line, name, cond in range_silent:
        print(f"  {name:34} {f}:{line}  if ({cond})")
    if not range_silent:
        print("  (none)")

    print(f"\n{len(quiet_only)} that speak below warning, so the log a report "
          f"arrives with does not carry them:\n")
    for f, line, name, cond in quiet_only:
        print(f"  {name:34} {f}:{line}  if ({cond})")
    if not quiet_only:
        print("  (none)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
