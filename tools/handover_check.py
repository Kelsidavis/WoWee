#!/usr/bin/env python3
"""The client's side of a handover: dead calls, and keys with two owners.

Handing a panel to FrameXML is two edits in two files - stop drawing it, and
send the key that opened it somewhere else - and both fail quietly when they go
wrong. Neither shows up as an error, a warning, or a log line. The panel simply
does not appear, which is also what a panel that was never handed over looks
like.

    tools/handover_check.py

TWO THINGS IT CHECKS

**Interface commands naming a function FrameXML does not define.** The client
asks the interface to do something by running a line of Lua. An unknown global
is nil, calling it yields nothing, and the client cannot tell that from a call
that worked. ToggleAllBags and ToggleWorldMap are later additions that 3.3.5
never had, and both were being called here - the bags key did nothing at all
for as long as it was written that way.

**A keybinding action driving the interface from more than one file.**
ImGui::IsKeyPressed does not consume a press, so every site that asks sees the
same one. Two sites that both toggle the same panel cancel each other within a
frame: it opens and shuts and nothing reaches the screen. That was the
character panel, reported three times, with every link in its chain correct -
because the chain was correct and ran twice.

Driving the interface, rather than merely watching the key: four actions are
watched from two files each and none of them clash, because a panel that is no
longer drawn still polls its key at the top of its own render and flips a flag
nothing reads. Only a second site that also runs a line of Lua can cancel the
first.

WHAT IT CANNOT SEE

Whether a function that exists does the right thing. Nor a driver separated
from its action by more than REACH characters of code, by another action, or by
a wrapper of the client's own - the window stops at the next action mentioned,
which is what keeps one key's block from borrowing the next one's call, and it
would equally hide a real driver written past one.

Both sections are read-and-judge, not verdicts. Both were checked against the
tree that had the bugs: it reports the two dead names and the two shared keys,
and the tree with them fixed reports nothing.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
SRC = ROOT / "src"
FRAMEXML = ROOT / "Data/interface"

# A Lua line the client hands to the interface, however it is spelled.
COMMAND = re.compile(r'(?:runInterfaceCommand|executeString)\s*\(\s*"((?:[^"\\]|\\.)*)"')
# Every global call inside such a line. Interface functions are capitalised;
# a lowercase name is a local or a method and is not ours to resolve.
CALL = re.compile(r'\b([A-Z][A-Za-z0-9_]*)\s*\(')
# Every mention of an action, not every isActionPressed call: the route table
# polls through a loop variable, so matching the call site saw one of the two
# owners of the character key and reported no conflict at all.
POLL = re.compile(r'\bAction::([A-Z_][A-Z_0-9]*)\b')
# How much code past a mention to look for the interface call it drives, in
# characters with comments blanked and whitespace collapsed. The route table
# lists its actions and runs them from a loop a few lines below.
REACH = 500
# A site that runs a line of Lua, as opposed to one that only reads the key.
# A site that runs a line of Lua, as opposed to one that only reads the key.
# The third alternative is a route table entry, which names its command in a
# string beside the action and leaves the running of it to a loop below.
DRIVES = re.compile(r'runInterfaceCommand|executeString|"[A-Z]\w*\(')
# Where the actions are declared, named and written to the settings file. Every
# one appears in each, which says nothing about who acts on it.
REGISTRY = {"src/ui/keybinding_manager.cpp", "src/addons/lua_action_api.cpp"}


def lua_globals():
    """Every function name the shipped interface defines at global scope."""
    names = set()
    for path in FRAMEXML.rglob("*.lua"):
        text = path.read_text(errors="ignore")
        names |= set(re.findall(r"^\s*function\s+([A-Za-z_]\w*)\s*\(", text, re.M))
        # Assigned rather than declared: `Foo = function(...)`.
        names |= set(re.findall(r"^\s*([A-Za-z_]\w*)\s*=\s*function\s*\(", text, re.M))
    return names


def blank_comments(text):
    """C++ comments replaced by spaces, keeping every offset and line number.

    Not removed: the search for the interface call a key drives runs over a
    window of following text, and this codebase comments heavily enough that
    ten lines of explanation ate the whole window. The character key's own call
    sat just past it, so the sweep missed the bug it was written for.
    """
    out = list(text)
    i, n = 0, len(text)
    while i < n:
        two = text[i:i + 2]
        if two == "//":
            j = text.find("\n", i)
            j = n if j < 0 else j
        elif two == "/*":
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
        elif text[i] in "\"'":
            quote, j = text[i], i + 1
            while j < n and text[j] != quote:
                j += 2 if text[j] == "\\" else 1
            i = j + 1
            continue
        else:
            i += 1
            continue
        for k in range(i, j):
            if out[k] != "\n":
                out[k] = " "
        i = j
    return "".join(out)


def cpp_sources():
    for path in sorted(SRC.rglob("*.cpp")):
        yield path, path.read_text(errors="ignore")


def main():
    defined = lua_globals()
    # Names the engine answers itself rather than FrameXML, so absence from the
    # interface says nothing about them.
    bindings = set()
    for path in (SRC / "addons").rglob("*.cpp"):
        text = path.read_text(errors="ignore")
        bindings |= set(re.findall(r'\{\s*(?:\.\w+\s*=\s*)?"([A-Za-z_]\w*)"\s*,', text))
        bindings |= set(re.findall(r'lua_setglobal\s*\(\s*\w+\s*,\s*(?:\.\w+\s*=\s*)?"([A-Za-z_]\w*)"', text))

    dead, polls, commands = [], {}, 0
    for path, text in cpp_sources():
        rel = path.relative_to(ROOT)
        for m in COMMAND.finditer(text):
            commands += 1
            line = text.count("\n", 0, m.start()) + 1
            body = m.group(1).replace('\\"', '"')
            # A definition is not a call. `function CalendarEventGetTypes()`
            # in a bootstrap chunk looks exactly like a call to it, and
            # reporting it says the name does not exist at the very moment it
            # is being brought into existence.
            declared = set(re.findall(r'\bfunction\s+([A-Za-z_]\w*)\s*\(', body))
            for call in CALL.finditer(body):
                name = call.group(1)
                if name in declared:
                    continue
                if name not in defined and name not in bindings:
                    dead.append((rel, line, name, body[:60]))
        if rel.as_posix() in REGISTRY:
            continue
        bare = blank_comments(text)
        for m in POLL.finditer(bare):
            # Only where the key drives the interface. Four other actions are
            # watched from two files each and none of them clash: a panel that
            # is no longer drawn still polls its key at the top of its render
            # and flips a flag of its own, which nothing reads. What broke the
            # character key was two sites driving FrameXML's one state.
            # Stop at the next action mentioned, whatever it is. Without that
            # the window ran out of one key's block and into the following
            # one's, and borrowed its call: the character key still looked
            # driven from two files after only one of them still drove it.
            nxt = POLL.search(bare, m.end())
            end = min(m.end() + REACH * 8, nxt.start() if nxt else len(bare))
            window = re.sub(r"\s+", " ", bare[m.end():end])
            if not DRIVES.search(window[:REACH]):
                continue
            line = text.count("\n", 0, m.start()) + 1
            polls.setdefault(m.group(1), []).append((rel, line))

    print(f"{len(defined)} interface functions, {len(bindings)} engine bindings, "
          f"{commands} interface commands, {sum(len(v) for v in polls.values())} "
          f"action mentions over {len(polls)} actions\n")

    print(f"{len(dead)} call(s) naming nothing that exists:\n")
    for rel, line, name, body in dead or []:
        print(f"  {rel}:{line}: {name} - in '{body}'")
    if not dead:
        print("  (none)")

    # More than one file, rather than more than one line: a file may look at a
    # key twice for its own reasons, and two files rarely mean to.
    shared = {a: sites for a, sites in polls.items()
              if len({r for r, _ in sites}) > 1}
    print(f"\n{len(shared)} action(s) acted on in more than one file:\n")
    for action, sites in sorted(shared.items()):
        print(f"  {action}")
        for rel, line in sites:
            print(f"      {rel}:{line}")
    if not shared:
        print("  (none)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
