#!/usr/bin/env python3
"""Config keys read into a field nothing else looks at.

A value written to settings.cfg and read back out of it looks like a working
setting from every angle a check normally takes: the key is there, the value
survives a restart, the field holds what the file said. None of that asks
whether anything acts on it.

This client kept three fields from when it drew its own chat window - whether
the window was locked, and the size a resize left it. FrameXML draws chat now
and chat_panel.cpp sets no window size at all, so the width and the height were
written every run and loaded back into fields nobody asked. The comments beside
them described capturing a resize and a reference kept in sync, neither of which
existed, which is what makes this shape worth a check rather than a read-through:
it looks implemented.

So for each key the loader knows, this takes the field it is loaded into and
asks whether anything outside the loader mentions it.

Two things it does not report, both on purpose. A local variable the loader
clamps into before assigning a member is not a field. And a field the *saver*
reads is still only the file talking to itself, so the saver does not count as a
reader either - which is the whole point, the chat window's size being saved
faithfully every run.

The existing dead_setting_check covers the other side of this: a CVar an options
panel control names that nothing reads. Between them, a setting has to be acted
on somewhere whichever file it lives in.
"""

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LOADER = ROOT / "src/ui/game_screen_minimap.cpp"

# The lvalue a config key is read into: the whole thing, so that the first
# letter is not eaten into a qualifier. An earlier version captured
# `psMeterSavedX_` for `dpsMeterSavedX_` and reported six fields as unread
# because it was grepping for names that do not exist.
ASSIGN = re.compile(r'key == "([a-z0-9_]+)"(?:(?!key ==)[\s\S]){0,200}?\b([A-Za-z_][\w:.]*)\s*=[^=]')
# A local declared in the loader and clamped into before a member is set.
LOCAL = re.compile(r'\b(?:int|float|bool|double|auto|const\s+\w+)\s+(\w+)\s*=')


def main():
    if not LOADER.is_file():
        print("the settings loader is missing - nothing checked.")
        return 1

    text = LOADER.read_text()
    locals_ = set(LOCAL.findall(text))

    fields = {}
    for match in ASSIGN.finditer(text):
        key, lvalue = match.group(1), match.group(2)
        member = lvalue.split(".")[-1].split("::")[-1]
        if not member or member in locals_:
            continue
        fields.setdefault(key, member)

    unread = []
    for key, member in sorted(fields.items()):
        found = subprocess.run(
            # Absolute, because this is not always run from the root: under
            # sweep_guard the working directory is the build tree, where "src"
            # does not exist - so every field found nothing and looked unread.
            ["grep", "-rn", "--include=*.cpp", "--include=*.hpp", "-w", member,
             str(ROOT / "src"), str(ROOT / "include")],
            capture_output=True, text=True).stdout.splitlines()
        # Three kinds of mention are not a use of the value.
        #
        # Its own declaration, which every field in a header has - without this
        # every one of them looks read, and the first version of this passed
        # with the chat window's size put back.
        #
        # The line the loader assigns it from, and the line the saver writes it
        # on. Those two are the file talking to itself, which is the whole shape
        # being looked for.
        #
        # Anything else counts, including in the loader: the damage meter's
        # position is read out of the file and handed to setDPSMeterPos on the
        # next line, and excluding that whole file called it unread.
        declaration = re.compile(
            r':\s*\d+:\s*(?:static\s+)?(?:const\s+)?'
            r'(?:float|int|bool|double|unsigned|std::\w+|ImVec2)\s+' + member + r'\s*[=;{]')
        readFromFile = re.compile(re.escape(member) + r'\s*=\s*(?:std::(?:sto[fid]|max|clamp)|\(|val\b)')
        writtenToFile = re.compile(r'out\s*<<')
        elsewhere = [line for line in found
                     if not declaration.search(line)
                     and not readFromFile.search(line)
                     and not writtenToFile.search(line)]
        if not elsewhere:
            unread.append((key, member))

    print(f"{len(fields)} config keys read into a field of their own.\n")
    for key, member in unread:
        print(f"  {key}: read into {member}, which nothing outside the settings "
              "file's own reading and writing ever looks at")

    if unread:
        print(f"\n{len(unread)} key(s) are kept in the config and acted on by nothing.")
        return 0

    if len(fields) < 90:
        print(f"\nonly {len(fields)} keys resolved to a field, which is fewer than the "
              "loader has - the parse stopped matching rather than finding nothing.")
        return 1

    print("every setting kept in the config file is read by something that uses it.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
