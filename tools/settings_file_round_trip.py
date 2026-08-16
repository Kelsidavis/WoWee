#!/usr/bin/env python3
"""Every line of the settings file, changed and read back.

The restart check drives settings through Lua, which reaches the seventy-two
rows of the schema and the six a Blizzard control drives. The file holds more
than that: where the quest tracker sits and how big it is, the chat window's
size and which tab was open, where the damage meter was left, the resolution.
Nothing reaches those from Lua and nothing else here checks them at all.

So this works on the file. A run writes it, every value in it is moved to
another value inside the range the loader will accept, and a second run loads
and writes it again. What comes back has to be what was put in.

A value moved outside its range proves nothing - the loader clamps, correctly,
and the check would be reporting its own bad arithmetic. The ranges come from
the clamps in the loader itself, which is the only machine-readable statement of
what each field will hold: a first version of this set every scale to zero and
reported five settings as lost when all five had been held to their minimum
exactly as intended.

The keybindings at the end of the file are moved too, onto keys the manager can
put a name to - it writes F1 through F12, single letters and a few named keys,
and nothing else - and that nothing is bound to. Not onto W, A, S, D, Q or E:
those are reserved so they cannot become UI shortcuts, and a binding onto one is
refused. Both of those cost a run to find, each looking like the file losing
bindings.

One key is expected not to survive and is named below rather than tolerated
silently.

Needs build/bin/framexml_run and an extracted interface, and skips rather than
fails without them.
"""

import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import settings_config_parse  # noqa: E402  (a sibling, not a package)

ROOT = Path(__file__).resolve().parent.parent
RUNNER = ROOT / "build" / "bin" / "framexml_run"
DATA = ROOT / "Data"
INTERFACE = ROOT / "Data/interface"
CONFIG_ROOT = ROOT / "logs/settings_round_trip_config"

# Read and deliberately dropped, so every run starts north-up. It dates from a
# commit about correcting the minimap's orientation, and the note where it is
# dropped says so.
EXPECTED_TO_DROP = {"minimap_rotate"}



def run():
    env = dict(os.environ)
    env["WOWEE_CONFIG_ROOT"] = str(CONFIG_ROOT)
    return subprocess.run([str(RUNNER), str(DATA), "return 1"],
                          capture_output=True, text=True, timeout=900, env=env)


# Keys nothing is bound to by default, for moving a binding onto without
# taking one off something else - a key belongs to one action at a time, so
# reusing one would be checking the conflict rule rather than the file.
# Keys the manager can put a name to - it writes F1 through F12 and single
# letters, and nothing else - which nothing is bound to by default. A key
# belongs to one action at a time, so reusing a bound one would be checking the
# conflict rule rather than the file. F13 upwards were the first choice and are
# not names this client has: twelve bindings came back on their defaults because
# what was written could not be read, which reads exactly like the file losing
# them.
# Not W, A, S, D, Q or E: the manager reserves the movement keys so they cannot
# become UI shortcuts, and refuses a binding onto one. Six of these came back on
# their defaults with those in the pool, which reads as the file losing them and
# is the client declining to do something it should decline to do.
SPARE_KEYS = ["G", "H", "R", "T", "U", "X", "Z",
              "F5", "F6", "F7", "F9", "F10", "F11", "F12", "Home"]


def moved(key, value, ranges, spare=None):
    """Another value this key will accept, or None to leave it alone."""
    try:
        now = float(value)
    except ValueError:
        # A binding: the section holds a key name rather than a number, and it
        # is the last part of this file nothing else watches through a change.
        if spare:
            return spare.pop()
        return None
    lo, hi = ranges.get(key, (None, None))
    if lo is None and value in ("0", "1"):
        # A bool, as far as anything here can tell. Only when the key has no
        # range of its own: a scale sitting at exactly 1 is not a bool, and
        # flipping it to 0 put four of them under their own minimum and
        # reported the clamp that caught it as the file losing them.
        return "1" if value == "0" else "0"
    if lo is None:
        # No clamp of its own: a small step, which every field here accepts.
        return str(int(now) + 1) if now == int(now) else f"{now + 0.5:g}"
    # A quarter and three quarters of the way along, whichever it is not near.
    first, second = lo + (hi - lo) * 0.25, lo + (hi - lo) * 0.75
    want = second if abs(now - first) < abs(now - second) else first
    # A field holding a whole number will truncate a fraction, and the check
    # would be reporting its own arithmetic again.
    if now == int(now) and lo == int(lo) and hi == int(hi):
        want = float(int(want))
        if want == now:
            want = float(int(first if want != int(first) else second))
    return f"{want:g}" if want != int(want) else str(int(want))


def main():
    if not RUNNER.is_file() or not INTERFACE.is_dir():
        print("framexml_run or an extracted interface is missing - no file was written.")
        return 0

    shutil.rmtree(CONFIG_ROOT, ignore_errors=True)
    CONFIG_ROOT.mkdir(parents=True, exist_ok=True)
    try:
        run()  # writes the file the way the client writes it
        settings = CONFIG_ROOT / "settings.cfg"
        if not settings.is_file():
            print("the run wrote no settings file - nothing to check.")
            return 1

        ranges = settings_config_parse.rangesByKey()
        lines, wanted = [], {}
        spare, inBindings = list(SPARE_KEYS), False
        for line in settings.read_text().splitlines():
            if not line.strip() or line.startswith("["):
                inBindings = line.strip() == "[Keybindings]"
                lines.append(line)
                continue
            key, _, value = line.partition("=")
            want = moved(key, value.strip(), ranges, spare if inBindings else None)
            if want is None or want == value.strip():
                lines.append(line)
                continue
            lines.append(f"{key}={want}")
            wanted[key] = want
        settings.write_text("\n".join(lines) + "\n")

        run()  # loads what was written, and writes it again
        came = {}
        for line in settings.read_text().splitlines():
            key, _, value = line.partition("=")
            came[key] = value.strip()
    except subprocess.TimeoutExpired:
        print("the runner did not finish - no file was checked.")
        return 1

    lost = []
    for key, want in sorted(wanted.items()):
        if key in EXPECTED_TO_DROP:
            continue
        got = came.get(key)
        if got is None:
            lost.append(f"{key}: written as {want} and gone from the file entirely")
            continue
        try:
            same = abs(float(got) - float(want)) <= 0.0001
        except ValueError:
            same = got == want
        if not same:
            lost.append(f"{key}: written as {want} and came back as {got}")

    print(f"{len(wanted)} of the file's values moved and read back.\n")
    for entry in lost:
        print(f"  {entry}")

    if lost:
        print(f"\n{len(lost)} value(s) did not survive being written and read.")
        return 0

    # The values come from the file being checked, so a key that stops being
    # written is not a failure here - it is one fewer thing checked, and the
    # check passes. Only the count catches that, which is why it is pinned
    # rather than loose: dropping the quest tracker's width from the saver took
    # this from 106 to 105 and it still said everything was fine.
    #
    # If a setting was removed on purpose, lower this in the same commit and say
    # which. If one was added, raise it.
    # 121 until the chat window's width and height went, and 119 until which
    # chat tab was open went after them - the same shape, found by the check
    # written from the first two. They were written and
    # read every run into fields nothing else touched: this client drew its own
    # chat window once, FrameXML draws it now, and the two numbers outlived it.
    # 118 until bank_combine_bags went with this client's bank window, which
    # was the only thing that read it.
    if len(wanted) < 117:
        print(f"\n{len(wanted)} values were moved where 117 were expected - a key "
              "has stopped being written to the file, so nothing here is checking "
              "it any more.")
        return 1

    print("every value written to the settings file is there when it is read back.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
