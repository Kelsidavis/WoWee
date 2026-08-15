#!/usr/bin/env python3
"""What this client draws itself, scaled to the screen it is on.

The interface's own frames are laid out in a fixed canvas and scaled to fit, so
they follow a tall screen by themselves. Everything this client draws is in
pixels and does not: an action bar slot is 48 pixels times its scale, and on a
2160-line screen that is 48 pixels.

So four things pick a default from the screen height - the buff bar scales
itself directly, and the bags, the action bars and this client's own windows
take their default that way. They have to pick the same, or neighbouring parts
of one HUD come up at different sizes. They did not: the buff bar used the
height over a reference height while the rest used steps of their own, and at
2160 lines that was 2.0 against 1.2.

This starts the client with nothing in its config but a screen height, and asks
what each of them chose. The answer has to be the buff bar's rule, held to
whatever range that setting's own row allows - a scale of 2 is what a 2160-line
screen wants and the window scale's row stops at 1.5, so 1.5 is the right answer
there and 2 is the right answer for the bars.

Canaried by putting one of them back on the steps it used to use, which reports
that scale at three of the four heights.

Note which assignment that has to be. Each of these defaults is set twice: once
before the config is read, from whatever height is known then, and again
afterwards from the height the file turned out to hold. The second is the one
that decides - loadSettings runs from the constructor, where there is no display
to ask - so a canary on the first is silently corrected by the second and the
check goes on passing.

Needs build/bin/framexml_run and an extracted interface, and skips rather than
fails without them.
"""

import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RUNNER = ROOT / "build" / "bin" / "framexml_run"
DATA = ROOT / "Data"
INTERFACE = ROOT / "Data/interface"
CONFIG_ROOT = ROOT / "logs/settings_display_scale_config"

# The buff bar's rule, from BuffBarMetrics: the height over a reference height,
# held between the bounds it uses. Written here because this checks the client
# against the rule rather than against itself.
REFERENCE_HEIGHT = 1080.0
MIN_AUTO, MAX_AUTO = 0.75, 2.0

# setting key -> the range its own row allows, which is what its default is
# held to.
WATCHED = {
    "actionbarscale": (0.5, 2.0),
    "windowuiscale": (0.75, 1.5),
    "bagscale": (0.75, 1.5),
}

HEIGHTS = [768, 1080, 1440, 2160]


def wanted(height, lo, hi):
    rule = min(MAX_AUTO, max(MIN_AUTO, height / REFERENCE_HEIGHT))
    return min(hi, max(lo, rule))


def main():
    if not RUNNER.is_file() or not INTERFACE.is_dir():
        print("framexml_run or an extracted interface is missing - no scale was asked for.")
        return 0

    lua = ('local out = {} ' +
           "".join(f'out[#out+1] = "{k}=" .. tostring(WoweeGetSetting("{k}")) '
                   for k in WATCHED) +
           'error("QQ" .. "SCALE " .. table.concat(out, " "))')

    bad, asked = [], 0
    for height in HEIGHTS:
        shutil.rmtree(CONFIG_ROOT, ignore_errors=True)
        CONFIG_ROOT.mkdir(parents=True, exist_ok=True)
        # Nothing but the height, so every scale below is a default rather than
        # something the file said.
        (CONFIG_ROOT / "settings.cfg").write_text(f"resolution_height={height}\n")

        env = dict(os.environ)
        env["WOWEE_CONFIG_ROOT"] = str(CONFIG_ROOT)
        try:
            run = subprocess.run([str(RUNNER), str(DATA), lua],
                                 capture_output=True, text=True, timeout=900, env=env)
        except subprocess.TimeoutExpired:
            print("the runner did not finish - no scale was asked for.")
            return 1

        payload = None
        for line in (run.stdout + run.stderr).splitlines():
            at = line.find("QQ" + "SCALE ")
            if at != -1:
                payload = line[at + len("QQSCALE "):]
                break
        if payload is None:
            bad.append(f"at {height} lines the client said nothing")
            continue

        got = {}
        for item in payload.split():
            key, _, value = item.partition("=")
            got[key] = value

        for key, (lo, hi) in WATCHED.items():
            asked += 1
            want = wanted(height, lo, hi)
            try:
                near = abs(float(got.get(key, "nan")) - want) <= 0.01
            except ValueError:
                near = False
            if not near:
                bad.append(f"at {height} lines {key} came up at {got.get(key)} "
                           f"where the rule and its own row give {want:g}")

    print(f"{asked} defaults asked for across {len(HEIGHTS)} screen heights.\n")
    for entry in bad:
        print(f"  {entry}")

    if bad:
        print(f"\n{len(bad)} default(s) do not follow the screen the way the buff bar does.")
        return 0

    if asked < len(HEIGHTS) * len(WATCHED):
        print("\nfewer defaults were asked for than there are to ask - the client "
              "stopped answering rather than answering right.")
        return 1

    print("every default this client picks for itself follows the screen the same way.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
