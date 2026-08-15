"""The settings loader's own statement of what each field will hold.

`GameScreen::loadSettings` clamps almost every value it reads, and that clamp is
the only machine-readable statement of a setting's range for the ones with no
schema row - which is exactly the six a Blizzard control drives. Two checks want
it, from different ends: one asks what a CVar's slider is allowed to hand to a
field, and one wants a value to perturb a config line to. They were parsing it
separately until tool_duplication_check said so.

Not a script. It defines the parse and nothing else, so importing it costs a
reader one hop rather than a second copy to keep in step.
"""

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LOADER = ROOT / "src/ui/game_screen_minimap.cpp"

# The gap must not cross another `key ==`, or a key with no clamp of its own
# takes the next one's - which put a quest tracker's height on a scale's range
# once, and a bool on a distance's another time.
CLAMP = re.compile(
    r'key == "([a-z0-9_]+)"(?:(?!key ==)[\s\S]){0,300}?'
    r'settingsPanel_\.(\w+)\s*=\s*std::clamp\([^,]+,\s*([-\d.]+)f?\s*,\s*([-\d.]+)f?\s*\)')

# Some are clamped into a local or another object rather than straight into a
# pending field, so the member is not always there to be had.
LOOSE = re.compile(
    r'key == "([a-z0-9_]+)"(?:(?!key ==)[\s\S]){0,300}?'
    r'std::clamp\([^,]+,\s*([-\d.]+)f?\s*,\s*([-\d.]+)f?\s*\)')


def clampedRanges():
    """config key -> (member or None, lo, hi) for every value the loader bounds."""
    text = LOADER.read_text()
    out = {}
    for m in LOOSE.finditer(text):
        out.setdefault(m.group(1), (None, float(m.group(2)), float(m.group(3))))
    for m in CLAMP.finditer(text):
        out[m.group(1)] = (m.group(2), float(m.group(3)), float(m.group(4)))
    return out


def rangesByMember():
    """member -> (lo, hi), for a caller that starts from the field."""
    out = {}
    for _, (member, lo, hi) in clampedRanges().items():
        if member:
            out.setdefault(member, (lo, hi))
    return out


def rangesByKey():
    """config key -> (lo, hi), for a caller that starts from the file."""
    return {key: (lo, hi) for key, (_, lo, hi) in clampedRanges().items()}
