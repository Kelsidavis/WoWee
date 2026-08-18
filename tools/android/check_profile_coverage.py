#!/usr/bin/env python3
"""Asset paths the client names in source that a data profile would drop.

make_minimal_data.py decides what to keep by directory. That is a guess about
what the client opens unless something checks it, and the guess has already been
wrong once: fonts live under misc/fonts, which no reasoning about "the login
screen needs interface art" would have reached.

This takes every asset path spelled out in the client's own source, resolves it
through the manifest the way the asset manager does, and reports the ones a
profile would leave behind. A path the client names and the profile drops is a
missing texture or a missing model at runtime.

    ./check_profile_coverage.py --source ~/Data --profile login

It reads literals, so it sees only paths that are written out in full. Anything
the client assembles at runtime from a DBC row is invisible here, which is why
this narrows the risk rather than removing it.
"""

import argparse
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
sys.path.insert(0, str(HERE))

from make_minimal_data import build_filter, load_profiles, matches, ENTRY_PATH  # noqa: E402

# A quoted literal holding at least one backslash, which is how the client
# spells a WoW asset path.
LITERAL = re.compile(r'"([A-Za-z][A-Za-z0-9_ .\-]*\\\\[^"]{2,120})"')

# The same shape in the interface's Lua and XML, where it is not always quoted
# and, by Blizzard's convention, usually carries no extension.
UI_LITERAL = re.compile(r'((?:Interface|Textures|Character|Item|Spell)\\{1,2}[A-Za-z0-9_ .\\-]{3,120})',
                        re.IGNORECASE)

# What an extensionless interface path turns out to be, in the order to try.
BARE_SUFFIXES = (".blp", ".m2", ".tga")

MANIFEST_KEY = re.compile(r'^\s*"((?:[^"\\]|\\.)*)":\s*\{')

# Extensions worth checking. A literal with a backslash and none of these is a
# format string, a registry key or a comment, not a file.
ASSET_SUFFIXES = (".m2", ".mdx", ".blp", ".wmo", ".dbc", ".ttf", ".mp3", ".wav",
                  ".ogg", ".adt", ".wdt", ".wdl", ".skin", ".anim", ".tga", ".png")


def load_manifest_keys(manifest):
    """Map each manifest key to its filesystem path."""
    keys = {}
    with open(manifest, encoding="utf-8") as handle:
        for line in handle:
            key = MANIFEST_KEY.search(line)
            path = ENTRY_PATH.search(line)
            if key and path:
                keys[key.group(1).replace("\\\\", "\\").lower()] = path.group(1)
    return keys


def source_literals():
    """Every asset-looking path literal in the client's own source."""
    found = set()
    for directory in ("src", "include"):
        for path in (ROOT / directory).rglob("*"):
            if path.suffix not in (".cpp", ".hpp", ".h", ".c"):
                continue
            try:
                text = path.read_text(encoding="utf-8", errors="ignore")
            except OSError:
                continue
            for match in LITERAL.finditer(text):
                literal = match.group(1).replace("\\\\", "\\").lower()
                if literal.endswith(ASSET_SUFFIXES):
                    found.add(literal)
    return found


def interface_literals(source):
    """Asset paths named by the interface's own Lua and XML.

    The login screen is built in GlueXML, not in C++, so a check that reads only
    src/ says nothing about the profile that exists to serve the login screen.
    These paths usually carry no extension, which is why they are resolved
    against the manifest rather than against the filesystem.
    """
    found = set()
    root = source / "interface"
    if not root.is_dir():
        return found
    for path in root.rglob("*"):
        if path.suffix.lower() not in (".lua", ".xml", ".toc"):
            continue
        try:
            text = path.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        for match in UI_LITERAL.finditer(text):
            literal = match.group(1).replace("\\\\", "\\").lower().rstrip(".\\ ")
            if literal:
                found.add(literal)
    return found


def resolve(literal, keys):
    """A manifest path for this literal, trying the extensions a bare path hides."""
    if literal in keys:
        return keys[literal]
    if "." not in literal.rsplit("\\", 1)[-1]:
        for suffix in BARE_SUFFIXES:
            if literal + suffix in keys:
                return keys[literal + suffix]
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", required=True, help="an extracted Data directory")
    parser.add_argument("--profile", default="login")
    parser.add_argument("--maps", help="comma separated map names, as make_minimal_data.py takes")
    args = parser.parse_args()

    profiles = load_profiles()
    if args.profile not in profiles:
        print("no profile named %r." % args.profile)
        return 1
    profile = profiles[args.profile]

    manifest = Path(args.source).expanduser().resolve() / "manifest.json"
    if not manifest.is_file():
        print("no manifest.json in %s." % args.source)
        return 1

    keys = load_manifest_keys(manifest)
    if not keys:
        print("the manifest parsed to zero entries, which means the format moved.")
        return 1

    literals = source_literals()
    if not literals:
        print("no asset path literals found in src/ - the pattern stopped matching, "
              "so this reports a clean run it did not perform.")
        return 1
    from_source = len(literals)
    literals |= interface_literals(Path(args.source).expanduser().resolve())
    from_interface = len(literals) - from_source

    maps = args.maps.split(",") if args.maps else profile.get("default_maps")
    keep = build_filter(profile, maps)

    # A path the profile's include list asked for and something else then took
    # away is a hole. A path the include list never asked for is the profile
    # doing its job. Asking the patterns directly beats guessing from the
    # top-level directory, which called a Stormwind doodad texture a hole
    # because the profile carries misc/fonts.
    include = profile.get("include", [])
    # An exclusion that drops something the client names is a hole unless the
    # profile says why it is not. Same shape as the reasons in
    # tools/settings_without_a_control.py, and for the same reason: a rule
    # nobody justified is indistinguishable from a rule nobody noticed.
    reasons = profile.get("exclude_reasons", {})

    holes, expected, deliberate, unknown, kept = [], {}, {}, [], 0
    for literal in sorted(literals):
        path = resolve(literal, keys)
        if path is None:
            unknown.append(literal)
            continue
        if keep(path):
            kept += 1
        elif matches(path, include):
            excused = next((pattern for pattern in reasons if matches(path, [pattern])), None)
            if excused:
                deliberate.setdefault(excused, 0)
                deliberate[excused] += 1
            else:
                holes.append((literal, path))
        else:
            expected.setdefault(path.split("/")[0], 0)
            expected[path.split("/")[0]] += 1

    print("%d asset paths named in src/ and %d more in the interface's Lua and XML; "
          "%d of them present in this extraction."
          % (from_source, from_interface, len(literals) - len(unknown)))
    print("profile %s keeps %d, drops %d.\n"
          % (args.profile, kept, len(holes) + sum(expected.values())))

    if expected:
        print("dropped from directories this profile does not carry, as intended:")
        for name in sorted(expected):
            print("  %-12s %d" % (name, expected[name]))
        print("")

    if deliberate:
        print("dropped by an exclusion the profile explains:")
        for pattern in sorted(deliberate):
            print("  %-28s %4d  %s" % (pattern, deliberate[pattern], reasons[pattern]))
        print("")

    if holes:
        print("%d path(s) the profile meant to carry and does not. Each is a texture "
              "or a model the client asks for by name and will not find:" % len(holes))
        for literal, path in holes:
            print("  MISSING  %s  ->  %s" % (literal, path))
    else:
        print("nothing the profile carries is missing a path the client names.")

    if unknown:
        print("\n%d literals are not in this extraction at all. That is expected for "
              "another expansion's assets, and a typo looks the same, so they are "
              "listed rather than counted." % len(unknown))
        for literal in unknown[:20]:
            print("  absent   %s" % literal)
        if len(unknown) > 20:
            print("  ... and %d more" % (len(unknown) - 20))

    return 1 if holes else 0


if __name__ == "__main__":
    sys.exit(main())
