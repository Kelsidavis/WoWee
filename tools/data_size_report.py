#!/usr/bin/env python3
"""How the extracted game data divides up, by category.

The numbers that decide what fits on a tablet come from here: the Data folder
is what the player copies to the device, and most of it is one or two
categories the client barely reads.

    tools/data_size_report.py ~/WoWee/Data/expansions/wotlk
    tools/data_size_report.py ~/WoWee/Data/expansions/wotlk --top 25
    tools/data_size_report.py ~/WoWee/Data/expansions/wotlk --by-ext
"""

import argparse
import os
import sys
from collections import defaultdict


# The categories that matter, in the order the client's own code reads them.
# The key is the first path segment of the extracted tree.
CATEGORIES = {
    "interface": "Interface art (icons, frames, the game's own UI)",
    "world": "World models (WMOs, and the models that reference them)",
    "creature": "Creature models and their skins",
    "character": "Character models (the player's own body parts)",
    "item": "Item models and icons",
    "spell": "Spell visuals",
    "terrain": "Terrain (ADT tiles, minimaps, terrain textures)",
    "environment": "Environment textures and effects",
    "misc": "Fonts, tilesets, dungeons, x-textures",
    "sound": "Sound effects and music",
    "dbfilesclient": "DBC tables (the game's data tables)",
    "db": "DBC tables in CSV form",
    "interface": "Interface art",
}

# Video and music live in their own archives and are worth naming separately,
# because they are the two things a player is most willing to lose.
VIDEO_EXT = {".avi", ".mpg", ".mpeg", ".ogv", ".wmv", ".mp4", ".mov", ".wma", ".wav"}


def walk(root):
    """Yield (relative path, size) for every file under root."""
    for dirpath, _dirnames, filenames in os.walk(root):
        for name in filenames:
            full = os.path.join(dirpath, name)
            try:
                size = os.path.getsize(full)
            except OSError:
                continue
            yield os.path.relpath(full, root), size


def category_of(relpath):
    parts = relpath.replace("\\", "/").split("/")
    return parts[0] if parts else "(root)"


def human(size):
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if size < 1024 or unit == "TB":
            return "%.1f %s" % (size, unit)
        size /= 1024.0


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("root", help="the extracted data directory")
    parser.add_argument("--top", type=int, default=20,
                        help="how many of the largest directories to list")
    parser.add_argument("--by-ext", action="store_true",
                        help="break down by file extension instead of directory")
    args = parser.parse_args()

    root = os.path.abspath(os.path.expanduser(args.root))
    if not os.path.isdir(root):
        print("not a directory: %s" % root)
        return 1

    by_cat = defaultdict(int)
    by_cat_files = defaultdict(int)
    by_ext = defaultdict(int)
    by_ext_files = defaultdict(int)
    by_dir = defaultdict(int)
    total = 0
    count = 0
    video_bytes = 0

    for rel, size in walk(root):
        total += size
        count += 1
        cat = category_of(rel)
        by_cat[cat] += size
        by_cat_files[cat] += 1
        by_dir[os.path.dirname(rel)] += size
        ext = os.path.splitext(rel)[1].lower() or "(none)"
        by_ext[ext] += size
        by_ext_files[ext] += 1
        if ext in VIDEO_EXT:
            video_bytes += size

    print("Root: %s" % root)
    print("%d files, %s" % (count, human(total)))
    print()

    if args.by_ext:
        print("%-12s %10s %8s  %s" % ("extension", "size", "files", "share"))
        for ext, size in sorted(by_ext.items(), key=lambda kv: -kv[1])[:30]:
            print("%-12s %10s %8d  %5.1f%%"
                  % (ext, human(size), by_ext_files[ext], 100.0 * size / total))
        return 0

    print("%-16s %10s %8s  %s" % ("category", "size", "files", "what it is"))
    for cat, size in sorted(by_cat.items(), key=lambda kv: -kv[1]):
        note = CATEGORIES.get(cat, "")
        print("%-16s %10s %8d  %s" % (cat, human(size), by_cat_files[cat], note))

    print()
    print("%-16s %10s" % ("video+audio", human(video_bytes)))

    print()
    print("Largest directories:")
    for d, size in sorted(by_dir.items(), key=lambda kv: -kv[1])[:args.top]:
        if size == 0:
            continue
        print("  %10s  %s" % (human(size), d or "(root)"))

    return 0


if __name__ == "__main__":
    sys.exit(main())
