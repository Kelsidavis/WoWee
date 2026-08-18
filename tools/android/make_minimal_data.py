#!/usr/bin/env python3
"""Cut an extracted Data directory down to something a phone can hold.

A full extraction is about 18 GB. The Android build cannot ask a player to copy
that onto a device, and does not need to: reaching the login screen and creating
a character touches a fraction of it.

This reads manifest.json, which is the client's index of everything it can open,
keeps the entries a profile allows, links or copies those files into a new
directory, and writes a manifest describing only what it kept. The client finds
its data root by looking for manifest.json, so a subset without one is not a
smaller install, it is a broken one.

Profiles live in data_profiles.json.

    ./make_minimal_data.py --source ~/Data --out ~/Data-login --profile login
    ./make_minimal_data.py --source ~/Data --out ~/Data-ek --profile world --maps azeroth
    ./make_minimal_data.py --source ~/Data --profile world --dry-run

--dry-run reports the size and touches nothing.
"""

import argparse
import json
import os
import re
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROFILES = HERE / "data_profiles.json"

# Entries are one per line, as manifest_writer.cpp emits them.
ENTRY_PATH = re.compile(r'"p":\s*"([^"]*)"')


def load_profiles():
    with open(PROFILES, encoding="utf-8") as handle:
        return json.load(handle)["profiles"]


_COMPILED = {}


def _compile(pattern):
    """Translate a glob to a regex where * stops at a separator and ** does not.

    fnmatch is not usable here: its * crosses /, so db/*.dbc would also match
    db/sub/spell.dbc. Nor is startswith on the text before a trailing /**, which
    matched nothing at all when the prefix itself held a wildcard.
    """
    out = []
    index = 0
    while index < len(pattern):
        char = pattern[index]
        if char == "*":
            if pattern[index:index + 2] == "**":
                out.append(".*")
                index += 2
                continue
            out.append("[^/]*")
        elif char == "?":
            out.append("[^/]")
        else:
            out.append(re.escape(char))
        index += 1
    return re.compile("^" + "".join(out) + "$")


def matches(path, patterns):
    """True if path matches any pattern. A trailing /** covers everything below."""
    for pattern in patterns:
        regex = _COMPILED.get(pattern)
        if regex is None:
            regex = _COMPILED[pattern] = _compile(pattern)
        if regex.match(path):
            return True
    return False


def build_filter(profile, maps):
    """Return a predicate over manifest 'p' paths.

    maps, when given, narrows terrain/maps to the named ones. It is applied
    after the profile, so a profile that excludes terrain entirely stays
    excluded.
    """
    include = profile.get("include", [])
    exclude = profile.get("exclude", [])
    wanted_maps = None
    if maps:
        wanted_maps = ["terrain/maps/%s/**" % m.strip().lower() for m in maps if m.strip()]

    def keep(path):
        if not matches(path, include):
            return False
        if matches(path, exclude):
            return False
        if wanted_maps is not None and path.startswith("terrain/maps/"):
            return matches(path, wanted_maps)
        return True

    return keep


def human(size):
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if size < 1024 or unit == "TB":
            return "%.1f %s" % (size, unit)
        size /= 1024.0


def place(src, dst, copy):
    """Hard link where the filesystem allows it, copy where it does not."""
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists():
        dst.unlink()
    if not copy:
        try:
            os.link(src, dst)
            return
        except OSError:
            pass
    shutil.copy2(src, dst)


def copy_repo_json(source, out, dry_run):
    """The expansion profiles and opcode tables are not in the manifest.

    They come from the repository rather than from anyone's MPQs, and the client
    reads them by path. A subset without them loads no expansion at all.
    """
    count = 0
    for rel in ("expansions", "opcodes"):
        root = source / rel
        if not root.is_dir():
            continue
        for path in root.rglob("*.json"):
            count += 1
            if not dry_run:
                place(path, out / path.relative_to(source), copy=False)
    return count


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", required=True,
                        help="an extracted Data directory, the one holding manifest.json")
    parser.add_argument("--out", help="where to write the subset (not needed with --dry-run)")
    parser.add_argument("--profile", default="login", help="a profile from data_profiles.json")
    parser.add_argument("--maps", help="comma separated map names to keep under terrain/maps")
    parser.add_argument("--copy", action="store_true",
                        help="copy instead of hard linking, for a different filesystem")
    parser.add_argument("--dry-run", action="store_true", help="report the size and stop")
    args = parser.parse_args()

    profiles = load_profiles()
    if args.profile not in profiles:
        print("no profile named %r. Known: %s" % (args.profile, ", ".join(sorted(profiles))))
        return 1
    profile = profiles[args.profile]

    source = Path(args.source).expanduser().resolve()
    manifest = source / "manifest.json"
    if not manifest.is_file():
        print("no manifest.json in %s - that is not an extracted data root." % source)
        return 1

    if not args.dry_run and not args.out:
        print("--out is required unless --dry-run is given.")
        return 1
    out = Path(args.out).expanduser().resolve() if args.out else None
    if out is not None and (out == source or source in out.parents):
        print("--out must not be inside --source.")
        return 1

    maps = args.maps.split(",") if args.maps else profile.get("default_maps")
    keep = build_filter(profile, maps)

    print("profile %s: %s" % (args.profile, profile.get("summary", "")))
    if maps:
        print("maps: %s" % ", ".join(maps))
    print("reading %s" % manifest)

    kept_lines = []
    seen = kept = missing = 0
    kept_bytes = 0
    total_bytes = 0

    with open(manifest, encoding="utf-8") as handle:
        for line in handle:
            found = ENTRY_PATH.search(line)
            if not found:
                continue
            seen += 1
            rel = found.group(1)
            src = source / rel
            try:
                size = src.stat().st_size
            except OSError:
                # The manifest lists it, the extraction did not produce it.
                missing += 1
                continue
            total_bytes += size
            if not keep(rel):
                continue
            kept += 1
            kept_bytes += size
            kept_lines.append((line.rstrip().rstrip(","), rel, src))

    if seen == 0:
        print("the manifest parsed to zero entries, which means the format moved, "
              "not that it is empty.")
        return 1

    print("\n%d of %d files, %s of %s (%.1f%%)"
          % (kept, seen, human(kept_bytes), human(total_bytes),
             100.0 * kept_bytes / total_bytes if total_bytes else 0.0))
    if missing:
        print("%d entries in the manifest have no file on disk and were skipped." % missing)

    if args.dry_run:
        return 0

    print("writing to %s" % out)
    out.mkdir(parents=True, exist_ok=True)
    for _, rel, src in kept_lines:
        place(src, out / rel, args.copy)

    with open(out / "manifest.json", "w", encoding="utf-8") as handle:
        handle.write("{\n")
        handle.write('  "version": 1,\n')
        handle.write('  "basePath": ".",\n')
        handle.write('  "fileCount": %d,\n' % len(kept_lines))
        handle.write('  "entries": {\n')
        for index, (text, _, _) in enumerate(kept_lines):
            handle.write(text)
            handle.write(",\n" if index + 1 < len(kept_lines) else "\n")
        handle.write("  }\n")
        handle.write("}\n")

    extra = copy_repo_json(source, out, dry_run=False)
    print("wrote manifest.json with %d entries, plus %d expansion and opcode files."
          % (len(kept_lines), extra))
    print("\nOn a device:\n  adb push %s/. /sdcard/Android/data/com.wowee.client/files/Data/" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
