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
REPO_ROOT = HERE.parent.parent

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

    maps, when given, narrows the world maps to the named ones. It is applied
    after the profile, so a profile that excludes the world entirely stays
    excluded.

    The maps live under world/maps/<name>/, which is where the client reads
    the terrain from and where an extraction puts it. The narrowing used to
    look under terrain/maps/, which is not a directory any extraction has - so
    it never matched anything and --maps did nothing at all, whatever it was
    told.
    """
    include = profile.get("include", [])
    exclude = profile.get("exclude", [])
    wanted_maps = None
    if maps and not any(m.strip().lower() == "all" for m in maps):
        wanted_maps = ["world/maps/%s/**" % m.strip().lower() for m in maps if m.strip()]

    def keep(path):
        if not matches(path, include):
            return False
        if matches(path, exclude):
            return False
        if wanted_maps is not None and path.startswith("world/maps/"):
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


# Exactly what `git ls-files Data/` lists, by name. A recursive glob for *.json
# also matches the overlay manifests a local extraction generates, and copying
# one of those describes thousands of files to the client that the subset does
# not contain: it indexes them, finds nothing, and the world renders with holes
# in it. Same list as the staging task in android/app/build.gradle.
#
# The expansion tables live in the expansion's own directory, which is where
# asset_extract puts them: the repository's Data/expansions/<id>/ for a
# checkout, or the same directory inside an extraction made with
# --expansion-subdir. They are looked up by name rather than by pattern,
# because the two layouts put them at different depths and a pattern that
# fits one of them matches nothing in the other.
EXPANSION_TABLES = [
    "dbc_layouts.json",
    "expansion.json",
    "opcodes.json",
    "update_fields.json",
]

OPCODE_TABLES = [
    "aliases.json",
    "canonical.json",
]


def copy_expansion_tables(exp_dir, expansion, out, dry_run):
    """The expansion's own tables, which are not in the manifest.

    They come from the repository rather than from anyone's MPQs, and the
    client reads them by path. A subset without them loads no expansion at
    all, so they are written where the client looks for them:
    out/expansions/<id>/.

    An extraction made with --expansion-subdir has them nowhere, because they
    were never part of the MPQs; the repository's Data/expansions/<id>/ holds
    them, and that is the second place looked in.
    """
    count = 0
    dest_dir = out / "expansions" / expansion
    repo_dir = REPO_ROOT / "Data" / "expansions" / expansion
    for name in EXPANSION_TABLES:
        path = exp_dir / name
        if not path.is_file():
            path = repo_dir / name
        if not path.is_file():
            continue
        count += 1
        if not dry_run:
            place(path, dest_dir / name, copy=False)
    return count


def copy_opcode_tables(data_root, out, dry_run):
    """The opcode tables, which sit beside the expansions rather than in one.

    The client reads the expansion's own opcodes.json; these two are the
    cross-expansion naming tables, and the repository's Data/opcodes/ holds
    them rather than any extraction.
    """
    count = 0
    dest_dir = out / "opcodes"
    for name in OPCODE_TABLES:
        path = data_root / "opcodes" / name
        if not path.is_file():
            path = REPO_ROOT / "Data" / "opcodes" / name
        if not path.is_file():
            continue
        count += 1
        if not dry_run:
            place(path, dest_dir / name, copy=False)
    return count



# The executable the extractor caches for Warden, and the DLLs the login
# integrity hash reads beside it. They are not manifest entries, so no profile
# keeps them, and a phone without them answers every Warden memory check as
# unmapped and cannot compute the integrity hash - a server that checks either
# sees a client that is not the real one.
#
# The extractor caches them into the expansion's own misc/ directory, which is
# where they are looked for here.
def copy_integrity_files(exp_dir, expansion, out, dry_run):
    count = 0
    misc = exp_dir / "misc"
    if not misc.is_dir():
        return 0
    dest_dir = out / "expansions" / expansion / "misc"
    for path in sorted(misc.iterdir()):
        if not path.is_file():
            continue
        if path.suffix.lower() not in (".exe", ".dll"):
            continue
        count += 1
        if not dry_run:
            place(path, dest_dir / path.name, copy=False)
    return count


def resolve_source(source):
    """Find the data root and the expansion directory in whatever was given.

    An extraction is a directory with a manifest.json in it, and that directory
    is the data root - the manifest's paths are relative to it. Two shapes are
    in the wild and both are accepted:

      <expansion>/            a directory holding manifest.json, the expansion's
                               own tables beside it, and misc/ with the
                               executables the extractor cached.
      <root>/expansions/<id>/ the same, one level down under a Data root
                               that also holds the opcode tables.

    Returns (data_root, expansion_dir, expansion_id).
    """
    if (source / "manifest.json").is_file():
        data_root = source
    else:
        data_root = None
        for candidate in sorted(source.glob("expansions/*/manifest.json")):
            data_root = candidate.parent
            break
        if data_root is None:
            return None, None, None

    expansion = ""
    try:
        with open(data_root / "manifest.json", encoding="utf-8") as handle:
            head = handle.read(4096)
        found = re.search(r'"expansion"\s*:\s*"([^"]*)"', head)
        if found:
            expansion = found.group(1)
    except OSError:
        pass

    if not expansion:
        # Fall back to the name of the directory the manifest sits in.
        expansion = data_root.name
        if expansion in ("expansions", "Data", "data"):
            expansion = ""

    # The expansion's own tables sit beside its manifest in an extraction, and
    # under Data/expansions/<id>/ in a checkout. Either way, the directory
    # that holds the manifest is the one that has them.
    return data_root, data_root, expansion


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", required=True,
                        help="an extracted Data directory, the one holding manifest.json")
    parser.add_argument("--out", help="where to write the subset (not needed with --dry-run)")
    parser.add_argument("--profile", default="login", help="a profile from data_profiles.json")
    parser.add_argument("--maps",
                        help="comma separated map names to keep under world/maps, "
                             "or 'all' for every map the extraction has")
    parser.add_argument("--copy", action="store_true",
                        help="copy instead of hard linking, for a different filesystem")
    parser.add_argument("--with-sound", action="store_true",
                        help="keep the game's sound effects (sound/, about 6 GB). "
                             "The profiles drop them: the client runs without them "
                             "(the audio coordinator warns and the game plays on), "
                             "and the music is the project's own soundtrack, which "
                             "is not in this tree at all.")
    parser.add_argument("--dry-run", action="store_true", help="report the size and stop")
    args = parser.parse_args()

    profiles = load_profiles()
    if args.profile not in profiles:
        print("no profile named %r. Known: %s" % (args.profile, ", ".join(sorted(profiles))))
        return 1
    profile = dict(profiles[args.profile])
    if args.with_sound and "sound/**" not in profile["include"]:
        # Added to the profile rather than to the profile file, so the shared
        # profiles keep the sizes they were measured at and this stays an
        # opt-in on the command line.
        profile["include"] = list(profile["include"]) + ["sound/**"]

    source = Path(args.source).expanduser().resolve()
    data_root, exp_dir, expansion = resolve_source(source)
    if data_root is None:
        print("no manifest.json in %s, and no expansions/*/manifest.json under it - "
              "that is not an extracted data root." % source)
        return 1
    manifest = data_root / "manifest.json"

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
            src = data_root / rel
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

    extra = copy_expansion_tables(exp_dir, expansion, out, dry_run=False)
    extra += copy_opcode_tables(source, out, dry_run=False)
    print("wrote manifest.json with %d entries, plus %d expansion and opcode files."
          % (len(kept_lines), extra))
    integrity = copy_integrity_files(exp_dir, expansion, out, dry_run=False)
    print("copied %d client executable and DLL files for Warden and the login hash."
          % integrity)
    if integrity == 0:
        print("no misc/WoW.exe in the source: a server that runs Warden "
              "will see failed memory checks from this device.")
    print("\nOn an Android device, after opening the app once:\n"
          "  adb push %s/. /sdcard/Android/data/com.wowee.client/files/Data/\n"
          "  adb shell chmod -R a+rwX /sdcard/Android/data/com.wowee.client/files/Data" % out)
    print("\nOn an iPad, copy the contents of %s into the WoWee app's Documents\n"
          "folder (Files > On My iPad > WoWee), replacing the Data folder there."
          % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
