#!/usr/bin/env python3
"""One table of values, written out in more than one file.

    tools/duplicate_table_check.py

WHY

The block scan reads twelve consecutive code lines and the similarity scan
reads whole functions. A table is neither: it is a braced initialiser, often
four or five lines, sitting inside a function that is otherwise unlike its
twin. Both scans reported zero while eight reputation standings were written
out in four places and the ten melee fallback chains in two.

That is the shape this repository fails at most - one fact in several places,
each copy plausible on its own. A table is the purest form of it: the values
*are* the knowledge, and nothing about a second copy looks wrong until the day
someone edits one of them.

Found on its first run, with the other two scans at zero:

    melee attack fallback chains    2 places (controller and capability probe)
    reputation standing names       4, two of which also had the thresholds
    resistance school names         2
    loot method names               2
    "play any attack this model has" 2 more, a fifth copy of the first subject

WHAT IT LOOKS FOR

Braced initialisers of four or more comma-separated items whose contents match
exactly, in more than one file. At least three of the items must be distinct:
without that the report is drowned by `{0, 0, 0, 0}` and by Vulkan struct
initialisers, which repeat because the API is shaped that way and carry no
knowledge to drift.

A second pass compares tables of strings with the spelling taken out -
case, spaces and punctuation dropped - so a set that drifted apart is
still reported. That is how the four instance difficulties were found:
"25 Normal" in the shared name, "25-Man" in the calendar and the lockout
table, "25-Man Normal" in chat, one value named three ways.

WHAT IT CANNOT SEE

A table that was reordered or renamed, or one whose values were reworded
rather than respelled. `repeated_literal_check.py` is the angle for that.

Nor a table split across a struct and its initialiser, or built at runtime.
"""
import collections
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
MIN_ITEMS = 4
MIN_DISTINCT = 3

DECL = re.compile(r"(\w+)\s*(?:\[\s*\w*\s*\])?\s*=\s*\{(.*?)\};", re.S)

# Tables read and judged as deliberately separate, keyed by their contents'
# first item and the names they appear under.
SETTLED = {
    # Byte signatures Warden scans memory for. They are the same bytes because
    # they match the same function in the same binary, and each site reads a
    # different span of it. Merging them would tie two scans to one span.
    "warden byte patterns": ("p2", "kWardenMemoryReadPattern", "kWardenMemcpyPattern"),
    # The eight corners of a box, enumerated. There is one way to write that
    # and it is shorter than any name for it.
    "AABB corners": ("corners",),
    # A full-screen quad's vertices and its two vertex attributes. Vulkan
    # boilerplate: identical because the API's shape is, with nothing to drift.
    "fullscreen quad": ("quadVerts", "attrs", "clearColor"),
    # One-pixel fallback textures - opaque white, and a flat normal. The values
    # are what the words mean; a shared constant would be a second name for
    # 255,255,255,255.
    "fallback pixels": ("white", "whitePixel", "flatNormal", "flatNormalPixel"),
    # Vulkan image subresources and clear colours. `{ASPECT_COLOR, 0, 1, 0, 1}`
    # is "the whole of mip zero, layer zero" and there is one way to write it;
    # these repeat because the API's shape does, and there is nothing in them
    # that can drift away from anything.
    # The client executables to look for. The auth handler needs the name to
    # report which binary a realm thinks it is talking to; Warden needs it to
    # find the module to scan. Same four names because it is the same client,
    # and sharing them would tie the login path to the anti-cheat one.
    "client executable names": ("candidateExes",),
    "vulkan subresources": ("subresourceRange", "imageSubresource",
                            "srcSubresource", "dstSubresource", "color",
                            "range"),
}


def settled_names():
    out = set()
    for names in SETTLED.values():
        out |= set(names)
    return out


def tables():
    found = collections.defaultdict(list)
    for base in ("src", "include"):
        root = ROOT / base
        if not root.is_dir():
            continue
        for path in sorted(root.rglob("*")):
            if path.suffix not in (".cpp", ".hpp", ".h"):
                continue
            text = re.sub(r"//[^\n]*", "", path.read_text(errors="ignore"))
            for m in DECL.finditer(text):
                items = [i.strip() for i in m.group(2).split(",") if i.strip()]
                if len(items) < MIN_ITEMS:
                    continue
                if any(len(i) > 60 for i in items):
                    continue
                if len(set(items)) < MIN_DISTINCT:
                    continue
                found["|".join(items)].append((str(path.relative_to(ROOT)), m.group(1)))
    return found


def respelled_key(contents):
    """The same table with its spelling taken out, or None if it is not one.

    Strings only. Numbers respell into each other far too easily - 0x10 and
    16 and 16.0f are one key under any normalisation worth the name - and a
    table of them carries no wording to drift.
    """
    items = contents.split("|")
    if not all(i.startswith('"') and i.endswith('"') for i in items):
        return None
    plain = [re.sub(r"[^a-z0-9]", "", i.lower()) for i in items]
    if any(not p for p in plain):
        return None
    return "|".join(plain)


def main() -> int:
    found = tables()
    if not found:
        print("Found no tables at all, which cannot be right - the scan broke.")
        return 1

    skip = settled_names()
    dupes = []
    for contents, sites in found.items():
        if len({f for f, _ in sites}) < 2:
            continue
        if all(name in skip for _, name in sites):
            continue
        dupes.append((len(contents.split("|")), contents, sites))
    dupes.sort(reverse=True)

    # The same table under different spellings. Grouped after the exact pass
    # so a group that is already reported there is not reported twice.
    respelled = collections.defaultdict(dict)
    for contents, sites in found.items():
        key = respelled_key(contents)
        if key is None:
            continue
        respelled[key].setdefault(contents, []).extend(sites)
    drifted = []
    for key, spellings in respelled.items():
        if len(spellings) < 2:
            continue
        sites = [site for group in spellings.values() for site in group]
        if all(name in skip for _, name in sites):
            continue
        drifted.append((len(key.split("|")), spellings))
    drifted.sort(reverse=True)

    print(f"{len(found)} tables of {MIN_ITEMS}+ items with {MIN_DISTINCT}+ distinct values")
    print(f"{len(SETTLED)} group(s) settled and not reported\n")
    print(f"{len(dupes)} written out in more than one file:")
    if not dupes:
        print("  (none)")
    for count, contents, sites in dupes:
        # The unsettled names first, and marked. A group is reported when any
        # one of its names is unjudged, and printing the first four sites hid
        # which that was: a dozen settled `subresourceRange` around a single
        # `range` read as a group that had already been judged and had not.
        ordered = sorted(sites, key=lambda site: site[1] in skip)
        where = ", ".join(f"{name}{'' if name not in skip else ' [settled]'} ({f})"
                          for f, name in ordered[:4])
        more = "" if len(sites) <= 4 else f", +{len(sites) - 4} more"
        print(f"  {count:3d} items  {where}{more}")
        print(f"            {contents[:100]}")

    print(f"\n{len(drifted)} written out more than once with different spellings:")
    if not drifted:
        print("  (none)")
    for count, spellings in drifted:
        print(f"  {count:3d} items")
        for contents, sites in sorted(spellings.items()):
            where = ", ".join(f"{name} ({f})" for f, name in sorted(sites)[:3])
            more = "" if len(sites) <= 3 else f", +{len(sites) - 3} more"
            print(f"            {contents[:80]}")
            print(f"              {where}{more}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
