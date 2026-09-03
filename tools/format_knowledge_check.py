#!/usr/bin/env python3
"""File-format knowledge that has to live in one place, and where it does not.

    tools/format_knowledge_check.py

WHY

The duplication that costs most in this tree is not a repeated block - that is
what tools/duplicate_block_check.py measures, and it is clean. It is the same
knowledge about a file format written twice in different words, which that
scan says outright it cannot see.

It has happened three times in the ADT reader alone. The header beside
decodeLayerAlpha exists because two copies of the MCAL decode had already
grown; a third was still in the mesh builder, and it was the copy that reached
the screen - so the fix that fills a four-bit alpha map's unpainted last row
and column went into the decoder the grass uses while the ground went on
drawing the strip. "Which chunk is under this point" had four copies, one of
which had no check on its answer and read the area under the player from a
chunk in the next zone.

WHAT IT DOES

Each entry below names a piece of format knowledge, the file that owns it, and
a pattern that only that knowledge produces. A hit anywhere else is a second
copy - or a new caller that should be going through the owner.

This is deliberately narrow. It knows about the decodes that have actually
been duplicated here, not about duplication in general, and it says nothing
about code it has not been told to look for.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent

# Files judged once and left alone: the pattern is there and the knowledge is
# not the same. Without this the same settled hits come back to every scan.
SETTLED = {
    ("the MCAL four-bit unpack", "src/pipeline/blp_loader.cpp"):
        "BLP's own four-bit alpha channel. The arithmetic is the same because "
        "a nibble scales to a byte the same way anywhere; the data is a "
        "texture's alpha and not a terrain chunk's blend, it is interleaved "
        "per pixel rather than run through a map, and the two formats are free "
        "to change without each other.",
}

# (what it is, the file that owns it, a regex only that knowledge writes)
RULES = [
    ("the MCAL four-bit unpack",
     "src/pipeline/adt_alpha.cpp",
     r"0x0F\)\s*\*\s*17|>>\s*4\)\s*\*\s*17"),
    ("the MCAL run-length decode",
     "src/pipeline/adt_alpha.cpp",
     r"0x80\)\s*!=\s*0.*\n.*0x7F\)\s*\+\s*1"),
    ("a point's texel in a chunk's alpha map",
     "src/pipeline/adt_alpha.cpp",
     r"\*\s*63\.0f|ALPHA_MAP_DIM\s*-\s*1\)\s*;?\s*$"),
    ("which chunk is under a point",
     "src/rendering/terrain_manager.cpp",
     r"(maxX|maxY)\s*-\s*gl[XY]\)\s*/\s*CHUNK_SIZE"),
]


def main():
    sources = sorted(list((ROOT / "src").rglob("*.cpp")) +
                     list((ROOT / "include").rglob("*.hpp")))
    findings = []
    settled = set()
    for what, owner, pattern in RULES:
        rx = re.compile(pattern, re.MULTILINE)
        for path in sources:
            rel = str(path.relative_to(ROOT))
            if rel == owner:
                continue
            try:
                text = path.read_text(errors="ignore")
            except OSError:
                continue
            if (what, rel) in SETTLED:
                settled.add((what, rel))
                continue
            for m in rx.finditer(text):
                line = text.count("\n", 0, m.start()) + 1
                findings.append((what, owner, rel, line))

    print(f"{len(RULES)} piece(s) of format knowledge, "
          f"{len(sources)} source file(s)")
    print(f"{len(settled)} file(s) judged and not reported")
    if not findings:
        print("\nEach lives in one place.")
        return 0

    print(f"\n{len(findings)} copy or copies outside the file that owns it:")
    for what, owner, rel, line in findings:
        print(f"  {rel}:{line}")
        print(f"      {what} - owned by {owner}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
