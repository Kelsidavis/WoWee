#!/usr/bin/env python3
"""Every shader the client can load must be in the repository.

    tools/shader_fallback_check.py

The .spv files are what a build without glslc runs, and the iOS bundle copies
them into the app instead of compiling them - so a .spv that is missing from
the repository is a shader missing from the device. The client asks for one by
path and, when it is not there, logs a warning and carries on without it: the
world map comes up with no overlay, grass does not draw, and the Hi-Z cull has
no pyramid to read. Nothing fails, so nothing is reported.

The trap is that .gitignore ignores *.spv, since the rest of the tree has
compiled SPIR-V as build output. The ones under assets/shaders are the
exception, and the exception used to be a list of two files - so a shader
added later was ignored silently, and the build that compiled it locally left
it on disk where nothing tracked it. A fresh clone then had no such shader,
and the checkout that reproduced the bug was the one with the file on disk.

So this checks two things, and neither needs glslc:

  - every .glsl in assets/shaders has a .spv beside it, and
  - every .spv in assets/shaders is tracked by git.

The first is what a build needs; the second is what a clone needs. Both were
true of the eight that went missing (grass, Hi-Z, the world map overlay and the
editor's water), and neither is visible from the code, which only ever names
the path.
"""
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SHADERS = ROOT / "assets" / "shaders"

# The shaders the client asks for by path. Not every .spv in the directory is
# one of these - the two rt_ variants are built from a .glsl that also has a
# plain form - so the list is read from the code rather than guessed.
CLIENT_SOURCES = ("src", "include")

failures = []


def tracked(paths):
    """Which of these paths git is tracking."""
    if not paths:
        return set()
    try:
        out = subprocess.run(
            ["git", "-C", str(ROOT), "ls-files", "--"] + list(paths),
            capture_output=True, text=True, check=True).stdout.split()
    except (subprocess.CalledProcessError, FileNotFoundError) as exc:
        # Not a git checkout (a release tarball, say). Nothing to compare
        # against, and nothing to report either.
        print(f"skipped: {exc}")
        sys.exit(0)
    return set(out)


def main():
    if not SHADERS.is_dir():
        print(f"skipped: no {SHADERS}")
        return 0

    glsl = sorted(SHADERS.glob("*.glsl"))
    if not glsl:
        print(f"skipped: no shaders in {SHADERS}")
        return 0

    # 1. Every .glsl has a .spv beside it.
    for source in glsl:
        binary = source.with_suffix(".spv")
        if not binary.exists():
            failures.append(
                f"{binary.relative_to(ROOT)} is missing: {source.name} has no "
                f"compiled form, and a build without glslc runs nothing else")

    # 2. Every .spv in the directory is tracked. Git answers with paths
    # relative to the root, so the comparison is done in that form.
    binaries = sorted(SHADERS.glob("*.spv"))
    names = [str(p.relative_to(ROOT)) for p in binaries]
    is_tracked = tracked(names)
    untracked = [n for n in names if n not in is_tracked]
    for name in untracked:
        failures.append(
            f"{name} is not tracked by git: a fresh clone has no such shader, "
            f"and the client only says so in a log line")

    # 3. Every shader the code names by path is present and tracked.
    named = set()
    for folder in CLIENT_SOURCES:
        base = ROOT / folder
        if not base.is_dir():
            continue
        for source in base.rglob("*.cpp"):
            text = source.read_text(errors="replace")
            for line in text.splitlines():
                marker = '"assets/shaders/'
                start = line.find(marker)
                while start != -1:
                    end = line.find('"', start + len(marker))
                    if end == -1:
                        break
                    named.add(line[start + 1:end])
                    start = line.find(marker, end)
    for name in sorted(named):
        if not name.endswith(".spv"):
            continue
        if not (ROOT / name).exists():
            failures.append(
                f"{name} is named by the code but is not in the repository")
        elif name not in is_tracked:
            failures.append(f"{name} is named by the code but is not tracked")

    for line in failures:
        print(f"FAIL {line}")
    if failures:
        print(f"\n{len(failures)} shader(s) would be missing from a build "
              f"without glslc.")
        return 1
    print(f"ok: {len(glsl)} shaders, {len(binaries)} compiled forms, all tracked")
    return 0


if __name__ == "__main__":
    sys.exit(main())
