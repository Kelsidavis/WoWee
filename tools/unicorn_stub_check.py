#!/usr/bin/env python3
"""Compile the !HAVE_UNICORN branch of the Warden emulator.

    tools/unicorn_stub_check.py [build-dir]

WHY THIS EXISTS

Unicorn Engine is optional. When it is missing, warden_emulator.cpp and
warden_module.cpp compile a stub branch instead - constructors, and one
short definition per method that answers false or zero.

Nothing builds that branch. Every CI leg installs libunicorn: apt on
Ubuntu, brew on macOS, mingw on Windows. So does every developer who
followed the setup instructions. The branch is written once, compiles
nowhere, and rots in place with no test failing and no warning printed.

It rotted. Issue #115: the stub defined WardenEmulator::getRegister,
which the header does not declare and nothing calls, so every build
without Unicorn stopped with "out-of-line definition does not match any
declaration". It was reported from outside, because inside the project
there was no configuration that could see it.

HOW

Take the real compile command for each translation unit that has
-DHAVE_UNICORN, drop the define, and ask the compiler to parse it.

The precompiled header has to go too. It was built with the define, so
leaving it in feeds the compiler the state it would have had and the
mismatch never surfaces - the first attempt at reproducing #115 passed
for exactly that reason.

WHAT IT CANNOT SEE

Whether the stubs are honest. Each one returns false or zero, and that
is a claim about what the client does without an emulator rather than
something a compiler can check.

Nor a break that only a different compiler reports. This runs whichever
one built the tree; #115 was found by GCC and reproduced here by clang,
but that is not guaranteed for the next one.
"""
import json
import pathlib
import shlex
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
DEFINE = "HAVE_UNICORN"


def compile_db(build_dir):
    """The compile_commands.json for this tree, or None with a reason."""
    candidates = []
    if build_dir:
        candidates.append(pathlib.Path(build_dir) / "compile_commands.json")
    candidates.append(ROOT / "build" / "compile_commands.json")
    for path in candidates:
        if path.is_file():
            return path, None
    return None, "no compile_commands.json in " + ", ".join(
        str(p.parent) for p in candidates)


def without_unicorn(entry):
    """The entry's command, minus the define, the PCH, and the output file."""
    args = shlex.split(entry.get("command") or " ".join(entry["arguments"]))
    out, i = [], 0
    while i < len(args):
        arg = args[i]
        if arg in ("-include-pch", "-o"):
            i += 2
            continue
        if arg == "-Xclang" and i + 1 < len(args) and args[i + 1] == "-include-pch":
            i += 4  # -Xclang -include-pch -Xclang <file>
            continue
        if arg == "-Winvalid-pch":
            i += 1
            continue
        if arg.startswith("-D" + DEFINE):
            i += 1
            continue
        if arg == "-c":
            out.append("-fsyntax-only")
            i += 1
            continue
        out.append(arg)
        i += 1
    return out


def main():
    build_dir = sys.argv[1] if len(sys.argv) > 1 else None
    path, why = compile_db(build_dir)
    if path is None:
        print("unicorn_stub_check: " + why)
        print("  Configure with cmake first; CMAKE_EXPORT_COMPILE_COMMANDS is")
        print("  on unconditionally, so a configured tree always has one.")
        return 1

    # The define is set on the whole wowee target, so it is on the command
    # line of every translation unit in it. Only the two that name it have a
    # branch to compile; checking the other 413 costs minutes and proves
    # nothing.
    entries, seen = [], set()
    for entry in json.load(open(path)):
        command = entry.get("command") or " ".join(entry["arguments"])
        if "-D" + DEFINE not in command:
            continue
        source = pathlib.Path(entry["file"])
        if source in seen or not source.is_file():
            continue
        if DEFINE not in source.read_text(errors="ignore"):
            continue
        seen.add(source)
        entries.append(entry)

    if not entries:
        # Unicorn was not found when this tree was configured, so the stub
        # branch is what the build already compiles. Nothing left to check.
        print("unicorn_stub_check: this tree builds without Unicorn already")
        return 0

    failed = []
    for entry in entries:
        name = pathlib.Path(entry["file"]).name
        # errors="replace": a compiler is free to quote a source line back at
        # us, and the Warden module tables hold raw bytes.
        result = subprocess.run(without_unicorn(entry), capture_output=True,
                                encoding="utf-8", errors="replace")
        if result.returncode != 0:
            failed.append((name, result.stderr.strip()))
        else:
            print("  ok  " + name)

    if failed:
        print()
        print("The Warden stubs do not compile without Unicorn:")
        for name, stderr in failed:
            print()
            print("--- " + name + " ---")
            print(stderr)
        print()
        print("These are the definitions under #else // !HAVE_UNICORN. They")
        print("have to match the header the same way the real ones do.")
        return 1

    print("unicorn_stub_check: %d translation unit(s) compile without Unicorn"
          % len(entries))
    return 0


if __name__ == "__main__":
    sys.exit(main())
