#!/usr/bin/env python3
"""One shader function, compiled into several shaders, drifting apart.

    tools/shader_function_check.py

WHY

GLSL has no linker here. Every shader is compiled on its own, so a function
two of them need is written into both, and nothing connects the copies. Seven
bodies are duplicated across the seventy-eight shaders in this tree:

    parallaxOcclusionMap    29 lines, in character.frag and wmo.frag
    localLightContribution  14 lines, in character.frag, m2.frag, terrain.frag
                            and wmo.frag
    sampleShadowPCF          8 lines, in four of them
    safeNormalize            6 lines, computeLodFactor 5, fallbackTangent 3

A copy that drifts does not fail to compile and does not raise. It shades one
kind of surface differently from the others - a character lit slightly unlike
the terrain under them, or a shadow softer on a wall than on the ground - and
that reads as an art problem rather than a code one.

WHAT IT LOOKS FOR

Every named function defined in more than one shader, and whether the copies
have the same body once comments and whitespace are normalised. Not whether a
function is duplicated - it has to be, without a linker - but whether the
duplicates still agree.

This is deliberately a guard rather than a fix. Sharing the bodies for real
means an include directive, a new file extension so the glob does not compile
the shared file as a shader, dependency edges so a change to it recompiles its
dependents, and regenerating every affected .spv - and those .spv files are
tracked, because they are the fallback when glslc is absent. That is worth
doing deliberately, not as a side effect of noticing.

WHAT IT CANNOT SEE

A function renamed in one shader, and two functions that do the same thing
under different names.
"""
import collections
import difflib
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent

# Copies read and judged as the same function, keyed by name.
SETTLED = {
    # wmo.frag hoists the normalised light vector into a local instead of
    # dividing twice. Same arithmetic, one line longer.
    "localLightContribution",
}


def functions(path):
    text = re.sub(r"//[^\n]*", "", path.read_text(errors="ignore"))
    out = []
    for m in re.finditer(r"^\s*(?:\w[\w\s]*?)\s+(\w+)\s*\([^;{]*\)\s*\{(?:\.\w+\s*=\s*)?", text, re.M):
        depth, j, started = 0, m.start(), False
        while j < len(text):
            if text[j] == "{":
                depth += 1
                started = True
            elif text[j] == "}":
                depth -= 1
                if started and depth == 0:
                    j += 1
                    break
            j += 1
        body = [re.sub(r"\s+", " ", line).strip()
                for line in text[m.start():j].split("\n")[1:-1]]
        out.append((m.group(1), tuple(line for line in body if line)))
    return out


def main() -> int:
    shaders = sorted((ROOT / "assets" / "shaders").glob("*.glsl"))
    if not shaders:
        print("No shaders found; the zero below would mean the scan broke.")
        return 1

    by_name = collections.defaultdict(list)
    for path in shaders:
        for name, body in functions(path):
            # main is a different function in every shader by definition.
            if name == "main" or len(body) < 3:
                continue
            by_name[name].append((path.name, body))

    shared = {n: v for n, v in by_name.items() if len({f for f, _ in v}) > 1}
    drifted = []
    for name, copies in sorted(shared.items()):
        if len({body for _, body in copies}) == 1:
            continue
        if name in SETTLED:
            continue
        drifted.append((name, copies))

    print(f"{len(shaders)} shaders, {len(shared)} function(s) compiled into more than one")
    print(f"{len(SETTLED)} settled as the same function written two ways\n")
    print(f"{len(drifted)} whose copies no longer agree:")
    if not drifted:
        print("  (none)")
    for name, copies in drifted:
        variants = collections.defaultdict(list)
        for f, body in copies:
            variants[body].append(f)
        print(f"  {name}: {len(copies)} copies, {len(variants)} different")
        ordered = list(variants.items())
        for body, files in ordered:
            print(f"      {len(body):2d} lines  {', '.join(files)}")
        for line in list(difflib.unified_diff(list(ordered[0][0]), list(ordered[1][0]),
                                              n=0, lineterm=""))[2:10]:
            print(f"        {line[:100]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
