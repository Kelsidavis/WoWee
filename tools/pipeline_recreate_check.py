#!/usr/bin/env python3
"""Renderers that rebuild a different pipeline than they first built.

Every renderer that owns a Vulkan pipeline builds it twice: once in
initialize, and again in recreatePipelines when the swapchain or the sample
count changes. The two have to describe the same pipeline, and nothing but
reading both makes that true.

Celestial did not. initialize asked for no depth test and recreatePipelines
asked for one, so the sun and the moon drew over everything until the first
swapchain rebuild and were occluded afterwards - a difference that appears on
a resolution change or an alt-tab, long after anyone was looking at the code
that caused it.

Compares the PipelineBuilder chain in initialize against the one in
recreatePipelines, with comments stripped: a trailing comment on one of the
two is not a difference, and reading it as one is how this sweep's first
version reported three renderers that were fine.

A renderer whose initialize calls a shared builder rather than writing the
chain inline is not checked, and does not need to be: that is the fix for this
bug class, and Celestial is built that way now. Four renderers still describe
the pipeline twice and those are what this reads.

Only the first chain in each function is read. A renderer that builds several
pipelines in one function - the WMO renderer's opaque, transparent and
wireframe passes, or the M2 renderer's particle, smoke and ribbon ones - is
out of scope here: those are different pipelines rather than one pipeline
described twice, and telling them apart needs the name each is assigned to.
"""
import difflib
import pathlib
import re
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent


def first_chain(src: str):
    """The PipelineBuilder call chain at the start of `src`, as set-calls."""
    m = re.search(r"PipelineBuilder\(\)(.*?)\.build\(", src, re.S)
    if not m:
        return None
    body = re.sub(r"//[^\n]*", "", m.group(1))
    return [f"{k}({re.sub(r'\s+', ' ', v).strip()})"
            for k, v in re.findall(r"\.(set\w+)\(([^;]*?)\)\s*\n", body)]


def main() -> int:
    root = REPO / "src" / "rendering"
    if not root.is_dir():
        print("src/rendering is not here; the zero below would mean the scan broke.")
        return 1

    checked = 0
    problems = []
    for path in sorted(root.rglob("*.cpp")):
        text = path.read_text(errors="ignore")
        if "recreatePipelines" not in text or "PipelineBuilder()" not in text:
            continue
        init = text.find("::initialize(")
        recreate = text.find("::recreatePipelines(")
        if init < 0 or recreate < 0:
            continue
        built_first = first_chain(text[init:recreate] if init < recreate else text[init:])
        built_again = first_chain(text[recreate:])
        if not built_first or not built_again:
            continue
        checked += 1
        if built_first != built_again:
            diff = [l for l in difflib.unified_diff(built_first, built_again,
                                                    "initialize", "recreate",
                                                    n=0, lineterm="")
                    if l.startswith(("+", "-")) and not l.startswith(("+++", "---"))]
            problems.append((path.name, diff))

    if checked == 0:
        print("Found no renderer that builds a pipeline twice, which cannot be "
              "right - the scan broke rather than the renderers changing.")
        return 1

    print(f"{checked} renderer(s) build a pipeline in both initialize and "
          f"recreatePipelines")
    print(f"\n{len(problems)} that rebuild a different one:")
    if not problems:
        print("  (none)")
    for name, diff in problems:
        print(f"  {name}")
        for line in diff:
            print(f"      {line}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
