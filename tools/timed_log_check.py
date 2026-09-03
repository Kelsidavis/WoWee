#!/usr/bin/env python3
"""A log line whose only bound is a clock: once a second, forever.

    tools/timed_log_check.py

WHY

This is the spam shape this tree actually produces. The line is guarded, so it
reads as bounded, and the guard is a rate limit rather than a count: at most
one a second. That is a bound on the rate and none at all on the total, so any
condition that holds for a while writes the same sentence for as long as it
holds - and the conditions that hold for a while are the ordinary ones. A
player standing on a spot with no floor under it wrote the same three
coordinates once a second for as long as they stood there; a held mouse button
answered "hit nothing" 27 times in a minute of play.

The fix is never a longer interval. It is to say it once per distinct thing -
per place, per answer, per phase - and to say so when there are no more to
report.

WHAT IT LOOKS FOR

A LOG_INFO, LOG_WARNING or LOG_ERROR guarded by a static timestamp and a
difference against it - "not again for another second" - with nothing that
could tell one occurrence from another: no set, map or vector of what has been
said, no comparison against the last thing said, no counter counting down.

LOG_DEBUG is not reported: it is off in a normal run.

WHAT IT CANNOT SEE

A rate limit written around the call rather than inside it, and one whose
distinguishing state is a member of the class. Both read as bounded here.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
LOUD = re.compile(r"\bLOG_(?:INFO|WARNING|ERROR)\s*\(")
# The clock has to be the guard, not just something the function also times:
# a static timestamp, and a difference against it compared with an interval.
CLOCK = re.compile(r"static[^\n]*(steady_clock|double|Uint32|float)[^\n]*"
                   r"(last|next|prev)\w*", re.I)
SINCE = re.compile(r"(now|current\w*|\bt\b)\s*-\s*\w*(last|prev)\w*\s*[<>]|"
                   r"\w*(last|prev)\w*\s*\+\s*\w+\s*[<>]|"
                   r"-\s*\w*(Last|Prev)\w*\s*[<>]", re.I)
# Something that can tell one occurrence from another.
DISTINCT = re.compile(r"\bstd::(set|map|unordered_set|unordered_map|vector)\b|"
                      r"\bcount\(|\binsert\(|\bfind\(|last\w*\s*!=|!=\s*last\w*|"
                      r"\b\w*[Rr]emaining\b|\b\w*Count\s*<|\bworst\w*\b|already\w*|"
                      # A budget ends the report; a rate limit only spaces it out.
                      r"\bLogBudget\b|\.take\(\)")
LOOK_BACK = 14   # lines of context above the log call that form its guard

# Sites read and judged. Each is a rate limit that is right as it stands.
SETTLED = {
    ("src/rendering/renderer.cpp", "SLOW render stage"):
        "A frame that takes too long is not a state that persists - it is one "
        "frame, and the next one is measured fresh. The rate limit is there to "
        "keep a bad second from filling the log, not to stand in for a count.",
}


def settled_for(path, window):
    for (f, needle), _ in SETTLED.items():
        if path.endswith(f) and needle in window:
            return True
    return False


def main() -> int:
    findings = []
    files = 0
    for path in sorted((ROOT / "src").rglob("*.cpp")):
        files += 1
        lines = path.read_text(errors="ignore").split("\n")
        for i, line in enumerate(lines):
            if not LOUD.search(line):
                continue
            window = "\n".join(lines[max(0, i - LOOK_BACK):i + 3])
            if not CLOCK.search(window) or not SINCE.search(window):
                continue
            if DISTINCT.search(window):
                continue
            rel = str(path.relative_to(ROOT))
            if settled_for(rel, window):
                continue
            findings.append((rel, i + 1, line.strip()[:78]))

    if not files:
        print("Found no sources. The zero below means the scan broke.")
        return 1
    print(f"{files} source file(s), {len(SETTLED)} site(s) settled and not reported\n")
    print(f"{len(findings)} log line(s) bounded by a clock and nothing else:")
    if not findings:
        print("  (none)")
    for f, line, text in findings:
        print(f"  {f}:{line}")
        print(f"      {text}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
