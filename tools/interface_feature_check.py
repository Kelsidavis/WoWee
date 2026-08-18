#!/usr/bin/env python3
"""A Lua global tested for truth, where a missing one answers with a no-op.

The client answers an unknown global with a no-op object so that an interface
written against a newer API does not die on the first missing name. That object
is truthy, callable, and indexable, and calling it returns another one. Which
means the ordinary Lua idiom for "is this available"

    (CloseMenus and CloseMenus())

is satisfied by a function that does not exist, and the branch that was meant
to run only when it did runs always. Escape shipped like that: it asked three
close-functions in turn, the first answered whether or not it was there, and
the press was consumed before it could reach the branch that opens the game
menu - so Escape could shut the menu and never raise it.

The client says this at startup in as many words: "unknown globals answer with
a no-op, so feature detection will read wrong". This finds the places doing it.

What passes: rawget, which returns nil for a name the client does not have, and
a type check on the result.

    (type(rawget(_G, 'CloseMenus')) == 'function' and CloseMenus())
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# Every way a Lua expression is handed to the interface from C++.
ENTRY_POINTS = (
    "askInterface",
    "evaluateBoolean",
    "interfaceCommandBoolean",
    "runInterfaceCommand",
)

# "Foo and Foo(" or "Foo and Foo:" or "Foo and Foo." - a global tested for its
# own truth and then used. Lower-case names are locals in these snippets and
# are not fed by the fallback, so only globals as the interface spells them.
TRUTH_TEST = re.compile(r"\b([A-Z][A-Za-z0-9_]*)\s+and\s+\1\s*[(:.]")

# Guarded forms, any of which makes the line above deliberate rather than a
# reading of the fallback.
GUARDED = re.compile(r"rawget\s*\(|type\s*\(")

STRING_LITERAL = re.compile(r'"((?:[^"\\]|\\.)*)"')


def lua_snippets(text: str):
    """Every string literal in a call to one of the entry points, joined.

    C++ splits a long Lua expression across adjacent literals, so the check has
    to see them as one string or a guard on one line and its use on the next
    would read as unguarded.
    """
    for entry in ENTRY_POINTS:
        for call in re.finditer(re.escape(entry) + r"\s*\(", text):
            # Walk to the matching close paren, so a snippet containing its own
            # parentheses is not cut short.
            depth = 0
            end = call.end() - 1
            for i in range(call.end() - 1, len(text)):
                if text[i] == "(":
                    depth += 1
                elif text[i] == ")":
                    depth -= 1
                    if depth == 0:
                        end = i
                        break
            body = text[call.end() - 1 : end + 1]
            joined = "".join(m.group(1) for m in STRING_LITERAL.finditer(body))
            if joined:
                yield text.count("\n", 0, call.start()) + 1, joined


def main() -> int:
    findings = []
    scanned = 0
    for path in sorted(ROOT.joinpath("src").rglob("*.cpp")):
        text = path.read_text(errors="ignore")
        if not any(e in text for e in ENTRY_POINTS):
            continue
        scanned += 1
        for line, snippet in lua_snippets(text):
            if GUARDED.search(snippet):
                continue
            for match in TRUTH_TEST.finditer(snippet):
                findings.append(
                    (path.relative_to(ROOT), line, match.group(1), snippet.strip()[:90])
                )

    print(f"{scanned} file(s) ask the interface anything.")
    if not findings:
        print("\nno global is tested for its own truth; a missing one cannot answer.")
        return 0

    print(f"\n{len(findings)} global(s) tested for truth, which a missing one passes:\n")
    for rel, line, name, snippet in findings:
        print(f"  {rel}:{line}: `{name} and {name}(...)`")
        print(f"      {snippet}")
    print(
        "\nUse rawget and a type check instead, so a name the client does not\n"
        "have reads as absent rather than as a no-op that answers yes."
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
