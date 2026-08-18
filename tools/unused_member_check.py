#!/usr/bin/env python3
"""Class members stored and never read.

    tools/unused_member_check.py

WHY

A member assigned in a constructor and never read again is dead state that
still costs a pointer's worth of lifetime coupling, and it is usually the
remains of something that was removed halfway.
WorldLoader::worldLoadGeneration_ carried a comment describing re-entrant load
detection that was never written.

Clang reports these as -Wunused-private-field, but only on some versions. The
clang on the Windows CI image reported two that clang 18 on Linux does not, so
a local build is not a gate for this class and CI catches it one per run.

This asks the same question with grep, so any host answers it.

WHAT IT DOES

For each class member declared in a header under include/, counts every
mention of that name across include/, src/ and tools/. A member whose only
mentions are its declaration and a constructor initialiser is reported.

WHAT IT CANNOT SEE

Members reached through a macro, or named the same as something else and so
appearing to be read when they are not. It is deliberately conservative: a
name mentioned anywhere else at all is left alone, so it under-reports rather
than crying wolf.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
SOURCES = [ROOT / "include", ROOT / "src", ROOT / "tools"]

# A trailing-underscore member declaration: `Type name_;` or `Type name_ = init;`
DECL = re.compile(
    r"^\s{2,}(?:mutable\s+|static\s+|const\s+)*"
    r"[A-Za-z_][\w:<>,\s\*&]*?[\s\*&]"
    # [ \t]* rather than \s*: under re.M a \s* here runs past the newline and
    # swallows the next blank line and comment into the match, after which the
    # declaration no longer equals its own line and is counted as a read. Every
    # member followed by a comment was silently skipped, which is how
    # AssetManager::looseReader_ reached Windows CI.
    # The name, with or without the trailing underscore this codebase mostly
    # uses. Matching only the underscored form left 314 members invisible, and
    # InventoryScreen::cKeyWasDown reached Windows CI through that hole.
    r"(\w+)[ \t]*(?:=[^;]*)?;[ \t]*(?://.*)?$", re.M)

# Settled: names whose only readers are outside what this can see.
EXPECTED = set()


def files():
    for base in SOURCES:
        if not base.is_dir():
            continue
        for path in base.rglob("*"):
            if path.suffix in (".cpp", ".hpp", ".h", ".inc", ".mm"):
                yield path


def main():
    corpus = {}
    for path in files():
        try:
            corpus[path] = path.read_text(errors="ignore")
        except OSError:
            pass
    if not corpus:
        print("Read no sources. The zero below means the scan broke.")
        return 1

    headers = [p for p in corpus if p.suffix in (".hpp", ".h")
               and str(p).startswith(str(ROOT / "include"))]

    # Every line that mentions a member, so each can be judged a read or a
    # write. Counting mentions alone is not enough: `nextResourceId_++` and
    # `markupParser_.parse(...)` are one mention each and are real uses.
    lines_for = {}
    for path, text in corpus.items():
        for raw in text.split("\n"):
            for name in set(re.findall(r"\b(\w+_)\b", raw)):
                lines_for.setdefault(name, []).append((path, raw))

    # Which files can legitimately touch a member declared in a given header.
    #
    # A private member is reachable only from its own class's methods, and
    # those live in the .cpp named after the header (or one of the files that
    # class was decomposed into). Pooling by name across the whole tree hides
    # the interesting case: worldLoader_ is a member of three unrelated
    # classes, and reads of one made the other two look used. Both fields
    # Windows CI reported were invisible here for exactly that reason.
    #
    # When no matching .cpp exists the scope falls back to everything, which
    # under-reports rather than inventing a finding.
    by_stem = {}
    for path in corpus:
        by_stem.setdefault(path.stem, []).append(path)

    def scope_for(header):
        stem = header.stem
        own = {header}
        for other_stem, paths in by_stem.items():
            if other_stem == stem or other_stem.startswith(stem + "_"):
                own.update(p for p in paths if p.suffix in (".cpp", ".mm"))
        return own if len(own) > 1 else None

    def is_write_only(name, path, decl_line, scope):
        """True when nothing in the declaring class's own files reads this."""
        assign = re.compile(r"(?<![=!<>])\b%s\s*=(?!=)" % re.escape(name))
        init = re.compile(r"[,:]\s*%s\s*\(" % re.escape(name))
        qualified = re.compile(r"(?:\.|->)\s*%s\b" % re.escape(name))
        for where, raw in lines_for.get(name, []):
            if scope is not None and where not in scope:
                # Outside the class's own files an unqualified name belongs to
                # some other class that happens to share it. A qualified one
                # (`app_.playerClass_`) is a real read of this member through
                # an object, which is legal for a public member and for a
                # friend, so it still counts.
                if not qualified.search(raw):
                    continue
            stripped = raw.strip()
            if where == path and stripped == decl_line.strip():
                continue                       # the declaration itself
            if init.search(raw):
                continue                       # a constructor initialiser
            m = assign.search(raw)
            if m and name not in raw[m.end():]:
                continue                       # written and not also read here
            return False                       # anything else reads it
        return True

    # Access level, tracked with a brace stack rather than guessed from the
    # nearest keyword.
    #
    # The guess broke on a nested aggregate: ToastManager::whisperSeenCount_
    # follows `struct WhisperToastEntry { ... };`, and taking the last
    # class-or-struct keyword before it read that struct's public default
    # instead of the enclosing class's private section. It was reported as
    # debt and not as a build failure, and Windows found it.
    OPENER = re.compile(r"\b(class|struct|union)\b[^;{}]*\{(?:\.\w+\s*=\s*)?")
    ACCESS_LABEL = re.compile(r"\b(public|protected|private)\s*:")

    def is_private_at(text, at):
        """Whether a declaration at this offset sits in a private section."""
        stack = []
        i = 0
        while i < at:
            ch = text[i]
            if ch == "{":
                line_start = text.rfind("\n", 0, i) + 1
                m = OPENER.search(text[line_start:i + 1])
                stack.append(("private" if m.group(1) == "class" else "public")
                             if m else None)
            elif ch == "}":
                if stack:
                    stack.pop()
            elif ch == ":":
                line_start = text.rfind("\n", 0, i) + 1
                m = ACCESS_LABEL.search(text[line_start:i + 1])
                if m and stack and stack[-1] is not None:
                    stack[-1] = m.group(1)
            i += 1
        for access in reversed(stack):
            if access is not None:
                return access == "private"
        return False

    def never_referenced(name, path, decl_line, scope):
        """True when the declaration is the only mention of this member.

        Searched directly rather than through the mention index, because that
        index is keyed on the trailing-underscore convention and this question
        has to cover members that do not follow it.
        """
        word = re.compile(r"\b%s\b" % re.escape(name))
        qualified = re.compile(r"(?:\.|->)\s*%s\b" % re.escape(name))
        for where in (scope if scope is not None else corpus):
            text = corpus.get(where)
            if text is None or not word.search(text):
                continue
            for raw in text.split("\n"):
                if not word.search(raw):
                    continue
                if where == path and raw.strip() == decl_line.strip():
                    continue
                return False
        # A public member can be read through an object from anywhere, so a
        # qualified mention outside the scope still counts.
        if scope is not None:
            for where, text in corpus.items():
                if where in scope:
                    continue
                if qualified.search(text):
                    return False
        return True

    dead, unreferenced = [], []
    for header in sorted(headers):
        text = corpus[header]
        scope = scope_for(header)
        for m in DECL.finditer(text):
            name = m.group(1)
            if name in EXPECTED or name.startswith("__"):
                continue
            decl_line = m.group(0)
            # clang's -Wunused-private-field is about non-static data members.
            # A static or constexpr constant is not one, and `operator` is the
            # tail of a deleted assignment operator rather than a member name.
            if name == "operator" or re.search(
                    r"\b(?:static|constexpr|enum|using|typedef)\b", decl_line):
                continue
            if not is_write_only(name, header, decl_line, scope):
                continue
            line = text.count("\n", 0, m.start()) + 1
            entry = (str(header.relative_to(ROOT)), line, name)
            # The debt count stays on the trailing-underscore convention it
            # was built for: its read/write analysis works off an index keyed
            # that way, and widening it there would report every local that
            # shares a name. The clang question below is asked of every member.
            if name.endswith("_"):
                dead.append(entry)
            # Clang's -Wunused-private-field fires only for a member nothing
            # mentions at all; a write still counts as a use. That subset is a
            # build failure on the Windows image, so it is counted apart from
            # the rest, which is debt rather than breakage.
            if never_referenced(name, header, decl_line, scope) and \
                    is_private_at(text, m.start()):
                unreferenced.append(entry)

    print(f"{len(headers)} headers, {len(lines_for)} member names\n")
    print(f"{len(dead)} members stored and never read:")
    for path, line, name in dead:
        print(f"  {path}:{line}  {name}")
    if not dead:
        print("  (none)")
    print()
    print(f"{len(unreferenced)} private and never referenced, which clang rejects:")
    for path, line, name in unreferenced:
        print(f"  {path}:{line}  {name}")
    if not unreferenced:
        print("  (none)")
    return 1 if dead else 0


if __name__ == "__main__":
    sys.exit(main())
