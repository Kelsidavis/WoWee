#!/usr/bin/env python3
"""Frames the takeover tables name that no file the loader opens declares.

    tools/framexml_promised_frames.py

The tables in src/ui/framexml_takeover.cpp are promises. A suppression row says
"FrameXML draws this, hide it when this client owns the element"; a check row
says "if FrameXML owns the element, these frames have to exist for it to have
arrived". Both are written from reading the interface by hand, and both go
stale silently - nothing checks that the frame named is one the interface can
actually produce, and a promise pointing at nothing fails invisibly: no error,
no warning on screen, just a frame that is not there.

THE CASE THAT NAMED THIS, WHICH WAS A FALSE ALARM

FocusFrame. focusframe.xml is in no manifest and included by no XML, and
"focusframe" is in the branch's default set - so it read as: this client's own
focus frame gated off, FrameXML's never built, nothing on screen. Wrong.
FocusFrame is declared in targetframe.xml, inheriting TargetFrameTemplate, and
loads with it. The file that shares its name is the unused one.

That is the mistake this exists to stop, and it is a mistake made by hand from
exactly the evidence this reads. So it is worth noting what the report would
have said: FocusFrame resolves, and the fix that looked obvious would have
declared the frame twice.

WHAT IT LOOKS FOR

Every name in the takeover tables, against every name declared by a file the
loader actually reaches - manifests and the Script/Include graph, via
framexml_source.loaded_files, so a file sitting in the folder unreferenced does
not count as a definition. $parent chains are followed within a file, and
nested, because they nest.

IT CURRENTLY FINDS ONE THING, AND IT IS NOT REAL

BuffButton1, which BuffFrame builds in Lua as "BuffButton"..i. A name assembled
from pieces is invisible here. That is the one standing false positive; it is
left in the count rather than special-cased, because a list with an exception
in it stops being checkable.

Verified failable: dropping TargetFrame.xml from the manifest takes the report
from one row to fourteen, FocusFrame among them.

WHAT IT CANNOT SEE

Whether a frame that exists is ever shown. That is the runtime takeover
check's question, and the two are complementary - this one finds what cannot
appear, the other finds what did not.
"""
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from framexml_source import loaded_files, without_comments

ROOT = Path(__file__).resolve().parent.parent
INTERFACE = ROOT / "Data/interface"
TAKEOVER = ROOT / "src/ui/framexml_takeover.cpp"

#: Rows of the two tables that name frames. The element enum tells them apart
#: from every other string literal in the file - a comment full of frame names
#: is not a promise, and neither is a log message.
ROW = re.compile(r"\{\s*UiElement::(\w+)\s*,\s*((?:\"[^\"]*\"\s*)+)")

#: Names the interface declares.
DECLARED = re.compile(r'\bname="([A-Za-z][A-Za-z0-9_]*)"')
#: ...and the ones it declares relative to whatever contains them. A status bar
#: written `name="$parentHealthBar"` inside TargetFrame is TargetFrameHealthBar
#: at runtime and appears under no such name in the file, so the suffix has to
#: be matched against a declared parent instead. Without this the report is
#: mostly bar and portrait children of frames that plainly exist.
RELATIVE = re.compile(r'\bname="\$parent([A-Za-z][A-Za-z0-9_]*)"')
#: ...and names it builds at runtime, where the literal is written out.
CREATED = re.compile(r'CreateFrame\s*\(\s*"[^"]*"\s*,\s*(?:\.\w+\s*=\s*)?"([A-Za-z][A-Za-z0-9_]*)"')

#: Elements whose rows carry an element name rather than frame names - the
#: first table in the file maps UiElement to the lowercase name a run asks for.
LOWERCASE = re.compile(r"^[a-z0-9]+$")


def promised():
    """frame name -> the element that promised it."""
    text = TAKEOVER.read_text(errors="ignore")
    out = {}
    for element, blob in ROW.findall(text):
        for lit in re.findall(r'"([^"]*)"', blob):
            for word in lit.split():
                if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", word):
                    continue
                if LOWERCASE.fullmatch(word):
                    continue  # the element's own name, not a frame
                out.setdefault(word, element)
    return out


#: A frame declaring what it is built from. Template children are declared
#: inside the template, in whatever file that lives in, and resolve against
#: whoever instantiates it - PartyMemberFrame1 is declared in partyframe.xml
#: and its health bar in partyframetemplates.xml.
INHERITS = re.compile(r'<\w+\b[^>]*\bname="([A-Za-z][A-Za-z0-9_]*)"[^>]*\binherits="([A-Za-z][A-Za-z0-9_]*)"'
                      r'|<\w+\b[^>]*\binherits="([A-Za-z][A-Za-z0-9_]*)"[^>]*\bname="([A-Za-z][A-Za-z0-9_]*)"')
#: A virtual frame and the relative names inside it.
TEMPLATE = re.compile(r'<(\w+)\b[^>]*\bname="([A-Za-z][A-Za-z0-9_]*)"[^>]*\bvirtual="true"')


def _template_suffixes(text):
    """template name -> the $parent suffixes declared inside it.

    Bounded by the next virtual declaration rather than by matching tags: the
    files nest several levels and a tag matcher here would be a parser. The
    overshoot only ever adds suffixes, and a suffix that belongs to a
    neighbouring template still names a real child of something.
    """
    out = {}
    marks = [(m.start(), m.group(2)) for m in TEMPLATE.finditer(text)]
    for i, (at, name) in enumerate(marks):
        end = marks[i + 1][0] if i + 1 < len(marks) else len(text)
        out[name] = set(RELATIVE.findall(text[at:end]))
    return out


def declared():
    """(every name declared, and per file: names, suffixes, inherits, templates)"""
    names, per_file = set(), []
    templates = {}
    inherits = {}
    for path in loaded_files(INTERFACE):
        text = without_comments(path.read_text(errors="ignore"))
        mine = set(DECLARED.findall(text)) | set(CREATED.findall(text))
        names |= mine
        per_file.append((mine, set(RELATIVE.findall(text))))
        templates.update(_template_suffixes(text))
        for a, b, c, d in INHERITS.findall(text):
            if a and b: inherits.setdefault(a, set()).add(b)
            if c and d: inherits.setdefault(d, set()).add(c)
    # A frame's own suffixes plus every one its templates bring with them.
    for frame, parents in inherits.items():
        extra = set()
        for t in parents:
            extra |= templates.get(t, set())
        if extra:
            per_file.append(({frame}, extra))
    return names, per_file


def resolves(name, names, per_file):
    """Is this a name the interface can produce?

    Either declared outright, or reached by hanging relative suffixes off a
    declared frame - within one file, because that is what $parent means, and
    repeatedly, because the chains nest. TargetFrameTextureFrameName is a
    $parentName inside a $parentTextureFrame inside TargetFrame, and stopping
    at one level reported it as a frame that cannot exist.

    Matching across files instead of within one is the trap here: some file
    declares a Focus and some other file hangs a Frame off its parent, and the
    two halves meet at a boundary neither crosses at runtime.
    """
    if name in names:
        return True
    for mine, suffixes in per_file:
        reached = {p for p in mine if name.startswith(p) and len(p) < len(name)}
        while reached:
            grown = set()
            for parent in reached:
                rest = name[len(parent):]
                if rest in suffixes:
                    return True
                grown |= {parent + s for s in suffixes
                          if name.startswith(parent + s) and len(parent + s) < len(name)}
            reached = grown - reached
    return False


def main():
    names, per_file = declared()
    want = promised()
    missing = sorted((n, e) for n, e in want.items()
                     if not resolves(n, names, per_file))

    print(f"{len(want)} frame name(s) promised by the takeover tables, "
          f"{len(names)} declared by files the loader reaches\n")
    print(f"{len(missing)} promised by a table and declared nowhere:\n")
    for name, element in missing:
        print(f"  {name:<36} {element}")
    if not missing:
        print("  (none)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
