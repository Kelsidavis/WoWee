#!/usr/bin/env python3
"""Widget methods FrameXML calls that answer nil - and so raise when called.

A missing *global* is harmless: the fallback makes it callable and it answers
nil. A missing *method* is not. The frame metatable answers a no-op only for
names in __WoweeWidgetMethods; anything else comes back nil, and `frame:Foo()`
on nil is "attempt to call method 'Foo' (a nil value)" - a real error that
takes down whatever handler asked.

So the known-methods set is doing the same job for methods that the fallback
does for globals, and a name missing from *it* is the crashing case.

**Read the false positives below before trusting a hit.** Four rounds of them
came out of the first run, and the first draft confidently reported GetOwner
and IsOwned as missing when both are implemented. A method counts as answered
if it is any of:

  - in __WoweeWidgetMethods (a recorded no-op)
  - registered from C, however the registration is spelled
  - defined in bootstrap Lua on *any* metatable name - mt, __WoweeFrameMT,
    animMeta, groupMeta - not only `function mt:`
  - defined by the interface itself on its own objects, either as
    `function Obj:Method()` or assigned as a field, including
    `self.Desaturate = AchievementIcon_Desaturate`, which names an existing
    function rather than an inline one

  - guarded by the caller: `if (context.Result) then return context:Result()`
    reads the method as a field first, so a missing one is a false branch and
    not an error. dump.lua does exactly this and was reported twice for it.

Run it after any batch of widget work. A hit that survives all five is a real
call on nil.
"""
import re, pathlib, collections

# Paths are resolved against the repository, not the working directory:
# run from build/ these globs matched nothing and the report came out
# clean from an empty scan, which is the exact shape of a check that
# cannot fail.
REPO = pathlib.Path(__file__).resolve().parent.parent

src = (REPO / "src" / "addons" / "lua_engine.cpp").read_text()

# The curated set that answers a no-op.
block = re.search(r'"__WoweeWidgetMethods = \{\\n"(.*?)"\}\\n"', src, re.S)
known = set(re.findall(r'(\w+)=1', block.group(1))) if block else set()

# Methods actually implemented, however they are registered.
impl = set(re.findall(r'set\("(\w+)"', src))
impl |= set(re.findall(r'\{(?:\.\w+\s*=\s*)?"(\w+)",\s*(?:\.\w+\s*=\s*)?lua_\w+\}', src))
impl |= set(re.findall(r"mt[:.]\s*(\w+)\s*=|function mt:(\w+)", src))
impl |= {m for pair in re.findall(r"\"function mt:(\w+)", src) for m in (pair,)}
impl |= set(re.findall(r"\"mt\.(\w+) =", src))
impl |= set(re.findall(r"\"mt\['(\w+)'\]", src))
# Methods the bootstrap Lua defines on a metatable, under whatever name that
# metatable is bound to there - mt, __WoweeFrameMT, animMeta, groupMeta.
# Looking only for `function mt:` reported GetOwner and IsOwned as missing when
# both are defined a few lines apart as `function __WoweeFrameMT:...`.
impl |= set(re.findall(r"function\s+[\w.]*[Mm][Tt]\w*\s*:\s*(\w+)", src))
impl |= set(re.findall(r"function\s+\w*[Mm]eta\w*\s*:\s*(\w+)", src))
impl |= set(re.findall(r"\w+\.(\w+)\s*=\s*function\s*\(self", src))
# Installed on the frame table itself rather than on the metatable, which
# is how a method that only some frame types have is given out - FrameXML
# tests for the presence of those, so they cannot live on the table every
# frame shares. Capitalised only: the same call writes __wid and __name.
impl |= set(re.findall(r'lua_setfield\(L, -2, "([A-Z]\w+)"\)', src))

answered = known | impl

# Methods the interface defines on its own objects. dump.lua's context:Write
# and the achievement buttons' Collapse are ordinary Lua methods on ordinary
# Lua tables - nothing to do with the frame metatable, and not missing.
interface_defined = set()
# One literal, so sweep_guard can see this sweep needs the interface and
# skip it where there is none. A path in separate components is invisible
# to that check, and the sweep then runs on CI, finds nothing, and is
# failed for reporting nothing.
for _f in list((REPO / "Data/interface").glob("framexml/*.lua")) + \
          list((REPO / "Data/interface").glob("addons/*/*.lua")):
    _t = _f.read_text(errors="ignore")
    interface_defined |= set(re.findall(r'\bfunction\s+[\w.]+[:.](\w+)\s*\(', _t))
    # `self.Desaturate = AchievementIcon_Desaturate` - assigned to a named
    # function, not an inline one. Requiring `= function` missed every method
    # installed that way, which is most of the achievement buttons'.
    interface_defined |= set(re.findall(r'[\w.\]\[]+\.(\w+)\s*=\s*[\w.]+\s*;?\s*$', _t, re.M))
    interface_defined |= set(re.findall(r'[\w.]+\.(\w+)\s*=\s*function', _t))
    # Assigned the *result of a call*, which is how dump.lua installs its
    # three name lookups:
    #     context.GetTableName = Pick_Cache_Function(DevTools_Cache_Table,
    #                                                DEVTOOLS_USE_TABLE_CACHE);
    # The earlier patterns wanted the right-hand side to be a bare name or an
    # inline function and to end on the same line, so all three read as missing
    # client methods when the interface defines them itself.
    interface_defined |= set(re.findall(r'[\w.\]\[]+\.(\w+)\s*=\s*[\w.]+\s*\(', _t))
answered |= interface_defined

interface = (REPO / "Data/interface")
files = list(interface.glob("framexml/*.lua")) + list(interface.glob("addons/*/*.lua"))

# obj:Method( - a real method call. Not obj.Method, which is a field read.
CALL = re.compile(r'(?<![\w."\'])([A-Za-z_]\w*)\s*:\s*([A-Z]\w*)\s*\(')

# obj.Method read as a field - the caller checking whether it exists.
GUARD = re.compile(r'\b([A-Za-z_]\w*)\.([A-Z]\w*)\b')

hits = collections.defaultdict(list)
for f in files:
    lines = f.read_text(errors="ignore").splitlines()
    guarded = {}          # (obj, meth) -> line it was tested on
    for i, line in enumerate(lines, 1):
        if "if" in line or "and" in line or "or" in line:
            for obj, meth in GUARD.findall(line):
                guarded[(obj, meth)] = i
        if line.lstrip().startswith("--") or line.lstrip().startswith("function"):
            continue
        for obj, meth in CALL.findall(line):
            if meth in answered:
                continue
            if meth.startswith("On"):        # script-handler names, read as fields
                continue
            # Tested a few lines above is the author handling its absence. The
            # window is small on purpose: a check far away is not protecting
            # this call.
            at = guarded.get((obj, meth))
            if at is not None and 0 <= i - at <= 3:
                continue
            hits[meth].append(f"{f.name}:{i}: {line.strip()[:80]}")

# Methods the emitter writes into the Lua it generates.
#
# These never appear in any .lua or .xml file, so the scan above cannot see
# them - and a method emitted for an attribute nobody implemented is the same
# "attempt to call method" as any other, only harder to trace because the call
# site does not exist in any file anyone reads. Reading `letters` and emitting
# SetMaxLetters is safe because SetMaxLetters exists; reading `horizTile` and
# emitting SetHorizTile would not be.
EMITTED = re.compile(r'\+\s*":([A-Z]\w*)\("')
emitter = (REPO / "src" / "ui" / "framexml_emitter.cpp").read_text(errors="ignore")
for meth in sorted(set(EMITTED.findall(emitter))):
    if meth in answered or meth.startswith("On"):
        continue
    hits[meth].append(("emitted by framexml_emitter.cpp",
                       "src/ui/framexml_emitter.cpp: written into generated Lua"))

for meth in sorted(hits, key=lambda m: -len(hits[m])):
    print(f"\n### {meth}  ({len(hits[meth])} call sites)")
    for h in hits[meth][:3]:
        print("   ", h)
# What it looked at, before what it found.
if not answered:
    print("Found no widget methods at all, which cannot be right - the "
          "metatable parse broke rather than every method disappearing.")
    raise SystemExit(1)
print(f"\n{len(answered)} widget method(s) this client answers, checked "
      f"against what the interface calls")
print(f"\n{len(hits)} methods called that neither the metatable nor the known set answers")
