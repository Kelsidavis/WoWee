# The widget system

The addon API used to answer without doing anything. `CreateFrame` returned a
table, events dispatched to it, and `CreateTexture` handed back an object whose
every method was a no-op - so an addon could be written, loaded and run without
putting a pixel on the screen.

There is now a real retained widget tree behind it. The same tree is what
FrameXML targets, because FrameXML is only Lua and XML over a widget system, so
building it once serves both goals: addons that draw, and a route to running the
original interface rather than imitating it.

## Shape

| Piece | Where | Notes |
|---|---|---|
| Widget tree, anchors, draw order, hit testing | `src/ui/widget_tree.cpp` | No Vulkan or ImGui, so the layout rules are testable without a device |
| Drawing, texture cache, backdrops, status bars | `src/ui/widget_renderer.cpp` | Reads `Interface\` art through the existing asset path |
| XML reader | `src/ui/xml_parser.cpp` | Enough for FrameXML: CDATA, comments, both quote styles |
| XML to Lua | `src/ui/framexml_emitter.cpp` | Emits the calls a script would make |
| Lua bindings | `src/addons/lua_engine.cpp` | Frames and regions are Lua tables carrying a `__wid` handle |

Coordinates follow WoW throughout - origin bottom-left, y upward - and flip once
at the point of drawing, so every anchor rule reads the way Blizzard documents
it rather than mirrored.

Anchors are constraints, not positions. An anchor says "this fraction of my rect
sits at that point", so one anchor plus a size places a frame and two opposing
anchors give the size as well. That is what `SetAllPoints` relies on, and how
most of FrameXML sizes its backgrounds without ever stating a size.

## Why XML becomes Lua

The alternative was to build widgets from C++ while walking the XML, which would
have meant a second implementation of everything `CreateFrame` already does -
parenting, naming, templates, script binding - kept in step with the first by
hand. Emitting Lua means XML frames and hand-written frames travel one path, a
template declared in XML is usable from a script without translation, and the
emitter's output is a string a test can read without a Lua state.

## Environment switches

Both are off by default. Both exist because the work they enable is not finished.

### `WOWEE_LUA_API_FALLBACK=1`

Unknown globals answer with a no-op instead of erroring, and every name asked
for is logged once and listed at shutdown.

This is how a large body of Lua gets brought up: rather than guessing which of
the missing functions matter, run it and collect the ones it actually reaches.

It has a real cost. Code that checks whether a function exists before using it -
which addons do constantly - sees everything as present and takes branches meant
for a different client. Names in `SCREAMING_SNAKE_CASE` are treated as constants
and still come back nil, because handing a function to something expecting a
number turns a missing value into a confusing type error further away.

### `WOWEE_LOAD_FRAMEXML=1`

Loads the original interface from `Interface/FrameXML/FrameXML.toc`, in the
order that manifest states, before any addon. It turns the fallback above on by
itself, because FrameXML cannot get through its own load without one.

Every file that fails is listed together at the end of the load, with the reason
carried up from whichever include or referenced script actually broke, and each
error carries the Lua call stack that reached it.

All 139 files in the manifest now load, in around 380ms.

    FrameXML: 13 Lua files and 126 XML files loaded, 0 failed in 377ms

That is the whole original interface built against this client's widget tree.
What remains is behaviour rather than loading: frames exist, are laid out and
are named the way FrameXML expects, but the API behind them mostly answers with
what the absence of a feature looks like.

### Whether an event actually arrives

    tools/addon_events.sh              every event this client fires
    tools/addon_events.sh ACTIONBAR    only those matching

FrameXML's frames update on events, so replacing one of the client's own
elements means knowing which of them arrive. Four different call styles
dispatch them - fireEvent, fireAddonEvent, an emit on a pending queue, and the
callback invoked directly - and grepping for one under-reports the rest badly:
the same question answered 6, 52, 73 and 147 depending on which was searched.
Ask the script.

### Working out what is still missing

    tools/framexml_api_gap.py <path to Interface/FrameXML>

reports 1,142 names FrameXML calls that this client does not define, out of
4,217 it calls in total and 2,724 it defines itself as it loads. That ranking
counts static call sites, though, and most are never reached.

The measurement that matters is a run with the fallback off:

    WOWEE_LUA_API_FALLBACK=0 WOWEE_LOAD_FRAMEXML=1 ./wowee

With the fallback on, a missing name answers and the gap is invisible. With it
off, the log names every one FrameXML actually reached - which is how the list
that mattered was found, rather than by guessing from the ranking.

Two tools check the front half of the pipeline, and neither has been the
constraint for some time: `tools/framexml_compile_check.cpp` asks Lua whether
every generated file compiles (140/140), and the emitter has unit tests in
`tests/test_framexml.cpp` covering the XML features that were silently absent -
template inheritance, `parentKey`, `id`, `<ScrollChild>`, button art, handler
argument names, and `$parent` through unnamed frames.

## Replacing one element at a time

This is how the interface was replaced, and it is finished: FrameXML draws all
fifty-two elements. `WOWEE_FRAMEXML_UI`, which named them one at a time, has
been removed along with the thing its other setting selected - the client's own
version of nearly every element has been deleted, so `=none` stopped meaning
"use this client's interface" and started meaning "draw nothing".

What survives in `framexml_takeover.hpp` is the accounting: the frames each
element stands or falls on, the handful of places this client still draws into
a frame FrameXML owns, and the net that hands an element back when FrameXML's
version of it was never built.

Two things had to happen for an element to change hands, and only the first is
obvious. They are what the notes in that file are about.

**The client stops drawing its own.** One `frameXmlOwns` check where it used to
draw. The keybinding has to follow: each panel polls its own key from inside
its own draw, so a panel that is no longer drawn never sees the key, and
handing one over made it unopenable until the key was routed to FrameXML's own
toggle instead.

**Anything the client renders itself needs handing over explicitly.** This is
the part that is easy to miss, because the frame appears and looks right and
is empty. FrameXML's frames are frames: the picture inside them is this
client's, and it has to be told where to go.

| what | how it is handed over |
|---|---|
| unit portrait | an offscreen character pass, given to the widget as a texture |
| minimap | a Vulkan pass of its own - told the frame's rect, since it cannot be sampled |
| world map | an ImGui window - told the rect, and to drop its own title bar |
| paperdoll model | a second offscreen pass, framed to the whole figure |

A frame carrying one of these draws it beneath its own regions, so the art
around it lands on top. The rect is a frame behind, because those passes have
already run by the time the tree lays out - which for a frame that does not
move is not visible.

## Known gaps

- Type is drawn from the game's own faces - FRIZQT, MORPHEUS, SKURRI, ARIALN
  and FRIENDS - at the size and colour FrameXML's 42 font objects specify. Each
  face is built into the atlas at one size and scaled, so a heading is the right
  face rather than the right rasterisation. Outlines are drawn by offsetting
  copies of the glyphs, which is what the effect amounts to at these sizes.
- `EditBox` takes text, keeps a caret and fires OnTextChanged, OnEnterPressed
  and the focus handlers. It has no selection, no clipboard and no scrolling
  past its own width. `Slider` drags and reports its value; `Cooldown` sweeps.
- The texture cache never evicts, and cannot yet: `uploadImGuiTexture` has no
  counterpart, so releasing one would mean tracking its image and memory and
  destroying them only once the GPU is done. `Interface\` art is small, bounded
  and reused, so this grows to a few hundred entries and stops; a session that
  loaded art from many addons would keep growing.
- The widget method set in `lua_engine.cpp` is enumerated rather than derived.
  A method outside it answers nil instead of doing nothing, which for an addon
  is an error rather than a shrug. Every such name is recorded once as
  `widget:Name`, so the gap shows up in the shutdown report rather than as a
  mystery; adding it to the set is a one-line fix.
- Blend modes are honoured only far enough to tell "added" apart from "drawn
  over". `alphaMode="ADD"` art carries no alpha channel of its own - it is a
  glow on black - so it is uploaded as a second copy of the image with its
  alpha taken from brightness, which over a dark scene lands close to where
  adding would. `MOD` and `ALPHAKEY` are still drawn as ordinary blending. One
  ImGui draw list has one blend state, so anything better means a second
  pipeline.

## Diagnostics

Every switch below is read once, from the environment, and costs nothing when
unset.

- `WOWEE_LOAD_FRAMEXML=0` turns Blizzard's interface off. It is loaded by
  default and it is the interface - there is no longer a second one behind it,
  so a run with this set has a world, a chat log in the terminal and no frames.
- `WOWEE_FRAMEXML_EMIT_DIR=/tmp/emit` writes the Lua each XML file became, one
  file per source file. A nil global or a frame in the wrong place is nearly
  always answered by one grep through this.
- `WOWEE_WIDGET_DUMP=1..5` reports what the renderer believes: 1 lists what was
  drawn, 2 every named widget whether drawn or not, 3 outlines them on screen,
  4 fills them solid, 5 also draws ImGui's own font atlas through the same call
  - which separates "AddImage does not work here" from "these textures are bad".
- `WOWEE_LUA_API_FALLBACK=0` turns off the stub that answers unknown globals,
  so the log names every API FrameXML actually reached.
- `WOWEE_EVENT_TRACE=UNIT_HEALTH,UNIT_MANA` reports each of those events and how
  many frames received it. An event that never arrives and an event nobody
  listens for look identical from outside - the frame simply does not change -
  and they need opposite fixes.

With any element handed over, a check runs once the tree has settled and
reports the frames that element stands or falls on: built or not, shown or
hidden, its rect, whether it landed off screen, whether its art reached the
GPU, and for a status bar the value and range it was given. It also names any
visible widget that landed outside the display, whoever owns it - a frame in
the wrong place is only findable by name if you can guess the name, and the
thing that looks wrong is rarely the thing you would have thought to check.

The missing-API report at shutdown separates three things that are not the
same, and writes the full list to `missing_api.txt` beside the log:

- names still undefined, which is the real gap;
- names read before the file defining them had loaded, which is normal;
- names built from an existing frame's, which are parts that frame may or may
  not have. FrameXML asks for these constantly and guards them properly.

The third category was 183 of 222 on one session. Reading the report without
that split says close to the opposite of the truth.
