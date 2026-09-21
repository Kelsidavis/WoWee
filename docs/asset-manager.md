# The asset manager

`wowee_assets` builds the asset tree this client reads, out of a World of
Warcraft installation you own. It is a native program - SDL3 and Dear ImGui,
both of which the client already carries - and it calls the extractor in its own
process. Nothing is shelled out to, and it needs no interpreter of its own. The
client draws the same panel itself; see [In the client](#in-the-client).

```
wowee_assets [game folder] [a later client] [where to put it]
```

All three are optional: each is a field in the window, filled by typing or
through the system's folder chooser, and the game folder can also be dropped
onto the window. On Windows, `asset_extract` run with no arguments - a
double-click - opens this window instead when it sits beside it.

## What it asks

1. **Which game's assets do you want to use?** The installation to take them
   from, and where the assets should go. The archives say which game they came
   from, so it reads that off rather than asking. The destination defaults to
   the per-user data directory the client itself looks in -
   `~/Library/Application Support/Wowee/Data` on macOS,
   `%LOCALAPPDATA%\Wowee\Data` on Windows, `$XDG_DATA_HOME/wowee/Data` (or
   `~/.local/share/wowee/Data`) on Linux - so a build lands where the client
   will find it.

2. **Which game does your server run?** Vanilla, The Burning Crusade, Wrath,
   Cataclysm, or Turtle. Preselected to whatever the folder turned out to be.

3. **Do you want updated assets?** Optional, and each can be left off. The
   second installation an upgrade needs is asked for beside it:

   | Upgrade | Needs | What it does |
   | --- | --- | --- |
   | Sharper foliage | nothing | Resamples every foliage texture into a sidecar beside the original. Reversible. |
   | Cataclysm's trees and doodads | a Cataclysm install | The re-authored foliage, where it sits at the same paths. Not offered for Cataclysm itself. |
   | Legion's creatures and doodads | a Legion install | Models converted to the format this client reads. |
   | Legion's player character models | a Legion install | Unproven on screen. |

4. **What will happen.** The steps, in the order they will run, and roughly how
   long. Models are imported before the foliage is sharpened, because the
   upscale resamples the textures of the models that are there.

## Several games in one folder

Each game is built under `expansions/<id>` in the destination, so building a
second one into the same folder adds to it rather than replacing it. The window
says which are already there, and the client offers the choice at its login
screen - an Assets row appears once more than one set is installed.

## Packs

**Save what I have as a pack** writes everything in the destination to one
`.zip` beside it, `wowee-<name>-<date>.zip`, with a `pack.json` describing it.
It is offered whenever the destination holds a built game, not only after a
build. With more than one game in there the button reads **Save all N as one
pack** and the pack is named `universal`, because that is what it holds.

**Install a pack...** unpacks one back over the destination. Any entry naming a
path outside the folder it is installing into is refused: a zip entry's name is
chosen by whoever built the archive.

## Where models come from

The importer asks the other installation for each model already here, by that
model's own path. It does not sweep the installation matching on the name inside
each file - Legion's earth elemental calls itself `ElementalEarth2` and lives at
`elementalearth.m2`, so that match never lands. A Legion installation is read
through CASC and a Cataclysm one through its MPQs, by the same pass. A model is
taken only when it has more than 30% more vertices than the one already there,
and it is resolved whole before anything is written: skins, the animations an
`AFID` chunk names, and every texture it names for itself. A model whose
textures cannot be resolved is refused rather than written half-finished,
because a model on disk that looks complete and draws flat white is worse than
one that is not there.

## In the client

The client carries the same panel, drawn through its own Vulkan-backed ImGui
on the login screen's paper. When it finds no extraction at startup it opens
the builder instead of the login screen, since there is no getting past login
without assets. With assets installed, "add or rebuild assets..." under "more
options" on the login card opens it again, with a way back. The client reads
its assets once at startup, so a build wants the client closed and reopened.
It is compiled in only where StormLib is (`WOWEE_HAVE_ASSET_PANEL`).

## Reporting what it looks like

`WOWEE_ASSETS_SCREENSHOT=<file> wowee_assets` writes a PNG of the window and
exits, for a bug report from a machine where taking a screenshot is the hard
part.
