# Upgraded assets

Borrowing art from a later World of Warcraft client you own, so the world this
client draws is the world a later one drew. Nothing here ships assets: every
file comes out of your own installation.

The worked example is Stranglethorn's canopy. The same tree, at the same path,
in two clients:

| | 3.3.5a | 4.3.4 |
|---|---|---|
| `StranglethornTree01.m2` | 156 vertices, 150 triangles | 897 vertices, 2430 triangles |
| canopy | crossed planes, one painted sheet | leaf cards, a leaf atlas |
| leaf texture | 256x256 | 512x512 |

Cataclysm re-authored the trees the classic zones still use and left them where
they were, so this is a file swap rather than a port.

## What can be borrowed, and what cannot

**Doodads, world models and creature models, from any MPQ-era client**: Classic,
TBC, WotLK, Cataclysm and Mists. Their models are `MD20`, their textures are
BLP, and `src/pipeline/m2_loader.cpp` reads both. This is the case that works.

**Warlords onwards needs a different reader.** 6.x moved to CASC, which
`asset_extract` does not read - it is built on StormLib and StormLib is an MPQ
library - and its models are the chunked `MD21`. `wowee_assets` reads that
storage and converts what comes out of it; see
[Reading a CASC installation](#reading-a-casc-installation). That path is
narrower than the MPQ one and worth understanding before relying on it.

**Player character models are unproven.** This is worth stating plainly
because it is the most-asked-for case. The pack tools refuse a character model
that replaces one the target already has: a third-party model pack's HD player
models, taken whole, gave a naked player with segmented limbs, and
`tools/asset_pack_curate.py` puts that down to a different geoset set - no 1,
no 701, no 1501 - where this client picks equipment geosets by number and
composites a 512x512 body texture from `CharSections.dbc`. Measured against
Legion's own `HumanMale`, that reason does not hold: it has the same geoset
groups as 3.3.5's, 1 and 701 included, 7% more vertices, and bone indices that
fit this client's format. What is unsettled is whether its body UVs still match
the skins `CharSections.dbc` names, so `wowee_assets` offers Legion's player
models as an upgrade marked unproven, and nobody has confirmed them on screen.
A character model the target has *never had* is a different thing - the naga in
a model pack are NPCs no player race wears - and is judged on its textures like
anything else.

**A conversion pack is not the same as a later client.** "WoW Legion Models for
WotLK 3.3.5a" and packs like it ship MPQ patch archives whose models have
already been converted: 548 of its 551 models are version 264, plain WotLK
MD20, and they load as they are. Point the extractor at the folder holding the
`Patch-*.MPQ` files with `--expansion wotlk` and it reads them like any other
patch chain. The CASC and MD21 limits above are about a *raw* Legion
installation, and they say nothing about what a converter has already done.

## Doing it

`wowee_assets` does this in one step, as its "Cataclysm's trees and doodads"
upgrade: it reads a Cataclysm installation's MPQs directly, takes a model under
`world/` only where Cataclysm's has more than 30% more vertices, and writes it
into the extraction rather than into `override/`. See `docs/asset-manager.md`.

By hand it is three steps. The first two are once per client, the third per
pack.

**1. Extract the art you want from the later client.** Point the extractor at
its `Data` directory. `--include` keeps this to seconds rather than unpacking a
whole client:

```sh
./build/bin/asset_extract \
    --mpq-dir /path/to/4.3.4/Data \
    --output /tmp/cata --expansion-subdir --skip-dbc \
    --include world/azeroth/stranglethorn
```

The expansion is detected from the archive set. 4.3.4 is `cata`; it extracts for
its art and cannot be played - see `docs/plan-cataclysm.md` for what that would
take.

**2. Build a pack from the extraction.** The pack is checked against the
expansion it is for, so it knows which of its files replace something and which
add something new:

```sh
python3 tools/asset_pack_from_client.py \
    --from /tmp/cata/expansions/cata \
    --against Data/expansions/wotlk \
    --include stranglethorn/passivedoodads/trees \
    --name cata-stranglethorn-trees
```

Each model is walked for what it draws - its skins, its animations, every
texture its header names, and for a `.wmo` its group files and their texture
list. A model is dropped only when the texture it is *drawn with* - its first slot,
and then only when that slot names its own art - is in neither the pack nor the
target. Everything else is kept: a pack of changed files is not a client, and
most of what its models draw is already in the expansion they are going over.
Checking the pack alone dropped 164 of 353 creature models whose "missing" art
was the base game's own. A reflection or a glow in that slot does not count
either, for reasons `asset_pack_curate.py` measured the hard way.

The result is a folder: `pack.json` beside a `Data/` tree. `pack.json` records
where it came from, what it was built against, and every file with its hash and
whether it replaces or adds.

**3. Install it.**

```sh
python3 tools/asset_pack_from_client.py \
    --install asset_packs/cata-stranglethorn-trees \
    --against ~/Library/Application\ Support/Wowee/Data/expansions/wotlk
```

The files land in the expansion's `override/` directory, which
`AssetManager::resolveFile` reads before the manifest. The extraction itself is
never touched, and `--uninstall` takes out exactly the files the pack listed.

`wowee_assets` has packs of its own, and they are not this kind: "Save what I
have as a pack" zips the whole destination, every game in it, and "Install a
pack..." unpacks such a `.zip` back over a destination at the paths it holds,
not into `override/`.

## How it resolves, and why additions used to fail

`resolveFile` tries, in order:

1. `override/<the manifest's path for this file>` - a replacement
2. `override/<the path itself>` - an addition, which has no manifest entry
3. the expansion's own manifest
4. the root `Data/` extraction's manifest, when an expansion's own directory is
   the one in use
5. a loose file under the data path

Step 2 is the one that lets a pack add. Cataclysm's tree needs
`STRANGLETHORNTREE_LEAVES_SET_COLOR.BLP`, a name 3.3.5 never had, so there is no
manifest entry to hang an override on. Without step 2 the model was reachable
and its own texture was not, and it rendered white. `tests/test_override_add.cpp`
pins the order.

## What it looks like on a real pack

A model pack of 20,014 files, built against a 3.3.5a install:

| include | models | files | dropped |
|---|---|---|---|
| `creature/` | 353 | 3732 | 1 |
| `world/` | 164 | 496 | 4 |
| `character/` | 2 | 0 | 2 refused as replacements |

The four dropped world models are the real thing rather than a false positive:
`stormwindlionbanner.m2` is drawn with
`world/expansion06/doodads/stormwind/7sw_stormwind_funerarybanner.blp`, a Legion
texture the pack did not ship, and it would have rendered white.

## Two packs, one directory

Packs are laid down file by file rather than owning `override/`, so several can
live there. Where two carry the same path the later install wins, and the
installer says so. Removing one leaves the other intact.

## Handing it to something that is not WoWee

WoWee reads loose files and never opens a patch archive, so the override
directory is all it needs. `mpq_build` packs that directory into one:

```sh
./build/bin/mpq_build --input <expansion>/override --output Patch-W.MPQ
```

Names inside the archive are the paths relative to the input directory,
backslashed and lowercased, with a `(listfile)` written so the contents can be
found by something other than guesswork. MPQ v1 by default, which is what a
3.3.5a client reads.

That is for the cases a directory cannot serve: giving the same art to a client
that only reads archives, standing beside the `Patch-*.MPQ` files a server's
players already install, or keeping a whole overlay as one file with its paths
intact. `asset_extract` reads back what it writes, which is how to check one.

## Reading a CASC installation

A Legion install is 43GB of `data.###` archives with a bucket index beside
them, and four lookups deep: the build config names a root file and an encoding
file by content key, encoding maps a content key to an encoding key, the `.idx`
buckets map an encoding key to an offset inside an archive, and every entry is
BLTE - a container of independently compressed chunks.

One step, in the window: tick one of the Legion upgrades and point the
installation box beside it at the Legion installation.

```sh
wowee_assets <your 3.3.5 install> "/path/to/World of Warcraft - Legion"
```

There is no sweep to run first. The importer asks the later installation for
each model already here, by that model's own path - CASC stores no filenames,
only a hash of the path, and a path already on disk is a path already known.
Matching on the name recorded inside each model is what the sweep was for, and
it does not work: Legion's earth elemental calls itself `ElementalEarth2` and
lives at `elementalearth.m2`, so the two never meet.

Each upgrade names the subtrees it takes - creature, world and item for the
creatures and doodads, character for the player models - and a model is taken
only where the later one is meaningfully bigger than the one already there. The
installation being built supplies both that comparison and the place each file
goes: CASC knows a FileDataID and not a path, so without it nothing could be
written at all.

Out of a Legion installation that is 1191 models in about five seconds, every
one of them resolved whole before anything is written - its skins, the
animations an `AFID` chunk names, and every texture it names for itself. The
output lands in the destination the client reads from, and nothing is written
into either game installation.

Two things about CASC shape everything else.

**It holds no filenames.** The root maps a FileDataID to a content key and a
64-bit hash of the path; the names themselves live in a community listfile that
is not part of an install. So a path cannot be listed out of one, only asked
for - `jenkins96()` is that question. A model's path is known because the
same model is already in the extraction; its skins and animations are named
inside it by FileDataID, through `SFID` and `AFID`, and fetched by id.

**A local install may not have a `.build.info`.** The one this was written
against does not, so the build config is found by reading `data/config` and
taking the file that declares a root and an encoding.

The `.idx` header is a `uint16` followed by six single bytes. Read as though
the second field were also 16 bits, every key comes out thirty bytes long and
every archive number is astronomical, which is the shape of that mistake.

### Converting a model

For a model that already existed in Wrath the format barely moved. Legion wraps
it in an `MD21` chunk whose internal offsets are relative to the chunk, so
lifting the chunk out gives a standalone MD20; the header's arrays sit where
they sat in 264, field for field - `BladesedgeSkyBox` has the same 17-byte
name, 7 global sequences, 4 bones, 1976 vertices and 15 textures in both
clients; and it still names its textures inline. So the conversion is to unwrap
it, stamp the version this client expects, and fetch the skins named by the
`SFID` chunk, the animations named by the `AFID` chunk, and the textures named
inside.

What cannot be finished is refused, before anything is written rather than
after - a half-written model is worse than none, because it is on disk, it
looks complete, and it renders white where a texture never arrived:

- **Particle and ribbon emitters.** Those two structs grew after Wrath. This is
  the common refusal: 63 of the 241 improved models below carry one.
- **A texture slot the client cannot fill.** A type 0 slot with no name is a
  model naming its own texture by FileDataID through a `TXID` chunk this does
  not read, and the client draws it flat white. Refused only where one of the
  model's batches actually samples that slot. The creature and character skin
  slots are empty by design and filled from the DBCs, so they do not count.
- **More monster-skin slots than the model it replaces.** A creature is dressed
  from its `CreatureDisplayInfo` row, which names as many skins as the shipped
  model wanted; Legion's boar carries a second slot for its mane that nothing
  can fill. 46 of 784 imported models.
- **A skin or a texture that does not arrive.** A model without its index data
  draws as a burst of spikes from the origin.

**Where the model goes is not in the archive.** CASC knows a FileDataID, not a
path, and the client looks for a model at a path. That placement comes from the
extraction being built - which is also what the model is being compared
against - so it is taken from there rather than guessed.

### Finding what is worth taking

Compare vertex counts by model name, against a baseline with `override/`
excluded so an already-installed pack does not hide its own source:

| against 3.3.5a | shared | identical | richer |
|---|---|---|---|
| Cataclysm | 22033 | 20992 | 205 |
| Legion | 15627 | 14451 | 241 |

The shape of it is that old art is not re-authored: better than nine in ten
shared models have the identical vertex count in both later clients. What is
left is worth having, though, and the two clients do not overlap much - 94
models are richer in both, 147 only in Legion, 111 only in this Cataclysm
extract.

Legion's are Teldrassil's canopy at 350 vertices to 2584, Tirisfal's graves at
1236 to 6964, Winterspring's trees at 422 to 2584, Booty Bay's at 148 to 924;
signs, benches, bookstacks, banners and statues; and the holiday doodads, which
were re-cut repeatedly. Of the 241, **165 convert as they are**.

Take the whole of a thing rather than part of it - see
[What to expect](#what-to-expect) - and check a family before taking it: a
model can be richer and still be the same shape, and one that is the same
vertex count is certainly not worth moving.

### What a later client cannot supply

Most of the *art* is there. Almost none of what a client needs to run is.
Sampling 400 real paths from each top-level directory of a 3.3.5a installation
and asking a Legion one for them:

| | files | in Legion |
|---|---|---|
| `spells`, `environments`, `xtextures` | 4785 | 100% |
| `item` | 43855 | 99.5% |
| `tileset` | 2366 | 99.2% |
| `dungeons` | 4328 | 98.5% |
| `character` | 11907 | 97.8% |
| `world` | 48862 | 97.2% |
| `interface` | 14672 | 97.0% |
| `creature` | 11594 | 96.8% |
| `textures` | 36162 | 34.8% |
| `shaders` | 572 | 18.5% |
| `sound` | 21065 | 5.2% |
| `dbfilesclient` | 245 | **0%** |

Three of those are hard stops.

**No DBC at all.** Not one of the 245 tables this client reads exists as a
`.dbc`; 126 exist as `.db2`, a different container with Legion's own schemas
and Legion's own ids, and the other 119 do not exist in any form. Nothing
starts without `Map`, `AreaTable`, `ChrRaces` and `CreatureDisplayInfo`, and
converted ids would not be the ids a 3.3.5 server sends anyway.

**The terrain is there by path and is not the same terrain.** Cataclysm split
the ADT: Legion's `azeroth_31_49.adt` holds `MVER MHDR MH2O MCNK` and nothing
else, while `MTEX`, `MMDX`, `MWMO`, `MDDF` and `MODF` moved into sibling
`_tex0` and `_obj0`/`_obj1` files. `src/pipeline/adt_loader.cpp` reads the
monolithic form, so it would find no textures, no doodads and no buildings.
Underneath that, Azeroth and Kalimdor are post-Cataclysm geography that will
not match a 3.3.5 server's collision or its quest positions - and Outland and
Northrend differ too, from the format split alone.

**Sound is effectively gone.** 22 of 300 sampled files are present, and the
survivors are ambience `.mp3`; the `.wav` files - creature sounds, weapon
impacts, NPC voice - were re-encoded and re-pathed.

So a later installation is an art source, not a client. The boundary is worth
stating plainly because the art coverage is high enough to suggest otherwise.

### What the light tables say

`tools/db2_read.py` reads WDC1, the DB2 that replaced DBC, which is how to ask
a later client where it actually uses a skybox. Its `LightSkybox` names the
model by FileDataID rather than by path, and `LightParams` keeps its own id
bit-packed into the tail of each record - the field structure reports that
column as having no bits at all.

Three measurements worth having before borrowing any sky:

**Skyboxes are not re-authored between expansions.** Every one of the 52 this
client ships exists in Cataclysm and in Legion, and the Cataclysm copies are
the same models with the same textures at the same resolutions - identical
vertex counts, identical texture lists, file sizes differing by the version
field alone. There is no like-for-like upgrade to take.

**A later client does not re-sky the old world either.** Of 64 light volumes
that sit at the same position in Wrath and in Legion, 8 differ, and all 8 are
instance maps: the Ulduar family moved to a layered sky and five indoor
instances dropped theirs. What Legion adds to Eastern Kingdoms and Kalimdor is
Cataclysm art - Twilight Highlands, Hyjal, Firelands, the Lost Isles - which is
new zones carrying their own skies, not replacements for old ones. Legion's own
skies stay on the Broken Isles and Argus.

**Some later skies are two domes.** Legion's `LightSkybox` carries a second
file id, and the six rows that use one name a `_Layer02` model - Suramar City,
Highmountain, Hyjal, Ulduar, the Valley of Eternal Blossoms, and Argus. No
`_Layer02` is ever named as a primary. `LightSkybox.dbc` here has one model
column, so taking the `_Layer01` half of a layered sky on its own gets half the
sky. The unlayered ones do not have this problem.

So the value in a later client's skies is the ones this one has never had, put
somewhere by hand, and not an upgrade to what is already there. Legion carries
195 of them and 191 convert, which is a lot of sky for zones that have to be
chosen one at a time.

## What to expect

Placements are still your client's. A Cataclysm tree stands where the 3.3.5 ADT
put a tree, which for the same species at the same scale looks right, and for a
model the later client also resized may not. Zone lighting, terrain textures and
the doodads you did not swap are unchanged - swapping a zone's canopy and not its
undergrowth is visible, so take the whole of a thing rather than part of it.
