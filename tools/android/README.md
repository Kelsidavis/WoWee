# Android data profiles

A full extraction is about 18 GB. A phone does not need it: reaching the login
screen and creating a character touches 4% of it.

`make_minimal_data.py` cuts an extracted `Data` directory down to a profile.
`data_profiles.json` defines the profiles. Nothing here extracts anything; run
`extract_assets.sh` first, as on desktop, then cut the result down.

```
tools/android/make_minimal_data.py --source ~/Data --profile login --dry-run
tools/android/make_minimal_data.py --source ~/Data --out ~/Data-login --profile login
adb push ~/Data-login/. /sdcard/Android/data/com.wowee.client/files/Data/
adb shell chmod -R a+rwX /sdcard/Android/data/com.wowee.client/files/Data
```

The `chmod` is needed after every push: what `adb push` writes there belongs to
the shell user, and the app cannot open its directories until they are opened up.

Files are hard linked where the filesystem allows it, so a subset beside its
source costs almost nothing. `--copy` for a different filesystem.

## The profiles

Measured against a 199,468 file WotLK extraction, 18.2 GB.

| Profile | Size | What it is for |
|---|---|---|
| `login` | 890 MB | Login screen, character select, character creation. No world. |
| `world --maps azeroth` | 7.3 GB | One continent. `--maps azeroth` by default. |
| `world --maps all` | 11.8 GB | Every map |
| `full` | 18.0 GB | Everything a client reads. Drops the macOS bundles, the manual and the intro movie. |

`world` is the honest number, not a target. It is dominated by `world/wmo`,
`creature` and `item`, which are shared across zones and cannot be narrowed by
map without walking each ADT for the models it references. That walk is the way
to a genuinely small playable set and has not been written.

Neither `login` nor `world` carries any sound. The client names 265 sound files
in its own source, and `sound/` is 6 GB.

The map names are the directories under `world/maps/`: `azeroth`, `kalimdor`,
`northrend`, `expansion01`, `expansion02`, `emeralddream` and the rest. `--maps
all` keeps every one.

## Where the space goes

Measured on the same extraction:

| Directory | Size | Files |
|---|---|---|
| `world/` | 7.5 GB | 47,683 |
| `sound/` | 6.0 GB | 21,065 |
| `textures/` | 1.1 GB | 36,162 |
| `creature/` | 1.0 GB | 11,590 |
| `item/` | 1019 MB | 43,853 |
| `interface/` | 477 MB | 14,662 |
| `character/` | 362 MB | 11,907 |
| `dungeons/` | 233 MB | 4,264 |
| `tileset/` | 145 MB | 2,364 |
| `dbfilesclient/` | 86 MB | 245 |

The four world maps are most of `world/`: kalimdor 1.1 GB, northrend 1.1 GB,
azeroth 737 MB, expansion01 721 MB.

Sound is 6 GB and it is not prunable by naming: the client reads 286 sound
files by name, but `SoundEntries.dbc` names 20,638 more at runtime, and between
them they reach 20,567 of the 21,065 files. Dropping sound drops 6 GB; keeping
it costs 6 GB.

## Why a subset needs its own manifest

The client locates its data root by finding `manifest.json`, and the asset
manager resolves every path through it. Copying a subtree without rewriting the
manifest produces a directory the client either cannot find or believes is full
of files that are not there. `make_minimal_data.py` writes a manifest describing
exactly what it kept.

The expansion profiles under `Data/expansions` and `Data/opcodes` come from this
repository rather than from anyone's MPQs and are not in the manifest, so they
are copied separately.

## Checking a profile

```
tools/android/check_profile_coverage.py --source ~/Data --profile login
```

This reads every asset path the client spells out, in `src/` and in the
interface's own Lua and XML, resolves each through the manifest the way the
asset manager does, and reports the ones the profile would drop. A drop from a
directory the profile never carried is the profile working. A drop from one it
does carry is a missing texture at runtime, and fails.

An exclusion that removes something the client names has to say why, in
`exclude_reasons`. An unexplained one is reported as a hole, because a rule
nobody justified reads exactly like a rule nobody noticed.

It reads literals, so it cannot see a path the client assembles from a DBC row
at runtime. It narrows the risk; it does not remove it. What removes it is
running the client, which for Android needs a device.

## What was checked

- All three profiles: nothing they carry is missing a path the client names.
- The `login` subset loads the interface identically to the full extraction.
  `framexml_run <data root>` against each gives 139 files, 0 failed, and the
  same missing-API counts, differing only in paths and timings.
- `test_data_profiles.py` runs as the `android_data_profiles` ctest.
