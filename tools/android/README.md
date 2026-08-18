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
```

Files are hard linked where the filesystem allows it, so a subset beside its
source costs almost nothing. `--copy` for a different filesystem.

## The profiles

Measured against a 199,468 file WotLK extraction, 18.1 GB.

| Profile | Size | What it is for |
|---|---|---|
| `login` | 787 MB, 23,072 files | Login screen, character select, character creation. No world. |
| `world` | 7.5 GB, 172,230 files | The above plus one continent. `--maps azeroth` by default. |
| `full` | 18.1 GB | Everything a client reads. Drops the macOS bundles and the manual. |

`world` is the honest number, not a target. It is dominated by `world/wmo`,
`creature` and `item`, which are shared across zones and cannot be narrowed by
map without walking each ADT for the models it references. That walk is the way
to a genuinely small playable set and has not been written.

Neither `login` nor `world` carries any sound. The client names 265 sound files
in its own source, and `sound/` is 6 GB.

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
