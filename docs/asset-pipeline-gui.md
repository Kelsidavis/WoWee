# Asset Pipeline GUI

WoWee includes a Python GUI for extraction and texture-pack management:

```bash
python3 tools/asset_pipeline_gui.py
```

The script is also executable directly: `./tools/asset_pipeline_gui.py`

## Supported Platforms

- Linux
- macOS
- Windows

The app uses Python's built-in `tkinter` module. If `tkinter` is missing, install the platform package:

- Linux (Debian/Ubuntu): `sudo apt install python3-tk`
- Fedora: `sudo dnf install python3-tkinter`
- Arch: `sudo pacman -S tk`
- macOS: use the official Python.org installer (includes Tk)
- Windows: use the official Python installer and enable Tcl/Tk support

## What It Does

- Runs `asset_extract` (or shell/PowerShell script fallback) to extract MPQ data
- Saves extraction config in `asset_pipeline/state.json`
- Installs texture packs from ZIP or folders (with zip-slip protection)
- Lets users activate/deactivate packs and reorder active pack priority
- Rebuilds `Data/override` from active pack order (runs in background thread)
- Shows current data state (`manifest.json`, entry count, override file count, last runs)
- Browses extracted assets with inline previews (images, 3D wireframes, data tables, text, hex dumps)

## Configuration Tab

### Path Settings

| Field | Description |
|-------|-------------|
| **WoW Data (MPQ source)** | Path to your WoW client's `Data/` folder containing `.MPQ` files |
| **Output Data directory** | Where extracted assets land. Defaults to `<project root>/Data` |
| **Extractor binary/script** | Optional. Leave blank for auto-detection (see below) |

### Extractor Auto-Detection

When no extractor path is configured, the GUI searches in order:

1. `build/bin/asset_extract` — CMake build with bin subdirectory
2. `build/asset_extract` — CMake build without bin subdirectory
3. `bin/asset_extract` — standalone binary
4. **Windows only**: `extract_assets.ps1` — invoked via `powershell -ExecutionPolicy Bypass -File`
5. **Linux/macOS only**: `extract_assets.sh` — invoked via `bash`

On Windows, `.exe` is appended to binary candidates automatically.

### Extraction Options

| Option | Description |
|--------|-------------|
| **Expansion** | `auto`, `classic`, `turtle`, `tbc`, or `wotlk`. Read-only dropdown. |
| **Locale** | `auto`, `enUS`, `enGB`, `deDE`, `frFR`, etc. Read-only dropdown. |
| **Threads** | Worker thread count. 0 = auto (uses all cores). |
| **Skip DBC extraction** | Skip database client files (faster if you only want textures). |
| **Generate DBC CSV** | Output human-readable CSV alongside binary DBC files. |
| **Verify CRC** | Check file integrity during extraction (slower but safer). |
| **Verbose output** | More detail in the Logs tab. |

### Buttons

| Button | Action |
|--------|--------|
| **Save Configuration** | Writes all settings to `asset_pipeline/state.json`. |
| **Run Extraction** | Starts the extractor in a background thread. Output streams to the Logs tab. |
| **Cancel Extraction** | Terminates a running extraction. Grayed out when idle, active during extraction. |
| **Refresh State** | Reloads the Current State tab. |

## Texture Packs Tab

### Installing Packs

- **Install ZIP**: Opens a file picker for `.zip` archives. Each member path is validated against zip-slip attacks before extraction.
- **Install Folder**: Opens a folder picker and copies the entire folder into the pipeline's internal pack storage.

### Managing Packs

| Button | Action |
|--------|--------|
| **Activate** | Adds the selected pack to the active override list. |
| **Deactivate** | Removes the selected pack from the active list (stays installed). |
| **Move Up / Move Down** | Changes priority order. Pack #1 is the base layer; higher numbers override lower. |
| **Rebuild Override** | Merges all active packs into `Data/override/` in a background thread. UI stays responsive. |
| **Uninstall** | Removes the pack from disk after confirmation. |

Pack list selection is preserved across refreshes — you can activate a pack and immediately reorder it without re-selecting.

## Pack Format

Supported pack layouts:

1. `PackName/Data/...`
2. `PackName/data/...`
3. `PackName/...` where top folders include game folders (`Interface`, `World`, `Character`, `Textures`, `Sound`)
4. Single wrapper directory containing any of the above

When multiple active packs contain the same file path, **later packs in active order win**.

## Asset Browser Tab

Browse and preview every extracted asset visually. Requires a completed extraction with a `manifest.json` in the output directory.

### Layout

- **Top bar**: Search entry, file type filter dropdown, Search/Reset buttons, result count
- **Left panel** (~30%): Directory tree built lazily from `manifest.json`
- **Right panel** (~70%): Preview area that adapts to the selected file type
- **Bottom bar**: File path, size, and CRC from manifest

### Search and Filtering

Type a substring into the search box and/or pick a file type from the dropdown, then click **Search**. The tree repopulates with matching results (capped at 5000 entries). Click **Reset** to restore the full tree.

File type filters: All, BLP, M2, WMO, DBC, ADT, Audio, Text.

### Preview Types

| Type | What You See |
|------|--------------|
| **BLP** | Converted to PNG via `blp_convert --to-png`, displayed as an image. Cached in `asset_pipeline/preview_cache/`. |
| **M2** | Wireframe rendering of model vertices and triangles on a Canvas. Drag to rotate, scroll to zoom. |
| **WMO** | Wireframe of group geometry (MOVT/MOVI chunks). Root WMOs auto-load the `_000` group file. |
| **CSV** (DBC exports) | Scrollable table with column names from `dbc_layouts.json`. First 500 rows loaded, click "Load more" for the rest. |
| **ADT** | Colored heightmap grid parsed from MCNK chunks. |
| **Text** (XML, LUA, JSON, HTML, TOC) | Syntax-highlighted scrollable text view. |
| **Audio** (WAV, MP3, OGG) | Metadata display — format, channels, sample rate, duration (WAV). |
| **Other** | Hex dump of the first 512 bytes. |

### Wireframe Controls

- **Left-click drag**: Rotate the model (azimuth + elevation)
- **Scroll wheel**: Zoom in/out
- Depth coloring: closer geometry renders brighter

### Optional Dependencies

| Dependency | Required For | Fallback |
|------------|-------------|----------|
| [Pillow](https://pypi.org/project/Pillow/) (`pip install Pillow`) | BLP image preview | Shows install instructions |
| `blp_convert` (built with project) | BLP → PNG conversion | Shows "not found" message |

All other previews (wireframe, table, text, hex) work without any extra dependencies.

### Cache

BLP previews are cached as PNG files in `asset_pipeline/preview_cache/` keyed by path and file size. Delete this directory to clear the cache.

## Current State Tab

Shows a summary of pipeline state:

- Output directory existence and `manifest.json` entry count
- Override folder file count and last build timestamp
- Installed and active pack counts with priority order
- Last extraction time, success/failure, and the exact command used
- Paths to the state file and packs directory

Click **Refresh** to reload, or it auto-refreshes after operations.

## Logs Tab

All extraction output, override rebuild messages, cancellations, and errors stream here in real time via a log queue polled every 120ms. Click **Clear Logs** to reset.

## State Files and Folders

| Path | Description |
|------|-------------|
| `asset_pipeline/state.json` | All configuration, pack metadata, and extraction history |
| `asset_pipeline/packs/<pack-id>/` | Installed pack contents (one directory per pack) |
| `asset_pipeline/preview_cache/` | Cached BLP → PNG conversions for the Asset Browser |
| `<Output Data>/override/` | Merged output from active packs |

The `asset_pipeline/` directory is gitignored.

## Typical Workflow

1. Launch: `python3 tools/asset_pipeline_gui.py`
2. **Configuration tab**: Browse to your WoW `Data/` folder, pick expansion, click **Save Configuration**.
3. Click **Run Extraction** — watch progress in the **Logs** tab. Cancel with **Cancel Extraction** if needed.
4. Switch to **Texture Packs** tab. Click **Install ZIP** and pick a texture pack.
5. Select the pack and click **Activate**.
6. (Optional) Install more packs, activate them, and use **Move Up/Down** to set priority.
7. Click **Rebuild Override** — the status bar shows progress, and the result appears in Logs.
8. (Optional) Switch to **Asset Browser** to explore extracted files — preview textures, inspect models, browse DBC tables.
9. Run wowee — it loads override textures on top of the extracted base assets.
