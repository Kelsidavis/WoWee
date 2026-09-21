# Troubleshooting Guide

This guide covers common issues and solutions for WoWee.

## Connection Issues

### "Authentication Failed"
- **Cause**: Incorrect server address, expired realm list, or version mismatch
- **Solution**:
  1. Verify the server chosen in the login card's **Server** list is the one you mean. For a server that is not listed, choose **Somewhere else...** and enter the address and port under **more options**
  2. Ensure the expansion under **more options** matches the server (Vanilla/TBC/WotLK/Turtle), and that your extracted data is for that expansion
  3. Check that the emulator server is running and reachable

### "Realm List Connection Failed"
- **Cause**: Server is down, firewall blocking connection, or DNS issue
- **Solution**:
  1. Verify server IP/hostname is correct
  2. Test connectivity: `ping realm-server-address`
  3. Check firewall rules for port 3724 (auth and realm list) and the realm's world port (8085 by default)
  4. Try using IP address instead of hostname (DNS issues)
  5. On a LAN where the realm advertises a public address your router cannot reach from inside, set `WOWEE_REALM_HOST_OVERRIDE` to the world server's local address; the realm's port is kept

### "Connection Lost During Login"
- **Cause**: Network timeout, server overload, or incompatible protocol version
- **Solution**:
  1. Check your network connection
  2. Reduce number of assets loading (lower graphics preset)
  3. Verify server supports this expansion version

## Startup Issues

### The Client Opens the Asset Builder Instead of the Login Screen
- **Cause**: No extracted assets were found. The client looks at `WOW_DATA_PATH` if set, then the per-user data directory (`~/Library/Application Support/Wowee/Data` on macOS, `%LOCALAPPDATA%\Wowee\Data` on Windows, `$XDG_DATA_HOME/wowee/Data` or `~/.local/share/wowee/Data` on Linux), then `./Data`
- **Solution**:
  1. Build the assets from the screen that opened, or with `wowee_assets` or `extract_assets.sh` (see [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md))
  2. Restart the client after a build: the asset manager, DBC tables and loaders are set up once at startup
  3. The log says which asset sets it found (`Assets: ...`), or names the directory it looked in when there are none
  4. A client built without StormLib has no builder; it shows the login screen with that message on the card instead

### "Failed to select Vulkan physical device"
- **Cause**: No GPU driver offering Vulkan 1.3. The client requires 1.3 and refuses a device that reports only 1.2
- **Solution**:
  1. The log lists every device the loader offered, with its Vulkan version and whether it can draw and present
  2. Update the GPU driver (or Mesa) to one that reports Vulkan 1.3

## Graphics Issues

### "VK_ERROR_DEVICE_LOST" or Client Crashes
- **Cause**: GPU driver issue, insufficient VRAM, or graphics feature incompatibility
- **Solution**:
  1. **Immediate**: Lower graphics settings:
     - Press Escape → Video → Graphics
     - Set the quality preset to **Low**
     - Set Upscaling to **Off**
     - Set Anti-aliasing (MSAA) to **Off**
  2. **Medium term**: Update GPU driver to latest version
  3. **Verify**: Use a graphics test tool to ensure GPU stability
  4. **If persists**: The log names the first operation that saw the device lost, and where the driver supports them, the fault address and the last pass each queue reached. Include it in the report. These switches narrow it down, one at a time:
     - `WOWEE_VK_SYNC_UPLOAD_ON_GRAPHICS=1` runs the synchronous upload batches on the graphics queue instead of a second queue
     - `WOWEE_VK_NO_ROBUST_BUFFERS=1` turns off robust buffer access (on by default)
     - `WOWEE_VK_NO_DYNAMIC_RENDERING=1` puts the shadow pass back on its render pass

### Black Screen or Rendering Issues
- **Cause**: Missing shaders, GPU memory allocation failure, or incorrect graphics settings
- **Solution**:
  1. Check logs: Look in `logs/wowee.log` (see [Check Logs](#check-logs)) for error messages
  2. Verify shaders compiled: Check for `.spv` files in `assets/shaders/`
  3. Reduce shadow distance: Press Escape → Video → Graphics → lower Shadow distance from 300 yards to 100
  4. Shadows cannot be switched off: turning them off loses the device, so the control was removed and shadows are held on. Shadow distance is the setting that reduces their cost
  5. If shadows specifically look wrong, run once with `WOWEE_VK_NO_DYNAMIC_RENDERING=1`; the log says which path the shadow pass used

### Low FPS or Frame Stuttering
- **Cause**: Too high graphics settings for your GPU, memory fragmentation, or asset loading
- **Solution**:
  1. Apply lower graphics preset: Escape → Video → Graphics → Low or Medium
  2. Disable MSAA: Set to "Off"
  3. Reduce View distance on the Graphics page
  4. Close other applications consuming GPU memory
  5. Check CPU usage - if high, reduce number of visible entities
  6. For a per-pass cost breakdown, run with `WOWEE_PASS_ABLATION=1`: it switches each pass off for a few seconds and reports what each was worth

### Water/Terrain Flickering
- **Cause**: Shadow mapping artifacts, terrain LOD issues, or GPU memory pressure
- **Solution**:
  1. Adjust shadow distance (150 to 200 yards)
  2. Check GPU memory usage

## Audio Issues

### No Sound
- **Cause**: Audio initialization failed, missing audio data, or incorrect mixer setup
- **Solution**:
  1. Check system audio is working: Test with another application
  2. Verify audio files extracted: Check for a `sound/` directory under `Data/expansions/<expansion>/`
  3. Unmute audio: Click the speaker button beside the minimap
  4. Check settings: Escape → Sound & Voice → Sound → Master volume > 0, and **Mute all sound** off

### Sound Cutting Out
- **Cause**: Audio buffer underrun, too many simultaneous sounds, or driver issue
- **Solution**:
  1. Lower audio volume: Escape → Sound & Voice → Reduce Master volume
  2. Disable distant ambient sounds: Reduce Ambience volume
  3. Reduce number of particle effects
  4. Update audio driver

## Gameplay Issues

### Character Stuck or Not Moving
- **Cause**: Network synchronization issue, collision bug, or server desync
- **Solution**:
  1. Try pressing Escape to deselect any target, then move
  2. Jump (Spacebar) to test physics
  3. Reload the character: Press Escape → Logout, then log back in
  4. Check for transport/vehicle state: click the mount you are riding on the action bar to dismount if applicable

### Spells Not Casting or Showing "Error"
- **Cause**: Cooldown, mana insufficient, target out of range, or server desync
- **Solution**:
  1. Verify spell is off cooldown (action bar shows availability)
  2. Check mana/energy: Look at player frame (top-left)
  3. Verify target range: Hover action bar button for range info
  4. Check server logs for error messages (combat log will show reason)

### Quests Not Updating
- **Cause**: Objective already completed in different session, quest giver not found, or network desync
- **Solution**:
  1. Check quest objective: Open quest log (L key) → Verify objective requirements
  2. Re-interact with NPC to trigger update packet
  3. Reload character if issue persists

### Items Not Appearing in Inventory
- **Cause**: Inventory full, item filter active, or network desync
- **Solution**:
  1. Check inventory space: Open inventory (B key) → Count free slots
  2. Verify item isn't already there: Search inventory for item name
  3. Check if bags are full: Open bag windows, consolidate items
  4. Reload character if item is still missing

### Interface Options Empty or Dead (Turtle WoW)

The Video, Interface and Audio option panels open but list nothing, and the log
carries `OptionsFrame.lua:424: attempt to index field 'text' (a nil value)`
followed by `VideoOptionsFrame`, `AudioOptionsFrame` and several
`OptionsFrame*ScrollFrame*` names reported as missing APIs.

- **Cause**: A packaging fault in the Turtle client, not in this one. Turtle
  back-ports `Interface\FrameXML\OptionsFrameTemplates.xml` — the file that
  declares `OptionsListButtonTemplate` — into `patch-4.mpq`, but no
  `FrameXML.toc` in the client lists it, so it never loads. Turtle's own
  `OptionsFrame.xml` inherits that template eighteen times, so the category
  buttons are built without the `text` and `bar` regions the template would
  have given them, and the first read of `button.text` raises. The raise aborts
  the rest of that file's frame build, which is where the other missing names
  come from - one cause, not several.
- **Solution**:
  1. Add `OptionsFrameTemplates.xml` to `Interface\FrameXML\FrameXML.toc`
     ahead of `OptionsFrame.xml`, in whichever patch archive supplies the toc
  2. Or report it upstream to Turtle - the real 1.12 client resolves a template
     when the frame is built, exactly as this one does, so it has the same
     nothing to inherit and the same failure
  3. Nothing else in the options framework needs changing; the template is
     present on disk and correct

See [#132](https://github.com/Kelsidavis/WoWee/issues/132).

## Performance Optimization

The quality preset on the Graphics page sets these values. Upscaling is a
separate page and no preset changes it.

### For Low-End GPUs
```
Graphics Preset: LOW
- View distance: 600 yards
- Shadow distance: 100 yards
- MSAA: OFF
- Normal Mapping: Disabled
- Surface depth (parallax): Disabled
- Ground clutter: 25%
```

### For Mid-Range GPUs
```
Graphics Preset: MEDIUM
- View distance: 1000 yards
- Shadow distance: 200 yards
- MSAA: 2x
- Normal Mapping: On (0.6 strength)
- Ground clutter: 60%
- Upscaling: FSR 1 or FSR 3 (optional)
```

### For High-End GPUs
```
Graphics Preset: HIGH or ULTRA
- View distance: 1600-2400 yards
- Shadow distance: 350-500 yards
- MSAA: 4-8x
- Normal Mapping: On (0.8-1.2 strength)
- Ground clutter: 100-150%
- Grass: On with Ultra
- Upscaling: FSR 3 (optional, for 4K)
```

## Getting Help

### Check Logs
Logs are written to `logs/wowee.log` in the working directory (typically `build/bin/` for a local build). Where that directory cannot be written, the log goes to a per-user directory instead and the console says where:
- macOS: `~/Library/Logs/Wowee/`
- Linux: `$XDG_STATE_HOME/wowee/logs/` or `~/.local/state/wowee/logs/`
- Windows: `%LOCALAPPDATA%\Wowee\logs\`

The log records warnings and errors by default. Run with `WOWEE_LOG_LEVEL=info` (or `debug`) for more detail.

Include relevant log entries when reporting issues. The first lines name the build, platform and GPU driver.

### Check Server Compatibility
- **AzerothCore**: Full support
- **TrinityCore**: Full support
- **Mangos**: Full support
- **Turtle WoW**: Full support (1.18)

### Report Issues
If you encounter a bug:
1. Find the log (see [Check Logs](#check-logs))
2. Reproduce the issue consistently
3. Gather system info: GPU, driver version, OS
4. Check if issue is expansion-specific (Classic/TBC/WotLK)
5. Report with detailed steps to reproduce

### Clear Cache
If experiencing persistent Warden issues, clear WoWee's Warden module cache:
```bash
# Linux/macOS
rm -rf ~/.local/share/wowee/warden_cache/

# Windows
rmdir %APPDATA%\wowee\warden_cache\ /s
```

Then restart WoWee to rebuild cache.
