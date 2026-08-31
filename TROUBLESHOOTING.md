# Troubleshooting Guide

This guide covers common issues and solutions for WoWee.

## Connection Issues

### "Authentication Failed"
- **Cause**: Incorrect server address, expired realm list, or version mismatch
- **Solution**:
  1. Verify server address in realm list is correct
  2. Ensure your WoW data directory is for the correct expansion (Vanilla/TBC/WotLK)
  3. Check that the emulator server is running and reachable

### "Realm List Connection Failed"
- **Cause**: Server is down, firewall blocking connection, or DNS issue
- **Solution**:
  1. Verify server IP/hostname is correct
  2. Test connectivity: `ping realm-server-address`
  3. Check firewall rules for port 3724 (auth) and 8085 (realm list)
  4. Try using IP address instead of hostname (DNS issues)

### "Connection Lost During Login"
- **Cause**: Network timeout, server overload, or incompatible protocol version
- **Solution**:
  1. Check your network connection
  2. Reduce number of assets loading (lower graphics preset)
  3. Verify server supports this expansion version

## Graphics Issues

### "VK_ERROR_DEVICE_LOST" or Client Crashes
- **Cause**: GPU driver issue, insufficient VRAM, or graphics feature incompatibility
- **Solution**:
  1. **Immediate**: Disable advanced graphics features:
     - Press Escape → Video Settings
     - Set graphics preset to **LOW**
     - Disable Water Refraction (requires FSR)
     - Disable MSAA (set to Off)
  2. **Medium term**: Update GPU driver to latest version
  3. **Verify**: Use a graphics test tool to ensure GPU stability
  4. **If persists**: Try FSR2 disabled mode - check renderer logs

### Black Screen or Rendering Issues
- **Cause**: Missing shaders, GPU memory allocation failure, or incorrect graphics settings
- **Solution**:
  1. Check logs: Look in `logs/wowee.log` (relative to the working directory, typically `build/bin/`) for error messages
  2. Verify shaders compiled: Check for `.spv` files in `assets/shaders/`
  3. Reduce shadow distance: Press Escape → Video Settings → Lower shadow distance from 300m to 100m
  4. Disable shadows entirely if issues persist

### Low FPS or Frame Stuttering
- **Cause**: Too high graphics settings for your GPU, memory fragmentation, or asset loading
- **Solution**:
  1. Apply lower graphics preset: Escape → LOW or MEDIUM
  2. Disable MSAA: Set to "Off"
  3. Reduce draw distance: Move further away from complex areas
  4. Close other applications consuming GPU memory
  5. Check CPU usage - if high, reduce number of visible entities

### Water/Terrain Flickering
- **Cause**: Shadow mapping artifacts, terrain LOD issues, or GPU memory pressure
- **Solution**:
  1. Increase shadow distance slightly (150m to 200m)
  2. Disable shadows entirely as last resort
  3. Check GPU memory usage

## Audio Issues

### No Sound
- **Cause**: Audio initialization failed, missing audio data, or incorrect mixer setup
- **Solution**:
  1. Check system audio is working: Test with another application
  2. Verify audio files extracted: Check for `.wav` files in `Data/Audio/`
  3. Unmute audio: Look for speaker icon in minimap (top-right) - click to unmute
  4. Check settings: Escape → Audio Settings → Master Volume > 0

### Sound Cutting Out
- **Cause**: Audio buffer underrun, too many simultaneous sounds, or driver issue
- **Solution**:
  1. Lower audio volume: Escape → Audio Settings → Reduce Master Volume
  2. Disable distant ambient sounds: Reduce Ambient Volume
  3. Reduce number of particle effects
  4. Update audio driver

## Gameplay Issues

### Character Stuck or Not Moving
- **Cause**: Network synchronization issue, collision bug, or server desync
- **Solution**:
  1. Try pressing Escape to deselect any target, then move
  2. Jump (Spacebar) to test physics
  3. Reload the character: Press Escape → Disconnect → Reconnect
  4. Check for transport/vehicle state: Press 'R' to dismount if applicable

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
  1. Check quest objective: Open quest log (Q key) → Verify objective requirements
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

### For Low-End GPUs
```
Graphics Preset: LOW
- Shadows: OFF
- MSAA: OFF
- Normal Mapping: Disabled
- Clutter Density: 25%
- Draw Distance: Minimum
- Particles: Reduced
```

### For Mid-Range GPUs
```
Graphics Preset: MEDIUM
- Shadows: 200m
- MSAA: 2x
- Normal Mapping: Basic
- Clutter Density: 60%
- FSR2: Enabled (if desired)
```

### For High-End GPUs
```
Graphics Preset: HIGH or ULTRA
- Shadows: 350-500m
- MSAA: 4-8x
- Normal Mapping: Full (1.2x strength)
- Clutter Density: 100-150%
- FSR2: Optional (for 4K smoothness)
```

## Getting Help

### Check Logs
Detailed logs are saved to `logs/wowee.log` in the working directory (typically `build/bin/`).

Include relevant log entries when reporting issues.

### Check Server Compatibility
- **AzerothCore**: Full support
- **TrinityCore**: Full support
- **Mangos**: Full support
- **Turtle WoW**: Full support (1.17)

### Report Issues
If you encounter a bug:
1. Enable logging: Watch console for error messages
2. Reproduce the issue consistently
3. Gather system info: GPU, driver version, OS
4. Check if issue is expansion-specific (Classic/TBC/WotLK)
5. Report with detailed steps to reproduce

### Clear Cache
If experiencing persistent issues, clear WoWee's cache:
```bash
# Linux/macOS
rm -rf ~/.wowee/warden_cache/
rm -rf ~/.wowee/asset_cache/

# Windows
rmdir %APPDATA%\wowee\warden_cache\ /s
rmdir %APPDATA%\wowee\asset_cache\ /s
```

Then restart WoWee to rebuild cache.
