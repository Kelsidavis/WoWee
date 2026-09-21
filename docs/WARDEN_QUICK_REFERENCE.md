# Warden Quick Reference

Warden is WoW's client integrity checking system. Wowee runs the Warden module via Unicorn
Engine CPU emulation - no Wine required - and builds the check replies itself.

---

## How It Works

Warden modules are native x86 Windows DLLs that the server encrypts and delivers at login.

1. Server sends `SMSG_WARDEN_DATA` (0x2E6) naming the module; the client answers `MODULE_MISSING` and the server streams the encrypted module
2. Client verifies and decrypts: MD5 → RC4 → RSA-2048 signature verify → zlib decompress
3. Unpacks the module image: relocations applied, imports resolved (Windows API hooks)
4. Executes entry point via Unicorn Engine x86 emulator
5. Client responds with check results via `CMSG_WARDEN_DATA` (0x2E7), built by `WardenHandler` from a `WoW.exe` image and pre-computed `.cr` files

A signature mismatch does not stop the load. A realm that signs its own module can name its
key as `wardenRsaModulus` in the expansion profile.

---

## Server Compatibility

| Server type | Expected result |
|-------------|-----------------|
| Warden disabled | Works (no Warden packets) |
| AzerothCore (local) | Works |
| ChromieCraft | Works |
| Warmane | Should work |
| Turtle WoW-derived | Needs a `.cr` file for the module; without one `HASH_REQUEST` goes unanswered |

---

## Module Cache

Modules are written to disk after download, but never read back - each session downloads
the module again:

```
~/.local/share/wowee/warden_cache/<MD5>.wdn     (%APPDATA%\wowee\warden_cache on Windows)
```

The same directory is where `<MD5>.cr` challenge/response files are read from.

Memory and code checks read a `WoW.exe` image, found in `$WOWEE_INTEGRITY_DIR`, a few
home-directory locations, or `Data/misc` / `Data/expansions/<id>/misc`.

---

## Dependency

Unicorn Engine is required for module execution:

```bash
sudo apt install libunicorn-dev   # Ubuntu/Debian
sudo dnf install unicorn-devel    # Fedora
sudo pacman -S unicorn            # Arch
brew install unicorn              # macOS
```

The client builds without Unicorn and skips module emulation; check replies are built by
`WardenHandler` either way.

---

## Key Files

```
include/game/warden_handler.hpp + src/game/warden_handler.cpp   - Packet handler, check replies
include/game/warden_module.hpp  + src/game/warden_module.cpp    - Module loader (8-step pipeline, MD5/RSA)
include/game/warden_emulator.hpp + src/game/warden_emulator.cpp - Unicorn Engine executor
include/game/warden_crypto.hpp  + src/game/warden_crypto.cpp    - RC4 + SHA1Randx key derivation
include/game/warden_memory.hpp  + src/game/warden_memory.cpp    - WoW.exe PE image + memory patching
include/game/warden_constants.hpp                               - Sub-opcodes and result codes
```

---

## Logs

```bash
grep -i warden logs/wowee.log
```

Key messages:
- `Warden: Module loaded successfully` - module downloaded and loaded
- `WardenModule: Calling module entry point` - emulation running
- `Warden: Loaded N CR entries` / `Warden: No .cr file found` - whether hash requests can be answered
- `Warden: (sync) Parsed N checks` / `Warden: (async) Parsed N checks` - a check request was answered
- `WardenMemory: WoW.exe not found` - memory checks will answer "unmapped"
- `packetsAfterGate=0` - server not responding after Warden exchange (debug level)

---

## Check Types

Check opcodes are per module (read from the `.cr` header). Each result goes into one
`CHEAT_CHECKS_RESULT` (`[0x02][uint16 len][uint32 checksum][results]`):

| Name | Response |
|------|----------|
| MEM (read memory) | `[0x00][bytes]`, or `[0xE9]` if unmapped |
| PAGE_A / PAGE_B (code by hash) | `[0x4A]` found, `[0x00]` not |
| MPQ (file hash) | `[0x00][sha1]`, or `[0x01]` not found |
| LUA | `[0x01]` (not found) |
| MODULE | `[0x4A]` for system DLLs, `[0x00]` otherwise |
| DRIVER | `[0x00]` |
| PROC (API check) | `[0x01]` |
| TIMING | `[0x01][uint32 ticks]` |

---

**Last Updated**: 2026-09-21
