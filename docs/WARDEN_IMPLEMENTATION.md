# Warden Implementation

**Status**: Partial - the module is loaded, decrypted, unpacked, relocated, its imports bound and its entry point run under Unicorn, and the `WardenFuncList` it returns is read back. The module's own packet handler is not used for checks: `WardenModule::processCheckRequest` has no caller, and `WardenHandler` builds every reply itself (see Check Responses).
**WoW Version**: 3.3.5a (build 12340); the handler also has paths for 1.12.1 and Turtle WoW-derived realms

---

## Overview

Warden is WoW's client integrity checking system. The server sends encrypted modules
containing native x86 code; the client is expected to load and execute them, then
return check results.

Wowee runs the x86 module through Unicorn Engine CPU emulation - in an emulated
environment with Windows API hooks, without Wine or a Windows OS. The check replies
are built by the client rather than by the module: memory reads are served from a
`WoW.exe` image on disk, and hash requests from pre-computed `.cr` files.

---

## Loading Pipeline (8 steps)

```
1. MD5       - Verify module checksum matches server challenge
2. RC4       - Decrypt module payload
3. RSA-2048  - Verify module signature (Blizzard's key, or the realm's - see Crypto Layer below)
4. zlib      - Decompress module
5. Parse     - Unpack Warden's own image format (copy/skip pair stream, or the native header Turtle-derived realms use)
6. Relocate  - Apply base relocations to load address
7. Bind      - Resolve imports (Windows API stubs + Warden callbacks)
8. Init      - Call module entry point via Unicorn Engine
```

No step's failure stops the load: `WardenModule::load` logs it and continues with
what it has. A module that did not unpack into an image is not emulated at all, and
the handler answers from its own code.

---

## Unicorn Engine Execution

The module entry point is called inside an Unicorn x86 emulator with:

- Executable memory mapped at the module's load address
- A simulated stack
- Windows API interception for calls the module makes

Hooked APIs are `VirtualAlloc`, `VirtualFree`, `GetTickCount`, `Sleep`,
`GetCurrentThreadId`, `GetCurrentProcessId` and `ReadProcessMemory`; any other import
is auto-stubbed to return 0. Each hook returns a plausible value without accessing
real process memory.

A module whose signature did not verify is run anyway, so a private server's own
module gets its chance. When it faults, that is logged as a warning rather than an
error (`WardenEmulator::setFailuresExpected`), since it is the expected outcome.

---

## Module Cache

After download, the raw (encrypted) module is written to disk:

```
~/.local/share/wowee/warden_cache/<MD5>.wdn     (%APPDATA%\wowee\warden_cache on Windows)
```

Nothing reads it back: the client answers every `MODULE_USE` with `MODULE_MISSING`,
so the module is downloaded and loaded again each session.

The same directory holds `<MD5>.cr` files - pre-computed challenge/response entries
for a module (a 17-byte header carrying the module's nine check opcodes, then 68-byte
entries of seed, reply and the next RC4 key pair). `WardenHandler::loadWardenCRFile`
reads one on `MODULE_USE`, and a `HASH_REQUEST` whose seed matches an entry is
answered from it.

---

## Crypto Layer

| Algorithm | Purpose |
|-----------|---------|
| RC4 | Encrypt/decrypt Warden traffic (separate in/out ciphers) |
| MD5 | Module identity hash |
| SHA1 | HMAC, check hashes and SHA1Randx key derivation |
| RSA-2048 | Module signature verification |

The signature is checked against Blizzard's retail modulus (`kRetailModulus` in
`src/game/warden_module.cpp`, the same across 1.12.1, 2.4.3 and 3.3.5a). A server that
signs its own module can name its key as `wardenRsaModulus` in the expansion profile -
512 hex characters for the 256-byte modulus - and the module is checked against that
instead. A mismatch is logged as a warning and loading continues.

---

## Opcodes

- `SMSG_WARDEN_DATA` = 0x2E6 - server sends module + checks
- `CMSG_WARDEN_DATA` = 0x2E7 - client sends results

Sub-opcodes inside the decrypted payload (`include/game/warden_constants.hpp`):

| Direction | Value | Name |
|-----------|-------|------|
| S->C | 0x00 | `MODULE_USE` (module hash, key, size) |
| S->C | 0x01 | `MODULE_CACHE` (module data chunk) |
| S->C | 0x02 | `CHEAT_CHECKS_REQUEST` |
| S->C | 0x03 | `MODULE_INITIALIZE` (no reply) |
| S->C | 0x05 | `HASH_REQUEST` (16-byte seed) |
| C->S | 0x00 | `MODULE_MISSING` |
| C->S | 0x01 | `MODULE_OK` |
| C->S | 0x02 | `CHEAT_CHECKS_RESULT` |
| C->S | 0x04 | `HASH_RESULT` |

---

## Check Responses

Check type opcodes differ per module; they are read from the `.cr` file header and
XORed with the request's last byte. `WardenHandler::handleWardenData` answers each one:

| Check type | Reply |
|------------|-------|
| `READ_MEMORY` (MEM) | `0x00` + bytes read from the `WoW.exe` image, or `0xE9` if unmapped |
| `FIND_MEM_IMAGE_CODE_BY_HASH` / `FIND_CODE_BY_HASH` (PAGE_A/B) | `0x4A` if an HMAC-SHA1 pattern search of the image finds it, else `0x00` |
| `HASH_CLIENT_FILE` (MPQ) | `0x00` + SHA1 of the file via the asset manager, else `0x01` |
| `GET_LUA_VARIABLE` (LUA) | `0x01` (not found) |
| `FIND_MODULE_BY_NAME` (MODULE) | `0x4A` for standard system DLLs, `0x00` otherwise |
| `FIND_DRIVER_BY_NAME` (DRIVER) | `0x00` (not found) |
| `API_CHECK` (PROC) | `0x01` (not found) |
| `CHECK_TIMING_VALUES` (TIMING) | `0x01` + millisecond tick count |

The reply is `[0x02][uint16 length][uint32 checksum][results]`, the checksum being the
XOR of the five words of SHA1(results).

`WardenMemory` loads the `WoW.exe` image the memory and code checks read from. It looks
in `$WOWEE_INTEGRITY_DIR`, a few home-directory locations, and `Data/misc` /
`Data/expansions/<id>/misc`, and fakes `KUSER_SHARED_DATA`. Without it, memory checks
answer `0xE9`.

A `HASH_REQUEST` with no matching `.cr` entry gets no reply on WotLK/TBC and Turtle
(AzerothCore tolerates the silence and rejects a wrong hash; Turtle-derived realms
close the connection) - `WOWEE_WARDEN_TURTLE_NO_CR=fallback` overrides that for Turtle.
1.12.1 realms are sent a SHA1 of the module image as a fallback.

---

## Key Files

```
include/game/warden_handler.hpp      - Packet handler interface
src/game/warden_handler.cpp          - handleWardenData, .cr loading, check replies
include/game/warden_constants.hpp    - Sub-opcodes, memory ranges, result codes
include/game/warden_module.hpp       - Module loader interface
src/game/warden_module.cpp           - 8-step pipeline
include/game/warden_emulator.hpp     - Emulator interface
src/game/warden_emulator.cpp         - Unicorn Engine executor + API hooks
include/game/warden_crypto.hpp       - Crypto interface
src/game/warden_crypto.cpp           - RC4 / key derivation
include/game/warden_memory.hpp       - PE image + memory patch interface
src/game/warden_memory.cpp           - WoW.exe PE loader, runtime globals patching, code-pattern search
```

---

## Performance

- `PAGE_A`/`PAGE_B` checks brute-force a pattern search over the image and can take
  seconds. A request containing one is built on a background thread (`std::async`) and
  sent from `WardenHandler::update`, so the main loop does not stall.
- Pattern-search results are cached per seed, hash and length (`WardenMemory::codePatternCache_`).

---

## Dependencies

Requires `libunicorn-dev` (Unicorn Engine); CMake defines `HAVE_UNICORN` when it is
found. The client compiles without it and skips emulation. Check replies are built by
`WardenHandler` either way, so they do not depend on it.

---

## References

- [WoWDev Wiki - Warden](https://wowdev.wiki/Warden)
- [WoWDev Wiki - SMSG_WARDEN_DATA](https://wowdev.wiki/SMSG_WARDEN_DATA)
- [TrinityCore Warden](https://github.com/TrinityCore/TrinityCore/tree/3.3.5/src/server/game/Warden)

---

**Last Updated**: 2026-09-21
