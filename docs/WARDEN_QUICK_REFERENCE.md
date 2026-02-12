# Warden Quick Reference

## TL;DR

**What works**: Servers with Warden disabled or permissive settings
**What doesn't work**: Warmane (requires module execution)
**What we have**: Complete crypto system, no module execution

---

## Testing a New Server

1. **Check if Warden is required**:
```
Connect → Look for SMSG_WARDEN_DATA (0x2E6)
If no Warden packet → Server doesn't use Warden ✅
If Warden packet → Continue testing
```

2. **Watch for server response**:
```
After CMSG_WARDEN_DATA sent:
  - Gets SMSG_CHAR_ENUM → SUCCESS ✅
  - Connection stays open but silent → Rejected ⏸️
  - Connection closes → Rejected ❌
```

3. **Check logs**:
```bash
tail -f logs/wowee.log | grep -i warden
```

Look for:
- `packetsAfterGate=0` (bad - server silent)
- `packetsAfterGate>0` (good - server responding)

---

## Quick Fixes

### Server Goes Silent
**Symptom**: Connection stays open, no SMSG_CHAR_ENUM
**Cause**: Server doesn't accept our response format
**Fix**: Try different response format (see below)

### Server Disconnects
**Symptom**: Connection closes after Warden response
**Cause**: Response is definitely wrong
**Fix**: Don't use that format, try others

### Can't Get Past Warden
**Solution 1**: Use a server with Warden disabled
**Solution 2**: Contact server admin for test account
**Solution 3**: Implement module execution (months of work)

---

## Trying New Response Formats

Edit `src/game/game_handler.cpp` around line 1850:

```cpp
std::vector<uint8_t> moduleResponse;

// Try your format here:
moduleResponse.push_back(0xYOUR_BYTE);
// Add more bytes...

// Existing code encrypts and sends
```

Rebuild and test:
```bash
cd build && cmake --build . -j$(nproc)
cd bin && ./wowee
```

---

## Response Formats Already Tried (Warmane)

| Format | Bytes | Result |
|--------|-------|--------|
| Empty | 0 | Silent ⏸️ |
| `[0x00][MD5][0x01]` | 18 | Silent ⏸️ |
| `[0x01]` | 1 | Disconnect ❌ |
| `[20-byte echo]` | 20 | Silent ⏸️ |
| `[0x01][SHA1]` | 21 | Silent ⏸️ |

---

## Module Packet Structure

```
Byte    Content
0       Opcode (varies each packet)
1-16    Seed (16 bytes for RC4)
17-36   Trailing data (20 bytes, possibly SHA1)
```

---

## Crypto Overview

```cpp
// Initialize (first packet only)
wardenCrypto_->initialize(moduleData);

// Decrypt incoming
auto plain = wardenCrypto_->decrypt(encrypted);

// Encrypt outgoing
auto encrypted = wardenCrypto_->encrypt(plain);
```

Keys are derived from:
- Hardcoded Warden module key (in `warden_crypto.cpp`)
- 16-byte seed from server
- XOR operation for output key

---

## Check Types Reference

| Opcode | Name | What It Checks | Our Response |
|--------|------|----------------|--------------|
| 0x00 | Module Info | Module status | `[0x00]` |
| 0x01 | Hash Check | File/memory hashes | `[0x01][0x00...]` |
| 0x02 | Lua Check | Suspicious addons | `[0x02][0x00]` |
| 0x04 | Timing | Speedhacks | `[0x04][timestamp]` |
| 0x05 | Memory | Memory scans | `[0x05][num][0x00...]` |

All responses are **faked** - we don't actually check anything.

---

## Common Errors

**Build fails**: Missing OpenSSL
```bash
sudo apt-get install libssl-dev
```

**Crypto init fails**: Bad module packet
```
Check log for "Warden: Initializing crypto"
Ensure packet is at least 17 bytes
```

**Always disconnects**: Server detects fake client
```
No easy fix - need module execution or different server
```

---

## Next Steps for Warmane Support

1. **Capture real WoW client packets** (Wireshark)
2. **Compare with our responses** (find differences)
3. **Implement matching format** (edit game_handler.cpp)
4. **OR**: Implement module execution (months)

---

## File Locations

```
Crypto:     src/game/warden_crypto.cpp
Hashes:     src/auth/crypto.cpp
Handler:    src/game/game_handler.cpp (handleWardenData)
Opcodes:    include/game/opcodes.hpp (0x2E6, 0x2E7)
Logs:       logs/wowee.log
Full docs:  docs/WARDEN_IMPLEMENTATION.md
```

---

## Support Resources

- [Full Implementation Doc](WARDEN_IMPLEMENTATION.md)
- [WoWDev Wiki - Warden](https://wowdev.wiki/Warden)
- [TrinityCore Source](https://github.com/TrinityCore/TrinityCore/tree/3.3.5)

---

**Last Updated**: 2026-02-12
**Status**: Working (permissive servers) | Not Working (Warmane)
