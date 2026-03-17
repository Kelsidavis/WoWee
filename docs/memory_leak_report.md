# Memory Leak Analysis Report

Date: 2026-03-17  
Repository: WoWee

## Scope

- Reviewed explicit heap allocation sites in `src/` and `include/`.
- Traced allocation/free paths for:
  - `new` / `delete`
  - `malloc` / `free`
  - Warden emulator virtual heap allocation
- Focus was on leak behavior during long-running sessions.

## Finding

### 1) Warden emulator heap growth leak (fixed)

- Location: `src/game/warden_emulator.cpp`
- Root cause:
  - `allocateMemory()` used a bump pointer (`nextHeapAddr_`) only.
  - `freeMemory()` removed entries from `allocations_`, but freed ranges were never reused.
  - Result: repeated `VirtualAlloc`/`VirtualFree` patterns still advanced heap head until exhaustion.

### Impact

- Emulated heap (`HEAP_SIZE = 16MB`) could exhaust over time despite correct frees.
- This can break Warden module execution paths in extended sessions.

## Patch Summary

- Added a free-list map in `WardenEmulator` to track reusable blocks.
- Updated allocator to use first-fit from free-list before bump allocation.
- Added adjacent block coalescing in `freeMemory()`.
- Added top-of-heap rollback when highest free block touches current bump pointer.
- Reset allocator state on `initialize()` to avoid stale state across reinitialization.

## Changed Files

- `include/game/warden_emulator.hpp`
- `src/game/warden_emulator.cpp`

## Notes

- This was a static code analysis pass (no full runtime sanitizer execution in this environment).
- No other definite leaks were confirmed in this pass.

