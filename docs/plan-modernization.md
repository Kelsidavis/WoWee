# Codebase Modernization — Phased Plan

**Status:** Phases 1, 3, 5, 6, 7 and the timeline-semaphore half of 8 complete. Phases 2 and 4 cancelled. Next: Phase 10.
**Scope:** code quality and modernity, codebase-wide. Not a performance effort. Where a phase
happens to help performance that is a side effect, and no phase here is justified by it.

This document records the plan and the decisions behind it. Phase outcomes belong in the
commit history, not here.

---

## 1. Where this codebase actually stands

Measured 2026-08-15, not assumed. The headline: **the usual C++ modernization checklist is already
done here.** What remains is C++20 adopted in name but not in practice, a Vulkan layer written to
1.0-era patterns, and concentrated build debt.

### Already good — do not spend time here

| Thing | Evidence |
|---|---|
| C++20 | `CMakeLists.txt:5` `set(CMAKE_CXX_STANDARD 20)`, `_REQUIRED ON` |
| **No manual `new`/`delete`** | `new T`: **0**. `delete`: **0**. 153 `unique_ptr`, 135 `shared_ptr` |
| clang-tidy, thoughtfully tuned | `.clang-tidy`: `bugprone-*`, `clang-analyzer-*`, `performance-*`, 10 `modernize-*`, 11 `readability-*`, with documented graphics-appropriate suppressions |
| Warnings as errors | `WOWEE_WARNINGS_AS_ERRORS` ON (`CMakeLists.txt:1282`) |
| Scoped enums | 116 `enum class` vs **6** plain `enum` |
| `constexpr` culture | 2812 `constexpr`, 3390 `const auto` |
| No C-style casts | `(float)x` form: **0** |
| Legacy C mostly gone | bare `printf` ≈3, `strcpy` 0, `NULL` **9** (rest were `VK_NULL_HANDLE` substring matches) |
| Optional-based returns | 160 `std::optional` |
| Tests | 155 test files, 89 CTest suites; `./test.sh` runs unit + lint |

Anyone proposing "use smart pointers" or "modernize your loops" has not read this code.

### Gap 1 — C++20 adopted in name, not in practice

| Feature | Uses |
|---|---|
| `std::span` | **2** |
| `std::ranges` | 6 |
| `std::bit_cast` | **0** |
| `std::format` | **0** |
| `std::jthread` | **0** (16 raw `std::thread`, 5 manual `.join()`) |
| concepts | **0** |
| `starts_with` / `ends_with` | **0** |

Against that: **303 `reinterpret_cast`**, **144 `memcpy`**, and **507 `std::snprintf`** — packet
parsing, GPU structs, and UI string building. Exactly where `span`, `bit_cast` and `format` pay.

`snprintf` clusters in UI: `combat_ui.cpp` (132), `game_screen_frames.cpp` (39),
`inventory_screen.cpp` (38). All the `char buf[N]` + `snprintf(buf, sizeof(buf), "%.3f", …)`
pattern — truncation-prone and type-unsafe by construction.

### Gap 2 — one surviving pocket of manual memory management

"Zero `new`/`delete`" is true, but `src/audio/audio_engine.cpp` does raw `std::malloc` /
`std::free` for miniaudio C interop (`ma_sound`, `ma_audio_buffer`) — 11 `malloc`, 33 `free`,
with hand-written cleanup on every early-return path. That is the classic shape of a leak on an
error branch, and it is the only place in the codebase not covered by RAII.

### Gap 3 — Vulkan 1.2 is required but essentially unused

`vk_context.cpp:314` does `require_api_version(1, 2, 0)`. Everything below is **core in 1.1 or
1.2** — already guaranteed, no extension, no new hardware floor:

| Feature | Core since | Uses |
|---|---|---|
| Timeline semaphores | 1.2 | **0** |
| Descriptor indexing | 1.2 | **0** |
| Buffer device address | 1.2 | **0** |
| `vkCmdDrawIndexedIndirectCount` | 1.2 | **0** |
| Dynamic rendering | 1.3 (ext on 1.2) | **0** — 12 `vkCreateRenderPass`, 29 `VkFramebuffer` |
| synchronization2 | 1.3 (ext on 1.2) | **0** — 30 `vkCmdPipelineBarrier` sites, 29 barrier structs |

Descriptor traffic today: 34 `vkAllocateDescriptorSets`, 44 `vkUpdateDescriptorSets`.
MoltenVK 1.4.2 (installed) implements **Vulkan 1.4** and exports the sync2 entry points, so the
macOS target is not the constraint.

### Gap 4 — build hygiene

- **No PCH, no unity build** (neither `target_precompile_headers` nor `UNITY_BUILD` in CMake).
- 413 `.cpp` files.
- `core/logger.hpp` included by **170** files; `pipeline/wowee_binary_io.hpp` by 144.
- `<cstdint>` in 330 headers, `<string>` 312, `<vector>` 306.
- Anchors: **`game_handler.hpp` is 5013 lines, included by 57 files**; `world_packets.hpp` 3387.

---

## 2. Ordering principle

Ordered by (value ÷ risk), not by how modern each item sounds.

1. **Build speed first.** Every later phase is edit-compile-test cycles. Phase 1 costs an
   afternoon and makes the rest cheaper.
2. **Prefer changes that are already paid for.** Timeline semaphores and descriptor indexing are
   core in the Vulkan version already required. Dynamic rendering and sync2 need 1.3 or
   extensions, so they rank lower despite being more fashionable.
3. **Safety before elegance.** `span`/`format`/`bit_cast` turn silent corruption into compile
   errors. Concepts and ranges just read nicer.
4. **One idiom at a time, codebase-wide.** Half-migrated barrier or render-pass code is worse than
   either end state — and is especially hazardous when a model writes from this plan, because it
   pattern-matches on whichever example it reads first.
5. **Lock in each gain.** Every language phase ends by enabling the matching clang-tidy check, so
   the improvement cannot silently regress. `.clang-tidy` currently has `WarningsAsErrors: ''`.

---

## 3. Phases

One phase per session. Each ends green: `./test.sh` passes (unit + lint), zero validation errors.

### Phase 1 — Precompiled header
`target_precompile_headers` with the measured-heavy set: `<cstdint>`, `<string>`, `<vector>`,
`<memory>`, `<unordered_map>`, `<functional>`, plus `core/logger.hpp`. Nothing else
project-specific — a PCH pulling in `game_handler.hpp` would make incremental builds worse.

**Exit:** clean-build wall time recorded before/after in §5. Revert if it does not improve.

### Phase 2 — Split the compile-time anchors — CANCELLED
The original plan was to split `game_handler.hpp` (5013 lines, 57 includers) and
`world_packets.hpp` (3387) to cut compile time. Measuring preprocessed output killed it:

| translation unit | preprocessed lines |
|---|---|
| includes `game_handler.hpp` | 133,121 |
| the stdlib + glm it includes, alone | 115,278 |
| **all 19 project headers, combined** | **17,843 (13%)** |

Splitting 5000 lines of declarations chases 13% of the cost, and the declarations still have to
exist somewhere. The 87% is the standard library, which a PCH removes for free — done in
`d938a41b0`.

Line count was the wrong metric. `wc -l` on a header says nothing about what it costs to compile;
`c++ -E` does. **If a future phase proposes splitting a header for build speed, measure first.**

Splitting `game_handler.hpp` may still be worth doing for *readability* — a 5000-line class is
hard to reason about — but that is a decomposition argument, not a compile-time one, and it should
be justified on its own terms.

### Phase 3 — `std::span` across buffer and packet boundaries
The highest safety-per-line item here. Replace `(ptr, length)` parameter pairs with `std::span`,
starting where the 303 `reinterpret_cast` and 144 `memcpy` cluster: packet parsers
(`src/game/*_handler*.cpp`), `wowee_binary_io.hpp`, ADT and asset loaders. Bounds become explicit
and checkable instead of conventional.

Go **file by file with tests after each** — this is parsing code where a mistake is silent
corruption, not a crash.

**Exit:** `span` uses up from 2 to real coverage across parsing; `reinterpret_cast` count down; a
test added that feeds a truncated buffer to a parser and expects clean rejection.

**Done for the parsers that read untrusted files** (`adt_loader`, `blp_loader`). The rest of the
codebase's `(ptr, len)` pairs are crypto and socket wrappers where the length comes from a buffer
the caller already owns — converting those is cosmetic, and the plan's own "safety before
elegance" rule says leave them. `wmo_loader` and `m2_loader` were audited and are already
defensive at the read primitive; only their chunk-bounds arithmetic needed widening.

Note the survey pattern that nearly wasted a session: grepping for
`uint8_t\s*\*\s*\w+,\s*size_t` matched `const char* name, uint32_t bit` in a dozen
`wowee_*.cpp` files — lambda parameters, not buffers. Tighten the pattern to require a
length-shaped name (`len|size|length|count`) before believing a count.

### Phase 4 — `std::format` for the 507 `snprintf` sites — CANCELLED
The case for this phase was that each `char buf[N]` + `snprintf` is a truncation *and* a
type-safety hazard. Measuring both halves killed it.

**Type safety is already enforced.** `CMakeLists.txt:1339` sets `-Wall -Wextra -Wpedantic` and
`:1351` adds `-Werror`. `-Wformat` fires on a mismatched specifier, so `%d` against a
`unsigned long long` is a *build error* in this project today, not a latent bug. Verified by
compiling one.

**Truncation is bounded and mostly unreachable.** `snprintf` cannot overrun — it truncates. And
of the 507 sites:

| | count |
|---|---|
| total `snprintf` | 507 |
| formatting any `%s` | 140 |
| formatting a `.c_str()` (unbounded input) | **40** |

The 40 are chat and UI lines — `"%s killed %s."` with player names, which WoW caps at 12
characters, into 128- and 256-byte buffers. The remaining 467 format integers and floats into
buffers that cannot overflow.

So the cost is 507 edits across UI code with **no test coverage for string output**, against a
benefit of "a name longer than the format budget would display in full". Every edit is a chance
to get `%02d` → `{:02}` or `%.1f` → `{:.1f}` subtly wrong, and nothing would catch it. That
trades a real regression risk for a cosmetic gain, which is the opposite of this plan's
"safety before elegance" rule.

Revisit only if a specific truncation is observed in practice, and then fix that site.

### Phase 5 — RAII the audio C interop
Wrap the miniaudio `malloc`/`free` pairs in `audio_engine.cpp` in a `unique_ptr` with a custom
deleter (or a small owning type). Removes the hand-written cleanup on every early-return path and
closes the last manual-memory hole in the codebase.

**Exit:** zero bare `malloc`/`free`, audio still works, `./test.sh --asan` clean.

### Phase 6 — `std::bit_cast` — DONE (reduced scope)
This phase was written as "replace the *punning* subset of the 303 `reinterpret_cast`... Today
those are UB-adjacent." **That was false on both counts.**

There is no `reinterpret_cast` punning in this codebase. Of 298 casts:

| form | count | what it is |
|---|---|---|
| `char*` / `const char*` | **206** | byte-level object access — explicitly legal, not punning |
| `ImTextureID`, `ffx*`, `VkDescriptorSet`, `void*` | ~50 | opaque handles for C APIs |
| `decltype(fns_->…)` | 14 | function pointers for dynamic loading |
| `uint64_t` | 4 | pointer-to-integer |

A search for `*reinterpret_cast<T*>(&x)` — the actual punning shape — returns **nothing**. The
codebase already used `memcpy`, which is well defined, not "UB-adjacent".

So the real scope was six `memcpy(&a, &b, sizeof(float))` sites. Converted regardless: `bit_cast` is the same operation as an expression rather than a
statement, which lets `Packet::readFloat`/`writeFloat` collapse to one line each. Cosmetic, not
a correctness fix.

### Phase 7 — `std::jthread` — DONE (reduced scope)
Converted one thread, deliberately. The stall watchdog in `application.cpp` had a hand-rolled
`std::atomic<bool>` stop flag *and* its teardown written twice — once in a `try/catch` that
existed for no other reason, once after the loop. A `jthread` destructor does both, so the flag,
the catch and the duplicate both went. `git diff -w`: five lines added, sixteen removed.

**Left the worker pools alone**, and that is the finding. `TerrainManager::stopWorkers` pairs its
flag with a `queueCV.notify_all()` and carries a comment explaining why it joins the way it does
(plain `join()` rather than `pthread_timedjoin_np`, which leaves the `std::thread` thinking it is
still joinable and terminates in the destructor). `thread_pool`, `world_loader` and
`world_socket` are the same shape. Those are considered designs, not omissions — a `stop_token`
would have to be threaded through the same condition variable to match what they already do, and
the payoff is nothing.

The rule this phase suggests: `jthread` is worth it where a thread has a **hand-rolled flag and
duplicated teardown**. Where the stop mechanism is already integrated with a condition variable,
it is a rewrite, not a modernization.

### Phase 8 — Vulkan: timeline semaphores + descriptor indexing
Both are core in the Vulkan 1.2 this project already requires, so neither needs an extension or
raises the hardware floor.

**Timeline semaphores: done.** `FrameData::inFlightFence` is replaced by one timeline semaphore
and a monotonic counter. The swapchain semaphores are unchanged and had to be — WSI does not
accept timeline semaphores, so `vkAcquireNextImageKHR` and `vkQueuePresentKHR` keep their binary
pair and the submit signals both. `resetFrameSyncState` no longer destroys and recreates the
frame sync objects: `vkDeviceWaitIdle` guarantees the counter has reached its last signalled
value, so re-baselining each slot to it leaves the slot already satisfied.

Verified on the target GPU with validation enabled: zero sync VUIDs, zero timeouts, correct
across swapchain rebuilds.

**Descriptor indexing: deferred, and not on modernity grounds.** The case for it was 34
`vkAllocateDescriptorSets` and 44 `vkUpdateDescriptorSets` call sites, but sites are not
per-frame cost. What descriptor indexing actually removes is per-draw *binds*, and measuring
those:

- **M2 already avoids them.** Draws are sorted by model, and the loop skips rebinding when the
  material has not changed (`batch.materialSet != currentMaterialSet`). The redundant-bind
  problem is solved by state tracking.
- **Terrain does not**, binding one set per visible chunk. But every chunk genuinely has its own
  textures and alpha maps, so no amount of deduplication helps — only a bindless array would.

That leaves a few hundred binds per frame in terrain, which is tens of microseconds. Real, but
not what this plan is for, and it is a large change to material binding across four renderers
plus `nonuniformEXT` in the shaders.

Its actual value is as an **enabler**: a bindless material array is what lets the GPU choose a
material without the CPU binding one, which is a precondition for GPU-driven terrain and for the
indirect path `docs/plan-grass.md` describes. Justify it from that work when it arrives, with a
feature that needs it, rather than as a modernization step on its own.

### Phase 9 — Vulkan: synchronization2 + dynamic rendering (one change, or neither)
30 barrier sites / 29 barrier structs (25 image, 2 buffer, 2 memory); 12 render passes, 29
framebuffers. These pair naturally — same Vulkan 1.3 idiom, both cross-cutting — and doing one
without the other leaves two idioms in the tree.

Be clear-eyed about the payoff: this is **readability and API modernity, not speed**. Barrier
dispatch is nanoseconds against a millisecond frame, and MoltenVK collapses sync2's fine-grained
masks into Metal's coarser model, so most of the precision advantage is lost on the primary
platform. Requires bumping `require_api_version` to 1.3 or enabling both extensions with feature
structs — check the hardware floor that implies first.

**Exit:** zero legacy `vkCmdPipelineBarrier`, zero `vkCreateRenderPass`, validation clean, and
`docs/plan-grass.md` §2 deviation row flipped to sync2.

### Phase 10 — Lock it in
Tighten `.clang-tidy` now that the codebase can pass more: add the `modernize-*` checks
corresponding to the work above, and set `WarningsAsErrors` to that list so regressions fail CI
rather than accumulating. Do this **last** — enabling them earlier would block phases with noise
from code not yet migrated.

**Exit:** `./test.sh --lint` green with the tightened set, CI enforcing it.

---

## 4. Explicitly NOT recommended

Written down so they are not re-proposed every time someone reads a blog post.

| Proposal | Why not |
|---|---|
| `std::format` for the **logging macros** | 3557 `LOG_*` sites through a variadic macro (`logger.hpp:121`). Enormous churn, no safety gain — the macro is already type-safe via templates. This is the opposite of Phase 4, where each `snprintf` carries a real buffer hazard. |
| `std::expected` | C++23; unavailable at C++20. 160 `std::optional` already covers most of it. Revisit only if the standard is bumped for another reason. |
| Concepts everywhere | A game client, not a generic library. Almost no templated public API to constrain — cost without payoff. |
| C++20 modules | Toolchain support across the Linux/Windows/macOS CI matrix is still uneven and the three major compilers disagree. Revisit in a few years. |
| Unity build | Conflicts with PCH gains, hurts incremental builds, and invites ODR surprises given the many TU-local statics here. Phase 1 gets most of the win without them. |
| Buffer device address | Core in 1.2 and available, but nothing currently needs GPU pointer-chasing. Adopt when a system actually calls for it, not preemptively. |
| Wholesale `reinterpret_cast` removal | Most are legitimate for graphics/network code — `.clang-tidy` disables the aggressive checks deliberately and documents why. Only the punning subset moves (Phase 6). |
| Replacing the 2365 `char*` | Overwhelmingly `const char*` string literals for Vulkan extension names, Lua API and file paths. Correct as-is. |

---
