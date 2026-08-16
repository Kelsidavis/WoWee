# Codebase Modernization — Phased Plan

**Status:** Phases 0–3 complete. Next: Phase 4 (`std::format` for 507 `snprintf` sites).
**Branch:** `master`. Phase 1 was done on `chore/modernization` and pushed to `master` as
`7b9d4a93d..2b5628a0f`; later phases continue directly on `master`.
**Scope:** code quality and modernity, codebase-wide. Not a performance effort. Where a phase
happens to help performance that is a side effect, and no phase here is justified by it.

Read this file at the start of every session. Update §5 at the end of each phase.

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

### Phase 2 — ~~Split the compile-time anchors~~ **CANCELLED — measured, not worth it**
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

### Phase 4 — `std::format` for the 507 `snprintf` sites
Each `char buf[N]` + `snprintf` is a truncation and type-safety hazard. `std::format` makes both
compile-time concerns. Work by cluster: `combat_ui.cpp` (132), `game_screen_frames.cpp` (39),
`inventory_screen.cpp` (38), then the tail.

Verify libc++/libstdc++/MSVC all have `<format>` on the CI matrix before starting; if any lag,
use `std::vformat` or defer this phase rather than adding a dependency.

**Exit:** `snprintf` count near zero, no fixed-size char buffers left in UI string building,
tests green. Enable a lint check to keep it out.

### Phase 5 — RAII the audio C interop
Wrap the miniaudio `malloc`/`free` pairs in `audio_engine.cpp` in a `unique_ptr` with a custom
deleter (or a small owning type). Removes the hand-written cleanup on every early-return path and
closes the last manual-memory hole in the codebase.

**Exit:** zero bare `malloc`/`free`, audio still works, `./test.sh --asan` clean.

### Phase 6 — `std::bit_cast` for type punning
Replace the *punning* subset of the 303 `reinterpret_cast` — the `float`↔`uint32` and
struct-reinterpret cases — with `std::bit_cast`. Those are UB-adjacent today; `bit_cast` is
well-defined and `constexpr`. **Leave genuine pointer casts alone**: they are legitimate in
graphics/network code, which is why `.clang-tidy` disables the aggressive cppcoreguidelines checks
and says so.

**Exit:** punning cases converted, tests green, no release-build codegen regression.

### Phase 7 — `std::jthread` and threading cleanup
16 raw `std::thread`, 5 manual `.join()`, 78 `std::mutex`, 35 `std::atomic`. Move to `jthread` for
automatic joining and `stop_token` for cooperative cancellation, removing hand-rolled shutdown
flags.

**Exit:** no manual `.join()` on owned threads, shutdown paths simplified, `--asan` clean, no new
races under a stress run.

### Phase 8 — Vulkan: timeline semaphores + descriptor indexing
Both **core in 1.2, already required** — no extension, no feature negotiation, no fallback path.
The cheapest real Vulkan modernization available here.

- *Timeline semaphores*: replace fence + binary-semaphore juggling around `MAX_FRAMES_IN_FLIGHT=2`
  and the per-frame deferred-cleanup queues (`vk_context.hpp:270,316`) with one monotonic counter.
  "Is frame N-2 done?" becomes a comparison instead of a protocol.
- *Descriptor indexing*: collapse per-material descriptor sets into bindless arrays, cutting the
  34 allocate / 44 update sites and the per-draw rebinding around them.

Split into two sessions if it runs long; they are independent.

**Exit:** frame sync expressed as timeline waits, deferred cleanup keyed on timeline value,
descriptor updates per frame down (number in §5), no visual regression, validation clean.

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

## 5. Phase log

*(append at the end of each session: what landed, what was measured, what is still open)*

- **Phase 0** — Survey. All numbers in §1 measured directly. Three findings reframed the work:
  - The standard modernization checklist is **already complete** (C++20, no `new`/`delete`, tuned
    clang-tidy, warnings-as-errors, 116 `enum class` vs 6 plain, 2812 `constexpr`). The remaining
    language gap is *depth*: `std::span` at 2 uses against 303 `reinterpret_cast`, 144 `memcpy`
    and 507 `snprintf`.
  - The codebase **requires Vulkan 1.2 and uses nothing from 1.1 or 1.2.** Timeline semaphores and
    descriptor indexing are already paid for and unused, which is why they outrank the more
    fashionable dynamic-rendering/sync2 migration.
  - "Zero manual memory management" was **almost** true — `audio_engine.cpp` still does raw
    `malloc`/`free` for miniaudio interop with hand-written cleanup on error paths (Phase 5).

  Method note: several counts were wrong on the first pass because `\b` is not supported in
  git grep's POSIX ERE — `printf(` matched `snprintf(`, and `NULL` matched `VK_NULL_HANDLE`,
  inflating both by two orders of magnitude. `git grep -P` works here; use it for
  word-boundary counts and re-verify any number before acting on it.

- **Pre-flight** — `origin/master` did not compile with Apple clang. The window-removal commits
  left `kMonthAbbrev`, `kColorRed`, `kColorGreen`, `kColorYellow` and `kDialogFlags` defined but
  unused in `src/ui/window_manager.cpp`, and `-Werror,-Wunused-const-variable` makes an orphaned
  constant a build failure. Fixed in `89969db40`. **Check whether CI runs the macOS build** — this
  should not have reached master.

  Verified green baseline before any modernization work: build exit 0, **156/156 tests pass**
  (2 skipped: `framexml_compiles`, `addon_xml_compiles`).

- **Phase 1** — Build hygiene. Done: `cc31a0a29` (test object library), `45ffa4625` (PCH).

  | | before | after |
  |---|---|---|
  | clean build, wall | 1:56.61 | **1:33.13** (−20%) |
  | clean build, user CPU | 1229.51 s | **969.12 s** (−21%) |
  | `logger.cpp` compilations | 112 | **2** |
  | tests | 156/156 | 156/156 |

  The plan predicted PCH would be the win. It was not — watching a real build showed
  `logger.cpp` being compiled **112 times**, once per test target that opted in. Static
  inspection could not have found that; only the build log showed it. Prefer reading a real
  build log over reasoning about CMake.

  Two things this surfaced, both worth remembering:
  - Three tests listed *both* `${TEST_COMMON_SOURCES}` and a direct path to `logger.cpp`. That
    was latent and harmless while the variable held an identical path string CMake could dedupe,
    and became a duplicate-symbol link failure the moment the variable held an object file. A
    redundancy can be invisible until the representation under it changes.
  - Checking a *running* build's log for `error:` proves nothing — it returned 0 at 29% and at
    92% of a build that failed at 98%. Gate on the exit code, never on a grep of a live log.

- **Phase 1b / Phase 2** — Done: `d938a41b0` (measured PCH), `23e4048fd` (sanitizer coverage).
  Phase 2 as written was **cancelled on evidence** — see above.

  Cumulative against the pre-work baseline:

  | | baseline | now |
  |---|---|---|
  | clean build, wall | 1:56.61 | **1:31.05** (−22%) |
  | clean build, user CPU | 1229.51 s | **941.23 s** (−23%) |
  | tests | 156/156 | 156/156 |

  Findings:
  - The first PCH list was picked by counting include-list appearances and missed every one of
    the expensive headers. `<chrono>` alone is 89k preprocessed lines; glm 89k; `<future>` 87k.
    Adding them took user CPU 969.1 s → 945.2 s.
  - A PCH on the **test** targets was tried and removed: ~140 executables each generating their
    own PCH cost slightly more than it saved (950.8 s vs 945.2 s). The main target wins because
    it is one target over 413 objects; the tests are the opposite shape. Measured, reverted,
    and recorded in the file so it is not re-attempted.
  - **Bug found: `./test.sh --asan` was sanitizing only two thirds of the suite.** The ASAN
    block iterates `ALL_TEST_TARGETS` from mid-file, and 47 tests are declared below it. Those
    built uninstrumented and reported as passing. Any CMake loop over a list that is still being
    appended to belongs at the end of the file — the same positional bug could recur with any
    future `foreach` over that list.

- **Phase 3** — `std::span` across the untrusted-file parsers. Done: `8baa3c457` (ADT),
  `b6d5206c8` (sanitizer coverage + ADT test), `8fb5f5e58` (BLP), `90f5ba7ca` (WMO widening).

  Three real out-of-bounds reads, all reachable from game data on disk:
  - **ADT / MCAL.** `sizeAlpha - skip` with `sizeAlpha` known only to be >= 1 and `skip` 8 — a
    four-byte chunk spelling `MCAL` wrapped it to 4294967292, which became a memcpy length.
  - **BLP / decompressors.** Source length derived from width and height rather than from the mip
    actually supplied; 4096x4096 DXT1 with an 8-byte mip read 2 MB from an 8-byte buffer. The
    call site carried a comment claiming the decompressors bounded themselves — they bounded the
    destination only.
  - **BLP / mip offset.** `offset + mipSize > size` with both `uint32_t` from the file:
    `0xFFFFFF00 + 0x200` is `0x100` and passed.

  The shape is identical every time: **a length that comes from the file, arithmetic on it that
  can wrap, and the guard placed at the call site rather than where the read happens.** That is
  what `span` fixes — the bound travels with the pointer, so the callee can enforce it.

  Method, which cost more than the fixes: **a robustness test you have not watched fail is not
  evidence.** The ADT test passed twice with the bug deliberately reintroduced — first because an
  out-of-bounds read is invisible without ASAN, then because the test used `0x28`/`0x2C` for
  offsets the parser reads at decimal 36/40, so it built a well-formed chunk that never entered
  the branch. Every fix here was verified by reverting it and confirming ASAN reports
  `heap-buffer-overflow` or `BUS` first.

  Full suite under ASAN with all 156 targets genuinely instrumented: **157/157 clean.** The 77
  targets the sanitizer had been skipping hide no further bugs.
