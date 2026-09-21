# WoWee Testing Guide

This document covers everything needed to build, run, lint, and extend the WoWee test suite.

---

## Table of Contents

1. [Overview](#overview)
2. [Prerequisites](#prerequisites)
3. [Test Suite Layout](#test-suite-layout)
4. [Building the Tests](#building-the-tests)
   - [Release Build (normal)](#release-build-normal)
   - [Debug + ASAN/UBSan Build](#debug--asanubsan-build)
5. [Running Tests](#running-tests)
   - [test.sh - the unified entry point](#testsh---the-unified-entry-point)
   - [Running directly with ctest](#running-directly-with-ctest)
   - [Interface sweeps (sweep_guard)](#interface-sweeps-sweep_guard)
6. [Headless FrameXML runs (framexml_run)](#headless-framexml-runs-framexml_run)
7. [Lint (clang-tidy)](#lint-clang-tidy)
   - [Running lint](#running-lint)
   - [Applying auto-fixes](#applying-auto-fixes)
   - [Configuration (.clang-tidy)](#configuration-clang-tidy)
8. [ASAN / UBSan](#asan--ubsan)
9. [Adding New Tests](#adding-new-tests)
10. [CI Reference](#ci-reference)

---

## Overview

WoWee uses **Catch2 v3** (amalgamated) for unit testing and **clang-tidy** for static analysis. The `test.sh` script is the single entry point for both.

| Command | What it does |
|---|---|
| `./test.sh` | Runs both unit tests (Release) and lint |
| `./test.sh --test` | Runs unit tests only (Release build) |
| `./test.sh --lint` | Runs clang-tidy only |
| `./test.sh --asan` | Runs unit tests under ASAN + UBSan (Debug build) |
| `FIX=1 ./test.sh --lint` | Applies clang-tidy auto-fixes in-place |

All commands exit non-zero on any failure.

---

## Prerequisites

The tests are configured by the same CMake project as the client, so the whole client dependency set has to be installed first - SDL3, Vulkan, OpenSSL, zlib and the rest. See [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md) for platform-specific dependency installation. Python 3 is optional: without it the Python-based tests (the interface sweeps and a few checks in `tools/`) are not registered. StormLib is optional too: without it the tests that build a real archive are left out.

On top of that, lint needs clang-tidy:

### Linux (Ubuntu / Debian)

```bash
sudo apt install -y clang-tidy
```

### Linux (Arch)

```bash
sudo pacman -S --needed clang
```

### macOS

```bash
brew install llvm
# Add LLVM tools to PATH so clang-tidy is found:
export PATH="$(brew --prefix llvm)/bin:$PATH"
```

### Windows (MSYS2)

Install the full toolchain as described in `BUILD_INSTRUCTIONS.md`, then add:

```bash
pacman -S --needed mingw-w64-x86_64-clang-tools-extra
```

---

## Test Suite Layout

There are about two hundred ctest entries, almost all of them one executable per `tests/test_<name>.cpp`, plus a few Python checks. `ctest -N` in a configured build directory lists them all. A sample by area:

```
tests/
  CMakeLists.txt                          - CMake test configuration

  # Core
  test_packet.cpp                         - Network packet encode/decode
  test_srp.cpp                            - SRP-6a authentication math (requires OpenSSL)
  test_realm_list.cpp                     - REALM_LIST parsing across the vanilla / TBC-WotLK layouts
  test_opcode_table.cpp                   - Opcode registry lookup
  test_opcode_tables.cpp                  - Opcode numbers against AzerothCore, _extends inheritance
  test_update_field_indices.cpp           - Update field slots against AzerothCore
  test_entity.cpp                         - ECS entity basics
  test_dbc_loader.cpp                     - DBC binary file parsing
  test_dbc_layouts.cpp                    - The layout files shipped in Data/expansions
  test_m2_structs.cpp                     - M2 model struct layout / alignment
  test_blp_loader.cpp                     - BLP texture file parsing
  test_frustum.cpp                        - View-frustum culling math
  test_frame_pacer.cpp                    - Frame pacing at 144Hz
  test_update_check.cpp                   - Version ordering for the update notice
  test_realm_patches.cpp                  - Realm patch directory naming and manifest validation
  test_env_flag.cpp                       - Reading a boolean from an environment variable

  # Assets
  test_data_paths.cpp                     - Per-user data directory and what counts as installed
  test_asset_inventory.cpp                - What is installed and whether it can be used
  test_asset_profiles.cpp                 - The asset sets the builder offers
  test_asset_pack.cpp                     - Pack round trip, and refusal of paths outside the destination
  test_model_import.cpp                   - Importing a later client's models
  test_install_probe.cpp                  - Finding the archives wherever the game folder was pointed
  test_extract_progress.cpp               - Extraction progress reporting (requires StormLib)

  # Interface
  test_framexml.cpp                       - FrameXML XML parsing and the emitter
  test_framexml_takeover.cpp              - Which interface draws which element
  test_settings_schema_consistency.cpp    - Settings rows that contradict themselves
  test_settings_panel_layout.cpp          - Options panels laid out inside their frame
  test_simple_html.cpp                    - SimpleHTML item text parsing
  test_gamepad.cpp                        - Stick arithmetic and the controller binding table

  # Animation
  test_animation_ids.cpp                  - Animation ID constants
  test_locomotion_fsm.cpp                 - Locomotion state machine transitions
  test_combat_fsm.cpp                     - Combat animation state machine
  test_activity_fsm.cpp                   - Activity state machine
  test_anim_capability.cpp                - Animation capability queries
  test_indoor_shadows.cpp                 - Indoor shadow rendering

  # Rendering
  test_pass_ablation.cpp                  - Phase walk and arithmetic of the WOWEE_PASS_ABLATION report

  # Transport & Spline
  test_spline.cpp                         - CatmullRomSpline math (interpolation, binary search, looping)
  test_transport_components.cpp           - Transport clock sync and animator
  test_transport_path_repo.cpp            - TransportPathRepository (DBC loading, path inference)

  # World Map
  test_world_map.cpp                      - World map integration tests
  test_world_map_coordinate_projection.cpp - UV projection, zone/continent spatial lookups
  test_world_map_exploration_state.cpp    - Server exploration mask, local tracking
  test_world_map_map_resolver.cpp         - Cross-map navigation (Outland, Northrend)
  test_world_map_view_state_machine.cpp   - COSMIC→WORLD→CONTINENT→ZONE transitions
  test_world_map_zone_metadata.cpp        - Zone level ranges and faction labels

  # Chat
  test_chat_markup_parser.cpp             - Item link and markup parsing
  test_gm_commands.cpp                    - GM command data table and dispatch
  test_macro_evaluator.cpp                - Macro conditional evaluation
```

Python checks registered with ctest (only when Python 3 is found):

| ctest name | Script |
|---|---|
| `sweep_guard` | `tools/sweep_guard.py` - the fast interface sweeps (see below) |
| `unicorn_stub_compiles` | `tools/unicorn_stub_check.py` - the Warden stub path compiles without Unicorn |
| `upscale_mip_colour` | `tools/upscale_mip_check.py` - the upscaler keeps colour in every mip |
| `android_data_profiles` | `tools/android/test_data_profiles.py` - the patterns in the Android data profiles still match |

The Catch2 v3 amalgamated source lives at:

```
extern/catch2/
  catch_amalgamated.hpp
  catch_amalgamated.cpp
```

---

## Building the Tests

Tests **are** built by default (`option(WOWEE_BUILD_TESTS "Build tests" ON)` in `CMakeLists.txt`). Pass `-DWOWEE_BUILD_TESTS=OFF` to skip them; the snippets below pass `-DWOWEE_BUILD_TESTS=ON` only to be explicit.

### Release Build (normal)

> **Note:** Per project rules, always use `rebuild.sh` for a full clean build. Direct `cmake --build` is fine for test-only incremental builds.

```bash
# Configure (only needed once)
cmake -B build -DCMAKE_BUILD_TYPE=Release -DWOWEE_BUILD_TESTS=ON

# Build all test targets
cmake --build build --parallel $(nproc)

# Or build specific test targets
cmake --build build --target test_packet test_spline test_world_map
```

Or simply run a full rebuild (builds everything including the main binary):

```bash
./rebuild.sh      # ~10 minutes - see BUILD_INSTRUCTIONS.md
```

### Debug + ASAN/UBSan Build

A separate CMake build directory is used so ASAN flags do not pollute the Release binary.

```bash
cmake -B build_asan \
  -DCMAKE_BUILD_TYPE=Debug \
  -DWOWEE_ENABLE_ASAN=ON \
  -DWOWEE_BUILD_TESTS=ON

cmake --build build_asan --parallel $(nproc)
```

CMake will print: `Test targets: ASAN + UBSan ENABLED` when configured correctly.

---

## Running Tests

### test.sh - the unified entry point

`test.sh` is the recommended way to run tests and/or lint. It handles build-directory discovery, dependency checking, and exit-code aggregation across both steps.

```bash
# Run everything (tests + lint) - default when no flags are given
./test.sh

# Tests only (Release build)
./test.sh --test

# Tests only under ASAN+UBSan (Debug build - requires build_asan/)
./test.sh --asan

# Lint only
./test.sh --lint

# Both tests and lint explicitly
./test.sh --test --lint

# Usage summary
./test.sh --help
```

**Exit codes:**

| Outcome | Exit code |
|---|---|
| All tests passed, lint clean | `0` |
| Any test failed | `1` |
| Any lint diagnostic | `1` |
| Both test failure and lint issues | `1` |

### Running directly with ctest

```bash
# Release build
cd build
ctest --output-on-failure

# ASAN build
cd build_asan
ctest --output-on-failure

# Run one specific test suite by name
ctest --output-on-failure -R srp

# List every registered test without running any
ctest -N

# Verbose output (shows every SECTION and REQUIRE)
ctest --output-on-failure -V
```

You can also run a test binary directly for detailed Catch2 output:

```bash
./build/bin/test_srp
./build/bin/test_srp --reporter console
./build/bin/test_srp "[srp]"    # run only tests tagged [srp]
```

### Interface sweeps (sweep_guard)

`tools/sweep_guard.py` runs the fast sweeps in `tools/` and fails when any count rises above its ceiling. It is registered as the `sweep_guard` ctest, and can be run on its own:

```bash
python3 tools/sweep_guard.py          # report and exit non-zero on a regression
python3 tools/sweep_guard.py --list   # show the ceilings without running anything
```

Several sweeps read inputs the repository does not carry: the extracted interface at `Data/interface`, CSV DBCs at `Data/db`, a server source tree named by `WOWEE_SERVER_SRC`, or `build/bin/framexml_run`. A sweep whose input is missing is reported as skipped rather than passed, so a green run on a checkout without them covers less than a green run with them.

---

## Headless FrameXML runs (framexml_run)

`framexml_run` loads the real FrameXML through the real emitter and Lua bindings, with no window and no game behind it, and runs Lua expressions against it. It reports load errors and each expression's errors separately, and its exit status is the number of expressions that raised, capped at 100. It is off the default build because it compiles the client a second time:

```bash
cmake -S . -B build -DWOWEE_BUILD_FRAMEXML_RUN=ON
cmake --build build --target framexml_run
```

Pass the **expansion data directory** - the one holding `interface/` - not the `Data` root. Given the root it finds no FrameXML to load, and a report of no load errors means nothing:

```bash
./build/bin/framexml_run "$HOME/Library/Application Support/Wowee/Data/expansions/wotlk" \
  'ToggleGameMenu()' 'ChatFrame1EditBox:Show()'
```

Fonts are read from `<expansion dir>/misc/fonts`. It writes its own log (`logs/framexml_run.log`) and keeps its config under `logs/framexml_run_config` unless `WOWEE_LOG_FILE` or `WOWEE_CONFIG_ROOT` say otherwise, so it does not overwrite the client's. Run it a second time with `WOWEE_LUA_API_FALLBACK=0` before trusting a clean report: by default an unknown global answers with a stand-in rather than nil, which hides that something depended on it.

---

## Lint (clang-tidy)

The project uses clang-tidy to enforce C++20 best practices on all first-party sources under `src/`. Third-party code (anything in `extern/`) and generated files are excluded.

### Running lint

```bash
./test.sh --lint
```

Under the hood the script:

1. Locates `clang-tidy` (tries `clang-tidy`, then versions 18 down to 14).
2. Uses `run-clang-tidy` for parallel execution when available; falls back to sequential.
3. Reads `build/compile_commands.json` (generated by CMake) and lints against a copy in `build/.tidy/` with the precompiled-header arguments removed, since clang-tidy cannot read a PCH built by a different compiler.
4. Feeds GCC stdlib include paths as `-isystem` extras so clang-tidy can resolve `<vector>`, `<string>`, etc. when the compile-commands were generated with GCC.

`compile_commands.json` is regenerated automatically by any CMake configure step. If you only want to update it without rebuilding:

```bash
cmake -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### Applying auto-fixes

Some clang-tidy checks can apply fixes automatically (e.g. `modernize-*`, `readability-*`):

```bash
FIX=1 ./test.sh --lint
```

> **Caution:** Review the diff before committing - automatic fixes occasionally produce non-idiomatic results in complex template code.

### Configuration (.clang-tidy)

The active check set is defined in [.clang-tidy](.clang-tidy) at the repository root.

**Enabled check categories:**

| Category | What it catches |
|---|---|
| `bugprone-*` | Common bug patterns (signed overflow, misplaced `=`, etc.) |
| `clang-analyzer-*` | Deep flow-analysis: null dereferences, memory leaks, dead stores |
| `performance-*` | Unnecessary copies, inefficient STL usage |
| `modernize-*` (subset) | Pre-C++11 patterns that should use modern equivalents |
| `readability-*` (subset) | Control-flow simplification, redundant code |

**Notable suppressions** (`.clang-tidy` lists them all, each with the reason and its finding count):

| Suppressed check | Reason |
|---|---|
| `bugprone-easily-swappable-parameters` | High false-positive rate in graphics/math APIs |
| `clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling` | Intentional low-level buffer code in rendering |
| `performance-avoid-endl` | `std::endl` is used intentionally for logger flushing |
| `bugprone-narrowing-conversions` | Float/int conversion is what graphics maths is made of |
| `bugprone-exception-escape` | Destructors that delegate to a logging `shutdown()` |

To suppress a specific warning inline, use:

```cpp
// NOLINT(bugprone-narrowing-conversions)
uint8_t byte = static_cast<uint8_t>(value); // NOLINT
```

---

## ASAN / UBSan

AddressSanitizer (ASAN) and Undefined Behaviour Sanitizer (UBSan) are applied to all test targets, and to the `wowee` client itself, when `WOWEE_ENABLE_ASAN=ON`.

Both the test executables **and** the `catch2_main` static library are recompiled with:

```
-fsanitize=address,undefined -fno-omit-frame-pointer
```

This means any heap overflow, stack buffer overflow, use-after-free, null dereference, signed integer overflow, or misaligned access detected during a test will abort the process and print a human-readable report to stderr.

### Workflow

```bash
# 1. Configure once (only needs to be re-run when CMakeLists.txt changes)
cmake -B build_asan \
  -DCMAKE_BUILD_TYPE=Debug \
  -DWOWEE_ENABLE_ASAN=ON \
  -DWOWEE_BUILD_TESTS=ON

# 2. Build test binaries (fast incremental after the first build)
cmake --build build_asan --target test_packet test_srp   # etc.

# 3. Run
./test.sh --asan
```

### Interpreting ASAN output

A failing ASAN report looks like:

```
==12345==ERROR: AddressSanitizer: heap-buffer-overflow on address 0x602000000010
READ of size 4 at 0x602000000010 thread T0
    #0 0x... in PacketBuffer::read_uint32 src/network/packet.cpp:42
    #1 0x... in test_packet tests/test_packet.cpp:88
```

Address the issue in the source file and re-run. Do **not** suppress ASAN reports without a code fix.

---

## Adding New Tests

1. **Create** `tests/test_<name>.cpp` with a standard Catch2 v3 structure:

```cpp
#include "catch_amalgamated.hpp"

TEST_CASE("SomeFeature does X", "[tag]") {
    REQUIRE(1 + 1 == 2);
}
```

2. **Register** the test in `tests/CMakeLists.txt` with the `wowee_add_test` helper, passing every source the test needs:

```cmake
# ── test_<name> ──────────────────────────────────────────────
wowee_add_test(test_<name>
    SOURCES test_<name>.cpp
            ${TEST_COMMON_SOURCES}                          # the logger, if the code under test logs
            ${CMAKE_SOURCE_DIR}/src/<module>/<file>.cpp)    # source under test
```

The helper adds the include directories, links `catch2_main` and GLM, registers the ctest under the target name with `test_` stripped, and calls `register_test_target()`. Leave `${TEST_COMMON_SOURCES}` out when the code under test should not need the logger: the link then fails if it starts using one. A test of the packet layer uses `wowee_add_packet_test(test_<name>)` instead, which links the packet sources and their libraries. A test that needs other libraries keeps a block of its own and must call `register_test_target(test_<name>)` itself:

```cmake
add_executable(test_<name>
    test_<name>.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/<module>/<file>.cpp
)
target_include_directories(test_<name> PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_<name> SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_<name> PRIVATE catch2_main OpenSSL::SSL OpenSSL::Crypto)
add_test(NAME <name> COMMAND test_<name>)
register_test_target(test_<name>)   # required - enables ASAN propagation
```

3. **Build** and verify:

```bash
cmake --build build --target test_<name>
./test.sh --test
```

The `register_test_target()` call is **mandatory** for hand-written blocks - without it the new test will not receive ASAN/UBSan flags when `WOWEE_ENABLE_ASAN=ON`, and on Windows it will not link Winsock.

---

## CI Reference

What the GitHub workflows actually run:

| Workflow / job | What it does |
|---|---|
| `Build` → Linux (x86-64, arm64) | Configures with `-DWOWEE_BUILD_TESTS=ON`, builds, builds `wowee_editor`, then `cd build && ctest --output-on-failure` |
| `Build` → macOS (arm64, x86-64) | Configures with `-DWOWEE_BUILD_TESTS=ON`, builds, then `cd build && ctest --output-on-failure` |
| `Build` → Windows (x86-64, arm64) | Builds the client and the tests (tests are on by default); does not run them |
| `Build` → Android (arm64) | Builds the APK; no tests |
| `Security` → Sanitizer Build (ASan/UBSan) | `RelWithDebInfo` build with `-fsanitize=address,undefined` passed through `CMAKE_C_FLAGS`, `CMAKE_CXX_FLAGS` and the linker flags, then `ctest --test-dir build --output-on-failure` with `ASAN_OPTIONS=detect_leaks=1:halt_on_error=1` and `UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1` |
| `Security` → CodeQL, Semgrep | Static analysis; no tests |

clang-tidy is not run in CI. Run `./test.sh --lint` locally.

The sanitizer job's configure step, for reproducing it locally:

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DWOWEE_BUILD_TESTS=ON \
  -DCMAKE_C_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer" \
  -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined" \
  -DCMAKE_SHARED_LINKER_FLAGS="-fsanitize=address,undefined"
cmake --build build --parallel $(nproc)
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 \
  ctest --test-dir build --output-on-failure
```

> See [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md) for full platform dependency installation steps required before any CI job.
