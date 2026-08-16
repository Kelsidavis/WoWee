#!/usr/bin/env bash
# test.sh - Run the Catch2 unit tests and/or clang-tidy linter.
#
# Usage:
#   ./test.sh                    # run both lint and tests (default)
#   ./test.sh --lint             # run clang-tidy only
#   ./test.sh --test             # run ctest unit tests only
#   ./test.sh --lint --test      # explicit: run both
#   ./test.sh --asan             # run ctest under ASAN/UBSan (requires build_asan/)
#   ./test.sh --test --asan      # same as above
#   FIX=1 ./test.sh --lint       # apply clang-tidy fix suggestions (use with care)
#
# Exit code is non-zero if any lint diagnostic or test failure is reported.
#
# Build directories:
#   build/       - Release build used for normal ctest  (cmake -DCMAKE_BUILD_TYPE=Release)
#   build_asan/  - Debug+ASAN build used with --asan    (cmake -DCMAKE_BUILD_TYPE=Debug
#                                                              -DWOWEE_ENABLE_ASAN=ON
#                                                              -DWOWEE_BUILD_TESTS=ON)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
RUN_LINT=0
RUN_TEST=0
RUN_ASAN=0

for arg in "$@"; do
    case "$arg" in
        --lint) RUN_LINT=1 ;;
        --test) RUN_TEST=1 ;;
        --asan) RUN_ASAN=1; RUN_TEST=1 ;;
        --help|-h)
            sed -n '2,18p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *)
            echo "Unknown argument: $arg"
            echo "Usage: $0 [--lint] [--test] [--asan]"
            exit 1
            ;;
    esac
done

# Default: run both when no flags provided
if [[ $RUN_LINT -eq 0 && $RUN_TEST -eq 0 ]]; then
    RUN_LINT=1
    RUN_TEST=1
fi

# ---------------------------------------------------------------------------
# ── UNIT TESTS ─────────────────────────────────────────────────────────────
# ---------------------------------------------------------------------------
OVERALL_FAILED=0

if [[ $RUN_TEST -eq 1 ]]; then
    if [[ $RUN_ASAN -eq 1 ]]; then
        BUILD_TEST_DIR="$SCRIPT_DIR/build_asan"
        TEST_LABEL="ASAN+UBSan"
    else
        BUILD_TEST_DIR="$SCRIPT_DIR/build"
        TEST_LABEL="Release"
    fi

    if [[ ! -d "$BUILD_TEST_DIR" ]]; then
        echo "Build directory not found: $BUILD_TEST_DIR"
        if [[ $RUN_ASAN -eq 1 ]]; then
            echo "Configure it with:"
            echo "  cmake -B build_asan -DCMAKE_BUILD_TYPE=Debug -DWOWEE_ENABLE_ASAN=ON -DWOWEE_BUILD_TESTS=ON"
        else
            echo "Run cmake first:  cmake -B build -DCMAKE_BUILD_TYPE=Release -DWOWEE_BUILD_TESTS=ON"
        fi
        exit 1
    fi

    # Check that CTestTestfile.cmake exists (tests were configured)
    if [[ ! -f "$BUILD_TEST_DIR/CTestTestfile.cmake" ]]; then
        echo "CTestTestfile.cmake not found in $BUILD_TEST_DIR - tests not configured."
        echo "Re-run cmake with -DWOWEE_BUILD_TESTS=ON"
        exit 1
    fi

    echo "──────────────────────────────────────────────"
    echo " Running unit tests [$TEST_LABEL]"
    echo "──────────────────────────────────────────────"
    if ! (cd "$BUILD_TEST_DIR" && ctest --output-on-failure); then
        OVERALL_FAILED=1
        echo ""
        echo "One or more unit tests FAILED."
    else
        echo ""
        echo "All unit tests passed."
    fi
fi

# ---------------------------------------------------------------------------
# ── LINT ───────────────────────────────────────────────────────────────────
# ---------------------------------------------------------------------------
if [[ $RUN_LINT -eq 0 ]]; then
    exit $OVERALL_FAILED
fi

# Dependency check
CLANG_TIDY=""
for candidate in clang-tidy clang-tidy-18 clang-tidy-17 clang-tidy-16 clang-tidy-15 clang-tidy-14; do
    if command -v "$candidate" >/dev/null 2>&1; then
        CLANG_TIDY="$candidate"
        break
    fi
done

if [[ -z "$CLANG_TIDY" ]]; then
    echo "clang-tidy not found. Install it with:"
    echo "  sudo apt-get install clang-tidy"
    exit 1
fi

echo "──────────────────────────────────────────────"
echo " Running clang-tidy lint"
echo "──────────────────────────────────────────────"
echo "Using: $($CLANG_TIDY --version | head -1)"

# run-clang-tidy runs checks in parallel; fall back to sequential if absent.
RUN_CLANG_TIDY=""
for candidate in run-clang-tidy run-clang-tidy-18 run-clang-tidy-17 run-clang-tidy-16 run-clang-tidy-15 run-clang-tidy-14; do
    if command -v "$candidate" >/dev/null 2>&1; then
        RUN_CLANG_TIDY="$candidate"
        break
    fi
done

# Build database check
COMPILE_COMMANDS="$SCRIPT_DIR/build/compile_commands.json"
if [[ ! -f "$COMPILE_COMMANDS" ]]; then
    echo "compile_commands.json not found at $COMPILE_COMMANDS"
    echo "Run cmake first:  cmake -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
    exit 1
fi

# clang-tidy cannot read a precompiled header built by a different compiler,
# and it usually is one: the PCH comes from whatever CXX cmake picked, while
# clang-tidy comes from whatever LLVM is on PATH. Mismatched, every file fails
# with "PCH file uses an older format that is no longer supported" and the real
# diagnostics never appear.
#
# So lint against a copy of the database with the PCH arguments removed. The
# headers are still included normally -- the PCH is only a parse cache.
TIDY_DB_DIR="$SCRIPT_DIR/build/.tidy"
mkdir -p "$TIDY_DB_DIR"
python3 - "$COMPILE_COMMANDS" "$TIDY_DB_DIR/compile_commands.json" <<'PYEOF'
import json, re, sys
src, dst = sys.argv[1], sys.argv[2]
db = json.load(open(src))
strip = re.compile(r'\s+-Xclang\s+-include-pch\s+-Xclang\s+\S+|\s+-include-pch\s+\S+|\s+-Winvalid-pch')
for entry in db:
    if 'command' in entry:
        entry['command'] = strip.sub('', entry['command'])
    if 'arguments' in entry:
        args, out = entry['arguments'], []
        i = 0
        while i < len(args):
            if args[i] == '-include-pch':
                i += 2
            elif args[i] == '-Xclang' and i + 1 < len(args) and args[i + 1] == '-include-pch':
                i += 4
            else:
                out.append(args[i]); i += 1
        entry['arguments'] = out
json.dump(db, open(dst, 'w'))
PYEOF
COMPILE_COMMANDS="$TIDY_DB_DIR/compile_commands.json"

# ---------------------------------------------------------------------------
# Source files to check (first-party only)
# ---------------------------------------------------------------------------
mapfile -t SOURCE_FILES < <(
    find "$SCRIPT_DIR/src" -type f \( -name '*.cpp' -o -name '*.cxx' -o -name '*.cc' \) | sort
)

if [[ ${#SOURCE_FILES[@]} -eq 0 ]]; then
    echo "No source files found in src/"
    exit 0
fi

echo "Linting ${#SOURCE_FILES[@]} source files..."

# ---------------------------------------------------------------------------
# Resolve GCC C++ stdlib include paths for clang-tidy
#
# compile_commands.json is generated by GCC. When clang-tidy processes those
# commands with its own clang driver it cannot locate GCC's libstdc++ headers.
# We query GCC for its include search list and forward each path as an
# -isystem extra argument so clang-tidy can find <vector>, <string>, etc.
# ---------------------------------------------------------------------------
EXTRA_TIDY_ARGS=()   # for direct clang-tidy:  --extra-arg=...
EXTRA_RUN_ARGS=()    # for run-clang-tidy:     -extra-arg=...
if command -v gcc >/dev/null 2>&1; then
    # Prepend clang's own resource include dir first so clang uses its own
    # versions of xmmintrin.h, ia32intrin.h, etc. rather than GCC's.
    clang_resource_inc="$($CLANG_TIDY -print-resource-dir 2>/dev/null || true)/include"
    if [[ -d "$clang_resource_inc" ]]; then
        EXTRA_TIDY_ARGS+=("--extra-arg=-isystem${clang_resource_inc}")
        EXTRA_RUN_ARGS+=("-extra-arg=-isystem${clang_resource_inc}")
    fi

    while IFS= read -r inc_path; do
        [[ -d "$inc_path" ]] || continue
        # Skip the GCC compiler built-in include dir - clang's resource dir above
        # provides compatible replacements for xmmintrin.h, ia32intrin.h, etc.
        [[ "$inc_path" == */gcc/* ]] && continue
        EXTRA_TIDY_ARGS+=("--extra-arg=-isystem${inc_path}")
        EXTRA_RUN_ARGS+=("-extra-arg=-isystem${inc_path}")
    done < <(
        gcc -E -x c++ - -v < /dev/null 2>&1 \
        | sed -n '/#include <\.\.\.> search starts here:/,/End of search list\./p' \
        | grep '^ ' \
        | sed 's/^ //'
    )
fi

# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------
FIX="${FIX:-0}"
FIX_FLAG=""
if [[ "$FIX" == "1" ]]; then
    FIX_FLAG="-fix"
    echo "Fix mode enabled - applying suggested fixes."
fi

LINT_FAILED=0

if [[ -n "$RUN_CLANG_TIDY" ]]; then
    echo "Running via $RUN_CLANG_TIDY (parallel)..."
    # run-clang-tidy takes a source-file regex; match our src/ tree
    SRC_REGEX="$(echo "$SCRIPT_DIR/src" | sed 's|/|\\/|g')"
    "$RUN_CLANG_TIDY" \
        -clang-tidy-binary "$CLANG_TIDY" \
        -p "$TIDY_DB_DIR" \
        $FIX_FLAG \
        "${EXTRA_RUN_ARGS[@]}" \
        "$SRC_REGEX" || LINT_FAILED=$?
else
    echo "run-clang-tidy not found; running sequentially..."
    for f in "${SOURCE_FILES[@]}"; do
        "$CLANG_TIDY" \
            -p "$TIDY_DB_DIR" \
            $FIX_FLAG \
            "${EXTRA_TIDY_ARGS[@]}" \
            "$f" || LINT_FAILED=$?
    done
fi

# ---------------------------------------------------------------------------
# Result
# ---------------------------------------------------------------------------
if [[ $LINT_FAILED -ne 0 ]]; then
    echo ""
    echo "clang-tidy reported issues. Fix them or add suppressions in .clang-tidy."
    OVERALL_FAILED=1
else
    echo ""
    echo "Lint passed."
fi

exit $OVERALL_FAILED
