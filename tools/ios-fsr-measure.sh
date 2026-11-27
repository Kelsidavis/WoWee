#!/usr/bin/env bash
# Measure the iPad's frame time with and without FSR1 upscaling.
#
# Run this from Opencode on the Mac, with the iPad connected and unlocked.
# It does the whole comparison: push a config, play, pull the log, and print
# the numbers side by side.
#
#   tools/ios-fsr-measure.sh --udid <udid>
#   tools/ios-fsr-measure.sh --udid <udid> --quality 1     # 0.77 scale
#   tools/ios-fsr-measure.sh --udid <udid> --only fsr1     # skip the baseline
#
# What it measures, and why:
#
# The client logs a "frame budget over Ns: X frames, Y fps" line every ten
# seconds (Application::reportStageTimes) with a per-stage breakdown, and a
# "SLOW render stage 'endFrame'" line for any stage over 50 ms. The stage
# breakdown is the answer: on the iPad every stage was under 50 ms except
# endFrame, which sat at 75-80 ms. endFrame is post-process, the UI overlay,
# submit and the present wait - so it is the GPU being the bottleneck, and
# the only fix is to give it fewer pixels.
#
# FSR1 renders the scene at a fraction of the swapchain size and upscales
# into it. That is a render scale, which is what the resolution selector
# cannot be on iPad: SDL's UIKit backend has no SetWindowSize, so the
# window stays at the native 2360x1640 whatever the resolution setting says.
#
# The profile lines are logged at INFO unless WOWEE_FRAME_PROFILE is set, and
# the default log level is WARNING in a release build. A Debug build logs at
# INFO by default, so the breakdown is already in the log - no env var
# needed, and none can be set on iOS anyway.
#
# If you built Release, the "frame budget" and per-stage lines will be
# missing and only the "SLOW render stage" warnings survive. The warning is
# enough to see whether endFrame is still the stage that dominates, but the
# fps number needs a Debug build.

set -euo pipefail

UDID=""
QUALITY=1        # 0=0.77 1=0.67 2=0.59 3=1.00
ONLY="both"       # both | fsr1 | baseline
SECONDS_TO_PLAY=45
APP_ID="com.wowee.client"

usage() {
    cat <<'EOF'
Usage: tools/ios-fsr-measure.sh --udid <udid> [options]

  --udid <id>       The iPad's UDID (xcrun devicectl list devices)
  --quality <0-3>   FSR1 render scale: 0=0.77 1=0.67 2=0.59 3=1.00 (default 1)
  --only <mode>     both | fsr1 | baseline (default both)
  --play <seconds>  How long to play in each run (default 45)
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --udid)    UDID="$2"; shift 2 ;;
        --quality) QUALITY="$2"; shift 2 ;;
        --only)    ONLY="$2"; shift 2 ;;
        --play)    SECONDS_TO_PLAY="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
    esac
done

if [[ -z "$UDID" ]]; then
    echo "--udid is required. xcrun devicectl list devices finds it." >&2
    usage
    exit 1
fi

case "$ONLY" in
    both|fsr1|baseline) ;;
    *) echo "--only must be both, fsr1 or baseline" >&2; exit 1 ;;
esac

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

# The app's data container, where settings.cfg and the log live.
CONTAINER="$(xcrun devicectl device info details --device "$UDID" 2>/dev/null \
    | grep -oE 'com\.wowee\.client\.Data-[A-Za-z0-9]+' | head -1)"

if [[ -z "$CONTAINER" ]]; then
    echo "Could not find the app's data container. Is the app installed?" >&2
    echo "  xcrun devicectl device info apps --device $UDID" >&2
    exit 1
fi

echo "Container: $CONTAINER"

push_config() {
    local mode="$1" dest="$2"
    # The keys the client reads for this test. antialiasing and vsync are off
    # so the measurement is the upscaler and not the present path.
    #
    # loglevel and frameprofile are the two diagnostics that reach a device:
    # an environment variable cannot, so they are read from the config file.
    # frameprofile=1 reports the per-stage and GPU breakdowns at warning level,
    # which is the level the log keeps by default.
    cat > "$WORK/settings.cfg" <<EOF
upscaling_mode=$mode
fsr_quality=$QUALITY
antialiasing=0
vsync=0
loglevel=info
frameprofile=1
EOF
    xcrun devicectl device copy to --device "$UDID" \
        --domain-type appDataContainer --domain-identifier "$CONTAINER" \
        --source "$WORK/settings.cfg" --destination "Documents/config/settings.cfg" \
        >/dev/null
    echo "  pushed settings.cfg (upscaling_mode=$mode)"
}

pull_log() {
    local out="$1"
    xcrun devicectl device copy from --device "$UDID" \
        --domain-type appDataContainer --domain-identifier "$CONTAINER" \
        --source "Documents/config/logs/wowee.log" --destination "$out" \
        >/dev/null 2>&1 || return 1
    [[ -s "$out" ]]
}

# The two lines that carry the answer.
summarize() {
    local log="$1" label="$2"
    if [[ ! -s "$log" ]]; then
        printf '%-10s : no log pulled\n' "$label"
        return
    fi
    # "frame budget over 10s: 130 frames, 13 fps, 76.9ms each"
    local budget
    budget="$(grep -o 'frame budget over [0-9]*s: [0-9]* frames, [0-9.]* fps, [0-9.]*ms each' "$log" | tail -1)"
    # "  endFrame: 76.2ms/frame avg, 91.4ms worst, 88% of the window"
    # The log prefixes every line with a timestamp and a level, so the leading
    # spaces of the stage line are not at the start of the line.
    local stages
    stages="$(grep -E 'ms/frame avg' "$log" | sed 's/^.*\] *//' | tail -8)"
    local slow
    slow="$(grep -o "SLOW render stage '[a-zA-Z]*': [0-9.]*ms" "$log" | tail -3 | tr '\n' ' ')"

    printf '%-10s : %s\n' "$label" "${budget:-no frame budget line}"
    if [[ -n "$stages" ]]; then
        echo "$stages" | sed 's/^/             /'
    fi
    if [[ -n "$slow" ]]; then
        printf '             slow: %s\n' "$slow"
    fi
}

run_one() {
    local label="$1" mode="$2"
    echo ""
    echo "=== $label (upscaling_mode=$mode) ==="

    push_config "$mode" "$WORK"

    # A fresh log each run, so the summary cannot pick up the previous run's
    # numbers.
    xcrun devicectl device process launch --device "$UDID" "$APP_ID" --terminate-existing >/dev/null 2>&1 || true

    echo "  playing for ${SECONDS_TO_PLAY}s - move around a bit, the numbers are per-frame"
    sleep "$SECONDS_TO_PLAY"

    xcrun devicectl device process terminate --device "$UDID" "$APP_ID" >/dev/null 2>&1 || true
    sleep 1

    local log="$WORK/$label.log"
    if pull_log "$log"; then
        summarize "$log" "$label"
    else
        printf '%-10s : could not pull the log\n' "$label"
        echo "             (is the app still installed? xcrun devicectl device info apps --device $UDID)"
    fi
}

if [[ "$ONLY" == "both" || "$ONLY" == "baseline" ]]; then
    run_one baseline 0
fi
if [[ "$ONLY" == "both" || "$ONLY" == "fsr1" ]]; then
    run_one fsr1 1
fi

echo ""
echo "Done. Baseline is upscaling_mode=0 (no upscaling); fsr1 is mode=1."
echo "The number to compare is the fps in the frame budget line, and whether"
echo "endFrame is still the stage that dominates."
