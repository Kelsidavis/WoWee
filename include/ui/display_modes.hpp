// display_modes.hpp - the resolutions this client offers, in one place.
//
// Both interfaces put a resolution picker on the same window. This client's
// settings panel had the list inside the function that drew it, and FrameXML's
// video panel asked GetScreenResolutions, which answered the current size and
// nothing else - a dropdown with one entry, whose SetScreenResolution did
// nothing anyway. Sharing the list rather than writing a second one, because
// the *position* in it is the protocol: the video panel's dropdown carries an
// index, not a size, and hands that index back to SetScreenResolution.
#pragma once

#include <string>

namespace wowee {
namespace ui {

inline constexpr int kDisplayResolutions[][2] = {
    {1280, 720},
    {1600, 900},
    {1920, 1080},
    {2560, 1440},
    {3840, 2160},
};
inline constexpr int kNumDisplayResolutions =
    static_cast<int>(sizeof(kDisplayResolutions) / sizeof(kDisplayResolutions[0]));

/// "1920x1080". The video panel splits on the x to work out whether to tag a
/// mode as widescreen, so the separator is part of the contract rather than
/// presentation.
inline std::string displayResolutionLabel(int index) {
    if (index < 0 || index >= kNumDisplayResolutions) return {};
    return std::to_string(kDisplayResolutions[index][0]) + "x" +
           std::to_string(kDisplayResolutions[index][1]);
}

/// Which entry a window of this size is, or the closest one below it - never
/// -1, because the dropdown selects by index and nothing is not a selection.
/// A window dragged to some size of its own lands on the largest mode it still
/// covers, which is what the picker should show as current.
inline int displayResolutionIndexFor(int w, int h) {
    int best = 0;
    for (int i = 0; i < kNumDisplayResolutions; ++i) {
        if (kDisplayResolutions[i][0] == w && kDisplayResolutions[i][1] == h) return i;
        if (kDisplayResolutions[i][0] <= w && kDisplayResolutions[i][1] <= h) best = i;
    }
    return best;
}

} // namespace ui
} // namespace wowee
