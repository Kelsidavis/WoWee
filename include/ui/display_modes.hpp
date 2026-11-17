// display_modes.hpp - the resolutions this client offers, in one place.
//
// Both interfaces put a resolution picker on the same window. This client's
// settings panel had the list inside the function that drew it, and FrameXML's
// video panel asked GetScreenResolutions, which answered the current size and
// nothing else - a dropdown with one entry, whose SetScreenResolution did
// nothing anyway. Sharing the list rather than writing a second one, because
// the *position* in it is the protocol: the video panel's dropdown carries an
// index, not a size, and hands that index back to SetScreenResolution.
//
// The list is built for the device it is running on, because a fixed list of
// desktop modes is the wrong list on a tablet. A tablet's window cannot be
// resized, so a resolution chosen there is the size the world is *drawn* at
// and the upscaler scales it onto the screen. The entries that make sense
// there are the screen's own size and fractions of it - and they are read
// from the screen, not written down, so the list is right on any device.
#pragma once

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

namespace wowee {
namespace ui {

/// One entry in the resolution list: a size the world can be drawn at.
struct DisplayResolution {
    int width = 0;
    int height = 0;
    /// Whether this entry is the screen's own size, and the world is drawn
    /// at it directly rather than scaled up from something smaller.
    bool native = false;
};

/// The resolutions offered on a device whose window cannot be resized.
///
/// The screen's own size first, and then fractions of it: three quarters,
/// half, and a third. Those are the sizes the upscaler scales up from, and
/// they are the only choices that mean anything here - a list of desktop
/// modes would offer shapes this screen does not have, and every one of them
/// would be a stretch.
///
/// The fractions are of the *smaller* dimension, so the picture keeps the
/// screen's shape at every size. A screen is not always wider than it is
/// tall - a tablet in portrait is taller - and dividing the wrong one would
/// ask for a resolution the screen cannot hold.
inline std::vector<DisplayResolution> displayResolutionsForScreen(int screenWidth,
                                                                 int screenHeight) {
    std::vector<DisplayResolution> list;
    if (screenWidth <= 0 || screenHeight <= 0) return list;

    list.push_back({screenWidth, screenHeight, true});

    // Fractions of the screen, each one both dimensions scaled by the same
    // factor - which is what keeps the picture's shape. Deriving one
    // dimension from the other undoes the scaling: halving the height and
    // recomputing the width from the aspect gives the original size back.
    //
    // Rounded to even numbers, because a Vulkan image wants an even size and
    // an odd one would be rounded again somewhere else.
    const float scales[] = {0.75f, 0.5f, 1.0f / 3.0f};
    for (float scale : scales) {
        const int w = static_cast<int>(static_cast<float>(screenWidth) * scale);
        const int h = static_cast<int>(static_cast<float>(screenHeight) * scale);
        if (w < 64 || h < 64) continue;   // too small to be worth offering
        const int evenW = w & ~1;
        const int evenH = h & ~1;
        // The same size twice would be a list with a duplicate in it, and the
        // dropdown carries a position rather than a size - so a duplicate
        // would be two positions for one thing.
        bool duplicate = false;
        for (const auto& existing : list) {
            if (existing.width == evenW && existing.height == evenH) { duplicate = true; break; }
        }
        if (!duplicate) list.push_back({evenW, evenH, false});
    }
    return list;
}

/// The resolutions offered on a device whose window can be resized.
///
/// The desktop list, written down. These are window sizes, and they are
/// picked by dragging or by choosing a mode the display offers.
inline std::vector<DisplayResolution> displayResolutionsForDesktop() {
    static constexpr int kModes[][2] = {
        {1280, 720},
        {1600, 900},
        {1920, 1080},
        {2560, 1440},
        {3840, 2160},
    };
    std::vector<DisplayResolution> list;
    for (const auto& mode : kModes) list.push_back({mode[0], mode[1], false});
    return list;
}

/// "1920x1080". The video panel splits on the x to work out whether to tag a
/// mode as widescreen, so the separator is part of the contract rather than
/// presentation.
inline std::string displayResolutionLabel(const DisplayResolution& resolution) {
    return std::to_string(resolution.width) + "x" +
           std::to_string(resolution.height);
}

/// Which entry a size is, or the closest one below it - never -1, because the
/// dropdown selects by index and nothing is not a selection. A window dragged
/// to some size of its own lands on the largest mode it still covers, which
/// is what the picker should show as current.
///
/// A size whose width and height are swapped is the same resolution on a
/// device that rotated: a tablet's screen goes from 2360x1640 to 1640x2360,
/// and the half-scale render target goes from 1180x820 to 820x1180. Without
/// that match the "largest mode it covers" fallback picks a different
/// fraction on each orientation - 1180x820 fits only 546x786 in portrait,
/// and 546x786 fits nothing in landscape, which snaps to the native screen
/// size - and the settings file is rewritten at every step. Two rotations
/// were enough to take an iPad from a 0.97 MP render target to the panel's
/// 3.87 MP one. Match the rotated size first so the fraction survives.
inline int displayResolutionIndexFor(const std::vector<DisplayResolution>& list, int w, int h) {
    if (list.empty()) return 0;
    for (std::size_t i = 0; i < list.size(); ++i) {
        if (list[i].width == w && list[i].height == h) return static_cast<int>(i);
    }
    for (std::size_t i = 0; i < list.size(); ++i) {
        if (list[i].width == h && list[i].height == w) return static_cast<int>(i);
    }
    int best = 0;
    for (std::size_t i = 0; i < list.size(); ++i) {
        if (list[i].width <= w && list[i].height <= h) best = static_cast<int>(i);
    }
    return best;
}

} // namespace ui
} // namespace wowee
