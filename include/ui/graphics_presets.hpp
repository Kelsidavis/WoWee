#pragma once

/// What the quality presets mean, for both places that offer them.
///
/// The settings window and the login screen each had their own copy of these
/// numbers, and they had drifted: the same preset named the same thing meant a
/// different picture depending on which one you chose it from. High was 350
/// yards of shadow with 4x MSAA and no FXAA in game, and 250 yards with 2x and
/// FXAA on at the login screen; Medium differed in clutter and multisampling;
/// Low still turned shadows off there after the client stopped honouring that.
///
/// It carries no includes, so the login screen, the settings panel and a test
/// can all read the same rows.

#include "ui/graphics_defaults.hpp"

namespace wowee {
namespace ui {

/// What each quality preset means, in the order Low, Medium, High, Ultra.
///
/// That order is what the config file holds. graphics_preset is written as an
/// index - 0 for Custom, then one per row here - so a preset added at the front
/// or in the middle changes what every saved file means, and a player who chose
/// Ultra comes back on High. Add at the end, or migrate the stored value.
///
/// The same is true of the antiAliasing and parallaxQuality columns below: each
/// is an index into a dropdown whose order is also persisted, so a row here can
/// be wrong in two directions at once if either list is reordered. See the note
/// on SettingDesc::choices.
///
/// One row per preset, where there used to be a block per preset - the same
/// ten settings assigned and then pushed at the renderer four times over. The
/// blocks had drifted apart, as four copies of one fact do:
///
///   * Low set a shadow distance of 100 and never told the renderer, so the
///     field said 100 and the shadows stayed at whatever the last preset left.
///   * Only Ultra had an opinion about FXAA. Going from Ultra to Low turned
///     everything else down and left FXAA running.
///   * Low left the normal map strength and parallax quality at whatever they
///     were, which does not matter while both are off and does the moment one
///     is switched back on by hand.
///
/// Reading them as a table is also what lets a preset be recognised again
/// afterwards without writing the numbers out a second time.
struct GraphicsPresetValues {
    float viewDistance;
    bool  shadows;
    float shadowDistance;
    int   antiAliasing;      ///< index into the four the panel offers
    bool  fxaa;
    bool  normalMapping;
    float normalMapStrength;
    bool  parallax;
    int   parallaxQuality;
    int   groundClutter;     ///< percent
    /// Ground cover from the terrain's own ground-effect data. Stated on every
    /// row rather than only the one that wants it: a preset that leaves a field
    /// alone is what left FXAA running after a drop from Ultra to Low.
    bool  grass;
    int   grassDensity;      ///< percent of what the terrain asks for
    int   grassHeight;       ///< percent
    int   grassDistance;     ///< yards
};

// Note the shadows column: every preset leaves them on.
//
// Low used to turn them off, which the client no longer honours - the control
// for it is off the settings panel and setShadowsEnabled holds them on, because
// turning them off loses the device. A preset that sets a value nothing acts on
// is a preset that lies about what it did, and it wrote shadows=0 to the config
// on the way past. Low leans on its short shadow distance instead, which is
// where the cost actually is.
// And the FXAA column: off on every row. Ultra used to turn it on over 8x
// MSAA, where there is nothing left for it to find and its sub-pixel filter
// only softened the resolve. It stays in the panel for anyone who wants it,
// which mostly means anyone running without MSAA.
constexpr GraphicsPresetValues kGraphicsPresets[] = {
    /* Low    */ { .viewDistance = 600.0f, .shadows = true,  .shadowDistance = 100.0f, .antiAliasing = 0, .fxaa = false, .normalMapping = false, .normalMapStrength = 0.6f, .parallax = false, .parallaxQuality = 0,  .groundClutter = 25,  .grass = false, .grassDensity = 70, .grassHeight = 50, .grassDistance = 215},
    /* Medium */ {.viewDistance = 1000.0f, .shadows = true,  .shadowDistance = 200.0f, .antiAliasing = 1, .fxaa = false, .normalMapping = true,  .normalMapStrength = 0.6f, .parallax = true,  .parallaxQuality = 0,  .groundClutter = 60,  .grass = false, .grassDensity = 70, .grassHeight = 50, .grassDistance = 215},
    /* High   */ {.viewDistance = 1600.0f, .shadows = true,  .shadowDistance = 350.0f, .antiAliasing = 2, .fxaa = false, .normalMapping = true,  .normalMapStrength = 0.8f, .parallax = true,  .parallaxQuality = 1, .groundClutter = 100, .grass = false, .grassDensity = 70, .grassHeight = 50, .grassDistance = 215},
    /* Ultra  */ {.viewDistance = 2400.0f, .shadows = true,  .shadowDistance = 500.0f, .antiAliasing = 3, .fxaa = false, .normalMapping = true,  .normalMapStrength = 1.2f, .parallax = true,  .parallaxQuality = 2, .groundClutter = 150, .grass = true,  .grassDensity = 70, .grassHeight = 50, .grassDistance = 215},
};

/// The number of presets, not counting Custom - which is not a set of values
/// but the name for "these are whatever you made them".
inline constexpr int kGraphicsPresetCount =
    static_cast<int>(sizeof(kGraphicsPresets) / sizeof(kGraphicsPresets[0]));

}  // namespace ui
}  // namespace wowee
