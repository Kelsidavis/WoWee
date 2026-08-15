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
};

// Note the shadows column: every preset leaves them on.
//
// Low used to turn them off, which the client no longer honours - the control
// for it is off the settings panel and setShadowsEnabled holds them on, because
// turning them off loses the device. A preset that sets a value nothing acts on
// is a preset that lies about what it did, and it wrote shadows=0 to the config
// on the way past. Low leans on its short shadow distance instead, which is
// where the cost actually is.
constexpr GraphicsPresetValues kGraphicsPresets[] = {
    /* Low    */ { 600.0f, true,  100.0f, 0, false, false, 0.6f, false, 0,  25},
    /* Medium */ {1000.0f, true,  200.0f, 1, false, true,  0.6f, true,  0,  60},
    /* High   */ {1600.0f, true,  350.0f, 2, false, true,  0.8f, true,  1, 100},
    /* Ultra  */ {2400.0f, true,  500.0f, 3, true,  true,  1.2f, true,  2, 150},
};

/// The number of presets, not counting Custom - which is not a set of values
/// but the name for "these are whatever you made them".
inline constexpr int kGraphicsPresetCount =
    static_cast<int>(sizeof(kGraphicsPresets) / sizeof(kGraphicsPresets[0]));

}  // namespace ui
}  // namespace wowee
