#pragma once

/// The asset builder, shown by the client when there is nothing to play with.
///
/// With no extraction the client still reaches its login screen, but cannot
/// get past it: the callbacks that would carry somebody through are the ones
/// that need assets, so an account can be typed and nothing happens. The
/// program that fixes that shipped beside the client as a separate window
/// people had to know to go and find. This is that same window's panel, drawn
/// here, so the client asks for what it needs instead of failing quietly.
///
/// Compiled only where the extractor is: see WOWEE_HAVE_ASSET_PANEL.

#ifdef WOWEE_HAVE_ASSET_PANEL

#include "panel.hpp"

namespace wowee::ui {

class FirstRunScreen {
public:
    FirstRunScreen();

    /// Draw into the frame already open. Returns true once a build has
    /// finished and written something the client can use.
    bool render();

private:
    assets::App app_;
};

}  // namespace wowee::ui

#endif  // WOWEE_HAVE_ASSET_PANEL
