#pragma once

namespace wowee {
namespace core {

/// How far the cursor may travel between press and release and still be a
/// click rather than a drag.
///
/// One number, because two places act on it: the camera arms its rotation
/// when the drag clears this, and the world click targets what is under the
/// cursor when it does not. They used to measure different things - the
/// camera summed |dx|+|dy| over every motion event against a flat 5 pixels,
/// the click took the straight line from press to release against this - so a
/// trackpad click that wandered a few pixels and came back armed the camera
/// while still counting as a click. The view swung and the target was picked,
/// or on a slightly wider wander the view swung and nothing was targeted.
///
/// Scaled by the window, with a floor: five pixels is a third of a percent of
/// a 1280-wide window and a tenth of that on a 3840-wide one, so a flat number
/// is a different gesture on every screen.
float clickDragThreshold();

} // namespace core
} // namespace wowee
