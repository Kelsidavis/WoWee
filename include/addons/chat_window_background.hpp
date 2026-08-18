#pragma once

// The chat window's background colour, and the repair of one saved wrong.
//
// The panel behind the chat text is a white image tinted by the interface:
// ChatFrameBackground, handed to SetVertexColor and SetAlpha by
// FCF_SetWindowColor and FCF_SetWindowAlpha with whatever GetChatWindowInfo
// answered. So the colour this client stores is the whole of what that panel
// looks like, and storing white at full alpha paints an opaque slab over the
// chat with the text lost in it - white on white.
//
// It did. The fresh-window colour here was white at full alpha until 7f052e7ff,
// and every install that ran a build before it wrote that white out to
// interface_state.cfg. Fixing the default left those files alone, so the fault
// outlived the fix on every existing install - which is how it came back as a
// report after it had been fixed. Hence a repair on the way in and not only a
// better default.

namespace wowee {
namespace addons {

/// The interface's own fresh-window background: DEFAULT_CHATFRAME_COLOR is
/// black and DEFAULT_CHATFRAME_ALPHA is a quarter, both in
/// floatingchatframe.lua.
inline constexpr float kChatBackgroundDefault[4] = {0.0f, 0.0f, 0.0f, 0.25f};

/// Replace a saved background that no interface would have chosen.
///
/// Pure white is not a colour the client offers or the interface defaults to;
/// it is the fingerprint of the build whose fresh-window colour was white, so
/// it is read as unset rather than as a preference. The alpha beside it is kept
/// where it differs from that build's - somebody moved a slider to reach it -
/// and replaced where it does not.
///
/// Returns true when something was changed, which the caller logs: a setting
/// quietly rewritten is worse than the setting.
inline bool repairChatBackground(float& r, float& g, float& b, float& alpha) {
    const bool white = (r == 1.0f && g == 1.0f && b == 1.0f);
    if (!white) return false;
    r = kChatBackgroundDefault[0];
    g = kChatBackgroundDefault[1];
    b = kChatBackgroundDefault[2];
    if (alpha == 1.0f) alpha = kChatBackgroundDefault[3];
    return true;
}

} // namespace addons
} // namespace wowee
