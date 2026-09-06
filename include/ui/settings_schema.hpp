#pragma once

/**
 * settings_schema.hpp - the client's settings, described once.
 *
 * The settings window draws these as ImGui controls. FrameXML's Interface
 * Options needs the same list to build its own panels out of, and an addon that
 * asks what this client can be told needs it too. Three readers, one list.
 *
 * Only the settings with no Blizzard equivalent belong here. The six a Blizzard
 * panel already has a working control for - view distance, mouse speed, the
 * minimap clock, friendly nameplates, ground clutter and the sound effects
 * volume - are bound to their CVar instead, in kClientCVars, so that FrameXML's
 * own control drives them rather than a second one appearing beside it. Those
 * six are named on the root panel, so that a player looking for one is told
 * where it is rather than concluding it is missing.
 *
 * Everything else this client can be told is here, in the order it should be
 * read: a category is one panel, and a section is one heading on it.
 */

#include <cstddef>
#include <string>

namespace wowee {
namespace ui {

/// What kind of control a setting wants.
enum class SettingKind {
    Bool,   ///< a checkbox
    Int,    ///< a slider over whole numbers
    Float,  ///< a slider over a range
    Enum,   ///< a dropdown; the value is the chosen index, written as a number
};

/// One setting, as something outside this client can understand it.
///
/// `section` and `tooltip` exist for the panels rather than for the value: a
/// list of sixty controls in one column is not an options screen, and a control
/// named "Jitter Sign" is not one either without a line saying what it is for.
struct SettingDesc {
    const char* key;       ///< what get/set answer to; stable, not shown
    const char* label;     ///< what a person reads
    SettingKind kind;
    float minValue;        ///< ignored for Bool and Enum
    float maxValue;
    float step;
    const char* category;  ///< which panel it belongs on
    const char* section;   ///< the heading above it on that panel; "" continues the last
    const char* tooltip;   ///< one or two lines shown on hover; "" for none
    /// Enum only: the labels, separated by '|', in index order.
    ///
    /// The index is what is written to the config file, not the label - so this
    /// list can be added to at the end and not reordered or inserted into. Put
    /// a 165 between the frame cap's 144 and 240 and every player who had
    /// chosen 240 has chosen 165, silently, at their next start. Nothing here
    /// would notice: the count still matches the range, every label is still
    /// non-empty, and the control still shows what the file now means.
    ///
    /// Renaming a label is safe, its index not having moved. Retiring one is
    /// not, and wants the value migrated on load rather than the row deleted.
    const char* choices;
    /// What this setting is when nobody has chosen. 0 or 1 for a Bool, the
    /// chosen index for an Enum.
    ///
    /// Here so that "restore defaults" is one fact rather than several. It was
    /// three lists before - one in each of the settings window's three restore
    /// buttons - and the options panels had no defaults at all, so the button
    /// the game puts on every one of them was a function that did nothing.
    float defaultValue;
    /// When this control is worth offering, as a test against another setting.
    ///
    /// "" is always. "key" is whenever that setting is on. "key=2" and "key!=2"
    /// compare its value. A control whose test fails is drawn greyed rather
    /// than hidden, so the panel does not change shape as things are switched
    /// on and off - and so that a player can see the setting exists and what it
    /// depends on.
    ///
    /// The settings window has always done this by wrapping each dependent
    /// control in an `if`. The options panels had no way to know, so they
    /// offered the FSR quality dropdown with upscaling off and the
    /// anti-aliasing dropdown while FSR 3 was doing its own - controls that
    /// answer, save, and change nothing.
    const char* enabledWhen = "";
    /// Where the value lives, when it is not a field of this client's own.
    ///
    /// "" is a field of SettingsPanel or ChatSettings, reached through the
    /// binding tables and saved in settings.cfg. The rest are the game's own
    /// stores: the rows that came off FrameXML's Interface pages read and
    /// write them exactly as the controls there did, so the interface sees
    /// the same CVar and the same CVAR_UPDATE it always has.
    ///
    ///   "cvar:NAME"     the CVar store, through GetCVar and SetCVar
    ///   "click:ACTION"  a modified click - GetModifiedClick and SetModifiedClick
    ///   "lua:GET|SET"   two globals: GET() answers it and SET(v) applies it.
    ///                   The cloak and the helm, which the server keeps.
    const char* store = "";
    /// Enum rows on a store: what each choice is kept as, '|'-separated in
    /// the order of `choices`. "" keeps the index, as a field does. Blizzard's
    /// dropdowns kept words - "none", "ALT" - and the interface reads those.
    const char* values = "";
    /// Lua run after a store is written, with the new value in `v`: what the
    /// game's own control did beyond writing its CVar - the global the
    /// interface reads, and the refresh that made the change show.
    const char* after = "";
};

/// Whether `enabledWhen` is satisfied, given a way to read the other setting.
///
/// Shared so that the settings window and the options panels cannot disagree
/// about when a control is live - the point of the field is that there is one
/// answer, and two readers of it.
template <typename ReadSetting>
bool settingEnabled(const SettingDesc& desc, ReadSetting read) {
    const std::string test = desc.enabledWhen;
    if (test.empty()) return true;
    const std::size_t bang = test.find("!=");
    if (bang != std::string::npos) {
        return read(test.substr(0, bang)) != test.substr(bang + 2);
    }
    const std::size_t eq = test.find('=');
    if (eq != std::string::npos) {
        return read(test.substr(0, eq)) == test.substr(eq + 1);
    }
    const std::string value = read(test);
    return !value.empty() && value != "0";
}

/// A setting's value as a string, the way a CVar carries one.
///
/// A whole number with no decimal point, a fraction without the trailing zeros.
/// std::to_string gives six decimals for everything, and the options panels
/// compare some of these as strings - a checkbox tests `value == "1"`, which
/// "1.000000" fails, and the box unticks itself every time the panel opens.
///
/// Written twice before this, once on each side of the bridge, ninety minutes
/// apart.
inline std::string settingNumberText(double v) {
    if (v == static_cast<long long>(v)) return std::to_string(static_cast<long long>(v));
    std::string s = std::to_string(v);
    while (s.size() > 1 && s.back() == '0') s.pop_back();
    if (!s.empty() && s.back() == '.') s.pop_back();
    return s;
}

/// Whether a setting string means on. Empty and "0" are the only falses - a
/// CVar arrives as a string, and in Lua every string including "0" is true, so
/// the test cannot be left to the caller.
inline bool settingIsOn(const std::string& v) { return !v.empty() && v != "0"; }

/// Every client setting FrameXML has no control of its own for.
///
/// The categories match the settings window's tabs so the two read the same way
/// round.
///
/// Adding one means seven places, not one. Each was a bug here before it was a
/// list, and each has a test that names the setting when it is missed:
///
///   1. A row here.
///   2. A pending field on SettingsPanel.
///   3. A FieldBinding row, or setSettingValue has nowhere to put the value and
///      answers false to everything that tries.
///   4. A branch in applySettingSideEffects, so changing it does something.
///   5. A line in GameScreen that applies it once the renderer exists. Applying
///      it while the config is being read cannot work - the constructor reads
///      the file and services are injected afterwards - and looks like it does,
///      because the control shows the saved value either way. That was field of
///      view: the camera kept the sixty degrees it is built with, and moving
///      the slider was the only thing that ever set it.
///   6. A line in GameScreen::saveSettings, or it works all session and is gone
///      at the next login.
///   7. A line in GameScreen::loadSettings, clamped to the range in the row.
///
/// And if the setting is kept twice - a pending field and a live member - both
/// halves move together. Every item in the minimap's context menu wrote one
/// half and not the other.
///
/// The tests are settings_apply_on_load and settings_schema_consistency. They
/// read the source, so they hold for a setting added after they were written;
/// each one was checked against the bug it describes before being trusted.
const SettingDesc* clientSettingsSchema(std::size_t& count);

/// The range a setting's own row declares.
///
/// A slider's limits were written again wherever the value was clamped on the
/// way in from settings.cfg, so widening a row moved the control without moving
/// the clamp behind it, and the setting silently snapped back on the next
/// start. Ask the row instead. Returns false for a key with no row, leaving the
/// arguments untouched.
bool settingRange(const std::string& key, float& lo, float& hi);

}  // namespace ui
}  // namespace wowee
