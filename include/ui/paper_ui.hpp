#pragma once

// The login screen's controls, drawn by hand.
//
// The screen behind them is a crayon drawing on paper, and an ImGui window
// sitting on top of it looked like a debugger left open over someone's
// artwork - a grey title bar, a drag handle, and four identical buttons in a
// row. None of that is fixable by restyling ImGui, because what reads as
// "ImGui" is the shape of the controls themselves: the flat rectangles, the
// label-on-the-right, the uniform spacing. So the login screen stopped asking
// for them.
//
// What is here is the smallest set of controls that screen needs, drawn into
// an ImDrawList as ink on paper: a sheet with a hand-drawn border, fields
// ruled like a form, a crayon button, a dropdown, a checkbox and a slider.
// The editing behind a field is TextEdit's, which knows nothing about any of
// this; what this adds is where the caret is on screen, what the mouse is
// pointing at, and which control has the keyboard.
//
// It is immediate mode, like everything else in this client's interface:
// controls are named by a string literal, keep no object of their own, and
// report what happened during the call that drew them. Only the handful of
// things that genuinely span frames - which control has focus, how far a long
// value has scrolled inside its box, whether a dropdown is open - live in the
// PaperUI itself.
//
// Nothing in here is specific to logging in. It is kept general enough that
// the screens after it could be moved off ImGui the same way, but it is not
// built out beyond what the login screen actually uses.

#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <imgui.h>

namespace wowee::ui {

class TextEdit;

/// The ink, paper and crayon of it.
///
/// Chosen against the drawing the login screen sits on: paper a shade
/// brighter than the photographed sheet behind it, so the card reads as
/// something laid on top rather than as part of it, and every line colour
/// taken from a crayon in the picture.
struct PaperTheme {
    ImU32 paperTop     = IM_COL32(0xFB, 0xF6, 0xEA, 0xFF);
    ImU32 paperBottom  = IM_COL32(0xEF, 0xE4, 0xCD, 0xFF);
    ImU32 paperEdge    = IM_COL32(0xD3, 0xC0, 0x9C, 0xFF);
    ImU32 shadow       = IM_COL32(0x2A, 0x1C, 0x10, 0x38);

    ImU32 ink          = IM_COL32(0x5A, 0x3A, 0x22, 0xFF);  ///< borders, headings
    ImU32 inkSoft      = IM_COL32(0x86, 0x64, 0x44, 0xFF);  ///< labels
    ImU32 pencil       = IM_COL32(0xA3, 0x91, 0x77, 0xFF);  ///< hints, disabled
    ImU32 fieldFill    = IM_COL32(0xFF, 0xFD, 0xF6, 0xC8);

    ImU32 crayonRed    = IM_COL32(0xC8, 0x42, 0x2F, 0xFF);
    ImU32 crayonRedLit = IM_COL32(0xDC, 0x55, 0x3F, 0xFF);
    ImU32 crayonRedDim = IM_COL32(0x9E, 0x31, 0x22, 0xFF);
    ImU32 crayonBlue   = IM_COL32(0x3F, 0x6F, 0xA8, 0xFF);
    ImU32 crayonGreen  = IM_COL32(0x3F, 0x7A, 0x3A, 0xFF);
    ImU32 highlighter  = IM_COL32(0xF6, 0xD9, 0x6B, 0x9C);
    ImU32 onCrayon     = IM_COL32(0xFF, 0xF8, 0xE8, 0xFF);  ///< text over a filled button

    /// The mount a picture sits in, and how far the picture is lifted towards
    /// the page.
    ///
    /// A character preview is a 3D scene lit for a dark interface - it clears
    /// to near black and stands its subject in front of a racial backdrop -
    /// and dropped straight onto cream paper it reads as a hole cut in the
    /// page. Three things close the gap. The mount is a warm tan rather than
    /// the white a photo mount would usually be, so paper to picture is a
    /// graded step rather than one jump through something brighter than
    /// either. The picture is veiled in the paper's own colour, which lifts
    /// its blacks the way a print left in the light fades. And its edges are
    /// feathered into the mount, so it stops without a line around it.
    ///
    /// `photoFade` is the knob: 0 leaves the preview exactly as rendered, and
    /// more of it washes further towards the page.
    ImU32 photoMat     = IM_COL32(0xE4, 0xD5, 0xB7, 0xFF);
    float photoFade    = 0.24f;
    /// Warms the picture on the way down. A multiply, so it only ever takes
    /// blue out of a cold render rather than brightening anything.
    ImU32 photoTint    = IM_COL32(0xFF, 0xF4, 0xE4, 0xFF);
};

/// `col` at `a` of its opacity, 0 to 1.
///
/// Free rather than a member because both the kit and the screens drawing
/// with it need it, and neither of them owns it.
inline ImU32 paperFade(ImU32 col, float a) {
    const float clamped = a < 0.0f ? 0.0f : (a > 1.0f ? 1.0f : a);
    const ImU32 alpha = static_cast<ImU32>(clamped * 255.0f + 0.5f);
    return (col & 0x00FFFFFFu) | (alpha << IM_COL32_A_SHIFT);
}

/// Which of the two draw lists the next controls go into.
///
/// The sheet and its contents go on the page; anything that has to cover them
/// - an open dropdown, a modal and the scrim under it - goes on the overlay.
/// Two lists rather than one ordering rule, because ImGui's background and
/// foreground lists already stack in that order and nothing else has to know.
enum class PaperLayer { Page, Overlay };

/// A running y down a fixed column, so a panel reads as a list of rows rather
/// than as a column of arithmetic.
struct Column {
    float x0 = 0.0f;
    float x1 = 0.0f;
    float y = 0.0f;

    [[nodiscard]] float width() const { return x1 - x0; }
    [[nodiscard]] ImVec2 at() const { return {x0, y}; }

    /// Takes `h` of vertical room and hands back the rect it occupied.
    std::pair<ImVec2, ImVec2> row(float h) {
        const ImVec2 a(x0, y);
        const ImVec2 b(x1, y + h);
        y += h;
        return {a, b};
    }
    /// The same, but only `w` wide and starting `dx` in from the left.
    std::pair<ImVec2, ImVec2> cell(float dx, float w, float h) {
        const ImVec2 a(x0 + dx, y);
        const ImVec2 b(x0 + dx + w, y + h);
        y += h;
        return {a, b};
    }
    void gap(float h) { y += h; }
};

class PaperUI {
public:
    // ---- frame ----------------------------------------------------------

    /// `scale` is pixels per layout unit; everything below is written in
    /// units and multiplied on the way out, so the card is the same size
    /// relative to the window on a phone and on a 4K monitor.
    void begin(float deltaSeconds, float scale);
    void end();

    [[nodiscard]] float scale() const { return scale_; }
    /// `units` converted to pixels at the current scale.
    [[nodiscard]] float px(float units) const { return units * scale_; }

    [[nodiscard]] PaperTheme& theme() { return theme_; }
    [[nodiscard]] const PaperTheme& theme() const { return theme_; }

    void setLayer(PaperLayer layer);

    /// Controls between these still draw, but hear nothing. What a modal does
    /// to the screen underneath it.
    void pushInert();
    void popInert();

    /// Whether a field has the keyboard this frame. The caller reports this
    /// to ImGui, which is what raises the on-screen keyboard on Android.
    [[nodiscard]] bool wantsTextInput() const { return textInputWanted_; }
    /// Whether the pointer is over anything of ours, so the rest of the
    /// client leaves the click alone.
    [[nodiscard]] bool wantsMouse() const { return mouseWanted_; }
    /// Ignore the rest of the press in flight.
    ///
    /// A control that appears under the pointer must not act on the click
    /// that put it there. Opening a sheet on a gear press puts a whole panel
    /// of controls under the cursor, one of which is then holding a press it
    /// never saw begin - a slider takes the value it was dropped on, a button
    /// fires on the release. Called by whatever raises the new surface; the
    /// press is ignored until the mouse comes up.
    void swallowPress() { pressSwallowed_ = true; }

    /// Claims the pointer over a region no control covers - the sheet's own
    /// margins, which are still not the world behind it.
    void claimMouse(ImVec2 a, ImVec2 b);

    // ---- focus ----------------------------------------------------------

    void focus(const char* id);
    void clearFocus();

    /// Whether a dropdown is down. Asked before the controls are drawn, so a
    /// screen can tell an Escape that closed a list from one that should
    /// close what is behind it.
    [[nodiscard]] bool popupOpen() const { return openPopup_ != 0; }

    // ---- ornament -------------------------------------------------------

    /// A sheet of paper: shadow, warm fill, a doubled hand-drawn border and,
    /// if asked, two strips of tape holding down the top corners.
    void sheet(ImVec2 a, ImVec2 b, bool taped = true);
    /// Dims everything already drawn, for a modal to sit on.
    void scrim(float alpha);

    /// A colour chosen for a dark interface, brought onto paper.
    ///
    /// WoW's class and faction colours are picked to glow against black; laid
    /// on cream they read as pastel smudges. Blending each of them part of
    /// the way to the page's own ink keeps them recognisable and legible at
    /// once - a paladin is still pink, and it is still a colour you can read.
    [[nodiscard]] ImU32 onPaper(ImVec4 colour) const;

    void text(ImVec2 at, const char* s, float size, ImU32 col, bool titleFace = false);
    void textCentered(float cx, float y, const char* s, float size, ImU32 col,
                      bool titleFace = false);
    void textRight(float rx, float y, const char* s, float size, ImU32 col,
                   bool titleFace = false);
    /// Draws `s` broken to `width` and answers the height it took.
    float wrapped(ImVec2 at, float width, const char* s, float size, ImU32 col);
    /// The height `wrapped` would take, without drawing anything. A panel
    /// sizes itself to its status message before it lays out the rows below.
    [[nodiscard]] float wrappedHeight(float width, const char* s, float size) const;
    [[nodiscard]] float textWidth(const char* s, float size, bool titleFace = false) const;
    /// How far the ink reaches below the top of a line drawn at this size -
    /// ascent to descent, which is taller than the em the size names.
    [[nodiscard]] float inkHeight(float size, bool titleFace = false) const;
    [[nodiscard]] float lineHeight(float size, bool titleFace = false) const;

    /// A line with a hand's waver in it. `seed` fixes the waver, so the same
    /// line is the same shape every frame rather than crawling. An
    /// `amplitude` of zero takes one from the thickness, which is what a
    /// thicker pen would do anyway.
    void rule(ImVec2 a, ImVec2 b, ImU32 col, float thickness, uint32_t seed,
              float amplitude = 0.0f);
    /// The loose double stroke under a heading.
    void squiggle(ImVec2 a, ImVec2 b, ImU32 col, float thickness, uint32_t seed);

    // ---- controls -------------------------------------------------------

    enum class ButtonKind {
        Primary,  ///< filled crayon; one to a card
        Quiet,    ///< outline only
    };
    bool button(const char* id, ImVec2 a, ImVec2 b, const char* label,
                ButtonKind kind = ButtonKind::Quiet, bool enabled = true);

    /// A small pill that is either chosen or not, in `accent` when it is.
    ///
    /// Ten races and ten classes are one question each, and a row of pills
    /// says so where ten little grey rectangles did not - they read as ten
    /// buttons that each do something. Rows of these also carry the faction
    /// and class colours, which a radio button has nowhere to put.
    bool chip(const char* id, ImVec2 a, ImVec2 b, const char* label, bool selected,
              ImU32 accent);
    /// How wide a chip has to be for its label. The caller wraps rows of them
    /// itself, since only it knows what the row has to fit inside.
    [[nodiscard]] float chipWidth(const char* label, float height) const;

    enum class Glyph { Gear, Cross, Eye, EyeClosed };
    /// A glyph in a circle of `radius`, drawn without a frame until hovered.
    bool glyphButton(const char* id, ImVec2 center, float radius, Glyph glyph);

    struct FieldResult {
        bool changed = false;    ///< the text differs from before the call
        bool submitted = false;  ///< Enter was pressed in it
        bool focused = false;    ///< it has the keyboard
    };
    struct FieldOpts {
        bool password = false;        ///< draw dots, never the characters
        const char* placeholder = nullptr;
        bool digitsOnly = false;
        /// Taking focus selects everything, so typing replaces rather than
        /// appends. What a remembered value wants and a fresh one does not.
        bool selectAllOnFocus = false;
    };
    FieldResult field(const char* id, ImVec2 a, ImVec2 b, TextEdit& edit,
                      const FieldOpts& opts);
    FieldResult field(const char* id, ImVec2 a, ImVec2 b, TextEdit& edit) {
        return field(id, a, b, edit, FieldOpts{});
    }

    /// Answers true on the frame the selection changes. `index` may be -1,
    /// which shows `preview` and matches no row.
    bool dropdown(const char* id, ImVec2 a, ImVec2 b, const std::string& preview,
                  const std::vector<std::string>& items, int* index);

    bool checkbox(const char* id, ImVec2 at, float boxSize, const char* label,
                  bool* value);
    bool sliderFloat(const char* id, ImVec2 a, ImVec2 b, float* value, float lo,
                     float hi, const char* fmt = "%.0f");
    bool sliderInt(const char* id, ImVec2 a, ImVec2 b, int* value, int lo, int hi);

    /// Underlined text that behaves as a button. `rightOf` is where it ends,
    /// so a caller can put the pointer test over the words and nothing else.
    bool link(const char* id, ImVec2 at, const char* label, float size,
              ImU32 col = 0);

    /// What a row of a list did this frame. -1 for "not this one".
    struct ListResult {
        int hovered = -1;
        int clicked = -1;    ///< picked with a single click
        int activated = -1;  ///< opened with a double click
    };
    /// Draws the inside of one row, given its rect and what state it is in.
    using RowDrawer = std::function<void(int index, ImVec2 a, ImVec2 b, bool selected,
                                         bool hovered)>;
    /// A ruled list of rows.
    ///
    /// The caller draws the inside of each row, because the columns differ
    /// from one screen to the next and the kit has no business knowing what a
    /// realm or a character is. What this owns is the paper under them: the
    /// ruling between rows, the highlight on the one being pointed at, the
    /// scroll and its mark in the margin, and which row was picked.
    ///
    /// Passing the selected index in rather than keeping it means a list
    /// whose selection moved by some other route - the arrow keys, a
    /// character having just been created - scrolls to show it.
    ListResult list(const char* id, ImVec2 a, ImVec2 b, int rowCount, float rowHeight,
                    int selected, const RowDrawer& drawRow);

    /// A picture from somewhere else laid on the page: a white mat, a soft
    /// shadow, and a printed edge. The client's character preview is a
    /// texture the renderer produced, and this only places it.
    void image(ImTextureID texture, ImVec2 a, ImVec2 b);

    /// Whether the pointer is over this rect, claiming it if it is. For the
    /// few things the kit places but does not draw.
    bool hover(ImVec2 a, ImVec2 b);

private:
    struct FieldState {
        float scroll = 0.0f;   ///< how far a field's text has slid left in its box
        float scrollY = 0.0f;  ///< how far a list has been scrolled down
        int lastSelected = -1; ///< so a list can tell when its selection moved
    };

    [[nodiscard]] static ImU32 hashId(const char* s);
    [[nodiscard]] ImFont* face(bool titleFace) const;
    [[nodiscard]] bool listening() const { return inert_ == 0 && openPopup_ == 0; }
    /// The same, for the one control a live popup belongs to.
    [[nodiscard]] bool listeningAs(ImU32 id) const {
        return inert_ == 0 && (openPopup_ == 0 || openPopup_ == id);
    }
    bool hovered(ImVec2 a, ImVec2 b);
    /// A fresh left-button press that this frame's controls may act on.
    [[nodiscard]] bool pressed() const;
    void resetBlink() { blink_ = 0.0f; }

    void drawSheetBody(ImDrawList* dl, ImVec2 a, ImVec2 b, bool taped);
    void wobblyRect(ImDrawList* dl, ImVec2 a, ImVec2 b, ImU32 col, float thickness,
                    float amplitude, uint32_t seed);
    void drawGlyph(ImDrawList* dl, ImVec2 center, float radius, Glyph glyph, ImU32 col);
    void drawFieldText(ImDrawList* dl, ImVec2 inner0, ImVec2 inner1, const TextEdit& edit,
                       const FieldOpts& opts, FieldState& state, bool focused, float size);

    // Measuring a field's text is done by the same two functions whichever way
    // it is drawn: masked text advances by a fixed step per character, plain
    // text by whatever the face says.
    [[nodiscard]] float advanceTo(const TextEdit& edit, size_t byte, bool password,
                                  float size) const;
    [[nodiscard]] size_t byteAtOffset(const TextEdit& edit, float dx, bool password,
                                      float size) const;

    PaperTheme theme_;
    ImDrawList* page_ = nullptr;
    ImDrawList* overlay_ = nullptr;
    ImDrawList* dl_ = nullptr;
    ImFont* titleFace_ = nullptr;
    ImFont* bodyFace_ = nullptr;

    float scale_ = 1.0f;
    float blink_ = 0.0f;
    int inert_ = 0;
    bool textInputWanted_ = false;
    /// The press that opened something is not a press on what it opened.
    bool pressSwallowed_ = false;
    bool mouseWanted_ = false;

    ImU32 focus_ = 0;
    ImU32 active_ = 0;      ///< holding the mouse down on it
    ImU32 dragging_ = 0;    ///< sweeping a selection through it
    ImU32 openPopup_ = 0;
    ImU32 justFocused_ = 0; ///< took focus this frame; used by selectAllOnFocus

    // Tab order, rebuilt every frame from the order fields are drawn in.
    ImU32 firstField_ = 0;
    ImU32 lastField_ = 0;
    bool takeFocusNext_ = false;
    bool focusLastAtEnd_ = false;

    struct PopupDraw {
        ImVec2 a, b;
        std::vector<std::string> items;
        int selected = -1;
        int hovered = -1;
        float rowHeight = 0.0f;
    };
    std::vector<PopupDraw> popups_;

    std::vector<std::pair<ImU32, FieldState>> fieldStates_;
    FieldState& fieldState(ImU32 id);
};

} // namespace wowee::ui
