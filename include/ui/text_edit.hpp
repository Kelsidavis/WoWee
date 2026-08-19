#pragma once

// The editing half of a text field, with none of the drawing in it.
//
// The login screen draws its own boxes rather than asking ImGui for them,
// which means this client now owns the parts a text field is actually made
// of: where the caret sits, what is selected, and what Ctrl+Left does when
// the caret is already at the start of a word. None of those questions needs
// a font, a device or a frame to answer, so none of them are kept where those
// things are. What is here is a string, two offsets into it, and the
// operations a keyboard performs on the pair - which is small enough for a
// test to hold in one hand.
//
// Offsets are byte offsets and the text is UTF-8. Every motion lands on a
// codepoint boundary: half of a multi-byte character is not a place a caret
// can be, and inserting there produces a string nothing can read back.

#include <cstddef>
#include <string>
#include <string_view>

namespace wowee::ui {

class TextEdit {
public:
    TextEdit() = default;
    explicit TextEdit(size_t maxBytes) : maxBytes_(maxBytes) {}

    // ---- contents -------------------------------------------------------

    [[nodiscard]] const std::string& text() const { return text_; }
    [[nodiscard]] const char* c_str() const { return text_.c_str(); }
    [[nodiscard]] bool empty() const { return text_.empty(); }
    [[nodiscard]] size_t size() const { return text_.size(); }

    /// Replaces the contents outright and puts the caret at the end, the way
    /// assigning to a field should rather than the way typing into one does.
    void setText(std::string_view s);
    void clear() { setText({}); }

    /// How many codepoints the string holds. The masked draw counts these
    /// rather than bytes - one dot per character the player typed, not one
    /// per byte it took to store it.
    [[nodiscard]] size_t codepointCount() const;

    // ---- caret and selection --------------------------------------------

    [[nodiscard]] size_t caret() const { return caret_; }
    [[nodiscard]] size_t anchor() const { return anchor_; }
    [[nodiscard]] bool hasSelection() const { return caret_ != anchor_; }
    [[nodiscard]] size_t selectionBegin() const { return caret_ < anchor_ ? caret_ : anchor_; }
    [[nodiscard]] size_t selectionEnd() const { return caret_ < anchor_ ? anchor_ : caret_; }
    [[nodiscard]] std::string selectedText() const;

    /// Puts the caret at `byte`, snapped to the boundary at or before it.
    /// `extend` leaves the anchor where it is, which is what a shift-click or
    /// a shifted arrow key wants; without it the selection collapses.
    void setCaret(size_t byte, bool extend = false);
    void selectAll();
    void deselect() { anchor_ = caret_; }

    /// Selects the run of like characters around `byte` - what a double click
    /// picks out. Runs of spaces select as a run, as they do everywhere else.
    void selectWordAt(size_t byte);

    // ---- editing. Each answers whether the text changed. -----------------

    /// Inserts `utf8` over the selection. Control characters are dropped - a
    /// paste carrying a newline is a paste of the text around it, not a field
    /// with a newline wedged into it - and the insert stops at maxBytes
    /// without ever splitting a codepoint to get there.
    bool insert(std::string_view utf8);
    bool insertCodepoint(unsigned int cp);
    bool backspace(bool wholeWord = false);
    bool deleteForward(bool wholeWord = false);
    bool deleteSelection();

    // ---- motion ----------------------------------------------------------

    void moveLeft(bool extend, bool wholeWord);
    void moveRight(bool extend, bool wholeWord);
    void moveHome(bool extend);
    void moveEnd(bool extend);

    // ---- boundaries ------------------------------------------------------
    //
    // Public because the drawing side needs them too: turning a mouse x into
    // a caret position means walking the same boundaries this walks.

    [[nodiscard]] size_t prevBoundary(size_t byte) const;
    [[nodiscard]] size_t nextBoundary(size_t byte) const;
    [[nodiscard]] size_t wordLeft(size_t byte) const;
    [[nodiscard]] size_t wordRight(size_t byte) const;
    /// Clamps to the string and snaps to the boundary at or before `byte`.
    [[nodiscard]] size_t clampToBoundary(size_t byte) const;

    /// Appends `cp` to `out` as UTF-8. Codepoints outside Unicode, and the
    /// surrogate range, are dropped rather than encoded.
    static void appendUtf8(std::string& out, unsigned int cp);

private:
    std::string text_;
    size_t caret_ = 0;
    size_t anchor_ = 0;
    /// One byte short of the 256-byte fields the login screen keeps, so a
    /// full field still has room for its terminator when it is copied out.
    size_t maxBytes_ = 255;
};

} // namespace wowee::ui
