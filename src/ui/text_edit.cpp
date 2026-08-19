#include "ui/text_edit.hpp"

#include <algorithm>

namespace wowee::ui {

namespace {

/// What a character counts as when a word motion is deciding where to stop.
///
/// Three classes rather than two, because the caret has to be able to get
/// past punctuation. With only word-or-not, Ctrl+Left in "wow.example.com"
/// walks one dot at a time and Ctrl+Left in "foo, bar" stops dead on the
/// comma: the run of separators is not a word, so nothing skips it.
enum class CharClass { Space, Word, Punct };

CharClass classify(unsigned char c) {
    if (c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\v' || c == '\f')
        return CharClass::Space;
    // Anything outside ASCII is part of a word. Deciding otherwise means
    // knowing the script, which this does not and does not need to.
    if (c >= 0x80) return CharClass::Word;
    if (c >= '0' && c <= '9') return CharClass::Word;
    if (c >= 'A' && c <= 'Z') return CharClass::Word;
    if (c >= 'a' && c <= 'z') return CharClass::Word;
    if (c == '_') return CharClass::Word;
    return CharClass::Punct;
}

bool isContinuation(unsigned char c) { return (c & 0xC0) == 0x80; }

/// How many bytes the character starting with `c` occupies.
size_t codepointLength(unsigned char c) {
    if ((c & 0x80) == 0x00) return 1;
    if ((c & 0xE0) == 0xC0) return 2;
    if ((c & 0xF0) == 0xE0) return 3;
    if ((c & 0xF8) == 0xF0) return 4;
    return 1;  // a stray continuation byte, or a lead byte no encoding defines
}

/// Drops a trailing character that a byte-count cut left half-written.
///
/// Popping continuation bytes alone is not enough: what is left behind is the
/// lead byte that promised them, which is itself not a character. The lead
/// byte says how many bytes it needs, so ask it.
void truncateToWholeCodepoints(std::string& s) {
    if (s.empty()) return;
    size_t lead = s.size() - 1;
    while (lead > 0 && isContinuation(static_cast<unsigned char>(s[lead]))) --lead;
    const unsigned char c = static_cast<unsigned char>(s[lead]);
    if (isContinuation(c) || lead + codepointLength(c) > s.size()) s.resize(lead);
}

/// A control character, and so not something a single-line field can hold.
/// Bytes at or above 0x80 are never control characters in UTF-8 - they are
/// the halves of some codepoint - so this only ever inspects ASCII.
bool isControl(unsigned char c) { return c < 0x20 || c == 0x7F; }

} // namespace

// ---------------------------------------------------------------------------
// Boundaries
// ---------------------------------------------------------------------------

size_t TextEdit::prevBoundary(size_t byte) const {
    size_t p = std::min(byte, text_.size());
    if (p == 0) return 0;
    --p;
    while (p > 0 && isContinuation(static_cast<unsigned char>(text_[p]))) --p;
    return p;
}

size_t TextEdit::nextBoundary(size_t byte) const {
    size_t p = std::min(byte, text_.size());
    if (p >= text_.size()) return text_.size();
    ++p;
    while (p < text_.size() && isContinuation(static_cast<unsigned char>(text_[p]))) ++p;
    return p;
}

size_t TextEdit::clampToBoundary(size_t byte) const {
    size_t p = std::min(byte, text_.size());
    while (p > 0 && p < text_.size() && isContinuation(static_cast<unsigned char>(text_[p])))
        --p;
    return p;
}

size_t TextEdit::wordLeft(size_t byte) const {
    size_t p = clampToBoundary(byte);
    while (p > 0 && classify(static_cast<unsigned char>(text_[prevBoundary(p)])) == CharClass::Space)
        p = prevBoundary(p);
    if (p == 0) return 0;
    const CharClass run = classify(static_cast<unsigned char>(text_[prevBoundary(p)]));
    while (p > 0 && classify(static_cast<unsigned char>(text_[prevBoundary(p)])) == run)
        p = prevBoundary(p);
    return p;
}

size_t TextEdit::wordRight(size_t byte) const {
    size_t p = clampToBoundary(byte);
    if (p >= text_.size()) return text_.size();
    const CharClass run = classify(static_cast<unsigned char>(text_[p]));
    if (run != CharClass::Space) {
        while (p < text_.size() && classify(static_cast<unsigned char>(text_[p])) == run)
            p = nextBoundary(p);
    }
    // Whatever the caret started on, it ends at the start of the next thing
    // rather than in the gap before it.
    while (p < text_.size() && classify(static_cast<unsigned char>(text_[p])) == CharClass::Space)
        p = nextBoundary(p);
    return p;
}

size_t TextEdit::codepointCount() const {
    size_t n = 0;
    for (char c : text_)
        if (!isContinuation(static_cast<unsigned char>(c))) ++n;
    return n;
}

// ---------------------------------------------------------------------------
// Contents
// ---------------------------------------------------------------------------

void TextEdit::setText(std::string_view s) {
    text_.assign(s);
    if (text_.size() > maxBytes_) {
        // Truncating by byte count can land mid-character; back off to the
        // boundary rather than storing half of one.
        text_.resize(maxBytes_);
        truncateToWholeCodepoints(text_);
    }
    caret_ = anchor_ = text_.size();
}

std::string TextEdit::selectedText() const {
    if (!hasSelection()) return {};
    return text_.substr(selectionBegin(), selectionEnd() - selectionBegin());
}

// ---------------------------------------------------------------------------
// Caret and selection
// ---------------------------------------------------------------------------

void TextEdit::setCaret(size_t byte, bool extend) {
    caret_ = clampToBoundary(byte);
    if (!extend) anchor_ = caret_;
}

void TextEdit::selectAll() {
    anchor_ = 0;
    caret_ = text_.size();
}

void TextEdit::selectWordAt(size_t byte) {
    if (text_.empty()) {
        caret_ = anchor_ = 0;
        return;
    }
    size_t p = clampToBoundary(byte);
    // A click past the end belongs to the last character, not to nothing.
    if (p >= text_.size()) p = prevBoundary(text_.size());

    const CharClass run = classify(static_cast<unsigned char>(text_[p]));
    size_t begin = p;
    while (begin > 0 &&
           classify(static_cast<unsigned char>(text_[prevBoundary(begin)])) == run)
        begin = prevBoundary(begin);
    size_t end = p;
    while (end < text_.size() &&
           classify(static_cast<unsigned char>(text_[end])) == run)
        end = nextBoundary(end);

    anchor_ = begin;
    caret_ = end;
}

// ---------------------------------------------------------------------------
// Editing
// ---------------------------------------------------------------------------

void TextEdit::appendUtf8(std::string& out, unsigned int cp) {
    if (cp > 0x10FFFF) return;
    if (cp >= 0xD800 && cp <= 0xDFFF) return;  // lone surrogate; not a character
    if (cp < 0x80) {
        out.push_back(static_cast<char>(cp));
    } else if (cp < 0x800) {
        out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
        out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    } else if (cp < 0x10000) {
        out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
        out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
        out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    } else {
        out.push_back(static_cast<char>(0xF0 | (cp >> 18)));
        out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
        out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
        out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    }
}

bool TextEdit::deleteSelection() {
    if (!hasSelection()) return false;
    const size_t begin = selectionBegin();
    text_.erase(begin, selectionEnd() - begin);
    caret_ = anchor_ = begin;
    return true;
}

bool TextEdit::insert(std::string_view utf8) {
    // Strip the characters a single-line field cannot hold before measuring
    // anything, so a paste of "name\n" is not refused for being one byte over.
    std::string clean;
    clean.reserve(utf8.size());
    for (char c : utf8)
        if (!isControl(static_cast<unsigned char>(c))) clean.push_back(c);

    const bool removed = deleteSelection();
    if (clean.empty()) return removed;

    const size_t room = maxBytes_ > text_.size() ? maxBytes_ - text_.size() : 0;
    if (room == 0) return removed;
    if (clean.size() > room) {
        // Never let the cap be what splits a character in half.
        clean.resize(room);
        truncateToWholeCodepoints(clean);
        if (clean.empty()) return removed;
    }

    text_.insert(caret_, clean);
    caret_ += clean.size();
    anchor_ = caret_;
    return true;
}

bool TextEdit::insertCodepoint(unsigned int cp) {
    if (cp < 0x20 || cp == 0x7F) return false;
    std::string encoded;
    appendUtf8(encoded, cp);
    if (encoded.empty()) return false;
    return insert(encoded);
}

bool TextEdit::backspace(bool wholeWord) {
    if (deleteSelection()) return true;
    if (caret_ == 0) return false;
    const size_t from = wholeWord ? wordLeft(caret_) : prevBoundary(caret_);
    text_.erase(from, caret_ - from);
    caret_ = anchor_ = from;
    return true;
}

bool TextEdit::deleteForward(bool wholeWord) {
    if (deleteSelection()) return true;
    if (caret_ >= text_.size()) return false;
    const size_t to = wholeWord ? wordRight(caret_) : nextBoundary(caret_);
    text_.erase(caret_, to - caret_);
    anchor_ = caret_;
    return true;
}

// ---------------------------------------------------------------------------
// Motion
// ---------------------------------------------------------------------------

void TextEdit::moveLeft(bool extend, bool wholeWord) {
    // An unshifted arrow over a selection collapses it to the near edge
    // rather than moving off the far one.
    if (!extend && hasSelection() && !wholeWord) {
        caret_ = anchor_ = selectionBegin();
        return;
    }
    setCaret(wholeWord ? wordLeft(caret_) : prevBoundary(caret_), extend);
}

void TextEdit::moveRight(bool extend, bool wholeWord) {
    if (!extend && hasSelection() && !wholeWord) {
        caret_ = anchor_ = selectionEnd();
        return;
    }
    setCaret(wholeWord ? wordRight(caret_) : nextBoundary(caret_), extend);
}

void TextEdit::moveHome(bool extend) { setCaret(0, extend); }

void TextEdit::moveEnd(bool extend) { setCaret(text_.size(), extend); }

} // namespace wowee::ui
