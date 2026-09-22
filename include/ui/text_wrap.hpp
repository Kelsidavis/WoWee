// text_wrap.hpp - Breaking a run of text into lines that fit a width.
//
// Pulled out of the renderer so it can be tested without a font. The widths
// come from a callable rather than from ImFont, which is the only thing the
// drawing side has that a test does not.
//
// FontStrings never wrapped at all before this: every draw went through one
// AddText at an unbounded width, so a label given a size by its XML or by two
// anchors drew a single line straight out of its own frame.
#pragma once

#include <algorithm>
#include <string>
#include <vector>

namespace wowee::ui {

/// A piece of text and whether it carries a colour of its own. The renderer's
/// markup parser produces these; the wrap works on them rather than on the
/// stripped string so a colour run split across two lines keeps its colour on
/// both.
struct WrapRun {
    std::string text;
    bool  hasColor = false;
    float rgba[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    /// The payload of the |H...|h this run sits inside - "item:3299" and the
    /// like - or empty when the run is not part of a link. Carried so a click
    /// has something to name; the display text alone cannot say what was
    /// clicked.
    std::string link;

    /// An inline texture: the file named by a |Tpath:h:w:ox:oy|t escape.
    ///
    /// When this is set the run draws the picture and `text` is empty. Coin
    /// amounts are written this way - GOLD_AMOUNT_TEXTURE is "%d" followed by
    /// one of these - so a run of them is what a price is made of.
    std::string texture;
    /// Height and width from the escape, in interface units. Zero means "the
    /// height of the line", which is what every coin escape asks for.
    float texHeight = 0.0f;
    float texWidth = 0.0f;
};

/// Whether two runs are the same style, so adjacent pieces can be merged back
/// into one rather than left as a string of one-word runs.
inline bool sameStyle(const WrapRun& a, const WrapRun& b) {
    // Two links are not one run even in the same colour: merging them would
    // leave a single rect covering both, and a click could not say which.
    if (a.link != b.link) return false;
    // A picture is never merged into a neighbouring run: it draws instead of
    // text rather than beside it, so a merge would lose one or the other.
    if (!a.texture.empty() || !b.texture.empty()) return false;
    if (a.hasColor != b.hasColor) return false;
    if (!a.hasColor) return true;
    return std::equal(std::begin(a.rgba), std::end(a.rgba), std::begin(b.rgba));
}

/// Break runs into lines no wider than wrapWidth.
///
/// `measure` answers the width of a string. A wrapWidth of zero means no soft
/// wrapping, which is what an auto-sized label wants - it is as wide as its own
/// text. A newline still breaks the line at any width: |n is WoW's spelling of
/// one, and a label with an explicit break gets one whether or not anything is
/// wrapping.
///
/// Words, not characters, unless a single word is wider than the whole box and
/// nonSpaceWrap says it may be broken. Thirty-six labels in FrameXML ask for
/// that, all of them prose in a narrow column.
template <typename Measure>
std::vector<std::vector<WrapRun>> wrapText(const std::vector<WrapRun>& runs,
                                           float wrapWidth, bool nonSpaceWrap,
                                           Measure measure) {
    std::vector<std::vector<WrapRun>> lines;
    lines.emplace_back();
    float x = 0.0f;
    auto place = [&](const WrapRun& style, const std::string& piece) {
        if (!lines.back().empty() && sameStyle(lines.back().back(), style)) {
            lines.back().back().text += piece;
        } else {
            WrapRun add = style;
            add.text = piece;
            lines.back().push_back(std::move(add));
        }
    };

    for (const WrapRun& run : runs) {
        // A picture is one indivisible piece. It carries no text, so the loop
        // below would skip it entirely and the coin beside a price would be
        // dropped between the parser and the screen.
        if (!run.texture.empty()) {
            // Zero means "as tall as the line", and the line's height is the
            // font's - which the measure callable is the only way to reach
            // from here.
            const float w = run.texWidth > 0.0f    ? run.texWidth
                          : run.texHeight > 0.0f   ? run.texHeight
                                                   : measure("W");
            if (wrapWidth > 0.0f && x > 0.0f && x + w > wrapWidth) {
                lines.emplace_back();
                x = 0.0f;
            }
            lines.back().push_back(run);
            x += w;
            continue;
        }
        size_t at = 0;
        while (at < run.text.size()) {
            // A newline breaks the line whatever the width is. |n is WoW's
            // spelling of one and the markup parser turns it into this, so a
            // label with an explicit break gets one even when nothing is
            // wrapping.
            if (run.text[at] == '\n') {
                lines.emplace_back();
                x = 0.0f;
                ++at;
                continue;
            }
            if (wrapWidth <= 0.0f) {
                // No soft wrapping: take everything up to the next hard break.
                const size_t stop = run.text.find('\n', at);
                place(run, run.text.substr(at, stop == std::string::npos
                                                   ? std::string::npos : stop - at));
                at = (stop == std::string::npos) ? run.text.size() : stop;
                continue;
            }
            // A word and the spaces that follow it, so a break falls between
            // words and the trailing space stays with the line above.
            size_t end = run.text.find_first_of(" \n", at);
            if (end == std::string::npos) {
                end = run.text.size();
            } else if (run.text[end] == '\n') {
                // Stop before it; the branch above takes the break itself.
            } else {
                while (end < run.text.size() && run.text[end] == ' ') ++end;
            }
            std::string word = run.text.substr(at, end - at);
            at = end;

            float w = measure(word);
            if (x > 0.0f && x + w > wrapWidth) {
                lines.emplace_back();
                x = 0.0f;
                // The break stands in for the space that separated them.
                while (!word.empty() && word.front() == ' ') word.erase(0, 1);
                w = measure(word);
            }

            if (w > wrapWidth && nonSpaceWrap) {
                // One word wider than the whole box. Broken by character,
                // which is the only thing left and what the attribute asks for.
                std::string part;
                for (char c : word) {
                    if (!part.empty() && x + measure(part + c) > wrapWidth) {
                        place(run, part);
                        lines.emplace_back();
                        x = 0.0f;
                        part.clear();
                    }
                    part += c;
                }
                if (!part.empty()) {
                    place(run, part);
                    x += measure(part);
                }
                continue;
            }

            place(run, word);
            x += w;
        }
    }
    return lines;
}

/// One line of plain text cut to fit a width, ending in "..." where it had to
/// be cut - what WoW does with a label too long for the width it was given.
///
/// Text that fits comes back as it is. Otherwise the longest start of it that
/// fits with the dots after it, cut between characters rather than inside one
/// (UTF-8), and without a space left hanging before the dots. When not even
/// the dots fit, the dots alone, and the clip takes what it must.
template <typename Measure>
std::string fitWithEllipsis(const std::string& text, float width, Measure measure) {
    if (measure(text) <= width) return text;
    static const std::string kDots = "...";
    std::size_t cut = text.size();
    while (cut > 0) {
        do {
            --cut;
        } while (cut > 0 && (static_cast<unsigned char>(text[cut]) & 0xC0) == 0x80);
        std::string head = text.substr(0, cut);
        while (!head.empty() && head.back() == ' ') head.pop_back();
        if (head.empty()) break;
        if (measure(head + kDots) <= width) return head + kDots;
    }
    return kDots;
}

}  // namespace wowee::ui
