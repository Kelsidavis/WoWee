#pragma once

// The subset of HTML a SimpleHTML frame understands.
//
// Item text arrives as either plain text or a small HTML document, and the
// page decides which by looking at how it starts. The plain kind is most
// letters and books; the HTML kind is anything with a picture or a heading -
// "<HTML><BODY><IMG src="Interface\Pictures\24475_gordawg_256"/></BODY></HTML>"
// is the whole of Gordawg's Imprint, and drawn as it stands the page showed
// the markup instead of the picture.
//
// What the client accepts, and so what this does: HTML and BODY as wrappers,
// H1-H3 and P as blocks with an optional align, BR as a line break, IMG with
// src, width, height and align, and A with href as a link. A heading draws in
// the frame's own font unless the frame was given one for it, and nothing in
// FrameXML gives one - so a heading is a block like a paragraph. Anything else is
// dropped as a tag and its contents kept. Whitespace collapses as it does in
// any HTML - a line break in the source is a space, and BR is how a page asks
// for a new line.
//
// Header-only and free of ImGui, like text_markup.hpp, so it can be tested.

#include <cctype>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

namespace wowee {
namespace ui {

struct HtmlBlock {
    enum class Kind { Text, Image };
    Kind kind = Kind::Text;
    /// "LEFT", "CENTER", "RIGHT", or empty for the frame's own justification.
    std::string align;
    /// Text blocks: the words, with WoW's own |-escapes intact and each BR
    /// already a '\n', so the ordinary markup draw can take it as it is. A
    /// link is rewritten to |H...|h form for the same reason.
    std::string text;
    /// Image blocks: the file, and the size the tag asked for - zero where it
    /// asked nothing, which means the picture's own.
    std::string src;
    float width = 0.0f;
    float height = 0.0f;
};

namespace detail {

inline std::string htmlUpper(std::string s) {
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

/// One attribute's value out of a tag's body, case-insensitive on the name.
/// Quoted with either quote, or bare up to the next space.
inline std::string htmlAttr(const std::string& tagBody, const char* name) {
    const std::string up = htmlUpper(tagBody);
    const std::string key = htmlUpper(name);
    size_t at = 0;
    while ((at = up.find(key, at)) != std::string::npos) {
        // A whole word: "src" must not match the tail of "imgsrc".
        const bool startOk = at == 0 ||
            std::isspace(static_cast<unsigned char>(up[at - 1]));
        size_t p = at + key.size();
        while (p < up.size() && std::isspace(static_cast<unsigned char>(up[p]))) ++p;
        if (!startOk || p >= up.size() || up[p] != '=') { at += key.size(); continue; }
        ++p;
        while (p < up.size() && std::isspace(static_cast<unsigned char>(up[p]))) ++p;
        if (p >= up.size()) return {};
        if (tagBody[p] == '"' || tagBody[p] == '\'') {
            const char q = tagBody[p];
            const size_t end = tagBody.find(q, p + 1);
            return tagBody.substr(p + 1, end == std::string::npos
                                             ? std::string::npos : end - p - 1);
        }
        size_t end = p;
        while (end < tagBody.size() &&
               !std::isspace(static_cast<unsigned char>(tagBody[end])) &&
               tagBody[end] != '/' && tagBody[end] != '>')
            ++end;
        return tagBody.substr(p, end - p);
    }
    return {};
}

inline std::string htmlAlign(const std::string& tagBody) {
    const std::string a = htmlUpper(htmlAttr(tagBody, "align"));
    return (a == "LEFT" || a == "CENTER" || a == "RIGHT") ? a : std::string();
}

}  // namespace detail

/// Whether a string is to be read as HTML rather than drawn as it stands.
///
/// It is when it opens with <HTML>, ignoring case and any leading whitespace.
/// The whitespace matters: ItemTextFrame sets "\n"..ItemTextGetText(), so every
/// page arrives with a line break in front of it.
inline bool looksLikeSimpleHtml(const std::string& text) {
    size_t i = 0;
    while (i < text.size() && std::isspace(static_cast<unsigned char>(text[i]))) ++i;
    if (text.size() - i < 5 || text[i] != '<') return false;
    return detail::htmlUpper(text.substr(i + 1, 4)) == "HTML";
}

/// The blocks of a SimpleHTML document, in reading order.
inline std::vector<HtmlBlock> parseSimpleHtml(const std::string& in) {
    std::vector<HtmlBlock> blocks;
    HtmlBlock cur;
    // Whether the last thing written into cur.text was a space, so runs of
    // whitespace collapse to one and none opens a line.
    bool lastSpace = true;
    std::string linkHref;

    auto flush = [&] {
        // A trailing space is where the source wrapped, not something to draw.
        while (!cur.text.empty() && cur.text.back() == ' ') cur.text.pop_back();
        if (!cur.text.empty()) blocks.push_back(cur);
        cur.text.clear();
        lastSpace = true;
    };
    auto startBlock = [&](std::string align) {
        flush();
        cur.kind = HtmlBlock::Kind::Text;
        cur.align = std::move(align);
    };
    auto put = [&](char c) {
        if (std::isspace(static_cast<unsigned char>(c))) {
            if (lastSpace) return;
            cur.text += ' ';
            lastSpace = true;
            return;
        }
        cur.text += c;
        lastSpace = false;
    };

    for (size_t i = 0; i < in.size();) {
        const char c = in[i];
        if (c == '&') {
            // The handful of entities a page can carry. Anything else is a
            // literal ampersand, which is what an unescaped one meant.
            static const struct { const char* name; char ch; } kEntities[] = {
                {.name = "&lt;", .ch = '<'},     {.name = "&gt;", .ch = '>'},
                {.name = "&amp;", .ch = '&'},    {.name = "&quot;", .ch = '"'},
                {.name = "&apos;", .ch = '\''}, {.name = "&nbsp;", .ch = ' '},
            };
            bool matched = false;
            for (const auto& e : kEntities) {
                const size_t n = std::char_traits<char>::length(e.name);
                if (in.compare(i, n, e.name) == 0) {
                    // A non-breaking space is still a space, just not one
                    // that collapses into its neighbour.
                    if (e.name[1] == 'n') { cur.text += ' '; lastSpace = false; }
                    else put(e.ch);
                    i += n;
                    matched = true;
                    break;
                }
            }
            if (!matched) { put('&'); ++i; }
            continue;
        }
        if (c != '<') { put(c); ++i; continue; }

        const size_t close = in.find('>', i + 1);
        if (close == std::string::npos) { put(c); ++i; continue; }
        std::string body = in.substr(i + 1, close - i - 1);
        i = close + 1;
        const bool closing = !body.empty() && body[0] == '/';
        if (closing) body.erase(0, 1);
        size_t nameEnd = 0;
        while (nameEnd < body.size() &&
               std::isalnum(static_cast<unsigned char>(body[nameEnd])))
            ++nameEnd;
        const std::string name = detail::htmlUpper(body.substr(0, nameEnd));

        if (name == "P" || name == "H1" || name == "H2" || name == "H3") {
            startBlock(closing ? std::string() : detail::htmlAlign(body));
        } else if (name == "BR") {
            // Kept even at the start of a block: a page opening with BRs is
            // asking for the space above its first line.
            while (!cur.text.empty() && cur.text.back() == ' ') cur.text.pop_back();
            cur.text += '\n';
            lastSpace = true;
        } else if (name == "IMG" && !closing) {
            // The text either side stays in its block and keeps its align;
            // only the picture stands on a line of its own.
            flush();
            HtmlBlock img;
            img.kind = HtmlBlock::Kind::Image;
            img.src = detail::htmlAttr(body, "src");
            img.align = detail::htmlAlign(body);
            img.width = static_cast<float>(
                std::atof(detail::htmlAttr(body, "width").c_str()));
            img.height = static_cast<float>(
                std::atof(detail::htmlAttr(body, "height").c_str()));
            if (!img.src.empty()) blocks.push_back(std::move(img));
        } else if (name == "A") {
            // Written the way a label writes a link, so the draw that already
            // files a clickable rect for |H...|h does the same here.
            if (!closing) {
                linkHref = detail::htmlAttr(body, "href");
                if (!linkHref.empty()) cur.text += "|H" + linkHref + "|h";
            } else if (!linkHref.empty()) {
                cur.text += "|h";
                linkHref.clear();
            }
        } else if ((name == "BODY" || name == "HTML") && closing) {
            startBlock({});
        }
        // Anything else is not a tag this frame knows, and its text stands.
    }
    flush();
    return blocks;
}

}  // namespace ui
}  // namespace wowee
