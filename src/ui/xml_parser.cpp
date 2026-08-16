#include "ui/xml_parser.hpp"

#include <cctype>
#include <cstring>

namespace wowee {
namespace ui {

namespace {

struct Cursor {
    const std::string& s;
    size_t i = 0;
    std::string error;

    [[nodiscard]] bool eof() const { return i >= s.size(); }
    [[nodiscard]] char peek(size_t ahead = 0) const {
        return (i + ahead < s.size()) ? s[i + ahead] : '\0';
    }
    bool starts(const char* lit) const { return s.compare(i, strlen(lit), lit) == 0; }
    void skipSpace() { while (!eof() && std::isspace(static_cast<unsigned char>(s[i]))) ++i; }
};

/// XML's five predefined entities, plus numeric references. FrameXML uses the
/// named ones in tooltip and label text constantly.
std::string decodeEntities(const std::string& in) {
    std::string out;
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i) {
        if (in[i] != '&') { out += in[i]; continue; }
        const size_t semi = in.find(';', i);
        if (semi == std::string::npos || semi - i > 10) { out += in[i]; continue; }
        const std::string ent = in.substr(i + 1, semi - i - 1);
        if      (ent == "lt")   out += '<';
        else if (ent == "gt")   out += '>';
        else if (ent == "amp")  out += '&';
        else if (ent == "quot") out += '"';
        else if (ent == "apos") out += '\'';
        else if (!ent.empty() && ent[0] == '#') {
            try {
                const int code = (ent.size() > 1 && (ent[1] == 'x' || ent[1] == 'X'))
                    ? std::stoi(ent.substr(2), nullptr, 16)
                    : std::stoi(ent.substr(1));
                // Only Latin-1 is emitted directly; anything wider is left as
                // written rather than producing a broken byte.
                if (code > 0 && code < 128) out += static_cast<char>(code);
                else { out += '&'; out += ent; out += ';'; }
            } catch (...) { out += '&'; out += ent; out += ';'; }
        }
        else { out += '&'; out += ent; out += ';'; continue; }
        i = semi;
    }
    return out;
}

/// Skip comments, the XML declaration and DOCTYPE. Returns true if anything was
/// consumed, so the caller can loop until the input settles.
bool skipIgnorable(Cursor& c) {
    c.skipSpace();
    if (c.starts("<!--")) {
        const size_t end = c.s.find("-->", c.i);
        c.i = (end == std::string::npos) ? c.s.size() : end + 3;
        return true;
    }
    if (c.starts("<?")) {
        const size_t end = c.s.find("?>", c.i);
        c.i = (end == std::string::npos) ? c.s.size() : end + 2;
        return true;
    }
    if (c.starts("<!DOCTYPE")) {
        const size_t end = c.s.find('>', c.i);
        c.i = (end == std::string::npos) ? c.s.size() : end + 1;
        return true;
    }
    return false;
}

bool nameChar(char ch) {
    return std::isalnum(static_cast<unsigned char>(ch)) ||
           ch == '_' || ch == '-' || ch == ':' || ch == '.';
}

bool parseElement(Cursor& c, XmlNode& out);

/// Read children and text until the matching close tag.
bool parseContent(Cursor& c, XmlNode& node) {
    std::string pending;
    while (!c.eof()) {
        if (c.starts("<![CDATA[")) {
            c.i += 9;
            const size_t end = c.s.find("]]>", c.i);
            const size_t stop = (end == std::string::npos) ? c.s.size() : end;
            // Verbatim: this is Lua, and decoding entities inside it would
            // corrupt every `a < b` in the file.
            pending += c.s.substr(c.i, stop - c.i);
            c.i = (end == std::string::npos) ? c.s.size() : end + 3;
            continue;
        }
        if (c.starts("<!--")) { skipIgnorable(c); continue; }
        if (c.starts("</")) {
            c.i += 2;
            const size_t nameStart = c.i;
            while (!c.eof() && nameChar(c.peek())) ++c.i;
            const std::string closeName = c.s.substr(nameStart, c.i - nameStart);
            c.skipSpace();
            if (c.peek() != '>') { c.error = "malformed closing tag for " + closeName; return false; }
            ++c.i;
            if (closeName != node.name) {
                c.error = "closing tag " + closeName + " does not match " + node.name;
                return false;
            }
            node.text += decodeEntities(pending);
            return true;
        }
        if (c.peek() == '<') {
            XmlNode child;
            if (!parseElement(c, child)) return false;
            node.children.push_back(std::move(child));
            continue;
        }
        pending += c.peek();
        ++c.i;
    }
    c.error = "unexpected end of input inside <" + node.name + ">";
    return false;
}

bool parseElement(Cursor& c, XmlNode& out) {
    while (skipIgnorable(c)) {}
    if (c.peek() != '<') { c.error = "expected an element"; return false; }
    ++c.i;

    const size_t nameStart = c.i;
    while (!c.eof() && nameChar(c.peek())) ++c.i;
    out.name = c.s.substr(nameStart, c.i - nameStart);
    if (out.name.empty()) { c.error = "element with no name"; return false; }

    for (;;) {
        c.skipSpace();
        if (c.eof()) { c.error = "unexpected end of input in <" + out.name + ">"; return false; }
        if (c.peek() == '/' && c.peek(1) == '>') { c.i += 2; return true; }   // self-closing
        if (c.peek() == '>') { ++c.i; return parseContent(c, out); }

        const size_t keyStart = c.i;
        while (!c.eof() && nameChar(c.peek())) ++c.i;
        const std::string key = c.s.substr(keyStart, c.i - keyStart);
        if (key.empty()) { c.error = "malformed attribute in <" + out.name + ">"; return false; }
        c.skipSpace();
        if (c.peek() != '=') { c.error = "attribute " + key + " has no value"; return false; }
        ++c.i;
        c.skipSpace();
        const char quote = c.peek();
        if (quote != '"' && quote != '\'') {
            c.error = "attribute " + key + " is not quoted";
            return false;
        }
        ++c.i;
        const size_t valStart = c.i;
        while (!c.eof() && c.peek() != quote) ++c.i;
        if (c.eof()) { c.error = "attribute " + key + " is not terminated"; return false; }
        out.attrs[key] = decodeEntities(c.s.substr(valStart, c.i - valStart));
        ++c.i;
    }
}

} // namespace

bool parseXml(const std::string& source, XmlNode& outRoot, std::string& error) {
    Cursor c{.s = source};
    while (skipIgnorable(c)) {}
    if (c.eof()) { error = "document is empty"; return false; }
    if (!parseElement(c, outRoot)) { error = c.error; return false; }
    return true;
}

} // namespace ui
} // namespace wowee
