#pragma once

// A small XML reader, enough for FrameXML.
//
// FrameXML is not general XML and does not need a general parser: no DTDs, no
// processing instructions beyond the declaration, no namespaces in use beyond a
// single xmlns attribute nobody reads. What it does need is the awkward parts -
// CDATA around inline Lua, comments anywhere, self-closing elements, and both
// quote styles on attributes - because those appear on nearly every page of it.
//
// Kept free of Lua and of the widget tree so it can be tested on its own.

#include <map>
#include <string>
#include <vector>

namespace wowee {
namespace ui {

struct XmlNode {
    std::string name;
    std::map<std::string, std::string> attrs;
    /// Text content, with CDATA sections concatenated in. Inline scripts arrive
    /// here.
    std::string text;
    std::vector<XmlNode> children;

    /// Attribute lookup, case-sensitive as XML requires. Returns nullptr when
    /// absent, which callers distinguish from present-but-empty.
    [[nodiscard]] const std::string* attr(const std::string& key) const {
        auto it = attrs.find(key);
        return it == attrs.end() ? nullptr : &it->second;
    }
    [[nodiscard]] std::string attrOr(const std::string& key, const std::string& fallback) const {
        const std::string* v = attr(key);
        return v ? *v : fallback;
    }
    /// FrameXML writes booleans as "true"/"false".
    [[nodiscard]] bool attrBool(const std::string& key, bool fallback = false) const {
        const std::string* v = attr(key);
        if (!v) return fallback;
        return *v == "true" || *v == "1";
    }
    [[nodiscard]] float attrFloat(const std::string& key, float fallback = 0.0f) const {
        const std::string* v = attr(key);
        if (!v || v->empty()) return fallback;
        try { return std::stof(*v); } catch (...) { return fallback; }
    }
    [[nodiscard]] const XmlNode* child(const std::string& name) const {
        for (const auto& c : children) if (c.name == name) return &c;
        return nullptr;
    }
};

/// Parse a document. Returns false and fills `error` on malformed input rather
/// than throwing, because one bad file among a hundred should not take the rest
/// of the interface down with it.
bool parseXml(const std::string& source, XmlNode& outRoot, std::string& error);

} // namespace ui
} // namespace wowee
