#pragma once

// Keeping the player's account name out of the log.
//
// Nearly every path this client logs is under the home directory - the data
// folder, the config, the log itself - so a log posted with a bug report
// carried the name the player signs in to their computer with, on line after
// line, and they had to find and redact each one. The home directory is
// written as "~" instead, which says just as much about where a file is.
//
// Header-only and free of the logger so it can be tested.

#include <cctype>
#include <cstdlib>
#include <string>

namespace wowee {
namespace core {

/// `text` with every whole-component occurrence of `home` written as "~".
///
/// Whole component: the match must end at a separator, a quote, the end of
/// the text or anything else that cannot continue a name - so a home of
/// /Users/k leaves /Users/kelsey alone rather than making it ~elsey. Windows
/// paths are matched without regard to case and with either separator, since
/// both spellings reach the log.
inline std::string redactHome(std::string text, const std::string& home) {
    // A home of "/" or empty would match everything or nothing useful.
    if (home.size() < 2) return text;
    std::string root = home;
    while (root.size() > 1 && (root.back() == '/' || root.back() == '\\')) root.pop_back();

#ifdef _WIN32
    constexpr bool kFoldCase = true;
#else
    constexpr bool kFoldCase = false;
#endif
    auto sameChar = [](char a, char b) {
        if (a == b) return true;
        const bool sepA = a == '/' || a == '\\';
        const bool sepB = b == '/' || b == '\\';
        if (sepA && sepB) return true;
        if (!kFoldCase) return false;
        return std::tolower(static_cast<unsigned char>(a)) ==
               std::tolower(static_cast<unsigned char>(b));
    };
    auto continuesName = [](char c) {
        return std::isalnum(static_cast<unsigned char>(c)) || c == '.' || c == '_' ||
               c == '-' || static_cast<unsigned char>(c) >= 0x80;
    };

    std::string out;
    out.reserve(text.size());
    size_t i = 0;
    while (i < text.size()) {
        bool match = text.size() - i >= root.size();
        for (size_t k = 0; match && k < root.size(); ++k) {
            match = sameChar(text[i + k], root[k]);
        }
        const size_t end = i + root.size();
        if (match && (end == text.size() || !continuesName(text[end]))) {
            out += '~';
            i = end;
            continue;
        }
        out += text[i++];
    }
    return out;
}

/// The home directory this process runs under, or empty if it will not say.
inline std::string homeDirectory() {
#ifdef _WIN32
    const char* home = std::getenv("USERPROFILE");
#else
    const char* home = std::getenv("HOME");
#endif
    return (home != nullptr) ? std::string(home) : std::string();
}

}  // namespace core
}  // namespace wowee
