#pragma once

#include <cctype>
#include <cstdint>
#include <deque>
#include <string>

namespace wowee {
namespace game {

/// One line the player has already been shown, for the spam filter.
struct RecentChatLine {
    uint64_t senderGuid = 0;
    std::string text;
    double at = 0.0;   ///< seconds, from any steady clock the caller likes
};

/// Whether this line is one the same sender has just said.
///
/// The Disable Spam Filter checkbox turns this off. What it filters is the
/// shape gold sellers use - the same line pasted again and again - so the test
/// is sender and text together within a short window. Two people saying "hi"
/// is not spam, and neither is one person saying "hi" twice an hour apart.
///
/// Case and surrounding space are ignored, because pasting the same line with
/// a capital changed is the cheapest way around a filter that does not.
inline bool repeatsRecentLine(const std::deque<RecentChatLine>& recent,
                              uint64_t senderGuid, const std::string& text,
                              double now, double windowSeconds = 30.0) {
    if (senderGuid == 0 || text.empty()) return false;

    auto squashed = [](const std::string& in) {
        std::string out;
        out.reserve(in.size());
        bool lastWasSpace = true;   // also trims the front
        for (unsigned char c : in) {
            if (std::isspace(c)) {
                if (!lastWasSpace) out.push_back(' ');
                lastWasSpace = true;
                continue;
            }
            out.push_back(static_cast<char>(std::tolower(c)));
            lastWasSpace = false;
        }
        while (!out.empty() && out.back() == ' ') out.pop_back();
        return out;
    };

    const std::string key = squashed(text);
    if (key.empty()) return false;
    for (const auto& line : recent) {
        if (line.senderGuid != senderGuid) continue;
        if (now - line.at > windowSeconds) continue;
        if (squashed(line.text) == key) return true;
    }
    return false;
}


} // namespace game
} // namespace wowee
