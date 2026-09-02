#pragma once

// The strings in SMSG_QUEST_QUERY_RESPONSE, and which of them is which.
//
// The response carries a run of C-strings after its numeric block - Title,
// Objectives, Details, EndText, CompletedText - and this client reads them at
// an offset it has to work out. So every read is checked for being text at all
// rather than trusted, and the two that can hold the "what to do now" line are
// chosen between rather than merged.
//
// Header-only and free of the packet reader so it can be tested: the choice
// between two strings is the part that was getting answered wrong, and it is
// pure.

#include <cctype>
#include <string>

namespace wowee::game {

/// Whether a string read out of the packet is text rather than the numbers on
/// either side of it: inside the length the field allows, free of control
/// characters, and carrying at least one letter.
///
/// UTF-8 lead and continuation bytes are text - a localized realm's quests are
/// full of them - but they do not count as the letter, because one ASCII letter
/// is what separates a sentence from a run of high bytes.
inline bool questTextIsReadable(const std::string& s, size_t minLen, size_t maxLen) {
    if (s.size() < minLen || s.size() > maxLen) return false;
    bool hasAlpha = false;
    for (unsigned char c : s) {
        if (c < 0x20 && c != '\t' && c != '\n' && c != '\r') return false;
        if (c >= 0x20 && c <= 0x7E && std::isalpha(c)) hasAlpha = true;
    }
    return hasAlpha;
}

/// The line a finished quest shows, out of the two strings that can hold one.
///
/// CompletedText is the fifth string and is what the game shows for a quest
/// that is done. EndText is the fourth, and on the clients that have no fifth
/// it is where "Return to Marshal Dughan at Goldshire in Elwynn Forest." lives.
/// So the fifth wins whenever it says anything and the fourth stands in when it
/// does not.
///
/// This used to take whichever of the two was longer, which is a different
/// question: a quest carrying both got its EndText wherever that ran longer
/// than its CompletedText, and neither read as text left the line blank rather
/// than falling back.
inline std::string questCompletedText(const std::string& endText,
                                      const std::string& completedText) {
    // The same bounds the reader uses for a one-line string: long enough to be
    // a sentence, short enough that a run of packet bytes cannot pass.
    constexpr size_t kMin = 8;
    constexpr size_t kMax = 600;
    if (questTextIsReadable(completedText, kMin, kMax)) return completedText;
    if (questTextIsReadable(endText, kMin, kMax)) return endText;
    return {};
}

} // namespace wowee::game
