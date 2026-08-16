// ChatMarkupParser - parse WoW markup into typed segments.
// Moved from inline lambdas in ChatPanel::render() (Phase 2.1).
#include "ui/chat/chat_markup_parser.hpp"
#include <algorithm>
#include <cstdlib>

namespace wowee { namespace ui {

ImVec4 ChatMarkupParser::parseWowColor(const std::string& text, size_t pos) {
    // |cAARRGGBB (10 chars total: |c + 8 hex)
    if (pos + 10 > text.size()) return ImVec4(1, 1, 1, 1);
    auto hexByte = [&](size_t offset) -> float {
        const char* s = text.c_str() + pos + offset;
        char buf[3] = {s[0], s[1], '\0'};
        return static_cast<float>(strtol(buf, nullptr, 16)) / 255.0f;
    };
    float a = hexByte(2);
    float r = hexByte(4);
    float g = hexByte(6);
    float b = hexByte(8);
    return ImVec4(r, g, b, a);
}

std::vector<ChatSegment> ChatMarkupParser::parse(const std::string& text) const {
    std::vector<ChatSegment> segments;
    size_t pos = 0;

    while (pos < text.size()) {
        // Find next special element: URL or WoW link
        size_t urlStart = text.find("https://", pos);

        // Find next WoW link (may be colored with |c prefix or bare |H)
        size_t linkStart = text.find("|c", pos);
        // Also handle bare |H links without color prefix
        size_t bareItem  = text.find("|Hitem:",  pos);
        size_t bareSpell = text.find("|Hspell:", pos);
        size_t bareQuest = text.find("|Hquest:", pos);
        size_t bareLinkStart = std::min({bareItem, bareSpell, bareQuest});

        // Determine which comes first
        size_t nextSpecial = std::min({urlStart, linkStart, bareLinkStart});

        if (nextSpecial == std::string::npos) {
            // No more special elements - remainder is plain text
            std::string remaining = text.substr(pos);
            if (!remaining.empty()) {
                segments.push_back({.type = SegmentType::Text, .text = std::move(remaining)});
            }
            break;
        }

        // Emit plain text before the special element
        if (nextSpecial > pos) {
            segments.push_back({.type = SegmentType::Text, .text = text.substr(pos, nextSpecial - pos)});
        }

        // Handle WoW link (|c... or bare |H...)
        if (nextSpecial == linkStart || nextSpecial == bareLinkStart) {
            ImVec4 linkColor(1, 1, 1, 1);
            size_t hStart = std::string::npos;

            if (nextSpecial == linkStart && text.size() > linkStart + 10) {
                // Parse |cAARRGGBB color
                linkColor = parseWowColor(text, linkStart);
                // Find the nearest |H link of any supported type
                size_t hItem  = text.find("|Hitem:",        linkStart + 10);
                size_t hSpell = text.find("|Hspell:",       linkStart + 10);
                size_t hQuest = text.find("|Hquest:",       linkStart + 10);
                size_t hAch   = text.find("|Hachievement:", linkStart + 10);
                hStart = std::min({hItem, hSpell, hQuest, hAch});
            } else if (nextSpecial == bareLinkStart) {
                hStart = bareLinkStart;
            }

            if (hStart != std::string::npos) {
                // Determine link type
                const bool isSpellLink = (text.compare(hStart, 8, "|Hspell:") == 0);
                const bool isQuestLink = (text.compare(hStart, 8, "|Hquest:") == 0);
                const bool isAchievLink = (text.compare(hStart, 14, "|Hachievement:") == 0);

                // Parse the first numeric ID after |Htype:
                size_t idOffset = isSpellLink ? 8 : (isQuestLink ? 8 : (isAchievLink ? 14 : 7));
                size_t entryStart = hStart + idOffset;
                size_t entryEnd = text.find(':', entryStart);
                uint32_t linkId = 0;
                if (entryEnd != std::string::npos) {
                    linkId = static_cast<uint32_t>(strtoul(
                        text.substr(entryStart, entryEnd - entryStart).c_str(), nullptr, 10));
                }

                // Parse quest level (second field after questId)
                uint32_t questLevel = 0;
                if (isQuestLink && entryEnd != std::string::npos) {
                    size_t lvlEnd = text.find(':', entryEnd + 1);
                    if (lvlEnd == std::string::npos) lvlEnd = text.find('|', entryEnd + 1);
                    if (lvlEnd != std::string::npos) {
                        questLevel = static_cast<uint32_t>(strtoul(
                            text.substr(entryEnd + 1, lvlEnd - entryEnd - 1).c_str(), nullptr, 10));
                    }
                }

                // Find display name: |h[Name]|h
                size_t nameTagStart = text.find("|h[", hStart);
                size_t nameTagEnd = (nameTagStart != std::string::npos)
                    ? text.find("]|h", nameTagStart + 3) : std::string::npos;

                std::string linkName = isSpellLink ? "Unknown Spell"
                                    : isQuestLink  ? "Unknown Quest"
                                    : isAchievLink ? "Unknown Achievement"
                                    : "Unknown Item";
                if (nameTagStart != std::string::npos && nameTagEnd != std::string::npos) {
                    linkName = text.substr(nameTagStart + 3, nameTagEnd - nameTagStart - 3);
                }

                // Find end of entire link sequence (|r or after ]|h)
                size_t linkEnd = (nameTagEnd != std::string::npos) ? nameTagEnd + 3 : hStart + idOffset;
                size_t resetPos = text.find("|r", linkEnd);
                if (resetPos != std::string::npos && resetPos <= linkEnd + 2) {
                    linkEnd = resetPos + 2;
                }

                // Build raw link text for shift-click re-insertion
                std::string rawLink = text.substr(nextSpecial, linkEnd - nextSpecial);

                // Emit appropriate segment type
                SegmentType stype = isSpellLink ? SegmentType::SpellLink
                                  : isQuestLink ? SegmentType::QuestLink
                                  : isAchievLink ? SegmentType::AchievementLink
                                  : SegmentType::ItemLink;

                ChatSegment seg;
                seg.type = stype;
                seg.text = std::move(linkName);
                seg.color = linkColor;
                seg.id = linkId;
                seg.extra = questLevel;
                seg.rawLink = std::move(rawLink);
                segments.push_back(std::move(seg));

                pos = linkEnd;
                continue;
            }

            // Not an item link - treat as colored text: |cAARRGGBB...text...|r
            if (nextSpecial == linkStart && text.size() > linkStart + 10) {
                ImVec4 cColor = parseWowColor(text, linkStart);
                size_t textStart = linkStart + 10; // after |cAARRGGBB
                size_t resetPos2 = text.find("|r", textStart);
                std::string coloredText;
                if (resetPos2 != std::string::npos) {
                    coloredText = text.substr(textStart, resetPos2 - textStart);
                    pos = resetPos2 + 2; // skip |r
                } else {
                    coloredText = text.substr(textStart);
                    pos = text.size();
                }
                // Strip any remaining WoW markup from the colored segment
                // (e.g. |H...|h pairs that aren't item links)
                std::string clean;
                for (size_t i = 0; i < coloredText.size(); i++) {
                    if (coloredText[i] == '|' && i + 1 < coloredText.size()) {
                        char next = coloredText[i + 1];
                        if (next == 'H') {
                            // Skip |H...|h
                            size_t hEnd = coloredText.find("|h", i + 2);
                            if (hEnd != std::string::npos) { i = hEnd + 1; continue; }
                        } else if (next == 'h') {
                            i += 1; continue; // skip |h
                        } else if (next == 'r') {
                            i += 1; continue; // skip |r
                        }
                    }
                    clean += coloredText[i];
                }
                if (!clean.empty()) {
                    ChatSegment seg;
                    seg.type = SegmentType::ColoredText;
                    seg.text = std::move(clean);
                    seg.color = cColor;
                    segments.push_back(std::move(seg));
                }
            } else {
                // Bare |c without enough chars for color - render literally
                segments.push_back({.type = SegmentType::Text, .text = "|c"});
                pos = nextSpecial + 2;
            }
            continue;
        }

        // Handle URL
        if (nextSpecial == urlStart) {
            size_t urlEnd = text.find_first_of(" \t\n\r", urlStart);
            if (urlEnd == std::string::npos) urlEnd = text.size();
            std::string url = text.substr(urlStart, urlEnd - urlStart);

            segments.push_back({.type = SegmentType::Url, .text = std::move(url)});
            pos = urlEnd;
            continue;
        }
    }

    return segments;
}

} // namespace ui
} // namespace wowee
