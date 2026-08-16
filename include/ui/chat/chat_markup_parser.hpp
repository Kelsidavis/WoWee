#pragma once

#include <imgui.h>
#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace ui {

/**
 * Segment types produced by ChatMarkupParser.
 *
 * Each segment represents a contiguous piece of a chat message
 * after WoW markup (|c...|r, |Hitem:...|h[...]|h, URLs) has been decoded.
 */
enum class SegmentType {
    Text,               // Plain text (render with base message color)
    ColoredText,        // Text with explicit |cAARRGGBB color
    ItemLink,           // |Hitem:ID:...|h[Name]|h
    SpellLink,          // |Hspell:ID:...|h[Name]|h
    QuestLink,          // |Hquest:ID:LEVEL|h[Name]|h
    AchievementLink,    // |Hachievement:ID:...|h[Name]|h
    Url,                // https://... URL
};

/**
 * A single parsed segment of a chat message.
 */
struct ChatSegment {
    SegmentType type    = SegmentType::Text;
    std::string text;           // display text (or URL)
    ImVec4      color   = ImVec4(1, 1, 1, 1);  // explicit color (for ColoredText / links)
    uint32_t    id      = 0;    // itemId / spellId / questId / achievementId
    uint32_t    extra   = 0;    // quest level (for QuestLink)
    std::string rawLink;        // full original markup for shift-click insertion
};

/**
 * Parses raw WoW-markup text into a flat list of typed segments.
 *
 * Extracted from ChatPanel::render() inline lambdas (Phase 2.1 of chat_panel_ref.md).
 * Pure logic - no ImGui calls, no game-state access. Fully unit-testable.
 */
class ChatMarkupParser {
public:
    /** Parse a raw chat message string into ordered segments. */
    [[nodiscard]] std::vector<ChatSegment> parse(const std::string& rawMessage) const;

    /** Parse |cAARRGGBB color code at given position. */
    static ImVec4 parseWowColor(const std::string& text, size_t pos);
};

} // namespace ui
} // namespace wowee
