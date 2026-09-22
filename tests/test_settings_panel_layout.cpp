// Do the options panels fit on the panels?
//
// The settings screens are generated: the schema says what the controls are and
// the Lua says where they go. Nothing else checks the arithmetic between those
// two, and nothing can - a control laid out past the bottom of the panel
// registers, refreshes, answers its value and reads back correctly. It is
// simply not on screen, and no behavioural test has an opinion about that.
//
// A search box added to the root panel in this session was drawn straight
// through the two blocks under it for exactly that reason: every check passed.
//
// So this one does the geometry. The column bounds are read out of the Lua
// itself rather than copied here, so the two cannot drift: change COLUMN_BOTTOM
// in the panel builder and this test lays the schema out against the new one.
#include <catch_amalgamated.hpp>

#include <algorithm>
#include <regex>
#include <string>
#include <vector>

#include "addons/addon_lua_snippets.hpp"
#include "ui/settings_schema.hpp"

using namespace wowee;

namespace {

/// A `local NAME = <number>` out of the panel builder's own source.
int luaConstant(const std::string& source, const std::string& name) {
    const std::regex pattern("local\\s+" + name + "\\s*=\\s*(-?[0-9]+)");
    std::smatch m;
    REQUIRE(std::regex_search(source, m, pattern));
    return std::stoi(m[1]);
}

/// The heights the builder reserves, which are its three `reserve(...)` calls.
constexpr int kHeadingHeight = 32;
constexpr int kCheckButtonHeight = 27;
constexpr int kSliderHeight = 50;  // and dropdowns, which reserve the same

/// The room a panel has, and how newLayout divides it.
///
/// The builder measures the frame it was given rather than reading constants,
/// so this measures the same way. The panel size is the fallback pair written
/// into newLayout itself - 413 x 428, which is what
/// InterfaceOptionsFramePanelContainer really comes out at, asked of the real
/// FrameXML through framexml_run.
///
/// This used to read COLUMN_X, COLUMN_WIDTH and COLUMN_BOTTOM out of the Lua.
/// Those described a container 623 wide and 446 tall that nothing has ever
/// been laid out in; the builder stopped using them when it started measuring,
/// and the test went on checking them. So it was laying the schema out against
/// a panel half again as wide as the real one, and passing.
struct Layout {
    int panelWidth;
    int panelHeight;
    int columnWidth;
    std::vector<int> columns;
    int bottom;
};

Layout panelLayout(const std::string& lua) {
    // The fallback dimensions, which are the measured ones.
    const std::regex fallbackW(R"(if\s+width\s*<=\s*0\s+then\s+width\s*=\s*([0-9]+))");
    const std::regex fallbackH(R"(if\s+height\s*<=\s*0\s+then\s+height\s*=\s*([0-9]+))");
    const std::regex spacing(
        R"(local\s+margin,\s*gap,\s*minWidth\s*=\s*([0-9]+),\s*([0-9]+),\s*([0-9]+))");
    std::smatch w;
    std::smatch h;
    std::smatch sp;
    REQUIRE(std::regex_search(lua, w, fallbackW));
    REQUIRE(std::regex_search(lua, h, fallbackH));
    REQUIRE(std::regex_search(lua, sp, spacing));

    Layout out;
    out.panelWidth = std::stoi(w[1]);
    out.panelHeight = std::stoi(h[1]);
    const int margin = std::stoi(sp[1]);
    const int gap = std::stoi(sp[2]);
    const int minWidth = std::stoi(sp[3]);

    const int twoWide = (out.panelWidth - margin * 2 - gap) / 2;
    if (twoWide >= minWidth) {
        out.columnWidth = twoWide;
        out.columns = {margin, margin + out.columnWidth + gap};
    } else {
        out.columnWidth = std::max(minWidth, out.panelWidth - margin * 2);
        out.columns = {margin};
    }
    out.bottom = -(out.panelHeight - 10);
    return out;
}

}  // namespace

TEST_CASE("every settings panel fits in its two columns", "[settings]") {
    const std::string lua = addons::kWoweeOptionsPanelLua;
    const int columnTop = luaConstant(lua, "COLUMN_TOP");
    const Layout layout = panelLayout(lua);
    const int columnBottom = layout.bottom;
    const int columnCount = static_cast<int>(layout.columns.size());
    REQUIRE(columnBottom < columnTop);

    std::size_t count = 0;
    const auto* schema = ui::clientSettingsSchema(count);
    REQUIRE(count > 0);

    // Walk the schema exactly as buildPanel does: a category is a panel, a
    // section adds a heading, and a control that will not fit moves to the
    // second column - after which there is nowhere else to go.
    std::string category, section;
    int column = 1;
    int y = columnTop;
    for (std::size_t i = 0; i <= count; ++i) {
        const bool last = (i == count);
        const std::string thisCategory = last ? std::string() : schema[i].category;
        if (last || thisCategory != category) {
            category = thisCategory;
            section.clear();
            column = 1;
            y = columnTop;
            if (last) break;
        }

        const int controlHeight = schema[i].kind == ui::SettingKind::Bool ? kCheckButtonHeight
                                                                          : kSliderHeight;
        const std::string thisSection = schema[i].section;
        int headingColumn = 0;
        if (!thisSection.empty() && thisSection != section) {
            section = thisSection;
            // A heading is kept with its first control, as addHeading asks
            // reserve to: both fit in this column, or both move.
            if (y - kHeadingHeight - controlHeight < columnBottom && column < columnCount) {
                ++column;
                y = columnTop;
            }
            y -= kHeadingHeight;
            headingColumn = column;
        }

        if (y - controlHeight < columnBottom && column < columnCount) {
            ++column;
            y = columnTop;
        }
        INFO("setting " << schema[i].key << " on panel " << schema[i].category
                        << " lands at " << (y - controlHeight) << " in column " << column
                        << ", past the bottom at " << columnBottom);
        CHECK(y - controlHeight >= columnBottom);
        // The heading over it is in the same column, not left at the foot of
        // the one before - which is how "Effects" came to sit alone at the
        // bottom of the Detail page.
        if (headingColumn != 0) CHECK(headingColumn == column);
        y -= controlHeight;
    }
}

TEST_CASE("a dropdown does not hang off the right of the panel", "[settings]") {
    // The dropdown is the widest control and the only one anchored back from
    // its column, because the template carries its own left inset. In the
    // second column that is the tightest fit on the panel.
    const std::string lua = addons::kWoweeOptionsPanelLua;
    const Layout layout = panelLayout(lua);
    const int lastColumnX = layout.columns.back();

    // What the builder does: anchor back by 14, then UIDropDownMenu_SetWidth
    // with the column width less 60. The template adds about 25 units of its
    // own chrome on each side of that.
    const int left = lastColumnX - 14;
    const int right = left + (layout.columnWidth - 60) + 25 * 2;

    INFO("a dropdown in the last column reaches " << right << " of "
         << layout.panelWidth);
    CHECK(right <= layout.panelWidth);
}

TEST_CASE("the root panel's blocks do not sit inside each other", "[settings]") {
    // This panel is laid out by hand, not generated, so the check is against
    // what the Lua says each block needs - a "needs N" note beside every
    // anchor. It is the panel a search box was inserted into the middle of
    // last pass, on top of the two blocks that were already there.
    const std::string lua = addons::kWoweeOptionsPanelLua;

    struct Block { int top; int needs; };
    std::vector<Block> blocks;
    const std::regex anchored(
        R"(SetPoint\("TOPLEFT",\s*-?[0-9]+,\s*(-[0-9]+)\)\s*--\s*needs\s+([0-9]+))");
    for (auto it = std::sregex_iterator(lua.begin(), lua.end(), anchored);
         it != std::sregex_iterator(); ++it) {
        blocks.push_back({-std::stoi((*it)[1]), std::stoi((*it)[2])});
    }
    // Every block on the panel carries one; a new block without a note would
    // be invisible to this check, so the count is asserted rather than assumed.
    REQUIRE(blocks.size() == 12);

    std::sort(blocks.begin(), blocks.end(),
              [](const Block& a, const Block& b) { return a.top < b.top; });

    int previousBottom = 0;
    for (const Block& b : blocks) {
        INFO("a block at -" << b.top << " starts inside the one above it, "
             << "which runs to -" << previousBottom);
        CHECK(b.top >= previousBottom);
        previousBottom = b.top + b.needs;
    }

    // InterfaceOptionsFramePanelContainer, measured rather than estimated.
    //
    // The frame the game puts these panels in is 648 x 520 in its own XML,
    // and the container inside it comes out at 413 x 428 - asked of the real
    // FrameXML through the headless runner. This said 492, which is 64 more
    // room than the panel has, and the About block was over the Okay and
    // Cancel buttons with this check passing.
    constexpr int kPanelHeight = 428;
    INFO("the root panel's content ends at -" << previousBottom);
    CHECK(previousBottom <= kPanelHeight);
}
