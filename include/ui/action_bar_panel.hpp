#pragma once

/// Which action-bar slot a number key presses.
///
/// All the drawing that used to be here is FrameXML's: the action bar, the
/// stance bar, the bag bar and the two thin bars are one frame there, and this
/// client stopped drawing any of them. What is left is the arithmetic the key
/// handler needs - a page and a button index give a slot - and the page itself,
/// which the interface owns and tells this client on every change.

namespace wowee {
namespace ui {

class ActionBarPanel {
public:
    static constexpr int kFrameXmlActionBarPages = 6;
    static constexpr int kBottomLeftActionPage = 6;
    static constexpr int kRightActionPage = 3;
    static constexpr int kLeftActionPage = 4;

    static int actionSlotForPage(int page, int buttonIndex) {
        return (page - 1) * 12 + buttonIndex;
    }

    int getMainActionBarPage() const { return mainActionBarPage_; }
    /// Told by the interface, which owns the page.
    void setMainActionBarPage(int page) {
        if (page >= 1 && page <= kFrameXmlActionBarPages) mainActionBarPage_ = page;
    }

private:
    int mainActionBarPage_ = 1;  // FrameXML main pages are 1..6.
};

} // namespace ui
} // namespace wowee
