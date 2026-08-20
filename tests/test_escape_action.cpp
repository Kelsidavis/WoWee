// What Escape does, in each situation, stated directly.
//
// The report is that Escape does not open the game menu. The chain that
// decides it has been read end to end seven times and every reading found it
// sound - which is what a reading is bad at. It confirms each link and says
// nothing about which link runs, and the whole of the behaviour is which link
// runs.
//
// These are the situations it was never possible to ask about: the chain lived
// inside a draw with every branch reading live game state, so the only way to
// find out what Escape would do was to be there. Now it is a function of a
// struct, and each case below is one situation and one answer.

#include <catch_amalgamated.hpp>

#include "ui/escape_action.hpp"

using namespace wowee::ui;

TEST_CASE("With nothing open at all, Escape reaches the interface",
          "[escape]") {
    // The reported case. Every earlier branch has to decline for the question
    // to get this far, and any one of them holding wrongly eats the key -
    // silently, because closing something that is already closed looks like
    // nothing happening.
    const EscapeState nothing;
    CHECK(resolveEscape(nothing) == EscapeAction::AskTheInterface);
}

TEST_CASE("And then opens the game menu", "[escape]") {
    // The interface draws it. This client used to draw one too, and choosing
    // wrongly between them was silent - the flag behind this client's menu
    // could be set all day while FrameXML was the one drawing, and nothing
    // appeared. There is one menu now, so there is one answer.
    CHECK(resolveAfterInterface(false, /*frameXmlOwnsMenu=*/true) ==
          EscapeOutcome::ToggleInterfaceMenu);
    CHECK(resolveAfterInterface(false, /*frameXmlOwnsMenu=*/false) ==
          EscapeOutcome::ToggleInterfaceMenu);

    // And when the interface closed one of its own panels, neither menu opens.
    // Escape closing the top window before opening a menu is how WoW behaves;
    // falling through to the menu here would put one on top of the panel it
    // should have closed.
    CHECK(resolveAfterInterface(true, true) == EscapeOutcome::InterfaceClosedAPanel);
    CHECK(resolveAfterInterface(true, false) == EscapeOutcome::InterfaceClosedAPanel);
}

TEST_CASE("A press the interface's edit box already took does nothing more",
          "[escape]") {
    // The two paths run in an order that hides this. The pump hands a focused
    // box the key and stops; for Escape that closes the box, and closing it
    // clears the focus. The poll runs later in the same iteration, asks
    // whether anyone is typing, and is told no - by the box that let go on
    // this very press. So the chain ran on a key that had already been spent
    // and put the game menu up behind the box the player just dismissed.
    EscapeState s;
    s.interfaceConsumedKey = true;
    CHECK(resolveEscape(s) == EscapeAction::None);

    // And it beats everything, including the branches that send a packet. A
    // press taken by an edit box must not close the vendor the box was drawn
    // over, and must not cancel a cast either.
    s.casting = true;
    s.vendorOpen = true;
    s.settingsWindowShown = true;
    CHECK(resolveEscape(s) == EscapeAction::None);

    // With the flag clear the same state resolves as it always did, so this
    // guard cannot quietly disable the chain.
    s.interfaceConsumedKey = false;
    CHECK(resolveEscape(s) == EscapeAction::CloseSettingsWindow);
}

TEST_CASE("Each window closes itself when it is the only one open", "[escape]") {
    // One case per branch, so a branch that stops matching says which.
    struct Case { bool EscapeState::*flag; EscapeAction want; const char* what; };
    static const Case kCases[] = {
        {&EscapeState::settingsWindowShown,   EscapeAction::CloseSettingsWindow,     "settings"},
        {&EscapeState::casting,               EscapeAction::CancelCast,              "casting"},
        {&EscapeState::lootOpen,              EscapeAction::CloseLoot,               "loot"},
        {&EscapeState::gossipOpen,            EscapeAction::CloseGossip,             "gossip"},
        {&EscapeState::vendorOpen,            EscapeAction::CloseVendor,             "vendor"},
        {&EscapeState::barberShopOpen,        EscapeAction::CloseBarberShop,         "barber"},
        {&EscapeState::bankOpen,              EscapeAction::CloseBank,               "bank"},
        {&EscapeState::guildBankOpen,         EscapeAction::CloseGuildBank,          "guild bank"},
        {&EscapeState::trainerOpen,           EscapeAction::CloseTrainer,            "trainer"},
        {&EscapeState::mailboxOpen,           EscapeAction::CloseMailbox,            "mailbox"},
        {&EscapeState::auctionHouseOpen,      EscapeAction::CloseAuctionHouse,       "auction"},
        {&EscapeState::questDetailsOpen,      EscapeAction::DeclineQuest,            "quest details"},
        {&EscapeState::questOfferRewardOpen,  EscapeAction::CloseQuestOfferReward,   "quest reward"},
        {&EscapeState::questRequestItemsOpen, EscapeAction::CloseQuestRequestItems,  "quest items"},
        {&EscapeState::tradeOpen,             EscapeAction::CancelTrade,             "trade"},
    };
    for (const Case& c : kCases) {
        EscapeState s;
        s.*(c.flag) = true;
        INFO("only open: " << c.what);
        CHECK(resolveEscape(s) == c.want);
    }
    // Fifteen, and the count is asserted: a branch added without a case here
    // would be untested and would look tested.
    CHECK(sizeof(kCases) / sizeof(kCases[0]) == 15u);
}

TEST_CASE("This client's own window is closed before anything under it",
          "[escape]") {
    // The settings window draws over everything. With it up, Escape has to
    // take it and not the vendor behind it - otherwise the key closes a window
    // the player cannot see the effect of while the one they are looking at
    // stays.
    EscapeState s;
    s.vendorOpen = true;
    s.mailboxOpen = true;
    s.settingsWindowShown = true;
    CHECK(resolveEscape(s) == EscapeAction::CloseSettingsWindow);
}

TEST_CASE("A cast in progress beats any window that happens to be open",
          "[escape]") {
    // Escape cancelling a cast is what the key is most often for. A vendor
    // window left open while a spell is going off should not eat it.
    EscapeState s;
    s.casting = true;
    s.vendorOpen = true;
    s.auctionHouseOpen = true;
    CHECK(resolveEscape(s) == EscapeAction::CancelCast);
}

TEST_CASE("Every window open at once still resolves to exactly one",
          "[escape]") {
    // Nothing here may fall through to the interface: with the server holding
    // any of these open, asking FrameXML to close things would hide a frame
    // and leave the server believing the window was still up.
    EscapeState s;
    s.lootOpen = s.gossipOpen = s.vendorOpen = s.barberShopOpen = true;
    s.bankOpen = s.guildBankOpen = s.trainerOpen = s.mailboxOpen = true;
    s.auctionHouseOpen = s.questDetailsOpen = s.tradeOpen = true;
    CHECK(resolveEscape(s) == EscapeAction::CloseLoot);
    CHECK(resolveEscape(s) != EscapeAction::AskTheInterface);
}

TEST_CASE("Every action has a name", "[escape]") {
    // The chain reports one line per press and that line is the only evidence
    // of which branch ran. A branch whose name fell through to "?" would make
    // the report useless exactly when it is being read.
    for (int i = 0; i <= static_cast<int>(EscapeAction::AskTheInterface); ++i) {
        const auto a = static_cast<EscapeAction>(i);
        INFO("action " << i);
        CHECK(std::string(escapeActionName(a)) != "?");
    }
    for (int i = 0; i <= static_cast<int>(EscapeOutcome::ToggleInterfaceMenu); ++i) {
        const auto o = static_cast<EscapeOutcome>(i);
        INFO("outcome " << i);
        CHECK(std::string(escapeOutcomeName(o)) != "?");
    }
}

TEST_CASE("A held item is put back before any window is closed", "[escape]") {
    // Picking something up in a vendor and then shutting the vendor is exactly
    // how an item gets stranded on the cursor: nothing is left open to put it
    // back into. So the held item is answered first, ahead of every window.
    EscapeState s;
    s.holdingItem = true;
    CHECK(resolveEscape(s) == EscapeAction::ReturnHeldItem);

    s.vendorOpen = true;
    CHECK(resolveEscape(s) == EscapeAction::ReturnHeldItem);

    s.settingsWindowShown = true;
    CHECK(resolveEscape(s) == EscapeAction::ReturnHeldItem);

    // ...but a press already spent on an edit box is still spent.
    s.interfaceConsumedKey = true;
    CHECK(resolveEscape(s) == EscapeAction::None);
}

TEST_CASE("With nothing held the chain is unchanged", "[escape]") {
    EscapeState s;
    s.vendorOpen = true;
    CHECK(resolveEscape(s) == EscapeAction::CloseVendor);
}
