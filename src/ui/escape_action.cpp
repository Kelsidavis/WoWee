#include "ui/escape_action.hpp"

namespace wowee::ui {

EscapeAction resolveEscape(const EscapeState& s) {
    // Before any of it: the press may already have been spent. A focused edit
    // box takes Escape in the event pump and closing the box is what that
    // does, so there is nothing left for the chain to act on - and acting
    // anyway puts the game menu up behind the box the player just dismissed.
    if (s.interfaceConsumedKey) return EscapeAction::None;
    // In order, and the order is the behaviour. Two rules hold it together:
    //
    // This client's own settings window goes first, because it is drawn over
    // everything and closing what is underneath while it is up would take the
    // wrong thing away.
    // An item on the cursor goes before every window, because closing one
    // while carrying an item is exactly how it gets stranded: pick something up
    // in a vendor, shut the vendor, and it is still on the cursor with nothing
    // left open to put it back into. Escape is the way out of a held item in
    // WoW, and it was the one thing this chain could not do.
    if (s.holdingItem)             return EscapeAction::ReturnHeldItem;
    if (s.settingsWindowShown)     return EscapeAction::CloseSettingsWindow;
    // A cast in progress goes before any window: Escape cancelling a cast is
    // what the key is most often for, and a window that happens to be open
    // should not eat it.
    if (s.casting)                 return EscapeAction::CancelCast;
    // Then everything the server believes is open, each closed through the
    // client so the closing packet is sent.
    if (s.lootOpen)                return EscapeAction::CloseLoot;
    if (s.gossipOpen)              return EscapeAction::CloseGossip;
    if (s.vendorOpen)              return EscapeAction::CloseVendor;
    if (s.barberShopOpen)          return EscapeAction::CloseBarberShop;
    if (s.bankOpen)                return EscapeAction::CloseBank;
    if (s.guildBankOpen)           return EscapeAction::CloseGuildBank;
    if (s.trainerOpen)             return EscapeAction::CloseTrainer;
    if (s.mailboxOpen)             return EscapeAction::CloseMailbox;
    if (s.auctionHouseOpen)        return EscapeAction::CloseAuctionHouse;
    if (s.questDetailsOpen)        return EscapeAction::DeclineQuest;
    if (s.questOfferRewardOpen)    return EscapeAction::CloseQuestOfferReward;
    if (s.questRequestItemsOpen)   return EscapeAction::CloseQuestRequestItems;
    if (s.tradeOpen)               return EscapeAction::CancelTrade;
    // Nothing this client owns is open. What is left is the interface's own -
    // the character sheet, the spellbook, the quest log, the world map - and
    // whether one of those is up is its answer to give, not ours.
    return EscapeAction::AskTheInterface;
}

EscapeOutcome resolveAfterInterface(bool interfaceClosedAPanel,
                                    bool frameXmlOwnsMenu) {
    if (interfaceClosedAPanel) return EscapeOutcome::InterfaceClosedAPanel;
    // The interface draws the menu, and there is no longer another answer.
    // frameXmlOwnsMenu is kept so the shape of the question survives its one
    // remaining answer, and so a client menu coming back is a change here
    // rather than at every call site.
    (void)frameXmlOwnsMenu;
    return EscapeOutcome::ToggleInterfaceMenu;
}

const char* escapeActionName(EscapeAction action) {
    switch (action) {
        case EscapeAction::None:                    return "nothing";
        case EscapeAction::ReturnHeldItem:          return "put the held item back";
        case EscapeAction::CloseSettingsWindow:     return "close the settings window";
        case EscapeAction::CancelCast:              return "cancel the cast";
        case EscapeAction::CloseLoot:               return "close loot";
        case EscapeAction::CloseGossip:             return "close gossip";
        case EscapeAction::CloseVendor:             return "close the vendor";
        case EscapeAction::CloseBarberShop:         return "close the barber shop";
        case EscapeAction::CloseBank:               return "close the bank";
        case EscapeAction::CloseGuildBank:          return "close the guild bank";
        case EscapeAction::CloseTrainer:            return "close the trainer";
        case EscapeAction::CloseMailbox:            return "close the mailbox";
        case EscapeAction::CloseAuctionHouse:       return "close the auction house";
        case EscapeAction::DeclineQuest:            return "decline the quest";
        case EscapeAction::CloseQuestOfferReward:   return "close the quest reward";
        case EscapeAction::CloseQuestRequestItems:  return "close the quest items";
        case EscapeAction::CancelTrade:             return "cancel the trade";
        case EscapeAction::AskTheInterface:         return "ask the interface";
    }
    return "?";
}

const char* escapeOutcomeName(EscapeOutcome outcome) {
    switch (outcome) {
        case EscapeOutcome::InterfaceClosedAPanel:
            return "the interface closed a panel of its own";
        case EscapeOutcome::ToggleInterfaceMenu:
            return "toggle the interface's game menu";
    }
    return "?";
}

}  // namespace wowee::ui
