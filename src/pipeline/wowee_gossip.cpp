#include "pipeline/wowee_gossip.hpp"
#include "pipeline/wowee_binary_io.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>

namespace wowee {
namespace pipeline {

namespace {

constexpr char kMagic[4] = {'W', 'G', 'S', 'P'};
constexpr uint32_t kVersion = 1;
constexpr char kExtension[] = ".wgsp";

} // namespace

const WoweeGossip::Entry* WoweeGossip::findById(uint32_t menuId) const {
    for (const auto& e : entries) {
        if (e.menuId == menuId) return &e;
    }
    return nullptr;
}

const char* WoweeGossip::optionKindName(uint8_t k) {
    switch (k) {
        case Close:        return "close";
        case Submenu:      return "submenu";
        case Vendor:       return "vendor";
        case Trainer:      return "trainer";
        case Quest:        return "quest";
        case Tabard:       return "tabard";
        case Banker:       return "banker";
        case Innkeeper:    return "innkeeper";
        case FlightMaster: return "flight";
        case TextOnly:     return "text";
        case Script:       return "script";
        case Battlemaster: return "battlemaster";
        case Auctioneer:   return "auctioneer";
        default:           return "unknown";
    }
}

bool WoweeGossipLoader::save(const WoweeGossip& cat,
                     const std::string& basePath) {
    return saveCatalog(cat, basePath, kMagic, kVersion, kExtension,
                       [](std::ofstream& os, const WoweeGossip::Entry& e) {
        writePOD(os, e.menuId);
        writeStr(os, e.titleText);
        uint8_t optCount = static_cast<uint8_t>(
            e.options.size() > 255 ? 255 : e.options.size());
        writePOD(os, optCount);
        writePadding(os, 3);
        for (uint8_t k = 0; k < optCount; ++k) {
            const auto& o = e.options[k];
            writePOD(os, o.optionId);
            writeStr(os, o.text);
            writePOD(os, o.kind);
            writePadding(os, 3);
            writePOD(os, o.actionTarget);
            writePOD(os, o.requiredFlags);
            writePOD(os, o.moneyCostCopper);
        }
                       });
}

WoweeGossip WoweeGossipLoader::load(
    const std::string& basePath) {
    return loadCatalog<WoweeGossip>(basePath, kMagic, kVersion, kExtension,
                              [](std::ifstream& is, WoweeGossip::Entry& e) {
        if (!readPOD(is, e.menuId)) { return false; }
        if (!readStr(is, e.titleText)) { return false; }
        uint8_t optCount = 0;
        if (!readPOD(is, optCount)) { return false; }
        if (!skipPadding(is, 3)) { return false; }
        e.options.resize(optCount);
        for (uint8_t k = 0; k < optCount; ++k) {
            auto& o = e.options[k];
            if (!readPOD(is, o.optionId)) { return false; }
            if (!readStr(is, o.text)) { return false; }
            if (!readPOD(is, o.kind)) { return false; }
            if (!skipPadding(is, 3)) { return false; }
            if (!readPOD(is, o.actionTarget) ||
                !readPOD(is, o.requiredFlags) ||
                !readPOD(is, o.moneyCostCopper)) { return false; }
        }
                                  return true;
                              });
}

bool WoweeGossipLoader::exists(const std::string& basePath) {
    return catalogExists(basePath, kExtension);
}

WoweeGossip WoweeGossipLoader::makeStarter(const std::string& catalogName) {
    WoweeGossip c;
    c.name = catalogName;
    {
        WoweeGossip::Entry e;
        e.menuId = 1;
        e.titleText = "Greetings, traveler. How can I help?";
        e.options.push_back({.optionId = 1, .text = "I want to browse your goods.",
                              .kind = WoweeGossip::Vendor, .actionTarget = 0,
                              .requiredFlags = WoweeGossip::Closes, .moneyCostCopper = 0});
        e.options.push_back({.optionId = 2, .text = "Train me.",
                              .kind = WoweeGossip::Trainer, .actionTarget = 0,
                              .requiredFlags = WoweeGossip::Closes, .moneyCostCopper = 0});
        e.options.push_back({.optionId = 3, .text = "Goodbye.",
                              .kind = WoweeGossip::Close, .actionTarget = 0,
                              .requiredFlags = WoweeGossip::Closes, .moneyCostCopper = 0});
        c.entries.push_back(e);
    }
    return c;
}

WoweeGossip WoweeGossipLoader::makeInnkeeper(const std::string& catalogName) {
    WoweeGossip c;
    c.name = catalogName;
    {
        // menuId 4001 deliberately matches what WCRT.makeStarter
        // and WCRT.makeMerchants set as Bartleby's gossipId
        // (currently 0 - set this when the demo content stack
        // is updated to wire WCRT.gossipId = 4001).
        WoweeGossip::Entry e;
        e.menuId = 4001;
        e.titleText =
            "Welcome to the inn! What'll it be - a room, "
            "a meal, or directions?";
        e.options.push_back({.optionId = 1, .text = "Make this inn my home.",
                              .kind = WoweeGossip::Innkeeper, .actionTarget = 0,
                              .requiredFlags = WoweeGossip::Closes, .moneyCostCopper = 0});
        e.options.push_back({.optionId = 2, .text = "Show me what you have for sale.",
                              .kind = WoweeGossip::Vendor, .actionTarget = 4001,
                              .requiredFlags = WoweeGossip::Closes, .moneyCostCopper = 0});
        e.options.push_back({.optionId = 3, .text = "I need to take a flight.",
                              .kind = WoweeGossip::FlightMaster, .actionTarget = 0,
                              .requiredFlags = WoweeGossip::Closes, .moneyCostCopper = 0});
        e.options.push_back({.optionId = 4, .text = "Tell me about the area.",
                              .kind = WoweeGossip::Submenu, .actionTarget = 4002,
                              .requiredFlags = 0, .moneyCostCopper = 0});
        e.options.push_back({.optionId = 5, .text = "Goodbye.",
                              .kind = WoweeGossip::Close, .actionTarget = 0,
                              .requiredFlags = WoweeGossip::Closes, .moneyCostCopper = 0});
        c.entries.push_back(e);
    }
    {
        // Submenu reached from the "tell me about the area" option.
        WoweeGossip::Entry e;
        e.menuId = 4002;
        e.titleText =
            "There's been bandit trouble of late. The Defias "
            "have a camp east of here. Mind your purse on the "
            "road.";
        e.options.push_back({.optionId = 1, .text = "Back.",
                              .kind = WoweeGossip::Submenu, .actionTarget = 4001,
                              .requiredFlags = 0, .moneyCostCopper = 0});
        e.options.push_back({.optionId = 2, .text = "Goodbye.",
                              .kind = WoweeGossip::Close, .actionTarget = 0,
                              .requiredFlags = WoweeGossip::Closes, .moneyCostCopper = 0});
        c.entries.push_back(e);
    }
    return c;
}

WoweeGossip WoweeGossipLoader::makeQuestGiver(const std::string& catalogName) {
    WoweeGossip c;
    c.name = catalogName;
    {
        WoweeGossip::Entry e;
        e.menuId = 5000;
        e.titleText =
            "I have work for someone of your obvious talent.";
        // Quest options reference WQT.questId values from
        // makeStarter/makeChain.
        e.options.push_back({.optionId = 1, .text = "Tell me about Bandit Trouble.",
                              .kind = WoweeGossip::Quest, .actionTarget = 1,
                              .requiredFlags = 0, .moneyCostCopper = 0});
        e.options.push_back({.optionId = 2, .text = "What's this about a camp?",
                              .kind = WoweeGossip::Quest, .actionTarget = 100,
                              .requiredFlags = 0, .moneyCostCopper = 0});
        e.options.push_back({.optionId = 3, .text = "I have business with the bank.",
                              .kind = WoweeGossip::Banker, .actionTarget = 0,
                              .requiredFlags = WoweeGossip::Closes, .moneyCostCopper = 0});
        e.options.push_back({.optionId = 4, .text = "Pay 10 gold to respec my talents.",
                              .kind = WoweeGossip::Script, .actionTarget = 9001,
                              .requiredFlags = WoweeGossip::Coinpouch | WoweeGossip::Closes,
                              .moneyCostCopper = 100000});  // 10g
        e.options.push_back({.optionId = 5, .text = "Goodbye.",
                              .kind = WoweeGossip::Close, .actionTarget = 0,
                              .requiredFlags = WoweeGossip::Closes, .moneyCostCopper = 0});
        c.entries.push_back(e);
    }
    return c;
}

} // namespace pipeline
} // namespace wowee
