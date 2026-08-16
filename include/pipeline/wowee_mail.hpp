#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Mail Template catalog (.wmal) - novel
// replacement for AzerothCore-style mail_loot_template SQL
// + the in-game mail subset of the inventory + currency
// systems. The 34th open format added to the editor.
//
// Defines templated mail messages with currency + item
// attachments. Triggered by:
//   • quest reward delivery (overflow mail when bag is full)
//   • auction house bid wins / sales completion
//   • achievement reward attachments
//   • GM correspondence
//   • holiday event mailings (Brewfest samples, Hallow's End
//     candy, anniversary thank-you notes)
//   • returned-mail-on-rejection
//
// Cross-references with previously-added formats:
//   WMAL.entry.senderNpcId           → WCRT.entry.creatureId
//   WMAL.entry.attachments.itemId    → WIT.entry.itemId
//
// Binary layout (little-endian):
//   magic[4]            = "WMAL"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     templateId (uint32)
//     senderNpcId (uint32)
//     subjectLen + subject
//     bodyLen + body
//     senderLen + senderName
//     moneyCopperAttached (uint32)
//     attachmentCount (uint8) / categoryId (uint8) /
//       cod (uint8) / returnable (uint8)
//     expiryDays (uint16) / pad[2]
//     attachments (each: itemId (uint32) + quantity (uint16) + pad[2])
struct WoweeMail {
    enum Category : uint8_t {
        QuestReward     = 0,
        Auction         = 1,
        GmCorrespondence = 2,
        AchievementReward = 3,
        EventMailing    = 4,
        Raffle          = 5,
        ScriptDelivery  = 6,
        ReturnedMail    = 7,
    };

    struct Attachment {
        uint32_t itemId = 0;
        uint16_t quantity = 1;
    };

    struct Entry {
        uint32_t templateId = 0;
        uint32_t senderNpcId = 0;       // 0 = system / no NPC
        std::string subject;
        std::string body;
        std::string senderName;          // fallback when senderNpcId=0
        uint32_t moneyCopperAttached = 0;
        uint8_t categoryId = QuestReward;
        uint8_t cod = 0;                 // 1 = cash on delivery
        uint8_t returnable = 1;          // 1 = unread mail returns
        uint16_t expiryDays = 30;
        std::vector<Attachment> attachments;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t templateId) const;

    static const char* categoryName(uint8_t c);
};

class WoweeMailLoader {
public:
    static bool save(const WoweeMail& cat,
                     const std::string& basePath);
    static WoweeMail load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-mail* variants.
    //
    //   makeStarter - 3 templates covering quest reward
    //                  overflow / auction house / GM gift.
    //   makeHoliday - 4 holiday-event mailings tied to WSEA
    //                  yearly events (Tricky Treats sample
    //                  pack, Brewfest sampler, Lunar Festival
    //                  blessing, Winter's Veil gift box).
    //   makeAuction - full auction-house template family:
    //                  outbid / won / sold / expired / cancelled.
    static WoweeMail makeStarter(const std::string& catalogName);
    static WoweeMail makeHoliday(const std::string& catalogName);
    static WoweeMail makeAuction(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
