#include "game/world_packets.hpp"
#include "game/packet_parsers.hpp"
#include "game/opcodes.hpp"
#include "game/character.hpp"
#include "auth/crypto.hpp"
#include "core/logger.hpp"
#include <algorithm>
#include <array>
#include <charconv>
#include <cctype>
#include <cmath>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <zlib.h>

namespace wowee {
namespace game {

bool parseAuctionMailSubject(const std::string& subject, AuctionMailSubject& result) {
    std::array<uint32_t, 5> fields{};
    size_t start = 0;
    size_t fieldCount = 0;
    while (start <= subject.size() && fieldCount < fields.size()) {
        const size_t end = subject.find(':', start);
        const size_t fieldEnd = end == std::string::npos ? subject.size() : end;
        if (fieldEnd == start) return false;
        const char* first = subject.data() + start;
        const char* last = subject.data() + fieldEnd;
        auto [parsedEnd, error] = std::from_chars(first, last, fields[fieldCount]);
        if (error != std::errc{} || parsedEnd != last) return false;
        ++fieldCount;
        if (end == std::string::npos) break;
        start = fieldEnd + 1;
    }

    // Modern subjects are itemEntry:0:response:lotId:itemCount. Legacy MaNGOS
    // cores emitted only itemEntry:0:response.
    if (fieldCount != 3 && fieldCount != 5) return false;
    if (fields[0] == 0 || fields[1] != 0 || fields[2] > 6) return false;
    result = {fields[0], fields[2],
              fieldCount == 5 ? fields[3] : 0,
              fieldCount == 5 ? fields[4] : 0};
    return true;
}

std::string formatAuctionMailSubject(const AuctionMailSubject& subject,
                                     const std::string& itemName) {
    static constexpr std::array<const char*, 7> kPrefixes = {
        "Outbid on ",
        "Auction won: ",
        "Auction successful: ",
        "Auction expired: ",
        "Auction cancelled: ",
        "Auction cancelled: ",
        "Sale Pending: ",
    };
    if (subject.response >= kPrefixes.size()) return itemName;
    return std::string(kPrefixes[subject.response]) + itemName;
}

// Auction-house mail carries a machine-readable invoice as its body:
//   "<hex ownerGuidLow>:<bid>:<buyout>[:<deposit>:<consignment>...]"
// The leading GUID field is right-justified to width 16, so it can arrive with
// leading spaces (e.g. "           88a79:6000:6000:0:0:0:0"). The retail client
// parses this into a formatted invoice rather than printing it verbatim.
bool parseAuctionMailBody(const std::string& body, AuctionMailInvoice& result) {
    std::array<uint32_t, 5> fields{};
    size_t fieldCount = 0;
    size_t start = 0;
    while (start <= body.size() && fieldCount < fields.size()) {
        // The GUID field is space-padded to width 16 — skip leading whitespace.
        while (start < body.size() &&
               std::isspace(static_cast<unsigned char>(body[start]))) {
            ++start;
        }
        const size_t end = body.find(':', start);
        const size_t fieldEnd = end == std::string::npos ? body.size() : end;
        if (fieldEnd == start) return false;
        const char* first = body.data() + start;
        const char* last = body.data() + fieldEnd;
        // Field 0 is the owner GUID in hex; the rest are decimal copper values.
        const int base = fieldCount == 0 ? 16 : 10;
        auto [parsedEnd, error] = std::from_chars(first, last, fields[fieldCount], base);
        if (error != std::errc{} || parsedEnd != last) return false;
        ++fieldCount;
        if (end == std::string::npos) break;
        start = fieldEnd + 1;
    }

    // Minimum invoice is ownerGuid:bid:buyout (auction-won mail); a successful
    // sale adds deposit and consignment. Any trailing fields (moneyDelay/eta on
    // some cores) are ignored.
    if (fieldCount < 3) return false;
    result.ownerGuidLow = fields[0];
    result.bid = fields[1];
    result.buyout = fields[2];
    result.deposit = fieldCount > 3 ? fields[3] : 0;
    result.consignment = fieldCount > 4 ? fields[4] : 0;
    return true;
}

bool ShowTaxiNodesParser::parse(network::Packet& packet, ShowTaxiNodesData& data) {
    // Minimum: windowInfo(4) + npcGuid(8) + nearestNode(4) + at least 1 mask uint32(4)
    size_t remaining = packet.getRemainingSize();
    if (remaining < 4 + 8 + 4 + 4) {
        LOG_ERROR("ShowTaxiNodesParser: packet too short (", remaining, " bytes)");
        return false;
    }
    data.windowInfo = packet.readUInt32();
    data.npcGuid = packet.readUInt64();
    data.nearestNode = packet.readUInt32();
    // Read as many mask uint32s as available (Classic/Vanilla=4, WotLK=12)
    size_t maskBytes = packet.getRemainingSize();
    uint32_t maskCount = static_cast<uint32_t>(maskBytes / 4);
    if (maskCount > TLK_TAXI_MASK_SIZE) maskCount = TLK_TAXI_MASK_SIZE;
    for (uint32_t i = 0; i < maskCount; ++i) {
        data.nodeMask[i] = packet.readUInt32();
    }
    LOG_INFO("ShowTaxiNodes: window=", data.windowInfo, " npc=0x", std::hex, data.npcGuid, std::dec,
             " nearest=", data.nearestNode, " maskSlots=", maskCount);
    return true;
}

bool ActivateTaxiReplyParser::parse(network::Packet& packet, ActivateTaxiReplyData& data) {
    size_t remaining = packet.getRemainingSize();
    if (remaining >= 4) {
        data.result = packet.readUInt32();
    } else if (remaining >= 1) {
        data.result = packet.readUInt8();
    } else {
        LOG_ERROR("ActivateTaxiReplyParser: packet too short");
        return false;
    }
    LOG_INFO("ActivateTaxiReply: result=", data.result);
    return true;
}

network::Packet ActivateTaxiExpressPacket::build(uint64_t npcGuid,
                                                const std::vector<uint32_t>& pathNodes) {
    // guid, node count, then the nodes. That is all of it.
    //
    // A total cost was being written between the guid and the count, and
    // HandleActivateTaxiExpressOpcode does not read one — so the server took
    // the cost as the number of nodes to expect. What followed was read as the
    // first node: the client's own count, a small number that is no taxi node
    // anyone has visited, which the server answers with ERR_TAXINOTVISITED and
    // rfinish. So a multi-hop flight was refused for a flight point the player
    // had certainly been to, and with a large enough cost the server ran off
    // the end of the buffer and dropped the packet instead.
    //
    // Every multi-hop route goes this way, which is why single hops worked and
    // nothing longer ever did.
    network::Packet packet(wireOpcode(Opcode::CMSG_ACTIVATETAXIEXPRESS));
    packet.writeUInt64(npcGuid);
    packet.writeUInt32(static_cast<uint32_t>(pathNodes.size()));
    for (uint32_t nodeId : pathNodes) {
        packet.writeUInt32(nodeId);
    }
    LOG_INFO("ActivateTaxiExpress: npc=0x", std::hex, npcGuid, std::dec,
             " nodes=", pathNodes.size());
    return packet;
}

network::Packet ActivateTaxiPacket::build(uint64_t npcGuid, uint32_t srcNode, uint32_t destNode) {
    network::Packet packet(wireOpcode(Opcode::CMSG_ACTIVATETAXI));
    packet.writeUInt64(npcGuid);
    packet.writeUInt32(srcNode);
    packet.writeUInt32(destNode);
    return packet;
}

network::Packet GameObjectUsePacket::build(uint64_t guid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GAMEOBJ_USE));
    packet.writeUInt64(guid);
    return packet;
}

// ============================================================
// Mail System
// ============================================================

network::Packet GetMailListPacket::build(uint64_t mailboxGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GET_MAIL_LIST));
    packet.writeUInt64(mailboxGuid);
    return packet;
}

network::Packet SendMailPacket::build(uint64_t mailboxGuid, const std::string& recipient,
                                      const std::string& subject, const std::string& body,
                                      uint64_t money, uint64_t cod,
                                      const std::vector<uint64_t>& itemGuids) {
    // WotLK 3.3.5a format
    network::Packet packet(wireOpcode(Opcode::CMSG_SEND_MAIL));
    packet.writeUInt64(mailboxGuid);
    packet.writeString(recipient);
    packet.writeString(subject);
    packet.writeString(body);
    packet.writeUInt32(0);       // stationery
    packet.writeUInt32(0);       // unknown
    uint8_t attachCount = static_cast<uint8_t>(itemGuids.size());
    packet.writeUInt8(attachCount);
    for (uint8_t i = 0; i < attachCount; ++i) {
        packet.writeUInt8(i);            // attachment slot index
        packet.writeUInt64(itemGuids[i]);
    }
    // WotLK represents both values as uint32, followed by the legacy
    // uint64/uint8 zero fields.  Writing the values as uint64 shifts COD into
    // the unknown fields and leaves the server one byte short.
    packet.writeUInt32(static_cast<uint32_t>(money));
    packet.writeUInt32(static_cast<uint32_t>(cod));
    packet.writeUInt64(0);
    packet.writeUInt8(0);
    return packet;
}

network::Packet MailTakeMoneyPacket::build(uint64_t mailboxGuid, uint32_t mailId) {
    network::Packet packet(wireOpcode(Opcode::CMSG_MAIL_TAKE_MONEY));
    packet.writeUInt64(mailboxGuid);
    packet.writeUInt32(mailId);
    return packet;
}

network::Packet MailTakeItemPacket::build(uint64_t mailboxGuid, uint32_t mailId, uint32_t itemGuidLow) {
    network::Packet packet(wireOpcode(Opcode::CMSG_MAIL_TAKE_ITEM));
    packet.writeUInt64(mailboxGuid);
    packet.writeUInt32(mailId);
    // WotLK expects attachment item GUID low, not attachment slot index.
    packet.writeUInt32(itemGuidLow);
    return packet;
}

network::Packet MailDeletePacket::build(uint64_t mailboxGuid, uint32_t mailId, uint32_t mailTemplateId) {
    network::Packet packet(wireOpcode(Opcode::CMSG_MAIL_DELETE));
    packet.writeUInt64(mailboxGuid);
    packet.writeUInt32(mailId);
    packet.writeUInt32(mailTemplateId);
    return packet;
}

network::Packet MailMarkAsReadPacket::build(uint64_t mailboxGuid, uint32_t mailId) {
    network::Packet packet(wireOpcode(Opcode::CMSG_MAIL_MARK_AS_READ));
    packet.writeUInt64(mailboxGuid);
    packet.writeUInt32(mailId);
    return packet;
}

// ============================================================================
// PacketParsers::parseMailList — WotLK 3.3.5a format (base/default)
// ============================================================================
bool PacketParsers::parseMailList(network::Packet& packet, std::vector<MailMessage>& inbox) {
    size_t remaining = packet.getRemainingSize();
    if (remaining < 5) return false;

    const size_t payloadSizeTotal = packet.getSize();
    uint32_t totalCount = packet.readUInt32();
    uint8_t shownCount = packet.readUInt8();
    (void)totalCount;

    LOG_INFO("SMSG_MAIL_LIST_RESULT (WotLK): total=", totalCount, " shown=", static_cast<int>(shownCount),
             " payloadBytes=", payloadSizeTotal);

    inbox.clear();
    inbox.reserve(shownCount);

    // Each entry: uint16 size + msgId(4) + type(1) + sender(min 4) + 7 fixed
    // uint32/float(28) + subject NUL(1) + body NUL(1) + attachCount(1) = 42 bytes.
    constexpr size_t kMinMailEntryBytes = 42;
    // Per attachment: slot(1) + guidLow(4) + itemId(4) + 7 enchant triplets(84)
    // + randProp(4) + suffix(4) + stack(4) + charges(4) + maxDur(4) + dur(4)
    // + trailing(1) = 118 bytes.
    constexpr size_t kAttachmentBytes = 1 + 4 + 4 + 7 * 12 + 4 + 4 + 4 + 4 + 4 + 4 + 1;

    for (uint8_t i = 0; i < shownCount; ++i) {
        remaining = packet.getRemainingSize();
        if (remaining < kMinMailEntryBytes) break;

        // The per-entry uint16 size is present on the wire but NOT trusted for
        // framing: some AzerothCore-derived cores over-declare it (observed a
        // consistent +4 per mail), so skipping to the declared end overshoots and
        // desyncs every mail after the first. Instead we advance by the natural
        // parse position — our field parse reads the complete documented WotLK
        // entry, so it lands exactly on the next entry on both correct and
        // over-declaring servers.
        packet.readUInt16(); // declared size (advisory only)
        const size_t startPos = packet.getReadPos();

        MailMessage msg;
        msg.messageId = packet.readUInt32();
        msg.messageType = packet.readUInt8();

        switch (msg.messageType) {
            case 0: msg.senderGuid = packet.readUInt64(); break;
            case 2: case 3: case 4: case 5:
                msg.senderEntry = packet.readUInt32(); break;
            default: msg.senderEntry = packet.readUInt32(); break;
        }

        msg.cod = packet.readUInt32();
        packet.readUInt32(); // unknown (was item text id before 3.3.3)
        msg.stationeryId = packet.readUInt32();
        msg.money = packet.readUInt32();
        msg.flags = packet.readUInt32();
        msg.expirationTime = packet.readFloat();
        msg.mailTemplateId = packet.readUInt32();
        msg.subject = packet.readString();
        // WotLK 3.3.5a always includes body text in SMSG_MAIL_LIST_RESULT.
        // mailTemplateId != 0 still carries a (possibly empty) body string.
        msg.body = packet.readString();

        uint8_t attachCount = packet.readUInt8();
        msg.attachments.reserve(attachCount);
        bool truncatedAttachment = false;
        for (uint8_t j = 0; j < attachCount; ++j) {
            if (!packet.hasRemaining(kAttachmentBytes)) {
                // Genuine buffer shortage mid-attachment (not the size-field quirk).
                LOG_WARNING("Mail entry ", static_cast<int>(i), " attachment ", static_cast<int>(j),
                            " truncated: remaining=", packet.getRemainingSize(),
                            " payloadBytes=", payloadSizeTotal);
                LOG_WARNING("Mail payload hex: ",
                            core::toHexString(packet.getData().data(), packet.getSize(), true));
                truncatedAttachment = true;
                break;
            }
            MailAttachment att;
            att.slot = packet.readUInt8();
            att.itemGuidLow = packet.readUInt32();
            att.itemId = packet.readUInt32();
            for (int e = 0; e < 7; ++e) {
                uint32_t enchId = packet.readUInt32();
                packet.readUInt32(); // duration
                packet.readUInt32(); // charges
                if (e == 0) att.enchantId = enchId;
            }
            att.randomPropertyId = packet.readUInt32();
            att.randomSuffix = packet.readUInt32();
            att.stackCount = packet.readUInt32();
            att.chargesOrDurability = packet.readUInt32();
            att.maxDurability = packet.readUInt32();
            packet.readUInt32(); // durability/current durability
            packet.readUInt8();  // unknown WotLK trailing byte per attachment
            msg.attachments.push_back(att);
        }

        msg.read = (msg.flags & 0x01) != 0;
        inbox.push_back(std::move(msg));

        if (truncatedAttachment) break;
        // Guard against a corrupt entry that consumed nothing, which would spin.
        if (packet.getReadPos() <= startPos) break;
    }

    LOG_INFO("Parsed ", inbox.size(), " mail messages");
    return true;
}

// ============================================================
// Bank System
// ============================================================

network::Packet BankerActivatePacket::build(uint64_t guid) {
    network::Packet p(wireOpcode(Opcode::CMSG_BANKER_ACTIVATE));
    p.writeUInt64(guid);
    return p;
}

network::Packet BuyBankSlotPacket::build(uint64_t guid) {
    network::Packet p(wireOpcode(Opcode::CMSG_BUY_BANK_SLOT));
    p.writeUInt64(guid);
    return p;
}

network::Packet AutoBankItemPacket::build(uint8_t srcBag, uint8_t srcSlot) {
    network::Packet p(wireOpcode(Opcode::CMSG_AUTOBANK_ITEM));
    p.writeUInt8(srcBag);
    p.writeUInt8(srcSlot);
    return p;
}

network::Packet AutoStoreBankItemPacket::build(uint8_t srcBag, uint8_t srcSlot) {
    network::Packet p(wireOpcode(Opcode::CMSG_AUTOSTORE_BANK_ITEM));
    p.writeUInt8(srcBag);
    p.writeUInt8(srcSlot);
    return p;
}

// ============================================================
// Guild Bank System
// ============================================================

network::Packet GuildBankerActivatePacket::build(uint64_t guid) {
    network::Packet p(wireOpcode(Opcode::CMSG_GUILD_BANKER_ACTIVATE));
    p.writeUInt64(guid);
    p.writeUInt8(0);  // full slots update
    return p;
}

network::Packet GuildBankQueryTabPacket::build(uint64_t guid, uint8_t tabId, bool fullUpdate) {
    network::Packet p(wireOpcode(Opcode::CMSG_GUILD_BANK_QUERY_TAB));
    p.writeUInt64(guid);
    p.writeUInt8(tabId);
    p.writeUInt8(fullUpdate ? 1 : 0);
    return p;
}

network::Packet GuildBankBuyTabPacket::build(uint64_t guid, uint8_t tabId) {
    network::Packet p(wireOpcode(Opcode::CMSG_GUILD_BANK_BUY_TAB));
    p.writeUInt64(guid);
    p.writeUInt8(tabId);
    return p;
}

network::Packet GuildBankDepositMoneyPacket::build(uint64_t guid, uint32_t amount) {
    network::Packet p(wireOpcode(Opcode::CMSG_GUILD_BANK_DEPOSIT_MONEY));
    p.writeUInt64(guid);
    p.writeUInt32(amount);
    return p;
}

network::Packet GuildBankWithdrawMoneyPacket::build(uint64_t guid, uint32_t amount) {
    network::Packet p(wireOpcode(Opcode::CMSG_GUILD_BANK_WITHDRAW_MONEY));
    p.writeUInt64(guid);
    p.writeUInt32(amount);
    return p;
}

// CMSG_GUILD_BANK_SWAP_ITEMS (3.3.5a). The first byte selects the code path:
//   bankToBank != 0 → move within the guild bank (two tabs); NOT used here.
//   bankToBank == 0 → transfer between the guild bank and player inventory.
// For the inventory path the server reads:
//   u8 tabId, u8 slotId, u32 itemEntry, u8 autoStore
//   if autoStore: u32 count, u8 (toChar, forced 1), u32 (0)   → bank→char auto
//   else:         u8 playerBag, u8 playerSlot, u8 toChar, u32 splitedAmount
// toChar: 1 = bank→character (withdraw), 0 = character→bank (deposit).
// The old builder omitted toChar, wrote splitCount as a mid-packet u8, and the
// deposit variant set bankToBank=1 — so item transfers were silently dropped.
network::Packet GuildBankSwapItemsPacket::buildBankToInventory(
    uint64_t guid, uint8_t tabId, uint8_t bankSlot,
    uint8_t destBag, uint8_t destSlot, uint32_t splitCount)
{
    network::Packet p(wireOpcode(Opcode::CMSG_GUILD_BANK_SWAP_ITEMS));
    p.writeUInt64(guid);
    p.writeUInt8(0);          // bankToBank = 0 (involves player inventory)
    p.writeUInt8(tabId);
    p.writeUInt8(bankSlot);
    p.writeUInt32(0);         // itemEntry (server ignores)
    if (destBag == 0xFF) {
        // Let the server drop the item into the first free inventory slot.
        p.writeUInt8(1);      // autoStore = 1
        p.writeUInt32(splitCount); // count (0 = whole stack)
        p.writeUInt8(1);      // toChar = 1 (bank → character)
        p.writeUInt32(0);     // trailing, always 0
    } else {
        p.writeUInt8(0);      // autoStore = 0
        p.writeUInt8(destBag);   // playerBag
        p.writeUInt8(destSlot);  // playerSlot
        p.writeUInt8(1);      // toChar = 1 (withdraw)
        p.writeUInt32(splitCount); // splitedAmount
    }
    return p;
}

network::Packet GuildBankSwapItemsPacket::buildInventoryToBank(
    uint64_t guid, uint8_t tabId, uint8_t bankSlot,
    uint8_t srcBag, uint8_t srcSlot, uint32_t splitCount)
{
    network::Packet p(wireOpcode(Opcode::CMSG_GUILD_BANK_SWAP_ITEMS));
    p.writeUInt64(guid);
    p.writeUInt8(0);          // bankToBank = 0 (involves player inventory)
    p.writeUInt8(tabId);
    p.writeUInt8(bankSlot);   // destination slot within the tab
    p.writeUInt32(0);         // itemEntry
    p.writeUInt8(0);          // autoStore = 0
    p.writeUInt8(srcBag);     // playerBag
    p.writeUInt8(srcSlot);    // playerSlot
    p.writeUInt8(0);          // toChar = 0 (character → bank = deposit)
    p.writeUInt32(splitCount); // splitedAmount
    return p;
}

bool GuildBankListParser::parse(network::Packet& packet, GuildBankData& data) {
    if (!packet.hasRemaining(14)) return false;

    data.money = packet.readUInt64();
    data.tabId = packet.readUInt8();
    data.withdrawAmount = static_cast<int32_t>(packet.readUInt32());
    uint8_t fullUpdate = packet.readUInt8();

    if (fullUpdate) {
        if (!packet.hasRemaining(1)) {
            LOG_WARNING("GuildBankListParser: truncated before tabCount");
            data.tabs.clear();
        } else {
            uint8_t tabCount = packet.readUInt8();
            // Cap at 8 (normal guild bank tab limit in WoW)
            if (tabCount > 8) {
                LOG_WARNING("GuildBankListParser: tabCount capped (requested=", static_cast<int>(tabCount), ")");
                tabCount = 8;
            }
            data.tabs.resize(tabCount);
            for (uint8_t i = 0; i < tabCount; ++i) {
                // Validate before reading strings
                if (!packet.hasData()) {
                    LOG_WARNING("GuildBankListParser: truncated tab at index ", static_cast<int>(i));
                    break;
                }
                data.tabs[i].tabName = packet.readString();
                if (!packet.hasData()) {
                    data.tabs[i].tabIcon.clear();
                } else {
                    data.tabs[i].tabIcon = packet.readString();
                }
            }
        }
    }

    if (!packet.hasRemaining(1)) {
        LOG_WARNING("GuildBankListParser: truncated before numSlots");
        data.tabItems.clear();
        return true;
    }

    uint8_t numSlots = packet.readUInt8();
    data.tabItems.clear();
    for (uint8_t i = 0; i < numSlots; ++i) {
        // Validate minimum bytes before reading slot (slotId(1) + itemEntry(4) = 5)
        if (!packet.hasRemaining(5)) {
            LOG_WARNING("GuildBankListParser: truncated slot at index ", static_cast<int>(i));
            break;
        }
        GuildBankItemSlot slot;
        slot.slotId = packet.readUInt8();
        slot.itemEntry = packet.readUInt32();
        if (slot.itemEntry != 0) {
            // Validate before reading enchant mask
            if (!packet.hasRemaining(4)) break;
            // Enchant info
            uint32_t enchantMask = packet.readUInt32();
            for (int bit = 0; bit < 10; ++bit) {
                if (enchantMask & (1u << bit)) {
                    if (!packet.hasRemaining(12)) {
                        LOG_WARNING("GuildBankListParser: truncated enchant data");
                        break;
                    }
                    uint32_t enchId = packet.readUInt32();
                    uint32_t enchDur = packet.readUInt32();
                    uint32_t enchCharges = packet.readUInt32();
                    if (bit == 0) slot.enchantId = enchId;
                    (void)enchDur; (void)enchCharges;
                }
            }
            // Validate before reading remaining item fields
            if (!packet.hasRemaining(12)) {
                LOG_WARNING("GuildBankListParser: truncated item fields");
                break;
            }
            slot.stackCount = packet.readUInt32();
            /*spare=*/ packet.readUInt32();
            slot.randomPropertyId = packet.readUInt32();
            if (slot.randomPropertyId) {
                if (!packet.hasRemaining(4)) {
                    LOG_WARNING("GuildBankListParser: truncated suffix factor");
                    break;
                }
                /*suffixFactor=*/ packet.readUInt32();
            }
        }
        data.tabItems.push_back(slot);
    }
    return true;
}

// ============================================================
// Auction House System
// ============================================================

network::Packet AuctionHelloPacket::build(uint64_t guid) {
    network::Packet p(wireOpcode(Opcode::MSG_AUCTION_HELLO));
    p.writeUInt64(guid);
    return p;
}

bool AuctionHelloParser::parse(network::Packet& packet, AuctionHelloData& data) {
    size_t remaining = packet.getRemainingSize();
    if (remaining < 12) {
        LOG_WARNING("AuctionHelloParser: too small, remaining=", remaining);
        return false;
    }
    data.auctioneerGuid = packet.readUInt64();
    data.auctionHouseId = packet.readUInt32();
    // WotLK has an extra uint8 enabled field; Vanilla does not
    if (packet.hasData()) {
        data.enabled = packet.readUInt8();
    } else {
        data.enabled = 1;
    }
    return true;
}

network::Packet AuctionListItemsPacket::build(
    uint64_t guid, uint32_t offset,
    const std::string& searchName,
    uint8_t levelMin, uint8_t levelMax,
    uint32_t invTypeMask, uint32_t itemClass,
    uint32_t itemSubClass, uint32_t quality,
    uint8_t usableOnly, uint8_t exactMatch)
{
    network::Packet p(wireOpcode(Opcode::CMSG_AUCTION_LIST_ITEMS));
    p.writeUInt64(guid);
    p.writeUInt32(offset);
    p.writeString(searchName);
    p.writeUInt8(levelMin);
    p.writeUInt8(levelMax);
    p.writeUInt32(invTypeMask);
    p.writeUInt32(itemClass);
    p.writeUInt32(itemSubClass);
    p.writeUInt32(quality);
    p.writeUInt8(usableOnly);
    p.writeUInt8(0);  // getAll (0 = normal search)
    // WotLK has no exact-match field here; the next byte is the sort count.
    // Keep the API argument for callers shared with older server profiles.
    (void)exactMatch;
    p.writeUInt8(0);
    return p;
}

network::Packet AuctionSellItemPacket::build(
    uint64_t auctioneerGuid, uint64_t itemGuid,
    uint32_t stackCount, uint32_t bid,
    uint32_t buyout, uint32_t duration,
    bool preWotlk)
{
    network::Packet p(wireOpcode(Opcode::CMSG_AUCTION_SELL_ITEM));
    p.writeUInt64(auctioneerGuid);
    if (!preWotlk) {
        // WotLK: itemCount(4) + per-item [guid(8) + stackCount(4)]
        p.writeUInt32(1);
        p.writeUInt64(itemGuid);
        p.writeUInt32(stackCount);
    } else {
        // Classic/TBC: just itemGuid, no count fields
        p.writeUInt64(itemGuid);
    }
    p.writeUInt32(bid);
    p.writeUInt32(buyout);
    p.writeUInt32(duration);
    return p;
}

network::Packet AuctionPlaceBidPacket::build(uint64_t auctioneerGuid, uint32_t auctionId, uint32_t amount) {
    network::Packet p(wireOpcode(Opcode::CMSG_AUCTION_PLACE_BID));
    p.writeUInt64(auctioneerGuid);
    p.writeUInt32(auctionId);
    p.writeUInt32(amount);
    return p;
}

network::Packet AuctionRemoveItemPacket::build(uint64_t auctioneerGuid, uint32_t auctionId) {
    network::Packet p(wireOpcode(Opcode::CMSG_AUCTION_REMOVE_ITEM));
    p.writeUInt64(auctioneerGuid);
    p.writeUInt32(auctionId);
    return p;
}

network::Packet AuctionListOwnerItemsPacket::build(uint64_t auctioneerGuid, uint32_t offset) {
    network::Packet p(wireOpcode(Opcode::CMSG_AUCTION_LIST_OWNER_ITEMS));
    p.writeUInt64(auctioneerGuid);
    p.writeUInt32(offset);
    return p;
}

network::Packet AuctionListBidderItemsPacket::build(
    uint64_t auctioneerGuid, uint32_t offset,
    const std::vector<uint32_t>& outbiddedIds)
{
    network::Packet p(wireOpcode(Opcode::CMSG_AUCTION_LIST_BIDDER_ITEMS));
    p.writeUInt64(auctioneerGuid);
    p.writeUInt32(offset);
    p.writeUInt32(static_cast<uint32_t>(outbiddedIds.size()));
    for (uint32_t id : outbiddedIds)
        p.writeUInt32(id);
    return p;
}

bool AuctionListResultParser::parse(network::Packet& packet, AuctionListResult& data, int numEnchantSlots) {
    // Entry layout, verified against the servers' BuildAuctionInfo
    // (vmangos / cmangos-tbc / AzerothCore):
    //   auctionId(4) + itemEntry(4)
    //   Vanilla (numEnchantSlots=1): enchantId(4) only — no duration/charges
    //   TBC/WotLK: numEnchantSlots × [id(4)+duration(4)+charges(4)]
    //     (TBC inspects 6 slots; WotLK 7 — PRISMATIC was added in 3.x)
    //   randProp(4) + suffix(4) + stack(4) + spellCharges(4) +
    //   flags(4, TBC/WotLK only) + ownerGuid(8) + startBid(4) + outbid(4) +
    //   buyout(4) + expire(4) + bidderGuid(8) + curBid(4)
    // → 64 bytes/entry vanilla, 136 TBC, 148 WotLK
    const bool vanillaLayout = numEnchantSlots <= 1;
    if (!packet.hasRemaining(4)) return false;

    uint32_t count = packet.readUInt32();
    // Cap auction count to prevent unbounded memory allocation
    const uint32_t MAX_AUCTION_RESULTS = 256;
    if (count > MAX_AUCTION_RESULTS) {
        LOG_WARNING("AuctionListResultParser: count capped (requested=", count, ")");
        count = MAX_AUCTION_RESULTS;
    }

    data.auctions.clear();
    data.auctions.reserve(count);

    const size_t minPerEntry = vanillaLayout
        ? 64
        : static_cast<size_t>(8 + numEnchantSlots * 12 + 20 + 8 + 16 + 8 + 4);
    for (uint32_t i = 0; i < count; ++i) {
        if (!packet.hasRemaining(minPerEntry)) break;
        AuctionEntry e;
        e.auctionId = packet.readUInt32();
        e.itemEntry = packet.readUInt32();
        // Vanilla sends only the permanent enchant id; TBC/WotLK send
        // id/duration/charges triplets for every inspected slot.
        e.enchantId = packet.readUInt32();
        if (!vanillaLayout) {
            packet.readUInt32(); // enchant1 duration
            packet.readUInt32(); // enchant1 charges
            for (int s = 1; s < numEnchantSlots; ++s) {
                packet.readUInt32(); // enchant N id
                packet.readUInt32(); // enchant N duration
                packet.readUInt32(); // enchant N charges
            }
        }
        e.randomPropertyId = packet.readUInt32();
        e.suffixFactor     = packet.readUInt32();
        e.stackCount       = packet.readUInt32();
        packet.readUInt32(); // item spell charges
        if (!vanillaLayout) packet.readUInt32(); // item flags (server always writes 0)
        e.ownerGuid        = packet.readUInt64();
        e.startBid         = packet.readUInt32();
        e.minBidIncrement  = packet.readUInt32();
        e.buyoutPrice      = packet.readUInt32();
        e.timeLeftMs       = packet.readUInt32();
        e.bidderGuid       = packet.readUInt64();
        e.currentBid       = packet.readUInt32();
        data.auctions.push_back(e);
    }

    // Trailer: totalCount everywhere; TBC/WotLK append a search-delay field
    // (vanilla ends after totalCount, so read the two independently).
    if (packet.hasRemaining(4)) data.totalCount = packet.readUInt32();
    if (packet.hasRemaining(4)) data.searchDelay = packet.readUInt32();
    return true;
}

bool AuctionCommandResultParser::parse(network::Packet& packet, AuctionCommandResult& data) {
    if (!packet.hasRemaining(12)) return false;
    data.auctionId = packet.readUInt32();
    data.action = packet.readUInt32();
    data.errorCode = packet.readUInt32();
    if (data.errorCode != 0 && data.action == 2 && packet.hasRemaining(4)) {
        data.bidError = packet.readUInt32();
    }
    return true;
}

// ============================================================
// Pet Stable System
// ============================================================

network::Packet ListStabledPetsPacket::build(uint64_t stableMasterGuid) {
    network::Packet p(wireOpcode(Opcode::MSG_LIST_STABLED_PETS));
    p.writeUInt64(stableMasterGuid);
    return p;
}

network::Packet StablePetPacket::build(uint64_t stableMasterGuid, uint8_t slot) {
    network::Packet p(wireOpcode(Opcode::CMSG_STABLE_PET));
    p.writeUInt64(stableMasterGuid);
    p.writeUInt8(slot);
    return p;
}

network::Packet UnstablePetPacket::build(uint64_t stableMasterGuid, uint32_t petNumber) {
    network::Packet p(wireOpcode(Opcode::CMSG_UNSTABLE_PET));
    p.writeUInt64(stableMasterGuid);
    p.writeUInt32(petNumber);
    return p;
}

network::Packet PetRenamePacket::build(uint64_t petGuid, const std::string& name, uint8_t isDeclined) {
    network::Packet p(wireOpcode(Opcode::CMSG_PET_RENAME));
    p.writeUInt64(petGuid);
    p.writeString(name);    // null-terminated
    p.writeUInt8(isDeclined);
    return p;
}

network::Packet SetTitlePacket::build(int32_t titleBit) {
    // CMSG_SET_TITLE: int32 titleBit (-1 = remove active title)
    network::Packet p(wireOpcode(Opcode::CMSG_SET_TITLE));
    p.writeUInt32(static_cast<uint32_t>(titleBit));
    return p;
}

network::Packet AlterAppearancePacket::build(uint32_t hairStyleEntry, uint32_t hairColor,
                                              uint32_t facialHairEntry, uint32_t skinColorEntry) {
    // WotLK: BarberShopStyle hair entry + raw color + BarberShopStyle facial
    // entry + optional BarberShopStyle skin entry (zero means unchanged).
    network::Packet p(wireOpcode(Opcode::CMSG_ALTER_APPEARANCE));
    p.writeUInt32(hairStyleEntry);
    p.writeUInt32(hairColor);
    p.writeUInt32(facialHairEntry);
    p.writeUInt32(skinColorEntry);
    return p;
}

} // namespace game
} // namespace wowee
