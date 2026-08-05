// SMSG_EQUIPMENT_SET_LIST and the three messages that answer it.
//
// Every guid in this family is packed — a mask byte and then only the non-zero
// bytes — and all of them were read and written as fixed eight-byte values. A
// set guid of one is two bytes on the wire, so the incoming read ate the set id
// and the front of the name, and with more than one set the loop lost its place
// entirely. The outgoing save was worse: a fixed eight bytes where the server
// calls readPackGUID leaves seven behind, so it took the index out of the
// padding and the name out of what was left of it.
//
// There was also a field on the incoming side that the server does not send. An
// ignore mask was being read as a uint32 between the icon name and the slots;
// SendEquipmentSetList writes no such word. An ignored slot is written as an
// item guid of literal one, and that *is* the mask.
//
// These build the packets the way AzerothCore builds and reads them.

#include <catch_amalgamated.hpp>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "network/packet.hpp"

using wowee::network::Packet;

namespace {

/// One set as Player::SendEquipmentSetList writes it: packed set guid, index,
/// name, icon name, then nineteen packed item guids with one meaning "ignore".
void appendSet(Packet& p, uint64_t setGuid, uint32_t index,
               const std::string& name, const std::string& icon,
               const std::array<uint64_t, 19>& items) {
    p.writePackedGuid(setGuid);
    p.writeUInt32(index);
    p.writeString(name);
    p.writeString(icon);
    for (uint64_t g : items) p.writePackedGuid(g);
}

struct ParsedSet {
    uint64_t setGuid = 0;
    uint32_t setId = 0;
    std::string name;
    std::string iconName;
    uint32_t ignoreSlotMask = 0;
    std::array<uint64_t, 19> itemGuids{};
};

/// The reader under test, in the shape handleEquipmentSetList uses.
std::vector<ParsedSet> readList(Packet& p) {
    std::vector<ParsedSet> out;
    if (!p.hasRemaining(4)) return out;
    const uint32_t count = p.readUInt32();
    for (uint32_t i = 0; i < count; ++i) {
        if (!p.hasRemaining(2)) break;
        ParsedSet es;
        es.setGuid = p.readPackedGuid();
        if (!p.hasRemaining(4)) break;
        es.setId = p.readUInt32();
        es.name = p.readString();
        es.iconName = p.readString();
        for (int slot = 0; slot < 19; ++slot) {
            if (!p.hasRemaining(1)) break;
            const uint64_t itemGuid = p.readPackedGuid();
            if (itemGuid == 1) {
                es.ignoreSlotMask |= (1u << slot);
                es.itemGuids[slot] = 0;
            } else {
                es.itemGuids[slot] = itemGuid;
            }
        }
        out.push_back(std::move(es));
    }
    return out;
}

Packet forReading(Packet& built) {
    return Packet(built.getOpcode(), built.getData());
}

std::array<uint64_t, 19> someItems() {
    std::array<uint64_t, 19> items{};
    items[0] = 0x40000000000001ULL;   // head
    items[4] = 1;                     // chest: ignore this slot
    items[15] = 0x40000000000002ULL;  // main hand
    return items;
}

}  // namespace

TEST_CASE("one equipment set reads back what was written", "[equipment_sets]") {
    Packet built(0);
    built.writeUInt32(1);
    appendSet(built, 1, 0, "Tanking", "Interface\\Icons\\INV_Shield_06", someItems());
    Packet p = forReading(built);

    auto sets = readList(p);
    REQUIRE(sets.size() == 1u);
    CHECK(sets[0].setGuid == 1u);
    CHECK(sets[0].setId == 0u);
    CHECK(sets[0].name == "Tanking");
    CHECK(sets[0].iconName == "Interface\\Icons\\INV_Shield_06");
    CHECK(sets[0].itemGuids[0] == 0x40000000000001ULL);
    CHECK(sets[0].itemGuids[15] == 0x40000000000002ULL);
    CHECK(p.getRemainingSize() == 0u);
}

TEST_CASE("an item guid of one means ignore the slot", "[equipment_sets]") {
    // The mask is not a field. It is this value, in the slot it applies to.
    Packet built(0);
    built.writeUInt32(1);
    appendSet(built, 7, 2, "Healing", "icon", someItems());
    Packet p = forReading(built);

    auto sets = readList(p);
    REQUIRE(sets.size() == 1u);
    CHECK((sets[0].ignoreSlotMask & (1u << 4)) != 0u);
    CHECK(sets[0].itemGuids[4] == 0u);
    CHECK((sets[0].ignoreSlotMask & (1u << 0)) == 0u);
}

TEST_CASE("three sets do not run into each other", "[equipment_sets]") {
    // This is what the fixed read really cost. One misparsed set is one wrong
    // name; a list of them means the second set is read out of the middle of
    // the first, and everything after is noise.
    Packet built(0);
    built.writeUInt32(3);
    appendSet(built, 1, 0, "Tanking", "iconA", someItems());
    appendSet(built, 0x1234, 1, "Healing", "iconB", someItems());
    appendSet(built, 0xFFFFFFFFFFULL, 2, "Damage", "iconC", someItems());
    Packet p = forReading(built);

    auto sets = readList(p);
    REQUIRE(sets.size() == 3u);
    CHECK(sets[0].name == "Tanking");
    CHECK(sets[1].name == "Healing");
    CHECK(sets[1].setGuid == 0x1234u);
    CHECK(sets[2].name == "Damage");
    CHECK(sets[2].setGuid == 0xFFFFFFFFFFULL);
    CHECK(sets[2].setId == 2u);
    CHECK(p.getRemainingSize() == 0u);
}

TEST_CASE("a saved set is laid out the way the server reads it",
          "[equipment_sets]") {
    // HandleEquipmentSetSave: readPackGUID, index, name, icon, nineteen packed
    // item guids. Read back here in exactly that order — a fixed eight bytes
    // for the guid leaves seven behind and every field after it lands wrong.
    Packet pkt(0);
    pkt.writePackedGuid(0);            // a new set has no guid yet
    pkt.writeUInt32(3);
    pkt.writeString("Farming");
    pkt.writeString("iconD");
    for (uint64_t g : someItems()) pkt.writePackedGuid(g);

    Packet p = forReading(pkt);
    CHECK(p.readPackedGuid() == 0u);
    CHECK(p.readUInt32() == 3u);
    CHECK(p.readString() == "Farming");
    CHECK(p.readString() == "iconD");
    CHECK(p.readPackedGuid() == 0x40000000000001ULL);
    for (int i = 1; i < 4; ++i) CHECK(p.readPackedGuid() == 0u);
    CHECK(p.readPackedGuid() == 1u);   // the ignored chest slot
    CHECK(p.getRemainingSize() > 0u);  // fourteen slots still to come
}

TEST_CASE("a new set's guid is one byte, not eight", "[equipment_sets]") {
    // The case that made every save malformed: zero packs to a single mask
    // byte. Eight zero bytes would leave the server reading the index out of
    // padding it should never have seen.
    Packet pkt(0);
    pkt.writePackedGuid(0);
    CHECK(pkt.getData().size() == 1u);
}
