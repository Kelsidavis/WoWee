// CMSG_ACTIVATETAXIEXPRESS — the multi-hop flight request.
//
// A total cost was being written between the guid and the node count, and
// HandleActivateTaxiExpressOpcode does not read one. So the server took the
// cost as the number of nodes to expect and read the client's own count as the
// first node — a small number that is no taxi node anyone has visited, which it
// answers with ERR_TAXINOTVISITED. A multi-hop flight was refused for a flight
// point the player had certainly been to, and with a large enough cost the
// server ran off the end of the buffer and dropped the packet instead.
//
// Single hops go through CMSG_ACTIVATETAXI and were unaffected, which is why
// only longer routes ever failed. These pin the shape against what the server
// reads.
#include <catch_amalgamated.hpp>
#include "game/world_packets.hpp"
#include "game/packet_parsers.hpp"
#include "core/application.hpp"

// The builders under test never reach the singleton; a null instance satisfies
// the linker for the translation units that inline isActiveExpansion().
namespace wowee {
namespace core {
Application* Application::instance = nullptr;
}
}

using namespace wowee::game;
using Bytes = std::vector<uint8_t>;

namespace {

uint32_t readU32(const Bytes& b, size_t at) {
    return static_cast<uint32_t>(b[at]) |
           (static_cast<uint32_t>(b[at + 1]) << 8) |
           (static_cast<uint32_t>(b[at + 2]) << 16) |
           (static_cast<uint32_t>(b[at + 3]) << 24);
}

uint64_t readU64(const Bytes& b, size_t at) {
    uint64_t v = 0;
    for (int i = 7; i >= 0; --i) v = (v << 8) | b[at + static_cast<size_t>(i)];
    return v;
}

} // namespace

TEST_CASE("ACTIVATETAXIEXPRESS is guid, count, then the nodes and nothing else",
          "[taxi][packet]") {
    constexpr uint64_t kNpc = 0x00000000000ABCDEull;
    const std::vector<uint32_t> route = {11, 26, 5};

    auto packet = ActivateTaxiExpressPacket::build(kNpc, route);
    const Bytes& bytes = packet.getData();

    // guid(8) + count(4) + one uint32 per node. Anything else in here is the
    // field the server does not read.
    REQUIRE(bytes.size() == 8 + 4 + route.size() * 4);
    REQUIRE(readU64(bytes, 0) == kNpc);
    REQUIRE(readU32(bytes, 8) == route.size());
    for (size_t i = 0; i < route.size(); ++i) {
        INFO("node " << i);
        REQUIRE(readU32(bytes, 12 + i * 4) == route[i]);
    }
}

TEST_CASE("The count is the node count, not a cost", "[taxi][packet]") {
    // The distinguishing test: with a cost in front, the value at offset 8
    // would be money rather than a small node count, and the first node would
    // land where the count belongs. A route of two nodes says 2 here.
    auto two = ActivateTaxiExpressPacket::build(1, {100, 200});
    REQUIRE(readU32(two.getData(), 8) == 2u);
    REQUIRE(readU32(two.getData(), 12) == 100u);

    auto five = ActivateTaxiExpressPacket::build(1, {1, 2, 3, 4, 5});
    REQUIRE(readU32(five.getData(), 8) == 5u);
    REQUIRE(five.getData().size() == 8 + 4 + 5 * 4);
}

TEST_CASE("A single-hop route still takes the express shape", "[taxi][packet]") {
    // Reachable: a one-hop route is built here whenever the express path is the
    // one chosen, and a count of one must not be mistaken for a node.
    auto one = ActivateTaxiExpressPacket::build(0x1122334455667788ull, {42});
    const Bytes& bytes = one.getData();
    REQUIRE(bytes.size() == 8 + 4 + 4);
    REQUIRE(readU64(bytes, 0) == 0x1122334455667788ull);
    REQUIRE(readU32(bytes, 8) == 1u);
    REQUIRE(readU32(bytes, 12) == 42u);
}

TEST_CASE("The single-hop request keeps its own three fields", "[taxi][packet]") {
    // CMSG_ACTIVATETAXI is guid, source, destination — a different message that
    // was never wrong, pinned so the two do not drift into each other.
    auto packet = ActivateTaxiPacket::build(0xDEADBEEFull, 7, 9);
    const Bytes& bytes = packet.getData();
    REQUIRE(bytes.size() == 8 + 4 + 4);
    REQUIRE(readU64(bytes, 0) == 0xDEADBEEFull);
    REQUIRE(readU32(bytes, 8) == 7u);
    REQUIRE(readU32(bytes, 12) == 9u);
}
