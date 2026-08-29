#pragma once

#include <cstdint>

namespace wowee::game {

/// How many of a stack a vendor sell should ask for.
///
/// One, which is what all four sell sites hardcoded, sells a single unit of a
/// twenty-stack and leaves nineteen behind - twice over, because the auto-sell
/// sweep also charged one unit's price into the total it reports and never
/// revisited the slot. Right-clicking an item at a vendor in 3.3.5 sells the
/// whole stack, CMSG_SELL_ITEM carries the count, and the server clamps it to
/// what is actually there.
///
/// A slot that has not said how many it holds still holds one.
[[nodiscard]] inline uint32_t sellCountForStack(uint32_t stackCount) {
    return stackCount > 0 ? stackCount : 1u;
}

}  // namespace wowee::game
