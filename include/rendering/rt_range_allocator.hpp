#pragma once

/// First-fit allocator over a range of element slots, for the ray tracing
/// scene's GPU pools.
///
/// Terrain streams in and out a tile at a time, so the triangle and node pools
/// see a steady churn of mesh-sized allocations. Freed ranges are coalesced
/// with their neighbours; the pool only grows when no free range fits, and the
/// caller then copies the old buffer into a larger one - offsets handed out
/// before the growth stay valid.

#include <cstdint>
#include <iterator>
#include <map>

namespace wowee::rendering {

class RtRangeAllocator {
public:
    static constexpr uint32_t kFailed = ~0u;

    explicit RtRangeAllocator(uint32_t capacity = 0) { grow(capacity); }

    /// Offset of `count` contiguous slots, or kFailed when nothing fits.
    uint32_t allocate(uint32_t count) {
        if (count == 0) return kFailed;
        for (auto it = free_.begin(); it != free_.end(); ++it) {
            if (it->second < count) continue;
            const uint32_t offset = it->first;
            const uint32_t rest = it->second - count;
            free_.erase(it);
            if (rest > 0) free_.emplace(offset + count, rest);
            used_ += count;
            return offset;
        }
        return kFailed;
    }

    void release(uint32_t offset, uint32_t count) {
        if (count == 0 || offset == kFailed) return;
        used_ -= count;
        auto next = free_.lower_bound(offset);
        if (next != free_.end() && offset + count == next->first) {
            count += next->second;
            next = free_.erase(next);
        }
        if (next != free_.begin()) {
            auto prev = std::prev(next);
            if (prev->first + prev->second == offset) {
                prev->second += count;
                return;
            }
        }
        free_.emplace(offset, count);
    }

    /// Extend the range to `newCapacity` slots. The new tail is free and joins
    /// a free range that ends at the old capacity.
    void grow(uint32_t newCapacity) {
        if (newCapacity <= capacity_) return;
        const uint32_t oldCapacity = capacity_;
        capacity_ = newCapacity;
        used_ += newCapacity - oldCapacity;  // release() subtracts it again
        release(oldCapacity, newCapacity - oldCapacity);
    }

    [[nodiscard]] uint32_t capacity() const { return capacity_; }
    [[nodiscard]] uint32_t used() const { return used_; }

private:
    std::map<uint32_t, uint32_t> free_;  // offset -> count
    uint32_t capacity_ = 0;
    uint32_t used_ = 0;
};

}  // namespace wowee::rendering
