#pragma once

#include <cstdint>
#include <vector>
#include <utility>
#include <algorithm>

namespace wowee {
namespace game {

/**
 * Sorted flat-vector field map shared by UpdateBlock and Entity.
 *
 * Replaces std::map<uint16_t, uint32_t> for both the per-packet UpdateBlock
 * fields (built monotonically by the parser) and the persistent Entity
 * fields (mutated by setField). For typical 60–200 entries the flat layout
 * is dramatically friendlier to the allocator and the cache than a
 * red-black tree, while preserving the same iterator-based API the
 * existing call sites already use.
 *
 * Iterator dereferences return std::pair<uint16_t, uint32_t> so existing
 * code using `it->first` / `it->second` and range-based for over
 * `[key, val]` continues to compile unchanged.
 */
class FlatFieldMap {
public:
    using value_type     = std::pair<uint16_t, uint32_t>;
    using const_iterator = std::vector<value_type>::const_iterator;
    using iterator       = std::vector<value_type>::iterator;

    [[nodiscard]] bool   empty() const noexcept { return data_.empty(); }
    [[nodiscard]] size_t size()  const noexcept { return data_.size(); }
    void   clear()                { data_.clear(); }
    void   reserve(size_t n)      { data_.reserve(n); }

    iterator       begin()        { return data_.begin(); }
    iterator       end()          { return data_.end(); }
    [[nodiscard]] const_iterator begin()  const { return data_.begin(); }
    [[nodiscard]] const_iterator end()    const { return data_.end(); }
    [[nodiscard]] const_iterator cbegin() const { return data_.cbegin(); }
    [[nodiscard]] const_iterator cend()   const { return data_.cend(); }

    // Append at end. Caller must ensure key is strictly greater than the
    // previously appended key - used by the UPDATE_OBJECT parser which
    // walks the bitmask low-to-high.
    void append_sorted(uint16_t key, uint32_t value) {
        data_.emplace_back(key, value);
    }

    // Insert in sorted position, or overwrite the existing value.
    void insert_or_assign(uint16_t key, uint32_t value) {
        auto it = std::lower_bound(
            data_.begin(), data_.end(), key,
            [](const value_type& p, uint16_t k) { return p.first < k; });
        if (it != data_.end() && it->first == key) {
            it->second = value;
        } else {
            data_.insert(it, value_type(key, value));
        }
    }

    // Map-style operator[] - used by Entity::setField. Returns a reference
    // to the (existing or newly inserted) value.
    uint32_t& operator[](uint16_t key) {
        auto it = std::lower_bound(
            data_.begin(), data_.end(), key,
            [](const value_type& p, uint16_t k) { return p.first < k; });
        if (it != data_.end() && it->first == key) return it->second;
        return data_.insert(it, value_type(key, 0u))->second;
    }

    [[nodiscard]] const value_type& back() const { return data_.back(); }
    value_type&       back()       { return data_.back(); }

    [[nodiscard]] const_iterator find(uint16_t key) const {
        auto it = std::lower_bound(
            data_.begin(), data_.end(), key,
            [](const value_type& p, uint16_t k) { return p.first < k; });
        if (it != data_.end() && it->first == key) return it;
        return data_.end();
    }
    iterator find(uint16_t key) {
        auto it = std::lower_bound(
            data_.begin(), data_.end(), key,
            [](const value_type& p, uint16_t k) { return p.first < k; });
        if (it != data_.end() && it->first == key) return it;
        return data_.end();
    }

private:
    std::vector<value_type> data_;
};

} // namespace game
} // namespace wowee
