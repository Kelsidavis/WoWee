#pragma once

#include <cstddef>
#include <cstdint>

namespace wowee {
namespace core {

/**
 * Monitors system memory and provides dynamic cache sizing
 */
class MemoryMonitor {
public:
    static MemoryMonitor& getInstance();

    /**
     * Initialize memory monitoring
     */
    void initialize();

    /**
     * Get total system RAM in bytes
     */
    [[nodiscard]] size_t getTotalRAM() const { return totalRAM_; }

    /**
     * Get currently available RAM in bytes
     */
    [[nodiscard]] size_t getAvailableRAM() const;

    /**
     * Get recommended cache budget (80% of available RAM, capped at 90% of total RAM)
     */
    [[nodiscard]] size_t getRecommendedCacheBudget() const;

    /**
     * Check if system is under memory pressure (< 10% RAM available)
     */
    [[nodiscard]] bool isMemoryPressure() const;

    /**
     * Check if system is under severe memory pressure (< 15% RAM available).
     * At this level, background loading should pause entirely until memory
     * is freed - continuing to allocate risks OOM-killing other applications.
     */
    [[nodiscard]] bool isSevereMemoryPressure() const;

private:
    MemoryMonitor() = default;
    size_t totalRAM_ = 0;
};

} // namespace core
} // namespace wowee
