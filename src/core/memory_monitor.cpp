#include "core/memory_monitor.hpp"
#include "core/logger.hpp"
#include <fstream>
#include <string>
#include <sstream>
#ifdef _WIN32
#include <windows.h>
#else
#include <sys/sysinfo.h>
#endif

namespace wowee {
namespace core {

#ifndef _WIN32
namespace {
size_t readMemAvailableBytesFromProc() {
    std::ifstream meminfo("/proc/meminfo");
    if (!meminfo.is_open()) return 0;

    std::string line;
    while (std::getline(meminfo, line)) {
        // Format: "MemAvailable:  123456789 kB"
        if (line.rfind("MemAvailable:", 0) != 0) continue;
        std::istringstream iss(line.substr(13));
        size_t kb = 0;
        iss >> kb;
        if (kb > 0) return kb * 1024ull;
        break;
    }
    return 0;
}
} // namespace
#endif

MemoryMonitor& MemoryMonitor::getInstance() {
    static MemoryMonitor instance;
    return instance;
}

void MemoryMonitor::initialize() {
#ifdef _WIN32
    ULONGLONG totalKB = 0;
    if (GetPhysicallyInstalledSystemMemory(&totalKB)) {
        totalRAM_ = static_cast<size_t>(totalKB) * 1024ull;
        LOG_INFO("System RAM detected: ", totalRAM_ / (1024 * 1024 * 1024), " GB");
    } else {
        totalRAM_ = 16ull * 1024 * 1024 * 1024;
        LOG_WARNING("Could not detect system RAM, assuming 16GB");
    }
#else
    struct sysinfo info;
    if (sysinfo(&info) == 0) {
        totalRAM_ = static_cast<size_t>(info.totalram) * info.mem_unit;
        LOG_INFO("System RAM detected: ", totalRAM_ / (1024 * 1024 * 1024), " GB");
    } else {
        // Fallback: assume 16GB
        totalRAM_ = 16ull * 1024 * 1024 * 1024;
        LOG_WARNING("Could not detect system RAM, assuming 16GB");
    }
#endif
}

size_t MemoryMonitor::getAvailableRAM() const {
#ifdef _WIN32
    MEMORYSTATUSEX status;
    status.dwLength = sizeof(status);
    if (GlobalMemoryStatusEx(&status)) {
        return static_cast<size_t>(status.ullAvailPhys);
    }
    return totalRAM_ / 2;
#else
    // Best source on Linux for reclaimable memory headroom.
    if (size_t memAvailable = readMemAvailableBytesFromProc(); memAvailable > 0) {
        return memAvailable;
    }

    struct sysinfo info;
    if (sysinfo(&info) == 0) {
        // Fallback approximation if /proc/meminfo is unavailable.
        size_t freeBytes = static_cast<size_t>(info.freeram) * info.mem_unit;
        size_t bufferBytes = static_cast<size_t>(info.bufferram) * info.mem_unit;
        size_t available = freeBytes + bufferBytes;
        return (totalRAM_ > 0 && available > totalRAM_) ? totalRAM_ : available;
    }
    return totalRAM_ / 2;  // Fallback: assume 50% available
#endif
}

size_t MemoryMonitor::getRecommendedCacheBudget() const {
    size_t available = getAvailableRAM();
    // Use 80% of available RAM for caches (very aggressive), but cap at 90% of total
    size_t budget = available * 80 / 100;
    size_t maxBudget = totalRAM_ * 90 / 100;
    return budget < maxBudget ? budget : maxBudget;
}

bool MemoryMonitor::isMemoryPressure() const {
    size_t available = getAvailableRAM();
    // Memory pressure if < 20% RAM available
    return available < (totalRAM_ * 20 / 100);
}

} // namespace core
} // namespace wowee
