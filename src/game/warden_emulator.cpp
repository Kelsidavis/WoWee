#include "game/warden_emulator.hpp"
#include "core/logger.hpp"
#include <cstring>
#include <chrono>
#include <iterator>

#ifdef HAVE_UNICORN
// Unicorn Engine headers
#include <unicorn/unicorn.h>
#endif

namespace wowee {
namespace game {

#ifdef HAVE_UNICORN

// Memory layout for emulated environment
// Note: heap must not overlap the module region (typically loaded at 0x400000)
// or the stack. Keep heap above 0x02000000 (32MB) to leave space for module + padding.
constexpr uint32_t STACK_BASE = 0x00100000;  // 1MB
constexpr uint32_t STACK_SIZE = 0x00100000;  // 1MB stack
constexpr uint32_t HEAP_BASE  = 0x02000000;  // 32MB - well above typical module base (0x400000)
constexpr uint32_t HEAP_SIZE  = 0x01000000;  // 16MB heap
constexpr uint32_t API_STUB_BASE = 0x70000000; // API stub area (high memory)

WardenEmulator::WardenEmulator()
    : 
     moduleBase_(0)
    , moduleSize_(0)
    , stackBase_(STACK_BASE)
    , stackSize_(STACK_SIZE)
    , heapBase_(HEAP_BASE)
    , heapSize_(HEAP_SIZE)
    , apiStubBase_(API_STUB_BASE)
    , nextApiStubAddr_(API_STUB_BASE)
    , 
     nextHeapAddr_(HEAP_BASE)
{
}

WardenEmulator::~WardenEmulator() {
    if (uc_) {
        uc_close(uc_);
    }
}

bool WardenEmulator::initialize(const void* moduleCode, size_t moduleSize, uint32_t baseAddress) {
    if (uc_) {
        LOG_ERROR("WardenEmulator: Already initialized");
        return false;
    }
    // Reset allocator state so re-initialization starts with a clean heap.
    allocations_.clear();
    freeBlocks_.clear();
    apiAddresses_.clear();
    apiHandlers_.clear();
    hooks_.clear();
    nextHeapAddr_ = heapBase_;
    nextApiStubAddr_ = apiStubBase_;
    apiCodeHookRegistered_ = false;

    {
        char addrBuf[32];
        std::snprintf(addrBuf, sizeof(addrBuf), "0x%X", baseAddress);
        LOG_INFO("WardenEmulator: Initializing x86 emulator (Unicorn Engine)");
        LOG_INFO("WardenEmulator:   Module: ", moduleSize, " bytes at ", addrBuf);
    }

    // Create x86 32-bit emulator
    uc_err err = uc_open(UC_ARCH_X86, UC_MODE_32, &uc_);
    if (err != UC_ERR_OK) {
        LOG_ERROR("WardenEmulator: uc_open failed: ", uc_strerror(err));
        return false;
    }

    moduleBase_ = baseAddress;
    moduleSize_ = (moduleSize + 0xFFF) & ~0xFFF; // Align to 4KB

    // Detect overlap between module and heap/stack regions early.
    uint32_t modEnd = moduleBase_ + moduleSize_;
    if (modEnd > heapBase_ && moduleBase_ < heapBase_ + heapSize_) {
        {
            char buf[256];
            std::snprintf(buf, sizeof(buf), "WardenEmulator: Module [0x%X, 0x%X) overlaps heap [0x%X, 0x%X) - adjust HEAP_BASE",
                          moduleBase_, modEnd, heapBase_, heapBase_ + heapSize_);
            LOG_ERROR(buf);
        }
        uc_close(uc_);
        uc_ = nullptr;
        return false;
    }

    // Map module memory (code + data)
    err = uc_mem_map(uc_, moduleBase_, moduleSize_, UC_PROT_ALL);
    if (err != UC_ERR_OK) {
        LOG_ERROR("WardenEmulator: Failed to map module memory: ", uc_strerror(err));
        uc_close(uc_);
        uc_ = nullptr;
        return false;
    }

    // Write module code to emulated memory
    err = uc_mem_write(uc_, moduleBase_, moduleCode, moduleSize);
    if (err != UC_ERR_OK) {
        LOG_ERROR("WardenEmulator: Failed to write module code: ", uc_strerror(err));
        uc_close(uc_);
        uc_ = nullptr;
        return false;
    }

    // Map stack
    err = uc_mem_map(uc_, stackBase_, stackSize_, UC_PROT_READ | UC_PROT_WRITE);
    if (err != UC_ERR_OK) {
        LOG_ERROR("WardenEmulator: Failed to map stack: ", uc_strerror(err));
        uc_close(uc_);
        uc_ = nullptr;
        return false;
    }

    // Initialize stack pointer (grows downward)
    uint32_t esp = stackBase_ + stackSize_ - 0x1000; // Leave some space at top
    uc_reg_write(uc_, UC_X86_REG_ESP, &esp);
    uc_reg_write(uc_, UC_X86_REG_EBP, &esp);

    // Map heap
    err = uc_mem_map(uc_, heapBase_, heapSize_, UC_PROT_READ | UC_PROT_WRITE);
    if (err != UC_ERR_OK) {
        LOG_ERROR("WardenEmulator: Failed to map heap: ", uc_strerror(err));
        uc_close(uc_);
        uc_ = nullptr;
        return false;
    }

    // Map API stub area
    err = uc_mem_map(uc_, apiStubBase_, 0x10000, UC_PROT_ALL);
    if (err != UC_ERR_OK) {
        LOG_ERROR("WardenEmulator: Failed to map API stub area: ", uc_strerror(err));
        uc_close(uc_);
        uc_ = nullptr;
        return false;
    }

    // Map a null guard page at address 0 (read-only, zeroed) so that NULL-pointer
    // dereferences in the module don't crash the emulator with UC_ERR_MAP.
    // This allows execution to continue past NULL reads, making diagnostics easier.
    err = uc_mem_map(uc_, 0x0, 0x1000, UC_PROT_READ);
    if (err != UC_ERR_OK) {
        // Non-fatal - just log it; the emulator will still function
        LOG_WARNING("WardenEmulator: could not map null guard page: ", uc_strerror(err));
    }

    // Add hooks for debugging and invalid memory access
    uc_hook hh;
    uc_hook_add(uc_, &hh, UC_HOOK_MEM_INVALID, (void*)hookMemInvalid, this, 1, 0);
    hooks_.push_back(hh);

    // Add code hook over the API stub area so Windows API calls are intercepted
    uc_hook apiHook;
    uc_hook_add(uc_, &apiHook, UC_HOOK_CODE, (void*)hookCode, this,
                API_STUB_BASE, API_STUB_BASE + 0x10000 - 1);
    hooks_.push_back(apiHook);
    apiCodeHookRegistered_ = true;

    {
        char sBuf[128];
        std::snprintf(sBuf, sizeof(sBuf), "WardenEmulator: Emulator initialized  Stack: 0x%X-0x%X  Heap: 0x%X-0x%X",
                      stackBase_, stackBase_ + stackSize_, heapBase_, heapBase_ + heapSize_);
        LOG_INFO(sBuf);
    }

    return true;
}

uint32_t WardenEmulator::hookAPI(const std::string& dllName,
                                 const std::string& functionName,
                                 std::function<uint32_t(WardenEmulator&, const std::vector<uint32_t>&)> handler) {
    // Allocate address for this API stub (16 bytes each)
    uint32_t stubAddr = nextApiStubAddr_;
    nextApiStubAddr_ += 16;

    // Store address mapping for IAT patching
    apiAddresses_[dllName][functionName] = stubAddr;

    // Determine stdcall arg count from known Windows APIs so the hook can
    // clean up the stack correctly (RETN N convention).
    static const std::pair<const char*, int> knownArgCounts[] = {
        {"VirtualAlloc",           4},
        {"VirtualFree",            3},
        {"GetTickCount",           0},
        {"Sleep",                  1},
        {"GetCurrentThreadId",     0},
        {"GetCurrentProcessId",    0},
        {"ReadProcessMemory",      5},
    };
    int argCount = 0;
    for (const auto& [name, cnt] : knownArgCounts) {
        if (functionName == name) { argCount = cnt; break; }
    }

    // Store the handler so hookCode() can dispatch to it
    apiHandlers_[stubAddr] = { argCount, std::move(handler) };

    // Write a RET (0xC3) at the stub address as a safe fallback in case
    // the code hook fires after EIP has already advanced past our intercept.
    if (uc_) {
        static constexpr uint8_t retInstr = 0xC3;
        uc_mem_write(uc_, stubAddr, &retInstr, 1);
    }

    {
        char hBuf[64];
        std::snprintf(hBuf, sizeof(hBuf), "0x%X (argCount=%d)", stubAddr, argCount);
        LOG_DEBUG("WardenEmulator: Hooked ", dllName, "!", functionName, " at ", hBuf);
    }

    return stubAddr;
}

uint32_t WardenEmulator::getAPIAddress(const std::string& dllName, const std::string& funcName) const {
    auto libIt = apiAddresses_.find(dllName);
    if (libIt == apiAddresses_.end()) return 0;
    auto funcIt = libIt->second.find(funcName);
    return (funcIt != libIt->second.end()) ? funcIt->second : 0;
}

void WardenEmulator::setupCommonAPIHooks() {
    LOG_INFO("WardenEmulator: Setting up common Windows API hooks...");

    // kernel32.dll
    hookAPI("kernel32.dll", "VirtualAlloc", apiVirtualAlloc);
    hookAPI("kernel32.dll", "VirtualFree", apiVirtualFree);
    hookAPI("kernel32.dll", "GetTickCount", apiGetTickCount);
    hookAPI("kernel32.dll", "Sleep", apiSleep);
    hookAPI("kernel32.dll", "GetCurrentThreadId", apiGetCurrentThreadId);
    hookAPI("kernel32.dll", "GetCurrentProcessId", apiGetCurrentProcessId);
    hookAPI("kernel32.dll", "ReadProcessMemory", apiReadProcessMemory);

    LOG_INFO("WardenEmulator: Common API hooks registered");
}

uint32_t WardenEmulator::writeData(const void* data, size_t size) {
    uint32_t addr = allocateMemory(size, 0x04);
    if (addr != 0) {
        if (!writeMemory(addr, data, size)) {
            freeMemory(addr);
            return 0;
        }
    }
    return addr;
}
uint32_t WardenEmulator::callFunction(uint32_t address, const std::vector<uint32_t>& args) {
    if (!uc_) {
        LOG_ERROR("WardenEmulator: Not initialized");
        return 0;
    }

    {
        char aBuf[32];
        std::snprintf(aBuf, sizeof(aBuf), "0x%X", address);
        LOG_DEBUG("WardenEmulator: Calling function at ", aBuf, " with ", args.size(), " args");
    }

    // Get current ESP
    uint32_t esp;
    uc_reg_read(uc_, UC_X86_REG_ESP, &esp);

    // Push arguments (stdcall: right-to-left)
    for (auto it = args.rbegin(); it != args.rend(); ++it) {
        esp -= 4;
        uint32_t arg = *it;
        uc_mem_write(uc_, esp, &arg, 4);
    }

    // Push return address (0xFFFFFFFF = terminator)
    uint32_t retAddr = 0xFFFFFFFF;
    esp -= 4;
    uc_mem_write(uc_, esp, &retAddr, 4);

    // Update ESP
    uc_reg_write(uc_, UC_X86_REG_ESP, &esp);

    // Execute until return address
    uc_err err = uc_emu_start(uc_, address, retAddr, 0, 0);
    if (err != UC_ERR_OK) {
        LOG_ERROR("WardenEmulator: Execution failed: ", uc_strerror(err));
        return 0;
    }

    // Get return value (EAX)
    uint32_t eax;
    uc_reg_read(uc_, UC_X86_REG_EAX, &eax);

    {
        char rBuf[32];
        std::snprintf(rBuf, sizeof(rBuf), "0x%X", eax);
        LOG_DEBUG("WardenEmulator: Function returned ", rBuf);
    }

    return eax;
}

bool WardenEmulator::readMemory(uint32_t address, void* buffer, size_t size) {
    if (!uc_) return false;
    uc_err err = uc_mem_read(uc_, address, buffer, size);
    return (err == UC_ERR_OK);
}

bool WardenEmulator::writeMemory(uint32_t address, const void* buffer, size_t size) {
    if (!uc_) return false;
    uc_err err = uc_mem_write(uc_, address, buffer, size);
    return (err == UC_ERR_OK);
}

std::string WardenEmulator::readString(uint32_t address, size_t maxLen) {
    std::vector<char> buffer(maxLen + 1, 0);
    if (!readMemory(address, buffer.data(), maxLen)) {
        return "";
    }
    buffer[maxLen] = '\0'; // Ensure null termination
    return std::string(buffer.data());
}

uint32_t WardenEmulator::allocateMemory(size_t size, [[maybe_unused]] uint32_t protection) {
    if (size == 0) return 0;

    // Align to 4KB
    size = (size + 0xFFF) & ~0xFFF;
    const uint32_t allocSize = static_cast<uint32_t>(size);

    // First-fit from free list so released blocks can be reused.
    for (auto it = freeBlocks_.begin(); it != freeBlocks_.end(); ++it) {
        if (it->second < size) continue;
        const uint32_t addr     = it->first;
        const size_t   blockSz  = it->second;
        freeBlocks_.erase(it);
        if (blockSz > size)
            freeBlocks_[addr + allocSize] = blockSz - size;
        allocations_[addr] = size;
        {
            char mBuf[32];
            std::snprintf(mBuf, sizeof(mBuf), "0x%X", addr);
            LOG_DEBUG("WardenEmulator: Reused ", size, " bytes at ", mBuf);
        }
        return addr;
    }

    const uint64_t heapEnd = static_cast<uint64_t>(heapBase_) + heapSize_;
    if (static_cast<uint64_t>(nextHeapAddr_) + size > heapEnd) {
        LOG_ERROR("WardenEmulator: Heap exhausted");
        return 0;
    }

    uint32_t addr = nextHeapAddr_;
    nextHeapAddr_ += allocSize;
    allocations_[addr] = size;

    {
        char mBuf[32];
        std::snprintf(mBuf, sizeof(mBuf), "0x%X", addr);
        LOG_DEBUG("WardenEmulator: Allocated ", size, " bytes at ", mBuf);
    }

    return addr;
}

bool WardenEmulator::freeMemory(uint32_t address) {
    auto it = allocations_.find(address);
    if (it == allocations_.end()) {
        {
            char fBuf[32];
            std::snprintf(fBuf, sizeof(fBuf), "0x%X", address);
            LOG_ERROR("WardenEmulator: Invalid free at ", fBuf);
        }
        return false;
    }

    {
        char fBuf[32];
        std::snprintf(fBuf, sizeof(fBuf), "0x%X", address);
        LOG_DEBUG("WardenEmulator: Freed ", it->second, " bytes at ", fBuf);
    }

    const size_t freedSize = it->second;
    allocations_.erase(it);

    // Insert in free list and coalesce adjacent blocks to limit fragmentation.
    auto [curr, inserted] = freeBlocks_.emplace(address, freedSize);
    if (!inserted) curr->second += freedSize;

    if (curr != freeBlocks_.begin()) {
        auto prev = std::prev(curr);
        if (static_cast<uint64_t>(prev->first) + prev->second == curr->first) {
            prev->second += curr->second;
            freeBlocks_.erase(curr);
            curr = prev;
        }
    }

    auto next = std::next(curr);
    if (next != freeBlocks_.end() &&
        static_cast<uint64_t>(curr->first) + curr->second == next->first) {
        curr->second += next->second;
        freeBlocks_.erase(next);
    }

    // Roll back the bump pointer if the highest free block reaches it.
    while (!freeBlocks_.empty()) {
        auto last = std::prev(freeBlocks_.end());
        if (static_cast<uint64_t>(last->first) + last->second == nextHeapAddr_) {
            nextHeapAddr_ = last->first;
            freeBlocks_.erase(last);
        } else {
            break;
        }
    }

    return true;
}
// ============================================================================
// Windows API Implementations
// ============================================================================

uint32_t WardenEmulator::apiVirtualAlloc(WardenEmulator& emu, const std::vector<uint32_t>& args) {
    // VirtualAlloc(lpAddress, dwSize, flAllocationType, flProtect)
    if (args.size() < 4) return 0;

    uint32_t lpAddress = args[0];
    uint32_t dwSize = args[1];
    uint32_t flAllocationType = args[2];
    uint32_t flProtect = args[3];

    {
        char vBuf[128];
        std::snprintf(vBuf, sizeof(vBuf), "WinAPI: VirtualAlloc(0x%X, %u, 0x%X, 0x%X)",
                      lpAddress, dwSize, flAllocationType, flProtect);
        LOG_DEBUG(vBuf);
    }

    // Ignore lpAddress hint for now
    return emu.allocateMemory(dwSize, flProtect);
}

uint32_t WardenEmulator::apiVirtualFree(WardenEmulator& emu, const std::vector<uint32_t>& args) {
    // VirtualFree(lpAddress, dwSize, dwFreeType)
    if (args.size() < 3) return 0;

    uint32_t lpAddress = args[0];

    {
        char vBuf[64];
        std::snprintf(vBuf, sizeof(vBuf), "WinAPI: VirtualFree(0x%X)", lpAddress);
        LOG_DEBUG(vBuf);
    }

    return emu.freeMemory(lpAddress) ? 1 : 0;
}

uint32_t WardenEmulator::apiGetTickCount([[maybe_unused]] WardenEmulator& emu, [[maybe_unused]] const std::vector<uint32_t>& args) {
    auto now = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    uint32_t ticks = static_cast<uint32_t>(ms & 0xFFFFFFFF);

    LOG_DEBUG("WinAPI: GetTickCount() = ", ticks);
    return ticks;
}

uint32_t WardenEmulator::apiSleep([[maybe_unused]] WardenEmulator& emu, const std::vector<uint32_t>& args) {
    if (args.empty()) return 0;
    uint32_t dwMilliseconds = args[0];

    LOG_DEBUG("WinAPI: Sleep(", dwMilliseconds, ")");
    // Don't actually sleep in emulator
    return 0;
}

uint32_t WardenEmulator::apiGetCurrentThreadId([[maybe_unused]] WardenEmulator& emu, [[maybe_unused]] const std::vector<uint32_t>& args) {
    LOG_DEBUG("WinAPI: GetCurrentThreadId() = 1234");
    return 1234; // Fake thread ID
}

uint32_t WardenEmulator::apiGetCurrentProcessId([[maybe_unused]] WardenEmulator& emu, [[maybe_unused]] const std::vector<uint32_t>& args) {
    LOG_DEBUG("WinAPI: GetCurrentProcessId() = 5678");
    return 5678; // Fake process ID
}

uint32_t WardenEmulator::apiReadProcessMemory(WardenEmulator& emu, const std::vector<uint32_t>& args) {
    // ReadProcessMemory(hProcess, lpBaseAddress, lpBuffer, nSize, lpNumberOfBytesRead)
    if (args.size() < 5) return 0;

    [[maybe_unused]] uint32_t hProcess = args[0];
    uint32_t lpBaseAddress = args[1];
    uint32_t lpBuffer = args[2];
    uint32_t nSize = args[3];
    uint32_t lpNumberOfBytesRead = args[4];

    {
        char rBuf[64];
        std::snprintf(rBuf, sizeof(rBuf), "WinAPI: ReadProcessMemory(0x%X, %u bytes)", lpBaseAddress, nSize);
        LOG_DEBUG(rBuf);
    }

    // Read from emulated memory and write to buffer
    std::vector<uint8_t> data(nSize);
    if (!emu.readMemory(lpBaseAddress, data.data(), nSize)) {
        return 0; // Failure
    }

    if (!emu.writeMemory(lpBuffer, data.data(), nSize)) {
        return 0; // Failure
    }

    if (lpNumberOfBytesRead != 0) {
        emu.writeMemory(lpNumberOfBytesRead, &nSize, 4);
    }

    return 1; // Success
}

// ============================================================================
// Unicorn Callbacks
// ============================================================================

void WardenEmulator::hookCode(uc_engine* uc, uint64_t address, [[maybe_unused]] uint32_t size, void* userData) {
    auto* self = static_cast<WardenEmulator*>(userData);
    if (!self) return;

    auto it = self->apiHandlers_.find(static_cast<uint32_t>(address));
    if (it == self->apiHandlers_.end()) return; // not an API stub - trace disabled to avoid spam

    const ApiHookEntry& entry = it->second;

    // Read stack: [ESP+0] = return address, [ESP+4..] = stdcall args
    uint32_t esp = 0;
    uc_reg_read(uc, UC_X86_REG_ESP, &esp);

    uint32_t retAddr = 0;
    uc_mem_read(uc, esp, &retAddr, 4);

    std::vector<uint32_t> args(static_cast<size_t>(entry.argCount));
    for (int i = 0; i < entry.argCount; ++i) {
        uint32_t val = 0;
        uc_mem_read(uc, esp + 4 + static_cast<uint32_t>(i) * 4, &val, 4);
        args[static_cast<size_t>(i)] = val;
    }

    // Dispatch to the C++ handler
    uint32_t retVal = 0;
    if (entry.handler) {
        retVal = entry.handler(*self, args);
    }

    // Simulate stdcall epilogue: pop return address + args
    uint32_t newEsp = esp + 4 + static_cast<uint32_t>(entry.argCount) * 4;
    uc_reg_write(uc, UC_X86_REG_EAX, &retVal);
    uc_reg_write(uc, UC_X86_REG_ESP, &newEsp);
    uc_reg_write(uc, UC_X86_REG_EIP, &retAddr);
}

void WardenEmulator::hookMemInvalid([[maybe_unused]] uc_engine* uc, int type, uint64_t address, int size, [[maybe_unused]] int64_t value, [[maybe_unused]] void* userData) {

    const char* typeStr = "UNKNOWN";
    switch (type) {
        case UC_MEM_READ_UNMAPPED: typeStr = "READ_UNMAPPED"; break;
        case UC_MEM_WRITE_UNMAPPED: typeStr = "WRITE_UNMAPPED"; break;
        case UC_MEM_FETCH_UNMAPPED: typeStr = "FETCH_UNMAPPED"; break;
        case UC_MEM_READ_PROT: typeStr = "READ_PROT"; break;
        case UC_MEM_WRITE_PROT: typeStr = "WRITE_PROT"; break;
        case UC_MEM_FETCH_PROT: typeStr = "FETCH_PROT"; break;
    }

    {
        char mBuf[128];
        std::snprintf(mBuf, sizeof(mBuf), "WardenEmulator: Invalid memory access: %s at 0x%llX (size=%d)",
                      typeStr, static_cast<unsigned long long>(address), size);
        LOG_ERROR(mBuf);
    }
}

#else // !HAVE_UNICORN
// Stub implementations - Unicorn Engine not available on this platform.
WardenEmulator::WardenEmulator()
    : uc_(nullptr), moduleBase_(0), moduleSize_(0)
    , stackBase_(0), stackSize_(0)
    , heapBase_(0), heapSize_(0)
    , apiStubBase_(0), nextApiStubAddr_(0), apiCodeHookRegistered_(false)
    , nextHeapAddr_(0) {}
WardenEmulator::~WardenEmulator() {}
bool WardenEmulator::initialize(const void*, size_t, uint32_t) { return false; }
uint32_t WardenEmulator::hookAPI(const std::string&, const std::string&,
    std::function<uint32_t(WardenEmulator&, const std::vector<uint32_t>&)>) { return 0; }
uint32_t WardenEmulator::callFunction(uint32_t, const std::vector<uint32_t>&) { return 0; }
bool WardenEmulator::readMemory(uint32_t, void*, size_t) { return false; }
bool WardenEmulator::writeMemory(uint32_t, const void*, size_t) { return false; }
std::string WardenEmulator::readString(uint32_t, size_t) { return {}; }
uint32_t WardenEmulator::allocateMemory(size_t, uint32_t) { return 0; }
bool WardenEmulator::freeMemory(uint32_t) { return false; }
void WardenEmulator::setupCommonAPIHooks() {}
uint32_t WardenEmulator::getAPIAddress(const std::string&, const std::string&) const { return 0; }
uint32_t WardenEmulator::writeData(const void*, size_t) { return 0; }
void WardenEmulator::hookCode(uc_engine*, uint64_t, uint32_t, void*) {}
void WardenEmulator::hookMemInvalid(uc_engine*, int, uint64_t, int, int64_t, void*) {}
#endif // HAVE_UNICORN

} // namespace game
} // namespace wowee
