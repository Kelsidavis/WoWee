#include "game/warden_emulator.hpp"
#include <iostream>
#include <cstring>
#include <chrono>

#ifdef HAVE_UNICORN
// Unicorn Engine headers
#include <unicorn/unicorn.h>
#endif

namespace wowee {
namespace game {

#ifdef HAVE_UNICORN

// Memory layout for emulated environment
constexpr uint32_t STACK_BASE = 0x00100000;  // 1MB
constexpr uint32_t STACK_SIZE = 0x00100000;  // 1MB stack
constexpr uint32_t HEAP_BASE  = 0x00200000;  // 2MB
constexpr uint32_t HEAP_SIZE  = 0x01000000;  // 16MB heap
constexpr uint32_t API_STUB_BASE = 0x70000000; // API stub area (high memory)

WardenEmulator::WardenEmulator()
    : uc_(nullptr)
    , moduleBase_(0)
    , moduleSize_(0)
    , stackBase_(STACK_BASE)
    , stackSize_(STACK_SIZE)
    , heapBase_(HEAP_BASE)
    , heapSize_(HEAP_SIZE)
    , apiStubBase_(API_STUB_BASE)
    , nextHeapAddr_(HEAP_BASE)
{
}

WardenEmulator::~WardenEmulator() {
    if (uc_) {
        uc_close(uc_);
    }
}

bool WardenEmulator::initialize(const void* moduleCode, size_t moduleSize, uint32_t baseAddress) {
    if (uc_) {
        std::cerr << "[WardenEmulator] Already initialized" << std::endl;
        return false;
    }

    std::cout << "[WardenEmulator] Initializing x86 emulator (Unicorn Engine)" << std::endl;
    std::cout << "[WardenEmulator]   Module: " << moduleSize << " bytes at 0x" << std::hex << baseAddress << std::dec << std::endl;

    // Create x86 32-bit emulator
    uc_err err = uc_open(UC_ARCH_X86, UC_MODE_32, &uc_);
    if (err != UC_ERR_OK) {
        std::cerr << "[WardenEmulator] uc_open failed: " << uc_strerror(err) << std::endl;
        return false;
    }

    moduleBase_ = baseAddress;
    moduleSize_ = (moduleSize + 0xFFF) & ~0xFFF; // Align to 4KB

    // Map module memory (code + data)
    err = uc_mem_map(uc_, moduleBase_, moduleSize_, UC_PROT_ALL);
    if (err != UC_ERR_OK) {
        std::cerr << "[WardenEmulator] Failed to map module memory: " << uc_strerror(err) << std::endl;
        uc_close(uc_);
        uc_ = nullptr;
        return false;
    }

    // Write module code to emulated memory
    err = uc_mem_write(uc_, moduleBase_, moduleCode, moduleSize);
    if (err != UC_ERR_OK) {
        std::cerr << "[WardenEmulator] Failed to write module code: " << uc_strerror(err) << std::endl;
        uc_close(uc_);
        uc_ = nullptr;
        return false;
    }

    // Map stack
    err = uc_mem_map(uc_, stackBase_, stackSize_, UC_PROT_READ | UC_PROT_WRITE);
    if (err != UC_ERR_OK) {
        std::cerr << "[WardenEmulator] Failed to map stack: " << uc_strerror(err) << std::endl;
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
        std::cerr << "[WardenEmulator] Failed to map heap: " << uc_strerror(err) << std::endl;
        uc_close(uc_);
        uc_ = nullptr;
        return false;
    }

    // Map API stub area
    err = uc_mem_map(uc_, apiStubBase_, 0x10000, UC_PROT_ALL);
    if (err != UC_ERR_OK) {
        std::cerr << "[WardenEmulator] Failed to map API stub area: " << uc_strerror(err) << std::endl;
        uc_close(uc_);
        uc_ = nullptr;
        return false;
    }

    // Add hooks for debugging and invalid memory access
    uc_hook hh;
    uc_hook_add(uc_, &hh, UC_HOOK_MEM_INVALID, (void*)hookMemInvalid, this, 1, 0);
    hooks_.push_back(hh);

    std::cout << "[WardenEmulator] ✓ Emulator initialized successfully" << std::endl;
    std::cout << "[WardenEmulator]   Stack: 0x" << std::hex << stackBase_ << " - 0x" << (stackBase_ + stackSize_) << std::endl;
    std::cout << "[WardenEmulator]   Heap:  0x" << heapBase_ << " - 0x" << (heapBase_ + heapSize_) << std::dec << std::endl;

    return true;
}

uint32_t WardenEmulator::hookAPI(const std::string& dllName,
                                 const std::string& functionName,
                                 std::function<uint32_t(WardenEmulator&, const std::vector<uint32_t>&)> handler) {
    // Allocate address for this API stub
    static uint32_t nextStubAddr = API_STUB_BASE;
    uint32_t stubAddr = nextStubAddr;
    nextStubAddr += 16; // Space for stub code

    // Store mapping
    apiAddresses_[dllName][functionName] = stubAddr;

    std::cout << "[WardenEmulator] Hooked " << dllName << "!" << functionName
              << " at 0x" << std::hex << stubAddr << std::dec << std::endl;

    // TODO: Write stub code that triggers a hook callback
    // For now, just return the address for IAT patching

    return stubAddr;
}

void WardenEmulator::setupCommonAPIHooks() {
    std::cout << "[WardenEmulator] Setting up common Windows API hooks..." << std::endl;

    // kernel32.dll
    hookAPI("kernel32.dll", "VirtualAlloc", apiVirtualAlloc);
    hookAPI("kernel32.dll", "VirtualFree", apiVirtualFree);
    hookAPI("kernel32.dll", "GetTickCount", apiGetTickCount);
    hookAPI("kernel32.dll", "Sleep", apiSleep);
    hookAPI("kernel32.dll", "GetCurrentThreadId", apiGetCurrentThreadId);
    hookAPI("kernel32.dll", "GetCurrentProcessId", apiGetCurrentProcessId);
    hookAPI("kernel32.dll", "ReadProcessMemory", apiReadProcessMemory);

    std::cout << "[WardenEmulator] ✓ Common API hooks registered" << std::endl;
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

std::vector<uint8_t> WardenEmulator::readData(uint32_t address, size_t size) {
    std::vector<uint8_t> result(size);
    if (!readMemory(address, result.data(), size)) {
        return {};
    }
    return result;
}

uint32_t WardenEmulator::callFunction(uint32_t address, const std::vector<uint32_t>& args) {
    if (!uc_) {
        std::cerr << "[WardenEmulator] Not initialized" << std::endl;
        return 0;
    }

    std::cout << "[WardenEmulator] Calling function at 0x" << std::hex << address << std::dec
              << " with " << args.size() << " args" << std::endl;

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
        std::cerr << "[WardenEmulator] Execution failed: " << uc_strerror(err) << std::endl;
        return 0;
    }

    // Get return value (EAX)
    uint32_t eax;
    uc_reg_read(uc_, UC_X86_REG_EAX, &eax);

    std::cout << "[WardenEmulator] Function returned 0x" << std::hex << eax << std::dec << std::endl;

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

uint32_t WardenEmulator::allocateMemory(size_t size, uint32_t protection) {
    // Align to 4KB
    size = (size + 0xFFF) & ~0xFFF;

    if (nextHeapAddr_ + size > heapBase_ + heapSize_) {
        std::cerr << "[WardenEmulator] Heap exhausted" << std::endl;
        return 0;
    }

    uint32_t addr = nextHeapAddr_;
    nextHeapAddr_ += size;

    allocations_[addr] = size;

    std::cout << "[WardenEmulator] Allocated " << size << " bytes at 0x" << std::hex << addr << std::dec << std::endl;

    return addr;
}

bool WardenEmulator::freeMemory(uint32_t address) {
    auto it = allocations_.find(address);
    if (it == allocations_.end()) {
        std::cerr << "[WardenEmulator] Invalid free at 0x" << std::hex << address << std::dec << std::endl;
        return false;
    }

    std::cout << "[WardenEmulator] Freed " << it->second << " bytes at 0x" << std::hex << address << std::dec << std::endl;
    allocations_.erase(it);
    return true;
}

uint32_t WardenEmulator::getRegister(int regId) {
    uint32_t value = 0;
    if (uc_) {
        uc_reg_read(uc_, regId, &value);
    }
    return value;
}

void WardenEmulator::setRegister(int regId, uint32_t value) {
    if (uc_) {
        uc_reg_write(uc_, regId, &value);
    }
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

    std::cout << "[WinAPI] VirtualAlloc(0x" << std::hex << lpAddress << ", " << std::dec
              << dwSize << ", 0x" << std::hex << flAllocationType << ", 0x" << flProtect << ")" << std::dec << std::endl;

    // Ignore lpAddress hint for now
    return emu.allocateMemory(dwSize, flProtect);
}

uint32_t WardenEmulator::apiVirtualFree(WardenEmulator& emu, const std::vector<uint32_t>& args) {
    // VirtualFree(lpAddress, dwSize, dwFreeType)
    if (args.size() < 3) return 0;

    uint32_t lpAddress = args[0];

    std::cout << "[WinAPI] VirtualFree(0x" << std::hex << lpAddress << ")" << std::dec << std::endl;

    return emu.freeMemory(lpAddress) ? 1 : 0;
}

uint32_t WardenEmulator::apiGetTickCount(WardenEmulator& emu, const std::vector<uint32_t>& args) {
    auto now = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    uint32_t ticks = static_cast<uint32_t>(ms & 0xFFFFFFFF);

    std::cout << "[WinAPI] GetTickCount() = " << ticks << std::endl;
    return ticks;
}

uint32_t WardenEmulator::apiSleep(WardenEmulator& emu, const std::vector<uint32_t>& args) {
    if (args.size() < 1) return 0;
    uint32_t dwMilliseconds = args[0];

    std::cout << "[WinAPI] Sleep(" << dwMilliseconds << ")" << std::endl;
    // Don't actually sleep in emulator
    return 0;
}

uint32_t WardenEmulator::apiGetCurrentThreadId(WardenEmulator& emu, const std::vector<uint32_t>& args) {
    std::cout << "[WinAPI] GetCurrentThreadId() = 1234" << std::endl;
    return 1234; // Fake thread ID
}

uint32_t WardenEmulator::apiGetCurrentProcessId(WardenEmulator& emu, const std::vector<uint32_t>& args) {
    std::cout << "[WinAPI] GetCurrentProcessId() = 5678" << std::endl;
    return 5678; // Fake process ID
}

uint32_t WardenEmulator::apiReadProcessMemory(WardenEmulator& emu, const std::vector<uint32_t>& args) {
    // ReadProcessMemory(hProcess, lpBaseAddress, lpBuffer, nSize, lpNumberOfBytesRead)
    if (args.size() < 5) return 0;

    uint32_t hProcess = args[0];
    uint32_t lpBaseAddress = args[1];
    uint32_t lpBuffer = args[2];
    uint32_t nSize = args[3];
    uint32_t lpNumberOfBytesRead = args[4];

    std::cout << "[WinAPI] ReadProcessMemory(0x" << std::hex << lpBaseAddress
              << ", " << std::dec << nSize << " bytes)" << std::endl;

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

void WardenEmulator::hookCode(uc_engine* uc, uint64_t address, uint32_t size, void* userData) {
    WardenEmulator* emu = static_cast<WardenEmulator*>(userData);
    std::cout << "[Trace] 0x" << std::hex << address << std::dec << std::endl;
}

void WardenEmulator::hookMemInvalid(uc_engine* uc, int type, uint64_t address, int size, int64_t value, void* userData) {
    WardenEmulator* emu = static_cast<WardenEmulator*>(userData);

    const char* typeStr = "UNKNOWN";
    switch (type) {
        case UC_MEM_READ_UNMAPPED: typeStr = "READ_UNMAPPED"; break;
        case UC_MEM_WRITE_UNMAPPED: typeStr = "WRITE_UNMAPPED"; break;
        case UC_MEM_FETCH_UNMAPPED: typeStr = "FETCH_UNMAPPED"; break;
        case UC_MEM_READ_PROT: typeStr = "READ_PROT"; break;
        case UC_MEM_WRITE_PROT: typeStr = "WRITE_PROT"; break;
        case UC_MEM_FETCH_PROT: typeStr = "FETCH_PROT"; break;
    }

    std::cerr << "[WardenEmulator] Invalid memory access: " << typeStr
              << " at 0x" << std::hex << address << std::dec
              << " (size=" << size << ")" << std::endl;
}

#else // !HAVE_UNICORN
// Stub implementations — Unicorn Engine not available on this platform.
WardenEmulator::WardenEmulator()
    : uc_(nullptr), moduleBase_(0), moduleSize_(0)
    , stackBase_(0), stackSize_(0)
    , heapBase_(0), heapSize_(0)
    , apiStubBase_(0), nextHeapAddr_(0) {}
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
uint32_t WardenEmulator::getRegister(int) { return 0; }
void WardenEmulator::setRegister(int, uint32_t) {}
void WardenEmulator::setupCommonAPIHooks() {}
uint32_t WardenEmulator::writeData(const void*, size_t) { return 0; }
std::vector<uint8_t> WardenEmulator::readData(uint32_t, size_t) { return {}; }
void WardenEmulator::hookCode(uc_engine*, uint64_t, uint32_t, void*) {}
void WardenEmulator::hookMemInvalid(uc_engine*, int, uint64_t, int, int64_t, void*) {}
#endif // HAVE_UNICORN

} // namespace game
} // namespace wowee
