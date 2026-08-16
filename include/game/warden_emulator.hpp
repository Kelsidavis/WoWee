#pragma once

#include <vector>
#include <cstdint>
#include <string>
#include <memory>
#include <map>
#include <unordered_map>
#include <functional>

// Forward declare unicorn types (will include in .cpp)
typedef struct uc_struct uc_engine;
typedef size_t uc_hook;

namespace wowee {
namespace game {

/**
 * Cross-platform x86 emulator for Warden modules
 *
 * Uses Unicorn Engine to emulate Windows x86 code on any platform.
 * Provides Windows API hooks and Warden callback infrastructure.
 *
 * Architecture:
 * - CPU Emulation: x86 (32-bit) via Unicorn Engine
 * - Memory: Emulated address space (separate from host process)
 * - API Hooks: Intercept Windows API calls and provide implementations
 * - Callbacks: Bridge between emulated module and native wowee code
 *
 * Benefits:
 * - Works on Linux/macOS/BSD without Wine
 * - Sandboxed execution (module can't harm host system)
 * - Full control over memory and API calls
 * - Can run on ARM/non-x86 hosts
 */
class WardenEmulator {
public:
    WardenEmulator();
    ~WardenEmulator();

    /**
     * Initialize emulator with module code
     *
     * @param moduleCode Loaded x86 code (post-relocation)
     * @param moduleSize Size of code in bytes
     * @param baseAddress Preferred base address (e.g., 0x400000)
     * @return true if initialization successful
     */
    bool initialize(const void* moduleCode, size_t moduleSize, uint32_t baseAddress = 0x400000);

    /**
     * Map Windows API function to implementation
     *
     * When emulated code calls this API, our hook will be invoked.
     *
     * @param dllName DLL name (e.g., "kernel32.dll")
     * @param functionName Function name (e.g., "VirtualAlloc")
     * @param handler Native function to call (receives emulator context)
     * @return Address where API was mapped (for IAT patching)
     */
    uint32_t hookAPI(const std::string& dllName,
                     const std::string& functionName,
                     std::function<uint32_t(WardenEmulator&, const std::vector<uint32_t>&)> handler);

    /**
     * Call emulated function
     *
     * @param address Address of function in emulated space
     * @param args Arguments to pass (stdcall convention)
     * @return Return value from function (EAX)
     */
    uint32_t callFunction(uint32_t address, const std::vector<uint32_t>& args = {});

    /**
     * Read memory from emulated address space
     */
    bool readMemory(uint32_t address, void* buffer, size_t size);

    /**
     * Write memory to emulated address space
     */
    bool writeMemory(uint32_t address, const void* buffer, size_t size);

    /**
     * Read string from emulated memory
     */
    std::string readString(uint32_t address, size_t maxLen = 256);

    /**
     * Allocate memory in emulated space
     *
     * Used by VirtualAlloc hook implementation.
     */
    uint32_t allocateMemory(size_t size, uint32_t protection);

    /**
     * Free memory in emulated space
     */
    bool freeMemory(uint32_t address);

    /**
     * Check if emulator is initialized
     */
    [[nodiscard]] bool isInitialized() const { return uc_ != nullptr; }

    /**
     * Get module base address
     */
    [[nodiscard]] uint32_t getModuleBase() const { return moduleBase_; }

    /**
     * Setup common Windows API hooks
     *
     * Hooks frequently used APIs with stub implementations.
     */
    void setupCommonAPIHooks();

    /**
     * Write data to emulated memory and return address
     *
     * Convenience helper that allocates, writes, and returns address.
     * Caller is responsible for freeing with freeMemory().
     */
    uint32_t writeData(const void* data, size_t size);

    // Look up an already-registered API stub address by DLL and function name.
    // Returns 0 if not found. Used by WardenModule::bindAPIs() for IAT patching.
    [[nodiscard]] uint32_t getAPIAddress(const std::string& dllName, const std::string& funcName) const;

private:
    // Memory layout for the emulated environment. Here rather than in the
    // .cpp because the fields below carry them as initialisers, and the .cpp
    // defined them inside #ifdef HAVE_UNICORN - so the stub constructor could
    // not see them and zeroed all five instead, leaving two constructors that
    // had to agree about seven fields and did not.
    //
    // The heap must not overlap the module region (typically loaded at
    // 0x400000) or the stack, so keep it above 32MB to leave the module room.
    static constexpr uint32_t kStackBase   = 0x00100000;  // 1MB
    static constexpr uint32_t kStackSize   = 0x00100000;  // 1MB stack
    static constexpr uint32_t kHeapBase    = 0x02000000;  // 32MB
    static constexpr uint32_t kHeapSize    = 0x01000000;  // 16MB heap
    static constexpr uint32_t kApiStubBase = 0x70000000;  // high memory

    uc_engine* uc_ = nullptr;                  // Unicorn engine instance
    uint32_t moduleBase_ = 0;         // Module base address
    uint32_t moduleSize_ = 0;         // Module size
    uint32_t stackBase_ = kStackBase; // Stack base address
    uint32_t stackSize_ = kStackSize; // Stack size
    uint32_t heapBase_ = kHeapBase;   // Heap base address
    uint32_t heapSize_ = kHeapSize;   // Heap size
    uint32_t apiStubBase_ = kApiStubBase;  // API stub base address

    // API hooks: DLL name -> Function name -> stub address
    std::unordered_map<std::string, std::unordered_map<std::string, uint32_t>> apiAddresses_;

    // API stub dispatch: stub address -> {argCount, handler}
    struct ApiHookEntry {
        int argCount;
        std::function<uint32_t(WardenEmulator&, const std::vector<uint32_t>&)> handler;
    };
    std::unordered_map<uint32_t, ApiHookEntry> apiHandlers_;
    uint32_t nextApiStubAddr_ = kApiStubBase;  // next free stub slot
    bool apiCodeHookRegistered_ = false; // true once UC_HOOK_CODE for stub range is added

    // Memory allocation tracking
    std::unordered_map<uint32_t, size_t> allocations_;
    std::map<uint32_t, size_t> freeBlocks_;  // free-list keyed by base address
    uint32_t nextHeapAddr_ = kHeapBase;

    // Hook handles for cleanup
    std::vector<uc_hook> hooks_;

    // Windows API implementations
    static uint32_t apiVirtualAlloc(WardenEmulator& emu, const std::vector<uint32_t>& args);
    static uint32_t apiVirtualFree(WardenEmulator& emu, const std::vector<uint32_t>& args);
    static uint32_t apiGetTickCount(WardenEmulator& emu, const std::vector<uint32_t>& args);
    static uint32_t apiSleep(WardenEmulator& emu, const std::vector<uint32_t>& args);
    static uint32_t apiGetCurrentThreadId(WardenEmulator& emu, const std::vector<uint32_t>& args);
    static uint32_t apiGetCurrentProcessId(WardenEmulator& emu, const std::vector<uint32_t>& args);
    static uint32_t apiReadProcessMemory(WardenEmulator& emu, const std::vector<uint32_t>& args);

    // Unicorn callbacks
    static void hookCode(uc_engine* uc, uint64_t address, uint32_t size, void* userData);
    static void hookMemInvalid(uc_engine* uc, int type, uint64_t address, int size, int64_t value, void* userData);
};

} // namespace game
} // namespace wowee
