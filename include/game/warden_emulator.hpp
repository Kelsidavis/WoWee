#pragma once

#include <vector>
#include <cstdint>
#include <string>
#include <memory>
#include <map>
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
     * Get CPU register value
     */
    uint32_t getRegister(int regId);

    /**
     * Set CPU register value
     */
    void setRegister(int regId, uint32_t value);

    /**
     * Check if emulator is initialized
     */
    bool isInitialized() const { return uc_ != nullptr; }

    /**
     * Get module base address
     */
    uint32_t getModuleBase() const { return moduleBase_; }

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

    /**
     * Read data from emulated memory into vector
     */
    std::vector<uint8_t> readData(uint32_t address, size_t size);

private:
    uc_engine* uc_;                  // Unicorn engine instance
    uint32_t moduleBase_;            // Module base address
    uint32_t moduleSize_;            // Module size
    uint32_t stackBase_;             // Stack base address
    uint32_t stackSize_;             // Stack size
    uint32_t heapBase_;              // Heap base address
    uint32_t heapSize_;              // Heap size
    uint32_t apiStubBase_;           // API stub base address

    // API hooks: DLL name -> Function name -> Handler
    std::map<std::string, std::map<std::string, uint32_t>> apiAddresses_;

    // Memory allocation tracking
    std::map<uint32_t, size_t> allocations_;
    std::map<uint32_t, size_t> freeBlocks_; // Free-list blocks keyed by base address
    uint32_t nextHeapAddr_;

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
