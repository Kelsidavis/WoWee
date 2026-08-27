// Writing to a MAP_JIT mapping, which on Apple Silicon needs asking first.
//
// A MAP_JIT page is executable and not writable until the thread turns JIT
// write protection off, so `mmap` succeeds and the *first store* raises SIGBUS.
// That is what took the Warden module loader down on arm64 Macs: the module
// image is mmap'd MAP_JIT and then zeroed, and the zeroing died - reported as a
// fault inside memset, with no bad pointer in it and nothing naming Warden.
//
// Found by fuzzing WardenModule::load with bytes a server could send, which
// crashed on every seed and stopped crashing with the window in place.
//
// This case is the mechanism rather than the loader: mmap the same way, write
// the same way, and assert the write lands. Without JitWriteWindow it does not
// fault gracefully - it takes the process out - so a regression here is a dead
// test binary, which is the loudest a test can be.
#include <catch_amalgamated.hpp>

#include "game/jit_write.hpp"

#include <cstdint>
#include <cstring>
#include <vector>

#ifndef _WIN32
#include <sys/mman.h>
#endif

using wowee::game::JitWriteWindow;

#if !defined(_WIN32) && defined(MAP_JIT)

TEST_CASE("a MAP_JIT mapping takes a write inside the window", "[jit]") {
    const size_t size = 4096;
    void* page = mmap(nullptr, size, PROT_READ | PROT_WRITE | PROT_EXEC,
                      MAP_PRIVATE | MAP_ANONYMOUS | MAP_JIT, -1, 0);
    if (page == MAP_FAILED) {
        // Hardened runtime without the JIT entitlement refuses the mapping
        // outright. Nothing to check, and nothing broken.
        SUCCEED("MAP_JIT is not available to this binary");
        return;
    }

    {
        JitWriteWindow jitWrite;
        std::memset(page, 0, size);
        auto* bytes = static_cast<uint8_t*>(page);
        for (size_t i = 0; i < 16; ++i) bytes[i] = static_cast<uint8_t>(i + 1);
    }

    // Read back with protection re-armed: reading was never the restricted half.
    const auto* bytes = static_cast<const uint8_t*>(page);
    for (size_t i = 0; i < 16; ++i) CHECK(bytes[i] == static_cast<uint8_t>(i + 1));

    munmap(page, size);
}

TEST_CASE("the window nests without re-arming early", "[jit]") {
    const size_t size = 4096;
    void* page = mmap(nullptr, size, PROT_READ | PROT_WRITE | PROT_EXEC,
                      MAP_PRIVATE | MAP_ANONYMOUS | MAP_JIT, -1, 0);
    if (page == MAP_FAILED) {
        SUCCEED("MAP_JIT is not available to this binary");
        return;
    }

    auto* bytes = static_cast<uint8_t*>(page);
    {
        JitWriteWindow outer;
        {
            JitWriteWindow inner;
            bytes[0] = 0xAB;
        }
        // The inner window closing must not have re-armed protection: the
        // outer one is still open, and this store is the half that would die.
        bytes[1] = 0xCD;
    }
    CHECK(bytes[0] == 0xAB);
    CHECK(bytes[1] == 0xCD);

    munmap(page, size);
}

#endif  // !_WIN32 && MAP_JIT

TEST_CASE("the window follows the mapping, not the platform", "[jit]") {
    // The window costs nothing where it is needed and takes the process out
    // where it is not: pthread_jit_write_protect_np answers a process without
    // com.apple.security.cs.allow-jit by trapping, not by failing. So it must
    // open only where the image is actually mapped MAP_JIT.
    //
    // 3.1.9 crashed every arm64 Mac at login on exactly that gap. The window
    // was gated on arm64 macOS while the mapping was gated on macOS *and* no
    // Unicorn, and the release build has Unicorn: no MAP_JIT page existed, and
    // the first Warden module the server sent trapped in the constructor.
    // Reported in #131. Both read WOWEE_MAP_JIT now.
#ifdef WOWEE_MAP_JIT
    SUCCEED("the module image is mapped MAP_JIT here");
#else
    CHECK_FALSE(JitWriteWindow::required());
#endif
}

TEST_CASE("the window closes back to none", "[jit]") {
    // Whatever the platform, a balanced set of windows leaves none open. On
    // anything but arm64 macOS this is all the class does.
    if constexpr (JitWriteWindow::required()) {
#ifdef WOWEE_JIT_WRITE_PROTECT
        CHECK(JitWriteWindow::openWindows() == 0);
        {
            JitWriteWindow one;
            CHECK(JitWriteWindow::openWindows() == 1);
            {
                JitWriteWindow two;
                CHECK(JitWriteWindow::openWindows() == 2);
            }
            CHECK(JitWriteWindow::openWindows() == 1);
        }
        CHECK(JitWriteWindow::openWindows() == 0);
#endif
    } else {
        JitWriteWindow noop;
        SUCCEED("no JIT write protection on this platform");
    }
}
