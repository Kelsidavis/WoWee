#pragma once

// Write access to a MAP_JIT mapping.
//
// Apple Silicon starts every thread with JIT write protection on. A MAP_JIT
// page is executable and *not* writable until the thread asks, so `mmap`
// succeeds and the first store into it raises SIGBUS - a crash with no bad
// pointer anywhere in it, which is why it surfaces as a fault inside memset
// rather than as anything to do with the caller.
//
// Everywhere else this is nothing: x86-64 macOS needs no such call, no other
// platform has MAP_JIT at all, and an emulated build never maps one.

#include <cstddef>

// Whether an executable image is mapped MAP_JIT here. Apple's hardened runtime
// refuses an anonymous PROT_EXEC mapping without it; an emulated build copies
// the image into the emulator instead of running it, so it never asks for
// PROT_EXEC and never wants MAP_JIT. Both the mapping and the window below read
// this one answer, because a window opened over memory that is not MAP_JIT
// calls a JIT API this process was never entitled to use, and that call traps
// the process rather than returning a failure.
#if defined(__APPLE__) && !defined(HAVE_UNICORN)
    #define WOWEE_MAP_JIT 1
#endif

#if defined(WOWEE_MAP_JIT) && (defined(__aarch64__) || defined(__arm64__))
    #include <pthread.h>
    #define WOWEE_JIT_WRITE_PROTECT 1
#endif

namespace wowee {
namespace game {

/// Writable for as long as this object is alive.
///
/// Nesting-safe on purpose. The sites that write the Warden module image run
/// one after another today, and a guard that re-armed protection when the
/// first inner scope closed would be a trap for whoever nests them tomorrow.
class JitWriteWindow {
public:
    JitWriteWindow() {
#ifdef WOWEE_JIT_WRITE_PROTECT
        if (depth_++ == 0) pthread_jit_write_protect_np(0);
#endif
    }
    ~JitWriteWindow() {
#ifdef WOWEE_JIT_WRITE_PROTECT
        if (--depth_ == 0) pthread_jit_write_protect_np(1);
#endif
    }
    JitWriteWindow(const JitWriteWindow&) = delete;
    JitWriteWindow& operator=(const JitWriteWindow&) = delete;

    /// Whether this platform needs the window at all, so a test can say which
    /// behaviour it is checking rather than guessing from the platform.
    [[nodiscard]] static constexpr bool required() {
#ifdef WOWEE_JIT_WRITE_PROTECT
        return true;
#else
        return false;
#endif
    }

#ifdef WOWEE_JIT_WRITE_PROTECT
    /// How many windows are open on this thread. Nothing outside a test needs
    /// this; it is how the nesting rule above is checked.
    [[nodiscard]] static int openWindows() { return depth_; }

private:
    static thread_local int depth_;
#endif
};

#ifdef WOWEE_JIT_WRITE_PROTECT
inline thread_local int JitWriteWindow::depth_ = 0;
#endif

}  // namespace game
}  // namespace wowee
