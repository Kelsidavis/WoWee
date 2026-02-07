#pragma once

// Cross-platform subprocess helpers for spawning ffplay (audio playback).
// Linux: fork/exec/kill/waitpid.  Windows: CreateProcess/TerminateProcess.

#include <string>
#include <vector>

#ifdef _WIN32
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #include <windows.h>

  using ProcessHandle = HANDLE;
  inline constexpr ProcessHandle INVALID_PROCESS = INVALID_HANDLE_VALUE;

#else
  #include <sys/types.h>
  #include <sys/wait.h>
  #include <unistd.h>
  #include <csignal>

  using ProcessHandle = pid_t;
  inline constexpr ProcessHandle INVALID_PROCESS = -1;

#endif

#include <filesystem>

namespace wowee {
namespace platform {

// Return a platform-appropriate temp file path for the given filename.
inline std::string getTempFilePath(const std::string& filename) {
    auto tmp = std::filesystem::temp_directory_path() / filename;
    return tmp.string();
}

// Spawn ffplay with the given arguments. Returns process handle.
// args should be the full argument list (e.g. {"-nodisp", "-autoexit", ...}).
// The executable "ffplay" is resolved from PATH.
inline ProcessHandle spawnProcess(const std::vector<std::string>& args) {
#ifdef _WIN32
    // Build command line string
    std::string cmdline = "ffplay";
    for (const auto& arg : args) {
        cmdline += " ";
        // Quote arguments that contain spaces
        if (arg.find(' ') != std::string::npos) {
            cmdline += "\"" + arg + "\"";
        } else {
            cmdline += arg;
        }
    }

    STARTUPINFOA si{};
    si.cb = sizeof(si);
    // Hide the subprocess window and suppress stdout/stderr
    si.dwFlags = STARTF_USESHOWWINDOW | STARTF_USESTDHANDLES;
    si.wShowWindow = SW_HIDE;
    si.hStdInput  = INVALID_HANDLE_VALUE;
    si.hStdOutput = INVALID_HANDLE_VALUE;
    si.hStdError  = INVALID_HANDLE_VALUE;

    PROCESS_INFORMATION pi{};

    // CreateProcessA needs a mutable char buffer for lpCommandLine
    std::vector<char> cmdBuf(cmdline.begin(), cmdline.end());
    cmdBuf.push_back('\0');

    BOOL ok = CreateProcessA(
        nullptr,          // lpApplicationName — resolve from PATH
        cmdBuf.data(),    // lpCommandLine
        nullptr, nullptr, // process/thread security
        FALSE,            // inherit handles
        CREATE_NO_WINDOW, // creation flags
        nullptr, nullptr, // environment, working dir
        &si, &pi
    );

    if (!ok) {
        return INVALID_PROCESS;
    }

    // We don't need the thread handle
    CloseHandle(pi.hThread);
    return pi.hProcess;

#else
    pid_t pid = fork();
    if (pid == 0) {
        // Child process
        setpgid(0, 0);
        freopen("/dev/null", "w", stdout);
        freopen("/dev/null", "w", stderr);

        // Build argv for exec
        std::vector<const char*> argv;
        argv.push_back("ffplay");
        for (const auto& arg : args) {
            argv.push_back(arg.c_str());
        }
        argv.push_back(nullptr);

        execvp("ffplay", const_cast<char* const*>(argv.data()));
        _exit(1); // exec failed
    }
    return (pid > 0) ? pid : INVALID_PROCESS;
#endif
}

// Kill a subprocess (and its children on Linux).
inline void killProcess(ProcessHandle& handle) {
    if (handle == INVALID_PROCESS) return;

#ifdef _WIN32
    TerminateProcess(handle, 0);
    WaitForSingleObject(handle, 2000);
    CloseHandle(handle);
#else
    kill(-handle, SIGTERM);  // kill process group
    kill(handle, SIGTERM);
    int status = 0;
    // Non-blocking wait with SIGKILL fallback after ~200ms
    for (int i = 0; i < 20; ++i) {
        pid_t ret = waitpid(handle, &status, WNOHANG);
        if (ret != 0) break;  // exited or error
        usleep(10000);         // 10ms
    }
    // If still alive, force kill
    if (waitpid(handle, &status, WNOHANG) == 0) {
        kill(-handle, SIGKILL);
        kill(handle, SIGKILL);
        waitpid(handle, &status, 0);
    }
#endif

    handle = INVALID_PROCESS;
}

// Check if a process has exited. If so, clean up and set handle to INVALID_PROCESS.
// Returns true if the process is still running.
inline bool isProcessRunning(ProcessHandle& handle) {
    if (handle == INVALID_PROCESS) return false;

#ifdef _WIN32
    DWORD result = WaitForSingleObject(handle, 0);
    if (result == WAIT_OBJECT_0) {
        // Process has exited
        CloseHandle(handle);
        handle = INVALID_PROCESS;
        return false;
    }
    return true;
#else
    int status = 0;
    pid_t result = waitpid(handle, &status, WNOHANG);
    if (result == handle) {
        handle = INVALID_PROCESS;
        return false;
    }
    return true;
#endif
}

} // namespace platform
} // namespace wowee
