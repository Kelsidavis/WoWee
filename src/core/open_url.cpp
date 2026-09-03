#include "core/open_url.hpp"

#if defined(_WIN32)
// Guarded, because the build already defines it: CMakeLists adds
// WIN32_LEAN_AND_MEAN to add_compile_definitions, and redefining it is an
// error under -Werror. The five other places that reach for windows.h all
// test first; this was the one that did not.
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <windows.h>
#  include <shellapi.h>
#elif defined(__APPLE__)
#  include <spawn.h>
#  include <sys/wait.h>
extern char** environ;
#else
#  include <spawn.h>
#  include <sys/wait.h>
extern char** environ;
#endif

namespace wowee {
namespace core {

// Open a URL in the user's default browser without invoking a shell -
// chat messages are remote-attacker-controlled, so the previous
// `system("xdg-open '" + url + "' &")` was a command-injection sink
// (a URL containing `'; rm -rf ~; '` would execute arbitrary commands).
//
// Refuses anything that isn't a plain http(s):// URL with safe ASCII -
// no shell metacharacters, no control bytes, no embedded NULs.
bool openExternalUrl(const std::string& url) {
    if (url.empty() || url.size() > 2048) return false;
    // Whitelist scheme to defang file:// / javascript: / data: etc.
    bool isHttp  = url.compare(0, 7, "http://") == 0;
    bool isHttps = url.compare(0, 8, "https://") == 0;
    if (!isHttp && !isHttps) return false;
    // Reject control bytes and the obvious shell-meta set so even a
    // future shell-using opener can't be tricked.
    for (unsigned char c : url) {
        if (c < 0x20 || c == 0x7F) return false;
        if (c == '\'' || c == '"' || c == '`' || c == '\\' ||
            c == '$'  || c == ';'  || c == '|' || c == '&'  ||
            c == '<'  || c == '>'  || c == '\n' || c == '\r') {
            return false;
        }
    }

#if defined(_WIN32)
    HINSTANCE rc = ShellExecuteA(nullptr, "open", url.c_str(),
                                 nullptr, nullptr, SW_SHOWNORMAL);
    return reinterpret_cast<INT_PTR>(rc) > 32;
#else
#  if defined(__APPLE__)
    const char* opener = "/usr/bin/open";
#  else
    const char* opener = "/usr/bin/xdg-open";
#  endif
    char* argv[] = {
        const_cast<char*>(opener),
        const_cast<char*>(url.c_str()),
        nullptr
    };
    pid_t pid = 0;
    int rc = posix_spawn(&pid, opener, nullptr, nullptr, argv, environ);
    if (rc != 0) return false;
    // Don't block the UI thread on the browser launch; reap async.
    int status = 0;
    waitpid(pid, &status, WNOHANG);
    return true;
#endif
}

} // namespace core
} // namespace wowee
