#pragma once

// Cross-platform socket abstractions for Windows (Winsock2) and POSIX.

#ifdef _WIN32
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #pragma comment(lib, "ws2_32.lib")

  using socket_t  = SOCKET;
#ifndef __MINGW32__
  using ssize_t   = int;             // recv/send return int on MSVC
#endif

  inline constexpr socket_t INVALID_SOCK = INVALID_SOCKET;

#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <netdb.h>
  #include <unistd.h>
  #include <fcntl.h>
  #include <cerrno>

  using socket_t = int;

  inline constexpr socket_t INVALID_SOCK = -1;

#endif

#include <cstring>

namespace wowee {
namespace net {

// ---- Winsock lifecycle (no-op on Linux) ----

#ifdef _WIN32
struct WinsockInit {
    WinsockInit() {
        WSADATA wsa;
        WSAStartup(MAKEWORD(2, 2), &wsa);
    }
    ~WinsockInit() { WSACleanup(); }
};
// Call once at program start (e.g. as a static in Application).
inline void ensureInit() {
    static WinsockInit instance;
}
#else
inline void ensureInit() {}
#endif

// ---- Portable helpers ----

inline void closeSocket(socket_t s) {
#ifdef _WIN32
    closesocket(s);
#else
    close(s);
#endif
}

inline bool setNonBlocking(socket_t s) {
#ifdef _WIN32
    u_long mode = 1;
    return ioctlsocket(s, FIONBIO, &mode) == 0;
#else
    int flags = fcntl(s, F_GETFL, 0);
    return fcntl(s, F_SETFL, flags | O_NONBLOCK) != -1;
#endif
}

inline int lastError() {
#ifdef _WIN32
    return WSAGetLastError();
#else
    return errno;
#endif
}

inline bool isWouldBlock(int err) {
#ifdef _WIN32
    return err == WSAEWOULDBLOCK;
#else
    return err == EAGAIN || err == EWOULDBLOCK;
#endif
}

inline bool isInProgress(int err) {
#ifdef _WIN32
    return err == WSAEWOULDBLOCK || err == WSAEALREADY;
#else
    return err == EINPROGRESS;
#endif
}

inline const char* errorString(int err) {
#ifdef _WIN32
    // Simple thread-local buffer for FormatMessage
    thread_local char buf[256];
    FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                   nullptr, err, 0, buf, sizeof(buf), nullptr);
    return buf;
#else
    return strerror(err);
#endif
}

// Portable send — Windows recv/send take char*, not void*.
inline ssize_t portableSend(socket_t s, const uint8_t* data, size_t len) {
    return ::send(s, reinterpret_cast<const char*>(data), static_cast<int>(len), 0);
}

inline ssize_t portableRecv(socket_t s, uint8_t* buf, size_t len) {
    return ::recv(s, reinterpret_cast<char*>(buf), static_cast<int>(len), 0);
}

} // namespace net
} // namespace wowee
