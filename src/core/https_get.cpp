#include "core/https_get.hpp"

#include <openssl/err.h>
#include <openssl/ssl.h>
#include <openssl/x509v3.h>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <memory>
#include <vector>

#include "network/net_platform.hpp"

namespace wowee::core {
namespace {

/// OpenSSL's own error text for the failure on top of the thread's queue.
std::string sslError() {
    const unsigned long code = ERR_get_error();
    if (code == 0) return "no detail";
    char buf[256];
    ERR_error_string_n(code, buf, sizeof(buf));
    return buf;
}

/// Connect a plain TCP socket, with the timeout applied to the socket rather
/// than to connect() itself - the handshake and the reads that follow are
/// where a wedged host actually costs, and SO_RCVTIMEO covers those.
socket_t connectTcp(const std::string& host, const std::string& port,
                    int timeoutSeconds, std::string* error) {
    net::ensureInit();

    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    addrinfo* found = nullptr;
    if (getaddrinfo(host.c_str(), port.c_str(), &hints, &found) != 0 || found == nullptr) {
        *error = "cannot resolve " + host;
        return INVALID_SOCK;
    }

    socket_t sock = INVALID_SOCK;
    for (addrinfo* at = found; at != nullptr; at = at->ai_next) {
        sock = ::socket(at->ai_family, at->ai_socktype, at->ai_protocol);
        if (sock == INVALID_SOCK) continue;

#ifdef _WIN32
        const DWORD timeout = static_cast<DWORD>(timeoutSeconds) * 1000;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<const char*>(&timeout), sizeof(timeout));
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO,
                   reinterpret_cast<const char*>(&timeout), sizeof(timeout));
#else
        timeval timeout{};
        timeout.tv_sec = timeoutSeconds;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
#endif

        if (::connect(sock, at->ai_addr, static_cast<int>(at->ai_addrlen)) == 0) break;
        net::closeSocket(sock);
        sock = INVALID_SOCK;
    }
    freeaddrinfo(found);

    if (sock == INVALID_SOCK) *error = "cannot reach " + host;
    return sock;
}

/// Where the headers stop. Returns npos when the terminator has not arrived.
std::size_t headerEnd(const std::string& raw) {
    return raw.find("\r\n\r\n");
}

/// A header's value, matched without regard to case as HTTP requires.
std::string headerValue(const std::string& headers, const std::string& name) {
    std::string lowerHeaders = headers;
    std::transform(lowerHeaders.begin(), lowerHeaders.end(), lowerHeaders.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    std::string lowerName = name;
    std::transform(lowerName.begin(), lowerName.end(), lowerName.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    std::size_t at = lowerHeaders.find("\r\n" + lowerName + ":");
    if (at == std::string::npos) return {};
    at += 2 + lowerName.size() + 1;
    const std::size_t end = headers.find("\r\n", at);
    std::string value = headers.substr(at, end == std::string::npos ? end : end - at);
    while (!value.empty() && (value.front() == ' ' || value.front() == '\t')) value.erase(0, 1);
    while (!value.empty() && (value.back() == ' ' || value.back() == '\r')) value.pop_back();
    return value;
}

/// Undo Transfer-Encoding: chunked. GitHub answers this way often enough that
/// leaving it out would make the reply unparseable at random.
std::string dechunk(const std::string& body) {
    std::string out;
    std::size_t at = 0;
    while (at < body.size()) {
        const std::size_t lineEnd = body.find("\r\n", at);
        if (lineEnd == std::string::npos) break;
        const std::size_t size =
            static_cast<std::size_t>(std::strtoul(body.substr(at, lineEnd - at).c_str(), nullptr, 16));
        if (size == 0) break;
        const std::size_t start = lineEnd + 2;
        if (start + size > body.size()) break;
        out.append(body, start, size);
        at = start + size + 2;   // past the chunk and its trailing CRLF
    }
    return out;
}

}  // namespace

HttpsResponse httpsGet(const std::string& host, const std::string& path,
                       const std::string& userAgent, int timeoutSeconds) {
    HttpsResponse result;

    socket_t sock = connectTcp(host, "443", timeoutSeconds, &result.error);
    if (sock == INVALID_SOCK) return result;

    // Unique_ptr rather than a goto chain: every failure below has to close
    // the socket and free the TLS state, and there are six of them.
    const std::unique_ptr<void, void (*)(void*)> socketGuard(
        reinterpret_cast<void*>(static_cast<std::intptr_t>(sock)),
        [](void* s) { net::closeSocket(static_cast<socket_t>(reinterpret_cast<std::intptr_t>(s))); });

    SSL_CTX* ctx = SSL_CTX_new(TLS_client_method());
    if (ctx == nullptr) {
        result.error = "no TLS context: " + sslError();
        return result;
    }
    const std::unique_ptr<SSL_CTX, void (*)(SSL_CTX*)> ctxGuard(ctx, SSL_CTX_free);

    // Verify the chain, and verify the name on it. SSL_set1_host is what
    // makes the second happen - without it a valid certificate for any host
    // at all would satisfy the first, which is most of the point of asking.
    SSL_CTX_set_verify(ctx, SSL_VERIFY_PEER, nullptr);
    if (SSL_CTX_set_default_verify_paths(ctx) != 1) {
        result.error = "no trust store: " + sslError();
        return result;
    }
    SSL_CTX_set_min_proto_version(ctx, TLS1_2_VERSION);

    SSL* ssl = SSL_new(ctx);
    if (ssl == nullptr) {
        result.error = "no TLS session: " + sslError();
        return result;
    }
    const std::unique_ptr<SSL, void (*)(SSL*)> sslGuard(ssl, SSL_free);

    SSL_set_tlsext_host_name(ssl, host.c_str());   // SNI
    SSL_set1_host(ssl, host.c_str());              // and check the name matches
    SSL_set_fd(ssl, static_cast<int>(sock));

    if (SSL_connect(ssl) != 1) {
        result.error = "TLS handshake failed: " + sslError();
        return result;
    }

    const std::string request =
        "GET " + path + " HTTP/1.1\r\n"
        "Host: " + host + "\r\n"
        "User-Agent: " + userAgent + "\r\n"
        "Accept: application/vnd.github+json\r\n"
        "Connection: close\r\n"
        "\r\n";
    if (SSL_write(ssl, request.data(), static_cast<int>(request.size())) <= 0) {
        result.error = "could not send the request: " + sslError();
        return result;
    }

    std::string raw;
    // A release document is a few kilobytes. The cap is there so a hostile or
    // broken endpoint cannot make this read until it runs out of memory.
    constexpr std::size_t kMaxBytes = 4u * 1024u * 1024u;
    char buffer[8192];
    while (raw.size() < kMaxBytes) {
        const int got = SSL_read(ssl, buffer, sizeof(buffer));
        if (got <= 0) break;   // close_notify, or the timeout expired
        raw.append(buffer, static_cast<std::size_t>(got));
    }

    const std::size_t split = headerEnd(raw);
    if (split == std::string::npos) {
        result.error = "no headers in the reply";
        return result;
    }
    const std::string headers = raw.substr(0, split);
    std::string body = raw.substr(split + 4);

    if (headers.rfind("HTTP/", 0) != 0) {
        result.error = "not an HTTP reply";
        return result;
    }
    const std::size_t firstSpace = headers.find(' ');
    if (firstSpace == std::string::npos) {
        result.error = "no status in the reply";
        return result;
    }
    result.status = std::atoi(headers.c_str() + firstSpace + 1);

    if (headerValue(headers, "Transfer-Encoding").find("chunked") != std::string::npos) {
        body = dechunk(body);
    }

    result.ok = true;
    result.body = std::move(body);
    return result;
}

}  // namespace wowee::core
