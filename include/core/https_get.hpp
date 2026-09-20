#pragma once

/// One HTTPS GET, blocking, for the small JSON documents this client asks
/// public APIs for.
///
/// Not a general HTTP client and not meant to become one: no keep-alive, no
/// POST, no cookies, no proxy. The client had no way to make an HTTPS request
/// at all, and the one thing it needs - asking GitHub what the newest release
/// is - does not justify taking on a dependency when OpenSSL is already
/// linked for the auth path.
///
/// Call it off the render thread. It blocks on connect, handshake and read.

#include <string>

namespace wowee::core {

struct HttpsResponse {
    /// False when the request never completed - DNS, connect, TLS or a read
    /// failed. `error` says which; `status` and `body` are then empty.
    bool ok = false;
    /// HTTP status, e.g. 200 or 404. Zero when `ok` is false.
    int status = 0;
    std::string body;
    std::string error;
};

/// GET https://<host><path>, following no redirects.
///
/// `userAgent` is not optional in practice: GitHub's API rejects a request
/// without one. `timeoutSeconds` covers each socket operation rather than the
/// whole exchange, which is enough to stop a launch hanging on a dead host.
HttpsResponse httpsGet(const std::string& host, const std::string& path,
                       const std::string& userAgent, int timeoutSeconds = 8);

}  // namespace wowee::core
