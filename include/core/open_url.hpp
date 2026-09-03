#pragma once

#include <string>

namespace wowee {
namespace core {

/// Open a URL in the player's browser, or answer false and do nothing.
///
/// Refuses anything that is not a plain http(s) URL of safe ASCII, and never
/// goes through a shell. The first caller was a chat link, which is text a
/// stranger on the server wrote, and that is the standard every caller is
/// held to: the About box's own address goes through the same check.
bool openExternalUrl(const std::string& url);

} // namespace core
} // namespace wowee
