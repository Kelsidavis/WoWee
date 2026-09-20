#include "core/update_check.hpp"

#include <nlohmann/json.hpp>

#include <array>
#include <cstdlib>
#include <fstream>
#include <string>

#include "core/https_get.hpp"
#include "core/logger.hpp"
#include "core/config_paths.hpp"
#include "core/version.hpp"

namespace wowee::core {
namespace {

constexpr const char* kHost = "api.github.com";
constexpr const char* kPath = "/repos/Kelsidavis/WoWee/releases/latest";

/// Read the toggle straight out of settings.cfg.
///
/// The settings panel owns the field and writes the file; reading it back
/// here rather than asking the panel keeps core from depending on the
/// interface for a value it needs before any of it is up. Absent means on,
/// which is the default the panel starts the field at.
bool wantsUpdateCheck() {
    std::ifstream in(getConfigRoot() + "/settings.cfg");
    if (!in.is_open()) return true;
    std::string line;
    while (std::getline(in, line)) {
        const std::size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        // The file spells it with underscores, as it spells the rest of
        // its bool rows; the schema key it is bound to has none.
        if (line.substr(0, eq) != "check_for_updates") continue;
        return line.substr(eq + 1) != "0";
    }
    return true;
}

/// The numbers out of a tag, stopping at the first thing that is not one.
///
/// `git describe` gives "v3.1.30" on a tagged build and "v3.1.30-7-gabc1234"
/// seven commits later; both are the 3.1.30 line as far as this is concerned,
/// and the suffix is handled by the caller rather than here.
std::array<long, 3> versionParts(const std::string& tag) {
    std::array<long, 3> parts{0, 0, 0};
    std::size_t at = (!tag.empty() && (tag[0] == 'v' || tag[0] == 'V')) ? 1 : 0;
    for (long& part : parts) {
        if (at >= tag.size() || std::isdigit(static_cast<unsigned char>(tag[at])) == 0) break;
        char* end = nullptr;
        part = std::strtol(tag.c_str() + at, &end, 10);
        at = static_cast<std::size_t>(end - tag.c_str());
        if (at < tag.size() && tag[at] == '.') ++at;
    }
    return parts;
}

}  // namespace

bool isNewerVersion(const std::string& candidate, const std::string& current) {
    if (candidate.empty() || current.empty()) return false;

    const std::array<long, 3> them = versionParts(candidate);
    const std::array<long, 3> us = versionParts(current);
    if (them != us) return them > us;

    // Same numbers, so there is nothing newer to go and get. Either this is
    // exactly that release, or it is a build with commits on top of the tag,
    // which is ahead of the release rather than behind it - somebody running
    // their own build of master should not be sent to download what they are
    // already past.
    return false;
}

UpdateCheck::~UpdateCheck() {
    if (thread_.joinable()) thread_.join();
}

void UpdateCheck::start() {
    if (started_.exchange(true)) return;   // once a session

    if (!wantsUpdateCheck()) {
        finished_.store(true);
        return;
    }
    // A build with no tag behind it has nothing to compare, and every release
    // would look newer than it.
    if (std::string(kVersion).empty() || std::string(kVersion) == "unknown") {
        finished_.store(true);
        return;
    }

    thread_ = std::thread(&UpdateCheck::run, this);
}

void UpdateCheck::run() {
    const std::string agent = std::string("WoWee/") + kVersion;
    const HttpsResponse reply = httpsGet(kHost, kPath, agent);

    if (!reply.ok) {
        // Not an error the player needs to see. No connection yet, a proxy,
        // GitHub down - none of it is their problem and none of it stops the
        // client doing what they opened it for.
        // WARNING, not INFO: this build logs INFO nowhere, and a check
        // that quietly never runs is indistinguishable from one that ran
        // and found nothing. One line a session either way.
        LOG_WARNING("Update check: ", reply.error);
        finished_.store(true);
        return;
    }
    if (reply.status != 200) {
        LOG_WARNING("Update check: GitHub answered ", reply.status);
        finished_.store(true);
        return;
    }

    std::string tag;
    std::string url;
    try {
        const nlohmann::json doc = nlohmann::json::parse(reply.body);
        tag = doc.value("tag_name", std::string{});
        url = doc.value("html_url", std::string{});
    } catch (const std::exception& exc) {
        LOG_WARNING("Update check: could not read the reply - ", exc.what());
        finished_.store(true);
        return;
    }

    if (isNewerVersion(tag, kVersion)) {
        LOG_WARNING("Update available: ", tag, " (this is ", kVersion, ")");
        std::lock_guard<std::mutex> lock(mutex_);
        newerVersion_ = tag;
        releaseUrl_ = url;
    } else {
        LOG_WARNING("Update check: ", kVersion, " is current (newest is ",
                    tag.empty() ? "unknown" : tag, ")");
    }
    finished_.store(true);
}

std::string UpdateCheck::newerVersion() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return newerVersion_;
}

std::string UpdateCheck::releaseUrl() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return releaseUrl_;
}

}  // namespace wowee::core
