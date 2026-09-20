#pragma once

/// Whether a newer WoWee has been released, asked of GitHub once at startup.
///
/// Telling somebody a fix exists is most of the value of having made it: a
/// bug reported against a version that was corrected weeks ago costs both
/// sides the whole exchange to discover that. This only ever reads - it
/// downloads nothing and installs nothing, and says where the release is so
/// the person decides.
///
/// Off unless the setting says otherwise, because it is a request to a third
/// party made on the player's connection.

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

namespace wowee::core {

class UpdateCheck {
public:
    ~UpdateCheck();

    /// Ask, on a thread of its own. Returns immediately. Does nothing when
    /// the check is switched off, when one has already run this session, or
    /// when this build has no version to compare against.
    void start();

    /// The newer version's tag, or empty when there is none, the check has
    /// not finished, or it failed. Safe to call every frame.
    [[nodiscard]] std::string newerVersion() const;

    /// Where to get it. Empty until there is something to get.
    [[nodiscard]] std::string releaseUrl() const;

    /// True once the answer is in, whatever the answer was.
    [[nodiscard]] bool finished() const { return finished_.load(); }

private:
    void run();

    mutable std::mutex mutex_;
    std::string newerVersion_;
    std::string releaseUrl_;

    std::thread thread_;
    std::atomic<bool> started_{false};
    std::atomic<bool> finished_{false};
};

/// Compare two `vX.Y.Z` tags, ignoring anything git describe appended.
///
/// Exposed for the tests: the ordering is the whole of the decision, and a
/// wrong answer either nags somebody who is current or silences the notice
/// for somebody who is not.
/// @return true when `candidate` is newer than `current`.
[[nodiscard]] bool isNewerVersion(const std::string& candidate, const std::string& current);

}  // namespace wowee::core
