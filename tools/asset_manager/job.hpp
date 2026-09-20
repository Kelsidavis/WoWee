#pragma once

/// A profile being built, on a thread of its own.
///
/// The window stays answerable while this runs, so every field the window reads
/// is behind the mutex and every field it writes is one the worker only reads.
/// Extraction happens in this process by calling Extractor::run - there is no
/// script to find, no interpreter to have, and nothing to go missing between
/// this program and the work it is asking for.

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "profiles.hpp"

namespace wowee::assets {

struct JobStage {
    std::string label;
    bool required = false;      ///< failing this one stops the rest
    bool done = false;
    bool ok = false;
    bool skipped = false;
    std::string skipReason;
};

class Job {
public:
    ~Job();

    /// Plan the stages a profile becomes, without running any of them.
    ///
    /// Separate from start() so the window can show what will happen before
    /// anybody commits to it.
    static std::vector<JobStage> plan(const Profile& profile, bool haveBorrow,
                                      bool haveLater);

    void start(const Profile& profile, std::string gameDir, std::string secondDir,
               std::string outputDir, bool haveBorrow, bool haveLater);
    void cancel();

    [[nodiscard]] bool running() const { return running_.load(); }
    [[nodiscard]] bool finished() const { return finished_.load(); }
    [[nodiscard]] bool succeeded() const { return succeeded_.load(); }

    /// A copy, because the worker is writing to the originals.
    [[nodiscard]] std::vector<JobStage> stages() const;
    [[nodiscard]] std::vector<std::string> log() const;
    [[nodiscard]] std::string currentLabel() const;
    [[nodiscard]] float progress() const;

private:
    void run(Profile profile, std::string gameDir, std::string secondDir,
             std::string outputDir, bool haveBorrow, bool haveLater);
    void say(const std::string& line);

    mutable std::mutex mutex_;
    std::vector<JobStage> stages_;
    std::vector<std::string> log_;
    std::string currentLabel_;
    std::size_t currentIndex_ = 0;

    /// How far through the stage now running, 0 to 1, for a stage that can
    /// say. Only extraction can: it is also the long one, and counting whole
    /// stages left the bar still for minutes in the middle of it.
    std::atomic<float> stageFraction_{0.0f};

    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> finished_{false};
    std::atomic<bool> succeeded_{false};
    std::atomic<bool> cancelled_{false};
};

}  // namespace wowee::assets
