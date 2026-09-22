#include "job.hpp"

#include <algorithm>
#include <atomic>
#include <filesystem>
#include <memory>

#include "../asset_extract/extractor.hpp"
#include "casc.hpp"
#include "model_import.hpp"
#include "upscale.hpp"

namespace wowee::assets {
namespace {

namespace fs = std::filesystem;

/// How much more geometry a later model needs before it is worth taking. Below
/// this the two are the same model, and swapping one for the other only risks
/// the conversion going wrong for no gain.
constexpr float kBetterRatio = 1.3f;

std::string kindLabel(const Step& step) {
    switch (step.kind) {
        case Kind::Extract:      return step.summary;
        case Kind::Upscale:      return step.summary;
        case Kind::ImportModels: return step.summary;
    }
    return step.summary;
}

}  // namespace

Job::~Job() {
    cancel();
    if (thread_.joinable()) thread_.join();
}

std::vector<JobStage> Job::plan(const Profile& profile, bool haveBorrow, bool haveLater) {
    std::vector<JobStage> out;
    for (const Step& step : profile.steps) {
        JobStage stage;
        stage.label = kindLabel(step);
        // Extraction is the only step whose failure makes the rest meaningless.
        // The others are enrichments: a pass that fails should say so and leave
        // a complete, playable extraction standing rather than discard it.
        stage.required = (step.kind == Kind::Extract);

        if (step.source == Source::Borrow && !haveBorrow) {
            stage.skipped = true;
            stage.skipReason = "no second installation to borrow from";
        } else if (step.source == Source::Later && !haveLater) {
            stage.skipped = true;
            stage.skipReason = "no later installation to take models from";
        }
        out.push_back(std::move(stage));
    }
    return out;
}

void Job::start(const Profile& profile, std::string gameDir, std::string secondDir,
                std::string outputDir, bool haveBorrow, bool haveLater) {
    if (running_.load()) return;
    if (thread_.joinable()) thread_.join();

    {
        std::lock_guard<std::mutex> lock(mutex_);
        stages_ = plan(profile, haveBorrow, haveLater);
        log_.clear();
        currentIndex_ = 0;
        currentLabel_.clear();
    }
    running_.store(true);
    finished_.store(false);
    succeeded_.store(false);
    cancelled_.store(false);

    thread_ = std::thread(&Job::run, this, profile, std::move(gameDir),
                          std::move(secondDir), std::move(outputDir),
                          haveBorrow, haveLater);
}

void Job::cancel() { cancelled_.store(true); }

void Job::say(const std::string& line) {
    std::lock_guard<std::mutex> lock(mutex_);
    log_.push_back(line);
}

std::vector<JobStage> Job::stages() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stages_;
}

std::vector<std::string> Job::log() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return log_;
}

std::string Job::currentLabel() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return currentLabel_;
}

float Job::progress() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stages_.empty()) return 0.0f;
    std::size_t done = 0;
    for (const JobStage& stage : stages_) done += stage.done ? 1 : 0;
    const float whole = 1.0f / static_cast<float>(stages_.size());
    // Whole stages, plus however far into the one running. Without the
    // second term the bar sat on a stage boundary for the length of an
    // extraction - minutes, on a real game - and read as a hang.
    const float within = std::clamp(stageFraction_.load(), 0.0f, 1.0f) * whole;
    return static_cast<float>(done) * whole + within;
}

void Job::run(Profile profile, std::string gameDir, std::string secondDir,
              std::string outputDir, bool haveBorrow, bool haveLater) {
    (void)secondDir;
    (void)haveBorrow;
    (void)haveLater;

    bool allOk = true;
    const std::size_t total = stages().size();

    for (std::size_t i = 0; i < total; ++i) {
        if (cancelled_.load()) {
            say("Stopped.");
            allOk = false;
            break;
        }

        JobStage stage;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stage = stages_[i];
            currentIndex_ = i;
            currentLabel_ = stage.label;
        }

        // Each stage starts from nothing; only extraction ever moves it.
        stageFraction_.store(0.0f);

        say("");
        say("=== [" + std::to_string(i + 1) + "/" + std::to_string(total) + "] " + stage.label);

        if (stage.skipped) {
            say("    skipped: " + stage.skipReason);
            std::lock_guard<std::mutex> lock(mutex_);
            stages_[i].done = true;
            stages_[i].ok = true;   // skipping on purpose is not a failure
            continue;
        }

        bool ok = false;
        const Step& step = profile.steps[i];
        if (step.kind == Kind::Extract) {
            tools::Extractor::Options opts;
            opts.mpqDir = gameDir;
            opts.outputDir = outputDir;
            opts.expansion = step.expansion;
            opts.expansionSubdir = true;
            say("    reading " + gameDir);
            say("    writing " + outputDir + "/expansions/" + step.expansion);
            // Called from an extraction worker, so it touches the atomic
            // directly and leaves say() - which takes the lock - to the
            // occasional line.
            // Atomic because two workers can be inside this at once: each
            // value of `done` reaches it on one thread, but neighbouring
            // values can arrive on different ones at the same moment.
            auto saidAt = std::make_shared<std::atomic<std::size_t>>(0);
            opts.onProgress = [this, saidAt](std::size_t done, std::size_t totalFiles) {
                if (totalFiles == 0) return;
                stageFraction_.store(static_cast<float>(done) /
                                     static_cast<float>(totalFiles));
                // A line every few thousand files: enough that the log is
                // visibly alive over a long extraction, and rare enough that
                // it is not itself the cost. The exchange means only the
                // thread that wins writes the line.
                std::size_t want = saidAt->load();
                if (done >= want && saidAt->compare_exchange_strong(want, done + 5000)) {
                    say("    " + std::to_string(done) + " / " +
                        std::to_string(totalFiles) + " files");
                }
            };
            try {
                ok = tools::Extractor::run(opts);
            } catch (const std::exception& exc) {
                say(std::string("    failed: ") + exc.what());
                ok = false;
            }
        }

        else if (step.kind == Kind::Upscale) {
            const std::string expansionDir =
                (fs::path(outputDir) / "expansions" / profile.expansion).string();
            say("    looking at every model for the textures it draws");
            const UpscalePlan plan = planUpscale(expansionDir);
            say("    " + std::to_string(plan.foliageModels) + " foliage models, " +
                std::to_string(plan.textures.size()) + " textures to resample, " +
                std::to_string(plan.sharedSkipped) + " shared sheets left alone");
            const UpscaleResult result = runUpscale(
                plan, 4, [this](const std::string& line) { say(line); }, cancelled_);
            say("    wrote " + std::to_string(result.written) + ", skipped " +
                std::to_string(result.skipped) + " already done, " +
                std::to_string(result.failed) + " failed");
            ok = result.failed == 0 || result.written > 0;
        }

        else if (step.kind == Kind::ImportModels) {
            const std::string expansionDir =
                (fs::path(outputDir) / "expansions" / profile.expansion).string();

            // What earlier imports left behind, before adding to them.
            const RepairResult repaired = repairEarlierImports(expansionDir);
            if (repaired.modelsRepaired > 0 || repaired.leftDrawn > 0) {
                say("    " + std::to_string(repaired.namesCleared) +
                    " missing texture names cleared from " +
                    std::to_string(repaired.modelsRepaired) + " earlier imports that never draw them" +
                    (repaired.leftDrawn > 0
                         ? "; " + std::to_string(repaired.leftDrawn) +
                               " that are drawn were left as they are"
                         : std::string()));
            }

            std::unique_ptr<ModelSource> source;
            CascStorage storage;
            std::string error;
            say("    opening " + secondDir);
            if (step.source == Source::Later) {
                if (storage.open(secondDir, &error)) {
                    say("    " + std::to_string(storage.rootCount()) + " files in it");
                    source = cascSource(storage);
                }
            } else {
                source = mpqSource(secondDir, step.expansion, &error);
            }

            if (!source) {
                say("    could not read it: " + error);
                ok = false;
            } else {
                ImportResult total;
                for (const std::string& prefix : step.prefixes) {
                    if (cancelled_.load()) break;
                    say("    looking through " + prefix);
                    const ImportResult part = importModels(
                        *source, expansionDir, expansionDir, prefix, kBetterRatio,
                        [this](const std::string& line) { say(line); }, cancelled_);
                    total.written += part.written;
                    total.refusedByGate += part.refusedByGate;
                    total.missingTextures += part.missingTextures;
                    total.missingSkin += part.missingSkin;
                    total.dressedByTable += part.dressedByTable;
                    total.tableSkinsBrought += part.tableSkinsBrought;
                    total.earlierImportsDressed += part.earlierImportsDressed;
                    total.earlierImportsRemoved += part.earlierImportsRemoved;
                    total.unusedTexturesCleared += part.unusedTexturesCleared;
                    total.hasEmitters += part.hasEmitters;
                    total.notBetter += part.notBetter;
                }
                const std::size_t refused = total.refusedByGate + total.missingTextures +
                                            total.missingSkin + total.dressedByTable;
                say("    took " + std::to_string(total.written) + " models; left " +
                    std::to_string(total.notBetter) + " alone as no better, " +
                    std::to_string(total.hasEmitters) + " that emit particles, " +
                    std::to_string(refused) + " that would not have arrived whole");
                if (total.unusedTexturesCleared > 0) {
                    say("    " + std::to_string(total.unusedTexturesCleared) +
                        " texture names cleared from models that never draw them");
                }
                if (total.tableSkinsBrought > 0) {
                    say("    " + std::to_string(total.tableSkinsBrought) +
                        " creature skins brought for the later meshes that wear them");
                }
                if (total.earlierImportsDressed > 0 || total.earlierImportsRemoved > 0) {
                    say("    earlier imports: " + std::to_string(total.earlierImportsDressed) +
                        " given the skins they were missing, " +
                        std::to_string(total.earlierImportsRemoved) +
                        " removed because nothing here can dress them");
                }
                ok = total.written > 0 || total.earlierImportsDressed > 0 ||
                     total.earlierImportsRemoved > 0;
                if (!ok) say("    nothing here improves on what is already extracted");
            }
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            stages_[i].done = true;
            stages_[i].ok = ok;
        }
        say(ok ? "    done" : "    did not finish");

        if (!ok) {
            allOk = false;
            if (stage.required) {
                say("    this one is required - stopping here");
                break;
            }
            say("    carrying on; what is already extracted is kept");
        }
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        currentLabel_.clear();
    }
    succeeded_.store(allOk);
    finished_.store(true);
    running_.store(false);
}

}  // namespace wowee::assets
