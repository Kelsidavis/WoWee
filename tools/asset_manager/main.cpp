/// WoWee Asset Manager - a native window for building this client's assets.
///
///     wowee_assets [game folder] [a later client] [where to put it]
///
/// Three questions in the order somebody actually has answers for: where is the
/// game, what do you want it to look like, and go.
///
/// This is a C++ program because the thing it replaced was not: a Tkinter window
/// driving shell scripts that drove Python scripts, each needing an interpreter
/// with the right modules built into it. On one machine `python3` had no Tkinter
/// and the window simply did not appear - a perfectly good interpreter sat one
/// directory away and nothing said so. A tool that installs the game's assets
/// should not itself need an install.
///
/// It uses SDL2 and Dear ImGui, both of which the client already carries, and
/// calls Extractor::run in this process. Nothing is shelled out to.

#include <SDL.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "imgui.h"
#include "backends/imgui_impl_sdl2.h"
#include "backends/imgui_impl_sdlrenderer2.h"

// The implementation is already compiled into open_format_emitter.cpp.
#include "stb_image_write.h"

#include "core/data_paths.hpp"
#include "pipeline/asset_inventory.hpp"
#include "core/local_time.hpp"
#include "ui/imgui_theme.hpp"

#include "casc.hpp"
#include "pack.hpp"
#include "install_scan.hpp"
#include "folder_picker.hpp"
#include "job.hpp"
#include "profiles.hpp"

namespace {

namespace fs = std::filesystem;
using namespace wowee::assets;

/// ImGui's built-in face is drawn at thirteen pixels. Everything here was laid
/// out against that, so it is the height the atlas is scaled from rather than a
/// size worth changing.
constexpr float kBaseFontSize = 13.0f;

/// FRIZQT draws smaller than ImGui's built-in face at the same nominal height,
/// so asking for thirteen of it puts noticeably smaller text into controls
/// sized for thirteen. This is the height that matches.
constexpr float kFrizqtRatio = 1.25f;

/// The log pane, once a build is running.
constexpr float kLogHeight = 200.0f;

/// Everything the window remembers between frames.
struct App {
    char gameDir[1024] = {0};
    char secondDir[1024] = {0};
    char outputDir[1024] = {0};

    InstallScan gameScan;
    InstallScan secondScan;

    /// Which game, and what was ticked on top of it.
    std::string selectedBase = "wotlk";
    std::vector<std::string> chosen;
    /// Whether the base has been set from what the game folder turned out to
    /// be. Once, so that choosing otherwise is not undone on the next frame.
    bool detected = false;

    Picker picker;
    /// Which row opened the in-window browser, so its answer goes back to the
    /// field that asked. Zero when nothing is being browsed.
    int pendingPick = 0;

    Job job;
    bool started = false;
    bool cascConfirmed = false;
    std::string cascError;

    // Keeping a copy of what was built. Offered once a build finishes rather
    // than assumed, and available at any time from the button.
    std::atomic<bool> packing{false};
    std::atomic<bool> packCancel{false};
    std::string packNote;
    std::atomic<bool> importing{false};
    /// Mutable so a const read of the note can still take the lock: the note is
    /// written from the packing thread and read while laying the window out.
    mutable std::mutex packMutex;
    bool offeredSave = false;
    /// The pack or import in flight, held rather than detached.
    ///
    /// Both workers write through this App for as long as they run. Detached,
    /// closing the window during one left it writing into an App that main had
    /// already destroyed. One thread covers both because the buttons that
    /// start them are disabled while either is running.
    std::thread packThread;

    /// Same shape as ~Job: ask the worker to stop, then wait for it. The body
    /// runs before any member above is destroyed, so the thread is gone before
    /// what it writes to is.
    ~App() {
        packCancel.store(true);
        if (packThread.joinable()) packThread.join();
    }
};

void wrapped(const char* text) {
    ImGui::PushTextWrapPos(ImGui::GetContentRegionAvail().x);
    ImGui::TextUnformatted(text);
    ImGui::PopTextWrapPos();
}

void dimmed(const char* text) {
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.62f, 0.62f, 0.62f, 1.0f));
    wrapped(text);
    ImGui::PopStyleColor();
}

void rescan(App& app) {
    app.gameScan = scanInstall(app.gameDir);
    app.secondScan = scanInstall(app.secondDir);
    app.cascConfirmed = false;
    app.cascError.clear();
}

/// Actually open a CASC install, on demand.
///
/// Not while somebody is typing: reading its encoding table is around a second
/// of work and this happens on the frame a button is pressed, not on the frame
/// a character is entered.
void confirmSecond(App& app) {
    if (app.secondScan.kind != InstallKind::Casc) return;
    app.cascConfirmed = confirmCasc(app.secondScan, &app.cascError);
}

bool haveGame(const App& app)   { return app.gameScan.kind == InstallKind::Mpq; }
bool haveBorrow(const App& app) { return app.secondScan.kind == InstallKind::Mpq; }
bool haveLater(const App& app)  { return app.secondScan.kind == InstallKind::Casc; }

/// One row: a path, a Browse button that opens the system's own chooser, and
/// whatever the scan made of what is there.
///
/// The button is what people look for. Typing a path still works and dropping a
/// folder on the window still works, but neither is discoverable, and a text
/// box with no button beside it reads as the only way in.
void folderRow(App& app, const char* label, char* buffer, std::size_t size,
               PickWhat what, const char* title, Picker& picker, int& pending, int id) {
    ImGui::PushID(id);
    ImGui::TextUnformatted(label);

    const float browse = ImGui::CalcTextSize("Browse...").x + ImGui::GetStyle().FramePadding.x * 2.0f;
    ImGui::SetNextItemWidth(-(browse + ImGui::GetStyle().ItemSpacing.x));
    if (ImGui::InputText("##path", buffer, size)) rescan(app);

    ImGui::SameLine();
    if (ImGui::Button("Browse...")) {
        std::string chosen;
        if (picker.ask(what, title, buffer, "", &chosen)) {
            std::snprintf(buffer, size, "%s", chosen.c_str());
            rescan(app);
        } else {
            pending = id;   // no system chooser here; the in-window one opened
        }
    }
    ImGui::PopID();
}

/// The games already extracted into a folder, newest-looking first.
///
/// Read fresh rather than remembered: a build finishing, a pack being
/// installed and somebody choosing a different folder all change it, and a
/// cached answer is one that goes stale in each of those.
std::vector<std::string> installedGames(const std::string& dataRoot) {
    std::vector<std::string> out;
    // The same reading the client takes of the same folder, so the two cannot
    // describe it differently. An expansion built by something else, or by a
    // later version of this, still counts - it is there and the client will
    // offer it.
    for (const wowee::pipeline::AssetSet& set : wowee::pipeline::takeInventory(dataRoot).sets) {
        if (set.usable()) out.push_back(set.name);
    }
    return out;
}

/// What is in the destination, in as much detail as there is.
std::vector<std::string> installedDetail(const std::string& dataRoot) {
    std::vector<std::string> out;
    for (const wowee::pipeline::AssetSet& set : wowee::pipeline::takeInventory(dataRoot).sets) {
        // A set nobody has built is a directory of protocol definitions this
        // client ships, not something to report as present or as a problem.
        if (set.usable() || set.anyAssets) out.push_back(set.summary());
    }
    return out;
}

void drawGame(App& app) {
    ImGui::SeparatorText("1.  Where is your game?");
    folderRow(app, "The World of Warcraft folder you want to build from",
              app.gameDir, sizeof(app.gameDir), PickWhat::Folder,
              "Choose your World of Warcraft folder", app.picker, app.pendingPick, 1);

    if (!app.gameScan.note.empty()) {
        const bool good = haveGame(app);
        ImGui::PushStyleColor(ImGuiCol_Text, good ? ImVec4(0.35f, 0.78f, 0.45f, 1.0f)
                                                  : ImVec4(0.85f, 0.65f, 0.30f, 1.0f));
        wrapped(app.gameScan.note.c_str());
        ImGui::PopStyleColor();
    } else {
        dimmed("You can also drag the folder onto this window.");
    }

    ImGui::Spacing();
    folderRow(app, "Where the assets should go", app.outputDir, sizeof(app.outputDir),
              PickWhat::Folder, "Choose where to put the built assets",
              app.picker, app.pendingPick, 4);

    const std::vector<std::string> detail = installedDetail(app.outputDir);
    if (detail.empty()) {
        dimmed("Nothing built here yet. This is where the client looks for its assets.");
    } else {
        // Several games can share one folder, each under its own name, and the
        // client picks between them at its login screen. Saying what is already
        // there is what makes that visible: otherwise a second game built into
        // the same place looks like it overwrote the first.
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.35f, 0.78f, 0.45f, 1.0f));
        wrapped("Already here:");
        ImGui::PopStyleColor();
        ImGui::Indent();
        for (const std::string& line : detail) dimmed(line.c_str());
        ImGui::Unindent();
        if (installedGames(app.outputDir).size() < 2) {
            dimmed("Build another game into this same folder and you can choose between "
                   "them when the client starts.");
        } else {
            dimmed("The client offers all of them at its login screen.");
        }
    }
}

void drawBase(App& app) {
    ImGui::SeparatorText("2.  Which game does your server run?");

    if (!haveGame(app)) {
        dimmed("Choose your game folder above first.");
        return;
    }

    // Whatever they pointed at, chosen for them. They can still say otherwise -
    // a 3.3.5 server can be played with a Cataclysm client's art - but the
    // common case is that the folder they just picked is the answer.
    if (!app.detected && !app.gameScan.expansion.empty()) {
        app.selectedBase = app.gameScan.expansion;
        app.detected = true;
    }

    for (const Base& base : bases()) {
        const bool isDetected = base.id == app.gameScan.expansion;
        std::string label = base.name + "   " + base.patch;
        if (isDetected) label += "     (this is what you pointed at)";
        if (ImGui::RadioButton(label.c_str(), app.selectedBase == base.id)) {
            app.selectedBase = base.id;
        }
        ImGui::Indent();
        dimmed(base.detail.c_str());
        ImGui::Unindent();
    }
}

void drawUpgrades(App& app) {
    const Base* base = baseById(app.selectedBase);
    if (base == nullptr || !haveGame(app)) return;

    ImGui::SeparatorText("3.  Do you want updated assets?");
    dimmed("Optional. Everything here is on top of the game above, and each one can "
           "be left off.");
    ImGui::Spacing();

    // Two upgrades can want the same second installation, and asking for it
    // once per upgrade puts the same field and the same warning on screen twice
    // - which reads as two different things being needed.
    Source asked = Source::None;

    for (const Upgrade& upgrade : upgrades()) {
        if (!upgradeSuitsBase(upgrade, *base)) continue;

        std::string why;
        const bool usable = sourceAvailable(upgrade.source, haveGame(app), haveBorrow(app),
                                            haveLater(app), &why);
        auto at = std::find(app.chosen.begin(), app.chosen.end(), upgrade.id);
        bool ticked = at != app.chosen.end();

        ImGui::PushID(upgrade.id.c_str());
        ImGui::BeginDisabled(!usable);
        if (ImGui::Checkbox(upgrade.name.c_str(), &ticked)) {
            if (ticked) app.chosen.push_back(upgrade.id);
            else app.chosen.erase(at);
        }
        ImGui::EndDisabled();

        ImGui::Indent();
        dimmed(upgrade.summary.c_str());

        // The second installation is asked for here, beside the thing that
        // needs it, rather than as a second box at the top that most people do
        // not need and nothing explains.
        // The installation an upgrade needs, asked for beside it - once per
        // installation, and whether or not it is already set. Hidden as soon as
        // it is valid, there would be no way to see which one is being used or
        // to point at a different one.
        if (upgrade.source != Source::None && upgrade.source != asked) {
            asked = upgrade.source;
            if (!usable) {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.85f, 0.65f, 0.30f, 1.0f));
                wrapped(why.c_str());
                ImGui::PopStyleColor();
            }
            folderRow(app, upgrade.source == Source::Later
                               ? "Your Legion (or later) installation"
                               : "Your Cataclysm installation",
                      app.secondDir, sizeof(app.secondDir), PickWhat::Folder,
                      "Choose the installation to take art from",
                      app.picker, app.pendingPick, 2);
            if (!app.secondScan.note.empty()) dimmed(app.secondScan.note.c_str());

            if (usable && upgrade.source == Source::Later && !app.cascConfirmed) {
                if (ImGui::Button("Check it")) confirmSecond(app);
                ImGui::SameLine();
                dimmed("Reads its file table - about a second.");
            }
            if (!app.cascError.empty() && upgrade.source == Source::Later) {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.88f, 0.42f, 0.38f, 1.0f));
                wrapped(app.cascError.c_str());
                ImGui::PopStyleColor();
            }
        } else if (upgrade.source != Source::None) {
            dimmed(upgrade.source == Source::Later
                       ? "Uses the same Legion installation as above."
                       : "Uses the same Cataclysm installation as above.");
        }
        ImGui::Unindent();
        ImGui::PopID();
    }
}

void drawPlan(App& app) {
    const Base* base = baseById(app.selectedBase);
    if (base == nullptr || !haveGame(app)) return;

    const Profile profile = assemble(*base, app.chosen);
    ImGui::SeparatorText("4.  What will happen");

    const std::vector<JobStage> planned =
        Job::plan(profile, haveBorrow(app), haveLater(app));
    for (std::size_t i = 0; i < planned.size(); ++i) {
        const JobStage& stage = planned[i];
        std::string line = "  " + std::to_string(i + 1) + ". " + stage.label;
        if (stage.skipped) line += "   - skipped, " + stage.skipReason;
        if (stage.skipped) dimmed(line.c_str()); else wrapped(line.c_str());
    }
    ImGui::Spacing();
    dimmed(("About " + std::to_string(profile.minutes()) +
            " minutes. Nothing is written into your game install.").c_str());
}

void startPack(App& app, const std::string& profileId) {
    std::string out = app.outputDir;
    if (out.empty()) out = (fs::current_path() / "Data").string();

    // A pack holds whatever is in the folder, which may be several games built
    // one after another. Named after the last thing built, a pack of four games
    // says it is one - and the person receiving it has no way to tell.
    const std::vector<std::string> inside = installedGames(out);
    const std::string name = inside.size() > 1 ? "universal" : profileId;

    const std::tm local = wowee::core::localTime(std::time(nullptr));
    char stamp[16];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d", &local);
    const std::string dest =
        (fs::path(out).parent_path() / ("wowee-" + name + "-" + stamp + ".zip")).string();

    app.packing.store(true);
    app.packCancel.store(false);
    {
        std::lock_guard<std::mutex> lock(app.packMutex);
        app.packNote = "Packing...";
    }
    // Joined, not detached. The button above is disabled while either worker
    // runs, so anything still joinable here has already finished.
    if (app.packThread.joinable()) app.packThread.join();
    app.packThread = std::thread([&app, out, dest, name]() {
        PackResult result = writePack(
            out, dest, name,
            [&app](std::size_t done, std::size_t total) {
                std::lock_guard<std::mutex> lock(app.packMutex);
                app.packNote = "Packing " + std::to_string(done) + " of " +
                               std::to_string(total) + " files...";
            },
            app.packCancel);
        std::lock_guard<std::mutex> lock(app.packMutex);
        if (!result.ok) {
            app.packNote = "Could not save: " + result.error;
        } else {
            const double raw = double(result.rawBytes) / (1024.0 * 1024.0 * 1024.0);
            const double packed = double(result.packedBytes) / (1024.0 * 1024.0 * 1024.0);
            char line[512];
            std::snprintf(line, sizeof(line),
                          "Saved %zu files - %.1f GB of assets in a %.1f GB file. %s",
                          result.files, raw, packed, dest.c_str());
            app.packNote = line;
        }
        app.packing.store(false);
    });
}

/// Install a pack somebody else built, into the same tree a build writes to.
void startImport(App& app, const std::string& zipPath) {
    std::string out = app.outputDir;
    if (out.empty()) out = (fs::current_path() / "Data").string();

    const PackInfo info = readPackInfo(zipPath);
    if (!info.ok) {
        std::lock_guard<std::mutex> lock(app.packMutex);
        app.packNote = "That does not look like a pack: " + info.error;
        return;
    }

    app.importing.store(true);
    app.packCancel.store(false);
    {
        std::lock_guard<std::mutex> lock(app.packMutex);
        app.packNote = "Installing " + (info.name.empty() ? std::string("a pack") : info.name) +
                       " - " + std::to_string(info.files) + " files...";
    }
    if (app.packThread.joinable()) app.packThread.join();
    app.packThread = std::thread([&app, zipPath, out]() {
        PackResult result = readPack(
            zipPath, out,
            [&app](std::size_t done, std::size_t total) {
                std::lock_guard<std::mutex> lock(app.packMutex);
                app.packNote = "Installing " + std::to_string(done) + " of " +
                               std::to_string(total) + " files...";
            },
            app.packCancel);
        std::lock_guard<std::mutex> lock(app.packMutex);
        if (!result.ok) {
            app.packNote = "Could not install that pack: " + result.error;
        } else {
            const double raw = double(result.rawBytes) / (1024.0 * 1024.0 * 1024.0);
            char line[512];
            std::snprintf(line, sizeof(line), "Installed %zu files - %.1f GB - into %s",
                          result.files, raw, out.c_str());
            app.packNote = line;
        }
        app.importing.store(false);
    });
}

/// How much of the window the actions need reserved at the bottom.
///
/// The button row always, and the progress bar and log once there is something
/// to watch - which is the moment the log matters most and the questions above
/// matter least.
float actionsHeight(const App& app) {
    const ImGuiStyle& style = ImGui::GetStyle();
    float height = 32.0f + style.ItemSpacing.y * 2.0f + style.FramePadding.y * 2.0f;
    {
        std::lock_guard<std::mutex> lock(app.packMutex);
        if (!app.packNote.empty()) height += ImGui::GetTextLineHeightWithSpacing();
    }
    if (app.started) {
        height += ImGui::GetFrameHeightWithSpacing();        // the progress bar
        height += ImGui::GetTextLineHeightWithSpacing();     // what it is doing
        height += kLogHeight + style.ItemSpacing.y * 2.0f;
    }
    return height;
}

void drawRun(App& app) {
    const Base* base = baseById(app.selectedBase);
    const Profile profile = base != nullptr ? assemble(*base, app.chosen) : Profile{};
    const bool busy = app.job.running() || app.packing.load() || app.importing.load();

    const bool ok = base != nullptr &&
                    profileAvailable(profile, haveGame(app), haveBorrow(app),
                                     haveLater(app), nullptr);

    ImGui::BeginDisabled(!ok || busy);
    if (ImGui::Button("Build my assets", ImVec2(180, 32))) {
        std::string out = app.outputDir;
        if (out.empty()) out = (fs::current_path() / "Data").string();
        // The folder the scan found the archives in, not the one that was
        // typed. They are usually the same and were assumed to be: when they
        // were not - somebody chose the folder that has Data inside it, which
        // is what the field asks for - the panel said "Found a game" from the
        // scan and the extractor then said "No MPQ archives found in" the
        // other path, having been handed the one nobody had checked.
        const auto readFrom = [](const InstallScan& scan, const std::string& typed) {
            return scan.dataDir.empty() ? typed : scan.dataDir;
        };
        app.job.start(profile, readFrom(app.gameScan, app.gameDir),
                      readFrom(app.secondScan, app.secondDir), out,
                      haveBorrow(app), haveLater(app));
        app.started = true;
    }
    ImGui::EndDisabled();

    if (app.job.running()) {
        ImGui::SameLine();
        if (ImGui::Button("Stop", ImVec2(90, 32))) app.job.cancel();
    }

    // Installing somebody else's pack asks nothing about your game, so it is
    // not behind the questions above: a person handed a pack has no archives to
    // point at and nothing to choose.
    ImGui::SameLine();
    ImGui::BeginDisabled(busy);
    if (ImGui::Button("Install a pack...", ImVec2(170, 32))) {
        std::string chosen;
        if (app.picker.ask(PickWhat::File, "Choose a pack to install", app.outputDir,
                           ".zip", &chosen)) {
            startImport(app, chosen);
        } else {
            app.pendingPick = 3;
        }
    }
    ImGui::EndDisabled();

    // Whatever is in the folder, whether this session put it there or a run
    // last week did. Gated on having built something now, a person who came
    // back to send someone their assets found the button dead.
    const std::vector<std::string> inside = installedGames(app.outputDir);
    ImGui::SameLine();
    ImGui::BeginDisabled(busy || inside.empty());
    const std::string save = inside.size() > 1
                                 ? "Save all " + std::to_string(inside.size()) +
                                       " as one pack"
                                 : std::string("Save what I have as a pack");
    if (ImGui::Button(save.c_str(), ImVec2(260, 32))) {
        startPack(app, profile.id);
    }
    ImGui::EndDisabled();
    {
        std::lock_guard<std::mutex> lock(app.packMutex);
        if (!app.packNote.empty()) dimmed(app.packNote.c_str());
    }

    if (app.started) {
        ImGui::Spacing();
        ImGui::ProgressBar(app.job.progress(), ImVec2(-1, 0));
        const std::string label = app.job.currentLabel();
        if (!label.empty()) wrapped(label.c_str());
        else if (app.job.finished()) {
            wrapped(app.job.succeeded() ? "Done - your assets are ready."
                                        : "Finished, but some steps did not complete.");
        }

        ImGui::Spacing();
        if (ImGui::BeginChild("log", ImVec2(0, kLogHeight), ImGuiChildFlags_Borders)) {
            for (const std::string& line : app.job.log()) {
                ImGui::TextUnformatted(line.c_str());
            }
            if (app.job.running()) ImGui::SetScrollHereY(1.0f);
        }
        ImGui::EndChild();
    }
}

/// How many frames to let settle before the picture is taken. ImGui sizes a
/// good deal of its layout from what it measured last frame, and hides a window
/// outright for its first frame or two while it fits itself, so an early one
/// is not what the window looks like.
constexpr int kShotFrame = 8;

/// The renderer's own pixels, as a PNG.
void writeScreenshot(SDL_Renderer* renderer, const char* path) {
    int width = 0;
    int height = 0;
    if (SDL_GetRendererOutputSize(renderer, &width, &height) != 0) return;
    std::vector<uint8_t> pixels(std::size_t(width) * std::size_t(height) * 4);
    if (SDL_RenderReadPixels(renderer, nullptr, SDL_PIXELFORMAT_ABGR8888,
                             pixels.data(), width * 4) != 0) {
        std::fprintf(stderr, "could not read the window back: %s\n", SDL_GetError());
        return;
    }
    if (stbi_write_png(path, width, height, 4, pixels.data(), width * 4) == 0) {
        std::fprintf(stderr, "could not write %s\n", path);
        return;
    }
    std::printf("wrote %s (%dx%d)\n", path, width, height);
}

/// The game's own typeface, if any extraction on this machine has produced it.
///
/// A window that installs the game's assets should look like the game, and the
/// face is most of that. It is looked for where the client itself reads from
/// and beside whatever this window is writing to - which covers the common case
/// of running this a second time, to add something to a tree already built.
/// Before any extraction there is nothing to find, and ImGui's built-in face is
/// what the layout was measured against anyway.
fs::path gameFont(const std::string& outputDir, const char* name) {
    std::vector<fs::path> roots;
    if (!outputDir.empty()) roots.emplace_back(outputDir);
    // Where the client reads from on this platform, which is not the same
    // directory on all three - written out by hand it was the macOS one, and
    // the window went without the game's face everywhere else.
    if (const fs::path userData = wowee::core::userDataRoot(); !userData.empty()) {
        roots.emplace_back(userData);
    }

    std::error_code ec;
    for (const fs::path& root : roots) {
        // Directly under the root for a single-expansion tree, and one level
        // into expansions/ for the layout the profiles write.
        std::vector<fs::path> here{root};
        const fs::path expansions = root / "expansions";
        for (fs::directory_iterator it(expansions, ec), end; it != end && !ec; it.increment(ec)) {
            here.push_back(it->path());
        }
        for (const fs::path& at : here) {
            for (const char* dir : {"fonts", "misc/fonts"}) {
                const fs::path file = at / dir / name;
                if (fs::is_regular_file(file, ec)) return file;
            }
        }
    }
    return {};
}

/// How many pixels the renderer puts down for each point the window is measured
/// in. One on an ordinary display, two on a Retina one.
float displayScale(SDL_Window* window, SDL_Renderer* renderer) {
    int windowWidth = 0;
    int pixelWidth = 0;
    SDL_GetWindowSize(window, &windowWidth, nullptr);
    if (SDL_GetRendererOutputSize(renderer, &pixelWidth, nullptr) != 0) return 1.0f;
    if (windowWidth <= 0 || pixelWidth <= 0) return 1.0f;
    return std::max(1.0f, float(pixelWidth) / float(windowWidth));
}

/// A startup failure, said somewhere it will be seen.
///
/// This is a windowed program on Windows, so it has no console to print to:
/// stderr goes nowhere and a double-click on a machine that cannot open a
/// window looked exactly like nothing happening at all. SDL's own box needs no
/// platform code and works before SDL_Init, so the one case that has no window
/// yet is covered too.
void startupFailure(const char* what) {
    std::fprintf(stderr, "%s\n", what);
    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "WoWee Asset Manager", what, nullptr);
}

}  // namespace

int main(int argc, char** argv) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        startupFailure((std::string("SDL could not start: ") + SDL_GetError()).c_str());
        return 1;
    }

    // As much of the page as the display will show at once, since what is not
    // shown has to be scrolled past. Clamped to the space the desktop actually
    // leaves, so a laptop does not open a window taller than its screen.
    int wide = 980;
    int high = 900;
    if (SDL_Rect usable; SDL_GetDisplayUsableBounds(0, &usable) == 0) {
        wide = std::min(wide, std::max(640, usable.w - 80));
        high = std::min(high, std::max(520, usable.h - 80));
    }

    SDL_Window* window = SDL_CreateWindow(
        "WoWee Asset Manager", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        wide, high, SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    if (window == nullptr) {
        startupFailure((std::string("Could not open a window: ") + SDL_GetError()).c_str());
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(
        window, -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
    if (renderer == nullptr) {
        // Software is slower and perfectly adequate for a form with a log in it.
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
    }
    if (renderer == nullptr) {
        startupFailure((std::string("Could not draw: ") + SDL_GetError()).c_str());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    wowee::ui::applyWoweeStyle(ImGui::GetStyle());

    // On a Retina display the window is 900 points wide and the thing drawn
    // into is 1800 pixels. SDL_Renderer does not bridge that on its own, so
    // without this the interface is drawn at point coordinates into a pixel
    // buffer: half size, in the top-left quarter of its own window, with the
    // rest left empty.
    //
    // The renderer is told to scale, which puts the layout back where it
    // belongs. The font atlas is then built at the pixel size it will actually
    // occupy and laid out as though it were half that, so the glyphs land one
    // texel to a pixel rather than being magnified along with everything else.
    const float scale = displayScale(window, renderer);
    ImGuiIO& io = ImGui::GetIO();

    App app;
    // Where the client itself looks when nobody has said otherwise. Data/ beside
    // the terminal's working directory was eight minutes of extraction the
    // client would never find, with nothing on screen saying where it went.
    const fs::path userData = wowee::core::userDataRoot();
    const std::string defaultOut =
        userData.empty() ? (fs::current_path() / "Data").string() : userData.string();
    std::snprintf(app.outputDir, sizeof(app.outputDir), "%s", defaultOut.c_str());

    // The same folders the window takes by drag and drop, for anyone who
    // already has the paths in a terminal.
    if (argc > 1) std::snprintf(app.gameDir, sizeof(app.gameDir), "%s", argv[1]);
    if (argc > 2) std::snprintf(app.secondDir, sizeof(app.secondDir), "%s", argv[2]);
    if (argc > 3) std::snprintf(app.outputDir, sizeof(app.outputDir), "%s", argv[3]);

    // FRIZQT is what the game writes its interface in. A little larger than
    // ImGui's built-in face at the same nominal height, so it is asked for at a
    // size that keeps the controls around it the size they were laid out at.
    const fs::path face = gameFont(app.outputDir, "frizqt__.ttf");
    if (!face.empty()) {
        ImFontConfig config;
        config.SizePixels = kBaseFontSize * kFrizqtRatio * scale;
        io.Fonts->AddFontFromFileTTF(face.string().c_str(), config.SizePixels);
    } else if (scale > 1.0f) {
        ImFontConfig config;
        config.SizePixels = kBaseFontSize * scale;
        io.Fonts->AddFontDefault(&config);
    }
    io.FontGlobalScale = 1.0f / scale;

    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);

    rescan(app);

    const char* shotPath = std::getenv("WOWEE_ASSETS_SCREENSHOT");
    int frames = 0;

    bool quit = false;
    while (!quit) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) quit = true;
            if (event.type == SDL_WINDOWEVENT &&
                event.window.event == SDL_WINDOWEVENT_CLOSE &&
                event.window.windowID == SDL_GetWindowID(window)) {
                quit = true;
            }
            if (event.type == SDL_DROPFILE && event.drop.file != nullptr) {
                std::snprintf(app.gameDir, sizeof(app.gameDir), "%s", event.drop.file);
                SDL_free(event.drop.file);
                rescan(app);
            }
        }

        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        ImGui::Begin("WoWee Asset Manager", nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoBringToFrontOnFocus);

        dimmed("Builds the assets this client reads out of a World of Warcraft "
               "install you own. Nothing is written into the install itself.");
        ImGui::Spacing();

        // The questions scroll; the button that answers them does not. With
        // everything in one scrolling page, adding a line anywhere above pushed
        // Build off the bottom of the window - the one control the whole
        // program exists to offer, reachable only by scrolling past the reading.
        const float actionsHigh = actionsHeight(app);
        if (ImGui::BeginChild("questions", ImVec2(0.0f, -actionsHigh))) {
            drawGame(app);
            ImGui::Spacing();
            drawBase(app);
            ImGui::Spacing();
            drawUpgrades(app);
            ImGui::Spacing();
            drawPlan(app);
        }
        ImGui::EndChild();

        // The in-window browser, on platforms with no chooser of their own. It
        // is a modal, so it is drawn last and over everything.
        if (std::string chosen; app.picker.draw(&chosen)) {
            switch (app.pendingPick) {
                case 1: std::snprintf(app.gameDir, sizeof(app.gameDir), "%s", chosen.c_str());
                        rescan(app);
                        break;
                case 2: std::snprintf(app.secondDir, sizeof(app.secondDir), "%s", chosen.c_str());
                        rescan(app);
                        break;
                case 3: startImport(app, chosen); break;
                default: break;
            }
            app.pendingPick = 0;
        }

        ImGui::Separator();
        drawRun(app);

        ImGui::End();

        ImGui::Render();
        // Re-read every frame: a window dragged between a Retina display and an
        // ordinary one changes this without resizing.
        const float now = displayScale(window, renderer);
        SDL_RenderSetScale(renderer, now, now);
        SDL_SetRenderDrawColor(renderer, 24, 24, 28, 255);
        SDL_RenderClear(renderer);
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);

        // A picture of the window, written once and then done with. What this
        // looks like cannot be checked by reading it, and a bug report about a
        // window is a screenshot; this is how one gets taken on a machine where
        // the person hitting the problem cannot easily take one.
        //
        // Read before the frame is presented, not after: presenting leaves the
        // buffer it came from undefined, and on Metal reading it back then
        // gives a blank image rather than an error.
        if (shotPath != nullptr && ++frames == kShotFrame) {
            writeScreenshot(renderer, shotPath);
            quit = true;
        }

        SDL_RenderPresent(renderer);
    }

    // Both workers are told to stop here and waited for in ~App, so they wind
    // down while SDL is being torn down rather than after it.
    app.job.cancel();
    app.packCancel.store(true);
    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
