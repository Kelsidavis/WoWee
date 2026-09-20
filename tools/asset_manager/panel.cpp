#include "panel.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include "imgui.h"

#include "core/data_paths.hpp"
#include "core/local_time.hpp"
#include "pipeline/asset_inventory.hpp"

#include "casc.hpp"
#include "pack.hpp"
#include "profiles.hpp"

namespace wowee::assets {
namespace {

namespace fs = std::filesystem;

/// The log pane, once a build is running.
constexpr float kLogHeight = 200.0f;

void wrapped(const char* text) {
    ImGui::PushTextWrapPos(ImGui::GetContentRegionAvail().x);
    ImGui::TextUnformatted(text);
    ImGui::PopTextWrapPos();
}

void dimmed(const char* text) {
    // The theme's own disabled tone, so this reads on a dark window and on
    // the client's cream page alike rather than being grey on both.
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
    wrapped(text);
    ImGui::PopStyleColor();
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
    ImGui::SeparatorText("1.  Which game's assets do you want to use?");
    // Named for what is being chosen rather than for what this program then
    // does with it. "The folder you want to build from" describes the build;
    // somebody deciding which of two installations to point at is choosing
    // whose art and sounds they are going to be looking at.
    folderRow(app, "The World of Warcraft installation to take them from",
              app.gameDir, sizeof(app.gameDir), PickWhat::Folder,
              "Choose the World of Warcraft installation to use", app.picker,
              app.pendingPick, 1);

    if (!app.gameScan.note.empty()) {
        const bool good = haveGame(app);
        ImGui::PushStyleColor(ImGuiCol_Text, good ? app.goodColor : app.warnColor);
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
        ImGui::PushStyleColor(ImGuiCol_Text, app.goodColor);
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
        dimmed("Choose the installation above first.");
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
                ImGui::PushStyleColor(ImGuiCol_Text, app.warnColor);
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
                ImGui::PushStyleColor(ImGuiCol_Text, app.errorColor);
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
    // The button row, at whatever size the buttons themselves came out.
    const float k = ImGui::GetFontSize() / 13.0f;
    float height = 32.0f * k + style.ItemSpacing.y * 2.0f + style.FramePadding.y * 2.0f;
    {
        std::lock_guard<std::mutex> lock(app.packMutex);
        if (!app.packNote.empty()) height += ImGui::GetTextLineHeightWithSpacing();
    }
    if (app.started) {
        height += ImGui::GetFrameHeightWithSpacing();        // the progress bar
        height += ImGui::GetTextLineHeightWithSpacing();     // what it is doing
        height += kLogHeight * k + style.ItemSpacing.y * 2.0f;
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

    // The row is fitted to the width there is rather than to a set of widths
    // that happen to add up on one window. Fixed boxes overran the right
    // edge of the card once the client began drawing this at its own scale,
    // and would again at any size nobody had looked at. The numbers below
    // are the proportions the row is divided in, not pixels.
    const bool showStop = app.job.running();
    const float stopWeight = showStop ? 90.0f : 0.0f;
    const float weights = 180.0f + stopWeight + 170.0f + 260.0f;
    const float gaps = ImGui::GetStyle().ItemSpacing.x * (showStop ? 3.0f : 2.0f);
    const float unit = std::max(0.0f, ImGui::GetContentRegionAvail().x - gaps) / weights;
    const float high = 32.0f * (ImGui::GetFontSize() / 13.0f);
    const auto share = [&](float weight) { return ImVec2(weight * unit, high); };

    ImGui::BeginDisabled(!ok || busy);
    if (ImGui::Button("Build my assets", share(180.0f))) {
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

    // showStop, not a fresh query: the widths above were divided on its
    // value, and a job finishing between there and here would drop the
    // button while its share of the row stayed reserved.
    if (showStop) {
        ImGui::SameLine();
        if (ImGui::Button("Stop", share(90.0f))) app.job.cancel();
    }

    // Installing somebody else's pack asks nothing about your game, so it is
    // not behind the questions above: a person handed a pack has no archives to
    // point at and nothing to choose.
    ImGui::SameLine();
    ImGui::BeginDisabled(busy);
    if (ImGui::Button("Install a pack...", share(170.0f))) {
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
    if (ImGui::Button(save.c_str(), share(260.0f))) {
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
        const float logHigh = kLogHeight * (ImGui::GetFontSize() / 13.0f);
        if (ImGui::BeginChild("log", ImVec2(0, logHigh), ImGuiChildFlags_Borders)) {
            for (const std::string& line : app.job.log()) {
                ImGui::TextUnformatted(line.c_str());
            }
            if (app.job.running()) ImGui::SetScrollHereY(1.0f);
        }
        ImGui::EndChild();
    }
}

}  // namespace

void rescan(App& app) {
    app.gameScan = scanInstall(app.gameDir);
    app.secondScan = scanInstall(app.secondDir);
    app.cascConfirmed = false;
    app.cascError.clear();
}

void initPanelDefaults(App& app) {
    // Where the client itself looks when nobody has said otherwise. Data/
    // beside the working directory was eight minutes of extraction the client
    // would never find, with nothing on screen saying where it went.
    const fs::path userData = wowee::core::userDataRoot();
    const std::string out =
        userData.empty() ? (fs::current_path() / "Data").string() : userData.string();
    std::snprintf(app.outputDir, sizeof(app.outputDir), "%s", out.c_str());
}

bool finishedSuccessfully(const App& app) {
    return app.started && app.job.finished() && app.job.succeeded();
}

void drawPanel(App& app) {
    // No standing header here: the standalone window and the client each say
    // why the panel is in front of somebody, and they do not say the same
    // thing. Drawn here it was said twice in the client.
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
}

}  // namespace wowee::assets
