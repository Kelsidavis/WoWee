#pragma once

/// The asset manager's window contents, without a window around them.
///
/// This began as the whole of wowee_assets and is shared now because the
/// client needs the same thing: somebody who has just installed WoWee and has
/// no assets yet reaches a login screen it cannot get past, and the program
/// that fixes that was a separate executable they had to know to go and find.
/// Everything here is ImGui calls against the job and scan code beside it -
/// no SDL, no renderer, no window - so the standalone tool draws it through
/// SDL_Renderer and the client draws it through Vulkan, from one source.

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "imgui.h"

#include "folder_picker.hpp"
#include "install_scan.hpp"
#include "job.hpp"

namespace wowee::assets {

/// Everything the panel remembers between frames.
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

    /// The three accents the panel says things in, set by whoever is drawing
    /// it. The standalone window is dark and the client's page is cream, and
    /// a green that reads on one does not read on the other. These are the
    /// dark ones; the client replaces them with its own crayons.
    ImVec4 goodColor{0.35f, 0.78f, 0.45f, 1.0f};
    ImVec4 warnColor{0.85f, 0.65f, 0.30f, 1.0f};
    ImVec4 errorColor{0.88f, 0.42f, 0.38f, 1.0f};

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

/// Point the output at where the client reads from, before anything is drawn.
/// Separate from the struct's own defaults because it asks the filesystem
/// where that is, which a member initialiser should not.
void initPanelDefaults(App& app);

/// Look at whichever folders are named now and say what is in them. Called
/// when a path changes, including by a drag and drop the host handled.
void rescan(App& app);

/// The panel itself, drawn into whatever window is already open. The caller
/// owns the Begin/End around it, because the tool wants the whole screen and
/// the client wants a card in the middle of its own.
void drawPanel(App& app);

/// Whether a build has finished and written something usable, which is the
/// point at which the client has assets it did not have when it started.
[[nodiscard]] bool finishedSuccessfully(const App& app);

}  // namespace wowee::assets
