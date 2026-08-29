#pragma once

#include "addons/lua_services.hpp"
#include "ui/widget_tree.hpp"
#include <map>
#include <functional>
#include <unordered_map>
#include <string>
#include <utility>
#include <vector>

struct lua_State;

namespace wowee::game { class GameHandler; }

namespace wowee::addons {

struct TocFile;  // forward declaration

class LuaEngine {
public:
    LuaEngine();
    ~LuaEngine();

    /// Drop one-shot interface sounds while the interface is building itself.
    ///
    /// The real client is silent here because it is behind a loading screen;
    /// ours has audio running, so the dropdown initializers that end in
    /// PlaySound("igMainMenuOpen") were audible - seven of them at once. See
    /// AddonManager::loadAllAddons.
    static void setUiSoundsSuppressed(bool suppressed) { uiSoundsSuppressed_ = suppressed; }
    static bool uiSoundsSuppressed() { return uiSoundsSuppressed_; }


    LuaEngine(const LuaEngine&) = delete;
    LuaEngine& operator=(const LuaEngine&) = delete;

    bool initialize();
    void shutdown();

    bool executeFile(const std::string& path);
    bool executeString(const std::string& code);

    /// Run a Lua expression and answer whether it came out true.
    ///
    /// executeString cannot say anything back, and some decisions belong to
    /// the interface: whether Escape closed one of its panels is FrameXML's
    /// answer to give, not something this client can work out from outside.
    bool evaluateBoolean(const std::string& expression);

    /// Error from the last executeFile/executeString that returned false.
    /// Lets a caller loading many files report them together rather than
    /// leaving the reasons scattered through the log.
    [[nodiscard]] const std::string& lastError() const { return lastError_; }

    void setGameHandler(game::GameHandler* handler);
    void setLuaServices(const LuaServices& services);

    // Fire a WoW event to all registered Lua handlers.
    void fireEvent(const std::string& eventName,
                   const std::vector<std::string>& args = {});

    // Try to dispatch a slash command via SlashCmdList. Returns true if handled.
    bool dispatchSlashCommand(const std::string& command, const std::string& args);

    // Call OnUpdate scripts on all frames that have one.
    void dispatchOnUpdate(float elapsed);
    /// Run the OnTextChanged handlers owed by text set from code.
    void drainPendingTextChanged();
    /// Run the OnAnimFinished handlers owed by frames shown to animate.
    void drainPendingAnimFinished();
    /// Age message-frame lines and drop the expired ones.
    void expireMessages(float elapsed);

    /// Feed the mouse to the widget tree: hover changes fire OnEnter/OnLeave,
    /// and a press and release on the same frame is a click. Coordinates are
    /// WoW's, origin bottom-left.
    /// Feeds the widget tree the cursor and which buttons are held.
    ///
    /// Right-click is not a nicety here: it is how WoW opens nearly every
    /// context menu, so a tree that only sees the left button can be looked at
    /// but not used.
    struct MouseButtons {
        bool left = false;
        bool right = false;
        bool middle = false;
    };
    /// Window pixels from the top-left, as ImGui reports them, plus the
    /// window height - converted here rather than by the caller, so the
    /// tree's coordinate space has one entrance.
    void dispatchMouse(float x, float y, float screenH, MouseButtons buttons);

    /// Whether the interface is holding a press: a button went down on a frame
    /// and has not come up, or a frame is being dragged or moved.
    ///
    /// This is the capture test, and the caller is expected to keep feeding the
    /// mouse while it answers true even when the cursor has wandered somewhere
    /// this client's own interface would otherwise claim. dispatchMouse is the
    /// only thing that advances any of the state above - the release path, and
    /// with it OnDragStop and OnReceiveDrag, runs nowhere else. Stopping the
    /// dispatch part way through a drag therefore does not pause it, it strands
    /// it: the button is still down as far as the tree is concerned, the drag
    /// never ends, and the drop lands wherever the cursor happens to be when
    /// the mouse is finally heard from again.
    [[nodiscard]] bool holdsMousePress() const;

    /// Take the cursor away from the interface: whatever it was over is told
    /// OnLeave and nothing is left highlighted.
    ///
    /// For when the mouse has gone somewhere this client's own interface claims
    /// and dispatchMouse is not going to run. Hover is only ever changed in
    /// there, so without this a frame the cursor merely slid off keeps its
    /// highlight and its tooltip for as long as the cursor stays away - the
    /// tree's last word on the subject is that the cursor is still on it.
    void releaseMouseHover();

    /// The wheel, to the frame under the cursor that asked for it.
    ///
    /// Returns true when a frame took it, so the caller knows not to also zoom
    /// the camera - scrolling a quest log and pulling the camera in at the same
    /// time is what happens otherwise. delta is WoW's: positive is up.
    bool dispatchMouseWheel(float x, float y, float delta);

    /// Tells a scroll frame when the room its child has to move changed.
    ///
    /// A scroll bar sizes and enables itself from OnScrollRangeChanged, so a
    /// range that becomes true without being announced leaves the bar disabled
    /// beside a frame that can scroll perfectly well. The change is noticed
    /// here, once a frame, because it follows from layout rather than from
    /// anything the interface called.
    void updateScrollRanges();

    /// Fires OnShow and OnHide for anything that appeared or went away.
    ///
    /// FrameXML declares 295 OnShow handlers and 189 OnHide, and they are
    /// where a panel fills itself in: QuestLog_OnShow is what puts quests in
    /// the quest log. None of them had ever run, so every panel opened with
    /// whatever it was built with and nothing since.
    ///
    /// Noticed after layout rather than fired from Show, because hiding a
    /// frame hides everything under it and none of those were told.
    void updateVisibility();

    /// Fires OnSizeChanged for anything whose rect changed. Declared by
    /// FrameXML and used heavily by addons, and never fired until now.
    void updateSizeChanges();

    /// Says once, a little after loading, how many frames are listening for
    /// the events a unit frame lives on.
    ///
    /// A bar that never moves has two possible causes and they need opposite
    /// fixes: the event is not arriving, or nothing registered for it. This
    /// separates them without anyone having to ask for a trace first, because
    /// the question comes up every time and the answer is one number.
    void reportEventListenersOnce();

    /// Log what the interface itself can see, for the takeover check.
    void runInterfaceProbe();

private:
    bool eventListenersReported_ = false;
    int  eventReportFrames_ = 0;
public:

    /// Typed text, one UTF-8 chunk as the platform reports it.
    void dispatchText(const char* utf8);
    /// A key that is not text: backspace, the arrows, enter, escape.
    void dispatchKey(int sdlKeycode, bool ctrlHeld);
    /// A key for a frame rather than an edit box. Returns true when a frame
    /// took it, so the caller knows not to also treat it as a movement key or
    /// a binding - which is the whole reason a dialog can be typed into.
    bool dispatchFrameKey(int sdlKeycode, bool down);

    /// A typed character, for a frame that asked for the keyboard and is not an
    /// edit box. OnChar is where a dialog reads digits: StackSplitFrame's own
    /// OnKeyDown deliberately ignores them and its OnChar is what builds the
    /// number up, so without this a stack could only be split with the arrows.
    void dispatchFrameChar(const char* utf8);

    /// Run whatever FrameXML has bound to this key, if anything. True if a
    /// binding ran.
    ///
    /// The interface ships 273 binding scripts and, until this existed, no key
    /// press could reach any of them: keys went to a focused edit box or to a
    /// frame listening for them, and nowhere else. The caller must not offer a
    /// key this client answers itself - see KeybindingManager::answersKey -
    /// because most bindings toggle something and two answers to one press
    /// cancel out.
    bool dispatchBindingKey(int sdlKeycode, bool shift, bool ctrl, bool alt,
                            bool down);

    /// The command this key holds, or empty. Whether the client would run it is
    /// a separate question - see clientActsOnBinding - and keeping the two
    /// apart is what lets a declined key be told from an unbound one.
    std::string bindingCommandFor(int sdlKeycode, bool shift, bool ctrl,
                                  bool alt);
    /// Whether an edit box currently has focus, so the client knows not to
    /// treat the same keystrokes as movement.
    ///
    /// A true answer sends the keystroke to the interface and nowhere else, so
    /// a stale one costs the player every key they press. The box that holds
    /// focus has to still be on screen: hiding one now releases it, and this
    /// asks as well, because a box can leave without ever being hidden - a
    /// panel destroyed with focus inside it, or one that went invisible
    /// through a parent before the frame that would have noticed ran.
    [[nodiscard]] bool editBoxHasFocus() const;
    void setEditFocus(uint32_t wid);

    // SavedVariables: load globals from file, save globals to file
    bool loadSavedVariables(const std::string& path);
    bool saveSavedVariables(const std::string& path, const std::vector<std::string>& varNames);

    // Store addon info in registry for GetAddOnInfo/GetNumAddOns
    void setAddonList(const std::vector<TocFile>& addons);

    /// The widget tree CreateFrame and CreateTexture build into. Owned here so
    /// its lifetime matches the Lua state that holds handles into it.
    ui::WidgetTree& widgets() { return widgets_; }

    lua_State* getState() { return L_; }
    [[nodiscard]] bool isInitialized() const { return L_ != nullptr; }

    /// Abort a chunk that runs longer than this many milliseconds, naming the
    /// Lua source and line it was on. Zero disables it.
    ///
    /// A runaway script otherwise freezes the client outright - the load runs
    /// on the main thread, so the window stops responding and the server drops
    /// the connection for want of a heartbeat. A C++ backtrace only says which
    /// binding it was inside; this says which line of Lua kept calling it.
    void setChunkTimeoutMs(unsigned long long ms) { chunkTimeoutMs_ = ms; }

    // Optional callback for Lua errors (displayed as UI errors to the player)
    /// Open this client's own settings window.
    ///
    /// The micro menu's game-menu button calls ToggleGameMenu, and the frame
    /// that answers it is suppressed - so the button did nothing at all. This
    /// client has settings of its own and that is what the button should
    /// reach, which is a decision about which interface owns the panel rather
    /// than a gap in the other one.
    using OpenSettingsCallback = std::function<void()>;
    void setOpenSettingsCallback(OpenSettingsCallback cb) {
        openSettingsCallback_ = std::move(cb);
    }
    [[nodiscard]] const OpenSettingsCallback& openSettingsCallbackRef() const {
        return openSettingsCallback_;
    }

    using LuaErrorCallback = std::function<void(const std::string&)>;
    void setLuaErrorCallback(LuaErrorCallback cb) { luaErrorCallback_ = std::move(cb); }

    /// Record a Lua error and keep it where a player can find it afterwards.
    ///
    /// Errors have always gone to the log with a traceback, and the log has to
    /// be captured to be read - which means asking someone to re-run the game
    /// down a pipe to answer "why does this panel do nothing". They go to
    /// ~/.wowee/lua_errors.txt as well now, deduplicated with a count, beside
    /// the missing-API report that is written the same way and for the same
    /// reason.
    void noteLuaError(const std::string& message);
    void writeLuaErrorReport() const;

private:
    lua_State* L_ = nullptr;
    ui::WidgetTree widgets_;
    game::GameHandler* gameHandler_ = nullptr;
    LuaServices luaServices_;
    LuaErrorCallback luaErrorCallback_;
    /// Distinct Lua errors this session and how often each fired. A handler
    /// that raises on every frame is one entry, not forty thousand.
    std::map<std::string, uint64_t> luaErrors_;
    OpenSettingsCallback openSettingsCallback_;
    /// How many events are being dispatched inside one another right now.
    /// Guards against two handlers triggering each other without end.
    int eventDepth_ = 0;
    std::string lastError_;
    unsigned long long chunkTimeoutMs_ = 0;

    /// Runs a bootstrap Lua chunk and says so when it fails.
    ///
    /// Seventeen of these ran with their result thrown away, so a syntax error
    /// in any one silently removed every method that chunk defined - and the
    /// only symptom was a method quietly answering as though unimplemented.
    void bootstrap(const char* code);

    /// The frame a typed key belongs to - see dispatchFrameKey.
    [[nodiscard]] const ui::Widget* topKeyboardFrame() const;

    void callFrameScript(uint32_t wid, const char* script, const char* arg = nullptr);
    /// The same, with a number. A handler that compares its argument against
    /// zero cannot be handed a numeric string.
    void callFrameScriptNumber(uint32_t wid, const char* script, double arg);
    void callFrameScript4Numbers(uint32_t wid, const char* script,
                                 double a, double b, double c, double d);
    /// Announce where the caret is, which is what sets cursorOffset on the
    /// interface's scrolling edit boxes.
    void fireCursorChanged(uint32_t wid);
    /// Three string arguments, which OnHyperlinkClick takes: the link, the
    /// text it drew, and the mouse button.
    void callFrameScript3(uint32_t wid, const char* script, const char* a,
                          const char* b, const char* c);
    /// Whether a frame declares a script. Used to find which ancestor of a
    /// clicked link actually wants to hear about it.
    bool frameHasScript(uint32_t wid, const char* script);
    /// OnColorSelect, whose three arguments the handler names r, g and b.
    void callFrameScriptColor(uint32_t wid, const char* script, const float rgb[3]);

    /// Push a frame's script and its self argument, ready for the arguments.
    ///
    /// Answers the traceback handler's stack index, or 0 when there is nothing
    /// to call - in which case the stack is already back where it started. On
    /// success the stack holds five values and the caller must finish with
    /// finishFrameScript, whatever it pushes in between.
    int beginFrameScript(uint32_t wid, const char* script);

    /// Call what beginFrameScript set up, report a failure, and unwind.
    ///
    /// `nargs` counts self. The unwind is four, not three - the traceback
    /// handler is still below the call - and getting that number wrong is a
    /// stack leak that shows up somewhere else entirely, which is why the five
    /// callers no longer each hold their own copy of it.
    void finishFrameScript(uint32_t wid, const char* script, int handlerIdx, int nargs);
    /// Which part of a colour picker the held mouse button is dragging: the
    /// wheel, the value bar, or nothing. Decided on the press and kept for the
    /// whole drag, so sliding from one onto the other does not switch which is
    /// being moved.
    uint32_t pickerPart_ = 0;
    bool frameAcceptsClick(uint32_t wid, const char* button);
    /// The nearest frame at or above `wid` registered for this button, or `wid`
    /// itself when none is. A click lands on the topmost frame taking the
    /// mouse, which is not always the one meant to answer it.
    /// The frame at or above this one that takes a drop, or zero.
    uint32_t dropOwnerOf(uint32_t wid);
    uint32_t clickOwnerOf(uint32_t wid, const char* button);

    /// Whether the last dispatched mouse position landed on any FrameXML
    /// widget. Dropping a carried item on the world rather than on a frame is
    /// how an item is destroyed, so the caller has to be able to tell the two
    /// apart - and only the widget tree knows.
public:
    [[nodiscard]] bool mouseOverFrameXml() const { return lastMouseHit_ != 0; }
private:
    uint32_t lastMouseHit_ = 0;
    /// Where the cursor last was, in interface units. Static so a widget
    /// method can reach it without a handle on the engine.
    static inline float sLastMouseX_ = 0.0f;
    static inline float sLastMouseY_ = 0.0f;
public:
    static float lastMouseX() { return sLastMouseX_; }
    static float lastMouseY() { return sLastMouseY_; }

    /// Names an unloaded load-on-demand addon will define, which must read as
    /// absent until it does. FrameXML decides whether to load a panel by asking
    /// for it - `if ( not AchievementFrame ) then AchievementFrame_LoadUI()` -
    /// and the missing-API fallback answering those with a truthy no-op makes
    /// every one of those guards read as "already loaded".
    void declareDeferredGlobals(const std::vector<std::string>& names);
private:

    uint32_t hoverWid_ = 0;
    /// The last frame clicked and when, so a second click on it can be told
    /// from the first. WoW's threshold, near enough.
    uint32_t lastClickWid_ = 0;
    double   lastClickTime_ = 0.0;
    static constexpr double kDoubleClickSeconds = 0.4;

    /// The edit box taking keystrokes, or zero. One at a time, which is
    /// what focus means.
    uint32_t focusedWid_ = 0;
    /// Per button, because a press and its release belong together: sliding off
    /// a button between them is how a player changes their mind, and holding
    /// one button while clicking another must not confuse the first.
    static constexpr int kMouseButtons = 3;
    uint32_t pressedWid_[kMouseButtons] = {0, 0, 0};
    bool buttonDown_[kMouseButtons] = {false, false, false};
    /// Where each button went down, so a drag can be told from a click by how
    /// far the cursor has travelled since. Without a threshold every click is a
    /// one-pixel drag and nothing would ever reach OnClick.
    float pressX_[kMouseButtons] = {0.0f, 0.0f, 0.0f};
    float pressY_[kMouseButtons] = {0.0f, 0.0f, 0.0f};
    /// The frame whose OnDragStart has run and not yet been stopped, and the
    /// button that started it. The frame is not always the one the press landed
    /// on - a drag belongs to the nearest ancestor registered for it - so the
    /// release is matched by button rather than by frame.
    /// Rate limit for the no-drag-owner line, which would otherwise be said
    /// on every frame the button is held.
    double lastNoDragOwnerSaid_ = 0.0;
    uint32_t draggingWid_ = 0;
    int draggingButton_ = -1;
    /// Last cursor position, which is what a moving frame is moved by.
    float cursorX_ = 0.0f, cursorY_ = 0.0f;
    bool haveCursor_ = false;

    /// Make unknown globals answer with a no-op instead of erroring, so a large
    /// body of Lua can be brought up and the names it actually needs collected
    /// from a run. Installed unconditionally, at the end of initialize().
    void installMissingApiFallback();
    /// Log the names collected, once, at shutdown.
    void reportMissingApi() const;



    void registerCoreAPI();
    void registerEventAPI();

private:
    static bool uiSoundsSuppressed_;
};

} // namespace wowee::addons
