#pragma once

// A retained widget tree with WoW's anchor layout.
//
// This is the thing the addon API was missing. CreateFrame answered, events
// dispatched, and CreateTexture handed back a table whose every method was a
// no-op - so an addon could compute and react but could not put a pixel on the
// screen. The API looked supported and nothing drew.
//
// The same tree is what FrameXML targets, because FrameXML is only Lua and XML
// over a widget system. Building it once serves both: addons that draw, and a
// route to running the original interface rather than imitating it.
//
// Deliberately free of Vulkan and ImGui so the layout rules can be tested
// without a device. Rendering lives in widget_renderer.
//
// Coordinates follow WoW, not the screen: the origin is the BOTTOM-left and y
// grows upward. Converting at the point of drawing keeps every anchor rule here
// readable against Blizzard's own documentation, rather than mirrored.

#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace wowee {
namespace ui {

/// Where within a rect a point sits. Fractions of width and height, with y
/// measured from the bottom: BOTTOMLEFT is (0,0) and TOPRIGHT is (1,1).
struct AnchorPoint {
    float fx = 0.0f;
    float fy = 0.0f;
};

/// Resolve a WoW point name. Unknown names resolve to CENTER, which is what an
/// unanchored frame falls back to anyway.
AnchorPoint resolveAnchorPoint(const std::string& name);


enum class WidgetKind : uint8_t { Frame, Texture, FontString };

/// Which of a button's several textures a region is, if any. A button carries
/// art for each state and shows one of them; without knowing which is which,
/// all of them draw at once and the button wears its disabled art over its
/// normal art with the highlight permanently on top.
enum class ButtonArt : uint8_t { None, Normal, Pushed, Highlight, Disabled,
                                 Checked, DisabledChecked };

/// Blizzard's five layers within a frame, drawn in this order.
enum class DrawLayer : uint8_t { Background, Border, Artwork, Overlay, Highlight };
DrawLayer parseDrawLayer(const std::string& name);

/// Frame strata, drawn in this order. Everything in a higher stratum draws over
/// everything in a lower one regardless of level.
enum class FrameStrata : uint8_t {
    World, Background, Low, Medium, High, Dialog,
    Fullscreen, FullscreenDialog, Tooltip
};
FrameStrata parseStrata(const std::string& name);
/// The name parseStrata would accept back, for GetFrameStrata.
const char* strataName(FrameStrata strata);

struct Anchor {
    std::string point = "CENTER";
    uint32_t relativeTo = 0;      ///< Widget id; 0 means "my parent".
    std::string relativePoint = "CENTER";
    float x = 0.0f;
    float y = 0.0f;
};

/// Whether a region's anchors pin *opposite edges* on one axis, which is the
/// only arrangement that decides a size - a 0 and a 1, not any two fractions
/// that differ. `xAxis` selects which axis to ask about.
///
/// The layout's own solve applies the same rule to the constraints it has
/// already resolved. This answers the question earlier, from the anchors alone,
/// for the passes that must know a size before the solve runs. The two must
/// agree: an axis that reports a span here and is then solved some other way,
/// or the reverse, is a region sized twice or not at all.
bool anchorsSpanAxis(const std::vector<Anchor>& anchors, bool xAxis);

struct Widget {
    uint32_t id = 0;
    WidgetKind kind = WidgetKind::Frame;
    uint32_t parent = 0;
    std::vector<uint32_t> children;
    std::string name;
    /// What CreateFrame was asked for - "Button", "StatusBar", "Texture".
    /// FrameXML branches on this constantly, and answering "Frame" for
    /// everything makes every one of those branches take the wrong side.
    std::string objectType = "Frame";
    uint32_t creationOrder = 0;

    std::vector<Anchor> anchors;
    float width = 0.0f;
    float height = 0.0f;
    bool shown = true;
    float alpha = 1.0f;
    /// Whether this frame takes the mouse. False by default, as in WoW, where a
    /// plain Frame is transparent to clicks until EnableMouse is called; Buttons
    /// switch it on for themselves.
    bool mouseEnabled = false;
    /// Whether the frame may be dragged around the screen, and which mouse
    /// buttons begin a drag on it. WoW keeps these separate: a bag window is
    /// movable and registers the left button for drag, while an item button
    /// registers for drag without being movable - dragging it picks the item up
    /// instead of moving the button.
    bool movable = false;
    /// Whether StartSizing will pick this frame up. A frame declares it in XML
    /// or is told by SetResizable; the chat window turns it off while docked.
    bool resizable = false;
    /// The bounds a resize is held inside, from SetMinResize/SetMaxResize.
    /// Zero means unbounded in that direction.
    float minResizeW = 0.0f;
    float minResizeH = 0.0f;
    float maxResizeW = 0.0f;
    float maxResizeH = 0.0f;
    /// Whether dragging is stopped at the screen edge. FrameXML declares it on
    /// 33 frames and addons set it on nearly every window they let the player
    /// move; without it a window dragged past the edge is gone for good, since
    /// the only way back is a drag on a title bar that is no longer reachable.
    bool clampedToScreen = false;
    // How far past each screen edge a clamped frame may sit. Positive is
    // inward, so a positive right lets that much hang off and a negative one
    // holds it clear. Only read when clampedToScreen is set.
    float clampInsetL = 0.0f, clampInsetR = 0.0f;
    float clampInsetT = 0.0f, clampInsetB = 0.0f;
    /// Whether clicking this frame brings it to the front of its strata. WoW
    /// calls it toplevel, and it is what stops one window staying buried under
    /// another once two overlap. FrameXML declares it on 102 frames.
    bool topLevel = false;
    /// Whether the frame receives OnKeyDown and OnKeyUp, and whether it lets
    /// the key through afterwards. WoW consumes by default and passes on only
    /// when asked, which is why a dialog swallows the movement keys while it
    /// is up and nothing else does.
    bool keyboardEnabled = false;
    bool propagateKeys = false;
    /// The frame's own scale, and the product of it with every scale above it.
    ///
    /// A frame's width, height and anchor offsets are in its own units, and
    /// WoW multiplies them by the effective scale to place it. Addons lean on
    /// this to fit a panel where it would not otherwise go. When nothing sets a
    /// scale every effScale is 1 and the arithmetic in layoutWidget is a
    /// no-op, so a tree that never scales lays out exactly as it did before.
    /// How far in from each edge the frame actually answers the mouse.
    ///
    /// Positive shrinks, negative expands - WoW's sense. PaperDollFrame is the
    /// case that matters: it covers the whole character sheet and takes the
    /// mouse, and declares 30 off its right and 45 off its bottom so the
    /// transparent parts do not swallow clicks meant for what is behind them.
    float hitInsetLeft = 0.0f, hitInsetRight = 0.0f;
    float hitInsetTop = 0.0f, hitInsetBottom = 0.0f;

    /// Where a running Translation animation has moved the frame to, in
    /// interface units. Kept apart from the anchors so an animation that is
    /// stopped or interrupted leaves no trace in them - nudging the anchors
    /// instead would make every played animation permanent.
    float animOffsetX = 0.0f, animOffsetY = 0.0f;

    /// The rect this frame last reported through OnSizeChanged. Kept on the
    /// widget rather than in a map beside it: the pass runs every frame over
    /// every widget, and a hash lookup each was paying for a comparison that
    /// two floats here answer directly.
    float lastReportedW = -1.0f, lastReportedH = -1.0f;

    /// The layout generation this widget's rect was last resolved in. Zero
    /// means never, which is older than any generation.
    uint64_t resolvedGen = 0;

    /// How far a button's label shifts while the button is held. Declared as
    /// <PushedTextOffset> on 11 templates; it is the small movement that makes
    /// a button feel pressed rather than merely recoloured.
    float pushedTextOffsetX = 0.0f, pushedTextOffsetY = 0.0f;

    /// A button's label colour while the cursor is over it and while it is
    /// disabled, from <HighlightFont> and <DisabledFont>. Unset means draw it
    /// the usual way, so a template that names neither is unaffected.
    bool  hasHighlightColor = false, hasDisabledColor = false;
    float highlightColor[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    float disabledColor[4]  = {0.5f, 0.5f, 0.5f, 1.0f};

    float scale = 1.0f;
    float effScale = 1.0f;
    bool dragLeft = false;
    bool dragRight = false;
    /// Whether a drag put this frame where it is. The single anchor a move
    /// leaves behind has to give way the next time the interface positions the
    /// frame itself: a bag is re-anchored every time it opens, with a bare
    /// SetPoint and no ClearAllPoints, and a leftover anchor on a different
    /// point turns that into two constraints on one axis - which sizes the
    /// frame from them and opens a bag with no width.
    bool userMoved = false;
    /// The facing a model frame was told to show, in radians. FrameXML rotates
    /// a paperdoll by keeping its own running total and calling SetRotation
    /// with it, so this is absolute rather than a delta.
    float modelFacing = 0.0f;
    /// A label whose size came from measuring its own text, and the text that
    /// was measured. Without the second, a label sized once keeps that size
    /// forever - the character sheet's level line stayed the width of the
    /// placeholder its XML shipped with, long after it read something else.
    bool autoSized = false;
    std::string measuredText;
    /// The size and face the rect above was measured with.
    ///
    /// Remembering only the text was not enough: the interface's own typeface
    /// is registered after the first frames have already been laid out, so a
    /// label measured before that kept a rect built from the fallback font and
    /// was never measured again, its text not having changed. Every one of them
    /// then drew glyphs wider than the box they were given.
    float measuredSize = 0.0f;
    std::string measuredFace;
    /// A label that declared a width and left its height at zero: WoW's
    /// wrapping paragraph. The width is the box to wrap inside and the height
    /// is however many lines that takes - the opposite of autoSized, where the
    /// text decides the width and there is nothing to wrap to.
    ///
    /// `<AbsDimension x="285" y="0"/>` is the idiom, and it is what every
    /// block of prose in the interface is: quest descriptions and objectives,
    /// mail bodies, gossip greetings. Sizing those from their text instead put
    /// each on one line running far outside the frame that clips it, and left
    /// whatever anchored below sitting on top of the lines that should have
    /// pushed it down.
    bool wrapsToWidth = false;

    FrameStrata strata = FrameStrata::Medium;
    bool strataExplicit = false;
    int level = 0;
    bool levelExplicit = false;
    DrawLayer layer = DrawLayer::Artwork;
    int subLevel = 0;

    // Texture regions.
    std::string texturePath;
    float texCoord[4] = {0.0f, 1.0f, 0.0f, 1.0f};   ///< left, right, top, bottom

    /// SetTexCoord's other form: a UV per corner, which is how the interface
    /// rotates a texture. Eight numbers in WoW's order - upper-left,
    /// lower-left, upper-right, lower-right - and only meaningful while
    /// texCoordRotated is set.
    ///
    /// The paperdoll's flyout arrow is the case that matters: the same art
    /// serves the left-hand slots pointing down and the right-hand slots
    /// pointing sideways, and the sideways one is the rotated form. Reading
    /// only the first four numbers of it mapped the art into a 16x38 frame at
    /// the wrong angle, which drew as a pale vertical bar beside the slot.
    bool  texCoordRotated = false;
    float texCoordQuad[8] = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f};
    float color[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    bool solidColor = false;    ///< SetTexture(r,g,b[,a]) rather than a file.

    // Backdrop, the bordered panel look most of the original interface is
    // built from. The edge file is a strip of eight square tiles - verified
    // against the art: UI-Tooltip-Border is 128x16 and UI-DialogBox-Border
    // 256x32, both exactly eight tiles wide.
    bool hasBackdrop = false;
    std::string bgFile;
    std::string edgeFile;
    bool  tileBackground = false;
    float edgeSize = 16.0f;
    float insetLeft = 0.0f, insetRight = 0.0f, insetTop = 0.0f, insetBottom = 0.0f;
    float backdropColor[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    float borderColor[4] = {1.0f, 1.0f, 1.0f, 1.0f};

    // StatusBar. Health, mana, cast bars and experience are all this one type.
    bool  isStatusBar = false;
    /// A slider shares the bar's range and value but is dragged rather than
    /// filled, and draws a thumb at the value instead of a fill to it.
    bool  isSlider = false;
    /// A cooldown darkens what it covers and wipes clear as the time runs out.
    /// Start is on the same clock GetTime answers with; zero duration means
    /// nothing is running.
    /// An edit box holds its own text and a cursor into it, rather than the
    /// font string a label uses: what is typed has to survive between frames
    /// and the caret has to know where it sits.
    /// A scroll frame shows a window onto a taller child. The child is laid
    /// out at its full size and moved by the scroll offset; what falls outside
    /// the frame is clipped rather than drawn.
    /// Whether the frame asked for the wheel. False by default, as in WoW,
    /// where a frame ignores it until EnableMouseWheel is called - which is
    /// what keeps the wheel zooming the camera everywhere else.
    bool  wheelEnabled = false;
    /// A disabled button is greyed and takes no clicks. True by default, as a
    /// button is until something disables it.
    bool  enabled = true;
    /// Whether OnEnter and OnLeave still run once this is disabled. WoW does
    /// not fire them on a disabled button unless the XML asks, which is how a
    /// greyed control explains *why* it is greyed: five templates here declare
    /// it, MainMenuBarMicroButton and LootRollButtonTemplate among them.
    bool  motionScriptsWhileDisabled = false;
    /// Whether this region is one of its owner's state textures, and which.
    ButtonArt buttonArt = ButtonArt::None;
    /// Whether a check button is checked, which decides between its checked
    /// art and none.
    bool  checked = false;
    /// A state the interface asked for outright, overriding what the mouse is
    /// doing. ActionButton_UpdateState holds a toggled ability's button down
    /// this way, and nothing about the cursor should undo that.
    enum class Forced : uint8_t { None, Normal, Pushed, Disabled };
    Forced forcedState = Forced::None;
    /// Highlight held on regardless of the cursor, which is how a selected tab
    /// stays lit once it has been clicked.
    bool  highlightLocked = false;

    bool  isScrollFrame = false;
    uint32_t scrollChild = 0;
    float scrollX = 0.0f, scrollY = 0.0f;
    /// The range last reported to the interface. A scroll bar sizes and
    /// enables itself from OnScrollRangeChanged, so the change has to be
    /// noticed and announced rather than merely being true.
    float reportedRangeX = -1.0f, reportedRangeY = -1.0f;

    bool  isEditBox = false;
    /// Whether this box takes the keyboard the moment it appears.
    ///
    /// Declared on 42 boxes in FrameXML and false on 40 of them, which is the
    /// point of the attribute - a box that grabs focus on sight takes the
    /// keyboard away from whatever the player was doing. The two that ask for
    /// it are the channel-name field and the box that names an equipment set,
    /// where typing is the only reason the dialog opened.
    bool  editAutoFocus = false;
    /// Where an edit box's text starts inside its own frame. WoW's default is
    /// nothing; the four units used here until now were a stand-in, and a box
    /// whose art has a wide border drew its text on top of it.
    float textInsetLeft = 4.0f, textInsetRight = 4.0f;
    float textInsetTop = 0.0f, textInsetBottom = 0.0f;
    std::string editText;
    size_t cursorPos = 0;
    /// The selected run, as byte offsets into editText, when there is one.
    ///
    /// HighlightText(start, stop) sets it and the interface uses it for one
    /// thing above all: autocomplete writes the completed name and highlights
    /// the part it added, so the next character typed replaces the completion
    /// rather than being appended to it. Without a selection, typing "Thr"
    /// into a whisper gave "Thrall" and the next keystroke made "Thralla".
    bool   hasSelection = false;
    size_t selStart = 0;
    size_t selEnd = 0;
    /// What has been typed into this box before, oldest first, and where the
    /// arrow keys are in it.
    ///
    /// FrameXML hands each sent line to AddHistoryLine and then leaves the
    /// walking of it to the client, which is why the chat box recalls what you
    /// typed in the real game and did nothing here. -1 means "not in the
    /// history", which is the state the box returns to at the end of it.
    std::vector<std::string> editHistory;
    int editHistoryPos = -1;
    /// How many the box was declared to keep, from historyLines. Zero means
    /// none, which several boxes ask for outright.
    int editHistoryLines = 0;
    /// Declared with ignoreArrows: left and right do not move the cursor, so
    /// they reach the game instead and a player can turn while typing.
    bool editIgnoreArrows = false;
    bool  editFocused = false;
    bool  editNumeric = false;
    bool  editMultiLine = false;
    int   editMaxLetters = 0;   ///< Zero is no limit, which is WoW's default.
    /// Whether the markup that draws nothing counts against that limit. WoW's
    /// default is no, and four boxes in the interface ask for yes - the macro
    /// editor among them, where the escapes are the point.
    bool  countInvisibleLetters = false;

    /// Whether a label too long for its box breaks onto another line. On, as
    /// WoW has it - a FontString given a width wraps inside it, and nothing
    /// here did, so every one of them drew a single line straight out of its
    /// own frame.
    bool  wordWrap = true;
    /// Break inside a word when a single word is wider than the box. Thirty-six
    /// labels ask for it, all of them prose in a narrow column.
    bool  nonSpaceWrap = false;
    /// Lines the last wrap produced, so the height a wrapped label reports
    /// matches what is drawn. FrameXML sizes panels from GetStringHeight.
    ///
    /// Mutable because it is a record of what the renderer did, not part of
    /// what the frame is - the draw pass holds every widget const and this is
    /// the one thing it learns.
    mutable int wrappedLines = 1;

    bool  isCooldown = false;
    double cooldownStart = 0.0;
    double cooldownDuration = 0.0;
    /// The shaded wedge grows instead of shrinking. An ability's cooldown
    /// shrinks as it becomes usable again; an aura's timer on a unit frame
    /// grows as the aura runs out, and the target frame and the totem bar both
    /// ask for that.
    bool  cooldownReverse = false;
    /// A bright line along the sweeping edge, which the target frame and the
    /// rune bar ask for.
    bool  cooldownDrawEdge = false;
    float sliderStep = 0.0f;
    std::string thumbTexture;
    /// The slider's grip, as a region rather than a file. A <ThumbTexture>
    /// declares a size and no anchors at all, because in WoW the slider is what
    /// puts it where the value says - so the layout moves it along the track
    /// (the same thing already done for the colour picker's thumbs). Left at 0
    /// for a slider that never declared one.
    uint32_t thumbRegion = 0;
    float barMin = 0.0f, barMax = 1.0f, barValue = 0.0f;
    std::string barTexture;
    float barColor[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    /// Which layer the bar's fill draws in. In the real client the fill is a
    /// region like any other and <StatusBar drawLayer="..."> is its layer;
    /// here it is drawn by the bar itself, so the bar carries the layer and
    /// sorts by it. ARTWORK is WoW's default for a bar that does not say.
    DrawLayer barLayer = DrawLayer::Artwork;
    bool  barVertical = false;

    /// Fraction filled, clamped. A zero or inverted range reads as empty rather
    /// than dividing by nothing.
    float barFraction() const {
        const float span = barMax - barMin;
        if (span <= 0.0f) return 0.0f;
        const float f = (barValue - barMin) / span;
        return f < 0.0f ? 0.0f : (f > 1.0f ? 1.0f : f);
    }

    /// A SimpleHTML holds one block of text on the frame itself, the way an
    /// edit box and a message frame do - the interface calls SetText on the
    /// frame rather than on a font string inside it. ItemTextFrame's page is
    /// one, which is why a letter opened to a blank page: the words were set
    /// and stored, and nothing drew text for anything but a FontString.
    ///
    /// The markup is ignored. Real SimpleHTML parses a subset of HTML and this
    /// draws the text as it stands, which is right for the pages the game
    /// actually ships - they are plain text with line breaks.
    bool  isSimpleHtml = false;
    /// A scrolling message frame keeps its own lines rather than a single
    /// string: chat is a list that grows at one end and falls off the other,
    /// and the frame draws as many as fit from the bottom up.
    bool  isMessageFrame = false;
    /// A line, and the three values the chat history hangs off it.
    ///
    /// FrameXML passes them to AddMessage and reads them back with
    /// GetMessageInfo when it moves a conversation into its own window - so
    /// they have to survive the round trip or the copy has nothing to copy.
    /// accessId groups the lines of one conversation; extra is the token that
    /// says which kind of chat it was.
    ///
    /// extra has to come back the same type it went in. The interface hands it
    /// straight to a table lookup, and for chat lines it is a number - handed
    /// back as text it keys nothing, which reads as "this line has no kind"
    /// rather than as a mistake.
    struct Message {
        std::string text;
        float color[4];
        float age = 0.0f;
        double lineId = 0.0;
        double accessId = 0.0;
        bool hasExtra = false;
        bool extraIsNumber = false;
        double extraNumber = 0.0;
        std::string extra;
    };
    std::deque<Message> messages;
    /// How long a message stays before it fades out, in seconds, and how long
    /// the fade itself takes. Zero means it never goes - which is what a chat
    /// frame wants and what UIErrorsFrame very much does not: it declares
    /// displayDuration="5" and every error was staying on screen for good.
    float messageDuration = 0.0f;
    float messageFadeDuration = 3.0f;
    /// Whether a new message goes above the ones already there. UIErrorsFrame
    /// asks for insertMode="TOP"; a chat frame does not.
    bool  messagesInsertTop = false;
    /// Extra space between message lines, which SetPadding sets. WoW's default
    /// is none; the 15% used here is the leading a line already carries.
    float messagePadding = 0.0f;
    size_t maxMessages = 128;
    /// How far back through the history the frame has been scrolled, in lines.
    int   messageScroll = 0;

    /// A tooltip holds lines the same way, but draws them from the top and
    /// sizes itself to fit them - which is the part a chat frame does not do,
    /// because a tooltip has no size of its own until it has something to say.
    bool  isTooltip = false;
    /// The frame this tooltip was last given by SetOwner. A tooltip is hidden
    /// when its owner stops being visible - see hideOrphanedTooltips.
    uint32_t tooltipOwnerId = 0;
    /// Whose tooltip this is, as the unit token SetUnit was given.
    ///
    /// GameTooltip:IsUnit(token) is the one question asked of it, and
    /// gametooltip.xml asks it inside OnTooltipSetUnit to decide whether to
    /// colour the name by reaction. Kept as the token rather than a guid
    /// because this layer knows nothing about guids; the binding resolves both
    /// sides when it compares, so a tooltip set for "target" answers yes to
    /// "mouseover" when they are the same unit.
    std::string tooltipUnit;
    struct TooltipLine {
        std::string left, right;
        float lc[4]; float rc[4];
        /// Whether this line breaks to fit rather than making the whole
        /// tooltip as wide as itself. FrameXML asks for it on every line of
        /// prose - an item's flavour text, a spell's description, a newbie
        /// tip - and the flag was read off the call and dropped, so one long
        /// sentence stretched the tooltip across the screen.
        bool wrap = false;
        /// Lines the wrap produced, filled by the sizing pass so the draw and
        /// the height agree on how tall this line is.
        mutable int lines = 1;
    };
    std::vector<TooltipLine> tooltipLines;
    /// A floor on the width the sizing pass may settle on, from
    /// SetMinimumWidth. FrameXML uses it to keep a money frame from being
    /// clipped by a tooltip narrower than the coins it is about to draw:
    /// SetTooltipMoney measures the money frame, asks the tooltip whether it is
    /// already that wide, and widens it if not.
    float tooltipMinWidth = 0.0f;

    // FontString regions.
    std::string text;
    float fontHeight = 12.0f;
    /// Items a model frame has been asked to try on, as ItemDisplayInfo id
    /// and inventory type. The dressing room's contents belong to the frame
    /// rather than to the application: a second one would have its own, and
    /// closing this one is what empties it.
    struct TryOnItem { uint32_t displayInfoId; uint8_t inventoryType; };
    std::vector<TryOnItem> tryOnItems;

    /// A creature this model frame has been told to show, by display id. Set
    /// by whichever binding decides - the stable's paperdoll is told a slot
    /// and resolves it - and read by the render loop, which is where the
    /// model is actually built. Zero is "nothing chosen".
    uint32_t modelDisplayId = 0;

    /// The colour a ColorSelect frame is showing, as r, g, b. Its own state
    /// rather than the frame's tint: a colour picker draws its wheel in every
    /// colour and this is only the one being chosen.
    float pickerColor[3] = {1.0f, 1.0f, 1.0f};
    /// The same colour as hue, saturation and value, which is what the wheel
    /// and the bar actually move. Kept beside the RGB rather than derived from
    /// it because the conversion is lossy in exactly the places a picker sits:
    /// black has no hue and grey has no saturation, so dragging the value bar
    /// down to zero and back would lose the colour that was being chosen.
    float pickerHSV[3] = {0.0f, 0.0f, 1.0f};

    /// Which part of a colour picker this region is, if any. The four are
    /// declared in XML as their own elements rather than inside a Layer -
    /// <ColorWheelTexture> and friends - and each is placed and drawn from the
    /// colour its ColorSelect parent holds.
    enum class ColorRole : uint8_t { None, Wheel, WheelThumb, Value, ValueThumb };
    ColorRole colorRole = ColorRole::None;

    /// Which named font object this region's type settings came from, so
    /// GetFontObject can hand it back. The fields copied out of one do not add
    /// up to the object itself, and FrameXML passes the object around - the
    /// options panels read a control's font object to get the colour to put a
    /// label back to when the control is re-enabled.
    std::string fontObjectName;
    /// Drawn added to what is under it rather than over it. Art authored for
    /// this has no alpha channel at all - it is a glow on black, and black is
    /// what adds nothing. Drawn the ordinary way it is a black slab instead,
    /// which is what covered the player frame while it pulsed.
    bool blendAdd = false;

    /// A texture the client renders rather than one read from a file - a unit
    /// portrait is a live view of the character, not an image on disk. Zero
    /// means the path above is used instead.
    uint64_t externalTexture = 0;

    /// The typeface a font object named, as it wrote it. Empty means
    /// whatever the renderer is already using.
    std::string fontFace;
    /// A dark copy of the text, offset. Distinct from the outline, which
    /// surrounds the glyphs on all sides; a shadow falls on one.
    bool  hasShadow = false;
    float shadowX = 1.0f, shadowY = -1.0f;
    float shadowColor[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    /// NORMAL or THICK, as a font object writes it. Empty is no outline.
    std::string fontOutline;
    /// Extra space between wrapped lines, which FrameXML reads back.
    float lineSpacing = 0.0f;
    std::string justifyH = "CENTER";
    /// TOP, MIDDLE or BOTTOM. FrameXML declares it on 93 font strings - a
    /// multi-line label in a fixed box sits differently for each.
    std::string justifyV = "MIDDLE";

    /// Minimap zoom step, 0 to 4. Kept here rather than in Lua because the
    /// interface sets it through one button and reads it back through another.
    int zoomLevel = 0;

    /// Whether a link drawn in this frame answers a click. FCF_SetUninteractable
    /// turns it off for a chat window the player has made click-through, and the
    /// GM chat addon turns it off while a ticket is being written. Enabled is
    /// the default because every frame that draws links wants them live.
    bool hyperlinksEnabled = true;

    /// A Minimap:PingLocation the interface asked for, in minimap-local
    /// interface units from its centre with +y up. Parked here for the same
    /// reason the zoom is: turning it into a world position needs the map's
    /// view radius and the camera bearing, and Lua can reach neither. The
    /// frame loop picks it up and clears the flag.
    bool pingRequested = false;
    float pingX = 0.0f, pingY = 0.0f;

    // Filled in by layout(). Screen rect in WoW coordinates: origin bottom-left.
    float left = 0.0f, bottom = 0.0f, rectW = 0.0f, rectH = 0.0f;
    /// Shown, with every ancestor shown. This is what WoW's IsVisible answers
    /// and what decides whether an OnUpdate runs.
    ///
    /// Distinct from `visible` below, which is this AND has somewhere to be
    /// drawn. A frame with no anchors is not drawn - that is WoW's rule and
    /// the reason a stray unanchored panel does not land in the middle of the
    /// screen - but it is still running. Eight of FrameXML's driver frames are
    /// exactly that: created by CreateFrame, never positioned, existing only
    /// to carry an OnUpdate. frameFadeManager drives every fade in the
    /// interface, frameFlashManager every flash, AnimUpdateFrame the whole
    /// animation system.
    bool  visibleChain = false;
    bool  visible = false;      ///< visibleChain, and anchored somewhere
    /// Whether the interface has been told this is on screen. Visibility is
    /// not a property a frame sets - it is shown, and every ancestor shown too
    /// - so becoming visible has to be noticed rather than announced at the
    /// point something was hidden three levels up.
    bool  reportedVisible = false;
    /// Set when Show() already ran this frame's OnShow itself, so the deferred
    /// visibility pass records the change without firing it a second time.
    bool  onShowFired = false;
    /// How many times Lua has flipped `shown` since the last visibility pass.
    ///
    /// Noticing a change by comparing to the last reported state cannot see a
    /// change that undoes itself first. `panel:Hide(); panel:Show()` is how
    /// FrameXML says "rebuild yourself" - it is the whole of QuestFrame's
    /// QUEST_DETAIL handler - and between the two calls nothing looks. The
    /// frame ends the frame exactly as it started, so a comparison finds
    /// nothing and OnShow, which is where the panel is actually filled in,
    /// never runs.
    ///
    /// Counted only from the Show and Hide bindings, so that the renderer
    /// forcing a suppressed window hidden cannot be mistaken for the interface
    /// asking for a rebuild.
    uint8_t shownToggles = 0;
    /// The nearest scroll frame above this one, or zero. Everything under a
    /// scroll frame is drawn clipped to it, which is what makes a window onto
    /// a taller child a window rather than a spill.
    uint32_t clipTo = 0;
    FrameStrata effStrata = FrameStrata::Medium;
    int   effLevel = 0;
};

/// A link drawn on screen this frame, and where it landed.
///
/// The runs carry the payload out of parseMarkup and the drawing pass is the
/// only place the rect is known, but the click arrives in the input pass - so
/// it is recorded here, where both sides already meet.
struct LinkRect {
    uint32_t    widget = 0;   ///< the font string the link was drawn in
    std::string link;         ///< "item:3299", "quest:1234:60"
    std::string text;         ///< what it drew, brackets and all
    float x0 = 0.0f, y0 = 0.0f, x1 = 0.0f, y1 = 0.0f;
};

class WidgetTree {
public:
    WidgetTree();

    /// Back to a tree holding nothing but the screen and UIParent.
    ///
    /// create() appends; it never replaces. An interface loaded over one that
    /// is already built therefore leaves both in the tree, both laid out and
    /// both drawn - a doubled interface, with the second copy of every frame
    /// sitting exactly on the first. So anything that tears the Lua state down
    /// has to bring the tree down with it: the frames belong to the state that
    /// built them.
    ///
    /// The user scale survives. It is the player's own setting rather than
    /// anything the interface built, and dropping it to 1 here would resize
    /// every frame to a scale nobody chose until something set it back. The
    /// effective scale is derived from it at each layout, so it needs nothing.
    void reset();

    /// Links drawn this frame. Cleared at the start of each draw and refilled
    /// by it, so a link that stopped being drawn stops being clickable in the
    /// same frame rather than one later.
    void clearLinkRects() { linkRects_.clear(); }
    void addLinkRect(const LinkRect& r) { linkRects_.push_back(r); }
    /// Every link the last draw filed, for the harness to print: a link that
    /// cannot be clicked is either one that was never filed or one the hit test
    /// misses, and only the list itself tells the two apart.
    [[nodiscard]] const std::vector<LinkRect>& linkRects() const { return linkRects_; }
    /// The last link drawn under this point, which is the topmost - the draw
    /// order is back to front and later rects sit over earlier ones.
    [[nodiscard]] const LinkRect* linkAt(float x, float y) const {
        for (auto it = linkRects_.rbegin(); it != linkRects_.rend(); ++it) {
            if (x >= it->x0 && x <= it->x1 && y >= it->y0 && y <= it->y1) return &*it;
        }
        return nullptr;
    }

    /// The screen-sized root every unparented widget hangs from. WoW calls it
    /// UIParent and addons anchor to it by name constantly.
    [[nodiscard]] uint32_t root() const { return rootId_; }

    uint32_t create(WidgetKind kind, uint32_t parent, const std::string& name);
    Widget* get(uint32_t id);
    [[nodiscard]] const Widget* get(uint32_t id) const;
    [[nodiscard]] size_t size() const { return widgets_.size(); }

    /// Anchor helpers. clearPoints is SetPoint's implicit reset when a frame is
    /// re-anchored from scratch, and what SetAllPoints does before pinning both
    /// corners.
    void clearPoints(uint32_t id);

    /// Set a size, and have it read back before the next layout.
    ///
    /// FrameXML sizes things and measures them in the same breath: a container
    /// frame sets the height of each piece of its background art and then adds
    /// those heights up to size itself. Storing only the requested size and
    /// answering GetHeight from the last laid-out rect makes that sum the
    /// *previous* frame's numbers, so the art and the buttons inside it end up
    /// describing two different frames.
    void setWidth(uint32_t id, float width);
    void setHeight(uint32_t id, float height);

    /// Pin a frame where it currently sits, on one anchor to its parent.
    ///
    /// What StartMoving does before the cursor takes over: a frame anchored to
    /// something else cannot be dragged without the drag fighting the anchor,
    /// so the anchors are replaced by a single one describing where it is now.
    void pinToCurrentPosition(uint32_t id);

    /// Shift every anchor by the same amount, which moves the frame.
    void nudge(uint32_t id, float dx, float dy);

    /// Put a frame in front of everything else in its strata.
    ///
    /// Strata come first in the draw order, so this only moves the frame within
    /// its own - a DIALOG frame raised above its peers still sits under a
    /// TOOLTIP one, which is what the strata are for.
    void raise(uint32_t id);
    /// The reverse, for Lower().
    void lower(uint32_t id);
    /// Shift every descendant that carries its own level, so a raised window
    /// takes what is inside it along. Public because SetFrameLevel needs the
    /// same behaviour.
    void shiftExplicitLevels(uint32_t id, int delta);

    /// The frame currently following the cursor, if any.
    ///
    /// StartMoving and StopMovingOrSizing are called from Lua, and the cursor
    /// is read by the input loop, so the two need somewhere to meet. It lives
    /// here beside the pressed and hovered frames rather than in the input
    /// loop, because that is what the bindings can already reach.
    [[nodiscard]] uint32_t movingWidget() const { return movingWid_; }
    void setMovingWidget(uint32_t id) { movingWid_ = id; }

    /// The frame a size grabber is dragging, and which corner it took hold of.
    ///
    /// Same arrangement as the moving frame above and for the same reason:
    /// StartSizing is called from Lua and the cursor is read by the input loop.
    /// The corner matters because dragging the left edge has to move the frame
    /// as well as resize it - its right edge must stay where it is.
    [[nodiscard]] uint32_t sizingWidget() const { return sizingWid_; }
    [[nodiscard]] const std::string& sizingPoint() const { return sizingPoint_; }
    void setSizingWidget(uint32_t id, const std::string& point) {
        sizingWid_ = id;
        sizingPoint_ = point;
    }

    /// Grow or shrink a frame being sized, by a cursor delta in interface
    /// units. Held inside SetMinResize/SetMaxResize.
    void resizeBy(uint32_t id, const std::string& point, float dx, float dy);
    void addPoint(uint32_t id, const Anchor& anchor);
    void setAllPoints(uint32_t id, uint32_t relativeTo);

    /// Resolve every widget's rect and visibility for a screen of this size.
    /// Lays the tree out for a window of this many pixels.
    ///
    /// FrameXML's coordinates are not pixels. The interface is authored against
    /// a virtual screen 768 units tall - a 232x100 unit frame is meant to look
    /// the same size on every display - so the tree is laid out in those units
    /// and the renderer multiplies by the scale on the way to the screen.
    /// Treating them as pixels drew the whole interface at half size on a
    /// 1528-tall window and at double on a 384-tall one.
    void layout(float pixelW, float pixelH);

    /// Resolve one widget's rect now, if anything has moved since it was last
    /// resolved.
    ///
    /// Rects used to be answered only from the once-a-frame pass, so a frame
    /// anchored inside a handler measured as though it had never been placed -
    /// its own height sitting at the origin - until the next frame. The real
    /// client answers a measurement whenever it is asked, and the interface is
    /// written to that: the quest tracker anchors each objective line and then
    /// reads its bottom edge to know how tall the block grew, and every one
    /// came back zero, so the tracker concluded it had nothing to show and
    /// collapsed itself.
    ///
    /// Only what the answer depends on is resolved - the widget, what it is
    /// anchored to, and the parents of both - rather than the whole tree. The
    /// interface anchors a row and measures it in the same breath, so a
    /// whole-tree pass per measurement turned one tracker update into hundreds
    /// of them: correct, and about five milliseconds each.
    ///
    /// Free when nothing has moved: each widget records the generation it was
    /// resolved in, and a repeat asks nothing.
    void resolveWidget(uint32_t id);

    /// How far a scroll child's contents actually reach, in interface units.
    ///
    /// A scroll child is very often smaller than what is inside it, because
    /// nothing in the interface resizes it - the client does. The talent tree
    /// is the plainest case: PlayerTalentFrameScrollChildFrame declares 320x50
    /// and holds eleven rows of talents 63 apart, and no line of FrameXML ever
    /// gives it a height. Taking the declared 50 meant a scroll range of zero
    /// and a tree that could not be scrolled at all.
    ///
    /// Never smaller than the child's own rect, so a frame that does set its
    /// own height (a HybridScrollFrame sets it from its rows) keeps it.
    /// Hidden frames do not count: a pool of buttons parked out of the way is
    /// not content, and counting it scrolls into empty space.
    void scrollContentExtent(uint32_t childId, float& outW, float& outH) const;

    /// Something moved, resized or changed parent - every rect is stale.
    void markLayoutDirty() { ++layoutGeneration_; }

    /// Pixels per interface unit, from the last layout.
    /// How many pixels one interface unit is worth, after the player's own
    /// scale. Everything that converts between units and pixels uses this, so
    /// hit testing and drawing stay in step.
    [[nodiscard]] float uiScale() const { return uiScale_; }

    /// The player's UI Scale, as WoW's video options mean it.
    ///
    /// One is the size the screen's height alone would give. Below one the
    /// interface is smaller and there is more room, which is what the slider
    /// was originally for.
    ///
    /// The ceiling was Blizzard's own 1.0 and a screen across a room wants more
    /// than that, but 2 was too far and the reason is arithmetic rather than
    /// taste. The interface lays out in a canvas kInterfaceHeight tall, so the
    /// height actually available to it is kInterfaceHeight / userScale_: at 2
    /// that is 384, and the frames are authored to fit the 600 of an 800x600
    /// screen. The options frame is very nearly that tall, so its own buttons
    /// went off the bottom - a scale that cannot be undone from inside the
    /// game, which is what Blizzard's ceiling was really guarding against.
    ///
    /// So the ceiling is that constraint stated directly rather than a number
    /// chosen to feel safe. kCVarRanges gives the slider the same value, and
    /// the confirmation dialog is the second line of defence.
    void setUserScale(float scale) {
        const float clamped = scale < 0.64f ? 0.64f
                            : (scale > kMaxUserScale ? kMaxUserScale : scale);
        if (clamped == userScale_) return;
        userScale_ = clamped;
        // Every rect is stale after a scale change, and this said so through
        // layoutDirty_ - the half of the mechanism nothing read. The live half
        // is the generation counter, so it says it through that now.
        markLayoutDirty();
    }
    [[nodiscard]] float userScale() const { return userScale_; }

    /// The canvas height the interface lays out in. The scale a screen of a
    /// given pixel height gets is pixelHeight / this, times the user's own.
    static constexpr float kInterfaceHeight = 768.0f;
    /// The shortest layout the shipped frames still fit in: the 600 of an
    /// 800x600 screen, which is the smallest the interface was authored for.
    static constexpr float kMinLayoutHeight = 600.0f;
    /// How far up the interface can be scaled before its own frames stop
    /// fitting on screen. See setUserScale.
    static constexpr float kMaxUserScale = kInterfaceHeight / kMinLayoutHeight;

    /// The screen-filling frame everything else hangs off.
    [[nodiscard]] uint32_t rootId() const { return rootId_; }
    /// UIParent, which is the root's one child at startup and the parent of
    /// nearly everything. Distinct from rootId() - see the note there.
    [[nodiscard]] uint32_t uiParentId() const { return uiParentId_; }

    /// Records a frame as a scroll frame, and keeps the list of them. Walking
    /// every widget each frame to find a handful is the kind of cost that does
    /// not show up until the interface is large, which it now is.
    /// Move a frame under a different parent.
    ///
    /// SetParent wrote a field on the Lua table and nothing else, so GetParent
    /// answered the new parent while the frame went on being laid out, clipped
    /// and shown or hidden by the old one. QuestInfo reparents every element of
    /// a quest into either the quest giver's panel or the quest log's scroll
    /// child on each display, and the consolidated buff container moves buffs
    /// in and out of itself, so both were placing frames that stayed where they
    /// were.
    ///
    /// Everything inherited - visibility, clipping, strata, level, scale - is
    /// resolved from the parent during layout, so this only has to fix the link
    /// and the two children lists.
    void setParent(uint32_t id, uint32_t newParent);

    void markScrollFrame(uint32_t id);

    /// Remember that a texture is meant to show the player's portrait.
    ///
    /// SetPortraitTexture names the texture to fill, and the handle it should
    /// be filled with is rebuilt whenever the portrait's render target is -
    /// so the assignment has to happen every frame rather than once here. A
    /// list, for the same reason scroll frames are a list: the alternative is
    /// scanning every widget in the tree for a flag, every frame.
    /// Claim this texture for a unit, dropping any claim another unit had on
    /// it. One at a time is the whole point: TargetFrame's portrait is asked
    /// for "target" and for "player" in turn, as the player targets themselves
    /// and then something else, and a texture left on two lists is handed two
    /// faces a frame and keeps whichever was written last.
    ///
    /// An empty unit releases it. Releasing clears the handle too - dropping
    /// off a list only stops the updates, and the last face would stay.
    void setPortraitUnit(uint32_t id, const std::string& unit);
    /// Every texture claimed for this unit, or an empty list.
    [[nodiscard]] const std::vector<uint32_t>& portraitsFor(const std::string& unit) const;

    /// What the mouse is doing, so state art can be chosen. The engine owns
    /// this - it is the only thing that knows what is under the cursor and
    /// what is being held - and the tree needs it to decide which of a
    /// button's textures to draw.
    void setInteraction(uint32_t hovered, uint32_t pressed) {
        hoveredId_ = hovered;
        pressedId_ = pressed;
    }
    /// Which frame is being held, for anything that draws differently while a
    /// button is down. The tree already chooses the pushed texture from this;
    /// the label moves with it.
    [[nodiscard]] uint32_t pressedWidget() const { return pressedId_; }
    /// Which frame the cursor is over, for anything that draws differently
    /// under it - a button's label lightens in WoW, and that is a font object
    /// the template names rather than a colour the renderer invents.
    [[nodiscard]] uint32_t hoveredWidget() const { return hoveredId_; }

    /// Record which frame a tooltip is describing, so it can be taken away
    /// with it. See hideOrphanedTooltips.
    void setTooltipOwner(uint32_t tooltipId, uint32_t ownerId);
    /// Hide any tooltip whose owner has stopped being visible.
    ///
    /// A tooltip is hidden by its owner's OnLeave, and a frame that is hidden
    /// while the cursor is on it never gets one - the loot window closing on
    /// the last item is exactly that, and its item description stayed on
    /// screen with nothing under it. Checked once a frame rather than fixed at
    /// each of the places a panel can close, because every one of them is the
    /// same omission and new ones would arrive with the same bug.
    void hideOrphanedTooltips();
    [[nodiscard]] const std::vector<uint32_t>& scrollFrames() const { return scrollFrames_; }
    [[nodiscard]] const std::vector<uint32_t>& playerPortraits() const { return portraitsFor("player"); }

    /// The widget published under this name, or null. Names are unique in
    /// FrameXML by convention, and the last one to claim a name wins, which is
    /// what a lookup by name means there too.
    Widget* findByName(std::string_view name);
    [[nodiscard]] const Widget* findByName(std::string_view name) const;

    /// The height the interface is authored against. Blizzard's own number.

    /// The frame under a point, or 0. Topmost wins, by the same ordering that
    /// decides what draws over what - so whatever the player can see on top is
    /// what they click. Regions are never hit: in WoW a texture is not a mouse
    /// target, its frame is.
    [[nodiscard]] uint32_t hitTest(float x, float y) const;

    /// The same, for the mouse wheel, which frames may take without taking the
    /// mouse. EnableMouseWheel and EnableMouse are separate in WoW and
    /// UIPanelScrollFrameTemplate asks for only the first - it declares
    /// OnMouseWheel and never enables the mouse. Asking the plain hit test
    /// meant no scroll frame in the interface was ever found under the cursor,
    /// so the wheel fell through to the camera and nothing scrolled.
    [[nodiscard]] uint32_t hitTestWheel(float x, float y) const;

    /// Widgets to draw, in the order to draw them. Only those that resolved to a
    /// visible, non-empty rect. Valid until the next layout().
    [[nodiscard]] const std::vector<const Widget*>& drawOrder() const { return drawOrder_; }

private:
    /// Both hit tests, which differ only in whether a frame that took the
    /// wheel without taking the mouse counts as under the cursor.
    [[nodiscard]] uint32_t hitTestFor(float x, float y, bool forWheel) const;
    std::vector<LinkRect> linkRects_;
    void layoutWidget(uint32_t id, float screenW, float screenH);
    /// The same, without descending into the children. What a single-widget
    /// resolve needs, and the body of the recursive one.
    void layoutWidgetSelf(uint32_t id, float screenW, float screenH);
    /// Resolve a widget after whatever its rect is measured from.
    void resolveChain(uint32_t id, float screenW, float screenH, int& depth);

    /// Bumped whenever anything moves. A widget resolved in this generation
    /// needs no further work; one from an older generation is stale.
    uint64_t layoutGeneration_ = 1;

    /// The size the last full pass ran at, so an on-demand one can match it.
    float lastPixelW_ = 0.0f;
    float lastPixelH_ = 0.0f;
    /// layout() moves widgets, and moving a widget raises the flag. Without
    /// this an on-demand pass would leave the tree dirty and every rect read
    /// after it would run another one.
    bool  layingOut_ = false;
    void collectDrawOrder();

    /// A deque, not a vector, because get() hands out a pointer into this and
    /// create() grows it. A vector reallocates, and any pointer taken before a
    /// create would dangle after one - a use-after-free waiting on the first
    /// caller that holds a Widget* across creating a child. A deque keeps
    /// references valid when it grows, which is the guarantee this needs.
    float uiScale_ = 1.0f;
    float userScale_ = 1.0f;
    std::vector<uint32_t> scrollFrames_;
    /// Which unit each portrait texture is showing, and the reverse. Two maps
    /// rather than one because both questions are asked every frame: the draw
    /// loop wants every texture for a unit, and a claim wants the unit a
    /// texture already had so it can be released from it.
    std::map<std::string, std::vector<uint32_t>> portraitsByUnit_;
    std::map<uint32_t, std::string> portraitUnitOf_;
    uint32_t hoveredId_ = 0;
    /// Tooltips that have been given an owner, so the check is over a handful
    /// of frames rather than every widget in the tree.
    std::vector<uint32_t> ownedTooltips_;
    uint32_t pressedId_ = 0;

    /// Whether a state texture should be drawn given what the mouse is doing.
    [[nodiscard]] bool buttonArtVisible(const Widget& w) const;
    std::deque<Widget> widgets_;   ///< Index 0 is a placeholder; id == index.
    /// The screen itself, above UIParent. Everything FrameXML draws hangs off
    /// UIParent, but not quite everything: a frame declared at XML top level
    /// with no parent of its own is parentless in WoW, and that is load
    /// bearing. Opening the world map runs UIParent:Hide() and then shows the
    /// map - so if the map is a child of UIParent it goes down with the rest
    /// of the interface and nothing is left on screen. Dropdowns, tooltips and
    /// the cinematic frame have to outlive the same call.
    uint32_t rootId_ = 0;
    uint32_t uiParentId_ = 0;
    uint32_t movingWid_ = 0;
    uint32_t sizingWid_ = 0;
    std::string sizingPoint_;
    uint32_t nextOrder_ = 1;
    std::vector<const Widget*> drawOrder_;
};

/// Hue-saturation-value to red-green-blue and back, with hue in turns rather
/// than degrees because everything that reads it here is an angle around a
/// wheel or a fraction of one.
///
/// Free functions beside the tree rather than inside the picker, because three
/// places need them and they are the sort of arithmetic that gets written
/// slightly differently each time it is written again: the binding converting
/// a colour it was handed, the renderer drawing every hue into the wheel, and
/// the input mapping a cursor back onto it.
void hsvToRgb(const float hsv[3], float rgb[3]);
void rgbToHsv(const float rgb[3], float hsv[3]);

} // namespace ui
} // namespace wowee
