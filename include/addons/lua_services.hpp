// lua_services.hpp - Dependency-injected services for Lua bindings.
// Replaces Application::getInstance() calls in domain API files (§5.2).
#pragma once

#include <functional>
#include <cstdint>
#include <string>
#include <vector>

namespace wowee::core  { class Window; }
namespace wowee::audio { class AudioCoordinator; }
namespace wowee::game  { class ExpansionRegistry; }

namespace wowee::addons {

struct LuaServices {
    core::Window*            window            = nullptr;
    audio::AudioCoordinator* audioCoordinator  = nullptr;
    game::ExpansionRegistry* expansionRegistry = nullptr;

    /// Run a macro body, as RunMacroText() does - one command per line,
    /// through the same path the action bar uses for a macro button.
    std::function<void(const std::string&)> runMacroText;

    /// This client's own slash commands, and a way to run one.
    ///
    /// The registry in ChatPanel answers about seventy names FrameXML has no
    /// equivalent for - /unstuck, /coords, /transport, /threat, the GM
    /// helpers. It used to be reached from this client's own chat input, and
    /// handing chat over took that away: FrameXML's ChatEdit_ParseText
    /// consults SlashCmdList and nothing else, so every one of them stopped
    /// being typeable. These two bridge the registry into that table.
    std::function<std::vector<std::string>()> clientChatCommandNames;
    std::function<bool(const std::string&, const std::string&)> runClientChatCommand;

    /// How far back the camera may be pulled, as a multiple of the original
    /// client's limit - the game's Max Camera Distance slider.
    ///
    /// A callback rather than a renderer pointer, because this file is given
    /// services rather than the application: the one thing it needs here is
    /// this number reaching the camera.
    std::function<void(float)> setCameraMaxDistanceFactor;

    /// How far the ground cover is drawn - the game's Ground Clutter Radius.
    std::function<void(float)> setGroundDetailDistance;

    /// Loop Music - whether a zone track runs on or stops at its end.
    std::function<void(bool)> setZoneMusicLooping;

    /// Which page of the main action bar is showing.
    ///
    /// The page belongs to the interface - ChangeActionBarPage moves it and
    /// six frames redraw from it - while the number keys are handled here, so
    /// the client has to be told. Without it those keys always cast page one,
    /// whatever the bar on screen was showing.
    std::function<void(int)> setActionBarPage;

    /// Close the program. The last resort behind Exit Game: the tidy path asks
    /// the server to log the character out first, and this is what answers
    /// when there is no server, no character, and no handler to ask.
    std::function<void()> quitApplication;

    /// The anisotropy ceiling on new samplers - the game's Texture Filtering.
    std::function<void(float)> setAnisotropyLimit;

    /// How far the world's clutter is drawn against how far the world is -
    /// the game's Environment Detail.
    std::function<void(float)> setEnvironmentDetail;

    /// How many particles a model emits, as a fraction of what it asks for -
    /// the game's Particle Density.
    std::function<void(float)> setParticleDensity;

    /// How much of the weather to draw, 0 to 1 - the game's Weather Detail.
    std::function<void(float)> setWeatherDensity;

    /// Ask for the interface to be reloaded, as ReloadUI() does.
    ///
    /// A request rather than the act: reloading shuts the Lua state down and
    /// builds a new one, and ReloadUI is called from inside that state - by a
    /// static popup's OnAccept, or by /reload going through the interface. Doing
    /// it there frees the machinery running the call. The application performs
    /// it between frames instead.
    std::function<void()> requestReloadUI;

    /// Read and write this client's audio settings by name, so FrameXML's own
    /// Sound options drive the same values the settings window does rather than
    /// writing to a CVar store nothing reads.
    ///
    /// Volumes are 0..1, which is what Blizzard's sliders use; the client keeps
    /// them as percentages and converts at this boundary. Known keys: master,
    /// music, ambient. "enableall" is the mute, inverted - the panel's checkbox
    /// asks whether sound is on.
    /// Read and write a client setting by name, as a string.
    ///
    /// FrameXML's options panels are bound to CVar names and their values live
    /// in this client's settings. One bridge rather than a getter and a setter
    /// per setting: the specific hooks below were seven pairs and growing, and
    /// each new option meant a field here, a lambda in Application, and a branch
    /// in each of GetCVar and SetCVar.
    ///
    /// Strings both ways because that is what a CVar is. A bool is "0" or "1".
    std::function<std::string(const std::string&)> getClientSetting;
    std::function<void(const std::string&, const std::string&)> setClientSetting;

    std::function<float(const std::string&)> getAudioSetting;
    std::function<void(const std::string&, float)> setAudioSetting;

    /// Push the client's own volume sliders at the audio system again.
    ///
    /// The interface's three enable switches multiply on top of what those
    /// sliders set, and they are applied by zeroing a channel. Zeroing has no
    /// inverse, so turning one back on needs the sliders re-applied underneath
    /// it before the switches are composed over them again.
    std::function<void()> reapplyAudioVolumes;

    /// Open this client's settings window, on a named tab when one is given.
    ///
    /// FrameXML's GameMenuFrame has Video, Sound and Interface buttons that
    /// call ShowUIPanel on option frames the shim only creates as shells. This
    /// is where those go instead - the settings themselves live in this
    /// client's panel, and handing over the menu must not hide them.
    std::function<void(const std::string&)> openSettings;

    /// Play an emote's animation on the player, by emote name.
    ///
    /// DoEmote sent the emote to the server and stopped there, so the player
    /// stood still while everyone was told they had danced. This client's own
    /// chat panel had always done both halves - the animation was one of the
    /// cases handing chat over to FrameXML quietly dropped.
    std::function<void(const std::string&)> playEmoteAnimation;

    /// Screen gamma, for the interface's own video options. Callbacks rather
    /// than a renderer pointer, to keep this header off the rendering ones.
    std::function<float()> getGamma;
    std::function<void(float)> setGamma;

    /// What the world map is showing, for the interface's own map.
    ///
    /// Callbacks and plain structs rather than the facade, on the same
    /// principle as the gamma pair above: this header stays off the rendering
    /// ones. The client keeps every one of these and drew them only itself, so
    /// FrameXML's map had a full set of readers and nothing to read.
    struct MapOverlay {
        std::string texture;   ///< texture prefix; the tiles are texture1..N
        int width = 0, height = 0;
        int offsetX = 0, offsetY = 0;
    };
    std::function<std::vector<MapOverlay>()> getMapOverlays;

    struct MapLandmark {
        std::string name;
        std::string description;
        int   icon = 0;
        float x = 0.0f, y = 0.0f;   ///< [0,1] across the map being shown
    };
    std::function<std::vector<MapLandmark>()> getMapLandmarks;

    /// The zone under a point on the map, in [0,1] map space, or empty. This
    /// is what names the label under the cursor.
    std::function<std::string(float, float)> getMapZoneNameAt;
    /// Drill into that zone, as clicking it does. False when there is none.
    std::function<bool(float, float)> clickMapPoint;
    /// Step out one level: zone to continent, continent to world.
    std::function<void()> zoomMapOut;

    /// The map's two dropdowns and its zoom-out button, which set the map from
    /// a continent index and a zone index rather than from a point on it.
    /// Both one-based, as the interface counts them; zone zero is the
    /// continent itself and continent zero is the world.
    std::function<std::vector<std::string>()> getMapContinentNames;
    std::function<std::vector<std::string>(int)> getMapZoneNames;
    /// False when the map refused the pair - an unknown continent, or a zone
    /// row past the end of that continent's list. The caller discarded this,
    /// so a dropdown pick the map could not honour looked exactly like one it
    /// had.
    std::function<bool(int, int)> setMapByIndex;
    /// What is shown now, in those same indices, and whether there is a level
    /// above it. The button that asks the last of these was disabled always.
    std::function<int()> getMapContinentIndex;
    std::function<int()> getMapZoneIndex;
    std::function<bool()> canZoomMapOut;
    /// Back to the zone the player is standing in, which the interface asks
    /// for every time it shows the map.
    std::function<void()> showPlayerMapZone;
    /// The WorldMapArea id being shown, and setting the map from one. Zero
    /// when a continent rather than a zone is shown, which is the branch the
    /// interface takes to ask about continents instead.
    /// Where a canonical world position falls on the map now showing, 0..1
    /// across the image. False when it is off this map. GetPlayerMapPosition
    /// answers in this space - FrameXML multiplies it by the map frame's width.
    std::function<bool(float, float, float, float&, float&)> mapUVForWorldPos;
    std::function<uint32_t()> getMapWorldAreaId;
    std::function<void(uint32_t)> setMapWorldAreaId;

    /// The zone the player is standing in, worked out from the terrain under
    /// them and refreshed every frame.
    ///
    /// The server's zone is only told to us on SMSG_INIT_WORLD_STATES, which
    /// arrives when the server notices a zone change and not otherwise - so
    /// reading that alone leaves the name stale, naming the last zone the
    /// server announced rather than the one being walked through. The real
    /// client works this out locally for exactly that reason.
    std::function<uint32_t()> getLiveZoneId;

    /// Whether the player is standing on a world PvP objective.
    ///
    /// The *area* rather than the zone, which is why it is not derived from
    /// getLiveZoneId: the flag sits on the subzone - Halaa, The Overlook, the
    /// Plaguelands towers - and resolving to the zone loses it. Wintergrasp is
    /// the one that survives either way, being its own zone.
    std::function<bool()> isOnOutdoorPvpObjective;

    /// Nameplates over hostile and neutral units, for nameplateShowEnemies.
    ///
    /// There is no counterpart for nameplateShowFriends: this client always
    /// draws player names and has no switch for them, so that CVar is stored
    /// and answered but changes nothing.
    std::function<bool()> getNameplatesShown;
    std::function<void(bool)> setNameplatesShown;

    /// Whether the minimap turns with the camera, for rotateMinimap.
    std::function<bool()> getMinimapRotate;
    std::function<void(bool)> setMinimapRotate;

    /// Speech bubbles over players' heads, for chatBubbles. The interface
    /// options have always had the checkbox; nothing was behind it.
    std::function<bool()> getChatBubblesShown;
    std::function<void(bool)> setChatBubblesShown;

    /// Whether looking up drags the view down, for mouseInvertPitch.
    ///
    /// There is deliberately no counterpart for mouseSpeed, which sits beside
    /// it on the same panel: FrameXML never declares that slider's range -
    /// neither the control nor OptionsSliderTemplate sets minValue or maxValue
    /// - so the number it writes has no scale to convert from, and the camera's
    /// own sensitivity runs 0.05 to 1. Mapping one onto the other would be a
    /// guess about both ends.
    /// mouseInvertPitch and gxVSync had a pair each here. Both are rows in
    /// kClientCVars now - the client has settings keys for them, which it did
    /// not when they were written, and a row costs nothing where a pair costs
    /// four places to keep in step.
    /// Windowed or full screen, for gxWindow - applied by RestartGx rather
    /// than by the SetCVar that precedes it, which is the order the video
    /// panel works in: it writes every changed CVar, then restarts the device
    /// once so the ones marked `restart` take effect together.
    std::function<bool()> getFullscreen;
    std::function<void(bool)> setFullscreen;

    /// The resolution, as a position in the shared mode list. The video
    /// panel's dropdown is built from GetScreenResolutions and hands back the
    /// row it was given, so the list and the index have to be one thing.
    std::function<int()> getResolutionIndex;
    std::function<void(int)> setResolutionIndex;

    /// Anti-aliasing, as a row in the same four modes this client's own panel
    /// offers. The video panel's Multisampling dropdown is built from
    /// GetMultisampleFormats and hands back the row, so the list and the index
    /// have to be one thing here too.
    std::function<int()> getAntiAliasingIndex;
    std::function<void(int)> setAntiAliasingIndex;

    /// The barber shop's selectors, for the interface's own barber panel.
    ///
    /// Selector numbers are FrameXML's BarberShopFrameSelector IDs: 1 hair
    /// style, 2 hair colour, 3 facial hair, 4 skin. The state behind them used
    /// to be built inside this client's own barber window, so with the panel
    /// handed over nothing had built it - these reach a version that does not
    /// depend on who is drawing.
    std::function<bool(int selector, std::string& name, bool& isCurrent)> getBarberStyleInfo;
    std::function<void(int selector, int direction)> setNextBarberStyle;
    std::function<uint32_t()> getBarberTotalCost;
    std::function<void()> barberReset;
    /// The Okay button. blizzard_barbershopui.xml names ApplyBarberShopStyle
    /// as an OnClick handler attribute, which is why nothing noticed it was
    /// unbound - the readiness report reads script bodies, not attributes.
    std::function<void()> barberApply;

    /// Whether the camera is inside a WMO, for IsIndoors and IsOutdoors.
    ///
    /// The renderer has tracked this all along and the macro conditionals
    /// [indoors] and [outdoors] already read it; only the Lua bindings were
    /// stubbed, answering a flat false and a flat true. Two paths to the same
    /// question, and only one of them was ever improved.
    std::function<bool()> isPlayerIndoors;

    /// Load a load-on-demand addon by name, as LoadAddOn() does.
    ///
    /// The interface asks for these itself: opening the talent frame is
    /// TalentFrame_LoadUI calling LoadAddOn("Blizzard_TalentUI"), and the same
    /// goes for the achievement, macro, key binding, trade skill and glyph
    /// panels. A stub answering "MISSING" left every one of them dead.
    ///
    /// Returns WoW's pair: loaded, and a reason when it did not. Reasons are
    /// WoW's own tokens - MISSING, DISABLED, LOAD_ON_DEMAND_ERROR.
    std::function<bool(const std::string& name, std::string& reason)> loadAddOn;
    /// Whether a named addon has been loaded, for IsAddOnLoaded.
    std::function<bool(const std::string& name)> isAddOnLoaded;

    /// Turn an addon on or off for the next run, as EnableAddOn and
    /// DisableAddOn do. It takes effect on reload rather than at once - an
    /// addon already loaded has its functions in the Lua state and its frames
    /// on screen, and neither can be taken back out.
    std::function<void(const std::string& name, bool enabled)> setAddOnEnabled;

    /// Every icon this install carries, as paths SetTexture accepts.
    ///
    /// The macro and guild bank pickers are grids over this list: they ask how
    /// many there are and then for one at a time by index. Built from the asset
    /// manifest rather than a fixed list, so an install carrying different art
    /// offers what it actually has.
    std::function<const std::vector<std::string>&()> listIconTextures;

    /// Save a screenshot, as the client's own binding and /screenshot do.
    ///
    /// Routed through the same call rather than reimplemented, so both put the
    /// file in the same place under the same name.
    std::function<void()> takeScreenshot;
};

} // namespace wowee::addons
