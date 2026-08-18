#include "ui/settings_schema.hpp"

#include "ui/graphics_defaults.hpp"

namespace wowee {
namespace ui {

namespace {

// Every setting this client has, except the five bound to a Blizzard control.
//
// Those five - mouse speed, the minimap clock, friendly nameplates, ground
// clutter and the sound effects volume - are driven from FrameXML's own Video,
// Sound and Interface panels through kClientCVars, and listing them here as
// well would draw a second control for the same value. The root panel names
// them and says where they are.
//
// View distance was a sixth until 2026-08-13, and should not have been: the
// Video panel that was supposed to drive it is suppressed along with the rest
// of FrameXML's game menu, so nothing the player can open offered it. Before
// leaving a setting out of here on the grounds that a Blizzard control has it,
// check that the frame holding that control is not in kSuppress.
//
// The order is the order they are read in: a category is a panel, a section is
// a heading on it, and a setting whose section is "" continues the one above.
// Ranges match the settings window's own sliders, because they are the ranges
// the client clamps to; a control that offers more than the client accepts is a
// control that appears to do nothing at the ends.
constexpr SettingDesc kSchema[] = {
    // ---------------------------------------------------------------- Graphics
    {"graphicspreset", "Quality preset", SettingKind::Enum, 0, 4, 1, "Graphics", "Quality",
     "Sets every graphics option at once. Changing any of them afterwards\n"
     "moves this to Custom.",
     "Custom|Low|Medium|High|Ultra", 0},
    // No shadows row, because turning them off crashes the client.
    //
    // With the casters skipped the shadow pass still begins, clears and
    // transitions its map - all of which was written deliberately, and none of
    // which is enough: the GPU faults within a second or so and the device is
    // lost. GPU-assisted validation reports nothing at all before it goes, so
    // the fault is inside a shader rather than in an API call, and it is not
    // found yet. Until it is, the control is off the panel and shadows are held
    // on: a setting whose only effect is to end the session is worse than a
    // setting that is missing.
    {"shadowdistance", "Shadow distance", SettingKind::Float, 40, 500, 10, "Graphics", "Shadows",
     // No longer conditional on a shadows toggle: there is not one, and the
     // stored value it used to read may still say 0 from before it went, which
     // would grey this out for good.
     "How far from you shadows are still drawn.", "", 300},
    // No view distance row here on purpose. The game's own Effects panel has
    // one - it writes the farclip CVar, which kClientCVars maps to this
    // client's viewdistance setting - and it is the place a player looks for
    // it. A row here was added while that panel was suppressed under
    // UiElement::GameMenu and could not be opened at all; that is no longer
    // true, so what it left behind was two controls for one number, each
    // showing a different value until one of them was touched.
    //
    // The reason it could not simply be deferred to before is that Blizzard's
    // slider stops at 1277, the range the original renderer had. kCVarRanges
    // gives farclip this client's own 400-2400 instead, so the native control
    // now covers everything the engine can do.
    // No water refraction row on purpose. It is not a choice any more: the
    // shoreline masks, the meniscus at the waterline and the underwater tint are
    // all written against water that refracts, and the flat fallback left them
    // reading against a surface that does not behave the way they assume. The
    // shader keeps its own guard for a frame whose scene copy is not there yet,
    // which is a different thing from a player turning the feature off.
    {"fogstrength", "Fog strength", SettingKind::Float, 0, 2, 0.05f, "Graphics", "Atmosphere",
     "How much distance fog, against what the zone asks for. 1 is the zone's\n"
     "own amount, higher brings it closer, 0 turns it off.", "", 0.4f},
    {"fogskyblend", "Fog blends with sky", SettingKind::Float, 0, 1, 0.05f, "Graphics", "",
     "How far distance fog takes the colour of the sky behind it. The zone's\n"
     "own fog colour has no relation to its sky, so in a dark zone the horizon\n"
     "turns pale against it. 0 is the zone's colour alone.", "", 0.7f},

    // Labelled for what it is rather than for the heading it sits under: a row
    // whose label repeats its own section reads on the panel as the heading
    // printed twice, once without a control. "Multisampling" is also what the
    // game's own video options call this dropdown.
    {"antialiasing", "Multisampling", SettingKind::Enum, 0, 3, 1, "Graphics", "Anti-aliasing",
     "Costs memory as well as time, and has no effect while FSR 3 is\n"
     "upscaling - FSR does its own.",
     "Off|2x MSAA|4x MSAA|8x MSAA", 1, "upscaling!=2"},
    // Two, not off, and not four.
    //
    // Off was the right default for the hardware this game shipped on, and it
    // left a fresh install with no anti-aliasing of any kind - no
    // multisampling, FXAA off, upscaling off. Nothing that can run this
    // renderer at all is troubled by 2x over geometry this light. Not 4x or 8x
    // because the memory is the part that still costs, and an integrated GPU
    // driving a high resolution display is a real case; the panel offers both
    // to anyone who wants them.
    {"fxaa", "FXAA", SettingKind::Bool, 0, 0, 0, "Graphics", "",
     "Smooths edges after everything else is drawn. Cheap, slightly soft,\n"
     "and can be used together with MSAA or FSR.", "", 0},

    {"normalmapping", "Normal mapping", SettingKind::Bool, 0, 0, 0, "Graphics", "Surfaces",
     "Light stone and cloth by their surface detail rather than flat.", "", 1},
    {"normalmapstrength", "Normal map strength", SettingKind::Float, 0, 2, 0.1f, "Graphics", "",
     "How pronounced that surface detail is. 1 is as authored.", "", 0.8f, "normalmapping"},
    {"parallax", "Parallax mapping", SettingKind::Bool, 0, 0, 0, "Graphics", "",
     "Give bricks and cobbles real depth when seen at an angle.", "", 1},
    {"parallaxquality", "Parallax quality", SettingKind::Enum, 0, 2, 1, "Graphics", "",
     "How many steps each surface is traced with: 16, 32 or 64.",
     "Low|Medium|High", 1, "parallax"},

    {"lensflare", "Lens flare", SettingKind::Float, 0, 2, 0.1f, "Graphics", "Sky",
     "How strong the sun's flare is. It warms toward amber as the sun nears\n"
     "the horizon, which is the dawn and dusk look; 0 turns it off entirely.",
     "", 1.0f},
    {"sharpstars", "Sharp stars", SettingKind::Bool, 0, 0, 0, "Graphics", "",
     "Draw the night sky's stars as points rather than from the sky model's\n"
     "own 256x256 star texture, which is stretched across the whole dome and\n"
     "gets softer the higher your resolution goes.", "", 1},

    // --------------------------------------------------------------- Upscaling
    {"upscaling", "Upscaling", SettingKind::Enum, 0, 2, 1, "Upscaling", "Mode",
     "Render below your resolution and scale up. FSR 1 is spatial and cheap;\n"
     "FSR 3 is temporal and sharper, and does its own anti-aliasing.",
     "Off|FSR 1 (spatial)|FSR 3 (temporal)", 0},
    {"fsrquality", "FSR quality", SettingKind::Enum, 0, 3, 1, "Upscaling", "",
     "How far below your resolution the world is drawn.",
     "Ultra Quality (77%)|Quality (67%)|Balanced (59%)|Native (100%)", 3, "upscaling!=0"},
    {"fsrsharpness", "FSR sharpness", SettingKind::Float, 0, 2, 0.1f, "Upscaling", "",
     "Sharpening applied after upscaling.", "", 1.6f, "upscaling!=0"},
    // Only when AMD's runtime is actually in the build. It is the one setting
    // here with no in-tree implementation behind it: the temporal upscaler is
    // this client's own compute shaders and runs either way, but frame
    // generation is the SDK's alone. With the backend off the control would
    // tick, save, and change nothing.
    // Their own category, which the interface's options panel turns into its
    // own page. They belong beside ground clutter under Graphics, but that
    // page is full - two rows pushed lens flare and sharp stars off the bottom
    // of its second column, which test_settings_panel_layout catches - and a
    // setting a player cannot find is not a setting.
    {"grassenabled", "Enable grass (experimental)", SettingKind::Bool, 0, 0, 0,
     "Grass", "Ground cover (experimental)",
     "Experimental. Grass grown from the terrain's own ground-effect data.\n"
     "Off by default: it is new, it costs time on the main thread as you\n"
     "move, and it still has known faults.", "", 0},
    {"grassdensity", "Density", SettingKind::Float, 0, 300, 5, "Grass", "",
     "How much grass grows, against the amount the terrain asks for.", "", 100},
    {"grassheight", "Height", SettingKind::Float, 50, 300, 5, "Grass", "",
     "How tall it grows. Taller grass shows the wind crossing it more.", "", 100},
    {"grassdistance", "Distance", SettingKind::Float, 30, 2000, 5, "Grass", "",
     "How far out grass draws, in yards. Past 45 the field thins with\n"
     "distance, and each blade grows in gently as you ride toward it.", "", 150},

#if WOWEE_HAS_AMD_FSR3_FRAMEGEN
    {"framegen", "Frame generation", SettingKind::Bool, 0, 0, 0, "Upscaling", "",
     "Experimental. FSR 3 only, and known broken on RADV/Mesa.",
     "", 0, "upscaling=2"},
#endif
    // A debugging aid, so it is not built into a release.
    //
    // Its own tooltip says the rest of the range is "for finding out why",
    // which is not a sentence a player can act on: the control has one correct
    // value and every other setting of it makes the picture worse. It stays in
    // a debug build, where the finding-out happens.
#ifndef NDEBUG
    {"fsrjittersign", "Jitter sign", SettingKind::Float, -2, 2, 0.02f, "Upscaling", "FSR 3 tuning",
     "Which way FSR 3's sub-pixel jitter is applied. 0.38 is the value that\n"
     "currently looks right; the rest of the range is for finding out why.", "", 0.38f, "upscaling=2"},
#endif

    // ----------------------------------------------------------------- Display
    {"fullscreen", "Fullscreen", SettingKind::Bool, 0, 0, 0, "Display", "Screen",
     "Takes effect the next time the window is rebuilt.", "", 0},
    {"vsync", "Vertical sync", SettingKind::Bool, 0, 0, 0, "Display", "",
     "Wait for the display before showing a frame. Removes tearing, and\n"
     "caps the frame rate at your refresh rate.", "", 1},
    {"framecap", "Frame rate limit", SettingKind::Enum, 0, 6, 1, "Display", "",
     "How many frames a second to draw at most - the fps cap. This client\n"
     "otherwise renders a twenty-year-old game as fast as the hardware\n"
     "allows, which on a laptop is heat and fan noise for frames nobody\n"
     "sees. Vertical sync already caps at your refresh rate; this is for\n"
     "capping below it.",
     "Unlimited|30|60|90|120|144|240", 0},
    // No brightness row here. The game's own Video panel has the Gamma slider,
     // which is the same number on a different scale - GetGamma answers this
     // setting divided by 50 - and it is where a player looks for it. Two
     // sliders for one value showed different numbers until one was touched.

    // ------------------------------------------------------------------ Camera
    {"fov", "Field of view", SettingKind::Float, 45, 110, 1, "Camera", "View",
     "How wide a view the camera takes. 70 is what the original client shows.", "", 70},
    // The client shakes the camera for spell effects and for thunderstorms, and
    // there was no control over it. There would not have been in 2004 - the
    // idea that this is something to offer is newer than the game - and it is
    // standard now, because for some people it is the difference between
    // playing and feeling ill. Defaults to the full amount, so nobody's picture
    // changes until they ask.
    {"camerashake", "Camera shake", SettingKind::Float, 0, 1, 0.05f, "Camera", "",
     "How much the view moves on its own: spell effects, thunder, and the\n"
     "sway while drunk. Zero stops it. Walking crooked while drunk is not\n"
     "affected - that happens to your character, not to the picture.", "", 1.0f},
    // No extended-zoom switch here. The game's own Camera panel has Max Camera
    // Distance, which is the same setting expressed as a multiple rather than
    // as a choice between two positions - and it wrote a CVar nothing read
    // while this checkbox did the work. kCVarRanges widens that slider past the
    // shipped ceiling of 2, so it reaches everywhere the checkbox used to and
    // every distance in between.
    {"camerastiffness", "Camera stiffness", SettingKind::Float, 5, 100, 1, "Camera", "",
     "How closely the camera keeps up with you. Higher is tighter and less\n"
     "floaty.", "", 30},
    {"pivotheight", "Pivot height", SettingKind::Float, 0, 3, 0.1f, "Camera", "",
     "How far above your feet the camera turns around. Lower feels more\n"
     "attached to the character.", "", 1.6f},
    {"smoothfollow", "Smooth follow", SettingKind::Bool, 0, 0, 0, "Camera", "",
     "Keep easing the camera while you turn, rather than snapping behind you.", "", 0},
    {"idleorbit", "Idle orbit", SettingKind::Bool, 0, 0, 0, "Camera", "",
     "Drift the camera slowly around you while you stand still.", "", 1},
    {"invertmouse", "Invert mouse look", SettingKind::Bool, 0, 0, 0, "Camera", "Mouse",
     "Push the mouse forward to look up.", "", 0},

    // --------------------------------------------------------------- Interface
    {"uiopacity", "Window opacity", SettingKind::Int, 20, 100, 5, "Interface", "Windows",
     "How solid this client's own windows are drawn.", "", 65},
    // Up to 3x because a phone needs it: the same 1080 lines that are an
    // ordinary monitor are a 420 dpi panel held at arm's length, and 1.5 does
    // not reach. Harmless on a desktop, where nobody drags it that far.
    {"windowuiscale", "Window scale", SettingKind::Float, 0.75f, 3.0f, 0.05f, "Interface", "",
     "Fonts, controls and spacing in this client's own windows. Not the\n"
     "interface's scale, which is in the game's own Video panel.", "", 1},
    {"latencymeter", "Latency meter", SettingKind::Bool, 0, 0, 0, "Interface", "",
     "The round trip to the server, beside the minimap.", "", 1},
    {"micromenu", "Micro menu buttons", SettingKind::Bool, 0, 0, 0, "Interface", "",
     "The row of shortcuts to the character sheet, spellbook and the rest.", "", 0},

    {"bagscale", "Bag scale", SettingKind::Float, 0.75f, 1.5f, 0.05f, "Interface", "Bags",
     "Size of the bag windows.", "", 1},
    {"separatebags", "Separate bag windows", SettingKind::Bool, 0, 0, 0, "Interface", "",
     "One window per bag, rather than everything in one.", "", 1},
    {"showkeyring", "Show keyring", SettingKind::Bool, 0, 0, 0, "Interface", "",
     "The key ring beside the bags.", "", 1},

    // ----------------------------------------------------------------- Minimap
    // Rotate-with-camera is deliberately absent. The settings window still
    // draws that checkbox and its handler pins it back off - the minimap is
    // north-up in this client and the control has not worked for as long as it
    // has existed. A tickbox that unticks itself is worse here than no tickbox
    // at all, so this list does not offer one.
    {"minimapsquare", "Square minimap", SettingKind::Bool, 0, 0, 0, "Minimap", "Appearance",
     "Draw the map as a square rather than a circle.", "", 0},
    {"minimapnpcdots", "Nearby NPC dots", SettingKind::Bool, 0, 0, 0, "Minimap", "",
     "Mark creatures near you on the map.", "", 0},
    {"minimapcoords", "Coordinates", SettingKind::Bool, 0, 0, 0, "Minimap", "",
     "Show your position below the map.", "", 0},

    // ------------------------------------------------------------- Action Bars
    // Up to 2, not 1.5: a slot is 48 pixels times this, and the scale a
    // 2160-line screen wants is 2. Stopping at 1.5 meant the control could not
    // ask for what the display needed, and the bars sat smaller than the buff
    // bar beside them however far it was dragged.
    {"actionbarscale", "Action bar scale", SettingKind::Float, 0.5f, 2.0f, 0.05f,
     "Action Bars", "Scale", "Size of every action bar slot.", "", 1},
    {"buffbarscale", "Buff bar scale", SettingKind::Float, 0.75f, 1.5f, 0.05f,
     "Action Bars", "", "Size of the buff and debuff icons, on top of the automatic scaling\n"
     "this client does for the display it is on.", "", 1},

    {"showbar2", "Bottom left bar", SettingKind::Bool, 0, 0, 0, "Action Bars", "Extra bars",
     "The second bar, above the main one. Client action page 6.", "", 0},
    {"bar2offsetx", "Bottom left - across", SettingKind::Float, -600, 600, 10,
     "Action Bars", "", "Move that bar sideways from its default place.", "", 0, "showbar2"},
    {"bar2offsety", "Bottom left - up", SettingKind::Float, -400, 400, 10,
     "Action Bars", "", "Move that bar up or down from its default place.", "", 0, "showbar2"},
    {"showrightbar", "Right side bar", SettingKind::Bool, 0, 0, 0, "Action Bars", "",
     "The upright bar at the right edge. Client action page 3.", "", 0},
    {"rightbaroffsety", "Right side - up", SettingKind::Float, -400, 400, 10,
     "Action Bars", "", "Move it up or down from the middle of the screen.", "", 0, "showrightbar"},
    {"showleftbar", "Left side bar", SettingKind::Bool, 0, 0, 0, "Action Bars", "",
     "The upright bar at the left edge. Client action page 4.", "", 0},
    {"leftbaroffsety", "Left side - up", SettingKind::Float, -400, 400, 10,
     "Action Bars", "", "Move it up or down from the middle of the screen.", "", 0, "showleftbar"},

    // ------------------------------------------------------------ Combat & HUD
    {"nameplatescale", "Nameplate scale", SettingKind::Float, 0.5f, 2.0f, 0.05f,
     "Combat & HUD", "Nameplates", "Size of the bars over creatures' heads.", "", 1},

    {"dpsmeter", "Damage meter", SettingKind::Bool, 0, 0, 0, "Combat & HUD", "Trackers",
     "Your damage and healing per second, above the action bar, while you\n"
     "are in combat.", "", 0},
    {"cooldowntracker", "Cooldown tracker", SettingKind::Bool, 0, 0, 0, "Combat & HUD", "",
     "Your longer cooldowns as they run, beside the action bar.", "", 0},
    {"raretracker", "Rare tracker", SettingKind::Bool, 0, 0, 0, "Combat & HUD", "",
     "Mark rare creatures near you on both maps.", "", 0},
    {"chesttracker", "Chest tracker", SettingKind::Bool, 0, 0, 0, "Combat & HUD", "",
     "Mark chests near you on both maps.", "", 0},

    {"damageflash", "Damage flash", SettingKind::Bool, 0, 0, 0, "Combat & HUD", "Screen effects",
     "A red vignette at the edges of the screen when you are hit.", "", 1},
    {"lowhealthvignette", "Low health vignette", SettingKind::Bool, 0, 0, 0,
     "Combat & HUD", "", "A red edge that pulses while you are below a fifth of\n"
     "your health.", "", 1},

    // ------------------------------------------------------------------- Sound
    {"musicvolume", "Music", SettingKind::Int, 0, 100, 5, "Sound", "Music and ambience",
     "", "", 30},
    {"woweemusic", "WoWee soundtrack", SettingKind::Bool, 0, 0, 0, "Sound", "",
     "Include this client's own music alongside the game's.", "", 1},
    {"ambientvolume", "Ambience", SettingKind::Int, 0, 100, 5, "Sound", "",
     "Wind, water, birds and the rest of the world's own noise.", "", 100},
    {"bellvolume", "City bells", SettingKind::Int, 0, 100, 5, "Sound", "",
     "The hour struck in the capital cities.", "", 50},

    {"uivolume", "Interface", SettingKind::Int, 0, 100, 5, "Sound", "Effects",
     "Clicks, bag sounds and window noises. Each of these is a balance\n"
     "against the others; the Sound Effects slider in the game's own Sound\n"
     "panel scales all of them together.", "", 100},
    {"combatvolume", "Combat", SettingKind::Int, 0, 100, 5, "Sound", "", "", "", 100},
    {"spellvolume", "Spells", SettingKind::Int, 0, 100, 5, "Sound", "", "", "", 100},
    {"movementvolume", "Movement", SettingKind::Int, 0, 100, 5, "Sound", "", "", "", 100},
    {"footstepvolume", "Footsteps", SettingKind::Int, 0, 100, 5, "Sound", "", "", "", 100},
    {"mountvolume", "Mounts", SettingKind::Int, 0, 100, 5, "Sound", "", "", "", 70},
    {"activityvolume", "Activity", SettingKind::Int, 0, 100, 5, "Sound", "",
     "Fishing, mining, forges and the rest.", "", 100},

    {"npcvoicevolume", "NPC voices", SettingKind::Int, 0, 100, 5, "Sound", "Voices", "", "", 100},
    {"characterspeech", "Character speech", SettingKind::Bool, 0, 0, 0, "Sound", "",
     "Your own character's grunts and greetings.", "", 1},

    // -------------------------------------------------------------------- Chat
    //
    // Which channels to join on entering the world. These are the client's own
    // doing rather than the interface's - it sends the join for each one - so
    // they belong here whichever chat window is on screen.
    //
    // Chat's appearance is deliberately not here. Timestamps, the font size,
    // the background and the fade belong to the chat frame the interface draws,
    // and it has its own controls for them; the copies in this client's own
    // settings window drive a chat panel that is not shown at all while
    // FrameXML owns chat, which is every run by default.
    {"joingeneral", "General", SettingKind::Bool, 0, 0, 0, "Chat", "Channels to join",
     "The zone-wide channel.", "", 1},
    {"jointrade", "Trade", SettingKind::Bool, 0, 0, 0, "Chat", "",
     "City-wide, and only in a city.", "", 1},
    {"joinlocaldefense", "LocalDefense", SettingKind::Bool, 0, 0, 0, "Chat", "",
     "Attacks on your zone.", "", 1},
    {"joinlfg", "LookingForGroup", SettingKind::Bool, 0, 0, 0, "Chat", "", "", "", 1},
    {"joinlocal", "Local", SettingKind::Bool, 0, 0, 0, "Chat", "", "", "", 1},

    // ---------------------------------------------------------------- Gameplay
    {"autoloot", "Auto loot", SettingKind::Bool, 0, 0, 0, "Gameplay", "Looting",
     "Take everything from a corpse without opening the window.", "", 0},
    {"autosellgrey", "Sell grey items", SettingKind::Bool, 0, 0, 0, "Gameplay", "",
     "Sell your grey items whenever you open a merchant.", "", 0},
    {"autorepair", "Repair at vendors", SettingKind::Bool, 0, 0, 0, "Gameplay", "",
     "Repair whenever you open a merchant who can.", "", 0},
};

}  // namespace

const SettingDesc* clientSettingsSchema(std::size_t& count) {
    count = sizeof(kSchema) / sizeof(kSchema[0]);
    return kSchema;
}

bool settingRange(const std::string& key, float& lo, float& hi) {
    for (const auto& row : kSchema) {
        if (key == row.key) {
            lo = row.minValue;
            hi = row.maxValue;
            return true;
        }
    }
    return false;
}

}  // namespace ui
}  // namespace wowee
