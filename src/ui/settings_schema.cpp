#include "ui/settings_schema.hpp"

#include "ui/graphics_defaults.hpp"

namespace wowee {
namespace ui {

namespace {

// Every setting this client has.
//
// Including the ones FrameXML's own Video, Sound and Interface panels used to
// drive through kClientCVars alone - mouse speed, the minimap clock, friendly
// nameplates, ground clutter, the sound effects volume, and the rows of the
// game's Effects page. Until 2026-09-05 those were left out of here on the
// grounds that a Blizzard control had them, which drew the client's pages and
// the game's own side by side in one window, in different words and different
// units. Each is a row now. Its CVar stays bound, so an addon or a macro
// writing the CVar reaches the same value the row shows, and the Blizzard
// control for it is in kRemoved: two controls for one value is one too many.
//
// The order is the order they are read in: a category is a panel, a section is
// a heading on it, and a setting whose section is "" continues the one above.
// Ranges match the settings window's own sliders, because they are the ranges
// the client clamps to; a control that offers more than the client accepts is a
// control that appears to do nothing at the ends.
constexpr SettingDesc kSchema[] = {
    // ---------------------------------------------------------------- Graphics
    //
    // Tooltips are written for a player, not a renderer: the first line says
    // what changes on screen, the second what it costs or when to want it. A
    // term of art goes in brackets after the plain words, never instead of
    // them. Both panels show the label as the title and these lines under it,
    // and the interface's panel adds the reason when a control is greyed, so
    // no line here needs to say what it depends on.
    {"graphicspreset", "Quality preset", SettingKind::Enum, 0, 4, 1, "Graphics", "Quality",
     "Sets every graphics option at once: view distance, shadows,\n"
     "anti-aliasing, surface detail, ground clutter and grass.\n"
     "Change any one of them afterwards and this reads Custom.",
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
     "How far away things still cast shadows, in yards. The shadow map\n"
     "covers this whole range, so a shorter distance also gives sharper\n"
     "shadows close to you.", "", 300},
    {"viewdistance", "View distance", SettingKind::Float, 400, 2400, 50, "Graphics", "View",
     "How far into the distance the world is drawn, in yards. The single\n"
     "largest cost in the picture: terrain, buildings and creatures are all\n"
     "drawn to this range.", "", 1900},
    // It used to be left out, deferred to the Effects panel of the game's own
    // Video options - as were ground clutter, its radius, brightness,
    // particle and weather detail, environment detail and texture filtering.
    // That panel is retired now and every one of them is a row here. There is
    // one client, and it was drawing two sets of graphics controls into one
    // window: this schema's pages are hosted inside VideoOptionsFrame, so the
    // list a player saw ran Resolution, Effects, Graphics, Grass, Upscaling,
    // Display - the first two Blizzard's and the rest ours, covering much of
    // the same ground in different words and different units.
    //
    // Each of the moved settings is bound to the cvar its old control wrote,
    // in kClientCVars, so an addon or a macro that writes the cvar still
    // reaches the same value this panel shows.
    // No water refraction row on purpose. It is not a choice any more: the
    // shoreline masks, the meniscus at the waterline and the underwater tint are
    // all written against water that refracts, and the flat fallback left them
    // reading against a surface that does not behave the way they assume. The
    // shader keeps its own guard for a frame whose scene copy is not there yet,
    // which is a different thing from a player turning the feature off.
    {"fogstrength", "Fog thickness", SettingKind::Float, 0, 2, 0.05f, "Graphics", "Atmosphere",
     "How heavy the distance fog is, against the zone's own design.\n"
     "1 is as designed. Below 1 thins it and 0 removes it; above 1\n"
     "brings it closer.", "", 0.4f},
    {"fogskyblend", "Fog takes the sky's colour", SettingKind::Float, 0, 1, 0.05f, "Graphics", "",
     "How much distant fog is tinted toward the sky behind it, so the\n"
     "horizon does not stand out pale against a dark sky. 0 uses the\n"
     "zone's fog colour alone; 1 matches the sky.", "", 0.7f},

    // Labelled for what it does, with the term of art in brackets: nobody
    // looks for "multisampling" when their edges are jagged.
    {"antialiasing", "Anti-aliasing (MSAA)", SettingKind::Enum, 0, 3, 1, "Graphics", "Anti-aliasing",
     "Smooths jagged edges by sampling each pixel several times.\n"
     "2x is cheap; 8x costs memory and fill rate at high resolutions.\n"
     "Works alongside FXAA and FSR upscaling.",
     "Off|2x|4x|8x", 1},
    // Two, not off, and not four.
    //
    // Off was the right default for the hardware this game shipped on, and it
    // left a fresh install with no anti-aliasing of any kind - no
    // multisampling, FXAA off, upscaling off. Nothing that can run this
    // renderer at all is troubled by 2x over geometry this light. Not 4x or 8x
    // because the memory is the part that still costs, and an integrated GPU
    // driving a high resolution display is a real case; the panel offers both
    // to anyone who wants them.
    {"fxaa", "Edge smoothing (FXAA)", SettingKind::Bool, 0, 0, 0, "Graphics", "",
     "A cheap smoothing pass over the finished picture. Catches edges\n"
     "MSAA misses, such as leaves and grass, and softens fine texture\n"
     "a little. Most useful with MSAA off or on weaker hardware.", "", 0},

    {"lensflare", "Lens flare", SettingKind::Float, 0, 2, 0.1f, "Graphics", "Sky",
     "How strong the sun's flare is when it is in view. It warms to\n"
     "amber near the horizon for the dawn and dusk look. 0 removes it.",
     "", 1.0f},
    {"sharpstars", "Sharp stars", SettingKind::Bool, 0, 0, 0, "Graphics", "",
     "Draw the night sky's stars as crisp points. Off, they come from\n"
     "the sky's own small star texture, which goes soft at high\n"
     "resolutions.", "", 1},

    // ------------------------------------------------------------------ Detail
    //
    // Its own page because the Graphics page is full: two columns of 384
    // pixels, which test_settings_panel_layout measures, and Graphics uses
    // most of them. These are the settings that describe how much of the
    // world is drawn rather than how it is lit.
    {"groundclutter", "Ground clutter", SettingKind::Int, 0, 150, 5, "Detail", "Ground cover",
     "How many small plants, tufts and stones are scattered over the\n"
     "ground, as a percentage of what the zone asks for. 100 is as\n"
     "designed; above it is denser than the original client drew.", "", 70},
    {"groundclutterdistance", "Ground clutter distance", SettingKind::Int, 70, 300, 10,
     "Detail", "",
     "How far out that clutter is drawn, in yards. Past this it simply\n"
     "is not there, so a short distance is a visible edge on open ground.", "", 140},

    {"normalmapping", "Surface bumps (normal mapping)", SettingKind::Bool, 0, 0, 0, "Detail", "Surfaces",
     "Light stone, wood, cloth and metal by their surface texture, so\n"
     "they catch the light like the real material rather than flat paint.", "", 1},
    {"normalmapstrength", "Bump strength", SettingKind::Float, 0, 2, 0.1f, "Detail", "",
     "How pronounced those surface bumps look. 1 is as the textures\n"
     "were made; higher exaggerates them.", "", 0.8f, "normalmapping"},
    {"parallax", "Surface depth (parallax)", SettingKind::Bool, 0, 0, 0, "Detail", "",
     "Gives bricks, cobbles and planks real depth when seen at an\n"
     "angle, so mortar lines sink and stones stand out.", "", 1},
    {"parallaxquality", "Surface depth quality", SettingKind::Enum, 0, 2, 1, "Detail", "",
     "How finely each surface is traced for that depth: 16, 32 or 64\n"
     "steps. Higher looks steadier up close and costs more on big walls.",
     "Low|Medium|High", 1, "parallax"},

    {"particledensity", "Particle density", SettingKind::Int, 10, 100, 5, "Detail", "Effects",
     "How many particles a spell, fire or waterfall throws, as a\n"
     "percentage of what it asks for. Lower thins every effect at once.", "", 100},
    {"weatherdetail", "Weather", SettingKind::Enum, 0, 3, 1, "Detail", "",
     "How heavy rain and snow fall. Off draws no weather at all, which\n"
     "leaves the sky and the sound of it and nothing in the air.",
     "Off|Light|Medium|Full", 3},
    {"environmentdetail", "Object detail", SettingKind::Int, 50, 150, 5, "Detail", "",
     "How far out doodads - crates, bushes, lamps, fences - keep being\n"
     "drawn, as a percentage. Lower empties the middle distance first.", "", 100},

    {"texturefiltering", "Texture filtering", SettingKind::Enum, 0, 4, 1, "Detail", "Textures",
     "How sharp textures stay when seen at a shallow angle - a road\n"
     "ahead, a floor underfoot. Costs little on any modern card, and\n"
     "applies to textures loaded from here on.",
     "Off|2x|4x|8x|16x", 4},

    // --------------------------------------------------------------- Upscaling
    {"upscaling", "Upscaling", SettingKind::Enum, 0, 2, 1, "Upscaling", "Mode",
     "Draw the world smaller, then scale it up to fill your screen,\n"
     "for more frames a second. FSR 1 is a cheap sharpening upscale.\n"
     "FSR 3 rebuilds detail from earlier frames: sharper and steadier,\n"
     "and it smooths edges too. Both work with MSAA and FXAA.",
     "Off|FSR 1 (spatial, cheap)|FSR 3 (temporal, sharper)", 0},
    {"fsrquality", "Render resolution", SettingKind::Enum, 0, 3, 1, "Upscaling", "",
     "How large the world is drawn before upscaling, as a share of your\n"
     "screen. Smaller is faster and softer. Native keeps full size and\n"
     "uses FSR 3 for its edge smoothing alone.",
     "Ultra Quality (77%)|Quality (67%)|Balanced (59%)|Native (100%)", 3, "upscaling!=0"},
    {"fsrsharpness", "Sharpening", SettingKind::Float, 0, 2, 0.1f, "Upscaling", "",
     "Sharpening applied after the upscale. Raise it if the picture\n"
     "reads soft; too far and edges grow bright halos.", "", 1.6f, "upscaling!=0"},
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
    {"grassenabled", "Grass (experimental)", SettingKind::Bool, 0, 0, 0,
     "Grass", "Ground cover (experimental)",
     "Grow grass from the terrain's own ground-effect data, bending in\n"
     "the wind and parting as you walk through it. Experimental: it\n"
     "costs time on the main thread as you move and has known faults.", "", 0},
    {"grassdensity", "Grass density", SettingKind::Float, 0, 300, 5, "Grass", "",
     "How much grass grows, as a percentage of what the terrain asks\n"
     "for. 100 is as designed.", "", 100, "grassenabled"},
    {"grassheight", "Grass height", SettingKind::Float, 50, 300, 5, "Grass", "",
     "How tall the blades grow, as a percentage. Taller grass shows\n"
     "the wind crossing it more.", "", 100, "grassenabled"},
    {"grassdistance", "Grass distance", SettingKind::Float, 30, 2000, 5, "Grass", "",
     "How far out grass is drawn, in yards. Beyond 45 the field thins\n"
     "with distance and each blade fades in as you ride toward it.\n"
     "Farther costs more each time you move.", "", 150, "grassenabled"},

#if WOWEE_HAS_AMD_FSR3_FRAMEGEN
    {"framegen", "Frame generation", SettingKind::Bool, 0, 0, 0, "Upscaling", "",
     "Insert a generated frame between each pair of real ones. FSR 3\n"
     "only. Experimental, and known broken on RADV/Mesa.",
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
     "Fill the screen instead of running in a window. Applies the next\n"
     "time the window is rebuilt, or at the next start.", "", 0},
    {"vsync", "Vertical sync", SettingKind::Bool, 0, 0, 0, "Display", "",
     "Show each frame in step with the display. Removes tearing and\n"
     "caps the frame rate at the display's refresh rate.", "", 1},
    {"framecap", "Frame rate limit", SettingKind::Enum, 0, 6, 1, "Display", "",
     "The most frames drawn each second - the fps cap. Unlimited runs\n"
     "the hardware flat out, which on a laptop means heat and fan noise\n"
     "for frames nobody sees. Vertical sync already caps at the display's\n"
     "rate; use this to cap below it.",
     "Unlimited|30|60|90|120|144|240", 0},
    {"brightness", "Brightness", SettingKind::Int, 0, 100, 5, "Display", "",
     "How bright the picture is. 50 leaves it as the zone was lit; below\n"
     "darkens and above lifts the shadows. The game's own panel called\n"
     "this gamma and reached the same value.", "", 50},
    // Gamma, on the game's own panel, is this same value on another scale -
    // GetGamma answers it divided by 50 - and both end in SetGamma, so the
    // two cannot disagree whichever a player reaches for.

    // ------------------------------------------------------------------ Camera
    {"fov", "Field of view", SettingKind::Float, 45, 110, 1, "Camera", "View",
     "How wide a view the camera takes, in degrees. 70 is what the\n"
     "original client shows; wider fits more in and stretches the edges.", "", 70},
    // The client shakes the camera for spell effects and for thunderstorms, and
    // there was no control over it. There would not have been in 2004 - the
    // idea that this is something to offer is newer than the game - and it is
    // standard now, because for some people it is the difference between
    // playing and feeling ill. Defaults to the full amount, so nobody's picture
    // changes until they ask.
    {"camerashake", "Camera shake", SettingKind::Float, 0, 1, 0.05f, "Camera", "",
     "How much the view moves on its own: spell effects, thunder, and\n"
     "the sway while drunk. 0 stops it. Walking crooked while drunk is\n"
     "not affected - that happens to your character, not the picture.", "", 1.0f},
    // Max Camera Distance was the game's own Camera page's until 2026-09-06,
    // when it was the one control on that page not already a row here - the
    // other two, follow speed and camera style, wrote camerastiffness and
    // smoothfollow. In yards rather than the multiple of 22 the game's slider
    // counted in; kClientCVars scales cameraDistanceMaxFactor by 22 for an
    // addon or a macro that still writes it. Before either there was an
    // extended-zoom checkbox of this client's own, a choice between two
    // positions where this is every distance between them.
    {"cameramaxdistance", "Max camera distance", SettingKind::Int, 22, 50, 1, "Camera", "",
     "How far back the camera can be pulled, in yards. 22 is what the\n"
     "original client gives; further out shows more of a fight and less\n"
     "of your character.", "", 22},
    {"camerastiffness", "Camera follow speed", SettingKind::Float, 5, 100, 1, "Camera", "",
     "How quickly the camera catches up when you move or turn. Higher\n"
     "is tighter and steadier; lower trails behind and feels floaty.", "", 30},
    {"pivotheight", "Camera pivot height", SettingKind::Float, 0, 3, 0.1f, "Camera", "",
     "The point the camera turns around, as a height above your feet in\n"
     "yards. Lower feels closer to the character; higher looks over it.", "", 1.6f},
    {"smoothfollow", "Smooth follow", SettingKind::Bool, 0, 0, 0, "Camera", "",
     "Ease the camera round behind you as you turn, rather than\n"
     "snapping straight to it.", "", 0},
    {"idleorbit", "Idle orbit", SettingKind::Bool, 0, 0, 0, "Camera", "",
     "Slowly circle the camera around you while you stand still.", "", 1},
    {"mousespeed", "Mouse sensitivity", SettingKind::Float, 0.05f, 1.0f, 0.05f, "Camera", "Mouse",
     "How far the view turns for a given movement of the mouse. The game's\n"
     "own panel offered this twice, as sensitivity and as look speed, and\n"
     "both wrote this one value.", "", 0.2f},
    {"invertmouse", "Invert mouse look", SettingKind::Bool, 0, 0, 0, "Camera", "",
     "Push the mouse forward to look up, as in a flight sim.", "", 0},

    // --------------------------------------------------------------- Interface
    //
    // From here to Gameplay, the rows on a store - see SettingDesc::store -
    // came off the game's own Interface pages on 2026-09-06: Controls,
    // Combat, Display, Objectives, Social, Names, Combat Text, Status Text,
    // Unit Frames, Buffs, Features and Help. Those pages are retired, and
    // every control on them that this client or its FrameXML reads is a row
    // here, written exactly as the control wrote it: the same CVar, the
    // global the interface reads in `after`, and the refresh that made it
    // show. The rows without a store are this client's own and were here
    // before. Not carried over: the movable world map, which this client's
    // own map does not read, and the locale list, which has one entry.
    {"uiopacity", "Window opacity", SettingKind::Int, 20, 100, 5, "Interface", "Windows",
     "How solid this client's own windows are: settings, meters and\n"
     "the like. 100 is opaque.", "", 65},
    // Up to 3x because a phone needs it: the same 1080 lines that are an
    // ordinary monitor are a 420 dpi panel held at arm's length, and 1.5 does
    // not reach. Harmless on a desktop, where nobody drags it that far.
    {"windowuiscale", "Window scale", SettingKind::Float, 0.75f, 3.0f, 0.05f, "Interface", "",
     "Size of the text and controls in this client's own windows. The\n"
     "game interface's own scale is in the game's Video panel.", "", 1},
    {"latencymeter", "Latency meter", SettingKind::Bool, 0, 0, 0, "Interface", "",
     "Show your ping - the round trip to the server - beside the minimap.", "", 1},
    {"micromenu", "Micro menu buttons", SettingKind::Bool, 0, 0, 0, "Interface", "",
     "The row of shortcut buttons to the character sheet, spellbook,\n"
     "talents and the rest.", "", 0},

    {"bagscale", "Bag scale", SettingKind::Float, 0.75f, 1.5f, 0.05f, "Interface", "Bags",
     "Size of the bag windows.", "", 1},
    {"separatebags", "Separate bag windows", SettingKind::Bool, 0, 0, 0, "Interface", "",
     "Open each bag in its own window, rather than all in one.", "", 1},
    {"showkeyring", "Show keyring", SettingKind::Bool, 0, 0, 0, "Interface", "",
     "Show the key ring button beside the bags.", "", 1},
    {"freebagslots", "Free space on the backpack", SettingKind::Bool, 0, 0, 0, "Interface", "",
     "Count the empty slots across all your bags on the backpack button.",
     "", 0, "", "cvar:displayFreeBagSlots"},

    {"enhancedtooltips", "Enhanced tooltips", SettingKind::Bool, 0, 0, 0, "Interface", "Tooltips",
     "Fuller tooltips on abilities: what one does in words, not only\n"
     "its numbers.", "", 1, "", "cvar:UberTooltips"},
    {"beginnertips", "Beginner tips", SettingKind::Bool, 0, 0, 0, "Interface", "",
     "Explain the basics in tooltips - what each part of the interface\n"
     "is for. Worth turning off once you know.", "", 1, "", "cvar:showNewbieTips",
     "", "SHOW_NEWBIE_TIPS = v"},
    {"colorblindmode", "Colourblind mode", SettingKind::Bool, 0, 0, 0, "Interface", "",
     "Say a unit's reaction and an item's quality in words in tooltips,\n"
     "rather than by colour alone, and add cues elsewhere.", "", 0,
     "", "cvar:colorblindMode", "",
     "ENABLE_COLORBLIND_MODE = v "
     "if WatchFrame_Update then WatchFrame_Update() end "
     "if IsAddOnLoaded and IsAddOnLoaded('Blizzard_AchievementUI') "
     "and AchievementFrame_ForceUpdate then AchievementFrame_ForceUpdate() end"},
    {"showitemlevel", "Item level in tooltips", SettingKind::Bool, 0, 0, 0, "Interface", "",
     "Show an item's level in its tooltip.", "", 1, "", "cvar:showItemLevel"},

    {"equipmentmanager", "Equipment manager", SettingKind::Bool, 0, 0, 0, "Interface", "Character sheet",
     "Save sets of gear on the character sheet and swap between them\n"
     "in one click.", "", 0, "", "cvar:equipmentManager"},
    {"previewtalents", "Preview talent changes", SettingKind::Bool, 0, 0, 0, "Interface", "",
     "Lay talent points out on the talent sheet before spending them,\n"
     "and confirm them all at once.", "", 1, "", "cvar:previewTalents"},

    // ----------------------------------------------------------------- Minimap
    // Rotate-with-camera was deliberately absent until 2026-09-06: the loader
    // dropped the saved value and every run started north-up, pending a look
    // at the rotated map on screen. It is a row now, saved like the rest -
    // the game's Display page that offered it is retired, and the minimap's
    // own menu had been turning it within a session all along.
    {"minimapclock", "Clock", SettingKind::Bool, 0, 0, 0, "Minimap", "Appearance",
     "Show the time of day under the minimap.", "", 0},
    {"minimapsquare", "Square minimap", SettingKind::Bool, 0, 0, 0, "Minimap", "",
     "Draw the minimap as a square rather than a circle.", "", 0},
    {"minimapnpcdots", "Nearby creature dots", SettingKind::Bool, 0, 0, 0, "Minimap", "",
     "Mark creatures near you as dots on the minimap.", "", 0},
    {"minimapcoords", "Coordinates", SettingKind::Bool, 0, 0, 0, "Minimap", "",
     "Show your map coordinates under the minimap.", "", 0},
    {"minimaprotate", "Rotate with the camera", SettingKind::Bool, 0, 0, 0, "Minimap", "",
     "Turn the minimap so the way you are facing is up, rather than\n"
     "north. The minimap's own menu has this too.", "", 0},

    // ------------------------------------------------------------- Action Bars
    // Up to 2, not 1.5: a slot is 48 pixels times this, and the scale a
    // 2160-line screen wants is 2. Stopping at 1.5 meant the control could not
    // ask for what the display needed, and the bars sat smaller than the buff
    // bar beside them however far it was dragged.
    {"actionbarscale", "Action bar scale", SettingKind::Float, 0.5f, 2.0f, 0.05f,
     "Action Bars", "Scale", "Size of every action bar button.", "", 1},
    {"buffbarscale", "Buff bar scale", SettingKind::Float, 0.75f, 1.5f, 0.05f,
     "Action Bars", "", "Size of the buff and debuff icons, on top of the automatic\n"
     "scaling for your display.", "", 1},
    {"showbar2", "Bottom left bar", SettingKind::Bool, 0, 0, 0, "Action Bars", "Extra bars",
     "A second bar above the main one. Holds the game's action page 6,\n"
     "where the original client keeps this bar.", "", 0},
    {"bar2offsetx", "Bottom left bar: move across", SettingKind::Float, -600, 600, 10,
     "Action Bars", "", "Shift that bar left or right of its usual place, in pixels.", "", 0, "showbar2"},
    {"bar2offsety", "Bottom left bar: move up", SettingKind::Float, -400, 400, 10,
     "Action Bars", "", "Shift that bar up or down from its usual place, in pixels.", "", 0, "showbar2"},
    {"showrightbar", "Right side bar", SettingKind::Bool, 0, 0, 0, "Action Bars", "",
     "An upright bar at the right edge of the screen. Holds action\n"
     "page 3.", "", 0},
    {"rightbaroffsety", "Right side bar: move up", SettingKind::Float, -400, 400, 10,
     "Action Bars", "", "Shift it up or down from the middle of the screen, in pixels.", "", 0, "showrightbar"},
    {"showleftbar", "Left side bar", SettingKind::Bool, 0, 0, 0, "Action Bars", "",
     "An upright bar at the left edge of the screen. Holds action\n"
     "page 4.", "", 0},
    {"leftbaroffsety", "Left side bar: move up", SettingKind::Float, -400, 400, 10,
     "Action Bars", "", "Shift it up or down from the middle of the screen, in pixels.", "", 0, "showleftbar"},

    // --------------------------------------------------------------------- HUD
    // Combat & HUD until 2026-09-06, when the fighting rows went to a Combat
    // page of their own and the nameplate rows to Names, and the game's own
    // Buffs page came here.
    {"dpsmeter", "Damage meter", SettingKind::Bool, 0, 0, 0, "HUD", "Trackers",
     "Show your damage and healing per second above the action bar\n"
     "while you are in combat.", "", 0},
    {"cooldowntracker", "Cooldown tracker", SettingKind::Bool, 0, 0, 0, "HUD", "",
     "Show your longer cooldowns counting down beside the action bar.", "", 0},
    {"raretracker", "Rare tracker", SettingKind::Bool, 0, 0, 0, "HUD", "",
     "Mark rare creatures near you on the minimap and the world map.", "", 0},
    {"chesttracker", "Chest tracker", SettingKind::Bool, 0, 0, 0, "HUD", "",
     "Mark treasure chests near you on the minimap.", "", 0},

    {"damageflash", "Damage flash", SettingKind::Bool, 0, 0, 0, "HUD", "Screen effects",
     "Flash a red edge around the screen when you take a hit.", "", 1},
    {"lowhealthvignette", "Low health warning", SettingKind::Bool, 0, 0, 0,
     "HUD", "", "Pulse a red edge around the screen while your health is\n"
     "below a fifth.", "", 1},
    {"screenedgeflash", "Low health edge flash", SettingKind::Bool, 0, 0, 0, "HUD", "",
     "The game's own red flash at the screen's edge while your health\n"
     "is low - beside the warning above, which is this client's.", "", 0,
     "", "cvar:screenEdgeFlash"},

    {"buffdurations", "Time left under buffs", SettingKind::Bool, 0, 0, 0, "HUD", "Buffs",
     "Count down the time left under each buff icon.", "", 1,
     "", "cvar:buffDurations", "",
     "SHOW_BUFF_DURATIONS = v if BuffFrame_UpdatePositions then BuffFrame_UpdatePositions() end"},
    {"dispellabledebuffs", "Highlight debuffs you can remove", SettingKind::Bool, 0, 0, 0, "HUD", "",
     "Frame the debuffs on your target that one of your spells can\n"
     "dispel.", "", 1, "", "cvar:showDispelDebuffs", "", "SHOW_DISPELLABLE_DEBUFFS = v"},
    {"castablebuffs", "Only buffs you can cast", SettingKind::Bool, 0, 0, 0, "HUD", "",
     "On the target frame, show only the buffs you could cast yourself.",
     "", 0, "", "cvar:showCastableBuffs", "", "SHOW_CASTABLE_BUFFS = v"},
    {"castabledebuffs", "Only debuffs you cast", SettingKind::Bool, 0, 0, 0, "HUD", "",
     "On the target frame, show only the debuffs that are yours.", "", 0,
     "", "cvar:showCastableDebuffs", "", "SHOW_CASTABLE_DEBUFFS = v"},

    // ------------------------------------------------------------------ Combat
    // The game's Combat page, less the nameplate rows, which are on Names with
    // the rest of the nameplates. The double-tap guard was on its ActionBars
    // page and is read by this client's own attack handling.
    {"assistattack", "Attack on assist", SettingKind::Bool, 0, 0, 0, "Combat", "Attacking",
     "Start attacking when /assist hands you a hostile target.", "", 0,
     "", "cvar:assistAttack"},
    {"stopautoattack", "Stop attacking when the target changes", SettingKind::Bool, 0, 0, 0,
     "Combat", "",
     "Picking a new target ends your auto-attack rather than carrying it\n"
     "over.", "", 0, "", "cvar:stopAutoAttackOnTargetChange"},
    {"secureabilitytoggle", "Ignore a double tap on Attack", SettingKind::Bool, 0, 0, 0,
     "Combat", "",
     "A second press of Attack within half a second is ignored, so a\n"
     "double tap cannot switch auto-attack straight back off.", "", 0},

    {"autoselfcast", "Cast helpful spells on yourself", SettingKind::Bool, 0, 0, 0, "Combat", "Casting",
     "A helpful spell cast with no friendly target goes on you, rather\n"
     "than waiting for one.", "", 0, "", "cvar:autoSelfCast"},
    {"selfcastkey", "Self cast key", SettingKind::Enum, 0, 3, 1, "Combat", "",
     "Hold this while casting to put the spell on yourself, whatever is\n"
     "targeted.", "None|Alt|Ctrl|Shift", 1, "", "click:SELFCAST", "NONE|ALT|CTRL|SHIFT"},
    {"focuscastkey", "Focus cast key", SettingKind::Enum, 0, 3, 1, "Combat", "",
     "Hold this while casting to put the spell on your focus target\n"
     "instead.", "None|Alt|Ctrl|Shift", 0, "", "click:FOCUSCAST", "NONE|ALT|CTRL|SHIFT"},

    {"targetoftarget", "Target of target", SettingKind::Bool, 0, 0, 0, "Combat", "Target",
     "A small frame beside the target's, showing what it is targeting.",
     "", 0, "", "cvar:showTargetOfTarget", "", "SHOW_TARGET_OF_TARGET = v"},
    {"targetoftargetwhen", "Show it", SettingKind::Enum, 0, 4, 1, "Combat", "",
     "When that frame is worth the room.",
     "Always|In a raid|In a party|Solo|In a raid or party", 0, "targetoftarget",
     "cvar:targetOfTargetMode", "5|1|2|3|4", "SHOW_TARGET_OF_TARGET_STATE = v"},
    {"targetcastbar", "Cast bar on the target frame", SettingKind::Bool, 0, 0, 0, "Combat", "",
     "Show what your target is casting under its frame.", "", 1,
     "", "cvar:showTargetCastbar"},

    // ------------------------------------------------------------------- Names
    // The game's Names page, whole, and the nameplate rows that were on
    // Combat & HUD. The names over heads are this client's own drawing, which
    // picks its CVar by what the unit is; the plates are its own too.
    {"myname", "My name", SettingKind::Bool, 0, 0, 0, "Names", "Names over heads",
     "Your own name over your head.", "", 0, "", "cvar:UnitNameOwn"},
    {"npcnames", "NPC names", SettingKind::Bool, 0, 0, 0, "Names", "",
     "Names over creatures you can talk to, and over hostile ones.", "", 1,
     "", "cvar:UnitNameNPC"},
    {"critternames", "Critters and vanity pets", SettingKind::Bool, 0, 0, 0, "Names", "",
     "Names over rabbits, squirrels and the pets that only follow you.",
     "", 1, "", "cvar:UnitNameNonCombatCreatureName"},
    {"guildnames", "Guild names", SettingKind::Bool, 0, 0, 0, "Names", "",
     "Players' guilds under their names.", "", 1, "", "cvar:UnitNamePlayerGuild"},
    {"playertitles", "Titles", SettingKind::Bool, 0, 0, 0, "Names", "",
     "Players' titles with their names.", "", 1, "", "cvar:UnitNamePlayerPVPTitle"},
    {"friendlynames", "Friendly players", SettingKind::Bool, 0, 0, 0, "Names", "",
     "Names over players on your side.", "", 1, "", "cvar:UnitNameFriendlyPlayerName"},
    {"friendlypetnames", "Friendly pets", SettingKind::Bool, 0, 0, 0, "Names", "",
     "Names over their pets.", "", 1, "", "cvar:UnitNameFriendlyPetName"},
    {"friendlyguardiannames", "Friendly guardians", SettingKind::Bool, 0, 0, 0, "Names", "",
     "Names over their guardians - summoned helpers that fight.", "", 1,
     "", "cvar:UnitNameFriendlyGuardianName"},
    {"friendlytotemnames", "Friendly totems", SettingKind::Bool, 0, 0, 0, "Names", "",
     "Names over their totems.", "", 1, "", "cvar:UnitNameFriendlyTotemName"},
    {"enemynames", "Enemy players", SettingKind::Bool, 0, 0, 0, "Names", "",
     "Names over players of the other faction.", "", 1, "", "cvar:UnitNameEnemyPlayerName"},
    {"enemypetnames", "Enemy pets", SettingKind::Bool, 0, 0, 0, "Names", "",
     "Names over their pets.", "", 1, "", "cvar:UnitNameEnemyPetName"},
    {"enemyguardiannames", "Enemy guardians", SettingKind::Bool, 0, 0, 0, "Names", "",
     "Names over their guardians.", "", 1, "", "cvar:UnitNameEnemyGuardianName"},
    {"enemytotemnames", "Enemy totems", SettingKind::Bool, 0, 0, 0, "Names", "",
     "Names over their totems.", "", 1, "", "cvar:UnitNameEnemyTotemName"},

    {"friendlyplates", "Friendly nameplates", SettingKind::Bool, 0, 0, 0, "Names", "Nameplates",
     "Name and health bars over friendly players and creatures, not\n"
     "only hostile ones. Shift+V toggles this too.", "", 0},
    {"friendlypetplates", "Friendly pets", SettingKind::Bool, 0, 0, 0, "Names", "",
     "Plates over friendly pets.", "", 1, "friendlyplates", "cvar:nameplateShowFriendlyPets"},
    {"friendlyguardianplates", "Friendly guardians", SettingKind::Bool, 0, 0, 0, "Names", "",
     "Plates over friendly guardians.", "", 1, "friendlyplates",
     "cvar:nameplateShowFriendlyGuardians"},
    {"friendlytotemplates", "Friendly totems", SettingKind::Bool, 0, 0, 0, "Names", "",
     "Plates over friendly totems.", "", 0, "friendlyplates", "cvar:nameplateShowFriendlyTotems"},
    {"enemyplates", "Enemy nameplates", SettingKind::Bool, 0, 0, 0, "Names", "",
     "Name and health bars over hostile players and creatures. The V\n"
     "key toggles this too.", "", 1},
    {"enemypetplates", "Enemy pets", SettingKind::Bool, 0, 0, 0, "Names", "",
     "Plates over enemy pets.", "", 1, "enemyplates", "cvar:nameplateShowEnemyPets"},
    {"enemyguardianplates", "Enemy guardians", SettingKind::Bool, 0, 0, 0, "Names", "",
     "Plates over enemy guardians.", "", 1, "enemyplates", "cvar:nameplateShowEnemyGuardians"},
    {"enemytotemplates", "Enemy totems", SettingKind::Bool, 0, 0, 0, "Names", "",
     "Plates over enemy totems.", "", 1, "enemyplates", "cvar:nameplateShowEnemyTotems"},
    {"nameplateoverlap", "Let nameplates overlap", SettingKind::Bool, 0, 0, 0, "Names", "",
     "Plates over units standing in a line may cover each other. Off,\n"
     "each is pushed clear of the others.", "", 0, "", "cvar:nameplateAllowOverlap"},
    {"nameplateclasscolours", "Class colours for enemy players", SettingKind::Bool, 0, 0, 0,
     "Names", "", "Colour an enemy player's health bar by their class.", "", 0,
     "", "cvar:ShowClassColorInNameplate"},
    {"nameplatecastbar", "Cast bar on the target's plate", SettingKind::Bool, 0, 0, 0, "Names", "",
     "Show what your target is casting under its nameplate.", "", 1,
     "", "cvar:showVKeyCastbar"},
    {"nameplatescale", "Nameplate scale", SettingKind::Float, 0.5f, 2.0f, 0.05f, "Names", "",
     "Size of the name and health bars over creatures' heads.", "", 1},

    // ------------------------------------------------------------- Combat Text
    // Blizzard_CombatText's own switches, which the game's page held. The six
    // engine-side rows that page also had are not here: this client floats
    // its own damage and healing and reads none of them.
    {"combattext", "Floating combat text", SettingKind::Bool, 0, 0, 0, "Combat Text", "Over your character",
     "Float the notices below over your character as they happen.", "", 1,
     "", "cvar:enableCombatText", "", "SHOW_COMBAT_TEXT = v"},
    {"combattextmode", "Direction", SettingKind::Enum, 0, 2, 1, "Combat Text", "",
     "Which way the text moves.", "Scroll up|Scroll down|Arc", 0, "combattext",
     "cvar:combatTextFloatMode", "1|2|3",
     "COMBAT_TEXT_FLOAT_MODE = v "
     "if CombatText_UpdateDisplayedMessages then CombatText_UpdateDisplayedMessages() end"},
    {"fctdodgeparrymiss", "Dodges, parries and misses", SettingKind::Bool, 0, 0, 0, "Combat Text", "",
     "Say when a swing at you missed, and why.", "", 1, "combattext",
     "cvar:fctDodgeParryMiss", "", "COMBAT_TEXT_SHOW_DODGE_PARRY_MISS = v"},
    {"fctdamagereduction", "Damage reduction", SettingKind::Bool, 0, 0, 0, "Combat Text", "",
     "How much of a hit your armour, resistances and absorbs took off.",
     "", 0, "combattext", "cvar:fctDamageReduction", "", "COMBAT_TEXT_SHOW_RESISTANCES = v"},
    {"fctrepchanges", "Reputation changes", SettingKind::Bool, 0, 0, 0, "Combat Text", "",
     "Reputation gained and lost.", "", 0, "combattext",
     "cvar:fctRepChanges", "", "COMBAT_TEXT_SHOW_REPUTATION = v"},
    {"fctreactives", "Reactive abilities", SettingKind::Bool, 0, 0, 0, "Combat Text", "",
     "When an ability that needs a trigger - a dodge, a critical - comes\n"
     "up.", "", 0, "combattext", "cvar:fctReactives", "", "COMBAT_TEXT_SHOW_REACTIVES = v"},
    {"fctfriendlyhealers", "Friendly healer names", SettingKind::Bool, 0, 0, 0, "Combat Text", "",
     "Who healed you, with the amount.", "", 0, "combattext",
     "cvar:fctFriendlyHealers", "", "COMBAT_TEXT_SHOW_FRIENDLY_NAMES = v"},
    {"fctcombatstate", "Entering and leaving combat", SettingKind::Bool, 0, 0, 0, "Combat Text", "",
     "A notice as a fight starts and as it ends.", "", 0, "combattext",
     "cvar:fctCombatState", "", "COMBAT_TEXT_SHOW_COMBAT_STATE = v"},
    {"fctcombopoints", "Combo points", SettingKind::Bool, 0, 0, 0, "Combat Text", "",
     "Each combo point as it is earned.", "", 0, "combattext",
     "cvar:fctComboPoints", "", "COMBAT_TEXT_SHOW_COMBO_POINTS = v"},
    {"fctlowmanahealth", "Low health and mana", SettingKind::Bool, 0, 0, 0, "Combat Text", "",
     "A warning as either runs low.", "", 1, "combattext",
     "cvar:fctLowManaHealth", "", "COMBAT_TEXT_SHOW_LOW_HEALTH_MANA = v"},
    {"fctenergygains", "Energy gains", SettingKind::Bool, 0, 0, 0, "Combat Text", "",
     "Mana, rage, energy and runic power as it comes in.", "", 0, "combattext",
     "cvar:fctEnergyGains", "", "COMBAT_TEXT_SHOW_ENERGIZE = v"},
    {"fctperiodicenergygains", "Periodic energy gains", SettingKind::Bool, 0, 0, 0, "Combat Text", "",
     "The same for the trickle from regeneration and effects over time.",
     "", 0, "combattext", "cvar:fctPeriodicEnergyGains", "", "COMBAT_TEXT_SHOW_PERIODIC_ENERGIZE = v"},
    {"fcthonorgains", "Honor gained", SettingKind::Bool, 0, 0, 0, "Combat Text", "",
     "Honor as it is earned.", "", 0, "combattext",
     "cvar:fctHonorGains", "", "COMBAT_TEXT_SHOW_HONOR_GAINED = v"},
    {"fctauras", "Auras gained and lost", SettingKind::Bool, 0, 0, 0, "Combat Text", "",
     "Each buff and debuff as it lands on you and as it fades.", "", 0, "combattext",
     "cvar:fctAuras", "", "COMBAT_TEXT_SHOW_AURAS = v COMBAT_TEXT_SHOW_AURA_FADE = v"},

    // ------------------------------------------------------------- Unit Frames
    // The game's Unit Frames and Status Text pages, and the threat rows from
    // its Display page - all of it about the frames FrameXML draws.
    {"partybackground", "Background behind the party", SettingKind::Bool, 0, 0, 0,
     "Unit Frames", "Party", "A dark panel behind the party frames.", "", 0,
     "", "cvar:showPartyBackground", "", "SHOW_PARTY_BACKGROUND = v"},
    {"hidepartyinraid", "Hide the party in a raid", SettingKind::Bool, 0, 0, 0, "Unit Frames", "",
     "Show only the raid frames while you are in one.", "", 0,
     "", "cvar:hidePartyInRaid", "", "HIDE_PARTY_INTERFACE = v"},
    {"partypets", "Party members' pets", SettingKind::Bool, 0, 0, 0, "Unit Frames", "",
     "A small frame under each party member for their pet.", "", 1,
     "", "cvar:showPartyPets", "", "SHOW_PARTY_PETS = v"},

    {"arenaframes", "Arena enemy frames", SettingKind::Bool, 0, 0, 0, "Unit Frames", "Arena",
     "Frames for the other team in an arena match.", "", 0,
     "", "cvar:showArenaEnemyFrames", "", "SHOW_ARENA_ENEMY_FRAMES = v"},
    {"arenapets", "Their pets", SettingKind::Bool, 0, 0, 0, "Unit Frames", "",
     "Frames for the other team's pets as well.", "", 1, "arenaframes",
     "cvar:showArenaEnemyPets", "", "SHOW_ARENA_ENEMY_PETS = v"},

    {"fullsizefocus", "Full-size focus frame", SettingKind::Bool, 0, 0, 0, "Unit Frames", "Focus",
     "Draw the focus frame at the target frame's size rather than\n"
     "smaller.", "", 0, "", "cvar:fullSizeFocusFrame"},

    {"playerstatustext", "On your frame", SettingKind::Bool, 0, 0, 0, "Unit Frames",
     "Health and power numbers", "Write the numbers on your own health and power bars.", "", 1,
     "", "cvar:playerStatusText"},
    {"petstatustext", "On your pet's frame", SettingKind::Bool, 0, 0, 0, "Unit Frames", "",
     "The same on your pet's bars.", "", 1, "", "cvar:petStatusText"},
    {"partystatustext", "On party frames", SettingKind::Bool, 0, 0, 0, "Unit Frames", "",
     "The same on each party member's bars.", "", 1, "", "cvar:partyStatusText"},
    {"targetstatustext", "On the target frame", SettingKind::Bool, 0, 0, 0, "Unit Frames", "",
     "The same on your target's bars.", "", 1, "", "cvar:targetStatusText"},
    {"statustextpercent", "As percentages", SettingKind::Bool, 0, 0, 0, "Unit Frames", "",
     "Percentages rather than the amounts.", "", 0, "", "cvar:statusTextPercentage"},
    {"xpbartext", "On the experience bar", SettingKind::Bool, 0, 0, 0, "Unit Frames", "",
     "Write your experience on the bar, not only when the mouse is\n"
     "over it.", "", 0, "", "cvar:xpBarText"},

    {"threatwarning", "Threat warning", SettingKind::Enum, 0, 3, 1, "Unit Frames", "Threat",
     "When to flash the target frame as a creature turns on you.",
     "Never|In dungeons and raids|In a party|Always", 3, "", "cvar:threatWarning", "0|1|2|3"},
    {"threatpercent", "Threat as a percentage", SettingKind::Bool, 0, 0, 0, "Unit Frames", "",
     "Show how close you are to pulling your target off whoever has it.",
     "", 0, "", "cvar:threatShowNumeric", "",
     "if InterfaceOptionsDisplayPanelShowAggroPercentage_SetFunc then "
     "InterfaceOptionsDisplayPanelShowAggroPercentage_SetFunc() end"},
    {"threatsounds", "Threat sounds", SettingKind::Bool, 0, 0, 0, "Unit Frames", "",
     "Play a sound as your threat rises to the next level.", "", 0,
     "", "cvar:threatPlaySounds"},


    // ------------------------------------------------------------------- Sound
    //
    // The game's own Sound panel is retired, so its switches are rows here.
    // Master volume, the mute and the sound-effects scale used to be written
    // out by hand in this client's window and driven by that panel as well -
    // one value with two controls, described differently by each.
    //
    // The six switches are bound to their cvars in kClientCVars, because the
    // audio code applies them from the cvar store; writing the setting writes
    // the store, and the panel asks for them to be re-applied.
    {"mutesound", "Mute all sound", SettingKind::Bool, 0, 0, 0, "Sound", "Output",
     "Silence everything at once. The speaker button beside the minimap is\n"
     "the same switch.", "", 0},
    {"mastervolume", "Master volume", SettingKind::Int, 0, 100, 5, "Sound", "",
     "One scale over everything below.", "", 100},
    {"soundinbackground", "Play while in the background", SettingKind::Bool, 0, 0, 0,
     "Sound", "",
     "Keep playing when another window has the focus.", "", 0},

    {"enablemusic", "Enable music", SettingKind::Bool, 0, 0, 0, "Sound", "Music",
     "Off silences the score without disturbing the volume below.", "", 1},
    {"musicvolume", "Music volume", SettingKind::Int, 0, 100, 5, "Sound", "",
     "The zone's music. With the WoWee soundtrack on, that too.", "", 30},
    {"woweemusic", "WoWee soundtrack", SettingKind::Bool, 0, 0, 0, "Sound", "",
     "Play this client's own music in the rotation alongside the game's.", "", 1},
    {"loopmusic", "Play without gaps", SettingKind::Bool, 0, 0, 0, "Sound", "",
     "Start the next track straight away rather than leaving the long\n"
     "silence between them the original client had.", "", 0},

    {"enableambience", "Enable ambience", SettingKind::Bool, 0, 0, 0, "Sound", "Ambience",
     "Off silences the world's own noise without disturbing the volumes below.", "", 1},
    {"ambientvolume", "Ambience volume", SettingKind::Int, 0, 100, 5, "Sound", "",
     "Wind, water, birds and the rest of the world's own noise.", "", 100},
    {"bellvolume", "City bells", SettingKind::Int, 0, 100, 5, "Sound", "",
     "The hour struck in the capital cities.", "", 50},

    {"npcvoicevolume", "NPC voices", SettingKind::Int, 0, 100, 5, "Sound", "Voices",
     "Greetings, farewells and quest speech from the people you talk to.", "", 100},
    {"characterspeech", "Character speech", SettingKind::Bool, 0, 0, 0, "Sound", "",
     "Your own character's grunts, greetings and emotes.", "", 1},
    {"errorspeech", "Spoken refusals", SettingKind::Bool, 0, 0, 0, "Sound", "",
     "Your character saying why an action was refused - \"I can't do that\n"
     "yet\", \"Not enough rage\".", "", 1},

    // ----------------------------------------------------------- Sound Effects
    //
    // Their own page for the reason Detail has one: nine sliders and a switch
    // do not fit beside the rest of Sound in two columns.
    {"enablesoundeffects", "Enable sound effects", SettingKind::Bool, 0, 0, 0,
     "Sound Effects", "Effects",
     "Off silences every effect below at once.", "", 1},
    {"effectsvolume", "All effects", SettingKind::Int, 0, 100, 5, "Sound Effects", "",
     "One scale over the seven below. This is the slider the game's own\n"
     "Sound panel called Sound Effects.", "", 100},
    {"uivolume", "Interface", SettingKind::Int, 0, 100, 5, "Sound Effects", "",
     "Clicks, bag sounds and window noises.", "", 100},
    {"combatvolume", "Combat", SettingKind::Int, 0, 100, 5, "Sound Effects", "",
     "Weapon swings, hits, blocks and parries.", "", 100},
    {"spellvolume", "Spells", SettingKind::Int, 0, 100, 5, "Sound Effects", "",
     "Casting, spell impacts and buffs landing.", "", 100},
    {"movementvolume", "Movement", SettingKind::Int, 0, 100, 5, "Sound Effects", "",
     "Jumping, landing, swimming and armour rustle.", "", 100},
    {"footstepvolume", "Footsteps", SettingKind::Int, 0, 100, 5, "Sound Effects", "",
     "Your own and others' footsteps, by the ground underfoot.", "", 100},
    {"mountvolume", "Mounts", SettingKind::Int, 0, 100, 5, "Sound Effects", "",
     "Hoofbeats, wingbeats and mount calls.", "", 70},
    {"activityvolume", "Activity", SettingKind::Int, 0, 100, 5, "Sound Effects", "",
     "Fishing, mining, forges and the rest of the world at work.", "", 100},

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
    // The one piece of chat's appearance that is here, because it is not
    // appearance: with the box hidden there is nothing to click, and the only
    // way in is a key. WoW calls the two "classic" and "im" and this is that
    // switch, phrased as what it does rather than as what Blizzard named it.
    {"chatboxvisible", "Chat box always visible", SettingKind::Bool, 0, 0, 0,
     "Chat", "Input",
     "Keep the chat input box on screen so it can be clicked into.\n"
     "Off, it appears when you press Enter and hides when you send.",
     "", 1},
    {"clicktofocus", "Click anywhere to type", SettingKind::Bool, 0, 0, 0, "Chat", "",
     "A click anywhere in the chat window puts the cursor in the input\n"
     "box, not only a click on the box.", "", 0, "", "cvar:wholeChatWindowClickable"},
    {"chatmousescroll", "Scroll with the mouse wheel", SettingKind::Bool, 0, 0, 0, "Chat", "",
     "The wheel scrolls the chat window while the mouse is over it.", "", 1,
     "", "cvar:chatMouseScroll", "",
     "if InterfaceOptionsSocialPanelChatMouseScroll_SetScrolling then "
     "InterfaceOptionsSocialPanelChatMouseScroll_SetScrolling(v) end"},
    {"removechatdelay", "No delay before links light up", SettingKind::Bool, 0, 0, 0, "Chat", "",
     "Item and player links answer the moment the mouse is over them,\n"
     "rather than after a pause.", "", 0, "", "cvar:removeChatDelay", "",
     "REMOVE_CHAT_DELAY = v"},

    {"chattimestamps", "Timestamps", SettingKind::Enum, 0, 4, 1, "Chat", "Messages",
     "Put the time in front of each line.",
     "None|3:45|3:45:12|3:45 pm|3:45:12 pm", 0, "", "cvar:showTimestamps",
     "none|%I:%M |%I:%M:%S |%I:%M %p |%I:%M:%S %p ",
     "if v == 'none' then CHAT_TIMESTAMP_FORMAT = nil else CHAT_TIMESTAMP_FORMAT = v end"},
    {"spamfilter", "Drop repeated lines", SettingKind::Bool, 0, 0, 0, "Chat", "",
     "A line pasted again a moment later by the same player is not\n"
     "shown.", "", 1, "", "cvar:spamFilter"},
    {"lootrollmessages", "Every loot roll", SettingKind::Bool, 0, 0, 0, "Chat", "",
     "Say each player's roll on an item, not only who won it.", "", 1,
     "", "cvar:showLootSpam"},
    {"guildmemberalerts", "Guild members coming and going", SettingKind::Bool, 0, 0, 0, "Chat", "",
     "A line when a guild member logs in or out.", "", 1, "", "cvar:guildMemberNotify"},

    {"joingeneral", "General", SettingKind::Bool, 0, 0, 0, "Chat", "Channels to join",
     "Everyone in your current zone.", "", 1},
    {"jointrade", "Trade", SettingKind::Bool, 0, 0, 0, "Chat", "",
     "Buying and selling. Shared between the capital cities, and only\n"
     "heard while you are in one.", "", 1},
    {"joinlocaldefense", "LocalDefense", SettingKind::Bool, 0, 0, 0, "Chat", "",
     "Alerts when towns in your zone come under attack.", "", 1},
    {"joinlfg", "LookingForGroup", SettingKind::Bool, 0, 0, 0, "Chat", "",
     "Finding people for dungeons, quests and raids.", "", 1},
    {"joinlocal", "Local", SettingKind::Bool, 0, 0, 0, "Chat", "",
     "The realm's Local channel, on realms that provide one.", "", 1},
    {"joinguildrecruitment", "GuildRecruitment", SettingKind::Bool, 0, 0, 0, "Chat", "",
     "Guilds looking for members, and players looking for a guild.", "", 0,
     "", "cvar:guildRecruitmentChannel"},
    // Speech bubbles were the game's own Social page's, two boxes among chat
    // options FrameXML reads for itself. These two the client reads - the
    // bubbles are drawn here - so they are rows. Party bubbles want both.
    {"chatbubbles", "Speech bubbles", SettingKind::Bool, 0, 0, 0, "Chat", "Bubbles",
     "Float what is said and yelled nearby over the speaker's head.", "", 1},
    {"chatbubblesparty", "Party chat in bubbles", SettingKind::Bool, 0, 0, 0, "Chat", "",
     "Float party and raid chat over the speaker's head as well.",
     "", 0, "chatbubbles"},

    // ---------------------------------------------------------------- Gameplay
    {"autoloot", "Auto loot", SettingKind::Bool, 0, 0, 0, "Gameplay", "Looting",
     "Take everything from a corpse or chest without opening the loot\n"
     "window.", "", 0},
    {"autolootkey", "Auto loot key", SettingKind::Enum, 0, 3, 1, "Gameplay", "",
     "Hold this while opening a corpse to do the opposite of the setting\n"
     "above, for that one corpse.", "None|Alt|Ctrl|Shift", 3, "",
     "click:AUTOLOOTTOGGLE", "NONE|ALT|CTRL|SHIFT"},
    {"lootatmouse", "Loot window at the mouse", SettingKind::Bool, 0, 0, 0, "Gameplay", "",
     "Open the loot window where the mouse is, rather than in its usual\n"
     "place.", "", 0, "", "cvar:lootUnderMouse", "", "LOOT_UNDER_MOUSE = v"},
    {"autosellgrey", "Sell junk automatically", SettingKind::Bool, 0, 0, 0, "Gameplay", "",
     "Opening any merchant sells every Poor quality item - the grey ones -\n"
     "from your backpack and bags, and adds the coin. Nothing of another\n"
     "colour is touched. There is no prompt and no undo, so a grey you\n"
     "were keeping on purpose goes with the rest.", "", 0},
    {"autorepair", "Repair automatically", SettingKind::Bool, 0, 0, 0, "Gameplay", "",
     "Opening a merchant who repairs mends everything damaged you are\n"
     "wearing, paid from your own coin - never the guild bank. If you\n"
     "cannot afford the whole bill nothing is repaired and it says so.", "", 0},
    // Secure Ability Toggle was on the game's own ActionBars page, the one
    // control there this client reads: the guard is in the client's own
    // attack handling, and FrameXML's action buttons are not the ones drawn.

    {"groundclearstarget", "Clicking the ground clears the target", SettingKind::Bool, 0, 0, 0,
     "Gameplay", "Controls",
     "A click on nothing drops your target. Off, the target stays until\n"
     "you pick another - what the game calls sticky targeting.", "", 1,
     "", "cvar:deselectOnClick"},
    {"autodismount", "Dismount to cast in flight", SettingKind::Bool, 0, 0, 0, "Gameplay", "",
     "Casting on a flying mount dismounts you first, rather than\n"
     "refusing the cast.", "", 0, "", "cvar:autoDismountFlying"},
    {"autoclearafk", "Leave Away on moving", SettingKind::Bool, 0, 0, 0, "Gameplay", "",
     "Moving or talking takes you out of Away.", "", 1, "", "cvar:autoClearAFK"},
    {"blocktrades", "Refuse trade requests", SettingKind::Bool, 0, 0, 0, "Gameplay", "",
     "Trade requests are refused before their window opens, and chat\n"
     "says so.", "", 0, "", "cvar:blockTrades"},

    {"instantquesttext", "Show quest text at once", SettingKind::Bool, 0, 0, 0, "Gameplay", "Quests",
     "Quest text appears whole, rather than being revealed line by\n"
     "line.", "", 0, "", "cvar:questFadingDisable", "", "QUEST_FADING_DISABLE = v"},
    {"autotrackquests", "Track a quest when you accept it", SettingKind::Bool, 0, 0, 0,
     "Gameplay", "", "A new quest goes straight onto the objectives tracker.", "", 1,
     "", "cvar:autoQuestWatch", "", "AUTO_QUEST_WATCH = v"},
    {"widerquesttracker", "Wider objectives tracker", SettingKind::Bool, 0, 0, 0, "Gameplay", "",
     "Give the tracker more room across, so long objectives fit on one\n"
     "line.", "", 0, "", "cvar:watchFrameWidth", "",
     "WATCH_FRAME_WIDTH = v if WatchFrame_SetWidth then WatchFrame_SetWidth(v) end"},

    {"showcloak", "Show your cloak", SettingKind::Bool, 0, 0, 0, "Gameplay", "Character",
     "Wear your cloak where others can see it. The server keeps this,\n"
     "so it follows the character rather than the client.", "", 1,
     "", "lua:ShowingCloak|ShowCloak"},
    {"showhelm", "Show your helm", SettingKind::Bool, 0, 0, 0, "Gameplay", "",
     "The same for the helm.", "", 1, "", "lua:ShowingHelm|ShowHelm"},
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
