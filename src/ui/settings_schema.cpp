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
     "The most frames drawn each second. Unlimited runs the hardware\n"
     "flat out, which on a laptop means heat and fan noise for frames\n"
     "nobody sees. Vertical sync already caps at the display's rate;\n"
     "use this to cap below it.",
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
    // No extended-zoom switch here. The game's own Camera panel has Max Camera
    // Distance, which is the same setting expressed as a multiple rather than
    // as a choice between two positions - and it wrote a CVar nothing read
    // while this checkbox did the work. kCVarRanges widens that slider past the
    // shipped ceiling of 2, so it reaches everywhere the checkbox used to and
    // every distance in between.
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
    {"invertmouse", "Invert mouse look", SettingKind::Bool, 0, 0, 0, "Camera", "Mouse",
     "Push the mouse forward to look up, as in a flight sim.", "", 0},

    // --------------------------------------------------------------- Interface
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

    // ----------------------------------------------------------------- Minimap
    // Rotate-with-camera is deliberately absent. The settings window still
    // draws that checkbox and its handler pins it back off - the minimap is
    // north-up in this client and the control has not worked for as long as it
    // has existed. A tickbox that unticks itself is worse here than no tickbox
    // at all, so this list does not offer one.
    {"minimapsquare", "Square minimap", SettingKind::Bool, 0, 0, 0, "Minimap", "Appearance",
     "Draw the minimap as a square rather than a circle.", "", 0},
    {"minimapnpcdots", "Nearby creature dots", SettingKind::Bool, 0, 0, 0, "Minimap", "",
     "Mark creatures near you as dots on the minimap.", "", 0},
    {"minimapcoords", "Coordinates", SettingKind::Bool, 0, 0, 0, "Minimap", "",
     "Show your map coordinates under the minimap.", "", 0},

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

    // ------------------------------------------------------------ Combat & HUD
    {"nameplatescale", "Nameplate scale", SettingKind::Float, 0.5f, 2.0f, 0.05f,
     "Combat & HUD", "Nameplates", "Size of the name and health bars over creatures' heads.", "", 1},

    {"dpsmeter", "Damage meter", SettingKind::Bool, 0, 0, 0, "Combat & HUD", "Trackers",
     "Show your damage and healing per second above the action bar\n"
     "while you are in combat.", "", 0},
    {"cooldowntracker", "Cooldown tracker", SettingKind::Bool, 0, 0, 0, "Combat & HUD", "",
     "Show your longer cooldowns counting down beside the action bar.", "", 0},
    {"raretracker", "Rare tracker", SettingKind::Bool, 0, 0, 0, "Combat & HUD", "",
     "Mark rare creatures near you on the minimap and the world map.", "", 0},
    {"chesttracker", "Chest tracker", SettingKind::Bool, 0, 0, 0, "Combat & HUD", "",
     "Mark treasure chests near you on the minimap.", "", 0},

    {"damageflash", "Damage flash", SettingKind::Bool, 0, 0, 0, "Combat & HUD", "Screen effects",
     "Flash a red edge around the screen when you take a hit.", "", 1},
    {"lowhealthvignette", "Low health warning", SettingKind::Bool, 0, 0, 0,
     "Combat & HUD", "", "Pulse a red edge around the screen while your health is\n"
     "below a fifth.", "", 1},

    // ------------------------------------------------------------------- Sound
    {"musicvolume", "Music", SettingKind::Int, 0, 100, 5, "Sound", "Music and ambience",
     "The zone's music. With the WoWee soundtrack on, that too.", "", 30},
    {"woweemusic", "WoWee soundtrack", SettingKind::Bool, 0, 0, 0, "Sound", "",
     "Play this client's own music in the rotation alongside the game's.", "", 1},
    {"ambientvolume", "Ambience", SettingKind::Int, 0, 100, 5, "Sound", "",
     "Wind, water, birds and the rest of the world's own noise.", "", 100},
    {"bellvolume", "City bells", SettingKind::Int, 0, 100, 5, "Sound", "",
     "The hour struck in the capital cities.", "", 50},

    {"uivolume", "Interface", SettingKind::Int, 0, 100, 5, "Sound", "Effects",
     "Clicks, bag sounds and window noises. Each slider here balances\n"
     "one kind of effect against the others; the Sound Effects slider\n"
     "in the game's own Sound panel scales them all together.", "", 100},
    {"combatvolume", "Combat", SettingKind::Int, 0, 100, 5, "Sound", "",
     "Weapon swings, hits, blocks and parries.", "", 100},
    {"spellvolume", "Spells", SettingKind::Int, 0, 100, 5, "Sound", "",
     "Casting, spell impacts and buffs landing.", "", 100},
    {"movementvolume", "Movement", SettingKind::Int, 0, 100, 5, "Sound", "",
     "Jumping, landing, swimming and armour rustle.", "", 100},
    {"footstepvolume", "Footsteps", SettingKind::Int, 0, 100, 5, "Sound", "",
     "Your own and others' footsteps, by the ground underfoot.", "", 100},
    {"mountvolume", "Mounts", SettingKind::Int, 0, 100, 5, "Sound", "",
     "Hoofbeats, wingbeats and mount calls.", "", 70},
    {"activityvolume", "Activity", SettingKind::Int, 0, 100, 5, "Sound", "",
     "Fishing, mining, forges and the rest of the world at work.", "", 100},

    {"npcvoicevolume", "NPC voices", SettingKind::Int, 0, 100, 5, "Sound", "Voices",
     "Greetings, farewells and quest speech from the people you talk to.", "", 100},
    {"characterspeech", "Character speech", SettingKind::Bool, 0, 0, 0, "Sound", "",
     "Your own character's grunts, greetings and emotes.", "", 1},

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

    // ---------------------------------------------------------------- Gameplay
    {"autoloot", "Auto loot", SettingKind::Bool, 0, 0, 0, "Gameplay", "Looting",
     "Take everything from a corpse or chest without opening the loot\n"
     "window.", "", 0},
    {"autosellgrey", "Sell grey items", SettingKind::Bool, 0, 0, 0, "Gameplay", "",
     "Sell every grey (junk) item in your bags whenever you open a\n"
     "merchant.", "", 0},
    {"autorepair", "Repair at vendors", SettingKind::Bool, 0, 0, 0, "Gameplay", "",
     "Repair your gear whenever you open a merchant who can.", "", 0},
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
