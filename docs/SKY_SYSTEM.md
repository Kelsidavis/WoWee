# Sky System & Azeroth Astronomy

## Overview

The sky rendering system in wowee follows World of Warcraft's WotLK (3.3.5a) architecture, where skyboxes are **authoritative** and procedural elements serve as fallbacks only. This document explains the lore-accurate celestial system, implementation details, and critical anti-patterns to avoid.

---

## Architecture

### Component Hierarchy

```
SkySystem (coordinator)
├── Skybox (fullscreen gradient from the DBC sky colours - underlay)
├── StarField (procedural points: debug, "Sharp stars", or fallback when no sky M2)
├── Celestial (sun + White Lady + Blue Child)
├── Clouds (atmospheric layer)
└── LensFlare (sun glow effect)

Renderer
└── skyboxModelRenderer_ (M2Renderer in sky mode - the original client's sky M2,
                          AUTHORITATIVE - includes baked stars, clouds, planets)
```

### Rendering Pipeline

```
LightingManager (DBC-driven)
  ↓ Light.dbc + LightParams.dbc + LightIntBand/LightFloatBand time-of-day bands
  ↓ produces: directionalDir, diffuseColor, skyColors, cloudDensity, fogDensity
  ↓ also resolves the active LightSkybox model path
  ↓
skyParamsFromLighting() → SkyParams (interface struct)
  ↓ adds: gameTime, weatherIntensity, useOriginalSkybox, sunOcclusion
  ↓
SkySystem::render(cmd, perFrameSet, camera, params)
  ├─→ Skybox (gradient, far plane)
  ├─→ if a sky M2 is active and no procedural stars: stop here
  ├─→ StarField (debug, "Sharp stars", or no sky M2)
  ├─→ Celestial (sun + 2 moons, uses directionalDir + gameTime)
  ├─→ Clouds (atmospheric layer, density raised by weather)
  └─→ LensFlare (screen-space sun glow, attenuated by sunOcclusion)
  ↓
Renderer draws the sky M2 (skyboxModelRenderer_) over the gradient
```

The sky is recorded after the terrain and before WMOs, doodads and characters.
Every sky layer sits on the far plane and depth-tests without writing, so pixels
a hill covers are skipped. Only the terrain goes first because it is the one
fully opaque pass; blended windows and leaves leave no depth to test against.

LightIntBand channels read by `LightingManager` (`LightParamsProfile::ColorChannel`):
0 is the sunlight (`diffuseColor`), 1 the ambient, 2-5 the sky gradient
(top, middle, band 1, band 2), 6 the sky's smog layer and 7 the fog.

---

## Celestial Bodies (Lore)

### The Two Moons of Azeroth

Azeroth has **two moons** visible in the night sky, both significant to the world's lore:

#### **White Lady** (Primary Moon)
- **Appearance**: Larger, brighter, pale white color (RGB: 0.8, 0.85, 1.0)
- **Size**: 40-unit diameter billboard
- **Intensity**: Full brightness (1.0), scaled by how dark the DBC sky is
- **Lore**: Tied to Elune, the Night Elf moon goddess
- **Cycle**: 30 game days per phase cycle (~12 real-world hours)

#### **Blue Child** (Secondary Moon)
- **Appearance**: Smaller, dimmer, pale blue color (RGB: 0.7, 0.8, 1.0)
- **Size**: 30-unit diameter billboard
- **Intensity**: 70% of White Lady's brightness
- **Position**: Offset to the right and slightly lower (+80 X, -40 Z)
- **Cycle**: 27 game days per phase cycle (~10.8 real-world hours, slightly faster)

#### **Visibility**
- **Night hours**: 19:00 - 5:00 (both moons visible)
- **Fade transitions**:
  - Fade in: 19:00 - 21:00 (dusk)
  - Full intensity: 21:00 - 3:00 (night)
  - Fade out: 3:00 - 5:00 (dawn)
- **Day hours**: 5:00 - 19:00 (moons not rendered)
- **Sky darkness gate**: `SkySystem::render` passes a night factor of
  `1 - smoothstep(0.08, 0.25, luminance(skyTopColor))`; both moons are skipped
  below 0.01. The DBC sky can stay daylight-bright well past 19:00, and a
  full-brightness moon on a blue sky reads as a second sun

### The Sun

- **Positioning**: Driven by `LightingManager::directionalDir`
  - Placement: `sunPosition = -directionalDir * 800` (light comes FROM sun)
  - `Celestial::renderSun` draws the disc along `directionalDir` instead when
    `-directionalDir` points below the horizon
  - The lens flare uses `sunDirectionFromLightDir()` (`sun_direction.hpp`),
    which keeps a sun below the horizon below it
  - Without a lighting manager, `SkyParams`' default `directionalDir` is used
- **Color**: Uses `LightingManager::diffuseColor` (DBC-driven, changes with time-of-day),
  mixed 52% toward a warm (1.0, 0.88, 0.55)
- **Size**: 95-unit diameter billboard
- **Visibility**: 5:00 - 19:00, fading in over 5:00-6:00 and out over 18:00-19:00

---

## Deterministic Moon Phases

### Server Time-Driven (NOT deltaTime)

Moon phases are computed from **server game time**, ensuring:
- ✅ **Deterministic**: Same game time always produces same moon phases
- ✅ **Persistent**: Phases consistent across sessions and server restarts
- ✅ **Lore-feeling**: Realistic cycles tied to game world time, not arbitrary timers

### Calculation Formula

```cpp
float computePhaseFromGameTime(float gameTime, float cycleDays) {
    constexpr float SECONDS_PER_GAME_DAY = 1440.0f;  // 1 game day = 24 real minutes
    float gameDays = gameTime / SECONDS_PER_GAME_DAY;
    float phase = fmod(gameDays / cycleDays, 1.0f);
    return (phase < 0.0f) ? phase + 1.0f : phase;  // Ensure positive
}

// Applied per moon
whiteLadyPhase = computePhaseFromGameTime(gameTime, 30.0f);  // 30 game days
blueChildPhase = computePhaseFromGameTime(gameTime, 27.0f);  // 27 game days
```

**Units mismatch:** the formula takes `gameTime` as seconds, but the value the
renderer passes is `GameHandler::getGameTime()`, the server's hour of day
(0-24, set from the login time packet). Divided by 1440 and 30, a whole day
moves the phase by under 0.001, so both moons stay at new moon.

### Phase Representation

- **Value range**: 0.0 - 1.0
  - `0.0` = New moon (dark)
  - `0.25` = First quarter (right half lit)
  - `0.5` = Full moon (fully lit)
  - `0.75` = Last quarter (left half lit)
  - `1.0` = New moon (wraps to 0.0)
- **Shader-driven**: Phase uniform passed to fragment shader for crescent/gibbous rendering

### Fallback Mode (Development)

If `gameTime < 0.0` (server time unavailable):
- Uses deltaTime accumulator: `moonPhaseTimer += deltaTime`
- Short cycle durations (4 minutes / 3.5 minutes) for quick testing
- **NOT used in production**: Should always use server time

---

## Sky Dome Rendering

### Camera-Locked Behavior (WoW Standard)

The gradient is a fullscreen triangle with no mesh (`skybox.vert.glsl`); the
fragment shader rebuilds each pixel's view ray from the projection and the
view's rotation only:

```glsl
// skybox.vert.glsl
gl_Position = vec4(TexCoord * 2.0 - 1.0, 1.0, 1.0);  // depth = 1.0 (far plane)

// skybox.frag.glsl
vec3 viewDir = vec3(ndcX / projection[0][0], ndcY / abs(projection[1][1]), -1.0);
vec3 worldDir = normalize(transpose(mat3(view)) * viewDir);  // rotation only
float elev = worldDir.z;  // zenith / mid / horizon bands blend on elevation
```

Stars, sun, moons and clouds put `z = w` in clip space for the same result.
The original sky M2 is re-centred on the camera every frame
(`Renderer::update`).

**Why this works:**
- ✅ **Translation ignored**: Sky centered on camera (doesn't "swim" when moving)
- ✅ **Rotation applied**: Sky follows camera look direction (feels "attached to view")
- ✅ **Far plane depth**: Depth test `LESS_OR_EQUAL` with depth writes off, so it shows only where nothing nearer was drawn
- ✅ **Celestial sphere illusion**: Stars/sky appear infinitely distant

### Time-Based Sky Drift (Optional)

Subtle rotation for atmospheric effect:

```cpp
float skyYawRotation = gameTime * skyRotationRate;
skyDomeMatrix = rotate(skyDomeMatrix, skyYawRotation, vec3(0, 0, 1));  // Yaw only
```

**Per-zone rotation rates:**
- Azeroth continents: `0.00001` rad/sec (very slow, barely noticeable)
- Outland: `0.00005` rad/sec (faster, "weird" alien sky feel)
- Northrend: `0.00002` rad/sec (subtle drift, aurora-like)
- Static zones: `0.0` (no rotation)

**Implementation status:** Not implemented. The original sky M2s animate on their own clock.

---

## Critical Anti-Patterns

### ❌ DO NOT: Latitude-Based Star Rotation

**Why it's wrong:**
- Azeroth is **not modeled as a spherical planet** with latitude/longitude in WoW client
- No coherent coordinate system for Earth-style celestial mechanics
- Stars are part of **authored skybox M2 models**, not dynamically positioned
- Breaks zone identity (Duskwood's gloomy sky shouldn't behave like Barrens)

**What happens if you do it anyway:**
- Stars won't match Blizzard's authored skyboxes when M2 models load
- Lore/continuity breaks (geographically close zones with different star rotation)
- "Swimming" stars during movement
- Undermines the "WoW feel"

**Correct approach:**
```cpp
// ✅ Per-zone artistic constants (NOT geography)
struct SkyProfile {
    float celestialTilt;      // Artistic pitch/roll (Outland = 15°, Azeroth = 0°)
    float skyYawOffset;       // Alignment offset for authored skybox
    float skyRotationRate;    // Time-based drift (0 = static)
};
```

### ❌ DO NOT: Always Render Procedural Stars

**Why it's wrong:**
- Skyboxes contain **baked stars** as part of zone mood/identity
- Procedural stars over skybox stars = double stars, visual clash
- Different zones have dramatically different skies (Outland purple nebulae, Northrend auroras)

**Correct gating logic** (`SkySystem::render`):
```cpp
bool renderProceduralStars = false;
if (debugSkyMode_) {
    renderProceduralStars = true;  // Debug: force for testing fog/cloud attenuation
} else if (proceduralStarsEnabled_) {
    renderProceduralStars = true;  // "Sharp stars": replace the sky model's star layer
} else if (!params.useOriginalSkybox) {
    renderProceduralStars = !params.skyboxHasStars;  // Fallback ONLY if skybox missing
}
```

**skyboxHasStars flag:**
- `skyParamsFromLighting()` sets it to the same value as `useOriginalSkybox`:
  `true` while a sky M2 is loaded, `false` for the gradient alone
- Prevents procedural stars from "leaking" over a real skybox's baked stars

**Sharp stars** (Graphics setting `sharpstars`, `Renderer::setSharpStars`) is the
one sanctioned case of both: it turns procedural stars on and calls
`M2Renderer::setSuppressBakedStars`, which skips batches `M2ModelClassifier`
marks as a sky model's star-point layer. The baked layer is one 256x256 DXT
texture stretched over the dome and goes soft at high resolutions.

### ❌ DO NOT: Universal Dual Moon Setup

**Why it's wrong:**
- Not all maps/continents have same celestial bodies
- Azeroth: White Lady + Blue Child (two moons)
- Outland: Different sky (alien world, broken planet)
- Forcing two moons everywhere breaks lore

**Correct approach:**
```cpp
struct SkyProfile {
    bool dualMoons;  // Azeroth = true, Outland = false
    // ... other per-map settings
};

// In Celestial::render()
if (dualMoonMode_ && mapUsesAzerothSky) {
    renderBlueChild(camera, timeOfDay);
}
```

**Current state:** `Celestial::render` checks `dualMoonMode_` alone. It defaults
to `true` and nothing calls `setDualMoonMode`, so whenever Celestial draws, it
draws both moons, on every map.

---

## Integration Points

### SkyParams Struct (Interface)

```cpp
struct SkyParams {
    // Sun/moon positioning
    glm::vec3 directionalDir;   // From LightingManager (sun direction)
    glm::vec3 sunColor;          // From LightingManager (DBC diffuse color)

    // Sky colors (gradient bands; top also gates moon visibility)
    glm::vec3 skyTopColor;
    glm::vec3 skyMiddleColor;
    glm::vec3 skyBand1Color;
    glm::vec3 skyBand2Color;

    // Atmospheric effects (star/moon occlusion)
    float cloudDensity;          // 0-1, from LightingManager
    float fogDensity;            // 0-1, from LightingManager
    float horizonGlow;           // 0-1, atmospheric scattering
    float weatherIntensity;      // 0-1, rain/snow (thickens clouds, attenuates lens flare)
    float sunOcclusion;          // 0 clear to 1 blocked, line of sight to the sun

    // Time
    float timeOfDay;             // 0-24 hours (for sun/moon visibility)
    float gameTime;              // Server hour of day, -1 = none (for moon phases)

    // Skybox control
    uint32_t skyboxModelId;      // Always 0; the model path comes from LightingManager
    bool skyboxHasStars;         // Does skybox include baked stars?
    bool useOriginalSkybox;      // Original camera-centered client M2 is active
};
```

The renderer fills it with `skyParamsFromLighting()`
(`include/rendering/sky_params_from_lighting.hpp`), shared by the parallel and
inline recording paths, then sets `sunOcclusion`.

### Lens Flare Occlusion

`Renderer::sampleSunOcclusion()` answers 1 (blocked) when the sun is below the
horizon, when the camera is inside a WMO, when a WMO bounding-box raycast toward
the sun hits within 600 yards, or when a geometric march of terrain heights
(steps growing by 1.4x out to 600 yards) finds ground above the ray.
`Renderer::update` eases `sunOcclusion_` toward that answer over a quarter
second so a hill edge does not snap the flare on and off. `LensFlare::render`
also weakens the flare near the horizon and under fog, cloud and weather.

### Star Occlusion by Weather

Clouds and fog affect star visibility:

```cpp
// In StarField::render()
float intensity = getStarIntensity(timeOfDay);  // Time-based (night = 1.0, day = 0.0)
intensity *= (1.0f - glm::clamp(cloudDensity * 0.7f, 0.0f, 1.0f));  // Heavy clouds hide stars
intensity *= (1.0f - glm::clamp(fogDensity * 0.3f, 0.0f, 1.0f));    // Fog dims stars

if (intensity <= 0.01f) {
    return;  // Don't render invisible stars
}
```

**Result:** Cloudy/foggy nights have fewer visible stars (realistic behavior)

---

## M2 Skybox System

### LightSkybox.dbc Integration

**DBC Chain:**
```
Light.dbc (spatial volumes)
  ↓ lightParamsId (per weather condition)
LightParams.dbc (profile mapping)
  ↓ skyboxId
LightSkybox.dbc (model paths)
  ↓ M2 model name
Environments\Stars\*.m2 (actual sky dome models)
```

**Skybox Loading Flow:**
1. `LightingManager` walks the in-range light volumes in weight order, picking
   each one's LightParams row (by weather and underwater state), and takes the
   first LightSkybox model path; the current sky is kept while any volume
   naming it still has weight. Indoors there is no sky model
2. `LightingManager::getActiveSkyboxPath()` returns that path
3. `Renderer::ensureSkyboxModel()` (from `Renderer::update`) loads it, trying
   `.m2` for a `.mdx`/`.mdl` name, plus its skin
4. The old sky stays up until the new model has been read and validated; a path
   that fails is remembered and not retried
5. `skyboxModelRenderer_`, an `M2Renderer` with `setSkyMode(true)`, draws it
   (depth-tested, no depth writes, no collision)
6. `WOWEE_NO_SKY_M2=1` skips the sky model and leaves the procedural sky

### Skybox Transition Blending (not implemented)

Sky models are swapped whole when the active path changes.

**Problem:** Hard swaps between skyboxes at zone boundaries look bad

**Solution:** Blend skyboxes using same volume weighting as lighting:

```cpp
// In SkySystem::render() - Vulkan path
// Bind a sky pipeline whose VkPipelineColorBlendAttachmentState is
// configured for additive blending (srcColor=SRC_ALPHA, dstColor=ONE,
// blendOp=ADD), then render each skybox in weight order:
if (activeVolumes.size() >= 2) {
    // Primary skybox (alpha = volumes[0].weight)
    skybox1->render(camera, volumes[0].weight);
    // Secondary skybox blends additively on top
    skybox2->render(camera, volumes[1].weight);
}
```

**Result:** Smooth crossfade between zone skies, no popping

### SkyProfile Configuration (not implemented)

**Per-map/continent settings:**

```cpp
std::map<uint32_t, SkyProfile> skyProfiles = {
    // Azeroth (Eastern Kingdoms)
    {0, {
        .skyboxModelId = 123,
        .celestialTilt = 0.0f,           // No tilt, standard orientation
        .skyYawOffset = 0.0f,
        .skyRotationRate = 0.00001f,     // Very slow drift
        .dualMoons = true                // White Lady + Blue Child
    }},

    // Kalimdor
    {1, {
        .skyboxModelId = 124,
        .celestialTilt = 0.0f,
        .skyYawOffset = 0.0f,
        .skyRotationRate = 0.00001f,
        .dualMoons = true
    }},

    // Outland (Burning Crusade)
    {530, {
        .skyboxModelId = 456,
        .celestialTilt = 15.0f,          // Tilted, alien feel
        .skyYawOffset = 45.0f,           // Rotated alignment
        .skyRotationRate = 0.00005f,     // Faster, "weird" drift
        .dualMoons = false               // Different celestial setup
    }},

    // Northrend (Wrath of the Lich King)
    {571, {
        .skyboxModelId = 789,
        .celestialTilt = 0.0f,
        .skyYawOffset = 0.0f,
        .skyRotationRate = 0.00002f,     // Subtle aurora-like drift
        .dualMoons = true
    }}
};
```

---

## Implementation Checklist

### ✅ Completed
- [x] SkySystem coordinator class
- [x] Skybox camera-locked rendering (translation ignored)
- [x] Procedural stars gated by `skyboxHasStars` flag
- [x] Two moons (White Lady + Blue Child) with independent phases
- [x] Deterministic moon phases from server `gameTime`
- [x] Sun positioning from lighting `directionalDir`
- [x] Star occlusion by cloud/fog density
- [x] SkyParams interface for lighting integration
- [x] Original client sky M2s via LightParams → LightSkybox, on every map that names one
- [x] Sky model star layer identified and replaced by point stars ("Sharp stars")
- [x] Sky drawn after terrain, depth-tested on the far plane
- [x] Lens flare occlusion (WMO interior, WMO raycast, terrain march)

### 🚧 Future Enhancements
- [ ] Moon phases from a game time in the units the formula expects
- [ ] Skybox transition blending (weighted crossfade)
- [ ] SkyProfile per map/continent
- [ ] Time-based sky rotation (optional drift)
- [ ] Moon position from shared sky arc system (not fixed offsets)
- [ ] Support for zone-specific celestial setups (Outland, etc.)

---

## Code References

**Key Files:**
- `include/rendering/sky_system.hpp` - Coordinator, SkyParams struct
- `src/rendering/sky_system.cpp` - Render pipeline, star gating logic
- `include/rendering/celestial.hpp` - Sun + dual moon system
- `src/rendering/celestial.cpp` - Moon phase calculations, rendering
- `include/rendering/starfield.hpp` - Procedural star fallback
- `src/rendering/starfield.cpp` - Star intensity + occlusion
- `include/rendering/skybox.hpp` - Fullscreen sky gradient
- `src/rendering/skybox.cpp` - Gradient pipeline and push constants
- `assets/shaders/skybox.vert.glsl`, `skybox.frag.glsl` - Fullscreen triangle, view-ray gradient
- `include/rendering/clouds.hpp` / `src/rendering/clouds.cpp` - Cloud layer
- `include/rendering/lens_flare.hpp` / `src/rendering/lens_flare.cpp` - Sun flare
- `include/rendering/sun_direction.hpp` - Sun direction from the light direction

**Integration Points:**
- `include/rendering/sky_params_from_lighting.hpp` - Builds SkyParams from LightingManager
- `src/rendering/renderer.cpp` - Records the sky after terrain, loads and draws the sky M2 (`ensureSkyboxModel`), samples sun occlusion
- `include/rendering/lighting_manager.hpp` - Provides directionalDir, colors, fog/cloud, active skybox path

---

## References

- **WoW 3.3.5a Client**: Environments\Stars\*.m2 (skybox models)
- **DBC Files**: Light.dbc, LightParams.dbc, LightSkybox.dbc, LightIntBand.dbc, LightFloatBand.dbc
- **WoWDev Wiki**: https://wowdev.wiki/Light.dbc (lighting system documentation)
- **Lore Sources**:
  - White Lady / Blue Child: https://wowpedia.fandom.com/wiki/Moon
  - Elune connection: https://wowpedia.fandom.com/wiki/Elune
