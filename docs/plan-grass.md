# GPU-Driven Grass — Phased Implementation Plan

**Status:** Phases 1 through 6 complete. Next: Phase 7.
**Branch:** `grass`
**Spec:** [`docs/grass-spec.md`](grass-spec.md) — every `spec §N` below refers to a numbered
section there. The spec is **not** authoritative; this plan and the repository are. See §2.

The facts in §1 were verified by reading the files cited; cite them rather than re-deriving
them. Keep §4 current as phases land.

---

## 1. Confirmed repository facts

Every statement below was checked against the file named. Cite these instead of re-deriving them.

### Vulkan core

| Fact | Where |
|---|---|
| `MAX_FRAMES_IN_FLIGHT = 2` | `include/rendering/vk_context.hpp:19` |
| Per-frame state array `FrameData frames[MAX_FRAMES_IN_FLIGHT]` | `include/rendering/vk_context.hpp:288` |
| Deferred destruction queue, one per frame slot (`deferredCleanup_`) | `include/rendering/vk_context.hpp:370` |
| Buffers via `VkBuffer::uploadToGPU(ctx, data, size, usage)` (GPU-local + staging) and `VkBuffer::createMapped(allocator, size, usage)` (host-visible) | `include/rendering/vk_buffer.hpp:16-50` |
| `VkBuffer::descriptorInfo(offset, range)` builds `VkDescriptorBufferInfo` | `include/rendering/vk_buffer.hpp` |
| Allocator is VMA (`VmaAllocation`, `AllocatedBuffer`) | `include/rendering/vk_buffer.hpp` |

**`synchronization2` is used throughout, through a wrapper.** Barriers are written in the newer form and lowered where the extension is absent:

```cpp
void cmdPipelineBarrier2(VkCommandBuffer cmd, const VkDependencyInfo& dep);  // vk_utils.hpp:151
```

With `VK_KHR_synchronization2` present this is `vkCmdPipelineBarrier2KHR`; without it the same dependency is lowered to `vkCmdPipelineBarrier`, ORing the per-barrier stage masks into the single pair the legacy call takes. Eleven files under `src/` use it.

**So write grass barriers as `VkDependencyInfo` + `cmdPipelineBarrier2`, following the spec.** The earlier instruction here was the opposite, and its reason — that enabling sync2 for one feature would add a device requirement — no longer holds: the wrapper adds no requirement, because the legacy path is what runs when the extension is missing.

### Shaders

- Sources: `assets/shaders/<name>.<stage>.glsl` → compiled **in place** to `<name>.<stage>.spv`.
- Stages recognised by filename: `.vert.glsl`, `.frag.glsl`, `.comp.glsl`, `.geom.glsl` (`CMakeLists.txt:459-467`).
- `compile_shaders()` does `file(GLOB "${SHADER_DIR}/*.glsl")` (`CMakeLists.txt:449`) — **a new shader is picked up automatically; no CMake edit is needed**, but CMake must be re-run for the glob to refresh.
- `.spv` files are **tracked in git** and are the fallback when `glslc` is absent. Commit both `.glsl` and `.spv`.
- Loading: `VkShader::loadFromFile(device, "assets/shaders/<name>.<stage>.spv")` (`src/rendering/hiz_system.cpp:231`).

### Existing GPU compute precedent — copy these conventions

`assets/shaders/m2_cull.comp.glsl` (76 lines) is the model to follow:

- `layout(local_size_x = 64) in;`
- `layout(std140, set = 0, binding = 0) uniform CullUniforms { vec4 frustumPlanes[6]; vec4 cameraPos; uint instanceCount; ... }` — note `cameraPos.w` is reused as `maxPossibleDistSq`.
- `layout(std430, set = 0, binding = 1) readonly buffer CullInput { ... };`
- `layout(std430, set = 0, binding = 2) writeonly buffer CullOutput { ... };`
- Early-out order: flags → loose distance → per-instance distance → 6-plane sphere test.

Compute pipeline creation: `HiZSystem::createComputePipeline()` (`src/rendering/hiz_system.cpp:173-247`) — `VkComputePipelineCreateInfo`, `ctx_->getPipelineCache()`, push-constant range, per-frame descriptor sets indexed `set_[frameIndex]`.

Dispatch site: `src/rendering/m2_renderer_render.cpp:706-886` — dispatches into the **primary frame command buffer**, `groupCount = (n + 63) / 64`.

There are **five** compute shaders in `assets/shaders/`: `fsr2_accumulate.comp.glsl`,
`fsr2_motion.comp.glsl`, `hiz_build.comp.glsl`, `m2_cull.comp.glsl`, `m2_cull_hiz.comp.glsl`.

**C++/GLSL binary-compatibility precedent:** `include/rendering/m2_renderer.hpp:689` —
`struct CullUniformsGPU { // matches CullUniforms in m2_cull_hiz.comp.glsl (std140)`. Copy that
pattern (and its comment convention) for the blade struct in Phase 1.

**Important delta:** M2 culling writes a `uint visibility[]` mask and **reads it back on the CPU** (barrier `VK_ACCESS_SHADER_WRITE_BIT → VK_ACCESS_HOST_READ_BIT`, `VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT → VK_PIPELINE_STAGE_HOST_BIT`, `m2_renderer_render.cpp:877-885`). It has **no atomic compaction and no compute-generated indirect draw.** Grass will be the first genuinely GPU-driven path in WoWee. That is a real extension, not duplication — build it, but model the *style* on `m2_cull.comp.glsl`.

### Coordinate system — **+Z is UP**

`include/core/coordinates.hpp:16` — *"+X = North, +Y = West, +Z = Up (height)"*, above the
canonical/server/ADT/render conversions themselves. Confirmed by `include/math/spline.hpp:39`
("Z-up convention").

**The spec is written Y-up and is wrong here.** §10 computes slope as
`1.0 - dot(terrainNormal, vec3(0,1,0))` and §31 blends toward `vec3(0,1,0)`. Both must be
`vec3(0,0,1)`. Using the spec's constants compiles fine and produces silently garbage slope and
lighting. In `m2.vert.glsl` the existing foliage code uses `pos.z` for height and `worldRef.xy`
for the horizontal plane — follow that.

### Wind and player interaction are ALREADY IMPLEMENTED — port, do not invent

`assets/shaders/m2.vert.glsl` has a working, tuned foliage system. Read it before writing any
wind or interaction code.

- Push constant `int isFoliage` — `-1` sky, `0` none, `1` foliage, `2` ground clutter (line 21).
- **Height weighting** (lines 91-95): `heightFactor = clamp(pos.z / swayRefHeight, 0, 1)`, then
  **squared** — *"quadratic so the roots stay planted and the motion collects at the tip."* That is
  spec §27's `windWeight`, already solved. `swayRefHeight` is 20 for tree-sized models; ground
  clutter passes its own height, because normalising a one-yard tuft against twenty moves it by
  nothing.
- **Three-layer wind** (lines 104-126), gated to `isFoliage == 1`: trunk sway
  (`windTime*0.8`, amp 0.35/0.25), branch sway (`windTime*1.7`, per-branch phase), leaf flutter
  (`windTime*4.5`, per-vertex). `windTime = fogParams.z`. Phase comes from
  `dot(worldRef.xy, vec2(...))` — **world-space stable, never frame-dependent**, which is exactly
  what spec §36 requires.
- Ground clutter (`isFoliage == 2`) is deliberately **excluded** from shader wind: each detail
  doodad plays its own one-bone animation, and a shader wind on top would be two swings of the
  same plant at two rates. Grass blades are procedural and have no authored animation, so they
  **do** want shader wind — but keep the same phase constants so grass and existing foliage move
  as one field rather than two systems.
- **Player brush + springback** (lines 130-185): bends away from `playerPos` and `playerWake`,
  takes whichever influence is stronger, smooth `reach` falloff, only reacts to foliage at the
  player's own level. Applied in **world space after the model transform**, because the
  displacement is a distance in yards, not something model scale should rescale. Reuse this math.

Spec §26/§27/§28 are therefore mostly a porting job, not a design job.

### Per-frame UBO — already has everything grass needs

`struct GPUPerFrameData` (`include/rendering/vk_frame_data.hpp:14-34`), bound as **set 0**:

```
mat4 view, projection, lightSpaceMatrix
vec4 lightDir       // xyz = direction
vec4 lightColor     // xyz = color
vec4 ambientColor   // xyz = color
vec4 viewPos        // xyz = camera position
vec4 fogColor
vec4 fogParams      // x = fogStart, y = fogEnd, z = TIME, w = water ripple strength
vec4 shadowParams
vec4 playerPos      // xyz = player world position, w = horizontal speed (yd/s)
vec4 playerWake     // xyz = trailing player position
vec4 localLightPosRadius[MAX_LOCAL_LIGHTS]
vec4 localLightColorIntensity[MAX_LOCAL_LIGHTS]
ivec4 localLightMeta // x = active light count
```

Consequences — **do not add a new UBO for any of these**:
- Wind time → `fogParams.z`.
- Distance cull / LOD → `viewPos.xyz`.
- Frustum planes → derive from `projection * view` (or extend the grass cull UBO the way `m2_cull` does).
- Lighting and subsurface → `lightDir`, `lightColor`, `ambientColor`.
- **Player interaction → `playerPos` and `playerWake`, which already exist for exactly this purpose.** The header comment reads: *"the foliage the player brushes past. playerWake trails the player by a fixed time constant, so clutter the player has already walked through springs back over that interval instead of snapping upright."* Spec §28 says do not invent a second entity-position system; this is the system.

### Terrain — the authoritative vegetation source

WoW's ADT format already encodes where grass grows. **Do not build a parallel classifier.**

`struct TextureLayer` (`include/pipeline/adt_loader.hpp:34-43`):
```cpp
uint32_t textureId;   // index into MTEX
uint32_t flags;
uint32_t offsetMCAL;  // offset to alpha map in MCAL
uint32_t effectId;    // ← ground-effect id for THIS layer
bool useAlpha() const;        // flags & 0x100
bool compressedAlpha() const; // flags & 0x200
```

`struct MapChunk` (`include/pipeline/adt_loader.hpp:47+`):
```cpp
uint32_t flags, indexX, indexY, areaId;
uint16_t holes;                      // 4x4 hole bitmask
float position[3];
HeightMap heightMap;                 // getHeight(x, y)
std::vector<TextureLayer> layers;    // up to 4
std::vector<uint8_t> alphaMap;       // MCAL blend weights
std::array<int8_t, 145*3> normals;   // compressed, 145 verts
```

`TerrainManager` (`include/rendering/terrain_manager.hpp:484-492`):
```cpp
struct GroundEffectEntry {
    std::array<uint32_t,4> doodadIds{};
    std::array<uint32_t,4> weights{};
    uint32_t density = 0;
};
bool groundEffectsLoaded_;
std::unordered_map<uint32_t, GroundEffectEntry> groundEffectById_;   // effectId -> config
std::unordered_map<uint32_t, std::string> groundDoodadModelById_;    // doodadId -> model path
float groundClutterDensityScale_ = 1.0f;
```

**This is the whole answer to spec §3–§7.** The chain is native, not invented:

```
texel → per-layer alpha weight (MCAL)
      → layer.effectId
      → groundEffectById_[effectId].density   (0 ⇒ NO GRASS: rock, road, bare dirt)
      → .doodadIds/.weights                   (which vegetation profile)
```

A layer with `effectId == 0` or an id absent from `groundEffectById_` is Blizzard explicitly saying *no vegetation here*. Roads, cliffs, and dirt already carry no ground effect. Slope comes from `MapChunk::normals`, root height from `HeightMap::getHeight`, and `holes` must suppress grass over cave openings.

`TerrainChunkGPU` (`include/rendering/terrain_renderer.hpp:34-77`) already has the splat maps on the GPU:
```cpp
VkTexture* baseTexture;
VkTexture* layerTextures[3];
VkTexture* alphaTextures[3];   // ← splat weights, already resident
int layerCount;
VkDescriptorSet materialSet;   // set 1: 7 samplers + params UBO
glm::vec3 boundingSphereCenter; float boundingSphereRadius;
int tileX, tileY;              // owning tile, for per-tile removal
int32_t megaBaseVertex; uint32_t megaFirstIndex, vertexCount;
```

Descriptor convention: **set 0 = per-frame UBO, set 1 = material.** Follow it.

**An editor-side vegetation system also exists** — `tools/editor/terrain_biomes.hpp:113`
(`struct VegetationAsset`) and `:123` (`struct BiomeVegetation`), placed by
`tools/editor/object_placer.hpp:86 populateBiome(const BiomeVegetation&, ...)`. Read these before
inventing a profile type (spec §6: do not duplicate an existing system). It is editor tooling, not
the runtime path, so it may not be the right home for runtime profiles — but its field set is a
strong prior for what a vegetation profile needs, and reusing its vocabulary keeps the two
consistent.

Streaming: `TerrainManager::unloadTile(int x, int y)` (`include/rendering/terrain_manager.hpp:224`) is the unload hook grass must participate in (spec §12). Tiles are finalized incrementally across frames (`finalizingTiles_`).

### Build & test

- `./test.sh` — unit tests + clang-tidy. `--asan`, `--lint`, `FIX=1 ./test.sh --lint`.
- `WOWEE_WARNINGS_AS_ERRORS` is **ON** by default. A warning fails the build.
- 160 tests via CTest, from 156 Catch2 v3 suites in `tests/`.
- Debug HUD: `PerformanceHUD` (`include/rendering/performance_hud.hpp`), F1 to toggle, `setShowTerrain(bool)` etc. — add grass counters here, do not build a new overlay.

---

## 2. Spec deviations (decided, do not relitigate)

| Spec asks | We do | Why |
|---|---|---|
| §22 `vkCmdPipelineBarrier2` | **follow the spec**, via `cmdPipelineBarrier2` | the wrapper lowers to the legacy call where the extension is absent, so the newer form costs no device requirement |
| §13 32-byte blade struct | measure and document the real packed layout | spec itself says do not force 32 bytes |
| §6 new `GrassType` enum | derive profiles from `GroundEffectEntry.doodadIds` | spec §6: "do not duplicate an existing WoWee enum/system" |
| §28 entity buffer | reuse `playerPos`/`playerWake` in `GPUPerFrameData` | already exists, purpose-built for foliage |
| §21 hierarchical compaction | plain `atomicAdd` first | spec §21: "do not implement the more complicated system prematurely" |
| §10/§31 up axis `vec3(0,1,0)` | `vec3(0,0,1)` | WoWee is **Z-up**; the spec is written Y-up. Silent garbage otherwise |
| §26/§27 wind | **superseded**: travelling waves, not the M2 port | the port shipped first, then was reworked at the owner's direction into wave fronts that sweep the field - the M2 model moves plants in place, and in-place motion across a whole field reads as a texture. Phase is still world-position seeded (spec §36) |
| §28 new interaction code | reuse the bend/springback in `m2.vert.glsl:130-185` | already handles wake, reach falloff and player level |

---

## 3. Phases

One phase per session. Each ends green: `./test.sh` passes, zero validation-layer errors.
Do **not** start the next phase in the same session — update §4 and stop.

### Phase 1 — GPU walking skeleton
**Goal:** the full GPU-driven path working end to end, on deliberately fake data.

Fixed test population (a flat grid of ~100k blades at a hardcoded world origin, no terrain input).
This isolates the Vulkan plumbing — the part where a mistake produces a black screen with no clue —
from the terrain logic, which is where correctness actually lives.

- Packed blade struct in C++ **and** GLSL, binary compatible. `static_assert` on `sizeof`/`alignof`; a test asserting both plus every member offset. Document the layout in this file.
- Source SSBO (`VK_BUFFER_USAGE_STORAGE_BUFFER_BIT`), shared/immutable.
- Per-frame ×2: visible-index SSBO, indirect buffer (`VkDrawIndexedIndirectCommand`, `INDIRECT | STORAGE`), counter.
- `assets/shaders/grass_cull.comp.glsl` — modeled on `m2_cull.comp.glsl`. Frustum + distance cull, `atomicAdd` compaction, writes `instanceCount`. Reset the counter each frame (`vkCmdFillBuffer` before dispatch).
- Barrier: `COMPUTE_SHADER → DRAW_INDIRECT | VERTEX_SHADER`, `SHADER_WRITE → INDIRECT_COMMAND_READ | SHADER_READ`, written as a `VkDependencyInfo` through `cmdPipelineBarrier2`. **No host readback.**
- Minimal `grass.vert.glsl` / `grass.frag.glsl`: flat quads, solid colour. No Bézier, no wind.
- `vkCmdDrawIndexedIndirect`.

**Exit:** flat green blades render at the test origin. Validation layers clean. Counter never read on CPU. Correct under 2 frames in flight (verify frame N doesn't stomp N-1: per-frame output buffers).

**Risk:** highest of any phase. If it stalls, the fault is almost always the barrier or a descriptor mismatch — check those before suspecting the shader.

### Phase 2 — Terrain suitability (CPU only, no Vulkan)
**Goal:** answer "should grass grow at this world position, and what kind" from real terrain data.

New `GrassTerrainAdapter`. Given a `MapChunk` + world XZ, return `{suitability 0..1, effectId, slope, rootHeight}`:
- Sample per-layer alpha weights from `MapChunk::alphaMap` (respect `useAlpha()` / `compressedAlpha()`).
- `layer.effectId → groundEffectById_` → density. Absent or 0 ⇒ suitability 0.
- Weight each layer's contribution by its alpha ⇒ **continuous** suitability, not binary (spec §4).
- Slope from `MapChunk::normals`; root Y from `HeightMap::getHeight`.
- `holes` bitmask ⇒ suitability 0.

**Exit:** new Catch2 suite, headless, no GPU. Cases: pure grass layer, grass→dirt blend, road, rock, hole, steep slope, multi-layer blend. Assert continuity across a blend boundary (no hard step).

This phase is where spec §44 ("if the terrain says no grass, put no grass") is actually won.

### Phase 3 — Real population generator
Replace Phase 1's fake grid. Deterministic world-space hash (position + tile coords + stable seed — **never** frame-dependent, spec §36). Density from Phase 2 suitability × `groundClutterDensityScale_`. Generate per terrain tile; allocate the source SSBO on tile load and release it in `unloadTile()` via the deferred-destruction queue.

**Exit:** grass follows real terrain. Walk a grass→dirt boundary and see it thin out. Load/unload tiles repeatedly — no leaks (`--asan`), no stale buffers. Blade identity stable across frames.

### Phase 4 — Vegetation profiles
Map `GroundEffectEntry.doodadIds`/`weights` → profiles controlling height range, width, root/tip colour, colour variation, curve strength, wind influence/stiffness, slope tolerance. Table-driven and data-derived; do not hardcode per-zone rules.

**Exit:** lush / dry / rocky terrain visibly differ in density, height, and colour.

### Phase 5 — Blade geometry, wind, interaction
Mostly a **porting** phase. Open `assets/shaders/m2.vert.glsl` first and work from it.

- 12-vertex strip, 5 segments, quadratic Bézier (spec §25), local basis built without per-vertex matrices. Remember **Z is up**.
- Wind: reuse the phase constants and `dot(worldRef.xy, ...)` world-space seeding from `m2.vert.glsl:104-126`. Grass needs two layers, not three — map trunk→gust and leaf→rustle, and **multiply** them (spec §27) rather than summing as the M2 path does for its three. Time from `fogParams.z`. Keep the quadratic height weighting from `:91-95`.
- Interaction: port the bend/springback from `m2.vert.glsl:130-185` — `playerPos` vs `playerWake`, stronger influence wins, `reach` falloff, player-level gate, applied in world space after the model transform.

**Exit:** blades curve and move coherently; walking through them bends and springs back; grass and nearby M2 foliage visibly move as one field, not two systems at different rates.

### Phase 6 — Shading
Root→tip gradient, terrain colour influence (mix, don't replace — spec §8), upright-normal blend (§31), `gl_FrontFacing` two-sided (§32), cheap wrapped-diffuse subsurface (§33) from `lightDir`/`lightColor`/`ambientColor`. Keep the fragment shader cheap; this is a high-overdraw workload.

**Exit:** grass reads as part of the terrain, no harsh dark edges, no neon-on-brown.

### Phase 7 — LOD, debug, validation sweep
Distance density falloff with no visible popping. `PerformanceHUD` counters: source count, visible count, cull %, dispatch size. Toggles: disable culling / terrain mask / wind / interaction, show mask, show density, show profile. Then run the full spec §43 validation list and record results in §4.

**Exit:** spec §43 swept, findings recorded, remaining limitations stated honestly.

---

## 4. Status

Phase 0 (reconnaissance) and Phase 0a (spec recovery) are done: the facts in §1 were
verified against the files cited, the deviations in §2 are decided, and all 50 spec
sections are in `docs/grass-spec.md` with navigable `## N.` headings.

### Phase 1 — done

The GPU-driven path runs end to end on the fixed test population: 99856 blades
(a 316x316 field, about 79 yards across, jittered off the lattice), planted
under the player the first frame a character position exists.

The field origin is latched, not followed: a field that tracked the player
would slide along under them and no blade would hold a fixed place in the
world, which is the property spec §36 asks for and which Phase 3's generator
must also have. Blades are generated around a local origin and placed by that
offset - added in the cull shader from its uniform block and in the vertex
shader from a push constant - so moving the field costs a uniform write rather
than regenerating and re-uploading the source buffer. Phase 3 generates real
world positions per tile and all of this goes.

| Piece | Where |
|---|---|
| Blade struct, C++ side | `include/rendering/grass_blade.hpp` |
| Cull + compaction | `assets/shaders/grass_cull.comp.glsl` |
| Draw | `assets/shaders/grass.vert.glsl`, `grass.frag.glsl` |
| Renderer | `include/rendering/grass_renderer.hpp`, `src/rendering/grass_renderer.cpp` |
| Layout test | `tests/test_grass_blade_layout.cpp` (ctest `grass_blade_layout`) |

**Blade layout** — 32 bytes, std430, asserted at compile time and in the test:

| offset | field | meaning |
|---|---|---|
| 0 | `positionHeight.xyz` | root world position (render space) |
| 12 | `positionHeight.w` | height, yards |
| 16 | `facingWidthPhase.x` | facing, radians about +Z |
| 20 | `facingWidthPhase.y` | width, yards |
| 24 | `facingWidthPhase.z` | tilt — carried, unused until Phase 5 |
| 28 | `facingWidthPhase.w` | wind phase seed — carried, unused until Phase 5 |

The two unused fields are carried now so the stride does not change when Phase 3
puts real data behind it. `GrassCullUniformsGPU` is 128 bytes, std140, laid out
like `CullUniformsGPU`.

**The counter is the draw command.** `instanceCount` in the
`VkDrawIndexedIndirectCommand` is both the cursor `atomicAdd` advances and the
field `vkCmdDrawIndexedIndirect` consumes, so compaction needs no second buffer
and no readback. The host zeroes that one field with `vkCmdFillBuffer` before
each dispatch and never reads it.

Two barriers, both `VkDependencyInfo` through `cmdPipelineBarrier2` per §2: the
fill before the dispatch that adds to it, and compute-write before
`DRAW_INDIRECT | VERTEX_SHADER`. Output buffers are per frame in flight, so
frame N cannot overwrite what N-1 is still drawing.

Decisions worth not relitigating:

- The source blades, the visible-index list and the indirect command are all
  device-local. Only the cull uniform block is host-visible.
- No vertex buffer. The quad comes from `gl_VertexIndex`; a six-index buffer
  shared by every blade satisfies `vkCmdDrawIndexedIndirect`'s requirement for
  a bound index buffer.
- `VK_CULL_MODE_NONE`: a blade is one quad and is seen from both faces.
- Grass draws after terrain and before WMO, so anything standing on the ground
  occludes it.
- Initialization is non-fatal, like the other effect renderers; `isReady()`
  gates both call sites.

**Not verified.** `./test.sh` is green at 161/161 and the lint gate reports
nothing in any grass file, but no run on hardware has happened yet, so "flat
green blades render" and "validation layers clean" are claims about the code
rather than observations.

### Phase 5 — brought forward, done

Out of order, because the Phase 1 blades were legible enough to be judged and
were wrong: too tall, too thick, cut square at the tip, and motionless. Shape
is not something to leave until after two phases of terrain work when it is
this visible.

- **Geometry.** Five segments, six rows of two vertices, bent along a quadratic
  Bezier. Width is near constant over the lower half and tapers to nothing at
  the top row, whose quad collapses to a triangle - that is the point. A
  `static_assert` ties the index list to the row count, because a mismatch
  reads rows the vertex shader never builds and cannot be detected at runtime.
- **Wind.** Shipped first as the M2 port (two layers multiplied, spec §27),
  later reworked into travelling waves at the owner's direction - a ~13 yard
  front sweeping along the wind, a larger gust band rolling over it, and a
  perpendicular wobble as fronts pass. Still world-position seeded, time from
  `fogParams.z`. Seed heads and blooms (profile-gated, drifting in world-space
  patches) landed with the same rework.
- **Player.** Ported from `m2.vert.glsl:139-199` - `playerPos` against
  `playerWake`, stronger influence wins, `reach` falloff, level gate. The size
  gate is dropped (every blade here is grass) and the bend is larger: trodden
  grass lies most of the way over where clutter only leans.
- **Sizes.** 0.28 yards nominal, 0.6-1.4 variation, 0.024 wide. The first
  version was 0.6 and 0.08 and stood chest-high on a character.

The Bezier does the height weighting the M2 path applies separately: the root
is a fixed control point, so motion collects at the tip on its own.

### Phase 4 — done

`pipeline::deriveProfile` in `include/pipeline/grass_profile.hpp`. Six cases,
ctest `grass_profile`.

**The doodad names are the classification.** The shipped detail set is named
`<zone><type><n>`, and the type is one of about a dozen three-letter codes:
`gra` 156 of them, `bus` 119, `roc` 71, `flo` 67, then bones, branches,
thorns, coral, mushrooms. An effect that plants mostly `gra` is meadow; one
that plants mostly `roc` is scree with a little growth between the stones. A
profile is the weighted blend of the five categories those fall into, so
nothing in it knows what a zone is - and a profile needing a per-zone
correction would be the wrong shape entirely.

The scales that change geometry and count (height, width, density) are applied
when the population is generated and never reach the device. Colour, colour
variation and stiffness go up as a small table the blade indexes, in the field
carried unused since Phase 1 for exactly this - the blade is still 32 bytes.

Stiffness divides both the wind bend and the player bend rather than being
subtracted from one, so stiff growth resists everything that moves it.

Two details that would otherwise be silent: an effect whose weights are all
zero means its four doodads are equally likely, not that none of them apply;
and an effect whose doodad ids resolve to no model still grows ordinary grass
rather than becoming bare ground.

### Phase 6 — done

The two-sided lighting, wrapped diffuse and root-to-tip gradient landed with
Phase 5. The rest:

- **Terrain colour influence (spec §8).** Each blade carries the mean colour
  of the ground under its root - the chunk's layer texture colours blended by
  the same alpha weights the terrain shader composites with, so the grass
  tints toward exactly the ground drawn beneath it. Mixed, not replaced:
  45% at the root falling to 20% at the tip. Mean colours are computed once
  per texture from the decoded base level and cached
  (`TerrainManager::getTerrainTextureMeanColor`); a failure yields neutral
  grey, which is a non-tint rather than a hole. The blade grew to 48 bytes
  for it - struct, both shaders and the layout test updated in one commit,
  which is the discipline the test exists to enforce.
- **Upright-normal blend (spec §31).** The per-blade normal is blended
  halfway toward +Z before shading. A field shades like a field; unblended
  per-blade normals make every blade a separate glint.

The fragment shader is still one gradient mix, one ground mix, one wrapped
diffuse - no texture fetches.

### Phase 2 — done

`pipeline::evaluateGrass(chunk, u, v, densityFor)` in
`include/pipeline/grass_terrain.hpp` answers what the terrain says about one
point: `{suitability, effectId, slope, rootHeight}`.

The density lookup arrives as a callback rather than a `TerrainManager`
reference. The table lives in the rendering layer, and this has no business
depending on it or on a device - which is what lets the whole suite run
headless with chunks built by hand.

- **Layers.** Layer 0 is the base; 1..3 take their weight off it in order, the
  way the terrain shader composites them. Each layer contributes its own weight
  if its ground effect has density, so a texel half grass and half road is half
  suitable rather than one or the other (spec §4).
- **Continuity.** Alpha is sampled bilinearly. Point-sampling would step in
  64ths of a chunk - about half a yard - and grass would end along a straight
  line wherever two textures meet. A test walks a blend boundary and asserts no
  step over 0.05.
- **Holes** are cave mouths and doorways cut through the terrain, so nothing
  grows there whatever the layers say.
- **Slope** comes from the chunk's own normals, tapering between 0.30 and 0.55
  (about 25 to 56 degrees) rather than stopping along a contour.

Ten cases, headless, ctest `grass_terrain`: pure grass, road, an id absent from
the table, no effect at all, a blend boundary, dominant-layer selection, holes,
three slopes, interpolated root height, and an empty chunk.

**MCAL decoding moved to `pipeline/adt_alpha.hpp`.** Two copies had already
grown - `terrain_mesh.cpp` and `terrain_manager.cpp` - and this would have been
a third. `terrain_manager` now uses the shared one; `terrain_mesh` still has
its own, because its version differs in what it leaves unreached texels as and
switching it is a separate change with its own risk. The fill value is a
parameter for that reason. The shared version also bounds the packed-alpha read
against the blob, which neither copy did.

### Phase 3 — generator done, GPU side outstanding

`pipeline::populateArea` in `include/pipeline/grass_population.hpp` turns
suitability into blades. Nine cases, ctest `grass_population`.

Everything about a blade - jitter, whether it survives thinning, height,
facing, phase - comes from an integer hash of the world lattice cell it sits
in. No floating point in a blade's identity, and no dependence on the window it
was generated in. **The lattice is anchored to the world, not to the centre**,
so moving the centre slides a window over a fixed population rather than
producing a different one. A test walks the window four yards east and requires
every overlapping blade to come back identical; without that, each rebuild
would visibly reshuffle the field.

Thinning is by suitability rather than a threshold, which is what carries
Phase 2's continuous boundary through into where blades actually appear.

**Deviation: sampling takes a per-chunk context.** `evaluateGrass` decoded the
chunk's alpha maps on every call - four kilobytes per layer per sample, which
at fifty thousand candidates is hundreds of megabytes of the same work.
`ChunkGrassContext::build` does it once per chunk. The old signature remains
for single samples and tests.

**Deviation from per-tile allocation.** The plan called for a source SSBO per
tile, allocated on tile load. A tile is 533 yards square; at a density that
looks like grass that is over a million blades and tens of megabytes for one
tile, and most of it is never in view. Generation is therefore windowed on the
player, at a radius, rebuilt when they leave it - which the world-anchored
lattice makes free of visual consequence.

**On the GPU.** `GrassRenderer::setPopulation` stages blades into a source
buffer allocated once at its full capacity (400k blades, 12 MB). The
population is replaced whenever the player leaves the window, and reallocating
a device-local buffer on that cadence would mean stalling the queue or
deferring a destroy every time. `bladeCount_` is raised only after the copy,
since the cull dispatches over it.

`Renderer::updateGrassPopulation` rebuilds when the player has moved 20 yards
from the window centre; the window is 55 yards, comfortably past the cull
distance so grass never ends at a visible circle. The sampler memoises one
decoded chunk, which is nearly always a hit because the generator walks cells
in world order.

The field-origin push constant is gone - generated blades carry world
positions - and with it the last of the Phase 1 test population.

**Still open:** the rebuild runs on the main thread. It logs how long it took;
if that shows as a stall it needs to move to a worker, which needs
`findChunkAt` to be safe against terrain streaming.
