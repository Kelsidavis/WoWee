# GPU-Driven Terrain-Aware Grass — Specification

> **Provenance.** This is the original task specification, recovered verbatim from the session
> transcript where it was first given (2026-08-15). It had only ever been pasted into a chat, so
> every `spec §N` reference in `docs/plan-grass.md` pointed at a document that existed nowhere on
> disk. Recovered so those references resolve.
>
> **This spec is not authoritative — the repository is.** It was written against a generic Vulkan
> engine and assumes APIs, a Y-up axis convention, and features WoWee does not have. Where the two
> disagree, the repository wins. See the *Spec deviations* table in `docs/plan-grass.md` for the
> conflicts already found and decided; do not relitigate them.

---

You are working directly on the **WoWee** project, an existing Vulkan-based game client/renderer.

Your task is to implement a **production-quality GPU-driven grass and vegetation rendering system**, inspired by the
density, movement, and visual quality of *Ghost of Tsushima*, while integrating naturally into WoWee's existing
renderer.

The system must be **terrain-aware**. Grass must look like it belongs to the terrain beneath it, and vegetation must never
appear on terrain that is supposed to be barren, dry dirt, rock, roads, buildings, water, or other non-grass surfaces.

The CPU should primarily provide static terrain/vegetation data and environment state.

The GPU should perform visibility culling, compaction, procedural blade generation, animation, and indirect rendering.

---

## 1. CRITICAL INSTRUCTIONS — INSPECT WoweE FIRST

Before writing implementation code:

1. Thoroughly inspect the WoWee repository.

2. Identify the actual existing:

    * Vulkan initialization
    * physical/device feature selection
    * command buffers
    * command submission
    * synchronization
    * frames-in-flight
    * descriptor management
    * pipeline abstractions
    * shader compilation
    * buffer abstractions
    * image/texture abstractions
    * camera system
    * world renderer
    * terrain renderer
    * terrain material system
    * terrain splat maps
    * biome/material IDs
    * entity/player system
    * environment/wind systems
    * existing indirect rendering
    * resource streaming
    * world/tile/chunk management
    * build system

3. Determine exactly where grass belongs in the existing renderer.

4. Do **not** invent WoWee APIs.

5. Do **not** create a parallel Vulkan renderer.

6. Do **not** duplicate existing buffer, descriptor, camera, terrain, entity, or pipeline systems.

7. If WoWee already has an abstraction capable of performing a required task, use it.

8. If a required feature does not exist, implement the smallest clean extension necessary.

9. Follow existing WoWee coding style and architecture.

10. Inspect actual shader conventions before writing GLSL.

The repository is authoritative.

The specification below describes the desired behavior and architecture, but should not override existing WoWee
architecture where doing so would reduce correctness or maintainability.

---

## 2. OBJECTIVE

Implement GPU-driven grass capable of rendering **millions of potential blades** with:

* GPU frustum culling
* GPU distance culling
* GPU density falloff
* GPU compaction
* indirect drawing
* procedural blade generation
* quadratic Bézier blade curves
* terrain-aware grass generation
* terrain-aware grass density
* terrain-aware grass type selection
* terrain-aware grass coloring
* terrain-aware blade characteristics
* wind animation
* large-scale wind gusts
* individual blade rustling
* player/NPC interaction
* stylized two-sided lighting
* soft vegetation shading
* approximate subsurface scattering
* CPU-independent per-frame visibility determination
* minimal CPU work after grass data has been generated

The grass should appear to be **growing naturally from the terrain**, not as a separate overlay.

---

## 3. TERRAIN IS AUTHORITATIVE

This is a fundamental requirement.

The renderer must not assume that all terrain should contain grass.

Before generating grass, determine whether the underlying terrain supports vegetation.

Grass must be suppressed on:

* dry dirt
* bare dirt
* rock
* stone
* cliffs where vegetation is inappropriate
* roads
* paths
* sand
* water
* buildings
* constructed surfaces
* cleared areas
* other explicitly non-vegetated materials

If WoWee already has terrain material IDs, splat maps, biome information, vegetation masks, or material weights, use
those systems.

**Do not create a duplicate terrain classification system if WoWee already contains this information.**

---

## 4. TERRAIN GRASS MASK

Use the existing terrain vegetation/grass mask if available.

Conceptually:

```glsl
  float grassMask = sampleGrassMask(worldPosition);
```

Where:

```text
  0.0 = no grass
  1.0 = maximum grass suitability
```

Use the mask to determine vegetation density.

A suitable location should conceptually follow:

```text
  grassMask
      ↓
  terrain material
      ↓
  vegetation profile
      ↓
  grass type
      ↓
  density
      ↓
  blade properties
```

Do not simply use a binary grass/no-grass decision if the terrain system provides continuous material weights.

Natural transitions should be possible:

```text
  dense grass
      ↓
  medium grass
      ↓
  sparse grass
      ↓
  isolated vegetation
      ↓
  bare dirt
```

---

## 5. TERRAIN MATERIAL / SPLAT MAP INTEGRATION

If WoWee uses terrain splat maps or multiple terrain materials, inspect and use them.

For example:

```text
  grass
  dirt
  rock
  sand
  mud
  stone
```

A conceptual suitability calculation might be:

```glsl
  float grassSuitability =
      grassLayerWeight *
      terrainVegetationMask;
```

The exact implementation must match WoWee's existing terrain representation.

Grass should not spawn merely because a terrain tile exists.

A terrain tile can contain perfectly valid terrain that is intentionally barren.

---

## 6. TERRAIN-AWARE GRASS TYPES

Support multiple vegetation profiles rather than one universal blade.

Conceptually:

```cpp
  enum class GrassType
  {
      None,
      ShortGreen,
      TallGreen,
      DryGrass,
      YellowGrass,
      SparseRockGrass,
      Custom
  };
```

Do not duplicate an existing WoWee enum/system if one exists.

Each vegetation profile should be capable of controlling:

```text
  density
  minimum height
  maximum height
  blade width
  root color
  tip color
  color variation
  curve strength
  wind influence
  wind stiffness
  roughness
  slope tolerance
  terrain suitability
```

The terrain determines which profiles are eligible.

---

## 7. MATCH GRASS STYLE TO TERRAIN

Grass must visually match the terrain beneath it.

For example:

**Lush Terrain**

Use:

* dense vegetation
* taller blades
* richer green colors
* stronger variation
* stronger wind response

**Dry Terrain**

Use:

* lower density
* shorter blades
* yellow/brown coloration
* lower saturation
* more subdued wind

**Rocky Terrain**

Use:

* sparse vegetation
* short blades
* vegetation concentrated in suitable soil pockets
* muted colors

**Bare Dirt**

Use:

**NO GRASS.**

**Roads and Paths**

Use:

**NO GRASS**, unless existing terrain data explicitly permits vegetation encroachment.

Do not make these rules blindly hard-coded if WoWee already contains equivalent terrain/material information.

---

## 8. TERRAIN-AWARE COLOR

Grass should visually belong to the terrain.

Where practical, obtain terrain color/material information and use it as an environmental influence.

Conceptually:

```glsl
  vec3 terrainColor = sampleTerrainColor(worldPosition);

  vec3 grassColor =
      mix(
          speciesBaseColor,
          terrainColor,
          terrainColorInfluence
      );
```

Do not simply replace grass color with terrain color.

The terrain should influence the palette rather than destroy the identity of the vegetation.

Avoid obvious combinations such as:

```text
  bright neon-green grass
          ↓
  brown dead terrain
```

unless the underlying environment intentionally supports it.

---

## 9. TERRAIN TRANSITIONS

Grass density should transition naturally between terrain materials.

Avoid hard boundaries such as:

```text
  GRASS GRASS GRASS
  -----------------
  DIRT DIRT DIRT
```

Prefer:

```text
  dense grass
      ↓
  medium grass
      ↓
  sparse grass
      ↓
  isolated blades
      ↓
  bare terrain
```

Use:

* terrain mask weights
* smooth interpolation
* deterministic world-space noise

where appropriate.

Noise must be stable in world space.

Do not use frame-dependent randomness.

---

## 10. TERRAIN SLOPE

Grass should account for terrain slope.

If WoWee exposes terrain normals, use them.

Conceptually:

```glsl
  float slope =
      1.0 - dot(
          terrainNormal,
          vec3(0.0, 1.0, 0.0)
      );
```

Use slope as a vegetation suitability parameter.

Do not automatically eliminate all grass on slopes.

Some vegetation types may naturally grow on steep terrain.

Each vegetation profile may specify its slope tolerance.

---

## 11. TERRAIN HEIGHT

Grass roots must be placed exactly on the terrain.

Use WoWee's existing terrain height query, heightmap, terrain mesh, or equivalent GPU representation.

Conceptually:

```text
  world XZ
      ↓
  terrain height
      ↓
  grass root Y
```

Do not assume a flat world.

Avoid:

* floating grass
* buried grass
* grass hovering above terrain
* visible gaps between roots and terrain

If terrain is dynamically streamed or modified, grass must remain consistent with the terrain system.

---

## 12. TERRAIN STREAMING

If WoWee streams terrain/world regions, vegetation must participate in that architecture.

Grass should be associated with the appropriate world region/chunk/tile.

When terrain unloads:

```text
  terrain region unload
          ↓
  grass resources released/unloaded
```

When terrain loads:

```text
  terrain region loads
          ↓
  vegetation data generated/loaded
```

Do not retain unnecessary grass data for unloaded terrain.

---

## 13. GRASS DATA STRUCTURE

Create a compact GPU representation for source grass blades.

The conceptual data is:

```cpp
  struct GrassBlade
  {
      glm::vec3 position;
      float orientation;
      float scale;
      float windInfluence;
      glm::vec3 curveControl;
  };
```

However, this conceptual structure does not naturally fit into 32 bytes under standard GPU alignment.

**Do not blindly force it into 32 bytes.**

Determine the most appropriate actual GPU representation using:

* packed 32-bit values
* quantization
* half precision where appropriate
* packed orientation/scale
* separate arrays
* a corrected aligned structure

The final structure must be explicitly documented.

For example, a valid implementation might use:

```cpp
  struct GrassBladeGPU
  {
      glm::vec4 positionOrientation;
      glm::vec4 scaleWindCurveX;
      glm::vec4 curveYZPadding;
  };
```

if that layout is appropriate.

But do not assume this exact representation.

Optimize for actual GPU memory bandwidth and WoWee's architecture.

Document:

* C++ structure
* GLSL structure
* alignment
* sizeof
* member offsets
* descriptor type

C++ and GLSL must be binary compatible.

---

## 14. SOURCE GRASS BUFFER

Create a Vulkan storage buffer containing source grass data.

Required usage:

```cpp
  VK_BUFFER_USAGE_STORAGE_BUFFER_BIT
```

Add transfer usage where necessary for WoWee's upload architecture.

The source buffer may contain millions of potential blades.

Prefer immutable/shared source data where possible.

---

## 15. VISIBLE GRASS BUFFER

Create a separate output storage buffer containing compacted visible blade information.

Prefer storing:

```glsl
  uint sourceBladeIndex;
```

instead of duplicating the complete blade structure if the vertex shader can efficiently fetch the source data.

This can substantially reduce memory bandwidth.

If duplicating the blade data is demonstrably better for WoWee's renderer, use that instead.

---

## 16. INDIRECT DRAW BUFFER

Allocate a Vulkan buffer containing:

```cpp
  VkDrawIndexedIndirectCommand
```

with:

```cpp
  VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT
```

and storage usage if the compute shader directly modifies it:

```cpp
  VK_BUFFER_USAGE_STORAGE_BUFFER_BIT
```

The command should contain:

```text
  indexCount
  instanceCount
  firstIndex
  vertexOffset
  firstInstance
```

The compute shader must update the instance count.

The CPU must not read the instance count back every frame.

---

## 17. COMPUTE SHADER

Create a GPU culling/compaction shader:

```glsl
  layout(local_size_x = 64) in;
```

Each invocation processes one source blade.

The general pipeline is:

```text
  source blade
      ↓
  terrain suitability
      ↓
  frustum culling
      ↓
  distance culling
      ↓
  density/falloff
      ↓
  atomic allocation
      ↓
  visible blade index
```

---

## 18. FRUSTUM CULLING

Use six camera frustum planes:

```glsl
  vec4 frustumPlanes[6];
```

Represent:

```text
  left
  right
  top
  bottom
  near
  far
```

Use a conservative bounding sphere or appropriate bounding volume.

Do not perform expensive per-vertex culling.

Each blade should have a suitable bounding radius/height.

---

## 19. DISTANCE CULLING

Calculate camera distance:

```glsl
  float distanceToCamera =
      distance(bladePosition, cameraPosition);
```

Cull blades beyond the configured maximum grass distance.

Support smooth density falloff.

Conceptually:

```glsl
  float density =
      1.0 -
      smoothstep(
          grassStartFade,
          grassEndFade,
          distanceToCamera
      );
```

Avoid obvious temporal popping.

---

## 20. TERRAIN-AWARE CULLING

If terrain suitability is inexpensive and available on the GPU, use it during GPU generation/culling as appropriate.

However, do not rely exclusively on runtime culling to compensate for generating enormous numbers of permanently
invalid blades.

The preferred architecture is:

```text
  terrain evaluation
      ↓
  generate only plausible vegetation
      ↓
  GPU frustum/distance culling
```

---

## 21. COMPACTION

Surviving blades must be written contiguously.

Use an atomic allocation mechanism.

Conceptually:

```glsl
  uint destination =
      atomicAdd(instanceCount, 1);
```

Then:

```glsl
  visibleBladeIndices[destination] =
      sourceBladeIndex;
```

There must be no gaps.

If profiling demonstrates that global atomic contention becomes a significant bottleneck, consider hierarchical
compaction:

```text
  per-workgroup counts
          ↓
  prefix sum
          ↓
  global allocation
          ↓
  compaction
```

Do not implement the more complicated system prematurely.

Start with the straightforward atomic implementation.

---

## 22. COMPUTE / INDIRECT SYNCHRONIZATION

Correctly synchronize:

```text
  COMPUTE SHADER WRITE
          ↓
  INDIRECT DRAW READ
```

and:

```text
  COMPUTE SHADER WRITE
          ↓
  VERTEX SHADER STORAGE READ
```

If WoWee uses synchronization2, prefer:

```cpp
  vkCmdPipelineBarrier2
```

with the appropriate:

```text
  VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT
```

to:

```text
  VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT
  VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT
```

and:

```text
  VK_ACCESS_2_SHADER_WRITE_BIT
```

to:

```text
  VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT
  VK_ACCESS_2_SHADER_READ_BIT
```

If WoWee uses the older synchronization API, use the equivalent vkCmdPipelineBarrier/buffer barriers.

Do not copy a generic barrier without considering WoWee's actual command ordering and resource usage.

The final synchronization must be Vulkan-correct.

---

## 23. FRAME-IN-FLIGHT SAFETY

WoWee may use multiple frames in flight.

Determine whether grass output buffers can safely be shared.

If not, create per-frame:

```text
  visible blade buffer
  indirect buffer
  counter
```

while keeping immutable source grass shared.

Never allow frame N's compute dispatch to overwrite resources still being consumed by frame N-1.

---

## 24. PROCEDURAL BLADE GEOMETRY

Do not upload individual grass meshes.

Generate blades procedurally in the vertex shader.

Use a fixed generic blade strip.

Approximately five segments are appropriate.

A compact topology such as a 12-vertex representation is acceptable.

Use:

```text
  root
  segment 1
  segment 2
  segment 3
  segment 4
  tip
```

with normalized blade height:

```glsl
  float t;
```

where:

```text
  0.0 = root
  1.0 = tip
```

Use the minimum vertex count that produces a convincing silhouette.

---

## 25. QUADRATIC BÉZIER CURVE

Generate the blade using a quadratic Bézier curve:

```glsl
  vec3 bezierQuadratic(
      vec3 p0,
      vec3 p1,
      vec3 p2,
      float t)
  {
      float u = 1.0 - t;

      return
          u * u * p0 +
          2.0 * u * t * p1 +
          t * t * p2;
  }
```

Where:

```text
  P0 = blade root
  P1 = curve control point
  P2 = blade tip
```

Apply:

* orientation
* scale
* terrain placement
* curvature

in world space.

Construct an efficient local basis around the blade direction.

Avoid unnecessarily expensive per-vertex matrix construction.

---

## 26. WIND SYSTEM

Implement two levels of wind.

**Large-Scale Wind**

Create coherent broad movement across groups of grass.

Conceptually:

```glsl
  float gust =
      sin(
          dot(worldPosition.xz, windDirection)
          * windFrequency
          +
          time * windSpeed
      );
```

The entire field should respond coherently.

---

## 27. INDIVIDUAL RUSTLING

Sample a panning 2D noise texture.

Use world-space coordinates:

```glsl
  vec2 noiseUV =
      worldPosition.xz * noiseScale +
      windDirection * time * noisePanSpeed;
```

The noise should produce individual blade variation.

Combine individual rustling with the large-scale gust:

```text
  individual rustle
          ×
  large-scale gust
```

rather than simply adding two unrelated movements.

Wind influence should increase toward the blade tip:

```glsl
  float windWeight =
      smoothstep(0.0, 1.0, t);
```

The root should remain mostly anchored.

---

## 28. PLAYER/NPC INTERACTION

Use WoWee's existing player/entity system.

Do not invent a second entity-position system.

Provide player/NPC positions to the GPU through the appropriate existing buffer/UBO/SSBO architecture.

Conceptually:

```glsl
  entityPositions[]
  entityCount
```

For each blade, calculate distance to relevant entities.

Within an interaction radius:

```text
  blade bends away from entity
```

Use a smooth falloff.

The root should remain relatively anchored while the upper blade bends.

Conceptually:

```glsl
  vec2 away =
      normalize(
          bladePosition.xz -
          entityPosition.xz
      );
```

Apply a rotational/bending deformation away from the entity.

---

## 29. VERTEX SHADER FLOW

The vertex shader should conceptually perform:

```text
  instance ID
      ↓
  visible blade index
      ↓
  source blade data
      ↓
  terrain/world parameters
      ↓
  Bezier curve
      ↓
  wind deformation
      ↓
  player/NPC interaction
      ↓
  world position
      ↓
  camera transform
      ↓
  clip-space position
```

Pass at least:

```glsl
  float bladeHeight;
  vec3 worldPosition;
```

to the fragment shader.

Pass/reconstruct normals as appropriate.

---

## 30. STYLIZED GRASS SHADING

Interpolate color from root to tip.

Conceptually:

```glsl
  vec3 grassColor =
      mix(
          rootColor,
          tipColor,
          smoothstep(0.0, 1.0, bladeHeight)
      );
```

Combine this with terrain/environment influence.

Avoid hard-coded colors where WoWee's material/environment system can provide appropriate values.

---

## 31. NORMAL CORRECTION

Calculate the true geometric normal.

Blend toward:

```glsl
  vec3 uprightNormal =
      vec3(0.0, 1.0, 0.0);
```

using a configurable parameter:

```glsl
  vec3 correctedNormal =
      normalize(
          mix(
              geometricNormal,
              uprightNormal,
              normalUprightBlend
          )
      );
```

This is intended to prevent harsh dark edges and produce a softer, fuller field appearance.

---

## 32. TWO-SIDED LIGHTING

Grass must render correctly from both sides.

Use:

```glsl
  gl_FrontFacing
```

and flip the normal when necessary:

```glsl
  vec3 N = correctedNormal;

  if (!gl_FrontFacing)
      N = -N;
```

Implement inexpensive two-sided diffuse lighting.

Do not introduce expensive physically based shading unless WoWee already requires it.

---

## 33. SUBSURFACE / TRANSLUCENCY APPROXIMATION

Implement a subtle fake vegetation subsurface effect.

Use the relationship between:

* light direction
* view direction
* blade normal

to approximate sunlight passing through thin grass.

The effect should be subtle.

Combine:

```text
  base color
  +
  diffuse lighting
  +
  ambient/environment contribution
  +
  back-lighting/subsurface approximation
```

Keep the fragment shader inexpensive.

Millions of grass fragments may be rendered.

---

## 34. GRASS GENERATION

Create or integrate a grass population generator.

It should support:

```text
  world-space distribution
  density
  seed
  orientation
  scale
  wind influence
  curve variation
  grass type
  terrain suitability
```

Generation must sample the underlying terrain.

Do not generate grass uniformly across every terrain surface.

---

## 35. TERRAIN → VEGETATION PROFILE

Create a configurable mapping from terrain/material/biome information to vegetation profiles.

Conceptually:

```text
  Terrain Material
          ↓
  Vegetation Profile
          ↓
  Grass Type
          ↓
  Density
          ↓
  Blade Parameters
```

Example:

```text
  lush_grass:
      density = high
      height = 0.4–1.0
      color = green
      wind = strong

  dry_grass:
      density = medium
      height = 0.2–0.6
      color = yellow/brown
      wind = moderate

  rock_grass:
      density = low
      height = 0.1–0.3
      color = muted

  bare_dirt:
      density = 0
```

These are examples only.

Use actual WoWee terrain/material data to determine appropriate profiles.

---

## 36. DETERMINISTIC RANDOMNESS

Grass variation should be deterministic.

Use world position, tile/chunk coordinates, and a stable seed.

Do not use frame-dependent random values.

A blade should not change identity or placement between frames.

This is especially important for:

* grass color
* height
* orientation
* species
* wind phase
* density

---

## 37. GRASS LOD

Support progressive density reduction.

Conceptually:

```text
  near:
      full vegetation

  medium:
      reduced density

  far:
      sparse vegetation

  very far:
      culled
```

Initially, LOD can be implemented through GPU density falloff.

Structure the system so hierarchical vegetation LOD can be added later.

---

## 38. RENDERING PIPELINE

Integrate the grass graphics pipeline with WoWee's existing pipeline architecture.

The final draw should be GPU-driven:

```cpp
  vkCmdDrawIndexedIndirect(...)
```

or WoWee's equivalent abstraction.

The CPU must not determine the visible blade count every frame.

The compute shader produces the indirect draw count.

---

## 39. DESCRIPTORS

A conceptual layout may include:

```text
  source grass SSBO
  visible grass SSBO
  camera/environment data
  wind/noise texture
  entity/interactor buffer
  indirect/counter data
  terrain data
```

Adapt the exact descriptor layout to WoWee.

Do not force a new descriptor architecture if one already exists.

---

## 40. VULKAN FEATURE REQUIREMENTS

Inspect WoWee's actual Vulkan feature negotiation.

Do not assume features/extensions are available.

Potentially relevant features include:

```text
  synchronization2
  descriptor indexing
  draw indirect count
  buffer device address
  shaderInt64
```

Only require features that are actually necessary.

If WoWee already enables a feature, use it where beneficial.

If it does not, determine whether the implementation can avoid requiring it.

Do not introduce unnecessary hardware requirements.

---

## 41. PERFORMANCE

The system should be designed for:

* millions of potential blades
* minimal CPU work
* minimal CPU/GPU synchronization
* coherent memory access
* low descriptor overhead
* efficient vertex processing
* inexpensive fragment shading
* minimal atomic contention

Pay particular attention to:

* source blade memory bandwidth
* visible blade memory bandwidth
* compute occupancy
* atomic counter contention
* vertex shader cost
* fragment overdraw
* terrain sampling cost

Grass is inherently a high-overdraw workload.

Optimize for the actual GPU rather than theoretical elegance.

---

## 42. DEBUGGING

Use WoWee's existing debug UI if available.

Expose useful values:

```text
  source blade count
  visible blade count
  culling percentage
  terrain-rejected count if available
  grass draw count
  compute dispatch size
```

Useful debug toggles:

```text
  disable culling
  disable terrain masking
  disable wind
  disable player interaction
  show terrain grass mask
  show blade bounds
  show frustum
  show density
  show only near grass
  show grass type
```

These should be integrated into existing WoWee debugging infrastructure where possible.

---

## 43. VALIDATION CASES

Explicitly test:

```text
  lush grass terrain
  grass → dirt transition
  grass → dry dirt transition
  grass → rock transition
  grass → road transition
  bare terrain
  dry terrain
  rocky terrain
  steep terrain
  mixed terrain materials
  terrain with multiple splat layers
  terrain streaming
```

Check for:

* grass on dry dirt
* grass on bare dirt
* grass on roads
* grass on rock where it should not exist
* incorrect colors
* incorrect vegetation type
* hard density boundaries
* floating grass
* buried grass
* grass appearing above/below terrain
* unstable random placement
* wind discontinuities
* visible popping
* excessive overdraw

---

## 44. IMPORTANT VISUAL REQUIREMENT

The final result should satisfy this visual rule:

│ ***The grass should look like it belongs to the terrain beneath it.***

If the player looks from a grassy area toward a dirt patch, the grass should naturally thin and disappear.

If the terrain becomes dry, vegetation should become appropriately sparse/dry.

If the terrain becomes rocky, grass should become sparse and adapted.

If the terrain contains no vegetation, there should be **no grass**.

Do not solve this by simply drawing a transparent grass texture over the terrain.

The vegetation must be structurally generated from terrain information.

---

## 45. CPU/GPU RESPONSIBILITY

Prefer:

```text
  CPU
   ├── terrain/world data
   ├── vegetation profiles
   ├── source blade generation
   ├── camera state
   ├── environment/wind parameters
   └── entity positions

  GPU
   ├── terrain suitability where appropriate
   ├── frustum culling
   ├── distance culling
   ├── density falloff
   ├── compaction
   ├── indirect command generation
   ├── blade geometry
   ├── wind
   ├── player interaction
   └── grass shading
```

Do not introduce a CPU readback of the visible blade count.

---

## 46. IMPLEMENTATION STRUCTURE

Use WoWee's existing architecture.

If new components are appropriate, a possible conceptual structure is:

```text
  GrassRenderer
  GrassBuffers
  GrassCompute
  GrassPipeline
  GrassPopulation
  GrassTerrainAdapter
  GrassShaders
```

But only create these components if they fit WoWee's existing design.

Do not create abstraction layers simply to satisfy this document.

---

## 47. NO PLACEHOLDER IMPLEMENTATION

Do not return a toy implementation.

Do not leave:

```text
  TODO
  FIXME
  pseudo-code
  imaginary APIs
  imaginary filenames
  placeholder Vulkan calls
  unimplemented shader functions
```

If an existing WoWee component is required, inspect it.

Use its actual API.

If something does not exist, implement it.

---

## 48. BUILD AND VALIDATION

After implementation:

1. Build WoWee.
2. Fix compiler errors.
3. Fix shader compilation errors.
4. Verify descriptor compatibility.
5. Verify Vulkan buffer usage flags.
6. Verify synchronization.
7. Verify command ordering.
8. Verify indirect draw arguments.
9. Run validation layers where available.
10. Check for validation errors.
11. Check for device loss.
12. Check for GPU synchronization issues.
13. Verify grass appears.
14. Verify terrain masking.
15. Verify culling.
16. Verify wind.
17. Verify player/NPC interaction.
18. Verify multiple frames in flight.
19. Verify terrain streaming if applicable.

If a full runtime environment is unavailable, perform every repository/build/shader/static validation possible and
clearly state what could not be runtime-tested.

---

## 49. FINAL REPORT

After implementation, provide:

1. Files changed.
2. Files created.
3. Existing WoWee systems reused.
4. New systems added.
5. GPU buffer layouts.
6. C++/GLSL structure compatibility.
7. Descriptor layouts.
8. Compute dispatch architecture.
9. Terrain integration.
10. Terrain → vegetation mapping.
11. Frustum culling.
12. Distance culling.
13. Compaction.
14. Indirect draw flow.
15. Synchronization/barriers.
16. Procedural blade generation.
17. Bézier implementation.
18. Wind implementation.
19. Player/NPC interaction.
20. Shading implementation.
21. Performance considerations.
22. Vulkan feature requirements.
23. Build results.
24. Runtime validation results.
25. Remaining limitations.

---

## 50. FINAL ARCHITECTURAL PRINCIPLE

Do not treat this specification as a reason to build a separate grass engine beside WoWee.

Build a **WoWee-native GPU-driven vegetation system**.

The existing WoWee renderer, terrain system, world representation, entity system, resource system, and Vulkan
architecture are authoritative.

The desired final pipeline is:

```text
                  WORLD / TERRAIN
                         │
                         ▼
                Terrain Materials
                         │
                         ▼
               Vegetation Suitability
                         │
                         ▼
                Grass Population
                         │
                         ▼
                Source Blade Buffer
                         │
                         ▼
                ┌─────────────────┐
                │  GPU COMPUTE    │
                │                 │
                │ Terrain Mask    │
                │ Frustum Cull    │
                │ Distance Cull   │
                │ Density         │
                │ Compaction      │
                └────────┬────────┘
                         │
                         ▼
               Visible Blade Buffer
                         │
                         ▼
              Indirect Draw Command
                         │
                         ▼
                Procedural Vertex
                         │
               ┌─────────┼─────────┐
               ▼         ▼         ▼
             Bézier    Wind     Interaction
               │         │         │
               └─────────┼─────────┘
                         ▼
                   Grass Geometry
                         │
                         ▼
                  Stylized Lighting
                         │
               ┌─────────┼─────────┐
               ▼         ▼         ▼
            Terrain    Two-Sided   Subsurface
             Color     Lighting    Approximation
               │         │         │
               └─────────┼─────────┘
                         ▼
                   Final Grass
```

The single most important rule is:

**If the terrain says there is no grass there, the grass renderer must not put grass there.**

Inspect first.

Integrate second.

Implement third.

Validate everything.
