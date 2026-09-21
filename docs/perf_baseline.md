# Performance Baseline - WoWee

> Phase 0.3 deliverable. Measurements taken before any optimization work.
> Re-run after each phase to quantify improvement.
>
> **Status:** the scenario numbers below were never recorded, and frame-time
> work has landed since (v3.1.24 took the frame from 19.7ms to 15.5ms - see
> CHANGELOG.md), so whatever is recorded now is not a pre-optimization baseline.

## Tracy Profiler Integration

Tracy is integrated under the `WOWEE_ENABLE_TRACY` CMake option (default: OFF).
When enabled, zero-cost zone markers instrument the following critical paths.

> **Setup**: Tracy is *not* vendored in this repo. `CMakeLists.txt` looks for
> `extern/tracy/public/TracyClient.cpp` and `extern/tracy/public/` headers
> under `WOWEE_ENABLE_TRACY`. Clone Tracy into that path before configuring
> with `-DWOWEE_ENABLE_TRACY=ON`:
>
> ```bash
> git clone --depth 1 --branch v0.11.1 https://github.com/wolfpld/tracy extern/tracy
> ```

### Instrumented Zones

| Zone Name | File | Purpose |
|-----------|------|---------|
| `Application::run` | src/core/application.cpp | Main loop entry |
| `Application::update` | src/core/application.cpp | Per-frame game logic |
| `Renderer::beginFrame` | src/rendering/renderer.cpp | Vulkan frame begin |
| `Renderer::endFrame` | src/rendering/renderer.cpp | Post-process + present |
| `Renderer::update` | src/rendering/renderer.cpp | Renderer per-frame update |
| `Renderer::renderWorld` | src/rendering/renderer.cpp | Main world draw call |
| `Renderer::renderShadowPass` | src/rendering/renderer.cpp | Shadow depth pass |
| `PostProcess::execute` | src/rendering/post_process_pipeline.cpp | FSR/FXAA post-process |
| `HiZSystem::buildPyramid` | src/rendering/hiz_system.cpp | Hi-Z depth pyramid build dispatch |
| `M2::computeBoneMatrices` | src/rendering/m2_renderer_internal.h | CPU skeletal animation |
| `M2Renderer::update` | src/rendering/m2_renderer_render.cpp | M2 instance update + culling |
| `TerrainManager::update` | src/rendering/terrain_manager.cpp | Terrain streaming logic |
| `TerrainManager::processReadyTiles` | src/rendering/terrain_manager.cpp | GPU tile uploads |
| `TerrainManager::processPendingUnloads` | src/rendering/terrain_manager.cpp | Time-budgeted tile unloads |
| `ADTLoader::load` | src/pipeline/adt_loader.cpp | ADT binary parsing |
| `AssetManager::loadTexture` | src/pipeline/asset_manager.cpp | Texture loading (DDS or PNG override, else BLP) |
| `AssetManager::loadDBC` | src/pipeline/asset_manager.cpp | DBC data file loading |
| `WorldSocket::update` | src/network/world_socket.cpp | Network packet dispatch |

`FrameMark` placed at the frame boundary in the `Application::run` main loop, just before `update()`, to track FPS.

### How to Profile

```bash
# Build with Tracy enabled
mkdir -p build_tracy && cd build_tracy
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DWOWEE_ENABLE_TRACY=ON
cmake --build . --parallel $(nproc)

# Run the client - Tracy will broadcast on default port (8086)
cd bin && ./wowee

# Connect with Tracy profiler GUI (separate download from https://github.com/wolfpld/tracy/releases)
# Or capture from CLI: tracy-capture -o trace.tracy
```

### Without Tracy

The client also measures itself, with no build option needed:

- `WOWEE_FRAME_PROFILE=1` - every 10 seconds, logs the average and worst
  time of each CPU stage of the frame and the GPU's per-pass timestamps from
  the last completed frame, at warning level instead of info.
- `WOWEE_PASS_ABLATION=1` - switches the world passes (terrain, grass, WMO,
  M2, ground clutter, far doodads, characters, sky, shadows) off one at a time
  for a few seconds each, with a baseline at both ends, and logs what the
  frame did without each. Stand still outdoors until the table is logged.
- `WOWEE_SINGLE_THREAD_RECORD=1` - records the world inline on the main thread
  instead of into parallel secondary command buffers, so each pass gets its
  own GPU mark (grass is otherwise counted inside terrain).

On MoltenVK a GPU timestamp resolves to the render pass containing it, so
every mark inside the scene pass reads the same clock: the first holds the
whole pass and the rest read near zero. The frame profile says so when it
sees it. Use the ablation run to attribute the scene pass there.

## Baseline Scenarios

> **TODO:** Record measurements once profiler is connected to a running instance.
> Each scenario should record: avg FPS, frame time (p50/p95/p99), and per-zone timings.

### Scenario 1: Stormwind (Heavy M2/WMO)
- **Location:** Stormwind City center
- **Load:** Dense M2 models (NPCs, doodads), multiple WMO interiors
- **Avg FPS:** _pending_
- **Frame time (p50/p95/p99):** _pending_
- **Top zones:** _pending_

### Scenario 2: The Barrens (Heavy Terrain)
- **Location:** Central Barrens
- **Load:** Many terrain tiles loaded, sparse M2, large draw distance
- **Avg FPS:** _pending_
- **Frame time (p50/p95/p99):** _pending_
- **Top zones:** _pending_

### Scenario 3: Dungeon Instance (WMO-only)
- **Location:** Any dungeon instance (e.g., Deadmines entrance)
- **Load:** WMO interior rendering, no terrain
- **Avg FPS:** _pending_
- **Frame time (p50/p95/p99):** _pending_
- **Top zones:** _pending_

## Notes

- When `WOWEE_ENABLE_TRACY` is OFF (default), all `ZoneScopedN` / `FrameMark` macros expand to nothing - zero runtime overhead.
- Tracy requires a network connection to capture traces. Run the Tracy profiler GUI or `tracy-capture` CLI alongside the client.
- Debug builds are significantly slower due to -Og and no LTO; use RelWithDebInfo for representative measurements.
