# Threading Model

This document describes the threading architecture of WoWee, the synchronisation
primitives that protect shared state, and the conventions that new code must
follow.

---

## Thread Inventory

| #  | Name / Role            | Created At                                      | Lifetime                      |
|----|------------------------|-------------------------------------------------|-------------------------------|
| 1  | **Main thread**        | `Application::run()` (`application.cpp`, called from `main.cpp`) | Entire session                |
| 2  | **Async network pump** | `WorldSocket::asyncPumpLoop()` started by `startAsyncPump()` inside `WorldSocket::connect()` (`world_socket.cpp`) | Connect → disconnect          |
| 3  | **Terrain workers**    | `TerrainManager::initialize()` spawns the worker pool that runs `workerLoop()` (`terrain_manager.cpp`) | Map load → `stopWorkers()` on shutdown |
| 4  | **Frame workers**      | `core::ThreadPool::frameWorkers()` (`include/core/thread_pool.hpp`), created on first use | Process lifetime |
| 5  | **I/O workers**        | `core::ThreadPool::ioWorkers()` (same header), two threads, created on first use | Process lifetime |
| 6  | **Watchdog**           | Inline lambda `std::thread` started in `Application::run()` (`application.cpp:1291`) | Start of main loop → exit from `run()` (joined by a scope guard) |
| 7  | **World preload**      | `WorldLoader` preload, up to four `std::thread`s held in `WorldPreload::workers` (`world_loader.cpp`) | Preload start → `cancelWorldPreload()` joins them |
| 8  | **Update check**       | `UpdateCheck::start()` (`update_check.cpp`), called from `Application` initialization | One request → joined in `~UpdateCheck()` |
| 9  | **Asset builder**      | `assets::Job::start()` (`tools/asset_manager/job.cpp`), the extractor's own pool inside it, and `App::packThread` for pack save/install (`tools/asset_manager/panel.cpp`) - only in a client built with `WOWEE_HAVE_ASSET_PANEL` | One build/pack → joined by `~Job()` / `~App()` |
| 10 | **Task-scoped**        | `std::async` / `std::thread(...).detach()` (various) | Task-scoped (entity model loading, normal-map gen, warden check processing, login backdrop decode) |

### Thread Responsibilities

* **Main thread** - SDL event pumping, frame pacing (`FramePacer`), game logic
  (entity update, camera, UI, Lua/FrameXML), GPU resource upload/finalization,
  command buffer recording other than the secondaries handed to the frame
  workers, Vulkan present.
* **Network pump** - `recv()` loop, header decryption, packet parsing.  Pushes
  parsed packets into `pendingPacketCallbacks_` (locked by `callbackMutex_`).
  The main thread drains this queue via `dispatchQueuedPackets()`.
  `WOWEE_NET_ASYNC_PUMP=0` disables the thread; the main thread then pumps
  the socket itself in `WorldSocket::update()`.
* **Terrain workers** - background ADT/WMO/M2 file I/O, mesh decoding, texture
  decompression.  Workers push completed `PendingTile` objects into `readyQueue`
  (locked by `queueMutex`).  The main thread finalizes (GPU upload) via
  `processReadyTiles()`.  `WOWEE_TERRAIN_WORKERS` overrides the pool size.
* **Frame workers** - per-frame parallel work submitted through
  `ThreadPool::submit()`: M2 and character bone animation, M2 visibility,
  camera floor queries, and the terrain/grass, WMO, M2, character and
  water/weather/effects secondary command buffers when parallel recording is enabled
  (`WOWEE_SINGLE_THREAD_RECORD` records inline on the main thread instead).
  Sized `hardware_concurrency() - 1`, clamped to 2..16.
* **I/O workers** - blocking file reads kept off the frame workers (music
  tracks, `MusicManager`).
* **Watchdog** - frame-stall detection every 250ms.  Reads
  `watchdogHeartbeatMs_` (atomic); after 1500ms without a beat it sets
  `watchdogRequestRelease` (atomic), and the main thread releases mouse
  capture, since SDL video calls are only safe from the main thread.
* **World preload** - reads the ADTs around the expected login position to
  warm the `AssetManager` file cache before Enter World.
* **Update check** - one HTTPS request to GitHub for the latest release tag;
  the result is read by the login screen under `UpdateCheck::mutex_`.
* **Asset builder** - extraction and pack jobs for the first-run screen; they
  report progress to the panel through mutex-guarded state and atomics.
* **Task-scoped** - short-lived tasks.  Each captures only the data it
  needs or uses a dedicated result channel (e.g. `std::future`,
  `completedNormalMaps_` with `normalMapResultsMutex_`).

---

## Shared State Map

### Legend

| Annotation              | Meaning |
|-------------------------|---------|
| `THREAD-SAFE: <mutex>`  | Protected by the named mutex/atomic. |
| `MAIN-THREAD-ONLY`      | Accessed exclusively by the main thread. No lock needed. |

### Asset Manager (`include/pipeline/asset_manager.hpp`)

| Variable                | Guard            | Notes |
|-------------------------|------------------|-------|
| `fileCache`             | `cacheMutex` (shared_mutex) | `shared_lock` for reads, `lock_guard` for writes/eviction |
| `dbcCache`              | `cacheMutex`     | `shared_lock` for the lookup, `lock_guard` for the insert; the load between them is unlocked, and the first table inserted for a name is the one every caller gets |
| `fileCacheTotalBytes`   | `cacheMutex`     | Written under exclusive lock only |
| `fileCacheAccessCounter`| `cacheMutex`     | Written under exclusive lock only |
| `fileCacheHits`         | `std::atomic`    | Incremented after releasing cacheMutex |
| `fileCacheMisses`       | `std::atomic`    | Never incremented (see Known Limitations) |

### Audio Engine (`src/audio/audio_engine.cpp`)

| Variable               | Guard                     | Notes |
|------------------------|---------------------------|-------|
| `gDecodedWavCache`     | `gDecodedWavCacheMutex` (shared_mutex) | `shared_lock` for cache hits, `lock_guard` for miss+eviction. Double-check after decoding. |

### World Socket (`include/network/world_socket.hpp`)

| Variable                  | Guard            | Notes |
|---------------------------|------------------|-------|
| `sockfd`, `encryptionEnabled`, `receiveBuffer`, `receiveReadOffset_`, `headerBytesDecrypted`, cipher state, `recentPacketHistory_` | `ioMutex_` | Consistent `lock_guard` in `send()` and `pumpNetworkIO()` |
| `pendingPacketCallbacks_` | `callbackMutex_` | Pump thread produces, main thread consumes in `dispatchQueuedPackets()` |
| `connected`               | `std::atomic<bool>` | Read every frame by `GameHandler`; atomic so the check does not wait on `ioMutex_` during a packet burst |
| `asyncPumpStop_`, `asyncPumpRunning_` | `std::atomic<bool>` | Memory-order acquire/release |
| `packetCallback`          | *implicit*       | Set once before `connect()` starts the pump thread |

### Terrain Manager (`include/rendering/terrain_manager.hpp`)

| Variable              | Guard                    | Notes |
|-----------------------|--------------------------|-------|
| `loadQueue`, `readyQueue`, `pendingTiles` | `queueMutex` + `queueCV` | Workers wait; main signals on enqueue/finalize |
| `tileCache_`, `tileCacheLru_` | `tileCacheMutex_` | Read/write by both main and workers |
| `uploadedM2Ids_`      | `uploadedM2IdsMutex_`    | Workers check, main inserts on finalize |
| `preparedWmoUniqueIds_`| `preparedWmoUniqueIdsMutex_` | Workers insert, main erases in `unloadTile()` |
| `missingAdtWarnings_` | `missingAdtWarningsMutex_` | Workers only |
| `workerRunning`       | `std::atomic<bool>`      | - |
| `placedDoodadIds`, `placedWmoIds`, `loadedTiles`, `failedTiles` | MAIN-THREAD-ONLY | Touched only from main-thread paths: finalization (`processReadyTiles()` → `advanceFinalization()`), tile streaming, and `unloadTile()` |

### Entity Manager (`include/game/entity.hpp`)

| Variable   | Guard            | Notes |
|------------|------------------|-------|
| `entities` | MAIN-THREAD-ONLY | All mutations via `dispatchQueuedPackets()` on main thread |

### Character Renderer (`include/rendering/character_renderer.hpp`)

| Variable                | Guard                     | Notes |
|------------------------|---------------------------|-------|
| `completedNormalMaps_` | `normalMapResultsMutex_`  | Detached threads push, main thread drains |
| `pendingNormalMapCount_`| `std::atomic<int>`       | acq_rel ordering; the last task out signals `normalMapDoneCV_`, which `shutdown()` and `clear()` wait on |

### Logger (`include/core/logger.hpp`)

| Variable    | Guard           | Notes |
|-------------|-----------------|-------|
| `minLevel_` | `std::atomic<int>` | Fast path check in `shouldLog()` |
| `fileStream`, `lastMessage_`, suppression state | `mutex` | Locked in `log()` |

### Application (`src/core/application.cpp`)

| Variable                | Guard              | Notes |
|-------------------------|--------------------|-------|
| `watchdogHeartbeatMs_`  | `std::atomic<int64_t>` | Member. Main stores through `beatWatchdog()` (each loop iteration, and from the world load, which presents its own frames); watchdog loads |
| `watchdogRequestRelease`| `std::atomic<bool>` | Local in `run()`. Watchdog stores, main exchanges |
| `watchdogRunning`       | `std::atomic<bool>` | Local in `run()`. Cleared by the scope guard that joins the watchdog |

### Update Check (`include/core/update_check.hpp`)

| Variable                  | Guard            | Notes |
|---------------------------|------------------|-------|
| `newerVersion_`, `releaseUrl_` | `mutex_`    | Check thread writes once, login screen reads every frame |
| `started_`, `finished_`   | `std::atomic<bool>` | - |

---

## Conventions for New Code

1. **Prefer `std::shared_mutex`** for read-heavy caches.  Use `std::shared_lock`
   for lookups and `std::lock_guard<std::shared_mutex>` for mutations.

2. **Annotate shared state** at the declaration site with either
   `// THREAD-SAFE: protected by <mutex_name>` or `// MAIN-THREAD-ONLY`.

3. **Keep lock scope minimal.**  Copy data under the lock, then process outside.

4. **Avoid detaching threads** when possible.  Prefer `std::async` with a
   `std::future` stored on the owning object so shutdown can wait for completion.
   Work that runs every frame goes on `ThreadPool::frameWorkers()` (blocking
   file reads on `ThreadPool::ioWorkers()`) rather than a `std::async` per
   frame, which creates and destroys an OS thread each call.  Unlike a
   `std::async` future, destroying a `ThreadPool` future does not wait for
   the task.  Pool tasks must not block on other pool tasks, except for the
   caller-runs-a-chunk pattern described in `thread_pool.hpp`.

5. **Use `std::atomic` for counters and flags** that are read/written without
   other invariants (e.g. cache hit stats, boolean run flags).

6. **No lock-order inversions.**  Current order (most-outer first):
   `ioMutex_` → `callbackMutex_` → `queueMutex` → `cacheMutex`.

7. **ThreadSanitizer** - run periodically with `-fsanitize=thread` to catch
   regressions:
   ```bash
   cmake -DCMAKE_CXX_FLAGS="-fsanitize=thread" .. && make -j$(nproc)
   ```

---

## Known Limitations

* `EntityManager::entities` relies on the convention that all entity mutations
  happen on the main thread through `dispatchQueuedPackets()`.  There is no
  compile-time enforcement.  If a future change introduces direct entity
  modification from the network pump thread, a mutex must be added.

* `packetCallback` in `WorldSocket` is set once before `connect()` and
  never modified afterwards.  This is safe in practice but not formally
  synchronized - do not change the callback after `connect()`.

* `fileCacheMisses` is declared as `std::atomic<size_t>` for consistency but is
  currently never incremented.  `fileCacheAccessCounter` is not a total either:
  it is bumped only when a missed file is inserted into the cache (hits do not
  touch it) and reset by `clearCache()`, so it approximates cached misses since
  the last clear.

