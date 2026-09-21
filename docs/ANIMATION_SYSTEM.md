# Animation System

FSM-based animation system. `CharacterAnimator` is written for any character
(player, NPC, companion), but today only the local player runs through it: the
Renderer owns one `AnimationController`, which owns one `CharacterAnimator`.
NPCs and other players are animated by direct `CharacterRenderer::playAnimation`
calls from `src/core/` (the per-frame creature and player loops in
`application.cpp`, `entity_spawner_processing.cpp`, and the combat, emote and
death callbacks in `animation_callback_handler.cpp`).

## Architecture

```
AnimationController          (thin adapter - bridges Renderer ↔ CharacterAnimator)
  └─ CharacterAnimator       (FSM composer - implements ICharacterAnimator)
       ├─ CombatFSM          (stun, hit reaction, spell cast, melee, ranged, charge)
       ├─ ActivityFSM         (emote, loot, sit/stand/kneel/sleep)
       ├─ LocomotionFSM       (idle, walk, run, sprint, jump, swim, strafe)
       └─ MountFSM            (mount idle, mount run, flight)

AnimationManager             (registry of CharacterAnimator instances by ID - unused)
AnimCapabilitySet            (probed once per model - cached resolved anim IDs)
AnimCapabilityProbe          (queries which animations a model supports)
```

### Priority Resolution

`CharacterAnimator::resolveAnimation()` runs every frame. The first FSM to
return a valid `AnimOutput` wins:

1. **Mount** - if mounted, return `MOUNT` (overrides everything)
2. **Combat** - stun > hit reaction > spell > charge > melee/ranged > combat idle
3. **Activity** - emote > loot > sit/stand transitions
4. **Locomotion** - run/walk/sprint/jump/swim/strafe/idle

If no FSM produces a valid output, the last animation continues (STAY policy).

### Overlay Layer

After resolution, `applyOverlays()` substitutes stealth animation variants
(stealth idle, stealth walk, stealth run) without changing sub-FSM state.

## File Map

### Headers (`include/rendering/animation/`)

| File | Purpose |
|---|---|
| `i_animator.hpp` | Base interface: `onEvent()`, `update()` |
| `i_character_animator.hpp` | 20 virtual methods (combat, spells, emotes, mounts, etc.) |
| `character_animator.hpp` | FSM composer - the single animator class |
| `locomotion_fsm.hpp` | Movement states: idle, walk, run, sprint, jump, swim |
| `combat_fsm.hpp` | Combat states: melee, ranged, spell cast, stun, hit reaction, sheathe/unsheathe. A dual-wielder's hand is chosen once, when a swing begins |
| `activity_fsm.hpp` | Activity states: emote, loot, sit/stand/kneel |
| `mount_fsm.hpp` | Mount states: idle, run, jump, rear-up, flight (taxi flights configured separately) |
| `anim_capability_set.hpp` | Probed capability flags + resolved animation IDs |
| `anim_capability_probe.hpp` | Probes a model for available animations |
| `anim_event.hpp` | `AnimEvent` enum (MOVE_START, MOVE_STOP, JUMP, etc.) |
| `animation_ids.hpp` | `anim::` constants for every AnimationData.dbc ID, `nameFromId()`, `validateAgainstDBC()` |
| `melee_anim_chains.hpp` | Ordered attack fallback chains per weapon kind, shared by AnimationController and the capability probe |
| `animation_manager.hpp` | Registry of CharacterAnimator instances (`get`/`remove` only; nothing creates or owns one) |
| `weapon_type.hpp` | `WeaponLoadout` struct, `RangedWeaponType` enum |
| `emote_registry.hpp` | Emote name → animation ID lookup |
| `footstep_driver.hpp` | Footstep sound event driver |
| `sfx_state_driver.hpp` | State-transition SFX (jump, land, swim enter/exit) |

### Sources (`src/rendering/animation/`)

| File | Purpose |
|---|---|
| `character_animator.cpp` | ICharacterAnimator implementation + priority resolver |
| `locomotion_fsm.cpp` | Locomotion state transitions + resolve logic |
| `combat_fsm.cpp` | Combat state transitions + resolve logic |
| `activity_fsm.cpp` | Activity state transitions + resolve logic |
| `mount_fsm.cpp` | Mount state transitions + resolve logic |
| `anim_capability_probe.cpp` | Model animation probing |
| `animation_ids.cpp` | ID → name table, DBC cross-check |
| `animation_manager.cpp` | Registry lookup and removal |
| `emote_registry.cpp` | Emote database |
| `footstep_driver.cpp` | Footstep timing logic |
| `sfx_state_driver.cpp` | SFX transition detection |

### Controller (`include/rendering/animation_controller.hpp` + `src/rendering/animation_controller.cpp`)

Thin adapter that:
- Collects per-frame input from camera/renderer → `CharacterAnimator::FrameInput`
- Forwards state changes (combat, emote, spell, mount, etc.) → `CharacterAnimator`
- Reads `AnimOutput` → applies via `CharacterRenderer`
- Discovers the mount's animations in `setMounted()`, runs `MountFSM::evaluate()`
  for the mount and seats the rider (`updateMountedAnimation()`)
- Resolves the melee sequence and its duration through `melee_anim_chains.hpp`
- Owns footstep and SFX drivers

## Key Types

- **`AnimEvent`** - discrete events: `MOVE_START`, `MOVE_STOP`, `JUMP`, `LANDED`, `MOUNT`, `DISMOUNT`, etc.
- **`AnimOutput`** - result of FSM resolution: `{animId, loop, valid}`. `valid=false` means STAY.
- **`AnimCapabilitySet`** - probed once per model load. Caches resolved IDs and capability flags.
- **`CharacterAnimator::FrameInput`** - per-frame input struct (movement flags, timers, animation state queries).

## Adding a New Animation State

1. Decide which FSM owns the state (combat, activity, locomotion, or mount).
2. Add the state enum to the FSM's `State` enum (`MountState` for the mount).
3. Add transitions in the FSM's `updateTransitions()` and the animation choice in
   its `resolve()` (`MountFSM::evaluate()` for the mount).
4. Add resolved ID fields to `AnimCapabilitySet` if the animation needs model probing.
5. If the state needs external triggering, add a method to `ICharacterAnimator` and implement in `CharacterAnimator`.

## Tests

Locomotion, combat and activity FSMs each have a test file in `tests/`
(MountFSM has none):
- `test_locomotion_fsm.cpp`
- `test_combat_fsm.cpp`
- `test_activity_fsm.cpp`
- `test_anim_capability.cpp`
- `test_animation_ids.cpp`
- `test_melee_anim_chains.cpp`
- `test_footstep_paths.cpp`, `test_m2_footstep_events.cpp`

Run all tests:
```bash
cd build && ctest --output-on-failure
```
