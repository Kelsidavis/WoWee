# Foliage face culling

The Storm Peaks snow trees around an Icecrown viewpoint contain
near-coincident triangles facing opposite directions, sometimes sampling
different parts of the texture. Drawing both sides can make those triangles
compete for depth. The M2 materials for these trees are one-sided; the camera's
projection makes the authored front face counter-clockwise in Vulkan.

In a v3.1.54 test build, the culling option stopped the flicker with wind both
enabled and disabled. With wind off, more snow was visible on these trees.
Other areas and lighting settings still need testing.

When enabled, the main M2 pass culls back faces only when a foliage batch has
a known, one-sided, opaque or alpha-keyed material. Sky, transparent effects,
other models, and materials marked two-sided retain their previous pipelines.
An instanced LOD group containing a reflected or degenerate model transform
also stays two-sided, because a draw has only one front-face convention.
Shadow-caster face culling is unchanged.

Launch with `WOWEE_M2_FOLIAGE_CULL_DIAG=1` to enable the experiment. Without
that flag, foliage retains the previous two-sided behavior. For comparisons,
`WOWEE_M2_NO_FOLIAGE_CULL=1` overrides the diagnostic flag.
`WOWEE_M2_NO_FOLIAGE_WIND=1` is a separate wind diagnostic and is not needed
for the culling experiment.

Other foliage families and lighting/shadow configurations still need in-game
comparison. In particular, a passing projection test or the Icecrown result
does not establish visual parity across the world.
