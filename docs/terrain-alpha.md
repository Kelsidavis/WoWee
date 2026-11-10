# Terrain alpha maps

Compressed MCAL runs store their texel count directly. Adding one to each run
misaligns successive runs and produces visible streaks in Northrend. Decoding
also stops at the next layer's offset, and the last-row/column repair for
four-bit maps is skipped when the MCNK do-not-fix flag is set.

The renderer uses WDT MPHD's 8-bit alpha flags to select the independent
coverage weights [documented for Northrend](https://wowdev.wiki/ADT/v18#Rendering),
retaining sequential-over blending otherwise. The MPHD flags specify alpha bit
depth; using them to select the blend formula is an association that still needs
visual comparisons on other maps. The same selection is passed to grass
placement. A missing WDT falls back to the previous sequential blend.

This selection currently applies to raw ADT tiles. Custom WHM/WOT terrain and
asset-tree WHM/WOT sidecars do not yet retain the WDT alpha/blend mode, and
therefore keep the existing sequential behavior.

`test_adt_alpha` covers synthetic compressed and four-bit maps. It can also
check an extracted 3.3.5a Icecrown tile when `WOWEE_ICECROWN_ADT` is set to
`northrend_31_16.adt`. `test_grass_terrain` checks blend weights and propagation
to terrain meshes. The Icecrown ground was checked in-game; other zones and the
WHM/WOT path still need testing.
