# Attached item materials

The character renderer also draws equipment models attached to bones. Their
submesh numbers and texture alpha are not character hair or body-atlas data:
opaque equipment batches must ignore texture alpha, alpha-key batches must test
it, and blended batches must not be forced to cutout. This is selected for
attached items including helms and shoulders, regardless of whether the wearer
is a player or creature. Character, creature, scene and enchant-effect
handling remain separate.

## Bloodsurge

The heroic item 50732 points to display 64530, model
`sword_1h_icecrownraid_d_03.m2`, and the blue item texture. The model's solid
batch is blend mode 0 and samples replaceable texture type 2. The atlas has
transparent alpha over painted RGB: in a deterministic triangle-centre sample,
688 of its 766 solid triangles hit painted pixels below half alpha. The old
character heuristic alpha-tested that batch and erased most of the sword. The
solid surfaces now look good in-game.

`test_character_material_policy` covers domain selection and material
decisions. When `WOWEE_BLOODSURGE_WEAPON_DIR` points to
`item/objectcomponents/weapon`, `test_weapon_material_assets` also checks the
Bloodsurge model, skin and texture.

At partial instance opacity the renderer switches opaque batches to a blended
pipeline, whose shader still multiplies opacity by the texture alpha. Thus an
opaque item or creature whose atlas has unrelated low alpha may remain too
transparent while fading or in a translucent state; that is not fixed here.

## Remaining effects

This correction addresses the opaque item surfaces, not all weapon effects.
Bloodsurge also has two additive batches and one particle emitter. The
character renderer currently suppresses additive batches on non-enchant
attachments to avoid detached cards on other items, does not advance those
attachments' own animation, and does not draw their particle emitters. Restore
those effects separately with appropriate blend/animation handling and
comparisons across multiple weapons; simply removing the suppression would
reintroduce the detached-card regression.
