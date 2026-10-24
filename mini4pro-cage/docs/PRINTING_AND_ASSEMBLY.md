# Printing and assembly (v0.3)

**The library printer is not confirmed.** These are generic FDM settings for a 0.4 mm nozzle.

## Parts

Each side has four parts. The right side is a mirror image of the left.

| File | Footprint (mm) | Height | Orientation |
|---|---|---|---|
| `PROVISIONAL_*_body_front.stl` | 172 × 145 | 35.5 | Bottom level on the bed, clip openings up |
| `PROVISIONAL_*_body_rear.stl` | 180 × 117 (fits diagonally) | 35.5 | Same |
| `PROVISIONAL_*_lid_front.stl` | 168 × 153 | 5.2 | Top face on the bed, pads and holes up |
| `PROVISIONAL_*_lid_rear.stl` | 159 × 153 | 5.2 | Same |
| `front_/rear_mount_fit_test.stl` | 128 × 113 | 33.1 | As exported |
| `*_clip_clearance_ladder.stl` | 61 × 10 | 12.4 | As exported |

Every file fits a 180 × 180 mm bed with 4 mm margin at some rotation, so it fits any common library printer. **Do not scale.**

## Material

| Use | Material |
|---|---|
| Clip ladders and fit tests | PLA is fine for geometry checks |
| Final cage | **PETG** preferred. PLA+ or tough PLA if that is all the library has. |
| Avoid | Plain PLA for the final cage (brittle), TPU (too flexible) |

Do the clip ladder in the final material. Snap fits behave differently in PLA and PETG.

## Slicer settings

| Setting | Value |
|---|---|
| Layer height | 0.16 to 0.20 mm |
| Walls | 3 or more (the bars then print solid) |
| Infill | 100% (the bars are too thin for infill to matter) |
| Supports | **Off** for lids. Bodies: try off first. The validator found 2.5% overhang, mostly under the clip floors. Enable supports only if the clip floor sags. |
| Brim | 3 to 5 mm. The bars are 2 to 3 mm wide. |
| First layer | Slow, with a clean bed. The lids are mostly first layer. |

## Weight

| Version | PETG |
|---|---|
| Default (`python mini4pro_cage.py`) | about 69 g for both sides |
| `variants/light.json` | about 52 g |

No hardware is required.

## Assembly

1. Remove brims. Clean the dovetail pockets and peg holes with a hobby knife.
2. Join each body front to body rear: slide the tongues down into the pockets. Add a drop of CA glue.
3. Join each lid front to lid rear the same way.
4. Press the lid onto the body: the 14 pegs on the rib tops go into the holes in the lid pads. Glue if the fit is loose.
5. Unfold the drone arms. Hold the cage under one side so each clip sits below its arm, next to the motor. Push up until both clips snap over the arms.
6. Turn every propeller by hand. Nothing may touch.

Removal: unsnap the two clips on each side. The drone cannot fold with the cages on.
