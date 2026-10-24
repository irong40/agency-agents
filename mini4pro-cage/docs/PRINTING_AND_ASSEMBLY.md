# Printing and assembly

**The library printer is not confirmed yet.** The settings below are generic FDM settings that work on the machines public libraries commonly run (Prusa, Bambu Lab, Ultimaker, Creality, MakerBot). Adjust them once you know the machine.

## Build volume

Every part fits the smallest common bed (180 × 180 mm, Prusa MINI+ and Bambu A1 mini) with at least 2.5 mm margin. The modular split keeps it that way, so nothing needs scaling. **Never scale these files:** scaling changes the clip fit and the propeller clearance.

| File | Footprint on bed (mm) | Height (mm) | Rotate on bed |
|---|---|---|---|
| `front_mount_fit_test.stl` / `rear_` | 169 × 28 | 24.9 | diagonal |
| `*_clip_clearance_ladder.stl` | 61 × 14 | 12.4 | none |
| `PROVISIONAL_*_front_module.stl` | 171 × 123 | 24.9 | about 45° |
| `PROVISIONAL_*_rear_module.stl` | 175 × 120 | 24.9 | about 40° |

The STLs are already oriented: upper ring on the bed, clip opening facing up.

## Material

| Use | Material | Why |
|---|---|---|
| Clip ladders and fit tests | PLA is fine | Cheap, fast, checks geometry |
| Final cage | **PETG** preferred | Flexes instead of shattering on impact. Snap clips last longer. |
| Final cage if only PLA is offered | PLA+ or tough PLA | Plain PLA cracks at the clips and goes brittle in cold air |
| Not recommended | TPU | Too flexible: the ring would deflect into the props |

Do the clip ladder test **in the final material**. PETG and PLA snap differently.

## Slicer settings (0.4 mm nozzle)

| Setting | Value |
|---|---|
| Layer height | 0.16 to 0.20 mm |
| Perimeters / walls | 3 (the thin members then print solid) |
| Infill | 100%, or 40% gyroid. The rings print solid either way. |
| Supports | **On, tree or organic, build plate only.** The struts and clip sit above the bed. |
| Brim | 5 mm. The upper ring touches the bed with only a 2.6 mm strip. |
| Bridging | Default bridge settings. The lower rail bridges up to 28 mm between ribs. |
| Speed | Outer walls 40 mm/s or slower for clean clip surfaces |

The validator measured 10 to 15% of each part's surface as more than 45° overhang in this orientation. Expect light support on the struts and under the clip.

## Print order

1. Both clip clearance ladders (each about 3 g).
2. Both fit tests (each about 4.5 g).
3. Only after the fit tests pass, and after the model is regenerated with your measurements: the four cage modules (left front, left rear, right front, right rear).

## Hardware

- 4× M2 × 10 mm screws with nuts, **nylon preferred**. Steel screws sit near the arms, and the compass location on the Mini 4 Pro is not known here.
- Or 4 short lengths of 1.75 mm filament as pins. Glue them in, or melt the ends flat with a soldering iron.
- Optional: 2.5 mm zip ties for the backup groove on each clip.

## Assembly

1. Remove supports and brim. Clean the inside of each clip with a hobby knife: any stringing there changes the fit.
2. Join each front module to its rear module at the outboard lap joint. Insert the two screws or pins. Each side is now one cage.
3. Unfold the drone arms fully.
4. Line the cage up from below so each clip sits under its arm. Press the front clip onto the arm until it snaps, then the rear clip.
5. Turn every propeller by hand through a full turn. Nothing may touch.
6. Optional: a zip tie through each clip groove.

**Removal:** unsnap the clips. The cage stays in one piece. The drone cannot fold with the cage installed, so remove it before folding.
