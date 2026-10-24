# Measurements needed

Set the drone on a flat table, arms fully unfolded, props on. Turn one blade of each prop straight outward. Calipers are best; a steel rule works for the long distances.

| # | Measure | How | Parameter |
|---|---|---|---|
| M1 | Prop diameter | Tip to tip, or the size printed on the blade | `prop_diameter` |
| M2 | Front to rear motor, same side | Prop screw centre to prop screw centre | `front_motor`, `rear_motor` x |
| M3 | Left front to right front motor | Centre to centre | `front_motor` y |
| M4 | Left rear to right rear motor | Centre to centre | `rear_motor` y |
| M5 | Prop hub height above the table, front and rear | Table to the top of the hub | rear z, `landing_z` |
| M6 | Arm width and height, front and rear | About 18 mm from the motor centre, toward the body. Record that distance. | `*_arm_w`, `*_arm_h`, `*_clip_dist` |
| M7 | Arm centre height above the table at that point | | `*_clip_dz` |
| M8 | Motor base diameter and height below the prop hub | Underside of the motor | hub ring, clip |
| M9 | Fuselage width at its widest | | `body_box` |
| M10 | Gimbal lens height and its distance ahead of the front motors | | `camera_pt` |

## Photos that cover several measurements

1. Straight down on the unfolded drone, on graph paper or a cutting mat, ruler in frame (M2 to M4, arm angles).
2. Side view at table height, ruler in frame (M5, M7, and the cage heights if you hold the product cage beside it).
3. Close-up under one front and one rear arm near the motor (M6, M8, vents and landing feet).

## Also useful

- A side photo of the reference product cage on the drone, to set `z_top` and `z_bot`.
- The library printer model and the filament types they stock.
