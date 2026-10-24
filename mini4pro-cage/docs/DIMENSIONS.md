# Dimension table (v0.2)

Every value lives in `cad/mini4pro_cage.py` (class `Params`) and can be overridden with a JSON file.

## Source categories

| Tag | Meaning |
|---|---|
| **VERIFIED** | Measured on the real drone or read from a source opened during this work. **Nothing is in this category yet.** |
| **DERIVED** | Computed from a published figure. The figure is recalled from DJI's public spec sheet, not opened during this session, because dji.com was unreachable from the build environment. |
| **ASSUMED** | Placeholder chosen to make the model buildable. Must be measured. |
| **DESIGN** | Engineering choice. Safe to tune. |

## Drone and propeller

| Parameter | Value (mm) | Tag | Source or derivation | What depends on it |
|---|---|---|---|---|
| Takeoff weight | under 249 g | DERIVED | DJI headline spec for the standard battery (recalled) | Weight budget, registration |
| Folded size L×W×H | 148 × 94 × 64 | DERIVED | DJI spec sheet (recalled) | Nothing in the model yet |
| Unfolded size with props L×W×H | 298 × 373 × 101 | DERIVED | DJI spec sheet (recalled) | Motor spacing below |
| `prop_diameter` | 152.4 | ASSUMED | 6.0 in class. Not confirmed for this propeller. | Ring radius, clearance |
| `front_motor` x | +73.0 | DERIVED | (298 − 152.4) / 2. Assumes the prop tips are the front and rear extremes. | Ring centre, joint location |
| `front_motor` y | +110.5 | DERIVED | (373 − 152.4) / 2. Assumes the front prop tips set the overall width. | Ring centre, left/right gap |
| `rear_motor` x | −73.0 | DERIVED | Same as front, mirrored | Ring centre |
| `rear_motor` y | +105.0 | ASSUMED | Rear motors assumed 5.5 mm narrower than front | Ring centre |
| Rear prop plane height vs front | −10.0 | ASSUMED | Rear arms on the Mini series sit lower. Value not measured. | Joint height, lap overlap |
| `motor_keepout_r` | 11.0 | ASSUMED | Motor bell radius plus 1.5 mm | Keep-out check only |
| `prop_cone_up` | 4.0 | ASSUMED | Blade tip rise under thrust | Keep-out cylinder height |

## Arms and clips (all ASSUMED until measured)

| Parameter | Value | Source | Notes |
|---|---|---|---|
| Arm direction | toward airframe centre | ASSUMED | Real arms are angled. Set `front_arm_angle` and `rear_arm_angle`. |
| `front_clip_dist`, `rear_clip_dist` | 22.0 | ASSUMED | Motor axis to clip centre along the arm |
| `front_clip_dz` | −12.0 | ASSUMED | Arm centreline height below the front prop plane |
| `rear_clip_dz` | −12.0 | ASSUMED | Relative to the rear prop plane |
| `front_arm_w` × `front_arm_h` | 12.0 × 8.0 | ASSUMED | Arm cross-section at the clip. Real arms taper and curve. |
| `rear_arm_w` × `rear_arm_h` | 12.0 × 8.0 | ASSUMED | Same |
| `body_box` | x ±75, y ±33, z −45 to +15 | ASSUMED | Fuselage keep-out for checks only |
| `camera_pt` | (80, 0, −25) | ASSUMED | Gimbal lens centre, for checks only |
| `landing_z` | −48.0 | ASSUMED | Table plane relative to the front prop plane |

## Design values

| Parameter | Value | Reason |
|---|---|---|
| `radial_clearance` | 6.0 | Prop tip to ring inner face at rest. Leaves room for ring deflection on impact. Confirm with the deflection test. |
| `prop_vertical_keepout` | 5.0 | No structure within 5 mm below the prop plane inside the disk |
| Ring centreline radius | 83.5 | prop radius + clearance + half ring width |
| `ring_w` × `ring_h` | 2.6 × 3.6 | Upper ring and lower rail section. `variants/light.json` uses 2.2 × 3.0. |
| `upper_dz` / `lower_dz` | +5.0 / −16.0 | Ring heights relative to each prop plane |
| `rib_spacing` / `rib_w` | 28 / 2.4 | Max arc between vertical ribs, also sets bridging length when printing |
| `front_body_gap`, `rear_body_gap` | 100° | Open sector facing the fuselage |
| `strut_d` / `strut_spread` | 3.2 / 62° | Clip-to-rail struts |
| `clip_len` / `clip_wall` | 14.0 / 1.8 | |
| `clip_clear` | 0.25 | Picked from the clearance ladder (0.10 / 0.25 / 0.40) |
| `snap_lip` | 0.8 | Lip overlap under the arm, each side |
| `joint_len` × `joint_w` | 16 × 9 | Lap joint block. Each half is 4.5 mm thick. |
| `joint_clear` | 0.15 | Gap between lap halves |
| `bolt_d` | 2.2 | M2 clearance. 1.75 mm filament pins also fit. |

## Values computed by the model (default parameters)

| Quantity | Value | Where |
|---|---|---|
| Front ring arc | 226° (286.75° to 153°) | `build/build_report.json` |
| Rear ring arc | 224° (31.25° to 254.75°) | same |
| Gap between arc ends at the joint | 0.23 mm (the two arcs meet) | same |
| Lap overlap height for bolts | 14.6 mm | same |
| Min radial prop clearance in model | 5.98 mm, front and rear | `build/validation_report.json` |

## Unresolved

1. Motor positions, prop plane heights and prop diameter. These set every ring position.
2. Arm direction, cross-section, clip location and clip height. These set whether the clip works at all.
3. Real fuselage, camera, sensor and landing-foot positions. The keep-out checks run against placeholders.
4. The reference product's layout (photos not received). The cage layout follows the written description only.
