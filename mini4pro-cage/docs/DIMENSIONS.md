# Dimension table (v0.3)

All values live in `cad/mini4pro_cage.py` (class `Params`) and can be overridden with a JSON file.

## Source categories

| Tag | Meaning |
|---|---|
| **VERIFIED** | Measured on the real drone. **Nothing is in this category yet.** |
| **PHOTO** | Proportion measured in pixels on the reference photos, then scaled to millimetres by a DERIVED value. Accuracy about ±5%. Product renders are not guaranteed orthographic. |
| **DERIVED** | Computed from a recalled DJI figure (dji.com was unreachable from the build environment). |
| **ASSUMED** | Placeholder. Must be measured. |
| **DESIGN** | Engineering choice. Safe to tune. |

## How the scale was set

Reference photo 3 is a near-orthographic view **from below** of both cages. Measured on the left cage, in pixels:

| Feature | Pixels | Ratio to top loop radius |
|---|---|---|
| Top loop (figure-8) radius | 123 | 1.00 |
| Top ring radius (thin circle, seen through from below) | 86 | 0.70 |
| Hub ring radius (around the clip) | 42 | 0.34 |
| Outer rail radius | 178 | 1.45 |
| Top loop half-width at the waist | 70 | 0.57 |
| Outer rail half-width at the waist | 129 | 1.05 |
| Front-to-rear motor spacing | 280 | 2.28 |
| Overall cage length | 646 | 5.25 |

Assumption to close the scale: the propeller tip sits about 9 mm inside the side wall at propeller height. Combined with DJI's published unfolded length (298 mm with propellers, recalled), the ratios solve to:

- top loop radius **63 mm**
- motor spacing **144 mm**
- propeller diameter **154 mm** (about 6.0 in)

Cross-checks: the predicted overall cage length is 331 mm, against 5.25 × 63 = 331 mm from the photo. The 144 mm spacing also agrees with an independent estimate from DJI's length and 6-inch props (146 mm). **The two estimates share the DJI figure, so they are not independent proof.** Measure M1 to M4.

## Drone

| Parameter | Value (mm) | Tag | What depends on it |
|---|---|---|---|
| `prop_diameter` | 154 | DERIVED | Every clearance |
| `front_motor` | (72, 109.5, 0) | DERIVED | Cage position |
| `rear_motor` | (−72, 109.5, 0) | DERIVED, z ASSUMED | Rear prop plane assumed level with front |
| `prop_cone_up` | 4 | ASSUMED | Lid height check |
| `motor_keepout_r` | 11 | ASSUMED | Check only |
| Arm direction | toward airframe centre | ASSUMED | Clip orientation. Photo clip tabs point the same way. |
| `*_clip_dist` | 18 | ASSUMED | Clip position along the arm |
| `*_clip_dz` | −12 | ASSUMED | Arm centre height |
| `*_arm_w` × `*_arm_h` | 12 × 8 | ASSUMED | Clip cavity |
| `body_box`, `camera_pt`, `landing_z` | see file | ASSUMED | Checks only |

## Cage

| Parameter | Value | Tag |
|---|---|---|
| `loop_r` top loop | 63 | PHOTO + DERIVED |
| `top_ring_r` | 44 | PHOTO (0.70) |
| `hub_ring_r` | 21.5 | PHOTO (0.34) |
| `rail_r` lower outer rail | 91.5 | PHOTO (1.45) |
| `rail_upper_inset` | 6 | DESIGN (FDM adaptation, see below) |
| `loop_waist_w` | 36 | PHOTO (0.57) |
| `rail_waist_w` | 66 | PHOTO (1.05) |
| `cut_theta` C-cut end | 60° | PHOTO |
| `spoke_thetas` | 60°, 121°, 180° | PHOTO (angles from the inboard direction toward the lobe end) |
| `short_spoke_theta` | −25° | PHOTO |
| `z_top` / `z_bot` | +12 / −24 | ASSUMED (photos give no reliable heights) |
| `lid_h` / `bot_h` | 2.2 / 2.4 | DESIGN |
| Member widths | 2.0 to 3.0 | DESIGN |
| `rib_spacing` | 55 (13 ribs per side) | PHOTO (approximate count) |
| `waist_feature_offset` | 3 | DESIGN (photo: 0) |
| `waist_leg_base_w` / `waist_leg_top_w` | 56 / 52 | DESIGN (photo: about 36) |
| `split_offset` | 7 | DESIGN |

## Deviations from the photographed product

1. **Side profile.** In the photos the top loop is the highest point and the outer rails sit lower and further out, joined by outward-sloping ribs. A slope that shallow (about 26°) cannot print on FDM without supports. v0.3 puts the upper outer rail in the flat lid and makes the ribs near-vertical. The plan view is unchanged.
2. **Waist leg and bar.** In the photo the inboard waist leg drops straight down from the top loop pinch. With 154 mm props, that pinch is only 80.5 mm from each motor axis, inside the 82.7 mm clearance zone. v0.3 moves the leg 16 mm inboard on a short lid link, and the bottom waist bar runs 20 mm past the inboard rail to meet it. Once the props are measured, set `waist_leg_*` to 36 to match the photo if clearance allows.
3. **Clips.** The photo clips cup the motor base, with a tab toward the arm. Without motor-base dimensions, v0.3 uses a snap C-clip on the arm 18 mm from the motor axis, carried on the hub ring.
4. **Splits.** The product is one molded piece per side. v0.3 is four printed parts per side: lid and body, each split at the waist with dovetails.

## Unresolved

1. Prop diameter, motor positions and prop plane heights (M1 to M5).
2. Arm section, clip location and motor-base shape (M6 to M8).
3. Cage heights (`z_top`, `z_bot`). A side photo with a ruler would settle them.
4. Fuselage, camera and landing-foot positions for the checks (M9, M10).
