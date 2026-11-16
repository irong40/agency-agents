# Validation (v0.3)

Results from `cad/validate.py`, run on the files in this folder. Full output: `build/validation_report.json`.

## Mesh integrity

| File | Watertight | Winding | Bodies | PETG mass (g) |
|---|---|---|---|---|
| Clip clearance ladders (×2) | yes | yes | 3 (three separate clips, by design) | 2.3 each |
| Mount fit tests (×2) | yes | yes | 1 | 4.8 each |
| Body front (L/R) | yes | yes | 1 | 9.7 each |
| Body rear (L/R) | yes | yes | 1 | 8.2 each |
| Lid front (L/R) | yes | yes | 1 | 8.5 each |
| Lid rear (L/R) | yes | yes | 1 | 8.2 each |

## Model checks against the parameters

| Check | Result |
|---|---|
| Material removed by the propeller keep-out, every part | 0 mm³ (no member enters the prop path) |
| Material removed by the arm keep-out, every part | 0 mm³ |
| Interference between any two parts, assembled | 0 mm³ |
| Min radial clearance, prop tip to cage, at prop height | 5.74 mm (keep-out set at 5.7) |
| Lowest lid point above the prop plane, inside the disk | 10.9 mm |
| Highest bottom-level point below the prop plane, inside the disk | 5.95 mm (the clip top, under the blade root) |
| Fuselage keep-out box | no intrusion, 12 mm gap |
| Left to right cage gap | 90 mm |
| Structure ahead of the camera within ±45° | yes. The front lobes will likely show in footage when the gimbal tilts up. |
| Lowest point vs assumed table plane | 22.8 mm above. The drone still lands on its own feet unless the real `landing_z` differs. |

## Defects found and fixed during v0.3

1. The inboard waist leg and its boss entered the front propeller's clearance zone. Caught by the keep-out trim report and fixed by moving the leg joint onto a short lid link.
2. The arm keep-out cut the clips. Fixed by adding clips after the subtraction.
3. Non-manifold STL edges from coincident primitives and pinch points. Fixed at the root: flat levels are now 2D outlines extruded once, and every 3D foot is 0.4 mm narrower than the outline it sits in.
4. The dovetail tongue overlapped the waist rib's pad. Fixed by moving the waist features 3 mm forward and the split 1 mm back.

## Not tested

Fit on the real drone, strength, deflection, crash behaviour, vibration, compass and obstacle-sensing effects, flight. A watertight mesh says nothing about these.

## Physical validation plan

| Step | Test | Pass |
|---|---|---|
| 1 | Measure M1 to M10, regenerate with `--json variants/measured.json`, rerun `validate.py` | All keep-out trims 0 |
| 2 | Clip ladders in final material | One clip snaps on by hand, holds without rattle, comes off by hand. Set `clip_clear`. |
| 3 | Fit tests on each arm | Clip seats. Hub ring clears the motor base. Turn the prop: the tip passes the three rib stubs with at least 5 mm (drill-bit gauge). |
| 4 | Full cage, assembled | All pegs and dovetails seat. Weigh the drone with cages (0.1 g scale). |
| 5 | Deflection: props off. Push the outer rail inward at mid-lobe with a luggage scale to 5 N | Inward movement under 5 mm |
| 6 | Retention: 10 N tug on each cage | Clips stay on |
| 7 | Power on, DJI Fly | No compass or IMU warnings. Check gimbal live view for the rings. Check obstacle-sensing alerts. |
| 8 | 30 s hover at 2 m, open area, calm air | No contact marks, no new vibration |
| 9 | Short gentle flight | Handling and battery use acceptable |

## Weight and regulations

The Mini 4 Pro is sold under 249 g with the standard battery. These cages add about 52 to 69 g. In the US, a recreational drone at 250 g or more must be registered with the FAA and must meet Remote ID rules. Check current FAA rules before flying with the cages installed.
