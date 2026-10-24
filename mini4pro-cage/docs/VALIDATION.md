# Validation

## What was actually tested

All results come from `cad/validate.py`, run on the STLs in this folder. Full output: `build/validation_report.json`.

### Mesh integrity

| File | Watertight | Winding consistent | Bodies | Volume (cm³) | PETG mass (g) |
|---|---|---|---|---|---|
| front_clip_clearance_ladder | yes | yes | 3 (3 separate clips, by design) | 2.57 | 3.3 |
| rear_clip_clearance_ladder | yes | yes | 3 (by design) | 2.57 | 3.3 |
| front_mount_fit_test | yes | yes | 1 | 3.48 | 4.4 |
| rear_mount_fit_test | yes | yes | 1 | 3.49 | 4.4 |
| PROVISIONAL left/right front module | yes | yes | 1 | 10.96 | 13.9 |
| PROVISIONAL left/right rear module | yes | yes | 1 | 11.28 | 14.3 |

**Full cage, both sides:** 44.5 cm³, about 55 g in PLA or 57 g in PETG, plus about 2 g of hardware. `variants/light.json` brings it to about 42 g in PETG.

### Defects found and fixed during this work

1. The joint block and ring ends intruded into the propeller clearance zone (0.4 mm clearance left). Fixed by moving the block outboard and subtracting a propeller keep-out cylinder from every part.
2. Two rib faces lay tangent to the ring cylinder, and the STL mesher dropped them (open mesh, wrong volume). Fixed by setting ribs 0.4 mm into the ring wall and 0.6 mm proud of it.
3. The export check left cached per-face meshes, which cracked the STL along shared edges. Fixed by clearing the cache before export. Export now refuses to write a file if any face fails to mesh.

### Geometry checks against the parameters

These checks are only as good as the ASSUMED parameters in `DIMENSIONS.md`.

| Check | Result |
|---|---|
| Min radial clearance, prop tip to cage, front / rear | 5.98 mm / 5.98 mm (target 6.0) |
| Structure inside either prop disk near the prop plane | none |
| Structure inside the motor keep-out | none |
| Structure inside the fuselage keep-out box | none (min gap 18.6 mm) |
| Front module vs rear module overlap, assembled | 0.012 mm³: tessellation noise across the 0.15 mm joint gap |
| Left vs right cage gap at the nose | 44.6 mm |
| Lowest point vs assumed table plane | 20 mm above. The cage is not a landing gear. |
| Distance to assumed camera lens | 33.6 mm |
| Structure ahead of the lens within ±45° horizontally | **yes**. The front rings may show in footage. Not fixable by moving the front gap alone. Test on the drone. |

### Not tested

Real fit, strength, deflection, crash behaviour, vibration, compass effect, obstacle sensing, flight behaviour. A watertight mesh says nothing about any of these.

## Physical validation plan

Run the steps in order. Stop at the first failure, fix the parameter, regenerate, and repeat that step.

| Step | Test | Pass criteria |
|---|---|---|
| 1 | Enter M1 to M10 into `variants/measured.json`. Regenerate, then rerun `validate.py`. | All checks clean |
| 2 | Print both clip ladders in the final material. Try each clip on its arm. | One clip snaps on with firm thumb pressure, does not rattle or slide, comes off by hand, leaves no marks. Set `clip_clear` to that clip's value. |
| 3 | Print both fit tests. Snap each onto its arm. | Clip seats fully. Nothing touches the motor bell, vents, wires or landing foot. |
| 4 | With the fit test on, turn the prop by hand. Slide a 5 mm drill bit between prop tip and ring at both ring segments. | The bit passes. The gap is at least 5 mm. |
| 5 | Print and assemble the full cage. Weigh the drone with the cage on a 0.1 g kitchen scale. | Record the weight. Over 250 g changes the FAA rules (see below). |
| 6 | Deflection: props off, motors off. Push the outermost ring point inward with a luggage scale to 5 N (about 0.5 kg). Measure the inward movement. | Movement is less than 5 mm (clearance minus 1 mm) |
| 7 | Retention: hold the drone and give the cage a firm 10 N tug in each direction. | Clips stay seated |
| 8 | Power on. Check DJI Fly for compass or IMU warnings. Tilt the gimbal through its full range in live view. | No new warnings. Note whether the rings show in frame. |
| 9 | Hover at 2 m in an open area, no wind, 30 seconds. Land and inspect. | No contact marks on the props or rings, no new vibration, no obstacle-sensing alerts caused by the cage |
| 10 | Short gentle flight. Compare battery use against a flight without the cage. | Acceptable handling and flight time |

After any impact, check the clips and joint for cracks before the next flight.

## Weight and regulations

The Mini 4 Pro is sold under 249 g with the standard battery. This cage adds about 42 to 57 g. In the US, a recreational drone at 250 g or more must be registered with the FAA and must meet Remote ID rules. Check the current FAA rules before flying with the cage installed.
