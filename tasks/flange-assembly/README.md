# Hold and align pipe flanges for horizontal pin insertion

Hold the loose pipe flange against the fixed flange and insert two locating pins horizontally in the top and bottom holes. The loose flange is grasped by the side lug; the demonstration ends with the flange and upper pin still held. It does not claim the pins support the flange after release.

## Relevance and scope

This is a lightweight fixture for evaluating the coordination needed to hold mating parts aligned while inserting locating pins. One arm supports the loose assembly against gravity and controls its orientation while the other approaches and inserts pins along a horizontal axis. Keeping the mating faces and bolt holes aligned during insertion is the assembly skill being evaluated. Flange alignment is a real pipework assembly operation; for example, [Enerpac's flange-alignment instructions](https://literature.enerpac.com/pdf/L4060_c_EN.pdf) describe aligning joint bolt holes. Industrial work can use dedicated alignment tools and lifting equipment; this fixture isolates a hand-supported alignment and insertion operation.

The specified physical counterpart uses printed ABS parts and aluminum-bodied pins to keep the moving assembly light for the bimanual YAM setup. Its 150 mm flange geometry and insertion clearances are retained, but the mass and material are deliberately different from a solid steel flange. The evaluation concerns coordination and geometric fit; it does not demonstrate handling industrial steel pipe loads, gasket compression, bolt tightening or pressure sealing. Evidence reported here is from simulation. The construction below is a specification for a physical reproduction, not a claim that these parts have already been manufactured, weighed or tested.

## Setup

All distances are metres and coordinates are in the calibrated YAM station frame, with `TOP = 0.7475`. The fixed flange is vertical, with its mating face at `(0.3145, 0.06, TOP + 0.17)`, supported by a fixed horizontal pipe 0.10 long and 0.054 in diameter and a 0.05 × 0.08 × 0.13 support. The loose 0.25 kg vertical flange starts at `(0.19, 0.20, TOP + 0.076)`, on three fixed staging blocks. It has a short pipe stub, a square coupling collar and a side lifting lug; these are rigidly attached. Both flanges use the CAD visual and a 0.016-thick perforated collision approximation with 0.016 diameter holes, on a 0.05499 bolt-circle radius.

Two 0.035 kg rigid locating pins lie parallel, with their heads toward negative x, on identical 0.05 × 0.028 × 0.185 supports. Their nominal initial positions are `(0.32, -0.30, TOP + 0.195)` and `(0.32, -0.18, TOP + 0.195)`. Each has a 0.05-long tapered shank of diameter 0.014 and a 0.02-diameter, 0.01-thick head. Their specified construction is aluminum with a small steel mass insert, detailed below. Task-file coordinates, sizes, colors and reset poses are specified in `task.py`; the generated collision assets and `build_collisions.py` are included. The station and all supports are fixed. There is no object jitter; both arms reset to the ready pose.

### Physical materials and construction

Dimensions in this table are millimetres. Use unfilled ABS filament for the printed parts. The [Stratasys ABS-M30 material datasheet](https://www.stratasys.com/contentassets/5460e85552cf40a097148d125e8be84f/mds_fdm_abs-m30_0925a.pdf?v=4aa6a3) gives a representative specific gravity of 1.05; this is a planning value, not a measurement of a finished print. Record the actual filament, slicing settings and finished mass for a physical trial.

| Part | Material and construction | Dimensions and mass requirement |
| --- | --- | --- |
| Loose flange | Printed ABS, with a closed outer skin and sparse interior; open center bore and bolt holes. | 150 outside diameter, 51 overall flange/neck depth, 16 plate thickness, 54 center bore; four 16 holes on a 54.99 bolt-circle radius. The **complete moving assembly**, including stub, collar, lug and joints, must weigh 250 ± 5 g; 250 g is not the flange plate's mass alone. |
| Loose pipe stub and collar | Printed ABS, fused into the flange assembly or rigidly bonded with ABS-compatible two-part epoxy. Keep joints inside the modeled envelope. | Stub: 36 outside diameter × 80 long. Collar: 40 × 40 × 30. Their local positions are defined by `PipeStub` and `GripCollar` in `assets/flange_with_stub.usda`. These are closed lightweight props, without a functional fluid passage. |
| Side lifting lug | Printed ABS connector and gripping block, rigidly joined to the same assembly. | Connector: 20 × 90 × 20; gripping block: 40 × 40 × 40. Local positions are `LiftingLugConnector` and `LiftingLugGrip` in the same asset. Included in the 250 g total. |
| Fixed flange and pipe | Printed ABS flange with the same flange/neck dimensions as the loose part; printed ABS pipe prop, rigidly joined and anchored to the station fixture. | Fixed pipe: 54 outside diameter × 100 long. Fixed bodies have no simulated free-body mass; their weight is carried by the fixture, not the robot. |
| Fixed mounting support | Printed ABS support, secured to the station with steel mounting hardware outside the insertion and gripper paths. | Main block: 50 × 80 × 130. Use an ABS saddle/riser to bridge the 13 mm clearance between the block top and the pipe's lowest surface; the pipe axis remains 170 mm above the table. Mounting joints and this riser are idealized by the fixed-body constraints and are not separate simulated geometry. |
| Three loose-assembly staging blocks | Printed ABS, fixed to the station; no support follows the assembly after pickup. | 60 × 80 × 56, 18 × 80 × 70, and 25 × 35 × 55, at the poses in `task.py`. |
| Two pin supports | Printed ABS, fixed to the station. | Each 50 × 28 × 185, at the poses in `task.py`. |
| Two locating pins | Machined 6061-T6 aluminum shank and head, with a retained 304 stainless-steel insert inside each head to match the simulated mass. | Shank: 14 diameter × 50 long; the first 10 tapers linearly from an 8 diameter tip to 14. Head: 20 diameter × 10 thick. Target finished mass: 35 ± 1 g each. Insert dimensions and calculation follow below. |

For the ABS assembly, use a starting print specification of a 0.4 mm nozzle, 0.2 mm layers, five perimeters (nominal 2 mm walls), 2 mm top/bottom skins and 15% gyroid infill. Keep continuous walls around the bores and lug connections. Boolean-union touching/overlapping component volumes before slicing, or print components separately and bond them rigidly. Adjust internal infill after the slicer's mass estimate and an actual scale measurement to reach 250 ± 5 g **including joints**; do not scale the exterior or change the bores, mating faces or grasp surfaces. These starting settings are not a validated print recipe or a guarantee of the target mass. Before a physical trial, check the finished dimensions and that the lug joints remain rigid while supporting the assembly; physical strength has not been validated here. Fixed ABS supports may use the same wall specification with 20% infill; their finished mass is not a scoring parameter.

The low mass comes from the sparse interior, not just renaming a solid part as ABS. An ideal 150 mm annular plate with a 54 mm bore and four 16 mm holes has volume `pi * (75^2 - 27^2 - 4*8^2) * 16 = 233,232 mm^3`, or about 245 g of solid ABS at 1.05 g/cm³, **before** adding the neck, stub, collar and lug. The 250 g assembly therefore cannot be specified as solid ABS. Its material budget is approximately 238 cm³ of ABS-equivalent volume, reduced by the mass of adhesive or other joints; infill and cavities provide the weight reduction while preserving the external contact geometry.

An entirely aluminum pin with the dimensions above would weigh about 27.7 g, rather than 35 g. The ideal shank plus head volume is 10.273 cm³; [the Aluminum Association's nominal density for 6061](https://www.aluminum.org/sites/default/files/2026-01/Teal%20Sheets%20December%202025.pdf) is 2.70 g/cm³. To reproduce 35 g without changing the outside shape, bore a nominal 15 mm diameter × 7.9 mm deep axial pocket into the **back of the head**, away from the insertion tip, and retain a flush 304 stainless insert with a fitted joint and retaining epoxy. At [304 stainless density 7.9 g/cm³](https://www.alleima.com/en/technical-center/material-datasheets/billets/sanmac-304304l/), replacing that aluminum volume adds approximately 7.3 g, giving about 35.0 g. Verify the finished mass on a scale and trim the internal insert as needed; the 24-sided simulated shaft and joint adhesive introduce small differences from this ideal calculation. An unballasted aluminum pin is a different mass configuration and is not the specified reproduction of the recorded run.

## Scoring

The final state alone is scored. Both flange axes must lie within 7.25 degrees of the horizontal pipe axis; all four corresponding bolt-hole centers must lie within 0.003 in 3D for the alignment credit of 0.4. A pin scores 0.3 when its axis points toward negative x within 7.25 degrees, its center is within 0.0045 of the top or bottom hole axis, and its bounding-box center is between 0.005 toward the head and 0.008 toward the tip from the mating plane. Only one pin counts per target hole; the side holes do not count. Without alignment the score is zero. Maximum is 1.0.

The scorer checks poses, not grasp contacts or velocities; a pin in a target hole can earn credit while held, and transient placements can be scored if present at the end. The demo stops with the second pin held, so it shows insertion but does not demonstrate release or unsupported retention. `python -m pytest -q test_score.py` exercises the geometry and partial-credit cases.

## Reproduction

Use Isaac Lab Arena commit `c7edf3a9d5`, Inspect Robots 0.60.0, Isaac Lab Arena 0.3.0 and Isaac Lab 17.0.2. Install this directory as an editable package in the YAM environment. The extensions register `yam_flange` and `yam_flange_scripted`. The scripted demo uses measured simulator object poses as feedback for alignment; those privileged poses are not provided to Dream-1.

```sh
inspect-robots run --embodiment yam_flange --policy yam_flange_scripted \
  -E task="$PWD/task.py" -E object_jitter=0 -E device=cpu \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hold the loose pipe flange against the fixed flange and insert two pins horizontally in the top and bottom holes." \
  --scorer task_score --max-steps 2700 --seed 0 --store-frames --log-dir ./demo-run
```

For Dream-1, substitute `--policy dream` and a separate log directory; retain the same task, instruction, seed, control rate, cameras and run limit. The scripted demonstration reported 1.0 at 2,231 steps in 226.7 seconds, environment revision `178813d85202`. Dream-1 reported 0.0 at 2,700 steps in 265.6 seconds, with the same environment revision. After both recordings, only the task file’s module docstring and one comment were updated; its resulting SHA-256 is `97b1eb57ed9ad668ad2720501a8608c995280b4025854bfeefa40fb5293a6071`, with no behavior change.

### Simulation differences and recorded evidence

The simulation assigns 0.250 kg to the complete loose assembly and 0.035 kg to each pin explicitly; it does not calculate their masses from a steel density or model printed infill and the pin inserts. The CAD's gray metallic shader (including the asset's internal `Steel` material name) and the pins' gold display color are retained visual styling, not physical material specifications. The physical material specification is the ABS/aluminum construction above. The internal mass distributions of sparse prints and head inserts are not modeled; agreement with the physical parts' inertia has not been validated.

The simulation uses simplified 16 mm perforated colliders, a rigid pipe stub and side lifting lug, and approximate contact friction. It does not model ABS layer structure, flexure or joint failure, aluminum surface finish, or measured ABS/aluminum friction. The fixed pipe is a solid cylindrical collider; the fixed fixture's mounting hardware and saddle/riser are omitted. It also omits gasket, threads, retaining clips, preload and fluid seal. It does not establish that the two pins retain the pipe when unsupported.

This materials revision changes only this README and the descriptive `context`, `setup` and `fidelity` fields in `task.json`. Geometry, assigned masses, contact parameters, controller, instruction, scoring and run limit remain the ones used for the reported demo and Dream-1 recordings; the existing evidence therefore describes the same simulated task. It does not validate the proposed physical construction. Any later change to the simulated task requires corresponding updated run evidence.

The original CAD source is [Innerscene pipe flange](https://www.innerscene.com/tools/library/3d-parts/pipe-flange-original-mep-cad-2a6f5077), [CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/).

The demonstration video places the station and right-wrist cameras from the same recorded run side by side; the Dream video uses the station camera.
