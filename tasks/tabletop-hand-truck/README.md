# Tip, roll and park a tabletop hand truck

Operate a deliberately miniature, loaded hand-truck replica on the supplied YAM station. Raise the nose by tipping about the passive wheels, roll toward the green mark, then stand and release it with the wood load retained. This tests a change between standing and wheel-supported transport. It does not establish full-size freight-handling competence or benchmark acceptance of miniaturization.

## Physical setup

The stock 1.10 × 1.30 m station table, top at Z=0.7475 m, arm bases, room and three calibrated 640×360 cameras are unchanged. Coordinates are metres in the supplied station frame. The initial axle midpoint is (0.460, 0.100, 0.77875); the frame has +90° yaw, with its nose toward +X. The cart's reference body is 300 mm tall, overall sleeve height 309 mm, width 115.625 mm and length 112.5 mm. Wheels are Ø62.5 × 12.5 mm with painted index stripes for video measurement; paint has no collision and its negligible mass is omitted. The cart mass is 172.281 g including the additional foam grip sleeve. The left jaws start physically around the sleeve; there is no attachment constraint. The right arm starts in the standard ready pose and is free.

A loose pine-like wood block is 60 × 35 × 100 mm, 105 g at the assumed 500 kg/m³ density. Its centre is (0, −0.0225, 0.052) in frame coordinates, with the same orientation as the cart. The green 40 × 200 × 1 mm visual mark is centred at (0.310, 0.100, 0.748); it has no collision. Reset replaces the cart and load and restores the starting robot joints. Nothing is consumed.

[Physical design](PHYSICAL_DESIGN.md), [dimension drawing](physical_design.svg), and [mass/inertia ledger](physical_design.json) distinguish the Magliner product reference, scaled values and independent build assumptions. `build_assets.py` regenerates the original geometry. `ASSETS.md` records provenance.

## Rules

The episode uses its final score, and invalid outcomes erase partial credit. Quarter credit requires a tilt >8° and the nose point >8 mm above the tabletop. Half credit additionally requires each wheel's net X travel and table-supported net travel >125 mm, signed wheel rotation >3.5 rad, and |rotation × 31.25 mm − travel| <25 mm. The axle must finish at 0.285<X<0.335 and |Y−0.100|<0.040 m. The nominal requested roll is 150 mm.

Full credit requires that half-credit state, tilt <2°, frame and load translation speeds <10 mm/s, no robot/cart contact for >0.5 s, both TCPs >120 mm from the handle, and all these finish conditions for 20 consecutive control steps (1 s). The loose load must remain within 25 mm per axis and 15° of its initial pose relative to the frame throughout. Robot/load contact, wheel centres above their normal height by >20 mm or below it by >8 mm, or both wheels lacking reported table contact for >0.25 s terminates with zero. A wheel's supported travel is counted only within four physics ticks of its own table-contact report. Each wheel must satisfy the supported-travel requirement; continuous support from each individual wheel is not required. Carrying the assembly to the destination cannot earn half or full credit.

## Run

Use an installed Isaac Sim / Isaac Lab / YAM Arena environment with RTX rendering and Inspect Robots. Validated runtime: Python 3.12.12, Isaac Sim 6.1.0.0, Isaac Lab 17.0.2, Inspect Robots 0.60.0. Run from this directory; `run_local.py` registers the task directly, so editable installation is unnecessary. CPU physics uses an RTX GPU for cameras. Physics is 480 Hz, control 20 Hz, 480 action limit (24 simulated seconds), fixed seed 0, no object jitter.

```sh
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export TASK_TELEMETRY_DIR="$PWD/logs/demo-native"
python -m unittest discover -v
python run_local.py run --embodiment yam_tabletop_cart --policy cart_scripted --instruction "Tip the compact loaded trolley onto its wheels, roll it to the near mark, stand and release it." --scorer task_score --max-steps 480 --seed 0 --store-frames --log-dir logs/demo --no-prompt --grader none --no-rerun
```

For Dream-1, set its credentials privately as required by the installed client, change policy to `dream -P model=dreamlabs/dream-1-beta`, and choose separate log and telemetry directories. All other settings stay the same. The scripted feasibility controller uses native pose feedback and joint-space IK, not model-only camera observations. It sends ordinary 14-value robot actions and never writes cart, load or wheel motion after reset. `TASK_TELEMETRY_DIR` only records diagnostics; it does not change physics. `python probe.py <new-output-directory>` also records native camera videos and contact traces for diagnosis.

## Fidelity

Geometry, masses, compound centre of mass and inertias are explicitly authored and audited in the running simulator. Frame material/dimensions are build assumptions with a source-scaled mass budget; the foam sleeve is additional mass. Hollow tubes use solid outer collision envelopes. Tires use 64-sided rigid convex cylinders and uniform-cylinder inertia, not deformable cushion rubber. Passive wheel bearings have zero target velocity/position, zero stiffness and tiny dissipative damping; they cannot propel the cart. Estimated metal friction is 0.5/0.4 and tire friction 0.9/0.7; restitution is zero. Small friction correlation/offset settings resolve the miniature contacts. Friction, sleeve compression, structural compliance and bearings need physical measurement; no hardware qualification is claimed.

## Validated simulation results

The final Inspect runs share environment revision `30e388c9a666e1570165dac34f888542c875ece1e088f67d12edadaa3cb50edb`. Scripted demo: **1.0**, 248 actions, 48.684 s wall time. Dream-1 beta: **0.0**, 157 actions, 45.449 s wall time, terminated by the invalid-outcome rules. Both use the same fixed layout, native cameras, default CLI bounds/delta approvals and 480-step cap. These are measured simulator results, not physical robot trials.

The exact-revision native audit verifies 172.281 g cart and 105 g load, compound inertias, original station bounds and camera calibration. In the demo the two wheels travel 164.14/160.24 mm with 5.359/5.246 rad signed rotation, retain the load, and finish upright, released and stable for one second. Model failure does not change the demonstrated feasibility result. The full six native H.264 views, initial setup images, Inspect records and native traces are supplied separately from this public source. No media ID has been assigned locally.
