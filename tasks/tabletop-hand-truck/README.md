# Tip, pull and park a full-size compact hand truck

Tip the empty hand truck onto its passive wheels, pull it **150 mm toward the arms**, stand it upright and release the handle and frame grasps. The cart stands on the floor in front of the arms. A smaller U-shaped robot platform supports both unchanged calibrated mounts; the cameras and their stand remain fixed. Both grasps begin acquired: one on the top handle and one on the upper frame brace. All cart motion comes from native contacts and ordinary 14 joint/gripper commands.

The reference is an actual **Wesco Mini Mover 220646**, fully deployed at **1:1 operating size**: 1041.4 mm high, 393.7 mm wide and 444.5 mm deep, with a 381 × 317.5 mm toe and 127 × 31.75 mm wheels. The current manufacturer listing says **9 lb / 4.08233133 kg**; its linked drawing and a 2019 catalog say **7.25 lb**. The current value is chosen and the disagreement is disclosed. The listed 24-inch height is folded storage height, not the operating configuration simulated here.

[PHYSICAL_DESIGN.md](PHYSICAL_DESIGN.md), [physical_design.json](physical_design.json) and the [dimension drawing](physical_design.svg) separate published product values from original engineering estimates. Only the cart's total weight is sourced; component masses and inertia are estimated. The 63.5 mm axle offset is inferred. The deployed telescope/folding mechanism is rigidized. The cart is empty, with its straps stored. Hardware behavior remains untested.

## Platform and reset

The user-authorized robot platform is **390 × 860 × 60 mm**, centered at (-0.125, 0, 0.7175) m, with top Z = 0.7475 m. Its center cutout leaves a rear bridge and full pads under the two robot mounts, giving the lower arm clearance. Its plan bounds are X = [-0.320, 0.070] m and Y = [-0.430, 0.430] m. Both complete mounting footprints and the camera stand foot fit inside it. Use a rigid anchored platform and the original calibrated mount arrangement. Robot bases remain at (0, ±0.305, 0.750) m with stock actuator settings. All three calibrated 640 × 360 cameras, lighting and floor remain unchanged.

The floor is Z = -0.0125 m. The nominal pre-grasp axle is (0.500, 0, 0.051) m with the toe toward +X. The left grasp target is local (0, +9, 1027.9) mm on the handle; the right is (0, -15, 562.1) mm on the upper brace. These are chosen proxy locations. The TCP advances 8 mm and 12 mm along each approach to place the bars inside the finger pads. Jaw closing axes are perpendicular to the crossbars. A two-second native settling stage acquires the grasps; actual reset poses are recorded and may differ from nominal. The floor parking mark is centered at axle X = 0.350 m. No chocks or straps are engaged. Nothing is consumed.

## Scoring

All progress comes from observed body poses, native contacts and passive wheel rotation:

- **0.25:** tilt exceeds 8° while the front toe is more than 8 mm above the floor.
- **0.5:** each wheel has net backward travel above 125 mm and paired supported backward travel above 125 mm. Rotation and displacement are counted over the same physics intervals when both wheels have a recent qualifying floor contact: separation ≤0.5 mm, normal-impulse magnitude >1e-7 Ns and age ≤4 physics ticks. Each supported rotation exceeds 0.8 rad and `abs(radius * supported_rotation - supported_travel)` is below 25 mm. The axle is in X = (0.315, 0.385) m and |Y| <40 mm.
- **1.0:** additionally stand within 2° of upright, keep frame translation below 10 mm/s and rotation below 2°/s, avoid robot/cart contact for more than 0.5 s, withdraw both TCPs more than 120 mm from either handle grip point, and hold this finish for one second.

The final score becomes zero if either wheel rises more than 20 mm or falls more than 8 mm from its nominal center height, both wheels lack floor support for more than0.25s while the cart is not upright on a qualifying nose/toe-rib floor contact, or the cart touches the robot platform, camera housing or camera stand. Earlier partial credit does not survive a later fault. The episode limit is 600 actions at 20 Hz (30 simulated seconds), seed 0.

## Physical evaluation procedure

For a physical trial, place a ruler along the backward (-X) route and a visible radial index mark on each wheel. Record both tires and the toe with fixed side views, plus an overhead view of the destination. These are evaluator measurement cameras; the robot still uses its original three calibrated views. Use slow-motion video (ideally 240 fps) to check simultaneous visible wheel lift lasting more than 0.25 s, and measure each wheel center's signed displacement between the initial and final frames. Count signed index-mark turns in the same direction, multiply by the 63.5 mm radius, and compare the arc length with travel. Sum displacement only over intervals with observed wheel/floor contact to check supported travel. Use a frame reference line and plumb line for tilt.

During the released finish, track frame marks at 20 Hz for one second against the ruler, check the 10 mm/s limit, and measure the 120 mm TCP clearances from both recorded grasp points. Use close views with sufficient scale and frame rate; if a threshold cannot be resolved, the physical result is inconclusive. The matched net arc check rejects sustained dragging, but transient slip can cancel. Native impulse filtering and its four-physics-tick freshness allowance (8.33 ms at 480 Hz) are simulator approximations. Video can establish visible separation and the longer support-loss interval, but does not measure normal force or prove exact equivalence to that force/freshness rule. No physical score is reported here.

## Run and evidence

The scripted oracle uses the supplied URDF masses and centers of mass for gravity offsets through ordinary joint targets, plus read-only simulator cart pose feedback for phase-boundary grasp calibration and axle-level feedback; Dream receives the standard images and joint state. Both use the same physical environment and default action approvals. The oracle retains measured joint-servo preload while planning from the acquired cart pose; its IK projects requested tool orientations into the joint limits, and diagnostics record the residuals and actual motion. The oracle never writes cart or wheel poses after reset and does not actuate wheels.

```bash
python run_local.py run --embodiment yam_tabletop_cart --policy cart_scripted \
  --instruction 'Using the acquired handle and frame grasps, tip the empty cart, pull it toward the floor mark, stand and release it.' \
  --scorer task_score --max-steps 600 --seed 0 --store-frames \
  --log-dir /path/to/results --no-prompt --grader none --no-rerun
```

The registered embodiment name is retained for compatibility. Replace `--policy cart_scripted` with `--policy dream -P model=dreamlabs/dream-1-beta` for the model baseline, using authorized credentials outside the source tree. `probe.py` uses the CLI's clamp and joint-delta approvals. `TASK_TELEMETRY_DIR` optionally records native poses, contacts, masses, inertia, actual wheel drives, platform bounds and robot mounting footprints. CPU rules/design checks are in `test_scoring.py`, `test_physical_design.py` and `test_gravity.py`.

Fresh CLI validation: scripted demo **1.0**, 424 actions / 21.20 simulated seconds; Dream-1 beta **0.0**, 517 actions / 25.85 simulated seconds. Both use seed 0, the same 600-step limit, default action approvals and environment revision `625f5138d1d1b5c2b0cccd1fe70c42c0179bdaecfbcc4a1247eebbbe4da9371b`. Native audits verify masses/inertias, passive wheels, stock motor limits, calibrated rig and supported robot mounting soles. The six native recordings contain 425 demo frames and 518 model frames per camera at 20 fps. Earlier tabletop prototypes have separate evidence.
