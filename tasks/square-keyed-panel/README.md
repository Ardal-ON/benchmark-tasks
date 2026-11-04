# Draft: four-piece square puzzle for an assembly workbench

Four separate 50 × 50 × 24 mm stylized manufactured inserts, each one rigid, must be fitted into
a 100 × 100 mm square at the actual centre of the simulated table. Large T-shaped tabs project
12 mm and widen to 20 mm (40% of a piece's body width). Four different offsets distinguish the
four internal seams, with 2 mm nominal clearance. The four pieces have tabs or recesses on two
sides. Four small orange marks indicate the target corners; they are not a supporting fixture.

The pieces begin at four scattered positions with different yaw angles. The simulation's shapes,
friction and clearance are approximate; they are not a validated manufacturing model. This is a
draft; the task board requires team sign-in for novelty review. A successful full assembly has not
yet been demonstrated. The scripted controller is under active physical validation; it is not yet a full-score demo.

The original open-loop scripted plan is not yet successful. The task, geometry, scorer and tests are on
a dedicated task branch based on the current upstream `main`; no media ID or model-run score has
been recorded. See `task.json` for the precise geometry, fixed initial configuration and scoring.
The tabletop target is fixed at `(0.36, 0.0)` metres, the station table's center. The corner marks
are visual indicators rather than physical assembly guides.

The geometry test enumerates all piece permutations and right-angle rotations on a 1 mm grid. It
finds one relative assembly, with the only other solutions being rotations of the whole square.
This is not a proof over arbitrary 3-D orientations, deformations or strategies.

## Procedural geometry and reproduction

The pieces are created at runtime from Isaac Lab cuboid and physics APIs. No authored `.usd` or
`.usda` files are required for the puzzle pieces. Isaac Sim still uses its normal in-memory USD
stage internally, and the robot and station continue to use the framework's existing assets.
The stock task-file API accepts single primitives or registered assets, not compound primitive
pieces. `procedural.py` therefore installs a task-scoped spawner adapter around the pinned YAM
Arena implementation; it restores the stock object factory after building this task. The adapter
uses framework internals and must be revalidated if the pinned framework changes.

The fixed-seed L40S scripted run of this procedural task scored **0.25/1** and ended by
`policy_stop` after 885 steps; it took approximately 188 seconds including initialization.
It is not a full-score demonstration. The dimension-corrected procedural preassembled
diagnostic settled and scored 1.0, but begins at the goal and does not demonstrate the task.

`square_demo.py` registers a separate `square_scripted` policy. It reads simulator piece
poses for Cartesian grasp correction and commands only robot joint targets. This privileged
feedback is for the feasibility demonstration; it is not given to a model policy. Piece poses
are cached by the scorer without changing scoring equations. Original motion and feedback
trial 04 reached 0.25; feedback trials 01, 02, 03, 05 and 06 reached 0.0. These are failed
assembly demonstrations, not evidence of full feasibility. Completed trial videos have been
collected for inspection; trial 06's final transfer was interrupted by an SSH outage.
A shorter first-piece diagnostic is under validation to isolate release drift before another
full assembly trial. The current candidate uses a shallow grasp with bounded pose corrections during alignment and a pause before opening the jaws.
A demo calibration file compensates measured jaw-opening offsets with Cartesian
position and yaw adjustments. These are controller calibration values; the scorer is unchanged.
Two isolated left-arm placements reached approximately 0.1 and 0.2 mm final position error during
calibration. A later gentler-grip diagnostic placed the first piece within 0.83 mm without offsets.
These isolated checks do not prove a full assembly; validation with all neighbors remains required.
Piece collider contact offsets are 0.5 mm with zero rest offset; position/velocity solver
iterations are 32/4. Robot colliders and actuator settings are unchanged. The carrying height is
70 mm above the tabletop. The current candidate uses a -20 mm finger-joint hold target rather than commanding fully
closed fingers; it grasps the upper 4 mm of the body. The robot lowers the held part to a 6 mm
bottom clearance before
releasing it, allowing the fingers to clear assembled neighbors. These candidate settings
require further physical validation; the current candidate has not demonstrated full score.
All four pieces individually reached submillimeter final placement in isolated diagnostics,
but the full trial with those independent offsets scored 0.0 due to insertion interference.
The current controller instead fits later pieces to the first piece's measured rigid pose,
adds an alignment hold above existing parts, and maintains horizontal feedback during descent.
It stops a failed scripted insertion if tilt exceeds 15 degrees. These controller rules do
not change task scoring. A successful full assembly remains unproven.
Use `-P trace_path=/absolute/path/poses.jsonl` to record measured piece poses every ten actions.

From a GPU allocation, set `SIM_ROOT` to the installed Robocurve runtime root, install this task's
editable package in the Arena environment (`uv pip install -e . --no-deps`), then:

```bash
"$SIM_ROOT/cluster/run-arena.sh" inspect-robots run --embodiment yam_arena --policy square_scripted \
  -E task="$PWD/task.py" -E top_cam=demo -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 --instruction 'assemble the four keyed pieces into a square at the center of the table' \
  --scorer task_score --max-steps 2400 --seed 0 --store-frames --log-dir /path/to/run-output
```

Tested with `yam-isaac-lab-arena` commit `0059d02a1a91bc6834c1d0846022891df0c78730`,
Isaac Lab Arena 0.3.1, Isaac Sim 6 and `inspect-robots` 0.60.0. The tested renderer requires
the node's NVIDIA NGX library to be visible in the container. Run offline checks with
`python -B -m unittest -v test_task` in the Arena environment. `geometry.py` defines the pieces;
`procedural.py` creates their primitive colliders and rigid bodies at runtime. Model-run evidence,
media upload, a full-score demo and the board's novelty check remain outstanding.

The current demonstration advances descent only when measured horizontal error is below 0.4 mm,
orientation error below 0.5 degrees, and height tracking error below 1 mm. The second arm waits
for the first arm to finish parking. A two-minute run limit allows these precision holds.
Placement uses 0.5 mm of body-to-body clearance within the unchanged 2 mm seam tolerance.
The parts and scorer are unchanged; full-score feasibility still requires a completed trial.

The current grasp closes along the parts' x direction, at body-local y=+17 mm for the left
arm and -22 mm for the right arm, with respective gripper yaw -90/+90 degrees. Offline IK
checks cover each pickup/placement and ±5-degree hover corrections; these grasp choices
avoid the joint-4 saturation observed in the previous front-edge grasp. Rotation integral
bias is capped at 6 degrees. This candidate remains under physical validation.

For the final component, `closure.py` fits the projected box-collider footprints against the
three measured neighbors and retains only targets satisfying the original scoring tolerances.
It reconstructs object roots from Arena's transformed local-bounds centers. The final descent
uses tighter feedback holds; this fit selects robot targets and never moves object poses.
The previous side-grip trial seated three pieces and scored 0.75; closure remains unproven.
