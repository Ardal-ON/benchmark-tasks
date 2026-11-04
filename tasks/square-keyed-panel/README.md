# Draft: four-piece square puzzle for an assembly workbench

Four separate 50 × 50 × 24 mm stylized manufactured inserts, each one rigid, must be fitted into
a 100 × 100 mm square at the actual centre of the simulated table. Large T-shaped tabs project
12 mm and widen to 20 mm (40% of a piece's body width). Four different offsets distinguish the
four internal seams, with 2 mm nominal clearance. The four pieces have tabs or recesses on two
sides. Four small orange marks indicate the target corners; they are not a supporting fixture.

The pieces begin at four scattered positions with different yaw angles. The simulation's shapes,
friction and clearance are approximate; they are not a validated manufacturing model. This is a
draft; the task board requires team sign-in for novelty review. A successful full assembly has not
yet been demonstrated. The current scripted controller is a first candidate, not a verified demo.

The initial scripted plan is not yet successful. The task, geometry, scorer and tests are on
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

From a GPU allocation, set `SIM_ROOT` to the installed Robocurve runtime root, then:

```bash
"$SIM_ROOT/cluster/run-arena.sh" inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task="$PWD/task.py" -E top_cam=demo -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 --instruction 'assemble the four keyed pieces into a square at the center of the table' \
  --scorer task_score --max-steps 1200 --seed 0 --store-frames --log-dir /path/to/run-output
```

Tested with `yam-isaac-lab-arena` commit `0059d02a1a91bc6834c1d0846022891df0c78730`,
Isaac Lab Arena 0.3.1, Isaac Sim 6 and `inspect-robots` 0.60.0. The tested renderer requires
the node's NVIDIA NGX library to be visible in the container. Run offline checks with
`python -B -m unittest -v test_task` in the Arena environment. `geometry.py` defines the pieces;
`procedural.py` creates their primitive colliders and rigid bodies at runtime. Model-run evidence,
media upload, a full-score demo and the board's novelty check remain outstanding.
