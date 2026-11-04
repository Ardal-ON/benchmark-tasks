# Sweep machining chips into a collection bay

Use a handled brush to clear five loose chip surrogates from an unpowered machining
tray into its green collection bay. Return the brush and withdraw with open grippers.
This tests gathering several small loose objects with a broad tool, including an edge
pass to recover a chip that slips around the head. There is no active machining.

The scripted physics demonstration scored **1.0** in **755 control steps** (37.75 simulated
seconds), with **success** termination and **85.04 seconds wall time** on an L40S. All three
complete camera videos were downloaded. The top sequence and final frame were visually
reviewed: all five chips are in the bay and the brush is parked. Six scoring checks pass.
Dream-1, authenticated novelty review and media upload remain pending.

## Reproduce

Use the configured YAM Arena RTX environment. Install this folder as an editable package
and keep its assets alongside the Python package:

```bash
uv pip install -e . --no-deps
python -m unittest discover -s . -p test_score.py
inspect-robots run --embodiment yam_chip_sweep --policy chip_sweep_scripted \
  -E task="$PWD/task.py" -E object_jitter=0 -E device=cpu -E top_cam=demo \
  -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction 'Use the brush to sweep all five metal chips into the green collection bay. Return the brush to its supports and open both grippers.' \
  --scorer task_score --max-steps 1800 --seed 0 --store-frames \
  --log-dir /path/outside/repository/chip-demo
```

On CARC, run this inside the L40S allocation using the installed
`$SIM_ROOT/cluster/run-arena.sh` environment wrapper. Render the completed JSON with
`inspect-robots video <run.json>`. For the model trial, use the same embodiment, scene,
seed and limit with `--policy dream -P model=dreamlabs/dream-1-beta`.

## Fixed setup

World x points forward, y left and z up. The tabletop is at z=0.7475 m. `task.py` lists
all fixed poses. `build_assets.py` regenerates the self-contained USD assets with usd-core.

- The metal tray is centered at (0.35,0), with a 350 × 190 × 12 mm floor, side rails,
  back wall, front lip, fasteners and reference marks. The green visual bay is coplanar
  with the floor, whose top is z=0.7595 m; no step blocks the sweep.
- Five rigid, blunt 12 × 8 × 6 mm blocks approximate small chips. Each is 6 g. Their
  centers start at xy=(0.405,-0.036), (0.428,0.030), (0.455,-0.012), (0.475,0.035),
  and (0.485,-0.038) m, at z=0.7625 m, with yaw 0, 0.3, -0.4, 0.6 and -0.2 rad.
- An 80 g rigid brush starts at root (0.24,0.27,0.7955) m. Its wooden handle, rubber
  grip sleeve and steel ferrule are visible. Its head spans 115 mm along world y;
  a 25 × 115 × 15 mm contact envelope represents the bristles. Two fixed supports at
  x=0.18 and 0.34, y=0.27 have heights 56 and 38 mm.

Reset restores all objects and both arms, clears contact/failure history, and settles
physics. Seed 0 and zero object jitter define the demonstrated layout. Nothing is consumed.

## Scoring

Use the final score. Each chip earns 0.16 only after a measured brush–chip contact and
when its entire world AABB is inside x=(0.185,0.305), y=(-0.085,0.085) m, bottom within
3 mm of z=0.7595 m, linear speed below 10 mm/s and angular speed below 0.5 rad/s.

Add 0.20 only when all five qualify and the brush is parked, resting, with both grippers
at least 75% open and both TCPs farther than 90 mm from its reference center. Parking
requires the Arena local-bounds reference center within 25 mm of (0.22875,0.27) m,
bottom within 6 mm of z=0.7855 m and vertical extent below 45 mm. Holding 1.0 for one
second triggers success. `scoring.py` is the numerical specification.

Any direct robot–chip contact permanently invalidates the trial. So does a brush/chip
bottom below tabletop minus 20 mm, nonfinite position, center outside x=(-0.1,0.95) m,
or |y|>0.62 m. Invalid trials score zero. Unqualified chips earn zero; no intermediate
maximum is retained. Tool contact is required for every credited chip.

## Physics and limitations

PhysX runs at 240 Hz with 20 Hz joint actions. Loose objects use the station's approximate
1.2/1.0 static/dynamic friction, 5 mm contact offset and zero rest offset. The brush's
visible bristle tufts share one solid contact envelope; they do not bend. Chips do not
curl, deform or cut. Materials and mass distribution are stylized and uncalibrated.
No adhesion, attachments, object-pose writes or scripted object motion are used.

The demonstration reads privileged object poses and FK to correct brush placement and
orientation. It sends only robot joint targets. Dream receives the ordinary camera and
joint observations through the same contact-tracking embodiment. Contact processing is
explicitly enabled; measured reports provide tool-use provenance and reject direct
handling. A real-world check could use video and a ruler with blunt training surrogates.

Tested sources: YAM Arena `0059d02a1a91bc6834c1d0846022891df0c78730`, Arena
`c7edf3a9d5d8276565208f72422f8f8a0806d6cf`; Python 3.12.12, Isaac Sim 6.1.0.0,
Isaac Lab 17.0.2, Arena 0.3.0, Inspect Robots 0.60.0, NumPy 2.3.1 and Torch 2.11.0+cu128.
Runtime hashes and pending submission preparation are in `validation.json`.
