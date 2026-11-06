# Fasten a bracket bolt with a ratchet in confined access

The right gripper starts holding the fitted ratchet on a threaded bolt. Use
short drive and return strokes to seat the bolt between the walls. All
manipulation uses normal dual-YAM joint and gripper actions; the left hand stays
clear. Completion is the seated bolt after verified ratchet cycles.

The fixed layout contains an M8x16 bolt (1.25 mm pitch, 13 mm AF head, 14 g), a
3 mm bracket on a 93 mm receiver stand, a 203 mm ratchet and 13.4 mm AF
socket, and two 162 mm tall walls. The walls limit the ratchet swing to
about ±16 degrees; the reference controller uses ±14 degrees. The ratchet has
two small visible flat rubber grip pads, allowing the rigid fingertips to hold
its otherwise rounded grip. Dimensions and starting poses are in
`ratchet_task/config.py`, `asset_geometry.py` and `build_assets.py`.

## Run

Use the existing Isaac Lab Arena/YAM environment with Isaac Sim 6.1.0,
IsaacLab 17.0.2, Arena 0.3.0, Inspect Robots 0.60.0 and yam-arena 0.1. All
cluster simulation and rendering must run on an allocated compute node.
From this folder:

```sh
uv pip install --no-deps -e .
python -m unittest discover -p 'test_*.py'
python diagnostics/demo.py /absolute/path/outside/source/demo
```

The full demo runner records native top and wrist H.264 videos, per-step
mechanism/scorer telemetry and `demo-result.json`. The registered reference policy is `confined_ratchet_scripted`.

Dream evaluation uses the same physical scene, 480 Hz CPU PhysX, 20 Hz actions,
native 640×360 top and wrist cameras, `top_cam=demo`, zero object jitter and seed
17. The horizon is 4,000 actions (200 simulated seconds). With the private
Dream credentials configured outside this source directory:

```sh
inspect-robots run --embodiment yam_confined_ratchet \
  -E task=task.py -E top_cam=demo -E device=cpu -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 -E object_jitter=0 \
  --policy dream -P model=dreamlabs/dream-1-beta \
  --instruction "Use the fitted socket ratchet to tighten the started bracket bolt with short back-and-forth strokes between the walls until the bolt is seated." \
  --seed 17 --max-steps 4000 --scorer task_score \
  --store-frames --no-rerun --no-prompt --grader none --log-dir /absolute/path/run
```

## Contact, rotation and scoring

The socket and receiver use open compound collision geometry. The bolt head
contacts the metric socket; no grasp attachment or policy-driven pose changes
are used. The handle, rotor and socket have masses of 399.405 g, 28.355 g and
40 g. Handle friction is 0.9 static / 0.7 dynamic; other task bodies use 0.55 /
0.4. The native one-way hard limit has 5 degrees of take-up and passive return
drag. The thread uses a native prismatic/revolute pitch coupling, with a hard
terminal seat. Thread ridges are visual, and microscopic tooth impacts,
material deformation and fastener strength are omitted. The invisible 14 g
carrier adds translational inertia and has no contact geometry.

The bolt starts engaged in the receiver with a 2.5 mm under-head gap, and the
socket starts fitted. Reset-only placement joints hold this assembled initial
condition during environment settling; they are removed before the first
policy action. The native thread stays active throughout manipulation. The
socket is never attached to the bolt by a joint: its hex faces carry contact
torque and it can lift off or fall away.

The current-state score is 0.2 after a quarter clockwise turn, 0.4 after one
turn, 0.7 for seating within −0.3/+0.2 mm after at least 1.8 turns, and 1.0
after a stable seated hold. The initialized assembly earns zero.
Seating must hold for 0.5 seconds with the socket engaged and at least six
ratchet drive/return cycles, each with actual bolt advance. The scorer rejects nonfinite states, thread-constraint
failure, fixture displacement above 2 mm and contact torque above 1 N m.
Contact torque is recorded from physical contact impulses; a calibrated torque
target is not required.

## Assets

The ratchet visual is derived from NVIDIA SimReady, NVIDIA, CC-BY-4.0,
pinned revision `653d91e182011f357cf1644013cc94ddf194d349`:
https://huggingface.co/datasets/nvidia/simready-assets/tree/653d91e182011f357cf1644013cc94ddf194d349

Source meshes and UVs are preserved, source materials converted to portable
UsdPreviewSurface, and textures reduced to 1K. The source was located through
the NVIDIA SimReady asset search API. The metric socket, bolt, fixture, grip
pads and walls are original parametric geometry. Licenses, hashes and
derivation records are under `assets/`. The optional imperial socket visual is
reference material; the runtime uses the dimensioned metric socket.

Portable assets and textures are included. Rebuild with OpenUSD, NumPy, SciPy
and Pillow using `python fetch_assets.py --prepare` followed by
`python build_assets.py`. Original large vendor downloads are ignored by Git.

## Results

The reviewed confined demo reached **1.0** in 1,393 actions
(69.65 simulated seconds; 301.7 wall seconds on an L40S).
It completed 31 drive/return cycles and 1.843 clockwise bolt turns, ending at a
0.197 mm gap. Maximum pitch error was 14.2 micrometres. Socket engagement held
throughout the trial and no integrity fault was reported.

Dream-1 evaluation is pending. Both evaluations use seed 17 and identical
physical sources. Runtime evidence revision:
`1cd38c2db0a77b52f15efa78c00f74834ccf7995dfa1f354601b659b17228d3f`.

Sources and evidence are retained locally. Media has not been uploaded and no
branch has been pushed or submitted. No media identifier is assigned.
