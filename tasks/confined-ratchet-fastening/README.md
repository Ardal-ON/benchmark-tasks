# Fasten a bolt with a ratchet in confined access

Development checkpoint: the open-bench assets load and settle in the actual YAM
station on an L40S. The mechanism, robot demo and Dream evaluation are not yet
qualified. No task-board upload, push or submission has been made.

The task uses an original M8x16, 1.25 mm pitch bolt with 13 mm AF head; an open
13.4 mm AF socket; and a detailed NVIDIA SimReady 203 mm ratchet. The bolt starts
loose in a holder. The ratchet has separate dynamic handle and output bodies;
the custom socket is permanently assembled to the output. The threaded
receiver and bracket have genuine open collision bores. Walls are a later
stage, disabled in the open-bench asset preflight.

## Assets and attribution

NVIDIA SimReady ratchet and optional imperial socket visual, NVIDIA,
CC-BY-4.0, pinned source revision
`653d91e182011f357cf1644013cc94ddf194d349`:
https://huggingface.co/datasets/nvidia/simready-assets/tree/653d91e182011f357cf1644013cc94ddf194d349

The ratchet visual preserves source meshes and UVs. Physics schemas were
removed and materials converted to portable UsdPreviewSurface. Textures were
reduced from 4K to 1K. No visual scaling was applied. The optional imperial socket
visual is preserved for reference; the runtime uses our dimensioned metric
socket instead. Licenses, SHA256 hashes, source URLs and derivative operations
are in `assets/`. Original source downloads are reproducible and ignored by Git;
portable visual meshes, runtime textures and licenses are committed.

Rebuild using an environment with OpenUSD, NumPy and Pillow:

```sh
python fetch_assets.py --prepare
python build_assets.py
python -m unittest discover -p 'test_*.py'
```

The asset search API used was
`https://nvidia-simready-central.hf.space/search`; identifiers were verified,
not invented. No other team's code was copied.

## Runtime and validation

Use the configured Arena/YAM environment. All cluster Python, asset processing,
simulation and rendering must run on an allocated compute node, never the login
node. The existing `cluster/run-arena.sh` activates the container environment.

From this task directory:

```sh
python diagnostics/assets.py /absolute/path/outside/source/assets-check
```

This creates native top and wrist camera images and measured settled poses.
The open-bench check used 480 Hz CPU PhysX, 20 Hz control, L40S RTX rendering,
128/64 solver iterations (the earlier asset-only run used 64/16) and 0.1 mm contact offsets. Five seconds of settled
observation showed approximately 0.012 mm ratchet-body drift and zero bolt
holder drift. The robot did not manipulate the tool in that check.

The mechanism now uses a bounded native torsional spring with 5 degree take-up,
100 N m/rad engaged stiffness, 1.2 N m capacity, and passive return drag. Its
free-axis reference follows return travel. The M8 thread uses a native PhysX
prismatic carrier, revolute bolt and rack/pinion pitch coupling; microscopic
thread ridges are visual. An auxiliary native twist-only terminal constraint
models hard seating. It carries seating torque without claiming clamp preload.
A 14 g numerical carrier adds translational inertia; it is invisible and has
no collisions. A 0.015 N m bounded angular bearing resists return backdrive.

A separate annular entry stop blocks pushing an unstarted bolt fully into the
receiver. Capture requires 1–2.5 mm tip insertion, 0.5 mm radial error or less,
5 degrees tilt or less, and 30 degrees clockwise starting rotation. The native
helix is installed before the entry stop is disabled. Reverse removal restores
the stop only after the actual bolt has cleared it. Task policies have no pose,
force or constraint mutation API.

The socket-to-bolt torque meter integrates actual normal-contact and friction
anchor impulses about the bolt axis every physics substep. A separate fixture
validated known external spindle loads of 0.1, 0.3, 0.5 and 0.7 N m to within
0.000006 N m mean error. Torque proof uses a control-period mean, not the spring
deflection estimate. Loaded free-running thread error was below 1 micrometre.
The complete mechanism gate is still under development; bench fixtures never
count as normal-action robot demos.

Dream remains gated on two identical frozen full-score robot demos and full
video review. The action limit will be frozen after timing the normal robot
demo. No media ID is assigned and no measured demo/Dream score is claimed.
