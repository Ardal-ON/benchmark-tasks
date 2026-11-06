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
64/16 solver iterations and 0.1 mm contact offsets. Five seconds of settled
observation showed approximately 0.012 mm ratchet-body drift and zero bolt
holder drift. The robot did not manipulate the tool in that check.

The intended clutch is a calibrated passive one-way torsional model. The bolt
thread uses a compliant helical force law, not colliding microscopic thread
ridges; the helical ridge is visual. Tightening is a low-torque bench task,
not an M8 service-load or damage prediction. These models require live
calibration before any demo or Dream trial can count as valid.

Dream remains gated on two identical frozen full-score robot demos and full
video review. The provisional limit is 7,200 actions at 20 Hz. No media ID is
assigned and no measured demo/Dream score is claimed.
