# Crack a walnut with a figure nutcracker

Lift the rear lever to open the mouth, load one intact walnut with the other
arm, then press the lever and leave the cracked shell released. This is a
fixed-base, two-arm tabletop figure task. The jaw and rear lever form one
rigid linkage; no motor assists the robot. The figure is an original reinforced
fabrication, with a 120 mm head, 60 mm mouth, 130 mm square base and 80 mm wide
rear grip. It is not a retail decorative soldier represented as functional.

[REAL_TRIAL.md](REAL_TRIAL.md) specifies the dimensioned figure, aluminum
load-bearing mechanism, PETG exterior, hinge/stop construction, mounting,
walnut sourcing/selection/conditioning, physical qualification and a practical
photo/video scoring procedure. [fabrication-envelope.csv](fabrication-envelope.csv)
lists every external figure primitive in millimetres; regenerate it with
`python export_envelopes.py`. Internal metal construction is specified in the
physical guide and replaced by rigid outer envelopes with assigned masses in
the simulator. No physical fabrication or force qualification is claimed.

Use a fresh, raw dried English walnut from one recorded supplier/lot per trial,
selected to 36 × 32 × 36 mm nominal and 14 g (tolerances in the physical guide).
Do not precrack, soak, score, glue or reuse specimens. Figure origin in station
coordinates is (0.320,−0.040,0.8675) m, yaw +90°; walnut reset centre is
(0.200,0.130,0.7665) m before settling on the 0.7475 m table. The walnut suture
starts horizontal, with the two simulated halves stacked in Z. Both arms start
in the standard YAM ready pose. Seed 0 is fixed, with no randomized offsets.

## Score visible outcomes

Score at the end, after two uninterrupted seconds of released, safe state:

* 1.0: video shows opposing jaw contact and at least1° jaw closing, followed
  by continuous mouth custody with no robot/other-object contact until a new
  fissure first appears at≥0.5mm and first reaches≥3mm normal opening. Reveal
  delay does not change this witness. Then both grippers open≥70mm and stay
  ≥80mm clear for final2s, fixture motion<10mm, shells supported on table/in
  mouth and no more than220mm above table. The documented crack may close.
* 0.25: no measurable fissure ever appeared, shell remains supported/seated in
  the specified mouth region with the same final released containment checks.
* 0: all other observable outcomes. Occluded/ambiguous physical custody or
  contact evidence is repeated with a fresh nut rather than guessed.

The physical guide gives scale placement, two camera views, seating boundaries,
height limits and an annotation record. `nutcracker_task/scoring.py` implements
the rubric without a native seam-event input. Simulated opening uses positive normal separation at matching
rim landmarks as a proxy for the visible fissure. Conservative surface bounds
and actual support contacts check containment and gripper release. Contact telemetry restricts
qualification to designated jaw surfaces, ribs and guides, excluding hat/body contacts.
A provisional native 200 N / 5 N m fixed-joint break remains the fracture-generation mechanism and is
logged for diagnosis, but it cannot by itself earn credit. The first measurable
0.5 mm fissure is checked once, so cracking elsewhere and later putting fragments in
the mouth fails. The simulated two-piece shell and biological fracture forces
remain approximations: see [FIDELITY.md](FIDELITY.md).

## Reproduce

In the existing Isaac/Arena environment:

```sh
uv pip install --no-deps --no-build-isolation -e .
python build_assets.py
python export_envelopes.py
python -m unittest -v test_scoring test_geometry
python diagnostics/demo.py <new-output-dir> 0
```

The demo uses simulator feedback for motion planning but robot-object forces
come from contact. Pose writes occur only at reset. Do not use the fixture-only
`diagnostics/figure_bench.py` as a robot demo: it has a diagnostic motor and
places the walnut by a pose write.

Model run, same seed, scene, cameras and 2,400-action limit:

```sh
inspect-robots run --embodiment yam_nutcracker --policy dream \
  -P model=dreamlabs/dream-1-beta \
  --instruction 'Lift the lever on the nutcracker’s back to open its mouth, put the walnut in the mouth, then press the lever to crack it.' \
  --scorer task_score --grader none --no-prompt --seed 0 --max-steps 2400 \
  --store-frames --log-dir <new-output-dir>
```

Physics: CPU PhysX, 480 Hz, control 20 Hz; three 640×360 policy RGB cameras plus a fixed960×540 front-oblique diagnostic camera; Dream
resizes inputs to 320×176 JPEG. Environment: Isaac Sim 6.1, IsaacLab 17.0.2,
Arena 0.3.0, yam-arena 0.1.0 and Inspect Robots 0.60.0 (verify actual runtime
versions in fresh logs). The fixed diagnostic camera is recorded by the demo driver from the same live trajectory; it is not an extra model input. Its station eye is(0.130,−0.140,0.875)m, target(0.280,−0.040,0.8775)m,24mm focal length and20.955mm horizontal aperture. The action space is fourteen joint/gripper targets.

Local 2026-11-17 evidence on the revised source: scripted demo **1.0**,1,553
of2,400 actions,243.684s wall; Dream-1 beta **0.0**,2,400 actions,370.181s wall,
no errored trials. Both use seed0 and environment revision
`d2bcea4533dbe1db36180c2d39d4963d59f25369131c29783bf2afc7c356210e`.
Terra reviewed the same-trajectory close-up showing opposing jaw interfaces,
closure and shell separation, plus the released final hold. One half settles on
the table, the other remains in the mouth. Post-flight half separation is not a
measurement of crack width.21offline tests and11geometry checks pass.
`validation.json` records provenance. This remains simulation evidence; physical
fabrication and qualification are unperformed.

Review media **M-V4JZ7P** contains the matching revised model run, full-score
demo and initial setup image. The demo displays synchronized station and
witness-camera views of the same trajectory; the model video uses the native
station camera. Physical fabrication and force qualification remain unperformed.

The board also contains a related two-handle nutcracker task with shell/kernel
sorting. This task uses a figure's rear lever and does not include sorting;
reviewers determine substantive novelty. No novelty acceptance is claimed.
