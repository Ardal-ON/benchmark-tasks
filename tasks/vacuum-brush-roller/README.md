# Refit a vacuum brush roller and lock its slotted end cap

Slide the roller into a detached floor head, insert a coin into the end-cap slot, turn it clockwise through a quarter turn when viewed from the end, and withdraw the coin. This maintenance operation uses the same insertion and coin-lock steps described for Dyson's soft roller cleaner head. [Dyson cleaner-head maintenance procedure](https://www.dyson.com/support/journey/troubleshooting/205976-01/check-for-blockages-within-the-soft-roller-cleaner-head-380060)

## Setup and reset

Use a detached head with a coin-operated end cap or reproduce the dimensioned assembly with `build_assets.py`. Set a 176 × 110 × 120 mm rigid bench support under the graphite ABS shell. The guide axis runs along +X, with the coin-facing end at X 407 mm, Y −185 mm and 158 mm above the table. Stage the 46 × 188 mm brush tube already engaged in its guides, 100 mm from the seated position. Set the 56 mm diameter cap to UNLOCK with the slot vertical. Positions follow the YAM station coordinates.

The quarter stands on edge at X 257 mm, Y −331 mm, with its center 30 mm above the table. Its two 28 × 5.4 × 3.5 mm rigid holder rails are fastened to a 40 × 28 × 17.8 mm base. The simulated unmarked blank shares the quarter's 24.26 mm diameter, 1.75 mm thickness and 5.67 g mass. [U.S. Mint coin specifications](https://www.usmint.gov/learn/coin-and-medal-programs/coin-specifications)

## Physical scoring

Use a ruler and the lock index after the hardware is still. Award 0.25 for an advance of at least 50 mm. Award 0.5 for seating within 3 mm of the closed stop. Award 1.0 when the roller is seated, the cap has reached the 90° lock index within 5° using the coin, and the coin is withdrawn at least 17 mm from the nominal seated cap face. Hold the completed state for one second. All other outcomes earn zero. The simulator reads the slide and cap angles from the articulation, measures at least 55° of cap motion while the coin is in the slot, and measures withdrawal from its bounds.

## Run a trial

Run from this directory inside Isaac Lab-Arena with the YAM Arena embodiment installed. The generated USD files are included. Run `python build_assets.py` in an OpenUSD Python environment to rebuild them.

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task="$PWD/task.py" -E top_cam=demo -E cam_width=640 -E cam_height=360 \
  -E object_jitter=0 -E device=cpu -E control_hz=20 --seed 17 \
  --instruction "$instruction" --scorer task_score --max-steps 1600 \
  --store-frames --log-dir ./runs/demo --no-prompt --grader none --no-rerun
```

For the matching Dream-1 trial use `--policy dream -P model=dreamlabs/dream-1-beta` and a fresh log directory. PhysX runs at 240 Hz and station cameras record at 20 Hz.

## Fidelity and assets

The floor head, support, brush roller and unmarked coin are original dimensioned geometry. The guided roller uses a passive axial slide with a closed stop. The cap has two convex collision banks around the slot and two lugs that contact stationary catches and retain the roller once turned. The shell profile and bristle tufts are visual. The detached head is unpowered during the operation.

The isolated `python check_mechanism.py` bench check applied an 8 N outward load. The locked roller stayed within 2.5 mm of its seat. After unlocking it returned to the open stop under the same load.

The simulator supplies the **I2RT Robotics YAM URDF and link meshes**, MIT, including the packaged Dream split-color variants. The station table and camera stand use YAM Arena's authored primitives. [I2RT robot sources](https://github.com/i2rt-robotics/i2rt) · [YAM Arena sources](https://github.com/kairosiann/yam-isaac-lab-arena)

## Simulation validation

Both trials used the NVIDIA L40S for RTX rendering and PhysX on CPU. Isaac Sim Kit 110.3.0 (00c488ae), exact dependency commits and the runtime asset hashes are recorded in `validation.json`.

| Policy | Score | Actions | Trial wall time |
| --- | ---: | ---: | ---: |
| Scripted YAM | 1.00 | 430 | 65.75 s |
| Dream-1 beta | 0.00 | 1,600 | 217.43 s |
