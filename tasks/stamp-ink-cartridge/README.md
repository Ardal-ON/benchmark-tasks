# Replace the ink pad in a spring-return self-inking stamp

One hand depresses the cap against its return spring and keeps the cartridge path clear. The second hand slides the fresh pad into the stamp while the used pad exits into the tray. Releasing the cap closes the inlet. Trodat documents depressing the cap to eject the old cartridge and then installing the replacement. The simulated bench fixture aligns the fresh pad so it can push out the used one during insertion. [Trodat Original Printy cartridge replacement](https://www.trodat.net/us/en/products/b2c/c1/at-home/c2/original-trodat-printy-2/c3/date-stamp-with-text-1/p/4817p2)

## Setup and reset

Use a self-inking stamp with front-to-back cartridge access, or reproduce the dimensioned dry ABS assembly from `build_assets.py`. The simulated cap is 92 × 46 × 14 mm, the frame footprint is 80 × 56 mm, and the raised assembly is 90.6 mm tall. A 290 × 130 × 105 mm rigid bench fixture fixes the frame upright. The cap has 16 mm of travel and opens two inlet gates. The used black pad and the base of the blue replacement each measure 49 × 42 × 8 mm. The replacement has a 9 × 33 × 34 mm thumb flange on a short tab. The black pad weighs 9 g and the blue pad and tab weigh 10 g.

Positions are in the table coordinate system with positive X away from the arms. The stamp stands at X 320 mm, Y 40 mm with the pad path running along X. The used pad starts centered. The fresh pad starts at X 245 mm on the 72 × 52 × 42 mm rigid support, with two side runners and its tab facing the robot. The flat 110 × 100 × 12 mm catch tray is centered at X 395 mm, Y 40 mm on a 105 × 92 × 29 mm riser. The tray surface is 41 mm above the fixture, 1 mm below the cartridge path. Reset both pads and fully raise the cap.

## Physical scoring

Check from a side photo and a ruler after releasing the cap. A used pad fully on the catch tray earns 0.5. The full score additionally requires the replacement to rest within 2.5 mm of its 75 mm seated stroke and the cap to come back within 2 mm of its raised position. Keep the completed state stable for one second. All other outcomes score zero. The simulation measures the guided slide and the used pad's bounds and resting speed.

## Run a trial

Run from this directory inside Isaac Lab-Arena with the YAM Arena embodiment installed. The generated USD is included. Rebuild it with `python build_assets.py` in an OpenUSD Python environment if needed.

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task="$PWD/task.py" -E top_cam=demo -E cam_width=640 -E cam_height=360 \
  -E object_jitter=0 -E device=cpu -E control_hz=20 --seed 17 \
  --instruction "$instruction" --scorer task_score --max-steps 1600 \
  --store-frames --log-dir ./runs/demo --no-prompt --grader none --no-rerun
```

For the matching Dream-1 trial use `--policy dream -P model=dreamlabs/dream-1-beta` and a fresh log directory. The scene uses 240 Hz PhysX and records the native station cameras at 20 Hz.

## Fidelity and assets

The stamp, pads and bench fixture are original dimensioned geometry. The incoming support and guide surfaces are represented by a passive slide and hard seat. The spring-loaded cap's inlet gates physically block the fresh pad until the cap is depressed. The old pad is a free rigid body and contacts the housing, fresh pad and tray. The model covers a dry pad exchange.

A control trial used the same right-arm push with the cap left raised. The inlet stopped the replacement after 26.3 mm of travel, before it could reach the used pad. The trial scored 0.0.

The simulator supplies the **I2RT Robotics YAM URDF and link meshes**, MIT, including the packaged Dream split-color variants. The station table and camera stand use YAM Arena's authored primitives. [I2RT robot sources](https://github.com/i2rt-robotics/i2rt) · [YAM Arena sources](https://github.com/kairosiann/yam-isaac-lab-arena)

## Simulation validation

Both trials used the NVIDIA L40S for RTX rendering and PhysX on CPU. Isaac Sim Kit 110.3.0 (00c488ae), exact dependency commits and the runtime asset hashes are recorded in `validation.json`.

| Policy | Score | Actions | Trial wall time |
| --- | ---: | ---: | ---: |
| Scripted YAM | 1.00 | 301 | 64.78 s |
| Dream-1 beta | 0.00 | 1,600 | 188.77 s |
