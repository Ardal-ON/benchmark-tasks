# Seat a flanged bushing with an arbor press

A compact manual press seats a stock-type flanged polymer sleeve in a steel pivot housing. The workshop fixture holds the housing in line with the ram, and the sleeve starts square at its chamfer. The hand pulls the lever, raises the ram, and disengages. The flange gives a visible seating datum after the load is released.

The setup represents bench assembly of a plain bearing. The proposed reference part is igus GFM-2528-16, with a 35 mm flange OD, 28 mm nominal sleeve OD, 25 mm nominal bore and 16.5 mm length. A 25–30 degree entry chamfer eases alignment. The housing is a 52 × 52 × 39 mm steel block with a through-bore of 28.000–28.021 mm for the stock part. The supplier notes that installation changes the inside diameter. The scored task evaluates seating rather than running clearance. [Installation procedure](https://www.igus.com/company/plain-bearings-install-and-quality-test-ca) · [Reference part dimensions](https://www.igus.com/ContentData/Products/Downloads/iglide_G300_FM_USen.pdf)

Replacing worn parts and assembling mechanical components are O*NET work activities for general maintenance and repair workers. [O*NET 49-9071.00](https://www.onetonline.org/link/summary/49-9071.00)

## Physical setup and reset

Position the fixture and press as shown in the setup image. The coordinate origin is between the arm bases at tabletop height. Positive X points away from the arms and positive Y points to the robot's left. The base of the press is at X 420 mm, Y −125 mm, with the press front facing the arms. Its cast-iron frame is bolted to the bench. The housing is centered on the anvil at X 402 mm, Y −125 mm and held by the fixture. Its top is 75 mm above the bench. The sleeve rests coaxially against the bore's entry chamfer with the flange datum 15 mm above the housing. The cast-iron compact press frame uses a steel hand lever with a cylindrical grip at a 160 mm working radius and a 34.6 mm flat platen. Both hands start clear.

For each physical reset, install an equivalent fresh bushing at the chamfer, raise the ram, and verify the full initial gap. A new bushing preserves the press fit from trial to trial. Use a rated press and fixture, and keep hands out of the ram throat during loading. Specify the same geometry and grip radius if substituting a different real press.

## Scoring

Inspect after unloading, with the ram raised and hands clear. A 7.5 mm retained advance produced by the press earns 0.5. An intact flange seated around the rim earns 1.0 if the face is square and the largest gap is at most 0.20 mm. Check at several points around the rim with a feeler gauge and compare the initial and final height with a ruler or depth gauge. Video shows the platen following the sleeve during insertion. Give zero below the partial mark or when the work remains held or damaged. A completed success must remain at rest for one second. Simulation scoring uses the same height datum, with a 0.20 mm numerical penetration allowance, a 32 mm hand-clearance proxy and a continuous platen and sleeve displacement check.

## Simulation and fidelity

The press uses a native PhysX rack-and-pinion constraint between the revolute lever and sliding ram. The effective pinion radius is 20 mm. Contact with the handle supplies the stroke and the platen pushes the sleeve. The sleeve and chamfered housing use rigid compound collision shapes. A coaxial guide represents the already-started fit. A force-limited yielding drive opposes axial motion and retains the partially or fully pressed position. Its 35 N provisional force cap is a modeled value. The implementation does not predict material deformation or the insertion force of a particular production batch.

## Reproduce

Run the commands from this folder inside the Isaac Lab-Arena environment with the official YAM Arena package and Inspect Robots. The packaged USD files are ready to load. To regenerate both overlays, start Isaac Sim before importing its PhysX schemas.

```bash
python -c 'from isaaclab.app import AppLauncher; app=AppLauncher(headless=True).app; from build_assets import build_press,build_fit; build_press(); build_fit(); app.close()'

instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task="$PWD/task.py" -E top_cam=demo -E cam_width=640 -E cam_height=360 \
  -E object_jitter=0 -E device=cpu -E control_hz=20 --seed 17 \
  --instruction "$instruction" --scorer task_score --max-steps 1200 \
  --store-frames --log-dir ./runs/demo --no-prompt --grader none --no-rerun
```

For the matching model run, use `--policy dream -P model=dreamlabs/dream-1-beta` and a separate log directory. Use the same frozen scene, physics, seed, action limit and camera settings. Record actual results from the completed run JSON. Physics runs at 240 Hz and the L40S renders the three native camera views.

## Recorded results

On an NVIDIA L40S, the scripted demo scored **1.0** after 264 actions and 39.04 seconds of wall time. The settled sleeve was at 15.135 mm of advance and the hand was clear. Dream-1 beta completed 1,200 actions in 125.80 seconds and scored **0.0**. Video and score both come from each completed run. The video uses the native top camera with a synchronized right wrist inset. See `validation.json` for the recorded runtime and file hashes.

## Assets and licenses

- **NVIDIA USDCraft-10K Small Benchtop Arbor Press**, `ds_R855_02`. Apache License 2.0, included in `assets/LICENSE.Apache-2.0`. The untouched source is `assets/source_press.usdc`, SHA-256 `f46073cbfeb5b8b1fc9b284b8f706c941e5c95562471970bd9a21f41070ed944`. The overlay adds the fixed base, transmission, lever extension, platen and joint settings. [Pinned asset and metadata](https://huggingface.co/datasets/nvidia/USDCraft-10K/tree/9265e910275b28e1317893141e1a838da9e657c1/rigid/industrial_warehouse_construction_energy/R855_arbor_press/02_small_benchtop_arbor_press_20_x_14_x_30_cm_in_gr)
- **I2RT Robotics YAM URDF and link meshes**, MIT. YAM Arena supplies the robot and its split-color Dream link variants from Dream Labs 1.0.23. The station table and camera stand are authored primitives in the simulator package. [Robot repository](https://github.com/i2rt-robotics/i2rt) · [YAM Arena repository](https://github.com/kairosiann/yam-isaac-lab-arena)
- The fixture and bushing meshes are authored from dimensions. The igus sources above are dimensional and procedural references.
