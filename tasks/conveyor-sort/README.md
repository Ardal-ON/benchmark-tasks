# Sort blocks from a moving conveyor

Five red and blue blocks travel left on a conveyor. Sort them into the matching bins, which are between the conveyor and the robots. **Media upload is pending review.**

The station table is z=0.7475 m; x is forward from the arms and y is left. Five 0.04 m plastic-like cubes, 0.05 kg each, start at x=0.34, z=0.7875 and y=0.30, 0.10, -0.10, -0.30, -0.50 m, in red, red, red, blue, blue order. A dark rubber-like 0.12 × 1.20 × 0.02 m belt is centred at (0.34, 0, 0.7575); ten thin yellow bands move in +y. Fixed red and blue plastic-like bins are centred at (0.15, 0.22) and (0.15, -0.22). Each has a 0.14 × 0.20 × 0.01 m floor and 0.01 m thick walls rising 0.06 m above the table. Reset restores the blocks and arms; the belt runs during the 0.5-second settling period, advancing the blocks approximately 6 mm before the first observation. No consumables are used.

The belt is a kinematic collider with PhysX surface velocity of 0.012 m/s in +y. Yellow visual-only stripes move at the same speed. In a five-second hold-still test, blocks stayed on the belt and travelled approximately 0.060 m. CPU PhysX is required: in a GPU-physics test, blocks fell through the conveyor. The RTX GPU still renders cameras. Rollers and deformation are not modelled. All five blocks passed the five-second transport check.

## Run

The tested environment used yam-arena commit `0059d02a1a91bc6834c1d0846022891df0c78730`, Isaac Lab-Arena commit `c7edf3a9d5d8276565208f72422f8f8a0806d6cf`, Isaac Sim 6.1.0.0, inspect-robots 0.60.0 and inspect-robots-dream 0.1.0. Install yam-arena and its dependencies as in its guide. From this folder, in the Arena environment:

```sh
uv pip install -e . --no-deps
python -m pytest -q test_score.py
python check_conveyor.py
TASK="$PWD/task.py"
INSTRUCTION='Sort the red and blue blocks from the moving conveyor into the matching colored bins.'
inspect-robots run --embodiment yam_conveyor --policy dream \
  -E task="$TASK" -E device=cpu -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir dream
inspect-robots run --embodiment yam_conveyor --policy yam_scripted \
  -E task="$TASK" -E device=cpu -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir demo
for log in dream/*.json demo/*.json; do inspect-robots video "$log"; done
```

The fixed layout uses `object_jitter=0`; the seed has no effect on object positions. The 2,400-step limit is 120 seconds of simulated time at 20 Hz. The scripted demonstration scored 1.0 in 575 steps and took about 79 seconds wall-clock on an L40S VM. The Dream-1 run scored 0.4 at 2,400 steps and took about 264 seconds wall-clock. Wall-clock time varies with hardware and policy latency.

## Score

At the end of the run, each correctly placed block earns 0.2. To count as placed, its entire horizontal bounding box must be inside its matching bin's 0.12 × 0.18 m interior; its bottom must be within 0.012 m of the bin floor at z=0.7575, and its linear speed must be below 0.01 m/s and angular speed below 0.5 rad/s. If all five meet the criterion for one second, the run ends at 1.0.

For each other block, let `L = clamp((bottom_z - 0.7675)/0.04, 0, 1)` and `P = clamp(1 - horizontal_distance_to_matching_bin_center/0.45, 0, 1)`. Its partial score is `0.2 × L × (0.2 + 0.78 × P)`. This gives continuous credit for elevation and progress toward the matching bin; elevation is a proxy for lifting, not a measurement of gripper contact. It is evaluated at the end, so a dropped block loses lifting credit. A block in the other bin does not count as a placement.
