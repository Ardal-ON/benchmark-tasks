# Figure nutcracker prototype

This is a work-in-progress simulation of a standing nutcracker figure. Its face points toward the station's overhead camera. The figure has two matching arms and hands at its sides, and a separate lever centered behind its body. The enlarged head is 120 mm wide, with a centered 60 mm wide recessed mouth for the approximately 36 mm walnut. The lever and lower jaw are a rigid mechanism: lifting the lever opens the mouth; pressing it closes the mouth. The intended two-arm task is to lift and hold the rear lever with one gripper, place a walnut in the mouth with the other, then press the lever to crack the walnut.

**The seed-0 figure demo scored 1.0 in simulation:** the robot opened the lever, loaded the walnut, pressed the lever, and released; the trace records a break with contacts on both figure bodies. The review found unintended right-gripper contact with the hat, so this is not yet a clean submission-ready demo. Earlier results for the scissor-style prototype do not apply. A diagnostic motor can articulate the lever and load a nut for mechanism tests; this is not a robot demo.

The walnut is a simplified two-piece rigid body joined by a native breakable joint. A provisional 200 N / 5 N m joint threshold is an uncalibrated approximation; external crushing force is not necessarily equal to joint force. Walnut fracture loads depend on orientation and specimen; [published measurements](https://onlinelibrary.wiley.com/doi/full/10.1111/j.1745-4549.2011.00601.x) show a wide range. The visible kernel is an attached visual patch, not a separately simulated kernel; shell deformation and multiple fragments are not modeled.

The robot uses 14-dimensional joint actions and three RGB cameras. Physics uses CPU PhysX at 480 Hz; control runs at 20 Hz; RTX renders 640×360 camera images. The Dream client resizes each frame to 320×176 JPEG before model input. Robot interactions are contact-driven. Object poses are written only at task reset, except `diagnostics/figure_bench.py`, which explicitly loads a nut by a pose write for fixture-only testing.

A 1.0 score requires a native seam break with recent contact from the upper figure body and lower moving body, at least 3 mm of displacement at corresponding shell-rim landmarks, the upper and lower figure roots within 10 mm of the hinge and shell heights between 0.7425 and 0.9675 m, and both robot grippers open and clear for two seconds. An intact walnut held in the mouth with released grippers can receive 0.25. The scorer and physical dimensions are provisional; the figure demo is a simulation and the force model remains uncalibrated. Seed offsets and cameras are not randomized.

Install the package in the Arena environment with `uv pip install --no-deps --no-build-isolation -e .`. Build the assets with `python build_assets.py`. Run the recorded seed-0 scripted demo with `python diagnostics/demo.py <new-output-dir> 0` (limit: 2,400 actions); it completed with score 1.0 in 1,551 actions in approximately 194 seconds of wall time on an L40S. The revision was `a907cf5c45851a38ebdabfeb489c185e6e36748709931a44ce754c9de9b4aa99`.

Run Dream-1 using the same scene and limit:

```sh
inspect-robots run --embodiment yam_nutcracker --policy dream \
  --instruction 'Lift the lever on the nutcracker’s back to open its mouth, put the walnut in the mouth, then press the lever to crack it.' \
  --scorer task_score --grader none --no-prompt --seed 0 --max-steps 2400 \
  --store-frames --log-dir <new-output-dir>
```

The recorded diagnostic `python diagnostics/figure_bench.py <new-output-dir>` uses a test-only motor and walnut pose write; it is not a robot demonstration. Dream-1 (`dreamlabs/dream-1-beta`) scored 0.0 at the 2,400-action limit on seed 0; the completed run took approximately 321 seconds of wall time and recorded 50 model requests. The demo and Dream run used the same environment revision.

The live task board also contains a pending walnut-cracking task using a two-handle nutcracker and kernel/shell sorting. This figure task uses a centered rear lever and does not include sorting; the degree of novelty is subject to review.

Contact telemetry identifies the upper and lower figure bodies as a whole, rather than distinguishing the two jaw surfaces from other contacts. The scaled hat generates a PhysX warning that its collision representation may not match the visual; the scripted right gripper contacts it.
