# Align a supported pipe joint and insert two bolts

Grip the ordinary straight pipe with one arm, clock and slide the supported
spool until its flange holes align, and insert two bolts horizontally with the
other arm. There is no added grip lug, collar or lifting handle. The moving
part is a solvent-jointed PVC-U pipe spool on V rests, with a normal elbow and
branch at its far end. The eccentric branch creates a roll moment that the
holding arm must resist during insertion; the two V rests carry its weight.

This evaluates the alignment/insertion stage of bench pipe fabrication. The
joint is a dry fit: gasket installation, the remaining bolts, washers, nuts,
torque and pressure sealing are outside the task. The demonstration may end
with the pipe and second bolt held. It does not claim unsupported bolt retention.

## Real parts and dimensions

The [GF PVC-U manufacturer catalog](https://www.gfps.com/content/dam/gfps/com/product-ranges/en/gfps-00019b-product-range-pvc-u-en.pdf)
provides these exact part rows (printed pages 12, 14, 63):

| Part | Product and catalog dimensions | Mass used |
| --- | --- | --- |
| Two fixed socket flanges | GF721730208, d32/DN25; 115 mm OD, 85 mm bolt circle, four 14 mm holes, 14 mm plate, 25 mm overall, 41 mm neck |183 g each |
| Straight pipe | GF161017108, 32 mm OD, 2.4 mm wall; 430 mm moving cut, 200 mm fixed cut |347 g/m |
| Moving elbow | GF721100108, 90°, 32 mm socket, 40 mm OD, 39 mm socket-mouth/intersection dimension, 17 mm z dimension |43 g |
| Branch |200 mm cut of the same pipe |69.4 g |

The complete moving spool has catalog-derived mass
`183 + 430×0.347 + 43 + 200×0.347 = 444.61 g`, including the moving flange, two pipe cuts and elbow.
These are catalog/planning masses, not measurements of a built specimen.
Solvent cement contribution and exact physical inertia are unmeasured. Use the
maker's compatible PVC-U solvent jointing process, 22 mm nominal socket insertion
for the elbow and flange (L−z/H1−z), cure as directed, and record actual finished
mass and socket depths. Do not add grasp features or change the pipe diameter.

Two [norelem 07170-112X50 DIN 931 A2-70 bolts](https://norelem.co.uk/medias/07170-Datasheet-32104-Hexagon-head-bolts-with-shaft-DIN-931-ISO-4014-en.pdf?context=bWFzdGVyfHJvb3R8Mjg3OTY5fGFwcGxpY2F0aW9uL3BkZnxhRGt5TDJnd1lpODVOVEk1T0RNeU1qYzFPVGs0THpBM01UY3dYMFJoZEdGemFHVmxkRjh6TWpFd05GOUlaWGhoWjI5dVgyaGxZV1JmWW05c2RITmZkMmwwYUY5emFHRm1kRjlFU1U1Zk9UTXhYMGxUVDE4ME1ERTBMUzFsYmk1d1pHWXw3ODM5MWIxYjg5ZDVlMWM5N2E1ZTQ2NGI3ZDk3MDU0NzI3NzhlNDljMzMwZGVkOGMyNjUwN2FjNDZkNjkxMjFh)
are M12×50, 19 mm across flats, 7.5 mm head height. Simulation uses a 12 mm smooth
shank and hexagonal head. Threads and tip/head chamfers are omitted; the
full-diameter blunt tip is a conservative insertion-clearance approximation.
The 63.2 g assigned mass is the ideal solid stainless envelope estimate at
7.9 g/cm³; actual threaded bolts must be weighed before physical comparison.
This is an explicit mass/inertia approximation, not a claimed measured bolt.

## Supports, mounting and reset

Use two smooth 90° V rests under each pipe. Each rest has 30 mm axial length,
60 mm width, flat bottom and two 45° faces. The notch vertex is 147.373 mm above
the table, giving a 170 mm axis for 32 mm pipe. Outer top height is 177.373 mm.
Machine the rests in aluminum or steel, use a thin low-friction liner without
changing the finished contact envelope, and bolt their feet to the station.
The maker's [vinyl mechanical joining guidance](https://www.gfps.com/content/dam/gfps/us/products/piping-systems/chlorfit/chlorfit-industrial/gfps-us-Training%20Material-PM-401Vinyl%20Mechanical%20Joining%20Rev%20005%2011-01-16-en.pdf)
requires the piping to be well supported. Here the fixed pipe is additionally
clamped to its rests with two ordinary straps; ideal fixed constraints replace
those straps and mounting hardware in simulation.

Coordinates are metres in the YAM station frame, tabletop Z=0.7475:

* Fixed mating face: (.3145, .060, .9175), pipe extends toward +X. Fixed rests at
  X=.400, .480, Y=.060. Its pipe cut runs 3–203 mm behind the flange face.
* Moving mating face initially: (.290, .060, .9175), pipe extends toward −X.
  Moving rests: X=−.020, .240, Y=.060. The 430 mm cut runs 3–433 mm behind the face;
  elbow socket begins at 411 mm, its intersection is at 450 mm. The branch points
  toward +Y at the target roll; its pipe cut runs 17–217 mm from that intersection.
  Reset the spool to −15° roll about world X; align its bolt clocking with the
  fixed flange by returning to 0°. Move its face to X=.3125, leaving 2 mm dry gap.
* A 50 × 25 × 113 mm metal staging block centred X=−.160, Y=.195 supports the lowered
  branch during reset. Clocking raises the branch clear of this block. With the 22.5 mm slide the
  block remains partly below it; release can let the branch return to the
  reset roll and lose hole alignment. It bears no branch load in the aligned
  held pose. The branch is never a grasp surface.
* Two 50 × 28 × 185 mm fixed pin stands are centred (.325, −.300) and (.325, −.180).
  Bolts start along+X with heads toward −X, shaft origins (.320, Y, .9425), then
  settle on the stands. The hex head overhangs the stand end.
* Both arms reset to YAM ready: (0, 1, 1, −.6, 0, 0) rad, grippers open. No jitter. The four bolt holes straddle the horizontal/vertical centrelines at 45 degrees; the two upper holes are the targets.

The robot pinches the bare pipe near X=−.090 before clocking, then holds near
X=−.0675 after sliding. Grip across the 32 mm diameter; the scripted target gap
is 24 mm under normal actuator limits. It never grasps the elbow/branch.
The other arm grasps each bolt, aligns it with a hole and pushes horizontally.
Fresh reset restores the intact spool and two bolts; inspect/clean contact
surfaces and verify the actual bore/bolt fit. No specimen is preattached to a
robot or assisted by a joint, pose write, alignment snap or external force.

## Score and simulation limits

Final-state score: 0.4 for opposed horizontal flange axes within 7.25° and all
four matching mating-face hole centres within 3 mm in 3D. Add 0.3 per distinct upper bolt only if its axis clears both ends of both 14 mm plate bores
using the actual 32-sided shaft/bore polygons at all four plate planes, with only 50 µm numerical contact tolerance, its head bearing face is no more than 3 mm from the loose
back face (at most 0.5 mm penetration tolerance), and its tip projects at least
2 mm beyond the fixed plate. Side holes and duplicates do not count. Maximum 1.
The scorer checks final geometry; actual continued bare-pipe holding and support must also be verified in the demonstration contact trace/video. Held parts can score; no release or load-bearing bolt retention is implied.
Measure hole alignment, head gap and projecting tip from calibrated close-up
front/side video or gauges on the real fixture; no simulated coordinates are
needed to apply that physical rubric. Thirteen offline tests cover false centre-only
seating, skewed bolts, duplicates, orientation, depth and partial credit.

`build_collisions.py` generates original analytic rigid envelopes and an area
check for 312 convex plate triangles, with true through holes. Inscribed 32-sided
hole polygons reduce radial clearance by at most 0.034 mm. The elbow uses two
intersecting annular envelopes; its internal flow passage and molding detail
are simplified. Pipe/flange stiffness, solvent joints and the steel rests are
rigid. Branch torque uses the component-mass centre estimate; compound-body
inertia is approximate. No sparse printed prop or mass ballast is used.

Assumed static/dynamic friction: spool 0.4/0.3, rests 0.25/0.2; bolt/default and
robot-pad friction come from the installed runtime. These are unmeasured. CPU
PhysX 480 Hz, 20 Hz control, 64 position/16 velocity iterations and CCD. No support
follows the pipe. Set `FLANGE_DIAGNOSTICS_DIR` to a new output directory to save 20 Hz contact/pose records at close. Contact diagnostics record the actual spool/rest and
spool/gripper colliders; ordinary actuator commands create all assembly forces.
The scripted policy uses measured simulated poses, not supplied to Dream.

## Reproduce and local evidence

Validated runtime: Isaac Sim 6.1.0.0, Isaac Lab 17.0.2, Arena 0.3.0,
YAM Arena 0.1.0 and Inspect Robots 0.60.0.

In the existing environment, regenerate assets with NumPy/SciPy and run
`python -m pytest -q test_score.py`. Use `run_local.py` to register this source for a single process without changing
the installed environment, or install its existing `yam_flange`/`yam_flange_scripted`
entry points. The command below uses the single-process registration:

```sh
python run_local.py run --embodiment yam_flange --policy yam_flange_scripted \
 -E task="$PWD/task.py" -E object_jitter=0 -E device=cpu \
 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
 --instruction 'Hold the supported pipe joint aligned by gripping the bare pipe, and insert two bolts horizontally in the two upper flange holes.' \
 --scorer task_score --grader none --no-prompt --no-rerun --seed 0 \
 --max-steps 2700 --store-frames --log-dir <new-output-dir>
```

For the matching baseline substitute `--policy dream -P model=dreamlabs/dream-1-beta`
and use a new output directory. Both policies receive the same geometry,
physics, cameras and 2,700-action limit.

| Current local evidence | Score | Actions | Simulated time | Wall time |
| --- | ---: | ---: | ---: | ---: |
| Scripted demo006 | **1.0** | 2,321 | 116.05 s | 442.50 s |
| Matching Dream001 | **0.0** | 2,700 | 135 s | 471.69 s |

Both trials completed without errors. Dream moved parts but did not align the
joint or insert a bolt. Both bolts remain seated after the working
hand withdraws. Final head gaps are about 0.916 mm and 0.299 mm. The 20 Hz
trace records both holding fingertips on the bare pipe and both moving rests
in every sampled insertion/push frame, with no working-hand/pipe or arm-to-arm
contact. This sampling does not prove contact at every physics substep.

The frozen runtime revision is
`1aed237f5899e16d92caf05db489a0831b8f0901f12d552ccbffef0a99c2ab29`.
Exact result hashes and scope are recorded in `validation.json`. Review
media **M-YNZ89J** contains the matching model run, full-score demo and setup
image. The demo and setup image combine synchronized station and oblique
fixture views of one trajectory; the model video uses the native station view.
Physical construction and force/friction qualification remain unperformed.


A paired holding check uses the same environment with scripted policy option
`-P release_probe=true --max-steps 700`: after 22 s of clocking/holding it opens the
left gripper, then withdraws it to ready while the right stays parked. This is
an explicit controller ablation, not a successful task demo. It uses normal
motor commands and no object pose/force writes. Compare flange roll/alignment
and support/hand contacts before and after release to check the branch load.


The fixed 1280 × 720 oblique fixture camera records the same live trajectory when
`FLANGE_DIAGNOSTICS_DIR` is set, including the whole spool and branch. Its eye
is (.60, −.70, 1.85) m, target (.15, .04, .88) m, 20 mm focal length, 20.955 mm aperture.
It is a diagnostic scene sensor; the model still receives only the standard
three 640 × 360 RGB streams. Encoder output and 20 Hz contact/pose trace are saved
at environment close. The controller smooths measured grasp offsets and uses
joint integral correction bounded to 0.08 rad, with unchanged physical gains
and effort limits. Pose feedback tracks the pipe axis at the bare-pipe grasp, rather than the asymmetric bounding-box centre or the flange tip beyond a support fulcrum. Failed grasps stop the scripted trial.


The insertion gripper approaches 27° downward from horizontal, placing its
palm above the pipe. Its jaws close transversely across the bolt head. Pickup
uses this same orientation. The scripted controller centres the grasp on the
observed settled head, then smoothly carries that correction through the lift.
It measures the rigid grasp transform and corrects a small bolt-axis tilt during
approach using ordinary joint commands. Failure to lift, a reversed grasp,
post-lift tilt above 10°, or excessive slip stops the trial.

Insertion has two stages. First, the held bolt is advanced through both plates,
leaving the head about 9.5 mm from the loose flange. This intermediate position
receives no bolt credit. Before opening the fingers, a separate controller gate
requires the shaft to fit all four bore planes and project beyond the fixed
plate. The gripper then backs away, closes, and pushes the head with its normal
finger surfaces. The ordinary final scorer still requires a head gap of at most
3 mm. The controller checks that strict score before clearing the head and
moving to the next bolt. No washer compression is represented. This sequence
is demonstrated by the local full-score run described above. Both policies use the same frozen runtime inputs.

The bolt shaft uses 32 solid convex triangular prisms. Native cooking audits
found that a single small-radius hull was reduced from 96 to 20 vertices; the
sector representation preserves the specified 32-sided, 12 mm envelope. Small
tip-chamfer hulls were also distorted by cooking, so that detail is omitted
in favour of a full-diameter blunt collision/visual proxy. Scoring still checks
the full shaft cross-section and both plate bores. No bore enlargement or
scoring-tolerance relaxation compensates for cooking.
