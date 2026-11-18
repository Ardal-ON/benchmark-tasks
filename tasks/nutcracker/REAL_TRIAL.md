# Physical figure, specimens and scoring

This revision chooses the reviewer's **dimensioned fabrication** option. It does
not identify a decorative retail soldier as a tested walnut-cracking device.
The fabrication below is an original metal-reinforced figure, not a commercial
product replica. It is specified but has not been built or load-qualified here.
The simulation demonstrates robot manipulation of its external geometry;
it does not establish material strength or biological fracture accuracy.

## Coordinate convention and fabrication

All dimensions below are millimetres. The figure frame has X left/right, +Y
out of the face, and +Z up. Its origin O is 120 mm above the table. The hinge
axis runs along X through (0,20,0). The rear lever and lower jaw form one rigid
assembly rotating from −45° (open) to +3° (closed), with the same zero frame as
the fixed body. In the station rotate the entire figure +90° about Z, putting
O at (320,−40,867.5) mm. Its face therefore points in station −X. The fixed
130 × 130 × 12 base sits on the 747.5 mm table. The outer decorative envelope
is completely specified in `fabrication-envelope.csv` and `build_assets.py`;
CSV rows give body, shape, centre, dimensions, and whether the surface collides.
These are outer envelopes, not instructions to make overlapping solid blocks.

Load path and fabrication, using 6061-T6 aluminum and stainless fasteners:

| Part | Stock and dimensions | Manufacture / attachment |
|---|---|---|
| Base | 130 × 130 × 12 aluminum plate | Four Ø6.5 through-holes at X/Y = ±50; M6 through-bolt with washers to a sacrificial table plate, or clamp that plate outside both arms' sweep. Base centre = figure X/Y origin. |
| Internal fixed spine | 30 wide × 8 thick × 150 high plate, centre (0,−22,−25) | Bottom at Z −100; two M5 bolts at X=±8 into base brackets. Upper end reaches Z+50. Mill a through-window X−7..+7, Z−23..+38 through the 8 mm thickness for the complete rear-link sweep, leaving two 8 mm side rails; radius internal corners R1. The link spans X±6, so has ≥1 mm lateral clearance. Two 30 × 30 × 3 angle brackets at bottom, each fixed by two M5 bolts to base/spine. Cosmetic body is relieved around spine. |
| Upper jaw (UpperLip) | 60 × 50 × 8 plate, centre (0,40,34), underside Z30 | Two M5 countersunk screws at (±18,22) in the jaw's XY plane to an aluminum angle bracket from spine. No screw protrudes below underside. Round outer edges R1; underside remains flat. |
| Hinge supports | Two 60 deep × 25 high × 6 thick cheeks | Inner faces X=±30, centres (±33,20,0), spanning Y−10..50/Z−12.5..12.5; Use a Ø6 × 92 hardened steel D-shaft along X through Y20/Z0 (one 0.5 mm-deep flat). Ream cheek bushings Ø6 H7. Broach/mill the matching D-bore in the 59.6 mm-wide integral moving boss; the axle rotates with the jaw inside fixed cheek bushings. Fit 0.2 mm PTFE thrust washers either side of the boss and retaining collars outside cheeks, centred X±38 and 3 mm thick (inner faces X±36.5). The shaft ends X±46; right crank occupies X40..44, retained by a circlip at X44.5. Use all-metal collars and captive grub screws on the flat. Two transverse backplate wings are each 29 wide × 6 thick in Y × 25 high in Z, centred (±21.5,−13,0), spanning X7..36 and X−36..−7. The 14 mm central gap clears the moving link throughout travel; front faces Y−10 meet cheek rear faces. Drill two Ø5.5 clearance holes per cheek in backplate at X=±33/Z=±7.5, axis Y, and M5 tapped holes 12 deep into each cheek end at matching locations. Backplate rear face Y−16 attaches to spine front face Y−18 via four 2 mm tubular spacers and M5 through-bolts at X=±11/Z=±7.5. No metal overlaps. Cosmetic shell is relieved around all fixed metal. |
| Moving lower jaw and hinge boss | One machined 6061 billet, initial stock 59.6 × 61 × 12 | Unite the 59.6 × 60 × 8 jaw envelope centred (0,45,0) with the 59.6 × 12 × 12 boss centred (0,20,0) into ONE machined part; do not bolt two overlapping solids together. Mill to that union profile, machine the Ø6 D-shaped hinge bore (0.5 mm flat) along X at Y20/Z0, and chamfer bore ends 0.2. Usable flat jaw top is Z4 for Y26..75; the integral rear boss rises to Z6 for Y14..26. Top-down M4 countersunk screws at X±10/Y35 attach to the rear-link tabs described below, not to the cosmetic carrier. Three 58 × 2 × 3 ribs centred Y25/40/55, Z5.5, and two 10 × 5 × 16 front stops centred (±14,71,12) are machined integrally from extra local stock or screwed from below with M2; the Y25 rib is united/trimmed into the boss top where they overlap. Add two integral 4 × 50 × 12 side guides centred (±27.8,40,10), topZ16, outside facesX±29.8, leaving51.6 mm clear between inner faces. They end at Y65 to clear the loading fingers behind their tips. These form a shallow walnut seat with the front stops, preventing lateral rolling during release; they are not grasp handles. Machine jaw and boss to the same 59.6 mm width, leaving 0.2 mm running clearance to each cheek at X±30; maintain this clearance through the full sweep. Round exposed edges R1. |
| Carrier cover (JawPivot) | PETG hollow 28 × 60 × 12 outer envelope, centre (0,−25,0), 1.5 wall | This is a cosmetic cover, not the jaw load path. Cut away the central bottom 12.2 mm-wide strip for the rear link; split the cover at X0 for printing and use two M3 cross-bolts at Y−45/−10 with 12.2 mm-long spacer tubes to clamp it around the aluminum link. All load passes from link tabs to the integral jaw/boss. Relieve the cover around the full hinge and stop sweep. |
| Rear link | 12 × 12 × 180 aluminum square tube, 1 wall, centre (0,−35,−12) | Extends Y−125..55 and Z−18..−6; its top is flush with the boss bottom Z−6 and lies 2 mm below the jaw underside Z−4. The angle tabs bridge that 2 mm offset. Fit two 20 × 20 × 3 aluminum angle tabs, each 20 mm long along Y centred Y35, on the left/right tube faces: vertical legs at X±6 and horizontal legs extend outward from X±6 to X±23, top face Z−4 and bottom Z−7; they do not extend inward into the tube. Attach vertical legs with two M4 cross-bolts through tube at Y30/40 and Z−12, using internal 10 mm-long crush sleeves. Drill matching M4 jaw holes at X±10/Y35 into horizontal legs with captive steel M4 nuts below, for ≥4 mm thread engagement. Countersink jaw-top heads flush. Cut tab corners R2 and deburr; they stay below the jaw top and behind the front lip. |
| Rear vertical lever | 18 × 18 × 100 aluminum square tube, 1 wall, centre (0,−120,−10) | At Z−12 mill 12.2 × 12.2 apertures through the front/back walls to receive the horizontal tube; insert it up to Y−125 and bolt to a 30 × 30 × 3 side angle with two M4 bolts and internal crush sleeves. Tube centre lines meet at (0,−120,−12); no solid walls overlap. Top Z40. |
| Grip | 80 × 24 × 24 PETG hollow block, 2 wall, centre (0,−120,40) | Clamp around top of lever using two M3 through-bolts; texture with 0.5 mm grooves. Grip is part of moving assembly. |
| Moving stop crank/pad | 4 mm-thick 6061 crank plate in X40..44 | Crank joins hinge point (42,20,0) to pad centre (42,−10,−19.5); centre-line length35.78, width6 in its YZ plane, plate thickness4 along X. Make the pad end a 4 × 10 × 3 block, united with the crank profile, centred at (42,−10,−19.5). Broach a D6 hub in a Ø14 boss at the hinge end and retain on the D-shaft with a circlip. The crank rotates with the lower jaw, entirely outboard of cheeks/backplate/spine (fixed metal ends at X36; crank startsX40). Remove the right mouth side outside X39 and relieve right collar X39..45 so all cosmetic surfaces clear the crank by ≥1 mm; the CSV/generator include these reliefs. Only the metal pad receives limit loads. |
| Fixed hard stops | Two M4 adjustable steel ball-tip screws and lock nuts | At −45° place open-stop tip at (42,−17.48,12.02), pointing down along −Z. At +3° place closed-stop tip at (42,−13.85,−22.80), pointing up along +Z. These are the pad's upper/lower rear edges, calculated about hingeY20/Z0. Use two 3 mm aluminum screw-holder L brackets at Z22 and Z−35, 20 × 20 plates with M4 threaded holes at the specified tipXY; attach them to an outboard 20 × 3 upright centredX50 (inner faceX48.5). Fix upright to right cheek using two M4 × 20 screws alongX at Y0/Z±6, through12.5 mm standoffs and into4.5 mm tapped cheek engagement. Holder arms run from X50 to42 only at Z22/−35, outside the swept crank/pad except the adjustable tips. Set screw free tips to the stated coordinates (~10/12.2 mm projection), verify actual −45°/+3° angles with a protractor and lock nuts. Limit loads pass through metal crank/shaft/jaw, never PETG. The simulator uses revolute limits for these small fixed screws; moving crank/pad are explicit colliders. |
| Figure exterior | PETG 1.5 mm walls, 15% infill only at bosses | Print red jacket, blue legs, flesh-colour face, paired sleeves/hands and navy hat using CSV envelopes; unite overlapping volumes per body, hollow inward. M3 screws fix to spine and base; decorative trim may be painted. Remove internal material intersecting the mechanism sweep plus 1 mm clearance. Mouth recess stays 60 wide, from Y8 to68; do not bridge it with print supports. |

Use threaded inserts at plastic bosses. Do not rely on adhesive to carry jaw
loads. Remove burrs, round exposed edges, verify free travel by hand, and check
that both jaw contact faces remain parallel at zero angle. Add captive washers
and prevailing-torque nuts so the hinge cannot unscrew during a trial. Target
fixed upper assembly mass 450 ±25 g and moving assembly 225 ±15 g; weigh each
separately. Remove internal cosmetic infill if heavy, or add captive steel
washers inside the jacket/grip if light; do not change any contact envelope.
These are build targets, not measured fabrication masses. Material tubes are
represented by solid outer collision boxes with assigned total masses in sim.
Internal brackets, bores, bolts and stop screws are omitted from collision
geometry. The hinge cheeks/backplate/integral boss are represented explicitly by
colliders; overlapping per-body CAD boxes are united/relieved in fabrication.
Small fasteners and cosmetic wall thickness remain geometric approximations.

Before robot use, bolt the fixture down, cycle unloaded twenty times, then
apply 300 N slowly between jaws using a force gauge and a sacrificial wooden
block, behind a clear guard. Reject loosening, cracks, >0.5 mm permanent jaw
movement or hinge play >0.5 mm. Check stop angles, then try ten selected nuts
with measured handle force. Record results rather than assuming qualification.
This load check has not been performed. A 59.6 mm-wide, 8 mm-thick jaw modelled
as a 50 mm cantilever has nominal bending stress 6FL/(bt²)≈23.6 MPa at 300 N;
that calculation excludes fasteners, local stresses and fatigue, and does not
replace the fixture check. Use eye protection and a transparent containment
screen outside the robot sweep; never insert fingers during a robot trial.
Shells and kernels from this fixture are test material, not food.

## Walnut supply and reset

Use one lot of raw, dried **English/Persian walnuts (Juglans regia), in shell**;
exclude black walnuts. A named retail source is [Nuts.com English Walnuts in
Shell](https://nuts.com/products/english-walnuts-in-shell?variant=67344289529878),
whose listing describes raw in-shell English walnuts. It does not promise a
cultivar: record supplier, lot, harvest/packing information and cultivar if
provided; do not label the lot Chandler without evidence. Use the same lot for
comparisons. Purchase extra for screening and fixture qualification.

Condition unpacked in a dry room for 48 hours at 20–24°C and 40–60% relative
humidity; record temperature/RH and storage time. This standardizes exposure,
not internal moisture content. Do not soak, roast, score, precrack, glue or
soften shells. Brush off loose material without water. Photograph each whole
shell from four sides and reject visible fissures, missing shell, mold or
insect holes. Select nuts with two equatorial diameters 34–38 mm and 30–34 mm,
pole-to-pole 34–38 mm, mass 12–16 g (calipers, 0.1 g scale). Record all three
axes and mass. The sim ellipsoid is 36 × 32 × 36 mm and 14 g, not a promise of
exact biological shape. Track rejected specimens and never reuse a loaded or
squeezed walnut in a later trial.

At reset set the lever to zero angle; place one intact walnut at table XY
(200,130) mm, with the long axis parallel to the table, the visible natural
suture horizontal, and thickness direction vertical (the simulated initial
halves are stacked in Z). Allow it to settle without a cradle. Photograph the
actual settled centre and orientation; the original pose numbers in older
media are approximate settled values. Seed 0 uses this fixed starting layout.
Empty the jaws, remove all fragments, inspect hinge/bolts and replace the nut
for every trial. Do not rebuild or reglue real shells to mimic the sim seam.
Reset both robot arms to the ready configuration: six arm joints (0,1,1,−0.6,0,0) radians and open grippers. Pinch the walnut from its sides for insertion, with the fingers closing left/right across the mouth and approach along its forward axis; a top/bottom pinch puts fingers in the closing-jaw path. The scripted demo uses measured hand-to-walnut offset after turning to place its centre and a softened loading grip. It opens only to about38mm while in the60mm mouth, withdraws70mm, then opens fully and parks the loading arm before the cracking press; spreading the fingers to94mm inside the mouth would collide with the guides.

## Practical real-trial score

Record 20 fps or faster synchronized front-oblique and side video. Include a
millimetre scale fixed beside the mouth in the same depth plane as the nut,
a 10 mm witness grid at base/hinge and a 5 mm grid on the table. Record the
starting intact shell close-up. Keep at least one camera's view of the mouth
unoccluded at the squeeze and both views during the final hold. If contact or
opening is occluded, the trial is unscorable and must be repeated with a fresh
nut; an occluded trial is not automatically a success or a failure.

Establish a **compression and custody witness**, using the continuous video:

1. Both designated jaw surfaces contact the intact shell while the lower jaw
   visibly closes at least **1 degree**. Use the external crank/hinge reference
   and a protractor or calibrated video overlay; contact without closing is
   insufficient. The loading gripper and all other robot parts must be clear
   of the shell. No force sensor is needed for this score witness.
2. From that compression until a new crack is documented, keep both shell
   pieces in the mouth custody region: their visible outer-bounding-box
   midpoints remain within figure-frame X ±45, Y −5..105, Z −55..55 mm relative
   to O. The figure face, hinge and ruler provide the reference in front/side
   views. No intervening contact with a gripper, other robot link, table or
   another object is allowed; only the designated jaw assembly and the other
   shell piece may contact the shell. The jaw assembly includes the plate,
   ribs, front/side guides, integral rear boss and mouth-roof underside.
   Leaving the region or other contact invalidates the witness irreversibly
   until the next trial with a fresh nut. Re-insertion cannot restore it.
3. Document the first measurable new fissure (normal opening ≥0.5 mm), and its
   first opening to **≥3 mm**, while that custody/exclusion chain remains
   intact. They may become visible when the lever is lifted after the squeeze;
   there is no arbitrary time limit between compression and reveal. A fissure
   or 3 mm opening observed before a valid witness cannot be qualified later
   by pressing the already cracked shell in the jaws. Tangential sliding alone
   does not establish a 3 mm open fissure. The first threshold observations
   and whether the witness was valid at them are irreversible.

Record irreversible visible damage, then score the **final uninterrupted two
seconds** of released containment. A documented crack can close on unloading:

* **1.0:** The compression/custody witness and first 3 mm opening are valid.
  Both shell pieces remain on the table or in the mouth, none below the table
  or more than220 mm above it, and the fixture witness marks move <10 mm.
  Both pieces stay within2mm translation and2° orientation of their final-hold starting poses; both grippers open ≥70 mm and remain ≥80 mm from shell, figure and grip for the final2s. A prior crack need not stay wedged open during that final hold.
* **0.25:** No fissure ≥0.5 mm was ever visible; shell is supported by the lower
  jaw and remains seated for the final2s. Its midpoint is within ±22 mm of the
  jaw's left/right centre,25–65 mm forward of moving-body origin and10–40 mm
  above it. Apply the same fixture/containment/open-gripper/clearance checks.
* **0:** Every other observable outcome, including direct gripper crushing,
  first crack outside valid custody, later insertion of precracked pieces,
  new crack from table impact, fixture displacement, drops or an incomplete
  released hold. Ambiguous/occluded contact or custody evidence is unscorable
  and repeated with a fresh nut, never presumed successful.

The crack itself may be hidden by the jaws during compression if jaw contact,
closing and custody/exclusion remain observable from the two views. If those
conditions cannot be resolved, reposition the cameras and repeat. This is an
operational video witness, not proof of the microscopic instant or force of
biological fracture. Use at least4pixels/mm in the mouth close-up so a0.5mm
opening spans at least2pixels. Smaller or unresolved fissures are a disclosed
observation limit, not a claim of intact shell material.

Annotate timestamp of first crack and contact views, maximum opening measured,
final hold start/end, seating/containment and release checks in
`real-trial-record.json` (template below). Two reviewers resolve a borderline
3 mm opening from the side close-up before assigning a score. No shell-root coordinates, native seam event or force sensor are needed by
physical annotators. Simulator reference points approximate visible piece
bounding-box midpoints; normal rim separation approximates fissure width.
The compression witness uses actual jaw-surface contacts and measured closing
angle, with no robot/other-object contact and continuous mouth custody before
both first thresholds. This avoids credit from a break event alone. The two
rigid convex halves and attached visual kernel remain geometric approximations.

The simulation checks clearance conservatively with150mm TCP-centred spheres
enclosing the complete YAM gripper/finger meshes and local outer boxes for all
visible figure pieces. A real pose can pass80mm surface clearance but fail this
conservative proxy. Shell support requires an actual contact on the table or lower jaw assembly
(including side/front guides), with impulse-derived contact magnitude>0.005N
within50ms. Vertical side contacts can support by friction. To avoid treating a
transient graze as rest, both pieces must stay inside a2mm/2° pose envelope for
the entire2s final hold. An intact shell transfers support structurally; broken
halves transfer it only through a contemporaneous pair contact (within two
physics ticks of support). This contact-plus-rest criterion does not claim an
exact measured force balance or calibrated material friction. Both pieces
stay within the1100×1300mm table centred at(360,0)mm, conservatively allowing a
26mm piece radius. Reconstructed hinge positions check mount stability without
mistaking normal jaw rotation for displacement.

```json
{"supplier":"", "lot":"", "cultivar_if_known":null,
 "axes_mm":[], "mass_g":null, "conditioning_hours":48,
 "temperature_C":null, "RH_percent":null, "intact_start_photo":"",
 "front_video":"", "side_video":"", "first_opening_time_s":null,
 "opening_mm":null, "opposing_contacts_visible":null,
 "no_gripper_shell_contact":null, "seated":null,
 "contained":null, "fixture_movement_mm":null,
 "gripper_opening_mm":[], "minimum_clearance_mm":null,
 "final_hold_s":null, "score":null, "annotation_notes":""}
```
