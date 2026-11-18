# Physical design: tabletop hand-truck replica

This is a user-requested miniature prototype for the unchanged YAM station table, not a full-size material-handling truck. The arms, table, room and calibrated cameras remain the stock station. Its physical build and contact parameters are not qualified on hardware; simulation and model results are recorded in the README.

## Reference and dimensions

[Magliner 116-UA-1030](https://www.magliner.com/116-ua-1030) lists a 48 in (1,219.2 mm) tall, 18.5 in wide, 18 in long hand truck weighing 24 lb, with an 18 × 7.5 in nose and 10 in wheels. Its [101030 wheel](https://www.magliner.com/10-inch-ballon-cushion-wheel-101030) is 10 × 2 in and 3 lb each. Uniform geometric scale 300/1,219.2 = 0.246062992 gives a 300 mm reference-height body, 115.625 mm reference width, 112.5 mm reference length, 112.5 × 46.875 mm nose, and 62.5 × 12.5 mm wheels. These are scaled reference dimensions, not proof of an exact replica: the simplified rails, braces, straight axle, brackets and nose are a buildable approximation, not the original castings and handle profile.

The added grip sleeve projects above the reference body, giving a 309 mm simulated overall height. See [dimension drawing](physical_design.svg) and [physical ledger](physical_design.json). The drawing is schematic, with dimensions controlling rather than drawing scale.

## Material and mass ledger

| Part | Construction assumption | Mass |
|---|---|---:|
| Two vertical rails | Aluminum, 284 mm long each, 8 mm outside / 6 mm inside diameter, 2,700 kg/m³ | 33.726 g |
| Four cross rails | Same tube, 74 mm each | 17.575 g |
| Nose | Aluminum plate 112.5 × 46.875 × 2 mm | 28.477 g |
| Axle | Steel, 103.125 mm long × 4 mm diameter, 7,850 kg/m³ | 10.173 g |
| Two axle brackets | Aluminum, each 8.536 × 34.375 × 20 mm | 31.690 g |
| Grip sleeve | Added foam sleeve, 60 mm long, 26 mm outside / 8 mm bore, 350 kg/m³ | 10.094 g |
| Two wheels | Each 62.5 × 12.5 mm; cushion-rubber and hub approximated as rigid wheel | 40.547 g |
| **Cart total** | Reference body 162.187 g plus added sleeve 10.094 g | **172.281 g** |
| Loose load | Wood, 60 × 35 × 100 mm, 500 kg/m³ | 105 g |

The 162.187 g reference-body target and 20.273 g per wheel derive from the manufacturer's total and wheel weights using the cube of the linear scale. The 8.536 mm bracket thickness is a calculated mass-budget closure, not a manufacturer dimension; all tube sizes, nose thickness, bearing details, foam density and simplified frame shape are design assumptions. Sleeve mass is additional, not hidden in the reference scaling. The ledger gives each component centre and inertia; its combined frame centre of mass is (0, 1.723, 95.134) mm in frame coordinates. The frame inertia is approximately diag(0.0014710, 0.0015652, 0.0001858) kg·m² with YZ product -0.00000569 kg·m²; wheels are separate bodies and use their own inertias.

## Build and fidelity

Build the frame from the specified tubes, plate, axle and brackets; mount both wheels on passive bearings and fit the sleeve. The rigid simulation uses outer collision envelopes for hollow tubes and approximates each cushion wheel by a rigid 64-sided convex cylinder with the scaled wheel mass. A painted visual index stripe supports rotation measurement; paint has no collision and negligible omitted mass. Each bearing has a zero-target, zero-stiffness dissipative angular drive (damping 2×10⁻⁷ in USD angular units, maximum torque 0.001 N·m); no wheel rotation is commanded. Specify and test the real wheel bearings, grip, contact friction and load position before physical use; friction, restitution and structural behavior are estimates. Cardboard/wood deformation, bearing compliance and tire deflection are not represented. This small tabletop apparatus changes reach, forces and handling relative to a floor-standing hand truck. Its relevance to industrial material moving is therefore unproven and subject to the benchmark miniaturization rule; the user has authorized exploration of the prototype.
