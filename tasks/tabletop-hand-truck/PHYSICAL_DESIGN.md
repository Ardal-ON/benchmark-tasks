# Physical design: full-size Wesco Mini Mover 220646

This is an original rigid-body proxy of an actual compact hand truck at **1:1 operating size**, on the supplied floor. The user-authorized smaller robot platform supports the original arm mounts and camera stand; stock motors, room and calibrated cameras remain unchanged. Published product dimensions anchor the model; internal geometry and mass distribution remain estimates. The folding nose/wheel linkage and telescoping handle are fixed in a deployed, latched state. Folding, latch operation and rated-capacity testing are outside this task.

## Published geometry and weight

The [manufacturer's current 220646 listing](https://catalog.wescomfg.com/item/platform-trucks-carts-dollies/plastic-platform-lightweight-folding-trucks/220646-1) specifies the following:

| Quantity | Actual product value used in the simulation |
|---|---|
| Extended operating height | 41 in = 1041.4 mm |
| Overall width and depth | 15.5 × 17.5 in = 393.7 × 444.5 mm |
| Nose plate | 15 × 12.5 in = 381 × 317.5 mm |
| Two wheels | 5 × 1.25 in = 127 × 31.75 mm; rubber tread and poly hub |
| Nominal current unit weight | 9 lb = 4.08233133 kg |

The [manufacturer-linked drawing](https://catalog.wescomfg.com/Asset/220646.pdf) includes photos and a specification list, not dimensioned internal CAD. It and the [2019 manufacturer-hosted catalog](https://www.wescomfg.com/wp-content/uploads/2019/08/SMO.pdf), page 4, list **7.25 lb**, conflicting with the current 9 lb listing. The higher current value is chosen explicitly. Neither value is represented as the measured weight of a physical test unit. The 110 lb capacity is not evaluated. The listed **24 in height is folded storage height**, not the operating height used here.

The manufacturer's aluminum construction, rubber tread/poly hub, telescoping handle, folding wheel/nose arrangement and stored elastic straps inform the original proxy. No product mesh, logo, proprietary CAD or reference photo is redistributed in this task.

## Estimated geometry and mass distribution

The **63.5 mm axle-to-frame offset is inferred**, not supplied by a dimensioned drawing: 444.5 mm overall depth minus 317.5 mm toe depth minus 63.5 mm tire radius, assuming the tires define the rear extremity. This assumption materially affects balance and reach and needs measurement before hardware reproduction.

Original rectangular hollow profiles approximate the frame: lower rails have 22 × 28 mm outer cross-sections and 550 mm length; the inner handle profiles are 16 × 20 mm. Their centers align, with 125 mm overlap in the deployed state. An effective common wall near 1.881 mm closes the selected 9 lb mass budget at estimated aluminum density 2700 kg/m³. Hollow sections determine inertia; their native collision shapes use outer envelopes. Plastic lower handhold and upper housing, the rubber top grip, hinge/wheel mounts and their positions are approximations of the referenced product, not measured component dimensions. Curved frame and cast features are simplified.

The 381 × 317.5 mm toe is an original open grid of side beams, transverse webs and underside ribs, with visible gaps. Member placement and thickness are assumed. It is not a solid invisible support plate. An estimated 8 mm circular steel axle reaches through both wheel hubs. Stored straps are noncolliding visual rigid proxies that contribute estimated mass; no strap supplies an elastic force. The deployed rigidization substitutes for unmodeled hinge locks and telescope latches.

Only the total unit weight is published. Component masses use estimated densities of 2700 kg/m³ aluminum, 7850 steel, 950 polymer and 1100 rubber, plus the mass-budget profile wall. Wheels use a rubber annulus outside an assumed 45 mm hub radius and a polymer hub with an assumed 8 mm bore. Each resulting wheel is approximately **0.410605 kg**; this is a derived estimate, not a manufacturer component weight. The remaining rigid body is **3.261121 kg**. Their sum is 4.08233133 kg without geometric or cubic mass scaling.

`build_assets.py` integrates centers and full inertia tensors with the parallel-axis theorem. Wheel inertia uses the two annular material regions. [physical_design.json](physical_design.json) records every component, geometry, mass, center, inertia, the compound tensor and assumptions. Rubber compliance, hub webbing, bearing resistance, frame deformation and material failure are unmeasured. Tires use authored 120-sided convex cylinders (240 vertices), with an explicit 255-vertex hull-cooking limit. The [PhysX schema](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/schemas/physxschema.html) defaults convex hulls to 64 vertices, which can coarsen an unauthored limit. Orange index marks are visual; negligible paint mass is omitted.

Each wheel's dissipative bearing drive has zero stiffness, zero position/velocity targets, damping 2e-7 in native USD angular-drive units and a 0.001 Nm maximum torque. The controller never propels wheels. Estimated static/dynamic friction is 0.5/0.4 for metal, 0.9/0.7 for tires and the assumed rubber handle, and 0.4/0.3 for polymer, with zero restitution. The resized platform retains the station material. Robot and cart collision contact margins are 0.2 mm with zero rest offset; robot motors retain stock gains and limits. These are modeling assumptions, not measured material pairs.

## Floor layout and fixture

The task relocates an empty cart. The stored straps are not engaged.

The empty cart rests on the original floor, Z = -0.0125 m, in front of the arms. Nominal pre-grasp axle is (0.500, 0, 0.051) m with toe toward +X; parking mark is centered at X = 0.350 m. The left grasp target is local (0, +9, 1027.9) mm on the handle; the right is (0, -15, 562.1) mm on the upper brace. Brace positions and cross-sections are estimates. TCP advances of 8/12 mm put the bars inside the finger pads. The wrists roll to align each jaw closing axis perpendicular to its crossbar. Native settling lasts 2 s; actual acquired poses are recorded.

Gripper position commands are 0.20 (left) and 0.35 (right), corresponding to nominal 18.78 mm and 32.865 mm open gaps under the supplied transmission model. These gaps are below the modeled bar cross-sections to retain contact during wrist deflection. Clamp forces depend on native contact and servo response and have not been measured on hardware. The stock 50 N finger effort caps remain unchanged. Gravity feedforward derives joint torques from the supplied YAM URDF masses and COMs, then divides by the unchanged position gains to form joint-target offsets. A 20 mm downward joint-target position preload and paired lateral offsets encourage tire contact. All motor force/velocity limits remain stock; native contact impulses and realized body motion determine the result. No object force, motor gain increase or cart/wheel pose write drives the episode. Folding and latch operation remain outside this rigid-deployed model.

The U-shaped robot platform is 390 × 860 × 60 mm, centered at (-0.125, 0, 0.7175) m; its top remains Z = 0.7475 m. It spans X = [-0.320, 0.070] m and Y = [-0.430, 0.430] m. A 220 × 860 mm rear bridge spans X=[-0.320,-0.100] m. Two 170 × 240 mm mount pads span X=[-0.100,0.070] m and Y=[0.190,0.430] m or [-0.430,-0.190] m, leaving a 380 mm center opening. All pieces are 60 mm thick and rigidly anchored. Robot bases remain (0, ±0.305, 0.750) m. Native mesh measurements of the mounting soles show both complete footprints inside the platform, including the roughly 201 mm lateral extent of each base. The fixed camera stand foot is also supported. An anchored rigid platform and the original mount/spacer arrangement are assumed; structural load capacity is not tested.

This is a short equipment-staging segment using an actual full-size compact device and its real handle height. Published envelope dimensions remove geometric miniaturization. The internal proxy, balance/forces, material response, physical construction and hardware safety remain unmeasured. The README describes a video/ruler evaluation method and its limits relative to native contact detection.
