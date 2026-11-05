# Mill and mineral-cast machine design

Use **`materia.project.json`** as the main project. Its `small` entrypoint is the
Small configuration with the selected Y drive; `compact-vmc` is the Compact VMC
layout with the same complete Z and spindle assemblies, including drawbar and
pneumatic release. `spindle-drive` is the editable motor study. The older bench
mill and enclosed cell belong to their own project. Shared model code and build
tooling remain in this directory.
Both sizes share `MillFramePreview.assemble` and `MillGuidedXYStage`: one nested
Y-saddle/X-table assembly, ribbed saddle geometry, parameterized T-slot table,
rail/block placement and catalog-sized saddle bolts. Small attaches its existing
XY drives through `MillXYStage`; Compact VMC supplies its HG25 geometry and larger
stack dimensions through `MillCapacityLayout`. Compact VMC retains its original
travel and table/spindle datums. Its XY drives remain unfitted, and it has no
Small motor pockets, nut seats or implied drive ratings.

Run `haxeon.json` to check and generate `build/mill/small.mtrg` and `large.mtrg`.

Open the source project and use the **Configuration** selector to choose **Small**
or **Compact VMC**. Stop simulation before switching. Opening a generated `.mtrg` file gives a single scene and does not expose the project selector.

Small and Compact VMC include **Machine bearing block**, a named CNC mission. Play
runs the shared bearing-block program once; the Mission panel follows its actual
CNC execution state and the CNC panel provides feed hold, resume and program
inspection. Both configurations use a shared ZPP AR155-I pneumatic vise and manual TS96-155
zero-point plate, with supplier STEP geometry and a tapped table adapter. Jaw
motion is geometric; air ports are exposed for a future verified pneumatic binding. Stock seating, work offset
and effective tool length derive from the table, vise and BT30 spindle. See
[WORKHOLDING.md](WORKHOLDING.md) for dimensions, source and reconstruction limits.
Open **`materia.bench.project.json`** for the independent **Bench mill** project,
with **Open bench mill** and **Enclosed cell** configurations. The enclosed cell
adds its pneumatic vise and door. Its bearing-block program is also a named,
single-pass mission. The legacy standalone entrypoint uses the same mission identity and finite execution.
The Start page has separate **Mill** and **Bench mills** cards. The spindle-drive
study belongs to the Small/VMC project. There is no Demo selector: configurations
select hardware, studies select design setups, and the Mission panel presents the task loaded with the machine.

The Small and Compact VMC missions are **kinematic machining previews**:
axis commands use explicit 20 mm/s velocity and 100 mm/s² acceleration assumptions. They do not qualify motor sizing,
cutting forces, structural performance or production capacity. The capacity-only
preview keeps its fixture envelope and contains no cutting mission.

Run `haxeon.machining.json` for G-code, datum and travel checks on both setups;
`app/tests/mill-missions/haxeon.json` exercises the actual CNC mission executor.
Its default backend is deterministic; set `MILL_MISSION_BACKEND=mujoco` to check
the desktop backend. The execution tests retain the real machine geometry and
joints while disabling general physical contacts; cutter/holder/stock interaction
is checked by the CNC kernel. These tests do not qualify machine contact dynamics.

The working legacy `BenchMill` has 250 mm X, 150 mm Y and 250 mm Z travel.
`MillFramePreview` develops a separate mechanical concept with 250 mm X, 160 mm Y and 250 mm Z travel.
Its output is a design study, not a machine released for manufacture.

The [axis motor sizing study](AXIS_SIZING.md) derives XYZ torque, speed, inertia
and Z holding loads from this mechanical model, including the alternative
Leadshine spindle payloads and explicit load/duty assumptions.

## Current geometry

`MillFrameLayout` owns casting datums. `MillXYStageLayout` and `MillZAxisLayout`
size the axis assemblies. Castings and separately itemized steel interfaces use
shared `MineralFoundation` recipes; other machine families keep their own layouts.

The reference base is 520 × 805 × 220 mm. The Y motor sits in a rear service bay beneath the column, with a continuous
60 mm casting roof and side supports. The column is 320 × 260 × 833 mm. Guide/bearing inserts stand 4 mm proud
of the casting as explicit raw finishing stock, with 20 mm embedded thickness. Core openings, walls, dimensions
and materials remain design assumptions requiring process and structural review.
The source retains nominal service interfaces separately from the raw finishing
stock and unresolved finishing-process accuracy allocations.

### Precision interfaces: DIY replication route (decided 2026-10-19)

The first machine is built without access to a mill large enough to finish the
909 mm base, so precision surfaces are **replicated, not machined**:

- **Steel stays at every bolted or load-bearing interface.** That covers rail
  and screw-bearing seats, nut/motor mounts, column clamp stations and foot
  seats. Epoxy granite cannot hold rail-bolt threads, and its resin binder
  creeps under steady bolt preload. These inserts carry threads and bearing
  area. They need only coarse placement from the mold fixture, not precision
  faces.
- **Rail and screw-bearing seats are replicated.** Each steel seat stops below
  its final mounting plane by a replication allowance. A thin layer of
  slideway replication compound is then cast against a precision master
  (surface plate, ground straightedge or ground bar) with release agent, and
  the rails bolt onto that layer. Rail holes are protected with release-coated
  plugs during replication, or drilled through the layer afterwards. Both
  rails of one axis are replicated in a single pour against one master, so
  they share one plane. The same pour can form a rail reference shoulder
  against a straightedge.
- **The allowance is set by error, not by habit.** It is the compound's
  minimum supported layer thickness, plus the expected insert height error
  from the mold fixture, plus the casting's cure distortion over the seat
  length. Until those are measured on a coupon, the layer thickness stays a
  declared assumption.
- **The column joint is grouted, not pad-matched.** The column stands on jack
  screws and is squared to the replicated Y plane by indicator. The gap at the
  four clamp stations is then filled with epoxy chocking compound, with the
  clamp bolts sleeved. Bolts are torqued only after cure. This sets Y–Z
  squareness without machining, and gives full bearing area instead of four
  pads touching at their high points.
- **Foot seats and other mounts are used as cast.** They keep their steel
  inserts. Slots and shims take up position error.

Pending, before geometry changes: choose the replication and chocking
compounds (layer limits, modulus, cure shrinkage, creep data), define the
masters and their flatness, add a replication allowance and grout gap to
`MillFrameLayout`, and run a coupon trial on a scrap casting. The geometry
change also updates `MillFrame`'s seat inserts and adds replication layer and
grout solids for the FEM packet.

Replication sets the plane only. Lateral straightness, rail parallelism and
squareness between axes come from steel side bars with push screws, adjusted
at assembly against a straightedge and a granite square: see
[../../RAIL_MOUNTING.md](../../RAIL_MOUNTING.md) (decisions, phases RM0–RM6)
and [../../METROLOGY_SOURCING.md](../../METROLOGY_SOURCING.md) (instruments and
compounds). Open code-review findings are in [REVIEW.md](REVIEW.md).

Four side-open foot pockets expose adjustment nuts. M16 foot envelopes pass
through 18 mm clearance bores in removable 60 × 60 × 15 mm roof plates. A support
nut below each plate pushes it into the casting ceiling; a separate nut locks
the adjustment. The plates install after cure, with no cast-in foot anchors or
upper stem pockets. Front foot stations sit beneath a full-height bridge;
underside cores start behind it.
Foot supplier ratings, lateral restraint, anchoring, floor suitability and insert
anchorage are unresolved. The shown pads/nuts are geometric concepts.

Y guides carry a 450 × 200 × 80 mm saddle. A compact steel plate fixes the Y nut directly to its front face; the nut and
lubrication fitting enter a 58 mm underside tunnel.
X guides, spaced 160 mm apart, carry a 400 × 210 × 40 mm T-slot table with
48 mm integral longitudinal ribs. These replace four isolated spacers and keep
the central drive corridor open. Table top remains 448 mm above the casting bottom.
The spindle datum and column/guide length retain the earlier 58 mm stack increase; the taller Z block footprint adds another 40 mm
to the column and Z guides.

The Y screw and motor share a 252 mm shaft height, at the saddle underside and
between the guides. The motor remains coaxial, in a recess through the lower
column. Its fixed bearing sits behind the Z screw. Z's lower bearing and guide
starts are raised 100 mm, with a 24 mm crossing-shaft bore in the lower Z support
plate. Z stroke, spindle position and table height remain unchanged.

The rear hatch mounts to steel anchors in the column. Allow 163 mm behind the
column rear face for modeled motor extraction (123 mm beyond the base), plus
cable/tool access. The front bearing is accessible from above. The former deep
base channel and tall nut bridge are removed; only 8 mm deep seat recesses
remain at the drive stations. The rail-side fields are lowered 20 mm. Central underside casting cores are restored. The lower column retains
side supports and a 60 mm casting roof above the motor bay. Sealing, cover edges,
mounting fasteners and structural qualification remain unfinished.

Service removals are declared as `ServiceRemoval` design requirements where the parts are built and swept against the whole mill by
`AssemblyIntegrity.service` (every 2 mm along the path, the screws that hold the removed parts taken away with them; nuts are not
screws, so they are listed): in `MillXYStage`, `yMotorWithdrawal` (rear hatch and the motor's four screws off, motor and shaft out of
the back), `xMotorWithdrawal` (hood, belt and the nuts behind the flange off; the motor with its mounting plate out of the saddle along
-X), `xScrewWithdrawal` (hood, belt and both bearings off; screw, nut, pulley and retainer out along -X) and `yScrewWithdrawal` (motor with
its mount, coupling, front bearing and the nut's bracket off; screw and nut out of the front along -Y); in `MillZAxis`, `zMotorWithdrawal`
(four screws off, straight up); in `MillFramePreview.attachSpindle`, spindle `motorWithdrawal` (belt, pulley, guard, screws and nuts off,
motor up) and `cartridgeWithdrawal` (tool, belt, motor and carrier, draw-bar cylinder off; the cartridge with its rotor, draw-bar and pulley
down out of the head, 210 mm, 56 mm short of the table at the upper Z stop). `MillServiceChecks` (`haxeon.service.json`, about 13 min: 158 s for the declared removals) sweeps
all of them clear and keeps the findings of the procedures that do not work:

- Two earlier findings were small reliefs, now cut. The old y-servo finding (the Y motor's round body met the base's rear apron, 82.8 mm3
  after 24 mm) is gone: the base's `rear-y-drive-seat` recess, 118 mm wide, now runs out to the rear edge. The X motor's connector envelope
  was 2 mm outside the plate pocket and met the saddle's end wall after 18 mm (53.8 mm3): the connector pocket now runs through the wall.
- The Y screw cannot leave by the rear with its nut: the 48 mm nut flange meets the Z axis' lower plate (24 mm crossing bore for the Y shaft)
  after 240 mm (326.7 mm3), then the Z floating bearing and the Z screw. The front is open once the nut's bracket is off.
- The Z screw cannot leave upwards with its nut: the nut's grease fitting meets the rear face of the Z adapter plate across the front of the
  channel after 72 mm (5.05 mm3, over the 100 mm of the plate). The fitting comes out first, or the plate takes a 6 x 1.5 mm relief that
  competes with the M80E seat on the same face: not cut. Down is closed by the channel floor; the nut is not a screw to thread out.
- The spindle cartridge cannot leave upwards: the head is over it (10,574 mm3 after 2 mm). Down it clears the table by 56 mm; with the
  tool in it would meet the table after 266 mm.

Way covers (`machinekit.cover`, chip protection): the mill built with `new BenchMill(false, false, true)` (a "covered" mill; the default
mill is unchanged) has a **box cover** at both ends of each axis, six in all, checked by `MillWayCoverChecks` (`haxeon.way-covers.json`, about
2 min). A first study of ring covers (a bellows round each screw end) found that none fits the mill as built: the Y ends had 32 and 27.5 mm of
space (the smallest bellows folds to 30 and its flanges would bolt to a 42 mm bearing housing), X had 14 mm between the saddle top and the
table underside above the screw, Z had the screw 26 mm from the column's side face, and the guides' blocks fill the 28 mm gaps between base,
saddle and table, so a ring cannot enclose them. A box cover has a roof and two walls and no floor: it stands on a surface of the machine, which
closes it, and its skins nest outward on the three walled sides only (`WayCoverPlan.CoverShape`), so nothing grows into the surface and one
cover encloses the guides and the screw together.

| Cover | stands on | fixed end (a steel wall) | moving end | folded / space | reach | skins, flange |
| --- | --- | --- | --- | --- | --- | --- |
| Y front | base top | wall 7 mm in front of the front bearing's stand, the coupling through a bore, the rails through slots; the cover encloses the bearing | saddle front face | 38 / 66 mm | 216 mm | 8, 242 x 114 |
| Y rear | base top | wall 2 mm in front of the rear bearing, the screw through a bore | saddle rear face | 30 / 32 mm | 182 mm | 10, 250 x 135 |
| X left | saddle top | wall on the near bridge behind the bearing, the coupling through a bore | table's left end face | 46 / 57 mm | 307 mm | 9, 124 x 65 (cut back to the table's width) |
| X right | saddle top | wall on the far bridge behind the bearing | table's right end face | 54 / 77 mm | 327 mm | 7, 124 x 60 |
| Z top | column side face | wall above the bearing, the coupling through a bore | carrier on the head over the nut | 54 / 90.5 mm | 340.5 mm | 8, 128 x 75 |
| Z bottom | column side face | wall under the bearing | carrier under the nut's bracket (clearing the nut flange's bolts) | 46 / 50.5 mm | 300.5 mm | 9, 132 x 78 |

All six are `BELLOWS-LAMELLA-8x8` (flat bellows, steel lamellas sewn into the fabric every 8 mm fold, 1:8; assumed values of the family). It is the
only catalogue row that folds into the room: every end has 32 to 90 mm and the steel telescopic rows (`TELESCOPIC-SS-250`, `-150`) fold to 174 mm
and more. A roll-up (spring roller) cover was considered and not built: its cassette needs the same length along the axis as the folded bellows
(a roller of 30 mm or so), so it gains nothing at the 32 mm Y rear, and the model has no strip envelope. X right and Z top could take a steel
telescopic cover if the walls moved out by 100 mm (a longer base or bridge); the lamella bellows are the wear item and the first spare.
Each cover is one BOM line (`WAYCOVER-...`: both flanges, the skins are not bought separately); the walls (`MILL-COVER-WALL-*`) and the two carriers
are steel plates to make; the twelve guide blocks carry 24 end scrapers (`WIPER-HG15-SCRAPER`, `WayWiperCatalog`, assumed values).

Each cover is placed by `BenchMill.coverFrom` from the mill's own poses (nothing is typed in per cover) and installed by `WayCover.installOn`.
Each flange is bolted to its wall, carrier or casting face with screws derived from `BoltedFace`s (flange holes only where the host face exists, 8
mm from its edge), and the check posed each cover at its near and far stop and measured the flanges the planned space and extended length apart.
The sweep (all 27 combinations of each axis at its ends and middle) finds nothing on the covers, and no finding the plain mill lacks (117 in both,
all of them the legacy mill's support gaps). The covers go on after the rails and screws they surround (`IntegrityOptions.fittedBefore`): the
cover is over the rail screws' driver access otherwise, so a rail is changed with the cover unbolted.

Geometry changes, all in the covered mill only (the default mill and the framed concept are untouched, so the frame (917) and Z-axis (423)
assertions stand; they build their layout from the default mill):

- **Z screw 670 to 490 mm**, its floating bearing and boss 180 mm higher (centre z 112.5 to 292.5). The lower 180 mm of the screw lay below the head's
  travel and carried only that bearing, which stood in the base's rear corner where the Y rear cover's stack goes; the lower cover's wall would
  also have stood at the table's height. The Z travel, the spindle reach and the head are as before. Cost: the Z bottom cover has 50.5 mm of
  space (a 5-fold bellows folds to 46), the first to fail if the head's travel grows downward.
- **Y screw 420 to 432.5 mm**, the rear bearing and its stand 12.5 mm further back, into a **bay in the column's lower front** (80 wide, 10 deep, 92 high;
  the bearing's housing stands 5 mm and the screw end 7.5 mm into the column, 10 mm of the 20 mm wall is left). It gives the rear cover 32 mm.
- **Saddle and table**: end faces tapped M5 for the moving flanges (`MillCasting.coverFaces`), no change of shape.
- **Base**: two chip gutters either side of the screw-support pad (21 mm wide, falling 1.5 % over 370 mm, 5.6 mm deep at the front) and two 14 mm
  drain holes through the top skin at their low ends (11 mm of skin is left under the gutter). The Y covers are declared a `GuardedChannel`
  on the base (`yCoverGutters`): the floor falls at least 1 % and the bores exist in the base, as `AssemblyIntegrity` checks; the holes open into the
  hollow base over the chip tray. A chip or coolant that gets past a wiper runs forward along the gutters; the covers' roofs shed the rest over
  the base's sides.
- New parts: six steel end walls, two carriers (14 mm, tapped to suit the flange holes), wipers. The overall envelope is unchanged (X walls end
  inside the bridges, the Y front wall inside the base's front edge).

Open, in the covered mill: the walls are fixed members on the base, the bridges and the column, and the carriers on the head, without derived
screws of their own (a weldment or an assumed bolting, to be detailed); the X and Z covers have no guarded channel (an X cover's floor is the
flat saddle top, and what gets in leaves over the saddle's ends onto the bridges and the tray; a Z cover is open to the column's side face); the
bellows are not drawn pleat by pleat (the nested skins are their envelope, with the loose skins' and flanges' sizes from the folds). The framed
concept (`MillXYStage`, `MillZAxis`) has its own layout: its X gap is 76 mm (28 mm of block and a 48 mm riser) with the screw 40 mm above the
saddle top, so a box cover fits there as well, but its supports are elsewhere (rear drive bay and hatch), and it is not covered here.

Z screw supports: each foot is bolted to a steel boss cast into the column's +x side wall (`MillCasting.ColumnBoss`, placed by
`BenchMill.zBosses`). The boss is a 62 x 29 x 16 mm tapped steel block (`MILL-Z-BOSS-FIXED`, `-FLOATING`) in a pocket cut in the 20 mm wall,
its back face at x = 75 and its face 1 mm proud of the wall at x = 91, which is the supports' foot plane, so the Z screw axis does not move.
The foot is 14 mm in front of the column face, so the rear row of holes is over the casting and the front row is over the boss's forward
part, which stands out of the column's front face beside the head (the head never comes near: its top is 27 mm below the upper boss at the
upper stop). The six screws are derived like the Y and X supports': M5 x 16 for the fixed support's four holes, M5 x 40 for the floating's two;
the mill derives 144 screws. Cast in, the bosses need no screws of their own.

Y travel increases from 150 to 160 mm without changing spindle reach or base
footprint. The table retains 25 mm tool-edge margin and clears the column-side envelope
by at least 3 mm at the rear stop. The 2 mm geometric overtravel allowance is not a verified
stopping distance. More travel requires revisiting the throat or lower Z guides.

X uses a
1:1 return drive: two 20-tooth HTD5M pulleys, 80 mm centers and a 15 mm wide belt.
Its motor sits in a saddle pocket below the X screw. A recessed motor plate and
removable end hood keep the drive inside the base width without widening the
casting. Both motors remain within the base perimeter over the full Y stroke.
The belt adds compliance and requires tensioning, alignment, retention and life
checks. Its 5000 N/mm cord rigidity per width is an assumption, not vendor data.

Screw bearings sit on steel pedestals. Nominal M5 × 55 bearing bolts have 12 mm
pedestal engagement; four M6 × 25 bolts anchor the X motor plate. These match the
modeled reference holes, not verified supplier drawings. Complete guide/block,
motor, nut, cover and insert drilling and fastener qualification are unfinished.

Z uses 180 mm guide spacing, four blocks at 141.4 mm vertical centers, a
234 × 222.8 × 20 mm carriage, and a central screw/nut
inside the column channel, fixed/floating bearings, a motor stand and brake
hardware. Its tapered steel head has a 234 mm rear flange, a 120 mm nose, two rows
of side windows and continuous top/bottom chords. Four nominal M6 × 16 bolts
attach the flange to blind carriage taps, with straight tool access beside the
180 mm rear body. Spindle reach remains fixed. The head carries the BT30 packaging study and
its braced centered rear belt motor. Nominal structural clearance is 3 mm. The X/Y geometric overrun
reserves are 7/2 mm; these are not established stopping distances.

## BT30 spindle packaging study

The [spindle drive requirements study](../spindle-drive-study/README.md) defines a
provisional 500–8000 rpm output envelope, compares vector-drive and servo options,
and screens modeled Leadshine motors. Its first sourcing target is a 3 kW class
spindle-duty system; this has not replaced or qualified the compact motor below.

`MillSpindleHead` now owns the cartridge, holder, drawbar, spring stack,
gripper/pull stud, reaction frame, pneumatic cylinder/rod/piston, centered rear
motor, belt and motor carrier. The legacy ER20 machine keeps its independent
motion/control example. All new parts have rebuild recipes.

The study targets 2.2 kW at 8000 rpm with a 1:1 HTD5M drive: 40T pulleys, a nominal 440 mm belt and 120 mm centers.
Its stated 5000 N/mm cord rigidity per width is an assumption. Its compact motor is
an **assumed** Ø90 × 160 mm body with a 110 mm flange; it is not the motor in the
Automation Technology kit. The cartridge uses the DAIYA reference's Ø90 body
and Ø125 flange, with explicitly assumed axial interfaces and mounting pattern.
The nominal 7:24 taper is packaging geometry, not a toleranced BT30 interface.

The motor sits on the centerline behind the release assembly on a braced steel
carrier with two side supports and four M6 head anchors. Recessed M5 heads keep
the motor fasteners clear of the belt. Four slotted mounting holes provide ±5 mm fore–aft
adjustment. A sliding register slot and tension screw, clearance for reaction-frame posts, actual
mounting bolts and separate pulleys make the fit reviewable. A bolted belt guard encloses the drive. Both ±5 mm adjustment endpoints are
checked against the full machine for interference. At the upper Z stop and nominal tension, a 50 mm upward
withdrawal envelope checks motor removal after removing the guard, belt, pulley and
motor fasteners. Belt selection and cable connectors still need supplier qualification.

The four-post release frame bears on the stationary cartridge rear housing.
It carries an assumed compact double-acting cylinder with an 80 mm bore,
100 mm square end caps and 46 mm overall height. Its bridge is 20 mm lower;
the complete release package ends 74 mm lower than the previous tall cylinder.
A matched piston and continuous rod retain a 10 mm study stroke, while the
release inspection uses 6.5 mm. The smaller mounting pattern uses four M6 bolts.
At an assumed 6 bar at the cylinder, theoretical push force is 3016 N; an explicit
20% allowance leaves 2413 N against the DAIYA reference's 2000 N unclamping force.
This provisional comparison does not qualify the actuator: minimum delivered
pressure, actual friction, required stroke and supplier interfaces remain inputs. The two inspection joints
are `spindle/drawbar-release` (4.5 mm) and `spindle/pistonRod-release` (6.5 mm,
including a 2 mm retracted gap). These are packaging assumptions, **not** a
matched supplier stroke/force specification. The spring stack does not deform
in the rigid-body view, and gripper jaw opening/tool ejection is not simulated.
Release inspection motions do not constitute an operating tool changer.

The longer holder shifts the head and lengthens the column/Z guides while
preserving the original tool-tip range and 250 mm Z stroke. Full-stroke CAD
checks include the actual spindle payload against fixed structure and the
Cartesian XYZ envelope, plus attachment clearances at the common head pose.
Nominal threaded engagements and belt/tooth contact are named audit exclusions.

Supplier comparison remains necessary before purchasing or making the head:

| Reference | Published information | Remaining selection issue |
| --- | --- | --- |
| [DAIYA BT30SP](https://daiya.co.jp/en/product/spindle/atc/) | 90 mm D1, 8000 rpm, 1200 N clamping and 2000 N unclamping | Controlled drawing, actuator stroke/interface, mass and bearing limits; different performance from PMK |
| [PMK 100MCB30SSS](https://pmkindia.com/datasheets/spindles_belt-driven-spindles-for-machining-center.pdf) | 100 mm housing, 12000 rpm, family clamping force 2800 N | Unclamping force/stroke and matched actuator still unknown |
| [Automation Technology complete kit](https://www.automationtechnologiesinc.com/download/17588/) | 90 mm BT30 cartridge, 6000 rpm, 2.2 kW motor; drawing labels motor L=380 mm and includes release-cylinder/frame drawings | Bulky reference; current motor and blue allocation cannot represent this kit; confirm actuator force at operating pressure and ambiguous drawing units |
| [Tormach 770MX release assembly](https://knowledgebase.tormach.com/770mx/bt30-power-drawbar-cylinder-770mx) | Two-stack double-acting 100 mm bore cylinder, 25 mm stroke, dedicated plate and standoffs | Reference for matched architecture; not interchangeable release hardware for another spindle |

The [Tormach head parts](https://knowledgebase.tormach.com/770mx/spindle-head-770mx)
illustrate the separate motor base, cartridge and drawbar mounting structure.
The current study adopts that separation without claiming those parts fit.
Actual release force versus pressure/stroke, spring preload, bearing reaction
loads, retention, spindle orientation and independent clamp/release feedback
remain selection gates. A magazine/rack, forks, exchange poses and sequencing
are a later assembly.

## Services and controls to implement

Keep lubrication and wiring accessible behind removable covers. Provide a fixed
lubrication manifold, metered guide/screw circuits and protected flexible runs to
moving members. Select fittings and way-cover compressed/extended envelopes with
the actual components; check maintenance access at each travel extreme. The
[HIWIN assembly manual](https://www.hiwin.com/wp-content/uploads/Linear-Guideway-Assembly-Manual-HIWIN-Corp.pdf)
is the installation reference; a lubrication schedule must follow the selected
rail, lubricant and operating conditions.

Separate dry drive bays from the wet cutting area. Design chip fall, tray removal,
coolant drainage and seals together. Add cable/hose carriers, independent home and
limit feedback, physical stops, stopping-distance checks and Z brake feedback.
Motor hoods alone do not provide complete chip/coolant protection or guarding.

ATC release permissives must include verified zero spindle speed, orientation
when required, confirmed exchange position, supported tool, adequate air supply
and coherent clamp feedback. Rotation requires confirmed clamping, release off,
release feedback off, closed guards and no control fault. Specify failure behavior
for lost air/power, sensor contradiction and lost clamp confirmation. Select a
mechanism that retains the tool without air/power. These are interface requirements;
the new mechanical assembly has no commissioned controller/interlock integration.

## Inspect and check

The FEM implementation has started with a posed geometry packet, datum checks,
analytical solver fixtures and a casting meshing pilot. See
[fem/README.md](fem/README.md) for commands and the current analysis boundary,
and [FEM_PLAN.md](FEM_PLAN.md) for the remaining implementation gates.
There is no assembled-machine FEM stiffness result yet.

Run these commands sequentially: the example configurations share one output directory.

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.xy-stage-tests.json
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.z-axis-tests.json
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.frame-tests.json
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.spindle-tests.json
./haxeon/scripts/haxeon run --project=machinekit/tests/casting/haxeon.json
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.frame.json
./app/run-built.sh --project="$PWD/machinekit/examples/bench-mill/build/mill-frame-concept.mtrg" --perspective
```

Outputs under `build/`:

- `mill-frame.mtrg`: castings, steel seats and leveling feet.
- `mill-z-axis.mtrg`: frame and mechanical Z assembly.
- `mill-xy-stage.mtrg`: isolated cross slide, drives, nominal bolts and hoods.
- `mill-frame-concept.mtrg`: complete mechanical XYZ concept and BT30 spindle study.
- `mill-frame-concept.json`: travel, dimensions, casting masses and audit status.
- `design-review/review.json`: assumptions, static guide study, idealized table
  section comparison, ATC candidate, services requirements and all nine stage statuses.
- `design-review/interfaces.csv`: modeled versus unresolved mounting interfaces.
- `design-review/*.step`: twelve local-coordinate concept parts for CAD review.
  These are not dimensioned/toleranced manufacturing drawings.

The preview exposes `xyStage/x`, `xyStage/y`, `zAxis/z`, spindle rotation
(`spindle/rotor-turn`) and the two release inspection joints. Tests cover joint
motion and signed screw/belt relations before and after recipe restoration,
connected solids, physical support contacts, foot openings, casting/steel
partitioning, drive containment and negative collision controls.

`MillXYStageIntegrity` checks continuous X sweeps against the actual saddle/drive,
then Cartesian-product X/Y envelopes against the fixed Y drive, frame and Z
assembly. It also checks stationary Y hardware inside the casting and continuous
rearward motor withdrawal after removing the rear hatch and its screws. The
ribbed table sweep preserves the central corridor; the saddle sweep fills its
moving motor pocket conservatively.
The return-drive fit inside the saddle is also checked with actual CAD.
`MillZAxisIntegrity` checks continuous moving structural envelopes against the
frame and stationary Z hardware. These are nominal-stroke structural checks,
not service/enclosure or braking qualification. `MillSpindleIntegrity` adds
actual payload and full Cartesian XYZ clearance checks.

## Nine-stage review and release gates

| Stage | Current result | Work still required |
| --- | --- | --- |
| 1. Envelope | Travel retained; motor bays inside base perimeter | Final spindle, covers and service-removal envelope |
| 2. X/Y drives | Recessed X return drive; protected Y direct drive | Verified motors, belt/tensioner, retention and life |
| 3. Base and feet | Side pockets, supported bored steel seats, foot envelopes | Ratings, anchoring, stability and casting process |
| 4. Saddle/table | Integral ribs and section comparison | Complete drilling and assembled stiffness assessment |
| 5. ATC spindle | Rebuildable BT30/drawbar/cylinder packaging study | Controlled spindle/drawbar/actuator drawings and selection |
| 6. Head/carriage | BT30 bore, bolted reaction frame and braced motor carrier | ATC collar, release reaction bridge, mounts and stiffness |
| 7. Interfaces | X motor plate, reference bearing bolts, foot bores | Remaining patterns, fasteners, access, welds and anchors |
| 8. Services | Motor hoods and defined service/control requirements | Way covers, lubrication, cables, coolant, sensors and controls |
| 9. Validation | Continuous sweeps, static guide admission, concept STEP packet | Whole-machine FEA, thermal/modal evidence, manufacturing drawings |

The guide study uses CAD-estimated moving masses, a 20 kg payload 50 mm above the
table, ±500 N cutting force per world axis, a conservative ±1 m/s² acceleration
envelope and a static factor of two. It examines eight travel corners and force
sign combinations. Rail ratings remain catalog assumptions; equal block stiffness
and rigid carriages are assumed. It does not establish fatigue life, positioning
accuracy or the actual controller's acceleration limits.

The table comparison evaluates an unperforated deck and rib section. Its second
moment of area improvement is not an assembled-machine stiffness multiplier.
Guide/joint compliance, pockets, insert anchorage, preload, thermal effects and
casting flexibility require separate analysis. The FEM tools currently cover
geometry export, solver fixtures and casting meshing; assembled static/modal
analysis and physical material/deflection measurements remain pending.

Manufacturing release remains blocked until those gates have evidence. Preserve
material-property provenance, replicate the precision interfaces and grout the
column joint as described under "Precision interfaces", and develop moulds, curing, lifting,
inspection and assembly instructions from the final geometry.

## Separate column joint and structural access

The 320 mm column now has a 440 mm wide inverted-T foot, with a 90 mm high
vertical skirt and tapered shoulders reaching the upright at 150 mm. `MineralCasting`
supports this optional structural foot as saved geometry; old recipes retain
their original box bodies. Base and column remain distinct castings.

Four nominal M12 × 35 clamps sit in side-entry pockets in the vertical skirt, each bearing on a steel
sole pad and engaging a separate embedded steel base anchor. The pattern is
380 mm across and 140 mm fore/aft. Machined pad pairs allow alignment/shimming;
anchor development, joint preload/strength and permanent locating features
remain engineering inputs. The central Y motor bay and rear extraction path
remain open. The taper stays closed above the pockets. A modeled Ø12 mm L-key
clears each bolt through the side opening; the bolt axes and steel seats stay vertical.

The Y beds stand 20 mm above the lowered side and center fields, while the
finished rail plane stays at 224 mm. A retained island supports the front
bearing insert. Y rails span 220 mm, with 101.4 mm between block centers along
Y. Four saddle mounting pads, relieved undersides, deep side openings, continuous
X-guide chords/end webs and the central nut spine replace shallow cosmetic
pockets. Four side-entry service pockets are centered on the actual Y-block bolt patterns.
Their width and depth follow both screw rows, with rounded corners, retained
side webs, a 12 mm mounting floor and a closed 14 mm top deck. Sixteen M4 × 16
screws engage the blocks by 4 mm, within the HGH15CA reference's M4 × 5 thread.
The screws remain vertical; an L-key enters from the front or rear opening,
so top access wells are unnecessary. Both legs of a Ø5 mm wrench envelope are
checked against the saddle and installed X motor. The 508 mm Y rails end in explicit
front-face reliefs in the lower Z bearing plate, retaining the plate's rear
section and anchor stations.

The Y nut now seats on a registered steel adapter backed by an integral saddle
boss and four diagonal webs joining the central spine to the guide-pad webs,
bottom chord and top deck. The front face and screw centerline stay unchanged.
A shallow rear locating lip registers the nut flange without changing its axial seat.
Four recessed M6 × 25 bolts secure the adapter; two integral locating pilots use
one round and one relieved seat. Six rear-entry M5 × 16 screws attach the nut
flange to blind adapter taps. Assemble these screws with the nut/adapter off the
saddle, before installing the ballscrew assembly; in-place nut extraction is not
claimed. These interfaces follow the existing generic SFU1605 study dimensions,
not a verified supplier drawing.

The former edge-tangent Ø10 grease hole is replaced by a deliberately open,
20 mm wide saddle service notch and a 14 mm wide, rounded-bottom slot in the
adapter. The adapter is still one connected part. A slim grease extension can
enter from the front without removing the table. Service pose: X +125 mm,
Y zero, Z retracted. The access audit includes all XY components, frame parts
and retracted Z components, rather than just the saddle and guide rails.

`machinekit.design.DesignChecks` supplies reusable minimum-dimension,
actual-solid material-envelope, blind-fastener-stack, coaxiality and tool-path
checks. `DesignInterfaces` facets on the adapter, saddle and ballnut declare local hole,
port and material requirements. Their hole data also generate the machining
tools. The stage saves `FastenerJoint` and `AccessPath` requirements, including
bench phases and service joint positions. `AssemblyDesignChecks` evaluates them
during the normal XY integrity audit, so blocked access or inadequate material
also fails preview/export generation. `MillYNutDesignChecks` only supplies
mill-specific assembly intent and the surrounding frame/head context.
Requirements survive recipe round trips, snapshots and nested placement;
member and joint references are remapped. See the shared design package README
for the interface model and independent pump-fixture test command.
The current study requires 3 mm around adapter M5 drillings, 4 mm around M6
counterbores, 5 mm around casting taps and retained 10 mm saddle-notch cheeks.
Both X rail mounting strips must retain a continuous 3 mm bearing land; the
notch stops before the front rail footprint.
Blind M5 taps have 8 mm usable thread, 8.5 mm cylindrical drilling plus a
conservative 1.5 mm drill-point allowance, leaving 2 mm behind the drill.
M5 ×16 screws engage 6 mm and have 2 mm to the usable thread end. The CAD bore
conservatively includes the drill-point envelope as a full-diameter cylinder.
M6 casting drill envelopes include their 1.5 mm point allowance too.

The assembly checks cover bench access to six nut screws, front access to four
installed adapter screws, and continuous nut/pilot insertion envelopes into the
saddle. The sequence installs the ballnut/adapter with its screw before final
bearing attachment; it does not require stripping the ballnut off its screw or
claim removal with the entire drive assembled.

The assumed lubrication service tool has an 8 mm slim extension, 10 mm elbow,
and 40 mm handling space outside the front face. Additional clearance is built
into the swept envelopes (10/12/42 mm respectively). Checks include the front
approach, the coupler mouth actually surrounding the nipple, a 5 mm disconnect
stroke, and elbow/hose/handling space. A separate 52 mm bend envelope reserves
a 20 mm hose centreline bend radius with 5 mm tube radius and 1 mm clearance. These are explicit tool requirements,
not a claim that an unspecified commercial grease gun fits. A supplier tool or
remote lubrication kit still needs selecting and its dimensions substituting.
Tests deliberately reject thin walls, nearby cavities, short/bottoming screws,
misaligned axes and obstructed tool approaches; the previous grease-hole and
closed-adapter geometries are also rejection fixtures. All dimensions are
process-specific study limits, not strength qualification. FEM and quantified
stiffness comparison remain deferred.

The upper Z motor stand has a broad bolted foot, open cheeks, motor register,
mounting bolts and removable front coupling guard. The motor can lift away
without removing the fixed bearing. Register/pattern dimensions remain
assumed motor interfaces, and bolt preload/welds require qualification.

`mill-spindle-service-study.mtrg` shows actual modeled spindle/release hardware
with the head and column. It no longer displays a blue reserved cylinder.
The preview and STEP packet remain a mechanical design study; manufacturing
qualification and installed lubrication, way covers, sensors and controls
remain outstanding.

The flat cutter preview includes nominal two-flute, 30-degree helical gullets; its
Envelope detail retains the full clearance envelope. The BT30 holder has opposed
flange drive slots so spindle orientation is visible. These are assumed visual
details rather than supplier-qualified cutting or drive geometry. In Materia,
use **Inspect** beside a spindle joint in the Machine pane for a live section;
this exposes drawbar and piston travel while leaving the generated CAD intact.

The [selected Leadshine Y-drive variant](Y_SERVO.md) fits the checked HZMotion
interfaces into this mechanical frame and stage. Open
`materia.y-servo.project.json` for the assembled preview.

Each column side now uses one continuous steel Z mounting strip for the rail
and both bearing anchors. The `railSeat`, `bearingLowerFace` and
`bearingUpperFace` regions retain their mounting datums on `column/railZLeft`
and `column/railZRight`. FEM bearing loads still use only the upper 20 mm region.

The [Compact VMC capacity study](CAPACITY_STUDY.md) derives a
400 × 300 × 380 mm working envelope with a 700 × 400 mm table. Open
`materia.capacity.project.json` for its movable packaging preview. It has its own
build outputs and does not replace the smaller mill. Drives, structural stiffness
and finished-machine capacity still require qualification at the larger size.

Both VMC configurations now use catalog Leadshine motors for every installed drive.
See [MOTOR_MOUNTS.md](MOTOR_MOUNTS.md) for the selected models, supplier/custom
mount split and the Compact variant’s still-unfitted XY drives. The spindle selection
is 1.5 kW with a 3:1 speed-increasing belt, replacing the assumed 2.2 kW envelope.

The Small and Compact VMC Z drives share an HZMotion M80E-BK15-C integrated
motor/bearing housing and BF15 support, with an SFU2005 screw. A flat T adapter
fits the housing between the guide rails, replacing the separate stand/bearing
stack. See `MOTOR_MOUNTS.md` for the STEP/drawing discrepancies and outstanding
brake, fastener and load qualifications.
