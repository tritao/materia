# Bench mill and mineral-cast machine design

The working legacy `BenchMill` has 250 mm X, 150 mm Y and 250 mm Z travel.
`MillFramePreview` develops a separate mechanical concept with 250 mm X, 160 mm Y and 250 mm Z travel.
Its output is a design study, not a machine released for manufacture.

## Current geometry

`MillFrameLayout` owns casting datums. `MillXYStageLayout` and `MillZAxisLayout`
size the axis assemblies. Castings and separately itemized steel interfaces use
shared `MineralFoundation` recipes; other machine families keep their own layouts.

The reference base is 520 × 909 × 220 mm. The Y motor sits in a rear service bay beneath the column, with a continuous
60 mm casting roof and side supports. The column is 320 × 260 × 833 mm. Guide/bearing inserts stand 4 mm proud
of the casting, with 16 mm embedded thickness. Core openings, walls, dimensions
and materials remain design assumptions requiring process and structural review.
That geometry assumes the seats are machined after casting. The chosen
process now uses replication instead; see below. The geometry has not been
updated yet.

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

Four side-open foot pockets expose adjustment nuts. M16 foot envelopes pass
through 18 mm steel seat bores and separate stem openings in the casting. Front
foot stations sit beneath a full-height bridge; underside cores start behind it.
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

A dedicated Ø10 mm grease-access passage reaches the existing radial fitting.
The Ø8 mm coupler envelope is checked with X parked at +125 mm and Y centered,
clearing the saddle and X guides/blocks. The guide-block access pockets stay
open. This is a geometry and assembly-access revision; FEM and a quantified
stiffness comparison are deferred.

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
