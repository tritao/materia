# Bench mill and mineral-cast machine design

The working legacy `BenchMill` has 250 mm X, 150 mm Y and 250 mm Z travel.
`MillFramePreview` develops a separate mechanical concept with 250 mm X, 160 mm Y and 250 mm Z travel.
Its output is a design study, not a machine released for manufacture.

## Current geometry

`MillFrameLayout` owns casting datums. `MillXYStageLayout` and `MillZAxisLayout`
size the axis assemblies. Castings and separately itemized steel interfaces use
shared `MineralFoundation` recipes; other machine families keep their own layouts.

The reference base is 520 × 909 × 220 mm. The Y motor sits in a rear service bay beneath the column, with a continuous
60 mm casting roof and side supports. The column is 320 × 260 × 768 mm. Guide/bearing inserts stand 4 mm proud
of the casting, with 16 mm embedded thickness. Core openings, walls, dimensions
and materials remain design assumptions requiring process and structural review.

Four side-open foot pockets expose adjustment nuts. M16 foot envelopes pass
through 18 mm steel seat bores and separate stem openings in the casting. Front
foot stations sit beneath a full-height bridge; underside cores start behind it.
Foot supplier ratings, lateral restraint, anchoring, floor suitability and insert
anchorage are unresolved. The shown pads/nuts are geometric concepts.

Y guides carry a 450 × 200 × 80 mm saddle. A gusseted steel bridge connects the lowered Y nut to its front face.
X guides, spaced 160 mm apart, carry a 400 × 210 × 40 mm T-slot table with
48 mm integral longitudinal ribs. These replace four isolated spacers and keep
the central drive corridor open. Table top remains 448 mm above the casting bottom.
The spindle datum and column/guide length retain the earlier 58 mm stack increase; the taller Z block footprint adds another 40 mm
to the column and Z guides.

The Y screw and motor share a 124 mm shaft height above the casting bottom.
The motor drives rearward through a coaxial coupling, with its body beneath the
column. Removable front and rear hatches use dedicated steel anchors and M4 screws.
The rear hatch admits axial motor removal; allow at least 163 mm behind the base
for the modeled extraction, plus cable and tool access. The saddle bridge runs in
an open central base channel above a continuous sole.

Y travel increases from 150 to 160 mm without changing spindle reach or base
footprint. The table retains 25 mm tool-edge margin and clears the lower Z guides
by 3 mm at the rear stop. The 2 mm geometric overtravel allowance is not a verified
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
180 mm rear body. Spindle reach and the existing manual spindle remain fixed.
The head carries the offset
belt motor. Nominal structural clearance is 3 mm. The X/Y geometric overrun
reserves are 7/2 mm; these are not established stopping distances.

## ATC spindle and head decision

The existing `SpindleCartridge` models a manual ER20 cartridge, not an ATC spindle.
Adding a cylinder alone cannot establish drawbar or toolholder compatibility.

A candidate for dimensional review is PMK's short-nose **100MCB30SSS BT30**.
The [manufacturer's datasheet](https://pmkindia.com/datasheets/spindles_belt-driven-spindles-for-machining-center.pdf)
lists a 100 mm housing, L8 of 264 mm, a maximum speed of 12000 rpm and 2800 N
clamping force for the 100 mm BT30 family. These are candidate catalog facts,
not a purchase selection or an installation drawing. Clamping force must not
be substituted for the actuator's required release force.

This candidate does not fit the current 70 mm head bore. Before changing the
installed spindle model, obtain its controlled drawing, gauge datum, mass,
bearing load limits, nose/toolholder/pull-stud specification, pulley journal and
mounting fits. Select the matching drawbar and release actuator together, including
release force/stroke, operating pressure, sensors and the release-force reaction
path. Then revise the head collar, ribs, motor shelf and belt layout around them.

Reserve access above the spindle for the release assembly, hose connections and
removal, throughout Z travel. A release reaction bridge must connect to the
specified stationary spindle structure; do not assume a load path through the
bearings. No actuator, ATC capability or supplier-sized keepout is presently
claimed by the preview. The magazine/exchange mechanism follows this interface.

The [Tormach BT30 upgrade procedure](https://knowledgebase.tormach.com/1100m/bt30-spindle-upgrade-m-to-mx-installation-manual)
is a reference for treating spindle/drawbar hardware as a matched installation;
its part numbers and air settings are not specifications for our candidate.

## Next spindle package: architecture and supplier comparison

Develop the cartridge, drawbar, release mechanism, motor mount and belt drive as
one spindle-head package. Retain the current preview until a package has real
mounting datums; the blue 120 × 200 mm volume remains a project allocation.
The working target for comparison is a compact BT30 package around 2.2 kW and
6000–8000 rpm, subject to the requested spindle speed/power choice.

The preferred motor position is behind the spindle, toward the carriage, on a
plate carried by both rear head webs and its mounting structure. A rear-corner
position is the fallback when the actuator or column prevents a centerline fit.
Check actual motor body, flange, connectors, cylinder fittings and maintenance
space before deciding the offset. The present narrow collar bridge is not the
mount for the future larger motor. Provide positive belt adjustment, a locking
arrangement, accessible mounting bolts and a removable belt guard. Check belt
radial loads against both spindle and motor limits.

Use a spring-clamped drawbar with the selected BT30 gripper and pull stud; the
release actuator acts through the supplier-approved reaction structure. Model
clamped, released and rotating-clearance positions, including tool ejection and
actuator retraction. Verify minimum inlet pressure, force versus stroke, return
behavior, and independent clamp/release feedback. A nominal cylinder bore or
catalog clamping force alone is not a release-system specification.

| Reference | Published information | Remaining selection issue |
| --- | --- | --- |
| [DAIYA BT30SP](https://daiya.co.jp/en/product/spindle/atc/) | 90 mm D1, 8000 rpm, 1200 N clamping and 2000 N unclamping | Controlled drawing, actuator stroke/interface, mass and bearing limits; different performance from PMK |
| [PMK 100MCB30SSS](https://pmkindia.com/datasheets/spindles_belt-driven-spindles-for-machining-center.pdf) | 100 mm housing, 12000 rpm, family clamping force 2800 N | Unclamping force/stroke and matched actuator still unknown |
| [Automation Technology complete kit](https://www.automationtechnologiesinc.com/download/17588/) | 90 mm BT30 cartridge, 6000 rpm, 2.2 kW motor; drawing labels motor L=380 mm and includes release-cylinder/frame drawings | Bulky reference; current motor and blue allocation cannot represent this kit; confirm actuator force at operating pressure and ambiguous drawing units |
| [Tormach 770MX release assembly](https://knowledgebase.tormach.com/770mx/bt30-power-drawbar-cylinder-770mx) | Two-stack double-acting 100 mm bore cylinder, 25 mm stroke, dedicated plate and standoffs | Reference for matched architecture; not interchangeable release hardware for another spindle |

The complete kit drawings demonstrate why the current 1.1 kW motor envelope and
120 × 200 mm reservation cannot simply be reused as verified BT30 geometry.
The [Tormach head parts](https://knowledgebase.tormach.com/770mx/spindle-head-770mx)
also distinguish the motor base/pivot plate, spindle assembly and power drawbar.

Implement a dedicated spindle-package assembly with explicit gauge-line,
cartridge-seat, pulley, motor-mount and release-reaction datums. Keep the legacy
ER20 machine independent. Replace the preview's legacy spindle-payload extraction
when the new package is selected, rather than accumulating special-case offsets.
Validate full XYZ motion, release motion, belt adjustment and removal envelopes,
then rerun guide loads using the new masses. Automatic tool exchange additionally
requires a magazine/rack, tool forks, exchange poses and control sequencing;
powered drawbar release alone is not a complete tool changer.

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

Run these commands sequentially: the example configurations share one output directory.

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.xy-stage-tests.json
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.z-axis-tests.json
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.frame-tests.json
./haxeon/scripts/haxeon run --project=machinekit/tests/casting/haxeon.json
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.frame.json
./app/run-built.sh --project="$PWD/machinekit/examples/bench-mill/build/mill-frame-concept.mtrg" --perspective
```

Outputs under `build/`:

- `mill-frame.mtrg`: castings, steel seats and leveling feet.
- `mill-z-axis.mtrg`: frame and mechanical Z assembly.
- `mill-xy-stage.mtrg`: isolated cross slide, drives, nominal bolts and hoods.
- `mill-frame-concept.mtrg`: complete mechanical XYZ concept and existing spindle.
- `mill-frame-concept.json`: travel, dimensions, casting masses and audit status.
- `design-review/review.json`: assumptions, static guide study, idealized table
  section comparison, ATC candidate, services requirements and all nine stage statuses.
- `design-review/interfaces.csv`: modeled versus unresolved mounting interfaces.
- `design-review/*.step`: seven local-coordinate concept parts for CAD review.
  These are not dimensioned/toleranced manufacturing drawings.

The preview exposes `xyStage/x`, `xyStage/y` and `zAxis/z`. Tests cover joint
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
not full spindle/service/enclosure or braking qualification.

## Nine-stage review and release gates

| Stage | Current result | Work still required |
| --- | --- | --- |
| 1. Envelope | Travel retained; motor bays inside base perimeter | Final spindle, covers and service-removal envelope |
| 2. X/Y drives | Recessed X return drive; protected Y direct drive | Verified motors, belt/tensioner, retention and life |
| 3. Base and feet | Side pockets, supported bored steel seats, foot envelopes | Ratings, anchoring, stability and casting process |
| 4. Saddle/table | Integral ribs and section comparison | Complete drilling and assembled stiffness assessment |
| 5. ATC spindle | Documented BT30 candidate and incompatibilities | Controlled spindle/drawbar/actuator drawings and selection |
| 6. Head/carriage | Taller carriage and braced, deep head root | ATC collar, release reaction bridge, mounts and stiffness |
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
casting flexibility require separate analysis. No FEA/modal solver or physical
material/deflection test is configured for this study.

Manufacturing release remains blocked until those gates have evidence. Preserve
material-property provenance, finish-machine the precision interfaces after
casting as specified by the chosen process, and develop moulds, curing, lifting,
inspection and assembly instructions from the final geometry.

## Z head and release-space study

`mill-spindle-service-study.mtrg` shows the revised head, carriage, existing
spindle drive and a blue **reserved space**, not installed actuator hardware.
The project allocation is 120 mm diameter × 200 mm high, beginning 80 mm above
the head. Those dimensions are assumptions and are not derived from a supplier
release-force or actuator drawing. The allocation clears the existing payload
and is swept through Z travel against the fixed structure and full XY envelope.
It contributes no mass to the machine load study.

The PMK BT30 candidate still needs a new spindle bore, release reaction bridge,
actuator mount, gauge datum and drive layout once supplier drawings are selected.
Current webs and flange fasteners are nominal geometry; welds, insert anchorage,
preload, stiffness, thermal behavior and release loads require qualification.

## Reference-inspired XYZ structure

The supplied machine image guides the arrangement, not dimensions or load ratings.
Y guides now span 220 mm and support a longer four-block footprint (85.4 mm
center spacing along Y). The saddle keeps continuous top/bottom
chords and central nut attachment material around its shallow side pockets.
X guides and integral table ribs share a 160 mm spacing. Rib spacing is a saved
component parameter rather than a drawing-only adjustment.

Z uses 141.4 mm block-center spacing, a taller carriage and a 190 mm deep rear
head flange around its 130 mm spindle collar. Upper/lower triangular webs brace
that root. The column and Z guides grow 40 mm to preserve the 250 mm Z stroke
and guide end margins. X/Y strokes remain 250/160 mm; table height, spindle
position, base footprint and the rear direct Y drive remain unchanged.

All eight XY blocks and all four Z blocks must physically contact their carrying
structures. Continuous CAD sweeps include the larger root, saddle and table ribs.
This step does not select or install the pending BT30 spindle or pneumatic release.
