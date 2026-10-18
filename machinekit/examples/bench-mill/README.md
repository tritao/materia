# Bench mill and mineral-cast machine design

The working legacy `BenchMill` has 250 mm X, 150 mm Y and 250 mm Z travel.
`MillFramePreview` develops a separate mechanical concept with 250 mm X, 160 mm Y and 250 mm Z travel.
Its output is a design study, not a machine released for manufacture.

## Current geometry

`MillFrameLayout` owns casting datums. `MillXYStageLayout` and `MillZAxisLayout`
size the axis assemblies. Castings and separately itemized steel interfaces use
shared `MineralFoundation` recipes; other machine families keep their own layouts.

The reference base is 520 × 909 × 220 mm. The Y motor sits in a rear service bay beneath the column, with a continuous
60 mm casting roof and side supports. The column is 320 × 260 × 728 mm. Guide/bearing inserts stand 4 mm proud
of the casting, with 16 mm embedded thickness. Core openings, walls, dimensions
and materials remain design assumptions requiring process and structural review.

Four side-open foot pockets expose adjustment nuts. M16 foot envelopes pass
through 18 mm steel seat bores and separate stem openings in the casting. Front
foot stations sit beneath a full-height bridge; underside cores start behind it.
Foot supplier ratings, lateral restraint, anchoring, floor suitability and insert
anchorage are unresolved. The shown pads/nuts are geometric concepts.

Y guides carry a 450 × 174 × 80 mm saddle. A gusseted steel bridge connects the lowered Y nut to its front face.
X guides, spaced 124 mm apart, carry a 400 × 210 × 40 mm T-slot table with
48 mm integral longitudinal ribs. These replace four isolated spacers and keep
the central drive corridor open. Table top remains 448 mm above the casting bottom.
The spindle datum and column/guide length retain the earlier 58 mm stack increase.

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

Z uses 180 mm guide spacing, four blocks, a broad carriage, a central screw/nut
inside the column channel, fixed/floating bearings, a motor stand and brake
hardware. Its ribbed steel head carries the existing manual spindle and offset
belt motor. Nominal structural clearance is 3 mm. The X/Y geometric overrun
reserves are 7/5.5 mm; these are not established stopping distances.

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
| 6. Head/carriage | Existing ribbed head and broad carriage retained | ATC collar, release reaction bridge, mounts and stiffness |
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
