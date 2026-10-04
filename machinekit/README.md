# MachineKit

MachineKit is a Haxe-only layer of machine components above CadKit. A
`MachineComponent` pairs a geometry generator with named connector frames and a
BOM line. `addTo(model, id)` registers an instance and its connectors on a
CadKit `AssemblyModel`, so components mate through the existing assembly API.

Connector frames use the CadKit joint convention: mated frames coincide, and
revolute or prismatic joints act along the frame's +Y axis. Component CAD
frames put their axis along +Z, and every connector's +Y points along +Z.

Geometry fidelity is selected with `ComponentDetail`: `Envelope` gives exact
standard boundary dimensions, and `Preview` adds recognizable exterior features
with simplified internals. Threads are semantic (size, pitch, length) and are
not modelled.

Standard sizes live in typed `Catalog` tables, separate from the generators.
Each catalog entry also exposes `catalog().metadata(designation)` with its source,
optional supplementary sources, standard, verified edition (or `null`), dimension
kind, and conformance level.
`Unverified` means the embedded dimensions still need a complete independent source check;
`verifiedFields` lists any fields already checked in an otherwise unverified row;
`GenericApproximation` means the generated part does not claim a complete standard
interface. The ISO 9409 bolt-pattern flange remains marked this way; NEMA motor
variants use a nominal manufacturer envelope. The deep-groove bearing table has independent SKF boundary-dimension
checks; the M5 screw row has independent Accu and Norelem checks for its listed
fields. NEMA frame rows use manufacturer drawing envelopes and retain `Mixed`
dimension metadata because drawing limits and nominal interface values differ.
Metadata describes the catalog entry, not manufacturing
certification of a generated part.

Each component also produces the machining it needs:

| Component | Catalog | Companion geometry |
| --- | --- | --- |
| `DeepGrooveBearing` | 625–6205, ISO 15 boundary dimensions | `housingSeat()` / `journalDiameter()` with named fits, plus explicit allowance helpers |
| `SocketHeadCapScrew` | M3–M20, ISO 4762 heads | `clearanceHole()` (ISO 273), `tapHole()`, `counterboreHole()` (DIN 974-1) |
| `HexBolt` | M3–M12, ISO 4017 (fully threaded) | `clearanceHole()`, `tapHole()`, `counterboreHole()` |
| `HexNut` | M3–M12, ISO 4032 | `pocket()` for a trapped-nut recess |
| `FlatWasher` | M3–M12, ISO 7089 | — |
| `NemaStepper` | NEMA 17, 23, 34 frame interfaces plus named motor variants | `mountingCutout()`, `mountScrew()`, `boltPattern()` |
| `ParallelKey` | DIN 6885-1 form B (square ends), by shaft diameter | — |
| `RetainingRing` | DIN 471 external, by shaft diameter | `grooveSpec()` (d2, m) |
| `ShaftCollar` | set-screw type, by bore diameter | — |
| `SteppedShaft` | — (built from arbitrary sections) | keyway/groove cuts, end chamfers, shoulder fillets/reliefs, semantic threaded ends, `journalDiameterAt()` |
| `FlangeBearingHousing` | — (sized from a `DeepGrooveBearing`) | `mountScrewPart()` |
| `PillowBlock` | Koyo/JTEKT UCP204–UCP213 base-mounted units | mounting connectors and hole envelope; `mountScrewPart()` and `billOfMaterials(true, length)` for mounting hardware |
| `Bushing` | — (proportional to bore diameter) | — |
| `ShaftCoupling` | — (proportional to the larger bore) | radial set-screw holes/connectors, `setScrewPart()`, `billOfMaterials()` |
| `LinearBearing` | LM8UU–LM20UU | `housingSeat()` with named housing fits; Preview adds end rims and seal tracks |
| `LeadScrewThread` | semantic metric trapezoidal or ACME family, diameter, pitch, starts and hand | `lead = pitch × starts`; positive rotation about +Z moves a right-hand nut toward -Z |
| `LeadScrew` | nominal cylindrical thread envelope | `input`, `output` connectors |
| `LeadScrewNut` | flanged preview sized from a `LeadScrewThread` | `travelPerRevolution()`, `rotationFor()`, `mountScrewPart()` |

Bearing `Slip`, `Transition`, and `Interference` fits provide named generic
layout allowances for journals and housing seats. They make the intended fit
visible in an assembly and are not ISO 286 production tolerances; use the
explicit allowance helpers when a drawing supplies its own limits.

`SteppedShaft` stacks coaxial cylindrical sections along +Z, producing square
shoulders at each diameter change. Keyways are cut on the shaft's local +Y
side, so a `ParallelKey` mated through the keyway's connector with a fixed
joint sits flush in the slot. Retaining-ring grooves add a connector of their
own name when given one, for a `RetainingRing` mated the same way. Optional
`ShaftDetail` data adds end chamfers, shoulder fillets and relief roots, plus
semantic reduced-diameter threaded ends; thread flanks remain unmodelled.
`journalDiameterAt()` and `journalDiameterAllowance()` make the selected fit
allowance explicit for a journal position.

`examples/robot-arm/` is a larger assembly: a six-axis arm on a `Pedestal` with a
`RobotFlange` tool mount, six revolute joints and a shipped pick motion. Its
[README](examples/robot-arm/README.md) covers the layout and how to run it.

`examples/MotorShaftBearings.hx` mounts a NEMA 17 motor on a plate with four
M3 screws and carries an output shaft on two 608 bearings through a continuous
coupling joint. The shaft steps down past the outboard bearing to carry a
retaining ring and an output key. It also produces the aggregated BOM.

To inspect this assembly in Materia, build the app and open the bundled project:

```sh
./haxeon/scripts/haxeon build --project=app/haxeon.json
./app/run-built.sh --project=./machinekit/examples/materia.project.json
```

The project entrypoint generates a CAD preview for every component and preserves
the assembly's connectors and joints. Select a part in the hierarchy to inspect
it; the continuous coupling joint is available through the assembly inspector.
The Sensor panel can rebuild and run this generated assembly without adding a
separate robot. The saved joint placement becomes the simulation start pose,
and simulated part poses are shown while stepping or running. Stop or Reset
restores the editable assembly pose. Joint controls are disabled during
simulation. Joint couplings belong to the RobotModel: MuJoCo enforces them
with joint equality constraints, and the deterministic backend enforces them
exactly. MuJoCo also supports assembly loop closures; other backends report
a clear diagnostic. Generated parts default to collision enabled and carry
bounded convex hulls from the CAD physical-part view. MuJoCo collides against
those hulls, while the deterministic backend uses boxes centred on their
geometry bounds. Flat parts use a minimum-thickness box with a Simulation
panel warning. One hull fills bores and U-shaped openings. Multiple hulls
per link, potentially from CoACD, require a separate future design decision.
Component dimensions are currently authored in `MotorShaftBearings.hx`; editing
that source and reopening the project regenerates the preview.

## Structural

`machinekit.structural` builds machine bases and frames from named 3D points
and straight members, rather than the connector-mating model above: a frame is
one rigid weldment, not a kinematic mechanism, so members carry no assembly
joints. Each cross-section (`RectTube`, `RoundTube`, `Angle`, `Channel`,
`FlatBar`, `TSlotExtrusion`) implements `StructuralProfile` and extrudes
itself along local +Z to a caller-chosen length, the same axis convention as
every other generator. The numeric `TSlotExtrusion(size)` constructor is a
generic square 2020/4040-style profile with a T-slot channel on each face and
a centre bore, all proportional to its `size` rather than a vendor's literal
table. Use `TSlotExtrusion.forProfile()` for catalog-backed MISUMI HFS5
profiles; those entries retain their slot, bore, and cross-section dimensions
and provenance. Geometry currently cuts one simplified slot into each outside
face. Multi-slot patterns for 2040, 2060, and 4040 profiles remain a known
limitation.

`FrameAssembly` registers named points with `point(name, x, y, z)`, then
members with `member(name, start, end, profile)`. Optional `FrameEndCut.Mitre(setback)`
and `FrameEndCut.Cope(setback)` treatments remove stock from either endpoint;
`Square` is the default. `geometry(name)` extrudes
and places the resulting member envelope in world space between its two points; a member's local
+Y (a channel's web-to-flange direction, a tube's height, ...) follows an
optional `reference` vector projected perpendicular to the member's axis,
defaulting to +Z (or +Y for a nearly vertical member); a reference parallel to
the member is rejected. `length()` reports the node-to-node distance, while
`cutLength()` and `cutList()` aggregate the post-cut stock lengths by profile
designation. The cut treatments currently describe trimmed profile envelopes;
mitres are clipped with a profile-aware sloped plane and copes remove a curved
notch sized from the profile envelope.

## Transmission

`machinekit.transmission` has `SpurGear` (full-depth involute teeth with explicit
profile shift and pitch-circle backlash, sampled as a polyline profile, extruded along +Z) and `Rack` (its straight-flank,
infinite-radius limit, extruded along +X with teeth along +Z). `GearPair.mesh(a, b)`
computes the profile-shifted centre distance and ratio for two gears with matching module and pressure angle and a
placement pose for `b` relative to `a`, rotated so a tooth space of `b` meets
`a`'s tooth at the mesh point. Pressure angle inputs are limited to 14.5°–25°;
`GearPair` also derives the operating pressure angle after profile shifting and
rejects combinations whose base circles cannot produce a real involute mesh.
Gears reject tooth counts whose selected profile shift is insufficient to avoid
undercut. A zero-shift gear therefore keeps the usual `ceil(2 /
sin²(pressureAngle))` limit, while a positive shift can model a small pinion.
Backlash reduces pitch-circle tooth thickness and is reported in the gear
designation and pair properties; it does not change centre distance.

`Sprocket` (roller chain) and `TimingPulley` (belt) use a coarser simplified
tooth outline than `SpurGear` — straight flanks between two radii rather than
sampled involute curves — since neither the ANSI B29.1 seating-curve nor the
rounded-trapezoid GT2/HTD profile is modelled exactly. `Sprocket.forChain()` uses tabulated ANSI 25/35/40/50 pitch and roller
diameter from Renold; the free-form constructor is marked generic. `TimingPulley`
requires a `TimingBeltProfile` (GT2, HTD, T5, XL, or explicit Custom) and puts
the belt family in its designation. Its outside diameter is the pitch diameter
less twice that profile's pitch-line differential.

## Assembly

`MachineComponent.massProperties()` computes mass, centre of mass, and centroidal
inertia from Preview geometry and material density. Mass is in kg, positions in
mm, and inertia in kg mm². Components with catalog values can declare mass with
an explicit centre of mass and optionally inertia. When a declared mass has no
inertia, its tensor is `null`.
`MachineAssembly.massProperties(?state)` rotates and combines member tensors at
the solved poses; it returns `null` inertia and lists the affected member IDs in
`unaccountedInertia` if any tensor is missing. Its `unaccounted` list continues
to identify extra BOM items omitted from the mass rollup. `addBomItem(item,
quantity, Point(kg, centreOfMass))` accounts for BOM-only masses fixed in the
assembly frame. Use `Attached(kg, instanceId, centreOfMass)` to attach a mass to
a member; its centre is in that member's frame and follows its joint pose.
`connectPorts(..., line, mass)` accepts either form. Its default mass is
`Unknown`, since a line has no implied attachment end. Mass is per item; both
forms use a point-mass inertia approximation at the resolved centre.

`MachineAssembly.addTo` checks CAD joints and tree structure, so a partially
wired assembly can still be shown and weighed. Call `validate()` to check
service connections and required inputs. Included member ports can be wired by
their prefixed member paths, but they do not appear in the containing assembly's
public `portNames()` or `port(name)` until `exposePort` publishes them. A required
input must be connected or explicitly exposed at each level. `validate()` also
traces each connected required consumer to a Supply or an exposed assembly
input, reporting the chain if it ends unfed. `upstream()` checks connection
validity while allowing other required inputs to remain unconnected. It returns
`{port, external}`: `external` is true when the trace ends at an exposed input,
and false when it reaches a Supply.
Supply and Consumer connection arguments
can be given in either order. Components declare service passages with
`addBridge` and changes of service kind with `addConversion`; `upstream()`
follows only those declared routes.

`include(id, assembly, ?pose)` keeps a snapshot of the included assembly as a
subassembly; records on the including level name its members by path
(`arm/link1`). Each level is four parts: `MechanicalAssembly` (members,
subassemblies, connectors, joints, couplings), `ServiceNetwork` (port
connections and exposures), `DriveSystem` (transmissions, belt paths, motors,
encoders) and `AssemblyInventory` (BOM extras). Checks, mass, `addTo` and
`definition()` work on one flattened view of all levels, built on demand.
`describe()` saves the nested ProjectKit definition with each level's MachineKit
facts beside it; `decode(text, ?registry)` rebuilds members through a
`ComponentRegistry` (`MachineKitComponents.defaultRegistry()` unless given).
Parts state what they are for through typed facets (`SuctionFacet`,
`GripFacet`, `WeldingSupplyFacet`, ...), each looked up with its class's `of`.

`machinekit.assembly` composes standalone `MachineComponent`s into small
machines, the same way `examples/MotorShaftBearings.hx` does, but as reusable
library classes rather than one-off scripts:

- `FlangeBearingAssembly` mounts a `DeepGrooveBearing` in a
  `FlangeBearingHousing` (bore and four screws along the same axis, like
  `NemaStepper`'s mounting face — not a classic two-bolt base-mount housing)
  with four `SocketHeadCapScrew`s. `addTo(model, id, ?pose)` places the whole
  block, centres the bearing in the bore, and seats the screws with their heads
  on the housing's outer face, long enough to pass through it. The bearing's own
  `front`/`axis`/`back` connectors stay reachable as `'<id>-bearing'` for
  mating a shaft through it.
- `PillowBlock` is the base-mounted UCP-style unit. Its UCP204–UCP213 catalog
  rows carry the shaft height, base envelope, two-bolt spacing, and mounting
  interface; its connectors expose the shaft axis, base, and bolt centres.
- `ShaftCoupling` is a reducer-capable rigid sleeve with independent bores and
  configurable radial tapped set-screw holes. Preview cuts one hole per
  configured screw, while Envelope keeps the turned sleeve and bores. Set-screw
  threads remain semantic; `billOfMaterials()` adds one screw per hole.
- `LinearAxis` drives a semantic `LeadScrew` through a `ShaftCoupling` and
  matching `LeadScrewNut`. Its `LinearGuideSystem` keeps two round guide rods, `LM8UU` linear
  bearings, and their shaft/housing fit intent together. `setTravel(state, millimetres)` couples screw rotation to carriage
  translation through the nut lead (pitch × starts, with handedness) and enforces the stroke. The carriage slide
  is parented to the fixed motor frame, so the carriage stays oriented while
  the screw rotates. Two `FlangeBearingAssembly`s support the screw near its ends; a
  `RectTube` member remains the layout frame rail. Guide and housing mounting
  details are still a preview rather than a structurally designed frame. The
  default axis uses a right-hand Tr10 × 2 single-start thread; other screw
  diameters need an explicit `LeadScrewThread`. Thread flanks are not modelled,
  and a family label does not assert a verified standard size.
- `LinearGuideSystem.forRailProfile()` provides a catalog-backed profile-rail
  alternative alongside the round-rod guide. The current `HIWIN` `MGN12C`
  row carries rail and block envelopes, mounting-hole pitch, block spacing, and
  explicit end margins. `LinearRailSystem.assembly()` exposes one prismatic
  joint per block, and its BOM contains the cut rail and matching blocks. The
  generated solids are nominal envelopes; catalog provenance and connector
  frames carry the interface dimensions used for layout.
- `LinearAxis.forRailProfile("MGN12C", ...)` selects that guide for the
  lead-screw axis. The carriage mounts the rail block through a fixed joint,
  the rail-to-block interface is recorded as a prismatic closure, and the
  axis travel limits and BOM include the profile rail hardware.

## Sheet stock and cutting plans

The stand-alone `examples/picking-station/` package depends on MachineKit and
ManufacturingKit. Its `pickingstation` source package contains the station
geometry and picking example code; MachineKit itself contains no picking
domain classes. The project connects rectangular cut blanks to
finished picking-bench and tote-station panels. Its versioned
`materia.project.json.sheet.json` companion stores stock specifications, part
requirements, manually placed blanks, ordered guillotine cuts, physical sheets,
executions, and reusable remnants. Length values carry an explicit unit and
exports normalize dimensions to millimetres. Plan coordinates use a lower-left
origin; 90° means a clockwise quarter-turn of the blank. The SVG is a planning
drawing, not a machine toolpath.

Run the full sample workflow, including CSV/SVG export, cutting confirmation,
reopen, and reuse of a recorded remnant:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/picking-station/haxeon.json -- \
  demo /tmp/picking-station.sheet.json /tmp/picking-station-exports
```

The example includes its companion record. Build and launch it in Materia:

```sh
./haxeon/scripts/haxeon build --project=app/haxeon.json
./app/run-built.sh --project=./machinekit/examples/picking-station/materia.project.json
```

The checked-in companion includes one available example sheet (`sheet-001`),
so the first plan is ready to select and validate. The Inspector's **Cut
planning** section defaults to **Nominal stock** specification dimensions; you
can switch to a particular physical sheet to plan against its measured size.
The collapsed **Operations** section lets you register stock, allocate or
release it, confirm completed cutting, and inspect resulting blanks and
remnants. Planning, validation, save, and CSV/SVG export are available without
physical inventory. A successful execution consumes the source and adds blank
and remnant records in one save. The second plan uses the first plan's
`shelf-drop` remnant. The separate finished-part BOM continues to count parts,
not consumed sheets.

For a new copy, `init <path>` creates an initial versioned companion record and
refuses to overwrite an existing file. The command-line inventory flow uses
`register`, `allocate`, `preview`, `cancel`, `execute ... CONFIRM`, `status`, and
`bom`; its project path is relative to the example directory, so use
`materia.project.json.sheet.json`. `preview` is read-only; `execute` requires an
allocation and explicit confirmation. Keep real operations in working copies
of both the project and its `.sheet.json` companion; the checked-in files are
shareable examples. The first release supports rectangular
blanks and straight guillotine cuts only. Automatic nesting, arbitrary
contours, machine toolpaths, and warehouse integrations are outside its scope.
The CAD viewport shows the authored sample layout; numeric edits update the
Inspector's layout preview and exports, but do not regenerate the 3D scene
artifact.

## Robotics

`machinekit.robotics` has mechanical generators for mounting a robot arm and
its tooling, not a link to `robotkit`'s runtime model (which references mesh
files by path, not CadKit geometry, so the bridge is at the level of a shared
`AssemblyModel`/BOM workflow, not a shared type):

- `RobotFlange` uses an ISO 9409-1-style bolt pattern sized by pitch-circle diameter
  from the standard's table (bolt count and size, pilot diameter, pin), with
  proportional outer diameter and thickness. Its mounting face is z=0 with the
  pilot boss standing proud of it; `mountingCutout()` cuts the matching blind
  pilot recess, bolt and pin holes. The raised pilot swaps the ISO interface
  roles, so its designation says style. Overriding the bolt count drops that
  style designation.
- `EndEffectorPlate` adapts a `RobotFlange`'s bolt pattern to a tool bolt
  circle outside the flange's bolts, the same cut-and-expose-a-new-pattern
  shape as `MotorPlate` in `MotorShaftBearings.hx`.
- `Pedestal` is a column stand with a configurable anchor circle, optional
  base gussets, leveling feet, and a central cable path, plus a
  `RobotFlange`-matching mount at its top. Its `top` connector points down
  into the column, so a flange mated there sits face-down with its pilot in
  the recess. `billOfMaterials()` adds one floor anchor screw per hole.

Run the smoke tests after building CadKit's native library:

```sh
./machinekit/scripts/test-haxeon
```

Set `CADKIT_BUILD_ROOT` when CadKit was built somewhere other than
`cadkit/build/debug`.
