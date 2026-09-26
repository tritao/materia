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
| `SocketHeadCapScrew` | M3–M12, ISO 4762 heads | `clearanceHole()` (ISO 273), `tapHole()`, `counterboreHole()` (DIN 974-1) |
| `HexBolt` | M3–M12, ISO 4017 (fully threaded) | `clearanceHole()`, `tapHole()`, `counterboreHole()` |
| `HexNut` | M3–M12, ISO 4032 | `pocket()` for a trapped-nut recess |
| `FlatWasher` | M3–M12, ISO 7089 | — |
| `NemaStepper` | NEMA 17, 23, 34 frame interfaces plus named motor variants | `mountingCutout()`, `mountScrew()`, `boltPattern()` |
| `ParallelKey` | DIN 6885-1 form B (square ends), by shaft diameter | — |
| `RetainingRing` | DIN 471 external, by shaft diameter | `grooveSpec()` (d2, m) |
| `ShaftCollar` | set-screw type, by bore diameter | — |
| `SteppedShaft` | — (built from arbitrary sections) | keyway and retaining-ring groove cuts, `diameterAt()` |
| `FlangeBearingHousing` | — (sized from a `DeepGrooveBearing`) | `mountScrewPart()` |
| `Bushing` | — (proportional to bore diameter) | — |
| `ShaftCoupling` | — (proportional to the larger bore) | `setScrewPart()` |
| `LinearBearing` | LM8UU–LM20UU | — |
| `LeadScrewThread` | semantic metric trapezoidal or ACME family, diameter, pitch, starts and hand | `lead = pitch × starts` |
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
own name when given one, for a `RetainingRing` mated the same way.

`examples/MotorShaftBearings.hx` mounts a NEMA 17 motor on a plate with four
M3 screws and carries an output shaft on two 608 bearings through a continuous
coupling joint. The shaft steps down past the outboard bearing to carry a
retaining ring and an output key. It also produces the aggregated BOM.

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
and provenance.

`FrameAssembly` registers named points with `point(name, x, y, z)`, then
members with `member(name, start, end, profile)`. `geometry(name)` extrudes
and places a member in world space between its two points; a member's local
+Y (a channel's web-to-flange direction, a tube's height, ...) follows an
optional `reference` vector projected perpendicular to the member's axis,
defaulting to +Z (or +Y for a nearly vertical member); a reference parallel to
the member is rejected. `cutList()` aggregates member lengths by profile
designation. Members run point to point and are not trimmed at joints, so
lengths and the cut list are centreline lengths, not saw-cut lengths.

## Transmission

`machinekit.transmission` has `SpurGear` (standard full-depth involute teeth,
sampled as a polyline profile, extruded along +Z) and `Rack` (its straight-flank,
infinite-radius limit, extruded along +X with teeth along +Z). `GearPair.mesh(a, b)`
computes the standard centre distance and ratio for two gears with matching module and pressure angle and a
placement pose for `b` relative to `a`, rotated so a tooth space of `b` meets
`a`'s tooth at the mesh point. Pressure angles are limited to 14.5°–25°.
Unshifted full-depth gears below `ceil(2 / sin²(pressureAngle))` teeth are
rejected because this generator does not model undercut or profile shift.

`Sprocket` (roller chain) and `TimingPulley` (belt) use a coarser simplified
tooth outline than `SpurGear` — straight flanks between two radii rather than
sampled involute curves — since neither the ANSI B29.1 seating-curve nor the
rounded-trapezoid GT2/HTD profile is modelled exactly. `Sprocket.forChain()` uses tabulated ANSI 25/35/40/50 pitch and roller
diameter from Renold; the free-form constructor is marked generic. `TimingPulley`
requires a `TimingBeltProfile` (GT2, HTD, T5, XL, or explicit Custom) and puts
the belt family in its designation. Its outside diameter is the pitch diameter
less twice that profile's pitch-line differential.

## Assembly

`machinekit.assembly` composes standalone `MachineComponent`s into small
machines, the same way `examples/MotorShaftBearings.hx` does, but as reusable
library classes rather than one-off scripts:

- `PillowBlock` mounts a `DeepGrooveBearing` in a
  `FlangeBearingHousing` (bore and four screws along the same axis, like
  `NemaStepper`'s mounting face — not a classic two-bolt base-mount housing)
  with four `SocketHeadCapScrew`s. `addTo(model, id, ?pose)` places the whole
  block, centres the bearing in the bore, and seats the screws with their heads
  on the housing's outer face, long enough to pass through it. The bearing's own
  `front`/`axis`/`back` connectors stay reachable as `'<id>-bearing'` for
  mating a shaft through it.
- `LinearAxis` drives a semantic `LeadScrew` through a `ShaftCoupling` and
  matching `LeadScrewNut`. Its `LinearGuideSystem` keeps two round guide rods, `LM8UU` linear
  bearings, and their shaft/housing fit intent together. `setTravel(state, millimetres)` couples screw rotation to carriage
  translation through the nut lead (pitch × starts, with handedness) and enforces the stroke. The carriage slide
  is parented to the fixed motor frame, so the carriage stays oriented while
  the screw rotates. Two `PillowBlock`s support the screw near its ends; a
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
- `Pedestal` is a column stand with a floor bolt pattern at its base and a
  `RobotFlange`-matching mount at its top; its `top` connector points down into
  the column, so a flange mated there sits face-down with its pilot in the
  recess.

Run the smoke tests after building CadKit's native library:

```sh
./machinekit/scripts/test-haxeon
```

Set `CADKIT_BUILD_ROOT` when CadKit was built somewhere other than
`cadkit/build/debug`.
