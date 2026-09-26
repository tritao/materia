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
Each component also produces the machining it needs:

| Component | Catalog | Companion geometry |
| --- | --- | --- |
| `DeepGrooveBearing` | 625–6205, ISO 15 boundary dimensions | `housingSeat()`, `journalDiameter()` |
| `SocketHeadCapScrew` | M3–M12, ISO 4762 heads | `clearanceHole()` (ISO 273), `tapHole()`, `counterboreHole()` (DIN 974-1) |
| `HexBolt` | M3–M12, ISO 4017 (fully threaded) | `clearanceHole()`, `tapHole()`, `counterboreHole()` |
| `HexNut` | M3–M12, ISO 4032 | `pocket()` for a trapped-nut recess |
| `FlatWasher` | M3–M12, ISO 7089 | — |
| `NemaStepper` | NEMA 17, 23, 34 | `mountingCutout()`, `mountScrew()`, `boltPattern()` |
| `ParallelKey` | DIN 6885-1 form A, by shaft diameter | — |
| `RetainingRing` | DIN 471 external, by shaft diameter | — |
| `ShaftCollar` | set-screw type, by bore diameter | — |
| `SteppedShaft` | — (built from arbitrary sections) | keyway and retaining-ring groove cuts, `diameterAt()` |
| `PillowBlockHousing` | — (sized from a `DeepGrooveBearing`) | `mountScrewPart()` |
| `Bushing` | — (proportional to bore diameter) | — |
| `ShaftCoupling` | — (proportional to the larger bore) | `setScrewPart()` |
| `LinearBearing` | LM8UU–LM20UU | — |
| `LeadScrewNut` | — (proportional to screw diameter) | `travelPerRevolution()`, `rotationFor()`, `mountScrewPart()` |

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
every other generator. `TSlotExtrusion` is a square 2020/4040-style profile
with a T-slot channel on each face and a centre bore, all proportional to its
`size` rather than a vendor's literal table, sized with margin so adjacent
faces' slot heads and the bore never intersect and sever the corner posts.

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
computes the standard centre distance and ratio for two same-module gears and a
placement pose for `b` relative to `a`, rotated so a tooth space of `b` meets
`a`'s tooth at the mesh point. Pressure angles are limited to 14.5°–25°.

`Sprocket` (roller chain) and `TimingPulley` (belt) use a coarser simplified
tooth outline than `SpurGear` — straight flanks between two radii rather than
sampled involute curves — since neither the ANSI B29.1 seating-curve nor the
rounded-trapezoid GT2/HTD profile is modelled exactly. Both are parametric by
pitch and tooth count, not a vendor catalog. `Sprocket` derives its root and
outside diameters from an optional roller diameter (default 0.625 × pitch);
`TimingPulley`'s outside diameter is the pitch diameter less twice the belt's
pitch-line differential (defaulted by pitch for the GT2/HTD family, 0.254 mm
for 2 mm GT2).

## Assembly

`machinekit.assembly` composes standalone `MachineComponent`s into small
machines, the same way `examples/MotorShaftBearings.hx` does, but as reusable
library classes rather than one-off scripts:

- `PillowBlock` mounts a `DeepGrooveBearing` in a flange-style
  `PillowBlockHousing` (bore and four screws along the same axis, like
  `NemaStepper`'s mounting face — not a classic two-bolt base-mount housing)
  with four `SocketHeadCapScrew`s. `addTo(model, id, ?pose)` places the whole
  block, centres the bearing in the bore, and seats the screws with their heads
  on the housing's outer face, long enough to pass through it. The bearing's own
  `front`/`axis`/`back` connectors stay reachable as `'<id>-bearing'` for
  mating a shaft through it.
- `LinearAxis` drives a `SteppedShaft` lead screw from a `NemaStepper` through
  a `ShaftCoupling`, with a `Carriage` riding it on a prismatic joint whose
  limits keep it between the two `PillowBlock`s. The pillow blocks (whose
  bearing bore must match the screw) sit near each end of the screw, and a
  `RectTube` rail (via `FrameAssembly`) runs alongside it, clear of the
  carriage, housings and screw heads. They are placed from the screw's layout in
  the same `AssemblyModel` rather than mated to it, since a real frame
  constrains the screw at both ends (a statically indeterminate assembly),
  which this simplified kinematic model does not attempt to capture.

## Robotics

`machinekit.robotics` has mechanical generators for mounting a robot arm and
its tooling, not a link to `robotkit`'s runtime model (which references mesh
files by path, not CadKit geometry, so the bridge is at the level of a shared
`AssemblyModel`/BOM workflow, not a shared type):

- `RobotFlange` is an ISO 9409-1 tool flange sized by pitch-circle diameter
  from the standard's table (bolt count and size, pilot diameter, pin), with
  proportional outer diameter and thickness. Its mounting face is z=0 with the
  pilot boss standing proud of it; `mountingCutout()` cuts the matching blind
  pilot recess, bolt and pin holes. Overriding the bolt count drops the ISO
  designation.
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
