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
| `HexBolt` | M3–M12, ISO 4017 heads | `clearanceHole()`, `tapHole()`, `counterboreHole()` |
| `HexNut` | M3–M12, ISO 4032 | `pocket()` for a trapped-nut recess |
| `FlatWasher` | M3–M12, ISO 7089 | — |
| `NemaStepper` | NEMA 17, 23, 34 | `mountingCutout()`, `mountScrew()`, `boltPattern()` |
| `ParallelKey` | DIN 6885-1 form A, by shaft diameter | — |
| `RetainingRing` | DIN 471 external, by shaft diameter | — |
| `ShaftCollar` | set-screw type, by bore diameter | — |
| `SteppedShaft` | — (built from arbitrary sections) | keyway and retaining-ring groove cuts, `diameterAt()` |
| `PillowBlockHousing` | — (sized from a `DeepGrooveBearing`) | `mountScrewPart()` |

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
`FlatBar`) implements `StructuralProfile` and extrudes itself along local +Z
to a caller-chosen length, the same axis convention as every other generator.

`FrameAssembly` registers named points with `point(name, x, y, z)`, then
members with `member(name, start, end, profile)`. `geometry(name)` extrudes
and places a member in world space between its two points; a member's local
+X (a channel's open side, an angle's leg corner, ...) follows an optional
`reference` vector projected perpendicular to the member's axis, defaulting to
+Z (or +Y for a nearly vertical member). `cutList()` aggregates member lengths
by profile designation.

## Transmission

`machinekit.transmission` has `SpurGear` (standard full-depth involute teeth,
sampled as a polyline profile, extruded along +Z) and `Rack` (its straight-flank,
infinite-radius limit, extruded along +X with teeth along +Z). `GearPair.mesh(a, b)`
computes the standard centre distance and ratio for two same-module gears and a
placement pose for `b` relative to `a`.

## Assembly

`machinekit.assembly` composes standalone `MachineComponent`s into small
machines, the same way `examples/MotorShaftBearings.hx` does, but as reusable
library classes rather than one-off scripts:

- `PillowBlock` mounts a `DeepGrooveBearing` in a flange-style
  `PillowBlockHousing` (bore and four screws along the same axis, like
  `NemaStepper`'s mounting face — not a classic two-bolt base-mount housing)
  with four `SocketHeadCapScrew`s. `addTo(model, id, ?pose)` places the whole
  block and mates the bearing and screws onto it; the bearing's own
  `front`/`axis`/`back` connectors stay reachable as `'<id>-bearing'` for
  mating a shaft through it.
- `LinearAxis` drives a `SteppedShaft` lead screw from a `NemaStepper` through
  a continuous coupling, with a `Carriage` riding it on a prismatic joint.
  Two `PillowBlock`s and a `RectTube` rail (via `FrameAssembly`) represent the
  fixed frame; they are independently placed in the same `AssemblyModel`
  rather than mated to the screw, since a real frame constrains the screw at
  both ends (a statically indeterminate assembly), which this simplified
  kinematic model does not attempt to capture.

## Robotics

`machinekit.robotics` has mechanical generators for mounting a robot arm and
its tooling, not a link to `robotkit`'s runtime model (which references mesh
files by path, not CadKit geometry, so the bridge is at the level of a shared
`AssemblyModel`/BOM workflow, not a shared type):

- `RobotFlange` is an ISO 9409-1 style tool flange (pilot boss, bolt circle,
  and locating pin, sized proportionally to the flange diameter rather than
  from a literal standard table), with a `mountingCutout()` companion like
  `NemaStepper`'s.
- `EndEffectorPlate` adapts a `RobotFlange`'s bolt pattern to a smaller tool
  bolt circle, the same cut-and-expose-a-new-pattern shape as `MotorPlate` in
  `MotorShaftBearings.hx`.
- `Pedestal` is a column stand with a floor bolt pattern at its base and a
  `RobotFlange`-matching mount at its top.

Run the smoke tests after building CadKit's native library:

```sh
./machinekit/scripts/test-haxeon
```

Set `CADKIT_BUILD_ROOT` when CadKit was built somewhere other than
`cadkit/build/debug`.
