---
title: Conventions
description: Frame handedness, axes, units, rotations, joint coordinates and coupling ratios, stated once for every kit.
status: draft
---

# Conventions

Every kit in Materia uses the same conventions for space, units and motion.
This page is the one place they are stated; kit documentation should link here
instead of restating them.

## The problem

A machine passes through many kits on its way from geometry to hardware: CadKit
and MachineKit model it, KinematicsKit and MotionKit plan it, SimKit simulates
it, RobotKit runs it. If two of them disagreed about which way is up, what unit
a length is in, or which side of a belt a ratio is measured from, the error
would show up far from where it was made: as a tool moving the wrong way, a
limit off by a factor of 1000, or a motor spinning backwards.

## The model

### Space

- **Right-handed frames, +X forward, +Y left, +Z up.** Gravity points along −Z.
- **Transforms are named `A_T_B`**: `A_T_B` maps coordinates expressed in frame
  `B` into frame `A`. Names compose like fractions:
  `world_T_tool = world_T_flange * flange_T_tool`.
- **Matrices are column-major and multiply column vectors**, the same as
  SceneKit.
- **Quaternions are ordered `x, y, z, w`.**
- **Angles are radians.**

External conventions (ROS, glTF's +Y-up, a camera's optical frame) are
converted once, at the boundary where the data enters Materia. The internal
model never changes to match them.

### Units

**Everything that moves, is planned, simulated, sensed or executed is SI**:
metres, radians, seconds, kilograms, newtons, pascals. Feeds are metres per
second, angular rates radians per second.

**Authored geometry is the exception.** Machine components and assemblies are
modelled in millimetres, because parts are specified, bought and drawn in
millimetres. This applies to MachineKit component dimensions, assembly
geometry, assembly joint travel, and some MachineKit fluid quantities (phase
travel in mm, phase rates in mm/s; supply pressure stays in Pa). GearKit and
StockKit's `RingStock` also use millimetres, to match gear standards. Cartesian
`Stock`, CamKit and ToolpathKit use metres.

**The conversion happens in one place.** A scene artifact records its length
unit (`metresPerUnit`). When an assembly is compiled into a robot model
(`AssemblySimulationBridge.toRobotModel` in `robotkit/cadbridge`), every length
is scaled to metres: positions, sliding-joint travel and velocity, overtravel,
racking tolerance, and coupling ratios. Mass follows as volume × density ×
scale³. Downstream of that seam, nothing knows the assembly was in millimetres.

### Joints and connectors

- **Joined connector frames coincide.** A joint places the child so that its
  named connector lands exactly on the parent's, then moves it by the joint's
  coordinate.
- **A joint acts along a unit axis expressed in the parent connector frame.**
  The default axis is the connector's **+Y**. A joint can name a different axis
  (`addMateOnAxis` in MachineKit, `mateOnAxis` in CadKit; despite their names,
  these create joints, see [Assemblies and mates](../modelling/assemblies-and-mates.md)).
- **MachineKit component frames put the component's own axis along +Z** (a
  rail's length, a motor's shaft), and their connectors' +Y points along that
  +Z, so default joints act along the component's axis.
- **`AxisBuilder` uses world-aligned connectors.** Its axes are therefore
  world directions: in the gantry, Z slides along world −Z, so a positive Z
  coordinate moves the tool down.

### Joint coordinates

An assembly joint's value is measured from the CAD placement, in assembly units
(millimetres for sliding joints, radians for rotating ones).

A robot model's joint coordinate is **zero at the authored initial pose** and
SI:

```
q_robot = (value − initial) × scale      (scale = 1 for rotating joints)
```

The joint's limits are rebased the same way. So a gantry authored with X at
750 mm has `q = 0` there, with limits −0.75 m to +0.75 m for 0 to 1500 mm of
travel.

### Couplings and transmissions

A **coupling** makes one joint follow others. Assemblies (`AssemblyJointCoupling`)
and robot models (`JointCoupling`) write it the same way:

```
follower = Σ (ratio_i × leader_i + offset_i)
```

The **leader is the coordinate you plan in** (the axis), and the follower moves
with it (the screw, pulley or motor). The offset is in follower units.

A RobotKit **transmission** (`SimpleTransmission`) maps an actuator to the
joint it drives. It is written from the joint's side:

```
joint = offset + actuator / ratio      ⇔      actuator = ratio × (joint − offset)
```

The offset is in joint units.

**In both, the ratio means the same thing: motor-side units per axis-side
unit.** For a GT2 belt on a 20-tooth pulley, that is 2π / 40 mm, or
0.157 rad/mm in the assembly and 157 rad/m in the robot model. The two
formulas look inverted only because one is written from the follower's side
and the other from the joint's.

A follower with several leaders, such as a CoreXY motor following both axes,
has one coupling per leader, and the follower's value is the sum of all of
them.

## In Materia

| Convention | Where it is defined or enforced |
|---|---|
| Frames, transforms, units, quaternions | `robotkit/ARCHITECTURE.md` (Frames and units) |
| Length unit of a scene | `SceneArtifact.metresPerUnit` (`projectkit`) |
| mm → SI conversion | `AssemblySimulationBridge.toRobotModel` (`robotkit/cadbridge`) |
| Joint axis and placement | `AssemblyModel.mate` / `mateOnAxis` (CadKit), `MachineAssembly.addMate` / `addMateOnAxis` (MachineKit) |
| World-aligned axis connectors | `AxisBuilder.connect` and `AxisBuilder.slide` (MachineKit) |
| Couplings | `AssemblyJointCoupling` (projectkit), `JointCoupling` (RobotKit) |
| Transmissions | `SimpleTransmission` (RobotKit) |

## What it isn't

- **Not a display convention.** These rules govern data and APIs. How a panel
  formats a value for a person is a separate, display-side choice.
- **Not negotiable per kit.** A kit that needs another convention (a camera's
  optical frame, glTF's +Y-up) converts at its boundary and documents that
  conversion.
- **Millimetres are not "the CAD unit".** They are the authoring unit for
  assemblies. A CadKit document can record another unit, and the conversion
  reads `metresPerUnit`; it does not assume millimetres.

## See also

- [The life of a machine](../start/life-of-a-machine.md), steps 3–5
- [Frames and transforms](frames.md)
- [Transmissions](../modelling/transmissions.md)
- [Assemblies and mates](../modelling/assemblies-and-mates.md)
