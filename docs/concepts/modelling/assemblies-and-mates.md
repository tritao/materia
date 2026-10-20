---
title: Assemblies and mates
description: How parts are placed and connected, why joints and mates are different things, and how loops are closed.
status: draft
---

# Assemblies and mates

## The problem

A machine is many parts, and most of them should not move relative to each
other while a few must. An assembly has to say three things:

- **where each part sits** when the machine is designed;
- **how parts may move**: this carriage slides, that pulley turns;
- **what must stay true** as they move: a linkage stays closed, a belt follows
  its axis.

It also has to say these things in a way that survives editing: if a part
changes size, its neighbours should follow without hand-placed coordinates
breaking.

## The model

Materia separates two ideas that CAD tools often blur:

> **Mates position parts. Joints move them.**

### Parts, definitions and connectors

A **component definition** is a reusable part: geometry plus named
**connectors**, which are frames on the part (a mounting face, a shaft axis, a
bolt hole). An **occurrence** is one placed copy of a definition. Connectors
belong to the definition, so every copy of a motor has the same `shaft`
connector.

A **grounded** occurrence is fixed to the world: nothing moves it.

### Joints: the machine's degrees of freedom

A **joint** connects a child occurrence to a parent through one connector on
each. The child's connector lands on the parent's, then moves along the joint's
axis by the joint's coordinate.

| Type | Coordinate | Example |
|---|---|---|
| `fixed` | none | a bracket bolted to a frame |
| `prismatic` | distance along the axis | a carriage on a rail |
| `revolute` | angle about the axis, with limits | a hinge |
| `continuous` | angle, unlimited | a pulley or a screw |

Joints form a **tree**: each occurrence hangs from at most one parent, and a
part's pose follows from its ancestors' coordinates (forward kinematics). The
tree is what is planned, simulated and driven.

### Closures: loops the tree cannot express

Real mechanisms have loops. A four-bar linkage, a scissor lift or an
excavator's hydraulic cylinder connects back to a part that already has a
parent. Such a connection becomes a **closure**: not a tree edge, but a
condition that two connector frames must meet.

- Closures can also use joint types with more than one coordinate, which tree
  joints never do: `spherical`, `cylindrical` and `planar`.
- A closure has no coordinate of its own. Its loop is satisfied by moving the
  tree joints along the loop.
- A **driven** joint is an input of the mechanism (where the motor or cylinder
  acts). Loop solves never move it; the other joints on its loops follow it.

Whether a connection is a tree joint or a closure is decided when it is
created, then saved, so saved states and motion tracks keyed by joint id stay
valid.

### Couplings: joints that follow other joints

A **coupling** ties one joint's coordinate to others:
`follower = Σ (ratio × leader + offset)`. A screw follows its axis; a CoreXY
motor follows both axes. Couplings carry the transmission's efficiency,
stiffness and backlash. See [Transmissions](transmissions.md) and
[Conventions](../foundations/conventions.md#couplings-and-transmissions).

### Mates: placement constraints

A **mate** is a geometric relation between two connectors that positions parts
without creating motion: `coincident`, `coaxial`, `planar` (with an offset),
`parallel`, `perpendicular`, `distance`, `angle` and `lock`.

A mate solve moves the free parts (and free joints) until the mates hold, then
reports what happened:

- **unsatisfied**: mates that could not be met;
- **implied**: mates already guaranteed by others (redundant);
- **still free**: parts that can still move under their mates.

When a set of mates leaves a part exactly one way to move, the editor can
**convert it to a joint**: a sliding or turning mate pair becomes a prismatic
or revolute joint.

### Design data and state

The **assembly definition** is versioned design data: definitions,
occurrences, joints, couplings, mates, plus the machine's actuators, sensors
and switches. An **assembly state** is separate: joint coordinates and root
poses for one configuration. World poses, flattened ids and the compiled
kinematic model are always derived, never saved.

### Diagnosis

Solving reports more than success or failure. Loop and mate solves report the
remaining degrees of freedom and which constraints depend on each other, and
they tell four situations apart:

- **conflicting**: the constraints cannot all hold (`x = 0` and `x = 1`);
- **redundant**: a constraint repeats what others already impose;
- **degenerate**: the count is right, but the pose is a special case where
  the constraints happen to line up;
- **blocked**: a joint limit stops the solve.

## In Materia

| Layer | Role | Key types |
|---|---|---|
| ProjectKit | The data: definitions, occurrences, joints, couplings, mates, state | `AssemblyDefinition`, `KinematicJoint`, `AssemblyMate`, `AssemblyJointCoupling`, `AssemblyStateRecord`, `AssemblyDefinitionCodec` |
| CadKit | Building and solving | `AssemblyModel` (builder), `AssemblyState`, `AssemblyLoopSolver`, `AssemblyMateSolver`, `AssemblyKinematics`, `ConstraintDiagnosis` |
| MachineKit | Machines from components | `MachineAssembly` (mechanical, drives, services, inventory), `AssemblyIntegrity` |
| KinematicsKit | The solver core | `KinematicModel`, Levenberg–Marquardt with closure tasks |

All assembly solving runs in Haxe on KinematicsKit; the CadKit native core only
supplies geometry such as face and edge axes.

`MachineAssembly` adds machine-level checks on top. **Structure** checks catch
a part with two parents or a cycle. **`AssemblyIntegrity`** checks that the
machine is physically put together:

- mated parts touch, and everything is held up;
- nothing overlaps at rest, except modelled meshes such as screw and nut or
  belt and pulley;
- sampled motion sweeps stay clear;
- screw, rack and gear geometry agree with their coupling ratios.

In the editor, the Inspector lists each movable joint's coordinate and each
part's mates. Parts with mates can be dragged under their mates; other parts
are dragged by solving the joints that move them. Mates added in the editor are
saved as an overlay on the project; the generated assembly is never edited
directly.

## What it isn't

- **Not physics.** Assembly solves are geometric: they find poses that satisfy
  constraints. Forces, contacts and dynamics belong to the
  [simulation](../execution/simulation-worlds.md), which is built from the
  assembly.
- **Not the robot model.** Planning uses a robot model compiled from the
  assembly, in SI units with joint zero at the starting pose (see
  [Conventions](../foundations/conventions.md#joint-coordinates)).
- **Mates are not joints.** A mate never becomes a tree edge and takes no part
  in loop solves or motion planning. Converting a mate to a joint is an
  explicit step.
- **Integrity sweeps are samples, not swept volumes.** A collision between
  samples can be missed.

## See also

- [The life of a machine](../start/life-of-a-machine.md), step 3
- [Machine components](machine-components.md)
- [Transmissions](transmissions.md)
- [The kinematic forest](../motion/kinematic-forest.md)
- [Identity](../foundations/identity.md)

## Open questions

- **"Mate" names two things in the API.** `AssemblyModel.mate` /
  `mateOnAxis` and `MachineAssembly.addMate` / `addMateOnAxis` create
  **joints**, not `AssemblyMate` records. Renaming the builder methods (to
  `join` / `addJoint`) would match the schema's vocabulary.
- **"planar" is both a closure joint type and a mate kind.** Both mean "a point
  on a plane, normals parallel"; only the mate has an offset.
- **MachineKit cannot express driven joints or closure-only types.**
  `MechanicalAssembly.copyJoint` drops `driven`, `MachineAssembly` has no
  `drive()`, and `addJoint` rejects `spherical`, `cylindrical` and `planar`.
- **Two rules for loop-closing joints.** `AssemblyModel` turns a joint that
  closes a loop into a closure automatically; `MachineAssembly` reports
  `assembly.multiple-parents` and asks for `addConstraint`.
- **Closure diagnosis is not shown in the Inspector yet**; only the mate solve's
  report reaches the status bar.
