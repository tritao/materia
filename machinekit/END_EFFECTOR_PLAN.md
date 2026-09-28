# End effector (EOAT) plan

An end effector is a mountable assembly with one robot-facing mount and named
working frames. A fixed end effector is one assembly. An automatic changer has a
robot-side stack and a choice of coupled tools, represented by separate assembly
configurations.

## Conventions and design decisions

- Use `machinekit.robotics.EndEffector` for one mountable unit and
  `machinekit.robotics.EndEffectorSet` for a robot-side stack with swappable
  units. `EndEffectorSet.configuration(id)` returns an `EndEffector`.
- `EndEffector` extends `MachineAssembly`, retaining its CAD, BOM, mass, port,
  and joint operations. Add narrow assembly accessors or hooks where subclass
  validation needs the mate graph or connector frames; avoid a second copy of
  assembly state.
- Keep `workingFrame` as the name for any operating point, including camera
  frames. The primary working frame becomes RobotKit's TCP when appropriate.
  RobotKit supports one TCP per `Tool`, so the bridge can create one `Tool` per
  selected working frame.
- Keep generic pneumatics in `machinekit.pneumatic`; keep grippers, changer
  halves, plates, and other robot-specific parts in `machinekit.robotics`.
  Vendor catalog parts can be added alongside these generic components.
- MachineKit connectors and joints act along connector +Y; the conventional
  robot tool approach is +Z. `EndEffectorFrames` owns this axis conversion,
  mm-to-m conversion, and conversion of `AssemblyFrame` to plain position and
  quaternion data.
- Compute end-effector mass in its mount frame. The assembly frame is not
  necessarily the robot flange frame. `RobotFlange` is part of the robot;
  `EndEffectorPlate` belongs to the end-effector BOM.

## E1: Core end effector (`machinekit.robotics`)

Add `EndEffector extends MachineAssembly` with:

- `mount(instanceId, connector)`: the single robot-facing connector on the
  root member.
- `workingFrame(name, instanceId, connector)`: a named frame on a component
  connector or one created by `addMemberConnector`; allow one primary frame.
- `mountTFrame(name, ?state)`: a working frame relative to the mount.
- `massPropertiesAtMount(?state)`: mass, centre of mass, and rotated centroidal
  inertia in the mount frame, preserving unaccounted mass/inertia reporting.
- `validate()`: run inherited structural and service checks, then require a
  mount; require every member to be connected to its root through `Mate`
  operations; resolve all working frames; reject duplicate frame names.
  `Constrain` operations may close loops but do not attach floating members.
  Preserve `MachineAssembly.validate()`'s warning return value.

Tests: an offset, rotated cup on a plate gives analytically correct frame and
mass results, including an `Attached` BOM mass. Missing mount, floating member,
and unknown working frame each fail validation.

## E2: Changer configurations

Add `EndEffectorSet`, representing a robot-side stack, with:

```haxe
changer(name, instanceId, connector, portMap:Array<{robot:String, tool:String}>);
addTool(id, tool:EndEffector);
configuration(toolId):EndEffector;
```

The selected configuration flattens the robot-side stack and selected tool,
mates their changer interfaces, and creates `connectPorts` operations from the
port map. The tool mount must match the declared changer interface. Only the
selected tool and its coupling-time connections appear in the result; there is
no mutable coupled-tool state. The master component's bridges carry pass-through
air. Its lock actuation is an ordinary required pneumatic consumer port. CAD,
BOM, mass, upstream tracing, and validation operate on the configuration.

Tests: two selected tools have different masses and TCPs; a cup's vacuum path
traces through the tool plate and master bridge to robot-side air; an unmapped
required tool port fails validation.

## E3: Generic components and example

Add simple geometry with mass derived from it:

- `machinekit.robotics`: `ToolChangerMaster`, `ToolChangerTool` (N air channels,
  one signal plug, pass-through bridges, and a lock port on the master),
  `ParallelGripper` (required open/close air, stroke, TCP connector), and a
  simple frame bar. Reuse `EndEffectorPlate`.
- `machinekit.pneumatic`: `PneumaticManifold` (one input bridged to N outputs),
  `VacuumGenerator` (`addConversion(air, vacuum)`), and `SuctionCup` (required
  vacuum input and contact connector). Add generic fittings here as needed.

Create `examples/eoat`: changer to frame to cup, with air flowing through a
manifold and generator to the cup, and two selectable tools. Its smoke test
builds both configurations and checks BOM, mass, TCPs, and upstream paths. The
example contains no vendor-specific component.

## E4: RobotKit bridge (`robotkit/cadbridge`)

`EndEffectorBridge.toTool(endEffector, frameName, ?state)` returns a
`robotkit.tool.Tool` with `flangeTTcp` from `mountTFrame`, mass from
`massPropertiesAtMount`, and a `Box` collision shape from envelope bounds in
the mount frame. Switch `MachineAssemblyMassBridge.applyToLink` to mount-frame
mass for an end effector. Add a CadBridge regression test using an `Attached`
tube mass; MachineKit already tests `Attached` directly.

Before finalizing box conversion, verify where RobotKit centres
`ToolCollisionShape.Box`. It carries only half-extents; if RobotKit assumes a
flange-centred box, an offset envelope needs a RobotKit follow-up. Test known
TCP, mass, and box values, then verify `Manipulator.tcpPose` places the cup
contact correctly.

## Out of scope

- Eins catalog entries.
- Document persistence and the editor.
- RobotKit's multi-capability gripper/vacuum runtime refactor, until simulation
  needs it after E4.
