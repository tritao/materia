# Mobile base plan

Goal: a wheeled mobile robot example that drives around a small room in the
editor, plans its own routes from the CAD geometry, sees obstacles with a
lidar, and eventually carries the robot arm to its work.

Most of the mobile stack already exists in RobotKit, all of it kinematic Haxe:
drive models (`robotkit/mobile`: `DifferentialDrive`, `AckermannDrive`,
`HolonomicDrive`, odometry), ideal rolling plants
(`runtime/DifferentialDrivePlant`), localization (wheel odometry, wheel+IMU,
simulation truth), navigation (`navigation/OccupancyGrid2`, `Costmap2`,
`AStarPlanner`, pure pursuit in `Navigation`/`Navigator`, `MotionGuard`) and
a planar lidar sensor with `LidarObstaclePerception`. The only place it all
runs together is a test (`WallFinishingScenarioTests`). What is missing is the
project side: the app's assembly bridge always fixes the assembly root to the
world, a project can only declare joint tracks (`robotMotions`), and MachineKit
has no wheels.

## Steps

**M0. Wheel parts.** MachineKit gains `motion/DriveWheel` (tread on a hub
boss, bored for a motor shaft) and `motion/CasterWheel` (swivel caster on a
top plate, modelled rigid), both with recipes, named faces and smoke checks.

**M1. The robot, geometry and joints.** `MobileBase` is a `MachineAssembly`:
an aluminium base plate with wheel slots, two NEMA 23 steppers on brackets
driving 150 mm wheels directly, front and rear casters, a battery, an upper
deck on four tube posts and a lidar on the front of the deck. The robot
drives along +X with its axles along Y, on the floor at z = 0.

The plates own the layout: a `ChassisPlate` takes named seats (a connector
frame plus the holes its fasteners need), and every part mates to a seat
through its own connector. Each wheel's bore mates to its motor's shaft on a
`continuous` joint (`wheel_l`, `wheel_r`) about that shaft, which points
outward, so the joints read like the motors' encoders: positive rolls the left
wheel forward and the right one back. (An earlier cut laid parts out by world
pose with derived connectors; it hid the part interfaces and let a changed
bracket or shaft silently leave the wheel behind.) Checks: wheels and casters
touch the floor, track width measured from the solved assembly, each wheel
turns about its own shaft, no interference. Start-page entry.

**M2. Project → mobile robot.** First RobotKit: the differential drive
derives each wheel's forward direction from its joint axis in the base frame
(forward is a spin about +Y; a wheel whose axis is not lateral is rejected) and
applies it in `DifferentialDrive.targets`, `DifferentialOdometry`, the drive
plant and the native `rk_simulation_set_differential_drive`, so CAD joints stay
mechanical truth and imported URDF/MJCF bases work either way.

The drive travels in the scene artifact (format 13, a `mobileBase` section),
not the manifest, for the reason C4 moved the machining job there: the
generator measures wheel radius and track from the CAD, so nothing is typed
twice. `AssemblySimulationBridge` turns it into `RobotModel.mobileBase`, with
the assembly's root link as the chassis (the assembly frame is the robot's
floor frame), and `AssemblyRobot` couples the wheels to that chassis in the
simulation and exposes the `MobileBase` drive view. The session gained
`openGeneratedProject` so a new generated field is wired once. Tests: the
CadBridge suite drives the real example (directions, chassis link, wheel
rates, odometry); the project-source suite opens it as a project on both
backends and checks a straight run and an in-place turn.

The robot/environment split moves to M3, where the room first appears: until
then the whole assembly is the robot.

**M3. Missions.** The scene artifact gains a `mission` section: steps that say
what to do in terms of the model (`goTo` a floor pose now; `pick`/`place` a part
in M5), kept apart from the `mobileBase` drive description. The app's
`MissionPlayer` turns each step into a RobotKit skill (`GoTo` wraps
`Navigator`) and advances it with `SkillRunner`, so steps compose the way
AutomationKit's `MissionExecutor` composes them. The map is the floor plan of
the robot's surroundings: the room's parts are not part of the robot (the drive
section names the robot's subtree and its origin), so they simulate as held
boxes, and those same boxes are rasterized into the occupancy grid, inflated by
the chassis' corner radius in `Costmap2`. The example's default entrypoint is
now a walled room with two shelves, a pillar and a dock, the robot driving a
looping round of them.

Driving it exposed three `Navigation` arrival faults on a base with a real
deceleration limit, fixed in RobotKit: braking aimed at the edge of the goal
tolerance (now the goal itself, on the shorter of route-left and straight line),
no latch once the goal position was reached (the robot rolled back out and
followed the path past the goal into a shelf), and no turn in place toward a
route behind the robot (it arced into the shelf it faced). A haxeon fix came out
of it too: a same-package type now outranks a same-named unpackaged one
(UiKit's `Path` hid `robotkit.navigation.Path` in the app).

**M4. Sensing and safety.** The lidar sensor mounts at the robot's `lidar`
port; `LidarObstaclePerception` feeds the costmap's dynamic layer and
`MotionGuard`. A dynamic box dropped on the route makes the base slow, stop
and replan. Editor overlays: planned path, costmap, lidar rays, odometry
ghost.

**M5. Mobile manipulator** (done). The robot arm stands on the deck's payload
seat, turned to work ahead; the room's shelves became two tables with place
seats and a free workpiece. The mission picks it at the north table, places it
on the east table's seat, picks it again and brings it back, then parks at the
dock: about a minute on MuJoCo, the part landing within a tenth of a millimetre
of each seat centre.

How it is built, each piece where it belongs:
- *Tools from the CAD.* MachineKit's `EndEffectorControls` derives an end
  effector's control channels, vacuum sensor and suction contacts from its
  parts' capabilities (cadbridge's `deriveBindings` now delegates to it); the
  generator writes them to the scene artifact's `robotTools` section.
- *A simulated device on the robot.* RobotKit's `SimulatedSuctionTool` works
  like the ejector it stands for: it reads its channel's output
  (`RobotRuntime.channelValue`, a new native getter), seals on the free part the
  tool link touches, carries it, lets go, and publishes `tool_vacuum_kpa` when
  the tool has a sensor. The app's `RobotGripPlayer` drives the same device.
- *Skills that only use the robot.* `HandlePart` runs a MotionKit program
  (`HandlingPlanRunner`, `ManipulatorMotion` like every arm program) down onto
  the part or seat read live, switches the tool channel there, and reads the
  outcome on the vacuum sensor, or trusts the program without one.

Faults this exposed and fixed: an uncommanded arm sagged under gravity and
faulted the robot (simulated robots gain a hold-at-rest option, staged in the
endpoint so it never competes with a first plan, which a hold command did with
the CNC router's); a navigation stop on arrival took the vacuum to its safe value
and dropped the part (process channels gain a stop policy: `keepOnStop`
channels, as gripper and vacuum channels are, keep their output through a
commanded stop or abort, still going safe on faults and emergency stops); the
runtime's motion state, cleared by stops, also held the channel outputs (now
kept apart, backed up and reset with the robot); MotionKit sampled a hair past
a pose line's end in two places (numeric derivatives, last path sample), the
error that made the wall-finishing scenario flaky; the cup has to press into the
part (3 mm, as a compliant cup does) to meet it within the motion tolerance.

Left from the restructure:
- The arm example still streams joint tracks with timed grips, so its grips
  actuate the suction device directly; moving it onto MotionKit plans with
  `SetOutput` events would give every grip the channel path.
- The arm's suction tool has no vacuum pressure sensor (it would need the hose
  re-routed through an inline sensor), so its picks are trusted, not sensed.
- The device protocol (`rkd6`) does not carry the stop policy yet; a real
  device would still drop a held part on a commanded stop.
- The test backend's robot-to-object contacts reach far enough to push the
  workpiece off its table as the arm passes, so pick and place are checked on
  MuJoCo only, like the arm example.
- `WallFinishingScenarioTests` still fails intermittently on an unreachable
  pose (an IK issue unrelated to the pose-line fix).

The navigation latch from M3 also changed here: latching anywhere inside the
goal tolerance left the base at the tolerance's edge with its heading off,
which put the construction skills' work patches out of the arm's reach; it now
keeps closing in while the goal lies ahead in its direction of travel.

**M6. Physics-driven wheels (optional).** A floating base on MuJoCo with tire
friction, so the wheels propel the chassis through contact: lift the
floating-robot rejection of drive/plant calls, wheel velocity actuators,
swivelling casters.

**M7. Facility routes (optional).** Drive AutomationKit `FacilityRouter`
lanes, dock and charge.

## Order

M0 → M1 → M2 → M3 is the core; M2 carries the risk (the robot/environment
split reverses A6's "world-fixed bodies join the root" rule and must not break
the arm and router examples). M4 is mostly wiring, M5 reuses the arm, M6 and
M7 are stretch goals.

## Progress

| Step | State | Commits |
| --- | --- | --- |
| M0 | done | c6e9da67 |
| M1 | done | c6e9da67, 512baf40 (plates own the layout, mates throughout) |
| M2 | done | c2d81b4f (wheel directions), 11b9a6c9 (drive in the scene artifact, bridge, app) |
| M3 | done | mission section, skill-hosting MissionPlayer, room cell, Navigation arrival fixes; haxeon 09279420 |
| M5 | done | b348ec51 (RobotKit tools, stop policy, HandlePart), next commit (robot tools section, app, cell) |
