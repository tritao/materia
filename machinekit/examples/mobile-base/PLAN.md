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

**M3. Missions.** A `mobileMission` (looping goals or stations)
played by an app `MobileMissionPlayer`: `Navigator` plans with A* and follows
with pure pursuit, commanding `MobileBase` each tick. The occupancy grid is
rasterized from the environment parts' footprints and inflated by the chassis
outline in `Costmap2`, so nothing is authored by hand. The scene becomes an
assembly that includes the robot (`include("robot", ...)`) beside its room;
the drive section names that robot subtree, whose root body becomes the moving
chassis while every other world-fixed body stays put. A small room with
shelves and a dock around the robot. Test: every goal reached, no inflated
cell entered, the mission loops.

**M4. Sensing and safety.** The lidar sensor mounts at the robot's `lidar`
port; `LidarObstaclePerception` feeds the costmap's dynamic layer and
`MotionGuard`. A dynamic box dropped on the route makes the base slow, stop
and replan. Editor overlays: planned path, costmap, lidar rays, odometry
ghost.

**M5. Mobile manipulator.** `RobotArm` included on the deck; the mission
gains a pick at a shelf (`GoTo` a `WorkPatchPlanner` pose, arm motion, grip).
Stretch: whole-body solving with KinematicsKit `RootMotion.Planar`.

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
| M2 | done | c2d81b4f (wheel directions), drive in the scene artifact + bridge + app (next commit) |
