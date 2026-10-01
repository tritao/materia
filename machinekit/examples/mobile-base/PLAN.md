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
driving 150 mm wheels directly on `continuous` joints `wheel_l`/`wheel_r`,
front and rear casters, a battery, an upper deck on four tube posts and a
lidar on the front of the deck. The robot drives along +X with its axles
along Y, on the floor at z = 0. Checks: wheels and casters touch the floor,
track width and wheel radius come from the assembly, no interference, the
wheel joints turn the wheels about their axles. Start-page entry.

**M2. Project → mobile robot.** A manifest `mobileBase` key names the robot
subtree and its drive wheels; radius and track width are measured from the
assembly. `AssemblySimulationBridge` makes that subtree's root body the robot
root with `RobotModel.mobileBase` set and the rest of the assembly static
environment, placed at the designed pose. Test: a commanded `Twist2` turns the
wheels as `DifferentialDrive` says, the chassis follows the arc in the
presentation snapshot, odometry agrees with simulation truth.

**M3. Missions.** A manifest `mobileMission` (looping goals or stations)
played by an app `MobileMissionPlayer`: `Navigator` plans with A* and follows
with pure pursuit, commanding `MobileBase` each tick. The occupancy grid is
rasterized from the environment parts' footprints and inflated by the chassis
outline in `Costmap2`, so nothing is authored by hand. A small room with
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
| M0 | done | (this commit) |
| M1 | done | (this commit) |
| M2 | next | |
