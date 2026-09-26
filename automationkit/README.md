# Materia Automation

`materia.automation` is the facility and fleet layer above RobotKit. It keeps
facility topology and operational work separate from `robotkit.world`, whose
responsibility remains live robot state and command routing.

The first domain model includes:

- `facility`: framed zones, stations, directed lanes, racks, and chargers;
- `task`: declarative `Transport`, `Pick`, `Place`, and `Charge` requests;
- `mission`: an ordered task sequence with explicit pending, running, and
  terminal states;
- `fleet`: a roster composed over `RobotWorld`, deterministic dispatch to the
  first ready robot, and exclusive lane reservations through `TrafficManager`.
- `mission`: `MissionExecutor` advances each ordered task through a configured
  RobotKit skill and releases the assignment on success, failure, or cancel.

Facility lanes retain a `robotkit.navigation.Path` and enforce frame agreement
with their endpoint stations. `FacilityRouter` plans the least-travel-time lane
sequence using lane speed and direction, then composes its centerlines into one
framed path with per-lane speed intervals. Pass `route.speedLimits()` to
`FollowPath` so `Navigation` brakes before entering a slower lane. Rack slot IDs and
facility entity IDs are validated at insertion. Fleet assignments require an attached, ready robot;
mission completion releases that robot for the next assignment. `TrafficManager`
allows one fleet member to own a lane or intersection at a time, supports
priority-ordered waits and blocked lanes, and can reserve a complete route's
lanes and junctions atomically. Route bundles hold all required resources until
`releaseRoute`, so a waiting route never holds a partial set of locks.
Facility `Intersection` regions can group crossing lanes under one exclusive
reservation. `TrafficSpeedZone` applies a temporary cap to selected lanes;
`TrafficManager.route()` uses active caps and reservations when choosing a
route, while the existing per-lane `PathSpeedLimit` intervals carry the
resulting speeds to navigation.

Task execution is an explicit composition point. `TaskSkillFactory` receives
the current task, assigned `Robot`, and `Facility`, then returns a configured
`robotkit.skill.Skill`. For example, an application can resolve a
`Transport`'s station IDs through `FacilityRouter` and return `FollowPath`; a perception
adapter can resolve a rack task into `PickPallet` or `PlacePallet`. The
executor owns sequencing and lifecycle propagation, while route planning,
perception, and robot-specific mechanism configuration remain explicit
application inputs.

An optional `TrafficManager` on `MissionExecutor` reserves a transport's full
route before starting its skill. The executor leaves the skill idle while the
route is queued, then releases the reservation on success, failure, or cancel.

The integration test executes a facility transport mission on a simulated
RobotKit robot, captures its observations and joint batches through
`RecordingRobot` into MCAP, then executes the same mission through `ReplayRobot`
and compares the generated target batches.

```haxe
var assignment = dispatcher.dispatch(mission);
var executor = new MissionExecutor(fleet, assignment, facility, skillFactory);
executor.start();
var state = executor.update(0.02);
```

Facility, mission, and fleet data remain above RobotKit. `RobotWorld` continues
to own live robot state and command routing only.

Run the focused Haxeon tests with:

```sh
./haxeon/scripts/haxeon run --project automationkit/tests/haxeon.json
```
