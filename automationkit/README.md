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

Facility lanes retain a `robotkit.navigation.Path` and enforce frame agreement
with their endpoint stations. Rack slot IDs and facility entity IDs are
validated at insertion. Fleet assignments require an attached, ready robot;
mission completion releases that robot for the next assignment. `TrafficManager`
allows one fleet member to own a lane at a time and makes reservations
idempotent for the current owner.

These are operational domain values and assignment boundaries. A planner and a
mission executor that translate facility tasks into RobotKit skills belong
above these models; the current types do not put facility or fleet state into
`RobotWorld`.

Run the focused Haxeon tests with:

```sh
./haxeon/scripts/haxeon run --project automationkit/tests/haxeon.json
```
