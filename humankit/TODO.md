# HumanKit follow-ups

The next milestone is planned in `plans/M5-WORKERS-IN-DOCUMENTS.md` (workers
as scene objects, jobs as versioned JSON). These come after it.

## Safety with robots

- A consumer for `HumanWorkerSignals`: speed-and-separation monitoring and
  protective stops for robots (with the mixed-scene-safety work).
- Exact capsule-to-shape separation. Today each robot link collision shape is
  a bounding sphere (`app/src/editor/RobotLinkBounds.hx`, registered through
  `HumanWorker.addRobotLinkPose`).
- A per-run report of near misses and time spent in hazard zones.

## Facilities

- Stations, racks and lanes authored in documents, so jobs can target
  facility IDs. `humankit-facility` builds jobs from AutomationKit facilities
  in code only, and the only facility is the hard-coded
  `app/src/editor/FacilityRouteDemo.hx`.

## Motion

- Path planning and obstacle avoidance around machines and racks; yielding to
  other workers and AGVs at lanes and intersections. Routes are followed
  blindly.
- Crouching and kneeling. `ApproachFor` fails for targets below the waist.
- A carry clip and gait for two-handed carrying of larger parts.
- Look-at for head and eyes (needs aim IK from AnimKit, see its TODO).
- Hand orientation. IK sets the wrist position only, so a part keeps the
  orientation it was carried with (apart from `Place` levelling it); yaw
  alignment and insertion need an oriented grasp.
- A grip animation: hands do not close on what they hold.

## Body

- Bone-length fitting to `HumanDescription`. An explicit description only
  scales the whole rig uniformly.
- 5th/50th/95th-percentile body presets for layout checks.
- Reach envelopes, and RULA/REBA-style ergonomic scoring from joint angles
  during a job.

## Physics detail

- The hand's collision capsule covers the fingers coarsely, so it overlaps a
  part while reaching onto it or withdrawing. `HumanWorker` pins the part to a
  hidden stabiliser at those moments. A closer-fitting hand (palm and finger
  capsules) would remove the need for the pin.
