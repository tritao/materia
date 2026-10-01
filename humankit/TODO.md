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
- The automatic step-back after placement does not avoid support or zone boxes;
  author a clear retreat route until obstacle-aware planning is available.
- A dedicated backward gait for short retreats; the current backward retreat
  keeps the heading and idle pose while translating away from the part. The
  library has `Walk_Bwd_Loop` (extracted, not wired).
- Floor-level picks below 0.3 m. Crouching (to about half a metre) and kneeling
  (to 0.3 m) work (`BODY.md`); a point on the floor needs a stoop or a deeper reach
  than the kneeling clip has.
- Turns in place use the root rotating over the idle pose, so the feet slide;
  `Turn90_L/R` are extracted but not wired, and root-motion (`_RM`) clips need
  support in the walker.
- Deep tops below table height (0.8 m deep, under about 0.85 m): bending over works standing, but with a crouch or a kneel it flips an arm.
  A braced hand on the top, and a gentler stance than a bow with the head low, are not done.
- Look-at for head and eyes (needs aim IK from AnimKit, see its TODO).
- Hand orientation. IK sets the wrist position only, so a part keeps the
  orientation it was carried with (apart from `Place` levelling it); yaw
  alignment and insertion need an oriented grasp.

## Body

- Judge naturalness beyond numbers: contact sheets at job events, and a
  reference-motion comparison (time-warped distance of wrist and pelvis paths
  to the authored `PickUp_Table`). `Naturalness` gives foot slide, balance and
  jerk; nothing yet compares to a reference or renders.
- Cache a body model per rig (shoulder, belly and hip positions over crouch,
  hinge and lean) so the planner is a pure function, and evaluate poses without
  skinning or touching the scene; today each probe is a full `advance(0)`.

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
