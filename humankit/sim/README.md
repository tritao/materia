# Human workers in SimKit

`HumanWorker` owns a `HumanBody` action queue and a capsule `HumanActor`. Construct it while the session is unsealed, bind each physical `Pick` and `Place` to a `SimObject`, then call `advance()` before each explicit `SimSession.step()`. The worker writes poses three fixed ticks ahead, matching `CharacterPreview`.

```haxe
var worker = new HumanWorker(session, character, proxy, new SimPose(0, 0, 0));
var pick = new Pick(graspPoint, [ArmR]);
var place = new Place(placePoint, [ArmR]);
worker.bindPick(pick, part, graspPoint);
worker.bindPlace(place, part, placePoint);
worker.run(new HumanJob().add(pick).add(new WalkTo([1, 0], 1)).add(place));
while (!job.isDone()) { worker.advance(); session.step(); }
```

A picked object follows the hand capsule. Placement transfers it to a stationary, noncolliding carrier at the target during a short dwell, then releases it with zero carrier velocity. The object position relative to its grasp point is retained, so `Place.target` is the hand's grasp point. `HumanWorker.dispose()` removes the step observer. In a sealed session, actor bodies remain owned by the session until session disposal.

Call `addZone(new HumanZone(id, polygon))` for floor polygons and `addRobotLink(id, body, radius)` for robot collision bounds. `onTick(worker, signals)` runs after each explicit session step and reports occupied zone IDs and minimum separation by robot ID. Zone occupancy samples the center and endpoints of each capsule projected onto the XY floor. Separation treats each human capsule as a line segment plus radius and each robot link as a bounding sphere centered on its snapshot body pose; overlapping bounds report zero. It is an approximation, not an exact capsule-to-collision-shape distance. Facility zone adapters can supply their polygon vertices without introducing an AutomationKit dependency here.
