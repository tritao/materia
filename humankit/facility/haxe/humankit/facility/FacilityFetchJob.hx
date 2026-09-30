package humankit.facility;

import humankit.ApproachFor;
import humankit.HumanJob;
import humankit.HumanLimb;
import humankit.HumanTargetBox;
import humankit.Pick;
import humankit.Place;
import humankit.ReleaseLimb;
import humankit.WalkTo;

/** A fetch plan resolved into one HumanJob when its delivery is named. */
class FacilityFetchJob {
  public final targets:FacilityTargets;
  public final rackId:String;
  public final slotId:String;

  public function new(targets:FacilityTargets, rackId:String, slotId:String) {
    this.targets = targets;
    this.rackId = rackId;
    this.slotId = slotId;
    targets.rackSlotPoint(rackId, slotId);
  }

  /**
   * The job: fetch the part from its slot, walk the lane, set it down at `placePoint`. A facility model
   * has no footprints, so the caller may describe what the worker stands at: `rackSurface` and
   * `stationSurface` are the tops the part rests on at each end, and `part` is the part's box. With them
   * the worker stands clear of each surface's edge and leans over it, and closes its fingers to the
   * part's size; without them it stands where its shoulder reaches the point and grips plainly.
   */
  public function deliver(stationId:String, placePoint:Array<Float>, ?rackSurface:HumanTargetBox,
      ?stationSurface:HumanTargetBox, ?part:HumanTargetBox):HumanJob {
    if (placePoint == null || placePoint.length < 3) throw "Delivery needs a world place point";
    var slot = targets.rackSlotPoint(rackId, slotId);
    var route = targets.route(rackId, stationId);
    var path = FacilityWalk.routeFromFacilityRoute(route);
    var station = path[path.length - 1];
    // After placing, step back to the station so the hands leave the part.
    return new HumanJob()
      .add(new ApproachFor(slot, ArmR, 1.0, false, null, rackSurface))
      .add(new Pick(slot, [ArmR], 0.35, part))
      .add(WalkTo.along(path, route.maximumSpeedMetersPerSecond))
      .add(new ApproachFor(placePoint, ArmR, 1.0, false, null, stationSurface))
      .add(new Place(placePoint, [ArmR]))
      .add(new WalkTo(station, route.maximumSpeedMetersPerSecond))
      .add(new ReleaseLimb(ArmR));
  }
}
