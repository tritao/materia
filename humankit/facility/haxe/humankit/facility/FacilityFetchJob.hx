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
   * The job: fetch the part from its slot, walk the lane, set it down at `placePoint`. The worker leans
   * over the rack's and the station's tops and closes its fingers to the part's size when the facility
   * describes them (a rack's and station's `surface`, a slot's `itemHalfExtents`); a caller may pass
   * `rackSurface`, `stationSurface`, or `part` to override or supply what the facility does not. With
   * neither, the worker stands where its shoulder reaches the point and grips plainly.
   *
   * Without a `placePoint` the part is set down at the middle of the station's surface, resting on it: the point
   * is the surface's top plus half the part's height, so the facility must describe both.
   */
  public function deliver(stationId:String, ?placePoint:Array<Float>, ?rackSurface:HumanTargetBox,
      ?stationSurface:HumanTargetBox, ?part:HumanTargetBox):HumanJob {
    var slot = targets.rackSlotPoint(rackId, slotId);
    if (rackSurface == null) rackSurface = targets.surfaceBox(rackId);
    if (stationSurface == null) stationSurface = targets.surfaceBox(stationId);
    if (part == null) part = targets.slotItemBox(rackId, slotId);
    if (placePoint == null) {
      if (stationSurface == null || part == null)
        throw "Delivery needs a world place point, or a station surface and a part size to rest the part on";
      placePoint = [stationSurface.center[0], stationSurface.center[1],
        stationSurface.center[2] + stationSurface.halfExtents[2] + part.halfExtents[2]];
    }
    if (placePoint.length < 3) throw "Delivery needs a world place point";
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
