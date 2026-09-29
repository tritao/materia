package humankit.facility;

import humankit.ApproachFor;
import humankit.HumanJob;
import humankit.HumanLimb;
import humankit.Pick;
import humankit.Place;
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

  public function deliver(stationId:String, placePoint:Array<Float>):HumanJob {
    if (placePoint == null || placePoint.length < 3) throw "Delivery needs a world place point";
    var slot = targets.rackSlotPoint(rackId, slotId);
    var route = targets.route(rackId, stationId);
    var path = FacilityWalk.routeFromFacilityRoute(route);
    var station = path[path.length - 1];
    // After placing, step back to the station so the hands leave the part.
    return new HumanJob()
      .add(new ApproachFor(slot, ArmR))
      .add(new Pick(slot, [ArmR]))
      .add(WalkTo.along(path, route.maximumSpeedMetersPerSecond))
      .add(new ApproachFor(placePoint, ArmR))
      .add(new Place(placePoint, [ArmR]))
      .add(new WalkTo(station, route.maximumSpeedMetersPerSecond));
  }
}
