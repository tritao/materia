package humankit.facility;

import materia.automation.facility.Facility;
import materia.automation.facility.FacilityRoute;
import materia.automation.facility.FacilityRouter;
import robotkit.mobile.Pose2;

/** Resolves facility IDs into positions in their shared facility frame. */
class FacilityTargets {
  public final facility:Facility;
  final router:FacilityRouter;

  public function new(facility:Facility) {
    this.facility = facility;
    router = new FacilityRouter(facility);
  }

  public function stationStandPose(stationId:String):Pose2 {
    var station = facility.station(stationId);
    if (station == null) throw 'Unknown station "$stationId"';
    return new Pose2(station.pose.x, station.pose.y, station.pose.yaw);
  }

  public function rackSlotPoint(rackId:String, slotId:String):Array<Float> {
    var rack = facility.rack(rackId);
    if (rack == null) throw 'Unknown rack "$rackId"';
    var pose = rack.slotPose(slotId);
    return [pose.x, pose.y, pose.z];
  }

  public function route(fromStationId:String, toStationId:String):FacilityRoute
    return router.route(fromStationId, toStationId);
}
