package humankit.facility;

import humankit.HumanTargetBox;
import materia.automation.facility.Facility;
import materia.automation.facility.FacilityRoute;
import materia.automation.facility.FacilityRouter;
import materia.automation.facility.Station;
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

  /** The top of a rack or station as a box in the facility frame, or null when the facility describes none. */
  public function surfaceBox(stationId:String):Null<HumanTargetBox> {
    var station:Null<Station> = facility.station(stationId);
    if (station == null) station = facility.rack(stationId);
    if (station == null) throw 'Unknown station "$stationId"';
    var surface = station.surface;
    if (surface == null) return null;
    var center = surface.centerIn(station.pose);
    // The box is the slab under the top, so its top face is at the surface height.
    var slab = Math.min(0.05, surface.top);
    return {center: [center.x, center.y, surface.top - slab], halfExtents: [surface.halfWidth, surface.halfDepth, slab],
      yaw: center.yaw};
  }

  /** The box of the item a rack slot holds, in the facility frame, or null when the facility does not say. */
  public function slotItemBox(rackId:String, slotId:String):Null<HumanTargetBox> {
    var rack = facility.rack(rackId);
    if (rack == null) throw 'Unknown rack "$rackId"';
    var slot = rack.slot(slotId);
    if (slot == null) throw 'Rack has no slot "$slotId"';
    var extents = slot.itemHalfExtents;
    if (extents == null) return null;
    var pose = rack.slotPose(slotId);
    return {center: [pose.x, pose.y, pose.z], halfExtents: extents.copy(), yaw: pose.yaw};
  }

  public function route(fromStationId:String, toStationId:String):FacilityRoute
    return router.route(fromStationId, toStationId);
}
