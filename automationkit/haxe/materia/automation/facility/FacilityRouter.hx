package materia.automation.facility;

import robotkit.mobile.Pose2;
import robotkit.navigation.Path;

/** Plans the minimum-travel-time lane route between named facility stations. */
class FacilityRouter {
  public final facility:Facility;
  public final endpointToleranceMeters:Float;

  public function new(facility:Facility, ?endpointToleranceMeters:Float = 0.1) {
    if (facility == null || !Math.isFinite(endpointToleranceMeters) ||
        endpointToleranceMeters < 0.0)
      throw "Facility router requires a facility and non-negative endpoint tolerance";
    this.facility = facility;
    this.endpointToleranceMeters = endpointToleranceMeters;
  }

  /** Returns the least-time route, respecting lane speeds and one-way direction. */
  public function route(fromStationId:String, toStationId:String):FacilityRoute {
    var start = facility.station(fromStationId);
    var goal = facility.station(toStationId);
    if (start == null || goal == null)
      throw "Facility route endpoints must name existing stations";
    if (fromStationId == toStationId)
      throw "Facility route requires distinct start and goal stations";
    if (start.frameId != goal.frameId)
      throw "Facility route endpoints must share a frame";

    var remaining = [for (station in facility.stations()) station.id];
    var travelTimes = new Map<String,Float>();
    var previous = new Map<String,FacilityRouteLeg>();
    travelTimes.set(fromStationId, 0.0);
    while (remaining.length > 0) {
      var current:Null<String> = null;
      var currentTime = 1.0e300;
      for (stationId in remaining) {
        var candidate = travelTimes.exists(stationId)
          ? travelTimes.get(stationId)
          : 1.0e300;
        if (candidate < currentTime ||
            (candidate == currentTime && current != null &&
              Reflect.compare(stationId, cast current) < 0)) {
          current = stationId;
          currentTime = candidate;
        }
      }
      if (current == null || currentTime >= 1.0e300) break;
      var currentId:String = cast current;
      remaining.remove(currentId);
      if (currentId == toStationId) break;

      for (lane in facility.lanes()) {
        var laneFrom = facility.station(lane.fromStationId);
        var laneTo = facility.station(lane.toStationId);
        if (laneFrom == null || laneTo == null ||
            !near(lane.centerline.start(), laneFrom.pose) ||
            !near(lane.centerline.goal(), laneTo.pose)) continue;
        var nextStationId:Null<String> = null;
        var reversed = false;
        if (lane.fromStationId == currentId) nextStationId = lane.toStationId;
        else if (lane.bidirectional && lane.toStationId == currentId) {
          nextStationId = lane.fromStationId;
          reversed = true;
        }
        if (nextStationId == null) continue;
        var nextId:String = cast nextStationId;
        if (remaining.indexOf(nextId) < 0) continue;
        var candidateTime = currentTime + lane.centerline.length /
          lane.maximumSpeedMetersPerSecond;
        var oldTime = travelTimes.exists(nextId) ? travelTimes.get(nextId) : 1.0e300;
        if (candidateTime + 1e-9 < oldTime) {
          travelTimes.set(nextId, candidateTime);
          previous.set(nextId, new FacilityRouteLeg(lane, currentId, nextId, reversed));
        }
      }
    }
    if (!previous.exists(toStationId))
      throw 'No directed facility route exists from "$fromStationId" to "$toStationId"';

    var legs:Array<FacilityRouteLeg> = [];
    var stationId = toStationId;
    while (stationId != fromStationId) {
      var leg:Null<FacilityRouteLeg> = previous.get(stationId);
      if (leg == null) throw "Facility route predecessor chain is incomplete";
      var value:FacilityRouteLeg = cast leg;
      legs.push(value);
      stationId = value.fromStationId;
    }
    legs.reverse();
    var poses:Array<Pose2> = [];
    var maximumSpeed = 1.0e300;
    for (leg in legs) {
      var from = facility.station(leg.fromStationId);
      var to = facility.station(leg.toStationId);
      if (from == null || to == null) throw "Facility route contains a missing station";
      var lanePoses = leg.lane.centerline.poses();
      if (leg.reversed) lanePoses.reverse();
      requireEndpoint(lanePoses[0], from.pose, leg.lane.id);
      requireEndpoint(lanePoses[lanePoses.length - 1], to.pose, leg.lane.id);
      lanePoses[0] = new Pose2(from.pose.x, from.pose.y, from.pose.yaw);
      lanePoses[lanePoses.length - 1] = new Pose2(to.pose.x, to.pose.y, to.pose.yaw);
      if (poses.length == 0) poses.push(lanePoses[0]);
      else poses[poses.length - 1] = lanePoses[0];
      for (index in 1...lanePoses.length) poses.push(lanePoses[index]);
      maximumSpeed = Math.min(maximumSpeed, leg.lane.maximumSpeedMetersPerSecond);
    }
    var routePath = new Path(poses, start.frameId);
    return new FacilityRoute(facility.id, fromStationId, toStationId, legs,
      routePath, maximumSpeed);
  }

  function requireEndpoint(pathPose:Pose2, stationPose:Pose2, laneId:String):Void {
    var dx = pathPose.x - stationPose.x;
    var dy = pathPose.y - stationPose.y;
    if (Math.pow(dx * dx + dy * dy, 0.5) > endpointToleranceMeters)
      throw 'Lane "$laneId" centerline does not reach its station endpoint';
  }

  function near(left:Pose2, right:Pose2):Bool {
    var dx = left.x - right.x;
    var dy = left.y - right.y;
    return Math.pow(dx * dx + dy * dy, 0.5) <= endpointToleranceMeters;
  }
}
