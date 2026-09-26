package materia.automation.fleet;

import materia.automation.facility.Facility;
import materia.automation.facility.FacilityRoute;
import materia.automation.facility.FacilityRouteLeg;
import materia.automation.facility.FacilityRouter;
import StringTools;

/** Priority reservations for facility lanes and route intersections. */
class TrafficManager {
  public final facility:Facility;
  public final fleet:Fleet;
  final ownerByLane = new Map<String,String>();
  final ownerByIntersection = new Map<String,String>();
  final blockedByLane = new Map<String,String>();
  final speedZonesById = new Map<String,TrafficSpeedZone>();
  final routeSignatureByRobot = new Map<String,String>();
  final routeResourcesByRobot = new Map<String,Array<String>>();
  final waiters:Array<TrafficWaiter> = [];
  var requestOrder = 0;

  public function new(facility:Facility, fleet:Fleet) {
    if (facility == null || fleet == null) throw "TrafficManager requires a facility and fleet";
    this.facility = facility;
    this.fleet = fleet;
  }

  /** Returns false when the lane is blocked, owned, or ahead of this request in the priority queue. */
  public function reserve(laneId:String, robotId:String, ?priority:Int = 0):Bool {
    if (facility.lane(laneId) == null) throw 'Unknown facility lane "$laneId"';
    requireFleetMember(robotId);
    var owner = ownerByLane.get(laneId);
    if (owner == robotId) return true;
    if (blockedByLane.exists(laneId)) return false;
    var resources = [laneKey(laneId)];
    for (areaId in conflictRegionsForLane(laneId))
      resources.push(intersectionKey('area:$areaId'));
    resources.sort(Reflect.compare);
    if (!respectsResourceOrder(resources, robotId)) return false;
    return acquire(resources, robotId, priority, null);
  }

  /** Reserves a station's intersection resource using the same priority queue as lanes. */
  public function reserveIntersection(stationId:String, robotId:String,
      ?priority:Int = 0):Bool {
    if (facility.station(stationId) == null)
      throw 'Unknown facility intersection "$stationId"';
    requireFleetMember(robotId);
    var owner = ownerByIntersection.get(stationId);
    if (owner == robotId) return true;
    var resources = [intersectionKey(stationId)];
    if (!respectsResourceOrder(resources, robotId)) return false;
    return acquire(resources, robotId, priority, null);
  }

  /**
   * Atomically reserves every lane and intermediate intersection in a route.
   * No resource is held while waiting, so route-level requests cannot deadlock.
   */
  public function reserveRoute(route:FacilityRoute, robotId:String,
      ?priority:Int = 0):Bool {
    if (route == null || route.facilityId != facility.id)
      throw "Traffic route must belong to this facility";
    requireFleetMember(robotId);
    var legs = route.legs();
    var resources:Array<String> = [];
    for (index in 0...legs.length) {
      var leg = legs[index];
      if (facility.lane(leg.lane.id) == null)
        throw 'Traffic route references unknown lane "${leg.lane.id}"';
      var blocked = blockedByLane.get(leg.lane.id);
      if (blocked != null) return false;
      resources.push(laneKey(leg.lane.id));
      for (areaId in conflictRegionsForLane(leg.lane.id))
        if (resources.indexOf(intersectionKey('area:$areaId')) < 0)
          resources.push(intersectionKey('area:$areaId'));
      if (index < legs.length - 1)
        resources.push(intersectionKey(leg.toStationId));
    }
    resources.sort(Reflect.compare);
    var signature = routeSignature(route, legs);
    var currentSignature = routeSignatureByRobot.get(robotId);
    if (currentSignature != null) {
      if (currentSignature != signature)
        throw 'Robot "$robotId" must release its current route before reserving another';
      return true;
    }
    if (ownsAnyResource(robotId))
      throw 'Robot "$robotId" must release individual reservations before reserving a route';
    return acquire(resources, robotId, priority, signature);
  }

  public function owner(laneId:String):Null<String> return ownerByLane.get(laneId);
  public function intersectionOwner(stationId:String):Null<String>
    return ownerByIntersection.get(stationId);
  public function conflictRegionOwner(id:String):Null<String>
    return ownerByIntersection.get('area:$id');
  public function laneBlockReason(laneId:String):Null<String> return blockedByLane.get(laneId);

  public function setSpeedZone(zone:TrafficSpeedZone):Void {
    if (zone == null) throw "Traffic speed zone is required";
    for (laneId in zone.lanes())
      if (facility.lane(laneId) == null) throw 'Unknown facility lane "$laneId"';
    speedZonesById.set(zone.id, zone);
  }

  public function clearSpeedZone(zoneId:String):Bool
    return speedZonesById.remove(zoneId);

  public function speedZone(zoneId:String):Null<TrafficSpeedZone>
    return speedZonesById.get(zoneId);

  public function speedZones():Array<TrafficSpeedZone> {
    var ids = [for (id in speedZonesById.keys()) id];
    ids.sort(Reflect.compare);
    return [for (id in ids) speedZonesById.get(id)];
  }

  /** Plans around closures and other robots' reservations with current speed caps. */
  public function route(fromStationId:String, toStationId:String,
      ?robotId:String):FacilityRoute {
    var unavailable:Array<String> = [];
    for (laneId in blockedByLane.keys()) unavailable.push(laneId);
    for (laneId in ownerByLane.keys())
      if (ownerByLane.get(laneId) != robotId && unavailable.indexOf(laneId) < 0)
        unavailable.push(laneId);
    for (intersection in facility.intersections()) {
      var owner = conflictRegionOwner(intersection.id);
      if (owner == null || owner == robotId) continue;
      for (laneId in intersection.lanes())
        if (unavailable.indexOf(laneId) < 0) unavailable.push(laneId);
    }
    var limits = new Map<String,Float>();
    for (zone in speedZones()) for (laneId in zone.lanes()) {
      var current = limits.get(laneId);
      if (current == null || zone.maximumSpeedMetersPerSecond < current)
        limits.set(laneId, zone.maximumSpeedMetersPerSecond);
    }
    return new FacilityRouter(facility).route(fromStationId, toStationId,
      unavailable, limits);
  }

  /** Prevents new reservations; the current owner may still release the lane. */
  public function blockLane(laneId:String, reason:String):Void {
    if (facility.lane(laneId) == null) throw 'Unknown facility lane "$laneId"';
    if (reason == null || reason.length == 0) throw "Blocked lane requires a reason";
    blockedByLane.set(laneId, reason);
  }

  public function unblockLane(laneId:String):Bool {
    if (!blockedByLane.exists(laneId)) return false;
    blockedByLane.remove(laneId);
    return true;
  }

  /** Releases a single lane; route bundles must use releaseRoute to stay atomic. */
  public function release(laneId:String, robotId:String):Bool {
    var current = ownerByLane.get(laneId);
    if (current == null || current != robotId ||
        isRouteResource(robotId, laneKey(laneId))) return false;
    ownerByLane.remove(laneId);
    for (areaId in conflictRegionsForLane(laneId)) {
      var stillHeld = false;
      var intersection = facility.intersection(areaId);
      for (otherLane in intersection.lanes())
        if (ownerByLane.get(otherLane) == robotId) stillHeld = true;
      if (!stillHeld) ownerByIntersection.remove('area:$areaId');
    }
    return true;
  }

  public function releaseIntersection(stationId:String, robotId:String):Bool {
    var current = ownerByIntersection.get(stationId);
    if (current == null || current != robotId ||
        isRouteResource(robotId, intersectionKey(stationId))) return false;
    ownerByIntersection.remove(stationId);
    return true;
  }

  /**
   * Releases a held route bundle, or removes this robot's pending request for
   * that exact route. The return value is true only when a held bundle released.
   */
  public function releaseRoute(route:FacilityRoute, robotId:String):Bool {
    if (route == null || route.facilityId != facility.id) return false;
    var legs = route.legs();
    var signature = routeSignature(route, legs);
    if (routeSignatureByRobot.get(robotId) != signature) {
      removeWaiter(robotId, resourcesForRoute(legs));
      return false;
    }
    var resources:Null<Array<String>> = routeResourcesByRobot.get(robotId);
    if (resources != null) {
      var values:Array<String> = cast resources;
      for (resource in values) clearOwner(resource, robotId);
    }
    routeResourcesByRobot.remove(robotId);
    routeSignatureByRobot.remove(robotId);
    removeWaiter(robotId, null);
    return true;
  }

  public function releaseRobot(robotId:String):Void {
    var routeResources:Null<Array<String>> = routeResourcesByRobot.get(robotId);
    if (routeResources != null) {
      var values:Array<String> = cast routeResources;
      for (resource in values) clearOwner(resource, robotId);
    }
    routeResourcesByRobot.remove(robotId);
    routeSignatureByRobot.remove(robotId);
    for (laneId in reservedLanes(robotId)) ownerByLane.remove(laneId);
    var intersections = [for (stationId in ownerByIntersection.keys())
      if (ownerByIntersection.get(stationId) == robotId) stationId];
    for (stationId in intersections) ownerByIntersection.remove(stationId);
    removeWaiter(robotId, null);
  }

  public function reservedLanes(robotId:String):Array<String> {
    var result:Array<String> = [];
    for (laneId in ownerByLane.keys()) if (ownerByLane.get(laneId) == robotId) result.push(laneId);
    result.sort(Reflect.compare);
    return result;
  }

  public function reservedIntersections(robotId:String):Array<String> {
    var result:Array<String> = [];
    for (stationId in ownerByIntersection.keys())
      if (ownerByIntersection.get(stationId) == robotId) result.push(stationId);
    result.sort(Reflect.compare);
    return result;
  }

  public function waitingRobots(resourceKey:String):Array<String> {
    var ordered = waiters.copy();
    ordered.sort(compareWaiters);
    var result:Array<String> = [];
    for (waiter in ordered) if (waiter.resources.indexOf(resourceKey) >= 0)
      result.push(waiter.robotId);
    return result;
  }

  public function waitingForLane(laneId:String):Array<String>
    return waitingRobots(laneKey(laneId));

  public function waitingForIntersection(stationId:String):Array<String>
    return waitingRobots(intersectionKey(stationId));

  function acquire(resources:Array<String>, robotId:String, priority:Int,
      signature:Null<String>):Bool {
    var waiter = findWaiter(robotId, resources);
    if (waiter == null) {
      waiter = new TrafficWaiter(robotId, resources.copy(), priority, requestOrder++);
      waiters.push(waiter);
    } else waiter.priority = priority;
    waiters.sort(compareWaiters);

    for (prior in waiters) {
      if (prior == waiter) break;
      if (overlaps(prior.resources, resources)) return false;
    }
    for (resource in resources) {
      var owner = ownerFor(resource);
      if (owner != null && owner != robotId) return false;
    }
    for (resource in resources) setOwner(resource, robotId);
    removeWaiter(robotId, resources);
    if (signature != null) {
      routeSignatureByRobot.set(robotId, signature);
      routeResourcesByRobot.set(robotId, resources.copy());
    }
    return true;
  }

  function resourcesForRoute(legs:Array<FacilityRouteLeg>):Array<String> {
    var resources:Array<String> = [];
    for (index in 0...legs.length) {
      resources.push(laneKey(legs[index].lane.id));
      for (areaId in conflictRegionsForLane(legs[index].lane.id))
        if (resources.indexOf(intersectionKey('area:$areaId')) < 0)
          resources.push(intersectionKey('area:$areaId'));
      if (index < legs.length - 1)
        resources.push(intersectionKey(legs[index].toStationId));
    }
    resources.sort(Reflect.compare);
    return resources;
  }

  function conflictRegionsForLane(laneId:String):Array<String> {
    var result:Array<String> = [];
    for (intersection in facility.intersections())
      if (intersection.lanes().indexOf(laneId) >= 0) result.push(intersection.id);
    return result;
  }

  function ownerFor(resource:String):Null<String> {
    if (StringTools.startsWith(resource, "lane:"))
      return ownerByLane.get(resource.substr(5));
    return ownerByIntersection.get(resource.substr(13));
  }

  function setOwner(resource:String, robotId:String):Void {
    if (StringTools.startsWith(resource, "lane:")) ownerByLane.set(resource.substr(5), robotId);
    else ownerByIntersection.set(resource.substr(13), robotId);
  }

  function clearOwner(resource:String, robotId:String):Void {
    if (StringTools.startsWith(resource, "lane:")) {
      var id = resource.substr(5);
      if (ownerByLane.get(id) == robotId) ownerByLane.remove(id);
    } else {
      var id = resource.substr(13);
      if (ownerByIntersection.get(id) == robotId) ownerByIntersection.remove(id);
    }
  }

  function ownsAnyResource(robotId:String):Bool {
    for (owner in ownerByLane) if (owner == robotId) return true;
    for (owner in ownerByIntersection) if (owner == robotId) return true;
    return false;
  }

  function respectsResourceOrder(requested:Array<String>, robotId:String):Bool {
    var held:Array<String> = [];
    for (laneId in reservedLanes(robotId)) held.push(laneKey(laneId));
    for (stationId in reservedIntersections(robotId))
      held.push(intersectionKey(stationId));
    for (owned in held) for (resource in requested)
      if (Reflect.compare(owned, resource) > 0) return false;
    return true;
  }

  function isRouteResource(robotId:String, resource:String):Bool {
    var resources:Null<Array<String>> = routeResourcesByRobot.get(robotId);
    return resources != null && cast(resources, Array<String>).indexOf(resource) >= 0;
  }

  function findWaiter(robotId:String, resources:Array<String>):Null<TrafficWaiter> {
    for (waiter in waiters) if (waiter.robotId == robotId &&
        sameResources(waiter.resources, resources)) return waiter;
    return null;
  }

  function removeWaiter(robotId:String, ?resources:Array<String>):Void {
    var kept:Array<TrafficWaiter> = [];
    for (waiter in waiters) if (waiter.robotId != robotId ||
        (resources != null && !sameResources(waiter.resources, resources))) kept.push(waiter);
    waiters.splice(0, waiters.length);
    for (waiter in kept) waiters.push(waiter);
  }

  static function sameResources(left:Array<String>, right:Array<String>):Bool {
    if (left.length != right.length) return false;
    var a = left.copy();
    var b = right.copy();
    a.sort(Reflect.compare);
    b.sort(Reflect.compare);
    for (index in 0...a.length) if (a[index] != b[index]) return false;
    return true;
  }

  static function overlaps(left:Array<String>, right:Array<String>):Bool {
    for (resource in left) if (right.indexOf(resource) >= 0) return true;
    return false;
  }

  static function compareWaiters(left:TrafficWaiter, right:TrafficWaiter):Int {
    if (left.priority != right.priority) return right.priority - left.priority;
    return left.order - right.order;
  }

  static function laneKey(laneId:String):String return 'lane:$laneId';
  static function intersectionKey(stationId:String):String return 'intersection:$stationId';

  static function routeSignature(route:FacilityRoute, legs:Array<FacilityRouteLeg>):String {
    var segments = [for (leg in legs)
      '${leg.lane.id}:${leg.reversed ? "reverse" : "forward"}'];
    return '${route.facilityId}|${route.fromStationId}|${route.toStationId}|${segments.join(",")}';
  }

  function requireFleetMember(robotId:String):Void {
    if (!fleet.containsRobot(robotId)) throw 'Robot "$robotId" is not in the fleet';
  }
}

private class TrafficWaiter {
  public final robotId:String;
  public final resources:Array<String>;
  public var priority:Int;
  public final order:Int;

  public function new(robotId:String, resources:Array<String>, priority:Int, order:Int) {
    this.robotId = robotId;
    this.resources = resources;
    this.priority = priority;
    this.order = order;
  }
}
