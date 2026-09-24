package materia.automation.fleet;

import materia.automation.facility.Facility;

/** Exclusive lane reservations for facilities whose corridors cannot be shared. */
class TrafficManager {
  public final facility:Facility;
  public final fleet:Fleet;
  final ownerByLane = new Map<String,String>();

  public function new(facility:Facility, fleet:Fleet) {
    if (facility == null || fleet == null) throw "TrafficManager requires a facility and fleet";
    this.facility = facility;
    this.fleet = fleet;
  }

  /** Returns false if another fleet member currently owns the lane. */
  public function reserve(laneId:String, robotId:String):Bool {
    if (facility.lane(laneId) == null) throw 'Unknown facility lane "$laneId"';
    if (!fleet.containsRobot(robotId)) throw 'Robot "$robotId" is not in the fleet';
    var owner = ownerByLane.get(laneId);
    if (owner != null) return owner == robotId;
    ownerByLane.set(laneId, robotId);
    return true;
  }

  public function owner(laneId:String):Null<String> return ownerByLane.get(laneId);

  public function release(laneId:String, robotId:String):Bool {
    var current = ownerByLane.get(laneId);
    if (current == null || current != robotId) return false;
    ownerByLane.remove(laneId);
    return true;
  }

  public function reservedLanes(robotId:String):Array<String> {
    var result:Array<String> = [];
    for (laneId in ownerByLane.keys()) if (ownerByLane.get(laneId) == robotId) result.push(laneId);
    result.sort(Reflect.compare);
    return result;
  }
}
