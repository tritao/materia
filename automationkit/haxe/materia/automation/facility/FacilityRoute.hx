package materia.automation.facility;

import robotkit.navigation.Path;

/** Shortest-time lane sequence and composed path through one facility frame. */
class FacilityRoute {
  public final facilityId:String;
  public final fromStationId:String;
  public final toStationId:String;
  public final path:Path;
  public final maximumSpeedMetersPerSecond:Float;
  final legValues:Array<FacilityRouteLeg>;

  public function new(facilityId:String, fromStationId:String, toStationId:String,
      legs:Array<FacilityRouteLeg>, path:Path, maximumSpeedMetersPerSecond:Float) {
    if (facilityId == null || facilityId.length == 0 || fromStationId == null ||
        fromStationId.length == 0 || toStationId == null || toStationId.length == 0 ||
        fromStationId == toStationId || legs == null || legs.length == 0 || path == null ||
        !Math.isFinite(maximumSpeedMetersPerSecond) || maximumSpeedMetersPerSecond <= 0.0)
      throw "Facility route requires distinct stations, lane legs, a path, and a speed limit";
    this.facilityId = facilityId;
    this.fromStationId = fromStationId;
    this.toStationId = toStationId;
    legValues = legs.copy();
    for (leg in legValues) if (leg == null) throw "Facility route legs cannot be null";
    var previousStation = fromStationId;
    for (leg in legValues) {
      if (leg.fromStationId != previousStation)
        throw "Facility route legs must form a connected station sequence";
      previousStation = leg.toStationId;
    }
    if (previousStation != toStationId || legValues[0].lane.centerline.frameId != path.frameId)
      throw "Facility route path frame and lane sequence must match its endpoints";
    this.path = new Path(path.poses(), path.frameId);
    this.maximumSpeedMetersPerSecond = maximumSpeedMetersPerSecond;
  }

  public function legs():Array<FacilityRouteLeg> return legValues.copy();
}
