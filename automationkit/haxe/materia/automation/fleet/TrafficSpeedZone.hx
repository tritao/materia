package materia.automation.fleet;

/** Temporary speed cap applied to the configured facility lanes. */
class TrafficSpeedZone {
  public final id:String;
  public final maximumSpeedMetersPerSecond:Float;
  final laneValues:Array<String>;

  public function new(id:String, laneIds:Array<String>, maximumSpeedMetersPerSecond:Float) {
    if (id == null || id.length == 0 || laneIds == null || laneIds.length == 0 ||
        !Math.isFinite(maximumSpeedMetersPerSecond) || maximumSpeedMetersPerSecond <= 0.0)
      throw "Traffic speed zone requires an ID, lanes, and a positive finite speed";
    laneValues = [];
    for (laneId in laneIds) {
      if (laneId == null || laneId.length == 0 || laneValues.indexOf(laneId) >= 0)
        throw "Traffic speed zone lane IDs must be non-empty and unique";
      laneValues.push(laneId);
    }
    this.id = id;
    this.maximumSpeedMetersPerSecond = maximumSpeedMetersPerSecond;
  }

  public function lanes():Array<String> return laneValues.copy();
}
