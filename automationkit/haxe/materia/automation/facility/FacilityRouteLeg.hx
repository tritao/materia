package materia.automation.facility;

/** One directed traversal of a facility lane. */
class FacilityRouteLeg {
  public final lane:Lane;
  public final fromStationId:String;
  public final toStationId:String;
  public final reversed:Bool;

  public function new(lane:Lane, fromStationId:String, toStationId:String,
      reversed:Bool) {
    if (lane == null || fromStationId == null || toStationId == null ||
        fromStationId == toStationId)
      throw "Facility route leg requires a lane and distinct endpoints";
    if (reversed && !lane.bidirectional)
      throw "One-way lane cannot be traversed in reverse";
    if (reversed) {
      if (fromStationId != lane.toStationId || toStationId != lane.fromStationId)
        throw "Reversed facility route leg does not match its lane endpoints";
    } else if (fromStationId != lane.fromStationId || toStationId != lane.toStationId)
      throw "Facility route leg does not match its lane direction";
    this.lane = lane;
    this.fromStationId = fromStationId;
    this.toStationId = toStationId;
    this.reversed = reversed;
  }
}
